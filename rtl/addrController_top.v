`include "sdram_map.vh"   // floppy-image byte offsets

module addrController_top(
	// clocks:
	input clk,
	output clk8,						// 8.125 MHz CPU clock
	output clk8_en_p,
	output clk8_en_n,
	output clk16_en_p,
	output clk16_en_n,

	// system config:
	input turbo,               // 0 = normal, 1 = faster
	input [1:0] configROMSize,  // 0 = 64K ROM, 1 = 128K ROM, 2 = 256K ROM
	input scsiPresent,          // 1 = machine has a SCSI bus; see rtl/mac_model.v
	input [1:0] configRAMSize,	// 0 = 128K, 1 = 512K, 2 = 1MB, 3 = 4MB RAM

	// 68000 CPU memory interface:
	input [23:0] cpuAddr,
	input _cpuUDS,
	input _cpuLDS,
	input _cpuRW,	
	input _cpuAS,
	
	// RAM/ROM:
	output [21:0] memoryAddr,
	output _memoryUDS,
	output _memoryLDS,	
	output _romOE,
	output _ramOE,	
	output _ramWE,	
	output videoBusControl,
	output dioBusControl,
	output cpuBusControl,
	output memoryLatch,
	
	// peripherals:
	output selectSCSI,
	output selectSCC,
	output selectIWM,
	output selectVIA,
	output selectRAM,
	output selectROM,
	output selectSEOverlay,
	
	// video:
	output hsync,
	output vsync,
	output _hblank,
	output _vblank,
	output loadPixels,
	input  vid_alt,
		
	input  snd_alt,
	output loadSound,
	output snd_advance,
		
	// misc
	input memoryOverlayOn,
	
	// interface to read dsk image from ram
	input [21:0] dskReadAddrInt,
	output dskReadAckInt,
	input [21:0] dskReadAddrExt,
	output dskReadAckExt,

	// interface for floppy_loader to write a mounted image into ram.
	// Shares the previously-unused extra slot 3; int is fixed priority over
	// ext on simultaneous requests (a concurrent dual mount only serializes
	// the first few words, since each loader's own address only advances on
	// its own ack). Write DATA is not routed through here - this module is
	// address/control only, matching dskReadAddr*/dskReadAck* above - MacPlus.sv
	// muxes the actual word using these same ack pulses.
	input [21:0] dskLoadAddrInt,
	input dskLoadReqInt,
	output dskLoadAckInt,
	input [21:0] dskLoadAddrExt,
	input dskLoadReqExt,
	output dskLoadAckExt,
	output dskLoadWrEn,
	// held for the whole grant cycle (unlike dskLoadAckInt/Ext, which pulse
	// only in busPhase 3) - MacPlus.sv needs this, not the ack pulses, to
	// mux which loader's write DATA reaches sdram.v, since that data must
	// be stable through busPhase 1 (CAS), not just busPhase 3.
	output dskLoadSelExt
);

	// -------------- audio engine ---------------
	// On real Mac hardware, the ASG PAL reads one sound sample per horizontal
	// line with perfectly periodic timing (one Mac line ≈ 352 clk8 at 7.8336 MHz).
	// The Mac has exactly 370 lines per frame, so 370 samples at 22,254 Hz.
	//
	// The MiSTer uses 806 VGA lines (168 clk8 each) = 135,408 clk8 per frame.
	// A Bresenham divider distributes 370 address advances across the frame.
	//
	// Previous implementation ran the Bresenham on sndReadAck events (every 16
	// clk8), giving ±16 clk8 jitter (~4.4%). This version runs at clk8 granularity,
	// reducing jitter to ±1 clk8 (~0.27%) — closely matching the real hardware's
	// line-synchronous, zero-jitter timing.

	assign loadSound = sndReadAck;

	// sndAdvance pulses (one clk8 wide) when the Bresenham divider advances
	// audioAddr. dataController_top uses this to commit the pre-buffered SDRAM
	// sample to the audio output with clk8-level timing precision, eliminating
	// the 0-15 clk8 output latency jitter from sndReadAck quantisation.
	reg sndAdvance;
	assign snd_advance = sndAdvance;

	localparam [17:0] SND_SIZE = 18'd135408;  // clk8 events per frame (168×806)
	localparam [17:0] SND_STEP = 18'd370;     // Bresenham step: 370 overflows per frame,
	                                           // but the last coincides with vblank reset
	                                           // (if takes priority over else-if), so
	                                           // 369 actual advances + initial addr = 370 samples

	reg [21:0] audioAddr;
	reg [17:0] snd_div;

	wire [17:0] snd_div_next = snd_div + SND_STEP;

	reg vblankD;
	always @(posedge clk) begin
		if (clk8_en_p) begin
			vblankD <= _vblank;
			sndAdvance <= 1'b0;

			// falling edge of _vblank = begin of vblank phase
			if (vblankD && !_vblank) begin
				audioAddr <= snd_alt ? 22'h3FA100 : 22'h3FFD00;
				snd_div <= 18'd0;
				sndAdvance <= 1'b1; // commit first sample at vblank
			end else if (snd_div_next >= SND_SIZE) begin
				snd_div <= snd_div_next - SND_SIZE;
				audioAddr <= audioAddr + 22'd2;
				sndAdvance <= 1'b1;
			end else begin
				snd_div <= snd_div_next;
			end
		end
	end

	assign dioBusControl = extraBusControl;

	// interleaved RAM access for CPU and video
	reg [1:0] busCycle;
	reg [1:0] busPhase;
	reg [1:0] extra_slot_count;

	always @(posedge clk) begin
		busPhase <= busPhase + 1'd1;
		if (busPhase == 2'b11)
			busCycle <= busCycle + 2'd1;
	end
	assign memoryLatch = busPhase == 2'd3;
	assign clk8 = !busPhase[1];
	assign clk8_en_p = busPhase == 2'b11;
	assign clk8_en_n = busPhase == 2'b01;
	assign clk16_en_p = !busPhase[0];
	assign clk16_en_n = busPhase[0];

	reg extra_slot_advance;
	always @(posedge clk)
		if (clk8_en_n) extra_slot_advance <= (busCycle == 2'b11);

	// allocate memory slots in the extra cycle
	always @(posedge clk) begin
		if(clk8_en_p && extra_slot_advance) begin
			extra_slot_count <= extra_slot_count + 2'd1;
		end
	end

	// video controls memory bus during the first clock of the four-clock cycle
	assign videoBusControl = (busCycle == 2'b00);
	// cpu controls memory bus during the second and fourth clock of the four-clock cycle
	assign cpuBusControl = (busCycle == 2'b01) || (busCycle == 2'b11);
	// IWM/audio gets 3rd cycle
	wire extraBusControl = (busCycle == 2'b10);

	// interconnects
	wire [21:0] videoAddr;
	
	// RAM/ROM control signals
	wire videoControlActive = _hblank;

	assign _romOE = ~(cpuBusControl && selectROM && _cpuRW);
	
	wire extraRamRead = sndReadAck;
	assign _ramOE = ~((videoBusControl && videoControlActive) || (extraRamRead) ||
						(cpuBusControl && selectRAM && _cpuRW));
	assign _ramWE = ~(cpuBusControl && selectRAM && !_cpuRW);
	
	assign _memoryUDS = cpuBusControl ? _cpuUDS : 1'b0;
	assign _memoryLDS = cpuBusControl ? _cpuLDS : 1'b0;
	wire [21:0] addrMux = sndReadAck ? audioAddr : videoBusControl ? videoAddr : cpuAddr[21:0];
	wire [21:0] macAddr;
	assign macAddr[15:0] = addrMux[15:0];

	// video and sound always addresses ram
	wire ram_access = (cpuBusControl && selectRAM) || videoBusControl || sndReadAck;
	wire rom_access = (cpuBusControl && selectROM);
	
	// simulate smaller RAM/ROM sizes
	assign macAddr[16] = rom_access && configROMSize == 2'b00 ? 1'b0 :     // force A16 to 0 for 64K ROM access
									addrMux[16]; 
	// Every boot ROM is written at offset 0 of its own 512KB slot, the 64K
	// image included, so A17 is forced to 0 here rather than to 1. With A16
	// also forced to 0 above, the image occupies bytes $00000-$0FFFF and
	// aliases every 64KB across the Mac's ROM window, which is what a real
	// 64K ROM does.
	assign macAddr[17] = ram_access && configRAMSize == 2'b00 ? 1'b0 :   // force A17 to 0 for 128K RAM access
									rom_access && configROMSize == 2'b01 ? 1'b0 :  // force A17 to 0 for 128K ROM access
									rom_access && configROMSize == 2'b00 ? 1'b0 :  // force A17 to 0 for 64K ROM access (image sits at its slot's offset 0)
									addrMux[17]; 
	assign macAddr[18] = ram_access && configRAMSize == 2'b00 ? 1'b0 :   // force A18 to 0 for 128K RAM access
	                     rom_access && configROMSize != 2'b11 ? 1'b0 : // force A18 to 0 for 64K/128K/256K ROM access
									addrMux[18]; 
	assign macAddr[19] = ram_access && configRAMSize[1] == 1'b0 ? 1'b0 : // force A19 to 0 for 128K or 512K RAM access
									rom_access ? 1'b0 : 								   // force A19 to 0 for ROM access
									addrMux[19]; 
	assign macAddr[20] = ram_access && configRAMSize != 2'b11 ? 1'b0 :   // force A20 to 0 for all but 4MB RAM access
									rom_access ? 1'b0 : 								   // force A20 to 0 for ROM access
									addrMux[20]; 
	assign macAddr[21] = ram_access && configRAMSize != 2'b11 ? 1'b0 :   // force A21 to 0 for all but 4MB RAM access
									rom_access ? 1'b0 : 								   // force A21 to 0 for ROM access
									addrMux[21]; 
	
			
	// floppy emulation gets extra slots 0 and 1
	assign dskReadAckInt = (extraBusControl == 1'b1) && (extra_slot_count == 0);
	assign dskReadAckExt = (extraBusControl == 1'b1) && (extra_slot_count == 1);
	// audio gets extra slot 2
	wire sndReadAck    = (extraBusControl == 1'b1) && (extra_slot_count == 2);
	// floppy image loader (mount-time SD->SDRAM copy) gets the previously
	// unused extra slot 3. Fixed priority: int over ext.
	//
	// sdram.v does not latch a memory cycle in one shot: it issues ACTIVE
	// (row/bank, plus the oe/we decision) from the signal values present
	// during busPhase 0, then WRITE (column address AND write data) from the
	// values present one clk_sys cycle later, in busPhase 1. Every memory
	// control signal must therefore be stable for the WHOLE four-phase bus
	// cycle - which is why MacPlus.sv's ROM download path only ever changes
	// dio_write while ~dioBusControl, and why dskReadAck* below are asserted
	// for a whole cycle rather than pulsed.
	//
	// So: sample the loader requests once at the cycle boundary and hold the
	// grant (and hence dskLoadWrEn and the memoryAddr mux) for the entire
	// cycle, and make the ack a late pulse in busPhase 3. A combinational
	// grant with a cycle-wide ack would let the loader drop wr_req at the
	// start of busPhase 1 - after RAS had committed the write, but before CAS
	// sampled the column address and data, so every word landed at a wrong
	// column with the CPU's data bus contents instead of the disk byte.
	reg dskLoadReqIntR, dskLoadReqExtR;
	always @(posedge clk) if (busPhase == 2'b11) begin
		dskLoadReqIntR <= dskLoadReqInt;
		dskLoadReqExtR <= dskLoadReqExt;
	end

	assign dskLoadSelExt = ~dskLoadReqIntR & dskLoadReqExtR;
	wire dskLoadGrant  = (extraBusControl == 1'b1) && (extra_slot_count == 3) && (dskLoadReqIntR | dskLoadReqExtR);
	wire dskLoadAck    = dskLoadGrant && (busPhase == 2'b11);
	assign dskLoadAckInt = dskLoadAck & ~dskLoadSelExt;
	assign dskLoadAckExt = dskLoadAck &  dskLoadSelExt;
	assign dskLoadWrEn   = dskLoadGrant;

	// Byte offsets of each floppy image within the disk region. Named in
	// rtl/sdram_map.vh so that they and the boot-ROM slots can be checked
	// against each other in one place.
	assign memoryAddr =
		dskReadAckInt ? dskReadAddrInt + `DSK_INT_BYTE_OFF:   // first dsk image at 1MB
		dskReadAckExt ? dskReadAddrExt + `DSK_EXT_BYTE_OFF:   // second dsk image at 2MB
		dskLoadGrant  ? (dskLoadSelExt ? dskLoadAddrExt + `DSK_EXT_BYTE_OFF : dskLoadAddrInt + `DSK_INT_BYTE_OFF) :
		macAddr;

	// address decoding
	addrDecoder ad(
		.configROMSize(configROMSize),
		.scsiPresent(scsiPresent),
		.address(cpuAddr),
		._cpuAS(_cpuAS),
		.memoryOverlayOn(memoryOverlayOn),
		.selectRAM(selectRAM),
		.selectROM(selectROM),
		.selectSCSI(selectSCSI),
		.selectSCC(selectSCC),
		.selectIWM(selectIWM),
		.selectVIA(selectVIA),
		.selectSEOverlay(selectSEOverlay));

	// video
	videoTimer vt(
		.clk(clk),
		.clk_en(clk8_en_p),
		.busCycle(busCycle), 
		.vid_alt(vid_alt),
		.videoAddr(videoAddr), 
		.hsync(hsync), 
		.vsync(vsync), 
		._hblank(_hblank),
		._vblank(_vblank), 
		.loadPixels(loadPixels));
		
endmodule
