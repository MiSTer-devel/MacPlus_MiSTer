`include "sdram_map.vh"   // floppy-image byte offsets

module addrController_top(
	// clocks:
	input clk,
	output clk8,						// 7.8336 MHz CPU clock
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
	output de,				// VGA_DE: exactly the 512 x 342 shifter pixels
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

	// floppy_loader write port into RAM, on the previously unused extra slot 3;
	// int has fixed priority. Address/control only, MacPlus.sv muxes the data.
	input [21:0] dskLoadAddrInt,
	input dskLoadReqInt,
	output dskLoadAckInt,
	input [21:0] dskLoadAddrExt,
	input dskLoadReqExt,
	output dskLoadAckExt,
	output dskLoadWrEn,
	// held for the whole grant cycle; MacPlus.sv muxes the write data on this
	output dskLoadSelExt,

	// floppy_sd_writer read port on the same slot, below both write requests;
	// same protocol, and sdram.v has the word in dout from the end of
	// busPhase 2, so the requester captures it on the ack edge
	input [21:0] dskFetchAddrInt,
	input dskFetchReqInt,
	output dskFetchAckInt,
	input [21:0] dskFetchAddrExt,
	input dskFetchReqExt,
	output dskFetchAckExt,
	output dskLoadRdEn
);

	// -------------- audio engine ---------------
	// As on the real Mac, one sound word per scan line, 370 a frame: the scan
	// advances where _hblank rises and restarts at the VBL (line 0).

	assign loadSound = sndReadAck;

	// sndAdvance pulses (one clk8 wide) when the scan advances audioAddr;
	// dataController_top commits the pre-buffered sample to the output on it.
	reg sndAdvance;
	assign snd_advance = sndAdvance;

	reg [21:0] audioAddr;

	reg vblankD, hblankD;
	always @(posedge clk) begin
		if (clk8_en_p) begin
			vblankD <= _vblank;
			hblankD <= _hblank;
			sndAdvance <= 1'b0;

			// falling edge of _vblank = begin of vblank phase = start of line 0
			if (vblankD && !_vblank) begin
				audioAddr <= snd_alt ? 22'h3FA100 : 22'h3FFD00;
				sndAdvance <= 1'b1; // commit first sample at vblank
			end else if (!hblankD && _hblank) begin	// start of lines 1-369
				audioAddr <= audioAddr + 22'd2;
				sndAdvance <= 1'b1;
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
	wire videoControlActive = loadPixels;	// every other video slot carries a word

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
	// the 64K ROM image sits at offset 0 of its slot, so A17 is forced to 0
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
	// sdram.v issues ACTIVE from busPhase 0 and WRITE (column and data) from
	// busPhase 1, so every control signal must hold for the whole 4-phase cycle.
	//
	// so the loader requests are sampled once at the cycle boundary, the grant
	// held for the whole cycle, and the ack pulsed late in busPhase 3
	reg dskLoadReqIntR, dskLoadReqExtR;
	reg dskFetchReqIntR, dskFetchReqExtR;
	always @(posedge clk) if (busPhase == 2'b11) begin
		dskLoadReqIntR  <= dskLoadReqInt;
		dskLoadReqExtR  <= dskLoadReqExt;
		dskFetchReqIntR <= dskFetchReqInt;
		dskFetchReqExtR <= dskFetchReqExt;
	end

	assign dskLoadSelExt = ~dskLoadReqIntR & dskLoadReqExtR;
	wire dskLoadGrant  = (extraBusControl == 1'b1) && (extra_slot_count == 3) && (dskLoadReqIntR | dskLoadReqExtR);
	wire dskLoadAck    = dskLoadGrant && (busPhase == 2'b11);
	assign dskLoadAckInt = dskLoadAck & ~dskLoadSelExt;
	assign dskLoadAckExt = dskLoadAck &  dskLoadSelExt;
	assign dskLoadWrEn   = dskLoadGrant;

	// reads take the slot only when neither write request has it; every term
	// is a busPhase-3 register, so the choice holds for the whole cycle
	wire dskFetchSelExt  = ~dskFetchReqIntR & dskFetchReqExtR;
	wire dskFetchGrant   = (extraBusControl == 1'b1) && (extra_slot_count == 3) &&
	                       ~(dskLoadReqIntR | dskLoadReqExtR) && (dskFetchReqIntR | dskFetchReqExtR);
	wire dskFetchAck     = dskFetchGrant && (busPhase == 2'b11);
	assign dskFetchAckInt = dskFetchAck & ~dskFetchSelExt;
	assign dskFetchAckExt = dskFetchAck &  dskFetchSelExt;
	assign dskLoadRdEn    = dskFetchGrant;

	// byte offsets of each floppy image within the disk region (rtl/sdram_map.vh)
	assign memoryAddr =
		dskReadAckInt ? dskReadAddrInt + `DSK_INT_BYTE_OFF:   // first dsk image at 1MB
		dskReadAckExt ? dskReadAddrExt + `DSK_EXT_BYTE_OFF:   // second dsk image at 2MB
		dskLoadGrant  ? (dskLoadSelExt ? dskLoadAddrExt + `DSK_EXT_BYTE_OFF : dskLoadAddrInt + `DSK_INT_BYTE_OFF) :
		dskFetchGrant ? (dskFetchSelExt ? dskFetchAddrExt + `DSK_EXT_BYTE_OFF : dskFetchAddrInt + `DSK_INT_BYTE_OFF) :
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
		.pix_en(clk16_en_p),
		.busCycle(busCycle), 
		.vid_alt(vid_alt),
		.videoAddr(videoAddr), 
		.hsync(hsync), 
		.vsync(vsync), 
		._hblank(_hblank),
		._vblank(_vblank), 
		.de(de),
		.loadPixels(loadPixels));
		
endmodule
