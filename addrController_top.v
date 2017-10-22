module addrController_top
(
	// clocks:
	input clk,
	input cep,						// 8.125 MHz CPU clock
	input cen,						// 8.125 MHz CPU clock

	// system config:
	input turbo,               // 0 = normal, 1 = faster
	input configROMSize,			// 0 = 64K ROM, 1 = 128K ROM
	input [1:0] configRAMSize,	// 0 = 128K, 1 = 512K, 2 = 1MB, 3 = 4MB RAM

	// 68000 CPU memory interface:
	input [23:0] cpuAddr,
	input _cpuUDS,
	input _cpuLDS,
	input _cpuRW,	

	// RAM/ROM:
	output [21:0] memoryAddr,
	output _memoryUDS,
	output _memoryLDS,	
	output _romOE,
	output _ramOE,	
	output _ramWE,	
	output dioBusControl,
	output cpuBusControl,

	// peripherals:
	output selectSCSI,
	output selectSCC,
	output selectIWM,
	output selectVIA,

	// video:
	input  _vblank,
	input  _hblank,

	input  snd_alt,
	output loadSound,

	// misc
	input memoryOverlayOn,

	// interface to read dsk image from ram
	input [21:0] dskReadAddrInt,
	output dskReadAckInt,
	input [21:0] dskReadAddrExt,
	output dskReadAckExt
);

// -------------- audio engine (may be moved into seperate module) ---------------
assign loadSound = audioReq & sndReadAck;

reg [21:0] audioAddr;
reg        audioReq;
always @(posedge clk) begin
	reg vblankD;
	reg hblankD;
	reg swap;
	reg sndReadAckD;

	sndReadAckD <= sndReadAck;
	if(sndReadAckD & ~sndReadAck) begin // prepare for next audio cycle
		vblankD  <= _vblank;
		hblankD  <= _hblank;
		audioReq <= 0;

		// falling adge of _vblank = begin of vblank phase
		if(vblankD && !_vblank) swap <= 1;

		if(hblankD && !_hblank) begin
			if(swap) audioAddr <= snd_alt ? 22'h3FA100 : 22'h3FFD00;
			else audioAddr <= audioAddr + 22'd2;
			swap <= 0;
			audioReq <= 1;
		end
	end
end

// interleaved RAM access for CPU and periphery
reg  [3:0] cycle;
wire [1:0] busCycle = cycle[1:0];
wire [1:0] subCycle = cycle[3:2];
always @(posedge clk) if(cep) cycle <= cycle + 2'd1;

assign cpuBusControl = turbo ? busCycle[0] : (busCycle == 1);
assign dioBusControl = (busCycle == 2);

assign dskReadAckInt = dioBusControl && (subCycle == 0);
assign dskReadAckExt = dioBusControl && (subCycle == 1);
wire   sndReadAck    = (busCycle == 0);


// interconnects
wire selectRAM, selectROM;

// RAM/ROM control signals
wire extraRomRead = dskReadAckInt || dskReadAckExt;
assign _romOE = ~(extraRomRead || (cpuBusControl && selectROM && _cpuRW)); 

assign _ramOE = ~(loadSound || (cpuBusControl && selectRAM && _cpuRW));
assign _ramWE = ~(cpuBusControl && selectRAM && !_cpuRW);

assign _memoryUDS = cpuBusControl ? _cpuUDS : 1'b0;
assign _memoryLDS = cpuBusControl ? _cpuLDS : 1'b0;
wire [21:0] addrMux  = loadSound ? audioAddr : cpuAddr[21:0];
wire [21:0] macAddr;
assign macAddr[15:0] = addrMux[15:0];

// video and sound always addresses ram
wire ram_access = (cpuBusControl && selectRAM) || loadSound;
wire rom_access = (cpuBusControl && selectROM);

// simulate smaller RAM/ROM sizes
assign macAddr[16] = rom_access && configROMSize == 1'b0 ? 1'b0 :    // force A16 to 0 for 64K ROM access
								addrMux[16]; 
assign macAddr[17] = ram_access && configRAMSize == 2'b00 ? 1'b0 :   // force A17 to 0 for 128K RAM access
								rom_access && configROMSize == 1'b1 ? 1'b0 : // force A17 to 0 for 128K ROM access
								rom_access && configROMSize == 1'b0 ? 1'b1 : // force A17 to 1 for 64K ROM access (64K ROM image is at $20000)
								addrMux[17]; 
assign macAddr[18] = ram_access && configRAMSize == 2'b00 ? 1'b0 :   // force A18 to 0 for 128K RAM access
								rom_access ? 1'b0 : 								   // force A18 to 0 for ROM access
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

assign memoryAddr = 
	dskReadAckInt ? dskReadAddrInt + 22'h100000:   // first dsk image at 1MB
	dskReadAckExt ? dskReadAddrExt + 22'h200000:   // second dsk image at 2MB
	macAddr;

// address decoding
addrDecoder ad(
	.address(cpuAddr),
	.memoryOverlayOn(memoryOverlayOn),
	.selectRAM(selectRAM),
	.selectROM(selectROM),
	.selectSCSI(selectSCSI),
	.selectSCC(selectSCC),
	.selectIWM(selectIWM),
	.selectVIA(selectVIA));

endmodule
