//============================================================================
//  Macintosh Plus
//
//  Port to MiSTer
//  Copyright (C) 2017-2019 Sorgelig
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [45:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output  [7:0] VIDEO_ARX,
	output  [7:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output  [1:0] VGA_SL,

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status OR'd with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	// I/O board button press simulation (active high)
	// b[1]: user button
	// b[0]: osd button
	output  [1:0] BUTTONS,

	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,   // 1 - signed audio samples, 0 - unsigned
	output  [1:0] AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

	//ADC
	inout   [3:0] ADC_BUS,

	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	// Open-drain User port.
	// 0 - D+/RX
	// 1 - D-/TX
	// 2..6 - USR2..USR6
	// Set USER_OUT to 1 to read from USER_IN.
	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0; 
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign LED_USER  = dio_download || (disk_act ^ |diskMotor);
assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;

assign VIDEO_ARX = status[8] ? 8'd16 : 8'd4;
assign VIDEO_ARY = status[8] ? 8'd9  : 8'd3; 

`include "build_id.v" 
localparam CONF_STR = {
	"MACPLUS;;",
	"-;",
	"F0,DSK,Mount Pri Floppy;",
	"F1,DSK,Mount Sec Floppy;",
	"-;",
	"S0,VHD,Mount HDD - SCSI2;",
	"S1,VHD,Mount HDD - SCSI6 (boot);",
	"-;",
	"O8,Aspect ratio,4:3,16:9;",
	"O9A,Memory,512KB,1MB,4MB;",
	"O5,Speed,Normal,Turbo;",
	"-;",
	"R6,Reset;",
	"V,v",`BUILD_DATE
};

////////////////////   CLOCKS   ///////////////////

wire clk_sys;
wire pll_locked;
		
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(SDRAM_CLK),
	.locked(pll_locked)
);

wire cep   = (stage == 0);
wire cen   = (stage == 4);
wire cel   = (stage == 7);
wire cepix = !stage[1:0];

reg [2:0] stage;
always @(negedge clk_sys) stage <= stage + 1'd1;

///////////////////////////////////////////////////

// interconnects
// CPU
wire        _cpuReset, _cpuResetOut, _cpuUDS, _cpuLDS, _cpuRW;
wire  [2:0] _cpuIPL;
wire  [7:0] cpuAddrHi;
wire [23:0] cpuAddr;
wire [15:0] cpuDataOut;

// RAM/ROM
wire        _romOE;
wire        _ramOE, _ramWE;
wire        _memoryUDS, _memoryLDS;
wire        videoBusControl;
wire        dioBusControl;
wire        cpuBusControl;
wire [21:0] memoryAddr;
wire [15:0] memoryDataOut;

// peripherals
wire        memoryOverlayOn, selectSCSI, selectSCC, selectIWM, selectVIA;	 
wire [15:0] dataControllerDataOut;

// audio
wire snd_alt;
wire loadSound;

// floppy disk image interface
wire        dskReadAckInt;
wire [21:0] dskReadAddrInt;
wire        dskReadAckExt;
wire [21:0] dskReadAddrExt;
wire  [1:0] diskMotor, diskAct, diskEject;

// the status register is controlled by the on screen display (OSD)
wire [31:0] status;
wire  [1:0] buttons;
wire  [2:0] img_mounted;
wire [15:0] sd_req_type;
wire [31:0] sd_lba;
wire  [1:0] sd_rd;
wire  [1:0] sd_wr;
wire        sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din;
wire        sd_buff_wr;

reg         ioctl_wr;
wire        ioctl_write;
reg         ioctl_wait = 0;

wire [10:0] ps2_key;
wire [24:0] ps2_mouse;
wire        capslock;

wire [24:0] ioctl_addr;
wire  [7:0] ioctl_data;

always @(posedge clk_sys) begin
	reg [7:0] temp;
	
	ioctl_wr <= 0;
	if(ioctl_write) begin
		if(~ioctl_addr[0]) temp <= ioctl_data;
		else begin
			dio_data <= {temp, ioctl_data};
			ioctl_wr <= 1;
		end
	end
end

hps_io #(.STRLEN($size(CONF_STR)>>3)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.conf_str(CONF_STR),

	.buttons(buttons),
	.status(status),

	.img_mounted(img_mounted),
	
	.sd_req_type(sd_req_type),
	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),

	.sd_conf(0),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),
	
	.ioctl_download(dio_download),
	.ioctl_index(dio_index),
	.ioctl_wr(ioctl_write),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_data),

	.ioctl_wait(ioctl_wait),

	.ps2_key(ps2_key),
	.ps2_kbd_led_use(3'b001),
	.ps2_kbd_led_status({2'b00, capslock}),

	.ps2_mouse(ps2_mouse)
);

wire  [1:0] cpu_busstate;
wire        cpu_clkena = cep && (cpuBusControl || (cpu_busstate == 2'b01));
reg  [15:0] cpuDataIn;
always @(posedge clk_sys) if(cel && cpuBusControl && ~cpu_busstate[0] && _cpuRW) cpuDataIn <= dataControllerDataOut;

TG68KdotC_Kernel #(0,0,0,0,0,0) m68k
(
	.clk            ( clk_sys        ),
	.nReset         ( _cpuReset      ),
	.clkena_in      ( cpu_clkena     ), 
	.data_in        ( cpuDataIn      ),
	.IPL            ( _cpuIPL        ),
	.IPL_autovector ( 1'b1           ),
	.berr           ( 1'b0           ),
	.clr_berr       ( 1'b0           ),
	.CPU            ( 2'b00          ),   // 00=68000
	.addr           ( {cpuAddrHi, cpuAddr} ),
	.data_write     ( cpuDataOut     ),
	.nUDS           ( _cpuUDS        ),
	.nLDS           ( _cpuLDS        ),
	.nWr            ( _cpuRW         ),
	.busstate       ( cpu_busstate   ), // 00-> fetch code 10->read data 11->write data 01->no memaccess
	.nResetOut      ( _cpuResetOut   ),
	.FC             (                )
);

/*
fx68k M68K
(
	.clk		( clk_sys ),
	
	.extReset( !_cpuReset ),
	.pwrUp	( !_cpuReset ),
	
	.enPhi1	(M68K_CLKENp),
	.enPhi2	(M68K_CLKENn),

	.eRWn		( _cpuRW ),
	.ASn		(M68K_AS_N),
	.UDSn		( _cpuUDS ),
	.LDSn		( _cpuLDS ),

	.FC0(M68K_FC[0]),
	.FC1(M68K_FC[1]),
	.FC2(M68K_FC[2]),

	.BGn(M68K_BG_N),
	.BRn(M68K_BR_N),
	.BGACKn(M68K_BGACK_N),

	.DTACKn(M68K_MBUS_DTACK_N),
	.VPAn(~M68K_INTACK),
	.BERRn(1),
	.IPL0n( _cpuIPL[0] ),
	.IPL1n( _cpuIPL[1] ),
	.IPL2n( _cpuIPL[2] ),
	.iEdb( cpuDataIn ),
	.oEdb( cpuDataOut ),
	.eab( {cpuAddrHi, cpuAddr} )
);
*/

assign VGA_R = {8{pixelOut}};
assign VGA_G = {8{pixelOut}};
assign VGA_B = {8{pixelOut}};
assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = cepix;
assign VGA_F1 = 0;
assign VGA_SL = 0;

wire screenWrite;
always @(*) begin
	case(configRAMSize)
		0:	screenWrite = ~_ramWE && &memoryAddr[16:15]; // 01A700 (018000)
		1:	screenWrite = ~_ramWE && &memoryAddr[18:15]; // 07A700 (078000)
		2:	screenWrite = ~_ramWE && &memoryAddr[19:15]; // 0FA700 (0F8000)
		3:	screenWrite = ~_ramWE && &memoryAddr[21:15]; // 3FA700 (3F8000)
	endcase
end

wire pixelOut, _hblank, _vblank;
video video
(
	.clk(clk_sys),
	.ce(cepix),

	.addr(cpuAddr[15:1]),
	.dataIn(cpuDataOut),
	.wr({~_cpuUDS & screenWrite, ~_cpuLDS & screenWrite}),

	._hblank(_hblank),
	._vblank(_vblank),

	.hsync(VGA_HS),
	.vsync(VGA_VS),
	.video_en(VGA_DE),
	.pixelOut(pixelOut)
);

wire [10:0] audio;
assign AUDIO_L = {audio[10:0], 5'b00000};
assign AUDIO_R = {audio[10:0], 5'b00000};
assign AUDIO_S = 0;
assign AUDIO_MIX = 0;

wire       status_turbo = status[5];
wire       status_reset = status[6];

wire [1:0] status_mem   = status[10:9]; // 128KB, 512KB, 1MB, 4MB
reg  [1:0] configRAMSize= 3;

reg n_reset = 0;
always @(posedge clk_sys) begin
	reg [15:0] rst_cnt;

	// various sources can reset the mac
	if(!pll_locked || status[0] || status_reset || buttons[1] || RESET || ~_cpuResetOut) begin
		rst_cnt <= '1;
		n_reset <= 0;
	end else if(rst_cnt) begin
		if(cen) rst_cnt <= rst_cnt - 1'd1;
		configRAMSize <= status_mem + 1'd1;
	end else n_reset <= 1;
end

addrController_top ac0
(
	.clk(clk_sys),
	.cep(cep),
	.cen(cen),

	.cpuAddr(cpuAddr),
	._cpuUDS(_cpuUDS),
	._cpuLDS(_cpuLDS),
	._cpuRW(_cpuRW),
	.turbo(real_turbo),
	.configROMSize(1), // 128KB
	.configRAMSize(configRAMSize),
	.memoryAddr(memoryAddr),
	._memoryUDS(_memoryUDS),
	._memoryLDS(_memoryLDS),
	._romOE(_romOE),
	._ramOE(_ramOE),
	._ramWE(_ramWE),
	.dioBusControl(dioBusControl),
	.cpuBusControl(cpuBusControl),
	.selectSCSI(selectSCSI),
	.selectSCC(selectSCC),
	.selectIWM(selectIWM),
	.selectVIA(selectVIA),
	._vblank(_vblank),
	._hblank(_hblank),
	.memoryOverlayOn(memoryOverlayOn),

	.snd_alt(snd_alt),
	.loadSound(loadSound),

	.dskReadAddrInt(dskReadAddrInt),
	.dskReadAckInt(dskReadAckInt),
	.dskReadAddrExt(dskReadAddrExt),
	.dskReadAckExt(dskReadAckExt)
);

dataController_top dc0
(
	.clk(clk_sys),
	.cep(cep),
	.cen(cen),
	
	._systemReset(n_reset),
	._cpuReset(_cpuReset), 
	._cpuIPL(_cpuIPL),
	._cpuUDS(_cpuUDS), 
	._cpuLDS(_cpuLDS), 
	._cpuRW(_cpuRW), 
	.cpuDataIn(cpuDataOut),
	.cpuDataOut(dataControllerDataOut), 	
	.cpuAddrRegHi(cpuAddr[12:9]),
	.cpuAddrRegMid(cpuAddr[6:4]),  // for SCSI
	.cpuAddrRegLo(cpuAddr[2:1]),		
	.selectSCSI(selectSCSI),
	.selectSCC(selectSCC),
	.selectIWM(selectIWM),
	.selectVIA(selectVIA),
	.cpuBusControl(cpuBusControl),
	.memoryDataOut(memoryDataOut),
	.memoryDataIn(sdram_do),

	// peripherals
	.ps2_key(ps2_key),
	.capslock(capslock),
	.ps2_mouse(ps2_mouse),
	.serialIn(0),

	// video
	._hblank(_hblank),
	._vblank(_vblank),

	.memoryOverlayOn(memoryOverlayOn),

	.audioOut(audio),
	.snd_alt(snd_alt),
	.loadSound(loadSound),
	
	// floppy disk interface
	.insertDisk({dsk_ext_ins, dsk_int_ins}),
	.diskSides({dsk_ext_ds, dsk_int_ds}),
	.diskEject(diskEject),
	.dskReadAddrInt(dskReadAddrInt),
	.dskReadAckInt(dskReadAckInt),
	.dskReadAddrExt(dskReadAddrExt),
	.dskReadAckExt(dskReadAckExt),

	.diskMotor(diskMotor),
	.diskAct(diskAct),

	.img_mounted(img_mounted),
	
	// block device interface for scsi disk
	.io_req_type(sd_req_type),
	.io_lba(sd_lba),
	.io_rd(sd_rd),
	.io_wr(sd_wr),
	.io_ack(sd_ack),

	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr)
);

reg disk_act;
always @(posedge clk_sys) begin
	integer timeout = 0;

	if(timeout) begin
		timeout <= timeout - 1;
		disk_act <= 1;
	end else begin
		disk_act <= 0;
	end

	if(|diskAct) timeout <= 1000000;
end

//////////////////////// DOWNLOADING ///////////////////////////

// include ROM download helper
wire        dio_download;
reg         dio_write;
wire [23:0] dio_addr = ioctl_addr[24:1];
wire  [7:0] dio_index;
reg  [15:0] dio_data;

// good floppy image sizes are 819200 bytes and 409600 bytes
reg dsk_int_ds, dsk_ext_ds;  // double sided image inserted
reg dsk_int_ss, dsk_ext_ss;  // single sided image inserted

// any known type of disk image inserted?
wire dsk_int_ins = dsk_int_ds || dsk_int_ss;
wire dsk_ext_ins = dsk_ext_ds || dsk_ext_ss;

// at the end of a download latch file size
// diskEject is set by macos on eject
always @(posedge clk_sys) begin
	reg old_down;

	old_down <= dio_download;
	if(old_down && ~dio_download && dio_index == 1) begin
		dsk_int_ds <= (dio_addr == 409600);   // double sides disk, addr counts words, not bytes
		dsk_int_ss <= (dio_addr == 204800);   // single sided disk
	end

	if(diskEject[0]) begin
		dsk_int_ds <= 0;
		dsk_int_ss <= 0;
	end
end

always @(posedge clk_sys) begin
	reg old_down;

	old_down <= dio_download;
	if(old_down && ~dio_download && dio_index == 2) begin
		dsk_ext_ds <= (dio_addr == 409600);   // double sided disk, addr counts words, not bytes
		dsk_ext_ss <= (dio_addr == 204800);   // single sided disk
	end

	if(diskEject[1]) begin
		dsk_ext_ds <= 0;
		dsk_ext_ss <= 0;
	end
end

// disk images are being stored right after os rom at word offset 0x80000 and 0x100000 
wire [20:0] dio_a = 
	(dio_index == 0)?dio_addr[20:0]:                 // os rom
	(dio_index == 1)?{21'h80000 + dio_addr[20:0]}:   // first dsk image at 512k word addr
	{21'h100000 + dio_addr[20:0]};                   // second dsk image at 1M word addr

always @(posedge clk_sys) begin
	reg old_cyc = 0;
	
	old_cyc <= dioBusControl;
	if(ioctl_wr) ioctl_wait <= 1;

	if(~dioBusControl) dio_write <= ioctl_wait;
	if(old_cyc & ~dioBusControl & dio_write) ioctl_wait <= 0;
end

// sdram used for ram/rom maps directly into 68k address space
wire download_cycle = dio_download && dioBusControl;

////////////////////////// SDRAM /////////////////////////////////

wire [24:0] sdram_addr = download_cycle ? { 4'b0001, dio_a[20:0] } : { 3'b000, ~_romOE, memoryAddr[21:1] };
wire [15:0] sdram_din  = download_cycle ? dio_data : memoryDataOut;
wire  [1:0] sdram_ds   = download_cycle ? 2'b11 : { !_memoryUDS, !_memoryLDS };
wire        sdram_we   = download_cycle ? dio_write : !_ramWE;
wire        sdram_oe   = download_cycle ? 1'b0 : (!_ramOE || !_romOE);
wire [15:0] sdram_do   = download_cycle ? 16'hffff : (dskReadAckInt || dskReadAckExt) ? extra_rom_data_demux : sdram_out;

// "extra rom" is used to hold the disk image. It's expected to be byte wide and
// we thus need to properly demultiplex the word returned from sdram in that case
wire [15:0] extra_rom_data_demux = memoryAddr[0]? {sdram_out[7:0],sdram_out[7:0]}:{sdram_out[15:8],sdram_out[15:8]};
wire [15:0] sdram_out;

assign SDRAM_CKE = 1;

sdram sdram
(
	// system interface
	.init    ( !pll_locked ),
	.clk     ( clk_sys     ),
	.sync    ( cep         ),

	// interface to the MT48LC16M16 chip
	.sd_data ( SDRAM_DQ    ),
	.sd_addr ( SDRAM_A     ),
	.sd_dqm  ( {SDRAM_DQMH, SDRAM_DQML} ),
	.sd_cs   ( SDRAM_nCS   ),
	.sd_ba   ( SDRAM_BA    ),
	.sd_we   ( SDRAM_nWE   ),
	.sd_ras  ( SDRAM_nRAS  ),
	.sd_cas  ( SDRAM_nCAS  ),

	// cpu/chipset interface
	// map rom to sdram word address $200000 - $20ffff
	.din     ( sdram_din   ),
	.addr    ( sdram_addr  ),
	.ds      ( sdram_ds    ),
	.we      ( sdram_we    ),
	.oe      ( sdram_oe    ),
	.dout    ( sdram_out   )
);


//////////////////////// TURBO HANDLING //////////////////////////

// cannot boot from SCSI if turbo enabled
// delay the turbo.
reg real_turbo = 0;
always @(posedge clk_sys) begin
	reg old_ack;
	integer ack_cnt = 0;
	
	old_ack <= sd_ack;
	if(old_ack && ~sd_ack && ack_cnt) ack_cnt <= ack_cnt - 1'd1;

	//Cancel delay if FDD is accesed.
	if(diskMotor) ack_cnt <= 0;
	if(!ack_cnt && dioBusControl) real_turbo <= status_turbo;

	if(~n_reset) begin
		real_turbo <= 0;
		ack_cnt <= 20;
	end
end

endmodule
