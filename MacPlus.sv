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
	//if VIDEO_ARX[12] or VIDEO_ARY[12] is set then [11:0] contains scaled size instead of aspect ratio.
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER, // Force VGA scaler

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,

`ifdef USE_FB
	// Use framebuffer in DDRAM (USE_FB=1 in qsf)
	// FB_FORMAT:
	//    [2:0] : 011=8bpp(palette) 100=16bpp 101=24bpp 110=32bpp
	//    [3]   : 0=16bits 565 1=16bits 1555
	//    [4]   : 0=RGB  1=BGR (for 16/24/32 modes)
	//
	// FB_STRIDE either 0 (rounded to 256 bytes) or multiple of pixel size (in bytes)
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,

	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif

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

	input         CLK_AUDIO, // 24.576 MHz
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

`ifdef USE_DDRAM
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
`endif

`ifdef USE_SDRAM
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
`endif

`ifdef DUAL_SDRAM
	//Secondary SDRAM
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

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

assign LED_USER  = (dio_download || |(diskAct ^ diskMotor));
assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;
assign VGA_SCALER= 0;


wire [1:0] ar = status[10:9];
video_freak video_freak
(
	.*,
	.VGA_DE_IN(VGA_DE),
	.VGA_DE(),

	.ARX((!ar) ? 12'd4 : (ar - 1'd1)),
	.ARY((!ar) ? 12'd3 : 12'd0),
	.CROP_SIZE(0),
	.CROP_OFF(0),
	.SCALE(status[12:11])
);

/*
// moved to the video_freak
assign VIDEO_ARX = status[9] ? 8'd16 : 12'd1535;
assign VIDEO_ARY = status[9] ? 8'd9  : 12'd1294;
*/

`include "build_id.v" 
localparam CONF_STR = {
	"MACPLUS;;",
	"-;",
	"F1,DSK,Mount Pri Floppy;",
	"F2,DSK,Mount Sec Floppy;",
	"-;",
	"S0,IMGVHD,Mount SCSI6;",
	"S1,IMGVHD,Mount SCSI2;",
	"-;",
	"O4,Memory,1MB,4MB;",
	"O5,Speed,8MHz,16MHz;",
	"O67,CPU,FX68K-68000,TG68K-68010,TG68K-68020;",
	"O9A,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"OBC,Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"-;",
	"R0,Reset;",
	"V,v",`BUILD_DATE
};



////////////////////   CLOCKS   ///////////////////
// 62.6688
// 31.3344
wire clk64;
wire clk32;
wire pll_locked;
		
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk64),
	.outclk_1(clk32),
	.locked(pll_locked)
);



	//assign SDRAM_CLK = clk64;

// ------------------------------ Plus Too Bus Timing ---------------------------------
// for stability and maintainability reasons the whole timing has been simplyfied:
//                00           01             10           11
//    ______ _____________ _____________ _____________ _____________ ___
//    ______X_video_cycle_X__cpu_cycle__X__IO_cycle___X__cpu_cycle__X___
//                        ^      ^    ^                      ^    ^
//                        |      |    |                      |    |
//                      video    | CPU|                      | CPU|
//                       read   write read                  write read

// -------------------------------------------------------------------------
// ------------------------------ data_io ----------------------------------
// -------------------------------------------------------------------------

// include ROM download helper
wire dio_download;
wire dio_write_i;
wire [23:0] dio_addr;
wire [4:0] dio_index;
wire [7:0] dio_data;

// good floppy image sizes are 819200 bytes and 409600 bytes
reg dsk_int_ds, dsk_ext_ds;  // double sided image inserted
reg dsk_int_ss, dsk_ext_ss;  // single sided image inserted

// any known type of disk image inserted?
wire dsk_int_ins = dsk_int_ds || dsk_int_ss;
wire dsk_ext_ins = dsk_ext_ds || dsk_ext_ss;

// at the end of a download latch file size
// diskEject is set by macos on eject
reg dio_download_d;
always @(posedge clk32) dio_download_d <= dio_download;

always @(posedge clk32) begin
	if(diskEject[0]) begin
		dsk_int_ds <= 1'b0;
		dsk_int_ss <= 1'b0;
	end else if(~dio_download && dio_download_d && dio_index == 1) begin
		//dsk_int_ds <= (dio_addr[23:1] == 409599);   // double sides disk, addr counts words, not bytes
		//dsk_int_ss <= (dio_addr[23:1] == 204799);   // single sided disk
		dsk_int_ds <= (dio_addr[23:1] == 409600);   // double sides disk, addr counts words, not bytes
		dsk_int_ss <= (dio_addr[23:1] == 204800);   // single sided disk
	end
end	

always @(posedge clk32) begin
	if(diskEject[1]) begin
		dsk_ext_ds <= 1'b0;
		dsk_ext_ss <= 1'b0;
	end else if(~dio_download && dio_download_d && dio_index == 2) begin
		//dsk_ext_ds <= (dio_addr[23:1] == 409599);   // double sided disk, addr counts words, not bytes
		//dsk_ext_ss <= (dio_addr[23:1] == 204799);   // single sided disk
		dsk_ext_ds <= (dio_addr[23:1] == 409600);   // double sided disk, addr counts words, not bytes
		dsk_ext_ss <= (dio_addr[23:1] == 204800);   // single sided disk
	end
end

// disk images are being stored right after os rom at word offset 0x80000 and 0x100000 
wire [21:1] dio_a = 
	(dio_index == 0)?dio_addr[21:1]:                 // os rom
	(dio_index == 1)?{21'h80000 + dio_addr[21:1]}:   // first dsk image at 512k word addr
	{21'h100000 + dio_addr[21:1]};                   // second dsk image at 1M word addr



	//assign SDRAM_CLK = clk64;
	// the configuration string is returned to the io controller to allow
	// it to control the menu on the OSD


	wire status_mem = status[4];
	wire status_turbo = status[5];
	wire [1:0] status_cpu = status[7:6];
	wire status_reset = status[0];

	// the status register is controlled by the on screen display (OSD)
	wire [31:0] status;
	wire [1:0] buttons;
	wire       ypbpr;
	// ps2 interface for mouse, to be mapped into user_io
	wire mouseClk;
	wire mouseData;
	wire keyClk;
	wire keyData;
	wire [63:0] rtc;

	wire [31:0] io_lba;
	wire [1:0] io_rd;
	wire [1:0] io_wr;
	wire       io_ack;
	wire [1:0] img_mounted;
	wire [31:0] img_size;
	wire [7:0] sd_buff_dout;
	wire       sd_buff_wr;
	wire [8:0] sd_buff_addr;
	wire [7:0] sd_buff_din;
	wire ioctl_wait;

	
wire [63:0] RTC;



hps_io #(.STRLEN($size(CONF_STR)>>3),.PS2DIV(1000), .VDNUM(2),.PS2WE(0)) hps_io
(
	.clk_sys(clk32),
	.HPS_BUS(HPS_BUS),

	.conf_str(CONF_STR),

	.buttons(buttons),
	.status(status),
	
		.img_mounted    ( img_mounted    ),
		.img_size       ( img_size       ),

	.sd_lba(io_lba),
	.sd_rd(io_rd),
	.sd_wr(io_wr),
	.sd_ack(io_ack),

	.sd_conf(0),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),
	
	.ioctl_download(dio_download),
	.ioctl_index(dio_index),
	.ioctl_wr(dio_write_i),
	.ioctl_addr(dio_addr),
	.ioctl_dout(dio_data),

	.ioctl_wait(ioctl_wait),

	.ps2_key(ps2_key),
	.ps2_kbd_led_use(3'b001),
	.ps2_kbd_led_status({2'b00, capslock}),
	.RTC(RTC),

	.ps2_mouse(ps2_mouse),
		.ps2_kbd_clk_out    ( keyClk         ),
		.ps2_kbd_data_out   ( keyData        ),
		.ps2_mouse_clk_out  ( mouseClk       ),
		.ps2_mouse_data_out ( mouseData	     )

	);



	// set the real-world inputs to sane defaults
	localparam serialIn = 1'b0,
				  configROMSize = 1'b1;  // 128K ROM

	wire [1:0] configRAMSize = status_mem?2'b11:2'b10; // 1MB/4MB
				  
	// interconnects
	// CPU
	wire clk8, _cpuReset, _cpuUDS, _cpuLDS, _cpuRW, _cpuAS;
	wire clk8_en_p, clk8_en_n;
	wire clk16_en_p, clk16_en_n;
	wire _cpuVMA, _cpuVPA, _cpuDTACK;
	wire E_rising, E_falling;
	wire [2:0] _cpuIPL;
	wire [2:0] cpuFC;
	wire [7:0] cpuAddrHi;
	wire [23:0] cpuAddr;
	wire [15:0] cpuDataOut;
	
	// RAM/ROM
	wire _romOE;
	wire _ramOE, _ramWE;
	wire _memoryUDS, _memoryLDS;
	wire videoBusControl;
	wire dioBusControl;
	wire cpuBusControl;
	wire [21:0] memoryAddr;
	wire [15:0] memoryDataOut;
	wire memoryLatch;
	
	// peripherals
	wire loadPixels, pixelOut, _hblank, _vblank, hsync, vsync;
	wire memoryOverlayOn, selectSCSI, selectSCC, selectIWM, selectVIA, selectRAM, selectROM;
	wire [15:0] dataControllerDataOut;
	
	// audio
	wire snd_alt;
	wire loadSound;

	// floppy disk image interface
	wire dskReadAckInt;
	wire [21:0] dskReadAddrInt;
	wire dskReadAckExt;
	wire [21:0] dskReadAddrExt;

	// dtack generation in turbo mode
	reg  turbo_dtack_en, cpuBusControl_d;
	always @(posedge clk32) begin
		if (!_cpuReset) begin
			turbo_dtack_en <= 0;
		end
		else begin
			cpuBusControl_d <= cpuBusControl;
			if (_cpuAS) turbo_dtack_en <= 0;
			if (!_cpuAS & ((!cpuBusControl_d & cpuBusControl) | (!selectROM & !selectRAM))) turbo_dtack_en <= 1;
		end
	end

	assign      _cpuVPA = (cpuFC == 3'b111) ? 1'b0 : ~(!_cpuAS && cpuAddr[23:21] == 3'b111);
	assign      _cpuDTACK = ~(!_cpuAS && cpuAddr[23:21] != 3'b111) | (status_turbo & !turbo_dtack_en);

	wire        cpu_en_p      = status_turbo ? clk16_en_p : clk8_en_p;
	wire        cpu_en_n      = status_turbo ? clk16_en_n : clk8_en_n;

	wire        is68000       = status_cpu == 0;
	assign      _cpuRW        = is68000 ? fx68_rw : tg68_rw;
	assign      _cpuAS        = is68000 ? fx68_as_n : tg68_as_n;
	assign      _cpuUDS       = is68000 ? fx68_uds_n : tg68_uds_n;
	assign      _cpuLDS       = is68000 ? fx68_lds_n : tg68_lds_n;
	assign      E_falling     = is68000 ? fx68_E_falling : tg68_E_falling;
	assign      E_rising      = is68000 ? fx68_E_rising : tg68_E_rising;
	assign      _cpuVMA       = is68000 ? fx68_vma_n : tg68_vma_n;
	assign      cpuFC[0]      = is68000 ? fx68_fc0 : tg68_fc0;
	assign      cpuFC[1]      = is68000 ? fx68_fc1 : tg68_fc1;
	assign      cpuFC[2]      = is68000 ? fx68_fc2 : tg68_fc2;
	assign      cpuAddr[23:1] = is68000 ? fx68_a : tg68_a[23:1];
	assign      cpuDataOut    = is68000 ? fx68_dout : tg68_dout;

	wire        fx68_rw;
	wire        fx68_as_n;
	wire        fx68_uds_n;
	wire        fx68_lds_n;
	wire        fx68_E_falling;
	wire        fx68_E_rising;
	wire        fx68_vma_n;
	wire        fx68_fc0;
	wire        fx68_fc1;
	wire        fx68_fc2;
	wire [15:0] fx68_dout;
	wire [23:1] fx68_a;

	fx68k fx68k (
		.clk        ( clk32 ),
		.extReset   ( !_cpuReset ),
		.pwrUp      ( !_cpuReset ),
		.enPhi1     ( cpu_en_p   ),
		.enPhi2     ( cpu_en_n   ),

		.eRWn       ( fx68_rw ),
		.ASn        ( fx68_as_n ),
		.LDSn       ( fx68_lds_n ),
		.UDSn       ( fx68_uds_n ),
		.E          ( ),
		.E_div      ( status_turbo ),
		.E_PosClkEn ( fx68_E_falling ),
		.E_NegClkEn ( fx68_E_rising ),
		.VMAn       ( fx68_vma_n ),
		.FC0        ( fx68_fc0 ),
		.FC1        ( fx68_fc1 ),
		.FC2        ( fx68_fc2 ),
		.BGn        ( ),
		.oRESETn    ( ),
		.oHALTEDn   ( ),
		.DTACKn     ( _cpuDTACK ),
		.VPAn       ( _cpuVPA ),
		.HALTn      ( 1'b1 ),
		.BERRn      ( 1'b1 ),
		.BRn        ( 1'b1 ),
		.BGACKn     ( 1'b1 ),
		.IPL0n      ( _cpuIPL[0] ),
		.IPL1n      ( _cpuIPL[1] ),
		.IPL2n      ( _cpuIPL[2] ),
		.iEdb       ( dataControllerDataOut ),
		.oEdb       ( fx68_dout ),
		.eab        ( fx68_a )
	);

	wire        tg68_rw;
	wire        tg68_as_n;
	wire        tg68_uds_n;
	wire        tg68_lds_n;
	wire        tg68_E_rising;
	wire        tg68_E_falling;
	wire        tg68_vma_n;
	wire        tg68_fc0;
	wire        tg68_fc1;
	wire        tg68_fc2;
	wire [15:0] tg68_dout;
	wire [31:0] tg68_a;

	tg68k tg68k (
		.clk        ( clk32      ),
		.reset      ( !_cpuReset ),
		.phi1       ( clk8_en_p  ),
		.phi2       ( clk8_en_n  ),
		.cpu        ( {status_cpu[1], |status_cpu} ),

		.dtack_n    ( _cpuDTACK  ),
		.rw_n       ( tg68_rw    ),
		.as_n       ( tg68_as_n  ),
		.uds_n      ( tg68_uds_n ),
		.lds_n      ( tg68_lds_n ),
		.fc         ( { tg68_fc2, tg68_fc1, tg68_fc0 } ),
		.reset_n    (  ),

		.E          (  ),
		.E_div      ( status_turbo ),
		.E_PosClkEn ( tg68_E_falling ),
		.E_NegClkEn ( tg68_E_rising  ),
		.vma_n      ( tg68_vma_n ),
		.vpa_n      ( _cpuVPA ),

		.br_n       ( 1'b1    ),
		.bg_n       (  ),
		.bgack_n    ( 1'b1 ),

		.ipl        ( _cpuIPL ),
		.berr       ( 1'b0 ),
		.din        ( dataControllerDataOut ),
		.dout       ( tg68_dout ),
		.addr       ( tg68_a )
	);

	addrController_top ac0(
		.clk(clk32),
		.clk8(clk8),
		.clk8_en_p(clk8_en_p),
		.clk8_en_n(clk8_en_n),
		.clk16_en_p(clk16_en_p),
		.clk16_en_n(clk16_en_n),
		.cpuAddr(cpuAddr), 
		._cpuUDS(_cpuUDS),
		._cpuLDS(_cpuLDS),
		._cpuRW(_cpuRW),
		._cpuAS(_cpuAS),
		.turbo (status_turbo),
		.configROMSize(configROMSize), 
		.configRAMSize(configRAMSize), 
		.memoryAddr(memoryAddr),
		.memoryLatch(memoryLatch),
		._memoryUDS(_memoryUDS),
		._memoryLDS(_memoryLDS),
		._romOE(_romOE), 
		._ramOE(_ramOE), 
		._ramWE(_ramWE),
		.videoBusControl(videoBusControl),	
		.dioBusControl(dioBusControl),	
		.cpuBusControl(cpuBusControl),	
		.selectSCSI(selectSCSI),
		.selectSCC(selectSCC),
		.selectIWM(selectIWM),
		.selectVIA(selectVIA),
		.selectRAM(selectRAM),
		.selectROM(selectROM),
		.hsync(hsync), 
		.vsync(vsync),
		._hblank(_hblank),
		._vblank(_vblank),
		.loadPixels(loadPixels),
		.memoryOverlayOn(memoryOverlayOn),

		.snd_alt(snd_alt),
		.loadSound(loadSound),

		.dskReadAddrInt(dskReadAddrInt),
		.dskReadAckInt(dskReadAckInt),
		.dskReadAddrExt(dskReadAddrExt),
		.dskReadAckExt(dskReadAckExt)
	);

	wire [1:0] diskEject;
	wire [1:0] diskMotor, diskAct;

	// addional ~8ms delay in reset
	wire rom_download = dio_download && (dio_index == 0);
	wire n_reset = (rst_cnt == 0);
	reg [15:0] rst_cnt;
	reg last_mem_config;
	reg [1:0] last_cpu_config;
	always @(posedge clk32) begin
		if (clk8_en_p) begin
			last_mem_config <= status_mem;
			last_cpu_config <= status_cpu;
	
			// various sources can reset the mac
			if(!pll_locked || status_reset || buttons[1] || 
				rom_download || (last_mem_config != status_mem) || (last_cpu_config != status_cpu)) 
				rst_cnt <= 16'd65535;
			else if(rst_cnt != 0)
				rst_cnt <= rst_cnt - 16'd1;
		end
	end


	dataController_top dc0(
		.clk32(clk32), 
		.clk8_en_p(clk8_en_p),
		.clk8_en_n(clk8_en_n),
		.E_rising(E_rising),
		.E_falling(E_falling),
		._systemReset(n_reset),
		._cpuReset(_cpuReset), 
		._cpuIPL(_cpuIPL),
		._cpuUDS(_cpuUDS), 
		._cpuLDS(_cpuLDS), 
		._cpuRW(_cpuRW), 
		._cpuVMA(_cpuVMA),
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
		.videoBusControl(videoBusControl),
		.memoryDataOut(memoryDataOut),
		.memoryDataIn(sdram_do),
		.memoryLatch(memoryLatch),

		// peripherals
		.keyClk(keyClk), 
		.keyData(keyData), 
		.mouseClk(mouseClk),
		.mouseData(mouseData),
		.serialIn(serialIn),
		.rtc(rtc),

		// video
		._hblank(_hblank),
		._vblank(_vblank), 
		.pixelOut(pixelOut),
		.loadPixels(loadPixels),

		.memoryOverlayOn(memoryOverlayOn),

		.audioOut(audio),
		.snd_alt(snd_alt),
		.loadSound(loadSound),

		// floppy disk interface
		.insertDisk( { dsk_ext_ins, dsk_int_ins} ),
		.diskSides( { dsk_ext_ds, dsk_int_ds} ),
		.diskEject(diskEject),
		.dskReadAddrInt(dskReadAddrInt),
		.dskReadAckInt(dskReadAckInt),
		.dskReadAddrExt(dskReadAddrExt),
		.dskReadAckExt(dskReadAckExt),
		.diskMotor(diskMotor),
		.diskAct(diskAct),

		// block device interface for scsi disk
		.img_mounted  ( img_mounted  ),
		.img_size     ( img_size     ),
		.io_lba       ( io_lba       ),
		.io_rd        ( io_rd        ),
		.io_wr        ( io_wr        ),
		.io_ack       ( io_ack       ),
		.sd_buff_addr ( sd_buff_addr ),
		.sd_buff_dout ( sd_buff_dout ),
		.sd_buff_din  ( sd_buff_din  ),
		.sd_buff_wr   ( sd_buff_wr   )
	);


wire [10:0] audio;
assign AUDIO_L = {audio[10:0], 5'b00000};
assign AUDIO_R = {audio[10:0], 5'b00000};
assign AUDIO_S = 1;
assign AUDIO_MIX = 0;

reg ce_pix;

always @(posedge clk32) begin : pix_div
	ce_pix <= ~ce_pix;
end

assign VGA_R = {8{pixelOut}};
assign VGA_G = {8{pixelOut}};
assign VGA_B = {8{pixelOut}};
assign CLK_VIDEO = clk32;
assign CE_PIXEL  = 1'b1;//ce_pix;
assign VGA_F1 = 0;
assign VGA_SL = 0;
assign VGA_DE = ~(~_vblank | ~_hblank);
assign VGA_VS = vsync;
assign VGA_HS = hsync;

/*
wire dio_clkref_n = ~(~download_cycle & download_cycle_d);
reg dio_write;
reg download_cycle_d;
always @(posedge clk32) begin
	download_cycle_d <= download_cycle;
	if (~dio_clkref_n) dio_write <= 0;
	if (dio_write_i) dio_write <= 1;
end
*/
reg dio_write;

always @(posedge clk32) begin
	reg old_cyc = 0;
	
	old_cyc <= dioBusControl;
	if(dio_write_i) ioctl_wait <= 1;

	if(~dioBusControl) dio_write <= ioctl_wait;
	if(old_cyc & ~dioBusControl & dio_write) ioctl_wait <= 0;
end

// sdram used for ram/rom maps directly into 68k address space
wire download_cycle = dio_download && dioBusControl;

wire [24:0] sdram_addr = download_cycle?{ 4'b0001, dio_a[21:1] }:{ 3'b000, ~_romOE, memoryAddr[21:1] };

wire [15:0] sdram_din = download_cycle?{dio_data,dio_data}:memoryDataOut;
wire [1:0] sdram_ds = download_cycle?{~dio_addr[0], dio_addr[0]}:{ !_memoryUDS, !_memoryLDS };
wire sdram_we = download_cycle?dio_write:!_ramWE;
wire sdram_oe = download_cycle?1'b0:(!_ramOE || !_romOE);

// during rom/disk download ffff is returned so the screen is black during download
// "extra rom" is used to hold the disk image. It's expected to be byte wide and
// we thus need to properly demultiplex the word returned from sdram in that case
wire [15:0] extra_rom_data_demux = memoryAddr[0]?
	{sdram_out[7:0],sdram_out[7:0]}:{sdram_out[15:8],sdram_out[15:8]};
wire [15:0] sdram_do = download_cycle?16'hffff:
	(dskReadAckInt || dskReadAckExt)?extra_rom_data_demux:
	sdram_out;

wire [15:0] sdram_out;

assign SDRAM_CKE         = 1'b1;

sdram sdram (
	// interface to the MT48LC16M16 chip
	.sd_clk  ( SDRAM_CLK   ),
	.sd_data        ( SDRAM_DQ                 ),
	.sd_addr        ( SDRAM_A                  ),
	.sd_dqm         ( {SDRAM_DQMH, SDRAM_DQML} ),
	.sd_cs          ( SDRAM_nCS                ),
	.sd_ba          ( SDRAM_BA                 ),
	.sd_we          ( SDRAM_nWE                ),
	.sd_ras         ( SDRAM_nRAS               ),
	.sd_cas         ( SDRAM_nCAS               ),

	// system interface
	.clk         ( clk64                    ),
	.sync          ( clk8                     ),
	//.clk_64         ( clk64                    ),
	//.clk_8          ( clk8                     ),
	.init           ( !pll_locked              ),

	// cpu/chipset interface
	// map rom to sdram word address $200000 - $20ffff
	.din            ( sdram_din                ),
	.addr           ( sdram_addr               ),
	.ds             ( sdram_ds                 ),
	.we             ( sdram_we                 ),
	.oe             ( sdram_oe                 ),
	.dout           ( sdram_out                )
);

endmodule

