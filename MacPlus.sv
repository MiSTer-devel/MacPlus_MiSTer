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
	inout  [48:0] HPS_BUS,

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
	output        VGA_DISABLE, // analog out is off

	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,

`ifdef MISTER_FB
	// Use framebuffer in DDRAM
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

`ifdef MISTER_FB_PALETTE
	// Palette control for 8bit modes.
	// Ignored for other video modes.
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
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

`ifdef MISTER_DUAL_SDRAM
	//Secondary SDRAM
	//Set all output SDRAM_* signals to Z ASAP if SDRAM2_EN is 0
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

assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0; 
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign LED_USER  = dio_download || (disk_act ^ |diskMotor);
assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;
assign VGA_SCALER= 0;
assign HDMI_FREEZE = 0;
assign VGA_DISABLE = 0;

wire [1:0] ar = status[8:7];
video_freak video_freak
(
	.*,
	.VGA_DE_IN(VGA_DE),
	.VGA_DE(),

	.ARX((!ar) ? 12'd256 : (ar - 1'd1)),
	.ARY((!ar) ? 12'd171 : 12'd0),
	.CROP_SIZE(0),
	.CROP_OFF(0),
	.SCALE(status[12:11])
);

`include "build_id.v" 
localparam CONF_STR = {
	"MACPLUS;UART115200;",
	"-;",
	"F1,DSK,Mount Pri Floppy;",
	"F2,DSK,Mount Sec Floppy;",
	"-;",
	"SC0,IMGVHD,Mount SCSI-6;",
	"SC1,IMGVHD,Mount SCSI-5;",
	"-;",
	"O78,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"OBC,Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"-;",
	"O9,Model,Plus,SE;",
	"O5,Speed,8MHz,16MHz;",
	"ODE,CPU,68000,68010,68020;",
	"O4,Memory,1MB,4MB;",
	"-;",
	//"OA,Serial,Off,On;",
	//"-;",
	"R0,Reset & Apply CPU+Memory;",
	"v,0;", // [optional] config version 0-99. 
	        // If CONF_STR options are changed in incompatible way, then change version number too,
			// so all options will get default values on first start.
	"V,v",`BUILD_DATE
};

wire status_turbo = status[5];

////////////////////   CLOCKS   ///////////////////

wire clk_sys, clk_mem;
wire pll_locked;

pll pll
(
	.refclk(CLK_50M),
	.outclk_0(clk_mem),
	.outclk_1(clk_sys),
	.locked(pll_locked)
);

reg       status_mem;
reg [1:0] status_cpu;
reg       status_mod;
reg       n_reset = 0;
always @(posedge clk_sys) begin
	reg [15:0] rst_cnt;

	if (clk8_en_p) begin
		// various sources can reset the mac
		if(~pll_locked || status[0] || buttons[1] || RESET || ~_cpuReset_o) begin
			rst_cnt <= '1;
			n_reset <= 0;
		end
		else if(rst_cnt) begin
			rst_cnt    <= rst_cnt - 1'd1;
			status_mem <= status[4];
			status_cpu <= status[14:13];
			status_mod <= status[9];
		end
		else begin
			n_reset <= 1;
		end
	end
end

///////////////////////////////////////////////////

localparam SCSI_DEVS = 2;

// the status register is controlled by the on screen display (OSD)
wire [31:0] status;
wire  [1:0] buttons;
wire [31:0] sd_lba[SCSI_DEVS];
wire  [SCSI_DEVS-1:0] sd_rd;
wire  [SCSI_DEVS-1:0] sd_wr;
wire  [SCSI_DEVS-1:0] sd_ack;
wire            [7:0] sd_buff_addr;
wire           [15:0] sd_buff_dout;
wire           [15:0] sd_buff_din[SCSI_DEVS];
wire                  sd_buff_wr;
wire  [SCSI_DEVS-1:0] img_mounted;
wire           [63:0] img_size;

wire        ioctl_write;
reg         ioctl_wait = 0;

wire [10:0] ps2_key;
wire [24:0] ps2_mouse;
wire        capslock;

wire [24:0] ioctl_addr;
wire [15:0] ioctl_data;

wire [32:0] TIMESTAMP;

hps_io #(.CONF_STR(CONF_STR), .VDNUM(SCSI_DEVS), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.buttons(buttons),
	.status(status),

	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),

	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),
	
	.img_mounted(img_mounted),
	.img_size(img_size),

	.ioctl_download(dio_download),
	.ioctl_index(dio_index),
	.ioctl_wr(ioctl_write),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_data),
	.ioctl_wait(ioctl_wait),

	.TIMESTAMP(TIMESTAMP),

	.ps2_key(ps2_key),
	.ps2_kbd_led_use(3'b001),
	.ps2_kbd_led_status({2'b00, capslock}),

	.ps2_mouse(ps2_mouse)
);

assign CLK_VIDEO = clk_sys;
assign CE_PIXEL  = 1;

assign VGA_R  = {8{pixelOut}};
assign VGA_G  = {8{pixelOut}};
assign VGA_B  = {8{pixelOut}};
assign VGA_DE = _vblank & _hblank;
assign VGA_VS = vsync;
assign VGA_HS = hsync;
assign VGA_F1 = 0;
assign VGA_SL = 0;

wire [10:0] audio;
assign AUDIO_L = {audio[10:0], 5'b00000};
assign AUDIO_R = {audio[10:0], 5'b00000};
assign AUDIO_S = 1;
assign AUDIO_MIX = 0;


// ------------------------------ Plus Too Bus Timing ---------------------------------
// for stability and maintainability reasons the whole timing has been simplyfied:
//                00           01             10           11
//    ______ _____________ _____________ _____________ _____________ ___
//    ______X_video_cycle_X__cpu_cycle__X__IO_cycle___X__cpu_cycle__X___
//                        ^      ^    ^                      ^    ^
//                        |      |    |                      |    |
//                      video    | CPU|                      | CPU|
//                       read   write read                  write read



// set the real-world inputs to sane defaults
localparam 	  configROMSize = 1'b1;  // 128K ROM

wire [1:0] configRAMSize = status_mem?2'b11:2'b10; // 1MB/4MB
			  
//
// Serial Ports
//
wire serialOut;
wire serialIn;
wire serialCTS;
wire serialRTS;

/*
assign serialIn = ~status[10] ? 0 : UART_RXD;
assign UART_TXD = serialOut;
assign serialCTS = UART_CTS;
assign UART_RTS = serialRTS;
assign UART_DTR = UART_DSR;
*/

//assign serialIn = ~status[10] ? 0 : UART_RXD;
assign serialIn =  UART_RXD;
assign UART_TXD = serialOut;
//assign UART_RTS = UART_CTS;
assign UART_RTS = serialRTS ;
assign UART_DTR = UART_DSR;

//assign {UART_RTS, UART_TXD, UART_DTR} = 0;
/*
	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,
*/


// interconnects
// CPU
wire clk8, _cpuReset, _cpuReset_o, _cpuUDS, _cpuLDS, _cpuRW, _cpuAS;
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
wire vid_alt, loadPixels, pixelOut, _hblank, _vblank, hsync, vsync;
wire memoryOverlayOn, selectSCSI, selectSCC, selectIWM, selectVIA, selectRAM, selectROM, selectSEOverlay;
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
always @(posedge clk_sys) begin
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
assign      _cpuReset_o   = is68000 ? fx68_reset_n : tg68_reset_n;
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
wire        fx68_reset_n;

fx68k fx68k (
	.clk        ( clk_sys ),
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
	.oRESETn    ( fx68_reset_n ),
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
wire        tg68_reset_n;

tg68k tg68k (
	.clk        ( clk_sys      ),
	.reset      ( !_cpuReset ),
	.phi1       ( cpu_en_p  ),
	.phi2       ( cpu_en_n  ),
	.cpu        ( {status_cpu[1], |status_cpu} ),

	.dtack_n    ( _cpuDTACK  ),
	.rw_n       ( tg68_rw    ),
	.as_n       ( tg68_as_n  ),
	.uds_n      ( tg68_uds_n ),
	.lds_n      ( tg68_lds_n ),
	.fc         ( { tg68_fc2, tg68_fc1, tg68_fc0 } ),
	.reset_n    ( tg68_reset_n ),

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

addrController_top ac0
(
	.clk(clk_sys),
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
	.turbo(status_turbo),
	.configROMSize({status_mod,~status_mod}),
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
	.selectSEOverlay(selectSEOverlay),
	.hsync(hsync), 
	.vsync(vsync),
	._hblank(_hblank),
	._vblank(_vblank),
	.loadPixels(loadPixels),
	.vid_alt(vid_alt),
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

dataController_top #(SCSI_DEVS) dc0
(
	.clk32(clk_sys), 
	.clk8_en_p(clk8_en_p),
	.clk8_en_n(clk8_en_n),
	.E_rising(E_rising),
	.E_falling(E_falling),
	.machineType(status_mod),
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
	.selectSEOverlay(selectSEOverlay),
	.cpuBusControl(cpuBusControl),
	.videoBusControl(videoBusControl),
	.memoryDataOut(memoryDataOut),
	.memoryDataIn(sdram_do),
	.memoryLatch(memoryLatch),

	// peripherals
	.ps2_key(ps2_key), 
	.capslock(capslock),
	.ps2_mouse(ps2_mouse),
	// serial uart
	.serialIn(serialIn),
	.serialOut(serialOut),
	.serialCTS(serialCTS),
	.serialRTS(serialRTS),

	// rtc unix ticks
	.timestamp(TIMESTAMP),

	// video
	._hblank(_hblank),
	._vblank(_vblank), 
	.pixelOut(pixelOut),
	.loadPixels(loadPixels),
	.vid_alt(vid_alt),

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

	// block device interface for scsi disk
	.img_mounted(img_mounted),
	.img_size(img_size[40:9]),
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

	if(|diskAct) timeout <= 500000;
end

//////////////////////// DOWNLOADING ///////////////////////////

// include ROM download helper
wire dio_download;
wire [23:0] dio_addr = ioctl_addr[24:1];
wire  [7:0] dio_index;

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
reg [20:0] dio_a;
reg [15:0] dio_data;
reg        dio_write;

always @(posedge clk_sys) begin
	reg old_cyc = 0;
	
	if(ioctl_write) begin
		dio_data <= {ioctl_data[7:0], ioctl_data[15:8]};
		dio_a <= dio_index[1:0] ? {dio_index[1:0], dio_addr[18:0]} : {dio_index[6], dio_addr[17:0]};
		ioctl_wait <= 1;
	end

	old_cyc <= dioBusControl;
	if(~dioBusControl) dio_write <= ioctl_wait;
	if(old_cyc & ~dioBusControl & dio_write) ioctl_wait <= 0;
end


// sdram used for ram/rom maps directly into 68k address space
wire download_cycle = dio_download && dioBusControl;

////////////////////////// SDRAM /////////////////////////////////

wire [24:0] sdram_addr = download_cycle ? {4'b0001, dio_a[20:0] } : 
                         ~_romOE        ? {4'b0001, 2'b00, status_mod, memoryAddr[18:1]} :
                                          {3'b000, (dskReadAckInt || dskReadAckExt), memoryAddr[21:1]};

wire [15:0] sdram_din  = download_cycle ? dio_data              : memoryDataOut;
wire  [1:0] sdram_ds   = download_cycle ? 2'b11                 : { !_memoryUDS, !_memoryLDS };
wire        sdram_we   = download_cycle ? dio_write             : !_ramWE;
wire        sdram_oe   = download_cycle ? 1'b0                  : (!_ramOE || !_romOE || dskReadAckInt || dskReadAckExt);
wire [15:0] sdram_do   = download_cycle ? 16'hffff : (dskReadAckInt || dskReadAckExt) ? extra_rom_data_demux : sdram_out;

// during rom/disk download ffff is returned so the screen is black during download
// "extra rom" is used to hold the disk image. It's expected to be byte wide and
// we thus need to properly demultiplex the word returned from sdram in that case
wire [15:0] extra_rom_data_demux = memoryAddr[0]? {sdram_out[7:0],sdram_out[7:0]}:{sdram_out[15:8],sdram_out[15:8]};
wire [15:0] sdram_out;

assign SDRAM_CKE = 1;

sdram sdram
(
	// system interface
	.init           ( !pll_locked              ),
	.clk_64         ( clk_mem                  ),
	.clk_8          ( clk8                     ),

	.sd_clk         ( SDRAM_CLK                ),
	.sd_data        ( SDRAM_DQ                 ),
	.sd_addr        ( SDRAM_A                  ),
	.sd_dqm         ( {SDRAM_DQMH, SDRAM_DQML} ),
	.sd_cs          ( SDRAM_nCS                ),
	.sd_ba          ( SDRAM_BA                 ),
	.sd_we          ( SDRAM_nWE                ),
	.sd_ras         ( SDRAM_nRAS               ),
	.sd_cas         ( SDRAM_nCAS               ),

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
