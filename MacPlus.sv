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

// SDRAM region map: RAM, boot ROMs, floppy image staging
`include "rtl/sdram_map.vh"

module emu
(
	`include "sys/emu_ports.vh"
);

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;

assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0; 
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

assign LED_USER  = dio_download || ldr_int_busy || ldr_ext_busy || wr_int_busy || wr_ext_busy || (disk_act ^ |diskMotor);
assign LED_DISK  = 0;
assign LED_POWER = 0;
assign BUTTONS   = 0;
assign VGA_SCALER= 0;
assign VGA_DISABLE = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

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
	"S2,DSK,Mount Pri Floppy;",
	"S3,DSK,Mount Sec Floppy;",
	"-;",
	"D0SC0,IMGVHD,Mount SCSI-6;",
	"D0SC1,IMGVHD,Mount SCSI-5;",
	"-;",
	// CD-ROM (SCSI ID 3). CUE/BIN/CHD/TOAST are translated host-side by
	// Main_MiSTer's support/mac into a flat 2048-byte-sector view of the data
	// track, so the core still sees plain 2048 sectors and the TOC the target
	// reports stays consistent with what the guest is shown. Audio tracks are
	// hidden from that view; rtl/cd_audio.sv plays them from the real track
	// list instead.
	// Extension list is MacLC.sv:81 verbatim - the host-side translation is
	// keyed off the file, not the core, so the lists must agree.
	// The D0 prefix here and on SC0/SC1/OI/OFG greys these items out on a
	// model with no SCSI bus, from status_menumask bit 0 below.
	"D0SC4,ISOTO*CUEBINCHD,Mount CD-ROM;",
	// Apple HD20 on the external floppy port: a hard disk image, so the SC
	// form and extension list of the SCSI slots. The external floppy sits
	// behind it on the daisy chain (rtl/iwm.v).
	"SC5,IMGVHD,Mount HD20;",
	"D0OI,CD-ROM Drive,Enabled,Disabled;",
	// Index 0 MUST be Full: `status` defaults to zero and unity is the wanted
	// default. Independent of the Mac's volume control on purpose -- on real
	// hardware the Mac's setting had no effect on the drive, which had its own
	// knob; this is the honest equivalent of that knob, and without it there is
	// no way to balance the two sources at all. Bits 15-16 (F,G) were free.
	"D0OFG,CD Volume,Full,3/4,1/2,Off;",
	"-;",
	"O78,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"OBC,Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"-;",
	// bits 1-3: five models need three bits and bit 10 is the serial input,
	// hence the config version bump below
	"O13,Model,Plus,SE,512K,128K,512Ke;",
	"O5,Speed,8MHz,16MHz;",
	"O6,Floppy Write,Off,On;",
	"ODE,CPU,68000,68010,68020;",
	"D1O4,Memory,1MB,4MB;",
	"-;",
	//"OA,Serial,Off,On;",
	//"-;",
	"R0,Reset & Apply CPU+Memory;",
	"v,1;", // [optional] config version 0-99.
	        // If CONF_STR options are changed in incompatible way, then change version number too,
			// so all options will get default values on first start.
	"V,v",`BUILD_DATE
};

wire status_turbo = status[5];

// OSD items unavailable on the selected model; driven beside mac_model below
wire [15:0] status_menumask;

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
reg [2:0] status_model;
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
			status_model <= status[3:1];
		end
		else begin
			n_reset <= 1;
		end
	end
end

///////////////////////////////////////////////////

// SCSI targets: index 0/1 are the disks at IDs 6/5, index 2 is the CD-ROM at
// ID 3. The index order here is the ncr5380's
// internal device order, NOT the hps_io slot order - see the slot mapping below.
localparam SCSI_DEVS   = 3;
localparam SCSI_CD_DEV = 2;
// VDNUM: slots 0/1 = SCSI disks (unchanged), slots 2/3 = the two floppies
// (converted from ioctl_download F1/F2 to real S-type block-device mounts),
// slot 4 = CD-ROM, slot 5 = the Apple HD20 on the external drive port.
localparam VDNUM = 6;

// the status register is controlled by the on screen display (OSD)
wire [31:0] status;
wire  [1:0] buttons;
wire [31:0] sd_lba[VDNUM];
wire  [VDNUM-1:0] sd_rd;
wire  [VDNUM-1:0] sd_wr;
wire  [VDNUM-1:0] sd_ack;
// 13 bits, not 8: hps_io drives [AW:0] with AW = WIDE ? 12 : 13, and every
// transfer this core did before the CD-ROM was 512 bytes = 256 16-bit words =
// exactly 8 bits, so the top 5 were silently truncated and nothing noticed.
// CD-DA frames are 2352 bytes = 1176 words and need 11. Consumers that still
// want 512-byte addressing slice [7:0] explicitly at their port - identical
// bits to what the implicit truncation gave them (MacLC.sv:203 does the same).
wire           [12:0] sd_buff_addr;
wire signed    [15:0] cd_snd_l, cd_snd_r;   // CD-DA pair, summed in by cd_mix
wire           [15:0] sd_buff_dout;
wire           [15:0] sd_buff_din[VDNUM];
wire                  sd_buff_wr;
wire  [VDNUM-1:0] img_mounted;
wire           [63:0] img_size;
wire                  img_readonly;

// SCSI (dataController_top) only ever sees slots 0/1 of the VDNUM=4 arrays
// above - these are its own narrower view, mirroring how each floppy_loader
// below gets scalar per-slot ports instead of an array (the same pattern
// UK101.sv uses per-drive: each consumer indexes the shared array itself,
// no consumer declares its own sub-array port).
// SCSI device index -> hps_io slot: 0 -> 0 (disk, ID 6), 1 -> 1 (disk, ID 5),
// 2 -> 4 (CD-ROM, ID 3). Slots 2/3 belong to the floppies, so the CD's slot is
// deliberately not contiguous with the disks' and every SCSI vector has to be
// assembled by hand rather than sliced.
wire [31:0] scsi_sd_lba[SCSI_DEVS];
wire [15:0] scsi_sd_buff_din[SCSI_DEVS];
wire [SCSI_DEVS-1:0] scsi_sd_rd, scsi_sd_wr;
assign sd_lba[0] = scsi_sd_lba[0];
assign sd_lba[1] = scsi_sd_lba[1];
assign sd_lba[4] = scsi_sd_lba[SCSI_CD_DEV];
assign sd_buff_din[0] = scsi_sd_buff_din[0];
assign sd_buff_din[1] = scsi_sd_buff_din[1];
assign sd_buff_din[4] = scsi_sd_buff_din[SCSI_CD_DEV];

// slot 5, the HD20; rtl/dcd.v owns it outright
wire [31:0] dcd_sd_lba;
wire        dcd_sd_rd, dcd_sd_wr;
wire [15:0] dcd_sd_buff_din;
assign sd_lba[5]      = dcd_sd_lba;
assign sd_buff_din[5] = dcd_sd_buff_din;

// CD-ROM drive present on the bus. Disabled (status[18] set) makes the CD
// target never answer selection, so the SCSI bus is bit-identical to a
// pre-CD build - both the period-purist switch and the A/B lever if the new
// target misbehaves on hardware.
wire cd_enable = ~status[18];

// sd_buff_din[2]/[3] driven below by each drive's floppy_sd_writer -
// only ever consulted by hps_io during a sd_wr session for that slot, which
// only the writer ever asserts, so no mux against the loader is needed here.

wire        ioctl_write;
reg         ioctl_wait = 0;

wire [10:0] ps2_key;
wire [24:0] ps2_mouse;
wire        capslock;

wire [24:0] ioctl_addr;
wire [15:0] ioctl_data;

wire [32:0] TIMESTAMP;

hps_io #(.CONF_STR(CONF_STR), .VDNUM(VDNUM), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),

	.buttons(buttons),
	.status(status),
	.status_menumask(status_menumask),

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
	.img_readonly(img_readonly),

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

// ---- Reclaim the Mac channel's headroom, then sum in the CD-DA pair ----
// `audio` is SIGNED (audio_latch x volume, so +-127*7), and it carries a
// +28,448 pedestal whenever sound is DISABLED -- deliberately, see
// dataController_top.sv:171 and PR #12 / bug #7. MiSTer's own DC_blocker
// strips that downstream, which is why nobody hears it today; but downstream is
// AFTER our sum, so full-scale CD audio would be added onto the pedestal and
// clipped before the framework ever saw it. rtl/cd_mix.v does the blocking in
// here, ahead of the adder.

// 48 kHz strobe: that is where sys/audio_out.sv point-samples AUDIO_L/R.
// 32.5 MHz / 677 = 48,006 Hz, 0.01% off -- irrelevant against a 1.9 Hz corner.
reg  [9:0] snd_ce_div = 0;
reg        snd_ce = 0;
always @(posedge clk_sys) begin
	if (snd_ce_div == 10'd676) begin snd_ce_div <= 0; snd_ce <= 1'b1; end
	else                       begin snd_ce_div <= snd_ce_div + 1'd1; snd_ce <= 1'b0; end
end

// The mount gate. NOT cd_audio.sv's `disc_audio`, tempting as that looks:
// that is `toc_valid && !t2_has_data`, i.e. AUDIO-ONLY disc, so
// it reads 0 for a mixed-mode disc -- a data track plus CD audio, which is
// exactly what a game with CD audio is -- and the correction would never engage
// for the main use case. Latch actual presence instead, the same way the floppy
// slots do above. Gated on mounted rather than playing because "playing" flips
// many times per disc and each flip would be a transition to manage mid-listen.
reg cd_mounted = 0;
always @(posedge clk_sys) begin
	if (img_mounted[4]) cd_mounted <= (img_size != 0);
	if (~cd_enable)     cd_mounted <= 1'b0;   // OSD "CD-ROM Drive: Disabled"
end

wire signed [15:0] mix_l, mix_r;
cd_mix cd_mix_inst (
	.clk        ( clk_sys              ),
	.ce         ( snd_ce               ),
	.cd_mounted ( cd_mounted           ),
	.mac_in     ( {audio[10:0], 5'b00000} ),
	.cd_l_in    ( cd_snd_l             ),
	.cd_r_in    ( cd_snd_r             ),
	.cd_vol     ( status[16:15]        ),
	.out_l      ( mix_l                ),
	.out_r      ( mix_r                )
);

assign AUDIO_L = mix_l;
assign AUDIO_R = mix_r;
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



// The model selection becomes hardware straps in one place; see rtl/mac_model.v.
wire [1:0] configROMSize;
wire       scsiPresent;   // 1 = this model has a SCSI bus; see rtl/mac_model.v
wire [1:0] configRAMSize;
wire       machineType;
wire [1:0] romSlot;
wire       drive800k;

mac_model mac_model
(
	.model         ( status_model  ),
	.mem_big       ( status_mem    ),
	.configROMSize ( configROMSize ),
	.scsiPresent   ( scsiPresent   ),
	.configRAMSize ( configRAMSize ),
	.machineType   ( machineType   ),
	.romSlot       ( romSlot       ),
	.drive800k     ( drive800k     ),
	.ramSoldered   (               )
);

// Grey out what a model cannot honour. A second mac_model instance is fed
// from the live status bits so greying follows the selection at once.
wire menu_scsiPresent;
wire menu_ramSoldered;

mac_model mac_model_menu
(
	.model         ( status[3:1]       ),
	.mem_big       ( 1'b0              ),
	.configROMSize (                   ),
	.scsiPresent   ( menu_scsiPresent  ),
	.configRAMSize (                   ),
	.machineType   (                   ),
	.romSlot       (                   ),
	.drive800k     (                   ),
	.ramSoldered   ( menu_ramSoldered  )
);

// Bit set = item unavailable, which is what uppercase D reads as.
assign status_menumask = {14'd0, menu_ramSoldered, ~menu_scsiPresent};
			  
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
wire        scsi_bus_hold; // SCSI pseudo-DMA back-pressure, see ncr5380.sv
wire [15:0] dataControllerDataOut;

// audio
wire snd_alt;
wire loadSound;
wire snd_advance;

// floppy disk image interface
wire dskReadAckInt;
wire [21:0] dskReadAddrInt;
wire dskReadAckExt;
wire [21:0] dskReadAddrExt;

// floppy image loader (SD-mount -> SDRAM), shared extra-slot-3 port
wire [21:0] ldr_int_wr_addr, ldr_ext_wr_addr;
wire        ldr_int_wr_req,  ldr_ext_wr_req;
wire        ldr_int_wr_ack,  ldr_ext_wr_ack;
wire [15:0] ldr_int_wr_data, ldr_ext_wr_data;
wire        dskLoadWrEn;
wire        dskLoadSelExt;

// floppy write-back (IWM write path -> SDRAM), same shared
// extra-slot-3 port. Combined with the loader's own request below, since
// addrController_top.v's arbiter is only
// ever given ONE request/ack per side - loader and committer share that
// one slot per side with the loader given fixed priority.
wire [21:0] wc_int_wr_addr, wc_ext_wr_addr;
wire        wc_int_wr_req,  wc_ext_wr_req;
wire        wc_int_wr_ack,  wc_ext_wr_ack;
wire [15:0] wc_int_wr_data, wc_ext_wr_data;

// SD persistence tap: mirrors each committed sector so
// floppy_sd_writer can shadow it out to the mounted .dsk over sd_wr.
wire        wc_int_commit_done,   wc_ext_commit_done;
wire [21:0] wc_int_commit_addr,   wc_ext_commit_addr;
wire        wc_int_commit_buf_wr, wc_ext_commit_buf_wr;
wire [7:0]  wc_int_commit_buf_addr, wc_ext_commit_buf_addr;
wire [15:0] wc_int_commit_buf_data, wc_ext_commit_buf_data;

// per-side combined (loader-or-committer) request presented to
// addrController_top.v; loader wins whenever it is requesting, since a
// mount and a write-commit contending for the same side is only possible
// as a rare corner case, never a steady-state situation.
wire [21:0] slot3_int_addr = ldr_int_wr_req ? ldr_int_wr_addr : wc_int_wr_addr;
wire        slot3_int_req  = ldr_int_wr_req | wc_int_wr_req;
wire [15:0] slot3_int_data = ldr_int_wr_req ? ldr_int_wr_data : wc_int_wr_data;
wire        slot3_int_ack;
assign ldr_int_wr_ack = slot3_int_ack &  ldr_int_wr_req;
assign wc_int_wr_ack  = slot3_int_ack & ~ldr_int_wr_req;

wire [21:0] slot3_ext_addr = ldr_ext_wr_req ? ldr_ext_wr_addr : wc_ext_wr_addr;
wire        slot3_ext_req  = ldr_ext_wr_req | wc_ext_wr_req;
wire [15:0] slot3_ext_data = ldr_ext_wr_req ? ldr_ext_wr_data : wc_ext_wr_data;
wire        slot3_ext_ack;
assign ldr_ext_wr_ack = slot3_ext_ack &  ldr_ext_wr_req;
assign wc_ext_wr_ack  = slot3_ext_ack & ~ldr_ext_wr_req;

// OSD write-protect toggle (defaults to protected/status[6]=0), ANDed per
// drive with that drive's own latched img_readonly - see floppy_loader.v.
wire wp_int = ~status[6] | ldr_int_readonly;
wire wp_ext = ~status[6] | ldr_ext_readonly;

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
// SCSI back-pressure. Until now the SCSI space acknowledged unconditionally,
// so a target that could not serve the next pseudo-DMA byte had no way to say
// so and the transfer silently took stale data instead. It is the mechanism
// behind the 2026-08-26 CD->disk
// corruption. scsi_bus_hold is already qualified down to a DACK data access
// that cannot be served (see ncr5380.sv), so this only ever stretches the
// pseudo-DMA window; register reads, and every other device, are untouched.
// Bounded by the target's ~516 ms io-stall watchdog, which aborts the command
// and thereby releases the hold.
assign      _cpuDTACK = ~(!_cpuAS && cpuAddr[23:21] != 3'b111) | (status_turbo & !turbo_dtack_en) | scsi_bus_hold;

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
	.configROMSize(configROMSize),
	.scsiPresent(scsiPresent),
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
	.snd_advance(snd_advance),

	.dskReadAddrInt(dskReadAddrInt),
	.dskReadAckInt(dskReadAckInt),
	.dskReadAddrExt(dskReadAddrExt),
	.dskReadAckExt(dskReadAckExt),

	.dskLoadAddrInt(slot3_int_addr),
	.dskLoadReqInt(slot3_int_req),
	.dskLoadAckInt(slot3_int_ack),
	.dskLoadAddrExt(slot3_ext_addr),
	.dskLoadReqExt(slot3_ext_req),
	.dskLoadAckExt(slot3_ext_ack),
	.dskLoadWrEn(dskLoadWrEn),
	.dskLoadSelExt(dskLoadSelExt)
);

wire [1:0] diskEject;
wire [1:0] diskMotor, diskAct;

dataController_top #(.SCSI_DEVS(SCSI_DEVS), .SCSI_CD_DEV(SCSI_CD_DEV)) dc0
(
	.clk32(clk_sys), 
	.clk8_en_p(clk8_en_p),
	.clk8_en_n(clk8_en_n),
	.clk16_en_n(clk16_en_n),
	.E_rising(E_rising),
	.E_falling(E_falling),
	.machineType(machineType),
	.turbo(status_turbo),
	._systemReset(n_reset),
	._cpuReset(_cpuReset), 
	._cpuIPL(_cpuIPL),
	._cpuUDS(_cpuUDS), 
	._cpuLDS(_cpuLDS), 
	._cpuRW(_cpuRW), 
	._cpuVMA(_cpuVMA),
	.cpuDataIn(cpuDataOut),
	.cpuDataOut(dataControllerDataOut), 	
	.scsi_bus_hold(scsi_bus_hold),
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
	.snd_advance(snd_advance),

	// floppy disk interface
	.insertDisk({dsk_ext_ins, dsk_int_ins}),
	.img800k({dsk_ext_ds, dsk_int_ds}),
	// the drive mechanism, from mac_model
	.drive800k(drive800k),
	// each floppy_loader's mount-time verdict on the medium it just loaded
	.mediaSides({ldr_ext_media_ds, ldr_int_media_ds}),
	.diskEject(diskEject),
	.dskReadAddrInt(dskReadAddrInt),
	.dskReadAckInt(dskReadAckInt),
	.dskReadAddrExt(dskReadAddrExt),
	.dskReadAckExt(dskReadAckExt),
	.diskMotor(diskMotor),
	.diskAct(diskAct),

	.writeProtect({wp_ext, wp_int}),
	.dskWriteAddrInt(wc_int_wr_addr),
	.dskWriteDataInt(wc_int_wr_data),
	.dskWriteReqInt(wc_int_wr_req),
	.dskWriteAckInt(wc_int_wr_ack),
	.dskWriteAddrExt(wc_ext_wr_addr),
	.dskWriteDataExt(wc_ext_wr_data),
	.dskWriteReqExt(wc_ext_wr_req),
	.dskWriteAckExt(wc_ext_wr_ack),

	.dskCommitDoneInt(wc_int_commit_done),
	.dskCommitAddrInt(wc_int_commit_addr),
	.dskCommitBufWrInt(wc_int_commit_buf_wr),
	.dskCommitBufAddrInt(wc_int_commit_buf_addr),
	.dskCommitBufDataInt(wc_int_commit_buf_data),
	.dskCommitDoneExt(wc_ext_commit_done),
	.dskCommitAddrExt(wc_ext_commit_addr),
	.dskCommitBufWrExt(wc_ext_commit_buf_wr),
	.dskCommitBufAddrExt(wc_ext_commit_buf_addr),
	.dskCommitBufDataExt(wc_ext_commit_buf_data),

	// block device interface for scsi disk
	.img_mounted({img_mounted[4], img_mounted[1:0]}),
	.img_size(img_size[40:9]),
	.cd_enable(cd_enable),
	.io_lba(scsi_sd_lba),
	.io_rd(scsi_sd_rd),
	.io_wr(scsi_sd_wr),
	.io_ack({sd_ack[4], sd_ack[1:0]}),

	.sd_buff_addr(sd_buff_addr[7:0]),
	.sd_buff_addr_hi(sd_buff_addr[12:8]),   // CD whole-frame bursts
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(scsi_sd_buff_din),
	.sd_buff_wr(sd_buff_wr),

	// CD-DA pair. Summed into the Mac's audio by cd_mix above, NOT wired
	// straight to AUDIO_L/R: that would half-wave clip every track against
	// the +127 disabled-sound pedestal, which is why the DC blocker has to
	// come first.
	.cd_snd_l(cd_snd_l),
	.cd_snd_r(cd_snd_r),

	// block device interface for the HD20 (slot 5)
	.dcd_sd_lba(dcd_sd_lba),
	.dcd_sd_rd(dcd_sd_rd),
	.dcd_sd_wr(dcd_sd_wr),
	.dcd_sd_ack(sd_ack[5]),
	.dcd_sd_buff_addr(sd_buff_addr[7:0]),
	.dcd_sd_buff_dout(sd_buff_dout),
	.dcd_sd_buff_din(dcd_sd_buff_din),
	.dcd_sd_buff_wr(sd_buff_wr),
	.dcd_img_mounted(img_mounted[5]),
	.dcd_img_size(img_size),
	.dcd_img_readonly(img_readonly)
);

// sd_rd/sd_wr are consumer outputs -> hps_io inputs, so the SCSI 2-bit view
// above, each floppy_loader's own scalar sd_rd request, each
// floppy_sd_writer's own scalar sd_wr request and the HD20's pair must be
// combined into the full VDNUM vectors here.
wire ldr_int_sd_rd, ldr_ext_sd_rd;
wire wr_int_sd_wr,  wr_ext_sd_wr;
assign sd_rd = {dcd_sd_rd, scsi_sd_rd[SCSI_CD_DEV], ldr_ext_sd_rd, ldr_int_sd_rd, scsi_sd_rd[1:0]};
assign sd_wr = {dcd_sd_wr, scsi_sd_wr[SCSI_CD_DEV], wr_ext_sd_wr, wr_int_sd_wr, scsi_sd_wr[1:0]};

// sd_lba is likewise shared per slot between the loader (valid while it is
// busy) and the writer (valid the rest of the time) - the writer itself
// never starts while loader_busy is asserted (see floppy_sd_writer.v), so
// this mux can never straddle a genuine simultaneous request.
wire [31:0] ldr_int_sd_lba, ldr_ext_sd_lba;
wire [31:0] wr_int_sd_lba, wr_ext_sd_lba;
assign sd_lba[2] = ldr_int_busy ? ldr_int_sd_lba : wr_int_sd_lba;
assign sd_lba[3] = ldr_ext_busy ? ldr_ext_sd_lba : wr_ext_sd_lba;

wire [15:0] wr_int_sd_buff_din, wr_ext_sd_buff_din;
assign sd_buff_din[2] = wr_int_sd_buff_din;
assign sd_buff_din[3] = wr_ext_sd_buff_din;

// wr_*_busy: queued-or-in-flight sd_wr against this slot (see
// floppy_sd_writer.v) - folded into LED_USER below alongside the loader's
// own busy, so the activity light also covers a pending SD flush after a
// write, not just a mount-time load.
wire wr_int_busy, wr_ext_busy;

wire        ldr_int_done, ldr_ext_done;
wire        ldr_int_busy, ldr_ext_busy;
wire [63:0] ldr_int_size, ldr_ext_size;
wire        ldr_int_readonly, ldr_ext_readonly;
wire        ldr_int_media_ds, ldr_ext_media_ds;

floppy_loader ldr_int
(
	.clk_sys(clk_sys),
	.reset(!pll_locked), // power-up only - a Mac "Reset & Apply" must not abort an in-flight mount

	.img_mounted(img_mounted[2]),
	.img_size(img_size),
	.img_readonly(img_readonly),

	.sd_lba(ldr_int_sd_lba),
	.sd_rd(ldr_int_sd_rd),
	.sd_ack(sd_ack[2]),

	.sd_buff_addr(sd_buff_addr[7:0]),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_wr(sd_buff_wr),

	.wr_addr(ldr_int_wr_addr),
	.wr_data(ldr_int_wr_data),
	.wr_req(ldr_int_wr_req),
	.wr_ack(ldr_int_wr_ack),

	.done(ldr_int_done),
	.loaded_size(ldr_int_size),
	.readonly_latched(ldr_int_readonly),
	.media_ds(ldr_int_media_ds),
	.busy(ldr_int_busy)
);

floppy_loader ldr_ext
(
	.clk_sys(clk_sys),
	.reset(!pll_locked),

	.img_mounted(img_mounted[3]),
	.img_size(img_size),
	.img_readonly(img_readonly),

	.sd_lba(ldr_ext_sd_lba),
	.sd_rd(ldr_ext_sd_rd),
	.sd_ack(sd_ack[3]),

	.sd_buff_addr(sd_buff_addr[7:0]),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_wr(sd_buff_wr),

	.wr_addr(ldr_ext_wr_addr),
	.wr_data(ldr_ext_wr_data),
	.wr_req(ldr_ext_wr_req),
	.wr_ack(ldr_ext_wr_ack),

	.done(ldr_ext_done),
	.loaded_size(ldr_ext_size),
	.readonly_latched(ldr_ext_readonly),
	.media_ds(ldr_ext_media_ds),
	.busy(ldr_ext_busy)
);

floppy_sd_writer wr_int
(
	.clk(clk_sys),
	.reset(!pll_locked),

	.img_mounted(img_mounted[2]),

	.commit_done(wc_int_commit_done),
	.commit_addr(wc_int_commit_addr),
	.commit_buf_wr(wc_int_commit_buf_wr),
	.commit_buf_addr(wc_int_commit_buf_addr),
	.commit_buf_data(wc_int_commit_buf_data),

	.readonly(ldr_int_readonly),
	.loader_busy(ldr_int_busy),
	// image length in 512-byte blocks; only 400K/800K images ever reach
	// insertDisk (see dsk_int_ss/ds below), so 13 bits covers every case
	// that can produce a commit - 1600 blocks for an 800K image.
	.size_blocks(ldr_int_size[21:9]),

	.sd_lba(wr_int_sd_lba),
	.sd_wr(wr_int_sd_wr),
	.sd_ack(sd_ack[2]),

	.sd_buff_addr(sd_buff_addr[7:0]),
	.sd_buff_din(wr_int_sd_buff_din),

	.busy(wr_int_busy)
);

floppy_sd_writer wr_ext
(
	.clk(clk_sys),
	.reset(!pll_locked),

	.img_mounted(img_mounted[3]),

	.commit_done(wc_ext_commit_done),
	.commit_addr(wc_ext_commit_addr),
	.commit_buf_wr(wc_ext_commit_buf_wr),
	.commit_buf_addr(wc_ext_commit_buf_addr),
	.commit_buf_data(wc_ext_commit_buf_data),

	.readonly(ldr_ext_readonly),
	.loader_busy(ldr_ext_busy),
	.size_blocks(ldr_ext_size[21:9]),

	.sd_lba(wr_ext_sd_lba),
	.sd_wr(wr_ext_sd_wr),
	.sd_ack(sd_ack[3]),

	.sd_buff_addr(sd_buff_addr[7:0]),
	.sd_buff_din(wr_ext_sd_buff_din),

	.busy(wr_ext_busy)
);

// word written into SDRAM this cycle when dskLoadWrEn is high - selects
// whichever side (int/ext) addrController_top's arbiter (fixed priority
// int-over-ext) actually granted this cycle, and within that side,
// whichever source (loader/committer) slot3_int_req/slot3_ext_req above
// selected. Must use dskLoadSelExt (held for the whole grant cycle), not
// ldr_ext_wr_ack/dskLoadAckExt - that ack is a late pulse in busPhase 3,
// one phase after sdram.v's CAS phase (busPhase 1) already latched this
// data, so gating on it left every ext (drive 2) write writing int's
// stale data instead of its own.
wire [15:0] slot3_wr_data = dskLoadSelExt ? slot3_ext_data : slot3_int_data;

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

// word address within the ROM region: slot number, then word offset
wire [20:0] dio_rom_addr  = {1'b0, dio_index[7:6], dio_addr[17:0]};
wire [20:0] rom_read_addr = {1'b0, romSlot,        memoryAddr[18:1]};

// good floppy image sizes are 819200 bytes and 409600 bytes; both mount on
// every model. These flags say only how big the file is: floppy.v's
// doubleSidedDisk decides the geometry.
reg dsk_int_ds, dsk_ext_ds;  // 819,200-byte image inserted
reg dsk_int_ss, dsk_ext_ss;  // 409,600-byte image inserted

// any known type of disk image inserted?
wire dsk_int_ins = dsk_int_ds || dsk_int_ss;
wire dsk_ext_ins = dsk_ext_ds || dsk_ext_ss;

// Floppies are S-type block-device mounts, loaded into SDRAM
// by floppy_loader (see instantiation above) instead of streamed in via
// ioctl_download. insertDisk therefore only goes true once ldr_*_done
// fires - i.e. once the WHOLE image is resident in SDRAM - never at the
// bare img_mounted pulse, so the Mac can never observe a partially-loaded
// disk. Also clear-on-mount (not just on eject/size-mismatch): a remount
// while already inserted must drop insertDisk immediately so nothing reads
// mid-reload, mirroring the SAVE-feature precedent in the UK101 core.
// diskEject is still set by macOS on eject, unchanged.
always @(posedge clk_sys) begin
	if (img_mounted[2] && img_size != 0) begin
		dsk_int_ds <= 1'b0;
		dsk_int_ss <= 1'b0;
	end
	else if (ldr_int_done) begin
		dsk_int_ds <= (ldr_int_size == 64'd819200);
		dsk_int_ss <= (ldr_int_size == 64'd409600);
	end

	if(diskEject[0]) begin
		dsk_int_ds <= 0;
		dsk_int_ss <= 0;
	end
end	

always @(posedge clk_sys) begin
	if (img_mounted[3] && img_size != 0) begin
		dsk_ext_ds <= 1'b0;
		dsk_ext_ss <= 1'b0;
	end
	else if (ldr_ext_done) begin
		dsk_ext_ds <= (ldr_ext_size == 64'd819200);
		dsk_ext_ss <= (ldr_ext_size == 64'd409600);
	end

	if(diskEject[1]) begin
		dsk_ext_ds <= 0;
		dsk_ext_ss <= 0;
	end
end

// ROM slots are 512KB apart in the ROM region (rtl/sdram_map.vh); each image
// starts at its slot's offset 0. Floppy images take the floppy_loader path
// above.
reg [20:0] dio_a;
reg [15:0] dio_data;
reg        dio_write;

always @(posedge clk_sys) begin
	reg old_cyc = 0;

	if(ioctl_write) begin
		dio_data <= {ioctl_data[7:0], ioctl_data[15:8]};
		dio_a <= dio_rom_addr;
		ioctl_wait <= 1;
	end

	old_cyc <= dioBusControl;
	if(~dioBusControl) dio_write <= ioctl_wait;
	if(old_cyc & ~dioBusControl & dio_write) ioctl_wait <= 0;
end


// sdram used for ram/rom maps directly into 68k address space
wire download_cycle = dio_download && dioBusControl;

////////////////////////// SDRAM /////////////////////////////////

// Region bases from rtl/sdram_map.vh, added rather than concatenated; exact
// because each base's low bits are zero below its payload.
wire dsk_cycle = dskReadAckInt || dskReadAckExt || dskLoadWrEn;

wire [24:0] sdram_addr = download_cycle ? (`SDRAM_ROM_BASE  + dio_a[20:0])       :
                         ~_romOE        ? (`SDRAM_ROM_BASE  + rom_read_addr)     :
                         dsk_cycle      ? (`SDRAM_DISK_BASE + memoryAddr[21:1])  :
                                          (`SDRAM_RAM_BASE  + memoryAddr[21:1]);

wire [15:0] sdram_din  = download_cycle ? dio_data  : dskLoadWrEn ? slot3_wr_data : memoryDataOut;
wire  [1:0] sdram_ds   = download_cycle ? 2'b11     : dskLoadWrEn ? 2'b11          : { !_memoryUDS, !_memoryLDS };
wire        sdram_we   = download_cycle ? dio_write : dskLoadWrEn ? 1'b1           : !_ramWE;
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
	// Region layout is rtl/sdram_map.vh; boot ROMs live at word $400000.
	.din            ( sdram_din                ),
	.addr           ( sdram_addr               ),
	.ds             ( sdram_ds                 ),
	.we             ( sdram_we                 ),
	.oe             ( sdram_oe                 ),
	.dout           ( sdram_out                )
);

endmodule
