/* verilator lint_off UNUSED */

// scsi.v
// implements a target only scsi device
  
module scsi
(
	input      clk,

	// scsi interface
	input 	  rst, // bus reset from initiator (ICR RST)
	input 	  sys_rst, // system/core reset, independent of the bus reset
	input 	  bus_busy, // another target on the bus currently holds BSY
	// CDROM targets only: drive present on the bus. A real AppleCD answers
	// selection with no disc inserted (the driver polls TEST UNIT READY to
	// detect insertion), so the CD target selects on this rather than on
	// `mounted`. Tied low it never answers at all, which makes the bus
	// bit-identical to a pre-CD build -- the period-purist switch and the A/B
	// lever if the new target misbehaves on hardware. Ignored when CDROM == 0.
	input 	  cd_enable,
	// CD-ROM command-set debug ladder (CDROM targets only). 0 = normal. 1..6
	// progressively re-enable commands so the OSD can bisect which one the
	// driver chokes on WITHOUT a rebuild; 7 = everything, same as 0. Every
	// command outside the enabled set answers CHECK CONDITION, which is a
	// legitimate SCSI response, so the bus stays healthy at every level.
	// MODE SENSE content bisect (CDROM targets only). Hardware proved the
	// MECHANISM of answering 0x1a is fine -- state 1 boots with every command
	// enabled -- so the fault is in our response BYTES. These states add one
	// component at a time, all at the SAME transfer length so only content
	// varies, and every one is a response a real drive may legitimately give:
	//   0 full        1 header only    2 +block descriptor  3 +page shell
	//   4 -p30 body   5 -p0E body      6 -p2A body          7 full
	// Hardware walked 1, 2 and 3 green and 0 hangs, so the block descriptor
	// and the page code/length byte are exonerated and the fault is in a page
	// PAYLOAD (bytes 14+). States 4-6 suppress exactly one page's payload, so
	// the state that boots names the page.
	input 	  sel,
	input 	  atn, // initiator requests to send a message
	output 	  bsy, // target holds bus

	output 	  msg,
	output 	  cd,
	output 	  io,

	output 	  req,
	input 	  ack, // initiator acknowledges a request

	input   [7:0] din, // data from initiator to target
	output  [7:0] dout, // data from target to initiator

	// interface to io controller
	input         img_mounted,
	input  [31:0] img_blocks,
	output [31:0] io_lba,
	output        io_rd,
	output reg 	  io_wr,
	input         io_ack,

	input   [7:0] sd_buff_addr,
	input   [4:0] sd_buff_addr_hi, // hps_io addr[12:8] (CD whole-frame bursts).
	                               // Unused until the cd_audio engine lands; a
	                               // 2352-byte frame needs 11 address bits and
	                               // this port carries the 3 above our 8.
	input  [15:0] sd_buff_dout,
	output [15:0] sd_buff_din,
	input         sd_buff_wr,

	// CPU BUS HOLD-OFF. High while this target is in a data phase and physically
	// cannot serve or accept the next byte -- the same condition that withdraws
	// REQ. ncr5380.sv turns it into a withheld DTACK on the pseudo-DMA window, so
	// a blind pump STALLS instead of transferring a stale byte. Deliberately
	// scoped to the DATA phases: a DACK access outside them already cannot ACK
	// (dma_ack is gated on the bus phase), and stalling the driver's status poll
	// on it would be a hang, not an interlock. See also the frontier-breach
	// detector below.
	output        data_holdoff,

	// CD-DA sample pair from the audio engine. EXACT zeros whenever the
	// drive is not playing, so a build with no disc mixes bit-identically
	// to one without the engine. Constant 0 on a disk target.
	output signed [15:0] cd_snd_l,
	output signed [15:0] cd_snd_r
);

// SCSI device id
parameter [2:0] ID = 0;

// CD-ROM personality. 0 = direct-access disk (unchanged in every respect).
// 1 = AppleCD-compatible CD-ROM: SONY CDU-8004 identity, 2048-byte logical
// blocks served as 4 consecutive 512-byte HPS blocks (lba/tlen <<2 at latch
// time, so the whole ring/flush machinery below runs in 512-byte units without
// modification), READ TOC, sub-channel, eject, and no-disc sense.
//
// Every cd_* wire folds to a constant when CDROM == 0, so a disk target
// synthesizes to exactly what it did before this parameter existed.
//
// TOC serving comes from the CD audio engine (rtl/cd_audio.sv), which parses
// the real track list out of the blob Main hands over on mount, so multi-track
// and audio discs report honestly. A flat data image still reads as the single
// data track spanning the disc that it is.
parameter CDROM = 0;

// Read-prefetch ring depth (number of 512-byte sectors held for reads). A real
// drive streams continuously off a spinning platter; the original two-sector
// double buffer stalled at every 512-byte boundary while the next block was
// fetched from the HPS. The ring keeps RING_BLOCKS sectors fetched AHEAD of the
// Mac so that latency is hidden. RING_LOG=1 reproduces the old double buffer
// exactly. WRITES are unchanged -- they stay on the two-slot buffer (slots 0/1).
// Ported from MacLC_MiSTer rtl/scsi.v, which uses the same value; we have ~475
// M10K free, so the depth is not fit-constrained.
parameter  RING_LOG    = 5;             // log2(sectors); 5 => 32 sectors / 16KB
localparam RING_BLOCKS = 1 << RING_LOG; // sectors buffered for reads
localparam BUF_AW      = 8 + RING_LOG;  // dpram word-address width (256 words/sector)

// A core reset must tear the target down as thoroughly as a bus reset: without
// it a reset landing mid-command leaves this target holding BSY (the phase FSM
// is not otherwise cleared) and the ROM's next boot scan finds a busy bus.
wire any_rst = rst | sys_rst;

localparam PHASE_IDLE        = 3'd0;
localparam PHASE_CMD_IN      = 3'd1;
localparam PHASE_DATA_OUT    = 3'd2;
localparam PHASE_DATA_IN     = 3'd3;
localparam PHASE_STATUS_OUT  = 3'd4;
localparam PHASE_MESSAGE_OUT = 3'd5;
reg [2:0]  phase;

// ------------ sector buffer IO controller read/write -----------------------
// the buffer itself. Holds RING_BLOCKS sectors for reads; writes use slots 0/1.
reg sd_buff_sel;         // WRITE double-buffer half (unchanged path)
reg [22:0] rd_hps_blk;   // READ ring: # of sectors the HPS has delivered this command

// HPS sector-buffer byte order. buffer0 always holds the byte the Mac reads
// FIRST (even byte) and buffer1 the odd byte. The real MiSTer HPS packs WIDE
// words LITTLE-endian (disk byte0 -> sd_buff_dout[7:0]), which is what the lane
// mapping below assumes. The LC carries a second `ifdef VERILATOR mapping
// for its own
// bench, which packs big-endian; we have no such bench, so there is nothing to
// switch on and the lanes stay as they always were.

// Buffer addressing. READS span the whole RING_BLOCKS-sector ring; WRITES stay
// on the original two-slot double buffer so the freshly-validated write path is
// byte-for-byte unchanged. A command is either a read or a write, so the two
// schemes never collide on a port. The two-slot addresses are zero-extended to
// BUF_AW by assignment, so RING_LOG=1 still compiles and exactly reproduces the
// original double buffer.
wire [22:0] rd_cur_blk = data_cnt[31:9];                       // sector the Mac is reading
wire [RING_LOG-1:0] rd_hps_slot = rd_hps_blk[RING_LOG-1:0];
wire [BUF_AW-1:0] hps_addr_wr = {sd_buff_sel, sd_buff_addr};   // write flush: slot 0/1
wire [BUF_AW-1:0] mac_addr_wr = data_cnt[9:1];                 // Mac write: slot 0/1
// HPS side (port A): read fills target the ring fetch-slot; write flushes keep
// the original sd_buff_sel half.
wire [BUF_AW-1:0] hps_addr = cmd_write ? hps_addr_wr : {rd_hps_slot, sd_buff_addr};
// Mac side (port B): reads address the full ring; writes the 2-slot half.
wire [BUF_AW-1:0] mac_addr = (phase == PHASE_DATA_IN) ? mac_addr_wr : data_cnt[BUF_AW:1];

wire [7:0] buffer0_dout;
scsi_dpram #(.ADDRWIDTH(BUF_AW)) buffer0
(
	.clock(clk),

	.address_a(hps_addr),
	.data_a(sd_buff_dout[7:0]),
	.wren_a(sd_buff_wr && !ca_io_active),
	.q_a(sd_buff_din[7:0]),

	.address_b(mac_addr),
	.data_b(din),
	.wren_b(buffer0_wr),
	.q_b(buffer0_dout)
);

wire [7:0] buffer1_dout;
scsi_dpram #(.ADDRWIDTH(BUF_AW)) buffer1
(
	.clock(clk),

	.address_a(hps_addr),
	.data_a(sd_buff_dout[15:8]),
	.wren_a(sd_buff_wr && !ca_io_active),
	.q_a(sd_buff_din[15:8]),

	.address_b(mac_addr),
	.data_b(din),
	.wren_b(buffer1_wr),
	.q_b(buffer1_dout)
);

reg old_io_ack;
always @(posedge clk) begin
	old_io_ack <= io_ack;
	if (phase == PHASE_IDLE)
		sd_buff_sel <= 0;
	else
		// ~ca_io_active: a CD-audio channel transfer started at bus-idle can
		// still be in flight when the Mac's next command reaches a data phase.
		// Its ack falling here would toggle the write double-buffer and bump
		// the ring counter below - wrong sectors served. MacLC hit exactly
		// this on hardware (2026-07-17: artifacted CD icons, then a wedged
		// READ). Same scope the io_busy term already has.
		if (old_io_ack & ~io_ack & ~ca_io_active) sd_buff_sel <= !sd_buff_sel;

	// READ ring fetch counter: # of sectors the HPS has delivered this command.
	// Reset alongside data_cnt (any non-transfer phase); bump on each io_ack
	// falling edge during a read. Writes never touch it (they use sd_buff_sel).
	if (phase != PHASE_DATA_OUT && phase != PHASE_DATA_IN &&
	    phase != PHASE_STATUS_OUT && phase != PHASE_MESSAGE_OUT)
		rd_hps_blk <= 23'd0;
	else if (old_io_ack & ~io_ack & cmd_read & ~ca_io_active)
		rd_hps_blk <= rd_hps_blk + 23'd1;
end

// -----------------------------------------------------------

// status replies
reg [7:0]  status;
`define STATUS_OK 8'h00
`define STATUS_CHECK_CONDITION 8'h02

// message codes
`define MSG_CMD_COMPLETE 8'h00
	
// drive scsi signals according to phase
assign msg = (phase == PHASE_MESSAGE_OUT);
assign cd = (phase == PHASE_CMD_IN) || (phase == PHASE_STATUS_OUT) || (phase == PHASE_MESSAGE_OUT);
assign io = (phase == PHASE_DATA_OUT) || (phase == PHASE_STATUS_OUT) || (phase == PHASE_MESSAGE_OUT);

// READ stall: for a block READ, the sector the Mac wants (rd_cur_blk) has not
// been fetched yet -- only sectors [0, rd_hps_blk) are in the ring. Gated on
// cmd_read because INQUIRY / READ CAPACITY / MODE SENSE / REQUEST SENSE also use
// DATA_OUT but serve data combinationally with no HPS fetch (rd_hps_blk stays
// 0), so they must NOT take this stall. Depth-independent; replaces the old
// two-slot "half being filled" test.
//
// This also closes the earlier "req asserts ~2 cycles before io_rd on
// entering DATA_OUT": at data_cnt=0 both counters are 0, so rd_cur_blk >=
// rd_hps_blk holds and REQ is suppressed until the first sector has actually
// landed -- rather than depending on io_rd having had time to rise.
//
// `mounted` in the read clause: media loss mid-READ stops the ring refill, and
// holding the CPU on data that will never arrive wedges the guest. With the
// medium gone the read completes with stale bytes and the driver gets its error
// through the normal status path instead (MacLC finding, HW 2026-07-17).
//
// wr_pending is included in the write/non-data clauses: between a block's
// req_wr edge and the flush actually issuing, neither io_wr nor io_ack is high,
// so the old term dropped the busy indication for that window and one extra
// byte could land in the slot the flush had not read yet.
wire   rd_cur_unfilled = (rd_cur_blk >= rd_hps_blk);
wire   io_busy = (phase == PHASE_DATA_OUT && cmd_read && mounted && rd_cur_unfilled) ||
                 (phase == PHASE_DATA_IN  && (io_wr | wr_pending | (io_ack & ~ca_io_active)) && data_cnt[9] == sd_buff_sel) ||
                 (phase != PHASE_DATA_OUT && phase != PHASE_DATA_IN && (io_rd_d | io_wr | wr_pending | (io_ack & ~ca_io_active)));

// A zero-length data phase (allocation length 0) never sees an ACK edge, so
// data_complete -- which only sets on one -- would never assert and REQ would be
// held forever. Treat "no data expected" as done on entry.
wire   data_done = data_complete || (data_len == 32'd0);
wire   data_phase_complete = ((phase == PHASE_DATA_OUT) || (phase == PHASE_DATA_IN)) && data_done;

assign req = (phase != PHASE_IDLE) && !ack && !io_busy && !data_phase_complete;

// The hold-off is io_busy's two DATA-phase clauses and nothing else. io_busy's
// third clause covers the non-data phases, where a DACK access is already inert.
// Explicitly zero under reset. Everywhere else in this file an undefined
// `phase` is harmless -- it settles before anything reads it -- but this signal
// reaches _cpuDTACK, where a spurious hold at power-on would freeze the CPU
// with nothing to release it. Cyclone V registers do power up at 0
// (= PHASE_IDLE), so this is belt and braces; on the CPU bus that is cheap.
assign data_holdoff = !any_rst && ((phase == PHASE_DATA_OUT) || (phase == PHASE_DATA_IN)) && io_busy;

assign bsy = (phase != PHASE_IDLE);

assign dout = (phase == PHASE_STATUS_OUT)?status:
	 (phase == PHASE_MESSAGE_OUT)?`MSG_CMD_COMPLETE:
	 (phase == PHASE_DATA_OUT)?cmd_dout:
	 8'h00;

// de-multiplex different data sources
wire [7:0] cmd_dout =
		cmd_read?(data_cnt[0] ? buffer1_dout : buffer0_dout):
		cmd_inquiry?inquiry_dout:
		cmd_read_capacity?read_capacity_dout:
		cmd_mode_sense?mode_sense_dout:
		cmd_request_sense?request_sense_dout:
		cmd_cd_toc?cd_toc_dout:
		(cmd_cd_t43f2 || cmd_cd_t43f1)?cd_toc2_dout:
		cmd_cd_toc43?cd_toc43_dout:
		cmd_cd_subq?cd_subq_dout:
		cmd_cd_subq43?cd_subq43_dout:
		cmd_cd_astat?cd_astat_dout:
		cmd_cd_hdr?cd_hdr_byte(data_cnt, cd_hdr_addr_r):
		8'h00;

// REQUEST SENSE (0x03) response: fixed-format sense data, 18 bytes.
//   byte 0  = 0x70  current error, no valid information field
//   byte 2  = sense key
//   byte 7  = 0x0a  additional sense length (10 => 18 total)
//   byte 12 = additional sense code (ASC)
// Unlike the LC -- whose disk path serves a static all-zeros NO SENSE block and
// keeps real keys for the CD target only -- this reports the actual reason the
// last command failed. Answering "NO SENSE" to the question "why did you CHECK?"
// is self-contradictory and gives a driver's retry logic nothing to act on. On
// the disk path there is exactly one error class, so the machinery is a register
// pair rather than the LC's CD state machine (see the sense latch below).
wire [7:0] request_sense_dout =
		(data_cnt == 32'd0 )?8'h70:
		(data_cnt == 32'd2 )?{4'd0, sense_key}:
		(data_cnt == 32'd7 )?8'h0a:
		(data_cnt == 32'd12)?sense_asc:
		8'h00;

// CDROM INQUIRY: a SONY CDU-8004, byte-exact from MAME's
// nscsi_cdrom_apple_device (data taken from an AppleCD 150 ROM). This is not a
// cosmetic vendor string -- Apple's CD-ROM extension binds only to drives it
// recognises, so the identity IS the compatibility. 5 + additional-length 0x31
// = 54-byte response.
function [7:0] cd_inquiry_byte;
	input [31:0] cnt;
	begin
		cd_inquiry_byte =
			(cnt == 32'd0 )?8'h05:  // CD-ROM device class
			(cnt == 32'd1 )?8'h80:  // removable
			(cnt == 32'd2 )?8'h02:  // ANSI SCSI-2 (dialect tier)
			(cnt == 32'd3 )?8'h02:
			(cnt == 32'd4 )?8'h31:  // additional length
			(cnt == 32'd8 )?"S":(cnt == 32'd9 )?"O":
			(cnt == 32'd10)?"N":(cnt == 32'd11)?"Y":
			((cnt >= 32'd12) && (cnt <= 32'd15))?" ":
			(cnt == 32'd16)?"C":(cnt == 32'd17)?"D":
			(cnt == 32'd18)?"-":(cnt == 32'd19)?"R":
			(cnt == 32'd20)?"O":(cnt == 32'd21)?"M":
			(cnt == 32'd22)?" ":(cnt == 32'd23)?"C":
			(cnt == 32'd24)?"D":(cnt == 32'd25)?"U":
			(cnt == 32'd26)?"-":(cnt == 32'd27)?"8":
			(cnt == 32'd28)?"0":(cnt == 32'd29)?"0":
			(cnt == 32'd30)?"4":(cnt == 32'd31)?" ":
			(cnt == 32'd32)?"1":(cnt == 32'd33)?".":
			(cnt == 32'd34)?"9":(cnt == 32'd35)?"a":
			(cnt == 32'd39)?8'hd0:(cnt == 32'd40)?8'h90:
			(cnt == 32'd41)?8'h27:(cnt == 32'd42)?8'h3e:
			(cnt == 32'd43)?8'h01:(cnt == 32'd44)?8'h04:
			(cnt == 32'd45)?8'h91:(cnt == 32'd47)?8'h18:
			(cnt == 32'd48)?8'h06:(cnt == 32'd49)?8'hf0:
			(cnt == 32'd50)?8'hfe:
			8'h00;
	end
endfunction

// output of inquiry command, identify as "SEAGATE ST225N" (disk) or the
// SONY CDU-8004 above (CD-ROM).
wire [7:0] inquiry_dout = (CDROM != 0) ? cd_inquiry_byte(data_cnt) : hd_inquiry_dout;
wire [7:0] hd_inquiry_dout =
		(data_cnt == 32'd4 )?8'd32:  // length

		(data_cnt == 32'd8 )?" ":(data_cnt == 32'd9 )?"S":
		(data_cnt == 32'd10)?"E":(data_cnt == 32'd11)?"A":
		(data_cnt == 32'd12)?"G":(data_cnt == 32'd13)?"A":
		(data_cnt == 32'd14)?"T":(data_cnt == 32'd15)?"E":
		(data_cnt == 32'd16)?" ":(data_cnt == 32'd17)?" ":
		(data_cnt == 32'd18)?" ":(data_cnt == 32'd19)?" ":
		(data_cnt == 32'd20)?" ":(data_cnt == 32'd21)?" ":
		(data_cnt == 32'd22)?" ":(data_cnt == 32'd23)?" ":
		(data_cnt == 32'd24)?" ":(data_cnt == 32'd25)?" ":

		(data_cnt == 32'd26)?"S":(data_cnt == 32'd27)?"T":
		(data_cnt == 32'd28)?"2":(data_cnt == 32'd29)?"2":
		(data_cnt == 32'd30)?"5":(data_cnt == 32'd31)?"N" + {5'd0, ID}: // TESTING. ElectronAsh.
		8'h00;

// output of read capacity command
//wire [31:0] capacity = 32'd41056;   // 40960 + 96 blocks = 20MB
//wire [31:0] capacity = 32'd1024096;   // 1024000 + 96 blocks = 500MB
// Initialised: a CDROM target answers MODE SENSE before any image has ever been
// mounted (drive present, no disc), so the capacity bytes must not be X.
reg [31:0] capacity = 32'd0;
reg        mounted = 0;
always @(posedge clk) begin
	// The CD keeps its disc across a reset, as a real AppleCD SC did. f254ffd
	// emptied the drive here so the Plus ROM could not boot-scan a mounted CD
	// and hang; that stopped the hang but it is not what the hardware did, and
	// the ROM is now headed off earlier by UNIT ATTENTION instead (see
	// cd_unit_attn below).
	//
	// A bus reset yields UNIT ATTENTION, not an eject; an external drive is not
	// power-cycled by a Mac reset; and the Plus ROM has no CD driver to command
	// an eject with even in principle.
	if (img_mounted) begin
		if (|img_blocks) begin
			// capacity is the LAST LBA, on both personalities: READ CAPACITY is
			// defined to return the address of the last logical block, not the
			// block count. CDROM counts in 2048-byte logical blocks, i.e. the
			// mounted 512-block count / 4 - 1.
			//
			// The disk path used to report img_blocks, advertising one block
			// MORE than the medium has. That was a knowingly deferred
			// follow-up, left alone on the grounds that changing it would alter
			// what every existing driver sees. It does -- but the extra block
			// was never actually usable, so no existing volume can hold data
			// there.
			//
			// MEASURED 2026-08-24, and NOT by the mechanism this comment used
			// to claim. It said such an access "stalled the bus" via an HPS
			// that could not service it. It does not: on the pre-fix build
			// 432955e3, HD SC Setup's Test Disk on a 40,960-block image FAILS
			// with "Problems writing data to disk" and the Mac carries on
			// working normally. The driver was told block 40,960 exists, wrote
			// to it, and got an error back -- no stall, no wedge. The
			// conclusion stands (the block is not usable); the mechanism was
			// wrong. The same test passes on this build. Reads past the end
			// were not separately characterised, so this says nothing about
			// them.
			capacity <= (CDROM != 0) ? ({2'b00, img_blocks[31:2]} - 1'd1)
			                         : (img_blocks - 1'd1);
			if (!mounted) $display("Image mounted on target %d, size: %d", ID, img_blocks);
			mounted <= 1;
		end else
			mounted <= 0;
	end else if ((CDROM != 0) && cd_eject_pulse)
		// EJECT (Apple 0xC0, or standard 0x1B START/STOP with LoEj): drop the
		// medium. The next img_mounted pulse is the "disc inserted" edge the
		// AppleCD driver's insertion poll is waiting for. The HPS-side image
		// stays mounted, which is harmless -- the target simply reports no-disc
		// until the user mounts again.
		mounted <= 0;
end

wire [7:0] read_capacity_dout =
		(data_cnt == 32'd0 )?capacity[31:24]:
		(data_cnt == 32'd1 )?capacity[23:16]:
		(data_cnt == 32'd2 )?capacity[15:8]:
		(data_cnt == 32'd3 )?capacity[7:0]:
		(data_cnt == 32'd6 )?((CDROM != 0)?8'h08:8'd2): // block length 2048 (CD) / 512 (disk)
		8'h00;

// CDROM MODE SENSE(6): header + 8-byte block descriptor (12 bytes), with
// device-specific byte 0x80 (write-protected -- the medium is read-only) and
// block length 0x000800 = 2048. A page 0x30 request additionally appends the
// 24-byte "magic Apple page" (0x30, 0x00, "APPLE COMPUTER, INC   "), byte-exact
// from MAME's apple_magic, which some Apple drivers probe even on CD drives.
// NOTE on the argument lists here and below: every module signal a serving
// function depends on is passed IN, rather than read from the function body.
// A continuous assignment that calls a function gets its sensitivity from the
// call's arguments, so a signal read only inside the body does not retrigger
// it -- the response then carries the PREVIOUS command's decode until some
// other argument changes. That showed up as byte 0 (and only byte 0) of every
// CD response being stale, because data_cnt incrementing is what re-evaluated
// the assignment. Passing them in is correct by construction.
// Page 0x0E (CD Audio Control) and 0x2A (MM Capabilities) are NOT optional:
// the AppleCD driver asks for 0x0E directly during startup. Serving the bare
// 12-byte header for a page the driver armed a longer blind transfer for leaves
// the host waiting for bytes that never come -- BERR beats, SCSI Manager retry,
// boot wedge. That is the hang seen on hardware 2026-08-20 with the drive
// enabled. Lengths and payloads follow MacLC_MiSTer, which is known to work
// with this driver.
function [7:0] cd_mode_sense_byte;
	input [31:0] cnt;
	input [5:0]  page;
	input [31:0] cap;
	begin
		cd_mode_sense_byte =
			// ---- common header + block descriptor (bytes 0..11)
			(cnt == 32'd0 )?((page == 6'h30) ? 8'd35 :   // mode data length = total-1
			                 (page == 6'h0E) ? 8'd27 :
			                 (page == 6'h2A) ? 8'd37 : 8'd11):
			(cnt == 32'd2 )?8'h80:                      // WP (read-only medium)
			(cnt == 32'd3 )?8'd8:                       // block descriptor length
			(cnt == 32'd5 )?cap[23:16]:
			(cnt == 32'd6 )?cap[15:8]:
			(cnt == 32'd7 )?cap[7:0]:
			(cnt == 32'd10)?8'h08:                      // block length 0x000800 = 2048
			// ---- page 0x30: the "magic Apple page" (24 bytes, total 36)
			(page == 6'h30)?(
			   (cnt == 32'd12)?8'h30:
			   (cnt == 32'd14)?"A":(cnt == 32'd15)?"P":
			   (cnt == 32'd16)?"P":(cnt == 32'd17)?"L":
			   (cnt == 32'd18)?"E":(cnt == 32'd19)?" ":
			   (cnt == 32'd20)?"C":(cnt == 32'd21)?"O":
			   (cnt == 32'd22)?"M":(cnt == 32'd23)?"P":
			   (cnt == 32'd24)?"U":(cnt == 32'd25)?"T":
			   (cnt == 32'd26)?"E":(cnt == 32'd27)?"R":
			   (cnt == 32'd28)?",":(cnt == 32'd29)?" ":
			   (cnt == 32'd30)?"I":(cnt == 32'd31)?"N":
			   (cnt == 32'd32)?"C":
			   ((cnt >= 32'd33) && (cnt <= 32'd35))?" ":8'h00):
			// ---- page 0x0E: CD Audio Control (16 bytes, total 28)
			// Port/volume bytes are static here; the audio engine makes
			// them the live MODE SELECT-writable state.
			(page == 6'h0E)?(
			   (cnt == 32'd12)?8'h0E:                   // page code
			   (cnt == 32'd13)?8'h0E:                   // page length = 14
			   (cnt == 32'd14)?8'h04:                   // IMMED=1, SOTC=0
			   (cnt == 32'd18)?8'd75:
			   (cnt == 32'd19)?8'd75:
			   (cnt == 32'd20)?8'h01:(cnt == 32'd21)?8'hff:  // port 0 -> ch1, full
			   (cnt == 32'd22)?8'h02:(cnt == 32'd23)?8'hff:  // port 1 -> ch2, full
			   8'h00):
			// ---- page 0x2A: MM Capabilities & Mechanical Status (26 B, total 38)
			(page == 6'h2A)?(
			   (cnt == 32'd12)?8'h2A:                   // page code
			   (cnt == 32'd13)?8'h18:                   // page length = 24
			   (cnt == 32'd16)?8'h71:  // multi-session | Mode 2 F2 | F1 | audio
			   (cnt == 32'd18)?8'h28:  // tray loading | eject
			   (cnt == 32'd19)?8'h03:  // separate channel mute | volume levels
			   (cnt == 32'd22)?8'h01:  // 256 volume levels
			   8'h00):
			8'h00;
	end
endfunction

wire [7:0] mode_sense_dout =
		(CDROM == 0) ? hd_mode_sense_dout :
		cd_mode_sense_byte(data_cnt, cd_page_r, capacity);
wire [7:0] hd_mode_sense_dout =
		(data_cnt == 32'd3 )?8'd8:
		(data_cnt == 32'd5 )?capacity[23:16]:
		(data_cnt == 32'd6 )?capacity[15:8]:
		(data_cnt == 32'd7 )?capacity[7:0]:
		(data_cnt == 32'd10 )?8'd2:
		8'h00;

// =====================================================================
// CD-ROM response synthesis. Every wire here folds to a constant when
// CDROM == 0, so a disk target is unaffected.
// =====================================================================

// Decoded CDB parameters, LATCHED at command completion.
//
// Serving functions must not read the cmd[] array combinationally. This module
// already establishes that pattern for the disk path -- lba6/tlen6 are decoded
// from cmd[] once at cmd_cpl and latched into lba/tlen, and everything
// downstream reads the registers. The CD serve paths follow it for the same two
// reasons: it keeps a wide combinational cone off a memory's outputs, and a
// continuous assignment that reads an array element is not reliably re-evaluated
// when that element changes (Icarus does not retrigger it, which showed up as
// byte 0 of every CD response carrying the PREVIOUS command's decode while
// bytes 1+ were correct -- the data_cnt increment was what retriggered it).
reg [1:0]  c1_op_r        = 2'b00;   // Apple READ TOC operation, CDB[9][7:6]
reg [7:0]  c1_trk_r       = 8'd0;    // Apple READ TOC track (BCD), CDB[5]
reg [7:0]  t43_start_r    = 8'd0;    // standard READ TOC start track, CDB[6]
reg [7:0]  t43_fmt_r      = 8'd0;    // standard READ TOC format byte, CDB[9]
reg [5:0]  cd_page_r      = 6'd0;    // MODE SENSE page code, CDB[2][5:0]
reg        cd_astat_vol_r = 1'b0;    // AUDIO STATUS asked for volumes, CDB[3]==1
reg [31:0] cd_alloc10_r   = 32'd0;   // raw 10-byte-CDB allocation, CDB[7:8]
reg [31:0] cd_hdr_addr_r  = 32'd0;   // READ HEADER address echo, CDB[2:5]

// ---- sub-channel ---------------------------------------------------------
// LIVE from the engine. These used to be constants, and that was NOT the
// cosmetic limitation it looked like.
//
// HARDWARE 2026-08-26, build 6E138B82: pressing Play made the track counter go
// 1 -> 2 -> back to 1, with the engine visibly starting a fetch each time and
// being cut short. The capture named the cause -- a repeated
// 16-byte data-in command (0x42 READ SUB-CHANNEL format 1, the player polling
// its display) and we answered, every single time:
//
//     audio status 0x13  =  "play operation completed"
//
// So the player issued PLAY, asked what was happening, was told the play had
// finished, stepped to the next track, and wrapped. **Reporting a stale
// "stopped" does not degrade playback, it PREVENTS it.** 3B+ is a prerequisite
// for audio working at all, not a polish step; it was scheduled after 3C/3D on
// the strength of a wrong guess about what these bytes are for.
//
// Layouts follow MacLC scsi.v:853-923, which is known to work with this driver.
// Two asymmetries in it are deliberate and easy to "tidy" into bugs:
//
//   * the STANDARD 0x42 plane reports mapped status (0x11 play / 0x12 paused /
//     0x13 completed) and BINARY M/S/F; the APPLE 0xC2 and 0xCC planes report
//     the engine's RAW ast_code and BCD M/S/F. Same facts, two dialects.
//   * 0xC2 byte 0 is 0x00, not the current control nibble, even though the
//     field is nominally ADR/control. That is MacLC's shipped behaviour against
//     the real AppleCD driver, so it is inherited rather than corrected.
localparam [7:0] CD_AST_STOPPED = 8'h13;

// Vendor-dialect BCD for the Apple planes. NOT cd_audio's bin2bcd, which had
// the 12-bit-concat truncation bug fixed in 6e138b8 -- this one builds the
// nibbles explicitly so the same mistake cannot recur.
function [7:0] cd_bin2bcd;             // 0..99
	input [7:0] v;
	reg [3:0] tens;
	reg [3:0] units;
	begin
		tens  = (v >= 8'd90) ? 4'd9 : (v >= 8'd80) ? 4'd8 :
		        (v >= 8'd70) ? 4'd7 : (v >= 8'd60) ? 4'd6 :
		        (v >= 8'd50) ? 4'd5 : (v >= 8'd40) ? 4'd4 :
		        (v >= 8'd30) ? 4'd3 : (v >= 8'd20) ? 4'd2 :
		        (v >= 8'd10) ? 4'd1 : 4'd0;
		units = v - ({4'd0, tens} * 8'd10);
		cd_bin2bcd = {tens, units};
	end
endfunction

// Apple READ Q SUBCODE (0xC2), 9 bytes:
// {control, track BCD, index, rel M/S/F, abs M/S/F}
function [7:0] cd_subq_byte;
	input [31:0] cnt;
	input [7:0]  trk;
	input [7:0]  rm, rs, rf;
	input [7:0]  am, as_, af;
	begin
		cd_subq_byte = (cnt == 32'd1) ? cd_bin2bcd(trk) :
		               (cnt == 32'd2) ? 8'h01 :          // index 1
		               (cnt == 32'd3) ? cd_bin2bcd(rm) :
		               (cnt == 32'd4) ? cd_bin2bcd(rs) :
		               (cnt == 32'd5) ? cd_bin2bcd(rf) :
		               (cnt == 32'd6) ? cd_bin2bcd(am) :
		               (cnt == 32'd7) ? cd_bin2bcd(as_) :
		               (cnt == 32'd8) ? cd_bin2bcd(af) : 8'h00;
	end
endfunction

// standard READ SUB-CHANNEL (0x42), format 1 (current position), 16 bytes.
// BINARY M/S/F here, unlike the Apple planes above -- see the dialect note.
function [7:0] cd_subq43_byte;
	input [31:0] cnt;
	input [7:0]  ast;                 // already mapped to 0x11/0x12/0x13
	input [7:0]  ctrl, trk;
	input [7:0]  am, as_, af;
	input [7:0]  rm, rs, rf;
	begin
		cd_subq43_byte =
			(cnt == 32'd1 )?ast:
			(cnt == 32'd3 )?8'd12:    // data length
			(cnt == 32'd4 )?8'h01:    // format code: current position
			(cnt == 32'd5 )?ctrl:     // ADR/control
			(cnt == 32'd6 )?trk:      // track
			(cnt == 32'd7 )?8'h01:    // index
			(cnt == 32'd9 )?am:       // absolute MSF
			(cnt == 32'd10)?as_:
			(cnt == 32'd11)?af:
			(cnt == 32'd13)?rm:       // relative MSF
			(cnt == 32'd14)?rs:
			(cnt == 32'd15)?rf:
			8'h00;
	end
endfunction

// Apple AUDIO STATUS (0xCC), 6 bytes. CDB[3]==1 asks for the channel volumes
// instead of the position; report both channels at full.
// RAW ast_code here, not the mapped standard one -- this is the Apple dialect.
function [7:0] cd_astat_byte;
	input [31:0] cnt;
	input        vol_form;
	input [7:0]  ast_raw;
	input [7:0]  ctrl;
	input [7:0]  am, as_, af;
	begin
		if (vol_form)
			cd_astat_byte = ((cnt == 32'd4) || (cnt == 32'd5)) ? 8'hff : 8'h00;
		else
			cd_astat_byte = (cnt == 32'd0) ? ast_raw :
			                (cnt == 32'd2) ? ctrl :
			                (cnt == 32'd3) ? cd_bin2bcd(am) :
			                (cnt == 32'd4) ? cd_bin2bcd(as_) :
			                (cnt == 32'd5) ? cd_bin2bcd(af) : 8'h00;
	end
endfunction

// READ HEADER (0x44), 8 bytes: {mode, 0, 0, 0, address}. LBA form only -- the
// MSF form needs an LBA->MSF divide this serve path does not have, and a clean
// rejection beats wrong data.
function [7:0] cd_hdr_byte;
	input [31:0] cnt;
	input [31:0] addr;
	begin
		cd_hdr_byte = (cnt == 32'd0) ? 8'h01 :   // mode 1 (data)
		              (cnt == 32'd4) ? addr[31:24] :
		              (cnt == 32'd5) ? addr[23:16] :
		              (cnt == 32'd6) ? addr[15:8]  :
		              (cnt == 32'd7) ? addr[7:0]   : 8'h00;
	end
endfunction

// buffer to store incoming commands
reg [3:0]  cmd_cnt;
reg [7:0]  cmd [9:0];

/* ----------------------- request data from/to io controller ----------------------- */

// CD-audio/TOC fetches own the address bus while their request is live.
assign io_lba = ca_io_active ? ca_io_lba : lba;

// READ prefetch (ring): keep issuing sequential sector fetches while sectors
// remain (rd_hps_blk < tlen) and the ring has space (fetched no more than
// RING_BLOCKS ahead of the Mac). This is a LEVEL signal -- the fetch engine
// below pumps one sector per io_ack until the ring is full, hiding per-sector
// HPS latency, versus the old 1-deep "fetch the next one at byte 20" which
// stalled the CPU at every 512-byte boundary. rd_hps_blk >= rd_cur_blk is
// invariant (the Mac stalls via io_busy before it can pass the fetch frontier),
// so the subtraction never underflows.
wire [22:0] rd_blk_total  = {7'd0, tlen};
wire        rd_blk_remain = (rd_hps_blk < rd_blk_total);
wire        rd_ring_space = ((rd_hps_blk - rd_cur_blk) < RING_BLOCKS);
wire req_rd = (phase == PHASE_DATA_OUT) && cmd_read && (data_len != 32'd0) &&
              !data_complete && rd_blk_remain && rd_ring_space && !cmd_aborted;

// generate an io_wr signal whenever a 512 byte block has been received or when the status
// phase of a write command has been reached.
// data_len != 0 guard: a zero-length WRITE reaches STATUS_OUT with no data phase;
// without the guard the STATUS_OUT clause would flush a stale sector-buffer block
// (the previous READ's data) to the command's LBA.
// data_in_seen on the STATUS_OUT clause is the same guard for the other way a
// write can reach STATUS with no data phase: being REJECTED. A WRITE refused for
// an out-of-range LBA -- or, on a CD, refused because the medium is read-only --
// reaches STATUS_OUT with cmd_write and data_len both still set, and flushed a
// stale sector-buffer block to the LBA it had just declined to write. Requiring
// that a data phase actually happened covers every rejection path at once.
// !cmd_aborted: the tail clause above is still true after an abort -- we are
// still in STATUS_OUT, still a write, still non-zero length -- so the flush the
// abort just cleared would re-arm on the very next cycle, re-assert io_busy, and
// suppress the REQ the abort needs in order to send its own status byte. An
// aborted command must not arm any NEW HPS transaction. See the abort branch in
// the phase FSM.
wire req_wr = ((((phase == PHASE_DATA_IN) && (data_cnt[8:0] == 0) && (data_cnt != 0)) ||
                ((phase == PHASE_STATUS_OUT) && data_in_seen))
               && cmd_write && (data_len != 32'd0) && !cmd_aborted);

// Did this command actually get a data-in phase? data_cnt cannot answer that:
// it keeps counting through STATUS_OUT and MESSAGE_OUT, so it is non-zero again
// the moment the status byte has gone out -- which is exactly when the tail
// flush of a rejected write used to fire.
reg data_in_seen;
always @(posedge clk) begin
	if((phase == PHASE_IDLE) || (phase == PHASE_CMD_IN)) data_in_seen <= 0;
	else if(phase == PHASE_DATA_IN) data_in_seen <= 1;
end

// wr_pending lives at module scope because io_busy must include it (see there).
reg wr_pending;

// Data-path io_rd (the sector-ring engine's own request). The MODULE OUTPUT is
// that ORed with the CD-audio engine's request.
//
// MISSING FROM THE 3B PORT, found 2026-08-26 by simulation. `ca_io_rd_w` was
// wired out of cd_audio and then read by nothing, so the audio/TOC engine could
// never raise a request on the HPS channel at all. Two consequences, and the
// second is the dangerous one:
//
//   * the engine can only make progress by PIGGY-BACKING on a disk fetch that
//     happens to be in flight, because io_lba is already muxed to its address;
//   * therefore a disk fetch issued while ca_io_active is high is served the
//     ENGINE's block, silently, into the sector ring. The `!io_rd` guard below
//     is what is supposed to prevent that, and it was blind.
//
// Three sites read io_rd_d (this register, io_busy, ca_grant) and exactly one
// reads the shared wire: the prefetch start guard, which is the interlock.
// Follows MacLC scsi.v:1812-1848 site for site.
reg io_rd_d;
assign io_rd = io_rd_d | ca_io_rd_w;

always @(posedge clk) begin
	reg old_wr;
	reg rd_busy;   // a read-prefetch sector fetch is outstanding

	// A reset aborts any in-flight/queued disk IO. Without this, io_rd/io_wr and
	// the pending latches survive it; if the Mac re-selects before a stale io_rd
	// clears via io_ack, the next CMD_IN sees io_busy=1 (phase!=DATA && io_rd),
	// REQ is suppressed, the command never transfers, the Mac times out and
	// resets again -- an intermittent reset/re-scan loop. These registers also
	// had no reset at all and powered up as X in
	// simulation, which made io_busy (and therefore req) X forever.
	// wdog_abort, not iostall_abort: the bus watchdog can also fire with
	// wr_pending set (io_busy's DATA_IN clause is qualified on
	// data_cnt[9] == sd_buff_sel, so a pending flush does not always hold it),
	// and a stale request left over an abort poisons the next command.
	if(any_rst || wdog_abort) begin
		io_rd_d    <= 1'b0;
		io_wr      <= 1'b0;
		wr_pending <= 1'b0;
		old_wr     <= 1'b0;
		rd_busy    <= 1'b0;
	end else begin
		old_wr <= req_wr;
		if(~old_wr & req_wr) wr_pending <= 1;

		// READ prefetch engine: while req_rd (sectors remain AND ring has space),
		// issue back-to-back sector fetches -- one per io_ack -- to keep the ring
		// filled ahead of the Mac. rd_busy holds across a fetch until rd_hps_blk
		// advances on the io_ack falling edge, so exactly one fetch is issued per
		// sector and the next can start immediately after.
		// io_rd in the guard is the SHARED wire on purpose: an audio block in
		// flight defers the ring's next fetch by one HPS block, nothing more.
		if(io_ack) io_rd_d <= 1'b0;
		else if(req_rd && !io_rd && !rd_busy) begin io_rd_d <= 1'b1; rd_busy <= 1'b1; end
		if(old_io_ack & ~io_ack) rd_busy <= 1'b0;

		// WRITE flush engine -- unchanged two-slot double-buffer behavior.
		if(io_ack) io_wr <= 1'b0;
		else if(wr_pending && !io_wr) begin io_wr <= 1'b1; wr_pending <= 0; end
	end
end

reg  stb_ack;
reg  stb_adv;
always @(posedge clk) begin
	reg old_ack;
	
	old_ack <= ack;
	stb_ack <= (~old_ack & ack); // on rising edge
	stb_adv <= (old_ack & ~ack); // on falling edge
end

reg buffer0_wr, buffer1_wr;

// store data on rising edge of ack, ...
always @(posedge clk) begin
	buffer0_wr <= 0;
	buffer1_wr <= 0;
	if(stb_ack) begin
		if(phase == PHASE_CMD_IN)  cmd[cmd_cnt] <= din;
		if(phase == PHASE_DATA_IN) begin
			buffer0_wr <= ~data_cnt[0];
			buffer1_wr <=  data_cnt[0];
		end
	end
end

// ... advance counter on falling edge
always @(posedge clk) begin
	if(phase == PHASE_IDLE) cmd_cnt <= 4'd0;
	else if(stb_adv && (phase == PHASE_CMD_IN) && (cmd_cnt != 15)) cmd_cnt <= cmd_cnt + 4'd1;
end

// count data bytes. don't increase counter while we are waiting for data from
// the io controller
reg [31:0] data_cnt;
reg        data_complete;

// For block transfers tlen contains the number of 512 bytes blocks to transfer.
// Most other commands have the bytes length stored in the transfer length field.
// And some have a fixed length idependent from any header field.
// The data transfer has finished once the data counter reaches this
// number.
//
// Allocation-length clamping. tlen6's 0 -> 256 mapping is the READ/WRITE(6)
// BLOCK-COUNT convention and does not apply to allocation lengths: for INQUIRY
// an allocation of 0 means "no data", and for REQUEST SENSE it means 4 bytes
// (the pre-SCSI-2 convention). Undo it for those, and never serve more than the
// response actually is -- over-serving leaves the initiator counting bytes that
// carry no meaning, under-serving leaves it armed for a transfer that never
// finishes. (MacLC root-caused a data-corruption class to exactly this.)
wire [31:0] alloc_len = (tlen == 16'd256) ? 32'd0 : {16'd0, tlen};
wire [31:0] sense_len = (tlen == 16'd256) ? 32'd4 : {16'd0, tlen};
// CD INQUIRY is 54 bytes (5 + additional-length 0x31); disk is the standard 36.
localparam [31:0] INQUIRY_LEN = (CDROM != 0) ? 32'd54 : 32'd36;
// The 10-byte CD commands carry their allocation raw in CDB[7:8]. tlen is
// <<2-scaled at latch time for READs only, so these must not use it.
// (latched as cd_alloc10_r at command completion -- see the CD decode block)

wire [31:0] data_len =
		 cmd_read_capacity?32'd8:
		 cmd_read?{ 7'd0, tlen, 9'd0 }:   // read command length is in 512 bytes blocks
		 cmd_write?{ 7'd0, tlen, 9'd0 }:  // write command length is in 512 bytes blocks
		 cmd_inquiry?((alloc_len < INQUIRY_LEN) ? alloc_len : INQUIRY_LEN):
		 cmd_request_sense?((sense_len < 32'd18) ? sense_len : 32'd18):
		 // ---- CD commands. 0x43/0x42 serve EXACTLY the allocation length,
		 // zero-filled past the real payload, with the header length fields
		 // still carrying the true size. Under-serving deadlocks: the Mac's
		 // blind-transfer primitive arms the FULL allocation and pumps for it,
		 // so a target that goes to STATUS early leaves the host armed with
		 // data that never comes (the LC chased this to a boot wedge).
		 // Every one of these serves EXACTLY the allocation, which is what the
		 // comment above has always claimed and what the code did not do. The
		 // caps that used to be here -- min(alloc, 512/64/16), and cd_toc_len's
		 // round-DOWN to a multiple of 4 -- are the same defect class proved on
		 // hardware for MODE SENSE; all seven under-served. One
		 // byte short deadlocks a blind initiator exactly as 227 short does.
		 // The byte functions already return 8'h00 past their real payload, so
		 // the extra length is zero fill.
		 cmd_cd_toc?cd_alloc10_r:
		 cmd_cd_toc43?cd_alloc10_r:
		 cmd_cd_subq43?cd_alloc10_r:
		 cmd_cd_subq?cd_alloc10_r:    // READ Q SUBCODE (9 real)
		 cmd_cd_astat?cd_alloc10_r:   // AUDIO STATUS (6 real)
		 cmd_cd_hdr?cd_alloc10_r:
		 cmd_cd_actl?cd_alloc10_r:    // AUDIO CONTROL: DataOut, discarded
		 ((CDROM != 0) && cmd_mode_select)?alloc_len:  // alloc 0 = no data (not 256)
		 // MODE SENSE serves the FULL allocation, padded with zeros past the
		 // real page data. The clamp that used to be here -- min(alloc,
		 // page size) -- was the 2026-08-21 "+MODE" boot hang: a real driver
		 // arms a generous buffer (0xff) or asks for page 0x3f and pumps
		 // blind for every byte, so stopping at the page's real size strands
		 // it forever. Byte 0 still reports the real data length, which is how
		 // the host knows where the padding starts. This matches the disk
		 // path, which has always served the full `tlen`.
		 ((CDROM != 0) && cmd_mode_sense)?alloc_len:
		 { 16'd0, tlen };                 // anything else: length in bytes

always @(posedge clk) begin
	if((phase != PHASE_DATA_OUT) && (phase != PHASE_DATA_IN) && (phase != PHASE_STATUS_OUT) && (phase != PHASE_MESSAGE_OUT)) begin
		data_cnt <= 0;
		data_complete <= 0;
	end else begin	
		if(stb_adv)begin	
			if(!data_complete) data_cnt <= data_cnt + 1'd1;
			data_complete <= (data_len - 1'd1) == data_cnt;
		end
	end
end

// check whether status byte has been sent
reg status_sent;
always @(posedge clk) begin
	if(phase != PHASE_STATUS_OUT) status_sent <= 0;
	else if(stb_adv) status_sent <= 1;
end

// check whether message byte has been sent
reg message_sent;
always @(posedge clk) begin
	if(phase != PHASE_MESSAGE_OUT) message_sent <= 0;
	else if(stb_adv) message_sent <= 1;
end

// Has the status byte for THIS command actually been delivered? Distinct from
// `status_sent`, which is cleared the moment the phase leaves STATUS_OUT and so
// reads 0 again in MESSAGE_OUT. The abort path needs the sticky answer: keying
// it on status_sent would send an abort taken in MESSAGE_OUT back to STATUS_OUT
// to re-issue a status the initiator already has.
reg status_done;
always @(posedge clk) begin
	if(phase == PHASE_IDLE) status_done <= 0;
	else if((phase == PHASE_STATUS_OUT) && stb_adv) status_done <= 1;
end

// This command has been aborted by a watchdog. Suppresses further HPS requests
// (see req_rd/req_wr) so the abort can get its own CHECK CONDITION out, and acts
// as the abort path's loop guard. Registered, so it is still 0 during the FIRST
// abort and 1 on any subsequent one.
reg cmd_aborted;
always @(posedge clk) begin
	if(any_rst || (phase == PHASE_IDLE)) cmd_aborted <= 0;
	else if(wdog_abort) cmd_aborted <= 1;
end

/* ----------------------- command decoding ------------------------------- */


// parse commands
wire [7:0] op_code = cmd[0];
wire [2:0] cmd_group = op_code[7:5];

// check if a complete command has been received
wire       cmd_cpl = cmd6_cpl || cmd10_cpl || cmd12_cpl;
wire       cmd6_cpl = (cmd_group == 3'b000) && (cmd_cnt == 6);
// Apple CD vendor commands 0xC0-0xCE are ALL 10-byte CDBs (MAME
// nscsi_cdrom_apple_device: command & 0xf0 == 0xc0 -> 10).
wire       cmd_apple_cd_op = (CDROM != 0) && (op_code[7:4] == 4'hc);
wire       cmd10_cpl = (((cmd_group == 3'b010) || (cmd_group == 3'b001)) && (cmd_cnt == 10))
                       || (cmd_apple_cd_op && (cmd_cnt == 10));
// Group 5 (0xA0-0xBF) = 12-byte CDBs, defined in SCSI-1. Nothing completed them
// before: the target sat in PHASE_CMD_IN forever, holding BSY, so any 12-byte
// command from any initiator wedged the bus until a reset -- a latent hang, never
// hit in practice only because MacOS sends none. Completing them makes an unknown
// group-5 opcode CHECK with invalid-op and release the bus, which is what a real
// drive does. Only cmd[0..9] are stored (the array is 10 deep and out-of-range
// writes are discarded); bytes 10-11 of a group-5 CDB are reserved + CONTROL.
wire       cmd12_cpl = (cmd_group == 3'b101) && (cmd_cnt == 12);

// https://en.wikipedia.org/wiki/SCSI_command
wire       cmd_read = cmd_read6 || cmd_read10;
wire       cmd_read6 = (op_code == 8'h08);
wire       cmd_read10 = (op_code == 8'h28);
wire       cmd_write = cmd_write6 || cmd_write10;
wire       cmd_write6 = (op_code == 8'h0a);
wire       cmd_write10 = (op_code == 8'h2a);
wire       cmd_inquiry = (op_code == 8'h12);
wire       cmd_format = (op_code == 8'h04);
wire       cmd_mode_select = (op_code == 8'h15);
wire       cmd_mode_sense = (op_code == 8'h1a);
wire       cmd_test_unit_ready = (op_code == 8'h00);
wire       cmd_read_capacity = (op_code == 8'h25);
wire       cmd_read_buffer = (op_code == 8'h3b);  // fake
wire       cmd_write_buffer = (op_code == 8'h3c); // fake
wire       cmd_verify6 = (op_code == 8'h13); // fake
wire       cmd_verify10 = (op_code == 8'h2f); // fake
// REQUEST SENSE (0x03) is MANDATORY in SCSI-1 for direct-access devices: after
// any CHECK CONDITION the initiator issues it to recover the sense data. The
// target previously rejected it (cmd_ok=0 -> CHECK CONDITION), so on hardware --
// where a transient error triggers the recovery path -- the Mac could never
// clear the condition and wedged.
wire       cmd_request_sense = (op_code == 8'h03);

// ----- Apple CD-ROM command set (CDROM targets only; all fold to 0 on disks).
// Oracle: MAME nscsi_cdrom_apple_device.
wire       cmd_cd_eject     = (CDROM != 0) && (op_code == 8'hc0);  // EJECT DISC
wire       cmd_cd_toc       = (CDROM != 0) && (op_code == 8'hc1);  // READ TOC (BCD/MSF)
wire       cmd_cd_subq      = (CDROM != 0) && (op_code == 8'hc2);  // READ Q SUBCODE (9 B)
wire       cmd_cd_astat     = (CDROM != 0) && (op_code == 8'hcc);  // AUDIO STATUS (6 B)
wire       cmd_cd_actl      = (CDROM != 0) && (op_code == 8'hce);  // AUDIO CONTROL (DataOut, discarded)
wire       cmd_cd_toc43     = (CDROM != 0) && (op_code == 8'h43);  // standard READ TOC
// This used to answer 0x43 with a format-0 response whatever was asked, because
// it ignored CDB[9] entirely. The AppleCD driver's actual dialect on the
// CDU-8004 identity is format 2 (FULL TOC), with format 1 (SESSION INFO) as
// the other real ask, so the engine pre-renders all three and these select.
wire       cmd_cd_t43f2     = cmd_cd_toc43 && (t43_fmt_r == 8'h80);
wire       cmd_cd_t43f1     = cmd_cd_toc43 && (t43_fmt_r == 8'h40);
wire       cmd_cd_subq43    = (CDROM != 0) && (op_code == 8'h42);  // standard READ SUB-CHANNEL
wire       cmd_cd_hdr       = (CDROM != 0) && (op_code == 8'h44);  // READ HEADER
wire       cmd_cd_prevent   = (CDROM != 0) && (op_code == 8'h1e);  // PREVENT/ALLOW MEDIUM REMOVAL
wire       cmd_cd_startstop = (CDROM != 0) && (op_code == 8'h1b);  // START/STOP UNIT
wire       cmd_cd_setspeed  = (CDROM != 0) && (op_code == 8'hbb);  // SET CD SPEED (12-byte CDB)
// Audio transport. Before the audio engine existed these were accepted as
// no-op GOOD rather than rejected: a CD player that gets CHECK on STOP raises
// an error dialog, and on a data disc these are never issued in the first
// place. They are real now. 0x01 REZERO and SEEK(6)/(10) carry Annex-C
// stop-audio semantics and are the AppleCD player's actual STOP button.
wire       cmd_cd_audio_nop = (CDROM != 0) && ((op_code == 8'hc8) || (op_code == 8'hc9) ||
                                               (op_code == 8'hca) || (op_code == 8'hcb) ||
                                               (op_code == 8'hcd) ||
                                               (op_code == 8'h47) || (op_code == 8'h48) ||
                                               (op_code == 8'h4b) || (op_code == 8'h4e) ||
                                               (op_code == 8'h01) ||
                                               (op_code == 8'h0b) || (op_code == 8'h2b) ||
                                               (op_code == 8'h45) || (op_code == 8'ha5));
// BOTH eject forms: the Apple vendor 0xC0 and the standard START/STOP UNIT
// with LoEj=1 / Start=0, which is how the System 7 AppleCD driver actually
// ejects. Missing the 0x1B form means `mounted` never drops, the driver's
// insertion poll sees READY and silently remounts the volume.
wire       cmd_cd_eject_any = cmd_cd_eject ||
                              (cmd_cd_startstop && cmd[4][1] && !cmd[4][0]);

// valid command in buffer? TODO: check for valid command parameters
wire  cmd_ok_hd = cmd_read || cmd_write || cmd_inquiry || cmd_test_unit_ready ||
		  cmd_read_capacity || cmd_mode_select || cmd_format || cmd_mode_sense ||
		  cmd_read_buffer || cmd_write_buffer || cmd_verify6 || cmd_verify10 ||
		  cmd_request_sense;

// The CD is READ-ONLY: WRITE / FORMAT / VERIFY / WRITE BUFFER are deliberately
// absent, so they CHECK with ILLEGAL REQUEST exactly as a real AppleCD does.
wire  cmd_ok_cd = cmd_read || cmd_inquiry || cmd_test_unit_ready ||
		  cmd_read_capacity || cmd_mode_select || cmd_mode_sense ||
		  cmd_request_sense || cmd_cd_eject || cmd_cd_toc || cmd_cd_subq ||
		  cmd_cd_astat || cmd_cd_actl || cmd_cd_audio_nop ||
		  cmd_cd_toc43 || cmd_cd_subq43 || cmd_cd_hdr ||
		  cmd_cd_prevent || cmd_cd_startstop || cmd_cd_setspeed;

// Our answer to a command against an empty drive: SK_NOT_READY + the vendor
// ASC 0xB0, from MAME's return_no_cd. MacOS demonstrably BRANCHES on this
// value -- MAME records that 0x3A makes it hammer the user to format the disc
// -- so it is not a free choice. 0xB0 is what the AppleCD driver expects and
// what this target ships; 3A/28/04 were each tried from the OSD during the
// wedge hunt and none of them was the fault.
wire [3:0] cd_nomedia_key = 4'h2;   // NOT READY
wire [7:0] cd_nomedia_asc = 8'hb0;
wire  cmd_ok = (CDROM != 0) ? cmd_ok_cd : cmd_ok_hd;

// Media-dependent commands fail with the AppleCD no-disc sense while no image
// is mounted. MAME's return_no_cd uses SK_NOT_READY + the vendor ASC 0xB0 --
// NOT the obvious 0x3A "medium not present", because 0x3A makes MacOS hammer
// the drive asking the user to format it.
wire  cd_needs_media = cmd_test_unit_ready || cmd_read || cmd_read_capacity ||
		  cmd_cd_toc || cmd_cd_subq || cmd_cd_astat || cmd_cd_actl ||
		  cmd_cd_toc43 || cmd_cd_subq43 || cmd_cd_hdr || cmd_cd_audio_nop;
// !ca_toc_ready: the engine fetches the real TOC from Main's MCDA blob after a
// mount, which is an HPS round-trip, not the ~150 cycles a local MSF
// conversion took. Serving media commands in that window returned the PREVIOUS
// disc's TOC, which is the wrong answer after a disc swap. Report the drive as
// not-ready-yet instead -- the correct SCSI answer for a drive still spinning
// up, and what the driver's retry path already handles.
//
// This MUST be the engine's flag and not a local one. TOC
// serving to the engine's RAMs, and every one of those serve paths already
// gates on ca_toc_ready and emits 0x00 when it is low. Gating readiness on a
// DIFFERENT, earlier flag therefore opened a window where a TOC command took
// GOOD status and an all-zeros payload -- a driver cannot retry that, because
// nothing told it anything was wrong. One flag, used by both.
wire  cd_no_media = (CDROM != 0) && (!mounted || !ca_toc_ready) && cd_needs_media;

// ---- UNIT ATTENTION after a reset ---------------------------------------
// A real drive raises UNIT ATTENTION on power-on or a bus reset and reports it
// on the next command, so an initiator learns its state is stale. The Plus
// asserts SCSI bus RST at boot, so the ROM's first command to the CD gets
// CHECK CONDITION and the scan moves on WITHOUT ever reading block 0 -- which
// is what stops it walking the partition map and hanging.
//
// This is why the block size can stay 2048, and it must: the 512-byte build
// (7fc96906) fixed the hang and then broke mounting, because ISO 9660 is
// DEFINED in 2048-byte sectors. Measured on hardware -- the driver reads 18
// blocks from LBA 0, which reaches the Primary Volume Descriptor at sector 16
// (byte 32768) only at 2048 bytes a block.
//
// Period-documented for THIS ROM: the "Loud Harmonicas" revision (4D1F8172,
// the one we run) exists because of drives that return UNIT ATTENTION on power
// up or reset in the boot sequence loop, so Apple explicitly accommodated it.
//
// CDROM ONLY, deliberately. A real disk raises it too, but making the boot
// disks CHECK on the ROM's first command risks the machine not booting AT ALL
// -- far worse than the bug being fixed, and it tests nothing we need.
//
// INQUIRY and REQUEST SENSE are exempt, per SCSI: they are how an initiator
// identifies and interrogates a device it has just reset. Reporting the
// condition CLEARS it; a sticky UNIT ATTENTION would make the drive unusable.
reg   cd_unit_attn = 1'b0;
wire  cd_unit_attn_rej = (CDROM != 0) && cd_unit_attn
                         && !cmd_inquiry && !cmd_request_sense;
always @(posedge clk) begin
	if (any_rst)                          cd_unit_attn <= (CDROM != 0);
	else if (new_cmd && cd_unit_attn_rej) cd_unit_attn <= 1'b0;
end

// ---- Spin-up: NOT READY for a while after a reset -----------------------
// UNIT ATTENTION above is correct SCSI but it did NOT stop the ROM. Measured
// on hardware (eea855d6): the same wedge as before, same `PIOS lba=1024`, same
// phase list stopping at STATUS. It is cleared by the command that reports it,
// so the ROM retried, read block 0, walked the partition map and hung.
//
// A real caddy drive takes SECONDS after power-on or a reset to spin the disc
// up and read its TOC, reporting NOT READY / ASC 0x04 ("logical unit is in
// process of becoming ready") throughout. That condition is PERSISTENT -- not
// consumed by being reported -- which is exactly the property UNIT ATTENTION
// lacked. The Plus boot scan runs a second or two after reset, so a real drive
// was not ready for any of it and the ROM moved on to the next ID.
//
// The block size stays 2048, so ISO 9660 mounting is untouched. The 512-byte
// build (7fc96906) fixed the hang and broke mounting instead: the driver reads
// 18 blocks from LBA 0 for the Primary Volume Descriptor at sector 16, which
// is only reachable at 2048 bytes a block.
//
// Deliberately NOT armed by img_mounted. A real drive spins up on insertion
// too, but that path is already gated by ca_toc_ready, and a multi-second
// stall on mount-after-boot would be a user-visible regression for no benefit.
// The reset is what the boot scan sees, and it is all we need.
//
// INQUIRY is not gated (it is not in cd_needs_media): a real drive identifies
// itself while the disc is still coming up.
reg  [SPINUP_LOG:0] cd_spinup = 0;
wire cd_spinning_up = (CDROM != 0) && !cd_spinup[SPINUP_LOG];
wire cd_spinup_rej  = cd_spinning_up && cd_needs_media;
always @(posedge clk) begin
	if (any_rst)                     cd_spinup <= 0;
	else if (!cd_spinup[SPINUP_LOG]) cd_spinup <= cd_spinup + 1'd1;
end

// READ HEADER in MSF form: rejected (see cd_hdr_byte).
wire  cd_hdr_msf_rej = cmd_cd_hdr && cmd[1][1];

// ----- LBA bounds ----------------------------------------------------------
// Nothing used to compare an incoming LBA against the mounted medium. An
// out-of-range address was latched and handed straight to the HPS, which cannot
// service it; io_busy then holds REQ low and the target sits on the bus until
// the io-stall watchdog fires ~516 ms later. A real drive fails the command
// immediately with ILLEGAL REQUEST / 0x21, which is what this does.
//
// These read the CDB combinationally rather than the lba/tlen registers,
// because those latch on the same edge cmd_cpl is evaluated -- at decision time
// they still hold the PREVIOUS command's values.
//
// One comparison covers both personalities: on the CD path `capacity` and the
// CDB address are both in 2048-byte logical blocks (the <<2 to 512-byte HPS
// sectors happens at latch time), and on the disk path both are 512-byte
// sectors.
wire [31:0] cdb_lba  = cmd6_cpl ? {11'd0, lba6} : lba10;
wire [16:0] cdb_blks = cmd6_cpl ? {8'd0, tlen6} : {1'b0, tlen10};
// One PAST the last block addressed. Widened to 33 bits before the add: the
// length can be 65535, so a 32-bit sum would wrap and a wildly out-of-range LBA
// would test as valid.
wire [32:0] cdb_end  = {1'b0, cdb_lba} + {16'd0, cdb_blks};
// cdb_blks != 0: a zero-length transfer moves no data and is legal, not an error.
// The test is on the END, not the start -- a transfer that begins in range and
// runs off the end is equally out of range, and checking only the start would
// stall on the last sector instead of refusing the command.
wire  lba_out_of_range = (cmd_read || cmd_write) && mounted && (cdb_blks != 0) &&
                         (cdb_end > ({1'b0, capacity} + 33'd1));

// ----- REQUEST SENSE state -------------------------------------------------
// Key/ASC latched when a command CHECKs, cleared by the next successful
// non-REQUEST-SENSE command (SCSI-1 semantics: sense persists until the next
// command from the same initiator, and REQUEST SENSE itself must not clear it
// before it has been served). The disk path has exactly one failure mode --
// an opcode we do not implement -- so the whole thing is these two registers.
// The CD target extends this latch with its media/audio conditions rather
// than adding a parallel CDROM-only path.
reg [3:0] sense_key = 4'd0;
reg [7:0] sense_asc = 8'd0;
reg       cd_prevent = 1'b0;   // PREVENT/ALLOW MEDIUM REMOVAL state

// New-command strobe: one clk on the CDB completing.
reg  cmd_cpl_d = 1'b0;
always @(posedge clk) cmd_cpl_d <= (phase == PHASE_CMD_IN) && cmd_cpl;
wire new_cmd = (phase == PHASE_CMD_IN) && cmd_cpl && !cmd_cpl_d;

// The eject actually happens (drops `mounted`) only if the medium is not locked.
wire cd_eject_pulse = new_cmd && cmd_cd_eject_any && !cd_prevent;

// ---- fetch-frontier violation detector -----------------------------------
// The frontier guard (`io_busy` -> `req`) is ADVISORY: it only removes REQ.
// Nothing in this target, and nothing in ncr5380.sv, refuses an ACK -- rdata
// serves cur_data on any DACK read, dma_ack is gated on the bus phase alone,
// and data_cnt advances on every ACK edge. A BLIND pseudo-DMA pump therefore
// walks straight through the frontier and is handed the ring slot's PREVIOUS
// occupant until the late fill lands: stale head, fresh tail, GOOD status.
// That is the 2026-08-26 CD->disk corruption. The Mac Plus SCSI Manager runs
// exactly such a pump,
// and the Plus has no bus hold-off with which to stop it.
//
// This does NOT stop the pump -- only the CPU hold-off can. It makes the
// breach LOUD: a read that was served unfilled
// sectors ends in CHECK CONDITION instead of GOOD, so the driver retries
// rather than writing garbage to disk and reporting success.
//
// Detection is exact, not heuristic. Inside a read data phase rd_cur_unfilled
// can only RISE as a result of the initiator's own advance across a sector
// boundary (rd_cur_blk = data_cnt[31:9]), and rd_hps_blk only ever grows,
// which clears it. So there is no poll-to-read race: an initiator that honours
// the withdrawn REQ cannot present an ACK while this holds, and a polite
// initiator therefore never trips it.
//
// The condition mirrors io_busy's READ clause term for term -- `mounted`
// included, because media loss mid-READ deliberately does not stall (see
// io_busy) and so is not a frontier breach; it keeps reporting through its own
// path. `data_phase_complete` is added because it too suppresses REQ, so an
// ACK past the transfer length is a DIFFERENT violation and must not be
// reported as this one.
wire frontier_breach = stb_ack && (phase == PHASE_DATA_OUT) && cmd_read
                       && mounted && rd_cur_unfilled && !data_phase_complete;

// Sticky for the command, cleared by the next CDB -- the same lifetime the
// sense latch uses, so the status byte and a follow-up REQUEST SENSE always
// agree about what happened.
reg frontier_violated = 1'b0;
always @(posedge clk) begin
	if (any_rst || new_cmd)   frontier_violated <= 1'b0;
	else if (frontier_breach) frontier_violated <= 1'b1;
end

always @(posedge clk) begin
	if (any_rst) begin
		sense_key  <= 4'd0;
		sense_asc  <= 8'd0;
		cd_prevent <= 1'b0;
	end else if (wdog_abort) begin
		// ABORTED COMMAND. The ASC deliberately carries the OPCODE that stalled
		// rather than a standard additional-sense code: if this ever fires we
		// need to know what stranded us, and a REQUEST SENSE is the only channel
		// out of the target. Non-standard, and worth it -- this is an error path
		// that would otherwise have been an unrecoverable hang.
		sense_key <= 4'hB;
		sense_asc <= op_code;
	end else if (frontier_breach) begin
		// ABORTED COMMAND / DATA PHASE ERROR. Key 0xB is the retryable key, and
		// it is already what wdog_abort uses for the other "target could not
		// sustain this transfer" case; the FIXED asc 0x4b is what tells the two
		// apart in a REQUEST SENSE, since wdog_abort carries the stalled opcode
		// there instead. Deliberately not MEDIUM ERROR: the medium is fine, the
		// target failed to keep the data phase fed.
		sense_key <= 4'hB;
		sense_asc <= 8'h4b;
	end else if (new_cmd) begin
		if (cd_unit_attn_rej) begin
			sense_key <= 4'h6;  // UNIT ATTENTION
			sense_asc <= 8'h29; // power on, reset, or bus device reset occurred
		end else if (cd_spinup_rej) begin
			sense_key <= 4'h2;  // NOT READY
			sense_asc <= 8'h04; // logical unit is in process of becoming ready
		end else if (!cmd_ok) begin
			sense_key <= 4'h5;  // ILLEGAL REQUEST
			sense_asc <= 8'h20; // invalid command operation code
		end else if (cd_no_media) begin
			sense_key <= cd_nomedia_key;
			sense_asc <= cd_nomedia_asc;
		end else if (lba_out_of_range) begin
			sense_key <= 4'h5;  // ILLEGAL REQUEST
			sense_asc <= 8'h21; // logical block address out of range
		end else if (cd_hdr_msf_rej) begin
			sense_key <= 4'h5;  // ILLEGAL REQUEST
			sense_asc <= 8'h24; // invalid field in CDB
		end else if (cmd_cd_eject_any) begin
			if (cd_prevent) begin
				sense_key <= 4'h5;  // ILLEGAL REQUEST
				sense_asc <= 8'h80; // "prevent bit is set" (MAME)
			end else begin
				sense_key <= 4'h2;  // NOT READY
				sense_asc <= 8'h3a; // medium not present, post-eject
			end
		end else if (cmd_cd_prevent) begin
			cd_prevent <= cmd[4][0];
			sense_key  <= 4'd0;
			sense_asc  <= 8'd0;
		end else if (!cmd_request_sense) begin
			sense_key <= 4'd0;  // NO SENSE
			sense_asc <= 8'd0;
		end
	end
end

// latch parameters once command is complete
reg [31:0] lba;
reg [15:0] tlen;

always @(posedge clk) begin
	if (old_io_ack & ~io_ack) lba <= lba + 1'd1;
	if(cmd_cpl && (phase == PHASE_CMD_IN)) begin
		// CDROM READs address 2048-byte logical blocks; the HPS block device is
		// 512-byte sectors, so scale lba/tlen by 4 AT LATCH TIME and the whole
		// downstream ring / flush / data_len machinery runs unmodified in
		// 512-byte units. Non-READ commands keep the raw CDB values, because
		// their lengths are byte counts, not block counts.
		if ((CDROM != 0) && cmd_read) begin
			lba  <= (cmd6_cpl?{11'd0, lba6}:lba10) << 2;
			tlen <= (cmd6_cpl?{7'd0, tlen6}:tlen10) << 2;
		end else begin
			lba <= cmd6_cpl?{11'd0, lba6}:lba10;
			tlen <= cmd6_cpl?{7'd0, tlen6}:tlen10;
		end

		// CD serve parameters, decoded here once (see the CD decode block).
		c1_op_r        <= cmd[9][7:6];
		c1_trk_r       <= cmd[5];
		t43_start_r    <= cmd[6];
		t43_fmt_r      <= cmd[9];
		cd_page_r      <= cmd[2][5:0];
		cd_astat_vol_r <= (cmd[3] == 8'h01);
		cd_alloc10_r   <= {16'd0, cmd[7], cmd[8]};
		cd_hdr_addr_r  <= {cmd[2], cmd[3], cmd[4], cmd[5]};
	end
end
   
// logical block address
wire [7:0] cmd1 = cmd[1];
wire [20:0] lba6 = { cmd1[4:0], cmd[2], cmd[3] };
wire [31:0] lba10 = { cmd[2], cmd[3], cmd[4], cmd[5] };

// transfer length
wire [8:0]  tlen6 = (cmd[4] == 0)?9'd256:{1'b0,cmd[4]};
wire [15:0] tlen10 = { cmd[7], cmd[8] };


// ---- bus watchdog --------------------------------------------------------
// A target that stops making progress must never hold BSY indefinitely: because
// bus_busy gates every other target, one stuck target takes the WHOLE bus down
// and the machine freezes -- boot disk included.
//
// We can be driven into that state by any CDB whose length we do not know.
// Lengths are defined for groups 0/1/2/5 and the Apple 0xC0-0xCF range; groups
// 3, 4, 7 and 0xD0-0xDF are not, so cmd_cpl never asserts and the target sits in
// COMMAND phase forever. The sources also disagree about 0xC0 EJECT (MAME: a
// 10-byte CDB; BlueSCSI: 6), so even a "known" opcode can strand us.
//
// A real drive fails a malformed command and releases the bus. So: if no ACK
// edge arrives for ~129 ms while we are not legitimately waiting on the HPS,
// abort with CHECK CONDITION and let go. That is well inside the Mac's own
// ~250 ms SCSI timeout, so the host sees an ordinary failed command rather than
// a dead bus. In normal operation this can never fire -- the initiator answers
// in microseconds -- so it is inert unless something is already broken.
// WDOG_LOG is overridden down to a few thousand cycles by the testbenches so the
// recovery can be exercised in reasonable sim time. The timeout VALUE is not the
// thing under test -- the recovery behaviour is.
parameter SPINUP_LOG = 27;              // 2^27 clks @31.3344MHz = ~4.3 s spin-up
parameter WDOG_LOG = 22;               // 2^22 clks @31.3344MHz = ~134 ms

// ---- IO-stall watchdog ----------------------------------------------------
// The bus watchdog above is RESET by io_busy, because a legitimate HPS sector
// fetch runs far longer than its period. That leaves exactly one state
// unguarded: a fetch that never completes AT ALL. io_busy then holds REQ low
// (see `req`) and resets the bus watchdog every cycle, so the target keeps BSY
// forever while the initiator polls for data that can never arrive -- a hang
// with no recovery path, by construction. Seen on hardware as the SCSI
// activity LED stuck on with the Mac spinning in its pseudo-DMA poll loop.
//
// A real drive that loses a sector fetch still releases the bus. This is a
// second, much longer timer that runs ONLY while io_busy holds, and aborts
// through the same path as the bus watchdog. Clearing the stale io_rd/io_wr
// is not optional: io_busy suppresses REQ, so without it the abort could not
// even send its own CHECK CONDITION, and the next command would inherit the
// wedge (the same failure the any_rst clear exists to prevent).
parameter IOWDOG_LOG = 24;             // 2^24 clks @31.3344MHz = ~535 ms
reg [IOWDOG_LOG-1:0] iowdog = 0;
wire iowdog_expired = &iowdog;
wire iostall_abort  = iowdog_expired;
always @(posedge clk) begin
	if (any_rst || !io_busy || (phase == PHASE_IDLE) || iostall_abort)
		iowdog <= 0;
	else
		iowdog <= iowdog + 1'd1;
end
reg [WDOG_LOG-1:0] wdog = 0;
wire wdog_expired = &wdog;
wire wdog_abort   = (wdog_expired && (phase != PHASE_IDLE)) || iostall_abort;

always @(posedge clk) begin
	if (any_rst || (phase == PHASE_IDLE) || stb_ack || stb_adv || io_busy || wdog_abort)
		wdog <= 0;
	else if (!wdog_expired)
		wdog <= wdog + 1'd1;
end

// the 5380 changes phase in the falling edge, thus we monitor it
// on the rising edge
// CD-audio engine strobes are 1-clk pulses raised on command acceptance
// below. Clearing them here, ahead of every branch, covers the watchdog
// abort path too - a clear placed only in the final else would let a pulse
// stretch across an abort cycle.
always @(posedge clk) begin
	ca_cmd_stb <= 1'b0; ca_read_stb <= 1'b0; ca_eject_stb <= 1'b0;
	if(any_rst) begin
		phase <= PHASE_IDLE;
	end else if (wdog_abort) begin
		// Give up and release the bus rather than wedge it -- but not before
		// telling the initiator, if it has not been told yet.
		//
		// This used to key on the phase, treating "in STATUS_OUT" as "status
		// already sent". That is false for a WRITE: req_wr's tail clause issues
		// the last partial-sector flush AT PHASE_STATUS_OUT, so a stalled write
		// aborts while already in STATUS_OUT with the status byte still
		// undelivered -- and the target dropped BSY with no status and no
		// COMMAND COMPLETE, leaving the driver polling for a completion that
		// could never arrive. That is the 2026-08-22 soak wedge.
		//
		// cmd_aborted is the loop guard: one attempt to report the error, then
		// release. A target must never be able to hold BSY indefinitely.
		if (status_done || cmd_aborted)
			phase <= PHASE_IDLE;
		else begin
			status <= `STATUS_CHECK_CONDITION;
			phase  <= PHASE_STATUS_OUT;
		end
	end else begin
		if(phase == PHASE_IDLE) begin
			// Only answer selection on a FREE bus (SEL asserted, no BSY). While
			// another target holds BSY its dout is wired-ORed onto the data bus,
			// so a stray bit in that byte could otherwise "select" this target
			// mid-dialog and two targets would then consume the shared ACK stream
			// in parallel -- command/LBA corruption, and misdirected writes.
			// A CD-ROM drive is present on the bus even with no disc inserted
			// (the AppleCD driver polls TEST UNIT READY to detect insertion),
			// so the CD target selects on cd_enable rather than on `mounted`.
			if(sel && din[ID] && ((CDROM != 0) ? cd_enable : mounted) && !bus_busy)
				phase <= PHASE_CMD_IN;
		end

		else if(phase == PHASE_CMD_IN) begin
			// check if a full command is in the buffer
			if(cmd_cpl) begin
				$display("New command on target %d: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", ID, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7], cmd[8], cmd[9]);
				// is this a supported and valid command?
				// (CDROM: media-dependent commands CHECK with the no-disc sense
				// while unmounted, and a prevent-blocked EJECT CHECKs too.)
				if(cmd_ok && !cd_no_media && !cd_audio_read_rej && !cd_hdr_msf_rej && !lba_out_of_range && !cd_unit_attn_rej && !cd_spinup_rej) begin
					// yes, continue
					status <= (cmd_cd_eject_any && cd_prevent) ? `STATUS_CHECK_CONDITION : `STATUS_OK;

					// Notify the CD-audio engine. All constant 0 on a disk target.
					// Raised HERE, on acceptance alongside STATUS_OK, because the
					// engine's contract (cd_audio.sv:39) is that the CDB is latched
					// with status GOOD already decided - not at raw decode.
					ca_cmd_stb   <= cmd_cd_audio_nop;
					ca_read_stb  <= cmd_read && (CDROM != 0);
					ca_eject_stb <= cmd_cd_eject_any && !cd_prevent;

					// continue according to command

					// these commands return data
					if(cmd_read || cmd_inquiry || cmd_read_capacity || cmd_mode_sense || cmd_read_buffer || cmd_request_sense ||
					   cmd_cd_toc || cmd_cd_toc43 || cmd_cd_subq || cmd_cd_subq43 || cmd_cd_astat || cmd_cd_hdr) phase <= PHASE_DATA_OUT;
					// these commands receive dataa
					else if(cmd_write || cmd_mode_select || cmd_write_buffer || cmd_cd_actl) phase <= PHASE_DATA_IN;
					// and all other valid commands are just "ok"
					else phase <= PHASE_STATUS_OUT;
				end else begin
					// no, report failure
					status <= `STATUS_CHECK_CONDITION;
					phase <= PHASE_STATUS_OUT;
				end
			end
		end

		// data_done, not data_complete: a zero-length data phase never sees an
		// ACK edge, so data_complete would never assert and the phase would hang.
		else if(phase == PHASE_DATA_OUT) begin
			if(data_done) begin
				// A read that was served unfilled sectors must not report GOOD.
				// The bytes have already gone to the initiator, so this cannot
				// un-corrupt the transfer -- it makes it RETRYABLE instead of
				// silent, which is the whole point. See frontier_breach.
				if(frontier_violated) status <= `STATUS_CHECK_CONDITION;
				phase <= PHASE_STATUS_OUT;
			end
		end

		else if(phase == PHASE_DATA_IN) begin
			if(data_done) phase <= PHASE_STATUS_OUT;
		end

		else if(phase == PHASE_STATUS_OUT) begin
			if(status_sent) phase <= PHASE_MESSAGE_OUT;
		end

		else if(phase == PHASE_MESSAGE_OUT) begin
			if(message_sent) phase <= PHASE_IDLE;
		end
		
		else
			phase <= PHASE_IDLE;  // should never happen
	end
end
   
   
// =====================================================================
// CD audio engine (CDROM targets only; every ca_* wire folds to a constant
// on a disk target). Owns the AppleCD playback state machine, the real TOC,
// and audio-frame streaming from the HPS windows.
// =====================================================================

wire        ca_io_active, ca_io_rd_w;
wire [31:0] ca_io_lba;
wire  [7:0] ca_ast_code, ca_cur_ctrl, ca_cur_trk;
wire  [7:0] ca_abs_m, ca_abs_s, ca_abs_f, ca_rel_m, ca_rel_s, ca_rel_f;
wire  [7:0] ca_toc_q0, ca_toc_q1, ca_toc_q2, ca_toc_q3;
wire        ca_toc_ready;
wire  [7:0] ca_t43_q0, ca_t43_q1, ca_t43_q2, ca_t43_q3;
wire  [9:0] ca_t43_len;
wire  [7:0] ca_t2_q0, ca_t2_q1, ca_t2_q2, ca_t2_q3;
wire  [9:0] ca_t2_len;
wire        ca_disc_audio;

// ---- TOC serving from the engine's pre-rendered response RAMs -----------
// Serve EXACTLY the armed allocation, zero-filled past the real payload. A
// fixed-size response that ignores a larger allocation strands a blind host
// exactly like the page 0x0E under-serve did (the 2026-08-20 boot wedge), so
// each plane below pads rather than stopping short. Carried over from the
// synthesized TOC this replaced; the rule outlived the code that taught it.
//
// A single-track TOC used to be synthesized from `capacity`. The engine builds
// the real one from Main's MCDA blob, so these address its images instead. All
// three RAMs are 1-clock registered reads, the same latency as the sector
// dpram this file already serves combinationally (buffer0/buffer1 at the
// cmd_dout mux), and they use the same even/odd plane split - so the underlying
// address advances every TWO bytes and the margin is identical to that proven
// path. MacLC's 4-deep data_cnt_next lookahead is for their FOUR parallel dout
// lanes, not for latency; we have one lane and use q0 alone.
function [7:0] cd_bcd2bin;
	input [7:0] v;
	cd_bcd2bin = (v[7:4] * 8'd10) + {4'd0, v[3:0]};
endfunction

// Apple 0xC1: layout [0..3] header, [4..7] lead-out, [8+4k..] track k+1.
// Mode (latched CDB[9][7:6]) picks the base; track mode indexes by BCD CDB[5].
// Reads past the 99 precomputed descriptors clamp to the last one with the
// byte-in-descriptor preserved - MAME's "keep returning the last track".
wire  [7:0] ca_toc_trk_bin = cd_bcd2bin(c1_trk_r);
wire  [8:0] ca_toc_trk_k   = (ca_toc_trk_bin == 8'd0)  ? 9'd0  :
                             (ca_toc_trk_bin >  8'd99) ? 9'd98 :
                             {1'b0, ca_toc_trk_bin} - 9'd1;
wire  [8:0] ca_toc_base    = (c1_op_r == 2'b01) ? 9'd4 :
                             (c1_op_r == 2'b10) ? (9'd8 + {ca_toc_trk_k[6:0], 2'b00}) :
                                                  9'd0;
wire  [8:0] ca_toc_raw     = ca_toc_base + data_cnt[8:0];
wire  [8:0] ca_toc_addr    = (ca_toc_raw < 9'd404) ? ca_toc_raw
                                                   : (9'd400 + {7'd0, ca_toc_raw[1:0]});
wire  [7:0] cd_toc_dout    = ca_toc_ready ? ca_toc_q0 : 8'h00;

// Standard 0x43 format 0 (MSF form): 4-byte header then 8-byte descriptors.
// A start track other than 1 skips whole descriptors, and the header's u16be
// data-length must then describe the SHORTENED response, not the whole table.
wire  [6:0] ca_t43_nreal = (ca_t43_len >= 10'd14) ? ((ca_t43_len - 10'd14) >> 3) + 7'd1 : 7'd1;
wire  [6:0] ca_t43_soff  =
	(t43_start_r == 8'h00 || t43_start_r == 8'h01) ? 7'd0 :
	(t43_start_r == 8'hAA)                          ? ca_t43_nreal :
	(t43_start_r >  {1'b0, ca_t43_nreal})           ? ca_t43_nreal :
	                                                  t43_start_r[6:0] - 7'd1;
wire  [9:0] ca_t43_flen  = {(7'd1 + ca_t43_nreal - ca_t43_soff), 3'b000} + 10'd2;
wire  [9:0] ca_t43_tot   = ca_t43_flen + 10'd2;
wire  [8:0] ca_t43_addr  = (data_cnt < 32'd4) ? data_cnt[8:0]
                         : (9'd4 + {ca_t43_soff, 3'b000} + (data_cnt[8:0] - 9'd4));
// `tot` and `flen` are passed IN rather than read from the body, per the rule
// established at cd_mode_sense_byte above. Reading them from the body is what
// the 2026-08-26 A&S caught: `ca_t43_tot` was reported assigned-but-never-read
// while `ca_t2_len` -- syntactically the same kind of body read one function
// down -- was not, so the two disagreed about a rule the file already knows.
// Passing them in is correct by construction and settles it either way.
function [7:0] t43_hdr_fix;
	input [31:0] cnt;
	input [9:0]  tot;
	input [9:0]  flen;
	input [7:0]  raw;
	t43_hdr_fix = (cnt >= {22'd0, tot}) ? 8'h00 :
	              (cnt == 32'd0) ? {6'd0, flen[9:8]} :
	              (cnt == 32'd1) ? flen[7:0] : raw;
endfunction
wire  [7:0] cd_toc43_dout = ca_toc_ready
                          ? t43_hdr_fix(data_cnt, ca_t43_tot, ca_t43_flen, ca_t43_q0)
                          : 8'h00;

// Format 2 (FULL TOC) / format 1 (SESSION INFO): the T2 plane IS the response
// image, linear addressing, session page at [496..507]. Zero-fill past the real
// payload - serving pads to the armed allocation, and the u16be length fields
// carry the true sizes.
wire  [8:0] ca_t2_addr = (cmd_cd_t43f1 ? 9'd496 : 9'd0) + data_cnt[8:0];
function [7:0] t2_fix;
	input [31:0] cnt;
	input [9:0]  len;
	input [7:0]  raw;
	t2_fix = (cnt >= {22'd0, len}) ? 8'h00 : raw;
endfunction
function [7:0] sess_fix;
	input [31:0] cnt;
	input [7:0]  raw;
	sess_fix = (cnt >= 32'd12) ? 8'h00 : raw;
endfunction
wire  [7:0] cd_toc2_dout = !ca_toc_ready ? 8'h00 :
                           cmd_cd_t43f1 ? sess_fix(data_cnt, ca_t2_q0)
                                        : t2_fix(data_cnt, ca_t2_len, ca_t2_q0);

// ---- live sub-channel / audio status serving ----------------------------
// The engine's position registers only become readable here, where they are in
// scope; the serve functions live up with the other CD responses and take every
// value as an ARGUMENT, per the rule at the cd_mode_sense_byte note.
//
// Standard audio-status codes ([PIONEER] 2-27C via Snow): the engine's ast_code
// is a raw drive code (0/1/3/5) and the 0x42 plane needs 0x11 play / 0x12
// paused / 0x13 completed. CD_AST_STOPPED remains the correct answer when the
// engine is not playing -- what was wrong before was reporting it ALWAYS.
wire  [7:0] ca_ast_std  = (ca_ast_code == 8'd0) ? 8'h11 :
                          (ca_ast_code == 8'd1) ? 8'h12 : CD_AST_STOPPED;

wire  [7:0] cd_subq_dout   = cd_subq_byte(data_cnt, ca_cur_trk,
                                          ca_rel_m, ca_rel_s, ca_rel_f,
                                          ca_abs_m, ca_abs_s, ca_abs_f);
wire  [7:0] cd_subq43_dout = cd_subq43_byte(data_cnt, ca_ast_std,
                                            ca_cur_ctrl, ca_cur_trk,
                                            ca_abs_m, ca_abs_s, ca_abs_f,
                                            ca_rel_m, ca_rel_s, ca_rel_f);
wire  [7:0] cd_astat_dout  = cd_astat_byte(data_cnt, cd_astat_vol_r,
                                           ca_ast_code, ca_cur_ctrl,
                                           ca_abs_m, ca_abs_s, ca_abs_f);

// A data READ aimed at an audio track must CHECK, not be served as garbage.
// Without this an audio-only disc mounts with a non-zero capacity and returns
// the audio extent as if it were a filesystem.
wire        cd_audio_read_rej = (CDROM != 0) && mounted && cmd_read && ca_disc_audio;




// Command strobes into the engine. Wired to the decode next; the engine is
// harmless with them low - it still acquires the TOC on the mount pulse,
// which is what makes this stage independently testable (a TOC fetch shows
// up on PIOS as win=TOC, the first thing that actually exercises the tag).
reg         ca_cmd_stb   = 1'b0;
reg         ca_read_stb  = 1'b0;
reg         ca_eject_stb = 1'b0;

// CD Audio Control page 0x0E output ports. MODE SELECT does not write these
// yet, so default to the drive's power-on state: port 0 = left at full, port
// 1 = right at full (channel 0x01 = left source, 0x02 = right).
wire  [7:0] cd_ap_ch0 = 8'h01, cd_ap_vol0 = 8'hff;
wire  [7:0] cd_ap_ch1 = 8'h02, cd_ap_vol1 = 8'hff;

// CA grant: the audio/TOC engine's fetches are HPS-channel-only (they never
// touch the SCSI bus), so they may interleave with an ACTIVE READ command's
// serving phase. DO NOT "tighten" this to full bus-idle: MacLC did, and it
// starved the frame stream to ~42 of the required 75 frames/s whenever the
// guest read data from the same disc - audible crackle from sample-hold at
// every late frame (their HW capture 2026-07-18). The io-free terms still
// serialize the channel per-op, and the ~ca_io_active scoping above keeps CA
// acks out of the data-path accounting. DATA_IN (writes) stays excluded: a CD
// is read-only so it never occurs.
wire ca_grant = (phase == PHASE_IDLE || (cmd_read && phase == PHASE_DATA_OUT))
                && !io_rd_d && !io_wr && !io_ack && mounted;

generate if (CDROM != 0) begin : g_cd_audio
	cd_audio #(.CLK_HZ(32'd31_334_400)) cd_audio_i (   // clk_sys rate; audio pitch verifies it
		// NOT .rst(rst): scsi.v's `rst` is the SCSI BUS reset (ICR RST from the
		// initiator), and cd_audio's `rst` means SYSTEM reset. Tying both to the
		// bus reset would wipe the TOC every time a driver resets the bus, which
		// they do routinely at init - cd_audio.sv:32-33 says the TOC and engine
		// state must SURVIVE a bus reset, and only playback stops.
		.clk(clk), .rst(sys_rst), .bus_rst(rst),
		.mounted(mounted), .img_mounted(img_mounted), .img_blocks(img_blocks),
		.cmd_stb(ca_cmd_stb), .cmd_op(cmd[0]),
		.cdb1(cmd[1]), .cdb2(cmd[2]), .cdb3(cmd[3]), .cdb4(cmd[4]),
		.cdb5(cmd[5]), .cdb6(cmd[6]), .cdb7(cmd[7]), .cdb8(cmd[8]), .cdb9(cmd[9]),
		.read_stb(ca_read_stb), .eject_stb(ca_eject_stb),
		.ap_ch0(cd_ap_ch0), .ap_vol0(cd_ap_vol0),
		.ap_ch1(cd_ap_ch1), .ap_vol1(cd_ap_vol1),
		.ch_grant(ca_grant),
		.ca_io_active(ca_io_active), .ca_io_rd(ca_io_rd_w), .ca_io_lba(ca_io_lba),
		.io_ack(io_ack),
		.sd_buff_addr(sd_buff_addr), .sd_buff_addr_hi(sd_buff_addr_hi),
		.sd_buff_dout(sd_buff_dout), .sd_buff_wr(sd_buff_wr),
		.ast_code(ca_ast_code), .cur_ctrl(ca_cur_ctrl), .cur_trk(ca_cur_trk),
		.abs_m(ca_abs_m), .abs_s(ca_abs_s), .abs_f(ca_abs_f),
		.rel_m(ca_rel_m), .rel_s(ca_rel_s), .rel_f(ca_rel_f),
		.toc_base(ca_toc_addr),
		.toc_q0(ca_toc_q0), .toc_q1(ca_toc_q1), .toc_q2(ca_toc_q2), .toc_q3(ca_toc_q3),
		.toc_ready(ca_toc_ready),
		.toc43_base(ca_t43_addr),
		.toc43_q0(ca_t43_q0), .toc43_q1(ca_t43_q1),
		.toc43_q2(ca_t43_q2), .toc43_q3(ca_t43_q3),
		.toc43_len(ca_t43_len),
		.toc2_base(ca_t2_addr),
		.toc2_q0(ca_t2_q0), .toc2_q1(ca_t2_q1),
		.toc2_q2(ca_t2_q2), .toc2_q3(ca_t2_q3),
		.toc2_len(ca_t2_len),
		.disc_audio(ca_disc_audio),
		.snd_l(cd_snd_l), .snd_r(cd_snd_r)
	);
end else begin : g_no_cd_audio
	assign ca_io_active = 1'b0;
	assign ca_io_rd_w   = 1'b0;
	assign ca_io_lba    = 32'd0;
	assign ca_ast_code  = 8'h05;
	assign ca_cur_ctrl  = 8'd0;
	assign ca_cur_trk   = 8'd0;
	assign {ca_abs_m, ca_abs_s, ca_abs_f} = 24'd0;
	assign {ca_rel_m, ca_rel_s, ca_rel_f} = 24'd0;
	assign {ca_toc_q0, ca_toc_q1, ca_toc_q2, ca_toc_q3} = 32'd0;
	assign ca_toc_ready = 1'b0;
	assign {ca_t43_q0, ca_t43_q1, ca_t43_q2, ca_t43_q3} = 32'd0;
	assign ca_t43_len   = 10'd0;
	assign {ca_t2_q0, ca_t2_q1, ca_t2_q2, ca_t2_q3} = 32'd0;
	assign ca_t2_len    = 10'd0;
	assign ca_disc_audio = 1'b0;
	assign cd_snd_l     = 16'sd0;
	assign cd_snd_r     = 16'sd0;
end
endgenerate

endmodule

module scsi_dpram #(parameter DATAWIDTH=8, ADDRWIDTH=9)
(
	input	                clock,

	input	[ADDRWIDTH-1:0] address_a,
	input	[DATAWIDTH-1:0] data_a,
	input	                wren_a,
	output reg [DATAWIDTH-1:0] q_a,

	input	[ADDRWIDTH-1:0] address_b,
	input	[DATAWIDTH-1:0] data_b,
	input	                wren_b,
	output reg [DATAWIDTH-1:0] q_b
);

reg [DATAWIDTH-1:0] ram[0:(1<<ADDRWIDTH)-1];

always @(posedge clock) begin
	if(wren_a) begin
		ram[address_a] <= data_a;
		q_a <= data_a;
	end else begin
		q_a <= ram[address_a];
	end
end

always @(posedge clock) begin
	if(wren_b) begin
		ram[address_b] <= data_b;
		q_b <= data_b;
	end else begin
		q_b <= ram[address_b];
	end
end

endmodule
