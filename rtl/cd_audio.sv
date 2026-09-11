// cd_audio.sv - AppleCD audio playback engine for the SCSI CD-ROM target.
//
// Instantiated inside scsi.v (CDROM instance only) so it shares the target's
// HPS io channel; only the two sound outputs leave the SCSI hierarchy. Byte
// behaviour follows MAME's nscsi_cdrom_apple_device. The HPS side (TOC blob
// and raw-audio block windows) is Main_MiSTer/support/maclc/maclc_cd.h.
//
//   MAIN FSM  - TOC blob fetch and parse, 0xC1/0x43 response tables,
//               command execution (SEARCH/PLAY/PAUSE/STOP/SCAN), status
//               refresh and playhead bookkeeping
//   FETCH FSM - streams 2352-byte frames into a 2-frame ping-pong buffer
//   SAMPLE    - 44.1 kHz cadence, 16-bit LE stereo, 588 samples per frame
//
// scsi.v raises ch_grant only while the target is bus-idle; MAIN has priority
// over FETCH via fr_ok, and a data READ stops playback.

module cd_audio #(
	parameter CLK_HZ = 32'd32_500_000   // clk rate, for the 44.1 kHz cadence
)(
	input             clk,
	input             rst,       // SYSTEM reset only (survives SCSI bus resets)
	input             bus_rst,   // SCSI bus reset: stops playback, TOC/engine state SURVIVES

	input             mounted,
	input             img_mounted,      // mount pulse: (re)acquire the TOC
	input      [31:0] img_blocks,       // 512-byte blocks of the data region

	// audio-family CDB, latched by scsi.v (status GOOD already returned)
	input             cmd_stb,
	input       [7:0] cmd_op,
	input       [7:0] cdb1, cdb2, cdb3, cdb4, cdb5, cdb6, cdb7, cdb8, cdb9,
	input             read_stb,         // data READ latched: stop playback
	input             eject_stb,

	// CD Audio Control page 0x0E output ports 0/1 (MODE SELECT-writable in
	// scsi.v): channel 0x01 = left, 0x02 = right, else mute; volume 0..255
	input       [7:0] ap_ch0, ap_vol0,
	input       [7:0] ap_ch1, ap_vol1,

	// shared HPS io channel
	input             ch_grant,
	output            ca_io_active,     // owns the channel (request/ack window)
	output            ca_io_rd,
	output     [31:0] ca_io_lba,
	input             io_ack,
	input       [7:0] sd_buff_addr,
	input       [4:0] sd_buff_addr_hi,  // hps_io addr[12:8]: whole-frame bursts
	input      [15:0] sd_buff_dout,
	input             sd_buff_wr,

	// live registers for AUDIO STATUS (0xCC) / READ Q SUBCODE (0xC2)
	output reg  [7:0] ast_code,         // 0 play, 1 paused, 3 end, 5 idle
	output reg  [7:0] cur_ctrl,
	output reg  [7:0] cur_trk,               // binary, 1-based; 0xC2/0xCC serve bin2bcd of these
	output reg  [7:0] abs_m, abs_s, abs_f,   // BINARY, no +150
	output reg  [7:0] rel_m, rel_s, rel_f,

	// 0xC1 READ TOC response RAM: bytes at toc_base .. toc_base+3.
	// Layout: [0..3]   mode-00 header {01, last BCD, 00, 00}
	//         [4..7]   mode-40 lead-out {M, S, F BCD, 00}
	//         [8+4k..] track k+1 descriptor {ctrl, M, S, F BCD}, k = 0..98
	input       [8:0] toc_base,
	output      [7:0] toc_q0, toc_q1, toc_q2, toc_q3,
	output reg        toc_ready,

	// standard 0x43 READ TOC response RAM (MMC format 0, MSF form, BINARY
	// values): {u16be data-len, first=1, last=N} + 8-byte descriptors
	// {00, adr_ctrl, track#, 00, 00, M, S, F(+150)} + 0xAA lead-out row.
	// Built by M_T43_* right after the 0xC1 table; toc_ready covers both.
	input       [8:0] toc43_base,
	output      [7:0] toc43_q0, toc43_q1, toc43_q2, toc43_q3,
	output reg  [9:0] toc43_len,

	// 0x43 format-2 (full TOC, cmd[9]=0x80) response RAM, MMC4 6.40.3.4.1 BCD
	// rule (POINT/TNO/MIN/SEC/FRAME binary, PMIN/PSEC/PFRAME BCD, +150 MSF):
	//   [0..3]     {u16be dlen, first session=01, last session=01}
	//   [4+11r..]  11-byte rows A0, A1, A2, then one per track
	//   [496..507] format-1 SESSION INFO page (cmd[9]=0x40)
	input       [8:0] toc2_base,
	output      [7:0] toc2_q0, toc2_q1, toc2_q2, toc2_q3,
	output reg  [9:0] toc2_len,

	// 1 = the disc has no data track; data READs then CHECK with ILLEGAL
	// REQUEST/0x64, which the Audio CD Access extension relies on
	output reg        disc_audio,

	output reg signed [15:0] snd_l,
	output reg signed [15:0] snd_r
);

// HPS window contract (Main_MiSTer support/maclc/maclc_cd.h)
localparam [31:0] TOC_BLK   = 32'h7FFF_0000;
localparam [31:0] AUDIO_BLK = 32'h4000_0000;

// ============================================================================
// RAMs (proven scsi_dpram primitive from scsi.v)
// ============================================================================
// raw MCDA blob: 1 KB as 512 x 16 (port a = HPS stream, port b = FSM reads)
reg          blob_cap;
reg          blob_blk;
reg  [8:0]   blob_ra;
wire [15:0]  blob_q_ram;
cd_sdp #(.DW(16), .AW(9)) blob_ram (
	.clock(clk),
	.waddr({blob_blk, sd_buff_addr}), .wdata(sd_buff_dout),
	.wr(blob_cap && sd_buff_wr),
	.raddr(blob_ra), .q(blob_q_ram)
);
// cd_sdp is a one-cycle-read RAM (raddr in cycle N, q in cycle N+1); every
// blob reader is aligned to that
wire [15:0] blob_q = blob_q_ram;
wire [7:0] blob_b0 = blob_q[7:0];      // even byte (LE lane order on FPGA)
wire [7:0] blob_b1 = blob_q[15:8];

// 0xC1 response: two byte planes x 256, two read ports each. Four mirrored
// 1w1r cd_sdp instances; a constant-zero wren on scsi_dpram makes Quartus
// drop the TDP template and build registers.
reg        resp_we;
reg  [8:0] resp_wa;
reg  [7:0] resp_wd;
reg        t43_we;            // 0x43 plane write port (driven by M_T43_*)
reg  [8:0] t43_wa;
reg  [7:0] t43_wd;
wire [7:0] re_q0, re_q1, ro_q0, ro_q1;
// even byte at/after addr x lives at plane index (x + x[0]) >> 1;
// odd  byte at/after addr x lives at plane index  x >> 1
wire [8:0] tb0 = toc_base;
wire [8:0] tb2 = toc_base + 9'd2;
wire       resp_we_e = resp_we && !resp_wa[0];
wire       resp_we_o = resp_we &&  resp_wa[0];
wire [8:0] tb0_e = (tb0 + {8'd0, tb0[0]}) >> 1;
wire [8:0] tb2_e = (tb2 + {8'd0, tb2[0]}) >> 1;
cd_sdp_mlab #(.DW(8), .AW(8)) resp_e0 (
	.clock(clk), .waddr(resp_wa[8:1]), .wdata(resp_wd), .wr(resp_we_e),
	.raddr(tb0_e[7:0]), .q(re_q0)
);
cd_sdp_mlab #(.DW(8), .AW(8)) resp_e1 (
	.clock(clk), .waddr(resp_wa[8:1]), .wdata(resp_wd), .wr(resp_we_e),
	.raddr(tb2_e[7:0]), .q(re_q1)
);
cd_sdp_mlab #(.DW(8), .AW(8)) resp_o0 (
	.clock(clk), .waddr(resp_wa[8:1]), .wdata(resp_wd), .wr(resp_we_o),
	.raddr(tb0[8:1]), .q(ro_q0)
);
cd_sdp_mlab #(.DW(8), .AW(8)) resp_o1 (
	.clock(clk), .waddr(resp_wa[8:1]), .wdata(resp_wd), .wr(resp_we_o),
	.raddr(tb2[8:1]), .q(ro_q1)
);
assign toc_q0 = tb0[0] ? ro_q0 : re_q0;
assign toc_q1 = tb0[0] ? re_q0 : ro_q0;   // byte at tb0+1: opposite plane, same pair
assign toc_q2 = tb2[0] ? ro_q1 : re_q1;
assign toc_q3 = tb2[0] ? re_q1 : ro_q1;

// 0x43 response planes: identical structure to the resp planes above.
wire [7:0] t43e_q0, t43e_q1, t43o_q0, t43o_q1;
wire [8:0] t43b0 = toc43_base;
wire [8:0] t43b2 = toc43_base + 9'd2;
wire       t43_we_e = t43_we && !t43_wa[0];
wire       t43_we_o = t43_we &&  t43_wa[0];
wire [8:0] t43b0_e = (t43b0 + {8'd0, t43b0[0]}) >> 1;
wire [8:0] t43b2_e = (t43b2 + {8'd0, t43b2[0]}) >> 1;
cd_sdp_mlab #(.DW(8), .AW(8)) t43_e0 (
	.clock(clk), .waddr(t43_wa[8:1]), .wdata(t43_wd), .wr(t43_we_e),
	.raddr(t43b0_e[7:0]), .q(t43e_q0)
);
cd_sdp_mlab #(.DW(8), .AW(8)) t43_e1 (
	.clock(clk), .waddr(t43_wa[8:1]), .wdata(t43_wd), .wr(t43_we_e),
	.raddr(t43b2_e[7:0]), .q(t43e_q1)
);
cd_sdp_mlab #(.DW(8), .AW(8)) t43_o0 (
	.clock(clk), .waddr(t43_wa[8:1]), .wdata(t43_wd), .wr(t43_we_o),
	.raddr(t43b0[8:1]), .q(t43o_q0)
);
cd_sdp_mlab #(.DW(8), .AW(8)) t43_o1 (
	.clock(clk), .waddr(t43_wa[8:1]), .wdata(t43_wd), .wr(t43_we_o),
	.raddr(t43b2[8:1]), .q(t43o_q1)
);
assign toc43_q0 = t43b0[0] ? t43o_q0 : t43e_q0;
assign toc43_q1 = t43b0[0] ? t43e_q0 : t43o_q0;
assign toc43_q2 = t43b2[0] ? t43o_q1 : t43e_q1;
assign toc43_q3 = t43b2[0] ? t43e_q1 : t43o_q1;

// format-2 (full TOC) response planes: identical structure again.
reg        t2_we;             // format-2 plane write port (driven by M_T2_*)
reg  [8:0] t2_wa;
reg  [7:0] t2_wd;
wire [7:0] t2e_q0, t2e_q1, t2o_q0, t2o_q1;
wire [8:0] t2b0 = toc2_base;
wire [8:0] t2b2 = toc2_base + 9'd2;
wire       t2_we_e = t2_we && !t2_wa[0];
wire       t2_we_o = t2_we &&  t2_wa[0];
wire [8:0] t2b0_e = (t2b0 + {8'd0, t2b0[0]}) >> 1;
wire [8:0] t2b2_e = (t2b2 + {8'd0, t2b2[0]}) >> 1;
cd_sdp_mlab #(.DW(8), .AW(8)) t2_e0 (
	.clock(clk), .waddr(t2_wa[8:1]), .wdata(t2_wd), .wr(t2_we_e),
	.raddr(t2b0_e[7:0]), .q(t2e_q0)
);
cd_sdp_mlab #(.DW(8), .AW(8)) t2_e1 (
	.clock(clk), .waddr(t2_wa[8:1]), .wdata(t2_wd), .wr(t2_we_e),
	.raddr(t2b2_e[7:0]), .q(t2e_q1)
);
cd_sdp_mlab #(.DW(8), .AW(8)) t2_o0 (
	.clock(clk), .waddr(t2_wa[8:1]), .wdata(t2_wd), .wr(t2_we_o),
	.raddr(t2b0[8:1]), .q(t2o_q0)
);
cd_sdp_mlab #(.DW(8), .AW(8)) t2_o1 (
	.clock(clk), .waddr(t2_wa[8:1]), .wdata(t2_wd), .wr(t2_we_o),
	.raddr(t2b2[8:1]), .q(t2o_q1)
);
assign toc2_q0 = t2b0[0] ? t2o_q0 : t2e_q0;
assign toc2_q1 = t2b0[0] ? t2e_q0 : t2o_q0;
assign toc2_q2 = t2b2[0] ? t2o_q1 : t2e_q1;
assign toc2_q3 = t2b2[0] ? t2e_q1 : t2o_q1;


// frame ping-pong: 2 x 2048 x 16 (1176 words = one 2352 B frame per half);
// each half is one whole-frame HPS transaction, addressed via sd_buff_addr_hi
reg         fr_cap;
reg         fr_half_w;
reg [11:0]  frame_ra;
wire [15:0] frame_q;
wire [10:0] fr_wword = {sd_buff_addr_hi[2:0], sd_buff_addr};
cd_sdp #(.DW(16), .AW(12)) frame_ram (
	.clock(clk),
	.waddr({fr_half_w, fr_wword}), .wdata(sd_buff_dout),
	.wr(fr_cap && sd_buff_wr),
	.raddr(frame_ra), .q(frame_q)
);

// ============================================================================
// helpers
// ============================================================================
function [7:0] bcd2bin;
	input [7:0] b;
	bcd2bin = {4'd0, b[7:4]} * 8'd10 + {4'd0, b[3:0]};
endfunction
// bin2bcd: units must be a separate 4-bit reg; a {4-bit, 8-bit} concat is
// 12 bits and the 8-bit return truncates the tens digit (MacLC's version).
function [7:0] bin2bcd;                // 0..99
	input [7:0] v;
	reg [3:0] tens;
	reg [3:0] units;   // 4-bit: a {4-bit, 8-bit} concat would truncate the tens digit
	begin
		tens = v >= 8'd90 ? 4'd9 : v >= 8'd80 ? 4'd8 :
		       v >= 8'd70 ? 4'd7 : v >= 8'd60 ? 4'd6 :
		       v >= 8'd50 ? 4'd5 : v >= 8'd40 ? 4'd4 :
		       v >= 8'd30 ? 4'd3 : v >= 8'd20 ? 4'd2 :
		       v >= 8'd10 ? 4'd1 : 4'd0;
		// the remainder is 0..9, so narrowing it to 4 bits is exact
		units   = v - ({4'd0, tens} * 8'd10);
		bin2bcd = {tens, units};
	end
endfunction
function [31:0] msf2lba;               // BCD M/S/F -> LBA
	input [7:0] m, s, f;
	msf2lba = {24'd0, bcd2bin(m)} * 32'd4500
	        + {24'd0, bcd2bin(s)} * 32'd75
	        + {24'd0, bcd2bin(f)};
endfunction
function [31:0] msf2lba_std;           // BINARY M/S/F (+150 space) -> disc LBA
	input [7:0] m, s, f;
	reg [31:0] v;
	begin
		v = {24'd0, m} * 32'd4500 + {24'd0, s} * 32'd75 + {24'd0, f};
		msf2lba_std = (v >= 32'd150) ? v - 32'd150 : 32'd0;
	end
endfunction

// ============================================================================
// io-channel request mux (each FSM owns its own request registers)
// ============================================================================
reg         toc_rd, toc_act;
reg  [31:0] toc_lba;
reg         fr_rd, fr_act;
reg  [31:0] fr_lba;
assign ca_io_rd     = toc_rd | fr_rd;
assign ca_io_lba    = toc_act ? toc_lba : fr_lba;
assign ca_io_active = toc_act | fr_act;

reg old_ack;
always @(posedge clk) old_ack <= io_ack;
wire ack_fall = old_ack & ~io_ack;

// ============================================================================
// playback state (owned by MAIN FSM) + sample-engine handshake wires
// ============================================================================
localparam ST_IDLE = 2'd0, ST_PLAY = 2'd1, ST_PAUSE = 2'd2, ST_END = 2'd3;
reg  [1:0]  pstate;
reg [31:0]  cur_lba, stop_lba;
reg         flush;                     // 1-clk: position changed, requeue audio
wire        frame_done;                // sample engine finished a frame

always @(*) begin
	case (pstate)
	ST_PLAY:  ast_code = 8'h00;
	ST_PAUSE: ast_code = 8'h01;
	ST_END:   ast_code = 8'h03;
	default:  ast_code = 8'h05;
	endcase
end

// ============================================================================
// MAIN FSM
// ============================================================================
localparam [4:0]
	M_IDLE     = 5'd0,
	M_ACQ_REQ  = 5'd1,  M_ACQ_WAIT = 5'd2,
	M_HDR_RD   = 5'd3,                      // stream words 0..5 into hdr regs
	M_EMIT_H   = 5'd4,                      // header slot bytes 0..3
	M_LO_DIV   = 5'd5,  M_EMIT_LO  = 5'd6,  // lead-out MSF -> slots 4..7
	M_TRK_RD   = 5'd7,                      // stream entry words (4) of track t_idx
	M_TRK_DIV  = 5'd8,  M_EMIT_TRK = 5'd9,
	M_BUILT    = 5'd10,
	M_CMD      = 5'd11,                     // decode a pending command
	M_CTRK_RD  = 5'd12,                     // track-mode: read start(k), start(k+1)
	M_APPLY    = 5'd13,
	M_REF_SCAN = 5'd14,                     // find track containing cur_lba
	M_REF_DIVA = 5'd15, M_REF_DIVR = 5'd16,
	// standard 0x43 MMC TOC table build
	M_T43_HDR  = 5'd17, M_T43_RD = 5'd18, M_T43_DIV = 5'd19, M_T43_EMIT = 5'd20,
	M_T2_HDR   = 5'd21, M_T2_RD  = 5'd22, M_T2_DIV  = 5'd23, M_T2_TRK  = 5'd24,
	M_T2_A2    = 5'd25, M_T2_A01 = 5'd26, M_T2_SESS = 5'd27,
	M_SCAN_GO  = 5'd28;                     // 0xCD standard-form audio scan
reg [4:0] mst;

// 0xCD AUDIO SCAN (AppleCD player FF/RW): cdb1 0x00 = FF, 0x10 = RW, MSF
// hex in cdb3-5. While scan_x the playhead moves +-8 sectors per frame;
// cleared by any other transport command, stop, or an end.
reg        scan_x;
reg        scan_dir;                        // 1 = rewind

reg  [2:0] step;                        // word-stream step within a state
reg  [6:0] t_idx;
reg [31:0] t_start;
reg  [7:0] t_ctrl;
reg [31:0] div_v;                       // shared iterative M/S/F divider
reg  [6:0] div_m, div_s;
reg  [2:0] emit_k;   // widened for the 8-byte 0x43 descriptors

reg        toc_valid;
reg  [6:0] n_tracks;

reg        t43_lo;            // building the 0xAA lead-out row
// 0x43 build sizing (cap 60 descriptors; the last-track byte stays honest)
wire [6:0] w_t43_nreal = (n_tracks > 7'd60) ? 7'd60 : n_tracks;
wire [9:0] w_t43_dlen  = {{2'd0, w_t43_nreal} + 9'd1, 3'b000} + 10'd2;
// format-2 build sizing: rows = 3 lead-in (A0/A1/A2) + tracks, 11 B each;
// cap 41 tracks so rows end at 4+44*11=488 < the session page at [496..]
reg        t2_lo;             // building the A2 (lead-out) row
reg        t2_has_data;       // any track with the data control bit (0x04)
reg  [4:0] t2_ek;             // 0..21 (A0+A1 pass is 22 writes)
reg  [8:0] t2_wbase;          // running track-row base (4 + 11*(3+k))
reg  [7:0] t2_fctrl;          // first track's adr/ctrl (A-rows + session page)
reg  [7:0] t2_fm, t2_fs, t2_ff; // first track start MSF, BINARY (session page)
wire [6:0] w_t2_n    = (n_tracks > 7'd41) ? 7'd41 : n_tracks;
wire [9:0] w_t2_rows = {3'd0, w_t2_n} + 10'd3;
wire [9:0] w_t2_dlen = {w_t2_rows[6:0], 3'b000} + {w_t2_rows[8:0], 1'b0}
                     + w_t2_rows + 10'd2;   // rows*11 + 2
reg [31:0] leadout_lba;

reg  [7:0] c_op, c_1, c_2, c_3, c_4, c_5, c_6, c_7, c_8, c_9;
reg        cmd_pend;
reg [31:0] c_addr;                      // resolved target address
reg [31:0] c_next;                      // start of following track (track mode)
// PLAY AUDIO(10)/(12) "from current position" sentinel
wire       play_lba_ff = (c_2 == 8'hFF) && (c_3 == 8'hFF) &&
                         (c_4 == 8'hFF) && (c_5 == 8'hFF);
reg  [6:0] c_trk;                       // 0-based requested track
reg  [6:0] c_trk2;                      // 0-based index whose START bounds the play
                                        // (vendor: c_trk+1; 0x48: end-track+1)

reg [31:0] ref_abs, ref_rel;
reg [31:0] scan_start;
reg  [6:0] scan_idx, scan_best_trk;
reg  [7:0] scan_ctrl_c, scan_best_ctrl;
reg [31:0] scan_best_start;
reg  [6:0] refm_hold, refs_hold;
reg [15:0] ref_cnt;

// track entry k lives at blob words 8+4k .. 11+4k
// (bytes 16+8k: +0 ctrl, +1 resv, +2..5 start LE, +6..7 pregap)
wire [8:0] entry_w = 9'd8 + {t_idx, 2'b00};

always @(posedge clk) begin
	if (rst) begin
		mst <= M_IDLE; step <= 0; emit_k <= 0;
		t43_lo <= 0; t43_we <= 0; t43_wa <= 0; t43_wd <= 0; toc43_len <= 0;
		t2_lo <= 0; t2_we <= 0; t2_wa <= 0; t2_wd <= 0; toc2_len <= 0;
		t2_ek <= 0; t2_wbase <= 0; t2_fctrl <= 8'h14;
		t2_fm <= 0; t2_fs <= 0; t2_ff <= 0;
		t2_has_data <= 0; disc_audio <= 0;
		blob_cap <= 0; blob_blk <= 0; blob_ra <= 0;
		toc_rd <= 0; toc_act <= 0; toc_lba <= 0;
		resp_we <= 0; resp_wa <= 0; resp_wd <= 0;
		toc_valid <= 0; toc_ready <= 0; n_tracks <= 7'd1; leadout_lba <= 0;
		cmd_pend <= 0; pstate <= ST_IDLE; cur_lba <= 0; stop_lba <= 0; flush <= 0;
		cur_ctrl <= 8'h14; cur_trk <= 8'h01;
		scan_x <= 1'b0; scan_dir <= 1'b0;
		abs_m <= 0; abs_s <= 0; abs_f <= 0; rel_m <= 0; rel_s <= 0; rel_f <= 0;
		ref_cnt <= 0; t_idx <= 0;
	end else begin
		resp_we <= 1'b0;
		t43_we  <= 1'b0;
		t2_we   <= 1'b0;
		flush   <= 1'b0;

		// command capture: never lost, executed from M_IDLE
		if (cmd_stb) begin
			c_op <= cmd_op; c_1 <= cdb1; c_2 <= cdb2; c_3 <= cdb3; c_4 <= cdb4;
			c_5 <= cdb5; c_6 <= cdb6; c_7 <= cdb7; c_8 <= cdb8; c_9 <= cdb9;
			cmd_pend <= 1'b1;
		end
		// a data READ (or eject / unmount) stops playback, as on the AppleCD
		if ((read_stb || eject_stb || bus_rst || !mounted) && pstate != ST_IDLE) begin
			pstate <= ST_IDLE;
			scan_x <= 1'b0;
		end

		// playhead advance, one frame at a time (+-8 while 0xCD scanning)
		if (frame_done) begin
			if (scan_x && scan_dir) begin
				// rewind: clamp at disc start, keep scanning in place
				cur_lba <= (cur_lba > 32'd8) ? cur_lba - 32'd8 : 32'd0;
			end
			else if (cur_lba + (scan_x ? 32'd8 : 32'd1) >= stop_lba) begin
				cur_lba <= stop_lba;
				pstate  <= ST_END;
				scan_x  <= 1'b0;
			end
			else cur_lba <= cur_lba + (scan_x ? 32'd8 : 32'd1);
		end

		case (mst)
		// -------------------------------------------------- idle / dispatch
		M_IDLE: begin
			blob_cap <= 1'b0;
			if (img_mounted && mounted) begin
				toc_ready <= 1'b0; toc_valid <= 1'b0; blob_blk <= 1'b0;
				pstate <= ST_IDLE;
				mst <= M_ACQ_REQ;
			end
			else if (mounted && !toc_ready) begin
				// need-driven (re)acquisition: a PRAM auto-restart resets this FSM but
				// the img_mounted pulse never repeats, so re-arm from the mounted latch
				toc_valid <= 1'b0; blob_blk <= 1'b0;
				mst <= M_ACQ_REQ;
			end
			else if (cmd_pend) begin
				cmd_pend <= 1'b0;
				mst <= M_CMD;
			end
			else begin
				ref_cnt <= ref_cnt + 16'd1;
				if ((&ref_cnt) && toc_ready && mounted) begin
					ref_abs <= cur_lba;
					scan_idx <= 0; scan_best_trk <= 0;
					scan_best_start <= 0; scan_best_ctrl <= 8'h14;
					step <= 0;
					mst <= toc_valid ? M_REF_SCAN : M_REF_DIVA;
				end
			end
		end

		// -------------------------------------------------- TOC blob fetch
		M_ACQ_REQ:
			// unmount escape: ca_grant requires mounted
			if (!mounted) mst <= M_IDLE;
			else if (ch_grant && !fr_act) begin
			toc_lba  <= TOC_BLK + {31'd0, blob_blk};
			toc_rd   <= 1'b1;
			toc_act  <= 1'b1;
			blob_cap <= 1'b1;
			mst <= M_ACQ_WAIT;
		end
		M_ACQ_WAIT: begin
			if (!mounted && !toc_act) begin toc_rd <= 1'b0; blob_cap <= 1'b0; mst <= M_IDLE; end
			if (io_ack) toc_rd <= 1'b0;
			if (ack_fall && toc_act) begin
				toc_act  <= 1'b0;
				blob_cap <= 1'b0;
				if (!blob_blk) begin blob_blk <= 1'b1; mst <= M_ACQ_REQ; end
				else begin blob_ra <= 9'd0; step <= 0; mst <= M_HDR_RD; end
			end
		end

		// ------------------------------------------ header words 0..5 stream
		// 1-cycle RAM: ra=0 presented on entry, so word k is in blob_q at
		// step k+1 (ra advances every cycle). Same contract as M_TRK_RD.
		M_HDR_RD: begin
			blob_ra <= blob_ra + 9'd1;
			step <= step + 3'd1;
			case (step)
			3'd1: toc_valid <= (blob_b0 == "M") && (blob_b1 == "C");       // word0
			3'd2: if (!((blob_b0 == "D") && (blob_b1 == "A"))) toc_valid <= 1'b0; // word1
			3'd3: ;                                                        // word2: version/first
			3'd4: begin                                                    // word3: {data_trk, last}
				if (toc_valid)
					n_tracks <= (blob_b0 == 8'd0) ? 7'd1 :
					            (blob_b0 > 8'd99) ? 7'd99 : blob_b0[6:0];
			end
			3'd5: if (toc_valid) leadout_lba[15:0]  <= blob_q;             // word4
			3'd6: begin                                                    // word5
				if (toc_valid) leadout_lba[31:16] <= blob_q;
				else begin
					n_tracks    <= 7'd1;
					leadout_lba <= {2'd0, img_blocks[31:2]};               // 2048-blocks
				end
				step <= 0; emit_k <= 0;
				mst <= M_EMIT_H;
			end
			default: ;
			endcase
		end

		// ------------------------------------------ response header [0..3]
		M_EMIT_H: begin
			resp_we <= 1'b1;
			emit_k  <= emit_k + 3'd1;
			case (emit_k)
			3'd0: begin resp_wa <= 9'd0; resp_wd <= 8'h01; end
			3'd1: begin resp_wa <= 9'd1; resp_wd <= bin2bcd({1'b0, n_tracks}); end
			3'd2: begin resp_wa <= 9'd2; resp_wd <= 8'h00; end
			default: begin
				resp_wa <= 9'd3; resp_wd <= 8'h00;
				div_v <= leadout_lba; div_m <= 0; div_s <= 0;
				mst <= M_LO_DIV;
			end
			endcase
		end
		M_LO_DIV: begin
			if ((div_v >= 32'd4500) && (div_m != 7'd99)) begin
				div_v <= div_v - 32'd4500; div_m <= div_m + 7'd1;
			end
			else if (div_v >= 32'd75) begin
				div_v <= div_v - 32'd75; div_s <= div_s + 7'd1;
			end
			else begin emit_k <= 0; mst <= M_EMIT_LO; end
		end
		M_EMIT_LO: begin
			resp_we <= 1'b1;
			emit_k  <= emit_k + 3'd1;
			case (emit_k)
			3'd0: begin resp_wa <= 9'd4; resp_wd <= bin2bcd({1'b0, div_m}); end
			3'd1: begin resp_wa <= 9'd5; resp_wd <= bin2bcd({1'b0, div_s}); end
			3'd2: begin resp_wa <= 9'd6; resp_wd <= bin2bcd(div_v[7:0]); end
			default: begin
				resp_wa <= 9'd7; resp_wd <= 8'h00;
				t_idx <= 0; step <= 0;
				mst <= M_TRK_RD;
			end
			endcase
		end

		// ---------------------------------- per-track descriptors, k = 0..98
		// stream entry words +0..+2 (ctrl, start lo, start hi); clamp index
		M_TRK_RD: begin
			if (!toc_valid) begin
				// synthesized single data track at LBA 0
				t_ctrl <= 8'h14; t_start <= 32'd0;
				div_v <= 32'd0; div_m <= 0; div_s <= 0;
				mst <= M_TRK_DIV;
			end else begin
				step <= step + 3'd1;
				case (step)
				3'd0: blob_ra <= 9'd8  + {(t_idx < n_tracks ? t_idx : n_tracks - 7'd1), 2'b00};
				3'd1: blob_ra <= blob_ra + 9'd1;
				3'd2: begin t_ctrl        <= blob_b0; blob_ra <= blob_ra + 9'd1; end
				3'd3: t_start[15:0]  <= blob_q;
				default: begin
					t_start[31:16] <= blob_q;
					div_v <= {blob_q, t_start[15:0]};
					div_m <= 0; div_s <= 0; step <= 0;
					mst <= M_TRK_DIV;
				end
				endcase
			end
		end
		M_TRK_DIV: begin
			if ((div_v >= 32'd4500) && (div_m != 7'd99)) begin
				div_v <= div_v - 32'd4500; div_m <= div_m + 7'd1;
			end
			else if (div_v >= 32'd75) begin
				div_v <= div_v - 32'd75; div_s <= div_s + 7'd1;
			end
			else begin emit_k <= 0; mst <= M_EMIT_TRK; end
		end
		M_EMIT_TRK: begin
			resp_we <= 1'b1;
			emit_k  <= emit_k + 3'd1;
			case (emit_k)
			3'd0: begin resp_wa <= 9'd8  + {t_idx, 2'b00}; resp_wd <= t_ctrl; end
			3'd1: begin resp_wa <= 9'd9  + {t_idx, 2'b00}; resp_wd <= bin2bcd({1'b0, div_m}); end
			3'd2: begin resp_wa <= 9'd10 + {t_idx, 2'b00}; resp_wd <= bin2bcd({1'b0, div_s}); end
			default: begin
				resp_wa <= 9'd11 + {t_idx, 2'b00}; resp_wd <= bin2bcd(div_v[7:0]);
				if (t_idx == 7'd98) mst <= M_BUILT;
				else begin t_idx <= t_idx + 7'd1; step <= 0; mst <= M_TRK_RD; end
			end
			endcase
		end
		M_BUILT: begin
			// 0xC1 table done; build the standard 0x43 table from the same
			// blob before declaring the TOC ready (toc_ready covers both).
			emit_k <= 0;
			mst <= M_T43_HDR;
		end

		// ---------------- standard 0x43 MMC TOC table (format 0, MSF form) ----
		M_T43_HDR: begin
			t43_we <= 1'b1;
			emit_k <= emit_k + 3'd1;
			case (emit_k)
			3'd0: begin t43_wa <= 9'd0; t43_wd <= {6'd0, w_t43_dlen[9:8]}; end
			3'd1: begin t43_wa <= 9'd1; t43_wd <= w_t43_dlen[7:0]; end
			3'd2: begin t43_wa <= 9'd2; t43_wd <= 8'h01; end
			default: begin
				t43_wa <= 9'd3; t43_wd <= {1'b0, n_tracks};
				toc43_len <= w_t43_dlen + 10'd2;
				t_idx <= 0; t43_lo <= 1'b0; step <= 0;
				mst <= M_T43_RD;
			end
			endcase
		end
		M_T43_RD: begin
			if (!toc_valid) begin
				// synthesized single data track at LBA 0
				t_ctrl <= 8'h14; t_start <= 32'd0;
				div_v <= 32'd150; div_m <= 0; div_s <= 0;
				mst <= M_T43_DIV;
			end else begin
				step <= step + 3'd1;
				case (step)
				3'd0: blob_ra <= 9'd8  + {(t_idx < n_tracks ? t_idx : n_tracks - 7'd1), 2'b00};
				3'd1: blob_ra <= blob_ra + 9'd1;
				3'd2: begin t_ctrl <= blob_b0; blob_ra <= blob_ra + 9'd1; end
				3'd3: t_start[15:0] <= blob_q;
				default: begin
					t_start[31:16] <= blob_q;
					div_v <= {blob_q, t_start[15:0]} + 32'd150;
					div_m <= 0; div_s <= 0; step <= 0;
					mst <= M_T43_DIV;
				end
				endcase
			end
		end
		M_T43_DIV: begin
			if ((div_v >= 32'd4500) && (div_m != 7'd99)) begin
				div_v <= div_v - 32'd4500; div_m <= div_m + 7'd1;
			end
			else if (div_v >= 32'd75) begin
				div_v <= div_v - 32'd75; div_s <= div_s + 7'd1;
			end
			else begin emit_k <= 0; mst <= M_T43_EMIT; end
		end
		M_T43_EMIT: begin
			t43_we <= 1'b1;
			t43_wa <= 9'd4 + {(t43_lo ? w_t43_nreal : t_idx), 3'b000} + {6'd0, emit_k};
			emit_k <= emit_k + 3'd1;
			case (emit_k)
			3'd0: t43_wd <= 8'h00;
			3'd1: t43_wd <= t_ctrl;
			3'd2: t43_wd <= t43_lo ? 8'hAA : ({1'b0, t_idx} + 8'd1);
			3'd3: t43_wd <= 8'h00;
			3'd4: t43_wd <= 8'h00;
			3'd5: t43_wd <= {1'b0, div_m};
			3'd6: t43_wd <= {1'b0, div_s};
			default: begin
				t43_wd <= div_v[7:0];
				if (t43_lo) begin
					// 0x43 format-0 table done; build the format-2 (full
					// TOC) table + session-info page before toc_ready.
					t2_ek <= 0; t2_lo <= 0; t2_wbase <= 9'd37;
					t2_has_data <= 0;
					t_idx <= 0; step <= 0;
					mst <= M_T2_HDR;
				end
				else if ((t_idx + 7'd1) < w_t43_nreal) begin
					t_idx <= t_idx + 7'd1; step <= 0; mst <= M_T43_RD;
				end
				else begin
					t43_lo <= 1'b1;
					div_v <= leadout_lba + 32'd150; div_m <= 0; div_s <= 0;
					mst <= M_T43_DIV;
				end
			end
			endcase
		end

		// ---------------- format-2 FULL TOC table + session-info page --------
		// (see the toc2 port comment for the layout; +150 MSF, BCD PMSF)
		M_T2_HDR: begin
			t2_we <= 1'b1;
			t2_ek <= t2_ek + 5'd1;
			case (t2_ek)
			5'd0: begin t2_wa <= 9'd0; t2_wd <= {6'd0, w_t2_dlen[9:8]}; end
			5'd1: begin t2_wa <= 9'd1; t2_wd <= w_t2_dlen[7:0]; end
			5'd2: begin t2_wa <= 9'd2; t2_wd <= 8'h01; end
			default: begin
				t2_wa <= 9'd3; t2_wd <= 8'h01;
				t_idx <= 0; step <= 0;
				mst <= M_T2_RD;
			end
			endcase
		end
		M_T2_RD: begin
			if (!toc_valid) begin
				// synthesized single data track at LBA 0 (+150 MSF)
				t_ctrl <= 8'h14; t_start <= 32'd0;
				div_v <= 32'd150; div_m <= 0; div_s <= 0;
				mst <= M_T2_DIV;
			end else begin
				step <= step + 3'd1;
				case (step)
				3'd0: blob_ra <= 9'd8  + {(t_idx < n_tracks ? t_idx : n_tracks - 7'd1), 2'b00};
				3'd1: blob_ra <= blob_ra + 9'd1;
				3'd2: begin t_ctrl <= blob_b0; blob_ra <= blob_ra + 9'd1; end
				3'd3: t_start[15:0] <= blob_q;
				default: begin
					t_start[31:16] <= blob_q;
					div_v <= {blob_q, t_start[15:0]} + 32'd150;
					div_m <= 0; div_s <= 0; step <= 0;
					mst <= M_T2_DIV;
				end
				endcase
			end
		end
		M_T2_DIV: begin
			if ((div_v >= 32'd4500) && (div_m != 7'd99)) begin
				div_v <= div_v - 32'd4500; div_m <= div_m + 7'd1;
			end
			else if (div_v >= 32'd75) begin
				div_v <= div_v - 32'd75; div_s <= div_s + 7'd1;
			end
			else begin
				t2_ek <= 0;
				mst <= t2_lo ? M_T2_A2 : M_T2_TRK;
			end
		end
		M_T2_TRK: begin
			t2_we <= 1'b1;
			t2_wa <= t2_wbase + {4'd0, t2_ek};
			t2_ek <= t2_ek + 5'd1;
			case (t2_ek)
			5'd0: t2_wd <= 8'h01;                       // session
			5'd1: t2_wd <= t_ctrl;                      // ADR/control
			5'd2: t2_wd <= 8'h00;                       // TNO
			5'd3: t2_wd <= {1'b0, t_idx} + 8'd1;        // POINT = track#, BINARY
			5'd4, 5'd5, 5'd6, 5'd7: t2_wd <= 8'h00;     // ATIME + zero
			5'd8: t2_wd <= bin2bcd({1'b0, div_m});      // PMIN (BCD)
			5'd9: t2_wd <= bin2bcd({1'b0, div_s});      // PSEC (BCD)
			default: begin
				t2_wd <= bin2bcd(div_v[7:0]);           // PFRAME (BCD)
				t2_has_data <= t2_has_data | t_ctrl[2]; // 0x04 = data track
				if (t_idx == 7'd0) begin
					// first track: A-rows' ctrl + session-info page values
					t2_fctrl <= t_ctrl;
					t2_fm <= {1'b0, div_m}; t2_fs <= {1'b0, div_s};
					t2_ff <= div_v[7:0];
				end
				t2_wbase <= t2_wbase + 9'd11;
				if ((t_idx + 7'd1) < w_t2_n) begin
					t_idx <= t_idx + 7'd1; step <= 0; mst <= M_T2_RD;
				end
				else begin
					t2_lo <= 1'b1;
					div_v <= leadout_lba + 32'd150; div_m <= 0; div_s <= 0;
					mst <= M_T2_DIV;
				end
			end
			endcase
		end
		M_T2_A2: begin                                  // lead-out row at [26..36]
			t2_we <= 1'b1;
			t2_wa <= 9'd26 + {4'd0, t2_ek};
			t2_ek <= t2_ek + 5'd1;
			case (t2_ek)
			5'd0: t2_wd <= 8'h01;
			5'd1: t2_wd <= t2_fctrl;
			5'd2: t2_wd <= 8'h00;
			5'd3: t2_wd <= 8'hA2;                       // POINT
			5'd4, 5'd5, 5'd6, 5'd7: t2_wd <= 8'h00;
			5'd8: t2_wd <= bin2bcd({1'b0, div_m});      // lead-out M (BCD)
			5'd9: t2_wd <= bin2bcd({1'b0, div_s});
			default: begin
				t2_wd <= bin2bcd(div_v[7:0]);
				t2_ek <= 0;
				mst <= M_T2_A01;
			end
			endcase
		end
		M_T2_A01: begin                                 // A0 at [4..14], A1 at [15..25]
			t2_we <= 1'b1;
			t2_wa <= 9'd4 + {4'd0, t2_ek};
			t2_ek <= t2_ek + 5'd1;
			case (t2_ek)
			5'd0:  t2_wd <= 8'h01;
			5'd1:  t2_wd <= t2_fctrl;
			5'd2:  t2_wd <= 8'h00;
			5'd3:  t2_wd <= 8'hA0;                      // POINT: first track#
			5'd8:  t2_wd <= 8'h01;                      // PMIN = bcd(first)=01
			5'd9:  t2_wd <= 8'h00;                      // PSEC = disc type 00
			5'd11: t2_wd <= 8'h01;
			5'd12: t2_wd <= t2_fctrl;
			5'd13: t2_wd <= 8'h00;
			5'd14: t2_wd <= 8'hA1;                      // POINT: last track#
			5'd19: t2_wd <= bin2bcd({1'b0, w_t2_n});    // PMIN = bcd(last)
			5'd21: begin
				t2_wd <= 8'h00;
				t2_ek <= 0;
				mst <= M_T2_SESS;
			end
			default: t2_wd <= 8'h00;                    // ATIME/zero/pad bytes
			endcase
		end
		M_T2_SESS: begin                                // session-info page [496..507]
			t2_we <= 1'b1;
			t2_wa <= 9'd496 + {4'd0, t2_ek};
			t2_ek <= t2_ek + 5'd1;
			case (t2_ek)
			5'd0: t2_wd <= 8'h00;                       // u16be len = 10
			5'd1: t2_wd <= 8'h0A;
			5'd2: t2_wd <= 8'h01;                       // first session
			5'd3: t2_wd <= 8'h01;                       // last session
			5'd4: t2_wd <= 8'h00;
			5'd5: t2_wd <= t2_fctrl;                    // ADR/control
			5'd6: t2_wd <= 8'h01;                       // first track in last session
			5'd7: t2_wd <= 8'h00;
			5'd8: t2_wd <= 8'h00;                       // MSF form: 00,M,S,F (hex)
			5'd9: t2_wd <= t2_fm;
			5'd10: t2_wd <= t2_fs;
			default: begin
				t2_wd <= t2_ff;
				toc2_len <= w_t2_dlen + 10'd2;
				disc_audio <= toc_valid && !t2_has_data;
				toc_ready <= 1'b1;
				mst <= M_IDLE;
			end
			endcase
		end

		// -------------------------------------------------- command execute
		M_CMD: begin
			mst <= M_IDLE;   // default; overridden below
			// any transport command other than the scan itself ends a scan
			if (c_op != 8'hcd) scan_x <= 1'b0;
			case (c_op)
			8'hca: begin                                   // AUDIO PAUSE
				if (c_1 == 8'h10) begin
					if (pstate == ST_PLAY) pstate <= ST_PAUSE;
				end else if (pstate == ST_PAUSE) pstate <= ST_PLAY;
			end
			8'hcd: begin                                   // AUDIO SCAN (FF/RW)
				// standard decode only: form in cdb9[7:6], LBA cdb2-5 / MSF binary cdb3-5
				// / track cdb5; direction cdb1 0x00 = FF, 0x10 = RW
				case (c_9[7:6])
				2'b00:   c_addr <= {c_2, c_3, c_4, c_5};       // LBA form
				2'b01:   c_addr <= msf2lba_std(c_3, c_4, c_5); // MSF form
				default: c_addr <= cur_lba;  // track form: scan from current position
				endcase
				mst <= M_SCAN_GO;
			end
			8'hc8, 8'hc9, 8'hcb: begin                     // vendor SEARCH/PLAY/STOP
				case (c_9[7:6])
				2'b01: begin                               // MSF (C9: bytes 3..5, else 5..7)
					c_addr <= (c_op == 8'hc9) ? msf2lba(c_3, c_4, c_5)
					                          : msf2lba(c_5, c_6, c_7);
					mst <= M_APPLY;
				end
				2'b10: begin                               // track number (BCD byte 5)
					if (bcd2bin(c_5) == 8'd0) begin
						// track 0: stop, as the AppleCD does
						if (c_op != 8'hcb) pstate <= ST_IDLE;
					end else begin
						c_trk <= (bcd2bin(c_5) - 8'd1 < 8'd99) ? bcd2bin(c_5) - 8'd1 : 7'd98;
						c_trk2 <= (bcd2bin(c_5) < 8'd99) ? bcd2bin(c_5) : 7'd99;
						step  <= 0;
						mst   <= toc_valid ? M_CTRK_RD : M_APPLY;
						if (!toc_valid) begin c_addr <= 32'd0; c_next <= leadout_lba; end
					end
				end
				default: begin                             // LBA (big-endian 2..5)
					c_addr <= {c_2, c_3, c_4, c_5};
					mst <= M_APPLY;
				end
				endcase
			end
			// ---- standard SCSI-2 audio set ----
			8'h47: begin                                   // PLAY AUDIO MSF (hex, +150)
				// start FF:FF:FF = play from the current position (the driver resumes
				// from pause this way)
				c_addr <= (c_3 == 8'hFF && c_4 == 8'hFF && c_5 == 8'hFF)
				          ? cur_lba : msf2lba_std(c_3, c_4, c_5);
				c_next <= msf2lba_std(c_6, c_7, c_8);
				mst <= M_APPLY;
			end
			8'h48: begin                                   // PLAY AUDIO TRACK/INDEX (binary)
				if (c_4 == 8'd0) pstate <= ST_IDLE;
				else begin
					c_trk  <= (c_4 - 8'd1 < 8'd99) ? c_4 - 8'd1 : 7'd98;
					c_trk2 <= (c_7 < 8'd99) ? c_7[6:0] : 7'd99;   // stop at start of end+1
					step   <= 0;
					mst    <= toc_valid ? M_CTRK_RD : M_APPLY;
					if (!toc_valid) begin c_addr <= 32'd0; c_next <= leadout_lba; end
				end
			end
			8'h45, 8'ha5: begin                            // PLAY AUDIO(10)/(12), LBA form
				// LBA 0xFFFFFFFF = play from the current position; length 0 = seek only
				// (the zero-length arm below). Length in frames: (10) cdb7..8, (12) cdb6..9.
				c_addr <= play_lba_ff ? cur_lba : {c_2, c_3, c_4, c_5};
				c_next <= (play_lba_ff ? cur_lba : {c_2, c_3, c_4, c_5}) +
				          ((c_op == 8'h45) ? {16'd0, c_7, c_8}
				                           : {c_6, c_7, c_8, c_9});
				mst <= M_APPLY;
			end
			8'h4b:                                         // PAUSE/RESUME (cdb8 bit0 = resume)
				if (c_8[0]) begin
					if (pstate == ST_PAUSE) pstate <= ST_PLAY;
				end else if (pstate == ST_PLAY) pstate <= ST_PAUSE;
			8'h4e:                                         // STOP PLAY
				if (pstate != ST_IDLE) pstate <= ST_IDLE;
			8'h01, 8'h0b, 8'h2b:                           // REZERO (player STOP) / SEEK(6/10)
				if (pstate != ST_IDLE) pstate <= ST_IDLE;  // Annex-C stop-audio (BlueSCSI)
			default: ;                                     // 0xCE handled in scsi.v
			endcase
		end
		// track mode: read start(k) then start(k+1), or leadout when last; data
		// arrives two states after the address, as M_HDR_RD/M_TRK_RD
		M_CTRK_RD: begin
			step <= step + 3'd1;
			case (step)
			3'd0: blob_ra <= 9'd9 + {(c_trk < n_tracks ? c_trk : n_tracks - 7'd1), 2'b00};
			3'd1: blob_ra <= blob_ra + 9'd1;
			3'd2: begin
				c_addr[15:0] <= blob_q;                    // start(k) lo
				blob_ra <= 9'd9 + {(c_trk2 < n_tracks ? c_trk2 : n_tracks - 7'd1), 2'b00};
			end
			3'd3: begin
				c_addr[31:16] <= blob_q;                   // start(k) hi
				blob_ra <= blob_ra + 9'd1;
			end
			3'd4: c_next[15:0] <= blob_q;                  // start(k+1) lo
			default: begin
				c_next[31:16] <= blob_q;                   // start(k+1) hi
				if (c_trk2 >= n_tracks) c_next <= leadout_lba;
				step <= 0;
				mst <= M_APPLY;
			end
			endcase
		end
		M_SCAN_GO: begin
			cur_lba  <= c_addr;
			flush    <= 1'b1;
			scan_x   <= 1'b1;
			scan_dir <= c_1[4];
			// FF needs headroom past a stale stop position
			if (!c_1[4] && stop_lba <= c_addr) stop_lba <= leadout_lba;
			pstate   <= ST_PLAY;
			mst <= M_IDLE;
		end
		M_APPLY: begin
			mst <= M_IDLE;
			case (c_op)
			8'hcb: begin                                   // STOP: set stop position
				stop_lba <= (c_9[7:6] == 2'b10) ? c_next : c_addr;
				if ((pstate == ST_PLAY || pstate == ST_PAUSE) &&
				    (cur_lba >= ((c_9[7:6] == 2'b10) ? c_next : c_addr)))
					pstate <= ST_IDLE;
			end
			8'hc9: begin                                   // PLAY
				if (c_1[4]) begin
					// address is the END; start stays at current position
					stop_lba <= c_addr;
					if (cur_lba < c_addr) pstate <= ST_PLAY;
				end else begin
					cur_lba <= c_addr;
					flush   <= 1'b1;
					pstate  <= ST_PLAY;
					if (c_9[7:6] == 2'b10) stop_lba <= leadout_lba;
					else if (stop_lba <= c_addr) stop_lba <= leadout_lba;
				end
			end
			8'h47, 8'h48, 8'h45, 8'ha5: begin              // standard range play
				if (c_addr == c_next) begin
					// zero-length play = seek only (SCSI-2): move the position, keep pstate
					cur_lba <= c_addr;
					flush   <= 1'b1;
				end else begin
					cur_lba  <= c_addr;
					stop_lba <= c_next;
					flush    <= 1'b1;
					pstate   <= (c_addr < c_next) ? ST_PLAY : ST_IDLE;
				end
			end
			default: begin                                 // SEARCH (0xC8) / SCAN (0xCD, v1 = seek)
				cur_lba <= c_addr;
				flush   <= 1'b1;
				if (c_9[7:6] == 2'b10) stop_lba <= c_next;
				else if (stop_lba <= c_addr) stop_lba <= leadout_lba;
				pstate  <= c_1[4] ? ST_PLAY : ST_PAUSE;
			end
			endcase
		end

		// -------------------------------- status refresh (track + abs/rel MSF)
		M_REF_SCAN: begin
			step <= step + 3'd1;
			case (step)
			3'd0: blob_ra <= 9'd8 + {scan_idx, 2'b00};
			3'd1: blob_ra <= blob_ra + 9'd1;
			3'd2: begin scan_ctrl_c <= blob_b0; blob_ra <= blob_ra + 9'd1; end
			3'd3: scan_start[15:0] <= blob_q;
			default: begin
				if ({blob_q, scan_start[15:0]} <= ref_abs) begin
					scan_best_start <= {blob_q, scan_start[15:0]};
					scan_best_trk   <= scan_idx;
					scan_best_ctrl  <= scan_ctrl_c;
				end
				step <= 0;
				if ((scan_idx + 7'd1) < n_tracks) scan_idx <= scan_idx + 7'd1;
				else begin
					div_v <= ref_abs; div_m <= 0; div_s <= 0;
					mst <= M_REF_DIVA;
				end
			end
			endcase
		end
		M_REF_DIVA: begin
			if (!toc_valid && step == 3'd0) begin
				// fallback path enters here directly: single data track at 0
				scan_best_start <= 32'd0; scan_best_trk <= 7'd0; scan_best_ctrl <= 8'h14;
				div_v <= ref_abs; div_m <= 0; div_s <= 0;
				step <= 3'd1;
			end
			else if ((div_v >= 32'd4500) && (div_m != 7'd99)) begin
				div_v <= div_v - 32'd4500; div_m <= div_m + 7'd1;
			end
			else if (div_v >= 32'd75) begin
				div_v <= div_v - 32'd75; div_s <= div_s + 7'd1;
			end
			else begin
				refm_hold <= div_m; refs_hold <= div_s;
				abs_f <= div_v[7:0];
				ref_rel <= ref_abs - scan_best_start;
				div_v <= ref_abs - scan_best_start; div_m <= 0; div_s <= 0;
				step <= 0;
				mst <= M_REF_DIVR;
			end
		end
		M_REF_DIVR: begin
			if ((div_v >= 32'd4500) && (div_m != 7'd99)) begin
				div_v <= div_v - 32'd4500; div_m <= div_m + 7'd1;
			end
			else if (div_v >= 32'd75) begin
				div_v <= div_v - 32'd75; div_s <= div_s + 7'd1;
			end
			else begin
				abs_m <= {1'b0, refm_hold};
				abs_s <= {1'b0, refs_hold};
				rel_m <= {1'b0, div_m};
				rel_s <= {1'b0, div_s};
				rel_f <= div_v[7:0];
				cur_ctrl    <= scan_best_ctrl;
				cur_trk <= {1'b0, scan_best_trk} + 8'd1;
				mst <= M_IDLE;
			end
		end
		default: mst <= M_IDLE;
		endcase
	end
end

// ============================================================================
// FETCH FSM: fill the free frame half while playing
// ============================================================================
reg  [1:0] fr_valid;
reg        fr_half_r;                  // half being played
reg  [1:0] fst;

reg [31:0] fetch_lba;
reg        fetch_sync;                 // reload fetch_lba from cur_lba
localparam F_IDLE = 2'd0, F_REQ = 2'd1, F_WAIT = 2'd2;

// sample engine tells us when it consumed a frame / which half it reads
wire       sample_consume;             // = frame_done
wire       sample_half = fr_half_r;

always @(posedge clk) begin
	if (rst) begin
		fst <= F_IDLE; fr_cap <= 0; fr_valid <= 2'b00; fr_half_w <= 0;
		fr_rd <= 0; fr_act <= 0; fr_lba <= 0; fetch_lba <= 0; fetch_sync <= 1'b1;
	end else begin
		// free the half that finished: sample_half has already flipped by the
		// clock this block sees frame_done, so index by frame_done_half_r
		if (flush) begin fr_valid <= 2'b00; fetch_sync <= 1'b1; end
		else if (frame_done) fr_valid[frame_done_half_r] <= 1'b0;

		case (fst)
		F_IDLE: begin
			fr_cap <= 1'b0;
			if (fetch_sync) begin fetch_lba <= cur_lba; fetch_sync <= 1'b0; end
			else if ((pstate == ST_PLAY) && mounted && toc_ready &&
			         !(&fr_valid) && (fetch_lba < stop_lba)) begin
				fr_half_w <= fr_valid[0] ? 1'b1 : 1'b0;
				fst <= F_REQ;
			end
		end
		F_REQ: begin
			if (flush) fst <= F_IDLE;
			else if (ch_grant && !toc_act && !toc_rd && !fr_rd && (mst == M_IDLE)) begin
				// lba = AUDIO window + disc LBA: ONE 2352-byte transaction
				// fills the whole half (Main keys blksz on this window)
				fr_lba <= AUDIO_BLK + fetch_lba;
				fr_rd  <= 1'b1;
				fr_act <= 1'b1;
				fr_cap <= 1'b1;
				fst <= F_WAIT;
			end
		end
		F_WAIT: begin
			if (io_ack) fr_rd <= 1'b0;
			if (ack_fall && fr_act) begin
				fr_act <= 1'b0;
				fr_cap <= 1'b0;
				fr_valid[fr_half_w] <= 1'b1;
				fetch_lba <= fetch_lba + 32'd1;
				fst <= F_IDLE;
			end
		end
		default: fst <= F_IDLE;
		endcase
	end
end

// ============================================================================
// SAMPLE engine: 44.1 kHz cadence; a frame = 588 stereo samples = 1176 words
// ============================================================================
reg [31:0] acc;
reg [10:0] widx;
reg  [1:0] sph;
reg        frame_done_r;
reg        frame_done_half_r;   // WHICH half just finished (captured pre-flip)
assign frame_done = frame_done_r;

// The 44.1 kHz targets are linearly interpolated on the way out, since
// sys/audio_out.sv samples with a free-running 48 kHz zero-order hold and
// the raw stair-step is audible. frac16 is a Q16 segment phase (increment
// 89 per clk, saturating, reset at each pair commit).
reg signed [15:0] snd_l_t, snd_r_t;   // cadence-tick targets (was snd_l/r)
reg signed [15:0] snd_l_p, snd_r_p;   // previous targets (segment start)
reg        [16:0] frac16;             // Q16 segment phase, saturating at 1.0

always @(posedge clk) begin
	if (rst) begin
		acc <= 0; widx <= 0; sph <= 0; frame_done_r <= 0; frame_done_half_r <= 0;
		snd_l_t <= 0; snd_r_t <= 0; snd_l_p <= 0; snd_r_p <= 0;
		frac16 <= 0; fr_half_r <= 0; frame_ra <= 0;
	end else begin
		frame_done_r <= 1'b0;
		if (!frac16[16]) frac16 <= frac16 + 17'd89;
		if (flush) begin
			widx <= 0; acc <= 0; sph <= 0; fr_half_r <= 1'b0;
		end
		else if (pstate == ST_PLAY && fr_valid[fr_half_r]) begin
			if (sph == 2'd0) begin
				acc <= acc + 32'd44_100;
				if (acc >= (CLK_HZ - 32'd44_100)) begin
					acc <= acc + 32'd44_100 - CLK_HZ;
					frame_ra <= {fr_half_r, widx};      // 1 + 11 = 12 bits
					sph <= 2'd1;
				end
			end else begin
				sph <= sph + 2'd1;
				case (sph)
				2'd1: frame_ra <= {fr_half_r, widx + 11'd1};
				2'd2: begin snd_l_p <= snd_l_t; snd_l_t <= frame_q; end
				default: begin
					snd_r_p <= snd_r_t; snd_r_t <= frame_q;
					frac16 <= 0; sph <= 2'd0;
					if (widx == 11'd1174) begin
						widx <= 0;
						fr_half_r <= ~fr_half_r;
						frame_done_r <= 1'b1;
						// nonblocking: captures the pre-flip half, the one that just finished
						frame_done_half_r <= fr_half_r;
					end else widx <= widx + 11'd2;
				end
				endcase
			end
		end
		else begin
			if (pstate != ST_PLAY) begin
				snd_l_t <= 0; snd_r_t <= 0; snd_l_p <= 0; snd_r_p <= 0;
			end
			// underrun (half not ready): hold position, emit silence
			if (pstate != ST_PLAY) acc <= 0;
		end
	end
end

// Interpolated output stage, the only driver of snd_l/snd_r. The output
// commits every 8th clk: sys/audio_out.sv accepts a value only after two
// equal consecutive captures, so a bus moving every clk_sys is rejected.
wire        [15:0] seg_f  = frac16[16] ? 16'hFFFF : frac16[15:0];
wire signed [16:0] seg_dl = {snd_l_t[15], snd_l_t} - {snd_l_p[15], snd_l_p};
wire signed [16:0] seg_dr = {snd_r_t[15], snd_r_t} - {snd_r_p[15], snd_r_p};
wire signed [33:0] seg_ml = seg_dl * $signed({1'b0, seg_f});
wire signed [33:0] seg_mr = seg_dr * $signed({1'b0, seg_f});
wire signed [16:0] sum_l  = {snd_l_p[15], snd_l_p} + $signed(seg_ml[32:16]);
wire signed [16:0] sum_r  = {snd_r_p[15], snd_r_p} + $signed(seg_mr[32:16]);
// CD Audio Control page 0x0E port scaling: source per port channel byte
// (0x01 = left, 0x02 = right, other = mute), then gain = (vol/255)^5 from
// cd_vol_lut.vh, a Q15 value measured against a real AppleCD drive. Applied
// before the 8-clk commit register.
wire signed [15:0] ap_src_l = (ap_ch0 == 8'h01) ? sum_l[15:0] :
                              (ap_ch0 == 8'h02) ? sum_r[15:0] : 16'sd0;
wire signed [15:0] ap_src_r = (ap_ch1 == 8'h02) ? sum_r[15:0] :
                              (ap_ch1 == 8'h01) ? sum_l[15:0] : 16'sd0;
`include "cd_vol_lut.vh"
wire [15:0] ap_gain_l = cd_vol_gain(ap_vol0);
wire [15:0] ap_gain_r = cd_vol_gain(ap_vol1);
wire signed [31:0] ap_scl_l = ap_src_l * $signed({1'b0, ap_gain_l});
wire signed [31:0] ap_scl_r = ap_src_r * $signed({1'b0, ap_gain_r});
reg  [2:0] odiv;
always @(posedge clk) begin
	if (rst || pstate != ST_PLAY) begin
		snd_l <= 0; snd_r <= 0; odiv <= 0;
	end else begin
		odiv <= odiv + 3'd1;
		if (odiv == 3'd0) begin
			snd_l <= ap_scl_l[30:15];
			snd_r <= ap_scl_r[30:15];
		end
	end
end

endmodule

// simple dual-port RAM (one write, one registered read) for the
// single-reader buffers above
module cd_sdp #(parameter DW = 16, AW = 12)
(
	input           clock,
	input  [AW-1:0] waddr,
	input  [DW-1:0] wdata,
	input           wr,
	input  [AW-1:0] raddr,
	output reg [DW-1:0] q
);
// forced M10K with no_rw_check, write and read in separate always blocks
// (AUTO turned the 2 Kbit planes into registers)
(* ramstyle = "M10K,no_rw_check" *) reg [DW-1:0] ram [0:(1<<AW)-1];
always @(posedge clock) if (wr) ram[waddr] <= wdata;
always @(posedge clock) q <= ram[raddr];
endmodule

// MLAB variant for the small 2 Kbit planes; same contract as cd_sdp. The
// ping-pong use never reads a plane being written, so no_rw_check is safe.
module cd_sdp_mlab #(parameter DW = 16, AW = 12)
(
	input           clock,
	input  [AW-1:0] waddr,
	input  [DW-1:0] wdata,
	input           wr,
	input  [AW-1:0] raddr,
	output reg [DW-1:0] q
);
(* ramstyle = "MLAB,no_rw_check" *) reg [DW-1:0] ram [0:(1<<AW)-1];
always @(posedge clock) if (wr) ram[waddr] <= wdata;
always @(posedge clock) q <= ram[raddr];
endmodule
