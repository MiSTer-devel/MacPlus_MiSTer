// Mount-time floppy image loader.
//
// Floppies were previously a one-way ioctl_download blob into SDRAM. This
// module replaces that with a real SD
// block-device mount: on img_mounted it streams the whole image in via
// sd_rd, sector by sector, into the SAME SDRAM byte offsets the read side
// (dskReadAddrInt/Ext in addrController_top.v) already expects, so nothing
// downstream of SDRAM changes. Still read-only - no sd_wr, no write-back.
//
// Each sector is staged into a small local BRAM as it streams in (sd_buff_wr
// has no rate limit against on-chip RAM), then drained out to SDRAM one word
// at a time through the shared extra-slot-3 port in addrController_top.v -
// that slot recurs roughly every 2us, so draining is the slow half of a
// mount (a 1600-sector 800K image takes on the order of a second). Load-
// then-drain per sector, not double-buffered - correctness first; the gate
// here is booting exactly as before, not load speed.
//
// `done` (and therefore the caller's insertDisk latch) does not fire until
// the whole image is resident, so the Mac can never observe a disk that is
// only partially loaded - the SD-mount equivalent of the old code's
// old_down-&&-~dio_download end-of-download latch.
//
// Phase 7 adds one passenger to that stream: the MEDIUM SNIFF (media_ds
// below). Every sector already passes through the staging BRAM word by
// word, so reading four words out of sector 2 as it goes by costs three
// compares and no extra access anywhere.
module floppy_loader
(
	input         clk_sys,
	input         reset,

	input         img_mounted,  // this slot's one-shot mount pulse
	input  [63:0] img_size,     // valid at img_mounted
	input         img_readonly, // valid at img_mounted

	output reg [31:0] sd_lba,
	output reg        sd_rd,
	input              sd_ack,

	input        [7:0] sd_buff_addr,
	input       [15:0] sd_buff_dout,
	input               sd_buff_wr,

	// shared extra-slot-3 SDRAM write port (see addrController_top.v)
	output reg  [21:0] wr_addr, // byte offset within THIS image, word-aligned
	output reg  [15:0] wr_data,
	output reg          wr_req,
	input                wr_ack,

	output reg          done,          // one clk_sys pulse: image now fully resident
	output reg  [63:0]  loaded_size,   // img_size, latched at this slot's own mount
	output reg           readonly_latched,

	// What the medium says about its own sidedness, latched with `done`.
	// 1 = the volume on this image is double-sided, or the image carries no
	// volume this can recognise - the two collapse into one bit because the
	// caller ANDs it with the drive mechanism and the file size, both of
	// which are ceilings, and an unrecognisable medium is a blank diskette:
	// whatever the user formats it as. See the sniff below.
	output reg          media_ds,
	output              busy
);

localparam IDLE         = 3'd0,
           SD_ASSERT    = 3'd1,
           SD_WAIT_ACK  = 3'd2,
           SD_WAIT_DONE = 3'd3,
           DRAIN_FETCH  = 3'd4,
           DRAIN_ASSERT = 3'd5,
           DRAIN_WAIT   = 3'd6,
           DONE_PULSE   = 3'd7;
reg [2:0] state;

assign busy = (state != IDLE);

reg [15:0] buf_mem [0:255];
reg [15:0] buf_rd;
always @(posedge clk_sys) buf_rd <= buf_mem[word_idx];

reg  [7:0] word_idx;   // 0..255 within the current sector
reg [10:0] sector;     // sector index within the image (up to 1600 for 800K)
reg [10:0] nsect;      // total sectors, latched at mount

// Latch every mount request unconditionally, mirroring the UK101
// disk_reader.sv precedent - a mount
// arriving while a previous load is still draining must not be dropped.
reg mount_pending;
always @(posedge clk_sys) begin
	if (reset) mount_pending <= 1'b0;
	else if (img_mounted && img_size != 0) mount_pending <= 1'b1;
	else if (state == IDLE && mount_pending) mount_pending <= 1'b0;
end

// ---------------------------------------------------------------------
// The medium sniff
//
// Nothing on a 3.5" diskette records whether it is single- or double-
// sided; that was a certification printed on the box, and the drive
// cannot tell. What CAN be told is how big the volume last formatted onto
// it is, and that is what the .Sony driver's address-field format byte
// has to agree with. So read the volume's own size out of the Master
// Directory Block and let the medium speak for itself, instead of
// asserting a geometry from the size of the file holding it - the defect
// this phase exists to fix.
//
// The MDB is at image byte 1024, i.e. file sector 2, under BOTH the
// single- and double-sided mappings (block 2 is cylinder 0 side 0 sector
// 2 either way), which is the whole reason this can be done before the
// geometry is known. drNmAlBlks and drAlBlkSiz sit at MDB offsets 18 and
// 20 in MFS and HFS alike; only the signature word differs. Their product
// is the volume's size, and 400K vs 800K is not a close call - the
// threshold below sits halfway between them, so no real volume lands near
// it.
//
// Anything else - a zero-filled image, a non-Mac disk, a file too short
// to have a sector 2 - is "the medium says nothing", which reports
// double-sided: a blank diskette in an 800K drive is whatever the user
// chooses to format it as, and floppy.v's format latch then takes over
// the moment they choose. That fall-through is also what keeps every
// image this core already reads reading the same way: an 819,200-byte
// file whose sector 2 is not a volume header - a copy-protected title, a
// non-standard layout - goes on being addressed exactly as it is today.
localparam [15:0] MDB_SIG_MFS = 16'hD2D7;
localparam [15:0] MDB_SIG_HFS = 16'h4244;
// Volume size in 512-byte blocks, midway between a 400K volume's 800 and
// an 800K volume's 1600.
localparam [23:0] SIDEDNESS_THRESHOLD = 24'd1200;

wire [15:0] mdb_word = {sd_buff_dout[7:0], sd_buff_dout[15:8]}; // as staged above
wire        mdb_wr   = (state == SD_WAIT_DONE) && sd_buff_wr && sd_ack &&
                       (sector == 11'd2);
wire        sniff_rst = reset || (state == IDLE && mount_pending); // this mount's first cycle

reg [15:0] mdb_sig;     // word 0:      drSigWord
reg [15:0] mdb_nalbk;   // word 9:      drNmAlBlks
reg [15:0] mdb_absz_h;  // words 10-11: drAlBlkSiz, big-endian
reg [15:0] mdb_absz_l;
reg        mdb_seen;    // sector 2 went by, so the four words above are this image's

always @(posedge clk_sys) begin
	if (sniff_rst) mdb_seen <= 1'b0;
	else if (mdb_wr) begin
		case (sd_buff_addr)
		8'd0:  mdb_sig    <= mdb_word;
		8'd9:  mdb_nalbk  <= mdb_word;
		8'd10: mdb_absz_h <= mdb_word;
		8'd11: begin mdb_absz_l <= mdb_word; mdb_seen <= 1'b1; end
		default: ;
		endcase
	end
end

// An MDB worth believing. drAlBlkSiz is a non-zero multiple of 512 by
// definition and never approaches 64K on a floppy, so the bounds below are
// both a sanity test and what keeps the multiply small: the multiplier is
// then only seven bits wide.
wire mdb_ok = mdb_seen &&
              ((mdb_sig == MDB_SIG_MFS) || (mdb_sig == MDB_SIG_HFS)) &&
              (mdb_absz_h == 16'd0) && (mdb_absz_l != 16'd0) &&
              (mdb_absz_l[8:0] == 9'd0) && (mdb_nalbk != 16'd0);

// drNmAlBlks * (drAlBlkSiz / 512), in 512-byte blocks. Shift-add over
// seven cycles rather than a 16x7 array or a DSP block: the answer is
// wanted once per mount and there are hundreds of thousands of idle cycles
// left before `done`.
reg [23:0] vol_blocks;
reg [23:0] mul_cand;
reg  [6:0] mul_mult;
reg  [2:0] mul_step;
reg        mul_busy;

always @(posedge clk_sys) begin
	if (sniff_rst) begin
		mul_busy   <= 1'b0;
		vol_blocks <= 24'd0;
	end
	else if (mdb_wr && sd_buff_addr == 8'd11) begin
		vol_blocks <= 24'd0;
		mul_cand   <= {8'd0, mdb_nalbk};
		mul_mult   <= mdb_word[15:9];
		mul_step   <= 3'd0;
		mul_busy   <= 1'b1;
	end
	else if (mul_busy) begin
		if (mul_mult[0]) vol_blocks <= vol_blocks + mul_cand;
		mul_cand <= {mul_cand[22:0], 1'b0};
		mul_mult <= {1'b0, mul_mult[6:1]};
		mul_step <= mul_step + 3'd1;
		if (mul_step == 3'd6) mul_busy <= 1'b0;
	end
end

// Published with `done`, so the caller sees it at the same instant it sees
// the mount complete. Held double-sided out of reset - the fall-through
// for a medium that has said nothing yet.
always @(posedge clk_sys) begin
	if (reset) media_ds <= 1'b1;
	else if (state == DONE_PULSE)
		media_ds <= !mdb_ok || (vol_blocks > SIDEDNESS_THRESHOLD);
end

always @(posedge clk_sys) begin
	done <= 1'b0; // default; pulsed explicitly below

	if (reset) begin
		state   <= IDLE;
		sd_rd   <= 1'b0;
		wr_req  <= 1'b0;
	end else begin
		case (state)
		IDLE: if (mount_pending) begin
			loaded_size      <= img_size;
			readonly_latched <= img_readonly;
			nsect            <= img_size[19:9]; // img_size / 512
			sector           <= 11'd0;
			state            <= SD_ASSERT;
		end

		SD_ASSERT: begin
			sd_lba <= {21'd0, sector};
			sd_rd  <= 1'b1;
			state  <= SD_WAIT_ACK;
		end

		SD_WAIT_ACK: if (sd_ack) state <= SD_WAIT_DONE;

		SD_WAIT_DONE: begin
			// hps_io's sd_buff_dout and the ROM-download ioctl_dout both come
			// from the same raw HPS word (io_din in hps_io.sv) - the existing
			// ROM download path byte-swaps it before writing to SDRAM
			// (`dio_data <= {ioctl_data[7:0], ioctl_data[15:8]}` below), and
			// the read side (extra_rom_data_demux in MacPlus.sv) expects that
			// same convention. Missing this swap here silently transposes
			// every byte pair in every mounted image.
			if (sd_buff_wr && sd_ack) buf_mem[sd_buff_addr] <= {sd_buff_dout[7:0], sd_buff_dout[15:8]};
			if (!sd_ack) begin
				sd_rd    <= 1'b0;
				word_idx <= 8'd0;
				state    <= DRAIN_FETCH;
			end
		end

		// One cycle for buf_rd to catch up to buf_mem[word_idx] before the
		// first DRAIN_ASSERT reads it - buf_rd is a registered (one-cycle-
		// latency) BRAM read, so reading it on the same cycle word_idx
		// changes would serve the PREVIOUS word.
		DRAIN_FETCH: state <= DRAIN_ASSERT;

		DRAIN_ASSERT: begin
			wr_addr <= {sector, 9'd0} + {word_idx, 1'b0};
			wr_data <= buf_rd;
			wr_req  <= 1'b1;
			state   <= DRAIN_WAIT;
		end

		DRAIN_WAIT: if (wr_ack) begin
			wr_req <= 1'b0;
			if (word_idx == 8'd255) begin
				if (sector == nsect - 11'd1) state <= DONE_PULSE;
				else begin
					sector <= sector + 11'd1;
					state  <= SD_ASSERT;
				end
			end else begin
				word_idx <= word_idx + 8'd1;
				state    <= DRAIN_FETCH;
			end
		end

		DONE_PULSE: begin
			done  <= 1'b1;
			state <= IDLE;
		end

		default: state <= IDLE;
		endcase
	end
end

endmodule
