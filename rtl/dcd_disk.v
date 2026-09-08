/* dcd_disk.v - the HPS sector path behind the DCD device.

   One 512-byte sector buffer and the block-device handshake for one hps_io
   slot, plus the mount state the Status reply needs (present, capacity,
   read-only). rtl/dcd.v drives it one block at a time, because that is how
   the protocol works.

   BYTE LANES ARE NOT A FREE CHOICE. The HPS packs disk byte 0 into
   sd_buff_dout[7:0], so even bytes go in buffer0 and odd ones in buffer1 -
   the same mapping rtl/scsi.v uses. Backwards, it transposes every byte pair
   in every sector and is invisible until something reads a filesystem. This
   module is byte-addressed on the command side so the caller never has to
   think about it.

   The buffer is a local copy of scsi_dpram's shape rather than an
   instantiation: scsi_dpram lives inside rtl/scsi.v, and a DCD device has to
   work on a 512Ke, a machine defined by having no SCSI at all.

   SD_BUFF_WR IS SHARED ACROSS EVERY SLOT and must be qualified with our own
   sd_ack, or another slot's transfer writes into our sector.
   rtl/floppy_loader.v makes the same guard for the same reason.

   The ack timeout is not defensive padding: without it a stalled or absent
   HPS response leaves the drive holding /HSHK forever, which the Mac sees as
   a hung bus rather than a failed command.
*/

module dcd_disk #(
	parameter ACK_TIMEOUT_BITS = 24   // ~0.5 s at clk_sys; benches override it
) (
	input             clk,
	input             _reset,

	// ---- one hps_io block-device slot ----
	output reg [31:0] sd_lba,
	output reg        sd_rd,
	output reg        sd_wr,
	input             sd_ack,
	input       [7:0] sd_buff_addr,
	input      [15:0] sd_buff_dout,
	output     [15:0] sd_buff_din,
	input             sd_buff_wr,

	input             img_mounted,   // this slot's one-shot mount pulse
	input      [63:0] img_size,      // valid at img_mounted
	input             img_readonly,  // valid at img_mounted

	// ---- mount state, consumed by the Status reply ----
	output reg        present,
	output reg [23:0] blockCount,    // capacity in 512-byte blocks
	output reg        readonly,

	// ---- command side ----
	input      [23:0] lba,
	input             rd_req,        // one-clock pulse
	input             wr_req,        // one-clock pulse
	output            busy,
	output reg        err,           // last request failed; cleared by the next

	// ---- sector buffer, byte addressed. buf_q is REGISTERED: it follows
	//      buf_addr by one clock, like any inferred block RAM. ----
	input       [8:0] buf_addr,
	output      [7:0] buf_q,
	input       [7:0] buf_d,
	input             buf_we
);

	localparam IDLE = 2'd0, WAIT_ACK = 2'd1, WAIT_DONE = 2'd2;
	reg [1:0] state;

	assign busy = (state != IDLE);

	// ------------------------------------------------------------------
	// Mount state
	// ------------------------------------------------------------------
	// img_size is a byte count; a DCD block number is 24 bits, so anything
	// past 2^24 blocks is clamped rather than allowed to wrap to a tiny
	// capacity. HFS is the practical ceiling long before this.
	//
	// NOT RESET BY _reset, deliberately. A mounted image is HOST state: a real
	// HD20 is a separate box with its own power supply, so a Mac reset neither
	// ejects its medium nor spins it down, and rtl/scsi.v settles the same
	// question for the CD-ROM the same way. It is also the only way the drive
	// can be identified at all -- the ROM's DCD probe at $418630 runs a few
	// hundred ms into every boot, while img_mounted is a one-shot that never
	// fires again, so clearing `present` on reset guarantees present=0 at
	// probe time on every boot.
	initial begin
		present    = 1'b0;
		blockCount = 24'd0;
		readonly   = 1'b0;
	end

	always @(posedge clk) begin
		if (img_mounted) begin
			present    <= (img_size != 64'd0);
			blockCount <= (img_size[63:33] != 0) ? 24'hFFFFFF : img_size[32:9];
			readonly   <= img_readonly;
		end
	end

	// ------------------------------------------------------------------
	// Buffer
	// ------------------------------------------------------------------
	// Port A is the HPS side, a whole 16-bit word at a time. Port B is the
	// command side, one byte at a time, with the lane chosen by buf_addr[0].
	wire [7:0] buf0_qa, buf1_qa, buf0_qb, buf1_qb;
	wire       hps_we = sd_buff_wr & sd_ack;

	assign sd_buff_din = {buf1_qa, buf0_qa};

	// The lane select has to be delayed to line up with the registered read,
	// or buf_q serves the right byte of the wrong word on every other access.
	reg buf_lane_q;
	always @(posedge clk) buf_lane_q <= buf_addr[0];
	assign buf_q = buf_lane_q ? buf1_qb : buf0_qb;

	dcd_dpram buffer0
	(
		.clock(clk),
		.address_a(sd_buff_addr), .data_a(sd_buff_dout[7:0]),
		.wren_a(hps_we), .q_a(buf0_qa),
		.address_b(buf_addr[8:1]), .data_b(buf_d),
		.wren_b(buf_we & ~buf_addr[0]), .q_b(buf0_qb)
	);

	dcd_dpram buffer1
	(
		.clock(clk),
		.address_a(sd_buff_addr), .data_a(sd_buff_dout[15:8]),
		.wren_a(hps_we), .q_a(buf1_qa),
		.address_b(buf_addr[8:1]), .data_b(buf_d),
		.wren_b(buf_we & buf_addr[0]), .q_b(buf1_qb)
	);

	// ------------------------------------------------------------------
	// Block-device handshake
	// ------------------------------------------------------------------
	// A block number at or past the capacity is refused here rather than sent
	// to the HPS. TashTwenty does the same in TranslateAddr, and the protocol
	// has somewhere to report it: the reply's status byte.
	wire outOfRange = !present || (lba >= blockCount);

	reg [ACK_TIMEOUT_BITS-1:0] timeout;

	always @(posedge clk or negedge _reset) begin
		if (!_reset) begin
			state   <= IDLE;
			sd_rd   <= 1'b0;
			sd_wr   <= 1'b0;
			sd_lba  <= 32'd0;
			err     <= 1'b0;
			timeout <= 0;
		end
		else case (state)
			IDLE:
				if (rd_req || wr_req) begin
					err <= 1'b0;
					// A write to a read-only mount fails here for the same
					// reason an out-of-range block does: silently discarding
					// it would report success for data that was never stored.
					if (outOfRange || (wr_req && readonly))
						err <= 1'b1;
					else begin
						sd_lba  <= {8'd0, lba};
						sd_rd   <= rd_req;
						sd_wr   <= wr_req & ~rd_req;
						timeout <= {ACK_TIMEOUT_BITS{1'b1}};
						state   <= WAIT_ACK;
					end
				end

			WAIT_ACK:
				if (sd_ack) state <= WAIT_DONE;
				else if (timeout == 0) begin
					sd_rd <= 1'b0;
					sd_wr <= 1'b0;
					err   <= 1'b1;
					state <= IDLE;
				end
				else timeout <= timeout - 1'b1;

			// hps_io holds sd_ack for the whole transfer and drops it when the
			// sector has moved; the request lines must stay asserted until it
			// does. This is rtl/floppy_loader.v's SD_WAIT_ACK/SD_WAIT_DONE
			// split, and the split matters: sd_ack is not a pulse.
			WAIT_DONE:
				if (!sd_ack) begin
					sd_rd <= 1'b0;
					sd_wr <= 1'b0;
					state <= IDLE;
				end

			default: state <= IDLE;
		endcase
	end

endmodule

// Two-port byte RAM, the shape rtl/scsi.v's scsi_dpram already proved on this
// core. Local rather than shared; see the header.
module dcd_dpram #(parameter DATAWIDTH=8, ADDRWIDTH=8)
(
	input                      clock,

	input     [ADDRWIDTH-1:0]  address_a,
	input     [DATAWIDTH-1:0]  data_a,
	input                      wren_a,
	output reg [DATAWIDTH-1:0] q_a,

	input     [ADDRWIDTH-1:0]  address_b,
	input     [DATAWIDTH-1:0]  data_b,
	input                      wren_b,
	output reg [DATAWIDTH-1:0] q_b
);

reg [DATAWIDTH-1:0] ram[0:(1<<ADDRWIDTH)-1];

always @(posedge clock) begin
	if (wren_a) begin
		ram[address_a] <= data_a;
		q_a <= data_a;
	end
	else q_a <= ram[address_a];
end

always @(posedge clock) begin
	if (wren_b) begin
		ram[address_b] <= data_b;
		q_b <= data_b;
	end
	else q_b <= ram[address_b];
end

endmodule
