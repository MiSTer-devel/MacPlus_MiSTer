/* dcd_disk.v - sector buffer and hps_io block-device handshake for the DCD.

   Even bytes in buffer0, odd in buffer1 (disk byte 0 is sd_buff_dout[7:0],
   as in rtl/scsi.v). sd_buff_wr is qualified with this slot's sd_ack.
*/

module dcd_disk #(
	parameter ACK_TIMEOUT_BITS = 24   // ~0.5 s at clk_sys
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

	// sector buffer, byte addressed; buf_q follows buf_addr by one clock
	input       [8:0] buf_addr,
	output      [7:0] buf_q,
	input       [7:0] buf_d,
	input             buf_we
);

	localparam IDLE = 2'd0, WAIT_ACK = 2'd1, WAIT_DONE = 2'd2;
	reg [1:0] state;

	assign busy = (state != IDLE);

	// mount state; not reset by _reset (a mounted image survives a Mac reset)
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

	// port A: HPS, 16-bit words; port B: command side, bytes
	wire [7:0] buf0_qa, buf1_qa, buf0_qb, buf1_qb;
	wire       hps_we = sd_buff_wr & sd_ack;

	assign sd_buff_din = {buf1_qa, buf0_qa};

	// lane select delayed to match the registered read
	reg buf_lane_q;
	always @(posedge clk) buf_lane_q <= buf_addr[0];
	assign buf_q = buf_lane_q ? buf1_qb : buf0_qb;

	scsi_dpram #(.ADDRWIDTH(8)) buffer0
	(
		.clock(clk),
		.address_a(sd_buff_addr), .data_a(sd_buff_dout[7:0]),
		.wren_a(hps_we), .q_a(buf0_qa),
		.address_b(buf_addr[8:1]), .data_b(buf_d),
		.wren_b(buf_we & ~buf_addr[0]), .q_b(buf0_qb)
	);

	scsi_dpram #(.ADDRWIDTH(8)) buffer1
	(
		.clock(clk),
		.address_a(sd_buff_addr), .data_a(sd_buff_dout[15:8]),
		.wren_a(hps_we), .q_a(buf1_qa),
		.address_b(buf_addr[8:1]), .data_b(buf_d),
		.wren_b(buf_we & buf_addr[0]), .q_b(buf1_qb)
	);

	// out-of-range blocks are refused here and reported in the status byte
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
				// a write to a read-only mount is refused like an out-of-range block
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

			// sd_ack is held for the whole transfer
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
