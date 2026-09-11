/* dcd.v - DCD (Apple HD20) device: command layer over rtl/dcd_link.v.

   Status ($03), MultiBlock Read ($00), MultiBlock Write ($01, continued as
   $41) and Write-Verify ($02/$42), protocol revision 1.2a (May 1985).
   Reply opcode = command opcode & $3F | $80; an error reply is the header
   group alone with status $81.

     Status  from Mac:   <$AA> <$81> <$B1> <$03> <5 more> <CHK>   1 group
             from drive: <$AA> <$83> <blks> <stat> <pad> <pad> <pad>
                               <332-byte identity block>
                               <4 pad> <CHK>                     49 groups

     Read    from Mac:   <$AA> <$81> <$CD> <$00> <blks> <adrH> <adrM> <adrL>
                                                              1 group
             from drive: N frames of
                         <$AA> <$80> <blks> <stat> <pad> <pad> <pad>
                               <20 tags> <512 data> <CHK>     77 groups each

     Write   from Mac:   <$AA> <$CD> <$81>
                         <op> <blks> <adrH> <adrM> <adrL> <pad>
                               <20 tags> <512 data> <CHK>     77 groups
             from drive: <$AA> <$80|op> <blks> <stat> <3 pad> <CHK>
                                                              1 group

   Identity block (offset, size, field):

       0   2  Device_Type        0
       2   2  Device_Manuf       1 (Apple)
       4   1  Device_Character   $F6, or $DE on a read-only image
       5   3  Num_Blocks         capacity - 1
       8   2  Num_Spares         0
      10   2  Num_BadBlocks      0
      12  52  Manuf_Reserved     0
      64 256  Icon               rtl/dcd_icon.vh
     320  12  drive name         Pascal string, shown by the Finder
*/
module dcd
(
	input        clk,
	input        cep,
	input        cen,

	input        _reset,

	input        ca0,
	input        ca1,
	input        ca2,
	input        _enable,

	input  [7:0] writeData,
	input        writeReq,
	output [7:0] readData,
	output       newByteReady,

	// ---- one hps_io block-device slot, see rtl/dcd_disk.v ----
	output [31:0] sd_lba,
	output        sd_rd,
	output        sd_wr,
	input         sd_ack,
	input   [7:0] sd_buff_addr,
	input  [15:0] sd_buff_dout,
	output [15:0] sd_buff_din,
	input         sd_buff_wr,

	input         img_mounted,
	input  [63:0] img_size,
	input         img_readonly,

	// CPU speed, for dcd_link's byte pacing
	input         turbo,

	// a DCD image is mounted; iwm.v steers the external port by this
	output        present
);

`include "dcd_icon.vh"

	// ---- the sector path and the mount state it publishes ----
	wire        readonly;
	wire [23:0] blockCount;
	wire        diskBusy, diskErr;
	reg  [23:0] diskLba;
	reg         diskRd, diskWr;
	wire  [7:0] bufQ;

	wire        dcdReset;
	wire [63:0] rxBuf;
	wire [3:0]  rxLen;
	wire        rxValid, rxBad;
	wire [6:0]  rxRspGroups;
	wire        rxStb;
	wire [7:0]  rxStbData;
	wire [9:0]  rxStbAddr;
	wire [9:0]  txAddr;
	reg         txReq;
	reg  [9:0]  txLen;
	wire        txBusy;
	wire        txAbort;
	reg  [7:0]  txData;

	// byte 26 of a frame is data byte 0 in both directions
	wire  [9:0] bufWrOff = rxStbAddr - 10'd26;
	wire        bufWe    = rxStb & (rxStbAddr >= 10'd26) & (rxStbAddr < 10'd538);
	wire  [8:0] bufAddr  = bufWe ? bufWrOff[8:0] : (txAddr[8:0] - 9'd26);

	dcd_disk disk
	(
		.clk(clk), ._reset(_reset),
		.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
		.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout),
		.sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
		.img_mounted(img_mounted), .img_size(img_size), .img_readonly(img_readonly),
		.present(present), .blockCount(blockCount), .readonly(readonly),
		.lba(diskLba), .rd_req(diskRd), .wr_req(diskWr),
		.busy(diskBusy), .err(diskErr),
		.buf_addr(bufAddr), .buf_q(bufQ), .buf_d(rxStbData), .buf_we(bufWe)
	);

	// /HSHK is claimed when a command is accepted and held for the whole command
	reg         txArm;

	wire [7:0]  opcode = rxBuf[7:0];

	dcd_link link
	(
		.clk(clk), .cep(cep), .cen(cen), .turbo(turbo),
		._reset(_reset),
		.ca0(ca0), .ca1(ca1), .ca2(ca2), ._enable(_enable),
		.writeData(writeData), .writeReq(writeReq),
		.readData(readData), .newByteReady(newByteReady),
		.present(present),
		.rxBuf(rxBuf), .rxLen(rxLen), .rxValid(rxValid), .rxBad(rxBad),
		.rxRspGroups(rxRspGroups),
		.rxStb(rxStb), .rxStbData(rxStbData), .rxStbAddr(rxStbAddr),
		.dcdReset(dcdReset),
		.txArm(txArm),
		.txReq(txReq), .txData(txData), .txAddr(txAddr), .txLen(txLen),
		.txBusy(txBusy), .txAbort(txAbort)
	);

	// reply payload, addressed by txAddr; offsets past the end read as zero

	// the 12-byte trailer is the drive name the Finder shows
	localparam [95:0] TRAILER = {8'd11, "MiSTer HD20"};   // Pascal string
	function [7:0] trailerChar;
		input [3:0] i;
		trailerChar = (i < 4'd12) ? TRAILER[8*(11 - i) +: 8] : 8'h00;
	endfunction

	wire [23:0] maxBlock = (blockCount == 24'd0) ? 24'd0 : (blockCount - 24'd1);

	// Device_Character: $D6 plus Writable ($20) or Write_Protected ($08)
	wire writeProtected = readonly;
	wire [7:0] deviceChar = 8'hD6 | (writeProtected ? 8'h08 : 8'h20);

	// the reply kind is latched at decode; a read answers N times
	localparam K_STATUS = 2'd0, K_READ = 2'd1, K_WRITE = 2'd2, K_ACK = 2'd3;
	reg [1:0] replyKind;
	reg [7:0] replyOp;
	reg [7:0] replyStat;
	reg [7:0] blksLeft;

	always @(*) begin
		// ---- the six-byte header, common to all four ----
		if      (txAddr == 10'd0)  txData = replyOp;
		else if (txAddr == 10'd1)  txData = (replyKind == K_READ ||
		                                     replyKind == K_WRITE) ? blksLeft
		                                                           : 8'h00;
		else if (txAddr == 10'd2)  txData = replyStat;
		else if (txAddr <  10'd6)  txData = 8'h00;  // three pads complete the header

		// ---- MultiBlock Write, and the generic ack: the header is the reply ----
		else if (replyKind == K_WRITE || replyKind == K_ACK) txData = 8'h00;

		// ---- MultiBlock Read: 20 tag bytes then the sector ----
		else if (replyKind == K_READ)
			txData = (txAddr < 10'd26)  ? 8'h00 :     // tags; see the header
			         (txAddr < 10'd538) ? bufQ  :     // 512 bytes of block
			                              8'h00;

		// ---- Status: identity block, header is 6 bytes, so identity k is 6+k ----
		else if (txAddr <  10'd8)  txData = 8'h00;              // 0  Device_Type
		else if (txAddr == 10'd8)  txData = 8'h00;              // 2  Device_Manuf
		else if (txAddr == 10'd9)  txData = 8'h01;              //    ...= 1, Apple
		else if (txAddr == 10'd10) txData = deviceChar;         // 4  Device_Character
		else if (txAddr == 10'd11) txData = maxBlock[23:16];    // 5  Num_Blocks
		else if (txAddr == 10'd12) txData = maxBlock[15:8];
		else if (txAddr == 10'd13) txData = maxBlock[7:0];
		else if (txAddr <  10'd18) txData = 8'h00;              // 8  Num_Spares
		                                                        // 10 Num_BadBlocks
		else if (txAddr <  10'd70) txData = 8'h00;              // 12 Manuf_Reserved
		else if (txAddr < 10'd326) txData = dcd_icon_byte(txAddr[7:0] - 8'd70);
		else if (txAddr < 10'd338) txData = trailerChar(txAddr[3:0] - 4'd6);

		// ---- pad to the end of the last group; the link layer adds CHK ----
		else                       txData = 8'h00;
	end

	// dispatch; unimplemented opcodes get a header-only reply of the requested length
	localparam C_IDLE  = 3'd0, C_FETCH = 3'd1, C_FETCH_GO = 3'd2,
	           C_WAIT  = 3'd3, C_SEND  = 3'd4, C_SENDING  = 3'd5;
	reg [2:0] cstate;
	reg       sending;

	// command block: <opcode><blocks><addrH><addrM><addrL><pad>
	wire  [7:0] cmdBlocks = rxBuf[15:8];
	wire [23:0] cmdLba    = {rxBuf[23:16], rxBuf[31:24], rxBuf[39:32]};
	wire  [9:0] askedLen  = rxRspGroups * 10'd7 - 10'd1;   // groups*7 - 1

	// Bit 6 is the continued-write marker; bits 5:0 are the opcode proper.
	wire  [5:0] cmdOp     = opcode[5:0];
	wire        cmdCont   = opcode[6];
	wire        cmdIsWr   = (cmdOp == 6'h01) || (cmdOp == 6'h02);

	// tested on cmdOp so the continued forms count as known
	wire        cmdKnown  = (cmdOp == 6'h00) || cmdIsWr || (cmdOp == 6'h03);

	// a write must have brought a whole sector
	reg lastLba_valid;
	reg [23:0] lastLba;
	reg wrFull;

	always @(posedge clk or negedge _reset) begin
		if (!_reset) wrFull <= 1'b0;
		else if (dcdReset) wrFull <= 1'b0;
		else if (rxStb && rxStbAddr == 10'd537) wrFull <= 1'b1;
		else if (rxValid || rxBad) wrFull <= 1'b0;
	end

	always @(posedge clk or negedge _reset) begin
		if (!_reset) begin
			txArm     <= 1'b0;
			txReq     <= 1'b0;
			txLen     <= 10'd0;
			diskRd    <= 1'b0;
			diskWr    <= 1'b0;
			diskLba   <= 24'd0;
			lastLba   <= 24'd0;
			lastLba_valid <= 1'b0;
			blksLeft  <= 8'd0;
			replyKind <= K_STATUS;
			replyOp   <= 8'h83;
			replyStat <= 8'h00;
			cstate    <= C_IDLE;
			sending   <= 1'b0;
		end
		// a DCD reset abandons the command in flight
		else if (dcdReset) begin
			txArm     <= 1'b0;
			txReq     <= 1'b0;
			diskRd    <= 1'b0;
			diskWr    <= 1'b0;
			blksLeft  <= 8'd0;
			lastLba_valid <= 1'b0;
			replyKind <= K_STATUS;
			replyOp   <= 8'h83;
			replyStat <= 8'h00;
			sending   <= 1'b0;
			cstate    <= C_IDLE;
		end
		else begin
			txReq  <= 1'b0;
			diskRd <= 1'b0;
			diskWr <= 1'b0;

			case (cstate)
			C_IDLE:
				if (rxValid && !txBusy && rxRspGroups != 7'd0) begin
					if (opcode == 8'h03) begin
						replyKind <= K_STATUS;
						replyOp   <= 8'h83;
						replyStat <= 8'h00;
						txLen     <= askedLen;
						txArm     <= 1'b1;
						lastLba_valid <= 1'b0;
						cstate    <= C_SEND;
					end
					else if (opcode == 8'h00 && cmdBlocks != 8'd0) begin
						replyKind <= K_READ;
						replyOp   <= 8'h80;
						blksLeft  <= cmdBlocks;
						diskLba   <= cmdLba;
						txLen     <= askedLen;
						txArm     <= 1'b1;
						lastLba_valid <= 1'b0;
						cstate    <= C_FETCH;
					end
					// a continued write needs an address to continue from
					else if (cmdIsWr && wrFull && (!cmdCont || lastLba_valid))
					begin
						replyKind <= K_WRITE;
						replyOp   <= {2'b10, cmdOp};
						blksLeft  <= cmdBlocks;
						diskLba   <= cmdCont ? (lastLba + 24'd1) : cmdLba;
						lastLba   <= cmdCont ? (lastLba + 24'd1) : cmdLba;
						lastLba_valid <= 1'b1;
						txLen     <= askedLen;
						txArm     <= 1'b1;
						cstate    <= C_FETCH;
					end
					// unimplemented opcode: header-only reply, success
					else if (!cmdKnown) begin
						replyKind <= K_ACK;
						replyOp   <= {2'b10, cmdOp};
						replyStat <= 8'h00;
						txLen     <= askedLen;
						txArm     <= 1'b1;
						lastLba_valid <= 1'b0;
						cstate    <= C_SEND;
					end
				end

			// read fetches, write commits; the same handshake either way
			C_FETCH: begin
				diskRd <= (replyKind == K_READ);
				diskWr <= (replyKind == K_WRITE);
				cstate <= C_FETCH_GO;
			end

			// one clock for dcd_disk to raise busy or err
			C_FETCH_GO: cstate <= C_WAIT;

			C_WAIT:
				if (!diskBusy) begin
					// refused fetch: status $81 (bit 0 is what the ROM tests); blksLeft kept
					if (diskErr) begin
						replyStat <= 8'h81;
						// a read's 77-group frame shortens to the header
						if (replyKind == K_READ) txLen <= 10'd6;
					end
					else replyStat <= 8'h00;
					cstate <= C_SEND;
				end

			C_SEND: begin
				txReq  <= 1'b1;
				cstate <= C_SENDING;
			end

			C_SENDING:
				// wait for txBusy up, then down; an abandoned frame must not arm the next block
				if (txAbort) begin
					sending <= 1'b0;
					txArm   <= 1'b0;
					cstate  <= C_IDLE;
				end
				else if (!sending) begin
					if (txBusy) sending <= 1'b1;
				end
				else if (!txBusy) begin
					sending <= 1'b0;
					// a write is one command per block; only a read continues
					if (replyKind != K_READ || replyStat[7] || blksLeft <= 8'd1)
					begin
						txArm  <= 1'b0;
						cstate <= C_IDLE;
					end
					else begin
						blksLeft <= blksLeft - 8'd1;
						diskLba  <= diskLba + 24'd1;
						cstate   <= C_FETCH;
					end
				end

			default: begin
				txArm  <= 1'b0;
				cstate <= C_IDLE;
			end
			endcase
		end
	end

endmodule
