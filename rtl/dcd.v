/* dcd.v - DCD (Apple HD20) device: command layer over rtl/dcd_link.v.

   Implements Status ($03), MultiBlock Read ($00), MultiBlock Write ($01/$41)
   and Write-Verify ($02/$42) over rtl/dcd_disk.v, and answers every other
   opcode with an empty block that reports failure.

   This is the May 1985 revision of the protocol (1.2a), not the March one:
   the shipping Plus ROM implements 1.2a and asks for 332 identity bytes
   behind a six-byte header. Frame layouts below were read out of ROM
   4D1F8172 and cross-checked against TashTwenty and the HD20 firmware.

   The framing

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

   The two count bytes are $80 | groups; $419ADC builds the pair with one
   `addi.l #$810081,d0`, so a Status command is one group out ($81) and 49
   groups back ($B1). 49 is not a constant here - it is whatever the Mac
   asked for, which is what both real implementations do.

   The reply opcode is the command opcode with bit 7 set, which the ROM
   checks explicitly: $419776 does `subi.b #$80,$19C(a1)` and compares
   against the opcode it sent, erroring $30 on a mismatch.

   Four properties of the write path, none of them guessable, all read out of
   the ROM's per-block loop at $419712:

   1. Each block is a separate command, transmitted and answered in turn.
      $419712's `tst.b d1 / beq $419754` returns immediately for a read; only
      a write re-transmits. So unlike a read, the drive answers one group per
      command rather than sending N frames off one.
   2. Subsequent blocks carry the opcode with bit 6 set -- $419716's
      `ori.b #$40,$19C(a1)` -- so $01 continues as $41 and $02 as $42.
   3. The reply opcode is the command opcode masked to 6 bits, then $80.
      $41975E masks the expected opcode with `andi.b #$3F` before $419776's
      `subi.b #$80` compare, so $41 must be answered $81 and not $C1 -- and,
      just as firmly, $02 must be answered $82 and not $81. Error $30 either
      way.
   4. The drive tracks the block address itself on a continued write. The
      reply lands on top of the command block at $19C, so by the time
      $41971C rewrites the count byte the three address bytes at $19E-$1A0
      hold the previous reply's status and padding -- zeros. Only $19D is
      refreshed. A drive that trusted the address would write every block of
      a multi-block write to block 0.

   The count byte at $19D is the number of blocks still to go, counting down,
   and the reply must echo it: $41978C compares it against the Mac's own d3
   before $4197EE decrements, and a mismatch is error $31. The same applies
   to a multiblock read, whose first frame carries N.

   MultiBlock Read is N separate transmissions, not one long one. The Mac
   sends the command once, then calls its receive engine N times, each call
   hunting a fresh $AA. So the drive fetches a block, sends a whole 77-group
   frame, fetches the next, sends again.

   An error is one group, not a short data frame: txLen 6, header group only.

   The status byte is $81. Bit 0 is the bit this ROM tests - $4197DA's
   `btst #24,d0` operates on the longword at $19E, so it names bit 0 of the
   status byte, and the drive firmware agrees (`Op_Failed EQU 001h`). Bit 7
   is what the document and TashTwenty use, and is invisible to this ROM.
   Sending both means a refused write - a read-only mount, an address out of
   range - is reported as a failure to either reader.

   Write-Verify is served as a plain write, a deliberate deviation. A real
   HD20 read the block back off the platter to compare; there is no platter
   here, so the read-back would compare a buffer against itself. The status
   byte still reports a refused or timed-out write, which is the part the
   driver acts on.

   The 20 tag bytes are zeros, also a deviation. They are the file-system
   block tags of the MFS/HFS era, which a real HD20 keeps on the medium
   beside each block; a plain disc image has nowhere to store them. The Mac
   copies them to $2FC onward but the ROM does not validate them.

   The identity block, and where each value comes from:

     off  size  field              value
       0     2  Device_Type        0
       2     2  Device_Manuf       1, which 1.2a gives as "Apple = 1"
       4     1  Device_Character   $F6, or $DE on a locked image
       5     3  Num_Blocks         highest block = capacity - 1
       8     2  Num_Spares         0
      10     2  Num_BadBlocks      0
      12    52  Manuf_Reserved     0
      64   256  Icon               rtl/dcd_icon.vh
     320    12  trailer            a Pascal string -- and it is user-visible

   Device_Character is $F6 on a writable mount and $DE on a locked one. The
   fixed bits are Mountable + Readable + Ejectable + Icon_Included +
   Disk_In_Place = $D6, and exactly one of Writable ($20) and
   Write_Protected ($08) joins them. TashTwenty writes the same $F6, which is
   what confirms 1.2a's bit values are the shipping ones. Ejectable is the
   one questionable bit -- a fixed disk arguably should not claim it, and
   TashTwenty's own comment writes it "ejectable (?)" -- but $F6 is the value
   known to mount on real hardware.

   Num_Blocks is capacity minus one. The Plus ROM never reads the field --
   there is no reference to $1A2-$1AB anywhere in $419600-$419E40 -- so it
   cannot be settled from the ROM, and this follows TashTwenty, which
   decrements it. Minus one is the safe direction either way: if the Mac
   wants a count, a volume loses one block; if it wants a maximum and gets a
   count, it can address one block past the end of the image.

   Capacity is the mounted image, not the 20 MB a real unit had. The protocol
   was built for it -- capacity and block number are both 24-bit -- and the
   Floppy Emu serves up to 2 GB in HD20 mode. The ceiling is HFS, not DCD.
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
	input        lstrb,
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

	// CPU speed, passed straight to dcd_link's byte pacing. See the note on
	// its `turbo` port for why the drive has to keep step with the CPU.
	input         turbo,

	// A DCD image is mounted. rtl/iwm.v uses this to decide whether the
	// external drive port is a Sony or a DCD; with nothing mounted the port
	// behaves exactly as it always has.
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

	// Byte 26 of a frame is data byte 0 in both directions - six header bytes
	// then the 20 tags - so one offset serves the read path's txAddr and the
	// write path's receive index. The subtraction is done on the full 10-bit
	// index and truncated after, which is exact only because the buffer is
	// exactly 512 bytes.
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

	// /HSHK is claimed the moment a command is accepted, not when the sector
	// is ready: the Mac's receive routine checks the sense line with no retry
	// budget (see the txArm note in rtl/dcd_link.v). It stays up for the whole
	// command - N frames on a multi-block read - and is a reg rather than
	// `cstate != C_IDLE` so it drops on the same edge the last frame retires.
	reg         txArm;

	wire [7:0]  opcode = rxBuf[7:0];

	dcd_link link
	(
		.clk(clk), .cep(cep), .cen(cen), .turbo(turbo),
		._reset(_reset),
		.ca0(ca0), .ca1(ca1), .ca2(ca2), .lstrb(lstrb), ._enable(_enable),
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

	// ------------------------------------------------------------------
	// Reply payload, addressed by txAddr
	// ------------------------------------------------------------------
	// Offsets past the end of the identity block read as zero, so a request
	// longer than 49 groups pads and a shorter one truncates - which is what a
	// drive that trusts the count byte necessarily does.

	// The 12-byte trailer at identity offset 320. BMOW's Floppy Emu treats it
	// as frame padding and TashTwenty puts its credits there, but on a Plus it
	// is user-visible - the drive name the Finder shows, as in "Completely
	// erase disk named <volume> (MiSTer HD20)?" - so changing it changes what
	// users see. That the Mac renders it correctly, length byte and all, at
	// this exact offset also confirms the identity block is byte-accurate
	// this far in.
	function [7:0] trailerChar;
		input [3:0] i;
		begin
			case (i)
			4'd0:  trailerChar = 8'd11;   // Pascal length
			4'd1:  trailerChar = "M";
			4'd2:  trailerChar = "i";
			4'd3:  trailerChar = "S";
			4'd4:  trailerChar = "T";
			4'd5:  trailerChar = "e";
			4'd6:  trailerChar = "r";
			4'd7:  trailerChar = " ";
			4'd8:  trailerChar = "H";
			4'd9:  trailerChar = "D";
			4'd10: trailerChar = "2";
			4'd11: trailerChar = "0";
			default: trailerChar = 8'h00;
			endcase
		end
	endfunction

	wire [23:0] maxBlock = (blockCount == 24'd0) ? 24'd0 : (blockCount - 24'd1);

	// Device_Character. Everything but the write pair is fixed: Mountable,
	// Readable, Ejectable, Icon_Included, Disk_In_Place = $D6, and exactly one
	// of Writable ($20) and Write_Protected ($08) joins it.
	wire writeProtected = readonly;
	wire [7:0] deviceChar = 8'hD6 | (writeProtected ? 8'h08 : 8'h20);

	// The four replies share a six-byte header and differ after it. The kind
	// is latched when the command is decoded rather than derived from the
	// opcode register, because a read answers N times and the opcode is long
	// gone by the last of them.
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

	// ------------------------------------------------------------------
	// Dispatch
	// ------------------------------------------------------------------
	// A command arrives with its checksum already verified by the link layer.
	//
	// An opcode we do not implement is answered with an empty block, not
	// dropped. The Mac states an expected reply length with every command, so
	// a header-only group of that length is a well-formed answer to anything,
	// and it is what TashTwenty does. A logic-analyser capture of a real HD20
	// (Tashtari, 68kMLA "Deciphering DCD (Hard Disk 20)") shows the case and
	// fixes the reply opcode:
	//
	//     Mac: 19 01 00 00 00 00      Mac: 1A 00 00 00 00 00
	//     DCD: 99 00 00 00 00 00      DCD: 9A 00 00 00 00 8A
	//
	// $19 is format and $1A is verify-format, the two operations behind
	// Initialize / Erase Disk. Neither is in the 1.2a diagnostic list or in
	// the Dec-84 Nisha firmware spec -- they postdate both -- but the Plus ROM
	// specifies the wire format completely at $419D08, whose
	// `andi.b #$3f,$19c(a1)` makes the expected reply opcode (op & $3F) | $80.
	// {2'b10, cmdOp} is that byte, and it agrees with the capture on both
	// commands. Dropping $19 instead leaves Erase Disk sitting through the
	// Mac's (deliberately long, $419D18) timeout to report "Initialization
	// failed", with the drive never wedged and the disk never touched.
	//
	// The ack is bounded on purpose. Answering "fine" to work that was not
	// done is harmless for $19/$1A -- there are no sector boundaries to lay
	// down on an image file -- and it must not spread to anything that ought
	// to report a genuine failure. So it covers only opcodes with no
	// implementation at all. A command that is implemented and then refuses
	// still takes its own path: a write that did not bring a full sector, or
	// a continued write with nothing to continue from, is not answered here,
	// and the refused-write path still reports $81.
	// A reply is exactly as long as the Mac asked for. txLen excludes the
	// checksum, so groups*7-1 puts CHK in the final slot of the final group.
	localparam C_IDLE  = 3'd0, C_FETCH = 3'd1, C_FETCH_GO = 3'd2,
	           C_WAIT  = 3'd3, C_SEND  = 3'd4, C_SENDING  = 3'd5;
	reg [2:0] cstate;
	reg       sending;

	// The command block is <opcode><blocks><addrH><addrM><addrL><pad>, which
	// is TashTwenty's RC_CMDN/RC_BLKS/RC_ADRH/RC_ADRM/RC_ADRL and the same six
	// bytes the ROM prefetches from $19C in its transmit prologue.
	wire  [7:0] cmdBlocks = rxBuf[15:8];
	wire [23:0] cmdLba    = {rxBuf[23:16], rxBuf[31:24], rxBuf[39:32]};
	wire  [9:0] askedLen  = {rxRspGroups, 3'b000} - {3'b000, rxRspGroups} - 10'd1;

	// Bit 6 is the continued-write marker; bits 5:0 are the opcode proper.
	wire  [5:0] cmdOp     = opcode[5:0];
	wire        cmdCont   = opcode[6];
	wire        cmdIsWr   = (cmdOp == 6'h01) || (cmdOp == 6'h02);

	// Opcodes with a real implementation below: Read, the two Writes, Status.
	// Tested on cmdOp rather than the whole byte so a continued form ($40/$41/
	// $42) counts as known too and cannot fall through to the generic ack.
	wire        cmdKnown  = (cmdOp == 6'h00) || cmdIsWr || (cmdOp == 6'h03);

	// A write command must have brought a whole sector with it. Without this
	// a truncated or malformed frame would commit whatever the buffer happened
	// to hold - which, after a read, is a different block of the user's disk.
	// Cleared by the frame that reports itself, so it is still the previous
	// frame's verdict on the cycle rxValid is read.
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
		// A DCD reset abandons the command in flight; otherwise the FSM
		// carries on into its reply and the link holds /HSHK low in the idle
		// state waiting for a transfer the Mac gave up on (HD Diag $28).
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
					// A continued write with nothing to continue from is not a
					// command: the address would be the zeros the reply left
					// behind at $19E, so serving it would write block 0.
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
					// Anything with no implementation at all: header-only
					// group of the length asked for, success status, no disk
					// access. Bounded to unknown opcodes; see above.
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

			// Read fetches, write commits. Same three states either way; the
			// block layer's busy/err handshake is identical.
			C_FETCH: begin
				diskRd <= (replyKind == K_READ);
				diskWr <= (replyKind == K_WRITE);
				cstate <= C_FETCH_GO;
			end

			// One clock for dcd_disk to act on the request: a good one raises
			// busy, a refused one raises err without ever going busy, and
			// C_WAIT below has to be able to tell those apart.
			C_FETCH_GO: cstate <= C_WAIT;

			C_WAIT:
				if (!diskBusy) begin
					// A failed fetch is answered, not dropped. TashTwenty
					// sends "only the header group" with the status MSB set.
					//
					// $81, not $80: `$4197DA btst #24,d0` operates on the
					// longword at $19E, so the bit it names is bit 0 of the
					// status byte, and the drive's own firmware agrees
					// (`Op_Failed EQU 001h`). Bit 7 is what 1.2a and
					// TashTwenty use, and this ROM cannot see it, so a status
					// of $80 alone reports a refused write as success. $81
					// carries both readings.
					//
					// blksLeft is deliberately not reset here. The ROM checks
					// the reply's block byte against its own counter first
					// ($41978C, error $31) and only then looks at the status,
					// so zeroing it would report the wrong failure.
					// TashTwenty leaves TX_BLKS alone on its error path for
					// the same reason.
					if (diskErr) begin
						replyStat <= 8'h81;
						// A write reply is one group already, so only the
						// read's 77-group frame needs shortening.
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
				// txBusy rises a clock after txReq and falls when the frame
				// is done, so this state waits for it up then down; `sending`
				// marks the second phase.
				//
				// An abandoned frame is not a finished one and txBusy cannot
				// tell them apart, so a multiblock read must not arm the next
				// block into a bus the Mac has already left - that puts an
				// unsolicited frame on the wire. Drop txArm and return to
				// C_IDLE. txArm is a level, cleared here, one state before the
				// link's TX_IDLE can look at it again.
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
					// A write is one command per block: the Mac transmits
					// again for the next one. Only a read keeps going off a
					// single command.
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
