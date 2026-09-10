/* dcd_link.v - DCD (Directly Connected Disk / Apple HD20) link layer.

   This is the framing engine only: phase-line state decode, the
   identification states, /HSHK, and 7-for-8 group coding in both directions
   with the checksum. The command layer (Status, MultiBlock Read, MultiBlock
   Write) and the storage back end sit on top of it in rtl/dcd.v.

   A DCD device is a peer of floppy.v on the interface the IWM already
   provides: the corrected DB-19 pinout has DCD on the ordinary RD and WR
   pins, with /ENBL2 as /Enable. Only PH0-PH2 are repurposed, from a
   drive-register address into a 3-bit handshake state bus.
   /WrReq and HDSel are N/C, so SEL is ignored here.

   The state is {ca2,ca1,ca0} as plain binary, and the Mac changes one line at
   a time, so intermediate states are seen and must not be acted on as
   commands. Only the settled value means anything.

        7  ID: sense 1   "drive connected"
        6  ID: sense 1   (# sides, on a Sony)
        5  ID: sense 0   this is a DCD and not a Sony  <-- the discriminator
        4  RESET asserted
        3  HOST asserted, Mac sensing /HSHK
        2  idle
        1  data mode - readData carries transmitted bytes
        0  HOFF asserted

   /HSHK is idle-high, asserted-low. The Plus ROM pins that down three ways:
   the transmit entry ($419A98) errors $10 if sense is 0 at idle, then spins
   `bmi` waiting for it to fall; the end of transmission spins `bpl` waiting
   for it to rise; and the receive entry ($419820) refuses to start unless it
   is already 0.

   And it is driven from both directions. A drive that only asserts /HSHK
   when it wants to talk answers no command at all: before sending anything
   the Mac asserts HOST and spins until the drive pulls /HSHK low, giving up
   with error $11. On hardware that presents as a diagnostic reporting "Comm
   error" while the ID probe still succeeds, because identification is a
   static level and needs no handshake.

     $419A9E  read sense                 must be 1 at idle, else error $10
     $419AA6  tst.b $200(a0)  ca0=1      -> state 3, HOST asserted
     $419AB0  subq.l #1,d7 / beq         timeout -> error $11
     $419AB6  read sense
     $419ABA  bmi $419AB0                spin while SENSE == 1
     $419ABC  tst.b $400(a0)  ca1=0      -> state 1, and only now send

   TashTwenty's receiver is the mirror image and settles the release too: wait
   while state 2, require state 3, `bcf PORTC,RC4` to assert; then wait while
   state 3, require state 1, receive. When the Mac has sent everything it
   returns to state 3 -- its IntEn3 comment reads "mac is done and is waiting
   for !HSHK to be deasserted" -- and the drive releases before idling.

   Every transmitted byte has its MSB set; on this interface the MSB is the
   data-ready signal. iwm.v latches readData on newByteReady
   and clears the latch after a read, and the driver polls with `dbmi`,
   looping while the value is non-negative. A byte with the MSB clear would be
   indistinguishable from an empty latch.

   The framing is asymmetric, and both the Plus ROM and the HD20's own Z8
   firmware agree on the asymmetry:

     Mac -> drive:  <$AA> <txGroups+$81> <rxGroups+$81> then groups of 8,
                    LSB byte first followed by 7 data bytes
     drive -> Mac:  <$AA> then groups of 8, 7 data bytes followed by the
                    LSB byte last

   The two count bytes carry the group count in each direction. Only the Mac
   sends them because only the drive needs telling - the Mac computed both
   numbers before the transfer started. They appear in no published document;
   they are read out of the ROM's transmitter and confirmed independently by
   the drive firmware, which masks exactly two bytes with $7F before reaching
   its LSB byte.

   7-for-8 coding:

     transmitted[i] = $80 | (data[i] >> 1)
     lsbByte        = $80 | (L0<<0 | L1<<1 | ... | L6<<6)   Ln = data[n] & 1

   The LSB bit order is data[n] -> bit n, and it is easy to get backwards.
   The specification's Figure 1 cannot settle it: its worked example is
   $31..$37, whose LSBs are 1,0,1,0,1,0,1 and whose LSB byte is $D5 -- a
   palindrome, identical under either order. Neither can a loopback bench,
   because a reversed packing is self-consistent and invisible to the
   checksum: it only permutes which of the seven bytes each +1 lands on, and
   the sum of seven bytes does not change. Four independent sources settle it,
   two on each side of the wire:

     Plus ROM, transmit  $419A4C..$419B06  six `roxr.b #1,d4` then `roxr.b #2,d4`
                         leaves [1, L6, L5, L4, L3, L2, L1, L0]
     Plus ROM, receive   $4198C4  `lsr.b #1,d4 / addx.b d1,d1` -- the first data
                         byte takes bit 0, the second bit 1, and so on
     HD20 firmware, rx   L1dfc  `rrc R8 / rlc R9` per byte, R8 the LSB byte, so
                         again byte n takes bit n
     HD20 firmware, tx   L1edc  `scf / rrc Rn / rrc R15` seven times plus one
                         more shift and `or R15,#$80` -- Ln accumulates at bit n

   Reversed, a Status reply's first byte, $83, reaches the Mac as $82 and
   fails the `$419776 subi.b #$80` opcode compare with error $30, while the
   checksum still passes and the link looks healthy.

   Checksum: an 8-bit sum of the decoded data bytes, sent as (-sum) & $FF, so
   a receiver validates by summing everything including the checksum byte to
   zero. Stated in neither specification; read out of both halves of the ROM
   driver independently (`neg.b d5` on transmit, `beq` on the running sum at
   $419A08 on receive).
*/

module dcd_link
(
	input         clk,
	input         cep,
	input         cen,

	// CPU speed, because the byte interval has to track it. The Mac polls for
	// each byte out of a pooled budget -- $4198C0 loads d6 with 80 and every
	// `dbmi d6` in the group shares it -- and at 16 MHz it burns ~114 tries per
	// group against a fixed 16 us byte, running out mid-group and erroring $22.
	// Halve the constant rather than tick on cen16, which would put the FSM in
	// two enable domains; iwm.v's readLatchClearTimer does the same for .Sony.
	input         turbo,

	input         _reset,

	// phase lines from the IWM. PH3 (lstrb), the daisy-chain select, is
	// handled in iwm.v; a hand-over reaches this layer as _enable going high.
	input         ca0,
	input         ca1,
	input         ca2,
	input         _enable,      // /ENBL2

	// byte interface, the same shape floppy.v presents to iwm.v
	input   [7:0] writeData,
	input         writeReq,
	output  [7:0] readData,
	output reg    newByteReady,

	input         present,      // a DCD image is mounted

	// ---- command layer above ----
	// A fully received payload is presented for one clock with rxValid (good
	// checksum) or rxBad. Flattened rather than an array port so this stays
	// plain Verilog-2001; byte 0 is in the low bits, which is receive order.
	output reg [63:0] rxBuf,
	output reg  [3:0] rxLen,
	output reg        rxValid,
	output reg        rxBad,

	// How many groups the Mac has asked to receive back, from the second count
	// byte, valid alongside rxValid. A DCD drive does not decide its own reply
	// length: both real implementations size the reply from this byte alone.
	output reg  [6:0] rxRspGroups,

	// Payload bytes as they are decoded, for a caller needing more than the
	// eight rxBuf holds. rxStbAddr counts from 0 at the first byte after the two
	// count bytes, checksum included. A MultiBlock Write's first block rides
	// with the command -- 538 payload bytes -- so it cannot come out of rxBuf.
	output reg        rxStb,
	output reg  [7:0] rxStbData,
	output reg  [9:0] rxStbAddr,

	// High while the Mac holds the reset state. The command layer must abandon
	// what it was doing: a reply still queued across a reset re-raises txReq
	// afterwards, and the link then holds /HSHK waiting for a state 1 that is
	// never coming (HD Diag error $28, "asserted but never released").
	output            dcdReset,

	// The command layer raises txReq with a payload; the link adds the sync,
	// the group coding and the checksum. txLen is the payload length excluding
	// the checksum, and txLen+1 must be a multiple of 7.
	//
	// The checksum must land in the last slot of the last group, not merely
	// after the data: where a reply is shorter than the groups requested the
	// padding goes before it. The HD20 firmware emits `com R4 / inc R4` as the
	// seventh byte of the group in which its counter reaches zero, and
	// TashTwenty sums one byte less than the block "so we can write the
	// checksum to the very last byte position". That is why txLen+1 filling
	// whole groups is the wire format and not a convenience.
	//
	// Arming and starting are two different things. $419820, the first
	// instruction of the Mac's receive routine, reads the sense line with no
	// retry budget: /HSHK must already be asserted, error $20 otherwise, and
	// the Mac arrives within microseconds of its own transmission ending -
	// long before an SD card can answer. The sync byte by contrast has a
	// $10000-spin budget ($419846). So the drive claims the bus immediately on
	// accepting a command and sends when the data is ready.
	//
	//   txArm  level: a reply is coming. Asserts /HSHK and waits for state 1.
	//   txReq  pulse: the payload behind txData/txLen is ready; send it.
	input             txArm,
	input             txReq,
	input       [7:0] txData,    // payload byte selected by txAddr
	output reg  [9:0] txAddr,
	input       [9:0] txLen,
	output reg        txBusy,

	// One-clock pulse: the frame was abandoned, not finished. txBusy falls
	// either way, so without this an abandoned read looks completed and the
	// next block is armed into a bus the Mac has already left.
	output reg        txAbort
);

	// The DCD sync byte. $AA in both directions: the specification says writes
	// use $96, but no $96 exists in the DCD engine of any Plus ROM revision nor
	// in the .Sony PTCH, and the May document's own handshake section says the
	// sync "is always $AA". Requiring $96 would deadlock against Apple's driver.
	localparam [7:0] SYNC = 8'hAA;

	// 128 clk8 per byte - 2 us per bit - the rate floppy.v already uses. A real
	// cell is 2.042 us because a real Mac clocks at 7.8333 MHz, but the IWM
	// models that as its nominal 8 MHz enable, so matching floppy.v keeps DCD
	// correct relative to everything else the IWM does.
	wire [7:0] byteTicks = turbo ? 8'd64 : 8'd128;   // 8 us / 16 us

	wire [2:0] state    = {ca2, ca1, ca0};
	wire       selected = present & ~_enable;

	assign dcdReset = selected & (state == 3'd4);

	// Mac-initiated command handshake, separate from the transmit FSM: two
	// different conversations sharing one wire.
	//
	// Arming requires passing through idle first. State 3 occurs at both ends
	// of every exchange, so a drive arming on state 3 alone would re-assert
	// /HSHK the instant it finished a reply and deadlock against the Mac's
	// end-of-transmission spin.
	localparam RXH_IDLE  = 3'd0, RXH_ARMED = 3'd1, RXH_READY = 3'd2,
	           RXH_DATA  = 3'd3, RXH_DONE  = 3'd4;
	reg [2:0] rxHs;

	// A reply request is remembered, not acted on at once: the command layer
	// raises txReq while the Mac is still in state 1 finishing its send, and a
	// drive grabbing /HSHK there would never release it for the end-of-command
	// acknowledgement. TashTwenty's Transmit refuses to start unless the state
	// is 2.
	reg txPend;

	// ------------------------------------------------------------------
	// Sense
	// ------------------------------------------------------------------
	// With nothing mounted every ID state reads 1 - the "phantom states" a
	// non-chaining DCD returns past the end of the chain. A real Sony also
	// answers 1 in state 5, so an absent DCD is indistinguishable from an
	// ordinary drive.
	reg hshk_n;   // 1 = de-asserted (idle), 0 = asserted
	reg senseBit;

	always @(*) begin
		if (!selected)
			senseBit = 1'b1;
		else case (state)
			3'd7:    senseBit = 1'b1;
			3'd6:    senseBit = 1'b1;
			3'd5:    senseBit = 1'b0;
			default: senseBit = hshk_n;
		endcase
	end

	// In data mode the byte is on the bus; everywhere else bit 7 is the sense
	// line, mirroring floppy.v's dual use of readData.
	reg [7:0] txByte;
	// State 0 presents data too, not just state 1: the Mac asserts the hold-off
	// and goes on reading the rest of the group ($419926-$419964 poll for four
	// more bytes and raise error $22 if they do not come). Gating on state 1
	// alone hands those reads $00.
	assign readData = ((state == 3'd1 || state == 3'd0) && txBusy)
	                  ? txByte : {senseBit, 7'b0000000};

	// ------------------------------------------------------------------
	// Receive: Mac -> drive
	// ------------------------------------------------------------------
	// Hold-off on this direction is not a retransmission. When the Mac is
	// transmitting and finds an SCC interrupt pending it drops ca0 mid-group
	// ($419B5A, after the group's fourth byte), finishes the group anyway,
	// sends one filler $00, services the interrupt, releases HOFF, sends a
	// fresh $AA and carries on with the next group. $419BC8's `subq.w #1,d6`
	// replaces the `dbra` it skipped, so nothing is resent and the checksum
	// keeps running. The HD20 firmware receives it the same way (L1e53).
	//
	// So bytes arriving while HOFF is asserted are real payload and must be
	// taken, and the $AA that follows is a bare resync rather than the start of
	// a frame - the count bytes do not come again - hence its own state.
	//
	// It cannot fire on a Status or a Read: $419B40 clears the hold-off flag on
	// the last group, and those commands are one group long.
	localparam RX_SYNC = 3'd0, RX_CNT1 = 3'd1, RX_CNT2 = 3'd2,
	           RX_GROUP = 3'd3, RX_RESYNC = 3'd4;

	reg  [2:0] rxState;
	reg  [2:0] rxIdx;      // position within the group, 0..7
	reg  [7:0] rxLsb;
	reg  [7:0] rxSum;
	reg  [6:0] rxGroups;   // groups still to come, including the current one
	reg  [3:0] rxCount;
	reg  [9:0] rxPos;      // payload bytes taken so far in this frame
	reg        rxHoff;     // HOFF seen during the current group

	// LSB byte first on this direction, so index 0 is the LSB byte and
	// indices 1..7 are data bytes 0..6. Data byte n takes bit n; see the
	// header for the four sources that settle the order.
	wire [2:0] rxDataIdx = rxIdx - 3'd1;
	wire       rxLsbBit  = rxLsb[rxDataIdx];
	wire [7:0] rxDecoded = {writeData[6:0], rxLsbBit};
	wire [7:0] rxSumNext = rxSum + rxDecoded;

	// A frame is under way from the sync byte until the last group lands.
	wire       rxInFrame = (rxState != RX_SYNC);
	wire       rxTake    = writeReq & selected &
	                       ((state == 3'd1) | (rxInFrame & (state == 3'd0)));

	// ------------------------------------------------------------------
	// Transmit: drive -> Mac
	// ------------------------------------------------------------------
	localparam TX_IDLE = 3'd0, TX_WAIT = 3'd1, TX_SYNC = 3'd2,
	           TX_DATA = 3'd3, TX_LSB  = 3'd4, TX_END  = 3'd5,
	           TX_HOFF = 3'd6;

	reg  [2:0] txState;
	reg  [7:0] txTick;
	reg  [2:0] txIdx;        // data byte within the group, 0..6
	reg  [7:0] txLsbAcc;
	reg  [7:0] txSum;
	reg  [9:0] txSent;       // payload bytes emitted, including the checksum

	// A hold-off seen mid-group. The group is finished anyway and the flag is
	// acted on at the group boundary, which is where the Plus ROM, the HD20
	// firmware and TashTwenty all agree it may be acted on. Nothing rewinds,
	// so nothing needs saving to restore.
	reg        txHoff;

	reg        txGo;         // the payload is ready; see txArm/txReq above

	wire [9:0] txTotal  = txLen + 10'd1;              // payload + CHK
	wire       txIsChk  = (txSent == txLen);
	wire [7:0] txSource = txIsChk ? (~txSum + 8'd1) : txData;
	wire [7:0] txLsbSet = txLsbAcc | (8'd1 << txIdx);

	always @(posedge clk or negedge _reset) begin
		if (!_reset) begin
			rxState      <= RX_SYNC;
			rxIdx        <= 0;
			rxLsb        <= 0;
			rxSum        <= 0;
			rxGroups     <= 0;
			rxCount      <= 0;
			rxPos        <= 0;
			rxHoff       <= 0;
			rxRspGroups  <= 0;
			rxBuf        <= 0;
			rxLen        <= 0;
			rxValid      <= 0;
			rxBad        <= 0;
			rxStb        <= 0;
			rxStbData    <= 0;
			rxStbAddr    <= 0;
			txGo         <= 0;
			txState      <= TX_IDLE;
			txTick       <= 0;
			txIdx        <= 0;
			txLsbAcc     <= 8'h80;
			txSum        <= 0;
			txSent       <= 0;
			txAddr       <= 0;
			txHoff       <= 0;
			txByte       <= 0;
			txBusy       <= 0;
			txAbort      <= 0;
			newByteReady <= 0;
			hshk_n       <= 1'b1;
			rxHs         <= RXH_IDLE;
			txPend       <= 1'b0;
		end
		else begin
			rxValid      <= 0;
			rxBad        <= 0;
			rxStb        <= 0;
			txAbort      <= 0;

			// Cleared under cen, not on every clock. newByteReady is set below
			// inside `if (cen)` and iwm.v latches with `if (cen &&
			// newByteReady)`, so clearing unconditionally holds it only on the
			// clock after a cen tick, where the two never coincide and no reply
			// byte reaches the latch. The clear comes before the transmit FSM,
			// so a byte presented on a cen tick still overrides it.
			if (cen) newByteReady <= 0;

			// State 4 is reset: the device performs the equivalent of a
			// power-up reset. Handled here rather than folded into _reset so
			// a mounted image is not disturbed.
			if (selected && state == 3'd4) begin
				rxState <= RX_SYNC;
				rxIdx   <= 0;
				rxCount <= 0;
				rxPos   <= 0;
				rxHoff  <= 1'b0;
				txState <= TX_IDLE;
				txBusy  <= 1'b0;
				txGo    <= 1'b0;
				hshk_n  <= 1'b1;
				rxHs    <= RXH_IDLE;
				txPend  <= 1'b0;
			end
			else begin

				// ---------------- command handshake ----------------
				// Runs only while we are not the initiator; a reply drives
				// /HSHK from the transmit FSM below, whose assignments come
				// later in this block and therefore win on the cycle txReq
				// arrives.
				if (txState == TX_IDLE && !txBusy) begin
					if (!selected) begin
						hshk_n <= 1'b1;
						rxHs   <= RXH_IDLE;
					end
					else case (rxHs)
					RXH_IDLE:
						if (state == 3'd2) rxHs <= RXH_ARMED;

					RXH_ARMED:
						if (state == 3'd3) begin
							hshk_n <= 1'b0;      // "ready to receive"
							rxHs   <= RXH_READY;
							// Re-sync the byte FSM here. State 3 out of state 2
							// always starts a fresh command, so any rxState left
							// from a frame the Mac abandoned is stale; carried
							// into the new frame it never finds sync again.
							rxState <= RX_SYNC;
						end

					RXH_READY:
						if (state == 3'd1) rxHs <= RXH_DATA;
						else if (state == 3'd2) begin
							hshk_n <= 1'b1;      // Mac changed its mind
							rxHs   <= RXH_IDLE;
						end

					RXH_DATA:
						// Back to 3 means the Mac has sent everything and is
						// waiting for the release; 2 means it abandoned the
						// transfer, which TashTwenty's IntEn2 treats as an
						// abort for the same reason.
						if (state == 3'd3) begin
							hshk_n <= 1'b1;
							rxHs   <= RXH_DONE;
						end
						else if (state == 3'd2) begin
							hshk_n <= 1'b1;
							rxHs   <= RXH_IDLE;
						end

					RXH_DONE:
						if (state == 3'd2) rxHs <= RXH_IDLE;

					default: rxHs <= RXH_IDLE;
					endcase
				end

				// ---------------- receive ----------------
				// The Mac transmits in state 1 and keeps going through the
				// rest of a group after dropping to state 0 for a hold-off.
				// The hold-off latch below is set before the case, so the
				// group-boundary arm clears it cleanly.
				if (selected && rxState == RX_GROUP && state == 3'd0)
					rxHoff <= 1'b1;

				if (rxTake) begin
					case (rxState)
					RX_SYNC:
						// Hunt for the sync. Anything else is noise from a
						// transfer we were not party to.
						if (writeData == SYNC) rxState <= RX_CNT1;

					RX_CNT1: begin
						// $80 | total groups, the total including the
						// command's own group, so a command with no data
						// sends $81. Settled from the drive firmware, which
						// masks with $7F into R10 and uses `djnz R10`
						// directly as its group loop: the masked value is
						// the number of groups received.
						rxGroups <= writeData[6:0];
						rxState  <= RX_CNT2;
					end

					RX_CNT2: begin
						// The second count is how many groups the Mac wants
						// back, same $80|n form: $419ADC builds the pair with
						// one `addi.l #$810081,d0`.
						rxRspGroups <= writeData[6:0];
						rxState <= RX_GROUP;
						rxIdx   <= 0;
						rxSum   <= 0;
						rxCount <= 0;
						rxPos   <= 0;
						rxHoff  <= 1'b0;
					end

					// A bare $AA after a hold-off, with no count bytes behind
					// it. Anything else -- the Mac's filler $00, or a byte
					// still in flight -- is skipped.
					RX_RESYNC:
						if (writeData == SYNC) begin
							rxState <= RX_GROUP;
							rxIdx   <= 3'd0;
						end

					RX_GROUP:
						if (rxIdx == 3'd0) begin
							rxLsb <= writeData;
							rxIdx <= 3'd1;
						end
						else begin
							rxSum <= rxSumNext;
							rxStb     <= 1'b1;
							rxStbData <= rxDecoded;
							rxStbAddr <= rxPos;
							rxPos     <= rxPos + 10'd1;
							if (rxCount < 4'd8) begin
								rxBuf[{rxCount, 3'b000} +: 8] <= rxDecoded;
								rxCount <= rxCount + 4'd1;
							end
							if (rxIdx == 3'd7) begin
								rxIdx <= 3'd0;
								if (rxGroups <= 7'd1) begin
									// Last group. The running sum including
									// the checksum byte must be zero.
									rxState <= RX_SYNC;
									// rxCount has not yet taken this cycle's
									// byte, and saturates at 8 because rxBuf
									// holds eight; without the clamp a 77-group
									// write reports a length rxBuf cannot have.
									rxLen   <= (rxCount < 4'd8) ? (rxCount + 4'd1)
									                           : 4'd8;
									if (rxSumNext == 8'd0) rxValid <= 1'b1;
									else                   rxBad   <= 1'b1;
								end
								else begin
									rxGroups <= rxGroups - 7'd1;
									// The group is complete and counted; the
									// hold-off only costs us the byte framing,
									// so hunt the resync $AA and carry on.
									if (rxHoff) begin
										rxState <= RX_RESYNC;
										rxHoff  <= 1'b0;
									end
								end
							end
							else
								rxIdx <= rxIdx + 3'd1;
						end
					endcase
				end

				// ---------------- transmit ----------------
				// txReq can land in any state on the way in, so latch it here
				// rather than only where the frame starts.
				if (txReq) txGo <= 1'b1;

				case (txState)
				TX_IDLE: begin
					// Remember a one-clock txReq until the bus reaches state
					// 2. Do not latch from txArm as well: it is a level, still
					// high for one clock after its frame finished, and that
					// tail becomes a phantom request that grabs the bus at the
					// start of the next command.
					if (txReq && selected) txPend <= 1'b1;
					else if (!txArm && !txGo) txPend <= 1'b0;

					// Only out of TX_IDLE, and only with the bus idle: see txPend
					// above.
					if ((txReq || txArm || txPend) && selected && state == 3'd2) begin
						txPend    <= 1'b0;
						// Assert /HSHK and wait for the Mac to come round to
						// state 1. It goes 2 -> 3 -> 1, sensing us in 3.
						hshk_n    <= 1'b0;
						txBusy    <= 1'b1;
						txState   <= TX_WAIT;
						rxHs      <= RXH_IDLE;
						txSent    <= 0;
						txAddr    <= 0;
						txIdx     <= 0;
						txSum     <= 0;
						txTick    <= 0;
						txLsbAcc  <= 8'h80;
						txHoff    <= 1'b0;
					end
				end

				// Wait for state 1, but only while the Mac is plausibly on its
				// way. TashTwenty's Transmit spins on states 2 and 3 and calls
				// XAbort on anything else; without that escape /HSHK stays low
				// for ever if the Mac goes somewhere unexpected (error $28).
				TX_WAIT:
					if (state == 3'd1) begin
						txState <= TX_SYNC;
						txTick  <= byteTicks;
					end
					// States 0-3 are all legitimate: 2 and 3 are the Mac on
					// its way in, 0 is a hold-off to wait out. Only 4-7 mean
					// it has abandoned us - a reset, or the ID states as it
					// moves on to the next device.
					else if (!selected || state >= 3'd4) begin
						hshk_n  <= 1'b1;
						txBusy  <= 1'b0;
						txGo    <= 1'b0;
						txAbort <= 1'b1;
						txState <= TX_IDLE;
					end

				// Armed, on the bus, waiting for the payload: the Mac spins
				// its $10000-try sync hunt here, which is where a real drive's
				// seek time goes. State 0 is a hold-off before anything has
				// been sent and is waited out; state 2 means the Mac gave up,
				// and without that escape /HSHK stays low for ever.
				TX_SYNC:
					if (!selected || state == 3'd2 || state >= 3'd4) begin
						hshk_n  <= 1'b1;
						txBusy  <= 1'b0;
						txGo    <= 1'b0;
						txAbort <= 1'b1;
						txState <= TX_IDLE;
					end
					else if (cen && txGo) begin
						if (txTick != 0) txTick <= txTick - 8'd1;
						else begin
							txByte       <= SYNC;
							newByteReady <= 1'b1;
							txState      <= TX_DATA;
							txTick       <= byteTicks;
							txLsbAcc     <= 8'h80;
							txIdx        <= 0;
						end
					end

				// Seven data bytes, then the LSB byte, on this direction.
				// A hold-off arriving mid-group does not abandon the group.
				// 1.2a pages 4-5 say otherwise ("that group will be ignored
				// and will not be included in the checksum ... restarts
				// reading data with the group that was interrupted"), but
				// nothing real behaves that way and the prose is simply wrong:
				//
				//   Plus ROM  $41991C asserts HOFF only after four bytes of the
				//             next group are already read; $419926-$419964 go on
				//             polling for the remaining four under an ~265 us
				//             budget and raise error $22 if they stop arriving.
				//             $41997A's `subq.w #1,d7` is the decrement the
				//             skipped `dbne` would have done, so nothing is
				//             backed up, and $419998 decodes the group in hand
				//             and reads the next one - checksum included.
				//   firmware  L1f4c-L1fac tests the hold-off line only after
				//             the LSB byte of the group.
				//   TashTwenty XSuspend: "Resume transmission after interrupted
				//             group."
				//
				// The March-85 timing figure agrees in its own words at t3. So
				// remember the hold-off and keep sending; it is acted on at the
				// group boundary in TX_LSB.
				//
				// Rewinding instead stalls the transfer and then faults the
				// machine, reproducibly: the mouse quadrature sits on the SCC's
				// DCD inputs and the driver asserts HOFF from SCC RR3 at byte 1
				// of every group, so any mouse movement during a transfer
				// triggers it.
				TX_DATA: begin
					if (state == 3'd0) txHoff <= 1'b1;
					// The Mac's error exit leaves 3, then 2. Neither is
					// legitimate mid-group: the Mac reads in state 1 and only
					// goes to 3 once it has taken the whole frame, by which
					// time TX_END owns the release. Seeing either here means it
					// has walked away, and without this the transmitter went on
					// clocking bytes into a bus nobody was reading.
					if (!selected || state == 3'd2 || state == 3'd3 || state >= 3'd4) begin
						hshk_n  <= 1'b1;
						txBusy  <= 1'b0;
						txGo    <= 1'b0;
						txHoff  <= 1'b0;
						txAbort <= 1'b1;
						txState <= TX_IDLE;
					end
					else if (cen) begin
						if (txTick != 0) txTick <= txTick - 8'd1;
						else begin
							txByte       <= {1'b1, txSource[7:1]};
							txLsbAcc     <= txSource[0] ? txLsbSet : txLsbAcc;
							txSum        <= txSum + txSource;
							newByteReady <= 1'b1;
							txTick       <= byteTicks;
							txSent       <= txSent + 10'd1;
							if (!txIsChk) txAddr <= txAddr + 10'd1;
							if (txIdx == 3'd6) txState <= TX_LSB;
							else               txIdx   <= txIdx + 3'd1;
						end
					end
				end

				TX_LSB: begin
					if (state == 3'd0) txHoff <= 1'b1;
					if (!selected || state == 3'd2 || state == 3'd3 || state >= 3'd4) begin
						hshk_n  <= 1'b1;
						txBusy  <= 1'b0;
						txGo    <= 1'b0;
						txHoff  <= 1'b0;
						txAbort <= 1'b1;
						txState <= TX_IDLE;
					end
					else if (cen) begin
						if (txTick != 0) txTick <= txTick - 8'd1;
						else begin
							txByte       <= txLsbAcc;
							newByteReady <= 1'b1;
							txTick       <= byteTicks;
							txIdx        <= 0;
							txLsbAcc     <= 8'h80;
							// The group boundary, and the only place a hold-off
							// is acted on. txAddr, txSent and txSum stay put:
							// the group that just finished counts and stays in
							// the checksum. The last group is never held off
							// ($4198EC), so TX_END wins if both are true.
							if (txSent >= txTotal)                txState <= TX_END;
							else if (txHoff || state == 3'd0)     txState <= TX_HOFF;
							else                                  txState <= TX_DATA;
						end
					end
				end

				// Acknowledge the hold-off, then resume with the next group.
				// The March-85 timing figure puts the acknowledgement here and
				// nowhere else ("Rene will acknowledge the holdoff immediately
				// after the last byte of the group is sent"), and the firmware
				// does the same: release /HSHK, wait for the Mac to drop the
				// hold-off, re-assert, send a fresh $AA. The Mac hunts for that
				// sync at $419992 and then carries straight on, which is why
				// TX_SYNC is the right place to come back to - it emits the
				// sync and clears txIdx/txLsbAcc for the new group.
				//
				// The escapes matter as much as the resume. The ROM's error
				// exit leaves state 2 or 3 behind, and without these the
				// transmitter would hold /HSHK low there for ever - one half
				// of the $28 wedge.
				TX_HOFF:
					if (!selected || state >= 3'd4 || state == 3'd2 || state == 3'd3) begin
						hshk_n  <= 1'b1;
						txBusy  <= 1'b0;
						txGo    <= 1'b0;
						txHoff  <= 1'b0;
						txAbort <= 1'b1;
						txState <= TX_IDLE;
					end
					else if (state == 3'd1) begin
						hshk_n  <= 1'b0;
						txHoff  <= 1'b0;
						txState <= TX_SYNC;
						txTick  <= byteTicks;
					end
					else hshk_n <= 1'b1;

				// The `cen` here saves the last byte of every frame. readData
				// presents txByte only while txBusy is set, and the IWM
				// latches on the cen tick after the one that offered it, so
				// tearing down on the next clk drops txBusy inside that gap
				// and the CPU latches $80 in place of the final LSB byte.
				TX_END:
					if (cen) begin
						hshk_n  <= 1'b1;
						txBusy  <= 1'b0;
						txGo    <= 1'b0;
						txState <= TX_IDLE;
					end

				default: txState <= TX_IDLE;
				endcase
			end
		end
	end

endmodule
