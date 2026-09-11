/* dcd_link.v - DCD (Directly Connected Disk / Apple HD20) link layer:
   phase-line states, /HSHK, 7-for-8 group coding and checksum. The command
   layer is rtl/dcd.v.

   State {ca2,ca1,ca0}, on the external drive port with /ENBL2 as enable:

        7  ID: sense 1   drive connected
        6  ID: sense 1
        5  ID: sense 0   a DCD, not a Sony
        4  reset
        3  host asserted, Mac sensing /HSHK
        2  idle
        1  data mode: readData carries the transmitted bytes
        0  hold-off

   /HSHK is idle-high, asserted by the drive to accept a command and to
   offer a reply. Transmitted bytes have the MSB set (the IWM's data-ready).

     Mac -> drive:  <$AA> <txGroups|$80> <rxGroups|$80> then groups of 8,
                    LSB byte first followed by 7 data bytes
     drive -> Mac:  <$AA> then groups of 8, 7 data bytes then the LSB byte

     transmitted[i] = $80 | (data[i] >> 1)
     lsbByte        = $80 | (L0<<0 | L1<<1 | ... | L6<<6)   Ln = data[n] & 1
     checksum       = -(sum of data bytes), last byte of the last group
*/

module dcd_link
(
	input         clk,
	input         cep,
	input         cen,

	// byte interval tracks CPU speed (the Mac's poll budget runs out at 16 MHz)
	input         turbo,

	input         _reset,

	// phase lines; PH3 (the chain select) is handled in iwm.v
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

	// received payload, one clock, with rxValid (checksum good) or rxBad
	output reg [63:0] rxBuf,
	output reg  [3:0] rxLen,
	output reg        rxValid,
	output reg        rxBad,

	// groups the Mac asked to receive back, from the second count byte
	output reg  [6:0] rxRspGroups,

	// each decoded payload byte as it arrives (a write does not fit rxBuf)
	output reg        rxStb,
	output reg  [7:0] rxStbData,
	output reg  [9:0] rxStbAddr,

	// Mac holds the reset state; abandon any command in flight
	output            dcdReset,

	// txArm (level) claims /HSHK when a command is accepted; txReq (pulse)
	// sends txLen payload bytes then the checksum. txLen+1 is a multiple of 7.
	input             txArm,
	input             txReq,
	input       [7:0] txData,    // payload byte selected by txAddr
	output reg  [9:0] txAddr,
	input       [9:0] txLen,
	output reg        txBusy,

	// one-clock pulse: the frame was abandoned rather than finished
	output reg        txAbort
);

	// sync byte, both directions
	localparam [7:0] SYNC = 8'hAA;

	// 2 us per bit, as floppy.v
	wire [7:0] byteTicks = turbo ? 8'd64 : 8'd128;   // 8 us / 16 us

	wire [2:0] state    = {ca2, ca1, ca0};
	wire       selected = present & ~_enable;

	assign dcdReset = selected & (state == 3'd4);

	// command handshake; arming requires passing through idle (state 2)
	localparam RXH_IDLE  = 3'd0, RXH_ARMED = 3'd1, RXH_READY = 3'd2,
	           RXH_DATA  = 3'd3, RXH_DONE  = 3'd4;
	reg [2:0] rxHs;

	// a reply request is held until the bus is idle (state 2)
	reg txPend;

	// sense: with nothing mounted every ID state reads 1
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

	// In data mode the byte is on the bus; otherwise bit 7 is the sense line.
	reg [7:0] txByte;
	// state 0 too: the Mac reads on through a hold-off
	assign readData = ((state == 3'd1 || state == 3'd0) && txBusy)
	                  ? txByte : {senseBit, 7'b0000000};

	// receive, Mac -> drive. A hold-off does not restart the group: the Mac
	// sends a filler $00, then a bare $AA and the next group.
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

	// LSB byte first on this direction; data byte n takes bit n
	wire [2:0] rxDataIdx = rxIdx - 3'd1;
	wire       rxLsbBit  = rxLsb[rxDataIdx];
	wire [7:0] rxDecoded = {writeData[6:0], rxLsbBit};
	wire [7:0] rxSumNext = rxSum + rxDecoded;

	// A frame is under way from the sync byte until the last group lands.
	wire       rxInFrame = (rxState != RX_SYNC);
	wire       rxTake    = writeReq & selected &
	                       ((state == 3'd1) | (rxInFrame & (state == 3'd0)));

	// transmit, drive -> Mac
	localparam TX_IDLE = 3'd0, TX_WAIT = 3'd1, TX_SYNC = 3'd2,
	           TX_DATA = 3'd3, TX_LSB  = 3'd4, TX_END  = 3'd5,
	           TX_HOFF = 3'd6;

	reg  [2:0] txState;
	reg  [7:0] txTick;
	reg  [2:0] txIdx;        // data byte within the group, 0..6
	reg  [7:0] txLsbAcc;
	reg  [7:0] txSum;
	reg  [9:0] txSent;       // payload bytes emitted, including the checksum

	// hold-off seen mid-group; acted on at the group boundary
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

			// cleared under cen, matching the IWM's latch
			if (cen) newByteReady <= 0;

			// state 4: DCD reset (kept apart from _reset so a mounted image survives)
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

				// command handshake, Mac -> drive
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
							// a new command always starts here, so drop any stale frame
							rxState <= RX_SYNC;
						end

					RXH_READY:
						if (state == 3'd1) rxHs <= RXH_DATA;
						else if (state == 3'd2) begin
							hshk_n <= 1'b1;      // Mac changed its mind
							rxHs   <= RXH_IDLE;
						end

					RXH_DATA:
						// 3: Mac done, waiting for the release; 2: transfer abandoned
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

				// the Mac finishes a group after dropping to state 0 for a hold-off
				if (selected && rxState == RX_GROUP && state == 3'd0)
					rxHoff <= 1'b1;

				if (rxTake) begin
					case (rxState)
					RX_SYNC:
						// hunt for the sync byte
						if (writeData == SYNC) rxState <= RX_CNT1;

					RX_CNT1: begin
						// first count: $80 | total groups, including this one
						rxGroups <= writeData[6:0];
						rxState  <= RX_CNT2;
					end

					RX_CNT2: begin
						// second count: groups the Mac wants back
						rxRspGroups <= writeData[6:0];
						rxState <= RX_GROUP;
						rxIdx   <= 0;
						rxSum   <= 0;
						rxCount <= 0;
						rxPos   <= 0;
						rxHoff  <= 1'b0;
					end

					// after a hold-off the Mac sends a bare $AA, no count bytes
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
									// last group: the sum including the checksum must be zero
									rxState <= RX_SYNC;
									// rxCount saturates at 8, the size of rxBuf
									rxLen   <= (rxCount < 4'd8) ? (rxCount + 4'd1)
									                           : 4'd8;
									if (rxSumNext == 8'd0) rxValid <= 1'b1;
									else                   rxBad   <= 1'b1;
								end
								else begin
									rxGroups <= rxGroups - 7'd1;
									// after a hold-off, hunt the resync $AA
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
				// txReq can arrive in any state
				if (txReq) txGo <= 1'b1;

				case (txState)
				TX_IDLE: begin
					// hold a txReq until the bus is idle; txArm is a level and is not latched
					if (txReq && selected) txPend <= 1'b1;
					else if (!txArm && !txGo) txPend <= 1'b0;

					if ((txReq || txArm || txPend) && selected && state == 3'd2) begin
						txPend    <= 1'b0;
						// assert /HSHK; the Mac comes round 2 -> 3 -> 1
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

				// wait for state 1
				TX_WAIT:
					if (state == 3'd1) begin
						txState <= TX_SYNC;
						txTick  <= byteTicks;
					end
					// states 4-7: a reset or the ID scan, the Mac has moved on
					else if (!selected || state >= 3'd4) begin
						hshk_n  <= 1'b1;
						txBusy  <= 1'b0;
						txGo    <= 1'b0;
						txAbort <= 1'b1;
						txState <= TX_IDLE;
					end

				// armed and waiting for the payload; state 2 means the Mac gave up
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

				// seven data bytes then the LSB byte; a hold-off is acted on at the group end
				TX_DATA: begin
					if (state == 3'd0) txHoff <= 1'b1;
					// states 2/3 mid-group mean the Mac has abandoned the frame
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
							// group boundary: the only place a hold-off is acted on
							if (txSent >= txTotal)                txState <= TX_END;
							else if (txHoff || state == 3'd0)     txState <= TX_HOFF;
							else                                  txState <= TX_DATA;
						end
					end
				end

				// hold-off: release /HSHK, wait for state 1, resend the sync
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

				// tear down on a cen tick so the IWM latches the last byte
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
