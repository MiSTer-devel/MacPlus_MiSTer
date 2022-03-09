/* ADB implementation for plus_too */

module adb(
	input            clk,
	input            clk_en,
	input            reset,
	input      [1:0] st,
	output           _int,
	input            viaBusy,
	output reg       listen,
	input      [7:0] adb_din,
	input            adb_din_strobe,
	output reg [7:0] adb_dout,
	output reg       adb_dout_strobe,
	
	output reg       capslock,

	input     [24:0] ps2_mouse,
	input     [10:0] ps2_key
);

localparam TALKINTERVAL = 17'd8000*4'd11; // 11 ms

reg   [3:0] cmd_r;
reg   [1:0] st_r;
wire  [1:0] r_r = cmd_r[1:0];
reg   [3:0] addr_r;
reg   [3:0] respCnt;
reg  [16:0] talkTimer;
reg         idleActive;

wire  [3:0] cmd = adb_din[3:0];
wire  [3:0] addr = adb_din[7:4];

reg         sendResponse;

always @(posedge clk) begin
	if (reset) begin
		respCnt <= 0;
		idleActive <= 0;
		cmd_r <= 0;
		listen <= 0;
		sendResponse <= 0;
	end else if (clk_en) begin
		st_r <= st;
		adb_dout_strobe <= 0;
		sendResponse <= 0;

		case (st)
		2'b00: // new command
		begin
			if (st_r != 2'b00)
				listen <= 1;

			respCnt <= 0;
			if (adb_din_strobe) begin
				idleActive <= 1;
				cmd_r <= cmd;
				addr_r <= addr;
				listen <= 0;

				if (addr_r != addr || cmd_r != cmd)
					talkTimer <= 0;
				else
					talkTimer <= TALKINTERVAL;

			end
		end

		2'b01, 2'b10: // even byte, odd byte
		begin
			// Reset, flush, talk
			if (!viaBusy && (cmd_r[3:1] == 0 || cmd_r[3:2] == 2'b11) && respCnt[0] == st[1]) begin
				sendResponse <= 1;
				respCnt <= respCnt + 1'd1;
			end
			if (sendResponse) begin
				adb_dout <= adbReg;
				adb_dout_strobe <= 1;
			end
			// Listen
			if (st_r != st) listen <= cmd_r[3:2] == 2'b10;
			if (cmd_r[3:2] == 2'b10 && respCnt[0] == st[1]) begin
				if (adb_din_strobe) begin
					listen <= 0;
					respCnt <= respCnt + 1'd1;
					// Listen : it's handled in the device specific part
					// The Listen command is to write to registers, some use cases:
					// - device ID and device handler writes
					// - LED status for the keyboard
				end
			end
		end

		2'b11: // idle
		begin
			if (cmd_r[3:2] == 2'b11 && idleActive) begin
				if (talkTimer != 0)
					talkTimer <= talkTimer - 1'd1;
				else begin
					adb_dout <= 8'hFF;
					adb_dout_strobe <= 1;
					talkTimer <= TALKINTERVAL;
					idleActive <= 0;
				end
			end
		end
		default: ;
		endcase
	end
end

// Device handlers
wire  [3:0] addrKeyboard = kbdReg3[11:8];
wire  [3:0] addrMouse = mouseReg3[11:8];

wire   mouseInt = (addr_r != addrMouse && mouseValid == 2'b01);
wire   keyboardInt = (addr_r != addrKeyboard && (keyboardValid == 1 || keyboardValid == 2));
wire   irq = mouseInt | keyboardInt | !adbValid;
wire   int_inhibit = respCnt < 3 && 
                     ((addr_r == addrMouse && mouseValid == 2'b01) ||
					  (addr_r == addrKeyboard && (keyboardValid == 1 || keyboardValid == 2)));
assign _int = ~(irq && (st == 2'b01 || st == 2'b10)) | int_inhibit;

// Mouse handler
reg  [15:0] mouseReg3;
reg   [6:0] X,Y;
reg   [1:0] mouseValid;

reg mstb;
always @(posedge clk) if (clk_en) mstb <= ps2_mouse[24];

wire       mouseStrobe = mstb ^ ps2_mouse[24];
wire [8:0] mouseX = {ps2_mouse[4], ps2_mouse[15:8]};
wire [8:0] mouseY = {ps2_mouse[5], ps2_mouse[23:16]};
wire       button = ps2_mouse[0];

always @(posedge clk) begin
	if (reset || cmd_r == 0) begin
		mouseReg3 <= 16'h6301; // device id: 3 device handler id: 1
		X <= 0;
		Y <= 0;
		mouseValid <= 0;
	end else if (clk_en) begin

		if (mouseStrobe) begin
			if (~mouseX[8] & |mouseX[7:6]) X <= 7'h3F;
			else if (mouseX[8] & ~mouseX[6]) X <= 7'h40;
			else X <= mouseX[6:0];

			if (~mouseY[8] & |mouseY[7:6]) Y <= 7'h40;
			else if (mouseY[8] & ~mouseY[6]) Y <= 7'h3F;
			else Y <= -mouseY[6:0];

			mouseValid <= 2'b01;
		end

		if (addr_r == addrMouse) begin

			if (mouseValid == 2'b01 && respCnt == 3)
				// mouse data sent
				mouseValid <= 2'b10;

			if ((mouseValid == 2'b10 && st == 2'b00) || cmd_r == 4'b0001) begin
				// Flush mouse data after read or flush command
				mouseValid <= 0;
				X <= 0;
				Y <= 0;
			end
		end

	end
end

// Keyboard handler
reg   [1:0] keyboardValid;
reg  [15:0] kbdReg0;
reg  [15:0] kbdReg2;
reg  [15:0] kbdReg3;
reg   [7:0] kbdFifo[8];
reg   [2:0] kbdFifoRd, kbdFifoWr;

always @(posedge clk) begin
	if (reset || cmd_r == 0) begin
		kbdReg0 <= 16'hFFFF;
		kbdReg2 <= 16'hFFFF;
		kbdReg3 <= 16'h6202; // device id: 2 device handler id: 2
		keyboardValid <= 0;
		kbdFifoRd <= 0;
		kbdFifoWr <= 0;
	end else if (clk_en) begin

		if (keyStrobe && keyData[6:0] != 7'h7F) begin
			// Store the keypress in the FIFO
			kbdFifo[kbdFifoWr] <= keyData;
			kbdFifoWr <= kbdFifoWr + 1'd1;
		end

		if (kbdFifoWr != kbdFifoRd && st == 2'b11 && keyboardValid < 2) begin
			// Read the FIFO when no other key processing in progress
			if (kbdReg0[6:0] == kbdFifo[kbdFifoRd][6:0])
				kbdReg0[7:0] <= kbdFifo[kbdFifoRd];
			else if (kbdReg0[14:8] == kbdFifo[kbdFifoRd][6:0])
				kbdReg0[15:8] <= kbdFifo[kbdFifoRd];
			else if (kbdReg0[7:0] == 8'hFF)
				kbdReg0[7:0] <= kbdFifo[kbdFifoRd];
			else
				kbdReg0[15:8] <= kbdFifo[kbdFifoRd];

			// kbdReg0 has a valid key
			keyboardValid <= keyboardValid + 1'd1;
			kbdFifoRd <= kbdFifoRd + 1'd1;
		end

		if (addr_r == addrKeyboard)	begin
			if (cmd_r == 4'b1010 && adb_din_strobe && st[1]^st[0]) begin
				// write into reg2 (keyboard LEDs)
				if (respCnt == 1) kbdReg2[2:0] <= adb_din[2:0];
			end

			if (keyboardValid != 0 && respCnt == 2)
				// Beginning of keyboard data read
				keyboardValid <= 2'd3;

			if ((keyboardValid == 3 && st == 2'b00) || cmd_r == 4'b0001) begin
				// Flush keyboard data after read or flush command
				keyboardValid <= 0;
				kbdReg0 <= 16'hFFFF;
				if (cmd_r == 4'b0001) begin
					// Flush
					kbdFifoRd <= 0;
					kbdFifoWr <= 0;
				end
			end
		end

	end
end

// Register 0 in the Apple Standard Mouse
// Bit   Meaning
// 15    Button status; 0 = down
// 14-8  Y move counts'
// 7     Not used (always 1)
// 6-0   X move counts

// Register 0 in the Apple Standard Keyboard
// Bit   Meaning
// 15    Key status for first key; 0 = down
// 14-8  Key code for first key; a 7-bit ASCII value
// 7     Key status for second key; 0 = down
// 6-0   Key code for second key; a 7-bit ASCII value

// Register 2 in the Apple Extended Keyboard
// Bit   Key
// 15    None (reserved)
// 14    Delete
// 13    Caps Lock
// 12    Reset
// 11    Control
// 10    Shift
// 9     Option
// 8     Command
// 7     Num Lock/Clear
// 6     Scroll Lock
// 5-3   None (reserved)
// 2     LED 3 (Scroll Lock) *
// 1     LED 2 (Caps Lock) *
// 0     LED 1 (Num Lock) *
//
// *Changeable via Listen Register 2

// Register 3 (common for all devices):
// Bit   Description
// 15    Reserved; must be 0
// 14    Exceptional event, device specific; always 1 if not used
// 13    Service Request enable; 1 = enabled
// 12    Reserved; must be 0
// 11-8  Device address
// 7-0   Device Handler ID

reg  [7:0] adbReg;
reg        adbValid;
reg [15:0] talkReg;

always @(*) begin
	adbReg = 8'hFF;
	adbValid = 0;
	talkReg = 0;
	if (addr_r == addrKeyboard) begin
		if (cmd_r[3:1] == 0) begin
			// reset
			if (respCnt == 0) adbValid = 1;
		end else begin
			// talk
			case (r_r)
			2'b00: talkReg = kbdReg0;
			2'b10: talkReg = kbdReg2;
			2'b11: talkReg = kbdReg3;
			default: ;
			endcase

			if (respCnt == 1) begin
				adbReg = talkReg[15:8];
				adbValid = 1;
			end
			if (respCnt == 2) begin
				adbReg = talkReg[7:0];
				adbValid = 1;
			end
		end
	end else if (addr_r == addrMouse) begin
		if (cmd_r[3:1] == 0) begin
			// reset
			if (respCnt == 0) adbValid = 1;
		end else begin
			// talk
			case (r_r)
			2'b00: talkReg = { ~button, Y, 1'b1, X };
			2'b11: talkReg = mouseReg3;
			default: ;
			endcase
			if (respCnt == 1) begin
				adbReg = talkReg[15:8];
				adbValid = 1;
			end
			if (respCnt == 2) begin
				adbReg = talkReg[7:0];
				adbValid = 1;
			end
		end
	end
end

reg       keyStrobe;
reg [7:0] keyData;
wire      press = ps2_key[9];
wire      capslock_key = (ps2_key[8:0] == 'h58);

always @(posedge clk) begin
	reg kstb;

	if (clk_en) begin
		kstb <= ps2_key[10];
		if (kstb ^ ps2_key[10]) begin
			case(ps2_key[8:0]) // Scan Code Set 2
			  9'h000: keyData[6:0] <= 7'h7F;
			  9'h001: keyData[6:0] <= 7'h65;	//F9
			  9'h002: keyData[6:0] <= 7'h7F;
			  9'h003: keyData[6:0] <= 7'h60;	//F5
			  9'h004: keyData[6:0] <= 7'h63;	//F3
			  9'h005: keyData[6:0] <= 7'h7A;	//F1
			  9'h006: keyData[6:0] <= 7'h78;	//F2
			  9'h007: keyData[6:0] <= 7'h7F;//7'h6F;	//F12 <OSD>
			  9'h008: keyData[6:0] <= 7'h7F;
			  9'h009: keyData[6:0] <= 7'h6D;	//F10
			  9'h00a: keyData[6:0] <= 7'h64;	//F8
			  9'h00b: keyData[6:0] <= 7'h61;	//F6
			  9'h00c: keyData[6:0] <= 7'h76;	//F4
			  9'h00d: keyData[6:0] <= 7'h30;	//TAB
			  9'h00e: keyData[6:0] <= 7'h32;	//~ (`)
			  9'h00f: keyData[6:0] <= 7'h7F;
			  9'h010: keyData[6:0] <= 7'h7F;
			  9'h011: keyData[6:0] <= 7'h37;	//LEFT ALT (command)
			  9'h012: keyData[6:0] <= 7'h38;	//LEFT SHIFT
			  9'h013: keyData[6:0] <= 7'h7F;
			  9'h014: keyData[6:0] <= 7'h36;	//CTRL
			  9'h015: keyData[6:0] <= 7'h0C;	//q
			  9'h016: keyData[6:0] <= 7'h12;	//1
			  9'h017: keyData[6:0] <= 7'h7F;
			  9'h018: keyData[6:0] <= 7'h7F;
			  9'h019: keyData[6:0] <= 7'h7F;
			  9'h01a: keyData[6:0] <= 7'h06;	//z
			  9'h01b: keyData[6:0] <= 7'h01;	//s
			  9'h01c: keyData[6:0] <= 7'h00;	//a
			  9'h01d: keyData[6:0] <= 7'h0D;	//w
			  9'h01e: keyData[6:0] <= 7'h13;	//2
			  9'h01f: keyData[6:0] <= 7'h7F;
			  9'h020: keyData[6:0] <= 7'h7F;
			  9'h021: keyData[6:0] <= 7'h08;	//c
			  9'h022: keyData[6:0] <= 7'h07;	//x
			  9'h023: keyData[6:0] <= 7'h02;	//d
			  9'h024: keyData[6:0] <= 7'h0E;	//e
			  9'h025: keyData[6:0] <= 7'h15;	//4
			  9'h026: keyData[6:0] <= 7'h14;	//3
			  9'h027: keyData[6:0] <= 7'h7F;
			  9'h028: keyData[6:0] <= 7'h7F;
			  9'h029: keyData[6:0] <= 7'h31;	//SPACE
			  9'h02a: keyData[6:0] <= 7'h09;	//v
			  9'h02b: keyData[6:0] <= 7'h03;	//f
			  9'h02c: keyData[6:0] <= 7'h11;	//t
			  9'h02d: keyData[6:0] <= 7'h0F;	//r
			  9'h02e: keyData[6:0] <= 7'h17;	//5
			  9'h02f: keyData[6:0] <= 7'h7F;
			  9'h030: keyData[6:0] <= 7'h7F;
			  9'h031: keyData[6:0] <= 7'h2D;	//n
			  9'h032: keyData[6:0] <= 7'h0B;	//b
			  9'h033: keyData[6:0] <= 7'h04;	//h
			  9'h034: keyData[6:0] <= 7'h05;	//g
			  9'h035: keyData[6:0] <= 7'h10;	//y
			  9'h036: keyData[6:0] <= 7'h16;	//6
			  9'h037: keyData[6:0] <= 7'h7F;
			  9'h038: keyData[6:0] <= 7'h7F;
			  9'h039: keyData[6:0] <= 7'h7F;
			  9'h03a: keyData[6:0] <= 7'h2E;	//m
			  9'h03b: keyData[6:0] <= 7'h26;	//j
			  9'h03c: keyData[6:0] <= 7'h20;	//u
			  9'h03d: keyData[6:0] <= 7'h1A;	//7
			  9'h03e: keyData[6:0] <= 7'h1C;	//8
			  9'h03f: keyData[6:0] <= 7'h7F;
			  9'h040: keyData[6:0] <= 7'h7F;
			  9'h041: keyData[6:0] <= 7'h2B;	//<,
			  9'h042: keyData[6:0] <= 7'h28;	//k
			  9'h043: keyData[6:0] <= 7'h22;	//i
			  9'h044: keyData[6:0] <= 7'h1F;	//o
			  9'h045: keyData[6:0] <= 7'h1D;	//0
			  9'h046: keyData[6:0] <= 7'h19;	//9
			  9'h047: keyData[6:0] <= 7'h7F;
			  9'h048: keyData[6:0] <= 7'h7F;
			  9'h049: keyData[6:0] <= 7'h2F;	//>.
			  9'h04a: keyData[6:0] <= 7'h2C;	//FORWARD SLASH
			  9'h04b: keyData[6:0] <= 7'h25;	//l
			  9'h04c: keyData[6:0] <= 7'h29;	//;
			  9'h04d: keyData[6:0] <= 7'h23;	//p
			  9'h04e: keyData[6:0] <= 7'h1B;	//-
			  9'h04f: keyData[6:0] <= 7'h7F;
			  9'h050: keyData[6:0] <= 7'h7F;
			  9'h051: keyData[6:0] <= 7'h7F;
			  9'h052: keyData[6:0] <= 7'h27;	//'"
			  9'h053: keyData[6:0] <= 7'h7F;
			  9'h054: keyData[6:0] <= 7'h21;	//[
			  9'h055: keyData[6:0] <= 7'h18;	// = 
			  9'h056: keyData[6:0] <= 7'h7F;
			  9'h057: keyData[6:0] <= 7'h7F;
			  9'h058: keyData[6:0] <= 7'h39;	//CAPSLOCK
			  9'h059: keyData[6:0] <= 7'h7B;	//RIGHT SHIFT
			  9'h05a: keyData[6:0] <= 7'h24;	//ENTER
			  9'h05b: keyData[6:0] <= 7'h1E;	//]
			  9'h05c: keyData[6:0] <= 7'h7F;
			  9'h05d: keyData[6:0] <= 7'h2A;	//BACKSLASH
			  9'h05e: keyData[6:0] <= 7'h7F;
			  9'h05f: keyData[6:0] <= 7'h7F;
			  9'h060: keyData[6:0] <= 7'h7F;
			  9'h061: keyData[6:0] <= 7'h7F;	//international left shift cut out (German '<>' key), 0x56 Set#1 code
			  9'h062: keyData[6:0] <= 7'h7F;
			  9'h063: keyData[6:0] <= 7'h7F;
			  9'h064: keyData[6:0] <= 7'h7F;
			  9'h065: keyData[6:0] <= 7'h7F;
			  9'h066: keyData[6:0] <= 7'h33;	//BACKSPACE
			  9'h067: keyData[6:0] <= 7'h7F;
			  9'h068: keyData[6:0] <= 7'h7F;
			  9'h069: keyData[6:0] <= 7'h53;	//KP 1
			  9'h06a: keyData[6:0] <= 7'h7F;
			  9'h06b: keyData[6:0] <= 7'h56;	//KP 4
			  9'h06c: keyData[6:0] <= 7'h59;	//KP 7
			  9'h06d: keyData[6:0] <= 7'h7F;
			  9'h06e: keyData[6:0] <= 7'h7F;
			  9'h06f: keyData[6:0] <= 7'h7F;
			  9'h070: keyData[6:0] <= 7'h52;	//KP 0
			  9'h071: keyData[6:0] <= 7'h41;	//KP .
			  9'h072: keyData[6:0] <= 7'h54;	//KP 2
			  9'h073: keyData[6:0] <= 7'h57;	//KP 5
			  9'h074: keyData[6:0] <= 7'h58;	//KP 6
			  9'h075: keyData[6:0] <= 7'h5B;	//KP 8
			  9'h076: keyData[6:0] <= 7'h35;	//ESCAPE
			  9'h077: keyData[6:0] <= 7'h47;	//NUMLOCK (Mac keypad clear?)
			  9'h078: keyData[6:0] <= 7'h67;	//F11 <OSD>
			  9'h079: keyData[6:0] <= 7'h45;	//KP +
			  9'h07a: keyData[6:0] <= 7'h55;	//KP 3
			  9'h07b: keyData[6:0] <= 7'h4E;	//KP -
			  9'h07c: keyData[6:0] <= 7'h43;	//KP *
			  9'h07d: keyData[6:0] <= 7'h5C;	//KP 9
			  9'h07e: keyData[6:0] <= 7'h7F;	//SCROLL LOCK / KP )
			  9'h07f: keyData[6:0] <= 7'h7F;
			  9'h080: keyData[6:0] <= 7'h7F;
			  9'h081: keyData[6:0] <= 7'h7F;
			  9'h082: keyData[6:0] <= 7'h7F;
			  9'h083: keyData[6:0] <= 7'h62;	//F7
			  9'h084: keyData[6:0] <= 7'h7F;
			  9'h085: keyData[6:0] <= 7'h7F;
			  9'h086: keyData[6:0] <= 7'h7F;
			  9'h087: keyData[6:0] <= 7'h7F;
			  9'h088: keyData[6:0] <= 7'h7F;
			  9'h089: keyData[6:0] <= 7'h7F;
			  9'h08a: keyData[6:0] <= 7'h7F;
			  9'h08b: keyData[6:0] <= 7'h7F;
			  9'h08c: keyData[6:0] <= 7'h7F;
			  9'h08d: keyData[6:0] <= 7'h7F;
			  9'h08e: keyData[6:0] <= 7'h7F;
			  9'h08f: keyData[6:0] <= 7'h7F;
			  9'h090: keyData[6:0] <= 7'h7F;
			  9'h091: keyData[6:0] <= 7'h7F;
			  9'h092: keyData[6:0] <= 7'h7F;
			  9'h093: keyData[6:0] <= 7'h7F;
			  9'h094: keyData[6:0] <= 7'h7F;
			  9'h095: keyData[6:0] <= 7'h7F;
			  9'h096: keyData[6:0] <= 7'h7F;
			  9'h097: keyData[6:0] <= 7'h7F;
			  9'h098: keyData[6:0] <= 7'h7F;
			  9'h099: keyData[6:0] <= 7'h7F;
			  9'h09a: keyData[6:0] <= 7'h7F;
			  9'h09b: keyData[6:0] <= 7'h7F;
			  9'h09c: keyData[6:0] <= 7'h7F;
			  9'h09d: keyData[6:0] <= 7'h7F;
			  9'h09e: keyData[6:0] <= 7'h7F;
			  9'h09f: keyData[6:0] <= 7'h7F;
			  9'h0a0: keyData[6:0] <= 7'h7F;
			  9'h0a1: keyData[6:0] <= 7'h7F;
			  9'h0a2: keyData[6:0] <= 7'h7F;
			  9'h0a3: keyData[6:0] <= 7'h7F;
			  9'h0a4: keyData[6:0] <= 7'h7F;
			  9'h0a5: keyData[6:0] <= 7'h7F;
			  9'h0a6: keyData[6:0] <= 7'h7F;
			  9'h0a7: keyData[6:0] <= 7'h7F;
			  9'h0a8: keyData[6:0] <= 7'h7F;
			  9'h0a9: keyData[6:0] <= 7'h7F;
			  9'h0aa: keyData[6:0] <= 7'h7F;
			  9'h0ab: keyData[6:0] <= 7'h7F;
			  9'h0ac: keyData[6:0] <= 7'h7F;
			  9'h0ad: keyData[6:0] <= 7'h7F;
			  9'h0ae: keyData[6:0] <= 7'h7F;
			  9'h0af: keyData[6:0] <= 7'h7F;
			  9'h0b0: keyData[6:0] <= 7'h7F;
			  9'h0b1: keyData[6:0] <= 7'h7F;
			  9'h0b2: keyData[6:0] <= 7'h7F;
			  9'h0b3: keyData[6:0] <= 7'h7F;
			  9'h0b4: keyData[6:0] <= 7'h7F;
			  9'h0b5: keyData[6:0] <= 7'h7F;
			  9'h0b6: keyData[6:0] <= 7'h7F;
			  9'h0b7: keyData[6:0] <= 7'h7F;
			  9'h0b8: keyData[6:0] <= 7'h7F;
			  9'h0b9: keyData[6:0] <= 7'h7F;
			  9'h0ba: keyData[6:0] <= 7'h7F;
			  9'h0bb: keyData[6:0] <= 7'h7F;
			  9'h0bc: keyData[6:0] <= 7'h7F;
			  9'h0bd: keyData[6:0] <= 7'h7F;
			  9'h0be: keyData[6:0] <= 7'h7F;
			  9'h0bf: keyData[6:0] <= 7'h7F;
			  9'h0c0: keyData[6:0] <= 7'h7F;
			  9'h0c1: keyData[6:0] <= 7'h7F;
			  9'h0c2: keyData[6:0] <= 7'h7F;
			  9'h0c3: keyData[6:0] <= 7'h7F;
			  9'h0c4: keyData[6:0] <= 7'h7F;
			  9'h0c5: keyData[6:0] <= 7'h7F;
			  9'h0c6: keyData[6:0] <= 7'h7F;
			  9'h0c7: keyData[6:0] <= 7'h7F;
			  9'h0c8: keyData[6:0] <= 7'h7F;
			  9'h0c9: keyData[6:0] <= 7'h7F;
			  9'h0ca: keyData[6:0] <= 7'h7F;
			  9'h0cb: keyData[6:0] <= 7'h7F;
			  9'h0cc: keyData[6:0] <= 7'h7F;
			  9'h0cd: keyData[6:0] <= 7'h7F;
			  9'h0ce: keyData[6:0] <= 7'h7F;
			  9'h0cf: keyData[6:0] <= 7'h7F;
			  9'h0d0: keyData[6:0] <= 7'h7F;
			  9'h0d1: keyData[6:0] <= 7'h7F;
			  9'h0d2: keyData[6:0] <= 7'h7F;
			  9'h0d3: keyData[6:0] <= 7'h7F;
			  9'h0d4: keyData[6:0] <= 7'h7F;
			  9'h0d5: keyData[6:0] <= 7'h7F;
			  9'h0d6: keyData[6:0] <= 7'h7F;
			  9'h0d7: keyData[6:0] <= 7'h7F;
			  9'h0d8: keyData[6:0] <= 7'h7F;
			  9'h0d9: keyData[6:0] <= 7'h7F;
			  9'h0da: keyData[6:0] <= 7'h7F;
			  9'h0db: keyData[6:0] <= 7'h7F;
			  9'h0dc: keyData[6:0] <= 7'h7F;
			  9'h0dd: keyData[6:0] <= 7'h7F;
			  9'h0de: keyData[6:0] <= 7'h7F;
			  9'h0df: keyData[6:0] <= 7'h7F;
			  9'h0e0: keyData[6:0] <= 7'h7F;	//ps2 extended key
			  9'h0e1: keyData[6:0] <= 7'h7F;
			  9'h0e2: keyData[6:0] <= 7'h7F;
			  9'h0e3: keyData[6:0] <= 7'h7F;
			  9'h0e4: keyData[6:0] <= 7'h7F;
			  9'h0e5: keyData[6:0] <= 7'h7F;
			  9'h0e6: keyData[6:0] <= 7'h7F;
			  9'h0e7: keyData[6:0] <= 7'h7F;
			  9'h0e8: keyData[6:0] <= 7'h7F;
			  9'h0e9: keyData[6:0] <= 7'h7F;
			  9'h0ea: keyData[6:0] <= 7'h7F;
			  9'h0eb: keyData[6:0] <= 7'h7F;
			  9'h0ec: keyData[6:0] <= 7'h7F;
			  9'h0ed: keyData[6:0] <= 7'h7F;
			  9'h0ee: keyData[6:0] <= 7'h7F;
			  9'h0ef: keyData[6:0] <= 7'h7F;
			  9'h0f0: keyData[6:0] <= 7'h7F;	//ps2 release code
			  9'h0f1: keyData[6:0] <= 7'h7F;
			  9'h0f2: keyData[6:0] <= 7'h7F;
			  9'h0f3: keyData[6:0] <= 7'h7F;
			  9'h0f4: keyData[6:0] <= 7'h7F;
			  9'h0f5: keyData[6:0] <= 7'h7F;
			  9'h0f6: keyData[6:0] <= 7'h7F;
			  9'h0f7: keyData[6:0] <= 7'h7F;
			  9'h0f8: keyData[6:0] <= 7'h7F;
			  9'h0f9: keyData[6:0] <= 7'h7F;
			  9'h0fa: keyData[6:0] <= 7'h7F;	//ps2 ack code
			  9'h0fb: keyData[6:0] <= 7'h7F;
			  9'h0fc: keyData[6:0] <= 7'h7F;
			  9'h0fd: keyData[6:0] <= 7'h7F;
			  9'h0fe: keyData[6:0] <= 7'h7F;
			  9'h0ff: keyData[6:0] <= 7'h7F;
			  9'h100: keyData[6:0] <= 7'h7F;
			  9'h101: keyData[6:0] <= 7'h7F;
			  9'h102: keyData[6:0] <= 7'h7F;
			  9'h103: keyData[6:0] <= 7'h7F;
			  9'h104: keyData[6:0] <= 7'h7F;
			  9'h105: keyData[6:0] <= 7'h7F;
			  9'h106: keyData[6:0] <= 7'h7F;
			  9'h107: keyData[6:0] <= 7'h7F;
			  9'h108: keyData[6:0] <= 7'h7F;
			  9'h109: keyData[6:0] <= 7'h7F;
			  9'h10a: keyData[6:0] <= 7'h7F;
			  9'h10b: keyData[6:0] <= 7'h7F;
			  9'h10c: keyData[6:0] <= 7'h7F;
			  9'h10d: keyData[6:0] <= 7'h7F;
			  9'h10e: keyData[6:0] <= 7'h7F;
			  9'h10f: keyData[6:0] <= 7'h7F;
			  9'h110: keyData[6:0] <= 7'h7F;
			  9'h111: keyData[6:0] <= 7'h37;	//RIGHT ALT (command)
			  9'h112: keyData[6:0] <= 7'h7F;
			  9'h113: keyData[6:0] <= 7'h7F;
			  9'h114: keyData[6:0] <= 7'h7F;
			  9'h115: keyData[6:0] <= 7'h7F;
			  9'h116: keyData[6:0] <= 7'h7F;
			  9'h117: keyData[6:0] <= 7'h7F;
			  9'h118: keyData[6:0] <= 7'h7F;
			  9'h119: keyData[6:0] <= 7'h7F;
			  9'h11a: keyData[6:0] <= 7'h7F;
			  9'h11b: keyData[6:0] <= 7'h7F;
			  9'h11c: keyData[6:0] <= 7'h7F;
			  9'h11d: keyData[6:0] <= 7'h7F;
			  9'h11e: keyData[6:0] <= 7'h7F;
			  9'h11f: keyData[6:0] <= 7'h3A;	//WINDOWS OR APPLICATION KEY (option)
			  9'h120: keyData[6:0] <= 7'h7F;
			  9'h121: keyData[6:0] <= 7'h7F;
			  9'h122: keyData[6:0] <= 7'h7F;
			  9'h123: keyData[6:0] <= 7'h7F;
			  9'h124: keyData[6:0] <= 7'h7F;
			  9'h125: keyData[6:0] <= 7'h7F;
			  9'h126: keyData[6:0] <= 7'h7F;
			  9'h127: keyData[6:0] <= 7'h7F;
			  9'h128: keyData[6:0] <= 7'h7F;
			  9'h129: keyData[6:0] <= 7'h7F;
			  9'h12a: keyData[6:0] <= 7'h7F;
			  9'h12b: keyData[6:0] <= 7'h7F;
			  9'h12c: keyData[6:0] <= 7'h7F;
			  9'h12d: keyData[6:0] <= 7'h7F;
			  9'h12e: keyData[6:0] <= 7'h7F;
			  9'h12f: keyData[6:0] <= 7'h7F;	
			  9'h130: keyData[6:0] <= 7'h7F;
			  9'h131: keyData[6:0] <= 7'h7F;
			  9'h132: keyData[6:0] <= 7'h7F;
			  9'h133: keyData[6:0] <= 7'h7F;
			  9'h134: keyData[6:0] <= 7'h7F;
			  9'h135: keyData[6:0] <= 7'h7F;
			  9'h136: keyData[6:0] <= 7'h7F;
			  9'h137: keyData[6:0] <= 7'h7F;
			  9'h138: keyData[6:0] <= 7'h7F;
			  9'h139: keyData[6:0] <= 7'h7F;
			  9'h13a: keyData[6:0] <= 7'h7F;
			  9'h13b: keyData[6:0] <= 7'h7F;
			  9'h13c: keyData[6:0] <= 7'h7F;
			  9'h13d: keyData[6:0] <= 7'h7F;
			  9'h13e: keyData[6:0] <= 7'h7F;
			  9'h13f: keyData[6:0] <= 7'h7F;
			  9'h140: keyData[6:0] <= 7'h7F;
			  9'h141: keyData[6:0] <= 7'h7F;
			  9'h142: keyData[6:0] <= 7'h7F;
			  9'h143: keyData[6:0] <= 7'h7F;
			  9'h144: keyData[6:0] <= 7'h7F;
			  9'h145: keyData[6:0] <= 7'h7F;
			  9'h146: keyData[6:0] <= 7'h7F;
			  9'h147: keyData[6:0] <= 7'h7F;
			  9'h148: keyData[6:0] <= 7'h7F;
			  9'h149: keyData[6:0] <= 7'h7F;
			  9'h14a: keyData[6:0] <= 7'h4B;	//KP /
			  9'h14b: keyData[6:0] <= 7'h7F;
			  9'h14c: keyData[6:0] <= 7'h7F;
			  9'h14d: keyData[6:0] <= 7'h7F;
			  9'h14e: keyData[6:0] <= 7'h7F;
			  9'h14f: keyData[6:0] <= 7'h7F;
			  9'h150: keyData[6:0] <= 7'h7F;
			  9'h151: keyData[6:0] <= 7'h7F;
			  9'h152: keyData[6:0] <= 7'h7F;
			  9'h153: keyData[6:0] <= 7'h7F;
			  9'h154: keyData[6:0] <= 7'h7F;
			  9'h155: keyData[6:0] <= 7'h7F;
			  9'h156: keyData[6:0] <= 7'h7F;
			  9'h157: keyData[6:0] <= 7'h7F;
			  9'h158: keyData[6:0] <= 7'h7F;
			  9'h159: keyData[6:0] <= 7'h7F;
			  9'h15a: keyData[6:0] <= 7'h4C;	//KP ENTER
			  9'h15b: keyData[6:0] <= 7'h7F;
			  9'h15c: keyData[6:0] <= 7'h7F;
			  9'h15d: keyData[6:0] <= 7'h7F;
			  9'h15e: keyData[6:0] <= 7'h7F;
			  9'h15f: keyData[6:0] <= 7'h7F;
			  9'h160: keyData[6:0] <= 7'h7F;
			  9'h161: keyData[6:0] <= 7'h7F;
			  9'h162: keyData[6:0] <= 7'h7F;
			  9'h163: keyData[6:0] <= 7'h7F;
			  9'h164: keyData[6:0] <= 7'h7F;
			  9'h165: keyData[6:0] <= 7'h7F;
			  9'h166: keyData[6:0] <= 7'h7F;
			  9'h167: keyData[6:0] <= 7'h7F;
			  9'h168: keyData[6:0] <= 7'h7F;
			  9'h169: keyData[6:0] <= 7'h77;	//END
			  9'h16a: keyData[6:0] <= 7'h7F;
			  9'h16b: keyData[6:0] <= 7'h3B;	//ARROW LEFT
			  9'h16c: keyData[6:0] <= 7'h73;	//HOME
			  9'h16d: keyData[6:0] <= 7'h7F;
			  9'h16e: keyData[6:0] <= 7'h7F;
			  9'h16f: keyData[6:0] <= 7'h7F;
			  9'h170: keyData[6:0] <= 7'h72;	//INSERT = HELP
			  9'h171: keyData[6:0] <= 7'h75;	//DELETE (KP clear?)
			  9'h172: keyData[6:0] <= 7'h3D;	//ARROW DOWN
			  9'h173: keyData[6:0] <= 7'h7F;
			  9'h174: keyData[6:0] <= 7'h3C;	//ARROW RIGHT
			  9'h175: keyData[6:0] <= 7'h3E;	//ARROW UP
			  9'h176: keyData[6:0] <= 7'h7F;
			  9'h177: keyData[6:0] <= 7'h7F;
			  9'h178: keyData[6:0] <= 7'h7F;
			  9'h179: keyData[6:0] <= 7'h7F;
			  9'h17a: keyData[6:0] <= 7'h79;	//PGDN <OSD>
			  9'h17b: keyData[6:0] <= 7'h7F;
			  9'h17c: keyData[6:0] <= 7'h69;	//PRTSCR (F13)
			  9'h17d: keyData[6:0] <= 7'h74;	//PGUP <OSD>
			  9'h17e: keyData[6:0] <= 7'h71;	//ctrl+break (F15)
			  9'h17f: keyData[6:0] <= 7'h7F;
			  9'h180: keyData[6:0] <= 7'h7F;
			  9'h181: keyData[6:0] <= 7'h7F;
			  9'h182: keyData[6:0] <= 7'h7F;
			  9'h183: keyData[6:0] <= 7'h7F;
			  9'h184: keyData[6:0] <= 7'h7F;
			  9'h185: keyData[6:0] <= 7'h7F;
			  9'h186: keyData[6:0] <= 7'h7F;
			  9'h187: keyData[6:0] <= 7'h7F;
			  9'h188: keyData[6:0] <= 7'h7F;
			  9'h189: keyData[6:0] <= 7'h7F;
			  9'h18a: keyData[6:0] <= 7'h7F;
			  9'h18b: keyData[6:0] <= 7'h7F;
			  9'h18c: keyData[6:0] <= 7'h7F;
			  9'h18d: keyData[6:0] <= 7'h7F;
			  9'h18e: keyData[6:0] <= 7'h7F;
			  9'h18f: keyData[6:0] <= 7'h7F;
			  9'h190: keyData[6:0] <= 7'h7F;
			  9'h191: keyData[6:0] <= 7'h7F;
			  9'h192: keyData[6:0] <= 7'h7F;
			  9'h193: keyData[6:0] <= 7'h7F;
			  9'h194: keyData[6:0] <= 7'h7F;
			  9'h195: keyData[6:0] <= 7'h7F;
			  9'h196: keyData[6:0] <= 7'h7F;
			  9'h197: keyData[6:0] <= 7'h7F;
			  9'h198: keyData[6:0] <= 7'h7F;
			  9'h199: keyData[6:0] <= 7'h7F;
			  9'h19a: keyData[6:0] <= 7'h7F;
			  9'h19b: keyData[6:0] <= 7'h7F;
			  9'h19c: keyData[6:0] <= 7'h7F;
			  9'h19d: keyData[6:0] <= 7'h7F;
			  9'h19e: keyData[6:0] <= 7'h7F;
			  9'h19f: keyData[6:0] <= 7'h7F;
			  9'h1a0: keyData[6:0] <= 7'h7F;
			  9'h1a1: keyData[6:0] <= 7'h7F;
			  9'h1a2: keyData[6:0] <= 7'h7F;
			  9'h1a3: keyData[6:0] <= 7'h7F;
			  9'h1a4: keyData[6:0] <= 7'h7F;
			  9'h1a5: keyData[6:0] <= 7'h7F;
			  9'h1a6: keyData[6:0] <= 7'h7F;
			  9'h1a7: keyData[6:0] <= 7'h7F;
			  9'h1a8: keyData[6:0] <= 7'h7F;
			  9'h1a9: keyData[6:0] <= 7'h7F;
			  9'h1aa: keyData[6:0] <= 7'h7F;
			  9'h1ab: keyData[6:0] <= 7'h7F;
			  9'h1ac: keyData[6:0] <= 7'h7F;
			  9'h1ad: keyData[6:0] <= 7'h7F;
			  9'h1ae: keyData[6:0] <= 7'h7F;
			  9'h1af: keyData[6:0] <= 7'h7F;
			  9'h1b0: keyData[6:0] <= 7'h7F;
			  9'h1b1: keyData[6:0] <= 7'h7F;
			  9'h1b2: keyData[6:0] <= 7'h7F;
			  9'h1b3: keyData[6:0] <= 7'h7F;
			  9'h1b4: keyData[6:0] <= 7'h7F;
			  9'h1b5: keyData[6:0] <= 7'h7F;
			  9'h1b6: keyData[6:0] <= 7'h7F;
			  9'h1b7: keyData[6:0] <= 7'h7F;
			  9'h1b8: keyData[6:0] <= 7'h7F;
			  9'h1b9: keyData[6:0] <= 7'h7F;
			  9'h1ba: keyData[6:0] <= 7'h7F;
			  9'h1bb: keyData[6:0] <= 7'h7F;
			  9'h1bc: keyData[6:0] <= 7'h7F;
			  9'h1bd: keyData[6:0] <= 7'h7F;
			  9'h1be: keyData[6:0] <= 7'h7F;
			  9'h1bf: keyData[6:0] <= 7'h7F;
			  9'h1c0: keyData[6:0] <= 7'h7F;
			  9'h1c1: keyData[6:0] <= 7'h7F;
			  9'h1c2: keyData[6:0] <= 7'h7F;
			  9'h1c3: keyData[6:0] <= 7'h7F;
			  9'h1c4: keyData[6:0] <= 7'h7F;
			  9'h1c5: keyData[6:0] <= 7'h7F;
			  9'h1c6: keyData[6:0] <= 7'h7F;
			  9'h1c7: keyData[6:0] <= 7'h7F;
			  9'h1c8: keyData[6:0] <= 7'h7F;
			  9'h1c9: keyData[6:0] <= 7'h7F;
			  9'h1ca: keyData[6:0] <= 7'h7F;
			  9'h1cb: keyData[6:0] <= 7'h7F;
			  9'h1cc: keyData[6:0] <= 7'h7F;
			  9'h1cd: keyData[6:0] <= 7'h7F;
			  9'h1ce: keyData[6:0] <= 7'h7F;
			  9'h1cf: keyData[6:0] <= 7'h7F;
			  9'h1d0: keyData[6:0] <= 7'h7F;
			  9'h1d1: keyData[6:0] <= 7'h7F;
			  9'h1d2: keyData[6:0] <= 7'h7F;
			  9'h1d3: keyData[6:0] <= 7'h7F;
			  9'h1d4: keyData[6:0] <= 7'h7F;
			  9'h1d5: keyData[6:0] <= 7'h7F;
			  9'h1d6: keyData[6:0] <= 7'h7F;
			  9'h1d7: keyData[6:0] <= 7'h7F;
			  9'h1d8: keyData[6:0] <= 7'h7F;
			  9'h1d9: keyData[6:0] <= 7'h7F;
			  9'h1da: keyData[6:0] <= 7'h7F;
			  9'h1db: keyData[6:0] <= 7'h7F;
			  9'h1dc: keyData[6:0] <= 7'h7F;
			  9'h1dd: keyData[6:0] <= 7'h7F;
			  9'h1de: keyData[6:0] <= 7'h7F;
			  9'h1df: keyData[6:0] <= 7'h7F;
			  9'h1e0: keyData[6:0] <= 7'h7F;	//ps2 extended key(duplicate, see $e0)
			  9'h1e1: keyData[6:0] <= 7'h7F;
			  9'h1e2: keyData[6:0] <= 7'h7F;
			  9'h1e3: keyData[6:0] <= 7'h7F;
			  9'h1e4: keyData[6:0] <= 7'h7F;
			  9'h1e5: keyData[6:0] <= 7'h7F;
			  9'h1e6: keyData[6:0] <= 7'h7F;
			  9'h1e7: keyData[6:0] <= 7'h7F;
			  9'h1e8: keyData[6:0] <= 7'h7F;
			  9'h1e9: keyData[6:0] <= 7'h7F;
			  9'h1ea: keyData[6:0] <= 7'h7F;
			  9'h1eb: keyData[6:0] <= 7'h7F;
			  9'h1ec: keyData[6:0] <= 7'h7F;
			  9'h1ed: keyData[6:0] <= 7'h7F;
			  9'h1ee: keyData[6:0] <= 7'h7F;
			  9'h1ef: keyData[6:0] <= 7'h7F;
			  9'h1f0: keyData[6:0] <= 7'h7F;	//ps2 release code(duplicate, see $f0)
			  9'h1f1: keyData[6:0] <= 7'h7F;
			  9'h1f2: keyData[6:0] <= 7'h7F;
			  9'h1f3: keyData[6:0] <= 7'h7F;
			  9'h1f4: keyData[6:0] <= 7'h7F;
			  9'h1f5: keyData[6:0] <= 7'h7F;
			  9'h1f6: keyData[6:0] <= 7'h7F;
			  9'h1f7: keyData[6:0] <= 7'h7F;
			  9'h1f8: keyData[6:0] <= 7'h7F;
			  9'h1f9: keyData[6:0] <= 7'h7F;
			  9'h1fa: keyData[6:0] <= 7'h7F;	//ps2 ack code(duplicate see $fa)
			  9'h1fb: keyData[6:0] <= 7'h7F;
			  9'h1fc: keyData[6:0] <= 7'h7F;
			  9'h1fd: keyData[6:0] <= 7'h7F;
			  9'h1fe: keyData[6:0] <= 7'h7F;
			  9'h1ff: keyData[6:0] <= 7'h7F;
			endcase
			if(capslock_key && press) capslock <= ~capslock;
			if(!(capslock_key && capslock)) begin
				keyData[7] <= ~press;
				keyStrobe <= 1;
			end
		end
		else begin
			keyStrobe <= 0;
		end
	end

	if (reset) capslock <= 0;
end

endmodule

