`timescale 1ns / 100ps

/*
 * PS2 Keyboard to Mac interface module
 */
module ps2_kbd
(
	input	clk,
	input	ce,

	input			 reset,

	input [10:0] ps2_key,
	output reg   capslock,
	 
	input  [7:0] data_out,
	input			 strobe_out,

	output [7:0] data_in,
	output 		 strobe_in
);

reg   [8:0] keymac;
reg			key_pending;
reg  [19:0] pacetimer;
reg			inquiry_active;
reg 			cmd_inquiry;
reg 			cmd_instant;
reg 			cmd_model;
reg 			cmd_test;

/* Latch commands from Mac */
always@(posedge clk or posedge reset) begin
	if (reset) begin
		cmd_inquiry <= 0;
		cmd_instant <= 0;
		cmd_model <= 0;
		cmd_test <= 0;
	end else if(ce) begin
		if (strobe_out) begin
			cmd_inquiry <= 0;
			cmd_instant <= 0;
			cmd_model <= 0;
			cmd_test <= 0;
			case(data_out)
				8'h10: cmd_inquiry <= 1;
				8'h14: cmd_instant <= 1;
				8'h16: cmd_model   <= 1;
				8'h36: cmd_test    <= 1;
			endcase
		end
	end
end

/* Divide our clock to pace our responses to the Mac. tick_short ticks
 * when we can respond to a command, and tick_long ticks when an inquiry
 * command shall timeout
 */
always@(posedge clk or posedge reset) begin
	if (reset)
		pacetimer <= 0;
	else if(ce) begin
		/* reset counter on command from Mac */
		if (strobe_out)
			pacetimer <= 0;		  
		else if (!tick_long)
			pacetimer <= pacetimer + 1'd1;
	end
end

wire tick_long  = pacetimer == 20'hfffff;
wire tick_short = pacetimer == 20'h00fff;

/* Delay inquiry responses to after tick_short */
always@(posedge clk or posedge reset) begin
	if (reset)
		inquiry_active <= 0;
	else if(ce) begin
		if (strobe_out | strobe_in)
			inquiry_active <= 0;
		else if (tick_short)
			inquiry_active <= cmd_inquiry;		  
	end	
end

wire pop_key = (cmd_instant & tick_short) | (inquiry_active & tick_long) | (inquiry_active & key_pending);

/* Reply to Mac */
assign strobe_in = ((cmd_model | cmd_test) & tick_short) | pop_key;	

/* Data to Mac */
assign data_in = cmd_test 	  ? 8'h7d :
		           cmd_model	  ? 8'h03 :
		           key_pending ? (keymac[8] ? 8'h79 : keymac[7:0]) : 
		                         8'h7b;	

wire press        = ps2_key[9];
wire capslock_key = (ps2_key[8:0] == 'h58);

/* Handle key_pending, and multi-byte keypad responses */
always @(posedge clk) begin
	reg old_stb;

	if(ce) begin
		old_stb <= ps2_key[10];
		if(old_stb != ps2_key[10]) begin
			/* Capslock handling */
			if(capslock_key && press) capslock <= ~capslock;
			
			if(!key_pending && !(capslock_key && capslock)) begin
				key_pending <= 1;
				keymac <= {key_code[8],~press,key_code[6:0]};
			end
		end
		
		if (pop_key) begin
			if (keymac[8]) keymac[8] <= 0;
			else key_pending <= 0;
		end
	end
	
	if (cmd_model | cmd_test | reset) key_pending <= 0;
	if (reset) capslock <= 0;
end


//use BRAM for table
wire [8:0] key_code = code[ps2_key[8:0]];
wire [8:0] code[512] =
'{
	/* 000 */ 9'h07b,
	/* 001 */ 9'h07b,	//F9
	/* 002 */ 9'h07b,
	/* 003 */ 9'h07b,	//F5
	/* 004 */ 9'h07b,	//F3
	/* 005 */ 9'h07b,	//F1
	/* 006 */ 9'h07b,	//F2
	/* 007 */ 9'h07b,	//F12 <OSD>
	/* 008 */ 9'h07b,
	/* 009 */ 9'h07b,	//F10
	/* 00a */ 9'h07b,	//F8
	/* 00b */ 9'h07b,	//F6
	/* 00c */ 9'h07b,	//F4
	/* 00d */ 9'h061,	//TAB
	/* 00e */ 9'h065,	//~ (`)
	/* 00f */ 9'h07b,
	/* 010 */ 9'h07b,
	/* 011 */ 9'h06f,	//LEFT ALT (command)
	/* 012 */ 9'h071,	//LEFT SHIFT
	/* 013 */ 9'h07b,
	/* 014 */ 9'h07b,	//CTRL (not mapped)
	/* 015 */ 9'h019,	//q
	/* 016 */ 9'h025,	//1
	/* 017 */ 9'h07b,
	/* 018 */ 9'h07b,
	/* 019 */ 9'h07b,
	/* 01a */ 9'h00d,	//z
	/* 01b */ 9'h003,	//s
	/* 01c */ 9'h001,	//a
	/* 01d */ 9'h01b,	//w
	/* 01e */ 9'h027,	//2
	/* 01f */ 9'h07b,
	/* 020 */ 9'h07b,
	/* 021 */ 9'h011,	//c
	/* 022 */ 9'h00f,	//x
	/* 023 */ 9'h005,	//d
	/* 024 */ 9'h01d,	//e
	/* 025 */ 9'h02b,	//4
	/* 026 */ 9'h029,	//3
	/* 027 */ 9'h07b,
	/* 028 */ 9'h07b,
	/* 029 */ 9'h063,	//SPACE
	/* 02a */ 9'h013,	//v
	/* 02b */ 9'h007,	//f
	/* 02c */ 9'h023,	//t
	/* 02d */ 9'h01f,	//r
	/* 02e */ 9'h02f,	//5
	/* 02f */ 9'h07b,
	/* 030 */ 9'h07b,
	/* 031 */ 9'h05b,	//n
	/* 032 */ 9'h017,	//b
	/* 033 */ 9'h009,	//h
	/* 034 */ 9'h00b,	//g
	/* 035 */ 9'h021,	//y
	/* 036 */ 9'h02d,	//6
	/* 037 */ 9'h07b,
	/* 038 */ 9'h07b,
	/* 039 */ 9'h07b,
	/* 03a */ 9'h05d,	//m
	/* 03b */ 9'h04d,	//j
	/* 03c */ 9'h041,	//u
	/* 03d */ 9'h035,	//7
	/* 03e */ 9'h039,	//8
	/* 03f */ 9'h07b,
	/* 040 */ 9'h07b,
	/* 041 */ 9'h057,	//<,
	/* 042 */ 9'h051,	//k
	/* 043 */ 9'h045,	//i
	/* 044 */ 9'h03f,	//o
	/* 045 */ 9'h03b,	//0
	/* 046 */ 9'h033,	//9
	/* 047 */ 9'h07b,
	/* 048 */ 9'h07b,
	/* 049 */ 9'h05f,	//>.
	/* 04a */ 9'h059,	//FORWARD SLASH
	/* 04b */ 9'h04b,	//l
	/* 04c */ 9'h053,	//;
	/* 04d */ 9'h047,	//p
	/* 04e */ 9'h037,	//-
	/* 04f */ 9'h07b,
	/* 050 */ 9'h07b,
	/* 051 */ 9'h07b,
	/* 052 */ 9'h04f,	//'"
	/* 053 */ 9'h07b,
	/* 054 */ 9'h043,	//[
	/* 055 */ 9'h031,	// = 
	/* 056 */ 9'h07b,
	/* 057 */ 9'h07b,
	/* 058 */ 9'h073,	//CAPSLOCK
	/* 059 */ 9'h071,	//RIGHT SHIFT
	/* 05a */ 9'h049,	//ENTER
	/* 05b */ 9'h03d,	//]
	/* 05c */ 9'h07b,
	/* 05d */ 9'h055,	//BACKSLASH
	/* 05e */ 9'h07b,
	/* 05f */ 9'h07b,
	/* 060 */ 9'h07b,
	/* 061 */ 9'h071,	//international left shift cut out (German '<>' key), 0x56 Set#1 code
	/* 062 */ 9'h07b,
	/* 063 */ 9'h07b,
	/* 064 */ 9'h07b,
	/* 065 */ 9'h07b,
	/* 066 */ 9'h067,	//BACKSPACE
	/* 067 */ 9'h07b,
	/* 068 */ 9'h07b,
	/* 069 */ 9'h127,	//KP 1
	/* 06a */ 9'h07b,
	/* 06b */ 9'h12d,	//KP 4
	/* 06c */ 9'h133,	//KP 7
	/* 06d */ 9'h07b,
	/* 06e */ 9'h07b,
	/* 06f */ 9'h07b,
	/* 070 */ 9'h125,	//KP 0
	/* 071 */ 9'h103,	//KP .
	/* 072 */ 9'h129,	//KP 2
	/* 073 */ 9'h12f,	//KP 5
	/* 074 */ 9'h131,	//KP 6
	/* 075 */ 9'h137,	//KP 8
	/* 076 */ 9'h07b,	//ESCAPE
	/* 077 */ 9'h07b,	//NUMLOCK (Mac keypad clear?)
	/* 078 */ 9'h07b,	//F11 <OSD>
	/* 079 */ 9'h10d,	//KP +
	/* 07a */ 9'h12b,	//KP 3
	/* 07b */ 9'h11d,	//KP -
	/* 07c */ 9'h105,	//KP *
	/* 07d */ 9'h139,	//KP 9
	/* 07e */ 9'h07b,	//SCROLL LOCK / KP )
	/* 07f */ 9'h07b,
	/* 080 */ 9'h07b,
	/* 081 */ 9'h07b,
	/* 082 */ 9'h07b,
	/* 083 */ 9'h07b,	//F7
	/* 084 */ 9'h07b,
	/* 085 */ 9'h07b,
	/* 086 */ 9'h07b,
	/* 087 */ 9'h07b,
	/* 088 */ 9'h07b,
	/* 089 */ 9'h07b,
	/* 08a */ 9'h07b,
	/* 08b */ 9'h07b,
	/* 08c */ 9'h07b,
	/* 08d */ 9'h07b,
	/* 08e */ 9'h07b,
	/* 08f */ 9'h07b,
	/* 090 */ 9'h07b,
	/* 091 */ 9'h07b,
	/* 092 */ 9'h07b,
	/* 093 */ 9'h07b,
	/* 094 */ 9'h07b,
	/* 095 */ 9'h07b,
	/* 096 */ 9'h07b,
	/* 097 */ 9'h07b,
	/* 098 */ 9'h07b,
	/* 099 */ 9'h07b,
	/* 09a */ 9'h07b,
	/* 09b */ 9'h07b,
	/* 09c */ 9'h07b,
	/* 09d */ 9'h07b,
	/* 09e */ 9'h07b,
	/* 09f */ 9'h07b,
	/* 0a0 */ 9'h07b,
	/* 0a1 */ 9'h07b,
	/* 0a2 */ 9'h07b,
	/* 0a3 */ 9'h07b,
	/* 0a4 */ 9'h07b,
	/* 0a5 */ 9'h07b,
	/* 0a6 */ 9'h07b,
	/* 0a7 */ 9'h07b,
	/* 0a8 */ 9'h07b,
	/* 0a9 */ 9'h07b,
	/* 0aa */ 9'h07b,
	/* 0ab */ 9'h07b,
	/* 0ac */ 9'h07b,
	/* 0ad */ 9'h07b,
	/* 0ae */ 9'h07b,
	/* 0af */ 9'h07b,
	/* 0b0 */ 9'h07b,
	/* 0b1 */ 9'h07b,
	/* 0b2 */ 9'h07b,
	/* 0b3 */ 9'h07b,
	/* 0b4 */ 9'h07b,
	/* 0b5 */ 9'h07b,
	/* 0b6 */ 9'h07b,
	/* 0b7 */ 9'h07b,
	/* 0b8 */ 9'h07b,
	/* 0b9 */ 9'h07b,
	/* 0ba */ 9'h07b,
	/* 0bb */ 9'h07b,
	/* 0bc */ 9'h07b,
	/* 0bd */ 9'h07b,
	/* 0be */ 9'h07b,
	/* 0bf */ 9'h07b,
	/* 0c0 */ 9'h07b,
	/* 0c1 */ 9'h07b,
	/* 0c2 */ 9'h07b,
	/* 0c3 */ 9'h07b,
	/* 0c4 */ 9'h07b,
	/* 0c5 */ 9'h07b,
	/* 0c6 */ 9'h07b,
	/* 0c7 */ 9'h07b,
	/* 0c8 */ 9'h07b,
	/* 0c9 */ 9'h07b,
	/* 0ca */ 9'h07b,
	/* 0cb */ 9'h07b,
	/* 0cc */ 9'h07b,
	/* 0cd */ 9'h07b,
	/* 0ce */ 9'h07b,
	/* 0cf */ 9'h07b,
	/* 0d0 */ 9'h07b,
	/* 0d1 */ 9'h07b,
	/* 0d2 */ 9'h07b,
	/* 0d3 */ 9'h07b,
	/* 0d4 */ 9'h07b,
	/* 0d5 */ 9'h07b,
	/* 0d6 */ 9'h07b,
	/* 0d7 */ 9'h07b,
	/* 0d8 */ 9'h07b,
	/* 0d9 */ 9'h07b,
	/* 0da */ 9'h07b,
	/* 0db */ 9'h07b,
	/* 0dc */ 9'h07b,
	/* 0dd */ 9'h07b,
	/* 0de */ 9'h07b,
	/* 0df */ 9'h07b,
	/* 0e0 */ 9'h07b,	//ps2 extended key
	/* 0e1 */ 9'h07b,
	/* 0e2 */ 9'h07b,
	/* 0e3 */ 9'h07b,
	/* 0e4 */ 9'h07b,
	/* 0e5 */ 9'h07b,
	/* 0e6 */ 9'h07b,
	/* 0e7 */ 9'h07b,
	/* 0e8 */ 9'h07b,
	/* 0e9 */ 9'h07b,
	/* 0ea */ 9'h07b,
	/* 0eb */ 9'h07b,
	/* 0ec */ 9'h07b,
	/* 0ed */ 9'h07b,
	/* 0ee */ 9'h07b,
	/* 0ef */ 9'h07b,
	/* 0f0 */ 9'h07b,	//ps2 release code
	/* 0f1 */ 9'h07b,
	/* 0f2 */ 9'h07b,
	/* 0f3 */ 9'h07b,
	/* 0f4 */ 9'h07b,
	/* 0f5 */ 9'h07b,
	/* 0f6 */ 9'h07b,
	/* 0f7 */ 9'h07b,
	/* 0f8 */ 9'h07b,
	/* 0f9 */ 9'h07b,
	/* 0fa */ 9'h07b,	//ps2 ack code
	/* 0fb */ 9'h07b,
	/* 0fc */ 9'h07b,
	/* 0fd */ 9'h07b,
	/* 0fe */ 9'h07b,
	/* 0ff */ 9'h07b,
	/* 100 */ 9'h07b,
	/* 101 */ 9'h07b,
	/* 102 */ 9'h07b,
	/* 103 */ 9'h07b,
	/* 104 */ 9'h07b,
	/* 105 */ 9'h07b,
	/* 106 */ 9'h07b,
	/* 107 */ 9'h07b,
	/* 108 */ 9'h07b,
	/* 109 */ 9'h07b,
	/* 10a */ 9'h07b,
	/* 10b */ 9'h07b,
	/* 10c */ 9'h07b,
	/* 10d */ 9'h07b,
	/* 10e */ 9'h07b,
	/* 10f */ 9'h07b,
	/* 110 */ 9'h07b,
	/* 111 */ 9'h06f,	//RIGHT ALT (command)
	/* 112 */ 9'h07b,
	/* 113 */ 9'h07b,
	/* 114 */ 9'h07b,
	/* 115 */ 9'h07b,
	/* 116 */ 9'h07b,
	/* 117 */ 9'h07b,
	/* 118 */ 9'h07b,
	/* 119 */ 9'h07b,
	/* 11a */ 9'h07b,
	/* 11b */ 9'h07b,
	/* 11c */ 9'h07b,
	/* 11d */ 9'h07b,
	/* 11e */ 9'h07b,
	/* 11f */ 9'h075,	//WINDOWS OR APPLICATION KEY (option)
	/* 120 */ 9'h07b,
	/* 121 */ 9'h07b,
	/* 122 */ 9'h07b,
	/* 123 */ 9'h07b,
	/* 124 */ 9'h07b,
	/* 125 */ 9'h07b,
	/* 126 */ 9'h07b,
	/* 127 */ 9'h07b,
	/* 128 */ 9'h07b,
	/* 129 */ 9'h07b,
	/* 12a */ 9'h07b,
	/* 12b */ 9'h07b,
	/* 12c */ 9'h07b,
	/* 12d */ 9'h07b,
	/* 12e */ 9'h07b,
	/* 12f */ 9'h07b,	
	/* 130 */ 9'h07b,
	/* 131 */ 9'h07b,
	/* 132 */ 9'h07b,
	/* 133 */ 9'h07b,
	/* 134 */ 9'h07b,
	/* 135 */ 9'h07b,
	/* 136 */ 9'h07b,
	/* 137 */ 9'h07b,
	/* 138 */ 9'h07b,
	/* 139 */ 9'h07b,
	/* 13a */ 9'h07b,
	/* 13b */ 9'h07b,
	/* 13c */ 9'h07b,
	/* 13d */ 9'h07b,
	/* 13e */ 9'h07b,
	/* 13f */ 9'h07b,
	/* 140 */ 9'h07b,
	/* 141 */ 9'h07b,
	/* 142 */ 9'h07b,
	/* 143 */ 9'h07b,
	/* 144 */ 9'h07b,
	/* 145 */ 9'h07b,
	/* 146 */ 9'h07b,
	/* 147 */ 9'h07b,
	/* 148 */ 9'h07b,
	/* 149 */ 9'h07b,
	/* 14a */ 9'h11b,	//KP /
	/* 14b */ 9'h07b,
	/* 14c */ 9'h07b,
	/* 14d */ 9'h07b,
	/* 14e */ 9'h07b,
	/* 14f */ 9'h07b,
	/* 150 */ 9'h07b,
	/* 151 */ 9'h07b,
	/* 152 */ 9'h07b,
	/* 153 */ 9'h07b,
	/* 154 */ 9'h07b,
	/* 155 */ 9'h07b,
	/* 156 */ 9'h07b,
	/* 157 */ 9'h07b,
	/* 158 */ 9'h07b,
	/* 159 */ 9'h07b,
	/* 15a */ 9'h119,	//KP ENTER
	/* 15b */ 9'h07b,
	/* 15c */ 9'h07b,
	/* 15d */ 9'h07b,
	/* 15e */ 9'h07b,
	/* 15f */ 9'h07b,
	/* 160 */ 9'h07b,
	/* 161 */ 9'h07b,
	/* 162 */ 9'h07b,
	/* 163 */ 9'h07b,
	/* 164 */ 9'h07b,
	/* 165 */ 9'h07b,
	/* 166 */ 9'h07b,
	/* 167 */ 9'h07b,
	/* 168 */ 9'h07b,
	/* 169 */ 9'h07b,	//END
	/* 16a */ 9'h07b,
	/* 16b */ 9'h10d,	//ARROW LEFT
	/* 16c */ 9'h07b,	//HOME
	/* 16d */ 9'h07b,
	/* 16e */ 9'h07b,
	/* 16f */ 9'h07b,
	/* 170 */ 9'h07b,	//INSERT = HELP
	/* 171 */ 9'h10f,	//DELETE (KP clear?)
	/* 172 */ 9'h111,	//ARROW DOWN
	/* 173 */ 9'h07b,
	/* 174 */ 9'h105,	//ARROW RIGHT
	/* 175 */ 9'h11b,	//ARROW UP
	/* 176 */ 9'h07b,
	/* 177 */ 9'h07b,
	/* 178 */ 9'h07b,
	/* 179 */ 9'h07b,
	/* 17a */ 9'h07b,	//PGDN <OSD>
	/* 17b */ 9'h07b,
	/* 17c */ 9'h07b,	//PRTSCR <OSD>
	/* 17d */ 9'h07b,	//PGUP <OSD>
	/* 17e */ 9'h07b,	//ctrl+break
	/* 17f */ 9'h07b,
	/* 180 */ 9'h07b,
	/* 181 */ 9'h07b,
	/* 182 */ 9'h07b,
	/* 183 */ 9'h07b,
	/* 184 */ 9'h07b,
	/* 185 */ 9'h07b,
	/* 186 */ 9'h07b,
	/* 187 */ 9'h07b,
	/* 188 */ 9'h07b,
	/* 189 */ 9'h07b,
	/* 18a */ 9'h07b,
	/* 18b */ 9'h07b,
	/* 18c */ 9'h07b,
	/* 18d */ 9'h07b,
	/* 18e */ 9'h07b,
	/* 18f */ 9'h07b,
	/* 190 */ 9'h07b,
	/* 191 */ 9'h07b,
	/* 192 */ 9'h07b,
	/* 193 */ 9'h07b,
	/* 194 */ 9'h07b,
	/* 195 */ 9'h07b,
	/* 196 */ 9'h07b,
	/* 197 */ 9'h07b,
	/* 198 */ 9'h07b,
	/* 199 */ 9'h07b,
	/* 19a */ 9'h07b,
	/* 19b */ 9'h07b,
	/* 19c */ 9'h07b,
	/* 19d */ 9'h07b,
	/* 19e */ 9'h07b,
	/* 19f */ 9'h07b,
	/* 1a0 */ 9'h07b,
	/* 1a1 */ 9'h07b,
	/* 1a2 */ 9'h07b,
	/* 1a3 */ 9'h07b,
	/* 1a4 */ 9'h07b,
	/* 1a5 */ 9'h07b,
	/* 1a6 */ 9'h07b,
	/* 1a7 */ 9'h07b,
	/* 1a8 */ 9'h07b,
	/* 1a9 */ 9'h07b,
	/* 1aa */ 9'h07b,
	/* 1ab */ 9'h07b,
	/* 1ac */ 9'h07b,
	/* 1ad */ 9'h07b,
	/* 1ae */ 9'h07b,
	/* 1af */ 9'h07b,
	/* 1b0 */ 9'h07b,
	/* 1b1 */ 9'h07b,
	/* 1b2 */ 9'h07b,
	/* 1b3 */ 9'h07b,
	/* 1b4 */ 9'h07b,
	/* 1b5 */ 9'h07b,
	/* 1b6 */ 9'h07b,
	/* 1b7 */ 9'h07b,
	/* 1b8 */ 9'h07b,
	/* 1b9 */ 9'h07b,
	/* 1ba */ 9'h07b,
	/* 1bb */ 9'h07b,
	/* 1bc */ 9'h07b,
	/* 1bd */ 9'h07b,
	/* 1be */ 9'h07b,
	/* 1bf */ 9'h07b,
	/* 1c0 */ 9'h07b,
	/* 1c1 */ 9'h07b,
	/* 1c2 */ 9'h07b,
	/* 1c3 */ 9'h07b,
	/* 1c4 */ 9'h07b,
	/* 1c5 */ 9'h07b,
	/* 1c6 */ 9'h07b,
	/* 1c7 */ 9'h07b,
	/* 1c8 */ 9'h07b,
	/* 1c9 */ 9'h07b,
	/* 1ca */ 9'h07b,
	/* 1cb */ 9'h07b,
	/* 1cc */ 9'h07b,
	/* 1cd */ 9'h07b,
	/* 1ce */ 9'h07b,
	/* 1cf */ 9'h07b,
	/* 1d0 */ 9'h07b,
	/* 1d1 */ 9'h07b,
	/* 1d2 */ 9'h07b,
	/* 1d3 */ 9'h07b,
	/* 1d4 */ 9'h07b,
	/* 1d5 */ 9'h07b,
	/* 1d6 */ 9'h07b,
	/* 1d7 */ 9'h07b,
	/* 1d8 */ 9'h07b,
	/* 1d9 */ 9'h07b,
	/* 1da */ 9'h07b,
	/* 1db */ 9'h07b,
	/* 1dc */ 9'h07b,
	/* 1dd */ 9'h07b,
	/* 1de */ 9'h07b,
	/* 1df */ 9'h07b,
	/* 1e0 */ 9'h07b,	//ps2 extended key(duplicate, see $e0)
	/* 1e1 */ 9'h07b,
	/* 1e2 */ 9'h07b,
	/* 1e3 */ 9'h07b,
	/* 1e4 */ 9'h07b,
	/* 1e5 */ 9'h07b,
	/* 1e6 */ 9'h07b,
	/* 1e7 */ 9'h07b,
	/* 1e8 */ 9'h07b,
	/* 1e9 */ 9'h07b,
	/* 1ea */ 9'h07b,
	/* 1eb */ 9'h07b,
	/* 1ec */ 9'h07b,
	/* 1ed */ 9'h07b,
	/* 1ee */ 9'h07b,
	/* 1ef */ 9'h07b,
	/* 1f0 */ 9'h07b,	//ps2 release code(duplicate, see $f0)
	/* 1f1 */ 9'h07b,
	/* 1f2 */ 9'h07b,
	/* 1f3 */ 9'h07b,
	/* 1f4 */ 9'h07b,
	/* 1f5 */ 9'h07b,
	/* 1f6 */ 9'h07b,
	/* 1f7 */ 9'h07b,
	/* 1f8 */ 9'h07b,
	/* 1f9 */ 9'h07b,
	/* 1fa */ 9'h07b,	//ps2 ack code(duplicate see $fa)
	/* 1fb */ 9'h07b,
	/* 1fc */ 9'h07b,
	/* 1fd */ 9'h07b,
	/* 1fe */ 9'h07b,
	/* 1ff */ 9'h07b
};

endmodule
