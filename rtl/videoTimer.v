// The Macintosh Plus raster: 704 x 370 at the 15.6672 MHz
// dot clock, 512 x 342 visible, 22.2545 kHz / 60.147 Hz.
//
//   hcnt  0-511   active (words 0-31)        vcnt  0-27    blank; VBL at line 0
//         512-703 blank (192 px)                   28-369  active
//   HSYNC* 535 for 288 px, into the next line      VSYNC*  lines 0-3
//
// hcnt counts pixels on pix_en (every other clk). A word is fetched on every
// other video slot (busCycle 0), loaded just before its first pixel.
module videoTimer(
	input clk,
	input clk_en,			// clk8_en_p: the clk8 edge, busCycle advances
	input pix_en,			// clk16_en_p: one pixel
	input [1:0] busCycle,
	input vid_alt,
	output [21:0] videoAddr,
	output reg hsync = 1'b0,	// power-up state is pixel 702: in the sync pulse
	output reg vsync = 1'b1,
	output reg _hblank = 1'b0,	// VIA PB6 (Plus): high on the active 512 px
	output reg _vblank = 1'b1,	// VIA CA1: falls at the VBL
	output reg de = 1'b0,
	output loadPixels
);

	localparam H_TOTAL     = 704;
	localparam H_ACTIVE    = 512;
	localparam HSYNC_START = 535;
	localparam HSYNC_END   = HSYNC_START + 288 - H_TOTAL;	// 119, next line
	localparam V_TOTAL     = 370;
	localparam V_BLANK     = 28;
	localparam VSYNC_LINES = 4;

	// use screen buffer address for a 4MB RAM layout-- it will wrap
	// around to the proper address for 1MB, 512K, and 128K layouts
	localparam kScreenBufferBase = 22'h3FA700;

	// power up two pixels before line 0, so word starts fall on the video slot
	// from the first clock (busCycle and busPhase power up at 0)
	reg [9:0] hcnt = H_TOTAL - 2;
	reg [8:0] vcnt = V_TOTAL - 1;

	wire       eol       = (hcnt == H_TOTAL - 1);
	wire [9:0] hcnt_next = eol ? 10'd0 : hcnt + 1'd1;
	wire [8:0] vcnt_next = !eol ? vcnt : (vcnt == V_TOTAL - 1) ? 9'd0 : vcnt + 1'd1;

	always @(posedge clk) begin
		if (pix_en) begin
			hcnt <= hcnt_next;
			vcnt <= vcnt_next;
			hsync   <= ~(hcnt_next >= HSYNC_START || hcnt_next < HSYNC_END);
			vsync   <= ~(vcnt_next < VSYNC_LINES);
			_hblank <= hcnt_next < H_ACTIVE;
			_vblank <= vcnt_next >= V_BLANK;
			de      <= hcnt_next < H_ACTIVE && vcnt_next >= V_BLANK;
		end else if (clk_en && busCycle == 2'd0 && hcnt[2:0] != 3'd0)
			hcnt <= {hcnt[9:3], 3'd0};	// startup: put word starts on the video slot
	end

	// The word fetched in this clk8. hcnt moves twice per clk8 and reaches the
	// word's first pixel at the slot's last quarter, so fpos (hcnt + 2) names
	// the same word for the whole clk8; 702-703 fetch word 0 of the next line.
	wire       wrap  = (hcnt >= H_TOTAL - 2);
	wire [9:0] fpos  = wrap ? 10'd0 : hcnt + 10'd2;
	wire [8:0] fline = wrap ? ((vcnt == V_TOTAL - 1) ? 9'd0 : vcnt + 1'd1) : vcnt;
	wire [8:0] frow  = fline - V_BLANK;
	wire       fetch = fpos < H_ACTIVE && fpos[3] == 1'b0 && fline >= V_BLANK;

	assign videoAddr = kScreenBufferBase - (vid_alt ? 16'h0 : 16'h8000) +
							 { frow, fpos[8:4], 1'b0 };

	assign loadPixels = fetch && busCycle == 2'd0;

endmodule
