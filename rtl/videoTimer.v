// generates 1024x768 (actually 512x768) @ 60Hz, from a 32.5MHz input clock
module videoTimer(
	input clk,
	input clk_en,
	input [1:0] busCycle,
	output [21:0] videoAddr,	 
	output reg hsync,
	output reg vsync,
	output _hblank,
	output _vblank,
	output loadPixels
);

	// timing data from http://tinyvga.com/vga-timing/1024x768@60Hz
	localparam 	kVisibleWidth = 128, // (1024/2)/4
					kTotalWidth = 168, // (1344/2)/4
					kVisibleHeightStart = 42,
					kVisibleHeightEnd = 725,
					kTotalHeight = 806,
					kHsyncStart = 131, // (1048/2)/4
					kHsyncEnd = 147, // (1184/2)/4-1
					kVsyncStart = 771,
					kVsyncEnd = 776,
					kPixelLatency = 1; // number of clk8 cycles from xpos==0 to when pixel data actually exits the video shift register

	// use screen buffer address for a 4MB RAM layout-- it will wrap
	// around to the proper address for 1MB, 512K, and 128K layouts
	localparam kScreenBufferBase = 22'h3FA700;
	
	reg [7:0] xpos;
	reg [9:0] ypos;

	wire endline = (xpos == kTotalWidth-1);

	always @(posedge clk) begin
		if (clk_en) begin
			if (endline)
				xpos <= 0;
			else if (xpos == 0 && busCycle != 0)
				// hold xpos at 0, until xpos and busCycle are in phase
				xpos <= 0;
			else
				xpos <= xpos + 1'b1;
		end
	end

	always @(posedge clk) begin
		if (clk_en) begin
			if (endline) begin
				if (ypos == kTotalHeight-1)
					ypos <= 0;
				else
					ypos <= ypos + 1'b1;	
			end
		end
	end

	always @(posedge clk) begin
		if (clk_en) begin
			hsync <= ~(xpos >= kHsyncStart+kPixelLatency && xpos <= kHsyncEnd+kPixelLatency);  
			vsync <= ~(ypos >= kVsyncStart && ypos <= kVsyncEnd);
		end
	end

	assign _hblank = ~(xpos >= kVisibleWidth);
	assign _vblank = ~(ypos < kVisibleHeightStart || ypos > kVisibleHeightEnd);
	
	// The 0,0 address actually starts below kScreenBufferBase, because the Mac screen buffer is
	// not displayed beginning at 0,0, but at 0,kVisibleHeightStart.
	// kVisibleHeightStart divided by 2 to account for vertical pixel doubling.
	// kVisibleWidth divided by 2 because it's the 8MHz visible width times 4 to get actual number of pixels, 
	// 	then divided by 8 bits per byte	
	assign videoAddr = kScreenBufferBase -
							 (kVisibleHeightStart/2 * kVisibleWidth/2) +
							 { ypos[9:1], xpos[6:2], 1'b0 };
	
	assign loadPixels = _vblank == 1'b1 && _hblank == 1'b1 && busCycle == 2'b00;
	
endmodule
