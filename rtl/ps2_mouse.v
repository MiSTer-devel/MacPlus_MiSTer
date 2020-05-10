`timescale 1ns / 100ps

/*
 * PS2 mouse protocol
 * Bit       7    6    5    4    3    2    1    0  
 * Byte 0: YOVR XOVR YSGN XSGN   1   MBUT RBUT LBUT
 * Byte 1:                 XMOVE
 * Byte 2:                 YMOVE
 */

/*
 * PS2 Mouse to Mac interface module
 */
module ps2_mouse
(
	input	clk,
	input	ce,

	input	reset,

	input [24:0] ps2_mouse,

	output reg x1,
	output reg y1,
	output reg x2,
	output reg y2,
	output reg button
);

reg  [9:0] xacc;
reg  [9:0] yacc;
reg [11:0] clkdiv;

wire strobe = (old_stb != ps2_mouse[24]);
reg  old_stb = 0;
always @(posedge clk) old_stb <= ps2_mouse[24];

/* Capture button state */
always@(posedge clk or posedge reset)
	if (reset) button <= 1;
	else if (strobe) button <= ~(|ps2_mouse[2:0]);

/* Clock divider to flush accumulators */
always@(posedge clk or posedge reset)
	if (reset) clkdiv <= 0;
	else if(ce) clkdiv <= clkdiv + 1'b1;

wire tick = (ce && clkdiv == 0);

/* Toggle output lines base on accumulator */
always@(posedge clk or posedge reset) begin
	if (reset) begin
		x1 <= 0;
		x2 <= 0;
	end else if (tick && xacc != 0) begin
		x1 <= ~x1;
		x2 <= ~x1 ^ ~xacc[9];
	end
end

always@(posedge clk or posedge reset) begin
	if (reset) begin
		y1 <= 0;
		y2 <= 0;
	end else if (tick && yacc != 0) begin
		y1 <= ~y1;
		y2 <= ~y1 ^ ~yacc[9];
	end
end

/* Movement accumulators. Needs tuning ! */
always@(posedge clk or posedge reset) begin
	if (reset) xacc <= 0;
	else begin
		/* Add movement, convert to a 10-bit number if not over */
		if (strobe && xacc[8] == xacc[9]) xacc <= xacc + { ps2_mouse[4], ps2_mouse[4], ps2_mouse[15:8] };
		else
		  /* Decrement */
		  if (tick && xacc != 0) xacc <= xacc + { {9{~xacc[9]}}, 1'b1 };
	end
end

always@(posedge clk or posedge reset) begin
	if (reset) yacc <= 0;
	else begin
		/* Add movement, convert to a 10-bit number if not over*/
		if (strobe && yacc[8] == yacc[9]) yacc <= yacc + { ps2_mouse[5], ps2_mouse[5], ps2_mouse[23:16] };
		else
		  /* Decrement */
		  if (tick && yacc != 0) yacc <= yacc + { {9{~yacc[9]}}, 1'b1 };
	end
end

endmodule
