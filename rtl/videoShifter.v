module videoShifter(
	input clk32,
	input pix_en,			// one pixel: clk16_en_p, every other clk
	input memoryLatch,
	input [15:0] dataIn,
	input loadPixels,
	output pixelOut
    );
	
	reg [15:0] shiftRegister;
	
	// a 0 bit is white, and a 1 bit is black
	// data is shifted out MSB first
	assign pixelOut = ~shiftRegister[15];
	
	always @(posedge clk32) begin
		// the word arrives at the video slot's memoryLatch, which is never a
		// pixel edge, so a load and a shift cannot collide
		if (loadPixels && memoryLatch) begin
			shiftRegister <= dataIn;
		end
		else if (pix_en) begin
			shiftRegister <= { shiftRegister[14:0], 1'b1 };
		end
	end

endmodule
