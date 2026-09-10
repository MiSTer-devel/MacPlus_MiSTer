//
// disk_pwm_duty.v -- the Mac's commanded floppy spindle duty.
//
// The 128K/512K control a 400K drive's spindle in software. The Mac writes a
// dithered sequence into the sound buffer and the commanded duty is derived
// from the low 6 bits of each word (Guide to the Macintosh Family Hardware,
// 400KB drive specification):
//
//   1. convert each value through the fixed 64-entry table below,
//   2. sum 100 consecutive conversions,
//   3. index = sum/(count/10) - 11, clamped to 0..399,
//   4. duty% = index / 4.19.
//
module disk_pwm_duty
(
	input             clk,
	input             sample_en,   // one pulse per sound-buffer word fetched
	input       [5:0] sample,      // low 6 bits of that word
	output reg  [8:0] duty_index   // 0..399; duty% = index/4.19
);

	// conversion table, per the 400KB drive specification
	function [5:0] pwm_convert(input [5:0] v);
		case (v)
		6'd0 : pwm_convert = 6'd0;
		6'd1 : pwm_convert = 6'd1;
		6'd2 : pwm_convert = 6'd59;
		6'd3 : pwm_convert = 6'd2;
		6'd4 : pwm_convert = 6'd60;
		6'd5 : pwm_convert = 6'd40;
		6'd6 : pwm_convert = 6'd54;
		6'd7 : pwm_convert = 6'd3;
		6'd8 : pwm_convert = 6'd61;
		6'd9 : pwm_convert = 6'd32;
		6'd10: pwm_convert = 6'd49;
		6'd11: pwm_convert = 6'd41;
		6'd12: pwm_convert = 6'd55;
		6'd13: pwm_convert = 6'd19;
		6'd14: pwm_convert = 6'd35;
		6'd15: pwm_convert = 6'd4;
		6'd16: pwm_convert = 6'd62;
		6'd17: pwm_convert = 6'd52;
		6'd18: pwm_convert = 6'd30;
		6'd19: pwm_convert = 6'd33;
		6'd20: pwm_convert = 6'd50;
		6'd21: pwm_convert = 6'd12;
		6'd22: pwm_convert = 6'd14;
		6'd23: pwm_convert = 6'd42;
		6'd24: pwm_convert = 6'd56;
		6'd25: pwm_convert = 6'd16;
		6'd26: pwm_convert = 6'd27;
		6'd27: pwm_convert = 6'd20;
		6'd28: pwm_convert = 6'd36;
		6'd29: pwm_convert = 6'd23;
		6'd30: pwm_convert = 6'd44;
		6'd31: pwm_convert = 6'd5;
		6'd32: pwm_convert = 6'd63;
		6'd33: pwm_convert = 6'd58;
		6'd34: pwm_convert = 6'd39;
		6'd35: pwm_convert = 6'd53;
		6'd36: pwm_convert = 6'd31;
		6'd37: pwm_convert = 6'd48;
		6'd38: pwm_convert = 6'd18;
		6'd39: pwm_convert = 6'd34;
		6'd40: pwm_convert = 6'd51;
		6'd41: pwm_convert = 6'd29;
		6'd42: pwm_convert = 6'd11;
		6'd43: pwm_convert = 6'd13;
		6'd44: pwm_convert = 6'd15;
		6'd45: pwm_convert = 6'd26;
		6'd46: pwm_convert = 6'd22;
		6'd47: pwm_convert = 6'd43;
		6'd48: pwm_convert = 6'd57;
		6'd49: pwm_convert = 6'd38;
		6'd50: pwm_convert = 6'd47;
		6'd51: pwm_convert = 6'd17;
		6'd52: pwm_convert = 6'd28;
		6'd53: pwm_convert = 6'd10;
		6'd54: pwm_convert = 6'd25;
		6'd55: pwm_convert = 6'd21;
		6'd56: pwm_convert = 6'd37;
		6'd57: pwm_convert = 6'd46;
		6'd58: pwm_convert = 6'd9;
		6'd59: pwm_convert = 6'd24;
		6'd60: pwm_convert = 6'd45;
		6'd61: pwm_convert = 6'd8;
		6'd62: pwm_convert = 6'd7;
		6'd63: pwm_convert = 6'd6;
		endcase
	endfunction

	// three pipeline stages, for timing
	reg [12:0] acc = 13'd0;
	reg  [6:0] cnt = 7'd0;

	// ---- stage 1: accumulate ------------------------------------------
	wire [12:0] sum_next = acc + {7'b0, pwm_convert(sample)};
	reg [12:0] sum_final = 13'd0;
	reg        sum_ready = 1'b0;

	always @(posedge clk) begin
		sum_ready <= 1'b0;
		if (sample_en) begin
			if (cnt == 7'd99) begin      // the 100th sample completes the window
				sum_final <= sum_next;
				sum_ready <= 1'b1;
				acc       <= 13'd0;
				cnt       <= 7'd0;
			end else begin
				acc <= sum_next;
				cnt <= cnt + 1'd1;
			end
		end
	end

	// ---- stage 2: scale (sum/10 as sum*205 >> 11, 0.100098) -----------
	reg [23:0] scaled  = 24'd0;
	reg        scl_rdy = 1'b0;
	always @(posedge clk) begin
		scl_rdy <= sum_ready;
		if (sum_ready) scaled <= sum_final * 24'd205;
	end

	// ---- stage 3: offset, clamp, publish ------------------------------
	wire [12:0] tenth = scaled[23:11];
	wire signed [14:0] idx = $signed({2'b0, tenth}) - 15'sd11;
	wire [8:0] idx_clamped = (idx <= 0)        ? 9'd0   :
	                         (idx >= 15'sd399) ? 9'd399 :
	                                             idx[8:0];

	// mid-scale until the Mac has written the buffer
	initial duty_index = 9'd200;

	always @(posedge clk) if (scl_rdy) duty_index <= idx_clamped;

endmodule
