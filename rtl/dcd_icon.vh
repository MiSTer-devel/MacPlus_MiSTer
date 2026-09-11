// dcd_icon.vh - the 32x32 icon a DCD device publishes in its identity block.
//
// 128 bytes of image then 128 bytes of mask, four bytes per row, MSB first.
//
// The drawing is this core's own:
//
//     ................................
//     ................................
//     ................................
//     ................................
//     ................................
//     ................................
//     ................................
//     ................................
//     ..############################..
//     ..#..........................#..
//     ..#..........................#..
//     ..############################..
//     ..#..........................#..
//     ..#..........................#..
//     ..#..####################....#..
//     ..#..#..................#.##.#..
//     ..#..#..................#.##.#..
//     ..#..####################....#..
//     ..#..........................#..
//     ..############################..
//     ..############################..
//     ................................
//     ................................
//     ................................
//     ................................
//     ................................
//     ................................
//     ................................
//     ................................
//     ................................
//     ................................
//     ................................
//
// mask: the solid silhouette

function [31:0] dcd_icon_row;
	input [4:0] row;
	begin
		case (row)
		5'd8:  dcd_icon_row = 32'h3ffffffc;   // top edge
		5'd9:  dcd_icon_row = 32'h20000004;
		5'd10: dcd_icon_row = 32'h20000004;
		5'd11: dcd_icon_row = 32'h3ffffffc;   // lid seam
		5'd12: dcd_icon_row = 32'h20000004;
		5'd13: dcd_icon_row = 32'h20000004;
		5'd14: dcd_icon_row = 32'h27ffff84;   // slot, top
		5'd15: dcd_icon_row = 32'h240000b4;   // slot sides, and the LED
		5'd16: dcd_icon_row = 32'h240000b4;
		5'd17: dcd_icon_row = 32'h27ffff84;   // slot, bottom
		5'd18: dcd_icon_row = 32'h20000004;
		5'd19: dcd_icon_row = 32'h3ffffffc;   // bottom edge
		5'd20: dcd_icon_row = 32'h3ffffffc;   // and its shadow
		default: dcd_icon_row = 32'h00000000;
		endcase
	end
endfunction

function [31:0] dcd_icon_mask;
	input [4:0] row;
	begin
		case (row)
		5'd8,  5'd9,  5'd10, 5'd11, 5'd12, 5'd13, 5'd14,
		5'd15, 5'd16, 5'd17, 5'd18, 5'd19, 5'd20:
		         dcd_icon_mask = 32'h3ffffffc;
		default: dcd_icon_mask = 32'h00000000;
		endcase
	end
endfunction

// Byte k of the 256-byte Icon field: 0..127 image, 128..255 mask.
function [7:0] dcd_icon_byte;
	input [7:0] k;
	reg [31:0] r;
	begin
		r = k[7] ? dcd_icon_mask(k[6:2]) : dcd_icon_row(k[6:2]);
		case (k[1:0])
		2'd0: dcd_icon_byte = r[31:24];
		2'd1: dcd_icon_byte = r[23:16];
		2'd2: dcd_icon_byte = r[15:8];
		2'd3: dcd_icon_byte = r[7:0];
		endcase
	end
endfunction
