// CPU wait states of the real Plus and SE at 8 MHz, from the Plus TSM PAL.
// phi = C8M clock (0-7) within the C1M period in which S0 falls:
//
//   ROM, IWM, SCSI, RAM in blanking   waits = (1 - phi) mod 4
//   RAM during display, Plus          waits = (5 - phi) mod 8
//   RAM during display, SE            the grid, less video's 1 slot in 4 (assumed)
//   SCC                               waits = 1 + (4 - phi) mod 8
//
// fx68k samples DTACK on phi2 of S4, so each rule is a release window at
// phase = target + 2. The VIA is E-clock timed and is not held here.
//
// Display follows the Mac raster, one line per sound word from the VBL:
// lines 0-27 blank, then the first 256 of every 352 CPU clocks, plus the
// sound slot at the end of each line.
module cpu_wait(
	input        clk,
	input        clk8_en_p,     // CPU phi1, once per 8 MHz clock
	input        authentic,     // 8 MHz authentic timing; 0 under turbo
	input        machineType,   // 0 = Plus/512K/128K, 1 = SE
	input        _vblank,       // falling edge = VBL = line 0
	input        snd_advance,   // one pulse per sound word = per Mac line
	input        _cpuAS,
	input        selectRAM,
	input        selectROM,
	input        selectSCSI,
	input        selectIWM,
	input        selectSCC,
	output [3:0] phase,         // C8M clock within the 16-clock slot frame
	output       display,       // video owns its RAM slots now
	output       hold           // 1 = withhold DTACK
);

	// 16-clock slot frame: the Plus uses phase[2:0] as C1M, the SE all four
	reg [3:0] ph = 4'd0;
	always @(posedge clk) if (clk8_en_p) ph <= ph + 4'd1;
	assign phase = ph;

	// Mac raster in CPU clocks: 352 per line, 256 display, sound slot last 8
	localparam [8:0] FIRST_DISPLAY_LINE = 9'd28;
	localparam [8:0] DISPLAY_CLOCKS     = 9'd256;
	localparam [8:0] SOUND_SLOT_CLOCK   = 9'd344;

	reg       vblankD = 1'b1, frame_pending = 1'b0;
	reg [8:0] line = 9'd0, xclk = 9'd0;
	always @(posedge clk) if (clk8_en_p) begin
		vblankD <= _vblank;
		if (vblankD && !_vblank) frame_pending <= 1'b1;
		if (snd_advance) begin
			// the scan's first advance follows the VBL edge by one clock
			line          <= frame_pending ? 9'd0 : line + 9'd1;
			frame_pending <= 1'b0;
			xclk          <= 9'd0;
		end else if (xclk != 9'h1FF)
			xclk <= xclk + 9'd1;
	end
	assign display = (line >= FIRST_DISPLAY_LINE && xclk < DISPLAY_CLOCKS) ||
	                 xclk >= SOUND_SLOT_CLOCK;

	// clocks since AS fell, to keep the SCC from completing on its first sample
	reg [1:0] as_age = 2'd0;
	always @(posedge clk)
		if (_cpuAS) as_age <= 2'd0;
		else if (clk8_en_p && as_age != 2'd3) as_age <= as_age + 2'd1;

	// release windows: phase = target + 2
	wire grid_ok    = ph[1:0] == 2'd3;                       // target 1 mod 4
	wire plus_slot  = ph[2:0] == 3'd7;                       // target 5 mod 8
	wire se_slot    = grid_ok && ph != 4'd3;                 // not video's slot
	wire ram_ok     = !display ? grid_ok : machineType ? se_slot : plus_slot;
	wire scc_ok     = plus_slot && as_age >= 2'd2;

	wire release_ok = selectSCC ? scc_ok :
	                  selectRAM ? ram_ok :
	                  (selectROM | selectIWM | selectSCSI) ? grid_ok : 1'b1;

	assign hold = authentic && !_cpuAS && !release_ok;

endmodule
