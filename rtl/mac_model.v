//
// mac_model.v -- OSD model selection to hardware straps.
// machineType is Plus-vs-SE, not a model index; pre-Plus models want 0.
//
module mac_model
(
	// Latched at reset by MacPlus.sv; the model cannot change mid-run.
	input      [2:0] model,
	input            mem_big,       // OSD Memory, Plus/SE only: 0 = 1MB, 1 = 4MB

	output reg [1:0] configROMSize, // 0 = 64K, 1 = 128K, 2 = 256K, 3 = 512K
	output reg [1:0] configRAMSize, // 0 = 128K, 1 = 512K, 2 = 1MB, 3 = 4MB
	output reg       machineType,   // 0 = Plus-like, 1 = SE
	output reg [1:0] romSlot,       // which bootN.rom to read
	output reg       drive800k,     // 1 = drive can use 800K double-sided media
	// 1 = RAM soldered down and mem_big ignored; greys the OSD Memory option
	output reg       ramSoldered,
	// 1 = SCSI bus present; clear makes the ROM mirror at A17 = 1 (no-SCSI test)
	output reg       scsiPresent
);

	// model 0 must be the Plus: `status` defaults to zero
	localparam [2:0] MODEL_PLUS  = 3'd0;
	localparam [2:0] MODEL_SE    = 3'd1;
	localparam [2:0] MODEL_512K  = 3'd2;
	localparam [2:0] MODEL_128K  = 3'd3;
	localparam [2:0] MODEL_512KE = 3'd4; // last entry in the OSD list - see above

	// slot numbers as Main_MiSTer packs them in ioctl_index[7:6]
	localparam [1:0] ROM_SLOT_PLUS = 2'd0; // releases/boot0.rom, 128K
	localparam [1:0] ROM_SLOT_SE   = 2'd1; // releases/boot1.rom, 256K
	localparam [1:0] ROM_SLOT_64K  = 2'd2; // releases/boot2.rom, 64K

	always @(*) begin
		case (model)
			MODEL_SE: begin
				configROMSize = 2'b10;                    // 256K, releases/boot1.rom
				configRAMSize = mem_big ? 2'b11 : 2'b10;  // 4MB / 1MB
				machineType   = 1'b1;
				romSlot       = ROM_SLOT_SE;
				drive800k     = 1'b1;
				scsiPresent   = 1'b1;
				ramSoldered   = 1'b0;                     // SIMM sockets
			end

			// The 512K and 128K run the 64K ROM in slot 2, have no SCSI, and
			// drive a single-sided mechanism.
			MODEL_512K: begin
				configROMSize = 2'b00;                    // 64K, releases/boot2.rom
				configRAMSize = 2'b01;                    // 512K, soldered
				machineType   = 1'b0;
				romSlot       = ROM_SLOT_64K;
				drive800k     = 1'b0;
				scsiPresent   = 1'b0;
				ramSoldered   = 1'b1;
			end

			MODEL_128K: begin
				configROMSize = 2'b00;                    // 64K, releases/boot2.rom
				configRAMSize = 2'b00;                    // 128K, soldered
				machineType   = 1'b0;
				romSlot       = ROM_SLOT_64K;
				drive800k     = 1'b0;
				scsiPresent   = 1'b0;
				ramSoldered   = 1'b1;
			end

			// A 512Ke is a Plus with 512K soldered in and no SCSI.
			MODEL_512KE: begin
				configROMSize = 2'b01;                    // same 128K ROM as the Plus
				configRAMSize = 2'b01;                    // 512K, soldered
				machineType   = 1'b0;
				romSlot       = ROM_SLOT_PLUS;
				drive800k     = 1'b1;
				scsiPresent   = 1'b0;   // the whole point of the model
				ramSoldered   = 1'b1;
			end

			// MODEL_PLUS, and every reserved encoding
			default: begin
				configROMSize = 2'b01;                    // 128K, releases/boot0.rom
				configRAMSize = mem_big ? 2'b11 : 2'b10;  // 4MB / 1MB
				machineType   = 1'b0;
				romSlot       = ROM_SLOT_PLUS;
				drive800k     = 1'b1;
				scsiPresent   = 1'b1;
				ramSoldered   = 1'b0;                     // SIMM sockets
			end
		endcase
	end

endmodule
