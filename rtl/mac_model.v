//
// mac_model.v -- map the OSD model selection onto the hardware straps, in one
// place, so that adding a model is a table entry rather than a hunt through
// MacPlus.sv.
//
// `machineType` is the one to be careful with: it is not a model index but a
// Plus-vs-SE boolean, and dataController_top hangs eight behavioural
// differences off it, including the SE's wholly different (ADB) keyboard.
// Every pre-Plus model wants machineType = 0, not its own model number.
//
// Model numbers are positions in MacPlus.sv's CONF_STR list, not historical
// order, and renumbering one changes what a saved config boots.
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
	// 1 = RAM is soldered down and `mem_big` is ignored, published so the OSD
	// can grey out the Memory option rather than offer a choice the model
	// cannot honour.
	output reg       ramSoldered,
	// 1 = this machine has a SCSI bus, consumed by rtl/addrDecoder.v. When it
	// is clear the ROM window mirrors at A17 = 1, which is how a machine says
	// it has no SCSI: the Plus ROM compares $420000 against $440000
	// ($4003E4), equal means no SCSI, and records it in $0B22 bit 7. Clear
	// also stops $58xxxx decoding. It is the only thing that distinguishes a
	// 512Ke from a Plus, since they run the same ROM; the SE needs no special
	// case, its 256K ROM making A17 a real address bit.
	output reg       scsiPresent
);

	// Model 0 must be the Plus: `status` defaults to zero, so a fresh or
	// reset config must not silently become some other machine.
	localparam [2:0] MODEL_PLUS  = 3'd0;
	localparam [2:0] MODEL_SE    = 3'd1;
	localparam [2:0] MODEL_512K  = 3'd2;
	localparam [2:0] MODEL_128K  = 3'd3;
	localparam [2:0] MODEL_512KE = 3'd4; // last entry in the OSD list - see above

	// Slot numbers, not one-hot bits: Main_MiSTer packs the slot as a plain
	// binary count in ioctl_index[7:6], so a slot number is the boot file number.
	localparam [1:0] ROM_SLOT_PLUS = 2'd0; // releases/boot0.rom, 128K
	localparam [1:0] ROM_SLOT_SE   = 2'd1; // releases/boot1.rom, 256K
	localparam [1:0] ROM_SLOT_64K  = 2'd2; // releases/boot2.rom, user's choice of 64K image

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

			// The 512K and 128K run the 64K ROM in slot 2. RAM is soldered and
			// its size is the one hardware difference between them. The 64K ROM
			// has no SCSI Manager, and both shipped a mechanically single-sided
			// drive, so only side 0 of an 800K image is ever read.
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

			// A 512Ke is a Plus with 512K soldered in and no SCSI: the same
			// 128K ROM and the same 800K drive, so the RAM strap and
			// scsiPresent are the entire model. See the port comment above for
			// how a machine tells its own ROM that it has no SCSI.
			MODEL_512KE: begin
				configROMSize = 2'b01;                    // same 128K ROM as the Plus
				configRAMSize = 2'b01;                    // 512K, soldered
				machineType   = 1'b0;
				romSlot       = ROM_SLOT_PLUS;
				drive800k     = 1'b1;
				scsiPresent   = 1'b0;   // the whole point of the model
				ramSoldered   = 1'b1;
			end

			// MODEL_PLUS, and every reserved encoding. Falling back to the Plus
			// keeps a stale saved config harmless instead of unbootable.
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
