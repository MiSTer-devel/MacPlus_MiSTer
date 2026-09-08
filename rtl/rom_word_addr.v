//
// rom_word_addr.v -- the SDRAM word address for one 512KB boot-ROM slot.
//
// MacPlus.sv uses this on two independent paths: the download side decides
// which slot an incoming byte belongs to, and the read side decides which
// slot the running model reads from. Instantiating one module on both keeps
// the two from drifting apart.
//
// Main_MiSTer sends slot N as `i << 6` in ioctl_index for i = 0..3
// (user_io.cpp, user_io_file_tx), which packs it as ioctl_index[7:6]. Four
// slots is Main_MiSTer's own ceiling -- its loader loop is `i < 4` -- so a
// fifth boot ROM needs a change there too, not just here.
//
module rom_word_addr
(
	input      [1:0]  slot,        // which bootN.rom: 0, 1, 2 or 3
	input      [17:0] word_offset, // word address within that 512KB image
	output     [20:0] addr         // word address within the reserved ROM region
);

	assign addr = {1'b0, slot, word_offset};

endmodule
