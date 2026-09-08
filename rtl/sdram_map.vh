//
// sdram_map.vh -- the SDRAM region map, in one place, in WORD units as
// sdram.v sees them. Consumed by MacPlus.sv's sdram_addr mux and by
// rtl/addrController_top.v. Regions must not overlap: ROM used to sublet the
// disk region's first megabyte, which is only two 512KB slots, so a third
// boot ROM landed on top of the internal floppy image.
//
// Two cautions if you widen this. MacPlus.sv drives a 25-bit sdram_addr into
// sdram.v's 24-bit port and only bits [22:0] reach the chip, so keep every
// base inside SDRAM_USABLE_WORDS. And bases are ADDED to their payloads, not
// concatenated -- exact only while each base's low bits are zero below the
// payload width.
//
`ifndef SDRAM_MAP_VH
`define SDRAM_MAP_VH

// Addressable space: sdram.v decodes only addr[22:0]. 8M words = 16MB.
`define SDRAM_USABLE_WORDS 25'h0800000

// Mac RAM, mapped straight into the 68000's address space. 4MB max.
`define SDRAM_RAM_BASE     25'h0000000
`define SDRAM_RAM_WORDS    25'h0200000

// Floppy image staging, historically "extra rom".
`define SDRAM_DISK_BASE    25'h0200000

// Boot ROMs: four 512KB slots, boot0..boot3 (Main_MiSTer's loader loop is
// `i < 4`, so four is its own ceiling). rom_word_addr.v places slot N.
`define SDRAM_ROM_BASE     25'h0400000
`define SDRAM_ROM_SLOTS    4
`define SDRAM_ROM_SLOT_WORDS 25'h0040000

// Byte offsets of each floppy image within the disk region; word = base+off/2.
`define DSK_INT_BYTE_OFF   22'h100000
`define DSK_EXT_BYTE_OFF   22'h200000

// Largest image either drive accepts: 819200 bytes (800K double-sided).
`define DSK_IMAGE_MAX_BYTES 25'h00C8000

`endif
