//
// sdram_map.vh -- the SDRAM region map, in word units as sdram.v sees them.
// Consumed by MacPlus.sv's sdram_addr mux and by rtl/addrController_top.v.
// Regions must not overlap, and only sdram_addr[22:0] reaches the chip, so
// keep every base within the low 8M words. Bases are added to their payloads
// rather than concatenated, which is exact only while each base's low bits
// are zero below the payload width.
//
`ifndef SDRAM_MAP_VH
`define SDRAM_MAP_VH

// Mac RAM, mapped straight into the 68000's address space. 4MB max.
`define SDRAM_RAM_BASE     25'h0000000

// Floppy image staging.
`define SDRAM_DISK_BASE    25'h0200000

// Boot ROMs: four 512KB slots, boot0..boot3 (Main_MiSTer's loader loop is
// `i < 4`, so four is its own ceiling). Slot N starts at N * 512KB.
`define SDRAM_ROM_BASE     25'h0400000

// Byte offsets of each floppy image within the disk region; word = base+off/2.
`define DSK_INT_BYTE_OFF   22'h100000
`define DSK_EXT_BYTE_OFF   22'h200000

`endif
