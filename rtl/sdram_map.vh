//
// sdram_map.vh -- SDRAM region map in words; regions must not overlap and
// bases are added to payloads (low bits zero below the payload width).
//
`ifndef SDRAM_MAP_VH
`define SDRAM_MAP_VH

// Mac RAM, mapped straight into the 68000's address space. 4MB max.
`define SDRAM_RAM_BASE     25'h0000000

// Floppy image staging.
`define SDRAM_DISK_BASE    25'h0200000

// Boot ROMs: four 512KB slots, boot0..boot3, slot N at N * 512KB.
`define SDRAM_ROM_BASE     25'h0400000

// Byte offsets of each floppy image within the disk region; word = base+off/2.
`define DSK_INT_BYTE_OFF   22'h100000
`define DSK_EXT_BYTE_OFF   22'h200000

`endif
