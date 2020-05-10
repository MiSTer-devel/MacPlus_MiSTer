derive_pll_clocks
derive_clock_uncertainty

set_multicycle_path -from {emu|m68k|*} -setup 2
set_multicycle_path -from {emu|m68k|*} -hold 1
