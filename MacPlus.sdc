derive_pll_clocks
derive_clock_uncertainty

#**************************************************************
# Set Multicycle Path
#**************************************************************

#set_multicycle_path -from {hps_io:hps_io|rtc[*]} -to {dataController_top:dc0|rtc:pram|secs[*]} -setup 4
#set_multicycle_path -from {hps_io:hps_io|rtc[*]} -to {dataController_top:dc0|rtc:pram|secs[*]} -hold 3