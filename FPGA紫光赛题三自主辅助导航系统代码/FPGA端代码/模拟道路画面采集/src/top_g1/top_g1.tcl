set_arch -family Logos -device PGL50H -speedgrade -6 -package FBG484
add_design "E:/FPGA/top_g1/source/top.v"
add_design "E:/FPGA/top_g1/source/timing_gen.v"
add_design "E:/FPGA/top_g1/source/osd_overlay.v"
add_design "E:/FPGA/top_g1/source/osd_alpha_mix.v"
add_design "E:/FPGA/top_g1/source/hdmi_tx.v"
add_design "E:/FPGA/top_g1/source/vendor/sync_vg.v"
add_design "E:/FPGA/top_g1/source/vendor/ms72xx_ctl.v"
add_design "E:/FPGA/top_g1/source/vendor/ms7200_ctl.v"
add_design "E:/FPGA/top_g1/source/vendor/ms7210_ctl.v"
add_design "E:/FPGA/top_g1/source/vendor/iic_dri.v"
add_design "E:/FPGA/top_g1/ipcore/pll/pll.v"
add_constraint "E:/FPGA/top_g1/constr/top_g1.fdc"
compile -top_module top
synthesize -ads -selected_syn_tool_opt 2
dev_map
pnr
exit
