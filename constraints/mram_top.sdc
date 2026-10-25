# mram_top.sdc -- timing constraints TEMPLATE for mram_top (SRAM boot copy build)
#
# This is a template, not a drop-in file. Before use:
#   1. Replace the placeholders:
#        <MRAM>        hierarchical path of the mram_top instance in the
#                      post-synthesis netlist (as SmartTime shows it)
#        <ACLK>        the 150 MHz clock of aclk, as already defined in the
#                      project (usually derived from the CCC/PLL; do not
#                      define it again here)
#        <PCI_CLK>     the 33 MHz PCI clock that clocks the PCI-side
#                      registers driving cmd_*, boot_hold, key_ok,
#                      rd_sample_dly, skip_wren, sram_reload and reading
#                      cmd_done, cmd_rd*_data, boot_status, boot_done,
#                      boot_fail, mram_wr_pending, mram_rd_slip
#        <CPU_RST_SYNC> the first register of the CPU-side reset
#                      synchronizer that cpu_reset_n feeds
#   2. In SmartTime's constraint editor, check that every command below
#      matches the objects it is meant to (a constraint that matches
#      nothing is silently useless). I could not check the netlist names
#      Synplify gives VHDL arrays and generate blocks, so the patterns use
#      wildcards; tighten them once you have seen the names.
#   3. All delay values marked PROPOSED are my engineering estimates, not
#      datasheet figures. See doc/LIBERO_INTEGRATION.md, section 4.
#
# Every signal of mram_top that crosses from another clock domain into
# aclk goes through one of these synchronizers (2 flops) or is sampled only
# under a handshake:
#   cmd_* requests, boot_hold   -> <MRAM>/u_cmd_ctrl/req_meta (stage 0)
#   cmd_wrsr_data, cmd_reg_addr -> sampled into u_cmd_ctrl/reg_cmd_wdata,
#                                  reg_cmd_addr only after the request is
#                                  seen through the synchronizer
#   key_ok                      -> <MRAM>/u_write_guard/key_ok_meta
#   rd_sample_dly               -> <MRAM>/u_backend/dly_q1   (quasi-static)
#   skip_wren                   -> <MRAM>/u_backend/skip_q1  (quasi-static)
#   sram_reload                 -> <MRAM>/reload_q1
# and every output read in another domain is an aclk register that the
# receiving side synchronizes (cmd_done) or reads only under a handshake
# or as a quasi-static status (cmd_rd*_data, boot_status, boot_done,
# boot_fail, mram_wr_pending, mram_rd_slip).

# ---------------------------------------------------------------------------
# 1. PCI clock <-> aclk: asynchronous, all crossings synchronized or
#    handshake-protected (list above). Cut the timing paths in both
#    directions.
# ---------------------------------------------------------------------------
set_false_path -from [get_clocks {<PCI_CLK>}] -to [get_clocks {<ACLK>}]
set_false_path -from [get_clocks {<ACLK>}] -to [get_clocks {<PCI_CLK>}]

# If any of the inputs above is driven from a domain other than <PCI_CLK>
# (for example a register on the CPU clock), cut it at the synchronizer
# input instead, for example:
# set_false_path -to [get_cells {<MRAM>/reload_q1*}]
# set_false_path -to [get_cells {<MRAM>/u_cmd_ctrl/req_meta*}]
# set_false_path -to [get_cells {<MRAM>/u_write_guard/*key_ok_meta*}]
# set_false_path -to [get_cells {<MRAM>/u_backend/dly_q1*}]
# set_false_path -to [get_cells {<MRAM>/u_backend/skip_q1*}]

# ---------------------------------------------------------------------------
# 2. cpu_reset_n -> CPU clock domain (50 MHz). cpu_reset_n is an aclk
#    register; the CPU side must have a reset synchronizer (open item OA6).
#    Do NOT cut aclk <-> CPU clock as a whole: the interconnect's own CDC
#    sits between them and needs its own (vendor) constraints.
# ---------------------------------------------------------------------------
set_false_path -from [get_cells {<MRAM>/u_boot_copy/cpu_reset_n*}] -to [get_cells {<CPU_RST_SYNC>}]

# ---------------------------------------------------------------------------
# 3. MRAM QSPI pins (mram_cs_n, mram_sclk, mram_io[3:0]).
#    SCLK, CS# and the IO outputs are all aclk registers; SCLK = aclk/4, so
#    the device sees each IO change half an SCLK period (2 aclk = 13.3 ns)
#    before the SCLK rising edge that samples it. Read data is sampled
#    rd_sample_dly aclk cycles after the FPGA's SCLK rising edge, which is
#    tuned on the board (README, "Read sample delay"). What matters is that
#    the pad delays are short and similar for SCLK and IO, so they are
#    bounded here rather than modelled exactly.
#    PROPOSED values; check the SmartTime report and the board margin.
# ---------------------------------------------------------------------------
set_max_delay 7.0 -from [get_clocks {<ACLK>}] -to [get_ports {mram_cs_n mram_sclk mram_io[*]}]
set_max_delay 7.0 -from [get_ports {mram_io[*]}] -to [get_clocks {<ACLK>}]

# ---------------------------------------------------------------------------
# 4. Everything else in mram_top, including the new m_axi_* port to the
#    interconnect (INITIATOR2), is synchronous to aclk and is timed by the
#    <ACLK> clock constraint itself. No exceptions.
# ---------------------------------------------------------------------------
