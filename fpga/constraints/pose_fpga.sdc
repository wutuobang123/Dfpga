//=============================================================================
// pose_fpga.sdc — 时序约束（本工程此前完全没有 SDC，导致 125MHz 逻辑静默违例）
//   关键路径：RGMII 接收 125MHz 逻辑域（UDP 解析 / SK01 组帧）
//=============================================================================

// ---- 主时钟 ----
create_clock -name clk50m    -period 20.000 -waveform {0 10.000} [get_ports {clk50m}]
create_clock -name rgmii_rxc -period  8.000 -waveform {0  4.000} [get_ports {rgmii_rxc}]

// ---- RGMII 接收输入延迟：PHY 内部已对 RXC 加约 90 度延迟，数据窗口中心对齐 ----
set_input_delay -clock rgmii_rxc -max 2.000 [get_ports {rgmii_rxd[0] rgmii_rxd[1] rgmii_rxd[2] rgmii_rxd[3] rgmii_rxdv}]
set_input_delay -clock rgmii_rxc -min 0.500 [get_ports {rgmii_rxd[0] rgmii_rxd[1] rgmii_rxd[2] rgmii_rxd[3] rgmii_rxdv}]
set_input_delay -clock rgmii_rxc -clock_fall -max 2.000 -add_delay [get_ports {rgmii_rxd[0] rgmii_rxd[1] rgmii_rxd[2] rgmii_rxd[3] rgmii_rxdv}]
set_input_delay -clock rgmii_rxc -clock_fall -min 0.500 -add_delay [get_ports {rgmii_rxd[0] rgmii_rxd[1] rgmii_rxd[2] rgmii_rxd[3] rgmii_rxdv}]

// ---- 异步时钟域之间不做时序分析 ----
set_clock_groups -asynchronous -group {clk50m} -group {rgmii_rxc}
