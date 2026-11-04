// acg720_60k.sdc —— 时序约束（W1/W2 阶段只约束输入时钟）
create_clock -name clk50M -period 20 -waveform {0 10} [get_ports {clk50M}]
