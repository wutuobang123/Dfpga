@echo off
REM 一键跑三个仿真（ModelSim 需在 PATH 里，或改成绝对路径）
cd /d %~dp0
if exist work rmdir /s /q work
vlib work
vlog ../src/camera/dvp_packer.v ../src/eth/eth_tx_fifo.v tb_dvp_packer.v tb_eth_tx_fifo.v tb_stream.v
echo ==== TB_DVP_PACKER ====
vsim -c -quiet work.tb_dvp_packer -do "run -all; quit -f"
echo ==== TB_ETH_TX_FIFO ====
vsim -c -quiet work.tb_eth_tx_fifo -do "run -all; quit -f"
echo ==== TB_STREAM ====
vsim -c -quiet work.tb_stream -do "run -all; quit -f"
