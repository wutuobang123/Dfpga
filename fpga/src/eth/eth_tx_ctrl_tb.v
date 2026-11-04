/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : eth_tx_ctrl_tb
// Description   : eth_tx_ctrl仿真文件
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ns
`define CLK1_PRIOD  40
`define CLK2_PRIOD  8

module eth_tx_ctrl_tb;

  reg          reset_p          ;

  reg          clk              ;
  reg   [15:0] data_i           ;
  reg          data_valid_i     ;

  reg          eth_txfifo_rd_clk;
  wire         tx_go            ;
  reg          tx_done          ;
  reg          eth_txfifo_rden  ;
  wire  [7:0]  eth_txfifo_dout  ;

initial clk = 1'b1;
always #(`CLK1_PRIOD/2)clk = ~clk;

initial eth_txfifo_rd_clk = 1'b1;
always #(`CLK2_PRIOD/2)eth_txfifo_rd_clk = ~eth_txfifo_rd_clk;

initial begin
  reset_p = 1'b1;
  #201;
  reset_p = 1'b0;
  #500;
  send_data(50,100);
  
  #2000;
  $stop;
end

task send_data;
  input [15:0]data_cnt;
  input [15:0]data_begin;
  begin
    data_valid_i = 1'b0;
    data_i = data_begin;
    @(posedge clk);
    #1 data_valid_i = 1'b1;
    repeat(data_cnt)
    begin
      @(posedge clk);
      #1 data_i = data_i + 1;
    end
    data_valid_i = 1'b0;    
  end
endtask

initial begin
  eth_txfifo_rden = 1'b0;
  tx_done = 1'b0;
  #500;
  forever begin
    @(negedge tx_go)
    #20;
    @(posedge eth_txfifo_rd_clk)
    #1 eth_txfifo_rden = 1;
    
    repeat(20)
      #`CLK2_PRIOD;
    
    eth_txfifo_rden = 0;
    
    tx_done = 1;
    #`CLK2_PRIOD;
    tx_done = 0;
  end
end

eth_tx_ctrl
#(                            
  .PAYLOAD_DATA_BYTE (2  ),
  .PAYLOAD_LENGTH    (10 )
)eth_tx_ctrl_inst
(
  .reset_p          (reset_p          ),

  .clk              (clk              ),
  .data_i           (data_i           ),
  .data_valid_i     (data_valid_i     ),

  .eth_txfifo_rd_clk(eth_txfifo_rd_clk),
  .tx_go            (tx_go            ),
  .tx_done          (tx_done          ),
  .eth_txfifo_rden  (eth_txfifo_rden  ),
  .eth_txfifo_dout  (eth_txfifo_dout  )
);

endmodule