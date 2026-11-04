/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : eth_tx_ctrl
// Description   : 以太网发送数据控制模块
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module eth_tx_ctrl
#(
  parameter PAYLOAD_DATA_BYTE = 2,
  parameter PAYLOAD_LENGTH    = 800
)
(
  input                            reset_p          ,

  input                            clk              ,
  input  [PAYLOAD_DATA_BYTE*8-1:0] data_i           ,
  input                            data_valid_i     ,

  input                            eth_txfifo_rd_clk,
  output reg                       tx_en_pulse      ,
  input                            tx_done          ,
  input                            eth_txfifo_rden  ,
  output    [7:0]                  eth_txfifo_dout  
);

localparam IDLE       = 4'b0001,
           TX_START   = 4'b0010,
           WAIT_DONE  = 4'b0100,
           DELAY      = 4'b1000;

localparam TX_DATA_BYTE = PAYLOAD_LENGTH * PAYLOAD_DATA_BYTE - 2'd2;

wire [13:0] rd_data_count;
reg  [3:0]  eth_interval_cnt;
reg  [3:0]  curr_state;
reg  [3:0]  next_state;

	eth_tx_fifo eth_tx_fifo(
		.Data(data_i), //input [15:0] Data
		.Reset(reset_p), //input Reset
		.WrClk(clk), //input WrClk
		.RdClk(eth_txfifo_rd_clk), //input RdClk
		.WrEn(data_valid_i), //input WrEn
		.RdEn(eth_txfifo_rden), //input RdEn
		.Rnum(rd_data_count), //output [13:0] Rnum
		.Almost_Empty(), //output Almost_Empty
		.Almost_Full(), //output Almost_Full
		.Q(eth_txfifo_dout), //output [7:0] Q
		.Empty(), //output Empty
		.Full() //output Full
	);

  always@(posedge eth_txfifo_rd_clk or posedge reset_p)
  begin
    if(reset_p)
      tx_en_pulse <= 1'b0;
    else if(curr_state == TX_START)
      tx_en_pulse <= 1'b1;
    else
      tx_en_pulse <= 1'b0;
  end

  always@(posedge eth_txfifo_rd_clk or posedge reset_p)
  begin
    if(reset_p)
      eth_interval_cnt <= 1'b0;
    else if(curr_state == DELAY)
      eth_interval_cnt <= eth_interval_cnt + 1'b1;
    else
      eth_interval_cnt <=  1'b0;
  end

//------------------------
//State machine
//------------------------
  always@(posedge eth_txfifo_rd_clk or posedge reset_p)
  begin
    if(reset_p)
      curr_state <= 1'b0;
    else
      curr_state <= next_state;
  end

  always@(*)
  begin
    case(curr_state)
      IDLE:
      begin
        if(rd_data_count >= TX_DATA_BYTE)
          next_state = TX_START;
        else
          next_state = curr_state;
      end
      
      TX_START: next_state = WAIT_DONE;
        
      WAIT_DONE:
      begin
        if(tx_done)
          next_state = DELAY;
        else
          next_state = curr_state;
      end

      DELAY:
      begin
        if(eth_interval_cnt == 4'd7)
          next_state = IDLE;
        else
          next_state = curr_state;
      end
      
      default: next_state = IDLE;
    endcase
  end
 
endmodule