//=============================================================================
// uart_tx.v — 简单 UART 发送（8N1），用于 FPGA 调试输出
//=============================================================================
module uart_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data,
    input  wire       start,
    output reg        tx,
    output reg        busy
);
    localparam integer DIV = CLK_HZ / BAUD;

    reg [15:0] cnt;
    reg [3:0]  bi;
    reg [9:0]  sh;
    reg        run;

    always @(posedge clk) begin
        if (rst) begin
            run <= 1'b0; busy <= 1'b0; tx <= 1'b1; cnt <= 16'd0; bi <= 4'd0; sh <= 10'h3FF;
        end else if (!run) begin
            if (start) begin
                sh   <= {1'b1, data, 1'b0};   // {stop, data[7:0], start}
                run  <= 1'b1;
                busy <= 1'b1;
                bi   <= 4'd0;
                cnt  <= 16'd0;
                tx   <= 1'b0;                 // 起始位
            end
        end else begin
            if (cnt == DIV - 1) begin
                cnt <= 16'd0;
                if (bi == 4'd9) begin
                    run  <= 1'b0;
                    busy <= 1'b0;
                    tx   <= 1'b1;
                end else begin
                    bi <= bi + 4'd1;
                    tx <= sh[bi + 4'd1];
                end
            end else begin
                cnt <= cnt + 16'd1;
            end
        end
    end
endmodule
