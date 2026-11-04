//=============================================================================
// dbg_stream.v — 周期性把 FPGA 内部状态通过 UART 打出来
//   帧格式: 0xA5 | 8 字节负载 | 0x5A
//   负载: ok_frames(2) err_pkts(2) status(1) disp_cnt(2) nz_lo(1)
//   status: bit0=ovl_en bit1=active_sel bit2=wr_sel bit3=flip_pending
//=============================================================================
module dbg_stream #(
    parameter integer CLK_HZ    = 50_000_000,
    parameter integer PERIOD_MS = 200
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] ok_frames,
    input  wire [15:0] err_pkts,
    input  wire [7:0]  status,
    input  wire [15:0] disp_cnt,
    input  wire [7:0]  key_hits,   // 按键次数（调试）
    input  wire [15:0] eth_frames,   // 收到的以太网帧总数（验证 RX 通路）
    input  wire [15:0] udp_pkts,     // UDP 层交付给解析器的包数
    output reg  [7:0]  tx_data,
    output reg         tx_start,
    input  wire        tx_busy
);
    localparam integer PERIOD = CLK_HZ / 1000 * PERIOD_MS;

    reg [31:0] timer;
    reg [3:0]  seq;
    reg        active;
    reg [15:0] ok_l, err_l, disp_l, eth_l, udp_l;
    reg [7:0]  nz_l;
    reg [7:0]  st_l;

    always @(posedge clk) begin
        if (rst) begin
            timer <= 32'd0; seq <= 4'd0; active <= 1'b0;
            tx_data <= 8'd0; tx_start <= 1'b0;
            ok_l <= 16'd0; err_l <= 16'd0; disp_l <= 16'd0; nz_l <= 16'd0; eth_l <= 16'd0; udp_l <= 16'd0; st_l <= 8'd0;
        end else begin
            tx_start <= 1'b0;

            if (!active) begin
                if (timer >= PERIOD) begin
                    timer  <= 32'd0;
                    active <= 1'b1;
                    seq    <= 4'd0;
                    // 锁存一份快照，保证一帧内数据一致
                    ok_l <= ok_frames; err_l <= err_pkts;
                    disp_l <= disp_cnt; nz_l <= key_hits; eth_l <= eth_frames; udp_l <= udp_pkts; st_l <= status;
                end else begin
                    timer <= timer + 32'd1;
                end
            end else if (!tx_busy && !tx_start) begin
                case (seq)
                    4'd0: tx_data <= 8'hA5;
                    4'd1: tx_data <= ok_l[15:8];
                    4'd2: tx_data <= ok_l[7:0];
                    4'd3: tx_data <= err_l[15:8];
                    4'd4: tx_data <= err_l[7:0];
                    4'd5: tx_data <= st_l;
                    4'd6: tx_data <= disp_l[15:8];
                    4'd7: tx_data <= disp_l[7:0];
                    4'd8: tx_data <= nz_l[7:0];
                    4'd9: tx_data <= eth_l[15:8];
                    4'd10: tx_data <= eth_l[7:0];
                    4'd11: tx_data <= udp_l[15:8];
                    4'd12: tx_data <= udp_l[7:0];
                    default: tx_data <= 8'h5A;
                endcase
                tx_start <= 1'b1;
                if (seq == 4'd13) begin
                    seq    <= 4'd0;
                    active <= 1'b0;
                end else begin
                    seq <= seq + 4'd1;
                end
            end
        end
    end
endmodule