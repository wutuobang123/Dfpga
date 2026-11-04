//=============================================================================
// tft_ctrl.v  —  5 寸 800x480 RGB565 并口驱动
//   ★ 管脚表：TFT_rgb[15:11]=B, [10:5]=G, [4:0]=R（高位是蓝，别接反）
//   DE 模式下 HS/VS 取负极性；若画面偏移只改 porch 参数，不影响时序正确性
//=============================================================================
`timescale 1ns / 1ps

module tft_ctrl #(
    parameter integer H_ACTIVE = 800,
    parameter integer H_FP     = 40,
    parameter integer H_SW     = 48,
    parameter integer H_BP     = 168,
    parameter integer V_ACTIVE = 480,
    parameter integer V_FP     = 13,
    parameter integer V_SW     = 3,
    parameter integer V_BP     = 90
)(
    input  wire        clk,          // 37.125MHz
    input  wire        rst_n,
    output wire        tft_clk,
    output reg         tft_de,
    output reg         tft_hs,
    output reg         tft_vs,
    output reg  [15:0] tft_rgb
);

    wire        hs, vs, de, frame;
    wire [11:0] x, y;
    wire [23:0] rgb;

    video_timing #(
        .H_ACTIVE(H_ACTIVE), .H_FP(H_FP), .H_SW(H_SW), .H_BP(H_BP),
        .V_ACTIVE(V_ACTIVE), .V_FP(V_FP), .V_SW(V_SW), .V_BP(V_BP),
        .HS_POL(1'b0), .VS_POL(1'b0)
    ) u_timing (
        .pix_clk(clk), .rst_n(rst_n),
        .hs(hs), .vs(vs), .de(de),
        .x(x), .y(y), .frame_tick(frame)
    );

    test_pattern #(
        .H_ACTIVE(H_ACTIVE), .V_ACTIVE(V_ACTIVE), .BAR_W(H_ACTIVE/8)
    ) u_pattern (
        .pix_clk(clk), .rst_n(rst_n), .frame_tick(frame),
        .x(x), .y(y), .de(de), .rgb(rgb)
    );

    assign tft_clk = clk;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tft_de  <= 1'b0;
            tft_hs  <= 1'b0;
            tft_vs  <= 1'b0;
            tft_rgb <= 16'h0000;
        end else begin
            tft_de  <= de;
            tft_hs  <= hs;
            tft_vs  <= vs;
            tft_rgb <= { rgb[7:3],     // B[7:3] -> [15:11]
                         rgb[15:10],   // G[7:2] -> [10:5]
                         rgb[23:19] }; // R[7:3] -> [4:0]
        end
    end

endmodule
