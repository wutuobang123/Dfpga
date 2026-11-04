//=============================================================================
// test_pattern.v  —  W1/W2 用测试图：8 色竖条 + 绿色边框 + 随帧移动的白色方块
//   移动方块用于肉眼判断"画面是活的"以及是否有撕裂/掉帧
//=============================================================================
`timescale 1ns / 1ps

module test_pattern #(
    parameter integer H_ACTIVE = 1280,
    parameter integer V_ACTIVE = 720,
    parameter integer BAR_W    = 160
)(
    input  wire        pix_clk,
    input  wire        rst_n,
    input  wire        frame_tick,
    input  wire [11:0] x,
    input  wire [11:0] y,
    input  wire        de,
    output reg  [23:0] rgb
);

    // ---- 移动方块横坐标 ----
    reg [11:0] box_x;
    wire [12:0] box_next = {1'b0, box_x} + 13'd64;

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n)             box_x <= 12'd0;
        else if (frame_tick)    box_x <= (box_next >= H_ACTIVE) ? 12'd0 : box_next[11:0];
    end

    // ---- 8 色竖条 ----
    wire [2:0] bar = x / BAR_W;
    reg  [23:0] bar_rgb;

    always @(*) begin
        case (bar)
            3'd0:    bar_rgb = 24'hFF_FF_FF;   // 白
            3'd1:    bar_rgb = 24'hFF_FF_00;   // 黄
            3'd2:    bar_rgb = 24'h00_FF_FF;   // 青
            3'd3:    bar_rgb = 24'h00_FF_00;   // 绿
            3'd4:    bar_rgb = 24'hFF_00_FF;   // 品红
            3'd5:    bar_rgb = 24'hFF_00_00;   // 红
            3'd6:    bar_rgb = 24'h00_00_FF;   // 蓝
            default: bar_rgb = 24'h00_00_00;   // 黑
        endcase
    end

    // ---- 叠加层 ----
    wire [12:0] x13 = {1'b0, x};
    wire [12:0] y13 = {1'b0, y};

    localparam [12:0] BOX_Y0 = (V_ACTIVE / 2) - 32;

    wire in_box_x = (x13 >= {1'b0, box_x}) && (x13 < ({1'b0, box_x} + 13'd64));
    wire in_box_y = (y13 >= BOX_Y0) && (y13 < (BOX_Y0 + 13'd64));

    wire border = (x == 12'd0) || (y == 12'd0) ||
                  (x == (H_ACTIVE - 1)) || (y == (V_ACTIVE - 1));

    always @(*) begin
        if (!de)                       rgb = 24'h00_00_00;
        else if (in_box_x && in_box_y) rgb = 24'hFF_FF_FF;
        else if (border)               rgb = 24'h00_FF_00;
        else                           rgb = bar_rgb;
    end

endmodule
