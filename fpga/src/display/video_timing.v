//=============================================================================
// video_timing.v  —  参数化视频时序发生器（HDMI / TFT 共用）
//   hcnt 0 .. H_TOTAL-1，vcnt 0 .. V_TOTAL-1；行内顺序 = 有效区 -> 前肩 -> 同步 -> 后肩
//=============================================================================
`timescale 1ns / 1ps

module video_timing #(
    parameter integer H_ACTIVE = 1280,
    parameter integer H_FP     = 110,
    parameter integer H_SW     = 40,
    parameter integer H_BP     = 220,
    parameter integer V_ACTIVE = 720,
    parameter integer V_FP     = 5,
    parameter integer V_SW     = 5,
    parameter integer V_BP     = 20,
    parameter         HS_POL   = 1'b1,
    parameter         VS_POL   = 1'b1
)(
    input  wire        pix_clk,
    input  wire        rst_n,
    output wire        hs,
    output wire        vs,
    output wire        de,
    output wire [11:0] x,
    output wire [11:0] y,
    output wire        frame_tick      // 每帧第一个像素处 1 个时钟脉冲
);

    localparam [11:0] H_TOTAL = H_ACTIVE + H_FP + H_SW + H_BP;
    localparam [11:0] V_TOTAL = V_ACTIVE + V_FP + V_SW + V_BP;

    reg [11:0] hcnt;
    reg [11:0] vcnt;

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            hcnt <= 12'd0;
            vcnt <= 12'd0;
        end else begin
            if (hcnt == H_TOTAL - 12'd1) begin
                hcnt <= 12'd0;
                vcnt <= (vcnt == V_TOTAL - 12'd1) ? 12'd0 : (vcnt + 12'd1);
            end else begin
                hcnt <= hcnt + 12'd1;
            end
        end
    end

    wire h_sync_raw = (hcnt >= H_ACTIVE) && (hcnt < (H_ACTIVE + H_FP + H_SW));
    wire v_sync_raw = (vcnt >= V_ACTIVE) && (vcnt < (V_ACTIVE + V_FP + V_SW));

    assign hs = HS_POL ? h_sync_raw : ~h_sync_raw;
    assign vs = VS_POL ? v_sync_raw : ~v_sync_raw;

    assign de = (hcnt < H_ACTIVE) && (vcnt < V_ACTIVE);
    assign x  = hcnt;
    assign y  = vcnt;

    assign frame_tick = (hcnt == 12'd0) && (vcnt == 12'd0);

endmodule
