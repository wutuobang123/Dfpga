//=============================================================================
// tft_preview.v — 5 寸屏本地预览：320x240 帧存 + 2 倍放大居中
//   写侧：摄像头像素时钟域，2x2 抽样后写入 BSRAM
//   读侧：37.125MHz 面板时钟域，按显示坐标反算源地址（最近邻 2 倍放大）
//   800x480 面板上画面占 640x480 居中，左右各 80 像素黑边
//=============================================================================
`timescale 1ns / 1ps

module tft_preview #(
    parameter integer H_ACTIVE = 800,
    parameter integer H_FP     = 40,
    parameter integer H_SW     = 48,
    parameter integer H_BP     = 168,
    parameter integer V_ACTIVE = 480,
    parameter integer V_FP     = 13,
    parameter integer V_SW     = 3,
    parameter integer V_BP     = 90,
    parameter integer IMG_W    = 320,
    parameter integer IMG_H    = 240
)(
    input  wire        pix_clk,      // 37.125MHz
    input  wire        rst_n,

    // 写口（pclk 域）
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [16:0] wr_addr,
    input  wire [15:0] wr_data,

    // 骨架位图（SK01），读口在 pix_clk 域
    input  wire        ovl_en,
    output wire        disp_tick,    // 显示帧起始脉冲，给骨架缓冲做同步翻转
    output wire [13:0] ovl_addr,
    input  wire [7:0]  ovl_dat,

    // 面板
    output wire        tft_clk,
    output reg         tft_de,
    output reg         tft_hs,
    output reg         tft_vs,
    output reg  [15:0] tft_rgb
);

    localparam integer IMG_PIX = IMG_W * IMG_H;
    localparam integer X_OFF   = (H_ACTIVE - IMG_W * 2) / 2;   // 80

    (* syn_ramstyle = "block_ram" *) reg [15:0] mem [0:IMG_PIX-1];

    // 摄像头 RGB565 是 R[15:11] 在高位，这块面板是 B[15:11] 在高位，写入时交换
    wire [15:0] wr_data_panel = {wr_data[4:0], wr_data[10:5], wr_data[15:11]};

    always @(posedge wr_clk) begin
        if (wr_en) mem[wr_addr] <= wr_data_panel;
    end

    // ---- 时序 ----
    wire        hs, vs, de;
    wire [11:0] x, y;

    video_timing #(
        .H_ACTIVE(H_ACTIVE), .H_FP(H_FP), .H_SW(H_SW), .H_BP(H_BP),
        .V_ACTIVE(V_ACTIVE), .V_FP(V_FP), .V_SW(V_SW), .V_BP(V_BP),
        .HS_POL(1'b0), .VS_POL(1'b0)
    ) u_timing (
        .pix_clk(pix_clk), .rst_n(rst_n),
        .hs(hs), .vs(vs), .de(de),
        .x(x), .y(y), .frame_tick(disp_tick)
    );

    // ---- 读地址：显示坐标 -> 源坐标（2 倍放大）----
    wire in_view = (x >= X_OFF) && (x < (X_OFF + IMG_W * 2)) && (y < IMG_H * 2);

    wire [11:0] xs = x - X_OFF;
    wire [8:0]  sx = xs[9:1];
    wire [8:0]  sy = y[8:1];

    wire [16:0] raddr = sy * IMG_W + sx;

    // 骨架位图地址：320x240 位图，每行 40 字节，字节高位=最左像素
    assign ovl_addr = sy * 14'd40 + {5'd0, sx[8:3]};

    // ---- 两级流水对齐 de 与数据 ----
    reg [15:0] q1;
    reg        de1, iv1;
    reg [15:0] q2;
    reg        de2, iv2;
    reg        hs1, vs1, hs2, vs2;
    reg [7:0]  bm1, bm2;        // 骨架位图字节（与 q1/q2 对齐）
    reg [2:0]  sxb1, sxb2;      // 列内位序（与 q1/q2 对齐）

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            q1 <= 16'd0; de1 <= 1'b0; iv1 <= 1'b0;
            q2 <= 16'd0; de2 <= 1'b0; iv2 <= 1'b0;
            hs1 <= 1'b0; vs1 <= 1'b0; hs2 <= 1'b0; vs2 <= 1'b0;
        end else begin
            q1  <= mem[raddr];
            de1 <= de;
            iv1 <= in_view;
            hs1 <= hs; vs1 <= vs;
            bm1 <= ovl_dat;
            sxb1<= sx[2:0];

            q2  <= q1;
            de2 <= de1;
            iv2 <= iv1;
            hs2 <= hs1; vs2 <= vs1;
            bm2 <= bm1;
            sxb2<= sxb1;
        end
    end

    assign tft_clk = pix_clk;

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            tft_de <= 1'b0; tft_hs <= 1'b0; tft_vs <= 1'b0; tft_rgb <= 16'd0;
        end else begin
            tft_de  <= de2;
            tft_hs  <= hs2;
            tft_vs  <= vs2;
            // 叠加：位=1 处显示绿色 0x07E0（面板 B-G-R 位序下 G 仍在中间，颜色一致）
            tft_rgb <= iv2 ? ((ovl_en && bm2[3'd7 - sxb2]) ? 16'h07E0 : q2)
                           : 16'h0000;
        end
    end

endmodule