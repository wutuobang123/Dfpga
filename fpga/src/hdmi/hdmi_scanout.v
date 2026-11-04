//=============================================================================
// hdmi_scanout.v — HDMI 原生 1280x720@60 扫描输出（取代 TFT 预览）
//   写侧：摄像头像素时钟域，2x2 抽样后写入 320x240 BSRAM
//   读侧：74.25MHz，3 倍放大居中（960x720，左右各 160 像素黑边）
//   叠加：读骨架位图，位=1 处画绿色
//=============================================================================
module hdmi_scanout #(
    parameter integer H_ACTIVE = 1280,
    parameter integer H_FP     = 110,
    parameter integer H_SW     = 40,
    parameter integer H_BP     = 220,
    parameter integer V_ACTIVE = 720,
    parameter integer V_FP     = 5,
    parameter integer V_SW     = 5,
    parameter integer V_BP     = 20,
    parameter integer IMG_W    = 320,
    parameter integer IMG_H    = 240,
    parameter integer SCALE    = 3
)(
    input  wire        pix_clk,      // 74.25MHz
    input  wire        rst_n,

    // 写口（摄像头 pclk 域）
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [16:0] wr_addr,
    input  wire [15:0] wr_data,

    // 骨架位图读口
    input  wire        ovl_en,
    output wire [13:0] ovl_addr,
    input  wire [7:0]  ovl_dat,
    output wire        disp_tick,

    // 输出（给 dvi_encoder）
    output wire        hs,
    output wire        vs,
    output wire        de,
    output wire [7:0]  r8,
    output wire [7:0]  g8,
    output wire [7:0]  b8
);
    localparam integer IMG_PIX = IMG_W * IMG_H;
    localparam integer X_OFF   = (H_ACTIVE - IMG_W * SCALE) / 2;   // 160
    localparam integer OS      = (SCALE == 3) ? 2 : 1;             // 地址移位

    (* syn_ramstyle = "block_ram" *) reg [15:0] mem [0:IMG_PIX-1];

    // 面板是 B-G-R 位序，写进来时换成标准 R-G-B 顺序存放
    wire [15:0] wr_rgb = {wr_data[4:0], wr_data[10:5], wr_data[15:11]};

    always @(posedge wr_clk) begin
        if (wr_en) mem[wr_addr] <= wr_rgb;
    end

    // ---- 720p 时序 ----
    wire [11:0] x, y;
    video_timing #(
        .H_ACTIVE(H_ACTIVE), .H_FP(H_FP), .H_SW(H_SW), .H_BP(H_BP),
        .V_ACTIVE(V_ACTIVE), .V_FP(V_FP), .V_SW(V_SW), .V_BP(V_BP),
        .HS_POL(1'b1), .VS_POL(1'b1)
    ) u_timing (
        .pix_clk(pix_clk), .rst_n(rst_n),
        .hs(hs), .vs(vs), .de(de),
        .x(x), .y(y), .frame_tick(disp_tick)
    );

    // ---- 显示坐标 -> 源坐标（整数倍放大，居中）----
    wire in_view = (x >= X_OFF) && (x < (X_OFF + IMG_W * SCALE)) && (y < IMG_H * SCALE);
    wire [11:0] xs = x - X_OFF;
    wire [8:0]  sx = xs[11:OS];        // /2 或 /4... 这里 SCALE=3 时用查表
    wire [8:0]  sy = y[11:OS];

    // SCALE=3 时不能用移位，改用乘除常量（综合成常量乘法）
    wire [11:0] sxq = xs / SCALE;
    wire [11:0] syq = y  / SCALE;
    wire [8:0]  sx_f = (SCALE == 3) ? sxq[8:0] : sx;
    wire [8:0]  sy_f = (SCALE == 3) ? syq[8:0] : sy;

    wire [16:0] raddr    = sy_f * IMG_W + sx_f;
    wire [13:0] bmp_addr = sy_f * 14'd40 + {5'd0, sx_f[8:3]};

    assign ovl_addr = bmp_addr;

    // ---- 两级流水对齐 ----
    reg [15:0] q1, q2;
    reg        de1, de2, iv1, iv2;
    reg [7:0]  bm1, bm2;
    reg [2:0]  sxb1, sxb2;

    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) begin
            q1 <= 16'd0; de1 <= 1'b0; iv1 <= 1'b0; bm1 <= 8'd0; sxb1 <= 3'd0;
            q2 <= 16'd0; de2 <= 1'b0; iv2 <= 1'b0; bm2 <= 8'd0; sxb2 <= 3'd0;
        end else begin
            q1  <= mem[raddr];
            de1 <= de;
            iv1 <= in_view;
            bm1 <= ovl_dat;
            sxb1<= sx_f[2:0];

            q2  <= q1;
            de2 <= de1;
            iv2 <= iv1;
            bm2 <= bm1;
            sxb2<= sxb1;
        end
    end

    // ---- 输出：位图位=1 -> 绿；否则原视频 ----
    reg [15:0] px;
    always @(posedge pix_clk or negedge rst_n) begin
        if (!rst_n) px <= 16'd0;
        else if (!iv2)                      px <= 16'd0;
        else if (ovl_en && bm2[3'd7 - sxb2]) px <= 16'h07E0;   // 绿
        else                                 px <= q2;
    end

    assign r8 = {px[15:11], px[15:13]};
    assign g8 = {px[10:5],  px[10:9]};
    assign b8 = {px[4:0],   px[4:2]};

endmodule
