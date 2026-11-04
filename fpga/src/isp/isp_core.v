//=============================================================================
// isp_core.v — 硬件 ISP：3x3 中值去噪 + 帧级自动亮度增益
//   输入：DVP_Capture 的像素流（pclk 域，每 2 拍一个 16bit RGB565 字）
//   输出：处理后的像素流 + 重新生成的 hs/vs/坐标（供打包和显示使用）
//
//   中值：2 行 BSRAM 行缓存，列->中值->中值，精确 3x3
//   增益：统计每帧绿色均值，帧末按阈值步进调整 Q8.8 增益（无除法）
//=============================================================================
`timescale 1ns / 1ps

module isp_core #(
    parameter integer W          = 640,
    parameter integer H          = 480,
    parameter integer GAIN_INIT  = 9'd300,       // 1.17x
    parameter integer GAIN_MIN   = 9'd256,       // 1.0x
    parameter integer GAIN_MAX   = 9'd480,       // 1.875x（9 位最大 511，别写超过 511 的数）
    parameter integer GAIN_STEP  = 9'd8,
    // 目标亮度用"6bit 绿色均值"表示：22 ≈ 90/255，32 ≈ 130/255
    // 阈值按像素数自适应，不能写死绝对值（曾经按 8bit 算导致增益永远加到上限）
    parameter integer TARGET_LOW  = 22,
    parameter integer TARGET_HIGH = 32
)(
    input  wire        pclk,
    input  wire        rst_p,          // 高有效
    input  wire        vs_in,          // DVP 场同步（DataVs）
    input  wire        de_in,          // 像素有效（每 2 拍一次）
    input  wire [15:0] pix_in,

    output reg  [15:0] pix_out,
    output reg         de_out,
    output reg         hs_out,
    output reg         vs_out,
    output reg  [11:0] x_out,
    output reg  [11:0] y_out,
    output wire [8:0]  gain_o
);

    //------------------------------------------------------------------
    // 行缓存（两块独立 BSRAM，各 1 读 1 写）
    //------------------------------------------------------------------
    (* syn_ramstyle = "block_ram" *) reg [15:0] mem1 [0:W-1];
    (* syn_ramstyle = "block_ram" *) reg [15:0] mem2 [0:W-1];

    reg [11:0] xin;
    always @(posedge pclk) begin
        if (rst_p)          xin <= 12'd0;
        else if (de_in)     xin <= (xin == W-1) ? 12'd0 : (xin + 12'd1);
    end

    //---------------- 级 A：读行缓存 + 写第 1 行缓存 ----------------
    reg [15:0] a1, a2, pixA;
    reg [11:0] xA;
    reg        ce1;

    always @(posedge pclk) ce1 <= de_in;

    always @(posedge pclk) begin
        if (de_in) begin
            a1      <= mem1[xin];
            a2      <= mem2[xin];
            mem1[xin] <= pix_in;
            pixA    <= pix_in;
            xA      <= xin;
        end
    end

    //---------------- 级 B：写第 2 行缓存 + 3 列移位 ----------------
    reg [15:0] sA0,sA1,sA2, sB0,sB1,sB2, sC0,sC1,sC2;
    reg        ce2, ce3;

    always @(posedge pclk) begin ce2 <= ce1; ce3 <= ce2; end

    always @(posedge pclk) begin
        if (ce1) begin
            mem2[xA] <= a1;
            sA0 <= a1; sA1 <= sA0; sA2 <= sA1;
            sB0 <= a2; sB1 <= sB0; sB2 <= sB1;
            sC0 <= pixA; sC1 <= sC0; sC2 <= sC1;
        end
    end

    //---------------- 级 C：中值 + 增益 ----------------
    wire [4:0] r_med, b_med;
    wire [5:0] g_med;          // 绿色是 6 位，别和 R/B 混在一起声明

    med9 #(.W(5)) u_med_r(
        .p00(sB2[15:11]), .p01(sB1[15:11]), .p02(sB0[15:11]),
        .p10(sA2[15:11]), .p11(sA1[15:11]), .p12(sA0[15:11]),
        .p20(sC2[15:11]), .p21(sC1[15:11]), .p22(sC0[15:11]),
        .med(r_med)
    );
    med9 #(.W(6)) u_med_g(
        .p00(sB2[10:5]), .p01(sB1[10:5]), .p02(sB0[10:5]),
        .p10(sA2[10:5]), .p11(sA1[10:5]), .p12(sA0[10:5]),
        .p20(sC2[10:5]), .p21(sC1[10:5]), .p22(sC0[10:5]),
        .med(g_med)
    );
    med9 #(.W(5)) u_med_b(
        .p00(sB2[4:0]), .p01(sB1[4:0]), .p02(sB0[4:0]),
        .p10(sA2[4:0]), .p11(sA1[4:0]), .p12(sA0[4:0]),
        .p20(sC2[4:0]), .p21(sC1[4:0]), .p22(sC0[4:0]),
        .med(b_med)
    );

    // 边框两行两列拿不到完整邻域，直接透传中心像素
    reg [11:0] vx, vy;
    wire border = (vy < 12'd2) || (vx < 12'd2);

    // 中心像素（前一行、前一列）
    wire [15:0] pix_ctr = sA1;

    reg [4:0] r_s, b_s;
    reg [5:0] g_s;
    always @(posedge pclk) begin
        if (ce2) begin
            r_s <= border ? pix_ctr[15:11] : r_med;
            g_s <= border ? pix_ctr[10:5]  : g_med;
            b_s <= border ? pix_ctr[4:0]   : b_med;
        end
    end

    //---------------- 自动增益 ----------------
    reg [8:0]  gain;
    reg [25:0] sum_g;

    wire [13:0] r_mul = r_s * gain;
    wire [14:0] g_mul = g_s * gain;
    wire [13:0] b_mul = b_s * gain;

    wire [4:0] r_g = (r_mul[13:8] > 5'd31) ? 5'd31 : r_mul[13:8];
    wire [5:0] g_g = (g_mul[14:8] > 6'd63) ? 6'd63 : g_mul[14:8];
    wire [4:0] b_g = (b_mul[13:8] > 5'd31) ? 5'd31 : b_mul[13:8];

    // DataVs 在硬件里是电平（消隐期一直是高），必须用上升沿，
    // 否则消隐期每个时钟都会步进一次增益，几拍就冲到上限（画面过曝成品红）
    reg vs_d;
    always @(posedge pclk) vs_d <= vs_in;
    wire vs_rise = vs_in & ~vs_d;

    // 帧像素计数：只有收够大半个帧的 vs 边沿才认作"帧结束"，
    // 防止 vs 毛刺导致一帧内多次调整增益（会把增益推到上限）
    reg [19:0] pix_cnt;
    wire frame_ok = (pix_cnt > (W * H * 3 / 4));

    always @(posedge pclk) begin
        if (rst_p)               pix_cnt <= 20'd0;
        else if (vs_rise)        pix_cnt <= 20'd0;
        else if (ce2 && !vs_in)  pix_cnt <= pix_cnt + 20'd1;
    end

    always @(posedge pclk) begin
        if (rst_p) begin
            gain  <= GAIN_INIT;
            sum_g <= 26'd0;
        end else if (vs_rise) begin
            // 帧末按阈值步进，避免除法（frame_ok 为假则本帧不计，保持原增益）
            if (!frame_ok) begin
                sum_g <= 26'd0;
            end else if (sum_g < (TARGET_LOW  * W * H)) begin
                gain <= (gain > GAIN_MAX - GAIN_STEP) ? GAIN_MAX : gain + GAIN_STEP;
            end else if (sum_g > (TARGET_HIGH * W * H)) begin
                gain <= (gain < GAIN_MIN + GAIN_STEP) ? GAIN_MIN : gain - GAIN_STEP;
            end
            sum_g <= 26'd0;
        end else if (ce2 && !vs_in) begin
            sum_g <= sum_g + {20'd0, g_s};   // 26 位
        end
    end

    assign gain_o = gain;

    //---------------- 输出流水 ----------------
    always @(posedge pclk) begin
        if (rst_p) begin
            pix_out <= 16'd0; de_out <= 1'b0; hs_out <= 1'b0; vs_out <= 1'b0;
            vx <= 12'd0; vy <= 12'd0; x_out <= 12'd0; y_out <= 12'd0;
        end else if (ce3) begin
            pix_out <= {r_g, g_g, b_g};
            de_out  <= 1'b1;
            x_out   <= vx;
            y_out   <= vy;
            hs_out  <= (vx == 12'd0);
            vs_out  <= (vx == 12'd0) && (vy == 12'd0);

            if (vx == W-1) begin
                vx <= 12'd0;
                vy <= (vy == H-1) ? 12'd0 : (vy + 12'd1);
            end else begin
                vx <= vx + 12'd1;
            end
        end else begin
            de_out <= 1'b0;
            hs_out <= 1'b0;
            vs_out <= 1'b0;
        end
    end

endmodule
