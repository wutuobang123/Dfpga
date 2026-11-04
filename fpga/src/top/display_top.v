//=============================================================================
// acg720_top.v  鈥? FPGA 寮傛瀯灏戝効浣撴€佽瘎浼扮郴缁?/ FPGA 绔《灞傦紙W1-W2 楠ㄦ灦锛?//   鐩爣鏉?: 灏忔鍝?ACG720 (GW5AT-LV60PG484)
//   闃舵   : 鍙屽睆閫氳矾楠岃瘉 鈥斺€?HDMI1 720p60 娴嬭瘯鍥?+ TFT 800x480 娴嬭瘯鍥?//   鍚庣画   : W3 鎺?OV5640 DVP锛學4 鎺?DDR3 甯х紦瀛橈紝W6 鎺ュ崈鍏嗙綉锛學8 鎺?OSD
//=============================================================================
`timescale 1ns / 1ps

module display_top(
    input  wire        clk50M,        // 鏉胯浇 50MHz 鏅舵尟   (Y18)
    input  wire        reset_n,       // 鎸夐敭 S0 澶嶄綅锛屼綆鏈夋晥 (F15)

    output wire [1:0]  led,           // 鎸囩ず鐏紙60K 灏佽 M22/N22 鏄?SSPI 涓撶敤鑴氾紝涓嶅彲鐢級

    // ---- HDMI1 (鐪熷疄 TMDS 宸垎瀵癸紝闈炵紪鐮佽姱鐗? ----
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_data_p,
    output wire [2:0]  tmds_data_n,

    // ---- TFT 5 瀵?800x480 RGB565 骞跺彛 ----
    output wire        tft_clk,
    output wire        tft_de,
    output wire        tft_hs,
    output wire        tft_vs,
    output wire        tft_pwm,       // 鑳屽厜 PWM锛屾湰鐗堝父浜?    output wire [15:0] tft_rgb        // [15:11]=B [10:5]=G [4:0]=R  鈽呮敞鎰忛珮浣嶆槸 B
);

    //------------------------------------------------------------------
    // 鏃堕挓锛?0M -> 371.25M /5 -> 74.25M(HDMI 鍍忕礌鏃堕挓) /2 -> 37.125M(TFT)
    //------------------------------------------------------------------
    wire clk_371m, clk_24m, clk_50m_o, pll_locked;

    camera_pll u_pll(
        .clkin    (clk50M),
        .clkout0  (clk_371m),        // 5x pixel clock
        .clkout1  (clk_24m),         // 鎽勫儚澶?XCLK锛堟湰闃舵鏈敤锛?        .clkout2  (clk_50m_o),
        .lock     (pll_locked),
        .mdclk    (clk50M),
        .reset    (~reset_n)
    );

    wire clk_74m;
    CLKDIV u_clkdiv_5(
        .RESETN (pll_locked),
        .HCLKIN (clk_371m),
        .CLKOUT (clk_74m),
        .CALIB  (1'b1)
    );
    defparam u_clkdiv_5.DIV_MODE = "5";

    // 74.25M 浜屽垎棰?-> 37.125MHz锛?00x480@60 鍍忕礌鏃堕挓锛?    // 鐢ㄧ浜岀骇 CLKDIV 鑰岄潪 FF 鍒嗛锛屼繚璇佽緭鍑鸿蛋鍏ㄥ眬鏃堕挓缃戠粶
    wire clk_37m;
    CLKDIV u_clkdiv_2(
        .RESETN (pll_locked),
        .HCLKIN (clk_74m),
        .CLKOUT (clk_37m),
        .CALIB  (1'b1)
    );
    defparam u_clkdiv_2.DIV_MODE = "2";

    wire rst_n = pll_locked & reset_n;

    //------------------------------------------------------------------
    // HDMI 閫氳矾锛?280x720@60  (1650 x 750, 姝ｆ瀬鎬у悓姝?
    //------------------------------------------------------------------
    wire        hd_clk = clk_74m;
    wire [11:0] hd_x, hd_y;
    wire        hd_hs, hd_vs, hd_de, hd_frame;
    wire [23:0] hd_rgb;

    video_timing #(
        .H_ACTIVE(1280), .H_FP(110), .H_SW(40), .H_BP(220),
        .V_ACTIVE( 720), .V_FP(  5), .V_SW( 5), .V_BP( 20),
        .HS_POL(1'b1), .VS_POL(1'b1)
    ) u_hd_timing (
        .pix_clk(hd_clk), .rst_n(rst_n),
        .hs(hd_hs), .vs(hd_vs), .de(hd_de),
        .x(hd_x), .y(hd_y), .frame_tick(hd_frame)
    );

    test_pattern #(
        .H_ACTIVE(1280), .V_ACTIVE(720), .BAR_W(160)
    ) u_hd_pattern (
        .pix_clk(hd_clk), .rst_n(rst_n), .frame_tick(hd_frame),
        .x(hd_x), .y(hd_y), .de(hd_de), .rgb(hd_rgb)
    );

    dvi_encoder u_dvi_encoder(
        .pixelclk (hd_clk),
        .pixelclk5x(clk_371m),
        .rst_n    (rst_n),
        .blue_din (hd_rgb[7:0]),
        .green_din(hd_rgb[15:8]),
        .red_din  (hd_rgb[23:16]),
        .hsync    (hd_hs),
        .vsync    (hd_vs),
        .de       (hd_de),
        .tmds_clk_p (tmds_clk_p),
        .tmds_clk_n (tmds_clk_n),
        .tmds_data_p(tmds_data_p),
        .tmds_data_n(tmds_data_n)
    );

    //------------------------------------------------------------------
    // TFT 閫氳矾锛?00x480@60 (1056 x 586, 璐熸瀬鎬у悓姝?
    //------------------------------------------------------------------
    tft_ctrl #(
        .H_ACTIVE(800), .H_FP(40), .H_SW(48), .H_BP(168),
        .V_ACTIVE(480), .V_FP(13), .V_SW( 3), .V_BP( 90)
    ) u_tft (
        .clk       (clk_37m),
        .rst_n     (rst_n),
        .tft_clk   (tft_clk),
        .tft_de    (tft_de),
        .tft_hs    (tft_hs),
        .tft_vs    (tft_vs),
        .tft_rgb   (tft_rgb)
    );

    assign tft_pwm = 1'b1;   // 鑳屽厜甯镐寒锛屽悗缁彲鎺?PWM 璋冨厜

    //------------------------------------------------------------------
    // 鐘舵€佹寚绀?    //------------------------------------------------------------------
    reg [23:0] beat_cnt;
    always @(posedge clk_50m_o) begin
        if (!rst_n) beat_cnt <= 24'd0;
        else        beat_cnt <= beat_cnt + 24'd1;
    end

    assign led[0] = pll_locked & rst_n;   // PLL 閿佸畾涓斿浣嶉噴鏀?    assign led[1] = hd_frame | beat_cnt[23];  // HDMI 甯ц剦鍐叉贩蹇冭烦锛屽父浜?甯哥伃閮借鏄庢病璺戣捣鏉?
endmodule

