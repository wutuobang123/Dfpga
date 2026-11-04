//=============================================================================
// acg720_top.v — FPGA 异构少儿体态评估系统 / FPGA 端主通路
//   范围（三周版）：OV5640 采集 -> 打包(帧号+行号) -> 千兆网 UDP 输出
//   不做显示、不做帧缓存（行流式，端到端延迟 = 1 行）
//   底板：小梅哥 ACG720 (GW5AT-LV60PG484)
//   参考：官方 ch59 (ov5640_udp_rgmii)，保留其成熟链路，改为 640x480 + 帧号
//=============================================================================
`timescale 1ns / 1ps

module acg720_top #(
    // ---------------- 图像 ----------------
    parameter IMAGE_WIDTH  = 640,
    parameter IMAGE_HEIGHT = 480,
    // ---------------- 网络（与 RK3568 约定的静态地址） ----------------
    parameter DST_MAC   = 48'hFF_FF_FF_FF_FF_FF,   // 广播，联调期先用广播
    parameter SRC_MAC   = 48'h00_0A_35_01_FE_C0,
    parameter DST_IP    = 32'hFF_FF_FF_FF,         // 受限广播：联调期 PC 不用配 IP 也能收
                                                   // 正式接 RK 时改回 32'hC0_A8_01_14
    parameter SRC_IP    = 32'hC0_A8_01_0A,         // 192.168.1.10 (FPGA)
    parameter DST_PORT  = 16'd8080,
    parameter SRC_PORT  = 16'd8080
)(
    input  wire        clk50m,
    input  wire        reset_n,

    // ---- OV5640 DVP ----
    output wire        camera_sclk,
    inout  wire        camera_sdat,
    input  wire        camera_vsync,
    input  wire        camera_href,
    input  wire        camera_pclk,
    output wire        camera_xclk,
    input  wire [7:0]  camera_data,
    output wire        camera_pwdn,
    output wire        camera_rst_n,

    // ---- RGMII 千兆网 ----
    input  wire        rgmii_rxc,      // PHY 送来的 125MHz 接收时钟
    input  wire [3:0]  rgmii_rxd,
    input  wire        rgmii_rxdv,
    output wire        rgmii_gtxc,
    output wire [3:0]  rgmii_txd,
    output wire        rgmii_txen,
    output wire        eth_mdc,
    inout  wire        eth_mdio,
    output wire        eth_rst_n,

    // ---- TFT 5 寸屏 800x480（本地预览）----
    output wire        tft_clk,
    output wire        tft_de,
    output wire        tft_hs,
    output wire        tft_vs,
    output wire        tft_pwm,
    output wire [15:0] tft_rgb,

    // ---- 用户按键（S1/S2 并联，按任一键=重新开始计分）----
    input  wire        key_restart,
    input  wire        key_restart2,

    // ---- 调试串口 ----
    output wire        fpga_uart_tx,

    // ---- HDMI TMDS 输出（两路，板上有两个 HDMI 口）----
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_data_p,
    output wire [2:0]  tmds_data_n,
    output wire        tmds2_clk_p,
    output wire        tmds2_clk_n,
    output wire [2:0]  tmds2_data_p,
    output wire [2:0]  tmds2_data_n,
    output wire        hdmi1_oe,
    output wire        hdmi2_oe,

    // ---- 状态灯 ----
    output wire [1:0]  led
);

    //------------------------------------------------------------------
    // 时钟
    //------------------------------------------------------------------
    wire pll_locked;
    wire loc_clk50m;
    wire loc_clk125m;
    wire loc_clk24m;

    Gowin_PLL u_pll(
        .clkin  (clk50m),
        .clkout0(loc_clk50m),
        .clkout1(loc_clk125m),
        .clkout2(loc_clk24m),
        .lock   (pll_locked),
        .mdclk  (clk50m),
        .reset  (~reset_n)
    );

    wire g_rst_p = ~pll_locked;          // 高有效内部复位

    //------------------------------------------------------------------
    // 显示时钟：50M -> 371.25M -> /5 74.25M -> /2 37.125M（800x480@60）
    //------------------------------------------------------------------
    wire clk_371m, clk_74m, clk_37m, disp_locked;

    camera_pll u_disp_pll(
        .clkin  (clk50m),
        .clkout0(clk_371m),
        .clkout1(),
        .clkout2(),
        .lock   (disp_locked),
        .mdclk  (clk50m),
        .reset  (~reset_n)
    );

    CLKDIV u_div5(
        .RESETN(disp_locked), .HCLKIN(clk_371m), .CLKOUT(clk_74m), .CALIB(1'b1)
    );
    defparam u_div5.DIV_MODE = "5";

    CLKDIV u_div2(
        .RESETN(disp_locked), .HCLKIN(clk_74m), .CLKOUT(clk_37m), .CALIB(1'b1)
    );
    defparam u_div2.DIV_MODE = "2";

    //------------------------------------------------------------------
    // PHY 配置（MDIO 写 RTL8211E 寄存器）
    //------------------------------------------------------------------
    wire phy_init;

    phy_reg_config u_phy_cfg(
        .Clk          (loc_clk50m),
        .Rst_n        (~g_rst_p),
        .Phy_rst_n    (eth_rst_n),
        .mdc          (eth_mdc),
        .mdio         (eth_mdio),
        .Phy_init_done(phy_init)
    );

    //------------------------------------------------------------------
    // 摄像头 SCCB 初始化
    //------------------------------------------------------------------
    wire camera_init_done;

    assign camera_xclk = loc_clk24m;

    camera_init #(
        .CAMERA_TYPE    ("ov5640"    ),
        .IMAGE_TYPE     (0           ),   // 0: RGB
        .IMAGE_WIDTH    (IMAGE_WIDTH ),
        .IMAGE_HEIGHT   (IMAGE_HEIGHT),
        .IMAGE_FLIP_EN  (0           ),
        .IMAGE_MIRROR_EN(0           )
    ) u_camera_init (
        .Clk         (loc_clk50m),
        .Rst_n       (~g_rst_p),
        .Init_Done   (camera_init_done),
        .camera_rst_n(camera_rst_n),
        .camera_pwdn (camera_pwdn),
        .i2c_sclk    (camera_sclk),
        .i2c_sdat    (camera_sdat)
    );

    //------------------------------------------------------------------
    // DVP 采集
    //------------------------------------------------------------------
    wire        pclk_bufg_o = camera_pclk;

    wire        image_data_valid;
    wire [15:0] image_data;
    wire        image_data_hs;
    wire        image_data_vs;
    wire [11:0] image_data_xaddr;
    wire [11:0] image_data_yaddr;

    DVP_Capture u_capture(
        .Rst_p    (g_rst_p         ),
        .PCLK     (pclk_bufg_o     ),
        .Vsync    (camera_vsync    ),
        .Href     (camera_href     ),
        .Data     (camera_data     ),
        .ImageState(               ),
        .DataValid(image_data_valid),
        .DataPixel(image_data      ),
        .DataHs   (image_data_hs   ),
        .DataVs   (image_data_vs   ),
        .Xaddr    (image_data_xaddr),
        .Yaddr    (image_data_yaddr)
    );

    //------------------------------------------------------------------
    // 硬件 ISP：3x3 中值去噪 + 帧级自动增益
    //------------------------------------------------------------------
    wire [15:0] isp_pix;
    wire        isp_de, isp_hs, isp_vs;
    wire [11:0] isp_x, isp_y;
    wire [8:0]  isp_gain;

    isp_core #(
        .W(IMAGE_WIDTH), .H(IMAGE_HEIGHT),
        .GAIN_INIT(9'd300), .GAIN_MIN(9'd256), .GAIN_MAX(9'd480)
    ) u_isp (
        .pclk   (pclk_bufg_o     ),
        .rst_p  (g_rst_p         ),
        .vs_in  (image_data_vs   ),
        .de_in  (image_data_valid),
        .pix_in (image_data      ),
        .pix_out(isp_pix         ),
        .de_out (isp_de          ),
        .hs_out (isp_hs          ),
        .vs_out (isp_vs          ),
        .x_out  (isp_x           ),
        .y_out  (isp_y           ),
        .gain_o (isp_gain        )
    );

    //------------------------------------------------------------------
    //------------------------------------------------------------------
    // 重启按键：S1(A20) 去抖 20ms，按下后保持 100ms
    //   期间每帧首行用 row=0x7FF 作控制包（正常行号 1~480，不影响图像）
    //------------------------------------------------------------------
    // 两键并联：任一按下即为按下（低有效），两者都松开才算松开
    wire       key_raw = key_restart & key_restart2;

    reg [2:0]  key_sync;
    reg [19:0] key_cnt;
    reg        key_stable;
    reg [24:0] hold_cnt;
    reg        key_req;
    reg [7:0]  key_hits;          // 按键次数（调试用，饱和）

    always @(posedge clk50m or posedge g_rst_p) begin
        if (g_rst_p) begin
            key_sync   <= 3'b111;
            key_cnt    <= 20'd0;
            key_stable <= 1'b1;
            hold_cnt   <= 25'd0;
            key_req    <= 1'b0;
            key_hits   <= 8'd0;
        end else begin
            key_sync <= {key_sync[1:0], key_raw};

            // 任意边沿（按下或松开）都触发，规避极性不确定
            if (key_sync[2] != key_stable) begin
                if (key_cnt == 20'd1_000_000) begin        // 20ms 去抖
                    key_cnt    <= 20'd0;
                    key_stable <= key_sync[2];
                    key_req    <= 1'b1;
                    hold_cnt   <= 25'd15_000_000;          // 保持 300ms
                    if (key_hits != 8'hFF) key_hits <= key_hits + 8'd1;
                end else begin
                    key_cnt <= key_cnt + 20'd1;
                end
            end else begin
                key_cnt <= 20'd0;
            end

            if (key_req) begin
                if (hold_cnt == 25'd0) key_req <= 1'b0;
                else                   hold_cnt <= hold_cnt - 25'd1;
            end
        end
    end

    // 控制包：按键期间整帧所有行都用 row=0x7FF（图像冻结 0.3 秒，换取可靠识别）
    wire [11:0] row_hdr = key_req ? 12'h7FF : (isp_y + 12'd1);

    // 打包：每条行首插入 1 个 16bit 头字 = {frame_id[3:0], row[11:0]}
    //   行数据 = IMAGE_WIDTH 个 RGB565 像素
    //   RK 端靠 frame_id 判帧、row 判行，丢一行不影响整帧
    //------------------------------------------------------------------
    wire [15:0] pixel_data;
    wire        pixel_data_valid;

    dvp_packer u_packer(
        .pclk      (pclk_bufg_o     ),
        .rst_p     (g_rst_p         ),
        .vsync_in  (isp_vs          ),
        .hs_in     (isp_hs          ),
        .valid_in  (isp_de          ),
        .data_in   (isp_pix         ),
        .row_in    (row_hdr        ),
        .word_out  (pixel_data      ),
        .word_valid(pixel_data_valid)
    );

    //------------------------------------------------------------------
    // 以太网发送：每行一个 UDP 包
    //------------------------------------------------------------------
    wire       tx_en_pulse;
    wire       tx_done;
    wire       fifo_rd;
    wire [7:0] fifo_dout;

    eth_tx_ctrl #(
        .PAYLOAD_DATA_BYTE(2                ),
        .PAYLOAD_LENGTH   (IMAGE_WIDTH + 1  )   // +1 = 行首头字
    ) u_eth_tx_ctrl (
        .reset_p          (g_rst_p       ),
        .clk              (pclk_bufg_o   ),
        .data_i           ({pixel_data[7:0], pixel_data[15:8]}),
        .data_valid_i     (pixel_data_valid),
        .eth_txfifo_rd_clk(loc_clk125m   ),
        .tx_en_pulse      (tx_en_pulse   ),
        .tx_done          (tx_done       ),
        .eth_txfifo_rden  (fifo_rd       ),
        .eth_txfifo_dout  (fifo_dout     )
    );

    wire [7:0] gmii_txd;
    wire       gmii_tx_clk;
    wire       gmii_txen;

    eth_udp_tx_gmii u_eth_tx(
        .clk125m     (loc_clk125m),
        .reset_p     (g_rst_p    ),
        .tx_en_pulse (tx_en_pulse),
        .tx_done     (tx_done    ),
        .dst_mac     (DST_MAC    ),
        .src_mac     (SRC_MAC    ),
        .dst_ip      (DST_IP     ),
        .src_ip      (SRC_IP     ),
        .dst_port    (DST_PORT   ),
        .src_port    (SRC_PORT   ),
        .data_length (IMAGE_WIDTH * 2 + 2),
        .payload_req_o(fifo_rd   ),
        .payload_dat_i(fifo_dout ),
        .gmii_tx_clk (gmii_tx_clk),
        .gmii_txen   (gmii_txen  ),
        .gmii_txd    (gmii_txd   )
    );

    gmii_to_rgmii u_gmii2rgmii(
        .reset_n     (reset_n    ),
        .gmii_tx_clk (gmii_tx_clk),
        .gmii_txd    (gmii_txd   ),
        .gmii_txen   (gmii_txen  ),
        .gmii_txer   (1'b0       ),
        .rgmii_tx_clk(rgmii_gtxc ),
        .rgmii_txd   (rgmii_txd  ),
        .rgmii_txen  (rgmii_txen )
    );

    //------------------------------------------------------------------
    // 以太网接收：RGMII->GMII -> UDP -> 8081 载荷字节流
    //------------------------------------------------------------------
    wire        gmii_rx_clk;
    wire [7:0]  gmii_rxd;
    wire        gmii_rxdv, gmii_rxer;

    rgmii_to_gmii u_rgmii2gmii(
        .reset       (g_rst_p    ),
        .rgmii_rx_clk(rgmii_rxc  ),
        .rgmii_rxd   (rgmii_rxd  ),
        .rgmii_rxdv  (rgmii_rxdv ),
        .gmii_rx_clk (gmii_rx_clk),
        .gmii_rxd    (gmii_rxd   ),
        .gmii_rxdv   (gmii_rxdv  ),
        .gmii_rxer   (gmii_rxer  )
    );

    wire [7:0]  udp_dat;
    wire        udp_vld;
    wire        udp_done;
    wire        udp_err;
    wire [15:0] udp_len;
    wire [31:0] udp_crc_dbg;

    //------------------------------------------------------------------
    // RX 卡死看门狗：包在到达但长时间没有"整包完成" -> 复位 UDP 层和 SK01 解析器
    //   背景：解析器靠 s_done 复位 byte_cnt，一旦某包结束信号丢失会永久错位
    //------------------------------------------------------------------
    reg        rxdv_d;
    reg [15:0] rxdv_since_done;
    reg [19:0] retry_timer;
    reg        rx_stuck_pulse;

    always @(posedge gmii_rx_clk or posedge g_rst_p) begin
        if (g_rst_p) begin
            rxdv_d          <= 1'b0;
            rxdv_since_done <= 16'd0;
            retry_timer     <= 20'd0;
            rx_stuck_pulse  <= 1'b0;
        end else begin
            rxdv_d         <= gmii_rxdv;
            rx_stuck_pulse <= 1'b0;
            if (udp_done || rx_stuck_pulse)
                rxdv_since_done <= 16'd0;
            else if (gmii_rxdv && !rxdv_d && rxdv_since_done != 16'hFFFF)
                rxdv_since_done <= rxdv_since_done + 16'd1;

            if (rxdv_since_done > 16'd200) begin
                if (retry_timer == 20'd0) begin
                    rx_stuck_pulse <= 1'b1;
                    retry_timer    <= 20'd1_000_000;   // 8ms 后允许再次复位
                end else begin
                    retry_timer <= retry_timer - 20'd1;
                end
            end else begin
                retry_timer <= 20'd0;
            end
        end
    end

    wire rx_rst = g_rst_p | rx_stuck_pulse;

    eth_udp_rx_gmii u_udp_rx(
        .reset_p        (rx_rst    ),
        .local_mac      (SRC_MAC   ),
        .local_ip       (SRC_IP    ),
        .local_port     (16'd8081  ),
        .clk125m_o      (          ),
        .exter_mac      (          ),
        .exter_ip       (          ),
        .exter_port     (          ),
        .rx_data_length (udp_len   ),
        .data_overflow_i(1'b0      ),
        .payload_valid_o(udp_vld   ),
        .payload_dat_o  (udp_dat   ),
        .one_pkt_done   (udp_done  ),
        .pkt_error      (udp_err   ),
        .debug_crc_check(udp_crc_dbg),
        .gmii_rx_clk    (gmii_rx_clk),
        .gmii_rxdv      (gmii_rxdv ),
        .gmii_rxd       (gmii_rxd  )
    );

    //------------------------------------------------------------------
    // SK01 v2 解析 + 双缓冲位图（骨架 OSD）
    //------------------------------------------------------------------
    wire        ovl_en;
    wire        disp_tick;
    wire [13:0] ovl_addr;
    wire [7:0]  ovl_dat;
    wire [15:0] ovl_frames, ovl_err, ovl_nz;
    wire        ovl_act, ovl_wr, ovl_flip;

    //------------------------------------------------------------------
    // 显示方案：1 = 取消 TFT，直接用 HDMI 原生 1280x720@60
    //           0 = TFT 本地预览，HDMI 镜像 TFT 时序
    //------------------------------------------------------------------
    localparam HDMI_ONLY = 0;   // 0 = TFT 本地预览（不再用 FPGA 的 HDMI）
    wire scan_clk = HDMI_ONLY ? clk_74m : clk_37m;

    sk01_rx u_sk01 (
        .rst_p    (rx_rst     ),
        .rx_clk   (gmii_rx_clk),
        .s_vld    (udp_vld    ),
        .s_dat    (udp_dat    ),
        .s_done   (udp_done   ),
        .rd_clk   (scan_clk   ),
        .disp_tick(disp_tick  ),
        .rd_addr  (ovl_addr   ),
        .rd_dat   (ovl_dat    ),
        .ovl_en   (ovl_en     ),
        .ok_frames(ovl_frames ),
        .err_pkts (ovl_err    ),
        .dbg_active_sel(ovl_act),
        .dbg_wr_sel    (ovl_wr ),
        .dbg_flip_pending(ovl_flip),
        .dbg_nz_bytes  (ovl_nz )
    );

    // 临时探针：统计收到的字节/包数，验证 RX 通路（后续换成 SK01 解析）
    reg [31:0] rx_bytes;
    reg [15:0] rx_pkts;
    always @(posedge gmii_rx_clk or posedge g_rst_p) begin
        if (g_rst_p) begin
            rx_bytes <= 32'd0;
            rx_pkts  <= 16'd0;
        end else begin
            if (udp_vld)  rx_bytes <= rx_bytes + 32'd1;
            if (udp_done) rx_pkts  <= rx_pkts  + 16'd1;
        end
    end

    //------------------------------------------------------------------
    // 显示输出（二选一，见 HDMI_ONLY）
    //------------------------------------------------------------------
    wire        fb_wr   = isp_de && (isp_y[0] == 1'b0) && (isp_x[0] == 1'b0);
    wire [16:0] fb_base = {6'd0, isp_y[11:1]} * 17'd320;
    wire [16:0] fb_addr = fb_base + {5'd0, isp_x[11:1]};

    wire        tft_clk_i, tft_de_i, tft_hs_i, tft_vs_i;
    wire [15:0] tft_rgb_i;

    generate
    if (HDMI_ONLY) begin : g_hdmi_native
        // ---- 取消 TFT，HDMI 原生 1280x720@60（3 倍放大居中）----
        wire [7:0] r8, g8, b8;
        wire       hs_o, vs_o, de_o;

        hdmi_scanout #(
            .H_ACTIVE(1280), .H_FP(110), .H_SW(40), .H_BP(220),
            .V_ACTIVE( 720), .V_FP(  5), .V_SW( 5), .V_BP( 20),
            .IMG_W(320), .IMG_H(240), .SCALE(3)
        ) u_scan (
            .pix_clk  (clk_74m  ),
            .rst_n    (reset_n & pll_locked & disp_locked),
            .wr_clk   (pclk_bufg_o),
            .wr_en    (fb_wr    ),
            .wr_addr  (fb_addr  ),
            .wr_data  (isp_pix  ),
            .ovl_en   (ovl_en   ),
            .ovl_addr (ovl_addr ),
            .ovl_dat  (ovl_dat  ),
            .disp_tick(disp_tick),
            .hs(hs_o), .vs(vs_o), .de(de_o),
            .r8(r8), .g8(g8), .b8(b8)
        );

        dvi_encoder u_dvi_encoder(
            .pixelclk   (clk_74m     ),
            .pixelclk5x (clk_371m    ),
            .rst_n      (disp_locked ),
            .blue_din   (b8          ),
            .green_din  (g8          ),
            .red_din    (r8          ),
            .hsync      (hs_o        ),
            .vsync      (vs_o        ),
            .de         (de_o        ),
            .tmds_clk_p (tmds_clk_p  ),
            .tmds_clk_n (tmds_clk_n  ),
            .tmds_data_p(tmds_data_p ),
            .tmds_data_n(tmds_data_n )
        );

        // 第二个 HDMI 口（板上有两个，都驱动上，插哪个都能出图）
        dvi_encoder u_dvi_encoder2(
            .pixelclk   (clk_74m     ),
            .pixelclk5x (clk_371m    ),
            .rst_n      (disp_locked ),
            .blue_din   (b8          ),
            .green_din  (g8          ),
            .red_din    (r8          ),
            .hsync      (hs_o        ),
            .vsync      (vs_o        ),
            .de         (de_o        ),
            .tmds_clk_p (tmds2_clk_p ),
            .tmds_clk_n (tmds2_clk_n ),
            .tmds_data_p(tmds2_data_p),
            .tmds_data_n(tmds2_data_n)
        );

        assign hdmi1_oe = 1'b1;
        assign hdmi2_oe = 1'b1;

        // TFT 面板不点亮
        assign tft_clk = 1'b0;
        assign tft_de  = 1'b0;
        assign tft_hs  = 1'b0;
        assign tft_vs  = 1'b0;
        assign tft_rgb = 16'd0;
        assign tft_pwm = 1'b0;
    end else begin : g_tft_mode
        tft_preview #(
            .H_ACTIVE(800), .H_FP(40), .H_SW(48), .H_BP(168),
            .V_ACTIVE(480), .V_FP(13), .V_SW( 3), .V_BP( 90),
            .IMG_W(320),    .IMG_H(240)
        ) u_tft (
            .ovl_en  (ovl_en  ),
            .disp_tick(disp_tick),
            .ovl_addr(ovl_addr),
            .ovl_dat (ovl_dat ),
            .pix_clk (clk_37m),
            .rst_n   (reset_n & pll_locked & disp_locked),
            .wr_clk  (pclk_bufg_o),
            .wr_en   (fb_wr),
            .wr_addr (fb_addr),
            .wr_data (isp_pix),
            .tft_clk (tft_clk_i),
            .tft_de  (tft_de_i),
            .tft_hs  (tft_hs_i),
            .tft_vs  (tft_vs_i),
            .tft_rgb (tft_rgb_i)
        );

        assign tft_clk = tft_clk_i;
        assign tft_de  = tft_de_i;
        assign tft_hs  = tft_hs_i;
        assign tft_vs  = tft_vs_i;
        assign tft_rgb = tft_rgb_i;
        assign tft_pwm = 1'b1;
        assign tmds2_clk_p  = 1'b0;
        assign tmds2_clk_n  = 1'b0;
        assign tmds2_data_p = 3'b000;
        assign tmds2_data_n = 3'b000;
        assign hdmi1_oe     = 1'b0;
        assign hdmi2_oe     = 1'b0;

        // 不使用 HDMI：全部拉低（TMDS 高速翻转会干扰 RGMII 接收，实测会丢包）
        assign tmds_clk_p   = 1'b0;
        assign tmds_clk_n   = 1'b0;
        assign tmds_data_p  = 3'b000;
        assign tmds_data_n  = 3'b000;
    end
    endgenerate

    //------------------------------------------------------------------
    // 调试串口：周期输出叠加通路状态（115200 8N1）
    //------------------------------------------------------------------
    reg [15:0] disp_cnt;
    always @(posedge scan_clk or posedge g_rst_p) begin
        if (g_rst_p)          disp_cnt <= 16'd0;
        else if (disp_tick)   disp_cnt <= disp_cnt + 16'd1;
    end

    // 统计收到的以太网帧（gmii_rxdv 上升沿），判断 RX 通路是否工作
    reg [15:0] eth_frm_cnt;
    always @(posedge gmii_rx_clk or posedge g_rst_p) begin
        if (g_rst_p) begin
            eth_frm_cnt <= 16'd0;
        end else begin
            if (gmii_rxdv && !rxdv_d) eth_frm_cnt <= eth_frm_cnt + 16'd1;
        end
    end

    wire [7:0] dbg_status = {4'd0, ovl_flip, ovl_wr, ovl_act, ovl_en};
    wire [7:0] uart_data;
    wire       uart_start, uart_busy;

    dbg_stream u_dbg (
        .clk      (clk50m      ),
        .rst      (g_rst_p     ),
        .ok_frames(ovl_frames  ),
        .err_pkts (ovl_err     ),
        .status   (dbg_status  ),
        .disp_cnt (disp_cnt    ),
        .key_hits (key_hits    ),
        .eth_frames(eth_frm_cnt),
        .udp_pkts (rx_pkts    ),
        .tx_data  (uart_data   ),
        .tx_start (uart_start  ),
        .tx_busy  (uart_busy   )
    );

    uart_tx #(.CLK_HZ(50_000_000), .BAUD(115200)) u_uart (
        .clk  (clk50m     ),
        .rst  (g_rst_p    ),
        .data (uart_data  ),
        .start(uart_start ),
        .tx   (fpga_uart_tx),
        .busy (uart_busy  )
    );

    //------------------------------------------------------------------
    // 状态灯：led0=PLL 锁定, led1=摄像头初始化完成
    //------------------------------------------------------------------
    assign led[0] = pll_locked & camera_init_done;   // 系统健康

    // led[1] = 骨架位图有效（ovl_en）
    //   亮  = 已收到并重组出完整位图（RX+解析+双缓冲 全通）
    //   灭  = 没收到有效帧（RX 时序 或 解析 有问题）
    assign led[1] = ovl_en;

endmodule