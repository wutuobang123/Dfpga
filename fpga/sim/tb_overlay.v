`timescale 1ns/1ps
//=============================================================================
// tb_overlay.v — 叠加通路端到端仿真
//   sk01_rx 收 7 个包 -> 双缓冲 -> tft_preview 扫描输出 -> 检查绿色像素
//=============================================================================
module tb_overlay;
    localparam W = 320, H = 240;

    // 时钟
    reg rx_clk = 0, pix_clk = 0, wr_clk = 0;
    always #4    rx_clk  = ~rx_clk;    // 125MHz
    always #13.5 pix_clk = ~pix_clk;   // 37.125MHz
    always #10   wr_clk  = ~wr_clk;    // 50MHz

    reg rst_p = 1;

    // ---- sk01_rx ----
    reg        s_vld = 0, s_done = 0;
    reg  [7:0] s_dat = 0;
    wire [13:0] ovl_addr;
    wire [7:0]  ovl_dat;
    wire        ovl_en, disp_tick;
    wire [15:0] ok_frames, err_pkts;
    wire        dbg_act, dbg_wr, dbg_flip;
    wire [15:0] dbg_nz;

    sk01_rx #(.AGE_MAX(27'd10_000_000)) u_sk01 (
        .rst_p(rst_p), .rx_clk(rx_clk),
        .s_vld(s_vld), .s_dat(s_dat), .s_done(s_done),
        .rd_clk(pix_clk), .disp_tick(disp_tick),
        .rd_addr(ovl_addr), .rd_dat(ovl_dat),
        .ovl_en(ovl_en), .ok_frames(ok_frames), .err_pkts(err_pkts),
        .dbg_active_sel(dbg_act), .dbg_wr_sel(dbg_wr),
        .dbg_flip_pending(dbg_flip), .dbg_nz_bytes(dbg_nz)
    );

    // ---- tft_preview ----
    wire        tft_clk, tft_de, tft_hs, tft_vs;
    wire [15:0] tft_rgb;

    tft_preview u_tft (
        .pix_clk(pix_clk), .rst_n(~rst_p),
        .wr_clk(wr_clk), .wr_en(1'b0), .wr_addr(17'd0), .wr_data(16'd0),
        .ovl_en(ovl_en), .disp_tick(disp_tick),
        .ovl_addr(ovl_addr), .ovl_dat(ovl_dat),
        .tft_clk(tft_clk), .tft_de(tft_de), .tft_hs(tft_hs), .tft_vs(tft_vs),
        .tft_rgb(tft_rgb)
    );

    // ---- 位图：第 10~29 行、第 100~119 列全 1（20x20 块）----
    reg [7:0] bmp [0:9599];

    function [15:0] crc_byte;
        input [15:0] c; input [7:0] d; integer i; reg [15:0] x;
        begin
            x = c ^ {d, 8'h00};
            for (i = 0; i < 8; i = i + 1)
                x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
            crc_byte = x;
        end
    endfunction

    task send_pkt(input integer idx, input integer total);
        integer i, n, base;
        reg [15:0] c;
        reg [7:0] hdr [0:15];
        begin
            n = (idx == total-1) ? 1200 : 1400;
            base = idx * 1400;
            hdr[0]=8'h53; hdr[1]=8'h4B; hdr[2]=8'h30; hdr[3]=8'h31;
            hdr[4]=8'd2;  hdr[5]=8'h01;
            hdr[6]=8'h00; hdr[7]=8'd1;
            hdr[8]=8'd1;  hdr[9]=8'd64;
            hdr[10]=8'd0; hdr[11]=8'd240;
            hdr[12]=8'd37; hdr[13]=8'd128;
            hdr[14]=idx;  hdr[15]=total;
            c = 16'hFFFF;
            for (i = 0; i < 16; i = i + 1) c = crc_byte(c, hdr[i]);
            for (i = 0; i < n;  i = i + 1) c = crc_byte(c, bmp[base+i]);
            for (i = 0; i < 16; i = i + 1) begin
                s_dat = hdr[i]; s_vld = 1; @(posedge rx_clk); s_vld = 0; @(posedge rx_clk);
            end
            for (i = 0; i < n; i = i + 1) begin
                s_dat = bmp[base+i]; s_vld = 1; @(posedge rx_clk); s_vld = 0; @(posedge rx_clk);
            end
            s_dat = c[15:8]; s_vld=1; @(posedge rx_clk); s_vld=0; @(posedge rx_clk);
            s_dat = c[7:0];  s_vld=1; @(posedge rx_clk); s_vld=0; @(posedge rx_clk);
            @(posedge rx_clk); s_done = 1; @(posedge rx_clk); s_done = 0;
        end
    endtask

    // 统计绿色像素（0x07E0）
    integer green_cnt = 0;
    integer de_cnt = 0;
    always @(posedge pix_clk) begin
        if (tft_de) begin
            de_cnt = de_cnt + 1;
            if (tft_rgb == 16'h07E0) green_cnt = green_cnt + 1;
        end
    end

    integer k;
    initial begin
        for (k = 0; k < 9600; k = k + 1) bmp[k] = 8'h00;
        // 20 行 x 20 列 的实心块：行 10..29，列 100..119 -> 字节列 12..14
        for (k = 0; k < 9600; k = k + 1) begin
            if ((k/40) >= 10 && (k/40) <= 29 && (k%40) >= 12 && (k%40) <= 14) begin
                if (k%40 == 12) bmp[k] = 8'h0F;        // 列 100..103 -> bit7..4
                else if (k%40 == 13) bmp[k] = 8'hFF;   // 列 104..111
                else                 bmp[k] = 8'hF0;   // 列 112..115 -> bit7..4
            end
        end

        repeat (10) @(posedge rx_clk); rst_p = 0; repeat (5) @(posedge rx_clk);

        // 发一整帧
        for (k = 0; k < 7; k = k + 1) send_pkt(k, 7);
        repeat (200) @(posedge rx_clk);
        $display("[T1] ok_frames=%0d err=%0d ovl_en=%b act=%b wr=%b flip=%b nz=%0d",
                 ok_frames, err_pkts, ovl_en, dbg_act, dbg_wr, dbg_flip, dbg_nz);

        // 等显示两帧
        green_cnt = 0; de_cnt = 0;
        repeat (2000000) @(posedge pix_clk);
        $display("[T2] 两帧内: de 像素=%0d  绿色像素=%0d (期望 >2000)", de_cnt, green_cnt);

        if (ok_frames == 0)   $display("[ERR] 没有成功帧");
        if (green_cnt < 2000) $display("[ERR] 屏幕上看不到叠加内容");
        if (ok_frames != 0 && green_cnt >= 2000) $display("TB_OVERLAY: PASS");
        else                                    $display("TB_OVERLAY: FAIL");
        $finish;
    end
endmodule
