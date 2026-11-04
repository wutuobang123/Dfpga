`timescale 1ns / 1ps
// 验证 isp_core：3x3 中值的去椒盐能力 + 输出像素数守恒
module tb_isp;

    localparam integer W = 8;
    localparam integer H = 6;

    reg clk = 0;
    always #5 clk = ~clk;

    reg        rst_p = 1;
    reg        vs = 0, de = 0;
    reg [15:0] pix = 0;

    wire [15:0] o_pix;
    wire        o_de, o_hs, o_vs;
    wire [11:0] o_x, o_y;
    wire [8:0]  o_gain;

    isp_core #(.W(W), .H(H)) dut (
        .pclk(clk), .rst_p(rst_p),
        .vs_in(vs), .de_in(de), .pix_in(pix),
        .pix_out(o_pix), .de_out(o_de), .hs_out(o_hs), .vs_out(o_vs),
        .x_out(o_x), .y_out(o_y), .gain_o(o_gain)
    );

    // 采集输出
    reg [15:0] cap [0:W*H-1];
    integer    cap_n = 0;
    always @(posedge clk) begin
        if (!rst_p && o_de) begin
            if (cap_n < W*H) cap[cap_n] = o_pix;
            cap_n = cap_n + 1;
        end
    end

    // 发一帧：base 为底色，salt 为孤立噪点
    task send_frame(input [15:0] base, input integer sx, input integer sy);
        integer x, y;
        begin
            // 模拟真实 DataVs：消隐期是电平，持续多个时钟
            @(posedge clk); vs = 1;
            repeat (200) @(posedge clk);
            vs = 0;
            repeat (4) @(posedge clk);
            for (y = 0; y < H; y = y + 1) begin
                for (x = 0; x < W; x = x + 1) begin
                    pix = ((x == sx) && (y == sy)) ? 16'hF7FF : base;  // 白点
                    de = 1'b1; @(posedge clk);
                    de = 1'b0; @(posedge clk);
                end
                repeat (3) @(posedge clk);
            end
        end
    endtask

    integer distinct, i;
    reg [15:0] first_val;
    integer errors = 0;

    initial begin
        repeat (5) @(posedge clk);
        rst_p = 0;
        repeat (2) @(posedge clk);

        send_frame(16'h5296, 4, 3);       // 底色 + 一个白噪点
        repeat (40) @(posedge clk);

        // 连续 30 帧亮画面：增益应回落到 GAIN_MIN 附近而不是停在最大
        repeat (30) send_frame(16'h7FFF, 0, 0);
        repeat (40) @(posedge clk);
        if (o_gain > 9'd340) begin
            $display("[ERR] 亮画面下增益未回落: %0d", o_gain);
            errors = errors + 1;
        end

        if (cap_n < W*H) begin
            $display("[ERR] 输出像素数不足: %0d < %0d", cap_n, W*H);
            errors = errors + 1;
        end else begin
            // 检查内部区域（跳过前两行两列）是否已经去噪且均匀
            first_val = cap[2*W + 2];
            distinct  = 0;
            for (i = 0; i < W*H; i = i + 1) begin
                if ((i/W >= 2) && (i%W >= 2)) begin
                    if (cap[i] !== first_val) begin
                        distinct = distinct + 1;
                        if (distinct <= 3)
                            $display("[ERR] 内部像素不一致 @(%0d,%0d) got=%h exp=%h",
                                     i%W, i/W, cap[i], first_val);
                    end
                end
            end
            if (distinct != 0) errors = errors + 1;
        end

        if (errors == 0) $display("TB_ISP: PASS  (pixels=%0d, gain=%0d, value=%h)", cap_n, o_gain, first_val);
        else             $display("TB_ISP: FAIL  (errors=%0d)", errors);
        $finish;
    end

    initial begin
        #500000; $display("TB_ISP: TIMEOUT"); $finish;
    end

endmodule
