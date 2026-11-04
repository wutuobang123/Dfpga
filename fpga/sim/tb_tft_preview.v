`timescale 1ns / 1ps
// 验证：320x240 帧存 -> 800x480 面板 的映射、居中、黑边、de 对齐
module tb_tft_preview;

    localparam integer H = 800, V = 480, IW = 320, IH = 240;
    localparam integer XOFF = (H - IW*2)/2;   // 80

    reg clk = 0;
    always #10 clk = ~clk;

    reg rst_n = 0;

    wire        tft_clk, tft_de, tft_hs, tft_vs;
    wire [15:0] tft_rgb;

    tft_preview #(
        .H_ACTIVE(H), .H_FP(40), .H_SW(48), .H_BP(168),
        .V_ACTIVE(V), .V_FP(13), .V_SW(3), .V_BP(90),
        .IMG_W(IW), .IMG_H(IH)
    ) dut (
        .pix_clk(clk), .rst_n(rst_n),
        .wr_clk(clk), .wr_en(1'b0), .wr_addr(17'd0), .wr_data(16'd0),
        .tft_clk(tft_clk), .tft_de(tft_de), .tft_hs(tft_hs), .tft_vs(tft_vs),
        .tft_rgb(tft_rgb)
    );

    // 帧存预置：mem[a] = a 的低 16 位，便于按坐标反查
    integer i;
    initial begin
        for (i = 0; i < IW*IH; i = i + 1) dut.mem[i] = i[15:0];
    end

    integer px_cnt = 0;
    integer errors = 0;
    integer x, y;
    reg [15:0] exp_v;
    reg        in_view;

    always @(posedge clk) begin
        if (rst_n && tft_de) begin
            x = px_cnt % H;
            y = px_cnt / H;
            px_cnt = px_cnt + 1;

            in_view = (x >= XOFF) && (x < XOFF + IW*2) && (y < IH*2);
            if (in_view)
                exp_v = (y/2)*IW + ((x - XOFF)/2);
            else
                exp_v = 16'h0000;

            if (tft_rgb !== exp_v) begin
                errors = errors + 1;
                if (errors <= 5)
                    $display("[ERR] x=%0d y=%0d got=%h exp=%h", x, y, tft_rgb, exp_v);
            end
        end
    end

    initial begin
        repeat (10) @(posedge clk);
        rst_n = 1;
        // 跑一帧多一点
        repeat (H * 700) @(posedge clk);
        if (errors == 0 && px_cnt >= H*V)
            $display("TB_TFT_PREVIEW: PASS  (%0d pixels checked)", px_cnt);
        else
            $display("TB_TFT_PREVIEW: FAIL  errors=%0d pixels=%0d", errors, px_cnt);
        $finish;
    end

endmodule
