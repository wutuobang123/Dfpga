`timescale 1ns / 1ps
// 集成测试：dvp_packer + eth_tx_fifo 的字节流
//   验证上线的字节序与 tools/udp_recv.py 的解析一致：
//   每个 16bit 字先出高字节，再出低字节（大端 RGB565）
module tb_stream;

    reg         rst_p = 1;
    reg         pclk  = 0;
    always #5   pclk  = ~pclk;      // 100MHz
    reg         rclk  = 0;
    always #4   rclk  = ~rclk;      // 125MHz

    reg         vs = 0, hs = 0, valid = 0;
    reg  [15:0] data = 0;
    reg  [11:0] row  = 0;

    wire [15:0] word;
    wire        word_v;

    dvp_packer u_packer(
        .pclk(pclk), .rst_p(rst_p),
        .vsync_in(vs), .hs_in(hs), .valid_in(valid),
        .data_in(data), .row_in(row),
        .word_out(word), .word_valid(word_v)
    );

    wire [7:0]  Q;
    wire        Empty, Full;
    wire [13:0] Rnum;
    reg  [15:0] fifo_din;

    // 与顶层一致的字节交换
    always @(*) fifo_din = {word[7:0], word[15:8]};

    eth_tx_fifo u_fifo(
        .Data(fifo_din), .Reset(rst_p), .WrClk(pclk), .RdClk(rclk),
        .WrEn(word_v), .RdEn(~Empty), .Rnum(Rnum),
        .Almost_Empty(), .Almost_Full(),
        .Q(Q), .Empty(Empty), .Full(Full)
    );

    localparam W = 4;
    localparam H = 2;

    reg [7:0] exp_q [0:255];
    integer   exp_n = 0;
    integer   got_n = 0;
    integer   errors = 0;

    always @(posedge rclk) begin
        if (!rst_p && !Empty && got_n < exp_n) begin
            if (Q !== exp_q[got_n]) begin
                $display("[ERR] byte %0d: got=%h exp=%h", got_n, Q, exp_q[got_n]);
                errors = errors + 1;
            end
            got_n = got_n + 1;
        end
    end

    task push_word(input [15:0] w);
        begin
            exp_q[exp_n] = w[15:8]; exp_n = exp_n + 1;   // 高字节先
            exp_q[exp_n] = w[7:0];  exp_n = exp_n + 1;   // 低字节后
        end
    endtask

    task do_frame(input [3:0] fid);
        integer r, i;
        begin
            @(posedge pclk); vs = 1;
            @(posedge pclk); vs = 0;
            repeat (3) @(posedge pclk);

            for (r = 0; r < H; r = r + 1) begin
                row = r + 1;
                push_word({fid, r[11:0]});          // 头字

                hs = 1'b1; @(posedge pclk); hs = 1'b0;

                for (i = 0; i < W; i = i + 1) begin
                    data  = 16'h1000 + (r * 16) + i;
                    valid = 1'b0; @(posedge pclk);
                    valid = 1'b1; push_word(data); @(posedge pclk);
                end
                valid = 1'b0;
                repeat (2) @(posedge pclk);
            end
        end
    endtask

    initial begin
        repeat (4) @(posedge pclk);
        rst_p = 0;
        repeat (2) @(posedge pclk);

        do_frame(4'd1);
        do_frame(4'd2);

        // 等字节全部流出
        repeat (4) @(posedge pclk);
        #2000;

        if (got_n !== exp_n) begin
            $display("[ERR] byte count: got=%0d exp=%0d", got_n, exp_n);
            errors = errors + 1;
        end
        if (errors == 0) $display("TB_STREAM: PASS  (%0d bytes, %0d words)", got_n, exp_n/2);
        else             $display("TB_STREAM: FAIL  (%0d errors)", errors);
        $finish;
    end

    initial begin
        #500000;
        $display("TB_STREAM: TIMEOUT");
        $finish;
    end

endmodule
