`timescale 1ns / 1ps
// 单元测试：行首头字、行号、帧号
module tb_dvp_packer;

    reg        clk = 0;
    always #5 clk = ~clk;

    reg        rst_p = 1;
    reg        vs = 0, hs = 0, valid = 0;
    reg [15:0] data = 0;
    reg [11:0] row  = 0;

    wire [15:0] word_out;
    wire        word_valid;

    dvp_packer dut(
        .pclk(clk), .rst_p(rst_p),
        .vsync_in(vs), .hs_in(hs), .valid_in(valid),
        .data_in(data), .row_in(row),
        .word_out(word_out), .word_valid(word_valid)
    );

    localparam W = 8;
    localparam H = 4;

    reg [15:0] exp_q [0:255];
    integer exp_n = 0;
    integer got_n = 0;
    integer errors = 0;

    always @(posedge clk) begin
        if (word_valid) begin
            if (!rst_p) begin
                if (got_n >= exp_n) begin
                    $display("[ERR] 多出的输出 word=%h @%0t", word_out, $time);
                    errors = errors + 1;
                end else if (word_out !== exp_q[got_n]) begin
                    $display("[ERR] 第 %0d 个不符: got=%h exp=%h", got_n, word_out, exp_q[got_n]);
                    errors = errors + 1;
                end
                got_n = got_n + 1;
            end
        end
    end

    task do_frame(input [3:0] fid);
        integer r, i;
        begin
            @(posedge clk); vs = 1;
            @(posedge clk); vs = 0;
            repeat (3) @(posedge clk);

            for (r = 0; r < H; r = r + 1) begin
                row = r + 1;

                // 期望：行首头字 {fid, r}
                exp_q[exp_n] = {fid, r[11:0]}; exp_n = exp_n + 1;

                hs = 1'b1;
                @(posedge clk);
                hs = 1'b0;

                for (i = 0; i < W; i = i + 1) begin
                    data  = 16'h1000 + (r * 16) + i;
                    valid = 1'b0;
                    @(posedge clk);
                    valid = 1'b1;
                    exp_q[exp_n] = data; exp_n = exp_n + 1;
                    @(posedge clk);
                end
                valid = 1'b0;
                repeat (2) @(posedge clk);
            end
        end
    endtask

    initial begin
        repeat (4) @(posedge clk);
        rst_p = 0;
        repeat (2) @(posedge clk);

        do_frame(4'd1);
        do_frame(4'd2);

        repeat (10) @(posedge clk);

        if (got_n !== exp_n) begin
            $display("[ERR] 字数不符: got=%0d exp=%0d", got_n, exp_n);
            errors = errors + 1;
        end

        if (errors == 0) $display("TB_DVP_PACKER: PASS  (%0d words)", got_n);
        else             $display("TB_DVP_PACKER: FAIL  (%0d errors)", errors);
        $finish;
    end

    initial begin
        #500000;
        $display("TB_DVP_PACKER: TIMEOUT");
        $finish;
    end

endmodule
