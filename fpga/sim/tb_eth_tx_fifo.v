`timescale 1ns / 1ps
// 单元测试：16bit 写 / 8bit 读、FWFT、字节序（低字节先出）
module tb_eth_tx_fifo;

    reg         Reset = 1;
    reg         WrClk = 0, RdClk = 0;
    always #5  WrClk = ~WrClk;     // 100MHz
    always #4  RdClk = ~RdClk;     // 125MHz

    reg  [15:0] Data = 0;
    reg         WrEn = 0;
    reg         RdEn = 0;
    wire [7:0]  Q;
    wire        Empty, Full;
    wire [13:0] Rnum;

    eth_tx_fifo dut(
        .Data(Data), .Reset(Reset), .WrClk(WrClk), .RdClk(RdClk),
        .WrEn(WrEn), .RdEn(RdEn), .Rnum(Rnum),
        .Almost_Empty(), .Almost_Full(),
        .Q(Q), .Empty(Empty), .Full(Full)
    );

    localparam NW = 16;
    reg [7:0] exp_q [0:63];
    integer exp_n = 0;
    integer got_n = 0;
    integer errors = 0;

    // 读侧采样：RdEn 同拍取 Q
    always @(posedge RdClk) begin
        if (RdEn && !Reset) begin
            if (got_n >= exp_n) begin
                $display("[ERR] 多读: Q=%h", Q); errors = errors + 1;
            end else if (Q !== exp_q[got_n]) begin
                $display("[ERR] 第 %0d 字节不符: got=%h exp=%h", got_n, Q, exp_q[got_n]);
                errors = errors + 1;
            end
            got_n = got_n + 1;
        end
    end

    integer k;
    initial begin
        repeat (5) @(posedge WrClk);
        Reset = 0;
        repeat (3) @(posedge WrClk);

        // 写字：每 2 个 WrClk 一个（模拟 DVP）
        for (k = 0; k < NW; k = k + 1) begin
            Data = 16'hC000 + k;
            WrEn = 1'b1;
            @(posedge WrClk);
            WrEn = 1'b0;
            @(posedge WrClk);
            exp_q[exp_n] = 8'h00 + k;        // 低字节先出
            exp_n = exp_n + 1;
            exp_q[exp_n] = 8'hC0;            // 高字节
            exp_n = exp_n + 1;
        end

        // 等状态稳定，检查 FWFT：未读时 Q 就应该是队首
        repeat (6) @(posedge RdClk);
        if (Empty) begin
            $display("[ERR] FIFO 不该为空"); errors = errors + 1;
        end
        if (Q !== exp_q[0]) begin
            $display("[ERR] FWFT 失败: Q=%h 期望队首=%h", Q, exp_q[0]);
            errors = errors + 1;
        end
        if (Rnum !== (NW*2)) begin
            $display("[ERR] Rnum=%0d 期望 %0d", Rnum, NW*2); errors = errors + 1;
        end

        // 连续读
        for (k = 0; k < NW*2; k = k + 1) begin
            RdEn = 1'b1;
            @(posedge RdClk);
        end
        RdEn = 1'b0;
        repeat (4) @(posedge RdClk);

        if (got_n !== NW*2) begin
            $display("[ERR] 读出字节数 %0d 期望 %0d", got_n, NW*2); errors = errors + 1;
        end
        if (!Empty) begin
            $display("[ERR] 读完后不该非空"); errors = errors + 1;
        end

        if (errors == 0) $display("TB_ETH_TX_FIFO: PASS  (%0d bytes)", got_n);
        else             $display("TB_ETH_TX_FIFO: FAIL  (%0d errors)", errors);
        $finish;
    end

    initial begin
        #200000;
        $display("TB_ETH_TX_FIFO: TIMEOUT");
        $finish;
    end

endmodule
