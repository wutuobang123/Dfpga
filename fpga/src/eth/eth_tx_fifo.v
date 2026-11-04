//=============================================================================
// eth_tx_fifo.v — 自研 16bit写 / 8bit读 异步 FIFO（FWFT）
//   替换官方加密 IP：原 IP 为 GW5AT-138 生成，在 60K 上被推断成 SSRAM，
//   而 GW5AT-60 没有 SSRAM 资源 -> 综合报 RP0007。
//
//   写侧：WrEn 每 2 拍来一个 16bit 字（DVP 特性），本模块拆成 2 个字节，
//         每拍写 1 字节，与上游平均速率正好匹配；2 字队列吸收突发。
//   读侧：FWFT（首字直通），Q 恒等于队首字节，满足 MAC 的
//         "payload_req_o 同拍取 payload_dat_i"。
//   存储：8bit x 2048 -> BSRAM。
//=============================================================================
`timescale 1ns / 1ps

module eth_tx_fifo(
    input  wire [15:0] Data,
    input  wire        Reset,
    input  wire        WrClk,
    input  wire        RdClk,
    input  wire        WrEn,
    input  wire        RdEn,
    output wire [13:0] Rnum,
    output wire        Almost_Empty,
    output wire        Almost_Full,
    output wire [7:0]  Q,
    output wire        Empty,
    output wire        Full
);

    localparam AW    = 11;
    localparam DEPTH = (1 << AW);

    (* syn_ramstyle = "block_ram" *) reg [7:0] mem [0:DEPTH-1];

    //==================================================================
    // 写侧：16bit 字 -> 2 个 8bit 字节
    //==================================================================
    reg [15:0] wq0, wq1;
    reg [1:0]  wq_cnt;
    reg        bphase;
    reg [AW:0] wbin;
    reg [AW:0] wgray;

    reg [15:0] nq0, nq1;
    reg [1:0]  ncnt;

    wire       wr_disp   = (wq_cnt != 2'd0);
    wire [7:0] wr_byte   = bphase ? wq0[15:8] : wq0[7:0];
    wire       word_done = wr_disp & bphase;
    wire       enq       = WrEn & (wq_cnt < 2'd2);

    always @(posedge WrClk or posedge Reset) begin
        if (Reset) begin
            wq0 <= 16'd0; wq1 <= 16'd0; wq_cnt <= 2'd0; bphase <= 1'b0;
            wbin <= 0;    wgray <= 0;
        end else begin
            // ---- 存储写入 ----
            if (wr_disp) begin
                mem[wbin[AW-1:0]] <= wr_byte;
                wbin  <= wbin + 1'b1;
                wgray <= ((wbin + 1'b1) >> 1) ^ (wbin + 1'b1);
                bphase <= ~bphase;
            end

            // ---- 队列：先入后出 ----
            nq0 = wq0; nq1 = wq1; ncnt = wq_cnt;
            if (enq) begin
                if (ncnt == 2'd0)      nq0 = Data;
                else if (ncnt == 2'd1) nq1 = Data;
                ncnt = ncnt + 1'b1;
            end
            if (word_done) begin
                nq0  = nq1;
                ncnt = ncnt - 1'b1;
            end
            wq0 <= nq0; wq1 <= nq1; wq_cnt <= ncnt;
        end
    end

    //==================================================================
    // 指针跨时钟同步（格雷码，打两拍）
    //==================================================================
    reg [AW:0] wgray_rs1, wgray_rs2;
    reg [AW:0] rgray_ws1, rgray_ws2;
    reg [AW:0] rbin, rgray;

    always @(posedge RdClk or posedge Reset) begin
        if (Reset) begin wgray_rs1 <= 0; wgray_rs2 <= 0; end
        else begin wgray_rs1 <= wgray; wgray_rs2 <= wgray_rs1; end
    end

    always @(posedge WrClk or posedge Reset) begin
        if (Reset) begin rgray_ws1 <= 0; rgray_ws2 <= 0; end
        else begin rgray_ws1 <= rgray; rgray_ws2 <= rgray_ws1; end
    end

    //==================================================================
    // 读侧：FWFT
    //==================================================================
    reg [7:0]    q_reg;

    wire rempty = (rgray == wgray_rs2);
    wire radv   = RdEn & ~rempty;

    // FWFT：地址取"下一次要读的位置"，数据在下一拍出现在 q_reg，
    // 正好等于新的队首。不要再插一级地址寄存器，否则会变成 2 拍延迟。
    wire [AW-1:0] raddr = rbin[AW-1:0] + radv;

    always @(posedge RdClk or posedge Reset) begin
        if (Reset) begin
            rbin <= 0; rgray <= 0; q_reg <= 8'd0;
        end else begin
            q_reg <= mem[raddr];
            if (radv) begin
                rbin  <= rbin + 1'b1;
                rgray <= ((rbin + 1'b1) >> 1) ^ (rbin + 1'b1);
            end
        end
    end

    //==================================================================
    // 标志与计数
    //==================================================================
    reg [AW:0] rbin_ws;      // 写域看到的读指针（二进制）
    integer i;
    always @(*) begin
        rbin_ws[AW] = rgray_ws2[AW];
        for (i = AW-1; i >= 0; i = i-1)
            rbin_ws[i] = rbin_ws[i+1] ^ rgray_ws2[i];
    end

    reg [AW:0] wbin_rs;      // 读域看到的写指针（二进制）
    integer k;
    always @(*) begin
        wbin_rs[AW] = wgray_rs2[AW];
        for (k = AW-1; k >= 0; k = k-1)
            wbin_rs[k] = wbin_rs[k+1] ^ wgray_rs2[k];
    end

    reg [AW:0] rbin_bin;
    integer m;
    always @(*) begin
        rbin_bin[AW] = rgray[AW];
        for (m = AW-1; m >= 0; m = m-1)
            rbin_bin[m] = rbin_bin[m+1] ^ rgray[m];
    end

    wire [AW:0] used_words = wbin_rs - rbin_ws;   // 写域：已用字数
    wire [AW:0] used_bytes = wbin_rs - rbin_bin;  // 读域：已用字节数

    assign Empty = rempty;
    assign Full  = (used_words == DEPTH[AW:0]);

    assign Rnum         = {2'b00, used_bytes};        // wbin/rbin 本身按字节计数
    assign Almost_Empty = (used_bytes < 2);
    assign Almost_Full  = (used_bytes > (DEPTH - 4));

    assign Q = q_reg;

endmodule
