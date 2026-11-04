/////////////////////////////////////////////////////////////////////////////////
// Module Name : sk01_rx
// Description : SK01 v2 分片包解析 + 双缓冲位图（骨架 OSD）
//
//   包格式（大端，头 16 字节）:
//     0  magic "SK01"   4  version=2   5  flags
//     6  frame_id(2)    8  width(2)    10 height(2)
//     12 payload_len(2)=整幅总长        14 pkt_idx   15 pkt_total
//     16 data[N]        16+N crc16(2)   CRC 覆盖 0..16+N-1
//
//   内存：双缓冲 2 x 9600B，写端口在收包时钟域，读端口在显示时钟域
/////////////////////////////////////////////////////////////////////////////////
module sk01_rx #(
    parameter integer BMP_BYTES = 9600,      // 320*240/8
    parameter integer CHUNK     = 1400,      // 每包最大数据
    parameter integer AGE_MAX   = 27'd41_250_000   // 无更新超时 ~0.33s @125MHz（对齐验收标准）
)(
    input  wire        rst_p,
    // ---- 收包侧（gmii_rx_clk 域）----
    input  wire        rx_clk,
    input  wire        s_vld,
    input  wire [7:0]  s_dat,
    input  wire        s_done,
    // ---- 显示侧（像素时钟域）----
    input  wire        rd_clk,
    input  wire        disp_tick,     // 显示帧起始脉冲（用于同步翻转，避免撕裂）
    input  wire [13:0] rd_addr,
    output reg  [7:0]  rd_dat,
    // ---- 状态 ----
    output wire        ovl_en,        // 位图有效（帧龄未超时）
    output reg  [15:0] ok_frames,     // 成功帧数（调试）
    output reg  [15:0] err_pkts,      // CRC/格式错误包数（调试）
    // ---- 调试观测 ----
    output wire        dbg_active_sel,
    output wire        dbg_wr_sel,
    output wire        dbg_flip_pending,
    output wire [15:0] dbg_nz_bytes   // 累计写入的非零字节数
);

    //---------------- CRC16-CCITT ----------------
    function [15:0] crc_byte;
        input [15:0] c;
        input [7:0]  d;
        integer i;
        reg [15:0] x;
        begin
            x = c ^ {d, 8'h00};
            for (i = 0; i < 8; i = i + 1)
                x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
            crc_byte = x;
        end
    endfunction

    //---------------- 收包状态 ----------------
    reg [11:0] byte_cnt;        // 包内字节序号
    reg [15:0] frame_id;
    reg [15:0] payload_len;
    reg [7:0]  pkt_idx, pkt_total;
    reg [15:0] data_len;        // 本包数据长度
    reg [15:0] crc_calc;
    reg [15:0] crc_recv;
    reg        crc_ok;
    reg        magic_ok, ver_ok;

    wire [15:0] data_end = 16'd16 + data_len;

    // 写地址 = pkt_idx*CHUNK + (byte_cnt-16)
    wire [13:0] wr_addr = pkt_idx * CHUNK + (byte_cnt - 12'd16);
    wire        wr_en   = s_vld && (byte_cnt >= 12'd16) && ({4'd0,byte_cnt} < data_end);

    // 双缓冲
    (* ram_style = "block" *) reg [7:0] mem_a [0:BMP_BYTES-1];
    (* ram_style = "block" *) reg [7:0] mem_b [0:BMP_BYTES-1];
    reg        wr_sel;          // 写哪一块（= active_sel 取反）
    reg        active_sel;      // 显示读哪一块
    reg [13:0] wr_addr_r;
    reg [7:0]  wr_dat_r;
    reg        wr_en_r;

    always @(posedge rx_clk) begin
        wr_addr_r <= wr_addr;
        wr_dat_r  <= s_dat;
        wr_en_r   <= wr_en;
        if (wr_en_r) begin
            if (wr_sel) mem_b[wr_addr_r] <= wr_dat_r;
            else        mem_a[wr_addr_r] <= wr_dat_r;
        end
    end

    always @(posedge rd_clk) begin
        rd_dat <= active_sel ? mem_b[rd_addr] : mem_a[rd_addr];
    end

    //---------------- 帧组装 ----------------
    reg [7:0]  pkt_seen;        // 当前帧已收到的分片位图
    reg [15:0] cur_fid;
    reg        fid_valid;
    reg [26:0] age_cnt;
    reg [15:0] tgt_total;
    reg        frame_done;      // rx 域：本帧收齐
    reg        ovl_rx;          // rx 域的有效标志
    reg        ovl_s1, ovl_s2;  // 同步到显示域

    // 帧龄计数放在 rx 域（125MHz），避免用慢时钟采快速脉冲导致漏采
    always @(posedge rx_clk or posedge rst_p) begin
        if (rst_p)                    age_cnt <= 27'd0;
        else if (frame_done)          age_cnt <= 27'd0;
        else if (age_cnt < AGE_MAX)   age_cnt <= age_cnt + 27'd1;
    end

    always @(posedge rx_clk or posedge rst_p) begin
        if (rst_p) ovl_rx <= 1'b0;
        else       ovl_rx <= fid_valid && (age_cnt < AGE_MAX);
    end

    // 慢变信号，两拍同步足够
    always @(posedge rd_clk or posedge rst_p) begin
        if (rst_p) begin ovl_s1 <= 1'b0; ovl_s2 <= 1'b0; end
        else       begin ovl_s1 <= ovl_rx; ovl_s2 <= ovl_s1; end
    end

    assign ovl_en = ovl_s2;

    // 统计写入的非零字节（验证位图数据真的进来了）
    reg [15:0] nz_cnt;
    always @(posedge rx_clk or posedge rst_p) begin
        if (rst_p) nz_cnt <= 16'd0;
        else if (wr_en && (s_dat != 8'd0)) nz_cnt <= nz_cnt + 16'd1;
    end
    assign dbg_nz_bytes = nz_cnt;

    // 翻转与显示帧同步：避免扫描中途换缓冲导致撕裂
    // wr_sel 是接完一整帧才翻转的慢变信号，两拍同步后做边沿检测
    reg w_s1, w_s2, w_s3;
    always @(posedge rd_clk or posedge rst_p) begin
        if (rst_p) begin w_s1 <= 1'b0; w_s2 <= 1'b0; w_s3 <= 1'b0; end
        else       begin w_s1 <= wr_sel; w_s2 <= w_s1; w_s3 <= w_s2; end
    end

    reg flip_pending, pending_sel;
    always @(posedge rd_clk or posedge rst_p) begin
        if (rst_p) begin
            active_sel   <= 1'b0;
            flip_pending <= 1'b0;
            pending_sel  <= 1'b0;
        end else begin
            if (w_s2 != w_s3) begin          // rx 域刚翻过一次 -> 有新帧可用
                pending_sel  <= w_s3;        // 刚写满的那块
                flip_pending <= 1'b1;
            end else if (flip_pending && disp_tick) begin
                active_sel   <= pending_sel; // 只在显示帧起点切换
                flip_pending <= 1'b0;
            end
        end
    end

    assign dbg_active_sel   = active_sel;
    assign dbg_wr_sel       = wr_sel;
    assign dbg_flip_pending = flip_pending;

    always @(posedge rx_clk or posedge rst_p) begin
        if (rst_p) begin
            byte_cnt  <= 12'd0;
            crc_calc  <= 16'hFFFF;
            crc_recv  <= 16'd0;
            crc_ok    <= 1'b0;
            magic_ok  <= 1'b0;
            ver_ok    <= 1'b0;
            pkt_idx   <= 8'd0;
            pkt_total <= 8'd0;
            data_len  <= 16'd0;
            payload_len <= 16'd0;
            frame_id  <= 16'd0;
            wr_sel    <= 1'b1;
            pkt_seen  <= 8'd0;
            cur_fid   <= 16'd0;
            fid_valid <= 1'b0;
            ok_frames <= 16'd0;
            err_pkts  <= 16'd0;
            frame_done<= 1'b0;
            tgt_total <= 16'd0;
        end else begin
            frame_done <= 1'b0;

            if (s_vld) begin
                // ---- 头部逐字节解析 ----
                case (byte_cnt)
                    12'd0: magic_ok <= (s_dat == 8'h53);
                    12'd1: magic_ok <= magic_ok && (s_dat == 8'h4B);
                    12'd2: magic_ok <= magic_ok && (s_dat == 8'h30);
                    12'd3: magic_ok <= magic_ok && (s_dat == 8'h31);
                    12'd4: ver_ok   <= (s_dat == 8'd2);
                    12'd6: frame_id[15:8] <= s_dat;
                    12'd7: frame_id[7:0]  <= s_dat;
                    12'd12: payload_len[15:8] <= s_dat;
                    12'd13: payload_len[7:0]  <= s_dat;
                    12'd14: pkt_idx    <= s_dat;
                    12'd15: begin
                        pkt_total <= s_dat;
                        tgt_total <= {8'd0, s_dat};
                        // 末片长度 = 总长 - pkt_idx*CHUNK，其余为 CHUNK
                        data_len  <= (s_dat != 8'd0 && pkt_idx == s_dat - 8'd1)
                                     ? (payload_len - pkt_idx * CHUNK)
                                     : CHUNK;
                    end
                    default: ;
                endcase

                // ---- CRC 累计（覆盖头+数据，不含末尾 2 字节）----
                if ({4'd0, byte_cnt} < data_end)
                    crc_calc <= crc_byte(crc_calc, s_dat);

                // ---- CRC 两字节 ----
                if ({4'd0, byte_cnt} == data_end)
                    crc_recv[15:8] <= s_dat;
                if ({4'd0, byte_cnt} == data_end + 16'd1)
                    crc_recv[7:0]  <= s_dat;

                byte_cnt <= byte_cnt + 12'd1;
            end

            if (s_done) begin
                byte_cnt <= 12'd0;
                crc_calc <= 16'hFFFF;

                // 长度对 + magic + ver + CRC 全过才算有效包
                if (magic_ok && ver_ok &&
                    (crc_calc == crc_recv) &&
                    ({4'd0, byte_cnt} == data_end + 16'd2)) begin

                    // 换帧则清零分片位图
                    if (!fid_valid || frame_id != cur_fid) begin
                        cur_fid  <= frame_id;
                        pkt_seen <= 8'd0;
                        fid_valid<= 1'b1;
                    end
                    pkt_seen[pkt_idx] <= 1'b1;

                    // 收齐所有分片 -> 翻转双缓冲
                    if ((pkt_seen | (8'd1 << pkt_idx)) == ((8'd1 << tgt_total) - 8'd1)) begin
                        wr_sel       <= ~wr_sel;    // 立刻换到另一块继续写
                        ok_frames    <= ok_frames + 16'd1;
                        frame_done   <= 1'b1;
                        pkt_seen     <= 8'd0;
                    end
                end else begin
                    err_pkts <= err_pkts + 16'd1;
                end
                magic_ok <= 1'b0;
                crc_ok   <= 1'b0;
            end
        end
    end

endmodule