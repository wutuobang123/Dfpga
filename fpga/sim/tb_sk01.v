`timescale 1ns/1ps
// SK01 v2 解析器验证：7 分片 -> 双缓冲翻转 -> 读回比对
module tb_sk01;
  reg rx_clk = 0, rd_clk = 0, rst_p = 1;
  reg s_vld = 0, s_done = 0;
  reg [7:0] s_dat = 0;
  reg [13:0] rd_addr = 0;
  reg disp_tick = 0;
  wire [7:0] rd_dat;
  wire ovl_en;
  wire [15:0] ok_frames, err_pkts;

  sk01_rx #(.AGE_MAX(26'd1_000_000)) dut(
    .rst_p(rst_p), .rx_clk(rx_clk),
    .s_vld(s_vld), .s_dat(s_dat), .s_done(s_done),
    .rd_clk(rd_clk), .disp_tick(disp_tick), .rd_addr(rd_addr), .rd_dat(rd_dat),
    .ovl_en(ovl_en), .ok_frames(ok_frames), .err_pkts(err_pkts)
  );

  always #4 rx_clk = ~rx_clk;    // 125MHz
  always #13.5 rd_clk = ~rd_clk; // 37.1MHz
  // 模拟显示帧起始：每 50 个 rd_clk 给一个 1 周期脉冲（同步）
  integer tick_cnt = 0;
  always @(posedge rd_clk) begin
    if (tick_cnt == 49) begin tick_cnt <= 0; disp_tick <= 1'b1; end
    else                begin tick_cnt <= tick_cnt + 1; disp_tick <= 1'b0; end
  end

  reg [7:0] bmp [0:9599];
  reg [7:0] hdr [0:15];
  integer errors = 0;

  function [15:0] crc_byte;
    input [15:0] c; input [7:0] d; integer i; reg [15:0] x;
    begin
      x = c ^ {d, 8'h00};
      for (i = 0; i < 8; i = i + 1)
        x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
      crc_byte = x;
    end
  endfunction

  // 发一包：idx 分片号，total 总片数，corrupt=1 时故意改坏 CRC
  task send_pkt(input integer idx, input integer total,
                input integer fid, input integer corrupt);
    integer i, n, base;
    reg [15:0] c;
    begin
      n    = (idx == total - 1) ? 1200 : 1400;
      base = idx * 1400;

      hdr[0]=8'h53; hdr[1]=8'h4B; hdr[2]=8'h30; hdr[3]=8'h31;
      hdr[4]=8'd2;  hdr[5]=8'h01;
      hdr[6]=fid[15:8]; hdr[7]=fid[7:0];
      hdr[8]=8'd1; hdr[9]=8'd64;      // 320
      hdr[10]=8'd0; hdr[11]=8'd240;   // 240
      hdr[12]=8'd37; hdr[13]=8'd128;  // 9600
      hdr[14]=idx;   hdr[15]=total;

      // 计算 CRC（覆盖头 + 数据）
      c = 16'hFFFF;
      for (i = 0; i < 16; i = i + 1) c = crc_byte(c, hdr[i]);
      for (i = 0; i < n;   i = i + 1) c = crc_byte(c, bmp[base + i]);
      if (corrupt) c = c ^ 16'h0001;

      // 逐字节送出
      for (i = 0; i < 16; i = i + 1) begin
        s_dat = hdr[i]; s_vld = 1; @(posedge rx_clk); s_vld = 0; @(posedge rx_clk);
      end
      for (i = 0; i < n; i = i + 1) begin
        s_dat = bmp[base + i]; s_vld = 1; @(posedge rx_clk); s_vld = 0; @(posedge rx_clk);
      end
      s_dat = c[15:8]; s_vld = 1; @(posedge rx_clk); s_vld = 0; @(posedge rx_clk);
      s_dat = c[7:0];  s_vld = 1; @(posedge rx_clk); s_vld = 0; @(posedge rx_clk);

      @(posedge rx_clk); s_done = 1; @(posedge rx_clk); s_done = 0;
      repeat (4) @(posedge rx_clk);
    end
  endtask

  task send_frame(input integer fid, input integer corrupt_idx);
    integer k;
    begin
      for (k = 0; k < 7; k = k + 1)
        send_pkt(k, 7, fid, (corrupt_idx == k));
    end
  endtask

  task read_back(input integer sel);
    integer i;
    begin
      for (i = 0; i < 9600; i = i + 1) begin
        rd_addr = i; @(posedge rd_clk); @(posedge rd_clk);
        if (rd_dat !== bmp[i]) begin
          if (errors < 5) $display("[ERR] addr %0d: got %02x exp %02x", i, rd_dat, bmp[i]);
          errors = errors + 1;
        end
      end
    end
  endtask

  initial begin
    $readmemh("bmp.hex", bmp);
    repeat (5) @(posedge rx_clk); rst_p = 0; repeat (3) @(posedge rx_clk);

    // ---- 用例1：完整一帧 ----
    send_frame(16'd100, -1);
    repeat (200) @(posedge rd_clk);   // 等 vsync 提交翻转
    read_back(0);
    $display("[T1] ok_frames=%0d err=%0d ovl_en=%b", ok_frames, err_pkts, ovl_en);
    if (ok_frames != 1) begin $display("[ERR] T1 帧数应 1"); errors = errors + 1; end
    if (err_pkts != 0)  begin $display("[ERR] T1 不应报错"); errors = errors + 1; end
    if (!ovl_en)        begin $display("[ERR] T1 ovl_en 应有效"); errors = errors + 1; end

    // ---- 用例2：坏 CRC 的包应被丢弃 ----
    send_frame(16'd101, 3);
    $display("[T2] ok_frames=%0d err=%0d", ok_frames, err_pkts);
    if (err_pkts != 1)  begin $display("[ERR] T2 应记录 1 个坏包"); errors = errors + 1; end
    if (ok_frames != 1) begin $display("[ERR] T2 坏帧不应翻转缓冲"); errors = errors + 1; end

    if (errors == 0) $display("TB_SK01: PASS");
    else             $display("TB_SK01: FAIL (%0d)", errors);
    $finish;
  end
endmodule