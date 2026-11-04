//=============================================================================
// dvp_packer.v — 行首插入头字 {frame_id[3:0], row[11:0]}
//   输入：DVP_Capture 的像素流（每 2 个 pclk 一个 16bit RGB565 字）
//   输出：每行 = 1 个头字 + IMAGE_WIDTH 个像素字，帧号在 VSYNC 上升沿递增
//   该模块被抽出来单测（tb_dvp_packer.v），顶层只做连线
//=============================================================================
`timescale 1ns / 1ps

module dvp_packer(
    input  wire        pclk,
    input  wire        rst_p,        // 高有效
    input  wire        vsync_in,     // DVP_Capture.DataVs
    input  wire        hs_in,        // DVP_Capture.DataHs
    input  wire        valid_in,     // DVP_Capture.DataValid
    input  wire [15:0] data_in,      // DVP_Capture.DataPixel
    input  wire [11:0] row_in,       // DVP_Capture.Yaddr

    output reg  [15:0] word_out,
    output reg         word_valid
);

    reg        hs_d1;
    reg        vs_d1;
    reg [15:0] data_d1;
    reg        valid_d1;

    always @(posedge pclk) begin
        hs_d1    <= hs_in;
        vs_d1    <= vsync_in;
        data_d1  <= data_in;
        valid_d1 <= valid_in;
    end

    wire hs_rise = hs_in  & ~hs_d1;
    wire vs_rise = vsync_in & ~vs_d1;

    reg [3:0] frame_id;
    always @(posedge pclk or posedge rst_p) begin
        if (rst_p)      frame_id <= 4'd0;
        else if (vs_rise) frame_id <= frame_id + 4'd1;
    end

    always @(posedge pclk or posedge rst_p) begin
        if (rst_p)
            word_out <= 16'd0;
        else if (hs_rise)
            word_out <= {frame_id, (row_in - 12'd1)};   // 行首头字
        else if (valid_d1)
            word_out <= data_d1;
    end

    always @(posedge pclk or posedge rst_p) begin
        if (rst_p)
            word_valid <= 1'b0;
        else if (hs_rise)
            word_valid <= 1'b1;
        else if (valid_d1)
            word_valid <= 1'b1;
        else
            word_valid <= 1'b0;
    end

endmodule
