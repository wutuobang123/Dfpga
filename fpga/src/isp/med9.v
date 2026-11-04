//=============================================================================
// med9.v — 3x3 精确中值（Smith 网络：列排序 -> 取 min/med/max 的三元中值）
//   比"中值的中值"近似更准，且只多两个比较器
//=============================================================================
`timescale 1ns / 1ps

module med9 #(parameter integer W = 5)(
    input  wire [W-1:0] p00, p01, p02,
    input  wire [W-1:0] p10, p11, p12,
    input  wire [W-1:0] p20, p21, p22,
    output wire [W-1:0] med
);
    // 三元素排序
    function [W-1:0] mn3; input [W-1:0] a,b,c;
        mn3 = (a<b) ? ((a<c)?a:c) : ((b<c)?b:c);
    endfunction
    function [W-1:0] mx3; input [W-1:0] a,b,c;
        mx3 = (a>b) ? ((a>c)?a:c) : ((b>c)?b:c);
    endfunction
    function [W-1:0] md3; input [W-1:0] a,b,c;
        begin
            md3 = (a<b) ? ((b<c)?b : ((a<c)?c:a))
                        : ((a<c)?a : ((b<c)?c:b));
        end
    endfunction

    wire [W-1:0] c0mn = mn3(p00,p10,p20);
    wire [W-1:0] c0md = md3(p00,p10,p20);
    wire [W-1:0] c0mx = mx3(p00,p10,p20);
    wire [W-1:0] c1mn = mn3(p01,p11,p21);
    wire [W-1:0] c1md = md3(p01,p11,p21);
    wire [W-1:0] c1mx = mx3(p01,p11,p21);
    wire [W-1:0] c2mn = mn3(p02,p12,p22);
    wire [W-1:0] c2md = md3(p02,p12,p22);
    wire [W-1:0] c2mx = mx3(p02,p12,p22);

    assign med = md3( mx3(c0mn,c1mn,c2mn),
                      md3(c0md,c1md,c2md),
                      mn3(c0mx,c1mx,c2mx) );
endmodule
