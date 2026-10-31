//====================================================================
// div_mips_mc.v   (MIPS multi-cycle CPU, step 12)  -  div / divu: LO = quotient, HI = remainder (iterative, 1 bit/cycle)
//
//   Restoring division on the magnitudes (FEATURES bit 6; removed in the ablation builds without DIV):
//     DIV state     : start = 1 -> load |a|, |b|, signs, count = 0
//     DIVLOOP state : one quotient bit per cycle; in the 32nd cycle done = 1 and q / r carry the FINAL values
//                     (combinational from the last step), so HI / LO are written at the end of that cycle.
//   Signs (div): quotient negative if the operand signs differ, remainder has the sign of the dividend (MIPS).
//   b = 0 (UNPREDICTABLE in MIPS32): q = all ones (sign-fixed), r = a  -- the same rule in mips_iss.py.
//   Cycles: F + D + DIV + 32 x DIVLOOP = 35.
//====================================================================
module div_mips_mc (
    input         clk,
    input         start,
    input  [31:0] a,
    input  [31:0] b,
    input         sgn,
    output        done,
    output [31:0] q,
    output [31:0] r
);
    reg [31:0] quo = 32'd0, rem = 32'd0, ub = 32'd0;
    reg [4:0]  cnt = 5'd0;
    reg        nq = 1'b0, nr = 1'b0;
    wire       na = sgn & a[31];
    wire       nb = sgn & b[31];

    // one restoring step
    wire [32:0] rem2 = {rem, quo[31]};
    wire [33:0] diff = {1'b0, rem2} - {2'b00, ub};
    wire        ge   = ~diff[33];
    wire [31:0] rem_n = ge ? diff[31:0] : rem2[31:0];
    wire [31:0] quo_n = {quo[30:0], ge};

    always @(posedge clk) begin
        if (start) begin
            quo <= na ? -a : a;
            ub  <= nb ? -b : b;
            rem <= 32'd0;
            cnt <= 5'd0;
            nq  <= na ^ nb;
            nr  <= na;
        end else begin
            quo <= quo_n;
            rem <= rem_n;
            cnt <= cnt + 5'd1;
        end
    end
    assign done = (cnt == 5'd31);
    assign q = nq ? -quo_n : quo_n;
    assign r = nr ? -rem_n : rem_n;
endmodule
