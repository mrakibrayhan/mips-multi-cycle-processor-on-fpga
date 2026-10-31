//====================================================================
// regfile_mips_mc.v   (MIPS multi-cycle CPU, step 4)  -  32 x 32 register file with SYNCHRONOUS read (two M9K copies)
//
//   Harris & Harris multicycle datapath (section 7.4.1): A1 = rs, A2 = rt from the IR, RD1 / RD2 are registered into the
//   non-architectural registers A and B at every edge; A3 / WD3 written at the end of a writeback state.
//   Here each read port is an M9K copy. The datapath drives the read ADDRESS input with
//        IRWrite ? fields of the memory output (the word being loaded into IR at this edge) : fields of IR
//   so the M9K address register always holds the IR's rs / rt and the M9K output registers ARE A and B
//   (mips_mc_spec.md section 6): valid from DECODE on, one cycle earlier than [HH]'s A / B, same value in every state that
//   uses them.
//
//   WRITE: at the end of MEMWB / ALUWB / IWB / JLWB / MFWB (same edge as [HH]). The next instruction latches its read
//   address one edge later (end of FETCH), so it always sees the new value: no bypass is needed. On the write edge itself
//   the read address may equal the write address (e.g. add $1,$1,$2): that output word is never used (the next state is
//   FETCH), so the M9K mixed-port read-during-write result does not matter ("no_rw_check").
//   $0: writes to address 0 are ignored, so rf[0] stays 0.
//   RESET: rst = 1 writes 0 to r0..r31, one register per cycle (counter 'clr'); rst must last >= 32 cycles (the board
//   holds it for milliseconds, the testbenches for 40 cycles).
//   Board view: the datapath puts the snapshot index on ra1 while the CPU is frozen (dbg).
//====================================================================
module regfile_mips_mc (
    input         clk,
    input         rst,
    input  [4:0]  ra1,          // read address input (latched at this edge)
    input  [4:0]  ra2,
    output [31:0] rd1,          // = rf[ra1 latched at the last edge]  (A)
    output [31:0] rd2,          // = rf[ra2 latched at the last edge]  (B)
    input         we,           // RegWrite
    input  [4:0]  wa,           // A3
    input  [31:0] wd            // WD3
);
    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] rf1 [0:31];
    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] rf2 [0:31];
    integer i;
    initial for (i = 0; i < 32; i = i + 1) begin rf1[i] = 32'd0; rf2[i] = 32'd0; end

    reg  [4:0]  clr = 5'd0;
    always @(posedge clk) clr <= rst ? clr + 5'd1 : 5'd0;
    wire        wen = rst | (we & (wa != 5'd0));
    wire [4:0]  wad = rst ? clr : wa;
    wire [31:0] wdd = rst ? 32'd0 : wd;

    reg [31:0] q1, q2;                                   // M9K read registers (no power-up value)
    always @(posedge clk) begin
        if (wen) rf1[wad] <= wdd;
        q1 <= rf1[ra1];
    end
    always @(posedge clk) begin
        if (wen) rf2[wad] <= wdd;
        q2 <= rf2[ra2];
    end
    assign rd1 = q1;
    assign rd2 = q2;
endmodule
