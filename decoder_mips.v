//====================================================================
// decoder_mips.v   (MIPS multi-cycle CPU, step 6)  -  instruction decoder (combinational, on the IR)
//
//   Legal instructions = mips_mc_spec.md section 3 (strict field checks), masked by FEATURES:
//     base: add sub and or slt lw sw beq addi j syscall break nop      bit 0 ALU   bit 1 SHIFT   bit 2 BR   bit 3 JUMP
//     bit 4 MEMX   bit 5 MUL (mult multu mfhi mflo mthi mtlo, and MIPS32 MUL since Step E)   bit 6 DIV
//   Outputs: 'legal' and one class signal per FSM path (the main FSM in controller_mips_mc.v picks the DECODE successor
//   from them), BrType, ImmSrc, memory size / sign, and the hi/lo selectors. mips_iss.py decode() is the same function
//   (checked word by word in the verification step 'decode-eq').
//====================================================================
module decoder_mips #(
    parameter [6:0] FEATURES = 7'h7F
) (
    input  [31:0] instr,
    output        legal,
    output        c_rtype,      // R-type ALU or shift (incl. nop)        -> EXECUTE
    output        c_itype,      // addi addiu slti sltiu andi ori xori lui -> IEXECUTE
    output        c_load,       //                                         -> MEMADR
    output        c_store,      //                                         -> MEMADR
    output        c_branch,     // beq bne blez bgtz bltz bgez             -> BRANCH
    output        c_j,          //                                         -> JUMP
    output        c_jal,        //                                         -> LINK
    output        c_jalr,       //                                         -> LINK
    output        c_jr,         //                                         -> JR
    output        c_mult,       // mult multu mul (SPECIAL2)               -> MULT (mul: -> MFWB)
    output        c_mthl,       // mthi mtlo                               -> MTHL
    output        c_mf,         // mfhi mflo                               -> MFWB
    output        c_div,        // div divu                                -> DIV
    output        c_stop,       // syscall break                           -> HALT
    output [2:0]  BrType,       // 0 beq 1 bne 2 blez 3 bgtz 4 bltz 5 bgez
    output [1:0]  ImmSrc,       // 00 sign 01 zero 10 upper
    output [1:0]  MemSize,      // 0 word 1 half 2 byte
    output        MemSgn,       // lb lh
    output        HiSel,        // mthi / mfhi (else lo)
    output        MulSigned     // mult / div (else multu / divu)
);
    localparam F_ALU = FEATURES[0], F_SHIFT = FEATURES[1], F_BR = FEATURES[2], F_JUMP = FEATURES[3],
               F_MEMX = FEATURES[4], F_MUL = FEATURES[5], F_DIV = FEATURES[6];
    wire [5:0] op = instr[31:26], fn = instr[5:0];
    wire [4:0] rs = instr[25:21], rt = instr[20:16], rd = instr[15:11], sh = instr[10:6];
    wire rs0 = (rs == 5'd0), rt0 = (rt == 5'd0), rd0 = (rd == 5'd0), sh0 = (sh == 5'd0);
    wire R = (op == 6'h00);

    // ---------------- R-type ----------------
    wire r_base  = R & sh0 & (fn == 6'h20 | fn == 6'h22 | fn == 6'h24 | fn == 6'h25 | fn == 6'h2A);   // add sub and or slt
    wire r_alux  = R & sh0 & (fn == 6'h21 | fn == 6'h23 | fn == 6'h26 | fn == 6'h27 | fn == 6'h2B) & F_ALU;
    wire r_shi   = R & rs0 & (fn == 6'h00 | fn == 6'h02 | fn == 6'h03) & F_SHIFT;                     // sll srl sra
    wire r_shv   = R & sh0 & (fn == 6'h04 | fn == 6'h06 | fn == 6'h07) & F_SHIFT;                     // sllv srlv srav
    wire r_nop   = (instr == 32'd0);
    wire r_jr    = R & (fn == 6'h08) & rt0 & rd0 & sh0 & F_JUMP;
    wire r_jalr  = R & (fn == 6'h09) & rt0 & sh0 & F_JUMP;
    wire r_stop  = R & (fn == 6'h0C | fn == 6'h0D);
    wire r_mf    = R & (fn == 6'h10 | fn == 6'h12) & rs0 & rt0 & sh0 & F_MUL;
    wire r_mt    = R & (fn == 6'h11 | fn == 6'h13) & rt0 & rd0 & sh0 & F_MUL;
    wire r_mult  = R & (fn == 6'h18 | fn == 6'h19) & rd0 & sh0 & F_MUL;
    wire r_div   = R & (fn == 6'h1A | fn == 6'h1B) & rd0 & sh0 & F_DIV;
    wire s2_mul  = (op == 6'h1C) & (fn == 6'h02) & sh0 & F_MUL;        // MIPS32 SPECIAL2 MUL rd = (rs x rt)[31:0] (Step E, G6)

    // ---------------- I / J-type ----------------
    wire i_addi  = (op == 6'h08);
    wire i_alux  = (op == 6'h09 | op == 6'h0A | op == 6'h0B | op == 6'h0C | op == 6'h0D | op == 6'h0E) & F_ALU;
    wire i_lui   = (op == 6'h0F) & rs0 & F_ALU;
    wire ld_w    = (op == 6'h23);
    wire ld_x    = (op == 6'h20 | op == 6'h21 | op == 6'h24 | op == 6'h25) & F_MEMX;
    wire st_w    = (op == 6'h2B);
    wire st_x    = (op == 6'h28 | op == 6'h29) & F_MEMX;
    wire b_beq   = (op == 6'h04);
    wire b_bne   = (op == 6'h05) & F_BR;
    wire b_blez  = (op == 6'h06) & rt0 & F_BR;
    wire b_bgtz  = (op == 6'h07) & rt0 & F_BR;
    wire b_regim = (op == 6'h01) & (rt == 5'd0 | rt == 5'd1) & F_BR;                 // bltz / bgez
    wire j_j     = (op == 6'h02);
    wire j_jal   = (op == 6'h03) & F_JUMP;

    assign c_rtype  = r_base | r_alux | r_shi | r_shv | r_nop;
    assign c_itype  = i_addi | i_alux | i_lui;
    assign c_load   = ld_w | ld_x;
    assign c_store  = st_w | st_x;
    assign c_branch = b_beq | b_bne | b_blez | b_bgtz | b_regim;
    assign c_j      = j_j;
    assign c_jal    = j_jal;
    assign c_jalr   = r_jalr;
    assign c_jr     = r_jr;
    assign c_mult   = r_mult | s2_mul;                                 // MUL: MULT, then MFWB writes LO into rd
    assign c_mthl   = r_mt;
    assign c_mf     = r_mf;
    assign c_div    = r_div;
    assign c_stop   = r_stop;
    assign legal    = c_rtype | c_itype | c_load | c_store | c_branch | c_j | c_jal | c_jalr | c_jr | c_mult | c_mthl |
                      c_mf | c_div | c_stop;

    assign BrType    = (op == 6'h04) ? 3'd0 : (op == 6'h05) ? 3'd1 : (op == 6'h06) ? 3'd2 : (op == 6'h07) ? 3'd3 :
                       rt[0] ? 3'd5 : 3'd4;
    assign ImmSrc    = (op == 6'h0C | op == 6'h0D | op == 6'h0E) ? 2'b01 : (op == 6'h0F) ? 2'b10 : 2'b00;
    assign MemSize   = (op[1:0] == 2'b11) ? 2'd0 : op[0] ? 2'd1 : 2'd2;    // 23/2B word, 21/25/29 half, 20/24/28 byte
    assign MemSgn    = ~op[2];                                             // lb 20 / lh 21 signed; lbu 24 / lhu 25 not
    assign HiSel     = ~fn[1];                                             // mfhi 10 / mthi 11; mflo 12 / mtlo 13
    assign MulSigned = ~fn[0];                                             // mult 18 / div 1A; multu 19 / divu 1B
endmodule
