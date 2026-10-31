//====================================================================
// cpu_core_mips_mc.v   (MIPS multi-cycle CPU, step 6)  -  core = controller + datapath + memory
//
//   Harris & Harris multicycle MIPS processor (section 7.4, fig. 7.30 "mips" + "mem") with synchronous-read M9K memories.
//   Board view (DBG_EN = 1): while view_rdy = 1 (frozen in FETCH, or halted / fault) the snapshot unit reads
//     dbg_idx 0..31  -> r0..r31  (register-file port 1)
//     dbg_idx 32..63 -> RAM word (idx - 32) at 0x10000000 + 4 (idx - 32)
//   dbg_val belongs to the index presented in the PREVIOUS cycle (synchronous read).
//   Trace outputs (testbenches only): register writes, stores, HI / LO writes.
//====================================================================
module cpu_core_mips_mc #(
    parameter [6:0] FEATURES  = 7'h7F,
    parameter       IMEM_AW   = 7,
    parameter       DMEM_AW   = 5,
    parameter       INIT_FILE = "program.hex",
    parameter       DATA_FILE = "data.hex",
    parameter       DBG_EN    = 1
) (
    input         clk,
    input         rst,
    output        halted,
    output        fault,
    output        retire,
    output [31:0] pc,
    output [31:0] instr,
    output [31:0] hi,
    output [31:0] lo,
    output [4:0]  state,
    output        zero,
    // board view
    input         freeze,
    output        view_rdy,
    input  [5:0]  dbg_idx,
    output [31:0] dbg_val,
    // trace
    output        tr_regw,
    output [4:0]  tr_wa,
    output [31:0] tr_wd,
    output        tr_memw,
    output [31:0] tr_adr,
    output [31:0] tr_mdata,
    output [1:0]  tr_msize,
    output        tr_hiw,
    output        tr_low
);
    wire IorD, IRWrite, PCEn, RegWrite, MemWrite, ALUSrcA, ShVar, HiWrite, LoWrite, DivStart, MulSigned, MemSgn, DivDone;
    wire BrCond, Zero;
    wire [1:0] RegDst, MemtoReg, ALUSrcB, ImmSrc, PCSrc, HiLoSrc, MemSize;
    wire [2:0] BrType;
    wire [3:0] ALUControl;
    wire [31:0] Instr, A, ia_adr, d_adr, d_wdata, mem_instr, d_q, d_rdata, dbg_rd;

    wire dbg = (DBG_EN != 0) & view_rdy;

    controller_mips_mc #(.FEATURES(FEATURES)) ctrl (
        .clk(clk), .rst(rst), .Instr(Instr), .BrCond(BrCond), .Alow(A[1:0]), .DivDone(DivDone), .freeze(freeze),
        .IorD(IorD), .IRWrite(IRWrite), .PCEn(PCEn), .RegWrite(RegWrite), .RegDst(RegDst), .MemtoReg(MemtoReg),
        .MemWrite(MemWrite), .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB), .ALUControl(ALUControl), .ShVar(ShVar), .ImmSrc(ImmSrc),
        .PCSrc(PCSrc), .BrType(BrType), .HiWrite(HiWrite), .LoWrite(LoWrite), .HiLoSrc(HiLoSrc), .DivStart(DivStart),
        .MulSigned(MulSigned), .MemSize(MemSize), .MemSgn(MemSgn), .halted(halted), .fault(fault), .retire(retire),
        .view_rdy(view_rdy), .state(state));

    wire rf_we; wire [4:0] rf_wa; wire [31:0] rf_wd;
    datapath_mips_mc #(.FEATURES(FEATURES)) dp (
        .clk(clk), .rst(rst), .mem_instr(mem_instr), .mem_rdata(d_rdata), .ia_adr(ia_adr), .d_adr(d_adr), .d_wdata(d_wdata),
        .IorD(IorD), .IRWrite(IRWrite), .PCEn(PCEn), .RegWrite(RegWrite), .RegDst(RegDst), .MemtoReg(MemtoReg),
        .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB), .ALUControl(ALUControl), .ShVar(ShVar), .ImmSrc(ImmSrc), .PCSrc(PCSrc),
        .BrType(BrType), .HiWrite(HiWrite), .LoWrite(LoWrite), .HiLoSrc(HiLoSrc), .DivStart(DivStart), .MulSigned(MulSigned),
        .DivDone(DivDone), .Instr(Instr), .BrCond(BrCond), .Zero(Zero), .A(A), .PC(pc), .HI(hi), .LO(lo),
        .rf_we(rf_we), .rf_wa(rf_wa), .rf_wd(rf_wd),
        .dbg(dbg), .dbg_ra(dbg_idx[4:0]), .dbg_adr(32'h10000000 | {25'd0, dbg_idx[4:0], 2'b00}), .dbg_rd(dbg_rd));

    mem_mips_mc #(.IMEM_AW(IMEM_AW), .DMEM_AW(DMEM_AW), .MEMX(FEATURES[4]), .INIT_FILE(INIT_FILE), .DATA_FILE(DATA_FILE)) mem (
        .clk(clk), .ia_adr(ia_adr), .instr(mem_instr), .d_adr(d_adr), .d_we(MemWrite), .d_size(MemSize), .d_sgn(MemSgn),
        .d_wdata(d_wdata), .d_q(d_q), .d_rdata(d_rdata));

    reg idx5 = 1'b0;
    always @(posedge clk) idx5 <= dbg_idx[5];
    assign dbg_val = idx5 ? d_q : dbg_rd;

    assign instr    = Instr;
    assign zero     = Zero;
    assign tr_regw  = rf_we;
    assign tr_wa    = rf_wa;
    assign tr_wd    = rf_wd;
    assign tr_memw  = MemWrite;
    assign tr_adr   = d_adr;
    assign tr_mdata = d_wdata;
    assign tr_msize = MemSize;
    assign tr_hiw   = HiWrite;
    assign tr_low   = LoWrite;
endmodule
