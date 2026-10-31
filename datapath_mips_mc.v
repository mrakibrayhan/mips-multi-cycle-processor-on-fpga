//====================================================================
// datapath_mips_mc.v   (MIPS multi-cycle CPU, step 5)  -  Harris & Harris multicycle datapath (section 7.4.1, fig. 7.27)
//                      + the extensions of mips_mc_spec.md section 5, with SYNCHRONOUS-read M9K memories (section 6)
//
//   Registers (flip-flops): PC (enable PCEn), IR (IRWrite), Data (every cycle), ALUOut (every cycle), HI / LO.
//   A / B = the output registers of the register-file M9K copies (regfile_mips_mc.v), read address = IR's rs / rt.
//   Muxes ([HH] names):
//     memory address  IorD      0 PC        1 ALUOut          (two M9K ports, see the comments of mem_mips_mc.sv)
//     SrcA            ALUSrcA   0 PC        1 A
//     SrcB            ALUSrcB   00 B   01 4   10 ImmExt   11 SignImm << 2
//     A3              RegDst    00 rt  01 rd  10 31 (jal)
//     WD3             MemtoReg  00 ALUOut   01 Data   10 HI   11 LO
//     PCNext          PCSrc     00 ALUResult   01 ALUOut   10 PCJump = {PC[31:28], Instr[25:0], 00}
//   ImmExt  ImmSrc  00 sign   01 zero   10 upper {imm, 16'b0}  (lui: rs = 0, so ALU = 0 + ImmExt)
//   Shift amount = ShVar ? SrcA[4:0] (rs) : Instr[10:6] (shamt).
//   BrCond (to the controller, PCEn = PCWrite | Branch & BrCond):
//     BrType 0 beq Zero   1 bne ~Zero   2 blez A[31] | Zero   3 bgtz ~A[31] & ~Zero   4 bltz A[31]   5 bgez ~A[31]
//   HI / LO (MUL or DIV built): HiWrite / LoWrite, HiLoSrc 0 = A (mthi / mtlo), 1 = product (mult), 2 = divider (div).
//     The multiplier (mul_mips_mc.v, step 11) and the divider (div_mips_mc.v, step 12) are instantiated only when
//     their FEATURES bit is set.
//   SYNCHRONOUS-read addresses (the M9K address registers hold what the [HH] registers hold):
//     ia_adr = PCEn ? PCNext : PC            (ROM port A -> instr = mem[PC])
//     d_adr  = IorD ? ALUOut : ALUResult     (data port  -> mem[ALUOut] in MEMREAD; store address in MEMWRITE)
//     ra1/2  = IRWrite ? memory-word fields : IR fields   (register file -> A = rf[rs], B = rf[rt])
//   Board view (dbg = 1, CPU frozen in FETCH or stopped): ra1 = dbg_ra, d_adr = dbg_adr.
//====================================================================
module datapath_mips_mc #(
    parameter [6:0] FEATURES = 7'h7F
) (
    input         clk,
    input         rst,
    // memory
    input  [31:0] mem_instr,     // ROM port A output = mem[PC]
    input  [31:0] mem_rdata,     // data port output (aligned) = mem[ALUOut]
    output [31:0] ia_adr,
    output [31:0] d_adr,
    output [31:0] d_wdata,
    // control (FSM outputs)
    input         IorD,
    input         IRWrite,
    input         PCEn,
    input         RegWrite,
    input  [1:0]  RegDst,
    input  [1:0]  MemtoReg,
    input         ALUSrcA,
    input  [1:0]  ALUSrcB,
    input  [3:0]  ALUControl,
    input         ShVar,
    input  [1:0]  ImmSrc,
    input  [1:0]  PCSrc,
    input  [2:0]  BrType,
    input         HiWrite,
    input         LoWrite,
    input  [1:0]  HiLoSrc,
    input         DivStart,
    input         MulSigned,
    output        DivDone,
    // status
    output reg [31:0] Instr,
    output        BrCond,
    output        Zero,
    output [31:0] A,
    output reg [31:0] PC,
    output reg [31:0] HI,
    output reg [31:0] LO,
    // register-file write (trace)
    output        rf_we,
    output [4:0]  rf_wa,
    output [31:0] rf_wd,
    // board view
    input         dbg,
    input  [4:0]  dbg_ra,
    input  [31:0] dbg_adr,
    output [31:0] dbg_rd
);
    localparam ALUX = FEATURES[0], SHIFT = FEATURES[1], MUL = FEATURES[5], DIV = FEATURES[6];
    localparam HILO = MUL | DIV;

    // ---------------- non-architectural registers ----------------
    reg [31:0] Data, ALUOut;
    wire [31:0] ALUResult, PCNext, B;
    initial begin PC = 32'd0; Instr = 32'd0; HI = 32'd0; LO = 32'd0; end

    always @(posedge clk) begin
        if (rst)          PC <= 32'd0;
        else if (PCEn)    PC <= PCNext;
        if (rst)          Instr <= 32'd0;
        else if (IRWrite) Instr <= mem_instr;
        Data   <= mem_rdata;
        ALUOut <= ALUResult;
    end

    // ---------------- register file (A / B = its M9K output registers) ----------------
    wire [31:0] rd1, rd2;
    wire [4:0]  ra1 = dbg ? dbg_ra : IRWrite ? mem_instr[25:21] : Instr[25:21];
    wire [4:0]  ra2 = IRWrite ? mem_instr[20:16] : Instr[20:16];
    wire [4:0]  WriteReg = (RegDst == 2'b01) ? Instr[15:11] : (RegDst == 2'b10) ? 5'd31 : Instr[20:16];
    reg  [31:0] Result;
    always @(*) begin
        case (MemtoReg)
            2'b00:   Result = ALUOut;
            2'b01:   Result = Data;
            2'b10:   Result = HILO ? HI : 32'd0;
            default: Result = HILO ? LO : 32'd0;
        endcase
    end
    regfile_mips_mc rf (.clk(clk), .rst(rst), .ra1(ra1), .ra2(ra2), .rd1(rd1), .rd2(rd2),
                        .we(RegWrite), .wa(WriteReg), .wd(Result));
    assign A = rd1;
    assign B = rd2;
    assign rf_we = RegWrite & (WriteReg != 5'd0);
    assign rf_wa = WriteReg;
    assign rf_wd = Result;
    assign dbg_rd = rd1;

    // ---------------- immediate, ALU sources ----------------
    wire [31:0] SignImm = {{16{Instr[15]}}, Instr[15:0]};
    wire [31:0] ImmExt  = (ImmSrc == 2'b01) ? {16'd0, Instr[15:0]} : (ImmSrc == 2'b10) ? {Instr[15:0], 16'd0} : SignImm;
    wire [31:0] SrcA    = ALUSrcA ? A : PC;
    reg  [31:0] SrcB;
    always @(*) begin
        case (ALUSrcB)
            2'b00:   SrcB = B;
            2'b01:   SrcB = 32'd4;
            2'b10:   SrcB = ImmExt;
            default: SrcB = {SignImm[29:0], 2'b00};
        endcase
    end
    wire [4:0] shamt = ShVar ? SrcA[4:0] : Instr[10:6];
    alu_mips #(.ALUX(ALUX), .SHIFT(SHIFT)) alu (.a(SrcA), .b(SrcB), .ctl(ALUControl), .shamt(shamt), .y(ALUResult), .zero(Zero));

    // ---------------- branch condition ----------------
    reg bc;
    always @(*) begin
        case (BrType)
            3'd0:    bc = Zero;
            3'd1:    bc = ~Zero;
            3'd2:    bc = A[31] | Zero;
            3'd3:    bc = ~A[31] & ~Zero;
            3'd4:    bc = A[31];
            default: bc = ~A[31];
        endcase
    end
    assign BrCond = bc;

    // ---------------- next PC ----------------
    assign PCNext = (PCSrc == 2'b00) ? ALUResult : (PCSrc == 2'b01) ? ALUOut : {PC[31:28], Instr[25:0], 2'b00};

    // ---------------- synchronous-read address inputs ----------------
    assign ia_adr  = rst ? 32'd0 : PCEn ? PCNext : PC;
    assign d_adr   = dbg ? dbg_adr : IorD ? ALUOut : ALUResult;
    assign d_wdata = B;

    // ---------------- HI / LO, multiplier, divider ----------------
    wire [63:0] prod;
    wire [31:0] quo, rem;
    generate if (MUL) begin : g_mul
        mul_mips_mc mul (.a(A), .b(B), .sgn(MulSigned), .p(prod));
    end else begin : g_nomul
        assign prod = 64'd0;
    end endgenerate
    generate if (DIV) begin : g_div
        div_mips_mc div (.clk(clk), .start(DivStart), .a(A), .b(B), .sgn(MulSigned), .done(DivDone), .q(quo), .r(rem));
    end else begin : g_nodiv
        assign quo = 32'd0; assign rem = 32'd0; assign DivDone = 1'b0;
    end endgenerate
    generate if (HILO) begin : g_hilo
        always @(posedge clk) begin
            if (HiWrite) HI <= (HiLoSrc == 2'd0) ? A : (HiLoSrc == 2'd1) ? prod[63:32] : rem;
            if (LoWrite) LO <= (HiLoSrc == 2'd0) ? A : (HiLoSrc == 2'd1) ? prod[31:0]  : quo;
        end
    end endgenerate
endmodule
