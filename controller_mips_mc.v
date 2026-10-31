//====================================================================
// controller_mips_mc.v   (MIPS multi-cycle CPU, step 6)  -  [HH] multicycle control unit (section 7.4.2):
//                        main FSM (Moore) + ALU decoder (aludec_mips.v) + instruction decoder (decoder_mips.v)
//
//   States 0..11 = [HH] fig. 7.42 (FETCH, DECODE, MEMADR, MEMREAD, MEMWB, MEMWRITE, EXECUTE, ALUWB, BRANCH,
//   ADDIEXECUTE -> IEXECUTE, ADDIWB -> IWB, JUMP). States 12..21 = extension states (mips_mc_spec.md section 7):
//   LINK, JLWB, JR, MULT, MTHL, MFWB, DIV, DIVLOOP, HALT, FAULT.
//   PCEn = PCWrite | (Branch & BrCond)   ([HH]: PCWrite | Branch & Zero).
//
//   Stops (section 8): DECODE -> HALT for syscall / break; DECODE -> FAULT for an illegal word or a jr / jalr target with
//   A[1:0] != 0; BRANCH -> HALT for a taken branch with offset -1 ("b .", the PC is written with its own address).
//   Board view: 'freeze' holds the FSM in FETCH (no IRWrite / PCWrite) = an instruction boundary; view_rdy = frozen in
//   FETCH, or halted / fault. 'retire' = 1 in the last cycle of every instruction (incl. the stopping one).
//====================================================================
module controller_mips_mc #(
    parameter [6:0] FEATURES = 7'h7F
) (
    input         clk,
    input         rst,
    input  [31:0] Instr,
    input         BrCond,
    input  [1:0]  Alow,         // A[1:0] (jr / jalr alignment)
    input         DivDone,
    input         freeze,
    // datapath control
    output reg    IorD,
    output reg    IRWrite,
    output        PCEn,
    output reg    RegWrite,
    output reg [1:0] RegDst,
    output reg [1:0] MemtoReg,
    output reg    MemWrite,
    output reg    ALUSrcA,
    output reg [1:0] ALUSrcB,
    output [3:0]  ALUControl,
    output        ShVar,
    output [1:0]  ImmSrc,
    output reg [1:0] PCSrc,
    output [2:0]  BrType,
    output reg    HiWrite,
    output reg    LoWrite,
    output reg [1:0] HiLoSrc,
    output reg    DivStart,
    output        MulSigned,
    output [1:0]  MemSize,
    output        MemSgn,
    // status
    output        halted,
    output        fault,
    output        retire,
    output        view_rdy,
    output reg [4:0] state
);
    localparam FETCH = 5'd0, DECODE = 5'd1, MEMADR = 5'd2, MEMREAD = 5'd3, MEMWB = 5'd4, MEMWRITE = 5'd5,
               EXECUTE = 5'd6, ALUWB = 5'd7, BRANCH = 5'd8, IEXECUTE = 5'd9, IWB = 5'd10, JUMP = 5'd11,
               LINK = 5'd12, JLWB = 5'd13, JR = 5'd14, MULT = 5'd15, MTHL = 5'd16, MFWB = 5'd17, DIV = 5'd18,
               DIVLOOP = 5'd19, HALT = 5'd20, FAULT = 5'd21;

    wire legal, c_rtype, c_itype, c_load, c_store, c_branch, c_j, c_jal, c_jalr, c_jr, c_mult, c_mthl, c_mf, c_div, c_stop;
    wire HiSel;
    decoder_mips #(.FEATURES(FEATURES)) dec (
        .instr(Instr), .legal(legal), .c_rtype(c_rtype), .c_itype(c_itype), .c_load(c_load), .c_store(c_store),
        .c_branch(c_branch), .c_j(c_j), .c_jal(c_jal), .c_jalr(c_jalr), .c_jr(c_jr), .c_mult(c_mult), .c_mthl(c_mthl),
        .c_mf(c_mf), .c_div(c_div), .c_stop(c_stop), .BrType(BrType), .ImmSrc(ImmSrc), .MemSize(MemSize), .MemSgn(MemSgn),
        .HiSel(HiSel), .MulSigned(MulSigned));

    reg [1:0] ALUOp;
    aludec_mips ad (.ALUOp(ALUOp), .funct(Instr[5:0]), .op(Instr[31:26]), .ALUControl(ALUControl), .ShVar(ShVar));

    // ---------------- next state ----------------
    reg [4:0] next;
    wire selfloop = (Instr[15:0] == 16'hFFFF);
    always @(*) begin
        case (state)
            FETCH:    next = freeze ? FETCH : DECODE;
            DECODE:   if (!legal)                          next = FAULT;
                      else if (c_stop)                     next = HALT;
                      else if (c_load | c_store)           next = MEMADR;
                      else if (c_rtype)                    next = EXECUTE;
                      else if (c_itype)                    next = IEXECUTE;
                      else if (c_branch)                   next = BRANCH;
                      else if (c_j)                        next = JUMP;
                      else if ((c_jr | c_jalr) && Alow != 2'b00) next = FAULT;
                      else if (c_jal | c_jalr)             next = LINK;
                      else if (c_jr)                       next = JR;
                      else if (c_mult)                     next = MULT;
                      else if (c_mthl)                     next = MTHL;
                      else if (c_mf)                       next = MFWB;
                      else                                 next = DIV;
            MEMADR:   next = c_load ? MEMREAD : MEMWRITE;
            MEMREAD:  next = MEMWB;
            EXECUTE:  next = ALUWB;
            IEXECUTE: next = IWB;
            BRANCH:   next = (BrCond & selfloop) ? HALT : FETCH;
            LINK:     next = JLWB;
            MULT:     next = (Instr[31:26] == 6'h1C) ? MFWB : FETCH;   // MUL (SPECIAL2): {HI,LO} = rs x rt, then rd = LO (Step E)
            DIV:      next = DIVLOOP;
            DIVLOOP:  next = DivDone ? FETCH : DIVLOOP;
            HALT:     next = HALT;
            FAULT:    next = FAULT;
            default:  next = FETCH;      // MEMWB MEMWRITE ALUWB IWB JUMP JLWB JR MTHL MFWB
        endcase
    end
    initial state = FETCH;
    always @(posedge clk) state <= rst ? FETCH : next;

    // ---------------- outputs (Moore; JLWB / MTHL / MFWB use the IR, which is constant during the instruction) -------------
    reg PCWrite, Branch;
    always @(*) begin
        IorD = 1'b0; IRWrite = 1'b0; PCWrite = 1'b0; Branch = 1'b0; RegWrite = 1'b0; RegDst = 2'b00; MemtoReg = 2'b00;
        MemWrite = 1'b0; ALUSrcA = 1'b0; ALUSrcB = 2'b00; ALUOp = 2'b00; PCSrc = 2'b00; HiWrite = 1'b0; LoWrite = 1'b0;
        HiLoSrc = 2'd0; DivStart = 1'b0;
        case (state)
            FETCH:    begin ALUSrcB = 2'b01; IRWrite = ~freeze; PCWrite = ~freeze; end
            DECODE:   begin ALUSrcB = 2'b11; end
            MEMADR:   begin ALUSrcA = 1'b1; ALUSrcB = 2'b10; end
            MEMREAD:  begin IorD = 1'b1; end
            MEMWB:    begin RegDst = 2'b00; MemtoReg = 2'b01; RegWrite = 1'b1; end
            MEMWRITE: begin IorD = 1'b1; MemWrite = 1'b1; end
            EXECUTE:  begin ALUSrcA = 1'b1; ALUSrcB = 2'b00; ALUOp = 2'b10; end
            ALUWB:    begin RegDst = 2'b01; MemtoReg = 2'b00; RegWrite = 1'b1; end
            BRANCH:   begin ALUSrcA = 1'b1; ALUSrcB = 2'b00; ALUOp = 2'b01; PCSrc = 2'b01; Branch = 1'b1; end
            IEXECUTE: begin ALUSrcA = 1'b1; ALUSrcB = 2'b10; ALUOp = 2'b11; end
            IWB:      begin RegDst = 2'b00; MemtoReg = 2'b00; RegWrite = 1'b1; end
            JUMP:     begin PCSrc = 2'b10; PCWrite = 1'b1; end
            LINK:     begin ALUSrcA = 1'b0; ALUSrcB = 2'b01; end
            JLWB:     begin PCWrite = 1'b1; RegWrite = 1'b1; MemtoReg = 2'b00;
                            if (c_jal) begin PCSrc = 2'b10; RegDst = 2'b10; end
                            else       begin PCSrc = 2'b00; ALUSrcA = 1'b1; ALUSrcB = 2'b00; RegDst = 2'b01; end
                      end
            JR:       begin ALUSrcA = 1'b1; ALUSrcB = 2'b00; PCSrc = 2'b00; PCWrite = 1'b1; end
            MULT:     begin HiWrite = 1'b1; LoWrite = 1'b1; HiLoSrc = 2'd1; end
            MTHL:     begin HiWrite = HiSel; LoWrite = ~HiSel; HiLoSrc = 2'd0; end
            MFWB:     begin RegDst = 2'b01; MemtoReg = HiSel ? 2'b10 : 2'b11; RegWrite = 1'b1; end
            DIV:      begin DivStart = 1'b1; end
            DIVLOOP:  begin HiWrite = DivDone; LoWrite = DivDone; HiLoSrc = 2'd2; end
            default:  ;
        endcase
    end
    assign PCEn     = PCWrite | (Branch & BrCond);
    assign halted   = (state == HALT);
    assign fault    = (state == FAULT);
    assign retire   = (state != FETCH) & (state != HALT) & (state != FAULT) & ((next == FETCH) | (next == HALT) | (next == FAULT));
    assign view_rdy = ((state == FETCH) & freeze) | halted | fault;
endmodule
