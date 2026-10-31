//====================================================================
// aludec_mips.v   (MIPS multi-cycle CPU, step 6)  -  [HH] ALU decoder (section 7.3.2 table 7.2 / 7.4.2), extended
//
//   ALUOp 00 -> ADD (PC + 4, branch target, memory address, jr / jalr target A + $0)
//   ALUOp 01 -> SUB (branches: A - B; Zero / A[31] give the condition)
//   ALUOp 10 -> by funct (R-type):  add/addu ADD  sub/subu SUB  and AND  or OR  xor XOR  nor NOR  slt SLT  sltu SLTU
//                                   sll/sllv SLL  srl/srlv SRL  sra/srav SRA   (ShVar = 1 for sllv / srlv / srav)
//   ALUOp 11 -> by opcode (I-type, [HH] ADDIExecute generalised): addi/addiu ADD  slti SLT  sltiu SLTU  andi AND
//                                   ori OR  xori XOR  lui ADD (rs = 0)
//   ALUControl codes: see alu_mips.v.
//====================================================================
module aludec_mips (
    input  [1:0] ALUOp,
    input  [5:0] funct,
    input  [5:0] op,
    output reg [3:0] ALUControl,
    output       ShVar
);
    reg [3:0] fctl, octl;
    always @(*) begin
        case (funct)
            6'h20, 6'h21: fctl = 4'b0010;
            6'h22, 6'h23: fctl = 4'b0110;
            6'h24:        fctl = 4'b0000;
            6'h25:        fctl = 4'b0001;
            6'h26:        fctl = 4'b0011;
            6'h27:        fctl = 4'b0100;
            6'h2A:        fctl = 4'b0111;
            6'h2B:        fctl = 4'b1000;
            6'h00, 6'h04: fctl = 4'b1001;
            6'h02, 6'h06: fctl = 4'b1010;
            6'h03, 6'h07: fctl = 4'b1011;
            default:      fctl = 4'b0010;
        endcase
        case (op)
            6'h0A:   octl = 4'b0111;
            6'h0B:   octl = 4'b1000;
            6'h0C:   octl = 4'b0000;
            6'h0D:   octl = 4'b0001;
            6'h0E:   octl = 4'b0011;
            default: octl = 4'b0010;            // addi addiu lui
        endcase
        case (ALUOp)
            2'b00:   ALUControl = 4'b0010;
            2'b01:   ALUControl = 4'b0110;
            2'b10:   ALUControl = fctl;
            default: ALUControl = octl;
        endcase
    end
    assign ShVar = (ALUOp == 2'b10) & (funct[5:3] == 3'b000) & funct[2];
endmodule
