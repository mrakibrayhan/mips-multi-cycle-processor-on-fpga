//====================================================================
// mem_mips_mc.sv  (MIPS multi-cycle CPU, step 3; SystemVerilog file because of the byte-enable RAM)
//                 [HH] unified memory with SYNCHRONOUS-read M9K blocks (mips_mc_spec.md sections 4 and 6)
//
//   Harris & Harris multicycle processor (section 7.4): ONE memory for instructions and data, address
//   Adr = IorD ? ALUOut : PC, read data -> IR (Fetch) and Data register (MemRead), write in MemWrite.
//   Here the memory map of the ARM cores:
//     ROM 0x00000000 : 2**IMEM_AW words, ONE M9K used as a DUAL-PORT ROM ($readmemh(INIT_FILE)):
//                      port A = instruction (Adr = PC), port B = data loads (Adr = ALUOut) from the ROM
//     RAM 0x10000000 : 2**DMEM_AW words, ONE M9K with byte enables ($readmemh(DATA_FILE))
//     other          : data reads 0, writes ignored.   Instructions are always read from the ROM.
//
//   SYNCHRONOUS READ = the M9K address registers hold what the [HH] registers hold:
//     port A    : the datapath drives ia_adr = PCEn ? PCNext : PC, i.e. the value the PC register takes at this edge.
//                 So 'instr' = mem[PC] in every cycle, exactly [HH]'s asynchronous read with Adr = PC.
//     data port : the datapath drives d_adr = IorD ? ALUOut : ALUResult, i.e. the value ALUOut takes at this edge (in
//                 MemAdr) or holds (MemRead / MemWrite). So 'd_rdata' = mem[ALUOut] in MemRead (-> Data register).
//     write     : address / store data / byte enables / we presented in MemWrite are latched at the end of MemWrite and
//                 written there: the same edge as [HH]'s synchronous write.
//   STORE: the data is copied onto every lane (sb 4 x byte, sh 2 x half) and the byte enables pick the lane(s).
//   d_size 0 word, 1 half, 2 byte. Little-endian. Word accesses ignore adr[1:0], halfword accesses ignore adr[0].
//   MEMX = 0 (ablation build without lb/lbu/lh/lhu/sb/sh): no aligner, word accesses only.
//   The RAM keeps its contents through a reset (KEY3); program the FPGA again to restore data.hex.
//====================================================================
module mem_mips_mc #(
    parameter IMEM_AW   = 7,                  // 128 words of ROM (board build)
    parameter DMEM_AW   = 5,                  // 32 words of RAM (board build)
    parameter MEMX      = 1,                  // byte / halfword accesses built
    parameter INIT_FILE = "program.hex",
    parameter DATA_FILE = "data.hex"
) (
    input         clk,
    // port A: instruction (Adr = PC)
    input  [31:0] ia_adr,                     // PCEn ? PCNext : PC
    output [31:0] instr,                      // mem[PC]
    // data port (Adr = ALUOut)
    input  [31:0] d_adr,                      // IorD ? ALUOut : ALUResult   (or the board-view address)
    input         d_we,                       // MemWrite
    input  [1:0]  d_size,                     // 0 word, 1 half, 2 byte
    input         d_sgn,                      // lb / lh
    input  [31:0] d_wdata,                    // B
    output [31:0] d_q,                        // raw word (board view)
    output [31:0] d_rdata                     // aligned / extended load data (-> Data register)
);
    // ---------------- ROM: one M9K, dual port ----------------
    (* romstyle = "M9K" *) reg [31:0] rom [0:(1 << IMEM_AW) - 1];
    initial $readmemh(INIT_FILE, rom);

    wire [IMEM_AW-1:0] ia_idx = ia_adr[IMEM_AW+1:2];
    wire [IMEM_AW-1:0] rb_idx = d_adr[IMEM_AW+1:2];
    reg  [31:0] rom_qa, rom_qb;                            // M9K read registers (no power-up value)
    always @(posedge clk) rom_qa <= rom[ia_idx];
    always @(posedge clk) rom_qb <= rom[rb_idx];
    assign instr = rom_qa;

    // ---------------- store lanes and byte enables ----------------
    wire sel_rom = (d_adr[31:28] == 4'h0);
    wire sel_ram = (d_adr[31:28] == 4'h1);
    wire [DMEM_AW-1:0] widx = d_adr[DMEM_AW+1:2];
    reg [31:0] sdata;
    reg [3:0]  be;
    always @(*) begin
        if (MEMX && d_size == 2'd2)      begin sdata = {4{d_wdata[7:0]}};  be = 4'b0001 << d_adr[1:0];        end   // sb
        else if (MEMX && d_size == 2'd1) begin sdata = {2{d_wdata[15:0]}}; be = d_adr[1] ? 4'b1100 : 4'b0011; end   // sh
        else                             begin sdata = d_wdata;            be = 4'b1111;                        end   // sw
    end
    wire wram = d_we & sel_ram;

    // ---------------- RAM: one 32-bit M9K with byte enables (Intel SystemVerilog byte-enable RAM template) ----------------
    (* ramstyle = "M9K, no_rw_check" *) logic [3:0][7:0] ram [0:(1 << DMEM_AW) - 1];
    initial $readmemh(DATA_FILE, ram);
    logic [31:0] ram_q;
    always @(posedge clk) begin
        if (wram) begin
            if (be[0]) ram[widx][0] <= sdata[7:0];
            if (be[1]) ram[widx][1] <= sdata[15:8];
            if (be[2]) ram[widx][2] <= sdata[23:16];
            if (be[3]) ram[widx][3] <= sdata[31:24];
        end
        ram_q <= ram[widx];
    end

    // ---------------- registered region / byte offset / size / sign (they belong to the word on the outputs) ----------------
    reg rom_r = 1'b0, ram_r = 1'b0, sgn_r = 1'b0;
    reg [1:0] a10_r = 2'd0, size_r = 2'd0;
    always @(posedge clk) begin
        rom_r  <= sel_rom;
        ram_r  <= sel_ram;
        a10_r  <= d_adr[1:0];
        size_r <= d_size;
        sgn_r  <= d_sgn;
    end
    assign d_q = ram_r ? ram_q : rom_r ? rom_qb : 32'd0;

    // ---------------- load aligner ----------------
    generate if (MEMX) begin : g_align
        reg [7:0] b;
        always @(*) begin
            case (a10_r)
                2'd0:    b = d_q[7:0];
                2'd1:    b = d_q[15:8];
                2'd2:    b = d_q[23:16];
                default: b = d_q[31:24];
            endcase
        end
        wire [15:0] h = a10_r[1] ? d_q[31:16] : d_q[15:0];
        assign d_rdata = (size_r == 2'd2) ? {{24{sgn_r & b[7]}}, b} :
                         (size_r == 2'd1) ? {{16{sgn_r & h[15]}}, h} : d_q;
    end else begin : g_word
        assign d_rdata = d_q;
    end endgenerate
endmodule
