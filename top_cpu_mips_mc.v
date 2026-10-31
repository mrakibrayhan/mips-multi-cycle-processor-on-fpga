//====================================================================
// top_cpu_mips_mc.v  (board top-level - RZ-EasyFPGA A2.2 / Cyclone IV EP4CE6)
//
//   MIPS-compatible MULTI-CYCLE CPU (Harris & Harris 2nd ed. section 7.4) with SYNCHRONOUS-read memories in M9K blocks:
//   dual-port ROM (1 M9K), byte-enable RAM (1 M9K), register file (2 M9K copies). Same pins, buttons, LEDs and
//   procedure as the ARM boards.
//
//   Buttons: KEY[3] = reset (the CPU is also held in reset until the first press of KEY[2]); KEY[2] = run.
//            The reset must last >= 32 CPU cycles (register-file clear); the button filter makes it milliseconds long.
//   LEDG (active low = lit): [0] fault   [1] started   [2] ALU Zero   [3] halted.
//
//   SNAPSHOT (snapshot_mips_mc.v): on halt / fault, or when the ISSP source bit is 1, the CPU is held at an instruction
//   boundary and r0..r31, the 32 RAM words and the status words are copied into five RAMs read with the
//   In-System Memory Content Editor:
//        R00 = r0..r15   R16 = r16..r31   D00 = RAM 0..15   D16 = RAM 16..31
//        S00 = 0 PC   1 IR   2 HI   3 LO   4 cycle counter   5 instruction counter   6 {fault, halted, started}
//              7 FSM state (20 = HALT, 21 = FAULT, 0 = FETCH after a manual snapshot)   8 FEATURES
//        PC after a stop: syscall / break / fault -> address + 4, "b ." -> its own address.
//   ISSP (altsource_probe, instance "MIPS"): source[0] = manual snapshot request;
//     probe (121 bits, LSB first):
//         [0] started  [1] halted  [2] fault  [3] snapshot done
//         [8:4]     FSM state
//         [24:9]    PC (low 16 bits)
//         [56:25]   IR (instruction register)
//         [88:57]   CYCLE COUNTER        CPU clock cycles while running (cycles held for a snapshot are not counted)
//         [120:89]  INSTRUCTION COUNTER  retired instructions (the stopping instruction included)
//       CPI = cycle counter / instruction counter = 'cycles' / 'instrs' of mips_iss.py.
//   RAM starting values come from data.hex (32 lines, line 1 = word 0 at 0x10000000), loaded when the FPGA is programmed.
//   KEY3 does not reload the RAM: program the FPGA again to restore the data.hex values.
//====================================================================
module top_cpu_mips_mc #(
    parameter [6:0] FEATURES  = 7'h7F,
    parameter       IMEM_AW   = 7,               // 128 instruction words (1 M9K)
    parameter       DMEM_AW   = 5,               // 32 data words (1 M9K with byte enables)
    parameter       INIT_FILE = "program.hex",
    parameter       DATA_FILE = "data.hex",
    parameter       DEBOUNCE  = 250000           // button filter length in clock cycles (the testbench makes it short)
) (
    input        clk,      // PIN_23
    input  [4:0] KEY,      // PIN_88,89,90,91,25 (only KEY[3], KEY[2] used)
    output [7:0] SEG,      // PIN_128,121,125,129,132,126,124,127
    output [3:0] DIG,      // PIN_133,135,136,137
    output [3:0] LEDG      // PIN_87,86,85,84
);
    //----------------------------------------------------------------
    // Clock divider: 50 MHz / 4 = 12.5 MHz for the CPU core
    //----------------------------------------------------------------
    reg [1:0] clkdiv = 2'b00;
    always @(posedge clk) clkdiv <= clkdiv + 2'b01;
    wire cpu_clk = clkdiv[1];

    //----------------------------------------------------------------
    // Buttons: KEY[3] reset (level), KEY[2] run (one pulse)
    //----------------------------------------------------------------
    wire key3_raw = ~KEY[3];
    wire rst_debounced;
    debounce #(.DEBOUNCE_LIMIT(DEBOUNCE)) db_rst (.clk(clk), .btn_in(key3_raw), .btn_out(rst_debounced));

    wire key2_raw = ~KEY[2];
    wire key2_debounced, step_pulse;
    debounce #(.DEBOUNCE_LIMIT(DEBOUNCE)) db_step (.clk(cpu_clk), .btn_in(key2_raw), .btn_out(key2_debounced));
    edge_detector ed_step (.clk(cpu_clk), .signal_in(key2_debounced), .pulse_out(step_pulse));

    reg started = 1'b0;
    always @(posedge cpu_clk) begin
        if (rst_debounced)   started <= 1'b0;
        else if (step_pulse) started <= 1'b1;
    end

    //----------------------------------------------------------------
    // CPU (held in reset until 'started')
    //----------------------------------------------------------------
    wire        snap_req, frz_req, view_rdy, snap_done, retire, halted, fault, zero;
    wire [6:0]  snap_idx, snap_widx;
    wire [31:0] pc, instr, hi, lo, view_val;
    wire [4:0]  fsm;
    wire        cpu_rst = rst_debounced | ~started;

    cpu_core_mips_mc #(.FEATURES(FEATURES), .IMEM_AW(IMEM_AW), .DMEM_AW(DMEM_AW), .INIT_FILE(INIT_FILE),
                       .DATA_FILE(DATA_FILE), .DBG_EN(1)) mycpu (
        .clk(cpu_clk), .rst(cpu_rst), .halted(halted), .fault(fault), .retire(retire), .pc(pc), .instr(instr), .hi(hi),
        .lo(lo), .state(fsm), .zero(zero), .freeze(frz_req), .view_rdy(view_rdy), .dbg_idx(snap_idx[5:0]),
        .dbg_val(view_val), .tr_regw(), .tr_wa(), .tr_wd(), .tr_memw(), .tr_adr(), .tr_mdata(), .tr_msize(), .tr_hiw(),
        .tr_low());

    //----------------------------------------------------------------
    // Counters (cpu_clk): count while the CPU runs (not held for a snapshot), stop at the halt / fault.
    //----------------------------------------------------------------
    reg [31:0] cycle_count = 32'd0, instr_count = 32'd0;
    wire held    = view_rdy & ~halted & ~fault;
    wire running = started & ~halted & ~fault & ~held;
    always @(posedge cpu_clk) begin
        if (cpu_rst) begin
            cycle_count <= 32'd0; instr_count <= 32'd0;
        end else if (running) begin
            cycle_count <= cycle_count + 32'd1;
            if (retire) instr_count <= instr_count + 32'd1;
        end
    end

    //----------------------------------------------------------------
    // Snapshot unit
    //----------------------------------------------------------------
    reg [31:0] extra_val;
    always @(*) begin
        case (snap_widx[3:0])
            4'd0:    extra_val = pc;
            4'd1:    extra_val = instr;
            4'd2:    extra_val = hi;
            4'd3:    extra_val = lo;
            4'd4:    extra_val = cycle_count;
            4'd5:    extra_val = instr_count;
            4'd6:    extra_val = {29'd0, fault, halted, started};
            4'd7:    extra_val = {27'd0, fsm};
            4'd8:    extra_val = {25'd0, FEATURES};
            default: extra_val = 32'd0;
        endcase
    end
    snapshot_mips_mc snap (.clk(cpu_clk), .rst(cpu_rst), .started(started), .trigger_auto(halted | fault),
                           .trigger_src(snap_req), .view_rdy(view_rdy), .view_val(view_val), .extra_val(extra_val),
                           .freeze(frz_req), .done(snap_done), .idx(snap_idx), .widx(snap_widx));

    //----------------------------------------------------------------
    // 7-seg display unused, LEDs (active low)
    //----------------------------------------------------------------
    assign SEG = 8'hFF;
    assign DIG = 4'hF;
    assign LEDG[0] = ~fault;
    assign LEDG[1] = ~started;
    assign LEDG[2] = ~zero;
    assign LEDG[3] = ~halted;

    //----------------------------------------------------------------
    // ISSP: source = manual snapshot request, probe = status + counters
    //----------------------------------------------------------------
    wire [120:0] probe_status = {instr_count, cycle_count, instr, pc[15:0], fsm, snap_done, fault, halted, started};
    altsource_probe #(
        .lpm_type                ("altsource_probe"),
        .lpm_hint                ("UNUSED"),
        .sld_auto_instance_index ("YES"),
        .sld_instance_index      (0),
        .SLD_NODE_INFO           (4746752),
        .sld_ir_width            (4),
        .instance_id             ("MIPS"),
        .probe_width             (121),
        .source_width            (1),
        .source_initial_value    ("0"),
        .enable_metastability    ("NO")
    ) inst_issp (
        .probe      (probe_status),
        .source     (snap_req),
        .source_clk (cpu_clk),
        .source_ena (1'b1)
    );
endmodule
