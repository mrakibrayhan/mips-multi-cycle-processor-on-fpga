//====================================================================
// ** Step E (debug unit alignment): S00 removed. FOUR RAMs R00 R16 D00 D16, sweep idx 0..63 (65 cycles), exactly
//    like the ARM / RISC-V / MIPS-SC boards. PC, IR, counters, halted/fault/started are read on the ISSP. **
// snapshot_mips_mc.v   (MIPS multi-cycle CPU, board view, step 14)  -  register / RAM / status snapshot for the
//                      In-System Memory Content Editor (same unit as the ARM boards, with a 5th RAM for the status)
//
//   Five 16-word RAMs (instance names shown in the Memory Content Editor):
//        R00 = r0 .. r15          R16 = r16 .. r31
//        D00 = RAM word  0 .. 15  (0x10000000 .. 0x1000003C)      D16 = RAM word 16 .. 31 (0x10000040 .. 0x1000007C)
//        S00 = status: 0 PC   1 IR (instruction)   2 HI   3 LO   4 cycle counter   5 instruction counter
//                      6 {29'b0, fault, halted, started}   7 FSM state   8 FEATURES   9..15 = 0
//   WHEN: the program halts or faults (trigger_auto) or the ISSP source bit is set (trigger_src, at any time).
//   The CPU is frozen at an instruction boundary (FSM held in FETCH) or is stopped: the core raises 'view_rdy', then the
//   unit sweeps idx 0..79 (0..31 registers and 32..63 RAM words through the core's view port; 64..79 status words from
//   the top). The register file / RAM are synchronous: the value of index i arrives one cycle later, so word (idx - 1)
//   is written in each cycle (81 cycles).
//   IDLE (CPU runs) -> WAIT (freeze, until view_rdy) -> SNAP (81 cycles) -> DONE (frozen while the trigger stays) -> IDLE.
//   RESET CLEAR: while rst = 1 the five RAMs are written with 0 (16 cycles; the button filter keeps rst high for ms).
//====================================================================
module snapshot_mips_mc (
    input         clk,           // cpu_clk
    input         rst,
    input         started,
    input         trigger_auto,  // halted | fault
    input         trigger_src,   // ISSP source bit (JTAG clock domain)
    input         view_rdy,
    input  [31:0] view_val,      // core dbg_val (belongs to the idx of the previous cycle)
    input  [31:0] extra_val,     // not used any more (S00 removed, 2026-10-29); kept so the board top is unchanged
    output        freeze,
    output        done,
    output [6:0]  idx,           // view index presented to the core
    output [6:0]  widx           // index of the word written in this cycle
);
    reg s1 = 1'b0, s2 = 1'b0;
    always @(posedge clk) begin
        s1 <= trigger_src;
        s2 <= s1;
    end
    wire trigger = trigger_auto | s2;

    localparam IDLE = 2'd0, WAIT = 2'd1, SNAP = 2'd2, DONE = 2'd3;
    reg [1:0] state = IDLE;
    reg [6:0] cnt   = 7'd0;
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            cnt   <= 7'd0;
        end else begin
            case (state)
                IDLE: if (started && trigger) state <= WAIT;
                WAIT: if (view_rdy) begin state <= SNAP; cnt <= 7'd0; end
                SNAP: if (cnt == 7'd64) state <= DONE; else cnt <= cnt + 7'd1;   // 4 RAMs: idx 0..63 (65 cycles)
                DONE: if (!trigger) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end
    assign freeze = (state != IDLE);
    assign done   = (state == DONE);
    assign idx    = cnt;
    assign widx   = cnt - 7'd1;
    wire   wr     = (state == SNAP) && (cnt != 7'd0);

    reg  [3:0]  clr = 4'd0;
    always @(posedge clk) clr <= rst ? clr + 4'd1 : 4'd0;

    wire [3:0]  word_in   = rst ? clr : widx[3:0];
    wire [31:0] snap_data = rst ? 32'd0 : view_val;            // S00 removed (status is on the ISSP)
    wire [2:0]  bank      = widx[6:4];             // 0 R00, 1 R16, 2 D00, 3 D16, 4 S00
    wire [31:0] q0, q1, q2, q3;

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R00"), .lpm_type ("altsyncram")
    ) snap_ram_R00 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && bank == 3'd0)), .q_a (q0));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R16"), .lpm_type ("altsyncram")
    ) snap_ram_R16 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && bank == 3'd1)), .q_a (q1));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D00"), .lpm_type ("altsyncram")
    ) snap_ram_D00 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && bank == 3'd2)), .q_a (q2));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D16"), .lpm_type ("altsyncram")
    ) snap_ram_D16 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && bank == 3'd3)), .q_a (q3));

endmodule
