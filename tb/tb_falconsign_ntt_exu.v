`timescale 1ns/1ps
// Standalone testbench for falconsign_ntt_exu.
module tb_falconsign_ntt_exu;

    localparam ADDR_W = 11;
    localparam N_WORDS = 32;

    reg clk, rst_n;
    reg start;
    wire start_ready, done, fail;
    wire [7:0] status;

    reg [ADDR_W-1:0] cfg_h_base, cfg_h_work_base;
    reg [ADDR_W-1:0] cfg_s2_base, cfg_s2_work_base;
    reg [ADDR_W-1:0] cfg_c_base, cfg_dst_base;

    // Port B memory model
    wire        mem_rd_en;
    wire [ADDR_W-1:0] mem_rd_addr;
    wire        mem_wr_en;
    wire [ADDR_W-1:0] mem_wr_addr;
    wire [255:0] mem_wr_data;

    reg [255:0] mem_h    [0:N_WORDS-1];
    reg [255:0] mem_s2   [0:N_WORDS-1];
    reg [255:0] mem_c    [0:N_WORDS-1];
    reg [255:0] mem_dst  [0:N_WORDS-1];
    reg [255:0] mem_h_work  [0:N_WORDS-1];
    reg [255:0] mem_s2_work [0:N_WORDS-1];
    reg [255:0] golden   [0:N_WORDS-1];

    wire [ADDR_W-1:0] rd_off;
    assign rd_off = mem_rd_addr;
    reg  [255:0] mem_rd_data_comb;
    always @(*) begin
        mem_rd_data_comb = (rd_off < 32)  ? mem_h[rd_off]  :
                           (rd_off < 64)  ? mem_s2[rd_off-32] :
                           (rd_off < 96)  ? mem_c[rd_off-64]  :
                           (rd_off < 128) ? mem_dst[rd_off-96] :
                           (rd_off < 160) ? mem_h_work[rd_off-128] :
                           (rd_off < 192) ? mem_s2_work[rd_off-160] : 256'd0;
    end
    // Combinational read (Port B with 0-cycle latency for simplicity)
    wire [255:0] mem_rd_data;
    assign mem_rd_data = mem_rd_data_comb;

    // ROMs
    wire [8:0]  ntt_tw_rom_addr;
    wire [13:0] ntt_tw_rom_data;
    wire [9:0]  ntt_psi_rom_addr;
    wire [13:0] ntt_psi_rom_data;

    falconsign_ntt_twiddle_rom #(.ADDR_W(9)) u_tw (
        .clk(clk), .addr(ntt_tw_rom_addr), .data(ntt_tw_rom_data));
    falconsign_ntt_psi_rom #(.ADDR_W(10)) u_psi (
        .clk(clk), .addr(ntt_psi_rom_addr), .data(ntt_psi_rom_data));

    // DUT
    falconsign_ntt_exu #(.ADDR_W(ADDR_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .start_ready(start_ready),
        .done(done), .fail(fail), .status(status),
        .cfg_h_base(cfg_h_base), .cfg_h_work_base(cfg_h_work_base),
        .cfg_s2_base(cfg_s2_base), .cfg_s2_work_base(cfg_s2_work_base),
        .cfg_c_base(cfg_c_base), .cfg_dst_base(cfg_dst_base),
        .mem_rd_en(mem_rd_en), .mem_rd_addr(mem_rd_addr),
        .mem_rd_data(mem_rd_data),
        .mem_wr_en(mem_wr_en), .mem_wr_addr(mem_wr_addr),
        .mem_wr_data(mem_wr_data),
        .twiddle_rom_addr(ntt_tw_rom_addr),
        .twiddle_rom_data(ntt_tw_rom_data),
        .psi_rom_addr(ntt_psi_rom_addr),
        .psi_rom_data(ntt_psi_rom_data)
    );

    // Memory write capture (posedge, same as real Port B)
    always @(posedge clk) begin
        if (mem_wr_en) begin
            if (mem_wr_addr < 32)       mem_h[mem_wr_addr]    <= mem_wr_data;
            if (mem_wr_addr >= 32 && mem_wr_addr < 64)  mem_s2[mem_wr_addr-32]  <= mem_wr_data;
            if (mem_wr_addr >= 64 && mem_wr_addr < 96)  mem_c[mem_wr_addr-64]   <= mem_wr_data;
            if (mem_wr_addr >= 96 && mem_wr_addr < 128) mem_dst[mem_wr_addr-96] <= mem_wr_data;
            if (mem_wr_addr >= 128 && mem_wr_addr < 160) mem_h_work[mem_wr_addr-128] <= mem_wr_data;
            if (mem_wr_addr >= 160 && mem_wr_addr < 192) mem_s2_work[mem_wr_addr-160] <= mem_wr_data;
        end
    end

    initial clk = 0;
    always #5 clk = ~clk;

    reg [31:0] errors;
    integer ei, li;
    reg [13:0] got, exp;

    initial begin
        rst_n = 0; start = 0;
        cfg_h_base  = 0;
        cfg_s2_base = 32;
        cfg_c_base  = 64;
        cfg_dst_base= 96;
        cfg_h_work_base = 128;
        cfg_s2_work_base = 160;
        errors = 0;

        $readmemh("DOC/ntt_test_h.hex", mem_h);
        $readmemh("DOC/ntt_test_s2.hex", mem_s2);
        $readmemh("DOC/ntt_test_c.hex", mem_c);
        $readmemh("DOC/ntt_test_s1_golden.hex", golden);

        #30 rst_n = 1;
        #20;
        if (start_ready) begin
            start <= 1;
            $display("[%0t] NTT EXU START", $time);
        end
        #10 start <= 0;

        wait(done || fail);
        if (fail) begin
            $display("[%0t] FAIL: status=0x%02h", $time, status);
        end else begin
            $display("[%0t] DONE", $time);
        end

        // Compare
        for (ei = 0; ei < 32; ei = ei + 1) begin
            for (li = 0; li < 16; li = li + 1) begin
                got = mem_dst[ei][li*16 +: 14];
                exp = golden[ei][li*16 +: 14];
                if (got !== exp) begin
                    errors = errors + 1;
                    if (errors <= 5)
                        $display("  MISMATCH w=%0d lane=%0d coeff=%0d: got=%0d exp=%0d",
                            ei, li, ei*16+li, got, exp);
                end
            end
        end

        if (errors == 0)
            $display("[%0t] ALL TESTS PASSED", $time);
        else
            $display("[%0t] FAILED: %0d mismatches", $time, errors);
        $finish;
    end

    reg [31:0] dbg_wr_cnt;
    reg [31:0] dbg_cyc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) dbg_cyc <= 0;
        else dbg_cyc <= dbg_cyc + 1;
    end
    initial dbg_wr_cnt = 0;
    always @(posedge clk) begin
        // Progress indicator every 5000 cycles
        if (dbg_cyc % 5000 == 1) begin
            $display("[cy=%0d] op=%0d ls=%0d bs=%0d bt=%0d nat=%0d",
                dbg_cyc, dut.op_state, dut.ls, dut.bs,
                dut.bt, dut.bt_nat_idx);
        end
        // Monitor writes — separate dst area tracking
        if (mem_wr_en && mem_wr_addr == 96 && dbg_wr_cnt < 5) begin
            $display("[cy=%0d] DST WR op=%0d ls=%0d lane[0..3]=%0d %0d %0d %0d",
                dbg_cyc, dut.op_state, dut.ls,
                mem_wr_data[0*16 +: 14], mem_wr_data[1*16 +: 14],
                mem_wr_data[2*16 +: 14], mem_wr_data[3*16 +: 14]);
            dbg_wr_cnt <= dbg_wr_cnt + 1;
        end
    end

    // Watchdog
    initial begin
        #50000000;
        $display("[%0t] WATCHDOG", $time);
        $finish;
    end

    initial begin
        /*if ($test$plusargs("DUMP_VCD")) begin
            $dumpfile("tb_falconsign_ntt_exu.vcd");
            $dumpvars(0, tb_falconsign_ntt_exu);
        end*/
        $dumpfile("tb_falconsign_ntt_exu.vcd");
        $dumpvars(0, tb_falconsign_ntt_exu);
    end

endmodule
