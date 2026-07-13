`timescale 1ns/1ps

// SamplerZ testbench — uses the real falcon_fp_fpu (not behavioral).
// Tests: single sample, pair mode, basic statistical distribution.

module tb_falconsign_samplerz_top;

    reg         clk;
    reg         rst_n;

    // SamplerZ command
    reg         cmd_valid;
    wire        cmd_ready;
    reg  [63:0] cmd_mu;
    reg  [63:0] cmd_sigma_inv;
    reg  [63:0] cmd_sigma_min;
    reg         cmd_pair_mode;

    wire        rsp_valid;
    reg         rsp_ready;
    wire [63:0] rsp_z0;
    wire [63:0] rsp_z1;
    wire        rsp_accept;
    wire [7:0]  rsp_status;

    wire        fpu_req_valid;
    wire        fpu_req_ready;
    wire [3:0]  fpu_req_op;
    wire [63:0] fpu_req_a;
    wire [63:0] fpu_req_b;
    wire [63:0] fpu_req_c;
    wire [1:0]  fpu_req_fmt;
    wire [2:0]  fpu_req_rm;
    wire [1:0]  fpu_req_fcvt_op;
    wire        fpu_rsp_valid;
    wire        fpu_rsp_ready;
    wire [63:0] fpu_rsp_result;
    wire [4:0]  fpu_rsp_flags;
    wire        fpu_busy;

    wire        rng_req;
    reg         rng_ack;
    reg  [255:0] rng_data;
    wire        busy;
    wire        done;
    wire        fail;

    // ─── DUT: SamplerZ ───
    falconsign_samplerz_top #(.RNG_DATA_W(256)) dut (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_mu(cmd_mu), .cmd_sigma_inv(cmd_sigma_inv),
        .cmd_sigma_min(cmd_sigma_min), .cmd_pair_mode(cmd_pair_mode),
        .rsp_valid(rsp_valid), .rsp_ready(rsp_ready),
        .rsp_z0(rsp_z0), .rsp_z1(rsp_z1),
        .rsp_accept(rsp_accept), .rsp_status(rsp_status),
        .fpu_req_valid(fpu_req_valid), .fpu_req_ready(fpu_req_ready),
        .fpu_req_op(fpu_req_op), .fpu_req_a(fpu_req_a),
        .fpu_req_b(fpu_req_b), .fpu_req_c(fpu_req_c),
        .fpu_req_fmt(fpu_req_fmt), .fpu_req_rm(fpu_req_rm), .fpu_req_fcvt_op(fpu_req_fcvt_op),
        .fpu_rsp_valid(fpu_rsp_valid), .fpu_rsp_ready(fpu_rsp_ready),
        .fpu_rsp_result(fpu_rsp_result),
        .rng_req(rng_req), .rng_ack(rng_ack),
        .rng_data(rng_data),
        .busy(busy), .done(done), .fail(fail)
    );

    // ─── Real FPU instance ───
    falcon_fp_fpu u_fpu (
        .clk(clk), .rst_n(rst_n),
        .req_valid(fpu_req_valid), .req_ready(fpu_req_ready),
        .req_op(fpu_req_op),
        .req_a(fpu_req_a), .req_b(fpu_req_b), .req_c(fpu_req_c),
        .req_fmt(2'b01), .req_rm(3'b000), .req_fcvt_op(2'b00),
        .rsp_valid(fpu_rsp_valid), .rsp_ready(fpu_rsp_ready),
        .rsp_result(fpu_rsp_result), .rsp_flags(fpu_rsp_flags),
        .busy(fpu_busy)
    );

    // ─── Clock ───
    initial begin
        clk   = 1'b0;
        rst_n = 1'b0;
        #20 rst_n = 1'b1;
    end
    always #5 clk = ~clk;  // 100 MHz


    // ─── RNG: PRNG-256 (LFSR-based, same as before) ───
    reg [31:0] rng_lfsr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rng_lfsr <= 32'hDEADBEEF;
            rng_ack  <= 1'b0;
            rng_data <= 256'd0;
        end else begin
            if (rng_req && !rng_ack) begin
                rng_ack <= 1'b1;
                rng_data <= {
                    rng_lfsr, rng_lfsr ^ 32'hAAAA5555,
                    rng_lfsr ^ 32'h5555AAAA, rng_lfsr ^ 32'hFFFF0000,
                    rng_lfsr ^ 32'h0000FFFF, rng_lfsr ^ 32'h12345678,
                    rng_lfsr ^ 32'h9ABCDEF0, rng_lfsr ^ 32'h0FEDCBA9
                };
                rng_lfsr <= {rng_lfsr[30:0], rng_lfsr[31] ^ rng_lfsr[21] ^ rng_lfsr[0]};
            end
            if (rng_ack && !rng_req)
                rng_ack <= 1'b0;
        end
    end

    // ─── Test sequence ───
    reg [31:0]  sample_count;
    reg [31:0]  accept_count;
    reg [31:0]  reject_count;
    reg [31:0]  cycle_cnt;
    reg [7:0]   test_phase;
    integer     i;
    integer     fd;

    initial begin
        clk        = 1'b0;
        rst_n      = 1'b0;
        cmd_valid  = 1'b0;
        cmd_mu     = 64'd0;
        cmd_sigma_inv = 64'd0;
        cmd_sigma_min = 64'd0;
        cmd_pair_mode = 1'b0;
        rsp_ready  = 1'b1;
        sample_count = 0;
        accept_count = 0;
        reject_count = 0;
        cycle_cnt    = 0;
        test_phase   = 0;

        fd = $fopen("samplerz_results.csv", "w");
        $fwrite(fd, "idx,z\n");

        #30 rst_n = 1'b1;
        #10;

        // ───── TEST 1: Single sample, mu=0, sigma_inv=1.0, sigma_min=1.0 ─────
        // Center=0, sigma'=1, should produce z ≈ 0 with high probability
        $display("=== TEST 1: Single sample (mu=0, sigma=1) ===");
        test_phase = 1;
        issue_sample(64'h0, 64'h3FF0000000000000, 64'h3FF0000000000000, 1'b0);
        wait_for_rsp();
        $display("  z0=%f accept=%b cycle=%0d", $bitstoreal(rsp_z0), rsp_accept, cycle_cnt);

        // ───── TEST 2: Single sample, mu=5.0 ─────
        // Center away from 0; will produce z ≈ 5 or nearby
        $display("=== TEST 2: Single sample (mu=5.0, sigma=1) ===");
        test_phase = 2;
        issue_sample($realtobits(5.0), 64'h3FF0000000000000, 64'h3FF0000000000000, 1'b0);
        wait_for_rsp();
        $display("  z0=%f accept=%b", $bitstoreal(rsp_z0), rsp_accept);

        $display("=== TEST 2B: Single sample (mu=-5.25, sigma=1) ===");
        issue_sample($realtobits(-5.25), 64'h3FF0000000000000, 64'h3FF0000000000000, 1'b0);
        wait_for_rsp();
        $display("  z0=%f accept=%b", $bitstoreal(rsp_z0), rsp_accept);
        if (($bitstoreal(rsp_z0) > -2.0) || ($bitstoreal(rsp_z0) < -9.0)) begin
            $display("TB_FAIL: negative mu sample is not centered near mu");
            $finish;
        end

        // ───── TEST 3: Pair mode, mu=2.5 ─────
        $display("=== TEST 3: Pair sample (mu=2.5, sigma=1) ===");
        test_phase = 3;
        issue_sample($realtobits(2.5), 64'h3FF0000000000000, 64'h3FF0000000000000, 1'b1);
        wait_for_rsp();
        $display("  z0=%f z1=%f accept=%b", $bitstoreal(rsp_z0), $bitstoreal(rsp_z1), rsp_accept);

        // ───── TEST 4: Single sample, sigma=5.0 (wider distribution) ─────
        $display("=== TEST 4: Wider sigma (mu=0, sigma=5) ===");
        test_phase = 4;
        issue_sample(64'h0, $realtobits(0.2), $realtobits(5.0), 1'b0);
        wait_for_rsp();
        $display("  z0=%f accept=%b", $bitstoreal(rsp_z0), rsp_accept);

        // ───── TEST 5: Collect many samples (statistical verification) ─────
        // Use sigma=5 (sigma_inv=0.2, sigma_min=5.0) for wider acceptance.
        $display("=== TEST 5: Bulk collect 500 samples (sigma=5) ===");
        test_phase = 5;
        sample_count = 0;
        accept_count = 0;
        reject_count = 0;
        cycle_cnt = 0;
        i = 0;
        while (i < 500 && cycle_cnt < 50000000) begin
            issue_sample($realtobits(0.0), $realtobits(0.2), $realtobits(5.0), 1'b0);
            wait_for_rsp();
            if (rsp_accept) begin
                accept_count = accept_count + 1;
                // Write accepted z value to CSV
                $fwrite(fd, "%0d,%0d,%f\n", accept_count, i, $bitstoreal(rsp_z0));
                i = i + 1;
            end else begin
                reject_count = reject_count + 1;
            end
        end
        sample_count = i;
        $display("  samples=%0d accepted=%0d rejected=%0d (cycle=%0d)",
            sample_count, accept_count, reject_count, cycle_cnt);

        $fclose(fd);

        // ───── Summary ─────
        $display("");
        if (fail)
            $display("TB_FAIL: SamplerZ reported fail");
        else if (reject_count > 400)
            $display("TB_FAIL: Too many rejections (%0d/500)", reject_count);
        else if (accept_count < 50)
            $display("TB_WARN: Very few accepts (%0d/500)", accept_count);
        else
            $display("TB_PASS: falconsign_samplerz_top  accept=%0d/500", accept_count);

        $finish;
    end

    // ─── Helper: issue sample command ───
    task issue_sample;
        input [63:0] mu;
        input [63:0] sigma_inv;
        input [63:0] sigma_min;
        input        pair_mode;
        begin
            @(posedge clk);
            while (!cmd_ready) @(posedge clk);
            cmd_valid     <= 1'b1;
            cmd_mu        <= mu;
            cmd_sigma_inv <= sigma_inv;
            cmd_sigma_min <= sigma_min;
            cmd_pair_mode <= pair_mode;
            @(posedge clk);
            cmd_valid <= 1'b0;
        end
    endtask

    // ─── Helper: wait for response ───
    task wait_for_rsp;
        begin
            while (!rsp_valid) begin
                @(posedge clk);
                if (cycle_cnt < 32'hFFFFFFF0) cycle_cnt <= cycle_cnt + 1;
            end
            @(posedge clk);  // sample response
        end
    endtask

    // ─── Watchdog ───
    initial begin
        #50000000;  // 5M cycles @ 10ns = 50ms
        $display("WATCHDOG TIMEOUT");
        $finish;
    end

    // ─── VCD dump ───
    initial begin
        $dumpfile("samplerz_dump.vcd");
        $dumpvars(0, tb_falconsign_samplerz_top);
    end

endmodule
