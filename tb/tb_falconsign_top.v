`timescale 1ns/1ps

module tb_falconsign_top;

    reg         clk;
    reg         rst_n;

    // Bus interface
    reg         bus_cs;
    reg         bus_wr;
    reg  [15:0] bus_addr;
    reg  [31:0] bus_wdata;
    wire [31:0] bus_rdata;
    wire        bus_ready;
    wire        bus_irq;

    // Status
    wire        busy;
    wire        done;
    wire        fail;
    wire [7:0]  status;

    localparam integer C_INT_BANK0_IDX = 1408;

    falconsign_top #(.ADDR_W(13), .LEVEL_W(4), .INDEX_W(10)) dut (
        .clk(clk), .rst_n(rst_n),
        .bus_cs(bus_cs), .bus_wr(bus_wr),
        .bus_addr(bus_addr), .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata), .bus_ready(bus_ready),
        .bus_irq(bus_irq),
        .busy(busy), .done(done), .fail(fail), .status(status)
    );

    // ─── Clock & Reset ───
    initial begin
        clk   = 1'b0;
        rst_n = 1'b0;
        #30 rst_n = 1'b1;
    end
    always #5 clk = ~clk;

    // ─── Phase name helper ───
    function [127:0] phase_name;
        input [3:0] p;
        case (p)
            4'd0: phase_name = "SI_Idle";
            4'd1: phase_name = "SH_SeedHash";
            4'd2: phase_name = "HP_HashToPoint";
            4'd3: phase_name = "FC_FFT";
            4'd4: phase_name = "TG_TargetGen";
            4'd5: phase_name = "FS_ffSampling";
            4'd6: phase_name = "VD_BhatMul";
            4'd7: phase_name = "IV_IFFT";
            4'd8: phase_name = "FI_FprToInt";
            4'd9: phase_name = "N1_NTT";
            4'd10: phase_name = "RC_RejCheck";
            4'd11: phase_name = "CN_Compress";
            4'd12: phase_name = "EN_Encode";
            4'd13: phase_name = "OU_Output";
            4'd14: phase_name = "SD_SendDone";
            default: phase_name = "UNKNOWN";
        endcase
    endfunction

    // ─── Phase transition tracking ───
    reg [3:0]  prev_st;
    reg [31:0] phase_start_cycle;
    reg [31:0] total_cycle;
    reg [31:0] restart_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_st <= 0;
            phase_start_cycle <= 0;
            total_cycle <= 0;
            restart_cnt <= 0;
        end else begin
            total_cycle <= total_cycle + 1;

            if (dut.st != prev_st) begin
                // Report phase and elapsed cycles
                if (prev_st != 0 || dut.st != 0) begin
                    $display("[T=%0d cy=%0d] PHASE: %s → %s (prev took %0d cy)",
                        $time, total_cycle,
                        phase_name(prev_st), phase_name(dut.st),
                        total_cycle - phase_start_cycle);
                end
                prev_st <= dut.st;
                phase_start_cycle <= total_cycle;

                // Track restarts
                if (prev_st == 4'd7 && dut.st == 4'd1) begin // RC→SH = restart
                    restart_cnt <= restart_cnt + 1;
                    $display("[T=%0d cy=%0d] *** RESTART #%0d: RC rejected, retrying with new salt ***",
                        $time, total_cycle, restart_cnt + 1);
                end

                if (prev_st == 4'd7 && dut.st == 4'd8) begin // RC→N1 = accepted, start NTT
                    $display("[T=%0d cy=%0d] FprToInt complete: starting NTT",
                        $time, total_cycle);
                end

                if (prev_st == 4'd8 && dut.st == 4'd7) begin
                    $display("[T=%0d cy=%0d] NTT complete: starting full (s1,s2) norm check",
                        $time, total_cycle);
                end

                if (prev_st == 4'd7 && dut.st == 4'd9) begin
                    $display("[T=%0d cy=%0d] RC accepted: norm_sq=%0d status=0x%02h",
                        $time, total_cycle, dut.norm_sq, dut.norm_status);
                end

                if (prev_st == 4'd9 && dut.st == 4'd1) begin
                    restart_cnt <= restart_cnt + 1;
                    $display("[T=%0d cy=%0d] *** RESTART #%0d: RC rejected, retrying with new salt ***",
                        $time, total_cycle, restart_cnt + 1);
                end

                if (prev_st == 4'd8 && dut.st == 4'd9) begin
                    $display("[T=%0d cy=%0d] NTT complete: starting full (s1,s2) norm check",
                        $time, total_cycle);
                end

                if (prev_st == 4'd9 && dut.st == 4'd10) begin
                    $display("[T=%0d cy=%0d] RC accepted: norm_sq=%0d status=0x%02h",
                        $time, total_cycle, dut.norm_sq, dut.norm_status);
                end

                // Completion
                if (dut.st == 4'd12) begin
                    $display("[T=%0d cy=%0d] *** SIGNING COMPLETE: total_cycles=%0d restarts=%0d ***",
                        $time, total_cycle, total_cycle, restart_cnt);
                end
            end
        end
    end

    // ─── Test sequence ───
    reg [31:0] tb_cycle;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tb_cycle <= 32'd0;
            bus_cs    <= 1'b0;
            bus_wr    <= 1'b0;
            bus_addr  <= 16'd0;
            bus_wdata <= 32'd0;
        end else begin
            tb_cycle <= tb_cycle + 32'd1;
            bus_cs   <= 1'b0;
            bus_wr   <= 1'b0;

            case (tb_cycle)
                32'd10: begin
                    $display("[T=%0d] Writing START command", $time);
                    bus_cs    <= 1'b1;
                    bus_wr    <= 1'b1;
                    bus_addr  <= 16'h0000;
                    bus_wdata <= 32'h00000001;
                end

                32'd20: begin
                    $display("[T=%0d] Reading status reg", $time);
                    bus_cs   <= 1'b1;
                    bus_wr   <= 1'b0;
                    bus_addr <= 16'h0004;
                end

                default: begin
                    if (done) begin
                        $display("[T=%0d] TEST PASSED: done=1 fail=%b", $time, fail);
                        #100;
                        $finish;
                    end
                    if (fail) begin
                        $display("[T=%0d] TEST FAILED: fail=1 status=%0d", $time, status);
                        #100;
                        $finish;
                    end
                end
            endcase
        end
    end

    // ─── HP phase: monitor coefficient writes ───
    reg [9:0]  hp_coeff_total;
    reg [15:0] hp_coeff_log [0:31];
    reg        hp_done_flag;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hp_coeff_total <= 0;
            hp_done_flag <= 0;
        end else begin
            hp_done_flag <= 0;
            if (dut.st == 4'd2 && dut.sn == 4'd2) begin
                if (dut.htp_coeff_valid) begin
                    if (hp_coeff_total < 32)
                        hp_coeff_log[hp_coeff_total] <= dut.htp_coeff;
                    hp_coeff_total <= hp_coeff_total + 1;
                end
            end
            if (dut.st == 4'd2 && dut.sn != 4'd2) begin
                hp_done_flag <= 1;
            end
            if (hp_done_flag) begin
                $display("[T=%0d] HP: %0d coeffs produced", $time, hp_coeff_total);
                $display("  First 8: %0d %0d %0d %0d %0d %0d %0d %0d",
                    hp_coeff_log[0], hp_coeff_log[1], hp_coeff_log[2], hp_coeff_log[3],
                    hp_coeff_log[4], hp_coeff_log[5], hp_coeff_log[6], hp_coeff_log[7]);
                // HP pack check disabled — generate block hierarchy differs in iverilog
                $display("  HP pack check skipped (generate hierarchy)");
            end
            // Reset per signing attempt
            if (dut.st == 4'd1 && dut.sn == 4'd1) hp_coeff_total <= 0;
        end
    end

    // ─── Watchdog ───
    reg [31:0] wd_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) wd_cnt <= 0;
        else begin
            wd_cnt <= wd_cnt + 1;
            if (wd_cnt == 32'd20000000) begin
                $display("[T=%0d] WATCHDOG at phase=%s cycle=%0d", $time, phase_name(dut.st), total_cycle);
                $display("  st=%0d sn=%0d", dut.st, dut.sn);
                $display("  SH: word_idx=%0d absorb=%0d shake_ready=%0d", dut.sh_word_idx, dut.shake_absorb, dut.shake_ready);
                $display("  HP: coeff_cnt=%0d wr_addr=%0d htp_ready=%0d", dut.hp_coeff_cnt, dut.hp_wr_addr, dut.htp_ready);
                $display("  FE: state=%0d op=%0d", dut.u_fe.state, dut.u_fe.op_q);
                $display("  TS: run=%0d state=%0d level=%0d", dut.u_ts.run_state, dut.u_ts.state_q, dut.u_ts.level_q);
                $display("  SZ: busy=%0d done=%0d fail=%0d", dut.sz_busy, dut.sz_done, dut.sz_fail);
                $display("  NTT: op_state=%0d ntt_done=%0d", dut.u_ntt.op_state, dut.ntt_done);
                $display("  Salt: cnt=%0d rc_fail=%0d", dut.salt_cnt, dut.rc_fail);
                $finish;
            end
        end
    end

    // ─── Dump ───
    /*initial begin
        if ($test$plusargs("DUMP_VCD")) begin
            $dumpfile("tb_falconsign_top.vcd");
            $dumpvars(0, tb_falconsign_top);
        end
    end*/
    initial begin
        $dumpfile("tb_falconsign_top.vcd");
        $dumpvars(0, tb_falconsign_top);
    end

endmodule
