`timescale 1ns/1ps

module tb_falconsign_hash_to_point_official_kat;
    localparam N          = 512;
    localparam WORD_COUNT = 10;
    localparam [2:0] LAST_BYTES = 3'd1;

    reg clk;
    reg rst_n;

    reg         shake_start;
    wire        shake_ready;
    reg         shake_absorb;
    reg  [63:0] shake_din;
    reg         shake_din_last;
    reg  [2:0]  shake_din_last_bytes;
    wire        shake_dout_valid;
    wire [63:0] shake_dout;
    wire        shake_fifo_wr_ready;
    wire        shake_fifo_rd_valid;
    wire [63:0] shake_fifo_rd_data;

    reg         htp_start;
    wire        htp_ready;
    wire        htp_hash_ready;
    wire [15:0] htp_coeff;
    wire        htp_coeff_valid;

    reg [63:0] kat_words [0:WORD_COUNT-1];
    reg [15:0] expected  [0:N-1];

    integer word_idx;
    integer coeff_idx;
    integer errors;
    reg     feed_enable;
    reg     htp_started;

    falconsign_shake256 u_shake (
        .clk(clk),
        .rst_n(rst_n),
        .start(shake_start),
        .ready(shake_ready),
        .absorb(shake_absorb),
        .din(shake_din),
        .din_last(shake_din_last),
        .din_last_bytes(shake_din_last_bytes),
        .dout_ready(shake_fifo_wr_ready),
        .dout_valid(shake_dout_valid),
        .dout(shake_dout)
    );

    falconsign_word_fifo #(.WIDTH(64), .DEPTH(32), .ADDR_W(5)) u_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .wr_valid(shake_dout_valid),
        .wr_ready(shake_fifo_wr_ready),
        .wr_data(shake_dout),
        .rd_valid(shake_fifo_rd_valid),
        .rd_ready(htp_hash_ready),
        .rd_data(shake_fifo_rd_data)
    );

    falconsign_hash_to_point #(.N(N)) u_htp (
        .clk(clk),
        .rst_n(rst_n),
        .start(htp_start),
        .ready(htp_ready),
        .hash_word(shake_fifo_rd_data),
        .hash_valid(shake_fifo_rd_valid),
        .hash_ready(htp_hash_ready),
        .coeff(htp_coeff),
        .coeff_valid(htp_coeff_valid)
    );

    always #5 clk = ~clk;

    initial begin
        $readmemh("SRC/tb/falcon512_kat0_htp_words.hex", kat_words);
        $readmemh("SRC/tb/falcon512_kat0_htp_expected.hex", expected);

        // Dump VCD for waveform viewing
        $dumpfile("tb_falconsign_hash_to_point_official_kat.vcd");
        $dumpvars(0, tb_falconsign_hash_to_point_official_kat);

        clk = 1'b0;
        rst_n = 1'b0;
        shake_start = 1'b0;
        shake_absorb = 1'b0;
        shake_din = 64'd0;
        shake_din_last = 1'b0;
        shake_din_last_bytes = 3'd0;
        htp_start = 1'b0;
        word_idx = 0;
        coeff_idx = 0;
        errors = 0;
        feed_enable = 1'b0;
        htp_started = 1'b0;

        #30 rst_n = 1'b1;
        @(posedge clk);
        shake_start <= 1'b1;
        @(posedge clk);
        shake_start <= 1'b0;
        feed_enable <= 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            word_idx <= 0;
            coeff_idx <= 0;
            shake_absorb <= 1'b0;
            shake_din <= 64'd0;
            shake_din_last <= 1'b0;
            shake_din_last_bytes <= 3'd0;
            htp_start <= 1'b0;
            feed_enable <= 1'b0;
            htp_started <= 1'b0;
        end else begin
            shake_absorb <= 1'b0;
            shake_din_last <= 1'b0;
            shake_din_last_bytes <= 3'd0;
            htp_start <= 1'b0;

            if (feed_enable && (word_idx < WORD_COUNT) && shake_ready) begin
                shake_absorb <= 1'b1;
                shake_din <= kat_words[word_idx];
                shake_din_last <= (word_idx == WORD_COUNT-1);
                shake_din_last_bytes <= (word_idx == WORD_COUNT-1) ? LAST_BYTES : 3'd0;
                if (word_idx == WORD_COUNT-1)
                    feed_enable <= 1'b0;
                word_idx <= word_idx + 1;
            end

            if (!feed_enable && (word_idx == WORD_COUNT) && shake_ready && htp_ready && !htp_started) begin
                htp_start <= 1'b1;
                htp_started <= 1'b1;
            end

            if (htp_coeff_valid) begin
                if (htp_coeff !== expected[coeff_idx]) begin
                    $display("OFFICIAL KAT mismatch coeff[%0d]: got=%0d expected=%0d",
                             coeff_idx, htp_coeff, expected[coeff_idx]);
                    errors = errors + 1;
                end
                coeff_idx <= coeff_idx + 1;
                if (coeff_idx == N-1) begin
                    if (errors == 0)
                        $display("OFFICIAL KAT PASSED: Falcon-512 count 0 HashToPoint all %0d coefficients match", N);
                    else
                        $display("OFFICIAL KAT FAILED: %0d mismatches", errors);
                    #20;
                    $finish;
                end
            end
        end
    end

    initial begin
        #5000000;
        $display("OFFICIAL KAT TIMEOUT: coeff_idx=%0d word_idx=%0d", coeff_idx, word_idx);
        $finish;
    end
endmodule
