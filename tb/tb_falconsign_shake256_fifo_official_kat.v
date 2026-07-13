`timescale 1ns/1ps

module tb_falconsign_shake256_fifo_official_kat;
    localparam WORD_COUNT = 10;
    localparam OUT_WORDS  = 160;
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
    wire        fifo_wr_ready;
    wire        fifo_rd_valid;
    reg         fifo_rd_ready;
    wire [63:0] fifo_rd_data;

    reg [63:0] kat_words [0:WORD_COUNT-1];
    reg [63:0] expected  [0:255];

    integer word_idx;
    integer out_idx;
    integer errors;
    integer gap;
    reg     feed_enable;
    reg     read_enable;

    falconsign_shake256 u_shake (
        .clk(clk),
        .rst_n(rst_n),
        .start(shake_start),
        .ready(shake_ready),
        .absorb(shake_absorb),
        .din(shake_din),
        .din_last(shake_din_last),
        .din_last_bytes(shake_din_last_bytes),
        .dout_ready(fifo_wr_ready),
        .dout_valid(shake_dout_valid),
        .dout(shake_dout)
    );

    falconsign_word_fifo #(.WIDTH(64), .DEPTH(32), .ADDR_W(5)) u_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .wr_valid(shake_dout_valid),
        .wr_ready(fifo_wr_ready),
        .wr_data(shake_dout),
        .rd_valid(fifo_rd_valid),
        .rd_ready(fifo_rd_ready),
        .rd_data(fifo_rd_data)
    );

    always #5 clk = ~clk;

    initial begin
        
        $dumpfile("tb_falconsign_shake256_fifo_official_kat.vcd");
        $dumpvars(0, tb_falconsign_shake256_fifo_official_kat);

        $readmemh("SRC/tb/falcon512_kat0_htp_words.hex", kat_words);
        $readmemh("SRC/tb/falcon512_kat0_htp_shake_words.hex", expected);

        clk = 1'b0;
        rst_n = 1'b0;
        shake_start = 1'b0;
        shake_absorb = 1'b0;
        shake_din = 64'd0;
        shake_din_last = 1'b0;
        shake_din_last_bytes = 3'd0;
        fifo_rd_ready = 1'b0;
        word_idx = 0;
        out_idx = 0;
        errors = 0;
        gap = 0;
        feed_enable = 1'b0;
        read_enable = 1'b0;

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
            out_idx <= 0;
            gap <= 0;
            shake_absorb <= 1'b0;
            shake_din <= 64'd0;
            shake_din_last <= 1'b0;
            shake_din_last_bytes <= 3'd0;
            fifo_rd_ready <= 1'b0;
            feed_enable <= 1'b0;
            read_enable <= 1'b0;
        end else begin
            shake_absorb <= 1'b0;
            shake_din_last <= 1'b0;
            shake_din_last_bytes <= 3'd0;
            fifo_rd_ready <= 1'b0;

            if (feed_enable && (word_idx < WORD_COUNT) && shake_ready) begin
                shake_absorb <= 1'b1;
                shake_din <= kat_words[word_idx];
                shake_din_last <= (word_idx == WORD_COUNT-1);
                shake_din_last_bytes <= (word_idx == WORD_COUNT-1) ? LAST_BYTES : 3'd0;
                if (word_idx == WORD_COUNT-1)
                    feed_enable <= 1'b0;
                word_idx <= word_idx + 1;
            end

            if (!feed_enable && (word_idx == WORD_COUNT) && shake_ready)
                read_enable <= 1'b1;

            if (read_enable) begin
                if (gap == 0) begin
                    fifo_rd_ready <= 1'b1;
                    gap <= 4;
                end else begin
                    gap <= gap - 1;
                end
            end

            if (fifo_rd_valid && fifo_rd_ready) begin
                if (fifo_rd_data !== expected[out_idx]) begin
                    $display("SHAKE FIFO KAT mismatch word[%0d]: got=%016h expected=%016h",
                             out_idx, fifo_rd_data, expected[out_idx]);
                    errors = errors + 1;
                end
                out_idx <= out_idx + 1;
                if (out_idx == OUT_WORDS-1) begin
                    if (errors == 0)
                        $display("SHAKE FIFO KAT PASSED: slow reader got first %0d words in order", OUT_WORDS);
                    else
                        $display("SHAKE FIFO KAT FAILED: %0d mismatches", errors);
                    #20;
                    $finish;
                end
            end
        end
    end

    initial begin
        #8000000;
        $display("SHAKE FIFO KAT TIMEOUT: out_idx=%0d word_idx=%0d", out_idx, word_idx);
        $finish;
    end
endmodule
