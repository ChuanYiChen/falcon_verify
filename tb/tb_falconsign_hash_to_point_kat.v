`timescale 1ns/1ps

module tb_falconsign_hash_to_point_kat;
    reg clk;
    reg rst_n;

    reg         shake_start;
    wire        shake_ready;
    reg         shake_absorb;
    reg  [63:0] shake_din;
    reg         shake_din_last;
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

    reg [2:0]  word_idx;
    reg [5:0]  coeff_idx;
    reg        feed_enable;
    reg [15:0] expected [0:31];
    integer errors;

    localparam [63:0] TEST_MSG_W0 = 64'h46414C434F4E5F53;
    localparam [63:0] TEST_MSG_W1 = 64'h49474E5F54455354;
    localparam [63:0] TEST_MSG_W2 = 64'h5F4D53475F56312E;
    localparam [63:0] TEST_MSG_W3 = 64'h305F5F5F5F5F5F5F;

    falconsign_shake256 u_shake (
        .clk(clk),
        .rst_n(rst_n),
        .start(shake_start),
        .ready(shake_ready),
        .absorb(shake_absorb),
        .din(shake_din),
        .din_last(shake_din_last),
        .din_last_bytes(3'd0),
        .dout_ready(shake_fifo_wr_ready),
        .dout_valid(shake_dout_valid),
        .dout(shake_dout)
    );

    falconsign_word_fifo #(.WIDTH(64), .DEPTH(16), .ADDR_W(4)) u_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .wr_valid(shake_dout_valid),
        .wr_ready(shake_fifo_wr_ready),
        .wr_data(shake_dout),
        .rd_valid(shake_fifo_rd_valid),
        .rd_ready(htp_hash_ready),
        .rd_data(shake_fifo_rd_data)
    );

    falconsign_hash_to_point #(.N(32)) u_htp (
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

    // VCD dump
    initial begin
        $dumpfile("tb_falconsign_hash_to_point_kat.vcd");
        $dumpvars(0, tb_falconsign_hash_to_point_kat);
    end

    initial begin
        expected[0]  = 16'd6493;
        expected[1]  = 16'd5608;
        expected[2]  = 16'd7344;
        expected[3]  = 16'd12218;
        expected[4]  = 16'd10717;
        expected[5]  = 16'd7664;
        expected[6]  = 16'd10587;
        expected[7]  = 16'd4580;
        expected[8]  = 16'd7234;
        expected[9]  = 16'd2534;
        expected[10] = 16'd7595;
        expected[11] = 16'd1997;
        expected[12] = 16'd276;
        expected[13] = 16'd11591;
        expected[14] = 16'd11314;
        expected[15] = 16'd10502;
        expected[16] = 16'd4727;
        expected[17] = 16'd6379;
        expected[18] = 16'd4042;
        expected[19] = 16'd3859;
        expected[20] = 16'd10270;
        expected[21] = 16'd6488;
        expected[22] = 16'd5210;
        expected[23] = 16'd2105;
        expected[24] = 16'd10750;
        expected[25] = 16'd6998;
        expected[26] = 16'd489;
        expected[27] = 16'd4122;
        expected[28] = 16'd4608;
        expected[29] = 16'd12137;
        expected[30] = 16'd1854;
        expected[31] = 16'd12070;
    end

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        shake_start = 1'b0;
        shake_absorb = 1'b0;
        shake_din = 64'd0;
        shake_din_last = 1'b0;
        htp_start = 1'b0;
        word_idx = 3'd0;
        coeff_idx = 6'd0;
        feed_enable = 1'b0;
        errors = 0;

        #30 rst_n = 1'b1;
        @(posedge clk);
        shake_start <= 1'b1;
        @(posedge clk);
        shake_start <= 1'b0;
        feed_enable <= 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            word_idx <= 3'd0;
            shake_absorb <= 1'b0;
            shake_din <= 64'd0;
            shake_din_last <= 1'b0;
            htp_start <= 1'b0;
            coeff_idx <= 6'd0;
            feed_enable <= 1'b0;
        end else begin
            htp_start <= 1'b0;

            if (feed_enable && word_idx < 3'd4 && shake_ready) begin
                shake_absorb <= 1'b1;
                shake_din_last <= (word_idx == 3'd3);
                case (word_idx)
                    3'd0: shake_din <= TEST_MSG_W0;
                    3'd1: shake_din <= TEST_MSG_W1;
                    3'd2: shake_din <= TEST_MSG_W2;
                    default: shake_din <= TEST_MSG_W3;
                endcase
                word_idx <= word_idx + 1'b1;
            end else begin
                shake_absorb <= 1'b0;
            end

            if (word_idx == 3'd4 && shake_ready && htp_ready && !htp_start) begin
                htp_start <= 1'b1;
            end

            if (htp_coeff_valid) begin
                if (htp_coeff !== expected[coeff_idx]) begin
                    $display("KAT mismatch coeff[%0d]: got=%0d expected=%0d",
                             coeff_idx, htp_coeff, expected[coeff_idx]);
                    errors = errors + 1;
                end
                coeff_idx <= coeff_idx + 1'b1;
                if (coeff_idx == 6'd31) begin
                    if (errors == 0)
                        $display("KAT PASSED: SHAKE256 -> Falcon HashToPoint first 32 coefficients match");
                    else
                        $display("KAT FAILED: %0d mismatches", errors);
                    #20;
                    $finish;
                end
            end
        end
    end

    initial begin
        #1000000;
        $display("KAT TIMEOUT: coeff_idx=%0d", coeff_idx);
        $finish;
    end
endmodule
