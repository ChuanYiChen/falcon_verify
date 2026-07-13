`timescale 1ns/1ps

module tb_falconsign_fpr_to_int16_norm;
    reg clk;
    reg rst_n;

    reg start_conv;
    wire conv_ready;
    wire conv_done;
    wire conv_fail;
    wire [7:0] conv_status;

    wire conv_rd_en;
    wire [4:0] conv_rd_addr;
    reg  [255:0] conv_rd_data;
    wire conv_wr_en;
    wire [4:0] conv_wr_addr;
    wire [255:0] conv_wr_data;

    reg start_norm;
    wire norm_ready;
    wire norm_done;
    wire norm_accept;
    wire norm_fail;
    wire [7:0] norm_status;
    wire [63:0] norm_sq;
    wire norm_rd_en;
    wire [4:0] norm_rd_addr;

    reg [255:0] src_mem [0:15];
    reg [255:0] dst_mem [0:1];

    falconsign_fpr_to_int16 #(.ADDR_W(5)) u_conv (
        .clk(clk),
        .rst_n(rst_n),
        .start(start_conv),
        .start_ready(conv_ready),
        .src_base(5'd0),
        .dst_base(5'd0),
        .coeff_count(5'd16),
        .mem_rd_en(conv_rd_en),
        .mem_rd_addr(conv_rd_addr),
        .mem_rd_data(conv_rd_data),
        .mem_wr_en(conv_wr_en),
        .mem_wr_addr(conv_wr_addr),
        .mem_wr_data(conv_wr_data),
        .done(conv_done),
        .fail(conv_fail),
        .status(conv_status)
    );

    falconsign_norm_i16_check #(.ADDR_W(5)) u_norm (
        .clk(clk),
        .rst_n(rst_n),
        .start(start_norm),
        .start_ready(norm_ready),
        .base_addr(5'd0),
        .word_count(5'd1),
        .bound_sq(64'd222),
        .mem_rd_en(norm_rd_en),
        .mem_rd_addr(norm_rd_addr),
        .mem_rd_data(dst_mem[norm_rd_addr]),
        .done(norm_done),
        .accept(norm_accept),
        .fail(norm_fail),
        .status(norm_status),
        .norm_sq(norm_sq)
    );

    always #5 clk = ~clk;

    always @(*) begin
        conv_rd_data = src_mem[conv_rd_addr];
    end

    always @(posedge clk) begin
        if (conv_wr_en)
            dst_mem[conv_wr_addr] <= conv_wr_data;
    end

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        start_conv = 1'b0;
        start_norm = 1'b0;

        // dump VCD for waveform viewing
        $dumpfile("tb_falconsign_fpr_to_int16_norm.vcd");
        $dumpvars(0, tb_falconsign_fpr_to_int16_norm);

        src_mem[0] = {128'd0, 64'd0, 64'h3ff0000000000000}; // 1.0
        src_mem[1] = {128'd0, 64'd0, 64'h4000000000000000}; // 2.0
        src_mem[2] = {128'd0, 64'd0, 64'hc008000000000000}; // -3.0
        src_mem[3] = {128'd0, 64'd0, 64'h4010000000000000}; // 4.0
        src_mem[4] = {128'd0, 64'd0, 64'h4014000000000000}; // 5.0
        src_mem[5] = {128'd0, 64'd0, 64'hc018000000000000}; // -6.0
        src_mem[6] = {128'd0, 64'd0, 64'h401c000000000000}; // 7.0
        src_mem[7] = {128'd0, 64'd0, 64'h4020000000000000}; // 8.0
        src_mem[8] = {128'd0, 64'd0, 64'h3fdf5c28f5c28f5c}; // 0.49 -> 0
        src_mem[9] = {128'd0, 64'd0, 64'h3fe0000000000000}; // 0.5  -> 0 (tie even)
        src_mem[10] = {128'd0, 64'd0, 64'h3fe8000000000000}; // 0.75 -> 1
        src_mem[11] = {128'd0, 64'd0, 64'hbfe8000000000000}; // -0.75 -> -1
        src_mem[12] = {128'd0, 64'd0, 64'h3ff8000000000000}; // 1.5 -> 2 (tie even)
        src_mem[13] = {128'd0, 64'd0, 64'h4004000000000000}; // 2.5 -> 2 (tie even)
        src_mem[14] = {128'd0, 64'd0, 64'hbff8000000000000}; // -1.5 -> -2
        src_mem[15] = {128'd0, 64'd0, 64'hc004000000000000}; // -2.5 -> -2
        dst_mem[0] = 256'd0;
        dst_mem[1] = 256'd0;

        #30 rst_n = 1'b1;
        @(posedge clk);
        start_conv <= 1'b1;
        @(posedge clk);
        start_conv <= 1'b0;

        wait (conv_done);
        @(posedge clk);

        if (conv_fail || conv_status != 8'h00) begin
            $display("TB_FAIL fpr_to_int16 status=0x%02h", conv_status);
            $finish;
        end
        if (dst_mem[0][63:0] !== {16'sd4, -16'sd3, 16'sd2, 16'sd1}) begin
            $display("TB_FAIL pack0 got %h", dst_mem[0][63:0]);
            $finish;
        end
        if (dst_mem[0][127:64] !== {16'sd8, 16'sd7, -16'sd6, 16'sd5}) begin
            $display("TB_FAIL pack1 got %h", dst_mem[0][127:64]);
            $finish;
        end
        if (dst_mem[0][191:128] !== {-16'sd1, 16'sd1, 16'sd0, 16'sd0}) begin
            $display("TB_FAIL fractional pack2 got %h", dst_mem[0][191:128]);
            $finish;
        end
        if (dst_mem[0][255:192] !== {-16'sd2, -16'sd2, 16'sd2, 16'sd2}) begin
            $display("TB_FAIL fractional pack3 got %h", dst_mem[0][255:192]);
            $finish;
        end

        @(posedge clk);
        start_norm <= 1'b1;
        @(posedge clk);
        start_norm <= 1'b0;

        wait (norm_done);
        @(posedge clk);

        if (!norm_accept || norm_fail || norm_status != 8'h00 || norm_sq != 64'd222) begin
            $display("TB_FAIL norm accept=%0d fail=%0d status=0x%02h norm_sq=%0d",
                     norm_accept, norm_fail, norm_status, norm_sq);
            $finish;
        end

        $display("TB_PASS falconsign_fpr_to_int16_norm");
        $finish;
    end

    initial begin
        #200000;
        $display("TB_TIMEOUT falconsign_fpr_to_int16_norm");
        $finish;
    end
endmodule
