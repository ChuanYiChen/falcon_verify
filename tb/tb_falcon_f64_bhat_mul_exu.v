`timescale 1ns/1ps

module tb_falcon_f64_bhat_mul_exu;
    localparam ADDR_W = 6;
    localparam T_BASE = 0;
    localparam Z_BASE = 8;
    localparam B01_BASE = 20;
    localparam B11_BASE = 28;
    localparam DST_BASE = 40;

    reg clk;
    reg rst_n;
    reg start;
    reg identity_mode;
    wire start_ready;
    wire done;
    wire fail;
    wire [7:0] status;

    wire mem_rd_en;
    wire [ADDR_W-1:0] mem_rd_addr;
    reg [255:0] mem_rd_data;
    wire mem_wr_en;
    wire [ADDR_W-1:0] mem_wr_addr;
    wire [255:0] mem_wr_data;

    wire fpu_req_valid;
    wire fpu_req_ready;
    wire [3:0] fpu_req_op;
    wire [63:0] fpu_req_a;
    wire [63:0] fpu_req_b;
    wire [63:0] fpu_req_c;
    wire fpu_rsp_valid;
    wire [63:0] fpu_rsp_result;
    wire [4:0] fpu_rsp_flags;

    reg [255:0] mem [0:63];
    integer errors;
    integer i;

    falcon_f64_bhat_mul_exu #(.ADDR_W(ADDR_W)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .start_ready(start_ready),
        .identity_mode(identity_mode),
        .t_base(T_BASE[ADDR_W-1:0]),
        .z_base(Z_BASE[ADDR_W-1:0]),
        .b00_base(6'd0),
        .b01_base(B01_BASE[ADDR_W-1:0]),
        .b10_base(6'd0),
        .b11_base(B11_BASE[ADDR_W-1:0]),
        .s2_fft_base(DST_BASE[ADDR_W-1:0]),
        .word_count(6'd2),
        .mem_rd_en(mem_rd_en),
        .mem_rd_addr(mem_rd_addr),
        .mem_rd_data(mem_rd_data),
        .mem_wr_en(mem_wr_en),
        .mem_wr_addr(mem_wr_addr),
        .mem_wr_data(mem_wr_data),
        .fpu_req_valid(fpu_req_valid),
        .fpu_req_ready(fpu_req_ready),
        .fpu_req_op(fpu_req_op),
        .fpu_req_a(fpu_req_a),
        .fpu_req_b(fpu_req_b),
        .fpu_req_c(fpu_req_c),
        .fpu_rsp_valid(fpu_rsp_valid),
        .fpu_rsp_result(fpu_rsp_result),
        .done(done),
        .fail(fail),
        .status(status)
    );

    falcon_fp_fpu u_fpu (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(fpu_req_valid),
        .req_ready(fpu_req_ready),
        .req_op(fpu_req_op),
        .req_a(fpu_req_a),
        .req_b(fpu_req_b),
        .req_c(fpu_req_c),
        .req_fmt(2'b01),
        .req_rm(3'b000),
        .req_fcvt_op(2'b00),
        .rsp_valid(fpu_rsp_valid),
        .rsp_ready(1'b1),
        .rsp_result(fpu_rsp_result),
        .rsp_flags(fpu_rsp_flags),
        .busy()
    );

    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (mem_rd_en)
            mem_rd_data <= mem[mem_rd_addr];
        if (mem_wr_en)
            mem[mem_wr_addr] <= mem_wr_data;
    end

    function [255:0] cplx_word;
        input real re;
        input real im;
        begin
            cplx_word = {128'd0, $realtobits(im), $realtobits(re)};
        end
    endfunction

    task issue_and_wait;
        input mode;
        integer timeout;
        begin
            identity_mode <= mode;
            @(posedge clk);
            if (!start_ready) begin
                $display("TB_ERROR start_ready=0");
                errors = errors + 1;
            end
            start <= 1'b1;
            @(posedge clk);
            start <= 1'b0;

            timeout = 0;
            while (!done && timeout < 2000) begin
                timeout = timeout + 1;
                @(posedge clk);
            end
            if (timeout >= 2000) begin
                $display("TB_ERROR timeout mode=%0d", mode);
                errors = errors + 1;
            end
            if (fail) begin
                $display("TB_ERROR fail mode=%0d status=%02x", mode, status);
                errors = errors + 1;
            end
            @(posedge clk);
        end
    endtask

    task expect_cplx;
        input [ADDR_W-1:0] addr;
        input real exp_re;
        input real exp_im;
        reg [63:0] exp_re_bits;
        reg [63:0] exp_im_bits;
        begin
            exp_re_bits = $realtobits(exp_re);
            exp_im_bits = $realtobits(exp_im);
            if (mem[addr][63:0] !== exp_re_bits || mem[addr][127:64] !== exp_im_bits) begin
                $display("TB_ERROR mem[%0d] got=(%h,%h) exp=(%h,%h)",
                         addr, mem[addr][63:0], mem[addr][127:64],
                         exp_re_bits, exp_im_bits);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        start = 1'b0;
        identity_mode = 1'b1;
        mem_rd_data = 256'd0;
        errors = 0;

        // VCD dump
        $dumpfile("tb_falcon_f64_bhat_mul_exu.vcd");
        $dumpvars(0, tb_falcon_f64_bhat_mul_exu);

        for (i = 0; i < 64; i = i + 1)
            mem[i] = 256'd0;

        // word 0:
        // d0=(2+j3)-(1+j1)=1+j2
        // d1=(5+j7)-(2+j3)=3+j4
        // b01=2+j0.5, b11=-1+j1
        // out=d0*b01+d1*b11 = -6+j3.5
        mem[T_BASE + 0] = cplx_word(2.0, 3.0);
        mem[Z_BASE + 0] = cplx_word(1.0, 1.0);
        mem[T_BASE + 2] = cplx_word(5.0, 7.0);
        mem[Z_BASE + 2] = cplx_word(2.0, 3.0);
        mem[B01_BASE + 0] = cplx_word(2.0, 0.5);
        mem[B11_BASE + 0] = cplx_word(-1.0, 1.0);

        // word 1:
        // d0=-1+j4, d1=2-j1, b01=0.5-j0.5, b11=3+j0
        // out=7.5-j0.5
        mem[T_BASE + 1] = cplx_word(3.0, 5.0);
        mem[Z_BASE + 1] = cplx_word(4.0, 1.0);
        mem[T_BASE + 3] = cplx_word(7.0, 2.0);
        mem[Z_BASE + 3] = cplx_word(5.0, 3.0);
        mem[B01_BASE + 1] = cplx_word(0.5, -0.5);
        mem[B11_BASE + 1] = cplx_word(3.0, 0.0);

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        issue_and_wait(1'b1);
        expect_cplx(DST_BASE + 0, 1.0, 2.0);
        expect_cplx(DST_BASE + 1, -1.0, 4.0);

        mem[DST_BASE + 0] = 256'd0;
        mem[DST_BASE + 1] = 256'd0;

        issue_and_wait(1'b0);
        expect_cplx(DST_BASE + 0, -6.0, 3.5);
        expect_cplx(DST_BASE + 1, 7.5, -0.5);

        if (errors == 0)
            $display("TB_PASS falcon_f64_bhat_mul_exu");
        else
            $display("TB_FAIL falcon_f64_bhat_mul_exu errors=%0d", errors);
        $finish;
    end

    initial begin
        #2000000;
        $display("TB_TIMEOUT falcon_f64_bhat_mul_exu");
        $finish;
    end
endmodule
