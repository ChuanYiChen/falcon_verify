`timescale 1ns/1ps
// Module: falconsign_top
// Purpose: top-level Falcon signing engine. It coordinates shared memory,
// SHAKE/HashToPoint, FFT, ffSampling, SamplerZ, Bhat multiplication, IFFT,
// integer conversion, NTT reconstruction and norm rejection.
//
// FalconSign Top — 3-layer control: phase FSM → task scheduler → EXU cluster.
module falconsign_top #(
    parameter ADDR_W=13, LEVEL_W=4, INDEX_W=10,
    parameter BFU_LANES=1  // 1=scalar BFU, 2=2-lane parallel BFU, 4=4-lane vector BFU
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        bus_cs,
    input  wire        bus_wr,
    input  wire [15:0] bus_addr,
    input  wire [31:0] bus_wdata,
    output reg  [31:0] bus_rdata,
    output reg         bus_ready,
    output reg         bus_irq,
    output wire        busy,
    output reg         done,
    output reg         fail,
    output reg  [7:0]  status
);
    localparam [15:0] REG_CR     = 16'h0000;
    localparam [15:0] REG_SR     = 16'h0004;
    localparam [15:0] REG_CFG    = 16'h0008;
    localparam [15:0] REG_MEM_HI = 16'h000C;
    localparam [3:0] SI=0, SH=1, HP=2, FC=3, TG=4, FS=5, VD=6, IV=7, FI=8, N1=9, RC=10, CN=11, EN=12, OU=13, SD=14;
    localparam integer MEM_DEPTH = 8192;
    localparam [4:0]  FALCON_LOGN = 5'd9;
    localparam [ADDR_W-1:0] FALCON_N_WORDS = 512;
    localparam [ADDR_W-1:0] HTP_C_WORDS    = FALCON_N_WORDS; // one FP64 complex coefficient per word
    localparam [5:0]        HTP_C_INT_WORDS = 6'd32;
    // Sign workspace map. Each FP64 complex polynomial occupies 512 memory
    // words. The full ffLDL tree stores every L10 polynomial coefficient
    // plus leaves: 2816 words.
    localparam [ADDR_W-1:0] LAYOUT_C_BASE      = 0;
    localparam [ADDR_W-1:0] LAYOUT_FFT_BASE    = 0;       // FFT(c), aliases t0
    localparam [ADDR_W-1:0] LAYOUT_T0_BASE     = LAYOUT_FFT_BASE;
    localparam [ADDR_W-1:0] LAYOUT_T1_BASE     = 512;
    localparam [ADDR_W-1:0] LAYOUT_TREE_BASE   = 1024;
    localparam [ADDR_W-1:0] LAYOUT_Z0_BASE     = 3840;
    localparam [ADDR_W-1:0] LAYOUT_Z1_BASE     = 4352;
    localparam [ADDR_W-1:0] LAYOUT_H_WORK_BASE = LAYOUT_Z1_BASE + {{(ADDR_W-6){1'b0}}, 6'd32};
    localparam [ADDR_W-1:0] LAYOUT_B00_BASE    = 4864;
    localparam [ADDR_W-1:0] LAYOUT_B01_BASE    = 5376;
    localparam [ADDR_W-1:0] LAYOUT_B10_BASE    = 5888;
    localparam [ADDR_W-1:0] LAYOUT_B11_BASE    = 6400;
    localparam [ADDR_W-1:0] LAYOUT_SIG_BASE    = 6912;
    localparam [ADDR_W-1:0] LAYOUT_C_INT_BASE  = 7424;
    localparam [ADDR_W-1:0] LAYOUT_H_BASE      = 7456;
    localparam [ADDR_W-1:0] LAYOUT_S1_BASE     = 7488;    // s1 output (32 words)
    localparam [ADDR_W-1:0] LAYOUT_TMP_BASE    = 7552;    // ffSampling internal scratch; scalar leaf scratch aliases SIG
    localparam [ADDR_W-1:0] LAYOUT_T_BASE      = LAYOUT_T0_BASE;
    localparam [ADDR_W-1:0] LAYOUT_Z_BASE      = LAYOUT_Z0_BASE;
    localparam [ADDR_W-1:0] NTT_N_WORDS        = 32;      // 512 int16 / 16 per word
    localparam [ADDR_W-1:0] LAYOUT_NORM_WORDS = NTT_N_WORDS;
    reg [3:0] st, sn;
    reg cr_start;
    reg cfg_bypass_fs;
    reg cfg_force_accept;
    reg cfg_start_at_fs;
    reg cfg_dynamic_tree;
    reg cfg_msg_only_hash;
    reg [1:0] mem_addr_hi;

    reg [63:0] hp_coeff_f64;
    reg [63:0] hp_coeff_x64;
    reg [10:0] hp_coeff_exp;
    reg [51:0] hp_coeff_frac;
    integer hp_coeff_ii;
    integer hp_coeff_pos;
    reg     hp_coeff_found;

    // Address layout decode. These constants define the shared-memory windows
    // consumed by the signing phases and the debug/testbench dumps.
    always @(*) begin
        hp_coeff_f64 = 64'd0;
        hp_coeff_x64 = {48'd0, htp_coeff};
        hp_coeff_exp = 11'd0;
        hp_coeff_frac = 52'd0;
        hp_coeff_pos = 0;
        if (htp_coeff != 16'd0) begin
            hp_coeff_found = 1'b0;
            for (hp_coeff_ii = 0; hp_coeff_ii < 16; hp_coeff_ii = hp_coeff_ii + 1) begin
                if (htp_coeff[15 - hp_coeff_ii] && !hp_coeff_found) begin
                    hp_coeff_pos = 15 - hp_coeff_ii;
                    hp_coeff_found = 1'b1;
                end
            end
            hp_coeff_exp = 11'd1023 + hp_coeff_pos;
            hp_coeff_frac = (hp_coeff_x64 << (63 - hp_coeff_pos)) >> 11;
            hp_coeff_f64 = {1'b0, hp_coeff_exp, hp_coeff_frac};
        end
    end

    // ─── Memory ───
    wire                mem_rd_en;
    wire [ADDR_W-1:0]   mem_rd_addr;
    wire [255:0]        mem_rd_data;
    wire                mem_wr_en;
    wire [ADDR_W-1:0]   mem_wr_addr;
    wire [255:0]        mem_wr_data;
    wire [255:0]        mem_b_rd_data;

    falconsign_memory #(.ADDR_W(ADDR_W),.DEPTH(MEM_DEPTH)) u_mem (
        .clk(clk),.rst_n(rst_n),
        .rd_en(mem_rd_en),.rd_addr(mem_rd_addr),.rd_data(mem_rd_data),
        .wr_en(mem_wr_en),.wr_addr(mem_wr_addr),.wr_data(mem_wr_data));
    assign mem_b_rd_data = mem_rd_data;

    // ─── FPU ───
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
    falcon_fp_fpu u_fpu(.clk(clk),.rst_n(rst_n),
        .req_valid(fpu_req_valid),.req_ready(fpu_req_ready),
        .req_op(fpu_req_op),.req_a(fpu_req_a),.req_b(fpu_req_b),.req_c(fpu_req_c),
        .req_fmt(fpu_req_fmt),.req_rm(fpu_req_rm),.req_fcvt_op(fpu_req_fcvt_op),
        .rsp_valid(fpu_rsp_valid),.rsp_ready(fpu_rsp_ready),
        .rsp_result(fpu_rsp_result),.rsp_flags(fpu_rsp_flags),.busy());
    assign fpu_req_fmt      = 2'd0;
    assign fpu_req_rm       = 3'd0;
    assign fpu_req_fcvt_op  = 2'd0;
    assign fpu_rsp_ready    = 1'b1;

    wire        sz_fpu_req_valid;
    wire        sz_fpu_req_ready;
    wire [3:0]  sz_fpu_req_op;
    wire [63:0] sz_fpu_req_a;
    wire [63:0] sz_fpu_req_b;
    wire [63:0] sz_fpu_req_c;
    wire        sz_fpu_rsp_valid;
    wire [63:0] sz_fpu_rsp_result;
    wire        fe_fpu_req_valid;
    wire        fe_fpu_req_ready;
    wire [3:0]  fe_fpu_req_op;
    wire [63:0] fe_fpu_req_a;
    wire [63:0] fe_fpu_req_b;
    wire [63:0] fe_fpu_req_c;
    wire        fe_fpu_rsp_valid;
    wire [63:0] fe_fpu_rsp_result;
    wire        vd_fpu_req_valid;
    wire        vd_fpu_req_ready;
    wire [3:0]  vd_fpu_req_op;
    wire [63:0] vd_fpu_req_a;
    wire [63:0] vd_fpu_req_b;
    wire [63:0] vd_fpu_req_c;
    wire        vd_fpu_rsp_valid;
    wire [63:0] vd_fpu_rsp_result;
    wire        tg_fpu_req_valid;
    wire        tg_fpu_req_ready;
    wire [3:0]  tg_fpu_req_op;
    wire [63:0] tg_fpu_req_a;
    wire [63:0] tg_fpu_req_b;
    wire [63:0] tg_fpu_req_c;
    wire        tg_fpu_rsp_valid;
    wire [63:0] tg_fpu_rsp_result;

    // ─── Shared 2-Lane FPU Array (replaces 2 BFU instances) ───
    // Reference: TCHES2025 Section 19.3 - Unified BFU/FPU Array
    // Shared by FC (FFT butterfly), FS (ffSampling SPLIT/MERGE), and VD (BhatMul) phases.
    // Phase-based muxing: FC/FS/VD are mutually exclusive, no arbitration needed.
    wire        sh_fpu_req_v, sh_fpu_req_r;
    wire [2:0]  sh_fpu_mode;
    wire [63:0] sh_fpu_a0_re, sh_fpu_a0_im, sh_fpu_b0_re, sh_fpu_b0_im;
    wire [63:0] sh_fpu_a1_re, sh_fpu_a1_im, sh_fpu_b1_re, sh_fpu_b1_im;
    wire [63:0] sh_fpu_w_re, sh_fpu_w_im;
    wire [63:0] sh_fpu_w1_re, sh_fpu_w1_im;
    wire        sh_fpu_rsp_v;
    wire [63:0] sh_fpu_y0_re, sh_fpu_y0_im, sh_fpu_y1_re, sh_fpu_y1_im;
    wire [63:0] sh_fpu_y0_re_1, sh_fpu_y0_im_1, sh_fpu_y1_re_1, sh_fpu_y1_im_1;
    wire        sh_fpu_busy;

    falconsign_shared_fpu_lanes u_sh_fpu (
        .clk(clk), .rst_n(rst_n),
        .req_valid(sh_fpu_req_v), .req_ready(sh_fpu_req_r),
        .req_mode(sh_fpu_mode),
        .req_a0_re(sh_fpu_a0_re), .req_a0_im(sh_fpu_a0_im),
        .req_b0_re(sh_fpu_b0_re), .req_b0_im(sh_fpu_b0_im),
        .req_a1_re(sh_fpu_a1_re), .req_a1_im(sh_fpu_a1_im),
        .req_b1_re(sh_fpu_b1_re), .req_b1_im(sh_fpu_b1_im),
        .req_w_re(sh_fpu_w_re), .req_w_im(sh_fpu_w_im),
        .req_w1_re(sh_fpu_w1_re), .req_w1_im(sh_fpu_w1_im),
        .rsp_valid(sh_fpu_rsp_v), .rsp_ready(1'b1),
        .rsp_y0_re(sh_fpu_y0_re), .rsp_y0_im(sh_fpu_y0_im),
        .rsp_y1_re(sh_fpu_y1_re), .rsp_y1_im(sh_fpu_y1_im),
        .rsp_y0_re_1(sh_fpu_y0_re_1), .rsp_y0_im_1(sh_fpu_y0_im_1),
        .rsp_y1_re_1(sh_fpu_y1_re_1), .rsp_y1_im_1(sh_fpu_y1_im_1),
        .busy(sh_fpu_busy)
    );

    // ─── Phase-based FPU lane mux ───
    wire use_sh_fpu_fc = (st == FC) || (st == IV);
    wire use_sh_fpu_vd = (st == VD);
    wire use_sh_fpu_fs = (st == FS);

    // FC phase: FFT forward FPU lane signals
    wire        fc_fpu_req_v, fc_fpu_req_r;
    wire [2:0]  fc_fpu_mode;
    wire [63:0] fc_fpu_a0_re, fc_fpu_a0_im, fc_fpu_b0_re, fc_fpu_b0_im;
    wire [63:0] fc_fpu_a1_re, fc_fpu_a1_im, fc_fpu_b1_re, fc_fpu_b1_im;
    wire [63:0] fc_fpu_w_re, fc_fpu_w_im;
    wire [63:0] fc_fpu_w1_re, fc_fpu_w1_im;
    wire        fc_fpu_rsp_v;
    wire [63:0] fc_fpu_y0_re, fc_fpu_y0_im, fc_fpu_y1_re, fc_fpu_y1_im;
    wire [63:0] fc_fpu_y0_re_1, fc_fpu_y0_im_1, fc_fpu_y1_re_1, fc_fpu_y1_im_1;

    // VD phase: BhatMul FPU lane signals (adapter from BFU-style to FPU-lane-style)
    wire        vd_bfu0_in_v, vd_bfu0_in_r;
    wire [63:0] vd_bfu0_br, vd_bfu0_bi, vd_bfu0_wr, vd_bfu0_wi;
    wire        vd_bfu0_out_v;
    wire [63:0] vd_bfu0_y0r, vd_bfu0_y0i;
    wire        vd_bfu1_in_v, vd_bfu1_in_r;
    wire [63:0] vd_bfu1_br, vd_bfu1_bi, vd_bfu1_wr, vd_bfu1_wi;
    wire        vd_bfu1_out_v;
    wire [63:0] vd_bfu1_y0r, vd_bfu1_y0i;

    // VD FPU lane adapter: BFU-style handshake → FPU-lane-style
    // BhatMul fires both BFUs simultaneously with a=0, using BUTTERFLY mode
    wire        vd_fpu_req_v = vd_bfu0_in_v;  // fire when BFU0 fires
    wire        vd_fpu_req_r;
    wire [2:0]  vd_fpu_mode = 3'd1;  // BUTTERFLY
    wire [63:0] vd_fpu_a0_re = 64'd0, vd_fpu_a0_im = 64'd0;  // a=0 for BhatMul
    wire [63:0] vd_fpu_b0_re = vd_bfu0_br, vd_fpu_b0_im = vd_bfu0_bi;
    wire [63:0] vd_fpu_a1_re = 64'd0, vd_fpu_a1_im = 64'd0;  // a=0 for BhatMul
    wire [63:0] vd_fpu_b1_re = vd_bfu1_br, vd_fpu_b1_im = vd_bfu1_bi;
    wire [63:0] vd_fpu_w_re  = vd_bfu0_wr, vd_fpu_w_im  = vd_bfu0_wi;   // lane 0 twiddle
    wire [63:0] vd_fpu_w1_re = vd_bfu1_wr, vd_fpu_w1_im = vd_bfu1_wi;  // lane 1 twiddle
    wire        vd_fpu_rsp_v;
    wire [63:0] vd_fpu_y0_re, vd_fpu_y0_im, vd_fpu_y1_re, vd_fpu_y1_im;
    wire [63:0] vd_fpu_y0_re_1, vd_fpu_y0_im_1, vd_fpu_y1_re_1, vd_fpu_y1_im_1;

    assign vd_bfu0_in_r = vd_fpu_req_r;
    assign vd_bfu1_in_r = vd_fpu_req_r;
    assign vd_bfu0_out_v = vd_fpu_rsp_v;
    assign vd_bfu1_out_v = vd_fpu_rsp_v;
    assign vd_bfu0_y0r  = vd_fpu_y0_re;  assign vd_bfu0_y0i = vd_fpu_y0_im;
    assign vd_bfu1_y0r  = vd_fpu_y0_re_1; assign vd_bfu1_y0i = vd_fpu_y0_im_1;

    // FS phase: ffSampling FPU lane signals (driven by u_fe)
    wire        fe_fpu_req_v;
    wire        fe_fpu_req_r;
    wire [2:0]  fe_fpu_mode;
    wire [63:0] fe_fpu_a0_re, fe_fpu_a0_im, fe_fpu_b0_re, fe_fpu_b0_im;
    wire [63:0] fe_fpu_a1_re, fe_fpu_a1_im, fe_fpu_b1_re, fe_fpu_b1_im;
    wire [63:0] fe_fpu_w_re, fe_fpu_w_im;
    wire [63:0] fe_fpu_w1_re, fe_fpu_w1_im;
    wire        fe_fpu_rsp_v;
    wire [63:0] fe_fpu_y0_re, fe_fpu_y0_im, fe_fpu_y1_re, fe_fpu_y1_im;
    wire [63:0] fe_fpu_y0_re_1, fe_fpu_y0_im_1, fe_fpu_y1_re_1, fe_fpu_y1_im_1;

    // FPU lane request mux (3-way: FC / VD / FS)
    assign sh_fpu_req_v = use_sh_fpu_fc ? fc_fpu_req_v :
                          use_sh_fpu_vd ? vd_fpu_req_v :
                          use_sh_fpu_fs ? fe_fpu_req_v : 1'b0;
    assign sh_fpu_mode  = use_sh_fpu_fc ? fc_fpu_mode  :
                          use_sh_fpu_vd ? vd_fpu_mode  :
                          use_sh_fpu_fs ? fe_fpu_mode  : 3'd0;
    assign sh_fpu_a0_re = use_sh_fpu_fc ? fc_fpu_a0_re :
                          use_sh_fpu_vd ? vd_fpu_a0_re :
                          use_sh_fpu_fs ? fe_fpu_a0_re : 64'd0;
    assign sh_fpu_a0_im = use_sh_fpu_fc ? fc_fpu_a0_im :
                          use_sh_fpu_vd ? vd_fpu_a0_im :
                          use_sh_fpu_fs ? fe_fpu_a0_im : 64'd0;
    assign sh_fpu_b0_re = use_sh_fpu_fc ? fc_fpu_b0_re :
                          use_sh_fpu_vd ? vd_fpu_b0_re :
                          use_sh_fpu_fs ? fe_fpu_b0_re : 64'd0;
    assign sh_fpu_b0_im = use_sh_fpu_fc ? fc_fpu_b0_im :
                          use_sh_fpu_vd ? vd_fpu_b0_im :
                          use_sh_fpu_fs ? fe_fpu_b0_im : 64'd0;
    assign sh_fpu_a1_re = use_sh_fpu_fc ? fc_fpu_a1_re :
                          use_sh_fpu_vd ? vd_fpu_a1_re :
                          use_sh_fpu_fs ? fe_fpu_a1_re : 64'd0;
    assign sh_fpu_a1_im = use_sh_fpu_fc ? fc_fpu_a1_im :
                          use_sh_fpu_vd ? vd_fpu_a1_im :
                          use_sh_fpu_fs ? fe_fpu_a1_im : 64'd0;
    assign sh_fpu_b1_re = use_sh_fpu_fc ? fc_fpu_b1_re :
                          use_sh_fpu_vd ? vd_fpu_b1_re :
                          use_sh_fpu_fs ? fe_fpu_b1_re : 64'd0;
    assign sh_fpu_b1_im = use_sh_fpu_fc ? fc_fpu_b1_im :
                          use_sh_fpu_vd ? vd_fpu_b1_im :
                          use_sh_fpu_fs ? fe_fpu_b1_im : 64'd0;
    assign sh_fpu_w_re  = use_sh_fpu_fc ? fc_fpu_w_re  :
                          use_sh_fpu_vd ? vd_fpu_w_re  :
                          use_sh_fpu_fs ? fe_fpu_w_re  : 64'd0;
    assign sh_fpu_w_im  = use_sh_fpu_fc ? fc_fpu_w_im  :
                          use_sh_fpu_vd ? vd_fpu_w_im  :
                          use_sh_fpu_fs ? fe_fpu_w_im  : 64'd0;
    assign sh_fpu_w1_re = use_sh_fpu_fc ? fc_fpu_w1_re :
                          use_sh_fpu_vd ? vd_fpu_w1_re :
                          use_sh_fpu_fs ? fe_fpu_w_re  : 64'd0;
    assign sh_fpu_w1_im = use_sh_fpu_fc ? fc_fpu_w1_im :
                          use_sh_fpu_vd ? vd_fpu_w1_im :
                          use_sh_fpu_fs ? fe_fpu_w_im  : 64'd0;

    // FPU lane response demux
    assign fc_fpu_req_r  = use_sh_fpu_fc ? sh_fpu_req_r  : 1'b0;
    assign vd_fpu_req_r  = use_sh_fpu_vd ? sh_fpu_req_r  : 1'b0;
    assign fc_fpu_rsp_v  = use_sh_fpu_fc ? sh_fpu_rsp_v  : 1'b0;
    assign vd_fpu_rsp_v  = use_sh_fpu_vd ? sh_fpu_rsp_v  : 1'b0;
    assign fe_fpu_rsp_v  = use_sh_fpu_fs ? sh_fpu_rsp_v  : 1'b0;
    assign fc_fpu_y0_re  = sh_fpu_y0_re;  assign fc_fpu_y0_im  = sh_fpu_y0_im;
    assign fc_fpu_y1_re  = sh_fpu_y1_re;  assign fc_fpu_y1_im  = sh_fpu_y1_im;
    assign fc_fpu_y0_re_1 = sh_fpu_y0_re_1; assign fc_fpu_y0_im_1 = sh_fpu_y0_im_1;
    assign fc_fpu_y1_re_1 = sh_fpu_y1_re_1; assign fc_fpu_y1_im_1 = sh_fpu_y1_im_1;
    assign vd_fpu_y0_re  = sh_fpu_y0_re;  assign vd_fpu_y0_im  = sh_fpu_y0_im;
    assign vd_fpu_y1_re  = sh_fpu_y1_re;  assign vd_fpu_y1_im  = sh_fpu_y1_im;
    assign vd_fpu_y0_re_1 = sh_fpu_y0_re_1; assign vd_fpu_y0_im_1 = sh_fpu_y0_im_1;
    assign vd_fpu_y1_re_1 = sh_fpu_y1_re_1; assign vd_fpu_y1_im_1 = sh_fpu_y1_im_1;
    assign fe_fpu_y0_re  = sh_fpu_y0_re;  assign fe_fpu_y0_im  = sh_fpu_y0_im;
    assign fe_fpu_y1_re  = sh_fpu_y1_re;  assign fe_fpu_y1_im  = sh_fpu_y1_im;
    assign fe_fpu_y0_re_1 = sh_fpu_y0_re_1; assign fe_fpu_y0_im_1 = sh_fpu_y0_im_1;
    assign fe_fpu_y1_re_1 = sh_fpu_y1_re_1; assign fe_fpu_y1_im_1 = sh_fpu_y1_im_1;

    // (FFT FPU client removed — parallel 2-BFU uses internal FPUs)

    // ─── 4-way FPU mux (SamplerZ / ffSampling EXU / TargetGen EXU / VerifyDecode EXU) ───
    // Track the owner of the outstanding request so a response is never routed
    // according to a later request from the other client.
    reg fpu_owner_valid;
    reg [1:0] fpu_owner;
    wire fpu_mux_idle = !fpu_owner_valid;
    wire fpu_sel_sz   = sz_fpu_req_valid;
    wire fpu_sel_fe   = !sz_fpu_req_valid && fe_fpu_req_valid;
    wire fpu_sel_tg   = !sz_fpu_req_valid && !fe_fpu_req_valid && tg_fpu_req_valid;
    wire fpu_sel_vd   = !sz_fpu_req_valid && !fe_fpu_req_valid && !tg_fpu_req_valid && vd_fpu_req_valid;

    assign fpu_req_valid = fpu_mux_idle && (sz_fpu_req_valid || fe_fpu_req_valid || tg_fpu_req_valid || vd_fpu_req_valid);
    assign fpu_req_op    = fpu_sel_sz  ? sz_fpu_req_op  :
                           fpu_sel_fe  ? fe_fpu_req_op  :
                           fpu_sel_tg  ? tg_fpu_req_op  : vd_fpu_req_op;
    assign fpu_req_a     = fpu_sel_sz  ? sz_fpu_req_a   :
                           fpu_sel_fe  ? fe_fpu_req_a   :
                           fpu_sel_tg  ? tg_fpu_req_a   : vd_fpu_req_a;
    assign fpu_req_b     = fpu_sel_sz  ? sz_fpu_req_b   :
                           fpu_sel_fe  ? fe_fpu_req_b   :
                           fpu_sel_tg  ? tg_fpu_req_b   : vd_fpu_req_b;
    assign fpu_req_c     = fpu_sel_sz  ? sz_fpu_req_c   :
                           fpu_sel_fe  ? fe_fpu_req_c   :
                           fpu_sel_tg  ? tg_fpu_req_c   : vd_fpu_req_c;
    assign sz_fpu_req_ready  = fpu_mux_idle && fpu_sel_sz && fpu_req_ready;
    assign fe_fpu_req_ready  = fpu_mux_idle && fpu_sel_fe && fpu_req_ready;
    assign tg_fpu_req_ready  = fpu_mux_idle && fpu_sel_tg && fpu_req_ready;
    assign vd_fpu_req_ready  = fpu_mux_idle && fpu_sel_vd && fpu_req_ready;
    assign sz_fpu_rsp_valid  = fpu_owner_valid && (fpu_owner == 2'd1) && fpu_rsp_valid;
    assign fe_fpu_rsp_valid  = fpu_owner_valid && (fpu_owner == 2'd2) && fpu_rsp_valid;
    assign tg_fpu_rsp_valid  = fpu_owner_valid && (fpu_owner == 2'd3) && fpu_rsp_valid;
    assign vd_fpu_rsp_valid  = fpu_owner_valid && (fpu_owner == 2'd0) && fpu_rsp_valid;
    assign sz_fpu_rsp_result  = fpu_rsp_result;
    assign fe_fpu_rsp_result  = fpu_rsp_result;
    assign tg_fpu_rsp_result  = fpu_rsp_result;
    assign vd_fpu_rsp_result  = fpu_rsp_result;

    // Bus register file. Software writes control/configuration and reads status
    // while Port C handles memory debug/load accesses.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fpu_owner_valid <= 1'b0;
            fpu_owner       <= 2'd0;
        end else begin
            if (fpu_owner_valid && fpu_rsp_valid && fpu_rsp_ready) begin
                fpu_owner_valid <= 1'b0;
            end
            if (fpu_mux_idle && fpu_req_valid && fpu_req_ready) begin
                fpu_owner_valid <= 1'b1;
                fpu_owner       <= fpu_sel_sz  ? 2'd1 :
                                   fpu_sel_fe  ? 2'd2 :
                                   fpu_sel_tg  ? 2'd3 : 2'd0;
            end
        end
    end

    // ─── SHAKE256 ───
    wire        shake_start;
    wire        shake_ready;
    reg         shake_absorb;
    reg  [63:0] shake_din;
    reg         shake_din_last;
    wire        shake_dout_valid;
    wire [63:0] shake_dout;
    wire        shake_fifo_wr_ready;
    wire        shake_fifo_rd_valid;
    wire [63:0] shake_fifo_rd_data;
    wire        htp_hash_ready;

    falconsign_shake256 u_shake(
        .clk(clk), .rst_n(rst_n),
        .start(shake_start), .ready(shake_ready),
        .absorb(shake_absorb),
        .din(shake_din), .din_last(shake_din_last), .din_last_bytes(3'd0),
        .dout_ready(shake_fifo_wr_ready), 
        .dout_valid(shake_dout_valid), .dout(shake_dout));

    falconsign_word_fifo #(.WIDTH(64), .DEPTH(8), .ADDR_W(3)) u_shake_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .wr_valid(shake_dout_valid),
        .wr_ready(shake_fifo_wr_ready),
        .wr_data(shake_dout),
        .rd_valid(shake_fifo_rd_valid),
        .rd_ready(htp_hash_ready),
        .rd_data(shake_fifo_rd_data)
    );

    // ─── HashToPoint ───
    wire        htp_start;
    wire        htp_ready;
    wire [63:0] htp_hash_word;
    wire        htp_hash_valid;
    wire [15:0] htp_coeff;
    wire        htp_coeff_valid;

    assign htp_hash_word  = shake_fifo_rd_data;
    assign htp_hash_valid = shake_fifo_rd_valid;

    falconsign_hash_to_point #(.N(512)) u_htp(
        .clk(clk), .rst_n(rst_n),
        .start(htp_start), .ready(htp_ready),
        .hash_word(htp_hash_word), .hash_valid(htp_hash_valid),
        .hash_ready(htp_hash_ready),
        .coeff(htp_coeff), .coeff_valid(htp_coeff_valid));

    // ─── HP coefficient packing → memory write ───
    reg  [3:0]         hp_coeff_cnt;    // c_int packs 16 coefficients per 256-bit word
    reg  [255:0]       hp_coeff_buf;
    reg  [ADDR_W-1:0]  hp_wr_addr;      // next packed c write address
    reg                hp_wr_en;
    reg  [255:0]       hp_wr_data;
    reg  [ADDR_W-1:0]  hp_cint_wr_addr;
    reg                hp_cint_wr_en;
    reg  [255:0]       hp_cint_wr_data;
    reg  [255:0]       hp_cint_buf [0:31];
    reg  [5:0]         hp_cint_words_q;
    reg  [5:0]         hp_cint_flush_idx;
    // ─── SH phase: test message absorption ───
    // A short hardcoded test message (32 bytes = 4 x 64-bit words)
    /*localparam [63:0] TEST_MSG_W0 = 64'h535F4E4F434C4146; // "FALCON_S", Keccak little-endian lane
    localparam [63:0] TEST_MSG_W1 = 64'h545345545F4E4749; // "IGN_TEST"
    localparam [63:0] TEST_MSG_W2 = 64'h2E31565F47534D5F; // "_MSG_V1."
    localparam [63:0] TEST_MSG_W3 = 64'h5F5F5F5F5F5F5F30; // "0_______"*/

    //Our Test Data
    localparam [63:0] TEST_MSG_W0 = 64'h6162636465666768; // "FALCON_S", Keccak little-endian lane
    localparam [63:0] TEST_MSG_W1 = 64'h696a6b6c6d6e6f70; // "IGN_TEST"
    localparam [63:0] TEST_MSG_W2 = 64'h7172737475767778; // "_MSG_V1."
    localparam [63:0] TEST_MSG_W3 = 64'h797a616263646566; // "0_______"

    
    reg [3:0] sh_word_idx;
    localparam [1:0] SHF_IDLE      = 2'd0;
    localparam [1:0] SHF_DRIVE     = 2'd1;
    localparam [1:0] SHF_PULSE     = 2'd2;
    localparam [1:0] SHF_LAST_WAIT = 2'd3;
    reg [1:0] sh_f_state;
    reg       sh_seen_busy;
    reg       sh_done_f;
    wire [3:0] sh_total_words = cfg_msg_only_hash ? 4'd4 : 4'd9;

    // ─── ffSampling EXU: task / memory / SamplerZ passthrough ───
    wire                fe_task_valid;
    wire                fe_task_ready;
    wire                fe_task_done;
    wire                fe_task_fail;
    wire [7:0]          fe_task_status;
    wire [67:0]         fe_task_word;
    wire                fe_mem_rd_en;
    wire                fe_mem_wr_en;
    wire [ADDR_W-1:0]   fe_mem_rd_addr;
    wire [ADDR_W-1:0]   fe_mem_wr_addr;
    wire [255:0]        fe_mem_rd_data;
    wire [255:0]        fe_mem_wr_data;
    wire [ADDR_W-1:0]   fe_twiddle_addr;
    wire [63:0]         fe_twiddle_re;
    wire [63:0]         fe_twiddle_im;
    wire                fe_sz_cmd_valid;
    wire                fe_sz_cmd_ready;
    wire [63:0]         fe_sz_cmd_mu;
    wire [63:0]         fe_sz_cmd_sigma_inv;
    wire                fe_sz_cmd_pair;

    // ─── FFT EXU ───
    wire                fft_cmd_valid;
    wire                fft_cmd_ready;
    wire [2:0]          fft_cmd_opcode;
    wire [4:0]          fft_cmd_logn;
    wire [ADDR_W-1:0]   fft_rd_addr;
    wire [ADDR_W-1:0]   fft_twiddle_addr;
    wire [255:0]        fft_rd_data;
    wire                fft_wr_en;
    wire                fft_rsp_valid;
    wire                fft_rsp_done;
    wire                fft_rsp_fail;
    wire                fft_busy;
    wire [ADDR_W-1:0]   fft_wr_addr;
    wire [255:0]        fft_wr_data;
    wire [63:0]         fft_twiddle_re;
    wire [63:0]         fft_twiddle_im;
    wire [7:0]          fft_rsp_status;
    wire [ADDR_W-1:0]   fft_mem_base = (st == IV) ? LAYOUT_Z_BASE : LAYOUT_FFT_BASE;

    // ─── FFT EXU: 1-BFU (packed 256-bit memory) ───
    // TODO: 2-BFU and 4-BFU need updating for 1R1W packed memory interface
    falcon_f64_fft_exu #(.ADDR_W(ADDR_W)) u_fft(
        .clk(clk),.rst_n(rst_n),
        .cmd_valid(fft_cmd_valid),.cmd_ready(fft_cmd_ready),
        .cmd_opcode(fft_cmd_opcode),.cmd_logn(fft_cmd_logn),
        .mem_rd_addr(fft_rd_addr),
        .mem_rd_data(fft_rd_data),
        .twiddle_addr(fft_twiddle_addr),.twiddle_re(fft_twiddle_re),.twiddle_im(fft_twiddle_im),
        .mem_wr_en(fft_wr_en),.mem_wr_addr(fft_wr_addr),
        .mem_wr_data(fft_wr_data),
        .rsp_valid(fft_rsp_valid),.rsp_done(fft_rsp_done),.rsp_fail(fft_rsp_fail),
        .rsp_status(fft_rsp_status),.busy(fft_busy));

    wire                tg_start;
    wire                tg_start_ready;
    wire                tg_done;
    wire                tg_fail;
    wire [7:0]          tg_status;
    wire                tg_mem_rd_en;
    wire                tg_mem_wr_en;
    wire [ADDR_W-1:0]   tg_mem_rd_addr;
    wire [ADDR_W-1:0]   tg_mem_wr_addr;
    wire [255:0]        tg_mem_wr_data;

    falcon_f64_target_gen_exu #(.ADDR_W(ADDR_W)) u_tg (
        .clk(clk),
        .rst_n(rst_n),
        .start(tg_start),
        .start_ready(tg_start_ready),
        .c_fft_base(LAYOUT_FFT_BASE),
        .t0_base(LAYOUT_T0_BASE),
        .t1_base(LAYOUT_T1_BASE),
        .b01_base(LAYOUT_B01_BASE),
        .b11_base(LAYOUT_B11_BASE),
        .word_count(FALCON_N_WORDS),
        .mem_rd_en(tg_mem_rd_en),
        .mem_rd_addr(tg_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .mem_wr_en(tg_mem_wr_en),
        .mem_wr_addr(tg_mem_wr_addr),
        .mem_wr_data(tg_mem_wr_data),
        .fpu_req_valid(tg_fpu_req_valid),
        .fpu_req_ready(tg_fpu_req_ready),
        .fpu_req_op(tg_fpu_req_op),
        .fpu_req_a(tg_fpu_req_a),
        .fpu_req_b(tg_fpu_req_b),
        .fpu_req_c(tg_fpu_req_c),
        .fpu_rsp_valid(tg_fpu_rsp_valid),
        .fpu_rsp_result(tg_fpu_rsp_result),
        .done(tg_done),
        .fail(tg_fail),
        .status(tg_status)
    );

    // ─── SamplerZ ───
    wire                sz_cmd_valid;
    wire                sz_cmd_ready;
    wire [63:0]         sz_cmd_mu;
    wire [63:0]         sz_cmd_sigma_inv;
    wire [63:0]         sz_cmd_sigma_min;
    wire                sz_cmd_pair;
    wire                sz_rsp_valid;
    wire                sz_rsp_accept;
    wire                sz_busy;
    wire                sz_done;
    wire                sz_fail;
    wire [63:0]         sz_rsp_z0;
    wire [63:0]         sz_rsp_z1;
    wire                sz_rng_req;
    wire                sz_rng_ack;
    wire [255:0]        sz_rng_data;
    falconsign_samplerz_top #(.RNG_DATA_W(256)) u_sz(
        .clk(clk),.rst_n(rst_n),
        .cmd_valid(sz_cmd_valid),.cmd_ready(sz_cmd_ready),
        .cmd_mu(sz_cmd_mu),.cmd_sigma_inv(sz_cmd_sigma_inv),
        .cmd_sigma_min(sz_cmd_sigma_min),.cmd_pair_mode(sz_cmd_pair),
        .rsp_valid(sz_rsp_valid),.rsp_ready(1'b1),.rsp_z0(sz_rsp_z0),.rsp_z1(sz_rsp_z1),
        .rsp_accept(sz_rsp_accept),.rsp_status(),
        .fpu_req_valid(sz_fpu_req_valid),.fpu_req_ready(sz_fpu_req_ready),
        .fpu_req_op(sz_fpu_req_op),.fpu_req_a(sz_fpu_req_a),
        .fpu_req_b(sz_fpu_req_b),.fpu_req_c(sz_fpu_req_c),
        .fpu_req_fmt(),.fpu_req_rm(),.fpu_req_fcvt_op(),
        .fpu_rsp_valid(sz_fpu_rsp_valid),.fpu_rsp_ready(),.fpu_rsp_result(sz_fpu_rsp_result),
        .rng_req(sz_rng_req),.rng_ack(sz_rng_ack),.rng_data(sz_rng_data),
        .busy(sz_busy),.done(sz_done),.fail(sz_fail));

    // ─── ffSampling EXU (SPLIT/MERGE/ADJUST) ───
    falcon_f64_ffsampling_exu #(.ADDR_W(ADDR_W)) u_fe(
        .clk(clk),.rst_n(rst_n),
        .task_valid(fe_task_valid),.task_ready(fe_task_ready),
        .task_word(fe_task_word),.task_done(fe_task_done),.task_fail(fe_task_fail),
        .task_status(fe_task_status),
        .mem_rd_en(fe_mem_rd_en),.mem_rd_addr(fe_mem_rd_addr),
        .mem_rd_data(fe_mem_rd_data),.mem_wr_en(fe_mem_wr_en),
        .mem_wr_addr(fe_mem_wr_addr),.mem_wr_data(fe_mem_wr_data),
        .twiddle_addr(fe_twiddle_addr),.twiddle_re(fe_twiddle_re),.twiddle_im(fe_twiddle_im),
        .fpu_req_valid(fe_fpu_req_valid),.fpu_req_ready(fe_fpu_req_ready),
        .fpu_req_op(fe_fpu_req_op),.fpu_req_a(fe_fpu_req_a),
        .fpu_req_b(fe_fpu_req_b),.fpu_req_c(fe_fpu_req_c),
        .fpu_rsp_valid(fe_fpu_rsp_valid),.fpu_rsp_result(fe_fpu_rsp_result),
        .sz_cmd_valid(fe_sz_cmd_valid),.sz_cmd_ready(fe_sz_cmd_ready),
        .sz_cmd_mu(fe_sz_cmd_mu),.sz_cmd_sigma_inv(fe_sz_cmd_sigma_inv),
        .sz_cmd_pair(fe_sz_cmd_pair),.sz_rsp_valid(sz_rsp_valid),
        .sz_rsp_z0(sz_rsp_z0),.sz_rsp_z1(sz_rsp_z1),
        // Shared FPU lane ports (SPLIT/MERGE via falconsign_shared_fpu_lanes)
        .fe_fpu_req_v(fe_fpu_req_v),.fe_fpu_req_r(fe_fpu_req_r),
        .fe_fpu_mode(fe_fpu_mode),
        .fe_fpu_a0_re(fe_fpu_a0_re),.fe_fpu_a0_im(fe_fpu_a0_im),
        .fe_fpu_b0_re(fe_fpu_b0_re),.fe_fpu_b0_im(fe_fpu_b0_im),
        .fe_fpu_a1_re(fe_fpu_a1_re),.fe_fpu_a1_im(fe_fpu_a1_im),
        .fe_fpu_b1_re(fe_fpu_b1_re),.fe_fpu_b1_im(fe_fpu_b1_im),
        .fe_fpu_w_re(fe_fpu_w_re),.fe_fpu_w_im(fe_fpu_w_im),
        .fe_fpu_w1_re(fe_fpu_w1_re),.fe_fpu_w1_im(fe_fpu_w1_im),
        .fe_fpu_rsp_v(fe_fpu_rsp_v),
        .fe_fpu_y0_re(fe_fpu_y0_re),.fe_fpu_y0_im(fe_fpu_y0_im),
        .fe_fpu_y1_re(fe_fpu_y1_re),.fe_fpu_y1_im(fe_fpu_y1_im),
        .fe_fpu_y0_re_1(fe_fpu_y0_re_1),.fe_fpu_y0_im_1(fe_fpu_y0_im_1),
        .fe_fpu_y1_re_1(fe_fpu_y1_re_1),.fe_fpu_y1_im_1(fe_fpu_y1_im_1));

    // Real B_hat second-component multiply:
    // s2_fft = (t0 - z0) * b01 + (t1 - z1) * b11.
    wire                vd_start;
    wire                vd_start_ready;
    wire                vd_done;
    wire                vd_fail;
    wire [7:0]          vd_status;
    wire                vd_mem_rd_en;
    wire                vd_mem_wr_en;
    wire [ADDR_W-1:0]   vd_mem_rd_addr;
    wire [ADDR_W-1:0]   vd_mem_wr_addr;
    wire [255:0]        vd_mem_wr_data;

    falcon_f64_bhat_mul_exu #(.ADDR_W(ADDR_W)) u_vd (
        .clk(clk),
        .rst_n(rst_n),
        .start(vd_start),
        .start_ready(vd_start_ready),
        .identity_mode(1'b0),
        .t_base(LAYOUT_T0_BASE),
        .z_base(LAYOUT_Z0_BASE),
        .b00_base(LAYOUT_B00_BASE),
        .b01_base(LAYOUT_B01_BASE),
        .b10_base(LAYOUT_B10_BASE),
        .b11_base(LAYOUT_B11_BASE),
        .s2_fft_base(LAYOUT_Z0_BASE),
        .word_count(FALCON_N_WORDS),
        .mem_rd_en(vd_mem_rd_en),
        .mem_rd_addr(vd_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .mem_wr_en(vd_mem_wr_en),
        .mem_wr_addr(vd_mem_wr_addr),
        .mem_wr_data(vd_mem_wr_data),
        .fpu_req_valid(vd_fpu_req_valid),
        .fpu_req_ready(vd_fpu_req_ready),
        .fpu_req_op(vd_fpu_req_op),
        .fpu_req_a(vd_fpu_req_a),
        .fpu_req_b(vd_fpu_req_b),
        .fpu_req_c(vd_fpu_req_c),
        .fpu_rsp_valid(vd_fpu_rsp_valid),
        .fpu_rsp_result(vd_fpu_rsp_result),
        // Shared BFU ports (connect to top-level BFU pool via phase mux)
        .vd_bfu0_in_v(vd_bfu0_in_v), .vd_bfu0_in_r(vd_bfu0_in_r),
        .vd_bfu0_b_re(vd_bfu0_br), .vd_bfu0_b_im(vd_bfu0_bi),
        .vd_bfu0_w_re(vd_bfu0_wr), .vd_bfu0_w_im(vd_bfu0_wi),
        .vd_bfu0_out_v(vd_bfu0_out_v),
        .vd_bfu0_y0r(vd_bfu0_y0r), .vd_bfu0_y0i(vd_bfu0_y0i),
        .vd_bfu1_in_v(vd_bfu1_in_v), .vd_bfu1_in_r(vd_bfu1_in_r),
        .vd_bfu1_b_re(vd_bfu1_br), .vd_bfu1_b_im(vd_bfu1_bi),
        .vd_bfu1_w_re(vd_bfu1_wr), .vd_bfu1_w_im(vd_bfu1_wi),
        .vd_bfu1_out_v(vd_bfu1_out_v),
        .vd_bfu1_y0r(vd_bfu1_y0r), .vd_bfu1_y0i(vd_bfu1_y0i),
        .done(vd_done),
        .fail(vd_fail),
        .status(vd_status)
    );
    wire                ts_start;
    wire                ts_start_ready;
    wire [LEVEL_W-1:0]  ts_cfg_depth;
    wire                ts_cfg_dynamic;
    wire [ADDR_W-1:0]   ts_t_base;
    wire [ADDR_W-1:0]   ts_tree_base;
    wire [ADDR_W-1:0]   ts_z_base;
    wire [ADDR_W-1:0]   ts_tmp_base;
    wire                ts_task_valid;
    wire                ts_task_ready;
    wire [67:0]         ts_task_word;
    wire                ts_task_done;
    wire                ts_task_fail;
    wire                ts_busy;
    wire                ts_done;
    wire                ts_fail;
    wire [7:0]          ts_task_status;
    wire [7:0]          ts_status;
    falconsign_ffsampling_task_update #(.LEVEL_W(LEVEL_W),.INDEX_W(INDEX_W),.ADDR_W(ADDR_W))
    u_ts(.clk(clk),.rst_n(rst_n),
        .start(ts_start),.start_ready(ts_start_ready),
        .cfg_depth(ts_cfg_depth),.cfg_dynamic_tree(ts_cfg_dynamic),
        .cfg_t_base(ts_t_base),.cfg_tree_base(ts_tree_base),.cfg_z_base(ts_z_base),
        .cfg_tmp_base(ts_tmp_base),
        .task_valid(ts_task_valid),.task_ready(ts_task_ready),
        .task_word(ts_task_word),.task_done(ts_task_done),.task_fail(ts_task_fail),
        .task_status(ts_task_status),.busy(ts_busy),.done(ts_done),.fail(ts_fail),.status(ts_status),
        .dbg_level(),.dbg_index(),.dbg_state());

    // ─── Task routing: scheduler → ffSampling EXU ───
    assign sz_cmd_valid    = fe_sz_cmd_valid;
    assign sz_cmd_mu       = fe_sz_cmd_mu;
    assign sz_cmd_sigma_inv = fe_sz_cmd_sigma_inv;
    assign sz_cmd_pair     = fe_sz_cmd_pair;
    assign fe_sz_cmd_ready = sz_cmd_ready;

    // ─── RNG ───
    wire                rng_seed_valid;
    wire                rng_seed_ready;
    wire                rng_valid;
    wire                rng_ready;
    wire [255:0]        rng_seed_key;
    wire [95:0]         rng_seed_nonce;
    wire [511:0]        rng_block;
    falconsign_chacha20_rng u_rng(.clk(clk),.rst_n(rst_n),
        .seed_valid(rng_seed_valid),.seed_ready(rng_seed_ready),
        .seed_key(rng_seed_key),.seed_nonce(rng_seed_nonce),
        .rng_valid(rng_valid),.rng_ready(rng_ready),
        .rng_block(rng_block),.busy());

    // ─── Task routing: scheduler → ffSampling EXU ───
    assign fe_task_valid = ts_task_valid;
    assign fe_task_word  = ts_task_word;
    assign ts_task_ready = fe_task_ready;
    assign ts_task_done  = fe_task_done;
    assign ts_task_fail  = fe_task_fail;
    assign ts_task_status = fe_task_status;

    // ─── Memory Port B: mux between HP-write and ffSampling EXU ───
    wire                fi_start;
    wire                fi_start_ready;
    wire                fi_done;
    wire                fi_fail;
    wire [7:0]          fi_status;
    wire                fi_mem_rd_en;
    wire                fi_mem_wr_en;
    wire [ADDR_W-1:0]   fi_mem_rd_addr;
    wire [ADDR_W-1:0]   fi_mem_wr_addr;
    wire [255:0]        fi_mem_wr_data;

    falconsign_fpr_to_int16 #(.ADDR_W(ADDR_W)) u_fpr_to_i16 (
        .clk(clk),
        .rst_n(rst_n),
        .start(fi_start),
        .start_ready(fi_start_ready),
        .src_base(LAYOUT_Z_BASE),
        .dst_base(LAYOUT_SIG_BASE),
        .coeff_count(FALCON_N_WORDS),
        .mem_rd_en(fi_mem_rd_en),
        .mem_rd_addr(fi_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .mem_wr_en(fi_mem_wr_en),
        .mem_wr_addr(fi_mem_wr_addr),
        .mem_wr_data(fi_mem_wr_data),
        .done(fi_done),
        .fail(fi_fail),
        .status(fi_status)
    );

    wire use_fc_portb   = (st == FC) || (st == IV);  // FFT reads via Port B
    wire use_fe_portb   = (st == FS);
    wire use_tg_portb   = (st == TG);
    wire use_vd_portb   = (st == VD);
    wire use_fi_portb   = (st == FI);
    wire use_norm_portb = (st == RC);
    wire use_ntt_portb  = (st == N1);
    assign fft_rd_data   = mem_b_rd_data;  // FFT reads from Port B
    assign fe_mem_rd_data = mem_b_rd_data;
    wire run_mem_rd_en    = use_fc_portb   ? fft_busy        :
                             use_fe_portb   ? fe_mem_rd_en   :
                             use_tg_portb   ? tg_mem_rd_en   :
                             use_vd_portb   ? vd_mem_rd_en   :
                             use_fi_portb   ? fi_mem_rd_en   :
                             use_norm_portb ? norm_mem_rd_en :
                             use_ntt_portb  ? ntt_mem_rd_en  : 1'b0;
    wire [ADDR_W-1:0] run_mem_rd_addr =
                             use_fc_portb   ? (fft_mem_base + fft_rd_addr) :
                             use_fe_portb   ? fe_mem_rd_addr   :
                             use_tg_portb   ? tg_mem_rd_addr   :
                             use_vd_portb   ? vd_mem_rd_addr   :
                             use_fi_portb   ? fi_mem_rd_addr   :
                             use_norm_portb ? norm_mem_rd_addr :
                             use_ntt_portb  ? ntt_mem_rd_addr  : {ADDR_W{1'b0}};
    // HP write path: combinational data/enable to avoid one-cycle pipeline loss
    wire              use_hp_portb = (st == HP);
    wire              hp_wr_en_comb  = use_hp_portb && htp_coeff_valid;
    wire [255:0]      hp_wr_data_comb = {128'd0, 64'd0, hp_coeff_f64};

    wire hp_cint_flush_wr_en = (st == HP)
        && (hp_wr_addr == (LAYOUT_C_BASE + HTP_C_WORDS))
        && (hp_cint_flush_idx < HTP_C_INT_WORDS);
    wire [ADDR_W-1:0] hp_cint_flush_wr_addr =
        LAYOUT_C_INT_BASE + {{(ADDR_W-6){1'b0}}, hp_cint_flush_idx};
    wire [255:0] hp_cint_flush_wr_data = hp_cint_buf[hp_cint_flush_idx[4:0]];

    wire run_mem_wr_en    = hp_cint_flush_wr_en ? 1'b1 :
                             use_fc_portb   ? fft_wr_en      :
                             use_fe_portb   ? fe_mem_wr_en   :
                             use_tg_portb   ? tg_mem_wr_en   :
                             use_vd_portb   ? vd_mem_wr_en   :
                             use_fi_portb   ? fi_mem_wr_en   :
                             use_hp_portb   ? hp_wr_en_comb  :
                             use_ntt_portb  ? ntt_mem_wr_en  : 1'b0;
    wire [ADDR_W-1:0] run_mem_wr_addr =
                             hp_cint_flush_wr_en ? hp_cint_flush_wr_addr :
                             use_fc_portb   ? (fft_mem_base + fft_wr_addr) :
                             use_fe_portb   ? fe_mem_wr_addr   :
                             use_tg_portb   ? tg_mem_wr_addr   :
                             use_vd_portb   ? vd_mem_wr_addr   :
                             use_fi_portb   ? fi_mem_wr_addr   :
                             use_hp_portb   ? hp_wr_addr       :
                             use_ntt_portb  ? ntt_mem_wr_addr  : {ADDR_W{1'b0}};
    wire [255:0] run_mem_wr_data =
                             hp_cint_flush_wr_en ? hp_cint_flush_wr_data :
                             use_fc_portb   ? fft_wr_data     :
                             use_fe_portb   ? fe_mem_wr_data   :
                             use_tg_portb   ? tg_mem_wr_data   :
                             use_vd_portb   ? vd_mem_wr_data   :
                             use_fi_portb   ? fi_mem_wr_data   :
                             use_hp_portb   ? hp_wr_data_comb  :
                             use_ntt_portb  ? ntt_mem_wr_data  : 256'd0;

    localparam [1:0] BUSM_IDLE  = 2'd0;
    localparam [1:0] BUSM_READ  = 2'd1;
    localparam [1:0] BUSM_CAP   = 2'd2;
    localparam [1:0] BUSM_WRITE = 2'd3;
    reg [1:0]        bus_mem_state;
    reg              bus_mem_wr_q;
    reg [ADDR_W-1:0] bus_mem_word_addr_q;
    reg [2:0]        bus_mem_lane_q;
    reg [31:0]       bus_mem_wdata_q;
    reg [255:0]      bus_mem_wr_word_q;

    wire             bus_mem_rd_en = (bus_mem_state == BUSM_READ);
    wire             bus_mem_wr_en = (bus_mem_state == BUSM_WRITE);
    wire [255:0]     bus_mem_lane_mask = ({224'd0, 32'hFFFF_FFFF} << (bus_mem_lane_q * 32));
    wire [255:0]     bus_mem_lane_data = ({224'd0, bus_mem_wdata_q} << (bus_mem_lane_q * 32));

    assign mem_rd_en   = bus_mem_rd_en ? 1'b1 : run_mem_rd_en;
    assign mem_rd_addr = bus_mem_rd_en ? bus_mem_word_addr_q : run_mem_rd_addr;
    assign mem_wr_en   = bus_mem_wr_en ? 1'b1 : run_mem_wr_en;
    assign mem_wr_addr = bus_mem_wr_en ? bus_mem_word_addr_q : run_mem_wr_addr;
    assign mem_wr_data = bus_mem_wr_en ? bus_mem_wr_word_q : run_mem_wr_data;

    // ─── Port A: FFT writes (packed 256-bit pairs) ───
    // During FFT: Port A = writes, Port B = reads (double-buffered, no data hazard)
    // ─── Twiddle ROM ───
    wire [7:0] twiddle_rom_addr = (st == FS) ? fe_twiddle_addr[7:0] : fft_twiddle_addr[7:0];
    wire [63:0] twiddle_rom_re;
    wire [63:0] twiddle_rom_im;
    wire [63:0] gm_rom_re;
    wire [63:0] gm_rom_im;
    falconsign_twiddle_rom #(.ADDR_W(8),.DEPTH(256)) u_tw(
        .clk(clk),.addr(twiddle_rom_addr),
        .twiddle_re(twiddle_rom_re),.twiddle_im(twiddle_rom_im));
    falconsign_gm_rom #(.ADDR_W(8),.DEPTH(255)) u_gm(
        .clk(clk),
        .addr(fe_twiddle_addr[7:0]),
        .gm_re(gm_rom_re),.gm_im(gm_rom_im));
    assign fft_twiddle_re = twiddle_rom_re;
    assign fft_twiddle_im = twiddle_rom_im;
    assign fe_twiddle_re  = gm_rom_re;
    assign fe_twiddle_im  = gm_rom_im;

    // ─── NTT (s1 = c - s2*h mod q) ───
    wire                ntt_start;
    wire                ntt_ready;
    wire                ntt_done;
    wire                ntt_fail;
    wire [7:0]          ntt_status;
    wire                ntt_mem_rd_en;
    wire                ntt_mem_wr_en;
    wire [ADDR_W-1:0]   ntt_mem_rd_addr;
    wire [ADDR_W-1:0]   ntt_mem_wr_addr;
    wire [255:0]        ntt_mem_wr_data;
    wire [8:0]  ntt_twiddle_rom_addr;
    wire [13:0] ntt_twiddle_rom_data;
    wire [9:0]  ntt_psi_rom_addr;
    wire [13:0] ntt_psi_rom_data;

    // NTT twiddle ROM (512 x 14)
    falconsign_ntt_twiddle_rom #(.ADDR_W(9)) u_ntt_tw (
        .clk(clk), .addr(ntt_twiddle_rom_addr), .data(ntt_twiddle_rom_data));

    // NTT psi table ROM (1024 x 14)
    falconsign_ntt_psi_rom #(.ADDR_W(10)) u_ntt_psi (
        .clk(clk), .addr(ntt_psi_rom_addr), .data(ntt_psi_rom_data));

    // NTT EXU
    falconsign_ntt_exu #(.ADDR_W(ADDR_W)) u_ntt (
        .clk(clk), .rst_n(rst_n),
        .start(ntt_start), .start_ready(ntt_ready),
        .done(ntt_done), .fail(ntt_fail), .status(ntt_status),
        .cfg_h_base(LAYOUT_H_BASE),
        .cfg_h_work_base(LAYOUT_H_WORK_BASE),
        .cfg_s2_base(LAYOUT_SIG_BASE),
        .cfg_s2_work_base(LAYOUT_Z1_BASE),
        .cfg_c_base(LAYOUT_C_INT_BASE),
        .cfg_dst_base(LAYOUT_S1_BASE),
        .mem_rd_en(ntt_mem_rd_en), .mem_rd_addr(ntt_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .mem_wr_en(ntt_mem_wr_en), .mem_wr_addr(ntt_mem_wr_addr),
        .mem_wr_data(ntt_mem_wr_data),
        .twiddle_rom_addr(ntt_twiddle_rom_addr),
        .twiddle_rom_data(ntt_twiddle_rom_data),
        .psi_rom_addr(ntt_psi_rom_addr),
        .psi_rom_data(ntt_psi_rom_data));

    assign ntt_start = (st == FI) && (sn == N1);

    // ─── Norm check / rejection check ───
    wire                norm_start;
    wire                norm_start_ready;
    wire                norm_done;
    wire                norm_accept;
    wire                norm_fail;
    wire [7:0]          norm_status;
    wire                norm_mem_rd_en;
    wire [ADDR_W-1:0] norm_mem_rd_addr;
    wire [63:0] norm_sq;
    localparam [63:0] FALCON512_BOUND_SQ = 64'd34034726;
    localparam [7:0]  MAX_RESTARTS = 8'd3;

    falconsign_norm_i16_sig_check #(.ADDR_W(ADDR_W)) u_norm (
        .clk(clk),
        .rst_n(rst_n),
        .start(norm_start),
        .start_ready(norm_start_ready),
        .s2_base(LAYOUT_SIG_BASE),
        .s1_base(LAYOUT_S1_BASE),
        .word_count(LAYOUT_NORM_WORDS),
        .bound_sq(FALCON512_BOUND_SQ),
        .mem_rd_en(norm_mem_rd_en),
        .mem_rd_addr(norm_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .done(norm_done),
        .accept(norm_accept),
        .fail(norm_fail),
        .status(norm_status),
        .norm_sq(norm_sq)
    );

    // ─── Restart / rejection ───
    reg [7:0] salt_cnt;      // incremented on each restart (RC reject)
    reg       rc_fail;       // RC rejection flag

    // ─── Config ───
    assign ts_cfg_depth   = FALCON_LOGN[LEVEL_W-1:0];
    assign ts_cfg_dynamic = cfg_dynamic_tree;
    assign ts_t_base      = LAYOUT_T_BASE;
    assign ts_tree_base   = LAYOUT_TREE_BASE;
    assign ts_z_base      = LAYOUT_Z_BASE;
    assign ts_tmp_base    = LAYOUT_TMP_BASE;
    assign ts_start       = (st != FS) && (sn == FS);
`ifndef SYNTHESIS
    integer debug_rng_nonce_base;
    initial begin
        debug_rng_nonce_base = 0;
        if (!$value$plusargs("RNG_NONCE=%d", debug_rng_nonce_base)) begin
            debug_rng_nonce_base = 0;
        end
    end
    wire [7:0] rng_nonce_lo = salt_cnt + debug_rng_nonce_base[7:0];
`else
    wire [7:0] rng_nonce_lo = salt_cnt;
`endif
    assign rng_seed_valid = (st == SH) || ((st == SI) && (sn == FS));
    assign rng_seed_key   = 256'h0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF;
    //assign rng_seed_nonce = {88'd0, rng_nonce_lo};
    assign rng_seed_nonce = 96'd0;  // In our testdata, we fix the nonce to be 0
    assign sz_rng_data    = rng_block[255:0];
    assign sz_rng_ack     = rng_valid && sz_rng_req;
    assign rng_ready      = sz_rng_req;
    assign sz_cmd_sigma_min = 64'h3FF47201BF1F7A75; // fpr_sigma_min[9]
    assign busy           = (st != SI) && (st != SD);
    wire bus_is_reg = (bus_addr == REG_CR) ||
                      (bus_addr == REG_SR) ||
                      (bus_addr == REG_CFG) ||
                      (bus_addr == REG_MEM_HI);

    // ─── phase control signals ───
    wire sh_done;  // SH: SHAKE absorb complete
    wire hp_done_sig; // HP: all N coeffs written

    // ─── FFT command ───
    // Guard: deassert cmd_valid when rsp_done is active to prevent the 2-BFU EXU
    // from re-starting. The 2-BFU EXU uses registered rsp_done_r which goes high
    // when state has already transitioned to IDLE (cmd_ready=1), causing an
    // unintended re-trigger if cmd_valid is still asserted.
    assign fft_cmd_valid  = ((st == FC) || (st == IV)) && !fft_rsp_done;
    assign fft_cmd_opcode = (st == IV) ? 3'd2 : 3'd3;  // Falcon half-complex FWD / INV
    assign fft_cmd_logn   = FALCON_LOGN;

    // ─── SHAKE256 absorb control (SH phase) ───
    // One-cycle pulse on SI→SH or RC→SH (restart) transition
    assign shake_start = ((st == SI) && (sn == SH)) || ((st == RC) && (sn == SH));

    // ─── HashToPoint control (HP phase) ───
    // One-cycle pulse on SH→HP transition
    assign htp_start = (st == SH) && (sn == HP);
    assign tg_start = (st == FC) && (sn == TG);
    assign vd_start = ((st == FS) && (sn == VD)) ||
                      (cfg_bypass_fs && (st == FC) && (sn == VD)) ||
                      (cfg_start_at_fs && cfg_bypass_fs && (st == SI) && (sn == VD));
    assign fi_start = (st == IV) && (sn == FI);
    assign norm_start = (st == N1) && (sn == RC);

    // ─── SH / HP done detection ───
    // SH done: absorbed all 4 words AND SHAKE has processed them (back in ready state after last permutation)
    // We detect SH done when: word_idx == 4 (all words sent) AND shake is back in absorb-ready state
    // Simple approach: done when word_idx == 4 and shake_ready (it finishes padding permute)
    assign sh_done = sh_done_f;

    // HP done: all 32 writes completed (32 × 16 = 512 coefficients)
    // HP done: all N coefficients written (hp_wr_addr counts 0..511 then stops at 512)
    assign hp_done_sig = (st == HP)
        && (hp_wr_addr == (LAYOUT_C_BASE + HTP_C_WORDS))
        && (hp_cint_flush_idx == HTP_C_INT_WORDS);

    // ─── Bus & FSM ───
    reg bus_pend, bus_pend_wr; reg[15:0] bus_pend_addr; reg[31:0] bus_pend_data;
    // Top-level signing phase FSM. It sequences hash, FFT, ffSampling,
    // BhatMul, IFFT, integer conversion, NTT reconstruction and norm check.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= SI; cr_start <= 0; bus_rdata <= 0; bus_ready <= 0; bus_irq <= 0;
            done <= 0; fail <= 0; status <= 0; bus_pend <= 0; bus_pend_wr <= 0;
            bus_pend_addr <= 0; bus_pend_data <= 0;
            bus_mem_state <= BUSM_IDLE;
            bus_mem_wr_q <= 1'b0;
            bus_mem_word_addr_q <= {ADDR_W{1'b0}};
            bus_mem_lane_q <= 3'd0;
            bus_mem_wdata_q <= 32'd0;
            bus_mem_wr_word_q <= 256'd0;
            cfg_bypass_fs <= 1'b0;
            cfg_force_accept <= 1'b0;
            cfg_start_at_fs <= 1'b0;
            cfg_dynamic_tree <= 1'b0;
            cfg_msg_only_hash <= 1'b0;
            mem_addr_hi <= 2'd0;
            sh_word_idx <= 0;
            sh_f_state <= SHF_IDLE;
            sh_seen_busy <= 1'b0;
            sh_done_f <= 1'b0;
            shake_absorb <= 0; shake_din <= 0; shake_din_last <= 0;
            hp_coeff_cnt <= 0; hp_coeff_buf <= 0;
            hp_wr_addr <= LAYOUT_C_BASE; hp_wr_en <= 0; hp_wr_data <= 0;
            hp_cint_wr_addr <= LAYOUT_C_INT_BASE; hp_cint_wr_en <= 0; hp_cint_wr_data <= 0;
            hp_cint_words_q <= 6'd0;
            hp_cint_flush_idx <= 6'd0;
            salt_cnt <= 0;
            rc_fail <= 0;
        end else begin
            st <= sn; bus_ready <= 0;
            if (bus_cs && !bus_pend && (bus_mem_state == BUSM_IDLE)) begin
                if (bus_is_reg) begin
                    bus_ready <= 1'b1;
                    if (bus_wr) begin
                        case (bus_addr)
                            REG_CR: begin
                                cr_start <= bus_wdata[0];
                                if (st == SD) bus_irq <= 1'b0;
                            end
                            REG_CFG: begin
                                cfg_bypass_fs    <= bus_wdata[0];
                                cfg_force_accept <= bus_wdata[1];
                                cfg_start_at_fs  <= bus_wdata[2];
                                cfg_dynamic_tree <= bus_wdata[3];
                                cfg_msg_only_hash <= bus_wdata[4];
                            end
                            REG_MEM_HI: begin
                                mem_addr_hi <= bus_wdata[1:0];
                            end
                            default: begin
                            end
                        endcase
                    end else begin
                        case (bus_addr)
                            REG_CR:     bus_rdata <= {31'd0, cr_start};
                            REG_SR:     bus_rdata <= {16'd0, status, 4'd0, fail, done, bus_irq, busy};
                            REG_CFG:    bus_rdata <= {27'd0, cfg_msg_only_hash,
                                                       cfg_dynamic_tree, cfg_start_at_fs,
                                                       cfg_force_accept, cfg_bypass_fs};
                            REG_MEM_HI: bus_rdata <= {30'd0, mem_addr_hi};
                            default:    bus_rdata <= 32'd0;
                        endcase
                    end
                end else begin
                    if (!busy) begin
                        bus_mem_wr_q <= bus_wr;
                        bus_mem_word_addr_q <= {mem_addr_hi, bus_addr[15:5]};
                        bus_mem_lane_q <= bus_addr[4:2];
                        bus_mem_wdata_q <= bus_wdata;
                        bus_mem_state <= BUSM_READ;
                    end else begin
                        bus_ready <= 1'b1;
                        if (!bus_wr)
                            bus_rdata <= 32'd0;
                    end
                end
            end
            case (bus_mem_state)
                BUSM_READ: begin
                    bus_mem_state <= BUSM_CAP;
                end
                BUSM_CAP: begin
                    if (bus_mem_wr_q) begin
                        bus_mem_wr_word_q <= (mem_rd_data & ~bus_mem_lane_mask) | bus_mem_lane_data;
                        bus_mem_state <= BUSM_WRITE;
                    end else begin
                        bus_rdata <= mem_rd_data[bus_mem_lane_q * 32 +: 32];
                        bus_ready <= 1'b1;
                        bus_mem_state <= BUSM_IDLE;
                    end
                end
                BUSM_WRITE: begin
                    bus_ready <= 1'b1;
                    bus_mem_state <= BUSM_IDLE;
                end
                default: begin
                    bus_mem_state <= BUSM_IDLE;
                end
            endcase
            case (st)
                SI: begin done<=0; fail<=0; bus_irq<=0; if (sn != SI) cr_start<=0; end
                SD: begin done<=1; bus_irq<=1;
                    if (bus_cs && bus_wr && bus_addr==REG_CR) bus_irq<=0; end
            endcase

            // ─── SH phase: absorb test message into SHAKE256 ───
            if (st == SH && sn == SH) begin
                shake_absorb <= 1'b0;
                case (sh_f_state)
                    SHF_IDLE: begin
                        sh_f_state <= SHF_DRIVE;
                    end
                    SHF_DRIVE: begin
                        if (shake_ready) begin
                            shake_din_last <= (sh_word_idx == (sh_total_words - 4'd1));
                    if (!cfg_msg_only_hash && (sh_word_idx < 4'd5)) begin
                        shake_din <= 64'd0;
                    end else if (cfg_msg_only_hash) begin
                        case (sh_word_idx[1:0])
                            2'd0: shake_din <= TEST_MSG_W0;
                            2'd1: shake_din <= TEST_MSG_W1;
                            2'd2: shake_din <= TEST_MSG_W2;
                            default: shake_din <= TEST_MSG_W3;
                        endcase
                    end else begin
                        case (sh_word_idx)
                            4'd5: shake_din <= TEST_MSG_W0;
                            4'd6: shake_din <= TEST_MSG_W1;
                            4'd7: shake_din <= TEST_MSG_W2;
                            default: shake_din <= TEST_MSG_W3;
                        endcase
                    end
                            sh_f_state <= SHF_PULSE;
                        end
                    end
                    SHF_PULSE: begin
                        shake_absorb <= 1'b1;
                        sh_word_idx <= sh_word_idx + 4'd1;
                        if (sh_word_idx == (sh_total_words - 4'd1))
                            sh_f_state <= SHF_LAST_WAIT;
                        else
                            sh_f_state <= SHF_DRIVE;
                    end
                    SHF_LAST_WAIT: begin
                        shake_absorb <= 1'b0;
                        if (!shake_ready)
                            sh_seen_busy <= 1'b1;
                        else if (sh_seen_busy) begin
                            sh_done_f <= 1'b1;
                            sh_f_state <= SHF_IDLE;
                        end
                    end
                    default: begin
                        sh_f_state <= SHF_IDLE;
                    end
                endcase
            end

            // HP phase: squeeze SHAKE -> HashToPoint -> write FFT-ready FP64 complex words.
            if (st == HP && sn == HP) begin
                shake_absorb <= 0;

                hp_cint_wr_en <= 0;
                if (htp_coeff_valid) begin
                    hp_wr_addr <= hp_wr_addr + 1'b1;
                    if (hp_coeff_cnt == 4'd15) begin
                        hp_cint_buf[hp_cint_words_q[4:0]] <= {htp_coeff, hp_coeff_buf[239:0]};
                        hp_cint_words_q <= hp_cint_words_q + 1'b1;
                        hp_coeff_cnt    <= 4'd0;
                        hp_coeff_buf    <= 256'd0;
                    end else begin
                        hp_coeff_buf[(hp_coeff_cnt * 16) +: 16] <= htp_coeff;
                        hp_coeff_cnt <= hp_coeff_cnt + 1'b1;
                    end
                end else if ((hp_wr_addr == (LAYOUT_C_BASE + HTP_C_WORDS))
                        && (hp_cint_flush_idx < HTP_C_INT_WORDS)) begin
                    hp_cint_flush_idx <= hp_cint_flush_idx + 1'b1;
                end
            end

            // ─── RC phase: determine pass/fail ───
            if (st == RC && norm_done) begin
                rc_fail <= cfg_force_accept ? 1'b0 : !norm_accept;
                status  <= cfg_force_accept ? 8'h00 : norm_status;
                if (!cfg_force_accept && !norm_accept && (salt_cnt >= MAX_RESTARTS)) begin
                    fail <= 1'b1;
                    status <= 8'h21;
                end
            end
            if (st == VD && vd_done && vd_fail) begin
                fail   <= 1'b1;
                status <= vd_status;
            end
            if (st == FS && ts_fail) begin
                fail   <= 1'b1;
                status <= ts_status;
            end
            if (st == TG && tg_done && tg_fail) begin
                fail   <= 1'b1;
                status <= tg_status;
            end
            if (st == FI && fi_done && fi_fail) begin
                fail   <= 1'b1;
                status <= fi_status;
            end

            // Reset HP counters on entry to SH (new signing operation or restart)
            if ((st == SI && sn == SH) || (st == RC && sn == SH)) begin
                hp_wr_addr   <= LAYOUT_C_BASE;
                hp_cint_wr_addr <= LAYOUT_C_INT_BASE;
                hp_coeff_cnt <= 0;
                hp_wr_en     <= 0;
                hp_cint_wr_en <= 0;
                hp_coeff_buf <= 0;
                hp_cint_words_q <= 6'd0;
                hp_cint_flush_idx <= 6'd0;
                sh_word_idx  <= 0;
                sh_f_state <= SHF_IDLE;
                sh_seen_busy <= 1'b0;
                sh_done_f <= 1'b0;
                shake_absorb <= 0;
                if (st == RC && sn == SH) salt_cnt <= salt_cnt + 1;
            end

            // At SH→HP transition, reset hp_wr_addr for the new HP phase
            if ((st == SH) && (sn == HP)) begin
                hp_wr_addr <= LAYOUT_C_BASE;
                hp_cint_wr_addr <= LAYOUT_C_INT_BASE;
                hp_coeff_cnt <= 0;
                hp_coeff_buf <= 0;
                hp_cint_words_q <= 6'd0;
                hp_cint_flush_idx <= 6'd0;
            end
        end
    end

    // ─── Phase FSM (next state) ───
    // Port-C read mux for register space versus memory/debug address space.
    always @(*) begin
        sn = st;
        case (st)
            SI: if (cr_start) sn = cfg_start_at_fs ? (cfg_bypass_fs ? VD : FS) : SH;
            SH: if (sh_done)  sn = HP;
            HP: if (hp_done_sig) sn = FC;
            FC: if (fft_rsp_done) sn = cfg_bypass_fs ? VD : TG;
            TG: if (tg_done)      sn = tg_fail ? SD : FS;
            FS: if (ts_fail)     sn = SD;
                else if (ts_done) sn = VD;
            VD: if (vd_done)     sn = vd_fail ? SD : IV;
            IV: if (fft_rsp_done) sn = FI;
            FI: if (fi_done)     sn = fi_fail ? SD : N1;
            N1: if (ntt_done)    sn = ntt_fail ? SD : RC;
            RC: begin
                if (norm_done) begin
                    if (cfg_force_accept) begin
                        sn = CN;
                    end else if (!norm_accept && (salt_cnt >= MAX_RESTARTS)) begin
                        sn = SD;
                    end else if (!norm_accept) begin
                        sn = SH;  // restart with new salt
                    end else begin
                        sn = CN;
                    end
                end
            end
            CN: if (skel_timer > 8'd5) sn = EN;
            EN: if (skel_timer > 8'd5) sn = OU;
            OU: if (skel_timer > 8'd3) sn = SD;
            SD: if (bus_cs && bus_wr && (bus_addr == REG_CR)) sn = SI;
            default: sn = SI;
        endcase
    end

    reg [7:0] skel_timer;
    // Debug and performance counters for phase timing, restarts and SamplerZ
    // rejection statistics.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) skel_timer <= 0;
        else if (st != sn) skel_timer <= 0;
        else skel_timer <= skel_timer + 1;
    end

endmodule
