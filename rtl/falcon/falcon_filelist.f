// ============================================================================
// Falcon-512 Post-Quantum Signature — Vivado Synthesis Filelist
// Target: xc7k70tfbv484-2 (Kintex-7)
// Generated: 2026-06-01
//
// Usage (Vivado Tcl):
//   set file_list [read_filelist "falcon_filelist.f"]
//   foreach f $file_list { read_verilog -sv $f }
//
// Usage (Vivado .xpr):
//   Add all files below to the project sources.
// ============================================================================

// ─── Arithmetic Primitives ───
rtl/falcon/falcon_f64_add.v
rtl/falcon/falcon_f64_mul.v
rtl/falcon/falcon_fp_fpu.v

// ─── FFT Butterfly Units ───
rtl/falcon/falcon_f64_complex_bfly.v
rtl/falcon/falcon_f64_complex_bfly4.v

// ─── Shared FPU Lanes ───
rtl/falcon/falconsign_shared_fpu_lanes.v

// ─── FFT Execution Units ───
rtl/falcon/falcon_f64_fft_exu.v
rtl/falcon/falcon_f64_fft_exu_2bfu.v
rtl/falcon/falcon_f64_fft_exu_4bfu.v
rtl/falcon/falcon_fft_addr_gen_cfg.v

// ─── ROM Tables ───
rtl/falcon/falconsign_twiddle_rom.v
rtl/falcon/falconsign_gm_rom.v
rtl/falcon/falconsign_ntt_twiddle_rom.v
rtl/falcon/falconsign_ntt_psi_rom.v

// ─── SHA-3 / SHAKE / RNG ───
rtl/falcon/falconsign_keccak_core.v
rtl/falcon/falconsign_shake256.v
rtl/falcon/falconsign_chacha20_rng.v
rtl/falcon/falconsign_word_fifo.v

// ─── Hash-to-Point ───
rtl/falcon/falconsign_hash_to_point.v

// ─── Target Generation ───
rtl/falcon/falcon_f64_target_gen_exu.v

// ─── SamplerZ ───
rtl/falcon/falconsign_samplerz_top.v

// ─── ffSampling ───
rtl/falcon/falcon_f64_ffsampling_exu.v
rtl/falcon/falconsign_ffsampling_task_update.v

// ─── BhatMul / Verify Decode ───
rtl/falcon/falcon_f64_bhat_mul_exu.v
rtl/falcon/falconsign_sig_decode.v

// ─── NTT ───
rtl/falcon/falconsign_ntt_bfly.v
rtl/falcon/falconsign_ntt_cg_addr.v
rtl/falcon/falconsign_ntt_exu.v

// ─── fpr_to_int16 / Norm Check ───
rtl/falcon/falconsign_fpr_to_int16.v
rtl/falcon/falconsign_norm_i16_sig_check.v

// ─── Memory ───
rtl/falcon/falconsign_memory.v

// ─── Verify Top ───
rtl/falcon/falconsign_verify_top.v

// ─── Top Level ───
rtl/falcon/falconsign_top.v
rtl/falcon/falconsign_norm_i16_check.v
