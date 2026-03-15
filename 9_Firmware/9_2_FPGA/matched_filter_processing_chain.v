`timescale 1ns / 1ps

/**
 * matched_filter_processing_chain.v
 *
 * Pulse compression processing chain for AERIS-10 FMCW radar.
 * Implements: FFT(signal) → FFT(reference) → Conjugate multiply → IFFT
 *
 * This is a SIMULATION-COMPATIBLE implementation that replaces the Xilinx
 * FFT IP cores (FFT_enhanced) with behavioral Radix-2 DIT FFT engines.
 * For synthesis, replace the behavioral FFT instances with the actual
 * Xilinx xfft IP blocks.
 *
 * Interface contract (from matched_filter_multi_segment.v line 361):
 *   .clk, .reset_n
 *   .adc_data_i, .adc_data_q, .adc_valid      <- from input buffer
 *   .chirp_counter                              <- 6-bit frame counter
 *   .long_chirp_real/imag, .short_chirp_real/imag <- reference (time-domain)
 *   .range_profile_i, .range_profile_q, .range_profile_valid -> output
 *   .chain_state                                -> 4-bit status
 *
 * Clock domain: clk (100 MHz system clock)
 * Data format:  16-bit signed (Q15 fixed-point)
 * FFT size:     1024 points
 *
 * Pipeline states:
 *   IDLE -> FWD_FFT (collect 1024 samples + bit-reverse copy)
 *        -> FWD_BUTTERFLY (forward FFT of signal)
 *        -> REF_BITREV (bit-reverse copy reference into work arrays)
 *        -> REF_BUTTERFLY (forward FFT of reference)
 *        -> MULTIPLY (conjugate multiply in freq domain)
 *        -> INV_BITREV (bit-reverse copy product)
 *        -> INV_BUTTERFLY (inverse FFT + 1/N scaling)
 *        -> OUTPUT (stream 1024 samples)
 *        -> DONE -> IDLE
 */

module matched_filter_processing_chain (
    input wire clk,
    input wire reset_n,

    // Input ADC data (from matched_filter_multi_segment buffer)
    input wire [15:0] adc_data_i,
    input wire [15:0] adc_data_q,
    input wire adc_valid,

    // Chirp counter (for future multi-chirp modes)
    input wire [5:0] chirp_counter,

    // Reference chirp (time-domain, latency-aligned by upstream buffer)
    input wire [15:0] long_chirp_real,
    input wire [15:0] long_chirp_imag,
    input wire [15:0] short_chirp_real,
    input wire [15:0] short_chirp_imag,

    // Output: range profile (pulse-compressed)
    output wire signed [15:0] range_profile_i,
    output wire signed [15:0] range_profile_q,
    output wire range_profile_valid,

    // Status
    output wire [3:0] chain_state
);

// ============================================================================
// PARAMETERS
// ============================================================================
localparam FFT_SIZE   = 1024;
localparam ADDR_BITS  = 10;    // log2(1024)

// State encoding (4-bit, up to 16 states)
localparam [3:0] ST_IDLE           = 4'd0;
localparam [3:0] ST_FWD_FFT        = 4'd1;   // Collect samples + bit-reverse
localparam [3:0] ST_FWD_BUTTERFLY  = 4'd2;   // Signal FFT butterflies
localparam [3:0] ST_REF_BITREV     = 4'd3;   // Bit-reverse copy reference
localparam [3:0] ST_REF_BUTTERFLY  = 4'd4;   // Reference FFT butterflies
localparam [3:0] ST_MULTIPLY       = 4'd5;   // Conjugate multiply
localparam [3:0] ST_INV_BITREV     = 4'd6;   // Bit-reverse copy product
localparam [3:0] ST_INV_BUTTERFLY  = 4'd7;   // IFFT butterflies + scale
localparam [3:0] ST_OUTPUT         = 4'd8;   // Stream results
localparam [3:0] ST_DONE           = 4'd9;   // Return to idle

reg [3:0] state;

// ============================================================================
// SIGNAL BUFFERS
// ============================================================================
// Input sample counter
reg [ADDR_BITS:0] fwd_in_count;     // 0..1024
reg fwd_frame_done;                  // All 1024 samples received

// Signal time-domain buffer
reg signed [15:0] fwd_buf_i [0:FFT_SIZE-1];
reg signed [15:0] fwd_buf_q [0:FFT_SIZE-1];

// Signal FFT output (frequency domain)
reg signed [15:0] fwd_out_i [0:FFT_SIZE-1];
reg signed [15:0] fwd_out_q [0:FFT_SIZE-1];
reg fwd_out_valid;

// Reference time-domain buffer
reg signed [15:0] ref_buf_i [0:FFT_SIZE-1];
reg signed [15:0] ref_buf_q [0:FFT_SIZE-1];

// Reference FFT output (frequency domain)
reg signed [15:0] ref_fft_i [0:FFT_SIZE-1];
reg signed [15:0] ref_fft_q [0:FFT_SIZE-1];

// ============================================================================
// CONJUGATE MULTIPLY OUTPUT
// ============================================================================
reg signed [15:0] mult_out_i [0:FFT_SIZE-1];
reg signed [15:0] mult_out_q [0:FFT_SIZE-1];
reg mult_done;

// ============================================================================
// INVERSE FFT OUTPUT
// ============================================================================
reg signed [15:0] ifft_out_i [0:FFT_SIZE-1];
reg signed [15:0] ifft_out_q [0:FFT_SIZE-1];
reg ifft_done;

// Output streaming
reg [ADDR_BITS:0] out_count;
reg out_valid_reg;
reg signed [15:0] out_i_reg, out_q_reg;

// ============================================================================
// BEHAVIORAL RADIX-2 DIT FFT (simulation only)
// ============================================================================
// Working arrays for FFT computation (shared between fwd, ref, and inv FFTs)
reg signed [31:0] work_re [0:FFT_SIZE-1];
reg signed [31:0] work_im [0:FFT_SIZE-1];

// Bit-reverse function
function [ADDR_BITS-1:0] bit_reverse;
    input [ADDR_BITS-1:0] val;
    integer b;
    begin
        bit_reverse = 0;
        for (b = 0; b < ADDR_BITS; b = b + 1)
            bit_reverse[ADDR_BITS-1-b] = val[b];
    end
endfunction

// FFT computation variables
integer fft_stage, fft_k, fft_j, fft_half, fft_span;
integer fft_idx_even, fft_idx_odd;
reg signed [31:0] tw_re, tw_im;
reg signed [31:0] t_re, t_im;
reg signed [31:0] u_re, u_im;
real tw_angle;

// ============================================================================
// MAIN STATE MACHINE
// ============================================================================
integer i;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state          <= ST_IDLE;
        fwd_in_count   <= 0;
        fwd_frame_done <= 0;
        fwd_out_valid  <= 0;
        mult_done      <= 0;
        ifft_done      <= 0;
        out_count      <= 0;
        out_valid_reg  <= 0;
        out_i_reg      <= 16'd0;
        out_q_reg      <= 16'd0;
    end else begin
        // Defaults
        out_valid_reg <= 1'b0;

        case (state)
        // ================================================================
        // IDLE: Wait for valid ADC data, start collecting 1024 samples
        // ================================================================
        ST_IDLE: begin
            fwd_in_count   <= 0;
            fwd_frame_done <= 0;
            fwd_out_valid  <= 0;
            mult_done      <= 0;
            ifft_done      <= 0;
            out_count      <= 0;

            if (adc_valid) begin
                // Store first sample (signal + reference)
                fwd_buf_i[0] <= $signed(adc_data_i);
                fwd_buf_q[0] <= $signed(adc_data_q);
                ref_buf_i[0] <= $signed(long_chirp_real);
                ref_buf_q[0] <= $signed(long_chirp_imag);
                fwd_in_count <= 1;
                state        <= ST_FWD_FFT;
            end
        end

        // ================================================================
        // FWD_FFT: Collect remaining samples, then bit-reverse copy signal
        // ================================================================
        ST_FWD_FFT: begin
            if (!fwd_frame_done) begin
                // Still collecting samples
                if (adc_valid && fwd_in_count < FFT_SIZE) begin
                    fwd_buf_i[fwd_in_count] <= $signed(adc_data_i);
                    fwd_buf_q[fwd_in_count] <= $signed(adc_data_q);
                    ref_buf_i[fwd_in_count] <= $signed(long_chirp_real);
                    ref_buf_q[fwd_in_count] <= $signed(long_chirp_imag);
                    fwd_in_count <= fwd_in_count + 1;
                end

                if (fwd_in_count == FFT_SIZE) begin
                    fwd_frame_done <= 1;

                    // Bit-reverse copy SIGNAL into work arrays (via <=)
                    for (i = 0; i < FFT_SIZE; i = i + 1) begin
                        work_re[bit_reverse(i[ADDR_BITS-1:0])] <= {{16{fwd_buf_i[i][15]}}, fwd_buf_i[i]};
                        work_im[bit_reverse(i[ADDR_BITS-1:0])] <= {{16{fwd_buf_q[i][15]}}, fwd_buf_q[i]};
                    end
                end
            end else begin
                // Bit-reverse copy settled on previous clock.
                // Now transition to butterfly computation.
                state <= ST_FWD_BUTTERFLY;
            end
        end

        // ================================================================
        // FWD_BUTTERFLY: Forward FFT of signal (all stages, simulation only)
        // ================================================================
        ST_FWD_BUTTERFLY: begin
            // In-place radix-2 DIT butterflies (blocking assignments)
            for (fft_stage = 0; fft_stage < ADDR_BITS; fft_stage = fft_stage + 1) begin
                fft_half = 1 << fft_stage;
                fft_span = fft_half << 1;
                for (fft_k = 0; fft_k < FFT_SIZE; fft_k = fft_k + fft_span) begin
                    for (fft_j = 0; fft_j < fft_half; fft_j = fft_j + 1) begin
                        fft_idx_even = fft_k + fft_j;
                        fft_idx_odd  = fft_idx_even + fft_half;

                        tw_angle = -2.0 * 3.14159265358979 * fft_j / (fft_span * 1.0);
                        tw_re = $rtoi($cos(tw_angle) * 32767.0);
                        tw_im = $rtoi($sin(tw_angle) * 32767.0);

                        t_re = (work_re[fft_idx_odd] * tw_re - work_im[fft_idx_odd] * tw_im) >>> 15;
                        t_im = (work_re[fft_idx_odd] * tw_im + work_im[fft_idx_odd] * tw_re) >>> 15;

                        u_re = work_re[fft_idx_even];
                        u_im = work_im[fft_idx_even];

                        work_re[fft_idx_even] = u_re + t_re;
                        work_im[fft_idx_even] = u_im + t_im;
                        work_re[fft_idx_odd]  = u_re - t_re;
                        work_im[fft_idx_odd]  = u_im - t_im;
                    end
                end
            end

            // Copy signal FFT results to fwd_out (saturate to 16-bit)
            for (i = 0; i < FFT_SIZE; i = i + 1) begin
                if (work_re[i] > 32767)
                    fwd_out_i[i] <= 16'sh7FFF;
                else if (work_re[i] < -32768)
                    fwd_out_i[i] <= 16'sh8000;
                else
                    fwd_out_i[i] <= work_re[i][15:0];

                if (work_im[i] > 32767)
                    fwd_out_q[i] <= 16'sh7FFF;
                else if (work_im[i] < -32768)
                    fwd_out_q[i] <= 16'sh8000;
                else
                    fwd_out_q[i] <= work_im[i][15:0];
            end

            fwd_out_valid <= 1;
            state <= ST_REF_BITREV;

            `ifdef SIMULATION
            $display("[MF_CHAIN] Forward FFT complete");
            `endif
        end

        // ================================================================
        // REF_BITREV: Bit-reverse copy reference into work arrays
        // ================================================================
        ST_REF_BITREV: begin
            for (i = 0; i < FFT_SIZE; i = i + 1) begin
                work_re[bit_reverse(i[ADDR_BITS-1:0])] <= {{16{ref_buf_i[i][15]}}, ref_buf_i[i]};
                work_im[bit_reverse(i[ADDR_BITS-1:0])] <= {{16{ref_buf_q[i][15]}}, ref_buf_q[i]};
            end
            state <= ST_REF_BUTTERFLY;
        end

        // ================================================================
        // REF_BUTTERFLY: Forward FFT of reference (same algorithm as signal)
        // ================================================================
        ST_REF_BUTTERFLY: begin
            for (fft_stage = 0; fft_stage < ADDR_BITS; fft_stage = fft_stage + 1) begin
                fft_half = 1 << fft_stage;
                fft_span = fft_half << 1;
                for (fft_k = 0; fft_k < FFT_SIZE; fft_k = fft_k + fft_span) begin
                    for (fft_j = 0; fft_j < fft_half; fft_j = fft_j + 1) begin
                        fft_idx_even = fft_k + fft_j;
                        fft_idx_odd  = fft_idx_even + fft_half;

                        tw_angle = -2.0 * 3.14159265358979 * fft_j / (fft_span * 1.0);
                        tw_re = $rtoi($cos(tw_angle) * 32767.0);
                        tw_im = $rtoi($sin(tw_angle) * 32767.0);

                        t_re = (work_re[fft_idx_odd] * tw_re - work_im[fft_idx_odd] * tw_im) >>> 15;
                        t_im = (work_re[fft_idx_odd] * tw_im + work_im[fft_idx_odd] * tw_re) >>> 15;

                        u_re = work_re[fft_idx_even];
                        u_im = work_im[fft_idx_even];

                        work_re[fft_idx_even] = u_re + t_re;
                        work_im[fft_idx_even] = u_im + t_im;
                        work_re[fft_idx_odd]  = u_re - t_re;
                        work_im[fft_idx_odd]  = u_im - t_im;
                    end
                end
            end

            // Copy reference FFT results to ref_fft (saturate to 16-bit)
            for (i = 0; i < FFT_SIZE; i = i + 1) begin
                if (work_re[i] > 32767)
                    ref_fft_i[i] <= 16'sh7FFF;
                else if (work_re[i] < -32768)
                    ref_fft_i[i] <= 16'sh8000;
                else
                    ref_fft_i[i] <= work_re[i][15:0];

                if (work_im[i] > 32767)
                    ref_fft_q[i] <= 16'sh7FFF;
                else if (work_im[i] < -32768)
                    ref_fft_q[i] <= 16'sh8000;
                else
                    ref_fft_q[i] <= work_im[i][15:0];
            end

            state <= ST_MULTIPLY;

            `ifdef SIMULATION
            $display("[MF_CHAIN] Reference FFT complete");
            `endif
        end

        // ================================================================
        // MULTIPLY: Conjugate multiply FFT(signal) x conj(FFT(reference))
        // (a+jb)(c-jd) = (ac+bd) + j(bc-ad)
        // Uses fwd_out (signal FFT) and ref_fft (reference FFT)
        // ================================================================
        ST_MULTIPLY: begin
            for (i = 0; i < FFT_SIZE; i = i + 1) begin : mult_loop
                reg signed [31:0] a, b, c, d;
                reg signed [31:0] ac, bd, bc, ad;
                reg signed [31:0] re_result, im_result;
                
                a = {{16{fwd_out_i[i][15]}}, fwd_out_i[i]};
                b = {{16{fwd_out_q[i][15]}}, fwd_out_q[i]};
                c = {{16{ref_fft_i[i][15]}}, ref_fft_i[i]};
                d = {{16{ref_fft_q[i][15]}}, ref_fft_q[i]};

                ac = (a * c) >>> 15;
                bd = (b * d) >>> 15;
                bc = (b * c) >>> 15;
                ad = (a * d) >>> 15;

                re_result = ac + bd;
                im_result = bc - ad;

                // Saturate
                if (re_result > 32767)
                    mult_out_i[i] <= 16'sh7FFF;
                else if (re_result < -32768)
                    mult_out_i[i] <= 16'sh8000;
                else
                    mult_out_i[i] <= re_result[15:0];

                if (im_result > 32767)
                    mult_out_q[i] <= 16'sh7FFF;
                else if (im_result < -32768)
                    mult_out_q[i] <= 16'sh8000;
                else
                    mult_out_q[i] <= im_result[15:0];
            end

            mult_done <= 1;
            state     <= ST_INV_BITREV;

            `ifdef SIMULATION
            $display("[MF_CHAIN] Conjugate multiply complete");
            `endif
        end

        // ================================================================
        // INV_BITREV: Bit-reverse copy conjugate-multiply product
        // ================================================================
        ST_INV_BITREV: begin
            for (i = 0; i < FFT_SIZE; i = i + 1) begin
                work_re[bit_reverse(i[ADDR_BITS-1:0])] <= {{16{mult_out_i[i][15]}}, mult_out_i[i]};
                work_im[bit_reverse(i[ADDR_BITS-1:0])] <= {{16{mult_out_q[i][15]}}, mult_out_q[i]};
            end
            state <= ST_INV_BUTTERFLY;
        end

        // ================================================================
        // INV_BUTTERFLY: IFFT butterflies (positive twiddle) + 1/N scaling
        // ================================================================
        ST_INV_BUTTERFLY: begin
            for (fft_stage = 0; fft_stage < ADDR_BITS; fft_stage = fft_stage + 1) begin
                fft_half = 1 << fft_stage;
                fft_span = fft_half << 1;
                for (fft_k = 0; fft_k < FFT_SIZE; fft_k = fft_k + fft_span) begin
                    for (fft_j = 0; fft_j < fft_half; fft_j = fft_j + 1) begin
                        fft_idx_even = fft_k + fft_j;
                        fft_idx_odd  = fft_idx_even + fft_half;

                        // IFFT twiddle: +2*pi (positive exponent for inverse)
                        tw_angle = +2.0 * 3.14159265358979 * fft_j / (fft_span * 1.0);
                        tw_re = $rtoi($cos(tw_angle) * 32767.0);
                        tw_im = $rtoi($sin(tw_angle) * 32767.0);

                        t_re = (work_re[fft_idx_odd] * tw_re - work_im[fft_idx_odd] * tw_im) >>> 15;
                        t_im = (work_re[fft_idx_odd] * tw_im + work_im[fft_idx_odd] * tw_re) >>> 15;

                        u_re = work_re[fft_idx_even];
                        u_im = work_im[fft_idx_even];

                        work_re[fft_idx_even] = u_re + t_re;
                        work_im[fft_idx_even] = u_im + t_im;
                        work_re[fft_idx_odd]  = u_re - t_re;
                        work_im[fft_idx_odd]  = u_im - t_im;
                    end
                end
            end

            // Scale by 1/N (right shift by log2(1024) = 10) and store
            for (i = 0; i < FFT_SIZE; i = i + 1) begin : ifft_scale
                reg signed [31:0] scaled_re, scaled_im;
                scaled_re = work_re[i] >>> ADDR_BITS;
                scaled_im = work_im[i] >>> ADDR_BITS;

                if (scaled_re > 32767)
                    ifft_out_i[i] <= 16'sh7FFF;
                else if (scaled_re < -32768)
                    ifft_out_i[i] <= 16'sh8000;
                else
                    ifft_out_i[i] <= scaled_re[15:0];

                if (scaled_im > 32767)
                    ifft_out_q[i] <= 16'sh7FFF;
                else if (scaled_im < -32768)
                    ifft_out_q[i] <= 16'sh8000;
                else
                    ifft_out_q[i] <= scaled_im[15:0];
            end

            ifft_done <= 1;
            state     <= ST_OUTPUT;

            `ifdef SIMULATION
            $display("[MF_CHAIN] Inverse FFT complete — range profile ready");
            `endif
        end

        // ================================================================
        // OUTPUT: Stream out 1024 range profile samples, one per clock
        // ================================================================
        ST_OUTPUT: begin
            if (out_count < FFT_SIZE) begin
                out_i_reg     <= ifft_out_i[out_count];
                out_q_reg     <= ifft_out_q[out_count];
                out_valid_reg <= 1'b1;
                out_count     <= out_count + 1;
            end else begin
                state <= ST_DONE;
            end
        end

        // ================================================================
        // DONE: Return to idle, ready for next frame
        // ================================================================
        ST_DONE: begin
            state <= ST_IDLE;

            `ifdef SIMULATION
            $display("[MF_CHAIN] Frame complete, returning to IDLE");
            `endif
        end

        default: state <= ST_IDLE;
        endcase
    end
end

// ============================================================================
// OUTPUT ASSIGNMENTS
// ============================================================================
assign range_profile_i     = out_i_reg;
assign range_profile_q     = out_q_reg;
assign range_profile_valid = out_valid_reg;
assign chain_state         = state;

// ============================================================================
// BUFFER INITIALIZATION (simulation)
// ============================================================================
integer init_idx;
initial begin
    for (init_idx = 0; init_idx < FFT_SIZE; init_idx = init_idx + 1) begin
        fwd_buf_i[init_idx]  = 16'd0;
        fwd_buf_q[init_idx]  = 16'd0;
        fwd_out_i[init_idx]  = 16'd0;
        fwd_out_q[init_idx]  = 16'd0;
        ref_buf_i[init_idx]  = 16'd0;
        ref_buf_q[init_idx]  = 16'd0;
        ref_fft_i[init_idx]  = 16'd0;
        ref_fft_q[init_idx]  = 16'd0;
        mult_out_i[init_idx] = 16'd0;
        mult_out_q[init_idx] = 16'd0;
        ifft_out_i[init_idx] = 16'd0;
        ifft_out_q[init_idx] = 16'd0;
        work_re[init_idx]    = 32'd0;
        work_im[init_idx]    = 32'd0;
    end
end

endmodule
