// =============================================================================
// fft1024_mag.v  —  FFT 1024 pontos, Radix-2 DIT, sequencial
//
// Correções desta versão:
//   1. always de ESCRITA e LEITURA das RAMs são SEPARADOS
//      → Quartus 12.1 infere dual-port M4K corretamente
//   2. Twiddle ROM: altsyncram explícito com UNREGISTERED (1 ciclo latência)
//   3. Threshold ajustado para 10000 (detecta sinais >= ~200 LSB)
//   4. ST_BFLY_C: estado extra de espera de 1 ciclo entre setar o endereço
//      de leitura do operando "b" e usar o dado — a RAM interna tem 1 ciclo
//      de latência registrada; sem esse estado o "b" saía sempre igual ao
//      "a" (bug que zerava o espectro inteiro).
//
// N=1024, NHALF=512 (twiddle table), 10 estágios (log2(1024))
//
// Quartus II 12.1 / Verilog-2001
// =============================================================================
module fft1024_mag (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    output reg  [9:0]  rd_addr,
    input  wire signed [15:0] rd_data,
    output reg  [9:0]  peak_bin,
    output reg         done
);

// ---------------------------------------------------------------------------
// Twiddle ROM: altsyncram 512x32 bits, UNREGISTERED = 1 ciclo latência
// ---------------------------------------------------------------------------
wire [31:0] tw_q;
reg  [8:0]  tw_addr;

altsyncram #(
    .operation_mode ("ROM"),
    .width_a        (32),
    .widthad_a      (9),
    .numwords_a     (512),
    .init_file      ("twiddle1024.mif"),
    .lpm_hint       ("ENABLE_RUNTIME_MOD=NO"),
    .outdata_reg_a  ("UNREGISTERED"),
    .ram_block_type ("M4K")
) u_twiddle (
    .clock0   (clk),
    .address_a(tw_addr),
    .q_a      (tw_q)
);

wire signed [15:0] tw_cos_q = tw_q[31:16];
wire signed [15:0] tw_sin_q = tw_q[15:0];

// ---------------------------------------------------------------------------
// RAMs internas: always SEPARADOS para escrita e leitura → infere M4K
// ---------------------------------------------------------------------------
(* ramstyle = "M4K" *) reg signed [31:0] ram_re [0:1023];
(* ramstyle = "M4K" *) reg signed [31:0] ram_im [0:1023];

reg         wr_en;
reg  [9:0]  wr_addr;
reg signed [31:0] wr_data_re, wr_data_im;
reg  [9:0]  ram_rd_addr;
reg signed [31:0] ram_re_q, ram_im_q;

// Porta A: escrita (always separado)
always @(posedge clk)
    if (wr_en) begin
        ram_re[wr_addr] <= wr_data_re;
        ram_im[wr_addr] <= wr_data_im;
    end

// Porta B: leitura (always separado)
always @(posedge clk) begin
    ram_re_q <= ram_re[ram_rd_addr];
    ram_im_q <= ram_im[ram_rd_addr];
end

// ---------------------------------------------------------------------------
// Bit-reversal 10 bits (fiação pura, 0 LEs)
// ---------------------------------------------------------------------------
function [9:0] br10;
    input [9:0] x;
    br10 = {x[0],x[1],x[2],x[3],x[4],x[5],x[6],x[7],x[8],x[9]};
endfunction

// ---------------------------------------------------------------------------
// Estados
// ---------------------------------------------------------------------------
localparam [3:0]
    ST_IDLE   = 4'd0,
    ST_LOAD   = 4'd1,
    ST_BFLY_A = 4'd2,
    ST_BFLY_B = 4'd3,
    ST_BFLY_C = 4'd4,
    ST_BFLY_W = 4'd5,
    ST_NEXT   = 4'd6,
    ST_MAG_A  = 4'd7,
    ST_MAG_B  = 4'd8,
    ST_DONE   = 4'd9;

reg [3:0]  state;
reg [10:0] load_cnt;
reg [3:0]  stage;
reg [9:0]  stride;
reg [9:0]  group_cnt;
reg [9:0]  pair_cnt;
reg [9:0]  idx_base;
reg [9:0]  idx_a, idx_b;
reg [9:0]  wr_addr_a, wr_addr_b;
reg        writing_b;
reg [9:0]  tw_step;

reg signed [31:0] a_re, a_im;
reg signed [15:0] w_re, w_im;
reg signed [47:0] wb_re48, wb_im48;
reg signed [31:0] wb_re, wb_im;

reg [9:0]  mag_idx;
reg [47:0] peak_mag;
reg [47:0] mag_sq;

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        state       <= ST_IDLE;
        done        <= 1'b0;
        peak_bin    <= 10'd0;
        rd_addr     <= 10'd0;
        load_cnt    <= 11'd0;
        tw_addr     <= 9'd0;
        tw_step     <= 10'd512;
        ram_rd_addr <= 10'd0;
        idx_base    <= 10'd0;
        wr_en       <= 1'b0;
        writing_b   <= 1'b0;
    end else begin
        done  <= 1'b0;
        wr_en <= 1'b0;

        case (state)

            ST_IDLE: begin
                if (start) begin
                    load_cnt <= 11'd0;
                    rd_addr  <= 10'd0;
                    state    <= ST_LOAD;
                end
            end

            ST_LOAD: begin
                if (load_cnt < 11'd1024)
                    rd_addr <= load_cnt[9:0] + 10'd1;

                if (load_cnt >= 11'd1 && load_cnt <= 11'd1024) begin
                    wr_en      <= 1'b1;
                    wr_addr    <= br10(load_cnt[9:0] - 10'd1);
                    wr_data_re <= {{16{rd_data[15]}}, rd_data};
                    wr_data_im <= 32'sd0;
                end

                if (load_cnt == 11'd1024) begin
                    stage       <= 4'd0;
                    stride      <= 10'd1;
                    tw_step     <= 10'd512;
                    group_cnt   <= 10'd0;
                    pair_cnt    <= 10'd0;
                    idx_base    <= 10'd0;
                    tw_addr     <= 9'd0;
                    ram_rd_addr <= 10'd0;
                    state       <= ST_BFLY_A;
                end else
                    load_cnt <= load_cnt + 11'd1;
            end

            // Registra índices; ram_rd_addr já aponta para idx_a (pré-carregado)
            ST_BFLY_A: begin
                idx_a <= idx_base + pair_cnt;
                idx_b <= idx_base + pair_cnt + stride;
                state <= ST_BFLY_B;
            end

            // ram_re_q = ram[idx_a] disponível; registra a e w; apresenta idx_b
            ST_BFLY_B: begin
                a_re        <= ram_re_q;
                a_im        <= ram_im_q;
                w_re        <= tw_cos_q;
                w_im        <= tw_sin_q;
                ram_rd_addr <= idx_b;
                wr_addr_a   <= idx_a;
                wr_addr_b   <= idx_b;
                state       <= ST_BFLY_C;
            end

            // Espera 1 ciclo: a RAM interna (leitura registrada) só reflete
            // idx_b na borda seguinte a esta. Sem este estado, ram_re_q em
            // ST_BFLY_W ainda contém ram[idx_a] (bug: "b" = "a" sempre).
            ST_BFLY_C: begin
                state <= ST_BFLY_W;
            end

            // ram_re_q = ram[idx_b] disponível; calcula; escreve idx_a
            ST_BFLY_W: begin
                wb_re48 = ($signed(w_re) * $signed(ram_re_q))
                        - ($signed(w_im) * $signed(ram_im_q));
                wb_im48 = ($signed(w_re) * $signed(ram_im_q))
                        + ($signed(w_im) * $signed(ram_re_q));
                wb_re = wb_re48[46:15];
                wb_im = wb_im48[46:15];

                wr_en      <= 1'b1;
                wr_addr    <= wr_addr_a;
                wr_data_re <= (a_re + wb_re) >>> 1;
                wr_data_im <= (a_im + wb_im) >>> 1;

                writing_b  <= 1'b1;
                state      <= ST_NEXT;
            end

            ST_NEXT: begin
                if (writing_b) begin
                    wr_en      <= 1'b1;
                    wr_addr    <= wr_addr_b;
                    wr_data_re <= (a_re - wb_re) >>> 1;
                    wr_data_im <= (a_im - wb_im) >>> 1;
                    writing_b  <= 1'b0;
                end else begin
                    if (pair_cnt < stride - 10'd1) begin
                        pair_cnt    <= pair_cnt + 10'd1;
                        tw_addr     <= tw_addr + tw_step[8:0];
                        ram_rd_addr <= idx_base + pair_cnt + 10'd1;
                        state       <= ST_BFLY_A;
                    end else begin
                        pair_cnt <= 10'd0;
                        tw_addr  <= 9'd0;
                        if (group_cnt < (10'd512 >> stage) - 10'd1) begin
                            group_cnt   <= group_cnt + 10'd1;
                            idx_base    <= idx_base + (stride << 1);
                            ram_rd_addr <= idx_base + (stride << 1);
                            state       <= ST_BFLY_A;
                        end else begin
                            group_cnt <= 10'd0;
                            idx_base  <= 10'd0;
                            if (stage == 4'd9) begin
                                mag_idx     <= 10'd1;
                                peak_bin    <= 10'd0;
                                peak_mag    <= 48'd0;
                                ram_rd_addr <= 10'd1;
                                state       <= ST_MAG_A;
                            end else begin
                                stage       <= stage + 4'd1;
                                stride      <= stride << 1;
                                tw_step     <= tw_step >> 1;
                                ram_rd_addr <= 10'd0;
                                state       <= ST_BFLY_A;
                            end
                        end
                    end
                end
            end

            ST_MAG_A: state <= ST_MAG_B;

            ST_MAG_B: begin
                mag_sq = ($signed(ram_re_q) * $signed(ram_re_q))
                       + ($signed(ram_im_q) * $signed(ram_im_q));

                if (mag_sq > peak_mag) begin
                    peak_mag <= mag_sq;
                    peak_bin <= mag_idx;
                end

                if (mag_idx == 10'd511) begin
                    state <= ST_DONE;
                end else begin
                    mag_idx     <= mag_idx + 10'd1;
                    ram_rd_addr <= mag_idx + 10'd1;
                    state       <= ST_MAG_A;
                end
            end

            ST_DONE: begin
                if (peak_mag < 48'd10000)
                    peak_bin <= 10'd0;
                done  <= 1'b1;
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
