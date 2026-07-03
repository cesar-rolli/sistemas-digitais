// =============================================================================
// i2s_receiver.v  —  Receptor I2S para WM8731 em modo SLAVE
//
// O FPGA gera BCLK e LRCLK. O WM8731 fornece dados em AUD_ADCDAT.
//
// Protocolo I2S padrão (Philips I2S):
//   - LRCLK = 0 → canal ESQUERDO
//   - LRCLK = 1 → canal DIREITO
//   - MSB do canal esquerdo aparece 1 BCLK APÓS a borda de DESCIDA do LRCLK
//   - Dados mudam na borda de DESCIDA do BCLK
//   - Dados são capturados na borda de SUBIDA do BCLK
//   - Formato: 16 bits (configurado no WM8731 via I2C)
//
// CORREÇÃO em relação à versão anterior:
//   1. A borda detectada para início do canal L é DESCIDA do LRCLK (L→0), OK
//      mas o bit_cnt precisa começar do SEGUNDO BCLK após a borda
//      (1 BCLK de delay do protocolo I2S padrão).
//   2. Capturamos exatamente 16 bits (MSB primeiro).
//   3. Paramos de capturar depois de 16 bits para não poluir com bits do R.
//
// Quartus II 12.1 / Verilog-2001
// =============================================================================
module i2s_receiver (
    input  wire        bclk,         // BCLK gerado pelo FPGA (mesmo domínio)
    input  wire        lrclk,        // LRCLK gerado pelo FPGA (mesmo domínio)
    input  wire        adc_dat,      // Dado serial do WM8731 (AUD_ADCDAT)
    input  wire        sys_clk,      // 50 MHz
    input  wire        rst,
    output reg  signed [15:0] sample_out,
    output reg         sample_valid
);

// ---------------------------------------------------------------------------
// Detecção de bordas — BCLK e LRCLK estão no mesmo domínio que sys_clk
// (gerados pelo mesmo always @(posedge sys_clk) no top-level)
// Precisamos de 1 ciclo de delay para detectar transições.
// ---------------------------------------------------------------------------
reg bclk_d,  lrclk_d;

always @(posedge sys_clk or posedge rst) begin
    if (rst) begin
        bclk_d  <= 1'b1;
        lrclk_d <= 1'b0;
    end else begin
        bclk_d  <= bclk;
        lrclk_d <= lrclk;
    end
end

wire bclk_rise = ( bclk && !bclk_d);   // borda de SUBIDA  do BCLK
wire bclk_fall = (!bclk &&  bclk_d);   // borda de DESCIDA do BCLK (não usada aqui)
wire lr_fall   = (!lrclk &&  lrclk_d); // DESCIDA do LRCLK → início do canal L

// ---------------------------------------------------------------------------
// Desserializador
//
// Estado: wait_skip → aguarda 1 BCLK de delay após lr_fall
//         capturing → captura 16 bits na subida do BCLK
// ---------------------------------------------------------------------------
reg [4:0]  bit_cnt;    // conta os bits capturados (0..15)
reg [15:0] shift_reg;  // registrador de deslocamento
reg        capturing;  // 1 = capturando canal L
reg        skip_bclk;  // 1 = aguardando o 1º BCLK após lr_fall (descarte)

always @(posedge sys_clk or posedge rst) begin
    if (rst) begin
        bit_cnt      <= 5'd0;
        shift_reg    <= 16'd0;
        capturing    <= 1'b0;
        skip_bclk    <= 1'b0;
        sample_out   <= 16'sd0;
        sample_valid <= 1'b0;
    end else begin
        sample_valid <= 1'b0;  // pulso de 1 ciclo

        // Borda de descida do LRCLK → começa canal esquerdo
        // Precisamos pular 1 subida de BCLK antes de começar a capturar
        if (lr_fall) begin
            capturing <= 1'b0;
            skip_bclk <= 1'b1;
            bit_cnt   <= 5'd0;
            shift_reg <= 16'd0;
        end

        // Subida de LRCLK → começa canal direito, para de capturar
        if (lrclk && !lrclk_d) begin
            capturing <= 1'b0;
            skip_bclk <= 1'b0;
        end

        // Processamento na subida do BCLK
        if (bclk_rise) begin
            if (skip_bclk) begin
                // Pula o 1º BCLK — este é o bit de delay do protocolo I2S
                skip_bclk <= 1'b0;
                capturing <= 1'b1;
                bit_cnt   <= 5'd0;
            end else if (capturing) begin
                // Desloca e captura: MSB primeiro
                shift_reg <= {shift_reg[14:0], adc_dat};
                bit_cnt   <= bit_cnt + 5'd1;

                if (bit_cnt == 5'd15) begin
                    // Capturamos o último (16º) bit
                    sample_out   <= {shift_reg[14:0], adc_dat};
                    sample_valid <= 1'b1;
                    capturing    <= 1'b0;
                end
            end
        end
    end
end

endmodule