// =============================================================================
// fft.v  —  Top-level: analisador de frequência WM8731 + FFT 256pts + 7-seg
//
// bin → freq: shift+add (191 = 128+32+16+8+4+2+1), sem lpm_mult/lpm_divide
// freq → BCD: Double-Dabble sequencial (15 ciclos, ~50 LEs)
//
// Display: HEX3=dezena_milhar, HEX2=milhar, HEX1=centena, HEX0=dezena
//          Unidade omitida (erro ±95 Hz irrelevante nessa casa)
//
// Placa: Altera DE2 (EP2C35F672C6), Quartus II 12.1
// =============================================================================
module fft (
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,

    input  wire        AUD_ADCDAT,
    output wire        AUD_ADCLRCK,
    output wire        AUD_BCLK,
    output wire        AUD_DACDAT,
    output wire        AUD_DACLRCK,
    output wire        AUD_XCK,

    output wire        I2C_SCLK,
    inout  wire        I2C_SDAT,
    output wire        TD_RESET,

    output wire [6:0]  HEX0,
    output wire [6:0]  HEX1,
    output wire [6:0]  HEX2,
    output wire [6:0]  HEX3,

    output wire [7:4]  LCD_DATA,
    output wire        LCD_RS,
    output wire        LCD_RW,
    output wire        LCD_EN,
    output wire        LCD_ON,
    output wire        LCD_BLON,

    output wire [17:0] LEDR
);

// ---------------------------------------------------------------------------
// Inativos
// ---------------------------------------------------------------------------
wire rst = ~KEY[0];

assign TD_RESET    = 1'b1;
assign AUD_DACDAT  = 1'b0;
assign AUD_DACLRCK = 1'b0;
assign LCD_DATA    = 4'b0;
assign LCD_RS      = 1'b0;
assign LCD_RW      = 1'b0;
assign LCD_EN      = 1'b0;
assign LCD_ON      = 1'b0;
assign LCD_BLON    = 1'b0;
assign LEDR        = 18'b0;

// ---------------------------------------------------------------------------
// Clocks WM8731 (slave): MCLK=12.5MHz, BCLK=3.125MHz, LRCLK≈48.8kHz
// ---------------------------------------------------------------------------
reg [1:0] mclk_div;
always @(posedge CLOCK_50 or posedge rst)
    if (rst) mclk_div <= 2'd0; else mclk_div <= mclk_div + 2'd1;
assign AUD_XCK = mclk_div[1];

reg [3:0] bclk_cnt;
always @(posedge CLOCK_50 or posedge rst)
    if (rst) bclk_cnt <= 4'd0; else bclk_cnt <= bclk_cnt + 4'd1;
wire bclk_int = bclk_cnt[3];

reg bclk_r;
reg [5:0] lr_cnt;
always @(posedge CLOCK_50 or posedge rst) begin
    if (rst) begin bclk_r <= 1'b1; lr_cnt <= 6'd0; end
    else begin
        bclk_r <= bclk_int;
        if (bclk_int & ~bclk_r) lr_cnt <= lr_cnt + 6'd1;
    end
end
wire lrclk_int = lr_cnt[5];

assign AUD_BCLK    = bclk_int;
assign AUD_ADCLRCK = lrclk_int;

// ---------------------------------------------------------------------------
// I2C
// ---------------------------------------------------------------------------
wire i2c_scl_out, i2c_sda_out, i2c_done;
wire [3:0] dbg_state, dbg_regidx;

assign I2C_SCLK = i2c_scl_out;
assign I2C_SDAT = i2c_sda_out ? 1'bz : 1'b0;

wm8731_i2c_config u_i2c (
    .clk(CLOCK_50), .rst(rst),
    .scl(i2c_scl_out), .sda_out(i2c_sda_out),
    .done(i2c_done),
    .dbg_state(dbg_state), .dbg_regidx(dbg_regidx)
);

// ---------------------------------------------------------------------------
// I2S
// ---------------------------------------------------------------------------
wire signed [15:0] sample_out;
wire               sample_valid;

i2s_receiver u_i2s (
    .bclk(bclk_int), .lrclk(lrclk_int),
    .adc_dat(AUD_ADCDAT), .sys_clk(CLOCK_50), .rst(rst),
    .sample_out(sample_out), .sample_valid(sample_valid)
);

// ---------------------------------------------------------------------------
// Buffer circular 256 amostras
// ---------------------------------------------------------------------------
reg signed [15:0] sample_buf [0:255];
reg [7:0] write_ptr;
reg       buf_rdy;

always @(posedge CLOCK_50 or posedge rst) begin
    if (rst) begin write_ptr <= 8'd0; buf_rdy <= 1'b0; end
    else begin
        // Reseta buf_rdy quando FFT começa → aguarda novo bloco de amostras
        if (fft_start) buf_rdy <= 1'b0;
        if (sample_valid && i2c_done) begin
            sample_buf[write_ptr] <= sample_out;
            write_ptr <= write_ptr + 8'd1;
            if (write_ptr == 8'd255) buf_rdy <= 1'b1;
        end
    end
end

wire [9:0] fft_rd_addr;
wire [7:0] rd_idx = fft_rd_addr[7:0] + write_ptr;
reg signed [15:0] fft_rd_data_r;
always @(posedge CLOCK_50)
    fft_rd_data_r <= sample_buf[rd_idx];

// ---------------------------------------------------------------------------
// FFT
// ---------------------------------------------------------------------------
wire [9:0] peak_bin;
wire       fft_done;
reg        fft_start, fft_busy;

always @(posedge CLOCK_50 or posedge rst) begin
    if (rst) begin fft_start <= 1'b0; fft_busy <= 1'b0; end
    else begin
        fft_start <= 1'b0;
        if (fft_done) fft_busy <= 1'b0;
        if (buf_rdy && !fft_busy) begin
            fft_start <= 1'b1; fft_busy <= 1'b1;
        end
    end
end

fft1024_mag u_fft (
    .clk(CLOCK_50), .rst(rst), .start(fft_start),
    .rd_addr(fft_rd_addr), .rd_data(fft_rd_data_r),
    .peak_bin(peak_bin), .done(fft_done)
);

// ---------------------------------------------------------------------------
// Passo 1: bin → freq via shift+add (191 = 0b10111111)
// freq = bin*128 + bin*32 + bin*16 + bin*8 + bin*4 + bin*2 + bin
// Máximo: 127*191 = 24257 → 15 bits
// ---------------------------------------------------------------------------
reg [14:0] freq_reg;
reg        bcd_start;

always @(posedge CLOCK_50 or posedge rst) begin
    if (rst) begin freq_reg <= 15'd0; bcd_start <= 1'b0; end
    else begin
        bcd_start <= 1'b0;
        if (fft_done) begin
            if (peak_bin == 10'd0)
                freq_reg <= 15'd0;
            else
                freq_reg <= ({7'd0, peak_bin[7:0]} << 7)
                          + ({9'd0, peak_bin[7:0]} << 5)
                          + ({10'd0,peak_bin[7:0]} << 4)
                          + ({11'd0,peak_bin[7:0]} << 3)
                          + ({12'd0,peak_bin[7:0]} << 2)
                          + ({13'd0,peak_bin[7:0]} << 1)
                          +  {14'd0,peak_bin[7:0]};
            bcd_start <= 1'b1;
        end
    end
end

// ---------------------------------------------------------------------------
// Passo 2: Double-Dabble sequencial (15 ciclos)
// Registrador de 35 bits: [34:20]=BCD 5 nibbles, [14:0]=dados
//
// CORREÇÃO: usamos t[35:0] (36 bits) para o add-3 intermediário,
// evitando perder o bit [34] na concatenação final.
// ---------------------------------------------------------------------------
reg [34:0] dabble;
reg [35:0] t_wide;     // variável temporária 36 bits para add-3
reg [3:0]  dab_cnt;
reg        dab_busy;

always @(posedge CLOCK_50 or posedge rst) begin
    if (rst) begin
        dabble   <= 35'd0;
        dab_cnt  <= 4'd0;
        dab_busy <= 1'b0;
        t_wide   <= 36'd0;
    end else if (bcd_start) begin
        dabble   <= {20'd0, freq_reg};
        dab_cnt  <= 4'd0;
        dab_busy <= 1'b1;
    end else if (dab_busy) begin
        // Add-3: verifica cada nibble BCD e soma 3 se >= 5
        t_wide = {1'b0, dabble};          // 36 bits, bit 35 = 0
        if (t_wide[34:31] >= 4'd5) t_wide[34:31] = t_wide[34:31] + 4'd3;
        if (t_wide[30:27] >= 4'd5) t_wide[30:27] = t_wide[30:27] + 4'd3;
        if (t_wide[26:23] >= 4'd5) t_wide[26:23] = t_wide[26:23] + 4'd3;
        if (t_wide[22:19] >= 4'd5) t_wide[22:19] = t_wide[22:19] + 4'd3;
        if (t_wide[18:15] >= 4'd5) t_wide[18:15] = t_wide[18:15] + 4'd3;
        // Shift esquerdo 1: usa t_wide[34:0] para capturar todos os 35 bits
        dabble <= (t_wide[34:0] << 1);

        dab_cnt <= dab_cnt + 4'd1;
        if (dab_cnt == 4'd14) dab_busy <= 1'b0;
    end
end

// Extrai os 4 dígitos mais significativos (descarta unidade = dabble[18:15])
wire [3:0] dig3 = dabble[34:31];  // dezena de milhar
wire [3:0] dig2 = dabble[30:27];  // milhar
wire [3:0] dig1 = dabble[26:23];  // centena
wire [3:0] dig0 = dabble[22:19];  // dezena

// ---------------------------------------------------------------------------
// 7-segmentos (cátodo comum, ativo-baixo)
// ---------------------------------------------------------------------------
function [6:0] seg7;
    input [3:0] d;
    case (d)
        4'd0: seg7 = 7'b1000000;  4'd1: seg7 = 7'b1111001;
        4'd2: seg7 = 7'b0100100;  4'd3: seg7 = 7'b0110000;
        4'd4: seg7 = 7'b0011001;  4'd5: seg7 = 7'b0010010;
        4'd6: seg7 = 7'b0000010;  4'd7: seg7 = 7'b1111000;
        4'd8: seg7 = 7'b0000000;  4'd9: seg7 = 7'b0010000;
        default: seg7 = 7'b1111111;
    endcase
endfunction

assign HEX3 = seg7(dig3);
assign HEX2 = seg7(dig2);
assign HEX1 = seg7(dig1);
assign HEX0 = seg7(dig0);

endmodule