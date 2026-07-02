// =============================================================================
// lcd_and_formatter.v  —  Quartus II 12.1 / Verilog-2001 compatible
// =============================================================================

// =============================================================================
// freq_to_lcd
// =============================================================================
module freq_to_lcd (
    input  wire        clk,
    input  wire        rst,
    input  wire [23:0] freq_hz,
    input  wire        buf_rdy,
    input  wire        fft_done,
    output reg  [127:0] line1,
    output reg  [127:0] line2
);

localparam [127:0] LINE1_WAIT =
    {8'h41,8'h67,8'h75,8'h61,8'h72,8'h64,8'h61,8'h6E,
     8'h64,8'h6F,8'h2E,8'h2E,8'h2E,8'h20,8'h20,8'h20};
localparam [127:0] LINE2_WAIT =
    {8'h43,8'h6F,8'h6E,8'h65,8'h63,8'h74,8'h65,8'h20,
     8'h4C,8'h69,8'h6E,8'h65,8'h2D,8'h49,8'h6E,8'h20};
localparam [127:0] LINE2_FIXED =
    {8'h52,8'h65,8'h73,8'h3A,8'h37,8'h35,8'h30,8'h48,
     8'h7A,8'h2F,8'h62,8'h69,8'h6E,8'h20,8'h20,8'h20};

always @(posedge clk or posedge rst) begin
    if (rst) begin
        line1 <= LINE1_WAIT;
        line2 <= LINE2_WAIT;
    end else if (!buf_rdy) begin
        line1 <= LINE1_WAIT;
        line2 <= LINE2_WAIT;
    end else if (fft_done) begin
        line1 <= {
            8'h46, 8'h72, 8'h65, 8'h71, 8'h3A,
            (freq_hz / 10000) % 10 + 8'h30,
            (freq_hz / 1000)  % 10 + 8'h30,
            (freq_hz / 100)   % 10 + 8'h30,
            (freq_hz / 10)    % 10 + 8'h30,
             freq_hz          % 10 + 8'h30,
            8'h48, 8'h7A, 8'h20, 8'h20, 8'h20
        };
        line2 <= LINE2_FIXED;
    end
end

endmodule


// =============================================================================
// lcd_controller
// HD44780 modo 4 bits. FSM linear com contador de 32 bits.
//
// Timing base: contador em ciclos de 50 MHz (20 ns).
// Cada "passo" = colocar nibble + EN=1 por 1µs + EN=0 + esperar delay.
//
// Init sequence (datasheet fig.24):
//   50ms  → nibble 0x3 → 5ms
//         → nibble 0x3 → 200µs
//         → nibble 0x3 → 200µs
//         → nibble 0x2 → 50µs   (4-bit mode)
//   Cmd 0x28 → Cmd 0x08 → Cmd 0x01 → Cmd 0x06 → Cmd 0x0C
//   Set addr 0x80 → write 16 chars → Set addr 0xC0 → write 16 chars
//   Repeat from Set addr 0x80
// =============================================================================
module lcd_controller (
    input  wire        clk,
    input  wire        rst,
    input  wire [127:0] line1,
    input  wire [127:0] line2,
    output reg         lcd_rs,
    output wire        lcd_rw,
    output reg         lcd_en,
    output reg  [7:4]  lcd_data
);

assign lcd_rw = 1'b0;

// Extrai caracteres das linhas latched
reg [127:0] l1, l2;
wire [7:0] ch [0:31];
genvar gi;
generate
    for (gi = 0; gi < 16; gi = gi + 1) begin : gch
        assign ch[gi]    = l1[127 - gi*8 -: 8];
        assign ch[gi+16] = l2[127 - gi*8 -: 8];
    end
endgenerate

// ---------------------------------------------------------------------------
// Estados
// ---------------------------------------------------------------------------
localparam [6:0]
    S_PWRUP   = 7'd0,
    S_I1      = 7'd1,   // nibble 0x3, wait 5ms
    S_I2      = 7'd2,   // nibble 0x3, wait 200us
    S_I3      = 7'd3,   // nibble 0x3, wait 200us
    S_I4      = 7'd4,   // nibble 0x2, wait 50us  (→4bit)
    S_FN_H    = 7'd5,   S_FN_L  = 7'd6,   // 0x28
    S_DO_H    = 7'd7,   S_DO_L  = 7'd8,   // 0x08
    S_CL_H    = 7'd9,   S_CL_L  = 7'd10,  // 0x01
    S_EM_H    = 7'd11,  S_EM_L  = 7'd12,  // 0x06
    S_DN_H    = 7'd13,  S_DN_L  = 7'd14,  // 0x0C
    S_L1_H    = 7'd15,  S_L1_L  = 7'd16,  // DDRAM 0x80  <- reentrada
    S_WR      = 7'd17,  // 32 nibbles: estados 17..48 (16 chars)
    S_L2_H    = 7'd49,  S_L2_L  = 7'd50,  // DDRAM 0xC0
    S_WR2     = 7'd51,  // 32 nibbles: estados 51..82 (16 chars)
    S_LATCH   = 7'd83;  // relatch e volta para S_L1_H

reg [6:0]  state;
reg [31:0] timer;
reg        en_phase;   // 0: EN alto, 1: EN baixo+delay

// Nibble a enviar em cada estado
// (rs=bit4, data=bits3:0)
reg [4:0]  cur_nib;
reg [4:0]  char_idx;  // 0..31

// Delay após cada estado (em ciclos de 50MHz)
reg [31:0] cur_delay;

// ---------------------------------------------------------------------------
// Lógica de decodificação do nibble e delay (combinacional, sem function)
// ---------------------------------------------------------------------------
reg [4:0]  nib_out;
reg [31:0] dly_out;
reg [7:0]  ch_sel;

always @(*) begin
    // Seleciona char para estados de escrita
    if (state >= S_WR && state < S_L2_H)
        ch_sel = ch[(state - S_WR) >> 1];
    else if (state >= S_WR2 && state < S_LATCH)
        ch_sel = ch[16 + ((state - S_WR2) >> 1)];
    else
        ch_sel = 8'h20;

    case (state)
        S_I1,S_I2,S_I3: nib_out = {1'b0, 4'h3};
        S_I4:           nib_out = {1'b0, 4'h2};
        S_FN_H:         nib_out = {1'b0, 4'h2};
        S_FN_L:         nib_out = {1'b0, 4'h8};
        S_DO_H:         nib_out = {1'b0, 4'h0};
        S_DO_L:         nib_out = {1'b0, 4'h8};
        S_CL_H:         nib_out = {1'b0, 4'h0};
        S_CL_L:         nib_out = {1'b0, 4'h1};
        S_EM_H:         nib_out = {1'b0, 4'h0};
        S_EM_L:         nib_out = {1'b0, 4'h6};
        S_DN_H:         nib_out = {1'b0, 4'h0};
        S_DN_L:         nib_out = {1'b0, 4'hC};
        S_L1_H:         nib_out = {1'b0, 4'h8};
        S_L1_L:         nib_out = {1'b0, 4'h0};
        S_L2_H:         nib_out = {1'b0, 4'hC};
        S_L2_L:         nib_out = {1'b0, 4'h0};
        default: begin
            // Chars: estados alternos hi/lo nibble
            if ((state - S_WR) % 2 == 0 || (state - S_WR2) % 2 == 0)
                nib_out = {1'b1, ch_sel[7:4]};
            else
                nib_out = {1'b1, ch_sel[3:0]};
        end
    endcase

    case (state)
        S_PWRUP: dly_out = 32'd2500000; // 50ms
        S_I1:    dly_out = 32'd250000;  //  5ms
        S_I2:    dly_out = 32'd10000;   //200us
        S_I3:    dly_out = 32'd10000;   //200us
        S_CL_L:  dly_out = 32'd100000;  //  2ms (clear precisa de mais tempo)
        default: dly_out = 32'd2500;    // 50us (todos os demais)
    endcase
end

// ---------------------------------------------------------------------------
// FSM sequencial
// ---------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        state     <= S_PWRUP;
        timer     <= 32'd2500000;
        en_phase  <= 1'b0;
        lcd_en    <= 1'b0;
        lcd_rs    <= 1'b0;
        lcd_data  <= 4'h0;
        l1        <= 128'd0;
        l2        <= 128'd0;
    end else begin
        lcd_en <= 1'b0;

        if (timer > 32'd0) begin
            timer <= timer - 32'd1;
        end else begin

            if (state == S_PWRUP) begin
                // Power-up: apenas esperou, agora latch e começa init
                l1    <= line1;
                l2    <= line2;
                state <= S_I1;
                timer <= 32'd50;  // tempo para levantar EN
                en_phase <= 1'b0;

            end else if (state == S_LATCH) begin
                // Fim do frame: relatch linhas e reinicia escrita
                l1    <= line1;
                l2    <= line2;
                state <= S_L1_H;
                timer <= 32'd50;
                en_phase <= 1'b0;

            end else if (!en_phase) begin
                // Levanta EN
                lcd_rs   <= nib_out[4];
                lcd_data <= nib_out[3:0];
                lcd_en   <= 1'b1;
                en_phase <= 1'b1;
                timer    <= 32'd50;  // EN alto por 1µs

            end else begin
                // Baixa EN, espera delay, avança estado
                lcd_en   <= 1'b0;
                en_phase <= 1'b0;
                timer    <= dly_out;
                state    <= state + 7'd1;
            end
        end
    end
end

endmodule