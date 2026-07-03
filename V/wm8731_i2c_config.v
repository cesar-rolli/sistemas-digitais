// =============================================================================
// wm8731_i2c_config.v  —  Configura o WM8731 via I2C bit-bang
//
// Substitui o i2c_terasic por um controlador próprio, simples e correto.
//
// Protocolo I2C:
//   START:  SDA 1→0 enquanto SCL=1
//   BIT:    fase0(SCL=0,SDA=dado) → fase1(SCL=1) → fase2(SCL=1) → fase3(SCL=0)
//   ACK:    bit 9, SDA=Z (lemos — mas não verificamos, simplificação)
//   STOP:   SDA 0→1 enquanto SCL=1
//
// SCL = 100 kHz → período = 500 ciclos de 50 MHz → cada fase = 125 ciclos
//
// Cada mensagem = 3 bytes (dev_addr + reg_byte + data_byte)
// 9 mensagens × ~640 µs ≈ 5.8 ms total
//
// Registradores WM8731 configurados:
//   R15=reset, R0/R1=line-in max, R4=line-in sel, R5=digital normal,
//   R6=ADC on mic off, R7=I2S 16b slave, R8=48kHz normal, R9=activate
//
// Quartus II 12.1 / Verilog-2001
// =============================================================================
module wm8731_i2c_config (
    input  wire       clk,       // 50 MHz
    input  wire       rst,       // reset síncrono ativo-alto
    output reg        scl,
    output reg        sda_out,   // SDA saída (0 = puxa baixo, 1 = release/hi-Z)
    output reg        done,
    output wire [3:0] dbg_state,
    output wire [3:0] dbg_regidx
);

// ---------------------------------------------------------------------------
// Tabela de configuração: 9 registradores WM8731
// Formato interno: {reg_addr[6:0], reg_data[8:0]} = 16 bits
// ---------------------------------------------------------------------------
reg [15:0] cfg [0:8];
initial begin
    cfg[0] = {7'h0F, 9'h000}; // R15 Reset
    cfg[1] = {7'h00, 9'h01F}; // R0  Left Line In  vol=max, unmuted
    cfg[2] = {7'h01, 9'h01F}; // R1  Right Line In vol=max, unmuted
    cfg[3] = {7'h04, 9'h010}; // R4  Analog path: LINE-IN sel, mic muted
    cfg[4] = {7'h05, 9'h000}; // R5  Digital path: normal
    cfg[5] = {7'h06, 9'h002}; // R6  Power: ADC on, line-in on, mic off
    cfg[6] = {7'h07, 9'h002}; // R7  Format: I2S, 16-bit, slave
    cfg[7] = {7'h08, 9'h000}; // R8  Sampling: 48kHz, normal mode
    cfg[8] = {7'h09, 9'h001}; // R9  Active
end

// ---------------------------------------------------------------------------
// Divisor de clock: 125 ciclos por fase → SCL = 50M / 500 = 100 kHz
// ---------------------------------------------------------------------------
localparam PHASE_CYC = 8'd124; // 125 ciclos (0..124)

reg [7:0] phase_cnt;   // contador de ciclos dentro da fase

wire phase_done = (phase_cnt == PHASE_CYC);

always @(posedge clk or posedge rst) begin
    if (rst)
        phase_cnt <= 8'd0;
    else if (phase_done)
        phase_cnt <= 8'd0;
    else
        phase_cnt <= phase_cnt + 8'd1;
end

// ---------------------------------------------------------------------------
// FSM principal
// Estados de bit: IDLE → START_A → START_B → SEND → ACK → (STOP_A → STOP_B)
// ---------------------------------------------------------------------------
localparam [3:0]
    ST_IDLE    = 4'd0,
    ST_WAIT    = 4'd1,   // espera inicial 50 ms
    ST_START_A = 4'd2,   // SDA cai, SCL alto
    ST_START_B = 4'd3,   // SCL desce
    ST_BIT_LO  = 4'd4,   // SCL baixo, coloca dado no SDA
    ST_BIT_HI  = 4'd5,   // SCL sobe, dado estável
    ST_STOP_A  = 4'd6,   // SDA=0, SCL sobe
    ST_STOP_B  = 4'd7,   // SDA sobe (STOP)
    ST_NEXT    = 4'd8,   // avança para próximo registrador
    ST_DONE    = 4'd9;

reg [3:0]  state;
reg [3:0]  reg_idx;    // 0..8 (registrador atual)
reg [1:0]  byte_idx;   // 0..2 (byte dentro da mensagem)
reg [3:0]  bit_idx;    // 0..8 (bit dentro do byte; 8=ACK)
reg [7:0]  shift;      // byte atual sendo enviado
reg [24:0] wait_cnt;   // contador para espera inicial

// Montagem dos 3 bytes da mensagem I2C:
//   byte 0: endereço do WM8731 (0x34 = 7'h1A << 1 | 0)
//   byte 1: {reg_addr[6:0], reg_data[8]}
//   byte 2: reg_data[7:0]
wire [7:0] msg0 = 8'h34;
wire [7:0] msg1 = {cfg[reg_idx][15:9], cfg[reg_idx][8]};
wire [7:0] msg2 =  cfg[reg_idx][7:0];

assign dbg_state  = state;
assign dbg_regidx = reg_idx;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state    <= ST_IDLE;
        done     <= 1'b0;
        scl      <= 1'b1;
        sda_out  <= 1'b1;
        reg_idx  <= 4'd0;
        byte_idx <= 2'd0;
        bit_idx  <= 4'd0;
        shift    <= 8'hFF;
        wait_cnt <= 25'd0;
    end else if (!done) begin
        case (state)

            // ----------------------------------------------------------------
            // Espera inicial: WM8731 precisa de pelo menos 1 ms após VCC
            // Usamos 50 ms para garantia = 2.500.000 ciclos
            // ----------------------------------------------------------------
            ST_IDLE: begin
                scl     <= 1'b1;
                sda_out <= 1'b1;
                if (wait_cnt < 25'd2499999)
                    wait_cnt <= wait_cnt + 25'd1;
                else begin
                    wait_cnt <= 25'd0;
                    reg_idx  <= 4'd0;
                    byte_idx <= 2'd0;
                    bit_idx  <= 4'd0;
                    state    <= ST_START_A;
                end
            end

            // ----------------------------------------------------------------
            // START: SDA desce enquanto SCL está alto
            // Dura 1 fase (125 ciclos)
            // ----------------------------------------------------------------
            ST_START_A: begin
                scl     <= 1'b1;
                sda_out <= 1'b0;   // SDA cai = condição START
                if (phase_done) begin
                    // Carrega primeiro byte
                    shift    <= msg0;
                    byte_idx <= 2'd0;
                    bit_idx  <= 4'd0;
                    state    <= ST_START_B;
                end
            end

            // START_B: SCL desce para começar os bits
            ST_START_B: begin
                scl     <= 1'b0;
                sda_out <= 1'b0;
                if (phase_done)
                    state <= ST_BIT_LO;
            end

            // ----------------------------------------------------------------
            // BIT_LO: SCL baixo, coloca o bit no SDA
            //   bit 0..7: dados (MSB primeiro)
            //   bit 8:    ACK — SDA=1 (hi-Z, slave puxa baixo)
            // ----------------------------------------------------------------
            ST_BIT_LO: begin
                scl <= 1'b0;
                if (bit_idx == 4'd8)
                    sda_out <= 1'b1;   // libera SDA para ACK do slave
                else
                    sda_out <= shift[7]; // MSB atual
                if (phase_done)
                    state <= ST_BIT_HI;
            end

            // BIT_HI: SCL sobe, dado estável
            ST_BIT_HI: begin
                scl <= 1'b1;
                // sda_out inalterado
                if (phase_done) begin
                    scl   <= 1'b0;  // SCL desce ao fim da fase
                    if (bit_idx == 4'd8) begin
                        // Fim do byte — verifica se há mais bytes ou STOP
                        if (byte_idx == 2'd2) begin
                            // Último byte da mensagem → STOP
                            sda_out  <= 1'b0;
                            state    <= ST_STOP_A;
                        end else begin
                            // Próximo byte
                            byte_idx <= byte_idx + 2'd1;
                            bit_idx  <= 4'd0;
                            // Carrega próximo byte
                            case (byte_idx + 2'd1)
                                2'd1: shift <= msg1;
                                2'd2: shift <= msg2;
                                default: shift <= 8'hFF;
                            endcase
                            state <= ST_BIT_LO;
                        end
                    end else begin
                        // Próximo bit
                        shift   <= {shift[6:0], 1'b0}; // shift MSB
                        bit_idx <= bit_idx + 4'd1;
                        state   <= ST_BIT_LO;
                    end
                end
            end

            // ----------------------------------------------------------------
            // STOP: SDA sobe enquanto SCL está alto
            // ----------------------------------------------------------------
            ST_STOP_A: begin
                scl     <= 1'b1;   // SCL sobe
                sda_out <= 1'b0;   // SDA ainda baixo
                if (phase_done)
                    state <= ST_STOP_B;
            end

            ST_STOP_B: begin
                scl     <= 1'b1;
                sda_out <= 1'b1;   // SDA sobe = condição STOP
                if (phase_done)
                    state <= ST_NEXT;
            end

            // ----------------------------------------------------------------
            // Avança para próximo registrador
            // ----------------------------------------------------------------
            ST_NEXT: begin
                scl     <= 1'b1;
                sda_out <= 1'b1;
                if (reg_idx == 4'd8) begin
                    done  <= 1'b1;
                    state <= ST_DONE;
                end else begin
                    reg_idx  <= reg_idx + 4'd1;
                    byte_idx <= 2'd0;
                    bit_idx  <= 4'd0;
                    state    <= ST_START_A;
                end
            end

            ST_DONE: begin
                done    <= 1'b1;
                scl     <= 1'b1;
                sda_out <= 1'b1;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule