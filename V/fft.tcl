# =============================================================================
# fft.tcl  —  Atribuições de pinos para o projeto fft na DE2
# Importar via: Assignments → Import Assignments → selecionar fft.tcl
# Ou: quartus_sh --tcl_eval source fft.tcl
# =============================================================================

# Dispositivo e família
set_global_assignment -name FAMILY "Cyclone II"
set_global_assignment -name DEVICE EP2C35F672C6
set_global_assignment -name TOP_LEVEL_ENTITY fft

# Pinos não conectados como entrada tri-state (evita warnings e oscilação)
set_global_assignment -name RESERVE_ALL_UNUSED_PINS "AS INPUT TRI-STATED"

# ---------------------------------------------------------------------------
# Arquivos fonte
# ---------------------------------------------------------------------------
set_global_assignment -name VERILOG_FILE fft.v
set_global_assignment -name VERILOG_FILE wm8731_i2c_config.v
set_global_assignment -name VERILOG_FILE i2s_receiver.v
set_global_assignment -name VERILOG_FILE fft1024_mag.v

# ---------------------------------------------------------------------------
# Clock principal 50 MHz
# ---------------------------------------------------------------------------
set_location_assignment PIN_N2  -to CLOCK_50

# ---------------------------------------------------------------------------
# Clock de 27 MHz do TV decoder (não usado, mas deve ser assignado para evitar
# que o roteador use o pino de forma errada)
# ---------------------------------------------------------------------------
set_location_assignment PIN_D13 -to CLOCK_27

# ---------------------------------------------------------------------------
# TD_RESET: deve ser mantido em 1 para que o TV decoder gere o clock de 27 MHz
# Se não for usado no projeto, o Quartus coloca HIGH via pull-up.
# Adicionamos como saída constante HIGH no top-level via assign.
# ---------------------------------------------------------------------------
set_location_assignment PIN_C4  -to TD_RESET

# ---------------------------------------------------------------------------
# KEY (reset ativo-baixo)
# ---------------------------------------------------------------------------
set_location_assignment PIN_G26 -to KEY[0]
set_location_assignment PIN_N23 -to KEY[1]
set_location_assignment PIN_P23 -to KEY[2]
set_location_assignment PIN_W26 -to KEY[3]

# ---------------------------------------------------------------------------
# WM8731 Audio CODEC
# ---------------------------------------------------------------------------
set_location_assignment PIN_C5  -to AUD_ADCLRCK
set_location_assignment PIN_B5  -to AUD_ADCDAT
set_location_assignment PIN_C6  -to AUD_DACLRCK
set_location_assignment PIN_A4  -to AUD_DACDAT
set_location_assignment PIN_A5  -to AUD_XCK
set_location_assignment PIN_B4  -to AUD_BCLK

# ---------------------------------------------------------------------------
# I2C (compartilhado com TV decoder, mas endereços diferentes)
# ---------------------------------------------------------------------------
set_location_assignment PIN_A6  -to I2C_SCLK
set_location_assignment PIN_B6  -to I2C_SDAT

# ---------------------------------------------------------------------------
# LCD HD44780 modo 4 bits — nibble alto (D7..D4)
# Tabela 4.6 do manual DE2:
#   LCD_DATA[7] → PIN_H3  (não H4 como estava errado antes)
#   LCD_DATA[6] → PIN_H4
#   LCD_DATA[5] → PIN_J3
#   LCD_DATA[4] → PIN_J4
# NOTA: O port no top-level é "output wire [7:4] LCD_DATA"
#       que mapeia: LCD_DATA[7]=bit3 do port, ..., LCD_DATA[4]=bit0 do port
# ---------------------------------------------------------------------------
set_location_assignment PIN_H3  -to LCD_DATA[7]
set_location_assignment PIN_H4  -to LCD_DATA[6]
set_location_assignment PIN_J3  -to LCD_DATA[5]
set_location_assignment PIN_J4  -to LCD_DATA[4]
set_location_assignment PIN_K4  -to LCD_RW
set_location_assignment PIN_K3  -to LCD_EN
set_location_assignment PIN_K1  -to LCD_RS
set_location_assignment PIN_L4  -to LCD_ON
set_location_assignment PIN_K2  -to LCD_BLON

# ---------------------------------------------------------------------------
# Displays 7-segmentos HEX0..HEX3 (tabela 4.4 do manual)
# ---------------------------------------------------------------------------
# HEX0
set_location_assignment PIN_AF10 -to HEX0[0]
set_location_assignment PIN_AB12 -to HEX0[1]
set_location_assignment PIN_AC12 -to HEX0[2]
set_location_assignment PIN_AD11 -to HEX0[3]
set_location_assignment PIN_AE11 -to HEX0[4]
set_location_assignment PIN_V14  -to HEX0[5]
set_location_assignment PIN_V13  -to HEX0[6]

# HEX1
set_location_assignment PIN_V20  -to HEX1[0]
set_location_assignment PIN_V21  -to HEX1[1]
set_location_assignment PIN_W21  -to HEX1[2]
set_location_assignment PIN_Y22  -to HEX1[3]
set_location_assignment PIN_AA24 -to HEX1[4]
set_location_assignment PIN_AA23 -to HEX1[5]
set_location_assignment PIN_AB24 -to HEX1[6]

# HEX2
set_location_assignment PIN_AB23 -to HEX2[0]
set_location_assignment PIN_V22  -to HEX2[1]
set_location_assignment PIN_AC25 -to HEX2[2]
set_location_assignment PIN_AC26 -to HEX2[3]
set_location_assignment PIN_AB26 -to HEX2[4]
set_location_assignment PIN_AB25 -to HEX2[5]
set_location_assignment PIN_Y24  -to HEX2[6]

# HEX3
set_location_assignment PIN_Y23  -to HEX3[0]
set_location_assignment PIN_AA25 -to HEX3[1]
set_location_assignment PIN_AA26 -to HEX3[2]
set_location_assignment PIN_Y26  -to HEX3[3]
set_location_assignment PIN_Y25  -to HEX3[4]
set_location_assignment PIN_U22  -to HEX3[5]
set_location_assignment PIN_W24  -to HEX3[6]

# HEX4 (5o digito da frequencia, tabela 4.4 do manual)
set_location_assignment PIN_U9   -to HEX4[0]
set_location_assignment PIN_U1   -to HEX4[1]
set_location_assignment PIN_U2   -to HEX4[2]
set_location_assignment PIN_T4   -to HEX4[3]
set_location_assignment PIN_R7   -to HEX4[4]
set_location_assignment PIN_R6   -to HEX4[5]
set_location_assignment PIN_T3   -to HEX4[6]

# ---------------------------------------------------------------------------
# LEDs vermelhos LEDR[0..17]  (tabela 4.3 do manual)
# ---------------------------------------------------------------------------
set_location_assignment PIN_AE23 -to LEDR[0]
set_location_assignment PIN_AF23 -to LEDR[1]
set_location_assignment PIN_AB21 -to LEDR[2]
set_location_assignment PIN_AC22 -to LEDR[3]
set_location_assignment PIN_AD22 -to LEDR[4]
set_location_assignment PIN_AD23 -to LEDR[5]
set_location_assignment PIN_AD21 -to LEDR[6]
set_location_assignment PIN_AC21 -to LEDR[7]
set_location_assignment PIN_AA14 -to LEDR[8]
set_location_assignment PIN_Y13  -to LEDR[9]
set_location_assignment PIN_AA13 -to LEDR[10]
set_location_assignment PIN_AC14 -to LEDR[11]
set_location_assignment PIN_AD15 -to LEDR[12]
set_location_assignment PIN_AE15 -to LEDR[13]
set_location_assignment PIN_AF13 -to LEDR[14]
set_location_assignment PIN_AE13 -to LEDR[15]
set_location_assignment PIN_AE12 -to LEDR[16]
set_location_assignment PIN_AD12 -to LEDR[17]

# ---------------------------------------------------------------------------
# Constraints de timing
# ---------------------------------------------------------------------------
set_global_assignment -name TIMEQUEST_MULTICORNER_TIMING_ANALYSIS_ENABLE ON
