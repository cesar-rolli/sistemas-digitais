# FFT em VHDL
Os scripts serão feitos em VHDL com o objetivo de realizar uma Fast Fourier Transformation (FFT) no FPGA da música em tempo real e exibir no monitor com conexão VGA as frequências fundamentais em tempo real.

O arquivo em Python é para fazer uma validação do script em VHDL, exibindo a FFT em tempo real em conjunto com a placa.

> No mac: habilitar o BlackHole 2ch

## Programas usados
1. 

## Fluxograma de construção
1. Fazer o código da FFT,
2. Testar no Model SIM,
3. Fazer o código do input (codec),
4. Fazer o código do output (monitor VGA),
5. Testar tudo:
    1. Função seno simples (wt),
    2. Série de Fourier (senos + cossenos),
    3. Música.

## Fluxograma do sinal
[Line-in azul]
    
↓

WM8731 ADC - I2S (AUD_ADCDAT, BCLK, LRCLK)

↓

Deserializador I2S (configurado via I2C na init) - 24 bits (pega 12 MSBs)
    
↓

Buffer/FIFO (acumula N amostras - 512 ou 1024)
    
↓

FFT (Cooley-Tukey)

↓

Display VGA

## Tópicos úteis
- Cooley–Tukey FFT algorithm
- FFT Radix-2

## Links úteis
- [FFT-using-Verilog-RADIX-2](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://github.com/Devashrutha/FFT-using-Verilog-RADIX-2&ved=2ahUKEwiH7K2Z1deUAxUElZUCHbm7IbQQFnoECDUQAQ&usg=AOvVaw1CnNzWro_sjdRqekM7obye)

- [Audio Effects](https://github.com/Reenforcements/VerilogDE2115AudioFilters)

- [Download Quartus II](https://www.altera.com/downloads/fpga-development-tools/quartus-ii-web-edition-design-software-version-12-1-b177-windows)

- [Download Model SIM](https://www.altera.com/downloads/simulation-tools/modelsim-fpgas-standard-edition-software-version-20-1)

![fft](/fft.png)