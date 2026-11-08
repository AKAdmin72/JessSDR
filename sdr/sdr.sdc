// 27 MHz board oscillator; Gowin derives the rPLL output clock from it.
create_clock -name clk27 -period 37.037 -waveform {0 18.518} [get_ports {clk}]
