top_tb: top.sv top_tb.sv sm83.sv decoder.sv ppu.sv
	iverilog -g2012 -o top_tb top_tb.sv
	vvp top_tb

ppu_tb: ppu_tb.sv ppu.sv
	iverilog -g2012 -o ppu_tb ppu_tb.sv
	vvp ppu_tb

ram_tb: ram_tb.sv ram.sv
	iverilog -g2012 -o ram_tb ram_tb.sv
	vvp ram_tb