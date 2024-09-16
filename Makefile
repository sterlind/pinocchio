top_tb: top.sv top_tb.sv sm83.sv decoder.sv
	iverilog -g2012 -o top_tb top_tb.sv
	vvp top_tb