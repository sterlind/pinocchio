top_tb: top_tb.sv
	iverilog -g2012 -o top_tb top_tb.sv
	vvp top_tb