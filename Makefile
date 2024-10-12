FLAGS = -g2012
SOURCES = rtl/*.sv sim/prim_sim.v rtl/blocks/*

all: run

compile:
	iverilog -o top.vvp -g2012 -Y .sv sim/top_tb.sv sim/prim_sim.v rtl/debounce.v rtl/*.sv rtl/blocks/*.v

run: compile
	vvp top.vvp

.PHONY: all compile run
