FLAGS = -g2012
TOP_VVP = top.vvp
SOURCES = rtl/*.sv sim/prim_sim.v rtl/blocks/*

all: run

compile:
	iverilog $(FLAGS) -o $(TOP_VVP) $(SOURCES)

run: compile
	vvp $(TOP_VVP)

clean:
	rm -f $(TOP_VVP)

.PHONY: all compile run clean
