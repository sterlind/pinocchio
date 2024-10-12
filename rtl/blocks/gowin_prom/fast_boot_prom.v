//Copyright (C)2014-2024 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//Tool Version: V1.9.9.03 Education (64-bit)
//Part Number: GW2A-LV18PG256C8/I7
//Device: GW2A-18
//Device Version: C
//Created Time: Fri Oct 11 18:52:01 2024

module fast_boot_prom (dout, clk, oce, ce, reset, ad);

output [7:0] dout;
input clk;
input oce;
input ce;
input reset;
input [7:0] ad;

wire [23:0] prom_inst_0_dout_w;
wire gw_gnd;

assign gw_gnd = 1'b0;

pROM prom_inst_0 (
    .DO({prom_inst_0_dout_w[23:0],dout[7:0]}),
    .CLK(clk),
    .OCE(oce),
    .CE(ce),
    .RESET(reset),
    .AD({gw_gnd,gw_gnd,gw_gnd,ad[7:0],gw_gnd,gw_gnd,gw_gnd})
);

defparam prom_inst_0.READ_MODE = 1'b0;
defparam prom_inst_0.BIT_WIDTH = 8;
defparam prom_inst_0.RESET_MODE = "SYNC";
defparam prom_inst_0.INIT_RAM_00 = 256'h3E4BE04AE045E043E042E0003E40E0403EFFFE314D2E0126D81E0016130E0006;
defparam prom_inst_0.INIT_RAM_01 = 256'h00000000000000000000000000000000000000FCC341E0413E47E0FC3E44E091;
defparam prom_inst_0.INIT_RAM_02 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
defparam prom_inst_0.INIT_RAM_03 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
defparam prom_inst_0.INIT_RAM_04 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
defparam prom_inst_0.INIT_RAM_05 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
defparam prom_inst_0.INIT_RAM_06 = 256'h0000000000000000000000000000000000000000000000000000000000000000;
defparam prom_inst_0.INIT_RAM_07 = 256'h50E0013E00000000000000000000000000000000000000000000000000000000;

endmodule //fast_boot_prom
