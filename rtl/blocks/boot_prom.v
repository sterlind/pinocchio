//Copyright (C)2014-2024 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//Tool Version: V1.9.9.03 Education (64-bit)
//Part Number: GW2A-LV18PG256C8/I7
//Device: GW2A-18
//Device Version: C
//Created Time: Sun Sep 22 18:30:24 2024

module boot_prom (dout, clk, oce, ce, reset, ad);

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
defparam prom_inst_0.INIT_RAM_00 = 256'hE0FC3E77773E32E2F33E0CE232803E110EFF2621FB207CCB329FFF21AFFFFE31;
defparam prom_inst_0.INIT_RAM_01 = 256'hF920052322131A080600D811F32034FE7B130096CD0095CD1A80102101041147;
defparam prom_inst_0.INIT_RAM_02 = 256'h0440E0913E42E057643E67F3180F2EF9200D3208283D0C0E992F219910EA193E;
defparam prom_inst_0.INIT_RAM_03 = 256'h062064FEC11E062862FE831E7C24130EF2201DF7200DFA2090FE44F00C0E021E;
defparam prom_inst_0.INIT_RAM_04 = 256'h1711CBC11711CBC504064FCB1820164F2005D2201542E09042F0E2873E0CE27B;
defparam prom_inst_0.INIT_RAM_05 = 256'h0E0089881F1108000D000C00830073030B000DCC6666EDCEC923222322F52005;
defparam prom_inst_0.INIT_RAM_06 = 256'h3C42A5B9A5B9423C3E33B9BB9F99DCDDCCEC0E6E6367BBBB99D9DDDDE66ECCDC;
defparam prom_inst_0.INIT_RAM_07 = 256'h50E0013EFE2086FB20052386781906F52034FE7D23FE20BE131A00A811010421;

endmodule //boot_prom
