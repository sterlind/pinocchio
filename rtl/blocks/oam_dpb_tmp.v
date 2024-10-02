//Copyright (C)2014-2024 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.9.03 Education (64-bit)
//Part Number: GW2A-LV18PG256C8/I7
//Device: GW2A-18
//Device Version: C
//Created Time: Wed Oct  2 03:58:40 2024

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

    oam_dpb your_instance_name(
        .douta(douta), //output [7:0] douta
        .doutb(doutb), //output [15:0] doutb
        .clka(clka), //input clka
        .ocea(ocea), //input ocea
        .cea(cea), //input cea
        .reseta(reseta), //input reseta
        .wrea(wrea), //input wrea
        .clkb(clkb), //input clkb
        .oceb(oceb), //input oceb
        .ceb(ceb), //input ceb
        .resetb(resetb), //input resetb
        .wreb(wreb), //input wreb
        .ada(ada), //input [7:0] ada
        .dina(dina), //input [7:0] dina
        .adb(adb), //input [6:0] adb
        .dinb(dinb) //input [15:0] dinb
    );

//--------Copy end-------------------
