rgbasm -p 0xff -o boot.o boot.asm
rgblink -p 0 boot.o -o boot.bin
truncate -s 256 boot.bin
