INCLUDE "hardware.inc"

SECTION "boot", ROM0[$000]
EntryPoint:
    ld b, $00
    ld c, $13
    ld d, $00
    ld e, $d8
    ld h, $01
    ld l, $4d
    ld sp, $fffe
    ld a, $40
    ldh [rLCDC], a
    ld a, $00
    ldh [rSCY], a
    ldh [rSCX], a
    ldh [rLYC], a
    ldh [rWY], a
    ldh [rWX], a
    ld a, $91
    ldh [rLY], a
    ld a, $fc
    ldh [rBGP], a
    ld a, $41
    ldh [rSTAT], a
    jp Done

SECTION "end", ROM0[$fc]
Done:
    ld a, $01
    ldh [$ff50], a