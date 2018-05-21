ENTRY(Startup)
MEMORY
{
    vtab : ORIGIN = 0x00000000,      LENGTH = 196
    rom  : ORIGIN = 197,             LENGTH = 255804 /* starting at end of vtab, to end of rom */
    ram  : ORIGIN = 0x1FFFE000,      LENGTH = 32K
    /*sram_l: ORIGIN = 0x1FFFE000, LENGTH = 0x1FFF
    sram_h: ORIGIN = 0x20000000, LENGTH = 0x5FFF*/
}

SECTIONS
{
 . = . + 0x8000;
 vtab  : { *(.vtab) } > vtab
 .text : { *(.text*) } > rom
 .data : { *(.data*) } > rom
 .bss  : { *(.bss COMMON) } > ram
 . = ALIGN(8);
 . = . + 0x1000;
 stack_top = .;
}
