/* Entry point for application */
ENTRY(Startup)

/* Optional, but allows ld to warn when application does not fit in memory. */
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
    . = . + 0x8000;    /* for OpenSDAv2.1 */
    vtab  : { *(.vtab) } > vtab
    .text : { *(.text*) } > rom
    .data : { *(.data*) } > rom
    .bss  : {
            /* allocate the stack */
            . = ALIGN(8);
            . = . + 0x0100;
            stack_top = .;
            /* insert and space variables appropriately */
           *(.bss COMMON) } > ram
}
