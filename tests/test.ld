ENTRY(test_entry)
MEMORY { RAM (rwx) : ORIGIN = 0x10000, LENGTH = 4M }
SECTIONS {
    .text : { *(.text*) *(.rodata*) } > RAM
    .data : { *(.data*) } > RAM
    .bss : { *(.bss*) *(COMMON) } > RAM
    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) }
}
