OUTPUT_ARCH(arm)
ENTRY(__start)
MEMORY
{
  ROM (rx) : ORIGIN = 0x00600000, LENGTH = 127926272
  RAM (rwx): ORIGIN = 0x40200000, LENGTH = 132120576
}
SECTIONS
{
  .text : {
      _stext = .;
      __text_start = .;
      *(.vectors)
      *(.text*)
      *(.fixup)
      *(.gnu.warning)
  } > ROM
  . = ALIGN(4096);
  .init_section : {
      _sinit = ABSOLUTE(.);
      KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
      KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
      _einit = ABSOLUTE(.);
      . = ALIGN(4096);
      _etext = .;
      _sztext = _etext - _stext;
  } > ROM
  .ARM.extab :
  {
      *(.ARM.extab*)
  } > ROM
  PROVIDE_HIDDEN (__exidx_start = .);
  .ARM.exidx :
  {
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > ROM
  PROVIDE_HIDDEN (__exidx_end = .);
  .rodata : {
      _srodata = .;
      *(.rodata*)
      *(.data.rel.ro*)
      KEEP(*(SORT(.scattered_array*)));
      . = ALIGN(4096);
      _erodata = .;
  } > ROM
  _eronly = LOADADDR(.data);
  .data : {
      _sdata = .;
      *(.data*)
      . = ALIGN(8);
      __start_impls = .;
      *(.impls)
      KEEP(*(.impls))
      . = ALIGN(4);
      __stop_impls = .;
      _edata = .;
  } > RAM AT > ROM
  .noinit : {
      _snoinit = ABSOLUTE(.);
      *(.noinit*)
      _enoinit = ABSOLUTE(.);
  } > RAM
  .bss : {
       _sbss = .;
       *(.bss*)
       . = ALIGN(4096);
       _ebss = .;
  } > RAM
  /DISCARD/ : {
       *(.exit.text)
       *(.exit.data)
       *(.exitcall.exit)
  }
  .stab 0 : { *(.stab) }
  .stabstr 0 : { *(.stabstr) }
  .stab.excl 0 : { *(.stab.excl) }
  .stab.exclstr 0 : { *(.stab.exclstr) }
  .stab.index 0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment 0 : { *(.comment) }
}
