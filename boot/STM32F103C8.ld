/*
 * STM32F103C8.ld  ——  链接脚本
 *
 * 告诉链接器这块芯片有多少 Flash、多少 RAM,以及程序的各个部分放哪。
 * Keil 那边对应的是 OBJ/Minibalance.sct 那个文件,内容等价。
 *
 * STM32F103C8T6:  Flash 64 KB @ 0x08000000    RAM 20 KB @ 0x20000000
 *
 * 顺带说明:GCC 没有 Keil 免费版那个 32 KB 链接上限,
 * 所以原来编不过的 35,940 字节镜像在这里可以直接编过。
 */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* 栈顶放在 RAM 最高处,往下长 */
_estack    = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size  = 0x200;    /* 512 B  */
_Min_Stack_Size = 0x400;    /* 1 KB   */

SECTIONS
{
  /* 中断向量表必须在 Flash 最开头 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* 程序代码 */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* 只读数据:常量、OLED 字模都在这里 */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM       : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH

  .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .);
                     KEEP(*(.preinit_array*))
                     PROVIDE_HIDDEN(__preinit_array_end = .); } >FLASH
  .init_array    : { PROVIDE_HIDDEN(__init_array_start = .);
                     KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*))
                     PROVIDE_HIDDEN(__init_array_end = .); } >FLASH
  .fini_array    : { PROVIDE_HIDDEN(__fini_array_start = .);
                     KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*))
                     PROVIDE_HIDDEN(__fini_array_end = .); } >FLASH

  /* 有初值的全局变量:初值存在 Flash 里,开机时由启动文件搬到 RAM */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* 没初值的全局变量:开机时由启动文件清零 */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* 检查堆和栈的空间还够不够,不够就在链接时报错,而不是运行时莫名其妙崩掉 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* 注意:这里【没有】丢弃 libgcc。Cortex-M3 没有浮点硬件,
     control.c 里的 float 运算全靠 libgcc 里的软件浮点函数,丢了就链接不过。 */
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
