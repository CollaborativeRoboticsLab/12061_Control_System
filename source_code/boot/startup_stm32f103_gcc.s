/**
 * startup_stm32f103_gcc.s
 *
 * STM32F103C8T6 (Medium Density) 启动文件 —— GNU 汇编语法。
 *
 * 为什么需要这个文件:
 *   原工程里的 USER/startup_stm32f10x_md.s 是 Keil 编译器 (armasm) 的语法,
 *   里面全是 AREA / EXPORT / PRESERVE8 / THUMB 这类指令,GCC 一句都看不懂。
 *   这个文件做的事情完全一样,只是换成了 GCC 认识的写法。
 *
 * 它干三件事:
 *   1. 摆好中断向量表(开头那张地址表,告诉 CPU 出事了跳哪去)
 *   2. 复位后:把已初始化的全局变量从 Flash 搬到 RAM,把未初始化的清零
 *   3. 跳进 main()
 *
 * 注意:这里【不】调用 SystemInit()。
 *   原厂启动文件里那一句本来就被注释掉了(第 132-135 行,注明"寄存器版本没用到"),
 *   时钟是 main() 里第一句 Stm32_Clock_Init(9) 自己配的。保持一致。
 */

    .syntax unified
    .cpu cortex-m3
    .thumb

/* ------------------------------------------------------------------
   中断向量表
   ------------------------------------------------------------------ */
    .section .isr_vector, "a", %progbits
    .type   g_pfnVectors, %object
g_pfnVectors:
    .word   _estack                     /* 0  栈顶地址        */
    .word   Reset_Handler               /* 1  复位            */
    .word   NMI_Handler
    .word   HardFault_Handler
    .word   MemManage_Handler
    .word   BusFault_Handler
    .word   UsageFault_Handler
    .word   0
    .word   0
    .word   0
    .word   0
    .word   SVC_Handler
    .word   DebugMon_Handler
    .word   0
    .word   PendSV_Handler
    .word   SysTick_Handler

    /* --- 外设中断,共 43 个 --- */
    .word   WWDG_IRQHandler                 /* 0  */
    .word   PVD_IRQHandler                  /* 1  */
    .word   TAMPER_IRQHandler               /* 2  */
    .word   RTC_IRQHandler                  /* 3  */
    .word   FLASH_IRQHandler                /* 4  */
    .word   RCC_IRQHandler                  /* 5  */
    .word   EXTI0_IRQHandler                /* 6   <- 按键用到 */
    .word   EXTI1_IRQHandler                /* 7   <- 按键用到 */
    .word   EXTI2_IRQHandler                /* 8   <- 按键用到 */
    .word   EXTI3_IRQHandler                /* 9   <- 按键用到 */
    .word   EXTI4_IRQHandler                /* 10  <- 按键用到 */
    .word   DMA1_Channel1_IRQHandler        /* 11 */
    .word   DMA1_Channel2_IRQHandler        /* 12 */
    .word   DMA1_Channel3_IRQHandler        /* 13 */
    .word   DMA1_Channel4_IRQHandler        /* 14 */
    .word   DMA1_Channel5_IRQHandler        /* 15 */
    .word   DMA1_Channel6_IRQHandler        /* 16 */
    .word   DMA1_Channel7_IRQHandler        /* 17 */
    .word   ADC1_2_IRQHandler               /* 18 */
    .word   USB_HP_CAN1_TX_IRQHandler       /* 19 */
    .word   USB_LP_CAN1_RX0_IRQHandler      /* 20 */
    .word   CAN1_RX1_IRQHandler             /* 21 */
    .word   CAN1_SCE_IRQHandler             /* 22 */
    .word   EXTI9_5_IRQHandler              /* 23  <- 按键用到 */
    .word   TIM1_BRK_IRQHandler             /* 24 */
    .word   TIM1_UP_IRQHandler              /* 25  <- 5ms 控制中断,最重要 */
    .word   TIM1_TRG_COM_IRQHandler         /* 26 */
    .word   TIM1_CC_IRQHandler              /* 27 */
    .word   TIM2_IRQHandler                 /* 28 */
    .word   TIM3_IRQHandler                 /* 29 */
    .word   TIM4_IRQHandler                 /* 30 */
    .word   I2C1_EV_IRQHandler              /* 31 */
    .word   I2C1_ER_IRQHandler              /* 32 */
    .word   I2C2_EV_IRQHandler              /* 33 */
    .word   I2C2_ER_IRQHandler              /* 34 */
    .word   SPI1_IRQHandler                 /* 35 */
    .word   SPI2_IRQHandler                 /* 36 */
    .word   USART1_IRQHandler               /* 37  <- MATLAB 命令接收 */
    .word   USART2_IRQHandler               /* 38 */
    .word   USART3_IRQHandler               /* 39 */
    .word   EXTI15_10_IRQHandler            /* 40  <- 按键用到 */
    .word   RTCAlarm_IRQHandler             /* 41 */
    .word   USBWakeUp_IRQHandler            /* 42 */
    .size   g_pfnVectors, .-g_pfnVectors

/* ------------------------------------------------------------------
   复位入口
   ------------------------------------------------------------------ */
    .section .text.Reset_Handler
    .weak   Reset_Handler
    .type   Reset_Handler, %function
Reset_Handler:
    ldr   sp, =_estack              /* 设置栈顶 */

    /* 把 .data(有初值的全局变量)从 Flash 搬到 RAM */
    ldr   r0, =_sdata
    ldr   r1, =_edata
    ldr   r2, =_sidata
    movs  r3, #0
    b     CopyDataLoopCheck
CopyDataLoop:
    ldr   r4, [r2, r3]
    str   r4, [r0, r3]
    adds  r3, r3, #4
CopyDataLoopCheck:
    adds  r4, r0, r3
    cmp   r4, r1
    bcc   CopyDataLoop

    /* 把 .bss(没初值的全局变量)清零 */
    ldr   r2, =_sbss
    ldr   r4, =_ebss
    movs  r3, #0
    b     ZeroBssLoopCheck
ZeroBssLoop:
    str   r3, [r2]
    adds  r2, r2, #4
ZeroBssLoopCheck:
    cmp   r2, r4
    bcc   ZeroBssLoop

    bl    __libc_init_array         /* C 库初始化(用不到也无害) */
    bl    main                      /* 进主程序,正常情况永不返回 */
    b     .
    .size Reset_Handler, .-Reset_Handler

/* ------------------------------------------------------------------
   默认中断处理:死循环。
   任何没有被 C 代码实现的中断如果意外触发,程序会停在这里,
   调试时一眼就能看出"跑飞到了没实现的中断"。
   ------------------------------------------------------------------ */
    .section .text.Default_Handler, "ax", %progbits
    .type   Default_Handler, %function
Default_Handler:
Infinite_Loop:
    b     Infinite_Loop
    .size Default_Handler, .-Default_Handler

/* 下面每一行的意思是:如果 C 代码里没写这个中断函数,就用 Default_Handler 顶上。
   写了的话(比如 control.c 里的 TIM1_UP_IRQHandler、
   usart_cmd.c 里的 USART1_IRQHandler),自动用 C 里那个。 */
    .macro  def_irq_handler handler_name
    .weak   \handler_name
    .thumb_set \handler_name, Default_Handler
    .endm

    def_irq_handler NMI_Handler
    def_irq_handler HardFault_Handler
    def_irq_handler MemManage_Handler
    def_irq_handler BusFault_Handler
    def_irq_handler UsageFault_Handler
    def_irq_handler SVC_Handler
    def_irq_handler DebugMon_Handler
    def_irq_handler PendSV_Handler
    def_irq_handler SysTick_Handler
    def_irq_handler WWDG_IRQHandler
    def_irq_handler PVD_IRQHandler
    def_irq_handler TAMPER_IRQHandler
    def_irq_handler RTC_IRQHandler
    def_irq_handler FLASH_IRQHandler
    def_irq_handler RCC_IRQHandler
    def_irq_handler EXTI0_IRQHandler
    def_irq_handler EXTI1_IRQHandler
    def_irq_handler EXTI2_IRQHandler
    def_irq_handler EXTI3_IRQHandler
    def_irq_handler EXTI4_IRQHandler
    def_irq_handler DMA1_Channel1_IRQHandler
    def_irq_handler DMA1_Channel2_IRQHandler
    def_irq_handler DMA1_Channel3_IRQHandler
    def_irq_handler DMA1_Channel4_IRQHandler
    def_irq_handler DMA1_Channel5_IRQHandler
    def_irq_handler DMA1_Channel6_IRQHandler
    def_irq_handler DMA1_Channel7_IRQHandler
    def_irq_handler ADC1_2_IRQHandler
    def_irq_handler USB_HP_CAN1_TX_IRQHandler
    def_irq_handler USB_LP_CAN1_RX0_IRQHandler
    def_irq_handler CAN1_RX1_IRQHandler
    def_irq_handler CAN1_SCE_IRQHandler
    def_irq_handler EXTI9_5_IRQHandler
    def_irq_handler TIM1_BRK_IRQHandler
    def_irq_handler TIM1_UP_IRQHandler
    def_irq_handler TIM1_TRG_COM_IRQHandler
    def_irq_handler TIM1_CC_IRQHandler
    def_irq_handler TIM2_IRQHandler
    def_irq_handler TIM3_IRQHandler
    def_irq_handler TIM4_IRQHandler
    def_irq_handler I2C1_EV_IRQHandler
    def_irq_handler I2C1_ER_IRQHandler
    def_irq_handler I2C2_EV_IRQHandler
    def_irq_handler I2C2_ER_IRQHandler
    def_irq_handler SPI1_IRQHandler
    def_irq_handler SPI2_IRQHandler
    def_irq_handler USART1_IRQHandler
    def_irq_handler USART2_IRQHandler
    def_irq_handler USART3_IRQHandler
    def_irq_handler EXTI15_10_IRQHandler
    def_irq_handler RTCAlarm_IRQHandler
    def_irq_handler USBWakeUp_IRQHandler

    .end
