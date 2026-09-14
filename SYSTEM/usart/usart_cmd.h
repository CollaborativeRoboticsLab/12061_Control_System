#ifndef __USART_CMD_H
#define __USART_CMD_H
#include "sys.h"

/* ============================================================
   usart_cmd.h  -  ASCII command layer for the WHEELTEC
                   linear inverted pendulum (register version)

   Replaces the compile-time LAB1_TEST_MODE switch with a
   run-time serial command, so the motor can be given a step
   input from MATLAB without recompiling and re-flashing.

   Wire format
     host -> board :  <COMMAND>\r          (CR, as MATLAB
                                            configureTerminator
                                            "CR/LF","CR" sends)
     board -> host :  <REPLY>\r\n          (CR/LF, as MATLAB reads)

   Baud rate is whatever uart_init() was called with; the stock
   firmware uses 128000, so MATLAB must open the port with
   connectSTM32(port,128000).
   ============================================================ */

#define UARTCMD_RX_BUF        48      /* one command line, incl. NUL   */
#define UARTCMD_PWM_LIMIT     6900    /* same clamp as Xianfu_Pwm()    */
#define UARTCMD_DEFAULT_MS    1000    /* motor watchdog if M has no ms */
#define UARTCMD_MAX_MS        10000   /* longest allowed motor pulse   */

/* Encoder end-stops, copied from the Auto_run() edge protection.
   These are ABSOLUTE counts, and they only describe real rail ends once
   the encoder frame has been established -- see HOME below. */
#define UARTCMD_ENC_MAX       9900
#define UARTCMD_ENC_MIN       5900

/* HOME calibration.
   Encoder_Init_TIM4 writes TIM4->CNT = 10000 at every reset, wherever the
   cart physically is, so out of reset the absolute limits above mean
   nothing: 10000 is already past ENC_MAX. The HOME command fixes that.
   The host puts the cart in the middle of the rail (by hand -- it takes a
   second and needs no motor), sends HOME, and the firmware writes the
   count to ENC_HOME and arms the limits.

   Until HOME has been sent, enc_homed is 0 and the limit test is skipped
   entirely -- testing an unknown frame would only produce confident
   nonsense. Instead every M command is capped to a nudge, which is enough
   to jog the cart under supervision but not enough to make a run at the
   end of the rail. */
#define UARTCMD_ENC_HOME      7925    /* == POSITION_MIDDLE in control.h */
#define UARTCMD_NUDGE_PWM     2500    /* M is capped to this before HOME */
#define UARTCMD_NUDGE_MS      300     /* and to this long                */

extern volatile u8  manual_mode;      /* 1 = PWM comes from the host   */
extern volatile int manual_pwm;       /* PWM the host asked for        */
extern volatile u8  stream_enable;    /* 1 = binary DataScope frames on*/
extern volatile u8  enc_homed;        /* 1 = HOME sent, limits are real */

void UartCmd_Init(void);   /* call once, after uart_init()             */
void UartCmd_Poll(void);   /* call from the main loop, as often as you can */
void UartCmd_Tick(void);   /* call from TIM1_UP_IRQHandler (every 5 ms)*/
void UartCmd_Abort(void);  /* kill manual mode from anywhere           */

#endif
