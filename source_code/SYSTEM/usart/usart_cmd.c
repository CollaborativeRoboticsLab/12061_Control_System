/* ============================================================
   usart_cmd.c  -  ASCII command layer for the WHEELTEC
                   linear inverted pendulum (register version)

   Pure ASCII source on purpose: no GBK characters, so it
   compiles identically under armcc (Keil) and arm-none-eabi-gcc
   (PlatformIO) with or without -finput-charset=GBK.

   Command set (case-insensitive on the wire; MATLAB's
   sendSTM32Command already upper-cases everything):

     PING                -> PONG
     VER                 -> VER=<firmware build id>
     A                   -> ANGLE=<0..4095>          raw ADC counts
     P                   -> POS=<encoder>            TIM4 counts
     V                   -> VOLT=<hundredths of a volt, e.g. 1184 = 11.84 V>
     M,<pwm>             -> PWM=<pwm>                run for 1000 ms
     M,<pwm>,<ms>        -> PWM=<pwm>                run for <ms> ms
     STOP                -> STOPPED
     BAL,1 / BAL,0       -> BAL=1 / BAL=0            balance controller
     SWING               -> SWING=1                  automatic swing-up
     ZERO                -> ZERO=<encoder>           set Position_Zero
     HOME                -> HOME=<encoder>           cart is mid-rail:
                                                     set the count and
                                                     arm the end-stops
     HOMED               -> HOMED=0 / HOMED=1        are they armed?
     GAIN,<name>,<value> -> GAIN=<name>,<value>      BKP BKD BKI PKP PKD
                                                     PKI AZ   (R4-6)
     K                   -> K=<bkp>,<bkd>,<bki>,<pkp>,<pkd>,<pki>,<az>
     IRESET              -> IRESET=0                 zero both integrators
     KICK,<pwm>[,<ms>]   -> KICK=<pwm>               timed disturbance
     STATUS              -> STATUS=<10 integers>     see below
     STREAM,1 / STREAM,0 -> STREAM=1 / STREAM=0      binary DataScope
     anything else       -> ERR=UNKNOWN

   STATUS fields, in order (field 1 signed, fields 2..9 unsigned,
   which is what the existing readStatus.m regexp expects):

     1 Moto            current PWM command, -6900..6900
     2 Encoder         cart position, TIM4 counts
     3 Angle           pendulum ADC counts, 0..4095
     4 Voltage         battery, hundredths of a volt (1184 = 11.84 V)
     5 Flag_Stop       1 = motor disabled
     6 manual_mode     1 = PWM is coming from the host
     7 auto_run        1 = automatic swing-up selected
     8 stream_enable   1 = binary DataScope frames are being sent
     9 rx_errors       dropped bytes: overrun + line overflow
    10 kick_ticks     5 ms units of disturbance still to run, 0 = idle

   Fields are APPEND-ONLY. A host that knows about nine of them must keep
   working against a board that sends ten, so readStatusWheeltec.m splits
   on commas and requires "at least 9" rather than matching exactly nine
   and anchoring the end of the line. Never reorder or remove a field:
   both sides address them by position.

   Streaming interlock
     stream_enable starts at 1, so a board that is never talked to
     behaves exactly like the stock firmware and the WHEELTEC
     upper-computer still works. The first valid ASCII command
     received switches streaming OFF automatically, so MATLAB never
     has to fight binary frames on the same UART. STREAM,1 turns it
     back on deliberately.
   ============================================================ */

#include "usart_cmd.h"
#include "control.h"
#include "usart.h"

/* Bump this whenever this file changes in a way the host must notice.
   The host checks it on connect, so "did I actually reflash?" stops
   being a question anyone has to guess about.
     R4-1  first version
     R4-2  USART1 preemption priority 0 (was 1, same as TIM1, which cost
           bytes to overrun); overrun now discards the whole line
     R4-3  End-stop guard is now DIRECTIONAL. The encoder is relative --
           TIM4->CNT is set to 10000 at boot wherever the cart happens to
           be -- so a blanket "refuse outside 5900..9900" test rejected
           every command on a freshly reset board, which reads exactly
           10000. The guard now refuses a command only when the cart is at
           a limit AND the command would push it further into that limit,
           so the cart can always drive back out.
     R4-4  The directional guard in R4-3 still hard-coded which way is
           which. If the PWM sign convention on a rig were the other way
           round, the guard would refuse exactly the direction that
           escapes the limit, and the cart could not be recovered over
           the serial link at all. So the limits are now ARMED, not
           assumed: they are skipped until the host sends HOME, and
           HOME writes the encoder count to a known place (the middle of
           the rail, where the student has just put the cart by hand).
           Before HOME every M command is capped to a nudge. That also
           removes the 2075-count blind motor drive the host used to
           need, and it makes Position_Zero right for swing-up.
     R4-5  Angle loop gained an INTEGRAL term, for Lab 4. The stock
           Balance() was pure PD, so "add I and watch what happens" was
           not something the rig could do at all. New gain name BKI, and
           K now reports FIVE values (bkp,bkd,bki,pkp,pkd) rather than
           four. New command IRESET zeroes the accumulator without
           touching the gains. Ki defaults to 0, so an untouched rig
           behaves exactly as it did under R4-4.
     R4-6  Lab 5 needs to compare controllers by how each recovers from
           THE SAME disturbance, so the disturbance has to be repeatable.
           A hand push is not: the spread between two pushes is wider than
           the spread between two controllers, which would make the whole
           experiment measure the experimenter's hand. New command KICK
           adds a timed PWM offset ON TOP of the balance output, so the
           controller stays in charge and what is measured is its
           recovery. M could not do this job -- it sets manual_mode, which
           REPLACES the balance loop, so an M sent while balancing simply
           drops the rod.
           STATUS gained a tenth field (kick ticks remaining) so the host
           can align recovery time to the exact instant of the kick rather
           than to a host-side timestamp with a serial round trip in it.
           The angle setpoint left control.h and became the run-time
           variable Angle_Zero (GAIN,AZ): a rig whose potentiometer is a
           couple of degrees out used to need a per-rig edit and reflash.
           Position_KI added for symmetry and, mainly, so Lab 5's claim
           about where integral action belongs can be tested rather than
           asserted. Both default to their R4-5 behaviour.              */
#define UARTCMD_VERSION "R4-6"

/* --- globals owned by the stock firmware -------------------- */
extern u8    Flag_Stop;
extern int   Encoder, Position_Zero;
extern int   Moto;
extern int   Voltage;
extern float Angle_Balance;
extern float Balance_KP, Balance_KD, Position_KP, Position_KD;
extern float Balance_KI, Balance_Integral;       /* R4-5 */
extern float Position_KI, Position_Integral;     /* R4-6 */
extern float Angle_Zero;                         /* R4-6 */
extern u8    auto_run, autorun_step0, autorun_step1, autorun_step2;
extern u8    success_flag, Swing_up;
extern long  success_count, wait_count;
extern void  Write_Encoder(u8 TIMX,int value);   /* encoder.c */

/* --- state exported to control.c ---------------------------- */
volatile u8  manual_mode   = 0;
volatile int manual_pwm    = 0;
volatile u8  stream_enable = 1;
volatile u8  enc_homed     = 0;   /* limits are off until HOME arrives */
volatile int kick_pwm      = 0;   /* R4-6: disturbance, summed into Moto */
volatile u32 kick_ticks    = 0;   /* R4-6: 5 ms units left, 0 = idle     */

/* --- private state ------------------------------------------ */
static volatile char rx_line[UARTCMD_RX_BUF];   /* being filled by ISR */
static volatile u8   rx_len   = 0;
static volatile char cmd_line[UARTCMD_RX_BUF];  /* handed to main loop */
static volatile u8   cmd_ready = 0;
static volatile u32  rx_errors = 0;
static volatile u8   rx_corrupt = 0;    /* a byte was lost in this line */
static volatile u32  manual_ticks = 0;          /* 5 ms units, 0 = idle */
static u8   first_command_seen = 0;

/* ============================================================
   Transmit helpers.  No printf: the stock retarget in usart.c
   drags in the whole C library and costs ~2 kB of flash that
   this board (35.9 kB image, 32 kB Keil-Lite limit) cannot spare.
   ============================================================ */
/* UARTCMD_PUTC is a seam so the parser can be exercised on a PC
   (see test/test_usart_cmd.c).  In a firmware build it expands to
   exactly the same two lines DataScope() uses.                    */
#ifndef UARTCMD_PUTC
#define UARTCMD_PUTC(c) do {                       \
            while((USART1->SR & 0x40) == 0);       \
            USART1->DR = (u8)(c);                  \
        } while(0)
#endif

static void cmd_putc(char c)
{
    UARTCMD_PUTC(c);                   /* wait for TC, as DataScope does */
}

static void cmd_puts(const char *s)
{
    while(*s) cmd_putc(*s++);
}

static void cmd_putint(long v)
{
    char buf[12];
    u8   n = 0;
    unsigned long u;

    if(v < 0) { cmd_putc('-'); u = (unsigned long)(-v); }
    else      {                u = (unsigned long)( v); }

    if(u == 0) { cmd_putc('0'); return; }

    while(u) { buf[n++] = (char)('0' + (u % 10)); u /= 10; }
    while(n) cmd_putc(buf[--n]);
}

/* one decimal place, for the PID gains */
static void cmd_putgain(float g)
{
    long scaled = (long)(g * 10.0f + (g >= 0 ? 0.5f : -0.5f));
    cmd_putint(scaled / 10);
    cmd_putc('.');
    cmd_putint(scaled >= 0 ? (scaled % 10) : (-scaled % 10));
}

static void cmd_end(void)
{
    cmd_putc('\r');
    cmd_putc('\n');
}

static void reply_str(const char *s)
{
    cmd_puts(s);
    cmd_end();
}

static void reply_int(const char *tag, long v)
{
    cmd_puts(tag);
    cmd_putint(v);
    cmd_end();
}

/* ============================================================
   Receive
   ============================================================ */
void USART1_IRQHandler(void)
{
    u16 sr = USART1->SR;

    if(sr & (1 << 3))              /* ORE: must read DR to clear */
    {
        (void)USART1->DR;
        rx_errors++;
        rx_corrupt = 1;            /* whole line is suspect now */
        return;
    }

    if(sr & (1 << 5))              /* RXNE */
    {
        char ch = (char)(USART1->DR & 0xFF);

        if(ch == '\r' || ch == '\n')
        {
            if(rx_len > 0)
            {
                if(rx_corrupt)
                {
                    /* A byte was lost somewhere in this line. Drop the
                       whole thing rather than risk parsing a mangled
                       command -- silently, so the host simply retries
                       instead of getting a misleading ERR= reply. */
                }
                else if(cmd_ready == 0)     /* main loop is free */
                {
                    u8 i;
                    for(i = 0; i < rx_len; i++) cmd_line[i] = rx_line[i];
                    cmd_line[rx_len] = '\0';
                    cmd_ready = 1;
                }
                else
                {
                    rx_errors++;            /* host is talking too fast */
                }
                rx_len = 0;
            }
            rx_corrupt = 0;                 /* fresh start for next line */
            /* bare CR/LF: ignore, keeps CRLF hosts happy */
        }
        else if(rx_len < (UARTCMD_RX_BUF - 1))
        {
            rx_line[rx_len++] = ch;
        }
        else
        {
            rx_len = 0;                     /* line too long, drop it */
            rx_errors++;
        }
    }
}

/* ============================================================
   Tiny parser
   ============================================================ */
static char up(char c)
{
    return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c;
}

static u8 at_end(const char *p)
{
    while(*p == ' ') p++;
    return (*p == '\0');
}

/* case-insensitive compare of *p against a literal; on match,
   advances *pp past it and returns 1.  *pp is left untouched
   when the literal does not match. */
static u8 eat(const char **pp, const char *lit)
{
    const char *p = *pp;
    while(*lit)
    {
        if(up(*p) != *lit) return 0;
        p++; lit++;
    }
    *pp = p;
    return 1;
}

/* whole line equals the literal, nothing after it */
static u8 is_cmd(const char *p, const char *lit)
{
    return eat(&p, lit) && at_end(p);
}

/* returns 1 if an integer was parsed */
static u8 get_int(const char **pp, long *out)
{
    const char *p = *pp;
    long v = 0;
    u8   neg = 0, digits = 0;

    while(*p == ' ' || *p == ',') p++;
    if(*p == '-') { neg = 1; p++; }
    else if(*p == '+') p++;

    while(*p >= '0' && *p <= '9')
    {
        v = v * 10 + (*p - '0');
        p++; digits++;
        if(digits > 9) return 0;
    }
    if(!digits) return 0;

    *out = neg ? -v : v;
    *pp  = p;
    return 1;
}

/* ============================================================
   Safety
   ============================================================ */
void UartCmd_Abort(void)
{
    manual_mode  = 0;
    manual_pwm   = 0;
    manual_ticks = 0;
    kick_pwm     = 0;              /* R4-6: a stop cancels the kick too */
    kick_ticks   = 0;
}

/* called from TIM1_UP_IRQHandler, every 5 ms */
void UartCmd_Tick(void)
{
    if(manual_ticks)
    {
        if(--manual_ticks == 0) manual_pwm = 0;   /* watchdog expired */
    }

    /* R4-6: the disturbance pulse expires the same way, and for the same
       reason. If the host crashes or the cable is pulled mid-kick, a
       permanent PWM offset must not be left summed into the controller's
       output -- the rod would be fighting a bias nobody can see. */
    if(kick_ticks)
    {
        if(--kick_ticks == 0) kick_pwm = 0;
    }
}

/* ============================================================
   Command dispatch
   ============================================================ */
static void ProcessCommand(const char *p)
{
    long a, b;

    while(*p == ' ') p++;

    /* --- the first ASCII command silently mutes DataScope ---- */
    if(!first_command_seen)
    {
        first_command_seen = 1;
        stream_enable = 0;
    }

    /* --- single-letter and fixed queries, tested whole-line --- */
    if(is_cmd(p, "PING"))   { reply_str("PONG");                        return; }
    if(is_cmd(p, "VER"))    { reply_str("VER=" UARTCMD_VERSION);        return; }
    if(is_cmd(p, "A"))      { reply_int("ANGLE=", (long)Angle_Balance); return; }
    if(is_cmd(p, "P"))      { reply_int("POS=",   (long)Encoder);       return; }
    if(is_cmd(p, "V"))      { reply_int("VOLT=",  (long)Voltage);       return; }

    if(is_cmd(p, "K"))
    {
        cmd_puts("K=");
        cmd_putgain(Balance_KP);  cmd_putc(',');
        cmd_putgain(Balance_KD);  cmd_putc(',');
        cmd_putgain(Balance_KI);  cmd_putc(',');   /* R4-5 */
        cmd_putgain(Position_KP); cmd_putc(',');
        cmd_putgain(Position_KD); cmd_putc(',');
        cmd_putgain(Position_KI); cmd_putc(',');   /* R4-6 */
        cmd_putgain(Angle_Zero);                   /* R4-6 */
        cmd_end();
        return;
    }

    if(is_cmd(p, "STOP"))
    {
        UartCmd_Abort();
        Flag_Stop = 1;
        Moto = 0;
        Set_Pwm(0);
        reply_str("STOPPED");
        return;
    }

    if(is_cmd(p, "SWING"))
    {
        UartCmd_Abort();
        auto_run      = 1;
        autorun_step0 = 0;
        autorun_step1 = 1;
        autorun_step2 = 0;
        success_flag  = 0;
        success_count = 0;
        wait_count    = 0;
        Swing_up      = 1;
        Flag_Stop     = 0;
        reply_str("SWING=1");
        return;
    }

    if(is_cmd(p, "ZERO"))
    {
        Position_Zero = Encoder;
        reply_int("ZERO=", (long)Position_Zero);
        return;
    }

    /* HOME: "the cart is now in the middle of the rail." Declare the
       encoder frame rather than trying to deduce it. Writing TIM4->CNT
       directly is what makes every absolute number in the stock
       firmware -- POSITION_MIDDLE, the 5900/9900 edge protection, the
       Auto_run target offsets -- mean what it says. */
    if(is_cmd(p, "HOME"))
    {
        UartCmd_Abort();                       /* never home under power */
        Write_Encoder(4, UARTCMD_ENC_HOME);
        Encoder       = UARTCMD_ENC_HOME;
        Position_Zero = UARTCMD_ENC_HOME;
        enc_homed     = 1;
        reply_int("HOME=", (long)UARTCMD_ENC_HOME);
        return;
    }

    /* R4-5: zero the angle-loop integrator without disturbing the gains.
       Needed between Lab 4 trials: a run that ended with the rod on the
       bench leaves a wound-up accumulator, and the next trial would start
       with a step of integral action that has nothing to do with the
       gains under test. */
    if(is_cmd(p, "IRESET"))
    {
        Balance_Integral  = 0;
        Position_Integral = 0;          /* R4-6: both, or neither */
        reply_int("IRESET=", 0);
        return;
    }

    if(is_cmd(p, "HOMED"))
    {
        reply_int("HOMED=", (long)enc_homed);
        return;
    }

    if(is_cmd(p, "STATUS"))
    {
        cmd_puts("STATUS=");
        cmd_putint((long)Moto);          cmd_putc(',');
        cmd_putint((long)Encoder);       cmd_putc(',');
        cmd_putint((long)Angle_Balance); cmd_putc(',');
        cmd_putint((long)Voltage);       cmd_putc(',');
        cmd_putint((long)Flag_Stop);     cmd_putc(',');
        cmd_putint((long)manual_mode);   cmd_putc(',');
        cmd_putint((long)auto_run);      cmd_putc(',');
        cmd_putint((long)stream_enable); cmd_putc(',');
        cmd_putint((long)rx_errors);     cmd_putc(',');
        cmd_putint((long)kick_ticks);    /* R4-6: field 10, append-only */
        cmd_end();
        return;
    }

    /* --- M,<pwm>[,<ms>] : the replacement for LAB1_TEST_MODE -- */
    if(eat(&p, "M"))
    {
        if(!get_int(&p, &a)) { reply_str("ERR=SYNTAX"); return; }

        if(a >  UARTCMD_PWM_LIMIT) a =  UARTCMD_PWM_LIMIT;
        if(a < -UARTCMD_PWM_LIMIT) a = -UARTCMD_PWM_LIMIT;

        if(get_int(&p, &b))
        {
            if(b < 0)              b = 0;
            if(b > UARTCMD_MAX_MS) b = UARTCMD_MAX_MS;
        }
        else
        {
            b = UARTCMD_DEFAULT_MS;
        }

        if(!enc_homed)
        {
            /* No HOME yet, so the encoder frame is unknown and the
               absolute limits below would be guesswork. Cap the command
               to a nudge instead: enough to jog the cart while someone
               is watching it, not enough to build up a run. */
            if(a >  UARTCMD_NUDGE_PWM) a =  UARTCMD_NUDGE_PWM;
            if(a < -UARTCMD_NUDGE_PWM) a = -UARTCMD_NUDGE_PWM;
            if(b >  UARTCMD_NUDGE_MS)  b =  UARTCMD_NUDGE_MS;
        }
        else
        {
            /* Directional end-stop test, now that the frame is real.
               Measured on this rig: POSITIVE pwm makes the encoder count
               DECREASE, so a positive command drives toward ENC_MIN and
               a negative one toward ENC_MAX. Refuse only the direction
               that digs deeper into the limit, so a cart sitting on a
               limit can always be driven back off it. */
            if( (a > 0 && Encoder <= UARTCMD_ENC_MIN) ||
                (a < 0 && Encoder >= UARTCMD_ENC_MAX) )
            {
                UartCmd_Abort();
                reply_str("ERR=ENDSTOP");
                return;
            }
        }

        auto_run     = 0;
        Flag_Stop    = 0;               /* release the software stop   */
        manual_pwm   = (int)a;
        manual_ticks = (u32)(b / 5);    /* TIM1 ticks are 5 ms         */
        if(manual_ticks == 0 && b > 0) manual_ticks = 1;
        manual_mode  = 1;

        reply_int("PWM=", a);
        return;
    }

    /* --- KICK,<pwm>[,<ms>] : the repeatable disturbance (R4-6) -----
       Lab 5 ranks controllers by how each one recovers from THE SAME
       disturbance, so the disturbance must be identical every time. A
       hand push is not: the spread between two pushes is wider than the
       spread between two controllers, so a hand-pushed lab measures the
       experimenter, not the gains.

       This cannot be built out of M. M sets manual_mode, and manual mode
       REPLACES the balance loop (see control.c); an M sent while the rod
       is balancing drops it. KICK instead adds a timed offset to what the
       controller already decided:

           Moto = Balance_Pwm - Position_Pwm + kick_pwm;

       so the loop stays closed for the whole event and the angle trace
       afterwards is a recovery, not a fall. */
    if(eat(&p, "KICK"))
    {
        if(!get_int(&p, &a)) { reply_str("ERR=SYNTAX"); return; }

        if(a >  UARTCMD_KICK_PWM_MAX) a =  UARTCMD_KICK_PWM_MAX;
        if(a < -UARTCMD_KICK_PWM_MAX) a = -UARTCMD_KICK_PWM_MAX;

        if(get_int(&p, &b))
        {
            if(b < 0)                   b = 0;
            if(b > UARTCMD_KICK_MAX_MS) b = UARTCMD_KICK_MAX_MS;
        }
        else
        {
            b = UARTCMD_KICK_DEFAULT_MS;
        }

        /* There has to be a controller running for this to disturb. With
           the motor stopped, in manual mode, or mid swing-up, a kick is
           either ignored outright or is an open-loop shove at an
           unsupported rod. Say so, rather than replying KICK= and letting
           the host record ten seconds of nothing. */
        if(Flag_Stop || manual_mode || auto_run)
        {
            reply_str("ERR=NOTBALANCING");
            return;
        }

        /* Same directional end-stop rule as M, for the same reason:
           refuse only a kick that would drive further into a limit the
           cart is already sitting on. Skipped until HOME, because before
           that the encoder frame is unknown and the test would be
           confident nonsense. */
        if( enc_homed &&
            ( (a > 0 && Encoder <= UARTCMD_ENC_MIN) ||
              (a < 0 && Encoder >= UARTCMD_ENC_MAX) ) )
        {
            reply_str("ERR=ENDSTOP");
            return;
        }

        kick_ticks = (u32)(b / 5);           /* TIM1 ticks are 5 ms */
        if(kick_ticks == 0 && b > 0) kick_ticks = 1;
        kick_pwm   = (int)a;

        reply_int("KICK=", a);
        return;
    }

    if(eat(&p, "BAL"))
    {
        if(!get_int(&p, &a)) { reply_str("ERR=SYNTAX"); return; }

        if(a)
        {
            UartCmd_Abort();
            auto_run  = 0;
            Swing_up  = 0;              /* forces Position_Zero=Encoder */
            Flag_Stop = 0;
            reply_str("BAL=1");
        }
        else
        {
            UartCmd_Abort();
            Flag_Stop = 1;
            Set_Pwm(0);
            reply_str("BAL=0");
        }
        return;
    }

    if(eat(&p, "GAIN"))
    {
        float *target;
        const char *echo;
        u8 is_setpoint = 0;                                    /* R4-6 */

        while(*p == ' ' || *p == ',') p++;

        if     (eat(&p, "BKP")) { target = &Balance_KP;  echo = "BKP"; }
        else if(eat(&p, "BKD")) { target = &Balance_KD;  echo = "BKD"; }
        else if(eat(&p, "BKI")) { target = &Balance_KI;  echo = "BKI"; }
        else if(eat(&p, "PKP")) { target = &Position_KP; echo = "PKP"; }
        else if(eat(&p, "PKD")) { target = &Position_KD; echo = "PKD"; }
        else if(eat(&p, "PKI")) { target = &Position_KI; echo = "PKI"; }
        else if(eat(&p, "AZ"))  { target = &Angle_Zero;  echo = "AZ";
                                  is_setpoint = 1; }
        else                    { reply_str("ERR=GAINNAME"); return; }

        if(!get_int(&p, &a)) { reply_str("ERR=SYNTAX"); return; }

        if(is_setpoint)
        {
            /* AZ is an ADC reading, not a gain, so the +/-20000 gain
               clamp would be meaningless here. Refuse an out-of-range
               value outright instead of silently clamping it: Turn_Off()
               only lets the motor run within +/-500 counts of this
               number, so a typo would look like dead hardware. */
            if(a < UARTCMD_AZ_MIN || a > UARTCMD_AZ_MAX)
            {
                reply_str("ERR=RANGE");
                return;
            }
            /* Whatever the angle integrator has accumulated was measured
               against the OLD target. Carrying it across would dump a
               step of integral action into the motor at the exact moment
               the setpoint moves. */
            Balance_Integral = 0;
        }
        else
        {
            if(a < -20000) a = -20000;
            if(a >  20000) a =  20000;
        }

        *target = (float)a;

        cmd_puts("GAIN=");
        cmd_puts(echo);
        cmd_putc(',');
        cmd_putint(a);
        cmd_end();
        return;
    }

    if(eat(&p, "STREAM"))
    {
        if(!get_int(&p, &a)) { reply_str("ERR=SYNTAX"); return; }
        stream_enable = a ? 1 : 0;
        reply_int("STREAM=", (long)stream_enable);
        return;
    }

    reply_str("ERR=UNKNOWN");
}

/* ============================================================
   Public entry points
   ============================================================ */
void UartCmd_Init(void)
{
    USART1->CR1 |= (1 << 5);           /* RXNEIE */

    /* Preemption priority 0: this MUST be able to interrupt the TIM1
       control ISR, which is at preemption 1.

       Why it matters: TIM1_UP_IRQHandler calls Get_Adc_Average(3,10), and
       that function does delay_us(200) after every one of its 10 samples,
       so the control ISR blocks for over 2 ms. A byte at 128000 baud takes
       78 us, so ~26 bytes would be lost to receiver overrun on every
       control tick. Anything longer than two or three characters arrived
       corrupted -- "STATUS" came back as ERR=UNKNOWN, "GAIN,BKP,400" as
       ERR=SYNTAX, and a lone "V" vanished entirely when its one letter was
       the byte that got dropped.

       With preemption 0 this handler runs immediately even mid-ADC. It is
       safe to do so: it only reads USART1->DR into a buffer, never touches
       the motor and never waits on a flag. Replies are sent from the main
       loop, not from here. */
    MY_NVIC_Init(0, 0, USART1_IRQn, 2);
}

void UartCmd_Poll(void)
{
    if(cmd_ready)
    {
        char local[UARTCMD_RX_BUF];
        u8   i;

        for(i = 0; i < UARTCMD_RX_BUF; i++)
        {
            local[i] = cmd_line[i];
            if(local[i] == '\0') break;
        }
        local[UARTCMD_RX_BUF - 1] = '\0';

        cmd_ready = 0;                 /* release the ISR buffer first */
        ProcessCommand(local);
    }
}
