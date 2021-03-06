/* INSTITUTO FEDERAL FLUMINENSE - CAMPOS CENTRO (2021.2): 25/05/2022
   ENGENHARIA DE CONTROLE E AUTOMA??O: MICROPROCESSADORES E MICROCONTROLADORES
   Prof.: ALEXANDRE LEITE
   Aluno: KAIQUE GUIMAR?ES CERQUEIRA
   P1: IMPLEMENTA??O DO PID COM ARITM?TICA DE PONTO FLUTUANTE
*/

#include <18F1220.h>
#fuses INTRC_IO,NOWDT,NOPROTECT,NOLVP,NOMCLR,NOBROWNOUT
#use delay(clock=8000000)

////////////////////////////////////////////////////////////////////////////////

/*========================== Definir vari?veis globais =======================*/
int8 memAddress=0;
signed int8 PID = 0;
float y = 0, ref = 0, u = 0;
float error[2];
float derivative = 0, proportional = 0, integrative = 0;
float Kp = 2;
float Ki = 0.25;
float Kd = 0.125;
float Ts = 0.03125; // Ou 31.25ms: usando bit-shifting, move-se 5 bits

/*============================= INTERRUPT DO PID =============================*/ 
#int_timer0 
void timer0_interrupt() {
   // SQUARE WAVE TEST
   output_bit(PIN_B0, 1);
   
   // Aquisition profiling: 124 us (both channels)
   set_adc_channel(0);
   delay_us(10);
   ref = read_adc();
   set_adc_channel(1);
   delay_us(10);
   y = read_adc();
   
   // Computing PID profiling: 240 us
   error[1] = error[0];
   error[0] = ref - y;
   // Ts ser? um subm?ltiplo de 2
   integrative += (error[1] * Ts) * Ki;
   derivative = ((error[0] - error[1]) / Ts) * Kd;
   proportional = error[0] * Kp;
   u = integrative + derivative + proportional;
   
   // Write EPROM profiling: 10 ms
   PID = (int8) u;
   write_eeprom(memAddress,PID);
   memAddress++;
   
   set_timer0(64535);            // 0.001s
   output_bit(PIN_B0, 0);
} // USANDO A DIRETIVA int_timer0, O COMPILADOR CUIDA DE LIMPAR A FLAG


void main() {
   
   /*=================== Configurar dois canais do ADC =======================*/
   setup_adc(  ADC_CLOCK_INTERNAL  );
   setup_adc_ports( sAN0 | sAN1 );
   set_adc_channel(0);
   delay_us( 10 );
   read_adc(ADC_START_ONLY);
   
   /*========================== Configurar timer =============================*/
   
   // INTERNAL CLOCK SOURCE(8MHz), PRESCALER 1:2 VALUE: 1uS PER INCREMENT(1MHz)
   setup_timer_0( T0_INTERNAL | T0_DIV_2 );
   // INITIALIZE TIMER0 CLOCK (WITH DEFINED VALUE)REDO CALC
   // 16-BIT TIMER: COUNT FROM 0 TO 65535
   // 8 MHz CLOCK, 1:8 PRESCALER, SET TIMER0 TO OVERFLOW IN 1ms
   set_timer0(64535);         // 65535 - (0.001 / ((4/8000000) * 2))
   
   /*======================== TIMER0 como Interrupt ==========================*/
   clear_interrupt( INT_TIMER0 );   // PULL DOWN TIMER0 INTERRUPT FLAG
   enable_interrupts( GLOBAL );     // ENABLES GLOBAL INTERRUPTS
   enable_interrupts( INT_TIMER0 ); // ENABLES TIMER0 OVERFLOW INTERRUPT
   
   while (1)
   {
      // While vazio
   }
}
