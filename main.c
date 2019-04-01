//**************************************************DEFINES, INCLUDES E DECLARAÇÃO DE VARIAVEIS E FUNCOES ***************************************************************//
#define F_CPU 8000000UL
#include <avr/io.h> 
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <string.h> 
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define Trigger_pin PD5 /* Trigger pin */ 
#include "lcd.h"
void inic(void);
void Sonar(void);
int lerad(void);
void Sensor(void);
void FogeParedes(void);
/*USART Function Declarations*/
void usart_init();
void usart_data_transmit(unsigned char data);
void usart_string_transmit(char * string);
/*HC-05 Bluetooth Function Declarations*/
void hc_05_bluetooth_transmit_byte(char data_byte);
void hc_05_bluetooth_transmit_string(char * transmit_string);

long count; // contador da distancia do sonar
int i, j; //variaveis para uso do for quando o carro vira aleatoriamente no fogeparedes
int TimerOverflow = 0; // este timerofv vai incrementar sempre que houver um overflow na int1
double distance; // dá a distancia em cm do sonar
char string[10]; //para armazenar caracteres e escrever no LCD
int sensor1, sensor2, sensor3; //lê valores lógicos das entradas no micro dos sensores
int estado = 0; //variavel aux para o led
int apagarlcd1 = 0, apagarlcd2 = 0, apagarlcd3 = 0, apagarlcd4=0; //variaveis aux para usar no LCD, para quando se muda de modo ele só limpar uma vez o LCD
int switchmodo1, switchmodo2, soma; // usado para distinguir o estado do interruptor de 3 estados
int valorad; // valor analogico da bateria entre 0 e 255 sendo que 0 e 0V e 255 é 3V
float bateriacarro; // da a bateria do carro em volts(valorad*2)
int contadorled1 = 0;// quando chega ao valor desejado na main, entra na zona do codigo onde liga e desliga o led. fizemos desta maneira para evitar
// o uso excessivo de interrupcões.
int contadorled2 = 0;
int contadorled3 = 0;
int numerorandom; //gera numero random para a funcao fogeparedes
int estadoanterior; // criamos para quando o carro perde a sua localização, faz exatamente o que fez antes de se perder


void inic() {
  srand(time(NULL));
  ADMUX |= (1 << ADLAR); //ARef, ajustado ESQUERDA, ch 0 aref , internal vref off
  ADCSRA |= (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // ADEN=1: liga o conversor||fator divisao 8
  DDRB |= (1 << PB4); //pwm 0c0b
  DDRB |= (1 << PB2); //led 1hz
  DDRD |= (1 << PD7); // pwm oc2a
  DDRD |= (1 << PD5); //TRIGGER

  //************************INICIALIZACOES DO SONAR,MOTORES E SENSORES***************************************************************************************************//
  PORTD = 0xFF; /* Turn on Pull-up */
  DDRC = 0b01111000; //motores de pc3 a pc6, sensores de pc0 a pc2  PC0 -> dir    PC1 -> MEIO        PC2 -> esq
  LCDInit(LS_BLINK | LS_ULINE);
  TIMSK1 = (1 << TOIE1); /* Enable Timer1 overflow interrupts */
  TCCR1A = 0; /* Set all bit to zero Normal operation */
  //***************************ocR0B(RODA ESQUERDA)************************************
  TCCR0A |= (1 << COM0B1) | (1 << COM0A1) | (1 << WGM00); // clear oc0b & OC0A when compare(down),set on up counting, phase correct , maximo
  TCCR0B |= (1 << CS02); //psc 256
  OCR0B = 0;
  //*****************************ocR2A(RODA DIREITA)*************************************
  TCCR2A |= (1 << COM2A1) | (1 << WGM20); // clear oc2a when compare(down),set on up counting, phase correct , maximo
  TCCR2B |= (1 << CS22); //psc 256
  OCR2A = 0;
}

//********************FUNCOES DE BLUETOOTH**************************************************************

/*USART Function Definitions*/
void usart_init() {
	//USART Baud Rate			:9600
	//USART Data Bits			:8
	//USART Stop Bits			:1
	//USART Mode				:synchronous Mode
	//USART Parity				:No Parity
  UBRR0H = 0;
  UBRR0L = 51; //ubrr=(FOSC/16BAUD) -1 = 50
  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
  UCSR0C |= (0 << UMSEL00) | (1 << UCSZ01) | (1 << UCSZ00);   
}

void usart_data_transmit(unsigned char data) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
  _delay_ms(1);
}

void usart_string_transmit(char * string) {
  while ( * string) {
    usart_data_transmit( * string++);
  }
}

/*HC-05 Bluetooth Function Definitions*/
void hc_05_bluetooth_transmit_byte(char data_byte) {
  usart_data_transmit(data_byte);
}

void hc_05_bluetooth_transmit_string(char * transmit_string) {
  usart_string_transmit(transmit_string);

}

//********************FIM FUNCOES BLUETOOTH*********************************************************

void ControlaBluetooth() {

  _delay_ms(500);
  _delay_ms(500);
  /*Delay of 1s*/

  usart_init();
  /*USART initialization*/

  hc_05_bluetooth_transmit_string("Modulo Bluetooth");
  /*Transmits a string to Bluetooth Module*/

  hc_05_bluetooth_transmit_byte(0x0d);
  /*Transmits Carriage return to Bluetooth Module*/

  hc_05_bluetooth_transmit_byte(0x0a);
  /*Transmits New Line to Bluetooth Module for new line*/

  hc_05_bluetooth_transmit_string("Controlar AGV");
  /*Transmits a string to Bluetooth Module*/

  hc_05_bluetooth_transmit_byte(0x0d);
  /*Transmits Carriage return to Bluetooth Module*/

  hc_05_bluetooth_transmit_byte(0x0a);
  /*Transmits New Line to Bluetooth Module for new line*/
}

ISR(TIMER1_OVF_vect) {
  TimerOverflow++; /* Increment Timer Overflow count */
}

void Sonar() {

  /* Give 10us trigger pulse on trig. pin to HC-SR04 */
  PORTD &= (~(1 << Trigger_pin));
  _delay_ms(10);
  PORTD |= (1 << Trigger_pin);
  _delay_us(10);
  PORTD &= (~(1 << Trigger_pin));

  TCNT1 = 0; /* Clear Timer counter */
  TCCR1B = 0x41; /* Capture on rising edge, No prescaler*/
  TIFR1 = 1 << ICF1; /* Clear ICP flag (Input Capture flag) */
  TIFR1 = 1 << TOV1; /* Clear Timer Overflow flag */

  /*Calculate width of Echo by Input Capture (ICP) */

  while ((TIFR1 & (1 << ICF1)) == 0); /* Wait for rising edge */
  TCNT1 = 0; /* Clear Timer counter */
  TCCR1B = 0x01; /* Capture on falling edge, No prescaler */
  TIFR1 = 1 << ICF1; /* Clear ICP flag (Input Capture flag) */
  TIFR1 = 1 << TOV1; /* Clear Timer Overflow flag */
  TimerOverflow = 0; /* Clear Timer overflow count */

  while ((TIFR1 & (1 << ICF1)) == 0); /* Wait for falling edge */
  count = ICR1 + (65535 * TimerOverflow); /* Take count */
  /* 8MHz Timer freq, sound speed =343 m/s */
  distance = (double) count / 446.47;
  dtostrf(distance, 2, 1, string); /* distance to string */
  strcat(string, "cm "); /* Concat unit i.e.cm */
  LCDWriteStringXY(0, 0, string);

 

}

void Sensor() {

  sensor1 = PINC & (1 << PC0); //direita
  sensor2 = PINC & (1 << PC1); //meio
  sensor3 = PINC & (1 << PC2); //esquerda

  //***********************ESTADO DOS SENSORES ILUSTRADOS NO LCD****************** SENSOR DIREITA- S1 SENSOR MEIO -S2 SENSOR ESQUERDA-S3********************

  if (sensor2 == 2)
    LCDWriteStringXY(3, 0, "P");
  if (sensor2 == 0)
    LCDWriteStringXY(3, 0, "B");
  if (sensor3 == 4)
    LCDWriteStringXY(1, 0, "P");
  if (sensor3 == 0)
    LCDWriteStringXY(1, 0, "B");
  if (sensor1 == 1)
    LCDWriteStringXY(5, 0, "P");
  if (sensor1 == 0)
    LCDWriteStringXY(5, 0, "B");

  //***************************Segue em linha reta*************************************** 		
 

    if (sensor2 == 2 && sensor1 == 1 && sensor3 == 4) //reta(todos detetam preto)

    {
        PORTC = (1 << PC4) | (1 << PC6); // andar para a frente
        OCR0B = 255;
        OCR2A = 255;
       
      }

    

    //***************************Segue em linha reta mais devagar*************************************** 		
    if (sensor2 == 2 && sensor1 != 1 && sensor3 != 4) //reta(preto no sensor do meio)

    {

      PORTC = (1 << PC4) | (1 << PC6); // andar para a frente
      if (bateriacarro < 5.5) {
        OCR0B = 180;
        OCR2A = 180;
      }
      if (bateriacarro >= 5.5) {
        OCR0B = 170;
        OCR2A = 170;
      }

    }

    // *************VIRA direita*******************************************
    // 	 	 
    if (sensor2 == 2 && sensor1 == 1 && sensor3 != 4) //vira para a direia numa curva apertada
    {
      PORTC = (1 << PC3) | (1 << PC6); //roda esquerda on , roda direita p trás
      estadoanterior = 2;
      if (bateriacarro < 5.5) {
        OCR0B = 180;
        OCR2A = 180;
      }
      if (bateriacarro >= 5.5) {
        OCR0B = 170;
        OCR2A = 170;
      }
    }

    //*******************vira esquerda******************************************
    if (sensor2 == 2 && sensor1 != 1 && sensor3 == 4) //vira para a esquerda numa curva apertada
    {
      PORTC = (1 << PC4) | (1 << PC5); // aroda direita on, roda esquerda gira ao contrário
      estadoanterior = 3;
      if (bateriacarro < 5.5) {
        OCR0B = 180;
        OCR2A = 180;
      }
      if (bateriacarro >= 5.5) {
        OCR0B = 170;
        OCR2A = 170;
      }

    }

    if (sensor2 != 2 && sensor1 != 1 && sensor3 != 4) //para
    {

      //***************

      if (estadoanterior == 2 || estadoanterior == 4) { //direita e frente

        PORTC = (0 << PC4) | (1 << PC6); //roda esquerda on
        if (bateriacarro < 5.5)

          OCR0B = 180;

        if (bateriacarro >= 5.5)

          OCR0B = 170;

      }
      //****************
      if (estadoanterior == 3 || estadoanterior == 5) {
        PORTC = (1 << PC4) | (0 << PC6); // aroda direita on

        if (bateriacarro < 5.5)

          OCR2A = 180;

        if (bateriacarro >= 5.5)

          OCR2A = 170;
      }

    }

    // *************VIRA direita*******************************************
    // 	 	 
    if (sensor2 != 2 && sensor1 == 1 && sensor3 != 4) //vira para a direia numa curva leve
    {
      PORTC = (0 << PC4) | (1 << PC6); //roda esquerda on
      if (bateriacarro < 5.5)
        estadoanterior = 4;
      OCR0B = 180;

      if (bateriacarro >= 5.5)

        OCR0B = 170;

    }

    //*******************vira esquerda******************************************
    if (sensor2 != 2 && sensor1 != 1 && sensor3 == 4) //vira para a esquerda numa curva leve
    {
      PORTC = (1 << PC4) | (0 << PC6); // aroda direita on
      estadoanterior = 5;
      if (bateriacarro < 5.5)

        OCR2A = 180;

      if (bateriacarro >= 5.5)

        OCR2A = 170;

    

  }

}

void FogeParedes() {
  LCDWriteStringXY(8, 0, "BAT=");
  if (bateriacarro < 1) {
    bateriacarro = 0;
  }
  dtostrf(bateriacarro, 2, 1, string);
  LCDWriteStringXY(12, 0, string);
  LCDWriteStringXY(15, 0, "V");
  if (distance > 60) {

    PORTC = (1 << PC4) | (1 << PC6); // andar para a frente
    OCR0B = 255;
    OCR2A = 255;

  }

  if (distance <= 60 && distance > 30) {

    PORTC = (1 << PC4) | (1 << PC6); // andar para a frente
    OCR0B = 100;
    OCR2A = 100;

  }
  if (distance <= 30) {

    numerorandom = rand() % 2;

    if (numerorandom == 1) {

      for (i = 0; i < 5900; i++) {
        for (j = 0; j < 20; j++) {

          PORTC = (1 << PC4) | (1 << PC5); //roda esquerda tras , roda direita p frente
          OCR0B = 100; //esquerda
          OCR2A = 100;
        }
      }

    }

    if (numerorandom == 0) {

      for (i = 0; i < 5900; i++) {
        for (j = 0; j < 20; j++) {
          PORTC = (1 << PC3) | (1 << PC6); //roda esquerda frente , roda direita p trás
          OCR0B = 100; //esquerda
          OCR2A = 100;
        }
      }
    }

  }

}
int lerad(void) {
  ADCSRA |= (1 << 6); //CONVERSAO ON
  while ((ADCSRA & (1 << ADSC)) != 0); //espera que a conversao seja feita

  valorad = ADCH;

  return valorad;
}

int main() {
  _delay_ms(1000);
  inic();
  ControlaBluetooth();

  while (1) {
    switchmodo1 = PINB & (1 << PB0);
    switchmodo2 = PINB & (1 << PB1);
    soma = switchmodo1 + switchmodo2;
    //********************Informação da bateria dos motores******************************************
    lerad();
    bateriacarro = valorad * 50 / 255;
    bateriacarro = (bateriacarro / 10) * 2; //  fez se esta multiplicacao e depois divisao. chegamos a conclusao 
	//que assim conseguiria-mos obter um valor + preciso

    LCDWriteStringXY(8, 0, "BAT=");
    if (bateriacarro < 1) {
      bateriacarro = 0;
    }
    dtostrf(bateriacarro, 2, 1, string);
    LCDWriteStringXY(12, 0, string);
    LCDWriteStringXY(15, 0, "V");
    //****************************************Controlar interruptores******************************************

    if (soma == 1) //linefollower
    {

      apagarlcd1++;
      apagarlcd2 = 0;
      apagarlcd3 = 0;
      if (apagarlcd1 == 1) {
        PORTC &= (0 << PC3) | (0 << PC4) | (0 << PC5) | (0 << PC6);
        LCDClear(); // limpa so 1x
        _delay_ms(1000);
        LCDWriteStringXY(5, 1, "SegLine");

      }
      Sensor();

      //***************************************************************LED PISCA 1hz****************************************
      contadorled1++;
	  contadorled2=0;
	  contadorled3=0;
      if (contadorled1 == 550) // valor dimensionado usando o simulador do atmega, para este codigo
      {

        estado++;
        if (estado == 1)

          PORTB |= (1 << PB2);
        if (estado == 2) {

          PORTB &= (0 << PB2);
          estado = 0;
        }
        contadorled1 = 0;
      }

    }

    if (soma == 2) // robot
    {

      apagarlcd2++;
      apagarlcd1 = 0;
      apagarlcd3 = 0;
      if (apagarlcd2 == 1) {
        PORTC &= (0 << PC3) | (0 << PC4) | (0 << PC5) | (0 << PC6);
        _delay_ms(1000);
        LCDClear(); // so limpa 1x
        LCDWriteStringXY(2, 1, "Smart Robot");
        LCDWriteStringXY(8, 0, "BAT=");
        if (bateriacarro < 1) {
          bateriacarro = 0;
        }
        dtostrf(bateriacarro, 2, 1, string);
        LCDWriteStringXY(12, 0, string);
        LCDWriteStringXY(15, 0, "V");

      }
 //***************************************************************LED PISCA 1hz****************************************
      contadorled2++;
	  contadorled1=0;
	  contadorled3=0;
      if (contadorled2 == 35) // valor dimensionado usando o siumulador do atmega, para este codigo
      {

        estado++;
        if (estado == 1)

          PORTB |= (1 << PB2);
        if (estado == 2) {

          PORTB &= (0 << PB2);
          estado = 0;
        }
        contadorled2 = 0;
      }
      Sonar();
      FogeParedes();
     
    }

    //*****************
    if (soma == 3) // bt 
    {

      //***************************************************************LED PISCA 1hz****************************************
      contadorled3++;
	  contadorled1=0;
	  contadorled2=0;
      if (contadorled3 == 800) // valor dimensionado usando o simulador do atmega, para este codigo
      {

        estado++;
        if (estado == 1)

          PORTB |= (1 << PB2);
        if (estado == 2) {

          PORTB &= (0 << PB2);
          estado = 0;
        }
        contadorled3 = 0;
      }

      apagarlcd3++;
      apagarlcd2 = 0;
      apagarlcd1 = 0;
      if (apagarlcd3 == 1) {
        PORTC &= (0 << PC3) | (0 << PC4) | (0 << PC5) | (0 << PC6);
        LCDClear(); // so limpa uma vez
        _delay_ms(1000);
        LCDWriteStringXY(3, 1, "BT Control");
      }
      //****************CONTROLAR MODOS VIA BT***************************************************************************************
      while (UDR0 == '1') {

        apagarlcd1++;
        apagarlcd2 = 0;
		apagarlcd4=0;
        if (apagarlcd1 == 1) {
          PORTC &= (0 << PC3) | (0 << PC4) | (0 << PC5) | (0 << PC6);
          LCDClear(); // limpa so 1x
         
          LCDWriteStringXY(5, 1, "SegLine");
          LCDWriteStringXY(8, 0, "BAT=");
          if (bateriacarro < 1) {
            bateriacarro = 0;
          }
          dtostrf(bateriacarro, 2, 1, string);
          LCDWriteStringXY(12, 0, string);
          LCDWriteStringXY(15, 0, "V");
        }
        Sensor();

        //***************************************************************LED PISCA 1hz****************************************
         contadorled1++;
		  contadorled2=0;
		  contadorled3=0;
         
        if (contadorled1 == 2000) // valor dimensionado usando o simulador do atmega, para este codigo
        {

          estado++;
          if (estado == 1)

            PORTB |= (1 << PB2);
          if (estado == 2) {

            PORTB &= (0 << PB2);
            estado = 0;
          }
          contadorled1 = 0;
        }

      }

      while (UDR0 == '2') {
        apagarlcd2++;
        apagarlcd1 = 0;
		apagarlcd4=0;

        if (apagarlcd2 == 1) {
          PORTC &= (0 << PC3) | (0 << PC4) | (0 << PC5) | (0 << PC6);
          _delay_ms(1000);
          LCDClear(); // so limpa 1x
          LCDWriteStringXY(2, 1, "Smart Robot");
          LCDWriteStringXY(8, 0, "BAT=");
          if (bateriacarro < 1) {
            bateriacarro = 0;
          }
          dtostrf(bateriacarro, 2, 1, string);
          LCDWriteStringXY(12, 0, string);
          LCDWriteStringXY(15, 0, "V");

        }

        Sonar();
        FogeParedes();
        //***************************************************************LED PISCA 1hz****************************************
         contadorled2++;
		 contadorled1=0;
		 contadorled3=0;
     ;
        if (contadorled2 == 35) // valor dimensionado usando o siumulador do atmega, para este codigo
        {

          estado++;
          if (estado == 1)

            PORTB |= (1 << PB2);
          if (estado == 2) {

            PORTB &= (0 << PB2);
            estado = 0;
          }
          contadorled2 = 0;
        }
      }

      //****************************************************************************************************************

      if (UDR0 == 'F')

      {

        PORTC = (1 << PC4) | (1 << PC6); // andar para a frente
        OCR0B = 255;
        OCR2A = 255;

      }

      if (UDR0 == 'R')

      {

        PORTC = (0 << PC4) | (1 << PC6); //roda esquerda on
        OCR0B = 255;

      }

      if (UDR0 == 'S')

      {
		  apagarlcd4++;
		  if (apagarlcd4==1)
		  
			LCDClear();
			
		  
		  
			   
		  
		  
		
        PORTC = (0 << PC4) | (0 << PC6); //stop
  
      }
      if (UDR0 == 'L')

      {

        PORTC = (1 << PC4) | (0 << PC6); //left
        OCR2A = 255;

      }

      if (UDR0 == 'B')

      {

        PORTC = (1 << PC3) | (1 << PC5); //back
        OCR2A = 255;
        OCR0B = 255;

      }

      if (UDR0 == 'A') {

        PORTC = (0 << PC3) | (0 << PC5); //stop

      }
    }

    //***********************************************************************************************************************

  }
  return 0;
}