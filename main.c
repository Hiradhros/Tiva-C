#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "tm4c_cmsis.h"
#include "bsp.h"
#include <intrinsics.h>


//   Global Variables
uint32_t   SW1, SW2;  	// input from PF0 and PF4
int say = 0;
int clock = 0;

//Function Prototypes
char readChar(void);
void printChar(char c);
void printString(char * string);
void PortF_Init(void);
void Delay(void);
void _pushbutton(void);
void _ertele(void);

//define easy to read names for registers
#define SYSCTL_RCGCUART (*((volatile unsigned long *) 0x400FE618 ))     //RCGCUART Base 0x400FE000
#define SYSCTL_RCGCGPIO (*((volatile unsigned long *) 0x400FE608 ))     //RCGCCGPIO  0x400FE000  0x608


// SysTick Handler fn.
void SysTick_Handler(void) {
  if (say < 100011) { 
   GPIOF->DATA ^= 0x08; // LED is green
  }
  else {
    GPIOF->DATA ^= 0x02;  // LED is red
  }
}

// Assignment # 1
void _pushbutton(void) {
  if (SW1 == 0x00){   // zero means SW1 is pressed
    say++;
    SysTick->LOAD = SYS_CLOCK_HZ / 2U - 1U;
    SysTick->VAL = 0;
    SysTick->CTRL = (1U << 2) | (1U <<1) | 1U;
    __enable_interrupt();
 }
}

  volatile unsigned int a ;  
  volatile int b ;
int main(void) 
{
  //a =  *(unsigned int*)GPIO_PORTF_DATA_R;
  // 1. Enable the UART module using the RCGCUART register (see page 344).
  SYSCTL_RCGCUART |= (1<<0); 
  
  // 2. Enable the clock to the appropriate GPIO module via the RCGCGPIO register (see page 340).
  // To find out which GPIO port to enable, refer to Table 23-5 on page 1351.
  SYSCTL_RCGCGPIO |= (1<<0); 
  
  // 3. Set the GPIO AFSEL bits for the appropriate pins (see page 671). To determine which GPIOs to
  // configure, see Table 23-4 on page 1344
  GPIOA->AFSEL = (1<<1)|(1<<0); 
  
  // 4. Configure the GPIO current level and/or slew rate as specified for the mode selected (see
  // page 673 and page 681
  
  // 5. Configure the PMCn fields in the GPIOPCTL register to assign the UART signals to the appropriate
  // pins (see page 688 and Table 23-5 on page 1351).
  GPIOA->PCTL = (1<<0)|(1<<4);  
  
  GPIOA->DEN = (1<<0)|(1<<1); 
  
  // Find  the Baud-Rate Divisor
  // BRD = 16,000,000 / (16 * 9600) = 104.16666666666666666666666666666666666666666666666666
  // UARTFBRD[DIVFRAC] = integer(0.166667 * 64 + 0.5) = 11
  
  
  // With the BRD values in hand, the UART configuration is written to the module in the following order
  
  // 1. Disable the UART by clearing the UARTEN bit in the UARTCTL register
  UART0->CTL &= ~(1<<0);
  
  // 2. Write the integer portion of the BRD to the UARTIBRD register
  UART0->IBRD = 104;      
  // 3. Write the fractional portion of the BRD to the UARTFBRD register.
  UART0->FBRD = 11; 
  
  // 4. Write the desired serial parameters to the UARTLCRH register (in this case, a value of 0x0000.0060)
  UART0->LCRH = (0x3<<5)|(1<<4);     // 8-bit, no parity, 1-stop bit
  
  // 5. Configure the UART clock source by writing to the UARTCC register
  //UART0_CC = 0x0;          
  
  // 6. Optionally, configure the ?DMA channel (see ?Micro Direct Memory Access (?DMA)? on page 585)
  // and enable the DMA option(s) in the UARTDMACTL register
  
  // 7. Enable the UART by setting the UARTEN bit in the UARTCTL register.
  UART0->CTL = (1<<0)|(1<<8)|(1<<9); 
  
  // Configure LED pins
  SYSCTL_RCGCGPIO |= (1<<5);
  GPIOF->LOCK = 0x4C4F434B;
  GPIOF->CR = 0xff;
  GPIOF->DIR = (1<<1)|(1<<2)|(1<<3)|(0<<4);  // make LED pins (PF1, PF2, and PF3) outputs & PF4 as input
  GPIOF->DEN = (1<<1)|(1<<2)|(1<<3)|(1<<4); // enable digital function on LED pins
  GPIOF->DATA &= ~((1<<1)|(1<<2)|(1<<3)); // turn off leds
  GPIOF->PUR = 0x10;  
  
  SW1 = GPIOF->DATA & 0x00000010;  //GPIO_PORTF_DATA_R & 0x10;  // read PF4 into SW1
  
  while(1)
  {
    printString("Bir komut girin ve daha sonra enter tusuna basin: \n\r");
    char c = readChar();
    printChar(c);
    printString("\n\r");
    
    switch(c)
    {
    case 'r':
      GPIOF->DATA = (1<<1);
      printString("Kirmizi LED'i yaktiniz... \n\r");
      printString("Komut Basarili! \n\r");
      break;
    case 'b':
      GPIOF->DATA = (1<<2);
      printString("Mavi LED'i yaktiniz... \n\r");
      printString("Komut Basarili! \n\r");
      break;
    case 'g':
      GPIOF->DATA = (1<<3);
      printString("Yesil LED'i yaktiniz... \n\r");
      printString("Komut Basarili! \n\r");
      break;
    case 'l':
      GPIOF->DATA &= ~((1<<1)|(1<<2)|(1<<3));
      _pushbutton();
      break;
    default:
      GPIOF->DATA &= ~((1<<1)|(1<<2)|(1<<3));
      break;
    }
}
}

char readChar(void)  
{
  char c;
  while((UART0->FR & (1<<4)) != 0);
  c = UART0->DR;                 
  return c;                    
}

void printChar(char c)  
{
  while((UART0->FR & (1<<5)) != 0);
  UART0->DR = c;          
}

void printString(char * string)
{
  while(*string)
  {
    printChar(*(string++));
  }
}