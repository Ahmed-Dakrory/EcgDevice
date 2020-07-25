
// This file contains macros and structs for convenience and readability.
#include <stdint.h>
#include "stm32l4xx.h"
#include <stdio.h>


#define GPIO_AF7_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_AF7_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */

// Macros to reduce typing later on
#define  REGISTER_32(ADDRESS) (*((volatile uint32_t *)(ADDRESS)))
#define ADC_BASE        0x50040000
#define ADC1_CCR        REGISTER_32(ADC_BASE +0x300 + 0x08)


#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
	




void initADC();
void Send_ADC_Value();
volatile int ADCValue=0;

#define MODE_CONTINUES 			 0
#define MODE_DISCRETE  			 1
#define MODE_MINUTE_DATA		 2

#define SEND_TRUE            1
#define SEND_FALSE           0

int sampleRate = 50;
int Mode = MODE_CONTINUES;
int SendNow = SEND_FALSE;
	

// On nucleo l432kc the Green LED is on port B pin 3
// If changing port remember to also activate the clock for that port.
// For port B that is the line: RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
#define GREEN_LED_PORT GPIOB
#define GREEN_LED_PIN 3





volatile int32_t SysTickCount = 0;
volatile static int32_t mainDummy = 0;
volatile static int32_t mainCh = '@';

// Simplest possible SysTick ISR (interrupt survice routine).
// TODO This "__attribute__ ((interrupt, used))" what does it do? 
// It seems to work also without it.
void __attribute__ ((interrupt, used)) SysTick_Handler(void)
//void SysTick_Handler(void)
{
  SysTickCount++;
}

// On a 32 bit CPU this should be atomic so we don't need to turn of interrupts.
int32_t sysTickGetCount()
{
  return SysTickCount;
}

// This can be used if a delay is needed before the SysTick is started.
// Once sys tick is running use that instead.
void busyWait(uint32_t delay)
{
  while (delay > 0)
  { 
    mainDummy++; // Change a volatile, just to be shure the compiler does not optimize this out.
    delay--;
  }
}

// Setup the port/pin that is connected to the LED (Using the Green LED, it works).
static void ledInit(GPIO_TypeDef* port, int pin)
{
  port->MODER &= ~(3U << (pin*2)); // Clear the 2 mode bits for this pin.
  port->MODER |= (1U << (pin*2)); // 1 for "General purpose output mode".
  port->OTYPER &= ~(0x1U << (pin*1)); // Clear output type, 0 for "Output push-pull".
  port->OSPEEDR &= ~(0x3U << (pin*2)); // Clear the 2 mode bits for this pin.
  port->OSPEEDR |= (0x1U << (pin*2)); // 1 for "Medium speed" or 2 for High speed.
  port->PUPDR &= ~(3U << (pin*2)); // Clear the 2 pull up/down bits. 0 for No pull-up, pull-down
}


// Setup the port/pin that is connected to the Input (Using the Sensor ).
static void InputSensorDigital(GPIO_TypeDef* port, int pin)
{
  port->MODER |= (0U << (pin*2)); // 1 for "General purpose input mode".
  port->PUPDR &= ~(3U << (pin*2)); // Clear the 2 pull up/down bits. 0 for No pull-up, pull-down
}

static void ledOn(GPIO_TypeDef* port, int pin)
{
  GPIOB->ODR |= (0x1U << GREEN_LED_PIN);
}

static void ledOff(GPIO_TypeDef* port, int pin)
{
  GPIOB->ODR &= ~(0x1U << GREEN_LED_PIN);
}


/* Configure the SysTick to have interrupt once every 1ms. */
/* Doing about the same as HAL_InitTick, */
static void sysTickInit()
{
  const uint32_t sysTickPriority = 0xfU; // 0-255, Lower number for sysTickPriority is higher priority.
  const uint32_t SystemCoreClock = 80000000U;
  const uint32_t SubPriority = 0;

  /* set reload register */
  SysTick->LOAD  = (uint32_t)(SystemCoreClock/25000 - 1UL); // May need some callibration.
  SysTick->VAL = 0UL;

  /* Enable SysTick IRQ and SysTick Timer */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk; 

  // Set SysTick priority. Do not know what NVIC_GetPriorityGrouping does. HAL did it that way.
  //uint32_t prioritygroup = NVIC_GetPriorityGrouping();
  //NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(prioritygroup, sysTickPriority, SubPriority));
  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */


  // It can be good with a tiny delay here for clocks to start up.
  busyWait(1);
}

// Ref [1] chapter 5.3.4 Sleep mode
void enterSleepMode()
{  
  /* Clear SLEEPDEEP bit of Cortex System Control Register */
  CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
  
  /* Request Wait For Interrupt */
  __WFI();
}


/**
 *  For simplicity it is assumed here that only sys tick is running
 * and waking the CPU from sleep. Sys tick is running as about 1 ms.
 * If other timers or high frequency IO is running we would need to
 * look at the sysTickGetCount() to get milli seconds passed. 
 */
void sleepWait(int timeMs)
{
  while(timeMs>0)
  {
    // typically there is an interrupty every ms.
    enterSleepMode();
    timeMs--;
  }
}


void errorHandler(int errorCode)
{
  for(;;)
  {
    for(int i=0; i<errorCode; i++)
    {     
      ledOn(GREEN_LED_PORT, GREEN_LED_PIN);
      sleepWait(100);

      ledOff(GREEN_LED_PORT, GREEN_LED_PIN);
      sleepWait(100);
    }
    sleepWait(800);
  }
}


struct fifo
{
  uint8_t head;
  uint8_t tail;
  char buffer[256];
};


static inline void fifoPut(volatile struct fifo *fifoPtr, char ch)
{ 
  fifoPtr->buffer[fifoPtr->head] = ch;
  fifoPtr->head++;
}

static inline int fifoIsFull(volatile struct fifo *fifoPtr)
{
  return (((uint8_t)(fifoPtr->head+1)) == fifoPtr->tail);
}

static inline int fifoIsEmpty(volatile struct fifo *fifoPtr)
{
  return (fifoPtr->head == fifoPtr->tail);
}

static inline char fifoTake(volatile struct fifo *fifoPtr)
{ 
  const char tmp = fifoPtr->buffer[fifoPtr->tail++];
  return tmp;
}

//volatile struct fifo lpuart1In = {0,0,{0}};
//volatile struct fifo lpuart1Out = {0,0,{0}};
volatile struct fifo usart1In = {0,0,{0}};
volatile struct fifo usart1Out = {0,0,{0}};
volatile struct fifo usart2In = {0,0,{0}};
volatile struct fifo usart2Out = {0,0,{0}};

enum
{
  DEV_LPUART1 = 0,
  DEV_USART1 = 1,
  DEV_USART2 = 2,
};

#if 0
void __attribute__ ((interrupt, used)) LPUART1_IRQHandler(void)
{
  volatile uint32_t tmp = LPUART1->ISR;

  // RXNE (Receive not empty)
  if (tmp & USART_ISR_RXNE_Msk)
  {
    // Simple receive, this never happens :(
    //mainDummy = USART1->RDR;
    fifoPut(&usart0In, USART1->RDR);
  }

  // TXE (transmit empty)
  if (tmp & USART_ISR_TXE_Msk)
  {
    if (!fifoIsEmpty(&usart1Out))
    {
      LPUART1->TDR = fifoTake(&usart1Out);
    }
    else
    {
      // We are done sending, disables further tx empty interrupts. 
      LPUART1->CR1 &= ~(USART_CR1_TXEIE_Msk);
    }
  }
}
#endif

void __attribute__ ((interrupt, used)) USART1_IRQHandler(void)
{
  volatile uint32_t tmp = USART1->ISR;

  // RXNE (Receive not empty)
  if (tmp & USART_ISR_RXNE_Msk)
  {
    // Simple receive, this never happens :(
    //mainDummy = USART1->RDR;
    fifoPut(&usart1In, USART1->RDR);
  }

  // TXE (transmit empty)
  if (tmp & USART_ISR_TXE_Msk)
  {
    if (!fifoIsEmpty(&usart1Out))
    {
      USART1->TDR = fifoTake(&usart1Out);
    }
    else
    {
      // We are done sending, disables further tx empty interrupts. 
      USART1->CR1 &= ~(USART_CR1_TXEIE_Msk);
    }
  }
}


void __attribute__ ((interrupt, used)) USART2_IRQHandler(void)
{
  volatile uint32_t tmp = USART2->ISR;

  // RXNE (Receive not empty)
  if (tmp & USART_ISR_RXNE_Msk)
  {
    // Simple receive, this never happens :(
    fifoPut(&usart2In, (int)(USART2->RDR & 0xFF));

    // For debugging count the RXNE interrupts.
    mainCh++; // Remove this when things work.
  }

  // TXE (transmit empty)
  if (tmp & USART_ISR_TXE_Msk)
  {
    if (!fifoIsEmpty(&usart2Out))
    {
      USART2->TDR = fifoTake(&usart2Out);
    }
    else
    {
      // We are done sending, disables further tx empty interrupts. 
      USART2->CR1 &= ~(USART_CR1_TXEIE_Msk);

      // This was just for testing, remove later...
      //USART2->CR1 |= (USART_CR1_RXNEIE_Msk);
    }
  }
}

/* This seems to work fine, we do get what the Nucleo sends */
static void setupIoPinTx(GPIO_TypeDef *base, uint32_t pin, uint32_t alternateFunction)
{
    uint32_t tmp = 0x00;

    /*
    Alternate function register.
    Set it to USART2. Remember to set the port mode to alternate mode also.
    */
    const uint32_t afrIndex = pin >> 3;
    const uint32_t afrOffset = (pin & 0x7) * 4;
    tmp = base->AFR[afrIndex];
    tmp &= ~(15U << afrOffset);
    tmp |= alternateFunction << afrOffset;
    base->AFR[afrIndex] = tmp;
  
    /* 
    Configure IO Direction mode (Input, Output, Alternate or Analog)
    00: Input mode 
    01: General purpose output mode
    10: Alternate function mode
    11: Analog mode (reset state)
    Need the alternate mode in this case. See also AFR.
    */
    tmp = base->MODER;
    tmp &= ~(3U << (pin * 2));
    tmp |= (2U << (pin * 2)); 
    base->MODER = tmp;

    /*
    Configure the IO Speed
    00: Low speed (reset state for most IO pins)
    01: Medium speed
    10: High speed
    11: Very high speed (reset state for PA13)
    */
    tmp = base->OSPEEDR;
    tmp &= ~(3U << (pin * 2));
    tmp |= (2U << (pin * 2)); 
    base->OSPEEDR = tmp;

    /* 
    Configure the IO Output Type
    0: Output push-pull (reset state)
    1: Output open-drain
    */
    //tmp = GPIOA->OTYPER;
    //tmp &= ~(1U << (pin * 1));
    //tmp |= (0U << (pin * 1)); 
    //GPIOA->OTYPER = tmp;

    /* 
    Activate the Pull-up or Pull down resistor for the current IO 
    00: No pull-up, pull-down (reset state for most IO pins)
    01: Pull-up
    10: Pull-down
    11: Reserved
    */
    //tmp = GPIOA->PUPDR;
    //tmp &= ~(3U << (pin * 2));
    //tmp |= (GPIO_NOPULL << (pin * 2)); 
    //GPIOA->PUPDR = tmp;
}

/**
 * Something wrong here with port configuration? 
 * Usart2 does not receive it can only send.
 * But with Usart1 both sending and receiving works.
 * It can be Uart or port configuration.
 * TODO
 * Probably both TX and RX pins can be configured the same way. 
 * If so only one method setupIoPinRxTx should be needed.
 * */
static void setupIoPinRx(GPIO_TypeDef *base, uint32_t pin, uint32_t alternateFunction)
{
    uint32_t tmp = 0x00;

    /*
    Alternate function register.
    Set it to USART2. Remember to set the port mode to alternate mode also.
    */
    const uint32_t afrIndex = pin >> 3;
    const uint32_t afrOffset = (pin & 0x7) * 4;
    tmp = base->AFR[afrIndex];
    tmp &= ~(15U << afrOffset);
    tmp |= alternateFunction << afrOffset;
    base->AFR[afrIndex] = tmp;
  
    /* 
    Configure IO Direction mode (Input, Output, Alternate or Analog)
    00: Input mode 
    01: General purpose output mode
    10: Alternate function mode
    11: Analog mode (reset state)
    Need the alternate mode in this case. See also AFR.
    I think it shall be alternate mode but perhaps it shall be input mode?
    */
    tmp = base->MODER;
    tmp &= ~(3U << (pin * 2));
    tmp |= (2U << (pin * 2)); 
    base->MODER = tmp;

    /*
    Configure the IO Speed
    00: Low speed (reset state for most IO pins)
    01: Medium speed
    10: High speed
    11: Very high speed (reset state for PA13)
    */
    tmp = base->OSPEEDR;
    tmp &= ~(3U << (pin * 2));
    tmp |= (2U << (pin * 2)); 
    base->OSPEEDR = tmp;

    /* 
    Configure the IO Output Type
    0: Output push-pull (reset state)
    1: Output open-drain
    Some examples use push-pull others open-drain, did not notice any difference.
    This pin is for input anyway so this should not matter.
    */
    //tmp = GPIOA->OTYPER;
    //tmp &= ~(1U << (pin * 1));
    //tmp |= (1U << (pin * 1)); 
    //GPIOA->OTYPER = tmp;

    /* 
    Activate the Pull-up or Pull down resistor for the current IO 
    00: No pull-up, pull-down (reset state for most IO pins)
    01: Pull-up
    10: Pull-down
    11: Reserved
    Seems resonable to activate Pull-up if open drain is used, but did not notice any improvement doing so.
    */
    tmp = GPIOA->PUPDR;
    tmp &= ~(3U << (pin * 2));
    tmp |= (1U << (pin * 2)); 
    GPIOA->PUPDR = tmp;
}



USART_TypeDef *usartGetPtr(int usartNr)
{
  switch(usartNr)
  {
    case DEV_USART1: return USART1;
    case DEV_USART2: return USART2;
    default : break;
  }
  return NULL;
}

int usartGetIrqN(int usartNr)
{
  switch(usartNr)
  {
    case DEV_LPUART1: return LPUART1_IRQn;
    case DEV_USART1: return USART1_IRQn;
    case DEV_USART2: return USART2_IRQn;
    default : break;
  }
  return 0;
}

volatile struct fifo *usartGetInFifo(int usartNr)
{
  switch(usartNr)
  {
    //case DEV_LPUART1: return &lpuart1In;
    case DEV_USART1: return &usart1In;
    case DEV_USART2: return &usart2In;
    default : break;
  }
  return 0;
}

volatile struct fifo *usartGetOutFifo(int usartNr)
{
  switch(usartNr)
  {
    //case DEV_LPUART1: return &lpuart1Out;
    case DEV_USART1: return &usart1Out;
    case DEV_USART2: return &usart2Out;
    default : break;
  }
  return 0;
}

#if 0
int lpuartInit(uint32_t baud)
{

  /* Disable the Peripheral, should be the reset state but just in case. */
  LPUART1->CR1 &= ~USART_CR1_UE_Msk;

  /*
  USART1_TX on PA9, USART1_RX on PA10. 
  But it is Usart2 that is connected to USB so using that one. 
  USART2 GPIO Configuration: USART2_TX on PA2, USART2_RX on PA3
  */

  // Clear the in and out FIFOs
  volatile struct fifo *inFifo=usartGetInFifo(DEV_LPUART1);
  volatile struct fifo *outFifo=usartGetOutFifo(DEV_LPUART1);
  memset((void*)inFifo, 0, sizeof(*inFifo));
  memset((void*)outFifo, 0, sizeof(*outFifo));


  // Enable Usart clock LPUART1EN
  RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN_Msk;

  // Select clock LPUART1SEL
  {
    uint32_t tmp = RCC->CCIPR;
    tmp &= ~(3 << 10);
    tmp |= ~(1 << 10);
    RCC->CCIPR = tmp;
  }

  // https://community.st.com/thread/48833-lpuart-ll-driver-not-working-hal-driver-working-stm32l476jg
  // PB11 LPUART1_TX
  // PB12 LPUART1_RTS
  // PB13 LPUART1_CTS
  // PB10 LPUART1_RX 
  // On the nucleo pins PB10 and PB11 are not available so could not test this. Or can other pins be used?

  setupIoPinTx(GPIOB, 11, GPIO_AF8_LPUART1);
  setupIoPinRx(GPIOB, 10, GPIO_AF8_LPUART1);



  /* 
  Select the desired baud rate using the USART_BRR register.
  https://community.st.com/thread/46664-setting-the-baudrate-for-usart-in-stm32f103rb
  Ref [1] chapter 36.5.4 USART baud rate generation 
  */
  uint32_t clockDiv = (256*32768) / baud;

  if ((clockDiv<0x300) || (clockDiv>0xfffff))
  {
    return -1;
  }

  //LPUART1->BRR = clockDiv;
  LPUART1->BRR = 0x300;


  /* Enable uart transmitter and receiver */
  LPUART1->CR1 |= USART_CR1_TE_Msk | USART_CR1_RE_Msk;


  // Bit 1 UESM: USART enable in Stop mode 
  // just testing, this should not be needed in this example.
  //USART2->CR1 |= USART_CR1_UESM_Msk;

  /*
  In asynchronous mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN, HDSEL and IREN  bits in the USART_CR3 register.
  */
  //USART2->CR2 &= ~(USART_CR2_LINEN_Msk | USART_CR2_CLKEN_Msk);
  //USART2->CR3 &= ~(USART_CR3_SCEN_Msk | USART_CR3_HDSEL_Msk | USART_CR3_IREN_Msk);
  LPUART1->CR2 &= ~(USART_CR2_CLKEN_Msk);



  //Set Usart1 interrupt priority. Lower number is higher priority.
  //uint32_t prioritygroup = NVIC_GetPriorityGrouping();
  //NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(prioritygroup, 45, 0));
  const int irqN = usartGetIrqN(DEV_LPUART1);
  NVIC_SetPriority (irqN, (1UL << __NVIC_PRIO_BITS) - 1UL);

  // This one helped.
  // https://community.st.com/thread/19735
  // I was missing to do NVIC_EnableIRQ.
  NVIC_EnableIRQ(irqN);

  /* Enable the Peripheral */
  LPUART1->CR1 |= USART_CR1_UE_Msk;

  /* 
  Shall interrupt be enabled before or after usart is enabled? 
  Did not notice any difference doing this before or after.
  RXNEIE
  0: Interrupt is inhibited
  1: A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_ISR 
  */
  LPUART1->CR1 |= USART_CR1_RXNEIE_Msk;

  return 0;
}
#endif

/**
This sets up the Usart 1 or 2. 
Sending works but receiving does not work on Usart2.
Returns 0 if OK.
*/
int usartInit(int usartNr, uint32_t baud)
{
  USART_TypeDef *usartPtr = usartGetPtr(usartNr);

  if (usartPtr == NULL)
  {
    return -1;
  }


  /* Disable the Peripheral, should be the reset state but just in case. */
  usartPtr->CR1 &= ~USART_CR1_UE_Msk;

  /*
  USART1_TX on PA9, USART1_RX on PA10. 
  But it is Usart2 that is connected to USB so using that one. 
  USART2 GPIO Configuration: USART2_TX on PA2, USART2_RX on PA3
  */

  // Clear the in and out FIFOs
  volatile struct fifo *inFifo=usartGetInFifo(1);
  volatile struct fifo *outFifo=usartGetOutFifo(1);
  //memset((void*)inFifo, 0, sizeof(*inFifo));
  //memset((void*)outFifo, 0, sizeof(*outFifo));


  // Enable Usart clock
  switch(usartNr)
  {
    case DEV_USART1:
      // If higher baud rate is needed change clock source.
      // Select clock LPUART1SEL
      /*{
        uint32_t tmp = RCC->CCIPR;
        tmp &= ~(3 << 0);
        tmp |= ~(1 << 0);
        RCC->CCIPR = tmp;
      }*/
      RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk;
      break;
    case DEV_USART2:
      // If higher baud rate is needed change clock source.
      // Select clock LPUART1SEL
      /*{
        uint32_t tmp = RCC->CCIPR;
        tmp &= ~(3 << 2);
        tmp |= ~(1 << 2);
        RCC->CCIPR = tmp;
      }*/
      RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN_Msk;  // bit 17
      break;
    default:
      return -1;
  }



  // Configure IO pins.
  switch(usartNr)
  {
    case DEV_USART1:
      setupIoPinTx(GPIOA, 9, GPIO_AF7_USART1);
      setupIoPinRx(GPIOA, 10, GPIO_AF7_USART1);
      // Alternatively it is possible to have USART1 on PB6, PB7.     
      //setupIoPinTx(GPIOB, 6, GPIO_AF7_USART1);
      //setupIoPinRx(GPIOB, 7, GPIO_AF7_USART1);
      break;
    case DEV_USART2:
      setupIoPinTx(GPIOA, 2, GPIO_AF7_USART2);
      setupIoPinRx(GPIOA, 3, GPIO_AF7_USART2);
      break;
    default:
      return -1;

    return 0;
  }


  /* 
  Ref [1] chapter 36.8.1 Control register 1 (USART_CR1)
  36.8.12 USART register map 
  8-bit character length: M[1:0] = 00, that is default.
  1 stop bit is the default value of number of stop bits. 
  That is what we want. So skipping those settings.
  */
  
  /* 
  OVER8
  0: Oversampling by 16 (default)
  1: Oversampling by 8
  */
  //usartPtr->CR1 |= (1 << 15);

  /* 
  Select the desired baud rate using the USART_BRR register.
  https://community.st.com/thread/46664-setting-the-baudrate-for-usart-in-stm32f103rb
  Ref [1] chapter 36.5.4 USART baud rate generation 
  */
  uint32_t divider = (4000000U) / baud;
  /*if (divider<0x300)
  {
    return -1;
  }*/
  usartPtr->BRR = divider;


  /* Enable uart transmitter and receiver */
  usartPtr->CR1 |= USART_CR1_TE_Msk | USART_CR1_RE_Msk;

  // Bit 1 UESM: USART enable in Stop mode 
  // just testing, this should not be needed in this example.
  //USART2->CR1 |= USART_CR1_UESM_Msk;

  /*
  In asynchronous mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN, HDSEL and IREN  bits in the USART_CR3 register.
  */
  //USART2->CR2 &= ~(USART_CR2_LINEN_Msk | USART_CR2_CLKEN_Msk);
  //USART2->CR3 &= ~(USART_CR3_SCEN_Msk | USART_CR3_HDSEL_Msk | USART_CR3_IREN_Msk);
  usartPtr->CR2 &= ~(USART_CR2_CLKEN_Msk);

  busyWait(1);


  //Set Usart1 interrupt priority. Lower number is higher priority.
  //uint32_t prioritygroup = NVIC_GetPriorityGrouping();
  //NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(prioritygroup, 45, 0));
  const int irqN = usartGetIrqN(usartNr);
  NVIC_SetPriority (irqN, (1UL << __NVIC_PRIO_BITS) - 1UL);

  // This one helped.
  // https://community.st.com/thread/19735
  // I was missing to do NVIC_EnableIRQ.
  NVIC_EnableIRQ(irqN);

  /* Enable the Peripheral */
  usartPtr->CR1 |= USART_CR1_UE_Msk;

  /* 
  Shall interrupt be enabled before or after usart is enabled? 
  Did not notice any difference doing this before or after.
  RXNEIE
  0: Interrupt is inhibited
  1: A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_ISR 
  */
  usartPtr->CR1 |= USART_CR1_RXNEIE_Msk;

  return 0;
}




// Ref [1] chapter 36.5.2 USART transmitter
void usartPutChar(int usartNr, int ch)
{
  volatile struct fifo *outFifo=usartGetOutFifo(usartNr);
  while (fifoIsFull(outFifo))
  {
    // wait.
  }

  fifoPut(outFifo, ch);

  /* 
  Now we need to trigger the ISR. Its done by enabling transmitter empty interrupt (it is empty so).
  TXEIE
  0: Interrupt is inhibited
  1: A USART interrupt is generated whenever TXE=1 in the USART_ISR register  
  */
  switch(usartNr)
  {
    /*case DEV_LPUART1:
      LPUART1->CR1 |= USART_CR1_TXEIE_Msk; 
      break;*/
    case DEV_USART1:
      USART1->CR1 |= USART_CR1_TXEIE_Msk; 
    case DEV_USART2:
      USART2->CR1 |= USART_CR1_TXEIE_Msk; 
    default:
      break;
  }
}

int usartGetChar(int usartNr)
{
  volatile struct fifo *inFifo=usartGetInFifo(usartNr);
  if (!fifoIsEmpty(inFifo))
  {
    return fifoTake(inFifo);
  }
  else
  {
    return -1;
  }
}


void usartPrint(int usartNr, char *str)
{
  while(*str)
  {
    usartPutChar(usartNr, *str++);
  }
}



static const char hexTable[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
static inline int encodeHex(int d)
{
  return hexTable[(d&0xF)];
}





#define True 1
#define False 0

int64_t isOpen = True;

void sendString(char str[]){
	
	// Print hello world. This works when testing.
  usartPrint(DEV_USART1, str);
  usartPutChar(DEV_USART1, '\n');
}

static char *itoa_simple_helper(char *dest, int i) {
  if (i <= -10) {
    dest = itoa_simple_helper(dest, i/10);
  }
  *dest++ = '0' - i%10;
  return dest;
}

char *conv2Str(char *dest, int i) {
  char *s = dest;
  if (i < 0) {
    *s++ = '-';
  } else {
    i = -i;
  }
  *itoa_simple_helper(s, i) = '\0';
  return dest;
}


unsigned int rand_interval(unsigned int now)
{
    if(now > 900){
			return 300;
		}else{
			return now + 10;
		}

   
}

int __main(void)
{
  /* 
  Set Interrupt Group Priority
  ref [1] does not say anything about SCB->AIRCR.
  It must be an ARM/Cortex thing and not an STM32 thing.
  http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0553a/Cihehdge.html
  */
  {
    uint32_t tmp =  SCB->AIRCR;
    const uint32_t VECTKEY = 0x5FAUL; // On writes, write 0x5FA to VECTKEY, otherwise the write is ignored.
    tmp &= ~((uint32_t)((0xFFFFUL << SCB_AIRCR_VECTKEY_Pos) | (7UL << SCB_AIRCR_PRIGROUP_Pos)));
    tmp |= ((uint32_t)VECTKEY << SCB_AIRCR_VECTKEY_Pos) |  (NVIC_PRIORITYGROUP_4 << SCB_AIRCR_PRIGROUP_Pos);
    SCB->AIRCR =  tmp;
  }



  /* 
  Use SysTick as time base source and configure 1ms tick (default clock after Reset is MSI)
  Configure the SysTick to have interrupt in 1ms time basis. 
  */
  sysTickInit();

  /*
  AHB2 peripheral clock enable register (RCC_AHB2ENR)
  If other ports than GPIOA and GPIOB are to be used those also need 
  their clocks to be enabled like this.
  */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN_Msk | RCC_AHB2ENR_GPIOAEN_Msk ;
	
	if (usartInit(DEV_USART1, 115200)!=0)
  {
    errorHandler(2);
  }
	
	
	// Setup the port/pin that is connected to the Green LED.
  ledInit(GREEN_LED_PORT, GREEN_LED_PIN);
	
 /*ADC Config*/
	//sendString("Load");
	
	initADC();

  
	


  int32_t mainTimerMs = sysTickGetCount() + 1000;
	int32_t countTimerMinute = sysTickGetCount() + 1000;
	
  int64_t mainDebugCounter = 0;

  // loop forever
  for(;;)
  {
		
		
			
		int data = ADC1->DR;
    
		const int32_t stc = sysTickGetCount();
    const int32_t diff = stc - mainTimerMs;
		const int32_t diffMinute = countTimerMinute - stc;
		 if ((diff)>=0)
    {
			
			

     
      mainTimerMs += sampleRate;
			
			
			//Mode Continus
			{
			if(Mode == MODE_CONTINUES){
				
				
				if(isOpen == True){
        ledOn(GREEN_LED_PORT, GREEN_LED_PIN);
				isOpen = False;
			}else{
				ledOff(GREEN_LED_PORT, GREEN_LED_PIN);
				
				isOpen = True;
			}
			
			
			
			
			Send_ADC_Value();
			}
		}
			
		//Mode Minute
			{
			if(Mode == MODE_MINUTE_DATA){
				if(diffMinute>0){
					
					
				if(isOpen == True){
        ledOn(GREEN_LED_PORT, GREEN_LED_PIN);
				isOpen = False;
			}else{
				ledOff(GREEN_LED_PORT, GREEN_LED_PIN);
				
				isOpen = True;
			}
			
				Send_ADC_Value();
				}
			}
		}
			
			
    }
   
	{
    /* MODE Send DISCRETE*/
    
			if(Mode == MODE_DISCRETE){
				
				if(SendNow == SEND_TRUE){
						Send_ADC_Value();
					ledOn(GREEN_LED_PORT, GREEN_LED_PIN);
					busyWait(10000);
					ledOff(GREEN_LED_PORT, GREEN_LED_PIN);
						SendNow = SEND_FALSE;
				}
				
			}
		
		/* End Of DISCRETE */
		}
		
		
    // This works, we can receive from usart1.
    {
      int ch = usartGetChar(DEV_USART1);
			
      if (ch>=0)
      {
				if (ch == 49){
					sampleRate = 50;
				}else if(ch == 50){
					sampleRate = 100;
				}else if(ch == 51){
					sampleRate = 200;
				}else if(ch == 52){
					sampleRate = 500;
				}else if(ch == 53){
					sampleRate = 750;
				}else if(ch == 54){
					sampleRate = 1000;
				}else if(ch == 48){//0
					Mode = MODE_CONTINUES;
				}else if(ch == 100){//d
					Mode = MODE_DISCRETE;
					ledOff(GREEN_LED_PORT, GREEN_LED_PIN);
				}else if(ch == 109){//m
					Mode = MODE_MINUTE_DATA;
					countTimerMinute = (5000 + sysTickGetCount());
				}else if(ch == 115){//s
					SendNow = SEND_TRUE;
				}
				
				
       
      }
    }

   

    

   
    /*
    Enter sleep mode, this works, CPU wakes up once every ms.
    Have verified that by blinking Led using the mainDebugCounter instead of stc.
    */
    mainDebugCounter++;
    enterSleepMode();
  }
}

void Send_ADC_Value(){
			ADC1->CR |= (1 << 2);     // start sampling
			unsigned int x = ADC1->DR;
			char s[100];
			sendString(conv2Str(s, x * 330 / 4096));
}

void initADC()
{
    // This routine enables the ADC and sets up Tim2 to trigger conversions
    // at the specified  rate of  on Channel 8
    // Configure PA3 as an ADC input

    RCC->AHB2ENR |= (1 << 0);     // ensure GPIOA is enabled    
    GPIOA->MODER |= (1 << 6) | (1 << 7); // Set mode to analogue (ADC) 
    // Select ADC clock source = 80MHz system clock
    RCC->CCIPR |= (1 << 29) | (1 << 28);
    RCC->AHB2ENR |= (1 << 13);    // Enable ADC
    RCC->AHB2RSTR &= ~(1 << 13);  // Take ADC out of reset
    ADC1->CR = 0;             // ensure  ADEN = 0 to allow configuration
    ADC1_CCR = (1 << 22);        // enable the voltage reference
    ADC1->CR = (1 << 28);         // Turn on the voltage reference

    // Start calibration sequence
    ADC1->CR |= (1 << 31);
    while(ADC1->CR & (1 << 31)); // wait for calibration to finish
   
			ADC1->SQR1 = 8 << 6;  // Channel 8 is the only channel in the conversion sequence.
    ADC1->CR |= (1 << 0);     // enable ADC
    
}
