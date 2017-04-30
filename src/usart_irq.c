

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/dac.h>

// /usr/arm-linux-gnueabi/include/string.h
#include <string.h>
#include <stdlib.h> // malloc
//#include <strings.h> // bzero
#include <stdio.h> // sprintf?


#include <errno.h>
// #include <stdio.h>
#include <unistd.h>

int _write(int file, char *ptr, int len);

static void clock_setup(void)
{
	/* Enable GPIOE clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART1. */
  // JA
	rcc_periph_clock_enable(RCC_USART1);
}

static void usart_setup(void)
{
	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ); // JA

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);


	/* Setup GPIO pins for USART1 receive. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO10);

	/* Setup USART1 TX and RX pin as alternate function. */
	// gpio_set_af(GPIOA, GPIO_AF7, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);  // JA  tx

	// gpio_set_af(GPIOA, GPIO_AF7, GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO10); // JA    rx

	/* Setup USART1 parameters. */
	// usart_set_baudrate(USART1, 38400);
	usart_set_baudrate(USART1, 115200); // JA
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Enable USART1 Receive interrupt. */
	usart_enable_rx_interrupt(USART1);

  // enable tx interuppt - to tell us when the tx buf is bready to bwrite	
  // usart_enable_tx_interrupt(USART1);

	/* Finally enable the USART. */
	usart_enable(USART1);
}



static void led_setup(void)
{
  // led
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_OTYPE_OD, GPIO0); 
}



static void dac_setup(void)
{
  // two dacs - seems to be pin pa4, and pa5 
  // http://www.sdvos.org/images/stm32f4discovery-pinout.png

  // using led - for dac output
  // also see void dac_software_trigger(data_channel dac_channel);

  // perhaps the 

  gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5);  // PA05   DAC2

  dac_disable(CHANNEL_2);
  dac_disable_waveform_generation(CHANNEL_2);
  dac_enable(CHANNEL_2);
  dac_set_trigger_source(DAC_CR_TSEL2_SW);
}





typedef struct Buffer 
{
  // be better to use an index -- easier to do the bwrite and bread pointers
  unsigned wi;
  unsigned ri;
  // unsigned size;
  // TODO make dynamic?
  char buf[1000];
} Buffer;



typedef struct App {

  Buffer  receive;
  Buffer  transmit;

} App;

App a;




static void bwrite(Buffer *r, void *buf, size_t count)
{
    size_t i = 0;
    char *p = buf;

    for(i = 0; i < count; ++i) {

			r->buf[r->wi++] = *p++;

      // handle ri wrap around...
      if(r->wi == 1000) 
        r->wi = 0;
    }
}


static size_t bread(Buffer *r, void *buf, size_t count)
{
  size_t i = 0;
  char *p = buf;

  for(i = 0; i < count && r->ri != r->wi; ++i) {

    // bwrite the output buffer
    *p++ = r->buf[r->ri++];

    // handle ri wrap around
    if(r->ri == 1000) 
      r->ri = 0;
  }  

  // number of chars bread
  return i;
}


int _write(int file, char *ptr, int len)
{
  // overloaded for printf etc....

  if (file == STDOUT_FILENO || file == STDERR_FILENO) {

    // bwrite the output buffer
    bwrite(&a.transmit, ptr, len);

    // enable tx interuppt.
    usart_enable_tx_interrupt(USART1);

    // and call trigger interupt... to start writing...
    usart1_isr();

    return len;
  }
  errno = EIO;
  return -1;
}



int main(void)
{
  uint16_t target = 0xffff;

	clock_setup();
	led_setup();
  dac_setup();
	usart_setup();

  memset(&a, 0, sizeof(App));

  printf("hithere!");

	while (1) {

    char buf[101];
    size_t n = bread( &a.receive, buf, 100);

    // we need to split by \n and handle properly.... 


    if(n != 0) {

        if(target == 0xffff)
          target = 0;
        else
          target = 0xffff;

        printf("1 target is %d", target);

        dac_load_data_buffer_single(target, RIGHT12, CHANNEL_2);
        dac_software_trigger(CHANNEL_2);
  

        // toggle led...
		    gpio_toggle(GPIOE, GPIO0);

        // trim what we bread
        if(buf[n - 1] == '\n')
          buf[n - 1] = 0;
        else
          buf[n] = 0;


        printf("*** you wrote %d chars '%s'\n> ", n, buf);

    }
	}

	return 0;
}

// ok it's not continuing to get the interupt - 

/*
  important,
  - As you know, when an event occurs, the corresponding flag bit is set, whether interrupts are enabled or not. 
  - If the associated interrupt is enabled, then the MCU jumps to the relevant ISR, where the event may be handled.
*/


void usart1_isr(void)
{
  // handle rx
	while ((USART_SR(USART1) & USART_SR_RXNE) != 0) {
      uint16_t ch = USART_DR(USART1);
      bwrite(&a.receive, &ch, 1);
  } 

  // handle tx
  while((USART_SR(USART1) & USART_SR_TXE) != 0) {
      // check if more chars to transmit 
      unsigned char ch = 0;
      uint32_t n = bread(&a.transmit, &ch, 1);
      if(n == 1) {
        // yes, then bwrite to usart
        USART_DR(USART1) = (uint16_t) ch & 0xff;
      } else {
        // no, then disable tx interupt and break from loop
        usart_disable_tx_interrupt(USART1);
        break;
      }
  }

  /*
    note tx_interupt - just enables an interupt for tx empty
      void usart_enable_tx_interrupt(uint32_t usart)
      {
        USART_CR1(usart) |= USART_CR1_TXEIE;
  */
}


/*
  // this just loops until, the the txe is empty?
  do {
    reg = USART_SR(CONSOLE_UART);
  } while ((reg & USART_SR_TXE) == 0); 
  USART_DR(CONSOLE_UART) = (uint16_t) c & 0xff;
*/



#if 0
void usart1_isr___(void)
{
	static uint8_t data = 'A';

  // RXNE - RX Buffer Not Empty
  // CR - control register
  // SR - status register
  // #define USART_SR_ORE			(1 << 3) - overrun 
  // rxneie - rx not empty interupt enable... 

  // what is SR_TXE  - empty. 
  // TXE: Transmit data buffer empty

  // i think the code just falls through - because extra flags are set... eg. the tx doesn't happen in another
  // interrupt...
  // are we allowed to malloc in an interrupt?

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		gpio_toggle(GPIOE, GPIO0);

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART1);

    // whole thing should be a while loop...
    // see the console uart exampe
    *a.pb++ = data;

		/* Enable transmit interrupt so it sends back the data. */
		usart_enable_tx_interrupt(USART1);
	}

  // don't really understand what's going on here...
  // it's like enabling the tx interupt means that it gets called...
  // ahhh - the uart will dispatch - on whatever.

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		/* Put data into the transmit register. */
		usart_send(USART1, data);

		/* Disable the TXE interrupt as we don't need it anymore. */
		usart_disable_tx_interrupt(USART1);
	}
}

#endif


////////////////////////////////////////////
/*
#define CONSOLE_UART USART1

void console_putc(char c);
void console_puts(char *s);

void console_putc(char c)
{
  uint32_t  reg;

  // this just loops until, the the txe is empty?
  do {
    reg = USART_SR(CONSOLE_UART);
  } while ((reg & USART_SR_TXE) == 0); 
  USART_DR(CONSOLE_UART) = (uint16_t) c & 0xff;
}


void console_puts(char *s)
{
  while (*s != '\000') {
    console_putc(*s);
    // Add in a carraige return, after sending line feed 
    if (*s == '\n') {
      console_putc('\r');
    }
    s++;
  }
}
*/


////////////////////////////////////////////

// the problem is the shrink operattion - where we copy and then adjust the pointer back
// is not going to work.  - because an interupt could occur in the middle of it.

// i think that's why the ring works - one just keeps going around in a circle... with a bwrite pointer, 
// and a bread pointer.

// ring buffer - just needs to be i % 1000 - really quite easy... ahhh except for  
// actually, i = ++i % 1000

// use the ring buffer. and then implement a, int bread(int sz, char *buf) operation on top of that 
// implement a non-blocking bwrite buffer as well... but first just do a blocking bwrite.


