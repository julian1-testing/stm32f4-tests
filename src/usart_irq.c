

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

// /usr/arm-linux-gnueabi/include/string.h
#include <string.h>
#include <stdlib.h> // malloc
//#include <strings.h> // bzero
#include <stdio.h> // sprintf?




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

	/* Finally enable the USART. */
	usart_enable(USART1);



}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO12 on GPIO port D for LED. */
	// gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);

  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_OTYPE_OD, GPIO0); // JA
}



// ok. we want want to factor, the ring buffers....

typedef struct Buffer 
{
  // be better to use an index -- easier to do the write and read pointers
  unsigned wi;
  unsigned ri;
  char buf[1000];
} Buffer;



typedef struct App {

  Buffer  receive;
/*
  // be better to use an index -- easier to do the write and read pointers
  unsigned wi;
  // char *rp;
  unsigned ri;
  char buf[1000];
*/

} App;

App a;


////////////////////////////////////////////
#define CONSOLE_UART USART1

void console_putc(char c);
void console_puts(char *s);

void console_putc(char c)
{
  uint32_t  reg;
  do {
    reg = USART_SR(CONSOLE_UART);
  } while ((reg & USART_SR_TXE) == 0); 
  USART_DR(CONSOLE_UART) = (uint16_t) c & 0xff;
}

void console_puts(char *s)
{
  while (*s != '\000') {
    console_putc(*s);
    /* Add in a carraige return, after sending line feed */
    if (*s == '\n') {
      console_putc('\r');
    }
    s++;
  }
}
////////////////////////////////////////////

// the problem is the shrink operattion - where we copy and then adjust the pointer back
// is not going to work.  - because an interupt could occur in the middle of it.

// i think that's why the ring works - one just keeps going around in a circle... with a write pointer, 
// and a read pointer.

// ring buffer - just needs to be i % 1000 - really quite easy... ahhh except for  
// actually, i = ++i % 1000

// use the ring buffer. and then implement a, int read(int sz, char *buf) operation on top of that 
// implement a non-blocking write buffer as well... but first just do a blocking write.




static void write(Buffer *a_, void *buf, size_t count)
{
    // TODO - put the size of the ring buffer in the buffer structure and use it.

    size_t i = 0;
    char *p = buf;

    for(i = 0; i < count; ++i) {

			a_->buf[a_->wi++] = *p++;

      // wrap around...
      if(a_->wi == 1000) 
        a_->wi = 0;
    }
}


static size_t read(Buffer *a_, void *buf, size_t count)
{
  size_t i = 0;
  char *p = buf;

  // we can't just subtract - because of wrap around...
  // therefore it has to be a loop
  // if count exceeds buffer - then that's a problem 

  for(i = 0; 
    i < count && a_->ri != a_->wi; 
    ++i) {

    // write the output buffer
    *p++ = a_->buf[a_->ri++];

    // handle wrap around
    if(a_->ri == 1000) 
      a_->ri = 0;
  }  

  // return number of characters written
  return i;
}



int main(void)
{
	clock_setup();
	gpio_setup();
	usart_setup();

  memset(&a, 0, sizeof(App));

	while (1) {

    char buf[101];
    size_t n = read( &a.receive, buf, 100);


    if(n != 0) {

        // we have data...
		    gpio_toggle(GPIOE, GPIO0);

        if(buf[n - 1] == '\n')
          buf[n - 1] = 0;
        else
          buf[n] = 0;

        // now we want to report what was written
        char s[1000];
        snprintf(s, 1000, "you wrote %d chars '%s'\n> ", n, buf);

        console_puts(s);
    }

	}

	return 0;
}

/*
  important,
  - As you know, when an event occurs, the corresponding flag bit is set, whether interrupts are enabled or not. 
  - If the associated interrupt is enabled, then the MCU jumps to the relevant ISR, where the event may be handled.
*/


void usart1_isr(void)
{
	uint32_t	reg;

	do {
		reg = USART_SR(USART1);
		if (reg & USART_SR_RXNE) {

      unsigned char ch = USART_DR(USART1);

      write(&a.receive, &ch, 1);
/*
      // write the buffer
			a.buf[a.wi] = USART_DR(USART1);

      // wrap around
      if(++a.wi == 1000)
        a.wi = 0;
*/

			/* Check for "overrun" */
      /*
			i = (recv_ndx_nxt + 1) % RECV_BUF_SIZE;
			if (i != recv_ndx_cur) {
				recv_ndx_nxt = i;
			}
      */
		}
	} while ((reg & USART_SR_RXNE) != 0); /* can read back-to-back interrupts */
}

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


