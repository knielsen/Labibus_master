#include <inttypes.h>
#include <string.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

/*
  Pinout:

  DI  PB1
  DE  PD3
  RE  PD2
  RO  PB0
*/


/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


static void
serial_output_str(const char *str)
{
  char c;

  while ((c = *str++))
    ROM_UARTCharPut(UART0_BASE, c);
}


static void
config_led(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
}


__attribute__ ((unused))
static void
led_on(void)
{
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
}


__attribute__ ((unused))
static void
led_off(void)
{
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
}


static void
rs485_tx_mode(void)
{
  ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
  ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
}


static void
rs485_rx_mode(void)
{
  ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);
  ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
}


static void
send_to_slave(const char *s)
{
  rs485_tx_mode();
ROM_SysCtlDelay(300);
  while (*s)
    ROM_UARTCharPut(UART1_BASE, *s++);
  while (ROM_UARTBusy(UART1_BASE))
    ;
ROM_SysCtlDelay(300);
  rs485_rx_mode();
}


static void
receive_from_slave(char *buf, uint32_t size)
{
  uint32_t i;
  uint32_t c;

ROM_SysCtlDelay(300);
  rs485_rx_mode();
ROM_SysCtlDelay(300);
  i = 0;
  for (;;)
  {
    c = ROM_UARTCharGet(UART1_BASE);
    if (c == '\n')
      break;
    if (c == '\r' || c == '\0')
      continue;
    if (i > size-1)
      continue;
    buf[i++] = c;
  }
  buf[i] = 0;

  ROM_SysCtlDelay((MCU_HZ/(3*1000))*2);
}



int main()
{
  /* Use the full 80MHz system clock. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  ROM_FPULazyStackingEnable();

  /* Configure serial. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 2400,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
  ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
  ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART1_BASE, (ROM_SysCtlClockGet()), 2400,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);
  rs485_tx_mode();

  config_led();

  ROM_SysCtlDelay(50000000);
  serial_output_str("Master initialised.\n");

  for (;;)
  {
    char buf[100];

    led_on();
    serial_output_str("Sending 'hello' to slave...\r\n");
    send_to_slave("bullu\r\n");
    led_off();
    receive_from_slave(buf, sizeof(buf));
    serial_output_str("Got from slave: '");
    serial_output_str(buf);
    serial_output_str("'\r\n");
  }
}
