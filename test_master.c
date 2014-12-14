#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "inc/hw_timer.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"


/*
  Pinout:

  DI  PB1
  DE  PD3
  RE  PD2
  RO  PB0
*/


#define MAX_DESCRIPTION 140
#define MAX_UNIT 20
#define MAX_REQ (20+MAX_DESCRIPTION+MAX_UNIT)

#define MAX_DEVICE 128


/*
  Timeouts. When waiting for a response from a slave, we have two timeouts
  that will be taken as slave not present or not responding.

  TIMEOUT_CHAR is the maximum time without receiving any new bytes; this
  should quickly flag a non-present device so we can move on to try a
  discover request on the next device id.

  TIMEOUT_RESPONSE is to avoid an infinite loop if we somehow continue to
  receive bytes without ever seeing a response end (\n) - though if we do,
  it would seem to indicate an errant device that is likely to disturb any
  further communication attempts also.

  The timeouts are in milliseconds.
*/
#define TIMEOUT_RESPONSE 2000   /* ToDo: reduce for higher baud rate. */
#define TIMEOUT_CHAR 10


/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


#if MAX_REQ > 255
#error MAX_REQ larger than 255, does not fit in uint8_t
#endif


/*
  Number of times a device is allowed to fail to respond to a poll or
  discover request, before being considered inactive.
*/
#define MAX_FAIL_RESPOND 10


/*
  Baudrate to use on the bus.

  The Atmega328 and similar UARTs used on the Arduinos is quite limited in
  their choice of serial line speed, it can only be CPU_FREQUENCY/(16*N) for
  integer N (or CPU_FREQUENCY/(8*N) in high-speed mode).

  These possible speeds do not match very well to the standard speeds
  available eg. to Linux stty, there are some few percentage errors.

  We want to run at 115200. But the closest speed the Arduino can do is
  16e6 / (8*17) = 117647 bits per second.

  So we run the master device at that same slightly-off bit rate. Then we can
  still debug using a standard speed of 115200 (with a slight error), and we
  get exact speed on the important intra-bus communications.
*/
#define RS485_BAUD (16000000/(8*17))


struct devdata {
  /* Time of last poll, or 0 if never polled yet. */
  uint64_t last_poll_time;
  /* Poll interval, in seconds. */
  uint16_t poll_interval;
  /*
    Active flag. Zero for a non-active device. Non-zero for an active device;
    then the value is the number of times the device is allowed to fail to
    respond to poll or discover before being considered inactive.
  */
  uint8_t active_count;
  /* Description, stored in quoted format (\xx). */
  uint8_t description[MAX_DESCRIPTION+1];
  /* Unit, stored in quoted format. */
  uint8_t unit[MAX_UNIT+1];
};


static struct devdata devices[MAX_DEVICE];
/* Index of next device to attempt discovery for. */
static uint32_t discover_idx = 0;


/*
  Seems we get an undefined reference to this if using sprintf().
  It doesn't seem to be actually used, just define a dummy one.
*/
void *_sbrk(uint32_t dummy) { for (;;) { } }


/* CRC-16. */
static const uint16_t crc16_tab[256] = {
  0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
  0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
  0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
  0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
  0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
  0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
  0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
  0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
  0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
  0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
  0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
  0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
  0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
  0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
  0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
  0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
  0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
  0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
  0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
  0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
  0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
  0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
  0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
  0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
  0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
  0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
  0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
  0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
  0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
  0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
  0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
  0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};


static uint32_t crc16(uint32_t byte, uint32_t crc_val)
{
  return crc16_tab[(crc_val ^ byte) & 0xff] ^ (crc_val >> 8);
}


static uint32_t crc16_buf(const uint8_t *buf, uint32_t len)
{
  uint32_t crc_val = 0;
  while (len > 0)
  {
    crc_val = crc16(*buf, crc_val);
    ++buf;
    --len;
  }
  return crc_val;
}


static uint32_t
hex2dec(uint32_t c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  else if (c >= 'A' && c <= 'F')
    return c - ('A'-10);
  else if (c >= 'a' && c <= 'f')
    return c - ('a'-10);
  else
    return 0;
}


static uint32_t
dec2hex(uint32_t x)
{
  if (x <= 9)
    return x + '0';
  else
    return x + ('a' - 10);
}


static void
serial_output_hexdig(uint32_t dig)
{
  ROM_UARTCharPut(UART0_BASE, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


__attribute__((unused))
static void
serial_output_hexbyte(uint8_t byte)
{
  serial_output_hexdig(byte >> 4);
  serial_output_hexdig(byte & 0xf);
}


static void
serial_output_str(const char *str)
{
  char c;

  while ((c = *str++))
    ROM_UARTCharPut(UART0_BASE, c);
}


__attribute__ ((unused))
static char *
uint32_tostring(char *buf, uint32_t val)
{
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p = '\0';
  return p;
}


 __attribute__ ((unused))
static void
println_uint32(uint32_t val)
{
  char buf[13];
  char *p = uint32_tostring(buf, val);
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_output_str(buf);
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
setup_timer(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
  ROM_TimerConfigure(WTIMER0_BASE, TIMER_CFG_PERIODIC_UP);
  ROM_TimerLoadSet64(WTIMER0_BASE, ~(uint64_t)0);
  ROM_TimerEnable(WTIMER0_BASE, TIMER_A);
}


static uint64_t
current_time(void)
{
  uint64_t v = ROM_TimerValueGet64(WTIMER0_BASE);
  /* Return time in milliseconds. */
  return v / (MCU_HZ / 1000);
}


static void
delay_milliseconds(uint32_t ms)
{
  uint64_t target_clocks = ROM_TimerValueGet64(WTIMER0_BASE) +
    (uint64_t)ms * (MCU_HZ / 1000);
  while (ROM_TimerValueGet64(WTIMER0_BASE) < target_clocks)
    ;
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
device_active(uint32_t dev)
{
  char buf[MAX_REQ + 50];
  snprintf(buf, sizeof(buf)-1, "ACTIVE %u|%u|%s|%s\n", (unsigned)dev,
           (unsigned)devices[dev].poll_interval, devices[dev].description,
           devices[dev].unit);
  serial_output_str(buf);
}


static void
device_inactive(uint32_t dev)
{
  char buf[50];
  snprintf(buf, sizeof(buf)-1, "INACTIVE %u\n", (unsigned)dev);
  serial_output_str(buf);
}


static void
device_poll_result(uint32_t dev, const char *val_str)
{
  char buf[50];
  snprintf(buf, sizeof(buf)-1, "POLL %u %s\n", (unsigned)dev, val_str);
  serial_output_str(buf);
}



static void
send_to_slave(const char *s)
{
  uint32_t crc;
  uint32_t c;

  rs485_tx_mode();
ROM_SysCtlDelay(300);
  /*
    Send a dummy byte of all one bits. This should ensure that the UART state
    machine can sync up to the byte boundary, as it prevents any new start bit
    being seen for one character's time.
  */
  ROM_UARTCharPut(UART1_BASE, 0xff);
  crc = 0;
  while ((c = *s++))
  {
    crc = crc16(c, crc);
#if TODO_FIX_QUOTING
    if (c < ' ' || c >= 127 || c == '!' || c == '?' || c == '|' || c == '\\' ||
        c == ':')
    {
      /* Handle escaping. */
      ROM_UARTCharPut(UART1_BASE, '\\');
      ROM_UARTCharPut(UART1_BASE, dec2hex(c >> 4));
      ROM_UARTCharPut(UART1_BASE, dec2hex(c & 0xf));
    }
    else
#endif
      ROM_UARTCharPut(UART1_BASE, c);
  }
  /* Send the CRC and the end marker. */
  ROM_UARTCharPut(UART1_BASE, dec2hex(crc >> 12));
  ROM_UARTCharPut(UART1_BASE, dec2hex((crc >> 8) & 0xf));
  ROM_UARTCharPut(UART1_BASE, dec2hex((crc >> 4) & 0xf));
  ROM_UARTCharPut(UART1_BASE, dec2hex(crc & 0xf));
  ROM_UARTCharPut(UART1_BASE, '\r');
  ROM_UARTCharPut(UART1_BASE, '\n');
  while (ROM_UARTBusy(UART1_BASE))
    ;
ROM_SysCtlDelay(300);
  rs485_rx_mode();
}


/*
  Try to receive a reply from a slave.

  Returns the number of bytes received. Returns 0 in case of timeout.
*/
static uint32_t
receive_from_slave(char *buf, uint32_t size)
{
  uint32_t i;
  uint32_t c;
  uint64_t start_time, last_char_time, now_time;

  start_time = last_char_time = current_time();

  /*
    Drain any existing junk in the UART FIFO before switching to receive mode
    on the RS485 line.
  */
  while (ROM_UARTCharsAvail(UART1_BASE))
    (void)ROM_UARTCharGet(UART1_BASE);

ROM_SysCtlDelay(300);
  rs485_rx_mode();
ROM_SysCtlDelay(300);
  i = 0;
  for (;;)
  {
    /* Wait for a char to arrive, or for timeout. */
    for (;;)
    {
      now_time = current_time();
      if (ROM_UARTCharsAvail(UART1_BASE))
        break;
      if (now_time - last_char_time >= TIMEOUT_CHAR ||
          now_time - start_time >= TIMEOUT_RESPONSE)
        return 0;
    }
    last_char_time = now_time;
    c = ROM_UARTCharGet(UART1_BASE);
    /* Wait for start-of-frame. */
    if (!i && c != '!')
      continue;

    if (c == '\n')
      break;
    if (c == '\r' || c == '\0')
      continue;
    if (i > size-1)
      continue;
    buf[i++] = c;
  }
  buf[i] = 0;

  /*
    Give the slave device 2 milliseconds to release transmit mode on the RS485
    line.
  */
  delay_milliseconds(2);

  return i;
}


static uint32_t
check_poll(uint32_t dev)
{
  struct devdata *p = &devices[dev];

  if (!p->active_count)
    return 0;
  if (!p->last_poll_time ||
      p->last_poll_time + 1000*p->poll_interval <= current_time())
    return 1;
  return 0;
}


static void
device_not_responding (uint32_t dev, uint32_t force_report)
{
  if (devices[dev].active_count > 0)
  {
    --devices[dev].active_count;
    if (devices[dev].active_count == 0)
    {
      devices[dev].last_poll_time = 0;
      devices[dev].poll_interval = 0;
      strcpy((char *)devices[dev].description, "");
      strcpy((char *)devices[dev].unit, "");
      device_inactive(dev);
    }
  }
  else if (force_report)
    device_inactive(dev);
}


static void
do_discover(uint32_t dev, uint32_t force_report)
{
  char buf[MAX_REQ];
  uint32_t rcv_len;
  char *p, *q, *descr_start, *unit_start, *crc_start;
  uint32_t descr_len, unit_len;
  uint32_t calc_crc, rcv_crc;
  uint32_t poll_interval;

  sprintf(buf, "?%02x:D|", (unsigned)(dev & 0x7f));

  led_on();
  send_to_slave(buf);
  led_off();
  rcv_len = receive_from_slave(buf, sizeof(buf));

  if (!rcv_len)
    goto badresponse;

  if (buf[0] != '!' ||
      buf[3] != ':' ||
      buf[4] != 'D')
    goto badresponse;

  if ( ((hex2dec(buf[1]) << 4) | hex2dec(buf[2])) != dev)
    goto badresponse;

  p = &buf[5];
  poll_interval = strtoul(p, &q, 10);
  if (q <= p)
    goto badresponse;
  if (*q != '|')
    goto badresponse;
  descr_start = q+1;
  for (p = descr_start; p < buf + rcv_len && *p != '|'; ++p)
    ;
  if (p >= buf + rcv_len)
    goto badresponse;
  descr_len = p - descr_start;
  if (descr_len > MAX_DESCRIPTION)
    goto badresponse;

  unit_start = p+1;
  for (p = unit_start; p < buf + rcv_len && *p != '|'; ++p)
    ;
  if (p >= buf + rcv_len)
    goto badresponse;
  unit_len = p - unit_start;
  if (unit_len > MAX_DESCRIPTION)
    goto badresponse;

  crc_start = p+1;
  if (crc_start - buf + 4 != rcv_len)
    goto badresponse;
  calc_crc = crc16_buf((uint8_t *)buf, crc_start-buf);
  rcv_crc = (hex2dec(crc_start[0]) << 12) |
    (hex2dec(crc_start[1]) << 8) |
    (hex2dec(crc_start[2]) << 4) |
    hex2dec(crc_start[3]);
  if (calc_crc != rcv_crc)
  {
    serial_output_str("CRC mismatch on device ");
    println_uint32(dev);
    goto badresponse;
  }

  /* Ok, device responded to discover request. Save its data. */
  if (!devices[dev].active_count)
    devices[dev].last_poll_time = 0;
  devices[dev].active_count = MAX_FAIL_RESPOND;
  if (poll_interval != devices[dev].poll_interval)
    force_report = 1;
  devices[dev].poll_interval = poll_interval;
  if (memcmp(devices[dev].description, descr_start, descr_len) ||
      devices[dev].description[descr_len] != '\0')
    force_report = 1;
  memcpy(devices[dev].description, descr_start, descr_len);
  devices[dev].description[descr_len] = '\0';
  if (memcmp(devices[dev].unit, unit_start, unit_len) ||
      devices[dev].unit[unit_len] != '\0')
    force_report = 1;
  memcpy(devices[dev].unit, unit_start, unit_len);
  devices[dev].unit[unit_len] = '\0';

  if (force_report)
    device_active(dev);

  return;

badresponse:
  device_not_responding(dev, force_report);
}


static void
do_poll(uint32_t dev)
{
  char buf[MAX_REQ];
  uint32_t rcv_len;
  char *p, *q, *val_start, *crc_start;
  uint32_t calc_crc, rcv_crc;
  uint64_t start_time;

  start_time = current_time();
  sprintf(buf, "?%02x:P|", (unsigned)(dev & 0x7f));

  led_on();
  send_to_slave(buf);
  led_off();
  rcv_len = receive_from_slave(buf, sizeof(buf));

  if (!rcv_len)
  {
    serial_output_str("Timeout from poll on device ");
    println_uint32(dev);
    goto badresponse;
  }

  if (buf[0] != '!' ||
      buf[3] != ':' ||
      buf[4] != 'P')
    goto badresponse;

  if ( ((hex2dec(buf[1]) << 4) | hex2dec(buf[2])) != dev)
    goto badresponse;

  val_start = &buf[5];
  for (p = val_start; p < buf + rcv_len && *p != '|'; ++p)
    ;
  if (p >= buf + rcv_len)
    goto badresponse;
  crc_start = p+1;
  if (crc_start - buf + 4 != rcv_len)
    goto badresponse;
  calc_crc = crc16_buf((uint8_t *)buf, crc_start-buf);
  rcv_crc = (hex2dec(crc_start[0]) << 12) |
    (hex2dec(crc_start[1]) << 8) |
    (hex2dec(crc_start[2]) << 4) |
    hex2dec(crc_start[3]);
  if (calc_crc != rcv_crc)
  {
    serial_output_str("CRC mismatch on device ");
    println_uint32(dev);
    goto badresponse;
  }

  /* Also check for a valid floating-point format for the value. */
  *p = '\0';
  strtof(val_start, &q);
  if (q != p)
    goto badresponse;

  /* Ok, device responded to poll request. */
  devices[dev].active_count = MAX_FAIL_RESPOND;
  device_poll_result(dev, val_start);
  devices[dev].last_poll_time = start_time;

  return;

badresponse:
  /*
    Do not reset the poll time the first few times that the device fails
    to respond to poll; this way we will re-poll the device immediately
    a couple of times before falling back to periodic poll (and eventually
    to giving up and declaring the device inactive).
  */
  if (devices[dev].active_count <= MAX_FAIL_RESPOND/2)
    devices[dev].last_poll_time = start_time;
  device_not_responding(dev, 0);
}


static uint64_t next_full_report_time = 0;

static void
poll_n_discover_loop(void)
{
  uint32_t do_full_report = 1;

  for (;;)
  {
    uint32_t dev;

    /* First, poll any device that has reached its next poll interval. */
    for (dev = 0; dev < MAX_DEVICE; ++dev)
    {
      if (check_poll(dev))
        do_poll(dev);
    }
    /*
      Next, send a discover request for the next device id in line.
      We send discover requests to all devices, active and non-active alike.
      This way, we will catch updated description/unit strings, even if the
      device manages to update quickly enough to not miss enough polls to be
      marked as non-active.
    */
    dev = discover_idx;
    do_discover(dev, do_full_report);
    ++dev;
    if (dev >= MAX_DEVICE)
    {
      dev = 0;
      if (do_full_report)
      {
        /*
          We will send a full report periodically, even if nothing changes.
          This should help avoid us somehow being out of sync for long.
        */
        next_full_report_time = current_time() + 5*60*1000;
        do_full_report = 0;
      }
      else if (current_time() >= next_full_report_time)
        do_full_report = 1;
    }
    discover_idx = dev;

    /* Server can send us a line to request full activity dump. */
    if (ROM_UARTCharsAvail(UART0_BASE))
    {
      do
      {
        uint8_t c = ROM_UARTCharGet(UART0_BASE);
        if (c == '\n')
          next_full_report_time = current_time();
      } while (ROM_UARTCharsAvail(UART0_BASE));
    }
  }
  /* NOTREACHED */
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
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 500000,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
  ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
  ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART1_BASE, (ROM_SysCtlClockGet()), RS485_BAUD,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);
  rs485_tx_mode();

  config_led();

  setup_timer();

  ROM_SysCtlDelay(50000000);
  serial_output_str("Master initialised.\n");

  poll_n_discover_loop();
}
