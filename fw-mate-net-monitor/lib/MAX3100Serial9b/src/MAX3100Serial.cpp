/*
  Arduino MAX3100Serial library.
  MAX3100Serial.cpp (C) 2016-2018 Ewan Parker.
  Adapted for Particle (C) 2023 Ian Hartwig.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.


  A Maxim Integrated MAX3100 external USART/UART communication library for
  Arduino, built to be source code compatible with the Serial library, etc.

  The Particle version implements an interrupt routine with read and write
  circular buffers. Data is transferred from the MAX3100 immediately by _isr()
  into buffers of a fixed size unless if they are full. Example size 8:
  Empty:
  buf[0] [1] [2] [3] [4] [5] [6] [7]
     tail ^ head
  With availableBytes() 2 in [1] and [2]:
  buf[0] [1] [2] [3] [4] [5] [6] [7]
     tail ^       ^ head
  Full:
  buf[0] [1] [2] [3] [4] [5] [6] [7]
             head ^   ^ tail
  Since head must point at an empty data location only size-1 locations are
  usable out of the allocated space size.
*/

#include <MAX3100Serial.h>
#include <Arduino.h>
#include <SPI.h>

// Commands.
#define MAX3100_CMD_WRITE_CONF      0b1100000000000000
#define MAX3100_CMD_READ_CONF       0b0100000000000000
#define MAX3100_CMD_WRITE_DATA      0b1000000000000000
#define MAX3100_CMD_READ_DATA       0b0000000000000000

// Configuration.
#define MAX3100_CONF_R              0b1000000000000000
#define MAX3100_CONF_T              0b0100000000000000
// SHDNo==0 can be used along with R/T to check for spi connectivity 
#define MAX3100_CONF_SHDN           0b0001000000000000
#define MAX3100_CONF_RM             0b0000010000000000
#define MAX3100_CONF_TM             0b0000100000000000
#define MAX3100_CONF_PE             0b0000000000100000
#define MAX3100_CONF_NOT_RT         0b0011111111111111

// Baud rates for clock multiplier x1.
#define MAX3100_CONF_BAUD_X1_115200 0b0000000000000000
#define MAX3100_CONF_BAUD_X1_57600  0b0000000000000001
#define MAX3100_CONF_BAUD_X1_38400  0b0000000000001000
#define MAX3100_CONF_BAUD_X1_19200  0b0000000000001001
#define MAX3100_CONF_BAUD_X1_9600   0b0000000000001010
#define MAX3100_CONF_BAUD_X1_4800   0b0000000000001011
#define MAX3100_CONF_BAUD_X1_2400   0b0000000000001100
#define MAX3100_CONF_BAUD_X1_1200   0b0000000000001101
#define MAX3100_CONF_BAUD_X1_600    0b0000000000001110
#define MAX3100_CONF_BAUD_X1_300    0b0000000000001111

// Baud rates for clock multiplier x2.
#define MAX3100_CONF_BAUD_X2_230400 0b0000000000000000
#define MAX3100_CONF_BAUD_X2_115200 0b0000000000000001
#define MAX3100_CONF_BAUD_X2_57600  0b0000000000000010
#define MAX3100_CONF_BAUD_X2_38400  0b0000000000001001
#define MAX3100_CONF_BAUD_X2_19200  0b0000000000001010
#define MAX3100_CONF_BAUD_X2_9600   0b0000000000001011
#define MAX3100_CONF_BAUD_X2_4800   0b0000000000001100
#define MAX3100_CONF_BAUD_X2_2400   0b0000000000001101
#define MAX3100_CONF_BAUD_X2_1200   0b0000000000001110
#define MAX3100_CONF_BAUD_X2_600    0b0000000000001111

#define MASK_8b                     0xFF
#define MASK_9b                     0x1FF

// Global SPI settings for all MAX3100 communications.
// Could run up to 4MHz, conservatively run at 2MHz
SPISettings spiSet = SPISettings(2000000, MSBFIRST, SPI_MODE0);


// Constructor.  We need the crystal speed and the chip select pin number.
MAX3100Serial::MAX3100Serial(uint32_t crystalFrequencykHz, pin_t csPin, pin_t intPin)
{
  // reset count metrics
  count_irq = 0;
  count_sent = 0;
  count_read = 0;
  count_read_err = 0;
  count_overflow = 0;
  // pre-setup hardware
  _intPin = intPin;
  _setChipSelectPin(csPin);
  _setClockMultiplier(crystalFrequencykHz);
  SPI.begin();
}


// Destructor
MAX3100Serial::~MAX3100Serial()
{
  end();
}


void MAX3100Serial::_setChipSelectPin(pin_t csPin)
{
  // Deselect the chip and set the pin as an output.
  _chipSelectPin = csPin;
  digitalWrite(csPin, HIGH);
  pinMode(csPin, OUTPUT);
}


void MAX3100Serial::_setClockMultiplier(uint32_t kHz)
{
  // Determine the clock multiplier from its frequency.
  if (kHz == 3686) _clockMultiplier = 2;    // 3.6864 MHz
  else _clockMultiplier = 1;                // 1.8432 MHz
}


// Wrapper for the 16-bit word transfers used by the MAX3100 exclusively
inline uint16_t MAX3100Serial::_transfer16b(uint16_t send_buf) {
  uint16_t read_buf = 0;
  SPI.beginTransaction(spiSet);
  digitalWrite(_chipSelectPin, LOW);
  read_buf |= SPI.transfer((send_buf >> 8) & MASK_8b);
  read_buf = read_buf << 8;
  read_buf |= SPI.transfer(send_buf & MASK_8b);
  digitalWrite(_chipSelectPin, HIGH);
  SPI.endTransaction();
  return read_buf;
}


// Set the communication rate.  Only allow "standard" rates, and default to
// 9600 bps if the rate is nonstandard or not supported.
// parityEnable: when non-0 set the PE bit to allow 9th bit send & receive
//   read() and write() will calculate Pt and check Pr bit
//   read9b() and write9b() will pass the 9th (Pt/Pr) bit through
void MAX3100Serial::begin(uint32_t speed, int parityEnable)
{
  _parityEnable = !!(parityEnable);
  uint16_t conf;
  if (_clockMultiplier == 2)
  {
    switch (speed)
    {
      case 230400 : conf = MAX3100_CONF_BAUD_X2_230400; break;
      case 115200 : conf = MAX3100_CONF_BAUD_X2_115200; break;
      case 57600  : conf = MAX3100_CONF_BAUD_X2_57600; break;
      case 38400  : conf = MAX3100_CONF_BAUD_X2_38400; break;
      case 19200  : conf = MAX3100_CONF_BAUD_X2_19200; break;
      case 4800   : conf = MAX3100_CONF_BAUD_X2_4800; break;
      case 2400   : conf = MAX3100_CONF_BAUD_X2_2400; break;
      case 1200   : conf = MAX3100_CONF_BAUD_X2_1200; break;
      case 600    : conf = MAX3100_CONF_BAUD_X2_600; break;
      default     : conf = MAX3100_CONF_BAUD_X2_9600; break;
    }
  }
  else
  {
    switch (speed)
    {
      case 115200 : conf = MAX3100_CONF_BAUD_X1_115200; break;
      case 57600  : conf = MAX3100_CONF_BAUD_X1_57600; break;
      case 38400  : conf = MAX3100_CONF_BAUD_X1_38400; break;
      case 19200  : conf = MAX3100_CONF_BAUD_X1_19200; break;
      case 4800   : conf = MAX3100_CONF_BAUD_X1_4800; break;
      case 2400   : conf = MAX3100_CONF_BAUD_X1_2400; break;
      case 1200   : conf = MAX3100_CONF_BAUD_X1_1200; break;
      case 600    : conf = MAX3100_CONF_BAUD_X1_600; break;
      case 300    : conf = MAX3100_CONF_BAUD_X1_300; break;
      default     : conf = MAX3100_CONF_BAUD_X1_9600; break;
    }
  }
  // Particle: sets pin mode of SCK, MOSI, MISO, SS
  SPI.begin(_chipSelectPin);
  // Configure for /RM (R set) and /TM (T becomes set) so that _irq() can take
  // action to send/receive words into the buffers. conf already contains baud
  // set bits.
  conf = MAX3100_CMD_WRITE_CONF | MAX3100_CONF_RM | MAX3100_CONF_TM | conf;
  if (_parityEnable) {
    conf |= MAX3100_CONF_PE;
  }
  _transfer16b(conf);
  // add interrupt
  pinMode(_intPin, INPUT_PULLUP);
  attachInterrupt(_intPin, &MAX3100Serial::_isr, this, FALLING);
}


/* immediately read MAX3100 conf register */
int MAX3100Serial::readConf()
{
  return _transfer16b(MAX3100_CMD_READ_CONF);
}


/* immediately read MAX3100 data register */
int MAX3100Serial::readData()
{
  int rreg = 0;
  while ((rreg & MAX3100_CONF_R) == 0) {
    rreg = _transfer16b(MAX3100_CMD_READ_DATA);
  }
  return rreg;
}


void MAX3100Serial::end()
{
  // Particle: does NOT reset the pin mode of the SPI pins.
  SPI.end();
}


void MAX3100Serial::_readBufAppend(uint16_t word)
{
  // throw away this data if read_buf is full
  if ((_read_buf_head + 1) % READ_BUF_SIZE == _read_buf_tail) {
    count_overflow++;
    return;
  }
  // otherwise store it in the buffer
  _read_buf[_read_buf_head] = word;
  _read_buf_head = (_read_buf_head + 1) % READ_BUF_SIZE;
  // count metrics
  count_read++;
}


/* respond to IRQ active for T or R set in FALLING edge */
void MAX3100Serial::_isr()
{
  count_irq++;
  int isr_read_count = 0;
  int isr_write_count = 0;
  uint16_t wword = 0;
  uint16_t rword = 0;
  uint16_t cword = _transfer16b(MAX3100_CMD_READ_CONF);
  // reading and writing data to the MAX3100 can happen in the same transaction.
  // R is set when new data is available in FIFO - /ISR low when R is set.
  // T is set when transmit buffer is empty - /ISR goes low on buffer becoming
  // empty and is cleared after a read data op.
  // this logic continues to run until all the interrupt setting conditions are
  // clear which is important to work with the FALLING edge trigger.
  while ((cword & MAX3100_CONF_R) || (cword & MAX3100_CONF_T)) {
    // try to write and read at the same time if conditions allow it
    if ((cword & MAX3100_CONF_SHDN) == 0 && 
        (cword & MAX3100_CONF_T) && _write_buf_head != _write_buf_tail) {
      wword = _write_buf[_write_buf_tail];
      rword = 0;
      _write_buf_tail = (_write_buf_tail + 1) % WRITE_BUF_SIZE;
      rword = _transfer16b(MAX3100_CMD_WRITE_DATA | (wword & MASK_9b));
      // the write might have come with read data too, so capture that too
      if (rword & MAX3100_CONF_R) {
        _readBufAppend(rword & MASK_9b);
      }
      // count metrics
      count_sent++;
      isr_write_count++;
    }
    // read only - must not be ready to write
    else {
      rword = _transfer16b(MAX3100_CMD_READ_DATA);
      // save data when available (R was set)
      if (rword & MAX3100_CONF_R) {
        _readBufAppend(rword & MASK_9b);
        isr_read_count++;
      }
      // or this should have cleared /IRQ (only T was set)
      else {
        break;
      }
    }
    // for next loop
    cword = _transfer16b(MAX3100_CMD_READ_CONF);
  }
  return;
}


/* stream read - extra mask on read9b */
int MAX3100Serial::read()
{
  uint16_t bytes = read9b();
  uint8_t byte = bytes & MASK_8b;
  uint16_t pr = (bytes >> 8) & 0x1;
  if (pr != __builtin_parity(byte)) {
    count_read_err++;
  }
  return byte;
}


/* stream read from software buffer */
int16_t MAX3100Serial::read9b()
{
  // wait for data to enter the buffer, avoid using available()
  while (_read_buf_head == _read_buf_tail) {}
  // give back the next item in buffer
  uint16_t rbuf = _read_buf[_read_buf_tail];
  _read_buf_tail = (_read_buf_tail + 1) % READ_BUF_SIZE;
  return rbuf & MASK_9b;
}


/* stream check for items in software buffer */
int MAX3100Serial::available()
{
  // _read_buf is empty when head == tail
  return !(_read_buf_head == _read_buf_tail);
}


int MAX3100Serial::_busy()
{
  // T flag is not set
  return !(_transfer16b(MAX3100_CMD_READ_CONF) & MAX3100_CONF_T);
}


size_t MAX3100Serial::write(uint8_t byte)
{
  uint16_t pt = (__builtin_parity(byte) & 0x1) << 8;
  return write9b(pt | byte);
}


/* stream write byte by byte - extended by Print::write */
size_t MAX3100Serial::write9b(uint16_t bytes)
{
  // Log.info("MAX3100Serial::write(0x%x) _write_buf_head=%d, _write_buf_tail=%d", b, _write_buf_head, _write_buf_tail);
  // give error if the byte could not be handled
  if ((_write_buf_head + 1) % WRITE_BUF_SIZE == _write_buf_tail) {
    return 0;
  }
  // if we can, write out immediately
  // this might generate an extra interrupt that will be ignored, yet is
  // important to do because _irq() will only be called when T transitions
  noInterrupts();  // protect ordering of read bytes
  uint16_t cword = _transfer16b(MAX3100_CMD_READ_CONF);
  uint16_t rword = 0;
  if (cword & MAX3100_CONF_T) {
    rword = _transfer16b(MAX3100_CMD_WRITE_DATA | (bytes & MASK_9b));
    // the write might have come with read data too, so capture that too
    if (rword & MAX3100_CONF_R) {
      _readBufAppend(rword & MASK_9b);
    }
    // count metrics
    count_sent++;
  }
  else {
    // otherwise, put into the write buffer and let _irq() send
    _write_buf[_write_buf_head] = (bytes & MASK_9b);
    _write_buf_head = (_write_buf_head + 1) % WRITE_BUF_SIZE;
  }
  interrupts();  // protect ordering of read bytes
  // ack the byte sent
  return 1;
}


/* stream flush to hardware output */
void MAX3100Serial::flush()
{
  // wait for _write_buf to empty
  // while (_write_buf_head != _write_buf_tail) {}
}


int MAX3100Serial::peek()
{
  // This is not currently implemented in the hardware.  It could be implemented
  // with a one byte software buffer, but this would prevent /RM interrupts from
  // firing.
  return -1;
}


/* return 1 if there is room for length in the write buffer else 0 */
bool MAX3100Serial::availableForWrite(size_t length)
{
  size_t available_length =
    ((WRITE_BUF_SIZE-1) - _write_buf_head) + _write_buf_tail;
  available_length %= WRITE_BUF_SIZE;  // when wrapped-around
  return available_length >= length;
}


/* return count of bytes in the read buffer */
int MAX3100Serial::availableBytes()
{
  return ((READ_BUF_SIZE + _read_buf_head) - _read_buf_tail) % READ_BUF_SIZE;
}


/* drop all bytes in the read buffer */
void MAX3100Serial::readBufPurge()
{
  _read_buf_tail = _read_buf_head;
}
