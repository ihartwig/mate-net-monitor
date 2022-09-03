/*
  $Id: MAX3100Serial.cpp,v 1.6 2018/03/07 12:40:42 ewan Exp $
  Arduino MAX3100Serial library.
  MAX3100Serial.cpp (C) 2016 Ewan Parker.

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

  The author may be reached at https://www.ewan.cc/ for queries.


  A Maxim Integrated MAX3100 external USART/UART communication library for
  Arduino, built to be source code compatible with the Serial library, etc.
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
#define MAX3100_CONF_RM             0b0000010000000000

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

// Global SPI settings for all MAX3100 communications.
SPISettings spiSet = SPISettings(4000000, MSBFIRST, SPI_MODE0);

// Constructor.  We need the crystal speed and the chip select pin number.
MAX3100Serial::MAX3100Serial(uint32_t crystalFrequencykHz, pin_t csPin)
{
  _rx_buffer_next = 0;
  _rx_buffer_last = 0;
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

// Set the communication rate.  Only allow "standard" rates, and default to
// 9600 bps if the rate is nonstandard or not supported.
void MAX3100Serial::begin(uint32_t speed)
{
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

  SPI.beginTransaction(spiSet);
  digitalWrite(_chipSelectPin, LOW);
  // Write the configuration: /RM (read interrupt mask) and chosen baud.
  // The assumption here is that the Arduino is interested in knowing when
  // serial data arrives, rather than the terminal connected to the MAX3100's
  // USART knowing.
  //SPI.transfer16(MAX3100_CMD_WRITE_CONF | MAX3100_CONF_RM | conf);
  conf = MAX3100_CMD_WRITE_CONF | MAX3100_CONF_RM | conf;
  SPI.transfer(0xFF & (conf >> 8));
  SPI.transfer(0xFF & (conf));
  digitalWrite(_chipSelectPin, HIGH);
  SPI.endTransaction();

  // Call into fetch to clear the fifo
  // this->fetch();
}

void MAX3100Serial::end()
{
  SPI.end();
}

int MAX3100Serial::read()
{
  if (!available()) {
    return -1;  // No data available
  }
  // read from the buffer first then update indexes
  int c = _rx_buffer[_rx_buffer_next];
  _rx_buffer_next = (_rx_buffer_next+1) % MAX3100_BUF_LEN;
  return c;
}

int MAX3100Serial::_read()
{
  int c = -1; // No data available.
  SPI.beginTransaction(spiSet);
  digitalWrite(_chipSelectPin, LOW);
  // uint16_t r = SPI.transfer16(MAX3100_CMD_READ_DATA);
  uint16_t rbuf = 0;
  rbuf |= SPI.transfer((uint8_t)(MAX3100_CMD_READ_DATA >> 8));
  rbuf = rbuf << 8;
  rbuf |= SPI.transfer(0);
  digitalWrite(_chipSelectPin, HIGH);
  SPI.endTransaction();
  if (rbuf & MAX3100_CONF_R)
    c = rbuf & 0xff;
  return c;
}

int MAX3100Serial::available()
{
  return (_rx_buffer_last != _rx_buffer_next);
}

int MAX3100Serial::_available()
{
  SPI.beginTransaction(spiSet);
  digitalWrite(_chipSelectPin, LOW);
  //uint16_t conf = SPI.transfer16(MAX3100_CMD_READ_CONF);
  uint16_t conf = 0;
  conf |= SPI.transfer((uint8_t)(MAX3100_CMD_READ_CONF >> 8));
  conf = conf << 8;
  conf |= SPI.transfer(0);
  digitalWrite(_chipSelectPin, HIGH);
  SPI.endTransaction();
  return (conf & MAX3100_CONF_R); // R flag is set
}

int MAX3100Serial::_busy()
{
  SPI.beginTransaction(spiSet);
  digitalWrite(_chipSelectPin, LOW);
  //uint16_t conf = SPI.transfer16(MAX3100_CMD_READ_CONF);
  uint16_t conf = 0;
  conf |= SPI.transfer((uint8_t)(MAX3100_CMD_READ_CONF >> 8));
  conf = conf << 8;
  conf |= SPI.transfer(0);
  digitalWrite(_chipSelectPin, HIGH);
  SPI.endTransaction();
  return (!(conf & MAX3100_CONF_T)); // T flag is not set
}

size_t MAX3100Serial::write(uint8_t b)
{
  // There is no buffer.  Wait for the tramsmit register to empty before
  // proceeding, otherwise the character may be lost.
  while (_busy()) {}
  SPI.beginTransaction(spiSet);
  digitalWrite(_chipSelectPin, LOW);
  //SPI.transfer16(MAX3100_CMD_WRITE_DATA | b);
  SPI.transfer((uint8_t)(MAX3100_CMD_WRITE_DATA >> 8));
  SPI.transfer(b);
  digitalWrite(_chipSelectPin, HIGH);
  SPI.endTransaction();
  return 1;
}

void MAX3100Serial::flush()
{
  // There is no buffer.  Wait for the transmit register to empty.
  while (_busy()) {}
}

int MAX3100Serial::peek()
{
  if (!available()) {
    return -1;  // No data available
  }
  return _rx_buffer[_rx_buffer_next];
}

int MAX3100Serial::fetch()
{
  // Respond to an IRQ by reading the byte into circular buffer
  int count = 0;
  int new_last = (_rx_buffer_last+1) % MAX3100_BUF_LEN;
  if (new_last == _rx_buffer_next) {
    // out of space, let the MAX3100 FIFO drop chars
    return -2;
  }
  int c = 0; // start assuming there is data to be read
  while (c != -1) {
    c = _read();
    if (c != -1) {
      // add to the buffer first then update indexes
      _rx_buffer[new_last] = c;
      _rx_buffer_last = new_last;
    }
    count++;
  }
  return count;
}
