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
// Could run up to 4MHz, conservatively run at 2MHz
SPISettings spiSet = SPISettings(2000000, MSBFIRST, SPI_MODE0);


// Constructor.  We need the crystal speed and the chip select pin number.
MAX3100Serial::MAX3100Serial(uint32_t crystalFrequencykHz, pin_t csPin)
{
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
  read_buf |= SPI.transfer((send_buf >> 8) & 0xFF);
  read_buf = read_buf << 8;
  read_buf |= SPI.transfer(send_buf & 0xFF);
  digitalWrite(_chipSelectPin, HIGH);
  SPI.endTransaction();
  return read_buf;
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
  // Particle: sets pin mode of SCK, MOSI, MISO, SS
  SPI.begin(_chipSelectPin);
  // Write the configuration: /RM (read interrupt mask) and chosen baud.
  // The assumption here is that the Arduino is interested in knowing when
  // serial data arrives, rather than the terminal connected to the MAX3100's
  // USART knowing.
  //SPI.transfer16(MAX3100_CMD_WRITE_CONF | MAX3100_CONF_RM | conf);
  conf = MAX3100_CMD_WRITE_CONF | MAX3100_CONF_RM | conf;
  _transfer16b(conf);
}


int MAX3100Serial::read_conf()
{
  return _transfer16b(MAX3100_CMD_READ_CONF);
}


void MAX3100Serial::end()
{
  // Particle: does NOT reset the pin mode of the SPI pins.
  SPI.end();
}


int MAX3100Serial::read()
{
  int rbuf = 0;
  while ((rbuf & MAX3100_CONF_R) == 0) {
    rbuf = _transfer16b(MAX3100_CMD_READ_DATA);
  }
  return rbuf & 0xFF;
}


int MAX3100Serial::available()
{
  // R flag is set
  return (_transfer16b(MAX3100_CMD_READ_CONF) & MAX3100_CONF_R);
}


int MAX3100Serial::_busy()
{
  // T flag is not set
  return !(_transfer16b(MAX3100_CMD_READ_CONF) & MAX3100_CONF_T);
}


size_t MAX3100Serial::write(uint8_t b)
{
  // There is no buffer.  Wait for the tramsmit register to empty before
  // proceeding, otherwise the character may be lost.
  while (_busy()) {}
  _transfer16b(MAX3100_CMD_WRITE_DATA | b);
  return 1;
}


void MAX3100Serial::flush()
{
  // There is no buffer.  Wait for the transmit register to empty.
  while (_busy()) {}
}


int MAX3100Serial::peek()
{
  // This is not currently implemented in the hardware.  It could be implemented
  // with a one byte software buffer, but this would prevent /RM interrupts from
  // firing.
  return -1;
}
