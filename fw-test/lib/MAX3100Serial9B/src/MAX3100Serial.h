/*
  $Id: MAX3100Serial.h,v 1.6 2018/03/07 12:40:42 ewan Exp $
  Arduino MAX3100Serial library.
  MAX3100Serial.h (C) 2016 Ewan Parker.

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

  https://datasheets.maximintegrated.com/en/ds/MAX3100.pdf


  Assemble the Arduino and MAX3100CPD circuit as follows.  This could be made 
  to work with the other MAX3100 packages with minor changes to the wiring.

Arduino                  MAX3100CPD
=======               ================             CH340/FTDI
MOSI 11 ----------> 1 DIN       Vcc 14 --- 5V      ==========
MISO 12 <---------- 2 DOUT       TX 13 ----------> RX
SCK  13 ----------> 3 SCLK       RX 12 <---------- TX
     ## ----------> 4 /CS      /RTS 11     GND --- GND
      2 <---------- 5 /IRQ     /CTS 10
             5V --- 6 /SHDN      X1  9 ----------- Crystal pin1 --- 22pF --- GND
            GND --- 7 GND        X2  8 ----------- Crystal pin2 --- 22pF --- GND

The Arduino pin used for the chip select signal is chosen in the calling code.
The crystal chosen above is either 1.8432 MHz or 3.6864 MHz and this is also
chosen by the calling code.
*/

#ifndef MAX3100SERIAL_H
#define MAX3100SERIAL_H

#include <Stream.h>

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#define READ_BUF_SIZE 1024  // in uint16_t words
#define WRITE_BUF_SIZE 256  // in uint16_t words
#define ISR_READ_MAX 8  // MAX3100 has 8-word FIFO
#define ISR_WRITE_MAX 8  // to match

class MAX3100Serial : public Stream
{
public:
  // public methods
  MAX3100Serial(uint32_t crystalFrequencykHz, pin_t chipSelectPin, pin_t intPin);
  ~MAX3100Serial();
  void begin(uint32_t speed);
  int readConf();
  int readData();
  void end();
  int peek();
  int availableForWrite(size_t length);

  virtual size_t write(uint8_t byte);
  virtual size_t write9b(uint16_t bytes);
  virtual int read();
  virtual int16_t read9b();
  virtual int available();
  virtual void flush();

  using Print::write;

  int count_irq;
  int count_sent;
  int count_read;
  int count_overflow;


private:
  pin_t _chipSelectPin;
  pin_t _intPin;
  uint16_t _clockMultiplier;
  // circular buffer for reading bytes for responding to interrupts
  uint16_t _read_buf[READ_BUF_SIZE];
  volatile int _read_buf_head;
  volatile int _read_buf_tail;
  // circular buffer for writing bytes too
  uint16_t _write_buf[WRITE_BUF_SIZE];
  volatile int _write_buf_head;
  volatile int _write_buf_tail;

  void _setChipSelectPin(pin_t csPin);
  void _setClockMultiplier(uint32_t kHz);
  inline uint16_t _transfer16b(uint16_t send_buf);
  void _readBufAppend(uint16_t word);
  void _isr();
  int _busy();
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round

#endif
