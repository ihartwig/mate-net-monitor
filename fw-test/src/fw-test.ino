/*
 * Project fw-test
 * Description: Boron + Mate Net interface test firmware.
 * Author: Ian Hartwig
 * Date: 30 Jul 2022
 */

#include "MAX3100Serial.h"
#include "PacketSerial.h"

#include <SPI.h>

// hw config
// mate net pins
// using a MAX3100 SPI UART instead
const pin_t PIN_MATE_UART_CS = A5;
const pin_t PIN_MATE_UART_IRQ = D6;
const pin_t PIN_MATE_IND = A0;
const unsigned long MATE_UART_BAUD = 9600;
const int MATE_UART_XTAL_FREQ_KHZ = 3686;
// mag net pins - not used same uart
// these pin are connected to TX/RX by rework; don't use
const pin_t PIN_MAG_RX = D5;
const pin_t PIN_MAG_TX = D4;
const pin_t PIN_MAG_IND = D2;
const pin_t PIN_MAG_DE = D3;
const unsigned long MAG_UART_BAUD = 9600;
// expansion pins
const pin_t PIN_EXP_ADC1 = A1;
const pin_t PIN_EXP_ADC2 = A2;
const pin_t PIN_EXP_ADC3 = A3;
const pin_t PIN_EXP_ADC4 = A4;
const pin_t PIN_LED = D7;

// uarts config
SerialLogHandler logHandler;  // default to Serial on debug USB
MAX3100Serial mate_uart = MAX3100Serial(MATE_UART_XTAL_FREQ_KHZ, PIN_MATE_UART_CS, PIN_MATE_UART_IRQ);

// sw config
SYSTEM_MODE(SEMI_AUTOMATIC);

// test serial buffer config
typedef PacketSerial_<COBS, 0, 256> PacketSerial;
PacketSerial mate_net_packet_serial;
PacketSerial mag_net_packet_serial;
// const int packet_buf_len = 1500 * 3;  // borrow from ethernet + overhead
const int packet_buf_len = 256;
// test packet from prbs_generate.js prbs11
// char test_packet_buf[] = \
//   "\x80\x50\x22\x15\x48\x0d\x07\x23\x75\xd4\x50\xa2\x45\x6a\x18\x4f\x2e\x72\xf7\x25\x76\x15\xc8\x5d\x25\x36\x3d\xd9\x57\x81\x30\xbe\x48\xed\x6b\x18\xef\x6a\x58\x67\x3f\x78\x53\x23\xf5\x84\x72\xb7\x0d\x67\x1f\x6c\x5b\xa6\xa7\x87\x33\x7f\xd0\x12\x0b\x44\xca\xfc\x21\x94\xf8\xe3\x6d\xdb\x56\xc1\xb8\xeb\x68\xd9\x77\x95\x38\x3b\x1a\xee\x2a\xd0\x32\x1f\x4c\x4f\xae\x22\xd5\x30\x3e\x18\xcf\x7e\x50\xe2\x6d\x7b\x12\xeb\x28\xf1\x66\x9f\x9c\x3d\x99\x7f\x90\x3a\x1a\x4e\x6e\xfa\xa2\x05\x42\x09\x45\x8a\x74\x74\xb4\xcc\xff\xe0\x0c\x07\x83\x31\xfe\xc0\xb8\x4b\x2c\xf3\xe7\x8f\x36\x7d\xf1\x46\x8b\x94\xb8\xcb\x7c\xd1\xf2\xc7\x3b\x7a\xd2\x33\x5f\xc4\x1a\x8e\x16\xc9\xbd\xe9\x49\x8d\xf7\x45\x4a\x0c\x47\xab\x20\xf4\x64\xbe\xc8\xbd\x49\x0d\xa7\x67\x5f\x44\x4a\xac\x03\x81\xb0\xee\x6a\xf8\x23\x15\xe8\x49\x2d\xb3\x6f\xda\x16\x49\xed\xcb\x5c\xc5\xfa\x42\x69\x79\x93\xfb\x82\xb1\x0e\xa6\x87\x93\x3b\xfa\x82\x11\x4a\x8c\x17\x89\x35\xbc\x69\xb9\xeb\xc8\x9d\x5d\x05\x22\x35\x5c\x05\x82\x71\x76\x95\x98\x7f\x30\x7e\x30\xde\x74\xf4\xe4\xee\xea\xa8\x00"\
//   "\x80\x50\x22\x15\x48\x0d\x07\x23\x75\xd4\x50\xa2\x45\x6a\x18\x4f\x2e\x72\xf7\x25\x76\x15\xc8\x5d\x25\x36\x3d\xd9\x57\x81\x30\xbe\x48\xed\x6b\x18\xef\x6a\x58\x67\x3f\x78\x53\x23\xf5\x84\x72\xb7\x0d\x67\x1f\x6c\x5b\xa6\xa7\x87\x33\x7f\xd0\x12\x0b\x44\xca\xfc\x21\x94\xf8\xe3\x6d\xdb\x56\xc1\xb8\xeb\x68\xd9\x77\x95\x38\x3b\x1a\xee\x2a\xd0\x32\x1f\x4c\x4f\xae\x22\xd5\x30\x3e\x18\xcf\x7e\x50\xe2\x6d\x7b\x12\xeb\x28\xf1\x66\x9f\x9c\x3d\x99\x7f\x90\x3a\x1a\x4e\x6e\xfa\xa2\x05\x42\x09\x45\x8a\x74\x74\xb4\xcc\xff\xe0\x0c\x07\x83\x31\xfe\xc0\xb8\x4b\x2c\xf3\xe7\x8f\x36\x7d\xf1\x46\x8b\x94\xb8\xcb\x7c\xd1\xf2\xc7\x3b\x7a\xd2\x33\x5f\xc4\x1a\x8e\x16\xc9\xbd\xe9\x49\x8d\xf7\x45\x4a\x0c\x47\xab\x20\xf4\x64\xbe\xc8\xbd\x49\x0d\xa7\x67\x5f\x44\x4a\xac\x03\x81\xb0\xee\x6a\xf8\x23\x15\xe8\x49\x2d\xb3\x6f\xda\x16\x49\xed\xcb\x5c\xc5\xfa\x42\x69\x79\x93\xfb\x82\xb1\x0e\xa6\x87\x93\x3b\xfa\x82\x11\x4a\x8c\x17\x89\x35\xbc\x69\xb9\xeb\xc8\x9d\x5d\x05\x22\x35\x5c\x05\x82\x71\x76\x95\x98\x7f\x30\x7e\x30\xde\x74\xf4\xe4\xee\xea\xa8\x00"\
//   "\x80\x50\x22\x15\x48\x0d\x07\x23\x75\xd4\x50\xa2\x45\x6a\x18\x4f\x2e\x72\xf7\x25\x76\x15\xc8\x5d\x25\x36\x3d\xd9\x57\x81\x30\xbe\x48\xed\x6b\x18\xef\x6a\x58\x67\x3f\x78\x53\x23\xf5\x84\x72\xb7\x0d\x67\x1f\x6c\x5b\xa6\xa7\x87\x33\x7f\xd0\x12\x0b\x44\xca\xfc\x21\x94\xf8\xe3\x6d\xdb\x56\xc1\xb8\xeb\x68\xd9\x77\x95\x38\x3b\x1a\xee\x2a\xd0\x32\x1f\x4c\x4f\xae\x22\xd5\x30\x3e\x18\xcf\x7e\x50\xe2\x6d\x7b\x12\xeb\x28\xf1\x66\x9f\x9c\x3d\x99\x7f\x90\x3a\x1a\x4e\x6e\xfa\xa2\x05\x42\x09\x45\x8a\x74\x74\xb4\xcc\xff\xe0\x0c\x07\x83\x31\xfe\xc0\xb8\x4b\x2c\xf3\xe7\x8f\x36\x7d\xf1\x46\x8b\x94\xb8\xcb\x7c\xd1\xf2\xc7\x3b\x7a\xd2\x33\x5f\xc4\x1a\x8e\x16\xc9\xbd\xe9\x49\x8d\xf7\x45\x4a\x0c\x47\xab\x20\xf4\x64\xbe\xc8\xbd\x49\x0d\xa7\x67\x5f\x44\x4a\xac\x03\x81\xb0\xee\x6a\xf8\x23\x15\xe8\x49\x2d\xb3\x6f\xda\x16\x49\xed\xcb\x5c\xc5\xfa\x42\x69\x79\x93\xfb\x82\xb1\x0e\xa6\x87\x93\x3b\xfa\x82\x11\x4a\x8c\x17\x89\x35\xbc\x69\xb9\xeb\xc8\x9d\x5d\x05\x22\x35\x5c\x05\x82\x71\x76\x95\x98\x7f\x30\x7e\x30\xde\x74\xf4\xe4\xee\xea\xa8\x00"\
//   "\x80\x50\x22\x15\x48\x0d\x07\x23\x75\xd4\x50\xa2\x45\x6a\x18\x4f\x2e\x72\xf7\x25\x76\x15\xc8\x5d\x25\x36\x3d\xd9\x57\x81\x30\xbe\x48\xed\x6b\x18\xef\x6a\x58\x67\x3f\x78\x53\x23\xf5\x84\x72\xb7\x0d\x67\x1f\x6c\x5b\xa6\xa7\x87\x33\x7f\xd0\x12\x0b\x44\xca\xfc\x21\x94\xf8\xe3\x6d\xdb\x56\xc1\xb8\xeb\x68\xd9\x77\x95\x38\x3b\x1a\xee\x2a\xd0\x32\x1f\x4c\x4f\xae\x22\xd5\x30\x3e\x18\xcf\x7e\x50\xe2\x6d\x7b\x12\xeb\x28\xf1\x66\x9f\x9c\x3d\x99\x7f\x90\x3a\x1a\x4e\x6e\xfa\xa2\x05\x42\x09\x45\x8a\x74\x74\xb4\xcc\xff\xe0\x0c\x07\x83\x31\xfe\xc0\xb8\x4b\x2c\xf3\xe7\x8f\x36\x7d\xf1\x46\x8b\x94\xb8\xcb\x7c\xd1\xf2\xc7\x3b\x7a\xd2\x33\x5f\xc4\x1a\x8e\x16\xc9\xbd\xe9\x49\x8d\xf7\x45\x4a\x0c\x47\xab\x20\xf4\x64\xbe\xc8\xbd\x49\x0d\xa7\x67\x5f\x44\x4a\xac\x03\x81\xb0\xee\x6a\xf8\x23\x15\xe8\x49\x2d\xb3\x6f\xda\x16\x49\xed\xcb\x5c\xc5\xfa\x42\x69\x79\x93\xfb\x82\xb1\x0e\xa6\x87\x93\x3b\xfa\x82\x11\x4a\x8c\x17\x89\x35\xbc\x69\xb9\xeb\xc8\x9d\x5d\x05\x22\x35\x5c\x05\x82\x71\x76\x95\x98\x7f\x30\x7e\x30\xde\x74\xf4\xe4\xee\xea\xa8\x00"\
//   "\x80\x50\x22\x15\x48\x0d\x07\x23\x75\xd4\x50\xa2\x45\x6a\x18\x4f\x2e\x72\xf7\x25\x76\x15\xc8\x5d\x25\x36\x3d\xd9\x57\x81\x30\xbe\x48\xed\x6b\x18\xef\x6a\x58\x67\x3f\x78\x53\x23\xf5\x84\x72\xb7\x0d\x67\x1f\x6c\x5b\xa6\xa7\x87\x33\x7f\xd0\x12\x0b\x44\xca\xfc\x21\x94\xf8\xe3\x6d\xdb\x56\xc1\xb8\xeb\x68\xd9\x77\x95\x38\x3b\x1a\xee\x2a\xd0\x32\x1f\x4c\x4f\xae\x22\xd5\x30\x3e\x18\xcf\x7e\x50\xe2\x6d\x7b\x12\xeb\x28\xf1\x66\x9f\x9c\x3d\x99\x7f\x90\x3a\x1a\x4e\x6e\xfa\xa2\x05\x42\x09\x45\x8a\x74\x74\xb4\xcc\xff\xe0\x0c\x07\x83\x31\xfe\xc0\xb8\x4b\x2c\xf3\xe7\x8f\x36\x7d\xf1\x46\x8b\x94\xb8\xcb\x7c\xd1\xf2\xc7\x3b\x7a\xd2\x33\x5f\xc4\x1a\x8e\x16\xc9\xbd\xe9\x49\x8d\xf7\x45\x4a\x0c\x47\xab\x20\xf4\x64\xbe\xc8\xbd\x49\x0d\xa7\x67\x5f\x44\x4a\xac\x03\x81\xb0\xee\x6a\xf8\x23\x15\xe8\x49\x2d\xb3\x6f\xda\x16\x49\xed\xcb\x5c\xc5\xfa\x42\x69\x79\x93\xfb\x82\xb1\x0e\xa6\x87\x93\x3b\xfa\x82\x11\x4a\x8c\x17\x89\x35\xbc\x69\xb9\xeb\xc8\x9d\x5d\x05\x22\x35\x5c\x05\x82\x71\x76\x95\x98\x7f\x30\x7e\x30\xde\x74\xf4\xe4\xee\xea\xa8\x00";
// const int test_packet_len = (256*5)+1;
// test packet from prbs_generate.js prbs7
char test_packet_buf[] = \
  "\x83\x0a\x3c\x8b\x3a\x9f\x43\x89\x36\xb7\xb1\xa5\xdc\xca\xbf\x80"\
  "\x83\x0a\x3c\x8b\x3a\x9f\x43\x89\x36\xb7\xb1\xa5\xdc\xca\xbf\x80"\
  "\x83\x0a\x3c\x8b\x3a\x9f\x43\x89\x36\xb7\xb1\xa5\xdc\xca\xbf\x80"\
  "\x83\x0a\x3c\x8b\x3a\x9f\x43\x89\x36\xb7\xb1\xa5\xdc\xca\xbf\x80";
const int test_packet_len = 64;
// char test_packet_buf[] = \"Lorem ipsum dolor sit amet, consectetur adipiscing elit.";
// const int test_packet_len = 57;
// "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Pellentesque eu aliquam quam, ac hendrerit eros. Vivamus vulputate vulputate eleifend. Ut ac blandit risus, sed suscipit felis. Praesent in leo eget massa tincidunt maximus. Integer tincidunt eros lorem, ac accumsan felis feugiat non. Praesent eu risus sed risus vestibulum dictum nec a magna. Donec pulvinar metus non lacus dignissim, id mattis odio iaculis. Donec vulputate, erat id porta sollicitudin, mi lorem vestibulum urna, vel dapibus mi justo tincidunt leo. Nam in sagittis metus,"\
// " ac fringilla urna. Aenean posuere mi urna, in scelerisque tellus fringilla a. Aliquam ac dignissim dolor. Aenean imperdiet eros eget arcu tincidunt accumsan. Maecenas tincidunt odio sit amet lectus sagittis commodo. Aenean rhoncus magna imperdiet pulvinar elementum."\
// "Aliquam consequat euismod lectus et dapibus. Vestibulum auctor est in justo tincidunt, at venenatis tortor placerat. Etiam at lacus non risus convallis ornare. Suspendisse eget euismod ex. Quisque quis purus et sapien interdum aliquet ut et urna. Ut dapibus nunc mollis massa ultricies, non pellentesque ex pretium. Integer sed enim volutpat, venenatis augue at, blandit justo. Maecenas ut ligula quis ex elementum maximus. Fusce hendrerit diam dui. Praesent diam odio, dignissim eget faucibus in, finibus eget enim. Vivamus et vestibulum augue."\
// "Suspendisse efficitur est sit amet urna fermentum vulputate. Aenean at mollis dolor, quis congue arcu. Aliquam ultrices luctus elit, id non.";
// const int test_packet_len = 1499;
char debug_packet_buf[packet_buf_len];
int debug_packet_len = 0;
system_tick_t debug_last_out_ms1 = 0;
system_tick_t debug_last_out_ms2 = 0;
system_tick_t debug_last_out_ms3 = 0;
int mate_net_tx_pkt = 0;
int mate_net_rx_pkt = 0;
int mate_net_rx_err = 0;
int debug_mate_count_read = 0;
int mag_net_tx_pkt = 0;
int mag_net_rx_pkt = 0;
int mag_net_rx_err = 0;
int debug_mag_count_read = 0;


void mateNetOnPacketReceived(
  const void* sender,
  const uint8_t* buffer,
  size_t size) {
  // check packet size 
  if (size != test_packet_len) {
    mate_net_rx_err++;
    Log.error(String::format("mate_net: bad len %d", size));
    return;
  }
  // check packet content
  for(int i=0; i<test_packet_len; i++) {
    if(test_packet_buf[i] != buffer[i]) {
      mate_net_rx_err++;
      Log.error(String::format("mate_net: bad char at %d: 0x%x expecting 0x%x", i, buffer[i], test_packet_buf[i]));
      return;
    }
  }
  // ok
  // Log.info(String::format("%s (%d)", buffer, size));
  mate_net_rx_pkt++;
}


void magNetOnPacketReceived(
  const void* sender,
  const uint8_t* buffer,
  size_t size) {
  // check packet size 
  if (size != test_packet_len) {
    mag_net_rx_err++;
    Log.error(String::format("mag_net: bad len %d", size));
    return;
  }
  // check packet content
  for(int i=0; i<test_packet_len; i++) {
    if(test_packet_buf[i] != buffer[i]) {
      mag_net_rx_err++;
      Log.error(String::format("mag_net: bad char at %d: 0x%x expecting 0x%x", i, buffer[i], test_packet_buf[i]));
      return;
    }
  }
  // ok
  // Log.info(String::format("%s (%d)", buffer, size));
  mag_net_rx_pkt++;  
}


// setup() runs once, when the device is first turned on.
void setup() {
  // mate net pin setup
  pinMode(PIN_MATE_IND, OUTPUT);
  analogWrite(PIN_MATE_IND, 255, 5);  // 0% 5Hz blink effect
  // analogWrite(PIN_MATE_IND, 127, 5);  // 50% 5Hz blink effect
  // .begin sets up PIN_MATE_UART_CS, PIN_MATE_UART_IRQ, and SPI interface
  mate_uart.begin(MATE_UART_BAUD, 1);
  // pinSetDriveStrength does not seem to work with analogWrite and Serial1
  // mag net pin setup
  pinMode(PIN_MAG_RX, INPUT);
  pinMode(PIN_MAG_TX, INPUT);
  pinMode(PIN_MAG_IND, OUTPUT);  // open-drain to vcc
  pinMode(PIN_MAG_DE, INPUT_PULLUP);  // activate loopback
  digitalWrite(PIN_MAG_IND, PinState::LOW);  // on no blink
  // analogWrite(PIN_MAG_IND, 127, 5);  // 50% 5Hz blink effect
  Serial1.begin(MAG_UART_BAUD, SERIAL_8N1);
  // expansion pins setup
  pinMode(PIN_EXP_ADC1, INPUT);
  pinMode(PIN_EXP_ADC2, INPUT);
  pinMode(PIN_EXP_ADC3, INPUT);
  pinMode(PIN_EXP_ADC4, INPUT);
  // debug
  delay(2000);  // console reconnect
  Log.info(String::format("%s: %s", Time.timeStr().c_str(), "Hello world!"));
  // packet interfaces
  mate_net_packet_serial.setStream(&mate_uart);
  mate_net_packet_serial.setPacketHandler(&mateNetOnPacketReceived);
  mag_net_packet_serial.setStream(&Serial1);
  mag_net_packet_serial.setPacketHandler(&magNetOnPacketReceived);
}


// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // analogWrite(PIN_MATE_IND, 127, 5);
  // Log.info("System.freeMemory: %lu", System.freeMemory());
  // Log.info("MAX3100 Conf: 0x%x, IRQ: %d, Sent: %d, Read: %d, Overflow: %d", mate_uart.readConf(), mate_uart.count_irq, mate_uart.count_sent, mate_uart.count_read, mate_uart.count_overflow);
  // Log.info("MAX3100 IRQ: %d", digitalRead(PIN_MATE_UART_IRQ));
  // Log.info("1234567\r\n");
  // mate_uart.write("1234567\r\n");
  // delay(100);
  // // debug capture of mate_uart
  // debug_packet_len = 0;
  // while(mate_uart.available() && debug_packet_len < packet_buf_len) {
  //   debug_packet_buf[debug_packet_len] = mate_uart.read();
  //   debug_packet_len++;
  // }
  // debug_packet_buf[debug_packet_len] = '\0';
  // Log.info("debug_packet_buf[0:%d]: %s", debug_packet_len, debug_packet_buf);
  // Log.info("debug_packet_buf[0]: 0x%x", debug_packet_buf[0]);
  // Log.info("debug_packet_buf[1]: 0x%x", debug_packet_buf[1]);
  // Log.info("debug_packet_buf[2]: 0x%x", debug_packet_buf[2]);
  // Log.info("debug_packet_buf[3]: 0x%x", debug_packet_buf[3]);
  // Log.info("debug_packet_buf[4]: 0x%x", debug_packet_buf[4]);
  // Log.info("debug_packet_buf[5]: 0x%x", debug_packet_buf[5]);
  // Log.info("debug_packet_buf[6]: 0x%x", debug_packet_buf[6]);
  // Log.info("debug_packet_buf[7]: 0x%x", debug_packet_buf[7]);
  // Log.info("debug_packet_buf[8]: 0x%x", debug_packet_buf[8]);
  // Log.info("debug_packet_buf[9]: 0x%x", debug_packet_buf[9]);
  // delay(1000);
  // return;

  // packet exchange
  mate_net_packet_serial.update();
  if(mate_net_packet_serial.overflow()) {
    Log.error("mate_net_packet_serial overflow!");
  }
  if(mate_uart.availableForWrite(test_packet_len)) {
    mate_net_packet_serial.send((uint8_t *)(&test_packet_buf), test_packet_len);
    mate_net_tx_pkt++;
  }
  mag_net_packet_serial.update();
  if(mag_net_packet_serial.overflow()) {
    Log.error("mag_net_packet_serial overflow!");
  }
  if(Serial1.availableForWrite()) {
    mag_net_packet_serial.send((uint8_t *)(&test_packet_buf), test_packet_len);
    mag_net_tx_pkt++;
  }

  // raw block exchange
  // if(mate_uart.availableForWrite(test_packet_len)) {
  //   mate_uart.write((char *)(&test_packet_buf));
  //   mate_net_tx_pkt++;
  // }
  // mate_uart.flush();
  // delay(100);
  // debug_packet_len = 0;
  // while(mate_uart.available() && debug_packet_len < packet_buf_len) {
  //   debug_packet_buf[debug_packet_len] = mate_uart.read();
  //   debug_packet_len++;
  // }
  // mateNetOnPacketReceived(NULL, (uint8_t *)(&debug_packet_buf), debug_packet_len);

  // debug out
  system_tick_t now_ms = millis() + 300000; // trigger messages at startup
  if (now_ms - debug_last_out_ms1 >= 500) {
    // leds
    if (mate_uart.count_read > debug_mate_count_read) {
      analogWrite(PIN_MATE_IND, 127, 5);
    } else {
      analogWrite(PIN_MATE_IND, 255, 5);  // 0% 5Hz blink effect
    }
    debug_mate_count_read = mate_uart.count_read;
    if (mag_net_tx_pkt > debug_mag_count_read) {
      analogWrite(PIN_MAG_IND, 127, 5);
    } else {
      digitalWrite(PIN_MAG_IND, PinState::LOW);
    }
    debug_mag_count_read = mag_net_tx_pkt;
    debug_last_out_ms1 = now_ms;
  }
  if (now_ms - debug_last_out_ms2 >= 5000) {
    // text summary
    Log.info("System.freeMemory: %lu", System.freeMemory());
    Log.info(
      "MAX3100 IRQ: %d, Sent: %d, Read: %d, Read Err: %d, Overflow: %d",
      mate_uart.count_irq, mate_uart.count_sent, mate_uart.count_read, mate_uart.count_read_err, mate_uart.count_overflow
    );
    Log.info(
      "mate_net_tx_pkt: %d, mate_net_rx_pkt: %d, mate_net_rx_err: %d",
      mate_net_tx_pkt, mate_net_rx_pkt, mate_net_rx_err
    );
    Log.info(
      "mag_net_tx_pkt: %d, mag_net_rx_pkt: %d, mag_net_rx_err: %d",
      mag_net_tx_pkt, mag_net_rx_pkt, mag_net_rx_err
    );
    debug_last_out_ms2 = now_ms;
  }
  // connect to particle cloud - can block for up to 1s
  if (Particle.connected() == false) {
    Particle.connect();
  }
  // comment out above to run offline only
  if (now_ms - debug_last_out_ms3 >= 300000 && Particle.connected() == true) {
    // cloud out - 2 devs publishing every 5 min fits in free tier
    Particle.publish("fw-test-status", String::format(
      "{"
        "\"mate_net_tx_pkt\": %d, \"mate_net_rx_pkt\": %d, \"mate_net_rx_err\": %d,"
        "\"mag_net_tx_pkt\": %d, \"mag_net_rx_pkt\": %d, \"mag_net_rx_err\": %d"
      "}",
      mate_net_tx_pkt, mate_net_rx_pkt, mate_net_rx_err,
      mag_net_tx_pkt, mag_net_rx_pkt, mag_net_rx_err
    ));
    Log.info("Particle.publish(\"fw-test-status\", ...) done");
    debug_last_out_ms3 = now_ms;
  }
}
