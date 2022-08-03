/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/ihartwig/repos/mate-net-monitor/fw-test/src/fw-test.ino"
/*
 * Project fw-test
 * Description: Boron + Mate Net interface test firmware.
 * Author: Ian Hartwig
 * Date: 30 Jul 2022
 */

#include "PacketSerial.h"

// hw config
// mate net pins
HAL_USB_USART_Config acquireUSBSerial1Buffer();
void mateNetOnPacketReceived(
  const void* sender,
  const uint8_t* buffer,
  size_t size);
void setup();
void loop();
#line 12 "/Users/ihartwig/repos/mate-net-monitor/fw-test/src/fw-test.ino"
const pin_t PIN_MATE_RX = RX;
const pin_t PIN_MATE_TX = TX;
const pin_t PIN_MATE_IND = A0;
const unsigned long MATE_BAUD = 9600;
// mag net pins - not used same uart
const pin_t PIN_MAG_RX = D5;
const pin_t PIN_MAG_TX = D4;
const pin_t PIN_MAG_IND = D2;
const pin_t PIN_MAG_EN = D3;
// expansion pins
const pin_t PIN_EXP_ADC1 = A1;
const pin_t PIN_EXP_ADC2 = A2;
const pin_t PIN_EXP_ADC3 = A3;
const pin_t PIN_EXP_ADC4 = A4;
const pin_t PIN_LED = D7;

// sw config
SerialLogHandler logHandler;  // default to Serial on debug USB
PacketSerial mate_net_packet_serial;
const int packet_buf_len = 1500 * 3;  // borrow from ethernet + overhead
char test_packet_buf[packet_buf_len];
int test_packet_len = 0;
char debug_packet_buf[packet_buf_len];
int debug_packet_len = 0;
system_tick_t debug_last_out_ms = 0;
int mate_net_tx_pkt = 0;
int mate_net_rx_pkt = 0;
int mate_net_rx_err = 0;


// allocate large buffers for hw serial for packet flinging
HAL_USB_USART_Config acquireUSBSerial1Buffer()
{
  HAL_USB_USART_Config conf = {0};

  // The usable buffer size will be 128
  static uint8_t usbserial1_rx_buffer[packet_buf_len];
  static uint8_t usbserial1_tx_buffer[packet_buf_len];

  conf.rx_buffer = usbserial1_rx_buffer;
  conf.tx_buffer = usbserial1_tx_buffer;
  conf.rx_buffer_size = packet_buf_len;
  conf.tx_buffer_size = packet_buf_len;

  return conf;
}

void mateNetOnPacketReceived(
  const void* sender,
  const uint8_t* buffer,
  size_t size) {
  mate_net_rx_pkt++;
}

// setup() runs once, when the device is first turned on.
void setup() {
  int ret = 0;
  // mate net pin setup
  pinMode(PIN_MATE_RX, INPUT_PULLDOWN);
  pinMode(PIN_MATE_TX, OUTPUT);
  pinMode(PIN_MATE_IND, OUTPUT);
  analogWrite(PIN_MATE_IND, 127, 5); // 50% 5Hz blink effect
  Serial1.begin(MATE_BAUD, SERIAL_8N1);
  // mag net pin setup
  pinMode(PIN_MAG_RX, INPUT);
  pinMode(PIN_MAG_TX, INPUT);
  pinMode(PIN_MAG_IND, OUTPUT);
  analogWrite(PIN_MAG_IND, 127, 5); // 50% 5Hz blink effect
  pinMode(PIN_MAG_EN, INPUT);
  // expansion pins setup
  pinMode(PIN_EXP_ADC1, INPUT);
  pinMode(PIN_EXP_ADC2, INPUT);
  pinMode(PIN_EXP_ADC3, INPUT);
  pinMode(PIN_EXP_ADC4, INPUT);
  // debug
  delay(1000);  // console reconnect
  Log.info(String::format("%s: %s", Time.timeStr().c_str(), "Hello world!"));
  // packet interfaces
  mate_net_packet_serial.setStream(&Serial1);
  mate_net_packet_serial.setPacketHandler(&mateNetOnPacketReceived);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // Serial1.write("this is a test test\r\n");
  // delay(1000);
  // // debug capture of Serial1
  // while(Serial1.available() && debug_packet_len < packet_buf_len) {
  //   debug_packet_buf[debug_packet_len] = Serial1.read();
  //   debug_packet_len++;
  // }
  // Log.info(String::format("debug_packet_buf: %s", debug_packet_buf));
  // delay(1000);
  //
  // packet exchange
  mate_net_packet_serial.update();
  if(mate_net_packet_serial.overflow()) {
    Log.error("mate_net_packet_serial overflow!");
  }
  if(Serial1.availableForWrite()) {
    digitalWrite(PIN_LED, PinState::HIGH);
    mate_net_packet_serial.send((uint8_t *)"this is a test packet", 22);
    mate_net_tx_pkt++;
    digitalWrite(PIN_LED, PinState::LOW);
  }
  // debug out
  system_tick_t now_ms = millis();
  if(now_ms - debug_last_out_ms >= 5000) {
    Log.info(String::format(
      "mate_net_tx_pkt: %d, mate_net_rx_pkt: %d, mate_net_rx_err: %d",
      mate_net_tx_pkt, mate_net_rx_pkt, mate_net_rx_err
    ));
    debug_last_out_ms = now_ms;
  }
  // cloud out TODO
  // uptime
  // mate net pkt stats
}