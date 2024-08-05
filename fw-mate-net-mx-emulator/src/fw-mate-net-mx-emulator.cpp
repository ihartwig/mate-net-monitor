/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/ihartwig/repos/mate-net-monitor/fw-mate-net-mx-emulator/src/fw-mate-net-mx-emulator.ino"
/*
 * Project fw-mate-net-monitor
 * Description: represents an MX charger that responds to scan and status requests.
 * Author: Ian Hartwig
 * Date:
 */

#include "MAX3100Serial.h"
#include "uMate.h"

// hw config
// mate net pins
// using a MAX3100 SPI UART instead
void setup();
void loop();
#line 14 "/Users/ihartwig/repos/mate-net-monitor/fw-mate-net-mx-emulator/src/fw-mate-net-mx-emulator.ino"
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
MateDeviceProtocol mate_bus(mate_uart); // connect mate_bus to this Steam9b supporting port
// MateDeviceProtocol mate_bus(mate_uart, &Serial);  // debug to USB Serial

// sw config
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);
// sw buffers
uint8_t port = 0;
packet_t packet = {0, 0, 0};
bool have_packet = false;
response_t response = {0};
uint8_t status_response[STATUS_RESP_SIZE] = {0x0U, 0x1U, 0x2U, 0x3U, 0x4U, 0x5U, 0x6U, 0x7U, 0x8U, 0x0AU, 0xBU, 0xCU, 0xDU};
// event timers - start with negative values for events sooner to startup
const system_tick_t disp_int_ms = 5000;
const system_tick_t dots_int_ms = 1200;
system_tick_t disp_last_ms = -disp_int_ms;
system_tick_t dots_last_ms = -dots_int_ms;

void setup() {
  // mate net pin setup
  pinMode(PIN_MATE_IND, OUTPUT);
  analogWrite(PIN_MATE_IND, 255, 5);  // 0% 5Hz blink effect
  // analogWrite(PIN_MATE_IND, 127, 5);  // 50% 5Hz blink effect
  // .begin sets up PIN_MATE_UART_CS, PIN_MATE_UART_IRQ, and SPI interface
  mate_uart.begin(MATE_UART_BAUD, 1);  // parity enable for 9-bit mode
  // pinSetDriveStrength does not seem to work with analogWrite and Serial1
  // mag net pin setup
  pinMode(PIN_MAG_RX, INPUT);
  pinMode(PIN_MAG_TX, INPUT);
  pinMode(PIN_MAG_IND, OUTPUT);  // open-drain to vcc
  pinMode(PIN_MAG_DE, INPUT_PULLUP);  // activate loopback
  digitalWrite(PIN_MAG_IND, PinState::LOW);  // on no blink
  // analogWrite(PIN_MAG_IND, 127, 5);  // 50% 5Hz blink effect
  Serial1.begin(MAG_UART_BAUD, SERIAL_8N1);
  // mate_bus.set_timeout(200);  // per-msg; default timeout 100ms
  // expansion pins setup
  pinMode(PIN_EXP_ADC1, INPUT);
  pinMode(PIN_EXP_ADC2, INPUT);
  pinMode(PIN_EXP_ADC3, INPUT);
  pinMode(PIN_EXP_ADC4, INPUT);
  // debug logging to usb serial
  delay(2000);  // console reconnect
  // auto logManager = LogManager::instance();
  // auto logTraceHandler = new StreamLogHandler(Serial, LOG_LEVEL_TRACE);
  // logManager->addHandler(logTraceHandler);
  spark::Log.info(String::format("%s: %s", Time.timeStr().c_str(), "MX emulator: Hello world!"));
  // particle cloud variables
  spark::Log.info("MX emulator: particle cloud not used");
  // digitalWrite(D7, PinState::LOW);
}


void loop() {
    system_tick_t now_ms = millis();
    // diagnostic counters
    if (now_ms - disp_last_ms >= disp_int_ms) {
      Serial.print("\n");
      Serial.flush();
      spark::Log.info(String::format(
        "MAX3100Serial: count_irq: %d, count_sent: %d, count_read: %d, count_read_err: %d, count_overflow: %d, available_bytes: %d",
        mate_uart.count_irq, mate_uart.count_sent, mate_uart.count_read, mate_uart.count_read_err, mate_uart.count_overflow, mate_uart.availableBytes()
      ));
      disp_last_ms = now_ms;
    }
    // give feedback that terminal is alive but we haven't received any packets
    if (now_ms - dots_last_ms >= dots_int_ms) {
      // print direct to usb serial so we don't have the header
      Serial.print(". ");
      Serial.flush();
      dots_last_ms = now_ms;
    }
    // try to receive mate packets
    have_packet = mate_bus.recv_packet(&port, &packet);
    if (have_packet) {
      dots_last_ms = now_ms;
      Serial.print("\n");
      Serial.flush();
      spark::Log.info(String::format(
        "recv_packet: {type %d, addr 0x%x, param 0x%x}", packet.type, packet.addr, packet.param
      ));
      // respond to port scan with device type
      if(packet.type == PacketType::Read && packet.addr == 0) {
        response.value = DeviceType::Mx;
        mate_bus.send_response(PacketType::Read, &response);
        spark::Log.info(String::format(
          "send_response: DeviceType %d", response.value
        ));
      }
      // respond to status request with random data
      else if (packet.type == PacketType::Status && packet.addr == 1) {
        mate_bus.send_response(PacketType::Status, (uint8_t*) &status_response, sizeof(status_response));
        spark::Log.info("send_response: {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x0A, 0xB, 0xC, 0xD}");
      }
      // report but skip everything else
    }

    return;
}
