/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/ihartwig/repos/mate-net-monitor/fw-mate-net-monitor/src/fw-mate-net-monitor.ino"
/*
 * Project fw-mate-net-monitor
 * Description: represents a MATE controller asking for status.
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
#line 14 "/Users/ihartwig/repos/mate-net-monitor/fw-mate-net-monitor/src/fw-mate-net-monitor.ino"
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
MateControllerProtocol mate_bus(mate_uart);  // MATE system connected to UART1

// sw config
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);
const system_tick_t pub_status_int_ms = 30000;
system_tick_t pub_status_last_ms = 0;
const system_tick_t mate_status_int_ms = 5000;
system_tick_t mate_status_last_ms = 0;
#define PUB_BUFFER_LEN (particle::protocol::MAX_EVENT_DATA_LENGTH)
#define MAX_BUFFER_LEN  (100)
#define MATE_PACKET_LEN (sizeof(packet_t) + 1)
#define MATE_RESP_LEN   (sizeof(response_t) + 1)
uint8_t mate_status_buf[STATUS_RESP_SIZE*2];  // based on MAX_PACKET_LEN?
// mate device ports from scan with -1 not found
int mate_port_hub = -1;
int mate_port_fx = -1;
int mate_port_mx = -1;
int mate_port_flexnetdc = -1;
int mate_port_dc = -1;
// particle cloud variables
int mate_devices_found = 0;  // bit mask of the mate devices found
int mate_status_mx_cnt_rx = 0;
int mate_status_mx_cnt_err = 0;
system_tick_t mate_status_update_ms = 0;
system_tick_t cloud_update_ms = 0;
char mate_monitor_stats[PUB_BUFFER_LEN];
char mate_status_mx_hex[STATUS_RESP_SIZE*2];  // encode response in hex chars


void setup() {
  // mate net pin setup
  pinMode(PIN_MATE_IND, OUTPUT);
  analogWrite(PIN_MATE_IND, 255, 5);  // 0% 5Hz blink effect
  // analogWrite(PIN_MATE_IND, 127, 5);  // 50% 5Hz blink effect
  // .begin sets up PIN_MATE_UART_CS, PIN_MATE_UART_IRQ, and SPI interface
  mate_uart.begin(MATE_UART_BAUD);
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
  // debug logging to usb serial
  delay(2000);  // console reconnect
  spark::Log.info(String::format("%s: %s", Time.timeStr().c_str(), "MATE emulator: Hello world!"));
  // particle cloud variables
  // Particle.variable("mate_monitor_uptime_ms", mate_monitor_uptime_ms);
  // Particle.variable("mate_devices_found", mate_devices_found);
  // Particle.variable("mate_status_mx_cnt_rx", mate_status_mx_cnt_rx);
  // Particle.variable("mate_status_mx_cnt_err", mate_status_mx_cnt_err);
  // Particle.variable("mate_status_mx_hex", mate_status_mx_hex);
  Particle.connect();
  spark::Log.info("MATE emulator: particle cloud ready");
  digitalWrite(D7, PinState::LOW);
}


void loop() {
    // startup: find attached devices - need at least mx
    if (mate_port_mx == -1) {
        digitalWrite(PIN_MATE_IND, PinState::HIGH);
        mate_port_hub = mate_bus.find_device(DeviceType::Hub);
        mate_port_fx = mate_bus.find_device(DeviceType::Fx);
        mate_port_mx = mate_bus.find_device(DeviceType::Mx);
        mate_port_flexnetdc = mate_bus.find_device(DeviceType::FlexNetDc);
        mate_port_dc = mate_bus.find_device(DeviceType::Dc);
        // diag for device presence
        mate_devices_found = 0;
        mate_devices_found |= (1 & (mate_port_hub!=-1))<<DeviceType::Hub;
        mate_devices_found |= (1 & (mate_port_fx!=-1))<<DeviceType::Fx;
        mate_devices_found |= (1 & (mate_port_mx!=-1))<<DeviceType::Mx;
        mate_devices_found |= (1 & (mate_port_flexnetdc!=-1))<<DeviceType::FlexNetDc;
        mate_devices_found |= (1 & (mate_port_dc!=-1))<<DeviceType::Dc;
        // local diag
        spark::Log.info(String::format("uptime_ms: %ld, mate_devices_found: 0x%x", millis(), mate_devices_found));
        // rescan timeout
        delay(1000);
    }
    // startup successful? no
    if (mate_port_mx == -1) {
        digitalWrite(PIN_MATE_IND, PinState::HIGH);
    }
    // startup successful? yes
    else {
        analogWrite(PIN_MATE_IND, 127, 5); // 50% 5Hz blink effect
    }
    // skip the rest if we haven't reach report interval yet
    system_tick_t now_ms = millis();
    if(now_ms - mate_status_last_ms < mate_status_int_ms) {
        return;
    }
    // request and report status
    if (mate_bus.available() && mate_port_mx != -1) {
        mate_status_update_ms = millis();
        bool new_status = mate_bus.read_status(mate_status_buf, STATUS_RESP_SIZE, 1, mate_port_mx);
        // todo: maybe we should timeout for this. apparently there is?
        if (new_status) {
            mate_status_mx_cnt_rx++;
        } else {
            mate_status_mx_cnt_err++;
        }
        // convert to hex string
        // snprintf((char *)&mate_status_mx_hex, MAX_BUFFER_LEN*2, "dummy status %d", 0);
        if (new_status) {
            for (int i = 0; i < STATUS_RESP_SIZE; i++) {
                snprintf(((char *)&mate_status_mx_hex)+(i*2), 3, "%02x", mate_status_buf[i]);
            }
            // null terminator added by last snprintf
        }
        mate_status_update_ms = millis() - mate_status_update_ms;
    } else {
        mate_status_update_ms = 0;
    }
    // local status updates
    snprintf(
        (char *)&mate_monitor_stats,
        PUB_BUFFER_LEN,
        "{\"uptime_ms\": %ld, \"mate_devices_found\": 0x%x, \"mate_status_mx_cnt_rx\": %d, \"mate_status_mx_cnt_err\": %d, \"mate_status_update_ms\": %ld, \"cloud_update_ms\": %ld",
        now_ms, mate_devices_found, mate_status_mx_cnt_rx, mate_status_mx_cnt_err, mate_status_update_ms, cloud_update_ms);
    spark::Log.info(mate_monitor_stats);
    mate_status_last_ms = now_ms;
    // (usually slower) cloud status updates
    if(now_ms - pub_status_last_ms >= pub_status_int_ms) {
        cloud_update_ms = millis();
        Particle.publish("mate_monitor_stats", (char *)&mate_monitor_stats);
        Particle.publish("mate_status_mx_hex", (char *)&mate_status_mx_hex);
        cloud_update_ms = millis() - cloud_update_ms;
        pub_status_last_ms = now_ms;
    }
    return;
}
