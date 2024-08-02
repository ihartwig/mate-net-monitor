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
int mate_bus_scan();
int mate_mx_status ();
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
MateControllerProtocol mate_bus(mate_uart); // connect mate_bus to this Steam9b supporting port
// MateControllerProtocol mate_bus(mate_uart, &Serial);  // debug to USB Serial

// sw config
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);
// event timers - start with negative values for events sooner to startup
const system_tick_t cloud_delay_ms = 30000;
const system_tick_t mate_scan_int_ms = 1000;
const system_tick_t mate_status_int_ms = 180000;
system_tick_t mate_scan_last_ms = -mate_scan_int_ms;
system_tick_t mate_status_last_ms = -cloud_delay_ms;
#define PUB_BUFFER_LEN   (particle::protocol::MAX_EVENT_DATA_LENGTH)
#define MAX_BUFFER_LEN   (100)
#define MATE_PACKET_LEN  (sizeof(packet_t) + 1)
#define MATE_RESP_LEN    (sizeof(response_t) + 1)
#define MATE_RETRIES     3
#define MATE_MX_PRSNT(x) (x & (0x1<<DeviceType::Mx))  // non-zero when MX is present
uint8_t mate_status_buf[STATUS_RESP_SIZE+1];  // size from MateNetPort.h
// mate device ports from scan with -1 not found
int mate_port_hub;
int mate_port_fx;
int mate_port_mx;
int mate_port_flexnetdc;
int mate_port_dc;
int mate_devices_found = 0;  // bit mask of the mate devices found
// particle cloud variables
int mate_status_mx_cnt_rx = 0;
int mate_status_mx_cnt_err = 0;
system_tick_t mate_status_update_ms = 0;
char mate_monitor_stats[PUB_BUFFER_LEN];
char mate_status_mx_hex[(STATUS_RESP_SIZE*2)+1];  // encode response in hex chars


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
  spark::Log.trace(String::format("%s: %s", Time.timeStr().c_str(), "MATE emulator: Hello world!"));
  // particle cloud variables
  Particle.variable("mate_scan_last_ms", mate_scan_last_ms);
  Particle.variable("mate_devices_found", mate_devices_found);
  Particle.variable("mate_status_mx_cnt_rx", mate_status_mx_cnt_rx);
  Particle.variable("mate_status_mx_cnt_err", mate_status_mx_cnt_err);
  Particle.variable("mate_status_mx_hex", mate_status_mx_hex);
  Particle.connect();
  spark::Log.info("MATE emulator: particle cloud ready");
  // digitalWrite(D7, PinState::LOW);
}


// sets a bit-mask of the devices found on bus
// returns retries used until scan success or MATE_RETRIES is exceeded
// retries until an MX Charger is found
int mate_bus_scan() {
  int retries_used = 0;
  mate_devices_found = 0;  // bit mask of the mate devices found
  spark::Log.trace("mate_bus_scan(): looking for mate devices");
  for (; !MATE_MX_PRSNT(mate_devices_found) && retries_used < MATE_RETRIES; retries_used++) {
    mate_bus.scan_ports();
    // mate device ports from scan with -1 not found
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
  }
  spark::Log.trace("mate_bus_scan(): exiting with devices 0x%x after %d retries", mate_devices_found, retries_used);
  return retries_used;
}


// gathers a new MX Charger status and transcribes that to ascii-hex
// returns retries used until new status or MATE_RETRIES is exceeded
int mate_mx_status () {
  int retries_used = 0;
  bool new_status = false;
  spark::Log.trace("mate_mx_status(): looking for MX Charger status");
  for (; !new_status && retries_used < MATE_RETRIES; retries_used++) {
    new_status = mate_bus.read_status(mate_status_buf, STATUS_RESP_SIZE, 1, mate_port_mx);
    // todo: maybe we should timeout for this. apparently there is?
    if (new_status) {
      mate_status_mx_cnt_rx++;
    } else {
      mate_status_mx_cnt_err++;
    }
  }
  // convert to hex string byte of status response at a time
  if (!new_status) {
    // blank out mate_status_buf if there was no new status
    memset((char *)&mate_status_mx_hex, 0, STATUS_RESP_SIZE);
  }
  for (int i = 0; i < STATUS_RESP_SIZE; i++) {
    snprintf(((char *)(&mate_status_mx_hex[i*2])), 3, "%02x", mate_status_buf[i]);
  }
  // null terminator added by last snprintf
  spark::Log.trace("mate_mx_status(): exiting after %d retries", retries_used);
  return retries_used;
}


void loop() {
    int mate_scan_retries = 0;
    int mate_status_retries = 0;
    // skip the rest if we haven't reach an action interval yet
    system_tick_t now_ms = millis();
    if (now_ms - mate_scan_last_ms < mate_scan_int_ms) {
      return;
    }
    mate_scan_last_ms = now_ms;
    // look for devices - this is mostly diagnostic
    mate_status_update_ms = millis();
    mate_scan_retries = mate_bus_scan();
    // and 
    if (mate_devices_found) {
      analogWrite(PIN_MATE_IND, 127, 5); // 50% 5Hz blink effect
    } else {
      digitalWrite(PIN_MAG_IND, PinState::LOW);  // on no blink
    }
    // continue to devices statuses
    if (now_ms - mate_status_last_ms < mate_status_int_ms) {
      return;
    }
    mate_status_last_ms = now_ms;
    // and get MX Charger status, regardless of bus scan result
    // default to looking for devices on port 0 (direct connect)
    if(!MATE_MX_PRSNT(mate_devices_found)) {
      mate_port_mx = 0;
    }
    mate_status_retries = mate_mx_status();
    mate_status_update_ms = millis() - mate_status_update_ms;
    // publish MX Charger status - with or without response
    // approx strlen 233 chars
    snprintf(
      (char *)&mate_monitor_stats,
      PUB_BUFFER_LEN,
      "{"
        "\"uptime_ms\": %ld, "
        "\"mate_devices_found\": \"0x%x\", "
        "\"mate_scan_retries\": %d, "
        "\"mate_status_mx_cnt_rx\": %d, "
        "\"mate_status_mx_cnt_err\": %d, "
        "\"mate_status_mx\": \"0x%s\", "
        "\"mate_status_retries\": %d, "
        "\"mate_status_update_ms\": %ld"
      "}",
      now_ms,
      mate_devices_found, mate_scan_retries,
      mate_status_mx_cnt_rx, mate_status_mx_cnt_err,
      mate_status_mx_hex, mate_status_retries,
      mate_status_update_ms
    );
    spark::Log.info("mate_monitor_stats (len %d):", strlen(mate_monitor_stats));
    spark::Log.print(mate_monitor_stats);  // to print over 200 chars
    spark::Log.print("\n");
    Particle.publish("fw-mate-net-monitor-status", mate_monitor_stats);
    spark::Log.info("Particle.publish(\"fw-mate-net-monitor-status\", ...) done");
    return;
}
