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
const unsigned long MAG_UART_BAUD = 19200;
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
const system_tick_t mate_scan_int_ms = 2000;  // also used for status updates
const system_tick_t cloud_update_int_ms = 30000;
const system_tick_t mag_status_old_ms =  1000;
system_tick_t mate_scan_last_ms = -mate_scan_int_ms;
system_tick_t cloud_update_last_ms = -cloud_update_int_ms;
system_tick_t mag_status_last_ms = -mag_status_old_ms;
// data lengths
#define PUB_BUFFER_LEN   (particle::protocol::MAX_EVENT_DATA_LENGTH)
#define MAX_BUFFER_LEN   (100)
#define MATE_PACKET_LEN  (sizeof(packet_t) + 1)
#define MATE_RESP_LEN    (sizeof(response_t) + 1)
#define MATE_RETRIES     9
#define MATE_MX_PRSNT(x) (x & (0x1<<DeviceType::Mx))  // non-zero when MX is present
#define MAGNET_INV_PACKET_LEN 21
#define MAGNET_INV_START_TIMEOUT_MS 35 // < 100-14-8-10-10-10ms gap for reset
#define MAGNET_BYTE_TIMEOUT_MS 7 // ~10ms gap between msg types
#define MAGNET_NC_TIMEOUT_MS 125
#define MAGNET_INV_MODEL 0x73 // MS4448PAE
// mate device ports numbers from scan - default to -1 not found
int mate_port_hub = -1;
int mate_port_fx = -1;
int mate_port_mx = -1;
int mate_port_flexnetdc = -1;
int mate_port_dc = -1;
int mate_devices_found = 0;  // bit mask of the mate devices found
// mag net Inverter Data
int magnet_ok_cnt; // this might roll over, but we don't care
int magnet_bad_cnt;
int magnet_nc_cnt;
int magnet_dat_cnt;
int magnet_q_cnt;
typedef struct MagNetInvMsg {
// Name              Byte   Comments
  uint8_t inv_status;   // 0      See MagInvStatus
  uint8_t inv_fault;    // 1      See MagInvFault
  uint16_t dc_volts;     // 2,3    1 count = 0.1V (real range = 0-100.0V)
  uint16_t dc_amps;      // 4,5    0-500 amps DC
  uint8_t ac_volts_out; // 6      0-150 Volts ac RMS MS Inverter rev 3.0 and later
  uint8_t ac_volts_in;  // 7      0-255 Volts ac peak MS Inverter rev 3.0 and later
  uint8_t inv_led;      // 8      IF = 0, then INV LED is off else INV Led on remote ON
  uint8_t chg_led;      // 9      IF = 0, then CHG LED is off else CHG Led on remote ON*/
  uint8_t inv_rev;      // 10     i.e. 10 = 1.0 Non-zero is mandatory for remote startup
  uint8_t batt_temp;    // 11     0-150 = 0-150 deg C
  uint8_t tran_temp;    // 12     0-150 = 0-150 deg C
  uint8_t fet_temp;     // 13     0-150 = 0-150 deg C*/
  uint8_t inv_model;    // 14     See MagInvModel
  // uint8_t blank;        // 15
  // 21 bit version
  uint8_t stack_mode;   // 15     See MagInvStackMode
  uint8_t ac_amps_in;   // 16     1 count = 1 Amp AC
  uint8_t ac_amps_out;  // 17     1 count = 1 Amp AC
  uint16_t ac_hz;       // 18,19  1 count = 0.1Hz
  uint8_t blank;        // 20
} MagNetInvMsg;
MagNetInvMsg new_inv_msg;
MagNetInvMsg last_inv_msg;
// particle cloud variables
uint8_t mate_status_buf[STATUS_RESP_SIZE+1];  // size from MateNetPort.h
int mate_status_mx_cnt_rx = 0;
int mate_status_mx_cnt_err = 0;
system_tick_t mate_status_last_ms = 0;  // timestamp of the last status update
char mate_monitor_stats[PUB_BUFFER_LEN];
char mate_status_mx_hex[(STATUS_RESP_SIZE*2)+1];  // encode response in hex chars
char mate_status_mx[PUB_BUFFER_LEN];
char mag_status_inv[PUB_BUFFER_LEN];


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
  // particle cloud
  Particle.connect();
  spark::Log.info("MATE emulator: particle cloud ready");
  // digitalWrite(D7, PinState::LOW);
}


// sets a bit-mask of the devices found on bus
// returns retries used until scan success or MATE_RETRIES is exceeded
// retries until any other device on the bus is found or -1
int mate_bus_scan() {
  int retries_used = -1;
  mate_devices_found = 0;  // bit mask of the mate devices found
  spark::Log.trace("mate_bus_scan(): looking for mate devices");
  while ((!mate_devices_found) && ++retries_used < MATE_RETRIES) {
    int available_bytes = mate_uart.availableBytes();
    if (available_bytes) {
      spark::Log.trace("mate_bus_scan(): purging read buf");
    }
    mate_uart.readBufPurge();
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


// unpack the 13 byte (STATUS_RESP_SIZE) MX status response
// reverse engineering from
// https://github.com/jorticus/pymate/blob/master/pymate/matenet/mx.py
void mate_mx_status_unpack(uint8_t *mate_status_buf, char *mx_status_buf, system_tick_t update_ms) {
  uint8_t *values = mate_status_buf;
  float bat_current_milli = (float)(values[0] & 0x0F) / 10.0;
  snprintf(
    mx_status_buf,
    PUB_BUFFER_LEN,
    "{"
      "\"uptime_ms\": %ld, "
      "\"mx_today_ah\": \"%d\", "
      "\"mx_today_kwh\": %.1f, "
      "\"mx_bat_cur_a\": %.1f, "
      "\"mx_pv_cur_a\": %.1f, "
      "\"mx_aux_mode_state\": \"0x%x\", "
      "\"mx_status\": \"0x%x\", "
      "\"mx_errors\": \"0x%x\", "
      "\"mx_bat_v\": %.1f, "
      "\"mx_pv_v\": %.1f"
    "}",
    update_ms,
    // The following was determined by poking values at the MATE unit...
    ((values[0] & 0x70) >> 4) | values[4],  // bat Ah - ignore bit7 (if 0, MATE hides the AH reading)
    (float)((values[3] << 8) | values[8]) / 10.0, //  kWh
    (float)((128 + (int8_t)values[2]) % 256) + bat_current_milli,  // bat current A
    (float)((128 + (int8_t)values[1]) % 256),  // pv current A
    values[5],  // aux mode (& 0x3F) state (& 0x40 == 0x40)
    values[6],  // status bitfield
    values[7],  // error bitfield
    (float)SWAPENDIAN_16(*(uint16_t *)(&values[9])) / 10.0,  // bat v in V
    (float)SWAPENDIAN_16(*(uint16_t *)(&values[11])) / 10.0  // pv v in V
  );
}


// gathers a new MX Charger status and transcribes that to ascii-hex
// when one is on a known port set by mate_port_mx.
// returns retries used until new status or MATE_RETRIES is exceeded or -1
int mate_mx_status () {
  int retries_used = -1;
  bool new_status = false;
  spark::Log.trace("mate_mx_status(): looking for MX Charger status");
  while (mate_port_mx != -1 && !new_status && ++retries_used < MATE_RETRIES) {
    int available_bytes = mate_uart.availableBytes();
    if (available_bytes) {
      spark::Log.trace("mate_mx_status(): purging read buf");
    }
    mate_uart.readBufPurge();
    new_status = mate_bus.read_status(mate_status_buf, STATUS_RESP_SIZE, 1, mate_port_mx);
    // todo: maybe we should timeout for this. apparently there is?
    if (new_status) {
      mate_status_mx_cnt_rx++;
      mate_status_last_ms = millis();
    } else {
      mate_status_mx_cnt_err++;
    }
  }
  // convert to hex string byte of status response at a time
  if (!new_status) {
    // blank out mate_status_buf if there was no new status
    memset((char *)&mate_status_buf, 0, STATUS_RESP_SIZE);
  }
  for (int i = 0; i < STATUS_RESP_SIZE; i++) {
    snprintf(((char *)(&mate_status_mx_hex[i*2])), 3, "%02x", mate_status_buf[i]);
  }  // null terminator added by last snprintf
  // convert status response to json 
  mate_mx_status_unpack(mate_status_buf, mate_status_mx, mate_status_last_ms);
  spark::Log.trace("mate_mx_status(): exiting after %d retries", retries_used);
  return retries_used;
}


/*
 * Find and read an inverter message from Serial1 to buf. Waits for
 * MAGNET_INV_START_TIMEOUT_MS but not more than MAGNET_NC_TIMEOUT_MS so as to
 * not block the main loop.
 *
 * Returns: 0 on complete message, -1 incomplete buffer, -2 nc timeout
 */
int magnet_read_inv_msg(char* buf, int buf_len) {
  int buf_i = 0;
  int new_byte = 0;
  bool inv_sync = false;
  volatile unsigned long new_byte_time = 0;
  volatile unsigned long last_byte_time = millis();
  while(true) {
    // read byte
    if(Serial1.available()) {
      new_byte = Serial1.read();
    }
    else {
      new_byte = -1;
    }
    // handle new byte
    new_byte_time = millis();
    if(new_byte >= 0) {
      // wait for long gap before inverter talks
      if(new_byte_time > last_byte_time + MAGNET_INV_START_TIMEOUT_MS) {
        inv_sync = true;
        // also need to reset the byte time since we may have been looking a while
        last_byte_time = new_byte_time;
      }
      // and just keep spinning if we haven't found it
      if(!inv_sync) {
        continue;
      }
      // too much time between bytes means another dev is talking now; abort
      else if (new_byte_time > last_byte_time + MAGNET_BYTE_TIMEOUT_MS) {
        return -1; // dropped packet
      }
      // record the new byte
      buf[buf_i] = new_byte;
      buf_i++;
      last_byte_time = new_byte_time;
      // check if we have a complete packet
      if(buf_i == MAGNET_INV_PACKET_LEN) {
        return 0; // filled packet
      }
      else if (buf_i >= MAGNET_INV_PACKET_LEN) {
        return -3; // oops! stop the overflow
      }
      // else keep looking for bytes
    }
    // let the loop move forward if we're not finding anything
    else if (new_byte_time > last_byte_time + MAGNET_NC_TIMEOUT_MS) {
      return -2;
    }
    // else: keep trying to read
    // break;
  } // end while
  return -3;
}


int magnet_interp_inv_msg(char* buf, char* inv_status, int buf_len) {
  MagNetInvMsg inv_msg;
  // fix endianness for 2-byte fields
  uint8_t swap;
  swap = buf[2];
  buf[2] = buf[3];
  buf[3] = swap;
  swap = buf[4];
  buf[4] = buf[5];
  buf[5] = swap;
  swap = buf[18];
  buf[18] = buf[19];
  buf[19] = swap;
  // shove the inverter packet into struct
  memcpy(&inv_msg, buf, sizeof(inv_msg));

  // sanity check this packet data
  if(inv_msg.inv_model != MAGNET_INV_MODEL) {
    return -5;
  }

  // convert to json string format
  snprintf(
    inv_status,
    PUB_BUFFER_LEN,
    "{"
      "\"inv_status\": %d, "
      "\"dc_volts\": %d, "
      "\"dc_amps\": %d, "
      "\"ac_volts_out\": %d, "
      "\"ac_volts_in\": %d, "
      "\"batt_temp\": %d, "
      "\"tran_temp\": %d, "
      "\"fet_temp\": %d, "
      "\"ac_amps_in\": %d, "
      "\"ac_amps_out\": %d, "
      "\"ac_hz\": %d"
    "}",
    inv_msg.inv_status,
    inv_msg.dc_volts,
    inv_msg.dc_amps,
    inv_msg.ac_volts_out,
    inv_msg.ac_volts_in,
    inv_msg.batt_temp,
    inv_msg.tran_temp,
    inv_msg.fet_temp,
    inv_msg.ac_amps_in,
    inv_msg.ac_amps_out,
    inv_msg.ac_hz);
  return 0;
}


void loop() {
  // first check for mag device updates since these stream in all the time
  char mag_inv_packet[MAGNET_INV_PACKET_LEN];
  memset(mag_inv_packet, 0, MAGNET_INV_PACKET_LEN);
  int magnet_ret = -6;
  magnet_ret = magnet_read_inv_msg(mag_inv_packet, MAGNET_INV_PACKET_LEN);
  if (magnet_ret == 0) {
    magnet_interp_inv_msg(mag_status_inv, mag_inv_packet, MAGNET_INV_PACKET_LEN);
    mag_status_last_ms = millis();
    analogWrite(PIN_MAG_IND, 127, 5);  // 50% 5Hz blink effect
  }
  else {
    digitalWrite(PIN_MAG_IND, PinState::LOW);  // on no blink
  }
  // then request mate updates on action interval
  int mate_scan_retries = 0;
  int mate_status_retries = 0;
  // skip the rest if we haven't reach an action interval yet
  system_tick_t now_ms = millis();
  if (now_ms - mate_scan_last_ms < mate_scan_int_ms) {
    return;
  }
  mate_scan_last_ms = now_ms;
  // look for devices - this is mostly diagnostic
  // sets mate_devices_found and mate_port_mx
  mate_scan_retries = mate_bus_scan();
  if (mate_devices_found) {
    analogWrite(PIN_MATE_IND, 127, 5); // 50% 5Hz blink effect
  } else {
    digitalWrite(PIN_MATE_IND, PinState::HIGH);  // on no blink
  }
  // continue to get MX Charger status if we see one is connected
  // uses mate_port_mx and sets mate_scan_retries, mate_status_last_ms, etc
  mate_status_retries = mate_mx_status();
  // publish MX Charger status - with or without response
  // approx strlen 236~300 chars
  snprintf(
    mate_monitor_stats,
    PUB_BUFFER_LEN,
    "{"
      "\"uptime_ms\": %ld, "
      "\"mate_devices_found\": \"0x%x\", "
      "\"mate_scan_retries\": %d, "
      "\"mate_status_mx_cnt_rx\": %d, "
      "\"mate_status_mx_cnt_err\": %d, "
      "\"mate_status_mx\": \"0x%s\", "
      "\"mate_status_retries\": %d, "
      "\"mate_status_last_ms\": %ld, "
      "\"magnet_ok_cnt\": %d, "
      "\"magnet_bad_cnt:\" %d, "
      "\"magnet_nc_cnt:\" %d "
      "\"magnet_dat_cnt:\" %d, "
      "\"magnet_q_cnt:\" %d"
    "}",
    millis(),
    mate_devices_found, mate_scan_retries,
    mate_status_mx_cnt_rx, mate_status_mx_cnt_err,
    mate_status_mx_hex, mate_status_retries,
    mate_status_last_ms,
    magnet_ok_cnt, magnet_bad_cnt, magnet_nc_cnt, magnet_dat_cnt, magnet_q_cnt
  );
  spark::Log.info("mate_monitor_stats (len %d):", strlen(mate_monitor_stats));
  spark::Log.print(mate_monitor_stats);  // to print over 200 chars
  spark::Log.print("\n");
  spark::Log.info("mate_status_mx (len %d):", strlen(mate_status_mx));
  spark::Log.print(mate_status_mx);  // to print over 200 chars
  spark::Log.print("\n");
  spark::Log.info("mag_status_inv (len %d):", strlen(mag_status_inv));
  spark::Log.print(mag_status_inv);  // to print over 200 chars
  spark::Log.print("\n");
  // cloud update at a slower rate
  if (now_ms - cloud_update_last_ms < cloud_update_int_ms) {
    Particle.publish("fw-mate-net-monitor-status", mate_monitor_stats);
    spark::Log.info("Particle.publish(\"fw-mate-net-monitor-status\", ...) done");
    // publish mate status if we got a successful update
    if (mate_status_retries >= 0) {
      Particle.publish("mate-status-mx", mate_status_mx);
      spark::Log.info("Particle.publish(\"mate-status-mx\", ...) done");
    }
    // publish mag status if we got an update within timeout
    if (now_ms - mag_status_last_ms < mag_status_old_ms) {
      Particle.publish("mag-status-inv", mag_status_inv);
      spark::Log.info("Particle.publish(\"mag-status-inv\", ...) done");
    }
  }
  return;
}
