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

#include "uMate.h"

// hw config
// mate net pins
void setup();
void loop();
#line 12 "/Users/ihartwig/repos/mate-net-monitor/fw-mate-net-monitor/src/fw-mate-net-monitor.ino"
const pin_t PIN_MATE_RX = RX;
const pin_t PIN_MATE_TX = TX;
const pin_t PIN_MATE_IND = A0;
const unsigned long MATE_BAUD = 9600;
const uint32_t MATE_CONFIG =  SERIAL_9N1;
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
// SerialLogHandler logHandler;  // default to Serial on debug USB
// MATEnet bus is connected to Hardware Serial 1 (9-bit communication)
MateControllerProtocol mate_bus(Serial1);
const system_tick_t mate_status_int_ms = 1000;
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
int mate_devices_found;  // bit mask of the mate devices found
int mate_status_mx_cnt_rx;
int mate_status_mx_cnt_err;
char mate_status_mx_hex[MAX_BUFFER_LEN*2];  // encode response in hex chars


void setup() {
    // begin with config for mate_bus
    Serial1.begin(MATE_BAUD, MATE_CONFIG);
    // debug logging to usb serial
    //Log.info(String::format("%s: %s", Time.timeStr().c_str(), "MATE emulator ready"));
    // particle cloud variables
    Particle.variable("mate_devices_found", mate_devices_found);
    Particle.variable("mate_status_mx_cnt_rx", mate_status_mx_cnt_rx);
    Particle.variable("mate_status_mx_cnt_err", mate_status_mx_cnt_err);
    Particle.variable("mate_status_mx_hex", mate_status_mx_hex);
}


void loop() {
    // startup: find attached devices - need at least mx
    if (mate_port_mx == -1) {
        mate_port_hub = mate_bus.find_device(DeviceType::Hub);
        mate_port_fx = mate_bus.find_device(DeviceType::Fx);
        mate_port_mx = mate_bus.find_device(DeviceType::Mx);
        mate_port_flexnetdc = mate_bus.find_device(DeviceType::FlexNetDc);
        mate_port_dc = mate_bus.find_device(DeviceType::Dc);
        // diag for device presence
        int mate_devices_found_tmp = 0;
        mate_devices_found_tmp |= (1 & (mate_port_hub!=-1))<<DeviceType::Hub;
        mate_devices_found_tmp |= (1 & (mate_port_fx!=-1))<<DeviceType::Fx;
        mate_devices_found_tmp |= (1 & (mate_port_mx!=-1))<<DeviceType::Mx;
        mate_devices_found_tmp |= (1 & (mate_port_flexnetdc!=-1))<<DeviceType::FlexNetDc;
        mate_devices_found_tmp |= (1 & (mate_port_dc!=-1))<<DeviceType::Dc;
        // write once for particle cloud
        mate_devices_found = mate_devices_found_tmp;
    }
    // startup successful?
    if (mate_port_mx == -1) {
        delay(1000);
        return;
    }
    // request and report status
    if (mate_bus.available()) {
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
    }
    delay(mate_status_int_ms);

    // static uint16_t nexttick = 0;
    // uint8_t byte0;
    // uint8_t buffer_len = MAX_BUFFER_LEN-2; //MATE_RESP_LEN-1;
    // uint8_t buffer[MAX_BUFFER_LEN];
    
    // uint8_t target_id = 0; // Broadcast

    // Packed representation of MATE response
    // struct {
    //     uint8_t     cmd;
    //     response_t  resp;
    // } resp_data  __attribute__ ((packed));

    // Handle PJON protocol stack
    // uint16_t status = pjon_bus.receive();
    // pjon_bus.update();

    // if (status == PJON_NAK) {
    //     pjon_bus.send_packet(0, "N", 1);
    // }

    // Incoming data from Outback Device
    // if (mate_bus.available()) {
    //     auto err = mate_bus.recv_data(&byte0, &buffer[1], &buffer_len);
    //     if (err == CommsStatus::Success) {
    //         buffer[0] = byte0;

    //         // Forward to USB Serial
    //         // pjon_bus.send_packet(
    //         //     target_id,
    //         //     buffer,
    //         //     buffer_len+1
    //         // );
    //     } else {
    //         if (err != CommsStatus::NoData) {
    //             buffer[0] = (uint8_t)err;
    //             // pjon_bus.send_packet(
    //             //     target_id,
    //             //     buffer, 1
    //             // );
    //         }
    //     }
    // }
}
