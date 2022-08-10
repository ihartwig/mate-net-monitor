/*
 * Project fw-mate-net-monitor
 * Description:
 * Author:
 * Date:
 */

#include "uMate.h"

// hw config
// mate net pins
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
#define DEVICE_ID (2)
#define MAX_BUFFER_LEN  (100)
#define MATE_PACKET_LEN (sizeof(packet_t) + 1)
#define MATE_RESP_LEN   (sizeof(response_t) + 1)
// MATEnet bus is connected to Hardware Serial 1 (9-bit communication)
MateControllerProtocol mate_bus_a(Serial1);


// setup() runs once, when the device is first turned on.
void setup() {
    // begin with config for mate_bus_a
    Serial1.begin(MATE_BAUD, MATE_CONFIG);
}


// loop() runs over and over again, as quickly as it can execute.
void loop() {
    static uint16_t nexttick = 0;
    uint8_t byte0;
    uint8_t buffer_len = MAX_BUFFER_LEN-2; //MATE_RESP_LEN-1;
    uint8_t buffer[MAX_BUFFER_LEN];
    
    uint8_t target_id = 0; // Broadcast

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
    if (mate_bus_a.available()) {
        auto err = mate_bus_a.recv_data(&byte0, &buffer[1], &buffer_len);
        if (err == CommsStatus::Success) {
            buffer[0] = byte0;

            // Forward to USB Serial
            // pjon_bus.send_packet(
            //     target_id,
            //     buffer,
            //     buffer_len+1
            // );
        } else {
            if (err != CommsStatus::NoData) {
                buffer[0] = (uint8_t)err;
                // pjon_bus.send_packet(
                //     target_id,
                //     buffer, 1
                // );
            }
        }
    }
}