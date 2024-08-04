#include "MateNetPort.h"
#include "MateDeviceProtocol.h"

// Receives a command from an attached MATE
bool MateDeviceProtocol::recv_packet(OUT uint8_t* port, OUT packet_t* packet)
{
    if (port == nullptr || packet == nullptr)
        return false;

    // Packets from a MATE are always sizeof(packet_t)
    uint8_t len = sizeof(packet_t);
    auto err = recv_data(port, reinterpret_cast<uint8_t*>(packet), &len);
    if ((err == CommsStatus::Success) && (len == sizeof(packet_t))) {
        // AVR is little-endian, but protocol is big-endian. Must swap bytes...
        packet->addr = SWAPENDIAN_16(packet->addr);
        packet->param = SWAPENDIAN_16(packet->param);
        return true;
    }
    return false;
}

// Sends a response back to an attached MATE
void MateDeviceProtocol::send_response(PacketType type, response_t* response)
{
    if (response == nullptr)
        return;

    response->value = SWAPENDIAN_16(response->value);
    send_data(type, reinterpret_cast<uint8_t*>(response), sizeof(response_t));
}

// Sends a response back to an attached MATE
// when the response is a buffer
void MateDeviceProtocol::send_response(PacketType type, uint8_t* data, uint8_t len)
{
    if (data == nullptr || len == 0)
        return;
    
    send_data(type, data, len);
}
