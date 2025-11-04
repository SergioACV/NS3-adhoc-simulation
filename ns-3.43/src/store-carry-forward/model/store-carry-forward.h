#ifndef STORE_CARRY_FORWARD_H
#define STORE_CARRY_FORWARD_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"

#include <queue>
#include <map>

namespace ns3 {

// -------------------
// Estructuras
// -------------------
struct BufferedPacket {
    Ptr<Node> sender;
    Ipv4Address dest;
    uint16_t port;
    uint32_t size;
    double timestamp;
};

// -------------------
// Funciones públicas
// -------------------
void BufferPacket(Ptr<Node> sender, Ipv4Address dest, uint16_t port, uint32_t size);
void SendBufferedPackets(Ptr<Node> sender, Ipv4Address dest, uint16_t port);
void SendSensorData(Ptr<Node> from, Ipv4Address toAddr, uint16_t port, Ptr<Packet> pkt);

} // namespace ns3

#endif // STORE_CARRY_FORWARD_H
