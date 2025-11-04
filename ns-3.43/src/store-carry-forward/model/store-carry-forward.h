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
    Ptr<Packet> packet; 
    uint32_t size;
    double timestamp;
};

// -------------------
// Funciones públicas
// -------------------
void BufferPacket(Ptr<Node> sender, Ipv4Address dest, uint16_t port, Ptr<const Packet> pkt);
bool HasBufferedPackets(Ptr<Node> node);
// Devuelve el siguiente paquete de la cola sin eliminarlo
BufferedPacket PeekBufferedPacket(Ptr<Node> node);

// Elimina el paquete que ya fue enviado
void PopBufferedPacket(Ptr<Node> node);


} // namespace ns3

#endif // STORE_CARRY_FORWARD_H
