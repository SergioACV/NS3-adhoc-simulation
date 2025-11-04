#ifndef STORE_CARRY_FORWARD_H
#define STORE_CARRY_FORWARD_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"

#include <queue>
#include <map>
#include <string>

namespace ns3 {

// -------------------
// Estructuras
// -------------------
struct BufferedPacket {
    Ptr<Node> sender;
    Ipv4Address dest;
    Ptr<Packet> packet; 
    uint32_t size;
    std::string msg; 
    double timestamp;
};

// -------------------
// Funciones públicas
// -------------------
void BufferPacket(Ptr<Node> sender, Ipv4Address dest, std::string msg, Ptr<const Packet> pkt);
bool HasBufferedPackets(Ptr<Node> node);
// Devuelve el siguiente paquete de la cola sin eliminarlo
BufferedPacket PeekBufferedPacket(Ptr<Node> node);

// Elimina el paquete que ya fue enviado
void PopBufferedPacket(Ptr<Node> node);

// ===================
// Métodos para el buffer enviado / SC
// ===================
void BufferPacketSent(Ptr<Node> sender, Ipv4Address dest, std::string msg, Ptr<const Packet> pkt);
bool HasBufferedPacketsSent(Ptr<Node> node);
// Devuelve el siguiente paquete de la cola sin eliminarlo
BufferedPacket PeekBufferedPacketSent(Ptr<Node> node);
// Elimina el paquete que ya fue enviado
void PopBufferedPacketSent(Ptr<Node> node);



} // namespace ns3

#endif // STORE_CARRY_FORWARD_H
