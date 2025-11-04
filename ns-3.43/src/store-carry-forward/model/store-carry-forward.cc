#include "store-carry-forward.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"    // Para Ipv4Address
#include "ns3/applications-module.h"

#include <map>
#include <queue>
#include <cmath>

namespace ns3 {

// -------------------
// Logging
// -------------------
NS_LOG_COMPONENT_DEFINE("StoreCarryForward");

// -------------------
// Variables internas
// -------------------
static 
std::map<uint32_t, std::queue<BufferedPacket>> g_packetBuffer;

// -------------------
void BufferPacket(Ptr<Node> sender, Ipv4Address dest, uint16_t port, uint32_t size) {
    BufferedPacket pkt = {sender, dest, port, size, Simulator::Now().GetSeconds()};
    g_packetBuffer[sender->GetId()].push(pkt);
    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Paquete bufferizado por Node "
                    << sender->GetId() << " hacia " << dest);
}

void SendSensorData(Ptr<Node> from, Ipv4Address toAddr, uint16_t port, Ptr<Packet> pkt)
{
    Ptr<Socket> socket = Socket::CreateSocket(from, UdpSocketFactory::GetTypeId());
    socket->Connect(InetSocketAddress(toAddr, port));
    socket->Send(pkt);
    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Cluster envía paquete a " 
                    << toAddr << ":" << port);
}


// Modifica SendBufferedPackets para aceptar destino
void SendBufferedPackets(Ptr<Node> sender, Ipv4Address dest, uint16_t port)
{
    auto &queue = g_packetBuffer[sender->GetId()];
    while (!queue.empty()) {
        BufferedPacket bpkt = queue.front();
        queue.pop();

        // Crear paquete vacío del tamaño original
        Ptr<Packet> pkt = Create<Packet>(bpkt.size);

        // Enviar al destino con puerto especificado
        SendSensorData(sender, dest, port, pkt);

        NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Paquete bufferizado enviado de Node "
                        << sender->GetId() << " a " << dest << ":" << port);
    }
}





} // namespace ns3
