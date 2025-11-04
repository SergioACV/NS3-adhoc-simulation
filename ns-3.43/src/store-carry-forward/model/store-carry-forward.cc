#include "store-carry-forward.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"    // Para Ipv4Address
#include "ns3/applications-module.h"

#include <map>
#include <queue>
#include <cmath>

// Definir un tamaño máximo de buffer por nodo
static const uint32_t MAX_BUFFER_SIZE = 50; // por ejemplo 50 paquetes

namespace ns3 {

// -------------------
// Logging
// -------------------
NS_LOG_COMPONENT_DEFINE("StoreCarryForward");

// -------------------
// Variables internas
// -------------------
static std::map<uint32_t, std::queue<BufferedPacket>> g_packetBuffer;



// -------------------
void BufferPacket(Ptr<Node> sender, Ipv4Address dest, uint16_t port, Ptr<const Packet> pkt) {
    BufferedPacket bpkt;
    bpkt.sender = sender;
    bpkt.dest = dest;
    bpkt.port = port;
    bpkt.packet = pkt->Copy();   // copiar el paquete
    bpkt.timestamp = Simulator::Now().GetSeconds();

    auto &queue = g_packetBuffer[sender->GetId()];

    // Si la cola ya está llena, descartar el paquete más antiguo (FIFO)
    if (queue.size() >= MAX_BUFFER_SIZE) {
        NS_LOG_WARN("[" << Simulator::Now().GetSeconds() << "s] Nodo "
                        << sender->GetId() << " buffer lleno, descartando paquete más antiguo");
        queue.pop();
    }

    // Añadir el nuevo paquete
    queue.push(bpkt);

    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Paquete bufferizado por Node "
                    << sender->GetId() << " hacia " << dest
                    << " (tamaño: " << pkt->GetSize() << " bytes), cola actual: " 
                    << queue.size() << ")");
}



// -------------------
bool HasBufferedPackets(Ptr<Node> node)
{
    uint32_t nodeId = node->GetId();

    if (g_packetBuffer.find(nodeId) == g_packetBuffer.end() ||
        g_packetBuffer[nodeId].empty())
    {
        NS_LOG_INFO("[" << Simulator::Now().GetSeconds()
                        << "s] Nodo " << nodeId
                        << " no tiene paquetes en el buffer (cola vacía).");
        return false;
    }

    NS_LOG_INFO("[" << Simulator::Now().GetSeconds()
                    << "s] Nodo " << nodeId
                    << " tiene " << g_packetBuffer[nodeId].size()
                    << " paquetes en el buffer listos para enviar.");
    return true;
}

// -------------------


    BufferedPacket PeekBufferedPacket(Ptr<Node> node) {
        auto it = g_packetBuffer.find(node->GetId());
        if (it != g_packetBuffer.end() && !it->second.empty()) {
            return it->second.front();
        }
        return BufferedPacket(); // paquete vacío si no hay nada
    }

    void PopBufferedPacket(Ptr<Node> node) {
        auto it = g_packetBuffer.find(node->GetId());
        if (it != g_packetBuffer.end() && !it->second.empty()) {
            it->second.pop();
        }
    }


} // namespace ns3
