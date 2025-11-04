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
static std::map<uint32_t, std::queue<BufferedPacket>> g_packetBuffer;       // buffer normal por nodo
static std::map<uint32_t, std::queue<BufferedPacket>> g_packetBufferSent;   // buffer para paquetes enviados o por reenviar

static const uint32_t MAX_BUFFER_SIZE = 50; // tamaño máximo del buffer

// ===================
// Métodos para el buffer normal
// ===================
void BufferPacket(Ptr<Node> sender, Ipv4Address dest, std::string msg, Ptr<const Packet> pkt) {
    BufferedPacket bpkt;
    bpkt.sender = sender;
    bpkt.dest = dest;
    bpkt.msg = msg;
    bpkt.packet = pkt->Copy();
    bpkt.timestamp = Simulator::Now().GetSeconds();

    auto &queue = g_packetBuffer[sender->GetId()];

    if (queue.size() >= MAX_BUFFER_SIZE) {
        NS_LOG_WARN("[" << Simulator::Now().GetSeconds() << "s] Nodo "
                        << sender->GetId() << " buffer lleno, descartando paquete más antiguo");
        queue.pop();
    }

    queue.push(bpkt);

    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Paquete bufferizado por Node "
                    << sender->GetId() << " hacia " << dest
                    << " (tamaño: " << pkt->GetSize() << " bytes), cola actual: " 
                    << queue.size() << ")");
}

bool HasBufferedPackets(Ptr<Node> node) {
    uint32_t nodeId = node->GetId();
    return g_packetBuffer.find(nodeId) != g_packetBuffer.end() && !g_packetBuffer[nodeId].empty();
}

BufferedPacket PeekBufferedPacket(Ptr<Node> node) {
    auto it = g_packetBuffer.find(node->GetId());
    if (it != g_packetBuffer.end() && !it->second.empty()) {
        return it->second.front();
    }
    return BufferedPacket();
}

void PopBufferedPacket(Ptr<Node> node) {
    auto it = g_packetBuffer.find(node->GetId());
    if (it != g_packetBuffer.end() && !it->second.empty()) {
        it->second.pop();
    }
}

// ===================
// Métodos para el buffer enviado / SC
// ===================
void BufferPacketSent(Ptr<Node> sender, Ipv4Address dest, std::string msg, Ptr<const Packet> pkt) {
    BufferedPacket bpkt;
    bpkt.sender = sender;
    bpkt.dest = dest;
    bpkt.msg = msg;
    bpkt.packet = pkt->Copy();
    bpkt.timestamp = Simulator::Now().GetSeconds();

    auto &queue = g_packetBufferSent[sender->GetId()];

    if (queue.size() >= MAX_BUFFER_SIZE) {
        NS_LOG_WARN("[" << Simulator::Now().GetSeconds() << "s] Nodo "
                        << sender->GetId() << " buffer enviado lleno, descartando paquete más antiguo");
        queue.pop();
    }

    queue.push(bpkt);

    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Paquete enviado bufferizado por Node "
                    << sender->GetId() << " hacia " << dest
                    << " (tamaño: " << pkt->GetSize() << " bytes), cola actual: " 
                    << queue.size() << ")");
}

bool HasBufferedPacketsSent(Ptr<Node> node) {
    uint32_t nodeId = node->GetId();
    return g_packetBufferSent.find(nodeId) != g_packetBufferSent.end() && !g_packetBufferSent[nodeId].empty();
}

BufferedPacket PeekBufferedPacketSent(Ptr<Node> node) {
    auto it = g_packetBufferSent.find(node->GetId());
    if (it != g_packetBufferSent.end() && !it->second.empty()) {
        return it->second.front();
    }
    return BufferedPacket();
}

void PopBufferedPacketSent(Ptr<Node> node) {
    auto it = g_packetBufferSent.find(node->GetId());
    if (it != g_packetBufferSent.end() && !it->second.empty()) {
        it->second.pop();
    }
}

} // namespace ns3
