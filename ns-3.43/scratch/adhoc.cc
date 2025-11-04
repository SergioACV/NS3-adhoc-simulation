#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/ssid.h"
#include "ns3/wifi-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/netanim-module.h"
#include "ns3/interference-helper.h"
#include "ns3/random-variable-stream.h" // REQUERIDO PARA ALARMA ALEATORIA

#include "ns3/store-carry-forward.h"


//Self-made modules
#include "ns3/multi-zone-propagation-loss-model-helper.h"

#include <cmath>
#include <vector>
#include <map>     // REQUERIDO PARA ESTADO DEL DRON
#include <limits>  // REQUERIDO PARA ESTADO DEL DRON

using namespace ns3;

// ---------------------------
// 🔹 Globales
// ---------------------------
static Ptr<MultiZonePropagationLossModel> g_multiZoneModel = 0;
static AnimationInterface* g_anim = nullptr; // Puntero global de NetAnim
static Ptr<UniformRandomVariable> g_rand;    // Generador aleatorio para alarmas
// Mapea cada Cluster Head a sus nodos
static std::map<uint32_t, NodeContainer> g_clusterNodesMap;

Ipv4Address g_superAddr; 


// Mapa para rastrear el "próximo evento de patrulla" de cada dron.
static std::map< Ptr<ConstantVelocityMobilityModel>, EventId > g_droneNextPatrolEvent;

// 🔹 CORRECCIÓN DE COMPILACIÓN: Era std::map, no std.map
static std::map< Ptr<ConstantVelocityMobilityModel>, uint32_t > g_dronePatrolIndex;


NS_LOG_COMPONENT_DEFINE("ManetRecolector");

// ---------------------------
// 🔹 Declaración de funciones
// ---------------------------
Ptr<YansWifiChannel> CreateMultiZoneChannel();
void ConfigureWifiAdhoc(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy);
void InstallWifiDevices(WifiHelper &wifi, WifiMacHelper &wifiMac, WifiPhyHelper &wifiPhy,
                        NodeContainer &sensors, NodeContainer &clusterHeads,
                        NodeContainer &superCluster, NodeContainer &recolectors,
                        NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                        NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices,
                        bool useLeaderSignalPower); // [NUEVO]

void ConfigureClusterMobility(NodeContainer &sensors, Vector A, Vector B, Vector C);
void ConfigureClusterHeadsAndSuper(NodeContainer &clusterHeads, NodeContainer &superCluster, MobilityHelper &mobility,
                                   Vector A, Vector B, Vector C, Vector centroid);
// Nota: ahora recibe A,B,C para distribuir inicios
std::vector<Ptr<ConstantVelocityMobilityModel>> ConfigureRecolectorMobilities(NodeContainer &recolectors, MobilityHelper &mobility, Vector A, Vector B, Vector C); // [MODIFICADA]

void InstallAodvAndStack(NodeContainer &sensors, NodeContainer &clusterHeads,
                         NodeContainer &superCluster, NodeContainer &recolectors);

Ipv4InterfaceContainer AssignIpAddresses(Ipv4AddressHelper &address,
                                         NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                                         NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices);

void SendSensorData(Ptr<Node> from, Ipv4Address toAddr, uint16_t port,
                    Ipv4Address finalDest, const std::string &msg);
void SendUploadData(Ptr<Node> from, Ipv4Address toAddr);
void SendForwardMessage(Ptr<Node> from, Ipv4Address toAddr);
void SendStatusEvent(Ptr<Node> from, Ipv4Address toAddr, std::string msg);

void InstallUdpServers(NodeContainer &sensors, NodeContainer &recolectors, NodeContainer &superCluster, NodeContainer &clusterHeads, double simTime);
// Función para instalar un socket "listener" en cada ClusterHead
void InstallClusterHeadReceivers(NodeContainer &clusterHeads, uint16_t listenPort = 5000);
void SetupIntraClusterClients(NodeContainer &sensors,NodeContainer &clusterHeads,Ipv4InterfaceContainer &interfaces);
// ScheduleRecolectorMovement ahora recibe simEnd para repetir hasta el final
void ScheduleRecolectorMovement(Ptr<ConstantVelocityMobilityModel> mv,
                                Vector A, Vector B, Vector C, double speed, double simEnd, Vector centroid);

void RunSimulation(double simTime);

void PeriodicProximityCheck(NodeContainer &clusterHeads,NodeContainer &recolectors,double checkInterval);
static void ReturnToSuperCallback(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simTime, double patrolSpeed);
static void SetupReturnToSuper(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simTime, double leadTime = 5.0, double patrolSpeed = 5.0);

double Dist2D(const Vector &a, const Vector &b);
void SchedulePeriodicProximityCheck(NodeContainer &clusterHeads,NodeContainer &recolector,double checkInterval = 1.0);

// -----------------------------------------------------------------
// 🔹 LÓGICA DE MOVIMIENTO Y ALARMA (SIMULADA) 🔹
// -----------------------------------------------------------------

// --- Declaración de funciones de la nueva lógica ---

double SetVelocityTowards(Ptr<ConstantVelocityMobilityModel> mv, Vector to, double speed);
void PatrolWaypointReached(Ptr<ConstantVelocityMobilityModel> mv, 
                           std::vector<Vector> route, double speed);
void StartPatrolling(Ptr<ConstantVelocityMobilityModel> mv, 
                     std::vector<Vector> route, double speed);
void ResumePatrolAfterAlarm(Ptr<ConstantVelocityMobilityModel> mv, Ptr<Node> droneNode, 
                            Ptr<Node> alarmNode, std::vector<Vector> route, double speed);
void TriggerRandomAlarm(NodeContainer sensors, NodeContainer recolector, 
                        std::vector<Ptr<ConstantVelocityMobilityModel>> recolectorMVs,
                        std::vector<Vector> route, double patrolSpeed, double alarmSpeed, double simTime);
std::pair<Ptr<ConstantVelocityMobilityModel>, Ptr<Node>> 
FindClosestDrone(Vector targetPos, NodeContainer& recolector, std::vector<Ptr<ConstantVelocityMobilityModel>>& recolectorMVs);

// --- Implementación de funciones de la nueva lógica ---

double SetVelocityTowards(Ptr<ConstantVelocityMobilityModel> mv, Vector to, double speed)
{
    Vector from = mv->GetPosition();
    Vector dir = to - from;
    double dist = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);

    if (dist < 1e-6)
    {
        mv->SetVelocity(Vector(0,0,0));
        return 0.0;
    }

    Vector vel = Vector(dir.x / dist * speed, dir.y / dist * speed, dir.z / dist * speed);
    mv->SetVelocity(vel);
    
    return dist / speed; // Devuelve el tiempo del trayecto
}

void PatrolWaypointReached(Ptr<ConstantVelocityMobilityModel> mv, 
                           std::vector<Vector> route, double speed)
{
    // 1. Averiguar dónde estamos y a dónde vamos
    uint32_t currentIndex = g_dronePatrolIndex[mv];
    uint32_t nextIndex = (currentIndex + 1) % route.size();
    
    // 2. Establecer la velocidad para el *siguiente* tramo
    double timeToNext = SetVelocityTowards(mv, route[nextIndex], speed);
    
    // 3. Almacenar el *nuevo* destino
    g_dronePatrolIndex[mv] = nextIndex;

    // 4. Programar esta *misma* función para que se llame al llegar al *siguiente* punto
    EventId nextEvent = Simulator::Schedule(Seconds(timeToNext), 
                                           &PatrolWaypointReached, 
                                           mv, route, speed);
    
    // 5. Almacenar el ID del evento para poder cancelarlo si hay una alarma
    g_droneNextPatrolEvent[mv] = nextEvent;
}

void StartPatrolling(Ptr<ConstantVelocityMobilityModel> mv, 
                     std::vector<Vector> route, double speed)
{
    // 1. Encontrar el punto de patrulla (A, B, o C) más cercano al punto de inicio del dron
    Vector startPos = mv->GetPosition();
    double minDist = std::numeric_limits<double>::max();
    uint32_t closestIndex = 0;

    for (uint32_t i = 0; i < route.size(); ++i)
    {
        Vector dir = route[i] - startPos;
        double dist = dir.x * dir.x + dir.y * dir.y; // sqr dist
        if (dist < minDist)
        {
            minDist = dist;
            closestIndex = i;
        }
    }

    // 2. Establecer la velocidad hacia ese primer punto
    double timeToFirst = SetVelocityTowards(mv, route[closestIndex], speed);

    // 3. Almacenar ese punto como nuestro destino actual
    g_dronePatrolIndex[mv] = closestIndex;

    // 4. Programar el bucle de patrulla para que comience al *llegar* a ese primer punto
    EventId firstEvent = Simulator::Schedule(Seconds(timeToFirst), 
                                             &PatrolWaypointReached, 
                                             mv, route, speed);

    g_droneNextPatrolEvent[mv] = firstEvent;
}

void ResumePatrolAfterAlarm(Ptr<ConstantVelocityMobilityModel> mv, Ptr<Node> droneNode, 
                            Ptr<Node> alarmNode, std::vector<Vector> route, double speed)
{
    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: Dron " << droneNode->GetId() 
                << " llegó a la alarma en Nodo " << alarmNode->GetId() << ". Reanudando patrulla.");

    // 1. Resetear colores en NetAnim
    if (g_anim)
    {
        // Resetea el dron a su color original (azul)
        g_anim->UpdateNodeColor(droneNode, 0, 0, 255);
        // Resetea el sensor a su color original (azul)
        g_anim->UpdateNodeColor(alarmNode, 0, 0, 255); 
    }

    // 2. Recuperar el destino de patrulla que tenía *antes* de la alarma
    uint32_t targetIndex = g_dronePatrolIndex[mv];
    Vector targetPatrolPos = route[targetIndex];

    // 3. Establecer velocidad hacia ese destino de patrulla
    double timeToPatrol = SetVelocityTowards(mv, targetPatrolPos, speed);
    
    // 4. Programar el bucle de patrulla para que se active al llegar
    EventId nextEvent = Simulator::Schedule(Seconds(timeToPatrol), 
                                           &PatrolWaypointReached, 
                                           mv, route, speed);
    
    g_droneNextPatrolEvent[mv] = nextEvent;
}

std::pair<Ptr<ConstantVelocityMobilityModel>, Ptr<Node>> 
FindClosestDrone(Vector targetPos, NodeContainer& recolector, std::vector<Ptr<ConstantVelocityMobilityModel>>& recolectorMVs)
{
    double minDist = std::numeric_limits<double>::max();
    Ptr<ConstantVelocityMobilityModel> closestMv = nullptr;
    Ptr<Node> closestNode = nullptr;

    for (uint32_t i = 0; i < recolectorMVs.size(); ++i)
    {
        Vector pos = recolectorMVs[i]->GetPosition();
        Vector dir = targetPos - pos;
        double dist = dir.x * dir.x + dir.y * dir.y; // sqr dist
        
        if (dist < minDist)
        {
            minDist = dist;
            closestMv = recolectorMVs[i];
            closestNode = recolector.Get(i);
        }
    }
    return {closestMv, closestNode};
}


void TriggerRandomAlarm(NodeContainer sensors, NodeContainer recolector, 
                        std::vector<Ptr<ConstantVelocityMobilityModel>> recolectorMVs,
                        std::vector<Vector> route, double patrolSpeed, double alarmSpeed, double simTime)
{
    // --- 1. Generar la alarma ---
    uint32_t sensorIndex = g_rand->GetInteger(0, sensors.GetN() - 1);
    Ptr<Node> alarmNode = sensors.Get(sensorIndex);
    Vector alarmPos = alarmNode->GetObject<MobilityModel>()->GetPosition();

    NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: 🚨 ¡ALARMA! Generada en Nodo Sensor " 
                << alarmNode->GetId() << " en (" << alarmPos.x << "," << alarmPos.y << ")");

    // --- 2. Encontrar el dron que responde ---
    auto [closestMv, closestNode] = FindClosestDrone(alarmPos, recolector, recolectorMVs);

    if (!closestMv) 
    {
        NS_LOG_WARN("Alarma generada, pero no hay drones para responder.");
        return;
    }
    
    NS_LOG_INFO("... Dron " << closestNode->GetId() << " es el más cercano y responderá.");

    // --- 3. Interrumpir la patrulla del dron ---
    EventId patrolEvent = g_droneNextPatrolEvent[closestMv];
    
    if (patrolEvent.IsPending())
    {
        Simulator::Cancel(patrolEvent);
        NS_LOG_INFO("... Patrulla del Dron " << closestNode->GetId() << " cancelada.");
    }
    
    // --- 4. Enviar dron a la alarma ---
    double timeToAlarm = SetVelocityTowards(closestMv, alarmPos, alarmSpeed);
    
    // --- 5. Actualizar NetAnim ---
    if (g_anim)
    {
        g_anim->UpdateNodeColor(closestNode, 0, 255, 0); // Dron que responde = VERDE
        g_anim->UpdateNodeColor(alarmNode, 255, 255, 0); // Nodo de alarma = AMARILLO
    }
    
    // --- 6. Programar la "reanudación de patrulla" ---
    Simulator::Schedule(Seconds(timeToAlarm), &ResumePatrolAfterAlarm, 
                        closestMv, closestNode, alarmNode, 
                        route, patrolSpeed);

    // --- 7. Programar la *próxima* alarma aleatoria ---
    double nextAlarmIn = g_rand->GetValue(15.0, 25.0); 
    if (Simulator::Now().GetSeconds() + nextAlarmIn < (simTime - 10.0)) // No programar si es muy al final
    {
        Simulator::Schedule(Seconds(nextAlarmIn), &TriggerRandomAlarm,
                            sensors, recolector, recolectorMVs, 
                            route, patrolSpeed, alarmSpeed, simTime);
    }
}


// ---------------------------
// 🔸 Implementación
// ---------------------------

Ptr<YansWifiChannel> CreateMultiZoneChannel()
{
    Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();

    // Modelo multi-zona
    Ptr<MultiZonePropagationLossModel> multiZone = CreateObject<MultiZonePropagationLossModel>();

    // ============================
    //  Definición de zonas (círculos)
    // ============================
    // Cluster A - zona con poca pérdida (campo abierto)
    multiZone->AddCircularZone(0.0, 0.0, 40.0, 51.0, 2.3);

    // Cluster B - zona intermedia
    multiZone->AddCircularZone(100.0, 0.0, 40.0, 59.0, 2.0);

    // Cluster C - zona densa con árboles
    multiZone->AddCircularZone(50.0, 86.6025403784, 40.0, 50.0, 3.5);

    // Zona por defecto (fuera de los clusters)
    multiZone->SetDefaultZone(46.0, 2.5);


    // Guardar el puntero global para poder activarle efectos dinámicos (lluvia)
    g_multiZoneModel = multiZone;

    // Configurar el canal WiFi
    channel->SetPropagationLossModel(multiZone);
    channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());

    return channel;
}

void ConfigureWifiAdhoc(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy)
{
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifiMac.SetType("ns3::AdhocWifiMac");

    // Crear canal usando el método auxiliar
    Ptr<YansWifiChannel> channel = CreateMultiZoneChannel();

    // Asignar el canal físico
    wifiPhy.SetChannel(channel);

    // [NUEVO] Configuración base de potencia para nodos normales
    wifiPhy.Set("TxPowerStart", DoubleValue(16.0));
    wifiPhy.Set("TxPowerEnd", DoubleValue(16.0));
}

void InstallWifiDevices(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy,
                        NodeContainer &sensors, NodeContainer &clusterHeads,
                        NodeContainer &superCluster, NodeContainer &recolectors,
                        NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                        NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices,
                        bool useLeaderSignalPower) // [NUEVO]
{
    // Nodos sensores (potencia normal)
    sensorDevices = wifi.Install(wifiPhy, wifiMac, sensors);

    // [NUEVO] Si la opción está activada, los líderes usan más potencia
    if (useLeaderSignalPower)
    {
        NS_LOG_INFO("⚡ Cluster Heads con mayor potencia de transmisión activado.");
        YansWifiPhyHelper highPowerPhy = wifiPhy;
        highPowerPhy.Set("TxPowerStart", DoubleValue(20.0));
        highPowerPhy.Set("TxPowerEnd", DoubleValue(20.0));
        headDevices = wifi.Install(highPowerPhy, wifiMac, clusterHeads);
    }
    else
    {
        headDevices = wifi.Install(wifiPhy, wifiMac, clusterHeads);
    }

    // Super Cluster y recolectores usan potencia base
    superDevices = wifi.Install(wifiPhy, wifiMac, superCluster);
    recolectorDevices = wifi.Install(wifiPhy, wifiMac, recolectors);

    // Log: mostrar potencia TX actual de cada cluster head (útil para verificar)
    for (uint32_t i = 0; i < clusterHeads.GetN(); ++i) {
        Ptr<NetDevice> nd = headDevices.Get(i);
        Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice>(nd);
        if (dev) {
            Ptr<YansWifiPhy> phy = DynamicCast<YansWifiPhy>(dev->GetPhy());
            if (phy) {
                NS_LOG_INFO("Head " << i << " potencia TX = " << phy->GetTxPowerStart());
            }
        }
    }
}

void ConfigureClusterMobility(NodeContainer &sensors, Vector A, Vector B, Vector C)
{
    auto setupCluster = [&](Vector center, uint32_t startIndex, uint32_t endIndex)
    {
        Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
        posAlloc->Add(Vector(center.x + 5.0, center.y + 0.0, 0));
        posAlloc->Add(Vector(center.x - 5.0, center.y + 3.0, 0));
        posAlloc->Add(Vector(center.x + 0.0, center.y - 4.0, 0));

        MobilityHelper mob;
        mob.SetPositionAllocator(posAlloc);
        mob.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                             "Mode", StringValue("Time"),
                             "Time", TimeValue(Seconds(2.0)),
                             "Speed", StringValue("ns3::ConstantRandomVariable[Constant=0.5]"),
                             "Bounds", RectangleValue(Rectangle(center.x - 10.0, center.x + 10.0,
                                                                 center.y - 10.0, center.y + 10.0)));
        NodeContainer group;
        for (uint32_t i = startIndex; i <= endIndex; ++i)
            group.Add(sensors.Get(i));
        mob.Install(group);
    };

    setupCluster(A, 0, 2);
    setupCluster(B, 3, 5);
    setupCluster(C, 6, 8);
}

void ConfigureClusterHeadsAndSuper(NodeContainer &clusterHeads, NodeContainer &superCluster, MobilityHelper &mobility,
                                   Vector A, Vector B, Vector C, Vector centroid)
{
    Ptr<ListPositionAllocator> headPositions = CreateObject<ListPositionAllocator>();
    headPositions->Add(A);
    headPositions->Add(B);
    headPositions->Add(C);
    mobility.SetPositionAllocator(headPositions);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(clusterHeads);

    Ptr<ListPositionAllocator> superPos = CreateObject<ListPositionAllocator>();
    superPos->Add(centroid);
    mobility.SetPositionAllocator(superPos);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(superCluster);
}

// [MODIFICADA] Configura movilidad para N recolectores y devuelve sus modelos
std::vector<Ptr<ConstantVelocityMobilityModel>> ConfigureRecolectorMobilities(NodeContainer &recolectors, MobilityHelper &mobility, Vector A, Vector B, Vector C)
{
    std::vector<Ptr<ConstantVelocityMobilityModel>> movers;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(recolectors);

    // Bases de inicio: A, B, C (cíclico si hay más drones)
    std::vector<Vector> bases = {A, B, C};

    for (uint32_t i = 0; i < recolectors.GetN(); ++i) {
        Ptr<Node> n = recolectors.Get(i);
        Ptr<ConstantVelocityMobilityModel> mv = n->GetObject<ConstantVelocityMobilityModel>();
        // selección base según índice
        Vector base = bases[i % bases.size()];
        // pequeño offset para evitar coincidencia exacta de posiciones
        double offset = 3.0 * i;
        mv->SetPosition(Vector(base.x + offset, base.y + 0.5 * offset, base.z));
        movers.push_back(mv);
        NS_LOG_INFO("Recolector " << i << " pos inicial = (" << mv->GetPosition().x << "," << mv->GetPosition().y << ")");
    }
    return movers;
}

void InstallAodvAndStack(NodeContainer &sensors, NodeContainer &clusterHeads,
                         NodeContainer &superCluster, NodeContainer &recolectors)
{
    AodvHelper aodv;
    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(sensors);
    stack.Install(clusterHeads);
    stack.Install(superCluster);
    stack.Install(recolectors);
}

Ipv4InterfaceContainer AssignIpAddresses(Ipv4AddressHelper &address,
                                         NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                                         NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices)
{
    address.SetBase("10.0.0.0", "255.255.255.0");
    NetDeviceContainer allDevices;
    allDevices.Add(sensorDevices);
    allDevices.Add(headDevices);
    allDevices.Add(superDevices);
    allDevices.Add(recolectorDevices);
    return address.Assign(allDevices);
}


void SendSensorData(Ptr<Node> from, Ipv4Address toAddr, uint16_t port,
                    Ipv4Address finalDest, const std::string &msg)
{
    if (from == nullptr) return;

    // Construir payload
    std::ostringstream oss;
    oss << finalDest << "|" << msg;
    std::string payload = oss.str();

    Ptr<Packet> packet = Create<Packet>((uint8_t*)payload.c_str(), payload.size());
    if (packet == nullptr) return;

    // Crear socket UDP
    Ptr<Socket> socket = Socket::CreateSocket(from, UdpSocketFactory::GetTypeId());
    if (socket == nullptr) return;

    // Conectar y enviar
    if (socket->Connect(InetSocketAddress(toAddr, port)) != 0) return;

    socket->Send(packet);

    // Cerrar el socket después de enviar
    socket->Close();
}

void SendStatusEvent(Ptr<Node> from, Ipv4Address chAddr, Ipv4Address finalDest, std::string msg)
{
    uint16_t port = 4000; // Puerto del CH

    // Creamos el payload con la IP destino y el mensaje
    std::ostringstream oss;
    oss << finalDest << "|" << msg;
    std::string payload = oss.str();

    // Crear paquete con ese contenido
    Ptr<Packet> packet = Create<Packet>((uint8_t*)payload.c_str(), payload.size());

    // Crear un socket UDP temporal
    Ptr<Socket> socket = Socket::CreateSocket(from, UdpSocketFactory::GetTypeId());
    socket->Connect(InetSocketAddress(chAddr, port));
    socket->Send(packet);

    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Nodo "
                    << from->GetId()
                    << " envía STATUS: \"" << msg << "\" a "
                    << finalDest);
}

void SetupIntraClusterClients(NodeContainer &sensors,
                              NodeContainer &clusterHeads,
                              Ipv4InterfaceContainer &interfaces)
{

    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        // Calcular el Cluster Head correspondiente (3 sensores por cluster)
        uint32_t headIndex = i / 3;
        Ptr<Node> headNode = clusterHeads.Get(headIndex);   // ahora usamos el nodo
        Ipv4Address headAddr = interfaces.GetAddress(9 + headIndex); // IP del CH

        double startTime = 2.0 + i * 0.2; // escalonamiento
        double interval  = 2.0;
        double stopTime  = 58.0;

        // Programar envíos periódicos de Status/Event al Cluster Head
        for (double t = startTime; t <= stopTime; t += interval)
        {
            Simulator::Schedule(Seconds(t), [=]() {
                    SendStatusEvent(sensors.Get(i), headAddr, headAddr,
                                    "Reporte periódico al CH " + std::to_string(headIndex));
                });

        }

    }
}


void ClusterHeadAppRecvCallback(uint32_t headIndex, Ptr<Node> chNode, Ptr<const Packet> packet)
{
    uint32_t pktSize = packet->GetSize();
    std::vector<uint8_t> buffer(pktSize);
    packet->CopyData(buffer.data(), pktSize);
    std::string payload(reinterpret_cast<char*>(buffer.data()), pktSize);

    
    // Separar IP destino y resto del mensaje
    size_t sep1 = payload.find('|');
    if (sep1 == std::string::npos)
    {
        NS_LOG_WARN("CH[" << headIndex << "] paquete mal formado: " << payload);
        return;
    }

    std::string ipStr = payload.substr(0, sep1);
    std::string rest = payload.substr(sep1 + 1);

    bool isResend = false;
    std::string msg;

    // Buscar si tiene el flag RESEND al final
    size_t sep2 = rest.rfind('|');
    if (sep2 != std::string::npos)
    {
        std::string flag = rest.substr(sep2 + 1);
        if (flag == "RESEND")
        {
            isResend = true;
            msg = rest.substr(0, sep2);
        }
        else
        {
            msg = rest; // paquete original sin flag
        }
    }
    else
    {
        msg = rest; // paquete original sin flag
    }

    Ipv4Address destIp(ipStr.c_str());

    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH[" << headIndex << "] recibió paquete con destino: " << destIp 
                    << (isResend ? " [RESEND]" : "")
                    << " | Mensaje: " << msg);

    // Verificar si el destino está en el mismo cluster
    bool inCluster = false;
    NodeContainer clusterNodes = g_clusterNodesMap[chNode->GetId()];
    for (uint32_t i = 0; i < clusterNodes.GetN(); ++i)
    {
        Ptr<Node> n = clusterNodes.Get(i);
        Ptr<Ipv4> ipv4 = n->GetObject<Ipv4>();
        if (!ipv4) continue;

        for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j)
        {
            Ipv4Address addr = ipv4->GetAddress(j, 0).GetLocal();
            if (addr == destIp)
            {
                inCluster = true;
                break;
            }
        }
        if (inCluster) break;
    }

    // Si destino es el mismo CH, redirigir al supercluster
    Ptr<Ipv4> chIpv4 = chNode->GetObject<Ipv4>();
    if (chIpv4 && destIp == chIpv4->GetAddress(1,0).GetLocal()) destIp = g_superAddr;

    Ptr<Packet> safeCopy = packet->Copy();

    if (inCluster)
    {
        // Enviar paquete directo (tanto original como RESEND)
        Ptr<Socket> socket = Socket::CreateSocket(chNode, UdpSocketFactory::GetTypeId());
        socket->Connect(InetSocketAddress(destIp, 4000));
        Ptr<Packet> newPacket = Create<Packet>((uint8_t*)msg.c_str(), msg.size());
        socket->Send(newPacket);
        NS_LOG_INFO("CH[" << headIndex << "] envió mensaje directo a " << destIp
                        << (isResend ? " [RESEND]" : " [ORIGINAL]"));
    }
    else
    {
        // Si no está en el cluster, bufferizar según si es RESEND o paquete original
        if (isResend)
        {
            BufferPacketSent(chNode, destIp, msg, safeCopy);
            NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH[" << headIndex << "] bufferizó paquete REENVÍO para " << destIp);
        }
        else
        {
            BufferPacket(chNode, destIp, msg, safeCopy);
            NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH[" << headIndex << "] bufferizó paquete para " << destIp);
        }
    }

}


void SensorAppRecvCallback(uint32_t nodeIndex, Ptr<const Packet> packet)
{
    uint32_t pktSize = packet->GetSize();
    uint8_t buffer[1024];
    packet->CopyData(buffer, pktSize);
    std::string msg(reinterpret_cast<char*>(buffer), pktSize);

    NS_LOG_INFO("[" << Simulator::Now().GetSeconds()
                    << "s] Sensor[" << nodeIndex << "] recibió mensaje: "
                    << msg);
}

void RecolectorAppRecvCallback(uint32_t recolectorIndex, Ptr<Node> recolectorNode, Ptr<const Packet> packet)
{
    uint32_t pktSize = packet->GetSize();
    std::vector<uint8_t> buffer(pktSize);   
    packet->CopyData(buffer.data(), pktSize);
    std::string payload(reinterpret_cast<char*>(buffer.data()), pktSize);

    

    // Separar IP destino y mensaje
    size_t sep1 = payload.find('|');
    if (sep1 == std::string::npos)
    {
        NS_LOG_WARN("Recolector[" << recolectorIndex << "] recibió paquete mal formado: " << payload);
        return;
    }

    std::string ipStr = payload.substr(0, sep1);
    std::string rest = payload.substr(sep1 + 1);

    bool isResend = false;
    std::string msg;

    size_t sep2 = rest.rfind('|'); // buscar último '|'
    if (sep2 != std::string::npos)
    {
        std::string flag = rest.substr(sep2 + 1);
        if (flag == "RESEND")
        {
            isResend = true;
            msg = rest.substr(0, sep2);
        }
        else
        {
            msg = rest; // paquete original sin flag
        }
    }
    else
    {
        msg = rest; // paquete original sin flag
    }

    //add log (improve log with ip and message separated)
    NS_LOG_INFO("[" << Simulator::Now().GetSeconds()
                    << "s] Recolector[" << recolectorIndex << "] recibió paquete para "
                    << ipStr << (isResend ? " [RESEND]" : "") 
                    << " | Mensaje: " << msg);

    Ipv4Address destIp(ipStr.c_str());

    Ptr<Packet> safeCopy = packet->Copy();
    if (isResend)
    {
        BufferPacketSent(recolectorNode, destIp, msg, safeCopy);
        //add sim time

        NS_LOG_INFO("["<<Simulator::Now().GetSeconds()<<"s] Recolector[" << recolectorIndex << "] bufferizó paquete REENVÍO para " << destIp);
    }
    else
    {
        BufferPacket(recolectorNode, destIp, msg, safeCopy);
        NS_LOG_INFO("["<<Simulator::Now().GetSeconds()<<"s]Recolector[" << recolectorIndex << "] bufferizó paquete para " << destIp);
    }
}



// Callback para el supercluster
void SuperClusterAppRecvCallback(uint32_t scIndex, Ptr<Node> scNode, Ptr<const Packet> packet)
{
    uint32_t pktSize = packet->GetSize();

    std::vector<uint8_t> buffer(pktSize);
    packet->CopyData(buffer.data(), pktSize);
    std::string payload(reinterpret_cast<char*>(buffer.data()), pktSize);

    // Separar IP destino y mensaje
    size_t sep = payload.find('|');

    //update log with destination ip and message
    NS_LOG_INFO("[" << Simulator::Now().GetSeconds()
                    << "s] SuperCluster[" << scIndex << "] recibió paquete para "
                    << payload.substr(0, sep) << " | Mensaje: " 
                    << payload.substr(sep + 1));
    
    if (sep == std::string::npos)
    {
        NS_LOG_WARN("SuperCluster[" << scIndex << "] recibió paquete mal formado: " << payload);
        return;
    }

    std::string ipStr = payload.substr(0, sep);
    std::string msg = payload.substr(sep + 1)+"|RESEND"; 

    //setear destip
    Ipv4Address destIp(ipStr.c_str());

    //discard if the message is intended for the supercluster itself
    Ptr<Ipv4> scIpv4 = scNode->GetObject<Ipv4>();
    if (scIpv4 && destIp == scIpv4->GetAddress(1,0).GetLocal()) 
    {
        NS_LOG_INFO("SuperCluster[" << scIndex << "] recibió mensaje destinado a sí mismo: " << msg);
        return;
    }

    // 🔹 Marcar el payload como RESEND antes de bufferizar
    std::string resendPayload = ipStr + "|" + msg + "|RESEND";
    Ptr<Packet> newPkt = Create<Packet>(reinterpret_cast<const uint8_t*>(resendPayload.c_str()), resendPayload.size());

    // Bufferizar el paquete completo en el buffer de reenvío
    BufferPacketSent(scNode, destIp, msg, newPkt);

    NS_LOG_INFO("SuperCluster[" << scIndex << "] bufferizó mensaje para REENVÍO a " << destIp);
}

void InstallUdpServers(NodeContainer &sensors, NodeContainer &recolectors, NodeContainer &superCluster, NodeContainer &clusterHeads, double simTime)
{
    // 1️⃣ Server en cada recolector (recibe datos de clusters)
    {
        uint16_t port = 5000;
        UdpServerHelper recolectorServer(port);

        for (uint32_t i = 0; i < recolectors.GetN(); ++i)
        {
            ApplicationContainer apps = recolectorServer.Install(recolectors.Get(i));
            apps.Start(Seconds(1.0));

            // Obtener la instancia del servidor UDP recién instalado
            Ptr<UdpServer> srv = DynamicCast<UdpServer>(apps.Get(0));

            // Conectar la traza Rx con el callback, pasando el ID del recolector
            srv->TraceConnectWithoutContext("Rx",
                MakeBoundCallback(&RecolectorAppRecvCallback, i, recolectors.Get(i)));

            NS_LOG_INFO("Servidor Recolector " << i
                        << " instalado en nodo " << recolectors.Get(i)->GetId()
                        << " (puerto " << port << ")");
        }

    }

    // 2️⃣ Server en el supercluster (recibe uploads de drones)
    {
        uint16_t port = 6000;
        UdpServerHelper server(port);
        ApplicationContainer apps = server.Install(superCluster.Get(0));
        apps.Start(Seconds(1.0));

        // Obtener la instancia del servidor UDP recién instalado
        Ptr<UdpServer> srv = DynamicCast<UdpServer>(apps.Get(0));

        // Conectar la traza Rx con el callback, pasando el índice 0 (único nodo del supercluster)
        srv->TraceConnectWithoutContext("Rx", MakeBoundCallback(&SuperClusterAppRecvCallback, 0, superCluster.Get(0)));

        NS_LOG_INFO("Servidor SuperCluster instalado en nodo " 
                    << superCluster.Get(0)->GetId() 
                    << " (puerto " << port << ")");

    }


    // 🔹 Servidor UDP interno en cada cluster head (para recibir de sensores)
    {
        uint16_t intraPort = 4000;
        UdpServerHelper intraServer(intraPort);

        for (uint32_t i = 0; i < clusterHeads.GetN(); ++i)
        {
            Ptr<Node> chNode = clusterHeads.Get(i);  // 🔹 obtenemos el puntero del Cluster Head

            ApplicationContainer apps = intraServer.Install(chNode);
            apps.Start(Seconds(0.5));

            // 🔹 Obtiene la instancia del servidor UDP recién instalado
            Ptr<UdpServer> srv = DynamicCast<UdpServer>(apps.Get(0));

            // 🔹 Conecta la traza Rx con tu callback, pasando también el nodo
            srv->TraceConnectWithoutContext("Rx",
                MakeBoundCallback(&ClusterHeadAppRecvCallback, i, chNode));

            NS_LOG_INFO("Servidor interno CH " << i
                        << " instalado en nodo " << chNode->GetId()
                        << " (puerto " << intraPort << ")");
        }
    }


    {
        uint16_t sensorPort = 4000; // Mismo puerto que usan los CH al reenviar
        UdpServerHelper sensorServer(sensorPort);

        for (uint32_t i = 0; i < sensors.GetN(); ++i)
        {
            ApplicationContainer apps = sensorServer.Install(sensors.Get(i));
            apps.Start(Seconds(1.0));

            Ptr<UdpServer> srv = DynamicCast<UdpServer>(apps.Get(0));
            srv->TraceConnectWithoutContext("Rx", MakeBoundCallback(&SensorAppRecvCallback, i));

            NS_LOG_INFO("Servidor sensor " << i << " (puerto " << sensorPort << ")");
        }
    }
}

void ScheduleRecolectorMovement(Ptr<ConstantVelocityMobilityModel> mv,
                                Vector A, Vector B, Vector C,
                                double speed,
                                double startTime,
                                Vector centroid)
{
    auto computeVelocity = [speed](const Vector &from, const Vector &to) {
        Vector dir = to - from;
        double len = std::sqrt(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);
        if (len == 0) return Vector(0,0,0);
        return Vector(dir.x / len * speed, dir.y / len * speed, dir.z / len * speed);
    };

    Vector start = mv->GetPosition();
    std::vector<Vector> route = {A, B, C}; // O rotación según inicio si quieres

    double t = startTime;
    Vector from = start;

    for (size_t i=0; i<route.size(); ++i) {
        Vector to = route[i];
        double dist = std::sqrt((to.x - from.x)*(to.x - from.x) + (to.y - from.y)*(to.y - from.y) + (to.z - from.z)*(to.z - from.z));
        double travelTime = dist / speed;
        Vector vel = computeVelocity(from, to);

        Simulator::Schedule(Seconds(t), &ConstantVelocityMobilityModel::SetVelocity, mv, vel);
        Simulator::Schedule(Seconds(t + travelTime), &ConstantVelocityMobilityModel::SetPosition, mv, to);

        t += travelTime;
        from = to;
    }

    // Finalmente, regresar al centroide suavemente
    Vector pos = route.back();
    double distToCentroid = std::sqrt((centroid.x - pos.x)*(centroid.x - pos.x) + (centroid.y - pos.y)*(centroid.y - pos.y) + (centroid.z - pos.z)*(centroid.z - pos.z));
    double travelTime = distToCentroid / speed;
    Vector velToCentroid = computeVelocity(pos, centroid);

    Simulator::Schedule(Seconds(t), &ConstantVelocityMobilityModel::SetVelocity, mv, velToCentroid);
    Simulator::Schedule(Seconds(t + travelTime), &ConstantVelocityMobilityModel::SetPosition, mv, centroid);

    // Cuando llegue, reinicia patrulla
    Simulator::Schedule(Seconds(t + travelTime), [=]() {
        ScheduleRecolectorMovement(mv, A, B, C, speed, Simulator::Now().GetSeconds(), centroid);
    });
}


// Calcula y aplica velocidad para que mv llegue a centroid justo al tiempo simEnd.
// Si startLead > 0, programa el cálculo en simEnd - startLead; si startLead == 0 calcula ahora.
static void ReturnToSuperCallback(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simEnd, double patrolSpeed)
{
    Vector pos = mv->GetPosition();
    Vector dir = centroid - pos;
    double dist = std::sqrt(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);

    if (dist <= 1e-9) {
        // Llegó al supercluster: reiniciar ruta
        ScheduleRecolectorMovement(mv, Vector(0,0,0), Vector(100,0,0), Vector(50,86.6,0), patrolSpeed, simEnd,centroid);
        return;
    }

    double now = Simulator::Now().GetSeconds();
    double remaining = simEnd - now;
    double speed = (remaining > 0.0) ? dist / remaining : 0.0;
    Vector vel = Vector(dir.x / dist * speed, dir.y / dist * speed, dir.z / dist * speed);

    mv->SetVelocity(vel);
    Simulator::Schedule(Seconds(dist / speed), &ReturnToSuperCallback, mv, centroid, simEnd, patrolSpeed);
}


static void SetupReturnToSuper(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simEnd, double leadTime, double patrolSpeed)
{
    double start = simEnd - leadTime;
    if (start < 0.0) start = 0.0;

    Simulator::Schedule(Seconds(start), &ReturnToSuperCallback, mv, centroid, simEnd, patrolSpeed);

    Simulator::Schedule(Seconds(simEnd), &ConstantVelocityMobilityModel::SetPosition, mv, centroid);
    Simulator::Schedule(Seconds(simEnd), &ConstantVelocityMobilityModel::SetVelocity, mv, Vector(0,0,0));
}

void PeriodicProximityCheck(NodeContainer &clusterHeads,
                            NodeContainer &recolectors,
                            double checkInterval)
{
    double ptDbm = 0.0;            
    double rxSensitivityDbm = -85; 
    double txGainDb = 0.0;
    double rxGainDb = 0.0;

    Ptr<MultiZonePropagationLossModel> multiZone = g_multiZoneModel;
    if (!multiZone) {
        NS_LOG_WARN("MultiZone model no inicializado. Abortando chequeo.");
        return;
    }

    for (uint32_t i = 0; i < clusterHeads.GetN(); ++i)
    {
        Ptr<Node> ch = clusterHeads.Get(i);
        Ptr<MobilityModel> mobCH = ch->GetObject<MobilityModel>();
        Vector posCH = mobCH->GetPosition();

        // Obtener los nodos que pertenecen a este cluster head
        auto &clusterNodes = g_clusterNodesMap[ch->GetId()];

        for (uint32_t j = 0; j < recolectors.GetN(); ++j)
        {
            Ptr<Node> recolector = recolectors.Get(j);
            Ptr<MobilityModel> mobR = recolector->GetObject<MobilityModel>();
            Vector posR = mobR->GetPosition();

            double dx = posCH.x - posR.x;
            double dy = posCH.y - posR.y;
            double dz = posCH.z - posR.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            double lossDb = multiZone->GetLoss(posCH, posR);
            double prDbm = ptDbm + txGainDb + rxGainDb - lossDb;

            bool canSend = prDbm >= rxSensitivityDbm;

            if (canSend)
            {
                NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH " << i
                                << " puede enviar a Recolector " << j
                                << " (Dist=" << distance << " m, PL=" << lossDb
                                << " dB, Pr=" << prDbm << " dBm)");

                // Direccion IP del recolector
                Ptr<Ipv4> ipv4 = recolector->GetObject<Ipv4>();
                Ipv4Address recolectorAddr = ipv4->GetAddress(1,0).GetLocal();

                // 1️⃣ Enviar paquetes bufferizados del CH
                while (HasBufferedPackets(ch)) {
                    BufferedPacket bpkt = PeekBufferedPacket(ch);
                    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH " << i
                                    << " enviando paquete bufferizado a Recolector " << j
                                    << " (dest=" << bpkt.dest << ", tamaño=" << bpkt.packet->GetSize() << " bytes)");

                    
                    SendSensorData(ch, recolectorAddr, 5000, bpkt.dest, bpkt.msg);
                    PopBufferedPacket(ch);
                }

                // 2️⃣ Revisar buffer de paquetes enviados por el recolector para reenvío
                while (HasBufferedPacketsSent(recolector)) {

                    //ADD LOG
                    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Recolector " << j
                                    << " revisando paquete bufferizado para reenvío al CH " << i);

                    BufferedPacket bpkt = PeekBufferedPacketSent(recolector);

                    // Solo enviar si el destino pertenece al cluster del CH
                    bool belongsToCluster = false;
                    for (uint32_t k = 0; k < clusterNodes.GetN(); ++k) {
                        Ptr<Node> node = clusterNodes.Get(k);
                        Ptr<Ipv4> ipv4Node = node->GetObject<Ipv4>();
                        if (ipv4Node && bpkt.dest == ipv4Node->GetAddress(1,0).GetLocal()) {
                            belongsToCluster = true;
                            break;
                        }
                    }

                    if (belongsToCluster) {

                        //get ch ip
                        Ptr<Ipv4> chIpv4 = ch->GetObject<Ipv4>();
                        Ipv4Address chAddr = chIpv4->GetAddress(1,0).GetLocal();
                        NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Recolector " << j
                                        << " reenviando paquete a CH " << i
                                        << " (dest=" << bpkt.dest << ", tamaño=" << bpkt.packet->GetSize() << " bytes)");
                        SendSensorData(recolector, chAddr, 4000, bpkt.dest, bpkt.msg);
                        PopBufferedPacketSent(recolector);
                    } else {
                        // No pertenece a este cluster, lo dejamos para otro CH
                        break;
                    }
                }
            }
            else
            {
                NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] CH " << i
                                << " NO puede enviar a Recolector " << j
                                << " (Dist=" << distance << " m, PL=" << lossDb
                                << " dB, Pr=" << prDbm << " dBm)");
            }
        }
    }

    Simulator::Schedule(Seconds(checkInterval),
                        &PeriodicProximityCheck,
                        std::ref(clusterHeads),
                        std::ref(recolectors),
                        checkInterval);
}



void SchedulePeriodicProximityCheck(NodeContainer &clusterHeads,
                                    NodeContainer &recolectors,
                                    double checkInterval)
{
    Simulator::Schedule(Seconds(checkInterval), [=, &clusterHeads, &recolectors]() {
        PeriodicProximityCheck(clusterHeads, recolectors, checkInterval);
    });
}

void SuperClusterProximityCheck(NodeContainer &superCluster,
                                NodeContainer &recolectors,
                                
                                double checkInterval)
{
    // Asumimos TxPower y sensibilidad fijas
    double txPowerDbm = 0.0;       // dBm
    double rxSensitivityDbm = -85.0; // dBm

    Simulator::Schedule(Seconds(checkInterval), &SuperClusterProximityCheck,
                        superCluster, recolectors, checkInterval);

    for (uint32_t i = 0; i < superCluster.GetN(); ++i)
    {
        Ptr<Node> scNode = superCluster.Get(i);
        Ptr<MobilityModel> scMob = scNode->GetObject<MobilityModel>();
        Vector scPos = scMob->GetPosition();

        
                

        for (uint32_t j = 0; j < recolectors.GetN(); ++j)
        {
            Ptr<Node> rcNode = recolectors.Get(j);
            Ptr<MobilityModel> rcMob = rcNode->GetObject<MobilityModel>();
            Vector rcPos = rcMob->GetPosition();

            // Calcular pérdida usando MultiZonePropagationLossModel global
            double pathLoss = g_multiZoneModel->GetLoss(rcPos, scPos); // rx = SC, tx = Recolector
            double rxPower = txPowerDbm - pathLoss;

            //get recolector address
            Ptr<Node> node = recolectors.Get(j);
            Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
            Ipv4Address recolectorAddr = ipv4->GetAddress(1,0).GetLocal();

            if (rxPower >= rxSensitivityDbm)
            {
                NS_LOG_INFO(Simulator::Now().GetSeconds() << "s] Recolector " << j
                    << " puede enviar a SuperCluster " << i
                    << " (Dist=" << CalculateDistance(rcPos, scPos)
                    << " m, PL=" << pathLoss
                    << " dB, Pr=" << rxPower << " dBm)");
                
                while (HasBufferedPackets(rcNode)) {
                    BufferedPacket bpkt = PeekBufferedPacket(rcNode);

                    //get supercluster ip
                    Ptr<Ipv4> scIpv4 = scNode->GetObject<Ipv4>();
                    Ipv4Address scAddr = scIpv4->GetAddress(1,0).GetLocal();
                    
                    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Recolector " << j
                                    << " enviando paquete bufferizado a SuperCluster " << i
                                    << " (dest=" << bpkt.dest << ", tamaño=" << bpkt.packet->GetSize() << " bytes)");

                    SendSensorData(rcNode, scAddr , 6000 , bpkt.dest, bpkt.msg );

                    // Eliminar paquete de la cola
                    PopBufferedPacket(rcNode);
                }


                while (HasBufferedPacketsSent(scNode)) {
                    BufferedPacket bpkt = PeekBufferedPacketSent(scNode);
                    
                    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] SuperCluster " << i
                                    << " enviando paquete bufferizado al Recolector " << j
                                    << " (dest=" << bpkt.dest << ", tamaño=" << bpkt.packet->GetSize() << " bytes)");

                    
                    SendSensorData(scNode, recolectorAddr , 5000 , bpkt.dest, bpkt.msg );

                    // Eliminar paquete de la cola
                    PopBufferedPacketSent(scNode);

                }
                

                
            }
            else
            {
                NS_LOG_INFO(Simulator::Now().GetSeconds() << "s] Recolector " << j
                    << " NO puede enviar a SuperCluster " << i
                    << " (Dist=" << CalculateDistance(rcPos, scPos)
                    << " m, PL=" << pathLoss
                    << " dB, Pr=" << rxPower << " dBm)");
            }
        }
    }
}



void ScheduleSuperClusterProximityCheck(NodeContainer &superCluster,
                                        NodeContainer &recolectors,
                                        double checkInterval)
{
    Simulator::Schedule(Seconds(checkInterval), [=, &superCluster, &recolectors]() {
        SuperClusterProximityCheck(superCluster, recolectors, checkInterval);
    });
}

void ScheduleSendToOtherCluster(Ptr<Node> srcNode, Ipv4Address destIp, uint16_t port, double sizeBytes, double delaySeconds) {
    Simulator::Schedule(Seconds(delaySeconds), [=]() {
        NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Nodo " 
                        << srcNode->GetId() 
                        << " preparando envío de paquete a nodo con IP " << destIp);

        // Crear el paquete
        Ptr<Packet> pkt = Create<Packet>(static_cast<uint32_t>(sizeBytes));

        // -----------------------------
        // Buscar el cluster head del nodo emisor
        // -----------------------------
        Ipv4Address clusterHeadIp;
        bool found = false;
        for (auto &pair : g_clusterNodesMap) {
            uint32_t chId = pair.first;
            NodeContainer nodes = pair.second;

            for (uint32_t i = 0; i < nodes.GetN(); ++i) {
                if (nodes.Get(i)->GetId() == srcNode->GetId()) {
                    Ptr<Ipv4> ipv4 = NodeList::GetNode(chId)->GetObject<Ipv4>();
                    clusterHeadIp = ipv4->GetAddress(1,0).GetLocal(); // interfaz ad-hoc
                    found = true;
                    break;
                }
            }
            if (found) break;
        }

        if (!found) {
            NS_LOG_WARN("Nodo " << srcNode->GetId() << " no pertenece a ningún cluster head, usando destIp directamente");
            clusterHeadIp = destIp;
        }

        NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "s] Nodo "
                        << srcNode->GetId() 
                        << " enviará paquete primero al ClusterHead con IP " << clusterHeadIp);

        // Bufferizar el paquete hacia el cluster head, indicando el destino final
        SendStatusEvent(srcNode, clusterHeadIp, destIp, "Saludos desde el nodo"+std::to_string(srcNode->GetId())+"!");
    });
}



void RunSimulation(double simTime)
{
    

    double startRain = 20.0;
    double endRain   = 40.0;

    if (g_multiZoneModel)
    {
        Simulator::Schedule(Seconds(startRain),
                            &MultiZonePropagationLossModel::SetRainEffect,
                            g_multiZoneModel, true);

        Simulator::Schedule(Seconds(endRain),
                            &MultiZonePropagationLossModel::SetRainEffect,
                            g_multiZoneModel, false);
    }
    else
    {
        NS_LOG_WARN("g_multiZoneModel is null: CreateMultiZoneChannel() debe llamarse antes de RunSimulation()");
    }

    

    Simulator::Stop(Seconds(simTime));
    
    Simulator::Run();
    Simulator::Destroy();
}

// ---------------------------
// 🔹 Main
// ---------------------------
int main(int argc, char *argv[])
{
    // Inicializar el generador aleatorio
    SeedManager::SetSeed(1); 
    g_rand = CreateObject<UniformRandomVariable>();

    NS_LOG_INFO("Simulación iniciando...");

    LogComponentEnable("ManetRecolector", LOG_LEVEL_INFO);
    LogComponentEnable("StoreCarryForward", LOG_LEVEL_INFO);

    double simTime = 120.0;
    uint32_t nSensors = 9, nClusterHeads = 3;
    uint32_t nRecolector = 1; // [NUEVO] número de drones por CLI
    bool useLeaderSignalPower = false; // [NUEVO]

    // [NUEVO] Activable desde línea de comandos
    CommandLine cmd;
    cmd.AddValue("useLeaderSignalPower", "Activa mayor potencia en los líderes de los clusters", useLeaderSignalPower);
    cmd.AddValue("nRecolector", "Número de nodos recolectors (drones)", nRecolector);
    cmd.AddValue("simTime", "Duración de la simulación (s)", simTime);
    cmd.Parse(argc, argv);

    NodeContainer sensors, clusterHeads, recolectors, superCluster;
    sensors.Create(nSensors);
    clusterHeads.Create(nClusterHeads);
    recolectors.Create(nRecolector);
    superCluster.Create(1);

    // ---------------------------
    // Crear mapa de nodos por cluster
    // ---------------------------
    for (uint32_t i = 0; i < clusterHeads.GetN(); ++i)
    {
        NodeContainer clusterNodes;

        // Distribuir sensores entre los cluster heads (simple round-robin)
        for (uint32_t j = 0; j < sensors.GetN(); ++j)
        {
            if (j % clusterHeads.GetN() == i)
            {
                clusterNodes.Add(sensors.Get(j));
            }
        }

        // Guardar en el mapa global
        g_clusterNodesMap[clusterHeads.Get(i)->GetId()] = clusterNodes;

        // Logging
        NS_LOG_INFO("ClusterHead " << i << " tiene " << clusterNodes.GetN() << " nodos.");
    }


    // Crear el objeto de animación aquí, al inicio de main()
    AnimationInterface anim("manet_recolector.xml");
    g_anim = &anim; // Asignar el puntero global AHORA

    WifiHelper wifi;
    WifiMacHelper wifiMac;
    YansWifiPhyHelper wifiPhy;
    ConfigureWifiAdhoc(wifi, wifiMac, wifiPhy);

    NetDeviceContainer sensorDevices, headDevices, superDevices, recolectorDevices;
    InstallWifiDevices(wifi, wifiMac, wifiPhy, sensors, clusterHeads, superCluster, recolectors,
                       sensorDevices, headDevices, superDevices, recolectorDevices,
                       useLeaderSignalPower); // [NUEVO]

    Vector A(0.0, 0.0, 0.0), B(100.0, 0.0, 0.0), C(50.0, 86.6025403784, 0.0);
    Vector centroid((A.x + B.x + C.x) / 3.0, (A.y + B.y + C.y) / 3.0, 0.0);

    // Almacenar la ruta de patrulla
    std::vector<Vector> patrolRoute = {A, B, C};
    double patrolSpeed = 10.0; // m/s
    double alarmSpeed = 20.0;  // m/s

    ConfigureClusterMobility(sensors, A, B, C);
    MobilityHelper mobility;
    ConfigureClusterHeadsAndSuper(clusterHeads, superCluster, mobility, A, B, C, centroid);

    // Configurar movilidad para múltiples recolectores y obtener sus modelos
    std::vector<Ptr<ConstantVelocityMobilityModel>> recolectorMVs = ConfigureRecolectorMobilities(recolectors, mobility, A, B, C);

    InstallAodvAndStack(sensors, clusterHeads, superCluster, recolectors);

    Ipv4AddressHelper address;
    Ipv4InterfaceContainer interfaces = AssignIpAddresses(address, sensorDevices, headDevices, superDevices, recolectorDevices);

    

    // Obtener direcciones IP de los recolectores
    std::vector<Ipv4Address> recolectorAddrs;
    for (uint32_t i = 0; i < recolectorDevices.GetN(); ++i)
    {
        Ptr<Node> node = recolectors.Get(i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        Ipv4Address addr = ipv4->GetAddress(1, 0).GetLocal(); // Interfaz 1 = WiFi Adhoc
        recolectorAddrs.push_back(addr);

        NS_LOG_INFO("Recolector " << i << " -> " << addr);
    }

    // Dirección del supercluster (único nodo)
    g_superAddr = superCluster.Get(0)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    NS_LOG_INFO("Dirección del supercluster: " << g_superAddr);


    // Instalar aplicaciones UDP (servidores en cada recolectors, clientes en cluster heads -> round-robin)
    InstallUdpServers(sensors, recolectors, superCluster, clusterHeads, simTime);

    // Configurar clientes intra-cluster (sensores -> cluster heads)
    //SetupIntraClusterClients(sensors, clusterHeads, interfaces);

    Ptr<Node> src = sensors.Get(2);
    Ipv4Address destIp = interfaces.GetAddress(7); // dirección IP del nodo destino
    uint16_t port = 6000;
    double packetSize = 100; // bytes
    double delay = 5.0; // segundos después de iniciar la simulación

    ScheduleSendToOtherCluster(src, destIp, port, packetSize, delay);

    // -----------------------------------------------------------------
    // 🔹 LÓGICA DE INICIO DE MOVIMIENTO 🔹
    // -----------------------------------------------------------------
    // 🔹 LÓGICA DE INICIO DE MOVIMIENTO 🔹
    for (uint32_t i = 0; i < recolectorMVs.size(); ++i) {
        Ptr<ConstantVelocityMobilityModel> mv = recolectorMVs[i];

        // Ahora usamos la nueva función que maneja todo el ciclo
        ScheduleRecolectorMovement(mv, A, B, C, patrolSpeed, simTime, centroid);
    }

    // Programar chequeos periódicos de proximidad (cluster heads -> recolectors)
    SchedulePeriodicProximityCheck(clusterHeads, recolectors);

    // Programar chequeos periódicos de proximidad (supercluster -> recolectors)
    double checkInterval = 1.0; // cada segundo
    ScheduleSuperClusterProximityCheck(superCluster, recolectors, checkInterval);


     // -----------------------------------------------------------------
    // 🔹 Colores Iniciales (Todos Iguales) 🔹
    // -----------------------------------------------------------------
    for (uint32_t i = 0; i < sensors.GetN(); ++i)
    {
        anim.UpdateNodeColor(sensors.Get(i), 0, 0, 255); // Sensores = Azul
    }
    for (uint32_t i = 0; i < clusterHeads.GetN(); ++i)
    {
        anim.UpdateNodeColor(clusterHeads.Get(i), 0, 0, 255); // Heads = Azul
    }
    for (uint32_t i = 0; i < recolectors.GetN(); ++i)
    {
        anim.UpdateNodeColor(recolectors.Get(i), 0, 0, 255); // Drones = Azul
    }
    anim.UpdateNodeColor(superCluster.Get(0), 0, 0, 255); // Super = Azul
    // -----------------------------------------------------------------


    // -----------------------------------------------------------------
    // 🔹 LÓGICA DE INICIO DE ALARMA 🔹
    // -----------------------------------------------------------------
    // Programamos la *primera* alarma en un tiempo aleatorio
    double firstAlarmTime = g_rand->GetValue(5.0, 15.0);
    Simulator::Schedule(Seconds(firstAlarmTime), &TriggerRandomAlarm,
                        sensors, recolectors, recolectorMVs,
                        patrolRoute, patrolSpeed, alarmSpeed, simTime);
    

    RunSimulation(simTime);

    return 0;
}
