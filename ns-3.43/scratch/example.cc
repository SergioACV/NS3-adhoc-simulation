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

// Mapa para rastrear el "próximo evento de patrulla" de cada dron.
static std::map< Ptr<ConstantVelocityMobilityModel>, EventId > g_droneNextPatrolEvent;

// 🔹 CORRECCIÓN DE COMPILACIÓN: Era std::map, no std.map
static std::map< Ptr<ConstantVelocityMobilityModel>, uint32_t > g_dronePatrolIndex;

NS_LOG_COMPONENT_DEFINE("ManetRecolector");

// ---------------------------
// 🔹 Declaración de funciones
// ---------------------------

// Funciones de Configuración
Ptr<YansWifiChannel> CreateMultiZoneChannel();
void ConfigureWifiAdhoc(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy);
void InstallWifiDevices(WifiHelper &wifi, WifiMacHelper &wifiMac, WifiPhyHelper &wifiPhy,
                        NodeContainer &sensors, NodeContainer &clusterHeads,
                        NodeContainer &superCluster, NodeContainer &recolector,
                        NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                        NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices,
                        bool useLeaderSignalPower);
void ConfigureClusterMobility(NodeContainer &sensors, Vector A, Vector B, Vector C);
void ConfigureClusterHeadsAndSuper(NodeContainer &clusterHeads, NodeContainer &superCluster, MobilityHelper &mobility,
                                     Vector A, Vector B, Vector C, Vector centroid);
std::vector<Ptr<ConstantVelocityMobilityModel>> ConfigureRecolectorMobilities(NodeContainer &recolector, MobilityHelper &mobility, Vector A, Vector B, Vector C);
void InstallAodvAndStack(NodeContainer &sensors, NodeContainer &clusterHeads,
                         NodeContainer &superCluster, NodeContainer &recolector);
Ipv4InterfaceContainer AssignIpAddresses(Ipv4AddressHelper &address,
                                         NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                                         NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices);
void SetupUdpApplications(NodeContainer &clusterHeads, NodeContainer &superCluster, NodeContainer &recolector,
                          const std::vector<Ipv4Address> &recolectorAddrs, Ipv4Address superAddr, double simTime);

// Funciones de Retorno
static void ReturnToSuperCallback(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simTime);
static void SetupReturnToSuper(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simTime, double leadTime = 5.0);
void RunSimulation(double simTime);


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
// 🔸 Implementación (Funciones Antiguas)
// ---------------------------

Ptr<YansWifiChannel> CreateMultiZoneChannel()
{
    Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
    Ptr<MultiZonePropagationLossModel> multiZone = CreateObject<MultiZonePropagationLossModel>();
    multiZone->AddCircularZone(0.0, 0.0, 40.0, 40.0, 20.0);
    multiZone->AddCircularZone(100.0, 0.0, 40.0, 60.0, 25.0);
    multiZone->AddCircularZone(50.0, 86.6025403784, 40.0, 80.0, 30.0);
    multiZone->SetDefaultZone(100.0, 35.0);
    g_multiZoneModel = multiZone;
    channel->SetPropagationLossModel(multiZone);
    channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
    return channel;
}

void ConfigureWifiAdhoc(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy)
{
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifiMac.SetType("ns3::AdhocWifiMac");
    Ptr<YansWifiChannel> channel = CreateMultiZoneChannel();
    wifiPhy.SetChannel(channel);
    wifiPhy.Set("TxPowerStart", DoubleValue(16.0));
    wifiPhy.Set("TxPowerEnd", DoubleValue(16.0));
}

void InstallWifiDevices(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy,
                        NodeContainer &sensors, NodeContainer &clusterHeads,
                        NodeContainer &superCluster, NodeContainer &recolector,
                        NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                        NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices,
                        bool useLeaderSignalPower)
{
    sensorDevices = wifi.Install(wifiPhy, wifiMac, sensors);
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
    superDevices = wifi.Install(wifiPhy, wifiMac, superCluster);
    recolectorDevices = wifi.Install(wifiPhy, wifiMac, recolector);

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

std::vector<Ptr<ConstantVelocityMobilityModel>> ConfigureRecolectorMobilities(NodeContainer &recolector, MobilityHelper &mobility, Vector A, Vector B, Vector C)
{
    std::vector<Ptr<ConstantVelocityMobilityModel>> movers;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(recolector);
    std::vector<Vector> bases = {A, B, C};

    for (uint32_t i = 0; i < recolector.GetN(); ++i) {
        Ptr<Node> n = recolector.Get(i);
        Ptr<ConstantVelocityMobilityModel> mv = n->GetObject<ConstantVelocityMobilityModel>();
        Vector base = bases[i % bases.size()];
        double offset = 3.0 * i;
        mv->SetPosition(Vector(base.x + offset, base.y + 0.5 * offset, base.z));
        movers.push_back(mv);
        NS_LOG_INFO("Recolector " << i << " pos inicial = (" << mv->GetPosition().x << "," << mv->GetPosition().y << ")");
    }
    return movers;
}

void InstallAodvAndStack(NodeContainer &sensors, NodeContainer &clusterHeads,
                         NodeContainer &superCluster, NodeContainer &recolector)
{
    AodvHelper aodv;
    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(sensors);
    stack.Install(clusterHeads);
    stack.Install(superCluster);
    stack.Install(recolector);
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

void SetupUdpApplications(NodeContainer &clusterHeads, NodeContainer &superCluster, NodeContainer &recolector,
                          const std::vector<Ipv4Address> &recolectorAddrs, Ipv4Address superAddr, double simTime)
{
    uint16_t port = 5000;
    for (uint32_t r = 0; r < recolector.GetN(); ++r) {
        UdpServerHelper server(port);
        ApplicationContainer serverApp = server.Install(recolector.Get(r));
        serverApp.Start(Seconds(1.0));
        serverApp.Stop(Seconds(simTime));
    }
    uint16_t coordPort = 6000;
    UdpServerHelper coordServer(coordPort);
    ApplicationContainer coordServerApp = coordServer.Install(superCluster.Get(0));
    coordServerApp.Start(Seconds(1.0));
    coordServerApp.Stop(Seconds(simTime));
    ApplicationContainer clientApps, coordClientApps;
    for (uint32_t i = 0; i < clusterHeads.GetN(); ++i)
    {
        uint32_t chosen = 0;
        if (!recolectorAddrs.empty()) {
            chosen = i % recolectorAddrs.size();
        }
        UdpClientHelper clientToRecolector(recolectorAddrs[chosen], port);
        clientToRecolector.SetAttribute("MaxPackets", UintegerValue(1000));
        clientToRecolector.SetAttribute("Interval", TimeValue(Seconds(2.0)));
        clientToRecolector.SetAttribute("PacketSize", UintegerValue(512));
        clientApps.Add(clientToRecolector.Install(clusterHeads.Get(i)));
        UdpClientHelper clientToSuper(superAddr, coordPort);
        clientToSuper.SetAttribute("MaxPackets", UintegerValue(1000));
        clientToSuper.SetAttribute("Interval", TimeValue(Seconds(5.0)));
        clientToSuper.SetAttribute("PacketSize", UintegerValue(64));
        coordClientApps.Add(clientToSuper.Install(clusterHeads.Get(i)));
    }
    clientApps.Start(Seconds(5.0));
    clientApps.Stop(Seconds(simTime - 1));
    coordClientApps.Start(Seconds(2.0));
    coordClientApps.Stop(Seconds(simTime - 1));
}


// 🔹 CORRECCIÓN DE LÓGICA: Esta función ahora cancela la patrulla
static void
ReturnToSuperCallback(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simEnd)
{
    // Esta función es llamada por el evento 'SetupReturnToSuper'.
    // Cancela cualquier evento de patrulla pendiente.
    EventId patrolEvent = g_droneNextPatrolEvent[mv];
    
    if (patrolEvent.IsPending())
    {
        Simulator::Cancel(patrolEvent);
        NS_LOG_INFO(Simulator::Now().GetSeconds() << "s: Dron " << mv->GetObject<Node>()->GetId() 
                    << " cancelando patrulla para volver a la base.");
    }
    
    // Ahora, procede a calcular la velocidad de retorno.
    Vector pos = mv->GetPosition();
    Vector dir = Vector(centroid.x - pos.x, centroid.y - pos.y, centroid.z - pos.z);
    double dist = std::sqrt(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);
    double now = Simulator::Now().GetSeconds();
    double remaining = simEnd - now;
    if (remaining <= 0.0 || dist <= 1e-9)
    {
        mv->SetPosition(centroid);
        mv->SetVelocity(Vector(0,0,0));
        return;
    }
    double speed = dist / remaining;
    Vector vel = Vector(dir.x / dist * speed, dir.y / dist * speed, dir.z / dist * speed);
    mv->SetVelocity(vel);
}

static void
SetupReturnToSuper(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simEnd, double leadTime)
{
    double start = simEnd - leadTime;
    if (start < 0.0) start = 0.0;
    // Programa la nueva versión de ReturnToSuperCallback
    Simulator::Schedule(Seconds(start), &ReturnToSuperCallback, mv, centroid, simEnd);
    
    // Estos eventos aseguran que termine exactamente en el centroide
    Simulator::Schedule(Seconds(simEnd), &ConstantVelocityMobilityModel::SetPosition, mv, centroid);
    Simulator::Schedule(Seconds(simEnd), &ConstantVelocityMobilityModel::SetVelocity, mv, Vector(0,0,0));
}

void RunSimulation(double simTime)
{
    // 'anim' y 'g_anim' ahora se crean en main()

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

    double simTime = 60.0;
    uint32_t nSensors = 9, nClusterHeads = 3;
    uint32_t nRecolector = 1; 
    bool useLeaderSignalPower = false; 

    CommandLine cmd;
    cmd.AddValue("useLeaderSignalPower", "Activa mayor potencia en los líderes de los clusters", useLeaderSignalPower);
    cmd.AddValue("nRecolector", "Número de nodos recolector (drones)", nRecolector);
    cmd.AddValue("simTime", "Duración de la simulación (s)", simTime);
    cmd.Parse(argc, argv);

    NodeContainer sensors, clusterHeads, recolector, superCluster;
    sensors.Create(nSensors);
    clusterHeads.Create(nClusterHeads);
    recolector.Create(nRecolector);
    superCluster.Create(1);

    // Crear el objeto de animación aquí, al inicio de main()
    AnimationInterface anim("manet_recolector.xml");
    g_anim = &anim; // Asignar el puntero global AHORA

    WifiHelper wifi;
    WifiMacHelper wifiMac;
    YansWifiPhyHelper wifiPhy;
    ConfigureWifiAdhoc(wifi, wifiMac, wifiPhy);

    NetDeviceContainer sensorDevices, headDevices, superDevices, recolectorDevices;
    InstallWifiDevices(wifi, wifiMac, wifiPhy, sensors, clusterHeads, superCluster, recolector,
                       sensorDevices, headDevices, superDevices, recolectorDevices,
                       useLeaderSignalPower);

    Vector A(0.0, 0.0, 0.0), B(100.0, 0.0, 0.0), C(50.0, 86.6025403784, 0.0);
    Vector centroid((A.x + B.x + C.x) / 3.0, (A.y + B.y + C.y) / 3.0, 0.0);
    
    // Almacenar la ruta de patrulla
    std::vector<Vector> patrolRoute = {A, B, C};
    double patrolSpeed = 10.0; // m/s
    double alarmSpeed = 20.0;  // m/s

    ConfigureClusterMobility(sensors, A, B, C);
    MobilityHelper mobility;
    ConfigureClusterHeadsAndSuper(clusterHeads, superCluster, mobility, A, B, C, centroid);
    std::vector<Ptr<ConstantVelocityMobilityModel>> recolectorMVs = ConfigureRecolectorMobilities(recolector, mobility, A, B, C);

    InstallAodvAndStack(sensors, clusterHeads, superCluster, recolector);

    Ipv4AddressHelper address;
    Ipv4InterfaceContainer interfaces = AssignIpAddresses(address, sensorDevices, headDevices, superDevices, recolectorDevices);

    std::vector<Ipv4Address> recolectorAddrs;
    uint32_t recolectorBaseIndex = nSensors + nClusterHeads + superCluster.GetN();
    for (uint32_t i = 0; i < nRecolector; ++i) {
        Ipv4Address addr = interfaces.GetAddress(recolectorBaseIndex + i);
        recolectorAddrs.push_back(addr);
    }
    Ipv4Address superAddr = interfaces.GetAddress(nSensors + nClusterHeads);
    SetupUdpApplications(clusterHeads, superCluster, recolector, recolectorAddrs, superAddr, simTime);

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
    for (uint32_t i = 0; i < recolector.GetN(); ++i)
    {
        anim.UpdateNodeColor(recolector.Get(i), 0, 0, 255); // Drones = Azul
    }
    anim.UpdateNodeColor(superCluster.Get(0), 0, 0, 255); // Super = Azul
    // -----------------------------------------------------------------


    // -----------------------------------------------------------------
    // 🔹 LÓGICA DE INICIO DE MOVIMIENTO 🔹
    // -----------------------------------------------------------------
    for (uint32_t i = 0; i < recolectorMVs.size(); ++i) {
        Ptr<ConstantVelocityMobilityModel> mv = recolectorMVs[i];
        
        // Inicia el bucle de patrulla
        StartPatrolling(mv, patrolRoute, patrolSpeed);
        
        // Programa el retorno al centroide al final (usando la función corregida)
        SetupReturnToSuper(mv, centroid, simTime, 5.0);
    }

    // -----------------------------------------------------------------
    // 🔹 LÓGICA DE INICIO DE ALARMA 🔹
    // -----------------------------------------------------------------
    // Programamos la *primera* alarma en un tiempo aleatorio
    double firstAlarmTime = g_rand->GetValue(5.0, 15.0);
    Simulator::Schedule(Seconds(firstAlarmTime), &TriggerRandomAlarm,
                        sensors, recolector, recolectorMVs,
                        patrolRoute, patrolSpeed, alarmSpeed, simTime);
    
    RunSimulation(simTime);

    return 0;
}