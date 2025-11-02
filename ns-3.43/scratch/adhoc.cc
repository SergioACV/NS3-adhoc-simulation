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

//Self-made modules
#include "ns3/multi-zone-propagation-loss-model-helper.h"

#include <cmath>
#include <vector>

using namespace ns3;

static Ptr<MultiZonePropagationLossModel> g_multiZoneModel = 0;

NS_LOG_COMPONENT_DEFINE("ManetRecolector");

// ---------------------------
// 🔹 Declaración de funciones
// ---------------------------
Ptr<YansWifiChannel> CreateMultiZoneChannel();
void ConfigureWifiAdhoc(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy);
void InstallWifiDevices(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy,
                        NodeContainer &sensors, NodeContainer &clusterHeads,
                        NodeContainer &superCluster, NodeContainer &recolector,
                        NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                        NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices,
                        bool useLeaderSignalPower); // [NUEVO]

void ConfigureClusterMobility(NodeContainer &sensors, Vector A, Vector B, Vector C);
void ConfigureClusterHeadsAndSuper(NodeContainer &clusterHeads, NodeContainer &superCluster, MobilityHelper &mobility,
                                   Vector A, Vector B, Vector C, Vector centroid);
// Nota: ahora recibe A,B,C para distribuir inicios
std::vector<Ptr<ConstantVelocityMobilityModel>> ConfigureRecolectorMobilities(NodeContainer &recolector, MobilityHelper &mobility, Vector A, Vector B, Vector C); // [MODIFICADA]

void InstallAodvAndStack(NodeContainer &sensors, NodeContainer &clusterHeads,
                         NodeContainer &superCluster, NodeContainer &recolector);

Ipv4InterfaceContainer AssignIpAddresses(Ipv4AddressHelper &address,
                                         NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                                         NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices);

void SetupUdpApplications(NodeContainer &clusterHeads, NodeContainer &superCluster, NodeContainer &recolector,
                          const std::vector<Ipv4Address> &recolectorAddrs, Ipv4Address superAddr, double simTime);

// ScheduleRecolectorMovement ahora recibe simEnd para repetir hasta el final
void ScheduleRecolectorMovement(Ptr<ConstantVelocityMobilityModel> mv,
                                Vector A, Vector B, Vector C, double speed, double simEnd);

void RunSimulation(double simTime);

static void ReturnToSuperCallback(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simEnd);
static void SetupReturnToSuper(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simEnd, double leadTime = 5.0);

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
    multiZone->AddCircularZone(0.0, 0.0, 40.0, 40.0, 20.0);

    // Cluster B - zona intermedia
    multiZone->AddCircularZone(100.0, 0.0, 40.0, 60.0, 25.0);

    // Cluster C - zona densa con árboles
    multiZone->AddCircularZone(50.0, 86.6025403784, 40.0, 80.0, 30.0);

    // Zona por defecto (fuera de los clusters)
    multiZone->SetDefaultZone(100.0, 35.0);

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
                        NodeContainer &superCluster, NodeContainer &recolector,
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
    recolectorDevices = wifi.Install(wifiPhy, wifiMac, recolector);

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
std::vector<Ptr<ConstantVelocityMobilityModel>> ConfigureRecolectorMobilities(NodeContainer &recolector, MobilityHelper &mobility, Vector A, Vector B, Vector C)
{
    std::vector<Ptr<ConstantVelocityMobilityModel>> movers;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(recolector);

    // Bases de inicio: A, B, C (cíclico si hay más drones)
    std::vector<Vector> bases = {A, B, C};

    for (uint32_t i = 0; i < recolector.GetN(); ++i) {
        Ptr<Node> n = recolector.Get(i);
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

    // Instalar UdpServer en cada recolector
    for (uint32_t r = 0; r < recolector.GetN(); ++r) {
        UdpServerHelper server(port);
        ApplicationContainer serverApp = server.Install(recolector.Get(r));
        serverApp.Start(Seconds(1.0));
        serverApp.Stop(Seconds(simTime));
        NS_LOG_INFO("Server UDP instalado en recolector " << r << " addr=" << recolectorAddrs[r]);
    }

    // Super cluster server (coordinación)
    uint16_t coordPort = 6000;
    UdpServerHelper coordServer(coordPort);
    ApplicationContainer coordServerApp = coordServer.Install(superCluster.Get(0));
    coordServerApp.Start(Seconds(1.0));
    coordServerApp.Stop(Seconds(simTime));

    ApplicationContainer clientApps, coordClientApps;

    // Para cada cluster head, elegir recolector por round-robin: head i -> recolector (i % nRecolectores)
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
        NS_LOG_INFO("ClusterHead " << i << " enviará a recolector index " << chosen << " addr=" << recolectorAddrs[chosen]);

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

void ScheduleRecolectorMovement(Ptr<ConstantVelocityMobilityModel> mv,
                                Vector A, Vector B, Vector C, double speed, double simEnd)
{
    auto computeVelocity = [speed](const Vector &from, const Vector &to) {
        Vector dir = to - from;
        double len = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
        if (len == 0)
            return Vector(0, 0, 0);
        return Vector(dir.x / len * speed, dir.y / len * speed, dir.z / len * speed);
    };

    // posición inicial del dron
    Vector start = mv->GetPosition();
    std::vector<Vector> route;

    // elegimos la secuencia rotada según punto de inicio (tolerancia 1.0 m)
    if (std::abs(start.x - A.x) < 1.0 && std::abs(start.y - A.y) < 1.0)
        route = {B, C, A};
    else if (std::abs(start.x - B.x) < 1.0 && std::abs(start.y - B.y) < 1.0)
        route = {C, A, B};
    else
        route = {A, B, C};

    // calcular y agendar secuencia inicial (desde 'start' hacia route[0], route[1], ...)
    double t = 0.0;
    Vector from = start;
    std::vector<double> legTimes;
    for (size_t i = 0; i < route.size(); ++i) {
        Vector to = route[i];
        double dist = std::sqrt((to.x - from.x)*(to.x - from.x) + (to.y - from.y)*(to.y - from.y));
        double travelTime = (speed > 0.0) ? (dist / speed) : std::numeric_limits<double>::infinity();
        legTimes.push_back(travelTime);
        Vector v = computeVelocity(from, to);
        Simulator::Schedule(Seconds(t), &ConstantVelocityMobilityModel::SetVelocity, mv, v);
        Simulator::Schedule(Seconds(t + travelTime), &ConstantVelocityMobilityModel::SetPosition, mv, to);
        t += travelTime;
        from = to;
    }

    double cycleTime = t;
    if (cycleTime <= 0.0) cycleTime = 1.0; // seguridad

    // Repetir el ciclo hasta cubrir simEnd (agendando a partir de 't')
    while (t < simEnd) {
        // cada ciclo recorre route in order; la 'from' para el primer leg del ciclo es route.back()
        for (size_t i = 0; i < route.size() && t < simEnd; ++i) {
            Vector prev = (i == 0) ? route.back() : route[i - 1];
            Vector to = route[i];
            double dist = std::sqrt((to.x - prev.x)*(to.x - prev.x) + (to.y - prev.y)*(to.y - prev.y));
            double travelTime = (speed > 0.0) ? (dist / speed) : std::numeric_limits<double>::infinity();
            Vector v = computeVelocity(prev, to);
            Simulator::Schedule(Seconds(t), &ConstantVelocityMobilityModel::SetVelocity, mv, v);
            Simulator::Schedule(Seconds(t + travelTime), &ConstantVelocityMobilityModel::SetPosition, mv, to);
            t += travelTime;
        }
    }
}

// Calcula y aplica velocidad para que mv llegue a centroid justo al tiempo simEnd.
// Si startLead > 0, programa el cálculo en simEnd - startLead; si startLead == 0 calcula ahora.
static void
ReturnToSuperCallback(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simEnd)
{
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
    Simulator::Schedule(Seconds(start), &ReturnToSuperCallback, mv, centroid, simEnd);
    Simulator::Schedule(Seconds(simEnd), &ConstantVelocityMobilityModel::SetPosition, mv, centroid);
    Simulator::Schedule(Seconds(simEnd), &ConstantVelocityMobilityModel::SetVelocity, mv, Vector(0,0,0));
}

void RunSimulation(double simTime)
{
    AnimationInterface anim("manet_recolector.xml");

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
    NS_LOG_INFO("Simulación iniciando...");

    double simTime = 60.0;
    uint32_t nSensors = 9, nClusterHeads = 3;
    uint32_t nRecolector = 1; // [NUEVO] número de drones por CLI
    bool useLeaderSignalPower = false; // [NUEVO]

    // [NUEVO] Activable desde línea de comandos
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

    WifiHelper wifi;
    WifiMacHelper wifiMac;
    YansWifiPhyHelper wifiPhy;
    ConfigureWifiAdhoc(wifi, wifiMac, wifiPhy);

    NetDeviceContainer sensorDevices, headDevices, superDevices, recolectorDevices;
    InstallWifiDevices(wifi, wifiMac, wifiPhy, sensors, clusterHeads, superCluster, recolector,
                       sensorDevices, headDevices, superDevices, recolectorDevices,
                       useLeaderSignalPower); // [NUEVO]

    Vector A(0.0, 0.0, 0.0), B(100.0, 0.0, 0.0), C(50.0, 86.6025403784, 0.0);
    Vector centroid((A.x + B.x + C.x) / 3.0, (A.y + B.y + C.y) / 3.0, 0.0);

    ConfigureClusterMobility(sensors, A, B, C);
    MobilityHelper mobility;
    ConfigureClusterHeadsAndSuper(clusterHeads, superCluster, mobility, A, B, C, centroid);

    // Configurar movilidad para múltiples recolectores y obtener sus modelos
    std::vector<Ptr<ConstantVelocityMobilityModel>> recolectorMVs = ConfigureRecolectorMobilities(recolector, mobility, A, B, C);

    InstallAodvAndStack(sensors, clusterHeads, superCluster, recolector);

    Ipv4AddressHelper address;
    Ipv4InterfaceContainer interfaces = AssignIpAddresses(address, sensorDevices, headDevices, superDevices, recolectorDevices);

    // Obtener direcciones IP de los recolectores
    std::vector<Ipv4Address> recolectorAddrs;
    uint32_t recolectorBaseIndex = nSensors + nClusterHeads + superCluster.GetN();
    for (uint32_t i = 0; i < nRecolector; ++i) {
        Ipv4Address addr = interfaces.GetAddress(recolectorBaseIndex + i);
        recolectorAddrs.push_back(addr);
        NS_LOG_INFO("Recolector " << i << " IP = " << addr);
    }

    // Dirección del super cluster (asumimos único)
    Ipv4Address superAddr = interfaces.GetAddress(nSensors + nClusterHeads);

    // Instalar aplicaciones UDP (servidores en cada recolector, clientes en cluster heads -> round-robin)
    SetupUdpApplications(clusterHeads, superCluster, recolector, recolectorAddrs, superAddr, simTime);

    

    // Programar movimiento de cada recolector (ruta rotada según inicio) y retorno
    for (uint32_t i = 0; i < recolectorMVs.size(); ++i) {
        Ptr<ConstantVelocityMobilityModel> mv = recolectorMVs[i];
        double speed = 10.0; // m/s
        ScheduleRecolectorMovement(mv, A, B, C, speed, simTime);
        // schedule return to centroid antes del fin de simulación
        SetupReturnToSuper(mv, centroid, simTime, 5.0);
    }

    RunSimulation(simTime);

    return 0;
}
