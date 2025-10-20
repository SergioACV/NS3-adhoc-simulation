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

#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ManetRecolector");

// ---------------------------
// 🔹 Declaración de funciones
// ---------------------------
void ConfigureWifiAdhoc(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy);
void InstallWifiDevices(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy,
                        NodeContainer &sensors, NodeContainer &clusterHeads,
                        NodeContainer &superCluster, NodeContainer &recolector,
                        NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                        NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices);

void ConfigureClusterMobility(NodeContainer &sensors, Vector A, Vector B, Vector C);
void ConfigureClusterHeadsAndSuper(NodeContainer &clusterHeads, NodeContainer &superCluster, MobilityHelper &mobility,
                                   Vector A, Vector B, Vector C, Vector centroid);
Ptr<ConstantVelocityMobilityModel> ConfigureRecolectorMobility(NodeContainer &recolector, MobilityHelper &mobility, Vector A);

void InstallAodvAndStack(NodeContainer &sensors, NodeContainer &clusterHeads,
                         NodeContainer &superCluster, NodeContainer &recolector);

Ipv4InterfaceContainer AssignIpAddresses(Ipv4AddressHelper &address,
                                         NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                                         NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices);

void SetupUdpApplications(NodeContainer &clusterHeads, NodeContainer &superCluster, NodeContainer &recolector,
                          Ipv4Address recolectorAddr, Ipv4Address superAddr, double simTime);

void ScheduleRecolectorMovement(Ptr<ConstantVelocityMobilityModel> mv,
                                Vector A, Vector B, Vector C, double speed);

void RunSimulation(double simTime);

// ---------------------------
// 🔸 Implementación
// ---------------------------

void ConfigureWifiAdhoc(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy)
{
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifiMac.SetType("ns3::AdhocWifiMac");
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
}

void InstallWifiDevices(WifiHelper &wifi, WifiMacHelper &wifiMac, YansWifiPhyHelper &wifiPhy,
                        NodeContainer &sensors, NodeContainer &clusterHeads,
                        NodeContainer &superCluster, NodeContainer &recolector,
                        NetDeviceContainer &sensorDevices, NetDeviceContainer &headDevices,
                        NetDeviceContainer &superDevices, NetDeviceContainer &recolectorDevices)
{
    sensorDevices = wifi.Install(wifiPhy, wifiMac, sensors);
    headDevices = wifi.Install(wifiPhy, wifiMac, clusterHeads);
    superDevices = wifi.Install(wifiPhy, wifiMac, superCluster);
    recolectorDevices = wifi.Install(wifiPhy, wifiMac, recolector);
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

Ptr<ConstantVelocityMobilityModel> ConfigureRecolectorMobility(NodeContainer &recolector, MobilityHelper &mobility, Vector A)
{
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(recolector);
    Ptr<ConstantVelocityMobilityModel> mv = recolector.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    mv->SetPosition(Vector(A.x, A.y, A.z));
    return mv;
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
                          Ipv4Address recolectorAddr, Ipv4Address superAddr, double simTime)
{
    uint16_t port = 5000;
    UdpServerHelper server(port);
    ApplicationContainer serverApp = server.Install(recolector.Get(0));
    serverApp.Start(Seconds(1.0));
    serverApp.Stop(Seconds(simTime));

    uint16_t coordPort = 6000;
    UdpServerHelper coordServer(coordPort);
    ApplicationContainer coordServerApp = coordServer.Install(superCluster.Get(0));
    coordServerApp.Start(Seconds(1.0));
    coordServerApp.Stop(Seconds(simTime));

    ApplicationContainer clientApps, coordClientApps;

    for (uint32_t i = 0; i < clusterHeads.GetN(); ++i)
    {
        UdpClientHelper clientToRecolector(recolectorAddr, port);
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

void ScheduleRecolectorMovement(Ptr<ConstantVelocityMobilityModel> mv,
                                Vector A, Vector B, Vector C, double speed)
{
    auto computeVelocity = [speed](const Vector &from, const Vector &to) {
        Vector dir = to - from;
        double len = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
        if (len == 0)
            return Vector(0, 0, 0);
        return Vector(dir.x / len * speed, dir.y / len * speed, dir.z / len * speed);
    };

    double edgeDist = std::sqrt((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));
    double edgeTime = edgeDist / speed;

    Vector vAB = computeVelocity(A, B);
    Vector vBC = computeVelocity(B, C);
    Vector vCA = computeVelocity(C, A);

    Simulator::Schedule(Seconds(0.0), &ConstantVelocityMobilityModel::SetVelocity, mv, vAB);
    Simulator::Schedule(Seconds(edgeTime), &ConstantVelocityMobilityModel::SetVelocity, mv, vBC);
    Simulator::Schedule(Seconds(2 * edgeTime), &ConstantVelocityMobilityModel::SetVelocity, mv, vCA);
    Simulator::Schedule(Seconds(3 * edgeTime), &ConstantVelocityMobilityModel::SetVelocity, mv, vAB);
    Simulator::Schedule(Seconds(4 * edgeTime), &ConstantVelocityMobilityModel::SetVelocity, mv, vBC);
    Simulator::Schedule(Seconds(5 * edgeTime), &ConstantVelocityMobilityModel::SetVelocity, mv, vCA);
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

// Public helper: main llama solo a esta función para programar el retorno.
// leadTime: segundos antes de simEnd en que comienza el retorno (default 5s).
static void
SetupReturnToSuper(Ptr<ConstantVelocityMobilityModel> mv, Vector centroid, double simEnd, double leadTime = 5.0)
{
    double start = simEnd - leadTime;
    if (start < 0.0) start = 0.0;
    // programar cálculo de velocidad en 'start' (o ahora si start==0)
    Simulator::Schedule(Seconds(start), &ReturnToSuperCallback, mv, centroid, simEnd);
    // asegurar posición y parada exacta al final
    Simulator::Schedule(Seconds(simEnd), &ConstantVelocityMobilityModel::SetPosition, mv, centroid);
    Simulator::Schedule(Seconds(simEnd), &ConstantVelocityMobilityModel::SetVelocity, mv, Vector(0,0,0));
}

void RunSimulation(double simTime)
{
    AnimationInterface anim("manet_recolector.xml");
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
    uint32_t nSensors = 9, nClusterHeads = 3, nRecolector = 1;

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
                       sensorDevices, headDevices, superDevices, recolectorDevices);

    Vector A(0.0, 0.0, 0.0), B(100.0, 0.0, 0.0), C(50.0, 86.6025403784, 0.0);
    Vector centroid((A.x + B.x + C.x) / 3.0, (A.y + B.y + C.y) / 3.0, 0.0);

    ConfigureClusterMobility(sensors, A, B, C);
    MobilityHelper mobility;
    ConfigureClusterHeadsAndSuper(clusterHeads, superCluster, mobility, A, B, C, centroid);
    Ptr<ConstantVelocityMobilityModel> mv = ConfigureRecolectorMobility(recolector, mobility, A);

    InstallAodvAndStack(sensors, clusterHeads, superCluster, recolector);

    Ipv4AddressHelper address;
    Ipv4InterfaceContainer interfaces = AssignIpAddresses(address, sensorDevices, headDevices, superDevices, recolectorDevices);

    Ipv4Address recolectorAddr = interfaces.GetAddress(nSensors + nClusterHeads + superCluster.GetN());
    Ipv4Address superAddr = interfaces.GetAddress(nSensors + nClusterHeads);

    SetupUdpApplications(clusterHeads, superCluster, recolector, recolectorAddr, superAddr, simTime);

    ScheduleRecolectorMovement(mv, A, B, C, 10.0);
    SetupReturnToSuper(mv, centroid, simTime, 5.0);
    RunSimulation(simTime);

    return 0;
}
