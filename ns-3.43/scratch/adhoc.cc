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


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ManetRecolector");

int main(int argc, char *argv[])
{
   

    NS_LOG_INFO("Simulación iniciando...");

    double simTime = 60.0; // segundos
    uint32_t nSensors = 8; // sensores totales (2 clústeres)
    uint32_t nClusterHeads = 2;
    uint32_t nRecolector = 1;

    NodeContainer sensors;
    sensors.Create(nSensors);

    NodeContainer clusterHeads;
    clusterHeads.Create(nClusterHeads);

    NodeContainer recolector;
    recolector.Create(nRecolector);

    // =====================
    // 1️⃣ Configuración Wi-Fi ad hoc
    // =====================
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    YansWifiPhyHelper wifiPhy ;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    NetDeviceContainer sensorDevices = wifi.Install(wifiPhy, wifiMac, sensors);
    NetDeviceContainer headDevices = wifi.Install(wifiPhy, wifiMac, clusterHeads);
    NetDeviceContainer recolectorDevices = wifi.Install(wifiPhy, wifiMac, recolector);

    // =====================
    // 2️⃣ Movilidad
    // =====================
    MobilityHelper mobility;

    // Sensores - estáticos
    Ptr<ListPositionAllocator> sensorPositions = CreateObject<ListPositionAllocator>();
    // Clúster 1 (x cerca de 0)
    sensorPositions->Add(Vector(0, 0, 0));
    sensorPositions->Add(Vector(10, 0, 0));
    sensorPositions->Add(Vector(5, 10, 0));
    sensorPositions->Add(Vector(8, 5, 0));
    // Clúster 2 (x cerca de 100)
    sensorPositions->Add(Vector(100, 0, 0));
    sensorPositions->Add(Vector(110, 5, 0));
    sensorPositions->Add(Vector(105, 10, 0));
    sensorPositions->Add(Vector(95, 5, 0));

    mobility.SetPositionAllocator(sensorPositions);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(sensors);

    // Cluster Heads (en el centro de cada grupo)
    Ptr<ListPositionAllocator> headPositions = CreateObject<ListPositionAllocator>();
    headPositions->Add(Vector(5, 5, 0));    // cluster 1
    headPositions->Add(Vector(105, 5, 0));  // cluster 2
    mobility.SetPositionAllocator(headPositions);
    mobility.Install(clusterHeads);

    // Recolector móvil (viaja entre los dos clusters)
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(recolector);
    recolector.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(0, 10, 0));
    recolector.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(2.0, 0, 0)); // mueve hacia el cluster 2

    // =====================
    // 3️⃣ Protocolo AODV
    // =====================
    AodvHelper aodv;
    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(sensors);
    stack.Install(clusterHeads);
    stack.Install(recolector);

    // =====================
    // 4️⃣ Direcciones IP
    // =====================
    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.255.0");

    NetDeviceContainer allDevices;
    allDevices.Add(sensorDevices);
    allDevices.Add(headDevices);
    allDevices.Add(recolectorDevices);

    Ipv4InterfaceContainer interfaces = address.Assign(allDevices);

    // =====================
    // 5️⃣ Aplicaciones UDP (simulan transferencia de datos)
    // =====================
    // Los cluster heads envían datos al recolector cuando está cerca
    uint16_t port = 5000;
    UdpServerHelper server(port);
    ApplicationContainer serverApp = server.Install(recolector.Get(0));
    serverApp.Start(Seconds(1.0));
    serverApp.Stop(Seconds(simTime));

    UdpClientHelper client(interfaces.GetAddress(nSensors), port);
    client.SetAttribute("MaxPackets", UintegerValue(1000));
    client.SetAttribute("Interval", TimeValue(Seconds(2.0)));
    client.SetAttribute("PacketSize", UintegerValue(512));

    ApplicationContainer clientApps;
    for (uint32_t i = 0; i < clusterHeads.GetN(); ++i)
    {
        UdpClientHelper client(interfaces.GetAddress(nSensors + i), port);
        clientApps.Add(client.Install(clusterHeads.Get(i)));
    }

    clientApps.Start(Seconds(5.0));
    clientApps.Stop(Seconds(simTime - 1));


     // =====================
    // 6️⃣ Animación y ejecución
    // =====================
    AnimationInterface anim("manet_recolector.xml");
    anim.SetConstantPosition(recolector.Get(0), 0, 10);
    

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}