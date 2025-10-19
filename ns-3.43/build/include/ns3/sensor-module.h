#ifdef NS3_MODULE_COMPILATION 
    error "Do not include ns3 module aggregator headers from other modules these are meant only for end user scripts." 
#endif 
#ifndef NS3_MODULE_SENSOR
    // Module headers: 
    #include <ns3/sensor-netdevice.h>
    #include <ns3/sensor-mac.h>
    #include <ns3/batsen-mac.h>
    #include <ns3/batsen-mac-header.h>
    #include <ns3/leach-mac.h>
    #include <ns3/sensor-csma.h>
    #include <ns3/leach-mac-header.h>
    #include <ns3/mac-trailer.h>
    #include <ns3/sensor-helper.h>
#endif 