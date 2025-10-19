#ifdef NS3_MODULE_COMPILATION 
    error "Do not include ns3 module aggregator headers from other modules these are meant only for end user scripts." 
#endif 
#ifndef NS3_MODULE_BATMAND
    // Module headers: 
    #include <ns3/batmand-routing-protocol.h>
    #include <ns3/batmand-structures.h>
    #include <ns3/batmand-header.h>
    #include <ns3/batmand-helper.h>
#endif 