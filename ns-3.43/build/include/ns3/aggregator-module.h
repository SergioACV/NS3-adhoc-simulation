#ifdef NS3_MODULE_COMPILATION 
    error "Do not include ns3 module aggregator headers from other modules these are meant only for end user scripts." 
#endif 
#ifndef NS3_MODULE_AGGREGATOR
    // Module headers: 
    #include <ns3/aggregator.h>
    #include <ns3/aggregator-helper.h>
#endif 