#ifdef NS3_MODULE_COMPILATION 
    error "Do not include ns3 module aggregator headers from other modules these are meant only for end user scripts." 
#endif 
#ifndef NS3_MODULE_STORE_CARRY_FORWARD
    // Module headers: 
    #include <ns3/store-carry-forward.h>
#endif 