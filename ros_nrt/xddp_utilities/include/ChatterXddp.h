#ifndef CHATTER_XDDP_H
#define CHATTER_XDDP_H

#include "MinimalPublisher.h"

class ChatterXDDP : public MinimalPublisher
{
    public: 
        ChatterXDDP(std::string topic);
        char* nrt_thread();
    private:
        static const char *msg[];

        static void fail(const char *reason);
        char buf[128], *devname;
	    int fd, ret;
};

#endif