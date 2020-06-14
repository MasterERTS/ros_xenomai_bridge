#ifndef CHATTER_XDDP_H
#define CHATTER_XDDP_H

#include "MinimalPublisher.h"

class ChatterXDDP : public MinimalPublisher
{
    public: 
        ChatterXDDP(std::string topic, unsigned int xddp_pipe);
        char* nrt_thread_read_write();
        char* nrt_thread_read();
        void nrt_thread_write(char buffer[128]);

    private:
        static const char *msg[];

        static void fail(const char *reason);
        char buf[128], *devname;
	    int fd, ret;
};

#endif