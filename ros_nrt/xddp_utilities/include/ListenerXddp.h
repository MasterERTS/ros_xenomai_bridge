#ifndef LISTENER_XDDP_H
#define LISTENER_XDDP_H

#include "MinimalSubscriber.h"

class ListenerXDDP : public MinimalSubscriber
{
    public: 
        ListenerXDDP(std::string topic);
        void nrt_thread_read_write();
    private:
        static const char *msg[];
};

#endif