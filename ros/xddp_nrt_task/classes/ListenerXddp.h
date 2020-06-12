#include "xddp_ros.h"

class ListenerXDDP : public MinimalSubscriber
{
    public: 
        ListenerXDDP();
        void nrt_thread();
    private:
        static const char *msg[];
};