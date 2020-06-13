#include "xddp_ros.h"

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