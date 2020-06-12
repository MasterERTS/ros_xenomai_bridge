#include "xddp_ros.h"

class ChatterXDDP : public MinimalPublisher
{
    public: 
        ChatterXDDP();
        void nrt_thread();
    private:
        static const char *msg[];
};