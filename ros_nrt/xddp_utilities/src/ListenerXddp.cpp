#include "ListenerXddp.h"

ListenerXDDP::ListenerXDDP(std::string topic) : MinimalSubscriber { topic }
{
    // do nothing
}

void ListenerXDDP::nrt_thread_read_write() 
{
    // do nothing
}