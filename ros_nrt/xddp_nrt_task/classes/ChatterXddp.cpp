#include "ChatterXddp.h"

ChatterXDDP::ChatterXDDP(std::string topic) : MinimalPublisher { topic }
{

	if (asprintf(&(this->devname), "/dev/rtp%d", PIPE_XDDP) < 0)
		fail("asprintf");

	this->fd = open(devname, O_RDWR);
	free(this->devname);

	if (this->fd < 0) fail("open");
}

void ChatterXDDP::fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}

char* ChatterXDDP::nrt_thread() 
{
    /* Get the next message from realtime_thread. */
	this->ret = read(this->fd, this->buf, sizeof(this->buf));
	if (this->ret <= 0) this->fail("read");

	/* Echo the message back to realtime_thread. */
	this->ret = write(this->fd, this->buf, this->ret);
	if (this->ret <= 0) this->fail("write");

    return(this->buf);
}