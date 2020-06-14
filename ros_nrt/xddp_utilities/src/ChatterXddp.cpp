#include "ChatterXddp.h"

ChatterXDDP::ChatterXDDP(std::string topic, unsigned int xddp_pipe) : MinimalPublisher { topic }
{
	this->req_n = 0;
	ROS_INFO("Reading through pipe rtp%d", xddp_pipe);
	if (asprintf(&(this->devname), "/dev/rtp%d", xddp_pipe) < 0)
		fail("asprintf");

	this->fd = open(devname, O_RDWR);
	free(this->devname);

	if (this->fd < 0) fail("open");

	ROS_INFO("Successfully opened pipe rtp%d", xddp_pipe);
}

void ChatterXDDP::fail(const char *reason)
{
	perror(reason);
	exit(EXIT_FAILURE);
}

char* ChatterXDDP::nrt_thread_read_write()
{
    /* Get the next message from realtime_thread. */
	this->ret = read(this->fd, this->buf, sizeof(this->buf));
	if (this->ret <= 0) this->fail("read");

	/* Echo the message back to realtime_thread. */
	this->ret = write(this->fd, this->buf, this->ret);
	if (this->ret <= 0) this->fail("write");

    return(this->buf);
}

char* ChatterXDDP::nrt_thread_read() 
{
    /* Get the next message from realtime_thread. */
	this->ret = read(this->fd, this->buf, sizeof(this->buf));
	if (this->ret <= 0) this->fail("read");

    return(this->buf);
}

void ChatterXDDP::nrt_thread_write(char buffer[256]) 
{
	if (this->req_n == 0) {
		/* Get the next message from realtime_thread. */
		this->ret = read(this->fd, this->buf, sizeof(this->buf));
		if (this->ret <= 0) this->fail("read");
		this->req_n++;
	}
	this->ret = write(this->fd, buffer, this->ret);
	if (this->ret <= 0) this->fail("write");
}