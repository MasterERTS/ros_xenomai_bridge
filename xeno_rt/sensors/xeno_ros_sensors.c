#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <malloc.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <rtdk.h>
#include <rtdm/rtipc.h>

pthread_t rt1;
pthread_t rt2;

#define XDDP_PORT_LASER 0
#define XDDP_PORT_ODOM  1

static void fail(const char *reason)
{
        perror(reason);
        exit(EXIT_FAILURE);
}

static void *rt_laser_task(void *arg)
{
        struct sockaddr_ipc saddr;
        int ret, s, len;
        struct timespec ts;
        float r_dist = 0.0, f_dist = 0.0, l_dist = 0.0;
        size_t poolsz;
        char buf[64];
        /*
         * Get a datagram socket to bind to the RT endpoint. Each
         * endpoint is represented by a port number within the XDDP
         * protocol namespace.
         */
        s = socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
        if (s < 0) {
                perror("socket");
                exit(EXIT_FAILURE);
        }
        /*
         * Set a local 16k pool for the RT endpoint. Memory needed to
         * convey datagrams will be pulled from this pool, instead of
         * Xenomai's system pool.
         */
        poolsz = 16384; /* bytes */
        ret = setsockopt(s, SOL_XDDP, XDDP_POOLSZ,
                         &poolsz, sizeof(poolsz));
        if (ret)
                fail("setsockopt");
        /*
         * Bind the socket to the port, to setup a proxy to channel
         * traffic to/from the Linux domain.
         *
         * saddr.sipc_port specifies the port number to use.
         */
        memset(&saddr, 0, sizeof(saddr));
        saddr.sipc_family = AF_RTIPC;
        saddr.sipc_port = XDDP_PORT_LASER;
        ret = bind(s, (struct sockaddr *)&saddr, sizeof(saddr));
        if (ret)
                fail("bind");
        printf("LaserScan socket bounded...\r\n");

        len = strlen("start laser");
        ret = sendto(s, "start laser", len, 0, NULL, 0);
        if (ret != len)
                fail("sendto");
        rt_printf("%s: sent %d bytes, \"%.*s\"\n",
                  __FUNCTION__, ret, ret, "start laser");
        for (;;) {
                len = strlen("ack");
                ret = recvfrom(s, buf, 64, 0, NULL, 0);
                if (ret <= 0)
                        fail("recvfrom");
                sscanf(buf, "%f %f %f", &l_dist, &f_dist, &r_dist);
                
                rt_printf("%s   => echoed by peer : [%.2f %.2f %.2f]\n", __FUNCTION__, l_dist, f_dist, r_dist);
                
                ret = sendto(s, "ack", len, 0, NULL, 0);
                if (ret != len)
                        fail("sendto");
                rt_printf("%s: sent %d bytes, \"%.*s\"\n",
                          __FUNCTION__, ret, ret, "ack");
                /*
                 * We run in full real-time mode (i.e. primary mode),
                 * so we have to let the system breathe between two
                 * iterations.
                 */
                ts.tv_sec = 0;
                ts.tv_nsec = 500000000; /* 500 ms */
                clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
        }
        return NULL;
}


static void *rt_odom_task(void *arg)
{
        struct sockaddr_ipc saddr;
        int ret, s, len;
        struct timespec ts;
        float px = 0.0, py = 0.0, oz = 0.0;
        size_t poolsz;
        char buf[64];
        /*
         * Get a datagram socket to bind to the RT endpoint. Each
         * endpoint is represented by a port number within the XDDP
         * protocol namespace.
         */
        s = socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP);
        if (s < 0) {
                perror("socket");
                exit(EXIT_FAILURE);
        }
        /*
         * Set a local 16k pool for the RT endpoint. Memory needed to
         * convey datagrams will be pulled from this pool, instead of
         * Xenomai's system pool.
         */
        poolsz = 16384; /* bytes */
        ret = setsockopt(s, SOL_XDDP, XDDP_POOLSZ,
                         &poolsz, sizeof(poolsz));
        if (ret)
                fail("setsockopt");
        /*
         * Bind the socket to the port, to setup a proxy to channel
         * traffic to/from the Linux domain.
         *
         * saddr.sipc_port specifies the port number to use.
         */
        memset(&saddr, 0, sizeof(saddr));
        saddr.sipc_family = AF_RTIPC;
        saddr.sipc_port = XDDP_PORT_ODOM;
        ret = bind(s, (struct sockaddr *)&saddr, sizeof(saddr));
        if (ret)
                fail("bind");
        printf("Odom socket bounded...\r\n");

        len = strlen("start odom");
        ret = sendto(s, "start odom", len, 0, NULL, 0);
        if (ret != len)
                fail("sendto");
        rt_printf("%s: sent %d bytes, \"%.*s\"\n",
                  __FUNCTION__, ret, ret, "start odom");
        for (;;) {
                len = strlen("ack");
                ret = recvfrom(s, buf, 64, 0, NULL, 0);
                if (ret <= 0)
                        fail("recvfrom");
                sscanf(buf, "%f %f %f", &px, &py, &oz);
                int rcvlen = strlen(buf);
                rt_printf("%s   => echoed by peer : [%.2f %.2f %.2f]\n", __FUNCTION__, px, py, oz);
                
                ret = sendto(s, "ack", len, 0, NULL, 0);
                if (ret != len)
                        fail("sendto");
                rt_printf("%s: sent %d bytes, \"%.*s\"\n",
                          __FUNCTION__, ret, ret, "ack");
                /*
                 * We run in full real-time mode (i.e. primary mode),
                 * so we have to let the system breathe between two
                 * iterations.
                 */
                ts.tv_sec = 0;
                ts.tv_nsec = 500000000; /* 500 ms */
                clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
        }
        return NULL;
}

static void cleanup_upon_sig(int sig)
{
        pthread_cancel(rt1);
        pthread_cancel(rt2);
        signal(sig, SIG_DFL);
        pthread_join(rt1, NULL);
        pthread_join(rt2, NULL);
}
int main(int argc, char **argv)
{
        printf("ROS Sensors : nrt/rt data exchange application !\r\n");
        struct sched_param rtparam = { .sched_priority = 42 };
        pthread_attr_t rtattr;
        sigset_t mask, oldmask;
        mlockall(MCL_CURRENT | MCL_FUTURE);
        sigemptyset(&mask);
        sigaddset(&mask, SIGINT);
        signal(SIGINT, cleanup_upon_sig);
        sigaddset(&mask, SIGTERM);
        signal(SIGTERM, cleanup_upon_sig);
        sigaddset(&mask, SIGHUP);
        signal(SIGHUP, cleanup_upon_sig);
        pthread_sigmask(SIG_BLOCK, &mask, &oldmask);

        rt_print_auto_init(1);
        pthread_attr_init(&rtattr);
        pthread_attr_setdetachstate(&rtattr, PTHREAD_CREATE_JOINABLE);
        pthread_attr_setinheritsched(&rtattr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&rtattr, SCHED_FIFO);
        pthread_attr_setschedparam(&rtattr, &rtparam);
        errno = pthread_create(&rt1, &rtattr, &rt_laser_task, NULL);
        if (errno)
                fail("pthread_create rt1");
        
        errno = pthread_create(&rt2, &rtattr, &rt_odom_task, NULL);
        if (errno)
                fail("pthread_create rt2");
        
        sigsuspend(&oldmask);
        return 0;
}