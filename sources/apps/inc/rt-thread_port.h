#ifndef __RT_THREAD_PORT_H__
#define __RT_THREAD_PORT_H__

#include <sal_type.h>
#include <sal_ipaddr.h>
#include <sal_socket.h>
#include <sal_netdb.h>

struct timeval {
  long    tv_sec;         /* seconds */
  long    tv_usec;        /* and microseconds */
};

#endif
