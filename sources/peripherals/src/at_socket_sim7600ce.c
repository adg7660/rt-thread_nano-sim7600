/*
 * File      : at_socket_esp8266.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-06-20     chenyong     first version
 */

#include <at.h>
#include <stdio.h>
#include <string.h>

#include <rtthread.h>
//#include <sys/socket.h>

#include <at_socket.h>
#include "board.h"

#if !defined(AT_SW_VERSION_NUM) || AT_SW_VERSION_NUM < 0x10200
#error "This AT Client version is older, please check and update latest AT Client!"
#endif

#define LOG_TAG              "at.sim7600ce"
#include <at_log.h>

#ifdef AT_DEVICE_SIM7600CE

#define SIM7600CE_MODULE_SEND_MAX_SIZE   2048
#define SIM7600CE_WAIT_CONNECT_TIME      15000
#define SIM7600CE_THREAD_STACK_SIZE      1024
#define SIM7600CE_THREAD_PRIORITY        (RT_THREAD_PRIORITY_MAX/2)

/* set real event by current socket and current state */
#define SET_EVENT(socket, event)       (((socket + 1) << 16) | (event))

/* AT socket event type */
#define SIM7600CE_EVENT_CONN_OK          (1L << 0)
#define SIM7600CE_EVENT_SEND_OK          (1L << 1)
#define SIM7600CE_EVENT_RECV_OK          (1L << 2)
#define SIM7600CE_EVNET_CLOSE_OK         (1L << 3)
#define SIM7600CE_EVENT_CONN_FAIL        (1L << 4)
#define SIM7600CE_EVENT_SEND_FAIL        (1L << 5)
#define SIM7600CE_EVENT_DOMAIN_OK        (1L << 6)

/* AT+QICSGP command default*/
char *QICSGP_CHINA_MOBILE = "AT+QICSGP=1,1,\"CMNET\",\"\",\"\",0";
char *QICSGP_CHINA_UNICOM = "AT+QICSGP=1,1,\"UNINET\",\"\",\"\",0";
char *QICSGP_CHINA_TELECOM = "AT+QICSGP=1,1,\"CTNET\",\"\",\"\",0";
static int cur_socket;
static int cur_send_bfsz;
static rt_event_t at_socket_event;
static rt_mutex_t at_event_lock;
static at_evt_cb_t at_evt_cb_set[] = {
        [AT_SOCKET_EVT_RECV] = NULL,
        [AT_SOCKET_EVT_CLOSED] = NULL,
};

static void at_tcp_ip_errcode_parse(int result)
{
    switch(result)
    {
        case 0   : LOG_D("%d : operation succeeded",                result); break;
        case 1   : LOG_E("%d : Network failure",                    result); break;
        case 2   : LOG_E("%d : Network not opened",                 result); break;
        case 3   : LOG_E("%d : Wrong parameter",                    result); break;
        case 4   : LOG_E("%d : Operation not supported",            result); break;
        case 5   : LOG_E("%d : Failed to create socket",            result); break;
        case 6   : LOG_E("%d : Failed to bind socket",              result); break;
        case 7   : LOG_E("%d : TCP server is already listening",    result); break;
        case 8   : LOG_E("%d : Busy",                               result); break;
        case 9   : LOG_E("%d : Sockets opened",                     result); break;
        case 10  : LOG_E("%d : Timeout",                            result); break;
        case 11  : LOG_E("%d : DNS parse failed for AT+CIPOPEN",    result); break;
        case 255 : LOG_E("%d : Unknown error",                      result); break;

        default  : LOG_E("%d : Unknown err code",                   result); break;
    }
}

static void at_socket_close_reason_parse(int result)
{
    switch(result)
    {
        case 0   : LOG_D("%d : Closed by local, active",            result); break;
        case 1   : LOG_E("%d : Closed by remote, passive",          result); break;
        case 2   : LOG_E("%d : Closed for sending timeout",         result); break;

        default  : LOG_E("%d : Unknown err code",                   result); break;
    }
}


static int at_socket_event_send(uint32_t event)
{
    return (int) rt_event_send(at_socket_event, event);
}

static int at_socket_event_recv(uint32_t event, uint32_t timeout, rt_uint8_t option)
{
    int result = 0;
    rt_uint32_t recved;

    result = rt_event_recv(at_socket_event, event, option | RT_EVENT_FLAG_CLEAR, timeout, &recved);
    if (result != RT_EOK)
    {
        return -RT_ETIMEOUT;
    }

    return recved;
}

/**
 * close socket by AT commands.
 *
 * @param current socket
 *
 * @return  0: close socket success
 *         -1: send AT commands error
 *         -2: wait socket event timeout
 *         -5: no memory
 */
static int sim7600ce_socket_close(int socket)
{
    int result = 0;

    rt_mutex_take(at_event_lock, RT_WAITING_FOREVER);

#ifdef ENABLE_4G_MODULE_SLEEP
    rt_pm_request(PM_SLEEP_MODE_SLEEP);
    sim7600ce_wkup();
#endif

    cur_socket = socket;

    if (at_exec_cmd(RT_NULL, "AT+CIPCLOSE=%d", socket) < 0)
    {
        result = -RT_ERROR;
        goto __exit;
    }

    if (at_socket_event_recv(SET_EVENT(socket, SIM7600CE_EVNET_CLOSE_OK), rt_tick_from_millisecond(300*3), RT_EVENT_FLAG_AND) < 0)
    {
        LOG_E("socket (%d) close failed, wait close OK timeout.", socket);
        result = -RT_ETIMEOUT;
        goto __exit;
    }

__exit:
#ifdef ENABLE_4G_MODULE_SLEEP
    sim7600ce_sleep();
    rt_pm_release(PM_SLEEP_MODE_SLEEP);
#endif

    rt_mutex_release(at_event_lock);

    return result;
}


/**
 * create TCP/UDP client or server connect by AT commands.
 *
 * @param socket current socket
 * @param ip server or client IP address
 * @param port server or client port
 * @param type connect socket type(tcp, udp)
 * @param is_client connection is client
 *
 * @return   0: connect success
 *          -1: connect failed, send commands error or type error
 *          -2: wait socket event timeout
 *          -5: no memory
 */
static int sim7600ce_socket_connect(int socket, char *ip, int32_t port, enum at_socket_type type, rt_bool_t is_client)
{
    int result = 0, event_result = 0;
    rt_bool_t retryed = RT_FALSE;

    RT_ASSERT(ip);
    RT_ASSERT(port >= 0);

    /* lock AT socket connect */
    rt_mutex_take(at_event_lock, RT_WAITING_FOREVER);

#ifdef ENABLE_4G_MODULE_SLEEP
    rt_pm_request(PM_SLEEP_MODE_SLEEP);
    sim7600ce_wkup();
#endif

    rt_kprintf("connect socket:%d, ip:%s, port:%d\r\n", socket, ip, port);

__retry:

    if (is_client)
    {
        switch (type)
        {
        case AT_SOCKET_TCP:
            /* send AT commands(eg: AT+CIPOPEN=0,"TCP","x.x.x.x", 1234) to connect TCP server */
            if (at_exec_cmd(RT_NULL, "AT+CIPOPEN=%d,\"TCP\",\"%s\",%d", socket, ip, port) < 0)
            {
                result = -RT_ERROR;
                goto __exit;
            }
            break;

        case AT_SOCKET_UDP:
            if (at_exec_cmd(RT_NULL, "AT+CIPOPEN=%d,\"UDP\",\"%s\",%d", socket, ip, port) < 0)
            {
                result = -RT_ERROR;
                goto __exit;
            }
            break;

        default:
            LOG_E("Not supported connect type : %d.", type);
            result = -RT_ERROR;
            goto __exit;
        }
    }

    /* waiting result event from AT URC, the device default connection timeout is 75 seconds, but it set to 10 seconds is convenient to use.*/
    if (at_socket_event_recv(SET_EVENT(socket, 0), rt_tick_from_millisecond(10 * 1000), RT_EVENT_FLAG_OR) < 0)
    {
        LOG_E("socket (%d) connect failed, wait connect result timeout.", socket);
        result = -RT_ETIMEOUT;
        goto __exit;
    }
    /* waiting OK or failed result */
    if ((event_result = at_socket_event_recv(SIM7600CE_EVENT_CONN_OK | SIM7600CE_EVENT_CONN_FAIL, rt_tick_from_millisecond(1 * 1000),
            RT_EVENT_FLAG_OR)) < 0)
    {
        LOG_E("socket (%d) connect failed, wait connect OK|FAIL timeout.", socket);
        result = -RT_ETIMEOUT;
        goto __exit;
    }
    /* check result */
    if (event_result & SIM7600CE_EVENT_CONN_FAIL)
    {
        if (!retryed)
        {
            LOG_E("socket (%d) connect failed, maybe the socket was not be closed at the last time and now will retry.", socket);
            if (sim7600ce_socket_close(socket) < 0)
            {
                goto __exit;
            }
            retryed = RT_TRUE;
            goto __retry;
        }
        LOG_E("socket (%d) connect failed, failed to establish a connection.", socket);
        result = -RT_ERROR;
        goto __exit;
    }

__exit:
#ifdef ENABLE_4G_MODULE_SLEEP
    sim7600ce_sleep();
    rt_pm_release(PM_SLEEP_MODE_SLEEP);
#endif

    /* unlock AT socket connect */
    rt_mutex_release(at_event_lock);

    return result;
}

#if 0
static int at_get_send_size(int socket, size_t *size, size_t *acked, size_t *nacked)
{
    at_response_t resp = at_create_resp(64, 0, rt_tick_from_millisecond(5000));
    int result = 0;

    if (!resp)
    {
        LOG_E("No memory for response structure!");
        result = -RT_ENOMEM;
        goto __exit;
    }

    if (at_exec_cmd(resp, "AT+CIPSEND=%d,0", socket) < 0)
    {
        result = -RT_ERROR;
        goto __exit;
    }

    if (at_resp_parse_line_args_by_kw(resp, "+CIPSEND:", "+CIPSEND: %d, %d, %d", size, acked, nacked) <= 0)
    {
        result = -RT_ERROR;
        goto __exit;
    }

__exit:
    if (resp)
    {
        at_delete_resp(resp);
    }

    return result;
}

static int at_wait_send_finish(int socket, size_t settings_size)
{
    /* get the timeout by the input data size */
    rt_tick_t timeout = rt_tick_from_millisecond(settings_size);
    rt_tick_t last_time = rt_tick_get();
    size_t size = 0, acked = 0, nacked = 0xFFFF;

    while (rt_tick_get() - last_time <= timeout)
    {
        at_get_send_size(socket, &size, &acked, &nacked);
        if (nacked == 0)
        {
            return RT_EOK;
        }
        rt_thread_delay(rt_tick_from_millisecond(50));
    }

    return -RT_ETIMEOUT;
}
#endif

/**
 * send data to server or client by AT commands.
 *
 * @param socket current socket
 * @param buff send buffer
 * @param bfsz send buffer size
 * @param type connect socket type(tcp, udp)
 *
 * @return >=0: the size of send success
 *          -1: send AT commands error or send data error
 *          -2: waited socket event timeout
 *          -5: no memory
 */
static int sim7600ce_socket_send(int socket, const char *buff, size_t bfsz, enum at_socket_type type)
{
    int result = RT_EOK;
    int event_result = 0;
    at_response_t resp = RT_NULL;
    size_t cur_pkt_size = 0, sent_size = 0;

    RT_ASSERT(buff);
    RT_ASSERT(bfsz > 0);

    resp = at_create_resp(128, 2, rt_tick_from_millisecond(5000));
    if (!resp)
    {
        LOG_E("No memory for response structure!");
        return -RT_ENOMEM;
    }

    rt_mutex_take(at_event_lock, RT_WAITING_FOREVER);

#ifdef ENABLE_4G_MODULE_SLEEP
    rt_pm_request(PM_SLEEP_MODE_SLEEP);
    sim7600ce_wkup();
#endif

//    LOG_D(">>>>>> ATSnd:%d\r\n", bfsz);
//    extern void dy_hex_print(void *p, int len);
//    dy_hex_print((void *)buff, bfsz);

    /* set current socket for send URC event */
    cur_socket = socket;
    /* set AT client end sign to deal with '>' sign.*/
    at_set_end_sign('>');

    while (sent_size < bfsz)
    {
        if (bfsz - sent_size < SIM7600CE_MODULE_SEND_MAX_SIZE)
        {
            cur_pkt_size = bfsz - sent_size;
        }
        else
        {
            cur_pkt_size = SIM7600CE_MODULE_SEND_MAX_SIZE;
        }

        /* send the "AT+CIPSEND" commands to AT server than receive the '>' response on the first line. */
        if (at_exec_cmd(resp, "AT+CIPSEND=%d,%d", socket, cur_pkt_size) < 0)
        {
            result = -RT_ERROR;
            goto __exit;
        }

        /* send the real data to server or client */
        result = (int) at_client_send(buff + sent_size, cur_pkt_size);
        if (result == 0)
        {
            result = -RT_ERROR;
            goto __exit;
        }

        /* waiting result event from AT URC */
        if (at_socket_event_recv(SET_EVENT(socket, 0), rt_tick_from_millisecond(300 * 3), RT_EVENT_FLAG_OR) < 0)
        {
            LOG_E("socket (%d) send failed, wait connect result timeout.", socket);
            result = -RT_ETIMEOUT;
            goto __exit;
        }
        /* waiting OK or failed result */
        if ((event_result = at_socket_event_recv(SIM7600CE_EVENT_SEND_OK | SIM7600CE_EVENT_SEND_FAIL, rt_tick_from_millisecond(1 * 1000),
                RT_EVENT_FLAG_OR)) < 0)
        {
            LOG_E("socket (%d) send failed, wait connect OK|FAIL timeout.", socket);
            result = -RT_ETIMEOUT;
            goto __exit;
        }
        /* check result */
        if (event_result & SIM7600CE_EVENT_SEND_FAIL)
        {
            LOG_E("socket (%d) send failed, return failed.", socket);
            result = -RT_ERROR;
            goto __exit;
        }

        if (type == AT_SOCKET_TCP)
        {
            cur_pkt_size = cur_send_bfsz;
			//at_wait_send_finish(socket, cur_pkt_size);
        }

        sent_size += cur_pkt_size;
    }

__exit:
#ifdef ENABLE_4G_MODULE_SLEEP
    sim7600ce_sleep();
    rt_pm_release(PM_SLEEP_MODE_SLEEP);
#endif

    /* reset the end sign for data conflict */
    at_set_end_sign(0);

    rt_mutex_release(at_event_lock);

    if (resp)
    {
        at_delete_resp(resp);
    }

    return result;
}

/**
 * domain resolve by AT commands.
 *
 * @param name domain name
 * @param ip parsed IP address, it's length must be 16
 *
 * @return  0: domain resolve success
 *         -2: wait socket event timeout
 *         -5: no memory
 */
static int sim7600ce_domain_resolve(const char *name, char ip[16])
{
#define RESOLVE_RETRY        5

    int i, result = RT_EOK;
    char recv_ip[16] = { 0 };
    at_response_t resp = RT_NULL;

    RT_ASSERT(name);
    RT_ASSERT(ip);

    /* The maximum response time is 14 seconds, affected by network status */
    resp = at_create_resp(128, 4, rt_tick_from_millisecond(14 * 1000));
    if (!resp)
    {
        LOG_E("No memory for response structure!");
        return -RT_ENOMEM;
    }

    rt_mutex_take(at_event_lock, RT_WAITING_FOREVER);

#ifdef ENABLE_4G_MODULE_SLEEP
    rt_pm_request(PM_SLEEP_MODE_SLEEP);
    sim7600ce_wkup();
#endif

    for(i = 0; i < RESOLVE_RETRY; i++)
    {
        if (at_exec_cmd(resp, "AT+CDNSGIP=\"%s\"", name) < 0)
        {
            result = -RT_ERROR;
            goto __exit;
        }

//        for (uint8_t j = 0; j < (int) resp->line_counts; j++)
//        {
//            LOG_D("###%s", at_resp_get_line(resp, j + 1));
//        }
#if 1
        if(at_resp_parse_line_args_by_kw(resp, "+CDNSGIP:", "+CDNSGIP: 1,%*[^,],\"%[^\"]", recv_ip) < 0)
        {
            rt_thread_delay(rt_tick_from_millisecond(100));
            /* resolve failed, maybe receive an URC CRLF */
            continue;
        }
#else
        char temp_buf[32];
        /* parse the third line of response data, get the IP address */
        if(at_resp_parse_line_args_by_kw(resp, "+CDNSGIP:", "+CDNSGIP: 1,%s", temp_buf) < 0)
        {
            rt_thread_delay(rt_tick_from_millisecond(100));
            /* resolve failed, maybe receive an URC CRLF */
            continue;
        }
        rt_kprintf("temp_buf:'%s'\n", temp_buf);

        char *p_str = NULL;
        if((p_str = strchr(temp_buf, ','))!= NULL)
        {
            rt_kprintf("p_str='%s'\n", p_str);
            if(strchr(p_str+2, '"') != NULL)
            {
                strncpy(recv_ip, p_str+2, strlen(p_str)-3);
            }
        }
#endif
//        rt_kprintf("recv_ip:'%s'\n", recv_ip);
        if (strlen(recv_ip) < 8)
        {
            rt_thread_delay(rt_tick_from_millisecond(100));
            /* resolve failed, maybe receive an URC CRLF */
            continue;
        }
        else
        {
            strncpy(ip, recv_ip, 15);
            ip[15] = '\0';
            break;
        }
    }

    if(i == RESOLVE_RETRY)
    {
        rt_kprintf("domain resolve fail\n");
    }

__exit:
#ifdef ENABLE_4G_MODULE_SLEEP
    sim7600ce_sleep();
    rt_pm_release(PM_SLEEP_MODE_SLEEP);
#endif

    rt_mutex_release(at_event_lock);

    if (resp)
    {
        at_delete_resp(resp);
    }

    return result;

}

/**
 * set AT socket event notice callback
 *
 * @param event notice event
 * @param cb notice callback
 */
static void sim7600ce_socket_set_event_cb(at_socket_evt_t event, at_evt_cb_t cb)
{
    if (event < sizeof(at_evt_cb_set) / sizeof(at_evt_cb_set[1]))
    {
        at_evt_cb_set[event] = cb;
    }
}

static void urc_connect_func(const char *data, rt_size_t size)
{
    int socket = 0;

    RT_ASSERT(data && size);

    sscanf(data, "%d%*[^0-9]", &socket);

    rt_kprintf("-->urc_connect_func:'%s', socket:%d\n", data, socket);
    if (strstr(data, "+CIPOPEN"))
    {
        at_socket_event_send(SET_EVENT(socket, SIM7600CE_EVENT_CONN_OK));
    }
    else
    {
        at_socket_event_send(SET_EVENT(socket, SIM7600CE_EVENT_CONN_FAIL));
    }
}

static void urc_send_func(const char *data, rt_size_t size)
{
    RT_ASSERT(data && size);

//    if (strstr(data, "SEND OK"))
//    {
        //rt_kprintf("--> urc_send_func:'%s'\n", data);

        int send_bfsz = 0;
        int socket = 0;
        int req_send_size = 0;
        sscanf(data, "+CIPSEND: %d,%d,%d", &socket, &req_send_size, &send_bfsz);
        //rt_kprintf("--> urc_send_bfsz_func:socket=%d, req_size=%d, real_size=%d\n", socket, req_send_size, send_bfsz);

        cur_send_bfsz = send_bfsz;

        at_socket_event_send(SET_EVENT(socket, SIM7600CE_EVENT_SEND_OK));
//    }
//    else if (strstr(data, "SEND FAIL"))
//    {
//        at_socket_event_send(SET_EVENT(cur_socket, SIM7600CE_EVENT_SEND_FAIL));
//    }
}

static void urc_close_func(const char *data, rt_size_t size)
{
    int socket = 0;
    int reason = 0;
    int err = 0;

    RT_ASSERT(data && size);

    rt_kprintf("at socket is disconnect!!!\n");

    if (strstr(data, "+CIPCLOSE"))
    {
        sscanf(data, "+CIPCLOSE: %d,%d", &socket, &err);
        at_tcp_ip_errcode_parse(err);
        at_socket_event_send(SET_EVENT(socket, SIM7600CE_EVNET_CLOSE_OK));
    }
    else if (strstr(data, "+IPCLOSE"))
    {
        sscanf(data, "+IPCLOSE: %d,%d", &socket, &reason);
        at_socket_close_reason_parse(reason);

        /* notice the socket is disconnect by remote */
        if (at_evt_cb_set[AT_SOCKET_EVT_CLOSED])
        {
            at_evt_cb_set[AT_SOCKET_EVT_CLOSED](socket, AT_SOCKET_EVT_CLOSED, NULL, 0);
        }
    }
}

static void urc_recv_func(const char *data, rt_size_t size)
{
    int socket = 0;
    rt_size_t bfsz = 0, temp_size = 0;
    rt_int32_t timeout;
    char *recv_buf = RT_NULL, temp[8];

    RT_ASSERT(data && size);

    rt_kprintf("recv:'%s'\n", data);

    /* get the current socket and receive buffer size by receive data */
    sscanf(data, "+RECEIVE,%d,%d", &socket, (int *) &bfsz);
    /* get receive timeout by receive buffer length */
    timeout = bfsz;

    if (socket < 0 || bfsz == 0)
        return;

    recv_buf = rt_calloc(1, bfsz);
    if (!recv_buf)
    {
        LOG_E("no memory for URC receive buffer (%d)!", bfsz);
        /* read and clean the coming data */
        while (temp_size < bfsz)
        {
            if (bfsz - temp_size > sizeof(temp))
            {
                at_client_recv(temp, sizeof(temp), timeout);
            }
            else
            {
                at_client_recv(temp, bfsz - temp_size, timeout);
            }
            temp_size += sizeof(temp);
        }
        return;
    }

    /* sync receive data */
    if (at_client_recv(recv_buf, bfsz, timeout) != bfsz)
    {
        LOG_E("receive size(%d) data failed!", bfsz);
        rt_free(recv_buf);
        return;
    }

    /* notice the receive buffer and buffer size */
    if (at_evt_cb_set[AT_SOCKET_EVT_RECV])
    {
        at_evt_cb_set[AT_SOCKET_EVT_RECV](socket, AT_SOCKET_EVT_RECV, recv_buf, bfsz);
    }
}

static void urc_ping_func(const char *data, rt_size_t size)
{
    static int icmp_seq = 0;
    int i, j = 0;
    int result, recv_len, time, ttl;
    int sent, rcvd, lost, min, max, avg;
    char dst_ip[16] = { 0 };

    RT_ASSERT(data);

    for (i=0;i<size;i++)
    {
        if(*(data+i) == '.')
            j++;
    }
    if (j != 0)
    {
        sscanf(data, "+CPING: %d,%[^,],%d,%d,%d", &result, dst_ip, &recv_len, &time, &ttl);
        if (result == 1)
            LOG_I("%d bytes from %s icmp_seq=%d ttl=%d time=%d ms", recv_len, dst_ip, icmp_seq++, ttl, time);
    }
    else
    {
        sscanf(data, "+CPING: %d,%d,%d,%d,%d,%d,%d", &result, &sent, &rcvd, &lost, &min, &max, &avg);
        if (result == 3)
        {
            LOG_I("%d sent %d received %d lost, min=%dms max=%dms average=%dms", sent, rcvd, lost, min, max, avg);
            icmp_seq = 0;
        }
    }
    if (result == 2)
    {
        LOG_E("ping: time out!!!");
        //at_tcp_ip_errcode_parse(result);
    }
}

int dy_rssi = 0;
int dy_ber = 0;
static void urc_csq_func(const char *data, rt_size_t size)
{
    //rt_kprintf("### CSQ:%s\r\n", data);

    if(strstr(data, "+CSQ") != NULL)
    {
        sscanf(data, "+CSQ: %d,%d", &dy_rssi, &dy_ber);
        rt_kprintf("### rssi:%d, ber:%d\r\n", dy_rssi, dy_ber);
    }
}

uint8_t sms_flag = 0;
uint8_t pb_flag = 0;
static void urc_func(const char *data, rt_size_t size)
{
    RT_ASSERT(data);

    LOG_I("URC data:'%.*s'", size, data);
    if(strstr(data, "SMS DONE") != NULL)
    {
        sms_flag = 1;
    }

    if(strstr(data, "PB DONE") != NULL)
    {
        pb_flag = 1;
    }
}

static struct at_urc urc_table[] = {
        {"RING",        "\r\n",                 urc_func},
        {"Call Ready",  "\r\n",                 urc_func},
        {"RDY",         "\r\n",                 urc_func},
        {"NO CARRIER",  "\r\n",                 urc_func},
        {"SMS DONE",    "\r\n",                 urc_func},
        {"PB DONE",     "\r\n",                 urc_func},
        {"+CIPOPEN",    "\r\n",                 urc_connect_func},
        {"+CIPSEND",    "\r\n",                 urc_send_func},
        {"+CIPCLOSE",   "\r\n",                 urc_close_func},
        {"+IPCLOSE",    "\r\n",                 urc_close_func},
        {"+RECEIVE",    "\r\n",                 urc_recv_func},
		{"+CPING:",     "\r\n",                 urc_ping_func},
        {"+CSQ:",       "\r\n",                 urc_csq_func},
        {"+CIPEVENT:",  "\r\n",                 urc_func},
};

#define AT_SEND_CMD(resp, resp_line, timeout, cmd)                                                              \
    do                                                                                                          \
    {                                                                                                           \
        if (at_exec_cmd(at_resp_set_info(resp, 256, resp_line, rt_tick_from_millisecond(timeout)), cmd) < 0)    \
        {                                                                                                       \
            result = -RT_ERROR;                                                                                 \
            goto __exit;                                                                                        \
        }                                                                                                       \
    } while(0);                                                                                                 \

char iccid[21];

void creg_n_result(int res)
{
    switch(res)
    {
        case 0:
            rt_kprintf("0: disable network registration unsolicited result code\r\n");
            break;
        
        case 1:
            rt_kprintf("1: enable network registration unsolicited result code +CREG: <stat>\r\n");
            break;
        
        case 2:
            rt_kprintf("2: enable network registration and location information unsolicited result code +CREG:<stat>[,<lac>,<ci>]\r\n");
            break;
    }
}
void creg_stat_result(int res)
{
    switch(res)
    {
        case 0:
            rt_kprintf("0: not registered, ME is not currently searching a new operator to register to\r\n");
            break;
        
        case 1:
            rt_kprintf("1: registered, home network\r\n");
            break;
        
        case 2:
            rt_kprintf("2: not registered, but ME is currently searching a new operator to register to\r\n");
            break;
        
        case 3:
            rt_kprintf("3: registration denied\r\n");
            break;
        
        case 4:
            rt_kprintf("4: unknown\r\n");
            break;
        
        case 5:
            rt_kprintf("5: registered, roaming\r\n");
            break;
    }
}

static void sim7600ce_init_thread_entry(void *parameter)
{
#define AT_RETRY                       10
#define CIMI_RETRY                     10
#define CSQ_RETRY                      20
#define CREG_RETRY                     10
#define CGREG_RETRY                    20
#define CPIN_RETRY                     10
#define CHANGE_BAUDRATE_RETRY          5

    at_response_t resp = RT_NULL;
    char parsed_data[20];
    rt_err_t result = RT_EOK;
    rt_size_t i;
    uint32_t baudrate = 0;

#ifdef ENABLE_4G_MODULE_SLEEP
    rt_pm_request(PM_SLEEP_MODE_SLEEP);
#endif

    resp = at_create_resp(256, 0, rt_tick_from_millisecond(300));
    if (!resp)
    {
        LOG_E("No memory for response structure!");
        result = -RT_ENOMEM;
        goto __exit;
    }
    LOG_D("Start initializing the SIM7600CE module");

    /* reset module */
//    AT_SEND_CMD(resp, 0, 300, "AT+CRESET");
//    rt_thread_delay(rt_tick_from_millisecond(2000));
    
    /* wait EC20 startup finish, Send AT every 500ms, if receive OK, SYNC success*/
    if (at_client_wait_connect(SIM7600CE_WAIT_CONNECT_TIME))
    {
        result = -RT_ETIMEOUT;
        goto __exit;
    }

    /* disable echo */
    AT_SEND_CMD(resp, 0, 300, "ATE0");
    /* Get the baudrate */
    AT_SEND_CMD(resp, 0, 300, "AT+IPR?");
    at_resp_parse_line_args_by_kw(resp, "+IPR:", "+IPR: %d", &baudrate);
    LOG_D("before Baudrate %d", baudrate);

#ifdef ENABLE_4G_MODULE_WORK_HIGH_SPEED
    char tmp_buf[32];

    //change sim7600 module baudrate
    for (i = 0; i < CHANGE_BAUDRATE_RETRY; i++)
    {
        sprintf(tmp_buf, "AT+IPR=%d", SIM7600_WORK_BAUDRATE);
        AT_SEND_CMD(resp, 0, 300, tmp_buf);
        rt_thread_delay(rt_tick_from_millisecond(200));
        set_uart_baudrate(SIM7600_UART_NAME, 4);

        at_exec_cmd(at_resp_set_info(resp, 256, 2, rt_tick_from_millisecond(300)), "AT+IPR?");
        if (at_resp_parse_line_args_by_kw(resp, "+IPR:", "+IPR: %d", &baudrate))
        {
            if(baudrate == SIM7600_WORK_BAUDRATE)
            {
                LOG_D("sim7600 change baudrate success[%d]", baudrate);
            }
            break;
        }

        set_uart_baudrate(SIM7600_UART_NAME, 1);
        rt_thread_delay(rt_tick_from_millisecond(1000));
    }
    if (i == CHANGE_BAUDRATE_RETRY)
    {
        LOG_E("sim7600 change baudrate failed");
        result = -RT_ERROR;
        goto __exit;
    }
#else
    LOG_E("sim7600 work baudrate %d", SIM7600_DEFAULT_BAUDRATE);
#endif

    /* get module version */
    AT_SEND_CMD(resp, 0, 300, "ATI");
    /* show module version */
    for (i = 0; i < (int) resp->line_counts - 1; i++)
    {
        LOG_D("%s", at_resp_get_line(resp, i + 1));
    }

    /* check SIM card */
    for (i = 0; i < CPIN_RETRY; i++)
    {
        at_exec_cmd(at_resp_set_info(resp, 256, 2, rt_tick_from_millisecond(5000)), "AT+CPIN?");

        if (at_resp_get_line_by_kw(resp, "READY"))
        {
            LOG_D("SIM card detection success");
            break;
        }
        rt_thread_delay(rt_tick_from_millisecond(1000));
    }
    if (i == CPIN_RETRY)
    {
        LOG_E("SIM card detection failed");
        result = -RT_ERROR;
        goto __exit;
    }

    /* Use AT+CIMI to query the IMSI of SIM card */
//    AT_SEND_CMD(resp, 2, 300, "AT+CIMI");
//    /* show module version */
//    for (i = 0; i < (int) resp->line_counts - 1; i++)
//    {
//        LOG_D("%s", at_resp_get_line(resp, i + 1));
//    }
    i = 0;
    while(at_exec_cmd(at_resp_set_info(resp, 256, 0, rt_tick_from_millisecond(300)), "AT+CIMI") < 0)
    {
        i++;
        LOG_D("AT+CIMI %d", i);
        if(i > CIMI_RETRY)
        {
            LOG_E("Read CIMI failed");
            result = -RT_ERROR;
            goto __exit;
        }
        rt_thread_delay(rt_tick_from_millisecond(1000));
    }

    /* Use AT+CICCID to query ICCID number of SIM card */
    AT_SEND_CMD(resp, 0, 300, "AT+CICCID");
    at_resp_parse_line_args_by_kw(resp, "+ICCID:", "+ICCID: %s", iccid);
    LOG_D("ICCID:'%s'", iccid);

    /* check the GSM network is registered */
    for (i = 0; i < CREG_RETRY; i++)
    {
        int a = 0;
        int b = 0;
        AT_SEND_CMD(resp, 0, 300, "AT+CREG?");
        at_resp_parse_line_args_by_kw(resp, "+CREG:", "+CREG: %s", parsed_data);
        rt_kprintf("CREG:'%s'\r\n", parsed_data);
        sscanf(parsed_data, "%d", &a);
        creg_n_result(a);
        if(a == 1)
        {
            sscanf(parsed_data, "%d, %d", &a, &b);
            creg_stat_result(b);
        }
        else if(a == 2)
        {
            //TODOzmb
            sscanf(parsed_data, "%d, %d", &a, &b);
            creg_stat_result(b);
        }

        if (!strncmp(parsed_data, "0,1", sizeof(parsed_data)) || !strncmp(parsed_data, "0,5", sizeof(parsed_data)))
        {
            LOG_D("GSM network is registered (%s)", parsed_data);
            break;
        }
        rt_thread_delay(rt_tick_from_millisecond(1000));
    }
    if (i == CREG_RETRY)
    {
        LOG_E("The GSM network is register failed (%s)", parsed_data);
        result = -RT_ERROR;
        goto __exit;
    }

    /* check the GPRS network is registered */
    for (i = 0; i < CGREG_RETRY; i++)
    {
        AT_SEND_CMD(resp, 0, 300, "AT+CGREG?");
        at_resp_parse_line_args_by_kw(resp, "+CGREG:", "+CGREG: %s", &parsed_data);
        if (!strncmp(parsed_data, "0,1", sizeof(parsed_data)) || !strncmp(parsed_data, "0,5", sizeof(parsed_data)))
        {
            LOG_D("GPRS network is registered (%s)", parsed_data);
            break;
        }
        rt_thread_delay(rt_tick_from_millisecond(1000));
    }
    if (i == CGREG_RETRY)
    {
        LOG_E("The GPRS network is register failed (%s)", parsed_data);
        result = -RT_ERROR;
        goto __exit;
    }

    /*Use AT+CEREG? to query current EPS Network Registration Status*/
    AT_SEND_CMD(resp, 0, 300, "AT+CEREG?");
    /* Use AT+COPS? to query current Network Operator */
    AT_SEND_CMD(resp, 0, 300, "AT+COPS?");
    at_resp_parse_line_args_by_kw(resp, "+COPS:", "+COPS: %*[^\"]\"%[^\"]", &parsed_data);
    rt_kprintf("##'%s'\n", parsed_data);
    if(strcmp(parsed_data,"CHINA MOBILE") == 0)
    {
        /* "CMCC" */
        LOG_I("%s", parsed_data);
        AT_SEND_CMD(resp, 0, 300, QICSGP_CHINA_MOBILE);
    }
    else if(strcmp(parsed_data,"CHN-UNICOM") == 0)
    {
        /* "UNICOM" */
        LOG_I("%s", parsed_data);
        AT_SEND_CMD(resp, 0, 300, QICSGP_CHINA_UNICOM);
    }
    else if(strcmp(parsed_data,"CHN-CT") == 0)
    {
        AT_SEND_CMD(resp, 0, 300, QICSGP_CHINA_TELECOM);
        /* "CT" */
        LOG_I("%s", parsed_data);
    }

    /* Enable automatic time zone update via NITZ and update LOCAL time to RTC */
    AT_SEND_CMD(resp, 0, 300, "AT+CTZU=1");
    /* Get RTC time */
    AT_SEND_CMD(resp, 0, 300, "AT+CCLK?");
    at_resp_parse_line_args_by_kw(resp, "+CCLK:", "+CCLK: \"%[^\"]", &parsed_data);
    rt_kprintf("4GModule Time: %s\n", parsed_data);

    /* Deactivate context profile */
    //AT_SEND_CMD(resp, 0, 40 * 1000, "AT+QIDEACT=1");
    /* Activate context profile */
    //AT_SEND_CMD(resp, 0, 150 * 1000, "AT+QIACT=1");
    /* Query the status of the context profile */
    //AT_SEND_CMD(resp, 0, 150 * 1000, "AT+QIACT?");
    //at_resp_parse_line_args_by_kw(resp, "+QIACT:", "+QIACT: %*[^\"]\"%[^\"]", &parsed_data);
    //LOG_I("%s", parsed_data);

    //config sock param type
    AT_SEND_CMD(resp, 0, 300, "AT+CIPCCFG=10,0,0,0,1,0,500");

    //open network
    AT_SEND_CMD(resp, 0, 1000, "AT+NETOPEN");

#ifdef ENABLE_4G_MODULE_SLEEP

#if LOWPOWER_4G
    //set RI wakeup mb
    AT_SEND_CMD(resp, 0, 1000, "AT+CFGRI=1,60,120");

    //set 4G module sleep
    sim7600ce_sleep();
    AT_SEND_CMD(resp, 0, 1000, "AT+CSCLK=1");
#endif

#endif

__exit:
    if (resp)
    {
        at_delete_resp(resp);
    }

    if (!result)
    {
        LOG_I("AT network initialize success!");
    }
    else
    {
        LOG_E("AT network initialize failed (%d)!", result);
    }

#ifdef ENABLE_4G_MODULE_SLEEP
    rt_pm_release(PM_SLEEP_MODE_SLEEP);
#endif
}

int sim7600ce_net_init(void)
{
#ifdef PKG_AT_INIT_BY_THREAD
    rt_thread_t tid;

    tid = rt_thread_create("sim7600ce_net_init", sim7600ce_init_thread_entry, RT_NULL,SIM7600CE_THREAD_STACK_SIZE, SIM7600CE_THREAD_PRIORITY, 20);
    if (tid)
    {
        rt_thread_startup(tid);
    }
    else
    {
        LOG_E("Create AT initialization thread fail!");
    }
#else
    sim7600ce_init_thread_entry(RT_NULL);
#endif

    return RT_EOK;
}

int sim7600ce_ping(int argc, char **argv)
{
    at_response_t resp = RT_NULL;

    if (argc != 2)
    {
        rt_kprintf("Please input: at_ping <host address>\n");
        return -RT_ERROR;
    }

    char temp_buf[128];
    sprintf(temp_buf, "\"%s\",1,4,64,1000,10000,255", argv[1]);
    resp = at_create_resp(128, 0, rt_tick_from_millisecond(5000));
    if (!resp)
    {
        rt_kprintf("No memory for response structure!\n");
        return -RT_ENOMEM;
    }

    if (at_exec_cmd(resp, "AT+CPING=%s", temp_buf) < 0)
    {
        rt_kprintf("AT send ping commands error!\n");
        return -RT_ERROR;
    }

    if (resp)
    {
        at_delete_resp(resp);
    }

    return RT_EOK;
}

int sim7600ce_connect(int argc, char **argv)
{
    int32_t port;

    if (argc != 3)
    {
        rt_kprintf("Please input: at_connect <host address>\n");
        return -RT_ERROR;
    }
    sscanf(argv[2],"%d",&port);
    sim7600ce_socket_connect(0, argv[1], port, AT_SOCKET_TCP, 1);

    return RT_EOK;
}

int sim7600ce_close(int argc, char **argv)
{
    if (sim7600ce_socket_close(0) < 0)
    {
        rt_kprintf("sim7600ce_socket_close fail\n");
    }
    else
    {
        rt_kprintf("sim7600ce_socket_closeed\n");
    }
    return RT_EOK;
}

int sim7600ce_send(int argc, char **argv)
{
    if(argc != 2)
    {
        rt_kprintf("Please input: at_send \"hello world^_^\"\n");
        return -RT_ERROR;
    }

    if (sim7600ce_socket_send(0, argv[1], strlen(argv[1]), AT_SOCKET_TCP) < 0)
    {
        rt_kprintf("sim7600ce_socket_send fail\n");
    }

    return RT_EOK;
}

int sim7600ce_domain(int argc, char **argv)
{
    char ip[16];

    if(argc != 2)
    {
        rt_kprintf("Please input: at_ping \"www.baidu.com\"\n");
        return -RT_ERROR;
    }

    if (sim7600ce_domain_resolve(argv[1], ip) < 0)
    {
        rt_kprintf("sim7600ce_socket_send fail\n");
    }
    else
    {
        rt_kprintf("%s : %s\n", argv[1], ip);
    }

    return RT_EOK;
}

int sim7600ce_ifconfig(void)
{
    at_response_t resp = RT_NULL;
    char resp_arg[AT_CMD_MAX_LEN] = { 0 };
    rt_err_t result = RT_EOK;

    resp = at_create_resp(128, 2, rt_tick_from_millisecond(300));
    if (!resp)
    {
        rt_kprintf("No memory for response structure!\n");
        return -RT_ENOMEM;
    }
    
    /* Query the status of the context profile */
    AT_SEND_CMD(resp, 0, 150 * 1000, "AT+IPADDR");
    at_resp_parse_line_args_by_kw(resp, "+IPADDR:", "+IPADDR: %s", &resp_arg);
    rt_kprintf("IP adress : %s\n", resp_arg);

__exit:
    if (resp)
    {
        at_delete_resp(resp);
    }

    return result;
}

int sim7600_get_csq(void)
{
    int result = 0;

    rt_mutex_take(at_event_lock, RT_WAITING_FOREVER);

#ifdef ENABLE_4G_MODULE_SLEEP
    rt_pm_request(PM_SLEEP_MODE_SLEEP);
    sim7600ce_wkup();
#endif

    if (at_exec_cmd(RT_NULL, "AT+CSQ") < 0)
    {
        result = -RT_ERROR;
        goto __exit;
    }

__exit:
#ifdef ENABLE_4G_MODULE_SLEEP
    sim7600ce_sleep();
    rt_pm_release(PM_SLEEP_MODE_SLEEP);
#endif

    rt_mutex_release(at_event_lock);

    return result;
}

#ifdef FINSH_USING_MSH
#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(sim7600ce_net_init, at_net_init, initialize AT network);
MSH_CMD_EXPORT_ALIAS(sim7600ce_ping, at_ping, AT ping network host);
MSH_CMD_EXPORT_ALIAS(sim7600ce_ifconfig, at_ifconfig, list the information of network interfaces);
MSH_CMD_EXPORT_ALIAS(sim7600ce_connect, at_connect, AT connect network host);
MSH_CMD_EXPORT_ALIAS(sim7600ce_close, at_close, AT close a socket);
MSH_CMD_EXPORT_ALIAS(sim7600ce_send, at_send, AT send a pack);
MSH_CMD_EXPORT_ALIAS(sim7600ce_domain, at_domain, AT domain resolve);
MSH_CMD_EXPORT_ALIAS(sim7600_get_csq, at_csq, get 4g csq);
#endif

static const struct at_device_ops sim7600ce_socket_ops = {
    sim7600ce_socket_connect,
    sim7600ce_socket_close,
    sim7600ce_socket_send,
    sim7600ce_domain_resolve,
    sim7600ce_socket_set_event_cb,
};

int sim7600_module_sleep(void)
{
#ifdef ENABLE_4G_MODULE_SLEEP
    sim7600ce_sleep();
#endif
    return RT_EOK;
}

int sim7600_module_wakeup(void)
{
#ifdef ENABLE_4G_MODULE_SLEEP
    sim7600ce_wkup();
#endif
    return RT_EOK;
}


static int at_socket_device_init(void)
{
    /* create current AT socket event */
    at_socket_event = rt_event_create("at_se", RT_IPC_FLAG_FIFO);
    if (at_socket_event == RT_NULL)
    {
        LOG_E("RT AT client port initialize failed! at_sock_event create failed!");
        return -RT_ENOMEM;
    }

    /* create current AT socket event lock */
    at_event_lock = rt_mutex_create("at_se", RT_IPC_FLAG_FIFO);
    if (at_event_lock == RT_NULL)
    {
        LOG_E("RT AT client port initialize failed! at_sock_lock create failed!");
        rt_event_delete(at_socket_event);
        return -RT_ENOMEM;
    }

    sim7600ce_hw_init();

    /* initialize AT client */
    at_client_init(AT_DEVICE_NAME, AT_DEVICE_RECV_BUFF_LEN);

    /* register URC data execution function  */
    at_set_urc_table(urc_table, sizeof(urc_table) / sizeof(urc_table[0]));

    /* initialize sim7600ce network */
    sim7600ce_net_init();

    /* set sim7600ce AT Socket options */
    at_socket_device_register(&sim7600ce_socket_ops);

    return RT_EOK;
}
INIT_APP_EXPORT(at_socket_device_init);

#endif /* AT_DEVICE_ESP8266 */
