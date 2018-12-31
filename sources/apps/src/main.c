
/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-07-22     Tanek        the first version
 */
#include <stdio.h>
#include <rthw.h>
#include <rtthread.h>

#include "stm32l1xx.h"

#include "board.h"

void test_thread(void *para)
{
    while(1)
    {
//        rt_kprintf("-->in test_thread\r\n");
        GpioToggle( &Led0 );
        
        rt_thread_delay(500);
    }
}

int main(void)
{
    rt_thread_t test_tid;

    test_tid = rt_thread_create("test", test_thread, "thread_test",
                           RT_MAIN_THREAD_STACK_SIZE, 4, 20);
    RT_ASSERT(test_tid != RT_NULL);

    rt_thread_startup(test_tid);
    
    return 0;
}
