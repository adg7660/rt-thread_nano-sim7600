/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2016-08-30     Aubr.Cool       the first version
 */

#include <stm32l1xx.h>
#include "stdio.h"

#include <rtthread.h>
#include "usart.h"

#ifdef RT_USING_UART

#ifndef RT_USING_DEVICE
#error "you must define RT_USING_DEVICE with uart device"
#endif

#ifndef RT_UART_RX_BUFFER_SIZE
#define RT_UART_RX_BUFFER_SIZE 64
#endif

/* STM32 uart driver */
struct stm32_uart
{
    struct rt_device parent;
    USART_TypeDef * uart_base;
    IRQn_Type uart_irq;

    /* buffer for reception */
    rt_uint8_t read_index, save_index;
    rt_uint8_t rx_buffer[RT_UART_RX_BUFFER_SIZE];
    UART_HandleTypeDef uart_handle;
//    uint8_t rx_data;
    uint8_t tx_data;
};
#ifdef RT_USING_UART1
struct stm32_uart uart1_device;
#endif

#ifdef RT_USING_UART2
struct stm32_uart uart2_device;
#endif

static int drv_putc(UART_HandleTypeDef *uart_handle, char c)
{
    while ((__HAL_UART_GET_FLAG(uart_handle, UART_FLAG_TXE) == RESET));
    uart_handle->Instance->DR = c;
    while ((__HAL_UART_GET_FLAG(uart_handle, UART_FLAG_TC) == RESET));
    return 1;
}

static int drv_getc(UART_HandleTypeDef *uart_handle)
{
    int ch;
    ch = -1;
    if (__HAL_UART_GET_FLAG(uart_handle, UART_FLAG_RXNE) != RESET)
        ch = uart_handle->Instance->DR & 0xff;
    return ch;
}

static void uart_irq_handler(struct stm32_uart* uart)
{
    rt_ubase_t level;
    /* enter interrupt */
    rt_interrupt_enter();
    level = rt_hw_interrupt_disable();
    uart->rx_buffer[uart->save_index] = (rt_uint8_t)drv_getc(&uart->uart_handle);;
    uart->save_index ++;
    if (uart->save_index >= RT_UART_RX_BUFFER_SIZE)
    {
        uart->save_index = 0;
    }
    rt_hw_interrupt_enable(level);
    
    /* invoke callback */
    if (uart->parent.rx_indicate != RT_NULL)
    {
        rt_size_t length;
        if (uart->read_index > uart->save_index)
            length = RT_UART_RX_BUFFER_SIZE - uart->read_index + uart->save_index;
        else
            length = uart->save_index - uart->read_index;

        uart->parent.rx_indicate(&uart->parent, length);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}

#ifdef RT_USING_UART1
void USART1_IRQHandler( void )
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* UART in mode Receiver -------------------------------------------------*/
    if ((__HAL_UART_GET_FLAG(&uart1_device.uart_handle, UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&uart1_device.uart_handle, UART_IT_RXNE) != RESET))
    {
        uart_irq_handler(&uart1_device);
        /* Clear RXNE interrupt flag */
        __HAL_UART_CLEAR_FLAG(&uart1_device.uart_handle, UART_FLAG_RXNE);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif
#ifdef RT_USING_UART2
void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    /* UART in mode Receiver -------------------------------------------------*/
    if ((__HAL_UART_GET_FLAG(&uart2_device.uart_handle, UART_FLAG_RXNE) != RESET) &&
            (__HAL_UART_GET_IT_SOURCE(&uart2_device.uart_handle, UART_IT_RXNE) != RESET))
    {
        uart_irq_handler(&uart2_device);
        /* Clear RXNE interrupt flag */
        __HAL_UART_CLEAR_FLAG(&uart2_device.uart_handle, UART_FLAG_RXNE);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

static void uart_io_init(struct stm32_uart* uart)
{
    GPIO_InitTypeDef GPIO_InitStruct;
#ifdef RT_USING_UART1
    if (uart->uart_base == USART1)
    {
        __HAL_RCC_USART1_FORCE_RESET( );
        __HAL_RCC_USART1_RELEASE_RESET( );
        __HAL_RCC_USART1_CLK_ENABLE( );
        
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else
#endif

#ifdef RT_USING_UART2
    if (uart->uart_base == USART2)
    {
        __HAL_RCC_USART2_FORCE_RESET( );
        __HAL_RCC_USART2_RELEASE_RESET( );
        __HAL_RCC_USART2_CLK_ENABLE( );
        
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = GPIO_PIN_2;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = GPIO_PIN_3;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else
#endif
    {
        RT_ASSERT((uart->uart_base == USART1) || (uart->uart_base == USART2));
    }
}

static void uart_init(struct stm32_uart* uart)
{
    uart->uart_handle.Instance = uart->uart_base;
    uart->uart_handle.Init.BaudRate = 115200;
    uart->uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart->uart_handle.Init.StopBits = UART_STOPBITS_1;
    uart->uart_handle.Init.Parity = UART_PARITY_NONE;
    uart->uart_handle.Init.Mode = UART_MODE_TX_RX;
    uart->uart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart->uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;
    if ( HAL_UART_Init( &uart->uart_handle ) != HAL_OK )
    {
        while(1);
    }

    HAL_NVIC_SetPriority( uart->uart_irq, 1, 0 );
    HAL_NVIC_EnableIRQ( uart->uart_irq );
    HAL_UART_Receive_IT( &uart->uart_handle, &uart->rx_buffer[uart->save_index], 1 );
}

void rt_hw_console_output(const char *str)
{
//    HAL_UART_Transmit(&uart1_device.uart_handle, (uint8_t *)str, strlen(str), 0xFF);
//    rt_thread_delay(1);
}

static rt_err_t rt_uart_init (rt_device_t dev)
{
    struct stm32_uart* uart;
    RT_ASSERT(dev != RT_NULL);
    uart = (struct stm32_uart *)dev;
    
    uart_io_init(uart);
    uart_init(uart);

    return RT_EOK;
}

static rt_err_t rt_uart_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct stm32_uart* uart;
    RT_ASSERT(dev != RT_NULL);
    uart = (struct stm32_uart *)dev;

    if (dev->flag & RT_DEVICE_FLAG_INT_RX)
    {
        /* Enable the UART Interrupt */
        NVIC_EnableIRQ(uart->uart_irq);
    }

    return RT_EOK;
}

static rt_err_t rt_uart_close(rt_device_t dev)
{
    struct stm32_uart* uart;
    RT_ASSERT(dev != RT_NULL);
    uart = (struct stm32_uart *)dev;

    if (dev->flag & RT_DEVICE_FLAG_INT_RX)
    {
        /* Disable the UART Interrupt */
        NVIC_DisableIRQ(uart->uart_irq);
    }

    return RT_EOK;
}

static rt_size_t rt_uart_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    struct stm32_uart* uart = (struct stm32_uart *)dev;
    rt_uint8_t *ptr;
    rt_size_t length;
    
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);
    
    ptr = (rt_uint8_t *) buffer;
    while (size)
    {
        /* interrupt receive */
        rt_base_t level;
        
        /* disable interrupt */
        level = rt_hw_interrupt_disable();
        if (uart->read_index != uart->save_index)
        {
            *ptr = uart->rx_buffer[uart->read_index];

            uart->read_index ++;
            if (uart->read_index >= RT_UART_RX_BUFFER_SIZE)
                uart->read_index = 0;
        }
        else
        {
            /* no data in rx buffer */

            /* enable interrupt */
            rt_hw_interrupt_enable(level);
            break;
        }

        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        ptr ++;
        size --;
    }

    length = (rt_uint32_t)ptr - (rt_uint32_t)buffer;
    return length;
}

static rt_size_t rt_uart_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    char *ptr = (char*) buffer;
    struct stm32_uart* uart = (struct stm32_uart *)dev;
    
    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    if (dev->open_flag & RT_DEVICE_FLAG_STREAM)
    {
        /* stream mode */
        while (size)
        {
            if (*ptr == '\n')
            {
                uart->tx_data = '\r';
                drv_putc(&uart->uart_handle, uart->tx_data);
            }

            uart->tx_data = (uint8_t)*ptr;
            drv_putc(&uart->uart_handle, uart->tx_data);

            ptr ++;
            size --;
        }
    }
    else
    {
        while (size)
        {
            uart->tx_data = (uint8_t)*ptr;
            drv_putc(&uart->uart_handle, uart->tx_data);

            ptr++;
            size--;
        }
    }

    return (rt_size_t)ptr - (rt_size_t)buffer;
}

int rt_hw_usart_init(void)
{
#ifdef RT_USING_UART1
    {
        struct stm32_uart* uart;

        /* get uart device */
        uart = &uart1_device;

        /* device initialization */
        uart->parent.type = RT_Device_Class_Char;
        uart->uart_base = USART1;
        uart->uart_irq = USART1_IRQn;
        uart->read_index = 0;
        uart->save_index = 0;
        rt_memset(uart->rx_buffer, 0, sizeof(uart->rx_buffer));

        /* device interface */
        uart->parent.init 	    = rt_uart_init;
        uart->parent.open 	    = rt_uart_open;
        uart->parent.close      = rt_uart_close;
        uart->parent.read 	    = rt_uart_read;
        uart->parent.write      = rt_uart_write;
        uart->parent.control    = RT_NULL;
        uart->parent.user_data  = RT_NULL;

        rt_device_register(&uart->parent, "uart1", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }
#endif

#ifdef RT_USING_UART2
    {
        struct stm32_uart* uart;

        /* get uart device */
        uart = &uart2_device;

        /* device initialization */
        uart->uart_base = USART2;
        uart->uart_irq = USART2_IRQn;
        uart->read_index = 0;
        uart->save_index = 0;
        rt_memset(uart->rx_buffer, 0, sizeof(uart->rx_buffer));

        /* device interface */
        uart->parent.type       = RT_Device_Class_Char;
        uart->parent.init 	    = rt_uart_init;
        uart->parent.open 	    = rt_uart_open;
        uart->parent.close      = rt_uart_close;
        uart->parent.read 	    = rt_uart_read;
        uart->parent.write      = rt_uart_write;
        uart->parent.control    = RT_NULL;
        uart->parent.user_data  = RT_NULL;

        rt_device_register(&uart->parent, "uart2", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    }
#endif /* RT_USING_UART2 */
    return 0;
}
INIT_BOARD_EXPORT(rt_hw_usart_init);

#endif /*RT_USING_UART*/

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
int __io_putchar( int c )
#else /* __GNUC__ */
int fputc( int c, FILE *stream )
#endif
{
    drv_putc(&uart1_device.uart_handle, c);
    return c;
}
