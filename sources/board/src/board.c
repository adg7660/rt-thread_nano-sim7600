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
 * 2017-07-24     Tanek        the first version
 */
#include <rthw.h>
#include <rtthread.h>

#include "stm32l1xx.h"
#include "usart.h"
#include "board.h"

#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define STM32_HEAP_BEGIN    (&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define STM32_HEAP_BEGIN    (__segment_end("HEAP"))
#else
extern int __bss_end;
#define STM32_HEAP_BEGIN    (&__bss_end)
#endif

#define STM32_SRAM_SIZE     32
#define STM32_SRAM_END      (0x20000000 + STM32_SRAM_SIZE * 1024)

/*!
 * Unique Devices IDs register set ( STM32L152x )
 */

#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )

/*!
 * LED GPIO pins objects
 */
Gpio_t Led0;    // Active Low

/*!
 * PushButton GPIO pin object
 */
Gpio_t PushButton0;

/*
 * MCU objects
 */
//Uart_t Uart1;
//Uart_t Uart2;
//Uart_t Uart3;

#define PUSH_DOWN               0

/*!
 * Nested interrupt counter.
 *
 * \remark Interrupt should only be fully disabled once the value is 0
 */
static uint8_t IrqNestLevel = 0;

//static BoardVersion_t BoardVersion = BOARD_VERSION_NONE;

void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

void BoardEnableIrq( void )
{
    IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

void BoardGetDevEUI( uint8_t *id )
{
    uint32_t *pDevEuiHWord = ( uint32_t* )&id[4];

    if( *pDevEuiHWord == 0 )
    {
        *pDevEuiHWord = BoardGetRandomSeed( );
    }
}

static void BoardUnusedIoInit( void )
{
    Gpio_t ioPin;

    if( GetBoardPowerSource( ) == USB_POWER )
    {
        HAL_DBGMCU_EnableDBGStopMode( );
        HAL_DBGMCU_EnableDBGSleepMode( );
        HAL_DBGMCU_EnableDBGStandbyMode( );
    }
    else
    {
        GpioInit( &ioPin, USB_DM, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &ioPin, USB_DP, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

        HAL_DBGMCU_DisableDBGSleepMode( );
        HAL_DBGMCU_DisableDBGStopMode( );
        HAL_DBGMCU_DisableDBGStandbyMode( );

        GpioInit( &ioPin, JTAG_TMS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &ioPin, JTAG_TCK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &ioPin, JTAG_TDI, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &ioPin, JTAG_TDO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &ioPin, JTAG_NRST, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    }

    GpioInit( &ioPin, BOOT_1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

void SystemClockReConfig( void )
{
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    /* Enable HSE */
    __HAL_RCC_HSE_CONFIG( RCC_HSE_ON );

    /* Wait till HSE is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSERDY ) == RESET )
    {
    }

    /* Enable PLL */
    __HAL_RCC_PLL_ENABLE( );

    /* Wait till PLL is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    {
    }

    /* Select PLL as system clock source */
    __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );

    /* Wait till PLL is used as system clock source */
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
    {
    }
}

void Button0_Callback_Func(void)
{
    static uint8_t count = 0;

    if(GpioRead( &PushButton0 ) == PUSH_DOWN)
    {
        HAL_Delay(60);
        if(GpioRead( &PushButton0 ) == PUSH_DOWN)
        {
            while(GpioRead( &PushButton0 ) == PUSH_DOWN);
            rt_kprintf("button 0 down[%d]\r\n", count++);
        }
    }
}

void BoardInitPeriph( void )
{
    // Init the GPIO pins
    GpioInit( &Led0, LED_0, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );

    GpioInit( &PushButton0, BUTTON_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioSetInterrupt( &PushButton0, IRQ_FALLING_EDGE, IRQ_MEDIUM_PRIORITY, Button0_Callback_Func );

    // Switch LED 0 OFF
    GpioWrite( &Led0, 1 );
}

void BoardDeInitMcu( void )
{
    Gpio_t ioPin;

    GpioInit( &ioPin, OSC_HSE_IN, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    GpioInit( &ioPin, OSC_HSE_OUT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

    GpioInit( &ioPin, OSC_LSE_IN, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1 );
    GpioInit( &ioPin, OSC_LSE_OUT, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1 );
}

uint8_t GetBoardPowerSource( void )
{
    return USB_POWER;
}

void SystemClockConfig( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    __HAL_RCC_PWR_CLK_ENABLE( );

    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
    if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        while(1);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
    {
        while(1);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        while(1);
    }

    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / RT_TICK_PER_SECOND );

    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    // HAL_NVIC_GetPriorityGrouping
    HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

void BoardInitMcu( void )
{
	HAL_Init();
	SystemClockConfig();

    BoardInitPeriph();
    BoardUnusedIoInit();
}

Gpio_t sim7600_rst_pin;
void sim7600ce_hw_reset(void)
{
    GpioWrite(&sim7600_rst_pin, 1);
    rt_thread_delay(rt_tick_from_millisecond(200));
    GpioWrite(&sim7600_rst_pin, 0);
}
void sim7600ce_hw_init(void)
{
    GpioInit( &sim7600_rst_pin, SIM7600_RST_PIN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );

    sim7600ce_hw_reset();

#ifdef ENABLE_4G_MODULE_SLEEP
    sleep_4G_init_timer();

    rt_pin_mode(SIM_RI_SIM7600, PIN_MODE_INPUT);
    rt_pin_attach_irq(SIM_RI_SIM7600, PIN_IRQ_MODE_RISING, wkup_mb_pin_callback, RT_NULL);
    rt_pin_irq_enable(SIM_RI_SIM7600, PIN_IRQ_ENABLE);
#endif
}

/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
	BoardInitMcu();

    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
	rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init((void*)STM32_HEAP_BEGIN, (void*)STM32_SRAM_END);
#endif
}

void SysTick_Handler(void)
{
	/* enter interrupt */
	rt_interrupt_enter();

    HAL_IncTick();
	rt_tick_increase();

	/* leave interrupt */
	rt_interrupt_leave();
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    rt_kprintf( "Wrong parameters value: file %s on line %d\r\n", file, line );
    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif

