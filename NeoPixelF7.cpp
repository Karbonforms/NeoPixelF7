#include <stm32f7xx_hal.h>
#include <chrono>

#include "NeoPixelF7_config.h"

using namespace std::chrono;

TIM_HandleTypeDef g_TimHandle;
DMA_HandleTypeDef g_HdmaTim1Ch1;

volatile bool g_DataSentFlag = true;

const uint32_t      WS_2812_CLK_FREQ            = 800000;
const nanoseconds   WS_2812_ZERO_HIGH_TIME_NS   = 400ns;
const nanoseconds   WS_2812_RESET_PERIOD_NS     = 50us;

using TimerTicks = duration<uint32_t , std::ratio<1, TIMER_CLK_FREQ>>;

uint32_t g_AutoReloadRegister = TIMER_CLK_FREQ / WS_2812_CLK_FREQ;
uint32_t g_ShortPulse;
uint32_t g_ResetCycleCount;

//uint16_t*                    g_PwmData; //       [(24 * NUM_KEYS) + 50];
uint16_t                    g_PwmData       [(24 * NUM_PIXELS) + 50];

void error_handler()
{
    __disable_irq();
    while (true)
    {}
}

void NeoPixelF7_show(const uint32_t* ptr, uint32_t num_pixels)
{
    while (!g_DataSentFlag);
    g_DataSentFlag = false;

    uint32_t length = 0;

//    for (auto& item: pixels_)
    for (uint32_t j = 0; j < num_pixels; ++j)
    {
//        const auto d = item.container().data();
//        const uint32_t* ptr = reinterpret_cast<uint32_t*>(d);

        for (int i = 23; i >= 0; i--)
        {
            g_PwmData[length++] = ptr[j] & (1 << i) ? g_ShortPulse << 1 : g_ShortPulse;
        }
    }

//    for (int i = 0; i < 50; i++)
    for (uint32_t i = 0; i < g_ResetCycleCount + 1; i++)
    {
        g_PwmData[length++] = 0;
    }

    HAL_TIM_PWM_Start_DMA(&g_TimHandle, TIM_CHANNEL_1, (uint32_t*) g_PwmData, length);
}

void calculate_timings()
{
    TimerTicks one_second_ticks = 1s;

    g_AutoReloadRegister = (one_second_ticks / WS_2812_CLK_FREQ).count();
    g_ShortPulse = duration_cast<TimerTicks>(WS_2812_ZERO_HIGH_TIME_NS).count();

    const auto reset_ticks = duration_cast<TimerTicks>(WS_2812_RESET_PERIOD_NS);
    g_ResetCycleCount = reset_ticks / (one_second_ticks / WS_2812_CLK_FREQ);
}

void NeoPixelF7_init()
{
//    const size_t sz = (24 * num_pixels) + 50;
//    g_PwmData = new uint16_t[sz];
//    memset(g_PwmData, 0, sz * sizeof(uint16_t));

    calculate_timings();

    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

    TIM_ClockConfigTypeDef          clock_source_config     = {0};
    TIM_MasterConfigTypeDef         master_config           = {0};
    TIM_OC_InitTypeDef              config_oc               = {0};
    TIM_BreakDeadTimeConfigTypeDef  break_dead_time_config  = {0};

    g_TimHandle.Instance = TIM1;
    g_TimHandle.Init.Prescaler = 0;
    g_TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_TimHandle.Init.Period = g_AutoReloadRegister - 1;
    g_TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_TimHandle.Init.RepetitionCounter = 0;
    g_TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&g_TimHandle) != HAL_OK)
    {
        error_handler();
    }
    clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&g_TimHandle, &clock_source_config) != HAL_OK)
    {
        error_handler();
    }
    if (HAL_TIM_PWM_Init(&g_TimHandle) != HAL_OK)
    {
        error_handler();
    }
    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&g_TimHandle, &master_config) != HAL_OK)
    {
        error_handler();
    }
    config_oc.OCMode = TIM_OCMODE_PWM1;
    config_oc.Pulse = 0;
    config_oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    config_oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    config_oc.OCFastMode = TIM_OCFAST_DISABLE;
    config_oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    config_oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&g_TimHandle, &config_oc, TIM_CHANNEL_1) != HAL_OK)
    {
        error_handler();
    }
    break_dead_time_config.OffStateRunMode = TIM_OSSR_DISABLE;
    break_dead_time_config.OffStateIDLEMode = TIM_OSSI_DISABLE;
    break_dead_time_config.LockLevel = TIM_LOCKLEVEL_OFF;
    break_dead_time_config.DeadTime = 0;
    break_dead_time_config.BreakState = TIM_BREAK_DISABLE;
    break_dead_time_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    break_dead_time_config.BreakFilter = 0;
    break_dead_time_config.Break2State = TIM_BREAK2_DISABLE;
    break_dead_time_config.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    break_dead_time_config.Break2Filter = 0;
    break_dead_time_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&g_TimHandle, &break_dead_time_config) != HAL_OK)
    {
        error_handler();
    }

    GPIO_InitTypeDef gpio_init_struct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio_init_struct.Pin = GPIO_PIN_8;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_init_struct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
}

// called by HAL_TIM_Base_Init
extern "C" void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
    if (tim_baseHandle->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();

        g_HdmaTim1Ch1.Instance = DMA2_Stream1;
        g_HdmaTim1Ch1.Init.Channel = DMA_CHANNEL_6;
        g_HdmaTim1Ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
        g_HdmaTim1Ch1.Init.PeriphInc = DMA_PINC_DISABLE;
        g_HdmaTim1Ch1.Init.MemInc = DMA_MINC_ENABLE;
        g_HdmaTim1Ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        g_HdmaTim1Ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        g_HdmaTim1Ch1.Init.Mode = DMA_NORMAL;
        g_HdmaTim1Ch1.Init.Priority = DMA_PRIORITY_LOW;
        g_HdmaTim1Ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&g_HdmaTim1Ch1) != HAL_OK)
        {
            error_handler();
        }

        __HAL_LINKDMA(tim_baseHandle, hdma[TIM_DMA_ID_CC1], g_HdmaTim1Ch1);
    }
}


// called by HAL_TIM_Base_DeInit
extern "C" void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
    if (tim_baseHandle->Instance == TIM1)
    {
        __HAL_RCC_TIM1_CLK_DISABLE();
        HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC1]);
    }
}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef*)
{
    HAL_TIM_PWM_Stop_DMA(&g_TimHandle, TIM_CHANNEL_1);
    g_DataSentFlag = true;
}

extern "C" void DMA2_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_HdmaTim1Ch1);
}