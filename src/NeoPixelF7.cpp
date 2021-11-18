//#include <chrono>
#include <cstring>
#include "NeoPixelF7.h"

#if defined(ARDUINO) 
    #include <Arduino.h>
#elif defined(__MBED__)
    #include <mbed.h>
#endif

#define METRICS

//using namespace std::chrono;

TIM_HandleTypeDef g_TimHandle;
DMA_HandleTypeDef g_TimerDMA;

volatile bool g_DataSentFlag = true;

#define WS_2812_CLK_FREQ            (800'000u)
#define WS_2812_ZERO_HIGH_TIME_NS   (400ull)
#define WS_2812_RESET_PERIOD_NS     (50'000ull)
#define ONE_SECOND_NS               (1'000'000'000ull)
//const nanoseconds   WS_2812_ZERO_HIGH_TIME_NS   = 400ns;
//const nanoseconds   WS_2812_RESET_PERIOD_NS     = 50us;

uint32_t get_timer_clock_speed()
{
#if defined(STM32G0xx) || defined(STM32F0xx)
    auto rval = HAL_RCC_GetPCLK1Freq();
    auto mask = RCC_CFGR_PPRE;
#else
    auto rval = HAL_RCC_GetPCLK2Freq();
    auto mask = RCC_CFGR_PPRE2;
#endif
    if((RCC->CFGR & mask) != 0)
        rval *= 2;
    return rval;
}

//using TimerTicks = duration<uint32_t , std::ratio<1, TIMER_CLK_FREQ>>;
//
//const TimerTicks ONE_SECOND_TICKS = 1s;

uint32_t g_AutoReloadRegister; // = TIMER_CLK_FREQ / WS_2812_CLK_FREQ;
uint32_t g_ShortPulse;
uint32_t g_ResetCycleCount;

uint32_t g_DataWaiting = false;
bool buffer_index = false;
bool g_init = false;

uint8_t                    g_PwmData0       [(24 * NUM_PIXELS) + 50];
uint8_t                    g_PwmData1       [(24 * NUM_PIXELS) + 50];
//uint16_t                    g_PwmData0       [(24 * NUM_PIXELS) + 50];
//uint16_t                    g_PwmData1       [(24 * NUM_PIXELS) + 50];
uint8_t* pwm_data[] = {g_PwmData0, g_PwmData1};
uint8_t* current_pwm_data = pwm_data[buffer_index];
//uint16_t* pwm_data[] = {g_PwmData0, g_PwmData1};
//uint16_t* current_pwm_data = pwm_data[buffer_index];

void print(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);

#if defined(ARDUINO)
    if (Serial)
        Serial.printf(fmt, args);
#elif defined(__MBED__)
    printf(fmt, args);
#endif

    va_end(args);
}

[[noreturn]] void error_handler(const char* fmt = nullptr, ...)
{
    va_list args;
    va_start(args, fmt);
    if (fmt) print(fmt, args);
    va_end(args);
    __disable_irq();
    while (true);
}

void NeoPixelF7_show(const uint32_t* ptr, uint32_t num_pixels)
{
    buffer_index = ! buffer_index;
    current_pwm_data = pwm_data[buffer_index];
    uint32_t length = 0;

    for (uint32_t j = 0; j < num_pixels; ++j)
    {
        for (int i = 23; i >= 0; i--)
        {
            current_pwm_data[length++] = ptr[j] & (1 << i) ? g_ShortPulse << 1 : g_ShortPulse;
//            g_PwmData[length++] = g_ShortPulse << (ptr[j] & (1 << i));
        }
    }

    for (uint32_t i = 0; i < g_ResetCycleCount + 1; i++)
    {
        current_pwm_data[length++] = 0;
    }

#ifdef METRICS
    uint32_t start = HAL_GetTick();
#endif
    while (!g_DataSentFlag);
#ifdef METRICS
    g_DataWaiting = HAL_GetTick() - start;
#endif
    g_DataSentFlag = false;
    HAL_TIM_PWM_Start_DMA(&g_TimHandle, TIM_CHANNEL_1, (uint32_t*) current_pwm_data, length);
}

void calculate_timings()
{
    uint32_t timhz = get_timer_clock_speed();
//    if (TIMER_CLK_FREQ != timhz)
//        error_handler();

    g_AutoReloadRegister    = timhz / WS_2812_CLK_FREQ;
    g_ShortPulse            = timhz * WS_2812_ZERO_HIGH_TIME_NS / ONE_SECOND_NS;
    g_ResetCycleCount       = timhz * WS_2812_RESET_PERIOD_NS / ONE_SECOND_NS / g_AutoReloadRegister;
//    g_AutoReloadRegister = (ONE_SECOND_TICKS / WS_2812_CLK_FREQ).count();
//    g_ShortPulse = duration_cast<TimerTicks>(WS_2812_ZERO_HIGH_TIME_NS).count();
    //    const auto reset_ticks = duration_cast<TimerTicks>(WS_2812_RESET_PERIOD_NS);
    //    g_ResetCycleCount = reset_ticks / (ONE_SECOND_TICKS / WS_2812_CLK_FREQ);
}

void NeoPixelF7_init()
{
    if (g_init) return;

    calculate_timings();

#if defined(STM32L4xx) || defined(STM32F3xx) || defined(STM32F1xx)
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
#elif defined(STM32F7xx)
    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
#elif defined(STM32G0xx)
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
#elif defined(STM32G4xx)
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
#elif defined(STM32F0xx)
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
#endif

    TIM_ClockConfigTypeDef          tim_clk_conf   = {0};
    TIM_OC_InitTypeDef              oc_conf        = {0};

    g_TimHandle.Instance = TIM1;
    g_TimHandle.Init.Period = g_AutoReloadRegister - 1;

    tim_clk_conf.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    oc_conf.OCMode = TIM_OCMODE_PWM1;

    if (HAL_TIM_Base_Init(&g_TimHandle) != HAL_OK) error_handler();

    __HAL_RCC_TIM1_CLK_ENABLE();

#if defined(STM32L4xx)
    g_TimerDMA.Instance = DMA1_Channel2;
    g_TimerDMA.Init.Request = DMA_REQUEST_7;
#elif defined(STM32F7xx)
    g_TimerDMA.Instance = DMA2_Stream1;
    g_TimerDMA.Init.Channel = DMA_CHANNEL_6;
#elif defined(STM32F3xx) || defined(STM32F1xx) || defined(STM32F0xx)
    g_TimerDMA.Instance = DMA1_Channel2;
#elif defined(STM32G0xx) || defined(STM32G4xx)
    g_TimerDMA.Instance = DMA1_Channel1;
    g_TimerDMA.Init.Request = DMA_REQUEST_TIM1_CH1;
#endif
    g_TimerDMA.Init.Direction = DMA_MEMORY_TO_PERIPH;
    g_TimerDMA.Init.MemInc = DMA_MINC_ENABLE;
    g_TimerDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    g_TimerDMA.Init.MemDataAlignment = DMA_PDATAALIGN_BYTE;
#if defined(STM32F7xx)
    g_HdmaTim1Ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#endif

    if (HAL_DMA_Init(&g_TimerDMA) != HAL_OK) error_handler();

    __HAL_LINKDMA(&g_TimHandle, hdma[TIM_DMA_ID_CC1], g_TimerDMA);

    if (HAL_TIM_ConfigClockSource(&g_TimHandle, &tim_clk_conf) != HAL_OK) error_handler();
    if (HAL_TIM_PWM_Init(&g_TimHandle) != HAL_OK) error_handler();
    if (HAL_TIM_PWM_ConfigChannel(&g_TimHandle, &oc_conf, TIM_CHANNEL_1) != HAL_OK) error_handler();

    GPIO_InitTypeDef gpio_init_struct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio_init_struct.Pin = GPIO_PIN_8;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
//    gpio_init_struct.Pull = GPIO_NOPULL;
//    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
#if defined (STM32F3xx) || defined(STM32G4xx)
    gpio_init_struct.Alternate = GPIO_AF6_TIM1;
#elif defined(STM32G0xx) || defined(STM32F0xx)
    gpio_init_struct.Alternate = GPIO_AF2_TIM1;
#elif defined(STM32F7xx) || defined(STM32L4xx)
    gpio_init_struct.Alternate = GPIO_AF1_TIM1;
#endif
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    g_init = true;
}

//void NeoPixelF7_reset()
//{
//    g_init = false;
//
//}

extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef*)
{
    HAL_TIM_PWM_Stop_DMA(&g_TimHandle, TIM_CHANNEL_1);
    g_DataSentFlag = true;
}

#if defined(STM32F7xx)
extern "C" void DMA2_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_TimerDMA);
}
#elif defined(STM32L4xx) || defined(STM32F3xx) || defined(STM32F1xx)
extern "C" void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_TimerDMA);
}
#elif defined(STM32G0xx) || defined(STM32G4xx)
extern "C" void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_TimerDMA);
}
#elif defined(STM32F0xx)
extern "C" void DMA1_Channel2_3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_TimerDMA);
}
#endif

uint32_t Pixels::create_color(const uint8_t& red, const uint8_t& green, const uint8_t& blue)
{
    return (green << 16 | red << 8 | blue);
}

void Pixels::set_color(const uint32_t& index, const uint32_t& color)
{
    if (index >= len_) return;
    pixels_[index] = color;
}

void Pixels::set_rgb(const uint32_t& index, const uint8_t& red, const uint8_t& green, const uint8_t& blue)
{
    if (index >= len_) return;
    pixels_[index] = create_color(red, green, blue);
}

void Pixels::clear(const uint32_t& index)
{
    if (index >= len_) return;
    pixels_[index] = 0;
}

void Pixels::clear_all()
{
    memset(pixels_, 0, NUM_PIXELS * sizeof(uint32_t));
}

void Pixels::show()
{
    NeoPixelF7_show(pixels_, len_);
}

void Pixels::begin()
{
    NeoPixelF7_init();
}

Pixels::Pixels(uint32_t* arr, uint32_t len)
:pixels_(arr)
,len_(len)
{}

