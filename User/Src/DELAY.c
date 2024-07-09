#include "DELAY.h"

void delay_us(uint32_t udelay)
{
    uint32_t startval, tickn, delays, wait;

    startval = SysTick->VAL;
    tickn = HAL_GetTick();
    //sysc = 72000;  //SystemCoreClock / (1000U / uwTickFreq);
    delays = udelay * 150; //sysc / 1000 * udelay;
    if(delays > startval)
    {
        while(HAL_GetTick() == tickn)
        {

        }
        wait = 150000 + startval - delays;
        while(wait < SysTick->VAL)
        {

        }
    }
    else
    {
        wait = startval - delays;
        while(wait < SysTick->VAL && HAL_GetTick() == tickn)
        {

        }
    }
}

void DWT_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

void DWT_Delay_us(uint32_t us) // microseconds
{
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);

    while (DWT->CYCCNT - startTick < delayTicks);
}
