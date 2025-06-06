#pragma once

#include "ISR.h"
#include "imxrt.h"
#include <cstdint>

/// @brief Configures the GPT1 for timed interrupts.
/// @param us Interval for the GPT1 overflow in MicroSeconds.
/// @note This function must be called before using the stepper motor, in doAtRelayOnce block.
/// @warning DO NOT MODIFY THE FUNCTION BODY.
void GPT_CONFIG_1(uint32_t us)
{
    CCM_CCGR1 |= CCM_CCGR1_GPT(CCM_CCGR_ON);

    GPT1_CR = 0; // turn off GPT1 timer
    GPT1_PR = 24 - 1; // prescale from 24 MHz to 1 MHz
    GPT1_OCR1 = us - 1; // interrupt interval in microseconds
    GPT1_IR = GPT_IR_OF1IE; // enable output compare 1
    GPT1_SR = 0x03; // clear status register
    GPT1_CR = GPT_CR_EN | GPT_CR_CLKSRC(1); // enable with ipg_clk (peripheral clock for general purpose timers)

    attachInterruptVector(IRQ_GPT1, ISR_PID_CONTROL);
    NVIC_ENABLE_IRQ(IRQ_GPT1);
}

void GPT_CONFIG_2(uint32_t us)
{
    CCM_CCGR1 = CCM_CCGR1_GPT(CCM_CCGR_ON);
    GPT2_CR = 0;
    GPT2_PR = 24 - 1;
    GPT2_OCR1 = us - 1;
    GPT2_IR = GPT_IR_OF1IE;
    GPT2_SR = 0x03;
    GPT2_CR = GPT_CR_EN | GPT_CR_CLKSRC(1);

    attachInterruptVector(IRQ_GPT2, ISR_I2C);
}
