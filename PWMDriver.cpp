/******************************************************************************
 * PWMDriver.cpp
 * 
 * PWM Driver module. Provides functions for driving single and groups of
 * PWM outputs from signal source objects. Versatile enough to handle
 * servo control, light dimming, audio output etc...
 * 
 * ****************************************************************************
 * Code Copyright(C) 2022 Don Goodeve. Open source - no restrictions
 * ***************************************************************************/

#include "PWMDriver.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <list>
#include <algorithm>
#include <math.h>
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"

extern int giCalls;

// Defines
//-----------------------------------------------------------------------------
#define kPicoClockHz            (125.0e6f)
#define kPWMCountMax            (0x10000)
#define kPWMCountMin            (100)      // Appears to be...
#define kDefaultPWMFrequency    (1000.0f)
#define _limit(a, min, max)     (((a)<(min))?(min):(((a)>(max))?(max):(a)))
#define _max(a, b)              (((a)>(b))?(a):(b))
#define _min(a, b)              (((a)<(b))?(a):(b))


// PWM driver class. Singleton that provides control for groups and individual
// PWM outputs.
//-----------------------------------------------------------------------------
PWMDriver::Source::Source(uint uGPIO) :
    mbRunning(false),
    muGPIO(uGPIO), muSlice(pwm_gpio_to_slice_num(uGPIO)),
    mfPWMRateHz(kDefaultPWMFrequency), mfClkDiv(1.0f), muWrapValue(kPWMCountMax) {
}

PWMDriver::Source::~Source() {
}

uint PWMDriver::Source::getGPIO(void) const {
    return muGPIO;
}

float PWMDriver::Source::getPWMRateHz(void) const {
    return mfPWMRateHz;
}

void PWMDriver::Source::setPWMConfiguration(float fPWMRateHz, float fClkDiv, uint uWrapValue) {
    printf("PWMDriver::Source::setPWMConfiguration(%.2f, %.2f, %d)\r\n", fPWMRateHz, fClkDiv, uWrapValue);
    mfPWMRateHz = fPWMRateHz;
    mfClkDiv = fClkDiv;
    muWrapValue = uWrapValue;

    pwm_set_enabled(muSlice, false);
    mbRunning = false;
    gpio_set_function(muGPIO, GPIO_FUNC_PWM);
    pwm_set_gpio_level(muGPIO, 0x0);    // Off
    pwm_config cPWMConfig(pwm_get_default_config());
    pwm_config_set_clkdiv(&cPWMConfig, mfClkDiv);
    pwm_config_set_output_polarity(&cPWMConfig, false, false);
    pwm_config_set_wrap(&cPWMConfig, (uint16_t)muWrapValue);
    pwm_init(muSlice, &cPWMConfig, false);
}

bool PWMDriver::Source::setActive(bool bActive) {
    if (bActive != mbRunning) {
        if (true == bActive) {
            pwm_set_enabled(muSlice, true);
            mbRunning = true;
            return true;
        } else {
            pwm_set_enabled(muSlice, false);
            mbRunning = false;
            return true;
        }
    }
    return false;
}

PWMDriver::UpdateOnWrapSource::UpdateOnWrapSource(uint uGPIO) :
    Source(uGPIO) {
}

PWMDriver::UpdateOnWrapSource::~UpdateOnWrapSource() {
}

bool PWMDriver::UpdateOnWrapSource::setActive(bool bActive) {
    resetSequence();
    return Source::setActive(bActive);
}

// Extra method for UpdateOnWrapSource:: subtype
void PWMDriver::UpdateOnWrapSource::updateSource(void) {
    float fNext(getNextSequence());
    fNext = _limit(fNext, 0.0f, 1.0f);
    uint uNext((uint)roundf(fNext * (float)muWrapValue));
    pwm_set_gpio_level(muGPIO, uNext);
}

// A group is a collection of sources that share a common refresh rate. Also if the
// group consists of UpdateOnWrapSource instances - the wrap interrupt mechanism
// will be set up for this group.
PWMDriver::Group::Group(void) :
    muIRQSlice(0), muWrapCount(kPWMCountMax), muClkDivider(1), mbManageIRQ(false) {
}

PWMDriver::Group::~Group() {
}

// Manage sources within group
bool PWMDriver::Group::addSource(Source *pSource) {
    std::list<Source*>::iterator cFind(std::find(mlSources.begin(), mlSources.end(), pSource));
    if (cFind != mlSources.end()) {
        return false;
    }

    // Add to list
    if (dynamic_cast<UpdateOnWrapSource*>(pSource) != nullptr) {
        mbManageIRQ = true;
    }

    mlSources.push_back(pSource);
    //printf("PWMDriver::Group::addSource - %d\r\n", mlSources.size());
    return true;
}

bool PWMDriver::Group::removeSource(Source *pSource) {
    std::list<Source*>::iterator cFind(std::find(mlSources.begin(), mlSources.end(), pSource));
    if (cFind == mlSources.end()) {
        return false;
    }
    mlSources.erase(cFind);
    return true;
}

void PWMDriver::Group::start(void) {
    for(std::list<Source*>::iterator cSource=mlSources.begin(); cSource != mlSources.end(); ++cSource) {
        (*cSource)->setActive(true);
    }            
}

void PWMDriver::Group::halt(void) {
    for(std::list<Source*>::iterator cSource=mlSources.begin(); cSource != mlSources.end(); ++cSource) {
        (*cSource)->setActive(false);
    }
}

void PWMDriver::Group::update(void) {
    for(std::list<Source*>::iterator cSource=mlSources.begin(); cSource != mlSources.end(); ++cSource) {
        UpdateOnWrapSource *pUOWSource(dynamic_cast<UpdateOnWrapSource*>(*cSource));
        if (pUOWSource != nullptr) {
            pUOWSource->updateSource();
        }
    }
}

void PWMDriver::Group::setPWMConfiguration(float fPWMRate, float fClkDiv, uint uWrapValue) {
    printf("PWMDriver::Group::setPWMConfiguration(%.2f, %.2f, %d)\r\n", fPWMRate, fClkDiv, uWrapValue);
    for(std::list<Source*>::iterator cSource=mlSources.begin(); cSource != mlSources.end(); ++cSource) {
        (*cSource)->setPWMConfiguration(fPWMRate, fClkDiv, uWrapValue);
    }
}

// Global declaration for PWMDriver singleton instance
PWMDriver *PWMDriver::mspInstance = nullptr;

PWMDriver::PWMDriver(void) {
}

PWMDriver::~PWMDriver() {
}

PWMDriver *PWMDriver::instance(void) {
    if (nullptr == mspInstance) {
        mspInstance = new PWMDriver();
    }
    return mspInstance;
}

// ISR as entrypoint to PWMDriver class
void PWMDriver::pwmISRStatic(void) {
    PWMDriver *pDriver(PWMDriver::instance());
    if (pDriver != nullptr) {
        pDriver->pwmISR();
    }
}

void PWMDriver::pwmISR(void) {
    // Determine which Interrupts have fired (ie. which group(s))
    uint32_t uResidualInterruptVector(pwm_get_irq_status_mask());

    // Toggle pin 21 each interrupt
    //gpio_put(16, !gpio_get_out_level(16));

    // Call group update functions
    uint32_t uCheckInterrupts(uResidualInterruptVector);
    uint uActiveSlice(0);
    while(uCheckInterrupts != 0) {
        if ((uCheckInterrupts & 0x1) != 0) {    // Slice is active - does it match to a group?
            pwm_clear_irq(uActiveSlice);
            for(std::list<Group*>::iterator cGroup=mlGroups.begin(); cGroup != mlGroups.end(); ++cGroup) {
                if (uActiveSlice == (*cGroup)->muIRQSlice) {
                    (*cGroup)->update();
                    break;
                }
            }
        }
        uCheckInterrupts >>= 1;
        uActiveSlice++;
    }
}

bool PWMDriver::addGroup(Group *pGroup) {
    printf("PWMDriver::addgroup\r\n");
    if (nullptr == pGroup) {
        return false;
    }
    if (true == pGroup->mlSources.empty()) {
        return false;   // Empty group
    }

    std::list<Group*>::iterator cFind(std::find(mlGroups.begin(), mlGroups.end(), pGroup));
    if (cFind != mlGroups.end()) {
        return false;
    }

    // Choose the smallest divisor consistent with achieving the desired frequency
    // This maximized the bit resolution available.
    float fTargetFrequency(pGroup->getDesiredUpdateFrequency());
    printf("PWMDriver::addGroup fTargetFrequency = %.4f\r\n", fTargetFrequency);

    // Figure out divider that will give us the largest number of bits at the desired target frequency
    // ie. maximizing the wrap count. Note the divisor is an 8.4 fixed-point number in the range 1.0
    // to 255.9375. Note that count is constrained on the lower end at kPWMCountMin. This was found
    // experimentally and appears to be enforced setting a maximum frequency to 125kHz on the PWM.
    float fClocksPerInterval(kPicoClockHz / fTargetFrequency);
    float fClkDiv8_4(1.0f);
    uint uWrapValue(_min(kPWMCountMax, (uint)roundf(fClocksPerInterval)));
    printf("fCPI=%.4f, fClkDiv8_4=%.4f, Wrap=%d\r\n", fClocksPerInterval, fClkDiv8_4, uWrapValue);
    if (fClocksPerInterval > (float)kPWMCountMax) {
        float fExactClockDivisor(fClocksPerInterval / (float)kPWMCountMax);
        fClkDiv8_4 = (ceilf(fExactClockDivisor * 16.0f) / 16.0f);
        uWrapValue = (uint)roundf((kPicoClockHz / fClkDiv8_4) / fTargetFrequency);
        printf("#2 fCPI=%.4f, fClkDiv8_4=%.4f, Wrap=%d\r\n", fClocksPerInterval, fClkDiv8_4, uWrapValue);
    }
    uWrapValue = (uWrapValue < kPWMCountMin)?kPWMCountMin:uWrapValue;
    float fRealizedFrequency((kPicoClockHz / fClkDiv8_4) / (float)uWrapValue);

    if (true == pGroup->mbManageIRQ) {
        // Set up the group interrupt - note there is just one PWM IRQ.
        // Mask our slice's IRQ output into the PWM block's single interrupt line,
        // register the interrupt handler and enable it. Nothing happens yet
        // because we have not started the PWM running for this slice.
        Source *pPrimary(pGroup->mlSources.front());
        pGroup->muIRQSlice = pwm_gpio_to_slice_num(pPrimary->getGPIO());
        printf("PWMDriver::addGroup - muIRQSlice = %d\r\n", pGroup->muIRQSlice);
        pwm_clear_irq(pGroup->muIRQSlice);
        pwm_set_irq_enabled(pGroup->muIRQSlice, true);
        irq_set_exclusive_handler(PWM_IRQ_WRAP, PWMDriver::pwmISRStatic);
        irq_set_enabled(PWM_IRQ_WRAP, true);
    }

    // Note setPWMConfiguration may disable the pwm irq if it is intending
    // to not use the interrupt-based update mechanism.
    pGroup->setPWMConfiguration(fRealizedFrequency, fClkDiv8_4, uWrapValue);

    // Add the group to the groups list - and go
    mlGroups.push_back(pGroup);

    // Start the group
    pGroup->start();

    return true;
}

bool PWMDriver::removeGroup(Group *pGroup) {
    if (nullptr == pGroup) {
        return false;
    }

    std::list<Group*>::iterator cFind(std::find(mlGroups.begin(), mlGroups.end(), pGroup));
    if (cFind == mlGroups.end()) {
        return false;
    }

    // Remove the group IRQ - will stop the group
    pGroup->halt();
    pwm_set_irq_enabled(pGroup->muIRQSlice, false);
    mlGroups.erase(cFind);
    return true;
}

