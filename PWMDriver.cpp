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


// Defines
//-----------------------------------------------------------------------------
#define kPicoClockHz            (125.0e6f)
#define kPWMCountMax            (0x10000)
#define kDefaultPWMFrequency    (1000.0f)
#define _limit(a, min, max)     (((a)<(min))?(min):(((a)>(max))?(max):(a))) 


// PWM driver class. Singleton that provides control for groups and individual
// PWM outputs.
//-----------------------------------------------------------------------------
PWMDriver::Source::Source(uint uGPIO) :
    mbRunning(false),
    muGPIO(uGPIO), muSlice(pwm_gpio_to_slice_num(uGPIO)),
    mfSampleRateHz(kDefaultPWMFrequency), mfClkDiv(1.0f), muWrapValue(kPWMCountMax) {
}

PWMDriver::Source::~Source() {
}

uint PWMDriver::Source::getGPIO(void) const {
    return muGPIO;
}

float PWMDriver::Source::getSampleRateHz(void) const {
    return mfSampleRateHz;
}


// Subclass overrideable
void PWMDriver::Source::setPWMConfiguration(float fSampleRateHz, float fClkDiv, uint uWrapValue) {
    mfSampleRateHz = fSampleRateHz;
    mfClkDiv = fClkDiv;
    muWrapValue = uWrapValue;
    configure();
}
                
void PWMDriver::Source::configure(void) {
    // Disable PWM output - reconfigure - enable PWM output.
    pwm_set_enabled(muSlice, false);
    mbRunning = false;
    gpio_set_function(muGPIO, GPIO_FUNC_PWM);
    pwm_set_gpio_level(muGPIO, 0x0);    // Off
    pwm_config cPWMConfig(pwm_get_default_config());
    pwm_config_set_clkdiv(&cPWMConfig, mfClkDiv);
    pwm_config_set_output_polarity(&cPWMConfig, true, true);
    pwm_config_set_wrap(&cPWMConfig, (uint16_t)muWrapValue);
    pwm_init(muSlice, &cPWMConfig, false);

    // Ready for first in sequence
    resetSequence();
}

void PWMDriver::Source::start(void) {
    if (false == mbRunning) {
        pwm_set_enabled(muSlice, true);
        updateSource();
        mbRunning = true;
    }
}

void PWMDriver::Source::updateSource(void) {
    float fNext(_limit(getNextSequence(), 0.0f, 1.0f));
    uint uNext((uint)roundf(fNext * (float)muWrapValue));
    pwm_set_gpio_level(muGPIO, uNext);
}

void PWMDriver::Source::halt(void) {
    if (true == mbRunning) {
        pwm_set_enabled(muSlice, false);
        mbRunning = false;
    }
}


// A group is a collection of sources that share a common sample rate and are updated in unison.
PWMDriver::Group::Group(void) :
    muIRQSlice(0), muWrapCount(kPWMCountMax), muClkDivider(1) {
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
    mlSources.push_back(pSource);
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
        (*cSource)->start();
    }            
}

void PWMDriver::Group::halt(void) {
    for(std::list<Source*>::iterator cSource=mlSources.begin(); cSource != mlSources.end(); ++cSource) {
        (*cSource)->halt();
    }
}

void PWMDriver::Group::update(void) {
    for(std::list<Source*>::iterator cSource=mlSources.begin(); cSource != mlSources.end(); ++cSource) {
        (*cSource)->updateSource();
    }
}

float PWMDriver::Group::getDesiredUpdateFrequency(void) const {
    if (false == mlSources.empty()) {
        return mlSources.front()->getDesiredUpdateFrequency();
    } else {
        return kDefaultPWMFrequency;
    }
}

void PWMDriver::Group::setPWMConfiguration(float fSampleRateHz, float fClkDiv, uint uWrapValue) {
    printf("PWMDriver::Group::setPWMConfiguration(%.2f, %.2f, %d)\r\n", fSampleRateHz, fClkDiv, uWrapValue);
    for(std::list<Source*>::iterator cSource=mlSources.begin(); cSource != mlSources.end(); ++cSource) {
        (*cSource)->setPWMConfiguration(fSampleRateHz, fClkDiv, uWrapValue);
    }
}

// Global declaration for PWMDriver singleton instance
PWMDriver *PWMDriver::mspInstance = nullptr;

PWMDriver::PWMDriver(void) {
    printf("PWMDriver::PWMDriver\r\n");
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
    // Determine which ISR (ie. which group)
    uint32_t uResidualInterruptVector(pwm_get_irq_status_mask());
    uint uActiveSlice(0);
    std::list<uint> lActiveSlices;
    while(uResidualInterruptVector != 0x0) {
        if ((uResidualInterruptVector & 0x1) != 0x0) {
            pwm_clear_irq(uActiveSlice);
            lActiveSlices.push_back(uActiveSlice);
        }
        uResidualInterruptVector >>= 1;
        uActiveSlice++;
    }

    // Call group update functions
    for(std::list<uint>::iterator cSlice = lActiveSlices.begin(); cSlice != lActiveSlices.end(); ++cSlice) {
        for(std::list<Group*>::iterator cGroup=mlGroups.begin(); cGroup != mlGroups.end(); ++cGroup) {
            if ((*cSlice) == (*cGroup)->muIRQSlice) {
                (*cGroup)->update();
            }
        }
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

    // Figure out divider that will give us the largest number of bits at the desired target frequency
    // ie. maximizing the wrap count. Note the divisor is an 8.4 fixed-point number in the range 1.x
    // to 255.f
    float fClocksPerInterval(kPicoClockHz / fTargetFrequency);
    float fClkDiv8_4(1.0f);
    uint uWrapValue(kPWMCountMax);
    if (fClocksPerInterval > (float)kPWMCountMax) {
        float fExactClockDivisor(fClocksPerInterval / (float)kPWMCountMax);
        fClkDiv8_4 = (ceilf(fExactClockDivisor * 16.0f) / 16.0f);
        uWrapValue = (uint)roundf((kPicoClockHz / fClkDiv8_4) / fTargetFrequency);
    }
    float fRealizedFrequency((kPicoClockHz / fClkDiv8_4) / (float)uWrapValue);
    pGroup->setPWMConfiguration(fRealizedFrequency, fClkDiv8_4, uWrapValue);

    // Set up the group interrupt - note there is just one PWM IRQ.
    Source *pPrimary(pGroup->mlSources.front());
    pGroup->muIRQSlice = pwm_gpio_to_slice_num(pPrimary->getGPIO());
    printf("PWMDriver::addGroup - muIRQSlice = %d\r\n", pGroup->muIRQSlice);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // register the interrupt handler and enable it.
    pwm_clear_irq(pGroup->muIRQSlice);
    pwm_set_irq_enabled(pGroup->muIRQSlice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, PWMDriver::pwmISRStatic);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Add the group to the list
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

