/******************************************************************************
 * SwitchScanner.cpp
 * 
 * Switch debounce and scanning. Handles setup of GPIO for switch hardware
 * and provides a means of attaching handlers to switch events. Also provides
 * debounce functionality.
 * 
 * ****************************************************************************
 * Code Copyright(C) 2022 Don Goodeve. Open source - no restrictions
 * ***************************************************************************/

#include "SwitchScanner.h" 
#include "hardware/gpio.h"


// Defines
//-----------------------------------------------------------------------------
#define kPicoClockHz            (125.0e6f)
#define kButtonScan             (10)
#define kDebounceCycles         (3)
#define kHoldDuration           (300)
#define kDoublePressMaxInterval (200)

#define _ignore(a)  


// SwitchScanner::Switch methods
//-----------------------------------------------------------------------------
SwitchScanner::Switch::Switch(uint uGPIO, bool bPullUp, bool bPullDown, bool bPositiveLogic) :
    muGPIO(uGPIO), mbPositiveLogic(bPositiveLogic), mbPullUp(bPullUp), mbPullDown(bPullDown),
    mbIsPressed(false), muLastPressTime(0), muLastReleaseTime(0) {

    // Configure GPIO hardware
    gpio_set_input_enabled(muGPIO, true);
    gpio_set_input_hysteresis_enabled(muGPIO, true);

    if (true == mbPullUp) {
        gpio_pull_up(muGPIO);
    } else if (true == mbPullDown) {
        gpio_pull_down(muGPIO);
    }
}

SwitchScanner::Switch::~Switch() {
}

uint SwitchScanner::Switch::getGPIO(void) const {
    return muGPIO;
}

bool SwitchScanner::Switch::isPositiveLogic(void) const {
    return mbPositiveLogic;
}

void SwitchScanner::Switch::assertedHigh(void) {
    if (true == mbPositiveLogic) {
        press();
    } else {
        release();
    }
}

void SwitchScanner::Switch::assertedLow(void) {
    if (true == mbPositiveLogic) {
        release();
    } else {
        press();
    }
}

// User-override button handling methods - default null implementation
void SwitchScanner::Switch::press(void) {
}

void SwitchScanner::Switch::release(void) {
}

void SwitchScanner::Switch::hold(void) {
}

void SwitchScanner::Switch::doublePress(void) {
}

SwitchScanner *SwitchScanner::mspInstance = nullptr;

SwitchScanner::SwitchScanner(void) :
    muGPIOVectorWriteIndex(0), muSwitchMask(0x0) {
    // Initialize state
    muStateVector = gpio_get_all();
    for(uint i=0; i<kSwitchScanVectorDepth; i++) {
        mpGPIOVector[i] = muStateVector;
    }
}

SwitchScanner::~SwitchScanner() {
}

void SwitchScanner::setScanActive(bool bActive) {
    if (true == bActive) {
        add_repeating_timer_ms(kSwitchScanInterval, SwitchScanner::timerCallback, NULL, &mcButtonTimer);
    } else {
        cancel_repeating_timer(&mcButtonTimer);
    }
}

bool SwitchScanner::addSwitch(Switch *pSwitch) {
    if (nullptr == pSwitch) {
        return false;
    }

    std::map<uint,Switch*>::iterator cFind(mmSwitches.find(pSwitch->getGPIO()));
    if (cFind != mmSwitches.end()) {
        return false;
    }

    mmSwitches[pSwitch->getGPIO()] = pSwitch;
    muSwitchMask |= (1 << pSwitch->getGPIO());
    return true;
}

bool SwitchScanner::removeSwitch(Switch *pSwitch) {
    std::map<uint,Switch*>::iterator cFind(mmSwitches.find(pSwitch->getGPIO()));
    if (cFind != mmSwitches.end()) {
        mmSwitches.erase(cFind);
        muSwitchMask &= ~(1 << pSwitch->getGPIO());
        return true;
    }
    return false;
}

bool SwitchScanner::timerCallback(struct repeating_timer *pTimer) {
    if (mspInstance != nullptr) {
        return mspInstance->switchScan();
    }
    return false;
}

bool SwitchScanner::switchScan(void) {
    uint32_t uBitVector(gpio_get_all());
    mpGPIOVector[muGPIOVectorWriteIndex] = uBitVector;
    muGPIOVectorWriteIndex = (muGPIOVectorWriteIndex<kSwitchScanVectorDepth)?(muGPIOVectorWriteIndex+1):0;

    // Determine asserted0 and asserted1 vectors across mpGPIOVector;
    uint32_t uAsserted0(0x00000000), uAsserted1(0xffffffff);
    for(uint i=0; i<kSwitchScanVectorDepth; i++) {
        uAsserted0 |= mpGPIOVector[i];
        uAsserted1 &= mpGPIOVector[i];
    }
    uAsserted0 = ~uAsserted0;

    // uAsserted1 has 1 in places where 1 is consistent,  uAsserted0 has 1 in places where 0 is consistent
    uint32_t uAssertedHigh((uAsserted1 & ~muStateVector) & muSwitchMask), uAssertedLow((uAsserted0 & muStateVector) & muSwitchMask);

    // Iterate over uRising, uFalling and generate Switch::press() and Switch::release() - modifying muStateVector as we go
    uint uSwitch(0);
    while(uAssertedHigh != 0x0) {
        if ((uAssertedHigh & 0x1) != 0) {
            mmSwitches[uSwitch]->assertedHigh();
            muStateVector |= (1 << uSwitch);
        }
        uAssertedHigh >>= 1;
        uSwitch++;
    }

    uSwitch = 0;
    while(uAssertedLow != 0x0) {
        if ((uAssertedLow & 0x1) != 0) {
            mmSwitches[uSwitch]->assertedLow();
            muStateVector |= (1 << uSwitch);
        }
        uAssertedLow >>= 1;
        uSwitch++;
    }

    return true;
}