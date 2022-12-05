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
#include <stdio.h>
#include "hardware/gpio.h"


// Defines
//-----------------------------------------------------------------------------

// SwitchScanner::Switch methods
//-----------------------------------------------------------------------------
SwitchScanner::Switch::Switch(uint uGPIO, Pull ePull, bool bPositiveLogic) :
    muGPIO(uGPIO), mbPositiveLogic(bPositiveLogic),
    mbIsPressed(false), mbIsHeld(false), muLastPressTime(0), muLastReleaseTime(0) {

    // Configure GPIO hardware
    gpio_set_input_enabled(muGPIO, true);
    gpio_set_input_hysteresis_enabled(muGPIO, true);

    if (kPullUp == ePull) {
        gpio_pull_up(muGPIO);
    } else if (kPullDown == ePull) {
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
        pressWrapper();
    } else {
        releaseWrapper();
    }
}

void SwitchScanner::Switch::assertedLow(void) {
    if (true == mbPositiveLogic) {
        releaseWrapper();
    } else {
        pressWrapper();
    }
}

void SwitchScanner::Switch::pressWrapper(void) {
    uint uNow(time_us_32());
    mbIsPressed = true;
    uint uDelayFromRelease(uNow - muLastReleaseTime);
    muLastPressTime = uNow;
    if (uDelayFromRelease < kDoublePressMaxInterval) {
        doublePress();
    } else {
        press();
    }
}

void SwitchScanner::Switch::releaseWrapper(void) {
    uint uNow(time_us_32());
    mbIsPressed = false;
    mbIsHeld = false;
    muLastReleaseTime = uNow;
    release();
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
    uint32_t uGPIOState(gpio_get_all());
    for(uint i=0; i<kSwitchScanVectorDepth; i++) {
        mpGPIOVector[i] = uGPIOState;
    }
    mspInstance = this;
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
    mpGPIOVector[muGPIOVectorWriteIndex] = uBitVector & muSwitchMask;
    muGPIOVectorWriteIndex = (muGPIOVectorWriteIndex<(kSwitchScanVectorDepth-1))?(muGPIOVectorWriteIndex+1):0;

    // Determine asserted0 and asserted1 vectors across mpGPIOVector and narrow to muSwitchMask
    uint32_t uAsserted0(0x00000000), uAsserted1(0xffffffff);
    for(uint i=0; i<kSwitchScanVectorDepth; i++) {
        uAsserted0 |= mpGPIOVector[i];
        uAsserted1 &= mpGPIOVector[i];
    }
    uAsserted0 = ~uAsserted0 & muSwitchMask;
    uAsserted1 &= muSwitchMask;
    //printf("GPIO: 0x%08x, A0: 0x%08x, A1:0x%08x\r\n", uBitVector, uAsserted0, uAsserted1);

    // uAsserted1 has 1 in places where 1 is consistent,  uAsserted0 has 1 in places where 0 is consistent
    uint32_t uAssertedHighChange((uAsserted1 & ~muStateVector) & muSwitchMask);  // Was low, now high
    uint32_t uAssertedLowChange((uAsserted0 & muStateVector) & muSwitchMask);    // Was high, now low

    //printf("GPIO: 0x%04x, A0: 0x%04x, A1: 0x%04x, AHC: 0x%04x, ALC:0x%04x, SV:0x%04x\r\n", uBitVector, uAsserted0, uAsserted1, uAssertedHighChange, uAssertedLowChange, muStateVector);

    // Iterate over uRising, uFalling and generate Switch::press() and Switch::release() - modifying muStateVector as we go
    uint uSwitch(0);
    while(uAssertedHighChange != 0x0) {
        if ((uAssertedHighChange & 0x1) != 0) {
            mmSwitches[uSwitch]->assertedHigh();
            muStateVector |= (1 << uSwitch);
        }
        uAssertedHighChange >>= 1;
        uSwitch++;
    }

    uSwitch = 0;
    while(uAssertedLowChange != 0x0) {
        if ((uAssertedLowChange & 0x1) != 0) {
            mmSwitches[uSwitch]->assertedLow();
            muStateVector &= (~(1 << uSwitch) & muSwitchMask);
        }
        uAssertedLowChange >>= 1;
        uSwitch++;
    }

    // Check for 'hold' events
    uint uNow(time_us_32());
    for(std::map<uint, Switch*>::iterator cIter = mmSwitches.begin(); cIter != mmSwitches.end(); ++cIter) {
        if ((false == (*cIter).second->mbIsHeld) && (true == (*cIter).second->mbIsPressed)) {
            if ((uNow - (*cIter).second->muLastPressTime) > kHoldUsec) {
                (*cIter).second->mbIsHeld = true;
                (*cIter).second->hold();
            }
        }
    }

    return true;
}