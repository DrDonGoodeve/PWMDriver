/******************************************************************************
 * main.cpp
 * 
 * Magic Broom main program file. Targetted to RP Zero (waveshare RP2040) board
 * with RGB LED (WS2812) accessed via GPIO16.
 * ****************************************************************************
 * Code Copyright(C) 2022 Don Goodeve. Open source - no restrictions
 * ***************************************************************************/


// Board configuration
//*****************************************************************************
// GPIO 2,3,4   - 2.2V LED drive (Red, Yellow, Orange)
// GPIO 5,6,7,8 - 3.2V LED drive (Blue, Green, Pink, White)
// All the above buffered by SN75468N low-side drive, direct connected (op.coll)
// GPIO1 Laser diode drive (buffered by 8k2 into 2N2222 op. coll)
// GPIO9 pull-down press switch
// GPIO10 pull-down radio-fob relay switch


// Includes
//*****************************************************************************
#include "pico/stdlib.h"
#include <stdio.h>
#include <list>
#include <math.h>
#include <memory.h>

#include "PWMDriver.h"
#include "SwitchScanner.h"
#include "WS2812.hpp"


// Macros
//*****************************************************************************
#define kClkFrequency       (125.0e6f)
#define kPulseFrequency     (1.0f)
#define kLEDUpdateFreq      (1000.0f)


#define LED_PIN 16  // GPIO 16
#define LED_LENGTH 1
#define kPi (3.141592654f)

#define _min(a,b)           (((a)<(b))?(a):(b))

/*
// Heartbeat - conventional PWMDriver implementation (PWMwrap-interrupt-driven)
//-------------------------------------
class Heartbeat : public PWMDriver::Source,
                  public PWMDriver::Group {
    
    private:
        bool mbCountingUp;
        float mfValue;
        float mfIncrement;

    public:
        Heartbeat(uint uGPIO) :
            PWMDriver::Source(uGPIO),
            mbCountingUp(true), mfValue(0.0f), mfIncrement(0.0f) {

            addSource(this);
            PWMDriver::instance()->addGroup(this);
        }

        ~Heartbeat(void) {
            PWMDriver::instance()->removeGroup(this);
            removeSource(this);
        }

        // Subclass required implementations
        virtual float getDesiredUpdateFrequency(void) {
            return kLEDUpdateFreq;
        }

        virtual void resetSequence(void) {
            mbCountingUp = true;
            mfValue = 0.0f;
            mfIncrement = (2.0f * kPulseFrequency) / mfSampleRateHz;
        }

        virtual float getNextSequence(void) {
            //printf("Heartbeat::getNextSequence - %.2f\r\n", mfValue);
            if (true == mbCountingUp) {
                mfValue += mfIncrement;
                if (mfValue >= 1.0f) {
                    mfValue = 1.0f;
                    mbCountingUp = false;
                }
            } else {
                mfValue -= mfIncrement;
                if (mfValue <= 0.0f) {
                    mfValue = 0.0f;
                    mbCountingUp = true;
                }
            }
            return mfValue;
        }
};
*/

static WS2812 *spLEDs = nullptr;
static bool timerCallback(struct repeating_timer *pTimer){
    // Ascending spiral RGB
    static float fPhase(0.0f), fAmplitude(0.1f);
    static float fPhaseShift(kPi/1.3f);
    static bool bRising(true);

    float fRed(sinf(fPhase)), fGreen(sinf(fPhase+fPhaseShift)), fBlue(sinf(fPhase+(2.0f*fPhaseShift)));
    fRed = (fRed*fRed)*fAmplitude; 
    fGreen = (fGreen*fGreen)*fAmplitude; 
    fBlue = (fBlue*fBlue)*fAmplitude; 
    uint8_t uRed((uint8_t)roundf(fRed*255.0f));
    uint8_t uGreen((uint8_t)roundf(fGreen*255.0f));
    uint8_t uBlue((uint8_t)roundf(fBlue*255.0f));

    spLEDs->fill(WS2812::RGB(uRed, uGreen, uBlue));
    spLEDs->show();

    if (true == bRising) {
        fAmplitude += 1.0e-2f;
        if (fAmplitude >= 1.0f) {
            bRising = false;
        }
    } else {
        fAmplitude -= 1.0e-2f;
        if (fAmplitude <= 0.0f) {
            bRising = true;
            fPhase = fPhase+(kPi/26.7f);
            fPhase = (fPhase>(2.0f*kPi))?(fPhase-(2.0f*kPi)):fPhase;
        }
    }

    return true;
}

int main(void) {
    // Initialize stdio functionality
    stdio_init_all();

    // 0. Initialize LED strip
    WS2812 cLEDStrip(
        LED_PIN,            // Data line is connected to pin 0. (GP0)
        LED_LENGTH,         // Strip is 6 LEDs long.
        pio0,               // Use PIO 0 for creating the state machine.
        0,                  // Index of the state machine that will be created for controlling the LED strip
                            // You can have 4 state machines per PIO-Block up to 8 overall.
                            // See Chapter 3 in: https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
        WS2812::FORMAT_GRB  // Pixel format used by the LED strip
    );
    spLEDs = &cLEDStrip;

    // Startup colourful heartbeat
    struct repeating_timer cHeartbeatTimer;
    add_repeating_timer_ms(10, timerCallback, NULL, &cHeartbeatTimer);
  
    while(true) {
        tight_loop_contents();
    }
}