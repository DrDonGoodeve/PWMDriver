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
#define kRGBLEDGPIO         (16)  // RP Zero allocation
#define kPi                 (3.141592654f)
#define _min(a,b)           (((a)<(b))?(a):(b))


//*** Local types
//*****************************************************************************
class Heartbeat {
    private:
        WS2812 mcLEDs;
        float mfPhase;
        float mfPhaseShift;
        float mfAmplitude;
        bool mbRising;

        struct repeating_timer mcHeartbeatTimer;

        static Heartbeat *mspSelf;

        static bool timerCallback(struct repeating_timer *pTimer) {
            if (mspSelf != nullptr) {
                return mspSelf->heartbeatCallback();
            }
            return false;
        }

        bool heartbeatCallback(void) {
            float fRed(sinf(mfPhase)), fGreen(sinf(mfPhase+mfPhaseShift)), fBlue(sinf(mfPhase+(2.0f*mfPhaseShift)));
            fRed = (fRed*fRed)*mfAmplitude; 
            fGreen = (fGreen*fGreen)*mfAmplitude; 
            fBlue = (fBlue*fBlue)*mfAmplitude; 
            uint8_t uRed((uint8_t)roundf(fRed*255.0f));
            uint8_t uGreen((uint8_t)roundf(fGreen*255.0f));
            uint8_t uBlue((uint8_t)roundf(fBlue*255.0f));

            mcLEDs.fill(WS2812::RGB(uRed, uGreen, uBlue));
            mcLEDs.show();

            if (true == mbRising) {
                mfAmplitude += 2.0e-2f;
                if (mfAmplitude >= 1.0f) {
                    mbRising = false;
                }
            } else {
                mfAmplitude -= 2.0e-2f;
                if (mfAmplitude <= 0.0f) {
                    mbRising = true;
                    mfPhase = mfPhase+(kPi/26.7f);
                    mfPhase = (mfPhase>(2.0f*kPi))?(mfPhase-(2.0f*kPi)):mfPhase;
                    mfPhaseShift += (kPi/7.0f);
                    mfPhaseShift = (mfPhaseShift>(2.0f*kPi))?(mfPhaseShift-(2.0f*kPi)):mfPhaseShift;
                }
            }
            mfPhase = (mfPhase>(2.0f*kPi))?(mfPhase-(2.0f*kPi)):mfPhase;

            return true;
        }

    public:
        Heartbeat(void) :
            mcLEDs(kRGBLEDGPIO, 1, pio0, 0, WS2812::FORMAT_GRB),
            mfPhase(0.0f), mfAmplitude(0.1f), mbRising(true), mfPhaseShift(kPi/1.3f) {
            mspSelf = this;

            // Startup colourful heartbeat
            add_repeating_timer_ms(10, timerCallback, NULL, &mcHeartbeatTimer);
        }

        ~Heartbeat() {
            cancel_repeating_timer(&mcHeartbeatTimer);
            mspSelf = nullptr;
        }
};

Heartbeat *Heartbeat::mspSelf = nullptr;



int main(void) {
    // Initialize stdio functionality
    stdio_init_all();

    Heartbeat cHeartbeat;

    while(true) {
        tight_loop_contents();
    }
}