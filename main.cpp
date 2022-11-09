/******************************************************************************
 * main.cpp
 * 
 * Main program file to test PWM driver.
 * ****************************************************************************
 * Code Copyright(C) 2022 Don Goodeve. Open source - no restrictions
 * ***************************************************************************/

#include "pico/stdlib.h"
#include <stdio.h>
#include <list>
#include <math.h>
#include "PWMDriver.h"

#define kIncrement      (0.01f)
#define kOnBoardLED     (PICO_DEFAULT_LED_PIN) // GPIO0
#define kUpdateFreq     (100.0f)


// PWM Source - main board LED
//-----------------------------------------------------------------------------
class Heartbeat : public PWMDriver::Source,
                  public PWMDriver::Group {
    
    private:
        bool mbCountingUp;
        float mfValue;

    public:
        Heartbeat(void) :
            PWMDriver::Source(kOnBoardLED),
            mbCountingUp(true), mfValue(0.0f) {

            addSource(this);
            PWMDriver::instance()->addGroup(this);
        }

        ~Heartbeat(void) {
            PWMDriver::instance()->removeGroup(this);
            removeSource(this);
        }

        // Subclass required implementations
        virtual float getDesiredUpdateFrequency(void) {
            return kUpdateFreq;
        }

        virtual void resetSequence(void) {
            printf("Heartbeat::resetSequence\r\n");
            mbCountingUp = true;
            mfValue = 0.0f;
        }

        virtual float getNextSequence(void) {
            //printf("Heartbeat::getNextSequence - %.2f\r\n", mfValue);
            if (true == mbCountingUp) {
                mfValue += kIncrement;
                if (mfValue >= 1.0f) {
                    mfValue = 1.0f;
                    mbCountingUp = false;
                }
            } else {
                mfValue -= kIncrement;
                if (mfValue <= 0.0f) {
                    mfValue = 0.0f;
                    mbCountingUp = true;
                }
            }
            return mfValue;
        }
};


int main(void) {
    // Initialize stdio functionality
    stdio_init_all();

    busy_wait_ms(10000);
    printf("Starting up...\r\n");

    Heartbeat cHeartbeat;

    // Everything after this point happens in the PWM interrupt handler, so we
    // can twiddle our thumbs
    while (true) {
        tight_loop_contents();
    }

    return 0;
}