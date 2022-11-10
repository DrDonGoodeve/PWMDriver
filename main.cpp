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

#define kPulseFrequency     (1.0f)
#define kOnBoardLED         (PICO_DEFAULT_LED_PIN) // GPIO25
#define kLEDUpdateFreq      (1000.0f)
#define kAudioPWM           (13)    // GPIO13, Pin17
#define kAudioUpdateFreq    (160.0e3f)
#define kSineFrequency      (8000.0f)
#define kPi                 (3.141592654f)

int giCalls = 0;

// PWM Source - main board LED
//-----------------------------------------------------------------------------
class Heartbeat : public PWMDriver::Source,
                  public PWMDriver::Group {
    
    private:
        bool mbCountingUp;
        float mfValue;
        float mfIncrement;

    public:
        Heartbeat(void) :
            PWMDriver::Source(kOnBoardLED),
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

class SineWave :  public PWMDriver::Source,
                  public PWMDriver::Group {
    
    private:
        uint muLUTIndex;
        float *mpfLUT;
        uint muLUTSize;

    public:
        SineWave(void) :
            PWMDriver::Source(kAudioPWM),
            mpfLUT(nullptr), muLUTIndex(0), muLUTSize(0) {

            addSource(this);
            PWMDriver::instance()->addGroup(this);

            //gpio_init(17);
            //gpio_set_dir(17, GPIO_OUT);
            //gpio_put(17, false);
            
            //gpio_init(18);
            //gpio_set_dir(18, GPIO_OUT);
            //gpio_put(18, false);
        }

        ~SineWave(void) {
            PWMDriver::instance()->removeGroup(this);
            removeSource(this);
            if (mpfLUT != nullptr) {
                delete []mpfLUT;
            }
        }

        // Subclass required implementations
        virtual float getDesiredUpdateFrequency(void) {
            return kAudioUpdateFreq;
        }

        virtual void resetSequence(void) {
            if (mpfLUT != nullptr) {
                delete []mpfLUT;
            }

            muLUTSize = ((uint)roundf(mfSampleRateHz / kSineFrequency));
            mpfLUT = new float[muLUTSize];
            muLUTIndex = 0;
            for(uint i=0; i<muLUTSize; i++) {
                float fPhase(((float)i / (float)muLUTSize) * (2.0f * kPi));
                mpfLUT[i] = 0.5f + (0.5f * sinf(fPhase));
                //mpfLUT[i] = (i<(muLUTSize/2)) ? 0.1f : 0.9f;
            }

            printf("muLUTSize = %d\r\n", muLUTSize);
        }

        virtual float getNextSequence(void) {
            giCalls++;
            //gpio_put(17, !gpio_get_out_level(17));
            float fReturn(mpfLUT[muLUTIndex++]);
            muLUTIndex = (muLUTIndex >= muLUTSize)?0:muLUTIndex;
            if (0 == muLUTIndex) {
                //gpio_put(18, !gpio_get_out_level(18));
            }
            return fReturn;
        }
};


int main(void) {
    // Initialize stdio functionality
    stdio_init_all();

    busy_wait_ms(10000);
    printf("Starting up...\r\n");

    Heartbeat cHeartbeat;
    SineWave cSineWave;

    // Everything after this point happens in the PWM interrupt handler, so we
    // can twiddle our thumbs
    while (true) {
        busy_wait_ms(1000);
        printf("Calls %d\r\n", giCalls);
        giCalls = 0;
        //tight_loop_contents();
    }

    return 0;
}