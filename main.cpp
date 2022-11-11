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
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "PWMDriver.h"

#define kPulseFrequency     (1.0f)
#define kLEDUpdateFreq      (1000.0f)
#define kAudioUpdateFreq    (160.0e3f)
#define kSineFrequency      (1000.0f)
#define kPi                 (3.141592654f)

int giCalls = 0;

// PWM Source - main board LED
//-----------------------------------------------------------------------------
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

// High-speed sinewave using PWMWrap interrupt driven implementation
//-------------------------------------
class SineWave :  public PWMDriver::Source,
                  public PWMDriver::Group {
    
    private:
        uint muLUTIndex;
        float *mpfLUT;
        uint muLUTSize;

    public:
        SineWave(uint uGPIO) :
            PWMDriver::Source(uGPIO),
            mpfLUT(nullptr), muLUTIndex(0), muLUTSize(0) {

            addSource(this);
            PWMDriver::instance()->addGroup(this);
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
            }

            printf("muLUTSize = %d\r\n", muLUTSize);
        }

        virtual float getNextSequence(void) {
            giCalls++;
            float fReturn(mpfLUT[muLUTIndex++]);
            muLUTIndex = (muLUTIndex >= muLUTSize)?0:muLUTIndex;
            return fReturn;
        }
};


// DMA-driven sinewave source
//-------------------------------------
class DMASineWave :  public PWMDriver::Source,
                     public PWMDriver::Group {
    
    private:
        uint16_t *mpLUT;
        uint muLUTSize;
        static DMASineWave *mspInstance;
        uint muDMAChannel;

        // Handle interrupt on completion of transfer sequence - reset to top
        // and restart DMA (1 interrupt per cycle through mpLUT).
        static void _DMAISR(void) {
            if (mspInstance != nullptr) {
                dma_channel_acknowledge_irq0(mspInstance->muDMAChannel);
                dma_channel_set_read_addr(mspInstance->muDMAChannel, mspInstance->mpLUT, true);
            }
        }

    public:
        DMASineWave(uint uGPIO) :
            PWMDriver::Source(uGPIO),
            mpLUT(nullptr), muLUTSize(0) {

            addSource(this);
            PWMDriver::instance()->addGroup(this);

            mspInstance = this;
        }

        ~DMASineWave(void) {
            PWMDriver::instance()->removeGroup(this);
            removeSource(this);
            if (mpLUT != nullptr) {
                delete []mpLUT;
            }
            mspInstance = nullptr;
        }

        // Subclass required implementations
        virtual float getDesiredUpdateFrequency(void) {
            return kAudioUpdateFreq;
        }

        // Override ensures that IRQ is not enabled
        virtual void configure(void) {
            printf("DMASineWave configure\r\n");
            PWMDriver::Source::configure();
            pwm_set_irq_enabled(muSlice, false);    // Ensure Wrap DMA for this slice is disabled

            // Grab DMA channel
            muDMAChannel = dma_claim_unused_channel(true);  // Required...
            dma_channel_config cDMAConfig(dma_get_channel_config(muDMAChannel));
            channel_config_set_transfer_data_size(&cDMAConfig, DMA_SIZE_16);
            channel_config_set_read_increment(&cDMAConfig, true);
            channel_config_set_write_increment(&cDMAConfig, false);
            channel_config_set_dreq(&cDMAConfig, DREQ_PWM_WRAP0 + muSlice);
            dma_channel_configure(
                muDMAChannel,               // Channel to be configured
                &cDMAConfig,                // The configuration we just created
                &pwm_hw->slice[muSlice].cc, // The initial write address
                mpLUT,                      // The initial read address
                muLUTSize,                  // Number of transfers; 16-bits each
                false                       // Do not start immediately.
            );

            // Completion interrupt via DMA_IRQ0
            irq_set_exclusive_handler(DMA_IRQ_0, _DMAISR);
        }

        virtual void start(void) {
            if (false == PWMDriver::Source::mbRunning) {
                dma_channel_set_irq0_enabled(muDMAChannel, true);
                dma_channel_start(muDMAChannel);
                pwm_set_enabled(muSlice, true);
                PWMDriver::Source::mbRunning = true;
            }
        }

        virtual void halt(void) {
            if (true == PWMDriver::Source::mbRunning) {
                pwm_set_enabled(muSlice, false);
                dma_channel_set_irq0_enabled(muDMAChannel, false);
                dma_channel_abort(muDMAChannel);
                PWMDriver::Source::mbRunning = false;
            }
        }

        // Overrides that manage DMA channel sequencing
        virtual void resetSequence(void) {
            if (mpLUT != nullptr) {
                delete []mpLUT;
            }

            muLUTSize = ((uint)roundf(mfSampleRateHz / kSineFrequency));
            mpLUT = new uint16_t[muLUTSize];
            float fMaxValue((float)muWrapValue);
            for(uint i=0; i<muLUTSize; i++) {
                float fPhase(((float)i / (float)muLUTSize) * (2.0f * kPi));
                float fValue(0.5f + (0.5f * sinf(fPhase)));
                uint16_t uValue((uint16_t)roundf(fValue * fMaxValue));
                mpLUT[i] = uValue;
            }

            printf("muLUTSize = %d\r\n", muLUTSize);
        }

        // Required - but not used (replaced by DMA)
        virtual float getNextSequence(void) {
            return 0.0f;
        }
};
DMASineWave *DMASineWave::mspInstance = nullptr;


int main(void) {
    // Initialize stdio functionality
    stdio_init_all();

    busy_wait_ms(10000);
    printf("Starting up...\r\n");

    Heartbeat cHeartbeat(PICO_DEFAULT_LED_PIN);
    //SineWave cSineWave(14); // Pin 19
    DMASineWave cDMASineWave(13); // Pin 17

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