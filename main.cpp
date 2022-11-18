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
#include <memory.h>
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "PWMDriver.h"

// Samples
#include "Bwowww.cpp"
#include "LaserShot1.cpp"
#include "LaserShot2.cpp"
#include "LaserShot3.cpp"

const unsigned char pTest[] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    32,32,32,32, 32,32,32,32, 32,32,32,32, 32,32,32,32,
    64,64,64,64, 64,64,64,64, 64,64,64,64, 64,64,64,64,
    96,96,96,96, 96,96,96,96, 96,96,96,96, 96,96,96,96, 
    128,128,128,128, 128,128,128,128, 128,128,128,128, 128,128,128,128, 
    160,160,160,160, 160,160,160,160, 160,160,160,160, 160,160,160,160,
    192,192,192,192, 192,192,192,192, 192,192,192,192, 192,192,192,192, 
    160,160,160,160, 160,160,160,160, 160,160,160,160, 160,160,160,160,
    128,128,128,128, 128,128,128,128, 128,128,128,128, 128,128,128,128, 
    96,96,96,96, 96,96,96,96, 96,96,96,96, 96,96,96,96, 
    64,64,64,64, 64,64,64,64, 64,64,64,64, 64,64,64,64,
    32,32,32,32, 32,32,32,32, 32,32,32,32, 32,32,32,32,
};
#define kpTestSize  (sizeof(pTest))

unsigned char pTest2[] = {
    64,64,64,64, 64,64,64,64, 64,64,64,64, 64,64,64,64,
    64,64,64,64, 64,64,64,64, 64,64,64,64, 64,64,64,64,
    64,64,64,64, 64,64,64,64, 64,64,64,64, 64,64,64,64,
    64,64,64,64, 64,64,64,64, 64,64,64,64, 64,64,64,64,
    64,64,64,64, 64,64,64,64, 64,64,64,64, 64,64,64,64,
    64,64,64,64, 64,64,64,64, 64,64,64,64, 64,64,64,64,
    64,64,64,64, 64,64,64,64, 64,64,64,64, 64,64,64,64,
    64,64,64,64, 64,64,64,64, 64,64,64,64, 64,64,64,64,
    128,128,128,128, 128,128,128,128, 128,128,128,128, 128,128,128,128, 
    128,128,128,128, 128,128,128,128, 128,128,128,128, 128,128,128,128, 
    128,128,128,128, 128,128,128,128, 128,128,128,128, 128,128,128,128, 
    128,128,128,128, 128,128,128,128, 128,128,128,128, 128,128,128,128, 
    128,128,128,128, 128,128,128,128, 128,128,128,128, 128,128,128,128, 
    128,128,128,128, 128,128,128,128, 128,128,128,128, 128,128,128,128, 
    128,128,128,128, 128,128,128,128, 128,128,128,128, 128,128,128,128, 
    128,128,128,128, 128,128,128,128, 128,128,128,128, 128,128,128,128, 
};
#define kpTest2Size  (sizeof(pTest2))



#define kClkFrequency       (125.0e6f)
#define kPulseFrequency     (1.0f)
#define kLEDUpdateFreq      (1000.0f)
#define kAudioUpdateFreq    (160.0e3f)
#define k8BitPWMUpdateFreq  (125.0e6f/256)
#define kSineFrequency      (1000.0f)
#define kPi                 (3.141592654f)

#define _min(a,b)           (((a)<(b))?(a):(b))


int giCalls = 0;
int giCalls2 = 0;


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
            //giCalls++;
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
        uint muDMATimer;

        // Handle interrupt on completion of transfer sequence - reset to top
        // and restart DMA (1 interrupt per cycle through mpLUT).
        static void _DMAISR(void) {
            giCalls++;
            if (mspInstance != nullptr) {
                giCalls2++;
                dma_channel_acknowledge_irq0(mspInstance->muDMAChannel);
                dma_channel_set_read_addr(mspInstance->muDMAChannel, mspInstance->mpLUT, true);
                //volatile uint16_t *pPWMCountRegister((volatile uint16_t*)&(pwm_hw->slice[mspInstance->muSlice].cc));
                //uint16_t uCount(*pPWMCountRegister);
                //*pPWMCountRegister = (10+(giCalls/10)) % 700;
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
            pwm_set_irq_enabled(muSlice, false);    // Ensure Wrap DMA for this slice is disabled
            PWMDriver::Source::configure(); // Setup PWM GPIO configuration and timing

            // Grab and setup DMA channel
            muDMAChannel = dma_claim_unused_channel(true);
            dma_channel_config cDMAConfig(dma_channel_get_default_config(muDMAChannel));
            channel_config_set_transfer_data_size(&cDMAConfig, DMA_SIZE_16);
            channel_config_set_read_increment(&cDMAConfig, true);
            channel_config_set_write_increment(&cDMAConfig, false);

            // Grab and setup DMA pacing timer (at audio rate - 44.1kHz)
            muDMATimer = dma_claim_unused_timer(true);
            dma_timer_set_fraction(muDMATimer, 15, 42517);  // Pretty damn exact...

            // Attach timer to DMA channel
            channel_config_set_dreq(&cDMAConfig, dma_get_timer_dreq(muDMATimer));
            volatile void *pPWMCountRegister(&(pwm_hw->slice[muSlice].cc));
            printf("target address for PWM count register 0x%08x\r\n", (uint32_t)pPWMCountRegister);
            dma_channel_configure(
                muDMAChannel,               // Channel to be configured
                &cDMAConfig,                // The configuration we just created
                pPWMCountRegister,          // The initial write address
                mpLUT,                      // The initial read address
                muLUTSize,                  // Number of transfers; 16-bits each
                false                       // Do not start immediately.
            );
            printf("DMAChannelConfigure mpLUT=0x%08x, muLUTSize=%d\r\n", mpLUT, muLUTSize);

            // Completion interrupt via DMA_IRQ0
            dma_channel_set_irq0_enabled(muDMAChannel, true);
            irq_set_exclusive_handler(DMA_IRQ_0, _DMAISR);
            irq_set_enabled(DMA_IRQ_0, true);
        }

        virtual bool start(void) {
            if (true == PWMDriver::Source::start()) {
                printf("DMASineWave::start\r\n");
                dma_channel_start(muDMAChannel);
                return true;
            }
            return false;
        }

        virtual bool halt(void) {
            if (true == PWMDriver::Source::halt()) {
                pwm_set_enabled(muSlice, false);
                dma_channel_abort(muDMAChannel);
                return true;
            }
            return false;
        }

        // Overrides that manage DMA channel sequencing
        virtual void resetSequence(void) {
            if (mpLUT != nullptr) {
                delete []mpLUT;
            }

            printf("resetSequence\r\n");
            muLUTSize = ((uint)roundf(44.1e3f / kSineFrequency));
            mpLUT = new uint16_t[muLUTSize];
            float fMaxValue((float)muWrapValue);
            for(uint i=0; i<muLUTSize; i++) {
                float fPhase(((float)i / (float)muLUTSize) * (2.0f * kPi));
                float fValue(0.5f + (0.5f * sinf(fPhase)));
                uint16_t uValue((uint16_t)roundf(fValue * fMaxValue));
                mpLUT[i] = uValue;
             }
            printf("\r\n");

            printf("DMASineWave muLUTSize = %d\r\n", muLUTSize);
        }

        // Required - but not used (replaced by DMA)
        virtual float getNextSequence(void) {
            printf("GetNextSequence\r\n");
            return 0.5f;
        }
};
DMASineWave *DMASineWave::mspInstance = nullptr;// DMA-driven sinewave source


//-------------------------------------
#define kSampleBufferSize   (1024)
class SamplePlayer :  public PWMDriver::Source,
                      public PWMDriver::Group {
    private:
        // Source sample information
        const uint8_t *mpSample;
        uint muSamples;
        uint muSampleRate;

        // DMA channel and timer
        static SamplePlayer *mspInstance;
        uint muDMAChannel;
        uint muDMATimer;

        // Double-buffer (16-bit)
        uint16_t mpBuffer[2 * kSampleBufferSize];
        const uint16_t *mpActiveBuffer;
        const uint16_t *mpPreparedBuffer;

        // Read state over samples
        uint muSamplesRemaining;
        const uint8_t *mpSampleRead;
        bool mbRepeat;
        bool mbPlaying;

        // If exhausted sample stream, returns false
        inline bool _copyTo16BitBuffer(uint16_t *pDest, uint uCopySize) {
            //printf("_copyTo16BitBuffer(pDest=0x%08x, uCopySize=%d)\r\n", (uint)pDest, uCopySize);
            if (0 == uCopySize) {
                return true;
            }

            if ((0 == muSamplesRemaining) && (false == mbRepeat)) {
                return false;
            }

            uint uThisCopySize(_min(uCopySize, muSamplesRemaining));
            uint16_t *pWr(pDest);
            for(uint i=0; i<uThisCopySize; i++) {
                *(pWr++) = (uint16_t)*(mpSampleRead++);
            }
            muSamplesRemaining -= uThisCopySize;

            // Handle buffer exhaustion - always return a complete buffer
            if (uThisCopySize < uCopySize) {
                uint uRemainder(uCopySize-uThisCopySize);
                if (false == mbRepeat) {
                    memset(pWr, 0x00, sizeof(uint16_t)*uRemainder);
                } else {
                    mpSampleRead = mpSample;
                    muSamplesRemaining = muSamples;
                    return _copyTo16BitBuffer(pWr, uRemainder);
                }
            }

            return true;
        }

        const uint16_t *prepareNextBuffer(void) {
            uint16_t *pPreparedBuffer((mpActiveBuffer != mpBuffer)?mpBuffer:&mpBuffer[kSampleBufferSize]);
            if (false == _copyTo16BitBuffer(pPreparedBuffer, kSampleBufferSize)) {
                return nullptr;
            }
            return pPreparedBuffer;
        }

        inline void handleIRQ(void) {
           mpActiveBuffer = mpPreparedBuffer;
           if (mpActiveBuffer != nullptr) {
                dma_channel_set_read_addr(muDMAChannel, mpActiveBuffer, true);
                mpPreparedBuffer = prepareNextBuffer();
           } else {
                giCalls2++;
                halt();
           }
        }

        // Handle interrupt on completion of transfer sequence.
        static void _DMAISR(void) {
            giCalls++;
            if (mspInstance != nullptr) {
                dma_channel_acknowledge_irq0(mspInstance->muDMAChannel);
                mspInstance->handleIRQ();
            }
        }

        static void _computeDMATimerNumDen(uint &uSampleRate, uint16_t &uNum, uint16_t &uDen) {
            float fTarget((float)uSampleRate), fMinError(1000.0f);
            int iBestDen(0), iBestNum(0);
            for (int iNum = 1; iNum < 65536; iNum++) {
                int iDen((int)roundf((kClkFrequency * (float)iNum) / fTarget));
                if (iDen >= 0x10000) {
                    break;
                }
                float fResult((kClkFrequency * iNum) / iDen);
                float fError(fabs(fTarget - fResult));
                if (fError < fMinError) {
                    fMinError = fError;
                    iBestDen = iDen;
                    iBestNum = iNum;
                }
            }
            uNum = (uint)iBestNum;
            uDen = (uint)iBestDen;
        }

    public:
        SamplePlayer(uint uGPIO, uint uSampleRate) :
            PWMDriver::Source(uGPIO),
            mpSample(nullptr), muSamples(0), muSampleRate(uSampleRate), mbRepeat(false),
            mbPlaying(false), mpActiveBuffer(nullptr), mpPreparedBuffer(nullptr) {

            addSource(this);
            mspInstance = this;
        }

        ~SamplePlayer(void) {
            PWMDriver::instance()->removeGroup(this);
            removeSource(this);

            mspInstance = nullptr;
        }

        void playSample(const uint8_t *pSample, uint uSamples, bool bRepeat) {
            halt();
            mpSample = pSample;
            muSamples = uSamples;
            mbRepeat = bRepeat;
            
            // Prepare sample buffer (from the top)
            mpSampleRead = mpSample;
            muSamplesRemaining = muSamples;
            mpActiveBuffer = mpPreparedBuffer = nullptr;
            mpActiveBuffer = prepareNextBuffer();
            mpPreparedBuffer = prepareNextBuffer();
            printf("mpActiveBuffer = 0x%08x, mpPreparedBuffer = 0x%08x\r\n", (uint)mpActiveBuffer, (uint)mpPreparedBuffer);

            PWMDriver::instance()->addGroup(this);  // Only fires first time...

            start();
        }

        //void stopPlaying(void) {
        //    if (true == mbPlaying) {
        //        PWMDriver::instance()->removeGroup(this);
        //        mbPlaying = false;
        //    }
        //}

        // Subclass required implementations
        virtual float getDesiredUpdateFrequency(void) {
            return k8BitPWMUpdateFreq;
        }

        // Override ensures that IRQ is not enabled
        virtual void configure(void) {
            printf("SamplePlayer configure\r\n");
            pwm_set_irq_enabled(muSlice, false);    // Ensure Wrap DMA for this slice is disabled
            PWMDriver::Source::configure(); // Setup PWM GPIO configuration and timing

            // Grab and setup DMA channel
            muDMAChannel = dma_claim_unused_channel(true);
            dma_channel_config cDMAConfig(dma_channel_get_default_config(muDMAChannel));
            channel_config_set_transfer_data_size(&cDMAConfig, DMA_SIZE_16); //!!!!!
            channel_config_set_read_increment(&cDMAConfig, true);
            channel_config_set_write_increment(&cDMAConfig, false);

            // Grab and setup DMA pacing timer (at audio rate - 44.1kHz)
            muDMATimer = dma_claim_unused_timer(true);
            uint16_t uNum(0), uDen(0);
            _computeDMATimerNumDen(muSampleRate, uNum, uDen);
            float fActualSampleRate((kClkFrequency * (float)uNum) / (float)uDen);
            printf("Timer X/Y for required sample rate %d = %d/%d - giving %f\r\n", muSampleRate, uNum, uDen, fActualSampleRate);
            dma_timer_set_fraction(muDMATimer, uNum, uDen);  // Should be pretty damn exact...

            // Attach timer to DMA channel
            channel_config_set_dreq(&cDMAConfig, dma_get_timer_dreq(muDMATimer));
            volatile void *pPWMCountRegister(&(pwm_hw->slice[muSlice].cc));
            uint uChannel(pwm_gpio_to_channel(muGPIO));
            if (uChannel != 0) {    // Channel B on higher 16-bits
                pPWMCountRegister = (void*)((uint)pPWMCountRegister+2);
            }

            //for(uint i=0; i<4; i++) {
            //    mpActiveBuffer = mpPreparedBuffer;
            //    mpPreparedBuffer = prepareNextBuffer();
            //    printf("iter mpActiveBuffer = 0x%08x, mpPreparedBuffer = 0x%08x\r\n", (uint)mpActiveBuffer, (uint)mpPreparedBuffer);               
            //}

            printf("--GPIO=%d, channel=%d, pPWMCountRegister=0x%08x\r\n", muGPIO, uChannel, (uint)pPWMCountRegister);
            dma_channel_configure(
                muDMAChannel,               // Channel to be configured
                &cDMAConfig,                // The configuration we just created
                pPWMCountRegister,          // The initial write address 8-bits
                mpActiveBuffer,             // The initial read address
                kSampleBufferSize,          // Number of transfers; 16-bits each
                false                       // Do not start immediately.
            );

            // Completion interrupt via DMA_IRQ0
            dma_channel_set_irq0_enabled(muDMAChannel, true);
            irq_set_exclusive_handler(DMA_IRQ_0, _DMAISR);
            irq_set_enabled(DMA_IRQ_0, true);
        }

        virtual bool start(void) {
            PWMDriver::Source::start();
            pwm_set_enabled(muSlice, true);
            //pwm_set_gpio_level(muGPIO, 0);
            //dma_channel_start(muDMAChannel);    
            dma_channel_set_read_addr(muDMAChannel, mpActiveBuffer, true);
 
            return true;       
            //if (true == PWMDriver::Source::start()) {
            //    pwm_set_gpio_level(muGPIO, 0);
            //    dma_channel_start(muDMAChannel);
            //    return true;
            //}
            //return false;
        }

        virtual bool halt(void) {
            dma_channel_abort(muDMAChannel);
            pwm_set_enabled(muSlice, false);
            //if (true == PWMDriver::Source::halt()) {
            //    pwm_set_enabled(muSlice, false);
            //    dma_channel_abort(muDMAChannel);
            //    return true;
            //}
            //return false;
            return true;
        }

        // Overrides that manage DMA channel sequencing
        virtual void resetSequence(void) {
        }

        // Required - but not used (replaced by DMA)
        virtual float getNextSequence(void) {
            printf("GetNextSequence\r\n");
            return 0.5f;
        }
};
SamplePlayer *SamplePlayer::mspInstance = nullptr;


int main(void) {
    // Initialize stdio functionality
    stdio_init_all();

    busy_wait_ms(10000);
    printf("Starting up...\r\n");

    Heartbeat cHeartbeat(PICO_DEFAULT_LED_PIN);
    //SineWave cSineWave(14); // Pin 19
    //DMASineWave cDMASineWave(14); // Pin 19
    SamplePlayer cLaser(14, 16000);
    //cLaser.playSample(pBwowww, kBwowwwSize, 16000, true);
    //cLaser.playSample(pLaserShot1, kLaserShot1Size, 16000, true);
    //cLaser.playSample(pLaserShot2, kLaserShot2Size, 16000, true);
    while(true) {
        cLaser.playSample(pBwowww, kBwowwwSize, true);
        busy_wait_ms(3000);
        cLaser.playSample(pLaserShot1, kLaserShot1Size, true);
        busy_wait_ms(1000);
        cLaser.playSample(pLaserShot2, kLaserShot2Size, true);
        busy_wait_ms(1000);
        cLaser.playSample(pLaserShot3, kLaserShot3Size, false);
        busy_wait_ms(1000);
        printf("Calls %d, Calls2 %d\r\n", giCalls, giCalls2);
        giCalls = 0; giCalls2 = 0;
    }
    ///uint8_t pRamp[256];
    //for(uint i=0; i<256; i++) {
    //    pRamp[i] = i;
   // }
    //cLaser.playSample((uint8_t*)pRamp, 256, 16000, true);

    // Everything after this point happens in the PWM interrupt handler, so we
    // can twiddle our thumbs
    while (true) {
        busy_wait_ms(1000);
        //tight_loop_contents();
    }

    return 0;
}