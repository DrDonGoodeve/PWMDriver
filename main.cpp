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
        bool mbActivityHigh;

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
            float fMultiplier((false==mbActivityHigh)?4.0f:255.0f);
            uint8_t uRed((uint8_t)roundf(fRed*fMultiplier));
            uint8_t uGreen((uint8_t)roundf(fGreen*fMultiplier));
            uint8_t uBlue((uint8_t)roundf(fBlue*fMultiplier));

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
            mbActivityHigh(false), mfPhase(0.0f), mfAmplitude(0.1f), mbRising(true), mfPhaseShift(kPi/1.3f) {
            mspSelf = this;

            // Startup colourful heartbeat
            add_repeating_timer_ms(10, timerCallback, NULL, &mcHeartbeatTimer);
        }

        ~Heartbeat() {
            cancel_repeating_timer(&mcHeartbeatTimer);
            mspSelf = nullptr;
        }

        static void setActivityLevel(bool bHighNotLow) {
            if (mspSelf != nullptr) {
                mspSelf->mbActivityHigh = bHighNotLow;
            }
        }
};
Heartbeat *Heartbeat::mspSelf = nullptr;

#define kRFRelayGPIO    (10)
#define kPushButtonGPIO (9)

#define kLaserGPIO      (1)
#define kLED1GPIO       (7)
#define kLED3GPIO       (5)
#define kLED4GPIO       (6)
#define kLED2GPIO       (4)
#define kLED5GPIO       (8)

class SequencedLED : public PWMDriver::UpdateOnWrapSource {
    public:
        class Step {
            public:
                float mfFinalLevel;
                uint muTransitionMsec;
                Step(float fFinalLevel, uint uTransitionMsec) :
                    mfFinalLevel(fFinalLevel), muTransitionMsec(uTransitionMsec) {
                }
        };

    private:
        std::list<Step> mlProgram;
        std::list<Step> mlSequence;
        float mfPreDelaySec;
        bool mbRepeat;

        float mfSecondsLeftInTransition;
        float mfTargetLevel;
        float mfCurrentLevel;

    public:
        SequencedLED(uint uGPIO) :
            UpdateOnWrapSource(uGPIO),
            mfSecondsLeftInTransition(0.0f), mfTargetLevel(0.0f), mfCurrentLevel(0.0f),
            mbRepeat(false), mfPreDelaySec(0.0f) {
        }

        ~SequencedLED() {
        }

        void setSequence(const std::list<Step> &lSteps=std::list<Step>{}, float fPreDelaySec=0.0f, bool bRepeat=true) {
            mlProgram = lSteps;
            mfPreDelaySec = fPreDelaySec;
            mbRepeat = bRepeat;

            resetSequence();
        }

        virtual void resetSequence(void) {
            mlSequence = mlProgram;
            mfTargetLevel = mfCurrentLevel;
            mfSecondsLeftInTransition = mfPreDelaySec;

        }

        virtual float getNextSequence(void) {
            if (mfSecondsLeftInTransition <= 0.0f) {
                if (false == mlSequence.empty()) {
                    Step cLastStep(mlSequence.front());
                    mlSequence.pop_front();
                    if (true == mbRepeat) {
                        mlSequence.push_back(cLastStep);
                    }
                    if (false == mlSequence.empty()) {
                        Step &cThisStep(mlSequence.front());
                        mfTargetLevel = cLastStep.mfFinalLevel;
                        mfSecondsLeftInTransition = ((float)cThisStep.muTransitionMsec / 1000.0f);
                    } else {
                        mfSecondsLeftInTransition = 0.0f;
                        mfCurrentLevel = mfTargetLevel;
                    }
                } else {
                    mfCurrentLevel = mfTargetLevel = 0.0f;
                }
                printf("fSec=%.3f, mfCL=%.3f, mfTL=%.3f\r\n", mfSecondsLeftInTransition, mfCurrentLevel, mfTargetLevel);
            }
            if (mfSecondsLeftInTransition > 0.0f) {
                float fCallbackPeriod(1.0f / getPWMRateHz());
                float fCallsRemaining(mfSecondsLeftInTransition / fCallbackPeriod);
                float fDelta((mfTargetLevel - mfCurrentLevel) / fCallsRemaining);
                mfCurrentLevel += fDelta;
                mfSecondsLeftInTransition -= fCallbackPeriod;
            }
            return mfCurrentLevel;
        }
};

static const std::list<SequencedLED::Step> slShortAlive {
    SequencedLED::Step(0.01f,1000), SequencedLED::Step(0.0f,1000), SequencedLED::Step(0.0f,1500)
};

static const std::list<SequencedLED::Step> slSlowLowPulse {
    SequencedLED::Step(0.2f,400), SequencedLED::Step(0.3f,400), SequencedLED::Step(0.2f,400), SequencedLED::Step(0.0f,400), SequencedLED::Step(0.0f,400)
};

static const std::list<SequencedLED::Step> slLowHeartbeatPulse {
    SequencedLED::Step(0.3f,100), SequencedLED::Step(0.0f,100),
    SequencedLED::Step(0.5f,100), SequencedLED::Step(0.0f,100),
    SequencedLED::Step(0.0f,500)
};

static const std::list<SequencedLED::Step> slHighHeartbeatPulse {
    SequencedLED::Step(0.7f,100), SequencedLED::Step(0.0f,100),
    SequencedLED::Step(1.0f,100), SequencedLED::Step(0.0f,100),
    SequencedLED::Step(0.0f,300)
};

static const std::list<SequencedLED::Step> slSlowHighPulse {
    SequencedLED::Step(0.3f,200), SequencedLED::Step(0.5f,200), SequencedLED::Step(0.8f,200),
    SequencedLED::Step(1.0f,200), SequencedLED::Step(1.0f,200), 
    SequencedLED::Step(0.8f,200), SequencedLED::Step(0.5f,200), SequencedLED::Step(0.3f,200), 
    SequencedLED::Step(0.0f,200), SequencedLED::Step(0.0f,200)
};

static const std::list<SequencedLED::Step> slSlowSustainPulse {
    SequencedLED::Step(0.3f,200), SequencedLED::Step(0.5f,200), SequencedLED::Step(0.8f,200),
    SequencedLED::Step(1.0f,200), SequencedLED::Step(1.0f,200), 
    SequencedLED::Step(0.8f,200), SequencedLED::Step(0.5f,200), SequencedLED::Step(0.3f,200),
};

static const std::list<SequencedLED::Step> slFastHighPulse {
    SequencedLED::Step(0.3f,100), SequencedLED::Step(0.5f,100), SequencedLED::Step(0.8f,100),
    SequencedLED::Step(1.0f,100), SequencedLED::Step(1.0f,100), 
    SequencedLED::Step(0.8f,100), SequencedLED::Step(0.5f,100), SequencedLED::Step(0.3f,100), 
    SequencedLED::Step(0.0f,100), SequencedLED::Step(0.0f,100)
};

class LEDGroup : public PWMDriver::Group {
    private:
        uint muPattern;
        SequencedLED mcLaser;
        SequencedLED mcLED1;
        SequencedLED mcLED2;
        SequencedLED mcLED3;
        SequencedLED mcLED4;
        SequencedLED mcLED5;

    public:
        LEDGroup(void) :
            mcLaser(kLaserGPIO), mcLED1(kLED1GPIO), mcLED2(kLED2GPIO), mcLED3(kLED3GPIO), mcLED4(kLED4GPIO), mcLED5(kLED5GPIO) {
            addSource(&mcLaser);
            addSource(&mcLED1);
            addSource(&mcLED2);
            addSource(&mcLED3);
            addSource(&mcLED4);
            addSource(&mcLED5);
            resetPattern();
        }

        virtual float getDesiredUpdateFrequency(void) const {
            return kLEDUpdateFreq;
        }

        void resetPattern(void) {
            muPattern = 0;
            mcLaser.setSequence();
            mcLED1.setSequence();
            mcLED2.setSequence();
            mcLED3.setSequence();
            mcLED4.setSequence();
            mcLED5.setSequence(slShortAlive);
        }

        void nextPattern(void) {
            muPattern++;
            switch(muPattern) {
                case 1: // Laser low heartbeat
                    mcLED1.setSequence();
                    mcLaser.setSequence(slLowHeartbeatPulse);
                    break;
                case 2: // Laser getting excited
                    mcLaser.setSequence(slHighHeartbeatPulse);
                    break;
                case 3: // Blue joins in
                    mcLaser.setSequence(slHighHeartbeatPulse);
                    mcLED1.setSequence(slLowHeartbeatPulse, 0.1f);
                    break;
                case 4: // Low Slow flow up
                    mcLED1.setSequence(slSlowLowPulse, 0.0f);
                    mcLED2.setSequence(slSlowLowPulse, 0.3f);
                    mcLED3.setSequence(slSlowLowPulse, 0.6f);
                    mcLED4.setSequence(slSlowLowPulse, 0.9f);
                    mcLED5.setSequence(slSlowLowPulse, 1.2f);
                    break;        
                case 5:
                    mcLED1.setSequence(slSlowHighPulse, 0.0f);
                    mcLED2.setSequence(slSlowHighPulse, 0.2f);
                    mcLED3.setSequence(slSlowHighPulse, 0.4f);
                    mcLED4.setSequence(slSlowHighPulse, 0.6f);
                    mcLED5.setSequence(slSlowHighPulse, 0.8f);
                    break;
                case 6:
                    mcLED1.setSequence(slFastHighPulse, 0.0f);
                    mcLED2.setSequence(slFastHighPulse, 0.15f);
                    mcLED3.setSequence(slFastHighPulse, 0.3f);
                    mcLED4.setSequence(slFastHighPulse, 0.45f);
                    mcLED5.setSequence(slFastHighPulse, 0.6f);
                    break;
                case 7:
                    mcLED1.setSequence(slFastHighPulse, 0.3f);
                    mcLED2.setSequence(slFastHighPulse, 0.15f);
                    mcLED3.setSequence(slFastHighPulse, 0.0f);
                    mcLED4.setSequence(slFastHighPulse, 0.15f);
                    mcLED5.setSequence(slFastHighPulse, 0.3f);
                    break;
                case 8:
                default:
                    mcLED1.setSequence(slFastHighPulse, 0.6f);
                    mcLED2.setSequence(slFastHighPulse, 0.45f);
                    mcLED3.setSequence(slFastHighPulse, 0.3f);
                    mcLED4.setSequence(slFastHighPulse, 0.15f);
                    mcLED5.setSequence(slFastHighPulse, 0.0f);
                    break;
            }
        }


}scMagicBroomLights;


class RFRelay : public SwitchScanner::Switch {
    public:
        RFRelay(void) :
            Switch(kRFRelayGPIO, kPullUp, false) {
        }

        void press(void) {
            printf("RFRelay down\r\n");
            Heartbeat::setActivityLevel(true);
            scMagicBroomLights.nextPattern();
        }

        void release(void) {
            printf("RFRelay release\r\n");
        }

        void hold(void) {
            printf("RFRelay hold\r\n");
            Heartbeat::setActivityLevel(false);
            scMagicBroomLights.resetPattern();
        }
};

class PushButton: public SwitchScanner::Switch {
    public:
        PushButton(void) :
            Switch(kPushButtonGPIO, kPullUp, false) {
        }

        void press(void) {
            printf("PushButton down\r\n");
            Heartbeat::setActivityLevel(true);
            scMagicBroomLights.nextPattern();
        }

        void release(void) {
            printf("PushButton release\r\n");
        }

        void hold(void) {
            printf("PushButton hold\r\n");
            scMagicBroomLights.resetPattern();
            Heartbeat::setActivityLevel(false);
        }
};


int main(void) {
    // Initialize stdio functionality
    stdio_init_all();

    Heartbeat cHeartbeat;

    // Buttons
    PushButton cPushButton;
    RFRelay cRFRelay;
    SwitchScanner cScanner;
    cScanner.addSwitch(&cPushButton);
    cScanner.addSwitch(&cRFRelay);
    cScanner.setScanActive(true);

    // LEDs and laser
    PWMDriver::instance()->addGroup(&scMagicBroomLights);
    scMagicBroomLights.start();


    while(true) {
        tight_loop_contents();
    }
}