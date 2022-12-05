/******************************************************************************
 * SwitchScanner.h
 * 
 * Switch debounce and scanning. Handles setup of GPIO for switch hardware
 * and provides a means of attaching handlers to switch events. Also provides
 * debounce functionality.
 * 
 * ****************************************************************************
 * Code Copyright(C) 2022 Don Goodeve. Open source - no restrictions
 * ***************************************************************************/

#include "pico/stdlib.h"
#include <map>


// Defines
//-----------------------------------------------------------------------------
#define kSwitchScanVectorDepth      (3)
#define kSwitchScanInterval         (10)        // msec
#define kHoldUsec                   (300000)    // usec
#define kDoublePressMaxInterval     (200000)


// SwitchScanner class
//-----------------------------------------------------------------------------
class SwitchScanner {
    public:
        class Switch {
            public:
                typedef enum {
                    kNoPull,
                    kPullUp,
                    kPullDown
                }Pull;       

            private:
                uint muGPIO;
                bool mbPositiveLogic;

                // Dynamic state
                friend class SwitchScanner;
                bool mbIsPressed;
                bool mbIsHeld;
                uint muLastPressTime;
                uint muLastReleaseTime;

                void assertedHigh(void);
                void assertedLow(void);

            public:
                Switch(uint uGPIO, Pull ePull=kNoPull, bool bPositiveLogic=true);
                ~Switch();

                uint getGPIO(void) const;
                bool isPositiveLogic(void) const;

                // Delegate to user-supplied methods
                void pressWrapper(void);
                void releaseWrapper(void);

                // User-override button handling methods
                virtual void press(void);
                virtual void release(void);
                virtual void hold(void);
                virtual void doublePress(void);
        };

    private:
        static SwitchScanner *mspInstance;

        struct repeating_timer mcButtonTimer;
        static bool timerCallback(struct repeating_timer *pTimer);
        bool switchScan(void);

        bool mbScanActive;
        std::map<uint,Switch*> mmSwitches;
        uint32_t muSwitchMask;

        // Data used to track consistency and changes
        uint32_t mpGPIOVector[kSwitchScanVectorDepth];
        uint32_t muStateVector;
        uint muGPIOVectorWriteIndex;

    public:
        SwitchScanner(void);
        ~SwitchScanner();

        void setScanActive(bool bActive=true);

        bool addSwitch(Switch *pSwitch);
        bool removeSwitch(Switch *pSwitch);
};
