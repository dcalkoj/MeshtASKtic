//Main head for RF433 Communication override.
#pragma once

#include "RadioLibInterface.h"
#include "RadioHead.h"
#include "RH_ASK.h"

template <class T> class RF433Interface : public RadioLibInterface
{
    public:
    RF433Interface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                    RADIOLIB_PIN_TYPE busy);

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    //// 2DO: Call RH_ASK init function
    virtual bool init() override;

    /// Apply any radio provisioning changes
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    //// 2DO: Not particularly needed for me?
    virtual bool reconfigure() override;

    /// Prepare hardware for sleep.  Call this _only_ for deep sleep, not needed for light sleep.
    virtual bool sleep() override;

    //// 2D0: HMMMM....
    //bool isIRQPending() override { return lora.getIrqFlags() != 0; }

    protected:
    RH_ASK RF433Driver;

    /**
     * Glue functions called from ISR land
     */
    virtual void disableInterrupt() override;

    /**
     * Enable a particular ISR callback glue function
     */
    virtual void enableInterrupt(void (*callback)()) override;

    /** can we detect a LoRa preamble on the current channel? */
    virtual bool isChannelActive() override;

    /** are we actively receiving a packet (only called during receiving state) */
    virtual bool isActivelyReceiving() override;

    /**
     * Start waiting to receive a message
     */
    virtual void startReceive() override;

    /**
     *  We override to turn on transmitter power as needed.
     */
    virtual void configHardwareForSend() override;

    /**
     * Add SNR data to received messages
     */
    virtual void addReceiveMetadata(meshtastic_MeshPacket *mp) override;

    virtual void setStandby() override;
};