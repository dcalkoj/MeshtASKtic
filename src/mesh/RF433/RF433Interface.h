//Main head for RF433 Communication override.
#pragma once

#include "RadioLibInterface.h"
#include "RadioHead.h"
#include "RH_ASK.h"

#include "MeshPacketQueue.h"
#include "concurrency/NotifiedWorkerThread.h"

class RF433Interface : public RadioInterface, private concurrency::OSThread
{
    public:
    

    RF433Interface();

    virtual bool canSleep() override;

    virtual bool wideLora() { return false; }

    /// Prepare hardware for sleep.  Call this _only_ for deep sleep, not needed for light sleep.
    virtual bool sleep() override;

    virtual ErrorCode send(meshtastic_MeshPacket *p) override;

    //Not supported: cancelling
    virtual bool cancelSending(NodeNum from, PacketId id) override;

    virtual bool init() override;

    //Not supported: reconfiguring
    virtual bool reconfigure() {return false; }

    virtual float getFreq() override;

    protected:

    RH_ASK *RF433Driver;
    bool isReceiving = false;
    //RadioBuffer radioBuffer __attribute__((__aligned__));


    virtual void saveFreq(float savedFreq) override;

    public:
    uint32_t rxBad = 0, rxGood = 0, txGood = 0, txRelay = 0;
    MeshPacketQueue txQueue = MeshPacketQueue(MAX_TX_QUEUE);
    static RF433Interface *instance;

    private:
    //virtual void onNotify(uint32_t notification) override;
    int32_t runOnce() override;

    //ACT II. RadioLibInterface
    /*public:

    

    

    virtual void disableInterrupt();
    virtual void enableInterrupt(void (*)());
    virtual void startReceive();
    virtual bool isChannelActive();
    virtual bool isActivelyReceiving();

    private:
    void setTransmitDelay();

    

    /// Used as our notification from the ISR
    enum PendingISR { ISR_NONE = 0, ISR_RX, ISR_TX, TRANSMIT_DELAY_COMPLETED };


    void startTransmitTimer(bool withDelay = true);


    void startTransmitTimerSNR(float snr);

    void handleTransmitInterrupt();
    void handleReceiveInterrupt();

    static void timerCallback(void *p1, uint32_t p2);

    virtual void onNotify(uint32_t notification) override;


    virtual void startSend(meshtastic_MeshPacket *txp);

    meshtastic_QueueStatus getQueueStatus();

    protected:
    bool receiveDetected();
    virtual bool canSendImmediately();
    void completeSending();*/

};