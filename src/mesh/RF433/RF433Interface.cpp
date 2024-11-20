#include "RF433Interface.h"
#include "configuration.h"
#include "error.h"
#include "mesh/NodeDB.h"

#include "Throttle.h"

//ACT I. OVERRIDE METHODS FROM SX126XINTERFACE
template <typename T>
RF433Interface<T>::RF433Interface(LockingArduinoHal *hal, RADIOLIB_PIN_TYPE cs, RADIOLIB_PIN_TYPE irq, RADIOLIB_PIN_TYPE rst,
                                    RADIOLIB_PIN_TYPE busy) : RadioLibInterface(hal, cs, irq, rst, busy)
{
    LOG_DEBUG("RF33Interface(cs=%d, irq=%d, rst=%d, busy=%d)", cs, irq, rst, busy);   
}

/// Initialise the Driver transport hardware and software.
/// Make sure the Driver is properly configured before calling init().
/// \return true if initialisation succeeded.
template <typename T> bool RF433Interface<T>::init()
{
    if(!RF433Driver.init()){
        LOG_DEBUG("RH_ASK_INIT FAILED!");
        return false;
    }
    return true;
}

template <typename T> bool RF433Interface<T>::reconfigure()
{
    return true;
}

template <typename T> void INTERRUPT_ATTR RF433Interface<T>::disableInterrupt(){
    NVIC_DisableIRQ(TIMER2_IRQn);
}

template <typename T> void INTERRUPT_ATTR RF433Interface<T>::enableInterrupt(void (*)()){
    NVIC_EnableIRQ(TIMER2_IRQn);
}

template <typename T> void RF433Interface<T>::setStandby(){
    checkNotification();

    RF433Driver.setModeIdle();
    isReceiving = false; // If we were receiving, not any more
    activeReceiveStart = 0;
    disableInterrupt();
    completeSending(); // If we were sending, not anymore
    RadioLibInterface::setStandby();
}

/**
 * Add SNR data to received messages
 */
template <typename T> void RF433Interface<T>::addReceiveMetadata(meshtastic_MeshPacket *mp)
{
    //LOG_DEBUG("PacketStatus %x", lora.getPacketStatus());
    LOG_DEBUG("addReceiveMetadata: SNR not available for ASK");
    mp->rx_snr = 433;
    mp->rx_rssi = 433;
} 

/** We override to turn on transmitter power as needed.
 */
// 2Do: Enable Sendmode? DONE..verify.
template <typename T> void RF433Interface<T>::configHardwareForSend()
{
    RF433Driver.setModeTx();
    RadioLibInterface::configHardwareForSend();
}


template <typename T> void RF433Interface<T>::startReceive()
{
#ifdef SLEEP_ONLY
    sleep();
#else
    setStandby();

    // We use a 16 bit preamble so this should save some power by letting radio sit in standby mostly.
    // Furthermore, we need the PREAMBLE_DETECTED and HEADER_VALID IRQ flag to detect whether we are actively receiving
    
    RF433Driver.setModeRx();
    RadioLibInterface::startReceive();

    // Must be done AFTER, starting transmit, because startTransmit clears (possibly stale) interrupt pending register bits
    enableInterrupt(isrRxLevel0);
#endif
}


/** Is the channel currently active? */
template <typename T> bool RF433Interface<T>::isChannelActive()
{
    // check if we can detect a LoRa preamble on the current channel
    bool result;

    setStandby();
    result = RF433Driver.available();
    if (result)
        return true;
    else
        LOG_ERROR("RF433 scanChannel %s%d", radioLibErr, result);
    assert(result != RADIOLIB_ERR_WRONG_MODEM);

    return false;
}

/** Could we send right now (i.e. either not actively receiving or transmitting)? */
template <typename T> bool RF433Interface<T>::isActivelyReceiving()
{
    // The IRQ status will be cleared when we start our read operation. Check if we've started a header, but haven't yet
    // received and handled the interrupt for reading the packet/handling errors.
    //return receiveDetected(lora.getIrqFlags(), RADIOLIB_SX126X_IRQ_HEADER_VALID, RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED);
    return RF433Driver.available();
}

template <typename T> bool RF433Interface<T>::sleep()
{
    // Not keeping config is busted - next time nrf52 board boots lora sending fails  tcxo related? - see datasheet
    // \todo Display actual typename of the adapter, not just `SX126x`
    LOG_DEBUG("RF433 entering sleep mode"); // (FIXME, don't keep config)
    setStandby();                            // Stop any pending operations

    return true;
}

//ACT II. METHODS FROM RADIOLIBINTERFACE

/** Could we send right now (i.e. either not actively receiving or transmitting)? */
template <typename T> bool RF433Interface<T>::canSendImmediately()
{
    // We wait _if_ we are partially though receiving a packet (rather than just merely waiting for one).
    // To do otherwise would be doubly bad because not only would we drop the packet that was on the way in,
    // we almost certainly guarantee no one outside will like the packet we are sending.
    bool busyTx = sendingPacket != NULL;
    bool busyRx = isReceiving && isActivelyReceiving();

    if (busyTx || busyRx) {
        if (busyTx) {
            LOG_WARN("Can not send yet, busyTx");
        }
        // If we've been trying to send the same packet more than one minute and we haven't gotten a
        // TX IRQ from the radio, the radio is probably broken.
        if (busyTx && !Throttle::isWithinTimespanMs(lastTxStart, 60000)) {
            LOG_ERROR("Hardware Failure! busyTx for more than 60s");
            RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_TRANSMIT_FAILED);
            // reboot in 5 seconds when this condition occurs.
            rebootAtMsec = lastTxStart + 65000;
        }
        if (busyRx) {
            LOG_WARN("Can not send yet, busyRx");
        }
        return false;
    } else
        return true;
}

//ACT III. OVERRIDE METHODS FROM RADIOINTERFACE
