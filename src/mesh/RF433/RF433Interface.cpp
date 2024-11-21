#include "RF433Interface.h"
#include "configuration.h"
#include "error.h"
#include "mesh/NodeDB.h"
#include "PowerMon.h"

#include "Throttle.h"

//ACT I. Transmit  (Reference: RadioInterface)
RF433Interface::RF433Interface() : NotifiedWorkerThread("RF433"){

}

bool RF433Interface::canSleep(){
    bool res = txQueue.empty();
    if (!res) { // only print debug messages if we are vetoing sleep
        LOG_DEBUG("radio wait to sleep, txEmpty=%d", res);
    }
    return res;
}

/// Prepare hardware for sleep.  Call this _only_ for deep sleep, not needed for light sleep.
bool RF433Interface::sleep(){
    LOG_DEBUG("RF433 entering sleep");
    if(canSleep()){
        RF433Driver.setModeIdle();
    }
    return true;
}

// This actually just queues a send. StartSend actually transmits.
ErrorCode RF433Interface::send(meshtastic_MeshPacket *p){
    printPacket("enqueuing for send", p);

    LOG_DEBUG("txGood=%d,txRelay=%d,rxGood=%d,rxBad=%d", txGood, txRelay, rxGood, rxBad);
    ErrorCode res = txQueue.enqueue(p) ? ERRNO_OK : ERRNO_UNKNOWN;

    if (res != ERRNO_OK) { // we weren't able to queue it, so we must drop it to prevent leaks
        packetPool.release(p);
        return res;
    }

    // set (random) transmit delay to let others reconfigure their radio,
    // to avoid collisions and implement timing-based flooding
    // LOG_DEBUG("Set random delay before transmitting.");
    setTransmitDelay();

    return res;
}


bool RF433Interface::cancelSending(NodeNum from, PacketId id){
    auto p = txQueue.remove(from, id);
    if (p)
        packetPool.release(p); // free the packet we just removed

    bool result = (p != NULL);
    LOG_DEBUG("cancelSending id=0x%x, removed=%d", id, result);
    return result;
}

bool RF433Interface::init(){
    if(!RF433Driver.init()){
        LOG_DEBUG("RH_ASK_INIT FAILED!");
        return false;
    }
return true;
}

float RF433Interface::getFreq(){
    return 433; //duh!
}

void RF433Interface::saveFreq(float savedFreq){
    savedFreq = 433; //duhhh!
}

//Act II. Recieve (Reference: RadioLibInterface)
 void RF433Interface::disableInterrupt(){
    NVIC_DisableIRQ(TIMER2_IRQn);
 }

void RF433Interface::enableInterrupt(void (*)()){
    NVIC_EnableIRQ(TIMER2_IRQn);
}

void RF433Interface::startReceive(){
    isReceiving = true;
    powerMon->setState(meshtastic_PowerMon_State_Lora_RXOn);
}


bool RF433Interface::isChannelActive(){
    return RF433Driver.available();
}

bool RF433Interface::isActivelyReceiving(){
    return RF433Driver.available();
}

/** if we have something waiting to send, start a short (random) timer so we can come check for collision before actually
 * doing the transmit */
void RF433Interface::setTransmitDelay(){
    meshtastic_MeshPacket *p = txQueue.getFront();
    if (p->rx_snr == 0 && p->rx_rssi == 0) {
        startTransmitTimer(true);
    } else {
        // If there is a SNR, start a timer scaled based on that SNR.
        LOG_DEBUG("rx_snr found. hop_limit:%d rx_snr:%f", p->hop_limit, p->rx_snr);
        startTransmitTimerSNR(p->rx_snr);
    }

}

/** random timer with certain min. and max. settings */
void RF433Interface::startTransmitTimer(bool withDelay){
    // If we have work to do and the timer wasn't already scheduled, schedule it now
    if (!txQueue.empty()) {
        uint32_t delay = !withDelay ? 1 : getTxDelayMsec();
        // LOG_DEBUG("xmit timer %d", delay);
        notifyLater(delay, TRANSMIT_DELAY_COMPLETED, false); // This will implicitly enable
    }
}

/** timer scaled to SNR of to be flooded packet */
void RF433Interface::startTransmitTimerSNR(float snr){
     // If we have work to do and the timer wasn't already scheduled, schedule it now
    if (!txQueue.empty()) {
        uint32_t delay = getTxDelayMsecWeighted(snr);
        // LOG_DEBUG("xmit timer %d", delay);
        notifyLater(delay, TRANSMIT_DELAY_COMPLETED, false); // This will implicitly enable
    }
}

void RF433Interface::handleTransmitInterrupt(){
    if (sendingPacket)
        completeSending();
    powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn); // But our transmitter is deffinitely off now
}

//Meaty
void RF433Interface::handleReceiveInterrupt(){

}

//Meaty
void RF433Interface::onNotify(uint32_t notification){

}

/** start an immediate transmit
 *  This method is  so subclasses can hook as needed, subclasses should not call directly
 */
//Meaty
void RF433Interface::startSend(meshtastic_MeshPacket *txp){

}

meshtastic_QueueStatus RF433Interface::getQueueStatus(){
    meshtastic_QueueStatus qs;

    qs.res = qs.mesh_packet_id = 0;
    qs.free = txQueue.getFree();
    qs.maxlen = txQueue.getMaxLen();

    return qs;
}

//Meaty
bool RF433Interface::receiveDetected(){

}

//Meaty
bool RF433Interface::canSendImmediately(){

}

void RF433Interface::completeSending(){
// We are careful to clear sending packet before calling printPacket because
    // that can take a long time
    auto p = sendingPacket;
    sendingPacket = NULL;

    if (p) {
        txGood++;
        if (!isFromUs(p))
            txRelay++;
        printPacket("Completed sending", p);

        // We are done sending that packet, release it
        packetPool.release(p);
        // LOG_DEBUG("Done with send");
    }
}