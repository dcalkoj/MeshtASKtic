#include "RF433Interface.h"
#include "configuration.h"
#include "error.h"
#include "mesh/NodeDB.h"
#include "PowerMon.h"
#include "main.h"

#include "Throttle.h"

#define RATE 2000
#define WB_IO1 17
#define WB_IO2 34

//ACT I. Transmit  (Reference: RadioInterface)

RF433Interface::RF433Interface() : NotifiedWorkerThread("RF433_ASK_Thread"){
    instance = this;
    RF433Driver = new RH_ASK(RATE,WB_IO1,WB_IO2);
}

RF433Interface *RF433Interface::instance;

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
        RF433Driver->setModeIdle();
    }
    return true;
}

// This actually just queues a send. StartSend actually transmits.
ErrorCode RF433Interface::send(meshtastic_MeshPacket *p){

    size_t numbytes = beginSending(p);

     if (numbytes > RH_ASK_MAX_MESSAGE_LEN){
        LOG_DEBUG("PACKET TOO BIG! Discarding.");
        sendingPacket=NULL;
        packetPool.release(p);
        return ERRNO_UNKNOWN;
    }

    RF433Driver->send((uint8_t *) &radioBuffer, numbytes);
    RF433Driver->waitPacketSent();

    sendingPacket=NULL;
    packetPool.release(p);
    return ERRNO_OK;


    //ErrorCode res = txQueue.enqueue(p) ? ERRNO_OK : ERRNO_UNKNOWN;
    
    
    
    /* Real Code from RadioLib Interface
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
    */
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
    if(!RF433Driver->init()){
        LOG_DEBUG("RH_ASK_INIT FAILED!");
        return false;
    }
pinMode(WB_IO2, OUTPUT);
digitalWrite(WB_IO2, LOW);
return true;
}

float RF433Interface::getFreq(){
    return 433; //duh!
}

void RF433Interface::saveFreq(float savedFreq){
    savedFreq = 433; //duhhh!
}

void RF433Interface::onNotify(uint32_t notification){

    LOG_DEBUG("I got notified :)\n");
}

//Act II. Recieve (Reference: RadioLibInterface)

//  void RF433Interface::disableInterrupt(){
//     //NVIC_DisableIRQ(TIMER2_IRQn);
//  }

// void RF433Interface::enableInterrupt(void (*)()){
//     //NVIC_EnableIRQ(TIMER2_IRQn);
// }

// void RF433Interface::startReceive(){
//     isReceiving = true;
//     powerMon->setState(meshtastic_PowerMon_State_Lora_RXOn);
// }


// bool RF433Interface::isChannelActive(){
//     return RF433Driver.available();
// }

// bool RF433Interface::isActivelyReceiving(){
//     return RF433Driver.available();
// }

// /** if we have something waiting to send, start a short (random) timer so we can come check for collision before actually
//  * doing the transmit */
// void RF433Interface::setTransmitDelay(){
//     meshtastic_MeshPacket *p = txQueue.getFront();
//     if (p->rx_snr == 0 && p->rx_rssi == 0) {
//         startTransmitTimer(true);
//     } else {
//         // If there is a SNR, start a timer scaled based on that SNR.
//         LOG_DEBUG("rx_snr found. hop_limit:%d rx_snr:%f", p->hop_limit, p->rx_snr);
//         startTransmitTimerSNR(p->rx_snr);
//     }

// }

// /** random timer with certain min. and max. settings */
// void RF433Interface::startTransmitTimer(bool withDelay){
//     // If we have work to do and the timer wasn't already scheduled, schedule it now
//     if (!txQueue.empty()) {
//         uint32_t delay = !withDelay ? 1 : getTxDelayMsec();
//         // LOG_DEBUG("xmit timer %d", delay);
//         notifyLater(delay, TRANSMIT_DELAY_COMPLETED, false); // This will implicitly enable
//     }
// }

// /** timer scaled to SNR of to be flooded packet */
// void RF433Interface::startTransmitTimerSNR(float snr){
//      // If we have work to do and the timer wasn't already scheduled, schedule it now
//     if (!txQueue.empty()) {
//         uint32_t delay = getTxDelayMsecWeighted(snr);
//         // LOG_DEBUG("xmit timer %d", delay);
//         notifyLater(delay, TRANSMIT_DELAY_COMPLETED, false); // This will implicitly enable
//     }
// }

// void RF433Interface::handleTransmitInterrupt(){
//     if (sendingPacket)
//         completeSending();
//     powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn); // But our transmitter is deffinitely off now
// }

// //Meaty
// void RF433Interface::handleReceiveInterrupt(){
//     uint32_t xmitMsec;

//     // when this is called, we should be in receive mode - if we are not, just jump out instead of bombing. Possible Race
//     // Condition?
//     if (!isReceiving) {
//         LOG_ERROR("handleReceiveInterrupt called when not in receive mode, which shouldn't happen.");
//         return;
//     }

//     isReceiving = false;

//     uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
//     uint8_t buflen = sizeof(buf);

//     if(RF433Driver.recv(buf, &buflen)){
//         LOG_DEBUG("Captured Raw tx.");
//         RF433Driver.printBuffer("Got: ", buf, buflen);
//     }
//     else{
//         LOG_ERROR("ignoring received packet due to error=");
//         rxBad++;
//     }

// }

// //Meaty
// void RF433Interface::onNotify(uint32_t notification){
//     switch(notification){
//     case ISR_TX:
//         handleTransmitInterrupt();
//         startReceive();
//         // LOG_DEBUG("tx complete - starting timer");
//         startTransmitTimer();
//         break;
//     case ISR_RX:
//         handleReceiveInterrupt();
//         startReceive();
//         // LOG_DEBUG("rx complete - starting timer");
//         startTransmitTimer();
//         break;
//     case TRANSMIT_DELAY_COMPLETED:
//         LOG_DEBUG("delay done");
//         if (!txQueue.empty()) {
//             if (!canSendImmediately()) {
//                 LOG_DEBUG("Currently Rx/Tx-ing: set random delay");
//                 setTransmitDelay(); // currently Rx/Tx-ing: reset random delay
//             } else {
//                 if (isChannelActive()) { // check if there is currently a LoRa packet on the channel
//                     LOG_DEBUG("Channel is active, try receiving first.");
//                     startReceive(); // try receiving this packet, afterwards we'll be trying to transmit again
//                     setTransmitDelay();
//                 } else {
//                     // Send any outgoing packets we have ready
//                     meshtastic_MeshPacket *txp = txQueue.dequeue();
//                     assert(txp);
//                     bool isLoraTx = txp->to != NODENUM_BROADCAST_NO_LORA;
//                     startSend(txp);

//                     if (isLoraTx) {
//                         // Packet has been sent, count it toward our TX airtime utilization.
//                         uint32_t xmitMsec = getPacketTime(txp);
//                         airTime->logAirtime(TX_LOG, xmitMsec);
//                     }
//                 }
//             }
//         } else {
//             // LOG_DEBUG("done with txqueue");
//         }
//         break;
//     default:
//         assert(0); // We expected to receive a valid notification from the ISR
//     }
// }

// /** start an immediate transmit
//  *  This method is  so subclasses can hook as needed, subclasses should not call directly
//  */
// //Meaty
// void RF433Interface::startSend(meshtastic_MeshPacket *txp){
//     printPacket("Starting low level send", txp);
//     if (txp->to == NODENUM_BROADCAST_NO_LORA) {
//         LOG_DEBUG("Drop Tx packet because dest is broadcast no-lora");
//         packetPool.release(txp);
//     } else if (disabled || !config.lora.tx_enabled) {
//         LOG_WARN("Drop Tx packet because LoRa Tx disabled");
//         packetPool.release(txp);
//     } else {
//         powerMon->setState(meshtastic_PowerMon_State_Lora_TXOn);

//         size_t numbytes = beginSending(txp);

//         const char *msg = "Hello World!";

//         auto res = RF433Driver.send((uint8_t *)msg, strlen(msg));
//         RF433Driver.waitPacketSent();

//         //int res = iface->startTransmit((uint8_t *)&radioBuffer, numbytes);
//         if (res != RADIOLIB_ERR_NONE) {
//             LOG_ERROR("startTransmit failed, error=%d", res);
//             RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_RADIO_SPI_BUG);

//             // This send failed, but make sure to 'complete' it properly
//             completeSending();
//             powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn); // Transmitter off now
//             startReceive(); // Restart receive mode (because startTransmit failed to put us in xmit mode)
//         }

//         // Must be done AFTER, starting transmit, because startTransmit clears (possibly stale) interrupt pending register
//         // bits
//         //enableInterrupt(isrTxLevel0);
//     }


// meshtastic_QueueStatus RF433Interface::getQueueStatus(){
//     meshtastic_QueueStatus qs;

//     qs.res = qs.mesh_packet_id = 0;
//     qs.free = txQueue.getFree();
//     qs.maxlen = txQueue.getMaxLen();

//     return qs;
// }

// //Meaty
// bool RF433Interface::receiveDetected(){
//     return isChannelActive();
// }

// //Meaty
// bool RF433Interface::canSendImmediately(){
//     bool busyTx = sendingPacket != NULL;
//     bool busyRx = isReceiving && isActivelyReceiving();
//      if (busyTx || busyRx) {
//         if (busyTx) {
//             LOG_WARN("Can not send yet, busyTx");
//         }
//         // If we've been trying to send the same packet more than one minute and we haven't gotten a
//         // TX IRQ from the radio, the radio is probably broken.
//         if (busyTx && !Throttle::isWithinTimespanMs(lastTxStart, 60000)) {
//             LOG_ERROR("Hardware Failure! busyTx for more than 60s");
//             RECORD_CRITICALERROR(meshtastic_CriticalErrorCode_TRANSMIT_FAILED);
//             // reboot in 5 seconds when this condition occurs.
//             rebootAtMsec = lastTxStart + 65000;
//         }
//         if (busyRx) {
//             LOG_WARN("Can not send yet, busyRx");
//         }
//         return false;
//     } else
//         return true;

// }

// void RF433Interface::completeSending(){
// // We are careful to clear sending packet before calling printPacket because
//     // that can take a long time
//     auto p = sendingPacket;
//     sendingPacket = NULL;

//     if (p) {
//         txGood++;
//         if (!isFromUs(p))
//             txRelay++;
//         printPacket("Completed sending", p);

//         // We are done sending that packet, release it
//         packetPool.release(p);
//         // LOG_DEBUG("Done with send");
//     }
// }