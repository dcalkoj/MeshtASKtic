#include "DemoMod.h"
#include "MeshService.h"
#include "configuration.h"
#include "main.h"

#include <assert.h>

meshtastic_MeshPacket *DemoModule::allocReply()
{
    assert(currentRequest); // should always be !NULL
//#ifdef DEBUG_PORT
    auto req = *currentRequest;
    auto &p = req.decoded;
    // The incoming message is in p.payload
    LOG_INFO("Received message from=0x%0x, id=%d, msg=%.*s", req.from, req.id, p.payload.size, p.payload.bytes);
//#endif

    screen->print("Sending Demo\n");

    const char *DemoStr = "Message Received!!";
    auto Demo = allocDataPacket();                 // Allocate a packet for sending
    Demo->decoded.payload.size = strlen(DemoStr); // You must specify how many bytes are in the Reply
    memcpy(Demo->decoded.payload.bytes, DemoStr, Demo->decoded.payload.size);
    //service->sendToMesh(Demo);

    return Demo;
}
