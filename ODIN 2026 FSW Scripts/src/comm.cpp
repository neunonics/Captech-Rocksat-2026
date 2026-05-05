#include <Arduino.h> // STANDARD ARDUINO LIBRARY
#include <Wire.h> // WIRE LIBRARY
#include <rockblock_9704.h> // IRIDIUM 9704 LIBRARY
#include <imt_queue.h> // IRIDIUM 9704 LIBRARY
#include <comm.h> // COMMS DEFINITIONS
#include <fsw.h> // FSW DEFINITIONS
#include <pins.h> // PINS DEFINITIONS

// Diagnostic state populated by RockBLOCK callbacks. Used to tell
// "send failed because topic not provisioned" apart from "send failed at radio".
static bool g_topic244Provisioned = false;
static bool g_provisioningSeen = false;
static int8_t g_lastSignalBars = -1;

static void onMessageProvisioning(const jsprMessageProvisioning_t *p) {
    g_provisioningSeen = true;
    g_topic244Provisioned = false;
    DEBUG_SERIAL.print("[COMM] Provisioning callback: provisioningSet=");
    DEBUG_SERIAL.print(p->provisioningSet ? "true" : "false");
    DEBUG_SERIAL.print(", topicCount=");
    DEBUG_SERIAL.println(p->topicCount);
    for (uint8_t i = 0; i < p->topicCount && i < JSPR_MAX_TOPICS; i++) {
        DEBUG_SERIAL.print("[COMM]   topic[");
        DEBUG_SERIAL.print(i);
        DEBUG_SERIAL.print("]: id=");
        DEBUG_SERIAL.print(p->provisioning[i].topicId);
        DEBUG_SERIAL.print(" name=");
        DEBUG_SERIAL.println(p->provisioning[i].topicName);
        if (p->provisioning[i].topicId == COMM_TOPIC) {
            g_topic244Provisioned = true;
        }
    }
    DEBUG_SERIAL.print("[COMM] Topic ");
    DEBUG_SERIAL.print(COMM_TOPIC);
    DEBUG_SERIAL.println(g_topic244Provisioned ? " IS provisioned." : " NOT provisioned!");
}

static void onConstellationState(const jsprConstellationState_t *s) {
    if (s->signalBars != g_lastSignalBars) {
        g_lastSignalBars = s->signalBars;
        DEBUG_SERIAL.print("[COMM] Signal bars=");
        DEBUG_SERIAL.print(s->signalBars);
        DEBUG_SERIAL.print(" level=");
        DEBUG_SERIAL.println(s->signalLevel);
    }
}

static void onMoMessageComplete(const uint16_t id, const rbMsgStatus_t status) {
    DEBUG_SERIAL.print("[COMM] MO complete id=");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" status=");
    DEBUG_SERIAL.println(status == RB_MSG_STATUS_OK ? "OK" : "FAIL");
}

static void registerCommCallbacks() {
    static const rbCallbacks_t cbs = {
        .messageProvisioning = onMessageProvisioning,
        .moMessageComplete = onMoMessageComplete,
        .mtMessageComplete = NULL,
        .constellationState = onConstellationState,
    };
    rbRegisterCallbacks(&cbs);
}

// -- INIT FUNCTIONS -- //
void clearQueue(){
    while(imtQueueMtGetFirst()){
        imtQueueMoRemove(); // Removes message in queue if one is found
    }
}

void initCOMMStatus(COMM &comm){
    comm.ENBL_STATUS = false; // COMMS are initially disabled
    comm.TX_ACTIVE = false; // No transmission is active at startup
    comm.messagesSent = 0; // Initialize messages sent counter to 0
}

bool initCOMM(COMM &comm){
    // Initialize COMMS pins
    digitalWrite(COMM_EN, HIGH); // Bring Iridium Enable Pin HIGH to enable COMMS XMIT

    unsigned long startTime = millis(); // Setup a timeout to prevent getting stuck if COMMS fails to enable
    const unsigned long timeout = 15000; // 15 second timeout for COMMS to enable

    while (digitalRead(COMM_BTD) == LOW) { // Wait for COMMS to enable (read 9704 Boot State)
        if (millis() - startTime > timeout) {
            DEBUG_SERIAL.println("[COMM] ERROR: COMMS failed to enable within timeout period.");
            return false; // Return false if COMMS fail to enable within the timeout period
        }
        delay(100); // Short delay to avoid sampling too quickly
    }
    DEBUG_SERIAL.println("[COMM] BTD asserted, modem booted.");

    // Register diagnostic callbacks BEFORE rbBegin so we capture provisioning
    // and constellation state events from the very first send.
    registerCommCallbacks();

    if(!rbBegin(COMM_SERIAL)) { // Initialize RockBLOCK communication
        DEBUG_SERIAL.println("[COMM] ERROR: rbBegin() failed (serial init / setApi / setSim / setState).");
        return false; // Return false if COMMS initialization fails
    }
    DEBUG_SERIAL.println("[COMM] rbBegin OK.");

    // README: wait at least 100ms after rbBegin before doing anything else.
    delay(500);

    clearQueue();

    comm.ENBL_STATUS = true; // Update COMMS status to enabled
    return true; // Enable pin pulled high, COMMS should be enabled
}

bool commShutDown(COMM &comm){
    // Disable COMMS
    rbEnd(); // End RockBLOCK communication
    digitalWrite(COMM_EN, LOW); // Disable COMMS

    unsigned long startTime = millis(); // Setup a timeout to prevent getting stuck if COMMS fails to disable
    const unsigned long timeout = 15000; // 15 second timeout for COMMS to enable

    while (digitalRead(COMM_BTD) == HIGH) { // Wait for COMMS to disable (read 9704 Boot State)
        if (millis() - startTime > timeout) {
            DEBUG_SERIAL.println("[COMM] ERROR: COMMS failed to disable within timeout period.");
            return false; // Return false if COMMS fail to disable within the timeout period
        }
        delay(100); // Short delay to avoid sampling too quickly
    }

    comm.ENBL_STATUS = false; // Update COMMS status to disabled
    return true; // Return true to indicate successful shutdown
}

bool sendMessage(FSW &fsw, COMM &comm, const String &orinLine, const String &hist){
    if ((fsw.currentMissionTime - fsw.lastTransmit) >= COMM_SEND_INTERVAL){
        fsw.lastTransmit = now();
        DEBUG_SERIAL.println("[COMM] Attempting to send message at: " + String(fsw.lastTransmit));

        if (!comm.ENBL_STATUS) {
            DEBUG_SERIAL.println("[COMM] ERROR: Cannot send message, COMMS not enabled.");
            return false; // Cannot send message if COMMS are not enabled
        }

        // clearQueue();

        String messageTemp;
        messageTemp = "a" + String(fsw.currentMissionTime - fsw.missionStartTime) + "|b" + fsw.epdsToSave + "|c" + fsw.fswToSave + "|d" + orinLine + "|e" + hist;
        int messageLength = messageTemp.length();
        char message[messageLength + 1]; // +1 for the null terminator that toCharArray writes
        messageTemp.toCharArray(message, messageLength + 1);

        DEBUG_SERIAL.print("[COMM] Message length: ");
        DEBUG_SERIAL.print(messageLength);
        DEBUG_SERIAL.print(" bytes (limit ");
        DEBUG_SERIAL.print((unsigned)(IMT_PAYLOAD_SIZE - IMT_CRC_SIZE));
        DEBUG_SERIAL.println(")");
        DEBUG_SERIAL.println("[COMM] Message content: " + messageTemp);

        int8_t signalStrength;
        signalStrength = rbGetSignal();
        DEBUG_SERIAL.println("[COMM] Attempting to send message with " + String(signalStrength) + " signal!");

        unsigned long sendStart = millis();
        bool sent = rbSendMessageAny(COMM_TOPIC, message, messageLength, COMM_TIMEOUT);
        unsigned long sendElapsed = millis() - sendStart;

        if(!sent) {
            DEBUG_SERIAL.print("[COMM] ERROR: rbSendMessageAny returned false after ");
            DEBUG_SERIAL.print(sendElapsed);
            DEBUG_SERIAL.println(" ms.");
            DEBUG_SERIAL.print("[COMM]   provisioning callback fired: ");
            DEBUG_SERIAL.println(g_provisioningSeen ? "yes" : "no");
            DEBUG_SERIAL.print("[COMM]   topic ");
            DEBUG_SERIAL.print(COMM_TOPIC);
            DEBUG_SERIAL.print(" provisioned: ");
            DEBUG_SERIAL.println(g_topic244Provisioned ? "yes" : "no");
            DEBUG_SERIAL.print("[COMM]   last signal bars: ");
            DEBUG_SERIAL.println(g_lastSignalBars);
            // Early-fail hint: if the call returned in well under the timeout,
            // the failure is at queue-add or jspr-put-originate, not radio.
            if (sendElapsed < (unsigned long)COMM_TIMEOUT * 900UL) {
                DEBUG_SERIAL.println("[COMM]   -> returned early; likely length/queue/JSPR rejection, NOT a radio timeout.");
            } else {
                DEBUG_SERIAL.println("[COMM]   -> ran to timeout; radio could not deliver in time (signal/antenna/power).");
            }
            return false; // Return false if message transmission fails
        }
        DEBUG_SERIAL.print("[COMM] Send took ");
        DEBUG_SERIAL.print(sendElapsed);
        DEBUG_SERIAL.println(" ms.");

        comm.messagesSent++; // Increment messages sent counter
        DEBUG_SERIAL.println("[COMM] Message sent: " + messageTemp);
        return true; // Return true to indicate successful transmission
    }
    return false; // Interval not elapsed; nothing sent this call.
}
