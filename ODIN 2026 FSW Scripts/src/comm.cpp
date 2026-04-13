#include <Arduino.h> // STANDARD ARDUINO LIBRARY
#include <Wire.h> // WIRE LIBRARY
#include <rockblock_9704.h> // IRIDIUM 9704 LIBRARY
#include <comm.h> // COMMS DEFINITIONS

int messagesSent = 0;

// Create callback for 9704 provisioning status updates (i.e., is the device established on the network)
void onMessageProvisioning(const jsprMessageProvisioning_t *messageProvisioning)
{
    if(messageProvisioning->provisioningSet == true)
    {
        Serial.print("Device is provisioned for ");
        Serial.print(messageProvisioning->topicCount);
        Serial.println(" topics");
        Serial.println("Provisioned topics:");
        for(int i = 0; i < messageProvisioning->topicCount; i++)
        {
            Serial.print("Topic name: ");
            Serial.print(messageProvisioning->provisioning[i].topicName);
            Serial.print(", Topic number: ");
            Serial.println(messageProvisioning->provisioning[i].topicId);
        }
    }
}

// Create callback for 9704 MO message completion (i.e., has the message been sent successfully)
void onMoComplete(const uint16_t id, const rbMsgStatus_t status)
{
    Serial.print("MO Complete: ID = ");
    Serial.print(id);
    Serial.print(", Status = ");
    Serial.println(status);
    if(status == RB_MSG_STATUS_OK)
    {
        messagesSent += 1;
        Serial.print("Message Sent: ");
        Serial.println(messagesSent);
    }
}

// Create callback for 9704 constellation state updates (i.e., signal strength)
void onConstellationState(const jsprConstellationState_t *state)
{
        Serial.print("Current Signal: ");
        Serial.println(state->signalBars);
}

// Create callback struct and register callbacks
static rbCallbacks_t myCallbacks =
{
.messageProvisioning = onMessageProvisioning,
.moMessageComplete = onMoComplete,
.constellationState = onConstellationState
};

// 9704 power-up sequence based on vendor specs
void StartupSequence(){
    // Read the state of the boot signal
    Serial.print("I_BTD state at boot: ");
    Serial.println(digitalRead(I_BTD));
    delay(100);
    // Set I_EN high to boot the module
    Serial.println("Setting I_EN HIGH...");
    digitalWrite(I_EN, HIGH);
    Serial.println("Set I_EN HIGH.");
    delay(100);
    // Read the state of the boot signal again
    Serial.print("I_BTD state after set: ");
    Serial.println(digitalRead(I_BTD));
    
    // define a timeout 
    unsigned long startTime = millis();
    const unsigned long timeout = 20000;
    
    // Wait for I_BTD to go high (module boot complete)
    while (digitalRead(I_BTD) == LOW){
        if (millis() - startTime > timeout) {
            Serial.println("ERROR: I_BTD did not go HIGH within timeout");
            break;
        }
        if ((millis() - startTime) % 1000 == 0){
            Serial.print("Waiting for I_BTD to go HIGH... ");
            Serial.println(digitalRead(I_BTD));
        }
        delay(100);
    }
    
    if (digitalRead(I_BTD) == HIGH) {
        Serial.println("RockBLOCK module powered up successfully");
    }
}

// 9704 shut-down sequence based on vendor specs
void ShutdownSequence(){
    // End session with RockBLOCK before powering down
    Serial.println("Ending RockBLOCK session...");
    rbEnd();

    // Pull I_EN Low to power down the module
    Serial.println("Setting I_EN LOW...");
    digitalWrite(I_EN, LOW);
    
    // Wait for I_BTD to go low (module power down)
    unsigned long startTime = millis();
    const unsigned long timeout = 20000; // 20 second timeout
    
    while (digitalRead(I_BTD) == HIGH){
        if (millis() - startTime > timeout) {
            Serial.println("WARNING: RockBLOCK did not power down within timeout");
            break;
        }
        delay(100);
    }
    
    if (digitalRead(I_BTD) == LOW) {
        Serial.println("RockBLOCK module powered down successfully");
    }
}


void setup() {
    // Start serial communication with FSW controller
  Serial.begin(9600);
    // Start serial communication with 9704
  Serial1.begin(230400);
  delay(1000);
  Serial.println("Starting RockBLOCK Async XMIT\r\n");
  // Start library
  if(rbBegin(Serial1))
  {
      delay(1000);
      //Register Callbacks
      rbRegisterCallbacks(&myCallbacks);
      Serial.println("Successfully started serial session with RB9704\r\n");
      //Queue and send Mobile-Originated (MO) message
      const char *message = "Hello World!";
      // Run async send to avoid blocking the main loop while sending
      if(rbSendMessageAsync(244, message, strlen(message)))
      {
          Serial.print("Sent MO: ");
          Serial.println(message);
      }
      else
      {
          Serial.println("Failed to queue message\r\n");
      }
  }
  else
  {
      Serial.println("Failed to begin the serial connection\r\n");
  }
}

void loop() {
    // Poll 9704 for state
    // Must poll at least once every 50ms
    rbPoll();
    delay(10);
    // If the message has been sent, end the session and power down the 9704 to end the test, otherwise end the connection
    if(messagesSent > 0)
    {
        //End serial connection
        if(rbEnd())
        {
            Serial.println("Ended connection successfully\r\n");
            Serial1.end();
            messagesSent = 0;
        }
        else
        {
            Serial.println("Failed to end connection\r\n");
        }
    }
}