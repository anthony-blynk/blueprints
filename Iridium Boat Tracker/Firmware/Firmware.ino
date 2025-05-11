/*******************************************************************************
  The MIT License (MIT)

  Copyright (C) 2024 Iridium Satellite LLC.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  IN THE SOFTWARE.
*******************************************************************************/
/*
  Dashboard
    Transfer data to and from a dashboard in the cloud that is accessible using
    a web browser or mobile phone.

    USER LED
      Not Connected - Blinking slowly (1 second on, 1 second off).
      Connected     - Fast blinks (1 to 5) indicating the strength of the connection.

    TX LED
      Flashes when Iridium 9704 Transceiver is transmitting to the Iridium Network.

    USER Button
      Push the USER button to immediately queue a message to the dashboard. As soon
      as the Iridium 9704 Launch Pad connects to a satellite it will send the data
      to the dashboard, check to see if an alert has been requested, and report the
      number of remaining free messages on the serial output.

    Serial Output
      >   sketch    >  Description of events and data sent to the Iridium 9704 Transceiver
                       from this sketch.

      < transceiver <  Description of events and data received from the Iridium 9704 Transceiver.

                    ^  Periodic update of signal strength, battery voltage, battery
                       charger state, and time until next update.

    Shutdown
      Press and hold the USER button to shut down the transceiver.
      The buzzer plays a shutdown melody and the USER LED displays a sleep pattern.
      It's now safe to disconnect the battery.

    Startup
      While the transceiver is shut down, press and hold the USER button to restart
      the transceiver. The buzzer plays a startup melody, the USER LED displays
      the signal strength, and updates resume on serial output.

    Transmitting Chirp
      To disable buzzer chirp when transmitting change
        buzzer.playToneWhenTransmitting();
      to
        buzzer.off();
 */

 #include "ArduinoJson.h"  // Arduino JSON library
 #include "LaunchPad.h"
 
 #define BLYNK_TEMPLATE_ID    "TMPL"
 #define BLYNK_TEMPLATE_NAME "Satellite Boat Tracker"
 #define BLYNK_AUTH_TOKEN "C8L2wOG5EEy3nrxXIgNoRUctetdW08a0"

 const int AUX_OUT1 = 5;
 const int AUX_OUT2 = 4;
 const int AUX_OUT3 = 7;

// #define BLYNK_TEMPLATE_NAME "Iridium 9704 Launch Pad Free"
// #define BLYNK_AUTH_TOKEN "StFfxoeeFLjFTk5T4Nzwg1Cd9C0irrgq"
 /**
  * Dashboard update type
  */
 enum UpdateType
 {
   UPDATE_TYPE_PERIODIC,     /**< Dashboard update initiated by dashboard update timer expiring. */
   UPDATE_TYPE_BUTTON_PRESS  /**< Dashboard update initiated by user button press. */
 };
 
 /**
  * Dashboard data
  */
 struct DashboardData
 {
   int        signalBars;         /**< Transceiver signal strength in bars. */
   int        signalStrengthdBm;  /**< Transceiver signal strength in dBm. */
   int        temperatureDegC;    /**< Transceiver temperature in degrees C. */
   float      batteryVoltage;     /**< Iridium 9704 Launch Pad battery voltage in volts. */
   UpdateType updateType;         /**< Update type (periodic timer or user button). */
   GnssData   gnss;               /**< GNSS data. */
 };
 
 /*
   Local function declarations:
  */
 static void loopStatusPrint(void);
 static void loopRequestUpdates(void);
 static void loopCheckUserButton(void);
 static void transceiverStartup(void);
 static void transceiverShutdown(void);
 static void dashboardUpdate(const UpdateType updateType);
 static void requestFreeMessagesRemaining(void);
 static bool sendMessageDataCallback(int topicId, int messageId, int segmentLength, int segmentStart, unsigned char buffer[], int bufferSize);
 static void sendMessageStatusCallback(int topicId, int requestReference, int messageId, SendMessageStatus status);
 static void receiveMessageDataCallback(int topicId, int messageId, int segmentLength, int segmentStart, const unsigned char buffer[]);
 static void receiveMessageStatusCallback(int topicId, int messageId, int messageLength, ReceiveMessageStatus status);
 static int  createDashboardMessage(const DashboardData & dashboardData, char buffer[], int bufferSize);
 static void processReceivedMessage(int topicId, int messageId);
 static void processHttpResponse(const char jsonString[]);
 static void processFreeMessagesResponse(const char jsonString[]);
 static int  getMessageRequestReference(void);
 static char getLatitudeDirection(const float latitude);
 static char getLongitudeDirection(const float longitude);
 static const char * convertTopicIdToString(const int topicId);
 static const char * convertUpdateTypeToString(const UpdateType updateType);
 static const char * convertMonthToString(int month);
 static void padString(int length, char string[], int stringBufferSize);
 
 
 const int    MAX_UPDATE_MESSAGE_LENGTH  = 500;                      /**< Maximum length of a dashboard update message. */
 const int    MAX_RECEIVE_MESSAGE_LENGTH = 200;                      /**< Maximum length of a response message (either to a dashboard update message or a free messages remaining message). */
 const int    INVALID_MESSAGE_ID = -1;                               /**< Invalid message id. */
 static char  updateMessageBuffer[MAX_UPDATE_MESSAGE_LENGTH];        /**< Buffer to store dashboard update message. */
 static char  receiveMessageBuffer[MAX_RECEIVE_MESSAGE_LENGTH];      /**< Buffer to store response message (either to a dashboard update message or a free messages remaining message). */
 static int   nextRequestReference = MIN_MESSAGE_REQUEST_REFERENCE;  /**< Next message to be sent's request reference (sketch's unique identifier for a transceiver message). */
 static int   updateMessageRequestReference;                         /**< Dashboard update message's application request reference (initialized to invalid value). */
 static int   updateMessageId = INVALID_MESSAGE_ID;                  /**< Dashboard update message's transceiver message id (initialized to invalid value). */
 static int   receivingMessageId = INVALID_MESSAGE_ID;               /**< Receiving message's transceiver message Id (initialized to invalid value). */
 static int   receivedMessageLength = 0;                             /**< Received message length (initialized to invalid value). */
 static bool  printedErrorNoAuthToken = false;                       /**< Flag to indicate if informed user that BLYNK_AUTH_TOKEN is not defined. */
 static Timer dashboardUpdateTimer;                                  /**< Dashboard periodic update timer. */
 static Timer gnssAndTemperatureUpdateTimer;                         /**< GNSS and temperature update timer. */
 static Timer statusUpdateTimer;                                     /**< Status update timer (used to reduce frequency of status messages). */
 
 const char   MESSAGE_EVENT_PROMPT[]            = "                 ";    /**< Prompt used to report message events. */
 const char   SEND_TO_TRANSCEIVER_PROMPT[]      = ">   sketch    >  ";    /**< Prompt used to report sending message related data to transceiver. */
 const char   RECEIVE_FROM_TRANSCEIVER_PROMPT[] = "< transceiver <  ";    /**< Prompt used to report receiving message related data from transceiver. */
 const char   STATUS_PROMPT[]                   = "                 ^ ";  /**< Prompt used to report system status. */
 
 static JsonDocument jsonDoc;     /**< Arduino JSON library document. */
 static JsonObject   jsonObject;  /**< Arduino JSON library object. */

 static int geoFenceAlarm;
 static int previousGeoFenceAlarm;
 static int geoFenceRadius;
 static float geoFenceLat;
 static float geoFenceLon;
 
 /**
  * Pre-built message used for requesting the number of free messages remaining in the free trial.
  */
 static const char REQUEST_FREE_MESSAGES_REMAINING_MESSAGE[] =
 "{"                                       // JSON data begin
   "\"cmd\":\"GET\","                      // JSON key:value pair
   "\"data\":\"free_messages_remaining\""  // JSON key:value pair
 "}"                                       // JSON data end
 "\x2f"                                    // Message CRC (MSB)
 "\xa6";                                   // Message CRC (LSB)
 
 /**
  * Called once by the Arduino Board Core package at startup.
  */
 void setup()
 {
   pinSetup();
 
   /*
     For USB CDC serial ports like the Iridium 9704 Launch Pad, Serial.begin() is irrelevant.
     You can use any baud rate and configuration for serial communication.
 
     NOTE: Baud rate 1200 is reserved to be used as a signal by the Arduino IDE
           to jump to the bootloader. It is not recommended to use 1200 baud.
   */
   Serial.begin(9600);
 
   // Wait with timeout for host USB-C cable to be connected to avoid missing log messages.
   const int SERIAL_PORT_READY_TIMEOUT_MS = 3000;
   waitForSerialPortReady(SERIAL_PORT_READY_TIMEOUT_MS);
 
   logSetup();
   batteryCharger.logDisable();
   buzzer.logDisable();
   gnss.logDisable();
   sdCard.logDisable();
   userButton.logDisable();
   userLed.logDisable();
   transceiver.logDisable();
 
   printVersion("Dashboard");
 
   sdCard.begin();
   batteryCharger.begin();
   userLed.showTransceiverSignalStrength();
   buzzer.playToneWhenTransmitting();
   gnss.begin();
 
   transceiver.setMessageCallbacks(sendMessageDataCallback, sendMessageStatusCallback, receiveMessageDataCallback, receiveMessageStatusCallback);
   transceiver.begin();
 
   const Ticks PERIODIC_UPDATE_TIME_MS = (5 * 60 * 1000);                      /**< Dashboard periodic update time in milliseconds. */
   dashboardUpdateTimer.setDurationMs(PERIODIC_UPDATE_TIME_MS);
 
   const Ticks GNSS_AND_TEMP_UPDATE_TIME_MS = (10 * 1000);                     /**< GNSS and temperature request update interval in milliseconds. */
   gnssAndTemperatureUpdateTimer.setDurationMs(GNSS_AND_TEMP_UPDATE_TIME_MS);
   gnssAndTemperatureUpdateTimer.start();
 
   const Ticks STATUS_UPDATE_PERIOD_MS = (5 * 1000);                           /**< Minimum time between status updates in milliseconds (to reduce frequency of updates). */
   statusUpdateTimer.setDurationMs(STATUS_UPDATE_PERIOD_MS);
   statusUpdateTimer.start();

   // Auxilary output pins
   pinMode(AUX_OUT1, OUTPUT);
   pinMode(AUX_OUT2, OUTPUT);
   pinMode(AUX_OUT3, OUTPUT);

  //  testCalculateDistance();
 }
 
 /**
  * Called repeatedly by the Arduino Board Core package.
  */
 void loop()
 {
   batteryCharger.loop();
   userLed.loop();
   userButton.loop();
   buzzer.loop();
   gnss.loop();
   sdCard.loop();
   transceiver.loop();
 
   loopCheckUserButton();

   if (transceiver.isReadyToSendMessage() == true)
   {
     if (strlen(BLYNK_AUTH_TOKEN) == 0)
     {
       if (printedErrorNoAuthToken == false)
       {
         /* Unable to send messages without a valid token. */
         buzzer.playToneSequence(FAILURE_TONE_SEQUENCE);
         Serial.println("\nERROR: Update the '#define BLYNK_AUTH_TOKEN' with your Blynk Auth Token.\n");
         printedErrorNoAuthToken = true;
       }
       return;
     }
 
     loopStatusPrint();
     loopRequestUpdates();
 
     if ((dashboardUpdateTimer.isExpired() == true) || (dashboardUpdateTimer.isRunning() == false))
     {
       dashboardUpdate(UPDATE_TYPE_PERIODIC);
     }
     else
     {
       int userButtonPressedTimeMs;
       if (userButton.pushed(userButtonPressedTimeMs) == true)
       {
         requestFreeMessagesRemaining();
         dashboardUpdate(UPDATE_TYPE_BUTTON_PRESS);
       }
     }
   }
 }
 
 /**
  * Print changing constellation state and battery status to the serial monitor.
  */
 static void loopStatusPrint(void)
 {
   static bool previousUsbSerialConnected   = false;  /**< Previous USB Serial Port connected state. */
   static bool previousConstellationVisible = false;  /**< Previous constellation visible. */
   static int  previousSignalStrengthBars   = -1;     /**< Previous constellation signal strength in bars. */
   static int  previousBatteryVoltage10x    = 0;      /**< Previous battery voltage multiplied by 10. */
 
   char  buffer[48];                                  /**< Buffer for creating a formatted output string. */
   bool  usbSerialConnected = (bool)Serial;           /**< USB serial port connected state (true indicates connected). */
   bool  constellationVisible;                        /**< Constellation visible. */
   int   signalStrengthBars;                          /**< Constellation signal strength in bars. */
   int   signalStrengthdBm;                           /**< Constellation signal strength in dBm. */
   float batteryVoltage;                              /**< Battery voltage. */
   int   batteryVoltage10x;                           /**< Battery voltage multiplied by 10. */
 
   if (transceiver.isReady() == false)
   {
     return;
   }
 
   bool usbSerialReconnected = ((previousUsbSerialConnected == false) && (usbSerialConnected == true));
   previousUsbSerialConnected = usbSerialConnected;
 
   /*
     Reduce the frequency of status messages by waiting for status update timer to expire.
     If the USB serial port was re-connected then print status immediately, regardless of status update timer.
   */
   if ((statusUpdateTimer.isExpired() == false) && (usbSerialReconnected == false))
   {
     return;
   }
   statusUpdateTimer.start();
 
   transceiver.getLastConstellationState(constellationVisible, signalStrengthBars, signalStrengthdBm);
   batteryVoltage = batteryCharger.batteryVoltageRead();
   batteryVoltage10x = (int)(batteryVoltage * 10);
 
   if ((usbSerialReconnected == true)
       || (previousConstellationVisible != constellationVisible)
       || (previousSignalStrengthBars   != signalStrengthBars)
       || (previousBatteryVoltage10x    != batteryVoltage10x))
   {
     Serial.print(STATUS_PROMPT);
     if (constellationVisible == false)
     {
       snprintf(buffer, sizeof(buffer), "Constellation not visible,");
     }
     else
     {
       snprintf(buffer, sizeof(buffer), "Signal Strength: %d bar%s,",
                signalStrengthBars, ((signalStrengthBars != 1) ? "s" : EMPTY_STRING));
     }
     padString(28, buffer, sizeof(buffer));
     Serial.print(buffer);
 
     previousConstellationVisible = constellationVisible;
     previousSignalStrengthBars   = signalStrengthBars;
     previousBatteryVoltage10x    = batteryVoltage10x;
 
     snprintf(buffer, sizeof(buffer), "Battery: %3.2fV (%s)",
              batteryVoltage, batteryCharger.chargerStatusString());
 
     Serial.print(buffer);
 
     Ticks timeRemainingMs = (dashboardUpdateTimer.getDurationMs() - dashboardUpdateTimer.elapsedTimeMs());
 
     int days;
     int hours;
     int minutes;
     int seconds;
     int milliseconds;
     timeConvert(timeRemainingMs, days, hours, minutes, seconds, milliseconds);
 
     snprintf(buffer, sizeof(buffer), ",  Next update in: %d:%02d", minutes, seconds);
     Serial.print(buffer);
 
     Serial.println();
   }
 }
 
 /**
  * Request GNSS receiver to update position, velocity, and time information.
  * Request transceiver to update temperature (if transceiver is ready).
  */
 static void loopRequestUpdates(void)
 {
   if (gnssAndTemperatureUpdateTimer.isExpired() == true)
   {
     gnss.requestPositionUpdate();
 
     if (transceiver.isReady() == true)
     {
       transceiver.temperatureUpdate();
     }
 
     gnssAndTemperatureUpdateTimer.start();
   }
 }
 
 /**
  * Check if the user button was pressed to either request the transceiver
  * to startup or shutdown depending on the current transceiver state.
  */
 static void loopCheckUserButton(void)
 {
   const int USER_BUTTON_SOS_DURATION_MS  = 5000;  /**< How long to hold down the user button to signal an SOS event. */

   if (userButton.pressedFor(USER_BUTTON_SOS_DURATION_MS))
   {
     sendSOSEvent();
   }
 }
 
 /**
  * Startup transceiver.
  */
 static void transceiverStartup(void)
 {
   Serial.println("Startup the transceiver.\n");
   buzzer.playToneSequence(STARTUP_TONE_SEQUENCE);
   userLed.showTransceiverSignalStrength();
   transceiver.begin();
   printedErrorNoAuthToken = false;
 }
 
 /**
  * Shutdown transceiver.
  */
 static void transceiverShutdown(void)
 {
   Serial.println("Shutdown the transceiver.\n");
   dashboardUpdateTimer.stop();
   transceiver.end();
   userLed.showPattern(SLEEP_LED_PATTERN);
   buzzer.playToneSequence(SHUTDOWN_TONE_SEQUENCE);
 }
 
 /**
  * Update the dashboard by sending a message to the transceiver containing
  * the the latest sensor data.
  *
  * @param[in] updateType - Update type (user button press or periodic).
  */
 static void dashboardUpdate(const UpdateType updateType)
 {
   DashboardData dashboardData;
 
   bool constellationVisible;
   transceiver.getLastConstellationState(constellationVisible, dashboardData.signalBars, dashboardData.signalStrengthdBm);
 
   dashboardData.temperatureDegC = transceiver.getLastTemperatureDegC();
   dashboardData.batteryVoltage  = batteryCharger.batteryVoltageRead();
   dashboardData.updateType      = updateType;
   gnss.getLastPositionVelocityTime(dashboardData.gnss);
 
   int messageLength = createDashboardMessage(dashboardData, updateMessageBuffer, sizeof(updateMessageBuffer));
   if (messageLength > 0)
   {

    Serial.println("**** debug1");
    Serial.println(updateMessageBuffer);
    Serial.println("**** debug1 end");

     updateMessageRequestReference = getMessageRequestReference();
 
     Serial.print(SEND_TO_TRANSCEIVER_PROMPT);
     Serial.print("Requesting transceiver to send update message to ");
     Serial.print(convertTopicIdToString(HTTP_TOPIC_ID));
     Serial.print(" topic (request reference ");
     Serial.print(updateMessageRequestReference);
     Serial.print(").");
     Serial.println();
     transceiver.messageSend(HTTP_TOPIC_ID, messageLength, updateMessageRequestReference);
   }
 
   // if the geo fence alarm was switched on then save the current position
   if (geoFenceAlarm && !previousGeoFenceAlarm)
   {
     geoFenceLat = dashboardData.gnss.latitudeDegrees;
     geoFenceLon = dashboardData.gnss.longitudeDegrees;
     Serial.printf("GEO Fence set to %.5f,%.5f\n", geoFenceLat, geoFenceLon);
   }

   checkGeoFenceBreach(dashboardData.gnss);

   dashboardUpdateTimer.start();
 }
 
 /**
  * Send a message to request the number of free messages remaining in the free trial.
  */
 static void requestFreeMessagesRemaining(void)
 {
   if (transceiver.isReadyToSendMessage() == true)
   {
     int requestReference = getMessageRequestReference();
     Serial.print(SEND_TO_TRANSCEIVER_PROMPT);
     Serial.print("Requesting transceiver to send message 'REQUEST_FREE_MESSAGES_REMAINING_MESSAGE' to ");
     Serial.print(convertTopicIdToString(FREE_MESSAGES_TOPIC_ID));
     Serial.print(" topic (request reference ");
     Serial.print(requestReference);
     Serial.print(").");
     Serial.println();
     transceiver.messageSend(FREE_MESSAGES_TOPIC_ID, strlen(REQUEST_FREE_MESSAGES_REMAINING_MESSAGE), requestReference);
   }
   else
   {
     Serial.println("!ERROR! Unable to request free messages remaining, transceiver is not ready to send messages.");
   }
 }
 
 /**
  * Called by the transceiver to request application provide a segment of send message data.
  *
  * Data flow direction: Application -> Transceiver.
  *
  * @param[in]     topicId       - Message endpoint topic Id.
  * @param[in]     messageId     - Transceiver id for the message.
  * @param[in]     segmentLength - Length of message data (starting from segmentStart) requested by the transceiver.
  * @param[in]     segmentStart  - Offset of message data (from the start of the message) requested by the transceiver.
  * @param[in,out] buffer        - Pointer to a buffer to store message data provided by the application.
  * @param[in]     bufferSize    - Size (capacity) of the buffer in bytes.
  *
  * @retval true  - Message data provided.
  * @retval false - Failed to obtain message data.
  */
 static bool sendMessageDataCallback(int topicId, int messageId, int segmentLength, int segmentStart, unsigned char buffer[], int bufferSize)
 {
   if ((buffer == nullptr) || (segmentLength <= 0) || (segmentLength > bufferSize))
   {
     // should never happen
     return (false);
   }
 
   if ((topicId != HTTP_TOPIC_ID) && (topicId != FREE_MESSAGES_TOPIC_ID))
   {
     Serial.print(MESSAGE_EVENT_PROMPT);
     Serial.print("Warning: Ignoring unexpected request for message data for ");
     Serial.print(convertTopicIdToString(topicId));
     Serial.print(" topic.");
     Serial.println();
     return (false);
   }
 
   Serial.print(SEND_TO_TRANSCEIVER_PROMPT);
   Serial.print("Provide data segment[");
   Serial.print(segmentStart);
   Serial.print("..");
   Serial.print(segmentStart + segmentLength - 1);
   Serial.print("] for message ");
   Serial.print(messageId);
   Serial.print(" on ");
   Serial.print(convertTopicIdToString(topicId));
   Serial.print(" topic to transceiver.");
   Serial.println();
 
   if (topicId == FREE_MESSAGES_TOPIC_ID)
   {
     memcpy(buffer, &REQUEST_FREE_MESSAGES_REMAINING_MESSAGE[segmentStart], segmentLength);
     return (true);
   }
 
   // HTTP Topic
   if (updateMessageId != messageId)
   {
     Serial.print(MESSAGE_EVENT_PROMPT);
     Serial.print("!ERROR! Data for message ");
     Serial.print(messageId);
     Serial.print(" to ");
     Serial.print(convertTopicIdToString(topicId));
     Serial.print(" topic was lost (not sent in time, buffer reused).");
     Serial.println();
   }
 
   memcpy(buffer, &updateMessageBuffer[segmentStart], segmentLength);
   return (true);
 }
 
 /**
  * Called by the transceiver to deliver send message status to the application.
  *
  * Data flow direction: Transceiver -> Application.
  *
  * @param[in] topicId          - Message endpoint topic Id.
  * @param[in] requestReference - The message's request reference id provided to the transceiver by the application.
  * @param[in] messageId        - Transceiver id for the message.
  * @param[in] status           - Send message status.
  */
 static void sendMessageStatusCallback(int topicId, int requestReference, int messageId, SendMessageStatus status)
 {
   if ((topicId != HTTP_TOPIC_ID) && (topicId != FREE_MESSAGES_TOPIC_ID))
   {
     Serial.print(MESSAGE_EVENT_PROMPT);
     Serial.print("Warning: Ignoring unexpected message status for ");
     Serial.print(convertTopicIdToString(topicId));
     Serial.print(" topic.");
     Serial.println();
     return;
   }
 
   switch (status)
   {
     case SEND_MESSAGE_STATUS_ACCEPTED:
       Serial.print(RECEIVE_FROM_TRANSCEIVER_PROMPT);
       Serial.print("Message (request reference ");
       Serial.print(requestReference);
       Serial.print(") accepted for sending as transceiver message id ");
       Serial.print(messageId);
       Serial.print(" to ");
       Serial.print(convertTopicIdToString(topicId));
       Serial.print(" topic.");
       Serial.println();
 
       if (topicId == HTTP_TOPIC_ID)
       {
         if (requestReference != updateMessageRequestReference)
         {
           Serial.print(MESSAGE_EVENT_PROMPT);
           Serial.print("!ERROR! Unexpected message request reference ");
           Serial.print(requestReference);
           Serial.print(") accepted for sending as transceiver message id ");
           Serial.print(messageId);
           Serial.print(" to ");
           Serial.print(convertTopicIdToString(topicId));
           Serial.print(" topic.");
           Serial.println();
         }
 
         /*
           Remember new message's id in case transceiver asks for an older message's
           data which is now lost (overwritten by new message in the message buffer).
         */
         updateMessageId = messageId;
       }
     break;
 
     case SEND_MESSAGE_STATUS_SUCCESS:
       if (topicId == HTTP_TOPIC_ID)
       {
         updateMessageId = INVALID_MESSAGE_ID;
       }
 
       Serial.print(RECEIVE_FROM_TRANSCEIVER_PROMPT);
       Serial.print("Message ");
       Serial.print(messageId);
       Serial.print(" sent successfully to ");
       Serial.print(convertTopicIdToString(topicId));
       Serial.print(" topic.");
       Serial.println();
     break;
 
     default:
     case SEND_MESSAGE_STATUS_SUBSCRIPTION_INVALID:
     case SEND_MESSAGE_STATUS_DISCARDED_ON_OVERFLOW:
     case SEND_MESSAGE_STATUS_CANCELLED_PRE_TRANSIT:
     case SEND_MESSAGE_STATUS_CANCELLED_IN_TRANSIT:
     case SEND_MESSAGE_STATUS_DISCARDED_ON_OVERLOW:
     case SEND_MESSAGE_STATUS_EXPIRED:
     case SEND_MESSAGE_STATUS_TRANSFER_TIMEOUT:
     case SEND_MESSAGE_STATUS_SEGMENT_NOT_SUPPLIED:
     case SEND_MESSAGE_STATUS_SEGMENT_INCORRECT:
     case SEND_MESSAGE_STATUS_NETWORK_ERROR:
     case SEND_MESSAGE_STATUS_PROTOCOL_ERROR:
     case SEND_MESSAGE_STATUS_LOCAL_CRC_ERROR:
     case SEND_MESSAGE_STATUS_CRC_ERROR_IN_TRANSFER:
     case SEND_MESSAGE_STATUS_USER_SUPPLIED_CRC_ERROR:
       if (topicId == HTTP_TOPIC_ID)
       {
         updateMessageId = INVALID_MESSAGE_ID;
       }
 
       Serial.print(MESSAGE_EVENT_PROMPT);
       Serial.print("!ERROR! Message ");
       Serial.print(messageId);
       Serial.print(" to ");
       Serial.print(convertTopicIdToString(topicId));
       Serial.print(" topic failed to send (");
       Serial.print(transceiver.getStatusDescription(status));
       Serial.print(").");
       Serial.println();
     break;
   }
 }
 
 /**
  * Called by the transceiver to deliver received message data to the application.
  *
  * Data flow direction: Transceiver -> Application.
  *
  * @param[in] topicId       - Message origin topic Id.
  * @param[in] messageId     - Transceiver id for the message.
  * @param[in] segmentLength - Length of message data (starting from segmentStart) received by the transceiver.
  * @param[in] segmentStart  - Offset of message data (from the start of the message) received by the transceiver.
  * @param[in] buffer        - Pointer to a buffer to store message data provided by this sketch.
  */
 static void receiveMessageDataCallback(int topicId, int messageId, int segmentLength, int segmentStart, const unsigned char buffer[])
 {
   if ((topicId != HTTP_TOPIC_ID) && (topicId != FREE_MESSAGES_TOPIC_ID))
   {
     Serial.print(MESSAGE_EVENT_PROMPT);
     Serial.print("Warning: Ignoring unexpected message data from ");
     Serial.print(convertTopicIdToString(topicId));
     Serial.print(" topic.");
     Serial.println();
     return;
   }
 
   if (messageId != receivingMessageId)
   {
     Serial.print(MESSAGE_EVENT_PROMPT);
     Serial.print("!ERROR! discarding unexpected message ");
     Serial.print(messageId);
     Serial.print(" data from ");
     Serial.print(convertTopicIdToString(topicId));
     Serial.print(" topic.");
     Serial.println();
     return;
   }
 
   if ((segmentStart + segmentLength) <= sizeof(receiveMessageBuffer))
   {
     Serial.print(RECEIVE_FROM_TRANSCEIVER_PROMPT);
     Serial.print("Received data segment[");
     Serial.print(segmentStart);
     Serial.print("..");
     Serial.print(segmentStart + segmentLength - 1);
     Serial.print("] for message ");
     Serial.print(messageId);
     Serial.print(" from ");
     Serial.print(convertTopicIdToString(topicId));
     Serial.print(" topic.");
     Serial.println();
 
     receivedMessageLength = max(receivedMessageLength, (segmentStart + segmentLength));
     memcpy(&receiveMessageBuffer[segmentStart], buffer, segmentLength);
   }
   else
   {
     Serial.print(MESSAGE_EVENT_PROMPT);
     Serial.print("!ERROR! receive buffer overflow (");
     Serial.print((segmentStart + segmentLength - 1));
     Serial.print(" bytes) for message ");
     Serial.print(messageId);
     Serial.print(" on ");
     Serial.print(convertTopicIdToString(topicId));
     Serial.print(" topic.");
     Serial.println();
   }
 }
 
 /**
  * Called by the transceiver to deliver receive message status to the application.
  *
  * NOTE: maxMessageLength is only valid when status is RECEIVE_MESSAGE_START.
  *       The actual message length may be smaller.
  *
  * Data flow direction: Transceiver -> Application.
  *
  * @param[in] topicId          - Message origin topic Id.
  * @param[in] messageId        - Transceiver id for the message.
  * @param[in] maxMessageLength - Maximum message length in bytes including the CRC.
  * @param[in] status           - Receive message status.
  */
 static void receiveMessageStatusCallback(int topicId, int messageId, int maxMessageLength, ReceiveMessageStatus status)
 {
   if ((topicId != HTTP_TOPIC_ID) && (topicId != FREE_MESSAGES_TOPIC_ID))
   {
     Serial.print(MESSAGE_EVENT_PROMPT);
     Serial.print("Warning: Ignoring unexpected message status from ");
     Serial.print(convertTopicIdToString(topicId));
     Serial.print(" topic.");
     Serial.println();
     return;
   }
 
   switch (status)
   {
     case RECEIVE_MESSAGE_START:
       Serial.print(RECEIVE_FROM_TRANSCEIVER_PROMPT);
       Serial.print("Starting to receive message ");
       Serial.print(messageId);
       Serial.print(" of at most ");
       Serial.print(maxMessageLength);
       Serial.print(" bytes from ");
       Serial.print(convertTopicIdToString(topicId));
       Serial.print(" topic.");
       Serial.println();
 
       if (receivingMessageId != INVALID_MESSAGE_ID)
       {
         Serial.print(MESSAGE_EVENT_PROMPT);
         Serial.println("!ERROR! Receiving multiple messages at the same time (receive buffer overwrite).");
       }
 
       // Start receiving a new message
       memset(receiveMessageBuffer, 0, sizeof(receiveMessageBuffer));
       receivingMessageId = messageId;
     break;
 
     case RECEIVE_MESSAGE_COMPLETE:
       Serial.print(RECEIVE_FROM_TRANSCEIVER_PROMPT);
       Serial.print("Message ");
       Serial.print(messageId);
       Serial.print(" from ");
       Serial.print(convertTopicIdToString(topicId));
       Serial.print(" topic successfully received.");
       Serial.println();
 
       if (receivingMessageId != INVALID_MESSAGE_ID)
       {
         processReceivedMessage(topicId, messageId);
         memset(receiveMessageBuffer, 0, sizeof(receiveMessageBuffer));
         receivedMessageLength = 0;
         receivingMessageId = INVALID_MESSAGE_ID;
       }
       else
       {
         Serial.print(MESSAGE_EVENT_PROMPT);
         Serial.print("!ERROR! Unexpected completion of message ");
         Serial.print(messageId);
         Serial.print(" from ");
         Serial.print(convertTopicIdToString(topicId));
         Serial.print(" topic.");
         Serial.println();
       }
     break;
 
     default:
     case RECEIVE_MESSAGE_CANCELLED:
     case RECEIVE_MESSAGE_FAILED_TIME_OUT:
     case RECEIVE_MESSAGE_FAILED_CRC_ERROR:
       Serial.print(MESSAGE_EVENT_PROMPT);
       Serial.print("!ERROR! Receiving message ");
       Serial.print(messageId);
       Serial.print(" from ");
       Serial.print(convertTopicIdToString(topicId));
       Serial.print(" topic failed (");
       Serial.print(transceiver.getStatusDescription(status));
       Serial.print(").");
       Serial.println();
 
       if (receivingMessageId == messageId)
       {
         // This message will never be fully received
         receivingMessageId = INVALID_MESSAGE_ID;
       }
     break;
   }
 }
 
 /**
  * Create a dashboard message.
  *
  * @param[in] dashboardData - Snapshot of the data used for message creation.
  * @param[in] buffer        - Pointer to a buffer to store message.
  * @param[in] bufferSize    - Size (capacity) of buffer in bytes.
  *
  * @return Length of the created message in bytes.
  */
 int createDashboardMessage(const DashboardData & dashboardData, char buffer[], int bufferSize)
 {
   /*
     Dashboard message template used by snprintf to create the dashboard message.
 
     The message template describes JSON data that contains an array of two elements
     that describe an HTTP command, HTTP URL pair.
   */
   const char MESSAGE_TEMPLATE[] =
     "{"                            // JSON data begin
     "\"requests\":"                // JSON key:value pair begin "array":
     "["                            // JSON array begin
       "{"                            // JSON array element begin
         "\"cmd\":\"GET\""              // JSON key:value pair: "cmd" : "get"
         ","                            // JSON key:value pair separator
         "\"url\":"                     // JSON key:value pair: "url" : "<value>"
         "\"https://blynk.cloud"        //  <value> Blynk cloud URL
         "/external/api/get?"           //  <value> Blynk HTTPS API Get Datastream Value
         "token=%s"                     //  <value> Blynk Blueprint token
         "&V0&V13&V14&V15&V19&V20&V21\""        //  <value> Blynk datastreams"
                                        // JSON key:value pair end
       "}"                            // JSON array element end
       ","                          // JSON array element deliminator
       "{"                            // JSON array element begin
         "\"cmd\":\"GET\""              // JSON key:value pair: "cmd" : "get"
         ","                            // JSON key separator
         "\"url\":"                     // JSON key:value pair: "url" : "<value>"
         "\"https://blynk.cloud"        //  <value> Blynk cloud URL
         "/external/api/batch/update?"  //  <value> Blynk HTTPS API Batch Update
         "token=%s"                     //  <value> Blynk Blueprint token
         "&V0=0"                        //  <value> Blynk datastream "Alert Switch" = OFF (reset switch)
         "&V1=%d"                       //  <value> Blynk datastream "Signal Bars"
         "&V2=%.2f"                     //  <value> Blynk datastream "Battery Voltage"
         "&V3=%s"                       //  <value> Blynk datastream "Update Reason" string
        //  "&V4=%s"                       //  <value> Blynk datastream "GNSS Data" string (date time UTC, latitude, longitude, speed)
                                        // Premium Blueprint Data Begin
         "&V5=%d"                       //  <value> Blynk datastream "Temperature"
         "&V6=%d"                       //  <value> Blynk datastream "Signal Level" (transceiver signal strength in dbm)
         "&V7=%s"                       //  <value> Blynk datastream "GNSS Timestamp string" (date time UTC)
         "&V8=%.6f&V8=%.6f"                       //  <value> Blynk datastream "Longitude,Latitude" map coordinates
        //  "%s"                           //  <value> Blynk datastream "Latitude,Longitude" map coordinates
         "&V9=%s"                       //  <value> Blynk datastream "Latitude Longitude" string
        //  "&v10=%d"                      //  <value> Blynk datastream "Altitude"
         "&v11=%d"                      //  <value> Blynk datastream "Speed"
         "&v12=%d\""                    //  <value> Blynk datastream "Heading"
                                      // JSON key:value pair end
       "}"                            // JSON array element end
     "]"                            // JSON array end
     "}";                           // JSON data end
 
   const char GNSS_NO_FIX_STRING[] = "No Fix";
 
   char gnssDateTimeString[25];  /**< Buffer for Blynk datastream "GNSS Timestamp" string (V7) (date time UTC). */
   char gnssLatLonString[26];    /**< Buffer for Blynk datastream "GNSS Latitude Longitude" string (V9) (latitude, longitude). */
   char gnssDataString[63];      /**< Buffer for Blynk datastream "GNSS Data" string (V4) (date time UTC, latitude, longitude, speed). */
   char mapLatLongString[28];    /**< Buffer for Blynk datastream "Map Coordinates" (V8) (Latitude Longitude). */
 
   // initialize string buffers to default value:
   snprintf(gnssDataString, sizeof(gnssDataString), GNSS_NO_FIX_STRING);
   snprintf(gnssLatLonString, sizeof(gnssLatLonString), GNSS_NO_FIX_STRING);
   snprintf(gnssDateTimeString, sizeof(gnssDateTimeString), GNSS_NO_FIX_STRING);
   snprintf(mapLatLongString, sizeof(mapLatLongString), EMPTY_STRING);
 
   if (dashboardData.gnss.timeValid == true)
   {
     snprintf(gnssDateTimeString, sizeof(gnssDateTimeString), "%04d-%3s-%02d %02d:%02d:%02d UTC",
                  dashboardData.gnss.year,
                  convertMonthToString(dashboardData.gnss.month),
                  dashboardData.gnss.day,
                  dashboardData.gnss.hour,
                  dashboardData.gnss.minute,
                  dashboardData.gnss.second);
 
     // Copy date and time string into GNSS data string as default value if no fix
     snprintf(gnssDataString, sizeof(gnssDataString), "%s", gnssDateTimeString);
   }
 
   if (dashboardData.gnss.fixValid == true)
   {
     snprintf(gnssLatLonString, sizeof(gnssLatLonString), "%.5f\xc2\xb0%c, %.5f\xc2\xb0%c",
                  abs(dashboardData.gnss.latitudeDegrees),
                  getLatitudeDirection(dashboardData.gnss.latitudeDegrees),
                  abs(dashboardData.gnss.longitudeDegrees),
                  getLongitudeDirection(dashboardData.gnss.longitudeDegrees));
 
     snprintf(gnssDataString, sizeof(gnssDataString), "%s, %s, %d km/h",
               gnssDateTimeString,
               gnssLatLonString,
               dashboardData.gnss.speedKph);
 
     snprintf(mapLatLongString, sizeof(mapLatLongString), "&V8=%.5f&V8=%.5f",
               dashboardData.gnss.longitudeDegrees,
               dashboardData.gnss.latitudeDegrees);
   }

   // Generate the message using the message template
   int length = snprintf(buffer, (bufferSize - MESSAGE_CRC_LENGTH), MESSAGE_TEMPLATE,
                         BLYNK_AUTH_TOKEN,       // Array[0] data
                         BLYNK_AUTH_TOKEN,       // Array[1] data
                         dashboardData.signalBars,
                         dashboardData.batteryVoltage,
                         convertUpdateTypeToString(dashboardData.updateType),
                        //  gnssDataString,
                         dashboardData.temperatureDegC,
                         dashboardData.signalStrengthdBm,
                         gnssDateTimeString,
                         dashboardData.gnss.longitudeDegrees,
                         dashboardData.gnss.latitudeDegrees,
                         gnssLatLonString,
                        //  mapLatLongString,
                        //  gnssLatLonString,
                        //  dashboardData.gnss.altitudeMeters,
                         dashboardData.gnss.speedKph,
                         dashboardData.gnss.headingDegrees);
 
   if (length >= (bufferSize - MESSAGE_CRC_LENGTH))
   {
     Serial.print(MESSAGE_EVENT_PROMPT);
     Serial.println("!ERROR! Message buffer overflow.");
     return (0);
   }
 
   // Calculate message CRC and append to message
   int  crc = TIL_crc(buffer, length, 0);
   byte crcMSB = (crc & 0xFF00) >> 8;
   byte crcLSB = (crc & 0x00FF);
 
   buffer[length++] = crcMSB;
   buffer[length++] = crcLSB;
 
   Serial.print(MESSAGE_EVENT_PROMPT);
   Serial.print("Update message created: Signal[");
   Serial.print(dashboardData.signalBars);
   Serial.print(" bar");
   if (dashboardData.signalBars != 1)
   {
     Serial.print("s");
   }
   Serial.print(", ");
   Serial.print(dashboardData.signalStrengthdBm);
   Serial.print(" dBm], Temp: ");
   Serial.print(dashboardData.temperatureDegC);
   Serial.print("\xc2\xb0");
   Serial.print("C, Battery: ");
   Serial.print(dashboardData.batteryVoltage);
   Serial.print("V, GNSS: ");
   Serial.print(gnssDataString);
   if (dashboardData.gnss.fixValid == true)
   {
     Serial.print(", ");
     Serial.print(dashboardData.gnss.headingDegrees);
     Serial.print("\xc2\xb0");
   }
   Serial.println();
 
   return (length);
 }
 
 /**
  * Process a received message.
  *
  * @param[in] topicId   - Message origin topic Id.
  * @param[in] messageId - Transceiver id for the message.
  */
 static void processReceivedMessage(int topicId, int messageId)
 {
   if (receivedMessageLength <= MESSAGE_CRC_LENGTH)
   {
     // This should be impossible.
     return;
   }
 
   // Check message CRC
   int crcMsb = receiveMessageBuffer[receivedMessageLength - 2];
   int crcLsb = receiveMessageBuffer[receivedMessageLength - 1];
   int messageCRC = (crcMsb << 8) | crcLsb;
 
   int checkCrc = TIL_crc(receiveMessageBuffer, (receivedMessageLength - MESSAGE_CRC_LENGTH), 0);
   if (messageCRC != checkCrc)
   {
     Serial.print(MESSAGE_EVENT_PROMPT);
     Serial.println("!ERROR! Discarding message received with invalid CRC.");
     return;
   }
 
   // Convert to string (for deserialization) by replacing CRC with 0
   receiveMessageBuffer[receivedMessageLength - 2] = 0;
   char * jsonString = receiveMessageBuffer;
 
   switch (topicId)
   {
     case HTTP_TOPIC_ID:
       processHttpResponse(jsonString);
     break;
 
     case FREE_MESSAGES_TOPIC_ID:
       processFreeMessagesResponse(jsonString);
     break;
 
     default:
       Serial.print(MESSAGE_EVENT_PROMPT);
       Serial.println("!ERROR! Message received for unknown topic.");
     break;
   }
 }
 
 /**
  * Process message received from HTTP topic.
  *
  * @param[in] jsonString - String containing JSON data.
  */
 static void processHttpResponse(const char jsonString[])
 {

   Serial.println("*** debug2");
   Serial.println(jsonString);
   Serial.println("*** debug2 end");

   /*
     Use ZERO-COPY when deserializing the JSON data.
     This means the string containing the data will be modified in memory
   */
   deserializeJson(jsonDoc, (char *)jsonString);
 
   jsonObject = jsonDoc["responses"][0];
   int alertStatus = jsonObject["status"];
 
   const int HTTP_SUCCESS = 200;
   if (alertStatus == HTTP_SUCCESS)
   {
     processReply();
   }
   return;
 }
 
 /**
  * Process message received from Free Message topic.
  *
  * @param[in] jsonString - String containing JSON data.
  */
 static void  processFreeMessagesResponse(const char jsonString[])
 {
   /*
     Use ZERO-COPY when deserializing the JSON data.
     This means the string containing the data will be modified in memory.
   */
   deserializeJson(jsonDoc, (char *)jsonString);
 
   int freeMessagesRemaining = jsonDoc["free_messages_remaining"];
   Serial.print(MESSAGE_EVENT_PROMPT);
   Serial.print("You have ");
   Serial.print(freeMessagesRemaining);
   Serial.println(" free messages remaining.");
 }
 
 /**
  * Get message request reference.
  *
  * A message's request reference is a unique identifier provided by the application
  * to the transceiver so that the application can later associate a message with the
  * transceiver's message id.
  *
  * @return Current message request reference.
  */
 static int getMessageRequestReference(void)
 {
   int answer = nextRequestReference;
 
   nextRequestReference += 1;
   if (nextRequestReference > MAX_MESSAGE_REQUEST_REFERENCE)
   {
     nextRequestReference = MIN_MESSAGE_REQUEST_REFERENCE;
   }
   return (answer);
 }
 
 /**
  * Determine the latitude direction (North or South).
  * North latitudes are greater than or equal to 0.
  * South latitudes are less than 0.
  *
  * @param[in] latitudeDeg - Latitude in decimal degrees.
  *
  * @retval 'N' - North latitudes.
  * @retval 'S' - South latitudes.
  */
 static char getLatitudeDirection(const float latitudeDeg)
 {
   if (latitudeDeg >= 0.0f)
   {
     return ('N');
   }
 
   return ('S');
 }
 
 /**
  * Determine the longitude direction (East or West).
  * East longitudes are greater than or equal to 0.
  * West longitudes are less than 0.
  *
  * @param[in] longitudeDeg - Longitude in decimal degrees.
  *
  * @retval 'E' - East longitudes.
  * @retval 'W' - West longitudes.
  */
 static char getLongitudeDirection(const float longitudeDeg)
 {
   if (longitudeDeg >= 0.0f)
   {
     return ('E');
   }
 
   return ('W');
 }
 
 /**
  * Convert a topic id to a string.
  *
  * @param[in] topicId - Topic id.
  *
  * @return Pointer to a string containing the topic id name.
  */
 static const char * convertTopicIdToString(const int topicId)
 {
   switch (topicId)
   {
     case REFLECTOR_TOPIC_ID:     return ("Reflector");
     case FREE_MESSAGES_TOPIC_ID: return ("Free Messages");
     case HTTP_TOPIC_ID:          return ("HTTP");
   }
 
   return ("unknown");
 }
 
 /**
  * Convert update type to a string.
  *
  * @param[in] updateType - Update type.
  *
  * @return Pointer to a string containing the update type.
  */
 static const char * convertUpdateTypeToString(const UpdateType updateType)
 {
   switch (updateType)
   {
     case UPDATE_TYPE_PERIODIC:     return ("Periodic");
     case UPDATE_TYPE_BUTTON_PRESS: return ("Button Press");
   }
 
   return (EMPTY_STRING);
 }
 
 /**
  * Convert month to string.
  *
  * @param[in] month - Month.
  *
  * @return String representing month number.
  */
 static const char * convertMonthToString(int month)
 {
   switch (month)
   {
     case 1:  return "JAN";
     case 2:  return "FEB";
     case 3:  return "MAR";
     case 4:  return "APR";
     case 5:  return "MAY";
     case 6:  return "JUN";
     case 7:  return "JUL";
     case 8:  return "AUG";
     case 9:  return "SEP";
     case 10: return "OCT";
     case 11: return "NOV";
     case 12: return "DEC";
   }
 
   return (EMPTY_STRING);
 }
 
 /**
  * Pad string
  *
  * @param[in]      length           - Final length of padded string.
  * @param[in, out] string           - Pointer to string to pad.
  * @param[in]      stringBufferSize - Size (capacity) of string buffer in bytes.
  */
 static void padString(int length, char string[], int stringBufferSize)
 {
   if (string == 0)
   {
     return;
   }
 
   int stringLength = strlen(string);
   while ((stringLength < length) && ((stringLength + 1) < stringBufferSize))
   {
     string[stringLength++] = ' ';
     string[stringLength] = 0;
   }
 }

void processReply(void) 
{
    Serial.print(MESSAGE_EVENT_PROMPT);
    Serial.println("Received reply from IoT dashboard, update successful.");

    // Serial.println("*** debug3:");
    // String s = jsonObject["data"];
    // Serial.println(s);

    // Extract the data field which contains our nested JSON string
    String cleanNestedJson = jsonObject["data"];
    if (cleanNestedJson.length() == 0)
    {
      return;
    }

    // Replace single quotes with double quotes to conform with JSON spec
    cleanNestedJson.replace("'", "\"");  

    JsonDocument nestedDoc;
    DeserializationError error = deserializeJson(nestedDoc, cleanNestedJson);
    if (error) 
    {
      Serial.printf("Nested JSON parsing failed: %s\n", error.c_str());
      Serial.printf(":%s:\n", cleanNestedJson);
      return;
    }    

    processAlarm(nestedDoc);
    processGeoFence(nestedDoc);
    processUpdateFreq(nestedDoc);
    processAuxOuts(nestedDoc);    
}

/*
* Sounds the Alarm if set on the dashboard
*/
void processAlarm(JsonDocument& nestedDoc) 
{
  if (nestedDoc["V0"] == 1) 
  {
    Serial.println("** Sound Alarm Received! **");
    buzzer.playToneSequence(ALERT_TONE_SEQUENCE);
  }
}

/*
* Sets the GEO Fence Radius
*/
void processGeoFence(JsonDocument& nestedDoc) 
{
  int a = nestedDoc["V14"];
  if (geoFenceAlarm != a)
  {
    previousGeoFenceAlarm = geoFenceAlarm;
    geoFenceAlarm = a;
    Serial.printf("GEO Fence Alarm changed to %i\n", geoFenceAlarm);
  }

  int r = nestedDoc["V13"];
  if (geoFenceRadius != r)
  {
    geoFenceRadius = r;
    Serial.printf("GEO Fence Radius changed to %i\n", geoFenceRadius);
  }
}

/*
* Sets the time between updates
  // TODO: use actual time values
*/
void processUpdateFreq(JsonDocument& nestedDoc) 
{
  int updateFreq = nestedDoc["V15"];
  int updateFreqMs = updateFreq * 1000;
  if (updateFreqMs != dashboardUpdateTimer.getDurationMs())
  {
     dashboardUpdateTimer.setDurationMs(updateFreqMs);
     Serial.printf("dashboard update interval set to %i secs\n", updateFreq);
  }
}


/*
* Sets the Auxilary Output pins from the Dashboard values
*/
void processAuxOuts(JsonDocument& nestedDoc) 
{
  // Set pins according to extracted values
  digitalWrite(AUX_OUT1, nestedDoc["V19"] ? HIGH : LOW);  
  digitalWrite(AUX_OUT2, nestedDoc["V20"] ? HIGH : LOW);
  digitalWrite(AUX_OUT3, nestedDoc["V21"] ? HIGH : LOW);
  
  // Debug output
  Serial.println("Aux Out Pin States:");
  Serial.printf("AUX_OUT1, V19 (Pin 5): %i\n", digitalRead(AUX_OUT1));
  Serial.printf("AUX_OUT2, V20 (Pin 4): %i\n", digitalRead(AUX_OUT2));
  Serial.printf("AUX_OUT3, V21 (Pin 7): %i\n", digitalRead(AUX_OUT3));
}

/**
 * Calculate distance between two points, A and B, on Earth's surface
 * @param latA Point A latitude in decimal degrees
 * @param lonA Point A longitude in decimal degrees
 * @param latB Point B latitude in decimal degrees
 * @param lonB Point B longitude in decimal degrees
 * @return Distance in meters
 */
float calculateDistance(float latA, float lonA, float latB, float lonB)
{
  // Earth's radius in meters
  const float R = 6371000.0;
  
  // Convert degrees to radians
  float latARad = radians(latA);
  float lonARad = radians(lonA);
  float latBRad = radians(latB);
  float lonBRad = radians(lonB);
  
  // Difference in coordinates
  float dLat = latBRad - latARad;
  float dLon = lonBRad - lonARad;
  
  // Haversine formula
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(latARad) * cos(latBRad) * 
            sin(dLon/2) * sin(dLon/2);
  
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  // Distance in meters
  float distance = R * c;
  
  return distance;
}

// void testCalculateDistance(void) 
// {
//   float ptALat = 51.590590763654625;
//   float ptALon = -0.09748588689186495;
//   float ptBLat = 51.59069197240872;
//   float ptBLon = -0.09402955955255156;
//   int expected = 239;
//   float d = calculateDistance(ptALat, ptALon, ptBLat, ptBLon);
//   Serial.printf("*** distance point a to point b: %.0f, expected: %i\n", d, expected);
// }

bool checkGeoFenceBreach(GnssData gnss) 
{
  Serial.println("chkGeo dbg 1");
  Serial.printf("geo alarm %i\n", geoFenceAlarm);
  Serial.printf("geo radius %i\n", geoFenceRadius);
  Serial.printf("geo fence lat,lon %.5f,%.5f\n", geoFenceLat, geoFenceLon);
  Serial.printf("gnss fix %i\n", gnss.fixValid);
  Serial.printf("gnss lat,lon %.5f,%.5f\n", gnss.latitudeDegrees, gnss.longitudeDegrees);
  
  if (!gnss.fixValid || (geoFenceAlarm != 1)) 
  {
    return false;
  } 
  Serial.println("chkGeo dbg 2");
  if (isnan(geoFenceLat) || isnan(geoFenceLon) || isnan(gnss.latitudeDegrees) || isnan(gnss.longitudeDegrees)) 
  {
    return false;
  } 

  Serial.println("chkGeo dbg 3");
  float fenceDistance = calculateDistance(gnss.latitudeDegrees, gnss.longitudeDegrees, geoFenceLat, geoFenceLon);
  bool geoFenceBreached = fenceDistance > geoFenceRadius;
  Serial.printf("chkGeo dbg 4 distance=%d, breached: %d\n", fenceDistance, geoFenceBreached);

  if (!geoFenceBreached) 
  {
    return false;
  }

  sendGeoFenceBreachedEvent();
  return true;
}

void sendEvent(const char* eventType, const char* eventDescription) 
{
  int length = prepareEventMessage(updateMessageBuffer, eventType);

  updateMessageRequestReference = getMessageRequestReference();

  Serial.print(SEND_TO_TRANSCEIVER_PROMPT);
  Serial.println(eventDescription);
  transceiver.messageSend(HTTP_TOPIC_ID, length, updateMessageRequestReference);

  // if (strcmp(eventType, "geofence") == 0) {
  //   delay(20000); // Only for geofence events
  // }
}

int prepareEventMessage(char* buffer, const char* eventCode) {
  static const char baseMessageTemplate[] =
  "{"                            // JSON data begin
  "\"requests\":"                // JSON key:value pair begin "array":
  "["                            // JSON array begin
   "{"                                       // JSON data begin
     "\"cmd\":\"GET\","                      // JSON key:value pair
     "\"url\":\"https://blynk.cloud/external/api/logEvent?token=" BLYNK_AUTH_TOKEN "&code=%s\""  // Using the defined token
   "}"
  "]"                            // JSON array end
  "}";                           // JSON data end

  // Calculate the required buffer size
  size_t neededSize = snprintf(NULL, 0, baseMessageTemplate, eventCode) + 1;
  char formattedMessage[neededSize];
  snprintf(formattedMessage, neededSize, baseMessageTemplate, eventCode);

  size_t baseLength = strlen(formattedMessage);
  memcpy(buffer, formattedMessage, baseLength);
  
  // Calculate message CRC and append to message
  int crc = TIL_crc(buffer, baseLength, 0);
  byte crcMSB = (crc & 0xFF00) >> 8;
  byte crcLSB = (crc & 0x00FF);
  buffer[baseLength] = crcMSB;
  buffer[baseLength + 1] = crcLSB;
  buffer[baseLength + 2] = 0x00;
  
  return baseLength + 2; // Base message length + 2 CRC bytes
}

// Wrapper functions for specific event types
void sendGeoFenceBreachedEvent(void) {
  sendEvent("geofence", "GEO Fence Breached event.");
}

void sendSOSEvent(void) {
  sendEvent("sos", "User button pressed for SOS event.");
}
