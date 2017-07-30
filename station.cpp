#include <SPI.h>
#include <RH_RF69.h>

// Frequency in MGhz
#define FREQ 433.0

// Pin configuration
#define CS 4
#define G0 3
#define RS 2

// Sensor pins
#define TEMP A0

// Radio object
RH_RF69 radio(CS, G0);

// etc. Configurations
#define TIMEOUT 500  // The amount of time in milliseconds to wait for a reply before timing out
int TRANS_POWER = 14; // The amount trans power should start out with (changes if the server signals NO_REPLY)

// Status register shift amounts
#define SEND_DATA 0       // Request that the sensor data be sent
#define TRANS_ERROR 1     // There was an error in the data being received
#define REPLY_ERROR 2        // The data was not received!

// Function prototypes
float getTemperature();
long calcChecksum(float&); 

void setup() {

  // Configure serial comm.
  Serial.begin(115200);

  Serial.println("Begining setup");
  
  // Configure appropriate pins as outputs
  pinMode(RS, OUTPUT);   // Reset
  digitalWrite(RS, LOW); // Pull RS pin low

  pinMode(TEMP, INPUT);  // TEMP input

  // Manually reset the radio
  digitalWrite(RS, HIGH);
  delay(10);
  digitalWrite(RS, LOW);
  delay(10);

  // Check to make sure the radio is ready to go
  if (!radio.init()) {
    // Radio didn't start
    Serial.println("Unable to start the radio");
    for(;;) {}
  }

  // Set the frequency and make sure it worked
  if (!radio.setFrequency(FREQ)) {
    // Failed to set the frequency
    Serial.println("Unable to set the frequency");
    for (;;) {}
  }
  
  // Set the transmitting power and 'true' because I am using the HCW model
  radio.setTxPower(TRANS_POWER, true);

  // Create an encryption key. This must be the same as the Rx transciever
  uint8_t key[] = { 0x05, 0x03, 0x01, 0x05, 0x06, 0x08, 0x02, 0x03,
                    0x08, 0x02, 0x01, 0x08, 0x07, 0x03, 0x04, 0x02};

  // Set the key
  radio.setEncryptionKey(key);

  Serial.println("The radio has been setup, and is ready to go");

}

void loop() {

  /*
   * The statoin's job is just to wait for commands from the server, so there
   * will be no delays in this loop (except for data transmission delays)
   */

  if (radio.available()) {

    Serial.println("Data detected, attemtping to parse");

    // These variables will be used to hold data for sending and receiving
    uint8_t receive[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t rSize = sizeof(receive);

    // We should have a message
    if (radio.recv(receive, &rSize)) {

      // Make sure there is actually a message to receive
      if (!rSize) {
        Serial.println("No data received!");
        // return;
      }

      // Check for the status register for instructions
      if (bitSet(receive[0], TRANS_ERROR)) {
        // TODO: not much to do here for now
      }

      if (bitSet(receive[0], REPLY_ERROR)) {
        // Increase the trans power by 1
        if (TRANS_POWER != 20) {
          TRANS_POWER++;
          radio.setTxPower(TRANS_POWER, true);
        }
      }
      
      if (bitSet(receive[0], SEND_DATA)) {
        // Send sensor data over!

        Serial.println("Data requested, sending now...");
        
        float temperature = getTemperature();

        // Build the data to send over
        uint8_t transmit[9];

        // No need to send any commands over the sreg, so just send the data
        union LongSplicer {
          long data;
          byte bytes[4];
        };

        union FloatSplicer {
          float data;
          byte bytes[4];
        };

        // Create the checksum
        long checkSum = calcChecksum(temperature);

        // Breakdown the long into bytes
        LongSplicer sLong;
        sLong.data = checkSum;

        // Breakdown the float into bytes
        FloatSplicer sFloat;
        sFloat.data = temperature;

        // Fillout the data to send, then cleanup
        transmit[1] = sFloat.bytes[0];
        transmit[2] = sFloat.bytes[1];
        transmit[3] = sFloat.bytes[2];
        transmit[4] = sFloat.bytes[3];

        transmit[5] = sLong.bytes[0];
        transmit[6] = sLong.bytes[1];
        transmit[7] = sLong.bytes[2];
        transmit[8] = sLong.bytes[3];

        // Transmit the data over
        radio.send(transmit, sizeof(transmit));
        radio.waitPacketSent();
        
      }
      
    } else {
      // There was an error in receiving the message
      // For now we will do nothing
      Serial.println("Unable to parse data!");
    }
  
  }
  
}

/*
 * Used to get the temperature reading off of the
 * sensor
 */
float getTemperature() {

  float voltage = analogRead(TEMP) * 5.0;
  voltage /= 1024.0;

  float temperatureC = (voltage - 0.5) * 100 ; 
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

  return temperatureF;
  
}

/*
 * Used to calculate the checksum for data checking (because
 * there is a possibility of error)
 */
long calcChecksum(float &temperature) {

  // Removing the decimal on the float.. Since the checksum is a long
  int intTemperature = static_cast<int>(temperature);
  
  return intTemperature;
  
}