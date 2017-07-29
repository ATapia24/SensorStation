#include <SPI.h>
#include <RH_RF69.h>

// Frequency in MGhz
#define FREQ 433.0

// Pin configuration
#define CS 4
#define G0 3
#define RS 2

// Radio object
RH_RF69 radio(CS, G0);

// etc. Configurations
#define RESEND 5      // The amount of times to resend the data
#define RETRY 5       // The amount of times to retry getting a reply before giving up
#define DELAY 5000    // The amount of milliseconds to delay at the end of the loop
#define TIMEOUT 1000  // The amount of time in milliseconds to wait for a reply before timing out

// Status register shift amounts
#define SEND_DATA 0       // Request that the sensor data be sent
#define TRANS_ERROR 1     // There was an error in the data being received
#define REPLY_ERROR 2        // The data was not received!

// Function prototypes
bool validChecksum(uint8_t*, float&);

void setup() {

  // Configure serial comm.
  Serial.begin(115200);

  Serial.println("Begining setup");
  
  // Configure appropriate pins as outputs
  pinMode(RS, OUTPUT);   // Reset
  digitalWrite(RS, LOW); // Pull RS pin low

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
  radio.setTxPower(20, true);

  // Create an encryption key. This must be the same as the Rx transciever
  uint8_t key[] = { 0x05, 0x03, 0x01, 0x05, 0x06, 0x08, 0x02, 0x03,
                    0x08, 0x02, 0x01, 0x08, 0x07, 0x03, 0x04, 0x02};

  // Set the key
  radio.setEncryptionKey(key);

  Serial.println("The radio has been setup, and is ready to go");

}

// A nice set of enumerations to use as 'states'
enum Status {
  NORMAL,
  NO_REPLY,
  ERROR
};

// Used to hold the current state of the server
Status state = NORMAL;

// These variables will be used to hold data for sending and receiving
uint8_t receive[RH_RF69_MAX_MESSAGE_LEN];
uint8_t rSize;

uint8_t *transmit;
uint8_t tSize;

void loop() {

  // Start by requesting data from the weather station
  transmit = new uint8_t[1];

  // Build the byte to send
  transmit[0] = 0x00;
  bitSet(transmit[0], SEND_DATA);

  // Set the appropriate bits depending on the state
  if (state == ERROR) {
    bitSet(transmit[0], TRANS_ERROR);
  } else if (state == NO_REPLY) {
    bitSet(transmit[0], REPLY_ERROR);
  }

  // Attemp to send the data to the station
  radio.send(transmit, sizeof(transmit));
  radio.waitPacketSent();

  delete [] transmit; // Cleanup

  // Wait for a reply
  if (radio.waitAvailableTimeout(TIMEOUT)) {

    // Attempt to receive the data
    if (radio.recv(receive, &rSize)) {

      // Make sure the state is updated to normal
      state = NORMAL;

      // TODO: Handle reply here
      /*
       * We have decided to use a checksum to make sure the data was not corrupted,
       * so that will have to be up to the station to generate the checksum, and the
       * server to check it.
       */

      // A union used for 'converting' the received data to a proper float
      union FloatBuilder {
        float data;
        uint8_t bytes[4];
      };

      // For now we will just be receiving temperature readings
      // Bring the temperature in
      FloatBuilder *specialCast = new FloatBuilder;
      specialCast->bytes[0] = receive[1];
      specialCast->bytes[1] = receive[2];
      specialCast->bytes[2] = receive[3];
      specialCast->bytes[3] = receive[4];

      float temperature = specialCast->data;

      // Cleanup
      delete specialCast;
      
      // Make sure the checksum cleared
      if (!validChecksum(receive, temperature)) {
        Serial.println("Checksum did not validate! Retrying later");
        state = ERROR;
        return;
      }

      // If all is good, then print the data
      Serial.println("Temperature: ");
      Serial.print(temperature, DEC);
      Serial.println("RSSI: ");
      Serial.print(radio.lastRssi());
      
    } else {

      // Make sure the state is updated to error
      state = ERROR;

      // There was an error receiving data
      Serial.println("There was an error in parsing received data! Retrying later");

    }

  } else {
    // No reply, so set the state here
    state = NO_REPLY;
    
    // No reply, so just try again later
    Serial.println("No reply from the station. Trying again later...");
  }

  // Making sure there is at least X second(s) between each interval
  delay(DELAY);

}

/*
 * This function does what its name implies, it uses
 * the provided checksum to make sure the data is valid.
 * 
 * !! We are only expecting temperature data right now !!
 */
bool validChecksum(uint8_t *data, float &temperature) {

  union CheckSum {
    unsigned long value;
    uint8_t bytes[4];
  };

  // Fill out the checksum
  CheckSum check;
  
  check.bytes[0] = data[5];
  check.bytes[1] = data[6];
  check.bytes[2] = data[7];
  check.bytes[3] = data[8];

  // Now compare the two
  return (check.value == static_cast<int>(temperature));

}