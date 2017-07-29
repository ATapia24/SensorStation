#include <SPI.h>
#include <RH_RF69.h>

// Frequency in MGhz
#define FREQ 433.0

// Pin configuration
#define CS 4
#define G0 3
#define RS 2

// Sensor pins
#define TEMP 10

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

// Function prototypes
void handleReplyInstruction(uint8_t*);
bool retryConnection(uint8_t*);

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
  radio.setTxPower(20, true);

  // Create an encryption key. This must be the same as the Rx transciever
  uint8_t key[] = { 0x05, 0x03, 0x01, 0x05, 0x06, 0x08, 0x02, 0x03,
                    0x08, 0x02, 0x01, 0x08, 0x07, 0x03, 0x04, 0x02};

  // Set the key
  radio.setEncryptionKey(key);

  Serial.println("The radio has been setup, and is ready to go");

}

enum Status {
  NORMAL,
  NO_REPLY,
  ERROR
};

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

  // Attemp to send the data to the station
  radio.send(transmit, sizeof(transmit));
  radio.waitPacketSent();

  delete [] transmit; // Cleanup

  // Wait for a reply
  if (radio.waitAvailableTimeout(TIMEOUT)) {

    // Attempt to receive the data
    if (radio.recv(receive, &rSize)) {

      // TODO: Handle reply here  
    
    } else {

      // There was an error receiving data
      // TODO: handle error here

    }

  } else {
    // No reply, so just try again later
    Serial.println("No reply from the station. Trying again later...");
  }

  // Making sure there is at least X seconds between each interval
  delay(DELAY);

}