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
#define RESEND 5    // The amount of times to resend the data
#define RETRY 5     // The amount of times to retry getting a reply before giving up

// Status register shift amounts
#define NO_REPLY 0   // Set this bit in the status register to signify
// no reply was received
#define ERR 1        // Set this bit in the status register to signify
// There was an error in receiving data

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
		for (;;) {}
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
		0x08, 0x02, 0x01, 0x08, 0x07, 0x03, 0x04, 0x02 };

	// Set the key
	radio.setEncryptionKey(key);

	Serial.println("The radio has been setup, and is ready to go");

}

void loop() {

	/*
	* For now, we are just going to sample a temperature sensor, and send the
	* data 5 times, for error detection.
	*/

	// 10 second delay I was talking about
	delay(1000);

	// Read in the temperature
	int temp = analogRead(TEMP);

	// Convert the data into something we can send
	/*
	*  The first byte is a status register, then the rest is data
	*/
	uint8_t data[] = { 0x00, highByte(temp), lowByte(temp) };

	// Create the necessary variables to hold the reply data
	uint8_t buff[RH_RF69_MAX_MESSAGE_LEN];  // Holds the bytes of data sent
	uint8_t len = sizeof(buff);             // ~~~~
	uint8_t amnt = 0;                        // Keeps track of how many replies we've had

											 // Prompt data being sent
	Serial.println("Attempting to send data");

	// Send the data as many times as defined above
	for (uint8_t i = 0; i < RESEND; i++) {

		// Send the data over
		radio.send(data, sizeof(data));
		// Make sure the data is sent before proceeding
		radio.waitPacketSent();

		// Promt the user
		Serial.println("Packet ");
		Serial.print(i + 1, DEC);
		Serial.print("/");
		Serial.print(RESEND, DEC);
		Serial.print(" sent");

		// Handle the reply from the other radio, if there is any
		if (radio.waitAvailableTimeout(1000)) {

			// We should have waited enough time 
			if (radio.recv(buff, &len)) {

				// Increment received replies
				amnt++;

				// TODO: handle reply instructions from the other radio

				// Check to see if we are on the last iteration, so we can check to see if the data
				// needs to be re-sent
				if ((i + 1) == RESEND) {

					// For kicks and giggles, calculate our reply ratio
					float replyRatio = static_cast<double>(amnt) / static_cast<double>(RESEND);
					Serial.println("Reply reliablity: ");
					Serial.print(replyRatio);
					Serial.print("%");

					// Check to see if we received all the replies
					if (amnt != RESEND) {
						// We didn't get all the replies we wanted
						Serial.println("Not all replies were received, even after multiple retries. Perhaps the server is down?");
					}
					else {
						// We got all the replies back
						Serial.println("We got a reply for all data sent!");
					}

				}
				else {
					// Print out the received data
					Serial.print("Reply: ");
					Serial.println((char*)buff);
				}

			}
			else {
				// We weren't able to receive the data.. for whatever reason
				Serial.println("Unable to receive the data");

				// TODO
			}

		}
		else {

			// Let the user know what's going on
			Serial.println("No reply, attempting to try again");

			// TODO

		}

	}

}

/*
* A simple function that attempts to retry getting a response from the
* other radio.
* @param data The data to send again. Make sure to set the status register
*             accordingly
*/
bool retryConnection(uint8_t *data) {

	// Try a specified amount of times before giving up
	for (uint8_t i = 0; i < RETRY; i++) {

		// Notifications...
		Serial.println("Attempting to resend data");
		Serial.print(" Try #");
		Serial.print(i + 1, DEC);

		// Send the data over
		radio.send(data, sizeof(data));
		// Make sure the data is sent before proceeding
		radio.waitPacketSent();

		// Check to see if the data sent
		if (radio.waitAvailableTimeout(1000)) {

			// Variables to hold the received data
			uint8_t buff[RH_RF69_MAX_MESSAGE_LEN];  // Holds the bytes of data sent
			uint8_t len = sizeof(buff);             // ~~~~

													// Attempt to receive the data from trying the connection
			if (radio.recv(buff, &len)) {

				// We received a reply, now we just handle the reply!
				handleReplyInstruction(buff, data);

				// Return true because we were able to get a reply
				return true;

			}
			else {
				// Sadly, the data receive failed
				Serial.println("An error occurred while trying to receive the data");
			}

		}
		else {

			Serial.println("No reply!");

		}

	}

	// Return false because if we get here.. we weren't able to get a reply
	return false;

}

/*
* This function's job is to take the byte of data that is the header of every message
* and do what it's telling us too!
* @param sReg The status part of the message sent from the other radio
* @param reply The entire reply message
* @param data The data that was sent to 'create' the reply
*/
void handleReplyInstruction(uint8_t *prev, uint8_t *data) {

	if (bitRead(prev[0], NO_REPLY) == 1 || bitRead(prev[0], ERR)) {
		// The radio isn't getting replies from us, or the data was corrupted
		// Either way, we need to resend our data

		// TODO: resend the data

	}

}
