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
#define DELAY 1000    // The amount of milliseconds to delay at the end of the loop
#define TIMEOUT 500  // The amount of time in milliseconds to wait for a reply before timing out

// Status register shift amounts
#define SEND_DATA 0       // Request that the sensor data be sent
#define TRANS_ERROR 1     // There was an error in the data being received
#define REPLY_ERROR 2        // The data was not received!

// Function prototypes
bool validChecksum(uint8_t[], int16_t&, int16_t&, int16_t&, uint16_t&, float&, float&, float&);

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

// Unions for breaking apart data into bytes
union LongSplicer {
	long data;
	byte bytes[4];
};

union FloatSplicer {
	float data;
	byte bytes[4];
};

union UInt16Splicer {
	uint16_t data;
	byte bytes[2];
};

union Int16Splicer {
	int16_t data;
	byte bytes[2];
};

// A nice set of enumerations to use as 'states'
enum Status {
	NORMAL,
	NO_REPLY,
	ERROR
};

// Used to hold the current state of the server
Status state = NORMAL;

void loop() {

	// Making sure there is at least X second(s) between each interval
	delay(DELAY);

	// Start by requesting data from the weather station
	uint8_t transmit[1];

	// Build the byte to send
	transmit[0] = 0x00;
	bitSet(transmit[0], SEND_DATA);

	// Set the appropriate bits depending on the state
	if (state == ERROR) {
		bitSet(transmit[0], TRANS_ERROR);
	}
	else if (state == NO_REPLY) {
		bitSet(transmit[0], REPLY_ERROR);
	}

	// Attemp to send the data to the station
	radio.send(transmit, sizeof(transmit));
	radio.waitPacketSent();

	// Wait for a reply
	if (radio.waitAvailableTimeout(TIMEOUT)) {

		// These variables will be used to hold data for sending and receiving
		uint8_t receive[RH_RF69_MAX_MESSAGE_LEN];
		uint8_t rSize = sizeof(receive);

		// Attempt to receive the data
		if (radio.recv(receive, &rSize)) {

			if (!rSize) {
				Serial.println("Transmission complete, but no data was sent!");
				return;
			}

			// Make sure the state is updated to normal
			state = NORMAL;

			// TODO: Handle reply here
			/*
			* We have decided to use a checksum to make sure the data was not corrupted,
			* so that will have to be up to the station to generate the checksum, and the
			* server to check it.
			*/

			// Convert all the received data into usable numbers
			// Temperature
			UInt16Splicer temperature;
			temperature.bytes[0] = receive[1];
			temperature.bytes[1] = receive[2];

			// X, Y, Z Orientation  
			Int16Splicer x, y, z;
			x.bytes[0] = receive[3];
			x.bytes[1] = receive[4];
			y.bytes[0] = receive[5];
			y.bytes[1] = receive[6];
			z.bytes[0] = receive[7];
			z.bytes[1] = receive[8];

			// X, Y, Z Acceleration
			FloatSplicer g_x, g_y, g_z;
			g_x.bytes[0] = receive[9];
			g_x.bytes[1] = receive[10];
			g_x.bytes[2] = receive[11];
			g_x.bytes[3] = receive[12];
			g_y.bytes[0] = receive[13];
			g_y.bytes[1] = receive[14];
			g_y.bytes[2] = receive[15];
			g_y.bytes[3] = receive[16];
			g_z.bytes[0] = receive[17];
			g_z.bytes[1] = receive[18];
			g_z.bytes[2] = receive[19];
			g_z.bytes[3] = receive[20];

			// Make sure the checksum cleared
			if (!validChecksum(receive, temperature.data, x.data, y.data, z.data, g_x.data, g_y.data, g_z.data)) {
				Serial.println("Checksum did not validate! Retrying later");
				state = ERROR;
				return;
			}

			// If all is good, then print the data
			Serial.print("Temperature: ");
			Serial.println(temperature.data, DEC);

			Serial.print("X: ");
			Serial.print(x.data);
			Serial.print(" Y: ");
			Serial.print(y.data);
			Serial.print(" Z: ");
			Serial.println(y.data);

			Serial.print("RSSI: ");
			Serial.println(radio.lastRssi());

		}
		else {

			// Make sure the state is updated to error
			state = ERROR;

			// There was an error receiving data
			Serial.println("There was an error in parsing received data! Retrying later");

		}

	}
	else {
		// No reply, so set the state here
		state = NO_REPLY;

		// No reply, so just try again later
		Serial.println("No reply from the station. Trying again later...");
	}

}

/*
* This function does what its name implies, it uses
* the provided checksum to make sure the data is valid.
*
* !! We are only expecting temperature data right now !!
*/
bool validChecksum(uint8_t data[], uint16_t &temperature, int16_t &x, int16_t &y, int16_t &z, float &g_x, float &g_y, float &g_z) {

	// Sum everything together for checking purposes :)
	long sum = temperature + x + y + z + static_cast<int>(g_x) + static_cast<int>(g_y) + static_cast<int>(g_z);

	LongSplicer checksum;
	checksum.data = sum;

	// Now compare the two
	return (checksum.bytes[0] == data[21]);

}