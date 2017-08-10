#include <SPI.h>
#include <RH_RF69.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

// Frequency in MGhz
#define FREQ 433.0

// Pin configuration
#define CS 2
#define G0 3
#define RS 8

// Sensor pins
#define TEMP 14

// Radio object
RH_RF69 radio(CS, G0);

// Accelerometer object
Adafruit_MMA8451 acc = Adafruit_MMA8451();

// etc. Configurations
#define TIMEOUT 500  // The amount of time in milliseconds to wait for a reply before timing out
int TRANS_POWER = 14; // The amount trans power should start out with (changes if the server signals NO_REPLY)

					  // Status register shift amounts
#define SEND_DATA 0       // Request that the sensor data be sent
#define TRANS_ERROR 1     // There was an error in the data being received
#define REPLY_ERROR 2        // The data was not received!

					  // Function prototypes
uint16_t getTemperatureRead();
byte calcChecksum(uint16_t&, int16_t&, int16_t&, int16_t&, float&, float&, float&);
void spliceData(uint16_t&, int16_t&, int16_t&, int16_t&, float&, float&, float&, long&, uint8_t[]);

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
	radio.setTxPower(TRANS_POWER, true);

	// Create an encryption key. This must be the same as the Rx transciever
	uint8_t key[] = { 0x05, 0x03, 0x01, 0x05, 0x06, 0x08, 0x02, 0x03,
		0x08, 0x02, 0x01, 0x08, 0x07, 0x03, 0x04, 0x02 };

	// Set the key
	radio.setEncryptionKey(key);

	Serial.println("The radio has been setup, and is ready to go");

	// Test to make sure the accelerometer is ready to go
	if (!acc.begin()) {
		Serial.println("Unable to start the accelerometer");
		for (;;) {}
	}

	// Set the range of the accelerometer
	acc.setRange(MMA8451_RANGE_2_G);

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

void loop() {

	/*
	* The statoin's job is just to wait for commands from the server, so there
	* will be no delays in this loop (except for data transmission delays)
	*/

	if (radio.available()) {

		Serial.println("Data detected, attempting to parse");

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

				// Get the temperature
				uint16_t temperature = getTemperatureRead();

				// Get the accelerometer data
				acc.read();
				int16_t x = acc.x, y = acc.y, z = acc.z;

				sensors_event_t accEvent;
				acc.getEvent(&accEvent);

				float g_x = accEvent.acceleration.x, g_y = accEvent.acceleration.y, g_z = accEvent.acceleration.z;

				// Build the data to send over
				uint8_t transmit[22];

				// No need to send any commands over the sreg, so just send the data
				// Create the checksum
				byte checkSum = calcChecksum(temperature, x, y, z, g_x, g_y, g_z);

				// Build the data to send
				spliceData(temperature, x, y, z, g_x, g_y, g_z, checkSum, transmit);

				// Transmit the data over
				radio.send(transmit, sizeof(transmit));
				radio.waitPacketSent();

			}

		}
		else {
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
uint16_t getTemperatureRead() {

	return analogRead(TEMP);

}

/*
* Used to calculate the checksum for data checking (because
* there is a possibility of error)
*/
byte calcChecksum(uint16_t &temperature, int16_t &x, int16_t &y, int16_t &z, float &g_x, float &g_y, float &g_z) {

	// Removing the decimal on the float.. Since the checksum is a long
	long sum = temperature + x + y + z + static_cast<int>(g_x) + static_cast<int>(g_y) + static_cast<int>(g_z);

	LongSplicer sumSplicer;
	sumSplicer.data = sum;

	return sumSplicer.bytes[0];

}

/*
* Breaks all the data given into the provided byte array, to be sent
* over packet radio
*/
void spliceData(uint16_t &temp, int16_t &x, int16_t &y, int16_t &z, float &g_x, float &g_y, float &g_z, byte &checksum, uint8_t data[]) {

	// Breakdown the temperature reading into bytes
	UInt16Splicer tempSplicer;
	tempSplicer.data = temp;

	// Breakdown all the axis' into bytes
	Int16Splicer xS, yS, zS;
	xS.data = x; yS.data = y; zS.data = z;

	// Breakdown all the g's into bytes
	FloatSplicer xgS, ygS, zgS;
	xgS.data = g_x; ygS.data = g_y; zgS.data = g_z;

	// Fillout the data to send, then cleanup

	// Temp bytes
	data[1] = tempSplicer.bytes[0];
	data[2] = tempSplicer.bytes[1];

	// X, Y, Z orientation bytes
	data[3] = xS.bytes[0];
	data[4] = xS.bytes[1];
	data[5] = yS.bytes[0];
	data[6] = yS.bytes[1];
	data[7] = zS.bytes[0];
	data[8] = zS.bytes[1];

	// X, Y, Z acceleration bytes
	data[9] = xgS.bytes[0];
	data[10] = xgS.bytes[1];
	data[11] = xgS.bytes[2];
	data[12] = xgS.bytes[3];
	data[13] = ygS.bytes[0];
	data[14] = ygS.bytes[1];
	data[15] = ygS.bytes[2];
	data[16] = ygS.bytes[3];
	data[17] = zgS.bytes[0];
	data[18] = zgS.bytes[1];
	data[19] = zgS.bytes[2];
	data[20] = zgS.bytes[3];

	// Add the checksum
	data[21] = checksum;

}
