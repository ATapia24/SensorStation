#include <SPI.h>
#include <RH_RF69.h>
#include <Ethernet.h>

// Frequency in MGhz
#define FREQ 433.0

// Pin configuration
#define CS 4
#define G0 3
#define RS 2

// Radio object
RH_RF69 radio(CS, G0);

// Ethernet object
EthernetClient client;

// etc. Configurations
#define RESEND 5      // The amount of times to resend the data
#define RETRY 5       // The amount of times to retry getting a reply before giving up
#define DELAY 5000    // The amount of milliseconds to delay at the end of the loop
#define TIMEOUT 500   // The amount of time in milliseconds to wait for a reply before timing out

// Status register shift amounts
#define SEND_DATA 0       // Request that the sensor data be sent
#define TRANS_ERROR 1     // There was an error in the data being received
#define REPLY_ERROR 2     // The data was not received!

// Ethernet configurations
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };    // Mac address
char domain[] = "";										// Domain to connect to
IPAddress address(192, 168, 1, 187);                    // set an IP to use just in case DHCP doens't work
String key = "";										// Authentication Key

// Function prototypes
bool validChecksum(uint8_t[], int16_t&, int16_t&, int16_t&, int16_t&, float&, float&, float&);
void parseData(uint8_t[], int16_t&, int16_t&, int16_t&, int16_t&, float&, float&, float&);
void postData(float&, int16_t&, int16_t&, int16_t&, float&, float&, float&, int16_t&);

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


	// Make sure the ethernet bridge is working
	if (Ethernet.begin(mac) == 0) {
		// Failed to obtain IP automatically, so set static IP
		Serial.println("Unable to configure the ethernet bridge using DHCP, using static IP");
		Ethernet.begin(mac, address);

	}
	else {
		Serial.println("Ethernet shield is ready to go");
	}

	// Give the ethernet shield time to setup
	delay(1000);

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

			/*
			* We have decided to use a checksum to make sure the data was not corrupted,
			* so that will have to be up to the station to generate the checksum, and the
			* server to check it.
			*/

			// Create all the necessary variables to hold the data
			int16_t temperature, x, y, z;
			float g_x, g_y, g_z;

			// Parse all the received data
			parseData(receive, temperature, x, y, z, g_x, g_y, g_z);

			// Make sure the checksum cleared
			if (!validChecksum(receive, temperature, x, y, z, g_x, g_y, g_z)) {
				Serial.println("Checksum did not validate! Retrying later");
				state = ERROR;
				return;
			}

			// Convert the temperature to usuable data
			float celsius = (((temperature * 5) / 1024.0) - 0.5) / 0.01,
				farenheight = (celsius * 1.8) + 32;

			int16_t rssi = radio.lastRssi();

			// If all is good, then print the data
			Serial.print("Temperature: ");
			Serial.println(farenheight, DEC);

			Serial.print("X: ");
			Serial.print(x);
			Serial.print(" Y: ");
			Serial.print(y);
			Serial.print(" Z: ");
			Serial.println(z);

			Serial.print("RSSI: ");
			Serial.println(rssi);
			postData(farenheight, x, y, z, g_x, g_y, g_z, rssi);

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
bool validChecksum(uint8_t data[], int16_t &temperature, int16_t &x, int16_t &y, int16_t &z, float &g_x, float &g_y, float &g_z) {

	// Sum everything together for checking purposes :)
	long sum = temperature + x + y + z + static_cast<int>(g_x) + static_cast<int>(g_y) + static_cast<int>(g_z);

	LongSplicer checksum;
	checksum.data = sum;

	// Now compare the two
	return (checksum.bytes[0] == data[21]);

}

/*
* For this function, all you have to do is pass the received data byte-array and the variables
* you want to fill up with the data.
*/
void parseData(uint8_t data[], int16_t &temperature, int16_t &x, int16_t &y, int16_t &z, float &g_x, float &g_y, float &g_z) {

	// Convert all the received data into usable numbers
	// Temperature
	Int16Splicer temp;
	temp.bytes[0] = data[1];
	temp.bytes[1] = data[2];

	// X, Y, Z Orientation  
	Int16Splicer xS, yS, zS;
	xS.bytes[0] = data[3];
	xS.bytes[1] = data[4];
	yS.bytes[0] = data[5];
	yS.bytes[1] = data[6];
	zS.bytes[0] = data[7];
	zS.bytes[1] = data[8];

	// X, Y, Z Acceleration
	FloatSplicer g_xS, g_yS, g_zS;
	g_xS.bytes[0] = data[9];
	g_xS.bytes[1] = data[10];
	g_xS.bytes[2] = data[11];
	g_xS.bytes[3] = data[12];
	g_yS.bytes[0] = data[13];
	g_yS.bytes[1] = data[14];
	g_yS.bytes[2] = data[15];
	g_yS.bytes[3] = data[16];
	g_zS.bytes[0] = data[17];
	g_zS.bytes[1] = data[18];
	g_zS.bytes[2] = data[19];
	g_zS.bytes[3] = data[20];

	// Pass the data over
	temperature = temp.data;

	x = xS.data;
	y = yS.data;
	z = zS.data;

	g_x = g_xS.data;
	g_y = g_yS.data;
	g_z = g_zS.data;

}

/*
* This function's job is to take the data, and post it to the internet!
*/
void postData(float &temperature, int16_t &x, int16_t &y, int16_t &z, float &g_x, float &g_y, float &g_z, int16_t &rssi) {

	String data = "GET /weather_post.php?t=";
	data.concat(temperature);
	data.concat("&x=");
	data.concat(x);
	data.concat("&y=");
	data.concat(y);
	data.concat("&z=");
	data.concat(z);
	data.concat("&gx=");
	data.concat(g_x);
	data.concat("&gy=");
	data.concat(g_y);
	data.concat("&gz=");
	data.concat(g_z);
	data.concat("&rssi=");
	data.concat(rssi);
	data.concat("&key=");
	data.concat(key);
	data.concat(" HTTP/1.1");

	// Attempt to connect to the server
	if (client.connect(domain, 80)) {

		// Make the HTTP request
		client.println(data.c_str());
		client.println("Host: www.vvcrobotics.club");
		client.println("Connection: close");
		client.println();

		delay(100);

		// read any bytes incoming and just dump them to the screen
		char b;
		while (client.available()) {
			b = client.read();
			Serial.print(b);
		}

		Serial.println("Web transaction complete");

		// Stop the client
		client.stop();

	}
	else {
		Serial.println("Unable to connect to the webserver");
	}

}