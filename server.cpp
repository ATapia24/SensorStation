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
#define TIMEOUT 2000   // The amount of time in milliseconds to wait for a reply before timing out

// Status register shift amounts
#define SEND_DATA 0       // Request that the sensor data be sent
#define TRANS_ERROR 1     // There was an error in the data being received
#define REPLY_ERROR 2     // The data was not received!

// Ethernet configurations
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };    // Mac address
char domain[] = "";                 // Domain to connect to
IPAddress address(192, 168, 1, 187);                    // set an IP to use just in case DHCP doens't work
String key = "";

// Function prototypes
bool validChecksum(uint8_t[], int16_t&, int16_t&, int16_t&, int16_t&, float&, float&, float&, float&, uint16_t&, uint16_t&,
	uint16_t&, uint16_t&, uint16_t&, float&, float&, float&, float&, float&, int16_t&, int16_t&);
void parseData(uint8_t[], int16_t&, int16_t&, int16_t&, int16_t&, float&, float&, float&, float&, uint16_t&, uint16_t&, uint16_t&, uint16_t&,
	uint16_t&, float&, float&, float&, float&, float&, int16_t&, int16_t&);
void postData(float&, int16_t&, int16_t&, int16_t&, int16_t&);

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

union UInt16Splicer {
	uint16_t data;
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
			int16_t temperature, x, y, z, uv, gas;
			uint16_t rgb_r, rgb_b, rgb_g, rgb_lux, rgb_int;
			float cx, cy, cz, ch, humidity, humidity_temp, altitude, press_temp, pres;

			// Parse all the received data
			parseData(receive, temperature, x, y, z, cx, cy, cz, ch, rgb_r, rgb_b, rgb_g, rgb_lux, rgb_int, humidity, humidity_temp, pres, altitude, press_temp, uv, gas);

			// Make sure the checksum cleared
			if (!validChecksum(receive, temperature, x, y, z, cx, cy, cz, ch, rgb_r, rgb_b, rgb_g, rgb_lux, rgb_int, humidity, humidity_temp, pres, altitude, press_temp, uv, gas)) {
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

			// Post the data to the net
			postData(farenheight, x, y, z, cx, cy, cz, ch, rgb_r, rgb_b, rgb_g, rgb_lux, rgb_int, humidity, humidity_temp, pres, altitude, press_temp, uv, gas, rssi);

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
bool validChecksum(uint8_t data[], int16_t &temperature, int16_t &x, int16_t &y, int16_t &z, float &cx, float &cy, float &cz, float &ch, uint16_t &rgb_r, uint16_t &rgb_b,
	uint16_t &rgb_g, uint16_t &rgb_lux, uint16_t &rgb_int, float &humidity, float &humidity_temp, float &pres, float &altitude, float &press_temp, int16_t &uv, int16_t &gas) {

	// Sum everything together for checking purposes :)
	long sum = temperature + x + y + z + static_cast<int>(cx) + static_cast<int>(cy) + static_cast<int>(cz)
		+ static_cast<int>(ch) + rgb_r + rgb_b + rgb_g + rgb_lux + rgb_int + static_cast<int>(humidity) + static_cast<int>(humidity_temp) + static_cast<int>(pres) + static_cast<int>(altitude)
		+ static_cast<int>(press_temp) + uv + gas;

	LongSplicer checksum;
	checksum.data = sum;

	// Now compare the two
	return (checksum.bytes[0] == data[59]);

}

/*
* For this function, all you have to do is pass the received data byte-array and the variables
* you want to fill up with the data.
*/
void parseData(uint8_t data[], int16_t &temperature, int16_t &x, int16_t &y, int16_t &z, float &cx, float &cy, float &cz, float &ch, uint16_t &rgb_r, uint16_t &rgb_b,
	uint16_t &rgb_g, uint16_t &rgb_lux, uint16_t &rgb_int, float &humidity, float &humidity_temp, float &pres, float &altitude, float &press_temp, int16_t &uv, int16_t &gas) {

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

	// Compass data
	FloatSplicer cxS, cyS, czS, heading;
	cxS.bytes[0] = data[9];
	cxS.bytes[1] = data[10];
	cxS.bytes[2] = data[11];
	cxS.bytes[3] = data[12];
	cyS.bytes[0] = data[13];
	cyS.bytes[1] = data[14];
	cyS.bytes[2] = data[15];
	cyS.bytes[3] = data[16];
	czS.bytes[0] = data[17];
	czS.bytes[1] = data[18];
	czS.bytes[2] = data[19];
	czS.bytes[3] = data[20];
	heading.bytes[0] = data[21];
	heading.bytes[1] = data[22];
	heading.bytes[2] = data[23];
	heading.bytes[3] = data[24];

	// RGB data
	UInt16Splicer rS, bS, gS, lux, intensity;
	rS.bytes[0] = data[25];
	rS.bytes[1] = data[26];
	bS.bytes[0] = data[27];
	bS.bytes[1] = data[28];
	gS.bytes[0] = data[29];
	gS.bytes[1] = data[30];
	lux.bytes[0] = data[31];
	lux.bytes[1] = data[32];
	intensity.bytes[0] = data[33];
	intensity.bytes[1] = data[34];

	// Humidity data
	FloatSplicer humidityS, humidityTemp;
	humidityS.bytes[0] = data[35];
	humidityS.bytes[1] = data[36];
	humidityS.bytes[2] = data[37];
	humidityS.bytes[3] = data[38];
	humidityTemp.bytes[0] = data[39];
	humidityTemp.bytes[1] = data[40];
	humidityTemp.bytes[2] = data[41];
	humidityTemp.bytes[3] = data[42];

	// Pressure data
	FloatSplicer pressure, altitudeS, pressureTemp;
	pressure.bytes[0] = data[43];
	pressure.bytes[1] = data[44];
	pressure.bytes[2] = data[45];
	pressure.bytes[3] = data[46];
	altitudeS.bytes[0] = data[47];
	altitudeS.bytes[1] = data[48];
	altitudeS.bytes[2] = data[49];
	altitudeS.bytes[3] = data[50];
	pressureTemp.bytes[0] = data[51];
	pressureTemp.bytes[1] = data[52];
	pressureTemp.bytes[2] = data[53];
	pressureTemp.bytes[3] = data[54];

	// Etc analog data
	Int16Splicer uvS, gasS;
	uvS.bytes[0] = data[55];
	uvS.bytes[1] = data[56];
	gasS.bytes[0] = data[57];
	gasS.bytes[1] = data[58];

	// Pass the data over
	temperature = temp.data; // Temperature

	x = xS.data; // Orientation
	y = yS.data;
	z = zS.data;

	cx = cxS.data; // Compass and heading
	cy = cyS.data;
	cz = czS.data;
	ch = heading.data;

	rgb_r = rS.data; // RGB and intensity
	rgb_b = bS.data;
	rgb_g = gS.data;
	rgb_lux = lux.data;
	rgb_int = intensity.data;

	humidity = humidityS.data; // Humidity and its temp
	humidity_temp = humidityTemp.data;

	pres = pressure.data; // Pressure, altitude, and its temp
	altitude = altitudeS.data;
	press_temp = pressureTemp.data;

	uv = uvS.data; // Gas and uv sensor data
	gas = gasS.data;

}

/*
* This function's job is to take the data, and post it to the internet!
*/
void postData(float &temperature, int16_t &x, int16_t &y, int16_t &z, float &cx, float &cy, float &cz, float &ch, uint16_t &rgb_r, uint16_t &rgb_b,
	uint16_t &rgb_g, uint16_t &rgb_lux, uint16_t &rgb_int, float &humidity, float &humidity_temp, float &pres, float &altitude, float &press_temp, int16_t &uv, int16_t &gas, int16_t &rssi) {

	String data = "GET /weather_post.php?t=";
	data.concat(temperature);
	data.concat("&x=");
	data.concat(x);
	data.concat("&y=");
	data.concat(y);
	data.concat("&z=");
	data.concat(z);
	data.concat("&rssi=");
	data.concat(rssi);
	data.concat("&cx=");
	data.concat(cx);
	data.concat("&cy=");
	data.concat(cy);
	data.concat("&cz=");
	data.concat(cz);
	data.concat("&ch=");
	data.concat(ch);
	data.concat("&r=");
	data.concat(rgb_r);
	data.concat("&g=");
	data.concat(rgb_g);
	data.concat("&b=");
	data.concat(rgb_b);
	data.concat("&lux=");
	data.concat(rgb_lux);
	data.concat("&hum=");
	data.concat(humidity);
	data.concat("&pres=");
	data.concat(pres);
	data.concat("&alt=");
	data.concat(altitude);
	data.concat("&press_temp=");
	data.concat(press_temp);
	data.concat("&uv=");
	data.concat(uv);
	data.concat("&gas=");
	data.concat(gas);
	data.concat("&key=");
	data.concat(key);

	Serial.println(data);

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