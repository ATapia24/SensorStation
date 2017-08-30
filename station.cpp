#include <SPI.h>
#include <RH_RF69.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TCS34725.h"
#include "Adafruit_Si7021.h"

// Frequency in MGhz
#define FREQ 433.0

// Pin configuration
#define CS 2
#define G0 3
#define RS 8

// Sensor pins
#define TEMP 14
#define UV_PIN 15
#define GAS_PIN 16

// Radio object
RH_RF69 radio(CS, G0);

// Magnetometer object
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1);

// Accelerometer object
Adafruit_MMA8451 acc = Adafruit_MMA8451();

// Pressure sensor object
Adafruit_BMP280 bmp;

// RGB sensor object
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

// Humidity sensor object
Adafruit_Si7021 hum;

// etc. Configurations
#define TIMEOUT 500  // The amount of time in milliseconds to wait for a reply before timing out
int TRANS_POWER = 14; // The amount trans power should start out with (changes if the server signals NO_REPLY)

					  // Status register shift amounts
#define SEND_DATA 0       // Request that the sensor data be sent
#define TRANS_ERROR 1     // There was an error in the data being received
#define REPLY_ERROR 2        // The data was not received!

					  // Function prototypes
uint16_t getTemperatureRead();
byte calcChecksum(int16_t&, int16_t&, int16_t&, int16_t&, float&, float&, float&, float&, uint16_t&, uint16_t&,
	uint16_t&, uint16_t&, uint16_t&, float&, float&, float&, float&, float&, int16_t&, int16_t&);
void spliceData(int16_t&, int16_t&, int16_t&, int16_t&, float&, float&, float&, float&, uint16_t&, uint16_t&, uint16_t&, uint16_t&, uint16_t&, float&, float&, float&,
	float&, float&, int16_t&, int16_t&, byte&, uint8_t[]);

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

	// Check to see if the BMP280 sensor is ready to go
	if (!bmp.begin()) {
		Serial.println("Unable to find BMP280 sensor");
		for (;;) {}
	}

	// Make sure the TCS34725 sensor is working
	if (!tcs.begin()) {
		Serial.println("Unable to find TCS34725 sensor");
		for (;;) {}
	}

	// Make sure the Si7021 sensor is working
	if (!hum.begin()) {
		Serial.println("Unable to the find the Si7021 sensor");
		for (;;) {}
	}

	// Make sure the HMC5883L is working
	if (!mag.begin()) {
		Serial.println("Unbale to find the HMC5883L sensor");
		for (;;) {}
	}

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
				int16_t temperature = getTemperatureRead();

				// Get the accelerometer data
				acc.read();
				int16_t x = acc.x, y = acc.y, z = acc.z;

				// Get the compass data
				float cx, cy, cz, ch;

				sensors_event_t compassEvent;
				mag.getEvent(&compassEvent);

				cx = compassEvent.magnetic.x; cy = compassEvent.magnetic.y; cz = compassEvent.magnetic.z;
				ch = atan2(cy, cx);

				// Get the RGB data
				uint16_t rgb_g, rgb_b, rgb_r, rgb_lux, rgb_intensity, rgb_c;

				tcs.getRawData(&rgb_r, &rgb_g, &rgb_b, &rgb_c);
				rgb_lux = tcs.calculateLux(rgb_r, rgb_g, rgb_b);
				rgb_intensity = tcs.calculateColorTemperature(rgb_r, rgb_g, rgb_b);

				// Get the humidity data
				float humidity, humidity_temp;

				humidity = hum.readHumidity();
				humidity_temp = hum.readTemperature();

				// Get the pressure data
				float pressure, pressure_altitude, pressure_temp;

				pressure = bmp.readPressure();
				pressure_altitude = bmp.readAltitude(1013.25);
				pressure_temp = bmp.readTemperature();

				// Get the etc analog data
				int16_t uv, gas;

				uv = analogRead(UV_PIN); gas = analogRead(GAS_PIN);

				// Build the data to send over
				uint8_t transmit[60];

				// No need to send any commands over the sreg, so just send the data
				// Create the checksum
				byte checkSum = calcChecksum(temperature, x, y, z, cx, cy, cz, ch, rgb_r, rgb_b, rgb_g, rgb_lux, rgb_intensity, humidity, humidity_temp, pressure, pressure_altitude,
					pressure_temp, uv, gas);

				// Build the data to send
				spliceData(temperature, x, y, z, cx, cy, cz, ch, rgb_r, rgb_b, rgb_g, rgb_lux, rgb_intensity, humidity, humidity_temp, pressure, pressure_altitude, pressure_temp,
					uv, gas, checkSum, transmit);

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
byte calcChecksum(int16_t &temperature, int16_t &x, int16_t &y, int16_t &z, float &cx, float &cy, float &cz, float &ch, uint16_t &rgb_r, uint16_t &rgb_b,
	uint16_t &rgb_g, uint16_t &rgb_lux, uint16_t &rgb_int, float &humidity, float &humidity_temp, float &pres, float &altitude, float &press_temp, int16_t &uv, int16_t &gas) {

	// Removing the decimal on the float.. Since the checksum is a long
	long sum = temperature + x + y + z + static_cast<int>(cx) + static_cast<int>(cy) + static_cast<int>(cz)
		+ static_cast<int>(ch) + rgb_r + rgb_b + rgb_g + rgb_lux + rgb_int + static_cast<int>(humidity) + static_cast<int>(humidity_temp) + static_cast<int>(pres) + static_cast<int>(altitude)
		+ static_cast<int>(press_temp) + uv + gas;

	LongSplicer sumSplicer;
	sumSplicer.data = sum;

	return sumSplicer.bytes[0];

}

/*
* Breaks all the data given into the provided byte array, to be sent
* over packet radio
*/
void spliceData(int16_t &temp, int16_t &x, int16_t &y, int16_t &z, float &cx, float &cy, float &cz, float &ch, uint16_t &rgb_r, uint16_t &rgb_b,
	uint16_t &rgb_g, uint16_t &rgb_lux, uint16_t &rgb_int, float &humidity, float &humidity_temp, float &pres, float &altitude, float &press_temp, int16_t &uv, int16_t &gas, byte &checksum, uint8_t data[]) {

	// Breakdown the temperature reading into bytes
	Int16Splicer tempSplicer;
	tempSplicer.data = temp;

	// Breakdown all the axis' into bytes
	Int16Splicer xS, yS, zS;
	xS.data = x; yS.data = y; zS.data = z;

	// Breakdown all the compas information into bytes
	FloatSplicer cxS, cyS, czS, heading;
	cxS.data = cx; cyS.data = cy; czS.data = cz; heading.data = ch;

	// Breakdown all the magnometer data into bytes
	UInt16Splicer rS, bS, gS, lux, intensity;
	rS.data = rgb_r; bS.data = rgb_b; gS.data = rgb_g;
	lux.data = rgb_lux; intensity.data = rgb_int;

	// Breakdown all the humidity data
	FloatSplicer humidityS, humidityTemp;
	humidityS.data = humidity; humidityTemp.data = humidity_temp;

	// Breakdown all the pressure data
	FloatSplicer pressure, altitudeS, pressureTemp;
	pressure.data = pres; altitudeS.data = altitude; pressureTemp.data = press_temp;

	// Analog conversion data
	Int16Splicer uvS, gasS;
	uvS.data = uv; gasS.data = gas;

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

	// Compass data bytes
	data[9] = cxS.bytes[0];
	data[10] = cxS.bytes[1];
	data[11] = cxS.bytes[2];
	data[12] = cxS.bytes[3];
	data[13] = cyS.bytes[0];
	data[14] = cyS.bytes[1];
	data[15] = cyS.bytes[2];
	data[16] = cyS.bytes[3];
	data[17] = czS.bytes[0];
	data[18] = czS.bytes[1];
	data[19] = czS.bytes[2];
	data[20] = czS.bytes[3];
	data[21] = heading.bytes[0];
	data[22] = heading.bytes[1];
	data[23] = heading.bytes[2];
	data[24] = heading.bytes[3];

	// RGB data bytes
	data[25] = rS.bytes[0];
	data[26] = rS.bytes[1];
	data[27] = bS.bytes[0];
	data[28] = bS.bytes[1];
	data[29] = gS.bytes[0];
	data[30] = gS.bytes[1];
	data[31] = lux.bytes[0];
	data[32] = lux.bytes[1];
	data[33] = intensity.bytes[0];
	data[34] = intensity.bytes[1];

	// Humidity bytes
	data[35] = humidityS.bytes[0];
	data[36] = humidityS.bytes[1];
	data[37] = humidityS.bytes[2];
	data[38] = humidityS.bytes[3];
	data[39] = humidityTemp.bytes[0];
	data[40] = humidityTemp.bytes[1];
	data[41] = humidityTemp.bytes[2];
	data[42] = humidityTemp.bytes[3];

	// Pressure bytes
	data[43] = pressure.bytes[0];
	data[44] = pressure.bytes[1];
	data[45] = pressure.bytes[2];
	data[46] = pressure.bytes[3];
	data[47] = altitudeS.bytes[0];
	data[48] = altitudeS.bytes[1];
	data[49] = altitudeS.bytes[2];
	data[50] = altitudeS.bytes[3];
	data[51] = pressureTemp.bytes[0];
	data[52] = pressureTemp.bytes[1];
	data[53] = pressureTemp.bytes[2];
	data[54] = pressureTemp.bytes[3];

	// Gas and UV bytes
	data[55] = uvS.bytes[0];
	data[56] = uvS.bytes[1];
	data[57] = gasS.bytes[0];
	data[58] = gasS.bytes[1];

	// Add the checksum101
	data[59] = checksum;

}
