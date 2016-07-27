/*
  MKR1000 WiFi RTC

  This sketch asks NTP for the Linux epoch and sets the internal Arduino MKR1000's RTC accordingly.

  created 08 Jan 2016
  by Arturo Guadalupi <a.guadalupi@arduino.cc>

  http://arduino.cc/en/Tutorial/WiFiRTC
  This code is in the public domain.
*/

#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <RTCZero.h>
const int buttonPin = 6;    // button input
const int motor1Pin = 3;    // H-bridge leg 1 (pin 2, 1A)
const int motor2Pin = 4;    // H-bridge leg 2 (pin 7, 2A)
const int conLedRed = 7;
const int ntpLedBlue = 8;
bool change = true;
unsigned long epoch;
unsigned long epochtest;
bool connected = false;
bool ntpsuccess = false;
int adder = 1;
int adderResetTime = 0;
int y = 0;
int dow = 0;
int mo = 0;
int d = 0;
int h = 0;
int m = 0;
int s = 0;
bool DST = true; // needs to be set, true if summer
int lastAddedMinutes = 0;

RTCZero rtc;

char ssid[] = "herotero";     //  your network SSID (name)
char pass[] = "innostunutsonni";   // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

WiFiServer server(80);

// Used for NTP
unsigned int localPort = 2390;      // local port to listen for UDP packets
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServer(194, 100, 49, 151);

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP

const int GMT = 3; //change this to adapt it to your time zone

void setup() {

	// set the button as an input:
	pinMode(buttonPin, INPUT);
	// set all the other pins you're using as outputs:
	pinMode(motor1Pin, OUTPUT);
	pinMode(motor2Pin, OUTPUT);
	pinMode(conLedRed, OUTPUT);
	pinMode(ntpLedBlue, OUTPUT);
	digitalWrite(conLedRed, LOW);
	digitalWrite(ntpLedBlue, LOW);

	Serial.begin(115200);

	// check if the WiFi module works
	if (WiFi.status() == WL_NO_SHIELD) {
		Serial.println("WiFi shield not present");
		// don't continue:
		while (true);
	}

	// attempt to connect to WiFi network:
	while (status != WL_CONNECTED) {
		Serial.print("Attempting to connect to SSID: ");
		Serial.println(ssid);
		// Connect to WPA/WPA2 network. Change this line if using open or WEP network:
		status = WiFi.begin(ssid, pass);
		// wait 10 seconds for connection:
		delay(10000);
	}

	server.begin();
	// you're connected now, so print out the status:
	printWifiStatus();
	digitalWrite(conLedRed, HIGH);
	connected = true;
	rtc.begin();

	int numberOfTries = 0, maxTries = 6;
	do {
		epoch = readLinuxEpochUsingNTP();
		numberOfTries++;
	} while ((epoch == 0) || (numberOfTries > maxTries));

	if (numberOfTries > maxTries) {
		Serial.print("NTP unreachable!!");
		while (1);
		ntpsuccess = false;
	}
	else {
		digitalWrite(ntpLedBlue, HIGH);
		Serial.print("Epoch received: ");
		Serial.println(epoch);
		rtc.setEpoch(epoch);
		ntpsuccess = true;
		Serial.println();
	}
}

void loop() {

	y = rtc.getYear();
	mo = rtc.getMonth();
	d = rtc.getDay();
	h = rtc.getHours() + GMT;
	m = rtc.getMinutes();
	s = rtc.getSeconds();

	// printDate();
	// printTime();
	// Serial.println();
	// delay(1000);
	// listen for incoming clients
	WiFiClient client = server.available();
	if (client) {
		Serial.println("new client");
		// an http request ends with a blank line
		boolean currentLineIsBlank = true;
		while (client.connected()) {
			if (client.available()) {
				char c = client.read();
				Serial.write(c);
				// if you've gotten to the end of the line (received a newline
				// character) and the line is blank, the http request has ended,
				// so you can send a reply
				if (c == '\n' && currentLineIsBlank) {
					// send a standard http response header
					client.println("HTTP/1.1 200 OK");
					client.println("Content-Type: text/html");
					client.println("Connection: close");  // the connection will be closed after completion of the response
					client.println("Refresh: 5");  // refresh the page automatically every 5 sec
					client.println();
					client.println("<!DOCTYPE HTML>");
					client.println("<html>");
					// output the value of each analog input pin
					client.print("Time ");
					client.print(h);
					client.print(":");
					client.print(m);
					client.print(":");
					client.print(s);
					client.println("<br>");
					client.print("Date ");
					client.print(d);
					client.print("/");
					client.print(mo);
					client.print("/");
					client.print(y);
					client.println("</html>");
					client.println("<br>");
					client.print("Last connection succesfull: ");
					client.print(connected);
					client.println("<br>");
					client.print("Last ntp connection succesfull: ");
					client.print(ntpsuccess);
					client.println("<br>");
					long rssi = WiFi.RSSI();
					client.print("signal strength (RSSI):");
					client.print(rssi);
					client.print(" dBm");
					client.println("<br>");
					client.print("Minutes added:");
					client.print(lastAddedMinutes);
					client.print(" min");
					client.println("</html>");
					break;
				}
				if (c == '\n') {
					// you're starting a new line
					currentLineIsBlank = true;
				}
				else if (c != '\r') {
					// you've gotten a character on the current line
					currentLineIsBlank = false;
				}
			}
		}
		// give the web browser time to receive the data
		delay(1);

		// close the connection:
		client.stop();
		Serial.println("client disonnected");
	}

	// Adjust time every minute
	if (s == 0)
	{
		Serial.print("Seconds: ");
		Serial.println(0);
		move(change);
		change = !change;
		if (adder == 1)
		{
			adderResetTime = m;
		}
		adder++;
	}

	// if the button is high, adjust time constantly:
	if (digitalRead(buttonPin) == HIGH) {
		move(change);
		change = !change;
		adder = 1;
	}

	// check every hour if for some reason clock is left behind
	if ((adderResetTime - m) == 0 && s == 0 && adder > 5)
	{
		int addminutes = 60 - adder;
		lastAddedMinutes = addminutes;
		Serial.print("Fixing seconds: ");
		Serial.println(addminutes);
		while (addminutes > 0)
		{
			move(change);
			change = !change;
			addminutes--;
		}
		adder = 1;
	}

	// Daylight savings

	//winter time
	if (dow == 7 && mo == 10 && d >= 25 && d <= 31 && h == 3 && m == 0 && s == 0 && DST == true)
	{
		//	wait 1h;
		delay(36000000);
		DST = false;
	}

	//summer time
	if (dow == 7 && mo == 3 && d >= 25 && d <= 31 && h == 2 && m == 0 && s == 0 && DST == false)
	{
		int i = 0;
		//	forward 1h;
		while (i < 60)
		{
			move(change);
			change = !change;
			i++;
		}
		DST = true;
	}

	// check sync rtc with ntp every hour
	if ((m == 30) && (s == 10))
	{
		// attempt to connect to WiFi network:
		if (WiFi.status() != 3) {
			Serial.print("Attempting to connect to SSID: ");
			Serial.println(ssid);
			// Connect to WPA/WPA2 network. Change this line if using open or WEP network:
			status = WiFi.begin(ssid, pass);
			// wait 10 seconds for connection:
			delay(10000);
			digitalWrite(conLedRed, HIGH);
		}

		if (WiFi.status() != 3)
		{
			Serial.println("Could not connect");
			digitalWrite(conLedRed, LOW);
			connected = false;
		}
		else
		{
			epochtest = readLinuxEpochUsingNTP();
			connected = true;
		}
		if (epochtest == 0)
		{
			Serial.println("Could not connect to timeserver");
			digitalWrite(ntpLedBlue, LOW);
			ntpsuccess = false;
		}
		else
		{
			digitalWrite(ntpLedBlue, HIGH);
			ntpsuccess = true;
			Serial.print("Epoch received: ");
			Serial.println(epochtest);
			rtc.setEpoch(epochtest);
			Serial.println();
			epochtest = 0;
		}
	}
}

void move(bool change)
{
	if (change == true)
	{
		digitalWrite(motor1Pin, LOW);   // set leg 1 of the H-bridge low
		digitalWrite(motor2Pin, HIGH);  // set leg 2 of the H-bridge high
		delay(1000);
	}
	else
	{
		digitalWrite(motor1Pin, HIGH);  // set leg 1 of the H-bridge high
		digitalWrite(motor2Pin, LOW);   // set leg 2 of the H-bridge low
		delay(1000);
	}
}

unsigned long readLinuxEpochUsingNTP()
{
	Udp.begin(localPort);
	sendNTPpacket(timeServer); // send an NTP packet to a time server
	// wait to see if a reply is available
	delay(1000);

	if (Udp.parsePacket()) {
		Serial.println("NTP time received");
		// We've received a packet, read the data from it
		Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

		//the timestamp starts at byte 40 of the received packet and is four bytes,
		// or two words, long. First, esxtract the two words:

		unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
		unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
		// combine the four bytes (two words) into a long integer
		// this is NTP time (seconds since Jan 1 1900):
		unsigned long secsSince1900 = highWord << 16 | lowWord;

		// now convert NTP time into everyday time:
		// Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
		const unsigned long seventyYears = 2208988800UL;
		// subtract seventy years:

		Udp.stop();
		return (secsSince1900 - seventyYears);
	}

	else {
		Udp.stop();
		return 0;
	}
}

void printTime()
{
	print2digits(rtc.getHours() + GMT);
	Serial.print(":");
	print2digits(rtc.getMinutes());
	Serial.print(":");
	print2digits(rtc.getSeconds());
	Serial.println();
}

void printDate()
{
	Serial.print(rtc.getDay());
	Serial.print("/");
	Serial.print(rtc.getMonth());
	Serial.print("/");
	Serial.print(rtc.getYear());

	Serial.print(" ");
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress & address)
{
	// set all bytes in the buffer to 0
	memset(packetBuffer, 0, NTP_PACKET_SIZE);
	// Initialize values needed to form NTP request
	// (see URL above for details on the packets)

	packetBuffer[0] = 0b11100011;   // LI, Version, Mode
	packetBuffer[1] = 0;     // Stratum, or type of clock
	packetBuffer[2] = 6;     // Polling Interval
	packetBuffer[3] = 0xEC;  // Peer Clock Precision
	// 8 bytes of zero for Root Delay & Root Dispersion
	packetBuffer[12] = 49;
	packetBuffer[13] = 0x4E;
	packetBuffer[14] = 49;
	packetBuffer[15] = 52;

	// all NTP fields have been given values, now
	// you can send a packet requesting a timestamp:
	Udp.beginPacket(address, 123); //NTP requests are to port 123
	Udp.write(packetBuffer, NTP_PACKET_SIZE);
	Udp.endPacket();
}

void printWifiStatus() {
	// print the SSID of the network you're attached to:
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	// print your WiFi shield's IP address:
	IPAddress ip = WiFi.localIP();
	Serial.print("IP Address: ");
	Serial.println(ip);

	// print the received signal strength:
	long rssi = WiFi.RSSI();
	Serial.print("signal strength (RSSI):");
	Serial.print(rssi);
	Serial.println(" dBm");
}

void print2digits(int number) {
	if (number < 10) {
		Serial.print("0");
	}
	Serial.print(number);
}
