/*
 * Mert Arduino and Tech - YouTube
 * WiFi Robot controlled by Phone (WiFibot/Android/IoT/ESP8266)
 * NodeMCU ESP8266 Tutorial 03
 * Please Subscribe for Support - https://goo.gl/RLT6DT
 */


/* include library */
#include <ESP8266WiFi.h>

/*
 * If you have no idea about this library, you can watch the tutorial below
 * NodeMCU ESP8266 Tutorial 01: Programming NodeMCU ESP-12E Using Arduino IDE
 * https://youtu.be/NtyFx1frYB0
 */

/* define port */
WiFiClient client;
WiFiServer server(80);

/* WIFI settings */
const char* ssid = "INVTech 2.4G";
const char* password = "INVTECH#2019";

/* data received from application */
String  data = "";


/* define L298N or L293D enable pins */
int motors_pin = 14; /* GPIO14(D5) -> Motor-A Enable */
int lights_pin = 12;  /* GPIO12(D6) -> Motor-B Enable */

void setup()
{

  /* initialize motor enable pins as output */
  pinMode(motors_pin, OUTPUT);
  pinMode(lights_pin, OUTPUT);

  /* start server communication */
  server.begin();
}

void loop()
{
    /* If the server available, run the "checkClient" function */
    client = server.available();
    if (!client) return;
    data = checkClient ();

/************************ Run function according to incoming data from application *************************/

    /* If the incoming data is "forward", run the "MotorForward" function */
    if (data == "open") OpenHangar();
    /* If the incoming data is "backward", run the "MotorBackward" function */
    else if (data == "close") MotorBackward();
    /* If the incoming data is "left", run the "TurnLeft" function */
    else if (data == "lights_on") LightsOn();
    /* If the incoming data is "right", run the "TurnRight" function */
    else if (data == "lights_off") TurnRight();
}

/********************************************* OPEN HANGAR *****************************************************/
void OpenHangar(void)
{
  digitalWrite(motors_pin, HIGH);
}
/********************************************* CLOSE HANGAR *****************************************************/
void CloseHangar(void)
{
  digitalWrite(motors_pin, LOW);
}
/********************************************* LIGHTS ON *****************************************************/
void LightsOn(void)
{
  digitalWrite(lights_pin, HIGH);
}
/********************************************* LIGHTS OFF *****************************************************/
void LightsOn(void)
{
  digitalWrite(lights_pin, LOW);
}

/********************************** RECEIVE DATA FROM the APP ******************************************/
String checkClient (void)
{
  while(!client.available()) delay(1);
  String request = client.readStringUntil('\r');
  request.remove(0, 5);
  request.remove(request.length()-9,9);
  return request;
}
