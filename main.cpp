//*******************************************************//
//************** General CONFIGURATION ******************//
//*******************************************************//

#include <Arduino.h> 
#include <WiFi.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <PubSubClient.h>
#include <driver/adc.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

//*******************************************************//
//******************* GLOBAL VARIABLES ******************//
//*******************************************************//

int 	v_temp;
int 	room_occu;
float 	temp;
int		cal_temp;
int		cal_temp2;
float 	gain;

//*******************************************************//
//******************FUNCTION DECLARATION******************//
//*******************************************************//

void 	wifi_function();
int 	adc_function();
void 	occupancy_setup();
int 	occupancy_function();
void 	mqtt_setup();
void 	mqtt_connect();
int 	mqtt_publish(float temp, int room_occu);
void 	lcd_setup();
int 	lcd_function(float temp, int room_occu);
void 	callback(char* subscribe_topic, byte* payload, unsigned int length);
void 	reconnect();
void	mqtt_subscribe_setup();
float	temp_calibration_1(int cal_temp, int cal_temp2);


WiFiClient espClient;
PubSubClient client(espClient);

//*******************************************************//
//******************** MAIN CODE ************************//
//*******************************************************//

void setup(){

	Serial.begin(115200);

	wifi_function();
	mqtt_subscribe_setup();
	mqtt_setup();
	occupancy_setup();
	lcd_setup();

}


void loop(){

	mqtt_connect();	
	client.loop();				//establish mqtt subscribe using the callback funtion (OBTAINS cal_temp)
	
	gain = temp_calibration_1(cal_temp, cal_temp2);

	v_temp = adc_function();			//reads the voltage of the temperature sensor (OBTAINS v_temp)
  	
	temp = (-1)*(v_temp)/(gain);

	Serial.println(temp);

	room_occu = occupancy_function();

	mqtt_publish(temp, room_occu);

	lcd_function(temp, room_occu);
 
}


//*******************************************************//
//************** WIFI FUNCTION SETUP ********************//
//*******************************************************//

const char* ssid = "NOVA";
const char* password =  "INOVAZZION";

void wifi_function(){
	delay(10);
	// CONNECT TO WIFI 
	Serial.println();
	Serial.print("Connecting to ssid: ");
	Serial.println(ssid);

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.println("Connected to WiFi!");
	Serial.println("IP Address: ");
	Serial.println(WiFi.localIP());
}

//*******************************************************//
//************** TEMP SENSOR FUNCTION *******************//
//*******************************************************//

int adc_function (){


	adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_2_5);
	
	esp_adc_cal_characteristics_t adc1_chars;
	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 0, &adc1_chars);

	adc1_config_width(ADC_WIDTH_BIT_12);
	
	int raw_value = adc1_get_raw(ADC1_CHANNEL_6);

	// Serial.println(raw_value);

	delay(500);

	return raw_value;
}

//*******************************************************//
//***************** OCCUPANCY FUNCTION ******************//
//*******************************************************//

#define sound_speed 0.034

long duration1, duration2;
float dist_cm1, dist_cm2;
int occupancy;

const int trigger1 = 5;
const int echo1 = 18;
const int trigger2 = 25;
const int echo2 = 26;

void occupancy_setup(){

	pinMode(trigger1, OUTPUT);			//Ultrasonic sensor1 (Right)
	pinMode(echo1, INPUT);

	pinMode(trigger2, OUTPUT);			//Ultrasonic sensor2 (Left)
	pinMode(echo2, INPUT);	

}

int occupancy_function(){

	//************** Ultrasonic Sensor 1 **************//

	digitalWrite(trigger1, LOW);
	delayMicroseconds(2);

  	digitalWrite(trigger1, HIGH);
  	delayMicroseconds(10);
  	digitalWrite(trigger1, LOW);	

  	//Returns the sound wave travel time in microseconds
  	duration1 = pulseIn(echo1, HIGH);
  
  	// Calculate the distance
  	dist_cm1 = duration1 * sound_speed/2;
   
 	//Prints the distance in the Serial Monitor
  	// Serial.print("\n Distance1 (cm): ");
  	// Serial.println(dist_cm1);

	//************** Ultrasonic Sensor 2 **************//

	digitalWrite(trigger2, LOW);
	delayMicroseconds(2);

  	digitalWrite(trigger2, HIGH);
  	delayMicroseconds(10);
  	digitalWrite(trigger2, LOW);	

  	//Returns the sound wave travel time in microseconds
  	duration2 = pulseIn(echo2, HIGH);
  
  	// Calculate distance
  	dist_cm2 = duration2 * sound_speed/2;
   
 	//Print Distance
  	// Serial.print("\n Distance2 (cm): ");
  	// Serial.println(dist_cm2);

	//***************Occupancy calculation**************//

	int dist_cm1_state, dist_cm2_state;

	if (dist_cm1 <= 25){
		dist_cm1_state = 1;
	}
	if(dist_cm1 > 25){
		dist_cm1_state = 0;
	}
	if(dist_cm2 <= 25){
		dist_cm2_state = 1;
	}
	if(dist_cm2 > 25){
		dist_cm2_state = 0;
	}

	//transitions:
	// WAITING --> ENTERING: NOTHING
	// WAITING --> EXITING: NOTHING
	// ENTERING --> WAITING: ++
	// ENTERING --> EXITING: NOTHING
	// EXITING --> WAITING: --
	// EXITING --> ENTERING: NOTHING
	
	// Define the states
	// typedef enum {
  	// WAITING,
  	// ENTERING,
  	// EXITING,
	// } state_t;

	// state_t state = WAITING;	// Initialize the state machine to start in WAITING

  	//switch (state) {
    	// case WAITING:
      	// if (dist_cm1_state == 1 && dist_cm2_state == 0) {
        // 	state = ENTERING;
		// 	Serial.println("\n WAITTING --> ENTERING ");
		// 	Serial.println(state);
		// 	occupancy++;
      	// }
      	// else if (dist_cm1_state == 0 && dist_cm2_state == 1) {
        // 	state = EXITING;
		// 	Serial.println("\n WAITTING --> EXITING ");
		// 	Serial.println(state);
		// 	occupancy--;
      	// }				
      	// break;
    	// case ENTERING:
      	// if (dist_cm1_state == 0 && dist_cm2_state == 1) {
        // 	state = EXITING;
		// 	Serial.println("\n ENTERING --> EXITING ");
		// 	Serial.println(state);
      	// } 
      	// else if (dist_cm1_state == 0 && dist_cm2_state == 0) {
        // 	state = WAITING;
		// 	Serial.println("\n ENTERING --> WAITING ");
		// 	Serial.println(state);	
		// 	occupancy++;		
      	// }		
      	// break;
    	// case EXITING:
      	// if (dist_cm1_state == 0 && dist_cm2_state == 0) {
        // 	state = WAITING;
		// 	Serial.println("\n EXITING --> WAITING ");
		// 	Serial.println(state);
		// 	occupancy--;			
      	// }
      	// else if (dist_cm1_state == 1 && dist_cm2_state == 0) {
        // 	state = ENTERING;
		// 	Serial.println("\n EXITING --> ENTERING ");
		// 	Serial.println(state);			
      	// }		
      	// break;
  	//}
	

  	// delay(500);

	return occupancy;

}

//*******************************************************//
//*************** LCD DISPLAY FUNCTION*******************//
//*******************************************************//

LiquidCrystal_I2C lcd(0x3F, 16, 2);// Set the LCD I2C address

void lcd_setup(){

	lcd.init();
	lcd.backlight ();
	lcd.print ("Hello World!");

	delay(1000);

}

int lcd_function(float temp, int room_occu){

	lcd.setCursor (0,0);
	lcd.print("Temp. = ");
	lcd.print (temp);
	lcd.setCursor (0,1);
	lcd.print("Occup. = ");
	lcd.print(room_occu);
	delay(1000);

	return 0;
}

//*******************************************************//
//************ MQTT CONNECTION FUNCTIONS ****************//
//*******************************************************//

const char *mqtt_server = "18.210.250.130";
const int 	mqtt_port = 1883;
const char *mqtt_user = "INOVA_USER";
const char *mqtt_pass = "INOVA";
const char *root_topic_publish = "temp";
const char *root_topic_publish2 = "occupancy";

char msg[16];
char msg2[16];

void mqtt_setup(){

	client.setServer(mqtt_server, mqtt_port);

}

void mqtt_connect() {

	while (!client.connected()) {

		Serial.print("Trying MQTT Connection...");
		// Create Client ID
		String clientId = "INOVAZZION_H_W_";
		clientId += String(random(0xffff), HEX);
		// Try connection

		if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
			
			Serial.println("CONNECTED! =) ");
		    } 
		else {
			Serial.print("FAILED :( ERROR ->  ");
			Serial.print(client.state());
			delay(5000);
		}
	}
}

int mqtt_publish(float temp, int room_occu){

	snprintf(msg, 16, "%0.2f", temp);
    client.publish(root_topic_publish, msg);

	snprintf(msg2, 16, "%d", room_occu);
    client.publish(root_topic_publish2, msg2);	
    
    delay(500);

	return 0;
}


//*******************************************************//
//*************** MQTT SUBSCRIBE FUNCTION ***************//
//*******************************************************//

// MQTT broker information

const char* mqtt_username = "INOVA_USER";
const char* mqtt_password = "INOVA";

// MQTT topic to subscribe to
const char* subscribe_topic1 = "caltemp";
const char* subscribe_topic2 = "caltemp2";

void mqtt_subscribe_setup() {

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password )) {
      Serial.println("MQTT client connected.");
      client.subscribe(subscribe_topic1);
	  client.subscribe(subscribe_topic2);
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* subscribe_topic, byte* payload, unsigned int length) {

	byte 		buffer[80];
   	
   	int i=0;
     	for (i;i<length;i++) {
       	buffer[i]=payload[i];
       	Serial.print((char)payload[i]);
       	}
	buffer[i]='\0'; //end string

	Serial.println(subscribe_topic);

	if(strcmp(subscribe_topic,subscribe_topic1)==0){

		sscanf((char*)buffer,"{\"1\":%d}", &cal_temp);
		// Serial.println("El numero es:");
		// Serial.println(cal_temp);

	} else if(strcmp(subscribe_topic, subscribe_topic2)== 0){

		sscanf((char*)buffer,"{\"1\":%d}", &cal_temp2);
		// Serial.println("El numero es:");
		// Serial.println(cal_temp);	
	}

	// sscanf((char*)buffer,"{\"1\":%d}", &cal_temp);
	// Serial.println("El numero es:");
	// Serial.println(cal_temp);

}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Reconnecting to MQTT broker...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password )) {
      Serial.println("MQTT client connected.");
      client.subscribe(subscribe_topic1);
	  client.subscribe(subscribe_topic2);
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


//*******************************************************//
//************* TEMP CALIBRATION FUNCTIONs ***************//
//*******************************************************//

int task_executed1 = 0;
int task_executed2 = 0;
float 	v_temp1;
float 	v_temp2;
int		m;


float	temp_calibration_1(int cal_temp, int cal_temp2){
		
		if (!cal_temp) {
            cal_temp = 0;
            // Serial.println("First number received: ");
			// Serial.println(cal_temp);
        } else {
            Serial.println("Input 1: ");
			Serial.println(cal_temp);
			Serial.println("Voltage 1: ");
			
			if(!task_executed1){

				v_temp1 = adc_function();
				task_executed1 = 1;
			}
			
			Serial.println(v_temp1);
        }

	
		if (!cal_temp2) {
            cal_temp2 = 0;
            // Serial.println("First number received: ");
			// Serial.println(cal_temp);
        } else {
            Serial.println("Input 2: ");
			Serial.println(cal_temp2);
			Serial.println("Voltage 2 ");

			if(!task_executed2){

				v_temp2 = adc_function();
				task_executed2 = 1;
			}

			Serial.println(v_temp2);
			
        }		



		m = (v_temp2 - v_temp1)/(cal_temp2 - cal_temp);
	
		Serial.println("Gain:");
		Serial.println(m);

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	return m;
}












