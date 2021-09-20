// #include "OneLiner.h"
#include <Arduino_LSM6DS3.h>

#define WHOAMI "Rufous_FW"
#define CYCLE_TIME 10ul

float vel[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float bias[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float sensor[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

int throttle[4] = {0, 0, 0, 0};

unsigned long start;
unsigned long duration;
unsigned long birthTime;

bool isConnected = false;

char OP_MODE = '0';


char* stateSTR = "SampleString";

void serialEvent() {
	/*
		serialEvent is a serial callback.
		but it doesn't ucking work on IOT nano33!!
				(workaround: call in loop)
	*/
	OP_MODE = Serial.read();
	if( OP_MODE == 'u' && Serial.available() == 4){
		throttle[0] = Serial.read();
		throttle[1] = Serial.read();
		throttle[2] = Serial.read();
		throttle[3] = Serial.read();
	}
	if( OP_MODE == 'B' && Serial.available() > 6){
		bias[0] = Serial.parseFloat();
		bias[1] = Serial.parseFloat();
		bias[2] = Serial.parseFloat();
		bias[3] = Serial.parseFloat();
		bias[4] = Serial.parseFloat();
		bias[5] = Serial.parseFloat();
	}
	OP_MODE = 'Z';
}

unsigned long dyse_hash(){
	unsigned long n = 0;
	for(int i=0; i < sizeof(stateSTR); i+=3){
		n += stateSTR[i];
	}
	return n;
}

void updateState(){
	stateSTR = "";
	sprintf(stateSTR, 
		"%s:%lu:%lu:%lu:%f,%f,%f,%f,%f,%f:%f,%f,%f,%f,%f,%f:%f,%f,%f,%f,%f,%f",
		WHOAMI, start, duration, birthTime,
		vel[0], vel[1], vel[2], vel[3], vel[4], vel[5],
		pose[0], pose[1], pose[2], pose[3], pose[4], pose[5],
		sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5]);
}

void dumpState(){
	//updateState();
	Serial.print(WHOAMI); Serial.print(':');
	Serial.print(start); Serial.print(':');
	Serial.print(duration); Serial.print(':');
	Serial.print(birthTime); Serial.print(':');
	Serial.print(vel[0], 4); Serial.print(',');
	Serial.print(vel[1], 4); Serial.print(',');
	Serial.print(vel[2], 4); Serial.print(',');
	Serial.print(vel[3], 4); Serial.print(',');
	Serial.print(vel[4], 4); Serial.print(',');
	Serial.print(vel[5], 4); Serial.print(':');
	Serial.print(pose[0], 4); Serial.print(',');
	Serial.print(pose[1], 4); Serial.print(',');
	Serial.print(pose[2], 4); Serial.print(',');
	Serial.print(pose[3], 4); Serial.print(',');
	Serial.print(pose[4], 4); Serial.print(',');
	Serial.print(pose[5], 4); Serial.print(':');
	Serial.print(sensor[0], 4); Serial.print(',');
	Serial.print(sensor[1], 4); Serial.print(',');
	Serial.print(sensor[2], 4); Serial.print(',');
	Serial.print(sensor[3], 4); Serial.print(',');
	Serial.print(sensor[4], 4); Serial.print(',');
	Serial.print(sensor[5], 4); Serial.print(':');
	Serial.print(bias[0], 4); Serial.print(',');
	Serial.print(bias[1], 4); Serial.print(',');
	Serial.print(bias[2], 4); Serial.print(',');
	Serial.print(bias[3], 4); Serial.print(',');
	Serial.print(bias[4], 4); Serial.print(',');
	Serial.println(bias[5], 4);
}

void readIMU(float* sensor){
	if (IMU.accelerationAvailable())
		IMU.readAcceleration(sensor[0], sensor[1], sensor[2]);

	if (IMU.gyroscopeAvailable())
		IMU.readGyroscope(sensor[3], sensor[4], sensor[5]);
}

void preProcessSignal(){
	sensor[0] = sensor[0] - bias[0];
	sensor[1] = sensor[1] - bias[1];
	sensor[2] = sensor[2] - bias[2];
	sensor[3] = sensor[3] - bias[3];
	sensor[4] = sensor[4] - bias[4];
	sensor[5] = sensor[5] - bias[5];
}

void updateOdometry(float* sensor, float* pose){
	for (int i=0; i < 3; i++){
		vel[i] += sensor[i] * (CYCLE_TIME * 0.0001);
		pose[i] += vel[i] * (CYCLE_TIME * 0.0001);
		pose[i + 3] += sensor[i + 3] * (CYCLE_TIME * 0.0001);
	}
}

void setup() {
	birthTime = millis();

	Serial.begin(9600);
	while (!Serial);

	if (!IMU.begin())		
		while (1);
}

void loop() {
	start = millis();
	readIMU(sensor);									// read IMU sensor registers
	preProcessSignal();
	updateOdometry(sensor, pose);						// update pose based on measurements

	if(isConnected)
		dumpState();
	// else
	// 	Serial.println(WHOAMI);

	if(Serial.available() && OP_MODE != 'B'){
		serialEvent();
		Serial.flush();
	}

	if(OP_MODE == 'Z')
		isConnected = true;


	////////		Normalize cycle time		////////
	duration = millis() - start;
	if (duration < CYCLE_TIME)
		delay(CYCLE_TIME - duration);
}
