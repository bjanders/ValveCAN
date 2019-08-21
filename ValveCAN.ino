#include <I2CDevice.h>
#include <MCP23017.h>
#include <mcp_can.h>
#include <OneWireExt.h>
#include <SPI.h>
#include <Wire.h>


/* TODO:
* - Excersice all valvues once a day
* - PID algorithm
* - take floor temperature into account
* - take outside temp into account?
* - night temperature
* - away temperature?
*/

const int CAN_CS_PIN = 10;

MCP_CAN CAN(CAN_CS_PIN);

char tempStr[10];
byte canMsgLen;
byte canBuf[8];
unsigned long canID;
byte mcp23017_GPIOA;
byte mcp23017_GPIOB;


I2CDevice portExpander(MCP23017_ADDR);


// CAN IDs 100 - 191 are used
// CAN ID 100: Set point for room 1
// CAN ID 101: Temperature for room 1
// CAN ID 110: Set point for room 2
// CAN ID 111: Temperature for room 1
// etc., i.e. 10 rooms are used (but we support up to 16)
//

const short INVALID_TEMP = 0xffff;

unsigned long lastPID = 0;
unsigned long millisNow = 0;
char pidStr[150];

byte kP = 1;
byte kI = 0;
byte kD = 0;
byte dt = 20;  // seconds

struct PID {
	byte id;
	short setPoint;
	short temperature;
	short err;
	short prevErr;
	short P;
	short I;
	short D;
	short output;
} pids[16];

void setup()
{
	Wire.begin();
	Serial.begin(9600);
	Serial.println("Starting CAN");
	while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz)) {
		delay(100);
	}
	initMCP23017Pins();
	initPIDs();
	Serial.println("Setup done");
}

void loop()
{
	if (CAN_MSGAVAIL == CAN.checkReceive()) {
		CAN.readMsgBufID(&canID, &canMsgLen, canBuf);
		if (canID >= 0x100 && canID < 0x200) {
			if (canMsgLen == 2) {
				setTemp((unsigned short)canID, (short)*canBuf);
			} else {
				Serial.println("Invalid message length");
			}
		}
	}
	millisNow = millis();
	if (millisNow - lastPID > dt * 1000) {
		calcPIDs();
		lastPID = millisNow;
	}
}


void initPIDs() {
	for (int i = 0; i < 16; i++) {
		pids[i].id = i;
		pids[i].setPoint = INVALID_TEMP;
		pids[i].temperature = INVALID_TEMP;
		pids[i].err = 0;
		pids[i].prevErr = 0;
		pids[i].P = 0;
		pids[i].I = 0;
		pids[i].D = 0;
		pids[i].output = 0;
	}
}

void setTemp(unsigned short canID, short temp) {
	byte index;
	if ((canID & 0xf) == 0) {
		Serial.print("Set point ");
		Serial.print(canID, HEX);
		Serial.print(": ");
		owTempToStrFull(tempStr, *((short *)canBuf));
		Serial.println(tempStr);
		index = (canID >> 4) - 0x10;
		if (temp != pids[index].setPoint) {
			pids[index].setPoint = temp;
		}
	} else if ((canID & 0xf) == 1) {
		Serial.print("Temperature ");
		Serial.print(canID, HEX);
		Serial.print(": ");
		owTempToStrFull(tempStr, *((short *)canBuf));
		Serial.println(tempStr);
		index = (canID >> 4) - 0x10;
		if (temp != pids[index].temperature) {
			pids[index].temperature = temp;
		}
	} else {
		Serial.print("Unknown CAN ID ");
		Serial.println(canID, HEX);
	}
}

void calcPID(struct PID *pid) {
	if (pid->setPoint == INVALID_TEMP || pid->temperature == INVALID_TEMP) {
		return;
	}
	pid->err = pid->setPoint - pid->temperature;
	pid->P = kP * pid->err;
	pid->I += pid->err * kI; // * dt
	//pid->D = kD * (pid->prevErr * pid->err) / dt;
	pid->prevErr = pid->err;
	pid->output = pid->P + pid->I + pid->D;
	sprintf(pidStr, "PID %d: err %d, P %d, I %d, D %d, output %d", pid->id, pid->err, pid->P, pid->I, pid->D, pid->output);
	Serial.println(pidStr);
	setValve(pid);
}

void calcPIDs() {
	for (int i = 0; i < 16; i++) {
		calcPID(&pids[i]);
	}
}


// NOTE: Powering valve shuts it off
void setValve(struct PID *pid) {
	byte reg;
	byte *stat;
	byte bit;
	if (pid->setPoint == INVALID_TEMP || pid->temperature == INVALID_TEMP) {
		Serial.print("Temperature or set point not received for ");
		Serial.println(pid->id);
		return;
	}
	if (pid->id < 8) {
		reg = MCP23017_GPIOA;
		stat = &mcp23017_GPIOA;
		bit = pid->id;
	} else {
		reg = MCP23017_GPIOB;
		stat = &mcp23017_GPIOB;
		bit = pid->id - 7;
	}
	if (pid->output > 0) {
		if (bitRead(*stat, bit) == 0) {
			// Already on
			return;
		}
		Serial.print("Turning on valve for ");
		// turn valve on, output to low
		bitClear(*stat, bit);
	} else {
		if (bitRead(*stat, bit) == 1) {
			// Already off
			return;
		}
		Serial.print("Turning off valve for ");
		// turn valve off, output to high
		bitSet(*stat, bit);
	}
	portExpander.writeRegister(reg, *stat);
	Serial.println(pid->id);
	Serial.print(mcp23017_GPIOA, BIN);
	Serial.print(' ');
	Serial.println(mcp23017_GPIOB, BIN);
}


void initMCP23017Pins(void)
{
	// Set all to output
	Wire.beginTransmission(MCP23017_ADDR);
	Wire.write(MCP23017_IODIRA);
	Wire.write(0x00);
	// Same for IODIRB,
	Wire.write(0x00);
	Wire.endTransmission();
	Wire.beginTransmission(MCP23017_ADDR);
	// Set all to low for GPIOA and GPIOB 
	Wire.write(MCP23017_GPIOA);
	Wire.write(0x00);
	Wire.write(0x00);
	Wire.endTransmission();
	mcp23017_GPIOA = 0;
	mcp23017_GPIOB = 0;
}