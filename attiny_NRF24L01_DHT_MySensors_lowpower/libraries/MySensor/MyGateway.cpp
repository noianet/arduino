/*
 The MySensors library adds a new layer on top of the RF24 library.
 It handles radio network routing, relaying and ids.

 Created by Henrik Ekblad <henrik.ekblad@gmail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
*/
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168__) || defined (__AVR_ATmega2560__) || defined (__AVR_ATmega32U4__)

#include "MyGateway.h"
#include "utility/MsTimer2.h"
#include "utility/PinChangeInt.h"


int8_t pinRx;
int8_t pinTx;
int8_t pinEr;
boolean buttonTriggeredInclusion;
volatile uint8_t countRx;
volatile uint8_t countTx;
volatile uint8_t countErr;
boolean inclusionMode; // Keeps track on inclusion mode

#define LED_ON  (LOW)
#define LED_OFF (HIGH)


MyGateway::MyGateway(uint8_t _cepin, uint8_t _cspin, uint8_t _inclusion_time, int8_t _inclusion_pin, int8_t _rx, int8_t _tx, int8_t _er) : MySensor(_cepin, _cspin) {
	pinInclusion = _inclusion_pin;
	inclusionTime = _inclusion_time;
	pinRx = _rx;
	pinTx = _tx;
	pinEr = _er;
}


void MyGateway::begin(rf24_pa_dbm_e paLevel, uint8_t channel, rf24_datarate_e dataRate, void (*inDataCallback)(char *)) {
	Serial.begin(BAUD_RATE);
	repeaterMode = true;
	isGateway = true;
	autoFindParent = false;
	setupRepeaterMode();

	if (inDataCallback != NULL) {
		useWriteCallback = true;
		dataCallback = inDataCallback;
	} else {
		useWriteCallback = false;
	}

	nc.nodeId = 0;
	nc.parentNodeId = 0;
	nc.distance = 0;
	inclusionMode = 0;
	buttonTriggeredInclusion = false;
	countRx = 0;
	countTx = 0;
	countErr = 0;

	// Setup led pins & set initial state of leds
	if (pinRx >= 0)
	{
		digitalWrite(pinRx, LED_OFF);
		pinMode(pinRx, OUTPUT);
	}
	if (pinTx >= 0)
	{
		digitalWrite(pinTx, LED_OFF);
		pinMode(pinTx, OUTPUT);
	}
	if (pinEr >= 0)
	{
		digitalWrite(pinEr, LED_OFF);
		pinMode(pinEr, OUTPUT);
	}
	// Setup digital in that triggers inclusion mode
	if (pinInclusion >= 0)
	{
		pinMode(pinInclusion, INPUT);
		// Add interrupt for inclusion button to pin
		PCintPort::attachInterrupt(pinInclusion, startInclusionInterrupt, RISING);
	}

	// Start up the radio library
	setupRadio(paLevel, channel, dataRate);
	RF24::openReadingPipe(WRITE_PIPE, BASE_RADIO_ID);
	RF24::openReadingPipe(CURRENT_NODE_PIPE, BASE_RADIO_ID);
	RF24::startListening();

	// Add led timer interrupt
    MsTimer2::set(300, ledTimersInterrupt);
    MsTimer2::start();

	// Send startup log message on serial
	serial(PSTR("0;0;%d;0;%d;Gateway startup complete.\n"),  C_INTERNAL, I_GATEWAY_READY);
}


void startInclusionInterrupt() {
	  buttonTriggeredInclusion = true;
}



void MyGateway::checkButtonTriggeredInclusion() {
   if (buttonTriggeredInclusion) {
    // Ok, someone pressed the inclusion button on the gateway
    // start inclusion mode for 1 munute.
    serial(PSTR("0;0;%d;0;%d;Inclusion started by button.\n"),  C_INTERNAL, I_LOG_MESSAGE);
    buttonTriggeredInclusion = false;
    setInclusionMode(true);
  }
}

void MyGateway::checkInclusionFinished() {
	if (inclusionMode && millis()-inclusionStartTime>60000UL*inclusionTime) {
	     // inclusionTimeInMinutes minute(s) has passed.. stop inclusion mode
	    setInclusionMode(false);
	 }
}

uint8_t MyGateway::h2i(char c) {
	uint8_t i = 0;
	if (c <= '9')
		i += c - '0';
	else if (c >= 'a')
		i += c - 'a' + 10;
	else
		i += c - 'A' + 10;
	return i;
}

void MyGateway::parseAndSend(char *commandBuffer) {
  boolean ok = false;
  char *str, *p, *value=NULL;
  uint8_t bvalue[MAX_PAYLOAD];
  uint8_t blen = 0;
  int i = 0;
  uint16_t destination = 0;
  uint8_t sensor = 0;
  uint8_t command = 0;
  uint8_t type = 0;
  uint8_t ack = 0;

  // Extract command data coming on serial line
  for (str = strtok_r(commandBuffer, ";", &p);       // split using semicolon
  		str && i < 6;         // loop while str is not null an max 5 times
  		str = strtok_r(NULL, ";", &p)               // get subsequent tokens
				) {
	switch (i) {
	  case 0: // Radioid (destination)
	 	destination = atoi(str);
		break;
	  case 1: // Childid
		sensor = atoi(str);
		break;
	  case 2: // Message type
		command = atoi(str);
		break;
	  case 3: // Should we request ack from destination?
		ack = atoi(str);
		break;
	  case 4: // Data type
		type = atoi(str);
		break;
	  case 5: // Variable value
		if (command == C_STREAM) {
			blen = 0;
			uint8_t val;
			while (*str) {
				val = h2i(*str++) << 4;
				val += h2i(*str++);
				bvalue[blen] = val;
				blen++;
			}
		} else {
			value = str;
			// Remove ending carriage return character (if it exists)
			uint8_t lastCharacter = strlen(value)-1;
			if (value[lastCharacter] == '\r')
				value[lastCharacter] = 0;
		}
		break;
	  }
	  i++;
  }

  if (destination==GATEWAY_ADDRESS && command==C_INTERNAL) {
    // Handle messages directed to gateway
    if (type == I_VERSION) {
      // Request for version
      serial(PSTR("0;0;%d;0;%d;%s\n"),C_INTERNAL, I_VERSION, LIBRARY_VERSION);
    } else if (type == I_INCLUSION_MODE) {
      // Request to change inclusion mode
      setInclusionMode(atoi(value) == 1);
    }
  } else {
    txBlink(1);
    msg.sender = GATEWAY_ADDRESS;
	msg.destination = destination;
	msg.sensor = sensor;
	msg.type = type;
	mSetCommand(msg,command);
	mSetRequestAck(msg,ack?1:0);
	mSetAck(msg,false);
	if (command == C_STREAM)
		msg.set(bvalue, blen);
	else
		msg.set(value);
    ok = sendRoute(msg);
    if (!ok) {
      errBlink(1);
    }
  }
}


void MyGateway::setInclusionMode(boolean newMode) {
  if (newMode != inclusionMode)
    inclusionMode = newMode;
    // Send back mode change on serial line to ack command
    serial(PSTR("0;0;%d;0;%d;%d\n"), C_INTERNAL, I_INCLUSION_MODE, inclusionMode?1:0);

    if (inclusionMode) {
      inclusionStartTime = millis();
    }
}

void MyGateway::processRadioMessage() {
	if (process()) {
	  // A new message was received from one of the sensors
	  MyMessage message = getLastMessage();
	  if (mGetCommand(message) == C_PRESENTATION && inclusionMode) {
		rxBlink(3);
	  } else {
		rxBlink(1);
	  }
	  // Pass along the message from sensors to serial line
	  serial(message);
	}

	checkButtonTriggeredInclusion();
	checkInclusionFinished();
}

void MyGateway::serial(const char *fmt, ... ) {
   va_list args;
   va_start (args, fmt );
   vsnprintf_P(serialBuffer, MAX_SEND_LENGTH, fmt, args);
   va_end (args);
   Serial.print(serialBuffer);
   if (useWriteCallback) {
	   // We have a registered write callback (probably Ethernet)
	   dataCallback(serialBuffer);
   }
}

void MyGateway::serial(MyMessage &msg) {
  serial(PSTR("%d;%d;%d;%d;%d;%s\n"),msg.sender, msg.sensor, mGetCommand(msg), mGetAck(msg), msg.type, msg.getString(convBuf));
}


void ledTimersInterrupt() {
  if (pinRx >= 0)
  {
    if(countRx && countRx != 255) {
	  // switch led on
	  digitalWrite(pinRx, LED_ON);
	} else if(!countRx) {
      // switching off
      digitalWrite(pinRx, LED_OFF);
    }
  }
  if(countRx != 255) { countRx--; }

  if (pinTx >= 0)
  {
    if(countTx && countTx != 255) {
      // switch led on
      digitalWrite(pinTx, LED_ON);
    } else if(!countTx) {
       // switching off
       digitalWrite(pinTx, LED_OFF);
    }
  }
  if(countTx != 255) { countTx--; }
  else if(inclusionMode) { countTx = 8; }

  if (pinEr >= 0)
  {
    if(countErr && countErr != 255) {
      // switch led on
      digitalWrite(pinEr, LED_ON);
    } else if(!countErr) {
      // switching off
      digitalWrite(pinEr, LED_OFF);
    }
  }
  if(countErr != 255) { countErr--; }
}

void MyGateway::rxBlink(uint8_t cnt) {
  if(countRx == 255) { countRx = cnt; }
}
void MyGateway::txBlink(uint8_t cnt) {
  if(countTx == 255 && !inclusionMode) { countTx = cnt; }
}
void MyGateway::errBlink(uint8_t cnt) {
  if(countErr == 255) { countErr = cnt; }
}

#endif