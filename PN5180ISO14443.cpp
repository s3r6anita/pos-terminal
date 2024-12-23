// NAME: PN5180ISO14443.h
//
// DESC: ISO14443 protocol on NXP Semiconductors PN5180 module for Arduino.
//
// Copyright (c) 2019 by Dirk Carstensen. All rights reserved.
//
// This file is part of the PN5180 library for the Arduino environment.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
//#define DEBUG 1

#include <Arduino.h>
#include "PN5180ISO14443.h"
#include <PN5180.h>
#include "Debug.h"

PN5180ISO14443::PN5180ISO14443(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin, SPIClass& spi) 
              : PN5180(SSpin, BUSYpin, RSTpin, spi) {
}

// установка параметров для работы стандарта ISO14443
bool PN5180ISO14443::setupRF() {
  PN5180DEBUG_PRINTLN(F("Loading RF-Configuration..."));
  PN5180DEBUG_ENTER;
  
  if (loadRFConfig(0x00, 0x80)) {  // ISO14443 parameters
    PN5180DEBUG_PRINTLN(F("done."));
  }
  else {
	PN5180DEBUG_EXIT;
	return false;
  }
  PN5180DEBUG_PRINTLN(F("Turning ON RF field..."));
  if (setRF_on()) {
    PN5180DEBUG_PRINTLN(F("done."));
  }
  else {
    PN5180DEBUG_EXIT;
    return false;
  }
  
  PN5180DEBUG_EXIT;
  return true;
}


uint16_t PN5180ISO14443::rxBytesReceived() {
	PN5180DEBUG_PRINTLN(F("PN5180ISO14443::rxBytesReceived()"));	
	PN5180DEBUG_ENTER;
	
	uint32_t rxStatus;
	uint16_t len = 0;

	readRegister(RX_STATUS, &rxStatus);
	// Lower 9 bits has length
	len = (uint16_t)(rxStatus & 0x000001ff);

	PN5180DEBUG_EXIT;
	return len;
}



/*
* buffer : must be 10 byte array
* buffer[0-1] is ATQA
* buffer[2] is sak
* buffer[3..6] is 4 byte UID
* buffer[7..9] is remaining 3 bytes of UID for 7 Byte UID tags
* kind : 0  we send REQA, 1 we send WUPA
*
* return value: the uid length:
* -	zero if no tag was recognized
* - -1 general error
* - -2 card in field but with error
* -	single Size UID (4 byte)
* -	double Size UID (7 byte)
* -	triple Size UID (10 byte) - not yet supported
*/
int8_t PN5180ISO14443::activateTypeA(uint8_t *buffer, uint8_t kind) {
	uint8_t cmd[7];
	uint8_t uidLength = 0;
	
	PN5180DEBUG_PRINTF(F("PN5180ISO14443::activateTypeA(*buffer, kind=%d)"), kind);
	PN5180DEBUG_PRINTLN();
	PN5180DEBUG_ENTER;

	// Load standard TypeA protocol already done in reset()
	if (!loadRFConfig(0x0, 0x80)) {
		PN5180DEBUG_PRINTLN(F("*** ERROR: Load standard TypeA protocol failed!"));
		PN5180DEBUG_EXIT;
		return -1;
	}

	// activate RF field
	setRF_on();
	// wait RF-field to ramp-up
	delay(10);
	
	// OFF Crypto
	if (!writeRegisterWithAndMask(SYSTEM_CONFIG, 0xFFFFFFBF)) {
		PN5180DEBUG_PRINTLN(F("*** ERROR: OFF Crypto failed!"));
		PN5180DEBUG_EXIT;
		return -1;
	}
	// clear RX CRC
	if (!writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE)) {
		PN5180DEBUG_PRINTLN(F("*** ERROR: Clear RX CRC failed!"));
		PN5180DEBUG_EXIT;
		return -1;
	}
	// clear TX CRC
	if (!writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE)) {
		PN5180DEBUG_PRINTLN(F("*** ERROR: Clear TX CRC failed!"));
		PN5180DEBUG_EXIT;
		return -1;
	}

	// set the PN5180 into IDLE state  
	if (!writeRegisterWithAndMask(SYSTEM_CONFIG, 0xFFFFFFF8)) {
		PN5180DEBUG_PRINTLN(F("*** ERROR: set IDLE state failed!"));
		PN5180DEBUG_EXIT;
		return -1;
	}
		
	// activate TRANSCEIVE routine  
	if (!writeRegisterWithOrMask(SYSTEM_CONFIG, 0x00000003)) {
		PN5180DEBUG_PRINTLN(F("*** ERROR: Activates TRANSCEIVE routine failed!"));
		PN5180DEBUG_EXIT;
		return -1;
	}
	
	// wait for wait-transmit state
	PN5180TransceiveStat transceiveState = getTransceiveState();
	if (PN5180_TS_WaitTransmit != transceiveState) {
		PN5180DEBUG_PRINTLN(F("*** ERROR: Transceiver not in state WaitTransmit!?"));
		PN5180DEBUG_EXIT;
		return -1;
	}
	
/*	uint8_t irqConfig = 0b0000000; // Set IRQ active low + clear IRQ-register
    writeEEprom(IRQ_PIN_CONFIG, &irqConfig, 1);
    // enable only RX_IRQ_STAT, TX_IRQ_STAT and general error IRQ
    writeRegister(IRQ_ENABLE, RX_IRQ_STAT | TX_IRQ_STAT | GENERAL_ERROR_IRQ_STAT);  
*/

	// clear all IRQs
	clearIRQStatus(0xffffffff); 

	//Send REQA/WUPA, 7 bits in last byte
	cmd[0] = (kind == 0) ? 0x26 : 0x52;
	if (!sendData(cmd, 1, 0x07)) {
		PN5180DEBUG_PRINTLN(F("*** ERROR: Send REQA/WUPA failed!"));
		PN5180DEBUG_EXIT;
		return 0;
	}
	
	// wait some mSecs for end of RF receiption
	delay(10);

	// READ 2 bytes ATQA into  buffer
	if (!readData(2, buffer)) {
		PN5180DEBUG(F("*** ERROR: READ 2 bytes ATQA failed!\n"));
		PN5180DEBUG_EXIT;
		return 0;
	}
	
	// 
	unsigned long startedWaiting = millis();
    PN5180DEBUG_PRINTLN(F("wait for PN5180_TS_WaitTransmit (max 200ms)"));
    PN5180DEBUG_OFF;
	while (PN5180_TS_WaitTransmit != getTransceiveState()) {   
		if (millis() - startedWaiting > 200) {
			PN5180DEBUG_ON;
			PN5180DEBUG_PRINTLN(F("*** ERROR: timeout in PN5180_TS_WaitTransmit!"));
			PN5180DEBUG_EXIT;
			return -1; 
		}	
	}
    PN5180DEBUG_ON;
	
	// clear all IRQs
	clearIRQStatus(0xffffffff); 
	
	// send Anti collision 1, 8 bits in last byte
	cmd[0] = 0x93;
	cmd[1] = 0x20;
	if (!sendData(cmd, 2, 0x00)) {
		PN5180DEBUG_PRINTLN(F("*** ERROR: Send Anti collision 1 failed!"));
		PN5180DEBUG_EXIT;
		return -2;
	}
	
	// wait some mSecs for end of RF receiption
	delay(5);

	uint8_t numBytes = rxBytesReceived();
	if (numBytes != 5) {
		PN5180DEBUG_PRINTLN(F("*** ERROR: Read 5 bytes sak failed!"));
		PN5180DEBUG_EXIT;
		return -2;
	};
	// read 5 bytes sak, we will store at offset 2 for later usage
	if (!readData(5, cmd+2)) {
		Serial.println("Read 5 bytes failed!");
		PN5180DEBUG_EXIT;
		return -2;
	}
	// We do have a card now! enable CRC and send anticollision
	// save the first 4 bytes of UID
	for (int i = 0; i < 4; i++) buffer[i] = cmd[2 + i];
	
	//Enable RX CRC calculation
	if (!writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01)) {
		PN5180DEBUG_EXIT;
		return -2;
	}
	//Enable TX CRC calculation
	if (!writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01)) {
		PN5180DEBUG_EXIT;
		return -2;
	}

	//Send Select anti collision 1, the remaining bytes are already in offset 2 onwards
	cmd[0] = 0x93;
	cmd[1] = 0x70;
	if (!sendData(cmd, 7, 0x00)) {
		// no remaining bytes, we have a 4 byte UID
		PN5180DEBUG_EXIT;
		return 4;
	}
	//Read 1 byte SAK into buffer[2]
	if (!readData(1, buffer+2)) {
		PN5180DEBUG_EXIT;
		return -2;
	}
	// Check if the tag is 4 Byte UID or 7 byte UID and requires anti collision 2
	// If Bit 3 is 0 it is 4 Byte UID - стр. 27 iso14443-3
	if ((buffer[2] & 0x04) == 0) {
		// Take first 4 bytes of anti collision as UID store at offset 3 onwards. job done
		for (int i = 0; i < 4; i++) buffer[3+i] = cmd[2 + i];
		uidLength = 4;
	}
	else {
		// Take First 3 bytes of UID, Ignore first byte 88(CT)
        // если байт = 88, то произошла коллизия и для упрощения мы не будем ее обрабатывать
		if (cmd[2] != 0x88) {
			PN5180DEBUG_EXIT;
			return 0;
		}
		for (int i = 0; i < 3; i++) buffer[3+i] = cmd[3 + i];
		// Clear RX CRC
		if (!writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE)) {
			PN5180DEBUG_EXIT;
			return -2;
		}
		// Clear TX CRC
		if (!writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE)) {
			PN5180DEBUG_EXIT;
			return -2;
		}
		// Do anti collision 2
		cmd[0] = 0x95;
		cmd[1] = 0x20;
		if (!sendData(cmd, 2, 0x00)) {
			PN5180DEBUG_EXIT;
			return -2;
		}
		//Read 5 bytes. we will store at offset 2 for later use
		if (!readData(5, cmd+2)) {
			PN5180DEBUG_EXIT;
			return -2;
		}
		// first 4 bytes belongs to last 4 UID bytes, we keep it.
		for (uint8_t i = 0; i < 4; i++) {
		  buffer[6 + i] = cmd[2+i];
		}
		//Enable RX CRC calculation
		if (!writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01)) {
			PN5180DEBUG_EXIT;
			return -2;
		}
		//Enable TX CRC calculation
		if (!writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01)) {
			PN5180DEBUG_EXIT;
			return -2;
		}
		//Send Select anti collision 2 
		cmd[0] = 0x95;
		cmd[1] = 0x70;
		if (!sendData(cmd, 7, 0x00)) {
			PN5180DEBUG_EXIT;
			return -2;
		}
		//Read 1 byte SAK into buffer[2]
		if (!readData(1, buffer + 2)) {
			PN5180DEBUG_EXIT;
			return -2;
		}

		uidLength = 7;
	}
	PN5180DEBUG_EXIT;
    return uidLength;
}

// чтение UID карты и проверки его корректности
int8_t PN5180ISO14443::readCardSerial(uint8_t *buffer) {
	PN5180DEBUG_PRINTLN(F("PN5180ISO14443::readCardSerial(*buffer)"));
	PN5180DEBUG_ENTER;
  
	// Always return 10 bytes
    // Offset 0..1 is ATQA
    // Offset 2 is SAK.
    // UID 4 bytes : offset 3 to 6 is UID, offset 7 to 9 to Zero
    // UID 7 bytes : offset 3 to 9 is UID
	// try to activate Type A until response or timeout
    uint8_t response[10] = { 0 };
	int8_t uidLength = activateTypeA(response, 0);

	if (!activateISO14443_4()) {
		Serial.println("Error activating ISO14443_4");
  	}

    uint8_t selectAID[] = {0x00, 0xA4, 0x04, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x04, 0x10, 0x10};

	if (sendAPDUwithHalfDuplexBlock(selectAID, sizeof(selectAID))) {
    	Serial.println(F("APDU sent successfully using I-Block"));
	} else {
    	Serial.println(F("Failed to send APDU using I-Block"));
	}

	uint8_t apduResponse[256];
	uint8_t responseLength = 0;

    delay(25);

	if (receiveAPDUResponse(apduResponse, responseLength)) {
    	Serial.println(F("APDU response received successfully"));
	} else {
    	Serial.println(F("Failed to receive APDU response"));
	}

    Serial.println();
    delay(100);

    uint8_t selectGPO[] = {0x80, 0xA8, 0x00, 0x00, 0x02, 0x83, 0x00};
//    uint8_t selectGPO[] = {0x80, 0xA8, 0x00, 0x00, 0x04, 0x83, 0x02, 0x83,  0x00};

	if (sendAPDUwithHalfDuplexBlock(selectGPO, sizeof(selectGPO))) {
    	Serial.println(F("APDU selectGPO sent successfully using I-Block"));
	} else {
    	Serial.println(F("Failed to send APDU selectGPO using I-Block"));
	}

	uint8_t apduResponse2[256];
	uint8_t responseLength2 = 0;

    delay(25);

	if (receiveAPDUResponse(apduResponse2, responseLength2)) {
    	Serial.println(F("APDU response selectGPO received successfully"));
	} else {
    	Serial.println(F("Failed to receive APDU selectGPO response"));
	}



    Serial.println();
    delay(100);

    findCardholderName();




	if (uidLength <= 0){
	  PN5180DEBUG_EXIT;
	  return uidLength;
	}
	// UID length must be at least 4 bytes
	if (uidLength < 4){
	  PN5180DEBUG_EXIT;
	  return 0;
	}
    // проверка, что ATQA != 0xFFFF
	if ((response[0] == 0xFF) && (response[1] == 0xFF))
	  uidLength = 0;

	// first UID byte should not be 0x00 or 0xFF
	if ((response[3] == 0x00) || (response[3] == 0xFF))
		uidLength = 0;
		
	// check for valid uid, skip first byte (0x04)
	// 0x04 0x00 0xFF 0x00 => invalid uid
	bool validUID = false;
	for (int i = 1; i < uidLength; i++) {
		if ((response[i+3] != 0x00) && (response[i+3] != 0xFF)) {
			validUID = true;
			break;
		}
	}
	if (uidLength == 4) {
		if ((response[3] == 0x88)) {
			// must not be the CT-flag (0x88)!
			validUID = false;
		};
	}
	if (uidLength == 7) {
		if ((response[6] == 0x88)) {
			// must not be the CT-flag (0x88)!
			validUID = false;
		};
		if ((response[6] == 0x00) && (response[7] == 0x00) && (response[8] == 0x00) && (response[9] == 0x00)) {
			validUID = false;
		};
		if ((response[6] == 0xFF) && (response[7] == 0xFF) && (response[8] == 0xFF) && (response[9] == 0xFF)) {
			validUID = false;
		};
	};

	if (validUID) {
		for (int i = 0; i < uidLength; i++) buffer[i] = response[i+3];
		PN5180DEBUG_EXIT;
		return uidLength;
	} else {
		PN5180DEBUG_EXIT;
		return 0;
	}
}

bool PN5180ISO14443::isCardPresent() {
	PN5180DEBUG_PRINTLN("PN5180ISO14443::isCardPresent()");
	PN5180DEBUG_ENTER;

    uint8_t buffer[10];
	
	bool ret = (readCardSerial(buffer) >=4);

	PN5180DEBUG_EXIT;
	return ret;
}


bool PN5180ISO14443::activateISO14443_4() {

    // RATS command, стр. 14 iso14443-4
  	uint8_t rats[] = {0xE0, 0x80}; // 8 = настройка FSD = 256 bytes
  	if (!sendData(rats, 2, 0x00)) {
    	Serial.println(F("Failed to send RATS"));
    	return false;
  	}

  	// wait some mSecs for end of RF receiption
  	delay(5);

	uint8_t atsLen = rxBytesReceived();
    uint8_t ats[255];

    // read ats
  	if (!readData(atsLen, ats)) {
    	Serial.println(F("Failed to receive ATS"));
    	return false;
  	}

	Serial.print("ATS = ");
    for (uint8_t i = 0; i < atsLen; i++) {
    	Serial.print(ats[i], HEX);
    	if (i < atsLen-1) Serial.print(":");
  	}
    Serial.println();

  	// PPS skiped, work on default params
	return true;
}

uint8_t currentBlockNumber = 0; // Начинается с 0 после активации

bool PN5180ISO14443::sendAPDUwithHalfDuplexBlock(uint8_t* apdu, size_t apduLen) {

  // временно поменял на 64, чтобы не тратить лишнюю память
    const uint8_t FSC = 64; // (Frame Size for proximity Card) соответствует установленному FSD = 256 bytes
    uint8_t pcb = 0x0A | currentBlockNumber;     // PCB для I-Block с CID following
    uint8_t cid = 0x00;     // будем считать, что ATS всегда возвращает CID=0b0000

    size_t offset = 0;      // Текущий указатель на данные APDU
    uint8_t frame_buffer[FSC];     // Буфер для кадра

    while (offset < apduLen) {
        // Рассчитываем оставшуюся длину данных
        size_t remaining = apduLen - offset;

        // Рассчитываем размер текущего фрейма
        size_t frameSize = remaining > (FSC - 2) ? (FSC - 2) : remaining; // FSC - PCB и CID

        // Формируем I-Block
        frame_buffer[0] = pcb;         // PCB
        frame_buffer[1] = cid;         // CID (если используется)
        memcpy(frame_buffer + 2, apdu + offset, frameSize); // INF

        // Отправляем I-Block
        if (!sendData(frame_buffer, frameSize + 2, 0x00)) {
            Serial.println(F("Failed to send I-Block"));
            return false;
        }

//        Serial.print(F("Sending I-Block = "));
//        for (uint8_t i = 0; i < frameSize+2; i++) {
//          Serial.print(frame_buffer[i], HEX);
//        }
//        Serial.println();

        // Смещаем указатель на переданные данные
        offset += frameSize;

//        // Устанавливаем флаг цепочки (Chaining Flag) в PCB, если есть оставшиеся данные
//        if (offset < apduLen) {
//            pcb |= 0x10; // Устанавливаем флаг цепочки (More Data Bit)
//        } else {
//            pcb &= ~0x10; // Сбрасываем флаг цепочки
//        }
    }

	currentBlockNumber ^= 1;
    return true;
}

bool PN5180ISO14443::receiveAPDUResponse(uint8_t* response, uint8_t& responseLen) {
    const size_t FSC = 256; // Максимальный размер кадра
    size_t receivedLen = rxBytesReceived(); // Количество доступных байт

    // Проверяем, есть ли данные для чтения
    if (receivedLen == 0) {
        Serial.println(F("No data received from card"));
        return false;
    }

    // Убедимся, что данные не превышают размер буфера
    if (receivedLen > FSC) {
        Serial.println(F("Received data exceeds frame size"));
        return false;
    }

    responseLen = receivedLen;

    // Чтение данных из карты
    if (!readData(responseLen, response)) {
        Serial.println(F("Failed to read response from card"));
        return false;
    }

    // Выводим полученные данные для отладки
    Serial.print(F("APDU Response: "));
    for (size_t i = 0; i < responseLen; i++) {
        Serial.print(response[i], HEX);
        if (i < responseLen - 1) Serial.print(":");
    }
    Serial.println();

//    // Send R-Block to confirm
//    uint8_t rBlock[] = {0xA2}; // R-Block
//    if (!sendData(rBlock, sizeof(rBlock), 0x00)) {
//        Serial.println(F("Failed to send R-Block"));
//        return false;
//    }
//    Serial.println(F("R-Block sent successfully"));

    // Проверка статусных байтов SW1 и SW2
    if (responseLen >= 2) {
        uint8_t sw1 = response[responseLen - 2];
        uint8_t sw2 = response[responseLen - 1];

        Serial.print(F("SW1: "));
        Serial.print(sw1, HEX);
        Serial.print(F(", SW2: "));
        Serial.println(sw2, HEX);

        // Проверяем успешное выполнение команды
        if (sw1 == 0x90 && sw2 == 0x00) {
            Serial.println(F("Command executed successfully"));
            return true;
        } else {
            Serial.println(F("Card returned an error"));
            return false;
        }
    } else {
        Serial.println(F("Response too short to contain SW1 and SW2"));
        return false;
    }
}

void PN5180ISO14443::findCardholderName() {
    uint8_t response[256];
    uint8_t responseLen = 0;

    // SFI = 2, запись 1
	// вернула 5A = PAN = 5228600562170292
	// вернула 5F 24 = Application Expiration Date = 26 10 31 (31 октября 2026 года)
    uint8_t readRecordSFI2[] = {0x00, 0xB2, 0x01, (0x02 << 3) | 0x04, 0x00};
//    uint8_t readRecordSFI2[] = {0x00, 0xB2, 0x01, (0x10 + 0x04), 0x00};

    // вернул AIP
//    uint8_t readRecordSFI2[] = {0x00, 0xB2, 0x01, (0x08 + 0x04), 0x00};

    // ничего полезного не вернул
//    uint8_t readRecordSFI2[] = {0x00, 0xB2, 0x02, (0x18 + 0x04), 0x00};

    // ничего полезного не вернул
//    uint8_t readRecordSFI2[] = {0x00, 0xB2, 0x01, (0x20 + 0x04), 0x00};


    if (sendAPDUwithHalfDuplexBlock(readRecordSFI2, sizeof(readRecordSFI2))) {
    	delay(25);
		if (receiveAPDUResponse(response, responseLen)) {
        	Serial.println(F("Record read successfully (SFI = 2, Record 1)"));
        	// Искать тег 5F20
        	for (int i = 0; i < responseLen; i++) {
            	if (response[i] == 0x5F && response[i + 1] == 0x20) {
                	uint8_t nameLength = response[i + 2];
                	Serial.print(F("Cardholder Name: "));
                	for (int j = 0; j < nameLength; j++) {
                    	Serial.print((char)response[i + 3 + j]);
                	}
                	Serial.println();
                	return; // Имя найдено, выходим
            	}
        	}
		}
    }
    Serial.println(F("Failed to read record (SFI = 2, Record 1)"));
}



bool PN5180ISO14443::mifareHalt() {
	uint8_t cmd[2];
	//mifare Halt
	cmd[0] = 0x50;
	cmd[1] = 0x00;
	sendData(cmd, 2, 0x00);
	return true;
}




bool PN5180ISO14443::mifareBlockRead(uint8_t blockno, uint8_t *buffer) {
	bool success = false;
	uint16_t len;
	uint8_t cmd[2];
	// Send mifare command 30,blockno
	cmd[0] = 0x30;
	cmd[1] = blockno;
	if (!sendData(cmd, 2, 0x00))
	  return false;
	//Check if we have received any data from the tag
	delay(5);
	len = rxBytesReceived();
	if (len == 16) {
		// READ 16 bytes into  buffer
		if (readData(16, buffer))
		  success = true;
	}
	return success;
}


uint8_t PN5180ISO14443::mifareBlockWrite16(uint8_t blockno, const uint8_t *buffer) {
	uint8_t cmd[2];
	// Clear RX CRC
	writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE);

	// Mifare write part 1
	cmd[0] = 0xA0;
	cmd[1] = blockno;
	sendData(cmd, 2, 0x00);
	readData(1, cmd);

	// Mifare write part 2
	sendData(buffer,16, 0x00);
	delay(10);

	// Read ACK/NAK
	readData(1, cmd);

	//Enable RX CRC calculation
	writeRegisterWithOrMask(CRC_RX_CONFIG, 0x1);
	return cmd[0];
}