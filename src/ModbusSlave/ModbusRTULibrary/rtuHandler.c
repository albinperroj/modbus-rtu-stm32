/*
 * rtuHandler.c
 *
 *  Created on: Jun 22, 2021
 *      Author: Albin Perroj
 */

#include "rtuHandler.h"
#include "rtuHandlerHelp.h"

/*--------------------------------------------------------
 * Functions
 */


void InitMBSlave(uint8_t id, MBHandler *mbHandler, UART_HandleTypeDef *uartPort, TIM_HandleTypeDef *timerT35, TIM_HandleTypeDef *timerT15,
		uint32_t prescaler, uint32_t baudrate, uint32_t wordLength, uint32_t stopBits, uint32_t parity,
		GPIO_TypeDef *RE_Port, uint16_t RE_Pin, GPIO_TypeDef *DE_Port, uint16_t DE_Pin,
		uint8_t* coilsArray, uint16_t coilsArrLength, uint16_t* registersArray, uint16_t registersArrLength);

void StartMBSlave(MBHandler *mbHandler);

void InitMBMaster(MBHandler *mbHandler, UART_HandleTypeDef *uartPort, TIM_HandleTypeDef *timerT35, TIM_HandleTypeDef *timerT15,
		uint32_t prescaler, uint32_t baudrate, uint32_t wordLength, uint32_t stopBits, uint32_t parity,
		GPIO_TypeDef *RE_Port, uint16_t RE_Pin, GPIO_TypeDef *DE_Port, uint16_t DE_Pin);

void WaitingResponse(MBHandler *mbHandler);

void MBSlave_HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart, MBHandler *mbHandler);

void MBMaster_HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart, MBHandler *mbHandler);

void MBUnit_HAL_UART_ErrorCallback(UART_HandleTypeDef *huart, MBHandler *mbHandler);

void MBSlave_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim, MBHandler *mbHandler);

void MBMaster_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim, MBHandler *mbHandler);

void MBUnit_HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim, MBHandler *mbHandler);

// Common operation functions
void AddToXBuffer(XBuffer *xBuffer, uint8_t u8Element);

void ClearXBuffer(XBuffer *xBuffer);

uint16_t BytesToReg(uint8_t lo_Byte, uint8_t ho_Byte);

uint8_t* RegToBytes(uint16_t reg);

void CalcTimerPeriod(Timer tmr, float_t *period, float_t baudrate, float_t wordLength);

uint16_t CalculateCRC(uint8_t* buffer, uint8_t length);

void Modify_TIM(TIM_HandleTypeDef *htim , uint32_t prescaler, uint32_t period);

void Modify_UART(UART_HandleTypeDef *huart, uint32_t baudrate, uint32_t wordLength, uint32_t stopBits, uint32_t parity);

void Set_Coil(uint8_t index, uint8_t value);

void Set_Reg(uint8_t index, uint16_t value);

uint8_t Get_Coil(uint8_t index);

uint16_t Get_Reg(uint8_t index);

void EnableRxMode(MBHandler *mbHandler);

void EnableTxMode(MBHandler *mbHandler);

void TransmitXBuffer(MBHandler *mbHandler, XBuffer *xBuffer);

void Error_Handler(void);

void printMsg( char *msg,...);


// Modbus implementation functions
// Slave
void ProcessMBRequest(MBHandler *mbHandler, XBuffer rxBuffer);

int ProcessFC1(MBHandler *mbHandler, XBuffer rxBuffer);

int ProcessFC15(MBHandler *mbHandler, XBuffer rxBuffer);

int ProcessFC3(MBHandler *mbHandler, XBuffer rxBuffer);

int ProcessFC16(MBHandler *mbHandler, XBuffer rxBuffer);

// Master
uint8_t* ReadCoils(MBHandler *mbHandler, uint8_t slaveID,uint16_t startAddress, uint16_t coilsNumber);

int WriteCoils(MBHandler *mbHandler, uint8_t slaveID, uint16_t startAddress, uint8_t inputCoilsData[]);

int ReadRegisters(MBHandler *mbHandler, uint8_t slaveID, uint16_t startAddress, uint16_t regsNumber, uint16_t* regsOutData);

int WriteRegisters(MBHandler *mbHandler, uint8_t slaveID, uint16_t startAddress,uint16_t inputRegsData[]);

// General
void HandleError(MBHandler *mbHandler, Error error);

void HandleException(MBHandler *mbHandler, Exception exeption, XBuffer rxBuffer);


/*-------------------------------------------------------------------------------------------------------
 * C code
 */

void InitMBSlave(uint8_t id, MBHandler *mbHandler, UART_HandleTypeDef *uartPort, TIM_HandleTypeDef *timerT35, TIM_HandleTypeDef *timerT15,
		uint32_t prescaler, uint32_t baudrate, uint32_t wordLength, uint32_t stopBits, uint32_t parity,
		GPIO_TypeDef *RE_Port, uint16_t RE_Pin, GPIO_TypeDef *DE_Port, uint16_t DE_Pin,
		uint8_t* coilsArray, uint16_t coilsArrLength, uint16_t* registersArray, uint16_t registersArrLength)
{
	// Slave id
	(*mbHandler).UnitID = id;

	// Modify UART port and assign it to MBHandler uartPort
	Modify_UART(uartPort, baudrate, wordLength, stopBits, parity);
	(*mbHandler).UART_Port = uartPort;

	// Modify TIM (T35)
	float_t periodT35 = 0;
	CalcTimerPeriod(T35, &periodT35, baudrate, 11);
	Modify_TIM(timerT35, prescaler, (uint32_t)periodT35);
	(*mbHandler).T35 = timerT35;

	// Modify TIM (T15)
	float_t periodT15 = 0;
	CalcTimerPeriod(T15, &periodT15, baudrate, 11);
	Modify_TIM(timerT15, prescaler, (uint32_t)periodT15);
	(*mbHandler).T15 = timerT15;


	// Set RE & DE pin
	(*mbHandler).RE_Port = RE_Port;
	(*mbHandler).RE_Pin = RE_Pin;

	(*mbHandler).DE_Port = DE_Port;
	(*mbHandler).DE_Pin = DE_Pin;

	// Set coils and registers
	(*mbHandler).CoilsData.CoilsArray = coilsArray;
	(*mbHandler).CoilsData.CoilsLength = coilsArrLength;

	(*mbHandler).RegistersData.RegistersArray = registersArray;
	(*mbHandler).RegistersData.RegistersLength = registersArrLength;


	printMsg("Slave:ID:%d; BaudRate:%d; WordLength:%d; Parity:%d; StopBits:%d; pT35:%d; tpT15:%d;\n",
				(*mbHandler).UnitID,
				(*((*mbHandler).UART_Port)).Init.BaudRate,
				(*((*mbHandler).UART_Port)).Init.WordLength,
				(*((*mbHandler).UART_Port)).Init.Parity,
				(*((*mbHandler).UART_Port)).Init.StopBits,
				(uint32_t)(*((*mbHandler).T35)).Init.Period,
				(uint32_t)(*((*mbHandler).T15)).Init.Period);
}

void StartMBSlave(MBHandler *mbHandler)
{
	// Enable Receive Mode
	EnableRxMode(mbHandler);

	// Clear RxBuffer
	ClearXBuffer(&((*mbHandler).RxBuffer));

	// Start uart receive interrupt (to receive 1 byte)
	if(HAL_UART_Receive_IT((*mbHandler).UART_Port, (*mbHandler).RxBuffer.u8Element, 1) != HAL_OK)
	{
		HandleError(mbHandler, UART_Error);
	}
	else
	{
		printMsg("Slave started!\n");
	}
}

void InitMBMaster(MBHandler *mbHandler, UART_HandleTypeDef *uartPort, TIM_HandleTypeDef *timerT35, TIM_HandleTypeDef *timerT15,
		uint32_t prescaler, uint32_t baudrate, uint32_t wordLength, uint32_t stopBits, uint32_t parity,
		GPIO_TypeDef *RE_Port, uint16_t RE_Pin, GPIO_TypeDef *DE_Port, uint16_t DE_Pin)
{
	// Unit ID
	(*mbHandler).UnitID = 0;

	// Modify UART port and assign it to MBHandler uartPort
	Modify_UART(uartPort, baudrate, wordLength, stopBits, parity);
	(*mbHandler).UART_Port = uartPort;

	// Modify TIM (T35)
	float_t periodT35 = 0;
	CalcTimerPeriod(T35, &periodT35, baudrate, 11);
	Modify_TIM(timerT35, prescaler, (uint32_t)periodT35);
	(*mbHandler).T35 = timerT35;

	// Modify TIM (T15)
	float_t periodT15 = 0;
	CalcTimerPeriod(T15, &periodT15, baudrate, 11);
	Modify_TIM(timerT15, prescaler, (uint32_t)periodT15);
	(*mbHandler).T15 = timerT15;


	// Set RE & DE pin
	(*mbHandler).RE_Port = RE_Port;
	(*mbHandler).RE_Pin = RE_Pin;

	(*mbHandler).DE_Port = DE_Port;
	(*mbHandler).DE_Pin = DE_Pin;

	printMsg("Master:ID:%d; BaudRate:%d; WordLength:%d; Parity:%d; StopBits:%d; pT35:%d; tpT15:%d;\n",
				(*mbHandler).UnitID,
				(*((*mbHandler).UART_Port)).Init.BaudRate,
				(*((*mbHandler).UART_Port)).Init.WordLength,
				(*((*mbHandler).UART_Port)).Init.Parity,
				(*((*mbHandler).UART_Port)).Init.StopBits,
				(uint32_t)(*((*mbHandler).T35)).Init.Period,
				(uint32_t)(*((*mbHandler).T15)).Init.Period);
}

void WaitingResponse(MBHandler *mbHandler)
{
	// Clear RxBuffer
	ClearXBuffer(&((*mbHandler).RxBuffer));

	// Start uart receive interrupt (to receive 1 byte)
	if(HAL_UART_Receive_IT((*mbHandler).UART_Port, (*mbHandler).RxBuffer.u8Element, 1) != HAL_OK)
	{
	 	HandleError(mbHandler, UART_Error);
	}
}


void MBSlave_HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart, MBHandler *mbHandler)
{
	// Check if the uart module is the one that is used to receive modbus requests
	if((*huart).Instance == (*((*mbHandler).UART_Port)).Instance)
	{
		//------------------------------------------------------------------------------------------------------------------
		// Stop T35 if it is in busy state, meaning that it is counting from 0 to 'periodT35'
		if( HAL_TIM_Base_GetState((*mbHandler).T35) == HAL_TIM_STATE_BUSY)
		{
			if(HAL_TIM_Base_Stop_IT((*mbHandler).T35) == HAL_OK)
			{
				__HAL_TIM_SetCounter((*mbHandler).T35, 0);
			}
			else
			{
				HandleError(mbHandler, T35_Error);
			}
		}

		// Stop T15 if it is in busy state, meaning that last byte was received ok
		if( HAL_TIM_Base_GetState((*mbHandler).T15) == HAL_TIM_STATE_BUSY )
		{
			if(HAL_TIM_Base_Stop_IT((*mbHandler).T15) == HAL_OK)
			{
				__HAL_TIM_SetCounter((*mbHandler).T15, 0);
			}
			else
			{
				HandleError(mbHandler, T15_Error);
			}
		}

		// Add value to RxBuffer
		AddToXBuffer(&((*mbHandler).RxBuffer),((*mbHandler).RxBuffer).u8Element[0]);

		// Restart uart receive interrupt (to receive the next byte of the request)
		if(HAL_UART_Receive_IT((*mbHandler).UART_Port, (*mbHandler).RxBuffer.u8Element, 1) != HAL_OK)
		{
			HandleError(mbHandler, UART_Error);
		}


		// Start T35 to detect if the last received byte was the modbus request last byte
		if(HAL_TIM_Base_Start_IT((*mbHandler).T35) != HAL_OK)
		{
			HandleError(mbHandler, T35_Error);
		}

		// Start T15 to detect if the bytes are received ok
		if(HAL_TIM_Base_Start_IT((*mbHandler).T15) != HAL_OK)
		{
			HandleError(mbHandler, T15_Error);
		}
		//-----------------------------------------------------------------------------------------------------------------------
	}
}

void MBMaster_HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart, MBHandler *mbHandler)
{
	// Check if the uart module is the one that is used to receive modbus requests
	if((*huart).Instance == (*((*mbHandler).UART_Port)).Instance)
	{
		//------------------------------------------------------------------------------------------------------------------
		// Stop T35 if it is in busy state, meaning that it is counting from 0 to 'periodT35'
		if( HAL_TIM_Base_GetState((*mbHandler).T35) == HAL_TIM_STATE_BUSY)
		{
			if(HAL_TIM_Base_Stop_IT((*mbHandler).T35) == HAL_OK)
			{
				__HAL_TIM_SetCounter((*mbHandler).T35, 0);
			}
			else
			{
				HandleError(mbHandler, T35_Error);
			}
		}

		// Stop T15 if it is in busy state, meaning that last byte was received ok
		if( HAL_TIM_Base_GetState((*mbHandler).T15) == HAL_TIM_STATE_BUSY )
		{
			if(HAL_TIM_Base_Stop_IT((*mbHandler).T15) == HAL_OK)
			{
				__HAL_TIM_SetCounter((*mbHandler).T15, 0);
			}
			else
			{
				HandleError(mbHandler, T15_Error);
			}
		}

		// Add value to RxBuffer
		AddToXBuffer(&((*mbHandler).RxBuffer),((*mbHandler).RxBuffer).u8Element[0]);

		// Restart uart receive interrupt (to receive the next byte of the request)
		if(HAL_UART_Receive_IT((*mbHandler).UART_Port, (*mbHandler).RxBuffer.u8Element, 1) != HAL_OK)
		{
			HandleError(mbHandler, UART_Error);
		}


		// Start T35 to detect if the last received byte was the modbus request last byte
		if(HAL_TIM_Base_Start_IT((*mbHandler).T35) != HAL_OK)
		{
			HandleError(mbHandler, T35_Error);
		}

		// Start T15 to detect if the bytes are received ok
		if(HAL_TIM_Base_Start_IT((*mbHandler).T15) != HAL_OK)
		{
			HandleError(mbHandler, T15_Error);
		}
		//-----------------------------------------------------------------------------------------------------------------------
	}
}

void MBUnit_HAL_UART_ErrorCallback(UART_HandleTypeDef *huart, MBHandler *mbHandler)
{
	// Check if the uart module is the one that is used in mb_slave
	if((*huart).Instance == (*((*mbHandler).UART_Port)).Instance)
	{
		uint32_t uartError = HAL_UART_GetError((*mbHandler).UART_Port);
		if(uartError == 0 && uartError == 1 && uartError == 2 && uartError == 3 && uartError == 4 && uartError == 5)
		{
			HandleError(mbHandler, UART_Error);
		}
	}
}

void MBSlave_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim, MBHandler *mbHandler)
{
	// Check if this timer module is the one that is used to detect the modbus rtu end of frame
	if((*htim).Instance == (*((*mbHandler).T35)).Instance)
	{
		//--------------------------------------------------------------------------------------------------------
		// Period of T35 was elapsed, end of frame detected, stopping the T35
		if( HAL_TIM_Base_Stop_IT((*mbHandler).T35) == HAL_OK)
		{
			__HAL_TIM_SetCounter((*mbHandler).T35, 0);
		}
		else
		{
			HandleError(mbHandler, T35_Error);
		}

		//  stopping the T15, end of frame detected
		if(HAL_TIM_Base_Stop_IT((*mbHandler).T15) == HAL_OK)
		{
			__HAL_TIM_SetCounter((*mbHandler).T15, 0);
		}
		else
		{
			HandleError(mbHandler, T15_Error);
		}


		if((*mbHandler).RxBuffer.T15Counter <= 2)
		{
/*
			printMsg("%d:\t",(*mbHandler).RxBuffer.T15Counter);
			// printing the modbus request received (debugging)
			for(int k=0; k< (*mbHandler).RxBuffer.Index; k++)
			{
				printMsg("%u\t", (*mbHandler).RxBuffer.u8Array[k]);
			}
			printMsg(":\n");
*/

			ProcessMBRequest(mbHandler, (*mbHandler).RxBuffer);

			// Clearing the RxBuffer to be able to receive the nest modbus request
			ClearXBuffer(&((*mbHandler).RxBuffer));
		}
		else
		{
			HandleError(mbHandler, Tx_Frame_Error);
		}
	}

	if((*htim).Instance == (*((*mbHandler).T15)).Instance)
	{
		(*mbHandler).RxBuffer.T15Counter ++;
	}
}

void MBMaster_HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim, MBHandler *mbHandler)
{
	// Check if this timer module is the one that is used to detect the modbus rtu end of frame
	if((*htim).Instance == (*((*mbHandler).T35)).Instance)
	{
		//--------------------------------------------------------------------------------------------------------
		// Period of T35 was elapsed, end of frame detected, stopping the T35
		if( HAL_TIM_Base_Stop_IT((*mbHandler).T35) == HAL_OK)
		{
			__HAL_TIM_SetCounter((*mbHandler).T35, 0);
		}
		else
		{
			HandleError(mbHandler, T35_Error);
		}

		//  stopping the T15, end of frame detected
		if(HAL_TIM_Base_Stop_IT((*mbHandler).T15) == HAL_OK)
		{
			__HAL_TIM_SetCounter((*mbHandler).T15, 0);
		}
		else
		{
			HandleError(mbHandler, T15_Error);
		}


		if((*mbHandler).RxBuffer.T15Counter <= 2)
		{
			printMsg("%d:\t",(*mbHandler).RxBuffer.T15Counter);
			// printing the modbus request received (debugging)
			for(int k=0; k< (*mbHandler).RxBuffer.Index; k++)
			{
				printMsg("%u\t", (*mbHandler).RxBuffer.u8Array[k]);
			}
			printMsg(":\n");

			(*mbHandler).masterReceivedResp = true;
		}
		else
		{
			HandleError(mbHandler, Tx_Frame_Error);
		}
	}

	if((*htim).Instance == (*((*mbHandler).T15)).Instance)
	{
		(*mbHandler).RxBuffer.T15Counter ++;
	}
}

void MBUnit_HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim, MBHandler *mbHandler)
{
	if((*htim).Instance == (*((*mbHandler).T15)).Instance)
	{
		HandleError(mbHandler, T15_Error);
	}

	if((*htim).Instance == (*((*mbHandler).T35)).Instance)
	{
		HandleError(mbHandler, T35_Error);
	}
}


// Common operation functions
void AddToXBuffer(XBuffer *xBuffer, uint8_t u8Element)
{
	(*xBuffer).u8Array[(*xBuffer).Index] = u8Element;
	(*xBuffer).Index ++;

	if((*xBuffer).Index > (XBuffer_MAX - 1))
	{
		(*xBuffer).overflow = true;
	}
	else
	{
		(*xBuffer).overflow = false;
	}
}

void ClearXBuffer(XBuffer *xBuffer)
{
	(*xBuffer).Index = 0;
	(*xBuffer).T15Counter = 0;
	(*xBuffer).u8Element[0] = 0;
	(*xBuffer).overflow = false;
}

uint16_t BytesToReg(uint8_t lo_Byte, uint8_t ho_Byte)
{
	uint16_t result;
	result = (ho_Byte << 8 | lo_Byte);
	return result;
}

uint8_t* RegToBytes(uint16_t reg)
{
	static uint8_t result[2];
	// Lo_Byte
	result[0] = (uint8_t)(reg & 255);
	// Ho_Byte
	result[1] = (uint8_t)(reg >> 8);
	return result;
}

void CalcTimerPeriod(Timer tmr, float_t *period, float_t baudrate, float_t wordLength)
{
	switch(tmr)
	{
	case T15:
	{
		float_t resultOne = 0;

		if(baudrate > 19200)
		{
			resultOne = 750;
		}
		else
		{
			resultOne = (1000000 * (1.5 * (wordLength / baudrate)));
		}

		(*period) = resultOne;

		break;
	}
	case T35:
	{
		float_t resultTwo = 0;

		if(baudrate > 19200)
		{
			resultTwo = 1750;
		}
		else
		{
			resultTwo = (1000000 * (3.5 * (wordLength / baudrate)));
		}

		(*period) = resultTwo;
		break;
	}
	default:
		break;
	}
}

uint16_t CalculateCRC(uint8_t* buffer, uint8_t length)
{
    uint16_t calcCRC, temp, flag;
    uint8_t i;
    uint8_t j;

    calcCRC = 0xFFFF;

    for (i = 0; i < length; i++)
    {
        calcCRC = calcCRC ^ buffer[i];
        for (j = 1; j <= 8; j++)
        {
            flag = calcCRC & 0x0001;
            calcCRC >>= 1;
            if (flag)
                calcCRC ^= 0xA001;
        }
    }

    // Reverse byte order.
    temp = calcCRC >> 8;
    calcCRC = (calcCRC << 8) | temp;
    calcCRC &= 0xFFFF;

    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return calcCRC;
}

void Modify_TIM(TIM_HandleTypeDef *htim , uint32_t prescaler, uint32_t period)
{
    // DeInit timer
	HAL_TIM_Base_DeInit(htim);

	(*htim).Init.Prescaler = prescaler;

	(*htim).Init.Period = period;

	// Init timer
	if (HAL_TIM_Base_Init(htim) == HAL_OK)
	{
		__HAL_TIM_CLEAR_FLAG(htim, TIM_SR_UIF);
	}
	else
	{
		Error_Handler();
	}
}

void Modify_UART(UART_HandleTypeDef *huart, uint32_t baudrate, uint32_t wordLength, uint32_t stopBits, uint32_t parity)
{
	// UART DeInit
	HAL_UART_DeInit(huart);

	(*huart).Init.BaudRate = baudrate;

	(*huart).Init.WordLength = wordLength;

	(*huart).Init.StopBits = stopBits;

	(*huart).Init.Parity = parity;


	// UART Init
	if (HAL_UART_Init(&(*huart)) != HAL_OK)
	{
		Error_Handler();
	}
}

void Set_Coil(uint8_t index, uint8_t value)
{
	if(index>=0 && index<20)
	{
		//CoilsSet[index] = value;
	}
}

void Set_Reg(uint8_t index, uint16_t value)
{
	if(index >= 0 && index<50)
	{
		//RegistersSet[index] = value;
	}
}

uint8_t Get_Coil(uint8_t index)
{
	uint8_t result = 0;

	if(index>=0 && index<20)
	{
		//result = (uint8_t)CoilsSet[index];
	}

	return result;
}

uint16_t Get_Reg(uint8_t index)
{
	uint16_t result = 0;

	if(index >= 0 && index<50)
	{
		//result = (uint16_t)RegistersSet[index];
	}

	return result;
}

void EnableRxMode(MBHandler *mbHandler)
{
	HAL_GPIO_WritePin((*mbHandler).RE_Port, (*mbHandler).RE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin((*mbHandler).DE_Port, (*mbHandler).DE_Pin, GPIO_PIN_RESET);
}

void EnableTxMode(MBHandler *mbHandler)
{
	HAL_GPIO_WritePin((*mbHandler).RE_Port, (*mbHandler).RE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin((*mbHandler).DE_Port, (*mbHandler).DE_Pin, GPIO_PIN_SET);
}

void TransmitXBuffer(MBHandler *mbHandler, XBuffer *xBuffer)
{
	HAL_UART_Abort_IT((*mbHandler).UART_Port);

	EnableTxMode(mbHandler);

	HAL_UART_Transmit((*mbHandler).UART_Port, (*mbHandler).TxBuffer.u8Array, (*mbHandler).TxBuffer.Index,100);

	ClearXBuffer(&(*mbHandler).TxBuffer);

	ClearXBuffer(&(*mbHandler).RxBuffer);

	printMsg("\n");

	EnableRxMode(mbHandler);

	// Restart uart receive interrupt (to receive the next byte of the request)
	if( HAL_UART_Receive_IT((*mbHandler).UART_Port, (*mbHandler).RxBuffer.u8Element, 1) != HAL_OK )
	{
		printMsg("->");
		HandleError(mbHandler, UART_Error);
		printMsg(":\n");
	}
}



// Modbus implementation functions
// Slave
void ProcessMBRequest(MBHandler *mbHandler, XBuffer rxBuffer)
{
	if((*mbHandler).UnitID == rxBuffer.u8Array[0])
	{
		//get received CRC from modbus PDU
		uint8_t rCRCLoB = rxBuffer.u8Array[rxBuffer.Index - 1];
		uint8_t rCRCHoB = rxBuffer.u8Array[rxBuffer.Index - 2];

		uint16_t receivedCRC = BytesToReg(rCRCLoB, rCRCHoB);
		//printMsg("rLoB:%d\trHoB:%d\treceivedCRC:%d\n", rCRCLoB, rCRCHoB, receivedCRC);

		// calculate CRC from the modbus PDU received
		uint16_t calculatedCRC = CalculateCRC(rxBuffer.u8Array, (rxBuffer.Index - 2));

		uint8_t cCRCLoB, cCRCHoB;
		uint8_t* tempCRCPtr = RegToBytes(calculatedCRC);
		cCRCLoB = *tempCRCPtr;
		cCRCHoB = *(tempCRCPtr + 1);

		 //printMsg("calculatedCRC:%d\tcLoB:%d\tcHoB:%d\n",calculatedCRC, cCRCLoB, cCRCHoB);

        if(receivedCRC == calculatedCRC)
        {
        	uint8_t functionCode = rxBuffer.u8Array[1];
        	switch(functionCode)
        	{
        	case MB_FC_Read_Coils:
        		//printMsg("FC:1, Read Coils:%d\n", ProcessFC1(mbHandler, rxBuffer));
        		ProcessFC1(mbHandler, rxBuffer);
        		break;
        	case MB_FC_Read_Registers:
        		//printMsg("FC:3, Read registers:%d\n", ProcessFC3(mbHandler, rxBuffer));
        		ProcessFC3(mbHandler, rxBuffer);
        		break;
        	case MB_FC_Write_Multiple_Coils:
        		//printMsg("FC:15, Write coils:%d\n", ProcessFC15(mbHandler, rxBuffer));
        		ProcessFC15(mbHandler, rxBuffer);
        		break;
        	case MB_FC_Write_Multiple_Registers:
        		//printMsg("FC:16, Write registers:%d\n", ProcessFC16(mbHandler, rxBuffer));
        		ProcessFC16(mbHandler, rxBuffer);
        		break;
        	default:
        		HandleException(mbHandler, Illegal_Function, rxBuffer);
        		break;
        	}
        }
        else
        {
        	HandleError(mbHandler, CRC_Mismatch);
        }
	}
	else
	{
		HandleError(mbHandler, Wrong_Slave_Id);
		printMsg("Wrong slave ID\n");
	}
}

int ProcessFC1(MBHandler *mbHandler, XBuffer rxBuffer)
{
	int result = 0;

	uint8_t slaveID = rxBuffer.u8Array[0];
	uint8_t funcCode  = rxBuffer.u8Array[1];
	uint16_t startAddr = BytesToReg(rxBuffer.u8Array[3], rxBuffer.u8Array[2]);
	uint16_t coilsNr = BytesToReg(rxBuffer.u8Array[5], rxBuffer.u8Array[4]);

	if(coilsNr >= 1 && coilsNr <= (*mbHandler).CoilsData.CoilsLength)
	{
		if(startAddr >= 0 && (startAddr + coilsNr) <= (*mbHandler).CoilsData.CoilsLength)
		{
			uint8_t byteNr = (uint8_t)(coilsNr / 8);
			if((coilsNr % 8) != 0)
				byteNr++;

			ClearXBuffer(&((*mbHandler).TxBuffer));

			AddToXBuffer(&((*mbHandler).TxBuffer), slaveID);

			AddToXBuffer(&((*mbHandler).TxBuffer), funcCode);

			AddToXBuffer(&((*mbHandler).TxBuffer), byteNr);

			uint8_t bitsNr = 0;
			uint8_t currentByte = 0;
			uint8_t currentCoil = 0;

			for(uint16_t coil = 0; coil < coilsNr; coil++)
			{
				currentCoil = coil + startAddr;

				bitWrite(currentByte, bitsNr, *( (((*mbHandler).CoilsData).CoilsArray) + currentCoil));

				bitsNr++;

				if(bitsNr > 7)
				{
					AddToXBuffer(&((*mbHandler).TxBuffer), currentByte);

					currentByte = 0;

					bitsNr = 0;
				}
				else
				{
					if(coil == (coilsNr - 1))
					{
						AddToXBuffer(&((*mbHandler).TxBuffer), currentByte);

						currentByte = 0;

						bitsNr = 0;
					}
				}
			}

			uint16_t calculatedCRC = CalculateCRC((*mbHandler).TxBuffer.u8Array, (*mbHandler).TxBuffer.Index);

			uint8_t* crcPtr = RegToBytes(calculatedCRC);

			AddToXBuffer(&((*mbHandler).TxBuffer), (uint8_t)(*(crcPtr + 1)));
			AddToXBuffer(&((*mbHandler).TxBuffer), (uint8_t)(*crcPtr));

/*			printMsg("->");
			for(int u=0; u<(*mbHandler).TxBuffer.Index ; u++)
			{
				printMsg("\t%u", (*mbHandler).TxBuffer.u8Array[u]);
			}
			printMsg(" :\n");*/

			TransmitXBuffer(mbHandler, &((*mbHandler).TxBuffer));

			result = 1;
		}
		else
		{
			HandleException(mbHandler, Illegal_Data_Address, rxBuffer);
			result = -2;
		}
	}
	else
	{
		HandleException(mbHandler, Illegal_Data_Value, rxBuffer);
		result = -3;
	}

	return result;
}

int ProcessFC15(MBHandler *mbHandler, XBuffer rxBuffer)
{
	int result = 0;

	uint8_t slaveID = rxBuffer.u8Array[0];
	uint8_t funcCode  = rxBuffer.u8Array[1];
	uint16_t startAddr = BytesToReg(rxBuffer.u8Array[3], rxBuffer.u8Array[2]);
	uint16_t coilsNr = BytesToReg(rxBuffer.u8Array[5], rxBuffer.u8Array[4]);
	uint8_t byteCount = rxBuffer.u8Array[6];

	uint8_t dataArray[byteCount];

	for(int i=0; i < byteCount; i++)
	{
		dataArray[i] = rxBuffer.u8Array[7 + i];
	}


	if(coilsNr >=1 && coilsNr <= (*mbHandler).CoilsData.CoilsLength)
	{
		if(startAddr >= 0 && (startAddr + coilsNr) <= (*mbHandler).CoilsData.CoilsLength)
		{
			uint8_t byteIndex = 0;

			uint8_t bitIndex = 0;

			for(int coil = 0; coil < coilsNr; coil++)
			{
				uint8_t currentCoil = startAddr + coil;

				*( ((*mbHandler).CoilsData.CoilsArray) + currentCoil ) = bitRead(dataArray[byteIndex], bitIndex);

				bitIndex++;

				if(bitIndex > 7)
				{
					byteIndex++;

					bitIndex = 0;
				}
				else
				{

				}
			}

			ClearXBuffer(&((*mbHandler).TxBuffer));

			AddToXBuffer(&((*mbHandler).TxBuffer), slaveID);

			AddToXBuffer(&((*mbHandler).TxBuffer), funcCode);

			uint8_t* startAddrPtr = RegToBytes(startAddr);

			AddToXBuffer(&((*mbHandler).TxBuffer), *(startAddrPtr + 1));

			AddToXBuffer(&((*mbHandler).TxBuffer), *(startAddrPtr));

			uint8_t* outQPtr = RegToBytes(coilsNr);

			AddToXBuffer(&((*mbHandler).TxBuffer), *(outQPtr + 1));

			AddToXBuffer(&((*mbHandler).TxBuffer), *(outQPtr));

			uint16_t calculatedCRC = CalculateCRC((*mbHandler).TxBuffer.u8Array, (*mbHandler).TxBuffer.Index);

			uint8_t* crcPtr = RegToBytes(calculatedCRC);

			AddToXBuffer(&((*mbHandler).TxBuffer), *(crcPtr + 1));

			AddToXBuffer(&((*mbHandler).TxBuffer), *(crcPtr));

		/*	printMsg("->");
			for(int u=0; u<(*mbHandler).TxBuffer.Index ; u++)
			{
				printMsg("\t%u", (*mbHandler).TxBuffer.u8Array[u]);
			}
			printMsg(" :\n");*/

			TransmitXBuffer(mbHandler, &((*mbHandler).TxBuffer));

			result = 1;
		}
		else
		{
			HandleException(mbHandler, Illegal_Data_Address, rxBuffer);

			result = -2;
		}
	}
	else
	{
		HandleException(mbHandler, Illegal_Data_Value, rxBuffer);

		result = -3;
	}

	return result;
}

int ProcessFC3(MBHandler *mbHandler, XBuffer rxBuffer)
{
	int result = 0;

	uint8_t slaveID = rxBuffer.u8Array[0];
	uint8_t funcCode = rxBuffer.u8Array[1];
	uint16_t startAddr = BytesToReg(rxBuffer.u8Array[3], rxBuffer.u8Array[2]);
	uint16_t registerNr = BytesToReg(rxBuffer.u8Array[5], rxBuffer.u8Array[4]);

	if(registerNr >=1 && registerNr <= (*mbHandler).RegistersData.RegistersLength)
	{
		if(startAddr >=0 && (startAddr + registerNr) <= (*mbHandler).RegistersData.RegistersLength)
		{
			ClearXBuffer(&((*mbHandler).TxBuffer));

			AddToXBuffer(&((*mbHandler).TxBuffer), slaveID);
			AddToXBuffer(&((*mbHandler).TxBuffer), funcCode);
			AddToXBuffer(&((*mbHandler).TxBuffer), (uint8_t)(2*registerNr));

			for(uint8_t reg=0; reg<registerNr; reg++)
			{
				uint8_t currentReg = reg + startAddr;

				uint16_t tempRegValue = *( ((*mbHandler).RegistersData.RegistersArray) + currentReg);

				uint8_t* tempRegPtr = RegToBytes(tempRegValue);

				AddToXBuffer(&((*mbHandler).TxBuffer), *(tempRegPtr + 1));
				AddToXBuffer(&((*mbHandler).TxBuffer), *(tempRegPtr));
			}

			uint16_t crc = CalculateCRC((*mbHandler).TxBuffer.u8Array, (*mbHandler).TxBuffer.Index);
			uint8_t* crcPtr = RegToBytes(crc);

			AddToXBuffer(&((*mbHandler).TxBuffer), *(crcPtr + 1));
			AddToXBuffer(&((*mbHandler).TxBuffer), *(crcPtr));

/*			printMsg("->");
			for(int u=0; u<(*mbHandler).TxBuffer.Index; u++)
			{
				printMsg("\t%u", (*mbHandler).TxBuffer.u8Array[u]);
			}
			printMsg(" :\n");*/

		    TransmitXBuffer(mbHandler, &((*mbHandler).TxBuffer));

		    result = 1;
		}
		else
		{
			HandleException(mbHandler, Illegal_Data_Address, rxBuffer);
			result = -2;
		}
	}
	else
	{
		HandleException(mbHandler, Illegal_Data_Value, rxBuffer);
		result = -3;
	}

	return result;
}

int ProcessFC16(MBHandler *mbHandler, XBuffer rxBuffer)
{
	int result = 0;

	uint8_t slaveID = rxBuffer.u8Array[0];
	uint8_t funcCode = rxBuffer.u8Array[1];
	uint16_t startAddr = BytesToReg(rxBuffer.u8Array[3], rxBuffer.u8Array[2]);
	uint16_t registerNr = BytesToReg(rxBuffer.u8Array[5], rxBuffer.u8Array[4]);

	uint8_t bytesNr = rxBuffer.u8Array[6];
	uint8_t dataArray[bytesNr];

	for(uint8_t byte=0; byte < bytesNr; byte++)
	{
		dataArray[byte] = rxBuffer.u8Array[7 + byte];
	}

	if(registerNr >= 1 && registerNr <= (*mbHandler).RegistersData.RegistersLength)
	{
		if(startAddr >= 0 && (startAddr + registerNr) <= (*mbHandler).RegistersData.RegistersLength)
		{
			for(uint16_t reg = 0; reg < registerNr; reg++)
			{
				uint16_t currentReg = startAddr + reg;

				*( ((*mbHandler).RegistersData.RegistersArray) + currentReg) = BytesToReg(dataArray[(uint8_t)(2*reg + 1)], dataArray[(uint8_t)(2*reg)]);
			}

			ClearXBuffer(&((*mbHandler).TxBuffer));

			AddToXBuffer(&((*mbHandler).TxBuffer), slaveID);

			AddToXBuffer(&((*mbHandler).TxBuffer), funcCode);

			uint8_t* startAddrPtr = RegToBytes(startAddr);
			AddToXBuffer(&((*mbHandler).TxBuffer), *(startAddrPtr + 1));
			AddToXBuffer(&((*mbHandler).TxBuffer), *(startAddrPtr));

			uint8_t* regNrPtr = RegToBytes(registerNr);
			AddToXBuffer(&((*mbHandler).TxBuffer), *(regNrPtr + 1));
			AddToXBuffer(&((*mbHandler).TxBuffer), *(regNrPtr));

			uint16_t crc = CalculateCRC((*mbHandler).TxBuffer.u8Array, (*mbHandler).TxBuffer.Index);
			uint8_t* crcPtr = RegToBytes(crc);

			AddToXBuffer(&((*mbHandler).TxBuffer), *(crcPtr + 1));
			AddToXBuffer(&((*mbHandler).TxBuffer), *(crcPtr));


/*			printMsg("->");
			for(int u=0; u<(*mbHandler).TxBuffer.Index; u++)
			{
				printMsg("\t%u", (*mbHandler).TxBuffer.u8Array[u]);
			}
			printMsg(" :\n");*/

		    TransmitXBuffer(mbHandler, &((*mbHandler).TxBuffer));

			result = 1;
		}
		else
		{
			HandleException(mbHandler, Illegal_Data_Address, rxBuffer);
			result=-2;
		}
	}
	else
	{
		HandleException(mbHandler, Illegal_Data_Value, rxBuffer);
		result = -3;
	}
	return result;
}


// Master
uint8_t* ReadCoils(MBHandler *mbHandler, uint8_t slaveID,uint16_t startAddress, uint16_t coilsNumber)
{
	(*mbHandler).masterReceivedResp = false;

	uint8_t* outputData = malloc(coilsNumber);

	uint8_t myByteNr = (uint8_t)(coilsNumber / 8);
	if((coilsNumber % 8) != 0)
		myByteNr++;

	ClearXBuffer(&((*mbHandler).TxBuffer));

	AddToXBuffer(&((*mbHandler).TxBuffer), slaveID);
	AddToXBuffer(&((*mbHandler).TxBuffer), 1);

	uint8_t* startAddrPtr = RegToBytes(startAddress);
	AddToXBuffer(&((*mbHandler).TxBuffer), *(startAddrPtr + 1));
	AddToXBuffer(&((*mbHandler).TxBuffer), *(startAddrPtr));

	uint8_t* coilsNumberPtr = RegToBytes(coilsNumber);
	AddToXBuffer(&((*mbHandler).TxBuffer), *(coilsNumberPtr + 1));
	AddToXBuffer(&((*mbHandler).TxBuffer), *(coilsNumberPtr));

	uint16_t crc = 	CalculateCRC((*mbHandler).TxBuffer.u8Array, (*mbHandler).TxBuffer.Index);
	uint8_t* crcPtr = RegToBytes(crc);
	AddToXBuffer(&((*mbHandler).TxBuffer), *(crcPtr + 1));
	AddToXBuffer(&((*mbHandler).TxBuffer), *(crcPtr));

	EnableTxMode(mbHandler);

	TransmitXBuffer(mbHandler, &((*mbHandler).TxBuffer));

	//WaitingResponse(mbHandler);

	while(!(*mbHandler).masterReceivedResp)
	{

	}



	uint16_t responceCRC = BytesToReg((*mbHandler).RxBuffer.u8Array[(*mbHandler).RxBuffer.Index - 1], (*mbHandler).RxBuffer.u8Array[(*mbHandler).RxBuffer.Index - 2]);
	uint16_t calculatedCRC = CalculateCRC((*mbHandler).RxBuffer.u8Array, (*mbHandler).RxBuffer.Index - 2);

	printMsg("cCRC:%d;rCRC;%d\n",calculatedCRC, responceCRC);

	if(responceCRC == calculatedCRC)
	{
		if((*mbHandler).RxBuffer.u8Array[0] == slaveID)
		{
			if((*mbHandler).RxBuffer.u8Array[1] == 1)
			{
				if((*mbHandler).RxBuffer.u8Array[2]==myByteNr)
				{
					uint8_t byteCount = (*mbHandler).RxBuffer.u8Array[2];
					uint8_t dataArray[byteCount];

					for(int i=0; i < byteCount; i++)
					{
						dataArray[i] = (*mbHandler).RxBuffer.u8Array[3 + i];
					}

					uint8_t byteIndex = 0;
					uint8_t bitIndex = 0;
					for(int coil = 0; coil < coilsNumber; coil++)
					{

						*(outputData + coil) = bitRead(dataArray[byteIndex], bitIndex);

						bitIndex++;

						if(bitIndex > 7)
						{
							byteIndex++;

							bitIndex = 0;
						}
						else
						{

						}
					}

					printMsg("->");
					for(int u=0; u<coilsNumber ; u++)
					{
						printMsg("\t%u", outputData[u]);
					}
					printMsg(" :\n");
				}
				else
				{
					printMsg("Response byte number not correct\n");
				}

			}
			else if((*mbHandler).RxBuffer.u8Array[1] == (80 + 1))
			{
				printMsg("Slave responded exception:%d\n",(*mbHandler).RxBuffer.u8Array[2]);
			}
			else
			{
				printMsg("Invaled response function code\n");
			}
		}
		else
		{
			printMsg("Slave responded with wrong slave address\n");
		}
	}
	else
	{
		printMsg("CRC mismatch, response ignored\n");
	}

	return outputData;
}

int WriteCoils(MBHandler *mbHandler, uint8_t slaveID, uint16_t startAddress, uint8_t inputCoilsData[])
{
	int result = 0;

	(*mbHandler).masterReceivedResp = false;

	uint8_t coilsNr = sizeof(inputCoilsData) / sizeof(inputCoilsData[0]);

	uint8_t bytesNr=(uint8_t)(coilsNr / 8);
	if((coilsNr % 8) != 0)
		bytesNr ++;

	ClearXBuffer(&((*mbHandler).TxBuffer));

	AddToXBuffer(&((*mbHandler).TxBuffer), slaveID);

	AddToXBuffer(&((*mbHandler).TxBuffer), 15);

	uint8_t* startAddrPtr = RegToBytes(startAddress);
	AddToXBuffer(&((*mbHandler).TxBuffer), *(startAddrPtr + 1));
	AddToXBuffer(&((*mbHandler).TxBuffer), *(startAddrPtr));

	uint8_t* coilsNrPtr = RegToBytes(coilsNr);
	AddToXBuffer(&((*mbHandler).TxBuffer), *(coilsNrPtr + 1));
	AddToXBuffer(&((*mbHandler).TxBuffer), *(coilsNrPtr));

	AddToXBuffer(&((*mbHandler).TxBuffer), bytesNr);

	uint8_t bitsNr = 0;
	uint8_t currentByte = 0;

	for(uint16_t coil=0; coil < coilsNr; coil++)
	{
		bitWrite(currentByte,bitsNr, inputCoilsData[coil]);
		bitsNr ++;

		if(bitsNr > 7)
		{
			AddToXBuffer(&((*mbHandler).TxBuffer), currentByte);

			currentByte = 0;

			bitsNr = 0;
		}
		else
		{
			if(coil == (coilsNr - 1))
			{
				AddToXBuffer(&((*mbHandler).TxBuffer), currentByte);

				currentByte = 0;

				bitsNr = 0;
			}
		}
	}

	uint16_t crc = 	CalculateCRC((*mbHandler).TxBuffer.u8Array, (*mbHandler).TxBuffer.Index);
	uint8_t* crcPtr = RegToBytes(crc);
	AddToXBuffer(&((*mbHandler).TxBuffer), *(crcPtr + 1));
	AddToXBuffer(&((*mbHandler).TxBuffer), *(crcPtr));


	EnableTxMode(mbHandler);

	TransmitXBuffer(mbHandler, &((*mbHandler).TxBuffer));

	//WaitingResponse(mbHandler);

	while(!(*mbHandler).masterReceivedResp)
	{

	}

	uint16_t responseCRC = BytesToReg((*mbHandler).RxBuffer.u8Array[(*mbHandler).RxBuffer.Index - 1], (*mbHandler).RxBuffer.u8Array[(*mbHandler).RxBuffer.Index - 2]);
	uint16_t calculatedCRC = CalculateCRC((*mbHandler).RxBuffer.u8Array, (*mbHandler).RxBuffer.Index - 2);

	printMsg("cCRC:%d;rCRC;%d\n",calculatedCRC, responseCRC);

	if(responseCRC == calculatedCRC)
	{
		if((*mbHandler).RxBuffer.u8Array[0] == slaveID)
		{
			if((*mbHandler).RxBuffer.u8Array[1] == 15)
			{
				uint16_t respStartAddr = BytesToReg((*mbHandler).RxBuffer.u8Array[3], (*mbHandler).RxBuffer.u8Array[2]) ;

				if(respStartAddr == startAddress)
				{
					uint16_t respCoilsNr = BytesToReg((*mbHandler).RxBuffer.u8Array[5], (*mbHandler).RxBuffer.u8Array[4]);
					if(respCoilsNr == coilsNr)
					{
						result = 1;
						printMsg("Write Coils status :success\n");
					}
					else
					{
						printMsg("Slave responded different coils number\n");
						result = -1;
					}
				}
				else
				{
					printMsg("Slave responded different start address\n");
					result = -1;
				}

			}
			else if((*mbHandler).RxBuffer.u8Array[1] == (80 + 15))
			{
				// code to write

				printMsg("Slave responded exception:%d\n",(*mbHandler).RxBuffer.u8Array[2]);
			}
			else
			{
				printMsg("Invaled response function code\n");
				result = -1;
			}
		}
		else
		{
			printMsg("Slave responded with wrong slave address\n");
			result = -1;
		}
	}
	else
	{
		printMsg("CRC mismatch, response ignored\n");
		result = -1;
	}

	return result;
}

int ReadRegisters(MBHandler *mbHandler, uint8_t slaveID, uint16_t startAddress, uint16_t regsNumber, uint16_t* regsOutData)
{
	int result = 0;

	(*mbHandler).masterReceivedResp = false;

	ClearXBuffer(&((*mbHandler).TxBuffer));

	AddToXBuffer(&((*mbHandler).TxBuffer), slaveID);





	return result;
}

int WriteRegisters(MBHandler *mbHandler, uint8_t slaveID, uint16_t startAddress,uint16_t inputRegsData[])
{
	return 1;
}

// General
void HandleError(MBHandler *mbHandler, Error error)
{
	switch(error)
	{
	case 0:
	{
		printMsg("uart error\n");
		break;
	}
	case 1:
	{
		printMsg("T35 error\n");
		break;
	}
	case 2:
	{
		printMsg("T15 error\n");
		break;
	}
	case 3:
	{
		printMsg("Receiving frame error\n");
		break;
	}
	case 4:
	{
		printMsg("CRC mismatch\n");
		break;
	}
	case 5:
	{
		printMsg("Wrong slave id\n");
		break;
	}
	default:
		break;
	}
}

void HandleException(MBHandler *mbHandler, Exception exeption, XBuffer rxBuffer)
{
	switch(exeption)
	{
	case 1:
	{
		printMsg("Exeption:%d\n",exeption);

		uint8_t slaveId = (uint8_t)rxBuffer.u8Array[0];
		uint8_t excFuncCode = (uint8_t)(rxBuffer.u8Array[1] + 80);

		AddToXBuffer(&((*mbHandler).ExpBuffer), slaveId);
		AddToXBuffer(&((*mbHandler).ExpBuffer), excFuncCode);
		AddToXBuffer(&((*mbHandler).ExpBuffer), (uint8_t)1);

		uint16_t calculatedCRC = CalculateCRC(((*mbHandler).ExpBuffer).u8Array, ((*mbHandler).ExpBuffer).Index);
		uint8_t* crcPtr = RegToBytes(calculatedCRC);

		AddToXBuffer(&((*mbHandler).ExpBuffer), *(crcPtr + 1));
		AddToXBuffer(&((*mbHandler).ExpBuffer), *crcPtr);

		TransmitXBuffer(mbHandler, &((*mbHandler).ExpBuffer));

		printMsg("Illegal function\n ");
		break;
	}
	case 2:
	{
		printMsg("Exeption:%d\n",exeption);

		uint8_t slaveId = (uint8_t)rxBuffer.u8Array[0];
		uint8_t excFuncCode = (uint8_t)(rxBuffer.u8Array[1] + 80);

		AddToXBuffer(&((*mbHandler).ExpBuffer), slaveId);
		AddToXBuffer(&((*mbHandler).ExpBuffer), excFuncCode);
		AddToXBuffer(&((*mbHandler).ExpBuffer), (uint8_t)2);

		uint16_t calculatedCRC = CalculateCRC(((*mbHandler).ExpBuffer).u8Array, ((*mbHandler).ExpBuffer).Index);
		uint8_t* crcPtr = RegToBytes(calculatedCRC);

		AddToXBuffer(&((*mbHandler).ExpBuffer), *(crcPtr + 1));
		AddToXBuffer(&((*mbHandler).ExpBuffer), *crcPtr);

		TransmitXBuffer(mbHandler, &((*mbHandler).ExpBuffer));

		printMsg("Illegal data address\n");
		break;
	}
	case 3:
	{
		printMsg("Exeption:%d\n",exeption);

		uint8_t slaveId = (uint8_t)rxBuffer.u8Array[0];
		uint8_t excFuncCode = (uint8_t)(rxBuffer.u8Array[1] + 80);

		AddToXBuffer(&((*mbHandler).ExpBuffer), slaveId);
		AddToXBuffer(&((*mbHandler).ExpBuffer), excFuncCode);
		AddToXBuffer(&((*mbHandler).ExpBuffer), (uint8_t)3);

		uint16_t calculatedCRC = CalculateCRC(((*mbHandler).ExpBuffer).u8Array, ((*mbHandler).ExpBuffer).Index);
		uint8_t* crcPtr = RegToBytes(calculatedCRC);

		AddToXBuffer(&((*mbHandler).ExpBuffer), *(crcPtr + 1));
		AddToXBuffer(&((*mbHandler).ExpBuffer), *crcPtr);

		TransmitXBuffer(mbHandler, &((*mbHandler).ExpBuffer));


		printMsg("Illegal data value\n");
		break;
	}
	case 4:
	{
		printMsg("Exeption:%d\n",exeption);

		uint8_t slaveId = (uint8_t)rxBuffer.u8Array[0];
		uint8_t excFuncCode = (uint8_t)(rxBuffer.u8Array[1] + 80);

		AddToXBuffer(&((*mbHandler).ExpBuffer), slaveId);
		AddToXBuffer(&((*mbHandler).ExpBuffer), excFuncCode);
		AddToXBuffer(&((*mbHandler).ExpBuffer), (uint8_t)4);

		uint16_t calculatedCRC = CalculateCRC(((*mbHandler).ExpBuffer).u8Array, ((*mbHandler).ExpBuffer).Index);
		uint8_t* crcPtr = RegToBytes(calculatedCRC);

		AddToXBuffer(&((*mbHandler).ExpBuffer), *(crcPtr + 1));
		AddToXBuffer(&((*mbHandler).ExpBuffer), *crcPtr);

		TransmitXBuffer(mbHandler, &((*mbHandler).ExpBuffer));

		printMsg("Server device failure\n ");
		break;
	}
	case 5:
	{
		printMsg("Exeption:%d\n",exeption);

		uint8_t slaveId = (uint8_t)rxBuffer.u8Array[0];
		uint8_t excFuncCode = (uint8_t)(rxBuffer.u8Array[1] + 80);

		AddToXBuffer(&((*mbHandler).ExpBuffer), slaveId);
		AddToXBuffer(&((*mbHandler).ExpBuffer), excFuncCode);
		AddToXBuffer(&((*mbHandler).ExpBuffer), (uint8_t)5);

		uint16_t calculatedCRC = CalculateCRC(((*mbHandler).ExpBuffer).u8Array, ((*mbHandler).ExpBuffer).Index);
		uint8_t* crcPtr = RegToBytes(calculatedCRC);

		AddToXBuffer(&((*mbHandler).ExpBuffer), *(crcPtr + 1));
		AddToXBuffer(&((*mbHandler).ExpBuffer), *crcPtr);

		TransmitXBuffer(mbHandler, &((*mbHandler).ExpBuffer));


		printMsg("Acknowledge\n");
		break;
	}
	case 6:
	{
		printMsg("Exeption:%d\n",exeption);

		uint8_t slaveId = (uint8_t)rxBuffer.u8Array[0];
		uint8_t excFuncCode = (uint8_t)(rxBuffer.u8Array[1] + 80);

		AddToXBuffer(&((*mbHandler).ExpBuffer), slaveId);
		AddToXBuffer(&((*mbHandler).ExpBuffer), excFuncCode);
		AddToXBuffer(&((*mbHandler).ExpBuffer), (uint8_t)6);

		uint16_t calculatedCRC = CalculateCRC(((*mbHandler).ExpBuffer).u8Array, ((*mbHandler).ExpBuffer).Index);
		uint8_t* crcPtr = RegToBytes(calculatedCRC);

		AddToXBuffer(&((*mbHandler).ExpBuffer), *(crcPtr + 1));
		AddToXBuffer(&((*mbHandler).ExpBuffer), *crcPtr);

		TransmitXBuffer(mbHandler, &((*mbHandler).ExpBuffer));


		printMsg("Server_Device_Busy\n");
		break;
	}
	default:
		break;
	}
}











