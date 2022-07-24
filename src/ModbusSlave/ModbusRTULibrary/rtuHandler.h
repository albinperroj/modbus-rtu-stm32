/*
 * rtuHandler.h
 *
 *  Created on: Jun 22, 2021
 *      Author: Albin Perroj
 */

#ifndef RTUHANDLER_H_
#define RTUHANDLER_H_

/*--------------------------------------------------------
 * My includes, "most of them are in 'mbslavehelp.h' header file"
 */
#include "rtuHandlerHelp.h"


/*--------------------------------------------------------
 * Custom data types
 */

typedef enum
{
	MB_FC_Read_Coils                   = 1,  // MB_FC=1 -> read coils or digital outputs
	MB_FC_Read_Discrete_input          = 2,  // MB_FC=2 -> read digital inputs
	MB_FC_Read_Registers               = 3,  // MB_FC=3 -> read registers or analog outputs
	MB_FC_Read_Input_Register          = 4,  // MB_FC=4 -> read analog outputs
	MB_FC_Write_Coil                   = 5,  // MB_FC=5 -> write single coil or digital output
	MB_FC_Write_Register               = 6,  // MB_FC=6 -> write single register
	MB_FC_Write_Multiple_Coils         = 15, // MB_FC=15-> write multiple coils or outputs
	MB_FC_Write_Multiple_Registers     = 16, // MB_FC=1 -> write multiple registers
	MB_FC_Read_Write_Multiple_Registers= 23  // MB_FC=23 ->read / write multiple registers
} modbusFC;


typedef enum
{
	Illegal_Function = 1,
	Illegal_Data_Address = 2,
	Illegal_Data_Value = 3,
	Slave_Device_Failure = 4,
	Acknowledge = 5,
	Slave_Device_Busy = 6
} Exception;

typedef enum
{
	UART_Error = 0,
	T35_Error = 1,
	T15_Error = 2,
	Rx_Frame_Error = 3,
	Tx_Frame_Error = 4,
	CRC_Mismatch = 5,
	Wrong_Slave_Id = 6
} Error;

typedef enum
{
	Receive_Mode = 1,
	Transmit_Mode = 2
} RxOrTxState;

typedef struct
{
	uint8_t u8Array[XBuffer_MAX];
	uint8_t u8Element[1];
	uint8_t Index;
    int16_t T15Counter;
    bool overflow;
} XBuffer;

typedef enum
{
	Slave = 1,
	Master =2
}UnitType;

typedef enum
{
	T15 = 1,
	T35 = 2
}Timer;

typedef struct
{
	uint8_t* CoilsArray;
	uint16_t CoilsLength;
}Coils;

typedef struct
{
	uint16_t* RegistersArray;
	uint16_t RegistersLength;
}Registers;

typedef struct
{
	uint8_t UnitID;

	UART_HandleTypeDef *UART_Port;

	GPIO_TypeDef *RE_Port;
	uint16_t RE_Pin;

	GPIO_TypeDef *DE_Port;
	uint16_t DE_Pin;

	TIM_HandleTypeDef *T35;

	TIM_HandleTypeDef *T15;

	XBuffer RxBuffer;

	XBuffer TxBuffer;

	XBuffer ExpBuffer;

	Coils CoilsData;

	Registers RegistersData;

	bool masterReceivedResp;

} MBHandler;


/*--------------------------------------------------------
 * Functions
 */


void InitMBSlave(uint8_t id, MBHandler *mbHandler, UART_HandleTypeDef *uartPort,
		TIM_HandleTypeDef *timerT35, TIM_HandleTypeDef *timerT15,
		uint32_t prescaler, uint32_t baudrate, uint32_t wordLength,
		uint32_t stopBits, uint32_t parity,
		GPIO_TypeDef *RE_Port, uint16_t RE_Pin,
		GPIO_TypeDef *DE_Port, uint16_t DE_Pin,
		uint8_t* coilsArray, uint16_t coilsArrLength,
		uint16_t* registersArray, uint16_t registersArrLength);

void StartMBSlave(MBHandler *mbHandler);

void InitMBMaster(MBHandler *mbHandler, UART_HandleTypeDef *uartPort,
		TIM_HandleTypeDef *timerT35, TIM_HandleTypeDef *timerT15,
		uint32_t prescaler, uint32_t baudrate, uint32_t wordLength,
		uint32_t stopBits, uint32_t parity,
		GPIO_TypeDef *RE_Port, uint16_t RE_Pin,
		GPIO_TypeDef *DE_Port, uint16_t DE_Pin);

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
void HandleException(MBHandler *mbHandler, Exception exeption, XBuffer rxBuffer);

void HandleError(MBHandler *mbHandler, Error error);




#endif /* RTUHANDLER_H_ */
