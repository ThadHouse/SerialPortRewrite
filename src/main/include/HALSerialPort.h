#pragma once


#include "hal/SerialPort.h"

typedef HAL_Handle HAL_SerialPortHandle;

extern "C" {
HAL_SerialPortHandle HAL2_InitializeSerialPort(HAL_SerialPort port, int32_t* status);
HAL_SerialPortHandle HAL2_InitializeSerialPortDirect(HAL_SerialPort port, const char* portName, int32_t* status);
void HAL2_CloseSerial(HAL_SerialPortHandle handle, int32_t* status);

void HAL2_SetSerialBaudRate(HAL_SerialPortHandle handle, int32_t baud, int32_t* status);

void HAL2_SetSerialDataBits(HAL_SerialPortHandle handle, int32_t bits, int32_t* status);

void HAL2_SetSerialParity(HAL_SerialPortHandle handle, int32_t parity, int32_t* status);

void HAL2_SetSerialStopBits(HAL_SerialPortHandle handle, int32_t stopBits, int32_t* status);

void HAL2_SetSerialWriteMode(HAL_SerialPortHandle handle, int32_t mode, int32_t* status);

void HAL2_SetSerialFlowControl(HAL_SerialPortHandle handle, int32_t flow, int32_t* status);

void HAL2_SetSerialTimeout(HAL_SerialPortHandle handle, double timeout, int32_t* status);

void HAL2_EnableSerialTermination(HAL_SerialPortHandle handle, char terminator, int32_t* status);

void HAL2_DisableSerialTermination(HAL_SerialPortHandle handle, int32_t* status);

void HAL2_SetSerialReadBufferSize(HAL_SerialPortHandle handle, int32_t size, int32_t* status);

void HAL2_SetSerialWriteBufferSize(HAL_SerialPortHandle handle, int32_t size, int32_t* status);

int32_t HAL2_GetSerialBytesReceived(HAL_SerialPortHandle handle, int32_t* status);

int32_t HAL2_ReadSerial(HAL_SerialPortHandle handle, char* buffer, int32_t count, int32_t* status);

int32_t HAL2_WriteSerial(HAL_SerialPortHandle handle, const char* buffer, int32_t count, int32_t* status);

void HAL2_FlushSerial(HAL_SerialPortHandle handle, int32_t* status);
void HAL2_ClearSerial(HAL_SerialPortHandle handle, int32_t* status);

}