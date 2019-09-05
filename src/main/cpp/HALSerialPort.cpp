#include "HALSerialPort.h"

#include <string>
#include "hal/cpp/SerialHelper.h"
#include "hal/SerialPort.h"
#include "hal/handles/HandlesInternal.h"
#include "hal/handles/IndexedHandleResource.h"
#include <iostream>

#include <stdio.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

namespace
{
struct SerialPort
{
    int portId;
    struct termios tty;
    int baudRate;

    int readBufferSize = 0;
    int writeBufferSize = 0;

    double timeout = 0;

    bool termination = false;
    char terminationChar = '\n';
};
} // namespace

namespace hal
{
IndexedHandleResource<HAL_SerialPortHandle, SerialPort, 4,
                      HAL_HandleEnum::Vendor> *serialPortHandles;
} // namespace hal

namespace hal
{
namespace init
{
void InitializeSerialPort2()
{
    static IndexedHandleResource<HAL_SerialPortHandle, SerialPort,
                                 4, HAL_HandleEnum::Vendor>
        spH;
    serialPortHandles = &spH;
}
} // namespace init
} // namespace hal

using namespace hal;

extern "C"
{
HAL_SerialPortHandle HAL2_InitializeSerialPort(HAL_SerialPort port, int32_t *status)
{
    //hal::init::CheckInit();

    hal::SerialHelper serialHelper;

    std::string portName = serialHelper.GetOSSerialPortName(port, status);

    if (*status < 0)
    {
        return HAL_kInvalidHandle;
    }

    return HAL2_InitializeSerialPortDirect(port, portName.c_str(), status);
}
HAL_SerialPortHandle HAL2_InitializeSerialPortDirect(HAL_SerialPort port, const char *portName, int32_t *status)
{
    auto handle = serialPortHandles->Allocate(static_cast<int16_t>(port), status);

    if (*status != 0)
    {
        return HAL_kInvalidHandle;
    }

    auto serialPort = serialPortHandles->Get(handle);

    if (serialPort == nullptr)
    {
        *status = HAL_HANDLE_ERROR;
        return HAL_kInvalidHandle;
    }

    std::cout << "Port Name " << portName << std::endl;

    serialPort->portId = open(portName, O_RDWR | O_NOCTTY);
    if (serialPort->portId < 0)
    {
        std::cout << "Failed to open port " << serialPort->portId << " " << errno << std::endl;
        *status = HAL_SERIAL_PORT_OPEN_ERROR;
        serialPortHandles->Free(handle);
        return HAL_kInvalidHandle;
    }

    std::memset(&serialPort->tty, 0, sizeof(serialPort->tty));

    serialPort->baudRate = B9600;
    cfsetospeed(&serialPort->tty, static_cast<speed_t>(serialPort->baudRate));
    cfsetispeed(&serialPort->tty, static_cast<speed_t>(serialPort->baudRate));

    serialPort->tty.c_cflag &= ~PARENB;
    serialPort->tty.c_cflag &= ~CSTOPB;
    serialPort->tty.c_cflag &= ~CSIZE;
    serialPort->tty.c_cflag |= CS8;

    serialPort->tty.c_cc[VMIN] = 0;
    serialPort->tty.c_cc[VTIME] = 10;

    serialPort->tty.c_cflag |= CREAD | CLOCAL;

    serialPort->tty.c_lflag &= ~(ICANON | ECHO | ISIG);
    /* Raw output mode, sends the raw and unprocessed data  ( send as it is).
    * If it is in canonical mode and sending new line char then CR
    * will be added as prefix and send as CR LF
    */
    serialPort->tty.c_oflag = ~OPOST;

    tcflush(serialPort->portId, TCIOFLUSH);
    if (tcsetattr(serialPort->portId, TCSANOW, &serialPort->tty) != 0)
    {
        close(serialPort->portId);
        *status = HAL_SERIAL_PORT_OPEN_ERROR;
        serialPortHandles->Free(handle);
        return HAL_kInvalidHandle;
    }
    return handle;
}

void HAL2_CloseSerial(HAL_SerialPortHandle handle, int32_t *status)
{
    auto port = serialPortHandles->Get(handle);
    serialPortHandles->Free(handle);

    if (port)
    {
        close(port->portId);
    }
    *status = 0;
}

void HAL2_SetSerialBaudRate(HAL_SerialPortHandle handle, int32_t baud, int32_t *status)
{
    int baudRate = -1;
    switch (baud)
    {
    case 9600:
        baudRate = B9600;
        break;
    case 19200:
        baudRate = B19200;
        break;
    case 38400:
        baudRate = B38400;
        break;
    case 57600:
        baudRate = B57600;
        break;
    case 115200:
        baudRate = B115200;
        break;
    default:
        *status = PARAMETER_OUT_OF_RANGE;
        return;
    }
}

void HAL2_SetSerialDataBits(HAL_SerialPortHandle handle, int32_t bits, int32_t *status)
{
}

void HAL2_SetSerialParity(HAL_SerialPortHandle handle, int32_t parity, int32_t *status)
{
}

void HAL2_SetSerialStopBits(HAL_SerialPortHandle handle, int32_t stopBits, int32_t *status)
{
}

void HAL2_SetSerialWriteMode(HAL_SerialPortHandle handle, int32_t mode, int32_t *status)
{
}

void HAL2_SetSerialFlowControl(HAL_SerialPortHandle handle, int32_t flow, int32_t *status)
{
}

void HAL2_SetSerialTimeout(HAL_SerialPortHandle handle, double timeout, int32_t *status)
{
    auto port = serialPortHandles->Get(handle);
    if (!port)
    {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    port->timeout = timeout;
    port->tty.c_cc[VTIME] = static_cast<int>(timeout * 10);
    tcsetattr(port->portId, TCSANOW, &port->tty);
}

void HAL2_EnableSerialTermination(HAL_SerialPortHandle handle, char terminator, int32_t *status)
{
    auto port = serialPortHandles->Get(handle);
    if (!port)
    {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    port->termination = true;
    port->terminationChar = terminator;
}

void HAL2_DisableSerialTermination(HAL_SerialPortHandle handle, int32_t *status)
{
    auto port = serialPortHandles->Get(handle);
    if (!port)
    {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    port->termination = false;
}

void HAL2_SetSerialReadBufferSize(HAL_SerialPortHandle handle, int32_t size, int32_t *status)
{
    auto port = serialPortHandles->Get(handle);
    if (!port)
    {
        *status = HAL_HANDLE_ERROR;
        return;
    }

    port->readBufferSize = size;
}

void HAL2_SetSerialWriteBufferSize(HAL_SerialPortHandle handle, int32_t size, int32_t *status)
{
    auto port = serialPortHandles->Get(handle);
    if (!port)
    {
        *status = HAL_HANDLE_ERROR;
        return;
    }

    port->writeBufferSize = size;
}

int32_t HAL2_GetSerialBytesReceived(HAL_SerialPortHandle handle, int32_t *status)
{
    auto port = serialPortHandles->Get(handle);
    if (!port)
    {
        *status = HAL_HANDLE_ERROR;
        return -1;
    }
    int bytes;
    ioctl(port->portId, FIONREAD, &bytes);
    return bytes;
}

int32_t HAL2_ReadSerial(HAL_SerialPortHandle handle, char *buffer, int32_t count, int32_t *status)
{
    // Don't do anything if 0 bytes were requested
    if (count == 0)
        return 0;

    auto port = serialPortHandles->Get(handle);
    if (!port)
    {
        *status = HAL_HANDLE_ERROR;
        return -1;
    }

    int n = 0, loc = 0;
    char buf = '\0';
    memset(buffer, '\0', count);
    *status = 0;

    do
    {
        n = read(port->portId, &buf, 1);
        if (n == 1)
        {
            buffer[loc] = buf;
            loc++;
            // If buffer is full, return
            if (loc == count)
                return loc;
            // If terminating, and termination was hit return;
            if (port->termination && buf == port->terminationChar)
                return loc;
        }
        else
        {
            // If nothing read, timeout
            return loc;
        }
    } while (true);
}

int32_t HAL2_WriteSerial(HAL_SerialPortHandle handle, const char *buffer, int32_t count, int32_t *status)
{
    auto port = serialPortHandles->Get(handle);
    if (!port)
    {
        *status = HAL_HANDLE_ERROR;
        return -1;
    }

    // TODO Determine if we need to handle termination on write

    int written = 0, spot = 0;
    do
    {
        written = write(port->portId, buffer + spot, count - spot);
        if (written > 0)
        {
            spot += written;
        }
    } while (spot < count);
}

void HAL2_FlushSerial(HAL_SerialPortHandle handle, int32_t *status)
{
    auto port = serialPortHandles->Get(handle);
    if (!port)
    {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    tcflush(port->portId, TCOFLUSH);
}
void HAL2_ClearSerial(HAL_SerialPortHandle handle, int32_t *status)
{
    auto port = serialPortHandles->Get(handle);
    if (!port)
    {
        *status = HAL_HANDLE_ERROR;
        return;
    }
    tcflush(port->portId, TCIFLUSH);
}
}