/*
    Modbus Tools

    Created: 2023
    Author: Serhii Marchuk, https://github.com/serhmarch

    Copyright (C) 2023  Serhii Marchuk

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include "server_rundevice.h"

#include <project/server_device.h>

mbServerRunDevice::mbServerRunDevice(mbServerPort *port)
{
    const Modbus::Defaults &d = Modbus::Defaults::instance();

    m_port = port;
    m_settings.isBroadcastEnabled = d.isBroadcastEnabled;
    memset(m_units, 0, sizeof(m_units));
    m_timestamp = 0;
}

mbServerRunDevice::~mbServerRunDevice()
{
}

#define CHECK_DELAY                                                 \
    uint delay = device->delay();                                   \
    if (delay > 0)                                                  \
    {                                                               \
        if (m_timestamp == 0)                                       \
        {                                                           \
            m_timestamp = mb::currentTimestamp();                   \
            return Modbus::Status_Processing;                       \
        }                                                           \
        if ((mb::currentTimestamp()-m_timestamp) < delay)           \
            return Modbus::Status_Processing;                       \
        m_timestamp = 0; /* Note: clear timestamp for next use */   \
    }

Modbus::StatusCode mbServerRunDevice::readCoils(uint8_t unit, uint16_t offset, uint16_t count, void *values)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->readCoils(offset, count, values);
    }
}

Modbus::StatusCode mbServerRunDevice::readDiscreteInputs(uint8_t unit, uint16_t offset, uint16_t count, void *values)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->readDiscreteInputs(offset, count, values);
    }
}

Modbus::StatusCode mbServerRunDevice::readHoldingRegisters(uint8_t unit, uint16_t offset, uint16_t count, uint16_t *values)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->readHoldingRegisters(offset, count, values);
    }
}

Modbus::StatusCode mbServerRunDevice::readInputRegisters(uint8_t unit, uint16_t offset, uint16_t count, uint16_t *values)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->readInputRegisters(offset, count, values);
    }
}

Modbus::StatusCode mbServerRunDevice::writeSingleCoil(uint8_t unit, uint16_t offset, bool value)
{
    if (isBroadcast(unit))
    {
        Q_FOREACH (mbServerDevice *device, m_devices)
            device->writeSingleCoil(offset, value);
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->writeSingleCoil(offset, value);
    }
}

Modbus::StatusCode mbServerRunDevice::writeSingleRegister(uint8_t unit, uint16_t offset, uint16_t value)
{
    if (isBroadcast(unit))
    {
        Q_FOREACH (mbServerDevice *device, m_devices)
            device->writeSingleRegister(offset, value);
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->writeSingleRegister(offset, value);
    }
}

Modbus::StatusCode mbServerRunDevice::readExceptionStatus(uint8_t unit, uint8_t *status)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->readExceptionStatus(status);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsReturnQueryData(uint8_t unit, uint8_t insize, const void *indata, uint8_t *outsize, void *outdata)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsReturnQueryData(insize, indata, outsize, outdata);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsRestartCommunicationsOption(uint8_t unit, bool clearEventLog)
{
    if (isBroadcast(unit))
    {
        Q_FOREACH (mbServerDevice *device, m_devices)
            device->diagnosticsRestartCommunicationsOption(clearEventLog);
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsRestartCommunicationsOption(clearEventLog);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsReturnDiagnosticRegister(uint8_t unit, uint16_t *value)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsReturnDiagnosticRegister(value);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsChangeAsciiInputDelimiter(uint8_t unit, char delimiter)
{
    if (isBroadcast(unit))
    {
        Q_FOREACH (mbServerDevice *device, m_devices)
            device->diagnosticsChangeAsciiInputDelimiter(delimiter);
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsChangeAsciiInputDelimiter(delimiter);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsForceListenOnlyMode(uint8_t unit)
{
    if (isBroadcast(unit))
    {
        Q_FOREACH (mbServerDevice *device, m_devices)
            device->diagnosticsForceListenOnlyMode();
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsForceListenOnlyMode();
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsClearCountersAndDiagnosticRegister(uint8_t unit)
{
    if (isBroadcast(unit))
    {
        Q_FOREACH (mbServerDevice *device, m_devices)
            device->diagnosticsClearCountersAndDiagnosticRegister();
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsClearCountersAndDiagnosticRegister();
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsReturnBusMessageCount(uint8_t unit, uint16_t *count)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsReturnBusMessageCount(m_port, count);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsReturnBusCommunicationErrorCount(uint8_t unit, uint16_t *count)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsReturnBusCommunicationErrorCount(m_port, count);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsReturnBusExceptionErrorCount(uint8_t unit, uint16_t *count)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsReturnBusExceptionErrorCount(count);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsReturnServerMessageCount(uint8_t unit, uint16_t *count)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsReturnServerMessageCount(count);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsReturnServerNoResponseCount(uint8_t unit, uint16_t *count)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsReturnServerNoResponseCount(count);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsReturnServerNAKCount(uint8_t unit, uint16_t *count)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsReturnServerNAKCount(count);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsReturnServerBusyCount(uint8_t unit, uint16_t *count)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsReturnServerBusyCount(count);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsReturnBusCharacterOverrunCount(uint8_t unit, uint16_t *count)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsReturnBusCharacterOverrunCount(count);
    }
}

Modbus::StatusCode mbServerRunDevice::diagnosticsClearOverrunCounterAndFlag(uint8_t unit)
{
    if (isBroadcast(unit))
    {
        Q_FOREACH (mbServerDevice *device, m_devices)
            device->diagnosticsClearOverrunCounterAndFlag();
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->diagnosticsClearOverrunCounterAndFlag();
    }
}

Modbus::StatusCode mbServerRunDevice::writeMultipleCoils(uint8_t unit, uint16_t offset, uint16_t count, const void *values)
{
    if (isBroadcast(unit))
    {
        Q_FOREACH (mbServerDevice *device, m_devices)
            device->writeMultipleCoils(offset, count, values);
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->writeMultipleCoils(offset, count, values);
    }
}

Modbus::StatusCode mbServerRunDevice::writeMultipleRegisters(uint8_t unit, uint16_t offset, uint16_t count, const uint16_t *values)
{
    if (isBroadcast(unit))
    {
        Q_FOREACH (mbServerDevice *device, m_devices)
            device->writeMultipleRegisters(offset, count, values);
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->writeMultipleRegisters(offset, count, values);
    }
}

Modbus::StatusCode mbServerRunDevice::reportServerID(uint8_t unit, uint8_t *count, uint8_t *data)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->reportServerID(count, data);
    }
}

Modbus::StatusCode mbServerRunDevice::maskWriteRegister(uint8_t unit, uint16_t offset, uint16_t andMask, uint16_t orMask)
{
    if (isBroadcast(unit))
    {
        Q_FOREACH (mbServerDevice *device, m_devices)
            device->maskWriteRegister(offset, andMask, orMask);
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->maskWriteRegister(offset, andMask, orMask);
    }
}

Modbus::StatusCode mbServerRunDevice::readWriteMultipleRegisters(uint8_t unit, uint16_t readOffset, uint16_t readCount, uint16_t *readValues, uint16_t writeOffset, uint16_t writeCount, const uint16_t *writeValues)
{
    if (isBroadcast(unit))
    {
        // Note: No need to fill read buffer and return it to client in broadcast mode.
        //       So use `writeMultipleRegisters`-part only.
        Q_FOREACH (mbServerDevice *device, m_devices)
            device->writeMultipleRegisters(writeOffset, writeCount, writeValues);
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->readWriteMultipleRegisters(readOffset, readCount, readValues, writeOffset, writeCount, writeValues);
    }
}

Modbus::StatusCode mbServerRunDevice::readDeviceIdentification(uint8_t unit, uint8_t readDeviceId, uint8_t objectId, uint8_t *dataSize, void *data, uint8_t *numberOfObjects, uint8_t *conformityLevel, bool *moreFollows, uint8_t *nextObjectId)
{
    if (isBroadcast(unit))
    {
        return Modbus::Status_Good;
    }
    else
    {
        mbServerDevice *device = this->device(unit);
        if (!device)
            return Modbus::Status_BadGatewayPathUnavailable;
        CHECK_DELAY
        return device->readDeviceIdentification(readDeviceId, objectId, dataSize, data, numberOfObjects, conformityLevel, moreFollows, nextObjectId);
    }
}

void mbServerRunDevice::setDevice(uint8_t unit, mbServerDevice *device)
{
    m_units[unit] = device;
    m_unitNumbers.insert(unit);
    m_devices.insert(device);
}
