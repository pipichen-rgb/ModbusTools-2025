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
#ifndef CLIENT_GLOBAL_H
#define CLIENT_GLOBAL_H

#include <mbcore.h>
#include <core_global.h>

class mbClientPort;
class mbClientDevice;
class mbClientDataViewItem;
class mbClientRunMessage;
class mbClientRunMessageRaw;

typedef mb::SharedPointer<mbClientRunMessage> mbClientRunMessagePtr;
typedef mb::SharedPointer<mbClientRunMessageRaw> mbClientRunMessageRawPtr;

struct mbClientMessageParams
{
    mbClientMessageParams()
    {
        func = MBF_READ_HOLDING_REGISTERS;
        offset = 0;
        count = 0;
        format = mb::Dec16;
        writeOffset = 0;
        writeCount = 0;
        writeFormat = mb::Dec16;
    }

    int func;
    union
    {
        uint16_t offset;
        uint16_t subfunc;
    };
    uint16_t count;
    mb::Format format;

    union
    {
        uint16_t writeOffset;
        uint16_t andMask;
    };

    union
    {
        uint16_t writeCount;
        uint16_t orMask;
    };

    mb::Format writeFormat;
    QString data;

};
Q_DECLARE_METATYPE(mbClientMessageParams)

namespace mb {

namespace Client {

typedef mbClientPort* PortHandle_t;
typedef mbClientDevice* DeviceHandle_t;
typedef mbClientDataViewItem* ItemHandle_t;

} // namespace Client

QStringList saveClientMessages(const QList<mbClientMessageParams> messages);

QList<mbClientMessageParams> restoreClientMessages(const QStringList &messages);

QString saveClientMessageParams(const mbClientMessageParams &params, bool useFunc = true, bool useData = true);

mbClientMessageParams restoreClientMessageParams(const QString &params, bool *ok = nullptr, uint8_t func = 0);

QHash<QString, QString> getClientParamMap(const QString &params);

} // namespace mb

#endif // CLIENT_GLOBAL_H
