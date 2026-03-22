#include "client_global.h"

#include <project/client_device.h>

int id_mbClientMessageParams = qRegisterMetaType<mbClientMessageParamsOLD>();

namespace mb {

QStringList saveClientMessages(const QList<mbClientMessageParamsOLD> messages)
{
    QStringList res;
    Q_FOREACH(const mbClientMessageParamsOLD &p, messages)
    {
        res.append(saveClientMessageParams(p));
    }
    return res;
}

QList<mbClientMessageParamsOLD> restoreClientMessages(const QStringList &messages)
{
    QList<mbClientMessageParamsOLD> res;
    bool ok = false;
    Q_FOREACH(const QString &s, messages)
    {
        auto p = restoreClientMessageParams(s, &ok);
        if (ok)
            res.append(p);
    }
    return res;
}

QString saveClientMessageParams(const mbClientMessageParamsOLD &params, bool useFunc, bool useData)
{
    QString res;
    switch(params.func)
    {
    case MBF_READ_COILS:
    case MBF_READ_DISCRETE_INPUTS:
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_READ_INPUT_REGISTERS:
        if (useData)
            res = QString("offset=%1;count=%2;format=%3").arg(params.offset).arg(params.count).arg(mb::enumFormatKey(params.format));
        else
            res = QString("offset=%1;count=%2").arg(params.offset).arg(params.count);
        break;
    case MBF_WRITE_SINGLE_COIL:
    case MBF_WRITE_SINGLE_REGISTER:
        if (useData)
            res = QString("offset=%1;format=%2;data=%3").arg(params.offset).arg(mb::enumFormatKey(params.format), params.data.toString());
        else
            res = QString("offset=%1").arg(params.offset);
        break;
    case MBF_READ_EXCEPTION_STATUS:
    case MBF_REPORT_SERVER_ID:
        if (useData)
            res = QString("format=%1").arg(mb::enumFormatKey(params.format));
        break;
    case MBF_DIAGNOSTICS:
        if (useData)
            res = QString("subfunc=%1;format=%2;data=%3").arg(params.subfunc).arg(mb::enumFormatKey(params.format), params.data.toString());
        else
            res = QString("subfunc=%1").arg(params.subfunc);
        break;
    case MBF_WRITE_MULTIPLE_COILS:
    case MBF_WRITE_MULTIPLE_REGISTERS:
        if (useData)
            res = QString("offset=%1;count=%2;format=%3;data=%4")
                  .arg(params.offset).arg(params.count).arg(mb::enumFormatKey(params.format), params.data.toString());
        else
            res = QString("offset=%1;count=%2")
                  .arg(params.offset).arg(params.count);
        break;
    case MBF_MASK_WRITE_REGISTER:
        res += QString("offset=%1;and=%2;or=%3").arg(params.offset).arg(params.andMask).arg(params.orMask);
        break;
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        if (useData)
            res += QString("readoffset=%1;readcount=%2;readformat=%3;writeoffset=%4;writecount=%5;writeformat=%6;data=%7")
                   .arg(params.offset)
                   .arg(params.count)
                   .arg(mb::enumFormatKey(params.format))
                   .arg(params.writeOffset)
                   .arg(params.writeCount)
                   .arg(mb::enumFormatKey(params.writeFormat), params.data.toString());
        else
            res += QString("readoffset=%1;readcount=%2;writeoffset=%3;writecount=%5")
                       .arg(params.offset)
                       .arg(params.count)
                       .arg(params.writeOffset)
                       .arg(params.writeCount);
        break;
    case MBF_READ_FIFO_QUEUE:
        if (useData)
            res += QString("offset=%1;format=%2").arg(params.offset).arg(mb::enumFormatKey(params.format));
        else
            res += QString("offset=%1").arg(params.offset);
        break;
    case MBF_ENCAPSULATED_INTERFACE_TRANSPORT:
        if (useData)
            res += QString("deviceId=%1;objectId=%2;format=%3").arg(params.deviceId).arg(params.objectId).arg(mb::enumFormatKey(params.format));
        else
            res += QString("deviceId=%1;objectId=%2").arg(params.deviceId).arg(params.objectId);
        break;
    default:
        break;
    }
    if (useFunc)
    {
        if (res.isEmpty())
            res = QString("FC%1").arg(params.func, 2, 10, QLatin1Char('0'));
        else
            res = QString("FC%1;%2").arg(params.func, 2, 10, QLatin1Char('0')).arg(res);
    }
    return res;
}

mbClientMessageParamsOLD restoreClientMessageParams(const QString &params, bool *ok, uint8_t func)
{
    mbClientMessageParamsOLD res;
    QHash<QString, QString> map = getClientParamMap(params);
    if (func == 0)
    {
        bool funcOk = false;
        Q_FOREACH(const QString &key, map.keys())
        {
            if (key.startsWith(QLatin1String("FC")))
            {
                func = static_cast<uint8_t>(key.midRef(2).toInt(&funcOk));
                if (!funcOk)
                {
                    if (ok)
                        *ok = false;
                    return res;
                }
                break;
            }
        }
        if (!funcOk)
        {
            // Support old format where func was a separate parameter
            func = static_cast<uint8_t>(map.value(QStringLiteral("func")).toInt(&funcOk));
            if (!funcOk)
            {
                if (ok)
                    *ok = false;
                return res;
            }
        }
    }
    res.func = func;
    switch (func)
    {
    case MBF_READ_COILS:
    case MBF_READ_DISCRETE_INPUTS:
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_READ_INPUT_REGISTERS:
        res.offset = static_cast<uint16_t>(map.value(QStringLiteral("offset"), "0").toInt());
        res.count = static_cast<uint16_t>(map.value(QStringLiteral("count"), "1").toInt());
        res.format = mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16"));
        return res;
    case MBF_WRITE_SINGLE_COIL:
    case MBF_WRITE_SINGLE_REGISTER:
        res.offset = static_cast<uint16_t>(map.value(QStringLiteral("offset"), "0").toInt());
        res.format = mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16"));
        res.data = map.value(QStringLiteral("data"));
        return res;
    case MBF_READ_EXCEPTION_STATUS:
    case MBF_REPORT_SERVER_ID:
        res.format = mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16"));
        return res;
    case MBF_DIAGNOSTICS:
        res.subfunc = static_cast<uint16_t>(map.value(QStringLiteral("subfunc"), "0").toInt());
        res.format = mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16"));
        res.data = map.value(QStringLiteral("data"));
        return res;
    case MBF_GET_COMM_EVENT_COUNTER:
    case MBF_GET_COMM_EVENT_LOG:
        return res;
    case MBF_WRITE_MULTIPLE_COILS:
    case MBF_WRITE_MULTIPLE_REGISTERS:
        res.offset = static_cast<uint16_t>(map.value(QStringLiteral("offset"), "0").toInt());
        res.count = static_cast<uint16_t>(map.value(QStringLiteral("count"), "1").toInt());
        res.format = mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16"));
        res.data = map.value(QStringLiteral("data"));
        return res;
    case MBF_MASK_WRITE_REGISTER:
        res.offset = static_cast<uint16_t>(map.value(QStringLiteral("offset"), "0").toInt());
        res.andMask = static_cast<uint16_t>(map.value(QStringLiteral("and"), "0").toInt());
        res.orMask = static_cast<uint16_t>(map.value(QStringLiteral("or"), "0").toInt());
        return res;
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        res.offset = static_cast<uint16_t>(map.value(QStringLiteral("readoffset"), "0").toInt());
        res.count = static_cast<uint16_t>(map.value(QStringLiteral("readcount"), "1").toInt());
        res.format = mb::enumFormatValue(map.value(QStringLiteral("readformat"), "Dec16"));
        res.writeOffset = static_cast<uint16_t>(map.value(QStringLiteral("writeoffset"), "0").toInt());
        res.writeCount = static_cast<uint16_t>(map.value(QStringLiteral("writecount"), "1").toInt());
        res.writeFormat = mb::enumFormatValue(map.value(QStringLiteral("writeformat"), "Dec16"));
        res.data = map.value(QStringLiteral("data"));
        return res;
    case MBF_READ_FIFO_QUEUE:
        res.offset = static_cast<uint16_t>(map.value(QStringLiteral("offset"), "0").toInt());
        res.format = mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16"));
        return res;
    case MBF_ENCAPSULATED_INTERFACE_TRANSPORT:
        res.deviceId = static_cast<uint16_t>(map.value(QStringLiteral("deviceId"), "1").toInt());
        res.objectId = static_cast<uint16_t>(map.value(QStringLiteral("objectId"), "0").toInt());
        res.format = mb::enumFormatValue(map.value(QStringLiteral("format"), "String"));
        return res;
    default:
        if (ok)
            *ok = false;
        return res;
    }
    if (ok)
        *ok = true;
    return res;
}

QHash<QString, QString> getClientParamMap(const QString &params)
{
    QHash<QString, QString> map;
    QStringList parts = params.split(QLatin1Char(';'), Qt::SkipEmptyParts);
    Q_FOREACH (const QString &part, parts)
    {
        QStringList keyValue = part.split(QLatin1Char('='), Qt::KeepEmptyParts);
        if (keyValue.size() == 2)
        {
            map.insert(keyValue.at(0), keyValue.at(1));
        }
        else
        {
            map.insert(part, QString());
        }
    }
    return map;
}

} // namespace mb

mbClientMessageConverter::mbClientMessageConverter()
{
    const auto &d = mbClientDevice::Defaults::instance();

    m_dataParams.swapBytes          = d.swapBytes         ;
    m_dataParams.registerOrder      = d.registerOrder     ;
    m_dataParams.byteArrayFormat    = d.byteArrayFormat   ;
    m_dataParams.stringEncoding     = d.stringEncoding    ;
    m_dataParams.stringLengthType   = d.stringLengthType  ;
    m_dataParams.byteArraySeparator = d.byteArraySeparator;
}

QByteArray mbClientMessageConverter::toByteArray(const mbClientMessageParams &params) const
{
    if (params.data().type() == QVariant::ByteArray)
        return params.data().toByteArray();
    QByteArray data;
    switch (params.format())
    {
    case mb::Bool:
    {
        QStringList ls = dataToStringList(params.data().toString());
        data = fromStringListBits(ls);
    }
        break;
    case mb::ByteArray:
    case mb::String:
    {
        auto memType = getMemoryType(params);
        int c;
        if (isBitMemory(memType))
            c = (params.count() + 7) / 8;
        else
            c = params.count() * 2;
        data = mb::toByteArray(params.data(),
                               params.format(),
                               Modbus::Memory_4x,
                               m_dataParams.swapBytes,
                               m_dataParams.registerOrder,
                               m_dataParams.byteArrayFormat,
                               m_dataParams.stringEncoding,
                               m_dataParams.stringLengthType,
                               m_dataParams.byteArraySeparator,
                               c);
    }
        break;
    default:
    {
        QStringList ls = dataToStringList(params.data().toString());
        data = fromStringListNumbers(ls, params.format());
    }
        break;
    }
    return data;
}

QVariant mbClientMessageConverter::toVariant(const mbClientMessageParams &params) const
{
    auto v = params.data();
    if (v.type() != QVariant::ByteArray)
        return v;
    QByteArray data = v.toByteArray();
    QStringList ls;
    switch (params.format())
    {
    case mb::Bool:
    {
        uint16_t count = params.count();
        if (!isBitMemory(params))
            count *= 16;
        ls = toStringListBits(data, count);
    }
        break;
    case mb::ByteArray:
    case mb::String:
        ls.append(mb::toVariant(data,
                                params.format(),
                                Modbus::Memory_0x,
                                m_dataParams.swapBytes,
                                m_dataParams.registerOrder,
                                m_dataParams.byteArrayFormat,
                                m_dataParams.stringEncoding,
                                m_dataParams.stringLengthType,
                                m_dataParams.byteArraySeparator,
                                data.count()).toString());
        break;
    default:
        ls = toStringListNumbers(data, params.format());
        break;
    }
    QVariant res;
    if (ls.count())
    {
        QStringList::ConstIterator it = ls.constBegin();
        QString s = *it;
        for (++it; it != ls.constEnd(); ++it)
            s += QStringLiteral(",") + *it;
        res = s;
    }
    return res;
}

QStringList mbClientMessageConverter::toStringListBits(const QByteArray &data, uint16_t count) const
{
    QStringList ls;
    for (int i = 0; i < count; i++)
    {
        bool v = data.at(i / 8) & (1 << (i % 8));
        QString s = v ? QStringLiteral("1") : QStringLiteral("0");
        ls.append(s);
    }
    return ls;
}

QStringList mbClientMessageConverter::toStringListNumbers(const QByteArray &data, mb::Format format) const
{
    QStringList ls;
    int sz = static_cast<int>(mb::sizeofFormat(format));
    int c = data.size() / sz;
    for (int i = 0; i < c; i++)
    {
        QByteArray numData = data.mid(i*sz, sz);
        QString s = mb::toVariant(numData,
                                  format,
                                  Modbus::Memory_4x,
                                  m_dataParams.swapBytes,
                                  m_dataParams.registerOrder,
                                  m_dataParams.byteArrayFormat,
                                  m_dataParams.stringEncoding,
                                  m_dataParams.stringLengthType,
                                  m_dataParams.byteArraySeparator,
                                  numData.count()).toString();
        ls.append(s);
    }
    size_t szRemainder = data.size() - c * sz;
    if (szRemainder)
    {
        quint64 v = 0;
        memcpy(&v, &data.data()[c*sz], szRemainder);
        QByteArray numData(reinterpret_cast<char*>(&v), sizeof(v));
        QString s = mb::toVariant(numData,
                                  format,
                                  Modbus::Memory_4x,
                                  m_dataParams.swapBytes,
                                  m_dataParams.registerOrder,
                                  m_dataParams.byteArrayFormat,
                                  m_dataParams.stringEncoding,
                                  m_dataParams.stringLengthType,
                                  m_dataParams.byteArraySeparator,
                                  numData.count()).toString();
        ls.append(s);
    }
    return ls;
}

QByteArray mbClientMessageConverter::fromStringListBits(const QStringList &ls) const
{
    QByteArray r((ls.count()+7)/8, '\0');
    int i = 0;
    Q_FOREACH(const QString &s, ls)
    {
        if (s == QStringLiteral("1"))
            r.data()[i / 8] |= (1 << (i % 8));
        i++;
    }
    return r;
}

QByteArray mbClientMessageConverter::fromStringListNumbers(const QStringList &ls, mb::Format format) const
{
    QByteArray data;
    Q_FOREACH(const QString &s, ls)
        data.append(mb::toByteArray(s,
                                    format,
                                    Modbus::Memory_4x,
                                    m_dataParams.swapBytes,
                                    m_dataParams.registerOrder,
                                    m_dataParams.byteArrayFormat,
                                    m_dataParams.stringEncoding,
                                    m_dataParams.stringLengthType,
                                    m_dataParams.byteArraySeparator,
                                    0));

    return data;
}

bool mbClientMessageConverter::fromStringNumber(mb::Format format, const QString &v, void *buff) const
{
    bool ok = false;
    switch (format)
    {
    case mb::Bin16:
        *reinterpret_cast<quint16*>(buff) = v.toUShort(&ok, 2);
        break;
    case mb::Oct16:
        *reinterpret_cast<quint16*>(buff) = v.toUShort(&ok, 8);
        break;
    case mb::Dec16:
        *reinterpret_cast<qint16*>(buff) = v.toShort(&ok, 10);
        break;
    case mb::UDec16:
        *reinterpret_cast<quint16*>(buff) = v.toUShort(&ok, 10);
        break;
    case mb::Hex16:
        *reinterpret_cast<quint16*>(buff) = v.toUShort(&ok, 16);
        break;
    case mb::Bin32:
        *reinterpret_cast<quint32*>(buff) = v.toULong(&ok, 2);
        break;
    case mb::Oct32:
        *reinterpret_cast<quint32*>(buff) = v.toULong(&ok, 8);
        break;
    case mb::Dec32:
        *reinterpret_cast<qint32*>(buff) = v.toLong(&ok, 10);
        break;
    case mb::UDec32:
        *reinterpret_cast<quint32*>(buff) = v.toULong(&ok, 10);
        break;
    case mb::Hex32:
        *reinterpret_cast<quint32*>(buff) = v.toULong(&ok, 16);
        break;
    case mb::Bin64:
        *reinterpret_cast<quint64*>(buff) = v.toULongLong(&ok, 2);
        break;
    case mb::Oct64:
        *reinterpret_cast<quint64*>(buff) = v.toULongLong(&ok, 8);
        break;
    case mb::Dec64:
        *reinterpret_cast<qint64*>(buff) = v.toLongLong(&ok, 10);
        break;
    case mb::UDec64:
        *reinterpret_cast<quint64*>(buff) = v.toULongLong(&ok, 10);
        break;
    case mb::Hex64:
        *reinterpret_cast<quint64*>(buff) = v.toULongLong(&ok, 16);
        break;
    case mb::Float:
        *reinterpret_cast<float*>(buff) = v.toFloat(&ok);
        break;
    case mb::Double:
        *reinterpret_cast<double*>(buff) = v.toDouble(&ok);
        break;
    default:
        break;
    }
    return ok;
}

QStringList mbClientMessageConverter::dataToStringList(const QString &s) const
{
    QStringList ls = s.split(',');
    return ls;
}

QString mbClientMessageConverter::getEventLogDescription(uint8_t eventId)
{
    QString res;
    if (eventId & 0x80)
    {
        res = "Receive Event: ";
        QStringList ls;
        if (eventId & 0x02) ls.append("Communication Error");
        if (eventId & 0x10) ls.append("Character Overrun");
        if (eventId & 0x20) ls.append("Currently in Listen Only Mode");
        if (eventId & 0x40) ls.append("Broadcast Received");
        res += ls.join(", ");
    }
    else if (eventId & 0x40)
    {
        res = "Send Event: ";
        QStringList ls;
        if (eventId & 0x01) ls.append("Read Exception Sent");
        if (eventId & 0x02) ls.append("Server Abort Exception Sent ");
        if (eventId & 0x04) ls.append("Server Busy Exception Sent");
        if (eventId & 0x08) ls.append("Server Program NAK Exception Sent");
        if (eventId & 0x10) ls.append("Write Timeout Error Occurred");
        if (eventId & 0x20) ls.append("Currently in Listen Only Mode");
        res += ls.join(", ");
    }
    else if (eventId == 0x04)
        res = "Entered Listen Only Mode";
    else if (eventId == 0x00)
        res = "Initiated Communication Restart";
    return res;
}

Modbus::MemoryType mbClientMessageConverter::getMemoryType(const mbClientMessageParams &params)
{
    switch (params.function())
    {
    case MBF_READ_COILS:
    case MBF_WRITE_SINGLE_COIL:
    case MBF_WRITE_MULTIPLE_COILS:
        return Modbus::Memory_Coils;
    case MBF_READ_DISCRETE_INPUTS:
        return Modbus::Memory_DiscreteInputs;
    case MBF_READ_INPUT_REGISTERS:
        return Modbus::Memory_InputRegisters;
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_WRITE_SINGLE_REGISTER:
    case MBF_WRITE_MULTIPLE_REGISTERS:
    case MBF_MASK_WRITE_REGISTER:
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
    case MBF_READ_FIFO_QUEUE:
        return Modbus::Memory_HoldingRegisters;
    }
    return Modbus::Memory_0x;
}

QStringList mbClientMessageConverter::saveClientMessages(const QList<mbClientMessageParams> messages) const
{
    QStringList res;
    Q_FOREACH(const mbClientMessageParams &p, messages)
    {
        res.append(saveClientMessageParams(p));
    }
    return res;
}

QList<mbClientMessageParams> mbClientMessageConverter::restoreClientMessages(const QStringList &messages) const
{
    QList<mbClientMessageParams> res;
    bool ok = false;
    Q_FOREACH(const QString &s, messages)
    {
        auto p = restoreClientMessageParams(s, &ok);
        if (ok)
            res.append(p);
    }
    return res;
}

QString mbClientMessageConverter::saveClientMessageParams(const mbClientMessageParams &params, bool useFunc, bool useData) const
{
    QString res;
    QStringList ls;
    switch(params.function())
    {
    case MBF_READ_COILS:
    case MBF_READ_DISCRETE_INPUTS:
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_READ_INPUT_REGISTERS:
        ls.append(serializeStringList({QStringLiteral("offset"), QString::number(params.offset())}, '='));
        ls.append(serializeStringList({QStringLiteral("count"), QString::number(params.count())}, '='));
        break;
    case MBF_WRITE_SINGLE_COIL:
    case MBF_WRITE_SINGLE_REGISTER:
        ls.append(serializeStringList({QStringLiteral("offset"), QString::number(params.offset())}, '='));
        if (useData)
        {
            ls.append(serializeStringList({QStringLiteral("format"), mb::enumFormatKey(params.format())}, '='));
            ls.append(serializeStringList({QStringLiteral("data"), params.data().toString()}, '='));
        }
        break;
    case MBF_READ_EXCEPTION_STATUS:
    case MBF_REPORT_SERVER_ID:
        break;
    case MBF_DIAGNOSTICS:
        ls.append(serializeStringList({QStringLiteral("subfunc"), QString::number(params.subfunction())}, '='));
        if (useData)
        {
            ls.append(serializeStringList({QStringLiteral("format"), mb::enumFormatKey(params.format())}, '='));
            ls.append(serializeStringList({QStringLiteral("data"), params.data().toString()}, '='));
        }
        break;
    case MBF_WRITE_MULTIPLE_COILS:
    case MBF_WRITE_MULTIPLE_REGISTERS:
        ls.append(serializeStringList({QStringLiteral("offset"), QString::number(params.offset())}, '='));
        ls.append(serializeStringList({QStringLiteral("count"), QString::number(params.count())}, '='));
        if (useData)
        {
            ls.append(serializeStringList({QStringLiteral("format"), mb::enumFormatKey(params.format())}, '='));
            ls.append(serializeStringList({QStringLiteral("data"), params.data().toString()}, '='));
        }
        break;
    case MBF_READ_FILE_RECORD:
    case MBF_WRITE_FILE_RECORD:
    {
        QStringList recordLs;
        for (auto &r : params.fileRecords())
            recordLs.append(QString("%1,%2,%3").arg(r.fileNumber).arg(r.recordNumber).arg(r.recordLength));
        QString sRecords = serializeStringList(recordLs, '|');
        ls.append(serializeStringList({QStringLiteral("records"), sRecords}, '='));
        if ((params.function() == MBF_WRITE_FILE_RECORD) && useData)
        {
            ls.append(serializeStringList({QStringLiteral("format"), mb::enumFormatKey(params.format())}, '='));
            auto s = this->toVariant(params).toString();
            ls.append(serializeStringList({QStringLiteral("data"), s}, '='));
        }
    }
        break;
    case MBF_MASK_WRITE_REGISTER:
        ls.append(serializeStringList({QStringLiteral("offset"), QString::number(params.offset())}, '='));
        ls.append(serializeStringList({QStringLiteral("and"), QString::number(params.writeOffset())}, '='));
        ls.append(serializeStringList({QStringLiteral("or"), QString::number(params.writeCount())}, '='));
        break;
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        ls.append(serializeStringList({QStringLiteral("readoffset"), QString::number(params.offset())}, '='));
        ls.append(serializeStringList({QStringLiteral("readcount"), QString::number(params.count())}, '='));
        ls.append(serializeStringList({QStringLiteral("writeoffset"), QString::number(params.writeOffset())}, '='));
        ls.append(serializeStringList({QStringLiteral("writecount"), QString::number(params.writeCount())}, '='));
        if (useData)
        {
            ls.append(serializeStringList({QStringLiteral("format"), mb::enumFormatKey(params.format())}, '='));
            ls.append(serializeStringList({QStringLiteral("data"), params.data().toString()}, '='));
        }
        break;
    case MBF_READ_FIFO_QUEUE:
        ls.append(serializeStringList({QStringLiteral("offset"), QString::number(params.offset())}, '='));
        break;
    case MBF_ENCAPSULATED_INTERFACE_TRANSPORT:
        ls.append(serializeStringList({QStringLiteral("deviceId"), QString::number(params.deviceId())}, '='));
        ls.append(serializeStringList({QStringLiteral("objectId"), QString::number(params.objectId())}, '='));
        break;
    default:
        break;
    }

    if (!ls.isEmpty())
        res = serializeStringList(ls, ';');

    if (useFunc)
    {
        if (res.isEmpty())
            res = QString("FC%1").arg(params.function(), 2, 10, QLatin1Char('0'));
        else
            res = QString("FC%1;%2").arg(params.function(), 2, 10, QLatin1Char('0')).arg(res);
    }
    return res;
}

mbClientMessageParams mbClientMessageConverter::restoreClientMessageParams(const QString &params, bool *ok, uint8_t func) const
{
    mbClientMessageParams res;
    QHash<QString, QString> map = getClientParamMap(params);
    if (func == 0)
    {
        bool funcOk = false;
        Q_FOREACH(const QString &key, map.keys())
        {
            if (key.startsWith(QLatin1String("FC")))
            {
                func = static_cast<uint8_t>(key.midRef(2).toInt(&funcOk));
                if (!funcOk)
                {
                    if (ok)
                        *ok = false;
                    return res;
                }
                break;
            }
        }
        if (!funcOk)
        {
            // Support old format where func was a separate parameter
            func = static_cast<uint8_t>(map.value(QStringLiteral("func")).toInt(&funcOk));
            if (!funcOk)
            {
                if (ok)
                    *ok = false;
                return res;
            }
        }
    }
    res.setFunction(func);
    switch (func)
    {
    case MBF_READ_COILS:
    case MBF_READ_DISCRETE_INPUTS:
    case MBF_READ_HOLDING_REGISTERS:
    case MBF_READ_INPUT_REGISTERS:
        res.setOffset(static_cast<uint16_t>(map.value(QStringLiteral("offset"), "0").toInt()));
        res.setCount(static_cast<uint16_t>(map.value(QStringLiteral("count"), "1").toInt()));
        res.setFormat(mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16")));
        break;
    case MBF_WRITE_SINGLE_COIL:
    case MBF_WRITE_SINGLE_REGISTER:
        res.setOffset(static_cast<uint16_t>(map.value(QStringLiteral("offset"), "0").toInt()));
        res.setFormat(mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16")));
        res.setData(map.value(QStringLiteral("data")));
        break;
    case MBF_READ_EXCEPTION_STATUS:
    case MBF_REPORT_SERVER_ID:
        res.setFormat(mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16")));
        break;
    case MBF_DIAGNOSTICS:
        res.setSubfunction(static_cast<uint16_t>(map.value(QStringLiteral("subfunc"), "0").toInt()));
        res.setFormat(mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16")));
        res.setData(map.value(QStringLiteral("data")));
        break;
    case MBF_GET_COMM_EVENT_COUNTER:
    case MBF_GET_COMM_EVENT_LOG:
        break;
    case MBF_WRITE_MULTIPLE_COILS:
    case MBF_WRITE_MULTIPLE_REGISTERS:
        res.setOffset(static_cast<uint16_t>(map.value(QStringLiteral("offset"), "0").toInt()));
        res.setCount(static_cast<uint16_t>(map.value(QStringLiteral("count"), "1").toInt()));
        res.setFormat(mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16")));
        res.setData(map.value(QStringLiteral("data")));
        break;
    case MBF_READ_FILE_RECORD:
    case MBF_WRITE_FILE_RECORD:
    {
        auto sRecords = map.value(QStringLiteral("records"));
        QStringList recordLs = deserializeStringList(sRecords, '|');
        QVector<Modbus::FileRecord> records;
        records.reserve(recordLs.size());
        Q_FOREACH(const QString &recordS, recordLs)
        {
            QStringList parts = deserializeStringList(recordS, ',');
            if (parts.size() == 3)
            {
                Modbus::FileRecord r;
                r.fileNumber   = static_cast<uint16_t>(parts.at(0).toInt());
                r.recordNumber = static_cast<uint16_t>(parts.at(1).toInt());
                r.recordLength = static_cast<uint16_t>(parts.at(2).toInt());
                records.append(r);
            }
        }
        res.setFileRecords(records);
        res.setFormat(mb::enumFormatValue(map.value(QStringLiteral("format"), "ByteArray")));
        auto v = map.value(QStringLiteral("data"));
        res.setData(v);
    }
        break;
    case MBF_MASK_WRITE_REGISTER:
        res.setOffset(static_cast<uint16_t>(map.value(QStringLiteral("offset"), "0").toInt()));
        res.setWriteOffset(static_cast<uint16_t>(map.value(QStringLiteral("and"), "0").toInt()));
        res.setWriteCount(static_cast<uint16_t>(map.value(QStringLiteral("or"), "0").toInt()));
        break;
    case MBF_READ_WRITE_MULTIPLE_REGISTERS:
        res.setOffset(static_cast<uint16_t>(map.value(QStringLiteral("readoffset"), "0").toInt()));
        res.setCount(static_cast<uint16_t>(map.value(QStringLiteral("readcount"), "1").toInt()));
        res.setWriteOffset(static_cast<uint16_t>(map.value(QStringLiteral("writeoffset"), "0").toInt()));
        res.setWriteCount(static_cast<uint16_t>(map.value(QStringLiteral("writecount"), "1").toInt()));
        res.setFormat(mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16")));
        res.setData(map.value(QStringLiteral("data")));
        break;
    case MBF_READ_FIFO_QUEUE:
        res.setOffset(static_cast<uint16_t>(map.value(QStringLiteral("offset"), "0").toInt()));
        res.setFormat(mb::enumFormatValue(map.value(QStringLiteral("format"), "Dec16")));
        break;
    case MBF_ENCAPSULATED_INTERFACE_TRANSPORT:
        res.setDeviceId(static_cast<uint8_t>(map.value(QStringLiteral("deviceId"), "1").toInt()));
        res.setObjectId(static_cast<uint8_t>(map.value(QStringLiteral("objectId"), "0").toInt()));
        res.setFormat(mb::enumFormatValue(map.value(QStringLiteral("format"), "String")));
        break;
    default:
        if (ok)
            *ok = false;
        return res;
    }
    if (ok)
        *ok = true;
    return res;
}

QHash<QString, QString> mbClientMessageConverter::getClientParamMap(const QString &params)
{
    QHash<QString, QString> map;
    QStringList parts = deserializeStringList(params, ';');
    Q_FOREACH (const QString &part, parts)
    {
        QStringList keyValue = deserializeStringList(part, '=');
        if (keyValue.size() == 2)
        {
            map.insert(keyValue.at(0), keyValue.at(1));
        }
        else
        {
            map.insert(part, QString());
        }
    }
    return map;
}

QString mbClientMessageConverter::serializeStringList(const QStringList &ls, QChar sep)
{
    QString result;
    bool first = true;
    Q_FOREACH (const QString &item, ls)
    {
        if (!first)
            result.append(sep);
        first = false;

        for (QChar ch : item)
        {
            if (ch == sep || ch == QLatin1Char('\\'))
                result.append(QLatin1Char('\\'));
            result.append(ch);
        }
    }
    return result;
}

QStringList mbClientMessageConverter::deserializeStringList(const QString &s, QChar sep)
{
    if (s.isEmpty())
        return QStringList();

    QStringList result;
    QString current;
    bool escaped = false;

    for (QChar ch : s)
    {
        if (escaped)
        {
            current.append(ch);
            escaped = false;
            continue;
        }

        if (ch == QLatin1Char('\\'))
        {
            escaped = true;
            continue;
        }

        if (ch == sep)
        {
            result.append(current);
            current.clear();
            continue;
        }

        current.append(ch);
    }

    if (escaped)
        current.append(QLatin1Char('\\'));

    result.append(current);
    return result;
}

