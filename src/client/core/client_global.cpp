#include "client_global.h"

int id_mbClientMessageParams = qRegisterMetaType<mbClientMessageParams>();

namespace mb {

QStringList saveClientMessages(const QList<mbClientMessageParams> messages)
{
    QStringList res;
    Q_FOREACH(const mbClientMessageParams &p, messages)
    {
        res.append(saveClientMessageParams(p));
    }
    return res;
}

QList<mbClientMessageParams> restoreClientMessages(const QStringList &messages)
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

QString saveClientMessageParams(const mbClientMessageParams &params, bool useFunc, bool useData)
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
            res = QString("offset=%1;format=%2;data=%3").arg(params.offset).arg(mb::enumFormatKey(params.format), params.data);
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
            res = QString("subfunc=%1;format=%2;data=%3").arg(params.subfunc).arg(mb::enumFormatKey(params.format), params.data);
        else
            res = QString("subfunc=%1").arg(params.subfunc);
        break;
    case MBF_WRITE_MULTIPLE_COILS:
    case MBF_WRITE_MULTIPLE_REGISTERS:
        if (useData)
            res = QString("offset=%1;count=%2;format=%3;data=%4")
                  .arg(params.offset).arg(params.count).arg(mb::enumFormatKey(params.format), params.data);
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
                   .arg(mb::enumFormatKey(params.writeFormat), params.data);
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
    default:
        break;
    }
    if (useFunc)
    {
        if (res.isEmpty())
            res = QString("func=%1").arg(params.func);
        else
            res = QString("func=%1;%2").arg(params.func).arg(res);
    }
    return res;
}

mbClientMessageParams restoreClientMessageParams(const QString &params, bool *ok, uint8_t func)
{
    mbClientMessageParams res;
    QHash<QString, QString> map = getClientParamMap(params);
    if (func == 0)
    {
        bool okInner = false;
        func = static_cast<uint8_t>(map.value(QStringLiteral("func")).toInt(&okInner));
        if (!okInner)
        {
            if (ok)
                *ok = false;
            return res;
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
