#ifndef CLIENT_SENDMESSAGE_GLOBAL_H
#define CLIENT_SENDMESSAGE_GLOBAL_H

#include <client_global.h>

struct mbClientSendMessageParams
{
    mbClientSendMessageParams()
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

namespace mb {

QStringList saveSendMessages(const QList<const mbClientSendMessageParams*> messages);

QList<const mbClientSendMessageParams*> restoreSendMessages(const QStringList &messages);

QString saveSendMessageParams(const mbClientSendMessageParams &params, bool useFunc = true);

mbClientSendMessageParams* restoreSendMessageParams(const QString &params, uint8_t func = 0);

QHash<QString, QString> getStringParamMap(const QString &params);

} // namespace mb

#endif // CLIENT_SENDMESSAGE_GLOBAL_H
