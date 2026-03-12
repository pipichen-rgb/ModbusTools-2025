#ifndef CLIENT_SENDMESSAGEGETCOMMEVENTLOGWIDGET_H
#define CLIENT_SENDMESSAGEGETCOMMEVENTLOGWIDGET_H

#include "client_sendmessagewidget.h"

class QLineEdit;
class QTableWidget;

class mbClientSendMessageGetCommEventLogWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString prefix;
        Strings();
        static const Strings &instance();
    };

public:
    static QString getEventLogDescription(uint8_t eventId);

public:
    mbClientSendMessageGetCommEventLogWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    QByteArray getData() const override;
    void setData(const QByteArray &data) override;

public:
    uint16_t getStatus() const;
    void setStatus(uint16_t v);

    uint16_t getEventCount() const;
    void setEventCount(uint16_t v);

    uint16_t getMessageCount() const;
    void setMessageCount(uint16_t v);

private:
    QLineEdit* m_lnStatus;
    QLineEdit* m_lnEventCount;
    QLineEdit* m_lnMessageCount;
    QTableWidget* m_tblEventLog;
};

#endif // CLIENT_SENDMESSAGEGETCOMMEVENTLOGWIDGET_H
