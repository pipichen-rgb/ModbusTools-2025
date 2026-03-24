#ifndef CLIENT_SENDMESSAGEGETCOMMEVENTCOUNTERWIDGET_H
#define CLIENT_SENDMESSAGEGETCOMMEVENTCOUNTERWIDGET_H

#include "client_sendmessagewidget.h"

class QLineEdit;

class mbClientSendMessageGetCommEventCounterWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString status;
        const QString count ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageGetCommEventCounterWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void prepareToSend() override;
    void setParams(mbClientMessageParams &params) override;

public:
    uint16_t getStatus() const;
    void setStatus(uint16_t v);

    uint16_t getCount() const;
    void setCount(uint16_t v);

private:
    QLineEdit* m_lnStatus;
    QLineEdit* m_lnCount;
};

#endif // CLIENT_SENDMESSAGEGETCOMMEVENTCOUNTERWIDGET_H
