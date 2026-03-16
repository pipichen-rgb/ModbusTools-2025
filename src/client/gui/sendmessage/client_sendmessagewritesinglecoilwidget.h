#ifndef CLIENT_SENDMESSAGEWRITESINGLECOILWIDGET_H
#define CLIENT_SENDMESSAGEWRITESINGLECOILWIDGET_H

#include "client_sendmessagewidget.h"

class QCheckBox;
class mbCoreAddressWidget;

class mbClientSendMessageWriteSingleCoilWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString address;
        const QString value  ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageWriteSingleCoilWidget(mbClientMessageConverter* conv, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void fillParams(mbClientMessageParams &params) const override;

public:    
    uint16_t getOffset() const;
    int getAddress() const;
    void setAddress(int v);

protected:
    mbCoreAddressWidget* m_address;
    QCheckBox* m_chkValue;
};

#endif // CLIENT_SENDMESSAGEWRITESINGLECOILWIDGET_H
