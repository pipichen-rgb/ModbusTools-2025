#ifndef CLIENT_SENDMESSAGEWRITESINGLEREGISTERWIDGET_H
#define CLIENT_SENDMESSAGEWRITESINGLEREGISTERWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QSpinBox;
class mbCoreAddressWidget;

class mbClientSendMessageWriteSingleRegisterWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString format ;
        const QString address;
        const QString value  ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageWriteSingleRegisterWidget(mbClientMessageConverter* conv, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void fillParams(mbClientMessageParams &params) const override;

public:
    uint16_t getOffset() const;
    int getAddress() const;
    void setAddress(int v);

private Q_SLOTS:
    void setDigitalFormat(int index);

private:
    QComboBox* m_cmbFormat;
    mbCoreAddressWidget* m_address;
    QSpinBox* m_spValue;
};

#endif // CLIENT_SENDMESSAGEWRITESINGLEREGISTERWIDGET_H
