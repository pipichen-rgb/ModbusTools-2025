#ifndef CLIENT_SENDMESSAGEREADEXCEPTIONSTATUSWIDGET_H
#define CLIENT_SENDMESSAGEREADEXCEPTIONSTATUSWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QLineEdit;
class mbCoreAddressWidget;

class mbClientSendMessageReadExceptionStatusWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString format ;
        const QString value  ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageReadExceptionStatusWidget(mbClientMessageConverter* conv, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void setParams(mbClientMessageParams &params) override;

public:
    mb::DigitalFormat digitalFormat() const;

private Q_SLOTS:
    void updateValue();

private:
    quint8 m_value;
    QComboBox* m_cmbFormat;
    QLineEdit* m_lnValue;
};

#endif // CLIENT_SENDMESSAGEREADEXCEPTIONSTATUSWIDGET_H
