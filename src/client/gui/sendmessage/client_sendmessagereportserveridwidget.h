#ifndef CLIENT_SENDMESSAGEREPORTSERVERIDWIDGET_H
#define CLIENT_SENDMESSAGEREPORTSERVERIDWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QPlainTextEdit;

class mbClientSendMessageReportServerIdWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString format;
        const QString data  ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageReportServerIdWidget(mbClientMessageConverter* conv, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void setParams(mbClientMessageParams &params) override;

public:
    mb::Format format() const;
    
private Q_SLOTS:
    void updateData();

private:
    QByteArray m_data;
    QComboBox* m_cmbFormat;
    QPlainTextEdit* m_txtData;
};

#endif // CLIENT_SENDMESSAGEREPORTSERVERIDWIDGET_H
