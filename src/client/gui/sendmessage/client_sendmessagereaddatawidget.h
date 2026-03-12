#ifndef CLIENT_SENDMESSAGEREADDATAWIDGET_H
#define CLIENT_SENDMESSAGEREADDATAWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QPlainTextEdit;

class mbClientSendMessageReadDataWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString prefix        ;
        const QString readDataFormat;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageReadDataWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    QByteArray getData() const override;
    void setData(const QByteArray &data) override;

private:
    QComboBox* m_cmbFormat;
    QPlainTextEdit* m_txtData;
    QByteArray m_data;
    bool m_isDirty;
};

#endif // CLIENT_SENDMESSAGEREADDATAWIDGET_H
