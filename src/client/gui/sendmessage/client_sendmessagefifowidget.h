#ifndef CLIENT_SENDMESSAGEFIFOWIDGET_H
#define CLIENT_SENDMESSAGEFIFOWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QSpinBox;
class QPlainTextEdit;
class mbCoreAddressWidget;

class mbClientSendMessageFIFOWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString prefix    ;
        const QString fifoFormat;
        const QString fifoOffset;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageFIFOWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    QByteArray getData() const override;
    void setData(const QByteArray &data) override;

public:
    uint16_t getOffset() const;

private:
    QComboBox* m_cmbFormat;
    QSpinBox* m_spOffset;
    QPlainTextEdit* m_txtData;
    QByteArray m_data;
    bool m_isDirty;
};

#endif // CLIENT_SENDMESSAGEFIFOWIDGET_H
