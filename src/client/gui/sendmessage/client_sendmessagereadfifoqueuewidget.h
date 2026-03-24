#ifndef CLIENT_SENDMESSAGEREADFIFOQUEUEWIDGET_H
#define CLIENT_SENDMESSAGEREADFIFOQUEUEWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QSpinBox;
class QPlainTextEdit;
class mbCoreAddressWidget;

class mbClientSendMessageReadFIFOQueueWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString format;
        const QString offset;
        const QString data  ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageReadFIFOQueueWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void prepareToSend() override;
    void fillParams(mbClientMessageParams &params) const override;
    void setParams(mbClientMessageParams &params) override;

public:
    mb::Format format() const;
    uint16_t getOffset() const;
    void setOffset(uint16_t v);

private Q_SLOTS:
    void updateData();

private:
    QByteArray m_data;
    QComboBox* m_cmbFormat;
    QSpinBox* m_spOffset;
    QPlainTextEdit* m_txtData;
};

#endif // CLIENT_SENDMESSAGEREADFIFOQUEUEWIDGET_H
