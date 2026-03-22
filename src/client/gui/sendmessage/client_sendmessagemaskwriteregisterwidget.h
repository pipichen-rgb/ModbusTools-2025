#ifndef CLIENT_SENDMESSAGEMASKWRITEREGISTERWIDGET_H
#define CLIENT_SENDMESSAGEMASKWRITEREGISTERWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QSpinBox;
class mbCoreAddressWidget;

class mbClientSendMessageMaskWriteRegisterWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString format ;
        const QString address;
        const QString maskAnd;
        const QString maskOr ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageMaskWriteRegisterWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void fillParams(mbClientMessageParams &params) const override;
    void setParams(mbClientMessageParams &params) override;

public:
    int getAddress() const;
    void setAddress(int v);

    uint16_t getOffset() const;
    void setOffset(uint16_t v);

    int getMaskAnd() const;
    void setMaskAnd(int v);

    int getMaskOr() const;
    void setMaskOr(int v);

    mb::DigitalFormat digitalFormat() const;

private Q_SLOTS:
    void setDigitalFormat(int index);

private:
    QComboBox* m_cmbFormat;
    mbCoreAddressWidget* m_address;
    QSpinBox* m_spMaskAnd;
    QSpinBox* m_spMaskOr;
};

#endif // CLIENT_SENDMESSAGEMASKWRITEREGISTERWIDGET_H
