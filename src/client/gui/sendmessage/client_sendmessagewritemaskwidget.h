#ifndef CLIENT_SENDMESSAGEWRITEMASKWIDGET_H
#define CLIENT_SENDMESSAGEWRITEMASKWIDGET_H

#include "client_sendmessagewidget.h"

class QSpinBox;
class mbCoreAddressWidget;

class mbClientSendMessageWriteMaskWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString prefix          ;
        const QString writeMaskAddress;
        const QString writeMaskAnd    ;
        const QString writeMaskOr     ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageWriteMaskWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    QByteArray getData() const override;
    void setData(const QByteArray &data) override;

public:
    void setModbusAddresNotation(mb::AddressNotation notation);
    
    int getAddress() const;
    void setAddress(int v);
    uint16_t getOffset() const;
    void setOffset(uint16_t v);

    uint16_t getMaskAnd() const;
    void setMaskAnd(uint16_t v);

    uint16_t getMaskOr() const;
    void setMaskOr(uint16_t v);

private:
    mbCoreAddressWidget* m_address;
    QSpinBox* m_spMaskAnd;
    QSpinBox* m_spMaskOr;
};

#endif // CLIENT_SENDMESSAGEWRITEMASKWIDGET_H
