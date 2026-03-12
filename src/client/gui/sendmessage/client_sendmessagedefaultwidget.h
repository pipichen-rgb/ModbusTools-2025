#ifndef CLIENT_SENDMESSAGEDEFAULTWIDGET_H
#define CLIENT_SENDMESSAGEDEFAULTWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QSpinBox;
class QPlainTextEdit;
class mbCoreAddressWidget;

class mbClientSendMessageDefaultWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString prefix        ;
        const QString defaultAddress;
        const QString defaultFormat ;
        const QString defaultCount  ;
        const QString defaultData   ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageDefaultWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    QByteArray getData() const override;
    void setData(const QByteArray &data) override;

public:
    uint16_t getOffset() const;
    uint16_t getCount() const;

private:
    int getAddress() const;
    void setAddress(int v);

private:
    QComboBox* m_cmbFormat;
    mbCoreAddressWidget* m_address;
    QSpinBox* m_spCount;
    QPlainTextEdit* m_txtData;
    QByteArray m_data;
    bool m_isDirty;
};

#endif // CLIENT_SENDMESSAGEDEFAULTWIDGET_H
