#ifndef CLIENT_SENDMESSAGEDIAGNWIDGET_H
#define CLIENT_SENDMESSAGEDIAGNWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QSpinBox;
class QPlainTextEdit;
class mbCoreAddressWidget;

class mbClientSendMessageDiagnWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString prefix          ;
        const QString diagnSubfunction;
        const QString diagnFormat     ;
        const QString diagnRequest    ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageDiagnWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    QByteArray getData() const override;
    void setData(const QByteArray &data) override;

public:
    uint16_t getSubfunction() const;
    void setSubfunction(uint16_t func);

private Q_SLOTS:
    void setCurrentDiagnSubfuncIndex(int funcIndex);

private:
    QComboBox* m_cmbSubfunction;
    QComboBox* m_cmbFormat;
    QPlainTextEdit* m_txtDataRequest;
    QPlainTextEdit* m_txtDataResponse;
    QList<uint16_t> m_diagnSubfuncNums;

};

#endif // CLIENT_SENDMESSAGEDIAGNWIDGET_H
