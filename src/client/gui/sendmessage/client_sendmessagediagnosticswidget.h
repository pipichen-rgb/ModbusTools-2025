#ifndef CLIENT_SENDMESSAGEDIAGNOSTICSWIDGET_H
#define CLIENT_SENDMESSAGEDIAGNOSTICSWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QSpinBox;
class QPlainTextEdit;
class mbCoreAddressWidget;

class mbClientSendMessageDiagnosticsWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString subfunction;
        const QString format     ;
        const QString request    ;
        const QString response   ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageDiagnosticsWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void fillParams(mbClientMessageParams &params) const override;
    void setParams(mbClientMessageParams &params) override;

public:
    mb::Format format() const;
    uint16_t getSubfunction() const;
    void setSubfunction(uint16_t func);

private Q_SLOTS:
    void setCurrentDiagnSubfuncIndex(int funcIndex);
    void updateResponseData();

private:
    QByteArray m_responseData;
    QComboBox* m_cmbSubfunction;
    QComboBox* m_cmbFormat;
    QPlainTextEdit* m_txtDataRequest;
    QPlainTextEdit* m_txtDataResponse;
    QList<uint16_t> m_diagnSubfuncNums;

};

#endif // CLIENT_SENDMESSAGEDIAGNWIDGET_H
