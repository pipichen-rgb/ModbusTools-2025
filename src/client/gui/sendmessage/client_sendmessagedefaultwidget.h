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
        const QString address;
        const QString format ;
        const QString count  ;
        const QString data   ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageDefaultWidget(uint8_t func, mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void fillParams(mbClientMessageParams &params) const override;
    void setParams(mbClientMessageParams &params) override;

public:
    mb::Format format() const;
    uint16_t getOffset() const;
    void setOffset(uint16_t offset);
    int getAddress() const;
    void setAddress(int v);
    uint16_t getCount() const;

protected:
    QComboBox* m_cmbFormat;
    mbCoreAddressWidget* m_address;
    QSpinBox* m_spCount;
    QPlainTextEdit* m_txtData;
};

class mbClientSendMessageReadDefaultWidget : public mbClientSendMessageDefaultWidget
{
    Q_OBJECT

public:
    mbClientSendMessageReadDefaultWidget(uint8_t func, mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void prepareToSend() override;
    void setParams(mbClientMessageParams &params) override;

protected Q_SLOTS:
    void updateData();

protected:
    QByteArray m_data;
};

class mbClientSendMessageWriteDefaultWidget : public mbClientSendMessageDefaultWidget
{
    Q_OBJECT

public:
    mbClientSendMessageWriteDefaultWidget(uint8_t func, mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void fillParams(mbClientMessageParams &params) const override;
    void setParams(mbClientMessageParams &params) override;
};

class mbClientSendMessageReadCoilsWidget : public mbClientSendMessageReadDefaultWidget
{
public:
    mbClientSendMessageReadCoilsWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);
};  

class mbClientSendMessageReadDiscreteInputsWidget : public mbClientSendMessageReadDefaultWidget
{
public:
    mbClientSendMessageReadDiscreteInputsWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);
};  

class mbClientSendMessageReadInputRegistersWidget : public mbClientSendMessageReadDefaultWidget
{
public:
    mbClientSendMessageReadInputRegistersWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);
};  

class mbClientSendMessageReadHoldingRegistersWidget : public mbClientSendMessageReadDefaultWidget
{
public:
    mbClientSendMessageReadHoldingRegistersWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);
};  

class mbClientSendMessageWriteMultipleCoilsWidget : public mbClientSendMessageWriteDefaultWidget
{
public:
    mbClientSendMessageWriteMultipleCoilsWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);
};    

class mbClientSendMessageWriteMultipleRegistersWidget : public mbClientSendMessageWriteDefaultWidget
{
public:
    mbClientSendMessageWriteMultipleRegistersWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);
};    

#endif // CLIENT_SENDMESSAGEDEFAULTWIDGET_H
