#ifndef CLIENT_SENDMESSAGEREADWRITEMULTIREGWIDGET_H
#define CLIENT_SENDMESSAGEREADWRITEMULTIREGWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QSpinBox;
class QPlainTextEdit;
class mbCoreAddressWidget;

class mbClientSendMessageReadWriteMultipleRegistersWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString writeFormat ;
        const QString writeAddress;
        const QString writeCount  ;
        const QString writeData   ;
        const QString readFormat  ;
        const QString readAddress ;
        const QString readCount   ;
        const QString readData    ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageReadWriteMultipleRegistersWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void fillParams(mbClientMessageParams &params) const override;
    void setParams(mbClientMessageParams &params) override;

public:
    void setModbusAddresNotation(mb::AddressNotation notation);
    
    mb::Format writeFormat() const;
    int getWriteAddress() const;
    void setWriteAddress(int v);
    uint16_t getWriteOffset() const;
    void setWriteOffset(uint16_t v);

    uint16_t getWriteCount() const;
    void setWriteCount(uint16_t v);

    mb::Format readFormat() const;
    int getReadAddress() const;
    void setReadAddress(int v);
    uint16_t getReadOffset() const;
    void setReadOffset(uint16_t v);

    uint16_t getReadCount() const;
    void setReadCount(uint16_t v);

private Q_SLOTS:
    void updateReadData();

private:
    QComboBox* m_cmbWriteFormat;
    mbCoreAddressWidget* m_writeAddress;
    QSpinBox* m_spWriteCount;
    QPlainTextEdit* m_txtWriteData;

    QByteArray m_readData;
    QComboBox* m_cmbReadFormat;
    mbCoreAddressWidget* m_readAddress;
    QSpinBox* m_spReadCount;
    QPlainTextEdit* m_txtReadData;
};

#endif // CLIENT_SENDMESSAGEREADWRITEMULTIREGWIDGET_H
