#ifndef CLIENT_SENDMESSAGEREADWRITEMULTIREGWIDGET_H
#define CLIENT_SENDMESSAGEREADWRITEMULTIREGWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QSpinBox;
class QPlainTextEdit;
class mbCoreAddressWidget;

class mbClientSendMessageReadWriteMultiRegWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString prefix                ;
        const QString rwMultiRegWriteAddress;
        const QString rwMultiRegWriteFormat ;
        const QString rwMultiRegWriteCount  ;
        const QString rwMultiRegWriteData   ;
        const QString rwMultiRegReadAddress ;
        const QString rwMultiRegReadFormat  ;
        const QString rwMultiRegReadCount   ;
        const QString rwMultiRegReadData    ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageReadWriteMultiRegWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    QByteArray getData() const override;
    void setData(const QByteArray &data) override;

public:
    void setModbusAddresNotation(mb::AddressNotation notation);
    
    int getWriteAddress() const;
    void setWriteAddress(int v);
    uint16_t getWriteOffset() const;
    void setWriteOffset(uint16_t v);

    uint16_t getWriteCount() const;
    void setWriteCount(uint16_t v);

    int getReadAddress() const;
    void setReadAddress(int v);
    uint16_t getReadOffset() const;
    void setReadOffset(uint16_t v);

    uint16_t getReadCount() const;
    void setReadCount(uint16_t v);

private:
    QComboBox* m_cmbWriteFormat;
    mbCoreAddressWidget* m_writeAddress;
    QSpinBox* m_spWriteCount;
    QPlainTextEdit* m_txtWriteData;
    QByteArray m_writeData;
    bool m_isWriteDirty;

    QComboBox* m_cmbReadFormat;
    mbCoreAddressWidget* m_readAddress;
    QSpinBox* m_spReadCount;
    QPlainTextEdit* m_txtReadData;
    QByteArray m_readData;
    bool m_isReadDirty;
};

#endif // CLIENT_SENDMESSAGEREADWRITEMULTIREGWIDGET_H
