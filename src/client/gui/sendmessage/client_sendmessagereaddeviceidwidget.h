#ifndef CLIENT_SENDMESSAGEREADDEVICEIDWIDGET_H
#define CLIENT_SENDMESSAGEREADDEVICEIDWIDGET_H

#include "client_sendmessagewidget.h"

class QComboBox;
class QSpinBox;
class QLineEdit;
class QCheckBox;
class QTableWidget;

class mbClientSendMessageReadDeviceIdWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString prefix            ;
        const QString readDeviceId      ;
        const QString readDeviceObjectId;
        const QString readDeviceFormat  ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageReadDeviceIdWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    QByteArray getData() const override;
    void setData(const QByteArray &data) override;

public:
    uint8_t getDeviceId() const;
    void setDeviceId(uint8_t v);

    uint8_t getObjectId() const;
    void setObjectId(uint8_t v);

    uint8_t getConformity() const;
    void setConformity(uint8_t v);

    uint8_t getNextObjectId() const;
    void setNextObjectId(uint8_t v);

    bool getMoreFollows() const;
    void setMoreFollows(bool v);

private:
    QSpinBox* m_spDeviceId;
    QSpinBox* m_spObjectId;
    QComboBox* m_cmbFormat;
    QLineEdit* m_lnConformity;
    QLineEdit* m_lnNextObjectId;
    QCheckBox* m_chbMoreFollows;
    QTableWidget* m_tblReadDeviceObjects;
};

#endif // CLIENT_SENDMESSAGEREADDEVICEIDWIDGET_H
