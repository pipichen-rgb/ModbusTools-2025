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
        const QString format  ;
        const QString deviceId;
        const QString objectId;
        const QString conformity;
        const QString nextObjectId;
        const QString moreFollows;
        const QString data;

        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageReadDeviceIdWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void fillParams(mbClientMessageParams &params) const override;
    void setParams(mbClientMessageParams &params) override;

public:
    mb::Format format() const;
    
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

private Q_SLOTS:
    void updateData();

private:
    QByteArray m_data;
    QComboBox* m_cmbFormat;
    QSpinBox* m_spDeviceId;
    QSpinBox* m_spObjectId;
    QLineEdit* m_lnConformity;
    QLineEdit* m_lnNextObjectId;
    QCheckBox* m_chbMoreFollows;
    QTableWidget* m_tblReadDeviceObjects;
};

#endif // CLIENT_SENDMESSAGEREADDEVICEIDWIDGET_H
