#ifndef CLIENT_SENDMESSAGEWIDGET_H
#define CLIENT_SENDMESSAGEWIDGET_H

#include <QWidget>
#include <client_global.h>

class mbClientSendMessageUi;

class mbClientSendMessageWidget : public QWidget
{
    Q_OBJECT
public:
    explicit mbClientSendMessageWidget(uint8_t func, mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    inline uint8_t function() const { return m_func; }
    virtual MBSETTINGS cachedSettings() const;
    virtual void setCachedSettings(const MBSETTINGS &settings);
    virtual void prepareToSend();
    virtual void fillParams(mbClientMessageParams &params) const;
    virtual void setParams(mbClientMessageParams &params);

protected:
    const uint8_t m_func;
    QString m_prefix;
    mbClientMessageConverter* m_conv;
};

#endif // CLIENT_SENDMESSAGEWIDGET_H
