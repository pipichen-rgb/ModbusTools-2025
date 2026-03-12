#ifndef CLIENT_SENDMESSAGEWIDGET_H
#define CLIENT_SENDMESSAGEWIDGET_H

#include <QWidget>
#include <mbcore.h>

class mbClientSendMessageUi;

class mbClientSendMessageWidget : public QWidget
{
    Q_OBJECT
public:
    explicit mbClientSendMessageWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    virtual MBSETTINGS cachedSettings() const;
    virtual void setCachedSettings(const MBSETTINGS &settings);
    virtual QByteArray getData() const;
    virtual void setData(const QByteArray &data);

protected:
    mbClientSendMessageUi* m_ui;
};

#endif // CLIENT_SENDMESSAGEWIDGET_H
