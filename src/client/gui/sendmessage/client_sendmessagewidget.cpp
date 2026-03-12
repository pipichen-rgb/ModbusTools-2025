#include "client_sendmessagewidget.h"

mbClientSendMessageWidget::mbClientSendMessageWidget(mbClientSendMessageUi *ui, QWidget *parent)
    : QWidget{parent}
{}

MBSETTINGS mbClientSendMessageWidget::cachedSettings() const
{
    return MBSETTINGS();
}

void mbClientSendMessageWidget::setCachedSettings(const MBSETTINGS &/*settings*/)
{

}

QByteArray mbClientSendMessageWidget::getData() const
{
    return QByteArray();
}

void mbClientSendMessageWidget::setData(const QByteArray &/*data*/)
{

}
