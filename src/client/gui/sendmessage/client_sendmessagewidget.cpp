#include "client_sendmessagewidget.h"

#include "client_sendmessageui.h"

mbClientSendMessageWidget::mbClientSendMessageWidget(uint8_t func, mbClientSendMessageUi *ui, QWidget *parent) :
    QWidget{parent},
    m_func{func},
    m_conv{ui->converter()}
{
    m_prefix = QString("Ui.SendMessage.FC%1.").arg(func, 2, 10, QChar('0'));
}

MBSETTINGS mbClientSendMessageWidget::cachedSettings() const
{
    return MBSETTINGS();
}

void mbClientSendMessageWidget::setCachedSettings(const MBSETTINGS &/*settings*/)
{
    // Note: Base implementation does nothing
}

void mbClientSendMessageWidget::prepareToSend()
{
    // Note: Base implementation does nothing
}

void mbClientSendMessageWidget::fillParams(mbClientMessageParams &/*params*/) const
{
    // Note: Base implementation does nothing
}

void mbClientSendMessageWidget::setParams(mbClientMessageParams &/*params*/)
{
    // Note: Base implementation does nothing
}
