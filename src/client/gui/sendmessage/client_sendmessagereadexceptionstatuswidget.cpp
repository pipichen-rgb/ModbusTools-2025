#include "client_sendmessagereadexceptionstatuswidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QFormLayout>

#include <client.h>
#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageReadExceptionStatusWidget::Strings::Strings() :
    format          (QStringLiteral("format")),
    value           (QStringLiteral("value"))
{
}

const mbClientSendMessageReadExceptionStatusWidget::Strings &mbClientSendMessageReadExceptionStatusWidget::Strings::instance()
{
    static const Strings s;
    return s;
}

mbClientSendMessageReadExceptionStatusWidget::mbClientSendMessageReadExceptionStatusWidget(mbClientMessageConverter* conv, QWidget *parent) :
    mbClientSendMessageWidget(MBF_READ_EXCEPTION_STATUS, conv, parent)
{
    // format
    m_cmbFormat = new QComboBox(this);
    const auto ls = mb::enumDigitalFormatKeyList();
    for (int i = 1; i < ls.count(); ++i) // pass DefaultDigitalFormat
    {
        const auto format = mb::enumDigitalFormatValueByIndex(i);
        m_cmbFormat->addItem(ls.at(i), static_cast<int>(format));
    }
    m_cmbFormat->setCurrentText(mb::enumDigitalFormatKey(mb::Dec));
    connect(m_cmbFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &mbClientSendMessageReadExceptionStatusWidget::updateValue);

    // value
    m_lnValue = new QLineEdit(this);
    m_lnValue->setMinimumSize(QSize(100, 0));
    updateValue();

    // Labels
    auto lblFormat = new QLabel(this);
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    auto lblValue = new QLabel(this);
    lblValue->setText(QCoreApplication::translate("mbClientSendMessageUi", "Status:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();

    formLayout->setWidget(0, QFormLayout::LabelRole, lblFormat);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_cmbFormat);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblValue);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_lnValue);
    formLayout->setItem(2, QFormLayout::FieldRole, verticalSpacer);

    this->setLayout(formLayout);
}

MBSETTINGS mbClientSendMessageReadExceptionStatusWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();
    MBSETTINGS m;
    m[m_prefix + s.format ] = m_cmbFormat->currentText();
    m[m_prefix + s.value  ] = m_value;

    return m;
}

void mbClientSendMessageReadExceptionStatusWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(m_prefix + s.format ); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
    it = m.find(m_prefix + s.value  ); if (it != end) m_value = static_cast<quint8>(it.value().toInt());
    updateValue();
}

void mbClientSendMessageReadExceptionStatusWidget::setParams(mbClientMessageParams &params)
{
    const auto digitalFormat = static_cast<mb::DigitalFormat>(m_cmbFormat->currentData().toInt());
    params.setFormat(mb::toFormat(digitalFormat));
    QByteArray data = m_conv->toByteArray(params);
    if (data.length() == 0)
        return;
    m_value = static_cast<quint8>(data.at(0));
    updateValue();
}

mb::DigitalFormat mbClientSendMessageReadExceptionStatusWidget::digitalFormat() const
{
    auto f = m_cmbFormat->currentData().toInt();
    return static_cast<mb::DigitalFormat>(f);
}

void mbClientSendMessageReadExceptionStatusWidget::updateValue()
{
    auto format = digitalFormat();
    QString s;
    switch (format)
    {   
    case mb::Bin:
        s = mb::toBinString(m_value);
        break;
    case mb::Oct:
        s = mb::toOctString(m_value);
        break;
    case mb::Hex:
        s = mb::toHexString(m_value);
        break;
    case mb::UDec:
        s = QString::number(m_value);
        break;
    default:
        s = QString::number(static_cast<qint8>(m_value));
        break;
    }
    m_lnValue->setText(s);

}
