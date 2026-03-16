#include "client_sendmessagewritesingleregisterwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QFormLayout>

#include <client.h>
#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageWriteSingleRegisterWidget::Strings::Strings() :
    format          (QStringLiteral("format")),
    address         (QStringLiteral("address")),
    value           (QStringLiteral("value"))
{
}

const mbClientSendMessageWriteSingleRegisterWidget::Strings &mbClientSendMessageWriteSingleRegisterWidget::Strings::instance()
{
    static const Strings s;
    return s;
}

mbClientSendMessageWriteSingleRegisterWidget::mbClientSendMessageWriteSingleRegisterWidget(mbClientMessageConverter* conv, QWidget *parent) :
    mbClientSendMessageWidget(MBF_WRITE_SINGLE_REGISTER, conv, parent)
{
    // format
    m_cmbFormat = new QComboBox(this);
    const auto ls = mb::enumDigitalFormatKeyList();
    for (int i = 1; i < ls.count(); ++i) // pass DefaultDigitalFormat
    {
        const auto format = static_cast<mb::DigitalFormat>(i);
        m_cmbFormat->addItem(ls.at(i), static_cast<int>(format));
    }
    m_cmbFormat->setCurrentText(mb::enumDigitalFormatKey(mb::Dec));
    connect(m_cmbFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &mbClientSendMessageWriteSingleRegisterWidget::setDigitalFormat);

    // address
    m_address = new mbCoreAddressWidget(this);
    m_address->setAddressType(Modbus::Memory_4x);
    m_address->setEnabledAddressType(false);
    connect(mbCore::globalCore(), &mbCore::addressNotationChanged, m_address, &mbCoreAddressWidget::setAddressNotation);

    // value
    m_spValue = new QSpinBox(this);
    m_spValue->setMinimumSize(QSize(100, 0));
    m_spValue->setMinimum(INT16_MIN);
    m_spValue->setMaximum(INT16_MAX);
    m_spValue->setValue(0);

    // Labels
    auto lblFormat = new QLabel(this);
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    auto lblAddress = new QLabel(this);
    lblAddress->setText(QCoreApplication::translate("mbClientSendMessageUi", "Address:", nullptr));

    auto lblValue = new QLabel(this);
    lblValue->setText(QCoreApplication::translate("mbClientSendMessageUi", "Value:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();

    formLayout->setWidget(0, QFormLayout::LabelRole, lblFormat);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_cmbFormat);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblAddress);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_address);
    formLayout->setWidget(2, QFormLayout::LabelRole, lblValue);
    formLayout->setWidget(2, QFormLayout::FieldRole, m_spValue);
    formLayout->setItem(3, QFormLayout::FieldRole, verticalSpacer);

    this->setLayout(formLayout);

    setDigitalFormat(m_cmbFormat->currentIndex());
}

MBSETTINGS mbClientSendMessageWriteSingleRegisterWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();
    MBSETTINGS m;
    m[m_prefix + s.format ] = m_cmbFormat->currentText();
    m[m_prefix + s.address] = getAddress();
    m[m_prefix + s.value  ] = m_spValue->value();

    return m;
}

void mbClientSendMessageWriteSingleRegisterWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(m_prefix + s.format ); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
    it = m.find(m_prefix + s.address); if (it != end) setAddress                  (it.value().toInt()   );
    it = m.find(m_prefix + s.value  ); if (it != end) m_spValue  ->setValue       (it.value().toInt()   );
}

void mbClientSendMessageWriteSingleRegisterWidget::fillParams(mbClientMessageParams &params) const
{
    params.setOffset(getOffset());

    const auto digitalFormat = static_cast<mb::DigitalFormat>(m_cmbFormat->currentData().toInt());
    params.setFormat(toFormat(digitalFormat));

    int base = 10;
    switch (digitalFormat)
    {
    case mb::Bin: base = 2; break;
    case mb::Oct: base = 8; break;
    case mb::Hex: base = 16; break;
    default: break;
    }

    QString value;
    if (digitalFormat == mb::Dec)
        value = QString::number(static_cast<qint16>(m_spValue->value()));
    else
        value = QString::number(static_cast<quint16>(m_spValue->value()), base);

    params.setData(value);
}

uint16_t mbClientSendMessageWriteSingleRegisterWidget::getOffset() const
{
    return m_address->getAddress().offset();
}

int mbClientSendMessageWriteSingleRegisterWidget::getAddress() const
{
    return m_address->getAddress().number();
}

void mbClientSendMessageWriteSingleRegisterWidget::setAddress(int v)
{
    mb::Address adr = m_address->getAddress();
    adr.setNumber(v);
    m_address->setAddress(adr);
}

void mbClientSendMessageWriteSingleRegisterWidget::setDigitalFormat(int index)
{
    Q_UNUSED(index)

    const auto digitalFormat = mb::enumDigitalFormatValueByIndex(index+1); // pass DefaultDigitalFormat
    const auto value16 = static_cast<quint16>(m_spValue->value());

    int minValue = 0;
    int maxValue = UINT16_MAX;
    int displayBase = 10;

    switch (digitalFormat)
    {
    case mb::Bin:
        displayBase = 2;
        break;
    case mb::Oct:
        displayBase = 8;
        break;
    case mb::Hex:
        displayBase = 16;
        break;
    case mb::Dec:
        minValue = INT16_MIN;
        maxValue = INT16_MAX;
        displayBase = 10;
        break;
    case mb::UDec:
    default:
        break;
    }

    m_spValue->setDisplayIntegerBase(displayBase);
    m_spValue->setMinimum(minValue);
    m_spValue->setMaximum(maxValue);

    if (digitalFormat == mb::Dec)
        m_spValue->setValue(static_cast<qint16>(value16));
    else
        m_spValue->setValue(value16);
}

mb::Format mbClientSendMessageWriteSingleRegisterWidget::toFormat(mb::DigitalFormat digitalFormat)
{
    switch (digitalFormat)
    {
    case mb::Bin:
        return mb::Bin16;
    case mb::Oct:
        return mb::Oct16;
    case mb::Hex:
        return mb::Hex16;
    case mb::UDec:
        return mb::UDec16;
    case mb::Dec:
    default:
        return mb::Dec16;
    }
}
