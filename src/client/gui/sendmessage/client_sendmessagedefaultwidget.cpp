#include "client_sendmessagedefaultwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QFormLayout>

#include <client.h>
#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageDefaultWidget::Strings::Strings() :
    defaultAddress         (QStringLiteral("address")),
    defaultFormat          (QStringLiteral("format")),
    defaultCount           (QStringLiteral("count")),
    defaultData            (QStringLiteral("data"))
{

}

const mbClientSendMessageDefaultWidget::Strings &mbClientSendMessageDefaultWidget::Strings::instance()
{
    static const Strings s;
    return s;
}

mbClientSendMessageDefaultWidget::mbClientSendMessageDefaultWidget(uint8_t function, mbClientMessageConverter* conv, QWidget *parent) :
    mbClientSendMessageWidget(function, conv, parent)
{
    // format
    m_cmbFormat = new QComboBox(this);
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbFormat->addItem(s);
    }
    m_cmbFormat->setCurrentIndex(mb::Dec16);

    // address
    m_address = new mbCoreAddressWidget(this);
    m_address->setEnabledAddressType(false);
    connect(mbCore::globalCore(), &mbCore::addressNotationChanged, m_address, &mbCoreAddressWidget::setAddressNotation);

    // count
    m_spCount = new QSpinBox(this);
    m_spCount->setMinimumSize(QSize(80, 0));
    m_spCount->setMinimum(1);
    m_spCount->setMaximum(MB_MAX_DISCRETS); // TODO: if register was choosen than change this value

    // text data
    m_txtData = new QPlainTextEdit(this);
    m_txtData->setMinimumSize(QSize(0, 100));
    m_txtData->setUndoRedoEnabled(true);
    m_txtData->setReadOnly(true);

    // Labels
    auto lblAddress = new QLabel(this);
    lblAddress->setText(QCoreApplication::translate("mbClientSendMessageUi", "Address:", nullptr));

    auto lblCount = new QLabel(this);
    lblCount->setText(QCoreApplication::translate("mbClientSendMessageUi", "Count:", nullptr));

    auto lblFormat = new QLabel(this);
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();

    formLayout->setWidget(0, QFormLayout::LabelRole, lblFormat);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_cmbFormat);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblAddress);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_address);
    formLayout->setWidget(2, QFormLayout::LabelRole, lblCount);
    formLayout->setWidget(2, QFormLayout::FieldRole, m_spCount);
    formLayout->setItem(3, QFormLayout::FieldRole, verticalSpacer);

    auto horizontalLayout = new QHBoxLayout();
    horizontalLayout->addLayout(formLayout);
    horizontalLayout->addWidget(m_txtData);
    horizontalLayout->setStretch(1, 1);

    this->setLayout(horizontalLayout);


}

MBSETTINGS mbClientSendMessageDefaultWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();
    MBSETTINGS m;
    m[m_prefix + s.defaultAddress] = getAddress();
    m[m_prefix + s.defaultFormat ] = m_cmbFormat->currentText();
    m[m_prefix + s.defaultCount  ] = m_spCount  ->value      ();

    return m;
}

void mbClientSendMessageDefaultWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(m_prefix + s.defaultAddress); if (it != end) setAddress                 (it.value().toInt()   );
    it = m.find(m_prefix + s.defaultFormat ); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
    it = m.find(m_prefix + s.defaultCount  ); if (it != end) m_spCount  ->setValue      (it.value().toInt()   );
    it = m.find(m_prefix + s.defaultData   ); if (it != end) m_txtData  ->setPlainText  (it.value().toString());
}

void mbClientSendMessageDefaultWidget::fillParams(mbClientMessageParams &params) const
{
    params.setOffset(getOffset());
    params.setCount(getCount());
    params.setFormat(mb::enumFormatValueByIndex(m_cmbFormat->currentIndex()));
}

uint16_t mbClientSendMessageDefaultWidget::getOffset() const
{
    return m_address->getAddress().offset();
}

int mbClientSendMessageDefaultWidget::getAddress() const
{
    return m_address->getAddress().number();
}

void mbClientSendMessageDefaultWidget::setAddress(int v)
{
    mb::Address adr = m_address->getAddress();
    adr.setNumber(v);
    m_address->setAddress(adr);
}

uint16_t mbClientSendMessageDefaultWidget::getCount() const
{
    return static_cast<uint16_t>(m_spCount->value());
}

mbClientSendMessageReadDefaultWidget::mbClientSendMessageReadDefaultWidget(uint8_t func, mbClientMessageConverter *conv, QWidget *parent)
    : mbClientSendMessageDefaultWidget(func, conv, parent)
{
    m_txtData->setReadOnly(true);
    connect(m_cmbFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &mbClientSendMessageReadDefaultWidget::setFormat);
}

void mbClientSendMessageReadDefaultWidget::setParams(mbClientMessageParams &params)
{
    mbClientSendMessageDefaultWidget::setParams(params);
    params.setFormat(mb::enumFormatValueByIndex(m_cmbFormat->currentIndex()));
    m_data = m_conv->toByteArray(params);
    m_txtData->setPlainText(m_conv->toVariant(params).toString());
}

void mbClientSendMessageReadDefaultWidget::setFormat(int index)
{
    if (m_data.isEmpty())
        return;
    mbClientMessageParams params;
    params.setFormat(mb::enumFormatValueByIndex(index));
    params.setData(m_data);
    m_txtData->setPlainText(m_conv->toVariant(params).toString());
}

mbClientSendMessageWriteDefaultWidget::mbClientSendMessageWriteDefaultWidget(uint8_t func, mbClientMessageConverter *conv, QWidget *parent)
    : mbClientSendMessageDefaultWidget(func, conv, parent)
{
    m_txtData->setReadOnly(false);
}

MBSETTINGS mbClientSendMessageWriteDefaultWidget::cachedSettings() const
{
    auto m = mbClientSendMessageDefaultWidget::cachedSettings();
    const Strings &s = Strings::instance();
    m[m_prefix + s.defaultData] = m_txtData->toPlainText();
    return m;
}

void mbClientSendMessageWriteDefaultWidget::setCachedSettings(const MBSETTINGS &settings)
{
    mbClientSendMessageDefaultWidget::setCachedSettings(settings);
    
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = settings.end();

    it = settings.find(m_prefix + s.defaultData); if (it != end) m_txtData->setPlainText(it.value().toString());
}

void mbClientSendMessageWriteDefaultWidget::fillParams(mbClientMessageParams &params) const
{
    mbClientSendMessageDefaultWidget::fillParams(params);
    params.setData(m_txtData->toPlainText());
}

mbClientSendMessageReadCoilsWidget::mbClientSendMessageReadCoilsWidget(mbClientMessageConverter *conv, QWidget *parent)
    : mbClientSendMessageReadDefaultWidget(MBF_READ_COILS, conv, parent)
{
    m_address->setAddressType(Modbus::Memory_0x);
}

mbClientSendMessageReadDiscreteInputsWidget::mbClientSendMessageReadDiscreteInputsWidget(mbClientMessageConverter *conv, QWidget *parent)
    : mbClientSendMessageReadDefaultWidget(MBF_READ_DISCRETE_INPUTS, conv, parent)
{
    m_address->setAddressType(Modbus::Memory_1x);
}

mbClientSendMessageReadInputRegistersWidget::mbClientSendMessageReadInputRegistersWidget(mbClientMessageConverter *conv, QWidget *parent)
    : mbClientSendMessageReadDefaultWidget(MBF_READ_INPUT_REGISTERS, conv, parent)
{
    m_address->setAddressType(Modbus::Memory_3x);
}

mbClientSendMessageReadHoldingRegistersWidget::mbClientSendMessageReadHoldingRegistersWidget(mbClientMessageConverter *conv, QWidget *parent)
    : mbClientSendMessageReadDefaultWidget(MBF_READ_HOLDING_REGISTERS, conv, parent)
{
    m_address->setAddressType(Modbus::Memory_4x);
}

mbClientSendMessageWriteMultipleCoilsWidget::mbClientSendMessageWriteMultipleCoilsWidget(mbClientMessageConverter *conv, QWidget *parent)
    : mbClientSendMessageWriteDefaultWidget(MBF_WRITE_MULTIPLE_COILS, conv, parent)
{
    m_address->setAddressType(Modbus::Memory_0x);
}

mbClientSendMessageWriteMultipleRegistersWidget::mbClientSendMessageWriteMultipleRegistersWidget(mbClientMessageConverter *conv, QWidget *parent)
    : mbClientSendMessageWriteDefaultWidget(MBF_WRITE_MULTIPLE_REGISTERS, conv, parent)
{
    m_address->setAddressType(Modbus::Memory_4x);
}
