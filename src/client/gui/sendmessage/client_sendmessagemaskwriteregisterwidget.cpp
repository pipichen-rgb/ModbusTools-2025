#include "client_sendmessagemaskwriteregisterwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QFormLayout>

#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageMaskWriteRegisterWidget::Strings::Strings() :
    format          (QStringLiteral("format")),
    address         (QStringLiteral("address")),
    maskAnd         (QStringLiteral("maskAnd")),
    maskOr          (QStringLiteral("maskOr"))
{
}

const mbClientSendMessageMaskWriteRegisterWidget::Strings &mbClientSendMessageMaskWriteRegisterWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageMaskWriteRegisterWidget::mbClientSendMessageMaskWriteRegisterWidget(mbClientSendMessageUi* ui, QWidget *parent) :
    mbClientSendMessageWidget(MBF_MASK_WRITE_REGISTER, ui, parent)
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
    connect(m_cmbFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &mbClientSendMessageMaskWriteRegisterWidget::setDigitalFormat);

    // address
    m_address = new mbCoreAddressWidget(this);
    m_address->setEnabledAddressType(false);
    m_address->setAddressType(Modbus::Memory_HoldingRegisters);

    // mask and
    m_spMaskAnd = new QSpinBox(this);
    m_spMaskAnd->setMinimumSize(QSize(80, 0));
    m_spMaskAnd->setMinimum(0);
    m_spMaskAnd->setMaximum(UINT16_MAX);
    m_spMaskAnd->setValue(0);
    m_spMaskAnd->setDisplayIntegerBase(16);

    // mask or
    m_spMaskOr = new QSpinBox(this);
    m_spMaskOr->setMinimumSize(QSize(80, 0));
    m_spMaskOr->setMinimum(0);
    m_spMaskOr->setMaximum(UINT16_MAX);
    m_spMaskOr->setValue(0);
    m_spMaskOr->setDisplayIntegerBase(16);
    
    // Labels
    auto lblFormat = new QLabel(this);
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    auto lblAddress = new QLabel(this);
    lblAddress->setText(QCoreApplication::translate("mbClientSendMessageUi", "Address:", nullptr));

    auto lblMaskAnd = new QLabel(this);
    lblMaskAnd->setText(QCoreApplication::translate("mbClientSendMessageUi", "Mask AND:", nullptr));

    auto lblMaskOr = new QLabel(this);
    lblMaskOr->setText(QCoreApplication::translate("mbClientSendMessageUi", "Mask OR:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();
    formLayout->setObjectName(QString::fromUtf8("formLayout"));

    formLayout->setWidget(0, QFormLayout::LabelRole, lblFormat);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_cmbFormat);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblAddress);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_address);
    formLayout->setWidget(2, QFormLayout::LabelRole, lblMaskAnd);
    formLayout->setWidget(2, QFormLayout::FieldRole, m_spMaskAnd);
    formLayout->setWidget(3, QFormLayout::LabelRole, lblMaskOr);
    formLayout->setWidget(3, QFormLayout::FieldRole, m_spMaskOr);
    formLayout->setItem(4, QFormLayout::FieldRole, verticalSpacer);

    this->setLayout(formLayout);

    setDigitalFormat(m_cmbFormat->currentIndex());
}

MBSETTINGS mbClientSendMessageMaskWriteRegisterWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[m_prefix + s.format ] = m_cmbFormat->currentText();
    m[m_prefix + s.address] = getAddress();
    m[m_prefix + s.maskAnd] = getMaskAnd();
    m[m_prefix + s.maskOr ] = getMaskOr ();

    return m;
}

void mbClientSendMessageMaskWriteRegisterWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(m_prefix + s.format ); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
    it = m.find(m_prefix + s.address); if (it != end) setAddress(it.value().toInt());
    it = m.find(m_prefix + s.maskAnd); if (it != end) setMaskAnd(it.value().toInt());
    it = m.find(m_prefix + s.maskOr ); if (it != end) setMaskOr (it.value().toInt());
}

void mbClientSendMessageMaskWriteRegisterWidget::fillParams(mbClientMessageParams &params) const
{
    const auto digitalFormat = static_cast<mb::DigitalFormat>(m_cmbFormat->currentData().toInt());
    params.setFormat(mb::toFormat(digitalFormat));
    params.setOffset(getOffset());
    params.setMaskAnd(getMaskAnd());
    params.setMaskOr(getMaskOr());
}

void mbClientSendMessageMaskWriteRegisterWidget::setParams(mbClientMessageParams &params)
{
    if (params.hasOffset())
        setOffset(params.offset());
    if (params.hasMaskAnd())
        setMaskAnd(params.maskAnd());
    if (params.hasMaskOr())
        setMaskOr(params.maskOr());
}

int mbClientSendMessageMaskWriteRegisterWidget::getAddress() const
{
    return m_address->getAddress().number();
}

void mbClientSendMessageMaskWriteRegisterWidget::setAddress(int v)
{
    mb::Address adr = m_address->getAddress();
    adr.setNumber(v);
    m_address->setAddress(adr);
}

uint16_t mbClientSendMessageMaskWriteRegisterWidget::getOffset() const
{
    return m_address->getAddress().offset();
}

void mbClientSendMessageMaskWriteRegisterWidget::setOffset(uint16_t v)
{
    mb::Address adr = m_address->getAddress();
    adr.setOffset(v);
    m_address->setAddress(adr);
}

int mbClientSendMessageMaskWriteRegisterWidget::getMaskAnd() const
{
    return m_spMaskAnd->value();
}

void mbClientSendMessageMaskWriteRegisterWidget::setMaskAnd(int v)
{
    m_spMaskAnd->setValue(v);
}

int mbClientSendMessageMaskWriteRegisterWidget::getMaskOr() const
{
    return m_spMaskOr->value();
}

void mbClientSendMessageMaskWriteRegisterWidget::setMaskOr(int v)
{
    m_spMaskOr->setValue(v);
}

mb::DigitalFormat mbClientSendMessageMaskWriteRegisterWidget::digitalFormat() const
{
    return static_cast<mb::DigitalFormat>(m_cmbFormat->currentData().toInt());
}

void mbClientSendMessageMaskWriteRegisterWidget::setDigitalFormat(int index)
{
    Q_UNUSED(index)

    const auto digitalFormat = this->digitalFormat();
    auto maskAnd = getMaskAnd();
    auto maskOr  = getMaskOr();

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

    m_spMaskAnd->setDisplayIntegerBase(displayBase);
    m_spMaskOr ->setDisplayIntegerBase(displayBase);
    m_spMaskAnd->setMinimum(minValue);
    m_spMaskAnd->setMaximum(maxValue);
    m_spMaskOr ->setMinimum(minValue);
    m_spMaskOr ->setMaximum(maxValue);

    if (digitalFormat == mb::Dec)
    {
        m_spMaskAnd->setValue(static_cast<qint16>(maskAnd));
        m_spMaskOr ->setValue(static_cast<qint16>(maskOr ));
    }
    else
    {
        m_spMaskAnd->setValue(static_cast<quint16>(maskAnd));
        m_spMaskOr ->setValue(static_cast<quint16>(maskOr ));
    }
}
