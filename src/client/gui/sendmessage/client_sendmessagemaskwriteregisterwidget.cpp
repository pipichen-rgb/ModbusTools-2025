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
    prefix                 (QStringLiteral("Ui.SendMessage.WriteMaskWidget.")),
    writeMaskAddress       (prefix+QStringLiteral("writeMaskAddress")),
    writeMaskAnd           (prefix+QStringLiteral("writeMaskAnd")),
    writeMaskOr            (prefix+QStringLiteral("writeMaskOr"))
{
}

const mbClientSendMessageMaskWriteRegisterWidget::Strings &mbClientSendMessageMaskWriteRegisterWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageMaskWriteRegisterWidget::mbClientSendMessageMaskWriteRegisterWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(ui, parent)
{
    this->setObjectName(QString::fromUtf8("pgWriteMask"));

    // address
    m_address = new mbCoreAddressWidget(this);
    m_address->setEnabledAddressType(false);
    m_address->setAddressType(Modbus::Memory_HoldingRegisters);

    // mask and
    m_spMaskAnd = new QSpinBox(this);
    m_spMaskAnd->setObjectName(QString::fromUtf8("spMaskAnd"));
    m_spMaskAnd->setMinimumSize(QSize(80, 0));
    m_spMaskAnd->setMinimum(0);
    m_spMaskAnd->setMaximum(UINT16_MAX);
    m_spMaskAnd->setValue(0);
    m_spMaskAnd->setDisplayIntegerBase(16);

    // mask or
    m_spMaskOr = new QSpinBox(this);
    m_spMaskOr->setObjectName(QString::fromUtf8("spMaskOr"));
    m_spMaskOr->setMinimumSize(QSize(80, 0));
    m_spMaskOr->setMinimum(0);
    m_spMaskOr->setMaximum(UINT16_MAX);
    m_spMaskOr->setValue(0);
    m_spMaskOr->setDisplayIntegerBase(16);
    
    // Labels
    auto lblAddress = new QLabel(this);
    lblAddress->setObjectName(QString::fromUtf8("lblAddress"));
    lblAddress->setText(QCoreApplication::translate("mbClientSendMessageUi", "Address:", nullptr));

    auto lblMaskAnd = new QLabel(this);
    lblMaskAnd->setObjectName(QString::fromUtf8("lblMaskAnd")); 
    lblMaskAnd->setText(QCoreApplication::translate("mbClientSendMessageUi", "Mask AND(Hex):", nullptr));

    auto lblMaskOr = new QLabel(this);
    lblMaskOr->setObjectName(QString::fromUtf8("lblMaskOr"));
    lblMaskOr->setText(QCoreApplication::translate("mbClientSendMessageUi", "Mask OR:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();
    formLayout->setObjectName(QString::fromUtf8("formLayout"));

    formLayout->setWidget(0, QFormLayout::LabelRole, lblAddress);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_address);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblMaskAnd);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_spMaskAnd);
    formLayout->setWidget(2, QFormLayout::LabelRole, lblMaskOr);
    formLayout->setWidget(2, QFormLayout::FieldRole, m_spMaskOr);
    formLayout->setItem(3, QFormLayout::FieldRole, verticalSpacer);

    this->setLayout(formLayout);
}

MBSETTINGS mbClientSendMessageMaskWriteRegisterWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[s.writeMaskAddress      ] = getAddress();
    m[s.writeMaskAnd          ] = m_spMaskAnd->value();
    m[s.writeMaskOr           ] = m_spMaskOr ->value();

    return m;
}

void mbClientSendMessageMaskWriteRegisterWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(s.writeMaskAddress); if (it != end) setAddress             (it.value().toInt()   );
    it = m.find(s.writeMaskAnd    ); if (it != end) m_spMaskAnd->setValue(it.value().toInt()   );
    it = m.find(s.writeMaskOr     ); if (it != end) m_spMaskOr ->setValue(it.value().toInt()   );
}

QByteArray mbClientSendMessageMaskWriteRegisterWidget::getData() const
{

}

void mbClientSendMessageMaskWriteRegisterWidget::setData(const QByteArray &data)
{

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

uint16_t mbClientSendMessageMaskWriteRegisterWidget::getMaskAnd() const
{
    return static_cast<uint16_t>(m_spMaskAnd->value());
}

void mbClientSendMessageMaskWriteRegisterWidget::setMaskAnd(uint16_t v)
{
    m_spMaskAnd->setValue(v);
}

uint16_t mbClientSendMessageMaskWriteRegisterWidget::getMaskOr() const
{
    return static_cast<uint16_t>(m_spMaskOr->value());
}

void mbClientSendMessageMaskWriteRegisterWidget::setMaskOr(uint16_t v)
{
    m_spMaskOr->setValue(v);
}
