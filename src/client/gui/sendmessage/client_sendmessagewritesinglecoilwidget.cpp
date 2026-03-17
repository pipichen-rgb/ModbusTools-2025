#include "client_sendmessagewritesinglecoilwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QCheckBox>
#include <QFormLayout>

#include <client.h>
#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageWriteSingleCoilWidget::Strings::Strings() :
    address         (QStringLiteral("address")),
    value           (QStringLiteral("value"))
{
}

const mbClientSendMessageWriteSingleCoilWidget::Strings &mbClientSendMessageWriteSingleCoilWidget::Strings::instance()
{
    static const Strings s;
    return s;
}

mbClientSendMessageWriteSingleCoilWidget::mbClientSendMessageWriteSingleCoilWidget(mbClientSendMessageUi* ui, QWidget *parent) :
    mbClientSendMessageWidget(MBF_WRITE_SINGLE_COIL, ui, parent)
{
    // address
    m_address = new mbCoreAddressWidget(this);
    m_address->setAddressType(Modbus::Memory_0x);
    m_address->setEnabledAddressType(false);
    connect(mbCore::globalCore(), &mbCore::addressNotationChanged, m_address, &mbCoreAddressWidget::setAddressNotation);

    // value
    m_chkValue = new QCheckBox(this);
    m_chkValue->setChecked(false);
    m_chkValue->setCheckable(true);

    // Labels
    auto lblAddress = new QLabel(this);
    lblAddress->setText(QCoreApplication::translate("mbClientSendMessageUi", "Address:", nullptr));

    auto lblValue = new QLabel(this);
    lblValue->setText(QCoreApplication::translate("mbClientSendMessageUi", "Value:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();

    formLayout->setWidget(0, QFormLayout::LabelRole, lblAddress);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_address);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblValue);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_chkValue);
    formLayout->setItem(2, QFormLayout::FieldRole, verticalSpacer);

    this->setLayout(formLayout);
}

MBSETTINGS mbClientSendMessageWriteSingleCoilWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();
    MBSETTINGS m;
    m[m_prefix + s.address] = getAddress();
    m[m_prefix + s.value  ] = m_chkValue->isChecked();

    return m;
}
    
void mbClientSendMessageWriteSingleCoilWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(m_prefix + s.address); if (it != end) setAddress                 (it.value().toInt()   );
    it = m.find(m_prefix + s.value  ); if (it != end) m_chkValue->setChecked     (it.value().toBool()  );
}

void mbClientSendMessageWriteSingleCoilWidget::fillParams(mbClientMessageParams &params) const
{
    params.setOffset(getOffset());
    QByteArray data(1, m_chkValue->isChecked());
    params.setData(data);
}

uint16_t mbClientSendMessageWriteSingleCoilWidget::getOffset() const
{
    return m_address->getAddress().offset();
}

int mbClientSendMessageWriteSingleCoilWidget::getAddress() const
{
    return m_address->getAddress().number();
}

void mbClientSendMessageWriteSingleCoilWidget::setAddress(int v)
{
    mb::Address adr = m_address->getAddress();
    adr.setNumber(v);
    m_address->setAddress(adr);
}

