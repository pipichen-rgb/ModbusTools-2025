#include "client_sendmessagepggetcommeventcounterwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QFormLayout>

#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageGetCommEventCounterWidget::Strings::Strings() :
    status(QStringLiteral("status")),
    count (QStringLiteral("count"))
{
}

const mbClientSendMessageGetCommEventCounterWidget::Strings &mbClientSendMessageGetCommEventCounterWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageGetCommEventCounterWidget::mbClientSendMessageGetCommEventCounterWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(MBF_GET_COMM_EVENT_COUNTER, ui, parent)
{
    // status
    m_lnStatus = new QLineEdit(this);
    m_lnStatus->setReadOnly(true);

    m_lnCount = new QLineEdit(this);
    m_lnCount->setReadOnly(true);

    // Labels
    auto lblStatus = new QLabel(this);
    lblStatus->setText(QCoreApplication::translate("mbClientSendMessageUi", "Status:", nullptr));

    auto lblCount = new QLabel(this);
    lblCount->setText(QCoreApplication::translate("mbClientSendMessageUi", "Count:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();
    formLayout->setObjectName(QString::fromUtf8("formGetCommEventCounterLayout"));

    formLayout->setWidget(0, QFormLayout::LabelRole, lblStatus);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_lnStatus);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblCount);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_lnCount);
    formLayout->setItem(2, QFormLayout::FieldRole, verticalSpacer);
    this->setLayout(formLayout);

    setStatus(0);
    setCount(0);
}

MBSETTINGS mbClientSendMessageGetCommEventCounterWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[s.status] = getStatus();
    m[s.count ] = getCount();
    return m;
}

void mbClientSendMessageGetCommEventCounterWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(s.status); if (it != end) setStatus(static_cast<uint16_t>(it.value().toInt()));
    it = m.find(s.count ); if (it != end) setCount(static_cast<uint16_t>(it.value().toInt()));
}

void mbClientSendMessageGetCommEventCounterWidget::setParams(mbClientMessageParams &params)
{
    setStatus(params.status());
    setCount(params.eventCount());
}

uint16_t mbClientSendMessageGetCommEventCounterWidget::getStatus() const
{
    return static_cast<uint16_t>(m_lnStatus->text().toUInt(nullptr, 16));
}

void mbClientSendMessageGetCommEventCounterWidget::setStatus(uint16_t v)
{
    m_lnStatus->setText(mb::toHexString(v));
}

uint16_t mbClientSendMessageGetCommEventCounterWidget::getCount() const
{
    return static_cast<uint16_t>(m_lnCount->text().toUInt());
}

void mbClientSendMessageGetCommEventCounterWidget::setCount(uint16_t v)
{
    m_lnCount->setText(QString::number(v));
}
