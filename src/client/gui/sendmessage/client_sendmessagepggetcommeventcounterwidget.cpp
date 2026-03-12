#include "client_sendmessagepggetcommeventcounterwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QFormLayout>

#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageGetCommEventCounterWidget::Strings::Strings() :
    prefix(QStringLiteral("Ui.SendMessage.GetCommEventCounterWidget."))
{
}

const mbClientSendMessageGetCommEventCounterWidget::Strings &mbClientSendMessageGetCommEventCounterWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageGetCommEventCounterWidget::mbClientSendMessageGetCommEventCounterWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(ui, parent)
{
    this->setObjectName(QString::fromUtf8("pgGetCommEventCounter"));

    // status
    m_lnStatus = new QLineEdit(this);
    m_lnStatus->setObjectName(QString::fromUtf8("lnGetCommEventCounterStatus"));
    m_lnStatus->setReadOnly(true);

    m_lnCount = new QLineEdit(this);
    m_lnCount->setObjectName(QString::fromUtf8("lnGetCommEventCounterCount"));
    m_lnCount->setReadOnly(true);

    // Labels
    auto lblStatus = new QLabel(this);
    lblStatus->setObjectName(QString::fromUtf8("lblGetCommEventCounterStatus"));
    lblStatus->setText(QCoreApplication::translate("mbClientSendMessageUi", "Status:", nullptr));

    auto lblCount = new QLabel(this);
    lblCount->setObjectName(QString::fromUtf8("lblGetCommEventCounterCount"));
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
