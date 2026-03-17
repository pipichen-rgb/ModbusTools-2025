#include "client_sendmessagepggetcommeventlogwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QLineEdit>
#include <QHeaderView>
#include <QTableWidget>
#include <QHBoxLayout>
#include <QFormLayout>

#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageGetCommEventLogWidget::Strings::Strings() :
    status      (QStringLiteral("status")),
    eventCount  (QStringLiteral("eventCount")),
    messageCount(QStringLiteral("messageCount")),
    eventData   (QStringLiteral("eventData"))
{
}

const mbClientSendMessageGetCommEventLogWidget::Strings &mbClientSendMessageGetCommEventLogWidget::Strings::instance()
{
    static Strings s;
    return s;
}

QString mbClientSendMessageGetCommEventLogWidget::getEventLogDescription(uint8_t eventId)
{
    QString res;
    if (eventId & 0x80)
    {
        res = "Receive Event: ";
        QStringList ls;
        if (eventId & 0x02) ls.append("Communication Error");
        if (eventId & 0x10) ls.append("Character Overrun");
        if (eventId & 0x20) ls.append("Currently in Listen Only Mode");
        if (eventId & 0x40) ls.append("Broadcast Received");
        res += ls.join(", ");
    }
    else if (eventId & 0x40)
    {
        res = "Send Event: ";
        QStringList ls;
        if (eventId & 0x01) ls.append("Read Exception Sent");
        if (eventId & 0x02) ls.append("Server Abort Exception Sent ");
        if (eventId & 0x04) ls.append("Server Busy Exception Sent");
        if (eventId & 0x08) ls.append("Server Program NAK Exception Sent");
        if (eventId & 0x10) ls.append("Write Timeout Error Occurred");
        if (eventId & 0x20) ls.append("Currently in Listen Only Mode");
        res += ls.join(", ");
    }
    else if (eventId == 0x04)
        res = "Entered Listen Only Mode";
    else if (eventId == 0x00)
        res = "Initiated Communication Restart";
    return res;
}

mbClientSendMessageGetCommEventLogWidget::mbClientSendMessageGetCommEventLogWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(MBF_GET_COMM_EVENT_LOG, ui, parent)
{
    // status
    m_lnStatus = new QLineEdit(this);
    m_lnStatus->setReadOnly(true);

    m_lnEventCount = new QLineEdit(this);
    m_lnEventCount->setReadOnly(true);

    m_lnMessageCount = new QLineEdit(this);
    m_lnMessageCount->setReadOnly(true);

    // table
    m_tblEventLog = new QTableWidget(this);
    m_tblEventLog->setRowCount(0);
    m_tblEventLog->setColumnCount(2);
    m_tblEventLog->horizontalHeader()->setStretchLastSection(true);
    QTableWidgetItem *item0 = new QTableWidgetItem();
    item0->setText(QCoreApplication::translate("mbClientSendMessageUi", "Event Id", nullptr));
    QTableWidgetItem *item1 = new QTableWidgetItem();
    item1->setText(QCoreApplication::translate("mbClientSendMessageUi", "Description", nullptr));
    m_tblEventLog->setHorizontalHeaderItem(0, item0);
    m_tblEventLog->setHorizontalHeaderItem(1, item1);

    // Labels
    auto lblStatus = new QLabel(this);
    lblStatus->setText(QCoreApplication::translate("mbClientSendMessageUi", "Status:", nullptr));

    auto lblEventCount = new QLabel(this);
    lblEventCount->setText(QCoreApplication::translate("mbClientSendMessageUi", "Event Count:", nullptr));

    auto lblMessageCount = new QLabel(this);
    lblMessageCount->setText(QCoreApplication::translate("mbClientSendMessageUi", "Message Count:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();

    formLayout->setWidget(0, QFormLayout::LabelRole, lblStatus);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_lnStatus);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblEventCount);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_lnEventCount);
    formLayout->setWidget(2, QFormLayout::LabelRole, lblMessageCount);
    formLayout->setWidget(2, QFormLayout::FieldRole, m_lnMessageCount);
    formLayout->setItem(3, QFormLayout::FieldRole, verticalSpacer);

    auto verticalLayout = new QVBoxLayout();
    verticalLayout->addLayout(formLayout);
    verticalLayout->addWidget(m_tblEventLog);

    this->setLayout(verticalLayout);

    setStatus(0);
    setEventCount(0);
    setMessageCount(0);
}

MBSETTINGS mbClientSendMessageGetCommEventLogWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[s.status      ] = getStatus();
    m[s.eventCount  ] = getEventCount();
    m[s.messageCount] = getMessageCount();
    m[s.eventData   ] = m_eventData;
    return m;
}

void mbClientSendMessageGetCommEventLogWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(s.status      ); if (it != end) setStatus(static_cast<uint16_t>(it.value().toInt()));
    it = m.find(s.eventCount  ); if (it != end) setEventCount(static_cast<uint16_t>(it.value().toInt()));
    it = m.find(s.messageCount); if (it != end) setMessageCount(static_cast<uint16_t>(it.value().toInt()));
    it = m.find(s.eventData   ); if (it != end) m_eventData = it.value().toByteArray();
}

void mbClientSendMessageGetCommEventLogWidget::setParams(mbClientMessageParams &params)
{
    setStatus(params.status());
    setEventCount(params.eventCount());
    setMessageCount(params.messageCount());
    m_eventData = m_conv->toByteArray(params);
    updateEventLog();
}

uint16_t mbClientSendMessageGetCommEventLogWidget::getStatus() const
{
    return static_cast<uint16_t>(m_lnStatus->text().toUInt(nullptr, 16));
}

void mbClientSendMessageGetCommEventLogWidget::setStatus(uint16_t v)
{
    m_lnStatus->setText(mb::toHexString(v));
}

uint16_t mbClientSendMessageGetCommEventLogWidget::getEventCount() const
{
    return static_cast<uint16_t>(m_lnEventCount->text().toUInt());
}

void mbClientSendMessageGetCommEventLogWidget::setEventCount(uint16_t v)
{
    m_lnEventCount->setText(QString::number(v));
}

uint16_t mbClientSendMessageGetCommEventLogWidget::getMessageCount() const
{
    return static_cast<uint16_t>(m_lnMessageCount->text().toUInt());
}

void mbClientSendMessageGetCommEventLogWidget::setMessageCount(uint16_t v)
{
    m_lnMessageCount->setText(QString::number(v));
}

void mbClientSendMessageGetCommEventLogWidget::updateEventLog()
{
    m_tblEventLog->setRowCount(0);
    for (int i = 0; i < m_eventData.length(); i++)
    {
        uint8_t eventId = reinterpret_cast<const uint8_t*>(m_eventData.constData())[i];
        m_tblEventLog->insertRow(i);
        QTableWidgetItem *item0 = new QTableWidgetItem(mb::toHexString(eventId));
        QTableWidgetItem *item1 = new QTableWidgetItem(getEventLogDescription(eventId));
        m_tblEventLog->setItem(i, 0, item0);
        m_tblEventLog->setItem(i, 1, item1);
    }
}
