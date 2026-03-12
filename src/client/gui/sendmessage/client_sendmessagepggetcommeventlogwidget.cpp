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
    prefix(QStringLiteral("Ui.SendMessage.GetCommEventLogWidget."))
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

mbClientSendMessageGetCommEventLogWidget::mbClientSendMessageGetCommEventLogWidget(mbClientSendMessageUi *ui, QWidget *parent) : mbClientSendMessageWidget(ui, parent)
{
    this->setObjectName(QString::fromUtf8("pgGetCommEventLog"));

    // status
    m_lnStatus = new QLineEdit(this);
    m_lnStatus->setObjectName(QString::fromUtf8("lnGetCommEventLogStatus"));
    m_lnStatus->setReadOnly(true);

    m_lnEventCount = new QLineEdit(this);
    m_lnEventCount->setObjectName(QString::fromUtf8("lnGetCommEventLogEventCount"));
    m_lnEventCount->setReadOnly(true);

    m_lnMessageCount = new QLineEdit(this);
    m_lnMessageCount->setObjectName(QString::fromUtf8("lnGetCommEventLogMessageCount"));
    m_lnMessageCount->setReadOnly(true);

    // table
    m_tblEventLog = new QTableWidget(this);
    m_tblEventLog->setObjectName(QString::fromUtf8("tblEventLog"));
    QTableWidgetItem *item0 = new QTableWidgetItem();
    item0->setText(QCoreApplication::translate("mbClientSendMessageUi", "Event Id", nullptr));
    QTableWidgetItem *item1 = new QTableWidgetItem();
    item1->setText(QCoreApplication::translate("mbClientSendMessageUi", "Description", nullptr));  
    m_tblEventLog->setHorizontalHeaderItem(0, item0);
    m_tblEventLog->setHorizontalHeaderItem(1, item1);
    m_tblEventLog->setRowCount(0);
    m_tblEventLog->setColumnCount(2);
    m_tblEventLog->horizontalHeader()->setStretchLastSection(true); 

    // Labels
    auto lblStatus = new QLabel(this);
    lblStatus->setObjectName(QString::fromUtf8("lblGetCommEventLogStatus"));
    lblStatus->setText(QCoreApplication::translate("mbClientSendMessageUi", "Status:", nullptr));

    auto lblEventCount = new QLabel(this);
    lblEventCount->setObjectName(QString::fromUtf8("lblGetCommEventLogEventCount"));
    lblEventCount->setText(QCoreApplication::translate("mbClientSendMessageUi", "Event Count:", nullptr));

    auto lblMessageCount = new QLabel(this);
    lblMessageCount->setObjectName(QString::fromUtf8("lblGetCommEventLogMessageCount"));
    lblMessageCount->setText(QCoreApplication::translate("mbClientSendMessageUi", "Message Count:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();
    formLayout->setObjectName(QString::fromUtf8("formGetCommEventLogLayout"));

    formLayout->setWidget(0, QFormLayout::LabelRole, lblStatus);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_lnStatus);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblEventCount);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_lnEventCount);
    formLayout->setWidget(2, QFormLayout::LabelRole, lblMessageCount);
    formLayout->setWidget(2, QFormLayout::FieldRole, m_lnMessageCount);
    formLayout->setItem(3, QFormLayout::FieldRole, verticalSpacer);

    this->setLayout(formLayout);
}

QByteArray mbClientSendMessageGetCommEventLogWidget::getData() const
{

}

void mbClientSendMessageGetCommEventLogWidget::setData(const QByteArray &data)
{
    m_tblEventLog->clearContents();
    for (int i = 0; i < data.length(); i++)
    {
        uint8_t eventId = reinterpret_cast<const uint8_t*>(data.constData())[i];
        m_tblEventLog->insertRow(i);
        QTableWidgetItem *item0 = new QTableWidgetItem(mb::toHexString(eventId));
        QTableWidgetItem *item1 = new QTableWidgetItem(getEventLogDescription(eventId));
        m_tblEventLog->setItem(i, 0, item0);
        m_tblEventLog->setItem(i, 1, item1);
    }
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
