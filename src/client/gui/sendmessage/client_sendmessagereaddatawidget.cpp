#include "client_sendmessagereaddatawidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>

mbClientSendMessageReadDataWidget::Strings::Strings() :
    prefix        (QStringLiteral("Ui.SendMessage.ReadDataWidget.")),
    readDataFormat(prefix+QStringLiteral("format"))
{
}

const mbClientSendMessageReadDataWidget::Strings &mbClientSendMessageReadDataWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageReadDataWidget::mbClientSendMessageReadDataWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(ui, parent)
{
    this->setObjectName(QString::fromUtf8("pgDefault"));

    // format
    m_cmbFormat = new QComboBox(this);
    m_cmbFormat->setObjectName(QString::fromUtf8("cmbDefaultFormat"));
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbFormat->addItem(s);
    }
    m_cmbFormat->setCurrentIndex(mb::Dec16);

    // text data
    m_txtData = new QPlainTextEdit(this);
    m_txtData->setObjectName(QString::fromUtf8("m_txtData"));
    m_txtData->setMinimumSize(QSize(0, 100));
    m_txtData->setUndoRedoEnabled(true);
    m_txtData->setReadOnly(true);

    // Labels
    auto lblFormat = new QLabel(this);
    lblFormat->setObjectName(QString::fromUtf8("lblFormat"));
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto horizontalLayout = new QHBoxLayout();
    horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
    horizontalLayout->addWidget(lblFormat);
    horizontalLayout->addWidget(m_cmbFormat);
    horizontalLayout->setStretch(1, 1);

    auto verticalLayout = new QVBoxLayout();
    verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    verticalLayout->addLayout(horizontalLayout);
    verticalLayout->addWidget(m_txtData);
    verticalLayout->setStretch(1, 1);
    this->setLayout(verticalLayout);
}

MBSETTINGS mbClientSendMessageReadDataWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[s.readDataFormat ] = m_cmbFormat->currentText();

    return m;
}

void mbClientSendMessageReadDataWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(s.readDataFormat ); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
}

QByteArray mbClientSendMessageReadDataWidget::getData() const
{

}

void mbClientSendMessageReadDataWidget::setData(const QByteArray &data)
{

}
