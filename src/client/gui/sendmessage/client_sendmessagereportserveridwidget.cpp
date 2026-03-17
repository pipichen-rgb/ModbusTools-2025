#include "client_sendmessagereportserveridwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>

mbClientSendMessageReportServerIdWidget::Strings::Strings() :
    format(QStringLiteral("format")),
    data  (QStringLiteral("data"))
{
}

const mbClientSendMessageReportServerIdWidget::Strings &mbClientSendMessageReportServerIdWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageReportServerIdWidget::mbClientSendMessageReportServerIdWidget(mbClientSendMessageUi* ui, QWidget *parent) :
    mbClientSendMessageWidget(MBF_REPORT_SERVER_ID, ui, parent)
{
    // format
    m_cmbFormat = new QComboBox(this);
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbFormat->addItem(s);
    }
    m_cmbFormat->setCurrentIndex(mb::Dec16);

    // text data
    m_txtData = new QPlainTextEdit(this);
    m_txtData->setMinimumSize(QSize(0, 100));
    m_txtData->setReadOnly(true);

    // Labels
    auto lblFormat = new QLabel(this);
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    // Layouts
    auto horizontalLayout = new QHBoxLayout();
    horizontalLayout->addWidget(lblFormat);
    horizontalLayout->addWidget(m_cmbFormat);
    horizontalLayout->setStretch(1, 1);

    auto verticalLayout = new QVBoxLayout();
    verticalLayout->addLayout(horizontalLayout);
    verticalLayout->addWidget(m_txtData);
    verticalLayout->setStretch(1, 1);
    this->setLayout(verticalLayout);

    connect(m_cmbFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &mbClientSendMessageReportServerIdWidget::updateData);
    updateData();
}

MBSETTINGS mbClientSendMessageReportServerIdWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[s.format] = m_cmbFormat->currentText();
    m[s.data  ] = m_data;

    return m;
}

void mbClientSendMessageReportServerIdWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(s.format); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
    it = m.find(s.data  ); if (it != end) m_data = it.value().toByteArray();
    updateData();
}

void mbClientSendMessageReportServerIdWidget::setParams(mbClientMessageParams &params)
{
    params.setFormat(format());
    m_data = m_conv->toByteArray(params);
    updateData();
}

mb::Format mbClientSendMessageReportServerIdWidget::format() const
{
    return mb::enumFormatValueByIndex(m_cmbFormat->currentIndex());
}

void mbClientSendMessageReportServerIdWidget::updateData()
{
    if (m_data.length() == 0)
    {
        m_txtData->setPlainText(QString());
        return;
    }
    mbClientMessageParams params;
    params.setFunction(function());
    params.setFormat(format());
    params.setCount(m_data.length()*8);
    params.setData(m_data);
    QString s = m_conv->toVariant(params).toString();
    m_txtData->setPlainText(s);
}
