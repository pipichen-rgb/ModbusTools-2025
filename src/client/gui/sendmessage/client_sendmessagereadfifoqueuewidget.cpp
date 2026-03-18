#include "client_sendmessagereadfifoqueuewidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QFormLayout>

#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageReadFIFOQueueWidget::Strings::Strings() :
    format(QStringLiteral("format")),
    offset(QStringLiteral("offset")),
    data(QStringLiteral("data"))
{
}

const mbClientSendMessageReadFIFOQueueWidget::Strings &mbClientSendMessageReadFIFOQueueWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageReadFIFOQueueWidget::mbClientSendMessageReadFIFOQueueWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(MBF_READ_FIFO_QUEUE, ui, parent)
{
    // format
    m_cmbFormat = new QComboBox(this);
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbFormat->addItem(s);
    }
    m_cmbFormat->setCurrentIndex(mb::Dec16);

    // offset
    m_spOffset = new QSpinBox(this);
    m_spOffset->setMinimumSize(QSize(80, 0));
    m_spOffset->setMinimum(0);
    m_spOffset->setMaximum(UINT16_MAX);

    // text data
    m_txtData = new QPlainTextEdit(this);
    m_txtData->setMinimumSize(QSize(0, 100));
    m_txtData->setUndoRedoEnabled(true);
    m_txtData->setReadOnly(true);

    // Labels
    auto lblFormat = new QLabel(this);
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    auto lblOffset = new QLabel(this);
    lblOffset->setText(QCoreApplication::translate("mbClientSendMessageUi", "Offset:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();

    formLayout->setWidget(0, QFormLayout::LabelRole, lblFormat);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_cmbFormat);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblOffset);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_spOffset);
    formLayout->setItem(2, QFormLayout::FieldRole, verticalSpacer);

    auto horizontalLayout = new QHBoxLayout();
    horizontalLayout->addLayout(formLayout);
    horizontalLayout->addWidget(m_txtData);
    horizontalLayout->setStretch(1, 1);
    this->setLayout(horizontalLayout);

    connect(m_cmbFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &mbClientSendMessageReadFIFOQueueWidget::updateData);
    updateData();

}

MBSETTINGS mbClientSendMessageReadFIFOQueueWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[m_prefix+s.format] = m_cmbFormat->currentText();
    m[m_prefix+s.offset] = getOffset();
    m[m_prefix+s.data  ] = m_data;
    return m;
}

void mbClientSendMessageReadFIFOQueueWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(m_prefix+s.format); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
    it = m.find(m_prefix+s.offset); if (it != end) setOffset(static_cast<uint16_t>(it.value().toInt()));
    it = m.find(m_prefix+s.data  ); if (it != end) m_data = it.value().toByteArray();
    updateData();
}

void mbClientSendMessageReadFIFOQueueWidget::fillParams(mbClientMessageParams &params) const
{
    params.setOffset(getOffset());
    params.setFormat(mb::enumFormatValueByIndex(m_cmbFormat->currentIndex()));
}

void mbClientSendMessageReadFIFOQueueWidget::setParams(mbClientMessageParams &params)
{
    params.setFormat(mb::enumFormatValueByIndex(m_cmbFormat->currentIndex()));
    m_data = m_conv->toByteArray(params);
    updateData();
}

mb::Format mbClientSendMessageReadFIFOQueueWidget::format() const
{
    return mb::enumFormatValueByIndex(m_cmbFormat->currentIndex());
}

uint16_t mbClientSendMessageReadFIFOQueueWidget::getOffset() const
{
    return static_cast<uint16_t>(m_spOffset->value());
}

void mbClientSendMessageReadFIFOQueueWidget::setOffset(uint16_t v)
{
    m_spOffset->setValue(static_cast<int>(v));
}

void mbClientSendMessageReadFIFOQueueWidget::updateData()
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
