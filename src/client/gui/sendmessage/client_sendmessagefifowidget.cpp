#include "client_sendmessagefifowidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QFormLayout>

#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageFIFOWidget::Strings::Strings() :
    prefix    (QStringLiteral("Ui.SendMessage.FIFOWidget.")),
    fifoFormat(prefix+QStringLiteral("format")),
    fifoOffset(prefix+QStringLiteral("offset"))
{
}

const mbClientSendMessageFIFOWidget::Strings &mbClientSendMessageFIFOWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageFIFOWidget::mbClientSendMessageFIFOWidget(mbClientSendMessageUi *ui, QWidget *parent) :
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

    // offset
    m_spOffset = new QSpinBox(this);
    m_spOffset->setObjectName(QString::fromUtf8("spFIFOOffset"));
    m_spOffset->setMinimumSize(QSize(80, 0));
    m_spOffset->setMinimum(0);
    m_spOffset->setMaximum(UINT16_MAX);

    // text data
    m_txtData = new QPlainTextEdit(this);
    m_txtData->setObjectName(QString::fromUtf8("txtFIFOData"));
    m_txtData->setMinimumSize(QSize(0, 100));
    m_txtData->setUndoRedoEnabled(true);
    m_txtData->setReadOnly(true);

    // Labels
    auto lblFormat = new QLabel(this);
    lblFormat->setObjectName(QString::fromUtf8("lblFIFOFormat"));
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format", nullptr));

    auto lblOffset = new QLabel(this);
    lblOffset->setObjectName(QString::fromUtf8("lblFIFOOffset"));
    lblOffset->setText(QCoreApplication::translate("mbClientSendMessageUi", "Offset", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();
    formLayout->setObjectName(QString::fromUtf8("formFIFOLayout"));

    formLayout->setWidget(0, QFormLayout::LabelRole, lblFormat);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_cmbFormat);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblOffset);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_spOffset);
    formLayout->setItem(2, QFormLayout::FieldRole, verticalSpacer);

    auto horizontalLayout = new QHBoxLayout();
    horizontalLayout->setObjectName(QString::fromUtf8("horizontalFIFOLayout"));
    horizontalLayout->addLayout(formLayout);
    horizontalLayout->addWidget(m_txtData);
    horizontalLayout->setStretch(1, 1);

    this->setLayout(horizontalLayout);
}

MBSETTINGS mbClientSendMessageFIFOWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[s.fifoFormat ] = m_cmbFormat->currentText();
    m[s.fifoOffset ] = m_spOffset ->value      ();

    return m;
}

void mbClientSendMessageFIFOWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(s.fifoFormat); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
    it = m.find(s.fifoOffset); if (it != end) m_spOffset ->setValue      (it.value().toInt()   );
}

QByteArray mbClientSendMessageFIFOWidget::getData() const
{

}

void mbClientSendMessageFIFOWidget::setData(const QByteArray &data)
{
}

uint16_t mbClientSendMessageFIFOWidget::getOffset() const
{
    return static_cast<uint16_t>(m_spOffset->value());
}
