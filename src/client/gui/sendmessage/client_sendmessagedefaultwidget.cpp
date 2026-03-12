#include "client_sendmessagedefaultwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QFormLayout>

#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageDefaultWidget::Strings::Strings() :
    prefix                 (QStringLiteral("Ui.SendMessage.DefaultWidget.")),
    defaultAddress         (prefix+QStringLiteral("address")),
    defaultFormat          (prefix+QStringLiteral("format")),
    defaultCount           (prefix+QStringLiteral("count")),
    defaultData            (prefix+QStringLiteral("data"))
{

}

const mbClientSendMessageDefaultWidget::Strings &mbClientSendMessageDefaultWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageDefaultWidget::mbClientSendMessageDefaultWidget(mbClientSendMessageUi *ui, QWidget *parent) :
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

    // address
    m_address = new mbCoreAddressWidget(this);
    m_address->setEnabledAddressType(false);

    // count
    m_spCount = new QSpinBox(this);
    m_spCount->setObjectName(QString::fromUtf8("spDefaultCount"));
    m_spCount->setMinimumSize(QSize(80, 0));
    m_spCount->setMinimum(1);
    m_spCount->setMaximum(MB_MAX_DISCRETS); // TODO: if register was choosen than change this value

    // text data
    m_txtData = new QPlainTextEdit(this);
    m_txtData->setObjectName(QString::fromUtf8("m_txtData"));
    m_txtData->setMinimumSize(QSize(0, 100));
    m_txtData->setUndoRedoEnabled(true);
    m_txtData->setReadOnly(true);

    // Labels
    auto lblAddress = new QLabel(this);
    lblAddress->setObjectName(QString::fromUtf8("lblAddress"));
    lblAddress->setText(QCoreApplication::translate("mbClientSendMessageUi", "Address:", nullptr));

    auto lblCount = new QLabel(this);
    lblCount->setObjectName(QString::fromUtf8("lblCount"));
    lblCount->setText(QCoreApplication::translate("mbClientSendMessageUi", "Count:", nullptr));

    auto lblFormat = new QLabel(this);
    lblFormat->setObjectName(QString::fromUtf8("lblFormat"));
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formLayout = new QFormLayout();
    formLayout->setObjectName(QString::fromUtf8("formLayout"));

    formLayout->setWidget(0, QFormLayout::LabelRole, lblFormat);
    formLayout->setWidget(0, QFormLayout::FieldRole, m_cmbFormat);
    formLayout->setWidget(1, QFormLayout::LabelRole, lblAddress);
    formLayout->setWidget(1, QFormLayout::FieldRole, m_address);
    formLayout->setWidget(2, QFormLayout::LabelRole, lblCount);
    formLayout->setWidget(2, QFormLayout::FieldRole, m_spCount);
    formLayout->setItem(3, QFormLayout::FieldRole, verticalSpacer);

    auto horizontalLayout = new QHBoxLayout();
    horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
    horizontalLayout->addLayout(formLayout);
    horizontalLayout->addWidget(m_txtData);
    horizontalLayout->setStretch(1, 1);

    this->setLayout(horizontalLayout);
}

MBSETTINGS mbClientSendMessageDefaultWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[s.defaultAddress] = getAddress();
    m[s.defaultFormat ] = m_cmbFormat->currentText();
    m[s.defaultCount  ] = m_spCount  ->value      ();
    m[s.defaultData   ] = m_txtData  ->toPlainText();

    return m;
}

void mbClientSendMessageDefaultWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(s.defaultAddress); if (it != end) setAddress                 (it.value().toInt()   );
    it = m.find(s.defaultFormat ); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
    it = m.find(s.defaultCount  ); if (it != end) m_spCount  ->setValue      (it.value().toInt()   );
    it = m.find(s.defaultData   ); if (it != end) m_txtData  ->setPlainText  (it.value().toString());
}

QByteArray mbClientSendMessageDefaultWidget::getData() const
{

}

void mbClientSendMessageDefaultWidget::setData(const QByteArray &data)
{

}

uint16_t mbClientSendMessageDefaultWidget::getOffset() const
{
    return m_address->getAddress().offset();
}

uint16_t mbClientSendMessageDefaultWidget::getCount() const
{
    return static_cast<uint16_t>(m_spCount->value());
}

int mbClientSendMessageDefaultWidget::getAddress() const
{
    return m_address->getAddress().number();
}

void mbClientSendMessageDefaultWidget::setAddress(int v)
{
    mb::Address adr = m_address->getAddress();
    adr.setNumber(v);
    m_address->setAddress(adr);
}
