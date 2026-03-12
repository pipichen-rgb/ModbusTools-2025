#include "client_sendmessagereadwritemultiregwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QFormLayout>

#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageReadWriteMultiRegWidget::Strings::Strings() :
    prefix                 (QStringLiteral("Ui.SendMessage.")),
    rwMultiRegWriteAddress (prefix+QStringLiteral("rwMultiRegWriteAddress")),
    rwMultiRegWriteFormat  (prefix+QStringLiteral("rwMultiRegWriteFormat")),
    rwMultiRegWriteCount   (prefix+QStringLiteral("rwMultiRegWriteCount")),
    rwMultiRegWriteData    (prefix+QStringLiteral("rwMultiRegWriteData")),
    rwMultiRegReadAddress  (prefix+QStringLiteral("rwMultiRegReadAddress")),
    rwMultiRegReadFormat   (prefix+QStringLiteral("rwMultiRegReadFormat")),
    rwMultiRegReadCount    (prefix+QStringLiteral("rwMultiRegReadCount")),
    rwMultiRegReadData     (prefix+QStringLiteral("rwMultiRegReadData"))
{
}

const mbClientSendMessageReadWriteMultiRegWidget::Strings &mbClientSendMessageReadWriteMultiRegWidget::Strings::instance()
{
    static const Strings s;
    return s;
}

mbClientSendMessageReadWriteMultiRegWidget::mbClientSendMessageReadWriteMultiRegWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(ui, parent)
{
    this->setObjectName(QString::fromUtf8("pgReadWriteMultiReg"));

    // write format
    m_cmbWriteFormat = new QComboBox(this);
    m_cmbWriteFormat->setObjectName(QString::fromUtf8("cmbWriteMultiRegFormat"));
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbWriteFormat->addItem(s);
    }
    m_cmbWriteFormat->setCurrentIndex(mb::Dec16);

    // write address
    m_writeAddress = new mbCoreAddressWidget(this);
    m_writeAddress->setEnabledAddressType(false);

    // write count
    m_spWriteCount = new QSpinBox(this);
    m_spWriteCount->setObjectName(QString::fromUtf8("spWriteMultiRegCount"));
    m_spWriteCount->setMinimumSize(QSize(80, 0));
    m_spWriteCount->setMinimum(1);
    m_spWriteCount->setMaximum(MB_MAX_DISCRETS); // TODO: if register was choosen than change this value

    // write text data
    m_txtWriteData = new QPlainTextEdit(this);
    m_txtWriteData->setObjectName(QString::fromUtf8("m_txtWriteMultiRegData"));
    m_txtWriteData->setMinimumSize(QSize(0, 100));
    m_txtWriteData->setUndoRedoEnabled(true);
    m_txtWriteData->setReadOnly(true);

    // read format
    m_cmbReadFormat = new QComboBox(this);
    m_cmbReadFormat->setObjectName(QString::fromUtf8("cmbReadMultiRegFormat"));
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbReadFormat->addItem(s);
    }
    m_cmbReadFormat->setCurrentIndex(mb::Dec16);

    // read address
    m_readAddress = new mbCoreAddressWidget(this);
    m_readAddress->setEnabledAddressType(false);

    // read count
    m_spReadCount = new QSpinBox(this);
    m_spReadCount->setObjectName(QString::fromUtf8("spReadMultiRegCount"));
    m_spReadCount->setMinimumSize(QSize(80, 0));
    m_spReadCount->setMinimum(1);
    m_spReadCount->setMaximum(MB_MAX_DISCRETS); // TODO: if register was choosen than change this value

    // read text data
    m_txtReadData = new QPlainTextEdit(this);
    m_txtReadData->setObjectName(QString::fromUtf8("m_txtReadMultiRegData"));
    m_txtReadData->setMinimumSize(QSize(0, 100));
    m_txtReadData->setUndoRedoEnabled(true);
    m_txtReadData->setReadOnly(true);

    // Labels
    auto lblWriteAddress = new QLabel(this);
    lblWriteAddress->setObjectName(QString::fromUtf8("lblWriteMultiRegAddress"));
    lblWriteAddress->setText(QCoreApplication::translate("mbClientSendMessageUi", "Address:", nullptr));

    auto lblWriteCount = new QLabel(this);
    lblWriteCount->setObjectName(QString::fromUtf8("lblWriteMultiRegCount"));
    lblWriteCount->setText(QCoreApplication::translate("mbClientSendMessageUi", "Count:", nullptr));

    auto lblWriteFormat = new QLabel(this);
    lblWriteFormat->setObjectName(QString::fromUtf8("lblWriteMultiRegFormat"));
    lblWriteFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    auto lblReadAddress = new QLabel(this);
    lblReadAddress->setObjectName(QString::fromUtf8("lblReadMultiRegAddress"));
    lblReadAddress->setText(QCoreApplication::translate("mbClientSendMessageUi", "Address:", nullptr));

    auto lblReadCount = new QLabel(this);
    lblReadCount->setObjectName(QString::fromUtf8("lblReadMultiRegCount"));
    lblReadCount->setText(QCoreApplication::translate("mbClientSendMessageUi", "Count:", nullptr));

    auto lblReadFormat = new QLabel(this);
    lblReadFormat->setObjectName(QString::fromUtf8("lblReadMultiRegFormat"));
    lblReadFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    // Spacer
    auto verticalWriteSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
    auto verticalReadSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto formWriteLayout = new QFormLayout();
    formWriteLayout->setObjectName(QString::fromUtf8("formWriteLayout"));
    formWriteLayout->setWidget(0, QFormLayout::LabelRole, lblWriteFormat);
    formWriteLayout->setWidget(0, QFormLayout::FieldRole, m_cmbWriteFormat);
    formWriteLayout->setWidget(1, QFormLayout::LabelRole, lblWriteAddress);
    formWriteLayout->setWidget(1, QFormLayout::FieldRole, m_writeAddress);
    formWriteLayout->setWidget(2, QFormLayout::LabelRole, lblWriteCount);
    formWriteLayout->setWidget(2, QFormLayout::FieldRole, m_spWriteCount);
    formWriteLayout->setItem(3, QFormLayout::FieldRole, verticalWriteSpacer);
    
    auto horizontalWriteLayout = new QHBoxLayout();
    horizontalWriteLayout->setObjectName(QString::fromUtf8("horizontalWriteLayout"));
    horizontalWriteLayout->addLayout(formWriteLayout);
    horizontalWriteLayout->addWidget(m_txtWriteData);
    horizontalWriteLayout->setStretch(1, 1);

    auto formReadLayout = new QFormLayout();
    formReadLayout->setObjectName(QString::fromUtf8("formReadLayout"));
    formReadLayout->setWidget(0, QFormLayout::LabelRole, lblReadFormat);
    formReadLayout->setWidget(0, QFormLayout::FieldRole, m_cmbReadFormat);
    formReadLayout->setWidget(1, QFormLayout::LabelRole, lblReadAddress);
    formReadLayout->setWidget(1, QFormLayout::FieldRole, m_readAddress);
    formReadLayout->setWidget(2, QFormLayout::LabelRole, lblReadCount);
    formReadLayout->setWidget(2, QFormLayout::FieldRole, m_spReadCount);
    formReadLayout->setItem(3, QFormLayout::FieldRole, verticalReadSpacer);
    
    auto horizontalReadLayout = new QHBoxLayout();
    horizontalReadLayout->setObjectName(QString::fromUtf8("horizontalReadLayout"));
    horizontalReadLayout->addLayout(formReadLayout);
    horizontalReadLayout->addWidget(m_txtReadData);
    horizontalReadLayout->setStretch(1, 1);

    auto verticalLayout = new QVBoxLayout();
    verticalLayout->addLayout(horizontalWriteLayout);
    verticalLayout->addLayout(horizontalReadLayout);

    this->setLayout(verticalLayout);
}

MBSETTINGS mbClientSendMessageReadWriteMultiRegWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[s.rwMultiRegWriteAddress] = getWriteAddress();
    m[s.rwMultiRegWriteFormat ] = m_cmbWriteFormat ->currentText();
    m[s.rwMultiRegWriteCount  ] = m_spWriteCount   ->value      ();
    m[s.rwMultiRegWriteData   ] = m_txtWriteData   ->toPlainText();
    m[s.rwMultiRegReadAddress ] = getReadAddress();
    m[s.rwMultiRegReadFormat  ] = m_cmbReadFormat  ->currentText();
    m[s.rwMultiRegReadCount   ] = m_spReadCount    ->value      ();
    m[s.rwMultiRegReadData    ] = m_txtReadData    ->toPlainText();

    return m;
}

void mbClientSendMessageReadWriteMultiRegWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(s.rwMultiRegWriteAddress); if (it != end) setWriteAddress                   (it.value().toInt()   );
    it = m.find(s.rwMultiRegWriteFormat ); if (it != end) m_cmbWriteFormat  ->setCurrentText(it.value().toString());
    it = m.find(s.rwMultiRegWriteCount  ); if (it != end) m_spWriteCount    ->setValue      (it.value().toInt()   );
    it = m.find(s.rwMultiRegWriteData   ); if (it != end) m_txtWriteData    ->setPlainText  (it.value().toString());
    it = m.find(s.rwMultiRegReadAddress ); if (it != end) setReadAddress                    (it.value().toInt()   );
    it = m.find(s.rwMultiRegReadFormat  ); if (it != end) m_cmbReadFormat  ->setCurrentText (it.value().toString());
    it = m.find(s.rwMultiRegReadCount   ); if (it != end) m_spReadCount    ->setValue       (it.value().toInt()   );
    it = m.find(s.rwMultiRegReadData    ); if (it != end) m_txtReadData    ->setPlainText   (it.value().toString());
}

QByteArray mbClientSendMessageReadWriteMultiRegWidget::getData() const
{

}

void mbClientSendMessageReadWriteMultiRegWidget::setData(const QByteArray &data)
{

}

int mbClientSendMessageReadWriteMultiRegWidget::getWriteAddress() const
{
    return m_writeAddress->getAddress().number();
}

void mbClientSendMessageReadWriteMultiRegWidget::setWriteAddress(int v)
{
    mb::Address adr = m_writeAddress->getAddress();
    adr.setNumber(v);
    m_writeAddress->setAddress(adr);
}

uint16_t mbClientSendMessageReadWriteMultiRegWidget::getWriteOffset() const
{
    return m_writeAddress->getAddress().offset();
}

void mbClientSendMessageReadWriteMultiRegWidget::setWriteOffset(uint16_t v)
{
    mb::Address adr = m_writeAddress->getAddress();
    adr.setOffset(v);
    m_writeAddress->setAddress(adr);
}

uint16_t mbClientSendMessageReadWriteMultiRegWidget::getWriteCount() const
{
    return static_cast<uint16_t>(m_spWriteCount->value());
}

void mbClientSendMessageReadWriteMultiRegWidget::setWriteCount(uint16_t v)
{
    m_spWriteCount->setValue(v);
}

int mbClientSendMessageReadWriteMultiRegWidget::getReadAddress() const
{
    return m_writeAddress->getAddress().number();
}

void mbClientSendMessageReadWriteMultiRegWidget::setReadAddress(int v)
{
    mb::Address adr = m_writeAddress->getAddress();
    adr.setNumber(v);
    m_writeAddress->setAddress(adr);
}

uint16_t mbClientSendMessageReadWriteMultiRegWidget::getReadOffset() const
{
    return m_writeAddress->getAddress().offset();
}

void mbClientSendMessageReadWriteMultiRegWidget::setReadOffset(uint16_t v)
{
    mb::Address adr = m_writeAddress->getAddress();
    adr.setOffset(v);
    m_writeAddress->setAddress(adr);
}

uint16_t mbClientSendMessageReadWriteMultiRegWidget::getReadCount() const
{
    return static_cast<uint16_t>(m_spReadCount->value());
}

void mbClientSendMessageReadWriteMultiRegWidget::setReadCount(uint16_t v)
{
    m_spReadCount->setValue(v);
}
