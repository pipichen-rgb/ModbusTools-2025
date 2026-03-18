#include "client_sendmessagereadwritemultipleregisterswidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QPlainTextEdit>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QFormLayout>

#include <gui/widgets/core_addresswidget.h>

mbClientSendMessageReadWriteMultipleRegistersWidget::Strings::Strings() :
    writeFormat (QStringLiteral("writeFormat")),
    writeAddress(QStringLiteral("writeAddress")),
    writeCount  (QStringLiteral("writeCount")),
    writeData   (QStringLiteral("writeData")),
    readFormat  (QStringLiteral("readFormat")),
    readAddress (QStringLiteral("readAddress")),
    readCount   (QStringLiteral("readCount")),
    readData    (QStringLiteral("readData"))
{
}

const mbClientSendMessageReadWriteMultipleRegistersWidget::Strings &mbClientSendMessageReadWriteMultipleRegistersWidget::Strings::instance()
{
    static const Strings s;
    return s;
}

mbClientSendMessageReadWriteMultipleRegistersWidget::mbClientSendMessageReadWriteMultipleRegistersWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(MBF_READ_WRITE_MULTIPLE_REGISTERS, ui, parent)
{
    // write format
    m_cmbWriteFormat = new QComboBox(this);
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbWriteFormat->addItem(s);
    }
    m_cmbWriteFormat->setCurrentIndex(mb::Dec16);

    // write address
    m_writeAddress = new mbCoreAddressWidget(this);
    m_writeAddress->setAddressType(Modbus::Memory_4x);
    m_writeAddress->setEnabledAddressType(false);

    // write count
    m_spWriteCount = new QSpinBox(this);
    m_spWriteCount->setMinimumSize(QSize(80, 0));
    m_spWriteCount->setMinimum(1);
    m_spWriteCount->setMaximum(MB_MAX_REGISTERS);

    // write text data
    m_txtWriteData = new QPlainTextEdit(this);
    m_txtWriteData->setMinimumSize(QSize(0, 100));
    m_txtWriteData->setUndoRedoEnabled(true);
    m_txtWriteData->setReadOnly(false);

    // read format
    m_cmbReadFormat = new QComboBox(this);
    ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbReadFormat->addItem(s);
    }
    m_cmbReadFormat->setCurrentIndex(mb::Dec16);

    // read address
    m_readAddress = new mbCoreAddressWidget(this);
    m_readAddress->setAddressType(Modbus::Memory_4x);
    m_readAddress->setEnabledAddressType(false);

    // read count
    m_spReadCount = new QSpinBox(this);
    m_spReadCount->setMinimumSize(QSize(80, 0));
    m_spReadCount->setMinimum(1);
    m_spReadCount->setMaximum(MB_MAX_REGISTERS);

    // read text data
    m_txtReadData = new QPlainTextEdit(this);
    m_txtReadData->setMinimumSize(QSize(0, 100));
    m_txtReadData->setUndoRedoEnabled(true);
    m_txtReadData->setReadOnly(true);

    // Labels
    auto lblWriteAddress = new QLabel(this);
    lblWriteAddress->setText(QCoreApplication::translate("mbClientSendMessageUi", "Address:", nullptr));

    auto lblWriteCount = new QLabel(this);
    lblWriteCount->setText(QCoreApplication::translate("mbClientSendMessageUi", "Count:", nullptr));

    auto lblWriteFormat = new QLabel(this);
    lblWriteFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    auto lblReadAddress = new QLabel(this);
    lblReadAddress->setText(QCoreApplication::translate("mbClientSendMessageUi", "Address:", nullptr));

    auto lblReadCount = new QLabel(this);
    lblReadCount->setText(QCoreApplication::translate("mbClientSendMessageUi", "Count:", nullptr));

    auto lblReadFormat = new QLabel(this);
    lblReadFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    // Groups
    auto groupWrite = new QGroupBox(this);
    groupWrite->setTitle(QCoreApplication::translate("mbClientSendMessageUi", "Write:", nullptr));

    auto groupRead = new QGroupBox(this);
    groupRead->setTitle(QCoreApplication::translate("mbClientSendMessageUi", "Read:", nullptr));

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

    groupWrite->setLayout(formWriteLayout);
    
    auto horizontalWriteLayout = new QHBoxLayout();
    horizontalWriteLayout->setObjectName(QString::fromUtf8("horizontalWriteLayout"));
    horizontalWriteLayout->addWidget(groupWrite);
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
    
    groupRead->setLayout(formReadLayout);

    auto horizontalReadLayout = new QHBoxLayout();
    horizontalReadLayout->setObjectName(QString::fromUtf8("horizontalReadLayout"));
    horizontalReadLayout->addWidget(groupRead);
    horizontalReadLayout->addWidget(m_txtReadData);
    horizontalReadLayout->setStretch(1, 1);

    auto verticalLayout = new QVBoxLayout();
    verticalLayout->addLayout(horizontalWriteLayout);
    verticalLayout->addLayout(horizontalReadLayout);

    this->setLayout(verticalLayout);

    connect(m_cmbReadFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &mbClientSendMessageReadWriteMultipleRegistersWidget::updateReadData);
    updateReadData();
}

MBSETTINGS mbClientSendMessageReadWriteMultipleRegistersWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[m_prefix+s.writeFormat ] = m_cmbWriteFormat ->currentText();
    m[m_prefix+s.writeAddress] = getWriteAddress();
    m[m_prefix+s.writeCount  ] = m_spWriteCount   ->value      ();
    m[m_prefix+s.writeData   ] = m_txtWriteData   ->toPlainText();
    m[m_prefix+s.readFormat  ] = m_cmbReadFormat  ->currentText();
    m[m_prefix+s.readAddress ] = getReadAddress();
    m[m_prefix+s.readCount   ] = m_spReadCount    ->value      ();
    m[m_prefix+s.readData    ] = m_readData                      ;

    return m;
}

void mbClientSendMessageReadWriteMultipleRegistersWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(m_prefix+s.writeFormat ); if (it != end) setWriteAddress                   (it.value().toInt()   );
    it = m.find(m_prefix+s.writeAddress); if (it != end) m_cmbWriteFormat  ->setCurrentText(it.value().toString());
    it = m.find(m_prefix+s.writeCount  ); if (it != end) setWriteCount      (it.value().toInt()   );
    it = m.find(m_prefix+s.writeData   ); if (it != end) m_txtWriteData    ->setPlainText  (it.value().toString());
    it = m.find(m_prefix+s.readFormat  ); if (it != end) setReadAddress                    (it.value().toInt()   );
    it = m.find(m_prefix+s.readAddress ); if (it != end) m_cmbReadFormat  ->setCurrentText (it.value().toString());
    it = m.find(m_prefix+s.readCount   ); if (it != end) setReadCount                      (it.value().toInt()   );
    it = m.find(m_prefix+s.readData    ); if (it != end) m_readData = it.value().toByteArray();
    updateReadData();
}

void mbClientSendMessageReadWriteMultipleRegistersWidget::fillParams(mbClientMessageParams &params) const
{
    params.setFormat(writeFormat());
    params.setWriteOffset(getWriteOffset());
    params.setWriteCount(getWriteCount());
    params.setOffset(getReadOffset());
    params.setCount(getReadCount());
    params.setData(m_txtWriteData->toPlainText());
}

void mbClientSendMessageReadWriteMultipleRegistersWidget::setParams(mbClientMessageParams &params)
{
    params.setFormat(readFormat());
    m_readData = m_conv->toByteArray(params);
    updateReadData();
}

mb::Format mbClientSendMessageReadWriteMultipleRegistersWidget::writeFormat() const
{
    return mb::enumFormatValueByIndex(m_cmbWriteFormat->currentIndex());
}

int mbClientSendMessageReadWriteMultipleRegistersWidget::getWriteAddress() const
{
    return m_writeAddress->getAddress().number();
}

void mbClientSendMessageReadWriteMultipleRegistersWidget::setWriteAddress(int v)
{
    mb::Address adr = m_writeAddress->getAddress();
    adr.setNumber(v);
    m_writeAddress->setAddress(adr);
}

uint16_t mbClientSendMessageReadWriteMultipleRegistersWidget::getWriteOffset() const
{
    return m_writeAddress->getAddress().offset();
}

void mbClientSendMessageReadWriteMultipleRegistersWidget::setWriteOffset(uint16_t v)
{
    mb::Address adr = m_writeAddress->getAddress();
    adr.setOffset(v);
    m_writeAddress->setAddress(adr);
}

uint16_t mbClientSendMessageReadWriteMultipleRegistersWidget::getWriteCount() const
{
    return static_cast<uint16_t>(m_spWriteCount->value());
}

void mbClientSendMessageReadWriteMultipleRegistersWidget::setWriteCount(uint16_t v)
{
    m_spWriteCount->setValue(v);
}

mb::Format mbClientSendMessageReadWriteMultipleRegistersWidget::readFormat() const
{
    return mb::enumFormatValueByIndex(m_cmbReadFormat->currentIndex());
}

int mbClientSendMessageReadWriteMultipleRegistersWidget::getReadAddress() const
{
    return m_readAddress->getAddress().number();
}

void mbClientSendMessageReadWriteMultipleRegistersWidget::setReadAddress(int v)
{
    mb::Address adr = m_readAddress->getAddress();
    adr.setNumber(v);
    m_readAddress->setAddress(adr);
}

uint16_t mbClientSendMessageReadWriteMultipleRegistersWidget::getReadOffset() const
{
    return m_readAddress->getAddress().offset();
}

void mbClientSendMessageReadWriteMultipleRegistersWidget::setReadOffset(uint16_t v)
{
    mb::Address adr = m_readAddress->getAddress();
    adr.setOffset(v);
    m_readAddress->setAddress(adr);
}

uint16_t mbClientSendMessageReadWriteMultipleRegistersWidget::getReadCount() const
{
    return static_cast<uint16_t>(m_spReadCount->value());
}

void mbClientSendMessageReadWriteMultipleRegistersWidget::setReadCount(uint16_t v)
{
    m_spReadCount->setValue(v);
}

void mbClientSendMessageReadWriteMultipleRegistersWidget::updateReadData()
{
    if (m_readData.length() == 0)
    {
        m_txtReadData->setPlainText(QString());
        return;
    }
    mbClientMessageParams params;
    params.setFunction(function());
    params.setFormat(readFormat());
    params.setCount(m_readData.length()/2);
    params.setData(m_readData);
    QString s = m_conv->toVariant(params).toString();
    m_txtReadData->setPlainText(s);
}
