#include "client_sendmessagereaddeviceidwidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QHeaderView>
#include <QTableWidget>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

mbClientSendMessageReadDeviceIdWidget::Strings::Strings() :
    prefix            (QStringLiteral("Ui.SendMessage.ReadDeviceIdWidget.")),
    readDeviceId      (prefix+QStringLiteral("deviceId")),
    readDeviceObjectId(prefix+QStringLiteral("objectId")),
    readDeviceFormat  (prefix+QStringLiteral("format"))
{
}

const mbClientSendMessageReadDeviceIdWidget::Strings &mbClientSendMessageReadDeviceIdWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageReadDeviceIdWidget::mbClientSendMessageReadDeviceIdWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(ui, parent)
{
    this->setObjectName(QString::fromUtf8("pgReadDeviceId"));

    // device id
    m_spDeviceId = new QSpinBox(this);
    m_spDeviceId->setObjectName(QString::fromUtf8("spReadDeviceId"));
    m_spDeviceId->setMinimumSize(QSize(80, 0));
    m_spDeviceId->setMinimum(0);
    m_spDeviceId->setMaximum(UINT8_MAX);
    m_spDeviceId->setValue(1);

    // object id
    m_spObjectId = new QSpinBox(this);
    m_spObjectId->setObjectName(QString::fromUtf8("spReadObjectId"));
    m_spObjectId->setMinimumSize(QSize(80, 0));
    m_spObjectId->setMinimum(0);
    m_spObjectId->setMaximum(UINT8_MAX);
    m_spObjectId->setValue(0);

    // format
    m_cmbFormat = new QComboBox(this);
    m_cmbFormat->setObjectName(QString::fromUtf8("cmbReadDeviceIdFormat"));
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbFormat->addItem(s);
    }
    m_cmbFormat->setCurrentIndex(mb::Dec16);

    // group response
    auto grResponse = new QGroupBox(this);
    grResponse->setObjectName(QString::fromUtf8("grResponse")); 
    grResponse->setTitle(QCoreApplication::translate("mbClientSendMessageUi", "Response", nullptr));

    // conformity
    m_lnConformity = new QLineEdit(grResponse);
    m_lnConformity->setObjectName(QString::fromUtf8("lnReadDeviceIdConformity"));
    m_lnConformity->setReadOnly(true);

    // next object id
    m_lnNextObjectId = new QLineEdit(grResponse);
    m_lnNextObjectId->setObjectName(QString::fromUtf8("lnReadDeviceIdNextObjectId"));
    m_lnNextObjectId->setReadOnly(true);

    // more follows
    m_chbMoreFollows = new QCheckBox(grResponse);
    m_chbMoreFollows->setObjectName(QString::fromUtf8("chbReadDeviceIdMoreFollows"));
    m_chbMoreFollows->setText(QCoreApplication::translate("mbClientSendMessageUi", "More Follows", nullptr));
    m_chbMoreFollows->setCheckable(false);

    // table read device objects
    m_tblReadDeviceObjects = new QTableWidget(grResponse);
    auto *item0 = new QTableWidgetItem();
    m_tblReadDeviceObjects->setHorizontalHeaderItem(0, item0);
    auto *item1 = new QTableWidgetItem();
    m_tblReadDeviceObjects->setHorizontalHeaderItem(1, item1);
    m_tblReadDeviceObjects->setObjectName(QString::fromUtf8("tblReadDeviceObjects"));
    m_tblReadDeviceObjects->setRowCount(0);
    m_tblReadDeviceObjects->setColumnCount(2);
    m_tblReadDeviceObjects->horizontalHeader()->setStretchLastSection(true);

    // Labels
    auto lblDeviceId = new QLabel(this);
    lblDeviceId->setObjectName(QString::fromUtf8("lblReadDeviceId"));
    lblDeviceId->setText(QCoreApplication::translate("mbClientSendMessageUi", "Device ID:", nullptr));

    auto lblObjectId = new QLabel(this);
    lblObjectId->setObjectName(QString::fromUtf8("lblReadDeviceObjectId"));
    lblObjectId->setText(QCoreApplication::translate("mbClientSendMessageUi", "Object ID:", nullptr));

    auto lblFormat = new QLabel(this);
    lblFormat->setObjectName(QString::fromUtf8("lblReadDeviceIdFormat"));
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    auto lblConformity = new QLabel(this);
    lblConformity->setObjectName(QString::fromUtf8("lblReadDeviceIdConformity"));
    lblConformity->setText(QCoreApplication::translate("mbClientSendMessageUi", "Conformity:", nullptr));

    auto lblNextObjectId = new QLabel(this);
    lblNextObjectId->setObjectName(QString::fromUtf8("lblReadDeviceIdNextObjectId"));
    lblNextObjectId->setText(QCoreApplication::translate("mbClientSendMessageUi", "Next Object ID:", nullptr));

    // Spacer
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto horizontalLayout1 = new QHBoxLayout();
    horizontalLayout1->setObjectName(QString::fromUtf8("horizontalLayout1"));
    horizontalLayout1->addWidget(lblDeviceId);
    horizontalLayout1->addWidget(m_spDeviceId);
    horizontalLayout1->addWidget(lblObjectId);
    horizontalLayout1->addWidget(m_spObjectId);
    horizontalLayout1->addWidget(lblFormat);
    horizontalLayout1->addWidget(m_cmbFormat);
    horizontalLayout1->addStretch(1);

    auto horizontalLayout2 = new QHBoxLayout();
    horizontalLayout2->setObjectName(QString::fromUtf8("horizontalLayout2"));
    horizontalLayout2->addWidget(lblConformity);
    horizontalLayout2->addWidget(m_lnConformity);
    horizontalLayout2->addWidget(lblNextObjectId);
    horizontalLayout2->addWidget(m_lnNextObjectId);
    horizontalLayout2->addWidget(m_chbMoreFollows);
    horizontalLayout2->addStretch(1);

    auto verticalLayout2 = new QVBoxLayout();
    verticalLayout2->addLayout(horizontalLayout2);
    verticalLayout2->addWidget(m_tblReadDeviceObjects);
    grResponse->setLayout(verticalLayout2);

    auto verticalLayout1 = new QVBoxLayout();
    verticalLayout1->setObjectName(QString::fromUtf8("verticalLayout1"));
    verticalLayout1->addLayout(horizontalLayout1);
    verticalLayout1->addWidget(grResponse);
    this->setLayout(verticalLayout1);
}

MBSETTINGS mbClientSendMessageReadDeviceIdWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[s.readDeviceId      ] = getDeviceId();
    m[s.readDeviceObjectId] = getObjectId();
    m[s.readDeviceFormat  ] = m_cmbFormat->currentText();

    return m;
}

void mbClientSendMessageReadDeviceIdWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(s.readDeviceId      ); if (it != end) setDeviceId                (it.value().toInt()   );
    it = m.find(s.readDeviceObjectId); if (it != end) setObjectId                (it.value().toInt()   );
    it = m.find(s.readDeviceFormat  ); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
}

QByteArray mbClientSendMessageReadDeviceIdWidget::getData() const
{

}

void mbClientSendMessageReadDeviceIdWidget::setData(const QByteArray &data)
{
    /*
    auto format = mb::enumFormatValueByIndex(m_cmbFormat->currentIndex());
    auto sz = data.length();
    for (int i = 0, c = 0; i+2 < sz; ++c)
    {
        uint8_t objectId = static_cast<uint8_t>(data[i]);
        uint8_t len = static_cast<uint8_t>(data[i+1]);
        if (i + 1 + len >= sz)
            len = sz - i - 2;
        QByteArray data = QByteArray::fromRawData(reinterpret_cast<const char*>(data.constData() + i + 2), len);
        auto v = mb::toVariant(data,
                                format,
                                Modbus::Memory_0x,
                                m_dataParams.swapBytes,
                                m_dataParams.registerOrder,
                                m_dataParams.byteArrayFormat,
                                m_dataParams.stringEncoding,
                                m_dataParams.stringLengthType,
                                m_dataParams.byteArraySeparator,
                                data.count()).toString();
        m_tblReadDeviceObjects->insertRow(c);
        QTableWidgetItem *item0 = new QTableWidgetItem(mb::toHexString(objectId));
        QTableWidgetItem *item1 = new QTableWidgetItem(v);
        m_tblReadDeviceObjects->setItem(c, 0, item0);
        m_tblReadDeviceObjects->setItem(c, 1, item1);
        i += len + 2;
    }
    */
}

uint8_t mbClientSendMessageReadDeviceIdWidget::getDeviceId() const
{
    return static_cast<uint8_t>(m_spDeviceId->value());
}

void mbClientSendMessageReadDeviceIdWidget::setDeviceId(uint8_t v)
{
    m_spDeviceId->setValue(v);
}

uint8_t mbClientSendMessageReadDeviceIdWidget::getObjectId() const
{
    return static_cast<uint8_t>(m_spObjectId->value());
}

void mbClientSendMessageReadDeviceIdWidget::setObjectId(uint8_t v)
{
    m_spObjectId->setValue(v);
}

uint8_t mbClientSendMessageReadDeviceIdWidget::getConformity() const
{
    return static_cast<uint8_t>(m_lnConformity->text().toUInt());
}

void mbClientSendMessageReadDeviceIdWidget::setConformity(uint8_t v)
{
    m_lnConformity->setText(QString::number(v));
}

uint8_t mbClientSendMessageReadDeviceIdWidget::getNextObjectId() const
{
    return static_cast<uint8_t>(m_lnNextObjectId->text().toUInt());
}

void mbClientSendMessageReadDeviceIdWidget::setNextObjectId(uint8_t v)
{
    m_lnNextObjectId->setText(QString::number(v));
}

bool mbClientSendMessageReadDeviceIdWidget::getMoreFollows() const
{
    return m_chbMoreFollows->isChecked();
}

void mbClientSendMessageReadDeviceIdWidget::setMoreFollows(bool v)
{
    m_chbMoreFollows->setChecked(v);
}
