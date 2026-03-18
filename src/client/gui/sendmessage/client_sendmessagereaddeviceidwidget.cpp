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
    format      (QStringLiteral("format")),
    deviceId    (QStringLiteral("deviceId")),
    objectId    (QStringLiteral("objectId")),
    conformity  (QStringLiteral("conformity")),
    nextObjectId(QStringLiteral("nextObjectId")),
    moreFollows (QStringLiteral("moreFollows")),
    data        (QStringLiteral("data"))
{
}

const mbClientSendMessageReadDeviceIdWidget::Strings &mbClientSendMessageReadDeviceIdWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageReadDeviceIdWidget::mbClientSendMessageReadDeviceIdWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(MBF_ENCAPSULATED_INTERFACE_TRANSPORT ,ui, parent)
{
    // format
    m_cmbFormat = new QComboBox(this);
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbFormat->addItem(s);
    }
    m_cmbFormat->setCurrentIndex(mb::String);

    // device id
    m_spDeviceId = new QSpinBox(this);
    m_spDeviceId->setMinimumSize(QSize(80, 0));
    m_spDeviceId->setMinimum(0);
    m_spDeviceId->setMaximum(UINT8_MAX);
    m_spDeviceId->setValue(1);

    // object id
    m_spObjectId = new QSpinBox(this);
    m_spObjectId->setMinimumSize(QSize(80, 0));
    m_spObjectId->setMinimum(0);
    m_spObjectId->setMaximum(UINT8_MAX);
    m_spObjectId->setValue(0);

    // group response
    auto grResponse = new QGroupBox(this);
    grResponse->setTitle(QCoreApplication::translate("mbClientSendMessageUi", "Response", nullptr));

    // conformity
    m_lnConformity = new QLineEdit(grResponse);
    m_lnConformity->setReadOnly(true);

    // next object id
    m_lnNextObjectId = new QLineEdit(grResponse);
    m_lnNextObjectId->setReadOnly(true);

    // more follows
    m_chbMoreFollows = new QCheckBox(grResponse);
    m_chbMoreFollows->setText(QCoreApplication::translate("mbClientSendMessageUi", "More Follows", nullptr));
    m_chbMoreFollows->setCheckable(false);

    // table read device objects
    m_tblReadDeviceObjects = new QTableWidget(grResponse);
    m_tblReadDeviceObjects->setRowCount(0);
    m_tblReadDeviceObjects->setColumnCount(2);
    m_tblReadDeviceObjects->horizontalHeader()->setStretchLastSection(true);
    auto *item0 = new QTableWidgetItem();
    item0->setText(QCoreApplication::translate("mbClientSendMessageUi", "Object Id", nullptr));
    m_tblReadDeviceObjects->setHorizontalHeaderItem(0, item0);
    auto *item1 = new QTableWidgetItem();
    item1->setText(QCoreApplication::translate("mbClientSendMessageUi", "Value", nullptr));
    m_tblReadDeviceObjects->setHorizontalHeaderItem(1, item1);

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

    setConformity(0);
    setNextObjectId(0);
    setMoreFollows(0);

    connect(m_cmbFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &mbClientSendMessageReadDeviceIdWidget::updateData);
    updateData();
}

MBSETTINGS mbClientSendMessageReadDeviceIdWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[m_prefix+s.format      ] = m_cmbFormat->currentText();
    m[m_prefix+s.deviceId    ] = getDeviceId();
    m[m_prefix+s.objectId    ] = getObjectId();
    m[m_prefix+s.conformity  ] = getConformity();
    m[m_prefix+s.nextObjectId] = getNextObjectId();
    m[m_prefix+s.moreFollows ] = getMoreFollows();
    m[m_prefix+s.data        ] = m_data;
    return m;
}

void mbClientSendMessageReadDeviceIdWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(m_prefix+s.format      ); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
    it = m.find(m_prefix+s.deviceId    ); if (it != end) setDeviceId    (it.value().toInt ());
    it = m.find(m_prefix+s.objectId    ); if (it != end) setObjectId    (it.value().toInt ());
    it = m.find(m_prefix+s.conformity  ); if (it != end) setConformity  (it.value().toInt ());
    it = m.find(m_prefix+s.nextObjectId); if (it != end) setNextObjectId(it.value().toInt ());
    it = m.find(m_prefix+s.moreFollows ); if (it != end) setMoreFollows (it.value().toBool());
    it = m.find(m_prefix+s.data        ); if (it != end) m_data = it.value().toByteArray();
    updateData();
}

void mbClientSendMessageReadDeviceIdWidget::fillParams(mbClientMessageParams &params) const
{
    params.setFormat(format());
    params.setDeviceId(getDeviceId());
    params.setObjectId(getObjectId());
}

void mbClientSendMessageReadDeviceIdWidget::setParams(mbClientMessageParams &params)
{
    setConformity(params.conformityLevel());
    setNextObjectId(params.nextObjectId());
    setMoreFollows(params.moreFollows());
    m_data = m_conv->toByteArray(params);
    updateData();
}

mb::Format mbClientSendMessageReadDeviceIdWidget::format() const
{
    return mb::enumFormatValueByIndex(m_cmbFormat->currentIndex());
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

void mbClientSendMessageReadDeviceIdWidget::updateData()
{
    auto format = mb::enumFormatValueByIndex(m_cmbFormat->currentIndex());
    auto sz = m_data.length();
    mbClientMessageParams params;
    params.setFunction(m_func);
    params.setFormat(format);
    m_tblReadDeviceObjects->setRowCount(0);
    for (int i = 0, c = 0; i+2 < sz; ++c)
    {
        uint8_t objectId = static_cast<uint8_t>(m_data[i]);
        uint8_t len = static_cast<uint8_t>(m_data[i+1]);
        if (i + 1 + len >= sz)
            len = sz - i - 2;
        QByteArray data = QByteArray::fromRawData(reinterpret_cast<const char*>(m_data.constData() + i + 2), len);
        params.setData(data);
        auto v = m_conv->toVariant(params).toString();
        m_tblReadDeviceObjects->insertRow(c);
        QTableWidgetItem *item0 = new QTableWidgetItem(mb::toHexString(objectId));
        QTableWidgetItem *item1 = new QTableWidgetItem(v);
        m_tblReadDeviceObjects->setItem(c, 0, item0);
        m_tblReadDeviceObjects->setItem(c, 1, item1);
        i += len + 2;
    }

}
