#include "client_sendmessagefilerecordswidget.h"

#include <QCoreApplication>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QHeaderView>
#include <QTableView>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <gui/widgets/core_addresswidget.h>
#include "client_sendmessageui.h"

struct mbClientSendMessageFileRecordsModel::Item
{
    Modbus::FileRecord file;
    QByteArray data;
    QString cache;

    QByteArray getData() const
    {
        auto expectedSize = file.recordLength*2;
        auto datalen = data.length();
        if (datalen != expectedSize)
        {
            QByteArray b = data;
            b.resize(expectedSize);
            if (expectedSize > datalen)
                memset(b.data()+datalen, 0, expectedSize-datalen);
            return b;
        }
        return data;
    }
};

mbClientSendMessageFileRecordsModel::mbClientSendMessageFileRecordsModel(mbClientMessageConverter* conv, QObject *parent)
    : QAbstractTableModel(parent),
    m_editMode(false),
    m_conv(conv),
    m_addressFormat(mb::Hex),
    m_format(mb::ByteArray)
{

}

mbClientSendMessageFileRecordsModel::~mbClientSendMessageFileRecordsModel()
{
    qDeleteAll(m_items);
}

int mbClientSendMessageFileRecordsModel::columnCount(const QModelIndex &/*parent*/) const
{
    return ColumnCount;
}

int mbClientSendMessageFileRecordsModel::rowCount(const QModelIndex &/*parent*/) const
{
    return m_items.count();
}

QVariant mbClientSendMessageFileRecordsModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    switch (role)
    {
    case Qt::DisplayRole:
        switch (orientation)
        {
        case Qt::Vertical:
            return section+1;
        case Qt::Horizontal:
            switch(section)
            {
            case Column_File:
                return QStringLiteral("File");
            case Column_Record:
                return QStringLiteral("Record");
            case Column_Length:
                return QStringLiteral("Len (reg)");
            case Column_Data:
                return QStringLiteral("Data");
            }
            break;
        }
        break;
    }
    return QVariant();
}

Qt::ItemFlags mbClientSendMessageFileRecordsModel::flags(const QModelIndex &index) const
{
    auto f = QAbstractTableModel::flags(index);
    if (m_editMode || (index.column() != Column_Data))
        f |= Qt::ItemIsEditable;
    return f;
}

QVariant mbClientSendMessageFileRecordsModel::data(const QModelIndex &index, int role) const
{
    switch (role)
    {
    case Qt::DisplayRole:
    case Qt::EditRole:
    {
        Item *item = m_items.value(index.row());
        if (item)
        {
            switch (index.column())
            {
            case Column_File:
                return toVariant(item->file.fileNumber);
            case Column_Record:
                return toVariant(item->file.recordNumber);
            case Column_Length:
                return item->file.recordLength;
            case Column_Data:
                return item->cache;
            }
        }
    }
    break;
    }
    return QVariant();
}

bool mbClientSendMessageFileRecordsModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    switch (role)
    {
    case Qt::EditRole:
    {
        Item *item = m_items.value(index.row());
        if (item)
        {
            switch (index.column())
            {
            case Column_File:
                item->file.fileNumber = toUInt16(value);
                break;
            case Column_Record:
                item->file.recordNumber = toUInt16(value);
                break;
            case Column_Length:
                item->file.recordLength = value.toUInt();
                break;
            case Column_Data:
            {
                mbClientMessageParams params;
                params.setFormat(m_format);
                params.setData(value);
                auto data = m_conv->toByteArray(params);
                setItemDataInner(item, data);
            }
                break;
            }
        }
    }
    break;
    }
    return false;
}

void mbClientSendMessageFileRecordsModel::fillParams(mbClientMessageParams &params, bool useData)
{
    auto fileRecords = params.fileRecords();
    fileRecords.resize(m_items.count());
    int i = 0;
    QByteArray data;
    Q_FOREACH(const auto &item, m_items)
    {
        fileRecords[i] = item->file;
        if (useData)
        {
            data.append(item->getData());
        }
        ++i;
    }
    params.setFileRecords(fileRecords);
    if (useData)
    {
        params.setFormat(m_format);
        params.setData(data);
        params.setCount(data.length());
    }
}

void mbClientSendMessageFileRecordsModel::setParams(mbClientMessageParams &params)
{
    beginResetModel();

    qDeleteAll(m_items);
    m_items.clear();
    for (int i = 0; i < params.fileRecords().count(); ++i)
    {
        Item *item = new Item;
        item->file = params.fileRecords().at(i);
        m_items.append(item);
    }

    int c = 0;
    auto b = m_conv->toByteArray(params);
    Q_FOREACH (Item *item, m_items)
    {
        auto len = item->file.recordLength*2;
        if ((c+len) <= b.length())
        {
            setItemDataInner(item, b.mid(c, len));
            c += len;
        }
    }
    endResetModel();
}

void mbClientSendMessageFileRecordsModel::setRecordData(const QList<QByteArray> &dataList)
{
    if (m_items.count() == dataList.count())
    {
        for (int i = 0; i < m_items.count(); ++i)
        {
            setItemDataInner(m_items.at(i), dataList.at(i));
        }
        Q_EMIT dataChanged(index(0, Column_Data), index(m_items.count()-1, Column_Data));
    }
}

void mbClientSendMessageFileRecordsModel::insertRecord(int i)
{
    if (i < 0 || i > m_items.count())
        i = m_items.count();
    Item *item = new Item;
    item->file.fileNumber = 0;
    item->file.recordNumber = 0;
    item->file.recordLength = 1;
    beginInsertRows(QModelIndex(), i, i);
    m_items.insert(i, item);
    endInsertRows();
}

void mbClientSendMessageFileRecordsModel::removeRecord(int i)
{
    Item *item = m_items.value(i);
    if (item)
    {
        beginRemoveRows(QModelIndex(), i, i);
        m_items.removeAt(i);
        delete item;
        endRemoveRows();
    }
}

bool mbClientSendMessageFileRecordsModel::moveUp(int i)
{
    if (i > 0)
        return moveTo(i, i-1);
    return false;
}

bool mbClientSendMessageFileRecordsModel::moveDown(int i)
{
    if (i < m_items.count() - 1)
        return moveTo(i, i+1);
    return false;
}

void mbClientSendMessageFileRecordsModel::clear()
{
    if (m_items.count())
    {
        beginResetModel();
        qDeleteAll(m_items);
        m_items.clear();
        endResetModel();
    }
}

void mbClientSendMessageFileRecordsModel::clearValues()
{
    if (m_items.count())
    {
        Q_FOREACH (auto *item, m_items)
        {
            setItemDataInner(item, QByteArray());
        }
        Q_EMIT dataChanged(index(0, Column_Data), index(m_items.count()-1, Column_Data));
    }
}

void mbClientSendMessageFileRecordsModel::setAddressFormat(mb::DigitalFormat format)
{
    if (m_addressFormat != format)
    {
        m_addressFormat = format;
        Q_EMIT dataChanged(index(0, Column_File), index(m_items.count()-1, Column_Record));
    }
}

void mbClientSendMessageFileRecordsModel::setDataFormat(mb::Format format)
{
    if (m_format != format)
    {
        m_format = format;
        for (Item *item : std::as_const(m_items))
            setItemDataInner(item, item->data);
        Q_EMIT dataChanged(index(0, Column_Data), index(m_items.count()-1, Column_Data));
    }
}

QVariant mbClientSendMessageFileRecordsModel::toVariant(uint16_t v) const
{
    switch (m_addressFormat)
    {
    case mb::Bin:
        return mb::toBinString(v);
    case mb::Oct:
        return mb::toOctString(v);
    case mb::Dec:
        return static_cast<qint16>(v);
    case mb::UDec:
        return static_cast<quint16>(v);
    case mb::Hex:
    default:
        return mb::toHexString(v);
    }
}

uint16_t mbClientSendMessageFileRecordsModel::toUInt16(const QVariant &v) const
{
    switch (m_addressFormat)
    {
    case mb::Bin:
        return v.toString().toUShort(nullptr, 2);
    case mb::Oct:
        return v.toString().toUShort(nullptr, 8);
    case mb::Dec:
        return static_cast<qint16>(v.toInt());
    case mb::UDec:
        return static_cast<quint16>(v.toUInt());
    case mb::Hex:
    default:
        return v.toString().toUShort(nullptr, 16);
    }
}

bool mbClientSendMessageFileRecordsModel::moveTo(int oldPos, int newPos)
{
    Item *item = m_items.value(oldPos);
    if (item)
    {
        beginRemoveRows(QModelIndex(), oldPos, oldPos);
        m_items.removeAt(oldPos);
        endRemoveRows();
        beginInsertRows(QModelIndex(), newPos, newPos);
        m_items.insert(newPos, item);
        endInsertRows();
        return true;
    }
    return false;
}

void mbClientSendMessageFileRecordsModel::setItemDataInner(Item *item, const QByteArray &data)
{
    mbClientMessageParams params;
    params.setFormat(m_format);
    params.setCount(data.count()*8);
    params.setData(data);
    item->data = data;
    item->cache = m_conv->toVariant(params).toString();
}

mbClientSendMessageFileRecordsWidget::Strings::Strings() :
    format     (QStringLiteral("format")),
    fileRecords(QStringLiteral("fileRecords")),
    data       (QStringLiteral("data"))
{
}

const mbClientSendMessageFileRecordsWidget::Strings &mbClientSendMessageFileRecordsWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageFileRecordsWidget::mbClientSendMessageFileRecordsWidget(uint8_t func, mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(func, ui, parent)
{
    m_fileRecordModel = new mbClientSendMessageFileRecordsModel(ui->converter(), this);

    // format
    m_cmbAddressFormat = new QComboBox(this);
    const auto dls = mb::enumDigitalFormatKeyList();
    for (int i = 1; i < dls.count(); ++i) // pass DefaultDigitalFormat
    {
        const auto format = mb::enumDigitalFormatValueByIndex(i);
        m_cmbAddressFormat->addItem(dls.at(i), static_cast<int>(format));
    }
    m_cmbAddressFormat->setCurrentText(mb::enumDigitalFormatKey(m_fileRecordModel->addressFormat()));
    connect(m_cmbAddressFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &mbClientSendMessageFileRecordsWidget::setAddressFormat);


    m_cmbFormat = new QComboBox(this);
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbFormat->addItem(s);
    }
    m_cmbFormat->setCurrentIndex(m_fileRecordModel->dataFormat());
    connect(m_cmbFormat, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index){
        auto format = mb::enumFormatValueByIndex(index);
        m_fileRecordModel->setDataFormat(format);
    });

    // file records
    m_tblFileRecords = new QTableView(this);
    m_tblFileRecords->setModel(m_fileRecordModel);
    m_tblFileRecords->horizontalHeader()->setStretchLastSection(true);
    
    // Labels
    auto lblFileRecords = new QLabel(this);
    lblFileRecords->setText(QCoreApplication::translate("mbClientSendMessageUi", "File Records:", nullptr));

    auto lblAddressFormat = new QLabel(this);
    lblAddressFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Address Format:", nullptr));

    auto lblFormat = new QLabel(this);
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Data Format:", nullptr));

    // Buttons
    auto btnFileRecordAdd = new QPushButton(this);
    QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(btnFileRecordAdd->sizePolicy().hasHeightForWidth());
    btnFileRecordAdd->setSizePolicy(sizePolicy1);
    QIcon icon;
    icon.addFile(QString::fromUtf8(":/core/icons/plus.ico"), QSize(), QIcon::Normal, QIcon::Off);
    btnFileRecordAdd->setIcon(icon);

    auto btnFileRecordDelete = new QPushButton(this);
    sizePolicy1.setHeightForWidth(btnFileRecordDelete->sizePolicy().hasHeightForWidth());
    btnFileRecordDelete->setSizePolicy(sizePolicy1);
    QIcon icon1;
    icon1.addFile(QString::fromUtf8(":/core/icons/minus.ico"), QSize(), QIcon::Normal, QIcon::Off);
    btnFileRecordDelete->setIcon(icon1);

    auto btnFileRecordMoveUp = new QPushButton(this);
    sizePolicy1.setHeightForWidth(btnFileRecordMoveUp->sizePolicy().hasHeightForWidth());
    btnFileRecordMoveUp->setSizePolicy(sizePolicy1);
    QIcon icon2;
    icon2.addFile(QString::fromUtf8(":/core/icons/arrow_up.png"), QSize(), QIcon::Normal, QIcon::Off);
    btnFileRecordMoveUp->setIcon(icon2);

    auto btnFileRecordMoveDown = new QPushButton(this);
    sizePolicy1.setHeightForWidth(btnFileRecordMoveDown->sizePolicy().hasHeightForWidth());
    btnFileRecordMoveDown->setSizePolicy(sizePolicy1);
    QIcon icon3;
    icon3.addFile(QString::fromUtf8(":/core/icons/arrow_down.png"), QSize(), QIcon::Normal, QIcon::Off);
    btnFileRecordMoveDown->setIcon(icon3);

    auto btnFileRecordClear = new QPushButton(this);
    sizePolicy1.setHeightForWidth(btnFileRecordClear->sizePolicy().hasHeightForWidth());
    btnFileRecordClear->setSizePolicy(sizePolicy1);
    QIcon icon4;
    icon4.addFile(QString::fromUtf8(":/core/icons/clear.png"), QSize(), QIcon::Normal, QIcon::Off);
    btnFileRecordClear->setIcon(icon4);

    connect(btnFileRecordAdd     , &QPushButton::clicked, this, &mbClientSendMessageFileRecordsWidget::slotFileRecordAdd     );
    connect(btnFileRecordDelete  , &QPushButton::clicked, this, &mbClientSendMessageFileRecordsWidget::slotFileRecordDelete  );
    connect(btnFileRecordMoveUp  , &QPushButton::clicked, this, &mbClientSendMessageFileRecordsWidget::slotFileRecordMoveUp  );
    connect(btnFileRecordMoveDown, &QPushButton::clicked, this, &mbClientSendMessageFileRecordsWidget::slotFileRecordMoveDown);
    connect(btnFileRecordClear   , &QPushButton::clicked, this, &mbClientSendMessageFileRecordsWidget::slotFileRecordClear   );

    // Spacer
    auto horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    auto verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    // Layouts
    auto horizontalLayout1 = new QHBoxLayout();
    horizontalLayout1->addWidget(lblFileRecords);
    horizontalLayout1->addItem(horizontalSpacer);
    horizontalLayout1->addWidget(lblAddressFormat);
    horizontalLayout1->addWidget(m_cmbAddressFormat);
    horizontalLayout1->addWidget(lblFormat);
    horizontalLayout1->addWidget(m_cmbFormat);
    horizontalLayout1->setStretch(1, 1);

    auto verticalLayout1 = new QVBoxLayout();
    verticalLayout1->setSpacing(2);
    verticalLayout1->addWidget(btnFileRecordAdd);
    verticalLayout1->addWidget(btnFileRecordDelete);
    verticalLayout1->addWidget(btnFileRecordMoveUp);
    verticalLayout1->addWidget(btnFileRecordMoveDown);
    verticalLayout1->addWidget(btnFileRecordClear);
    verticalLayout1->addItem(verticalSpacer);  

    auto horizontalLayout2 = new QHBoxLayout();
    horizontalLayout2->addWidget(m_tblFileRecords);
    horizontalLayout2->addLayout(verticalLayout1);

    auto verticalLayout2 = new QVBoxLayout(this);
    verticalLayout2->addLayout(horizontalLayout1);
    verticalLayout2->addLayout(horizontalLayout2);

    this->setLayout(verticalLayout2);
}

MBSETTINGS mbClientSendMessageFileRecordsWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    mbClientMessageParams params;
    m_fileRecordModel->fillParams(params, true);

    MBSETTINGS m;
    m[m_prefix+s.format     ] = m_cmbFormat->currentText();
    m[m_prefix+s.fileRecords] = params.fileRecordsAsByteArray();
    m[m_prefix+s.data       ] = params.data();
    return m;
}

void mbClientSendMessageFileRecordsWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    mbClientMessageParams params;
    it = m.find(m_prefix+s.format     ); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
    it = m.find(m_prefix+s.fileRecords); if (it != end) params.setFileRecords(it.value().toByteArray());
    it = m.find(m_prefix+s.data       ); if (it != end) params.setData(it.value().toByteArray());
    params.setFormat(m_fileRecordModel->dataFormat());

    m_fileRecordModel->setParams(params);
}

void mbClientSendMessageFileRecordsWidget::fillParams(mbClientMessageParams &params) const
{
    m_fileRecordModel->fillParams(params, m_fileRecordModel->editMode());
}

void mbClientSendMessageFileRecordsWidget::setParams(mbClientMessageParams &params)
{
    m_fileRecordModel->setParams(params);
}

mb::DigitalFormat mbClientSendMessageFileRecordsWidget::addressFormat() const
{
    return static_cast<mb::DigitalFormat>(m_cmbAddressFormat->currentData().toInt());
}

uint8_t mbClientSendMessageFileRecordsWidget::getRecordsCount() const
{
    return static_cast<uint8_t>(m_fileRecordModel->rowCount());
}

void mbClientSendMessageFileRecordsWidget::slotFileRecordAdd()
{
    int i = currentFileRecordIndex();
    m_fileRecordModel->insertRecord(i);
}

void mbClientSendMessageFileRecordsWidget::slotFileRecordDelete()
{
    int i = currentFileRecordIndex();
    m_fileRecordModel->removeRecord(i);
}

void mbClientSendMessageFileRecordsWidget::slotFileRecordMoveUp()
{
    int i = currentFileRecordIndex();
    m_fileRecordModel->moveUp(i);
}

void mbClientSendMessageFileRecordsWidget::slotFileRecordMoveDown()
{
    int i = currentFileRecordIndex();
    m_fileRecordModel->moveDown(i);
}
void mbClientSendMessageFileRecordsWidget::slotFileRecordClear()
{
    m_fileRecordModel->clear();
}

void mbClientSendMessageFileRecordsWidget::setAddressFormat(int)
{
    auto format = addressFormat();
    m_fileRecordModel->setAddressFormat(format);
}

int mbClientSendMessageFileRecordsWidget::currentFileRecordIndex() const
{
    auto indexes = m_tblFileRecords->selectionModel()->selectedIndexes();
    if (indexes.count())
        return indexes.first().row();
    return -1;
}

mbClientSendMessageReadFileRecordsWidget::mbClientSendMessageReadFileRecordsWidget(mbClientSendMessageUi *ui, QWidget *parent) :
            mbClientSendMessageFileRecordsWidget(MBF_READ_FILE_RECORD, ui, parent)
{
    m_fileRecordModel->setEditMode(false);
}

void mbClientSendMessageReadFileRecordsWidget::prepareToSend()
{
    m_fileRecordModel->clearValues();
}

mbClientSendMessageWriteFileRecordsWidget::mbClientSendMessageWriteFileRecordsWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageFileRecordsWidget(MBF_WRITE_FILE_RECORD, ui, parent)
{
    m_fileRecordModel->setEditMode(true);
}
