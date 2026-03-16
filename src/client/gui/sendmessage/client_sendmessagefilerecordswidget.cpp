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

struct mbClientSendMessageFileRecordsModel::Item
{
    Modbus::FileRecord file;
    QByteArray data;
    QString cache;

    QByteArray getData() const
    {
        auto expectedSize = file.recordLength*2;
        if (data.length() != expectedSize)
        {
            QByteArray b = data;
            b.resize(expectedSize);
            return b;
        }
        return data;
    }
};

mbClientSendMessageFileRecordsModel::mbClientSendMessageFileRecordsModel(QObject *parent)
    : QAbstractTableModel{parent}
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
                return QStringLiteral("File (hex)");
            case Column_Record:
                return QStringLiteral("Record (hex)");
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
                return mb::toHexString(item->file.fileNumber);
            case Column_Record:
                return mb::toHexString(item->file.recordNumber);
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
                item->file.fileNumber = value.toString().toUShort(nullptr, 16);
                break;
            case Column_Record:
                item->file.recordNumber = value.toString().toUShort(nullptr, 16);
                break;
            case Column_Length:
                item->file.recordLength = value.toUInt();
                break;
            case Column_Data:
                item->data = value.toByteArray();
                item->cache = mb::toVariant(item->data,
                                            m_format,
                                            Modbus::Memory_0x,
                                            mb::SwapNo,
                                            mb::R0R1R2R3,
                                            mb::DefaultDigitalFormat,
                                            "UTF-8",
                                            mb::DefaultStringLengthType,
                                            " ",
                                            item->data.count()).toString();

                break;
            }
        }
    }
    break;
    }
    return false;
}

void mbClientSendMessageFileRecordsModel::fillParams(mbClientMessageParamsOLD &params, bool useData)
{
    params.fileRecords.resize(m_items.count());
    int i = 0;
    QVariantList ls;
    Q_FOREACH(const auto &item, m_items)
    {
        params.fileRecords[i] = item->file;
        if (useData)
            ls.append(item->getData());
        ++i;
    }
    params.data = ls;
}

void mbClientSendMessageFileRecordsModel::setParams(const mbClientMessageParamsOLD &params)
{
    beginResetModel();

    qDeleteAll(m_items);
    m_items.clear();

    QVariantList ls = params.data.toList();
    for (int i = 0; i < params.fileRecords.count(); ++i)
    {
        Item *item = new Item;
        item->file = params.fileRecords.at(i);
        if (i < ls.count())
            setItemDataInner(item, ls.at(i).toByteArray());
        m_items.append(item);
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

void mbClientSendMessageFileRecordsModel::setFormat(mb::Format format)
{
    if (m_format != format)
    {
        m_format = format;
        for (Item *item : std::as_const(m_items))
            setItemDataInner(item, item->data);
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
    beginResetModel();
    qDeleteAll(m_items);
    m_items.clear();
    endResetModel();
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
    item->data = data;
    item->cache = mb::toVariant(item->data,
                                m_format,
                                Modbus::Memory_0x,
                                mb::SwapNo,
                                mb::R0R1R2R3,
                                mb::DefaultDigitalFormat,
                                "UTF-8",
                                mb::DefaultStringLengthType,
                                " ",
                                item->data.count()).toString();
}

mbClientSendMessageFileRecordsWidget::Strings::Strings() :
    prefix          (QStringLiteral("Ui.SendMessage.FileRecordsWidget.")),
    fileRecordFormat(prefix+QStringLiteral("format")),
    fileRecordData  (prefix+QStringLiteral("data"))
{

}

const mbClientSendMessageFileRecordsWidget::Strings &mbClientSendMessageFileRecordsWidget::Strings::instance()
{
    static Strings s;
    return s;
}

mbClientSendMessageFileRecordsWidget::mbClientSendMessageFileRecordsWidget(mbClientSendMessageUi *ui, QWidget *parent) :
    mbClientSendMessageWidget(ui, parent)
{
    this->setObjectName(QString::fromUtf8("pgFileRecords"));

    // format
    m_cmbFormat = new QComboBox(this);
    m_cmbFormat->setObjectName(QString::fromUtf8("cmbFileRecordFormat"));
    auto ls = mb::enumFormatKeyList();
    Q_FOREACH (const QString &s, ls)
    {
        m_cmbFormat->addItem(s);
    }
    m_cmbFormat->setCurrentIndex(mb::Dec16);

    // file records
    m_tblFileRecords = new QTableView(this);
    m_tblFileRecords->setObjectName(QString::fromUtf8("tblFileRecords"));
    m_fileRecordModel = new mbClientSendMessageFileRecordsModel(this);
    m_tblFileRecords->setModel(m_fileRecordModel);
    m_tblFileRecords->horizontalHeader()->setStretchLastSection(true);
    
    // Labels
    auto lblFileRecords = new QLabel(this);
    lblFileRecords->setObjectName(QString::fromUtf8("lblFileRecords"));
    lblFileRecords->setText(QCoreApplication::translate("mbClientSendMessageUi", "File Records:", nullptr));

    auto lblFormat = new QLabel(this);
    lblFormat->setObjectName(QString::fromUtf8("lblFormat"));
    lblFormat->setText(QCoreApplication::translate("mbClientSendMessageUi", "Format:", nullptr));

    // Buttons
    auto btnFileRecordAdd = new QPushButton(this);
    btnFileRecordAdd->setObjectName(QString::fromUtf8("btnFileRecordAdd"));
    QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);
    sizePolicy1.setHeightForWidth(btnFileRecordAdd->sizePolicy().hasHeightForWidth());
    btnFileRecordAdd->setSizePolicy(sizePolicy1);
    QIcon icon;
    icon.addFile(QString::fromUtf8(":/core/icons/plus.ico"), QSize(), QIcon::Normal, QIcon::Off);
    btnFileRecordAdd->setIcon(icon);

    auto btnFileRecordDelete = new QPushButton(this);
    btnFileRecordDelete->setObjectName(QString::fromUtf8("btnFileRecordDelete"));
    sizePolicy1.setHeightForWidth(btnFileRecordDelete->sizePolicy().hasHeightForWidth());
    btnFileRecordDelete->setSizePolicy(sizePolicy1);
    QIcon icon1;
    icon1.addFile(QString::fromUtf8(":/core/icons/minus.ico"), QSize(), QIcon::Normal, QIcon::Off);
    btnFileRecordDelete->setIcon(icon1);

    auto btnFileRecordMoveUp = new QPushButton(this);
    btnFileRecordMoveUp->setObjectName(QString::fromUtf8("btnFileRecordMoveUp"));
    sizePolicy1.setHeightForWidth(btnFileRecordMoveUp->sizePolicy().hasHeightForWidth());
    btnFileRecordMoveUp->setSizePolicy(sizePolicy1);
    QIcon icon2;
    icon2.addFile(QString::fromUtf8(":/core/icons/arrow_up.png"), QSize(), QIcon::Normal, QIcon::Off);
    btnFileRecordMoveUp->setIcon(icon2);

    auto btnFileRecordMoveDown = new QPushButton(this);
    btnFileRecordMoveDown->setObjectName(QString::fromUtf8("btnFileRecordMoveDown"));
    sizePolicy1.setHeightForWidth(btnFileRecordMoveDown->sizePolicy().hasHeightForWidth());
    btnFileRecordMoveDown->setSizePolicy(sizePolicy1);
    QIcon icon3;
    icon3.addFile(QString::fromUtf8(":/core/icons/arrow_down.png"), QSize(), QIcon::Normal, QIcon::Off);
    btnFileRecordMoveDown->setIcon(icon3);

    auto btnFileRecordClear = new QPushButton(this);
    btnFileRecordClear->setObjectName(QString::fromUtf8("btnFileRecordClear"));
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
    horizontalLayout1->setObjectName(QString::fromUtf8("horizontalLayout1"));
    horizontalLayout1->addWidget(lblFileRecords);
    horizontalLayout1->addItem(horizontalSpacer);
    horizontalLayout1->addWidget(lblFormat);
    horizontalLayout1->addWidget(m_cmbFormat);
    horizontalLayout1->setStretch(1, 1);

    auto verticalLayout1 = new QVBoxLayout();
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
    verticalLayout2->setObjectName(QString::fromUtf8("verticalLayout2"));
    verticalLayout2->addLayout(horizontalLayout1);
    verticalLayout2->addLayout(horizontalLayout2);

    this->setLayout(verticalLayout2);
}

MBSETTINGS mbClientSendMessageFileRecordsWidget::cachedSettings() const
{
    const Strings &s = Strings::instance();

    MBSETTINGS m;
    m[s.fileRecordFormat] = m_cmbFormat->currentText();
    m[s.fileRecordData  ] = getData();

    return m;
}

void mbClientSendMessageFileRecordsWidget::setCachedSettings(const MBSETTINGS &m)
{
    const Strings &s = Strings::instance();

    MBSETTINGS::const_iterator it;
    MBSETTINGS::const_iterator end = m.end();

    it = m.find(s.fileRecordFormat ); if (it != end) m_cmbFormat->setCurrentText(it.value().toString());
    it = m.find(s.fileRecordData   ); if (it != end) setData(it.value().toByteArray());
}

QByteArray mbClientSendMessageFileRecordsWidget::getData() const
{
    // TODO:
}

void mbClientSendMessageFileRecordsWidget::setData(const QByteArray &data)
{
    // TODO:
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

int mbClientSendMessageFileRecordsWidget::currentFileRecordIndex() const
{
    auto indexes = m_tblFileRecords->selectionModel()->selectedIndexes();
    if (indexes.count())
        return indexes.first().row();
    return -1;
}

