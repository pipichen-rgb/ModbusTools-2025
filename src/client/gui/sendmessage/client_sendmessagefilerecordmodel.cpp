#include "client_sendmessagefilerecordmodel.h"

struct mbClientSendMessageFileRecordModel::Item
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

mbClientSendMessageFileRecordModel::mbClientSendMessageFileRecordModel(QObject *parent)
    : QAbstractTableModel{parent}
{

}

mbClientSendMessageFileRecordModel::~mbClientSendMessageFileRecordModel()
{
    qDeleteAll(m_items);
}

int mbClientSendMessageFileRecordModel::columnCount(const QModelIndex &/*parent*/) const
{
    return ColumnCount;
}

int mbClientSendMessageFileRecordModel::rowCount(const QModelIndex &/*parent*/) const
{
    return m_items.count();
}

QVariant mbClientSendMessageFileRecordModel::headerData(int section, Qt::Orientation orientation, int role) const
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

QVariant mbClientSendMessageFileRecordModel::data(const QModelIndex &index, int role) const
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

bool mbClientSendMessageFileRecordModel::setData(const QModelIndex &index, const QVariant &value, int role)
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

void mbClientSendMessageFileRecordModel::fillParams(mbClientMessageParams &params, bool useData)
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

void mbClientSendMessageFileRecordModel::setParams(const mbClientMessageParams &params)
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

void mbClientSendMessageFileRecordModel::setRecordData(const QList<QByteArray> &dataList)
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

void mbClientSendMessageFileRecordModel::setFormat(mb::Format format)
{
    if (m_format != format)
    {
        m_format = format;
        for (Item *item : std::as_const(m_items))
            setItemDataInner(item, item->data);
        Q_EMIT dataChanged(index(0, Column_Data), index(m_items.count()-1, Column_Data));
    }
}

void mbClientSendMessageFileRecordModel::insertRecord(int i)
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

void mbClientSendMessageFileRecordModel::removeRecord(int i)
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

bool mbClientSendMessageFileRecordModel::moveUp(int i)
{
    if (i > 0)
        return moveTo(i, i-1);
    return false;
}

bool mbClientSendMessageFileRecordModel::moveDown(int i)
{
    if (i < m_items.count() - 1)
        return moveTo(i, i+1);
    return false;
}

void mbClientSendMessageFileRecordModel::clear()
{
    beginResetModel();
    qDeleteAll(m_items);
    m_items.clear();
    endResetModel();
}

bool mbClientSendMessageFileRecordModel::moveTo(int oldPos, int newPos)
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

void mbClientSendMessageFileRecordModel::setItemDataInner(Item *item, const QByteArray &data)
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
