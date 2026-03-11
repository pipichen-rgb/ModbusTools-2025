#ifndef CLIENT_SENDMESSAGEFILERECORDMODEL_H
#define CLIENT_SENDMESSAGEFILERECORDMODEL_H

#include <QAbstractTableModel>

#include <client_global.h>

class mbClientSendMessageFileRecordModel : public QAbstractTableModel
{
public:
    enum Columns
    {
        Column_File,
        Column_Record,
        Column_Length,
        Column_Data,
        ColumnCount
    };

public:
    mbClientSendMessageFileRecordModel(QObject *parent = nullptr);
    ~mbClientSendMessageFileRecordModel();

public: // QAbstractItemModel interface
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    QVariant data(const QModelIndex &index, int role) const override;
    bool setData(const QModelIndex &index, const QVariant &value, int role) override;

public:
    void fillParams(mbClientMessageParams &params, bool useData = true);
    void setParams(const mbClientMessageParams &params);
    void setRecordData(const QList<QByteArray>& dataList);

public:
    inline mb::Format format() const { return m_format; }
    void setFormat(mb::Format format);
    void insertRecord(int i);
    void removeRecord(int i);
    bool moveUp(int i);
    bool moveDown(int i);
    void clear();

private:
    struct Item;
    bool moveTo(int oldPos, int newPos);
    void setItemDataInner(Item *item, const QByteArray &data);

private:
    QList<Item*> m_items;
    mb::Format m_format;
};

#endif // CLIENT_SENDMESSAGEFILERECORDMODEL_H
