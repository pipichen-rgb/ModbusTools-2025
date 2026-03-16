#ifndef CLIENT_SENDMESSAGEFILERECORDSWIDGET_H
#define CLIENT_SENDMESSAGEFILERECORDSWIDGET_H

#include <QAbstractTableModel>

#include <client_global.h>
#include "client_sendmessagewidget.h"

class QComboBox;
class QTableView;

class mbClientSendMessageFileRecordsModel : public QAbstractTableModel
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
    mbClientSendMessageFileRecordsModel(QObject *parent = nullptr);
    ~mbClientSendMessageFileRecordsModel();

public: // QAbstractItemModel interface
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    QVariant data(const QModelIndex &index, int role) const override;
    bool setData(const QModelIndex &index, const QVariant &value, int role) override;

public:
    void fillParams(mbClientMessageParamsOLD &params, bool useData = true);
    void setParams(const mbClientMessageParamsOLD &params);
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

class mbClientSendMessageFileRecordsWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString prefix          ;
        const QString fileRecordFormat;
        const QString fileRecordData  ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageFileRecordsWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    QByteArray getData() const override;
    void setData(const QByteArray &data) override;

public:
    uint8_t getRecordsCount() const;

private Q_SLOTS: // file records
    void slotFileRecordAdd     ();
    void slotFileRecordDelete  ();
    void slotFileRecordMoveUp  ();
    void slotFileRecordMoveDown();
    void slotFileRecordClear   ();

private:
    int currentFileRecordIndex() const;

private:
    QComboBox* m_cmbFormat;
    QTableView* m_tblFileRecords;
    mbClientSendMessageFileRecordsModel* m_fileRecordModel;
    bool m_isDirty;
};

#endif // CLIENT_SENDMESSAGEFILERECORDSWIDGET_H
