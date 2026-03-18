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
    mbClientSendMessageFileRecordsModel(mbClientMessageConverter* conv, QObject *parent = nullptr);
    ~mbClientSendMessageFileRecordsModel();

public: // QAbstractItemModel interface
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    Qt::ItemFlags flags(const QModelIndex &index) const override;
    QVariant data(const QModelIndex &index, int role) const override;
    bool setData(const QModelIndex &index, const QVariant &value, int role) override;

public:
    void fillParams(mbClientMessageParams &fillParams, bool useData);
    void setParams(mbClientMessageParams &params);
    void setRecordData(const QList<QByteArray>& dataList);

public:
    inline bool editMode() const { return m_editMode; }
    inline void setEditMode(bool editMode) { m_editMode = editMode; }
    inline mb::Format format() const { return m_format; }
    void insertRecord(int i);
    void removeRecord(int i);
    bool moveUp(int i);
    bool moveDown(int i);
    void clear();
    void setFormat(mb::Format format);

private:
    struct Item;
    bool moveTo(int oldPos, int newPos);
    void setItemDataInner(Item *item, const QByteArray &data);

private:
    bool m_editMode;
    mbClientMessageConverter* m_conv;
    QList<Item*> m_items;
    mb::Format m_format;
};

class mbClientSendMessageFileRecordsWidget : public mbClientSendMessageWidget
{
    Q_OBJECT

public:
    struct Strings
    {
        const QString format     ;
        const QString fileRecords;
        const QString data       ;
        Strings();
        static const Strings &instance();
    };

public:
    mbClientSendMessageFileRecordsWidget(uint8_t func, mbClientSendMessageUi* ui, QWidget *parent = nullptr);

public:
    MBSETTINGS cachedSettings() const override;
    void setCachedSettings(const MBSETTINGS &settings) override;
    void fillParams(mbClientMessageParams &params);
    void setParams(mbClientMessageParams &params);

public:
    uint8_t getRecordsCount() const;

protected Q_SLOTS: // file records
    void slotFileRecordAdd     ();
    void slotFileRecordDelete  ();
    void slotFileRecordMoveUp  ();
    void slotFileRecordMoveDown();
    void slotFileRecordClear   ();

protected:
    int currentFileRecordIndex() const;
    QByteArray saveFileRecordData(const QVector<Modbus::FileRecord> &fileRecords) const;
    QVector<Modbus::FileRecord> restoreFileRecordData(const QByteArray &data) const;

protected:
    QComboBox* m_cmbFormat;
    QTableView* m_tblFileRecords;
    mbClientSendMessageFileRecordsModel* m_fileRecordModel;
    bool m_isDirty;
};

class mbClientSendMessageReadFileRecordsWidget : public mbClientSendMessageFileRecordsWidget
{
    Q_OBJECT
public:
    explicit mbClientSendMessageReadFileRecordsWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);
};

class mbClientSendMessageWriteFileRecordsWidget : public mbClientSendMessageFileRecordsWidget
{
    Q_OBJECT
public:
    explicit mbClientSendMessageWriteFileRecordsWidget(mbClientSendMessageUi* ui, QWidget *parent = nullptr);
};

#endif // CLIENT_SENDMESSAGEFILERECORDSWIDGET_H
