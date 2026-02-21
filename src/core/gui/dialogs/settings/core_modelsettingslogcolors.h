#ifndef CORE_MODELSETTINGSLOGCOLORS_H
#define CORE_MODELSETTINGSLOGCOLORS_H

#include <QAbstractTableModel>
#include <QColor>

#include <mbcore.h>

class mbCoreModelSettingsLogColors : public QAbstractTableModel
{
    Q_OBJECT

public:
    struct ColorFormat
    {
        ColorFormat(mb::LogFlag ftype) :
            type(ftype),
            name(mb::toString(ftype))
        {}

        mb::LogFlag type;
        QString name;
        QColor color;
    };

    typedef QList<ColorFormat> ColorFormats;

public:
    explicit mbCoreModelSettingsLogColors(QObject *parent = nullptr);

public:
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;

public:
    mb::IntColorMap colorMap() const;
    void setColorMap(const mb::IntColorMap &map);

public Q_SLOTS:
    void setDefaultColors();

private:
    ColorFormats m_colorFormats;
};

#endif // CORE_MODELSETTINGSLOGCOLORS_H
