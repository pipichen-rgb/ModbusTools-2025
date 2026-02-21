#include "core_modelsettingslogcolors.h"

#include <gui/logview/core_logview.h>

mbCoreModelSettingsLogColors::mbCoreModelSettingsLogColors(QObject *parent)
    : QAbstractTableModel{parent},
    m_colorFormats({
      ColorFormat(mb::Log_Error),
      ColorFormat(mb::Log_Warning),
      ColorFormat(mb::Log_Info),
      ColorFormat(mb::Log_Tx),
      ColorFormat(mb::Log_Rx),
      ColorFormat(mb::Log_Debug)
    })
{
    setDefaultColors();
}

QVariant mbCoreModelSettingsLogColors::headerData(int section, Qt::Orientation orientation, int role) const
{
    switch (role)
    {
    case Qt::DisplayRole:
        switch (orientation)
        {
        case Qt::Horizontal:
            switch (section)
            {
            case 0: return QStringLiteral("Color");
            }
            break;
        case Qt::Vertical:
            if (section < m_colorFormats.count())
                return m_colorFormats.at(section).name;
            break;
        }
        break;
    }

    return QVariant();
}

int mbCoreModelSettingsLogColors::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return m_colorFormats.count();
}

int mbCoreModelSettingsLogColors::columnCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return 1;
}

QVariant mbCoreModelSettingsLogColors::data(const QModelIndex &index, int role) const
{
    switch (role)
    {
    case Qt::BackgroundRole:
    {
        int r = index.row();
        if (r < m_colorFormats.count())
            return m_colorFormats.at(r).color;
    }
        break;
    }

    return QVariant();
}

bool mbCoreModelSettingsLogColors::setData(const QModelIndex &index, const QVariant &value, int role)
{
    switch (role)
    {
    case Qt::BackgroundRole:
    {
        int r = index.row();
        if (r < m_colorFormats.count())
        {
            QColor color = value.value<QColor>();
            m_colorFormats[r].color = color;
            Q_EMIT dataChanged(index, index, {role});
            return true;
        }
    }
        break;
    }

    return false;
}

mb::IntColorMap mbCoreModelSettingsLogColors::colorMap() const
{
    mb::IntColorMap result;
    Q_FOREACH (const ColorFormat &fmt, m_colorFormats)
        result[fmt.type] = fmt.color;
    return result;
}

void mbCoreModelSettingsLogColors::setColorMap(const mb::IntColorMap &map)
{
    for (ColorFormat &localfmt : m_colorFormats)
    {
        auto it = map.constFind(localfmt.type);
        if (it != map.constEnd())
            localfmt.color = it.value();
    }

    if (!m_colorFormats.isEmpty())
        Q_EMIT dataChanged(createIndex(0, 0), createIndex(m_colorFormats.count() - 1, 0));
}

void mbCoreModelSettingsLogColors::setDefaultColors()
{
    setColorMap(mbCoreLogView::Defaults::instance().colors);
}
