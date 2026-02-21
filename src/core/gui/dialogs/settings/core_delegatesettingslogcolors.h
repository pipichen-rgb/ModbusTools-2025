#ifndef CORE_DELEGATESETTINGSLOGCOLORS_H
#define CORE_DELEGATESETTINGSLOGCOLORS_H

#include <QStyledItemDelegate>

class mbCoreDelegateSettingsLogColors : public QStyledItemDelegate
{
public:
    explicit mbCoreDelegateSettingsLogColors(QObject *parent = nullptr);

public:
    bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index) override;
};

#endif // CORE_DELEGATESETTINGSLOGCOLORS_H
