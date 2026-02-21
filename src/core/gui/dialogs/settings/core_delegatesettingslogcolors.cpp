#include "core_delegatesettingslogcolors.h"

#include <QEvent>

#include <core.h>
#include <gui/core_ui.h>
#include <gui/dialogs/core_dialogs.h>

mbCoreDelegateSettingsLogColors::mbCoreDelegateSettingsLogColors(QObject *parent)
    : QStyledItemDelegate{parent}
{
}

bool mbCoreDelegateSettingsLogColors::editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index)
{
    switch (event->type())
    {
    case QEvent::MouseButtonDblClick:
    {
        mbCoreUi *ui = mbCore::globalCore()->coreUi();
        QColor color1 = model->data(index, Qt::BackgroundRole).value<QColor>();
        QColor color2 = ui->dialogsCore()->getColor(color1, ui, "Color");
        if (color2.isValid())
            model->setData(index, QVariant::fromValue(color2), Qt::BackgroundRole);
    }
        return true;
    default:
        return QStyledItemDelegate::editorEvent(event, model, option, index);
    }
}
