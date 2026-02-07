#include "core_statisticsui.h"

mbCoreStatisticsUi::mbCoreStatisticsUi(QWidget *parent) :
    QWidget(parent)
{
}

void mbCoreStatisticsUi::setStatWindowTitle(const QString &name)
{
    setWindowTitle(QString("%1 [Stat]").arg(name));
}
