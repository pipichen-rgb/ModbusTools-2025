#include "core_portstatisticsui.h"

mbCorePortStatisticsUi::mbCorePortStatisticsUi(mbCorePort *port, QWidget *parent) :
    mbCoreStatisticsUi(parent),
    m_port(port)
{
}

mbCorePortStatisticsUi::~mbCorePortStatisticsUi()
{
}
