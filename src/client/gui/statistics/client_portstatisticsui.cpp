#include "client_portstatisticsui.h"
#include "ui_client_portstatisticsui.h"

#include <project/client_port.h>

mbClientPortStatisticsUi::mbClientPortStatisticsUi(mbClientPort *port, QWidget *parent) :
    mbCorePortStatisticsUi(port, parent),
    ui(new Ui::mbClientPortStatisticsUi)
{
    ui->setupUi(this);
}

mbClientPortStatisticsUi::~mbClientPortStatisticsUi()
{
    delete ui;
}
