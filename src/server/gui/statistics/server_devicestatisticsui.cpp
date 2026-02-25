#include "server_devicestatisticsui.h"
#include "ui_server_devicestatisticsui.h"

#include <project/server_device.h>

mbServerDeviceStatisticsUi::mbServerDeviceStatisticsUi(mbServerDevice *device, QWidget *parent) :
    mbCoreDeviceStatisticsUi(device, parent),
    ui(new Ui::mbServerDeviceStatisticsUi)
{
    ui->setupUi(this);

    m_ui.lnSinceTimestamp       = ui->lnSinceTimestamp      ;
    m_ui.lnLastStatus           = ui->lnLastStatus          ;
    m_ui.lnLastTimestamp        = ui->lnLastTimestamp       ;
    m_ui.lnLastSuccessTimestamp = ui->lnLastSuccessTimestamp;
    m_ui.lnLastErrorStatus      = ui->lnLastErrorStatus     ;
    m_ui.lnLastErrorTimestamp   = ui->lnLastErrorTimestamp  ;
    m_ui.txtLastErrorText       = ui->txtLastErrorText      ;
    m_ui.lnCountTx              = ui->lnCountTx             ;
    m_ui.lnCountRx              = ui->lnCountRx             ;
    m_ui.lnCountGood            = ui->lnCountGood           ;
    m_ui.lnCountBad             = ui->lnCountBad            ;
    m_ui.lnCountBadStandard     = ui->lnCountBadStandard    ;

    connect(ui->btnReset, &QPushButton::clicked, this, &mbServerDeviceStatisticsUi::resetStatistics);
}

mbServerDeviceStatisticsUi::~mbServerDeviceStatisticsUi()
{
    delete ui;
}

void mbServerDeviceStatisticsUi::syncStatistics()
{
    mbCoreDeviceStatisticsUi::syncStatistics();
    //auto s = device()->statistics();
}
