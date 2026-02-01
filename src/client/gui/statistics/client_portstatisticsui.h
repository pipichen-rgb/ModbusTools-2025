#ifndef CLIENT_PORTSTATISTICSUI_H
#define CLIENT_PORTSTATISTICSUI_H

#include <gui/statistics/core_portstatisticsui.h>

class mbClientPort;

namespace Ui {
class mbClientPortStatisticsUi;
}

class mbClientPortStatisticsUi : public mbCorePortStatisticsUi
{
    Q_OBJECT

public:
    explicit mbClientPortStatisticsUi(mbClientPort *port, QWidget *parent = nullptr);
    ~mbClientPortStatisticsUi();

private:
    Ui::mbClientPortStatisticsUi *ui;
};

#endif // CLIENT_PORTSTATISTICSUI_H
