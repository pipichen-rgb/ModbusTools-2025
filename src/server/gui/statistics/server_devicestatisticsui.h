#ifndef SERVER_DEVICESTATISTICSUI_H
#define SERVER_DEVICESTATISTICSUI_H

#include <gui/statistics/core_devicestatisticsui.h>

class mbServerDevice;

namespace Ui {
class mbServerDeviceStatisticsUi;
}

class mbServerDeviceStatisticsUi : public mbCoreDeviceStatisticsUi
{
    Q_OBJECT

public:
    explicit mbServerDeviceStatisticsUi(mbServerDevice *device, QWidget *parent = nullptr);
    ~mbServerDeviceStatisticsUi();

public:
    inline mbServerDevice *device() const { return reinterpret_cast<mbServerDevice*>(m_device); }

public:
    void syncStatistics() override;

private:
    Ui::mbServerDeviceStatisticsUi *ui;
};

#endif // SERVER_DEVICESTATISTICSUI_H
