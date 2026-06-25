#include "lidar.hpp"

int main()
{
    // Lidar lidar("192.168.1.2", 6201, "192.168.1.62", 6101);
    Lidar lidar("127.0.0.1", 6201, "127.0.0.1", 6101);

    lidar.sync_time(0, 0);
    lidar.set_work_mode(true, false, false, false, false);

    while (true)
    {
        lidar.get_imu();
        lidar.get_cloud();
    }

    return 0;
}