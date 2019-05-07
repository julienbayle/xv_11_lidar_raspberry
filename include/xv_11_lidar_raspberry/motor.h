#include <pthread.h>

namespace xv_11_lidar_raspberry {
    int pwm_ ;
    int pwmRange_;
    pthread_t thread_ ;

    void startPwm (int value);
    void setPwm (int value);
    void stopPwm ();
    
}