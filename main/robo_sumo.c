#include "bluetooth.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"

void app_main(void) {
  //   ESP_LOGI(TAG, "Inicializando o driver do Servo...");
  //   init_servo();
  //
  //   int angle = 0;
  //   int step = 10;
  //   bool up = true;
  //
  //   while (1) {
  //     set_servo_angle(angle);
  //     vTaskDelay(50 / portTICK_PERIOD_MS);
  //
  //     if (up) {
  //       angle += step;
  //       if (angle >= 180) {
  //         angle = 180;
  //         up = false;
  //       }
  //     } else {
  //       angle -= step;
  //       if (angle <= 0) {
  //         angle = 0;
  //         up = true;
  //       }
  //     }
  //   }
  init_bluetooth();
}
