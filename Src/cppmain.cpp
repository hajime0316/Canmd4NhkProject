#include "cppmain.hpp"

#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// モジュールのインクルード
#include "stm32_printf/stm32_printf.h"
#include "canmd_manager/canmd_manager.h"
#include "stm32_easy_can/stm32_easy_can.h"

void setup(void) {
    // ソフトウェアモジュール初期化
    canmd_manager_init();
    // ハードウェアモジュールスタート
    stm32_printf_init(&huart1);
}

void loop(void) {

}
