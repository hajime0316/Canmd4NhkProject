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
    stm32_easy_can_init(&hcan, 1, 0X7FF);   // TODO: 1をmd_idに変える

    // セットアップルーチン
    while(!canmd_manager_is_motor_setup_data_received());
    stm32_printf("Setup routine was finished!\r\n");
}

void loop(void) {
    int motor_control_data[2];
    MotorSetupData motor_setup_data[2];
    // モーターコントロールデータ取得
    canmd_manager_get_motor_control_data(motor_control_data);
    // PID制御のゲイン取得
    canmd_manager_get_motor_setup_data(motor_setup_data);

    // デバッグ出力
    stm32_printf("%5d  %5d  ", motor_control_data[0], motor_control_data[1]);
    // stm32_printf("%3d  ", md_id);
    for(int i = 0; i < 2; i++) {
        stm32_printf("|  ");
        stm32_printf("%2d  ",motor_setup_data[i].control_mode);
        stm32_printf("%4d  ", motor_setup_data[i].kp);
        stm32_printf("%4d  ", motor_setup_data[i].ki);
        stm32_printf("%4d  ", motor_setup_data[i].kd);
        // stm32_printf("%4d", g_velocity[i]);
    }
    stm32_printf("\r\n");
}

//**************************
//    CAN通信受信割り込み
//**************************
void stm32_easy_can_interrupt_handler(void)
{
	int receive_id;
	int receive_dlc;
	unsigned char receive_message[8];

	// 受信データ取得
	stm32_easy_can_get_receive_message(&receive_id, &receive_dlc, receive_message);

	// 受信データ処理
    MdDataType receive_md_data_type
        = canmd_manager_set_can_receive_data(receive_message, receive_dlc);
/*
    // 送信データ生成
    int transmit_id;
    int transmit_dlc;
    unsigned char transmit_message[8];

    // CAN通信の送信ID生成
    transmit_id = md_id   << 5 | 0b00000 ;
    //            送信元ID(5bit)  送信先ID(5bit)

    if(receive_md_data_type != MD_DATA_TYPE_MOTOR_CONTROL_DATA) {
        // 受信メッセージをそのまま送信メッセージとする
        transmit_dlc = receive_dlc;
        for(int i = 0; i < receive_dlc; i++) {
            transmit_message[i] = receive_message[i];
        }
    }
    else {
        // エンコーダのカウント値を送信メッセージとする
        static Stm32Velocity divided_encoder_count[2] = {&htim2, &htim3};
        for(int i = 0; i < 2; i++) {
            divided_encoder_count[i].periodic_calculate_velocity();
        }
        transmit_dlc = 3;
        transmit_message[0] = (MD_DATA_TYPE_MOTOR_CONTROL_DATA             << 6           )
                            | (divided_encoder_count[0].get_velocity() >> 5 & 0b111000)
                            | (divided_encoder_count[1].get_velocity() >> 8 & 0b111   );
        transmit_message[1] = divided_encoder_count[0].get_velocity() & 0XFF;
        transmit_message[2] = divided_encoder_count[1].get_velocity() & 0XFF;
    }

    // データ送信
    stm32_easy_can_transmit_message(transmit_id, transmit_dlc, transmit_message);
*/
	return;
}
