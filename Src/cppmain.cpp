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
#include "stm32_antiphase_pwm/stm32_antiphase_pwm.hpp"
#include "stm32_velocity/stm32_velocity.hpp"
#include "pid/pid.hpp"
#include "stm32_led/stm32_led.hpp"
#include "stm32_access_flash_byte/stm32_access_flash_byte.hpp"
#include "stm32_long_push_switch/stm32_long_push_switch.hpp"

#define CONTROL_LOOP_TIME 0.01 // sec

void sw_enc_0_event_callback();
void sw_enc_1_event_callback();

static int md_id = 0;
static Stm32Velocity* velocity[2];
static Stm32Led led_gp(LED_GP_GPIO_Port, LED_GP_Pin, GPIO_PIN_RESET);
static Stm32Led led_enc[2] = {
    {LED_ENC2_GPIO_Port, LED_ENC2_Pin, GPIO_PIN_RESET},
    {LED_ENC1_GPIO_Port, LED_ENC1_Pin, GPIO_PIN_RESET}
};
static Stm32AccessFlashByte* flash_memory_enc[2];
static GPIO_TypeDef* sw_enc_gpio_port[2] = {SW_ENC2_GPIO_Port, SW_ENC1_GPIO_Port};
static uint16_t sw_enc_pin[2] = {SW_ENC2_Pin, SW_ENC1_Pin};
static Stm32LongPushSwitch* sw_enc[2];

void setup(void) {
    // flash_memory 初期化
    for(int i = 0; i < 2; i++) {
        flash_memory_enc[i] = new Stm32AccessFlashByte(i);
    }

    // velocityモジュール初期化
    velocity[0] = new Stm32Velocity(&htim4, flash_memory_enc[0]->get());
    velocity[1] = new Stm32Velocity(&htim19, flash_memory_enc[0]->get());    

    // LEDをすべて点灯させる
    led_gp.setOn();
    for (int i = 0; i < 2; i++) {
        led_enc[i].setOn();
    }
    
    // md_id初期化
    md_id = HAL_GPIO_ReadPin(DIP_SW_4_GPIO_Port, DIP_SW_4_Pin) << 3
          | HAL_GPIO_ReadPin(DIP_SW_3_GPIO_Port, DIP_SW_3_Pin) << 2
          | HAL_GPIO_ReadPin(DIP_SW_2_GPIO_Port, DIP_SW_2_Pin) << 1
          | HAL_GPIO_ReadPin(DIP_SW_1_GPIO_Port, DIP_SW_1_Pin) << 0;

    if(md_id == 0) {
        md_id = 0X7FF;

        // TODO: ここにLEDによるエラー表示の処理を入れる
    }

    // ソフトウェアモジュール初期化
    canmd_manager_init();
    // ハードウェアモジュールスタート
    stm32_printf_init(&huart1);
    stm32_easy_can_init(&hcan, md_id, 0X7FF);
    for (int i = 0; i < 2; i++) {
        sw_enc[i] = new Stm32LongPushSwitch(sw_enc_gpio_port[i], sw_enc_pin[i], GPIO_PIN_RESET, 10);
    }
    sw_enc[0]->set_event_callback(sw_enc_0_event_callback);
    sw_enc[1]->set_event_callback(sw_enc_1_event_callback);

    // 100msecタイマスタート
    HAL_TIM_Base_Start_IT(&htim7);
    // 50msecタイマスタート
    HAL_TIM_Base_Start_IT(&htim13);

    // LED_ENCを点滅させる
    for (int i = 0; i < 2; i++) {
        led_enc[i].setFlash(2);
    }

    // Debug Output
    stm32_printf("\r\n...\r\n");
    // TODO: PWMの周波数の表示
    stm32_printf("md id = %d\r\n", md_id);
    stm32_printf("Setup routine start.\r\n");

    // セットアップルーチン
    while(!canmd_manager_is_motor_setup_data_received());
    stm32_printf("Setup routine was finished!\r\n");

    // 初期化内容によってLED_ENCのステータスを変更
    MotorSetupData motor_setup_data[2];
    canmd_manager_get_motor_setup_data(motor_setup_data);
    for (int i = 0; i < 2; i++) {
        switch (motor_setup_data[i].control_mode)
        {
            case DUTY_RATE_MODE:
                led_enc[i].setOff();
                break;

            case PID_MODE:
                led_enc[i].setOn();
                break;

            default:
                break;
        }
    }

    // 10msecタイマスタート
    HAL_TIM_Base_Start_IT(&htim6);

}

void loop(void) {
    int motor_control_data[2];
    MotorSetupData motor_setup_data[2];
    // モーターコントロールデータ取得
    canmd_manager_get_motor_control_data(motor_control_data);
    // PID制御のゲイン取得
    canmd_manager_get_motor_setup_data(motor_setup_data);

    // デバッグ出力
    stm32_printf("%3d  ", md_id);
    for(int i = 0; i < 2; i++) {
        stm32_printf("|  ");
        stm32_printf("%5d  ", motor_control_data[i]);
        stm32_printf("%4d  ", velocity[i]->get_velocity());
        // コントロールモードの表示
        switch (motor_setup_data[i].control_mode)
        {
            case DUTY_RATE_MODE:
                stm32_printf("DUTY_RATE  ");
                break;
            
            case PID_MODE:
                stm32_printf("PID  ");
                break;

            default:
                stm32_printf("XXXX  ");
                break;
        }
        stm32_printf("%4d  ", motor_setup_data[i].kp);
        stm32_printf("%4d  ", motor_setup_data[i].ki);
        stm32_printf("%4d  ", motor_setup_data[i].kd);
    }
    stm32_printf("\r\n");
}

//**************************
//    タイマ割り込み関数
//**************************
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// 10msecタイマ
	if(htim->Instance == TIM6) {
		int motor_control_data[2] = {};
        double duty_rate[2];

		// モータコントロールデータ取得
		canmd_manager_get_motor_control_data(motor_control_data);

        // モーターセットアップデータの取得
        MotorSetupData motor_setup_data[2];
        canmd_manager_get_motor_setup_data(motor_setup_data);

        // 速度計算
        for (int i = 0; i < 2; i++){
            velocity[i]->periodic_calculate_velocity();
        }

        // PIDモジュールを作成
        static Pid pid_module[2] = {
            {
                (double)motor_setup_data[0].kp,
                (double)motor_setup_data[0].ki,
                (double)motor_setup_data[0].kd,
                CONTROL_LOOP_TIME
            },
            {
                (double)motor_setup_data[1].kp,
                (double)motor_setup_data[1].ki,
                (double)motor_setup_data[1].kd,
                CONTROL_LOOP_TIME
            }
        };

        for (int i = 0; i < 2; i++)
        {
            //// モーターコントロールモードによってduty比の決め方を分ける
            switch (motor_setup_data[i].control_mode)
            {
            case DUTY_RATE_MODE:
                duty_rate[i] = motor_control_data[i] / (double)MOTOR_CONTROL_DATA_MAX;
                break;
            
            case PID_MODE:
                // PID制御の計算
                int pid_output;

                // モーターコントロールデータが0の時は
                // 強制的に出力を0にしてブレーキをかける
                if(motor_control_data[i] == 0) {
                    pid_module[i].reset_internal_state();
                    pid_output = 0;
                }
                else {
                    pid_output = pid_module[i].pid_calc(
                        velocity[i]->get_velocity(),
                        motor_control_data[i]);
                }

                // duty比計算
                duty_rate[i] = pid_output / (double)PID_OUTPUT_MAX;
                
                break;

            default:
                duty_rate[i] = 0;
                break;
            }
            
        }

		// PWMのデューティー比更新
        static Stm32AntiphasePwm pwm0(&htim3, TIM_CHANNEL_4, TIM_CHANNEL_3);
        static Stm32AntiphasePwm pwm1(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2);

        pwm0.update_duty(duty_rate[0]);
        pwm1.update_duty(duty_rate[1]);
	}
	// 100msecタイマ
	if(htim->Instance == TIM7) {
		if(canmd_manager_time_out_check()) {
            led_gp.setOn();
		}
		else {
            led_gp.setFlash(4);
		}
        Stm32LongPushSwitch::interrupt_handler();
	}
    // 50msecタイマ
    if(htim->Instance == TIM13) {
        Stm32Led::interrupt_handler();
    }
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
        static Stm32Velocity divided_encoder_count[2] = {&htim4, &htim19};
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

	return;
}

//********************************************
//    プッシュスイッチ0のイベントコールバック
//********************************************
void sw_enc_0_event_callback() {
    // エンコーダの回転方向を反転
    velocity[0]->reverse_rotation();
    // flash memoryの値を反転
    unsigned char temp = flash_memory_enc[0]->get();
    flash_memory_enc[0]->save(temp);
    // LEDを点灯させる
    led_enc[0].setOn();
}

//********************************************
//    プッシュスイッチ1のイベントコールバック
//********************************************
void sw_enc_1_event_callback() {
    // エンコーダの回転方向を反転
    velocity[1]->reverse_rotation();
    // flash memoryの値を反転
    unsigned char temp = flash_memory_enc[1]->get();
    flash_memory_enc[1]->save(temp);
    // LEDを点灯させる
    led_enc[1].setOn();
}
