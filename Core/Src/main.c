#include "main.h"
#include "tim.h"
#include "gpio.h"

//varijable part uno
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint8_t Is_First_Captured = 0;
uint32_t Distance = 0;
uint8_t Object_Detected = 0;
uint8_t Servo_Angle_At_Detection = 0;
uint8_t current_servo_angle = 0;
uint16_t current_pulse_value = 0;

// Trigger pin
#define TRIGGER_GPIO_Port GPIOA
#define TRIGGER_Pin GPIO_PIN_1

// Laser pin
#define LASER_GPIO_Port GPIOA
#define LASER_Pin GPIO_PIN_4

// Stepper motor pinovi
#define IN1_Pin GPIO_PIN_3
#define IN1_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_4
#define IN2_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_5
#define IN3_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_6
#define IN4_GPIO_Port GPIOB

// Tipkala
#define BUTTON1_GPIO_Port GPIOB
#define BUTTON1_Pin GPIO_PIN_0
#define BUTTON2_GPIO_Port GPIOB
#define BUTTON2_Pin GPIO_PIN_1
#define BUTTON3_GPIO_Port GPIOA
#define BUTTON3_Pin GPIO_PIN_2

// LED diode za svaki mod
#define LED_MOD1_GPIO_Port GPIOA
#define LED_MOD1_Pin GPIO_PIN_6
#define LED_MOD2_GPIO_Port GPIOA
#define LED_MOD2_Pin GPIO_PIN_8
#define LED_MOD3_GPIO_Port GPIOA
#define LED_MOD3_Pin GPIO_PIN_9

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;  // Timer za debounce

// varijable stepper
int32_t stepper_position = 0;
int32_t stepper_target = 0;
uint8_t stepper_sequence[8][4] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};
uint8_t stepper_index = 0;

// Podešeni omjer koraka na koracnom s obzirom na kut
#define STEPS_PER_DEGREE 10
#define MAX_STEPPER_STEPS 1800

// Granice za detekciju(mozda nije potrebno radi s tim, ponekad ne bez toga)
#define MIN_DETECTION_DISTANCE 10
#define MAX_DETECTION_DISTANCE 40
#define NOISE_FILTER_DISTANCE 5

// Modovi rada(varijable dos)
uint8_t operation_mode = 1;
volatile uint8_t button1_pressed_flag = 0;
volatile uint8_t button2_pressed_flag = 0;
volatile uint8_t button3_pressed_flag = 0;
uint32_t mode2_timer = 0;
uint8_t mode2_active = 0;
uint8_t mode2_servo_stopped = 0;

#define MODE2_TIMEOUT 15000

// Varijable za zadnje poznate pozicije i vrijeme
uint8_t last_detected_angle = 0;
uint32_t last_detection_time = 0;
#define ANGLE_MEMORY_TIME 2000

// varijable za ledblink(lijepse izgleda)
uint8_t led_blink_state = 0;
uint32_t last_blink_time = 0;
#define BLINK_INTERVAL 500 // 500ms za treptanje

// VArijable za debouncing s TIM16
volatile uint32_t button1_debounce_time = 0;
volatile uint32_t button2_debounce_time = 0;
volatile uint32_t button3_debounce_time = 0;
#define DEBOUNCE_DELAY 50 // 50ms
#define TIM16_PERIOD_MS 1 // Perioda TIM16 timer-a u ms

// Varijable za pracenje najblizeg objekta u modu 3
uint32_t closest_distance = 0;
uint8_t closest_angle = 0;
uint8_t tracking_closest = 0;

// Varijable za  laser aktivan i kad krene
uint8_t laser_active = 0;
uint32_t laser_start_time = 0;
#define LASER_DURATION 5000 // 5 sekundi

// varijable za mod 3 nadopuna
uint8_t mode3_button_pressed = 0;
uint8_t mode3_servo_moving = 0;
uint8_t mode3_scan_complete = 0;
uint32_t last_mode3_check_time = 0;

// Funkcija za upravljanje laserom
void Laser_Control(uint8_t state) {
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    laser_active = state;
    if(state) {
        laser_start_time = HAL_GetTick();
    }
}

// Laser gasi kad gotov
void Check_Laser_Timeout(void) {
    if(laser_active && (HAL_GetTick() - laser_start_time >= LASER_DURATION)) {
        Laser_Control(0);
    }
}

// Funkcija za postavljanje kuta serva
void Servo_Set_Angle(uint8_t angle) {
    if(angle > 180) angle = 180;
    current_servo_angle = angle;
    current_pulse_value = 500 + ((angle * 2000) / 180);
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, current_pulse_value);
}

// Funkcija za LED diode
void Set_LED_Mod1(uint8_t state) {
    HAL_GPIO_WritePin(LED_MOD1_GPIO_Port, LED_MOD1_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Set_LED_Mod2(uint8_t state) {
    HAL_GPIO_WritePin(LED_MOD2_GPIO_Port, LED_MOD2_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Set_LED_Mod3(uint8_t state) {
    HAL_GPIO_WritePin(LED_MOD3_GPIO_Port, LED_MOD3_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Led Blink Update
void Update_Mode_LEDs(void) {
    uint32_t current_time = HAL_GetTick();

    if(current_time - last_blink_time >= BLINK_INTERVAL) {
        led_blink_state = !led_blink_state;
        last_blink_time = current_time;
    }

    // GAASI LED
    Set_LED_Mod1(0);
    Set_LED_Mod2(0);
    Set_LED_Mod3(0);

    // UPali u trazenom modu
    switch(operation_mode) {
        case 1:
            Set_LED_Mod1(led_blink_state); // MOD 1: Zuta
            break;
        case 2:
            Set_LED_Mod2(led_blink_state); // MOD 2: Crvena
            break;
        case 3:
            Set_LED_Mod3(led_blink_state); // MOD 3: Zelena
            break;
    }
}

// Ultrazvucni trigger i mjerenje filtar
void HCSR04_Start_With_Filter(void) {
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
    HAL_Delay(5);
    HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, current_pulse_value);
}

// Stepper/KOracni motor
void Stepper_Update(void) {
    static uint32_t last_step_time = 0;
    uint32_t current_time = HAL_GetTick();

    if(current_time - last_step_time < 3) return;

    int32_t position_diff = stepper_target - stepper_position;

    if(position_diff > 0) {
        stepper_index = (stepper_index + 1) % 8;
        stepper_position++;
        last_step_time = current_time;
    }
    else if(position_diff < 0) {
        stepper_index = (stepper_index == 0) ? 7 : stepper_index - 1;
        stepper_position--;
        last_step_time = current_time;
    }

    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, stepper_sequence[stepper_index][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, stepper_sequence[stepper_index][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, stepper_sequence[stepper_index][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, stepper_sequence[stepper_index][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Preciznijie okretanje steppera na trazenu tocku
void Stepper_Turn_To_Exact_Position(void) {
    // Racunanje pozicije na temelju zadnje ocitanog kuta
    uint8_t corrected_angle = 180 - last_detected_angle;
    int32_t new_target = corrected_angle * STEPS_PER_DEGREE;

    // Ograniči min max
    if(new_target > MAX_STEPPER_STEPS) new_target = MAX_STEPPER_STEPS;
    if(new_target < 0) new_target = 0;

    // Postavi novu pozicikju SAMO ako se razlikuje od trenutne
    if(new_target != stepper_target) {
        stepper_target = new_target;
        // Upali laser kada se stepper počne okretati prema objektu/predmetu
        Laser_Control(1);
    }
}

// Funkcija za praćenje najbližeg objekta (mod 3)
void Track_Closest_Object(void) {
    if(Object_Detected) {
        if(Distance < closest_distance || closest_distance == 0) {
            closest_distance = Distance;
            closest_angle = current_servo_angle;
            tracking_closest = 1;
        }
    }
}

// Funckija za ocitavanje echa iz ultrazvucnog i racunanje duljine
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM2) {
        if(Is_First_Captured == 0) {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Is_First_Captured = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        } else {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Is_First_Captured = 0;

            uint32_t Difference;
            if(IC_Val2 > IC_Val1) Difference = IC_Val2 - IC_Val1;
            else Difference = (0xFFFF - IC_Val1) + IC_Val2;

            Distance = (Difference * 34) / 2000;

            if(Distance >= MIN_DETECTION_DISTANCE && Distance <= MAX_DETECTION_DISTANCE) {
                Object_Detected = 1;
                Servo_Angle_At_Detection = current_servo_angle;

                // MOD 1: Spremi točnu poziciju i okreni stepper
                if(operation_mode == 1) {
                    last_detected_angle = Servo_Angle_At_Detection;
                    last_detection_time = HAL_GetTick();
                    Stepper_Turn_To_Exact_Position();
                }
                // MOD 2: Spremi poziciju, zaustavi servo i okreni stepper
                else if(operation_mode == 2 && !mode2_active) {
                    last_detected_angle = Servo_Angle_At_Detection;
                    last_detection_time = HAL_GetTick();
                    mode2_active = 1;
                    mode2_servo_stopped = 1;
                    mode2_timer = HAL_GetTick();
                    Stepper_Turn_To_Exact_Position();
                }
                // MOD 3: Prati najbliži objekt
                else if(operation_mode == 3) {
                    Track_Closest_Object();
                }
            }
            else {
                Object_Detected = 0;
            }

            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

// TIM16 callback za debounce
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM16) {
        static uint8_t button1_prev_state = 0;
        static uint8_t button2_prev_state = 0;
        static uint8_t button3_prev_state = 0;

        // Pročitaj trenutna stanja tipki
        uint8_t button1_state = HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin);
        uint8_t button2_state = HAL_GPIO_ReadPin(BUTTON2_GPIO_Port, BUTTON2_Pin);
        uint8_t button3_state = HAL_GPIO_ReadPin(BUTTON3_GPIO_Port, BUTTON3_Pin);

        // Button 1 debounce
        if (button1_state != button1_prev_state) {
            button1_debounce_time++;
            if (button1_debounce_time >= (DEBOUNCE_DELAY / TIM16_PERIOD_MS)) {
                if (button1_state == GPIO_PIN_SET) {
                    button1_pressed_flag = 1;
                }
                button1_prev_state = button1_state;
                button1_debounce_time = 0;
            }
        } else {
            button1_debounce_time = 0;
        }

        // Button 2 debounce
        if (button2_state != button2_prev_state) {
            button2_debounce_time++;
            if (button2_debounce_time >= (DEBOUNCE_DELAY / TIM16_PERIOD_MS)) {
                if (button2_state == GPIO_PIN_SET) {
                    button2_pressed_flag = 1;
                }
                button2_prev_state = button2_state;
                button2_debounce_time = 0;
            }
        } else {
            button2_debounce_time = 0;
        }

        // Button 3 debounce
        if (button3_state != button3_prev_state) {
            button3_debounce_time++;
            if (button3_debounce_time >= (DEBOUNCE_DELAY / TIM16_PERIOD_MS)) {
                if (button3_state == GPIO_PIN_SET) {
                    button3_pressed_flag = 1;
                }
                button3_prev_state = button3_state;
                button3_debounce_time = 0;
            }
        } else {
            button3_debounce_time = 0;
        }
    }
}

// Inicijalizacija TIM16 za debounce
void Debounce_Timer_Init(void) {
    HAL_TIM_Base_Start_IT(&htim16);
}

// funkcija za kad su pritisnute tipke
void Handle_Button_Presses(void) {
    if(button1_pressed_flag) {
        operation_mode = 1;
        button1_pressed_flag = 0;
        mode2_active = 0;
        mode2_servo_stopped = 0;
        Object_Detected = 0;
        tracking_closest = 0;
        closest_distance = 0;
        mode3_servo_moving = 0;
        mode3_scan_complete = 0;
        Laser_Control(0);
        Update_Mode_LEDs();
    }
    else if(button2_pressed_flag) {
        operation_mode = 2;
        button2_pressed_flag = 0;
        mode2_active = 0;
        mode2_servo_stopped = 0;
        Object_Detected = 0;
        tracking_closest = 0;
        closest_distance = 0;
        mode3_servo_moving = 0;
        mode3_scan_complete = 0;
        Laser_Control(0);
        Update_Mode_LEDs();
    }
    else if(button3_pressed_flag) {
        operation_mode = 3;
        button3_pressed_flag = 0;
        mode2_active = 0;
        mode2_servo_stopped = 0;
        Object_Detected = 0;
        tracking_closest = 0;
        closest_distance = 0;
        mode3_servo_moving = 0;
        mode3_scan_complete = 0;
        Laser_Control(0);
        Update_Mode_LEDs();
    }
}

int main(void) {

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM14_Init();
    MX_TIM16_Init();  // Inicijaliziraj TIM16

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

    // Inicijaliziraj TIM16 za debounce
    Debounce_Timer_Init();

    // Postavljanje polariteta input capture za HC-SR04
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

    // L3ED DIODE ISKLJUCENE
    Set_LED_Mod1(0);
    Set_LED_Mod2(0);
    Set_LED_Mod3(0);

    // servo na položaj (180°)
    Servo_Set_Angle(180);
    HAL_Delay(1000);

    //  stepper motor - postavi na trenutnu poziciju
    stepper_target = stepper_position;
    while(stepper_position != stepper_target) {
        Stepper_Update();
        HAL_Delay(1);
    }
    HAL_Delay(1000);

       Update_Mode_LEDs();

    uint8_t servo_angle = 180;           // Početni kut serva
    uint8_t servo_direction = 0;         // Smjer kretanja serva (0 = smanjuje se, neradi obrnuto)
    uint8_t object_detected_previous = 0;// Prethodno stanje
    uint32_t last_measurement_time = 0;  // Vrijeme zadnjeg mjerenja
    uint32_t last_servo_move_time = 0;   // Vrijeme zadnjeg pomicanja serva


    while (1) {
        Handle_Button_Presses();

        Check_Laser_Timeout();

        // MOD 2: Provjera timeouta za pauzu nakon detekcije
        if(operation_mode == 2 && mode2_active) {
            if(HAL_GetTick() - mode2_timer >= MODE2_TIMEOUT) {
                mode2_active = 0;
                mode2_servo_stopped = 0;
                Object_Detected = 0;
            }
        }

        // MOD 3: Kontinuirano praćenje stanja tipka za ručno skeniranje
        if(operation_mode == 3) {
            uint32_t current_time = HAL_GetTick();

            // Provjera stanja tipke svakih 50ms (debounce ako radi )
            if(current_time - last_mode3_check_time >= 50) {
                mode3_button_pressed = (HAL_GPIO_ReadPin(BUTTON3_GPIO_Port, BUTTON3_Pin) == GPIO_PIN_SET);
                last_mode3_check_time = current_time;
            }

            // Ako je tipka pritisnuta, pomiči servo
            if(mode3_button_pressed) {
                mode3_servo_moving = 1;
                mode3_scan_complete = 0;

                // Pomakni servo svakih 20ms(jedan stupanj?)
                if(HAL_GetTick() - last_servo_move_time >= 20) {
                    if(servo_direction == 0) {
                        servo_angle--;
                        if(servo_angle <= 0) {
                            servo_angle = 0;
                            servo_direction = 1;  // Promijeni smjer
                        }
                    } else {
                        servo_angle++;
                        if(servo_angle >= 180) {
                            servo_angle = 180;
                            servo_direction = 0;
                        }
                    }
                    Servo_Set_Angle(servo_angle);
                    last_servo_move_time = HAL_GetTick();
                }
            }
            // failsafe kada je tipka pustena nakon skeniranja, okreni stepper prema objektu pri ponovom pritisku
            else if(mode3_servo_moving && !mode3_scan_complete) {
                mode3_scan_complete = 1;
                mode3_servo_moving = 0;

                if(tracking_closest) {
                    last_detected_angle = closest_angle;
                    Stepper_Turn_To_Exact_Position();
                }
            }
        }

        // mjerenje ultrazvučnim senzorom svakih 100ms
        if(HAL_GetTick() - last_measurement_time >= 100) {
            HCSR04_Start_With_Filter();
            last_measurement_time = HAL_GetTick();
        }

        // Detekcija promjene stanja objekta (pojava/nestanak)
        if(Object_Detected && !object_detected_previous) {
            object_detected_previous = Object_Detected;
        }
        else if(!Object_Detected && object_detected_previous) {
            object_detected_previous = Object_Detected;
        }

        Update_Mode_LEDs();

        // MOD 1: KOntinuirano skeniranje - servo se kontinuirano okreće
        if(operation_mode == 1) {
            if(HAL_GetTick() - last_servo_move_time >= 20) {
                if(servo_direction == 0) {
                    servo_angle--;
                    if(servo_angle <= 0) {
                        servo_angle = 0;
                        servo_direction = 1;
                        HAL_Delay(500);
                    }
                } else {
                    servo_angle++;
                    if(servo_angle >= 180) {
                        servo_angle = 180;
                        servo_direction = 0;
                        HAL_Delay(500);
                    }
                }
                Servo_Set_Angle(servo_angle);
                last_servo_move_time = HAL_GetTick();
            }
        }
        // MOD 2: Automatsko skeniranje s pauzom na detekciji
        else if(operation_mode == 2) {
            if(!mode2_servo_stopped && HAL_GetTick() - last_servo_move_time >= 20) {
                if(servo_direction == 0) {
                    servo_angle--;
                    if(servo_angle <= 0) {
                        servo_angle = 0;
                        servo_direction = 1;
                        HAL_Delay(500);
                    }
                } else {
                    servo_angle++;
                    if(servo_angle >= 180) {
                        servo_angle = 180;
                        servo_direction = 0;
                        HAL_Delay(500);
                    }
                }
                Servo_Set_Angle(servo_angle);
                last_servo_move_time = HAL_GetTick();
            }
        }
        // MOD 3: "Ručno" skeniranje - servo se kreće dok nije stisnuta, koracni samo dok je tipka stisnuta
        else if(operation_mode == 3 && mode3_button_pressed) {
            if(HAL_GetTick() - last_servo_move_time >= 20) {
                if(servo_direction == 0) {
                    servo_angle--;
                    if(servo_angle <= 0) {
                        servo_angle = 0;
                        servo_direction = 1;
                    }
                } else {
                    servo_angle++;
                    if(servo_angle >= 180) {
                        servo_angle = 180;
                        servo_direction = 0;
                    }
                }
                Servo_Set_Angle(servo_angle);
                last_servo_move_time = HAL_GetTick();
            }
        }

        Stepper_Update();

        if(stepper_position == stepper_target && laser_active) {
            // Dodaj laser funckiju
        }

        if(HAL_GetTick() - last_detection_time > ANGLE_MEMORY_TIME) {
            last_detected_angle = 0;
        }

        HAL_Delay(1);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
