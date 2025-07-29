/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "si5351.h"
#include "stm32f1xx_hal_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

uint8_t enc;
//uint32_t freq;
uint32_t freq[3];
char choice;
uint8_t number[3][6];
uint8_t choiced_num;
uint8_t choiced_channel;
char interface_mode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
err_t si5351_set_frequency(uint8_t output, uint32_t frequency) {
    // Проверка допустимости выхода и частоты
    if (output > 2) return ERROR_INVALIDPARAMETER;
    if (frequency < 8000 || frequency > 150000000) return ERROR_INVALIDPARAMETER;

    // Определение R-делителя для частот < 500 кГц
    si5351RDiv_t r_div = SI5351_R_DIV_1;
    uint32_t r_div_value = 1;
    if (frequency < 500000) {
        uint32_t min_freq = frequency;
        while (min_freq < 500000 && r_div_value < 128) {
            r_div_value *= 2;
            min_freq = frequency * r_div_value;
        }
        switch (r_div_value) {
            case 2:   r_div = SI5351_R_DIV_2;   break;
            case 4:   r_div = SI5351_R_DIV_4;   break;
            case 8:   r_div = SI5351_R_DIV_8;   break;
            case 16:  r_div = SI5351_R_DIV_16;  break;
            case 32:  r_div = SI5351_R_DIV_32;  break;
            case 64:  r_div = SI5351_R_DIV_64;  break;
            case 128: r_div = SI5351_R_DIV_128; break;
            default:  r_div = SI5351_R_DIV_1;   break;
        }
    }

    // Расчет частоты для мультисинта (до R-делителя)
    double f_ms = (double)frequency * r_div_value;

    // Подбор делителя мультисинта (8-900)
    uint32_t div = (uint32_t)(800000000.0 / f_ms); // Целевой делитель для ~800 МГц
    if (div < 8) div = 8;
    if (div > 900) div = 900;

    // Расчет частоты PLL
    double f_pll = f_ms * div;
    if (f_pll < 600000000 || f_pll > 900000000) {
        // Корректировка при выходе за пределы 600-900 МГц
        div = (f_pll < 600000000) ? (uint32_t)ceil(600000000.0 / f_ms) : 900;
        f_pll = f_ms * div;
    }

    // Настройка PLL
    double f_xtal = (double)m_si5351Config.crystalFreq;
    uint32_t mult = (uint32_t)(f_pll / f_xtal);
    double fraction = (f_pll / f_xtal) - mult;
    uint32_t num = (uint32_t)round(fraction * 1048575.0); // 20-битный числитель
    uint32_t denom = 1048575; // 20-битный знаменатель

    // Выбор PLL: выход 2 → PLL_B, остальные → PLL_A
    si5351PLL_t pll = (output == 2) ? SI5351_PLL_B : SI5351_PLL_A;

    // Применение настроек
    ASSERT_STATUS(si5351_setupPLL(pll, mult, num, denom));
    ASSERT_STATUS(si5351_setupMultisynth(output, pll, div, 0, 1)); // Целочисленный режим
    ASSERT_STATUS(si5351_setupRdiv(output, r_div));

    return ERROR_NONE;
}



void uint32_to_str(uint32_t num, char *str) {
    char tmp[12]; // Временный буфер
    int i = 0;

    // Обрабатываем 0 отдельно
    if (num == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }

    // Записываем цифры в обратном порядке
    while (num > 0) {
        tmp[i++] = '0' + (num % 10);
        num /= 10;
    }

    // Разворачиваем строку
    int j = 0;
    while (i-- > 0) {
        str[j++] = tmp[i];
    }
    str[j] = '\0';
}
uint32_t array_to_uint32_t(uint8_t arr[]){
	uint32_t ans=0;
	for(int i=0;i<6;++i){
		ans+=arr[i]*(pow(10,i));
	}
	return ans;
}

uint32_t get_encoder(){
	return TIM1->CNT/4;//для энкодера использующегося в проекте
}

void set_encoder(uint32_t e){
	TIM1->CNT=e*4;
}

void print_interface_mode0(){
	ssd1306_SetCursor(1, 1);
	ssd1306_Fill(Black);
	char buff[12];
	uint32_to_str(freq[0], buff);
	ssd1306_WriteString(buff, Font_7x10, White);
	ssd1306_WriteString("   ", Font_7x10, White);
	uint32_to_str(freq[1], buff);
	ssd1306_WriteString(buff, Font_7x10, White);
	ssd1306_SetCursor(1, 10);//для переноса на следующую строку
	uint32_to_str(freq[2], buff);
	ssd1306_WriteString(buff, Font_7x10, White);
	ssd1306_WriteString("   ", Font_7x10, White);
	if(choiced_channel==0){
		ssd1306_WriteString("ch0", Font_7x10, White);
	}else if(choiced_channel==1){
		ssd1306_WriteString("ch1", Font_7x10, White);
	}else{
		ssd1306_WriteString("ch2", Font_7x10, White);
	}
	ssd1306_UpdateScreen();
}


void print_interface_mode1(){


	ssd1306_SetCursor(1, 1);
	ssd1306_Fill(Black);
	//char buff[12];
	if(choiced_num==0){
		for(int i=5;i>=0;--i){
			char t=48+number[choiced_channel][i];
			char str[2] = {t, '\0'};
			ssd1306_WriteString(str, Font_7x10, White);
		}
		freq[choiced_channel]=array_to_uint32_t(number[choiced_channel]);
		ssd1306_WriteString("   ", Font_7x10, White);
		ssd1306_WriteString("send", Font_11x18, White);
	}else{
		for(int i=5;i>=0;--i){
			if(i==choiced_num-1){
				char t=48+number[choiced_channel][i];
				char str[2] = {t, '\0'};
				ssd1306_WriteString(str, Font_11x18, White);
			}else{
				char t=48+number[choiced_channel][i];
				char str[2] = {t, '\0'};
				ssd1306_WriteString(str, Font_7x10, White);
			}
		}
		ssd1306_WriteString("   ", Font_7x10, White);
		ssd1306_WriteString("send", Font_7x10, White);
	}
	ssd1306_UpdateScreen();
}

void int_mode_0(){
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {  // Если кнопка нажата (подтяжка к VCC)
		choice=1;

	}
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);  // Ждём отпускания
	HAL_Delay(10);
	if(choice==0 && enc!=get_encoder()){
		enc=get_encoder();
		if(enc>2){
			enc=0;
			set_encoder(0);
		}else if(enc<0){
			enc=2;
			set_encoder(2);
		}
		choiced_channel=enc;
		print_interface_mode0();
	}else if(choice==1){
		enc=1;
		set_encoder(1);
		choiced_num=1;//потому что есть send который будем считать за 0 положение
		interface_mode=1;
		choice=0;
		print_interface_mode1();
	}

}

void increase_left(){
	for(int i=choiced_num;i<7;++i){
		if(number[choiced_channel][i-1]==9){
			number[choiced_channel][i-1]=0;
		}else{
			number[choiced_channel][i-1]++;
			break;
		}
	}
}

void decrease_left() {
    int i = choiced_num;
    while (i < 7) {
        if (number[choiced_channel][i-1] == 0) {
            number[choiced_channel][i-1] = 9;
            i++; // Переходим к следующей цифре слева
        } else {
            number[choiced_channel][i-1] -= 1;
            break;
        }
    }
}

void min_freq(){
	number[choiced_channel][0]=8;
	number[choiced_channel][1]=0;
	number[choiced_channel][2]=0;
	number[choiced_channel][3]=0;
	number[choiced_channel][4]=0;
	number[choiced_channel][5]=0;

}

void max_freq(){
	number[choiced_channel][0]=0;
	number[choiced_channel][1]=0;
	number[choiced_channel][2]=0;
	number[choiced_channel][3]=0;
	number[choiced_channel][4]=6;
	number[choiced_channel][5]=1;

}

void int_mode_1(){
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {  // Если кнопка нажата (подтяжка к VCC)
		if(choice==0){

			choice=1;
			if(choiced_num!=0){
				enc=number[choiced_channel][choiced_num-1];
				set_encoder(enc);
			}
		}else{
			choice=0;
			enc=choiced_num;
			set_encoder(enc);
		}

	}
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);  // Ждём отпускания
	HAL_Delay(10);
	if(choice==0 && enc!=get_encoder()){
		enc=get_encoder();
		if(enc>5000){
			set_encoder(6);
			enc=6;
		}
		else if(enc>6){
			enc=0;
			set_encoder(0);
		}
		choiced_num=enc;
		print_interface_mode1();
	}else if(choice==1){
		if(choiced_num==0){
			choice=0;
			interface_mode=0;
			freq[choiced_channel]=array_to_uint32_t(number[choiced_channel]);
			if(freq[choiced_channel]<8){
				freq[choiced_channel]=8;
				min_freq();
			}else if(freq[choiced_channel]>160000){
				freq[choiced_channel]=160000;
				max_freq();
			}
			si5351_set_frequency(choiced_channel, freq[choiced_channel]*1000);
			si5351_enableOutputs(0xFF);
			enc=choiced_channel;
			set_encoder(enc);
			print_interface_mode0();
		}else if(enc!=get_encoder()){
			if(get_encoder()>5000){
				enc=9;
				set_encoder(9);
				number[choiced_channel][choiced_num-1]=0;
				decrease_left();
			}
			else if(get_encoder()>9){
				enc=0;
				set_encoder(0);
				number[choiced_channel][choiced_num-1]=9;
				increase_left();
			}else{
				enc=get_encoder();
				number[choiced_channel][choiced_num-1]=enc;
			}
			print_interface_mode1();

		}
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */






/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  ssd1306_Init();
  si5351_Init();
  enc=0;//значение энкодера с учётом дребзга
  set_encoder(0);//выставление энкодера в 0
  freq[0]=8;//начальная минимальная частота канала 0
  freq[1]=8;//начальная минимальная частота канала 1
  freq[2]=8;//начальная минимальная частота канала 2
  choice=0;//переменная для считывания был ли нажат энкодер
  number[0][0]=8;//массив значения частоты в кГц канала 0
  number[1][0]=8;//массив значения частоты в кГц канала 1
  number[2][0]=8;//массив значения частоты в кГц канала 2
  choiced_num=0;//переменная для определения выбранной цифры в массиве частоты
  choiced_channel=0;// номер выбранного канала
  interface_mode=0;//переменная для определения что должно показыватиься на экране(0-значения частот, 1-редактирование частоты)
  si5351_set_frequency(0, 8000);//устанвливаем частоту в минимальную
  si5351_set_frequency(1, 8000);//устанвливаем частоту в минимальную
  si5351_set_frequency(2, 8000);//устанвливаем частоту в минимальную
  si5351_enableOutputs(0xFF);//включаем все выходы
  print_interface_mode0();//выводим на экран начальный интерфейс
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(interface_mode==0){
		  int_mode_0();
	  }else{
		  int_mode_1();
	  }





















	  /*if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {  // Если кнопка нажата (подтяжка к VCC)
		  //choice=1;
		  if(choice==0){
			  choice=1;
			  enc=number[choiced];
			  TIM1->CNT=enc/4;
		  }else{
			  choice=0;
			  si5351_set_frequency(2, freq);
			  si5351_enableOutputs(2);
			  enc=choiced;
			  TIM1->CNT=enc/4;
		  }
	      while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);  // Ждём отпускания
	  }
	  HAL_Delay(10);
	  if(choice==1){
		  if(enc!=TIM1->CNT/4){
			  enc=TIM1->CNT/4;
			  if(enc>9){
				  enc=0;
			  }else if(enc<0){
				  enc=9;
			  }
			  number[choiced]=enc;
			  freq=array_to_uint32_t(number);
			  ssd1306_SetCursor(1, 1);
			  char dt[12];
			  uint32_to_str(freq, dt);
			  ssd1306_Fill(Black);
			  ssd1306_WriteString(dt, Font_7x10, White);
			  ssd1306_UpdateScreen();
		  }
	  }else{
		  if(enc!=TIM1->CNT/4){
			  enc=TIM1->CNT/4;
			  if(enc>5){
				  enc=0;
			  }else if(enc<0){
				  enc=5;
			  }
			  choiced=enc;
		  }
	  }*/

  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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

#ifdef  USE_FULL_ASSERT
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
