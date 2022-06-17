/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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


////////////////////////////////////USADAS PARA CALLBACK ANTERIOR////////////////
/*volatile int subida = 1; // Persiana subida
volatile int bajada = 0; // Persiana bajada
volatile int subir = 0; // Persiana subiendo
volatile int bajar = 0; // Persiana bajando
volatile int modo = 0; // Modo manual(0) automático(1)
volatile int cmodo = 0; // Entero que nos indicará si el usuario ha pulsado 1 o más veces el botón*/
////////////////////////////////////////////////////////////////////////////////

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define pasosporrev 4096 // para el motor

volatile int button_1 = 0; // bandera para interrupcion
volatile int button_2 = 0;

uint32_t ADC_val[3];
uint32_t distancia = 0;

char aux; //char borrar;
char key [4] = {0,0,0,0};

uint8_t estado = 0;
uint8_t preestado = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_1) {
		button_1 = 1;
	}

	if(GPIO_Pin ==  GPIO_PIN_2) {
		button_2 = 1;
	}
}


int debouncer(volatile int* button_int, GPIO_TypeDef* GPIO_port, uint16_t GPIO_number){
	static uint8_t button_count=0;
	static int counter=0;

	if (*button_int==1){
		if (button_count==0) {
			counter=HAL_GetTick();
			button_count++;
		}
		if (HAL_GetTick()-counter>=20){
			counter=HAL_GetTick();
			if (HAL_GPIO_ReadPin(GPIO_port, GPIO_number)!=1){
				button_count=1;
			}
			else{
				button_count++;
			}
			if (button_count==4){ //Periodo antirebotes
				button_count=0;
				*button_int=0;
				return 1;
			}
		}
	}
	return 0;
}

int joystick_arriba () {
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 100) ==  HAL_OK) {
		ADC_val[0] = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
	if(ADC_val[0] < 350) {return 1;}
	else {return 0;}
}

int joystick_abajo () {
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 100) ==  HAL_OK) {
		ADC_val[0] = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
	if(ADC_val[0] > 675) {return 1;}
	else {return 0;}
}

void delay_us (uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while(__HAL_TIM_GET_COUNTER(&htim3)<us);
}

void step_motor_set_rpm(int rpm) // max 13, min 1
{
	delay_us(60000000/pasosporrev/rpm);
}

void step_motor_half (int step) {
	switch(step) {
	case 0:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
		break;
	case 7:
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
		break;
	}
}

void step_motor_angle (float angle, int direction, int rpm) {
	float angperseq = 0.703125;
	int numberofseq = (int) (angle/angperseq);

	for (int seq=0; seq < numberofseq; seq++) {
		if (direction == 0) { // sentido horario
			for (int step = 7; step>=0; step--) {
				step_motor_half(step);
				step_motor_set_rpm(rpm);
			}
		}
		else if (direction == 1) { // sentido antihorario
			for (int step = 0; step<8; step++) {
				step_motor_half(step);
				step_motor_set_rpm(rpm);
			}
		}
	}
}

int hall () {
	HAL_ADC_Start(&hadc2);
	if(HAL_ADC_PollForConversion(&hadc2, 100) ==  HAL_OK) {
		ADC_val[1] = HAL_ADC_GetValue(&hadc2);
	}
	HAL_ADC_Stop(&hadc2);
	if(ADC_val[1] < 510) {return 1;}
	else {return 0;}
}

uint32_t ultrasonicRead (void) {
	int cuenta = 0;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	while(!(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4)));
	while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4)) {
		cuenta = cuenta + 1;
		delay_us(1);
	}
	return cuenta * 0.051;
}

uint32_t ldr () {
	HAL_ADC_Start(&hadc3);
	if(HAL_ADC_PollForConversion(&hadc3, 100) ==  HAL_OK) {
		ADC_val[2] = HAL_ADC_GetValue(&hadc3);
	}
	HAL_ADC_Stop(&hadc3);
	return ADC_val[2];
}

void motion () {
	 if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3)) {  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1); }
	 else {  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 0); }
}

char keypad (void)
{
	/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_5, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);  // Pull the R2 High
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);  // Pull the R3 High
	//HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High

	if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_9)))   // if the Col 1 is low
	{
		while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_9)));   // wait till the button is pressed
		return '1';
	}

	if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10)))   // if the Col 2 is low
	{
		while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10)));   // wait till the button is pressed
		return '2';
	}

	if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15)))   // if the Col 3 is low
	{
		while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15)));   // wait till the button is pressed
		return '3';
	}

	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_6, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);  // Pull the R3 High
	//HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High

	if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_9)))   // if the Col 1 is low
		{
			while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_9)));   // wait till the button is pressed
			return '4';
		}

		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10)))   // if the Col 2 is low
		{
			while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10)));   // wait till the button is pressed
			return '5';
		}

		if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15)))   // if the Col 3 is low
		{
			while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15)));   // wait till the button is pressed
			return '6';
		}

	/*if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));   // wait till the button is pressed
		return 'B';
	}*/


	/* Make ROW 3 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);  // Pull the R2 High
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_7, GPIO_PIN_SET);  // Pull the R3 High
	//HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High

	if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_9)))   // if the Col 1 is low
			{
				while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_9)));   // wait till the button is pressed
				return '7';
			}

			if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10)))   // if the Col 2 is low
			{
				while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_10)));   // wait till the button is pressed
				return '8';
			}

			if ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15)))   // if the Col 3 is low
			{
				while ((HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15)));   // wait till the button is pressed
				return '9';
			}

	/*if (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)))   // if the Col 4 is low
	{
		while (!(HAL_GPIO_ReadPin (C4_PORT, C4_PIN)));   // wait till the button is pressed
		return 'C';
	}*/
	return 'x';
}


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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start(&htim3);
HAL_TIM_Base_Start(&htim8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(debouncer(&button_1, GPIOD, GPIO_PIN_1) == 1) {
		  estado ++;
		  if (estado >= 3) {estado = 0;}
		  button_1 = 0;
	  }

	  if(debouncer(&button_2, GPIOD, GPIO_PIN_2) == 1) {
		  	  preestado = estado;
		  	  estado = 3;
		  	  __HAL_TIM_SET_COUNTER(&htim8, 0);
	  		  button_2 = 0;
	  	  }


	  motion();

	  switch(estado) {
	  	  case 0: // modo manual
	  		  //mostramos el estado en el que estamos
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
		  	  //
		  if(__HAL_TIM_GET_COUNTER(&htim8)<100) { //no leemos distancia
			  }
		  else {
			  __HAL_TIM_SET_COUNTER(&htim8, 0); //ponemos a cero el temporizador
			  distancia = ultrasonicRead(); // ahora si, leemos distancia
		  }

		  if(distancia > 3 && distancia < 11) {
			  //obstaculo
		  }
		  else if (distancia == 3){
			  //solo se permite subir
			  if(joystick_arriba()) { step_motor_angle(1,1,5); }
		  }
		  else if (distancia == 11) {
			  // funcionamiento normal
			  if (hall()) {if(joystick_abajo()) { step_motor_angle(1,0,5); }}
			 		  else {
			 			  if(joystick_arriba()) { step_motor_angle(1,1,5); }
			 			  if(joystick_abajo()) { step_motor_angle(1,0,5); }
			 		  }
		  }
		  break;


	  	  case 1: // modo automatico 1: "invierno / intimidad"
	  		  // mostramos el estado en el que estamos
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	 	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);

	 	  if(__HAL_TIM_GET_COUNTER(&htim8)<100) { //no leemos distancia
	 	  }
	 	  else {
	 		  __HAL_TIM_SET_COUNTER(&htim8, 0); //ponemos a cero el temporizador
	 		  distancia = ultrasonicRead(); // ahora si, leemos distancia
	 	  }

	 	  if (ldr()>1000 && distancia > 3) {
	 		 if(distancia > 3 && distancia < 11) {
	 			 //obstaculo
	 		 }
	 		 else {
	 			 step_motor_angle(1,0,5); //bajando
	 		 }
	 	  }
	 	  else if (ldr()<1000 && hall()==0) {
	 			 step_motor_angle(1,1,5); //subiendo
	 	  }
	 	  break;



	  	  case 2:  // modo automatico 2: "verano / inverso"
	  		  // mostramos el estado en el que estamos
	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);

	 	 if(__HAL_TIM_GET_COUNTER(&htim8)<100) { //no leemos distancia
	 	 }
	 	 else {
	 		 __HAL_TIM_SET_COUNTER(&htim8, 0); //ponemos a cero el temporizador
	 		 distancia = ultrasonicRead(); // ahora si, leemos distancia
	 	 }

	 	 if (ldr()<800 && distancia > 3) {
	 		 if(distancia > 3 && distancia < 11) {
	 			 //obstaculo
	 		 }
	 		 else {
	 			 step_motor_angle(1,0,5); // bajando
	 		 }
	 	 }
	 	 else if (ldr()>800 && hall()==0) {
	 		 			 step_motor_angle(1,1,5); // subiendo
	 	 }
	 	 break;




	  	  case 3: // caja fuerte?
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);

	      aux = keypad();
	      	  	  	  	  // if (aux != 'x') { borrar = aux; }
	      if (aux == '1') { key[0] = aux; }
	      if (aux == '5' && key[0] == '1') { key[1] = aux; }
	      if (aux == '2' && key[1] == '5') { key[2] = aux; }
	      if (aux == '1' && key[2] == '2') {
	    	  key[3] = aux;
	    	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	    	  __HAL_TIM_SET_COUNTER(&htim8, 0);
	      }

	      if(__HAL_TIM_GET_COUNTER(&htim8)>=8000) {
	    		  estado = preestado;
	    		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
	    		  key[0] = 0; key[1] = 0; key[2] = 0; key[3] = 0;
	      }

	  	  break;

	  	  default:
	  		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	  		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	  		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	      break;


	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_10B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 40-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 40000-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0xffff-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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

