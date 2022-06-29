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
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//////////////DEFINES///////////////////////////////////////////////////////////////////////

//////////////Para mostrar el estado////////////
#define ESTADO0_PORT GPIOD
#define ESTADO0_PIN GPIO_PIN_12

#define ESTADO1_PORT GPIOD
#define ESTADO1_PIN GPIO_PIN_13

#define ESTADO2_PORT GPIOD
#define ESTADO2_PIN GPIO_PIN_14
///////////////////////////////////////////////


/////////////// Para el motor //////////////////

#define pasosporrev 4096

#define M1_PORT GPIOD
#define M1_PIN GPIO_PIN_8

#define M2_PORT GPIOD
#define M2_PIN GPIO_PIN_9

#define M3_PORT GPIOD
#define M3_PIN GPIO_PIN_10

#define M4_PORT GPIOD
#define M4_PIN GPIO_PIN_11
///////////////////////////////////////////////

//////////////////Para el hall////////////////
#define VAL_LIM_HALL 450
// para mayor sensibilidad aumentar valor; para menor, disminuir
//////////////////////////////////////////////

////////////////// Para ultrasonidos //////////
#define DIST_MIN 3
#define DIST_MAX_1 12
#define DIST_MAX_2 24

#define US_TRIGGER_PORT GPIOD
#define US_TRIGGER_PIN GPIO_PIN_15

#define US_ECHO_PORT GPIOD
#define US_ECHO_PIN GPIO_PIN_4
//////////////////////////////////////////////

////////Para LDR//////////////////////////////
#define UMBRAL_MODO_1 1000
#define UMBRAL_MODO_2 800
/////////////////////////////////////////////

//////////////// Para movimiento /////////////
#define MOTION_PORT GPIOD
#define MOTION_PIN GPIO_PIN_3

#define MOTION_LED_PORT GPIOD
#define MOTION_LED_PIN GPIO_PIN_0
/////////////////////////////////////////////

//////////////////Para el PAD///////////////
#define PAD_S1_PORT GPIOD
#define PAD_S1_PIN GPIO_PIN_5

#define PAD_S2_PORT GPIOD
#define PAD_S2_PIN GPIO_PIN_6

#define PAD_S3_PORT GPIOD
#define PAD_S3_PIN GPIO_PIN_7

#define PAD_E1_PORT GPIOA
#define PAD_E1_PIN GPIO_PIN_9

#define PAD_E2_PORT GPIOA
#define PAD_E2_PIN GPIO_PIN_10

#define PAD_E3_PORT GPIOA
#define PAD_E3_PIN GPIO_PIN_15
////////////////////////////////////////////

//////////Para la CONTRASEÑA///////////////
#define P1 '1'
#define P2 '5'
#define P3 '2'
#define P4 '1'

#define PAD_LED_V_PORT GPIOE
#define PAD_LED_V_PIN GPIO_PIN_7

#define PAD_LED_R_PORT GPIOB
#define PAD_LED_R_PIN GPIO_PIN_2
//////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////VARIABLES//////////////////////////////////////////////////////////

//banderas para los botones y temporizaciones
volatile int button_1 = 0;
volatile int button_2 = 0;
volatile int tiempo1 = 0;
volatile int tiempo2 = 0;

//vector para guardar las lecturas del ADC
uint32_t ADC_val[3];

//variable para guardar la distancia medida, en cm
uint32_t distancia = 0;

//variables para la contraseña
char aux;
char key [4] = {0,0,0,0};

//variables para los estados
uint8_t estado = 0;
uint8_t preestado = 0;

//////////////////////////////////////////////////////////////////////////////////////////////


//////////////INTERRUPCIONES/////////////////////////////////////////////////////////////////
// Dos botones
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_1) {
		button_1 = 1;
	}

	if(GPIO_Pin ==  GPIO_PIN_2) {
		button_2 = 1;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim9)
	{
		tiempo2 = 1;
	}

	if (htim == &htim12)
	{
		tiempo1 = 1;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////


/////////////ANTIRREBOTES///////////////////////////////////////////////////////////////////
// SEgún se realizó en el laboratorio
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
/////////////////////////////////////////////////////////////////////////////////////////////

////////JOYSTICK////////////////////////////////////////////////////////////////////////////
// Una funcion para saber si esta hacia arriba y otra hacia abajo, segun el valor leido en
// el ADC.
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
////////////////////////////////////////////////////////////////////////////////////////////

////////DELAY EN MICROSEGUNDOS/////////////////////////////////////////////////////////////
void delay_us (uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while(__HAL_TIM_GET_COUNTER(&htim3)<us);
}
//////////////////////////////////////////////////////////////////////////////////////////

////////MOTOR PASO A PASO/////////////////////////////////////////////////////////////////
//Funcion para determinar las rpm a las que gira el motor. Funciona bien entre 1 y 13.
//Nosotros lo usaremos a 10 rpm en todos los casos.
void step_motor_set_rpm(int rpm)
{
	delay_us(60000000/pasosporrev/rpm);
}

//Determina la secuencia de los pines a nivel alto/bajo para poder mover el motor.
//Se van activando en orden. El primero se pone a nivel alto, se le añade el segundo,
//se elimina el primero, se añade el tercero, se elimina el segundo, etc.
void step_motor_half (int step) {
	switch(step) {
	case 0:
		HAL_GPIO_WritePin(M1_PORT, M1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2_PORT, M2_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3_PORT, M3_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4_PORT, M4_PIN, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(M1_PORT, M1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2_PORT, M2_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M3_PORT, M3_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4_PORT, M4_PIN, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(M1_PORT, M1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_PORT, M2_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M3_PORT, M3_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4_PORT, M4_PIN, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(M1_PORT, M1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_PORT, M2_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M3_PORT, M3_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M4_PORT, M4_PIN, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(M1_PORT, M1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_PORT, M2_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3_PORT, M3_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M4_PORT, M4_PIN, GPIO_PIN_RESET);
		break;
	case 5:
		HAL_GPIO_WritePin(M1_PORT, M1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_PORT, M2_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3_PORT, M3_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M4_PORT, M4_PIN, GPIO_PIN_SET);
		break;
	case 6:
		HAL_GPIO_WritePin(M1_PORT, M1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M2_PORT, M2_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3_PORT, M3_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4_PORT, M4_PIN, GPIO_PIN_SET);
		break;
	case 7:
		HAL_GPIO_WritePin(M1_PORT, M1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(M2_PORT, M2_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M3_PORT, M3_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(M4_PORT, M4_PIN, GPIO_PIN_SET);
		break;
	}
}

// Finalmente, la funcion que realmente usamos en el main, ya que incluye las dos
// anteriores. Esta funcion hace que el motor gire un angulo introducido, en un
// sentido u otro, y a unas rpm determinadas (entre 1 y 13 rpm funciona).
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
////////////////////////////////////////////////////////////////////////////////////////////////

///////SENSOR DE EFECTO HALL///////////////////////////////////////////////////////////////////
//Se lee como ADC para poder ajustar la sensibilidad a la deseada, segun
//la disposicion fisica final de la maqueta. Es el sensor de fin de carrera
//superior
int hall () {
	HAL_ADC_Start(&hadc2);
	if(HAL_ADC_PollForConversion(&hadc2, 100) ==  HAL_OK) {
		ADC_val[1] = HAL_ADC_GetValue(&hadc2);
	}
	HAL_ADC_Stop(&hadc2);
	if(ADC_val[1] < VAL_LIM_HALL) {return 1;}
	else {return 0;}
}
////////////////////////////////////////////////////////////////////////////////////////////////

////////SENSOR DE ULTRASONIDOS/////////////////////////////////////////////////////////////////
//Lee la distancia que hay de un lado a otro en la parte de abajo, donde
//deberia bajar la persiana. Es utilizado como sensor de fin de carrera
//inferior y tambien como elemento de seguridad, pues si se detecta la
//presencia de un obstaculo, no se baja la persiana.
//
//Se envia una señal a nivel alto de 10 microsegundos. Se espera un tiempo
//durante el cual el sensor emite ultrasonidos y finalmente se lee durante
//cuantos microsegundos la señal que devuelve el sensor se mantiene a
//nivel alto.
uint32_t ultrasonicRead (void) {
	int cuenta = 0;
	HAL_GPIO_WritePin(US_TRIGGER_PORT, US_TRIGGER_PIN, 1);
	delay_us(10);
	HAL_GPIO_WritePin(US_TRIGGER_PORT, US_TRIGGER_PIN, 0);
	while(!(HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN)));
	while(HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN)) {
		cuenta = cuenta + 1;
		delay_us(1);
	}
	return cuenta * 0.051;
}
///////////////////////////////////////////////////////////////////////////////////////////////

///////LDR////////////////////////////////////////////////////////////////////////////////////
uint32_t ldr () {
	HAL_ADC_Start(&hadc3);
	if(HAL_ADC_PollForConversion(&hadc3, 100) ==  HAL_OK) {
		ADC_val[2] = HAL_ADC_GetValue(&hadc3);
	}
	HAL_ADC_Stop(&hadc3);
	return ADC_val[2];
}
//////////////////////////////////////////////////////////////////////////////////////////////

////SENSOR DE MOVIMIENTO/////////////////////////////////////////////////////////////////////
//Cuando el sensor detecta movimiento, su salida se pone a nivel alto durante un tiempo
//determinado por la posicion de un potenciometro. Otro potenciometro determina la
//sensibilidad.
//En esta funcion simplemente se traduce el estado de esa salida del sensor a un LED
void motion () {
	 if(HAL_GPIO_ReadPin(MOTION_PORT, MOTION_PIN)) {  HAL_GPIO_WritePin(MOTION_LED_PORT, MOTION_LED_PIN, 1); }
	 else {  HAL_GPIO_WritePin(MOTION_LED_PORT, MOTION_LED_PIN, 0); }
}
//////////////////////////////////////////////////////////////////////////////////////////////

/////PANEL DE CONTRASEÑA/////////////////////////////////////////////////////////////////////
//Su funcionamiento es el siguiente: el panel es una matriz. Las columnas se van poniendo
//a nivel alto una a la vez y, en cada caso, se lee si alguna de las filas se pone a
//nivel alto. En caso afirmativo, se ha producido el contacto entre la columna y la fila
//correspondiente.
char keypad (void)
{
	//LED rojo se enciende mientras estemos permitiendo la introduccion de contraseña
	HAL_GPIO_WritePin(PAD_LED_R_PORT, PAD_LED_R_PIN, 1);
	// Ponemos solo la primera columna a nivel alto
	HAL_GPIO_WritePin (PAD_S1_PORT, PAD_S1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (PAD_S2_PORT, PAD_S2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (PAD_S3_PORT, PAD_S3_PIN, GPIO_PIN_RESET);

	if ((HAL_GPIO_ReadPin (PAD_E1_PORT, PAD_E1_PIN)))
	{
		while ((HAL_GPIO_ReadPin (PAD_E1_PORT, PAD_E1_PIN)));
		return '1';
	}

	if ((HAL_GPIO_ReadPin (PAD_E2_PORT, PAD_E2_PIN)))
	{
		while ((HAL_GPIO_ReadPin (PAD_E2_PORT, PAD_E2_PIN)));
		return '2';
	}

	if ((HAL_GPIO_ReadPin (PAD_E3_PORT, PAD_E3_PIN)))
	{
		while ((HAL_GPIO_ReadPin (PAD_E3_PORT, PAD_E3_PIN)));
		return '3';
	}

	// Ahora la segunda
	HAL_GPIO_WritePin (PAD_S1_PORT, PAD_S1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (PAD_S2_PORT, PAD_S2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin (PAD_S3_PORT, PAD_S3_PIN, GPIO_PIN_RESET);

	if ((HAL_GPIO_ReadPin (PAD_E1_PORT, PAD_E1_PIN)))
		{
			while ((HAL_GPIO_ReadPin (PAD_E1_PORT, PAD_E1_PIN)));
			return '4';
		}

		if ((HAL_GPIO_ReadPin (PAD_E2_PORT, PAD_E2_PIN)))
		{
			while ((HAL_GPIO_ReadPin (PAD_E2_PORT, PAD_E2_PIN)));
			return '5';
		}

		if ((HAL_GPIO_ReadPin (PAD_E3_PORT, PAD_E3_PIN)))
		{
			while ((HAL_GPIO_ReadPin (PAD_E3_PORT, PAD_E3_PIN)));
			return '6';
		}

	// La tercera
	HAL_GPIO_WritePin (PAD_S1_PORT, PAD_S1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (PAD_S2_PORT, PAD_S2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin (PAD_S3_PORT, PAD_S3_PIN, GPIO_PIN_SET);

	if ((HAL_GPIO_ReadPin (PAD_E1_PORT, PAD_E1_PIN)))
			{
				while ((HAL_GPIO_ReadPin (PAD_E1_PORT, PAD_E1_PIN)));
				return '7';
			}

			if ((HAL_GPIO_ReadPin (PAD_E2_PORT, PAD_E2_PIN)))
			{
				while ((HAL_GPIO_ReadPin (PAD_E2_PORT, PAD_E2_PIN)));
				return '8';
			}

			if ((HAL_GPIO_ReadPin (PAD_E3_PORT, PAD_E3_PIN)))
			{
				while ((HAL_GPIO_ReadPin (PAD_E3_PORT, PAD_E3_PIN)));
				return '9';
			}
	return 'x';
}

//Esta funcion comprueba si se introduce la contraseña correcta
//sin sobrepasar el tiempo limite. El motivo de este diseño
//es que a veces se detectan numeros aleatorios por contactos
//indeseados, suponemos que por la baja calidad del panel.
void password () {
	if (aux == P1) { key[0] = aux; }
	if (aux == P2 && key[0] == P1) { key[1] = aux; }
	if (aux == P3 && key[1] == P2) { key[2] = aux; }
	if (aux == P4 && key[2] == P3) {
		key[3] = aux;
		HAL_GPIO_WritePin(PAD_LED_V_PORT, PAD_LED_V_PIN, 1);
		tiempo2 = 0;
		__HAL_TIM_SET_COUNTER(&htim9, 0);

	}
	if(tiempo2) {
		estado = preestado;
		HAL_GPIO_WritePin(PAD_LED_R_PORT, PAD_LED_R_PIN, 0);
		HAL_GPIO_WritePin(PAD_LED_V_PORT, PAD_LED_V_PIN, 0);
		key[0] = 0; key[1] = 0; key[2] = 0; key[3] = 0;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////


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
  MX_TIM12_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start(&htim3);
HAL_TIM_Base_Start_IT(&htim9);
HAL_TIM_Base_Start_IT(&htim12);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(debouncer(&button_1, GPIOD, GPIO_PIN_1) == 1) {
		  if(estado == 3) {
			  // Nada
		  }
		  else {
			  estado ++;
			  if (estado >= 3) {estado = 0;}
			  	  button_1 = 0;
		  	  }
	  }

	  if(debouncer(&button_2, GPIOD, GPIO_PIN_2) == 1) {
		  	  preestado = estado;
		  	  estado = 3;
		  	 tiempo2 = 0;
		  	__HAL_TIM_SET_COUNTER(&htim9, 0);
	  		  button_2 = 0;
	  	  }


	  motion(); //sensor de movimiento + led correspondiente gestionado aqui

	  //switch case para los estados
	  switch(estado) {
	  	  case 0: // modo manual
	  		  //mostramos el estado en el que estamos en primer lugar
	  		  HAL_GPIO_WritePin(ESTADO0_PORT, ESTADO0_PIN, 1);
	  		  HAL_GPIO_WritePin(ESTADO1_PORT, ESTADO1_PIN, 0);
	  		  HAL_GPIO_WritePin(ESTADO2_PORT, ESTADO2_PIN, 0);
		  	  //leemos la distancia:
	  		  if(tiempo1 == 0) {  //no leemos distancia: el por qué
			  	  	  		//está en las características del
			  	  	  	  	//sensor
			  }
	  		  else {
	  			  distancia = ultrasonicRead(); // ahora si, leemos distancia
	  			  tiempo1 = 0;
	  		  }
		  	  //permitimos el movimiento del motor segun la situacion:
	  		  if(distancia > DIST_MIN && distancia < DIST_MAX_1) {
			  //obstaculo o persiana abajo. Solamente permitimos la subida.
	  			  if(joystick_arriba()) { step_motor_angle(1,1,5); }
	  		  }
	  		  else if (distancia >= DIST_MAX_1 && distancia <= DIST_MAX_2) {
			  // funcionamiento normal, permitimos subida (solo si el hall lo permite) y bajada.
	  			  if (hall()) {if(joystick_abajo()) { step_motor_angle(1,0,5); }}
	  			  else {
			 			  if(joystick_arriba()) { step_motor_angle(1,1,5); }
			 			  if(joystick_abajo()) { step_motor_angle(1,0,5); }
	  			  }
	  		  }
	  		  break;


	  	  case 1: // modo automatico 1: "invierno / intimidad"
	  		  // mostramos el estado en el que estamos
	  		  HAL_GPIO_WritePin(ESTADO0_PORT, ESTADO0_PIN, 0);
	  		  HAL_GPIO_WritePin(ESTADO1_PORT, ESTADO1_PIN, 1);
	  		  HAL_GPIO_WritePin(ESTADO2_PORT, ESTADO2_PIN, 0);

	  		  if(tiempo1 == 0) { //no leemos distancia: el por qué
  	  	  	  	  	  	  //está en las características del
  	  	  	  	  	  	  //sensor
	  		  }
	  		  else {
	  			  distancia = ultrasonicRead(); // ahora si, leemos distancia
	  			  tiempo1 = 0;
	  		  }
	 	  	  //Según el valor del LDR actuamos de una u otra forma.
	  		  if (ldr()>UMBRAL_MODO_1 && distancia > DIST_MIN) {
	  			  if(distancia > DIST_MIN && distancia < DIST_MAX_1) {
	 			 //obstaculo o persiana abajo
	  			  }
	  			  else {
	  				  step_motor_angle(1,0,5); //bajando
	  			  }
	  		  }
	  		  else if (ldr()<UMBRAL_MODO_1 && hall()==0) {
	  			  step_motor_angle(1,1,5); //subiendo mientras hall lo permita
	  		  }
	  		  break;



	  	  case 2:  // modo automatico 2: "verano / inverso"
	  		  // mostramos el estado en el que estamos
	  		  HAL_GPIO_WritePin(ESTADO0_PORT, ESTADO0_PIN, 0);
	  		  HAL_GPIO_WritePin(ESTADO1_PORT, ESTADO1_PIN, 0);
	  		  HAL_GPIO_WritePin(ESTADO2_PORT, ESTADO2_PIN, 1);
	      	  //leemos la distancia
	  		  if(tiempo1 == 0) { //no leemos distancia: el por qué
 	  	  	  	  	  	  //está en las características del
	 		 	 	 	  //sensor
	  		  }
	  		  else {
	  			  distancia = ultrasonicRead(); // ahora si, leemos distancia
	  			  tiempo1 = 0;
	  		  }
	 	 	 //segun el valor del ldr, actuamos de una u otra forma.
	  		  if (ldr()<UMBRAL_MODO_2 && distancia > DIST_MIN) {
	  			  if(distancia > DIST_MIN && distancia < DIST_MAX_1) {
	 			 //obstaculo o persiana abajo
	  			  }
	  			  else {
	  				  step_motor_angle(1,0,5); // bajando
	  			  }
	  		  }
	  		  else if (ldr()>UMBRAL_MODO_2 && hall()==0) {
	  			  step_motor_angle(1,1,5); // subiendo mientras el hall lo permita
	  		  }
	  		  break;




	  	  case 3: // caja fuerte, contraseña
	  		  //apagamos los estados 0, 1 , 2
	  		  //el estado actual se muestra con otro led,
	  		  //gestionado en las funciones keypad() y password()
	  		  HAL_GPIO_WritePin(ESTADO0_PORT, ESTADO0_PIN, 0);
	  		  HAL_GPIO_WritePin(ESTADO1_PORT, ESTADO1_PIN, 0);
	  		  HAL_GPIO_WritePin(ESTADO2_PORT, ESTADO2_PIN, 0);
	  		  aux = keypad(); //Se lee si se pulsa algun boton
	  		  password(); // Se comprueba si se van introduciendo
		  	  	  	  // los numeros correctos. Se enciende un LED
		  	  	  	  // verde en caso de éxito.
	  		  break;

	  	  default:
	  		  HAL_GPIO_WritePin(ESTADO0_PORT, ESTADO0_PIN, 0);
	  		  HAL_GPIO_WritePin(ESTADO1_PORT, ESTADO1_PIN, 0);
	  		  HAL_GPIO_WritePin(ESTADO2_PORT, ESTADO2_PIN, 0);
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
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 40000-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 8000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 40000-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 100-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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

