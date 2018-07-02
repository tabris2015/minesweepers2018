/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
volatile uint16_t adc_scan[SCAN_MAX];
volatile uint16_t adc_scan_avg[SCAN_MAX];
float32_t v_adc_mV[SCAN_MAX];
float32_t v_bat_mV;

float32_t i_motor_mA[MOTOR_MAX][IS_MAX] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float32_t i_quis_mA[MOTOR_MAX][IS_MAX] = {1.22, 1.409, 1353.281, 112.503, 0.0, 0.0, 0.0, 0.0};

float32_t bemf_mV[MOTOR_MAX] = {0.0, 0.0, 0.0, 0.0};
float32_t k_vel[MOTOR_MAX] = {100.0, 100.0, 100.0, 100.0};

float32_t pid_k[MOTOR_MAX][PID_MAX] = {	{1.0, 75.0, 0.005}, \
																				{1.0, 75.0, 0.005}, \
																				{1.0, 75.0, 0.005}, \
																				{1.0, 75.0, 0.005}	};
float32_t pid_e[MOTOR_MAX][PID_MAX] = {	{0.0, 0.0, 0.0}, \
																				{0.0, 0.0, 0.0}, \
																				{0.0, 0.0, 0.0}, \
																				{0.0, 0.0, 0.0}	};
float32_t const a_r = 0.01 / (0.01 + 0.05); // T / (T + tau)

float32_t u[MOTOR_MAX] = {0.0, 0.0, 0.0, 0.0};
float32_t r[MOTOR_MAX] = {0.0, 0.0, 0.0, 0.0};

float32_t w_n[MOTOR_MAX][2] =	{	{0.188, 0.776}, \
																{0.188, 0.776}, \
																{0.188, 0.776}, \
																{0.188, 0.776}	};
float32_t u_n_global[MOTOR_MAX][2] = {{0.0, 0.0}, \
																			{0.0, 0.0}, \
																			{0.0, 0.0}, \
																			{0.0, 0.0}};

float32_t r_global[MOTOR_MAX];	// vel desired in rad/s
float32_t d_global[MOTOR_MAX];	// vel estimation in rad/s

float32_t P_n[MOTOR_MAX][4] = {	{	10.0,	0.0,			\
																	0.0, 	10.0	},	\
																{	10.0,	0.0,			\
																	0.0, 	10.0	},	\
																{	10.0,	0.0,			\
																	0.0, 	10.0	},	\
																{	10.0,	0.0,			\
																	0.0, 	10.0,	}	};
float32_t k_n[2], aux41[2], aux14[2], aux44[4];
float32_t a = 0.999, aux, e_n, y_n;
float32_t ainv;

arm_matrix_instance_f32 mP_n[MOTOR_MAX];
arm_matrix_instance_f32 mk_n;
arm_matrix_instance_f32 mu_n[MOTOR_MAX];
arm_matrix_instance_f32 muT_n[MOTOR_MAX];
arm_matrix_instance_f32 maux41;
arm_matrix_instance_f32 maux14;
arm_matrix_instance_f32 maux44;

uint16_t r_motor_mOhm[MOTOR_MAX] = {1300, 700, 1200, 1300};
uint16_t current_k[MOTOR_MAX][IS_MAX] = {{696, 696}, {696, 696}, {8500, 8500}, {8500, 8500}};
uint16_t r_sense_Ohm = 1000;
uint16_t C_mm = 200;
uint16_t R_mm = 125;

volatile uint32_t * pwm_value[MOTOR_MAX][IS_MAX];

volatile uint8_t adc_avg_cplt = 0;
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PB0     ------> ADC1_IN8 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PB0     ------> ADC1_IN8 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

void updateRef(void) {
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// TIM3 callback, 1000Hz interrupt (sampling frec)
	if (htim->Instance == TIM4) {
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_scan, SCAN_MAX);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
	volatile static uint8_t cnt = 0;
	volatile static uint16_t adc_scan_sum[SCAN_MAX] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	
	if (AdcHandle->Instance == ADC1) {
		for (uint8_t i = 0; i < SCAN_MAX; i++)
			adc_scan_sum[i] += adc_scan[i];
		cnt++;
		if (cnt >= 10) {
			for (uint8_t i = 0; i < SCAN_MAX; i++) {
				adc_scan_avg[i] = adc_scan_sum[i] / cnt;
				adc_scan_sum[i] = 0;
			}
			adc_avg_cplt = 1;
			cnt = 0;
		}
	}
}

void control_loop(void){
	uint8_t i, j;
	float32_t aux1, aux2, aux3, y_est[MOTOR_MAX];
	
	adc_avg_cplt = 0;
	
	// data update
	for (i = 0; i < MOTOR_MAX; i++) {
		u_n_global[i][1] = d_global[i];
		u_n_global[i][0] = u[i];
	}
	
	// data measurement
	for (i = 0; i < SCAN_MAX; i++) {
		v_adc_mV[i] = adc_scan_avg[i] * 3300.0 / 4095;
	}
	v_bat_mV = v_adc_mV[V_BAT] * 13.16 / 3.3;
	
	aux1 = mem_data[V_LIN] * 1000;
	aux2 = C_mm * mem_data[W_ANG];
	aux3 = (aux1 + aux2) / R_mm;
	r_global[MOTOR_B_R] = aux3;
	r_global[MOTOR_F_R] = aux3;
	aux3 = (aux1 - aux2) / R_mm;
	r_global[MOTOR_B_L] = aux3;
	r_global[MOTOR_F_L] = aux3;
	
	for (i = 0; i < MOTOR_MAX; i++) {
		aux1 = d_global[i];
		
		i_motor_mA[i][R] = v_adc_mV[i * 2] / r_sense_Ohm * current_k[i][R] - i_quis_mA[i][R];
		//i_motor_mA[i][R] -= i_quis_mA[i][R];
		i_motor_mA[i][L] = v_adc_mV[i * 2 + 1] / r_sense_Ohm * current_k[i][L] - i_quis_mA[i][L];
		//i_motor_mA[i][L] -= i_quis_mA[i][L];
		
		bemf_mV[i] = v_bat_mV * (*pwm_value[i][R] - *pwm_value[i][L]) / 4095;
		bemf_mV[i] -= (i_motor_mA[i][R] - i_motor_mA[i][L]) * r_motor_mOhm[i] / 1000;
		d_global[i] = bemf_mV[i] * 2 * PI / 60 / k_vel[i];
		arm_dot_prod_f32(w_n[i], u_n_global[i], 2, &y_est[i]);
		mem_data[i] = y_est[i];
		
		r[i] = (1 - a_r) * r[i] + a_r * r_global[i];
		
		pid_e[i][P] = r[i] - mem_data[i];
		u[i] = pid_k[i][P] * pid_e[i][P];
		pid_e[i][D] = (aux1 - d_global[i]) / 0.01;
		u[i] += pid_k[i][D] * pid_e[i][D];
		pid_e[i][I] += pid_e[i][P] * 0.01;
		aux2 = pid_k[i][I] * pid_e[i][I];
		if (aux2 > 6.2832) {
			pid_e[i][I] = 6.2832;
		}
		if (aux2 < -6.2832) {
			pid_e[i][I] = -6.2832;
		}
		u[i] += aux2;
		u[i] = r[i];
		
		aux3 = d_global[i] * 1.0;
		if (u[i] > 6.2832) {
			u[i] = 6.2832;
		}
		if (u[i] < -6.2832) {
			u[i] = -6.2832;
		}
		
		if (u[i] < 0.0) {
			*pwm_value[i][R] = 0;
			*pwm_value[i][L] = (uint16_t)(-u[i] * 4095 / 6.2832);
		}
		else {
			*pwm_value[i][L] = 0;
			*pwm_value[i][R] = (uint16_t)(u[i] * 4095 / 6.2832);
		}
	}
	
	j++;
	if (j == 5) {
		j = 0;
		identification_loop();
	}
}

void identification_loop(void) {
	static const float32_t a = 0.999;
	static float32_t ainv = 1/a;
	
	float32_t aux, e_n, y_n;
	uint8_t i;
	
	for (i = 0; i < MOTOR_MAX; i++) {
		//RLS motor i
		arm_mat_mult_f32(&mP_n[i], &mu_n[i], &maux41);
		arm_dot_prod_f32(u_n_global[i], aux41, 2, &aux);
		aux = 1/(a + aux);
		arm_scale_f32(aux41, aux, k_n, 2);
		
		arm_dot_prod_f32(w_n[i], u_n_global[i], 2, &y_n);
		
		e_n = d_global[i] - y_n;
		
		arm_scale_f32(k_n, e_n, aux41, 2);
		arm_add_f32(w_n[i], aux41, w_n[i], 2);
		
		arm_mat_mult_f32(&muT_n[i], &mP_n[i], &maux14);
		arm_mat_mult_f32(&mk_n, &maux14, &maux44);
		arm_mat_sub_f32(&mP_n[i], &maux44, &maux44);
		arm_mat_scale_f32(&maux44, ainv, &mP_n[i]);
	}
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
