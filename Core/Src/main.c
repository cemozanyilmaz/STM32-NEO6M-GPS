/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : MPU6050 IMU measurement application using STM32 HAL
 * @author         : Cem Ozan Yilmaz
 * @date           : 12.09.2026 (dd/mm/yyyy)
 ******************************************************************************
 * @details
 *
 * This application interfaces the MPU6050 IMU with a NUCLEO-L476RG
 * over I2C using a custom STM32 HAL-based MPU6050 driver.
 *
 * The application:
 * - Verifies the sensor identity using the WHO_AM_I register
 * - Wakes the MPU6050 and configures its operating parameters
 * - Configures the accelerometer and gyroscope measurement ranges
 * - Configures the digital low-pass filter and sample rate
 * - Reads acceleration, angular velocity, and internal temperature
 * - Sends the measurement results over UART
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "neo6m.h"
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

/* USER CODE BEGIN PV */
 NEO6M_Data gps_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  NEO6M_Init(&huart1);
  char uart_buffer[320];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
    NEO6M_Process();
    gps_data = NEO6M_GetData();

    int32_t lat = (int32_t)(gps_data.latitude * 1000000);
    int32_t lon = (int32_t)(gps_data.longitude * 1000000);
    int32_t hdop = (int32_t)(gps_data.hdop * 100);
    int32_t altitude = (int32_t)(gps_data.altitude * 100);
    int32_t pdop = (int32_t)(gps_data.pdop * 100);
    int32_t vdop = (int32_t)(gps_data.vdop * 100);

    char lat_dir = (gps_data.latitude >= 0) ? 'N' : 'S';
    char lon_dir = (gps_data.longitude >= 0) ? 'E' : 'W';


    snprintf(uart_buffer, sizeof(uart_buffer),
            "Valid: %d | Time: %02d:%02d:%02d | Date: %02d/%02d/%02d | Lat: %ld.%06ld %c | Lon: %ld.%06ld %c | Speed: %ld.%02ld | Course: %ld.%02ld | Fix: %d | Fix Type: %d | Sat: %d | View: %d | PDOP: %ld.%02ld | HDOP: %ld.%02ld | VDOP: %ld.%02ld | Alt: %ld.%02ld m | Used PRN: ",
            gps_data.valid,
            gps_data.hour,
            gps_data.minute,
            gps_data.second,
            gps_data.day,
            gps_data.month,
            gps_data.year,
            (long)(lat / 1000000),
            (long)labs(lat % 1000000),
            lat_dir,
            (long)(lon / 1000000),
            (long)labs(lon % 1000000),
            lon_dir,
            (long)gps_data.speed_knots,
            (long)labs((long)(gps_data.speed_knots * 100) % 100),
            (long)gps_data.course,
            (long)labs((long)(gps_data.course * 100) % 100),
            gps_data.fix_quality,
            gps_data.fix_type,
            gps_data.satellites,
            gps_data.satellites_in_view,
            (long)(pdop / 100),
            (long)labs(pdop % 100),
            (long)(hdop / 100),
            (long)labs(hdop % 100),
            (long)(vdop / 100),
            (long)labs(vdop % 100),
            (long)(altitude / 100),
            (long)labs(altitude % 100));

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)uart_buffer,
                      strlen(uart_buffer),
                      HAL_MAX_DELAY);


    /*
    * Print satellites used in position fix.
    */
    for (uint8_t i = 0; i < gps_data.satellite_prn_count; i++)
    {
        snprintf(uart_buffer, sizeof(uart_buffer),
                "%02d ",
                gps_data.satellite_prn[i]);

        HAL_UART_Transmit(&huart2,
                          (uint8_t *)uart_buffer,
                          strlen(uart_buffer),
                          HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)"\r\n",
                      2,
                      HAL_MAX_DELAY);


    /*
    * Print satellites currently in view.
    */
    for (uint8_t i = 0; i < gps_data.gsv_satellite_count; i++)
    {
        snprintf(uart_buffer, sizeof(uart_buffer),
                "Satellite %02d | PRN: %02d | Elevation: %02d deg | Azimuth: %03d deg | SNR: %02d dB-Hz\r\n",
                i + 1,
                gps_data.gsv_satellites[i].prn,
                gps_data.gsv_satellites[i].elevation,
                gps_data.gsv_satellites[i].azimuth,
                gps_data.gsv_satellites[i].snr);

        HAL_UART_Transmit(&huart2,
                          (uint8_t *)uart_buffer,
                          strlen(uart_buffer),
                          HAL_MAX_DELAY);
    }


    HAL_UART_Transmit(&huart2,
                      (uint8_t *)"\r\n",
                      2,
                      HAL_MAX_DELAY);

    HAL_Delay(3000);
    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        NEO6M_RxCallback();
    }
}

static void SYSCLKConfig_STOP(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  uint32_t pFLatency = 0;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* Enable PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Get the Clocks configuration according to the internal RCC registers */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

  /* Select PLL as system clock source and keep HCLK, PCLK1 and PCLK2 clocks dividers as before */
  RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
  {
    Error_Handler();
  }
}
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
