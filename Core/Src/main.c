/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Główny plik projektu. Realizuje nieblokującą obsługę
  * czujnika MPU6050 (I2C + DMA) oraz komunikację UART z PC.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "protocol.h"     // Deklaracje funkcji i stałych naszego protokołu
#include <string.h>       // Funkcje pamięciowe (np. memcpy)
#include <stdbool.h>      // Obsługa typu bool
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Struktura przechowująca pojedynczą próbkę z akcelerometru.
   Używamy int16_t, ponieważ MPU6050 zwraca dane w 16-bitowym kodzie U2 ze znakiem. */
typedef struct {
  int16_t x;  // Przyspieszenie w osi X
  int16_t y;  // Przyspieszenie w osi Y
  int16_t z;  // Przyspieszenie w osi Z
} accel_sample_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Definicje stałych sprzętowych dla czujnika MPU6050 */
#define MPU6050_ADDR_DEFAULT (0x68 << 1) // Domyślny adres I2C (przesunięty o 1 bit dla biblioteki HAL)
#define MPU6050_ADDR_ALT     (0x69 << 1) // Alternatywny adres I2C (gdy pin AD0 = 1)
#define MPU6050_PWR_MGMT_1   0x6B        // Rejestr zarządzania energią (służy m.in. do wybudzania)
#define MPU6050_ACCEL_XOUT_H 0x3B        // Adres pierwszego rejestru przechowującego wyniki pomiarów
#define MPU6050_WHO_AM_I     0x75        // Rejestr identyfikacji układu (oczekiwana wartość: 0x68)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* --- ZMIENNE DO OBSŁUGI BUFORÓW KOŁOWYCH UART --- */
#define USART_RXBUF_LEN 1024  // Rozmiar bufora dla danych przychodzących z PC
#define USART_TXBUF_LEN 2048  // Rozmiar bufora dla danych wysyłanych do PC

uint8_t USART_RxBuf[USART_RXBUF_LEN]; // Pamięć bufora odbiorczego
uint8_t USART_TxBuf[USART_TXBUF_LEN]; // Pamięć bufora nadawczego

/* Wskaźniki buforów kołowych (volatile, ponieważ są modyfikowane wewnątrz przerwań) */
volatile uint16_t USART_RX_Empty = 0; // Wskaźnik zapisu (głowa) bufora RX
volatile uint16_t USART_RX_Busy = 0;  // Wskaźnik odczytu (ogon) bufora RX
volatile uint16_t USART_TX_Empty = 0; // Wskaźnik zapisu (głowa) bufora TX
volatile uint16_t USART_TX_Busy = 0;  // Wskaźnik odczytu (ogon) bufora TX

/* --- ZMIENNE PROJEKTOWE --- */
static uint16_t g_mpu_addr = 0;        // Przechowuje wykryty i potwierdzony adres I2C czujnika
volatile uint8_t g_mpu_init_done = 0;  // Flaga informująca o zakończeniu procesu inicjalizacji
volatile uint8_t g_mpu_init_ok   = 0;  // Flaga informująca, czy inicjalizacja zakończyła się sukcesem

/* FLAGA SYNCHRONIZACYJNA: PRZERWANIE -> PĘTLA GŁÓWNA */
volatile uint8_t g_mpu_new_data = 0;   // Ustawiana w callbacku I2C, gdy DMA zakończy odczyt próbek

/* Główny bufor cykliczny na przeliczone próbki z czujnika */
accel_sample_t accel_buf[ACCEL_BUFFER_SIZE]; // Tablica o rozmiarze 1024 próbek (zdefiniowane w protocol.h)
volatile uint16_t accel_head  = 0;           // Wskaźnik zapisu nowej próbki (głowa bufora)
volatile uint32_t accel_count = 0;           // Całkowita liczba zebranych próbek od włączenia systemu

volatile uint8_t  g_acq_enabled  = 0;    // Flaga określająca, czy proces próbkowania jest włączony (START/STOP)
volatile uint32_t g_interval_ms  = 100;  // Interwał próbkowania w milisekundach (domyślnie 100 ms)
volatile uint32_t g_tick10ms     = 0;    // Globalny licznik inkrementowany przez przerwanie Timera
volatile uint32_t g_time_base_ms = 0;    // Zmienna przechowująca referencyjny czas podany przez PC

/* Bufory robocze dla transferów DMA magistrali I2C */
static uint8_t mpu_raw[6];      // Bufor na 6 surowych bajtów pomiarowych z MPU6050
static uint8_t whoami_raw = 0;  // Bufor na odpowiedź z rejestru WHO_AM_I

/* Zmienne zarządzające stanem magistrali I2C */
static volatile uint8_t s_i2c_busy = 0; // Semafor (1 = magistrala zajęta transferem DMA)
typedef enum { I2C_OP_NONE, I2C_OP_PROBE_WHOAMI, I2C_OP_WAKE_WRITE, I2C_OP_ACCEL_READ } i2c_op_t;
static volatile i2c_op_t s_i2c_op = I2C_OP_NONE; // Kod aktualnie wykonywanej operacji I2C
static uint16_t s_probe_addr = 0;                // Tymczasowo testowany adres urządzenia

/* Zmienne do nieblokującej obsługi czasu i diody LED */
static uint32_t s_next_acq_ms = 0;      // Znak czasu, kiedy należy wykonać następny pomiar
static uint8_t  s_led_hold_active = 0;  // Flaga informująca, czy dioda LED powinna się aktualnie świecić
static uint32_t s_led_hold_until  = 0;  // Znak czasu, kiedy należy zgasić diodę LED
static uint8_t  s_mpu_led_hold_started = 0; // Zabezpieczenie przed wielokrotnym miganiem diodą przy starcie

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void LED_Service(void);
static void LED_OnForMs(uint32_t ms);
static void MPU6050_Init_Service(void);
static void MPU_ProcessData(void);
static void Acq_Service(void);

/* Narzędzia inline do obsługi upływu czasu (odporne na przepełnienie zmiennej) */
static inline uint32_t now_ms(void) { return HAL_GetTick(); }
static inline uint8_t time_reached_ms(uint32_t t) { return (int32_t)(now_ms() - t) >= 0; }

/* Funkcje wyzwalające transfery I2C przez DMA */
static uint8_t I2C_Start_WhoAmI_Read(uint16_t addr);
static uint8_t I2C_Start_Wake_Write(uint16_t addr);
static uint8_t I2C_Start_Accel_Read(uint16_t addr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --- FUNKCJA ODCZYTU Z UARTA (DLA PĘTLI GŁÓWNEJ) --- */
uint8_t USART_kbhit(){
	if(USART_RX_Empty==USART_RX_Busy){
		return 0;
	}else{
		return 1;
	}
}

/* Pobiera jeden bajt z cyklicznego bufora odbiorczego */
int16_t USART_getchar(void)
{
    // Jeśli wskaźnik zapisu różni się od wskaźnika odczytu, bufor nie jest pusty
    if (USART_RX_Empty != USART_RX_Busy)
    {
        uint8_t tmp = USART_RxBuf[USART_RX_Busy]; // Pobranie najstarszego bajtu
        USART_RX_Busy++;                          // Przesunięcie wskaźnika odczytu

        // Obsługa zawijania indeksu (circular buffer)
        if (USART_RX_Busy >= USART_RXBUF_LEN) USART_RX_Busy = 0;
        return tmp; // Zwrócenie pobranego bajtu
    }
    return -1; // Brak nowych danych
}

/* --- FUNKCJE OBSŁUGI PRZERWAŃ UART --- */

/* Callback wywoływany przez sprzęt po pomyślnym odebraniu 1 bajtu przez UART */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	 if(huart==&huart2){
		 USART_RX_Empty++; // Inkrementacja wskaźnika zapisu nowej danej
		 if(USART_RX_Empty>=USART_RXBUF_LEN)USART_RX_Empty=0; // Zawinięcie bufora

         // Zlecenie sprzętowi nasłuchu kolejnego bajtu do nowej pozycji w buforze
		 HAL_UART_Receive_IT(&huart2,&USART_RxBuf[USART_RX_Empty],1);
	 }
}

/* Callback wywoływany przez sprzęt po całkowitym wysłaniu 1 bajtu przez UART */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
   if(huart==&huart2){
       // Sprawdzenie, czy w buforze nadawczym znajdują się jeszcze dane do wysłania
	   if(USART_TX_Empty!=USART_TX_Busy){
		   uint8_t tmp=USART_TxBuf[USART_TX_Busy]; // Pobranie bajtu z bufora
		   USART_TX_Busy++; // Przesunięcie wskaźnika odczytu
		   if(USART_TX_Busy >= USART_TXBUF_LEN)USART_TX_Busy=0;

           // Wysłanie pobranego bajtu. Po jego wysłaniu, przerwanie wywoła się ponownie.
		   HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	   }
   }
}

/* --- FUNKCJE OBSŁUGI PRZERWAŃ I2C (ZOPTYMALIZOWANE) --- */

/* Callback wywoływany po zakończeniu transferu odbierania danych przez DMA z magistrali I2C */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (s_i2c_op == I2C_OP_ACCEL_READ) {
      // Zakończono pobieranie próbek - ustawiamy flagę dla pętli głównej
      g_mpu_new_data = 1;
  }
  else if (s_i2c_op == I2C_OP_PROBE_WHOAMI) {
      // Potwierdzono obecność układu - zapisujemy jego adres sprzętowy
      g_mpu_addr = s_probe_addr;
  }

  // Zakończenie operacji: wyzerowanie typu operacji i zwolnienie semafora
  s_i2c_op = I2C_OP_NONE;
  s_i2c_busy = 0;
}

/* Callback wywoływany po zakończeniu transferu wysyłania danych przez DMA z magistrali I2C */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (s_i2c_op == I2C_OP_WAKE_WRITE) {
    // Rejestr wybudzania został poprawnie zapisany
    g_mpu_init_done = 1;
    g_mpu_init_ok = 1;

    // Potwierdzenie wizualne (dioda LED świeci przez 2 sekundy po inicjalizacji)
    if (!s_mpu_led_hold_started) {
        s_mpu_led_hold_started = 1;
        LED_OnForMs(2000);
    }
  }

  // Zwolnienie semafora
  s_i2c_op = I2C_OP_NONE;
  s_i2c_busy = 0;
}

/* Callback wywoływany, gdy na magistrali I2C wystąpi błąd (np. NACK, utrata arbitrażu) */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  // W przypadku błędu zwalniamy zasoby, aby zapobiec deadlockom
  s_i2c_op = I2C_OP_NONE;
  s_i2c_busy = 0;
  g_mpu_init_ok = 0; // Oznaczenie braku komunikacji z czujnikiem
}

/* --- LOGIKA BIZNESOWA: PRZETWARZANIE WYNIKÓW --- */

/* Funkcja uruchamiana w pętli głównej. Przelicza surowe bajty na czytelne wartości. */
static void MPU_ProcessData(void)
{
    if (g_mpu_new_data) // Wykonaj tylko, jeśli przerwanie DMA zgłosiło kompletność ramki
    {
        g_mpu_new_data = 0; // Reset flagi do obsługi kolejnych pomiarów

        // MPU6050 przesyła dane w konwencji Big-Endian (MSB najpierw).
        // Mikrokontroler ARM pracuje w trybie Little-Endian.
        // Wykonujemy przesunięcie bitowe starszego bajtu i logiczne OR z młodszym bajtem.
        accel_buf[accel_head].x = (int16_t)((mpu_raw[0] << 8) | mpu_raw[1]);
        accel_buf[accel_head].y = (int16_t)((mpu_raw[2] << 8) | mpu_raw[3]);
        accel_buf[accel_head].z = (int16_t)((mpu_raw[4] << 8) | mpu_raw[5]);

        // Aktualizacja wskaźnika głowy bufora z zachowaniem właściwości kołowych (modulo)
        accel_head = (accel_head + 1) % ACCEL_BUFFER_SIZE;
        accel_count++; // Inkrementacja globalnego licznika zebranych próbek

        // Wizualna sygnalizacja pracy (zmiana stanu LED przy każdej udanej próbce)
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
}

/* --- POZOSTAŁE FUNKCJE --- */

/* Callback wywoływany przez sprzętowy timer. Buduje podstawę czasu dla aplikacji. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) g_tick10ms++;
}

/* Włącza diodę LED i zapamiętuje czas jej wyłączenia (implementacja nieblokująca) */
static void LED_OnForMs(uint32_t ms)
{
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  s_led_hold_active = 1;
  s_led_hold_until  = now_ms() + ms;
}

/* Sprawdza warunki wyłączenia diody LED (wywoływane cyklicznie w pętli głównej) */
static void LED_Service(void)
{
  if (s_led_hold_active && time_reached_ms(s_led_hold_until)) {
    s_led_hold_active = 0;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  }
}

/* --- INICJOWANIE TRANSFERÓW DMA I2C --- */

/* Wysyła zapytanie do czujnika w celu odczytu jego ID */
static uint8_t I2C_Start_WhoAmI_Read(uint16_t addr)
{
  if (s_i2c_busy) return 0; // Jeśli magistrala zajęta, zrezygnuj
  s_i2c_busy = 1; s_i2c_op = I2C_OP_PROBE_WHOAMI;

  if (HAL_I2C_Mem_Read_DMA(&hi2c1, addr, MPU6050_WHO_AM_I, 1, &whoami_raw, 1) != HAL_OK) {
    s_i2c_busy = 0; return 0;
  }
  return 1; // Żądanie dodane do kolejki sprzętowej pomyślnie
}

/* Wpisuje wartość '0' do rejestru zarządzania energią, wybudzając układ */
static uint8_t I2C_Start_Wake_Write(uint16_t addr)
{
  if (s_i2c_busy) return 0;
  s_i2c_busy = 1; s_i2c_op = I2C_OP_WAKE_WRITE;
  static uint8_t data = 0; // Wartość 0 dla rejestru PWR_MGMT_1 oznacza działanie ciągłe

  if (HAL_I2C_Mem_Write_DMA(&hi2c1, addr, MPU6050_PWR_MGMT_1, 1, &data, 1) != HAL_OK) {
    s_i2c_busy = 0; return 0;
  }
  return 1;
}

/* Żąda odczytu 6 bajtów danych (rejestry X_H, X_L, Y_H, Y_L, Z_H, Z_L) */
static uint8_t I2C_Start_Accel_Read(uint16_t addr)
{
  if (s_i2c_busy) return 0;
  s_i2c_busy = 1; s_i2c_op = I2C_OP_ACCEL_READ;

  if (HAL_I2C_Mem_Read_DMA(&hi2c1, addr, MPU6050_ACCEL_XOUT_H, 1, mpu_raw, 6) != HAL_OK) {
    s_i2c_busy = 0; return 0;
  }
  return 1;
}

/* --- LOGIKA URUCHAMIANIA I KONTROLI CZUJNIKA --- */

/* Maszyna stanów inicjalizująca czujnik (uruchamiana cyklicznie) */
static void MPU6050_Init_Service(void)
{
  // Deklaracja stanów inicjalizacji. State jest typu static, więc pamięta swój stan między wywołaniami.
  static enum { START, TRY_68, WAIT_68, TRY_69, WAIT_69, WAKE, WAIT_DONE, FAILED, DONE } state = START;

  if (state == DONE || state == FAILED) return; // Jeśli proces się zakończył, funkcja nie robi już nic

  switch(state) {
    case START: g_mpu_init_ok = 0; state = TRY_68; break; // Inicjalizacja początku maszyny stanów

    // Krok 1: Próba kontaktu na domyślnym adresie I2C
    case TRY_68: s_probe_addr = MPU6050_ADDR_DEFAULT; if(I2C_Start_WhoAmI_Read(s_probe_addr)) state = WAIT_68; break;
    case WAIT_68: if(!s_i2c_busy) { if(g_mpu_addr) state = WAKE; else state = TRY_69; } break;

    // Krok 2: Próba kontaktu na alternatywnym adresie I2C (jeśli Krok 1 zawiódł)
    case TRY_69: s_probe_addr = MPU6050_ADDR_ALT; if(I2C_Start_WhoAmI_Read(s_probe_addr)) state = WAIT_69; break;
    case WAIT_69: if(!s_i2c_busy) { if(g_mpu_addr) state = WAKE; else state = FAILED; } break;

    // Krok 3: Wybudzenie urządzenia
    case WAKE: if(I2C_Start_Wake_Write(g_mpu_addr)) state = WAIT_DONE; else state = FAILED; break;
    case WAIT_DONE: if(g_mpu_init_ok) state = DONE; else if(!s_i2c_busy) state = FAILED; break;

    default: state = FAILED; break;
  }
}

/* Realizuje harmonogram pobierania próbek z czujnika w czasie */
static void Acq_Service(void)
{
  // Blokada: Brak zezwolenia (komenda STOP z PC) lub czujnik niezainicjalizowany poprawnie
  if (!g_acq_enabled || !g_mpu_init_ok) return;
  // Blokada: Upłynęło za mało czasu od ostatniego pomiaru
  if (!time_reached_ms(s_next_acq_ms)) return;

  // Zabezpieczenie na wypadek ustawienia interwału 0 (ograniczenie dolne do 1ms)
  uint32_t interval = g_interval_ms;
  if (interval == 0) interval = 1;

  // Zapisz czas kolejnego żądanego pomiaru
  s_next_acq_ms = now_ms() + interval;

  // Zleć kontrolerowi DMA odczyt danych z magistrali
  I2C_Start_Accel_Read(g_mpu_addr);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* Inicjalizacja maszyny stanów parsera UART */
  Protocol_Init();

  /* Uruchomienie Timera sprzętowego wyzwalającego przerwania do celów odliczania czasu */
  HAL_TIM_Base_Start_IT(&htim3);

  /* Pierwsze wyzwolenie odbioru przez UART. Po odebraniu 1 bajtu funkcja callback wywoła kolejne */
  HAL_UART_Receive_IT(&huart2, &USART_RxBuf[USART_RX_Empty], 1);

  /* Ustalamy pierwsze wyzwolenie odczytu z czujnika z 50 ms opóźnieniem, dając mu czas na ustabilizowanie zasilania */
  s_next_acq_ms = now_ms() + 50;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    MPU6050_Init_Service(); // Sprawdza postęp inicjalizacji sprzętu
    Acq_Service();          // Wyzwala transfer DMA jeśli minął odpowiedni czas
    MPU_ProcessData();      // Jeżeli pobrano próbki (flaga ustawiona w callbacku), przelicza je na liczby
    LED_Service();          // Analizuje warunki czasowe dla zgaszenia diody

    /* Analiza wszystkich bajtów zebranych asynchronicznie w buforze RX UART */
    int16_t ch;
    while ((ch = USART_getchar()) != -1) {
       Protocol_ParseByte((uint8_t)ch); // Przekazanie bajtu do analizatora ramek protokołu
    }

    /* Wait For Interrupt: usypia rdzeń procesora w oczekiwaniu na zdarzenia sprzętowe.
       Zakomentowane (//), aby uniknąć błędu z programatorem "Target is not responding".
       Odkomentuj na sam koniec po zdebugowaniu projektu. */
    // __WFI();
    /* USER CODE END WHILE */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * where the assert_param error has occurred.
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
