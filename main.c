#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_conf.h"
#include "string.h"

//Версия программного обеспечения
#define ProgrammVersion_D 6 //день
#define ProgrammVersion_M 10 //месяц
#define ProgrammVersion_Y 23 //год

#define Enable_Potenciometr 1 //Задействовать потенциометры
//#define Enable_Potenciometr 0 //Не задействовать потенциометры


//..........................................................................................................................
//..........................................................................................................................
//.................                            ПАРАМЕТРЫ УСТРОЙСТВА                                         ................
//..........................................................................................................................
//.................                            ПАРАМЕТРЫ УСТРОЙСТВА                                         ................
//..........................................................................................................................
//.................                            ПАРАМЕТРЫ УСТРОЙСТВА                                         ................
//..........................................................................................................................
//.................                            ПАРАМЕТРЫ УСТРОЙСТВА                                         ................
//..........................................................................................................................
//.................                            ПАРАМЕТРЫ УСТРОЙСТВА                                         ................
//..........................................................................................................................
//.................                            ПАРАМЕТРЫ УСТРОЙСТВА                                         ................
//..........................................................................................................................
//.................                            ПАРАМЕТРЫ УСТРОЙСТВА                                         ................
//..........................................................................................................................
//.................                            ПАРАМЕТРЫ УСТРОЙСТВА                                         ................
//..........................................................................................................................
//.................                            ПАРАМЕТРЫ УСТРОЙСТВА                                         ................
//..........................................................................................................................
//..........................................................................................................................


//Приоритеты прерываний
#define Prioritet_MainTimer 0 //Максимальный приоритет основного таймера отсчитывающего время и таймауты
#define PrioritetReciveGSM 1 //Приоритет приема байта от GSM модуля
#define PrioritetBluetooth 2 //Приоритет таймера bluetooth
#define Prioritet_LedTimer 3 //Приоритет таймера светодиода

//Значения таймаутов для различных ситуаций (частота таймера 20 кГц)
#define Send_Char_Timeout  10000 //Таймаут отправки байта по Usart 10мс
#define ADC_Timeout  5000 //Измерения АЦП 5мс
#define Send_MessageBT_Timeout  2000000 //Таймаут отсылки сообщения по bluetooth 2с
#define Idle_Timeout  120000000 //Таймаут простоя до отключения питания 120с
#define MaximumTimeForceRestart 1800000000; //Время принудительной перезагрузки 30 минут
#define PeriodAlarmSend 1200000000; //Периодичность с которой отправляются сообщения об открытой двери 20 минут
#define MinimumPeriod  60 //Минимальный период установки измерительных циклов, сек
#define MaximumPeriodSendData  10 //Максимальное количество пропуска передачи данных




















//..........................................................................................................................
//..........................................................................................................................
//.................                        ДЕКЛАРАЦИЯ ПРОЦЕДУР И ФУНКЦИЙ                                    ................
//..........................................................................................................................
//.................                        ДЕКЛАРАЦИЯ ПРОЦЕДУР И ФУНКЦИЙ                                    ................
//..........................................................................................................................
//.................                        ДЕКЛАРАЦИЯ ПРОЦЕДУР И ФУНКЦИЙ                                    ................
//..........................................................................................................................
//.................                        ДЕКЛАРАЦИЯ ПРОЦЕДУР И ФУНКЦИЙ                                    ................
//..........................................................................................................................
//.................                        ДЕКЛАРАЦИЯ ПРОЦЕДУР И ФУНКЦИЙ                                    ................
//..........................................................................................................................
//.................                        ДЕКЛАРАЦИЯ ПРОЦЕДУР И ФУНКЦИЙ                                    ................
//..........................................................................................................................
//.................                        ДЕКЛАРАЦИЯ ПРОЦЕДУР И ФУНКЦИЙ                                    ................
//..........................................................................................................................
//.................                        ДЕКЛАРАЦИЯ ПРОЦЕДУР И ФУНКЦИЙ                                    ................
//..........................................................................................................................
//.................                        ДЕКЛАРАЦИЯ ПРОЦЕДУР И ФУНКЦИЙ                                    ................
//..........................................................................................................................
//..........................................................................................................................


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////      Функции для работы с постоянной памятью устройства     ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Сектора flash памяти
//0. 0x08000000-0x08003FFF (16 кБ) Исполняемая программа. Сектор №0-№4 (128 кБ)
//1. 0x08004000-0x08007FFF (16 кБ) Исполняемая программа. Сектор №0-№4 (128 кБ)
//2. 0x08008000-0x0800BFFF (16 кБ) Исполняемая программа. Сектор №0-№4 (128 кБ)
//3. 0x0800C000-0x0800FFFF (16 кБ) Исполняемая программа. Сектор №0-№4 (128 кБ)
//4. 0x08010000-0x0801FFFF (64 кБ) Исполняемая программа. Сектор №0-№4 (128 кБ)
//5. 0x08020000-0x0803FFFF (128 кБ) Буфер исполняемой программы. Сектор №5 (128 кБ)
//6. 0x08040000-0x0805FFFF (128 кБ) Параметры устройства. Сектор №6 (128 кБ)
//7. 0x08060000-0x0807FFFF (128 кБ) Буфер параметров устройства. Сектор №7 (128 кБ)
//8. 0x08080000-0x0809FFFF (128 кБ) Измерения. Сектор №8 (128 кБ)
//9. 0x080A0000-0x080BFFFF (128 кБ) Измерения. Сектор №9 (128 кБ)
//10. 0x080C0000-0x080DFFFF (128 кБ) Измерения. Сектор №10 (128 кБ)
//11. 0x080E0000-0x080FFFFF (128 кБ) Внутренний программатор. Сектор №11 (128 кБ)

#define ProgrammatorAddress 0x080E0000 //Адрес области паяти в которой находится внутренний программатор
#define SectorProgrammBuf FLASH_Sector_5 //Сектор Flash памяти в которую копируется прошивка устройства
#define ProgrammAddressBuf 0x08020000 //Адрес области flash в которую копируется прошивка устройства
#define ParametrAddress 0x08040000 //Адрес области flash в которой хранятся параметры устройства
#define SectorParametr FLASH_Sector_6 //Сектор Flash памяти в которой хранятся параметры устройства
#define ParametrAddressBuf 0x08060000 //Адрес области flash в которую записываются параметры устройства
#define SectorParametrBuf FLASH_Sector_7 //Сектор Flash памяти в которую записываются параметры устройства
#define ParametrFlagWriteAddress 0x0805FFFC //Адрес флага корректности записи таблицы параметров

#define SectorMeas1 FLASH_Sector_8 //Сектор Flash памяти в которую записываются результаты измерений №1
#define Meas1Address 0x08080000 //Адрес результатов измерений №1
#define SectorMeas2 FLASH_Sector_9 //Сектор Flash памяти в которую записываются результаты измерений №2
#define Meas2Address 0x080A0000 //Адрес результатов измерений №2
#define SectorMeas3 FLASH_Sector_10 //Сектор Flash памяти в которую записываются результаты измерений №3
#define Meas3Address 0x080C0000 //Адрес результатов измерений №3

#define GPRS_Settings_String_Address 0x40024000 //(256 байт) Адрес строки инициализации GPRS АРN
#define GPRS_UserPass_String_Address 0x40024100 //(256 байт) Адрес строки логина и пароля GPRS
#define FTP_Settings_String_Address 0x40024200 //(256 байт) Адрес строки настроек FTP
#define FTP_File_String_Address 0x40024300 //(256 байт) Адрес строки названия имени файла
#define ChID_List_Address 0x40024400 //(256 байт) Адрес списка каналов
#define SMS_Number_String_Address 0x40024500 //(13 байт) Адрес строки [12] номера телефона, на который будут приходить отчеты
#define Cycle_Period_Address 0x40024510 //Адрес периода измерений
#define Period_Send_Data_Address 0x40024520 //Адрес количества пропуска циклов передачи данных
#define Current_Period_Send_Data_Address 0x40024530 //Адрес текущего количества пропуска циклов передачи данных
#define FTP_File_Append_Address 0x40024540 //Адрес флага "Дописывать в конец файла"
#define SMS_Code_String_Address 0x40024550 //Адрес кода доступа по SMS

//Параметры устройства
unsigned char DeviceType=22;
unsigned char DeviceAddress=255; //Адрес устройства по умолчанию
unsigned int DeviceSerial=0;//Серийный номер устройства
unsigned int ServiceCode=0; //Код сервисного режима (141592 - прошивка программы, 832735 - калибровка, 793238 - измениеие настроек)
unsigned int CyclePeriod=3600; //Период измерительного цикла (секунды)
unsigned int CycleStart=0; //Начальное время измерения, секунды от начала суток
unsigned int CycleSendDataPeriod=0; //Количество пропусков измерительных циклов до передачи данных, шт.
unsigned int CurrentCycleSendDataPeriod=0; //Текущее количество пропусков измерительных циклов до передачи данных, шт.

//Одна запись {YY,MM,DD,Hour,  min,sec,Temp1,Temp2,  Val1,Val2,Val3,Val4,  Var1,Var2,Var3,Var4}
typedef struct //Структура, включающая в себя результаты измерения, хранящиеся в памяти устройства
	{
	unsigned char YY; //Годы
	unsigned char MM; //Месяцы
	unsigned char DD; //Дни
	unsigned char hour; //Часы
	unsigned char min; //Минуты
	unsigned char sec; //Секунды
	float Temperature; //Температура
	float Value; //Измеренное значение
	float Variation; //Вариация измеренного значения
	unsigned int ChID; //ИД канала
	unsigned char FlagMeas; //Флаг наличия записи
	unsigned char FlagFTP; //Флаг передачи данных на FTP
	}TMemoryResult;
volatile TMemoryResult MemoryResult[128]; //Переменная для хранения результатов измерения

unsigned char Check_Float(uint32_t Address); //Функция, осуществляет проверку числа с плавающей точкой на NAN, INF
unsigned int Calculate_CRC(uint32_t StartAddress, uint32_t Length); //Функция, осуществляющая расчет контрольной суммы области flash памяти
unsigned char Recive_Data_To_Flash(uint32_t Address, uint32_t Length, uint16_t Sector, unsigned int CRC32); //Функция, осуществляющая прием данных и запись их в flash память//Функция возвращает 0 если не совпадают контрольные суммы
void GetCycle_Settings(void); //Функция, осуществляющая чтение параметров устройства из SRAM памяти
void SetCycle_Settings(void); //Функция, осуществляющая сохранение параметров устройства в SRAM память
void Save_FTP_Settings(void);//Функция, сохраняющая настройки FTP в SRAM
void Read_FTP_Settings(void);//Функция, читающая настройки FTP из SRAM
void Save_GPRS_Settings(void);//Функция, сохраняющая настройки FTP в SRAM
void Read_GPRS_Settings(void);//Функция, читающая настройки FTP из SRAM
void Save_GPRS_User_Pass(void); //Функция, сохраняющая GPRS_User_Pass в SRAM
void Read_GPRS_User_Pass(void); //Функция, читающая GPRS_UserPass из SRAM
void Save_FTP_FileName(void); //Функция, сохраняющая имя файла FTP в SRAM
void Read_FTP_FileName(void); //Функция, читающая имя файла FTP из SRAM
void Save_SMS_Settings(void); //Функция, сохраняющая настройки управления по SMS в SRAM                      //
void Read_SMS_Settings(void); //Функция, Считывающая настройки управления по SMS из SRAM                     //
void Save_ChIDList(void); //Функция, сохраняющая ChIDList SRAM
void Read_ChIDList(void); //Функция, читающая ChIDList SRAM
void Save_Record(TMemoryResult Write_MeasResult); //Функция, сохраняющая измеренное значение во flash память
TMemoryResult Read_Record(unsigned int IndexRecord); //Функция, читающая запись из flash памяти
void ClearFTPFlags(void); //Функция, очищающая все флаги FTP
void Read_Parametr_Device(void); //Функция, осуществляющая чтение калибровочных таблиц из flash памяти
void Copy_Parametr_Buf(void); //Функция, осуществляющая копирование параметров устройства из буфера в постоянное хранилище
void Jump_To_Application(uint32_t Address); //Функция, осуществляющая переход к приложению которое копирующет прошивку устройства из одной области пямяти в другую

//Таблица для расчета CRC32
unsigned int TableCRC32[256]=
{0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,

0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,

0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,

0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////         Функции реализации задержек и таймаутов          ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Основной таймер
#define Timer_Main  TIM2
#define Timer_Main_IRQn TIM2_IRQn
#define RCC_APB1Periph_TIM_Main RCC_APB1Periph_TIM2

typedef struct //Структура, включающая в себя значения счетчиков таймаутов
	{
	unsigned int DelayMain; //Счетчик для функции задержки в основной программе
	unsigned int DelayBluetooth; //Счетчик для функции задержки в прерывании bluetooth
	signed int TimeoutGSM_Char; //Счетчик таймаута для отправки символа в GSM модуль
	signed int TimeoutGSM_AT; //Счетчик таймаута Выполнения AT команды GSM модулем
	signed int TimeoutADC; //Счетчик таймаута измерения АЦП
	signed int TimeoutSendMessageBluetooth; //Счетчик таймаута для отправки сообщения по bluetooth
	signed int TimeoutIdle; //Счетчик таймаута простоя устройства
	signed int TimeoutRS485_Char; //Счетчик таймаута для отправки символа в RS485 модуль
	signed int TimeoutRS485_Instruction; //Счетчик таймаута выполнения инструкции
	unsigned int ForceRestart; //Счетчик таймаута форсированной перезагрузки, мкс
	unsigned int AlarmSend; //Счетчик периода передачи сообщения об открытой двери, мкс
	unsigned int TimeFTPSession; //Время FTP сессии, мкс
	}TTimeouts;
volatile TTimeouts Timeouts; //Переменная для хранения счетчиков таймаутов и задержек


void Main_Timer_Init(void); //Функция инициализации основного таймера
void TIM2_IRQHandler(void); //Событие срабатывания основного таймера
void Delay_us(unsigned int Val); //Функция задержки, мкс
void Delay_us_Bluetooth(unsigned int Val); //Функция задержки в прерывании bluetooth, мкс


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////                Функции для работы с RTC               ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct //Структура, включающая в себя значения даты и времени
	{
	uint8_t WD; //День недели
	uint8_t DD; //День
	uint8_t MM; //Месяц
	uint8_t YY; //Год
	uint8_t hour; //Часы
	uint8_t min; //Минуты
	uint8_t sec; //Секунды
	}TDateTime;
volatile TDateTime DateTime; //Переменная для хранения даты и времени

typedef struct //Структура, включающая в себя значения времени измерительного цикла
	{
	uint8_t hour; //Часы
	uint8_t min; //Минуты
	uint8_t sec; //Секунды
	}TCycleTime;
volatile TCycleTime CycleTime; //Переменная для хранения времени срабатывания буддильника

typedef enum _TDeviceMode //Режим работы устройства
	{
	DeviceModeCycle=0, //Режим циклических измерений
	DeviceModeCycleFTP=1, //Режим циклических измерений c передачей данных на FTP сервер
	DeviceModeCommand=2, //Режим обработки инструкций
	}
TDeviceMode;
volatile TDeviceMode DeviceMode=DeviceModeCommand; //Режим работы устройства

void RTC__Init(void); //Функция инициализации часов реального времени
void RTC_Get_Date_Time(void); //Функция получения текущей отметки времени
void RTC_Set_Date_Time(void); //Функция установки текущей даты и времени
void RTC_Get_Alarm_Time(void); //Функция получения времени будильника
void RTC_Set_Alarm_Time(void); //Функция установки времени будильника
void RTC_Set_Next_Cycle_Time(void); //Функция установки времени срабатывания будильника
void Get_DeviceMode(void); //Функция получения режима работы устройства


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////               Функции для работы с Bluetooth          ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Вывод RESET Bluetooth модуля (Низкий уровень - модуль не активен,  высокий уровень - модуль активен)
#define BT_Reset_Port  GPIOA
#define BT_Reset_Pin  GPIO_Pin_8

//Вывод PIO_11 Bluetooth модуля (Низкий уровень - режим передачи данных,  высокий уровень - AT режим)
#define BT_AT_Port  GPIOC
#define BT_AT_Pin  GPIO_Pin_9

//Rx USART контроллера
#define BT_Rx_Port  GPIOA
#define BT_Rx_Pin  GPIO_Pin_10

//Tx USART контроллера
#define BT_Tx_Port  GPIOA
#define BT_Tx_Pin  GPIO_Pin_9

//USART Bluetooth модуля
#define RCC_APB2Periph_USART_BT RCC_APB2Periph_USART1
#define USART_BT USART1

//Таймер Bluetooth
#define Timer_Bluetooth  TIM4
#define Timer_Bluetooth_IRQn TIM4_IRQn
#define RCC_APB1Periph_TIM_Bluetooth RCC_APB1Periph_TIM4

typedef struct //Структура, включающая в себя буфер на 2048 символов
	{
	unsigned char Buffer[2048]; //Буфер
	unsigned int Buffer_Len; //Размер находящегося в буфере сообщения
	unsigned int Buffer_Pos; //Позиция курсора, перемещающегося при работе с буфером
	unsigned char Start_Collect_Flag; //Флаг сборки сообщения (1 - идет процесс выборки сообщения, 0 - ожидается маркер начала сообщения)
	unsigned char Previous_Byte; //Предыдущий прочитанный из USART байт
	unsigned char Current_Byte; //Текущий прочитанный из USART байт
	}TMessageSelectionBluetooth;
TMessageSelectionBluetooth MessageSelectionBluetooth; //Переменная, включающая в себя буфер на 2048 символов

typedef struct //Структура, используемая для интерпретации сообщений
	{
	//Определение границ блоков в сообщении
	unsigned int TypePosStart; //Начало блока "тип сообщения"
	unsigned int TypePosEnd; //Конец блока "тип сообщения"
	unsigned int AddressPosStart; //Начало блока "адрес"
	unsigned int AddressPosEnd; //Конец блока "адрес"
	unsigned int TransactionPosStart; //Начало блока "номер транзакции"
	unsigned int TransactionPosEnd; //Конец блока "номер транзакции"
	unsigned int InstructionPosStart; //Начало блока "инструкция"
	unsigned int InstructionPosEnd; //Конец блока "инструкция"
	unsigned int DataPosStart; //Начало блока "данные"
	unsigned int DataPosEnd; //Конец блока "данные"
	unsigned char CorrectMessageFlag; //Флаг корректности принятого сообщения (1 - корректное сообщение, 0 - некорректное сообщение)
	unsigned char Address; //Адрес  устройства, которому предназначено сообщение
	}TMessageInterpretationBluetooth;
TMessageInterpretationBluetooth MessageInterpretationBluetooth; //Переменная, используемая для интерпретации сообщений

void Bluetooth_Init(void); //Функция инициализации bluetooth модуля
void Bluetooth_Send_AT(const unsigned char* str); //Функция посылки bluetooth модулю AT команды
void Bluetooth_Send_Message(void); //Функция отсылки сообщения по bluetooth
void Bluetooth_Interpretation_Message(void); //Функция производит интерпретацию сообщений принятых по Bluetooth
void TIM4_IRQHandler(void); //Функция, вызываемая по прерыванию таймера Bluetooth
void Bluetooth_Unsigned_Int_To_Char_Buf(unsigned int Val, unsigned int StartPos, unsigned int EndPos); //Функция производит преобразование целого положительного числа в строку и помещает ее в буфер Bluetooth
unsigned int Bluetooth_Char_To_Unsigned_Int_Buf(unsigned int StartPos, unsigned int EndPos); //Функция производит преобразование части строки, находящейся в буфере Bluetooth, в целое число без знака
signed int Bluetooth_Char_To_Signed_Int_Buf(unsigned int StartPos, unsigned int EndPos); //Функция производит преобразование части строки, находящейся в буфере Bluetooth, в целое число со знаком
void Bluetooth_HexChar_Buf_To_GPRS_Settings(unsigned int StartPos, unsigned int EndPos); // Функция производит преобразование части строки в формате HEX, находящейся в буфере Bluetooth, в символьную строку GSM.GPRS_Settings /////
void Bluetooth_HexChar_Buf_To_FTP_Settings(unsigned int StartPos, unsigned int EndPos); //Функция производит преобразование части строки в формате HEX, находящейся в буфере Bluetooth, в символьную строку GSM.FTP_Settings /////
void Bluetooth_HexChar_Buf_To_GPRS_User_Pass(unsigned int StartPos, unsigned int EndPos); //Функция производит преобразование части строки в формате HEX, находящейся в буфере Bluetooth, в символьную строку GSM.GPRS_User_Pass ////
void Bluetooth_HexChar_Buf_To_FTP_FileName(unsigned int StartPos, unsigned int EndPos); // Функция производит преобразование части строки в формате HEX, находящейся в буфере Bluetooth, в символьную строку GSM.GPRS_User_Pass ////
void Bluetooth_Send_ErrorData(void);//Функция отсылает ответ ErrorData
void Bluetooth_Send_ErrorCh(void);//Функция отсылает ответ ErrorCh
void Bluetooth_Send_ErrorSensor(void);//Функция отсылает ответ ErrorSensor
void Bluetooth_Send_End(void);//Функция отсылает ответ End
void Bluetooth_Send_CRCError(void);//Функция отсылает ответ CRCError
void Bluetooth_Send_CRC_Ok(void);//Функция отсылает ответ CRC_Ok
void Bluetooth_Execution_GetType(void);//Функция производит выполнение инструкции GetType
void Bluetooth_Execution_GetSerial(void);//Функция производит выполнение инструкции GetSerial
void Bluetooth_Execution_GetProgVersion(void);//Функция производит выполнение инструкции GetProgVersion
void Bluetooth_Execution_UploadProgramm(void);//Функция производит выполнение инструкции UploadProgramm
void Bluetooth_Execution_UploadSettings(void); //Функция производит выполнение инструкции UploadSettings
void Bluetooth_Execution_DownloadSettings(void); //Функция производит выполнение инструкции DownloadSettings
void Bluetooth_Execution_SetServiceMode(void); //Функция производит выполнение инструкции SetServiceMode
void Bluetooth_Execution_SetCycleSettings(void); //Функция производит выполнение инструкции SetCycleSettings
void Bluetooth_Execution_GetCycleSettings(void); //Функция производит выполнение инструкции GetCycleSettings
void Bluetooth_Execution_SetClock(void); //Функция производит выполнение инструкции SetClock
void Bluetooth_Execution_GetClock(void); //Функция производит выполнение инструкции GetClock
void Bluetooth_Execution_SetFTPSettings(void); //Функция производит выполнение инструкции SetFTPSetting
void Bluetooth_Execution_GetFTPSettings(void); //Функция производит выполнение инструкции GetFTPSetting
void Bluetooth_Execution_SetGPRSSettings(void); //Функция производит выполнение инструкции SetGPRSSetting
void Bluetooth_Execution_GetGPRSSettings(void); //Функция производит выполнение инструкции GetGPRSSetting
void Bluetooth_Execution_SetGPRSUserPass(void); //Функция производит выполнение инструкции SetGPRSUserPass
void Bluetooth_Execution_GetGPRSUserPass(void); //Функция производит выполнение инструкции GetGPRSUserPass
void Bluetooth_Execution_SetFTPFileName(void); //Функция производит выполнение инструкции SetFTPFileName
void Bluetooth_Execution_GetFTPFileName(void); //Функция производит выполнение инструкции GetFTPFileName
void Bluetooth_Execution_SetSMSSettings(void); //Функция производит выполнение инструкции SetSMSSettings
void Bluetooth_Execution_GetSMSSettings(void); //Функция производит выполнение инструкции GetSMSSettings
void Bluetooth_Execution_SetChIDList(void); //Функция производит выполнение инструкции SetChIDList
void Bluetooth_Execution_GetChIDList(void); //Функция производит выполнение инструкции GetChIDList
void Bluetooth_Execution_DownloadData(void); //Функция производит выполнение инструкции DownloadData
void Bluetooth_Execution_Test(void); //Функция производит выполнение инструкции Test
void SetNameBlooetooth(void); //Функция производит установку имени bluetooth модуля

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////         Функции системы питания устройства            ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Вывод включения питания устройства (0-устройство выключено, 1-устройство включено)
#define Power_Port GPIOA
#define Power_Pin GPIO_Pin_15

//Вывод питания Bluetooth и GSM модулей (0-модули выключены, 1-модули включены)
#define Power_BT_GSM_Port GPIOB
#define Power_BT_GSM_Pin GPIO_Pin_0

//Вывод питания устройств на линии RS485 (0-питание линии отключено, 1-питание линии включено)
#define Power_RS485_Port GPIOC
#define Power_RS485_Pin GPIO_Pin_4

//Вывод светодиода
#define Led_Port GPIOC
#define Led_Pin GPIO_Pin_8

//Таймер светодиода
#define Timer_Led  TIM3
#define Timer_Led_IRQn TIM3_IRQn
#define RCC_APB1Periph_TIM_Led RCC_APB1Periph_TIM3

void Power_Init(void);//Функция производит инициализацию системы питания
void Power_ON_BT_GSM(void); //       Функция производит плавное включение питания GSM и Bluetooth модулей
void TIM3_IRQHandler(void);//Событие срабатывания таймера светодиода
void Start_Led(unsigned int Period);//Функция запуска таймера светодиода
void Stop_Led();//Функция остановки таймера светодиода


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////                  Функции для работы с RS485                             ///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Вывод RE_DE RS485 модуля (Низкий уровень - прием данных,  высокий уровень - передача данных)
#define RS485_RE_DE_Port GPIOA
#define RS485_RE_DE_Pin GPIO_Pin_4

//Вывод Rx (USART контроллера)
#define RS485_Rx_Port  GPIOA
#define RS485_Rx_Pin  GPIO_Pin_3
#define RS485_Rx_GPIO_PinSource  GPIO_PinSource3

//Вывод Tx (USART контроллера)
#define RS485_Tx_Port  GPIOA
#define RS485_Tx_Pin  GPIO_Pin_2
#define RS485_Tx_GPIO_PinSource  GPIO_PinSource2

//USART RS485 модуля
#define RCC_APB1Periph_USART_RS485 RCC_APB1Periph_USART2
#define USART_RS485 USART2
#define GPIO_AF_USART_RS485 GPIO_AF_USART2

unsigned int ChIDList[64]; //Список сенсоров

typedef struct //Структура, включающая в себя параметры работы обмена данными с датчиками по RS485
	{
	unsigned char Buffer[2048]; //Приемный буфер USART
	unsigned int Buffer_Pos; //Позиция указателя в буфере
	unsigned int  Buffer_Len; //Размер данных в буфере
	unsigned char Transaction; //Номертранзакции
	unsigned char Attempt; //Количество попыток выполнения команды
	unsigned char Message_Complete_flag; //Флаг окончания сборки сообщения
	unsigned char Start_Collect_Flag; //Флаг начала процесса сборки сообщения
	unsigned char Previous_Byte;
	unsigned char Current_Byte;
	unsigned char  CorrectResultFlag;
	float Val;
	float Var;
	float Temp;
	}TRS485;
volatile TRS485 RS485; //Переменная для хранения параметров RS485 модуля
void RS485_Init(void);//Функция инициализации RS85 модуля
void RS485_Send_Char(unsigned char Byte);//Передача символа в RS485 модуль


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////           Функции для работы с GSM модулем            ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Rx (контроллера) GSM
#define GSM_Rx_Port  GPIOC
#define GSM_Rx_Pin  GPIO_Pin_7
#define GSM_Rx_PinSource GPIO_PinSource7

//Tx (контроллера) GSM
#define GSM_Tx_Port  GPIOC
#define GSM_Tx_Pin  GPIO_Pin_6
#define GSM_Tx_PinSource GPIO_PinSource6

//USART GSM модуля
#define RCC_APB2Periph_USART_GSM RCC_APB2Periph_USART6
#define USART_GSM USART6
#define GPIO_AF_USART_GSM GPIO_AF_USART6

//Таймер приемника данных от GSM модуля
#define Timer_ReciveGSM  TIM5
#define Timer_ReciveGSM_IRQn TIM5_IRQn
#define RCC_APB1Periph_TIM_ReciveGSM RCC_APB1Periph_TIM5

typedef struct //Структура, включающая в себя параметры работы GSM модуля
	{
	unsigned char UsartRxNE; //Флаг наличия байта в буфере USART GSM 0 - буфер пуст
	unsigned char UsartInByte; //Принятый в USART байт
	unsigned char Attempt; //Количество попыток выполнения команды
	unsigned int ATErrorFlag; //Ошибка выполнения AT команды (1-ошибка выполнения команды)
	char Buffer[256]; //Выходной буфер для отправки данных GSM модулю
	char OutSMS[256]; //Буфер для хранения выходного SMS сообщения
	char SMSNumber[13]; //Номер, на который приходят SMS с результатами выполнения инструкции
	unsigned int SMSCode; //Код доступа к управлению устройством
	char IMEI[16]; //IMEI модуля
	unsigned char REG_MODE; //Режим регистрации в сети оператора
	unsigned char REG_STAT; //Статус регистрации в сети
	unsigned char SQ_RSSI; //Уровень сигнала
	unsigned char SQ_BER; //Процент ошибок
	char Name_Operator[16]; //Имя оператора
	unsigned char Connected; //Флаг успешной регистрации в сети 0-нет подключения
	char GPRS_Settings[256]; //
	char GPRS_User_Pass[256]; //
	unsigned char GPRS_Connected; //Флаг успешной регистрации в сети 0-нет подключения
	char FTP_Settings[256]; //Настройки FTP соединеня -  "server:port","username","password",mode
    char FTP_File_Name[256]; //Имя файла FTP
	unsigned int FTP_Append; //0-перезапись файла, 1-добавление данных в конец файла
	unsigned char FTP_Connected; //Флаг успешного подключения к FTP серверу 0-нет подключения
	char FTP_Out_String[256]; //Строка для записи в файл
	}TGSM;
TGSM GSM; //Переменная для хранения параметров GSM модуля


unsigned char FTPInProgress; //Флаг процесса передачи данных (0x00 передачаданных не производится)


void GSM_Init(void); //Функция инициализации GSM модуля
void TIM5_IRQHandler(void); //Функция, вызываемая по прерыванию таймера приемника байта от GSM модуля
unsigned char GSM_Recive_Char(void); //Прием символа от GSM модуля
void GSM_Send_Char(unsigned char Byte); //Передача символа в GSM модуль
void GSM_Send_String(void); //Передача строки из GSM.Buffer в GSM модуль
void GSM_Send_AT(void); //Передача AT команды из GSM.Buffer в GSM модуль
void GSM_Get_Signal_Quality(unsigned int Timeout_ms); //Функция получения уровня сигнала GSM
void GSM_Get_Network_Registration_Status(unsigned int Timeout_ms); //Функция получения статуса регистрации в сети GSM
void GSM_Get_Name_Operator(unsigned int Timeout_ms); //Функция получения имени оператора GSM
void GSM_Get_IMEI(unsigned int Timeout_ms); //Функция получения IMEI GSM модуля
void GSM_GPRS_Close(unsigned int Timeout_ms); //Функция закрытия сессии GPRS
void GSM_GPRS_Set_PDP(unsigned int Timeout_ms); //Функция задания PDP контекста
void GSM_GPRS_Open(unsigned int Timeout_ms); //Функция открытия сессии GPRS
void GSM_FTP_TO(unsigned int Timeout_ms); //Функция установки таймаута ожидания ответа от FTP сервера
void GSM_DNS_Get_IP(unsigned int Timeout_ms); //Функция IP адреса сервера по DNS
void GSM_FTP_Open(unsigned int Timeout_ms); //Функция открытия соединения с FTP сервером
void GSM_FTP_Type(unsigned int Timeout_ms); //Функция открытия установки типа передачи по FTP
void GSM_FTP_Open_File(unsigned int Timeout_ms); //Функция открытия файла FTP
void GSM_FTP_Close_File(unsigned int Timeout_ms); //Функция закрытия файла FTP
void GSM_FTP_Send_string(unsigned int Timeout_ms); //Функция открытия изменения рабочего каталога FTP
void GSM_Connect(void); //Функция подключения к сети GSM
void GSM_GPRS_Connect(void); //Функция подключения к GPRS
void GSM_FTP_Connect(void); //Функция подключения к FTP серверу
void GSM_Send_Data_FTP(void); //Функция передачи данных на FTP сервер
void GSM_Send_SMS(unsigned int Timeout_ms); //Функция передачи SMS сообщения из
void GSM_Send_Alarm_SMS(void); //Функция передачи SMS сообщения в случае открытия ящика
void GSM_Send_Alarm_FTP(void); //Функция передачи данных на FTP сервер в случае открытия ящика
void GSM_SMS_Execution_SetCycleSettings(void); //Функция производит выполнение инструкции SetCycleSettings из СМС
void GSM_SMS_Execution_SetSMSSettings(void); //Функция производит выполнение инструкции SetCycleSettings из СМС
void GSM_SMS_HexChar_Buf_To_FTP_Settings(unsigned int StartPos, unsigned int EndPos); // Функция производит преобразование части строки в формате HEX, находящейся в буфере SMS, в символьную строку GSM.FTP_Settings
void GSM_SMS_Execution_SetFTPSettings(void); //Функция производит выполнение инструкции SetFTPSettings из СМС
void GSM_SMS_HexChar_Buf_To_FTP_FileName(unsigned int StartPos, unsigned int EndPos); // Функция производит преобразование части строки в формате HEX, находящейся в буфере SMS, в символьную строку GSM.GPRS_User_Pass
unsigned int GSM_SMS_Char_To_Unsigned_Int_Buf(unsigned int StartPos, unsigned int EndPos); //Функция производит преобразование части строки, находящейся в буфере SMS, в целое число без знака
void GSM_SMS_Execution_SetFTPFileName(void); //Функция производит выполнение инструкции SetFTPFileName  из СМС
void GSM_Read_ALL_SMS(unsigned int Timeout_ms); //Чтение всех SMS и выполнение команд
void GSM_Delete_All_SMS(unsigned int Timeout_ms); //Удаление всех SMS


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////          Функции системы измерения напряжений             ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Вход АЦП - уровень заряда аккумулятора
#define Bat1_Level_Port  GPIOA
#define Bat1_Level_Pin  GPIO_Pin_6

//Вход АЦП - уровень заряда аккумулятора
#define Bat2_Level_Port  GPIOA
#define Bat2_Level_Pin  GPIO_Pin_7

//Вход АЦП - термистор (Температура устройства)
#define Termistor_Port  GPIOC
#define Termistor_Pin  GPIO_Pin_5

//Вход АЦП - Потенциометры (Питание)
#define Power_Potenciometr_Port  GPIOA
#define Power_Potenciometr_Pin  GPIO_Pin_5

//Вход АЦП - Потенциометр (Канал№1)
#define CH1_Port  GPIOA
#define CH1_Pin  GPIO_Pin_0

//Вход АЦП - Потенциометр (Канал№2)
#define CH2_Port  GPIOC
#define CH2_Pin  GPIO_Pin_3

//Вход АЦП - Потенциометр (Канал№3)
#define CH3_Port  GPIOC
#define CH3_Pin  GPIO_Pin_2

//Вход АЦП - Потенциометр (Канал№4)
#define CH4_Port  GPIOC
#define CH4_Pin  GPIO_Pin_1

//Вход АЦП - Потенциометр (Канал№5)
#define CH5_Port  GPIOC
#define CH5_Pin  GPIO_Pin_0

//Вход АЦП - Потенциометр (Канал№6)
#define CH6_Port  GPIOA
#define CH6_Pin  GPIO_Pin_1

//АЦП измерительной системы
#define ADC_MeasSystem ADC1
#define RCC_APB2Periph_ADC_MeasSystem RCC_APB2Periph_ADC1

typedef struct //Структура, включающая в себя результат измерения
	{
	unsigned int TermistorRes; //Сопротивление термистора при нуле градусах цельсия, Ом
	unsigned int TermistorConstRes; //Сопротивление резистора в делители термистора, Ом
	unsigned int PowerConstRes; //Сопротивление резистора в делители термистора, Ом
	float Bat1Level; //Уровень заряда аккумулятора, В
	float Bat2Level; //Уровень заряда аккумулятора, В
	float Temperature; //Внутренняя температура устройства, градусы цельсия
	float CHPowerVoltage; //Результат измерения напряжения питания потенциометров, мВ
	float CHPowerRes; //Результат измерения сопротивления потенциометров, Ом
	float CH1Val; //Результат измерения напряжения на потенциометре канала №1, мВ/В
	float CH2Val; //Результат измерения напряжения на потенциометре канала №1, мВ/В
	float CH3Val; //Результат измерения напряжения на потенциометре канала №1, мВ/В
	float CH4Val; //Результат измерения напряжения на потенциометре канала №1, мВ/В
	float CH5Val; //Результат измерения напряжения на потенциометре канала №1, мВ/В
	float CH6Val; //Результат измерения напряжения на потенциометре канала №1, мВ/В
	}TMeasSystem;
volatile TMeasSystem MeasSystem; //Переменная для результатов измерения напряжений

void Meas_System_Init(void); //Функция инициализации системы измерения напряжений
void Meas_System_GetValue(void); //Функция измерения напряжения на потенциометрах
void MeasCycle(void); //Функция измерения цикла




















//..........................................................................................................................
//..........................................................................................................................
//.................                            ПРОЦЕДУРЫ И ФУНКЦИИ                                          ................
//..........................................................................................................................
//.................                            ПРОЦЕДУРЫ И ФУНКЦИИ                                          ................
//..........................................................................................................................
//.................                            ПРОЦЕДУРЫ И ФУНКЦИИ                                          ................
//..........................................................................................................................
//.................                            ПРОЦЕДУРЫ И ФУНКЦИИ                                          ................
//..........................................................................................................................
//.................                            ПРОЦЕДУРЫ И ФУНКЦИИ                                          ................
//..........................................................................................................................
//.................                            ПРОЦЕДУРЫ И ФУНКЦИИ                                          ................
//..........................................................................................................................
//.................                            ПРОЦЕДУРЫ И ФУНКЦИИ                                          ................
//..........................................................................................................................
//.................                            ПРОЦЕДУРЫ И ФУНКЦИИ                                          ................
//..........................................................................................................................
//.................                            ПРОЦЕДУРЫ И ФУНКЦИИ                                          ................
//..........................................................................................................................
//..........................................................................................................................


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////        Функции для работы с flash памятью устройства        ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////  Функция, осуществляющая расчет контрольной суммы области flash памяти  	/////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int Calculate_CRC(uint32_t StartAddress, uint32_t Length) //Length - количество 4-хбайтовых слов
	{
	unsigned int i; //Счетчик
	unsigned int CRCResult=0xffffffff;
	unsigned char DataByte;
	uint32_t Address;
	for (i = 0; i < Length*4; ++i)
		{
		Address=StartAddress+i; DataByte=*(unsigned char*)Address;
		CRCResult=(CRCResult >> 8) ^ TableCRC32[DataByte ^ (CRCResult & 0x000000ff)];
		}
	CRCResult=~CRCResult;
    return CRCResult;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                   Функция, осуществляет проверку числа с плавающей точкой на NAN, INF       //////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char Check_Float(uint32_t Address) //Возвращает значение 0x00 в случае отсутствии числового значения float
	{
	uint32_t Addr;
	unsigned char Byte0; //Байт 0 (старший)
	unsigned char Byte1; //Байт 1 (старший)
	Addr=Address+3; Byte0=*(unsigned char*)Addr;
	Addr=Address+2; Byte1=*(unsigned char*)Addr;
	if (((Byte0&0x7f)==0x7f)&((Byte1&0x80)==0x80)) //Если биты порядка равны единицам
		{return 0x00;}
	return 0xff;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////          Функция, осуществляющая прием данных и запись их в flash память       //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char Recive_Data_To_Flash(uint32_t Address, uint32_t Length, uint16_t Sector, unsigned int CRC32) //Функция возвращает 0 если не совпадают контрольные суммы
	{
	unsigned int i; //Счетчики
	unsigned char DataByte0; //0-й байт для сбоки слова
	unsigned char DataByte1; //1-й байт для сбоки слова
	unsigned char DataByte2; //2-й байт для сбоки слова
	unsigned char DataByte3; //3-й байт для сбоки слова
	unsigned int LocalCRC; //Контрольная сумма для принятых данных
	uint32_t Data4Byte; //Слово, записываемое в память
	uint32_t WriteAddress; //Адрес области в которую производится запись
		//Запись данных в flash память
	USART_ClearFlag(USART_BT, USART_FLAG_RXNE); //Очистка флага наличия байта в буфере USART
	FLASH_Unlock(); //Разрешается редактирование памяти
	//Очистка памяти
	FLASH_EraseSector(Sector,VoltageRange_3); //Очистка сектора Flash памяти в которую копируются данные
	//Запись памяти
	Bluetooth_Send_Message(); //Отсылается ответ по Bluetooth
	WriteAddress=Address; //Адрес области в которую производится запись
	for (i = 0; i < Length; ++i) //Копирование данных во flash память
		{
		//Принимается 4 байта
		//Байт0
		while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //Ожидается прибытие байта в буфер bluetooth
		DataByte0=USART_ReceiveData(USART_BT); //Забирается байт из буфера Usart
		//Байт1
		while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //Ожидается прибытие байта в буфер bluetooth
		DataByte1=USART_ReceiveData(USART_BT); //Забирается байт из буфера Usart
		//Байт2
		while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //Ожидается прибытие байта в буфер bluetooth
		DataByte2=USART_ReceiveData(USART_BT); //Забирается байт из буфера Usart
		//Байт3
		while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //Ожидается прибытие байта в буфер bluetooth
		DataByte3=USART_ReceiveData(USART_BT); //Забирается байт из буфера Usart
		//Слово из 4-х байт записывается в flash пррамять
		Data4Byte=DataByte3*0x1000000+DataByte2*0x10000+DataByte1*0x100+DataByte0*0x1;
		FLASH_ProgramWord(WriteAddress,Data4Byte);
		WriteAddress+=4;
		}
	FLASH_Lock(); //Включается защита памяти от редактирования
	//Проверка контрольной суммы
	LocalCRC=Calculate_CRC(Address, Length); //Расчет контрольной суммы
	if (LocalCRC==CRC32) //Если совпадают контрольные суммы
		{return 0xff;}
	else //Если не совпадают контрольные суммы
		{return 0x00;}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////  Функция, осуществляющая чтение параметров цикла из SRAM памяти       ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GetCycle_Settings(void)
	{
	uint32_t Address;
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	RTC_Get_Alarm_Time();
	Address=Cycle_Period_Address; CyclePeriod=*(unsigned int*)Address; //Период измерительного цикла (секунды)
	CycleStart=CycleTime.hour*3600+CycleTime.min*60+CycleTime.sec; //Начальное время измерения, секунды от начала суток
	Address=Period_Send_Data_Address; CycleSendDataPeriod=*(unsigned int*)Address; //Количество пропусков измерительных циклов до передачи данных, шт.
	Address=Current_Period_Send_Data_Address; CurrentCycleSendDataPeriod=*(unsigned int*)Address; //Количество пропусков измерительных циклов до передачи данных, шт.
	PWR_BackupAccessCmd(DISABLE); //Запрещается доступ крегистру
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////  Функция, осуществляющая сохранение параметров цикла в SRAM память       /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetCycle_Settings(void)
	{
	uint32_t Address;
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	Address=Cycle_Period_Address; *(unsigned int*)Address=CyclePeriod; //Период измерительного цикла (секунды)
	Address=Period_Send_Data_Address; *(unsigned int*)Address=CycleSendDataPeriod; //Количество пропусков измерительных циклов до передачи данных, шт.
	if (CurrentCycleSendDataPeriod>Period_Send_Data_Address) {CurrentCycleSendDataPeriod=0;};
	Address=Current_Period_Send_Data_Address; *(unsigned int*)Address=CurrentCycleSendDataPeriod; //Количество пропусков измерительных циклов до передачи данных, шт.
	//Будильник устанавливается на начальное время измерения, секунды от начала суток
	CycleTime.hour=(CycleStart/3600)%24;
	CycleTime.min=(CycleStart/60)%60;
	CycleTime.sec=(CycleStart)%60;
	RTC_Set_Alarm_Time();
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, сохраняющая настройки FTP в SRAM                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_FTP_Settings(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //Слово, записываемое в память
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 64; ++i)
		{
		DataByte0=GSM.FTP_Settings[i*4];
		DataByte1=GSM.FTP_Settings[i*4+1];
		DataByte2=GSM.FTP_Settings[i*4+2];
		DataByte3=GSM.FTP_Settings[i*4+3];
		Data4Byte=DataByte3*0x1000000+DataByte2*0x10000+DataByte1*0x100+DataByte0*0x1;
		Address=FTP_Settings_String_Address+i*4;*(uint32_t*)Address=Data4Byte;
		}
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, читающая настройки FTP из SRAM                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_FTP_Settings(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //Слово, записываемое в память
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 64; ++i)
		{
		Address=FTP_Settings_String_Address+i*4; Data4Byte=*(uint32_t*)Address; //Считывается слово
		//Слово раскладывается на компоненты
		DataByte0=Data4Byte%0x100;
		DataByte1=(Data4Byte/0x100)%0x100;
		DataByte2=(Data4Byte/0x10000)%0x100;
		DataByte3=(Data4Byte/0x1000000)%0x100;
		GSM.FTP_Settings[i*4]=DataByte0;
		GSM.FTP_Settings[i*4+1]=DataByte1;
		GSM.FTP_Settings[i*4+2]=DataByte2;
		GSM.FTP_Settings[i*4+3]=DataByte3;
		}
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, сохраняющая настройки GPRS в SRAM                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_GPRS_Settings(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //Слово, записываемое в память
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 64; ++i)
		{
		DataByte0=GSM.GPRS_Settings[i*4];
		DataByte1=GSM.GPRS_Settings[i*4+1];
		DataByte2=GSM.GPRS_Settings[i*4+2];
		DataByte3=GSM.GPRS_Settings[i*4+3];
		Data4Byte=DataByte3*0x1000000+DataByte2*0x10000+DataByte1*0x100+DataByte0*0x1;
		Address=GPRS_Settings_String_Address+i*4;*(uint32_t*)Address=Data4Byte;
		}
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, читающая настройки GPRS из SRAM                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_GPRS_Settings(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //Слово, записываемое в память
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 64; ++i)
		{
		Address=GPRS_Settings_String_Address+i*4; Data4Byte=*(uint32_t*)Address; //Считывается слово
		//Слово раскладывается на компоненты
		DataByte0=Data4Byte%0x100;
		DataByte1=(Data4Byte/0x100)%0x100;
		DataByte2=(Data4Byte/0x10000)%0x100;
		DataByte3=(Data4Byte/0x1000000)%0x100;
		GSM.GPRS_Settings[i*4]=DataByte0;
		GSM.GPRS_Settings[i*4+1]=DataByte1;
		GSM.GPRS_Settings[i*4+2]=DataByte2;
		GSM.GPRS_Settings[i*4+3]=DataByte3;
		}
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, сохраняющая GPRS_User_Pass в SRAM                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_GPRS_User_Pass(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //Слово, записываемое в память
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 64; ++i)
		{
		DataByte0=GSM.GPRS_User_Pass[i*4];
		DataByte1=GSM.GPRS_User_Pass[i*4+1];
		DataByte2=GSM.GPRS_User_Pass[i*4+2];
		DataByte3=GSM.GPRS_User_Pass[i*4+3];
		Data4Byte=DataByte3*0x1000000+DataByte2*0x10000+DataByte1*0x100+DataByte0*0x1;
		Address=GPRS_UserPass_String_Address+i*4;*(uint32_t*)Address=Data4Byte;
		}
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, читающая GPRS_UserPass из SRAM                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_GPRS_User_Pass(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //Слово, записываемое в память
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 64; ++i)
		{
		Address=GPRS_UserPass_String_Address+i*4; Data4Byte=*(uint32_t*)Address; //Считывается слово
		//Слово раскладывается на компоненты
		DataByte0=Data4Byte%0x100;
		DataByte1=(Data4Byte/0x100)%0x100;
		DataByte2=(Data4Byte/0x10000)%0x100;
		DataByte3=(Data4Byte/0x1000000)%0x100;
		GSM.GPRS_User_Pass[i*4]=DataByte0;
		GSM.GPRS_User_Pass[i*4+1]=DataByte1;
		GSM.GPRS_User_Pass[i*4+2]=DataByte2;
		GSM.GPRS_User_Pass[i*4+3]=DataByte3;
		}
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, сохраняющая имя файла FTP в SRAM                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_FTP_FileName(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //Слово, записываемое в память
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 64; ++i)
		{
		DataByte0=GSM.FTP_File_Name[i*4];
		DataByte1=GSM.FTP_File_Name[i*4+1];
		DataByte2=GSM.FTP_File_Name[i*4+2];
		DataByte3=GSM.FTP_File_Name[i*4+3];
		Data4Byte=DataByte3*0x1000000+DataByte2*0x10000+DataByte1*0x100+DataByte0*0x1;
		Address=FTP_File_String_Address+i*4;*(uint32_t*)Address=Data4Byte;
		}
	Address=FTP_File_Append_Address;*(uint32_t*)Address=GSM.FTP_Append;
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, читающая имя файла FTP из SRAM                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_FTP_FileName(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //Слово, записываемое в память
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 64; ++i)
		{
		Address=FTP_File_String_Address+i*4; Data4Byte=*(uint32_t*)Address; //Считывается слово
		//Слово раскладывается на компоненты
		DataByte0=Data4Byte%0x100;
		DataByte1=(Data4Byte/0x100)%0x100;
		DataByte2=(Data4Byte/0x10000)%0x100;
		DataByte3=(Data4Byte/0x1000000)%0x100;
		GSM.FTP_File_Name[i*4]=DataByte0;
		GSM.FTP_File_Name[i*4+1]=DataByte1;
		GSM.FTP_File_Name[i*4+2]=DataByte2;
		GSM.FTP_File_Name[i*4+3]=DataByte3;
		}
	Address=FTP_File_Append_Address; GSM.FTP_Append=*(uint32_t*)Address;
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, сохраняющая настройки управления по SMS в SRAM                      //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_SMS_Settings(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //Слово, записываемое в память
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 3; ++i)
		{
		DataByte0=GSM.SMSNumber[i*4];
		DataByte1=GSM.SMSNumber[i*4+1];
		DataByte2=GSM.SMSNumber[i*4+2];
		DataByte3=GSM.SMSNumber[i*4+3];
		Data4Byte=DataByte3*0x1000000+DataByte2*0x10000+DataByte1*0x100+DataByte0*0x1;
		Address=SMS_Number_String_Address+i*4;*(uint32_t*)Address=Data4Byte;
		}
	Address=SMS_Code_String_Address;*(uint32_t*)Address=GSM.SMSCode;
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, Считывающая настройки управления по SMS из SRAM                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_SMS_Settings(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //Слово, записываемое в память
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 3; ++i)
		{
		Address=SMS_Number_String_Address+i*4; Data4Byte=*(uint32_t*)Address; //Считывается слово
		//Слово раскладывается на компоненты
		DataByte0=Data4Byte%0x100;
		DataByte1=(Data4Byte/0x100)%0x100;
		DataByte2=(Data4Byte/0x10000)%0x100;
		DataByte3=(Data4Byte/0x1000000)%0x100;
		GSM.SMSNumber[i*4]=DataByte0;
		GSM.SMSNumber[i*4+1]=DataByte1;
		GSM.SMSNumber[i*4+2]=DataByte2;
		GSM.SMSNumber[i*4+3]=DataByte3;
		}
	Address=SMS_Code_String_Address; GSM.SMSCode=*(uint32_t*)Address;
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, сохраняющая ChIDList SRAM                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_ChIDList(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 64; ++i)
		{
		Address=ChID_List_Address+i*4;*(uint32_t*)Address=ChIDList[i];
		}
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, читающая ChIDList из SRAM                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_ChIDList(void)
	{
	unsigned char i; //Счетчик
	uint32_t Address;
	PWR_BackupAccessCmd(ENABLE); //Разрешается доступ к регистру
	PWR_BackupRegulatorCmd(ENABLE); //Включается регуляторнапряжения
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //Ожидания приведения в готовность регулятора напряжения
	for (i = 0; i < 64; ++i)
		{
		Address=ChID_List_Address+i*4;ChIDList[i]=*(uint32_t*)Address;
		}
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                               Функция, сохраняющая измеренное значение во flash память                                 //
//   Одна запись {YY,MM,DD,Hour,  min,sec,Temp1,Temp2,  Val1,Val2,Val3,Val4,  Var1,Var2,Var3,Var4, ChId1,ChId2,ChId3,ChId4}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_Record(TMemoryResult Write_MeasResult)
	{
	unsigned int i; //Счетчик
	uint32_t Data4Byte; //Слово, записываемое в память
	uint32_t RecordAddress; //Адрес записи
	uint32_t Address;
	uint32_t SectorAddress; //Адрес текущегно сектора
	signed int TempValue=0; //Временная величина
	unsigned char Bytes0; //Байт0
	unsigned char Bytes1; //Байт1
	unsigned char Bytes2; //Байт2
	unsigned char Bytes3; //Байт3
	unsigned char ReadByte; //Читаемый байт
	FLASH_Unlock(); //Разрешается редактирование памяти
	//////////////////////////////////////////////////////////////////////
			//Поиск пустой ячейки
	RecordAddress=0x00;
	//Поиск в секторе №8
	SectorAddress=Meas1Address;
	i=0x00;
	while ((RecordAddress==0)&(i<0x1000))
		{
		Address=SectorAddress+i*0x20+0x18; ReadByte=*(unsigned char*)Address; //Читается флаг наличия записи
		if (ReadByte==0xff) {RecordAddress=SectorAddress+i*0x20;}
		i++;
		}
	if ((i==0x1000)&(RecordAddress!=0)) //Если необходимо очистить следующий сектор
		{
		FLASH_EraseSector(SectorMeas2,VoltageRange_3); //Очистка следующего сектора Flash памяти
		}
	//Поиск в секторе №9
	if (RecordAddress==0) //Если в предыдущем секторе не найдено пустых ячеек
		{
		SectorAddress=Meas2Address;
		i=0x00;
		while ((RecordAddress==0)&(i<0x1000))
			{
			Address=SectorAddress+i*0x20+0x18; ReadByte=*(unsigned char*)Address; //Читается флаг наличия записи
			if (ReadByte==0xff) {RecordAddress=SectorAddress+i*0x20;}
			i++;
			}
		if ((i==0x1000)&(RecordAddress!=0)) //Если необходимо очистить следующий сектор
			{
			FLASH_EraseSector(SectorMeas3,VoltageRange_3); //Очистка следующего сектора Flash памяти
			}
		}
	//Поиск в секторе №10
	if (RecordAddress==0) //Если в предыдущем секторе не найдено пустых ячеек
		{
		SectorAddress=Meas3Address;
		i=0x00;
		while ((RecordAddress==0)&(i<0x1000))
			{
			Address=SectorAddress+i*0x20+0x18; ReadByte=*(unsigned char*)Address; //Читается флаг наличия записи
			if (ReadByte==0xff) {RecordAddress=SectorAddress+i*0x20;}
			i++;
			}
		if ((i==0x1000)&(RecordAddress!=0)) //Если необходимо очистить следующий сектор
			{
			FLASH_EraseSector(SectorMeas1,VoltageRange_3); //Очистка следующего сектора Flash памяти
			}
		}
	//Если не найдено пустых ячеек
	if (RecordAddress==0)
		{
		if ((i==0x1000)&(RecordAddress!=0)) //Если необходимо очистить следующий сектор
			{
			FLASH_EraseSector(SectorMeas1,VoltageRange_3); //Очистка следующего сектора Flash памяти
			}
		RecordAddress=Meas1Address;
		}
	//////////////////////////////////////////////////////////////////////
					//Запись данных
	//Одна запись {YY,MM,DD,  Hour,min,sec,Temp1,Temp2,  Val1,Val2,Val3,Val4,  Var1,Var2,Var3,Var4, ChId1,ChId2,ChId3,ChId4}
	//Сохранение первого слова
	Bytes0=Write_MeasResult.YY;
	Bytes1=Write_MeasResult.MM;
	Bytes2=Write_MeasResult.DD;
	Bytes3=0;
	Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
	Address=RecordAddress+0x00; FLASH_ProgramWord(Address,Data4Byte);
	//Сохранение второго слова
	Bytes0=Write_MeasResult.hour;
	Bytes1=Write_MeasResult.min;
	Bytes2=Write_MeasResult.sec;
	Bytes3=0;
	Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
	Address=RecordAddress+0x04; FLASH_ProgramWord(Address,Data4Byte);
	//Сохранение третьего слова CHID
	Data4Byte=(uint32_t) Write_MeasResult.ChID;
	Address=RecordAddress+0x08; FLASH_ProgramWord(Address,Data4Byte);
	//Сохранение четвертого слова Value
	TempValue=(float) Write_MeasResult.Value*100000;
	TempValue=(signed int) TempValue+1000000000;
	Data4Byte=(uint32_t) TempValue;
	Address=RecordAddress+0x0C; FLASH_ProgramWord(Address,Data4Byte);
	//Сохранение пятого слова Variation
	TempValue=(float) Write_MeasResult.Variation*100000;
	TempValue=(signed int) TempValue+1000000000;
	Data4Byte=(uint32_t) TempValue;
	Address=RecordAddress+0x10; FLASH_ProgramWord(Address,Data4Byte);
	//Сохранение шестого слова Температура
	TempValue=(float) Write_MeasResult.Temperature*100000;
	TempValue=(signed int) TempValue+1000000000;
	Data4Byte=(uint32_t) TempValue;
	Address=RecordAddress+0x14; FLASH_ProgramWord(Address,Data4Byte);
	//Сохранение седьмого слова Флаг наличия записи
	Bytes0=0x00;
	Bytes1=0x00;
	Bytes2=0x00;
	Bytes3=0x00;
	Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
	Address=RecordAddress+0x18; FLASH_ProgramWord(Address,Data4Byte);
	//Сохранение восьмого слова Флаг отправки на FTP
	Bytes0=0xff;
	Bytes1=0xff;
	Bytes2=0xff;
	Bytes3=0xff;
	Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
	Address=RecordAddress+0x1C; FLASH_ProgramWord(Address,Data4Byte);
	//////////////////////////////////////////////////////////////////////
	FLASH_Lock(); //Включается защита памяти от редактирования
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           Функция, читающая запись из flash памяти                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TMemoryResult Read_Record(unsigned int IndexRecord)
	{
	TMemoryResult TempResult; //Временная переменная для хранения результатов
	TempResult.FlagFTP=0xff;
	TempResult.FlagMeas=0xff;
	uint32_t RecordAddress; //Адрес записи
	uint32_t Address;
	uint32_t Data4Byte; //Слово, читаемое из памяти
	signed int TempValue=0; //Временная величина
	RecordAddress=Meas1Address+0x20*IndexRecord;
	//Одна запись {YY,MM,DD,  Hour,min,sec,Temp1,Temp2,  Val1,Val2,Val3,Val4,  Var1,Var2,Var3,Var4, ChId1,ChId2,ChId3,ChId4}
	//Чтение первого слова
	Address=RecordAddress+0x00; Data4Byte=*(uint32_t*)Address;
	TempResult.YY=(Data4Byte/0x1000000)%0x100;
	TempResult.MM=(Data4Byte/0x10000)%0x100;
	TempResult.DD=(Data4Byte/0x100)%0x100;
	//Чтение второго слова
	Address=RecordAddress+0x04; Data4Byte=*(uint32_t*)Address;
	TempResult.hour=(Data4Byte/0x1000000)%0x100;
	TempResult.min=(Data4Byte/0x10000)%0x100;
	TempResult.sec=(Data4Byte/0x100)%0x100;
	//Чтение третьего слова CHID
	Address=RecordAddress+0x08; Data4Byte=*(uint32_t*)Address;
	TempResult.ChID=(uint32_t) Data4Byte;
	//Чтение четвертого слова Value
	Address=RecordAddress+0x0C; Data4Byte=*(uint32_t*)Address;
	TempValue=(signed int) Data4Byte;
	TempValue=(signed int) TempValue-1000000000;
	TempResult.Value=(float) TempValue/100000;
	//Чтение пятого слова Variation
	Address=RecordAddress+0x10; Data4Byte=*(uint32_t*)Address;
	TempValue=(signed int) Data4Byte;
	TempValue=(signed int) TempValue-1000000000;
	TempResult.Variation=(float) TempValue/100000;
	//Чтение шестого слова Температура
	Address=RecordAddress+0x14; Data4Byte=*(uint32_t*)Address;
	TempValue=(signed int) Data4Byte;
	TempValue=(signed int) TempValue-1000000000;
	TempResult.Temperature=(float) TempValue/100000;
	//Чтение седьмого слова Флаг наличия записи
	Address=RecordAddress+0x18; Data4Byte=*(uint32_t*)Address;
	TempResult.FlagMeas=Data4Byte%0x100;
	//Чтение восьмого слова Флаг отправки на FTP
	Address=RecordAddress+0x1c; Data4Byte=*(uint32_t*)Address;
	TempResult.FlagFTP=Data4Byte%0x100;
	return TempResult;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          Функция, очищающая все флаги FTP                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ClearFTPFlags(void)
	{
	unsigned int i; //Счетчик
	uint32_t Data4Byte; //Слово, записываемое в память
	uint32_t RecordAddress; //Адрес записи
	uint32_t Address;
	uint32_t SectorAddress; //Адрес текущегно сектора
	unsigned char Bytes0; //Байт0
	unsigned char Bytes1; //Байт1
	unsigned char Bytes2; //Байт2
	unsigned char Bytes3; //Байт3
	unsigned char ReadByte; //Читаемый байт
	FLASH_Unlock(); //Разрешается редактирование памяти
	//////////////////////////////////////////////////////////////////////
	//Поиск пустой ячейки
	RecordAddress=0x00;
	//Поиск в секторе №8
	SectorAddress=Meas1Address;
	for (i = 0x00; i < 0x1000; ++i)
		{
		RecordAddress=SectorAddress+i*0x20;
		Address=RecordAddress+0x18; ReadByte=*(unsigned char*)Address; //Читается флаг наличия записи
		if (ReadByte!=0xff)
			{
			//Сохранение восьмого слова Флаг отправки на FTP
			Bytes0=0x00;
			Bytes1=0x00;
			Bytes2=0x00;
			Bytes3=0x00;
			Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
			Address=RecordAddress+0x1C; FLASH_ProgramWord(Address,Data4Byte);
			}
		}
	//Поиск в секторе №9
	SectorAddress=Meas2Address;
	for (i = 0x00; i < 0x1000; ++i)
		{
		RecordAddress=SectorAddress+i*0x20;
		Address=RecordAddress+0x18; ReadByte=*(unsigned char*)Address; //Читается флаг наличия записи
		if (ReadByte!=0xff)
			{
			//Сохранение восьмого слова Флаг отправки на FTP
			Bytes0=0x00;
			Bytes1=0x00;
			Bytes2=0x00;
			Bytes3=0x00;
			Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
			Address=RecordAddress+0x1C; FLASH_ProgramWord(Address,Data4Byte);
			}
		}
	//Поиск в секторе №10
	SectorAddress=Meas3Address;
	for (i = 0x00; i < 0x1000; ++i)
		{
		RecordAddress=SectorAddress+i*0x20;
		Address=RecordAddress+0x18; ReadByte=*(unsigned char*)Address; //Читается флаг наличия записи
		if (ReadByte!=0xff)
			{
			//Сохранение восьмого слова Флаг отправки на FTP
			Bytes0=0x00;
			Bytes1=0x00;
			Bytes2=0x00;
			Bytes3=0x00;
			Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
			Address=RecordAddress+0x1C; FLASH_ProgramWord(Address,Data4Byte);
			}
		}
	//////////////////////////////////////////////////////////////////////
	FLASH_Lock(); //Включается защита памяти от редактирования
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////  Функция, осуществляющая чтение параметров устройства из flash и SRAM памяти  ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_Parametr_Device(void)
	{
	uint32_t Address;
	Address=ParametrAddress; DeviceSerial=*(uint32_t*)Address; //Считывается серийный номер устройства
	Address=ParametrAddress+4; MeasSystem.TermistorConstRes=*(uint32_t*)Address; //Считывается сопротивление постоянного резистора в цепи термистора
	Address=ParametrAddress+8; MeasSystem.TermistorRes=*(uint32_t*)Address; //Считывается сопротивление термистора
	Address=ParametrAddress+12; MeasSystem.PowerConstRes=*(uint32_t*)Address; //Считывается сопротивление резистора в цепи потенциометров
	Read_FTP_Settings(); //Получить параметры FTP соединения из SRAM
	Read_GPRS_Settings(); //Получить параметры GPRS соединения из SRAM
	Read_GPRS_User_Pass(); //Получить имя пользователя и пароль GPRS соединения из SRAM
	Read_FTP_FileName(); //Получить имя файла из SRAM
	Read_ChIDList(); //Получить список измерительных каналов из SRAM
	Read_SMS_Settings(); //Получить настройки доступа по СМС из SRAM
	RTC_Get_Date_Time();
	RTC_Get_Alarm_Time();
	Get_DeviceMode(); //Определяется режим работы устройства
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////   Функция, осуществляющая копирование параметров устройства из буфера в постоянное хранилище   //////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Copy_Parametr_Buf(void)
	{
	unsigned int i; //Счетчик
	uint32_t WriteAddress; //Адрес области в которую производится запись
	uint32_t ReadAddress; //Адрес области из которой копируются данные
	//Копирование данных из одной области памяти в другую
	FLASH_Unlock(); //Разрешается редактирование памяти
	//Очистка памяти
	FLASH_EraseSector(SectorParametr,VoltageRange_3); //Очистка сектора Flash памяти в которую копируются данные
	//Копирование данных
	WriteAddress=ParametrAddress; //Адрес области в которую производится запись
	ReadAddress=ParametrAddressBuf; //Адрес из которой производится чтение данных
	for (i = 0; i < 0x8000; ++i) //Копирование данных
		{
		FLASH_ProgramWord(WriteAddress,*(uint32_t*)ReadAddress);
		ReadAddress+=4;
		WriteAddress+=4;
		}
	//Установка флага корректности записи таблицы параметров
	FLASH_ProgramWord(ParametrFlagWriteAddress,0x00000000);
	FLASH_Lock(); //Включается защита памяти от редактирования
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Функция, осуществляющая переход к приложению которое копирует прошивку устройства из одной области пямяти в другую  //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Jump_To_Application(uint32_t Address)
	{
	__disable_irq(); //запрещаются глобальные прерывании
	typedef  void (*pFunction)(void);
	pFunction Jump_To_Application; //Объявление указателя на функцию
	uint32_t JumpAddress;
	JumpAddress = *(uint32_t*) (Address + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	__set_MSP(*(vu32*) Address);
	Jump_To_Application();
	}




















////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////         Функции реализации задержек и таймаутов          ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////     Функция инициализации основного таймера     ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Main_Timer_Init(void)
	{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM_Main, ENABLE); //Тактирование основного тайммера
	//Инициализация таймера
    TIM_TimeBaseInitTypeDef Timer_InitStructure;
    TIM_TimeBaseStructInit(&Timer_InitStructure);
    Timer_InitStructure.TIM_Prescaler = 8-1; //Предделитель таймера (частота тиков таймера 1 МГц)
    Timer_InitStructure.TIM_Period = 50; //Таймер будет срабатывать каждые 50 мкс (20 кГц)
    Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // пределитель тактирования таймера
    Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; //Направление сччета - на увеличение
    Timer_InitStructure.TIM_RepetitionCounter = 0; //Событие будет возникать каждое переполнение счетчика
    TIM_TimeBaseInit(Timer_Main, &Timer_InitStructure); //Инициализация таймера
    //Конфигурация прерывания
    TIM_ITConfig(Timer_Main, TIM_IT_Update, ENABLE); //Включить прерывание по переполнению счетчика
	NVIC_SetPriority(Timer_Main_IRQn,Prioritet_MainTimer); //Устанавливается приоритет прерывания
	NVIC_EnableIRQ(Timer_Main_IRQn); //Разрешается обработка прерывания от таймера
	//Запуск таймера
	TIM_Cmd(Timer_Main, ENABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////     Событие срабатывания основного таймера       ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TIM2_IRQHandler(void)
	{
	TIM_ClearITPendingBit(Timer_Main, TIM_IT_Update); //Сброс флага ожидания прерывания
	if (Timeouts.DelayMain != 0) {Timeouts.DelayMain--;} //Декрементация счетчика для функции задержки в основной программе
	if (Timeouts.DelayBluetooth != 0) {Timeouts.DelayBluetooth--;} //Декрементация счетчика для функции задержки в прерывании bluetooth
	if (Timeouts.TimeoutGSM_Char > 0) {Timeouts.TimeoutGSM_Char-=50;} //Декрементация счетчика таймаута отправки символа в GSM модуль
	if (Timeouts.TimeoutGSM_AT > 0) {Timeouts.TimeoutGSM_AT-=50;} //Декрементация счетчика таймаута выполнения AT команды GSM модулем
	if (Timeouts.TimeoutADC > 0) {Timeouts.TimeoutADC-=50;} //Декрементация счетчика таймаута выполнения AT команды GSM модулем
	if (Timeouts.TimeoutSendMessageBluetooth > 0) {Timeouts.TimeoutSendMessageBluetooth-=50;} //Декрементация счетчика таймаута передачи сообщения по bluetooth
	if (Timeouts.TimeoutIdle> 0) {Timeouts.TimeoutIdle-=50;} //Декрементация счетчика таймаута простоя устройства
	if (Timeouts.TimeoutRS485_Char > 0) {Timeouts.TimeoutRS485_Char-=50;} //Декрементация счетчика таймаута отправки символа в RS485 модуль
	if (Timeouts.TimeoutRS485_Instruction > 0) {Timeouts.TimeoutRS485_Instruction-=50;} //Декрементация счетчика таймаута ожидания результата инструкции

	Timeouts.TimeFTPSession+=50; //Увеличивается время FTP сессии, мкс

	if (Timeouts.ForceRestart > 50)
		{Timeouts.ForceRestart-=50;}//Декрементация счетчика таймаута принудительного выключения
	else
		{

		if (FTPInProgress!=0x00) //Если данные успешно переданы
			{
			ClearFTPFlags(); //Очистка флагов передачи данных по FTP
			}
		GPIO_ResetBits(Power_Port, Power_Pin); Delay_us(3000000); //Выключается питание устройства
		}

	if (Timeouts.AlarmSend > 50) {Timeouts.AlarmSend-=50;} //Декрементация счетчика счетчика периода передачи сообщения об открытой двери, мкс

	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////               Функция задержки, мкс         /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Delay_us(unsigned int Val)
	{
	Timeouts.DelayMain = Val/50;
	while (Timeouts.DelayMain != 0){};
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////           Функция задержки в прерывании bluetooth, мкс        ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Delay_us_Bluetooth(unsigned int Val)
	{
	Timeouts.DelayBluetooth = Val/50;
	while (Timeouts.DelayBluetooth != 0){};
	}




















////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////                 Функции для работы с RTC              ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////        Функция инициализации часов реального времени          ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RTC__Init(void)
	{
	PWR_BackupAccessCmd(ENABLE);
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);
	RTC_InitTypeDef RTC_InitStructure;
	RCC_RTCCLKCmd(DISABLE);
	RTC_WaitForSynchro(); //Ожидается синхронизация
	//Установить источник тактирования кварц 32768
	RCC_LSEConfig(RCC_LSE_ON);
	while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY) {}
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	RTC_StructInit(&RTC_InitStructure);
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	RTC_InitStructure.RTC_SynchPrediv = 0x7FFF;
	RTC_Init(&RTC_InitStructure);
    //Включается RTC
    RCC_RTCCLKCmd(ENABLE);//Включается RTC
    RTC_WaitForSynchro();//Ожидается синхронизация
    //Настраиваетя вывод сигнала будильника на PC13
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource13, GPIO_AF_TAMPER);
    RTC_OutputConfig(RTC_Output_AlarmA, RTC_OutputPolarity_High);
    RTC_OutputTypeConfig(RTC_OutputType_PushPull);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////        Функция получения текущей отметки времени           ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RTC_Get_Date_Time(void)
	{
	PWR_BackupAccessCmd(ENABLE);
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	RTC_TimeStructInit(&RTC_TimeStructure);
	RTC_DateStructInit(&RTC_DateStructure);
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	DateTime.hour=RTC_TimeStructure.RTC_Hours;
    DateTime.min=RTC_TimeStructure.RTC_Minutes;
    DateTime.sec=RTC_TimeStructure.RTC_Seconds;
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
    DateTime.YY=RTC_DateStructure.RTC_Year;
    DateTime.MM=RTC_DateStructure.RTC_Month;
    DateTime.DD=RTC_DateStructure.RTC_Date;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////        Функция установки текущей даты и времени            ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RTC_Set_Date_Time(void)
	{
	PWR_BackupAccessCmd(ENABLE);
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	RTC_TimeStructInit(&RTC_TimeStructure);
	RTC_DateStructInit(&RTC_DateStructure);
	//Установка времени
	RTC_TimeStructure.RTC_H12 = RTC_HourFormat_24;
	RTC_TimeStructure.RTC_Hours=DateTime.hour;
    RTC_TimeStructure.RTC_Minutes=DateTime.min;
    RTC_TimeStructure.RTC_Seconds=DateTime.sec;
	RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
	//Установка даты
    RTC_DateStructure.RTC_Year=DateTime.YY;
    RTC_DateStructure.RTC_Month=DateTime.MM;
    RTC_DateStructure.RTC_Date=DateTime.DD;
    RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////           Функция получения времени будильника             ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RTC_Get_Alarm_Time(void)
	{
	RTC_AlarmTypeDef Alarm_InitStructure;
   	RTC_AlarmStructInit(&Alarm_InitStructure);
	RTC_GetAlarm(RTC_Format_BIN, RTC_Alarm_A, &Alarm_InitStructure);
	CycleTime.hour=Alarm_InitStructure.RTC_AlarmTime.RTC_Hours;
	CycleTime.min=Alarm_InitStructure.RTC_AlarmTime.RTC_Minutes;
    CycleTime.sec=Alarm_InitStructure.RTC_AlarmTime.RTC_Seconds;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////          Функция установки времени будильника              ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RTC_Set_Alarm_Time(void)
	{
	PWR_BackupAccessCmd(ENABLE);
	RTC_AlarmCmd(RTC_Alarm_A, DISABLE); //Выключается будильник
	RTC_AlarmTypeDef Alarm_InitStructure;
   	RTC_AlarmStructInit(&Alarm_InitStructure);
   	Alarm_InitStructure.RTC_AlarmTime.RTC_H12=RTC_HourFormat_24;
   	Alarm_InitStructure.RTC_AlarmTime.RTC_Hours=CycleTime.hour;
   	Alarm_InitStructure.RTC_AlarmTime.RTC_Minutes=CycleTime.min;
   	Alarm_InitStructure.RTC_AlarmTime.RTC_Seconds=CycleTime.sec;
   	Alarm_InitStructure.RTC_AlarmDateWeekDay=1;
   	Alarm_InitStructure.RTC_AlarmDateWeekDaySel=RTC_AlarmDateWeekDaySel_WeekDay; //Срабатывание будильника будет происходить в определенный день недели
   	Alarm_InitStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay; //Будильник будет срабатывать каждый день
   	RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &Alarm_InitStructure);
   	RTC_AlarmCmd(RTC_Alarm_A, ENABLE); //Включается будильник
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////        Функция установки времени срабатывания будильника      ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RTC_Set_Next_Cycle_Time(void)
	{
	unsigned int CurrentTimeSecond; //Текущее время (количество секунд от начала дня)
	unsigned int CycleTimeSecond; //Время измерительного цикла (количество секунд от начала дня)
	RTC_Get_Date_Time(); //Получается текущая отметка времени
	RTC_Get_Alarm_Time(); //Получается установленное время будильника
	CurrentTimeSecond=DateTime.hour*3600+DateTime.min*60+DateTime.sec;
	CycleTimeSecond=CycleTime.hour*3600+CycleTime.min*60+CycleTime.sec;
	//Время срабатывания будильника инкрементруется до тех пор пока не превысит текущую отметку времени + 20 секунд
	if ((CurrentTimeSecond+CyclePeriod)<CycleTimeSecond) {CurrentTimeSecond+=86400;} //Если текущая отметка времени + период измерений меньше времени следующего периода (текущая отметка времени перешла на следующие сутки)
	while ((CurrentTimeSecond+20)>CycleTimeSecond) {CycleTimeSecond+=CyclePeriod;}
	CycleTimeSecond=CycleTimeSecond%86400;  //Определяется время срабатывания будильника относительно начала суток
	CycleTime.hour=(CycleTimeSecond/3600)%24;
	CycleTime.min=(CycleTimeSecond/60)%60;
	CycleTime.sec=CycleTimeSecond%60;
	RTC_Set_Alarm_Time(); //Устанавливается время следующего измерительного цикла
	RTC_ClearFlag(RTC_FLAG_ALRAF); //Сбрасывается флаг срабатывания будильника
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////         Функция получения режима работы устройства         ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Get_DeviceMode(void)
	{
	GetCycle_Settings(); //Получить параметры периодичности измерений
	if (RTC_GetFlagStatus(RTC_FLAG_ALRAF)==RESET) //Если пробуждение произошло не посрабатыванию будильника
		{DeviceMode=DeviceModeCommand;}
	else
		{
		//Определяется необходимость передачи данных на FTP сервер
		if (CurrentCycleSendDataPeriod==0)
			{
			DeviceMode=DeviceModeCycleFTP;//Измерение с передачей данных
			CurrentCycleSendDataPeriod=CycleSendDataPeriod;
			}
		else
			{
			DeviceMode=DeviceModeCycle;//Измерение без передачи данных
			CurrentCycleSendDataPeriod--;
			}
		}
	SetCycle_Settings(); //Установить параметры периодичности измерений
	RTC_ClearFlag(RTC_FLAG_ALRAF); //Сбрасывается флаг срабатывания будильника
	}




















////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////               Функции для работы с Bluetooth          ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////         Функция инициализации bluetooth модуля             ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Init(void)
	{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//Тактирование USART Bluetooth
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART_BT, ENABLE);
	//Вывод RESET Bluetooth модуля
	GPIO_InitStructure.GPIO_Pin = BT_Reset_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BT_Reset_Port, &GPIO_InitStructure);
	//Вывод PIO_11 Bluetooth модуля
	GPIO_InitStructure.GPIO_Pin = BT_AT_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BT_AT_Port, &GPIO_InitStructure);
	//Tx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = BT_Rx_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(BT_Rx_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	//Конфигурация USART Bluetooth
	USART_InitStructure.USART_BaudRate = 9600;                                       // Скорость
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // длина пакета 1байт/8бит
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // 1 стоп-бит
	USART_InitStructure.USART_Parity = USART_Parity_No;                              // Без контроля четности
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // Разрешаем прием и передачу
	USART_Init(USART_BT, &USART_InitStructure);
	USART_Cmd(USART_BT, ENABLE);
	GPIO_SetBits(BT_AT_Port, BT_AT_Pin); Delay_us(10000);
	GPIO_ResetBits(BT_Reset_Port, BT_Reset_Pin); Delay_us(10000);
	GPIO_SetBits(BT_Reset_Port, BT_Reset_Pin);
	//Перевод модуля в режим передачи данных
	GPIO_ResetBits(BT_AT_Port, BT_AT_Pin); Delay_us(10000);
	GPIO_ResetBits(BT_Reset_Port, BT_Reset_Pin); Delay_us(10000);
	GPIO_SetBits(BT_Reset_Port, BT_Reset_Pin);
	//Таймер Bluetooth
	TIM_TimeBaseInitTypeDef Timer_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM_Bluetooth,ENABLE); //Тактирование таймера приемопередатчика
    TIM_TimeBaseStructInit(&Timer_InitStructure); //Заполняются поля структуры значениями поумолчанию
    Timer_InitStructure.TIM_Prescaler = 8; //Выставляется предделитель
    Timer_InitStructure.TIM_Period = 50; //Выставляется период срабатывания таймера 50мкс
    TIM_TimeBaseInit(Timer_Bluetooth, &Timer_InitStructure); //Инициализация таймера
    TIM_ITConfig(Timer_Bluetooth, TIM_IT_Update, ENABLE); //Настраиваем таймер для генерации прерывания по обновлению (переполнению)
    TIM_Cmd(Timer_Bluetooth, ENABLE); //Запускается таймер
    NVIC_SetPriority(Timer_Bluetooth_IRQn,PrioritetBluetooth); //Устанавливается приоритет прерывания
    NVIC_EnableIRQ(Timer_Bluetooth_IRQn); //Разрешаем соответствующее прерывание
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////         Функция посылки bluetooth модулю AT команды             //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_AT(const unsigned char* str)
	{
	unsigned int i;
	for(i = 0; str[i]; i++)
		{
		while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); //Ожидается окончание отправки байта
		USART_SendData(USART_BT,str[i]); //Отправка байта
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////         Функция отсылки сообщения по bluetooth               /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_Message(void)
	{
	Timeouts.TimeoutSendMessageBluetooth=Send_MessageBT_Timeout; //Запускается счетчик таймаута отсылки сообщения по bluetooth
	//Добавляются символы окончания сообщения
	MessageSelectionBluetooth.Buffer[MessageSelectionBluetooth.Buffer_Len]=0x0d;
	MessageSelectionBluetooth.Buffer[MessageSelectionBluetooth.Buffer_Len+1]=0x0a;
	MessageSelectionBluetooth.Buffer_Len=MessageSelectionBluetooth.Buffer_Len+2;
	MessageSelectionBluetooth.Buffer_Pos=0; //Сбрасывается позиция курсора вбуфере Bluetooth
	while ((USART_GetFlagStatus(USART_BT, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutSendMessageBluetooth>50)) {}; //Ожидается окончание передачи байта
	USART_SendData(USART_BT, 0x0A); //Отсылается символ начла строки
	while (1)
		{
		if (Timeouts.TimeoutSendMessageBluetooth<50) {return;}//Если истекло время, отведенное на отсылку сообщения по Bluetooth
		if (USART_GetFlagStatus(USART_BT, USART_FLAG_TC) != RESET) //Если ушел предыдущий байт
			{
			if (MessageSelectionBluetooth.Buffer_Pos>=MessageSelectionBluetooth.Buffer_Len) {return;}//Если достигнут конец сообщения в буфере
			else
				{
				USART_SendData(USART_BT, MessageSelectionBluetooth.Buffer[MessageSelectionBluetooth.Buffer_Pos]); //Отсылаеся байт по Bluetooth
				MessageSelectionBluetooth.Buffer_Pos++; //Инкрементируется счетчик
				}
			}
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////       Функция производит интерпретацию сообщений принятых по Bluetooth                           /////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Interpretation_Message(void)
	{
	unsigned int i; //Счетчик
	unsigned int CurrentDelimiterNumber=0; //текущий номер разделителя
	MessageInterpretationBluetooth.CorrectMessageFlag=1; //Устанавливается флаг корректности сообщения
	MessageInterpretationBluetooth.TypePosStart=0; //Начало блока "тип сообщения"
	MessageInterpretationBluetooth.TypePosEnd=0; //Конец блока "тип сообщения"
	MessageInterpretationBluetooth.AddressPosStart=0; //Начало блока "адрес"
	MessageInterpretationBluetooth.AddressPosEnd=0; //Конец блока "адрес"
	MessageInterpretationBluetooth.TransactionPosStart=0; //Начало блока "транзакции"
	MessageInterpretationBluetooth.TransactionPosEnd=0; //Конец блока "транзакции"
	MessageInterpretationBluetooth.InstructionPosStart=0; //Начало блока "инструкции"
	MessageInterpretationBluetooth.InstructionPosEnd=0; //Конец блока "инструкции"
	MessageInterpretationBluetooth.DataPosStart=0; //Начало блока "данные"
	MessageInterpretationBluetooth.DataPosEnd=0; //Конец блока "данные"
	//Находятся разделители
	for (i = 0; i < MessageSelectionBluetooth.Buffer_Len; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]=='/') //Если обнаружен разделитель
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //Если обнаружен 0-й разделитель
				MessageInterpretationBluetooth.TypePosStart=i+1; //Запоминается начало блока "тип сообщения"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 1: //Если обнаружен 1-й разделитель
				MessageInterpretationBluetooth.TypePosEnd=i-1; //Запоминается конец блока "тип сообщения"
				MessageInterpretationBluetooth.AddressPosStart=i+1; //Запоминается начало блока "адрес"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 2: //Если обнаружен 2-й разделитель
				MessageInterpretationBluetooth.AddressPosEnd=i-1; //Запоминается конец блока "адрес"
				MessageInterpretationBluetooth.TransactionPosStart=i+1; //Запоминается начало блока "транзакция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 3: //Если обнаружен 3-й разделитель
				MessageInterpretationBluetooth.TransactionPosEnd=i-1; //Запоминается конец блока "транзакция"
				MessageInterpretationBluetooth.InstructionPosStart=i+1; //Запоминается начало блока "инструкция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 4: //Если обнаружен 4-й разделитель
				MessageInterpretationBluetooth.InstructionPosEnd=i-1; //Запоминается конец блока "инструкция"
				MessageInterpretationBluetooth.DataPosStart=i+1; //Запоминается начало блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 5: //Если обнаружен5-й разделитель
				MessageInterpretationBluetooth.DataPosEnd=i-1; //Запоминается конец блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				}
			}
		}
	//Проверяется количество разделителей
	if (CurrentDelimiterNumber!=6) {return;} //Если количество разделителей не коррректно
	//Проверяется корректность блока "тип сообщения"
	if (MessageInterpretationBluetooth.TypePosEnd!=MessageInterpretationBluetooth.TypePosStart) {return;}
	if (MessageSelectionBluetooth.Buffer[MessageInterpretationBluetooth.TypePosStart]!='Q') {return;}//Если сообщение - не является запросом
	MessageSelectionBluetooth.Buffer[MessageInterpretationBluetooth.TypePosStart]='R'; //Изменяется тип сообщения
	//Проверяется корректность блока "адрес"
	if (MessageInterpretationBluetooth.AddressPosStart>MessageInterpretationBluetooth.AddressPosEnd) {return;}//Если блок "данные" пустой, то завершается выполнение функции
	for (i = MessageInterpretationBluetooth.AddressPosStart; i <= MessageInterpretationBluetooth.AddressPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {return;}}//Если символ - не является цифрой
	MessageInterpretationBluetooth.Address=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.AddressPosStart, MessageInterpretationBluetooth.AddressPosEnd); //Расшифровывается адрес в сообщении
	if ((MessageInterpretationBluetooth.Address!=DeviceAddress)&(MessageInterpretationBluetooth.Address!=0)) {return;} //Если не совпадает адрес в сообщении с адресом устройства или широковещательным адресом
	//Проверяется корректность блока "транзакция"
	if (MessageInterpretationBluetooth.TransactionPosStart>MessageInterpretationBluetooth.TransactionPosEnd) {return;}//Если размер блока "транзакция" меньше 1го символа, то завершается выполнение функции
	//-------------------------------Выполняется инструкция
	Bluetooth_Execution_GetType();//Функция производит выполнение инструкции GetType
	Bluetooth_Execution_GetSerial();//Функция производит выполнение инструкции GetSerial
	Bluetooth_Execution_GetProgVersion();//Функция производит выполнение инструкции GetProgVersion
	Bluetooth_Execution_UploadProgramm();//Функция производит выполнение инструкции UploadProgramm
	Bluetooth_Execution_UploadSettings(); //Функция производит выполнение инструкции UploadSettings
	Bluetooth_Execution_DownloadSettings(); //Функция производит выполнение инструкции DownloadSettings
	Bluetooth_Execution_SetServiceMode(); //Функция производит выполнение инструкции SetServiceMode
	Bluetooth_Execution_Test();
	Bluetooth_Execution_SetCycleSettings(); //Функция производит выполнение инструкции SetCycleSettings
	Bluetooth_Execution_GetCycleSettings(); //Функция производит выполнение инструкции GetCycleSettings
	Bluetooth_Execution_SetClock(); //Функция производит выполнение инструкции SetClock
	Bluetooth_Execution_GetClock(); //Функция производит выполнение инструкции GetClock
	Bluetooth_Execution_SetFTPSettings(); //Функция производит выполнение инструкции SetFTPSetting
	Bluetooth_Execution_GetFTPSettings(); //Функция производит выполнение инструкции GetFTPSetting
	Bluetooth_Execution_SetGPRSSettings(); //Функция производит выполнение инструкции SetGPRSSetting
	Bluetooth_Execution_GetGPRSSettings(); //Функция производит выполнение инструкции GetGPRSSetting
	Bluetooth_Execution_SetGPRSUserPass(); //Функция производит выполнение инструкции SetGPRSUserPass
	Bluetooth_Execution_GetGPRSUserPass(); //Функция производит выполнение инструкции GetGPRSUserPass
	Bluetooth_Execution_SetFTPFileName(); //Функция производит выполнение инструкции SetFTPFileName
	Bluetooth_Execution_GetFTPFileName(); //Функция производит выполнение инструкции GetFTPFileName
	Bluetooth_Execution_SetChIDList(); //Функция производит выполнение инструкции SetChIDList
	Bluetooth_Execution_GetChIDList(); //Функция производит выполнение инструкции GetChIDList
	Bluetooth_Execution_SetSMSSettings(); //Функция производит выполнение инструкции SetSMSSettings
	Bluetooth_Execution_GetSMSSettings(); //Функция производит выполнение инструкции GetSMSSettings
	Bluetooth_Execution_DownloadData(); //Функция производит выполнение инструкции DownloadData
	Timeouts.TimeoutIdle=Idle_Timeout; //Устанавливается таймаут простоя устройства
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////                  Функция, вызываемая по прерыванию таймера Bluetooth              /////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TIM4_IRQHandler(void)
	{
	if (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET) //Если в приемном буфере USART_BT нет байта для чтения
		{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //Очищается бит прерывания
		return; //Выход из функции
		}
	MessageSelectionBluetooth.Previous_Byte=MessageSelectionBluetooth.Current_Byte; //Запоминается предыдущий принятый байт
	MessageSelectionBluetooth.Current_Byte=USART_ReceiveData(USART_BT); //Забирается байт из USART
	if (MessageSelectionBluetooth.Start_Collect_Flag) //Если идет процесс выборки сообщений
		{
		MessageSelectionBluetooth.Buffer[MessageSelectionBluetooth.Buffer_Pos]=MessageSelectionBluetooth.Current_Byte; //Заносится текущий байт в буфер сообщения
		MessageSelectionBluetooth.Buffer_Pos++; //Инкрементируется счетчик положения курсора в буфере сообщения
		MessageSelectionBluetooth.Buffer_Len=MessageSelectionBluetooth.Buffer_Pos; //Вычисляется количество данных в буфере сообщения
		if ((MessageSelectionBluetooth.Previous_Byte=='/')&(MessageSelectionBluetooth.Current_Byte=='%')) //Если обнаружен маркер конца сообщения
			{
			MessageSelectionBluetooth.Start_Collect_Flag=0; //Останавливается сборка сообщения
			Bluetooth_Interpretation_Message(); //Распознается инструкция, полученная в сообщении
			TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //Очищается бит прерывания
			return; //Выход из функции
			}
		}
	if (MessageSelectionBluetooth.Buffer_Pos>2047) //Если достигнут конец буфера
		{
		MessageSelectionBluetooth.Buffer_Pos=0; //Сбрасывается положение курсора в буфере
		MessageSelectionBluetooth.Buffer_Len=0; //Сбрасывается счетчик количества байт в буфере
		MessageSelectionBluetooth.Start_Collect_Flag=0; //Сбрасывается Флаг сборки сообщения
		MessageSelectionBluetooth.Current_Byte=0; //Сбрасывается принятый байт
		MessageSelectionBluetooth.Previous_Byte=0; //Сбрасывается предыдущий байт
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //Очищается бит прерывания
		return; //Выход из функции
		}
	if ((MessageSelectionBluetooth.Previous_Byte=='%')&(MessageSelectionBluetooth.Current_Byte=='/')) //Если обнаружен маркер начала сообщения
		{
		MessageSelectionBluetooth.Start_Collect_Flag=1; //Устанавливается флаг запуска процесса выборки сообщения
		MessageSelectionBluetooth.Buffer[0]=MessageSelectionBluetooth.Previous_Byte; //Заносится предыдущий байт в буфер сообщения
		MessageSelectionBluetooth.Buffer[1]=MessageSelectionBluetooth.Current_Byte; //Заносится текущий байт в буфер сообщения
		MessageSelectionBluetooth.Buffer_Pos=2; //Задается позиция курсора в буфере сообщения
		MessageSelectionBluetooth.Buffer_Len=2; //Задается количество заполненных байт в буфере
		}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //Очищается бит прерывания
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//         Функция производит преобразование целого положительного числа в строку и помещает ее в буфер Bluetooth       ////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Unsigned_Int_To_Char_Buf(unsigned int Val, unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //Счетчик
	for (i = EndPos; i >= StartPos; --i)
		{
		MessageSelectionBluetooth.Buffer[i]=Val%10+48;
		Val=Val/10;
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////    Функция производит преобразование части строки, находящейся в буфере Bluetooth, в целое число без знака       /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int Bluetooth_Char_To_Unsigned_Int_Buf(unsigned int StartPos, unsigned int EndPos) //StartPos - позиция первого символа, EndPos - позиция последнего символа
	{
	unsigned int i; //Счетчик
	unsigned int Out=0; //Выходная переменная
	unsigned int M=1; //Множитель текущего разряда
	for (i = EndPos; i >= StartPos; --i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]>47)&(MessageSelectionBluetooth.Buffer[i]<58)) //Если символ - цифра
			{
			Out=Out+M*(MessageSelectionBluetooth.Buffer[i]-48);
			M=M*10; //Увеличивается множитель текущего разряда;
			}
		}
	return Out;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////    Функция производит преобразование части строки, находящейся в буфере Bluetooth, в целое число со знаком       /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
signed int Bluetooth_Char_To_Signed_Int_Buf(unsigned int StartPos, unsigned int EndPos) //StartPos - позиция первого символа, EndPos - позиция последнего символа
	{
	unsigned int i; //Счетчик
	signed int Out=0; //Выходная переменная
	unsigned int M=1; //Множитель текущего разряда
	signed int Znak=1; //множитель, определяющий знак числа
	for (i = EndPos; i >= StartPos; --i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]>47)&(MessageSelectionBluetooth.Buffer[i]<58)) //Если символ - цифра
			{
			Out=Out+M*(MessageSelectionBluetooth.Buffer[i]-48);
			M=M*10; //Увеличивается множитель текущего разряда;
			}
		if (MessageSelectionBluetooth.Buffer[i]==45) //Если символ - '-'
			{
			Znak=-1;
			}
		}
	Out=Out*Znak; //Задается знак числа
	return Out;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Функция производит преобразование части строки в формате HEX, находящейся в буфере Bluetooth, в символьную строку GSM.GPRS_Settings /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_HexChar_Buf_To_GPRS_Settings(unsigned int StartPos, unsigned int EndPos)
{
	unsigned int i; //Счетчик
	unsigned char j=0; //Счетчик
	unsigned char TempByte=0; //Временный байт
	unsigned char Byte1=0; //Первый полубайт(старший)
	unsigned char Byte2=0; //Второй полубайт(младший)
	i = StartPos;
	while ((i < EndPos+1)&&(j < 256))
		{
		TempByte=MessageSelectionBluetooth.Buffer[i]; Byte1=0;
		if (TempByte=='0') {Byte1=0;}
		if (TempByte=='1') {Byte1=1;}
		if (TempByte=='2') {Byte1=2;}
		if (TempByte=='3') {Byte1=3;}
		if (TempByte=='4') {Byte1=4;}
		if (TempByte=='5') {Byte1=5;}
		if (TempByte=='6') {Byte1=6;}
		if (TempByte=='7') {Byte1=7;}
		if (TempByte=='8') {Byte1=8;}
		if (TempByte=='9') {Byte1=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte1=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte1=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte1=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte1=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte1=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte1=15;}
		i++;
		TempByte=MessageSelectionBluetooth.Buffer[i]; Byte2=0;
		if (TempByte=='0') {Byte2=0;}
		if (TempByte=='1') {Byte2=1;}
		if (TempByte=='2') {Byte2=2;}
		if (TempByte=='3') {Byte2=3;}
		if (TempByte=='4') {Byte2=4;}
		if (TempByte=='5') {Byte2=5;}
		if (TempByte=='6') {Byte2=6;}
		if (TempByte=='7') {Byte2=7;}
		if (TempByte=='8') {Byte2=8;}
		if (TempByte=='9') {Byte2=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte2=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte2=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte2=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte2=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte2=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte2=15;}
		i++;
		GSM.GPRS_Settings[j]=Byte1*16+Byte2;j++;
		}
	GSM.GPRS_Settings[j]='\0'; //Добавляется завершающий символ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Функция производит преобразование части строки в формате HEX, находящейся в буфере Bluetooth, в символьную строку GSM.FTP_Settings /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_HexChar_Buf_To_FTP_Settings(unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //Счетчик
	unsigned char j=0; //Счетчик
	unsigned char TempByte=0; //Временный байт
	unsigned char Byte1=0; //Первый полубайт(старший)
	unsigned char Byte2=0; //Второй полубайт(младший)
	i = StartPos;
	while ((i < EndPos+1)&&(j < 256))
		{
		TempByte=MessageSelectionBluetooth.Buffer[i]; Byte1=0;
		if (TempByte=='0') {Byte1=0;}
		if (TempByte=='1') {Byte1=1;}
		if (TempByte=='2') {Byte1=2;}
		if (TempByte=='3') {Byte1=3;}
		if (TempByte=='4') {Byte1=4;}
		if (TempByte=='5') {Byte1=5;}
		if (TempByte=='6') {Byte1=6;}
		if (TempByte=='7') {Byte1=7;}
		if (TempByte=='8') {Byte1=8;}
		if (TempByte=='9') {Byte1=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte1=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte1=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte1=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte1=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte1=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte1=15;}
		i++;
		TempByte=MessageSelectionBluetooth.Buffer[i]; Byte2=0;
		if (TempByte=='0') {Byte2=0;}
		if (TempByte=='1') {Byte2=1;}
		if (TempByte=='2') {Byte2=2;}
		if (TempByte=='3') {Byte2=3;}
		if (TempByte=='4') {Byte2=4;}
		if (TempByte=='5') {Byte2=5;}
		if (TempByte=='6') {Byte2=6;}
		if (TempByte=='7') {Byte2=7;}
		if (TempByte=='8') {Byte2=8;}
		if (TempByte=='9') {Byte2=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte2=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte2=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte2=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte2=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte2=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte2=15;}
		i++;
		GSM.FTP_Settings[j]=Byte1*16+Byte2;j++;
		}
	GSM.FTP_Settings[j]='\0'; //Добавляется завершающий символ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Функция производит преобразование части строки в формате HEX, находящейся в буфере Bluetooth, в символьную строку GSM.GPRS_User_Pass ////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_HexChar_Buf_To_GPRS_User_Pass(unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //Счетчик
	unsigned char j=0; //Счетчик
	unsigned char TempByte=0; //Временный байт
	unsigned char Byte1=0; //Первый полубайт(старший)
	unsigned char Byte2=0; //Второй полубайт(младший)
	i = StartPos;
	while ((i < EndPos+1)&&(j < 256))
		{
		TempByte=MessageSelectionBluetooth.Buffer[i]; Byte1=0;
		if (TempByte=='0') {Byte1=0;}
		if (TempByte=='1') {Byte1=1;}
		if (TempByte=='2') {Byte1=2;}
		if (TempByte=='3') {Byte1=3;}
		if (TempByte=='4') {Byte1=4;}
		if (TempByte=='5') {Byte1=5;}
		if (TempByte=='6') {Byte1=6;}
		if (TempByte=='7') {Byte1=7;}
		if (TempByte=='8') {Byte1=8;}
		if (TempByte=='9') {Byte1=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte1=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte1=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte1=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte1=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte1=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte1=15;}
		i++;
		TempByte=MessageSelectionBluetooth.Buffer[i]; Byte2=0;
		if (TempByte=='0') {Byte2=0;}
		if (TempByte=='1') {Byte2=1;}
		if (TempByte=='2') {Byte2=2;}
		if (TempByte=='3') {Byte2=3;}
		if (TempByte=='4') {Byte2=4;}
		if (TempByte=='5') {Byte2=5;}
		if (TempByte=='6') {Byte2=6;}
		if (TempByte=='7') {Byte2=7;}
		if (TempByte=='8') {Byte2=8;}
		if (TempByte=='9') {Byte2=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte2=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte2=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte2=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte2=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte2=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte2=15;}
		i++;
		GSM.GPRS_User_Pass[j]=Byte1*16+Byte2;j++;
		}
	GSM.GPRS_User_Pass[j]='\0'; //Добавляется завершающий символ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Функция производит преобразование части строки в формате HEX, находящейся в буфере Bluetooth, в символьную строку GSM.GPRS_User_Pass ////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_HexChar_Buf_To_FTP_FileName(unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //Счетчик
	unsigned char j=0; //Счетчик
	unsigned char TempByte=0; //Временный байт
	unsigned char Byte1=0; //Первый полубайт(старший)
	unsigned char Byte2=0; //Второй полубайт(младший)
	i = StartPos;
	while ((i < EndPos+1)&&(j < 256))
		{
		TempByte=MessageSelectionBluetooth.Buffer[i]; Byte1=0;
		if (TempByte=='0') {Byte1=0;}
		if (TempByte=='1') {Byte1=1;}
		if (TempByte=='2') {Byte1=2;}
		if (TempByte=='3') {Byte1=3;}
		if (TempByte=='4') {Byte1=4;}
		if (TempByte=='5') {Byte1=5;}
		if (TempByte=='6') {Byte1=6;}
		if (TempByte=='7') {Byte1=7;}
		if (TempByte=='8') {Byte1=8;}
		if (TempByte=='9') {Byte1=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte1=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte1=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte1=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte1=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte1=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte1=15;}
		i++;
		TempByte=MessageSelectionBluetooth.Buffer[i]; Byte2=0;
		if (TempByte=='0') {Byte2=0;}
		if (TempByte=='1') {Byte2=1;}
		if (TempByte=='2') {Byte2=2;}
		if (TempByte=='3') {Byte2=3;}
		if (TempByte=='4') {Byte2=4;}
		if (TempByte=='5') {Byte2=5;}
		if (TempByte=='6') {Byte2=6;}
		if (TempByte=='7') {Byte2=7;}
		if (TempByte=='8') {Byte2=8;}
		if (TempByte=='9') {Byte2=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte2=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte2=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte2=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte2=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte2=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte2=15;}
		i++;
		GSM.FTP_File_Name[j]=Byte1*16+Byte2;j++;
		}
	GSM.FTP_File_Name[j]='\0'; //Добавляется завершающий символ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////                   Функция отсылает ответ ErrorData              ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_ErrorData(void) //Функция отсылает сообщение об ошибке данных
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //Если сообщение широковещательное, то ответ не отсылается
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	Pos=MessageInterpretationBluetooth.DataPosStart;
	MessageSelectionBluetooth.Buffer[Pos]='E'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='o'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='D'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='a'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='t'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='a'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message();
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////                   Функция отсылает ответ ErrorCh                ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_ErrorCh(void) //Функция отсылает сообщение об ошибке канала
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //Если сообщение широковещательное, то ответ не отсылается
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	Pos=MessageInterpretationBluetooth.DataPosStart;
	MessageSelectionBluetooth.Buffer[Pos]='E'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='o'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='C'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='h'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message();
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////                Функция отсылает ответ ErrorSensor               ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_ErrorSensor(void) //Функция отсылает сообщение об ошибке сенсора
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //Если сообщение широковещательное, то ответ не отсылается
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	Pos=MessageInterpretationBluetooth.DataPosStart;
	MessageSelectionBluetooth.Buffer[Pos]='E'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='o'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='S'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='e'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='n'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='s'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='o'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message();
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////                  Функция отсылает ответ End                     ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_End(void) //Функция отсылает сообщение о завершении передачи блока данных
	{
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	Pos=MessageInterpretationBluetooth.DataPosStart;
	MessageSelectionBluetooth.Buffer[Pos]='E'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='n'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='d'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message();
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////                  Функция отсылает ответ CRCError                ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_CRCError(void) //Функция отсылает сообщение об ошибке контрольной суммы
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //Если сообщение широковещательное, то ответ не отсылается
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	Pos=MessageInterpretationBluetooth.DataPosStart;
	MessageSelectionBluetooth.Buffer[Pos]='C'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='R'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='C'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='E'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='o'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='r'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message();
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////                   Функция отсылает ответ CRC_Ok                 ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_CRC_Ok(void) //Функция отсылает сообщение об ошибке контрольной суммы
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //Если сообщение широковещательное, то ответ не отсылается
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	Pos=MessageInterpretationBluetooth.DataPosStart;
	MessageSelectionBluetooth.Buffer[Pos]='C'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='R'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='C'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='_'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='O'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='k'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message();
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////     Функция производит выполнение инструкции GetType      /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetType(void)
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //Если Адрес сообщения широковещательный, то ответ не посылается
	unsigned char StrInstruction[7]={'G','e','t','T','y','p','e'};
	unsigned char InstructionLength=7; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//В сообщение добавляется результат выполнения инструкции
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Bluetooth_Unsigned_Int_To_Char_Buf(DeviceType, Pos, Pos+2); Pos+=3;//В буфер добавляется тип устройства
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////     Функция производит выполнение инструкции GetSerial      ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetSerial(void)
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //Если Адрес сообщения широковещательный, то ответ не посылается
	unsigned char StrInstruction[9]={'G','e','t','S','e','r','i','a','l'};//Получить серийный номер устройства
	unsigned char InstructionLength=9; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Bluetooth_Unsigned_Int_To_Char_Buf(DeviceSerial, Pos, Pos+7); Pos+=8;//В буфер добавляется серийный номер
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////     Функция производит выполнение инструкции GetProgVersion      /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetProgVersion(void)
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //Если Адрес сообщения широковещательный, то ответ не посылается
	unsigned char StrInstruction[14]={'G','e','t','P','r','o','g','V','e','r','s','i','o','n'};//Получить версию программного обеспечения устройства
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Bluetooth_Unsigned_Int_To_Char_Buf(ProgrammVersion_D, Pos, Pos+1); Pos+=2; //Версия программного обеспечения (день)
	MessageSelectionBluetooth.Buffer[Pos]='.'; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(ProgrammVersion_M, Pos, Pos+1); Pos+=2; //Версия программного обеспечения (месяц)
	MessageSelectionBluetooth.Buffer[Pos]='.'; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(ProgrammVersion_Y, Pos, Pos+1); Pos+=2; //Версия программного обеспечения (год)
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////     Функция производит выполнение инструкции UploadProgramm      /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_UploadProgramm(void) //Выгрузить в устройство новую внутреннюю программу
	{
	unsigned char StrInstruction[14]={'U','p','l','o','a','d','P','r','o','g','r','a','m','m'};
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned char CRC_Ok; //Флаг корректности контрольной суммы
	unsigned int i; //Счетчик
	unsigned int DelimiterData1Pos=0; //Позиция первого разделителя (запятая) в блоке "Данные"
	unsigned int LengthData; //Размер записываемых в flash память данных (количество слов из 4-х байт)
	unsigned int CRC32; //Принятая контрольная сумма для блока данных
	if (ServiceCode!=141592) {return;} //Код сервисного режима (141592 - прошивка программы, 832735 - калибровка, 793238 - измениеие настроек)
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//Проверка блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//Если блок "данные" пустой
	//Определяются позиции запятых в блоке "Данные"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //Если в блоке "данные" обнаружена первая запятая
		}
	//Проверка количества элементов блока "Данные"
	if (DelimiterData1Pos==0) {Bluetooth_Send_ErrorData(); return;} //Если в блоке "данные" меньше 1 запятой
	//Проверка размера элементов блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //Если первый элемент пустой
	if (DelimiterData1Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //Если второй элемент пустой
	//Проверка первого элемента блока "Данные" (количество слов) на корректность
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}}//Если символ - не является цифрой
	LengthData=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.DataPosStart, DelimiterData1Pos-1); //Размер записываемых в flash память данных (количество слов из 4-х байт)
	if (LengthData>32760) {return; Bluetooth_Send_ErrorData(); return;}//Если размер блока данных превышает объем выделяемой памяти
	//Проверка второго элемента блока "Данные" (контрольная сумма) на корректность
	for (i = DelimiterData1Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //Если символ - не является цифрой
	CRC32=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, MessageInterpretationBluetooth.DataPosEnd); //Контрольная сумма CRC32
	//Производится загрузка программы
	CRC_Ok=Recive_Data_To_Flash(ProgrammAddressBuf, LengthData, SectorProgrammBuf, CRC32); //Производится загрузка программы
	if (CRC_Ok) //Если совпали контрольные суммы
		{
		Bluetooth_Send_CRC_Ok();
		Jump_To_Application(ProgrammatorAddress); //Переход к внутреннему программатору
		}
	else
		{Bluetooth_Send_CRCError();}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////     Функция производит выполнение инструкции UploadSettings      /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_UploadSettings(void) //Выгрузить в устройство массив с настройками и параметрами
	{
	unsigned char StrInstruction[14]={'U','p','l','o','a','d','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned char CRC_Ok; //Флаг корректности контрольной суммы
	unsigned int i; //Счетчик
	unsigned int DelimiterData1Pos=0; //Позиция первого разделителя (запятая) в блоке "Данные"
	unsigned int LengthData; //Размер записываемых в flash память данных (количество слов из 4-х байт)
	unsigned int CRC32; //Принятая контрольная сумма для блока данных
	if (ServiceCode!=793238) {return;} //Код сервисного режима (141592 - прошивка программы, 832735 - калибровка, 793238 - измениеие настроек)
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//Проверка блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//Если блок "данные" пустой
	//Определяются позиции запятых в блоке "Данные"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //Если в блоке "данные" обнаружена первая запятая
		}
	//Проверка количества элементов блока "Данные"
	if (DelimiterData1Pos==0) {Bluetooth_Send_ErrorData(); return;} //Если в блоке "данные" меньше 1 запятой
	//Проверка размера элементов блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //Если первый элемент пустой
	if (DelimiterData1Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //Если второй элемент пустой
	//Проверка первого элемента блока "Данные" (количество слов) на корректность
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}}//Если символ - не является цифрой
	LengthData=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.DataPosStart, DelimiterData1Pos-1); //Размер записываемых в flash память данных (количество слов из 4-х байт)
	if (LengthData>254) {Bluetooth_Send_ErrorData(); return;}//Если размер блока данных превышает объем выделяемой памяти
	//Проверка второго элемента блока "Данные" (контрольная сумма) на корректность
	for (i = DelimiterData1Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //Если символ - не является цифрой
	CRC32=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, MessageInterpretationBluetooth.DataPosEnd); //Контрольная сумма CRC32
	//Производится загрузка параметров
	CRC_Ok=Recive_Data_To_Flash(ParametrAddressBuf, LengthData, SectorParametrBuf, CRC32); //Производится загрузка таблицы параметров
	if (CRC_Ok) //Если совпали контрольные суммы
		{
		Bluetooth_Send_CRC_Ok();
		Copy_Parametr_Buf(); //Копируются параметры из буфера в постоянное хранилище
		Read_Parametr_Device(); //Чтение параметров
		}
	else
		{Bluetooth_Send_CRCError();}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////     Функция производит выполнение инструкции DownloadSettings    /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_DownloadSettings(void) //Загрузить из  устройства массив с настройками и параметрами
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //Если Адрес сообщения широковещательный, то ответ не посылается
	unsigned char StrInstruction[16]={'D','o','w','n','l','o','a','d','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=16; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	unsigned char TempByte; //Временный байт
	uint32_t Address; //Адрес ячейки памяти
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	MessageSelectionBluetooth.Buffer[MessageInterpretationBluetooth.TypePosStart]='R'; //Изменяется тип сообщения
	Pos=MessageInterpretationBluetooth.DataPosStart;
	MessageSelectionBluetooth.Buffer[Pos]='0'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='2'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='5'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='6'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(Calculate_CRC(ParametrAddress, 256),Pos,Pos+10); Pos+=11; //Контрольная сумма
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	Delay_us_Bluetooth(1000000); //Задержка 1 сек
	//Передача данных
	for (i = 0; i < 1024; ++i)
		{
		Address=ParametrAddress+i; TempByte=*(unsigned char*)Address;
		while (USART_GetFlagStatus(USART_BT, USART_FLAG_TC) == RESET) {};
		Delay_us_Bluetooth(1500); //Задержка 1.5 мс
		USART_SendData(USART_BT, TempByte); //Передается байт в USART_BT
		}
	while (USART_GetFlagStatus(USART_BT, USART_FLAG_TC) == RESET) {}; //Ожидается пока уйдет байт
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////     Функция производит выполнение инструкции SetServiceMode       /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetServiceMode(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','S','e','r','v','i','c','e','M','o','d','e'};
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка блока "Данные" на корректность
	if (MessageInterpretationBluetooth.DataPosStart>MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //Если блок "данные" пустой, то завершается выполнение функции
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}}//Если символ - не является цифрой
	ServiceCode=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.DataPosStart, MessageInterpretationBluetooth.DataPosEnd); //Код сервисного режима (141592 - прошивка программы, 832735 - калибровка, 793238 - измениеие настроек)
	if (MessageInterpretationBluetooth.Address==0) {return;} //Если Адрес сообщения широковещательный, то ответ не посылается
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////      Функция производит выполнение инструкции SetCycleSettings        ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetCycleSettings(void)
	{
	unsigned char StrInstruction[16]={'S','e','t','C','y','c','l','e','S','e','t','t','i','n','g','s'};//Установить параметры измерительного цикла
	unsigned char InstructionLength=16; //Размер инструкции с которой производится сравнение
	unsigned int DelimiterData1Pos=0; //Позиция первого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData2Pos=0; //Позиция второго разделителя (запятая) в блоке "Данные"
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//Проверка блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//Если блок "данные" пустой
	//Определяются позиции запятых в блоке "Данные"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos!=0)&(DelimiterData2Pos==0)) {DelimiterData2Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //Если в блоке "данные" обнаружена первая запятая
		}
	//Проверка количества элементов блока "Данные"
	if (DelimiterData2Pos==0) {Bluetooth_Send_ErrorData(); return;} //Если в блоке "данные" меньше 1 запятой
	//Проверка размера элементов блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //Если первый элемент пустой
	if (DelimiterData2Pos==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //Если второй элемент пустой
	if (DelimiterData2Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //Если третий элемент пустой
	//Проверка первого элемента блока "Данные" (CyclePeriod) на корректность
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}}//Если символ - не является цифрой
	CyclePeriod=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.DataPosStart, DelimiterData1Pos-1); //Период измерений
	//Проверка второго элемента блока "Данные" (CycleStart) на корректность
	for (i = DelimiterData1Pos+1; i < DelimiterData2Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //Если символ - не является цифрой
	CycleStart=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, DelimiterData2Pos-1); //Начальное время измерения
	//Проверка третьего элемента блока "Данные" (CycleSendDataPeriod) на корректность
	for (i = DelimiterData2Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //Если символ - не является цифрой
	CycleSendDataPeriod=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData2Pos+1, MessageInterpretationBluetooth.DataPosEnd); //Период передачи данных
	if (CycleSendDataPeriod>MaximumPeriodSendData) {Bluetooth_Send_ErrorData(); return;} //Если количество пропусков передачи данных больше 255
	if (CyclePeriod>86400) {Bluetooth_Send_ErrorData(); return;} //Если период больше 24 часов
	if (CyclePeriod<MinimumPeriod) {Bluetooth_Send_ErrorData(); return;} //Если период меньше 5 минут
	if (CycleStart>86399) {Bluetooth_Send_ErrorData(); return;}
	CurrentCycleSendDataPeriod=0;//Сброс текущего пропуска передачи данных
	SetCycle_Settings(); //Установка настроек циклов измерения
	RTC_Set_Next_Cycle_Time(); //Устанавливается время следующего цикла
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////      Функция производит выполнение инструкции GetCycleSettings        ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetCycleSettings(void)
	{
	unsigned char StrInstruction[16]={'G','e','t','C','y','c','l','e','S','e','t','t','i','n','g','s'};//Получить параметры измерительного цикла
	unsigned char InstructionLength=16; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	Pos=MessageInterpretationBluetooth.DataPosStart;
	GetCycle_Settings(); //Получаются параметры работы цикла
	Bluetooth_Unsigned_Int_To_Char_Buf(CyclePeriod, Pos, Pos+5); Pos+=6; //Период измерений
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(CycleStart, Pos, Pos+5); Pos+=6; //Время следующего измерения от начала суток
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(CycleSendDataPeriod, Pos, Pos+3); Pos+=4; //Период отправки данных
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции SetClock              ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetClock(void)
	{
	unsigned char StrInstruction[8]={'S','e','t','C','l','o','c','k'};//Установить параметры измерительного цикла
	unsigned char InstructionLength=8; //Размер инструкции с которой производится сравнение
	unsigned int DelimiterData1Pos=0; //Позиция первого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData2Pos=0; //Позиция второго разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData3Pos=0; //Позиция третьего разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData4Pos=0; //Позиция четвертого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData5Pos=0; //Позиция пятого разделителя (запятая) в блоке "Данные"
	unsigned int i; //Счетчик
	unsigned int TS_Year=0; //Год
	unsigned int TS_Month=0; //Месяц
	unsigned int TS_Day=0; //День
	unsigned int TS_Hour=0; //Час
	unsigned int TS_Min=0; //Минуты
	unsigned int TS_Sec=0; //Секунды
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//Проверка блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//Если блок "данные" пустой
	//Определяются позиции запятых в блоке "Данные"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData4Pos!=0)&(DelimiterData5Pos==0)) {DelimiterData5Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData3Pos!=0)&(DelimiterData4Pos==0)) {DelimiterData4Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData2Pos!=0)&(DelimiterData3Pos==0)) {DelimiterData3Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos!=0)&(DelimiterData2Pos==0)) {DelimiterData2Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //Если в блоке "данные" обнаружена первая запятая
		}
	//Проверка количества элементов блока "Данные"
	if (DelimiterData5Pos==0) {Bluetooth_Send_ErrorData(); return;} //Если в блоке "данные" меньше 1 запятой
	//Проверка размера элементов блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //Если первый элемент пустой
	if (DelimiterData2Pos==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //Если второй элемент пустой
	if (DelimiterData3Pos==DelimiterData2Pos) {Bluetooth_Send_ErrorData(); return;} //Если третий элемент пустой
	if (DelimiterData4Pos==DelimiterData3Pos) {Bluetooth_Send_ErrorData(); return;} //Если четвертый элемент пустой
	if (DelimiterData5Pos==DelimiterData4Pos) {Bluetooth_Send_ErrorData(); return;} //Если пятый элемент пустой
	if (DelimiterData5Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //Если шестой элемент пустой
	//Проверка первого элемента блока "Данные" (Год) на корректность
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}}//Если символ - не является цифрой
	TS_Year=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.DataPosStart, DelimiterData1Pos-1);
	//Проверка второго элемента блока "Данные" (Месяц) на корректность
	for (i = DelimiterData1Pos+1; i < DelimiterData2Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //Если символ - не является цифрой
	TS_Month=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, DelimiterData2Pos-1);
	//Проверка второго элемента блока "Данные" (День) на корректность
	for (i = DelimiterData2Pos+1; i < DelimiterData3Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //Если символ - не является цифрой
	TS_Day=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData2Pos+1, DelimiterData3Pos-1);
	//Проверка второго элемента блока "Данные" (Час) на корректность
	for (i = DelimiterData3Pos+1; i < DelimiterData4Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //Если символ - не является цифрой
	TS_Hour=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData3Pos+1, DelimiterData4Pos-1);
	//Проверка второго элемента блока "Данные" (Минуты) на корректность
	for (i = DelimiterData4Pos+1; i < DelimiterData5Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //Если символ - не является цифрой
	TS_Min=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData4Pos+1, DelimiterData5Pos-1); //Начальное время измерения
	//Проверка третьего элемента блока "Секунды" (CycleSendDataPeriod) на корректность
	for (i = DelimiterData5Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //Если символ - не является цифрой
	TS_Sec=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData5Pos+1, MessageInterpretationBluetooth.DataPosEnd); //Период передачи данных
	//Проверка корректности данных
	if (TS_Year>99) {Bluetooth_Send_ErrorData(); return;}
	if (TS_Month>12) {Bluetooth_Send_ErrorData(); return;}
	if (TS_Day>31) {Bluetooth_Send_ErrorData(); return;}
	if (TS_Hour>23) {Bluetooth_Send_ErrorData(); return;}
	if (TS_Min>59) {Bluetooth_Send_ErrorData(); return;}
	if (TS_Sec>59) {Bluetooth_Send_ErrorData(); return;}
	DateTime.YY=TS_Year;
	DateTime.MM=TS_Month;
	DateTime.DD=TS_Day;
	DateTime.hour=TS_Hour;
	DateTime.min=TS_Min;
	DateTime.sec=TS_Sec;
	RTC_Set_Date_Time(); //Установка текущего времени
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции GetClock              ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetClock(void)
	{
	unsigned char StrInstruction[8]={'G','e','t','C','l','o','c','k'};//Установить параметры измерительного цикла
	unsigned char InstructionLength=8; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	Pos=MessageInterpretationBluetooth.DataPosStart;
	RTC_Get_Date_Time(); //Получается текущая отметка времени
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.YY, Pos, Pos+1); Pos+=2; //Год
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.MM, Pos, Pos+1); Pos+=2; //месяц
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.DD, Pos, Pos+1); Pos+=2; //День
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.hour, Pos, Pos+1); Pos+=2; //Часы
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.min, Pos, Pos+1); Pos+=2; //Минуты
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.sec, Pos, Pos+1); Pos+=2; //Секунды
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции SetFTPSettings               /////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetFTPSettings(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','F','T','P','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
		//Проверка блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//Если блок "данные" пустой
	//Проверка строки на корректность
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]<'0') {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if (MessageSelectionBluetooth.Buffer[i]>'f') {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if ((MessageSelectionBluetooth.Buffer[i]>'9')&&(MessageSelectionBluetooth.Buffer[i]<'A')) {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if ((MessageSelectionBluetooth.Buffer[i]>'F')&&(MessageSelectionBluetooth.Buffer[i]<'a')) {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		}
	//Данные преобразуются в символьный вид
	Bluetooth_HexChar_Buf_To_FTP_Settings(MessageInterpretationBluetooth.DataPosStart,MessageInterpretationBluetooth.DataPosEnd);
	Save_FTP_Settings(); //Строка сохраняется в SRAM
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции GetFTPSettings               /////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetFTPSettings(void)
	{
	unsigned char StrInstruction[14]={'G','e','t','F','T','P','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	unsigned char Len_STR; //Размер отсылаемой строки
	unsigned char TempByte; //Временный байт
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Read_FTP_Settings(); //Строка извлекается из SRAM
	Len_STR=strlen(GSM.FTP_Settings); //Размер отсылаемой строки
	if ((Len_STR>0)&&(Len_STR<257)) //Если строка имеет правильный размер
		{
		for (i = 0; i < Len_STR; ++i)
			{
			//Отсылается старший полубайт
			TempByte=GSM.FTP_Settings[i]/16;
			if (TempByte==0) {MessageSelectionBluetooth.Buffer[Pos]='0';}
			if (TempByte==1) {MessageSelectionBluetooth.Buffer[Pos]='1';}
			if (TempByte==2) {MessageSelectionBluetooth.Buffer[Pos]='2';}
			if (TempByte==3) {MessageSelectionBluetooth.Buffer[Pos]='3';}
			if (TempByte==4) {MessageSelectionBluetooth.Buffer[Pos]='4';}
			if (TempByte==5) {MessageSelectionBluetooth.Buffer[Pos]='5';}
			if (TempByte==6) {MessageSelectionBluetooth.Buffer[Pos]='6';}
			if (TempByte==7) {MessageSelectionBluetooth.Buffer[Pos]='7';}
			if (TempByte==8) {MessageSelectionBluetooth.Buffer[Pos]='8';}
			if (TempByte==9) {MessageSelectionBluetooth.Buffer[Pos]='9';}
			if (TempByte==10) {MessageSelectionBluetooth.Buffer[Pos]='A';}
			if (TempByte==11) {MessageSelectionBluetooth.Buffer[Pos]='B';}
			if (TempByte==12) {MessageSelectionBluetooth.Buffer[Pos]='C';}
			if (TempByte==13) {MessageSelectionBluetooth.Buffer[Pos]='D';}
			if (TempByte==14) {MessageSelectionBluetooth.Buffer[Pos]='E';}
			if (TempByte==15) {MessageSelectionBluetooth.Buffer[Pos]='F';}
			Pos++;
			//Отсылается младший полубайт
			TempByte=GSM.FTP_Settings[i]%16;
			if (TempByte==0) {MessageSelectionBluetooth.Buffer[Pos]='0';}
			if (TempByte==1) {MessageSelectionBluetooth.Buffer[Pos]='1';}
			if (TempByte==2) {MessageSelectionBluetooth.Buffer[Pos]='2';}
			if (TempByte==3) {MessageSelectionBluetooth.Buffer[Pos]='3';}
			if (TempByte==4) {MessageSelectionBluetooth.Buffer[Pos]='4';}
			if (TempByte==5) {MessageSelectionBluetooth.Buffer[Pos]='5';}
			if (TempByte==6) {MessageSelectionBluetooth.Buffer[Pos]='6';}
			if (TempByte==7) {MessageSelectionBluetooth.Buffer[Pos]='7';}
			if (TempByte==8) {MessageSelectionBluetooth.Buffer[Pos]='8';}
			if (TempByte==9) {MessageSelectionBluetooth.Buffer[Pos]='9';}
			if (TempByte==10) {MessageSelectionBluetooth.Buffer[Pos]='A';}
			if (TempByte==11) {MessageSelectionBluetooth.Buffer[Pos]='B';}
			if (TempByte==12) {MessageSelectionBluetooth.Buffer[Pos]='C';}
			if (TempByte==13) {MessageSelectionBluetooth.Buffer[Pos]='D';}
			if (TempByte==14) {MessageSelectionBluetooth.Buffer[Pos]='E';}
			if (TempByte==15) {MessageSelectionBluetooth.Buffer[Pos]='F';}
			Pos++;
			}
		}
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции SetGPRSSettings               ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetGPRSSettings(void)
	{
	unsigned char StrInstruction[15]={'S','e','t','G','P','R','S','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=15; //Размер инструкции с которой производится сравнение
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
		//Проверка блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//Если блок "данные" пустой
	//Проверка строки на корректность
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]<'0') {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if (MessageSelectionBluetooth.Buffer[i]>'f') {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if ((MessageSelectionBluetooth.Buffer[i]>'9')&&(MessageSelectionBluetooth.Buffer[i]<'A')) {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if ((MessageSelectionBluetooth.Buffer[i]>'F')&&(MessageSelectionBluetooth.Buffer[i]<'a')) {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		}
	//Данные преобразуются в символьный вид
	Bluetooth_HexChar_Buf_To_GPRS_Settings(MessageInterpretationBluetooth.DataPosStart,MessageInterpretationBluetooth.DataPosEnd);
	Save_GPRS_Settings(); //Строка сохраняется в SRAM
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции GetGPRSSettings              /////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetGPRSSettings(void)
	{
	unsigned char StrInstruction[15]={'G','e','t','G','P','R','S','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=15; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	unsigned char Len_STR; //Размер отсылаемой строки
	unsigned char TempByte; //Временный байт
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Read_GPRS_Settings(); //Строка извлекается из SRAM
	Len_STR=strlen(GSM.GPRS_Settings); //Размер отсылаемой строки
	if ((Len_STR>0)&&(Len_STR<257)) //Если строка имеет правильный размер
		{
		for (i = 0; i < Len_STR; ++i)
			{
			//Отсылается старший полубайт
			TempByte=GSM.GPRS_Settings[i]/16;
			if (TempByte==0) {MessageSelectionBluetooth.Buffer[Pos]='0';}
			if (TempByte==1) {MessageSelectionBluetooth.Buffer[Pos]='1';}
			if (TempByte==2) {MessageSelectionBluetooth.Buffer[Pos]='2';}
			if (TempByte==3) {MessageSelectionBluetooth.Buffer[Pos]='3';}
			if (TempByte==4) {MessageSelectionBluetooth.Buffer[Pos]='4';}
			if (TempByte==5) {MessageSelectionBluetooth.Buffer[Pos]='5';}
			if (TempByte==6) {MessageSelectionBluetooth.Buffer[Pos]='6';}
			if (TempByte==7) {MessageSelectionBluetooth.Buffer[Pos]='7';}
			if (TempByte==8) {MessageSelectionBluetooth.Buffer[Pos]='8';}
			if (TempByte==9) {MessageSelectionBluetooth.Buffer[Pos]='9';}
			if (TempByte==10) {MessageSelectionBluetooth.Buffer[Pos]='A';}
			if (TempByte==11) {MessageSelectionBluetooth.Buffer[Pos]='B';}
			if (TempByte==12) {MessageSelectionBluetooth.Buffer[Pos]='C';}
			if (TempByte==13) {MessageSelectionBluetooth.Buffer[Pos]='D';}
			if (TempByte==14) {MessageSelectionBluetooth.Buffer[Pos]='E';}
			if (TempByte==15) {MessageSelectionBluetooth.Buffer[Pos]='F';}
			Pos++;
			//Отсылается младший полубайт
			TempByte=GSM.GPRS_Settings[i]%16;
			if (TempByte==0) {MessageSelectionBluetooth.Buffer[Pos]='0';}
			if (TempByte==1) {MessageSelectionBluetooth.Buffer[Pos]='1';}
			if (TempByte==2) {MessageSelectionBluetooth.Buffer[Pos]='2';}
			if (TempByte==3) {MessageSelectionBluetooth.Buffer[Pos]='3';}
			if (TempByte==4) {MessageSelectionBluetooth.Buffer[Pos]='4';}
			if (TempByte==5) {MessageSelectionBluetooth.Buffer[Pos]='5';}
			if (TempByte==6) {MessageSelectionBluetooth.Buffer[Pos]='6';}
			if (TempByte==7) {MessageSelectionBluetooth.Buffer[Pos]='7';}
			if (TempByte==8) {MessageSelectionBluetooth.Buffer[Pos]='8';}
			if (TempByte==9) {MessageSelectionBluetooth.Buffer[Pos]='9';}
			if (TempByte==10) {MessageSelectionBluetooth.Buffer[Pos]='A';}
			if (TempByte==11) {MessageSelectionBluetooth.Buffer[Pos]='B';}
			if (TempByte==12) {MessageSelectionBluetooth.Buffer[Pos]='C';}
			if (TempByte==13) {MessageSelectionBluetooth.Buffer[Pos]='D';}
			if (TempByte==14) {MessageSelectionBluetooth.Buffer[Pos]='E';}
			if (TempByte==15) {MessageSelectionBluetooth.Buffer[Pos]='F';}
			Pos++;
			}
		}
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции SetGPRSUserPass               ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetGPRSUserPass(void)
	{
	unsigned char StrInstruction[15]={'S','e','t','G','P','R','S','U','s','e','r','P','a','s','s'};
	unsigned char InstructionLength=15; //Размер инструкции с которой производится сравнение
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
		//Проверка блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//Если блок "данные" пустой
	//Проверка строки на корректность
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]<'0') {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if (MessageSelectionBluetooth.Buffer[i]>'f') {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if ((MessageSelectionBluetooth.Buffer[i]>'9')&&(MessageSelectionBluetooth.Buffer[i]<'A')) {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if ((MessageSelectionBluetooth.Buffer[i]>'F')&&(MessageSelectionBluetooth.Buffer[i]<'a')) {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		}
	//Данные преобразуются в символьный вид
	Bluetooth_HexChar_Buf_To_GPRS_User_Pass(MessageInterpretationBluetooth.DataPosStart,MessageInterpretationBluetooth.DataPosEnd);
	Save_GPRS_User_Pass(); //Строка сохраняется в SRAM
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции GetGPRSUserPass              /////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetGPRSUserPass(void)
	{
	unsigned char StrInstruction[15]={'G','e','t','G','P','R','S','U','s','e','r','P','a','s','s'};
	unsigned char InstructionLength=15; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	unsigned char Len_STR; //Размер отсылаемой строки
	unsigned char TempByte; //Временный байт
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Read_GPRS_User_Pass(); //Строка извлекается из SRAM
	Len_STR=strlen(GSM.GPRS_User_Pass); //Размер отсылаемой строки
	if ((Len_STR>0)&&(Len_STR<257)) //Если строка имеет правильный размер
		{
		for (i = 0; i < Len_STR; ++i)
			{
			//Отсылается старший полубайт
			TempByte=GSM.GPRS_User_Pass[i]/16;
			if (TempByte==0) {MessageSelectionBluetooth.Buffer[Pos]='0';}
			if (TempByte==1) {MessageSelectionBluetooth.Buffer[Pos]='1';}
			if (TempByte==2) {MessageSelectionBluetooth.Buffer[Pos]='2';}
			if (TempByte==3) {MessageSelectionBluetooth.Buffer[Pos]='3';}
			if (TempByte==4) {MessageSelectionBluetooth.Buffer[Pos]='4';}
			if (TempByte==5) {MessageSelectionBluetooth.Buffer[Pos]='5';}
			if (TempByte==6) {MessageSelectionBluetooth.Buffer[Pos]='6';}
			if (TempByte==7) {MessageSelectionBluetooth.Buffer[Pos]='7';}
			if (TempByte==8) {MessageSelectionBluetooth.Buffer[Pos]='8';}
			if (TempByte==9) {MessageSelectionBluetooth.Buffer[Pos]='9';}
			if (TempByte==10) {MessageSelectionBluetooth.Buffer[Pos]='A';}
			if (TempByte==11) {MessageSelectionBluetooth.Buffer[Pos]='B';}
			if (TempByte==12) {MessageSelectionBluetooth.Buffer[Pos]='C';}
			if (TempByte==13) {MessageSelectionBluetooth.Buffer[Pos]='D';}
			if (TempByte==14) {MessageSelectionBluetooth.Buffer[Pos]='E';}
			if (TempByte==15) {MessageSelectionBluetooth.Buffer[Pos]='F';}
			Pos++;
			//Отсылается младший полубайт
			TempByte=GSM.GPRS_User_Pass[i]%16;
			if (TempByte==0) {MessageSelectionBluetooth.Buffer[Pos]='0';}
			if (TempByte==1) {MessageSelectionBluetooth.Buffer[Pos]='1';}
			if (TempByte==2) {MessageSelectionBluetooth.Buffer[Pos]='2';}
			if (TempByte==3) {MessageSelectionBluetooth.Buffer[Pos]='3';}
			if (TempByte==4) {MessageSelectionBluetooth.Buffer[Pos]='4';}
			if (TempByte==5) {MessageSelectionBluetooth.Buffer[Pos]='5';}
			if (TempByte==6) {MessageSelectionBluetooth.Buffer[Pos]='6';}
			if (TempByte==7) {MessageSelectionBluetooth.Buffer[Pos]='7';}
			if (TempByte==8) {MessageSelectionBluetooth.Buffer[Pos]='8';}
			if (TempByte==9) {MessageSelectionBluetooth.Buffer[Pos]='9';}
			if (TempByte==10) {MessageSelectionBluetooth.Buffer[Pos]='A';}
			if (TempByte==11) {MessageSelectionBluetooth.Buffer[Pos]='B';}
			if (TempByte==12) {MessageSelectionBluetooth.Buffer[Pos]='C';}
			if (TempByte==13) {MessageSelectionBluetooth.Buffer[Pos]='D';}
			if (TempByte==14) {MessageSelectionBluetooth.Buffer[Pos]='E';}
			if (TempByte==15) {MessageSelectionBluetooth.Buffer[Pos]='F';}
			Pos++;
			}
		}
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции SetFTPFileName               ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetFTPFileName(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','F','T','P','F','i','l','e','N','a','m','e'};
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned int DelimiterData1Pos=0; //Позиция первого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData2Pos=0; //Позиция второго разделителя (запятая) в блоке "Данные"
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
		//Проверка блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//Если блок "данные" пустой
	//Определяются позиции запятых в блоке "Данные"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //Если в блоке "данные" обнаружена первая запятая
		}
	//Проверка количества элементов блока "Данные"
	if (DelimiterData1Pos==0) {Bluetooth_Send_ErrorData(); return;} //Если в блоке "данные" меньше 1 запятой
	//Проверка размера элементов блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //Если первый элемент пустой
	if (DelimiterData2Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //Если второй элемент пустой
	//Проверка первого элемента блока на корректность (строка)
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]<'0') {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if (MessageSelectionBluetooth.Buffer[i]>'f') {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if ((MessageSelectionBluetooth.Buffer[i]>'9')&&(MessageSelectionBluetooth.Buffer[i]<'A')) {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if ((MessageSelectionBluetooth.Buffer[i]>'F')&&(MessageSelectionBluetooth.Buffer[i]<'a')) {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		}
	//Проверка второго элемента блока "Данные" на корректность
	for (i = DelimiterData1Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //Если символ - не является цифрой
	//Данные преобразуются в символьный вид
	Bluetooth_HexChar_Buf_To_FTP_FileName(MessageInterpretationBluetooth.DataPosStart,DelimiterData1Pos-1);
	GSM.FTP_Append=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, MessageInterpretationBluetooth.DataPosEnd); //Период передачи данных
	Save_FTP_FileName(); //Строка сохраняется в SRAM
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции GetFTPFileName              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetFTPFileName(void)
	{
	unsigned char StrInstruction[14]={'G','e','t','F','T','P','F','i','l','e','N','a','m','e'};
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	unsigned char Len_STR; //Размер отсылаемой строки
	unsigned char TempByte; //Временный байт
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Read_FTP_FileName(); //Строка извлекается из SRAM
	Len_STR=strlen(GSM.FTP_File_Name); //Размер отсылаемой строки
	if ((Len_STR>0)&&(Len_STR<257)) //Если строка имеет правильный размер
		{
		for (i = 0; i < Len_STR; ++i)
			{
			//Отсылается старший полубайт
			TempByte=GSM.FTP_File_Name[i]/16;
			if (TempByte==0) {MessageSelectionBluetooth.Buffer[Pos]='0';}
			if (TempByte==1) {MessageSelectionBluetooth.Buffer[Pos]='1';}
			if (TempByte==2) {MessageSelectionBluetooth.Buffer[Pos]='2';}
			if (TempByte==3) {MessageSelectionBluetooth.Buffer[Pos]='3';}
			if (TempByte==4) {MessageSelectionBluetooth.Buffer[Pos]='4';}
			if (TempByte==5) {MessageSelectionBluetooth.Buffer[Pos]='5';}
			if (TempByte==6) {MessageSelectionBluetooth.Buffer[Pos]='6';}
			if (TempByte==7) {MessageSelectionBluetooth.Buffer[Pos]='7';}
			if (TempByte==8) {MessageSelectionBluetooth.Buffer[Pos]='8';}
			if (TempByte==9) {MessageSelectionBluetooth.Buffer[Pos]='9';}
			if (TempByte==10) {MessageSelectionBluetooth.Buffer[Pos]='A';}
			if (TempByte==11) {MessageSelectionBluetooth.Buffer[Pos]='B';}
			if (TempByte==12) {MessageSelectionBluetooth.Buffer[Pos]='C';}
			if (TempByte==13) {MessageSelectionBluetooth.Buffer[Pos]='D';}
			if (TempByte==14) {MessageSelectionBluetooth.Buffer[Pos]='E';}
			if (TempByte==15) {MessageSelectionBluetooth.Buffer[Pos]='F';}
			Pos++;
			//Отсылается младший полубайт
			TempByte=GSM.FTP_File_Name[i]%16;
			if (TempByte==0) {MessageSelectionBluetooth.Buffer[Pos]='0';}
			if (TempByte==1) {MessageSelectionBluetooth.Buffer[Pos]='1';}
			if (TempByte==2) {MessageSelectionBluetooth.Buffer[Pos]='2';}
			if (TempByte==3) {MessageSelectionBluetooth.Buffer[Pos]='3';}
			if (TempByte==4) {MessageSelectionBluetooth.Buffer[Pos]='4';}
			if (TempByte==5) {MessageSelectionBluetooth.Buffer[Pos]='5';}
			if (TempByte==6) {MessageSelectionBluetooth.Buffer[Pos]='6';}
			if (TempByte==7) {MessageSelectionBluetooth.Buffer[Pos]='7';}
			if (TempByte==8) {MessageSelectionBluetooth.Buffer[Pos]='8';}
			if (TempByte==9) {MessageSelectionBluetooth.Buffer[Pos]='9';}
			if (TempByte==10) {MessageSelectionBluetooth.Buffer[Pos]='A';}
			if (TempByte==11) {MessageSelectionBluetooth.Buffer[Pos]='B';}
			if (TempByte==12) {MessageSelectionBluetooth.Buffer[Pos]='C';}
			if (TempByte==13) {MessageSelectionBluetooth.Buffer[Pos]='D';}
			if (TempByte==14) {MessageSelectionBluetooth.Buffer[Pos]='E';}
			if (TempByte==15) {MessageSelectionBluetooth.Buffer[Pos]='F';}
			Pos++;
			}
		}
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	if (GSM.FTP_Append>0) {MessageSelectionBluetooth.Buffer[Pos]='1';} else {MessageSelectionBluetooth.Buffer[Pos]='0';}
	Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции SetSMSSettings               ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetSMSSettings(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','S','M','S','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned int DelimiterData1Pos=0; //Позиция первого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData2Pos=0; //Позиция второго разделителя (запятая) в блоке "Данные"
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
		//Проверка блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//Если блок "данные" пустой
	//Определяются позиции запятых в блоке "Данные"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //Если в блоке "данные" обнаружена первая запятая
		}
	//Проверка количества элементов блока "Данные"
	if (DelimiterData1Pos==0) {Bluetooth_Send_ErrorData(); return;} //Если в блоке "данные" меньше 1 запятой
	//Проверка размера элементов блока "Данные"
	if ((MessageInterpretationBluetooth.DataPosStart+12)!=DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //Если первый элемент не равен 12 символам
	if (DelimiterData2Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //Если второй элемент пустой
	//Проверка первого элемента блока на корректность (строка)
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]<'+') {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		if (MessageSelectionBluetooth.Buffer[i]>'9') {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра
		}
	//Проверка второго элемента блока "Данные" на корректность
	for (i = DelimiterData1Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //Если символ - не является цифрой
	//Считываются данные
	for (i = 0; i < 12; ++i) {GSM.SMSNumber[i]=MessageSelectionBluetooth.Buffer[MessageInterpretationBluetooth.DataPosStart+i];}
	GSM.SMSNumber[12]='\0';
	//Код доступа
	GSM.SMSCode=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, MessageInterpretationBluetooth.DataPosEnd);
	Save_SMS_Settings(); //Строка сохраняется в SRAM
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции GetSMSSettings              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetSMSSettings(void)
	{
	unsigned char StrInstruction[14]={'G','e','t','S','M','S','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Read_SMS_Settings(); //Строка извлекается из SRAM
	for (i = 0; i < 12; ++i) {MessageSelectionBluetooth.Buffer[Pos]=GSM.SMSNumber[i];Pos++;} //Отсылается номер телефона
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++; //Запятая
	Bluetooth_Unsigned_Int_To_Char_Buf(GSM.SMSCode, Pos, Pos+10);Pos+=11; //Отправляется код доступа
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции SetChIDList              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetChIDList(void)
	{
	unsigned char StrInstruction[11]={'S','e','t','C','h','I','D','L','i','s','t'};
	unsigned char InstructionLength=11; //Размер инструкции с которой производится сравнение
	unsigned int i; //Счетчик
	unsigned int j; //Счетчик
	unsigned int StartChID; //Начальная позиция ИД канала
	unsigned int StopChID; //Начальная позиция ИД канала
	//Проверка на соответствие инструкции
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//Проверка блока "Данные"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//Если блок "данные" пустой
	//Проверка первого элемента блока на корректность (строка)
	for (i = MessageInterpretationBluetooth.DataPosStart; i < MessageInterpretationBluetooth.DataPosEnd+1; ++i)
		{
		if (((MessageSelectionBluetooth.Buffer[i]<'0')||(MessageSelectionBluetooth.Buffer[i]>'9'))&&(MessageSelectionBluetooth.Buffer[i]!=',')) {Bluetooth_Send_ErrorData(); return;} //Если символ не цифра и не запятая
		}
	//Очищается лист с ИД каналов
	for (i = 0; i <  64; ++i)
		{
		ChIDList[i]=0;
		}
	//Производится интерпретация данных
	j=0;
	StartChID=MessageInterpretationBluetooth.DataPosStart;
	for (i = MessageInterpretationBluetooth.DataPosStart; i <  MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]==',')
			{
			StopChID=i;
			if (StopChID>StartChID+1)
				{
				ChIDList[j]=Bluetooth_Char_To_Unsigned_Int_Buf(StartChID,StopChID-1); //Элемент листа
				j++;
				}
			StartChID=i+1;
			}
		if (j>63) {Bluetooth_Send_ErrorData(); return;} //Если превышено количество сенсоров
		}
	if ( MessageInterpretationBluetooth.DataPosEnd>StartChID+1)
		{
		ChIDList[j]=Bluetooth_Char_To_Unsigned_Int_Buf(StartChID, MessageInterpretationBluetooth.DataPosEnd); //Элемент листа
		}
	Save_ChIDList(); //сохраняется в SRAM
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции GetChIDList              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetChIDList(void)
	{
	unsigned char StrInstruction[11]={'G','e','t','C','h','I','D','L','i','s','t'};
	unsigned char InstructionLength=11; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	Read_ChIDList(); //Читается из SRAM ChIDList
	Pos=MessageInterpretationBluetooth.DataPosStart;
	for (i = 0; i <  64; ++i)
		{
		if (ChIDList[i]!=0)
			{
			Bluetooth_Unsigned_Int_To_Char_Buf(ChIDList[i], Pos, Pos+10);Pos=Pos+11;
			MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
			}
		}
	Pos--;
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //Отсылается ответ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции DownloadData              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_DownloadData(void)
	{
	unsigned char StrInstruction[12]={'D','o','w','n','l','o','a','d','D','a','t','a'};
	unsigned char InstructionLength=12; //Размер инструкции с которой производится сравнение
	unsigned int pos=0; //Позиция курсора в буфере сообщений
	unsigned int i; //Счетчик
	signed int TempVal;
	TMemoryResult TempResult;
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//В сообщение добавляется результат выполнения инструкции
	for (i = 0x00; i < 0x3000; ++i)
		{
		TempResult=Read_Record(i);
		if (TempResult.FlagMeas!=0xff)
			{
			//Дата
			MessageSelectionBluetooth.Buffer[0]=(TempResult.DD/10)%10 + 0x30; MessageSelectionBluetooth.Buffer[1]=TempResult.DD%10 + 0x30;
			MessageSelectionBluetooth.Buffer[2]='/';
			MessageSelectionBluetooth.Buffer[3]=(TempResult.MM/10)%10 + 0x30; MessageSelectionBluetooth.Buffer[4]=TempResult.MM%10 + 0x30;
			MessageSelectionBluetooth.Buffer[5]='/';
			MessageSelectionBluetooth.Buffer[6]='2';MessageSelectionBluetooth.Buffer[7]='0';MessageSelectionBluetooth.Buffer[8]=(TempResult.YY/10)%10 + 0x30;MessageSelectionBluetooth.Buffer[9]=TempResult.YY%10 + 0x30;
			MessageSelectionBluetooth.Buffer[10]=';';
			MessageSelectionBluetooth.Buffer[11]=(TempResult.hour/10)%10 + 0x30;MessageSelectionBluetooth.Buffer[12]=TempResult.hour%10 + 0x30;
			MessageSelectionBluetooth.Buffer[13]=':';
			MessageSelectionBluetooth.Buffer[14]=(TempResult.min/10)%10 + 0x30;	MessageSelectionBluetooth.Buffer[15]=TempResult.min%10 + 0x30;
			MessageSelectionBluetooth.Buffer[16]=':';
			MessageSelectionBluetooth.Buffer[17]=(TempResult.sec/10)%10 + 0x30;	MessageSelectionBluetooth.Buffer[18]=TempResult.sec%10 + 0x30;
			MessageSelectionBluetooth.Buffer[19]='.';
			MessageSelectionBluetooth.Buffer[20]='0';
			MessageSelectionBluetooth.Buffer[21]=';';
			//ChID
			MessageSelectionBluetooth.Buffer[22]=(TempResult.ChID/1000000000)%10+0x30;
			MessageSelectionBluetooth.Buffer[23]=(TempResult.ChID/100000000)%10+0x30;
			MessageSelectionBluetooth.Buffer[24]=(TempResult.ChID/10000000)%10+0x30;
			MessageSelectionBluetooth.Buffer[25]=(TempResult.ChID/1000000)%10+0x30;
			MessageSelectionBluetooth.Buffer[26]=(TempResult.ChID/100000)%10+0x30;
			MessageSelectionBluetooth.Buffer[27]=(TempResult.ChID/10000)%10+0x30;
			MessageSelectionBluetooth.Buffer[28]=(TempResult.ChID/1000)%10+0x30;
			MessageSelectionBluetooth.Buffer[29]=(TempResult.ChID/100)%10+0x30;
			MessageSelectionBluetooth.Buffer[30]=(TempResult.ChID/10)%10+0x30;
			MessageSelectionBluetooth.Buffer[31]=TempResult.ChID%10+0x30;
			//val
			pos=32;
			MessageSelectionBluetooth.Buffer[pos]=';';pos++;
			TempVal=TempResult.Value*100000;
			if (TempVal<0)
				{
				TempVal=TempVal*-1;
				MessageSelectionBluetooth.Buffer[pos]='-'; pos++;
				}
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/100000000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/10000000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/1000000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/100000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]='.';pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/10000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/1000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/100)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/10)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=TempVal%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=';';pos++;
			//Var
			TempVal=TempResult.Variation*100000;
			if (TempVal<0)
				{
				TempVal=TempVal*-1;
				MessageSelectionBluetooth.Buffer[pos]='-'; pos++;
				}
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/100000000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/10000000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/1000000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/100000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]='.';pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/10000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/1000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/100)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/10)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=TempVal%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=';';pos++;
			//Temp
			TempVal=TempResult.Temperature*100;
			if (TempVal<0)
				{
				TempVal=TempVal*-1;
				MessageSelectionBluetooth.Buffer[pos]='-'; pos++;
				}
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/1000)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/100)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]='.';pos++;
			MessageSelectionBluetooth.Buffer[pos]=(TempVal/10)%10+0x30;pos++;
			MessageSelectionBluetooth.Buffer[pos]=TempVal%10+0x30;pos++;
			//Завершение строки
			MessageSelectionBluetooth.Buffer[pos]=0x0d;pos++;
			MessageSelectionBluetooth.Buffer[pos]=0x0a;pos++;
			//Отправка строки
			MessageSelectionBluetooth.Buffer_Len=pos;
			Bluetooth_Send_Message(); //Отсылается ответ
			}
		}
	MessageSelectionBluetooth.Buffer[0]='E';
	MessageSelectionBluetooth.Buffer[1]='n';
	MessageSelectionBluetooth.Buffer[2]='d';
	//Завершение строки
	MessageSelectionBluetooth.Buffer[3]=0x0d;
	MessageSelectionBluetooth.Buffer[4]=0x0a;
	//Отправка строки
	MessageSelectionBluetooth.Buffer_Len=4;
	Bluetooth_Send_Message(); //Отсылается ответ
	ClearFTPFlags(); //Сбрасываются флаги передачи данных по FTP
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////      Функция производит выполнение инструкции test        /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_Test(void)
	{
	unsigned char StrInstruction[4]={'T','e','s','t'};
	unsigned char InstructionLength=4; //Размер инструкции с которой производится сравнение
	unsigned int Pos=0; //Позиция курсора в строке
	unsigned int i; //Счетчик
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка соответствия адреса
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //Если не совпадает адрес в сообщении с адресом устройства
	//Выполняется тестирование всех систем
	///////////////////////////////////////
	//1. Отправка и прием СМС
	///////////////////////////////////////
	//Отправка СМС с серийным номером
	Start_Led(100);//Функция запуска таймера светодиода
	strcpy(GSM.OutSMS,"Test SMS");
	GSM.OutSMS[0]=(DeviceSerial/10000000000)%10+0x30;
	GSM.OutSMS[0]=(DeviceSerial/1000000000)%10+0x30;
	GSM.OutSMS[1]=(DeviceSerial/100000000)%10+0x30;
	GSM.OutSMS[2]=(DeviceSerial/10000000)%10+0x30;
	GSM.OutSMS[3]=(DeviceSerial/1000000)%10+0x30;
	GSM.OutSMS[4]=(DeviceSerial/100000)%10+0x30;
	GSM.OutSMS[5]=(DeviceSerial/10000)%10+0x30;
	GSM.OutSMS[6]=(DeviceSerial/1000)%10+0x30;
	GSM.OutSMS[7]=(DeviceSerial/100)%10+0x30;
	GSM.OutSMS[8]=(DeviceSerial/10)%10+0x30;
	GSM.OutSMS[9]=(DeviceSerial/1)%10+0x30;
	GSM.OutSMS[10]='\0';
	GSM.Attempt=3; //3 попытки
	GSM.ATErrorFlag=0xff;
	while ((GSM.Attempt>0)&&(GSM.ATErrorFlag!=0))
		{
		GSM_Send_SMS(30000);
		GSM.Attempt--;
		}
	//Прием и анализ всех СМС
	GSM_Read_ALL_SMS(10000);
	//Удаление всех СМС
	GSM_Delete_All_SMS(5000);
	///////////////////////////////////////
	//2. Измерение всех сенсоров
	///////////////////////////////////////
	Start_Led(1000);//Функция запуска таймера светодиода
	MeasCycle();
	///////////////////////////////////////
	//3. Передача данных на ФТП сервер
	///////////////////////////////////////
	Start_Led(100);//Функция запуска таймера светодиода
	GSM_Send_Data_FTP();
	///////////////////////////////////////
	//4. Пероверка часов реального времени
	///////////////////////////////////////
	CyclePeriod=30; //Устанавливается период 30 секунд
	RTC_Set_Next_Cycle_Time();
	SetNameBlooetooth();
	//Выключается питание устройства
	GPIO_ResetBits(Power_Port, Power_Pin); Delay_us(3000000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////      Функция производит установку имени bluetooth модуля    /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetNameBlooetooth(void)
	{
	//Перевод модуля в AT режим
	GPIO_SetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(10000);
	//GPIO_ResetBits(BT_Reset_Port, BT_Reset_Pin);
	Delay_us(10000);
	//GPIO_SetBits(BT_Reset_Port, BT_Reset_Pin);
	//Установка имени устройства
	Delay_us(1000000);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'A');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'T');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'+');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'N');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'A');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'M');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'E');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'=');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'"');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/10000000)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/1000000)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/100000)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/10000)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/1000)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/100)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/10)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/1)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,0x0D);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,0x0A);
	Delay_us(1000000);
	//Перевод модуля в AT режим
	GPIO_ResetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(1000000);
	//Перевод модуля в AT режим
	GPIO_SetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(10000);
	GPIO_ResetBits(BT_Reset_Port, BT_Reset_Pin);
	Delay_us(10000);
	GPIO_SetBits(BT_Reset_Port, BT_Reset_Pin);
	//Установка имени устройства
	Delay_us(1000000);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'A');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'T');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'+');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'N');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'A');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'M');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'E');
	//while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'=');

	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/10000000)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/1000000)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/100000)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/10000)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/1000)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/100)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/10)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,(DeviceSerial/1)%10+0x30);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,0x0D);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,0x0A);
	Delay_us(1000000);
	//Перевод модуля в AT режим
	GPIO_ResetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(1000000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////      Функция Проверяет канал Обмена bluetooth модуля     /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Test_Chanela_Blooetooth(void)
{
	//Перевод модуля в AT режим включеный
	GPIO_SetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(10000);
	//GPIO_ResetBits(BT_Reset_Port, BT_Reset_Pin);
	Delay_us(10000);
	//GPIO_SetBits(BT_Reset_Port, BT_Reset_Pin);
	//Отправка AT команды
	Delay_us(1000000);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'A');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'T');

	Delay_us(10000);
	// инициализирую буфер
	char buffer_RX[2] = {0,};
	while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //Ожидается прибытие байта в буфер bluetooth
	buffer_RX[0]=USART_ReceiveData(USART_BT); //Забирается байт из буфера Usart
	while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //Ожидается прибытие байта в буфер bluetooth
	buffer_RX[1]=USART_ReceiveData(USART_BT); //Забирается байт из буфера Usart

	Delay_us(1000000);
	//Перевод модуля в AT режим сброс
	GPIO_ResetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(1000000);
}

















////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////         Функции системы питания устройства            ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////            Функция производит инициализацию системы питания              /////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Power_Init(void)
	{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//Вывод включения питания устройства
	GPIO_InitStructure.GPIO_Pin = Power_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Power_Port, &GPIO_InitStructure);
	GPIO_SetBits(Power_Port, Power_Pin); //Включается питание устройства
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM_Led, ENABLE); //Тактирование таймера светодиода
	//Вывод питания Bluetooth и GSM модулей
	GPIO_InitStructure.GPIO_Pin = Power_BT_GSM_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Power_BT_GSM_Port, &GPIO_InitStructure);
	//Вывод питания устройств на линии RS485
	GPIO_InitStructure.GPIO_Pin = Power_RS485_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Power_RS485_Port, &GPIO_InitStructure);
	//Вывод светодиода
	GPIO_InitStructure.GPIO_Pin = Led_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(Led_Port, &GPIO_InitStructure);
    //Инициализация таймера светодиода
    TIM_TimeBaseInitTypeDef Timer_InitStructure;
    TIM_TimeBaseStructInit(&Timer_InitStructure);
    Timer_InitStructure.TIM_Prescaler = 8000-1; //Предделитель таймера (частота тиков таймера 1 кГц)
    Timer_InitStructure.TIM_Period = 1000; //Таймер будет срабатывать каждые 1 с (1 Гц)
    Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // пределитель тактирования таймера
    Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; //Направление сччета - на увеличение
    Timer_InitStructure.TIM_RepetitionCounter = 0; //Событие будет возникать каждое переполнение счетчика
    TIM_TimeBaseInit(Timer_Led, &Timer_InitStructure); //Инициализация таймера
    //Конфигурация прерывания таймера светодиода
    TIM_ITConfig(Timer_Led, TIM_IT_Update, ENABLE); //Включить прерывание по переполнению счетчика
	NVIC_SetPriority(Timer_Led_IRQn,Prioritet_LedTimer); //Устанавливается приоритет прерывания
	NVIC_EnableIRQ(Timer_Led_IRQn); //Разрешается обработка прерывания от таймера
	GPIO_ResetBits(Power_RS485_Port, Power_RS485_Pin); //Выключается питание устройств на линии RS485
	//Остановка таймера
	TIM_Cmd(Timer_Led, DISABLE);
	GPIO_SetBits(Led_Port, Led_Pin); //Включить светодиод
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////       Функция производит плавное включение питания GSM и Bluetooth модулей              //////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Power_ON_BT_GSM(void)
	{
	int i;
	for (i = 0; i < 25; ++i)
		{
		GPIO_ToggleBits(Power_BT_GSM_Port, Power_BT_GSM_Pin);
		Delay_us(50);
		}
	GPIO_SetBits(Power_BT_GSM_Port, Power_BT_GSM_Pin);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////     Событие срабатывания таймера светодиода      ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TIM3_IRQHandler(void)
	{
	TIM_ClearITPendingBit(Timer_Led, TIM_IT_Update); //Сброс флага ожидания прерывания
	GPIO_ToggleBits(Led_Port, Led_Pin); //Переключить светодиод
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////         Функция запуска таймера светодиода       ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Start_Led(unsigned int Period) //Период, мс
	{
	TIM_Cmd(Timer_Led, DISABLE);
	//Инициализация таймера
    TIM_TimeBaseInitTypeDef Timer_InitStructure;
    TIM_TimeBaseStructInit(&Timer_InitStructure);
    Timer_InitStructure.TIM_Prescaler = 8000-1; //Предделитель таймера (частота тиков таймера 1 кГц)
    Timer_InitStructure.TIM_Period = Period; //Таймер будет срабатывать каждые Period/1000с
    Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // пределитель тактирования таймера
    Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; //Направление сччета - на увеличение
    Timer_InitStructure.TIM_RepetitionCounter = 0; //Событие будет возникать каждое переполнение счетчика
    TIM_TimeBaseInit(Timer_Led, &Timer_InitStructure); //Инициализация таймера
	TIM_Cmd(Timer_Led, ENABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////      Функция остановки таймера светодиода        ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Stop_Led()
	{
	TIM_Cmd(Timer_Led, DISABLE);
	GPIO_ResetBits(Led_Port, Led_Pin); //Выключить светодиод
	}




















////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////                  Функции для работы с RS485                             ///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////         Функция инициализации RS85 модуля                  ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RS485_Init(void)
	{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART_RS485, ENABLE); //Тактирование USART RS485
	//Вывод RE_DE RS485
	GPIO_InitStructure.GPIO_Pin = RS485_RE_DE_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(RS485_RE_DE_Port, &GPIO_InitStructure);
	//Вывод RS485 (Tx контроллера)
	GPIO_InitStructure.GPIO_Pin = RS485_Tx_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(RS485_Tx_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(RS485_Tx_Port, RS485_Tx_GPIO_PinSource, GPIO_AF_USART_RS485);
	//Вывод RS485 (Rx контроллера)
	GPIO_InitStructure.GPIO_Pin = RS485_Rx_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(RS485_Rx_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(RS485_Rx_Port, RS485_Rx_GPIO_PinSource, GPIO_AF_USART_RS485);
	//Конфигурация USART RS485
	USART_InitStructure.USART_BaudRate = 9600;                                       // Скорость
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // длина пакета 1байт/8бит
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // 1 стоп-бит
	USART_InitStructure.USART_Parity = USART_Parity_No;                              // Без контроля четности
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // Разрешаем прием и передачу
	USART_Init(USART_RS485, &USART_InitStructure);
	USART_Cmd(USART_RS485, ENABLE);
	GPIO_ResetBits(RS485_RE_DE_Port, RS485_RE_DE_Pin); //Включается приемник RS485
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           Передача символа в RS485 модуль                         ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RS485_Send_Char(unsigned char Byte)
	{
	Timeouts.TimeoutRS485_Char=Send_Char_Timeout; //Запускается счетчик таймаута отсылки байта по Usart
	while ((USART_GetFlagStatus(USART_RS485, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutRS485_Char>0)) {}; //Ожидается окончание передачи байта
	USART_SendData(USART_RS485, Byte); //Отсылается символ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           Передача строки в RS485 модуль                         ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RS485_Send_String(void)
	{
	unsigned int i; //Счетчик
	for (i = 0; i < strlen(RS485.Buffer); ++i)
		{
		Timeouts.TimeoutRS485_Char=Send_Char_Timeout;  //Запускается счетчик таймаута отсылки байта по Usart
		while ((USART_GetFlagStatus(USART_RS485, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutRS485_Char>50)) {}; //Ожидается окончание передачи байта
		USART_SendData(USART_RS485,  RS485.Buffer[i]); //Отсылается символ
		}
	while ((USART_GetFlagStatus(USART_RS485, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutRS485_Char>50)) {}; //Ожидается окончание передачи байта
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           Получение результатов измерения                        ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RS485_GetValue_Sensor(unsigned int Timestamp, unsigned int ChId,unsigned int Timeout_ms)
	{
	char TempSTR[2]={' ', '\0'}; //Временная строка из одного символа + завершающий символ
	RS485.CorrectResultFlag=0x00; //Сбрасывается флаг корректности результата
		//Формирование запроса
	strcpy(RS485.Buffer," %/Q/0/");
	//В запрос добавляется Transaction
	RS485.Transaction++;
	RS485.Transaction=RS485.Transaction%10;
	TempSTR[0]=RS485.Transaction+0x30; strcat(RS485.Buffer,TempSTR);
	strcat(RS485.Buffer,"/GetValue/");
	//В запрос добавляется Timetstamp
	TempSTR[0]=(Timestamp/1000000000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(Timestamp/100000000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(Timestamp/10000000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(Timestamp/1000000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(Timestamp/100000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(Timestamp/10000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(Timestamp/1000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(Timestamp/100)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(Timestamp/10)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(Timestamp/1)%10+0x30; strcat(RS485.Buffer,TempSTR);
	//В запрос добавляется ','
	TempSTR[0]=','; strcat(RS485.Buffer,TempSTR);
	//В запрос добавляется ChID
	TempSTR[0]=(ChId/1000000000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(ChId/100000000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(ChId/10000000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(ChId/1000000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(ChId/100000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(ChId/10000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(ChId/1000)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(ChId/100)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(ChId/10)%10+0x30; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=(ChId/1)%10+0x30; strcat(RS485.Buffer,TempSTR);
	//В запрос добавляется "/%" CR LF
	strcat(RS485.Buffer,"/%");
	TempSTR[0]=0x0d; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=0x0a; strcat(RS485.Buffer,TempSTR);
		//Передача Запроса
	GPIO_SetBits(RS485_RE_DE_Port, RS485_RE_DE_Pin); //Включается передатчик RS485
	Timeouts.TimeoutRS485_Instruction=5000; while (Timeouts.TimeoutRS485_Instruction>50)//Ожидается включение передатчика, 5мс
	RS485_Send_String(); //Передача команды
		//Прием Ответа
	GPIO_ResetBits(RS485_RE_DE_Port, RS485_RE_DE_Pin); //Включается приемник RS485
	Timeouts.TimeoutRS485_Instruction=5000; while (Timeouts.TimeoutRS485_Instruction>50)//Ожидается включение приемника, 5мс
	//Сбрасываются флаги и начальные параметры
	RS485.Message_Complete_flag=0x00; //Флаг окончания сборки сообщения
	RS485.Start_Collect_Flag=0x00; //Флаг начала процесса сборки сообщения
	RS485.Buffer_Pos=0x00; //Позиция курсора в буфере
	RS485.Previous_Byte=0x00; //Текущий байт
	RS485.Current_Byte=0x00; //Предыдущий байт
	Timeouts.TimeoutRS485_Instruction=Timeout_ms*1000; //Устанавливается таймаут приема ответа
	//Запускается процесс сборки сообщения
	while (RS485.Message_Complete_flag==0)
		{
		if (Timeouts.TimeoutRS485_Instruction<51) {return;} //Если превышен таймаут ожидания инструкции
		if (RS485.Buffer_Pos>1023) {return;}//Если переполнился буфер
		if (USART_GetFlagStatus(USART_RS485, USART_FLAG_RXNE) != RESET) //Если в приемном буфере USART есть байт
			{
			RS485.Previous_Byte=RS485.Current_Byte; //Запоминается предыдущий принятый байт
			RS485.Current_Byte=USART_ReceiveData(USART_RS485); //Забирается байт из USART
			if (RS485.Start_Collect_Flag!=0) //Если идет процесс сборки сообщения
				{
				RS485.Buffer[RS485.Buffer_Pos]=RS485.Current_Byte; //Заносится текущий байт в буфер сообщения
				RS485.Buffer_Pos++; //Инкрементируется счетчик положения курсора в буфере сообщения
				RS485.Buffer_Len=RS485.Buffer_Pos; //Вычисляется количество данных в буфере сообщения
				if ((RS485.Previous_Byte=='/')&&(RS485.Current_Byte=='%')) {RS485.Message_Complete_flag=0xff;} //Если обнаружен маркер конца сообщения, то останавливается процесс сборки сообщения
				}
			if ((RS485.Previous_Byte=='%')&&(RS485.Current_Byte=='/')) //Если обнаружен маркер начала сообщения
				{
				RS485.Start_Collect_Flag=0xff; //Устанавливается флаг запуска процесса выборки сообщения
				RS485.Buffer[0]=RS485.Previous_Byte; //Заносится предыдущий байт в буфер сообщения
				RS485.Buffer[1]=RS485.Current_Byte; //Заносится текущий байт в буфер сообщения
				RS485.Buffer_Pos=2; //Задается позиция курсора в буфере сообщения
				}
			}
		}
	if (RS485.Message_Complete_flag==0x00) {return;} //Если сообщение не собрано
		//Проверка корректности сообщения
	unsigned int i; //Счетчик
	unsigned int CurrentDelimiterNumber=0; //текущий номер разделителя
	unsigned int TypePosStart=0; //Начало блока "тип сообщения"
	unsigned int TypePosEnd=0; //Конец блока "тип сообщения"
	unsigned int AddressPosStart=0; //Начало блока "адрес"
	unsigned int AddressPosEnd=0; //Конец блока "адрес"
	unsigned int TransactionPosStart=0; //Начало блока "транзакции"
	unsigned int TransactionPosEnd=0; //Конец блока "транзакции"
	unsigned int InstructionPosStart=0; //Начало блока "инструкции"
	unsigned int InstructionPosEnd=0; //Конец блока "инструкции"
	unsigned int DataPosStart=0; //Начало блока "данные"
	unsigned int DataPosEnd=0; //Конец блока "данные"
	//Находятся разделители частей сообщения
	for (i = 0; i < RS485.Buffer_Len; ++i)
		{
		if (RS485.Buffer[i]=='/') //Если обнаружен разделитель
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //Если обнаружен 0-й разделитель
				TypePosStart=i+1; //Запоминается начало блока "тип сообщения"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 1: //Если обнаружен 1-й разделитель
				TypePosEnd=i-1; //Запоминается конец блока "тип сообщения"
				AddressPosStart=i+1; //Запоминается начало блока "адрес"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 2: //Если обнаружен 2-й разделитель
				AddressPosEnd=i-1; //Запоминается конец блока "адрес"
				TransactionPosStart=i+1; //Запоминается начало блока "транзакция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 3: //Если обнаружен 3-й разделитель
				TransactionPosEnd=i-1; //Запоминается конец блока "транзакция"
				InstructionPosStart=i+1; //Запоминается начало блока "инструкция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 4: //Если обнаружен 4-й разделитель
				InstructionPosEnd=i-1; //Запоминается конец блока "инструкция"
				DataPosStart=i+1; //Запоминается начало блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 5: //Если обнаружен5-й разделитель
				DataPosEnd=i-1; //Запоминается конец блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				}
			}
		}
	if (CurrentDelimiterNumber!=6) {return;} //Проверяется количество блоков в сообщении
	//Проверяется размер блоков
	if (TypePosStart>TypePosEnd) {return;}//Если блок "Тип" пустой
	if (AddressPosStart>AddressPosEnd) {return;}//Если блок "Адрес" пустой
	if (TransactionPosStart>TransactionPosEnd) {return;}//Если блок "Транзакция" пустой
	if (InstructionPosStart>InstructionPosEnd) {return;}//Если блок "Инструкция" пустой
	if (DataPosStart>DataPosEnd) {return;}//Если блок "данные" пустой
	//Проверяется корректность блока "тип сообщения"
	if (TypePosEnd!=TypePosStart) {return;}
	if (RS485.Buffer[TypePosStart]!='R') {return;}//Если сообщение - не является ответом
	//Проверяется корректность блока "адрес"
	for (i = AddressPosStart; i <= AddressPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//Если символ - не является цифрой
	//Проверяется корректность блока "транзакция"
	if (TransactionPosStart!=TransactionPosEnd) {return;}//Если размер блока "транзакция" не равен одному символу, то завершается выполнение функции
	if (RS485.Buffer[TransactionPosStart]!=RS485.Transaction+0x30) {return;}//Если номер транзакции не соответствует отправленному
	//Проверяется корректность блока "Инструкция"
	unsigned char StrInstruction[8]={'G','e','t','V','a','l','u','e'};
	unsigned char InstructionLength=8; //Размер инструкции с которой производится сравнение
	if ((InstructionPosEnd-InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = InstructionPosStart; i <= InstructionPosEnd; ++i) {if (RS485.Buffer[i]!=StrInstruction[i-InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка блока "Данные"
	if (DataPosStart>DataPosEnd)  {return;}//Если блок "данные" пустой
		//Анализ принятых данных
	unsigned int DelimiterData1Pos=0; //Позиция первого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData2Pos=0; //Позиция второго разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData3Pos=0; //Позиция третьего разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData4Pos=0; //Позиция четвертого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData5Pos=0; //Позиция пятого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData6Pos=0; //Позиция шестого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData7Pos=0; //Позиция седьмого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData8Pos=0; //Позиция восьмого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData9Pos=0; //Позиция девятого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData10Pos=0; //Позиция десятого разделителя (запятая) в блоке "Данные"
	//Определяются позиции запятых в блоке "Данные"
	for (i = DataPosStart; i <= DataPosEnd; ++i)
		{
		if ((RS485.Buffer[i]==',')&(DelimiterData9Pos!=0)&(DelimiterData10Pos==0)) {DelimiterData10Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((RS485.Buffer[i]==',')&(DelimiterData8Pos!=0)&(DelimiterData9Pos==0)) {DelimiterData9Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((RS485.Buffer[i]==',')&(DelimiterData7Pos!=0)&(DelimiterData8Pos==0)) {DelimiterData8Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((RS485.Buffer[i]==',')&(DelimiterData6Pos!=0)&(DelimiterData7Pos==0)) {DelimiterData7Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((RS485.Buffer[i]==',')&(DelimiterData5Pos!=0)&(DelimiterData6Pos==0)) {DelimiterData6Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((RS485.Buffer[i]==',')&(DelimiterData4Pos!=0)&(DelimiterData5Pos==0)) {DelimiterData5Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((RS485.Buffer[i]==',')&(DelimiterData3Pos!=0)&(DelimiterData4Pos==0)) {DelimiterData4Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((RS485.Buffer[i]==',')&(DelimiterData2Pos!=0)&(DelimiterData3Pos==0)) {DelimiterData3Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos!=0)&(DelimiterData2Pos==0)) {DelimiterData2Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //Если в блоке "данные" обнаружена первая запятая
		}
	//Проверка количества элементов блока "Данные"
	if (DelimiterData10Pos==0) {return;} //Если в блоке "данные" меньше 10 запятых
		//Проверка размера элементов блока "Данные"
	//ChID
	unsigned int InChId=0;
	if (DelimiterData1Pos==DelimiterData2Pos) {return;} //Если второй элемент пустой
	for (i = DelimiterData1Pos+1; i < DelimiterData2Pos; ++i)
		{InChId*=10; InChId+=RS485.Buffer[i]-0x30;}
	if (InChId!=ChId) {return;}
	//Value
	float Znak=1;
	float InValue=0;
	float M=10;
	if (DelimiterData4Pos==DelimiterData3Pos) {return;} //элемент пустой
	for (i = DelimiterData3Pos+1; i < DelimiterData4Pos; ++i)
		{
		if ((RS485.Buffer[i]>='0')&&(RS485.Buffer[i]<='9'))
			{
			if (M==10) {InValue=M*InValue; InValue+=RS485.Buffer[i]-0x30;}
			else { InValue+=(RS485.Buffer[i]-0x30)*M; M*=0.1;}
			}
		if (RS485.Buffer[i]=='.') {M=0.1;}
		if (RS485.Buffer[i]=='-') {Znak=-1;}
		}
	RS485.Val=InValue*Znak;
	//Проверка на OutOfRange
	for (i = DelimiterData3Pos+1; i < DelimiterData4Pos; ++i)
		{
		if ((RS485.Buffer[i]!='0')&&(RS485.Buffer[i]!='1')&&(RS485.Buffer[i]!='2')&&(RS485.Buffer[i]!='3')&&(RS485.Buffer[i]!='4')&&(RS485.Buffer[i]!='5')&&(RS485.Buffer[i]!='6')&&(RS485.Buffer[i]!='7')&&(RS485.Buffer[i]!='8')&&(RS485.Buffer[i]!='9')&&(RS485.Buffer[i]!='.')&&(RS485.Buffer[i]!='-'))
			{
			RS485.Val=9999;
			}
		}
	//Variation
	Znak=1;
	InValue=0;
	M=10;
	if (DelimiterData5Pos==DelimiterData4Pos) {return;} //элемент пустой
	for (i = DelimiterData4Pos+1; i < DelimiterData5Pos; ++i)
		{
		if ((RS485.Buffer[i]>='0')&&(RS485.Buffer[i]<='9'))
			{
			if (M==10) {InValue=M*InValue; InValue+=RS485.Buffer[i]-0x30;}
			else { InValue+=(RS485.Buffer[i]-0x30)*M; M*=0.1;}
			}
		if (RS485.Buffer[i]=='.') {M=0.1;}
		if (RS485.Buffer[i]=='-') {Znak=-1;}
		}
	RS485.Var=InValue*Znak;
	//Temperature
	Znak=1;
	InValue=0;
	M=10;
	if (DelimiterData5Pos==DelimiterData6Pos) {return;} //элемент пустой
	for (i = DelimiterData5Pos+1; i < DelimiterData6Pos; ++i)
		{
		if ((RS485.Buffer[i]>='0')&&(RS485.Buffer[i]<='9'))
			{
			if (M==10) {InValue=M*InValue; InValue+=RS485.Buffer[i]-0x30;}
			else { InValue+=(RS485.Buffer[i]-0x30)*M; M*=0.1;}
			}
		if (RS485.Buffer[i]=='.') {M=0.1;}
		if (RS485.Buffer[i]=='-') {Znak=-1;}
		}
	RS485.Temp=InValue*Znak;
	RS485.CorrectResultFlag=0xff;
	}




















////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////           Функции для работы с GSM модулем            ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////           Функция инициализации GSM модуля                 ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Init(void)
	{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//Тактирование USART GSM
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART_GSM, ENABLE);
	//RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GSM_Rx_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GSM_Rx_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(GSM_Rx_Port, GSM_Rx_PinSource, GPIO_AF_USART_GSM);
	//Tx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GSM_Tx_Pin;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GSM_Tx_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(GSM_Tx_Port, GSM_Tx_PinSource, GPIO_AF_USART_GSM);
	//Конфигурация USART GSM
	USART_InitStructure.USART_BaudRate = 9600;                                       // Скорость
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // длина пакета 1байт/8бит
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // 1 стоп-бит
	USART_InitStructure.USART_Parity = USART_Parity_No;                              // Без контроля четности
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // Разрешаем прием и передачу
	USART_Init(USART_GSM, &USART_InitStructure);
	USART_Cmd(USART_GSM, ENABLE);
	//Таймер приема данных от GSM модуля
	TIM_TimeBaseInitTypeDef Timer_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM_ReciveGSM,ENABLE); //Тактирование таймера приемопередатчика
	TIM_TimeBaseStructInit(&Timer_InitStructure); //Заполняются поля структуры значениями поумолчанию
	Timer_InitStructure.TIM_Prescaler = 8; //Выставляется предделитель
	Timer_InitStructure.TIM_Period = 100; //Выставляется период срабатывания таймера 100мкс
	TIM_TimeBaseInit(Timer_ReciveGSM, &Timer_InitStructure); //Инициализация таймера
	TIM_ITConfig(Timer_ReciveGSM, TIM_IT_Update, ENABLE); //Настраиваем таймер для генерации прерывания по обновлению (переполнению)
	TIM_Cmd(Timer_ReciveGSM, ENABLE); //Запускается таймер
	NVIC_SetPriority(Timer_ReciveGSM_IRQn,PrioritetReciveGSM); //Устанавливается приоритет прерывания
	NVIC_EnableIRQ(Timer_ReciveGSM_IRQn); //Разрешаем соответствующее прерывание
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////           Функция, вызываемая по прерыванию таймера приемника байта от GSM модуля           //////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TIM5_IRQHandler(void)
	{
	if (USART_GetFlagStatus(USART_GSM,USART_FLAG_RXNE) != RESET) //Проверяется наличие байта в приемном буфере GSM
		{
		GSM.UsartInByte=USART_ReceiveData(USART_GSM); //Забирается байт из буфера Usart
		GSM.UsartRxNE=0xff; //Устанавливается флаг наличия в буфере байта
		USART_SendData(USART_BT,GSM.UsartInByte);
		}
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); //Очищается бит прерывания
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           Прием символа от GSM модуля                           //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char GSM_Recive_Char(void)
	{
	GSM.UsartRxNE=0x00; //Сбрасывается флаг
	return GSM.UsartInByte;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           Передача символа в GSM модуль                         //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_Char(unsigned char Byte)
	{
	Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //Запускается счетчик таймаута отсылки байта по Usart
	while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //Ожидается окончание передачи байта
	USART_SendData(USART_GSM, Byte); //Отсылается символ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////             Передача строки из GSM.Buffer в GSM модуль                          //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_String(void)
	{
	unsigned int i; //Счетчик
	for (i = 0; i < strlen(GSM.Buffer); ++i)
		{
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //Запускается счетчик таймаута отсылки байта по Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //Ожидается окончание передачи байта
		USART_SendData(USART_GSM,  GSM.Buffer[i]); //Отсылается символ
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////           Передача AT команды из GSM.Buffer в GSM модуль                        //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_AT(void)
	{
	unsigned int i; //Счетчик
	for (i = 0; i < strlen(GSM.Buffer); ++i)
		{
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //Запускается счетчик таймаута отсылки байта по Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //Ожидается окончание передачи байта
		USART_SendData(USART_GSM, GSM.Buffer[i]); //Отсылается символ
		}
	GSM_Send_Char(0x0d);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                       Функция получения уровня сигнала GSM                      //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Get_Signal_Quality(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="+csq:"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	//Отправляется запрос на получение уровня сигнала
	strcpy(GSM.Buffer,"at+csq"); GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	//Принимаются параметры
	GSM.SQ_RSSI=0; //Уровень сигнала
	while ((Byte!=',')&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //Принимается байт
			if ((Byte >= '0') && (Byte <= '9')) {GSM.SQ_RSSI*=10; GSM.SQ_RSSI+=Byte-0x30;}  //Если символ - цифра
			}
		}
	GSM.SQ_BER=0; //Процент ошибок
	while ((Byte!=0x0d)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if (Byte >= '0' && Byte <= '9') {GSM.SQ_BER*=10; GSM.SQ_BER+=Byte-0x30;} //Если символ - цифра
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                Функция получения статуса регистрации в сети GSM                 //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Get_Network_Registration_Status(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="+creg:"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	//Отправляется запрос на получение статуса регистрации в сети
	strcpy(GSM.Buffer,"at+creg?"); GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	//Принимаются параметры
	GSM.REG_MODE=0; //Режим регистрации в сети оператора
	while ((Byte!=',')&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //Принимается байт
			if ((Byte >= '0') && (Byte <= '9')) {GSM.REG_MODE*=10; GSM.REG_MODE+=Byte-0x30;}  //Если символ - цифра
			}
		}
	GSM.REG_STAT=0; //Статус регистрации в сети
	while ((Byte!=0x0d)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if (Byte >= '0' && Byte <= '9') {GSM.REG_STAT*=10; GSM.REG_STAT+=Byte-0x30;} //Если символ - цифра
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                      Функция получения имени оператора GSM                      //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Get_Name_Operator(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="+cops:"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	unsigned char i=0; //Счетчик
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	for (i = 0; i < 16; ++i) {GSM.Name_Operator[i]=' ';} //Очищается имя оператора
	//Отправляется запрос на получение имени оператора
	strcpy(GSM.Buffer,"at+cops?"); GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	//Принимается имя оператора
	i=0;
	while ((Byte!=0x22)&&(Timeouts.TimeoutGSM_AT>0)) //Производится поиск первой кавычки
		{
		if (GSM.UsartRxNE != RESET) {Byte=GSM_Recive_Char();}//Принимается символ
		if (Timeouts.TimeoutGSM_AT<=0) {return;}
		}
	Byte=0x00;
	while ((Byte!=0x22)&&(Timeouts.TimeoutGSM_AT>0)&&(i<16))  //Считывается имя оператора до 2-й кавычки
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if (Byte!=0x22) {GSM.Name_Operator[i]=Byte; i++;}

			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                          Функция получения IMEI GSM модуля                      //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Get_IMEI(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	unsigned char i=0; //Счетчик
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	for (i = 0; i < 16; ++i) {GSM.IMEI[i]=' ';} //Очищается IMEI
	//Отправляется запрос на получение IMEI
	strcpy(GSM.Buffer,"at+cgsn"); GSM_Send_AT(); //Передача AT команды
	//Принимается IMEI
	i=0;
	while ((Timeouts.TimeoutGSM_AT>0)&&(i<16))  //Считывается IMEI
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if (Byte >= '0' && Byte <= '9') {GSM.IMEI[i]=Byte; i++;} //Если символ - цифра
			}
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                            Функция закрытия сессии GPRS                         //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_GPRS_Close(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="ok"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	//Отправляется запрос на получение статуса регистрации в сети
	strcpy(GSM.Buffer,"at#gprs=0"); GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                             Функция задания PDP контекста                       //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_GPRS_Set_PDP(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="ok"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	//Отправляется запрос на получение статуса регистрации в сети
	strcpy(GSM.Buffer,"at+cgdcont=1,");
	strcat(GSM.Buffer,"\"IP\",");
	strcat(GSM.Buffer,GSM.GPRS_Settings);
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                              Функция открытия сессии GPRS                       //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_GPRS_Open(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="ok"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	//Отправляется запрос на открытие сессии GPRS
	strcpy(GSM.Buffer,"at#sgact=1,1,");
	strcat(GSM.Buffer,GSM.GPRS_User_Pass);
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////         Функция установки таймаута ожидания ответа от FTP сервера               //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_TO(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="ok"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	//Отправляется запрос на установку таймаута 50 секунд
	strcpy(GSM.Buffer,"AT#FTPTO=500");
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                        Функция IP адреса сервера по DNS                         //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_DNS_Get_IP(unsigned int Timeout_ms)
	{
	char Temp_FTP_Settings[256]; //Настройки FTP соединеня -  "server:port","username","password",mode
	unsigned char StartServerName=0;
	unsigned char EndServerName=0;
	char Temp_Server_Name[256]; //server
	unsigned char EndTempServerName; //
	unsigned char DNSFlag=0; //0-IP, 1-DNS
	unsigned int i=0; //Счетчик
	unsigned int j=0;
	//Определение положения имени сервера в строке
	for (i = 0; i < 255; ++i)
		{
		if ((StartServerName!=0)&&(EndServerName==0)&&((GSM.FTP_Settings[i]==0x22)||(GSM.FTP_Settings[i]==0x3a)))	{EndServerName=i-1;}
		if ((StartServerName==0)&&(GSM.FTP_Settings[i]==0x22))	{StartServerName=i+1;}
		}
	//Определяется форма записи адреса сервера
	if (EndServerName<StartServerName) {return;}
	for (i = StartServerName; i <= EndServerName; ++i)
		{
		if ((GSM.FTP_Settings[i]<0x2E)||(GSM.FTP_Settings[i]>0x39)||(GSM.FTP_Settings[i]==0x2F)) {DNSFlag=0xff;}
		Temp_Server_Name[j]=GSM.FTP_Settings[i];
		j++;
		}
	Temp_Server_Name[j]='\0'; //Добавляется завершающий символ
	if (DNSFlag==0) {return;} //Если в строке указан IP адрес, то дальнейших действий не требуется
	//Отправляется Запрос на получение IP по DNS
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]=","; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	strcpy(GSM.Buffer,"at#qdns=");
	strcat(GSM.Buffer,Temp_Server_Name);
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	//Принимается IP адрес
	while ((Byte!=0x22)&&(Timeouts.TimeoutGSM_AT>0))  //Производится поиск первой кавычки
		{
		if (GSM.UsartRxNE != RESET) {Byte=GSM_Recive_Char();}//Принимается символ
		if (Timeouts.TimeoutGSM_AT<=0) {return;}
		}
	i=0;
	Byte=0x00;
	while ((Byte!=0x22)&&(Timeouts.TimeoutGSM_AT>0)&&(i<256))  //Считывается IP до 2-й кавычки
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte!=0x2F)&&(Byte<0x3A)&&(Byte>0x2D))
				{
				Temp_Server_Name[i]=Byte;
				EndTempServerName=i;
				i++;
				}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0)
		{GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	else
		{return;}
	//Вносятся изменения в строку настроек FTPSettings
	Temp_FTP_Settings[0]=0x22; //Первая кавычка
	for (i = 0; i <= EndTempServerName; ++i) {Temp_FTP_Settings[i+1]=Temp_Server_Name[i];} //IPадрес
	i=EndTempServerName+2;
	j=EndServerName+1;
	while ((i<256)&&(j<256)) {Temp_FTP_Settings[i]=GSM.FTP_Settings[j]; i++; j++;} //Дописывается строка
	for (i = 0; i <256 ; ++i) {GSM.FTP_Settings[i]=Temp_FTP_Settings[i];} //Из временной строки данные переносятся в постоянную
	}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                  Функция открытия соединения с FTP сервером                     //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Open(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="ok"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	//Отправляется запрос на установку соединения с FTP сервером
	strcpy(GSM.Buffer,"at#ftpopen=");
	strcat(GSM.Buffer,GSM.FTP_Settings);
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                  Функция закрытия соединения с FTP сервером                     //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Close(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="ok"; //Ключевое слово, содержащееся в ответе перед результатами
	char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	//Отправляется запрос на установку соединения с FTP сервером
	strcpy(GSM.Buffer,"at#ftpclose");
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////              Функция открытия установки типа передачи по FTP                  //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Type(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="ok"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	//Отправляется запрос на установку типа передачи данных по FTP (BIN)
	strcpy(GSM.Buffer,"at#ftptype=0");
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                              Функция открытия файла FTP                         //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Open_File(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="ok"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	//Отправляется запрос открыте файла
	if (GSM.FTP_Append)
		{
		strcpy(GSM.Buffer,"at#ftpapp=");
		}
	else
		{
		strcpy(GSM.Buffer,"at#ftpput=");
		}
	strcat(GSM.Buffer,GSM.FTP_File_Name);
	strcat(GSM.Buffer,",1"); //Командный режим
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                              Функция закрытия файла FTP                         //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Close_File(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
		char ReplySTR[]="> "; //Ключевое слово, содержащееся в ответе перед результатами
		unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
		unsigned char ReplySTRIndex=0;
		Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
		GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
		//Отправляется запрос открыте файла
		GSM_Send_Char('a');
		GSM_Send_Char('t');
		GSM_Send_Char('#');
		GSM_Send_Char('f');
		GSM_Send_Char('t');
		GSM_Send_Char('p');
		GSM_Send_Char('a');
		GSM_Send_Char('p');
		GSM_Send_Char('p');
		GSM_Send_Char('e');
		GSM_Send_Char('x');
		GSM_Send_Char('t');
		GSM_Send_Char('=');
		GSM_Send_Char('0');
		GSM_Send_Char('0');
		GSM_Send_Char('2');
		GSM_Send_Char(',');
		GSM_Send_Char('1');
		GSM_Send_Char(0x0d);
		//Поиск ключевого слова в ответе GSM модуля
		while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
			{
			if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
				{
				Byte=GSM_Recive_Char(); //Принимается символ
				if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
				if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
				}
			}
		if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
		if (GSM.ATErrorFlag!=0x00) {return;}
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //Запускается счетчик таймаута отсылки байта по Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //Ожидается окончание передачи байта
		USART_SendData(USART_GSM,  0x0d); //Отсылается символ
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //Запускается счетчик таймаута отсылки байта по Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //Ожидается окончание передачи байта
		USART_SendData(USART_GSM,  0x0a); //Отсылается символ
		GSM_Send_Char(0x0d);
		strcpy(ReplySTR,"ok"); //Ключевое слово, содержащееся в ответе перед результатами
		ReplySTRLen=strlen(ReplySTR); //Размер строки
		ReplySTRIndex=0;
		//Поиск ключевого слова в ответе GSM модуля
		while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
			{
			if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
				{
				Byte=GSM_Recive_Char(); //Принимается символ
				if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
				if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
				}
			}
		if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                       Функция отправки строки на FTP сервер                    //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Send_string(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="> "; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	unsigned int i; //Счетчик
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
	//Отправляется запрос открыте файла
	GSM_Send_Char('a');
	GSM_Send_Char('t');
	GSM_Send_Char('#');
	GSM_Send_Char('f');
	GSM_Send_Char('t');
	GSM_Send_Char('p');
	GSM_Send_Char('a');
	GSM_Send_Char('p');
	GSM_Send_Char('p');
	GSM_Send_Char('e');
	GSM_Send_Char('x');
	GSM_Send_Char('t');
	GSM_Send_Char('=');
	GSM_Send_Char(((strlen(GSM.FTP_Out_String)+2)/100)%10+0x30);
	GSM_Send_Char(((strlen(GSM.FTP_Out_String)+2)/10)%10+0x30);
	GSM_Send_Char((strlen(GSM.FTP_Out_String)+2)%10+0x30);
	GSM_Send_Char(',');
	GSM_Send_Char('0');
	GSM_Send_Char(0x0d);
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	if (GSM.ATErrorFlag!=0x00) {return;}
	for (i = 0; i < strlen(GSM.FTP_Out_String); ++i)
		{
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //Запускается счетчик таймаута отсылки байта по Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //Ожидается окончание передачи байта
		USART_SendData(USART_GSM,  GSM.FTP_Out_String[i]); //Отсылается символ
		}
	GSM_Send_Char(0x0d);
	GSM_Send_Char(0x0a);
	strcpy(ReplySTR,"ok"); //Ключевое слово, содержащееся в ответе перед результатами
	ReplySTRLen=strlen(ReplySTR); //Размер строки
	ReplySTRIndex=0;
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           Функция подключения к сети GSM                        //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Connect(void)
	{
	Delay_us(5000000); //Задержка 5с
	GSM.Connected=0x00;
	GSM.REG_STAT=0x00;
	GSM.ATErrorFlag=0xff;
	//Ожидание соединения с сетью
	GSM.Attempt=30; //30 попыток
	while ((GSM.REG_STAT!=1)&&(GSM.Attempt!=0))
		{
		Delay_us(2000000); //Задержка 2с
		GSM_Get_Network_Registration_Status(2000); //Запрос статуса соединения с сетью
		Timeouts.TimeoutGSM_AT=0;
		if (GSM.ATErrorFlag!=0x00) {GSM.REG_STAT=0x00;}
		GSM.Attempt--;
		}
	if (GSM.REG_STAT==1) {GSM.Connected=0xff;} //Если произошла регистрация в сети
	Delay_us(5000000);
	//Получение уровня сигнала
	GSM.Attempt=3; //3 попытки
	GSM.ATErrorFlag=0xff;
	while ((GSM.Attempt!=0)&&(GSM.ATErrorFlag!=0x00))
		{
		Delay_us(1000000); //Задержка 1с
		GSM_Get_Signal_Quality(2000); //Запрос уровня сигнала
		Timeouts.TimeoutGSM_AT=0;
		GSM.Attempt--;
		}
	//Получение имени оператора
	GSM.Attempt=3; //3 попытки
	GSM.ATErrorFlag=0xff;
	while ((GSM.Attempt!=0)&&(GSM.ATErrorFlag!=0x00))
		{
		Delay_us(1000000); //Задержка 1с
		GSM_Get_Name_Operator(2000); //Запрос имени оператора
		Timeouts.TimeoutGSM_AT=0;
		GSM.Attempt--;
		}
	//Получение IMEI
	Delay_us(1000000); //Задержка 1с
	GSM_Get_IMEI(2000);
	GSM.Attempt=3; //3 попытки
	GSM.ATErrorFlag=0xff;
	while ((GSM.Attempt!=0)&&(GSM.ATErrorFlag!=0x00))
		{
		Delay_us(1000000); //Задержка 1с
		GSM_Get_Signal_Quality(2000); //Запрос уровня сигнала
		Timeouts.TimeoutGSM_AT=0;
		GSM.Attempt--;
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           Функция подключения к GPRS                        //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_GPRS_Connect(void)
	{
	Delay_us(5000000); //Задержка 5с
	GSM.GPRS_Connected=0x00;
	//Открытие GPRS сеанса
	GSM.Attempt=3; //3 попытоки
	while ((GSM.GPRS_Connected==0x00)&&(GSM.Attempt!=0))
		{
		Delay_us(2000000); //Задержка 2с
		GSM_GPRS_Close(5000); //Закрыть сессию GPRS
		Delay_us(2000000); //Задержка 2с
		GSM_GPRS_Set_PDP(5000); //Настройка PDP
		if (GSM.ATErrorFlag==0x00)
			{
			Delay_us(2000000); //Задержка 2с
			GSM_GPRS_Open(60000); //Открыть сессию GPRS
			if (GSM.ATErrorFlag==0x00) {GSM.GPRS_Connected=0xff;}
			}
		GSM.Attempt--;
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           Функция подключения к FTP серверу                     //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Connect(void)
	{
	Delay_us(5000000); //Задержка 5с
	GSM.FTP_Connected=0x00;
	//Открытие GPRS сеанса
	GSM.Attempt=3; //3 попытки
	while ((GSM.FTP_Connected==0x00)&&(GSM.Attempt!=0))
		{
		Delay_us(2000000); //Задержка 2с
		GSM_FTP_TO(5000); //Задать таймаут 50 с
		Delay_us(2000000); //Задержка 2с
		GSM_DNS_Get_IP(10000); //Определить IP
		Delay_us(2000000); //Задержка 2с
		GSM_FTP_Open(60000); //Соединиться с FTP сервером
		if (GSM.ATErrorFlag==0x00)
			{
			Delay_us(2000000); //Задержка 2с
			GSM_FTP_Type(5000); //Задать тип FTP соеднения ASCII
			if (GSM.ATErrorFlag==0x00)
				{
				Delay_us(2000000); //Задержка 2с
				GSM_FTP_Open_File(10000);
				if (GSM.ATErrorFlag==0x00) {GSM.FTP_Connected=0xff;}
				}
			}
		GSM.Attempt--;
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           Функция передачи данных на FTP сервер                 //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_Data_FTP(void)
	{

	FTPInProgress=0xff; //Устанавливается флаг процесса передачи данных

	TMemoryResult TempResult;
	unsigned char FTPOKFlag=0;
	unsigned int i;
	signed int TempVal;
	unsigned char pos;
	TempResult.YY=DateTime.YY;
	TempResult.MM=DateTime.MM;
	TempResult.DD=DateTime.DD;
	TempResult.hour=DateTime.hour;
	TempResult.min=DateTime.min;
	TempResult.sec=DateTime.sec;
	//Инициализация GSM и Bluetooth модуля
	GPIO_ResetBits(Power_RS485_Port, Power_RS485_Pin); //Отключается питание на линии RS485
	Delay_us(2000000); //Задержка 2 с
	//Устанавливается соединение и начинается передача данных
	GSM_Connect(); //Ожидается подключение к сети GSM
	GSM_GPRS_Connect(); //Ожидается подключение к сети GPRS
	if (GSM.GPRS_Connected)
		{
		GSM_FTP_Connect();
		if (GSM.FTP_Connected)
			{
			FTPOKFlag=0xff;//Устанавливается флаг успешной передачи данных по ftp
				//Добавляются строки с уровнем сигнала GSM
			//Канал8 (SQ_RSSI)
			TempResult.ChID=DeviceSerial*100+8;
			TempResult.Value=GSM.SQ_RSSI;
			TempResult.Variation=GSM.SQ_RSSI;
			TempResult.Temperature=MeasSystem.Temperature;
			Save_Record(TempResult);
			//Канал9 (TimeFTPSession)
			TempResult.ChID=DeviceSerial*100+9;
			TempResult.Value=Timeouts.TimeFTPSession/1000000; //Секунды
			TempResult.Variation=GSM.SQ_BER;
			TempResult.Temperature=MeasSystem.Temperature;
			Save_Record(TempResult);
				//Передача измеренных значений
			for (i = 0x00; i < 0x3000; ++i)
				{
				//Если осталось меньше минуты до окончания передачи данных
				if (Timeouts.ForceRestart<60000000)
					{
					ClearFTPFlags(); //Очистка флагов передачи данных по FTP
					}

				TempResult=Read_Record(i);
				if ((TempResult.FlagMeas!=0xff)&(TempResult.FlagFTP==0xff))
					{
					//Дата
					GSM.FTP_Out_String[0]=(TempResult.DD/10)%10 + 0x30;GSM.FTP_Out_String[1]=TempResult.DD%10 + 0x30;
					GSM.FTP_Out_String[2]='/';
					GSM.FTP_Out_String[3]=(TempResult.MM/10)%10 + 0x30;GSM.FTP_Out_String[4]=TempResult.MM%10 + 0x30;
					GSM.FTP_Out_String[5]='/';
					GSM.FTP_Out_String[6]='2';GSM.FTP_Out_String[7]='0';GSM.FTP_Out_String[8]=(TempResult.YY/10)%10 + 0x30;GSM.FTP_Out_String[9]=TempResult.YY%10 + 0x30;
					GSM.FTP_Out_String[10]=';';
					GSM.FTP_Out_String[11]=(TempResult.hour/10)%10 + 0x30;GSM.FTP_Out_String[12]=TempResult.hour%10 + 0x30;
					GSM.FTP_Out_String[13]=':';
					GSM.FTP_Out_String[14]=(TempResult.min/10)%10 + 0x30;	GSM.FTP_Out_String[15]=TempResult.min%10 + 0x30;
					GSM.FTP_Out_String[16]=':';
					GSM.FTP_Out_String[17]=(TempResult.sec/10)%10 + 0x30;	GSM.FTP_Out_String[18]=TempResult.sec%10 + 0x30;
					GSM.FTP_Out_String[19]='.';
					GSM.FTP_Out_String[20]='0';
					GSM.FTP_Out_String[21]=';';
					//ChID
					GSM.FTP_Out_String[22]=(TempResult.ChID/1000000000)%10+0x30;
					GSM.FTP_Out_String[23]=(TempResult.ChID/100000000)%10+0x30;
					GSM.FTP_Out_String[24]=(TempResult.ChID/10000000)%10+0x30;
					GSM.FTP_Out_String[25]=(TempResult.ChID/1000000)%10+0x30;
					GSM.FTP_Out_String[26]=(TempResult.ChID/100000)%10+0x30;
					GSM.FTP_Out_String[27]=(TempResult.ChID/10000)%10+0x30;
					GSM.FTP_Out_String[28]=(TempResult.ChID/1000)%10+0x30;
					GSM.FTP_Out_String[29]=(TempResult.ChID/100)%10+0x30;
					GSM.FTP_Out_String[30]=(TempResult.ChID/10)%10+0x30;
					GSM.FTP_Out_String[31]=TempResult.ChID%10+0x30;
					//val
					pos=32;
					GSM.FTP_Out_String[pos]=';';pos++;
					TempVal=TempResult.Value*100000;
					if (TempVal<0)
						{
						TempVal=TempVal*-1;
						GSM.FTP_Out_String[pos]='-'; pos++;
						}
						GSM.FTP_Out_String[pos]=(TempVal/100000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/1000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/100000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]='.';pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/1000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/100)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=TempVal%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=';';pos++;
					//Var
					TempVal=TempResult.Variation*100000;
					if (TempVal<0)
						{
						TempVal=TempVal*-1;
						GSM.FTP_Out_String[pos]='-'; pos++;
						}
					GSM.FTP_Out_String[pos]=(TempVal/100000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/1000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/100000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]='.';pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/1000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/100)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=TempVal%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=';';pos++;
					//Temp
					TempVal=TempResult.Temperature*100;
					if (TempVal<0)
						{
						TempVal=TempVal*-1;
						GSM.FTP_Out_String[pos]='-'; pos++;
						}
					GSM.FTP_Out_String[pos]=(TempVal/1000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/100)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]='.';pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=TempVal%10+0x30;pos++;
					//Завершение строки
					GSM.FTP_Out_String[pos]='\0';pos++;
					//Отправка строки
					GSM.Attempt=3; GSM.ATErrorFlag=0xff;
					while ((GSM.ATErrorFlag!=0x00)&&(GSM.Attempt!=0))
						{
						Delay_us(200000); //Задержка 200 мс
						GSM_FTP_Send_string(3000); //Отправка строки (таймаут 3 секунды)
						GSM.Attempt--;
						}
					}
				}
			//Закрывается файл
			GSM.Attempt=3; GSM.ATErrorFlag=0xff;
			while ((GSM.ATErrorFlag!=0x00)&&(GSM.Attempt!=0))
				{
				Delay_us(200000); //Задержка 200 мс
				GSM_FTP_Close_File(10000); //Закрытие файла (таймаут 10 секунд)
				GSM.Attempt--;
				}
			//Закрывается соединение
			Delay_us(1000000); //Задержка 1000 мс
			GSM_FTP_Close(10000); //Закрытие соединения (таймаут 10 секунд)
			Delay_us(1000000); //Задержка 1000 мс
			}
		}
	//Проверка успешности передачи данных
	if (FTPOKFlag!=0) //Если данные успешно переданы
		{
		ClearFTPFlags(); //Очистка флагов передачи данных по FTP
		}

	FTPInProgress=0x00; //Сбрасывается флаг процесса передачи данных
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                          Функция передачи SMS сообщения из строки GSM.OutSMS          ////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_SMS(unsigned int Timeout_ms)
	{
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="ok"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	unsigned int i;
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
		//Установить текстовый формат сообщений (AT+CMGF=1)
	strcpy(GSM.Buffer,"AT+CMGF=1");
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>50))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	Delay_us(200000); //Задержка 200мс
	if (Timeouts.TimeoutGSM_AT<51) {return;}
	//Проверка номера телефона
	for (i = 0; i < 11; ++i)
		{
		if (GSM.SMSNumber[i]<'+') {return;} //Если символ не цифра
		if (GSM.SMSNumber[i]>'9') {return;} //Если символ не цифра
		}
	//Команда на отправку сообщения
	strcpy(ReplySTR,"> ");
	strcpy(GSM.Buffer,"AT+CMGS=");
	strcat(GSM.Buffer,GSM.SMSNumber); //Задается номер телефона
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT<51) {return;}
	Delay_us(200000); //Задержка 200мс
		//Отправка сообщения
	strcpy(ReplySTR,"ok");
	for (i = 0; i < strlen(GSM.OutSMS); ++i)
		{
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //Запускается счетчик таймаута отсылки байта по Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //Ожидается окончание передачи байта
		USART_SendData(USART_GSM,  GSM.OutSMS[i]); //Отсылается символ
		Delay_us(20000); //Задержка 20 мс
		}
	Delay_us(200000); //Задержка 200мс
	//Отсылка завершающего символа (Ctrl+Z)
	Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //Запускается счетчик таймаута отсылки байта по Usart
	while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //Ожидается окончание передачи байта
	USART_SendData(USART_GSM,  0x1A); //Отсылается символ
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>50) {GSM.ATErrorFlag=0x00;} //Если не превышен таймаут выполнения AT команды, то сбрасывается флаг ошибки AT команды
	}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////                          Функция передачи SMS сообщения в случае открытия ящика                  ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_Alarm_SMS(void)
	{
	//Отправка СМС
	GSM.OutSMS[0]=(DeviceSerial/10000000000)%10+0x30;
	GSM.OutSMS[0]=(DeviceSerial/1000000000)%10+0x30;
	GSM.OutSMS[1]=(DeviceSerial/100000000)%10+0x30;
	GSM.OutSMS[2]=(DeviceSerial/10000000)%10+0x30;
	GSM.OutSMS[3]=(DeviceSerial/1000000)%10+0x30;
	GSM.OutSMS[4]=(DeviceSerial/100000)%10+0x30;
	GSM.OutSMS[5]=(DeviceSerial/10000)%10+0x30;
	GSM.OutSMS[6]=(DeviceSerial/1000)%10+0x30;
	GSM.OutSMS[7]=(DeviceSerial/100)%10+0x30;
	GSM.OutSMS[8]=(DeviceSerial/10)%10+0x30;
	GSM.OutSMS[9]=(DeviceSerial/1)%10+0x30;
	GSM.OutSMS[10]='-';
	GSM.OutSMS[11]='O';
	GSM.OutSMS[12]='P';
	GSM.OutSMS[13]='E';
	GSM.OutSMS[14]='N';
	GSM.OutSMS[15]='\0';
	GSM.Attempt=2; //2 попытки
	GSM.ATErrorFlag=0xff;
	while ((GSM.Attempt>0)&&(GSM.ATErrorFlag!=0))
		{
		GSM_Send_SMS(30000);
		GSM.Attempt--;
		}
	}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////                       Функция передачи данных на FTP сервер в случае открытия ящика              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_Alarm_FTP(void)
	{
	TMemoryResult TempResult;
	unsigned char FTPOKFlag=0;
	unsigned int i;
	signed int TempVal;
	unsigned char pos;
	Start_Led(100);//Функция запуска таймера светодиода
	//Получение текущей отметки времени
	RTC_Get_Date_Time();
	TempResult.YY=DateTime.YY;
	TempResult.MM=DateTime.MM;
	TempResult.DD=DateTime.DD;
	TempResult.hour=DateTime.hour;
	TempResult.min=DateTime.min;
	TempResult.sec=DateTime.sec;
	Meas_System_GetValue(); //Произвести измерение всех внутренних каналов
	//Канал10(.Bat1Level)
	TempResult.ChID=DeviceSerial*100+10;
	TempResult.Value=MeasSystem.Bat1Level;
	TempResult.Variation=MeasSystem.Bat1Level;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//Канал11 //Bat2Level
	TempResult.ChID=DeviceSerial*100+11;
	TempResult.Value=MeasSystem.Bat2Level;
	TempResult.Variation=MeasSystem.Bat2Level;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//Внесение записи в канал12 режим работы(Командный режим-1, Режим измерений-0)
	TempResult.ChID=DeviceSerial*100+12;
	TempResult.Value=1;
	TempResult.Variation=1;
	TempResult.Temperature=1;
	Save_Record(TempResult);
	//Устанавливается соединение и начинается передача данных
	GSM_Connect(); //Ожидается подключение к сети GSM
	GSM_GPRS_Connect(); //Ожидается подключение к сети GPRS
	if (GSM.GPRS_Connected)
		{
		GSM_FTP_Connect();
		if (GSM.FTP_Connected)
			{
			FTPOKFlag=0xff;//Устанавливается флаг успешной передачи данных по ftp
				//Добавляются строки с уровнем сигнала GSM
			//Канал8 (SQ_RSSI)
			TempResult.ChID=DeviceSerial*100+8;
			TempResult.Value=GSM.SQ_RSSI;
			TempResult.Variation=GSM.SQ_RSSI;
			TempResult.Temperature=MeasSystem.Temperature;
			Save_Record(TempResult);
			//Канал9 (TimeFTPSession)
			TempResult.ChID=DeviceSerial*100+9;
			TempResult.Value=Timeouts.TimeFTPSession/1000000; //Секунды
			TempResult.Variation=GSM.SQ_BER;
			TempResult.Temperature=MeasSystem.Temperature;
			Save_Record(TempResult);
			//Передача измеренных значений
			for (i = 0x00; i < 0x3000; ++i)
				{
				TempResult=Read_Record(i);
				if ((TempResult.FlagMeas!=0xff)&(TempResult.FlagFTP==0xff))
					{
					//Дата
					GSM.FTP_Out_String[0]=(TempResult.DD/10)%10 + 0x30;GSM.FTP_Out_String[1]=TempResult.DD%10 + 0x30;
					GSM.FTP_Out_String[2]='/';
					GSM.FTP_Out_String[3]=(TempResult.MM/10)%10 + 0x30;GSM.FTP_Out_String[4]=TempResult.MM%10 + 0x30;
					GSM.FTP_Out_String[5]='/';
					GSM.FTP_Out_String[6]='2';GSM.FTP_Out_String[7]='0';GSM.FTP_Out_String[8]=(TempResult.YY/10)%10 + 0x30;GSM.FTP_Out_String[9]=TempResult.YY%10 + 0x30;
					GSM.FTP_Out_String[10]=';';
					GSM.FTP_Out_String[11]=(TempResult.hour/10)%10 + 0x30;GSM.FTP_Out_String[12]=TempResult.hour%10 + 0x30;
					GSM.FTP_Out_String[13]=':';
					GSM.FTP_Out_String[14]=(TempResult.min/10)%10 + 0x30;	GSM.FTP_Out_String[15]=TempResult.min%10 + 0x30;
					GSM.FTP_Out_String[16]=':';
					GSM.FTP_Out_String[17]=(TempResult.sec/10)%10 + 0x30;	GSM.FTP_Out_String[18]=TempResult.sec%10 + 0x30;
					GSM.FTP_Out_String[19]='.';
					GSM.FTP_Out_String[20]='0';
					GSM.FTP_Out_String[21]=';';
					//ChID
					GSM.FTP_Out_String[22]=(TempResult.ChID/1000000000)%10+0x30;
					GSM.FTP_Out_String[23]=(TempResult.ChID/100000000)%10+0x30;
					GSM.FTP_Out_String[24]=(TempResult.ChID/10000000)%10+0x30;
					GSM.FTP_Out_String[25]=(TempResult.ChID/1000000)%10+0x30;
					GSM.FTP_Out_String[26]=(TempResult.ChID/100000)%10+0x30;
					GSM.FTP_Out_String[27]=(TempResult.ChID/10000)%10+0x30;
					GSM.FTP_Out_String[28]=(TempResult.ChID/1000)%10+0x30;
					GSM.FTP_Out_String[29]=(TempResult.ChID/100)%10+0x30;
					GSM.FTP_Out_String[30]=(TempResult.ChID/10)%10+0x30;
					GSM.FTP_Out_String[31]=TempResult.ChID%10+0x30;
					//val
					pos=32;
					GSM.FTP_Out_String[pos]=';';pos++;
					TempVal=TempResult.Value*100000;
					if (TempVal<0)
						{
						TempVal=TempVal*-1;
						GSM.FTP_Out_String[pos]='-'; pos++;
						}
						GSM.FTP_Out_String[pos]=(TempVal/100000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/1000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/100000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]='.';pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/1000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/100)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=TempVal%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=';';pos++;
					//Var
					TempVal=TempResult.Variation*100000;
					if (TempVal<0)
						{
						TempVal=TempVal*-1;
						GSM.FTP_Out_String[pos]='-'; pos++;
						}
					GSM.FTP_Out_String[pos]=(TempVal/100000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/1000000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/100000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]='.';pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/1000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/100)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=TempVal%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=';';pos++;
					//Temp
					TempVal=TempResult.Temperature*100;
					if (TempVal<0)
						{
						TempVal=TempVal*-1;
						GSM.FTP_Out_String[pos]='-'; pos++;
						}
					GSM.FTP_Out_String[pos]=(TempVal/1000)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=(TempVal/100)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]='.';pos++;
					GSM.FTP_Out_String[pos]=(TempVal/10)%10+0x30;pos++;
					GSM.FTP_Out_String[pos]=TempVal%10+0x30;pos++;
					//Завершение строки
					GSM.FTP_Out_String[pos]='\0';pos++;
					//Отправка строки
					GSM.Attempt=3; GSM.ATErrorFlag=0xff;
					while ((GSM.ATErrorFlag!=0x00)&&(GSM.Attempt!=0))
						{
						Delay_us(200000); //Задержка 200 мс
						GSM_FTP_Send_string(3000); //Отправка строки (таймаут 3 секунды)
						GSM.Attempt--;
						}
					}
				}
			//Закрывается файл
			GSM.Attempt=3; GSM.ATErrorFlag=0xff;
			while ((GSM.ATErrorFlag!=0x00)&&(GSM.Attempt!=0))
				{
				Delay_us(200000); //Задержка 200 мс
				GSM_FTP_Close_File(10000); //Закрытие файла (таймаут 10 секунд)
				GSM.Attempt--;
				}
			//Закрывается соединение
			Delay_us(1000000); //Задержка 1000 мс
			GSM_FTP_Close(10000); //Закрытие соединения (таймаут 10 секунд)
			Delay_us(1000000); //Задержка 1000 мс
			}
		}
	//Проверка успешности передачи данных
	if (FTPOKFlag!=0) //Если данные успешно переданы
		{
		ClearFTPFlags(); //Очистка флагов передачи данных по FTP
		}
	}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////      Функция производит выполнение инструкции SetCycleSettings из СМС ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_Execution_SetCycleSettings(void)
	{
	unsigned char StrInstruction[16]={'S','e','t','C','y','c','l','e','S','e','t','t','i','n','g','s'};//Установить параметры измерительного цикла
	unsigned char InstructionLength=16; //Размер инструкции с которой производится сравнение
	unsigned int CurrentDelimiterNumber=0; //текущий номер разделителя
	unsigned int TypePosStart=0; //Начало блока "тип сообщения"
	unsigned int TypePosEnd=0; //Конец блока "тип сообщения"
	unsigned int AddressPosStart=0; //Начало блока "адрес"
	unsigned int AddressPosEnd=0; //Конец блока "адрес"
	unsigned int TransactionPosStart=0; //Начало блока "транзакции"
	unsigned int TransactionPosEnd=0; //Конец блока "транзакции"
	unsigned int InstructionPosStart=0; //Начало блока "инструкции"
	unsigned int InstructionPosEnd=0; //Конец блока "инструкции"
	unsigned int DataPosStart=0; //Начало блока "данные"
	unsigned int DataPosEnd=0; //Конец блока "данные"
	unsigned int DelimiterData1Pos=0; //Позиция первого разделителя (запятая) в блоке "Данные"
	unsigned int DelimiterData2Pos=0; //Позиция второго разделителя (запятая) в блоке "Данные"
	unsigned int i; //Счетчик
	//Проверка корректности сообщения
	//Находятся разделители частей сообщения
	for (i = 0; i < RS485.Buffer_Len; ++i)
		{
		if (RS485.Buffer[i]=='/') //Если обнаружен разделитель
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //Если обнаружен 0-й разделитель
				TypePosStart=i+1; //Запоминается начало блока "тип сообщения"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 1: //Если обнаружен 1-й разделитель
				TypePosEnd=i-1; //Запоминается конец блока "тип сообщения"
				AddressPosStart=i+1; //Запоминается начало блока "адрес"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 2: //Если обнаружен 2-й разделитель
				AddressPosEnd=i-1; //Запоминается конец блока "адрес"
				TransactionPosStart=i+1; //Запоминается начало блока "транзакция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 3: //Если обнаружен 3-й разделитель
				TransactionPosEnd=i-1; //Запоминается конец блока "транзакция"
				InstructionPosStart=i+1; //Запоминается начало блока "инструкция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 4: //Если обнаружен 4-й разделитель
				InstructionPosEnd=i-1; //Запоминается конец блока "инструкция"
				DataPosStart=i+1; //Запоминается начало блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 5: //Если обнаружен5-й разделитель
				DataPosEnd=i-1; //Запоминается конец блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				}
			}
		}
	if (CurrentDelimiterNumber!=6) {return;} //Проверяется количество блоков в сообщении
	//Проверяется размер блоков
	if (TypePosStart>TypePosEnd) {return;}//Если блок "Тип" пустой
	if (AddressPosStart>AddressPosEnd) {return;}//Если блок "Адрес" пустой
	if (TransactionPosStart>TransactionPosEnd) {return;}//Если блок "Транзакция" пустой
	if (InstructionPosStart>InstructionPosEnd) {return;}//Если блок "Инструкция" пустой
	if (DataPosStart>DataPosEnd) {return;}//Если блок "данные" пустой
	//Проверяется корректность блока "тип сообщения"
	if (TypePosEnd!=TypePosStart) {return;}
	if (RS485.Buffer[TypePosStart]!='Q') {return;}//Если сообщение - не является запросом
	//Проверяется корректность блока "адрес"
	for (i = AddressPosStart; i <= AddressPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//Если символ - не является цифрой
	unsigned int TempCode; //принятый код доступа
	//Преобразование в целое без знака
	TempCode=0;
	unsigned int M=1; //Множитель текущего разряда
	for (i = AddressPosEnd; i >= AddressPosStart; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {TempCode=TempCode+M*(RS485.Buffer[i]-48);M=M*10;}}
	//Проверяется код доступа
	if (TempCode!=GSM.SMSCode) {return;}//код доступа не принят
	//Проверяется корректность блока "Инструкция"
	if ((InstructionPosEnd-InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = InstructionPosStart; i <= InstructionPosEnd; ++i) {if (RS485.Buffer[i]!=StrInstruction[i-InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка блока "Данные"
	if (DataPosStart>DataPosEnd)  {return;}//Если блок "данные" пустой
	//Определяются позиции запятых в блоке "Данные"
	for (i = DataPosStart; i <= DataPosEnd; ++i)
		{
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos!=0)&(DelimiterData2Pos==0)) {DelimiterData2Pos=i;}//Если в блоке "данные" обнаружена вторая запятая
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //Если в блоке "данные" обнаружена первая запятая
		}
	//Проверка количества элементов блока "Данные"
	if (DelimiterData2Pos==0) {return;} //Если в блоке "данные" меньше 2 запятых
	//Проверка первого элемента блока "Данные" (CyclePeriod) на корректность
	for (i = DataPosStart; i < DelimiterData1Pos; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//Если символ - не является цифрой
	//Преобразование в целое без знака
	CyclePeriod=0;
	M=1; //Множитель текущего разряда
	for (i = DelimiterData1Pos-1; i >= DataPosStart; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {CyclePeriod=CyclePeriod+M*(RS485.Buffer[i]-48);M=M*10;}}
		//Проверка второго элемента блока "Данные" (CycleStart) на корректность
	for (i = DelimiterData1Pos+1; i < DelimiterData2Pos; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}} //Если символ - не является цифрой
	//Преобразование в целое без знака
	CycleStart=0;
	M=1; //Множитель текущего разряда
	for (i = DelimiterData2Pos-1; i >= DelimiterData1Pos+1; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {CycleStart=CycleStart+M*(RS485.Buffer[i]-48);M=M*10;}}
	//Проверка третьего элемента блока "Данные" (CycleSendDataPeriod) на корректность
	for (i = DelimiterData2Pos+1; i <= DataPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}} //Если символ - не является цифрой
	//Преобразование в целое без знака
	CycleSendDataPeriod=0;
	M=1; //Множитель текущего разряда
	for (i = DataPosEnd; i >= DelimiterData2Pos+1; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {CycleSendDataPeriod=CycleSendDataPeriod+M*(RS485.Buffer[i]-48);M=M*10;}}
	//Проверка значений
	if (CycleSendDataPeriod>MaximumPeriodSendData) {return;} //Если количество пропусков передачи данных больше 255
	if (CyclePeriod>86400) {return;} //Если период больше 24 часов
	if (CyclePeriod<MinimumPeriod) {return;} //Если период меньше 5 минут
	if (CycleStart>86399) {return;} //
	SetCycle_Settings(); //Установка настроек циклов измерения
	RTC_Set_Next_Cycle_Time(); //Устанавливается время следующего цикла
	//Отправка ответа
	Delay_us(5000000);
	for (i = 0; i <= RS485.Buffer_Len; ++i) {GSM.OutSMS[i]=RS485.Buffer[i];} //Формирование строки
	GSM.OutSMS[RS485.Buffer_Len+1]='\0';
	GSM.OutSMS[2]='R';
	GSM_Send_SMS(10000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////      Функция производит выполнение инструкции SetCycleSettings из СМС ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_Execution_SetSMSSettings(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','S','M','S','S','e','t','t','i','n','g','s'};//Установить параметры измерительного цикла
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned int CurrentDelimiterNumber=0; //текущий номер разделителя
	unsigned int TypePosStart=0; //Начало блока "тип сообщения"
	unsigned int TypePosEnd=0; //Конец блока "тип сообщения"
	unsigned int AddressPosStart=0; //Начало блока "адрес"
	unsigned int AddressPosEnd=0; //Конец блока "адрес"
	unsigned int TransactionPosStart=0; //Начало блока "транзакции"
	unsigned int TransactionPosEnd=0; //Конец блока "транзакции"
	unsigned int InstructionPosStart=0; //Начало блока "инструкции"
	unsigned int InstructionPosEnd=0; //Конец блока "инструкции"
	unsigned int DataPosStart=0; //Начало блока "данные"
	unsigned int DataPosEnd=0; //Конец блока "данные"
	unsigned int DelimiterData1Pos=0; //Позиция первого разделителя (запятая) в блоке "Данные"
	unsigned int i; //Счетчик
	//Проверка корректности сообщения
	//Находятся разделители частей сообщения
	for (i = 0; i < RS485.Buffer_Len; ++i)
		{
		if (RS485.Buffer[i]=='/') //Если обнаружен разделитель
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //Если обнаружен 0-й разделитель
				TypePosStart=i+1; //Запоминается начало блока "тип сообщения"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 1: //Если обнаружен 1-й разделитель
				TypePosEnd=i-1; //Запоминается конец блока "тип сообщения"
				AddressPosStart=i+1; //Запоминается начало блока "адрес"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 2: //Если обнаружен 2-й разделитель
				AddressPosEnd=i-1; //Запоминается конец блока "адрес"
				TransactionPosStart=i+1; //Запоминается начало блока "транзакция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 3: //Если обнаружен 3-й разделитель
				TransactionPosEnd=i-1; //Запоминается конец блока "транзакция"
				InstructionPosStart=i+1; //Запоминается начало блока "инструкция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 4: //Если обнаружен 4-й разделитель
				InstructionPosEnd=i-1; //Запоминается конец блока "инструкция"
				DataPosStart=i+1; //Запоминается начало блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 5: //Если обнаружен5-й разделитель
				DataPosEnd=i-1; //Запоминается конец блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				}
			}
		}
	if (CurrentDelimiterNumber!=6) {return;} //Проверяется количество блоков в сообщении
	//Проверяется размер блоков
	if (TypePosStart>TypePosEnd) {return;}//Если блок "Тип" пустой
	if (AddressPosStart>AddressPosEnd) {return;}//Если блок "Адрес" пустой
	if (TransactionPosStart>TransactionPosEnd) {return;}//Если блок "Транзакция" пустой
	if (InstructionPosStart>InstructionPosEnd) {return;}//Если блок "Инструкция" пустой
	if (DataPosStart>DataPosEnd) {return;}//Если блок "данные" пустой
	//Проверяется корректность блока "тип сообщения"
	if (TypePosEnd!=TypePosStart) {return;}
	if (RS485.Buffer[TypePosStart]!='Q') {return;}//Если сообщение - не является запросом
	//Проверяется корректность блока "адрес"
	for (i = AddressPosStart; i <= AddressPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//Если символ - не является цифрой
	unsigned int TempCode; //принятый код доступа
	//Преобразование в целое без знака
	TempCode=0;
	unsigned int M=1; //Множитель текущего разряда
	for (i = AddressPosEnd; i >= AddressPosStart; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {TempCode=TempCode+M*(RS485.Buffer[i]-48);M=M*10;}}
	//Проверяется код доступа
	if (TempCode!=GSM.SMSCode) {return;}//код доступа не принят
	//Проверяется корректность блока "Инструкция"
	if ((InstructionPosEnd-InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = InstructionPosStart; i <= InstructionPosEnd; ++i) {if (RS485.Buffer[i]!=StrInstruction[i-InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка блока "Данные"
	if (DataPosStart>DataPosEnd)  {return;}//Если блок "данные" пустой
	//Определяются позиции запятых в блоке "Данные"
	for (i = DataPosStart; i <= DataPosEnd; ++i)
		{
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //Если в блоке "данные" обнаружена первая запятая
		}
	//Проверка количества элементов блока "Данные"
	if (DelimiterData1Pos==0) {return;} //Если в блоке "данные" меньше 1 запятой
	//Проверка первого элемента блока на корректность (строка)
	for (i = DataPosStart; i < DelimiterData1Pos; ++i)
		{
		if (RS485.Buffer[i]<'+') {return;} //Если символ не цифра
		if (RS485.Buffer[i]>'9') {return;} //Если символ не цифра
		}
	//Проверка второго элемента блока "Данные" на корректность
	for (i = DelimiterData1Pos+1; i <= DataPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}} //Если символ - не является цифрой
	//Считываются данные
	for (i = 0; i < 12; ++i) {GSM.SMSNumber[i]=RS485.Buffer[DataPosStart+i];}
	GSM.SMSNumber[12]='\0';
	//Код доступа
	GSM.SMSCode=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1,DataPosEnd);
	//Сохранение настроек
	Save_SMS_Settings(); //Строка сохраняется в SRAM
	//Отправка ответа
	Delay_us(5000000);
	for (i = 0; i <= RS485.Buffer_Len; ++i) {GSM.OutSMS[i]=RS485.Buffer[i];} //Формирование строки
	GSM.OutSMS[RS485.Buffer_Len+1]='\0';
	GSM.OutSMS[2]='R';
	GSM_Send_SMS(10000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Функция производит преобразование части строки в формате HEX, находящейся в буфере SMS, в символьную строку GSM.FTP_Settings        /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_HexChar_Buf_To_FTP_Settings(unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //Счетчик
	unsigned char j=0; //Счетчик
	unsigned char TempByte=0; //Временный байт
	unsigned char Byte1=0; //Первый полубайт(старший)
	unsigned char Byte2=0; //Второй полубайт(младший)
	i = StartPos;
	while ((i < EndPos+1)&&(j < 256))
		{
		TempByte=RS485.Buffer[i]; Byte1=0;
		if (TempByte=='0') {Byte1=0;}
		if (TempByte=='1') {Byte1=1;}
		if (TempByte=='2') {Byte1=2;}
		if (TempByte=='3') {Byte1=3;}
		if (TempByte=='4') {Byte1=4;}
		if (TempByte=='5') {Byte1=5;}
		if (TempByte=='6') {Byte1=6;}
		if (TempByte=='7') {Byte1=7;}
		if (TempByte=='8') {Byte1=8;}
		if (TempByte=='9') {Byte1=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte1=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte1=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte1=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte1=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte1=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte1=15;}
		i++;
		TempByte=RS485.Buffer[i]; Byte2=0;
		if (TempByte=='0') {Byte2=0;}
		if (TempByte=='1') {Byte2=1;}
		if (TempByte=='2') {Byte2=2;}
		if (TempByte=='3') {Byte2=3;}
		if (TempByte=='4') {Byte2=4;}
		if (TempByte=='5') {Byte2=5;}
		if (TempByte=='6') {Byte2=6;}
		if (TempByte=='7') {Byte2=7;}
		if (TempByte=='8') {Byte2=8;}
		if (TempByte=='9') {Byte2=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte2=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte2=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte2=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte2=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte2=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte2=15;}
		i++;
		GSM.FTP_Settings[j]=Byte1*16+Byte2;j++;
		}
	GSM.FTP_Settings[j]='\0'; //Добавляется завершающий символ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////      Функция производит выполнение инструкции SetFTPSettings из СМС ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_Execution_SetFTPSettings(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','F','T','P','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned int CurrentDelimiterNumber=0; //текущий номер разделителя
	unsigned int TypePosStart=0; //Начало блока "тип сообщения"
	unsigned int TypePosEnd=0; //Конец блока "тип сообщения"
	unsigned int AddressPosStart=0; //Начало блока "адрес"
	unsigned int AddressPosEnd=0; //Конец блока "адрес"
	unsigned int TransactionPosStart=0; //Начало блока "транзакции"
	unsigned int TransactionPosEnd=0; //Конец блока "транзакции"
	unsigned int InstructionPosStart=0; //Начало блока "инструкции"
	unsigned int InstructionPosEnd=0; //Конец блока "инструкции"
	unsigned int DataPosStart=0; //Начало блока "данные"
	unsigned int DataPosEnd=0; //Конец блока "данные"
	unsigned int i; //Счетчик
	//Проверка корректности сообщения
	//Находятся разделители частей сообщения
	for (i = 0; i < RS485.Buffer_Len; ++i)
		{
		if (RS485.Buffer[i]=='/') //Если обнаружен разделитель
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //Если обнаружен 0-й разделитель
				TypePosStart=i+1; //Запоминается начало блока "тип сообщения"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 1: //Если обнаружен 1-й разделитель
				TypePosEnd=i-1; //Запоминается конец блока "тип сообщения"
				AddressPosStart=i+1; //Запоминается начало блока "адрес"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 2: //Если обнаружен 2-й разделитель
				AddressPosEnd=i-1; //Запоминается конец блока "адрес"
				TransactionPosStart=i+1; //Запоминается начало блока "транзакция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 3: //Если обнаружен 3-й разделитель
				TransactionPosEnd=i-1; //Запоминается конец блока "транзакция"
				InstructionPosStart=i+1; //Запоминается начало блока "инструкция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 4: //Если обнаружен 4-й разделитель
				InstructionPosEnd=i-1; //Запоминается конец блока "инструкция"
				DataPosStart=i+1; //Запоминается начало блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 5: //Если обнаружен5-й разделитель
				DataPosEnd=i-1; //Запоминается конец блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				}
			}
		}
	if (CurrentDelimiterNumber!=6) {return;} //Проверяется количество блоков в сообщении
	//Проверяется размер блоков
	if (TypePosStart>TypePosEnd) {return;}//Если блок "Тип" пустой
	if (AddressPosStart>AddressPosEnd) {return;}//Если блок "Адрес" пустой
	if (TransactionPosStart>TransactionPosEnd) {return;}//Если блок "Транзакция" пустой
	if (InstructionPosStart>InstructionPosEnd) {return;}//Если блок "Инструкция" пустой
	if (DataPosStart>DataPosEnd) {return;}//Если блок "данные" пустой
	//Проверяется корректность блока "тип сообщения"
	if (TypePosEnd!=TypePosStart) {return;}
	if (RS485.Buffer[TypePosStart]!='Q') {return;}//Если сообщение - не является запросом
	//Проверяется корректность блока "адрес"
	for (i = AddressPosStart; i <= AddressPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//Если символ - не является цифрой
	unsigned int TempCode; //принятый код доступа
	//Преобразование в целое без знака
	TempCode=0;
	unsigned int M=1; //Множитель текущего разряда
	for (i = AddressPosEnd; i >= AddressPosStart; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {TempCode=TempCode+M*(RS485.Buffer[i]-48);M=M*10;}}
	//Проверяется код доступа
	if (TempCode!=GSM.SMSCode) {return;}//код доступа не принят
	//Проверяется корректность блока "Инструкция"
	if ((InstructionPosEnd-InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = InstructionPosStart; i <= InstructionPosEnd; ++i) {if (RS485.Buffer[i]!=StrInstruction[i-InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка блока "Данные"
	if (DataPosStart>DataPosEnd)  {return;}//Если блок "данные" пустой
	//Проверка строки на корректность
	for (i = DataPosStart; i <= DataPosEnd; ++i)
		{
		if (RS485.Buffer[i]<'0') {return;} //Если символ не цифра
		if (RS485.Buffer[i]>'f') {return;} //Если символ не цифра
		if ((RS485.Buffer[i]>'9')&&(RS485.Buffer[i]<'A')) {return;} //Если символ не цифра
		if ((RS485.Buffer[i]>'F')&&(RS485.Buffer[i]<'a')) {return;} //Если символ не цифра
		}
	//Данные преобразуются в символьный вид
	GSM_SMS_HexChar_Buf_To_FTP_Settings(DataPosStart,DataPosEnd);
	Save_FTP_Settings(); //Строка сохраняется в SRAM
	//Отправка ответа
	Delay_us(5000000);
	for (i = 0; i <= RS485.Buffer_Len; ++i) {GSM.OutSMS[i]=RS485.Buffer[i];} //Формирование строки
	GSM.OutSMS[RS485.Buffer_Len+1]='\0';
	GSM.OutSMS[2]='R';
	GSM_Send_SMS(10000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Функция производит преобразование части строки в формате HEX, находящейся в буфере SMS, в символьную строку GSM.GPRS_User_Pass ////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_HexChar_Buf_To_FTP_FileName(unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //Счетчик
	unsigned char j=0; //Счетчик
	unsigned char TempByte=0; //Временный байт
	unsigned char Byte1=0; //Первый полубайт(старший)
	unsigned char Byte2=0; //Второй полубайт(младший)
	i = StartPos;
	while ((i < EndPos+1)&&(j < 256))
		{
		TempByte=RS485.Buffer[i]; Byte1=0;
		if (TempByte=='0') {Byte1=0;}
		if (TempByte=='1') {Byte1=1;}
		if (TempByte=='2') {Byte1=2;}
		if (TempByte=='3') {Byte1=3;}
		if (TempByte=='4') {Byte1=4;}
		if (TempByte=='5') {Byte1=5;}
		if (TempByte=='6') {Byte1=6;}
		if (TempByte=='7') {Byte1=7;}
		if (TempByte=='8') {Byte1=8;}
		if (TempByte=='9') {Byte1=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte1=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte1=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte1=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte1=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte1=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte1=15;}
		i++;
		TempByte=RS485.Buffer[i]; Byte2=0;
		if (TempByte=='0') {Byte2=0;}
		if (TempByte=='1') {Byte2=1;}
		if (TempByte=='2') {Byte2=2;}
		if (TempByte=='3') {Byte2=3;}
		if (TempByte=='4') {Byte2=4;}
		if (TempByte=='5') {Byte2=5;}
		if (TempByte=='6') {Byte2=6;}
		if (TempByte=='7') {Byte2=7;}
		if (TempByte=='8') {Byte2=8;}
		if (TempByte=='9') {Byte2=9;}
		if ((TempByte=='A')||(TempByte=='a')) {Byte2=10;}
		if ((TempByte=='B')||(TempByte=='b')) {Byte2=11;}
		if ((TempByte=='C')||(TempByte=='c')) {Byte2=12;}
		if ((TempByte=='D')||(TempByte=='d')) {Byte2=13;}
		if ((TempByte=='E')||(TempByte=='e')) {Byte2=14;}
		if ((TempByte=='F')||(TempByte=='f')) {Byte2=15;}
		i++;
		GSM.FTP_File_Name[j]=Byte1*16+Byte2;j++;
		}
	GSM.FTP_File_Name[j]='\0'; //Добавляется завершающий символ
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////    Функция производит преобразование части строки, находящейся в буфере SMS, в целое число без знака       /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int GSM_SMS_Char_To_Unsigned_Int_Buf(unsigned int StartPos, unsigned int EndPos) //StartPos - позиция первого символа, EndPos - позиция последнего символа
	{
	unsigned int i; //Счетчик
	unsigned int Out=0; //Выходная переменная
	unsigned int M=1; //Множитель текущего разряда
	for (i = EndPos; i >= StartPos; --i)
		{
		if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) //Если символ - цифра
			{
			Out=Out+M*(RS485.Buffer[i]-48);
			M=M*10; //Увеличивается множитель текущего разряда;
			}
		}
	return Out;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        Функция производит выполнение инструкции SetFTPFileName  из СМС        ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_Execution_SetFTPFileName(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','F','T','P','F','i','l','e','N','a','m','e'};
	unsigned char InstructionLength=14; //Размер инструкции с которой производится сравнение
	unsigned int CurrentDelimiterNumber=0; //текущий номер разделителя
	unsigned int TypePosStart=0; //Начало блока "тип сообщения"
	unsigned int TypePosEnd=0; //Конец блока "тип сообщения"
	unsigned int AddressPosStart=0; //Начало блока "адрес"
	unsigned int AddressPosEnd=0; //Конец блока "адрес"
	unsigned int TransactionPosStart=0; //Начало блока "транзакции"
	unsigned int TransactionPosEnd=0; //Конец блока "транзакции"
	unsigned int InstructionPosStart=0; //Начало блока "инструкции"
	unsigned int InstructionPosEnd=0; //Конец блока "инструкции"
	unsigned int DataPosStart=0; //Начало блока "данные"
	unsigned int DataPosEnd=0; //Конец блока "данные"
	unsigned int DelimiterData1Pos=0; //Позиция первого разделителя (запятая) в блоке "Данные"
	unsigned int i; //Счетчик
	//Проверка корректности сообщения
	//Находятся разделители частей сообщения
	for (i = 0; i < RS485.Buffer_Len; ++i)
		{
		if (RS485.Buffer[i]=='/') //Если обнаружен разделитель
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //Если обнаружен 0-й разделитель
				TypePosStart=i+1; //Запоминается начало блока "тип сообщения"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 1: //Если обнаружен 1-й разделитель
				TypePosEnd=i-1; //Запоминается конец блока "тип сообщения"
				AddressPosStart=i+1; //Запоминается начало блока "адрес"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 2: //Если обнаружен 2-й разделитель
				AddressPosEnd=i-1; //Запоминается конец блока "адрес"
				TransactionPosStart=i+1; //Запоминается начало блока "транзакция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 3: //Если обнаружен 3-й разделитель
				TransactionPosEnd=i-1; //Запоминается конец блока "транзакция"
				InstructionPosStart=i+1; //Запоминается начало блока "инструкция"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 4: //Если обнаружен 4-й разделитель
				InstructionPosEnd=i-1; //Запоминается конец блока "инструкция"
				DataPosStart=i+1; //Запоминается начало блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				case 5: //Если обнаружен5-й разделитель
				DataPosEnd=i-1; //Запоминается конец блока "данные"
				CurrentDelimiterNumber++; //Инкрементируется текущий номер разделителя
				break;
				}
			}
		}
	if (CurrentDelimiterNumber!=6) {return;} //Проверяется количество блоков в сообщении
	//Проверяется размер блоков
	if (TypePosStart>TypePosEnd) {return;}//Если блок "Тип" пустой
	if (AddressPosStart>AddressPosEnd) {return;}//Если блок "Адрес" пустой
	if (TransactionPosStart>TransactionPosEnd) {return;}//Если блок "Транзакция" пустой
	if (InstructionPosStart>InstructionPosEnd) {return;}//Если блок "Инструкция" пустой
	if (DataPosStart>DataPosEnd) {return;}//Если блок "данные" пустой
	//Проверяется корректность блока "тип сообщения"
	if (TypePosEnd!=TypePosStart) {return;}
	if (RS485.Buffer[TypePosStart]!='Q') {return;}//Если сообщение - не является запросом
	//Проверяется корректность блока "адрес"
	for (i = AddressPosStart; i <= AddressPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//Если символ - не является цифрой
	unsigned int TempCode; //принятый код доступа
	//Преобразование в целое без знака
	TempCode=0;
	unsigned int M=1; //Множитель текущего разряда
	for (i = AddressPosEnd; i >= AddressPosStart; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {TempCode=TempCode+M*(RS485.Buffer[i]-48);M=M*10;}}
	//Проверяется код доступа
	if (TempCode!=GSM.SMSCode) {return;}//код доступа не принят
	//Проверяется корректность блока "Инструкция"
	if ((InstructionPosEnd-InstructionPosStart+1)!=InstructionLength) {return;} //Если размер инструкции не соответствует требуемому, то завершается выполнение  инструкции
	for (i = InstructionPosStart; i <= InstructionPosEnd; ++i) {if (RS485.Buffer[i]!=StrInstruction[i-InstructionPosStart]) {return;}} //Если обнаружено несоответствие символов, то завершается выполнение  инструкции
	//Проверка блока "Данные"
	if (DataPosStart>DataPosEnd)  {return;}//Если блок "данные" пустой
	//Определяются позиции запятых в блоке "Данные"
	for (i = DataPosStart; i <= DataPosEnd; ++i)
		{
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //Если в блоке "данные" обнаружена первая запятая
		}
	//Проверка количества элементов блока "Данные"
	if (DelimiterData1Pos==0) {return;} //Если в блоке "данные" меньше 1 запятой
	//Проверка размера элементов блока "Данные"
	if (DataPosStart==DelimiterData1Pos) {return;} //Если первый элемент пустой
	//Проверка первого элемента блока на корректность (строка)
	for (i = DataPosStart; i < DelimiterData1Pos; ++i)
		{
		if (RS485.Buffer[i]<'0') {return;} //Если символ не цифра
		if (RS485.Buffer[i]>'f') {return;} //Если символ не цифра
		if ((RS485.Buffer[i]>'9')&&(RS485.Buffer[i]<'A')) {return;} //Если символ не цифра
		if ((RS485.Buffer[i]>'F')&&(RS485.Buffer[i]<'a')) {return;} //Если символ не цифра
		}
	//Проверка второго элемента блока "Данные" на корректность
	for (i = DelimiterData1Pos+1; i <= DataPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}} //Если символ - не является цифрой
	//Данные преобразуются в символьный вид
	GSM_SMS_HexChar_Buf_To_FTP_FileName(DataPosStart,DelimiterData1Pos-1);
	GSM.FTP_Append=GSM_SMS_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, DataPosEnd); //Период передачи данных
	Save_FTP_FileName(); //Строка сохраняется в SRAM
	//Отправка ответа
	Delay_us(5000000);
	for (i = 0; i <= RS485.Buffer_Len; ++i) {GSM.OutSMS[i]=RS485.Buffer[i];} //Формирование строки
	GSM.OutSMS[RS485.Buffer_Len+1]='\0';
	GSM.OutSMS[2]='R';
	GSM_Send_SMS(10000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                        Чтение всех SMS и выполнение команд                      //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Read_ALL_SMS(unsigned int Timeout_ms)
	{
	Delay_us(1000000); //Задержка 1с
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="ok"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
		//Установить текстовый формат сообщений (AT+CMGF=1)
	strcpy(GSM.Buffer,"AT+CMGF=1");
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>50))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	/////////////////////////////////
	//Чтение сообщений (используется структура RS485)
	/////////////////////////////////
	RS485.Message_Complete_flag=0x00; //Флаг окончания сборки сообщения
	RS485.Start_Collect_Flag=0x00; //Флаг начала процесса сборки сообщения
	RS485.Buffer_Pos=0x00; //Позиция курсора в буфере
	RS485.Previous_Byte=0x00; //Текущий байт
	RS485.Current_Byte=0x00; //Предыдущий байт
	Delay_us(1000000); //Задержка 1с
	if (Timeouts.TimeoutGSM_AT<51) {return;}
		//Прочитать все сообщения (AT+CMGL=ALL)
	strcpy(GSM.Buffer,"AT+CMGL=ALL");
	GSM_Send_AT(); //Передача AT команды
	//Запускается процесс сборки сообщения
	while (RS485.Message_Complete_flag==0x00)
		{
		if (Timeouts.TimeoutGSM_AT<51) {return;} //Если превышен таймаут ожидания
		if (RS485.Buffer_Pos>1023) {return;}//Если переполнился буфер
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			RS485.Previous_Byte=RS485.Current_Byte; //Запоминается предыдущий принятый байт
			RS485.Current_Byte=GSM_Recive_Char(); //Принимается символ
			if (RS485.Start_Collect_Flag!=0) //Если идет процесс сборки сообщения
				{
				RS485.Buffer[RS485.Buffer_Pos]=RS485.Current_Byte; //Заносится текущий байт в буфер сообщения
				RS485.Buffer_Pos++; //Инкрементируется счетчик положения курсора в буфере сообщения
				RS485.Buffer_Len=RS485.Buffer_Pos; //Вычисляется количество данных в буфере сообщения
				if ((RS485.Previous_Byte=='/')&&(RS485.Current_Byte=='%')) {RS485.Message_Complete_flag=0xff;} //Если обнаружен маркер конца сообщения, то останавливается процесс сборки сообщения
				}
			if ((RS485.Previous_Byte=='%')&&(RS485.Current_Byte=='/')) //Если обнаружен маркер начала сообщения
				{
				RS485.Start_Collect_Flag=0xff; //Устанавливается флаг запуска процесса выборки сообщения
				RS485.Buffer[0]=RS485.Previous_Byte; //Заносится предыдущий байт в буфер сообщения
				RS485.Buffer[1]=RS485.Current_Byte; //Заносится текущий байт в буфер сообщения
				RS485.Buffer_Pos=2; //Задается позиция курсора в буфере сообщения
				}
			}
		}
	if (RS485.Message_Complete_flag!=0x00) //Если сообщение собрано
		{
		GSM_SMS_Execution_SetCycleSettings(); //Выполнение инструкции SetCycleSettings
 		GSM_SMS_Execution_SetFTPSettings();  //Выполнение инструкции SetCycleSettings
 		GSM_SMS_Execution_SetFTPFileName(); //Выполнение инструкции SetFTPFileName
 		GSM_SMS_Execution_SetSMSSettings(); //Выполнение инструкции SetCycleSettings
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                                    Удаление всех SMS                            //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Delete_All_SMS(unsigned int Timeout_ms)
	{
	Delay_us(1000000); //Задержка 1с
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //устанавливается таймаут выполнения AT команды
	unsigned char Byte=0x00; //Байт, прочитанный из USART
	char ReplySTR[]="ok"; //Ключевое слово, содержащееся в ответе перед результатами
	unsigned char ReplySTRLen=strlen(ReplySTR); //Размер строки
	unsigned char ReplySTRIndex=0;
	GSM.ATErrorFlag=0xff; //Устанавливается флаг ошибки AT команды
		//Установить текстовый формат сообщений (AT+CMGF=1)
	strcpy(GSM.Buffer,"AT+CMGF=1");
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>50))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	Delay_us(1000000); //Задержка 1с
	if (Timeouts.TimeoutGSM_AT<51) {return;}
		//удалить все сообщения (AT+CMGD=1,4)
	strcpy(GSM.Buffer,"AT+CMGD=1,4");
	GSM_Send_AT(); //Передача AT команды
	//Поиск ключевого слова в ответе GSM модуля
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>50))
		{
		if (GSM.UsartRxNE != RESET) //Если в буфере USART есть байт
			{
			Byte=GSM_Recive_Char(); //Принимается символ
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //Символы преобразуются в строчные
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	Delay_us(1000000); //Задержка 1с
	}




















////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////             Функции системы измерения напряжений                        ///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////        Функция инициализации системы измерения напряжений        ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Meas_System_Init(void)//Функция инициализации измерительной системы
	{
	//Инициализация портов контроллера
	GPIO_InitTypeDef GPIO_InitStructure;
	//Вход АЦП - уровень заряда аккумулятора 1
	GPIO_InitStructure.GPIO_Pin = Bat1_Level_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Bat1_Level_Port, &GPIO_InitStructure);
	//Вход АЦП - уровень заряда аккумулятора 2
	GPIO_InitStructure.GPIO_Pin = Bat2_Level_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Bat2_Level_Port, &GPIO_InitStructure);
	//Вход АЦП - термистор (Температура устройства)
	GPIO_InitStructure.GPIO_Pin = Termistor_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Termistor_Port, &GPIO_InitStructure);
	//Вход АЦП - Потенциометры (Питание)
	GPIO_InitStructure.GPIO_Pin = Power_Potenciometr_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Power_Potenciometr_Port, &GPIO_InitStructure);
	//Вход АЦП - Потенциометр (Канал№1)
	GPIO_InitStructure.GPIO_Pin = CH1_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH1_Port, &GPIO_InitStructure);
	//Вход АЦП - Потенциометр (Канал№2)
	GPIO_InitStructure.GPIO_Pin = CH2_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH2_Port, &GPIO_InitStructure);
	//Вход АЦП - Потенциометр (Канал№3)
	GPIO_InitStructure.GPIO_Pin = CH3_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH3_Port, &GPIO_InitStructure);
	//Вход АЦП - Потенциометр (Канал№4)
	GPIO_InitStructure.GPIO_Pin = CH4_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH4_Port, &GPIO_InitStructure);
	//Вход АЦП - Потенциометр (Канал№5)
	GPIO_InitStructure.GPIO_Pin = CH5_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH5_Port, &GPIO_InitStructure);
	//Вход АЦП - Потенциометр (Канал№6)
	GPIO_InitStructure.GPIO_Pin = CH6_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH6_Port, &GPIO_InitStructure);
	//Инициализация АЦП контроллера
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC_MeasSystem, ENABLE);
	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_Init(ADC_MeasSystem, &ADC_InitStructure);
	//Включается АЦП
	ADC_Cmd(ADC_MeasSystem, ENABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////         Функция измерения напряжения на потенциометрах           ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Meas_System_GetValue(void)
	{
	unsigned int i; //Счетчик
	unsigned int ADC_Data_Sum=0; //Временная переменная
						//Измерение напряжения на термисторе
	ADC_Data_Sum=0;
	ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_15, 1, ADC_SampleTime_480Cycles); //Настройка канала АЦП
	for (i = 0; i < 1024; i++)
		{
		Timeouts.TimeoutADC=ADC_Timeout; //Запускается счетчик таймаута измерения АЦП
		ADC_SoftwareStartConv(ADC_MeasSystem); //Запуск измерения
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //Ожидание окончания измерения
		ADC_Data_Sum=ADC_Data_Sum + ADC_GetConversionValue(ADC_MeasSystem); //Чтение результатов измерения
		}
	//Расчет температуры
	MeasSystem.Temperature=(float) (ADC_Data_Sum/1024); //Среднее значение АЦП
	MeasSystem.Temperature=(float) (MeasSystem.Temperature*MeasSystem.TermistorConstRes/(4096-MeasSystem.Temperature)); //Определение текущего сопротивления термистора
	MeasSystem.Temperature=(float) (MeasSystem.Temperature-MeasSystem.TermistorRes)/3.85;
						//Измерение напряжения Bat1
	ADC_Data_Sum=0;
	ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_6, 1, ADC_SampleTime_480Cycles); //Настройка канала АЦП
	for (i = 0; i < 1024; i++)
		{
		Timeouts.TimeoutADC=ADC_Timeout; //Запускается счетчик таймаута измерения АЦП
		ADC_SoftwareStartConv(ADC_MeasSystem); //Запуск измерения
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //Ожидание окончания измерения
		ADC_Data_Sum=ADC_Data_Sum + ADC_GetConversionValue(ADC_MeasSystem); //Чтение результатов измерения
		}
	//Расчет напряжения
	MeasSystem.Bat1Level=(float) (ADC_Data_Sum/1024); //Среднее значение АЦП
	MeasSystem.Bat1Level=(float) (MeasSystem.Bat1Level*15.3/4096); //Значение напряжения, В
						//Измерение напряжения Bat2
	ADC_Data_Sum=0;
	ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_7, 1, ADC_SampleTime_480Cycles); //Настройка канала АЦП
	for (i = 0; i < 1024; i++)
		{
		Timeouts.TimeoutADC=ADC_Timeout; //Запускается счетчик таймаута измерения АЦП
		ADC_SoftwareStartConv(ADC_MeasSystem); //Запуск измерения
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //Ожидание окончания измерения
		ADC_Data_Sum=ADC_Data_Sum + ADC_GetConversionValue(ADC_MeasSystem); //Чтение результатов измерения
		}
	//Расчет напряжения
	MeasSystem.Bat2Level=(float) (ADC_Data_Sum/1024); //Среднее значение АЦП
	MeasSystem.Bat2Level=(float) (MeasSystem.Bat2Level*15.3/4096); //Значение напряжения, В
	//Измерение напряжения на потенциометрах
	MeasSystem.CHPowerVoltage=0;
	MeasSystem.CHPowerRes=0;
	MeasSystem.CH1Val=0;
	MeasSystem.CH2Val=0;
	MeasSystem.CH3Val=0;
	MeasSystem.CH4Val=0;
	MeasSystem.CH5Val=0;
	MeasSystem.CH6Val=0;
	float TempVoltage=0; //Переменная для временного хранения напряжения на потенциометрах
	float TempRes=0; //Переменная для временного хранения сопротивления потенциометров
	for (i = 0; i < 1024; i++)
		{
		//Измерение CHPowerVoltage и CHPowerRes
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_5, 1, ADC_SampleTime_480Cycles); //Настройка канала АЦП
		Timeouts.TimeoutADC=ADC_Timeout; //Запускается счетчик таймаута измерения АЦП
		ADC_SoftwareStartConv(ADC_MeasSystem); //Запуск измерения
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //Ожидание окончания измерения
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //Чтение результатов измерения
		TempVoltage=(float) (ADC_Data_Sum*3300/4096); //Напряжение на потенциометрах, мВ
		TempRes=(float) (TempVoltage*MeasSystem.PowerConstRes/(3300-TempVoltage)); //Сопротивление термисторов
		MeasSystem.CHPowerVoltage+=TempVoltage;
		MeasSystem.CHPowerRes+=TempRes;
		//Измерение CH1
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_0, 1, ADC_SampleTime_480Cycles); //Настройка канала АЦП
		Timeouts.TimeoutADC=ADC_Timeout; //Запускается счетчик таймаута измерения АЦП
		ADC_SoftwareStartConv(ADC_MeasSystem); //Запуск измерения
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //Ожидание окончания измерения
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //Чтение результатов измерения
		MeasSystem.CH1Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //Расчет результата измерения
		//Измерение CH2
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_13, 1, ADC_SampleTime_480Cycles); //Настройка канала АЦП
		Timeouts.TimeoutADC=ADC_Timeout; //Запускается счетчик таймаута измерения АЦП
		ADC_SoftwareStartConv(ADC_MeasSystem); //Запуск измерения
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //Ожидание окончания измерения
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //Чтение результатов измерения
		MeasSystem.CH2Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //Расчет результата измерения
		//Измерение CH3
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_12, 1, ADC_SampleTime_480Cycles); //Настройка канала АЦП
		Timeouts.TimeoutADC=ADC_Timeout; //Запускается счетчик таймаута измерения АЦП
		ADC_SoftwareStartConv(ADC_MeasSystem); //Запуск измерения
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //Ожидание окончания измерения
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //Чтение результатов измерения
		MeasSystem.CH3Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //Расчет результата измерения
		//Измерение CH4
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_11, 1, ADC_SampleTime_480Cycles); //Настройка канала АЦП
		Timeouts.TimeoutADC=ADC_Timeout; //Запускается счетчик таймаута измерения АЦП
		ADC_SoftwareStartConv(ADC_MeasSystem); //Запуск измерения
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //Ожидание окончания измерения
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //Чтение результатов измерения
		MeasSystem.CH4Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //Расчет результата измерения
		//Измерение CH5
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_10, 1, ADC_SampleTime_480Cycles); //Настройка канала АЦП
		Timeouts.TimeoutADC=ADC_Timeout; //Запускается счетчик таймаута измерения АЦП
		ADC_SoftwareStartConv(ADC_MeasSystem); //Запуск измерения
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //Ожидание окончания измерения
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //Чтение результатов измерения
		MeasSystem.CH5Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //Расчет результата измерения
		//Измерение CH6
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_1, 1, ADC_SampleTime_480Cycles); //Настройка канала АЦП
		Timeouts.TimeoutADC=ADC_Timeout; //Запускается счетчик таймаута измерения АЦП
		ADC_SoftwareStartConv(ADC_MeasSystem); //Запуск измерения
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //Ожидание окончания измерения
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //Чтение результатов измерения
		MeasSystem.CH6Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //Расчет результата измерения
		}
	//Расчет средних значений
	MeasSystem.CHPowerVoltage=(float) (MeasSystem.CHPowerVoltage/1024);
	MeasSystem.CHPowerRes=(float) (MeasSystem.CHPowerRes/1024);
	MeasSystem.CH1Val=(float) (MeasSystem.CH1Val/1024);
	MeasSystem.CH2Val=(float) (MeasSystem.CH2Val/1024);
	MeasSystem.CH3Val=(float) (MeasSystem.CH3Val/1024);
	MeasSystem.CH4Val=(float) (MeasSystem.CH4Val/1024);
	MeasSystem.CH5Val=(float) (MeasSystem.CH5Val/1024);
	MeasSystem.CH6Val=(float) (MeasSystem.CH6Val/1024);
	}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////                      Функция измерения цикла                   ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MeasCycle(void)
	{
	TMemoryResult TempResult;
	unsigned char i; //Счетчик
	TempResult.YY=DateTime.YY;
	TempResult.MM=DateTime.MM;
	TempResult.DD=DateTime.DD;
	TempResult.hour=DateTime.hour;
	TempResult.min=DateTime.min;
	TempResult.sec=DateTime.sec;
	Meas_System_GetValue(); //Произвести измерение всех внутренних каналов
	//Измерение стационарных каналов
			//Если задействованы потенциометры
			#if Enable_Potenciometr==1
	//Канал1
	TempResult.ChID=DeviceSerial*100+1;
	TempResult.Value=MeasSystem.CH1Val;
	TempResult.Variation=MeasSystem.CH1Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//Канал2
	TempResult.ChID=DeviceSerial*100+2;
	TempResult.Value=MeasSystem.CH2Val;
	TempResult.Variation=MeasSystem.CH2Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//Канал3
	TempResult.ChID=DeviceSerial*100+3;
	TempResult.Value=MeasSystem.CH3Val;
	TempResult.Variation=MeasSystem.CH3Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//Канал4
	TempResult.ChID=DeviceSerial*100+4;
	TempResult.Value=MeasSystem.CH4Val;
	TempResult.Variation=MeasSystem.CH4Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//Канал5
	TempResult.ChID=DeviceSerial*100+5;
	TempResult.Value=MeasSystem.CH5Val;
	TempResult.Variation=MeasSystem.CH5Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//Канал6
	TempResult.ChID=DeviceSerial*100+6;
	TempResult.Value=MeasSystem.CH6Val;
	TempResult.Variation=MeasSystem.CH6Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//Канал7 (CHPowerRes)
	TempResult.ChID=DeviceSerial*100+7;
	TempResult.Value=MeasSystem.CHPowerRes;
	TempResult.Variation=MeasSystem.CHPowerRes;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
			#endif
	//Канал10(.Bat1Level)
	TempResult.ChID=DeviceSerial*100+10;
	TempResult.Value=MeasSystem.Bat1Level;
	TempResult.Variation=MeasSystem.Bat1Level;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//Проверка напряжения внешнего аккумулятора
	if ((TempResult.Value>1)&(TempResult.Value<11))
		{
		GPIO_ResetBits(Power_Port, Power_Pin); Delay_us(3000000); //Выключается питание устройства
		return; //Выход из функции
		}
	//Канал11 //Bat2Level
	TempResult.ChID=DeviceSerial*100+11;
	TempResult.Value=MeasSystem.Bat2Level;
	TempResult.Variation=MeasSystem.Bat2Level;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//Канал12 режим работы(Командный режим-1, Режим измерений-0)
	if (DeviceMode==DeviceModeCommand) //Если в командном режиме
		{
		TempResult.ChID=DeviceSerial*100+12;
		TempResult.Value=1;
		TempResult.Variation=1;
		TempResult.Temperature=MeasSystem.Temperature;
		Save_Record(TempResult);
		}
	else
		{
		TempResult.ChID=DeviceSerial*100+12;
		TempResult.Value=0;
		TempResult.Variation=0;
		TempResult.Temperature=MeasSystem.Temperature;
		Save_Record(TempResult);
		}
	//Измерения на линии RS485
	GPIO_SetBits(Power_RS485_Port, Power_RS485_Pin); //Включается питание линии RS485
	Delay_us(3000000); //Задержка 3 с
	//В массив с результатами измерений заносятся результаты опроса внешних устройств
	for (i = 0; i < 64; ++i)
		{
		if (ChIDList[i]!=0)
			{
			//Каждые 4 измерения выключается питание на 10 секунд (для уменьшения нагрузки на линию и уменьшения нагрева линейного регулятора)
			if ((i==4)||(i==8)||(i==12)||(i==16)||(i==20)||(i==24)||(i==28)||(i==32)||(i==36)||(i==40)||(i==44)||(i==48)||(i==52)||(i==56)||(i==60))
				{
				GPIO_ResetBits(Power_RS485_Port, Power_RS485_Pin); //Выключается питание линии RS485
				Delay_us(18000000); //Задержка 18 с
				GPIO_SetBits(Power_RS485_Port, Power_RS485_Pin); //Включается питание линии RS485
				Delay_us(2000000); //Задержка 2 с
				}
			//Измерение - 2 попытки
			RS485.Attempt=2; RS485.CorrectResultFlag=0;
			while ((RS485.Attempt>0)&&(RS485.CorrectResultFlag==0))
				{
				RS485_GetValue_Sensor(10,ChIDList[i],6000); //Запрос на измерение, таймаут 6 секунд
				RS485.Attempt--;
				}
			if (RS485.CorrectResultFlag!=0)
				{
				TempResult.ChID=ChIDList[i];
				TempResult.Value=RS485.Val;
				TempResult.Variation=RS485.Var;
				TempResult.Temperature=RS485.Temp;
				Save_Record(TempResult);
				}
			}
		}
	GPIO_ResetBits(Power_RS485_Port, Power_RS485_Pin); //Выключается питание линии RS485
	}























////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////                       Основная программа                             ////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(void)
	{
	TMemoryResult TempResult;
	unsigned int i; //Счетчик
	SystemInit(); // Настройки тактирования
	__enable_irq(); //Разрешаются глобальные прерывании
	//Тактирование портов
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	Power_Init(); //Инициализация системы питания
	GPIO_SetBits(Power_BT_GSM_Port, Power_BT_GSM_Pin); //Включается питание Bluetooth и GSM модулей
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	//Инициализация подсистем
	Timeouts.ForceRestart=MaximumTimeForceRestart; //Устанавливается время принудительной перезагрузки
	Main_Timer_Init(); //Инициализация основного таймера
	Read_Parametr_Device(); //Чтение параметров устройства
	//Инициализация часов реального времени
	if (RTC_GetFlagStatus(RTC_FLAG_INITS)==RESET) //Если часы не инициализированы
		{
		RTC__Init();
		//Установка текущего времени
		DateTime.DD=31;	DateTime.MM=12;	DateTime.YY=01; DateTime.hour=23; DateTime.min=57; DateTime.sec=57;
		RTC_Set_Date_Time();
		//Установка времени будильника
		CycleTime.hour=23; CycleTime.min=58; CycleTime.sec=20;
		RTC_Set_Alarm_Time();
		//Очищается список каналов
		for (i = 0; i < 64; ++i) {ChIDList[i]=0;}
		Save_ChIDList(); //сохраняется в SRAM
		}
	RTC_Set_Next_Cycle_Time(); //Установить время следующего измерения
				//Инициализация подсистем
	//Инициализация измерительной системы
	Meas_System_Init();//Функция инициализации измерительной системы
	RS485_Init(); //Инициализация RS485
	GPIO_ResetBits(Power_RS485_Port, Power_RS485_Pin); //Выключается питание линии RS485
	//Выбор параметров работы
	if (DeviceMode==DeviceModeCommand) //Если в командном режиме
		{
		Delay_us(3000000); //Задержка 3 с
		Bluetooth_Init();
		Test_Chanela_Blooetooth();
		GSM_Init();
		Delay_us(1000000); //Задержка 1 с
		Timeouts.TimeoutIdle=Idle_Timeout; //Устанавливается таймаут простоя устройства
		Delay_us(3000000); //Задержка 1 с
		GSM_Send_Alarm_SMS(); //Передача SMS сообщения в случае открытия ящика
		while (Timeouts.TimeoutIdle>100) {};
		}
	else  //Если в режиме измерений
		{
		Start_Led(1000);//Функция запуска таймера светодиода
		MeasCycle(); //Измерение цикла
		//Если необходима передача данных
		if (DeviceMode==DeviceModeCycleFTP)
			{
			Timeouts.TimeFTPSession=0; //Сбрасывается счетчик времени FTP сессии
			Start_Led(200);//Функция запуска таймера светодиода
			//Power_ON_BT_GSM(); //Включается питание Bluetooth и GSM модулей
			Delay_us(3000000); //Задержка 3 с
			Bluetooth_Init();
			Test_Chanela_Blooetooth();
			GSM_Init();
			Delay_us(5000000); //Задержка 5 с
			GSM_Send_Data_FTP();
			//Прием и анализ всех СМС
			GSM_Read_ALL_SMS(5000);
			//Удаление всех СМС
			GSM_Delete_All_SMS(5000);
			}
		}
	if (DeviceMode==DeviceModeCommand) {GSM_Send_Alarm_FTP();} //Если в командном режиме то отправляется сообщение об открытии двери
	RTC_Set_Next_Cycle_Time(); //Установить время следующего измерения
	GPIO_ResetBits(Power_Port, Power_Pin); Delay_us(3000000); //Выключается питание устройства

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Если питание не отключено по причине открытой двери
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	unsigned int CounterSendAlarm=0;
	while (1)
		{
		Timeouts.ForceRestart=MaximumTimeForceRestart; //Устанавливается время принудительной перезагрузки
		Start_Led(50);//Функция запуска таймера светодиода
		RTC_Set_Next_Cycle_Time(); //Установить время следующего измерения
		GPIO_ResetBits(Power_Port, Power_Pin); //Выключается питание устройства
		Delay_us(3000000); //Задержка 3с
		GPIO_ResetBits(Power_BT_GSM_Port, Power_BT_GSM_Pin); //Выключается питание Bluetooth и GSM модулей
		//Ожидается время следующей передачи
		Timeouts.AlarmSend=PeriodAlarmSend;	while (Timeouts.AlarmSend>200) {};
		Start_Led(200);//Функция запуска таймера светодиода
		GPIO_SetBits(Power_BT_GSM_Port, Power_BT_GSM_Pin); //Включается питание Bluetooth и GSM модулей
		Delay_us(3000000); //Задержка 3 с
		//Bluetooth_Init();
		Delay_us(10000000); //Задержка 10с
		if ((CounterSendAlarm%4)==0)
			{
			GSM_Send_Alarm_SMS(); //Передача SMS сообщения в случае открытия ящика
			}
		GSM_Send_Alarm_FTP(); //Отправляется сообщение об открытии двери
		Delay_us(5000000); //Задержка 5с
		CounterSendAlarm=CounterSendAlarm+1;
		}
	}

