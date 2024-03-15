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

//������ ������������ �����������
#define ProgrammVersion_D 6 //����
#define ProgrammVersion_M 10 //�����
#define ProgrammVersion_Y 23 //���

#define Enable_Potenciometr 1 //������������� �������������
//#define Enable_Potenciometr 0 //�� ������������� �������������


//..........................................................................................................................
//..........................................................................................................................
//.................                            ��������� ����������                                         ................
//..........................................................................................................................
//.................                            ��������� ����������                                         ................
//..........................................................................................................................
//.................                            ��������� ����������                                         ................
//..........................................................................................................................
//.................                            ��������� ����������                                         ................
//..........................................................................................................................
//.................                            ��������� ����������                                         ................
//..........................................................................................................................
//.................                            ��������� ����������                                         ................
//..........................................................................................................................
//.................                            ��������� ����������                                         ................
//..........................................................................................................................
//.................                            ��������� ����������                                         ................
//..........................................................................................................................
//.................                            ��������� ����������                                         ................
//..........................................................................................................................
//..........................................................................................................................


//���������� ����������
#define Prioritet_MainTimer 0 //������������ ��������� ��������� ������� �������������� ����� � ��������
#define PrioritetReciveGSM 1 //��������� ������ ����� �� GSM ������
#define PrioritetBluetooth 2 //��������� ������� bluetooth
#define Prioritet_LedTimer 3 //��������� ������� ����������

//�������� ��������� ��� ��������� �������� (������� ������� 20 ���)
#define Send_Char_Timeout  10000 //������� �������� ����� �� Usart 10��
#define ADC_Timeout  5000 //��������� ��� 5��
#define Send_MessageBT_Timeout  2000000 //������� ������� ��������� �� bluetooth 2�
#define Idle_Timeout  120000000 //������� ������� �� ���������� ������� 120�
#define MaximumTimeForceRestart 1800000000; //����� �������������� ������������ 30 �����
#define PeriodAlarmSend 1200000000; //������������� � ������� ������������ ��������� �� �������� ����� 20 �����
#define MinimumPeriod  60 //����������� ������ ��������� ������������� ������, ���
#define MaximumPeriodSendData  10 //������������ ���������� �������� �������� ������




















//..........................................................................................................................
//..........................................................................................................................
//.................                        ���������� �������� � �������                                    ................
//..........................................................................................................................
//.................                        ���������� �������� � �������                                    ................
//..........................................................................................................................
//.................                        ���������� �������� � �������                                    ................
//..........................................................................................................................
//.................                        ���������� �������� � �������                                    ................
//..........................................................................................................................
//.................                        ���������� �������� � �������                                    ................
//..........................................................................................................................
//.................                        ���������� �������� � �������                                    ................
//..........................................................................................................................
//.................                        ���������� �������� � �������                                    ................
//..........................................................................................................................
//.................                        ���������� �������� � �������                                    ................
//..........................................................................................................................
//.................                        ���������� �������� � �������                                    ................
//..........................................................................................................................
//..........................................................................................................................


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////      ������� ��� ������ � ���������� ������� ����������     ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//������� flash ������
//0. 0x08000000-0x08003FFF (16 ��) ����������� ���������. ������ �0-�4 (128 ��)
//1. 0x08004000-0x08007FFF (16 ��) ����������� ���������. ������ �0-�4 (128 ��)
//2. 0x08008000-0x0800BFFF (16 ��) ����������� ���������. ������ �0-�4 (128 ��)
//3. 0x0800C000-0x0800FFFF (16 ��) ����������� ���������. ������ �0-�4 (128 ��)
//4. 0x08010000-0x0801FFFF (64 ��) ����������� ���������. ������ �0-�4 (128 ��)
//5. 0x08020000-0x0803FFFF (128 ��) ����� ����������� ���������. ������ �5 (128 ��)
//6. 0x08040000-0x0805FFFF (128 ��) ��������� ����������. ������ �6 (128 ��)
//7. 0x08060000-0x0807FFFF (128 ��) ����� ���������� ����������. ������ �7 (128 ��)
//8. 0x08080000-0x0809FFFF (128 ��) ���������. ������ �8 (128 ��)
//9. 0x080A0000-0x080BFFFF (128 ��) ���������. ������ �9 (128 ��)
//10. 0x080C0000-0x080DFFFF (128 ��) ���������. ������ �10 (128 ��)
//11. 0x080E0000-0x080FFFFF (128 ��) ���������� ������������. ������ �11 (128 ��)

#define ProgrammatorAddress 0x080E0000 //����� ������� ����� � ������� ��������� ���������� ������������
#define SectorProgrammBuf FLASH_Sector_5 //������ Flash ������ � ������� ���������� �������� ����������
#define ProgrammAddressBuf 0x08020000 //����� ������� flash � ������� ���������� �������� ����������
#define ParametrAddress 0x08040000 //����� ������� flash � ������� �������� ��������� ����������
#define SectorParametr FLASH_Sector_6 //������ Flash ������ � ������� �������� ��������� ����������
#define ParametrAddressBuf 0x08060000 //����� ������� flash � ������� ������������ ��������� ����������
#define SectorParametrBuf FLASH_Sector_7 //������ Flash ������ � ������� ������������ ��������� ����������
#define ParametrFlagWriteAddress 0x0805FFFC //����� ����� ������������ ������ ������� ����������

#define SectorMeas1 FLASH_Sector_8 //������ Flash ������ � ������� ������������ ���������� ��������� �1
#define Meas1Address 0x08080000 //����� ����������� ��������� �1
#define SectorMeas2 FLASH_Sector_9 //������ Flash ������ � ������� ������������ ���������� ��������� �2
#define Meas2Address 0x080A0000 //����� ����������� ��������� �2
#define SectorMeas3 FLASH_Sector_10 //������ Flash ������ � ������� ������������ ���������� ��������� �3
#define Meas3Address 0x080C0000 //����� ����������� ��������� �3

#define GPRS_Settings_String_Address 0x40024000 //(256 ����) ����� ������ ������������� GPRS ��N
#define GPRS_UserPass_String_Address 0x40024100 //(256 ����) ����� ������ ������ � ������ GPRS
#define FTP_Settings_String_Address 0x40024200 //(256 ����) ����� ������ �������� FTP
#define FTP_File_String_Address 0x40024300 //(256 ����) ����� ������ �������� ����� �����
#define ChID_List_Address 0x40024400 //(256 ����) ����� ������ �������
#define SMS_Number_String_Address 0x40024500 //(13 ����) ����� ������ [12] ������ ��������, �� ������� ����� ��������� ������
#define Cycle_Period_Address 0x40024510 //����� ������� ���������
#define Period_Send_Data_Address 0x40024520 //����� ���������� �������� ������ �������� ������
#define Current_Period_Send_Data_Address 0x40024530 //����� �������� ���������� �������� ������ �������� ������
#define FTP_File_Append_Address 0x40024540 //����� ����� "���������� � ����� �����"
#define SMS_Code_String_Address 0x40024550 //����� ���� ������� �� SMS

//��������� ����������
unsigned char DeviceType=22;
unsigned char DeviceAddress=255; //����� ���������� �� ���������
unsigned int DeviceSerial=0;//�������� ����� ����������
unsigned int ServiceCode=0; //��� ���������� ������ (141592 - �������� ���������, 832735 - ����������, 793238 - ��������� ��������)
unsigned int CyclePeriod=3600; //������ �������������� ����� (�������)
unsigned int CycleStart=0; //��������� ����� ���������, ������� �� ������ �����
unsigned int CycleSendDataPeriod=0; //���������� ��������� ������������� ������ �� �������� ������, ��.
unsigned int CurrentCycleSendDataPeriod=0; //������� ���������� ��������� ������������� ������ �� �������� ������, ��.

//���� ������ {YY,MM,DD,Hour,  min,sec,Temp1,Temp2,  Val1,Val2,Val3,Val4,  Var1,Var2,Var3,Var4}
typedef struct //���������, ���������� � ���� ���������� ���������, ���������� � ������ ����������
	{
	unsigned char YY; //����
	unsigned char MM; //������
	unsigned char DD; //���
	unsigned char hour; //����
	unsigned char min; //������
	unsigned char sec; //�������
	float Temperature; //�����������
	float Value; //���������� ��������
	float Variation; //�������� ����������� ��������
	unsigned int ChID; //�� ������
	unsigned char FlagMeas; //���� ������� ������
	unsigned char FlagFTP; //���� �������� ������ �� FTP
	}TMemoryResult;
volatile TMemoryResult MemoryResult[128]; //���������� ��� �������� ����������� ���������

unsigned char Check_Float(uint32_t Address); //�������, ������������ �������� ����� � ��������� ������ �� NAN, INF
unsigned int Calculate_CRC(uint32_t StartAddress, uint32_t Length); //�������, �������������� ������ ����������� ����� ������� flash ������
unsigned char Recive_Data_To_Flash(uint32_t Address, uint32_t Length, uint16_t Sector, unsigned int CRC32); //�������, �������������� ����� ������ � ������ �� � flash ������//������� ���������� 0 ���� �� ��������� ����������� �����
void GetCycle_Settings(void); //�������, �������������� ������ ���������� ���������� �� SRAM ������
void SetCycle_Settings(void); //�������, �������������� ���������� ���������� ���������� � SRAM ������
void Save_FTP_Settings(void);//�������, ����������� ��������� FTP � SRAM
void Read_FTP_Settings(void);//�������, �������� ��������� FTP �� SRAM
void Save_GPRS_Settings(void);//�������, ����������� ��������� FTP � SRAM
void Read_GPRS_Settings(void);//�������, �������� ��������� FTP �� SRAM
void Save_GPRS_User_Pass(void); //�������, ����������� GPRS_User_Pass � SRAM
void Read_GPRS_User_Pass(void); //�������, �������� GPRS_UserPass �� SRAM
void Save_FTP_FileName(void); //�������, ����������� ��� ����� FTP � SRAM
void Read_FTP_FileName(void); //�������, �������� ��� ����� FTP �� SRAM
void Save_SMS_Settings(void); //�������, ����������� ��������� ���������� �� SMS � SRAM                      //
void Read_SMS_Settings(void); //�������, ����������� ��������� ���������� �� SMS �� SRAM                     //
void Save_ChIDList(void); //�������, ����������� ChIDList SRAM
void Read_ChIDList(void); //�������, �������� ChIDList SRAM
void Save_Record(TMemoryResult Write_MeasResult); //�������, ����������� ���������� �������� �� flash ������
TMemoryResult Read_Record(unsigned int IndexRecord); //�������, �������� ������ �� flash ������
void ClearFTPFlags(void); //�������, ��������� ��� ����� FTP
void Read_Parametr_Device(void); //�������, �������������� ������ ������������� ������ �� flash ������
void Copy_Parametr_Buf(void); //�������, �������������� ����������� ���������� ���������� �� ������ � ���������� ���������
void Jump_To_Application(uint32_t Address); //�������, �������������� ������� � ���������� ������� ���������� �������� ���������� �� ����� ������� ������ � ������

//������� ��� ������� CRC32
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
///////////////////////////         ������� ���������� �������� � ���������          ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//�������� ������
#define Timer_Main  TIM2
#define Timer_Main_IRQn TIM2_IRQn
#define RCC_APB1Periph_TIM_Main RCC_APB1Periph_TIM2

typedef struct //���������, ���������� � ���� �������� ��������� ���������
	{
	unsigned int DelayMain; //������� ��� ������� �������� � �������� ���������
	unsigned int DelayBluetooth; //������� ��� ������� �������� � ���������� bluetooth
	signed int TimeoutGSM_Char; //������� �������� ��� �������� ������� � GSM ������
	signed int TimeoutGSM_AT; //������� �������� ���������� AT ������� GSM �������
	signed int TimeoutADC; //������� �������� ��������� ���
	signed int TimeoutSendMessageBluetooth; //������� �������� ��� �������� ��������� �� bluetooth
	signed int TimeoutIdle; //������� �������� ������� ����������
	signed int TimeoutRS485_Char; //������� �������� ��� �������� ������� � RS485 ������
	signed int TimeoutRS485_Instruction; //������� �������� ���������� ����������
	unsigned int ForceRestart; //������� �������� ������������� ������������, ���
	unsigned int AlarmSend; //������� ������� �������� ��������� �� �������� �����, ���
	unsigned int TimeFTPSession; //����� FTP ������, ���
	}TTimeouts;
volatile TTimeouts Timeouts; //���������� ��� �������� ��������� ��������� � ��������


void Main_Timer_Init(void); //������� ������������� ��������� �������
void TIM2_IRQHandler(void); //������� ������������ ��������� �������
void Delay_us(unsigned int Val); //������� ��������, ���
void Delay_us_Bluetooth(unsigned int Val); //������� �������� � ���������� bluetooth, ���


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////                ������� ��� ������ � RTC               ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct //���������, ���������� � ���� �������� ���� � �������
	{
	uint8_t WD; //���� ������
	uint8_t DD; //����
	uint8_t MM; //�����
	uint8_t YY; //���
	uint8_t hour; //����
	uint8_t min; //������
	uint8_t sec; //�������
	}TDateTime;
volatile TDateTime DateTime; //���������� ��� �������� ���� � �������

typedef struct //���������, ���������� � ���� �������� ������� �������������� �����
	{
	uint8_t hour; //����
	uint8_t min; //������
	uint8_t sec; //�������
	}TCycleTime;
volatile TCycleTime CycleTime; //���������� ��� �������� ������� ������������ �����������

typedef enum _TDeviceMode //����� ������ ����������
	{
	DeviceModeCycle=0, //����� ����������� ���������
	DeviceModeCycleFTP=1, //����� ����������� ��������� c ��������� ������ �� FTP ������
	DeviceModeCommand=2, //����� ��������� ����������
	}
TDeviceMode;
volatile TDeviceMode DeviceMode=DeviceModeCommand; //����� ������ ����������

void RTC__Init(void); //������� ������������� ����� ��������� �������
void RTC_Get_Date_Time(void); //������� ��������� ������� ������� �������
void RTC_Set_Date_Time(void); //������� ��������� ������� ���� � �������
void RTC_Get_Alarm_Time(void); //������� ��������� ������� ����������
void RTC_Set_Alarm_Time(void); //������� ��������� ������� ����������
void RTC_Set_Next_Cycle_Time(void); //������� ��������� ������� ������������ ����������
void Get_DeviceMode(void); //������� ��������� ������ ������ ����������


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////               ������� ��� ������ � Bluetooth          ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//����� RESET Bluetooth ������ (������ ������� - ������ �� �������,  ������� ������� - ������ �������)
#define BT_Reset_Port  GPIOA
#define BT_Reset_Pin  GPIO_Pin_8

//����� PIO_11 Bluetooth ������ (������ ������� - ����� �������� ������,  ������� ������� - AT �����)
#define BT_AT_Port  GPIOC
#define BT_AT_Pin  GPIO_Pin_9

//Rx USART �����������
#define BT_Rx_Port  GPIOA
#define BT_Rx_Pin  GPIO_Pin_10

//Tx USART �����������
#define BT_Tx_Port  GPIOA
#define BT_Tx_Pin  GPIO_Pin_9

//USART Bluetooth ������
#define RCC_APB2Periph_USART_BT RCC_APB2Periph_USART1
#define USART_BT USART1

//������ Bluetooth
#define Timer_Bluetooth  TIM4
#define Timer_Bluetooth_IRQn TIM4_IRQn
#define RCC_APB1Periph_TIM_Bluetooth RCC_APB1Periph_TIM4

typedef struct //���������, ���������� � ���� ����� �� 2048 ��������
	{
	unsigned char Buffer[2048]; //�����
	unsigned int Buffer_Len; //������ ������������ � ������ ���������
	unsigned int Buffer_Pos; //������� �������, ��������������� ��� ������ � �������
	unsigned char Start_Collect_Flag; //���� ������ ��������� (1 - ���� ������� ������� ���������, 0 - ��������� ������ ������ ���������)
	unsigned char Previous_Byte; //���������� ����������� �� USART ����
	unsigned char Current_Byte; //������� ����������� �� USART ����
	}TMessageSelectionBluetooth;
TMessageSelectionBluetooth MessageSelectionBluetooth; //����������, ���������� � ���� ����� �� 2048 ��������

typedef struct //���������, ������������ ��� ������������� ���������
	{
	//����������� ������ ������ � ���������
	unsigned int TypePosStart; //������ ����� "��� ���������"
	unsigned int TypePosEnd; //����� ����� "��� ���������"
	unsigned int AddressPosStart; //������ ����� "�����"
	unsigned int AddressPosEnd; //����� ����� "�����"
	unsigned int TransactionPosStart; //������ ����� "����� ����������"
	unsigned int TransactionPosEnd; //����� ����� "����� ����������"
	unsigned int InstructionPosStart; //������ ����� "����������"
	unsigned int InstructionPosEnd; //����� ����� "����������"
	unsigned int DataPosStart; //������ ����� "������"
	unsigned int DataPosEnd; //����� ����� "������"
	unsigned char CorrectMessageFlag; //���� ������������ ��������� ��������� (1 - ���������� ���������, 0 - ������������ ���������)
	unsigned char Address; //�����  ����������, �������� ������������� ���������
	}TMessageInterpretationBluetooth;
TMessageInterpretationBluetooth MessageInterpretationBluetooth; //����������, ������������ ��� ������������� ���������

void Bluetooth_Init(void); //������� ������������� bluetooth ������
void Bluetooth_Send_AT(const unsigned char* str); //������� ������� bluetooth ������ AT �������
void Bluetooth_Send_Message(void); //������� ������� ��������� �� bluetooth
void Bluetooth_Interpretation_Message(void); //������� ���������� ������������� ��������� �������� �� Bluetooth
void TIM4_IRQHandler(void); //�������, ���������� �� ���������� ������� Bluetooth
void Bluetooth_Unsigned_Int_To_Char_Buf(unsigned int Val, unsigned int StartPos, unsigned int EndPos); //������� ���������� �������������� ������ �������������� ����� � ������ � �������� �� � ����� Bluetooth
unsigned int Bluetooth_Char_To_Unsigned_Int_Buf(unsigned int StartPos, unsigned int EndPos); //������� ���������� �������������� ����� ������, ����������� � ������ Bluetooth, � ����� ����� ��� �����
signed int Bluetooth_Char_To_Signed_Int_Buf(unsigned int StartPos, unsigned int EndPos); //������� ���������� �������������� ����� ������, ����������� � ������ Bluetooth, � ����� ����� �� ������
void Bluetooth_HexChar_Buf_To_GPRS_Settings(unsigned int StartPos, unsigned int EndPos); // ������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ Bluetooth, � ���������� ������ GSM.GPRS_Settings /////
void Bluetooth_HexChar_Buf_To_FTP_Settings(unsigned int StartPos, unsigned int EndPos); //������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ Bluetooth, � ���������� ������ GSM.FTP_Settings /////
void Bluetooth_HexChar_Buf_To_GPRS_User_Pass(unsigned int StartPos, unsigned int EndPos); //������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ Bluetooth, � ���������� ������ GSM.GPRS_User_Pass ////
void Bluetooth_HexChar_Buf_To_FTP_FileName(unsigned int StartPos, unsigned int EndPos); // ������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ Bluetooth, � ���������� ������ GSM.GPRS_User_Pass ////
void Bluetooth_Send_ErrorData(void);//������� �������� ����� ErrorData
void Bluetooth_Send_ErrorCh(void);//������� �������� ����� ErrorCh
void Bluetooth_Send_ErrorSensor(void);//������� �������� ����� ErrorSensor
void Bluetooth_Send_End(void);//������� �������� ����� End
void Bluetooth_Send_CRCError(void);//������� �������� ����� CRCError
void Bluetooth_Send_CRC_Ok(void);//������� �������� ����� CRC_Ok
void Bluetooth_Execution_GetType(void);//������� ���������� ���������� ���������� GetType
void Bluetooth_Execution_GetSerial(void);//������� ���������� ���������� ���������� GetSerial
void Bluetooth_Execution_GetProgVersion(void);//������� ���������� ���������� ���������� GetProgVersion
void Bluetooth_Execution_UploadProgramm(void);//������� ���������� ���������� ���������� UploadProgramm
void Bluetooth_Execution_UploadSettings(void); //������� ���������� ���������� ���������� UploadSettings
void Bluetooth_Execution_DownloadSettings(void); //������� ���������� ���������� ���������� DownloadSettings
void Bluetooth_Execution_SetServiceMode(void); //������� ���������� ���������� ���������� SetServiceMode
void Bluetooth_Execution_SetCycleSettings(void); //������� ���������� ���������� ���������� SetCycleSettings
void Bluetooth_Execution_GetCycleSettings(void); //������� ���������� ���������� ���������� GetCycleSettings
void Bluetooth_Execution_SetClock(void); //������� ���������� ���������� ���������� SetClock
void Bluetooth_Execution_GetClock(void); //������� ���������� ���������� ���������� GetClock
void Bluetooth_Execution_SetFTPSettings(void); //������� ���������� ���������� ���������� SetFTPSetting
void Bluetooth_Execution_GetFTPSettings(void); //������� ���������� ���������� ���������� GetFTPSetting
void Bluetooth_Execution_SetGPRSSettings(void); //������� ���������� ���������� ���������� SetGPRSSetting
void Bluetooth_Execution_GetGPRSSettings(void); //������� ���������� ���������� ���������� GetGPRSSetting
void Bluetooth_Execution_SetGPRSUserPass(void); //������� ���������� ���������� ���������� SetGPRSUserPass
void Bluetooth_Execution_GetGPRSUserPass(void); //������� ���������� ���������� ���������� GetGPRSUserPass
void Bluetooth_Execution_SetFTPFileName(void); //������� ���������� ���������� ���������� SetFTPFileName
void Bluetooth_Execution_GetFTPFileName(void); //������� ���������� ���������� ���������� GetFTPFileName
void Bluetooth_Execution_SetSMSSettings(void); //������� ���������� ���������� ���������� SetSMSSettings
void Bluetooth_Execution_GetSMSSettings(void); //������� ���������� ���������� ���������� GetSMSSettings
void Bluetooth_Execution_SetChIDList(void); //������� ���������� ���������� ���������� SetChIDList
void Bluetooth_Execution_GetChIDList(void); //������� ���������� ���������� ���������� GetChIDList
void Bluetooth_Execution_DownloadData(void); //������� ���������� ���������� ���������� DownloadData
void Bluetooth_Execution_Test(void); //������� ���������� ���������� ���������� Test
void SetNameBlooetooth(void); //������� ���������� ��������� ����� bluetooth ������

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////         ������� ������� ������� ����������            ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//����� ��������� ������� ���������� (0-���������� ���������, 1-���������� ��������)
#define Power_Port GPIOA
#define Power_Pin GPIO_Pin_15

//����� ������� Bluetooth � GSM ������� (0-������ ���������, 1-������ ��������)
#define Power_BT_GSM_Port GPIOB
#define Power_BT_GSM_Pin GPIO_Pin_0

//����� ������� ��������� �� ����� RS485 (0-������� ����� ���������, 1-������� ����� ��������)
#define Power_RS485_Port GPIOC
#define Power_RS485_Pin GPIO_Pin_4

//����� ����������
#define Led_Port GPIOC
#define Led_Pin GPIO_Pin_8

//������ ����������
#define Timer_Led  TIM3
#define Timer_Led_IRQn TIM3_IRQn
#define RCC_APB1Periph_TIM_Led RCC_APB1Periph_TIM3

void Power_Init(void);//������� ���������� ������������� ������� �������
void Power_ON_BT_GSM(void); //       ������� ���������� ������� ��������� ������� GSM � Bluetooth �������
void TIM3_IRQHandler(void);//������� ������������ ������� ����������
void Start_Led(unsigned int Period);//������� ������� ������� ����������
void Stop_Led();//������� ��������� ������� ����������


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////                  ������� ��� ������ � RS485                             ///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//����� RE_DE RS485 ������ (������ ������� - ����� ������,  ������� ������� - �������� ������)
#define RS485_RE_DE_Port GPIOA
#define RS485_RE_DE_Pin GPIO_Pin_4

//����� Rx (USART �����������)
#define RS485_Rx_Port  GPIOA
#define RS485_Rx_Pin  GPIO_Pin_3
#define RS485_Rx_GPIO_PinSource  GPIO_PinSource3

//����� Tx (USART �����������)
#define RS485_Tx_Port  GPIOA
#define RS485_Tx_Pin  GPIO_Pin_2
#define RS485_Tx_GPIO_PinSource  GPIO_PinSource2

//USART RS485 ������
#define RCC_APB1Periph_USART_RS485 RCC_APB1Periph_USART2
#define USART_RS485 USART2
#define GPIO_AF_USART_RS485 GPIO_AF_USART2

unsigned int ChIDList[64]; //������ ��������

typedef struct //���������, ���������� � ���� ��������� ������ ������ ������� � ��������� �� RS485
	{
	unsigned char Buffer[2048]; //�������� ����� USART
	unsigned int Buffer_Pos; //������� ��������� � ������
	unsigned int  Buffer_Len; //������ ������ � ������
	unsigned char Transaction; //���������������
	unsigned char Attempt; //���������� ������� ���������� �������
	unsigned char Message_Complete_flag; //���� ��������� ������ ���������
	unsigned char Start_Collect_Flag; //���� ������ �������� ������ ���������
	unsigned char Previous_Byte;
	unsigned char Current_Byte;
	unsigned char  CorrectResultFlag;
	float Val;
	float Var;
	float Temp;
	}TRS485;
volatile TRS485 RS485; //���������� ��� �������� ���������� RS485 ������
void RS485_Init(void);//������� ������������� RS85 ������
void RS485_Send_Char(unsigned char Byte);//�������� ������� � RS485 ������


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////           ������� ��� ������ � GSM �������            ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Rx (�����������) GSM
#define GSM_Rx_Port  GPIOC
#define GSM_Rx_Pin  GPIO_Pin_7
#define GSM_Rx_PinSource GPIO_PinSource7

//Tx (�����������) GSM
#define GSM_Tx_Port  GPIOC
#define GSM_Tx_Pin  GPIO_Pin_6
#define GSM_Tx_PinSource GPIO_PinSource6

//USART GSM ������
#define RCC_APB2Periph_USART_GSM RCC_APB2Periph_USART6
#define USART_GSM USART6
#define GPIO_AF_USART_GSM GPIO_AF_USART6

//������ ��������� ������ �� GSM ������
#define Timer_ReciveGSM  TIM5
#define Timer_ReciveGSM_IRQn TIM5_IRQn
#define RCC_APB1Periph_TIM_ReciveGSM RCC_APB1Periph_TIM5

typedef struct //���������, ���������� � ���� ��������� ������ GSM ������
	{
	unsigned char UsartRxNE; //���� ������� ����� � ������ USART GSM 0 - ����� ����
	unsigned char UsartInByte; //�������� � USART ����
	unsigned char Attempt; //���������� ������� ���������� �������
	unsigned int ATErrorFlag; //������ ���������� AT ������� (1-������ ���������� �������)
	char Buffer[256]; //�������� ����� ��� �������� ������ GSM ������
	char OutSMS[256]; //����� ��� �������� ��������� SMS ���������
	char SMSNumber[13]; //�����, �� ������� �������� SMS � ������������ ���������� ����������
	unsigned int SMSCode; //��� ������� � ���������� �����������
	char IMEI[16]; //IMEI ������
	unsigned char REG_MODE; //����� ����������� � ���� ���������
	unsigned char REG_STAT; //������ ����������� � ����
	unsigned char SQ_RSSI; //������� �������
	unsigned char SQ_BER; //������� ������
	char Name_Operator[16]; //��� ���������
	unsigned char Connected; //���� �������� ����������� � ���� 0-��� �����������
	char GPRS_Settings[256]; //
	char GPRS_User_Pass[256]; //
	unsigned char GPRS_Connected; //���� �������� ����������� � ���� 0-��� �����������
	char FTP_Settings[256]; //��������� FTP ��������� -  "server:port","username","password",mode
    char FTP_File_Name[256]; //��� ����� FTP
	unsigned int FTP_Append; //0-���������� �����, 1-���������� ������ � ����� �����
	unsigned char FTP_Connected; //���� ��������� ����������� � FTP ������� 0-��� �����������
	char FTP_Out_String[256]; //������ ��� ������ � ����
	}TGSM;
TGSM GSM; //���������� ��� �������� ���������� GSM ������


unsigned char FTPInProgress; //���� �������� �������� ������ (0x00 �������������� �� ������������)


void GSM_Init(void); //������� ������������� GSM ������
void TIM5_IRQHandler(void); //�������, ���������� �� ���������� ������� ��������� ����� �� GSM ������
unsigned char GSM_Recive_Char(void); //����� ������� �� GSM ������
void GSM_Send_Char(unsigned char Byte); //�������� ������� � GSM ������
void GSM_Send_String(void); //�������� ������ �� GSM.Buffer � GSM ������
void GSM_Send_AT(void); //�������� AT ������� �� GSM.Buffer � GSM ������
void GSM_Get_Signal_Quality(unsigned int Timeout_ms); //������� ��������� ������ ������� GSM
void GSM_Get_Network_Registration_Status(unsigned int Timeout_ms); //������� ��������� ������� ����������� � ���� GSM
void GSM_Get_Name_Operator(unsigned int Timeout_ms); //������� ��������� ����� ��������� GSM
void GSM_Get_IMEI(unsigned int Timeout_ms); //������� ��������� IMEI GSM ������
void GSM_GPRS_Close(unsigned int Timeout_ms); //������� �������� ������ GPRS
void GSM_GPRS_Set_PDP(unsigned int Timeout_ms); //������� ������� PDP ���������
void GSM_GPRS_Open(unsigned int Timeout_ms); //������� �������� ������ GPRS
void GSM_FTP_TO(unsigned int Timeout_ms); //������� ��������� �������� �������� ������ �� FTP �������
void GSM_DNS_Get_IP(unsigned int Timeout_ms); //������� IP ������ ������� �� DNS
void GSM_FTP_Open(unsigned int Timeout_ms); //������� �������� ���������� � FTP ��������
void GSM_FTP_Type(unsigned int Timeout_ms); //������� �������� ��������� ���� �������� �� FTP
void GSM_FTP_Open_File(unsigned int Timeout_ms); //������� �������� ����� FTP
void GSM_FTP_Close_File(unsigned int Timeout_ms); //������� �������� ����� FTP
void GSM_FTP_Send_string(unsigned int Timeout_ms); //������� �������� ��������� �������� �������� FTP
void GSM_Connect(void); //������� ����������� � ���� GSM
void GSM_GPRS_Connect(void); //������� ����������� � GPRS
void GSM_FTP_Connect(void); //������� ����������� � FTP �������
void GSM_Send_Data_FTP(void); //������� �������� ������ �� FTP ������
void GSM_Send_SMS(unsigned int Timeout_ms); //������� �������� SMS ��������� ��
void GSM_Send_Alarm_SMS(void); //������� �������� SMS ��������� � ������ �������� �����
void GSM_Send_Alarm_FTP(void); //������� �������� ������ �� FTP ������ � ������ �������� �����
void GSM_SMS_Execution_SetCycleSettings(void); //������� ���������� ���������� ���������� SetCycleSettings �� ���
void GSM_SMS_Execution_SetSMSSettings(void); //������� ���������� ���������� ���������� SetCycleSettings �� ���
void GSM_SMS_HexChar_Buf_To_FTP_Settings(unsigned int StartPos, unsigned int EndPos); // ������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ SMS, � ���������� ������ GSM.FTP_Settings
void GSM_SMS_Execution_SetFTPSettings(void); //������� ���������� ���������� ���������� SetFTPSettings �� ���
void GSM_SMS_HexChar_Buf_To_FTP_FileName(unsigned int StartPos, unsigned int EndPos); // ������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ SMS, � ���������� ������ GSM.GPRS_User_Pass
unsigned int GSM_SMS_Char_To_Unsigned_Int_Buf(unsigned int StartPos, unsigned int EndPos); //������� ���������� �������������� ����� ������, ����������� � ������ SMS, � ����� ����� ��� �����
void GSM_SMS_Execution_SetFTPFileName(void); //������� ���������� ���������� ���������� SetFTPFileName  �� ���
void GSM_Read_ALL_SMS(unsigned int Timeout_ms); //������ ���� SMS � ���������� ������
void GSM_Delete_All_SMS(unsigned int Timeout_ms); //�������� ���� SMS


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////          ������� ������� ��������� ����������             ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//���� ��� - ������� ������ ������������
#define Bat1_Level_Port  GPIOA
#define Bat1_Level_Pin  GPIO_Pin_6

//���� ��� - ������� ������ ������������
#define Bat2_Level_Port  GPIOA
#define Bat2_Level_Pin  GPIO_Pin_7

//���� ��� - ��������� (����������� ����������)
#define Termistor_Port  GPIOC
#define Termistor_Pin  GPIO_Pin_5

//���� ��� - ������������� (�������)
#define Power_Potenciometr_Port  GPIOA
#define Power_Potenciometr_Pin  GPIO_Pin_5

//���� ��� - ������������ (�����1)
#define CH1_Port  GPIOA
#define CH1_Pin  GPIO_Pin_0

//���� ��� - ������������ (�����2)
#define CH2_Port  GPIOC
#define CH2_Pin  GPIO_Pin_3

//���� ��� - ������������ (�����3)
#define CH3_Port  GPIOC
#define CH3_Pin  GPIO_Pin_2

//���� ��� - ������������ (�����4)
#define CH4_Port  GPIOC
#define CH4_Pin  GPIO_Pin_1

//���� ��� - ������������ (�����5)
#define CH5_Port  GPIOC
#define CH5_Pin  GPIO_Pin_0

//���� ��� - ������������ (�����6)
#define CH6_Port  GPIOA
#define CH6_Pin  GPIO_Pin_1

//��� ������������� �������
#define ADC_MeasSystem ADC1
#define RCC_APB2Periph_ADC_MeasSystem RCC_APB2Periph_ADC1

typedef struct //���������, ���������� � ���� ��������� ���������
	{
	unsigned int TermistorRes; //������������� ���������� ��� ���� �������� �������, ��
	unsigned int TermistorConstRes; //������������� ��������� � �������� ����������, ��
	unsigned int PowerConstRes; //������������� ��������� � �������� ����������, ��
	float Bat1Level; //������� ������ ������������, �
	float Bat2Level; //������� ������ ������������, �
	float Temperature; //���������� ����������� ����������, ������� �������
	float CHPowerVoltage; //��������� ��������� ���������� ������� ��������������, ��
	float CHPowerRes; //��������� ��������� ������������� ��������������, ��
	float CH1Val; //��������� ��������� ���������� �� ������������� ������ �1, ��/�
	float CH2Val; //��������� ��������� ���������� �� ������������� ������ �1, ��/�
	float CH3Val; //��������� ��������� ���������� �� ������������� ������ �1, ��/�
	float CH4Val; //��������� ��������� ���������� �� ������������� ������ �1, ��/�
	float CH5Val; //��������� ��������� ���������� �� ������������� ������ �1, ��/�
	float CH6Val; //��������� ��������� ���������� �� ������������� ������ �1, ��/�
	}TMeasSystem;
volatile TMeasSystem MeasSystem; //���������� ��� ����������� ��������� ����������

void Meas_System_Init(void); //������� ������������� ������� ��������� ����������
void Meas_System_GetValue(void); //������� ��������� ���������� �� ��������������
void MeasCycle(void); //������� ��������� �����




















//..........................................................................................................................
//..........................................................................................................................
//.................                            ��������� � �������                                          ................
//..........................................................................................................................
//.................                            ��������� � �������                                          ................
//..........................................................................................................................
//.................                            ��������� � �������                                          ................
//..........................................................................................................................
//.................                            ��������� � �������                                          ................
//..........................................................................................................................
//.................                            ��������� � �������                                          ................
//..........................................................................................................................
//.................                            ��������� � �������                                          ................
//..........................................................................................................................
//.................                            ��������� � �������                                          ................
//..........................................................................................................................
//.................                            ��������� � �������                                          ................
//..........................................................................................................................
//.................                            ��������� � �������                                          ................
//..........................................................................................................................
//..........................................................................................................................


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////        ������� ��� ������ � flash ������� ����������        ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////  �������, �������������� ������ ����������� ����� ������� flash ������  	/////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int Calculate_CRC(uint32_t StartAddress, uint32_t Length) //Length - ���������� 4-��������� ����
	{
	unsigned int i; //�������
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
/////////////                   �������, ������������ �������� ����� � ��������� ������ �� NAN, INF       //////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char Check_Float(uint32_t Address) //���������� �������� 0x00 � ������ ���������� ��������� �������� float
	{
	uint32_t Addr;
	unsigned char Byte0; //���� 0 (�������)
	unsigned char Byte1; //���� 1 (�������)
	Addr=Address+3; Byte0=*(unsigned char*)Addr;
	Addr=Address+2; Byte1=*(unsigned char*)Addr;
	if (((Byte0&0x7f)==0x7f)&((Byte1&0x80)==0x80)) //���� ���� ������� ����� ��������
		{return 0x00;}
	return 0xff;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////          �������, �������������� ����� ������ � ������ �� � flash ������       //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char Recive_Data_To_Flash(uint32_t Address, uint32_t Length, uint16_t Sector, unsigned int CRC32) //������� ���������� 0 ���� �� ��������� ����������� �����
	{
	unsigned int i; //��������
	unsigned char DataByte0; //0-� ���� ��� ����� �����
	unsigned char DataByte1; //1-� ���� ��� ����� �����
	unsigned char DataByte2; //2-� ���� ��� ����� �����
	unsigned char DataByte3; //3-� ���� ��� ����� �����
	unsigned int LocalCRC; //����������� ����� ��� �������� ������
	uint32_t Data4Byte; //�����, ������������ � ������
	uint32_t WriteAddress; //����� ������� � ������� ������������ ������
		//������ ������ � flash ������
	USART_ClearFlag(USART_BT, USART_FLAG_RXNE); //������� ����� ������� ����� � ������ USART
	FLASH_Unlock(); //����������� �������������� ������
	//������� ������
	FLASH_EraseSector(Sector,VoltageRange_3); //������� ������� Flash ������ � ������� ���������� ������
	//������ ������
	Bluetooth_Send_Message(); //���������� ����� �� Bluetooth
	WriteAddress=Address; //����� ������� � ������� ������������ ������
	for (i = 0; i < Length; ++i) //����������� ������ �� flash ������
		{
		//����������� 4 �����
		//����0
		while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //��������� �������� ����� � ����� bluetooth
		DataByte0=USART_ReceiveData(USART_BT); //���������� ���� �� ������ Usart
		//����1
		while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //��������� �������� ����� � ����� bluetooth
		DataByte1=USART_ReceiveData(USART_BT); //���������� ���� �� ������ Usart
		//����2
		while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //��������� �������� ����� � ����� bluetooth
		DataByte2=USART_ReceiveData(USART_BT); //���������� ���� �� ������ Usart
		//����3
		while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //��������� �������� ����� � ����� bluetooth
		DataByte3=USART_ReceiveData(USART_BT); //���������� ���� �� ������ Usart
		//����� �� 4-� ���� ������������ � flash ��������
		Data4Byte=DataByte3*0x1000000+DataByte2*0x10000+DataByte1*0x100+DataByte0*0x1;
		FLASH_ProgramWord(WriteAddress,Data4Byte);
		WriteAddress+=4;
		}
	FLASH_Lock(); //���������� ������ ������ �� ��������������
	//�������� ����������� �����
	LocalCRC=Calculate_CRC(Address, Length); //������ ����������� �����
	if (LocalCRC==CRC32) //���� ��������� ����������� �����
		{return 0xff;}
	else //���� �� ��������� ����������� �����
		{return 0x00;}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////  �������, �������������� ������ ���������� ����� �� SRAM ������       ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GetCycle_Settings(void)
	{
	uint32_t Address;
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
	RTC_Get_Alarm_Time();
	Address=Cycle_Period_Address; CyclePeriod=*(unsigned int*)Address; //������ �������������� ����� (�������)
	CycleStart=CycleTime.hour*3600+CycleTime.min*60+CycleTime.sec; //��������� ����� ���������, ������� �� ������ �����
	Address=Period_Send_Data_Address; CycleSendDataPeriod=*(unsigned int*)Address; //���������� ��������� ������������� ������ �� �������� ������, ��.
	Address=Current_Period_Send_Data_Address; CurrentCycleSendDataPeriod=*(unsigned int*)Address; //���������� ��������� ������������� ������ �� �������� ������, ��.
	PWR_BackupAccessCmd(DISABLE); //����������� ������ ���������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////  �������, �������������� ���������� ���������� ����� � SRAM ������       /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetCycle_Settings(void)
	{
	uint32_t Address;
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
	Address=Cycle_Period_Address; *(unsigned int*)Address=CyclePeriod; //������ �������������� ����� (�������)
	Address=Period_Send_Data_Address; *(unsigned int*)Address=CycleSendDataPeriod; //���������� ��������� ������������� ������ �� �������� ������, ��.
	if (CurrentCycleSendDataPeriod>Period_Send_Data_Address) {CurrentCycleSendDataPeriod=0;};
	Address=Current_Period_Send_Data_Address; *(unsigned int*)Address=CurrentCycleSendDataPeriod; //���������� ��������� ������������� ������ �� �������� ������, ��.
	//��������� ��������������� �� ��������� ����� ���������, ������� �� ������ �����
	CycleTime.hour=(CycleStart/3600)%24;
	CycleTime.min=(CycleStart/60)%60;
	CycleTime.sec=(CycleStart)%60;
	RTC_Set_Alarm_Time();
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           �������, ����������� ��������� FTP � SRAM                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_FTP_Settings(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //�����, ������������ � ������
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
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
//                                           �������, �������� ��������� FTP �� SRAM                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_FTP_Settings(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //�����, ������������ � ������
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
	for (i = 0; i < 64; ++i)
		{
		Address=FTP_Settings_String_Address+i*4; Data4Byte=*(uint32_t*)Address; //����������� �����
		//����� �������������� �� ����������
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
//                                           �������, ����������� ��������� GPRS � SRAM                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_GPRS_Settings(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //�����, ������������ � ������
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
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
//                                           �������, �������� ��������� GPRS �� SRAM                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_GPRS_Settings(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //�����, ������������ � ������
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
	for (i = 0; i < 64; ++i)
		{
		Address=GPRS_Settings_String_Address+i*4; Data4Byte=*(uint32_t*)Address; //����������� �����
		//����� �������������� �� ����������
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
//                                           �������, ����������� GPRS_User_Pass � SRAM                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_GPRS_User_Pass(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //�����, ������������ � ������
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
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
//                                           �������, �������� GPRS_UserPass �� SRAM                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_GPRS_User_Pass(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //�����, ������������ � ������
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
	for (i = 0; i < 64; ++i)
		{
		Address=GPRS_UserPass_String_Address+i*4; Data4Byte=*(uint32_t*)Address; //����������� �����
		//����� �������������� �� ����������
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
//                                           �������, ����������� ��� ����� FTP � SRAM                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_FTP_FileName(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //�����, ������������ � ������
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
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
//                                           �������, �������� ��� ����� FTP �� SRAM                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_FTP_FileName(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //�����, ������������ � ������
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
	for (i = 0; i < 64; ++i)
		{
		Address=FTP_File_String_Address+i*4; Data4Byte=*(uint32_t*)Address; //����������� �����
		//����� �������������� �� ����������
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
//                                           �������, ����������� ��������� ���������� �� SMS � SRAM                      //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_SMS_Settings(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //�����, ������������ � ������
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
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
//                                           �������, ����������� ��������� ���������� �� SMS �� SRAM                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_SMS_Settings(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	unsigned char DataByte0;
	unsigned char DataByte1;
	unsigned char DataByte2;
	unsigned char DataByte3;
	uint32_t Data4Byte; //�����, ������������ � ������
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
	for (i = 0; i < 3; ++i)
		{
		Address=SMS_Number_String_Address+i*4; Data4Byte=*(uint32_t*)Address; //����������� �����
		//����� �������������� �� ����������
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
//                                           �������, ����������� ChIDList SRAM                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_ChIDList(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
	for (i = 0; i < 64; ++i)
		{
		Address=ChID_List_Address+i*4;*(uint32_t*)Address=ChIDList[i];
		}
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           �������, �������� ChIDList �� SRAM                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_ChIDList(void)
	{
	unsigned char i; //�������
	uint32_t Address;
	PWR_BackupAccessCmd(ENABLE); //����������� ������ � ��������
	PWR_BackupRegulatorCmd(ENABLE); //���������� �������������������
	while (!(PWR->CSR & (PWR_FLAG_BRR))); //�������� ���������� � ���������� ���������� ����������
	for (i = 0; i < 64; ++i)
		{
		Address=ChID_List_Address+i*4;ChIDList[i]=*(uint32_t*)Address;
		}
	PWR_BackupAccessCmd(DISABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                               �������, ����������� ���������� �������� �� flash ������                                 //
//   ���� ������ {YY,MM,DD,Hour,  min,sec,Temp1,Temp2,  Val1,Val2,Val3,Val4,  Var1,Var2,Var3,Var4, ChId1,ChId2,ChId3,ChId4}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Save_Record(TMemoryResult Write_MeasResult)
	{
	unsigned int i; //�������
	uint32_t Data4Byte; //�����, ������������ � ������
	uint32_t RecordAddress; //����� ������
	uint32_t Address;
	uint32_t SectorAddress; //����� ��������� �������
	signed int TempValue=0; //��������� ��������
	unsigned char Bytes0; //����0
	unsigned char Bytes1; //����1
	unsigned char Bytes2; //����2
	unsigned char Bytes3; //����3
	unsigned char ReadByte; //�������� ����
	FLASH_Unlock(); //����������� �������������� ������
	//////////////////////////////////////////////////////////////////////
			//����� ������ ������
	RecordAddress=0x00;
	//����� � ������� �8
	SectorAddress=Meas1Address;
	i=0x00;
	while ((RecordAddress==0)&(i<0x1000))
		{
		Address=SectorAddress+i*0x20+0x18; ReadByte=*(unsigned char*)Address; //�������� ���� ������� ������
		if (ReadByte==0xff) {RecordAddress=SectorAddress+i*0x20;}
		i++;
		}
	if ((i==0x1000)&(RecordAddress!=0)) //���� ���������� �������� ��������� ������
		{
		FLASH_EraseSector(SectorMeas2,VoltageRange_3); //������� ���������� ������� Flash ������
		}
	//����� � ������� �9
	if (RecordAddress==0) //���� � ���������� ������� �� ������� ������ �����
		{
		SectorAddress=Meas2Address;
		i=0x00;
		while ((RecordAddress==0)&(i<0x1000))
			{
			Address=SectorAddress+i*0x20+0x18; ReadByte=*(unsigned char*)Address; //�������� ���� ������� ������
			if (ReadByte==0xff) {RecordAddress=SectorAddress+i*0x20;}
			i++;
			}
		if ((i==0x1000)&(RecordAddress!=0)) //���� ���������� �������� ��������� ������
			{
			FLASH_EraseSector(SectorMeas3,VoltageRange_3); //������� ���������� ������� Flash ������
			}
		}
	//����� � ������� �10
	if (RecordAddress==0) //���� � ���������� ������� �� ������� ������ �����
		{
		SectorAddress=Meas3Address;
		i=0x00;
		while ((RecordAddress==0)&(i<0x1000))
			{
			Address=SectorAddress+i*0x20+0x18; ReadByte=*(unsigned char*)Address; //�������� ���� ������� ������
			if (ReadByte==0xff) {RecordAddress=SectorAddress+i*0x20;}
			i++;
			}
		if ((i==0x1000)&(RecordAddress!=0)) //���� ���������� �������� ��������� ������
			{
			FLASH_EraseSector(SectorMeas1,VoltageRange_3); //������� ���������� ������� Flash ������
			}
		}
	//���� �� ������� ������ �����
	if (RecordAddress==0)
		{
		if ((i==0x1000)&(RecordAddress!=0)) //���� ���������� �������� ��������� ������
			{
			FLASH_EraseSector(SectorMeas1,VoltageRange_3); //������� ���������� ������� Flash ������
			}
		RecordAddress=Meas1Address;
		}
	//////////////////////////////////////////////////////////////////////
					//������ ������
	//���� ������ {YY,MM,DD,  Hour,min,sec,Temp1,Temp2,  Val1,Val2,Val3,Val4,  Var1,Var2,Var3,Var4, ChId1,ChId2,ChId3,ChId4}
	//���������� ������� �����
	Bytes0=Write_MeasResult.YY;
	Bytes1=Write_MeasResult.MM;
	Bytes2=Write_MeasResult.DD;
	Bytes3=0;
	Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
	Address=RecordAddress+0x00; FLASH_ProgramWord(Address,Data4Byte);
	//���������� ������� �����
	Bytes0=Write_MeasResult.hour;
	Bytes1=Write_MeasResult.min;
	Bytes2=Write_MeasResult.sec;
	Bytes3=0;
	Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
	Address=RecordAddress+0x04; FLASH_ProgramWord(Address,Data4Byte);
	//���������� �������� ����� CHID
	Data4Byte=(uint32_t) Write_MeasResult.ChID;
	Address=RecordAddress+0x08; FLASH_ProgramWord(Address,Data4Byte);
	//���������� ���������� ����� Value
	TempValue=(float) Write_MeasResult.Value*100000;
	TempValue=(signed int) TempValue+1000000000;
	Data4Byte=(uint32_t) TempValue;
	Address=RecordAddress+0x0C; FLASH_ProgramWord(Address,Data4Byte);
	//���������� ������ ����� Variation
	TempValue=(float) Write_MeasResult.Variation*100000;
	TempValue=(signed int) TempValue+1000000000;
	Data4Byte=(uint32_t) TempValue;
	Address=RecordAddress+0x10; FLASH_ProgramWord(Address,Data4Byte);
	//���������� ������� ����� �����������
	TempValue=(float) Write_MeasResult.Temperature*100000;
	TempValue=(signed int) TempValue+1000000000;
	Data4Byte=(uint32_t) TempValue;
	Address=RecordAddress+0x14; FLASH_ProgramWord(Address,Data4Byte);
	//���������� �������� ����� ���� ������� ������
	Bytes0=0x00;
	Bytes1=0x00;
	Bytes2=0x00;
	Bytes3=0x00;
	Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
	Address=RecordAddress+0x18; FLASH_ProgramWord(Address,Data4Byte);
	//���������� �������� ����� ���� �������� �� FTP
	Bytes0=0xff;
	Bytes1=0xff;
	Bytes2=0xff;
	Bytes3=0xff;
	Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
	Address=RecordAddress+0x1C; FLASH_ProgramWord(Address,Data4Byte);
	//////////////////////////////////////////////////////////////////////
	FLASH_Lock(); //���������� ������ ������ �� ��������������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           �������, �������� ������ �� flash ������                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TMemoryResult Read_Record(unsigned int IndexRecord)
	{
	TMemoryResult TempResult; //��������� ���������� ��� �������� �����������
	TempResult.FlagFTP=0xff;
	TempResult.FlagMeas=0xff;
	uint32_t RecordAddress; //����� ������
	uint32_t Address;
	uint32_t Data4Byte; //�����, �������� �� ������
	signed int TempValue=0; //��������� ��������
	RecordAddress=Meas1Address+0x20*IndexRecord;
	//���� ������ {YY,MM,DD,  Hour,min,sec,Temp1,Temp2,  Val1,Val2,Val3,Val4,  Var1,Var2,Var3,Var4, ChId1,ChId2,ChId3,ChId4}
	//������ ������� �����
	Address=RecordAddress+0x00; Data4Byte=*(uint32_t*)Address;
	TempResult.YY=(Data4Byte/0x1000000)%0x100;
	TempResult.MM=(Data4Byte/0x10000)%0x100;
	TempResult.DD=(Data4Byte/0x100)%0x100;
	//������ ������� �����
	Address=RecordAddress+0x04; Data4Byte=*(uint32_t*)Address;
	TempResult.hour=(Data4Byte/0x1000000)%0x100;
	TempResult.min=(Data4Byte/0x10000)%0x100;
	TempResult.sec=(Data4Byte/0x100)%0x100;
	//������ �������� ����� CHID
	Address=RecordAddress+0x08; Data4Byte=*(uint32_t*)Address;
	TempResult.ChID=(uint32_t) Data4Byte;
	//������ ���������� ����� Value
	Address=RecordAddress+0x0C; Data4Byte=*(uint32_t*)Address;
	TempValue=(signed int) Data4Byte;
	TempValue=(signed int) TempValue-1000000000;
	TempResult.Value=(float) TempValue/100000;
	//������ ������ ����� Variation
	Address=RecordAddress+0x10; Data4Byte=*(uint32_t*)Address;
	TempValue=(signed int) Data4Byte;
	TempValue=(signed int) TempValue-1000000000;
	TempResult.Variation=(float) TempValue/100000;
	//������ ������� ����� �����������
	Address=RecordAddress+0x14; Data4Byte=*(uint32_t*)Address;
	TempValue=(signed int) Data4Byte;
	TempValue=(signed int) TempValue-1000000000;
	TempResult.Temperature=(float) TempValue/100000;
	//������ �������� ����� ���� ������� ������
	Address=RecordAddress+0x18; Data4Byte=*(uint32_t*)Address;
	TempResult.FlagMeas=Data4Byte%0x100;
	//������ �������� ����� ���� �������� �� FTP
	Address=RecordAddress+0x1c; Data4Byte=*(uint32_t*)Address;
	TempResult.FlagFTP=Data4Byte%0x100;
	return TempResult;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          �������, ��������� ��� ����� FTP                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ClearFTPFlags(void)
	{
	unsigned int i; //�������
	uint32_t Data4Byte; //�����, ������������ � ������
	uint32_t RecordAddress; //����� ������
	uint32_t Address;
	uint32_t SectorAddress; //����� ��������� �������
	unsigned char Bytes0; //����0
	unsigned char Bytes1; //����1
	unsigned char Bytes2; //����2
	unsigned char Bytes3; //����3
	unsigned char ReadByte; //�������� ����
	FLASH_Unlock(); //����������� �������������� ������
	//////////////////////////////////////////////////////////////////////
	//����� ������ ������
	RecordAddress=0x00;
	//����� � ������� �8
	SectorAddress=Meas1Address;
	for (i = 0x00; i < 0x1000; ++i)
		{
		RecordAddress=SectorAddress+i*0x20;
		Address=RecordAddress+0x18; ReadByte=*(unsigned char*)Address; //�������� ���� ������� ������
		if (ReadByte!=0xff)
			{
			//���������� �������� ����� ���� �������� �� FTP
			Bytes0=0x00;
			Bytes1=0x00;
			Bytes2=0x00;
			Bytes3=0x00;
			Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
			Address=RecordAddress+0x1C; FLASH_ProgramWord(Address,Data4Byte);
			}
		}
	//����� � ������� �9
	SectorAddress=Meas2Address;
	for (i = 0x00; i < 0x1000; ++i)
		{
		RecordAddress=SectorAddress+i*0x20;
		Address=RecordAddress+0x18; ReadByte=*(unsigned char*)Address; //�������� ���� ������� ������
		if (ReadByte!=0xff)
			{
			//���������� �������� ����� ���� �������� �� FTP
			Bytes0=0x00;
			Bytes1=0x00;
			Bytes2=0x00;
			Bytes3=0x00;
			Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
			Address=RecordAddress+0x1C; FLASH_ProgramWord(Address,Data4Byte);
			}
		}
	//����� � ������� �10
	SectorAddress=Meas3Address;
	for (i = 0x00; i < 0x1000; ++i)
		{
		RecordAddress=SectorAddress+i*0x20;
		Address=RecordAddress+0x18; ReadByte=*(unsigned char*)Address; //�������� ���� ������� ������
		if (ReadByte!=0xff)
			{
			//���������� �������� ����� ���� �������� �� FTP
			Bytes0=0x00;
			Bytes1=0x00;
			Bytes2=0x00;
			Bytes3=0x00;
			Data4Byte=Bytes3+Bytes2*0x100+Bytes1*0x10000+Bytes0*0x1000000;
			Address=RecordAddress+0x1C; FLASH_ProgramWord(Address,Data4Byte);
			}
		}
	//////////////////////////////////////////////////////////////////////
	FLASH_Lock(); //���������� ������ ������ �� ��������������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////  �������, �������������� ������ ���������� ���������� �� flash � SRAM ������  ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Read_Parametr_Device(void)
	{
	uint32_t Address;
	Address=ParametrAddress; DeviceSerial=*(uint32_t*)Address; //����������� �������� ����� ����������
	Address=ParametrAddress+4; MeasSystem.TermistorConstRes=*(uint32_t*)Address; //����������� ������������� ����������� ��������� � ���� ����������
	Address=ParametrAddress+8; MeasSystem.TermistorRes=*(uint32_t*)Address; //����������� ������������� ����������
	Address=ParametrAddress+12; MeasSystem.PowerConstRes=*(uint32_t*)Address; //����������� ������������� ��������� � ���� ��������������
	Read_FTP_Settings(); //�������� ��������� FTP ���������� �� SRAM
	Read_GPRS_Settings(); //�������� ��������� GPRS ���������� �� SRAM
	Read_GPRS_User_Pass(); //�������� ��� ������������ � ������ GPRS ���������� �� SRAM
	Read_FTP_FileName(); //�������� ��� ����� �� SRAM
	Read_ChIDList(); //�������� ������ ������������� ������� �� SRAM
	Read_SMS_Settings(); //�������� ��������� ������� �� ��� �� SRAM
	RTC_Get_Date_Time();
	RTC_Get_Alarm_Time();
	Get_DeviceMode(); //������������ ����� ������ ����������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////   �������, �������������� ����������� ���������� ���������� �� ������ � ���������� ���������   //////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Copy_Parametr_Buf(void)
	{
	unsigned int i; //�������
	uint32_t WriteAddress; //����� ������� � ������� ������������ ������
	uint32_t ReadAddress; //����� ������� �� ������� ���������� ������
	//����������� ������ �� ����� ������� ������ � ������
	FLASH_Unlock(); //����������� �������������� ������
	//������� ������
	FLASH_EraseSector(SectorParametr,VoltageRange_3); //������� ������� Flash ������ � ������� ���������� ������
	//����������� ������
	WriteAddress=ParametrAddress; //����� ������� � ������� ������������ ������
	ReadAddress=ParametrAddressBuf; //����� �� ������� ������������ ������ ������
	for (i = 0; i < 0x8000; ++i) //����������� ������
		{
		FLASH_ProgramWord(WriteAddress,*(uint32_t*)ReadAddress);
		ReadAddress+=4;
		WriteAddress+=4;
		}
	//��������� ����� ������������ ������ ������� ����������
	FLASH_ProgramWord(ParametrFlagWriteAddress,0x00000000);
	FLASH_Lock(); //���������� ������ ������ �� ��������������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  �������, �������������� ������� � ���������� ������� �������� �������� ���������� �� ����� ������� ������ � ������  //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Jump_To_Application(uint32_t Address)
	{
	__disable_irq(); //����������� ���������� ����������
	typedef  void (*pFunction)(void);
	pFunction Jump_To_Application; //���������� ��������� �� �������
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
///////////////////////////         ������� ���������� �������� � ���������          ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////     ������� ������������� ��������� �������     ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Main_Timer_Init(void)
	{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM_Main, ENABLE); //������������ ��������� ��������
	//������������� �������
    TIM_TimeBaseInitTypeDef Timer_InitStructure;
    TIM_TimeBaseStructInit(&Timer_InitStructure);
    Timer_InitStructure.TIM_Prescaler = 8-1; //������������ ������� (������� ����� ������� 1 ���)
    Timer_InitStructure.TIM_Period = 50; //������ ����� ����������� ������ 50 ��� (20 ���)
    Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ����������� ������������ �������
    Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; //����������� ������ - �� ����������
    Timer_InitStructure.TIM_RepetitionCounter = 0; //������� ����� ��������� ������ ������������ ��������
    TIM_TimeBaseInit(Timer_Main, &Timer_InitStructure); //������������� �������
    //������������ ����������
    TIM_ITConfig(Timer_Main, TIM_IT_Update, ENABLE); //�������� ���������� �� ������������ ��������
	NVIC_SetPriority(Timer_Main_IRQn,Prioritet_MainTimer); //��������������� ��������� ����������
	NVIC_EnableIRQ(Timer_Main_IRQn); //����������� ��������� ���������� �� �������
	//������ �������
	TIM_Cmd(Timer_Main, ENABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////     ������� ������������ ��������� �������       ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TIM2_IRQHandler(void)
	{
	TIM_ClearITPendingBit(Timer_Main, TIM_IT_Update); //����� ����� �������� ����������
	if (Timeouts.DelayMain != 0) {Timeouts.DelayMain--;} //������������� �������� ��� ������� �������� � �������� ���������
	if (Timeouts.DelayBluetooth != 0) {Timeouts.DelayBluetooth--;} //������������� �������� ��� ������� �������� � ���������� bluetooth
	if (Timeouts.TimeoutGSM_Char > 0) {Timeouts.TimeoutGSM_Char-=50;} //������������� �������� �������� �������� ������� � GSM ������
	if (Timeouts.TimeoutGSM_AT > 0) {Timeouts.TimeoutGSM_AT-=50;} //������������� �������� �������� ���������� AT ������� GSM �������
	if (Timeouts.TimeoutADC > 0) {Timeouts.TimeoutADC-=50;} //������������� �������� �������� ���������� AT ������� GSM �������
	if (Timeouts.TimeoutSendMessageBluetooth > 0) {Timeouts.TimeoutSendMessageBluetooth-=50;} //������������� �������� �������� �������� ��������� �� bluetooth
	if (Timeouts.TimeoutIdle> 0) {Timeouts.TimeoutIdle-=50;} //������������� �������� �������� ������� ����������
	if (Timeouts.TimeoutRS485_Char > 0) {Timeouts.TimeoutRS485_Char-=50;} //������������� �������� �������� �������� ������� � RS485 ������
	if (Timeouts.TimeoutRS485_Instruction > 0) {Timeouts.TimeoutRS485_Instruction-=50;} //������������� �������� �������� �������� ���������� ����������

	Timeouts.TimeFTPSession+=50; //������������� ����� FTP ������, ���

	if (Timeouts.ForceRestart > 50)
		{Timeouts.ForceRestart-=50;}//������������� �������� �������� ��������������� ����������
	else
		{

		if (FTPInProgress!=0x00) //���� ������ ������� ��������
			{
			ClearFTPFlags(); //������� ������ �������� ������ �� FTP
			}
		GPIO_ResetBits(Power_Port, Power_Pin); Delay_us(3000000); //����������� ������� ����������
		}

	if (Timeouts.AlarmSend > 50) {Timeouts.AlarmSend-=50;} //������������� �������� �������� ������� �������� ��������� �� �������� �����, ���

	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////               ������� ��������, ���         /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Delay_us(unsigned int Val)
	{
	Timeouts.DelayMain = Val/50;
	while (Timeouts.DelayMain != 0){};
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////           ������� �������� � ���������� bluetooth, ���        ///////////////////////////////////////////
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
//////////////////////////////                 ������� ��� ������ � RTC              ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////        ������� ������������� ����� ��������� �������          ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RTC__Init(void)
	{
	PWR_BackupAccessCmd(ENABLE);
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);
	RTC_InitTypeDef RTC_InitStructure;
	RCC_RTCCLKCmd(DISABLE);
	RTC_WaitForSynchro(); //��������� �������������
	//���������� �������� ������������ ����� 32768
	RCC_LSEConfig(RCC_LSE_ON);
	while ((RCC->BDCR & RCC_BDCR_LSERDY) != RCC_BDCR_LSERDY) {}
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	RTC_StructInit(&RTC_InitStructure);
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	RTC_InitStructure.RTC_SynchPrediv = 0x7FFF;
	RTC_Init(&RTC_InitStructure);
    //���������� RTC
    RCC_RTCCLKCmd(ENABLE);//���������� RTC
    RTC_WaitForSynchro();//��������� �������������
    //������������ ����� ������� ���������� �� PC13
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
/////////////////////////////        ������� ��������� ������� ������� �������           ///////////////////////////////////
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
/////////////////////////////        ������� ��������� ������� ���� � �������            ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RTC_Set_Date_Time(void)
	{
	PWR_BackupAccessCmd(ENABLE);
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	RTC_TimeStructInit(&RTC_TimeStructure);
	RTC_DateStructInit(&RTC_DateStructure);
	//��������� �������
	RTC_TimeStructure.RTC_H12 = RTC_HourFormat_24;
	RTC_TimeStructure.RTC_Hours=DateTime.hour;
    RTC_TimeStructure.RTC_Minutes=DateTime.min;
    RTC_TimeStructure.RTC_Seconds=DateTime.sec;
	RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure);
	//��������� ����
    RTC_DateStructure.RTC_Year=DateTime.YY;
    RTC_DateStructure.RTC_Month=DateTime.MM;
    RTC_DateStructure.RTC_Date=DateTime.DD;
    RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////           ������� ��������� ������� ����������             ///////////////////////////////////
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
/////////////////////////////          ������� ��������� ������� ����������              ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RTC_Set_Alarm_Time(void)
	{
	PWR_BackupAccessCmd(ENABLE);
	RTC_AlarmCmd(RTC_Alarm_A, DISABLE); //����������� ���������
	RTC_AlarmTypeDef Alarm_InitStructure;
   	RTC_AlarmStructInit(&Alarm_InitStructure);
   	Alarm_InitStructure.RTC_AlarmTime.RTC_H12=RTC_HourFormat_24;
   	Alarm_InitStructure.RTC_AlarmTime.RTC_Hours=CycleTime.hour;
   	Alarm_InitStructure.RTC_AlarmTime.RTC_Minutes=CycleTime.min;
   	Alarm_InitStructure.RTC_AlarmTime.RTC_Seconds=CycleTime.sec;
   	Alarm_InitStructure.RTC_AlarmDateWeekDay=1;
   	Alarm_InitStructure.RTC_AlarmDateWeekDaySel=RTC_AlarmDateWeekDaySel_WeekDay; //������������ ���������� ����� ����������� � ������������ ���� ������
   	Alarm_InitStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay; //��������� ����� ����������� ������ ����
   	RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &Alarm_InitStructure);
   	RTC_AlarmCmd(RTC_Alarm_A, ENABLE); //���������� ���������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////        ������� ��������� ������� ������������ ����������      ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RTC_Set_Next_Cycle_Time(void)
	{
	unsigned int CurrentTimeSecond; //������� ����� (���������� ������ �� ������ ���)
	unsigned int CycleTimeSecond; //����� �������������� ����� (���������� ������ �� ������ ���)
	RTC_Get_Date_Time(); //���������� ������� ������� �������
	RTC_Get_Alarm_Time(); //���������� ������������� ����� ����������
	CurrentTimeSecond=DateTime.hour*3600+DateTime.min*60+DateTime.sec;
	CycleTimeSecond=CycleTime.hour*3600+CycleTime.min*60+CycleTime.sec;
	//����� ������������ ���������� ��������������� �� ��� ��� ���� �� �������� ������� ������� ������� + 20 ������
	if ((CurrentTimeSecond+CyclePeriod)<CycleTimeSecond) {CurrentTimeSecond+=86400;} //���� ������� ������� ������� + ������ ��������� ������ ������� ���������� ������� (������� ������� ������� ������� �� ��������� �����)
	while ((CurrentTimeSecond+20)>CycleTimeSecond) {CycleTimeSecond+=CyclePeriod;}
	CycleTimeSecond=CycleTimeSecond%86400;  //������������ ����� ������������ ���������� ������������ ������ �����
	CycleTime.hour=(CycleTimeSecond/3600)%24;
	CycleTime.min=(CycleTimeSecond/60)%60;
	CycleTime.sec=CycleTimeSecond%60;
	RTC_Set_Alarm_Time(); //��������������� ����� ���������� �������������� �����
	RTC_ClearFlag(RTC_FLAG_ALRAF); //������������ ���� ������������ ����������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////         ������� ��������� ������ ������ ����������         ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Get_DeviceMode(void)
	{
	GetCycle_Settings(); //�������� ��������� ������������� ���������
	if (RTC_GetFlagStatus(RTC_FLAG_ALRAF)==RESET) //���� ����������� ��������� �� �������������� ����������
		{DeviceMode=DeviceModeCommand;}
	else
		{
		//������������ ������������� �������� ������ �� FTP ������
		if (CurrentCycleSendDataPeriod==0)
			{
			DeviceMode=DeviceModeCycleFTP;//��������� � ��������� ������
			CurrentCycleSendDataPeriod=CycleSendDataPeriod;
			}
		else
			{
			DeviceMode=DeviceModeCycle;//��������� ��� �������� ������
			CurrentCycleSendDataPeriod--;
			}
		}
	SetCycle_Settings(); //���������� ��������� ������������� ���������
	RTC_ClearFlag(RTC_FLAG_ALRAF); //������������ ���� ������������ ����������
	}




















////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////               ������� ��� ������ � Bluetooth          ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////         ������� ������������� bluetooth ������             ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Init(void)
	{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//������������ USART Bluetooth
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART_BT, ENABLE);
	//����� RESET Bluetooth ������
	GPIO_InitStructure.GPIO_Pin = BT_Reset_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BT_Reset_Port, &GPIO_InitStructure);
	//����� PIO_11 Bluetooth ������
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
	//������������ USART Bluetooth
	USART_InitStructure.USART_BaudRate = 9600;                                       // ��������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // ����� ������ 1����/8���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // 1 ����-���
	USART_InitStructure.USART_Parity = USART_Parity_No;                              // ��� �������� ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // ��������� ����� � ��������
	USART_Init(USART_BT, &USART_InitStructure);
	USART_Cmd(USART_BT, ENABLE);
	GPIO_SetBits(BT_AT_Port, BT_AT_Pin); Delay_us(10000);
	GPIO_ResetBits(BT_Reset_Port, BT_Reset_Pin); Delay_us(10000);
	GPIO_SetBits(BT_Reset_Port, BT_Reset_Pin);
	//������� ������ � ����� �������� ������
	GPIO_ResetBits(BT_AT_Port, BT_AT_Pin); Delay_us(10000);
	GPIO_ResetBits(BT_Reset_Port, BT_Reset_Pin); Delay_us(10000);
	GPIO_SetBits(BT_Reset_Port, BT_Reset_Pin);
	//������ Bluetooth
	TIM_TimeBaseInitTypeDef Timer_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM_Bluetooth,ENABLE); //������������ ������� �����������������
    TIM_TimeBaseStructInit(&Timer_InitStructure); //����������� ���� ��������� ���������� �����������
    Timer_InitStructure.TIM_Prescaler = 8; //������������ ������������
    Timer_InitStructure.TIM_Period = 50; //������������ ������ ������������ ������� 50���
    TIM_TimeBaseInit(Timer_Bluetooth, &Timer_InitStructure); //������������� �������
    TIM_ITConfig(Timer_Bluetooth, TIM_IT_Update, ENABLE); //����������� ������ ��� ��������� ���������� �� ���������� (������������)
    TIM_Cmd(Timer_Bluetooth, ENABLE); //����������� ������
    NVIC_SetPriority(Timer_Bluetooth_IRQn,PrioritetBluetooth); //��������������� ��������� ����������
    NVIC_EnableIRQ(Timer_Bluetooth_IRQn); //��������� ��������������� ����������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////         ������� ������� bluetooth ������ AT �������             //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_AT(const unsigned char* str)
	{
	unsigned int i;
	for(i = 0; str[i]; i++)
		{
		while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); //��������� ��������� �������� �����
		USART_SendData(USART_BT,str[i]); //�������� �����
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////         ������� ������� ��������� �� bluetooth               /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_Message(void)
	{
	Timeouts.TimeoutSendMessageBluetooth=Send_MessageBT_Timeout; //����������� ������� �������� ������� ��������� �� bluetooth
	//����������� ������� ��������� ���������
	MessageSelectionBluetooth.Buffer[MessageSelectionBluetooth.Buffer_Len]=0x0d;
	MessageSelectionBluetooth.Buffer[MessageSelectionBluetooth.Buffer_Len+1]=0x0a;
	MessageSelectionBluetooth.Buffer_Len=MessageSelectionBluetooth.Buffer_Len+2;
	MessageSelectionBluetooth.Buffer_Pos=0; //������������ ������� ������� ������� Bluetooth
	while ((USART_GetFlagStatus(USART_BT, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutSendMessageBluetooth>50)) {}; //��������� ��������� �������� �����
	USART_SendData(USART_BT, 0x0A); //���������� ������ ����� ������
	while (1)
		{
		if (Timeouts.TimeoutSendMessageBluetooth<50) {return;}//���� ������� �����, ���������� �� ������� ��������� �� Bluetooth
		if (USART_GetFlagStatus(USART_BT, USART_FLAG_TC) != RESET) //���� ���� ���������� ����
			{
			if (MessageSelectionBluetooth.Buffer_Pos>=MessageSelectionBluetooth.Buffer_Len) {return;}//���� ��������� ����� ��������� � ������
			else
				{
				USART_SendData(USART_BT, MessageSelectionBluetooth.Buffer[MessageSelectionBluetooth.Buffer_Pos]); //��������� ���� �� Bluetooth
				MessageSelectionBluetooth.Buffer_Pos++; //���������������� �������
				}
			}
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////       ������� ���������� ������������� ��������� �������� �� Bluetooth                           /////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Interpretation_Message(void)
	{
	unsigned int i; //�������
	unsigned int CurrentDelimiterNumber=0; //������� ����� �����������
	MessageInterpretationBluetooth.CorrectMessageFlag=1; //��������������� ���� ������������ ���������
	MessageInterpretationBluetooth.TypePosStart=0; //������ ����� "��� ���������"
	MessageInterpretationBluetooth.TypePosEnd=0; //����� ����� "��� ���������"
	MessageInterpretationBluetooth.AddressPosStart=0; //������ ����� "�����"
	MessageInterpretationBluetooth.AddressPosEnd=0; //����� ����� "�����"
	MessageInterpretationBluetooth.TransactionPosStart=0; //������ ����� "����������"
	MessageInterpretationBluetooth.TransactionPosEnd=0; //����� ����� "����������"
	MessageInterpretationBluetooth.InstructionPosStart=0; //������ ����� "����������"
	MessageInterpretationBluetooth.InstructionPosEnd=0; //����� ����� "����������"
	MessageInterpretationBluetooth.DataPosStart=0; //������ ����� "������"
	MessageInterpretationBluetooth.DataPosEnd=0; //����� ����� "������"
	//��������� �����������
	for (i = 0; i < MessageSelectionBluetooth.Buffer_Len; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]=='/') //���� ��������� �����������
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //���� ��������� 0-� �����������
				MessageInterpretationBluetooth.TypePosStart=i+1; //������������ ������ ����� "��� ���������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 1: //���� ��������� 1-� �����������
				MessageInterpretationBluetooth.TypePosEnd=i-1; //������������ ����� ����� "��� ���������"
				MessageInterpretationBluetooth.AddressPosStart=i+1; //������������ ������ ����� "�����"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 2: //���� ��������� 2-� �����������
				MessageInterpretationBluetooth.AddressPosEnd=i-1; //������������ ����� ����� "�����"
				MessageInterpretationBluetooth.TransactionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 3: //���� ��������� 3-� �����������
				MessageInterpretationBluetooth.TransactionPosEnd=i-1; //������������ ����� ����� "����������"
				MessageInterpretationBluetooth.InstructionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 4: //���� ��������� 4-� �����������
				MessageInterpretationBluetooth.InstructionPosEnd=i-1; //������������ ����� ����� "����������"
				MessageInterpretationBluetooth.DataPosStart=i+1; //������������ ������ ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 5: //���� ���������5-� �����������
				MessageInterpretationBluetooth.DataPosEnd=i-1; //������������ ����� ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				}
			}
		}
	//����������� ���������� ������������
	if (CurrentDelimiterNumber!=6) {return;} //���� ���������� ������������ �� ����������
	//����������� ������������ ����� "��� ���������"
	if (MessageInterpretationBluetooth.TypePosEnd!=MessageInterpretationBluetooth.TypePosStart) {return;}
	if (MessageSelectionBluetooth.Buffer[MessageInterpretationBluetooth.TypePosStart]!='Q') {return;}//���� ��������� - �� �������� ��������
	MessageSelectionBluetooth.Buffer[MessageInterpretationBluetooth.TypePosStart]='R'; //���������� ��� ���������
	//����������� ������������ ����� "�����"
	if (MessageInterpretationBluetooth.AddressPosStart>MessageInterpretationBluetooth.AddressPosEnd) {return;}//���� ���� "������" ������, �� ����������� ���������� �������
	for (i = MessageInterpretationBluetooth.AddressPosStart; i <= MessageInterpretationBluetooth.AddressPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {return;}}//���� ������ - �� �������� ������
	MessageInterpretationBluetooth.Address=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.AddressPosStart, MessageInterpretationBluetooth.AddressPosEnd); //���������������� ����� � ���������
	if ((MessageInterpretationBluetooth.Address!=DeviceAddress)&(MessageInterpretationBluetooth.Address!=0)) {return;} //���� �� ��������� ����� � ��������� � ������� ���������� ��� ����������������� �������
	//����������� ������������ ����� "����������"
	if (MessageInterpretationBluetooth.TransactionPosStart>MessageInterpretationBluetooth.TransactionPosEnd) {return;}//���� ������ ����� "����������" ������ 1�� �������, �� ����������� ���������� �������
	//-------------------------------����������� ����������
	Bluetooth_Execution_GetType();//������� ���������� ���������� ���������� GetType
	Bluetooth_Execution_GetSerial();//������� ���������� ���������� ���������� GetSerial
	Bluetooth_Execution_GetProgVersion();//������� ���������� ���������� ���������� GetProgVersion
	Bluetooth_Execution_UploadProgramm();//������� ���������� ���������� ���������� UploadProgramm
	Bluetooth_Execution_UploadSettings(); //������� ���������� ���������� ���������� UploadSettings
	Bluetooth_Execution_DownloadSettings(); //������� ���������� ���������� ���������� DownloadSettings
	Bluetooth_Execution_SetServiceMode(); //������� ���������� ���������� ���������� SetServiceMode
	Bluetooth_Execution_Test();
	Bluetooth_Execution_SetCycleSettings(); //������� ���������� ���������� ���������� SetCycleSettings
	Bluetooth_Execution_GetCycleSettings(); //������� ���������� ���������� ���������� GetCycleSettings
	Bluetooth_Execution_SetClock(); //������� ���������� ���������� ���������� SetClock
	Bluetooth_Execution_GetClock(); //������� ���������� ���������� ���������� GetClock
	Bluetooth_Execution_SetFTPSettings(); //������� ���������� ���������� ���������� SetFTPSetting
	Bluetooth_Execution_GetFTPSettings(); //������� ���������� ���������� ���������� GetFTPSetting
	Bluetooth_Execution_SetGPRSSettings(); //������� ���������� ���������� ���������� SetGPRSSetting
	Bluetooth_Execution_GetGPRSSettings(); //������� ���������� ���������� ���������� GetGPRSSetting
	Bluetooth_Execution_SetGPRSUserPass(); //������� ���������� ���������� ���������� SetGPRSUserPass
	Bluetooth_Execution_GetGPRSUserPass(); //������� ���������� ���������� ���������� GetGPRSUserPass
	Bluetooth_Execution_SetFTPFileName(); //������� ���������� ���������� ���������� SetFTPFileName
	Bluetooth_Execution_GetFTPFileName(); //������� ���������� ���������� ���������� GetFTPFileName
	Bluetooth_Execution_SetChIDList(); //������� ���������� ���������� ���������� SetChIDList
	Bluetooth_Execution_GetChIDList(); //������� ���������� ���������� ���������� GetChIDList
	Bluetooth_Execution_SetSMSSettings(); //������� ���������� ���������� ���������� SetSMSSettings
	Bluetooth_Execution_GetSMSSettings(); //������� ���������� ���������� ���������� GetSMSSettings
	Bluetooth_Execution_DownloadData(); //������� ���������� ���������� ���������� DownloadData
	Timeouts.TimeoutIdle=Idle_Timeout; //��������������� ������� ������� ����������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////                  �������, ���������� �� ���������� ������� Bluetooth              /////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TIM4_IRQHandler(void)
	{
	if (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET) //���� � �������� ������ USART_BT ��� ����� ��� ������
		{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //��������� ��� ����������
		return; //����� �� �������
		}
	MessageSelectionBluetooth.Previous_Byte=MessageSelectionBluetooth.Current_Byte; //������������ ���������� �������� ����
	MessageSelectionBluetooth.Current_Byte=USART_ReceiveData(USART_BT); //���������� ���� �� USART
	if (MessageSelectionBluetooth.Start_Collect_Flag) //���� ���� ������� ������� ���������
		{
		MessageSelectionBluetooth.Buffer[MessageSelectionBluetooth.Buffer_Pos]=MessageSelectionBluetooth.Current_Byte; //��������� ������� ���� � ����� ���������
		MessageSelectionBluetooth.Buffer_Pos++; //���������������� ������� ��������� ������� � ������ ���������
		MessageSelectionBluetooth.Buffer_Len=MessageSelectionBluetooth.Buffer_Pos; //����������� ���������� ������ � ������ ���������
		if ((MessageSelectionBluetooth.Previous_Byte=='/')&(MessageSelectionBluetooth.Current_Byte=='%')) //���� ��������� ������ ����� ���������
			{
			MessageSelectionBluetooth.Start_Collect_Flag=0; //��������������� ������ ���������
			Bluetooth_Interpretation_Message(); //������������ ����������, ���������� � ���������
			TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //��������� ��� ����������
			return; //����� �� �������
			}
		}
	if (MessageSelectionBluetooth.Buffer_Pos>2047) //���� ��������� ����� ������
		{
		MessageSelectionBluetooth.Buffer_Pos=0; //������������ ��������� ������� � ������
		MessageSelectionBluetooth.Buffer_Len=0; //������������ ������� ���������� ���� � ������
		MessageSelectionBluetooth.Start_Collect_Flag=0; //������������ ���� ������ ���������
		MessageSelectionBluetooth.Current_Byte=0; //������������ �������� ����
		MessageSelectionBluetooth.Previous_Byte=0; //������������ ���������� ����
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //��������� ��� ����������
		return; //����� �� �������
		}
	if ((MessageSelectionBluetooth.Previous_Byte=='%')&(MessageSelectionBluetooth.Current_Byte=='/')) //���� ��������� ������ ������ ���������
		{
		MessageSelectionBluetooth.Start_Collect_Flag=1; //��������������� ���� ������� �������� ������� ���������
		MessageSelectionBluetooth.Buffer[0]=MessageSelectionBluetooth.Previous_Byte; //��������� ���������� ���� � ����� ���������
		MessageSelectionBluetooth.Buffer[1]=MessageSelectionBluetooth.Current_Byte; //��������� ������� ���� � ����� ���������
		MessageSelectionBluetooth.Buffer_Pos=2; //�������� ������� ������� � ������ ���������
		MessageSelectionBluetooth.Buffer_Len=2; //�������� ���������� ����������� ���� � ������
		}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //��������� ��� ����������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//         ������� ���������� �������������� ������ �������������� ����� � ������ � �������� �� � ����� Bluetooth       ////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Unsigned_Int_To_Char_Buf(unsigned int Val, unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //�������
	for (i = EndPos; i >= StartPos; --i)
		{
		MessageSelectionBluetooth.Buffer[i]=Val%10+48;
		Val=Val/10;
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////    ������� ���������� �������������� ����� ������, ����������� � ������ Bluetooth, � ����� ����� ��� �����       /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int Bluetooth_Char_To_Unsigned_Int_Buf(unsigned int StartPos, unsigned int EndPos) //StartPos - ������� ������� �������, EndPos - ������� ���������� �������
	{
	unsigned int i; //�������
	unsigned int Out=0; //�������� ����������
	unsigned int M=1; //��������� �������� �������
	for (i = EndPos; i >= StartPos; --i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]>47)&(MessageSelectionBluetooth.Buffer[i]<58)) //���� ������ - �����
			{
			Out=Out+M*(MessageSelectionBluetooth.Buffer[i]-48);
			M=M*10; //������������� ��������� �������� �������;
			}
		}
	return Out;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////    ������� ���������� �������������� ����� ������, ����������� � ������ Bluetooth, � ����� ����� �� ������       /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
signed int Bluetooth_Char_To_Signed_Int_Buf(unsigned int StartPos, unsigned int EndPos) //StartPos - ������� ������� �������, EndPos - ������� ���������� �������
	{
	unsigned int i; //�������
	signed int Out=0; //�������� ����������
	unsigned int M=1; //��������� �������� �������
	signed int Znak=1; //���������, ������������ ���� �����
	for (i = EndPos; i >= StartPos; --i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]>47)&(MessageSelectionBluetooth.Buffer[i]<58)) //���� ������ - �����
			{
			Out=Out+M*(MessageSelectionBluetooth.Buffer[i]-48);
			M=M*10; //������������� ��������� �������� �������;
			}
		if (MessageSelectionBluetooth.Buffer[i]==45) //���� ������ - '-'
			{
			Znak=-1;
			}
		}
	Out=Out*Znak; //�������� ���� �����
	return Out;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ Bluetooth, � ���������� ������ GSM.GPRS_Settings /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_HexChar_Buf_To_GPRS_Settings(unsigned int StartPos, unsigned int EndPos)
{
	unsigned int i; //�������
	unsigned char j=0; //�������
	unsigned char TempByte=0; //��������� ����
	unsigned char Byte1=0; //������ ��������(�������)
	unsigned char Byte2=0; //������ ��������(�������)
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
	GSM.GPRS_Settings[j]='\0'; //����������� ����������� ������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ Bluetooth, � ���������� ������ GSM.FTP_Settings /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_HexChar_Buf_To_FTP_Settings(unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //�������
	unsigned char j=0; //�������
	unsigned char TempByte=0; //��������� ����
	unsigned char Byte1=0; //������ ��������(�������)
	unsigned char Byte2=0; //������ ��������(�������)
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
	GSM.FTP_Settings[j]='\0'; //����������� ����������� ������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ Bluetooth, � ���������� ������ GSM.GPRS_User_Pass ////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_HexChar_Buf_To_GPRS_User_Pass(unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //�������
	unsigned char j=0; //�������
	unsigned char TempByte=0; //��������� ����
	unsigned char Byte1=0; //������ ��������(�������)
	unsigned char Byte2=0; //������ ��������(�������)
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
	GSM.GPRS_User_Pass[j]='\0'; //����������� ����������� ������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ Bluetooth, � ���������� ������ GSM.GPRS_User_Pass ////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_HexChar_Buf_To_FTP_FileName(unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //�������
	unsigned char j=0; //�������
	unsigned char TempByte=0; //��������� ����
	unsigned char Byte1=0; //������ ��������(�������)
	unsigned char Byte2=0; //������ ��������(�������)
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
	GSM.FTP_File_Name[j]='\0'; //����������� ����������� ������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////                   ������� �������� ����� ErrorData              ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_ErrorData(void) //������� �������� ��������� �� ������ ������
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //���� ��������� �����������������, �� ����� �� ����������
	unsigned int Pos=0; //������� ������� � ������ ���������
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
////////////////////////////                   ������� �������� ����� ErrorCh                ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_ErrorCh(void) //������� �������� ��������� �� ������ ������
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //���� ��������� �����������������, �� ����� �� ����������
	unsigned int Pos=0; //������� ������� � ������ ���������
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
////////////////////////////                ������� �������� ����� ErrorSensor               ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_ErrorSensor(void) //������� �������� ��������� �� ������ �������
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //���� ��������� �����������������, �� ����� �� ����������
	unsigned int Pos=0; //������� ������� � ������ ���������
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
////////////////////////////                  ������� �������� ����� End                     ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_End(void) //������� �������� ��������� � ���������� �������� ����� ������
	{
	unsigned int Pos=0; //������� ������� � ������ ���������
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
////////////////////////////                  ������� �������� ����� CRCError                ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_CRCError(void) //������� �������� ��������� �� ������ ����������� �����
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //���� ��������� �����������������, �� ����� �� ����������
	unsigned int Pos=0; //������� ������� � ������ ���������
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
////////////////////////////                   ������� �������� ����� CRC_Ok                 ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Send_CRC_Ok(void) //������� �������� ��������� �� ������ ����������� �����
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //���� ��������� �����������������, �� ����� �� ����������
	unsigned int Pos=0; //������� ������� � ������ ���������
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
////////////////////////////////     ������� ���������� ���������� ���������� GetType      /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetType(void)
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //���� ����� ��������� �����������������, �� ����� �� ����������
	unsigned char StrInstruction[7]={'G','e','t','T','y','p','e'};
	unsigned char InstructionLength=7; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//� ��������� ����������� ��������� ���������� ����������
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Bluetooth_Unsigned_Int_To_Char_Buf(DeviceType, Pos, Pos+2); Pos+=3;//� ����� ����������� ��� ����������
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////     ������� ���������� ���������� ���������� GetSerial      ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetSerial(void)
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //���� ����� ��������� �����������������, �� ����� �� ����������
	unsigned char StrInstruction[9]={'G','e','t','S','e','r','i','a','l'};//�������� �������� ����� ����������
	unsigned char InstructionLength=9; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Bluetooth_Unsigned_Int_To_Char_Buf(DeviceSerial, Pos, Pos+7); Pos+=8;//� ����� ����������� �������� �����
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////     ������� ���������� ���������� ���������� GetProgVersion      /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetProgVersion(void)
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //���� ����� ��������� �����������������, �� ����� �� ����������
	unsigned char StrInstruction[14]={'G','e','t','P','r','o','g','V','e','r','s','i','o','n'};//�������� ������ ������������ ����������� ����������
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Bluetooth_Unsigned_Int_To_Char_Buf(ProgrammVersion_D, Pos, Pos+1); Pos+=2; //������ ������������ ����������� (����)
	MessageSelectionBluetooth.Buffer[Pos]='.'; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(ProgrammVersion_M, Pos, Pos+1); Pos+=2; //������ ������������ ����������� (�����)
	MessageSelectionBluetooth.Buffer[Pos]='.'; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(ProgrammVersion_Y, Pos, Pos+1); Pos+=2; //������ ������������ ����������� (���)
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////     ������� ���������� ���������� ���������� UploadProgramm      /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_UploadProgramm(void) //��������� � ���������� ����� ���������� ���������
	{
	unsigned char StrInstruction[14]={'U','p','l','o','a','d','P','r','o','g','r','a','m','m'};
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned char CRC_Ok; //���� ������������ ����������� �����
	unsigned int i; //�������
	unsigned int DelimiterData1Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int LengthData; //������ ������������ � flash ������ ������ (���������� ���� �� 4-� ����)
	unsigned int CRC32; //�������� ����������� ����� ��� ����� ������
	if (ServiceCode!=141592) {return;} //��� ���������� ������ (141592 - �������� ���������, 832735 - ����������, 793238 - ��������� ��������)
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//�������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//���� ���� "������" ������
	//������������ ������� ������� � ����� "������"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //���� � ����� "������" ���������� ������ �������
		}
	//�������� ���������� ��������� ����� "������"
	if (DelimiterData1Pos==0) {Bluetooth_Send_ErrorData(); return;} //���� � ����� "������" ������ 1 �������
	//�������� ������� ��������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	if (DelimiterData1Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	//�������� ������� �������� ����� "������" (���������� ����) �� ������������
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}}//���� ������ - �� �������� ������
	LengthData=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.DataPosStart, DelimiterData1Pos-1); //������ ������������ � flash ������ ������ (���������� ���� �� 4-� ����)
	if (LengthData>32760) {return; Bluetooth_Send_ErrorData(); return;}//���� ������ ����� ������ ��������� ����� ���������� ������
	//�������� ������� �������� ����� "������" (����������� �����) �� ������������
	for (i = DelimiterData1Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //���� ������ - �� �������� ������
	CRC32=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, MessageInterpretationBluetooth.DataPosEnd); //����������� ����� CRC32
	//������������ �������� ���������
	CRC_Ok=Recive_Data_To_Flash(ProgrammAddressBuf, LengthData, SectorProgrammBuf, CRC32); //������������ �������� ���������
	if (CRC_Ok) //���� ������� ����������� �����
		{
		Bluetooth_Send_CRC_Ok();
		Jump_To_Application(ProgrammatorAddress); //������� � ����������� �������������
		}
	else
		{Bluetooth_Send_CRCError();}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////     ������� ���������� ���������� ���������� UploadSettings      /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_UploadSettings(void) //��������� � ���������� ������ � ����������� � �����������
	{
	unsigned char StrInstruction[14]={'U','p','l','o','a','d','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned char CRC_Ok; //���� ������������ ����������� �����
	unsigned int i; //�������
	unsigned int DelimiterData1Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int LengthData; //������ ������������ � flash ������ ������ (���������� ���� �� 4-� ����)
	unsigned int CRC32; //�������� ����������� ����� ��� ����� ������
	if (ServiceCode!=793238) {return;} //��� ���������� ������ (141592 - �������� ���������, 832735 - ����������, 793238 - ��������� ��������)
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//�������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//���� ���� "������" ������
	//������������ ������� ������� � ����� "������"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //���� � ����� "������" ���������� ������ �������
		}
	//�������� ���������� ��������� ����� "������"
	if (DelimiterData1Pos==0) {Bluetooth_Send_ErrorData(); return;} //���� � ����� "������" ������ 1 �������
	//�������� ������� ��������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	if (DelimiterData1Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	//�������� ������� �������� ����� "������" (���������� ����) �� ������������
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}}//���� ������ - �� �������� ������
	LengthData=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.DataPosStart, DelimiterData1Pos-1); //������ ������������ � flash ������ ������ (���������� ���� �� 4-� ����)
	if (LengthData>254) {Bluetooth_Send_ErrorData(); return;}//���� ������ ����� ������ ��������� ����� ���������� ������
	//�������� ������� �������� ����� "������" (����������� �����) �� ������������
	for (i = DelimiterData1Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //���� ������ - �� �������� ������
	CRC32=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, MessageInterpretationBluetooth.DataPosEnd); //����������� ����� CRC32
	//������������ �������� ����������
	CRC_Ok=Recive_Data_To_Flash(ParametrAddressBuf, LengthData, SectorParametrBuf, CRC32); //������������ �������� ������� ����������
	if (CRC_Ok) //���� ������� ����������� �����
		{
		Bluetooth_Send_CRC_Ok();
		Copy_Parametr_Buf(); //���������� ��������� �� ������ � ���������� ���������
		Read_Parametr_Device(); //������ ����������
		}
	else
		{Bluetooth_Send_CRCError();}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////     ������� ���������� ���������� ���������� DownloadSettings    /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_DownloadSettings(void) //��������� ��  ���������� ������ � ����������� � �����������
	{
	if (MessageInterpretationBluetooth.Address==0) {return;} //���� ����� ��������� �����������������, �� ����� �� ����������
	unsigned char StrInstruction[16]={'D','o','w','n','l','o','a','d','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=16; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	unsigned char TempByte; //��������� ����
	uint32_t Address; //����� ������ ������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	MessageSelectionBluetooth.Buffer[MessageInterpretationBluetooth.TypePosStart]='R'; //���������� ��� ���������
	Pos=MessageInterpretationBluetooth.DataPosStart;
	MessageSelectionBluetooth.Buffer[Pos]='0'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='2'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='5'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='6'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(Calculate_CRC(ParametrAddress, 256),Pos,Pos+10); Pos+=11; //����������� �����
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //���������� �����
	Delay_us_Bluetooth(1000000); //�������� 1 ���
	//�������� ������
	for (i = 0; i < 1024; ++i)
		{
		Address=ParametrAddress+i; TempByte=*(unsigned char*)Address;
		while (USART_GetFlagStatus(USART_BT, USART_FLAG_TC) == RESET) {};
		Delay_us_Bluetooth(1500); //�������� 1.5 ��
		USART_SendData(USART_BT, TempByte); //���������� ���� � USART_BT
		}
	while (USART_GetFlagStatus(USART_BT, USART_FLAG_TC) == RESET) {}; //��������� ���� ����� ����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////     ������� ���������� ���������� ���������� SetServiceMode       /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetServiceMode(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','S','e','r','v','i','c','e','M','o','d','e'};
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ����� "������" �� ������������
	if (MessageInterpretationBluetooth.DataPosStart>MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //���� ���� "������" ������, �� ����������� ���������� �������
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}}//���� ������ - �� �������� ������
	ServiceCode=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.DataPosStart, MessageInterpretationBluetooth.DataPosEnd); //��� ���������� ������ (141592 - �������� ���������, 832735 - ����������, 793238 - ��������� ��������)
	if (MessageInterpretationBluetooth.Address==0) {return;} //���� ����� ��������� �����������������, �� ����� �� ����������
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////      ������� ���������� ���������� ���������� SetCycleSettings        ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetCycleSettings(void)
	{
	unsigned char StrInstruction[16]={'S','e','t','C','y','c','l','e','S','e','t','t','i','n','g','s'};//���������� ��������� �������������� �����
	unsigned char InstructionLength=16; //������ ���������� � ������� ������������ ���������
	unsigned int DelimiterData1Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData2Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//�������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//���� ���� "������" ������
	//������������ ������� ������� � ����� "������"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos!=0)&(DelimiterData2Pos==0)) {DelimiterData2Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //���� � ����� "������" ���������� ������ �������
		}
	//�������� ���������� ��������� ����� "������"
	if (DelimiterData2Pos==0) {Bluetooth_Send_ErrorData(); return;} //���� � ����� "������" ������ 1 �������
	//�������� ������� ��������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	if (DelimiterData2Pos==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	if (DelimiterData2Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	//�������� ������� �������� ����� "������" (CyclePeriod) �� ������������
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}}//���� ������ - �� �������� ������
	CyclePeriod=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.DataPosStart, DelimiterData1Pos-1); //������ ���������
	//�������� ������� �������� ����� "������" (CycleStart) �� ������������
	for (i = DelimiterData1Pos+1; i < DelimiterData2Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //���� ������ - �� �������� ������
	CycleStart=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, DelimiterData2Pos-1); //��������� ����� ���������
	//�������� �������� �������� ����� "������" (CycleSendDataPeriod) �� ������������
	for (i = DelimiterData2Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //���� ������ - �� �������� ������
	CycleSendDataPeriod=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData2Pos+1, MessageInterpretationBluetooth.DataPosEnd); //������ �������� ������
	if (CycleSendDataPeriod>MaximumPeriodSendData) {Bluetooth_Send_ErrorData(); return;} //���� ���������� ��������� �������� ������ ������ 255
	if (CyclePeriod>86400) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������ 24 �����
	if (CyclePeriod<MinimumPeriod) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������ 5 �����
	if (CycleStart>86399) {Bluetooth_Send_ErrorData(); return;}
	CurrentCycleSendDataPeriod=0;//����� �������� �������� �������� ������
	SetCycle_Settings(); //��������� �������� ������ ���������
	RTC_Set_Next_Cycle_Time(); //��������������� ����� ���������� �����
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////      ������� ���������� ���������� ���������� GetCycleSettings        ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetCycleSettings(void)
	{
	unsigned char StrInstruction[16]={'G','e','t','C','y','c','l','e','S','e','t','t','i','n','g','s'};//�������� ��������� �������������� �����
	unsigned char InstructionLength=16; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	Pos=MessageInterpretationBluetooth.DataPosStart;
	GetCycle_Settings(); //���������� ��������� ������ �����
	Bluetooth_Unsigned_Int_To_Char_Buf(CyclePeriod, Pos, Pos+5); Pos+=6; //������ ���������
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(CycleStart, Pos, Pos+5); Pos+=6; //����� ���������� ��������� �� ������ �����
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(CycleSendDataPeriod, Pos, Pos+3); Pos+=4; //������ �������� ������
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� SetClock              ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetClock(void)
	{
	unsigned char StrInstruction[8]={'S','e','t','C','l','o','c','k'};//���������� ��������� �������������� �����
	unsigned char InstructionLength=8; //������ ���������� � ������� ������������ ���������
	unsigned int DelimiterData1Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData2Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData3Pos=0; //������� �������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData4Pos=0; //������� ���������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData5Pos=0; //������� ������ ����������� (�������) � ����� "������"
	unsigned int i; //�������
	unsigned int TS_Year=0; //���
	unsigned int TS_Month=0; //�����
	unsigned int TS_Day=0; //����
	unsigned int TS_Hour=0; //���
	unsigned int TS_Min=0; //������
	unsigned int TS_Sec=0; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//�������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//���� ���� "������" ������
	//������������ ������� ������� � ����� "������"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData4Pos!=0)&(DelimiterData5Pos==0)) {DelimiterData5Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData3Pos!=0)&(DelimiterData4Pos==0)) {DelimiterData4Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData2Pos!=0)&(DelimiterData3Pos==0)) {DelimiterData3Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos!=0)&(DelimiterData2Pos==0)) {DelimiterData2Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //���� � ����� "������" ���������� ������ �������
		}
	//�������� ���������� ��������� ����� "������"
	if (DelimiterData5Pos==0) {Bluetooth_Send_ErrorData(); return;} //���� � ����� "������" ������ 1 �������
	//�������� ������� ��������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	if (DelimiterData2Pos==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	if (DelimiterData3Pos==DelimiterData2Pos) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	if (DelimiterData4Pos==DelimiterData3Pos) {Bluetooth_Send_ErrorData(); return;} //���� ��������� ������� ������
	if (DelimiterData5Pos==DelimiterData4Pos) {Bluetooth_Send_ErrorData(); return;} //���� ����� ������� ������
	if (DelimiterData5Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	//�������� ������� �������� ����� "������" (���) �� ������������
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}}//���� ������ - �� �������� ������
	TS_Year=Bluetooth_Char_To_Unsigned_Int_Buf(MessageInterpretationBluetooth.DataPosStart, DelimiterData1Pos-1);
	//�������� ������� �������� ����� "������" (�����) �� ������������
	for (i = DelimiterData1Pos+1; i < DelimiterData2Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //���� ������ - �� �������� ������
	TS_Month=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, DelimiterData2Pos-1);
	//�������� ������� �������� ����� "������" (����) �� ������������
	for (i = DelimiterData2Pos+1; i < DelimiterData3Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //���� ������ - �� �������� ������
	TS_Day=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData2Pos+1, DelimiterData3Pos-1);
	//�������� ������� �������� ����� "������" (���) �� ������������
	for (i = DelimiterData3Pos+1; i < DelimiterData4Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //���� ������ - �� �������� ������
	TS_Hour=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData3Pos+1, DelimiterData4Pos-1);
	//�������� ������� �������� ����� "������" (������) �� ������������
	for (i = DelimiterData4Pos+1; i < DelimiterData5Pos; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //���� ������ - �� �������� ������
	TS_Min=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData4Pos+1, DelimiterData5Pos-1); //��������� ����� ���������
	//�������� �������� �������� ����� "�������" (CycleSendDataPeriod) �� ������������
	for (i = DelimiterData5Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //���� ������ - �� �������� ������
	TS_Sec=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData5Pos+1, MessageInterpretationBluetooth.DataPosEnd); //������ �������� ������
	//�������� ������������ ������
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
	RTC_Set_Date_Time(); //��������� �������� �������
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� GetClock              ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetClock(void)
	{
	unsigned char StrInstruction[8]={'G','e','t','C','l','o','c','k'};//���������� ��������� �������������� �����
	unsigned char InstructionLength=8; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	Pos=MessageInterpretationBluetooth.DataPosStart;
	RTC_Get_Date_Time(); //���������� ������� ������� �������
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.YY, Pos, Pos+1); Pos+=2; //���
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.MM, Pos, Pos+1); Pos+=2; //�����
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.DD, Pos, Pos+1); Pos+=2; //����
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.hour, Pos, Pos+1); Pos+=2; //����
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.min, Pos, Pos+1); Pos+=2; //������
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++;
	Bluetooth_Unsigned_Int_To_Char_Buf(DateTime.sec, Pos, Pos+1); Pos+=2; //�������
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� SetFTPSettings               /////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetFTPSettings(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','F','T','P','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
		//�������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//���� ���� "������" ������
	//�������� ������ �� ������������
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]<'0') {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if (MessageSelectionBluetooth.Buffer[i]>'f') {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if ((MessageSelectionBluetooth.Buffer[i]>'9')&&(MessageSelectionBluetooth.Buffer[i]<'A')) {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if ((MessageSelectionBluetooth.Buffer[i]>'F')&&(MessageSelectionBluetooth.Buffer[i]<'a')) {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		}
	//������ ������������� � ���������� ���
	Bluetooth_HexChar_Buf_To_FTP_Settings(MessageInterpretationBluetooth.DataPosStart,MessageInterpretationBluetooth.DataPosEnd);
	Save_FTP_Settings(); //������ ����������� � SRAM
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� GetFTPSettings               /////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetFTPSettings(void)
	{
	unsigned char StrInstruction[14]={'G','e','t','F','T','P','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	unsigned char Len_STR; //������ ���������� ������
	unsigned char TempByte; //��������� ����
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Read_FTP_Settings(); //������ ����������� �� SRAM
	Len_STR=strlen(GSM.FTP_Settings); //������ ���������� ������
	if ((Len_STR>0)&&(Len_STR<257)) //���� ������ ����� ���������� ������
		{
		for (i = 0; i < Len_STR; ++i)
			{
			//���������� ������� ��������
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
			//���������� ������� ��������
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
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� SetGPRSSettings               ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetGPRSSettings(void)
	{
	unsigned char StrInstruction[15]={'S','e','t','G','P','R','S','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=15; //������ ���������� � ������� ������������ ���������
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
		//�������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//���� ���� "������" ������
	//�������� ������ �� ������������
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]<'0') {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if (MessageSelectionBluetooth.Buffer[i]>'f') {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if ((MessageSelectionBluetooth.Buffer[i]>'9')&&(MessageSelectionBluetooth.Buffer[i]<'A')) {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if ((MessageSelectionBluetooth.Buffer[i]>'F')&&(MessageSelectionBluetooth.Buffer[i]<'a')) {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		}
	//������ ������������� � ���������� ���
	Bluetooth_HexChar_Buf_To_GPRS_Settings(MessageInterpretationBluetooth.DataPosStart,MessageInterpretationBluetooth.DataPosEnd);
	Save_GPRS_Settings(); //������ ����������� � SRAM
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� GetGPRSSettings              /////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetGPRSSettings(void)
	{
	unsigned char StrInstruction[15]={'G','e','t','G','P','R','S','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=15; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	unsigned char Len_STR; //������ ���������� ������
	unsigned char TempByte; //��������� ����
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Read_GPRS_Settings(); //������ ����������� �� SRAM
	Len_STR=strlen(GSM.GPRS_Settings); //������ ���������� ������
	if ((Len_STR>0)&&(Len_STR<257)) //���� ������ ����� ���������� ������
		{
		for (i = 0; i < Len_STR; ++i)
			{
			//���������� ������� ��������
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
			//���������� ������� ��������
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
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� SetGPRSUserPass               ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetGPRSUserPass(void)
	{
	unsigned char StrInstruction[15]={'S','e','t','G','P','R','S','U','s','e','r','P','a','s','s'};
	unsigned char InstructionLength=15; //������ ���������� � ������� ������������ ���������
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
		//�������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//���� ���� "������" ������
	//�������� ������ �� ������������
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]<'0') {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if (MessageSelectionBluetooth.Buffer[i]>'f') {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if ((MessageSelectionBluetooth.Buffer[i]>'9')&&(MessageSelectionBluetooth.Buffer[i]<'A')) {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if ((MessageSelectionBluetooth.Buffer[i]>'F')&&(MessageSelectionBluetooth.Buffer[i]<'a')) {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		}
	//������ ������������� � ���������� ���
	Bluetooth_HexChar_Buf_To_GPRS_User_Pass(MessageInterpretationBluetooth.DataPosStart,MessageInterpretationBluetooth.DataPosEnd);
	Save_GPRS_User_Pass(); //������ ����������� � SRAM
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� GetGPRSUserPass              /////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetGPRSUserPass(void)
	{
	unsigned char StrInstruction[15]={'G','e','t','G','P','R','S','U','s','e','r','P','a','s','s'};
	unsigned char InstructionLength=15; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	unsigned char Len_STR; //������ ���������� ������
	unsigned char TempByte; //��������� ����
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Read_GPRS_User_Pass(); //������ ����������� �� SRAM
	Len_STR=strlen(GSM.GPRS_User_Pass); //������ ���������� ������
	if ((Len_STR>0)&&(Len_STR<257)) //���� ������ ����� ���������� ������
		{
		for (i = 0; i < Len_STR; ++i)
			{
			//���������� ������� ��������
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
			//���������� ������� ��������
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
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� SetFTPFileName               ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetFTPFileName(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','F','T','P','F','i','l','e','N','a','m','e'};
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned int DelimiterData1Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData2Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
		//�������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//���� ���� "������" ������
	//������������ ������� ������� � ����� "������"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //���� � ����� "������" ���������� ������ �������
		}
	//�������� ���������� ��������� ����� "������"
	if (DelimiterData1Pos==0) {Bluetooth_Send_ErrorData(); return;} //���� � ����� "������" ������ 1 �������
	//�������� ������� ��������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart==DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	if (DelimiterData2Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	//�������� ������� �������� ����� �� ������������ (������)
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]<'0') {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if (MessageSelectionBluetooth.Buffer[i]>'f') {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if ((MessageSelectionBluetooth.Buffer[i]>'9')&&(MessageSelectionBluetooth.Buffer[i]<'A')) {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if ((MessageSelectionBluetooth.Buffer[i]>'F')&&(MessageSelectionBluetooth.Buffer[i]<'a')) {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		}
	//�������� ������� �������� ����� "������" �� ������������
	for (i = DelimiterData1Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //���� ������ - �� �������� ������
	//������ ������������� � ���������� ���
	Bluetooth_HexChar_Buf_To_FTP_FileName(MessageInterpretationBluetooth.DataPosStart,DelimiterData1Pos-1);
	GSM.FTP_Append=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, MessageInterpretationBluetooth.DataPosEnd); //������ �������� ������
	Save_FTP_FileName(); //������ ����������� � SRAM
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� GetFTPFileName              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetFTPFileName(void)
	{
	unsigned char StrInstruction[14]={'G','e','t','F','T','P','F','i','l','e','N','a','m','e'};
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	unsigned char Len_STR; //������ ���������� ������
	unsigned char TempByte; //��������� ����
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Read_FTP_FileName(); //������ ����������� �� SRAM
	Len_STR=strlen(GSM.FTP_File_Name); //������ ���������� ������
	if ((Len_STR>0)&&(Len_STR<257)) //���� ������ ����� ���������� ������
		{
		for (i = 0; i < Len_STR; ++i)
			{
			//���������� ������� ��������
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
			//���������� ������� ��������
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
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� SetSMSSettings               ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetSMSSettings(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','S','M','S','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned int DelimiterData1Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData2Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
		//�������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//���� ���� "������" ������
	//������������ ������� ������� � ����� "������"
	for (i = MessageInterpretationBluetooth.DataPosStart; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if ((MessageSelectionBluetooth.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //���� � ����� "������" ���������� ������ �������
		}
	//�������� ���������� ��������� ����� "������"
	if (DelimiterData1Pos==0) {Bluetooth_Send_ErrorData(); return;} //���� � ����� "������" ������ 1 �������
	//�������� ������� ��������� ����� "������"
	if ((MessageInterpretationBluetooth.DataPosStart+12)!=DelimiterData1Pos) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� �� ����� 12 ��������
	if (DelimiterData2Pos==MessageInterpretationBluetooth.DataPosEnd) {Bluetooth_Send_ErrorData(); return;} //���� ������ ������� ������
	//�������� ������� �������� ����� �� ������������ (������)
	for (i = MessageInterpretationBluetooth.DataPosStart; i < DelimiterData1Pos; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]<'+') {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		if (MessageSelectionBluetooth.Buffer[i]>'9') {Bluetooth_Send_ErrorData(); return;} //���� ������ �� �����
		}
	//�������� ������� �������� ����� "������" �� ������������
	for (i = DelimiterData1Pos+1; i <= MessageInterpretationBluetooth.DataPosEnd; ++i)
		{if ((MessageSelectionBluetooth.Buffer[i]<47)|(MessageSelectionBluetooth.Buffer[i]>58)) {Bluetooth_Send_ErrorData(); return;}} //���� ������ - �� �������� ������
	//����������� ������
	for (i = 0; i < 12; ++i) {GSM.SMSNumber[i]=MessageSelectionBluetooth.Buffer[MessageInterpretationBluetooth.DataPosStart+i];}
	GSM.SMSNumber[12]='\0';
	//��� �������
	GSM.SMSCode=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, MessageInterpretationBluetooth.DataPosEnd);
	Save_SMS_Settings(); //������ ����������� � SRAM
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� GetSMSSettings              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetSMSSettings(void)
	{
	unsigned char StrInstruction[14]={'G','e','t','S','M','S','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	Pos=MessageInterpretationBluetooth.DataPosStart;
	Read_SMS_Settings(); //������ ����������� �� SRAM
	for (i = 0; i < 12; ++i) {MessageSelectionBluetooth.Buffer[Pos]=GSM.SMSNumber[i];Pos++;} //���������� ����� ��������
	MessageSelectionBluetooth.Buffer[Pos]=','; Pos++; //�������
	Bluetooth_Unsigned_Int_To_Char_Buf(GSM.SMSCode, Pos, Pos+10);Pos+=11; //������������ ��� �������
	MessageSelectionBluetooth.Buffer[Pos]='/'; Pos++;
	MessageSelectionBluetooth.Buffer[Pos]='%'; Pos++;
	MessageSelectionBluetooth.Buffer_Len=Pos;
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� SetChIDList              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_SetChIDList(void)
	{
	unsigned char StrInstruction[11]={'S','e','t','C','h','I','D','L','i','s','t'};
	unsigned char InstructionLength=11; //������ ���������� � ������� ������������ ���������
	unsigned int i; //�������
	unsigned int j; //�������
	unsigned int StartChID; //��������� ������� �� ������
	unsigned int StopChID; //��������� ������� �� ������
	//�������� �� ������������ ����������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//�������� ����� "������"
	if (MessageInterpretationBluetooth.DataPosStart+2>MessageInterpretationBluetooth.DataPosEnd)  {Bluetooth_Send_ErrorData(); return;}//���� ���� "������" ������
	//�������� ������� �������� ����� �� ������������ (������)
	for (i = MessageInterpretationBluetooth.DataPosStart; i < MessageInterpretationBluetooth.DataPosEnd+1; ++i)
		{
		if (((MessageSelectionBluetooth.Buffer[i]<'0')||(MessageSelectionBluetooth.Buffer[i]>'9'))&&(MessageSelectionBluetooth.Buffer[i]!=',')) {Bluetooth_Send_ErrorData(); return;} //���� ������ �� ����� � �� �������
		}
	//��������� ���� � �� �������
	for (i = 0; i <  64; ++i)
		{
		ChIDList[i]=0;
		}
	//������������ ������������� ������
	j=0;
	StartChID=MessageInterpretationBluetooth.DataPosStart;
	for (i = MessageInterpretationBluetooth.DataPosStart; i <  MessageInterpretationBluetooth.DataPosEnd; ++i)
		{
		if (MessageSelectionBluetooth.Buffer[i]==',')
			{
			StopChID=i;
			if (StopChID>StartChID+1)
				{
				ChIDList[j]=Bluetooth_Char_To_Unsigned_Int_Buf(StartChID,StopChID-1); //������� �����
				j++;
				}
			StartChID=i+1;
			}
		if (j>63) {Bluetooth_Send_ErrorData(); return;} //���� ��������� ���������� ��������
		}
	if ( MessageInterpretationBluetooth.DataPosEnd>StartChID+1)
		{
		ChIDList[j]=Bluetooth_Char_To_Unsigned_Int_Buf(StartChID, MessageInterpretationBluetooth.DataPosEnd); //������� �����
		}
	Save_ChIDList(); //����������� � SRAM
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� GetChIDList              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_GetChIDList(void)
	{
	unsigned char StrInstruction[11]={'G','e','t','C','h','I','D','L','i','s','t'};
	unsigned char InstructionLength=11; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	Read_ChIDList(); //�������� �� SRAM ChIDList
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
	Bluetooth_Send_Message(); //���������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� DownloadData              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_DownloadData(void)
	{
	unsigned char StrInstruction[12]={'D','o','w','n','l','o','a','d','D','a','t','a'};
	unsigned char InstructionLength=12; //������ ���������� � ������� ������������ ���������
	unsigned int pos=0; //������� ������� � ������ ���������
	unsigned int i; //�������
	signed int TempVal;
	TMemoryResult TempResult;
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//� ��������� ����������� ��������� ���������� ����������
	for (i = 0x00; i < 0x3000; ++i)
		{
		TempResult=Read_Record(i);
		if (TempResult.FlagMeas!=0xff)
			{
			//����
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
			//���������� ������
			MessageSelectionBluetooth.Buffer[pos]=0x0d;pos++;
			MessageSelectionBluetooth.Buffer[pos]=0x0a;pos++;
			//�������� ������
			MessageSelectionBluetooth.Buffer_Len=pos;
			Bluetooth_Send_Message(); //���������� �����
			}
		}
	MessageSelectionBluetooth.Buffer[0]='E';
	MessageSelectionBluetooth.Buffer[1]='n';
	MessageSelectionBluetooth.Buffer[2]='d';
	//���������� ������
	MessageSelectionBluetooth.Buffer[3]=0x0d;
	MessageSelectionBluetooth.Buffer[4]=0x0a;
	//�������� ������
	MessageSelectionBluetooth.Buffer_Len=4;
	Bluetooth_Send_Message(); //���������� �����
	ClearFTPFlags(); //������������ ����� �������� ������ �� FTP
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////      ������� ���������� ���������� ���������� test        /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Bluetooth_Execution_Test(void)
	{
	unsigned char StrInstruction[4]={'T','e','s','t'};
	unsigned char InstructionLength=4; //������ ���������� � ������� ������������ ���������
	unsigned int Pos=0; //������� ������� � ������
	unsigned int i; //�������
	if ((MessageInterpretationBluetooth.InstructionPosEnd-MessageInterpretationBluetooth.InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = MessageInterpretationBluetooth.InstructionPosStart; i <= MessageInterpretationBluetooth.InstructionPosEnd; ++i) {if (MessageSelectionBluetooth.Buffer[i]!=StrInstruction[i-MessageInterpretationBluetooth.InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ������������ ������
	if (MessageInterpretationBluetooth.Address!=DeviceAddress) {return;} //���� �� ��������� ����� � ��������� � ������� ����������
	//����������� ������������ ���� ������
	///////////////////////////////////////
	//1. �������� � ����� ���
	///////////////////////////////////////
	//�������� ��� � �������� �������
	Start_Led(100);//������� ������� ������� ����������
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
	GSM.Attempt=3; //3 �������
	GSM.ATErrorFlag=0xff;
	while ((GSM.Attempt>0)&&(GSM.ATErrorFlag!=0))
		{
		GSM_Send_SMS(30000);
		GSM.Attempt--;
		}
	//����� � ������ ���� ���
	GSM_Read_ALL_SMS(10000);
	//�������� ���� ���
	GSM_Delete_All_SMS(5000);
	///////////////////////////////////////
	//2. ��������� ���� ��������
	///////////////////////////////////////
	Start_Led(1000);//������� ������� ������� ����������
	MeasCycle();
	///////////////////////////////////////
	//3. �������� ������ �� ��� ������
	///////////////////////////////////////
	Start_Led(100);//������� ������� ������� ����������
	GSM_Send_Data_FTP();
	///////////////////////////////////////
	//4. ��������� ����� ��������� �������
	///////////////////////////////////////
	CyclePeriod=30; //��������������� ������ 30 ������
	RTC_Set_Next_Cycle_Time();
	SetNameBlooetooth();
	//����������� ������� ����������
	GPIO_ResetBits(Power_Port, Power_Pin); Delay_us(3000000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////      ������� ���������� ��������� ����� bluetooth ������    /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetNameBlooetooth(void)
	{
	//������� ������ � AT �����
	GPIO_SetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(10000);
	//GPIO_ResetBits(BT_Reset_Port, BT_Reset_Pin);
	Delay_us(10000);
	//GPIO_SetBits(BT_Reset_Port, BT_Reset_Pin);
	//��������� ����� ����������
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
	//������� ������ � AT �����
	GPIO_ResetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(1000000);
	//������� ������ � AT �����
	GPIO_SetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(10000);
	GPIO_ResetBits(BT_Reset_Port, BT_Reset_Pin);
	Delay_us(10000);
	GPIO_SetBits(BT_Reset_Port, BT_Reset_Pin);
	//��������� ����� ����������
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
	//������� ������ � AT �����
	GPIO_ResetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(1000000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////      ������� ��������� ����� ������ bluetooth ������     /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Test_Chanela_Blooetooth(void)
{
	//������� ������ � AT ����� ���������
	GPIO_SetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(10000);
	//GPIO_ResetBits(BT_Reset_Port, BT_Reset_Pin);
	Delay_us(10000);
	//GPIO_SetBits(BT_Reset_Port, BT_Reset_Pin);
	//�������� AT �������
	Delay_us(1000000);
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'A');
	while(!USART_GetFlagStatus(USART_BT,USART_FLAG_TC)); USART_SendData(USART_BT,'T');

	Delay_us(10000);
	// ������������� �����
	char buffer_RX[2] = {0,};
	while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //��������� �������� ����� � ����� bluetooth
	buffer_RX[0]=USART_ReceiveData(USART_BT); //���������� ���� �� ������ Usart
	while (USART_GetFlagStatus(USART_BT, USART_FLAG_RXNE) == RESET){}; //��������� �������� ����� � ����� bluetooth
	buffer_RX[1]=USART_ReceiveData(USART_BT); //���������� ���� �� ������ Usart

	Delay_us(1000000);
	//������� ������ � AT ����� �����
	GPIO_ResetBits(BT_AT_Port, BT_AT_Pin);
	Delay_us(1000000);
}

















////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////         ������� ������� ������� ����������            ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////            ������� ���������� ������������� ������� �������              /////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Power_Init(void)
	{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//����� ��������� ������� ����������
	GPIO_InitStructure.GPIO_Pin = Power_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Power_Port, &GPIO_InitStructure);
	GPIO_SetBits(Power_Port, Power_Pin); //���������� ������� ����������
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM_Led, ENABLE); //������������ ������� ����������
	//����� ������� Bluetooth � GSM �������
	GPIO_InitStructure.GPIO_Pin = Power_BT_GSM_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Power_BT_GSM_Port, &GPIO_InitStructure);
	//����� ������� ��������� �� ����� RS485
	GPIO_InitStructure.GPIO_Pin = Power_RS485_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Power_RS485_Port, &GPIO_InitStructure);
	//����� ����������
	GPIO_InitStructure.GPIO_Pin = Led_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(Led_Port, &GPIO_InitStructure);
    //������������� ������� ����������
    TIM_TimeBaseInitTypeDef Timer_InitStructure;
    TIM_TimeBaseStructInit(&Timer_InitStructure);
    Timer_InitStructure.TIM_Prescaler = 8000-1; //������������ ������� (������� ����� ������� 1 ���)
    Timer_InitStructure.TIM_Period = 1000; //������ ����� ����������� ������ 1 � (1 ��)
    Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ����������� ������������ �������
    Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; //����������� ������ - �� ����������
    Timer_InitStructure.TIM_RepetitionCounter = 0; //������� ����� ��������� ������ ������������ ��������
    TIM_TimeBaseInit(Timer_Led, &Timer_InitStructure); //������������� �������
    //������������ ���������� ������� ����������
    TIM_ITConfig(Timer_Led, TIM_IT_Update, ENABLE); //�������� ���������� �� ������������ ��������
	NVIC_SetPriority(Timer_Led_IRQn,Prioritet_LedTimer); //��������������� ��������� ����������
	NVIC_EnableIRQ(Timer_Led_IRQn); //����������� ��������� ���������� �� �������
	GPIO_ResetBits(Power_RS485_Port, Power_RS485_Pin); //����������� ������� ��������� �� ����� RS485
	//��������� �������
	TIM_Cmd(Timer_Led, DISABLE);
	GPIO_SetBits(Led_Port, Led_Pin); //�������� ���������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////       ������� ���������� ������� ��������� ������� GSM � Bluetooth �������              //////////////
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
//////////////////////////////     ������� ������������ ������� ����������      ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TIM3_IRQHandler(void)
	{
	TIM_ClearITPendingBit(Timer_Led, TIM_IT_Update); //����� ����� �������� ����������
	GPIO_ToggleBits(Led_Port, Led_Pin); //����������� ���������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////         ������� ������� ������� ����������       ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Start_Led(unsigned int Period) //������, ��
	{
	TIM_Cmd(Timer_Led, DISABLE);
	//������������� �������
    TIM_TimeBaseInitTypeDef Timer_InitStructure;
    TIM_TimeBaseStructInit(&Timer_InitStructure);
    Timer_InitStructure.TIM_Prescaler = 8000-1; //������������ ������� (������� ����� ������� 1 ���)
    Timer_InitStructure.TIM_Period = Period; //������ ����� ����������� ������ Period/1000�
    Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ����������� ������������ �������
    Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; //����������� ������ - �� ����������
    Timer_InitStructure.TIM_RepetitionCounter = 0; //������� ����� ��������� ������ ������������ ��������
    TIM_TimeBaseInit(Timer_Led, &Timer_InitStructure); //������������� �������
	TIM_Cmd(Timer_Led, ENABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////      ������� ��������� ������� ����������        ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Stop_Led()
	{
	TIM_Cmd(Timer_Led, DISABLE);
	GPIO_ResetBits(Led_Port, Led_Pin); //��������� ���������
	}




















////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////                  ������� ��� ������ � RS485                             ///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////         ������� ������������� RS85 ������                  ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RS485_Init(void)
	{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART_RS485, ENABLE); //������������ USART RS485
	//����� RE_DE RS485
	GPIO_InitStructure.GPIO_Pin = RS485_RE_DE_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(RS485_RE_DE_Port, &GPIO_InitStructure);
	//����� RS485 (Tx �����������)
	GPIO_InitStructure.GPIO_Pin = RS485_Tx_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(RS485_Tx_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(RS485_Tx_Port, RS485_Tx_GPIO_PinSource, GPIO_AF_USART_RS485);
	//����� RS485 (Rx �����������)
	GPIO_InitStructure.GPIO_Pin = RS485_Rx_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(RS485_Rx_Port, &GPIO_InitStructure);
	GPIO_PinAFConfig(RS485_Rx_Port, RS485_Rx_GPIO_PinSource, GPIO_AF_USART_RS485);
	//������������ USART RS485
	USART_InitStructure.USART_BaudRate = 9600;                                       // ��������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // ����� ������ 1����/8���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // 1 ����-���
	USART_InitStructure.USART_Parity = USART_Parity_No;                              // ��� �������� ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // ��������� ����� � ��������
	USART_Init(USART_RS485, &USART_InitStructure);
	USART_Cmd(USART_RS485, ENABLE);
	GPIO_ResetBits(RS485_RE_DE_Port, RS485_RE_DE_Pin); //���������� �������� RS485
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           �������� ������� � RS485 ������                         ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RS485_Send_Char(unsigned char Byte)
	{
	Timeouts.TimeoutRS485_Char=Send_Char_Timeout; //����������� ������� �������� ������� ����� �� Usart
	while ((USART_GetFlagStatus(USART_RS485, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutRS485_Char>0)) {}; //��������� ��������� �������� �����
	USART_SendData(USART_RS485, Byte); //���������� ������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           �������� ������ � RS485 ������                         ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RS485_Send_String(void)
	{
	unsigned int i; //�������
	for (i = 0; i < strlen(RS485.Buffer); ++i)
		{
		Timeouts.TimeoutRS485_Char=Send_Char_Timeout;  //����������� ������� �������� ������� ����� �� Usart
		while ((USART_GetFlagStatus(USART_RS485, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutRS485_Char>50)) {}; //��������� ��������� �������� �����
		USART_SendData(USART_RS485,  RS485.Buffer[i]); //���������� ������
		}
	while ((USART_GetFlagStatus(USART_RS485, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutRS485_Char>50)) {}; //��������� ��������� �������� �����
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           ��������� ����������� ���������                        ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RS485_GetValue_Sensor(unsigned int Timestamp, unsigned int ChId,unsigned int Timeout_ms)
	{
	char TempSTR[2]={' ', '\0'}; //��������� ������ �� ������ ������� + ����������� ������
	RS485.CorrectResultFlag=0x00; //������������ ���� ������������ ����������
		//������������ �������
	strcpy(RS485.Buffer," %/Q/0/");
	//� ������ ����������� Transaction
	RS485.Transaction++;
	RS485.Transaction=RS485.Transaction%10;
	TempSTR[0]=RS485.Transaction+0x30; strcat(RS485.Buffer,TempSTR);
	strcat(RS485.Buffer,"/GetValue/");
	//� ������ ����������� Timetstamp
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
	//� ������ ����������� ','
	TempSTR[0]=','; strcat(RS485.Buffer,TempSTR);
	//� ������ ����������� ChID
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
	//� ������ ����������� "/%" CR LF
	strcat(RS485.Buffer,"/%");
	TempSTR[0]=0x0d; strcat(RS485.Buffer,TempSTR);
	TempSTR[0]=0x0a; strcat(RS485.Buffer,TempSTR);
		//�������� �������
	GPIO_SetBits(RS485_RE_DE_Port, RS485_RE_DE_Pin); //���������� ���������� RS485
	Timeouts.TimeoutRS485_Instruction=5000; while (Timeouts.TimeoutRS485_Instruction>50)//��������� ��������� �����������, 5��
	RS485_Send_String(); //�������� �������
		//����� ������
	GPIO_ResetBits(RS485_RE_DE_Port, RS485_RE_DE_Pin); //���������� �������� RS485
	Timeouts.TimeoutRS485_Instruction=5000; while (Timeouts.TimeoutRS485_Instruction>50)//��������� ��������� ���������, 5��
	//������������ ����� � ��������� ���������
	RS485.Message_Complete_flag=0x00; //���� ��������� ������ ���������
	RS485.Start_Collect_Flag=0x00; //���� ������ �������� ������ ���������
	RS485.Buffer_Pos=0x00; //������� ������� � ������
	RS485.Previous_Byte=0x00; //������� ����
	RS485.Current_Byte=0x00; //���������� ����
	Timeouts.TimeoutRS485_Instruction=Timeout_ms*1000; //��������������� ������� ������ ������
	//����������� ������� ������ ���������
	while (RS485.Message_Complete_flag==0)
		{
		if (Timeouts.TimeoutRS485_Instruction<51) {return;} //���� �������� ������� �������� ����������
		if (RS485.Buffer_Pos>1023) {return;}//���� ������������ �����
		if (USART_GetFlagStatus(USART_RS485, USART_FLAG_RXNE) != RESET) //���� � �������� ������ USART ���� ����
			{
			RS485.Previous_Byte=RS485.Current_Byte; //������������ ���������� �������� ����
			RS485.Current_Byte=USART_ReceiveData(USART_RS485); //���������� ���� �� USART
			if (RS485.Start_Collect_Flag!=0) //���� ���� ������� ������ ���������
				{
				RS485.Buffer[RS485.Buffer_Pos]=RS485.Current_Byte; //��������� ������� ���� � ����� ���������
				RS485.Buffer_Pos++; //���������������� ������� ��������� ������� � ������ ���������
				RS485.Buffer_Len=RS485.Buffer_Pos; //����������� ���������� ������ � ������ ���������
				if ((RS485.Previous_Byte=='/')&&(RS485.Current_Byte=='%')) {RS485.Message_Complete_flag=0xff;} //���� ��������� ������ ����� ���������, �� ��������������� ������� ������ ���������
				}
			if ((RS485.Previous_Byte=='%')&&(RS485.Current_Byte=='/')) //���� ��������� ������ ������ ���������
				{
				RS485.Start_Collect_Flag=0xff; //��������������� ���� ������� �������� ������� ���������
				RS485.Buffer[0]=RS485.Previous_Byte; //��������� ���������� ���� � ����� ���������
				RS485.Buffer[1]=RS485.Current_Byte; //��������� ������� ���� � ����� ���������
				RS485.Buffer_Pos=2; //�������� ������� ������� � ������ ���������
				}
			}
		}
	if (RS485.Message_Complete_flag==0x00) {return;} //���� ��������� �� �������
		//�������� ������������ ���������
	unsigned int i; //�������
	unsigned int CurrentDelimiterNumber=0; //������� ����� �����������
	unsigned int TypePosStart=0; //������ ����� "��� ���������"
	unsigned int TypePosEnd=0; //����� ����� "��� ���������"
	unsigned int AddressPosStart=0; //������ ����� "�����"
	unsigned int AddressPosEnd=0; //����� ����� "�����"
	unsigned int TransactionPosStart=0; //������ ����� "����������"
	unsigned int TransactionPosEnd=0; //����� ����� "����������"
	unsigned int InstructionPosStart=0; //������ ����� "����������"
	unsigned int InstructionPosEnd=0; //����� ����� "����������"
	unsigned int DataPosStart=0; //������ ����� "������"
	unsigned int DataPosEnd=0; //����� ����� "������"
	//��������� ����������� ������ ���������
	for (i = 0; i < RS485.Buffer_Len; ++i)
		{
		if (RS485.Buffer[i]=='/') //���� ��������� �����������
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //���� ��������� 0-� �����������
				TypePosStart=i+1; //������������ ������ ����� "��� ���������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 1: //���� ��������� 1-� �����������
				TypePosEnd=i-1; //������������ ����� ����� "��� ���������"
				AddressPosStart=i+1; //������������ ������ ����� "�����"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 2: //���� ��������� 2-� �����������
				AddressPosEnd=i-1; //������������ ����� ����� "�����"
				TransactionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 3: //���� ��������� 3-� �����������
				TransactionPosEnd=i-1; //������������ ����� ����� "����������"
				InstructionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 4: //���� ��������� 4-� �����������
				InstructionPosEnd=i-1; //������������ ����� ����� "����������"
				DataPosStart=i+1; //������������ ������ ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 5: //���� ���������5-� �����������
				DataPosEnd=i-1; //������������ ����� ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				}
			}
		}
	if (CurrentDelimiterNumber!=6) {return;} //����������� ���������� ������ � ���������
	//����������� ������ ������
	if (TypePosStart>TypePosEnd) {return;}//���� ���� "���" ������
	if (AddressPosStart>AddressPosEnd) {return;}//���� ���� "�����" ������
	if (TransactionPosStart>TransactionPosEnd) {return;}//���� ���� "����������" ������
	if (InstructionPosStart>InstructionPosEnd) {return;}//���� ���� "����������" ������
	if (DataPosStart>DataPosEnd) {return;}//���� ���� "������" ������
	//����������� ������������ ����� "��� ���������"
	if (TypePosEnd!=TypePosStart) {return;}
	if (RS485.Buffer[TypePosStart]!='R') {return;}//���� ��������� - �� �������� �������
	//����������� ������������ ����� "�����"
	for (i = AddressPosStart; i <= AddressPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//���� ������ - �� �������� ������
	//����������� ������������ ����� "����������"
	if (TransactionPosStart!=TransactionPosEnd) {return;}//���� ������ ����� "����������" �� ����� ������ �������, �� ����������� ���������� �������
	if (RS485.Buffer[TransactionPosStart]!=RS485.Transaction+0x30) {return;}//���� ����� ���������� �� ������������� �������������
	//����������� ������������ ����� "����������"
	unsigned char StrInstruction[8]={'G','e','t','V','a','l','u','e'};
	unsigned char InstructionLength=8; //������ ���������� � ������� ������������ ���������
	if ((InstructionPosEnd-InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = InstructionPosStart; i <= InstructionPosEnd; ++i) {if (RS485.Buffer[i]!=StrInstruction[i-InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ����� "������"
	if (DataPosStart>DataPosEnd)  {return;}//���� ���� "������" ������
		//������ �������� ������
	unsigned int DelimiterData1Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData2Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData3Pos=0; //������� �������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData4Pos=0; //������� ���������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData5Pos=0; //������� ������ ����������� (�������) � ����� "������"
	unsigned int DelimiterData6Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData7Pos=0; //������� �������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData8Pos=0; //������� �������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData9Pos=0; //������� �������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData10Pos=0; //������� �������� ����������� (�������) � ����� "������"
	//������������ ������� ������� � ����� "������"
	for (i = DataPosStart; i <= DataPosEnd; ++i)
		{
		if ((RS485.Buffer[i]==',')&(DelimiterData9Pos!=0)&(DelimiterData10Pos==0)) {DelimiterData10Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((RS485.Buffer[i]==',')&(DelimiterData8Pos!=0)&(DelimiterData9Pos==0)) {DelimiterData9Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((RS485.Buffer[i]==',')&(DelimiterData7Pos!=0)&(DelimiterData8Pos==0)) {DelimiterData8Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((RS485.Buffer[i]==',')&(DelimiterData6Pos!=0)&(DelimiterData7Pos==0)) {DelimiterData7Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((RS485.Buffer[i]==',')&(DelimiterData5Pos!=0)&(DelimiterData6Pos==0)) {DelimiterData6Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((RS485.Buffer[i]==',')&(DelimiterData4Pos!=0)&(DelimiterData5Pos==0)) {DelimiterData5Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((RS485.Buffer[i]==',')&(DelimiterData3Pos!=0)&(DelimiterData4Pos==0)) {DelimiterData4Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((RS485.Buffer[i]==',')&(DelimiterData2Pos!=0)&(DelimiterData3Pos==0)) {DelimiterData3Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos!=0)&(DelimiterData2Pos==0)) {DelimiterData2Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //���� � ����� "������" ���������� ������ �������
		}
	//�������� ���������� ��������� ����� "������"
	if (DelimiterData10Pos==0) {return;} //���� � ����� "������" ������ 10 �������
		//�������� ������� ��������� ����� "������"
	//ChID
	unsigned int InChId=0;
	if (DelimiterData1Pos==DelimiterData2Pos) {return;} //���� ������ ������� ������
	for (i = DelimiterData1Pos+1; i < DelimiterData2Pos; ++i)
		{InChId*=10; InChId+=RS485.Buffer[i]-0x30;}
	if (InChId!=ChId) {return;}
	//Value
	float Znak=1;
	float InValue=0;
	float M=10;
	if (DelimiterData4Pos==DelimiterData3Pos) {return;} //������� ������
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
	//�������� �� OutOfRange
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
	if (DelimiterData5Pos==DelimiterData4Pos) {return;} //������� ������
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
	if (DelimiterData5Pos==DelimiterData6Pos) {return;} //������� ������
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
//////////////////////////////           ������� ��� ������ � GSM �������            ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////           ������� ������������� GSM ������                 ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Init(void)
	{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//������������ USART GSM
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
	//������������ USART GSM
	USART_InitStructure.USART_BaudRate = 9600;                                       // ��������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      // ����� ������ 1����/8���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                           // 1 ����-���
	USART_InitStructure.USART_Parity = USART_Parity_No;                              // ��� �������� ��������
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  // ��������� ����� � ��������
	USART_Init(USART_GSM, &USART_InitStructure);
	USART_Cmd(USART_GSM, ENABLE);
	//������ ������ ������ �� GSM ������
	TIM_TimeBaseInitTypeDef Timer_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM_ReciveGSM,ENABLE); //������������ ������� �����������������
	TIM_TimeBaseStructInit(&Timer_InitStructure); //����������� ���� ��������� ���������� �����������
	Timer_InitStructure.TIM_Prescaler = 8; //������������ ������������
	Timer_InitStructure.TIM_Period = 100; //������������ ������ ������������ ������� 100���
	TIM_TimeBaseInit(Timer_ReciveGSM, &Timer_InitStructure); //������������� �������
	TIM_ITConfig(Timer_ReciveGSM, TIM_IT_Update, ENABLE); //����������� ������ ��� ��������� ���������� �� ���������� (������������)
	TIM_Cmd(Timer_ReciveGSM, ENABLE); //����������� ������
	NVIC_SetPriority(Timer_ReciveGSM_IRQn,PrioritetReciveGSM); //��������������� ��������� ����������
	NVIC_EnableIRQ(Timer_ReciveGSM_IRQn); //��������� ��������������� ����������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////           �������, ���������� �� ���������� ������� ��������� ����� �� GSM ������           //////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TIM5_IRQHandler(void)
	{
	if (USART_GetFlagStatus(USART_GSM,USART_FLAG_RXNE) != RESET) //����������� ������� ����� � �������� ������ GSM
		{
		GSM.UsartInByte=USART_ReceiveData(USART_GSM); //���������� ���� �� ������ Usart
		GSM.UsartRxNE=0xff; //��������������� ���� ������� � ������ �����
		USART_SendData(USART_BT,GSM.UsartInByte);
		}
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); //��������� ��� ����������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           ����� ������� �� GSM ������                           //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char GSM_Recive_Char(void)
	{
	GSM.UsartRxNE=0x00; //������������ ����
	return GSM.UsartInByte;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           �������� ������� � GSM ������                         //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_Char(unsigned char Byte)
	{
	Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //����������� ������� �������� ������� ����� �� Usart
	while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //��������� ��������� �������� �����
	USART_SendData(USART_GSM, Byte); //���������� ������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////             �������� ������ �� GSM.Buffer � GSM ������                          //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_String(void)
	{
	unsigned int i; //�������
	for (i = 0; i < strlen(GSM.Buffer); ++i)
		{
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //����������� ������� �������� ������� ����� �� Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //��������� ��������� �������� �����
		USART_SendData(USART_GSM,  GSM.Buffer[i]); //���������� ������
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////           �������� AT ������� �� GSM.Buffer � GSM ������                        //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_AT(void)
	{
	unsigned int i; //�������
	for (i = 0; i < strlen(GSM.Buffer); ++i)
		{
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //����������� ������� �������� ������� ����� �� Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //��������� ��������� �������� �����
		USART_SendData(USART_GSM, GSM.Buffer[i]); //���������� ������
		}
	GSM_Send_Char(0x0d);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                       ������� ��������� ������ ������� GSM                      //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Get_Signal_Quality(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="+csq:"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	//������������ ������ �� ��������� ������ �������
	strcpy(GSM.Buffer,"at+csq"); GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	//����������� ���������
	GSM.SQ_RSSI=0; //������� �������
	while ((Byte!=',')&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //����������� ����
			if ((Byte >= '0') && (Byte <= '9')) {GSM.SQ_RSSI*=10; GSM.SQ_RSSI+=Byte-0x30;}  //���� ������ - �����
			}
		}
	GSM.SQ_BER=0; //������� ������
	while ((Byte!=0x0d)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if (Byte >= '0' && Byte <= '9') {GSM.SQ_BER*=10; GSM.SQ_BER+=Byte-0x30;} //���� ������ - �����
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                ������� ��������� ������� ����������� � ���� GSM                 //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Get_Network_Registration_Status(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="+creg:"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	//������������ ������ �� ��������� ������� ����������� � ����
	strcpy(GSM.Buffer,"at+creg?"); GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	//����������� ���������
	GSM.REG_MODE=0; //����� ����������� � ���� ���������
	while ((Byte!=',')&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //����������� ����
			if ((Byte >= '0') && (Byte <= '9')) {GSM.REG_MODE*=10; GSM.REG_MODE+=Byte-0x30;}  //���� ������ - �����
			}
		}
	GSM.REG_STAT=0; //������ ����������� � ����
	while ((Byte!=0x0d)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if (Byte >= '0' && Byte <= '9') {GSM.REG_STAT*=10; GSM.REG_STAT+=Byte-0x30;} //���� ������ - �����
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                      ������� ��������� ����� ��������� GSM                      //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Get_Name_Operator(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="+cops:"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	unsigned char i=0; //�������
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	for (i = 0; i < 16; ++i) {GSM.Name_Operator[i]=' ';} //��������� ��� ���������
	//������������ ������ �� ��������� ����� ���������
	strcpy(GSM.Buffer,"at+cops?"); GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	//����������� ��� ���������
	i=0;
	while ((Byte!=0x22)&&(Timeouts.TimeoutGSM_AT>0)) //������������ ����� ������ �������
		{
		if (GSM.UsartRxNE != RESET) {Byte=GSM_Recive_Char();}//����������� ������
		if (Timeouts.TimeoutGSM_AT<=0) {return;}
		}
	Byte=0x00;
	while ((Byte!=0x22)&&(Timeouts.TimeoutGSM_AT>0)&&(i<16))  //����������� ��� ��������� �� 2-� �������
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if (Byte!=0x22) {GSM.Name_Operator[i]=Byte; i++;}

			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                          ������� ��������� IMEI GSM ������                      //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Get_IMEI(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	unsigned char i=0; //�������
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	for (i = 0; i < 16; ++i) {GSM.IMEI[i]=' ';} //��������� IMEI
	//������������ ������ �� ��������� IMEI
	strcpy(GSM.Buffer,"at+cgsn"); GSM_Send_AT(); //�������� AT �������
	//����������� IMEI
	i=0;
	while ((Timeouts.TimeoutGSM_AT>0)&&(i<16))  //����������� IMEI
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if (Byte >= '0' && Byte <= '9') {GSM.IMEI[i]=Byte; i++;} //���� ������ - �����
			}
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                            ������� �������� ������ GPRS                         //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_GPRS_Close(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="ok"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	//������������ ������ �� ��������� ������� ����������� � ����
	strcpy(GSM.Buffer,"at#gprs=0"); GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                             ������� ������� PDP ���������                       //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_GPRS_Set_PDP(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="ok"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	//������������ ������ �� ��������� ������� ����������� � ����
	strcpy(GSM.Buffer,"at+cgdcont=1,");
	strcat(GSM.Buffer,"\"IP\",");
	strcat(GSM.Buffer,GSM.GPRS_Settings);
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                              ������� �������� ������ GPRS                       //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_GPRS_Open(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="ok"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	//������������ ������ �� �������� ������ GPRS
	strcpy(GSM.Buffer,"at#sgact=1,1,");
	strcat(GSM.Buffer,GSM.GPRS_User_Pass);
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////         ������� ��������� �������� �������� ������ �� FTP �������               //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_TO(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="ok"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	//������������ ������ �� ��������� �������� 50 ������
	strcpy(GSM.Buffer,"AT#FTPTO=500");
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                        ������� IP ������ ������� �� DNS                         //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_DNS_Get_IP(unsigned int Timeout_ms)
	{
	char Temp_FTP_Settings[256]; //��������� FTP ��������� -  "server:port","username","password",mode
	unsigned char StartServerName=0;
	unsigned char EndServerName=0;
	char Temp_Server_Name[256]; //server
	unsigned char EndTempServerName; //
	unsigned char DNSFlag=0; //0-IP, 1-DNS
	unsigned int i=0; //�������
	unsigned int j=0;
	//����������� ��������� ����� ������� � ������
	for (i = 0; i < 255; ++i)
		{
		if ((StartServerName!=0)&&(EndServerName==0)&&((GSM.FTP_Settings[i]==0x22)||(GSM.FTP_Settings[i]==0x3a)))	{EndServerName=i-1;}
		if ((StartServerName==0)&&(GSM.FTP_Settings[i]==0x22))	{StartServerName=i+1;}
		}
	//������������ ����� ������ ������ �������
	if (EndServerName<StartServerName) {return;}
	for (i = StartServerName; i <= EndServerName; ++i)
		{
		if ((GSM.FTP_Settings[i]<0x2E)||(GSM.FTP_Settings[i]>0x39)||(GSM.FTP_Settings[i]==0x2F)) {DNSFlag=0xff;}
		Temp_Server_Name[j]=GSM.FTP_Settings[i];
		j++;
		}
	Temp_Server_Name[j]='\0'; //����������� ����������� ������
	if (DNSFlag==0) {return;} //���� � ������ ������ IP �����, �� ���������� �������� �� ���������
	//������������ ������ �� ��������� IP �� DNS
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]=","; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	strcpy(GSM.Buffer,"at#qdns=");
	strcat(GSM.Buffer,Temp_Server_Name);
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	//����������� IP �����
	while ((Byte!=0x22)&&(Timeouts.TimeoutGSM_AT>0))  //������������ ����� ������ �������
		{
		if (GSM.UsartRxNE != RESET) {Byte=GSM_Recive_Char();}//����������� ������
		if (Timeouts.TimeoutGSM_AT<=0) {return;}
		}
	i=0;
	Byte=0x00;
	while ((Byte!=0x22)&&(Timeouts.TimeoutGSM_AT>0)&&(i<256))  //����������� IP �� 2-� �������
		{
		if (GSM.UsartRxNE != RESET)
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte!=0x2F)&&(Byte<0x3A)&&(Byte>0x2D))
				{
				Temp_Server_Name[i]=Byte;
				EndTempServerName=i;
				i++;
				}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0)
		{GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	else
		{return;}
	//�������� ��������� � ������ �������� FTPSettings
	Temp_FTP_Settings[0]=0x22; //������ �������
	for (i = 0; i <= EndTempServerName; ++i) {Temp_FTP_Settings[i+1]=Temp_Server_Name[i];} //IP�����
	i=EndTempServerName+2;
	j=EndServerName+1;
	while ((i<256)&&(j<256)) {Temp_FTP_Settings[i]=GSM.FTP_Settings[j]; i++; j++;} //������������ ������
	for (i = 0; i <256 ; ++i) {GSM.FTP_Settings[i]=Temp_FTP_Settings[i];} //�� ��������� ������ ������ ����������� � ����������
	}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                  ������� �������� ���������� � FTP ��������                     //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Open(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="ok"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	//������������ ������ �� ��������� ���������� � FTP ��������
	strcpy(GSM.Buffer,"at#ftpopen=");
	strcat(GSM.Buffer,GSM.FTP_Settings);
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                  ������� �������� ���������� � FTP ��������                     //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Close(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="ok"; //�������� �����, ������������ � ������ ����� ������������
	char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	//������������ ������ �� ��������� ���������� � FTP ��������
	strcpy(GSM.Buffer,"at#ftpclose");
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////              ������� �������� ��������� ���� �������� �� FTP                  //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Type(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="ok"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	//������������ ������ �� ��������� ���� �������� ������ �� FTP (BIN)
	strcpy(GSM.Buffer,"at#ftptype=0");
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                              ������� �������� ����� FTP                         //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Open_File(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="ok"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	//������������ ������ ������� �����
	if (GSM.FTP_Append)
		{
		strcpy(GSM.Buffer,"at#ftpapp=");
		}
	else
		{
		strcpy(GSM.Buffer,"at#ftpput=");
		}
	strcat(GSM.Buffer,GSM.FTP_File_Name);
	strcat(GSM.Buffer,",1"); //��������� �����
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                              ������� �������� ����� FTP                         //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Close_File(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
		char ReplySTR[]="> "; //�������� �����, ������������ � ������ ����� ������������
		unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
		unsigned char ReplySTRIndex=0;
		Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
		GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
		//������������ ������ ������� �����
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
		//����� ��������� ����� � ������ GSM ������
		while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
			{
			if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
				{
				Byte=GSM_Recive_Char(); //����������� ������
				if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
				if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
				}
			}
		if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
		if (GSM.ATErrorFlag!=0x00) {return;}
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //����������� ������� �������� ������� ����� �� Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //��������� ��������� �������� �����
		USART_SendData(USART_GSM,  0x0d); //���������� ������
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //����������� ������� �������� ������� ����� �� Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //��������� ��������� �������� �����
		USART_SendData(USART_GSM,  0x0a); //���������� ������
		GSM_Send_Char(0x0d);
		strcpy(ReplySTR,"ok"); //�������� �����, ������������ � ������ ����� ������������
		ReplySTRLen=strlen(ReplySTR); //������ ������
		ReplySTRIndex=0;
		//����� ��������� ����� � ������ GSM ������
		while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
			{
			if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
				{
				Byte=GSM_Recive_Char(); //����������� ������
				if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
				if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
				}
			}
		if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                       ������� �������� ������ �� FTP ������                    //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Send_string(unsigned int Timeout_ms)
	{
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="> "; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	unsigned int i; //�������
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
	//������������ ������ ������� �����
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
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	if (GSM.ATErrorFlag!=0x00) {return;}
	for (i = 0; i < strlen(GSM.FTP_Out_String); ++i)
		{
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //����������� ������� �������� ������� ����� �� Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //��������� ��������� �������� �����
		USART_SendData(USART_GSM,  GSM.FTP_Out_String[i]); //���������� ������
		}
	GSM_Send_Char(0x0d);
	GSM_Send_Char(0x0a);
	strcpy(ReplySTR,"ok"); //�������� �����, ������������ � ������ ����� ������������
	ReplySTRLen=strlen(ReplySTR); //������ ������
	ReplySTRIndex=0;
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>0) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           ������� ����������� � ���� GSM                        //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Connect(void)
	{
	Delay_us(5000000); //�������� 5�
	GSM.Connected=0x00;
	GSM.REG_STAT=0x00;
	GSM.ATErrorFlag=0xff;
	//�������� ���������� � �����
	GSM.Attempt=30; //30 �������
	while ((GSM.REG_STAT!=1)&&(GSM.Attempt!=0))
		{
		Delay_us(2000000); //�������� 2�
		GSM_Get_Network_Registration_Status(2000); //������ ������� ���������� � �����
		Timeouts.TimeoutGSM_AT=0;
		if (GSM.ATErrorFlag!=0x00) {GSM.REG_STAT=0x00;}
		GSM.Attempt--;
		}
	if (GSM.REG_STAT==1) {GSM.Connected=0xff;} //���� ��������� ����������� � ����
	Delay_us(5000000);
	//��������� ������ �������
	GSM.Attempt=3; //3 �������
	GSM.ATErrorFlag=0xff;
	while ((GSM.Attempt!=0)&&(GSM.ATErrorFlag!=0x00))
		{
		Delay_us(1000000); //�������� 1�
		GSM_Get_Signal_Quality(2000); //������ ������ �������
		Timeouts.TimeoutGSM_AT=0;
		GSM.Attempt--;
		}
	//��������� ����� ���������
	GSM.Attempt=3; //3 �������
	GSM.ATErrorFlag=0xff;
	while ((GSM.Attempt!=0)&&(GSM.ATErrorFlag!=0x00))
		{
		Delay_us(1000000); //�������� 1�
		GSM_Get_Name_Operator(2000); //������ ����� ���������
		Timeouts.TimeoutGSM_AT=0;
		GSM.Attempt--;
		}
	//��������� IMEI
	Delay_us(1000000); //�������� 1�
	GSM_Get_IMEI(2000);
	GSM.Attempt=3; //3 �������
	GSM.ATErrorFlag=0xff;
	while ((GSM.Attempt!=0)&&(GSM.ATErrorFlag!=0x00))
		{
		Delay_us(1000000); //�������� 1�
		GSM_Get_Signal_Quality(2000); //������ ������ �������
		Timeouts.TimeoutGSM_AT=0;
		GSM.Attempt--;
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           ������� ����������� � GPRS                        //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_GPRS_Connect(void)
	{
	Delay_us(5000000); //�������� 5�
	GSM.GPRS_Connected=0x00;
	//�������� GPRS ������
	GSM.Attempt=3; //3 ��������
	while ((GSM.GPRS_Connected==0x00)&&(GSM.Attempt!=0))
		{
		Delay_us(2000000); //�������� 2�
		GSM_GPRS_Close(5000); //������� ������ GPRS
		Delay_us(2000000); //�������� 2�
		GSM_GPRS_Set_PDP(5000); //��������� PDP
		if (GSM.ATErrorFlag==0x00)
			{
			Delay_us(2000000); //�������� 2�
			GSM_GPRS_Open(60000); //������� ������ GPRS
			if (GSM.ATErrorFlag==0x00) {GSM.GPRS_Connected=0xff;}
			}
		GSM.Attempt--;
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           ������� ����������� � FTP �������                     //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_FTP_Connect(void)
	{
	Delay_us(5000000); //�������� 5�
	GSM.FTP_Connected=0x00;
	//�������� GPRS ������
	GSM.Attempt=3; //3 �������
	while ((GSM.FTP_Connected==0x00)&&(GSM.Attempt!=0))
		{
		Delay_us(2000000); //�������� 2�
		GSM_FTP_TO(5000); //������ ������� 50 �
		Delay_us(2000000); //�������� 2�
		GSM_DNS_Get_IP(10000); //���������� IP
		Delay_us(2000000); //�������� 2�
		GSM_FTP_Open(60000); //����������� � FTP ��������
		if (GSM.ATErrorFlag==0x00)
			{
			Delay_us(2000000); //�������� 2�
			GSM_FTP_Type(5000); //������ ��� FTP ��������� ASCII
			if (GSM.ATErrorFlag==0x00)
				{
				Delay_us(2000000); //�������� 2�
				GSM_FTP_Open_File(10000);
				if (GSM.ATErrorFlag==0x00) {GSM.FTP_Connected=0xff;}
				}
			}
		GSM.Attempt--;
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                           ������� �������� ������ �� FTP ������                 //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_Data_FTP(void)
	{

	FTPInProgress=0xff; //��������������� ���� �������� �������� ������

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
	//������������� GSM � Bluetooth ������
	GPIO_ResetBits(Power_RS485_Port, Power_RS485_Pin); //����������� ������� �� ����� RS485
	Delay_us(2000000); //�������� 2 �
	//��������������� ���������� � ���������� �������� ������
	GSM_Connect(); //��������� ����������� � ���� GSM
	GSM_GPRS_Connect(); //��������� ����������� � ���� GPRS
	if (GSM.GPRS_Connected)
		{
		GSM_FTP_Connect();
		if (GSM.FTP_Connected)
			{
			FTPOKFlag=0xff;//��������������� ���� �������� �������� ������ �� ftp
				//����������� ������ � ������� ������� GSM
			//�����8 (SQ_RSSI)
			TempResult.ChID=DeviceSerial*100+8;
			TempResult.Value=GSM.SQ_RSSI;
			TempResult.Variation=GSM.SQ_RSSI;
			TempResult.Temperature=MeasSystem.Temperature;
			Save_Record(TempResult);
			//�����9 (TimeFTPSession)
			TempResult.ChID=DeviceSerial*100+9;
			TempResult.Value=Timeouts.TimeFTPSession/1000000; //�������
			TempResult.Variation=GSM.SQ_BER;
			TempResult.Temperature=MeasSystem.Temperature;
			Save_Record(TempResult);
				//�������� ���������� ��������
			for (i = 0x00; i < 0x3000; ++i)
				{
				//���� �������� ������ ������ �� ��������� �������� ������
				if (Timeouts.ForceRestart<60000000)
					{
					ClearFTPFlags(); //������� ������ �������� ������ �� FTP
					}

				TempResult=Read_Record(i);
				if ((TempResult.FlagMeas!=0xff)&(TempResult.FlagFTP==0xff))
					{
					//����
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
					//���������� ������
					GSM.FTP_Out_String[pos]='\0';pos++;
					//�������� ������
					GSM.Attempt=3; GSM.ATErrorFlag=0xff;
					while ((GSM.ATErrorFlag!=0x00)&&(GSM.Attempt!=0))
						{
						Delay_us(200000); //�������� 200 ��
						GSM_FTP_Send_string(3000); //�������� ������ (������� 3 �������)
						GSM.Attempt--;
						}
					}
				}
			//����������� ����
			GSM.Attempt=3; GSM.ATErrorFlag=0xff;
			while ((GSM.ATErrorFlag!=0x00)&&(GSM.Attempt!=0))
				{
				Delay_us(200000); //�������� 200 ��
				GSM_FTP_Close_File(10000); //�������� ����� (������� 10 ������)
				GSM.Attempt--;
				}
			//����������� ����������
			Delay_us(1000000); //�������� 1000 ��
			GSM_FTP_Close(10000); //�������� ���������� (������� 10 ������)
			Delay_us(1000000); //�������� 1000 ��
			}
		}
	//�������� ���������� �������� ������
	if (FTPOKFlag!=0) //���� ������ ������� ��������
		{
		ClearFTPFlags(); //������� ������ �������� ������ �� FTP
		}

	FTPInProgress=0x00; //������������ ���� �������� �������� ������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                          ������� �������� SMS ��������� �� ������ GSM.OutSMS          ////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_SMS(unsigned int Timeout_ms)
	{
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="ok"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	unsigned int i;
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
		//���������� ��������� ������ ��������� (AT+CMGF=1)
	strcpy(GSM.Buffer,"AT+CMGF=1");
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>50))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	Delay_us(200000); //�������� 200��
	if (Timeouts.TimeoutGSM_AT<51) {return;}
	//�������� ������ ��������
	for (i = 0; i < 11; ++i)
		{
		if (GSM.SMSNumber[i]<'+') {return;} //���� ������ �� �����
		if (GSM.SMSNumber[i]>'9') {return;} //���� ������ �� �����
		}
	//������� �� �������� ���������
	strcpy(ReplySTR,"> ");
	strcpy(GSM.Buffer,"AT+CMGS=");
	strcat(GSM.Buffer,GSM.SMSNumber); //�������� ����� ��������
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT<51) {return;}
	Delay_us(200000); //�������� 200��
		//�������� ���������
	strcpy(ReplySTR,"ok");
	for (i = 0; i < strlen(GSM.OutSMS); ++i)
		{
		Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //����������� ������� �������� ������� ����� �� Usart
		while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //��������� ��������� �������� �����
		USART_SendData(USART_GSM,  GSM.OutSMS[i]); //���������� ������
		Delay_us(20000); //�������� 20 ��
		}
	Delay_us(200000); //�������� 200��
	//������� ������������ ������� (Ctrl+Z)
	Timeouts.TimeoutGSM_Char=Send_Char_Timeout;  //����������� ������� �������� ������� ����� �� Usart
	while ((USART_GetFlagStatus(USART_GSM, USART_FLAG_TC) == RESET)&&(Timeouts.TimeoutGSM_Char>0)) {}; //��������� ��������� �������� �����
	USART_SendData(USART_GSM,  0x1A); //���������� ������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>0))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	if (Timeouts.TimeoutGSM_AT>50) {GSM.ATErrorFlag=0x00;} //���� �� �������� ������� ���������� AT �������, �� ������������ ���� ������ AT �������
	}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////                          ������� �������� SMS ��������� � ������ �������� �����                  ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_Alarm_SMS(void)
	{
	//�������� ���
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
	GSM.Attempt=2; //2 �������
	GSM.ATErrorFlag=0xff;
	while ((GSM.Attempt>0)&&(GSM.ATErrorFlag!=0))
		{
		GSM_Send_SMS(30000);
		GSM.Attempt--;
		}
	}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////                       ������� �������� ������ �� FTP ������ � ������ �������� �����              ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Send_Alarm_FTP(void)
	{
	TMemoryResult TempResult;
	unsigned char FTPOKFlag=0;
	unsigned int i;
	signed int TempVal;
	unsigned char pos;
	Start_Led(100);//������� ������� ������� ����������
	//��������� ������� ������� �������
	RTC_Get_Date_Time();
	TempResult.YY=DateTime.YY;
	TempResult.MM=DateTime.MM;
	TempResult.DD=DateTime.DD;
	TempResult.hour=DateTime.hour;
	TempResult.min=DateTime.min;
	TempResult.sec=DateTime.sec;
	Meas_System_GetValue(); //���������� ��������� ���� ���������� �������
	//�����10(.Bat1Level)
	TempResult.ChID=DeviceSerial*100+10;
	TempResult.Value=MeasSystem.Bat1Level;
	TempResult.Variation=MeasSystem.Bat1Level;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//�����11 //Bat2Level
	TempResult.ChID=DeviceSerial*100+11;
	TempResult.Value=MeasSystem.Bat2Level;
	TempResult.Variation=MeasSystem.Bat2Level;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//�������� ������ � �����12 ����� ������(��������� �����-1, ����� ���������-0)
	TempResult.ChID=DeviceSerial*100+12;
	TempResult.Value=1;
	TempResult.Variation=1;
	TempResult.Temperature=1;
	Save_Record(TempResult);
	//��������������� ���������� � ���������� �������� ������
	GSM_Connect(); //��������� ����������� � ���� GSM
	GSM_GPRS_Connect(); //��������� ����������� � ���� GPRS
	if (GSM.GPRS_Connected)
		{
		GSM_FTP_Connect();
		if (GSM.FTP_Connected)
			{
			FTPOKFlag=0xff;//��������������� ���� �������� �������� ������ �� ftp
				//����������� ������ � ������� ������� GSM
			//�����8 (SQ_RSSI)
			TempResult.ChID=DeviceSerial*100+8;
			TempResult.Value=GSM.SQ_RSSI;
			TempResult.Variation=GSM.SQ_RSSI;
			TempResult.Temperature=MeasSystem.Temperature;
			Save_Record(TempResult);
			//�����9 (TimeFTPSession)
			TempResult.ChID=DeviceSerial*100+9;
			TempResult.Value=Timeouts.TimeFTPSession/1000000; //�������
			TempResult.Variation=GSM.SQ_BER;
			TempResult.Temperature=MeasSystem.Temperature;
			Save_Record(TempResult);
			//�������� ���������� ��������
			for (i = 0x00; i < 0x3000; ++i)
				{
				TempResult=Read_Record(i);
				if ((TempResult.FlagMeas!=0xff)&(TempResult.FlagFTP==0xff))
					{
					//����
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
					//���������� ������
					GSM.FTP_Out_String[pos]='\0';pos++;
					//�������� ������
					GSM.Attempt=3; GSM.ATErrorFlag=0xff;
					while ((GSM.ATErrorFlag!=0x00)&&(GSM.Attempt!=0))
						{
						Delay_us(200000); //�������� 200 ��
						GSM_FTP_Send_string(3000); //�������� ������ (������� 3 �������)
						GSM.Attempt--;
						}
					}
				}
			//����������� ����
			GSM.Attempt=3; GSM.ATErrorFlag=0xff;
			while ((GSM.ATErrorFlag!=0x00)&&(GSM.Attempt!=0))
				{
				Delay_us(200000); //�������� 200 ��
				GSM_FTP_Close_File(10000); //�������� ����� (������� 10 ������)
				GSM.Attempt--;
				}
			//����������� ����������
			Delay_us(1000000); //�������� 1000 ��
			GSM_FTP_Close(10000); //�������� ���������� (������� 10 ������)
			Delay_us(1000000); //�������� 1000 ��
			}
		}
	//�������� ���������� �������� ������
	if (FTPOKFlag!=0) //���� ������ ������� ��������
		{
		ClearFTPFlags(); //������� ������ �������� ������ �� FTP
		}
	}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////      ������� ���������� ���������� ���������� SetCycleSettings �� ��� ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_Execution_SetCycleSettings(void)
	{
	unsigned char StrInstruction[16]={'S','e','t','C','y','c','l','e','S','e','t','t','i','n','g','s'};//���������� ��������� �������������� �����
	unsigned char InstructionLength=16; //������ ���������� � ������� ������������ ���������
	unsigned int CurrentDelimiterNumber=0; //������� ����� �����������
	unsigned int TypePosStart=0; //������ ����� "��� ���������"
	unsigned int TypePosEnd=0; //����� ����� "��� ���������"
	unsigned int AddressPosStart=0; //������ ����� "�����"
	unsigned int AddressPosEnd=0; //����� ����� "�����"
	unsigned int TransactionPosStart=0; //������ ����� "����������"
	unsigned int TransactionPosEnd=0; //����� ����� "����������"
	unsigned int InstructionPosStart=0; //������ ����� "����������"
	unsigned int InstructionPosEnd=0; //����� ����� "����������"
	unsigned int DataPosStart=0; //������ ����� "������"
	unsigned int DataPosEnd=0; //����� ����� "������"
	unsigned int DelimiterData1Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int DelimiterData2Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int i; //�������
	//�������� ������������ ���������
	//��������� ����������� ������ ���������
	for (i = 0; i < RS485.Buffer_Len; ++i)
		{
		if (RS485.Buffer[i]=='/') //���� ��������� �����������
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //���� ��������� 0-� �����������
				TypePosStart=i+1; //������������ ������ ����� "��� ���������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 1: //���� ��������� 1-� �����������
				TypePosEnd=i-1; //������������ ����� ����� "��� ���������"
				AddressPosStart=i+1; //������������ ������ ����� "�����"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 2: //���� ��������� 2-� �����������
				AddressPosEnd=i-1; //������������ ����� ����� "�����"
				TransactionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 3: //���� ��������� 3-� �����������
				TransactionPosEnd=i-1; //������������ ����� ����� "����������"
				InstructionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 4: //���� ��������� 4-� �����������
				InstructionPosEnd=i-1; //������������ ����� ����� "����������"
				DataPosStart=i+1; //������������ ������ ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 5: //���� ���������5-� �����������
				DataPosEnd=i-1; //������������ ����� ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				}
			}
		}
	if (CurrentDelimiterNumber!=6) {return;} //����������� ���������� ������ � ���������
	//����������� ������ ������
	if (TypePosStart>TypePosEnd) {return;}//���� ���� "���" ������
	if (AddressPosStart>AddressPosEnd) {return;}//���� ���� "�����" ������
	if (TransactionPosStart>TransactionPosEnd) {return;}//���� ���� "����������" ������
	if (InstructionPosStart>InstructionPosEnd) {return;}//���� ���� "����������" ������
	if (DataPosStart>DataPosEnd) {return;}//���� ���� "������" ������
	//����������� ������������ ����� "��� ���������"
	if (TypePosEnd!=TypePosStart) {return;}
	if (RS485.Buffer[TypePosStart]!='Q') {return;}//���� ��������� - �� �������� ��������
	//����������� ������������ ����� "�����"
	for (i = AddressPosStart; i <= AddressPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//���� ������ - �� �������� ������
	unsigned int TempCode; //�������� ��� �������
	//�������������� � ����� ��� �����
	TempCode=0;
	unsigned int M=1; //��������� �������� �������
	for (i = AddressPosEnd; i >= AddressPosStart; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {TempCode=TempCode+M*(RS485.Buffer[i]-48);M=M*10;}}
	//����������� ��� �������
	if (TempCode!=GSM.SMSCode) {return;}//��� ������� �� ������
	//����������� ������������ ����� "����������"
	if ((InstructionPosEnd-InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = InstructionPosStart; i <= InstructionPosEnd; ++i) {if (RS485.Buffer[i]!=StrInstruction[i-InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ����� "������"
	if (DataPosStart>DataPosEnd)  {return;}//���� ���� "������" ������
	//������������ ������� ������� � ����� "������"
	for (i = DataPosStart; i <= DataPosEnd; ++i)
		{
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos!=0)&(DelimiterData2Pos==0)) {DelimiterData2Pos=i;}//���� � ����� "������" ���������� ������ �������
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //���� � ����� "������" ���������� ������ �������
		}
	//�������� ���������� ��������� ����� "������"
	if (DelimiterData2Pos==0) {return;} //���� � ����� "������" ������ 2 �������
	//�������� ������� �������� ����� "������" (CyclePeriod) �� ������������
	for (i = DataPosStart; i < DelimiterData1Pos; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//���� ������ - �� �������� ������
	//�������������� � ����� ��� �����
	CyclePeriod=0;
	M=1; //��������� �������� �������
	for (i = DelimiterData1Pos-1; i >= DataPosStart; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {CyclePeriod=CyclePeriod+M*(RS485.Buffer[i]-48);M=M*10;}}
		//�������� ������� �������� ����� "������" (CycleStart) �� ������������
	for (i = DelimiterData1Pos+1; i < DelimiterData2Pos; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}} //���� ������ - �� �������� ������
	//�������������� � ����� ��� �����
	CycleStart=0;
	M=1; //��������� �������� �������
	for (i = DelimiterData2Pos-1; i >= DelimiterData1Pos+1; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {CycleStart=CycleStart+M*(RS485.Buffer[i]-48);M=M*10;}}
	//�������� �������� �������� ����� "������" (CycleSendDataPeriod) �� ������������
	for (i = DelimiterData2Pos+1; i <= DataPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}} //���� ������ - �� �������� ������
	//�������������� � ����� ��� �����
	CycleSendDataPeriod=0;
	M=1; //��������� �������� �������
	for (i = DataPosEnd; i >= DelimiterData2Pos+1; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {CycleSendDataPeriod=CycleSendDataPeriod+M*(RS485.Buffer[i]-48);M=M*10;}}
	//�������� ��������
	if (CycleSendDataPeriod>MaximumPeriodSendData) {return;} //���� ���������� ��������� �������� ������ ������ 255
	if (CyclePeriod>86400) {return;} //���� ������ ������ 24 �����
	if (CyclePeriod<MinimumPeriod) {return;} //���� ������ ������ 5 �����
	if (CycleStart>86399) {return;} //
	SetCycle_Settings(); //��������� �������� ������ ���������
	RTC_Set_Next_Cycle_Time(); //��������������� ����� ���������� �����
	//�������� ������
	Delay_us(5000000);
	for (i = 0; i <= RS485.Buffer_Len; ++i) {GSM.OutSMS[i]=RS485.Buffer[i];} //������������ ������
	GSM.OutSMS[RS485.Buffer_Len+1]='\0';
	GSM.OutSMS[2]='R';
	GSM_Send_SMS(10000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////      ������� ���������� ���������� ���������� SetCycleSettings �� ��� ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_Execution_SetSMSSettings(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','S','M','S','S','e','t','t','i','n','g','s'};//���������� ��������� �������������� �����
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned int CurrentDelimiterNumber=0; //������� ����� �����������
	unsigned int TypePosStart=0; //������ ����� "��� ���������"
	unsigned int TypePosEnd=0; //����� ����� "��� ���������"
	unsigned int AddressPosStart=0; //������ ����� "�����"
	unsigned int AddressPosEnd=0; //����� ����� "�����"
	unsigned int TransactionPosStart=0; //������ ����� "����������"
	unsigned int TransactionPosEnd=0; //����� ����� "����������"
	unsigned int InstructionPosStart=0; //������ ����� "����������"
	unsigned int InstructionPosEnd=0; //����� ����� "����������"
	unsigned int DataPosStart=0; //������ ����� "������"
	unsigned int DataPosEnd=0; //����� ����� "������"
	unsigned int DelimiterData1Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int i; //�������
	//�������� ������������ ���������
	//��������� ����������� ������ ���������
	for (i = 0; i < RS485.Buffer_Len; ++i)
		{
		if (RS485.Buffer[i]=='/') //���� ��������� �����������
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //���� ��������� 0-� �����������
				TypePosStart=i+1; //������������ ������ ����� "��� ���������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 1: //���� ��������� 1-� �����������
				TypePosEnd=i-1; //������������ ����� ����� "��� ���������"
				AddressPosStart=i+1; //������������ ������ ����� "�����"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 2: //���� ��������� 2-� �����������
				AddressPosEnd=i-1; //������������ ����� ����� "�����"
				TransactionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 3: //���� ��������� 3-� �����������
				TransactionPosEnd=i-1; //������������ ����� ����� "����������"
				InstructionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 4: //���� ��������� 4-� �����������
				InstructionPosEnd=i-1; //������������ ����� ����� "����������"
				DataPosStart=i+1; //������������ ������ ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 5: //���� ���������5-� �����������
				DataPosEnd=i-1; //������������ ����� ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				}
			}
		}
	if (CurrentDelimiterNumber!=6) {return;} //����������� ���������� ������ � ���������
	//����������� ������ ������
	if (TypePosStart>TypePosEnd) {return;}//���� ���� "���" ������
	if (AddressPosStart>AddressPosEnd) {return;}//���� ���� "�����" ������
	if (TransactionPosStart>TransactionPosEnd) {return;}//���� ���� "����������" ������
	if (InstructionPosStart>InstructionPosEnd) {return;}//���� ���� "����������" ������
	if (DataPosStart>DataPosEnd) {return;}//���� ���� "������" ������
	//����������� ������������ ����� "��� ���������"
	if (TypePosEnd!=TypePosStart) {return;}
	if (RS485.Buffer[TypePosStart]!='Q') {return;}//���� ��������� - �� �������� ��������
	//����������� ������������ ����� "�����"
	for (i = AddressPosStart; i <= AddressPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//���� ������ - �� �������� ������
	unsigned int TempCode; //�������� ��� �������
	//�������������� � ����� ��� �����
	TempCode=0;
	unsigned int M=1; //��������� �������� �������
	for (i = AddressPosEnd; i >= AddressPosStart; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {TempCode=TempCode+M*(RS485.Buffer[i]-48);M=M*10;}}
	//����������� ��� �������
	if (TempCode!=GSM.SMSCode) {return;}//��� ������� �� ������
	//����������� ������������ ����� "����������"
	if ((InstructionPosEnd-InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = InstructionPosStart; i <= InstructionPosEnd; ++i) {if (RS485.Buffer[i]!=StrInstruction[i-InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ����� "������"
	if (DataPosStart>DataPosEnd)  {return;}//���� ���� "������" ������
	//������������ ������� ������� � ����� "������"
	for (i = DataPosStart; i <= DataPosEnd; ++i)
		{
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //���� � ����� "������" ���������� ������ �������
		}
	//�������� ���������� ��������� ����� "������"
	if (DelimiterData1Pos==0) {return;} //���� � ����� "������" ������ 1 �������
	//�������� ������� �������� ����� �� ������������ (������)
	for (i = DataPosStart; i < DelimiterData1Pos; ++i)
		{
		if (RS485.Buffer[i]<'+') {return;} //���� ������ �� �����
		if (RS485.Buffer[i]>'9') {return;} //���� ������ �� �����
		}
	//�������� ������� �������� ����� "������" �� ������������
	for (i = DelimiterData1Pos+1; i <= DataPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}} //���� ������ - �� �������� ������
	//����������� ������
	for (i = 0; i < 12; ++i) {GSM.SMSNumber[i]=RS485.Buffer[DataPosStart+i];}
	GSM.SMSNumber[12]='\0';
	//��� �������
	GSM.SMSCode=Bluetooth_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1,DataPosEnd);
	//���������� ��������
	Save_SMS_Settings(); //������ ����������� � SRAM
	//�������� ������
	Delay_us(5000000);
	for (i = 0; i <= RS485.Buffer_Len; ++i) {GSM.OutSMS[i]=RS485.Buffer[i];} //������������ ������
	GSM.OutSMS[RS485.Buffer_Len+1]='\0';
	GSM.OutSMS[2]='R';
	GSM_Send_SMS(10000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ SMS, � ���������� ������ GSM.FTP_Settings        /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_HexChar_Buf_To_FTP_Settings(unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //�������
	unsigned char j=0; //�������
	unsigned char TempByte=0; //��������� ����
	unsigned char Byte1=0; //������ ��������(�������)
	unsigned char Byte2=0; //������ ��������(�������)
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
	GSM.FTP_Settings[j]='\0'; //����������� ����������� ������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////      ������� ���������� ���������� ���������� SetFTPSettings �� ��� ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_Execution_SetFTPSettings(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','F','T','P','S','e','t','t','i','n','g','s'};
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned int CurrentDelimiterNumber=0; //������� ����� �����������
	unsigned int TypePosStart=0; //������ ����� "��� ���������"
	unsigned int TypePosEnd=0; //����� ����� "��� ���������"
	unsigned int AddressPosStart=0; //������ ����� "�����"
	unsigned int AddressPosEnd=0; //����� ����� "�����"
	unsigned int TransactionPosStart=0; //������ ����� "����������"
	unsigned int TransactionPosEnd=0; //����� ����� "����������"
	unsigned int InstructionPosStart=0; //������ ����� "����������"
	unsigned int InstructionPosEnd=0; //����� ����� "����������"
	unsigned int DataPosStart=0; //������ ����� "������"
	unsigned int DataPosEnd=0; //����� ����� "������"
	unsigned int i; //�������
	//�������� ������������ ���������
	//��������� ����������� ������ ���������
	for (i = 0; i < RS485.Buffer_Len; ++i)
		{
		if (RS485.Buffer[i]=='/') //���� ��������� �����������
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //���� ��������� 0-� �����������
				TypePosStart=i+1; //������������ ������ ����� "��� ���������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 1: //���� ��������� 1-� �����������
				TypePosEnd=i-1; //������������ ����� ����� "��� ���������"
				AddressPosStart=i+1; //������������ ������ ����� "�����"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 2: //���� ��������� 2-� �����������
				AddressPosEnd=i-1; //������������ ����� ����� "�����"
				TransactionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 3: //���� ��������� 3-� �����������
				TransactionPosEnd=i-1; //������������ ����� ����� "����������"
				InstructionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 4: //���� ��������� 4-� �����������
				InstructionPosEnd=i-1; //������������ ����� ����� "����������"
				DataPosStart=i+1; //������������ ������ ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 5: //���� ���������5-� �����������
				DataPosEnd=i-1; //������������ ����� ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				}
			}
		}
	if (CurrentDelimiterNumber!=6) {return;} //����������� ���������� ������ � ���������
	//����������� ������ ������
	if (TypePosStart>TypePosEnd) {return;}//���� ���� "���" ������
	if (AddressPosStart>AddressPosEnd) {return;}//���� ���� "�����" ������
	if (TransactionPosStart>TransactionPosEnd) {return;}//���� ���� "����������" ������
	if (InstructionPosStart>InstructionPosEnd) {return;}//���� ���� "����������" ������
	if (DataPosStart>DataPosEnd) {return;}//���� ���� "������" ������
	//����������� ������������ ����� "��� ���������"
	if (TypePosEnd!=TypePosStart) {return;}
	if (RS485.Buffer[TypePosStart]!='Q') {return;}//���� ��������� - �� �������� ��������
	//����������� ������������ ����� "�����"
	for (i = AddressPosStart; i <= AddressPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//���� ������ - �� �������� ������
	unsigned int TempCode; //�������� ��� �������
	//�������������� � ����� ��� �����
	TempCode=0;
	unsigned int M=1; //��������� �������� �������
	for (i = AddressPosEnd; i >= AddressPosStart; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {TempCode=TempCode+M*(RS485.Buffer[i]-48);M=M*10;}}
	//����������� ��� �������
	if (TempCode!=GSM.SMSCode) {return;}//��� ������� �� ������
	//����������� ������������ ����� "����������"
	if ((InstructionPosEnd-InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = InstructionPosStart; i <= InstructionPosEnd; ++i) {if (RS485.Buffer[i]!=StrInstruction[i-InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ����� "������"
	if (DataPosStart>DataPosEnd)  {return;}//���� ���� "������" ������
	//�������� ������ �� ������������
	for (i = DataPosStart; i <= DataPosEnd; ++i)
		{
		if (RS485.Buffer[i]<'0') {return;} //���� ������ �� �����
		if (RS485.Buffer[i]>'f') {return;} //���� ������ �� �����
		if ((RS485.Buffer[i]>'9')&&(RS485.Buffer[i]<'A')) {return;} //���� ������ �� �����
		if ((RS485.Buffer[i]>'F')&&(RS485.Buffer[i]<'a')) {return;} //���� ������ �� �����
		}
	//������ ������������� � ���������� ���
	GSM_SMS_HexChar_Buf_To_FTP_Settings(DataPosStart,DataPosEnd);
	Save_FTP_Settings(); //������ ����������� � SRAM
	//�������� ������
	Delay_us(5000000);
	for (i = 0; i <= RS485.Buffer_Len; ++i) {GSM.OutSMS[i]=RS485.Buffer[i];} //������������ ������
	GSM.OutSMS[RS485.Buffer_Len+1]='\0';
	GSM.OutSMS[2]='R';
	GSM_Send_SMS(10000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ������� ���������� �������������� ����� ������ � ������� HEX, ����������� � ������ SMS, � ���������� ������ GSM.GPRS_User_Pass ////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_HexChar_Buf_To_FTP_FileName(unsigned int StartPos, unsigned int EndPos)
	{
	unsigned int i; //�������
	unsigned char j=0; //�������
	unsigned char TempByte=0; //��������� ����
	unsigned char Byte1=0; //������ ��������(�������)
	unsigned char Byte2=0; //������ ��������(�������)
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
	GSM.FTP_File_Name[j]='\0'; //����������� ����������� ������
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////    ������� ���������� �������������� ����� ������, ����������� � ������ SMS, � ����� ����� ��� �����       /////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int GSM_SMS_Char_To_Unsigned_Int_Buf(unsigned int StartPos, unsigned int EndPos) //StartPos - ������� ������� �������, EndPos - ������� ���������� �������
	{
	unsigned int i; //�������
	unsigned int Out=0; //�������� ����������
	unsigned int M=1; //��������� �������� �������
	for (i = EndPos; i >= StartPos; --i)
		{
		if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) //���� ������ - �����
			{
			Out=Out+M*(RS485.Buffer[i]-48);
			M=M*10; //������������� ��������� �������� �������;
			}
		}
	return Out;
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////        ������� ���������� ���������� ���������� SetFTPFileName  �� ���        ////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_SMS_Execution_SetFTPFileName(void)
	{
	unsigned char StrInstruction[14]={'S','e','t','F','T','P','F','i','l','e','N','a','m','e'};
	unsigned char InstructionLength=14; //������ ���������� � ������� ������������ ���������
	unsigned int CurrentDelimiterNumber=0; //������� ����� �����������
	unsigned int TypePosStart=0; //������ ����� "��� ���������"
	unsigned int TypePosEnd=0; //����� ����� "��� ���������"
	unsigned int AddressPosStart=0; //������ ����� "�����"
	unsigned int AddressPosEnd=0; //����� ����� "�����"
	unsigned int TransactionPosStart=0; //������ ����� "����������"
	unsigned int TransactionPosEnd=0; //����� ����� "����������"
	unsigned int InstructionPosStart=0; //������ ����� "����������"
	unsigned int InstructionPosEnd=0; //����� ����� "����������"
	unsigned int DataPosStart=0; //������ ����� "������"
	unsigned int DataPosEnd=0; //����� ����� "������"
	unsigned int DelimiterData1Pos=0; //������� ������� ����������� (�������) � ����� "������"
	unsigned int i; //�������
	//�������� ������������ ���������
	//��������� ����������� ������ ���������
	for (i = 0; i < RS485.Buffer_Len; ++i)
		{
		if (RS485.Buffer[i]=='/') //���� ��������� �����������
			{
			switch(CurrentDelimiterNumber)
				{
				case 0: //���� ��������� 0-� �����������
				TypePosStart=i+1; //������������ ������ ����� "��� ���������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 1: //���� ��������� 1-� �����������
				TypePosEnd=i-1; //������������ ����� ����� "��� ���������"
				AddressPosStart=i+1; //������������ ������ ����� "�����"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 2: //���� ��������� 2-� �����������
				AddressPosEnd=i-1; //������������ ����� ����� "�����"
				TransactionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 3: //���� ��������� 3-� �����������
				TransactionPosEnd=i-1; //������������ ����� ����� "����������"
				InstructionPosStart=i+1; //������������ ������ ����� "����������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 4: //���� ��������� 4-� �����������
				InstructionPosEnd=i-1; //������������ ����� ����� "����������"
				DataPosStart=i+1; //������������ ������ ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				case 5: //���� ���������5-� �����������
				DataPosEnd=i-1; //������������ ����� ����� "������"
				CurrentDelimiterNumber++; //���������������� ������� ����� �����������
				break;
				}
			}
		}
	if (CurrentDelimiterNumber!=6) {return;} //����������� ���������� ������ � ���������
	//����������� ������ ������
	if (TypePosStart>TypePosEnd) {return;}//���� ���� "���" ������
	if (AddressPosStart>AddressPosEnd) {return;}//���� ���� "�����" ������
	if (TransactionPosStart>TransactionPosEnd) {return;}//���� ���� "����������" ������
	if (InstructionPosStart>InstructionPosEnd) {return;}//���� ���� "����������" ������
	if (DataPosStart>DataPosEnd) {return;}//���� ���� "������" ������
	//����������� ������������ ����� "��� ���������"
	if (TypePosEnd!=TypePosStart) {return;}
	if (RS485.Buffer[TypePosStart]!='Q') {return;}//���� ��������� - �� �������� ��������
	//����������� ������������ ����� "�����"
	for (i = AddressPosStart; i <= AddressPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}}//���� ������ - �� �������� ������
	unsigned int TempCode; //�������� ��� �������
	//�������������� � ����� ��� �����
	TempCode=0;
	unsigned int M=1; //��������� �������� �������
	for (i = AddressPosEnd; i >= AddressPosStart; --i)
		{if ((RS485.Buffer[i]>47)&(RS485.Buffer[i]<58)) {TempCode=TempCode+M*(RS485.Buffer[i]-48);M=M*10;}}
	//����������� ��� �������
	if (TempCode!=GSM.SMSCode) {return;}//��� ������� �� ������
	//����������� ������������ ����� "����������"
	if ((InstructionPosEnd-InstructionPosStart+1)!=InstructionLength) {return;} //���� ������ ���������� �� ������������� ����������, �� ����������� ����������  ����������
	for (i = InstructionPosStart; i <= InstructionPosEnd; ++i) {if (RS485.Buffer[i]!=StrInstruction[i-InstructionPosStart]) {return;}} //���� ���������� �������������� ��������, �� ����������� ����������  ����������
	//�������� ����� "������"
	if (DataPosStart>DataPosEnd)  {return;}//���� ���� "������" ������
	//������������ ������� ������� � ����� "������"
	for (i = DataPosStart; i <= DataPosEnd; ++i)
		{
		if ((RS485.Buffer[i]==',')&(DelimiterData1Pos==0)) {DelimiterData1Pos=i;} //���� � ����� "������" ���������� ������ �������
		}
	//�������� ���������� ��������� ����� "������"
	if (DelimiterData1Pos==0) {return;} //���� � ����� "������" ������ 1 �������
	//�������� ������� ��������� ����� "������"
	if (DataPosStart==DelimiterData1Pos) {return;} //���� ������ ������� ������
	//�������� ������� �������� ����� �� ������������ (������)
	for (i = DataPosStart; i < DelimiterData1Pos; ++i)
		{
		if (RS485.Buffer[i]<'0') {return;} //���� ������ �� �����
		if (RS485.Buffer[i]>'f') {return;} //���� ������ �� �����
		if ((RS485.Buffer[i]>'9')&&(RS485.Buffer[i]<'A')) {return;} //���� ������ �� �����
		if ((RS485.Buffer[i]>'F')&&(RS485.Buffer[i]<'a')) {return;} //���� ������ �� �����
		}
	//�������� ������� �������� ����� "������" �� ������������
	for (i = DelimiterData1Pos+1; i <= DataPosEnd; ++i)
		{if ((RS485.Buffer[i]<47)|(RS485.Buffer[i]>58)) {return;}} //���� ������ - �� �������� ������
	//������ ������������� � ���������� ���
	GSM_SMS_HexChar_Buf_To_FTP_FileName(DataPosStart,DelimiterData1Pos-1);
	GSM.FTP_Append=GSM_SMS_Char_To_Unsigned_Int_Buf(DelimiterData1Pos+1, DataPosEnd); //������ �������� ������
	Save_FTP_FileName(); //������ ����������� � SRAM
	//�������� ������
	Delay_us(5000000);
	for (i = 0; i <= RS485.Buffer_Len; ++i) {GSM.OutSMS[i]=RS485.Buffer[i];} //������������ ������
	GSM.OutSMS[RS485.Buffer_Len+1]='\0';
	GSM.OutSMS[2]='R';
	GSM_Send_SMS(10000);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                        ������ ���� SMS � ���������� ������                      //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Read_ALL_SMS(unsigned int Timeout_ms)
	{
	Delay_us(1000000); //�������� 1�
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="ok"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
		//���������� ��������� ������ ��������� (AT+CMGF=1)
	strcpy(GSM.Buffer,"AT+CMGF=1");
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>50))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	/////////////////////////////////
	//������ ��������� (������������ ��������� RS485)
	/////////////////////////////////
	RS485.Message_Complete_flag=0x00; //���� ��������� ������ ���������
	RS485.Start_Collect_Flag=0x00; //���� ������ �������� ������ ���������
	RS485.Buffer_Pos=0x00; //������� ������� � ������
	RS485.Previous_Byte=0x00; //������� ����
	RS485.Current_Byte=0x00; //���������� ����
	Delay_us(1000000); //�������� 1�
	if (Timeouts.TimeoutGSM_AT<51) {return;}
		//��������� ��� ��������� (AT+CMGL=ALL)
	strcpy(GSM.Buffer,"AT+CMGL=ALL");
	GSM_Send_AT(); //�������� AT �������
	//����������� ������� ������ ���������
	while (RS485.Message_Complete_flag==0x00)
		{
		if (Timeouts.TimeoutGSM_AT<51) {return;} //���� �������� ������� ��������
		if (RS485.Buffer_Pos>1023) {return;}//���� ������������ �����
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			RS485.Previous_Byte=RS485.Current_Byte; //������������ ���������� �������� ����
			RS485.Current_Byte=GSM_Recive_Char(); //����������� ������
			if (RS485.Start_Collect_Flag!=0) //���� ���� ������� ������ ���������
				{
				RS485.Buffer[RS485.Buffer_Pos]=RS485.Current_Byte; //��������� ������� ���� � ����� ���������
				RS485.Buffer_Pos++; //���������������� ������� ��������� ������� � ������ ���������
				RS485.Buffer_Len=RS485.Buffer_Pos; //����������� ���������� ������ � ������ ���������
				if ((RS485.Previous_Byte=='/')&&(RS485.Current_Byte=='%')) {RS485.Message_Complete_flag=0xff;} //���� ��������� ������ ����� ���������, �� ��������������� ������� ������ ���������
				}
			if ((RS485.Previous_Byte=='%')&&(RS485.Current_Byte=='/')) //���� ��������� ������ ������ ���������
				{
				RS485.Start_Collect_Flag=0xff; //��������������� ���� ������� �������� ������� ���������
				RS485.Buffer[0]=RS485.Previous_Byte; //��������� ���������� ���� � ����� ���������
				RS485.Buffer[1]=RS485.Current_Byte; //��������� ������� ���� � ����� ���������
				RS485.Buffer_Pos=2; //�������� ������� ������� � ������ ���������
				}
			}
		}
	if (RS485.Message_Complete_flag!=0x00) //���� ��������� �������
		{
		GSM_SMS_Execution_SetCycleSettings(); //���������� ���������� SetCycleSettings
 		GSM_SMS_Execution_SetFTPSettings();  //���������� ���������� SetCycleSettings
 		GSM_SMS_Execution_SetFTPFileName(); //���������� ���������� SetFTPFileName
 		GSM_SMS_Execution_SetSMSSettings(); //���������� ���������� SetCycleSettings
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////                                    �������� ���� SMS                            //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GSM_Delete_All_SMS(unsigned int Timeout_ms)
	{
	Delay_us(1000000); //�������� 1�
	Timeouts.TimeoutGSM_AT=Timeout_ms*1000; //��������������� ������� ���������� AT �������
	unsigned char Byte=0x00; //����, ����������� �� USART
	char ReplySTR[]="ok"; //�������� �����, ������������ � ������ ����� ������������
	unsigned char ReplySTRLen=strlen(ReplySTR); //������ ������
	unsigned char ReplySTRIndex=0;
	GSM.ATErrorFlag=0xff; //��������������� ���� ������ AT �������
		//���������� ��������� ������ ��������� (AT+CMGF=1)
	strcpy(GSM.Buffer,"AT+CMGF=1");
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>50))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	Delay_us(1000000); //�������� 1�
	if (Timeouts.TimeoutGSM_AT<51) {return;}
		//������� ��� ��������� (AT+CMGD=1,4)
	strcpy(GSM.Buffer,"AT+CMGD=1,4");
	GSM_Send_AT(); //�������� AT �������
	//����� ��������� ����� � ������ GSM ������
	while ((ReplySTRIndex<ReplySTRLen)&&(Timeouts.TimeoutGSM_AT>50))
		{
		if (GSM.UsartRxNE != RESET) //���� � ������ USART ���� ����
			{
			Byte=GSM_Recive_Char(); //����������� ������
			if ((Byte >= 'A') && (Byte <= 'Z')) {Byte+=0x20;} //������� ������������� � ��������
			if (ReplySTR[ReplySTRIndex]==Byte) {ReplySTRIndex++;} else {ReplySTRIndex=0;}
			}
		}
	Delay_us(1000000); //�������� 1�
	}




















////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////             ������� ������� ��������� ����������                        ///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////        ������� ������������� ������� ��������� ����������        ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Meas_System_Init(void)//������� ������������� ������������� �������
	{
	//������������� ������ �����������
	GPIO_InitTypeDef GPIO_InitStructure;
	//���� ��� - ������� ������ ������������ 1
	GPIO_InitStructure.GPIO_Pin = Bat1_Level_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Bat1_Level_Port, &GPIO_InitStructure);
	//���� ��� - ������� ������ ������������ 2
	GPIO_InitStructure.GPIO_Pin = Bat2_Level_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Bat2_Level_Port, &GPIO_InitStructure);
	//���� ��� - ��������� (����������� ����������)
	GPIO_InitStructure.GPIO_Pin = Termistor_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Termistor_Port, &GPIO_InitStructure);
	//���� ��� - ������������� (�������)
	GPIO_InitStructure.GPIO_Pin = Power_Potenciometr_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Power_Potenciometr_Port, &GPIO_InitStructure);
	//���� ��� - ������������ (�����1)
	GPIO_InitStructure.GPIO_Pin = CH1_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH1_Port, &GPIO_InitStructure);
	//���� ��� - ������������ (�����2)
	GPIO_InitStructure.GPIO_Pin = CH2_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH2_Port, &GPIO_InitStructure);
	//���� ��� - ������������ (�����3)
	GPIO_InitStructure.GPIO_Pin = CH3_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH3_Port, &GPIO_InitStructure);
	//���� ��� - ������������ (�����4)
	GPIO_InitStructure.GPIO_Pin = CH4_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH4_Port, &GPIO_InitStructure);
	//���� ��� - ������������ (�����5)
	GPIO_InitStructure.GPIO_Pin = CH5_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH5_Port, &GPIO_InitStructure);
	//���� ��� - ������������ (�����6)
	GPIO_InitStructure.GPIO_Pin = CH6_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(CH6_Port, &GPIO_InitStructure);
	//������������� ��� �����������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC_MeasSystem, ENABLE);
	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_Init(ADC_MeasSystem, &ADC_InitStructure);
	//���������� ���
	ADC_Cmd(ADC_MeasSystem, ENABLE);
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////         ������� ��������� ���������� �� ��������������           ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Meas_System_GetValue(void)
	{
	unsigned int i; //�������
	unsigned int ADC_Data_Sum=0; //��������� ����������
						//��������� ���������� �� ����������
	ADC_Data_Sum=0;
	ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_15, 1, ADC_SampleTime_480Cycles); //��������� ������ ���
	for (i = 0; i < 1024; i++)
		{
		Timeouts.TimeoutADC=ADC_Timeout; //����������� ������� �������� ��������� ���
		ADC_SoftwareStartConv(ADC_MeasSystem); //������ ���������
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //�������� ��������� ���������
		ADC_Data_Sum=ADC_Data_Sum + ADC_GetConversionValue(ADC_MeasSystem); //������ ����������� ���������
		}
	//������ �����������
	MeasSystem.Temperature=(float) (ADC_Data_Sum/1024); //������� �������� ���
	MeasSystem.Temperature=(float) (MeasSystem.Temperature*MeasSystem.TermistorConstRes/(4096-MeasSystem.Temperature)); //����������� �������� ������������� ����������
	MeasSystem.Temperature=(float) (MeasSystem.Temperature-MeasSystem.TermistorRes)/3.85;
						//��������� ���������� Bat1
	ADC_Data_Sum=0;
	ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_6, 1, ADC_SampleTime_480Cycles); //��������� ������ ���
	for (i = 0; i < 1024; i++)
		{
		Timeouts.TimeoutADC=ADC_Timeout; //����������� ������� �������� ��������� ���
		ADC_SoftwareStartConv(ADC_MeasSystem); //������ ���������
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //�������� ��������� ���������
		ADC_Data_Sum=ADC_Data_Sum + ADC_GetConversionValue(ADC_MeasSystem); //������ ����������� ���������
		}
	//������ ����������
	MeasSystem.Bat1Level=(float) (ADC_Data_Sum/1024); //������� �������� ���
	MeasSystem.Bat1Level=(float) (MeasSystem.Bat1Level*15.3/4096); //�������� ����������, �
						//��������� ���������� Bat2
	ADC_Data_Sum=0;
	ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_7, 1, ADC_SampleTime_480Cycles); //��������� ������ ���
	for (i = 0; i < 1024; i++)
		{
		Timeouts.TimeoutADC=ADC_Timeout; //����������� ������� �������� ��������� ���
		ADC_SoftwareStartConv(ADC_MeasSystem); //������ ���������
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //�������� ��������� ���������
		ADC_Data_Sum=ADC_Data_Sum + ADC_GetConversionValue(ADC_MeasSystem); //������ ����������� ���������
		}
	//������ ����������
	MeasSystem.Bat2Level=(float) (ADC_Data_Sum/1024); //������� �������� ���
	MeasSystem.Bat2Level=(float) (MeasSystem.Bat2Level*15.3/4096); //�������� ����������, �
	//��������� ���������� �� ��������������
	MeasSystem.CHPowerVoltage=0;
	MeasSystem.CHPowerRes=0;
	MeasSystem.CH1Val=0;
	MeasSystem.CH2Val=0;
	MeasSystem.CH3Val=0;
	MeasSystem.CH4Val=0;
	MeasSystem.CH5Val=0;
	MeasSystem.CH6Val=0;
	float TempVoltage=0; //���������� ��� ���������� �������� ���������� �� ��������������
	float TempRes=0; //���������� ��� ���������� �������� ������������� ��������������
	for (i = 0; i < 1024; i++)
		{
		//��������� CHPowerVoltage � CHPowerRes
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_5, 1, ADC_SampleTime_480Cycles); //��������� ������ ���
		Timeouts.TimeoutADC=ADC_Timeout; //����������� ������� �������� ��������� ���
		ADC_SoftwareStartConv(ADC_MeasSystem); //������ ���������
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //�������� ��������� ���������
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //������ ����������� ���������
		TempVoltage=(float) (ADC_Data_Sum*3300/4096); //���������� �� ��������������, ��
		TempRes=(float) (TempVoltage*MeasSystem.PowerConstRes/(3300-TempVoltage)); //������������� �����������
		MeasSystem.CHPowerVoltage+=TempVoltage;
		MeasSystem.CHPowerRes+=TempRes;
		//��������� CH1
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_0, 1, ADC_SampleTime_480Cycles); //��������� ������ ���
		Timeouts.TimeoutADC=ADC_Timeout; //����������� ������� �������� ��������� ���
		ADC_SoftwareStartConv(ADC_MeasSystem); //������ ���������
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //�������� ��������� ���������
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //������ ����������� ���������
		MeasSystem.CH1Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //������ ���������� ���������
		//��������� CH2
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_13, 1, ADC_SampleTime_480Cycles); //��������� ������ ���
		Timeouts.TimeoutADC=ADC_Timeout; //����������� ������� �������� ��������� ���
		ADC_SoftwareStartConv(ADC_MeasSystem); //������ ���������
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //�������� ��������� ���������
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //������ ����������� ���������
		MeasSystem.CH2Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //������ ���������� ���������
		//��������� CH3
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_12, 1, ADC_SampleTime_480Cycles); //��������� ������ ���
		Timeouts.TimeoutADC=ADC_Timeout; //����������� ������� �������� ��������� ���
		ADC_SoftwareStartConv(ADC_MeasSystem); //������ ���������
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //�������� ��������� ���������
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //������ ����������� ���������
		MeasSystem.CH3Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //������ ���������� ���������
		//��������� CH4
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_11, 1, ADC_SampleTime_480Cycles); //��������� ������ ���
		Timeouts.TimeoutADC=ADC_Timeout; //����������� ������� �������� ��������� ���
		ADC_SoftwareStartConv(ADC_MeasSystem); //������ ���������
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //�������� ��������� ���������
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //������ ����������� ���������
		MeasSystem.CH4Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //������ ���������� ���������
		//��������� CH5
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_10, 1, ADC_SampleTime_480Cycles); //��������� ������ ���
		Timeouts.TimeoutADC=ADC_Timeout; //����������� ������� �������� ��������� ���
		ADC_SoftwareStartConv(ADC_MeasSystem); //������ ���������
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //�������� ��������� ���������
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //������ ����������� ���������
		MeasSystem.CH5Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //������ ���������� ���������
		//��������� CH6
		ADC_RegularChannelConfig(ADC_MeasSystem, ADC_Channel_1, 1, ADC_SampleTime_480Cycles); //��������� ������ ���
		Timeouts.TimeoutADC=ADC_Timeout; //����������� ������� �������� ��������� ���
		ADC_SoftwareStartConv(ADC_MeasSystem); //������ ���������
		while((ADC_GetFlagStatus(ADC_MeasSystem, ADC_FLAG_EOC) == RESET)&&(Timeouts.TimeoutADC>0)); //�������� ��������� ���������
		ADC_Data_Sum=ADC_GetConversionValue(ADC_MeasSystem); //������ ����������� ���������
		MeasSystem.CH6Val+=(float) ADC_Data_Sum*3300000/(4096*TempVoltage); //������ ���������� ���������
		}
	//������ ������� ��������
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
///////////////////////                      ������� ��������� �����                   ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MeasCycle(void)
	{
	TMemoryResult TempResult;
	unsigned char i; //�������
	TempResult.YY=DateTime.YY;
	TempResult.MM=DateTime.MM;
	TempResult.DD=DateTime.DD;
	TempResult.hour=DateTime.hour;
	TempResult.min=DateTime.min;
	TempResult.sec=DateTime.sec;
	Meas_System_GetValue(); //���������� ��������� ���� ���������� �������
	//��������� ������������ �������
			//���� ������������� �������������
			#if Enable_Potenciometr==1
	//�����1
	TempResult.ChID=DeviceSerial*100+1;
	TempResult.Value=MeasSystem.CH1Val;
	TempResult.Variation=MeasSystem.CH1Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//�����2
	TempResult.ChID=DeviceSerial*100+2;
	TempResult.Value=MeasSystem.CH2Val;
	TempResult.Variation=MeasSystem.CH2Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//�����3
	TempResult.ChID=DeviceSerial*100+3;
	TempResult.Value=MeasSystem.CH3Val;
	TempResult.Variation=MeasSystem.CH3Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//�����4
	TempResult.ChID=DeviceSerial*100+4;
	TempResult.Value=MeasSystem.CH4Val;
	TempResult.Variation=MeasSystem.CH4Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//�����5
	TempResult.ChID=DeviceSerial*100+5;
	TempResult.Value=MeasSystem.CH5Val;
	TempResult.Variation=MeasSystem.CH5Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//�����6
	TempResult.ChID=DeviceSerial*100+6;
	TempResult.Value=MeasSystem.CH6Val;
	TempResult.Variation=MeasSystem.CH6Val;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//�����7 (CHPowerRes)
	TempResult.ChID=DeviceSerial*100+7;
	TempResult.Value=MeasSystem.CHPowerRes;
	TempResult.Variation=MeasSystem.CHPowerRes;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
			#endif
	//�����10(.Bat1Level)
	TempResult.ChID=DeviceSerial*100+10;
	TempResult.Value=MeasSystem.Bat1Level;
	TempResult.Variation=MeasSystem.Bat1Level;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//�������� ���������� �������� ������������
	if ((TempResult.Value>1)&(TempResult.Value<11))
		{
		GPIO_ResetBits(Power_Port, Power_Pin); Delay_us(3000000); //����������� ������� ����������
		return; //����� �� �������
		}
	//�����11 //Bat2Level
	TempResult.ChID=DeviceSerial*100+11;
	TempResult.Value=MeasSystem.Bat2Level;
	TempResult.Variation=MeasSystem.Bat2Level;
	TempResult.Temperature=MeasSystem.Temperature;
	Save_Record(TempResult);
	//�����12 ����� ������(��������� �����-1, ����� ���������-0)
	if (DeviceMode==DeviceModeCommand) //���� � ��������� ������
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
	//��������� �� ����� RS485
	GPIO_SetBits(Power_RS485_Port, Power_RS485_Pin); //���������� ������� ����� RS485
	Delay_us(3000000); //�������� 3 �
	//� ������ � ������������ ��������� ��������� ���������� ������ ������� ���������
	for (i = 0; i < 64; ++i)
		{
		if (ChIDList[i]!=0)
			{
			//������ 4 ��������� ����������� ������� �� 10 ������ (��� ���������� �������� �� ����� � ���������� ������� ��������� ����������)
			if ((i==4)||(i==8)||(i==12)||(i==16)||(i==20)||(i==24)||(i==28)||(i==32)||(i==36)||(i==40)||(i==44)||(i==48)||(i==52)||(i==56)||(i==60))
				{
				GPIO_ResetBits(Power_RS485_Port, Power_RS485_Pin); //����������� ������� ����� RS485
				Delay_us(18000000); //�������� 18 �
				GPIO_SetBits(Power_RS485_Port, Power_RS485_Pin); //���������� ������� ����� RS485
				Delay_us(2000000); //�������� 2 �
				}
			//��������� - 2 �������
			RS485.Attempt=2; RS485.CorrectResultFlag=0;
			while ((RS485.Attempt>0)&&(RS485.CorrectResultFlag==0))
				{
				RS485_GetValue_Sensor(10,ChIDList[i],6000); //������ �� ���������, ������� 6 ������
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
	GPIO_ResetBits(Power_RS485_Port, Power_RS485_Pin); //����������� ������� ����� RS485
	}























////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////                       �������� ���������                             ////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(void)
	{
	TMemoryResult TempResult;
	unsigned int i; //�������
	SystemInit(); // ��������� ������������
	__enable_irq(); //����������� ���������� ����������
	//������������ ������
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	Power_Init(); //������������� ������� �������
	GPIO_SetBits(Power_BT_GSM_Port, Power_BT_GSM_Pin); //���������� ������� Bluetooth � GSM �������
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	//������������� ���������
	Timeouts.ForceRestart=MaximumTimeForceRestart; //��������������� ����� �������������� ������������
	Main_Timer_Init(); //������������� ��������� �������
	Read_Parametr_Device(); //������ ���������� ����������
	//������������� ����� ��������� �������
	if (RTC_GetFlagStatus(RTC_FLAG_INITS)==RESET) //���� ���� �� ����������������
		{
		RTC__Init();
		//��������� �������� �������
		DateTime.DD=31;	DateTime.MM=12;	DateTime.YY=01; DateTime.hour=23; DateTime.min=57; DateTime.sec=57;
		RTC_Set_Date_Time();
		//��������� ������� ����������
		CycleTime.hour=23; CycleTime.min=58; CycleTime.sec=20;
		RTC_Set_Alarm_Time();
		//��������� ������ �������
		for (i = 0; i < 64; ++i) {ChIDList[i]=0;}
		Save_ChIDList(); //����������� � SRAM
		}
	RTC_Set_Next_Cycle_Time(); //���������� ����� ���������� ���������
				//������������� ���������
	//������������� ������������� �������
	Meas_System_Init();//������� ������������� ������������� �������
	RS485_Init(); //������������� RS485
	GPIO_ResetBits(Power_RS485_Port, Power_RS485_Pin); //����������� ������� ����� RS485
	//����� ���������� ������
	if (DeviceMode==DeviceModeCommand) //���� � ��������� ������
		{
		Delay_us(3000000); //�������� 3 �
		Bluetooth_Init();
		Test_Chanela_Blooetooth();
		GSM_Init();
		Delay_us(1000000); //�������� 1 �
		Timeouts.TimeoutIdle=Idle_Timeout; //��������������� ������� ������� ����������
		Delay_us(3000000); //�������� 1 �
		GSM_Send_Alarm_SMS(); //�������� SMS ��������� � ������ �������� �����
		while (Timeouts.TimeoutIdle>100) {};
		}
	else  //���� � ������ ���������
		{
		Start_Led(1000);//������� ������� ������� ����������
		MeasCycle(); //��������� �����
		//���� ���������� �������� ������
		if (DeviceMode==DeviceModeCycleFTP)
			{
			Timeouts.TimeFTPSession=0; //������������ ������� ������� FTP ������
			Start_Led(200);//������� ������� ������� ����������
			//Power_ON_BT_GSM(); //���������� ������� Bluetooth � GSM �������
			Delay_us(3000000); //�������� 3 �
			Bluetooth_Init();
			Test_Chanela_Blooetooth();
			GSM_Init();
			Delay_us(5000000); //�������� 5 �
			GSM_Send_Data_FTP();
			//����� � ������ ���� ���
			GSM_Read_ALL_SMS(5000);
			//�������� ���� ���
			GSM_Delete_All_SMS(5000);
			}
		}
	if (DeviceMode==DeviceModeCommand) {GSM_Send_Alarm_FTP();} //���� � ��������� ������ �� ������������ ��������� �� �������� �����
	RTC_Set_Next_Cycle_Time(); //���������� ����� ���������� ���������
	GPIO_ResetBits(Power_Port, Power_Pin); Delay_us(3000000); //����������� ������� ����������

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//���� ������� �� ��������� �� ������� �������� �����
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	unsigned int CounterSendAlarm=0;
	while (1)
		{
		Timeouts.ForceRestart=MaximumTimeForceRestart; //��������������� ����� �������������� ������������
		Start_Led(50);//������� ������� ������� ����������
		RTC_Set_Next_Cycle_Time(); //���������� ����� ���������� ���������
		GPIO_ResetBits(Power_Port, Power_Pin); //����������� ������� ����������
		Delay_us(3000000); //�������� 3�
		GPIO_ResetBits(Power_BT_GSM_Port, Power_BT_GSM_Pin); //����������� ������� Bluetooth � GSM �������
		//��������� ����� ��������� ��������
		Timeouts.AlarmSend=PeriodAlarmSend;	while (Timeouts.AlarmSend>200) {};
		Start_Led(200);//������� ������� ������� ����������
		GPIO_SetBits(Power_BT_GSM_Port, Power_BT_GSM_Pin); //���������� ������� Bluetooth � GSM �������
		Delay_us(3000000); //�������� 3 �
		//Bluetooth_Init();
		Delay_us(10000000); //�������� 10�
		if ((CounterSendAlarm%4)==0)
			{
			GSM_Send_Alarm_SMS(); //�������� SMS ��������� � ������ �������� �����
			}
		GSM_Send_Alarm_FTP(); //������������ ��������� �� �������� �����
		Delay_us(5000000); //�������� 5�
		CounterSendAlarm=CounterSendAlarm+1;
		}
	}

