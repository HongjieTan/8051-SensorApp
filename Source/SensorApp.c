#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "SensorApp.h"
#include "DebugTrace.h"
#include "Sensor.h"   //���ͷ�ļ�

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
const cId_t SensorApp_ClusterList[SensorApp_MAX_CLUSTERS] =
{
  SensorApp_CLUSTERID
};

const SimpleDescriptionFormat_t SensorApp_SimpleDesc =
{
  SensorApp_ENDPOINT,              //  int Endpoint;
  SensorApp_PROFID,                //  uint16 AppProfId[2];
  SensorApp_DEVICEID,              //  uint16 AppDeviceId[2];
  SensorApp_DEVICE_VERSION,        //  int   AppDevVer:4;
  SensorApp_FLAGS,                 //  int   AppFlags:4;
  SensorApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)SensorApp_ClusterList,  //  byte *pAppInClusterList;
  SensorApp_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)SensorApp_ClusterList   //  byte *pAppInClusterList;
};

endPointDesc_t SensorApp_epDesc;

byte SensorApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SensorApp_Init() is called.
devStates_t SensorApp_NwkState;

byte SensorApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SensorApp_DstAddr;

uint16 RxLen;        //���ڽ������ݳ���
uint8 UartDataBuf[128]; //�������ݻ�����ָ��

void SensorApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SensorApp_SendTheMessage( uint8 *buf, uint16 len );
void SerialApp_SendTemp(void);
static void SensorApp_rxCB(uint8 port,uint8 event);
void ChangeAddrFormat(uint8 *dest,uint8 *src, uint8 length);

void SensorApp_Init( byte task_id )
{
  halUARTCfg_t uartConfig;
  
  SensorApp_TaskID = task_id;
  SensorApp_NwkState = DEV_INIT;
  SensorApp_TransID = 0;

  SensorApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  SensorApp_DstAddr.endPoint = 0;
  SensorApp_DstAddr.addr.shortAddr = 0;

  SensorApp_epDesc.endPoint = SensorApp_ENDPOINT;
  SensorApp_epDesc.task_id = &SensorApp_TaskID;
  SensorApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SensorApp_SimpleDesc;
  SensorApp_epDesc.latencyReq = noLatencyReqs;

  
  afRegister( &SensorApp_epDesc );

  //���ô��ڲ���
  uartConfig.configured = TRUE;               //ʹ�ܴ���
  uartConfig.baudRate   = HAL_UART_BR_57600; //������115200
  uartConfig.flowControl = FALSE;             //�ر�����
  uartConfig.callBackFunc = SensorApp_rxCB;     //�ص�����
  HalUARTOpen(HAL_UART_PORT_0, &uartConfig);  //�򿪴���
 
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "SensorApp", HAL_LCD_LINE_1 );
#endif
}

UINT16 SensorApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;        // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SensorApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          //do nothing
          
          break;

        case KEY_CHANGE:
          // do nothing
          
          break;

        case AF_DATA_CONFIRM_CMD:
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          //���ؽ��յ���Ϣʱ����
          SensorApp_MessageMSGCB(MSGpkt);

          break;

        case ZDO_STATE_CHANGE:
          SensorApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          
          if ( (SensorApp_NwkState == DEV_ZB_COORD)
              || (SensorApp_NwkState == DEV_ROUTER)
              || (SensorApp_NwkState == DEV_END_DEVICE) )
          {
            // ��������ɹ�������LED2
            HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
#if !defined(ZDO_COORDINATOR) 
            if (SensorApp_NwkState == DEV_END_DEVICE)//�ն�
            { 
              // ������ն˽ڵ㣬������ʱ��������Ϣ"Hello"
              osal_start_timerEx( SensorApp_TaskID,
                                SensorApp_SEND_MSG_EVT,
                                SensorApp_SEND_MSG_TIMEOUT );
            }
#endif
          }
          break;

        default:
          break;
      }

      osal_msg_deallocate( (uint8 *)MSGpkt );

      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SensorApp_TaskID );
    }

    return (events ^ SYS_EVENT_MSG);
  }
  
  if ( events & SensorApp_SEND_MSG_EVT )//��Ϣ�����¼�
  {
#if !defined(ZDO_COORDINATOR)
    // ��ʱ����ʱ��������Ϣ���ͺ���
    SerialApp_SendTemp();
#endif
    // ���ö�ʱ��
    osal_start_timerEx( SensorApp_TaskID,
                        SensorApp_SEND_MSG_EVT,
                        SensorApp_SEND_MSG_TIMEOUT );

    return (events ^ SensorApp_SEND_MSG_EVT);
  }
  if ( events & SensorApp_UART_RX_CB_EVT )//���ڷ����¼�
  {
    HalUARTWrite(HAL_UART_PORT_0, &UartDataBuf[0], RxLen); //������ʾ��������
    SensorApp_SendTheMessage(&UartDataBuf[0], RxLen);       //���߷�����Ϣ
    return (events ^ SensorApp_UART_RX_CB_EVT);
  }
  return 0;
}

void SensorApp_SendTheMessage( uint8 *buf, uint16 len )
{
  SensorApp_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;  //��Ϣ���ͷ�ʽ���㲥
  SensorApp_DstAddr.endPoint = SensorApp_ENDPOINT;            //Ŀ���ն˱�ţ����ĸ��ն˴�����Ϣ
  SensorApp_DstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;//Ĭ�ϵĹ㲥��ַ��0XFFFF
  
  if( AF_DataRequest( &SensorApp_DstAddr, &SensorApp_epDesc,
                       SensorApp_CLUSTERID,
                       len,
                       buf,
                       &SensorApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // ���ͳɹ�����˸ LED1
     HalLedBlink( HAL_LED_1, 1, 50, 250 );
     osal_memset(buf, 0, len);  //�建����
  }
  else
  {
    // Error occurred in request to send.
  }
}

void SerialApp_SendTemp(void)
{
  char* tvalue;
  TEMPERATURE temperature;
  
  temperature.BUF.Head = '&';
  tvalue = ReadTemp();
  uint8  value = tvalue[1]*10+tvalue[2]-15;
  temperature.BUF.value[0]=value/10+'0';
  temperature.BUF.value[1]=value%10+ '0';
  temperature.BUF.Tail = 'C';
  
  SensorApp_SendTheMessage((uint8 *)&temperature, sizeof(temperature));
}

void SensorApp_MessageMSGCB(afIncomingMSGPacket_t *pkt)
{
  uint8 vt = ' ';
  
  switch(pkt->clusterId)
  {
    case SensorApp_CLUSTERID:
      // ���ճɹ�����˸ LED1
      HalLedBlink( HAL_LED_1, 1, 50, 250 );
      
      HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[0]), pkt->cmd.DataLength ); //���յ�������ͨ�����ڴ�ӡ����
      HalUARTWrite(HAL_UART_PORT_0, &vt, 1);  //�ո�
      
    break;
  }
}

static void SensorApp_rxCB(uint8 port, uint8 event)
{
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)))
  {
    RxLen = Hal_UART_RxBufLen(HAL_UART_PORT_0);  //���ջ��������ݳ���,�ֽ�Ϊ��λ
    HalUARTRead( HAL_UART_PORT_0, &UartDataBuf[0], RxLen); //�����ջ��������ݵ��ڴ�databuf[3]~[len+3]
  
    osal_set_event(SensorApp_TaskID,SensorApp_UART_RX_CB_EVT);  //�д�������ʱ������Ӧ�¼�
  }
}

