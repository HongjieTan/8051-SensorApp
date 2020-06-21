#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "SensorApp.h"
#include "DebugTrace.h"
#include "Sensor.h"   //添加头文件

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

uint16 RxLen;        //串口接收数据长度
uint8 UartDataBuf[128]; //串口数据缓存区指针

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

  //配置串口并打开
  uartConfig.configured = TRUE;               //使能串口
  uartConfig.baudRate   = HAL_UART_BR_57600; //波特率115200
  uartConfig.flowControl = FALSE;             //关闭流控
  uartConfig.callBackFunc = SensorApp_rxCB;     //回调函数
  HalUARTOpen(HAL_UART_PORT_0, &uartConfig);  //打开串口
 
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
          //网关接收到消息时处理
          SensorApp_MessageMSGCB(MSGpkt);

          break;

        case ZDO_STATE_CHANGE:
          SensorApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          
          if ( (SensorApp_NwkState == DEV_ZB_COORD)
              || (SensorApp_NwkState == DEV_ROUTER)
              || (SensorApp_NwkState == DEV_END_DEVICE) )
          {
            // 加入网络成功，点亮LED2
            HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
#if !defined(ZDO_COORDINATOR) 
            if (SensorApp_NwkState == DEV_END_DEVICE)//终端
            { 
              // 如果是终端节点，启动定时器发送消息"Hello"
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
  
  if ( events & SensorApp_SEND_MSG_EVT )//消息发送事件
  {
#if !defined(ZDO_COORDINATOR)
    // 定时器到时，调用消息发送函数
    SerialApp_SendTemp();
#endif
    // 重置定时器
    osal_start_timerEx( SensorApp_TaskID,
                        SensorApp_SEND_MSG_EVT,
                        SensorApp_SEND_MSG_TIMEOUT );

    return (events ^ SensorApp_SEND_MSG_EVT);
  }
  if ( events & SensorApp_UART_RX_CB_EVT )//串口发送事件
  {
    HalUARTWrite(HAL_UART_PORT_0, &UartDataBuf[0], RxLen); //串口显示发送内容
    SensorApp_SendTheMessage(&UartDataBuf[0], RxLen);       //无线发送消息
    return (events ^ SensorApp_UART_RX_CB_EVT);
  }
  return 0;
}

void SensorApp_SendTheMessage( uint8 *buf, uint16 len )
{
  SensorApp_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;  //消息发送方式，广播
  SensorApp_DstAddr.endPoint = SensorApp_ENDPOINT;            //目标终端编号，由哪个终端处理消息
  SensorApp_DstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;//默认的广播地址，0XFFFF
  
  if( AF_DataRequest( &SensorApp_DstAddr, &SensorApp_epDesc,
                       SensorApp_CLUSTERID,
                       len,
                       buf,
                       &SensorApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // 发送成功后闪烁 LED1
     HalLedBlink( HAL_LED_1, 1, 50, 250 );
     osal_memset(buf, 0, len);  //清缓冲区
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
      // 接收成功后闪烁 LED1
      HalLedBlink( HAL_LED_1, 1, 50, 250 );
      
      HalUARTWrite(HAL_UART_PORT_0, &(pkt->cmd.Data[0]), pkt->cmd.DataLength ); //将收到的数据通过串口打印出来
      HalUARTWrite(HAL_UART_PORT_0, &vt, 1);  //空格
      
    break;
  }
}

static void SensorApp_rxCB(uint8 port, uint8 event)
{
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)))
  {
    RxLen = Hal_UART_RxBufLen(HAL_UART_PORT_0);  //接收缓冲区数据长度,字节为单位
    HalUARTRead( HAL_UART_PORT_0, &UartDataBuf[0], RxLen); //读接收缓冲区数据到内存databuf[3]~[len+3]
  
    osal_set_event(SensorApp_TaskID,SensorApp_UART_RX_CB_EVT);  //有串口数据时产生相应事件
  }
}

