#include <string.h>

// SimpleLink includes
#include "simplelink.h"

// driverlib includes
#include "hw_memmap.h"
#include "hw_common_reg.h"


#include "hw_ints.h"
#include "hw_types.h"
#include "hw_uart.h"
#include "hw_wdt.h"
#include "rom.h"
#include "wdt.h"
#include "rom_map.h"
#include "pin.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "interrupt.h"
#include "udma.h"
#include "json.h"
#include "timer.h"
// common interface includes
#include "uart_if.h"
#include "udma_if.h"
#include "timer_if.h"
#include "WDT_if.h"
#include "uart.h"
#include "common.h"
#include "pinmux.h"
#include "gpio_if.h"
#include <math.h>
#include "i2c_if.h"
#include "smartconfig.h"
#include "adc.h"
// HTTP Client lib
#include <http/client/httpcli.h>
#include <http/client/common.h>

//pnarasim
#include "netcfg.h"

// JSON Parser
#include "jsmn.h"
//#define WATCHDOG
#define APPLICATION_VERSION             "1.1.1"
#define APP_NAME                        "The ThingsCloud Temperature"
#define WD_PERIOD_MS                    20000
#define MAP_SysCtlClockGet              80000000
#define MILLISECONDS_TO_TICKS(ms)       ((MAP_SysCtlClockGet / 1000) * (ms))


#define BAUD_RATE                       19200

#define NO_OF_SAMPLES 		128
unsigned long pulAdcSamples[4096];


#define SH_GPIO_3                       3

#define POST_REQUEST_URI_AC            "/devices/deviceData"
#define POST_REQUEST_URI_DC            "/devices/deviceData"

#define POST_HEADER                     "{"
#define POST_timestamp                           "\n\"dts\":"
#define POST_DEVICEID                           ",\n\"device_id\":201840,"
#define POST_SLAVE                              "\n\"slave_id\":1"


#define POST_HEADER1                    ",\n\"data\":"
#define POST_BEGIN                              "{"

#define POST_END                                "\n}"

#define POST_TAIL1                              "\n}"

#define LOGIN                           1
#define TIME                            2
#define POST                            3

#define Debug    1
#define Slave    2


#define PUT_REQUEST_URI                 "/put"
#define PUT_DATA                        "PUT request."

#define GET_REQUEST_URI                 "/api/timestamp"


#define HOST_PORT                       80
#define TIME_PORT                       80

#define PROXY_IP                         <proxy_ip>
#define PROXY_PORT                       <proxy_port>

#define READ_SIZE                       1450
#define MAX_BUFF_SIZE                   1460

#define TX_EN  MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,GPIO_PIN_6)
#define TX_DIS   MAP_GPIOPinWrite(GPIOA2_BASE,GPIO_PIN_6,0)

#define RX_DIS   MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_6,GPIO_PIN_6)
#define RX_EN   MAP_GPIOPinWrite(GPIOA1_BASE,GPIO_PIN_6,0)


// Application specific status/error codes
typedef enum{
/* Choosing this number to avoid overlap with host-driver's error codes */
DEVICE_NOT_IN_STATION_MODE = -0x7D0,
DEVICE_START_FAILED = DEVICE_NOT_IN_STATION_MODE - 1,
INVALID_HEX_STRING = DEVICE_START_FAILED - 1,
TCP_RECV_ERROR = INVALID_HEX_STRING - 1,
TCP_SEND_ERROR = TCP_RECV_ERROR - 1,
FILE_NOT_FOUND_ERROR = TCP_SEND_ERROR - 1,
INVALID_SERVER_RESPONSE = FILE_NOT_FOUND_ERROR - 1,
FORMAT_NOT_SUPPORTED = INVALID_SERVER_RESPONSE - 1,
FILE_OPEN_FAILED = FORMAT_NOT_SUPPORTED - 1,
FILE_WRITE_ERROR = FILE_OPEN_FAILED - 1,
INVALID_FILE = FILE_WRITE_ERROR - 1,
SERVER_CONNECTION_FAILED = INVALID_FILE - 1,
GET_HOST_IP_FAILED = SERVER_CONNECTION_FAILED  - 1,

STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulDestinationIP; // IP address of destination server
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
static unsigned int g_uiDeviceModeConfig;
unsigned long  g_ulStaIp = 0;
unsigned char g_buff[MAX_BUFF_SIZE+1];
const char *soft_layer = "api.thingsio.ai"; //"<host name>"
//const char *soft_layer = "netracollab.ntpc.co.in";
const char *Time_server = "baas.thethingscloud.com"; //"<host name>"
char slave_str[4];
bool g_logged_in = false,proceed = false;
bool critiacl_section = false;
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//const int Adress[12] = {30001, 30002};

int temp_index;

const char*name_list[2] = {
		"temp"
};

//const char *Time_server = "198.211.122.7"; //"<host name>"
char Query_set[8];
long bytesReceived = 0; // variable to store the file size
char ses_id[500],recv_buffer[20];
char post_data[1000];
char timestamp[10];
char no_of_slaves;
char Post_string_size[4];

int uartRxLength,s=0;
int result;
char slave_list[256];
char instance=0;
long uint_count;
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
static void BoardInit(void);
static void DisplayBanner(char * AppName);

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************
void uart_init();
void WlanAPMode();

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************
static Mesage(unsigned char *m)
{
    UDMASetupTransfer(UDMA_CH9_UARTA0_TX,
                        UDMA_MODE_BASIC,
                        8,
                        UDMA_SIZE_8,
                        UDMA_ARB_8,
                        (void *)m,
                        UDMA_SRC_INC_8,
                        (void *)(UARTA0_BASE+UART_O_DR),
                        UDMA_DST_INC_NONE);

//
// Enable TX DMA request
//
MAP_UARTDMAEnable(UARTA0_BASE,UART_DMA_TX);

}

static inline void HIBEntrePreamble()
{
    HWREG(0x400F70B8) = 1;
    UtilsDelay(800000/5);
    HWREG(0x400F70B0) = 1;
    UtilsDelay(800000/5);

    HWREG(0x4402E16C) |= 0x2;
    UtilsDelay(800);
    HWREG(0x4402F024) &= 0xF7FFFFFF;
}
//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
            {
                SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

//
// Information about the connected AP (like name, MAC etc) will be
// available in 'slWlanConnectAsyncResponse_t'-Applications
// can use it if required

// Copy new connection SSID and BSSID to global parameters
        memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
        STAandP2PModeWlanConnected.ssid_name,
        pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
        memcpy(g_ucConnectionBSSID,
        pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
        SL_BSSID_LENGTH);
            }
            break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

// If the user has initiated 'Disconnect' request,
//'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
                {

                }
            else
            {

            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        case SL_WLAN_STA_CONNECTED_EVENT:
        {
// when device is in AP mode and any client connects to device cc3xxx
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

//
// Information about the connected client (like SSID, MAC etc) will be
// available in 'slPeerInfoAsyncResponse_t' - Applications
// can use it if required
//

        }
        break;

        case SL_WLAN_STA_DISCONNECTED_EVENT:
        {
// when client disconnects from device (AP)
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

//
// Information about the connected client (like SSID, MAC etc) will
// be available in 'slPeerInfoAsyncResponse_t' - Applications
// can use it if required
//
//
        }
        break;

        default:
        {

        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

                //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

        break;
        case SL_NETAPP_IP_LEASED_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            g_ulStaIp = (pNetAppEvent)->EventData.ipLeased.ip_address;

        }
        break;

        case SL_NETAPP_IP_RELEASED_EVENT:
        {
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

        }
        break;


        default:
        {

        }
        break;
    }
 }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
SlHttpServerResponse_t *pHttpResponse)
{
// Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
//
// Most of the general errors are not FATAL are are to be handled
// appropriately by the application
//
UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
pDevEvent->EventData.deviceEvent.status,
pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
//
// This application doesn't work w/ socket - Events are not expected
//
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
                switch( pSock->socketAsyncEvent.SockTxFailData.status )
                {
                    case SL_ECLOSE:
                        UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                pSock->socketAsyncEvent.SockAsyncData.sd);
                        break;
                    default:
                        UART_PRINT("[SOCK ERROR] - TX FAILED : socket %d , reason"
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockAsyncData.sd,
                                pSock->socketAsyncEvent.SockTxFailData.status);
                }
                break;

                default:
                    UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
/*****************************************************************************/
#ifdef WATCHDOG
void watchdogIntHandler(void)
{
    GPIO_IF_LedOn(MCU_ALL_LED_IND);
    MAP_UtilsDelay(50);
    GPIO_IF_LedOff(MCU_ALL_LED_IND);


}

/*******************************************************************************************************/

static watchdogack()
{
    MAP_WatchdogIntClear(WDT_BASE);
}
#endif
/*****************************************************************************/
//
//! \brief This function initializes the application variables
//!
//! \param    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_ulStaIp = 0;
    g_ulGatewayIP = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
}

static void delay(unsigned int d)
{
    unsigned int i;
    for (i=0;i<d*100;i++){}
}


//*****************************************************************************//

static int ConfigureMode(int iMode)
{

    long   lRetVal = -1;

    lRetVal = sl_WlanSetMode(iMode);
    ASSERT_ON_ERROR(lRetVal);



    //Mesage("Device is configured in AP mode\n\r");

    /* Restart Network processor */
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    // reset status bits
    CLR_STATUS_BIT_ALL(g_ulStatus);

    return sl_Start(NULL,NULL,NULL);
}
//*******************************************************************************//
//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
          _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

          unsigned char ucVal = 1;
          unsigned char ucConfigOpt = 0;
          unsigned char ucConfigLen = 0;
          unsigned char ucPower = 0;

          long lRetVal = -1;
          long lMode = -1;


          lMode = sl_Start(0, 0, 0);
          ASSERT_ON_ERROR(lMode);
          sl_WlanPolicySet(SL_POLICY_CONNECTION,SL_CONNECTION_POLICY(1,0,0,0,1),NULL,0);
          if(ROLE_STA != lRetVal && g_uiDeviceModeConfig == ROLE_STA )
          {
              if (ROLE_AP == lRetVal)
              {
                  // If the device is in AP mode, we need to wait for this event
                  // before doing anything
                  while(!IS_IP_ACQUIRED(g_ulStatus))
                  {
                  #ifndef SL_PLATFORM_MULTI_THREADED
                    _SlNonOsMainLoopTask();
                  #endif
                  }
              }
              //Switch to STA Mode
              lRetVal = ConfigureMode(ROLE_STA);
              ASSERT_ON_ERROR( lRetVal);
          }
          GPIO_IF_LedToggle(MCU_RED_LED_GPIO);
          //Device is in STA Mode and Force AP Jumper is Connected
          if(ROLE_AP != lRetVal && g_uiDeviceModeConfig == ROLE_AP )
          {
               //Switch to AP Mode
               lRetVal = ConfigureMode(ROLE_AP);
               ASSERT_ON_ERROR( lRetVal);

          }
          // If the device is not in station-mode, try configuring it in station-mode

          // Get the device's version-information
          ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
          ucConfigLen = sizeof(ver);
          lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                      &ucConfigLen, (unsigned char *)(&ver));
          ASSERT_ON_ERROR(lRetVal);

          UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
          UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
          ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
          ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
          ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
          ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
          ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

          //
          // Device in station-mode. Disconnect previous connection if any
          // The function returns 0 if 'Disconnected done', negative number if already
          // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
          // other return-codes
          //
          GPIO_IF_LedToggle(MCU_RED_LED_GPIO);
          if (g_uiDeviceModeConfig != ROLE_AP)
                  {
          lRetVal = sl_WlanDisconnect();
          if(0 == lRetVal)
          {
              // Wait
              while(IS_CONNECTED(g_ulStatus))
              {
      #ifndef SL_PLATFORM_MULTI_THREADED
                    _SlNonOsMainLoopTask();
      #endif
              }
          }
                  }
          // Enable DHCP client
          lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
          ASSERT_ON_ERROR(lRetVal);
          if (g_uiDeviceModeConfig != ROLE_AP)
          {
          // Disable scan
          ucConfigOpt = SL_SCAN_POLICY(0);
          lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
          ASSERT_ON_ERROR(lRetVal);
          }
          // Set Tx power level for station mode
          // Number between 0-15, as dB offset from max power - 0 will set max power
          ucPower = 0;
          lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
                  WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
          ASSERT_ON_ERROR(lRetVal);

          // Set PM policy to normal
          lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
          ASSERT_ON_ERROR(lRetVal);

          // Unregister mDNS services
          lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
          ASSERT_ON_ERROR(lRetVal);

          // Remove  all 64 filters (8*8)
          memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
          lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                             sizeof(_WlanRxFilterOperationCommandBuff_t));
          ASSERT_ON_ERROR(lRetVal);
          GPIO_IF_LedToggle(MCU_RED_LED_GPIO);

          lRetVal = sl_Stop(0);
          ASSERT_ON_ERROR(lRetVal);
          if (lRetVal!=0)
          {
              lRetVal = sl_Stop(0);
              ASSERT_ON_ERROR(lRetVal);
          }
          InitializeAppVariables();
          lMode = sl_Start(0, 0, 0);
          ASSERT_ON_ERROR(lMode);

          return SUCCESS;
}




//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  Status value
//!
//! \return  None
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect()
{

// Wait for WLAN Event
    UART_PRINT("Awaiting an AP connection...");
                while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
                {
                    // wait till connects to an AP
                    GPIO_IF_LedToggle(MCU_RED_LED_GPIO);
                    _SlNonOsMainLoopTask();
                    delay(5000);
                }
    GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    UART_PRINT("connected to an AP\n");
    return SUCCESS;

}

void WlanAPMode(  )
{

    unsigned char ucDHCP;
    long lRetVal = -1;

    InitializeAppVariables();

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
        UART_PRINT("Failed to configure the device in its default state \n\r");

        LOOP_FOREVER();
    }

    // UART_PRINT("Device is configured in default state \n\r");

    //
    // Asumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(NULL,NULL,NULL);

    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start the device \n\r");
        LOOP_FOREVER();
    }
    //
    // Configure the networking mode and ssid name(for AP mode)
    //
    if(lRetVal != ROLE_AP)
    {
        if(ConfigureMode(lRetVal) != ROLE_AP)
            {
                UART_PRINT("Unable to set AP mode, exiting Application...\n\r");
                    sl_Stop(SL_STOP_TIMEOUT);
                    LOOP_FOREVER();
            }
    }

    while(!IS_IP_ACQUIRED(g_ulStatus))
    {
    //looping till ip is acquired
    }

        unsigned char len = sizeof(SlNetCfgIpV4Args_t);
        SlNetCfgIpV4Args_t ipV4 = {0};

        // get network configuration
        lRetVal = sl_NetCfgGet(SL_IPV4_AP_P2P_GO_GET_INFO,&ucDHCP,&len,
        (unsigned char *)&ipV4);
        if (lRetVal < 0)
        {
            UART_PRINT("Failed to get network configuration \n\r");
            LOOP_FOREVER();
        }

        UART_PRINT("Connect a client to Device\n\r");
        while(!IS_IP_LEASED(g_ulStatus))
        {
        //wating for the client to connect
        }
        UART_PRINT("Client is connected to Device\n\r");
        // Switch off Network processor
        lRetVal = sl_Stop(SL_STOP_TIMEOUT);
        UART_PRINT("WLAN AP example executed successfully");
        while(1);
}




//*****************************************************************************
//
//! \brief Handler for parsing JSON data
//!
//! \param[in]  ptr - Pointer to http response body data
//!
//! \return 0 on success else error code on failure
//!
//*****************************************************************************
int ParseJSONData(char *ptr)
{
    long lRetVal = 0;
    int noOfToken;
    jsmn_parser parser;
    jsmntok_t   *tokenList;


    /* Initialize JSON PArser */
    jsmn_init(&parser);

    /* Get number of JSON token in stream as we we dont know how many tokens need to pass */
    noOfToken = jsmn_parse(&parser, (const char *)ptr, strlen((const char *)ptr), NULL, 10);
    if(noOfToken <= 0)
        {
            UART_PRINT("Failed to initialize JSON parser\n\r");
            return -1;

        }

    /* Allocate memory to store token */
    tokenList = (jsmntok_t *) malloc(noOfToken*sizeof(jsmntok_t));
    if(tokenList == NULL)
    {
    UART_PRINT("Failed to allocate memory\n\r");
    return -1;
    }

/* Initialize JSON Parser again */
    jsmn_init(&parser);
    noOfToken = jsmn_parse(&parser, (const char *)ptr, strlen((const char *)ptr), tokenList, noOfToken);
    if(noOfToken < 0)
    {
        UART_PRINT("Failed to parse JSON tokens\n\r");
        lRetVal = noOfToken;
    }
    else
    {
        UART_PRINT("Successfully parsed %ld JSON tokens\n\r", noOfToken);
    }

    free(tokenList);

    return lRetVal;
}

/*!
\brief This function read respose from server and dump on console

\param[in]      httpClient - HTTP Client object

\return         0 on success else -ve

\note

\warning
*/
static int FlushHTTPResponse(HTTPCli_Handle httpClient)
{
        int lRetVal = -1,status;
        char error[15];
        status = HTTPCli_getResponseStatus(httpClient);

        sprintf(error,"post status %d",status);
        UART_PRINT(error);
        UART_PRINT("\n");

        char *respFields[2] = {
                                HTTPCli_FIELD_NAME_CONTENT_LENGTH,
                                NULL
                                };
        char buf[356];
        int len;
        bool moreFlag = 0;

        /* Store previosly store array if any */
        HTTPCli_setResponseFields(httpClient, respFields);

        do {
                // Filter the response headers and get the set response field
                //
                // ...
                // Content-type: text/xml; charset=utf-8\r\n
                // Content-length: 34
                // ...
                lRetVal = HTTPCli_getResponseField(httpClient, buf, sizeof(buf), &moreFlag);

                // process data in buf if field is content length
                // Zero is the index of Content length in respFields array
                if (lRetVal == 0) {
                len = (int)strtoul(buf, NULL, 0);
            }

            //loop till the end of the response fields
            } while (lRetVal != HTTPCli_FIELD_ID_END);
        //UART_PRINT(buf);
        while (len > 0) {
        len -= HTTPCli_readRawResponseBody(httpClient, buf, sizeof(buf));
        // process buf data and save
        }

        UART_PRINT(buf);
        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
        return 0;

}
/************************************************************************************************/
/***********************************************************************************************************/


static int readResponse(HTTPCli_Handle httpClient)
{
        long lRetVal = 0;
        int bytesRead = 0;
        int i,j;
        char error[15];
        unsigned long len;

        char *S_id,*P;
        char *dataBuffer = NULL;

        const char *ids[4] = {
                                HTTPCli_FIELD_NAME_CONTENT_LENGTH,
                                HTTPCli_FIELD_NAME_CONNECTION,
                                HTTPCli_FIELD_NAME_CONTENT_TYPE,
                                NULL
                            };
        //JsonNode *jn,*jns;

        /* Read HTTP POST request status code */

          j=0;

        HTTPCli_setResponseFields(httpClient, (const char **)ids);



        switch (instance)
        {
            case TIME:

            	delay(4000);
            	lRetVal = HTTPCli_getResponseStatus(httpClient);
            	              if(lRetVal==200)
            	              {
            	                     UART_PRINT("Time status 200\n");
            	                      // GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
            	               }
            	               else
            	               {
            	                      sprintf(error,"Time status %d",lRetVal);
            	                      UART_PRINT(error);
            	                      UART_PRINT("\n");
            	                }
            	                      len = 2500;
            	                      dataBuffer = calloc(len,sizeof(char));
            	                      bytesRead = HTTPCli_readRawResponseBody(httpClient, (char *)dataBuffer, len);
            	                      dataBuffer[bytesRead] = '\0';
            	                      P = strstr(dataBuffer,"\"timestamp\"");
            	                      for (i=0;i<10;i++)
            	                      {
            	                             timestamp[i]=*(P+12+i);
            	                       }
            	                            UART_PRINT(timestamp);
            	                            break;
            case POST:
                            FlushHTTPResponse(httpClient);
                            break;
        }
        /* treating data as a plain text */

        lRetVal = 0;

            if(len > sizeof(g_buff) && (dataBuffer != NULL))
            {
                UART_PRINT("Freeing allocated databuffer\r\n");
                free(dataBuffer);
            }
        return lRetVal;

}

//*****************************************************************************
//
//! \brief HTTP POST Demonstration
//!
//! \param[in]  httpClient - Pointer to http client
//!
//! \return 0 on success else error code on failure
//!
//*****************************************************************************
static int HTTPPostMethod(HTTPCli_Handle httpClient, char*post_uri)
{
        bool moreFlags = 1;
        bool lastFlag = 1;
        long lRetVal = 0;
        HTTPCli_Field fields[5] = {
        		{HTTPCli_FIELD_NAME_HOST, soft_layer},
        		                                    {HTTPCli_FIELD_NAME_CONTENT_TYPE, "application/json"},
        		                                    {HTTPCli_FIELD_NAME_AUTHORIZATION,"Bearer eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.IjViMjBhN2UyODQ2YjFhNDBiMmIxYTFlZiI.5xoErxmyV5sCc3JYBHwpwXnfH6Mk7S9XVVWqvwDB9Cs"},
        		                                    {NULL, NULL}
                                      };
        /* Set request header fields to be send for HTTP request. */
        HTTPCli_setRequestFields(httpClient, fields);
        /* Send POST method request. */
        /* Here we are setting moreFlags = 1 as there are some more header fields need to send
        other than setted in previous call HTTPCli_setRequestFields() at later stage.
        Please refer HTTP Library API documentaion @ref HTTPCli_sendRequest for more information.
        */
        moreFlags = 1;
        lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_POST,POST_REQUEST_URI_DC , moreFlags);
        if(lRetVal < 0)
        {
            UART_PRINT("Failed to send HTTP POST request header.\n\r");
            return lRetVal;
        }
        /* Here we are setting lastFlag = 1 as it is last header field.
        Please refer HTTP Library API documentaion @ref HTTPCli_sendField for more information.
        */
        lastFlag = 1;
        lRetVal = HTTPCli_sendField(httpClient, HTTPCli_FIELD_NAME_CONTENT_LENGTH, (const char *)Post_string_size, lastFlag);
        if(lRetVal < 0)
        {
            UART_PRINT("Failed to send HTTP POST request header.\n\r");
            return lRetVal;
        }

        /* Send POST data/body */
        lRetVal = HTTPCli_sendRequestBody(httpClient, post_data, (strlen(post_data)));
        if(lRetVal < 0)
        {
            UART_PRINT("Failed to send HTTP POST request body.\n\r");
            return lRetVal;
        }
        instance = POST;
        lRetVal = readResponse(httpClient);
        return lRetVal;
}

//*****************************************************************************


static int HTTPGetMethod(HTTPCli_Handle httpClient)
{

        long lRetVal = 0;

        instance = TIME;

        HTTPCli_Field fields[4] = {
                                    {HTTPCli_FIELD_NAME_HOST, Time_server},
                                    {HTTPCli_FIELD_NAME_ACCEPT, "*/*"},
                                    {HTTPCli_FIELD_NAME_CONTENT_LENGTH, "0"},
                                    {NULL, NULL}
                                };
        bool        moreFlags;


        /* Set request header fields to be send for HTTP request. */
        HTTPCli_setRequestFields(httpClient, fields);

        /* Send GET method request. */
        /* Here we are setting moreFlags = 0 as there are no more header fields need to send
        at later stage. Please refer HTTP Library API documentaion @ HTTPCli_sendRequest
        for more information.
        */
        moreFlags = 0;
        lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_GET, GET_REQUEST_URI, moreFlags);
        if(lRetVal < 0)
        {
            UART_PRINT("Failed to send HTTP GET request.\n\r");
            return lRetVal;
        }
        MAP_UtilsDelay(0xffff);
        lRetVal = readResponse(httpClient);
        return lRetVal;
}

//*****************************************************************************
//
//! Function to connect to AP
//!
//! \param  none
//!
//! \return Error-code or SUCCESS
//!
//*****************************************************************************
static long ConnectToAP()
{
        int lRetVal = -1;

        //
        // Following function configure the device to default state by cleaning
        // the persistent settings stored in NVMEM (viz. connection profiles &
        // policies, power policy etc)
        //
        // Applications may choose to skip this step if the developer is sure
        // that the device is in its desired state at start of applicaton
        //
        // Note that all profiles and persistent settings that were done on the
        // device will be lost
        //

        // Connecting to WLAN AP - Set with static parameters defined at the top
        // After this call we will be connected and have IP address
        lRetVal = WlanConnect();
        return 0;
}

//*****************************************************************************
//
//! Function to connect to HTTP server
//!
//! \param  httpClient - Pointer to HTTP Client instance
//!
//! \return Error-code or SUCCESS
//!
//*****************************************************************************
static int ConnectToHTTPServer(HTTPCli_Handle httpClient,const char *host_name)
{
        long lRetVal = -1;
        struct sockaddr_in addr;

#ifdef USE_PROXY
        struct sockaddr_in paddr;
        paddr.sin_family = AF_INET;
        paddr.sin_port = htons(PROXY_PORT);
        paddr.sin_addr.s_addr = sl_Htonl(PROXY_IP);
        HTTPCli_setProxy((struct sockaddr *)&paddr);
        #endif
    //    UART_PRINT(Buf);
        /* Resolve HOST NAME/IP */
        lRetVal = sl_NetAppDnsGetHostByName((signed char *)host_name,
        strlen((const char *)host_name),
        &g_ulDestinationIP,SL_AF_INET);
        if(lRetVal < 0)
        {
            ASSERT_ON_ERROR(GET_HOST_IP_FAILED);
        }

        /* Set up the input parameters for HTTP Connection */
        addr.sin_family = AF_INET;
        if (host_name == Time_server)
        {
            addr.sin_port = htons(TIME_PORT);
        }
        if(host_name == soft_layer)
        {
            addr.sin_port = htons(HOST_PORT);
        }

        addr.sin_addr.s_addr = sl_Htonl(g_ulDestinationIP);
        /* Testing HTTPCli open call: handle, address params only */
        HTTPCli_construct(httpClient);
        lRetVal = HTTPCli_connect(httpClient, (struct sockaddr *)&addr, 0, NULL);
        if (lRetVal < 0)
        {
            UART_PRINT("Connection to server failed. error(%d)\n\r", lRetVal);
            ASSERT_ON_ERROR(SERVER_CONNECTION_FAILED);
        }
        else
        {
            UART_PRINT("Connection to server created successfully\r\n");
        }
        return 0;
}



/************************************************************************************************************************/
static void ReadDeviceConfiguration()
{
        unsigned int uiGPIOPort;
        unsigned char pucGPIOPin;
        unsigned char ucPinValue;

        //Read GPIO
        GPIO_IF_GetPortNPin(SH_GPIO_3,&uiGPIOPort,&pucGPIOPin);
        ucPinValue = GPIO_IF_Get(SH_GPIO_3,uiGPIOPort,pucGPIOPin);

        //If Connected to VCC, Mode is AP
        if(ucPinValue == 1)
        {
            //AP Mode
            UART_PRINT("assigning AP role\r\n");
            g_uiDeviceModeConfig = ROLE_AP;
        }
        else
        {
            //STA Mode
            UART_PRINT("assigning STA  role\r\n");
            g_uiDeviceModeConfig = ROLE_STA;
        }

}
//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************

void construct_post_buffer(int l,int ind)
{
    char *pcBufLocation;
    int temp,i,lRetVal,start_val,end_val;
    unsigned long  uiAdcInputPin;
       unsigned int  uiChannel;
       unsigned int  uiIndex=0;
       unsigned long ulSample;
       char * AppName;

       //
       // Initialize Board configurations
       //
       uiIndex=0;


   #ifdef CC3200_ES_1_2_1
           //
           // Enable ADC clocks.###IMPORTANT###Need to be removed for PG 1.32
           //
           HWREG(GPRCM_BASE + GPRCM_O_ADC_CLK_CONFIG) = 0x00000043;
           HWREG(ADC_BASE + ADC_O_ADC_CTRL) = 0x00000004;
           HWREG(ADC_BASE + ADC_O_ADC_SPARE0) = 0x00000100;
           HWREG(ADC_BASE + ADC_O_ADC_SPARE1) = 0x0355AA00;
   #endif
           //
           // Pinmux for the selected ADC input pin
           //
           MAP_PinTypeADC(uiAdcInputPin,PIN_MODE_255);

           //
           // Convert pin number to channel number
           //

           uiAdcInputPin = PIN_58;
   		//uiAdcInputPin = PIN_59;
   		//uiAdcInputPin = PIN_60;

           switch(uiAdcInputPin)
           {
               case PIN_58:
                   uiChannel = ADC_CH_1;
                   break;
               case PIN_59:
                   uiChannel = ADC_CH_2;
                   break;
               case PIN_60:
                   uiChannel = ADC_CH_3;
                   break;
               default:
                   break;
           }

           //
           // Configure ADC timer which is used to timestamp the ADC data samples
           //
           MAP_ADCTimerConfig(ADC_BASE,2^17);

           //
           // Enable ADC timer which is used to timestamp the ADC data samples
           //
           MAP_ADCTimerEnable(ADC_BASE);

           //
           // Enable ADC module
           //
           MAP_ADCEnable(ADC_BASE);

           //
           // Enable ADC channel
           //

           MAP_ADCChannelEnable(ADC_BASE, uiChannel);

           while(uiIndex < NO_OF_SAMPLES + 4)
           {
               if(MAP_ADCFIFOLvlGet(ADC_BASE, uiChannel))
               {
                   ulSample = MAP_ADCFIFORead(ADC_BASE, uiChannel);
                   pulAdcSamples[uiIndex++] = ulSample;
               }


           }

           MAP_ADCChannelDisable(ADC_BASE, uiChannel);

           uiIndex = 0;

           //UART_PRINT("\n\rTotal no of 32 bit ADC data printed :4096 \n\r");
           //UART_PRINT("\n\rADC data format:\n\r");
           //UART_PRINT("\n\rbits[13:2] : ADC sample\n\r");
           //UART_PRINT("\n\rbits[31:14]: Time stamp of ADC sample \n\r");

           //
           // Print out ADC samples
           //
           float output_voltage, sensor_data,faren;
           double temperature, temp1, temp2;


           //UART_PRINT("\n\rVoltage is %f\n\r",((pulAdcSamples[4] >> 2 ) & 0x0FFF)*1.4/4096);
//           UART_PRINT("\n\r");
    switch(l)
    {
    case 1:   start_val = 0;
              end_val = 1;
              break;
  }

    char *temp_str;
    temp_str = (char*)calloc(50,sizeof(char));

    pcBufLocation = post_data;

    strcpy(pcBufLocation,POST_HEADER);
        pcBufLocation += strlen(POST_HEADER);

        strcpy(pcBufLocation , POST_timestamp);
        pcBufLocation += strlen(POST_timestamp);
        strcpy(pcBufLocation , timestamp);
        pcBufLocation += strlen(timestamp);

        strcpy(pcBufLocation , POST_DEVICEID);
        pcBufLocation += strlen(POST_DEVICEID);

        strcpy(pcBufLocation , POST_SLAVE);
        pcBufLocation += strlen(POST_SLAVE);


        strcpy(pcBufLocation,POST_HEADER1);
        pcBufLocation += strlen(POST_HEADER1);

        strcpy(pcBufLocation,POST_BEGIN);
        pcBufLocation += strlen(POST_BEGIN);

    /***************************** phase1 parameters***************************************/


    for (i = start_val;i<end_val;i++)
    {
                    UART_PRINT("\n\rVoltage is %f\n\r",(((float)((pulAdcSamples[4+uiIndex] >> 2 ) & 0x0FFF))*1.4)/4096);
                    output_voltage = (((float)((pulAdcSamples[4+uiIndex] >> 2 ) & 0x0FFF))*1.4)/4096;
                    uiIndex++;
                    temperature = -((output_voltage-0.5)*100);
                    UART_PRINT("\n\r temperature = %f",temperature);


    	   sprintf(temp_str,"\n\"%s\":%f",name_list[i],temperature);
    	   strcpy(pcBufLocation , temp_str);
    	   pcBufLocation += strlen(temp_str);
    }
    /***************************************************************************************/

      sprintf(slave_str,"%d",slave_list[ind]);

            strcpy(pcBufLocation , POST_END);
             pcBufLocation += strlen(POST_END);


            strcpy(pcBufLocation,POST_TAIL1);
            pcBufLocation += strlen(POST_TAIL1);
      free(temp_str);
}




static send_session(int session_no,HTTPCli_Struct httpClient,int ind)
{
    int lRetVal;


    GPIO_IF_LedToggle(MCU_ORANGE_LED_GPIO);
    construct_post_buffer(session_no,ind);
    UART_PRINT("size of post_data ====== %d\r\n", strlen(post_data));
    sprintf((char *)Post_string_size, "%d", strlen(post_data));
    UART_PRINT(" post data = %s len = %s\r\n", post_data, Post_string_size);


    lRetVal = ConnectToHTTPServer(&httpClient,soft_layer);
      if(lRetVal < 0)
      {
         UART_PRINT("CONNECTION FAILED");
      }

    #ifdef WATCHDOG
    if (IS_CONNECTED(g_ulStatus)) watchdogack();
    #endif
    GPIO_IF_LedToggle(MCU_ORANGE_LED_GPIO);
    lRetVal = HTTPPostMethod(&httpClient,POST_REQUEST_URI_DC);
    if(lRetVal < 0)
    {
       Mesage("HTTP Post failed.\n\r");
    }

    GPIO_IF_LedToggle(MCU_ORANGE_LED_GPIO);
    HTTPCli_disconnect(&httpClient);
    HTTPCli_destruct(&httpClient);
    HTTPCli_delete(&httpClient);

}
static void
DisplayBanner(char * AppName)
{
        UART_PRINT("\n\n\n\r");
        UART_PRINT("\t\t *************************************************\n\r");
        UART_PRINT("\t\t      CC3200 %s Application       \n\r", AppName);
        UART_PRINT("\t\t *************************************************\n\r");
        UART_PRINT("\n\n\n\r");
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
        /* In case of TI-RTOS vector table is initialize by OS itself */
        #ifndef USE_TIRTOS
        //
        // Set vector table base
        //
        #if defined(ccs) || defined(gcc)
        MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
        #endif
        #if defined(ewarm)
        MAP_IntVTableBaseSet((unsigned long)&__vector_table);
        #endif
        #endif
        //
        // Enable Processor
        //
        MAP_IntMasterEnable();
        MAP_IntEnable(FAULT_SYSTICK);

        PRCMCC3200MCUInit();
}

void Recv_response()
{
   int i =0;
   char c;
   delay(1000);  // PLAY WITH THIS ALSO TO SEE THE DIFFERENCE
   while(UARTCharsAvail(CONSOLE1) && i<25)
    {
        recv_buffer[i++] = UARTCharGetNonBlocking(CONSOLE1);
    }
    uartRxLength = i;
    recv_buffer[i] = '\0';
    proceed = true;
}

static void Uart1IntHandler(void)
{

    Recv_response();
    MAP_UARTIntClear(UARTA1_BASE,UART_INT_RX);
}


int main()
{
        long lRetVal = -1;

        //HTTPCli_Struct httpClient;

        unsigned long ulResetCause;

        static int ind;
        float *pfCurrTemp;

        //

        // Board Initialization

        //

        BoardInit();
        //

        // Configure the pinmux settings for the peripherals exercised

        //

        PinMuxConfig();

        PinConfigSet(PIN_58,PIN_STRENGTH_2MA|PIN_STRENGTH_4MA,PIN_TYPE_STD_PD);

        // Configuring UART

        UDMAInit();

        uart_init();

        GPIO_IF_LedConfigure(LED1|LED2|LED3);


        ulResetCause = PRCMSysResetCauseGet();

        //

        // If watchdog triggered reset request hibernate

        // to clean boot the system

        //

        if( ulResetCause == PRCM_WDT_RESET )

        {

            HIBEntrePreamble();

            MAP_PRCMOCRRegisterWrite(0,1);

            MAP_PRCMHibernateWakeupSourceEnable(PRCM_HIB_SLOW_CLK_CTR);

            MAP_PRCMHibernateIntervalSet(330);

            MAP_PRCMHibernateEnter();

        }

        GPIO_IF_LedOff(MCU_ALL_LED_IND);

        //

        // Display banner

        //

        DisplayBanner(APP_NAME);

        if( ulResetCause == PRCM_HIB_EXIT &&  (MAP_PRCMOCRRegisterRead(0) & 1) == 1 )

           {

               UART_PRINT("Reset Cause        : Watchdog Reset\n\r");

           }

           else

           {

               UART_PRINT("Reset Cause        : Power On\n\r");

           }

        ReadDeviceConfiguration();

        HTTPCli_Struct httpClient;

        HTTPCli_Struct httpClient1;

        if (g_uiDeviceModeConfig == ROLE_AP)

        {

            s=1;

            WlanAPMode();

        }

        else

        {

#ifdef WATCHDOG

            WDT_IF_Init(watchdogIntHandler, MILLISECONDS_TO_TICKS(WD_PERIOD_MS));

#endif

            lRetVal = ConfigureSimpleLinkToDefaultState();

            if(lRetVal < 0)

            {

                if (DEVICE_NOT_IN_STATION_MODE == lRetVal)

                {

                    UART_PRINT("Failed to configure the device in its default state, "

                    "Error-code: %d\n\r", DEVICE_NOT_IN_STATION_MODE);

                }

                return -1;

            }

            UART_PRINT("Simple link configured to default state\n");
            lRetVal = ConnectToAP();

            if(lRetVal < 0)

            {

                LOOP_FOREVER();

            }

            s=1;

            while(1)

            {

               #ifdef WATCHDOG

                           if (IS_CONNECTED(g_ulStatus)) watchdogack();

               #endif

                   GPIO_IF_LedToggle(MCU_ORANGE_LED_GPIO);
                   lRetVal = ConnectToHTTPServer(&httpClient1,Time_server);

                   if(lRetVal < 0)

                   {

                       UART_PRINT("CONNECTION FAILED");
                   }

                   GPIO_IF_LedToggle(MCU_ORANGE_LED_GPIO);

               #ifdef WATCHDOG

                           if (IS_CONNECTED(g_ulStatus)) watchdogack();

               #endif
                   lRetVal = HTTPGetMethod(&httpClient1);

                   if(lRetVal < 0)

                   {

                       UART_PRINT("HTTP  Get failed.\n\r");

                   }

                   GPIO_IF_LedToggle(MCU_ORANGE_LED_GPIO);

               #ifdef WATCHDOG

                           if (IS_CONNECTED(g_ulStatus)) watchdogack();

               #endif

                   HTTPCli_disconnect(&httpClient1); //for timestamp

                   HTTPCli_destruct(&httpClient1);

                   HTTPCli_delete(&httpClient1);

                   float temp;
//                   TMP006DrvGetTemp(&temp);
//                   UART_PRINT("temp = %f",temp);
                  send_session(1,httpClient,ind);

       #ifdef WATCHDOG

                   if (IS_CONNECTED(g_ulStatus)) watchdogack();

       #endif

                   GPIO_IF_LedToggle(MCU_ORANGE_LED_GPIO);

                   ind++;

                   if(ind >= no_of_slaves)

                   {

                       ind = 0;

                       delay(9000);

                       delay(9000);

                       delay(9000);

                   }

                   GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

                   GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);


#ifdef WATCHDOG

            WDT_IF_DeInit();

#endif

            }

        }

}
void uart_init()
{
    MAP_UARTConfigSetExpClk(UARTA1_BASE,MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                        9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                   UART_CONFIG_PAR_NONE));
     MAP_UARTFIFOEnable(UARTA0_BASE);
     MAP_UARTFIFOLevelSet (UARTA0_BASE,UART_FIFO_TX4_8,UART_FIFO_RX4_8 );
     InitTerm();
     MAP_UARTFlowControlSet(UARTA1_BASE, UART_FLOWCONTROL_TX);
     MAP_UARTFIFOEnable(UARTA1_BASE);
     MAP_UARTFIFOLevelSet (UARTA1_BASE,UART_FIFO_TX4_8,UART_FIFO_RX1_8);
     MAP_UARTIntRegister(UARTA1_BASE, Uart1IntHandler);
     MAP_UARTIntEnable(UARTA1_BASE,UART_INT_RX);

}