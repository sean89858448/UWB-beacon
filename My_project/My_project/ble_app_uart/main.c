#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_uart.h"
#include "nrf_uart.h"

#include "nrf_delay.h"

//標示  SoftDevice BLE標籤
#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

//間格時間
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
//訊號長度
#define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
//製作商數據長度
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
//0x02為 Beacon
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
//RSSI強度
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
//Nordic Semiconductor ASA 的公司標識符，不改成004         C，，ios手機搜索不到設備。
#define APP_COMPANY_IDENTIFIER          0x004C                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
//設定最大、最小值範圍 
#define APP_MAJOR_VALUE                 0xFF, 0xFF                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x00, 0x01                         /**< Minor value used to identify Beacons. */
//設定  UUID
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xff             /**< Proprietary UUID for Beacon. */

static uint8_t update_beacon_info[] =
{
    0x02,0x15,0x00,0x01,0x02,0x03,0x04,0x05,
    0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,
    0x0E,0x0F,0xFF,0xFF,0x00,0x01,0xC3
};

static uint8_t update2_beacon_info[] =
{
    0x02,0x15,0x01,0x12,0x23,0x34,0x45,0x56, 
    0x67,0x78,0x89,0x9a,0xab,0xbc,0xcd,0xde, 
    0xef,0xff,0x12,0x34,0x56,0x78,0xB5
};

static uint8_t cmdIndex = 0;
static uint8_t ID_Index = 0;

uint8_t times_A = 0, times_B = 0;

static bool update_request = false;

char AT_TurnOn_Command[5][17] = {
  {"AT"},
  {"AT+RST"},
  {"AT+anchor_tag=0,1"},
  {"AT+interval=10"},
  {"AT+switchdis=1"}
};

#define UART_TX_BUF_SIZE                256                                /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                /**< UART RX buffer size. */

//用作錯誤代碼的值
#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
//儲存編碼廣告的       Buffer
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

//包含指向編碼廣告數據的指針的簡短結構。
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0
    }
};


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in this implementation.
};


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);                        //錯誤處理函數，發生錯誤時調用

    NRF_LOG_DEFAULT_BACKENDS_INIT();                  //初始化   log 設置，配置     log 通道
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


void uart_event_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          NRF_UART_BAUDRATE_115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
                       
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("uart init\n");
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void){
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier  = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data         = m_beacon_info;
    manuf_specific_data.data.size           = APP_BEACON_INFO_LENGTH;

    //建立和設置廣告數據
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // 初始化廣告參數（在開始廣告時使用）
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    //選擇廣告類型
    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;   //不可連接不可掃描的非定向廣告事件。
    m_adv_params.p_peer_addr     = NULL;                                                      //Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;                                        //允許來自任何設備的掃描請求和連接請求
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;                              //廣告間隔
    m_adv_params.duration        = 0;                                                         //延遲時間

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


static void advertising_update(void)
{
    uint32_t                 err_code;
    ble_advdata_t            advdata;
    uint8_t                  flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    ble_advdata_manuf_data_t manuf_specific_data;
    
    uint16_t        ID ,cm, tag_ID1 = 0, tag_ID2;
    uint16_t        Info_ID[3], Info_cm[3], AT_index = 0, cmd[30];
    uint16_t        Info_cm1[3], Info_cm2[3];

    memset(&advdata, 0, sizeof(advdata));
    memset(&manuf_specific_data, 0, sizeof(manuf_specific_data));

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.size          = APP_BEACON_INFO_LENGTH;

    if(!update_request){
        manuf_specific_data.data.p_data = update_beacon_info;
        update_request = true;
    }else{
        //流水號   minor
        if(times_B == 255){
            times_A++;
            times_B = 0;
            if(times_A = 255) times_A = 0;
        }
        update2_beacon_info[20] = times_A;
        update2_beacon_info[21] = times_B;
        manuf_specific_data.data.p_data = update2_beacon_info;
        update_request = false;
    }
    
    int s = 0;
    //將字串轉為數字
    if(cmd[3] == ':') s = 4;
    if(cmd[4] == ':') s = 5;
    
    for(int i = 2 ; i < s ; i++) {
        if((cmd[i] >= '0')&&(cmd[i] <= '9')) {
              ID = ID * 10 + cmd[i] - '0';        //抓取  ID及距離，並更新
        }
    }
    for(int i = s ; i < s + 5 ; i++) {
        if((cmd[i] >= '0')&&(cmd[i] <= '9')) {
              cm = cm * 10 + cmd[i] - '0';        //抓取  ID及距離，並更新
        }
    }

    //判斷距離並調整為兩個字
    Info_ID[ID_Index] = ID;

    Info_cm1[ID_Index] = cm / 256;
    Info_cm2[ID_Index] = cm % 256;

    cmd[0] = '\0', cmd[1] = '\0';

    ID_Index++;
    if(ID_Index == 3) {
        //若是相同就刪除
        for(int i = 0 ; i < 3 ; i++) {
             for(int j = 0 ; j < i ; j++) {
                if(Info_ID[i] == Info_ID[j] && i != j) { 
                    Info_ID[i] = 0;
                    Info_cm[i] = 0;
                }
            }
        }
    }
    //UUID自訂義用來辨識
    update2_beacon_info[2] = 0xdb, update2_beacon_info[3] = 0x15, update2_beacon_info[4] = 0x17,update2_beacon_info[5] = 0x14, update2_beacon_info[6] = 0x28, update2_beacon_info[7] = 0xFF;
    //UUID的　　ID、 距離
    update2_beacon_info[8]  = 0x00;
    update2_beacon_info[9]  = Info_ID[0], update2_beacon_info[10] = Info_cm1[0], update2_beacon_info[11] = Info_cm2[0];
    update2_beacon_info[12] = Info_ID[1], update2_beacon_info[13] = Info_cm1[1], update2_beacon_info[14] = Info_cm2[1], 
    update2_beacon_info[15] = Info_ID[2], update2_beacon_info[16] = Info_cm1[2], update2_beacon_info[17] = Info_cm2[2];
    //tx power
    update2_beacon_info[22] = 0xC5;

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;
    
    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)                   //false代表  Buffer沒資料
    {
        nrf_pwr_mgmt_run();
    }
}

int AT_command_judge(uint8_t *print_value, uint16_t timeout)
{
    uint8_t        cr1, cmd1[30], cmdIndex = 0;
    cmd1[0] = 0, cmd1[1] = 0;
    while((cmd1[0] != "O") && (cmd1[1] != "K")){
        NRF_LOG_INFO("%s\r\n", print_value);
        nrf_delay_ms(timeout);
        if(app_uart_get(&cr1) == NRF_SUCCESS){
            cmd1[cmdIndex++] = cr1;
            cmd1[cmdIndex]   = 0;
            if(cr1 == "\n"){
                cmdIndex = 0;
            }
        }
    }
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint8_t        cr, i = 0;        
    uint32_t       per_distance;
    uint16_t       ID ,cm, tag_ID1 = 0, tag_ID2;
    uint8_t        Info_ID[3], Info_cm[3], AT_index = 0, cmd[30];
    uint16_t       times1 = 0, times2 = 0;
    // Initialize.
    uart_init();
    log_init();
    timers_init();
    leds_init();
    power_management_init();
    ble_stack_init();
    // Start execution.

    advertising_init();
    advertising_start();

    nrf_gpio_cfg_output(3);
    nrf_gpio_cfg_output(30);

    NRF_LOG_INFO("SYSTEM on");

    nrf_delay_ms(2000);
    NRF_LOG_INFO("AT+anchor_tag=0,1\r\n");
    nrf_delay_ms(500);
    NRF_LOG_INFO("AT+interval=10\r\n");
    nrf_delay_ms(500);
    NRF_LOG_INFO("AT+RST\r\n");
    nrf_delay_ms(800);
    NRF_LOG_INFO("AT+switchdis=1\r\n");
    nrf_delay_ms(2000);

    nrf_gpio_pin_clear(3);//PMOS關閉
    NRF_LOG_INFO("bu01 init");

    uint16_t n = 4590;
    char hex_val[50];
    sprintf(hex_val, "%x", n);
    NRF_LOG_INFO("%s\r\n",hex_val);

    

    // Enter main loop.
    for (;;) {
        nrf_delay_ms(1000);
        times1++;
         if(times1 >= 10){
            times1 = 0;
            times2++;
            if(times2 >= 2){
                times2 = 0;
                nrf_gpio_pin_clear(3);//PMOS開啟
                nrf_delay_ms(500);
                nrf_gpio_pin_clear(30);//RST腳位 1
                nrf_delay_ms(500);
                nrf_gpio_pin_set(30);
                nrf_delay_ms(2000);
                NRF_LOG_INFO("BU01 RST");
                AT_command_judge("AT+switchdis=1\r\n", 2000);
                NRF_LOG_INFO("PMOS ON");
            }else{
                AT_command_judge("AT+switchdis=0\r\n", 2000);
                nrf_gpio_pin_set(3);//PMOS關閉
                nrf_delay_ms(500);
                NRF_LOG_INFO("PMOS OFF");
            }
        }        switch (app_uart_get(&cr)) {
            case NRF_SUCCESS:
                cmd[cmdIndex++] = cr;
                cmd[cmdIndex]   = 0;
                ID = 0, cm = 0;
                if(cr == '\n') {
                    NRF_LOG_INFO("%s\n", cmd);
                    if((cmd[7] == 'T') && (cmd[8] == 'A') && (cmd[9] == 'G')){           //判斷  TAG_ID編號
                        tag_ID1 = 0, cmd[7]= 'A';
                        //將字串轉為數字
                        for(int i = 14 ; i < 17; i++) {
                            if((cmd[i] >= '0')&&(cmd[i] <= '9')) {
                                  tag_ID1 = tag_ID1 * 10 + cmd[i] - '0';
                            }
                        }
                        if(tag_ID1 >= 255){
                            tag_ID2 = tag_ID1 / 255;
                            tag_ID1 = tag_ID1 % 255;
                        }
                        //set up major
                        update2_beacon_info[18] = tag_ID2   , update2_beacon_info[19] = tag_ID1;
                        NRF_LOG_INFO("major: %x %x\n", tag_ID2, tag_ID1);
                        times_B++;  
                    }
                    if((cmd[0] == 'a') && (cmd[1] == 'n')) {
                        int s = 0;
                        //將字串轉為數字
                        if(cmd[3] == ':') s = 4;
                        if(cmd[4] == ':') s = 5;
                        
                        for(int i = 2 ; i < s ; i++) {
                            if((cmd[i] >= '0')&&(cmd[i] <= '9')) {
                                  ID = ID * 10 + cmd[i] - '0';        //抓取  ID及距離，並更新
                            }
                        }
                        for(int i = s ; i < 10 ; i++) {
                            if((cmd[i] >= '0')&&(cmd[i] <= '9')) {
                                  cm = cm * 10 + cmd[i] - '0';        //抓取  ID及距離，並更新
                            }
                        }
  
                        //判斷距離並調整為兩個字
                        if(cm > 1590) cm = 1590;
                        Info_ID[ID_Index] = ID;
                        Info_cm[ID_Index] = (cm / 100) * 16 + (cm % 100) / 6;
                        if((cm % 100) % 6 >= 3 && (cm % 100) < 90) Info_cm[ID_Index]++;

                        NRF_LOG_INFO("ALL:   %s",cmd);
                        NRF_LOG_INFO("Hex distance:  %d  %x\n", Info_ID[ID_Index], Info_cm[ID_Index]);
                        cmd[0] = '\0', cmd[1] = '\0';
                        ID_Index++;
                        if(ID_Index == 3) {
                            //若是相同就刪除
                            for(int i = 0 ; i < 3 ; i++) {
                                 for(int j = 0 ; j < i ; j++) {
                                    if(Info_ID[i] == Info_ID[j] && i != j) { 
                                        Info_ID[i] = 0;
                                        Info_cm[i] = 0;
                                    }
                                }
                            }

                            //UUID自訂義用來辨識
                            update2_beacon_info[2] = 0xdb, update2_beacon_info[3] = 0x15, update2_beacon_info[4] = 0x17,update2_beacon_info[5] = 0x14, update2_beacon_info[6] = 0x28, update2_beacon_info[7] = 0xFF;
                            //UUID的　　ID、 距離
                            update2_beacon_info[8]  = 0x00      , update2_beacon_info[9]  = 0x00;
                            update2_beacon_info[10] = 0x00      , update2_beacon_info[11] = 0x00;
                            update2_beacon_info[12] = Info_ID[0], update2_beacon_info[13] = Info_cm[0];
                            update2_beacon_info[14] = Info_ID[1], update2_beacon_info[15] = Info_cm[1];
                            update2_beacon_info[16] = Info_ID[2], update2_beacon_info[17] = Info_cm[2];
                            //tx power
                            update2_beacon_info[22] = 0xC5;

                            printf("Recieve Data: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n", APP_DEVICE_TYPE, APP_ADV_DATA_LENGTH, 0xdb, 0x15, 0x17, 0x14, 0x28, 0xFF, 0x00, 0x00, 0x00, 0x00, Info_ID[0], Info_cm[0], Info_ID[1], Info_cm[1], Info_ID[2], Info_cm[2], tag_ID2, tag_ID1, times_A, times_B, 0xC5);
                            printf("update finish\n");
                            times_B++;
                            ID_Index = 0;
                        }
                    }
                    cmdIndex = 0;
                }
                
                break;
            default:
                advertising_update();
                break;
        }
        
        idle_state_handle();
    }
}

