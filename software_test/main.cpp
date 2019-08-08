#include "mbed.h"
#include "rtos.h"
#include "drivers/QSPI.h"

#define RX_BUFFER_SIZE      (128)  // N.B. Must fit into uint8_t

#define CARRIAGE_RETURN     ('\r')
#define LINE_FEED           ('\n')

#define SLEEP_TIME                  500 // (msec)
#define PRINT_AFTER_N_LOOPS         20
/* QSPI_FLASH */
#define CMD_READ           0x03
#define CMD_WRITE          0x02
#define CMD_ERASE          0x20
#define CMD_RDSR           0x5
#define CMD_WREN           0x6
#define CMD_RSTEN          0x66
#define CMD_RST            0x99
#define STATUS_REG_SIZE    2
#define BIT_WIP            0x1
#define BIT_WEL            0x2
#define BUF_SIZE           12

// hardware ssel (where applicable)
QSPI qspi_device(QSPI_FLASH1_IO0, QSPI_FLASH1_IO1, QSPI_FLASH1_IO2, QSPI_FLASH1_IO3, QSPI_FLASH1_SCK, QSPI_FLASH1_CSN); // io0, io1, io2, io3, sclk, ssel

DigitalOut      user_led(LED2);
DigitalIn       button(BUTTON1);
Serial          uart_device(USBTX, USBRX);
Thread          *RX_THREAD_POINTER;
Thread          *WIFI_THREAD_POINTER;
WiFiInterface   *wifi;
DigitalOut      trustx_reset(P6_3);

I2C i2c(I2C_SDA , I2C_SCL);


/*
 * Volatile variable declarations
 */
volatile bool     _periodComplete;
volatile char     _rxCircularBuffer[ RX_BUFFER_SIZE ];
volatile int16_t  _rxPutIndex;

/*
 * Variable declarations
 */
const int addr7bit = 0x30;      // 7-bit I2C address
const int addr8bit = 0x30 << 1; // 8-bit I2C address, 0x60

/*
 * Module function prototypes
 */
static void init_peripheral(void);
static void init_uart(void);
static void led_toggle(void);

static const char *sec2str(nsapi_security_t sec);
static int scan_demo(WiFiInterface *wifi);
static int wifi_test(void);

void Rx_interrupt(void);
void rx_thread(void const *argument);
void wifi_thread(void const *argument);

static bool SerialGetCommand(char * const s_CmdBuf, int8_t maxCmdSize);
static void ProcessCommands(void);
static uint16_t AsciiHexToValue( const char * const text, int16_t numChars );

static bool mem_ready(void);
static int write_enable(void);
static int flash_init(void);
static int sector_erase(unsigned int flash_addr);
static int qspi_flash_test(void);

static int optiga_trust_init(void);
static int optiga_trust_test(void);

/***************************************
 * Main Function
 ***************************************/
int main(){
    
    Thread t_rx(rx_thread);
    RX_THREAD_POINTER = &t_rx;  // Set thread pointer as globally-accessible 
    t_rx.set_priority(osPriorityHigh);
    Thread wifi_rx(wifi_thread);   
    WIFI_THREAD_POINTER = &wifi_rx;  // Set thread pointer as globally-accessible 
    wifi_rx.set_priority(osPriorityHigh);

    /* init on chip device */
    init_peripheral();

    uart_device.printf("\r\nStart\r\n");
    
    while(true) {
        Thread::wait(1000);
    }
     
    return 0;
}

/******************************************************************************
 * Function: Rx_interrupt
 *****************************************************************************/
// Rx Interupt routine
void Rx_interrupt(void){
    
    (*RX_THREAD_POINTER).signal_set(0x1); // dereference of RX_THREAD_POINTER

}
 
 /******************************************************************************
 * Function: rx_thread
 *****************************************************************************/
// Read received chars from UART
void rx_thread(void const *argument)
{

    while (true) {

        // Signal flags that are reported as event are automatically cleared.
        Thread::signal_wait(0x1);
        while (uart_device.readable()) {

            //   uart_device.putc(uart_device.getc());             // read data from UART
            _rxCircularBuffer[ _rxPutIndex++ ] = uart_device.getc();
            if ( _rxPutIndex > RX_BUFFER_SIZE - 1) {
                _rxPutIndex = 0;
            }

        }
        ProcessCommands();

    }
}

void wifi_thread(void const *argument)
{
    while(true) {
        // Signal flags that are reported as event are automatically cleared.
        Thread::signal_wait(0x1);

        /* wifi Connection init */
        
        uart_device.printf("\r\nwifi_thread\r\n");
        wifi_test();

    }
}

/*
 *  Function: init_peripheral()
 */
static void init_peripheral(void)
{
    led_toggle();
    init_uart();
    // optiga_trust_init();    
} 

/*
 * Function: init_uart
 */

static void init_uart(void)
{
    // Setup a serial interrupt function to receive data
    uart_device.attach(&Rx_interrupt, Serial::RxIrq);
}


/*
 * Function: led_toggle
 */
static void led_toggle(void)
{
    user_led = !user_led;
}


/******************************************************************************
 * Function: sec2str
 *****************************************************************************/
static const char *sec2str(nsapi_security_t sec)
{
    switch (sec) {
        case NSAPI_SECURITY_NONE:
            return "None";
        case NSAPI_SECURITY_WEP:
            return "WEP";
        case NSAPI_SECURITY_WPA:
            return "WPA";
        case NSAPI_SECURITY_WPA2:
            return "WPA2";
        case NSAPI_SECURITY_WPA_WPA2:
            return "WPA/WPA2";
        case NSAPI_SECURITY_UNKNOWN:
        default:
            return "Unknown";
    }
}

/******************************************************************************
 * Function: scan_demo
 *****************************************************************************/
static int scan_demo(WiFiInterface *wifi)
{
    WiFiAccessPoint *ap;

    uart_device.printf("Scan:\r\n");

    int count = wifi->scan(NULL, 0);

    if (count <= 0) {
        uart_device.printf("scan() failed with return value: %d\r\n", count);
        return 0;
    }

    /* Limit number of network arbitrary to 15 */
    count = count < 15 ? count : 15;

    ap = new WiFiAccessPoint[count];
    count = wifi->scan(ap, count);

    if (count <= 0) {
        uart_device.printf("scan() failed with return value: %d\r\n", count);
        return 0;
    }

    for (int i = 0; i < count; i++) {
        uart_device.printf("Network: %s secured: %s BSSID: %hhX:%hhX:%hhX:%hhx:%hhx:%hhx RSSI: %hhd Ch: %hhd\r\n", ap[i].get_ssid(),
               sec2str(ap[i].get_security()), ap[i].get_bssid()[0], ap[i].get_bssid()[1], ap[i].get_bssid()[2],
               ap[i].get_bssid()[3], ap[i].get_bssid()[4], ap[i].get_bssid()[5], ap[i].get_rssi(), ap[i].get_channel());
    }
    uart_device.printf("%d networks available.\r\n\n", count);

    delete[] ap;
    return count;
}

/******************************************************************************
 * Function: wifi_test
 *****************************************************************************/
static int wifi_test(void)
{
    uart_device.printf("WiFi example\r\n");
    #ifdef MBED_MAJOR_VERSION
    uart_device.printf("Mbed OS version %d.%d.%d\r\n\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
    #endif

    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
        uart_device.printf("ERROR: No WiFiInterface found.\r\n");
        return -1;
    }

    // int count = scan_demo(wifi);
    // if (count == 0) {
    //     uart_device.printf("No WIFI APs found - can't continue further.\r\n");
    //     return -1;
    // }

    uart_device.printf("\nConnecting to %s...\r\n\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
        uart_device.printf("\nConnection error: %d\r\n", ret);
        uart_device.printf("\nWiFi Connection Failed\r\n");
        return -1;
    }

    uart_device.printf("WiFi Connection Success\r\n\n");
    uart_device.printf("MAC: %s\r\n", wifi->get_mac_address());
    uart_device.printf("IP: %s\r\n", wifi->get_ip_address());
    uart_device.printf("Netmask: %s\r\n", wifi->get_netmask());
    uart_device.printf("Gateway: %s\r\n", wifi->get_gateway());
    uart_device.printf("RSSI: %d\r\n\n", wifi->get_rssi());

    wifi->disconnect();
 
    uart_device.printf("\nDone\r\n");
    // uart_device.printf("\nWiFi_Connection_Success\r\n");
    return ret;
}

/******************************************************************************
 * Function: mem_ready
 *****************************************************************************/
static bool mem_ready(void)
{
    char status_value[STATUS_REG_SIZE] = {0xFF};
    int retries = 10000;
    bool mem_ready = true;

    do {
        retries--;
        if (QSPI_STATUS_OK != qspi_device.command_transfer(CMD_RDSR, -1, NULL, 0, status_value, STATUS_REG_SIZE)) {
            printf("Reading Status Register failed \n");
        }
        wait_ms(1);
    } while ((status_value[0] & BIT_WIP) != 0 && retries);

    if ((status_value[0] & BIT_WIP) != 0) {
        printf ("mem_ready FALSE: status value = 0x%x\n", (int)status_value[0]);
        mem_ready = false;
    }
    return mem_ready;
}

/******************************************************************************
 * Function: write_enable
 *****************************************************************************/
static int write_enable(void)
{
    char status_value[STATUS_REG_SIZE] = {0};
    int status = -1;

    if (QSPI_STATUS_OK != qspi_device.command_transfer(CMD_WREN, -1, NULL, 0, NULL, 0)) {
        printf("Sending WREN command FAILED \n");
        return status;
    }

    if (false == mem_ready()) {
        printf("Device not ready \n");
        return status;
    }

    if (QSPI_STATUS_OK != qspi_device.command_transfer(CMD_RDSR, -1, NULL, 0, status_value, STATUS_REG_SIZE)) {
        printf("Reading Status Register failed \n");
        return status;
    }

    if ((status_value[0] & BIT_WEL)) {
        status = 0;
    }
    return status;
}

/******************************************************************************
 * Function: flash_init
 *****************************************************************************/
static int flash_init(void)
{
    int status = QSPI_STATUS_OK;
    char status_value[STATUS_REG_SIZE] = {0};

    // Read the Status Register from device
    status =  qspi_device.command_transfer(CMD_RDSR, -1, NULL, 0, status_value, STATUS_REG_SIZE);
    if (status != QSPI_STATUS_OK) {
        printf("Reading Status Register failed: value = 0x%x\n", (int)status_value[0]);
        return status;
    }

    // Send Reset Enable
    status = qspi_device.command_transfer(CMD_RSTEN, -1, NULL, 0, NULL, 0);
    if (status == QSPI_STATUS_OK) {
        printf("Sending RSTEN Success \n");
    } else {
        printf("Sending RSTEN failed \n");
        return status;
    }

    if (false == mem_ready()) {
        printf("Device not ready \n");
        return -1;
    }

    // Send Reset
    status = qspi_device.command_transfer(CMD_RST, -1, NULL, 0, NULL, 0);
    if (status == QSPI_STATUS_OK) {
        printf("Sending RST Success \n");
    } else {
        printf("Sending RST failed \n");
        return status;
    }

    if (false == mem_ready()) {
        printf("Device not ready \n");
        return -1;
    }
    return status;
}

/******************************************************************************
 * Function: sector_erase
 *****************************************************************************/
static int sector_erase(unsigned int flash_addr)
{
    if (0 != write_enable()) {
        printf("Write Enabe failed \n");
        return -1;
    }

    if (QSPI_STATUS_OK!= qspi_device.command_transfer(CMD_ERASE, (((int)flash_addr) & 0x00FFF000), NULL, 0, NULL, 0))
    {
        printf("Erase failed\n");
        return -1;
    }

    if (false == mem_ready()) {
        printf("Device not ready \n");
        return -1;
    }

    return 0;
}

/******************************************************************************
 * Function: qspi_flash_test
 *****************************************************************************/
static int qspi_flash_test(void) {
    char tx_buf[BUF_SIZE] = { 'h', 'e', 'l', 'l', 'o', '\0' };
    char rx_buf[BUF_SIZE] = {0};
    size_t buf_len = sizeof(tx_buf);
    qspi_status_t result;
    uint32_t address = 0x1000;

    result = qspi_device.configure_format(QSPI_CFG_BUS_SINGLE, QSPI_CFG_BUS_SINGLE,
                                        QSPI_CFG_ADDR_SIZE_24, QSPI_CFG_BUS_SINGLE,
                                        QSPI_CFG_ALT_SIZE_8, QSPI_CFG_BUS_SINGLE, 0);
    if (result != QSPI_STATUS_OK) {
        printf("Config format failed\n");
    }

    if (QSPI_STATUS_OK != flash_init()) {
        printf ("Init failed\n");
        return -1;
    }

    if (0 != sector_erase(address)) {
        return -1;
    }

    if (0 != write_enable()) {
        printf("Write Enabe failed \n");
        return -1;
    }

    result = qspi_device.write(CMD_WRITE, -1, address, tx_buf, &buf_len);
    if (result != QSPI_STATUS_OK) {
        printf("Write failed\n");
        return result;
    }
    printf("Write done: %s \n", tx_buf);

    if (false == mem_ready()) {
        printf("Device not ready \n");
        return -1;
    }

    result = qspi_device.read(CMD_READ, -1, address, rx_buf, &buf_len);
    if (result != QSPI_STATUS_OK) {
        printf("Read failed\n");
        return result;
    }

    printf ("Data Read = %s\n", rx_buf);

    if (strncmp(tx_buf, rx_buf, BUF_SIZE) == 0) {
        uart_device.printf("\nQSPI_FLASH_TEST_Success\r\n");
    }
    else {
        uart_device.printf("\nQSPI_FLASH_TEST_Failed\r\n");
    }

    return 0;
}

/******************************************************************************
 * Function: optiga_trust_init
 *****************************************************************************/
static int optiga_trust_init(void)
{
    int len = 0;
    uint8_t data_buffer[10] = {0x82, 0x00};
    uint16_t data_length = 1;

    /* POR */
    trustx_reset != trustx_reset;
    wait_ms(SLEEP_TIME);
    trustx_reset != trustx_reset;

    // read and write takes the 8-bit version of the address
    // set up configuration register (at 0x01)
    i2c.write(addr8bit, (char *)data_buffer, data_length);
    if( len != 2) {
        uart_device.printf("\r\noptiga i2c write failed\r\n");
        return len;
    }
    uart_device.printf("\r\noptiga i2c write success\r\n");

    wait(0.5);

    data_length = 4;
    len=i2c.read(addr8bit, (char *)data_buffer, 4);
    if( len != 4) {
        uart_device.printf("\r\noptiga i2c read failed\r\n");
        return len;
    }
    uart_device.printf("\r\noptiga i2c read success\r\n");

    uart_device.printf("\r\noptiga output data:0x%2X, 0x%2X, 0x%2X, 0x%2X\r\n", data_buffer[0], data_buffer[1], data_buffer[2], data_buffer[3]);

    return 0;
}

/******************************************************************************
 * Function: optiga_trust_test
 *****************************************************************************/
static int optiga_trust_test(void)
{
    int len = 0;
    uint8_t data_buffer[10] = {0x82, 0x00};
    uint16_t data_length = 1;

    // read and write takes the 8-bit version of the address
    // set up configuration register (at 0x01)
    i2c.write(addr8bit, (char *)data_buffer, data_length);
    if( len != 2) {
        uart_device.printf("\r\noptiga i2c write failed\r\n");
        return len;
    }
    uart_device.printf("\r\noptiga i2c write success\r\n");

    wait(0.5);

    data_length = 4;
    len=i2c.read(addr8bit, (char *)data_buffer, 4);
    if( len != 4) {
        uart_device.printf("\r\noptiga i2c read failed\r\n");
        return len;
    }
    uart_device.printf("\r\noptiga i2c read success\r\n");

    uart_device.printf("\r\noptiga output data:0x%2X, 0x%2X, 0x%2X, 0x%2X\r\n", data_buffer[0], data_buffer[1], data_buffer[2], data_buffer[3]);

    return 0;
}


/******************************************************************************
 * Function: SerialGetCommand
 *****************************************************************************/
static bool
SerialGetCommand(char * const szCmdBuf, int8_t maxCmdSize)
{
    static int16_t  rxCmdIndex = 0;
    uint8_t   cmdIndexTemp;
    uint8_t   nn = 0;
    bool      fCmdFound;

    /*
     * NB: Assumes _rxCircularBuffer is less than 255 bytes
     */
    if ( _rxPutIndex == rxCmdIndex ) {
        return false;
    }

    fCmdFound = false;
    cmdIndexTemp = rxCmdIndex;

    while ( _rxPutIndex != rxCmdIndex ) {
        static char  next_char; // Force to not use stack variable

        next_char = _rxCircularBuffer[ rxCmdIndex++ ];
        if ( rxCmdIndex > RX_BUFFER_SIZE - 1 ) {
            rxCmdIndex = 0;
        }

        if ( next_char == CARRIAGE_RETURN || next_char == LINE_FEED ) {
            fCmdFound = true;
            break;
        }

        if ( nn < maxCmdSize - 1) {
            szCmdBuf[nn] = next_char;
            nn++;
        }
        else {
            szCmdBuf[nn] = 0;
            return false;
        }
    }

    if (fCmdFound) {
        szCmdBuf[nn] = 0;
        return nn > 0 ? true : false;  // Ignore single CRs or LFs
    }

    rxCmdIndex = cmdIndexTemp;
    return false;
}

/*
 *************************************************************
 * Function: ProcessCommands
 *************************************************************
 */
static void
ProcessCommands(void)
{
    char      command[RX_BUFFER_SIZE - 1];
    char      send_string[10];

     while (SerialGetCommand(command, sizeof(command))) {
    
        switch(command[0]) {

            case 'A':
                led_toggle();
                uart_device.printf("LED_TOGGLE_TEST_Success\r\n");
                
                if(button.read()) {
                    uart_device.printf("BUTTON_RELEASED_TEST_Success\r\n");                
                }
                else {
                    uart_device.printf("BUTTON_PRESSED_TEST_Success\r\n");
                }
                qspi_flash_test();
                // optiga_trust_test();
                wifi_test();
                uart_device.printf("ALL_TEST_DONE_Success\r\n");
                break;

            case 'B':
                if(button.read()) {
                    uart_device.printf("BUTTON_RELEASED_TEST_Success\r\n");                
                }
                else {
                    uart_device.printf("BUTTON_PRESSED_TEST_Success\r\n");
                }
                break;

            case 'I':
                // optiga_trust_test();
                break;

            case 'L':
                led_toggle();
                uart_device.printf("LED_TOGGLE_TEST_Success\r\n");
                break;

            case 'Q':
                qspi_flash_test();                
                break;

            case 'W':
                // wifi_test();
                (*WIFI_THREAD_POINTER).signal_set(0x1); // dereference of WIFI_THREAD_POINTER
                break;
                
            default:
                uart_device.printf("COMMAND_ERROR\r\n");
                break;
        }
    }
}

/******************************************************************************
 * Function: AsciiHexToValue
 *****************************************************************************/
static uint16_t
AsciiHexToValue( const char * const text, int16_t numChars )
{
    int n;
    uint16_t value = 0;
    int shift = (numChars-1) * 4;

    for( n = 0; n < numChars; n++ )
    {
        uint16_t nibble;

        if( text[n] >= 'a' )
        {
                nibble = (uint16_t) (0x0A + text[n] - 'a');
        }
        else if( text[n] >= 'A' )
        {
                nibble = (uint16_t) (0x0A + text[n] - 'A');
        }
        else
        {
                nibble = (uint16_t) (text[n] - '0');
        }
        nibble &= 0x000F;
        value |= (nibble << shift);
        shift -= 4;
    }
    return value;
}

