#include <Arduino.h>
#include "LoRaWan-Arduino.h" //http://librarymanager/All#SX126x
#include <SPI.h>

#include <stdio.h>
#include <mutex>

#include "mbed.h"
#include "rtos.h"

#include "SparkFun_SHTC3.h"     //Click here to get the library: http://librarymanager/All#SparkFun_SHTC3

SHTC3 g_shtc3;                  // Declare an instance of the SHTC3 class
using namespace std::chrono_literals;
using namespace std::chrono;

// -------------------------- GPS --------------------------------------//
#include <TinyGPS.h>    //http://librarymanager/All#TinyGPS

TinyGPS gps;
String tmp_data = "";
int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W
// ---------------------------------------------------------------------//

bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                      /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_1                   /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5             /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3                      /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;         /* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_EU868;    /* Region:EU868*/
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;         /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;                      /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {LORAWAN_ADR_OFF, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);
void lorawan_unconf_finished(void);
void lorawan_conf_finished(bool result);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                          lorawan_rx_handler, lorawan_has_joined_handler,
                                          lorawan_confirm_class_handler, lorawan_join_failed_handler,
                                          lorawan_unconf_finished, lorawan_conf_finished
                                         };
//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x06, 0x40, 0x7D};
uint8_t nodeAppEUI[8] = {0xA9, 0x03, 0xAC, 0xB0, 0xBE, 0xEA, 0x26, 0x03};
uint8_t nodeAppKey[16] = {0x8E, 0x1D, 0x47, 0x0C, 0x31, 0xD9, 0x17, 0x72, 0x30, 0xD5, 0xCE, 0xA1, 0x39, 0x03, 0x10, 0x99};

// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 20000                        /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

mbed::Ticker appTimer;
void tx_lora_periodic_handler(void);

static uint32_t count = 0;
static uint32_t count_fail = 0;

bool send_now = false;

/*** blink ****/
// Variables will change:
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis_lora = 0;        // will store last time LED was updated
unsigned long previousMillis_send_frame = 0;        // will store last time LED was updated

// constants won't change:
const long interval_lora = 1000;           // interval at which to blink (milliseconds)
const long interval_send_frame = 20000;           // interval at which to blink (milliseconds)
/**************/

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);


  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }
  // Initialize LoRa chip.
  lora_rak11300_init();

  digitalWrite(LED_BLUE, 1);

  Serial.println("=====================================");
  Serial.println("Welcome to RAK11300 LoRaWan!!!");
  Serial.println("Type: OTAA");
  Serial.println("Region: EU868");
  Serial.println("=====================================");

  //gps init
  pinMode(WB_IO2, OUTPUT);

  digitalWrite(WB_IO2, 0);
  delay(1000);
  digitalWrite(WB_IO2, 1);
  delay(1000);
  Serial1.begin(9600);
  while (!Serial1);
  Serial.println("Gps started!");


  // Setup the EUIs and Keys
  if (doOTAA)
  {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }
  else
  {
    lmh_setNwkSKey(nodeNwsKey);
    lmh_setAppSKey(nodeAppsKey);
    lmh_setDevAddr(nodeDevAddr);
  }

  // Initialize LoRaWan
  uint32_t err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  // Start Join procedure
  lmh_join();
}



// ------------------------------------------------- GPS ------------------------------------------------- //
void direction_parse(String tmp)
{
    if (tmp.indexOf(",E,") != -1)
    {
        direction_E_W = 0;
    }
    else
    {
        direction_E_W = 1;
    }
    
    if (tmp.indexOf(",S,") != -1)
    {
        direction_S_N = 0;
    }
    else
    {
        direction_S_N = 1;
    }
}


void loop()
{
  // Every LORAWAN_APP_INTERVAL milliseconds send_now will be set
  // true by the application timer and collects and sends the data
  if (send_now)
  {
    Serial.println("Sending frame now...");
    send_now = false;
    send_lora_frame();
  }
}

/**@brief LoRa function for handling HasJoined event.
*/
void lorawan_has_joined_handler(void)
{
  if (doOTAA == true)
  {
    Serial.println("OTAA Mode, Network Joined!");
  }
  else
  {
    Serial.println("ABP Mode");
  }

  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    // Start the application timer. Time has to be in microseconds
    appTimer.attach(tx_lora_periodic_handler, (std::chrono::microseconds)(LORAWAN_APP_INTERVAL * 1000));
  }
}
/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}
/**@brief Function for handling LoRaWan received data from Gateway

   @param[in] app_data  Pointer to rx data
*/
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
                app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void lorawan_unconf_finished(void)
{
  Serial.println("TX finished");
}

void lorawan_conf_finished(bool result)
{
  Serial.printf("Confirmed TX %s\n", result ? "success" : "failed");
}

void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    return;
  }
  data_get();

  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
}

/**@brief Function for handling user timerout event.
*/
void tx_lora_periodic_handler(void)
{
  appTimer.attach(tx_lora_periodic_handler, (std::chrono::microseconds)(LORAWAN_APP_INTERVAL * 1000));
  // This is a timer interrupt, do not do lengthy things here. Signal the loop() instead
  send_now = true;
}

//---------------------------Temp Hum---------------------------

void errorDecoder(SHTC3_Status_TypeDef message)   // The errorDecoder function prints "SHTC3_Status_TypeDef" resultsin a human-friendly way
{
  switch (message)
  {
    case SHTC3_Status_Nominal:
      Serial.print("Nominal");
      break;
    case SHTC3_Status_Error:
      Serial.print("Error");
      break;
    case SHTC3_Status_CRC_Fail:
      Serial.print("CRC Fail");
      break;
    default:
      Serial.print("Unknown return code");
      break;
  }
}




String data = "";
void data_get()
{
  Serial.print("result: ");
  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  g_shtc3.update();

  float temp = g_shtc3.toDegC();
  float hum = g_shtc3.toPercent();

  data = "Tem:" + String(temp) + "C " + "Hum:" + String(hum) + "% ";
  Serial.println(data);

  uint16_t t = temp * 100;
  uint16_t h = hum * 100;


  //-------------GPS-------------------
   bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial1.available())
    {
      char c = Serial1.read();
      tmp_data += c;
      if (gps.encode(c))
        newData = true;
    }
  }
  direction_parse(tmp_data);
  tmp_data = "";

  float flat, flon;
  int32_t ilat, ilon;


  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
    
  Serial.print("LAT=");
  Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
  Serial.print(" | LON=");
  Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  Serial.printf("\n");
  
  flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat;
  ilat = flat * 100000;
  flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon;
  ilon = flon * 100000;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;



  //result: T=28.25C, RH=50.00%, P=958.57hPa, light=100.46 lux
  m_lora_app_data.buffer[0] = 0x02;
  m_lora_app_data.buffer[1] = (uint8_t)(t >> 8);
  m_lora_app_data.buffer[2] = (uint8_t)t;
  m_lora_app_data.buffer[3] = (uint8_t)(h >> 8);
  m_lora_app_data.buffer[4] = (uint8_t)h;
  m_lora_app_data.buffer[5] = (ilat & 0xFF000000) >> 24;
  m_lora_app_data.buffer[6] = (ilat & 0x00FF0000) >> 16;
  m_lora_app_data.buffer[7] = (ilat & 0x0000FF00) >> 8;
  m_lora_app_data.buffer[8] =  ilat & 0x000000FF;
  if (direction_S_N == 0)
  {
    m_lora_app_data.buffer[9] = 'S';
  }
  else
  {
    m_lora_app_data.buffer[9] = 'N';
  }
  //lon data
  m_lora_app_data.buffer[10] = (ilon & 0xFF000000) >> 24;
  m_lora_app_data.buffer[11] = (ilon & 0x00FF0000) >> 16;
  m_lora_app_data.buffer[12] = (ilon & 0x0000FF00) >> 8;
  m_lora_app_data.buffer[13] =  ilon & 0x000000FF;
  if (direction_E_W == 0)
  {
    m_lora_app_data.buffer[14] = 'E';
  }
  else
  {
    m_lora_app_data.buffer[14] = 'W';
    }
  gps.stats(&chars, &sentences, &failed);
    if (chars == 0)
      Serial.println("** No characters received from GPS: check wiring **");
  
  m_lora_app_data.buffsize = 15;
  for(int i = 0; i < m_lora_app_data.buffsize; i++)
    Serial.printf("%u\n", m_lora_app_data.buffer[i]);
}
