
/**
  Arduino IDE版本为 1.8.19
  esp32开发板 版本为 1.0.6
  串口打印波特率为 9600
*/

#include <WiFi.h>                                                                                 // WIFI库
#include <HTTPClient.h>                                                                           // http 请求
#include <SoftwareSerial.h>                                                                       // 软串口库
#include <SHT3x.h>                                                                                // 导入SHT30传感器库    
#include <TinyGPS++.h>                                                                            // GPS库
#include <math.h>                                                                                 // 字符解析
#include <driver/i2s.h>                                                                           // 设置I2S引脚
#include "sos-iir-filter.h"                                                                       // 引入
#include <Wire.h>                                                                                 // I2C协议库
#include <Adafruit_GFX.h>                                                                         // 图像库
#include <Adafruit_SSD1306.h>                                                                     // 屏幕库

// --------------------------------------------------- Debug ------------------------------------------------------
#define debugRx             3                                                                     // 定义软串口 RX 
#define debugTx             1                                                                     // 定义软串口 TX
#define debugBaud           9600                                                                  // Debug波特率
SoftwareSerial DebugSerial;                                                                       // 实例化
// ----------------------------------------------------------------------------------------------------------------

// --------------------------------------------------- OLED ------------------------------------------------------
#define SCREEN_WIDTH        128                                                                   // OLED display width, in pixels
#define SCREEN_HEIGHT       64                                                                    // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// ----------------------------------------------------------------------------------------------------------------

// ---------------------------------------------------- 声音 -------------------------------------------------------
#define LEQ_PERIOD          1                                                                     // second(s)
#define WEIGHTING           C_weighting                                                           // Also avaliable: 'C_weighting' or 'None' (Z_weighting)
#define LEQ_UNITS           "LAeq"                                                                // customize based on above weighting used
#define DB_UNITS            "dBA"                                                                 // customize based on above weighting used
#define USE_DISPLAY         0
// NOTE: Some microphones require at least DC-Blocker filter
#define MIC_EQUALIZER       ICS43434                                                              // See below for defined IIR filters or set to 'None' to disable
#define MIC_OFFSET_DB       3.0103                                                                // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration
// Customize these values from microphone datasheet
#define MIC_SENSITIVITY     -26                                                                   // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB          94.0                                                                  // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB     116.0                                                                 // dB - Acoustic overload point
#define MIC_NOISE_DB        29                                                                    // dB - Noise floor
#define MIC_BITS            24                                                                    // valid number of bits in I2S data
#define MIC_CONVERT(s)      (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT    0                                                                     // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY) / 20) * ((1 << (MIC_BITS - 1)) - 1);
#define I2S_WS              15
#define I2S_SCK             14
#define I2S_SD              32
#define I2S_PORT            I2S_NUM_0
SOS_IIR_Filter DC_BLOCKER = {
  gain: 1.0,
sos: {{ -1.0, 0.0, +0.9992, 0}}
};
SOS_IIR_Filter ICS43434 = {
  gain: 0.477326418836803,
sos: { // Second-Order Sections {b1, b2, -a1, -a2}
    { +0.96986791463971267, 0.23515976355743193, -0.06681948004769928, -0.00111521990688128},
    { -1.98905931743624453, 0.98908924206960169, +1.99755331853906037, -0.99755481510122113}
  }
};
SOS_IIR_Filter ICS43432 = {
gain: -0.457337023383413,
sos: { // Second-Order Sections {b1, b2, -a1, -a2}
    { -0.544047931916859, -0.248361759321800, +0.403298891662298, -0.207346186351843},
    { -1.909911869441421, +0.910830292683527, +1.790285722826743, -0.804085812369134},
    { +0.000000000000000, +0.000000000000000, +1.148493493802252, -0.150599527756651}
  }
};
SOS_IIR_Filter INMP441 = {
  gain: 1.00197834654696,
sos: { // Second-Order Sections {b1, b2, -a1, -a2}
    { -1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}
  }
};
SOS_IIR_Filter SPH0645LM4H_B_RB = {
  gain: 1.00123377961525,
sos: { // Second-Order Sections {b1, b2, -a1, -a2}
    { -1.0, 0.0, +0.9992, 0}, // DC blocker, a1 = -0.9992
    { -1.988897663539382, +0.988928479008099, +1.993853376183491, -0.993862821429572}
  }
};
SOS_IIR_Filter A_weighting = {
  gain: 0.169994948147430,
sos: { // Second-Order Sections {b1, b2, -a1, -a2}
    { -2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
    { +4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
    { -0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}
  }
};
SOS_IIR_Filter C_weighting = {
gain: -0.491647169337140,
sos: {
    { +1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883},
    { +0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559},
    { -2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430}
  }
};
// Sampling
#define SAMPLE_RATE         48000                                                                 // Hz, fixed to design of IIR filters
#define SAMPLE_BITS         32                                                                    // bits
#define SAMPLE_T            int32_t
#define SAMPLES_SHORT       (SAMPLE_RATE / 8)                                                     // ~125ms
#define SAMPLES_LEQ         (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE       (SAMPLES_SHORT / 16)
#define DMA_BANKS           32
struct sum_queue_t {
  float sum_sqr_SPL;
  float sum_sqr_weighted;
  uint32_t proc_ticks;
};
QueueHandle_t samples_queue;
// Static buffer for block of samples
float samples[SAMPLES_SHORT] __attribute__((aligned(4)));
// I2S Microphone sampling setup
void mic_i2s_init() {
  const i2s_config_t i2s_config = {
mode: i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
sample_rate: SAMPLE_RATE,
bits_per_sample: i2s_bits_per_sample_t(SAMPLE_BITS),
channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,
communication_format: i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
dma_buf_count: DMA_BANKS,
dma_buf_len: DMA_BANK_SIZE,
use_apll: true,
tx_desc_auto_clear: false,
    fixed_mclk: 0
  };
  const i2s_pin_config_t pin_config = {
bck_io_num:   I2S_SCK,
ws_io_num:    I2S_WS,
data_out_num: -1,                                                                                 // not used
data_in_num:  I2S_SD
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
#if (MIC_TIMING_SHIFT > 0)
  REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));
  REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);
#endif
  i2s_set_pin(I2S_PORT, &pin_config);
}
#define I2S_TASK_PRI   4
#define I2S_TASK_STACK 2048
void mic_i2s_reader_task(void* parameter) {
  mic_i2s_init();
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);
  while (true) {
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);
    TickType_t start_tick = xTaskGetTickCount();
    SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
    for (int i = 0; i < SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);
    sum_queue_t q;
    q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);
    q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);
    q.proc_ticks = xTaskGetTickCount() - start_tick;
    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}

sum_queue_t q;
uint32_t Leq_samples = 0;
double Leq_sum_sqr = 0;
double Leq_dB = 0;
// ----------------------------------------------------------------------------------------------------------------

// --------------------------------------------------- GPS --------------------------------------------------------
#define RXD0                26                                                                    // 硬串口0 RX
#define TXD0                27                                                                    // 硬串口0 TX
#define GPSBaud             9600                                                                  // GPS波特率
#define GPSSerial           Serial                                                                // 定义串口
TinyGPSPlus gps;                                                                                  // 实例化GPS
double Lat;                                                                                       // 纬度
double Lon;                                                                                       // 经度
// ----------------------------------------------------------------------------------------------------------------

// -------------------------------------------------- SHT30 -------------------------------------------------------
SHT3x Sensor;                                                                                     // 实例化对象
#define getSHTInterval      500                                                                   // 获取数据间隔时间（1秒 = 1000毫秒）
unsigned long getSHTData = 0;                                                                     // 记录存储数据时间
float shtTemperature = 0;                                                                         // 存储获取温度
float shtHumidity = 0;                                                                            // 存储获取湿度度
// ----------------------------------------------------------------------------------------------------------------

// -------------------------------------------------- PM2.5 -------------------------------------------------------
#define PMSerial            Serial1                                                               // 串口1别名
#define RXD1                25                                                                    // 25引脚连接模块RX引脚
#define TXD1                33                                                                    // 33引脚连接模块TX引脚
#define dataOne             0x42                                                                  // 固定头1
#define dataTwo             0x4d                                                                  // 固定头2
unsigned char PMDataValue[32] = {};                                                               // 接收32位数据
int Pm_Value = 0;                                                                                 // 记录PM2.5浓度值
int PM_10Value = 0;                                                                               // 记录PM10浓度值
int PM_1Value = 0;                                                                                // 记录PM1浓度值
// ----------------------------------------------------------------------------------------------------------------

// ------------------------------------------------- 4G模块 --------------------------------------------------------
#define ATObject            Serial2                                                               // AT指令发送对象
#define ATHandshake         "AT\r\n"                                                              // 握手测试
#define ATIsSIMInserted     "AT+CPIN?\r\n"                                                        // 检测 SIM 卡是否插入
#define ATQuerySignal       "AT+CSQ\r\n"                                                          // 查询信号强度
#define ATQueryNetworkReg   "AT+CEREG?\r\n"                                                       // 查询网络的注册状态
#define ATRegisterStatus    "AT+CGATT?\r\n"                                                       // 查询网络附着状态
#define ATQueryIP           "AT+CGPADDR\r\n"                                                      // 显示PDP上下文获得的IP地址
#define ATHTTPON            "AT+HTTPINIT\r\n"                                                     // 开启HTTP服务

#define ATSetURL            "AT+HTTPPARA=\"URL\",\"http://195.35.48.12:8080/data/data/saveDataList\"\r\n"       // 设置URL地址
#define ATSetHead           "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n"                    // 设置JSON格式头文件 

#define ATSendPOST          "AT+HTTPACTION=1\r\n"                                                 // 发送HTTP POST请求
#define ATGetHead           "AT+HTTPHEAD\r\n"                                                     // 读取HTTP响应头
#define ATGetData           "AT+HTTPREAD=0,100\r\n"                                               // 读取 HTTP 响应信息
#define ATHTTPOFF           "AT+HTTPTERM\r\n"                                                     // 结束 HTTP 服务
// ----------------------------------------------------------------------------------------------------------------

// -------------------------------------------------- WIFI --------------------------------------------------------
//const char* ssid = "Goldfish4Tech-2.4G00";                                                          // WiFi名称
//const char* password = "Arduino_Maker";                                                             // WiFi密码

const char* ssid = "mengci";                                                                      // WiFi名称
const char* password = "12345678";                                                                // WiFi密码

const char* serverName = "http://195.35.48.12:8080/data/data/saveDataList";                       // WiFi请求链接
WiFiClient client;                                                                                // WiFi请求实例化
HTTPClient http;                                                                                  // http请求
// ----------------------------------------------------------------------------------------------------------------

// -------------------------------------------------- 按钮 --------------------------------------------------------
#define debounceDelay       20                                                                    // 按钮保持时间
#define buttonPin           35                                                                    // 设置按键引脚
bool buttonFlag = HIGH;                                                                           // 按键状态
bool lastButtonFlag = HIGH;                                                                       // 上一次按键
unsigned long lastDebounceTime = 0;                                                               // 按键按下去的初始时间
// ----------------------------------------------------------------------------------------------------------------

// ------------------------------------------------- 系统参数 ------------------------------------------------------
String account = "CEDAR04";                                                                       // 账户

#define WiFiDelayTime       (30 * 1000UL)                                                         // WiFi等待时间
unsigned long systemRunTime = 0;                                                                  // 记录系统阶段运行时间
bool runModel = false;                                                                            // 运行模式（false为WiFi true为4G模块）

// 数据打印 ------------
#define printIntervalTime   1000                                                                  // 打印间隔时间
unsigned long printTime = 0;                                                                      // 记录打印时间

// 数据显示 ------------
#define updateIntervalTime  1000                                                                  // 数据显示间隔时间
unsigned long updateTime = 0;                                                                     // 记录更新显示时间
int runShowSubscript = 0;                                                                         // 屏幕运行切换页面

// 数据上传 ------------
#define uploadIntervalTime  (10 * 1000UL)                                                         // 数据上传间隔时间
unsigned long uploadDataTime = 0;                                                                 // 记录上传时间

// ----------------------------------------------------------------------------------------------------------------

void setup() {
  // ------------------- Deubug --------------------
  DebugSerial.begin(debugBaud, SWSERIAL_8N1, debugRx, debugTx, false);                            // 设置Debug软串口调试
  // -------------------- 屏幕 --------------------
  while (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {                                            // Address 0x3D for 128x64
    DebugSerial.println(F("Screen recognition failed!"));
    delay(500);
  }
  display.clearDisplay();                                                                         // 清除屏幕
  display.setTextSize(1);                                                                         // 设置字体大小
  display.setTextColor(WHITE);                                                                    // 设置颜色
  display.setCursor(0, 20);                                                                       // 设置位置
  display.println("Network connection...");                                                       // 设置显示文字
  display.display();                                                                              // 更新显示
  // --------------------- 声音 ---------------------
  setCpuFrequencyMhz(80);                                                                         // It should run as low as 80MHz
  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, NULL);
  // --------------------- GPS ---------------------
  GPSSerial.begin(GPSBaud, SERIAL_8N1, RXD0, TXD0);                                               // gps通讯
  // --------------------- 按钮 ---------------------
  pinMode(buttonPin, INPUT);                                                                      // 设置按键模式为输入
  // --------------------- SHT30 -------------------
  Sensor.Begin();                                                                                 // 传感器连接
  // --------------------- PM2.5 -------------------
  PMSerial.begin(9600, SERIAL_8N1, RXD1, TXD1);                                                   // 设置串口1 波特率9600
  // --------------------- 4G模块 -------------------
  ATObject.begin(115200);                                                                         // 设置串口2 波特率115200
  // --------------------- 网络连接 -------------------
  WiFi.begin(ssid, password);                                                                     // 连接WiFi
  systemRunTime = millis();                                                                       // 记录当前阶段时间
  bool tempRunState = true;                                                                       // 临时变量
  DebugSerial.println("WIFI Connecting...");
  while (tempRunState == true) {                                                                  // 判定设定时间
    // WiFi连接
    if (WiFi.status() == WL_CONNECTED) {                                                          // 判断WiFi是否连接
      tempRunState = false;                                                                       // 状态关闭
      DebugSerial.println("WIFI Connecting OK");
    }

    // 判断是否超时
    if (millis() - systemRunTime >= WiFiDelayTime) {                                              // 判断超时时间
      tempRunState = false;                                                                       // 状态关闭
      runModel = true;                                                                            // 时间超时 则开启4G
    }
  }

  // 4G模式
  if (runModel == true) {                                                                         // 如果WiFi超时
    while (ATObject.available() > 0) {                                                            // 判断是否有初始化数据情况
      ATObject.read();                                                                            // 全部读掉
    }
    InitializeConnection();                                                                       // 初始化连接
  }

  // 启动打印提示
  DebugSerial.println("Started successfully");                                                    // 打印启动打印提示
}

void loop() {
  // ------------------ 数据获取 -------------------
  getDBData();                                                                                    // 获取声音数据
  getGPSData();                                                                                   // 获取GPS数据
  getSHT30Data();                                                                                 // 获取SHT30数据
  getPmData();                                                                                    // 获取空气Pm数据
  getbutton();                                                                                    // 获取按钮状态

  // ------------------ 数据打印 -------------------
  if (millis() - printTime >= printIntervalTime) {                                                // 判断间隔时间进行打印数据
    printTime = millis();                                                                         // 记录阶段时间
    DebugSerial.print(F("Sound(dB): "));
    DebugSerial.println(Leq_dB);
    DebugSerial.print(F("Lon: "));
    DebugSerial.println(Lon, 4);
    DebugSerial.print(F("Lat: "));
    DebugSerial.println(Lat, 4);
    DebugSerial.print("Temperature(℃): ");
    DebugSerial.println(shtTemperature);
    DebugSerial.print("Humidity(%): ");
    DebugSerial.println(shtHumidity);
    DebugSerial.print(F("Pm2.5: "));
    DebugSerial.println(Pm_Value);
    DebugSerial.print(F("Pm10: "));
    DebugSerial.println(PM_10Value);
    DebugSerial.print(F("Pm1: "));
    DebugSerial.println(PM_1Value);
    DebugSerial.println();
  }

  // ------------------ 数据显示 -------------------
  setShowData();                                                                                  // 设置屏幕显示

  // ------------------ 数据上传 -------------------
  if (millis() - uploadDataTime >= uploadIntervalTime) {                                          // 判断间隔时间
    // 屏幕更新一下显示
    setUploadStatus(0);
    if (runModel == true) {                                                                       // 4G模块上传       模式
      dataSendServe();
    } else {                                                                                      // WiFi网络上传     模式
      if (WiFi.status() == WL_CONNECTED) {                                                        // 判断WiFi状态
        http.begin(client, serverName);                                                           // 设置连接
        http.addHeader("Content-Type", "application/json");                                       // 设置格式
        String setSendJson = "{\"dataDtoList\":[{\"account\":\"" + account +
                             "\",\"date\":\"\",\"humid\":\"" + String(shtHumidity) +
                             "\",\"latitude\":\"" + String(Lat, 4) +
                             "\",\"longitude\":\"" + String(Lon, 4) +
                             "\",\"pm1\":\"" + String(Pm_Value) +
                             "\",\"pm2\":\"" + String(PM_10Value) +
                             "\",\"pm3\":\"" + String(PM_1Value) +
                             "\",\"sound\":\"" + String(Leq_dB) +
                             "\",\"temp\":\"" + String(shtTemperature) +
                             "\",\"time\":\"\"}]}";

        int httpResponseCode = http.POST(setSendJson);                                            // 发送post请求
        DebugSerial.print("HTTP Response code: ");                                                // 打印提示
        DebugSerial.println(httpResponseCode);                                                    // 打印提示
        http.end();                                                                               // 关闭请求                                                               // 记录阶段时间
      } else {
        DebugSerial.println("--- WiFi Disconnected ---");
        esp_restart();                                                                            // WIFI断网直接重启
      }
    }
    // 屏幕更新一下显示
    setUploadStatus(1);
    updateTime = millis();                                                                        // 记录屏幕更新显示时间

    uploadDataTime = millis();                                                                    // 记录时间
  }

  // loop回括号
}

// 获取声音数据
void getDBData() {
  if (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {
    double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);
    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }
    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_samples += SAMPLES_SHORT;
    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;
      if (Leq_dB <= 150) {
        //Serial.printf("%.1f\n", Leq_dB);
      } else {
        Leq_dB = 150;
      }
    }
    //DebugSerial.print(F("Sound(dB): "));
    //DebugSerial.println(Leq_dB);
  }
}

// 获取GPS数据
void getGPSData() {
  if (GPSSerial.available() > 0) {                                                                // 如果串口有数据
    while (GPSSerial.available() > 0) {                                                           // 循环读取数据
      gps.encode(GPSSerial.read());                                                               // 读取数据
    }
    Lat = gps.location.lat();                                                                     // 存储经度
    Lon = gps.location.lng();                                                                     // 存储纬度
  }
}

// 获取SHT30数据
void getSHT30Data() {
  if (millis() - getSHTData >= getSHTInterval) {                                                  // 判断时间是否满足
    getSHTData = millis();                                                                        // 记录当前时间
    Sensor.UpdateData();                                                                          // 获取传感器数据
    shtTemperature = Sensor.GetTemperature();                                                     // 存储温度
    shtHumidity = Sensor.GetRelHumidity();                                                        // 存储湿度
  }
}

// 获取空气数据
void getPmData() {
  if (PMSerial.available() > 0) {                                                                 // 如果PM2.5串口有数据
    char tempChar = char(PMSerial.read());                                                        // 首字节数据读取
    delay(2);                                                                                     // 延迟2毫秒
    if (tempChar == dataOne) {                                                                    // 判断固定帧头
      PMDataValue[0] = tempChar;                                                                  // 数据存储
      tempChar = char(PMSerial.read());                                                           // 读取字节
      delay(2);                                                                                   // 延迟2毫秒
      if (tempChar == dataTwo) {                                                                  // 判断固定帧头
        PMDataValue[1] = tempChar;
        int tempPmValue = 2;                                                                      // 存储下标
        while (PMSerial.available() > 0) {
          tempChar = char(PMSerial.read());                                                       // 依次读取字节存储
          delay(2);
          PMDataValue[tempPmValue] = tempChar;
          if (++tempPmValue >= 32) {
            tempPmValue = 31;
          }
        }
      }
    }
    //                     高                低
    PM_1Value = PMDataValue[10] << 8 | PMDataValue[11];                                           // 获取PM1浓度
    Pm_Value = PMDataValue[12] << 8 | PMDataValue[13];                                            // 获取PM2.5浓度
    PM_10Value = PMDataValue[14] << 8 | PMDataValue[15];                                          // 获取PM10浓度
  }
}

// 获取按钮状态
void getbutton() {
  int reading = digitalRead(buttonPin);                                                           // 起始状态为1（高电平）
  if (reading != lastButtonFlag) {                                                                // 当状态发生改变，给时间赋值
    lastDebounceTime = millis();                                                                  // 并记录时间
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {                                            // 判断时间
    if (reading != buttonFlag) {                                                                  // 当状态发生改变
      buttonFlag = reading;                                                                       // 赋值给buttonFlag
      if (buttonFlag == LOW) {                                                                    // 判断状态
        Serial.println("button true");
        if (++runShowSubscript >= 3) {                                                            // 状态变化
          runShowSubscript = 0;                                                                   // 重置为0
        }
        updateTime = millis() - updateIntervalTime;                                               // 更新时间
      }
    }
  }
  lastButtonFlag = reading;                                                                       // 存储状态
}

// 设置屏幕数据显示
void setShowData() {
  if (millis() - updateTime >= updateIntervalTime) {                                              // 判断间隔时间
    updateTime = millis();                                                                        // 记录阶段时间
    display.clearDisplay();                                                                       // 清除屏幕
    display.setTextSize(1);                                                                       // 设置字体大小
    display.setTextColor(WHITE);                                                                  // 设置颜色
    // 显示模式状态
    if (runModel == true) {                   // 4G 模式
      display.setCursor(0, 0);
      display.println("4G");
    } else {                                  // WIFI 模式
      display.setCursor(0, 0);
      display.println("WIFI");
    }
    // 对应屏幕显示状态
    if (runShowSubscript == 0) {
      display.setCursor(0, 15);
      display.println("Temp(C): " + String(shtTemperature));
      display.setCursor(0, 30);
      display.println("Hum(%): " + String(shtHumidity));
    } else if (runShowSubscript == 1) {
      display.setCursor(0, 15);
      display.println("Pm2.5: " + String(Pm_Value));
      display.setCursor(0, 30);
      display.println("Pm10: " + String(PM_10Value));
      display.setCursor(0, 45);
      display.println("Pm1: " + String(PM_1Value));
    } else if (runShowSubscript == 2) {
      display.setCursor(0, 15);
      display.println("Sound(db): " + String(Leq_dB));
    }
    
    //if (++runShowSubscript >= 3) {
    //  runShowSubscript = 0;
    //}
    display.display();                                                                            // 更新显示
  }
}

// 设置屏幕显示数据上传状态
void setUploadStatus(int tempVal) {
  display.clearDisplay();                                                                       // 清除屏幕
  display.setTextSize(1);                                                                       // 设置字体大小
  display.setTextColor(WHITE);                                                                  // 设置颜色
  // 显示模式状态
  display.setCursor(0, 0);
  display.println(runModel == true ? "4G" : "WIFI");
  display.setCursor(0, 20);
  display.println(tempVal == 0 ? "data uploading" : "data uploaded");
  display.display();                                                                            // 更新显示
}

// 连接初始化
void InitializeConnection() {
  bool setFlag = true;                // 设置状态
  uint8_t ATDataSendState = 0x00;     // 发送AT指令流程状态
  while (setFlag == true) {
    switch (ATDataSendState) {
      case 0x00: {                    // AT 握手测试
          if (setSendAT(ATHandshake, "OK", 2, 2)) {
            DebugSerial.println(F("0x00 -- OK"));
            ATDataSendState++;                        // 状态自增
          } else {
            DebugSerial.println(F("0x00 -- 握手不成功"));
          }
        } break;
      case 0x01: {                    // 检测SIM卡是否插入
          if (setSendAT(ATIsSIMInserted, "OK", 2, 2)) {
            DebugSerial.println(F("0x01 -- OK"));
            ATDataSendState++;                        // 状态自增
          } else {
            DebugSerial.println(F("0x01 -- SIM卡错误"));
          }
        } break;
      case 0x02: {                    // 查询射频信号质量
          if (setSendAT(ATQuerySignal, "OK", 2, 2)) {
            DebugSerial.println(F("0x01 -- OK"));
            ATDataSendState++;                        // 状态自增
          } else {
            DebugSerial.println(F("0x01 -- 信号检测错误"));
          }
        } break;
      case 0x03: {                    // 查询网络的注册状态
          if (setSendAT(ATQueryNetworkReg, "OK", 2, 2)) {
            DebugSerial.println(F("0x03 -- OK"));
            ATDataSendState++;                        // 状态自增
          } else {
            DebugSerial.println(F("0x03 -- 网络注册错误"));
          }
        } break;
      case 0x04: {                    // 查询网络附着状态
          if (setSendAT(ATRegisterStatus, "OK", 2, 2)) {
            DebugSerial.println(F("0x04 -- OK"));
            ATDataSendState++;                        // 状态自增
          } else {
            DebugSerial.println(F("0x04 -- 网络状态错误"));
          }
        } break;
      case 0x05: {                    // 显示PDP上下文获得的IP地址
          if (setSendAT(ATQueryIP, "OK", 2, 2)) {
            DebugSerial.println(F("0x05 -- OK"));
            ATDataSendState++;                        // 状态自增
          } else {
            DebugSerial.println(F("0x05 -- IP地址获取错误"));
          }
        } break;
      case 0x06: {                    // 开启HTTP服务
          if (setSendAT(ATHTTPON, "OK", 2, 2)) {
            DebugSerial.println(F("0x06 -- OK"));
            ATDataSendState++;                        // 状态自增
          } else {
            DebugSerial.println(F("0x06 -- 开启HTTP服务错误"));
          }
        } break;
      case 0x07: {                    // 设置URL路径
          if (setSendAT(ATSetURL, "OK", 2, 2)) {
            DebugSerial.println(F("0x07 -- OK"));
            ATDataSendState++;                        // 状态自增
          } else {
            DebugSerial.println(F("0x07 -- 设置URL地址错误"));
          }
        } break;
      case 0x08: {                    // 设置头文件
          if (setSendAT(ATSetHead, "OK", 2, 2)) {
            DebugSerial.println(F("0x08 -- OK"));
          } else {
            DebugSerial.println(F("0x08 -- 设置JSON格式头文件错误"));
          }
          setFlag = false;
        } break;
      default: break;
    }
  }
}

// 设置数据发送
void dataSendServe() {
  String setSendJson = "{\"dataDtoList\":[{\"account\":\"" + account +
                       "\",\"date\":\"\",\"humid\":\"" + String(shtHumidity) +
                       "\",\"latitude\":\"" + String(Lat, 4) +
                       "\",\"longitude\":\"" + String(Lon, 4) +
                       "\",\"pm1\":\"" + String(Pm_Value) +
                       "\",\"pm2\":\"" + String(PM_10Value) +
                       "\",\"pm3\":\"" + String(PM_1Value) +
                       "\",\"sound\":\"" + String(Leq_dB) +
                       "\",\"temp\":\"" + String(shtTemperature) +
                       "\",\"time\":\"\"}]}";
  String setPostLength = "AT+HTTPDATA=" + String(setSendJson.length()) + ",1000\r\n";               // 设置HTTP POST 数据 长度

  bool setFlag = true;                // 设置状态
  uint8_t ATDataSendState = 0x00;     // 发送AT指令流程状态
  while (setFlag == true) {
    switch (ATDataSendState) {
      case 0x00: {                    // 设置HTTP POST 数据 长度
          if (setSendAT(setPostLength, "DOWNLOAD", 2, 1)) {
            DebugSerial.println(F("0x00 -- OK"));
          } else {
            DebugSerial.println(F("0x00 -- 设置发送数据长度"));
          }
          ATDataSendState++;                        // 状态自增
        } break;
      case 0x01: {                    // 发送json数据
          if (setSendAT(setSendJson, "OK", 2, 1)) {
            DebugSerial.println(F("0x01 -- OK"));
          } else {
            DebugSerial.println(F("0x01 -- 设置发送数据"));
          }
          ATDataSendState++;                        // 状态自增
        } break;
      case 0x02: {                    // 发送HTTP POST请求
          if (setSendAT(ATSendPOST, "OK", 2, 1)) {
            DebugSerial.println(F("0x02 -- OK"));
          } else {
            DebugSerial.println(F("0x02 -- 发送HTTP POST请求"));
          }
          ATDataSendState++;                        // 状态自增
          delay(5);
        } break;
      case 0x03: {                    // 接收数据打印
          if (ATObject.available() > 0) {
            String temVal = ATObject.readString();
            DebugSerial.println(temVal);
            DebugSerial.println();
          }
          setFlag = false;
        } break;
      default: break;
    }
  }
}

// 发送at指令 传入        AT指令            握手数据         等待秒数       最多重新发送次数
bool setSendAT(String ATValue, String catchValue, float waitSeconds, int repeatCount) {
  bool successFlag = false;               // 记录返回值
  bool endFlag = false;                   // 记录小状态机是否结束
  int sendCount = 0;                      // 记录发送次数
  uint8_t ATSendState = 0x00;             // 发送AT指令状态
  unsigned long ATSendTime = 0;           // 记录每次发送数据时间
  String takeOverValue = "";              // 接收每次请求后 返回的数据信息
  waitSeconds = waitSeconds * 1000;       // 转换为毫秒
  while (!endFlag) {                      // 状态循环
    switch (ATSendState) {
      // 数据发送
      case 0x00: {
          DebugSerial.println(F("---------------------------------"));
          DebugSerial.println(F("参数: "));
          DebugSerial.print(F("发送AT: ")); DebugSerial.println(ATValue);
          DebugSerial.print(F("握手数据: ")); DebugSerial.println(catchValue);
          DebugSerial.print(F("等待秒数: ")); DebugSerial.println(waitSeconds);
          DebugSerial.print(F("重发次数: ")); DebugSerial.println(repeatCount);
          DebugSerial.println(F("---------------------------------"));
          ATObject.print(ATValue);                    // 发送AT指令
          ATSendTime = millis();                      // 记录时间
          sendCount++;                                // 记录发送次数
          ATSendState++;                              // 状态自增
        } break;
      // 发送后，数据反馈接收
      case 0x01: {
          // 在设定 毫秒 与 等待数据之间
          if (millis() - ATSendTime <= waitSeconds) {
            if (ATObject.available() > 0 && millis() - ATSendTime >= 100) {              // 判断是否有数据
              takeOverValue = ATObject.readString();                                     // 获取最长字符串为 300（readString）
              DebugSerial.println(F("-------------接收数据--------------"));
              DebugSerial.println(takeOverValue);                                        // 从主串口打开出来
              DebugSerial.println(F("---------------------------------"));
              ATSendState++;                                                             // 获取数据后到数据处理
            }
          } else {
            ATSendState = 0x03;
          }
        } break;
      // 发送成功后，退出并提示
      case 0x02: {
          if (takeOverValue.indexOf(catchValue) != -1) {
            DebugSerial.println("数据反馈成功");
            endFlag = true;
            successFlag = true;
          } else {
            DebugSerial.println("数据反馈错误");
            endFlag = true;
            successFlag = false;
          }
        } break;
      // 没有发送成功，重新发送
      case 0x03: {
          if (sendCount >= repeatCount) {
            DebugSerial.println(F("重发次数超出，false状态"));
            endFlag = true;
            successFlag = false;
          } else {
            ATSendState = 0x00;
            DebugSerial.println(F("重新发送"));
          }
        } break;
      default: break;
    }
  }
  DebugSerial.println();
  // 返回状态
  return successFlag;
}
