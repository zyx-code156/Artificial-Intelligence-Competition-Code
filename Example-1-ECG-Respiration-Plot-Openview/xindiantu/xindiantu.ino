//////////////////////////////////////////////////////////////////////////////////////////
//
//   ESP32 Library for ADS1292R Shield/Breakout
//
//   Copyright (c) 2017 ProtoCentral
//   Heartrate and respiration computation based on original code from Texas Instruments
//   
//
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   Requires g4p_control graphing library for processing.  Built on V4.1
//   Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install
//   If you have bought the breakout the connection with the ESP32 board is as follows:
//
//  |ads1292r pin label| ESP32 Connection   |Pin Function      |
//  |----------------- |:--------------------:|-----------------:|
//  | 5V              | +5V / +3.3V           |  Supply voltage  |
//  | PWDN/RESET       | GPIO 4               |  Reset           |
//  | START            | GPIO 33              |  Start Input     |
//  | DRDY             | GPIO 32              |  Data Ready Outpt|
//  | CS               | GPIO 5               |  Chip Select     |
//  | MOSI             | GPIO 23              |  Slave In        |
//  | MISO             | GPIO 19              |  Slave Out       |
//  | SCK              | GPIO 18              |  Serial Clock    |
//  | GND              | Gnd                  |  Gnd             |
//  | RXD              | TX                   |  Tx             |
//
/////////////////////////////////////////////////////////////////////////////////////////


#include "protocentralAds1292r.h"
#include "ecgRespirationAlgo.h"
#include <SPI.h>

volatile uint8_t globalHeartRate = 0;
volatile uint8_t globalRespirationRate=0;

//Pin declartion the other you need are controlled by the SPI library
const int ADS1292_DRDY_PIN = 32;
const int ADS1292_CS_PIN = 5;
const int ADS1292_START_PIN = 33;
const int ADS1292_PWDN_PIN = 4;

#define CES_CMDIF_PKT_START_1   0x0A
#define CES_CMDIF_PKT_START_2   0xFA
#define CES_CMDIF_TYPE_DATA     0x02
#define CES_CMDIF_PKT_STOP      0x0B
#define DATA_LEN                11  // 增加了数据包长度
#define ZERO                    0

volatile char DataPacket[DATA_LEN];
const char DataPacketFooter[2] = {ZERO, CES_CMDIF_PKT_STOP};
const char DataPacketHeader[5] = {CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2, DATA_LEN, ZERO, CES_CMDIF_TYPE_DATA};

int16_t ecgWaveBuff, ecgFilterout;
int16_t resWaveBuff,respFilterout;

ads1292r ADS1292R;
ecg_respiration_algorithm ECG_RESPIRATION_ALGORITHM;

// 优化后的UART数据发送函数
void sendDataThroughUART(void){
  // 重新组织数据包结构，增加校验和
  DataPacket[0] = (ecgFilterout >> 8) & 0xFF;
  DataPacket[1] = ecgFilterout & 0xFF;
  DataPacket[2] = (resWaveBuff >> 8) & 0xFF;
  DataPacket[3] = resWaveBuff & 0xFF;
  DataPacket[4] = globalRespirationRate;
  DataPacket[5] = globalHeartRate;
  
  // 添加状态指示位 (修复: 移除不存在的getStatus()调用)
  DataPacket[6] = 0; // 占位，原getStatus()不存在
  
  // 计算并添加校验和
  uint8_t checksum = 0;
  for(int i=0; i<7; i++) {
    checksum ^= DataPacket[i];
  }
  DataPacket[7] = checksum;
  
  // 保留额外空间用于未来扩展
  DataPacket[8] = 0;
  DataPacket[9] = 0;
  DataPacket[10] = 0;

  // 发送数据包头部
  Serial.write(DataPacketHeader, 5);

  // 修复: 使用const_cast移除volatile属性，并转换为const uint8_t*
  const uint8_t* dataPtr = reinterpret_cast<const uint8_t*>(const_cast<char*>(DataPacket));
  Serial.write(dataPtr, DATA_LEN);

  // 发送数据包尾部
  Serial.write(DataPacketFooter, 2);
}

void setup()
{
  delay(2000);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  //CPOL = 0, CPHA = 1
  SPI.setDataMode(SPI_MODE1);
  // 提高SPI时钟速度以匹配更高的波特率
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  pinMode(ADS1292_DRDY_PIN, INPUT);
  pinMode(ADS1292_CS_PIN, OUTPUT);
  pinMode(ADS1292_START_PIN, OUTPUT);
  pinMode(ADS1292_PWDN_PIN, OUTPUT);

  // 修改波特率为115200
  Serial.begin(115200);
  
  // 发送初始化信息
  while(!Serial) delay(10); // 等待串口初始化完成
  Serial.println("ADS1292R初始化开始...");
  
  ADS1292R.ads1292Init(ADS1292_CS_PIN,ADS1292_PWDN_PIN,ADS1292_START_PIN);
  
  // 发送初始化完成信息
  Serial.println("初始化完成，开始数据采集...");
}

void loop()
{
  ads1292OutputValues ecgRespirationValues;
  boolean ret = ADS1292R.getAds1292EcgAndRespirationSamples(ADS1292_DRDY_PIN,ADS1292_CS_PIN,&ecgRespirationValues);

  if (ret == true)
  {
    ecgWaveBuff = (int16_t)(ecgRespirationValues.sDaqVals[1] >> 8) ;  // 忽略24位中的低8位
    resWaveBuff = (int16_t)(ecgRespirationValues.sresultTempResp>>8) ;

    if(ecgRespirationValues.leadoffDetected == false)
    {
      // 处理ECG信号
      ECG_RESPIRATION_ALGORITHM.ECG_ProcessCurrSample(&ecgWaveBuff, &ecgFilterout);   // 过滤40Hz线路噪声，161阶
      ECG_RESPIRATION_ALGORITHM.QRS_Algorithm_Interface(ecgFilterout,&globalHeartRate); // 计算心率
      
      // 处理呼吸信号 (取消注释以启用)
      respFilterout = ECG_RESPIRATION_ALGORITHM.Resp_ProcessCurrSample(resWaveBuff);
      ECG_RESPIRATION_ALGORITHM.RESP_Algorithm_Interface(respFilterout,&globalRespirationRate);
    }
    else
    {
      // 检测到导联脱落时归零输出
      ecgFilterout = 0;
      respFilterout = 0;
    }

    // 发送数据
    sendDataThroughUART();
  }
}