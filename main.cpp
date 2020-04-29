#include "mbed.h"

#include "fsl_port.h"
#include "fsl_gpio.h"
#include <vector>

#define UINT14_MAX        16383

// FXOS8700CQ I2C address

#define FXOS8700CQ_SLAVE_ADDR0 (0x1E<<1) // with pins SA0=0, SA1=0
#define FXOS8700CQ_SLAVE_ADDR1 (0x1D<<1) // with pins SA0=1, SA1=0
#define FXOS8700CQ_SLAVE_ADDR2 (0x1C<<1) // with pins SA0=0, SA1=1
#define FXOS8700CQ_SLAVE_ADDR3 (0x1F<<1) // with pins SA0=1, SA1=1

// FXOS8700CQ internal register addresses

#define FXOS8700Q_STATUS 0x00
#define FXOS8700Q_OUT_X_MSB 0x01
#define FXOS8700Q_OUT_Y_MSB 0x03
#define FXOS8700Q_OUT_Z_MSB 0x05
#define FXOS8700Q_M_OUT_X_MSB 0x33
#define FXOS8700Q_M_OUT_Y_MSB 0x35
#define FXOS8700Q_M_OUT_Z_MSB 0x37
#define FXOS8700Q_WHOAMI 0x0D
#define FXOS8700Q_XYZ_DATA_CFG 0x0E
#define FXOS8700Q_CTRL_REG1 0x2A
#define FXOS8700Q_M_CTRL_REG1 0x5B
#define FXOS8700Q_M_CTRL_REG2 0x5C
#define FXOS8700Q_WHOAMI_VAL 0xC7


I2C i2c( PTD9,PTD8);
Serial pc(USBTX, USBRX);

Timer debounce;

InterruptIn btnRecord(SW2);
InterruptIn btnFlag(SW3);
EventQueue queue(32 * EVENTS_EVENT_SIZE);

DigitalOut led1(LED1);

Thread t;


int m_addr = FXOS8700CQ_SLAVE_ADDR1;
int idR[32] = {0};
int indexR = 0;

std::vector <float> data_x;
std::vector <float> data_y;
std::vector <float> data_z;
std::vector <int> data_hori;

float acc_move = 0.0;

void FXOS8700CQ_readRegs(int addr, uint8_t * data, int len) {
   char tmp = addr;
   i2c.write(m_addr, &tmp, 1, true);
   i2c.read(m_addr, (char *)data, len);
}


void FXOS8700CQ_writeRegs(uint8_t * data, int len) {
   i2c.write(m_addr, (char *)data, len);
}


void record(void) {
   float x=0.0, y=0.0, z=0.0;
   int16_t acc16;
   uint8_t res[6];
   FXOS8700CQ_readRegs(FXOS8700Q_OUT_X_MSB, res, 6);
   acc16 = (res[0] << 6) | (res[1] >> 2);
   if (acc16 > UINT14_MAX/2)
     acc16 -= UINT14_MAX;
   x = ((float)acc16) / 4096.0f;
   acc16 = (res[2] << 6) | (res[3] >> 2);
   if (acc16 > UINT14_MAX/2)
     acc16 -= UINT14_MAX;
   y = ((float)acc16) / 4096.0f;
   acc16 = (res[4] << 6) | (res[5] >> 2);
   if (acc16 > UINT14_MAX/2)
     acc16 -= UINT14_MAX;
   z = ((float)acc16) / 4096.0f;
   //printf("%f, %f, %f\n", x * 1000.0, y * 1000.0, z * 1000.0);


   data_x.push_back(x);
   data_y.push_back(y);
   data_z.push_back(z);
   data_hori.push_back(0.5*9.8*sqrt(x*x+y*y)*0.1*0.1 > 0.05 ? 1:0);
}

void flash_LED(void){
    led1 = (led1 == 1)?0:1;
}

void stop_log(void) {
  //printf("---stop---\n");
  for (auto &i : idR)
    queue.cancel(i);
  led1 = 1;

  for(uint i=0;i<data_x.size();i++){
        //pc.printf("%d\nX:%f\nY:%f\nZ:%f\n",i,data_x[i],data_y[i],data_z[i]);
    pc.printf("%f\n%f\n%f\n%d\n",data_x[i],data_y[i],data_z[i],data_hori[i]);
  }
  acc_move = 0;
  data_x.clear();
  data_y.clear();
  data_z.clear();
}

void start_log(void) {
  //printf("log_start\n");
  idR[indexR++] = queue.call_every(100,record);
  idR[indexR++] = queue.call_every(1000,flash_LED);
  indexR = indexR % 32;
  queue.call_in(10000,stop_log);
}

void initFXOS8700Q(void) {
  uint8_t data[2];
  FXOS8700CQ_readRegs( FXOS8700Q_CTRL_REG1, &data[1], 1);
  data[1] |= 0x01;
  data[0] = FXOS8700Q_CTRL_REG1;
  FXOS8700CQ_writeRegs(data, 2);
}


int main() {
    led1 = 1;
    initFXOS8700Q();

    t.start(callback(&queue, &EventQueue::dispatch_forever));
    btnRecord.fall(queue.event(start_log));

}