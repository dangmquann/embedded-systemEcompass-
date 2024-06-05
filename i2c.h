#ifndef __I2C_H__
#define __I2C_H__

// dinh nghia cac chan I2C
#define READ_MASK                          0x01
#define WRITE_MASK                         0xFE
#define DATA_SHIFT                         1
#define ACC_DEVICE_ADDRESS                 0x1D
#define RESET_MASK                         0x00
#define RIGHT_SHIFT(x,y)                   (x >> y)
#define LEFT_SHIFT(x,y)                    (x << y)
#define I2C0_SCL                           (24)
#define I2C0_SDA                           (25)
#define READ(x)                            ((x<<1)|(0x01))
#define WRITE(x)                           ((x<<1)&(0xFE))

extern unsigned char DATA_READ[6];

void initI2C0(void);
unsigned char readI2C_single(unsigned char DEV_ADR, unsigned char REG_ADR);
void readI2C_multi(unsigned char DEV_ADR,unsigned char REG_ADR, int max_count);
void sendI2C_single(unsigned char DEV_ADR, unsigned char REG_ADR, unsigned char DATA);
void sendI2C_multi(unsigned char DEV_ADR, unsigned char REG_ADR, int max_count, unsigned char data_wr[]);
#endif