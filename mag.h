#ifndef __MAG_H__
#define __MAG_H__

#define MAG_DEVICE_ADDRESS                 0x0E
#define MAG_CTRL_REG1                      0x10
#define MAG_CTRL_REG2                      0x11
#define MAG_DR_STATUS                      0x00
#define MAG_OUT_X_MSB                      0x01
#define MAG_OUT_X_LSB                      0x02
#define MAG_OUT_Y_MSB                      0x03
#define MAG_OUT_Y_LSB                      0x04
#define MAG_OUT_Z_MSB                      0x05
#define MAG_OUT_Z_LSB                      0x06
#define MAG_DEVICE_ID_REGISTER_ADDRESS     0x07
#define MAG_SYSMOD                         0x08
#define MAG_OFF_X_MSB                      0x09
#define MAG_OFF_X_LSB                      0x0A
#define MAG_OFF_Y_MSB                      0x0B
#define MAG_OFF_Y_LSB                      0x0C
#define MAG_OFF_Z_MSB                      0x0D
#define MAG_OFF_Z_LSB                      0x0E

// Khoi tao cac mang chua gia tri XYZ
extern short int MAG_DATA_READ_AXIS[3];
extern short int MAG_DATA_MAX_AXIS[3];
extern short int MAG_DATA_MIN_AXIS[3];
extern short int MAG_DATA_AVERAGE_AXIS[3];
extern short int MAG_DATA_HI_CALIBRATED[3];
extern unsigned char DR_STATUS_DATA;
extern short int ANGLE;

void initMagnetometer(void);
void Magnetometer_Acq(void);
void Magnetometer_Avg(void);
void Magnetometer_Run(void);
#endif