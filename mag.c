#include "MKL46Z4.h"
#include "i2c.h"
#include "lcd.h"
#include <math.h>
#include "mag.h"

void initMagnetometer(void)
{
    unsigned char control_reg1;

    // khoi tao bit va tan so
    unsigned char config_info[2];
		config_info[0] = 0x01; // 16bit	
		config_info[1] = 0x80; // 50Hz
	
    control_reg1 = readI2C_single(MAG_DEVICE_ADDRESS, MAG_CTRL_REG1);

		// neu dung chan thi dua thong tin vao i2c dieu khien mag
		if(control_reg1!=0x01)
		{
			sendI2C_multi(MAG_DEVICE_ADDRESS,MAG_CTRL_REG1,2,config_info);
		}
}

void Magnetometer_Acq(void)
{
    readI2C_multi(MAG_DEVICE_ADDRESS, MAG_OUT_X_MSB, 6);

    // KKet noi cac byte thap va cao
    for(int i = 0; i < 3; i++)
    {
        MAG_DATA_READ_AXIS[i] = ((short int)((DATA_READ[2 * i] << 8) | DATA_READ[2 * i + 1]));
    }

    // Tìm max va min
    for(int i = 0; i < 3; i++)
    {
        if((MAG_DATA_MAX_AXIS[i] == 0) && (MAG_DATA_MIN_AXIS[i] == 0))
        {
            MAG_DATA_MAX_AXIS[i] = MAG_DATA_READ_AXIS[i];
            MAG_DATA_MIN_AXIS[i] = MAG_DATA_READ_AXIS[i];
        }
        else if (MAG_DATA_READ_AXIS[i] > MAG_DATA_MAX_AXIS[i])
        {
            MAG_DATA_MAX_AXIS[i] = MAG_DATA_READ_AXIS[i];
        }
        else if (MAG_DATA_READ_AXIS[i] < MAG_DATA_MIN_AXIS[i])
        {
            MAG_DATA_MIN_AXIS[i] = MAG_DATA_READ_AXIS[i];
        }
    }
}

void Magnetometer_Avg(void)
{
    // Tính toán giá tri trung bình
    for(int i = 0; i < 3; i++)
    {
        MAG_DATA_AVERAGE_AXIS[i] = (MAG_DATA_MAX_AXIS[i] + MAG_DATA_MIN_AXIS[i]) / 2;
    }
}

void Magnetometer_Run(void)
{

			// Doc gia tri cam bien
			readI2C_multi(MAG_DEVICE_ADDRESS, MAG_OUT_X_MSB, 6);

			// Ket noi cac byte cao va thap
			for(int i = 0; i < 3; i++)
			{
					MAG_DATA_READ_AXIS[i] = ((short int)((DATA_READ[2 * i] << 8) | DATA_READ[2 * i + 1]));
			}

			// tinh gia tri hieu chinh bang mag_read - mag_avg
			for(int i = 0; i < 3; i++)
			{
					MAG_DATA_HI_CALIBRATED[i] = MAG_DATA_READ_AXIS[i] - MAG_DATA_AVERAGE_AXIS[i];
			}

			// Tính toán góc
			if((MAG_DATA_HI_CALIBRATED[1] == 0) && (MAG_DATA_HI_CALIBRATED[0] > 0))
			{
					ANGLE = 0;
			}
			else if((MAG_DATA_HI_CALIBRATED[1] == 0) && (MAG_DATA_HI_CALIBRATED[0] < 0))
			{
					ANGLE = 180;
			}
			else if(MAG_DATA_HI_CALIBRATED[1] < 0)
			{
					ANGLE = 270 - (atan(((double)MAG_DATA_HI_CALIBRATED[0] / (double)MAG_DATA_HI_CALIBRATED[1])) * 57.29); // nhan voi 180/pi
			}
			else
			{
					ANGLE = 90 - (atan(((double)MAG_DATA_HI_CALIBRATED[0] / (double)MAG_DATA_HI_CALIBRATED[1])) * 57.29); // nhan voi 180/pi
			}
}
