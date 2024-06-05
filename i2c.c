#include "MKL46Z4.h"
#include "i2c.h"
#include "delay.h"

void initI2C0(void)
{
		//khoi tao clock cho portE va I2C0
		SIM->SCGC5|=SIM_SCGC5_PORTE_MASK;
		SIM->SCGC4|=SIM_SCGC4_I2C0_MASK;

		//Khoi tao SDL va SDA
		PORTE->PCR[I2C0_SDA]|= PORT_PCR_MUX(5);
		PORTE->PCR[I2C0_SCL]|= PORT_PCR_MUX(5);

		// khoi tao SCL pullup
		PORTE->PCR[I2C0_SCL]|= PORT_PCR_PS_MASK |PORT_PCR_PE_MASK;
		// khoi tao SDA pullup
		PORTE->PCR[I2C0_SDA]|= PORT_PCR_PS_MASK |PORT_PCR_PE_MASK;
	
		// Cai dat tan so clock 100kHz va toc do baud rate 100kbps
		I2C0->F |= I2C_F_MULT(02) | I2C_F_ICR(0x00);
		// kich hoat I2C
		I2C0->C1 |= I2C_C1_IICEN_MASK;
}


unsigned char readI2C_single(unsigned char DEV_ADR, unsigned char REG_ADR)
{   
		unsigned char dummy_read = 0;
		unsigned char data = 0;
	
		//Select transmit and start I2C
		I2C0->C1 |= I2C_C1_TX_MASK;
		I2C0->C1 |= I2C_C1_MST_MASK;
		
		//Send MAG device address with write bit
		I2C0->D = WRITE(DEV_ADR);
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Send MAG register address
		I2C0->D = REG_ADR;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |=I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Repeated start to change to read mode
		I2C0->C1 |= I2C_C1_RSTA_MASK;
			
		//Send MAG device address and a Read Bit
		I2C0->D = READ(DEV_ADR);
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S|= I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
		
		//Sending NAK to ensure right after the data is read, NAK signal is sent
		I2C0->C1 |= I2C_C1_TXAK_MASK;
		//Set the I2C in Receiver Mode to read data from MAG3110
		I2C0->C1 &= (~I2C_C1_TX_MASK);
				
		//Read dummy data
		dummy_read = I2C0->D;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		
		//Read real data
		data = I2C0->D;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
			
		//Stop I2C
		I2C0->C1 &= (~I2C_C1_MST_MASK);
		// Clear Transmit Nack by setting TXAK to 0
		I2C0->C1 &= ~(I2C_C1_TXAK_MASK);
			
		delay();
		return data;
}

void readI2C_multi(unsigned char DEV_ADR,unsigned char REG_ADR, int num_bytes)
{   
		unsigned char dummy_data = 0;
		//Select transmit and start I2C
		I2C0->C1 |= I2C_C1_TX_MASK;
		I2C0->C1 |= I2C_C1_MST_MASK;
		
		//Send MAG address with write bit
		I2C0->D = WRITE(DEV_ADR);
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Send MAG register address 
		I2C0->D = REG_ADR;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |=I2C_S_IICIF_MASK;
		//Wait ACK reg address from MAG
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Repeated Start to start read data
		I2C0->C1|=I2C_C1_RSTA_MASK;
			
		//Send MAG device address and a Read Bit
		I2C0->D = READ(DEV_ADR);
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
		
		//Set the I2C in Receiver Mode
		I2C0->C1&=(~I2C_C1_TX_MASK);
		//Read Dummy Magnetometer Data
		dummy_data = I2C0->D;
		for(int i = 0; i < num_bytes; i++)
		{
			if(i < (num_bytes - 2))//read normally
					{
					 while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
					 I2C0->S|= I2C_S_IICIF_MASK;
						DATA_READ[i] = I2C0->D;
					}
			else
					{//Read two final bytes differently to ensure the NAK signal
						
						//Sending NAK to ensure right after the data is read, NAK signal is sent
						I2C0->C1 |= (I2C_C1_TXAK_MASK);
						
						while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
						I2C0->S |= I2C_S_IICIF_MASK;
						
						DATA_READ[i] = I2C0->D;
						while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
						I2C0->S|= I2C_S_IICIF_MASK;
						
						i++;
						DATA_READ[i]=I2C0->D;
						while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
						I2C0->S|= I2C_S_IICIF_MASK;
							
						//Stop
						I2C0->C1 &= (~I2C_C1_MST_MASK);
						// Clear Transmit Nack by setting TXAK to 0
						I2C0->C1 &= (~I2C_C1_TXAK_MASK);
					}
		}
		delay();
}

void sendI2C_single(unsigned char DEV_ADR, unsigned char REG_ADR, unsigned char DATA)
{
		//Select transmit and start I2C
		I2C0->C1 |= I2C_C1_TX_MASK;
		I2C0->C1 |= I2C_C1_MST_MASK;
		
		//Send MAG device address with write bit
		I2C0->D = WRITE(DEV_ADR);
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		//Wait ACK device address from MAG
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Send MAG register address 
		I2C0->D = REG_ADR;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |=I2C_S_IICIF_MASK;
		//Wait ACK reg address from MAG
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Send data
		I2C0->D = DATA;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |=I2C_S_IICIF_MASK;
		//Wait ACK reg address from MAG
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
		
		//Stop
		I2C0->C1&=(~I2C_C1_MST_MASK);
		
		delay();
}


void sendI2C_multi(unsigned char DEV_ADR, unsigned char REG_ADR, int num_bytes, unsigned char data_bytes[])
{
		//Select transmit and start I2C
		I2C0->C1 |= I2C_C1_TX_MASK;
		I2C0->C1 |= I2C_C1_MST_MASK;
		
		//Send MAG address with write bit
		I2C0->D = WRITE(DEV_ADR);
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |= I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		//Send MAG register address 
		I2C0->D = REG_ADR;
		while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
		I2C0->S |=I2C_S_IICIF_MASK;
		while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
			
		// Send the multiple bytes of data
		for(int i = 0;i < num_bytes; i++)
		{
			I2C0->D = data_bytes[i];
			while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
			I2C0->S|= I2C_S_IICIF_MASK;
			while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
		}
		//Stop 
		I2C0->C1&=(~I2C_C1_MST_MASK);
		delay();
}
