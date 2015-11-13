#include <rtthread.h>
#include "board.h"
#include "drv_mpu6050.h"

#ifdef RT_USING_SPI
#include "rt_stm32f10x_spi.h"

static int rt_hw_spi_init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
    /* register spi bus */
    {
        static struct stm32_spi_bus stm32_spi;
        
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); 
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 				

				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOA, &GPIO_InitStructure);
			
        stm32_spi_register(SPI1, &stm32_spi, "spi1");
    }
		
		    /* register spi bus */
    {
        static struct stm32_spi_bus stm32_spi;
        
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); 
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 				

				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOB, &GPIO_InitStructure);
			
        stm32_spi_register(SPI2, &stm32_spi, "spi2");
    }
	
		return RT_EOK;
}

INIT_BOARD_EXPORT(rt_hw_spi_init);

#endif /* RT_USING_SPI */

	
#ifdef RT_USING_NRF24L01

#include "nRF24L01.h"

static int rt_nRF24L01_init(void)
{
    /* attach nRF24L01 cs */
    {
        static struct rt_spi_device spi_device;
        static struct stm32_spi_cs  spi_cs;
				GPIO_InitTypeDef GPIO_InitStructure;
			
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 				
			
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
				GPIO_Init(GPIOA, &GPIO_InitStructure);

        spi_cs.GPIOx = GPIOA;
        spi_cs.GPIO_Pin = GPIO_Pin_4;

        GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;
        GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);
        GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);

        rt_spi_bus_attach_device(&spi_device, "RF2401", "spi1", (void*)&spi_cs);
    }

	rt_hw_nRF24L01_init("RF2401");
	return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_nRF24L01_init);	

#endif /* RT_USING_NRF24L01 */	

		

