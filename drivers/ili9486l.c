/**
  ******************************************************************************
  * @file    ili9486l.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file includes the LCD driver for ILI9486L LCD.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "ili9486l.h"
#include <board.h>
#include <rtthread.h>

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup ILI9486L
  * @brief This file provides a set of functions needed to drive the 
  *        ILI9486L LCD.
  * @{
  */

/** @defgroup ILI9486L_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup ILI9486L_Private_Defines
	* @brief	选择BANK1-BORSRAM1 连接 TFT，地址范围为0X60000000~0X63FFFFFF
	* 				FSMC_A16 接LCD的DC(寄存器/数据选择)脚
	* 				16 bit => FSMC[24:0]对应HADDR[25:1]
	* 				寄存器基地址 = 0X60000000
	* 				RAM基地址 = 0X60020000 = 0X60000000+2^16*2 = 0X60000000 + 0X20000 = 0X60020000
	* 				当选择不同的地址线时，地址要重新计算。
  * @{
	*/
#define Bank1_LCD_D    ((uint32_t)0x60020000)    //Disp Data ADDR
#define Bank1_LCD_C    ((uint32_t)0x60000000)	   //Disp Reg ADDR

/**
  * @}
  */ 
  
/** @defgroup ILI9486L_Private_Macros
  * @{
  */
#define LCD_IO_WriteReg(x) 	*(__IO uint16_t *) (Bank1_LCD_C) = (x)
#define LCD_IO_WriteData(x) *(__IO uint16_t *) (Bank1_LCD_D) = (x)

/**
  * @}
  */  

/** @defgroup ILI9486L_Private_Variables
  * @{
  */ 

LCD_DrvTypeDef   ili9486l_drv = 
{
  ili9486l_Init,
  ili9486l_ReadID,
  ili9486l_DisplayOn,
  ili9486l_DisplayOff,
	ili9486l_SetDisplayDirection,
  ili9486l_SetCursor,
  ili9486l_WritePixel,
  ili9486l_ReadPixel,
  ili9486l_SetDisplayWindow,
  ili9486l_DrawHLine,
  ili9486l_DrawVLine,
  ili9486l_GetLcdPixelWidth,
  ili9486l_GetLcdPixelHeight,
  ili9486l_DrawBitmap,
  ili9486l_DrawRGBImage, 
};

/**
  * @}
  */ 
  
/** @defgroup ili9486l_Private_FunctionPrototypes
  * @{
  */
static void LCD_IO_Init(void);
static uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize);
static void LCD_Delay (uint32_t delay);

/**
  * @}
  */ 
  
/** @defgroup ili9486l_Private_Functions
  * @{
  */   

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
void ili9486l_Init(void)
{
    /* Initialize ILI9486L low level bus layer ----------------------------------*/
    LCD_IO_Init();

    /* Configure LCD */
    ili9486l_WriteReg(LCD_POWER1);
    ili9486l_WriteData(0x0F);
    ili9486l_WriteData(0x0F);

    ili9486l_WriteReg(LCD_POWER2);
    ili9486l_WriteData(0x41);

    ili9486l_WriteReg(LCD_VCOM1);
    ili9486l_WriteData(0x00);
    ili9486l_WriteData(0x3A);

    ili9486l_WriteReg(LCD_FRMCTR1);
    ili9486l_WriteData(0xD0);
    ili9486l_WriteData(0x11);

    //	 ili9486l_WriteReg(LCD_INVTR);
    //	 ili9486l_WriteData(0x00);

    ili9486l_WriteReg(LCD_DFC);
    ili9486l_WriteData(0x02);
    ili9486l_WriteData(0x22);
    //	 ili9486l_WriteData(0x3B);

    ili9486l_WriteReg(LCD_ETMOD);
    ili9486l_WriteData(0xC6);

    ili9486l_WriteReg(LCD_PGAMMA);
    ili9486l_WriteData(0x00);
    ili9486l_WriteData(0x0B);
    ili9486l_WriteData(0x10);
    ili9486l_WriteData(0x03);
    ili9486l_WriteData(0x10);
    ili9486l_WriteData(0x08);
    ili9486l_WriteData(0x33);
    ili9486l_WriteData(0x89);
    ili9486l_WriteData(0x48);
    ili9486l_WriteData(0x07);
    ili9486l_WriteData(0x0E);
    ili9486l_WriteData(0x0C);
    ili9486l_WriteData(0x28);
    ili9486l_WriteData(0x2D);
    ili9486l_WriteData(0x0F);

    ili9486l_WriteReg(LCD_NGAMMA);
    ili9486l_WriteData(0x00);
    ili9486l_WriteData(0x12);
    ili9486l_WriteData(0x17);
    ili9486l_WriteData(0x03);
    ili9486l_WriteData(0x11);
    ili9486l_WriteData(0x08);
    ili9486l_WriteData(0x37);
    ili9486l_WriteData(0x67);
    ili9486l_WriteData(0x4C);
    ili9486l_WriteData(0x07);
    ili9486l_WriteData(0x0F);
    ili9486l_WriteData(0x0C);
    ili9486l_WriteData(0x2F);
    ili9486l_WriteData(0x34);
    ili9486l_WriteData(0x0F);

    ili9486l_WriteReg(LCD_3GAMMA_EN);
    ili9486l_WriteData(0x18);
    ili9486l_WriteData(0xA3);
    ili9486l_WriteData(0x12);
    ili9486l_WriteData(0x02);
    ili9486l_WriteData(0xB2);
    ili9486l_WriteData(0x12);
    ili9486l_WriteData(0xFF);
    ili9486l_WriteData(0x10);
    ili9486l_WriteData(0x00);

    ili9486l_WriteReg(LCD_PRC);
    ili9486l_WriteData(0xA9);
    ili9486l_WriteData(0x91);
    ili9486l_WriteData(0x2D);
    ili9486l_WriteData(0x0A);
    ili9486l_WriteData(0x4F);

    ili9486l_WriteReg(0XF8);
    ili9486l_WriteData(0x21);
    ili9486l_WriteData(0x04);

    ili9486l_WriteReg(LCD_MAC);
    ili9486l_WriteData(0x48);

    ili9486l_WriteReg(LCD_PIXEL_FORMAT);
    ili9486l_WriteData(0x55);

    ili9486l_WriteReg(0xF9);  
    ili9486l_WriteData(0x00);  
    ili9486l_WriteData(0x08);  

    ili9486l_WriteReg(0xF4);  
    ili9486l_WriteData(0x00);  
    ili9486l_WriteData(0x00); 
    ili9486l_WriteData(0x08);  
    ili9486l_WriteData(0x91); 
    ili9486l_WriteData(0x04); 

    ili9486l_WriteReg(LCD_DINVON);
    ili9486l_WriteReg(LCD_SLEEP_OUT);
    LCD_Delay(120);	
    ili9486l_WriteReg(LCD_DISPLAY_ON);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
uint16_t ili9486l_ReadID(void)
{
//  LCD_IO_Init();
  return ((uint16_t)ili9486l_ReadData(LCD_READ_ID4, LCD_READ_ID4_SIZE));
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ili9486l_DisplayOn(void)
{
  /* Display On */
  ili9486l_WriteReg(LCD_DISPLAY_ON);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9486l_DisplayOff(void)
{
  /* Display Off */
  ili9486l_WriteReg(LCD_DISPLAY_OFF);
}

/**
  * @brief  Set the LCD cursor.
  * @param  Xpos: X position 
  * @param  Ypos: Y position
	* @retval None
  */
void ili9486l_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	/*Set X position*/
	LCD_IO_WriteReg(LCD_COLUMN_ADDR);
	LCD_IO_WriteData(Xpos >> 8);
	LCD_IO_WriteData(Xpos & 0xFF);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_WIDTH >> 8);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_WIDTH & 0xFF);
	
	/*Set Y position*/
	LCD_IO_WriteReg(LCD_PAGE_ADDR);
	LCD_IO_WriteData(Ypos >> 8);
	LCD_IO_WriteData(Ypos & 0xFF);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_HEIGHT >> 8);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_HEIGHT & 0xFF);
}

/**
  * @brief  Draws a pixel on LCD.
  * @param  Xpos: X position 
  * @param  Ypos: Y position
  * @param  RGB_Code: Pixel color in RGB mode (5-6-5)  
  * @retval None
  */
void ili9486l_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code)
{
	/*Set X position*/
	LCD_IO_WriteReg(LCD_COLUMN_ADDR);
	LCD_IO_WriteData(Xpos >> 8);
	LCD_IO_WriteData(Xpos & 0xFF);
	LCD_IO_WriteData(Xpos >> 8);
	LCD_IO_WriteData(Xpos & 0xFF);
	
	/*Set Y position*/
	LCD_IO_WriteReg(LCD_PAGE_ADDR);
	LCD_IO_WriteData(Ypos >> 8);
	LCD_IO_WriteData(Ypos & 0xFF);
	LCD_IO_WriteData(Ypos >> 8);
	LCD_IO_WriteData(Ypos & 0xFF);
	
	LCD_IO_WriteReg(LCD_GRAM);
	LCD_IO_WriteData(RGB_Code);
}

/**
  * @brief  Read a pixel from LCD.
  * @param  Xpos: X position 
  * @param  Ypos: Y position 
  * @retval RGB_Code: Pixel color in RGB mode (5-6-5) 
  */
uint16_t ili9486l_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
	__IO uint16_t RGB_Code;
	
	/*Set X position*/
	LCD_IO_WriteReg(LCD_COLUMN_ADDR);
	LCD_IO_WriteData(Xpos >> 8);
	LCD_IO_WriteData(Xpos & 0xFF);
	LCD_IO_WriteData(Xpos >> 8);
	LCD_IO_WriteData(Xpos & 0xFF);
	
	/*Set Y position*/
	LCD_IO_WriteReg(LCD_PAGE_ADDR);
	LCD_IO_WriteData(Ypos >> 8);
	LCD_IO_WriteData(Ypos & 0xFF);
	LCD_IO_WriteData(Ypos >> 8);
	LCD_IO_WriteData(Ypos & 0xFF);
	
	LCD_IO_WriteReg(LCD_RAMRD);
	RGB_Code = *(__IO uint16_t *) (Bank1_LCD_D);//dummy read
	RGB_Code = *(__IO uint16_t *) (Bank1_LCD_D);

	return RGB_Code;
}

/**
  * @brief  Sets display window.
  * @param  LayerIndex: layer index
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: LCD window width
  * @param  Height: LCD window height  
  * @retval None
  */
void ili9486l_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
	uint16_t Win_Width = Xpos + Width - 1;
	uint16_t Win_Height = Ypos + Height - 1;
		
	/* Colomn address set */
	ili9486l_WriteReg(LCD_COLUMN_ADDR);
	ili9486l_WriteData(Xpos >> 8);
	ili9486l_WriteData(Xpos & 0xFF);
	ili9486l_WriteData(Win_Width >> 8);
	ili9486l_WriteData(Win_Width & 0xFF);
	
		/* Page address set */
	ili9486l_WriteReg(LCD_PAGE_ADDR); 
	ili9486l_WriteData(Ypos >> 8);
	ili9486l_WriteData(Ypos & 0xFF);
	ili9486l_WriteData(Win_Height >> 8);
	ili9486l_WriteData(Win_Height & 0xFF);
}

/**
  * @brief  Draws an horizontal line.
	* @param  RGB_Code: Pixel color in RGB mode (5-6-5) 
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  * @retval None
  */
void ili9486l_DrawHLine(uint16_t RGB_Code, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
	uint16_t index;
	
	/*Set X position*/
	LCD_IO_WriteReg(LCD_COLUMN_ADDR);
	LCD_IO_WriteData(Xpos >> 8);
	LCD_IO_WriteData(Xpos & 0xFF);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_WIDTH >> 8);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_WIDTH & 0xFF);
	
	/*Set Y position*/
	LCD_IO_WriteReg(LCD_PAGE_ADDR);
	LCD_IO_WriteData(Ypos >> 8);
	LCD_IO_WriteData(Ypos & 0xFF);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_HEIGHT >> 8);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_HEIGHT & 0xFF);
	
	LCD_IO_WriteReg(LCD_GRAM);
	for(index = 0; index < Length; index++)
	{
		LCD_IO_WriteData(RGB_Code);
	}
}

/**
  * @brief  Draws an vertical line.
	* @param  RGB_Code: Pixel color in RGB mode (5-6-5) 
  * @param  Xpos: X position
  * @param  Ypos: Y position
  * @param  Length: Line length
  * @retval None
  */
void ili9486l_DrawVLine(uint16_t RGB_Code, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
	uint8_t MAC_Value;
	uint16_t index;
	
	/*Set X position*/
	LCD_IO_WriteReg(LCD_PAGE_ADDR);
	LCD_IO_WriteData(Xpos >> 8);
	LCD_IO_WriteData(Xpos & 0xFF);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_WIDTH >> 8);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_WIDTH & 0xFF);
	
	/*Set Y position*/
	LCD_IO_WriteReg(LCD_COLUMN_ADDR);
	LCD_IO_WriteData(Ypos >> 8);
	LCD_IO_WriteData(Ypos & 0xFF);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_HEIGHT >> 8);
	LCD_IO_WriteData(ili9486l_LCD_PIXEL_HEIGHT & 0xFF);
	
	/* Toggle the MV bit (address is updated in vertical writing direction) */
	MAC_Value = ili9486l_ReadData(LCD_RDDMADCTL, 1);
	LCD_IO_WriteReg(LCD_MAC);
	LCD_IO_WriteData(MAC_Value ^ 0x20);
	
	LCD_IO_WriteReg(LCD_GRAM);
	for(index = 0; index < Length; index++)
	{
		LCD_IO_WriteData(RGB_Code);
	}
	
	/* Toggle the MV bit (address is updated in horizontal writing direction) */
	LCD_IO_WriteReg(LCD_MAC);
	LCD_IO_WriteData(MAC_Value);
}

/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
void ili9486l_WriteReg(uint8_t LCD_Reg)
{
  LCD_IO_WriteReg(LCD_Reg);
}

/**
  * @brief  Writes data to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
void ili9486l_WriteData(uint16_t RegValue)
{
  LCD_IO_WriteData(RegValue);
}

/**
  * @brief  Reads the selected LCD Register.
  * @param  RegValue: Address of the register to read
  * @param  ReadSize: Number of bytes to read
  * @retval LCD Register Value.
  */
uint32_t ili9486l_ReadData(uint16_t RegValue, uint8_t ReadSize)
{
  /* Read a max of 4 bytes */
  return (LCD_IO_ReadData(RegValue, ReadSize));
}

/**
  * @brief  Get LCD PIXEL WIDTH.
  * @param  None
  * @retval LCD PIXEL WIDTH.
  */
uint16_t ili9486l_GetLcdPixelWidth(void)
{
  /* Return LCD PIXEL WIDTH */
  return ili9486l_LCD_PIXEL_WIDTH;
}

/**
  * @brief  Get LCD PIXEL HEIGHT.
  * @param  None
  * @retval LCD PIXEL HEIGHT.
  */
uint16_t ili9486l_GetLcdPixelHeight(void)
{
  /* Return LCD PIXEL HEIGHT */
  return ili9486l_LCD_PIXEL_HEIGHT;
}

/**
  * @brief  Draws a bitmap picture loaded in the STM32 MCU internal memory.
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pBmp: Pointer to Bmp picture address
  * @retval None
  */
void ili9486l_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pBmp)
{
	uint8_t OFFSET_BITMAP;
	uint32_t index;
	uint32_t BmpSize;
	
	OFFSET_BITMAP = *(uint16_t *) (pBmp + 10);
	BmpSize = *(uint16_t *) (pBmp + 2);
	BmpSize |= ((*(uint16_t *) (pBmp + 4)) << 16);
	BmpSize = (BmpSize - OFFSET_BITMAP) / 2;
	pBmp += OFFSET_BITMAP;
	
	LCD_IO_WriteReg(LCD_GRAM);
	for(index = 0; index < BmpSize; index++)
	{
		LCD_IO_WriteData(*(uint16_t *) pBmp);
		pBmp += sizeof(uint16_t);
	}
}

/**
  * @brief  Displays picture.
  * @param  pData: picture address.
  * @param  Xpos: Image X position in the LCD
  * @param  Ypos: Image Y position in the LCD
  * @param  Xsize: Image X size in the LCD
  * @param  Ysize: Image Y size in the LCD
  * @retval None
  */
void ili9486l_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pData)
{
	uint32_t index;
  uint32_t ImageSize = 0;

  ImageSize = (Xsize * Ysize);

  LCD_IO_WriteReg(LCD_GRAM);
	for(index = 0; index < ImageSize; index++)
	{
		LCD_IO_WriteData(*(uint16_t *) pData);
		pData += sizeof(uint16_t);
	}
}

/**
  * @brief  Set the display direction to the LCD.
  * @param  Angle: Display angle
  * @retval None
  */
void ili9486l_SetDisplayDirection(uint16_t Angle)
{
	switch(Angle)
	{
	case 0:	
		ili9486l_WriteReg(LCD_MAC);
		ili9486l_WriteData(0x48);
		ili9486l_LCD_PIXEL_WIDTH = 320;
		ili9486l_LCD_PIXEL_HEIGHT = 480;
		ili9486l_SetDisplayWindow(0,0,ili9486l_LCD_PIXEL_WIDTH,
														ili9486l_LCD_PIXEL_HEIGHT);break;
	case 90:	
		ili9486l_WriteReg(LCD_MAC);
		ili9486l_WriteData(0xE8);
		ili9486l_LCD_PIXEL_WIDTH = 480;
		ili9486l_LCD_PIXEL_HEIGHT = 320;
		ili9486l_SetDisplayWindow(0,0,ili9486l_LCD_PIXEL_WIDTH,
														ili9486l_LCD_PIXEL_HEIGHT);break;
	case 180:
		ili9486l_WriteReg(LCD_MAC);
		ili9486l_WriteData(0x88);
		ili9486l_LCD_PIXEL_WIDTH = 320;
		ili9486l_LCD_PIXEL_HEIGHT = 480;
		ili9486l_SetDisplayWindow(0,0,ili9486l_LCD_PIXEL_WIDTH,
														ili9486l_LCD_PIXEL_HEIGHT);break;
	case 270:
		ili9486l_WriteReg(LCD_MAC);
		ili9486l_WriteData(0x28);
		ili9486l_LCD_PIXEL_WIDTH = 480;
		ili9486l_LCD_PIXEL_HEIGHT = 320;
		ili9486l_SetDisplayWindow(0,0,ili9486l_LCD_PIXEL_WIDTH,
														ili9486l_LCD_PIXEL_HEIGHT);break;
	default:			
		ili9486l_WriteReg(LCD_MAC);
		ili9486l_WriteData(0x88);
		ili9486l_LCD_PIXEL_WIDTH = 320;
		ili9486l_LCD_PIXEL_HEIGHT = 480;
		ili9486l_SetDisplayWindow(0,0,ili9486l_LCD_PIXEL_WIDTH,
														ili9486l_LCD_PIXEL_HEIGHT);break;
	}
}

/******************************************************************************
                            Static Functions
*******************************************************************************/
static void LCD_IO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  Timing; 
    FSMC_NORSRAMTimingInitTypeDef  ExtTiming;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE, ENABLE);//使能PD,PE时钟  
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);//使能FSMC时钟  

    GPIO_InitStructure.GPIO_Pin = 0XCFB3;//PD0,1,4,5,7,8,9,10,11,14,15 AF OUT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化  

    GPIO_InitStructure.GPIO_Pin = (0X1FF<<7);//PE7~15,AF OUT
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用输出
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化  

    GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_FSMC); //FSMC_D2, AF12
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_FSMC); //FSMC_D3, AF12
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource4,GPIO_AF_FSMC); //LCD_RD,  AF12
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_FSMC); //LCD_WR,  AF12 
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource7,GPIO_AF_FSMC); //LCD_CS,  AF12
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_FSMC); //FSMC_D13,AF12 
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_FSMC); //FSMC_D14,AF12
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource10,GPIO_AF_FSMC);//FSMC_D15,AF12
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource11,GPIO_AF_FSMC);//LCD_RS,  AF12
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_FSMC);//FSMC_D0, AF12
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_FSMC);//FSMC_D1, AF12

    GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_FSMC); //FSMC_D4, AF12
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_FSMC); //FSMC_D5, AF12
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_FSMC); //FSMC_D6, AF12
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource10,GPIO_AF_FSMC);//FSMC_D7, AF12
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_FSMC);//FSMC_D8, AF12
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource12,GPIO_AF_FSMC);//FSMC_D9, AF12
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_FSMC);//FSMC_D10,AF12
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_FSMC);//FSMC_D11,AF12
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource15,GPIO_AF_FSMC);//FSMC_D12,AF12
    /* Timing */
    Timing.FSMC_AddressSetupTime = 0x00;	 //地址建立时间（ADDSET）为16个HCLK 1/168M=6ns*16=96ns	
    Timing.FSMC_AddressHoldTime = 0x01;	 //地址保持时间（ADDHLD）模式A未用到	
    Timing.FSMC_DataSetupTime = 0x06;		 //数据保存时间为60个HCLK	=6*60=360ns
    Timing.FSMC_BusTurnAroundDuration = 0x00;
    Timing.FSMC_CLKDivision = 0x00;
    Timing.FSMC_DataLatency = 0x00;
    Timing.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A 
    /* ExtTiming */
    ExtTiming.FSMC_AddressSetupTime =0x02;	      //地址建立时间（ADDSET）为9个HCLK =54ns 
    ExtTiming.FSMC_AddressHoldTime = 0x01;	 //地址保持时间（A		
    ExtTiming.FSMC_DataSetupTime = 0x07;		 //数据保存时间为6ns*9个HCLK=54ns
    ExtTiming.FSMC_BusTurnAroundDuration = 0x00;
    ExtTiming.FSMC_CLKDivision = 0x00;
    ExtTiming.FSMC_DataLatency = 0x00;
    ExtTiming.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A 

    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;//  这里我们使用NE1 ，也就对应BTCR[0],[1]。
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; // 不复用数据地址
    FSMC_NORSRAMInitStructure.FSMC_MemoryType =FSMC_MemoryType_SRAM;// FSMC_MemoryType_SRAM;  //SRAM   
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;//存储器数据宽度为16bit   
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode =FSMC_BurstAccessMode_Disable;// FSMC_BurstAccessMode_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;   
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;	//  存储器写使能
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;   
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable; // 读写使用不同的时序
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &Timing; //读写时序
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &ExtTiming;  //写时序

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  //初始化FSMC配置
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  // 使能BANK1 
}

static uint32_t LCD_IO_ReadData(uint16_t RegValue, uint8_t ReadSize)
{
	__IO uint32_t Data;
	uint8_t i;
	
	*(__IO uint16_t *) (Bank1_LCD_C) = RegValue;
	Data = *(__IO uint16_t *) (Bank1_LCD_D);//read dummy
	for(i = 0; i < ReadSize; i++)
	{
		Data <<= 8;
		Data |= *(__IO uint16_t *) (Bank1_LCD_D);
	}
	return Data;	
}

//extern void Delay_ms(u32 tmp);
static void LCD_Delay (uint32_t delay)
{
//	Delay_ms(delay);
		rt_thread_mdelay(delay);
}
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */ 

/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
