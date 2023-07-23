/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2013 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************//*!
*
* @file flash.c
*
* @author Freescale
*
* @version 0.0.1
*
* @date Jun. 25, 2013
*
* @brief providing APIs for configuring FLASH module (FLASH). 
*
*******************************************************************************
*
* provide APIs for configuring FLASH module (FLASH)
******************************************************************************/
#include "flash.h"
/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Constants and macros
******************************************************************************/

/******************************************************************************
* Local types
******************************************************************************/

/******************************************************************************
* Local function prototypes
******************************************************************************/

/******************************************************************************
* Local variables
******************************************************************************/

/******************************************************************************
* Local functions
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* define NVM APIs
*
*//*! @addtogroup nvm_api_list
* @{
*******************************************************************************/
#if defined(MCU_SKEAZ1284)
/*****************************************************************************//*!
  *
  * @brief initialize flash driver.
  *        
  * @param[in]  u32BusClock current bus clock.
  *
  * @return flash error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_Init(uint32_t u32BusClock)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	uint8_t clkDIV = u32BusClock/1000000L - 1;
	
	if(!(FTMRE_FSTAT & FTMRE_FSTAT_CCIF_MASK))
	{
		u16Err |= FLASH_ERR_INIT_CCIF;
		return u16Err;
	}
	/* initialize the flash clock to be within spec 1MHz 
	 * 
	 */
	if(!(FTMRE_FCLKDIV & FTMRE_FCLKDIV_FDIVLCK_MASK))
	{
		/* FCLKDIV register is not locked */
		if((FTMRE_FCLKDIV & FTMRE_FCLKDIV_FDIVLD_MASK) &&
                  ((FTMRE_FCLKDIV & FTMRE_FCLKDIV_FDIV_MASK) != FTMRE_FCLKDIV_FDIV(clkDIV)))
		{
			/* flash clock prescaler is loaded but with wrong value */
			u16Err |= FLASH_ERR_INIT_FDIV;
			return (u16Err);
		}
		FTMRE_FCLKDIV = (FTMRE_FCLKDIV & ~(FTMRE_FCLKDIV_FDIV_MASK)) | FTMRE_FCLKDIV_FDIV(clkDIV);
                
#if 0
		FTMRE_FCLKDIV  |= FTMRE_FCLKDIV_FDIVLCK_MASK; /* lock the prescaler */
#endif
	}
	else
	{
		/* FCLKDIV register is locked */
		if((FTMRE_FCLKDIV & FTMRE_FCLKDIV_FDIV_MASK) != FTMRE_FCLKDIV_FDIV(clkDIV))
		{
			/* flash clock prescaler is wrong */
			u16Err |= FLASH_ERR_INIT_FDIV;
		}
	}
	return (u16Err);	
}

/*****************************************************************************//*!
  *
  * @brief program flash routine.
  *        
  * @param[in]   u32NVMTargetAddress programed flash address.
  * @param[in]   pData programming data pointer.
  * @param[in]   u16SizeBytes programming data length.
  *
  * @return flash error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_Program(uint32_t u32NVMTargetAddress, uint8_t *pData, uint16_t u16SizeBytes)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	uint16_t u16w2LongWordCount = u16SizeBytes>>3;
	uint8_t  u8WrLeftBytes = (u16SizeBytes & 0x07);
	uint16_t u16WrLeftLongWords = u8WrLeftBytes>>2;
	uint32_t u32WrTargetAddress = u32NVMTargetAddress;
	uint32_t u32DwData0,u32DwData1;
	uint32_t *pDwData = (uint32_t*)pData;
	int  i;
	
	// Check address to see if it is aligned to 4 bytes
	// Global address [1:0] must be 00.
	if(u32NVMTargetAddress & 0x03)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Loop for the two longwords (8 bytes) programming
	for(i = 0; i < u16w2LongWordCount; i++)
	{
		u32DwData0 = *pDwData++;
		u32DwData1 = *pDwData++;
		u16Err = FLASH_Program2LongWords(u32WrTargetAddress, u32DwData0, u32DwData1);
		if(u16Err)
		{
			goto EndP;
			//break;
		}
		u32WrTargetAddress += 8;
	}
	// Loop for the single longword (4 bytes) programming
	for(i = 0; i < u16WrLeftLongWords; i++)
	{
		u32DwData0 = *pDwData++;
		u16Err = FLASH_Program1LongWord(u32WrTargetAddress, u32DwData0);
		if(u16Err)
		{			
			goto EndP;
			//break;
		}
		u32WrTargetAddress += 4;
	}
	u8WrLeftBytes = (u8WrLeftBytes-(u16WrLeftLongWords<<2));	// calculate the # of bytes that are not programmed
	if(!u8WrLeftBytes){
		return (u16Err);
	}
        
#if     defined(BIG_ENDIAN)                
	u32DwData0 = 0;
	pData = (uint8_t*)pDwData;	// pointer to the left bytes
	for(i = u8WrLeftBytes; i >0; i--)
	{
		u32DwData0 <<= 8;
		u32DwData0 |= *pData++;	// MSB byte first
	}
	// Calculate how many bytes need to be filled with 0xFFs
	// in order to form a single longword for the left bytes of data
	u8WrLeftBytes = 4 - u8WrLeftBytes;	
	//  
	for(i = u8WrLeftBytes; i >0; i--)
	{
		u32DwData0 <<= 8;
		u32DwData0 |= 0xFF;	// MSB byte first
	}
#else
	u32DwData0 = 0xFFFFFFFFL;        
	pData = (uint8_t*)pDwData+u8WrLeftBytes-1;	// pointer to the left bytes
	for(i = u8WrLeftBytes; i >0; i--)
	{
		u32DwData0 <<= 8;
		u32DwData0 |= *pData--;	// MSB byte first
	}
#endif	
	// Now program the last longword
	u16Err = FLASH_Program1LongWord(u32WrTargetAddress, u32DwData0);	
EndP:	
	return (u16Err);
}
/*****************************************************************************//*!
  *
  * @brief program flash routine, program 1 long word to flash.
  *        
  * @param[in]   u32NVMTargetAddress programed flash address.
  * @param[in]   u32DwData programming data.
  *
  * @return flash error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_Program1LongWord(uint32_t u32NVMTargetAddress, uint32_t u32DwData)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Check address to see if it is aligned to 4 bytes
	// Global address [1:0] must be 00.
	if(u32NVMTargetAddress & 0x03)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FLASH_CMD_PROGRAM;// program FLASH command
	FTMRE_FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16]
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRE_FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRE_FCCOBLO = u32NVMTargetAddress;
	FTMRE_FCCOBHI = u32NVMTargetAddress>>8;
	// Write index to specify the word0 (MSB word) to be programmed
	FTMRE_FCCOBIX = 0x2;
#if     defined(BIG_ENDIAN)        
	// Write the word  0
	FTMRE_FCCOBHI = (u32DwData>>16)>>8;
	FTMRE_FCCOBLO = (u32DwData>>16);
#else        
	FTMRE_FCCOBHI = (u32DwData) >>8;
	FTMRE_FCCOBLO = (u32DwData);
#endif        
	// Write index to specify the word1 (LSB word) to be programmed
	FTMRE_FCCOBIX = 0x3;
	// Write the word1 
#if     defined(BIG_ENDIAN)        
	FTMRE_FCCOBHI = (u32DwData) >>8;
	FTMRE_FCCOBLO = (u32DwData);
#else
	FTMRE_FCCOBHI = (u32DwData>>16)>>8;
	FTMRE_FCCOBLO = (u32DwData>>16);
#endif        
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
	
	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief program flash routine, program 2long word to flash.
  *        
  * @param[in]   u32NVMTargetAddress programed flash address.
  * @param[in]   u32DwData0 programming data0.
  * @param[in]   u32DwData1 programming data1.
  *
  * @return flash error status.
  *
  * @ Pass/ Fail criteria: none
*****************************************************************************/

uint16_t FLASH_Program2LongWords(uint32_t u32NVMTargetAddress, uint32_t u32DwData0, uint32_t u32DwData1)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Check address to see if it is aligned to 4 bytes
	// Global address [1:0] must be 00.
	if(u32NVMTargetAddress & 0x03)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FLASH_CMD_PROGRAM;// program FLASH command
	FTMRE_FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16]
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRE_FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRE_FCCOBLO = u32NVMTargetAddress;
	FTMRE_FCCOBHI = u32NVMTargetAddress>>8;
        
	// Write index to specify the word0 (MSB word) to be programmed
	FTMRE_FCCOBIX = 0x2;
	// Write the word 0
#if     defined(BIG_ENDIAN)                
	//FTMRE_FCCOB = (u32DwData0>>16) & 0xFFFF;
	FTMRE_FCCOBHI = (u32DwData0>>16)>>8;
	FTMRE_FCCOBLO = (u32DwData0>>16);
        
	// Write index to specify the word1 (LSB word) to be programmed
	FTMRE_FCCOBIX = 0x3;
	// Write word 1
	//FTMRE_FCCOB = (u32DwData0) & 0xFFFF;
	FTMRE_FCCOBHI = (u32DwData0) >>8;
	FTMRE_FCCOBLO = (u32DwData0);
	
	// Write index to specify the word0 (MSB word) to be programmed
	FTMRE_FCCOBIX = 0x4;
	// Write the word2
	//FTMRE_FCCOB = (u32DwData1>>16) & 0xFFFF;
	FTMRE_FCCOBHI = (u32DwData1>>16)>>8;
	FTMRE_FCCOBLO = (u32DwData1>>16);
        
	// Write index to specify the word1 (LSB word) to be programmed
	FTMRE_FCCOBIX = 0x5;
	// Write word 3
	//FTMRE_FCCOB = (u32DwData1) & 0xFFFF;
	FTMRE_FCCOBHI = (u32DwData1) >>8;
	FTMRE_FCCOBLO = (u32DwData1);
#else
	//FTMRE_FCCOB = (u32DwData0) & 0xFFFF;
	FTMRE_FCCOBHI = (u32DwData0) >>8;
	FTMRE_FCCOBLO = (u32DwData0);
        
	// Write index to specify the word1 (LSB word) to be programmed
	FTMRE_FCCOBIX = 0x3;
	// Write word 1
	FTMRE_FCCOBHI = (u32DwData0>>16)>>8;
	FTMRE_FCCOBLO = (u32DwData0>>16);
	
	// Write index to specify the word0 (MSB word) to be programmed
	FTMRE_FCCOBIX = 0x4;
	// Write the word2
	//FTMRE_FCCOB = (u32DwData1) & 0xFFFF;
	FTMRE_FCCOBHI = (u32DwData1) >>8;
	FTMRE_FCCOBLO = (u32DwData1);
        
	// Write index to specify the word1 (LSB word) to be programmed
	FTMRE_FCCOBIX = 0x5;
	// Write word 3
	//FTMRE_FCCOB = (u32DwData1>>16) & 0xFFFF;
	FTMRE_FCCOBHI = (u32DwData1>>16)>>8;
	FTMRE_FCCOBLO = (u32DwData1>>16);
#endif
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
	
	return (u16Err);
}



/*****************************************************************************//*!
  *
  * @brief erase flash sector, each flash sector is of 512 bytes long.
  *      
  *        
  * @param[in]   u32NVMTargetAddress erase sector address.
  *
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseSector(uint32_t u32NVMTargetAddress)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Check address to see if it is aligned to 4 bytes
	// Global address [1:0] must be 00.
	if(u32NVMTargetAddress & 0x03)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FLASH_CMD_ERASE_SECTOR;// EEPROM FLASH command
	FTMRE_FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16]
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRE_FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRE_FCCOBLO = u32NVMTargetAddress;
	FTMRE_FCCOBHI = u32NVMTargetAddress>>8;
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	

	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief unsecure device with unsecure command.
  * 
  *        
  * @param none. 
  *
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_Unsecure(void)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FTMRE_CMD_UNSECURE;// EEPROM FLASH command
	FTMRE_FCCOBLO = 0;// memory address bits[23:16]
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	

	return (u16Err);
}


/*****************************************************************************//*!
  *
  * @brief erase flash/EEPROM block.
  *        
  * @param[in] u32NVMTargetAddress: target address in NVM. 
  * @param[in] bIsEEPROM: specify the target address is within EEPROM,1 if in EEPROM block, 0 if in FLASH block.  
  *					
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseBlock(uint32_t u32NVMTargetAddress, uint8_t bIsEEPROM)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FTMRE_CMD_ERASE_BLOCK;// erase FLASH block command
	FTMRE_FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block
	if(bIsEEPROM)
	{
		FTMRE_FCCOBLO |= 0x80;	// bit 23 = 1 for EEPROM block
	}
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRE_FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRE_FCCOBLO = u32NVMTargetAddress;
	FTMRE_FCCOBHI = u32NVMTargetAddress>>8;
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	

	return (u16Err);
}


/*****************************************************************************//*!
  *
  * @brief erase verify flash/EEPROM block.
  *        
  * @param[in] u32NVMTargetAddress: target address in NVM. 
  * @param[in] bIsEEPROM: specify the target address is within EEPROM,1 if in EEPROM block, 0 if in FLASH block.  
  *
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseVerifyBlock(uint32_t u32NVMTargetAddress, uint8_t bIsEEPROM)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FTMRE_CMD_ERASE_VERIFY_BLOCK;// erase FLASH block command
	FTMRE_FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block
	if(bIsEEPROM)
	{
		FTMRE_FCCOBLO |= 0x80;	// bit 23 = 1 for EEPROM block
	}
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRE_FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRE_FCCOBLO = u32NVMTargetAddress;
	FTMRE_FCCOBHI = u32NVMTargetAddress>>8;
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	

	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief erase verify flash section.
  *        
  * @param[in]  u32NVMTargetAddress: target address in NVM, must be longword aligned. 
  * @param[in]  u16LongWordCount:   number of longwords to be verified.
  *	
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseVerifySection(uint32_t u32NVMTargetAddress, uint16_t u16LongWordCount)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	// Check address to see if it is aligned to 4 bytes
	// Global address [1:0] must be 00.
	if(u32NVMTargetAddress & 0x03)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}	
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FLASH_CMD_ERASE_VERIFY_SECTION;// erase verify FLASH section command
	FTMRE_FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRE_FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRE_FCCOBLO = u32NVMTargetAddress;
	FTMRE_FCCOBHI = u32NVMTargetAddress>>8;

	// Write index to specify the # of longwords to be verified
	FTMRE_FCCOBIX = 0x2;
	// Write the # of longwords 
	FTMRE_FCCOBLO = u16LongWordCount;
	FTMRE_FCCOBHI = u16LongWordCount>>8;
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	

	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief erase verify all flash and EEPROM blocks.
  *        
  * @param  none.
  *	
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseVerifyAll(void)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FTMRE_CMD_ERASE_VERIFY_ALL;// erase verify all flash & EEPROM blocks
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
#if 0	
	if(FTMRE_FERSTAT & (FTMRE_FERSTAT_SFDIF_MASK))
	{
		u16Err |= EEPROM_ERR_SINGLE_BIT_FAULT;
	}
	if(FTMRE_FERSTAT & (FTMRE_FERSTAT_DFDIF_MASK))
	{
		u16Err |= EEPROM_ERR_DOUBLE_BIT_FAULT;
	}

	if(FTMRE_FSTAT & FTMRE_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
#endif	
	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief erase all flash and EEPROM blocks.
  *        
  * @param  none.
  *	
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseAll(void)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FTMRE_CMD_ERASE_ALL;// erase  all flash & EEPROM blocks
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
#if 0	
	if(FTMRE_FERSTAT & (FTMRE_FERSTAT_SFDIF_MASK))
	{
		u16Err |= EEPROM_ERR_SINGLE_BIT_FAULT;
	}
	if(FTMRE_FERSTAT & (FTMRE_FERSTAT_DFDIF_MASK))
	{
		u16Err |= EEPROM_ERR_DOUBLE_BIT_FAULT;
	}
#endif	

	if(FTMRE_FSTAT & FTMRE_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	
	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief program once command, write data buff to IFR.
  *        
  * @param[in]  u8PhraseIndex phrase index.
  * @param[in]  pData8Bytes  programming data pointer.
  *
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_ProgramOnce(uint8_t u8PhraseIndex, uint8_t *pData8Bytes)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	int i;
	
	// Check the phrase index to if it is out of boundary
	if(u8PhraseIndex > 7)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[17:16]	
	FTMRE_FCCOBHI = FLASH_CMD_PROGRAMONCE;// command
	FTMRE_FCCOBLO = 0;// memory address bits[17:16]
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRE_FCCOBIX = 0x1;
	// Write the phrase index
	FTMRE_FCCOBLO = u8PhraseIndex;
	FTMRE_FCCOBHI = u8PhraseIndex>>8;
	
	// Write 4 words
	for(i = 0; i < 4; i++)
	{
		// Write index to specify the word (MSB word) to be programmed
		FTMRE_FCCOBIX = 0x2+i;
		// Write the word 0
		FTMRE_FCCOBHI = ((uint16_t)pData8Bytes[(i<<1)+1]);
                FTMRE_FCCOBLO = pData8Bytes[(i<<1)];
	}
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
	
	return (u16Err);
}
/*****************************************************************************//*!
  *
  * @brief read data from IFR address.
  *        
  * @param[in]  u8PhraseIndex phrase index.
  * @param[out]  pData8Bytes  read data pointer.
  *	
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_ReadOnce(uint8_t u8PhraseIndex, uint8_t *pData8Bytes)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	int i;
	
	// Check the phrase index to if it is out of boundary
	if(u8PhraseIndex > 7)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[17:16]	
	FTMRE_FCCOBHI = FLASH_CMD_READONCE;// command
	FTMRE_FCCOBLO = 0;// memory address bits[17:16]
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRE_FCCOBIX = 0x1;
	// Write the phrase index
	FTMRE_FCCOBLO = u8PhraseIndex;
	FTMRE_FCCOBHI = u8PhraseIndex >>8;
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	// Read 4 words
	for(i = 0; i < 4; i++)
	{
		// Read the word (MSB word) indexed by FCCOBIX 
		FTMRE_FCCOBIX = 0x2+i;
		pData8Bytes[i<<1] = FTMRE_FCCOBLO;
		pData8Bytes[(i<<1)+1] = FTMRE_FCCOBHI;
	}	
	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}

	return (u16Err);
}


/*****************************************************************************//*!
  *
  * @brief erase flash/Flash block without wait.
  *        
  * @param[in] u32NVMTargetAddress: target address in NVM.  
  * @param[in] bIsEEPROM: specify the target address is within EEPROM, 
  *						1 if in EEPROM block, 0 if in FLASH block. 
  *					
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseBlock_NoWait(uint32_t u32NVMTargetAddress, uint8_t bIsEEPROM)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FTMRE_CMD_ERASE_BLOCK;// erase FLASH block command
	FTMRE_FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block
	if(bIsEEPROM)
	{
		FTMRE_FCCOBLO |= 0x80;	// bit 23 = 1 for EEPROM block
	}
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRE_FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRE_FCCOBLO = u32NVMTargetAddress;
	FTMRE_FCCOBHI = u32NVMTargetAddress>>8;
	
	// Launch the command
	FTMRE_FSTAT = 0x80;

	return (u16Err);
}


/*****************************************************************************//*!
  *
  * @brief verify backdoor key. NOTE: this routine must be running from RAM or EEPROM.
  *        
  * @param[in] u32NVMTargetAddress: target address in NVM, must be longword aligned.  
  * @param[in] pKey:   pointer to 8-byte key.
  *	
  * @return error status.
  *
  * @ Pass/ Fail criteria: none
*****************************************************************************/
 
uint16_t FLASH_VerifyBackdoorKey(uint8_t *pKey)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
    uint16_t i;
        
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FLASH_CMD_BACKDOOR_ACCESS;// erase verify FLASH section command
	FTMRE_FCCOBLO = 0;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block
        // Write index to specify the lower byte memory address bits[15:0] to be loaded
 
        for(i = 0; i < 4; i++)
        {
            FTMRE_FCCOBIX = 0x1+i;
            FTMRE_FCCOBLO = *pKey++;
            FTMRE_FCCOBHI = *pKey++;
        }
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);

	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	

	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief set user margin level.
  *        
  * @param[in] u32NVMTargetAddress: target address in NVM, must be longword aligned.  
  * @param[in] u16MarginLevel:  user margin level.
  * @param[in] bIsEEPROM:  true if EEPROM address, false if flash address.
  *       
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
  *
*****************************************************************************/
 
uint16_t FLASH_SetUserMarginLevel(uint32_t u32NVMTargetAddress, uint16_t u16MarginLevel, uint8_t bIsEEPROM)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
 	if(u16MarginLevel > 2)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);		
	}
       
	// Clear error flags
	FTMRE_FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRE_FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRE_FCCOBHI = FLASH_CMD_SET_USER_MARGIN_LEVEL;// erase verify FLASH section command
	FTMRE_FCCOBLO = u32NVMTargetAddress >> 16;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block
	if(bIsEEPROM)
	{
		FTMRE_FCCOBLO |= 0x80;	// bit 23 = 1 for EEPROM block
	}
	FTMRE_FCCOBIX = 0x1;
	FTMRE_FCCOBHI = u32NVMTargetAddress >> 8;//
	FTMRE_FCCOBLO = u32NVMTargetAddress;

        FTMRE_FCCOBIX = 0x2;
	FTMRE_FCCOBHI = u16MarginLevel >> 8;//
	FTMRE_FCCOBLO = u16MarginLevel;

	// Launch the command
	FLASH_LaunchCMD(TRUE);

	// Check error status
	if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}

	return (u16Err);
}
/*****************************************************************************//*!
  *
  * @brief check flash error status.
  *        
  * @param none.
  *       
  * @return flash error status.
  *
  * @ Pass/ Fail criteria: none.
  *
*****************************************************************************/
uint16_t FLASH_CheckErrStatus(void)
{
    uint16_t u16Err = FLASH_ERR_SUCCESS;  
    if(FTMRE_FSTAT & FTMRE_FSTAT_ACCERR_MASK)
    {
            u16Err |= FLASH_ERR_ACCESS;
    }
    if(FTMRE_FSTAT & FTMRE_FSTAT_FPVIOL_MASK)
    {
            u16Err |= FLASH_ERR_PROTECTION;		
    }
    if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
    {
            u16Err |= FLASH_ERR_MGSTAT0;		
    }
    if(FTMRE_FSTAT & FTMRE_FSTAT_MGSTAT_MASK)
    {
            u16Err |= FLASH_ERR_MGSTAT1;		
    }
    return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief launch NVM command. if bWaitComplete is true, then wait till command
  *        is completed. It should be placed in RAM for some commands like
  *        verify backdoor key if enabling flash controller bit is cleared. Otherwise,
  *        it can be executed in flash.
  *        
  * @param[in] bWaitComplete:  true if need wait till command completed before return.  
  *	
  *
  * @return none
  *
  * @ Pass/ Fail criteria: none
*****************************************************************************/
#if (defined(IAR))
__ramfunc  void FLASH_LaunchCMD(uint8_t bWaitComplete)
#else
void FLASH_LaunchCMD(uint8_t bWaitComplete)
#endif
{
#if     defined(FLASH_ENABLE_STALLING_FLASH_CONTROLLER)
     MCM_PLACR |= MCM_PLACR_ESFC_MASK;          /* enable stalling flash controller when flash is busy */
#endif
    FTMRE_FSTAT = 0x80;
    if(bWaitComplete)
    {
      // Wait till command is completed
      while (!(FTMRE_FSTAT & FTMRE_FSTAT_CCIF_MASK));
    }
}
#elif defined(MCU_SKEAZN642)
/*****************************************************************************//*!
  *
  * @brief initialize flash driver.
  *        
  * @param[in]  u32BusClock current bus clock.
  *
  * @return flash error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_Init(uint32_t u32BusClock)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	uint8_t clkDIV = u32BusClock/1000000L - 1;
	
	if(!(FTMRH->FSTAT & FTMRH_FSTAT_CCIF_MASK))
	{
		u16Err |= FLASH_ERR_INIT_CCIF;
		return u16Err;
	}
	/* initialize the flash clock to be within spec 1MHz 
	 * 
	 */
	if(!(FTMRH->FCLKDIV & FTMRH_FCLKDIV_FDIVLCK_MASK))
	{
		/* FCLKDIV register is not locked */
		if((FTMRH->FCLKDIV & FTMRH_FCLKDIV_FDIVLD_MASK) && 
                  ((FTMRH->FCLKDIV & FTMRH_FCLKDIV_FDIV_MASK) != FTMRH_FCLKDIV_FDIV(clkDIV)))
		{
			/* flash clock prescaler is loaded but with wrong value */
			u16Err |= FLASH_ERR_INIT_FDIV;
			return (u16Err);
		}
		FTMRH->FCLKDIV = (FTMRH->FCLKDIV & ~(FTMRH_FCLKDIV_FDIV_MASK)) | FTMRH_FCLKDIV_FDIV(clkDIV);
                
#if 0
		FTMRH->FCLKDIV  |= FTMRH_FCLKDIV_FDIVLCK_MASK; /* lock the prescaler */
#endif
	}
	else
	{
		/* FCLKDIV register is locked */
		if((FTMRH->FCLKDIV & FTMRH_FCLKDIV_FDIV_MASK) != FTMRH_FCLKDIV_FDIV(clkDIV))
		{
			/* flash clock prescaler is wrong */
			u16Err |= FLASH_ERR_INIT_FDIV;
		}
	}
	return (u16Err);	
}

/*****************************************************************************//*!
  *
  * @brief program flash routine.
  *        
  * @param[in]   u32NVMTargetAddress programed flash address.
  * @param[in]   pData programming data pointer.
  * @param[in]   u16SizeBytes programming data length.
  *
  * @return flash error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_Program(uint32_t u32NVMTargetAddress, uint8_t *pData, uint16_t u16SizeBytes)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	uint16_t u16w2LongWordCount = u16SizeBytes>>3;
	uint8_t  u8WrLeftBytes = (u16SizeBytes & 0x07);
	uint16_t u16WrLeftLongWords = u8WrLeftBytes>>2;
	uint32_t u32WrTargetAddress = u32NVMTargetAddress;
	uint32_t u32DwData0,u32DwData1;
	uint32_t *pDwData = (uint32_t*)pData;
	int  i;
	
	// Check address to see if it is aligned to 4 bytes
	// Global address [1:0] must be 00.
	if(u32NVMTargetAddress & 0x03)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Loop for the two longwords (8 bytes) programming
	for(i = 0; i < u16w2LongWordCount; i++)
	{
		u32DwData0 = *pDwData++;
		u32DwData1 = *pDwData++;
		u16Err = FLASH_Program2LongWords(u32WrTargetAddress, u32DwData0, u32DwData1);
		if(u16Err)
		{
			goto EndP;
			//break;
		}
		u32WrTargetAddress += 8;
	}
	// Loop for the single longword (4 bytes) programming
	for(i = 0; i < u16WrLeftLongWords; i++)
	{
		u32DwData0 = *pDwData++;
		u16Err = FLASH_Program1LongWord(u32WrTargetAddress, u32DwData0);
		if(u16Err)
		{			
			goto EndP;
			//break;
		}
		u32WrTargetAddress += 4;
	}
	u8WrLeftBytes = (u8WrLeftBytes-(u16WrLeftLongWords<<2));	// calculate the # of bytes that are not programmed
	if(!u8WrLeftBytes){
		return (u16Err);
	}
        
#if     defined(BIG_ENDIAN)                
	u32DwData0 = 0;
	pData = (uint8_t*)pDwData;	// pointer to the left bytes
	for(i = u8WrLeftBytes; i >0; i--)
	{
		u32DwData0 <<= 8;
		u32DwData0 |= *pData++;	// MSB byte first
	}
	// Calculate how many bytes need to be filled with 0xFFs
	// in order to form a single longword for the left bytes of data
	u8WrLeftBytes = 4 - u8WrLeftBytes;	
	//  
	for(i = u8WrLeftBytes; i >0; i--)
	{
		u32DwData0 <<= 8;
		u32DwData0 |= 0xFF;	// MSB byte first
	}
#else
	u32DwData0 = 0xFFFFFFFFL;        
	pData = (uint8_t*)pDwData+u8WrLeftBytes-1;	// pointer to the left bytes
	for(i = u8WrLeftBytes; i >0; i--)
	{
		u32DwData0 <<= 8;
		u32DwData0 |= *pData--;	// MSB byte first
	}
#endif	
	// Now program the last longword
	u16Err = FLASH_Program1LongWord(u32WrTargetAddress, u32DwData0);	
EndP:	
	return (u16Err);
}
/*****************************************************************************//*!
  *
  * @brief program flash routine, program 1 long word to flash.
  *        
  * @param[in]   u32NVMTargetAddress programed flash address.
  * @param[in]   u32DwData programming data.
  *
  * @return flash error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_Program1LongWord(uint32_t u32NVMTargetAddress, uint32_t u32DwData)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Check address to see if it is aligned to 4 bytes
	// Global address [1:0] must be 00.
	if(u32NVMTargetAddress & 0x03)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FLASH_CMD_PROGRAM;// program FLASH command
	FTMRH->FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16]
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRH->FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRH->FCCOBLO = u32NVMTargetAddress;
	FTMRH->FCCOBHI = u32NVMTargetAddress>>8;
	// Write index to specify the word0 (MSB word) to be programmed
	FTMRH->FCCOBIX = 0x2;
#if     defined(BIG_ENDIAN)        
	// Write the word  0
	FTMRH->FCCOBHI = (u32DwData>>16)>>8;
	FTMRH->FCCOBLO = (u32DwData>>16);
#else        
	FTMRH->FCCOBHI = (u32DwData) >>8;	
	FTMRH->FCCOBLO = (u32DwData);	
#endif        
	// Write index to specify the word1 (LSB word) to be programmed
	FTMRH->FCCOBIX = 0x3;
	// Write the word1 
#if     defined(BIG_ENDIAN)        
	FTMRH->FCCOBHI = (u32DwData) >>8;	
	FTMRH->FCCOBLO = (u32DwData);	
#else
	FTMRH->FCCOBHI = (u32DwData>>16)>>8;
	FTMRH->FCCOBLO = (u32DwData>>16);
#endif        
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
	
	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief program flash routine, program 2long word to flash.
  *        
  * @param[in]   u32NVMTargetAddress programed flash address.
  * @param[in]   u32DwData0 programming data0.
  * @param[in]   u32DwData1 programming data1.
  *
  * @return flash error status.
  *
  * @ Pass/ Fail criteria: none
*****************************************************************************/

uint16_t FLASH_Program2LongWords(uint32_t u32NVMTargetAddress, uint32_t u32DwData0, uint32_t u32DwData1)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Check address to see if it is aligned to 4 bytes
	// Global address [1:0] must be 00.
	if(u32NVMTargetAddress & 0x03)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FLASH_CMD_PROGRAM;// program FLASH command
	FTMRH->FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16]
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRH->FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRH->FCCOBLO = u32NVMTargetAddress;
	FTMRH->FCCOBHI = u32NVMTargetAddress>>8;
        
	// Write index to specify the word0 (MSB word) to be programmed
	FTMRH->FCCOBIX = 0x2;
	// Write the word 0
#if     defined(BIG_ENDIAN)                
	//FTMRH_FCCOB = (u32DwData0>>16) & 0xFFFF;
	FTMRH->FCCOBHI = (u32DwData0>>16)>>8;
	FTMRH->FCCOBLO = (u32DwData0>>16);
        
	// Write index to specify the word1 (LSB word) to be programmed
	FTMRH->FCCOBIX = 0x3;
	// Write word 1
	//FTMRH_FCCOB = (u32DwData0) & 0xFFFF;
	FTMRH->FCCOBHI = (u32DwData0) >>8;
	FTMRH->FCCOBLO = (u32DwData0);
	
	// Write index to specify the word0 (MSB word) to be programmed
	FTMRH->FCCOBIX = 0x4;
	// Write the word2
	//FTMRH_FCCOB = (u32DwData1>>16) & 0xFFFF;
	FTMRH->FCCOBHI = (u32DwData1>>16)>>8;
	FTMRH->FCCOBLO = (u32DwData1>>16);
        
	// Write index to specify the word1 (LSB word) to be programmed
	FTMRH->FCCOBIX = 0x5;
	// Write word 3
	//FTMRH_FCCOB = (u32DwData1) & 0xFFFF;
	FTMRH->FCCOBHI = (u32DwData1) >>8;
	FTMRH->FCCOBLO = (u32DwData1);
#else
	//FTMRH_FCCOB = (u32DwData0) & 0xFFFF;
	FTMRH->FCCOBHI = (u32DwData0) >>8;
	FTMRH->FCCOBLO = (u32DwData0);
        
	// Write index to specify the word1 (LSB word) to be programmed
	FTMRH->FCCOBIX = 0x3;
	// Write word 1
	FTMRH->FCCOBHI = (u32DwData0>>16)>>8;
	FTMRH->FCCOBLO = (u32DwData0>>16);
	
	// Write index to specify the word0 (MSB word) to be programmed
	FTMRH->FCCOBIX = 0x4;
	// Write the word2
	//FTMRH_FCCOB = (u32DwData1) & 0xFFFF;
	FTMRH->FCCOBHI = (u32DwData1) >>8;
	FTMRH->FCCOBLO = (u32DwData1);        
        
	// Write index to specify the word1 (LSB word) to be programmed
	FTMRH->FCCOBIX = 0x5;
	// Write word 3
	//FTMRH_FCCOB = (u32DwData1>>16) & 0xFFFF;
	FTMRH->FCCOBHI = (u32DwData1>>16)>>8;
	FTMRH->FCCOBLO = (u32DwData1>>16);
#endif
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
	
	return (u16Err);
}



/*****************************************************************************//*!
  *
  * @brief erase flash sector, each flash sector is of 512 bytes long.
  *      
  *        
  * @param[in]   u32NVMTargetAddress erase sector address.
  *
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseSector(uint32_t u32NVMTargetAddress)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Check address to see if it is aligned to 4 bytes
	// Global address [1:0] must be 00.
	if(u32NVMTargetAddress & 0x03)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FLASH_CMD_ERASE_SECTOR;// EEPROM FLASH command
	FTMRH->FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16]
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRH->FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRH->FCCOBLO = u32NVMTargetAddress;
	FTMRH->FCCOBHI = u32NVMTargetAddress>>8;
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}
	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief unsecure device with unsecure command.
  * 
  *        
  * @param none. 
  *
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_Unsecure(void)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FTMRH_CMD_UNSECURE;// EEPROM FLASH command
	FTMRH->FCCOBLO = 0;// memory address bits[23:16]
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
	return (u16Err);
}


/*****************************************************************************//*!
  *
  * @brief erase flash/EEPROM block.
  *        
  * @param[in] u32NVMTargetAddress: target address in NVM. 
  * @param[in] bIsEEPROM: specify the target address is within EEPROM,1 if in EEPROM block, 0 if in FLASH block.  
  *					
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseBlock(uint32_t u32NVMTargetAddress, uint8_t bIsEEPROM)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FTMRH_CMD_ERASE_BLOCK;// erase FLASH block command
	FTMRH->FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block
	if(bIsEEPROM)
	{
		FTMRH->FCCOBLO |= 0x80;	// bit 23 = 1 for EEPROM block
	}
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRH->FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRH->FCCOBLO = u32NVMTargetAddress;
	FTMRH->FCCOBHI = u32NVMTargetAddress>>8;
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}
	return (u16Err);
}


/*****************************************************************************//*!
  *
  * @brief erase verify flash/EEPROM block.
  *        
  * @param[in] u32NVMTargetAddress: target address in NVM. 
  * @param[in] bIsEEPROM: specify the target address is within EEPROM,1 if in EEPROM block, 0 if in FLASH block.  
  *
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseVerifyBlock(uint32_t u32NVMTargetAddress, uint8_t bIsEEPROM)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FTMRH_CMD_ERASE_VERIFY_BLOCK;// erase FLASH block command
	FTMRH->FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block
	if(bIsEEPROM)
	{
		FTMRH->FCCOBLO |= 0x80;	// bit 23 = 1 for EEPROM block
	}
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRH->FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRH->FCCOBLO = u32NVMTargetAddress;
	FTMRH->FCCOBHI = u32NVMTargetAddress>>8;
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief erase verify flash section.
  *        
  * @param[in]  u32NVMTargetAddress: target address in NVM, must be longword aligned. 
  * @param[in]  u16LongWordCount:   number of longwords to be verified.
  *	
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseVerifySection(uint32_t u32NVMTargetAddress, uint16_t u16LongWordCount)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	// Check address to see if it is aligned to 4 bytes
	// Global address [1:0] must be 00.
	if(u32NVMTargetAddress & 0x03)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}	
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FLASH_CMD_ERASE_VERIFY_SECTION;// erase verify FLASH section command
	FTMRH->FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRH->FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRH->FCCOBLO = u32NVMTargetAddress;
	FTMRH->FCCOBHI = u32NVMTargetAddress>>8;

	// Write index to specify the # of longwords to be verified
	FTMRH->FCCOBIX = 0x2;
	// Write the # of longwords 
	FTMRH->FCCOBLO = u16LongWordCount;
	FTMRH->FCCOBHI = u16LongWordCount>>8;
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief erase verify all flash and EEPROM blocks.
  *        
  * @param  none.
  *	
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseVerifyAll(void)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FTMRH_CMD_ERASE_VERIFY_ALL;// erase verify all flash & EEPROM blocks
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
#if 0	
	if(FTMRH->FERSTAT & (FTMRH_FERSTAT_SFDIF_MASK))
	{
		u16Err |= EEPROM_ERR_SINGLE_BIT_FAULT;
	}
	if(FTMRH->FERSTAT & (FTMRH_FERSTAT_DFDIF_MASK))
	{
		u16Err |= EEPROM_ERR_DOUBLE_BIT_FAULT;
	}

	if(FTMRH->FSTAT & FTMRH_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
#endif	
	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief erase all flash and EEPROM blocks.
  *        
  * @param  none.
  *	
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseAll(void)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FTMRH_CMD_ERASE_ALL;// erase  all flash & EEPROM blocks
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
#if 0	
	if(FTMRH->FERSTAT & (FTMRH_FERSTAT_SFDIF_MASK))
	{
		u16Err |= EEPROM_ERR_SINGLE_BIT_FAULT;
	}
	if(FTMRH->FERSTAT & (FTMRH_FERSTAT_DFDIF_MASK))
	{
		u16Err |= EEPROM_ERR_DOUBLE_BIT_FAULT;
	}
#endif	

	if(FTMRH->FSTAT & FTMRH_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	
	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief program once command, write data buff to IFR.
  *        
  * @param[in]  u8PhraseIndex phrase index.
  * @param[in]  pData8Bytes  programming data pointer.
  *
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_ProgramOnce(uint8_t u8PhraseIndex, uint8_t *pData8Bytes)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	int i;
	
	// Check the phrase index to if it is out of boundary
	if(u8PhraseIndex > 7)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[17:16]	
	FTMRH->FCCOBHI = FLASH_CMD_PROGRAMONCE;// command
	FTMRH->FCCOBLO = 0;// memory address bits[17:16]
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRH->FCCOBIX = 0x1;
	// Write the phrase index
	FTMRH->FCCOBLO = u8PhraseIndex;
	FTMRH->FCCOBHI = u8PhraseIndex>>8;
	
	// Write 4 words
	for(i = 0; i < 4; i++)
	{
		// Write index to specify the word (MSB word) to be programmed
		FTMRH->FCCOBIX = 0x2+i;
		// Write the word 0
		FTMRH->FCCOBHI = ((uint16_t)pData8Bytes[(i<<1)+1]);
                FTMRH->FCCOBLO = pData8Bytes[(i<<1)];
	}
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	
	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
	
	return (u16Err);
}
/*****************************************************************************//*!
  *
  * @brief read data from IFR address.
  *        
  * @param[in]  u8PhraseIndex phrase index.
  * @param[out]  pData8Bytes  read data pointer.
  *	
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/

uint16_t FLASH_ReadOnce(uint8_t u8PhraseIndex, uint8_t *pData8Bytes)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	int i;
	
	// Check the phrase index to if it is out of boundary
	if(u8PhraseIndex > 7)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);
	}
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[17:16]	
	FTMRH->FCCOBHI = FLASH_CMD_READONCE;// command
	FTMRH->FCCOBLO = 0;// memory address bits[17:16]
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRH->FCCOBIX = 0x1;
	// Write the phrase index
	FTMRH->FCCOBLO = u8PhraseIndex;	
	FTMRH->FCCOBHI = u8PhraseIndex >>8;	
	// Launch the command
	FLASH_LaunchCMD(TRUE);
	// Read 4 words
	for(i = 0; i < 4; i++)
	{
		// Read the word (MSB word) indexed by FCCOBIX 
		FTMRH->FCCOBIX = 0x2+i;
		pData8Bytes[i<<1] = FTMRH->FCCOBLO; 
		pData8Bytes[(i<<1)+1] = FTMRH->FCCOBHI; 
	}	
	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_FPVIOL_MASK)
	{
		u16Err |= FLASH_ERR_PROTECTION;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}

	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief erase flash/Flash block without wait.
  *        
  * @param[in] u32NVMTargetAddress: target address in NVM.  
  * @param[in] bIsEEPROM: specify the target address is within EEPROM, 
  *						1 if in EEPROM block, 0 if in FLASH block. 
  *					
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
*****************************************************************************/
uint16_t FLASH_EraseBlock_NoWait(uint32_t u32NVMTargetAddress, uint8_t bIsEEPROM)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
	
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FTMRH_CMD_ERASE_BLOCK;// erase FLASH block command
	FTMRH->FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block
	if(bIsEEPROM)
	{
		FTMRH->FCCOBLO |= 0x80;	// bit 23 = 1 for EEPROM block
	}
	// Write index to specify the lower byte memory address bits[15:0] to be loaded
	FTMRH->FCCOBIX = 0x1;
	// Write the lower byte memory address bits[15:0]
	FTMRH->FCCOBLO = u32NVMTargetAddress;
	FTMRH->FCCOBHI = u32NVMTargetAddress>>8;
	
	// Launch the command
	FTMRH->FSTAT = 0x80;

	return (u16Err);
}


/*****************************************************************************//*!
  *
  * @brief verify backdoor key. NOTE: this routine must be running from RAM or EEPROM.
  *        
  * @param[in] u32NVMTargetAddress: target address in NVM, must be longword aligned.  
  * @param[in] pKey:   pointer to 8-byte key.
  *	
  * @return error status.
  *
  * @ Pass/ Fail criteria: none
*****************************************************************************/
 
uint16_t FLASH_VerifyBackdoorKey(uint8_t *pKey)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
    uint16_t i;
        
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FLASH_CMD_BACKDOOR_ACCESS;// erase verify FLASH section command
	FTMRH->FCCOBLO = 0;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block	
        // Write index to specify the lower byte memory address bits[15:0] to be loaded
 
        for(i = 0; i < 4; i++)
        {
            FTMRH->FCCOBIX = 0x1+i;
            FTMRH->FCCOBLO = *pKey++;
            FTMRH->FCCOBHI = *pKey++;      
        }
	
	// Launch the command
	FLASH_LaunchCMD(TRUE);

	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}	
	return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief set user margin level.
  *        
  * @param[in] u32NVMTargetAddress: target address in NVM, must be longword aligned.  
  * @param[in] u16MarginLevel:  user margin level.
  * @param[in] bIsEEPROM:  true if EEPROM address, false if flash address.
  *       
  * @return error status.
  *
  * @ Pass/ Fail criteria: none.
  *
*****************************************************************************/
 
uint16_t FLASH_SetUserMarginLevel(uint32_t u32NVMTargetAddress, uint16_t u16MarginLevel, uint8_t bIsEEPROM)
{
	uint16_t u16Err = FLASH_ERR_SUCCESS;
 	if(u16MarginLevel > 2)
	{
		u16Err = FLASH_ERR_INVALID_PARAM;
		return (u16Err);		
	}
       
	// Clear error flags
	FTMRH->FSTAT = 0x30;
	
	// Write index to specify the command code to be loaded
	FTMRH->FCCOBIX = 0x0;
	// Write command code and memory address bits[23:16]	
	FTMRH->FCCOBHI = FLASH_CMD_SET_USER_MARGIN_LEVEL;// erase verify FLASH section command
	FTMRH->FCCOBLO = u32NVMTargetAddress >> 16;// memory address bits[23:16] with bit23 = 0 for Flash block, 1 for EEPROM block	
	if(bIsEEPROM)
	{
		FTMRH->FCCOBLO |= 0x80;	// bit 23 = 1 for EEPROM block
	}
	FTMRH->FCCOBIX = 0x1;
	FTMRH->FCCOBHI = u32NVMTargetAddress >> 8;// 
	FTMRH->FCCOBLO = u32NVMTargetAddress;	

        FTMRH->FCCOBIX = 0x2;
	FTMRH->FCCOBHI = u16MarginLevel >> 8;// 
	FTMRH->FCCOBLO = u16MarginLevel;	

	// Launch the command
	FLASH_LaunchCMD(TRUE);

	// Check error status
	if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
	{
		u16Err |= FLASH_ERR_ACCESS;
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT0;		
	}
	if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
	{
		u16Err |= FLASH_ERR_MGSTAT1;		
	}
	return (u16Err);
}
/*****************************************************************************//*!
  *
  * @brief check flash error status.
  *        
  * @param none.
  *       
  * @return flash error status.
  *
  * @ Pass/ Fail criteria: none.
  *
*****************************************************************************/
uint16_t FLASH_CheckErrStatus(void)
{
    uint16_t u16Err = FLASH_ERR_SUCCESS;  
    if(FTMRH->FSTAT & FTMRH_FSTAT_ACCERR_MASK)
    {
            u16Err |= FLASH_ERR_ACCESS;
    }
    if(FTMRH->FSTAT & FTMRH_FSTAT_FPVIOL_MASK)
    {
            u16Err |= FLASH_ERR_PROTECTION;		
    }
    if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT0_MASK)
    {
            u16Err |= FLASH_ERR_MGSTAT0;		
    }
    if(FTMRH->FSTAT & FTMRH_FSTAT_MGSTAT1_MASK)
    {
            u16Err |= FLASH_ERR_MGSTAT1;		
    }
    return (u16Err);
}

/*****************************************************************************//*!
  *
  * @brief launch NVM command. if bWaitComplete is true, then wait till command
  *        is completed. It should be placed in RAM for some commands like
  *        verify backdoor key if enabling flash controller bit is cleared. Otherwise,
  *        it can be executed in flash.
  *        
  * @param[in] bWaitComplete:  true if need wait till command completed before return.  
  *	
  *
  * @return none
  *
  * @ Pass/ Fail criteria: none
*****************************************************************************/
void FLASH_LaunchCMD(uint8_t bWaitComplete)
{
#if     defined(FLASH_ENABLE_STALLING_FLASH_CONTROLLER)
     MCM->PLACR |= MCM_PLACR_ESFC_MASK;          /* enable stalling flash controller when flash is busy */
#endif
    FTMRH->FSTAT = 0x80;    
    if(bWaitComplete)
    {
      // Wait till command is completed
      while (!(FTMRH->FSTAT & FTMRH_FSTAT_CCIF_MASK));
    }
}
#else
/* Write the flash memory implementation for your device. */
#endif
/*! @} End of nvm_api_list                                               						*/

