/*
 * flash.h
 * Created on: 15-Jul-2024
 * Author: Le Huu An
 * Used for Phase detect remote
 */

#ifndef __FLASH_H
#define __FLASH_H
 
#include "main.h"

#include <string.h>
#include "user.h"

/*DEFINE*/
	#define ADDRESS_DATA_STORAGE 0x800FC00
	
	#define UPDATE_ERROR 0x4E
	#define UPDATE_SUCCESS 0x59
	
	#define PASSWORD_UPDATE 0xACCA
/************************************************************************************/
/*DECLARE STRUCT*/

	#pragma pack(1)
	typedef struct{
		uint16_t pw;
		uint16_t id;
        uint16_t zero_pA;
        uint16_t zero_pB;
        uint16_t zero_pC;
        uint16_t zero_pS;
	}info_t;
	#pragma pack()


/*DECLARE FUNCTION*/
	
	void Flash_Erase(uint32_t address);
	
	void Flash_Write_Array(uint32_t address, uint8_t *arr, uint16_t len);
	
	void Flash_Write_Struct(uint32_t address, info_t data);
	
	void Flash_Read_Array(uint32_t address, uint8_t *arr, uint16_t len);
	
	void Flash_Read_Struct(uint32_t address, info_t *data);

	void Load_Infor_Func(void);
	
	uint8_t Update_NEW_Infor(void);

/*EXTERN*/

	extern info_t vInfor_default;
	extern info_t vInfor_cache;	
/************************************************************************************/
#endif /* FLASH_H*/


