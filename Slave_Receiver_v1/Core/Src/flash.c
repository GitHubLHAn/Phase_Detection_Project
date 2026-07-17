/*
 * flash.c
 * Created on: 15-Jul-2024
 * Author: Le Huu An
 * Used for Phase detect remote
 */

#include "flash.h"

/**********************************************************************************************************************************/
/*******DECLARE VARIABLE********/
	chute_info_t vInfor_default = {
        .pw = PASSWORD_UPDATE;
        .id = 0;
        .zero_pA = 2000;
        .zero_pB = 2000;
        .zero_pC = 2000;
        .zero_pD = 2000;
    };
	chute_info_t vInfor_cache;	

/**********************************************************************************************************************************/
/*******FUNCTION********/

//================================================================================*/
// Flash erase a sector
//================================================================================*/

void Flash_Erase(uint32_t address)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.Banks = 1;
    EraseInitStruct.NbPages = 1;
    EraseInitStruct.PageAddress = address;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    uint32_t page_err;
    HAL_FLASHEx_Erase(&EraseInitStruct, &page_err);
    HAL_FLASH_Lock();
}

//================================================================================*/
// Write an array into the flash
//================================================================================*/
void Flash_Write_Array(uint32_t address, uint8_t *arr, uint16_t len)
{
    uint16_t *pt = (uint16_t*)arr;
    HAL_FLASH_Unlock();
    for(uint8_t i=0; i<(len+1)/2; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 2*i, *pt);
        pt++;
    }
    HAL_FLASH_Lock();
}

//================================================================================*/
// Write a struct into the flash
//================================================================================*/

void Flash_Write_Struct(uint32_t address, info_t pDATA)
{
    Flash_Write_Array(address, (uint8_t*)&pDATA, sizeof(pDATA));	
}

//================================================================================*/
// Read an array from the flash
//================================================================================*/

void Flash_Read_Array(uint32_t address, uint8_t *arr, uint16_t len)
{
    uint16_t *pt = (uint16_t*)arr;
    for(uint8_t i=0; i<(len+1)/2; i++)
    {
        *pt = *(__IO uint16_t*)(address + 2*i);
        pt++;
    }
}

//================================================================================*/
// Read a struct from the flash
//================================================================================*/

void Flash_Read_Struct(uint32_t address, info_t *pDATA)
{
    Flash_Read_Array(address, (uint8_t*)pDATA, sizeof(info_t));
}

//================================================================================*/
// Read information of chute from flash if the password right
// Otherwise write the default information into the flash
//================================================================================*/

void Load_Infor_Func(void)
{
    Init_Chute_Infor(&vInfor_default);
    
    Flash_Read_Struct(ADDRESS_DATA_STORAGE, &vInfor_cache);
    if(vChute_Infor_cache.pw == PASSWORD_UPDATE)
    {
        //data ok
        return;
    }
    else		// no ever flash or change new version
    {			
        Flash_Erase(ADDRESS_DATA_STORAGE);
        HAL_Delay(50);
        Flash_Write_Struct(ADDRESS_DATA_STORAGE, vInfor_default);
    
        OFF_LED_DEBUG();
        ON_LED_DEBUG(); 	HAL_Delay(100);
        OFF_LED_DEBUG(); 	HAL_Delay(500);
        ON_LED_DEBUG(); 	HAL_Delay(100);
        OFF_LED_DEBUG(); 	HAL_Delay(500);
        ON_LED_DEBUG(); 	HAL_Delay(100);
        OFF_LED_DEBUG(); 	HAL_Delay(500);
        NVIC_System_Reset();
    }
}
	
/*================================================================================*/
/*												UPDATE NEW ADDRESS INTO FLASH														*/
/*================================================================================*/

uint8_t Update_NEW_Infor(void)
{
    uint8_t result = UPDATE_ERROR;

    __disable_irq();

    /*Clear Information in flash*/
    Flash_Erase(ADDRESS_DATA_STORAGE);
    HAL_Delay(10);
    
    /*Write struct to Flash*/
    Flash_Write_Struct(ADDRESS_DATA_STORAGE, vInfor_cache);
    HAL_Delay(10);
    /*Read and check again*/
    Flash_Read_Struct(ADDRESS_DATA_STORAGE, &vInfor_cache);
    HAL_Delay(10);
    
    if(memcmp(vChute_Infor_cache.identification_arr, Get_New_ID_arr, NUM_IDENTIFICATION) == 0){
        result = UPDATE_SUCCESS;
    }
    else{
        Flash_Erase(ADDRESS_DATA_STORAGE);
        HAL_Delay(10);
    }

    _enable_irq();

    return result;
}
	
	

/**********************************************************************************************************************************/
/*******END PAGE********/
