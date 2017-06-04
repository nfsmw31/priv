//###########################################################################
//
// FILE:    FlashHandle.c
//
// TITLE:   Flash routins for SAVE,LOAD,CLREEPROM ...
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.01| 02 DEC 2010| D.R. | Creation
//
//##################################################################

#include <string.h>

#include "FltCntrl.def"
#include "Err_Hndl.def"
#include "Flash.def"
#include "DevTools.def"
#include "Ser_Comm.def"
#include "Exe_IO.def"
#include "PtpGenerator.def"
#include "Drive_Table.def"
#include "CommFdbk.def"

#include "Extrn_Asm.var"
#include "nvram_tabl.var"
#include "FltCntrl.var"
#include "Flash.var"
#include "Drive.var"
#include "FlashHandle.var"
#include "Design.def"
#include "MotorSetup.var"
#include "Motor.var"
#include "User_Var.var"
#include "Record.var"
#include "init.var"
#include "Foldback.var"
#include "Ser_Comm.var"
#include "Exe_IO.var"
#include "PtpGenerator.var"
#include "AN_FLTR.var"
#include "Units.var"
#include "Exe_Hndl.var"
#include "ExFbVar.var"
#include "CommFdbk.var"

#include "Prototypes.pro"
#include "cal_conf.h"
#include "co_usr.h"

//**********************************************************
// Function Name: SalSaveToFlash
// Description:
//        This function is called in response to the SAVE command.
//      to save parameters to FLASH
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int SalSaveToFlash(int drive)
{
   static int i = 0, save_state = 0;
   int status = 0;
   long long s64_temp;
   
   // Avoid updating RUNtime if AT is in process. BZ5413
   PosTuneActiveCommand(&s64_temp,drive);
   if((int)s64_temp == 1)  return (AT_ACTIVE);
   
   if (s16_Flash_Fault_Flag == 1)
   {
// Example for debug log      LogDebugData(2);   // Log to debug sector in flash the way SAVE finished
      return  FLASH_INVALID;
   }

   if ((u16_Vendor_ID[0] == 0x0001) && (drive != 0))
   {
// Example for debug log      LogDebugData(3);   // Log to debug sector in flash the way SAVE finished
      return (NOT_SUPPORTED_ON_HW);  // axis 1 is not supported on Spansion
   }

   switch (save_state)
   {
      case 0:  // Initialization state
         u16_Flash_Params_Checksum = 0; // Ininitialize checksum

         i = 0; // initiliaze table index

         u16_Flash_Index = FLASH_PARAM_PARAMS_ADDRESS;

         p_Param_Ram_Table = (struct Variable_Address *)Nvram_Table_Axis_0;

         if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
         {
            u32_Param_Sector_Addr = (long)(FLASH_TYPE_1_PARAM_ADDR);
         }
         else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            u32_Param_Sector_Addr = (long)(AX0_PARAM_BLOCK) * FLASH_BLOCK_SIZE;
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            u32_Param_Sector_Addr = (long)(AX0_PARAM_SECTOR + (6 * drive)) * FLASH_SECTOR_SIZE;
         }

         s16_Save_Sector_Counter = 0;

         // copy port parametrs (CANopen communication segment) into drive parameters to be saved in FLASH
         CanCopyPortLibToFlashImg();

// Example for debug log         LogDebugData(1);   // Log to debug sector in flash that SAVE started

         save_state++;
      break;

      case 1: //prepare flash for writing
         if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
         {
            if (EraseBlock(AX0_FLASH_TYPE_1_PARAM_BLOCK) != FLASH_PENDING) save_state++;
         }
         else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            if (EraseBlock(AX0_PARAM_BLOCK) != FLASH_PENDING) save_state++;
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            if (EraseSector(AX0_PARAM_SECTOR + (6 * drive) + s16_Save_Sector_Counter) != FLASH_PENDING) s16_Save_Sector_Counter++;
            if (s16_Save_Sector_Counter == 5) save_state++;
         }
      break;

      case 2: // Each background save one parameter
         if (i < NUMBER_OF_NVRAM_VARIABLES)
         {
            //write value to FLASH
            status = WriteFlash(u32_Param_Sector_Addr, u16_Flash_Index, (unsigned int*)p_Param_Ram_Table[i].ram_ptr, p_Param_Ram_Table[i].size);
            if (status == FLASH_FAIL)  // equivalent to "s16_Flash_Fault_Flag = 1"
            {
               break;
            }

            u16_Flash_Index +=   p_Param_Ram_Table[i].size;

            i++; // increment i to next parameter to be saved
         }
         else
         {
            save_state++;
         }
      break;

      case 3:  // Write checksum and stamp
         status = FlashWriteWord(u16_Flash_Params_Checksum, u32_Param_Sector_Addr, FLASH_PARAMS_CHECKSUM_ADDRESS);

         // if there was a checksum fault make sure drive will not enable itself
         if ((BGVAR(s64_SysNotOk) & NVRAM_CHECKSUM_FAULT) != 0) DisableCommand(drive);

         // Clear the EEPROM checksum fault indication
         BGVAR(s64_SysNotOk) &= ~NVRAM_CHECKSUM_FAULT;

         save_state++;
      break;

      case 4:
         FlashWriteWord(FLASH_PARAM_STAMP_VALUE, u32_Param_Sector_Addr, FLASH_PARAM_STAMP_ADDRESS);

         /*if ( ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))      &&
              (u16_Flash_Type == 1)                                      &&
              (BGVAR(u16_P1_01_CTL_At_SPI_Flash_Mapped) != (BGVAR(u16_P1_01_CTL) & 0x00FF))         )
         {
            save_state++;
         }
         else*/
         {
            save_state = 0;
         }
      break;

      case 5:
         // LXM28E, need to store P1-01 (operating mode part) in the SPI Flash
         // Erase serial flash param sector
         status = SerialFlashEraseSector(PARAM_SECTOR_ADDR);
         if (status == FLASH_STATUS_SUCCESS)
         {
            save_state++;
         }
         else if (status == STATUS_FLASH_FAIL)
         {
            save_state = 0;
         }
      break;

      case 6:

         SerialFlashWriteByte(PARAM_P1_01_ADDR, (BGVAR(u16_P1_01_CTL) & 0x00FF));
         BGVAR(u16_P1_01_CTL_At_SPI_Flash) = BGVAR(u16_P1_01_CTL) & 0x00FF;
         save_state = 0;

      break;

   }

   if (s16_Flash_Fault_Flag == 1)                 // 1 indicates that FlashWriteWord() failed (FLASH_FAIL)
   {
      s16_Save_Flash_Fault_Acc_Counter++;         // count the accumulated number of SAVE failures
      s16_Save_Flash_Fault_Counter++;             // count the number of SAVE failures for retry
      if (s16_Save_Flash_Fault_Counter < 5)       // retry
      {
         s16_Flash_Fault_Flag = 0;                // remove the failure indication
         save_state = 0;                          // reset the SAVE state machine
// Example for debug log         LogDebugData(4);   // Log to debug sector in flash the way SAVE finished
         return (SAL_NOT_FINISHED);               // return as not finished for retry
      }
      else
      {
         save_state = 0;
// Example for debug log         LogDebugData(5);   // Log to debug sector in flash the way SAVE finished
         return SAVE_FAILED;
      }
   }

   if (save_state == 0)
   {
      s16_Save_Flash_Fault_Counter = 0;

      if((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
      {// for LX28/26 if save completed clear the Wn737.
         BGVAR(s16_Drive_Default_Config_Wrn_Flag) = 0;
      }

// Example for debug log      LogDebugData(6);   // Log to debug sector in flash the way SAVE finished
      return (SAL_SUCCESS);
   }
   else return (SAL_NOT_FINISHED);
}


//**********************************************************
// Function Name: LoadFromFlashStateMachine
// Description:
// Author: D.R.
// Algorithm:
// Revisions: nitsan: calling this function will load all params from nvram.
//                    no change to previous behavior.
//**********************************************************
int LoadFromFlashStateMachine(int drive)
{
   return LoadFromFlashStateMachineInternal(drive, LOAD_ALL_PARAMS);
}

//**********************************************************
// Function Name: LoadFromFlashStateMachineInternal
// Description: This function does the actual nvram loading
//              segment -  LOAD_ALL_PARAMS: load all parameters from nvram
//                         LOAD_CAN_COMM_PARAMS: load only the CANopen communication segment (0x1000 to 0x1fff)
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int LoadFromFlashStateMachineInternal(int drive, int segment)
{
   // AXIS_OFF;
   static int load_state = 0, i = 0;

   int i2 = 0;
   int i3 = 0;

   if (s16_Flash_Fault_Flag == 1) return  FLASH_INVALID;
   if ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) return (NOT_SUPPORTED_ON_HW);  // axis 1 is not supported on Spansion

   switch (load_state)
   {
      case 0:     // Check validity and initialization state
         u16_Flash_Params_Checksum = 0;      //Initialize checksum

         if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
         {
            u32_Param_Sector_Addr = (long)(FLASH_TYPE_1_PARAM_ADDR);
         }
         else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            u32_Param_Sector_Addr = (long)(AX0_PARAM_BLOCK) * FLASH_BLOCK_SIZE;
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            u32_Param_Sector_Addr = (long)(AX0_PARAM_SECTOR + (6 * drive)) * FLASH_SECTOR_SIZE;
         }

         p_u16_Flash_Params =  &Flash[u32_Param_Sector_Addr];

         // Check if Flash contain data
         if (p_u16_Flash_Params[FLASH_PARAM_STAMP_ADDRESS] != FLASH_PARAM_STAMP_VALUE) return NVRAM_EMPTY;

         u16_Flash_Index = FLASH_PARAM_PARAMS_ADDRESS;

         p_Param_Ram_Table = (struct Variable_Address *)Nvram_Table_Axis_0;
         load_state++;  // increment state

         i = 0;  // initiliaze table index
      break;

      case 1:  // Check in advance if the data on the flash matches the structure of "AXIS0_ADDRESS_TABLE".
               // Do this by a dummy-read of the parameter-data from the flash. The variable
               // "u16_Flash_Params_Checksum" will be generated by the values from the flash.
               // Avoid loading flash-data in case that the checksum is wrong.
         u16_Flash_Params_Checksum = 0;                  // Initialize checksum
         i = 0;                                          // Initialize table-index
         i2 = 0;                                         // Initialize data-size index
         u16_Flash_Index = FLASH_PARAM_PARAMS_ADDRESS;   // Initialize index where the parameter payload starts
         while (i < NUMBER_OF_NVRAM_VARIABLES)
         {
            // Generate checksum from data out of the flash
            i2 = p_Param_Ram_Table[i].size;
            while (i2 > 0)
            {
               u16_Flash_Params_Checksum += p_u16_Flash_Params[u16_Flash_Index + i2 - 1];
               i2--;
            }

            //calculate Nvram_Table_Axis_0 index , for ax0 - even idexes , ax1 - odd.
            u16_Flash_Index +=  p_Param_Ram_Table[i].size;
            //increment i to next parameter to be loaded
            i++;
         }

         // Check if the checksum is correct
         if (u16_Flash_Params_Checksum != p_u16_Flash_Params[FLASH_PARAMS_CHECKSUM_ADDRESS])
         {
            // **********************************************************************************************************
            // Error: At this place we know that the data on the flash does not match the structure "AXIS0_ADDRESS_TABLE"
            // **********************************************************************************************************
            load_state = 0;
            if((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
            {// for non LX28/26 set alarm
               BGVAR(s16_Flash_Checksum_Fault_Flag) = 1;
            }
            return CHECKSUM_ERROR;
         }
         else // The checksum is correct
         {
            // Re-initialize the required variables for loading the payload
            u16_Flash_Params_Checksum = 0;                  // Initialize checksum
            i = 0;                                          // Initialize table-index
            u16_Flash_Index = FLASH_PARAM_PARAMS_ADDRESS;   // Initialize index where the parameter payload starts

            BGVAR(s16_Flash_Checksum_Fault_Flag) = 0;
            load_state++;
         }
      break;

      case 2:  // Check in advance if the version string on the flash matches the version string of the fimrware.
               // This situation occurs when downloading a firmware with an updated "p_s8_CDHD_Drive_Version" string to
               // the Drive. In this case raise an error and avoid loading flash data.

         i2 = 0;  // Offset of location in the flash where the firmware expects the version string
         i = 0;   // Help index

         // Find the place of "BGVAR(u8_SwVersion)" in the RAM mapping and calculate
         // the expected location of the corresponding data in the flash.
         while (i < NUMBER_OF_NVRAM_VARIABLES)
         {
            if(p_Param_Ram_Table[i].ram_ptr == (int *)&BGVAR(u8_SwVersion)[0])
            {
               break; // Leave the while loop
            }
            else
            {
               i2 += p_Param_Ram_Table[i].size; // Calculate the location in the flash
            }
            //increment i to next parameter to be loaded
            i++;
         }
         // If we found the place where we expect the version string in the flash
         if(i < NUMBER_OF_NVRAM_VARIABLES)
         {
            // Read the version string
            for(i=0; i<20; i++)
            {
               BGVAR(u8_SwVersion)[i] = p_u16_Flash_Params[u16_Flash_Index + i2 + i];
            }

            // ****************************************************************************
            // 1st part of Bugzilla 5404 feature added.
            // Generate an array that holds the firmware version as binary values.
            // The 2nd part of the Bugzilla item is done at the end of the "Initialization"
            // function where the error "FIRMWARE_VERSION_NOT_SUPPORTED" is raised.
            // ****************************************************************************
            // First clear array
            for(i2=0; i2<FIRMWARE_VERSION_DIGITS; i2++)
            {
               BGVAR(u8_SwVersion_Binary)[i2] = 0;
            }

            i2=0; // Index for the binary array "BGVAR(u8_SwVersion_Binary)"
            i3=0; // Index for the ASCII array "BGVAR(u8_SwVersion)"
            // Now generate an array that holds the firmware version as binary numbers
            while(i2 < FIRMWARE_VERSION_DIGITS)
            {
               // Go to next valid character '0'...'9'
               if((BGVAR(u8_SwVersion)[i3] < '0') || (BGVAR(u8_SwVersion)[i3] > '9'))
               {
                  i3++; // Increment by 1, either the string is seperated by '.' or by a single letter (e.g. 'a' or 'f')
               }
               // Generate binary number out of the ASCII characters
               while((BGVAR(u8_SwVersion)[i3] >= '0') && (BGVAR(u8_SwVersion)[i3] <= '9'))
               {
                  BGVAR(u8_SwVersion_Binary)[i2] = BGVAR(u8_SwVersion_Binary)[i2] * 10;
                  BGVAR(u8_SwVersion_Binary)[i2] += (BGVAR(u8_SwVersion)[i3] - '0');
                  i3++;
                  // Leave while loop if we exceed the "BGVAR(u8_SwVersion)" array,
                  // which is 20 entries long.
                  if(i3 >= 20)
                  {
                     i2 = FIRMWARE_VERSION_DIGITS;
                     break; // Leave while loop
                  }
               }

               i2++;
            }


            // **********************************************************************************************
            // Now check if the version string of the firmware matches the version string stored in the flash
            // **********************************************************************************************
            if(strcmp(BGVAR(u8_SwVersion),p_s8_CDHD_Drive_Version) != 0)
            {
               // **************************************************************************************************
               // Error: At this place we know that the version strings in the firmware and on the flash don't match.
               // ***************************************************************************************************
               load_state = 0;         // Reset state machine
               if((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
               {// for non LX28/26 set alarm
                  BGVAR(s16_Flash_Checksum_Fault_Flag) = 1;
               }
               return CHECKSUM_ERROR;  // Return a checksum error indication, do not load flash data.
            }
         }

         i = 0;         // Initialize table-index for loading the payload
         load_state++;
      break;

      case 3:    // Read parameters from FLASH
         if (segment == LOAD_CAN_COMM_PARAMS)
         {
            // search for start of CANopen communication segment in nvram params
            while ((i < NUMBER_OF_NVRAM_VARIABLES) && (p_Param_Ram_Table[i].ram_ptr != (int*)CAN_COMM_NV_START_ADDR))
            {
               u16_Flash_Index +=  p_Param_Ram_Table[i].size;
               i++;
            }
         }

         while (i < NUMBER_OF_NVRAM_VARIABLES)
         {
            //copy values from flash to RAM
            Memcpy(p_Param_Ram_Table[i].ram_ptr, (int*)&p_u16_Flash_Params[u16_Flash_Index], p_Param_Ram_Table[i].size);
            //calculate Nvram_Table_Axis_0 index , for ax0 - even idexes , ax1 - odd.
            u16_Flash_Index +=  p_Param_Ram_Table[i].size;

            //increment i to next parameter to be loaded
            i++;


            if ((segment == LOAD_CAN_COMM_PARAMS) && (p_Param_Ram_Table[i].ram_ptr == (int*)CAN_COMM_NV_END_ADDR))
            {
               // if loading only CAN comm segment, leave loop after last param is loaded.
               break;
            }
         }

         // move to next state
         if (segment == LOAD_CAN_COMM_PARAMS)
            load_state = 0;   // CANopen communication segment, skip checksum and finish state machine
         else
            load_state++;
      break;

      case 4:  //Check checksum state of data loaded from the Drive. The variable "u16_Flash_Params_Checksum"
               // is generated within "Memcpy" by the values from the RAM.
         if (u16_Flash_Params_Checksum != p_u16_Flash_Params[FLASH_PARAMS_CHECKSUM_ADDRESS])
         {
            load_state = 0;
            if((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
            {// for non LX28/26 set alarm
               BGVAR(s16_Flash_Checksum_Fault_Flag) = 1;
            }
            return CHECKSUM_ERROR;
         }
         else
         {
            BGVAR(s16_Flash_Checksum_Fault_Flag) = 0;
            load_state++;
         }
      break;

      case 5:
         if (InitSinparam(drive) != SAL_NOT_FINISHED) load_state++;
      break;

      case 6:     //Final state
         // Check if MTP read need to be initiated
         if (BGVAR(u16_MTP_Mode) && (FEEDBACK_SERVOSENSE || FEEDBACK_STEGMANN))
         {
            BGVAR(u16_Init_From_MTP) = 1;
            BGVAR(s16_DisableInputs) &= ~MTP_READ_DIS_MASK;
         }
         load_state = 0;
      break;
   }

   if (load_state == 0) return (SAL_SUCCESS);
   else return (SAL_NOT_FINISHED);
}


//**********************************************************
// Function Name: SalLoadFromFlashCommand
// Description:
//        This function is called in response to the LOAD command.
//      to load parameters from the FLASH
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int SalLoadFromFlashCommand(int drive)
{
   int returned_value;

   if (Enabled(drive)) return (DRIVE_ACTIVE);

   returned_value = LoadFromFlash(drive);
   if (returned_value == SAL_SUCCESS)
   {
      InitAll(drive);
   }

   return (returned_value);
}


//**********************************************************
// Function Name: SalRestoreFactorySettingsCommand
// Description:
//        This function is called in response to the CLREEPROM command.
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int SalRestoreFactorySettingsCommand(int drive)
{
   static int clreeprom_state = 0;
   int ret_val = SAL_NOT_FINISHED;

   if (Enabled(drive)) return (DRIVE_ACTIVE);
   if ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) return (NOT_SUPPORTED_ON_HW);  // axis 1 is not supported on Spansion
   if (1 == s16_Number_Of_Parameters)
   { 
      if (0x1234 != s64_Execution_Parameter[0])
      {
         return VALUE_OUT_OF_RANGE;
      }
      else   
         BGVAR(u16_Factory_Restore_Erase_Param_Only) = 0;
   }      
   switch (clreeprom_state)
   {
      case 0:
         if (Enabled(drive)) return (DRIVE_ACTIVE);
         if (s16_Flash_Fault_Flag == 1) return FLASH_INVALID;
         s16_Save_Sector_Counter = 0;
         clreeprom_state++;
         BGVAR(u16_Is_Factory_Restore_Running) = 1;
      break;

      case 1://Execute erase all instruction:
         if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
         {
            if (EraseBlock(AX0_FLASH_TYPE_1_PARAM_BLOCK) != FLASH_PENDING) clreeprom_state++;
         }
         else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            if (EraseBlock(AX0_PARAM_BLOCK) != FLASH_PENDING) clreeprom_state++;
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            if (EraseSector(AX0_PARAM_SECTOR + (6 * drive) + s16_Save_Sector_Counter) != FLASH_PENDING) s16_Save_Sector_Counter++;
            if (s16_Save_Sector_Counter == 5) clreeprom_state++;
         }
      break;

      case 2:// erase  SINPARAM:
         if (BGVAR(u16_Factory_Restore_Erase_Param_Only) == 1)
         {
            // If only erase param is required, skip this state
            clreeprom_state++;
         }
         else
         {
            if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
            {
               if (EraseBlock(AX0_FLASH_TYPE_1_SINPARAM_BLOCK) != FLASH_PENDING)
               {
                  clreeprom_state++;
                  BGVAR(u16_Flash_Sinparam_Index) = 0;
               }
            }
            else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
            { // Spansion uses Block (32KWord) Arrangement
               if (EraseBlock(AX0_SINPARAM_BLOCK) != FLASH_PENDING)
               {
                  clreeprom_state++;
                  BGVAR(u16_Flash_Sinparam_Index) = 0;
               }
            }
            else
            { // SST uses Sector (2KWord) Arrangement
               if (EraseSector(AX0_SINPARAM_SECTOR + drive) != FLASH_PENDING)
               {
                  clreeprom_state++;
                  BGVAR(u16_Flash_Sinparam_Index) = 0;
               }
            }
         }
      break;

      case 3://clr fault log
         if (BGVAR(u16_Factory_Restore_Erase_Param_Only) == 1)
         {
            // If only erase param is required, skip this state
            clreeprom_state++;
         }
         else
         {
            if (ClearFlashFaultLog(drive) != SAL_NOT_FINISHED) clreeprom_state++;
         }
      break;

      case 4:
         STORE_EXECUTION_PARAM_0
         ResetAllParam(drive);
         RESTORE_EXECUTION_PARAM_0
         clreeprom_state++;
      break;

      case 5:
         InitAll(drive);
         clreeprom_state++;
      break;

      case 6:
            BGVAR(s64_SysNotOk) &= ~NVRAM_CHECKSUM_FAULT;  // Clear the EEPROM checksum fault indication
            clreeprom_state++;
            ret_val = SAL_NOT_FINISHED;
      break;

      case 7: // Initialize at the end the post factory restore actions
         // After a factory restore additional task may need to be handled
         BGVAR(u16_Post_Factory_Restore_Actions) |= POST_FACT_REST_SET_OPMODE_UPON_BUNDLE;
         clreeprom_state = 0;
         ret_val = SAL_SUCCESS;
      break;
   }

   if (ret_val != SAL_NOT_FINISHED)
   {
      BGVAR(u16_Factory_Restore_Erase_Param_Only) = 1;
      BGVAR(u16_Is_Factory_Restore_Running) = 0;
   }

   return (ret_val);
}


//**********************************************************
// Function Name: LoadFromFlash
// Description:
//        This function is called on initialization to load parameters from flash
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int LoadFromFlash(int drive)
{
   int return_value;

   if (s16_Flash_Fault_Flag == 1) return  FLASH_INVALID;

   return_value = SAL_NOT_FINISHED;

   while (return_value == SAL_NOT_FINISHED)
   {
      return_value = LoadFromFlashStateMachine(drive);
   }

   if (return_value != SAL_SUCCESS)
   {// flash load failed !
      if((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
      {// for non LX28/26 set alarm
         BGVAR(s16_Flash_Checksum_Fault_Flag) = 1;
      }
      else
      {// for LX28/26 set Wn737 (drive is using default configuration)
         BGVAR(s16_Drive_Default_Config_Wrn_Flag) = 1;
      }
   }

   return return_value;
}


void Memcpy(int* ram_ptr, int* flash_ptr, unsigned int len)
{
   int i;

   for (i = 0; i < len; i++)
   {
      ram_ptr[i] = flash_ptr[i];
      u16_Flash_Params_Checksum += ram_ptr[i]; //update checksum
   }
}


//**********************************************************
// Function Name: InitSinparam
// Description:
//        This function is called  to load SINPARAMS  from flash
//
//
// Author: D.R. / APH
// Algorithm:
// Revisions:
//**********************************************************
int InitSinparam(int drive)
{
   // AXIS_OFF;
   static int state = 0;
   unsigned int i;
   long addr;
   int checksum = 0;
   int s16_new_format = 0;          // Variable that indicates with the new data-structure in the flash
   int s16_new_format_offset = 0;   // Variable that helps with reading the data correctly due to the 0x1234 stamp information at the beginning
                                    // of the data, which is not suppoed to be considered
   unsigned int u16_sin_param_data_size = FLASH_SINPARAMS_DATA_SIZE;

   if ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) return (NOT_SUPPORTED_ON_HW);  // axis 1 is not supported on Spansion

   switch (state)
   {
      case 0:
         BGVAR(u16_Flash_Sinparam_Index) = 0;

         if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
         {
            addr = (long)(FLASH_TYPE_1_SINPARAM_ADDR);
         }
         else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            addr = (long)(AX0_SINPARAM_BLOCK) * FLASH_BLOCK_SIZE;
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            addr = (long)(AX0_SINPARAM_SECTOR + drive) * FLASH_SECTOR_SIZE;
         }
         p_u16_Flash_Sinparam =  &Flash[addr];

         if ( (u16_Vendor_ID[0] != 0x0001) || (u16_Flash_Type == 1) ) // If not 16Mb or 32 Mb Spansion ID
         {
            // Check if the last field in the sector holds the data 0xABCD, which is an indication for the "new format".
            if (p_u16_Flash_Sinparam[FLASH_SECTOR_SIZE - 1] == 0xABCD)
            {
               s16_new_format = 1; // Indicate new format which includes further data
               s16_new_format_offset = 1;
               u16_sin_param_data_size += VOLTAGE_CORRECT_FLASH_SIZE + 1; // + 1 due to special stamp value unequal 0 at the beginning of the data since the checksum can be 0
            }
            // Check if the last field in the sector holds the data 0xABCE, which is an indication for the "new format" including 2 sets of voltage correction data.
            else if (p_u16_Flash_Sinparam[FLASH_SECTOR_SIZE - 1] == 0xABCE)
            {
               s16_new_format = 2; // Indicate new format which includes further data
               s16_new_format_offset = 1;
               u16_sin_param_data_size += 2*(VOLTAGE_CORRECT_FLASH_SIZE) + 1; // + 1 due to special stamp value unequal 0 at the beginning of the data since the checksum can be 0
            }
         }

         if ( (u16_Vendor_ID[0] == 0x0001) && (u16_Flash_Type != 1) )  // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            while ((p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index)] == 0) && (BGVAR(u16_Flash_Sinparam_Index) < FLASH_BLOCK_SIZE))
               BGVAR(u16_Flash_Sinparam_Index) += u16_sin_param_data_size;
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            while ((p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index)] == 0) && (BGVAR(u16_Flash_Sinparam_Index) < FLASH_SECTOR_SIZE))
               BGVAR(u16_Flash_Sinparam_Index) += u16_sin_param_data_size;
         }

         if (BGVAR(u16_Flash_Sinparam_Index) == 0) // check if sector empty -> When saving data (also for the first time), zeros are written
            return SAL_SUCCESS;                    // to the flash in order to delete existing data (see "SaveSineZeroParams").

         //check if valid sinparam found. Reduce for-loop iteration by "s16_new_format_offset" since the stamp information 0x1234
         // is also not part of the checksum and therefore reduce the for-loop iteration by 1 if the new format is applied.
         for (i = 1; i < (u16_sin_param_data_size - s16_new_format_offset); i++)
            checksum += p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + i + s16_new_format_offset];

         if (checksum == p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + s16_new_format_offset]) //sinparams are valid
         {
            VAR(AX0_s16_Sine_Offset)   = p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + 1 + s16_new_format_offset];
            VAR(AX0_s16_Cosine_Offset) = p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + 2 + s16_new_format_offset];
            VAR(AX0_s16_Sine_Gain_Fix) = p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + 3 + s16_new_format_offset];
            VAR(AX0_s16_Sine_Gain_Shr) = p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + 4 + s16_new_format_offset];
            VAR(AX0_s16_Swr2d_Fix)     = p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + 5 + s16_new_format_offset];
            VAR(AX0_s16_Swr2d_Shr)     = p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + 6 + s16_new_format_offset];

            // Here read the additional data
            if(s16_new_format > 0)
            {
               // Read first the number of section value
               BGVAR(u16_Voltage_Correct_Number_Of_Sections) = p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + 7 + s16_new_format_offset];
               // Now read the voltage array
               for(i=0; i<VOLTAGE_CORRECT_ARRAY_SIZE; i++)
               {
                   BGVAR(s16_Voltage_Correct_Array)[i] = p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + 8 + s16_new_format_offset + i];
               }

               if(s16_new_format > 1)
               {
                  // Now read first the number of section of analog input 2 value
                  BGVAR(u16_Voltage_Correct_2_Number_Of_Sections) = p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + 7 + s16_new_format_offset + VOLTAGE_CORRECT_ARRAY_SIZE + 1];
                  // Now read the voltage array of analog input 2
                  for(i=0; i<VOLTAGE_CORRECT_ARRAY_SIZE; i++)
                  {
                      BGVAR(s16_Voltage_Correct_2_Array)[i] = p_u16_Flash_Sinparam[BGVAR(u16_Flash_Sinparam_Index) + 8 + s16_new_format_offset + VOLTAGE_CORRECT_ARRAY_SIZE + 1 + i];
                  }
               }
            }
            return SAL_SUCCESS;
         }
         //checksum invalid, init sine gain to unity
         VAR(AX0_s16_Sine_Offset) = 0;
         VAR(AX0_s16_Cosine_Offset) = 0;
         VAR(AX0_s16_Sine_Gain_Fix) = 0x4000;
         VAR(AX0_s16_Sine_Gain_Shr) = 14;
         VAR(AX0_s16_Swr2d_Fix) = 0x4000;
         VAR(AX0_s16_Swr2d_Shr) = 14;

         state++;       // Sector (or Block) mapping invalid, erase the sector
      break;

      case 1:
         if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
         {
            if (EraseBlock(AX0_FLASH_TYPE_1_SINPARAM_BLOCK) == FLASH_DONE) state = 0;
         }
         else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            if (EraseBlock(AX0_SINPARAM_BLOCK) == FLASH_DONE) state = 0;
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            if (EraseSector(AX0_SINPARAM_SECTOR + drive) == FLASH_DONE) state = 0;
         }
         BGVAR(u16_Flash_Sinparam_Index) = 0;
      break;
   }

   if (state == 0) return SAL_SUCCESS;
   else return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: WriteSinparams2Flash
// Description:
//   This function writes sinparams to flash
//
//   Old structure
//   -------------
//   |-Checksum-|-SinParamData 1-|...|-SinParamData 6-|
//
//
//
//   New structure
//   -------------
//   |-0x1234-|-Checksum-|-SinParamData 1-|...|-SinParamData 6-|-NumberOfSectors-|-VoltageValue 0-|...|-VoltageValue 10-|-NumberOfSectors2-|-Voltage2Value 0-|...|-Voltage2Value 10-|.............|-0xABCE in the very last memory location of the flash sector-|
//
// Author: D.R. / APH
// Algorithm:
// Revisions:
//**********************************************************
void WriteSinparams2Flash(int drive)
{
   // AXIS_OFF;
   int checksum;
   long sector_block_start_addr;
   int s16_new_format_offset = 0; // Variable that indicates with a 1 the new data-structre in the flash
   unsigned int i;

   if ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) return;  // axis 1 is not supported on Spansion

   if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
   {
      sector_block_start_addr = (long)(FLASH_TYPE_1_SINPARAM_ADDR);
   }
   else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
   { // Spansion uses Block (32KWord) Arrangement
      sector_block_start_addr = (long)(AX0_SINPARAM_BLOCK) * FLASH_BLOCK_SIZE;
   }
   else
   { // SST uses Sector (2KWord) Arrangement
      sector_block_start_addr = (long)(AX0_SINPARAM_SECTOR + drive) * FLASH_SECTOR_SIZE;
   }

   if ( (u16_Vendor_ID[0] != 0x0001) || (u16_Flash_Type == 1) ) // If not Spansion ID
   {
      s16_new_format_offset = 1; // Indicate new format which includes further data.
   }

   EnableFlashWrite(1); // enable write access to FLASH
   FlashWriteWord(VAR(AX0_s16_Sine_Offset), sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 1 + s16_new_format_offset);
   FlashWriteWord(VAR(AX0_s16_Cosine_Offset), sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 2 + s16_new_format_offset);
   FlashWriteWord(VAR(AX0_s16_Sine_Gain_Fix), sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 3 + s16_new_format_offset);
   FlashWriteWord(VAR(AX0_s16_Sine_Gain_Shr), sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 4 + s16_new_format_offset);
   FlashWriteWord(VAR(AX0_s16_Swr2d_Fix), sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 5 + s16_new_format_offset);
   FlashWriteWord(VAR(AX0_s16_Swr2d_Shr), sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 6 + s16_new_format_offset);
   checksum = VAR(AX0_s16_Sine_Offset) + VAR(AX0_s16_Cosine_Offset) + VAR(AX0_s16_Sine_Gain_Fix) + VAR(AX0_s16_Sine_Gain_Shr) + VAR(AX0_s16_Swr2d_Fix) + VAR(AX0_s16_Swr2d_Shr);

   if(s16_new_format_offset == 1)
   {
      // Write as first entry a value unequal 0 for the search algorithm in InitSinparam
      FlashWriteWord(0x1234, sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 0);

      // Write the number of section value
      FlashWriteWord(BGVAR(u16_Voltage_Correct_Number_Of_Sections), sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 7 + s16_new_format_offset);
      checksum += BGVAR(u16_Voltage_Correct_Number_Of_Sections);

      // Now write the voltage array
      for(i=0; i<VOLTAGE_CORRECT_ARRAY_SIZE; i++)
      {
          FlashWriteWord(BGVAR(s16_Voltage_Correct_Array)[i], sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 8 + s16_new_format_offset + i);
          checksum += BGVAR(s16_Voltage_Correct_Array)[i];
      }

      // Write the number of section value for analog input 2
      FlashWriteWord(BGVAR(u16_Voltage_Correct_2_Number_Of_Sections), sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 7 + s16_new_format_offset + VOLTAGE_CORRECT_ARRAY_SIZE + 1);
      checksum += BGVAR(u16_Voltage_Correct_2_Number_Of_Sections);

      // Now write the voltage array
      for(i=0; i<VOLTAGE_CORRECT_ARRAY_SIZE; i++)
      {
          FlashWriteWord(BGVAR(s16_Voltage_Correct_2_Array)[i], sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 8 + s16_new_format_offset + VOLTAGE_CORRECT_ARRAY_SIZE + 1 + i);
          checksum += BGVAR(s16_Voltage_Correct_2_Array)[i];
      }

      // Write in the very last entry the value 0xABCE in order to highlight the new format including the voltage correction data
      FlashWriteWord(0xABCE, sector_block_start_addr, FLASH_SECTOR_SIZE - 1);
   }

   FlashWriteWord(checksum, sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + 0 + s16_new_format_offset);

   EnableFlashWrite(0); // disable write access to FLASH
}


//**********************************************************
// Function Name: SaveSineZeroParams
// Description:
//        This function is called  to save SINPARAMS and the voltage correction
//        data for dual-loop correction to the flash.
//
//
// Author: D.R. / APH
// Algorithm:
// Revisions:
//**********************************************************
int SaveSineZeroParams(int drive)
{
   static int state = 0;
   unsigned int i;
   long sector_block_start_addr;
   unsigned int u16_sin_param_data_size = FLASH_SINPARAMS_DATA_SIZE;
   int s16_return_value;

   if ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) return (NOT_SUPPORTED_ON_HW);  // axis 1 is not supported on Spansion

   if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
   {
      sector_block_start_addr = (long)(FLASH_TYPE_1_SINPARAM_ADDR);
   }
   else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
   { // Spansion uses Block (32KWord) Arrangement
      sector_block_start_addr = (long)(AX0_SINPARAM_BLOCK) * FLASH_BLOCK_SIZE;
   }
   else
   { // SST uses Sector (2KWord) Arrangement
      sector_block_start_addr = (long)(AX0_SINPARAM_SECTOR + drive) * FLASH_SECTOR_SIZE;
   }

   switch (state)
   {
      case 0: // check if there is free space in the sector
         if ( (u16_Vendor_ID[0] == 0x0001) && (u16_Flash_Type != 1) ) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            if (BGVAR(u16_Flash_Sinparam_Index) < FLASH_BLOCK_SIZE - (2 * u16_sin_param_data_size)) // * 2 because we want to delete the current values and append new ones.
            {
               for (i = 0; i < u16_sin_param_data_size; i++) // first overide existing data by 0. This is needed in order to find the latest data in the flash sector when reading the data.
                  FlashWriteWord(0, sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + i);

               //increment index to next free space
               BGVAR(u16_Flash_Sinparam_Index) += u16_sin_param_data_size;
               WriteSinparams2Flash(drive);

               if (s16_Flash_Fault_Flag == 1)                     // 1 indicates that FlashWriteWord() failed (FLASH_FAIL)
               {
                  s16_Sin_Zero_Flash_Fault_Acc_Counter++;         // count the accumulated number of Sin Zero failures
                  s16_Sin_Zero_Flash_Fault_Counter++;             // count the number of Sin Zero failures for retry
                  if (s16_Sin_Zero_Flash_Fault_Counter < 5)       // retry
                  {
                     s16_Flash_Fault_Flag = 0;                    // remove the failure indication
                     return (SAL_NOT_FINISHED);                   // return as not finished for retry
                  }
                  else
                  {
                     return SAVE_FAILED;
                  }
               }

               s16_Sin_Zero_Flash_Fault_Counter = 0;
               return SAL_SUCCESS;
            }
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            u16_sin_param_data_size += 2*(VOLTAGE_CORRECT_FLASH_SIZE) + 1; // + 1 due to special stamp value unequal 0 at the beginning of the data since the checksum can be 0
            p_u16_Flash_Sinparam =  &Flash[sector_block_start_addr];                                     // Let pointer point to the beginning of the SINPARAM sector

            if((p_u16_Flash_Sinparam[FLASH_SECTOR_SIZE - 1] == 0xABCE)  &&                               // If there is data stored in the new format
               (BGVAR(u16_Flash_Sinparam_Index) < FLASH_SECTOR_SIZE - (2 * u16_sin_param_data_size) - 1) // * 2 because we want to delete the current values and append new ones. Additional -1 because
              )                                                                                          // the very last entry in the memory section indicates the new format.
            {
               for (i = 0; i < u16_sin_param_data_size; i++) // first overide existing data by 0. This is needed in order to find the latest data in the flash sector when reading the data.
                  FlashWriteWord(0, sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + i);

               //increment index to next free space
               BGVAR(u16_Flash_Sinparam_Index) += u16_sin_param_data_size;
               WriteSinparams2Flash(drive);

               if (s16_Flash_Fault_Flag == 1)                     // 1 indicates that FlashWriteWord() failed (FLASH_FAIL)
               {
                  s16_Sin_Zero_Flash_Fault_Acc_Counter++;         // count the accumulated number of Sin Zero failures
                  s16_Sin_Zero_Flash_Fault_Counter++;             // count the number of Sin Zero failures for retry
                  if (s16_Sin_Zero_Flash_Fault_Counter < 5)       // retry
                  {
                     s16_Flash_Fault_Flag = 0;                    // remove the failure indication
                     return (SAL_NOT_FINISHED);                   // return as not finished for retry
                  }
                  else
                  {
                     return SAVE_FAILED;
                  }
               }

               s16_Sin_Zero_Flash_Fault_Counter = 0;
               return SAL_SUCCESS;
            }
         }
         state++; //erase the sector

      case 1: // We reach this state in case that there is not enough memory left in the sector
         if ( (u16_Vendor_ID[0] == 0x0001) && (u16_Flash_Type != 1) ) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            if (EraseBlock(AX0_SINPARAM_BLOCK) == FLASH_DONE)
            {
               state = 0;
               BGVAR(u16_Flash_Sinparam_Index) = 0;
               WriteSinparams2Flash(drive);

               if (s16_Flash_Fault_Flag == 1)                     // 1 indicates that FlashWriteWord() failed (FLASH_FAIL)
               {
                  s16_Sin_Zero_Flash_Fault_Acc_Counter++;         // count the accumulated number of Sin Zero failures
                  s16_Sin_Zero_Flash_Fault_Counter++;             // count the number of Sin Zero failures for retry
                  if (s16_Sin_Zero_Flash_Fault_Counter < 5)       // retry
                  {
                     s16_Flash_Fault_Flag = 0;                    // remove the failure indication
                     state = 1;
                     return (SAL_NOT_FINISHED);                   // return as not finished for retry
                  }
                  else
                  {
                     return SAVE_FAILED;
                  }
               }

            }
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            if (u16_Flash_Type == 1)
            {
               s16_return_value = EraseBlock(AX0_FLASH_TYPE_1_SINPARAM_BLOCK);
            }
            else
            {
               s16_return_value = EraseSector(AX0_SINPARAM_SECTOR + drive);
            }

            if (s16_return_value == FLASH_DONE)
            {
               state = 0;
               BGVAR(u16_Flash_Sinparam_Index) = 0;

               // First write for 1 time zeros at the beginning of the sector, which indiactes used memory
               u16_sin_param_data_size += 2*(VOLTAGE_CORRECT_FLASH_SIZE) + 1; // + 1 due to special stamp value unequal 0 at the beginning of the data since the checksum can be 0
               for (i = 0; i < u16_sin_param_data_size; i++) // first overide existing data by 0. This is needed in order to find the latest data in the flash sector when reading the data.
                  FlashWriteWord(0, sector_block_start_addr, BGVAR(u16_Flash_Sinparam_Index) + i);
               //increment index to next free space
               BGVAR(u16_Flash_Sinparam_Index) += u16_sin_param_data_size;
               WriteSinparams2Flash(drive);

               if (s16_Flash_Fault_Flag == 1)                     // 1 indicates that FlashWriteWord() failed (FLASH_FAIL)
               {
                  s16_Sin_Zero_Flash_Fault_Acc_Counter++;         // count the accumulated number of Sin Zero failures
                  s16_Sin_Zero_Flash_Fault_Counter++;             // count the number of Sin Zero failures for retry
                  if (s16_Sin_Zero_Flash_Fault_Counter < 5)       // retry
                  {
                     s16_Flash_Fault_Flag = 0;                    // remove the failure indication
                     state = 1;
                     return (SAL_NOT_FINISHED);                   // return as not finished for retry
                  }
                  else
                  {
                     return SAVE_FAILED;
                  }
               }

            }
         }
      break;
   }

   if (state == 0)
   {
      s16_Sin_Zero_Flash_Fault_Counter = 0;
      return SAL_SUCCESS;
   }
   else return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: WriteFaultLog2Flash
// Description:
//   This function writes 1 fault from ram to flash
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void WriteFaultLog2Flash(int drive, int fault_ptr)
{
   unsigned int temp, addr = BGVAR(u16_Flash_Faults_Index);
   long sector_block_start_addr;

   if ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) return;  // axis 1 is not supported on Spansion

   if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
   {
      sector_block_start_addr = (long)(FLASH_TYPE_1_FAULT_LOG_ADDR);
   }
   else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
   { // Spansion uses Block (32KWord) Arrangement
      sector_block_start_addr = (long)(AX0_FAULT_LOG_BLOCK) * FLASH_BLOCK_SIZE;
   }
   else
   { // SST uses Sector (2KWord) Arrangement
      sector_block_start_addr = (long)(AX0_FAULT_LOG_SECTOR + drive) * FLASH_SECTOR_SIZE;
   }

   temp = (BGVAR(s_Fault_Log_Image)[fault_ptr].minutes << 8) | BGVAR(s_Fault_Log_Image)[fault_ptr].seconds;

   FlashWriteWord(FLASH_FAULT_STAMP, sector_block_start_addr, addr + FLASH_FAULT_STAMP_ADDR);
   FlashWriteWord(temp, sector_block_start_addr, addr + FLASH_FAULT_SEC_ADDR);

   temp = BGVAR(s_Fault_Log_Image)[fault_ptr].fault_id | (BGVAR(s_Fault_Log_Image)[fault_ptr].first_fault_after_power_up << 15);
   FlashWriteWord(temp, sector_block_start_addr, addr + FLASH_FAULT_ID_ADDR);
   FlashWriteWord(BGVAR(s_Fault_Log_Image)[fault_ptr].hours, sector_block_start_addr, addr + FLASH_FAULT_HOURS_ADDR);

   BGVAR(u16_Flash_Faults_Index) += FLASH_FAULT_DATA_SIZE;
   BGVAR(u16_Flash_Fault) = fault_ptr;
}


//**********************************************************
// Function Name: FlashWriteFault
// Description:
//   This function writes fault to flash
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int FlashWriteFault(int drive, int fault_ptr)
{
   static int state = 0;
   int flt, len;

   if ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) return (NOT_SUPPORTED_ON_HW);  // axis 1 is not supported on Spansion

   if ( (u16_Vendor_ID[0] == 0x0001) && (u16_Flash_Type != 1) ) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
   { // Spansion uses Block (32KWord) Arrangement
      len = FLASH_BLOCK_SIZE - FLASH_FAULT_DATA_SIZE;
   }
   else
   { // SST uses Sector (2KWord) Arrangement
      len = FLASH_SECTOR_SIZE - FLASH_FAULT_DATA_SIZE;
   }

   switch (state)
   {
      case 0:
         if ( BGVAR(u16_Flash_Faults_Index) < len )
         {
            WriteFaultLog2Flash(drive, fault_ptr);

            if (s16_Flash_Fault_Flag == 1)                     // 1 indicates that FlashWriteWord() failed (FLASH_FAIL)
            {
               s16_Fault_Log_Flash_Fault_Acc_Counter++;         // count the accumulated number of Fault Log failures
               s16_Fault_Log_Flash_Fault_Counter++;             // count the number of Fault Log failures for retry
               if (s16_Fault_Log_Flash_Fault_Counter < 5)       // retry
               {
                  s16_Flash_Fault_Flag = 0;                    // remove the failure indication
                  state = 1;
                  return (SAL_NOT_FINISHED);                   // return as not finished for retry
               }
               else
               {
                  return SAVE_FAILED;
               }
            }

            s16_Fault_Log_Flash_Fault_Counter = 0;
            return SAL_SUCCESS;
         }
         state++;
      break;

      case 1: // no more space on sector , erase flash and dump all fault log from RAM to FLASH
         if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
         {
            if (EraseBlock(AX0_FLASH_TYPE_1_FAULT_LOG_BLOCK) == FLASH_DONE) state++;
         }
         else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            if (EraseBlock(AX0_FAULT_LOG_BLOCK) == FLASH_DONE) state++;
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            if (EraseSector(AX0_FAULT_LOG_SECTOR + drive) == FLASH_DONE) state++;
         }
      break;

      case 2:
         BGVAR(u16_Flash_Faults_Index) = 0;
         flt = BGVAR(u16_Fault_Log_Ptr); // point to last fault
         do {
            flt++;
            if (flt >= FAULT_LOG_LEN) flt = 0;
            if (BGVAR(s_Fault_Log_Image)[flt].fault_id != 0) WriteFaultLog2Flash(drive, flt);

         } while (flt != BGVAR(u16_Fault_Log_Ptr));

         BGVAR(u16_Flash_Fault) = BGVAR(u16_Fault_Log_Ptr);

         state = 0;

         if (s16_Flash_Fault_Flag == 1)                     // 1 indicates that FlashWriteWord() failed (FLASH_FAIL)
         {
            s16_Fault_Log_Flash_Fault_Acc_Counter++;         // count the accumulated number of Fault Log failures
            s16_Fault_Log_Flash_Fault_Counter++;             // count the number of Fault Log failures for retry
            if (s16_Fault_Log_Flash_Fault_Counter < 5)       // retry
            {
               s16_Flash_Fault_Flag = 0;                    // remove the failure indication
               state = 1;
               return (SAL_NOT_FINISHED);                   // return as not finished for retry
            }
            else
            {
               return SAVE_FAILED;
            }
         }
      break;
   }

   if (state == 0)
   {
      s16_Fault_Log_Flash_Fault_Counter = 0;
      return SAL_SUCCESS;
   }
   else return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: LoadFaultLogFromFlash
// Description:
//   This function loads fault log from FLASH to RAM
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void LoadFaultLogFromFlash(int drive)
{
   int i, len;
   long addr;

   if ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) return;  // axis 1 is not supported on Spansion

   BGVAR(u16_Flash_Faults_Index) = 0;
   if ( (u16_Vendor_ID[0] == 0x0001) && (u16_Flash_Type != 1) ) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
   { // Spansion uses Block (32KWord) Arrangement
      len = FLASH_BLOCK_SIZE / FLASH_FAULT_DATA_SIZE - 1;
   }
   else
   { // SST uses Sector (2KWord) Arrangement
      len = FLASH_SECTOR_SIZE / FLASH_FAULT_DATA_SIZE - 1;
   }

   ClearRamFaultLog(drive);

   if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
   {
      addr = (long)(FLASH_TYPE_1_FAULT_LOG_ADDR);
   }
   else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
   { // Spansion uses Block (32KWord) Arrangement
      addr = (long)(AX0_FAULT_LOG_BLOCK) * FLASH_BLOCK_SIZE;
   }
   else
   { // SST uses Sector (2KWord) Arrangement
      addr = (long)(AX0_FAULT_LOG_SECTOR + drive) * FLASH_SECTOR_SIZE;
   }
   p_u16_Flash_Faults =  &Flash[addr];

   while ((p_u16_Flash_Faults[BGVAR(u16_Flash_Faults_Index)] == FLASH_FAULT_STAMP) && (BGVAR(u16_Flash_Faults_Index) < len))
      BGVAR(u16_Flash_Faults_Index) += FLASH_FAULT_DATA_SIZE;

   BGVAR(u16_Flash_Fault) = (BGVAR(u16_Flash_Faults_Index) / FLASH_FAULT_DATA_SIZE) - 1;
   if (BGVAR(u16_Flash_Fault) >= FAULT_LOG_LEN) BGVAR(u16_Flash_Fault) = FAULT_LOG_LEN - 1;

   //check that FLASH empty after last stamp
   if (p_u16_Flash_Faults[BGVAR(u16_Flash_Faults_Index)] != 0xFFFF)
   {
      if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
      {
         while (EraseBlock(AX0_FLASH_TYPE_1_FAULT_LOG_BLOCK) == FLASH_PENDING)
            asm(" NOP");
      }
      else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
      { // Spansion uses Block (32KWord) Arrangement
         while (EraseBlock(AX0_FAULT_LOG_BLOCK) == FLASH_PENDING)
            asm(" NOP");
      }
      else
      { // SST uses Sector (2KWord) Arrangement
         while (EraseSector(AX0_FAULT_LOG_SECTOR + drive) == FLASH_PENDING)
            asm(" NOP");
      }
      BGVAR(u16_Flash_Faults_Index) = 0;
      BGVAR(u16_Flash_Fault) = 0;
      BGVAR(u16_Fault_Log_Ptr) = 0;
      return;
   }

   addr = (long)BGVAR(u16_Flash_Faults_Index);
   //load faults from flash to RAM
   for (i = BGVAR(u16_Flash_Fault); i >= 0 ; i--)
   {
       addr = addr - FLASH_FAULT_DATA_SIZE;
      if (addr < 0) break;
      BGVAR(s_Fault_Log_Image)[i].seconds =  p_u16_Flash_Faults[addr + FLASH_FAULT_SEC_ADDR] & 0x00FF;
      BGVAR(s_Fault_Log_Image)[i].minutes =  p_u16_Flash_Faults[addr + FLASH_FAULT_MIN_ADDR] >> 8;
      BGVAR(s_Fault_Log_Image)[i].fault_id = p_u16_Flash_Faults[addr + FLASH_FAULT_ID_ADDR] & 0x7FFF; // mask MSB
      BGVAR(s_Fault_Log_Image)[i].first_fault_after_power_up = p_u16_Flash_Faults[addr + FLASH_FAULT_ID_ADDR] >> 15; //shift MSB to LSB
      BGVAR(s_Fault_Log_Image)[i].hours =    p_u16_Flash_Faults[addr + FLASH_FAULT_HOURS_ADDR];
   }

   //init RAM flt pointer
   BGVAR(u16_Fault_Log_Ptr) = BGVAR(u16_Flash_Fault);
}


//**********************************************************
// Function Name: ClearRamFaultLog
// Description:
//   This function clears Fault log at RAM
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void ClearRamFaultLog(int drive)
{
   int i;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //Clear Fault Log at RAM
   for (i = 0; i < FAULT_LOG_LEN; i++)
    {
      BGVAR(s_Fault_Log_Image)[i].seconds = 0;
      BGVAR(s_Fault_Log_Image)[i].minutes = 0;
      BGVAR(s_Fault_Log_Image)[i].hours = 0;
      BGVAR(s_Fault_Log_Image)[i].fault_id = 0xFF;
      BGVAR(s_Fault_Log_Image)[i].first_fault_after_power_up = 0;
    }
   BGVAR(u16_Fault_Log_Ptr) = 0;
   BGVAR(u16_Flash_Fault) = 0;
}


//**********************************************************
// Function Name: ClearFlashFaultLog
// Description:
//   This function erases Fault log at RAM and FLASH
//
// Author:  D.R.
// Algorithm:
// Revisions:
//**********************************************************
int ClearFlashFaultLog(int drive)
{
   static int state = 0;

   if (s16_Flash_Fault_Flag == 1) return  FLASH_INVALID; // check for FLASH validaty
   if ((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) return (NOT_SUPPORTED_ON_HW);  // axis 1 is not supported on Spansion

   switch (state)
   {
      case 0:
         if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
         {
            if (EraseBlock(AX0_FLASH_TYPE_1_FAULT_LOG_BLOCK) != FLASH_PENDING) state++;
         }
         else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
         { // Spansion uses Block (32KWord) Arrangement
            if (EraseBlock(AX0_FAULT_LOG_BLOCK) != FLASH_PENDING) state++;
         }
         else
         { // SST uses Sector (2KWord) Arrangement
            if (EraseSector(AX0_FAULT_LOG_SECTOR + drive) != FLASH_PENDING) state++;
         }
         return SAL_NOT_FINISHED;
      case 1:
         BGVAR(u16_Flash_Faults_Index) = 0;
         BGVAR(u16_Flash_Fault) = 0;
         ClearRamFaultLog(drive);   //Clear Fault Log at RAM
         state = 0;
      break;

   }
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalTriggerNvSaveViaPParam
// Description:
//          This function is called upon a write access of
//          P-parameter P10-08 .
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalTriggerNvSaveViaPParam(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // If the user writes a 1 and the nv save bit is not yet set.
   if ((lparam == 1) && ((BGVAR(u16_Lexium_Bg_Action) & LEX_DO_NV_SAVE) == 0))
   {
      // Set the do nv-save bit
      BGVAR(u16_Lexium_Bg_Action) |= LEX_DO_NV_SAVE;
   }

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadNvSaveStatusStatusViaPParam
// Description:
//          This function is called upon a read access of
//          P-parameter P10-08. A return value of 1 means, that
//          a nv-save process is pending/running. 0 means that
//          the nv-save process is completed.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadNvSaveStatusViaPParam(long long* lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(((BGVAR(u16_Lexium_Bg_Action) & LEX_DO_NV_SAVE) == 0) &&
      ((BGVAR(u16_Lexium_Bg_Action_Applied) & LEX_DO_NV_SAVE) == 0))
   {
      *lparam = 0; // No nv-save process pending
   }
   else
   {
      *lparam = 1; // Nv-save process pending
   }

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalTriggerFaultHistClearViaPParam
// Description:
//          This function is called upon a write access of
//          P-parameter P4-00 and P4-10.
// Rev:     Fix bug now only P4-00 and P4-10 can clear history and P4-01 to P4-04 are read only.
//
// Author: APH
// Algorithm:
// Revisions: Moshe
//**********************************************************
int SalTriggerFaultHistClearViaPParam(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL) return (VALUE_TOO_LOW);
   if (lparam > 0LL) return (VALUE_TOO_HIGH);

   // If the user writes a 0 and the clear fault history bit is not yet set.
   if ((BGVAR(u16_Lexium_Bg_Action) & LEX_CLEAR_FAULT_HISTORY) == 0)
   {
      BGVAR(u16_Lexium_Bg_Action) |= LEX_CLEAR_FAULT_HISTORY;      // Set the clear fault history bit
   }

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadFaultHistClearStatusViaPParam
// Description:
//          This function is called upon a read access of
//          P-parameter P8-69. A return value of 1 means, that
//          a clear fault history process is pending/running.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadFaultHistClearStatusViaPParam(long long* lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((BGVAR(u16_Lexium_Bg_Action) & LEX_CLEAR_FAULT_HISTORY) == 0)
   {
      *lparam = 0; // No clear fault history process pending
   }
   else
   {
      *lparam = 1; // Clear fault history process pending
   }

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalTriggerFactoryRestoreViaPParam
// Description:
//          This function is called upon a write access of
//          P-parameter P10-09.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalTriggerFactoryRestoreViaPParam(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   if (lparam == 1)
    // The following code has been commented since SHNDR wants to apply a factory restore only after power-up
   { //Set flag for "Factory Restore" on next power up, and save this flag

      s16_Factory_Restore_On_Power_Up = 1;
      BGVAR(u16_Lexium_Bg_Action) |= LEX_DO_NV_SAVE;
   }

//  Older version, to perform "Factory Restore" immediately:
//    // If the user writes a 1 and the nv save bit is not yet set.
//    if ((lparam == 1) && ((BGVAR(u16_Lexium_Bg_Action) & LEX_DO_FACTORY_RESTORE) == 0))
//    {
//       // Set the do factory restore bit
//       BGVAR(u16_Lexium_Bg_Action) |= LEX_DO_FACTORY_RESTORE;
//    }

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadFactoryRestoreStatusViaPParam
// Description:
//          This function is called upon a read access of
//          P-parameter P10-09. A return value of:
//          1 : factory restore process is pending
//          0 : factory restore process was not requested
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadFactoryRestoreStatusViaPParam(long long* lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   *lparam = s16_Factory_Restore_On_Power_Up;
//
//    if((BGVAR(u16_Lexium_Bg_Action) & LEX_DO_FACTORY_RESTORE) == 0)
//    {
//       *lparam = 0; // No factory restore process pending
//    }
//    else
//    {
//       *lparam = 1; // Factory restore process pending
//    }

   return SAL_SUCCESS;
}

/////////   Support for MTP     /////////

// This will return the value of a single byte from the MTP Data (taken from the image array)
int GetByteFromMTP(unsigned int u16_byte_address, int *s16_byte_data)
{
   unsigned int u16_long_address, u16_shr_value;

   if (u16_byte_address >= MTP_SIZE_IN_BYTES) return (VALUE_OUT_OF_RANGE);

   u16_long_address = u16_byte_address >> 2;        // Int address to be read
   u16_shr_value = ((u16_byte_address % 4) << 3);  // Offset inside the long address

   *s16_byte_data = (u32_MTP_Image[u16_long_address] >> u16_shr_value) & 0x00FF;

   return (SAL_SUCCESS);
}


// This will return the value of a single word from the MTP Data (taken from the image array)
int GetWordFromMTP(unsigned int u16_byte_address, int *s16_word_data)
{
   unsigned int u16_long_address, u16_shr_value;

   if (u16_byte_address >= (MTP_SIZE_IN_BYTES - 1)) return (VALUE_OUT_OF_RANGE);

   u16_long_address = u16_byte_address >> 2;       // long address to be read
   u16_shr_value = ((u16_byte_address % 4) << 3);  // Offset inside the long address

   switch (u16_shr_value)
   {
      case 0:
         *s16_word_data = (int)(u32_MTP_Image[u16_long_address] & 0x0000FFFF);
      break;

      case 8:
      case 16:
         *s16_word_data = (int)((u32_MTP_Image[u16_long_address] >> u16_shr_value) & 0x0000FFFF);
      break;

      default: // SHR=24
         *s16_word_data = (int)(((u32_MTP_Image[u16_long_address] >> 24) & 0x000000FF) | ((u32_MTP_Image[u16_long_address+1] << 8) & 0x0000FF00));
      break;
   }
   return (SAL_SUCCESS);
}


// This will return the value of a single long from the MTP Data (taken from the image array)
// It does not have to be aligned
int GetLongFromMTP(unsigned int u16_byte_address, long *s32_long_data)
{
   unsigned int u16_long_address, u16_shr_value;

   if (u16_byte_address >= (MTP_SIZE_IN_BYTES - 3)) return (VALUE_OUT_OF_RANGE);

   u16_long_address = u16_byte_address >> 2;       // long address to be read
   u16_shr_value = ((u16_byte_address % 4) << 3);  // Offset inside the long address

   switch (u16_shr_value)
   {
      case 0:
         *s32_long_data = u32_MTP_Image[u16_long_address];
      break;

      case 8:
         *s32_long_data = ((u32_MTP_Image[u16_long_address] >> 8) & 0x00FFFFFF) | ((u32_MTP_Image[u16_long_address+1] << 24) & 0xFF000000);
      break;

      case 16:
         *s32_long_data = ((u32_MTP_Image[u16_long_address] >> 16) & 0x0000FFFF) | ((u32_MTP_Image[u16_long_address+1] << 16) & 0xFFFF0000);
      break;

      default: // SHR=24
         *s32_long_data = ((u32_MTP_Image[u16_long_address] >> 24) & 0x000000FF) | ((u32_MTP_Image[u16_long_address+1] << 8) & 0xFFFFFF00);
      break;
   }

   return (SAL_SUCCESS);
}


// Retutn the address of the base address of the specified segment
// Read the segment number. If not equal to the requested one, jump to the next till the requested ne is found
int GetSegmentAddress(unsigned int u16_data_segment, unsigned int *u16_segment_address)
{
   unsigned int u16_address = 0, u16_iteration_cntr = 0;
   int s16_read_data, s16_ret_val = SAL_SUCCESS;

   while (u16_iteration_cntr < 100) // 100 is to avoid endless loop (no more than 100 segments)
   {
      u16_iteration_cntr++;

      if ((u16_address + 4) > MTP_SIZE_IN_BYTES) return NOT_AVAILABLE;

      s16_ret_val = GetWordFromMTP(u16_address, &s16_read_data);
      if (s16_ret_val == SAL_SUCCESS) // Compare uiDataFieldIdentification to u16_data_segment to check if segment found
      {
         if (s16_read_data == 0) // If 0 then segment not found (as 0 is not a valid segment value)
            return NOT_AVAILABLE;
         else
         {
            if (s16_read_data == u16_data_segment) // Segment found, exit function
            {
               *u16_segment_address = u16_address;
               return SAL_SUCCESS;
            }
            else // Look for the next segment by checking the offset stored on segment base address+4
            {
               if ((u16_address+4) > MTP_SIZE_IN_BYTES) return NOT_AVAILABLE;
               else
               {
                  s16_ret_val = GetWordFromMTP(u16_address+4, &s16_read_data);
                  if (s16_ret_val == SAL_SUCCESS) // Add uiDataFieldSize to u16_address
                  {
                     if (s16_read_data < 4) // If <4 then segment not found (as 4 is the minumum segment size)
                        return NOT_AVAILABLE;
                     else
                        u16_address += s16_read_data;
                  }
                  else return s16_ret_val;
               }
            }
         }
      }
      else return s16_ret_val;
   }
   return s16_ret_val;
}


// Get data from a specific address in a specific segment in the MTP
// Refer to I:\eng\Magnetic Encoder\Firmware\Documents\MTP for more details
int GetDataFromMTP(unsigned int u16_data_segment, unsigned int u16_address_in_segment, long *s32_data, unsigned int u16_MTP_size)
{
   unsigned int u16_address;
   int s16_read_data, s16_ret_val = SAL_SUCCESS;
   long s32_read_data;

   s16_ret_val = GetSegmentAddress(u16_data_segment, &u16_address);
   if (s16_ret_val == SAL_SUCCESS)
   {
      // Segement found - Read its size for range check
      s16_ret_val = GetWordFromMTP(u16_address+4, &s16_read_data);
      if (s16_ret_val == SAL_SUCCESS) // Compare uiDataFieldSize to u16_address_in_segment
      {
         if (s16_read_data < u16_address_in_segment) // Check if address in segment range
         {
            return VALUE_OUT_OF_RANGE;
         }
         else // Address in range, read it according to the size
         {
            u16_address += u16_address_in_segment;

            if (u16_MTP_size == 8) s16_ret_val = GetByteFromMTP(u16_address, &s16_read_data);
            else
               if (u16_MTP_size == 16) s16_ret_val = GetWordFromMTP(u16_address, &s16_read_data);
               else
                  s16_ret_val = GetLongFromMTP(u16_address, &s32_read_data);

            if (s16_ret_val == SAL_SUCCESS)
            {
               if (u16_MTP_size == 8) *s32_data = ((long)s16_read_data) & 0x000000FF;
               else if (u16_MTP_size == 16) *s32_data = ((long)s16_read_data) & 0x0000FFFF;
                  else *s32_data = s32_read_data;
            }
            else return s16_ret_val;
         }
      }
   }
   return s16_ret_val;
}


void CalcMTPCrc(unsigned int *u16_crc_value, unsigned int u16_MTP_crc_polynom, unsigned int u16_data)
{
   unsigned int u16_pos_bit, u16_bit_position, u16_reg_bit, u16_data_bit;

   u16_bit_position = 0x8000;
   for (u16_pos_bit = 0; u16_pos_bit < 16; u16_pos_bit++)
   {
      u16_data_bit = ((u16_data & u16_bit_position) == u16_bit_position) ? 1 : 0;
      u16_reg_bit  = (((*u16_crc_value) & 0x8000) == 0x8000) ? 1 : 0;
      if (u16_reg_bit != u16_data_bit)
      {
         *u16_crc_value = ((*u16_crc_value) << 1) ^ u16_MTP_crc_polynom;
      }
      else
      {
         *u16_crc_value = ((*u16_crc_value) << 1);
      }
      u16_bit_position = u16_bit_position >> 1;
   }
}

unsigned int u16_Crc_Debug_Value1, u16_Crc_Debug_Value2, u16_Crc_Debug_Value3, u16_Crc_Debug_Value4;
int InitFromMTP(int drive)
{
   // AXIS_OFF;
   static unsigned int u16_param_index = 0, u16_byte_address = 0, u16_char_index = 0, u16_MTP_entry_cntr = 0;
   static unsigned int u16_timeout_cntr = 0, u16_read_retry_cntr, u16_datasegment2 = 9;
   static unsigned int u16_datasegment2_size = 0, u16_retry_counter = 0, u16_data_field_counter = 0;
   static unsigned long u32_owner = 0;
   static          int s16_num_of_hidden_datafields = 0, s16_last_segment = 0;
   static long s32_time_stamp;
   long s32_value = 0L, s32_value_2 = 0L, s32_response = 0L;
   unsigned int u16_MTP_size = 0, u16_drive_size = 0, u16_crc_value, u16_index_in_seg, u16_segment_size;
   int s16_temp_size = 0, s16_dummy, s16_ret_val = SAL_NOT_FINISHED;

   // If Feedback not ready to operate, wait till it is
   if ((FEEDBACK_SERVOSENSE)&& (BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK)) return s16_ret_val;
   if ((FEEDBACK_STEGMANN) && (BGVAR(s16_DisableInputs) & HIFACE_DIS_MASK))     return s16_ret_val;
   if(!(BGVAR(s16_DisableInputs) & MTP_READ_DIS_MASK))                          return s16_ret_val;

   switch (BGVAR(u16_Read_Mtp_State))
   {
      case MTP_INIT_INIT:  // Init state
         u16_param_index      = 0;
         u16_byte_address     = 0;
         u16_read_retry_cntr  = 0;

         BGVAR(s16_Motor_Drive_Mismatch) = 0;
         BGVAR(u16_MTP_Load_Done) = 0;
         // Set CRC Polynomial according to MTPMODE, to ensure incorrect CRC calculations if the MTP Mode
         // is different from the Mode for which the Database was intended.
         if ( (BGVAR(u16_MTP_Mode) == 1) || (BGVAR(u16_MTP_Mode) == 2) ) // Calculate 16bit CRC with polynomial
            BGVAR_DEFINE(u16_MTP_CRC_Polynom) = 0x1021; //  x^16 + x^12 + x^5 + 1 for SE (SrvSns and Hiperface)
         else if (BGVAR(u16_MTP_Mode) == 3) // Calculate 16bit CRC with polynomial x^16 + x^12 + x^7 + 1
            BGVAR_DEFINE(u16_MTP_CRC_Polynom) = 0x1081; // for CDHD (SrvSns and Hiperface)

         if (FEEDBACK_SERVOSENSE)
         {
            s16_ret_val = SrvSns_Acquire(&u32_owner, drive);            //Acquire ServoSense device
            if (s16_ret_val == SAL_SUCCESS)
            {
               u16_timeout_cntr = 0;
            }
            else if (s16_ret_val != SAL_NOT_FINISHED)
            {
               break;
            }
            else
               u16_timeout_cntr++;

            if(u16_timeout_cntr > 100)
            {
               BGVAR(s16_MTP_Error) = 9;
               BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
               SrvSns_Release(&u32_owner, drive);
               s16_ret_val = SRVSNS_DRV_ACQ_TIMEOUT;
               break;
            }
         }
         else
            u16_timeout_cntr = 0;

         u16_retry_counter = 0;
         if (BGVAR(u16_MTP_Mode) == 2)
            u16_datasegment2 = 9;
         else if (BGVAR(u16_MTP_Mode) == 3)
            u16_datasegment2 = 0;

         BGVAR(u16_MTP_ID) = 0;  // reset the mtp data crc

         if ( (s16_ret_val == SAL_SUCCESS) && FEEDBACK_SERVOSENSE )
            BGVAR(u16_Read_Mtp_State) = MTP_INIT_COPY_FROM_FEEDBACK;
         else if(FEEDBACK_STEGMANN) // Hiperface
            BGVAR(u16_Read_Mtp_State) = MTP_INIT_HF_FEEDBACK_PREPARE;
      break;

      // On HiperFace first needs to detect if MPHASE data is stored in DataField 5 or 9
      // This will be done according to the validity of DataField 0
      // The procedure described in: I:\eng\Magnetic Encoder\Firmware\Documents\MTP\Motor Type Plate - Specification.doc Sec 4.5.1.1
      // In Short: Check if u16_segment_size==data field size and uiDataFieldIdentification==0xF000 to check if DataSegement 1 is located in Datafield 0 or 4
      case MTP_INIT_HF_FEEDBACK_PREPARE:
         s16_ret_val = HifaceGetFieldSize(0, &u16_segment_size);
//         u16_Crc_Debug_Value2 = u16_segment_size;
         if ( (s16_ret_val == SAL_SUCCESS) && (BGVAR(u16_MTP_Mode) == 2) )
         {
            if (u16_segment_size >= 8)
            {
               s16_ret_val = HifaceReadAddr(0, 4, &s32_response); // Read uiDataFieldSize
               if (s16_ret_val == SAL_SUCCESS)
               {
//                  u16_Crc_Debug_Value3 = (unsigned int)(s32_response & 0x0000FFFF);
//                  u16_Crc_Debug_Value4 = (unsigned int)(s32_response >> 8);
                  if ((s32_response & 0x0000FFFF) == u16_segment_size)
                  {
                     s16_ret_val = HifaceReadAddr(0, 0, &s32_response); // Read uiDataFieldIdentification
                     if (s16_ret_val == SAL_SUCCESS)
                     {
                        if ((s32_response & 0x0000FFFF) == 0x0001)
                        u16_datasegment2 = 5;
//                        u16_Crc_Debug_Value4 = u16_datasegment2;
                     }
                     else { BGVAR(s16_MTP_Error) = 8; BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT; break;}
                  }
               }
               else { BGVAR(s16_MTP_Error) = 7; BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT; break;}
            }
         }
         else if ( (s16_ret_val == SAL_SUCCESS) /* && (BGVAR(u16_MTP_Mode) != 1) */ )
         {
            u16_datasegment2 = 0;
            u16_datasegment2_size = u16_segment_size;
            BGVAR(u16_Read_Mtp_State) = MTP_INIT_COPY_FROM_FEEDBACK;
            break;
         }
         else // Hiface Read Failure
         {
            u16_retry_counter++;
            if (u16_retry_counter > 1)
            {
               BGVAR(s16_MTP_Error) = 6;
               BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
               break;
            }
            else
            {
               s16_ret_val = SAL_NOT_FINISHED;
               break;
            }
         }

         u16_datasegment2_size = 0;
         s16_ret_val = HifaceGetFieldSize(u16_datasegment2, &u16_datasegment2_size);
//         u16_Crc_Debug_Value2 = s16_ret_val;
//         u16_Crc_Debug_Value3 = u16_datasegment2_size;
//         u16_Crc_Debug_Value4 = u16_datasegment2;
         if ((s16_ret_val == SAL_SUCCESS) && u16_datasegment2_size)
            BGVAR(u16_Read_Mtp_State)   = MTP_INIT_COPY_FROM_FEEDBACK;
         else // Could not read the relevant data, probably means datafields on the Hiperface needs
            BGVAR(u16_Read_Mtp_State)   = MTP_HF_FEEDBACK_READ_DATAFIELD_3; // to be opened
         s16_ret_val = SAL_NOT_FINISHED;
      break;

      case MTP_HF_FEEDBACK_READ_DATAFIELD_3: // Copy Datafield 3 so the relevant segments can be opened
         u16_timeout_cntr++;
         if (u16_timeout_cntr > 500)
         {
             BGVAR(s16_MTP_Error) = 10;
             BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
             s16_ret_val = FDBKTYPE_COMM_ERR;
             break;
         }

         // This DataField 3 till word #28 holds the sizes of the next following datafileds
         // Copy it temporarily to u32_MTP_Image, later it will be over-written when not used
         u16_Crc_Debug_Value4 = u16_byte_address;
         if (u16_byte_address <= 28)
            s16_ret_val = HifaceReadAddr(3, u16_byte_address, &s32_response);
         else
         {
            GetWordFromMTP(18, &s16_num_of_hidden_datafields);
            u16_Crc_Debug_Value2 = s16_num_of_hidden_datafields;
            if (s16_num_of_hidden_datafields > 6) s16_num_of_hidden_datafields = 6;
            if (s16_num_of_hidden_datafields < 0) s16_num_of_hidden_datafields = 0;
            u16_data_field_counter = 0;
            BGVAR(u16_Read_Mtp_State) = MTP_HF_FEEDBACK_OPEN_DATAFIELDS;
            s16_ret_val = SAL_NOT_FINISHED;
         }

         if (s16_ret_val != SAL_NOT_FINISHED)
         {
            if (s16_ret_val != SAL_SUCCESS)
            {
               u16_read_retry_cntr++;
               if (u16_read_retry_cntr > 3) // Allow 3 times reading failure in a row
               {
                  BGVAR(s16_MTP_Error) = 11;                  // Indicate an Error
                  BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
               }
               else
                  s16_ret_val = SAL_NOT_FINISHED;
            }
            else
            {
               u32_MTP_Image[u16_byte_address>>2] = (unsigned long)s32_response;
               u16_byte_address += 4;

               u16_read_retry_cntr = 0;
               u16_timeout_cntr = 0;
               s16_ret_val = SAL_NOT_FINISHED;
            }
         }
      break;

      case MTP_HF_FEEDBACK_OPEN_DATAFIELDS:
         u16_Crc_Debug_Value3 = u16_data_field_counter;
         if (u16_data_field_counter < s16_num_of_hidden_datafields)
         {
            GetWordFromMTP(20 + (u16_data_field_counter << 1), &s16_temp_size);
            s16_ret_val = HifaceOpenDataField(u16_data_field_counter + 4, s16_temp_size, 0);
            if (s16_ret_val != SAL_SUCCESS)
            {
               BGVAR(s16_MTP_Error) = 12;               // Indicate an Error
               BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
            }
            else
            {
               s16_ret_val = SAL_NOT_FINISHED;
               u16_data_field_counter++;
               s32_time_stamp = Cntr_1mS;
               BGVAR(u16_Read_Mtp_State) = MTP_HF_FEEDBACK_DELAY;
            }
         }
         else
         {
            u16_timeout_cntr = 0;
            u16_byte_address = 0;
            u16_read_retry_cntr = 0;
            s16_ret_val = SAL_NOT_FINISHED;
            s32_time_stamp = Cntr_1mS;
            BGVAR(u16_Read_Mtp_State) = MTP_HF_FEEDBACK_FINAL_DELAY;
         }
      break;

      case MTP_HF_FEEDBACK_FINAL_DELAY:
      case MTP_HF_FEEDBACK_DELAY:
         if (PassedTimeMS(200L,s32_time_stamp))
         {
            if (BGVAR(u16_Read_Mtp_State) == MTP_HF_FEEDBACK_FINAL_DELAY)
               BGVAR(u16_Read_Mtp_State) = MTP_INIT_HF_FEEDBACK_PREPARE; // After all DataFields are open, go back to init from MTP
            else
               BGVAR(u16_Read_Mtp_State) = MTP_HF_FEEDBACK_OPEN_DATAFIELDS;
         }
      break;

      case MTP_INIT_COPY_FROM_FEEDBACK:  // Copy MTP from Feedback to local buffer
         u16_timeout_cntr++;
         if (u16_timeout_cntr > 500)
         {
            BGVAR(s16_MTP_Error) = 2;
            BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
            if (FEEDBACK_SERVOSENSE)
            {
               SrvSns_Release(&u32_owner, drive);
               s16_ret_val = SRVSNS_BUSY_TIMEOUT;
            }
            else// Hiperface
            {
               s16_ret_val = FDBKTYPE_COMM_ERR;
            }
            break;
         }

         if (FEEDBACK_SERVOSENSE)
            s16_ret_val = SrvSnsReadAddr(u32_owner, u16_byte_address, &s32_response, drive, 0);
         else // if (BGVAR(u16_MTP_Mode) == 2) // Hiperface
         {
//            u16_Crc_Debug_Value3 = u16_datasegment2;
//            u16_Crc_Debug_Value4 = u16_datasegment2_size;
            if (u16_byte_address <= (u16_datasegment2_size - 4))
               s16_ret_val = HifaceReadAddr(u16_datasegment2, u16_byte_address, &s32_response);
            else
            {
               s32_response = 0L;
               s16_ret_val = SAL_SUCCESS;
            }
         }

         if (s16_ret_val != SAL_NOT_FINISHED)
         {
            if (s16_ret_val != SAL_SUCCESS)
            {
//               u16_Crc_Debug_Value2 = u16_byte_address;
//               u16_Crc_Debug_Value3 = s16_ret_val;
               u16_read_retry_cntr++;
               if (u16_read_retry_cntr > 3) // Allow 3 times reading failure in a row
               {
                  BGVAR(s16_MTP_Error) = 1;                  // Indicate an Error
                  BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
                  if (FEEDBACK_SERVOSENSE)
                     SrvSns_Release(&u32_owner, drive);
               }
               else
                  s16_ret_val = SAL_NOT_FINISHED;
            }
            else
            {
               u32_MTP_Image[u16_byte_address >> 2] = (unsigned long)s32_response;

               u16_byte_address += 4;
               if (u16_byte_address >= MTP_SIZE_IN_BYTES)
               {
                  u16_MTP_entry_cntr = 0;
                  s16_last_segment = 0;
                  BGVAR(u16_Read_Mtp_State) = MTP_INIT_VALIDATE;
                  if(FEEDBACK_SERVOSENSE)
                      SrvSns_Release(&u32_owner, drive);
               }

               u16_read_retry_cntr = 0;
               u16_timeout_cntr = 0;
               s16_ret_val = SAL_NOT_FINISHED;
            }
         }
      break;

      case MTP_INIT_VALIDATE: // Check that all segments needed appear in the MTP and their CRC are correct
         if (s16_last_segment != p_MTP_Data_Table[u16_MTP_entry_cntr].u16_data_segment)
         {
            s16_last_segment = p_MTP_Data_Table[u16_MTP_entry_cntr].u16_data_segment;

            if (GetDataFromMTP(s16_last_segment, 4, &s32_response, 16) != SAL_SUCCESS) // This will indicate if the segment exists, and gets its size while doing so
            {
               BGVAR(s16_MTP_Error) = 3;               // Indicate an Error
               BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
               s16_ret_val = NOT_AVAILABLE;
               break;
            }

            u16_segment_size = (unsigned int)s32_response;            // Check CRC value
            u16_crc_value = 0;
            for (u16_index_in_seg = 0; u16_index_in_seg < u16_segment_size; u16_index_in_seg += 2)
            {
               if (GetDataFromMTP(s16_last_segment, u16_index_in_seg, &s32_response, 16) != SAL_SUCCESS)
               {
                  BGVAR(s16_MTP_Error) = 4;                  // Indicate an Error
                  BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
                  s16_ret_val = NOT_AVAILABLE;
                  break;
               }
               if (u16_index_in_seg < (u16_segment_size - 2)) // Calculate CRC on all words except the CRC field
                  CalcMTPCrc(&u16_crc_value, BGVAR(u16_MTP_CRC_Polynom), (unsigned int)(s32_response));
            }
            u16_Crc_Debug_Value1 = u16_crc_value;
            if (u16_crc_value != (unsigned int)(s32_response))
            {
               BGVAR(s16_MTP_Error) = 5;               // Indicate an Error
               BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
               s16_ret_val = NOT_AVAILABLE;
               break;
            }
         }
         u16_MTP_entry_cntr++;
         if (u16_MTP_entry_cntr >= s16_MTP_Data_Table_Size)
            BGVAR(u16_Read_Mtp_State) = MTP_INIT_EXTRACT_DATA;
      break;

      case MTP_INIT_EXTRACT_DATA: // Extract data from MTP to local variables
         // Check if KCSATFACTOR and KCVG are initialized to a value <> 0 otherwise there will be no motion,
         if ((u16_param_index == 0) && (BGVAR(u16_MTP_Mode) == 1)) //  therefore something is wrong
         {
            GetDataFromMTP(0x8001, 60, &s32_value, 32); // This is the address of lPWMSatFactor
            GetDataFromMTP(0x8001, 64, &s32_value_2, 32); // This is the address of lCurrentLoopKvgGainFactor
            if ((s32_value == 0L) || (s32_value_2 == 0L))
            {
               BGVAR(s16_MTP_Error) = 13;
               BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
               break;
            }
         }

         u16_MTP_size = 8 * p_MTP_Data_Table[u16_param_index].u16_MTP_size_in_bytes;
         if (u16_MTP_size > 32) // More than 32-bits of data read it byte by byte
         {
            u16_char_index = 0;
            BGVAR(u16_Read_Mtp_State) = MTP_INIT_EXTRACT_STRING;
         }
         else
         {
            s16_ret_val = GetDataFromMTP(p_MTP_Data_Table[u16_param_index].u16_data_segment, p_MTP_Data_Table[u16_param_index].u16_address_in_data_segment, &s32_value, u16_MTP_size);

            if (s16_ret_val == SAL_SUCCESS)
            {
               u16_drive_size = 8 * p_MTP_Data_Table[u16_param_index].u16_drive_size_in_bytes;
               if (u16_MTP_size <= 16)
               {
                  if (u16_drive_size <= 16)
                  {
                     if (p_MTP_Data_Table[u16_param_index].f_scaling_factor != 1.0)
                        *(int*)p_MTP_Data_Table[u16_param_index].p_var_name = (int)((float)((int)s32_value) * p_MTP_Data_Table[u16_param_index].f_scaling_factor + 0.5);
                     else
                        *(int*)p_MTP_Data_Table[u16_param_index].p_var_name = (int)s32_value;
                  }
                  else // 32 Bits
                  {
                     if (p_MTP_Data_Table[u16_param_index].f_scaling_factor != 1.0)
                        *(long*)p_MTP_Data_Table[u16_param_index].p_var_name = (long)((float)(int)s32_value * p_MTP_Data_Table[u16_param_index].f_scaling_factor + 0.5);
                     else
                        *(long*)p_MTP_Data_Table[u16_param_index].p_var_name = (long)((int)s32_value);
                  }
               }
               else // MTP Size = 32
               {
                  if (u16_drive_size <= 16)
                  {
                     if (p_MTP_Data_Table[u16_param_index].f_scaling_factor != 1.0)
                        *(int*)p_MTP_Data_Table[u16_param_index].p_var_name = (int)((float)s32_value * p_MTP_Data_Table[u16_param_index].f_scaling_factor + 0.5);
                     else
                        *(int*)p_MTP_Data_Table[u16_param_index].p_var_name = (int)s32_value;

                     // Calculate MTP 16bit data 16bit CRC with polynomial: x^16 + x^12 + x^5 + 1
                     CalcMTPCrc(&(BGVAR(u16_MTP_ID)), BGVAR(u16_MTP_CRC_Polynom), (unsigned int)s32_value);
                  }
                  else
                  {
                     if (p_MTP_Data_Table[u16_param_index].f_scaling_factor != 1.0)
                        *(long*)p_MTP_Data_Table[u16_param_index].p_var_name = (long)((float)s32_value * p_MTP_Data_Table[u16_param_index].f_scaling_factor + 0.5);
                     else
                        *(long*)p_MTP_Data_Table[u16_param_index].p_var_name = (long)s32_value;

                     // Calculate MTP 32bit data 16bit CRC with polynomial: x^16 + x^12 + x^5 + 1
                     CalcMTPCrc(&(BGVAR(u16_MTP_ID)), BGVAR(u16_MTP_CRC_Polynom), (unsigned int)(s32_value & 0x0FFFF));
                     CalcMTPCrc(&(BGVAR(u16_MTP_ID)), BGVAR(u16_MTP_CRC_Polynom), (unsigned int)(s32_value >> 16));
                  }
               }

               u16_param_index++;
               s16_ret_val = SAL_NOT_FINISHED; // To continue calling this function
               if (u16_param_index >= s16_MTP_Data_Table_Size)
                  BGVAR(u16_Read_Mtp_State) = MTP_INIT_CONCLUDE;
            }
            else
            {
               BGVAR(s16_MTP_Error) = 14;
               s16_ret_val = MTP_READ_FAILURE;
               BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
            }
         }
      break;

      case MTP_INIT_EXTRACT_STRING: // This will read more than 32bits byte by byte (used for text fields)
         s16_ret_val = GetDataFromMTP(p_MTP_Data_Table[u16_param_index].u16_data_segment, p_MTP_Data_Table[u16_param_index].u16_address_in_data_segment + u16_char_index, &s32_value, 8);

         if (s16_ret_val == SAL_SUCCESS)
         {
            *(int*)((long)p_MTP_Data_Table[u16_param_index].p_var_name + (long)u16_char_index) = (int)s32_value;

            u16_char_index++;
            s16_ret_val = SAL_NOT_FINISHED; // To continue calling this function
            if (u16_char_index >= p_MTP_Data_Table[u16_param_index].u16_MTP_size_in_bytes)
            {
               *(int*)((long)p_MTP_Data_Table[u16_param_index].p_var_name + (long)u16_char_index) = 0;
               u16_char_index--;
               // Replace "0x20" padding in MTP by null terminator
               while ((u16_char_index>0) && (*(int*)((long)p_MTP_Data_Table[u16_param_index].p_var_name + (long)u16_char_index) == 0x20))
               {
                  *(int*)((long)p_MTP_Data_Table[u16_param_index].p_var_name + (long)u16_char_index) = 0;
                  u16_char_index--;
               }

               u16_param_index++;
               if (u16_param_index >= s16_MTP_Data_Table_Size)
                  BGVAR(u16_Read_Mtp_State) = MTP_INIT_CONCLUDE;
               else BGVAR(u16_Read_Mtp_State) = MTP_INIT_EXTRACT_DATA; // Continue to rest of params
            }
         }
         else
         {
            s16_ret_val = MTP_READ_FAILURE;
            BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;
         }
      break;

      case MTP_INIT_CONCLUDE:
         if (BGVAR(u16_MTP_Mode) == 1) // Init From ServoSense
         {
            // Check for motor/drive match (bundle check)
            // Drive power is defined accordingly:
            // DICONT DIPEAK SIZE    POWER
            // 0.64     2     1     0W-50W
            // 0.9     2.7    1     51W-100W
            // 1.4     4.5    1     101W-200W
            // 2.6     7.8    1     201W-400W
            // 4.5    13.5    1     401W-750W
            // 7      21.0    2     751W-1000W
            // 7      21.0    2     1001W-1500W ; Differ between the SIZE2 1000W/1500W done via the relevant EEPROM Field
            // 12     36.0    3     1501W-2000W
            // 23     61.0    4     2001W-3000W
            // 23     61.0    4     3001W-4500W ; Differ between the SIZE4 3000W/4500W done via the relevant EEPROM Field
            switch (BGVAR(s16_Drive_I_Peak_Arms))
            {
               case 27:
                  u32_Drive_Min_Power = 51;
                  u32_Drive_Max_Power = 100;
               break;
               case 45:
                  u32_Drive_Min_Power = 101;
                  u32_Drive_Max_Power = 200;
               break;
               case 78:
                  u32_Drive_Min_Power = 201;
                  u32_Drive_Max_Power = 400;
               break;
               case 135:
                  u32_Drive_Min_Power = 401;
                  u32_Drive_Max_Power = 750;
               break;
               case 210:  // Differ between 1000W and 1500W by the power EEprom field
                  if (BGVAR(u16_Drive_Power_Rating) == 1500)
                  {
                     u32_Drive_Min_Power = 1001;
                     u32_Drive_Max_Power = 1500;
                  }
                  else
                  {
                     u32_Drive_Min_Power = 751;
                     u32_Drive_Max_Power = 1000;
                  }
               break;
               case 360:
                  u32_Drive_Min_Power = 1501;
                  u32_Drive_Max_Power = 2000;
               break;
               case 610:  // Differ between 3000W and 4500W by the power EEprom field
                  if (BGVAR(u16_Drive_Power_Rating) == 3000)
                  {
                     u32_Drive_Min_Power = 2001;
                     u32_Drive_Max_Power = 3000;
                  }
                  else
                  {
                     u32_Drive_Min_Power = 3001;
                     u32_Drive_Max_Power = 4500;
                  }
               break;
               case 1190:
                  u32_Drive_Min_Power = 4501;
                  u32_Drive_Max_Power = 30000;  // (arbitrary big value, should be set correctly when next power rating will be available)
               break;

               default:
               case 20:
                  u32_Drive_Min_Power = 0;
                  u32_Drive_Max_Power = 50;
               break;
            }

            if ( (BGVAR(u32_MTP_Power) < u32_Drive_Min_Power) ||
                 (BGVAR(u32_MTP_Power) > u32_Drive_Max_Power)   )
            {  // Bundle check for SE only
               if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
                  BGVAR(s16_Motor_Drive_Mismatch) = 1;
            }

            if (BGVAR(u16_MTP_FdbkType) == 0x0009) // 0x0009 is SE MTP value for ServoSense
            {
               if(BGVAR(u32_MTP_Multi_Turn_Number) > 1)// SE MTP defines 1 turn for single turn encoder
                  BGVAR(u16_FdbkType) = SERVOSENSE_MULTI_TURN_COMM_FDBK;
               else
                  BGVAR(u16_FdbkType) = SERVOSENSE_SINGLE_TURN_COMM_FDBK;

               SalFdbkCommand((long long)BGVAR(u16_FdbkType), drive);
            }
            // Transform from MBEMF units on the MTP (0.1Vrms/Krpm) to MKT*1000 =>
            // (sqrt(3)*60)/sqrt(2)/2pi/10 = 1.169 // sqrt(2) is because MKT is in Nm/A and not Nm/Arms
            BGVAR(u32_Motor_Kt) = (unsigned long)((float)BGVAR(u32_MTP_Bemf) * 1.169);
            SalMotorKtCommand((long long)BGVAR(u32_Motor_Kt), drive);

            // Set MENCRES according to the number of bits define on the MTP (-2 because of lines instead of counts)
//            BGVAR(u32_User_Motor_Enc_Res) =  1L << (long)(BGVAR(u16_MTP_Mencres_Bits) - 2);
//            SalMotorEncResCommand((long long)BGVAR(u32_User_Motor_Enc_Res), drive);
            SalMotorEncResCommand((long long)(1L << (long)(BGVAR(u16_MTP_Mencres_Bits) - 2)), drive);

            // Set MFBDIR bit #0
            if (BGVAR(u16_MTP_MfbDir) == 1)
               BGVAR(u16_Motor_Feedback_Direction) |= 0x0001;
            else
               BGVAR(u16_Motor_Feedback_Direction) &= ~0x0001;
            // SalMotorFeedbackDirectionCommand((long long)BGVAR(u16_Motor_Feedback_Direction), drive); no need for this line as wrting to the internal param is enough

            // As SE direction and CDHD one are opposite, negate the direction value as read from the MTP
            if (BGVAR(u16_MTP_Dir) == 0) BGVAR(s16_Direction) = 1;
            else BGVAR(s16_Direction) = 0;

            if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
            {// Schneider drive
               if(BGVAR(u16_P1_01_CTL_Current) & 0x100)
               {// change direction according to P1-01 nibble "B" (0- FW=CCW (Schneider default) ; 1- FW=CW)
                // preform only if P1-01 nibble "B" is in reverse direction (=1).
                  if (BGVAR(s16_Direction) == 0) BGVAR(s16_Direction) = 1;
                  else                           BGVAR(s16_Direction) = 0;
               }
            }

            if(BGVAR(s16_Load_Defaults_From_MTP) == 1)            // load write default values from MTP
            {
               if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
               {// Schneider drive
                  // set P1-40 value to MSPEED from MTP, convert from internal to user units
                  BGVAR(s32_P1_40_VCM) = (long)MultS64ByFixS64ToS64((long long)BGVAR(u32_Mspeed),
                                     BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPM]).s64_unit_conversion_to_user_fix,
                                     BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPM]).u16_unit_conversion_to_user_shr);

                  // divide by 1000 to get the actual value (we are not passing through the parser)
                  BGVAR(s32_P1_40_VCM) /= 1000;

                  //SalWriteVCMCommand((long long)BGVAR(s32_P1_40_VCM),drive);   // update P1-40 value.
               }

               BGVAR(s32_V_Lim_Design) = (long)BGVAR(u32_Mspeed);    // Set VLIM to MSPEED (SE request)
               VLimCommand((long long)BGVAR(s32_V_Lim_Design), drive, &s16_dummy);

               BGVAR(s16_Load_Defaults_From_MTP) = 0;               // reset the load from MTP flag.
            }

            // Call MICONT sal function to init other design functions reside in the sal
            SalMotorIContCommand((long long)BGVAR(s32_Motor_I_Cont), drive);

            // If ILIM=0 set it to MIN(DIPEAK,MIPEAK), as it means parameters where not set by the user
            if (BGVAR(s32_Ilim_User) == 0L) //  (SE request))
            {
               BGVAR(s32_Ilim_User) = min(BGVAR(s32_Motor_I_Peak), BGVAR(s32_Drive_I_Peak));
            }
            SalIlimCommand((long long)BGVAR(s32_Ilim_User), drive);

            // Init MIFOLDFTHRESH to MICONT if not set
            if (BGVAR(u32_Motor_I_Fold_Fault_Threshold) == 0L)
               BGVAR(u32_Motor_I_Fold_Fault_Threshold) = (unsigned long)(((long long)BGVAR(s32_Motor_I_Cont)*26214LL) / (long long)BGVAR(s32_Drive_I_Peak));
            // Init MIFOLDWTHRESH to MIPEAK if not set
            if (BGVAR(u32_Motor_I_Fold_Warning_Threshold) == 0L)
               BGVAR(u32_Motor_I_Fold_Warning_Threshold) = (unsigned long)(((long long)BGVAR(s32_Motor_I_Peak)*26214LL) / (long long)BGVAR(s32_Drive_I_Peak));

            // This is to recalculate IMAX according to parameters read from the MTP
            CalcImax(DRIVE_PARAM);
            CalcFoldbackParam(DRIVE_PARAM, MOTOR_FOLDBACK);
         } // End of if (BGVAR(u16_MTP_Mode) == 1)

         if (BGVAR(u16_MTP_Mode) == 3) // Init From ServoSense for MPC Motors
         {
            BGVAR(u16_FdbkType) = BGVAR(u16_MTP_FdbkType);

            SalFdbkCommand((long long)BGVAR(u16_MTP_FdbkType), drive);// Assign the feedback type according to MTP

            // Set MENCRES according to the number of bits define on the MTP (-2 because of lines instead of counts)
            SalMotorEncResCommand((long long)(1L << (long)(BGVAR(u16_MTP_Mencres_Bits) - 2)), drive);// call the Mencres handler to trigger feedback config

            if (BGVAR(u8_MotorName)[0] == 0) // If 1st Character is NULL the Motor-Name String is empty...
            { // Copy the MTP Motor-Name into CDHD Motor-Name...
               strcpy(BGVAR(u8_MotorName), BGVAR(u8_MTP_MotorName));
               BGVAR(u16_MotorName_Mismatch) = 0;
            }
            else // If Motor-Name in CDHD Memory has some value, compare to MTP Motor-Name
            {
               if (strcmp(BGVAR(u8_MotorName), BGVAR(u8_MTP_MotorName)) != 0)
                  BGVAR(u16_MotorName_Mismatch) = 1;
               else // If Motor-Name is CDHD Memory is identical to MTP Motor-Name, reset Fault.
                  BGVAR(u16_MotorName_Mismatch) = 0;
            }

            if (BGVAR(u16_MTP_MfbDir) == 1)            // Set MFBDIR bit #0
               BGVAR(u16_Motor_Feedback_Direction) |= 0x0001;
            else
               BGVAR(u16_Motor_Feedback_Direction) &= ~0x0001;

//            BGVAR(s16_Direction) = BGVAR(u16_MTP_Dir); // Set DIR
            // per Dmitry to allow User to set Rotation Direction, May 25, 2016

            // Call MICONT and MIPEAK sal functions to init other design functions reside in the sal
            SalMotorIContCommand((long long)BGVAR(s32_Motor_I_Cont), drive);
            SalMotorIPeakCommand((long long)BGVAR(s32_Motor_I_Peak), drive);

            // Init MIFOLDFTHRESH to MICONT if not set
            if (BGVAR(u32_Motor_I_Fold_Fault_Threshold) == 0L)
               BGVAR(u32_Motor_I_Fold_Fault_Threshold) = (unsigned long)(((long long)BGVAR(s32_Motor_I_Cont) * 26214LL) / (long long)BGVAR(s32_Drive_I_Peak));
            // Init MIFOLDWTHRESH to MIPEAK if not set
            if (BGVAR(u32_Motor_I_Fold_Warning_Threshold) == 0L)
               BGVAR(u32_Motor_I_Fold_Warning_Threshold) = (unsigned long)(((long long)BGVAR(s32_Motor_I_Peak) * 26214LL) / (long long)BGVAR(s32_Drive_I_Peak));
            
            BGVAR(s32_V_Lim_Design) = (long)BGVAR(u32_Mspeed);    // Set VLIM=MSPEED to save the user to set this param when MTP exists
//            VLimCommand((long long)BGVAR(s32_V_Lim_Design), drive, &s16_dummy);

            // If ILIM = 0 set it to MIN(DIPEAK,MIPEAK), as it means parameters where not set by the user
            if (BGVAR(s32_Ilim_User) == 0L) //  (SE request))
            {
               BGVAR(s32_Ilim_User) = min(BGVAR(s32_Motor_I_Peak), BGVAR(s32_Drive_I_Peak));
            }
            CalcImax(drive); // Imax calc should be before Ilim calc
            SalIlimCommand((long long)BGVAR(s32_Ilim_User),drive);
            CalcFoldbackParam(drive, MOTOR_FOLDBACK);
         } // End of if (BGVAR(u16_MTP_Mode) == 3)

         // Add the difference between MPHASE stored on the MTP and the one used by the drive
         // For a CDHD drive (utilizing MTPMODE 3), if DIR = 1 is used, change MTP MPHASE by 180;
         // for a SE drive (utilizing MTPMODE 1), if DIR = 0 is used, change MTP PHASE by 180;
         if ( (BGVAR(s16_Direction) == 0) == (BGVAR(u16_MTP_Mode) == 1) )
            VAR(AX0_s16_Electrical_Phase_Offset) = BGVAR(u16_Electrical_Phase_Offset_User) + 0x8000;
         else
            VAR(AX0_s16_Electrical_Phase_Offset) = BGVAR(u16_Electrical_Phase_Offset_User);

         BGVAR(u16_MTP_Load_Done) = 1;
         // Initiate comm feedback state machine
         BGVAR(u16_Comm_Fdbk_Init_Ran) = 0;
         BGVAR(u16_Read_Mtp_State) = MTP_INIT_CONFIG;
      break;

      case MTP_INIT_CONFIG:
         // call the design routine with feedback config to initiate feedback init STM
         s16_ret_val = DesignRoutine(1, DRIVE_PARAM);
         if (s16_ret_val != SAL_NOT_FINISHED) BGVAR(u16_Read_Mtp_State) = MTP_INIT_INIT;

         if (s16_ret_val == SAL_SUCCESS)
         {
            // Call these functions to calc internal variables.
            VLimCommand((long long)BGVAR(s32_V_Lim_Design), drive, &s16_dummy);
            AnalogVelocityConfig(drive);

            BGVAR(s16_DisableInputs) &= ~MTP_READ_DIS_MASK;
            // Check if configured motor equal motor name on MTP
            CheckMotorNumberCompatibility(drive);
            // If the P1-82 parameter equals 0
            if ( (BGVAR(u32_Configuerd_MotorType) == 0) || // For SE, update only if no previous value
                 (BGVAR(u16_MTP_Mode) == 3)               ) // For CDHD with MPC, always update
            {  // Set P1-82 to the motor file name from the MTP - BYang00001306
               BGVAR(u32_Configuerd_MotorType) = BGVAR(u32_MotorFileName);
            }
         }
      break;
   }

   return s16_ret_val;
}


void CheckMotorNumberCompatibility(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   if ( (BGVAR(u32_Configuerd_MotorType) != 0)                       &&
        (BGVAR(u32_Configuerd_MotorType) != BGVAR(u32_MotorFileName))  )
   {
      BGVAR(u64_Sys_Warnings) |= MOTOR_TYPE_COMPATIBILITY_WRN_MASK;
   }
}


//**********************************************************
// Function Name: SalWriteCfgMotorType
// Description:
//   Write for P1-82, Configuerd Motor Type
// Author: Udi
// Algorithm:
//          Allow write any number.
//          A fault will result if value is different then MTP value for "MTP Motor Type" (== MTP Motor File Name)
// Revisions:
//**********************************************************
int SalWriteCfgMotorType(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u32_Configuerd_MotorType) = (long) param;
   CheckMotorNumberCompatibility(drive);

   return (SAL_SUCCESS);
}


// Added a backdoor here, setting MTPMODE=0x1234 will set MTPMODE=3 but will inhibit the bundle-check
int SalSetMtpMode(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL) return (VALUE_TOO_LOW);

   if ((lparam > 3LL) && (lparam != (long long)0x1234) && (lparam != (long long)0x2345)) return (VALUE_TOO_HIGH);

   if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
   {
      if ((unsigned int)lparam == 0x2345) return (NOT_AVAILABLE);
      // Dont support MTPMODE on SE HW as it uses SCIC for the modbus hence cannot be used to communicate
      if ( (lparam == 2LL) || (lparam == 3LL) )  return (NOT_AVAILABLE); // with hiperface

      // On SE as MTPMODE is supposed to be always "1", allow setting it to 0 only if password is not
      // activated after cycling power MTPMODE = 1 anyway
      if (((unsigned int)lparam == 0) && (!s16_Password_Flag)) return (PASSWORD_PROTECTED);
      if ((unsigned int)lparam == 0x1234)
      {
         lparam = 1LL;
      }
   }
   else
   {
      if ((unsigned int)lparam == 0x1234) return (NOT_AVAILABLE);
      if ((unsigned int)lparam == 0x2345)
      {
         lparam = 3LL;
      }
   }

   BGVAR(u16_MTP_Mode_User) = (unsigned int)lparam;

   BGVAR(u16_Bundle_Check) = 0;

   if ((BGVAR(u16_MTP_Mode_User) != 0x1234) && (BGVAR(u16_MTP_Mode_User) != 0x2345))
   {
      BGVAR(u16_Bundle_Check) = 1;
   }

   // This will cause a flt to force the user to save and cycle power for this to have affect
   if ((BGVAR(u16_MTP_Mode) != (int)lparam) && (u16_background_ran) && (lparam != 0L))
      BGVAR(u16_Power_Cycle_Needed) = 1;

   BGVAR(u16_MTP_Mode) = (unsigned int)lparam;

   return (MtpModeInit(BGVAR(u16_MTP_Mode), drive));
}


// Separated settings of Data Tables and Array Length to allow Internal Settings
int MtpModeInit(unsigned int mtp_mode, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (mtp_mode == 1)   // Fill MTP_Data_Table according to the MTPMODE
   {
      p_MTP_Data_Table = s_MTP_1_Data_Table;
      s16_MTP_Data_Table_Size = NUM_OF_MTP_1_PARAMS;
      s16_MTP_Data_Table_Inc_Dummies_Size = NUM_OF_MTP_1_PARAMS_AND_DUMMIES;
   }
   else if (mtp_mode == 2)
   {
      p_MTP_Data_Table = s_MTP_2_Data_Table;
      s16_MTP_Data_Table_Size = NUM_OF_MTP_2_PARAMS;
      s16_MTP_Data_Table_Inc_Dummies_Size = NUM_OF_MTP_2_PARAMS_AND_DUMMIES;
   }
   else
   {
      p_MTP_Data_Table = s_MTP_3_Data_Table;
      s16_MTP_Data_Table_Size = NUM_OF_MTP_3_PARAMS;
      s16_MTP_Data_Table_Inc_Dummies_Size = NUM_OF_MTP_3_PARAMS_AND_DUMMIES;
   }

   BGVAR(u16_MTP_Load_Done) = 0;

   return (SAL_SUCCESS);
}


/////////   End of Support for MTP     /////////

//**********************************************************
// Function Name: FlashWriteESI
// Description:
// This function rewrites ESI flash section
// u16_private_label = 1 - STX
// u16_private_label = 2 - PBA
// u16_private_label = 3 - AKRIBIS
// u16_private_label = 4 - MPC
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
/*int FlashWriteESI(unsigned int u16_private_label)
{
   static int s16_ESI_state = 0;
   int i;

   switch (s16_ESI_state)
   {
      case 0:
        //save current ESI data
       for(i = 0; i < 1024; i++)
       {
          s16_Debug_Ram[0][i] = SerialFlashReadByte(ESI_SECTOR_ADDR + i * 2);
          s16_Debug_Ram[0][i] = ((s16_Debug_Ram[0][i] << 8) & 0xFF00);
          s16_Debug_Ram[0][i] |= SerialFlashReadByte(ESI_SECTOR_ADDR + i * 2 + 1);
       }

       s16_ESI_state = 1;
      break;

      case 1:   // Erase ESI flash sector
         if (SerialFlashEraseSector(ESI_SECTOR_ADDR) == FLASH_STATUS_SUCCESS)
            s16_ESI_state = 2;
      break;

      case 2:
         //write 16 bytes until the vendor ID (0x00 - 0x0F)
         SerialFlashWriteBuffer(ESI_SECTOR_ADDR, 8, (unsigned int*)&s16_Debug_Ram[0][0]);

         //write 116 bytes from after vendor ID until first brand name (0x12 - 0x85)
         SerialFlashWriteBuffer(ESI_SECTOR_ADDR + 0x12, 58, (unsigned int*)&s16_Debug_Ram[0][9]);

         //write 32 bytes from after first brand name to second brand name (0x8A - 0xA9)
         SerialFlashWriteBuffer(ESI_SECTOR_ADDR + 0x8A, 16, (unsigned int*)&s16_Debug_Ram[0][69]);

         //write the rest of the data from after second brand name to the end (0xAE - end)
         SerialFlashWriteBuffer(ESI_SECTOR_ADDR + 0xAE, 929, (unsigned int*)&s16_Debug_Ram[0][87]);

         //write private label data
         switch(u16_private_label)
         {
            case 1://CDHD
               //Vendor ID
               SerialFlashWriteByte(ESI_VENDOR_ID_HI_ADDR, 0x0002);
               SerialFlashWriteByte(ESI_VENDOR_ID_LO_ADDR, 0x00E1);

               //first brand name
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR1, 0x0043); //C
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR2, 0x0044); //D
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR3, 0x0048); //H
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR4, 0x0044); //D

               //second brand name
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR1, 0x0043); //C
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR2, 0x0044); //D
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR3, 0x0048); //H
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR4, 0x0044); //D
            break;

            case 2://PBA
               //Vendor ID
               SerialFlashWriteByte(ESI_VENDOR_ID_HI_ADDR, 0x0002);
               SerialFlashWriteByte(ESI_VENDOR_ID_LO_ADDR, 0x00BA);

               //first brand name
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR1, 0x0020); // space
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR2, 0x004D); //M
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR3, 0x0054); //T
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR4, 0x0020); // space

               //second brand name
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR1, 0x0020); // space
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR2, 0x004D); //M
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR3, 0x0054); //T
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR4, 0x0020); // space
            break;

            case 3://AKRIBIS
               //Vendor ID
               SerialFlashWriteByte(ESI_VENDOR_ID_HI_ADDR, 0x0002);
               SerialFlashWriteByte(ESI_VENDOR_ID_LO_ADDR, 0x00E1);

               //first brand name
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR1, 0x0020); // space
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR2, 0x0041); //A
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR3, 0x0053); //S
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR4, 0x0044); //D

               //second brand name
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR1, 0x0020); //space
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR2, 0x0041); //A
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR3, 0x0053); //S
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR4, 0x0044); //D
            break;

            case 4://MPC
               //Vendor ID
               SerialFlashWriteByte(ESI_VENDOR_ID_HI_ADDR, 0x0006);
               SerialFlashWriteByte(ESI_VENDOR_ID_LO_ADDR, 0x0078);

               //first brand name
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR1, 0x0046); //F
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR2, 0x0050); //P
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR3, 0x0052); //R
               SerialFlashWriteByte(ESI_FIRST_BRAND_NAME_ADDR4, 0x004F); //O

               //second brand name
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR1, 0x0046); //F
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR2, 0x0050); //P
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR3, 0x0052); //R
               SerialFlashWriteByte(ESI_SECOND_BRAND_NAME_ADDR4, 0x004F); //O
            break;
         } // end of switch(u16_private_label)

         s16_ESI_state = 3;
      break;
   }//end of switch (s16_ESI_state)

   return (s16_ESI_state == 3);   //return value: 0 not ended, 1 ended
}
*/
// This will indicate if MTPMODE is being used and read correctly
// 0 - MTP not used
// 1 - MTP used and read correctly
// 2 - MTP used but not read correctly
// 3 - MTP reading needs power cycle or clearfaults
// 4 - MTP used and reading is in processs
int MtpStatusCommand(long long *data,int drive)
{
   int s16_ret_val = 0; // Defult is that MTP is not used
   REFERENCE_TO_DRIVE; // Defeat compilation error

   if (BGVAR(u16_MTP_Mode) > 0)
   {
      if (BGVAR(s16_MTP_Error)) s16_ret_val = 2; // Check is there was and error reading the MRP
      else if (BGVAR(u16_Read_Mtp_State) > MTP_INIT_INIT) s16_ret_val = 4; // Check if the MTP reading is in process looking at the state machine
      else if ((BGVAR(s64_SysNotOk_2) & CYCLE_POWER_NEEDED_FLT_MASK) || (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK) || (BGVAR(s16_DisableInputs) & MTP_READ_DIS_MASK) || (BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK)) s16_ret_val = 3; // If MTPMODE>0 was set or feedback fault indicate that MTP is not read
      else s16_ret_val = 1; // Otherwise - all OK
   }

   *data = (long long)s16_ret_val;

   return SAL_SUCCESS;
}

int MtpInfoCommand(int drive)
{
   static int s16_state = 0;
   REFERENCE_TO_DRIVE;

   if (u8_Output_Buffer_Free_Space < 80) return SAL_NOT_FINISHED;

   switch (s16_state)
   {
      case 0:
         if ( (BGVAR(u16_MTP_Mode) != 1) && (BGVAR(u16_MTP_Mode) != 3) )
            PrintStringCrLf("MTP is not used (MTPMODE<>1 && MTPMODE<>3)", 0);
         else
         {
            PrintStringCrLf("MTP Data:", 0);
            PrintString("MTP Mode: ", 0);
            PrintSignedInt16(BGVAR(u16_MTP_Mode));
            PrintCrLf();
            s16_state++;
         }
      break;

      case 1:
         PrintString("Motor Name: ", 0);
         PrintStringCrLf(u8_MotorName, 0);
         if (BGVAR(u16_MTP_Mode) == 1)
         {
            PrintString("Motor File Name #: ", 0);
            PrintUnsignedInt32(u32_MotorFileName);
            PrintCrLf();
         }
         else if (BGVAR(u16_MTP_Mode) == 3)
         {
            PrintString("Motor Number #: ", 0);
            PrintUnsignedInt32(u32_MotorFileName);
            PrintCrLf();
            PrintString("Motor Ser. Num.: ", 0);
            PrintStringCrLf(u8_MotorSerialNumber, 0);
         }
         s16_state++;
      break;

      case 2:
         if (BGVAR(u16_MTP_Mode) == 3)
         {
            PrintString("Motor Mfg. Date: ", 0);
            PrintUnsignedInt16((int)(u32_MTP_Mfg_Date >> 16));
            PrintString("-", 0);
            PrintUnsignedInt16((int)((u32_MTP_Mfg_Date >> 8) & 0xFF));
            PrintString("-", 0);
            PrintUnsignedInt16((int)(u32_MTP_Mfg_Date & 0xFF));
            PrintCrLf();
         }
         PrintString("Feedback #: ", 0);
         PrintUnsignedInt16(u16_MTP_FdbkType);
         PrintCrLf();
         PrintString("Motor-Encoder #: ", 0);
         PrintUnsignedInt16(u16_MTP_MEncType);
         PrintCrLf();
         PrintString("Bits in Rev: ", 0);
         PrintUnsignedInt16(u16_MTP_Mencres_Bits);
         PrintCrLf();
         s16_state++;
      break;

      case 3:
         PrintString("Motor Power: ", 0);
         PrintUnsignedInt32(u32_MTP_Power);
         if (BGVAR(u16_MTP_Mode) == 1)
         {
            PrintString(" (Drive Power Range: ", 0);
            PrintUnsignedInt32(u32_Drive_Min_Power);
            PrintChar('-');
            PrintUnsignedInt32(u32_Drive_Max_Power);
            PrintString(")", 0);
         }
         PrintCrLf();
         if (BGVAR(u16_MTP_Mode) == 1)
         {
            PrintString("Back-Emf: ", 0);
            PrintUnsignedInt32(u32_MTP_Bemf);
         }
         else if (BGVAR(u16_MTP_Mode) == 3)
         {
            PrintString("Torque-Constant: ", 0);
            PrintUnsignedInt32(u32_Motor_Kt);
         }
         PrintCrLf();
/*         PrintString("Max Temperature: ", 0);
         PrintSignedInt16(s16_ServoSense_Therm_Trip_Level);
         PrintCrLf();
         PrintString("Brake Engage Time: ", 0);
         PrintSignedInt16(u16_Motor_Brake_Engage_Time);
         PrintCrLf();
         PrintString("Brake Release Time: ", 0);
         PrintSignedInt16(u16_Motor_Brake_Release_Time);
         PrintCrLf(); */
         s16_state++;
      break;

      case 4:
         PrintString("Max Temperature: ", 0);
         PrintSignedInt16(s16_ServoSense_Therm_Trip_Level);
         PrintCrLf();
         PrintString("Brake Engage Time: ", 0);
         PrintSignedInt16(u16_Motor_Brake_Engage_Time);
         PrintCrLf();
         PrintString("Brake Release Time: ", 0);
         PrintSignedInt16(u16_Motor_Brake_Release_Time);
         PrintCrLf();
         s16_state = 0;
      break;
   }

   if (s16_state == 0) return SAL_SUCCESS;
   else return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: InitDebugLog
// Description: Initilizes debug log sector in flash.
//
// Debug log uses entire flash sector.
// First it checks for a stamp on the first address of the flash sector.
// If stamp does not exist, it erases the flash sector, writes the stamp, and inits a pointer to the word after stamp.
// If stamp exists, it inits a pointer to the first empty word (empty word is 0xFFFF)
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
void InitDebugLog(void)
{
   if ((u16_Vendor_ID[0] == 0x0001) && (u16_Device_ID[0] == 0x2249)) return;  // Not supported on Spansion 16Mbit

   // init pointer to debug log flash sector
   p_u16_Debug_Log_Flash = &Flash[DEBUG_LOG_BLOCK_ADDRESS];

   if (*p_u16_Debug_Log_Flash == 1234)
   {
      // Stamp exists. Look for the first empty word (empty word is 0xFFFF), up to 24000 words
      while ((*p_u16_Debug_Log_Flash != 0xFFFF) && (p_u16_Debug_Log_Flash <= &Flash[DEBUG_LOG_BLOCK_ADDRESS + 24000]))
      {
         (unsigned long)p_u16_Debug_Log_Flash++;
      }
   }
   else
   {
      // Stamp does not exist. Erase debug log block in flash.
      while (SalClearDebugLog(555LL, 0) == SAL_NOT_FINISHED);
   }
}


//**********************************************************
// Function Name: LogDebugData
// Description: Write debug data to the next available location in the log sector in flash (up to 24000 words).
//
// Comment: the last 'if' condition is part of the debug log mechanism. The code above is specific to the desired debug.
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
void LogDebugData(unsigned int u16_data)
{
   // Do not log the same data again
   if (u16_Debug_Log_Prev_Data == u16_data) return;

   // SalRestoreFactorySettingsCommand() can be called from several places and from FACTORYRESTORE command.
   // If there is an indication that SalRestoreFactorySettingsCommand() was called, do not log the indication inside SalRestoreFactorySettingsCommand(),
   // otherwise, do log it (indicates that FACTORYRESTORE command was issued).
   if ( (u16_data == 13) &&
        ((u16_Debug_Log_Prev_Data == 11) || (u16_Debug_Log_Prev_Data == 12) || (u16_Debug_Log_Prev_Data == 14) || (u16_Debug_Log_Prev_Data == 15) || (u16_Debug_Log_Prev_Data == 16)) )
   {
      return;
   }

   u16_Debug_Log_Prev_Data = u16_data;

   if (p_u16_Debug_Log_Flash <= &Flash[DEBUG_LOG_BLOCK_ADDRESS + 24000])
   {
      FlashWriteWord(u16_data, ((long)p_u16_Debug_Log_Flash - 0x00100000L), 0);   // Write the stamp
      (unsigned long)p_u16_Debug_Log_Flash++;
   }
}


//**********************************************************
// Function Name: SalClearDebugLog
// Description: Clear the debug log. Erase debug log and write stamp.
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SalClearDebugLog(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ( (param != 1LL) && (param != 555LL) ) return VALUE_OUT_OF_RANGE;
// uncomment only when using the debug log feature   u16_Get_Debug_Log_Flag = (unsigned int)param;
// uncomment only when using the debug log feature   if (u16_Get_Debug_Log_Flag == 1) return SAL_SUCCESS;
// uncomment only when using the debug log feature
// uncomment only when using the debug log feature   u16_Get_Debug_Log_Flag = 0;
// uncomment only when using the debug log feature
// uncomment only when using the debug log feature   if (EraseBlock(DEBUG_LOG_BLOCK) != FLASH_DONE) return SAL_NOT_FINISHED;
// uncomment only when using the debug log feature
// uncomment only when using the debug log feature   FlashWriteWord(1234, DEBUG_LOG_BLOCK_ADDRESS, 0);   // Write the stamp
// uncomment only when using the debug log feature   p_u16_Debug_Log_Flash = &Flash[DEBUG_LOG_BLOCK_ADDRESS];
// uncomment only when using the debug log feature   (unsigned long)p_u16_Debug_Log_Flash++;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: GetDebugLogCommand
// Description:
//   This function called in response to GETDEBUGLOG command
//
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int GetDebugLogCommand(void)
{
   static int get_debug_state = 0;
   int i;

   switch (get_debug_state)
   {
      case 0:
         //check for free space at output buffer
         if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;

         // Count number of logged data
         u16_Debug_Log_Count = 0;
         p_u16_Get_Debug_Log = &Flash[DEBUG_LOG_BLOCK_ADDRESS];
         while ((u16_Debug_Log_Count <= 24000) && (*p_u16_Get_Debug_Log != 0xFFFF))
         {
            (unsigned long)p_u16_Get_Debug_Log++;
            u16_Debug_Log_Count++;
         }

         u16_Debug_Log_Count_Div_6 = u16_Debug_Log_Count / 6;
         if ((u16_Debug_Log_Count_Div_6 * 6) < u16_Debug_Log_Count) u16_Debug_Log_Count_Div_6++;

         PrintDebugHeader();         // Print header
         u16_Debug_Log_Index = 0;
         u16_Debug_Log_Line_Index = 0;
         p_u16_Get_Debug_Log = &Flash[DEBUG_LOG_BLOCK_ADDRESS];

         get_debug_state++;         //go to next state
      break;

      case 1:
         //check for free space at output buffer
         if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;

         if (u16_Debug_Log_Line_Index >= (u16_Debug_Log_Count_Div_6 + 1))         //Check if all data have been printed (the '+1' is to send an extra line beacuse SST Excel omits the last line)
         {
            get_debug_state = 0;
            return SAL_SUCCESS;
         }

         for(i = 0; i < 5; i++)
         {
            PrintSignedInteger((int)p_u16_Get_Debug_Log[u16_Debug_Log_Index]);         //Print debug data
            PrintChar(COMMA);
            u16_Debug_Log_Index++;
         }
         PrintSignedInteger((int)p_u16_Get_Debug_Log[u16_Debug_Log_Index]);         //Print debug data
         u16_Debug_Log_Index++;
         PrintCrLf();

         u16_Debug_Log_Line_Index++;

      break;

   }

   return   SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: PrintDebugHeader
// Description:
//   This function prints debug log header
//
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
void PrintDebugHeader(void)
{
   PrintString("CDHD Recording", 0);   //Print recording header
   PrintCrLf();

   PrintUnsignedInteger(u16_Debug_Log_Count_Div_6);   // Print recording length and gap
   PrintChar(COMMA);
   PrintUnsignedLong(1UL);
   PrintCrLf();
   PrintChar(DOUBLE_QUOTE);
   PrintString("Debug1", 0);
   PrintChar(DOUBLE_QUOTE);
   PrintChar(COMMA);
   PrintChar(DOUBLE_QUOTE);
   PrintString("Debug2", 0);
   PrintChar(DOUBLE_QUOTE);
   PrintChar(COMMA);
   PrintChar(DOUBLE_QUOTE);
   PrintString("Debug3", 0);
   PrintChar(DOUBLE_QUOTE);
   PrintChar(COMMA);
   PrintChar(DOUBLE_QUOTE);
   PrintString("Debug4", 0);
   PrintChar(DOUBLE_QUOTE);
   PrintChar(COMMA);
   PrintChar(DOUBLE_QUOTE);
   PrintString("Debug5", 0);
   PrintChar(DOUBLE_QUOTE);
   PrintChar(COMMA);
   PrintChar(DOUBLE_QUOTE);
   PrintString("Debug6", 0);
   PrintChar(DOUBLE_QUOTE);
   PrintCrLf();
}
