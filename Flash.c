
//###########################################################################
//
// FILE:    FlashSST.c
//
// TITLE:   Flash routines
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.02| 16 Jan 2011 | A.H. | Modifications, for SST39VF1601 Device
//
//##################################################################

#include "DSP2834x_Device.h"

#include "Flash.def"
#include "ModCntrl.def"

#include "Flash.var"
#include "extrn_asm.var"
#include "FltCntrl.var"

#include "Prototypes.pro"


//**********************************************************
// Function Name: InitFlash
// Description: Initialize the flash driver
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void InitFlash()
{
   // TODO read FLASH ID

   u16_Flash_Erase_Suspended = 0;
   u16_Flash_Erase_Timeout = 10L;
   s16_Flash_Fault_Flag = 0;
   s16_Flash_Erase_Semaphore = SEMAPHORE_FREE;
}

#pragma CODE_SECTION(EnableFlashWrite, "ramfunc_5");
void EnableFlashWrite(int en)
{
 en+=0;
/*
* The drive firmware is not supposed to erase or to write to the boot area (lower 8 or 32 KW).
*
*   int s16_delay;
*   if (en)
*   {  //  flash write en
*      GpioDataRegs.GPBDAT.bit.GPIO52 = 1;
*      for (s16_delay = 0; s16_delay < 100; s16_delay++);
*   }
*   else
*   {  // TODO flash write dis
*      GpioDataRegs.GPBDAT.bit.GPIO52 = 0;
*   }
*/
}


//**********************************************************
// Function Name: EraseSector
// Description: Erases the flash sector
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(EraseSector, "ramfunc_5");
int EraseSector(int sector)
{
   static int erase_state = 0;
   int flash_staus = FLASH_PENDING;
   long timer, s32_index, sector_start = FLASH_SECTOR_SIZE * sector;

   //check that erasing of other sector is not in process
   if ((s16_Flash_Erase_Semaphore != SEMAPHORE_FREE) & (s16_Flash_Erase_Semaphore != sector))
      return FLASH_PENDING;

   PieCtrlRegs.PIEIER10.bit.INTx8 = 0;    // disable BG ISR (executed from flash) while erasing flash sector

   switch (erase_state)
   {
      case 0:
         if ((sector < 0) || (sector >= s16_Num_Of__2KW_Sectors))
         {
            PieCtrlRegs.PIEIER10.bit.INTx8 = 1;   // re-enable BG ISR
            return FLASH_FAIL;
         }

         EnableFlashWrite(1);
         s16_Flash_Erase_Semaphore = sector; //take semaphore

         Flash[0x5555] = 0xAA;
         Flash[0x2AAA] = 0x55;
         Flash[0x5555] = 0x80;
         Flash[0x5555] = 0xAA;
         Flash[0x2AAA] = 0x55;
         Flash[sector_start] = u16_Erase__2KW_Code;
         erase_state++;

         timer = Cntr_1mS; // Wait until Bit-7 complements (goes to zero)
         // to ensure that Erasing actually started...
         while ( ((Flash[sector_start] & BIT_7_MASK) != 0) && (PassedTimeMS(1L, timer) == 0) );

      case 1: // Erase-Suspend is used to "slice" the Erase operation into
      // manageable time intervals that will not impede other Background Tasks
         if (u16_Flash_Erase_Suspended == 1) // if erase was suspended earlier
         {
            Flash[sector_start] = 0x30; // resume erase for an other time-slice
            u16_Flash_Erase_Suspended = 0;
         }
         timer=Cntr_1mS;

         while ( (flash_staus == FLASH_PENDING)                     &&
                 (PassedTimeMS(u16_Flash_Erase_Timeout, timer) == 0)  )
         { // Wait for timeout or for DONE or for FAIL
            flash_staus = FlashDataPoll(sector_start);   // returns FLASH_DONE or FLASH_PENDING
         }
         if (flash_staus == FLASH_PENDING) // erase not finished yet
         {
            Flash[sector_start] = 0xB0; // suspend erase to allow other BG Tasks
            DSP28x_usDelay(2000); // 20usec delay
            u16_Flash_Erase_Suspended = 1;
            PieCtrlRegs.PIEIER10.bit.INTx8 = 1;   // re-enable BG ISR
            return FLASH_PENDING;
         }
         // erase completed
         EnableFlashWrite(0);
         flash_staus = FLASH_PENDING;         // Continue to erase verification
         erase_state = 2;
      break;

      case 2: // Verify that all sector is erased (all 0xFFFF)

         u32_Erase_Sum = 0;
         for (s32_index = sector_start; s32_index < sector_start + 0x800; s32_index++)
         {
            u32_Erase_Sum += (unsigned long)Flash[s32_index];
         }

         if (u32_Erase_Sum == 0x07FFF800)   // 0x07FFF800 = 0x800 * 0xFFFF
         {
            s16_Erase_Success_Counter++;         // count the accumulated number of success erase
            s16_Erase_Failure_Retry_Counter = 0;
            s16_Flash_Erase_Semaphore = SEMAPHORE_FREE;
            flash_staus = FLASH_DONE;
         }
         else
         {
            s16_Erase_Failure_Counter++;         // count the accumulated number of failed erase
            flash_staus = FLASH_PENDING;         // Retry from the beginning
            s16_Erase_Failure_Retry_Counter++;
            if (s16_Erase_Failure_Retry_Counter >= 3)
            {
               s16_Erase_Failure_Retry_Counter = 0;
               s16_Flash_Erase_Semaphore = SEMAPHORE_FREE;
               flash_staus = FLASH_DONE;
            }
         }

         erase_state = 0;
      break;
   }

   PieCtrlRegs.PIEIER10.bit.INTx8 = 1;   // re-enable BG ISR

   return flash_staus;
}


//**********************************************************
// Function Name: EraseBlock
// Description: Erases the flash block
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(EraseBlock, "ramfunc_3");
int EraseBlock(int block)
{
   static int erase_state = 0;
   int flash_staus = FLASH_PENDING;
   long timer, s32_index, block_addr;

   if (block >= 0)
   {
      block_addr = block * FLASH_BLOCK_SIZE;
   }
   else
   {
      // support erase 4KW blocks at the bottom of the flash (64Mb flash)
      block_addr = (-block) * FLASH_BOTTOM_BLOCK_SIZE;
   }


   //check that erasing of other sector is not in process
   if ((s16_Flash_Erase_Semaphore != SEMAPHORE_FREE) & (s16_Flash_Erase_Semaphore != block))
      return FLASH_PENDING;

   PieCtrlRegs.PIEIER10.bit.INTx8 = 0;    // disable BG ISR (executed from flash) while erasing flash sector

   switch (erase_state)
   {
      case 0:
         if ((block < -7) || (block >= s16_Num_Of_32KW_Blocks))
         {
            PieCtrlRegs.PIEIER10.bit.INTx8 = 1;   // re-enable BG ISR
            return FLASH_FAIL;
         }

         EnableFlashWrite(1);

         Flash[0x5555] = 0xAA;
         Flash[0x2AAA] = 0x55;
         Flash[0x5555] = 0x80;
         Flash[0x5555] = 0xAA;
         Flash[0x2AAA] = 0x55;
         Flash[block_addr] = u16_Erase_32KW_Code;

         timer = Cntr_1mS; // Wait until Bit-7 complements (goes to zero)
         // to ensure that Erasing actually started...
         while ( ((Flash[block_addr] & BIT_7_MASK) != 0) && (PassedTimeMS(1L, timer) == 0) );

         s16_Flash_Erase_Semaphore = block;
         erase_state++;

      case 1: // Erase-Suspend is used to "slice" the Erase operation into
      // manageable time intervals that will not impede other Background Tasks
         if (u16_Flash_Erase_Suspended == 1) // if erase was suspended earlier
         {
            Flash[block_addr] = 0x30; // resume erase for an other time-slice
            u16_Flash_Erase_Suspended = 0;
         }
         timer=Cntr_1mS;

         while ( (flash_staus == FLASH_PENDING)                     &&
                 (PassedTimeMS(u16_Flash_Erase_Timeout, timer) == 0)  )
         { // Wait for timeout or for DONE or for FAIL
            flash_staus = FlashDataPoll(block_addr);   // returns FLASH_DONE or FLASH_PENDING
         }
         if (flash_staus == FLASH_PENDING) // erase not finished yet
         {
            Flash[block_addr] = 0xB0; // suspend erase to allow other BG Tasks
            DSP28x_usDelay(2000); //20usec delay
            u16_Flash_Erase_Suspended = 1;
            PieCtrlRegs.PIEIER10.bit.INTx8 = 1;   // re-enable BG ISR
            return FLASH_PENDING;
         }
         // erase completed
         EnableFlashWrite(0);
         flash_staus = FLASH_PENDING;         // Continue to erase verification
         erase_state = 2;
      break;

      case 2: // Verify that all sector is erased (all 0xFFFF)

         u32_Erase_Sum = 0;
         for (s32_index = block_addr; s32_index < block_addr + (unsigned long)0x8000; s32_index++)
         {
            u32_Erase_Sum += (unsigned long)Flash[s32_index];
         }

         if (u32_Erase_Sum == 0x7FFF8000)   // 0x7FFF8000 = 0x8000 * 0xFFFF
         {
            s16_Erase_Success_Counter++;         // count the accumulated number of success erase
            s16_Erase_Failure_Retry_Counter = 0;
            s16_Flash_Erase_Semaphore = SEMAPHORE_FREE;
            flash_staus = FLASH_DONE;
         }
         else
         {
            s16_Erase_Failure_Counter++;         // count the accumulated number of failed erase
            flash_staus = FLASH_PENDING;         // Retry from the beginning
            s16_Erase_Failure_Retry_Counter++;
            if (s16_Erase_Failure_Retry_Counter >= 3)
            {
               s16_Erase_Failure_Retry_Counter = 0;
               s16_Flash_Erase_Semaphore = SEMAPHORE_FREE;
               flash_staus = FLASH_DONE;
            }
         }

         erase_state = 0;
      break;
   }

   PieCtrlRegs.PIEIER10.bit.INTx8 = 1;   // re-enable BG ISR

   return flash_staus;
}


//**********************************************************
// Function Name: FlasWriteWord
// Description: Writes word to flash
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FlashWriteWord, "ramfunc_3");
int FlashWriteWord(unsigned int wr_value, long sector_start_addr, int offset)
{
   int flash_status;
   long timer;

   PieCtrlRegs.PIEIER10.bit.INTx8 = 0;    // disable BG ISR (executed from flash) while erasing flash sector

   if (((sector_start_addr + offset) >= 0x100000) && ((sector_start_addr + offset) <= 0x107FFF))
      EnableFlashWrite(1); // Operate Write-Enable only if writing to Write-Protected Sector
   Flash[0x5555] = 0xAA;
   Flash[0x2AAA] = 0x55;
   Flash[0x5555] = 0xA0;
   Flash[sector_start_addr + offset] = wr_value;
   timer = Cntr_1mS; // Wait until Bit-7 complements to ensure that Writing actually started...
   while ( ((wr_value & BIT_7_MASK) == (Flash[sector_start_addr + offset] & BIT_7_MASK)) &&
           (PassedTimeMS(1L, timer) == 0)                                                  );
   timer = Cntr_1mS;
   do
   {
      flash_status = FlashDataPoll(sector_start_addr + offset);
   }
   while ((flash_status == FLASH_PENDING) && (PassedTimeMS(10L, timer) == 0));

   if (Flash[sector_start_addr + offset] != wr_value )   // verify written value
   {
      s16_Flash_Fault_Flag = 1;
      flash_status = FLASH_FAIL;
   }

   EnableFlashWrite(0);

   PieCtrlRegs.PIEIER10.bit.INTx8 = 1;   // re-enable BG ISR

   return flash_status;
}


//**********************************************************
// Function Name: WriteFlash
// Description:
//   This function writes "len" words to FLASH
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(WriteFlash, "ramfunc_3");
int WriteFlash(long sector_start_addr, unsigned int addr, unsigned int* data_ptr, int len)
{
   int i;
   int result=0;

   for (i = 0; i < len; i++)
   {
      u16_Flash_Params_Checksum += (int)*data_ptr;   // Update Checksum
      result = FlashWriteWord(*data_ptr, sector_start_addr, addr + i);
      if (result == FLASH_FAIL) return result;
      data_ptr++;
   }
   return result;
}


//**********************************************************
// Function Name: FlashDataPoll
// Description: Flash data poll algoritm, see figure 21 in SST39VF1601 DataSheet
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FlashDataPoll, "ramfunc_3");
int FlashDataPoll(long addr)
{
   volatile int s16_read_val_old, s16_read_val_new;

   s16_read_val_old = Flash[addr] & FLASH_POLL_MASK;
   asm(" NOP");
   asm(" NOP");
   s16_read_val_new = Flash[addr] & FLASH_POLL_MASK;

   if (s16_read_val_old == s16_read_val_new) return FLASH_DONE;
   else return FLASH_PENDING;
}


//**********************************************************
// Function Name: ReadProductID
// Description: Read the Flash Product ID from the Device
//
// Author: A.H.
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(ReadProductID, "ramfunc_5");
void ReadProductID(unsigned int u16_index)
{
   unsigned int u16_delay;

   if (u16_index == 1)
   {
      SetFlashPage(2);
   }

   Flash[0x5555] = 0xAA; // Entering Product ID Reading Mode
   Flash[0x2AAA] = 0x55;
   Flash[0x5555] = 0x90;

   for(u16_delay=0 ; u16_delay<30 ; u16_delay++) // MICROCHIP requires at least 150 ns entry time
   {
      u16_delay++;
      u16_delay++;
      u16_delay -= 2;
   }

   u16_Vendor_ID[u16_index] = Flash[0];
   if ((u16_Vendor_ID[u16_index] & 0x00FF) == 0x0001)  // Spansion provides Manufacturer ID on bits 7-0 while bits 15-8 are don't care.
   {                                                   // SST provides Manufacturer ID on bits 15-0
      u16_Vendor_ID[u16_index] = u16_Vendor_ID[u16_index] & 0x00FF;
   }

   u16_Device_ID[u16_index] = Flash[1];

   if ((u16_Vendor_ID[u16_index] == 0x00BF) && (u16_Device_ID[u16_index] == 0x227E))   // MICROCHIP - check further information to identify the exact device
   {
      u16_Device_ID_Subinfo_1[u16_index] = Flash[0x0e];
      u16_Device_ID_Subinfo_2[u16_index] = Flash[0x0f];
      // Subinfo_1   Subinfo_2   Device ID
      // --------    --------    ------------
      //  0x220C      0x2200     SST38VF6401B
      //  0x220C      0x2201     SST38VF6402B
      //  0x2210      0x2200     SST38VF6403B - this device should be used
      //  0x2210      0x2201     SST38VF6404B
   }
   else
   {
      // Skip this two lines for ((u16_Vendor_ID[u16_index] == 0x00BF) && (u16_Device_ID[u16_index] == 0x227E))
      // otherwise the flash remain in Product ID Reading Mode, and can not be erased or programmed.
   Flash[0x5555] = 0xAA; // ID Reading Mode Exit
   Flash[0x2AAA] = 0x55;
   }

   Flash[0x5555] = 0xF0;

   for(u16_delay=0 ; u16_delay<30 ; u16_delay++) // MICROCHIP requires at least 150 ns exit time
   {
      u16_delay++;
      u16_delay++;
      u16_delay -= 2;
   }

   if (u16_index == 1)
   {
      SetFlashPage(0);
   }

   return;
}

