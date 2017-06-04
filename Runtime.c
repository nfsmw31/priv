//###########################################################################
//
// FILE:    Runtime.c
//
// TITLE:   Run Time counter routines
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.01| 01 DEC 2010| D.R. | Creation
//
//##################################################################

#include "Flash.def"
#include "ModCntrl.def"
#include "Err_Hndl.def"

#include "Flash.var"
#include "Init.var"
#include "extrn_asm.var"
#include "Runtime.var"

#include "Prototypes.pro"


//**********************************************************
// Function Name: InitRuntimeCounter
// Description: Initilizes Run Time counter.
//
// Runtime counter uses entire flash sector. It starts when sector erased (all bits are 1)
// On every second the firmware updated run time counter and set next bit to 0
// When all seconds bits are 0, the firmware erase sector (all bits 1 again) and zero next day bit.
// Bits from sector address 0 to 32399 are used to store seconds: number of 0 bits = number of seconds
// Bits from sector addres 32400 to 32767 used to store days 1bit = 6days: number of 0 bits = day*6
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void InitRuntimeCounter(void)
{
   int drive = 0;
   unsigned long i = 0L;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // init pointer to run-time flash sector
   p_u16_Runtime_Flash = &Flash[RUN_TIME_COUNTER_BLOCK_ADDRESS];

   //init state mashine
   u16_Run_Time_State = 0;
   s32_RTHandlerCntr = 0;
   BGVAR(u32_Run_Time_Counter) = 0;

   //find number of passed seconds
   while ((p_u16_Runtime_Flash[i] != 0xFFFF) && (i < FLASH_RUN_TIME_SECONDS_SIZE))
   {
      i++;
   }

   if (i > 0)
   {
      BGVAR(u32_Run_Time_Counter) = (i - 1) * 16;
      BGVAR(u32_Run_Time_Counter) += NumberOfZeroBits(p_u16_Runtime_Flash[i - 1]);
   }

   //find number of passed days , days bits start after seconds bits
   i = p_u16_Runtime_Flash[FLASH_RUN_TIME_DAYS_START];
   if (i == 0xFFFF) i = 0; //empty
   //calculate seconds from days. every zero bit is 6 days = 6days *24hours *60min *60sec = 518400
   BGVAR(u32_Run_Time_Counter) += i * 518400;

   UpdateBgTime();
}

//**********************************************************
// Function Name: UpdateBgTime
// Description: Updates baground time variables.
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void UpdateBgTime()
{
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   u16_RunTimeSecs = BGVAR(u32_Run_Time_Counter) % 60;
   u16_RunTimeMins = (BGVAR(u32_Run_Time_Counter) / 60 ) % 60;
   u16_RunTimeHours = BGVAR(u32_Run_Time_Counter)  / 3600;
}

//**********************************************************
// Function Name: NumberOfZeroBits
// Description: calculate number of zeros in 16bit integer
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int NumberOfZeroBits(unsigned int value)
{
   int i = 0;
   while (value != 0)
   {
      value >>= 1;
      i++;
   }
   return 16 - i;
}

//**********************************************************
// Function Name: UpdateDaysCounter
// Description: updates 6-days counter in flash
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int UpdateDaysCounter()
{
   int days;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (EraseBlock(RUN_TIME_COUNTER_BLOCK) != FLASH_DONE) return SAL_NOT_FINISHED;

   u16_Run_Time_Days_Ptr = FLASH_RUN_TIME_DAYS_START;
    days = (BGVAR(u32_Run_Time_Counter) + 1) / FLASH_RUN_TIME_SECONDS ;

   FlashWriteWord(days, RUN_TIME_COUNTER_BLOCK_ADDRESS, FLASH_RUN_TIME_DAYS_START);
   if (s16_Flash_Fault_Flag == 1)
   {
      //try again
      s16_Run_Time_Flash_Fault_Acc_Counter++;         // count the accumulated number of run time failures
      s16_Flash_Fault_Flag = 0;
      FlashWriteWord(days, RUN_TIME_COUNTER_BLOCK_ADDRESS, FLASH_RUN_TIME_DAYS_START);
      if (s16_Flash_Fault_Flag == 1)
      {
         s16_Run_Time_Flash_Fault_Acc_Counter++;      // count the accumulated number of run time failures
         s16_Flash_Fault_Flag = 0;                    // defeat the fault
      }
   }

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: UpdateRuntimeCounter
// Description: updates seconds counter in flash
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int UpdateRuntimeCounter()
{
   unsigned int temp;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //check if all seconds bits are used
   if (((BGVAR(u32_Run_Time_Counter)+1) % FLASH_RUN_TIME_SECONDS) == 0)
   {
      if (UpdateDaysCounter() != SAL_SUCCESS) return SAL_NOT_FINISHED;
   }
   else
   {
      u16_Run_Time_Seconds_Ptr = (BGVAR(u32_Run_Time_Counter) % FLASH_RUN_TIME_SECONDS) >> 4;
      // extract secs pointer
      temp = (p_u16_Runtime_Flash[u16_Run_Time_Seconds_Ptr] & 0x0FFFFL) >> 1;
      FlashWriteWord(temp, RUN_TIME_COUNTER_BLOCK_ADDRESS, u16_Run_Time_Seconds_Ptr);
      if (s16_Flash_Fault_Flag == 1)
      {
         // try again
         s16_Run_Time_Flash_Fault_Acc_Counter++;         // count the accumulated number of run time failures
         s16_Flash_Fault_Flag = 0;
         FlashWriteWord(temp, RUN_TIME_COUNTER_BLOCK_ADDRESS, u16_Run_Time_Seconds_Ptr);
         if (s16_Flash_Fault_Flag == 1)
         {
            s16_Run_Time_Flash_Fault_Acc_Counter++;      // count the accumulated number of run time failures
            s16_Flash_Fault_Flag = 0;                    // defeat the fault
         }
      }

   }

   BGVAR(u32_Run_Time_Counter)++; //update counter in ram

   UpdateBgTime(); //update secs/mins/hours counters

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: RunTimeHandler
// Description:
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void RunTimeHandler(void)
{
   switch (u16_Run_Time_State)
   {
      case 0:
         if (PassedTimeMS(1000L, s32_RTHandlerCntr))
            {
            s32_RTHandlerCntr = Cntr_1mS;
            u16_Run_Time_State++;
         }
         else return;
      case 1:
         if (UpdateRuntimeCounter() == SAL_SUCCESS) u16_Run_Time_State = 0;
      break;
   }

   return;
}

//**********************************************************
// Function Name: TrunCommand
// Description:
//   This function called in response to TRUN command
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int TrunCommand(void)
{
   PrintUnsignedInteger(u16_RunTimeHours);
   PrintChar(':');
   if (u16_RunTimeMins < 10) PrintChar('0');
   PrintUnsignedInteger(u16_RunTimeMins);
   PrintChar(':');
   if (u16_RunTimeSecs < 10) PrintChar('0');
   PrintUnsignedInteger(u16_RunTimeSecs);
   PrintCrLf();

   return SAL_SUCCESS;
}

