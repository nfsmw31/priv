//###########################################################################
//
// FILE: Record.c
//
// TITLE:   Record routines
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.01| 31 Jul 2002 | D.R. | Creation
//
//##################################################################

#include "DSP2834x_Device.h"
#include "Modbus_Comm.pro"


#include "MultiAxis.def"
#include "AutoTune.def"
#include "Record.def"
#include "Design.def"
#include "Err_Hndl.def"
#include "Ser_Comm.def"
#include "ExFbVar.def"
#include "AutoTune.var"
#include "Ser_Comm.var"
#include "Extrn_Asm.var"
#include "Drive.var"
#include "User_Var.var"
#include "Drive_Table.var"
#include "Units.var"
#include "Record.var"
#include "Exe_IO.def"
#include "ExFbVar.var"
#include "FlashHandle.var"
#include "objects.h"
#include "Prototypes.pro"
#include "init.var"


extern unsigned int u16_Output[];


//**********************************************************
// Macro  Name: IncRecPtr
// Description:
//   Increments cyclic pointer to record buffer
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
#define IncRecPtr()\
{\
   s16_Debug_Ram_Ptr++;\
   if (s16_Debug_Ram_Ptr == u16_Ram_Rec_Length_Avaliable)\
      s16_Debug_Ram_Ptr = 0;\
}

//**********************************************************
// Macro  Name: IncRtRecPtr
// Description:
//   Increments cyclic pointer to record buffer
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
#define IncRtRecPtr()\
{\
   s16_Debug_Ram_Ptr++;\
   if (s16_Debug_Ram_Ptr == u16_Ram_Rec_Length_Word_Avaliable)\
      s16_Debug_Ram_Ptr = 0;\
}


//**********************************************************
// Function Name: RtRecord
// Description:
//   This function performes RT recording, called from MTS_ISR.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(RtRecord, "ramfunc");
void RtRecord(void)
{
   int i;

   if (s16_Record_Flags & RT_UPDATE_DATA_PTR)
   {
      s16_Record_Flags &= ~RT_UPDATE_DATA_PTR;
      //set pointer to next packet
      u16_Data_Packet_Ptr = u16_Data_Packet_Ptr + s16_Recording_Length + DATA_HEADER_LEN;
      if (u16_Data_Packet_Ptr >= u16_Ram_Rec_Length_Word_Avaliable) u16_Data_Packet_Ptr -= u16_Ram_Rec_Length_Word_Avaliable;
   }

   if ((s16_Record_Flags & REC_TRIGGERED_MASK) == 0) return;   //check if recording triggered

   if (--s32_Gap_Cntr) return;   // Check for samples gap. do it only if recgap != 0

   s32_Gap_Cntr = s32_Gap_Value;   // Start Gap_Counter

   if (s16_Debug_Ram_Ptr == u16_Record_End) //Finished to record data packet
   {
      //Fill data packet header
      p_s16_Debug_Ram[u16_Record_Start] = STAMP_DATA_PACKET_TYPE;
      p_s16_Debug_Ram[u16_Record_Start + 1] = u16_Packet_Counter;
      p_s16_Debug_Ram[u16_Record_Start + 2] = u16_Data_Packet_Len;

      u16_Packet_Counter++; //increase packet counter
      u16_Record_Start = s16_Debug_Ram_Ptr; // Set the recording start address for next packet
      p_s16_Debug_Ram[u16_Record_Start] = 0xcccc; //clear stamp till packet not filled with data
      s16_Debug_Ram_Ptr += DATA_HEADER_LEN;
      if (s16_Debug_Ram_Ptr >= u16_Ram_Rec_Length_Word_Avaliable) s16_Debug_Ram_Ptr -= u16_Ram_Rec_Length_Word_Avaliable;

      // Set the recording end address for next packet, according to the recording length
      u16_Record_End = s16_Debug_Ram_Ptr + s16_Recording_Length;
      // Check if this value is past the end of the buffer. If so, wrap it around (cyclic buffer).
      if (u16_Record_End >= u16_Ram_Rec_Length_Word_Avaliable) u16_Record_End -= u16_Ram_Rec_Length_Word_Avaliable;
   }

   //Copy Data to buffer
   for (i = 0; i < s16_Record_Channel_Number; i++)
   {
      if (s16_Debug_Ram_Ptr == u16_Data_Packet_Ptr)  // buffer overrun !!!
      {
         s16_Record_Flags |= RT_RECORD_BUFFER_OVERRUN; //set error bit
         u16_Data_Packet_Ptr = u16_Data_Packet_Ptr + s16_Recording_Length + DATA_HEADER_LEN;
         if (u16_Data_Packet_Ptr >= u16_Ram_Rec_Length_Word_Avaliable)
            u16_Data_Packet_Ptr -= u16_Ram_Rec_Length_Word_Avaliable;
      }

      p_s16_Debug_Ram[s16_Debug_Ram_Ptr] = *p_s16_Rec_Var_Ptr[i];
      //Increment Recording pointer
      IncRtRecPtr();
   }
}


#pragma CODE_SECTION(Record, "ramfunc_6");
void Record(void)
{
   int i;

   if ( !(s16_Record_Flags & RECORD_ON_MASK) ) return;   // Check if record active

   if (s16_Record_Flags & RT_RECORD_ON_MASK)
   {
       RtRecord();
       return;
   }

   if (--s32_Gap_Cntr) return;   // Check for samples gap. do it only if recgap != 0

   s32_Gap_Cntr = s32_Gap_Value;   // Start Gap_Counter

   if (s16_Record_Flags & REC_TRIGGERED_MASK)   //check if trigger armed
   {
      if (s16_Debug_Ram_Ptr == u16_Record_End) //If recording finished stop recording
      {
         s16_Record_Flags &= ~RECORD_ON_MASK;
         s16_Record_Flags |= DATA_AVAILABLE_MASK;
         s16_Record_Flags &= ~REC_TRIGGERED_MASK;
         s16_Record_Flags &= ~WAITING_FOR_TRIGGER_MASK;
         //Disabling the Trigger notification
         AX0_u16_PRB_Mode &=~0x04;
         return;
      }
   }

   for (i = 0; i < s16_Record_Channel_Number; i++)   // Copy Data to buffer
      s16_Debug_Ram[i][s16_Debug_Ram_Ptr] = *p_s16_Rec_Var_Ptr[i];

   IncRecPtr();   //Increment Recording pointer
}


//**********************************************************
// Function Name: IsS64ALTB
// Description:
//   Service function that returns
//   1 - if s64_A < s64_B (both vars are handled as signed values)
//   0 - otherwise
//   This to avoid "if(s64_A < s64_B)" which result in calling lib function that reside in flash
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(IsS64ALTB, "ramfunc_2");
int IsS64ALTB(long long s64_A, long long s64_B)
{
   long long s64_diff;

   if ((s64_A >= 0) && (s64_B < 0))
      return 0;             // s64_A >= 0, s64_B < 0 ==> s64_A > s64_B
   else if ((s64_A < 0) && (s64_B >= 0))
      return  1;            // s64_A < 0, s64_B >= 0 ==> s64_A < s64_B
   else
   {
      s64_diff = s64_A - s64_B;
      if (s64_diff < 0)
         return 1;          // s64_A - s64_B < 0  ==> s64_A < s64_B
      else
         return 0;          // s64_A - s64_B >= 0  ==> s64_A >= s64_B
   }
}

//**********************************************************
// Function Name: IsS64ALEQB
// Description:
//   Service function that returns
//   1 - if s64_A <= s64_B (both vars are handled as signed values)
//   0 - otherwise
//   This to avoid "if(s64_A <= s64_B)" which result in calling lib function that reside in flash
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(IsS64ALEQB, "ramfunc_2");
int IsS64ALEQB(long long s64_A, long long s64_B)
{
   long long s64_diff;

   if ((s64_A >= 0) && (s64_B < 0))
      return 0;             // s64_A >= 0, s64_B < 0 ==> s64_A > s64_B
   else if ((s64_A < 0) && (s64_B >= 0))
      return 1;             // s64_A < 0, s64_B >= 0 ==> s64_A < s64_B
   else
   {
      s64_diff = s64_A - s64_B;
      if (s64_diff <= 0)
         return 1;          // s64_A - s64_B <= 0  ==> s64_A <= s64_B
      else
         return 0;          // s64_A - s64_B > 0   ==> s64_A > s64_B
   }
}

//**********************************************************
// Function Name: IsU64ALTB
// Description:
//   Service function that returns
//   1 - if s64_A < s64_B (both vars are handled as unsigned values,
//       despite the data type in the function prototype)
//   0 - otherwise
//   This to avoid "if(s64_A < s64_B)" which result in calling lib function that reside in flash
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(IsU64ALTB, "ramfunc_2");
int IsU64ALTB(long long s64_A, long long s64_B)
{
   long long s64_diff;

   if ((s64_A >= 0) && (s64_B < 0))
      return 1;             // s64_A >= 0, s64_B >> 0 ==> s64_A < s64_B
   else if ((s64_A < 0) && (s64_B >= 0))
      return 0;             // s64_A >> 0, s64_B >= 0 ==> s64_A > s64_B
   else
   {
      s64_diff = s64_A - s64_B;
      if (s64_diff < 0)
         return 1;          // s64_A - s64_B < 0  ==> s64_A < s64_B
      else
         return 0;          // s64_A - s64_B >= 0  ==> s64_A >= s64_B
   }
}

//**********************************************************
// Function Name: IsU64ALEQB
// Description:
//   Service function that returns
//   1 - if s64_A <= s64_B (both vars are handled as unsigned values,
//       despite the data type in the function prototype)
//   0 - otherwise
//   This to avoid "if(s64_A <= s64_B)" which result in calling lib function that reside in flash
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(IsU64ALEQB, "ramfunc_2");
int IsU64ALEQB(long long s64_A, long long s64_B)
{
   long long s64_diff;

   if ((s64_A >= 0) && (s64_B < 0))
      return 1;             // s64_A >= 0, s64_B >> 0 ==> s64_A < s64_B
   else if ((s64_A < 0) && (s64_B >= 0))
      return 0;             // s64_A >> 0, s64_B >= 0 ==> s64_A > s64_B
   else
   {
      s64_diff = s64_A - s64_B;
      if (s64_diff <= 0)
         return 1;          // s64_A - s64_B <= 0  ==> s64_A <= s64_B
      else
         return 0;          // s64_A - s64_B > 0   ==> s64_A > s64_B
   }
}

//**********************************************************
// Function Name: ArmRecTrigger
// Description:
//   This function check recordind triger condition, called from MTS_ISR
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(ArmRecTrigger, "ramfunc_6");
void ArmRecTrigger()
{
   long long s64_abs_trig_val, s64_abs_trig_level;
   unsigned int u16_mask;

   if ( !(s16_Record_Flags & RECORD_ON_MASK) ) return;           // Check if record active

   if ( !(s16_Record_Flags & REC_TRIGGER_DEFINED_MASK) ) return; // Check if trigger defined

   // trigger already activated , recording in progress
   if (s16_Record_Flags & REC_TRIGGERED_MASK) return;

   // Signal PRB that Trigger occured
   if (AX0_u16_PRB_Mode == 1)
   {
      AX0_u16_PRB_Mode |=4;
      AX0_u16_PRB_Counter = AX0_u16_PRB_Counter_Period-1;
      if ((AX0_u16_PRB_Type == 0) || (AX0_u16_PRB_Type == 1))  AX0_u16_LFSR = 1;
   }

   if (u16_Rec_Trig_Width == REC_TRIG_16)
   {
      s64_Rec_Trig_Val = (long long)*(int*)p_Rec_Trig_Val;
      if (!u16_Rec_Trig_Sign) s64_Rec_Trig_Val &= 0x000FFFFLL;//clear sign bits
   }
   else if (u16_Rec_Trig_Width == REC_TRIG_32)
   {
       s64_Rec_Trig_Val = (long long)*(long*)p_Rec_Trig_Val;
      if (!u16_Rec_Trig_Sign) s64_Rec_Trig_Val &= 0x000FFFFFFFFLL;//clear sign bits
   }
   else if (u16_Rec_Trig_Width == REC_TRIG_64)
      s64_Rec_Trig_Val = (long long)*(long long*)p_Rec_Trig_Val;

   // If absolute triggering is used calculate the abs values
   if (s16_Rec_Trig_Direction >= 2)
   {
      s64_abs_trig_val = s64_Rec_Trig_Val;
      if (s64_Rec_Trig_Val < 0LL) s64_abs_trig_val = -s64_Rec_Trig_Val;
      s64_abs_trig_level = s64_Rec_Trig_Level;
      if (s64_Rec_Trig_Level < 0LL) s64_abs_trig_level = -s64_Rec_Trig_Level;
   }

   // Set-up Trigger with at least one occurrence of "No-Trigger"
   if ( !(s16_Record_Flags & WAITING_FOR_TRIGGER_MASK))
   {
      u16_mask = 0;
      if (s16_Rec_Trig_Direction==POS_REC_TRIG)
      {
         if (u16_Rec_Trig_Sign)
         {
            if (IsS64ALEQB(s64_Rec_Trig_Val, s64_Rec_Trig_Level)) u16_mask = WAITING_FOR_TRIGGER_MASK; // Arm Trigger...
         }
         else
         {
            if (IsU64ALEQB(s64_Rec_Trig_Val, s64_Rec_Trig_Level)) u16_mask = WAITING_FOR_TRIGGER_MASK;
         }
      } 
      else if (s16_Rec_Trig_Direction==NEG_REC_TRIG)
      {
         if (u16_Rec_Trig_Sign)
         {
            if (IsS64ALEQB(s64_Rec_Trig_Level, s64_Rec_Trig_Val)) u16_mask = WAITING_FOR_TRIGGER_MASK;
         }
         else
         {
            if (IsU64ALEQB(s64_Rec_Trig_Level, s64_Rec_Trig_Val)) u16_mask = WAITING_FOR_TRIGGER_MASK;
         }
      }
      else if (s16_Rec_Trig_Direction==ABS_MORE_THAN_REC_TRIG)
      {
         if (IsU64ALEQB(s64_abs_trig_val, s64_abs_trig_level))
         {
            u16_mask = WAITING_FOR_TRIGGER_MASK;
         }
      }
      else if (s16_Rec_Trig_Direction==ABS_LESS_THAN_REC_TRIG)
      {
         if (IsU64ALEQB(s64_abs_trig_level, s64_abs_trig_val))
         {
            u16_mask = WAITING_FOR_TRIGGER_MASK;
         }
      }
      s16_Record_Flags |= u16_mask;
   }      
   // Check if trigger occured
   else if (s16_Record_Flags & WAITING_FOR_TRIGGER_MASK)
   {
      u16_mask = 0;
      if (s16_Rec_Trig_Direction==POS_REC_TRIG)
      {
         if (u16_Rec_Trig_Sign)
         {
            if (IsS64ALTB(s64_Rec_Trig_Level, s64_Rec_Trig_Val)) u16_mask = REC_TRIGGERED_MASK;
         }
         else
         {
            if (IsU64ALTB(s64_Rec_Trig_Level, s64_Rec_Trig_Val)) u16_mask = REC_TRIGGERED_MASK;
         }
      }
      else if (s16_Rec_Trig_Direction==NEG_REC_TRIG)
      {
         if (u16_Rec_Trig_Sign)
         {
            if (IsS64ALTB(s64_Rec_Trig_Val, s64_Rec_Trig_Level)) u16_mask = REC_TRIGGERED_MASK; 
         }
         else
         {
             if (IsU64ALTB(s64_Rec_Trig_Val, s64_Rec_Trig_Level)) u16_mask = REC_TRIGGERED_MASK;
         }
      }
      else if (s16_Rec_Trig_Direction==ABS_LESS_THAN_REC_TRIG)
      {
         if (IsU64ALTB(s64_abs_trig_val, s64_abs_trig_level)) u16_mask = REC_TRIGGERED_MASK;
      }
      else if (s16_Rec_Trig_Direction==ABS_MORE_THAN_REC_TRIG)
      {
         if (IsU64ALTB(s64_abs_trig_level, s64_abs_trig_val)) u16_mask = REC_TRIGGERED_MASK;
      }
      s16_Record_Flags |= u16_mask;
   }

   // Set the recording start address, according to the recording length
   if ( u16_Trig_Location <= s16_Debug_Ram_Ptr)
      u16_Record_Start = s16_Debug_Ram_Ptr - u16_Trig_Location;
   else
      u16_Record_Start = s16_Debug_Ram_Ptr - u16_Trig_Location + u16_Ram_Rec_Length_Avaliable;

   if (s16_Record_Flags & RT_RECORD_ON_MASK)
   {
      s16_Debug_Ram_Ptr = DATA_HEADER_LEN;
      u16_Record_Start = 0;
      u16_Data_Packet_Ptr = 0;  // set ptr to beginning of RT data

      p_s16_Debug_Ram[u16_Data_Packet_Ptr] = 0x0CCCC; //clear stamp (means that packet is not ready)
      // Set the recording end address, according to the recording length
      u16_Record_End = s16_Recording_Length + DATA_HEADER_LEN;
   }
   else
   {
      // Set the recording end address, according to the recording length
      u16_Record_End = u16_Record_Start + s16_Recording_Length;
      // Check if this value is past the end of the buffer. If so, wrap it around (cyclic buffer).
      if (u16_Record_End >= u16_Ram_Rec_Length_Avaliable)  u16_Record_End -= u16_Ram_Rec_Length_Avaliable;
   }
}


#pragma CODE_SECTION(RtRecordOutputProcessor, "ramfunc");
void RtRecordOutputProcessor(void)
{
   if ((s16_Record_Flags & PRINT_RT_RECORD)==0) return; //check if RT GET active

      while ((ScibRegs.SCIFFTX.bit.TXFFST < 0x0E) && (u16_Packet_Start_Ptr != u16_Packet_End_Ptr))
      {  //  Write data from output buffer to SCI-A
         ScibRegs.SCITXBUF =  p_s16_Debug_Ram[u16_Packet_Start_Ptr];      // LSB data
         ScibRegs.SCITXBUF =  p_s16_Debug_Ram[u16_Packet_Start_Ptr] >> 8; // MSB data

         u16_Packet_Check_Sum+=p_s16_Debug_Ram[u16_Packet_Start_Ptr]; // update checksum
         //  Increment output buffer cyclic pointer
         u16_Packet_Start_Ptr++;
         if (u16_Packet_Start_Ptr >= u16_Ram_Rec_Length_Word_Avaliable)
            u16_Packet_Start_Ptr -= u16_Ram_Rec_Length_Word_Avaliable ; // cyclic buffer
      }

      if ((u16_Packet_Start_Ptr == u16_Packet_End_Ptr) && (ScibRegs.SCIFFTX.bit.TXFFST < 0x0E))
      {
         ScibRegs.SCITXBUF =   0x10000 - u16_Packet_Check_Sum;       // LSB checksum
         ScibRegs.SCITXBUF =  (0x10000 - u16_Packet_Check_Sum) >> 8; // MSB checksum
         s16_Record_Flags &= ~PRINT_RT_RECORD;
         s16_Record_Flags |= RT_UPDATE_DATA_PTR;
      }
   }

//**********************************************************
// Function Name: SalGetModeCommand
// Description:
//   This function called in response to GETMODE command
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalGetModeCommand(long long lparam, int drive)
{
   drive+=0;//for REMARK avoid (can be deleted)

   if(lparam == 1 || lparam == 2) // getmode == 1 and 2 were valid for CD, but not in CDHD
       return VALUE_OUT_OF_RANGE;

   drive = 0;
   BGVAR(u16_Get_Mode) = (int)lparam; //one GETMODE for both axis
   drive = 1;
   BGVAR(u16_Get_Mode) = (int)lparam; //one GETMODE for both axis

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: RecordCommand
// Description:
//   This function called in response to RECORD command
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int RecordCommand(void)
{
   int i, ret_val = SAL_SUCCESS,drive = 0;;


   if (s16_Number_Of_Parameters < 3) return SYNTAX_ERROR;

   if ( (BGVAR(s16_Pos_Tune_State) > 0)              &&
        (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE) &&
        (s64_Execution_Parameter[1] == 0LL)             ) return (AT_ACTIVE);

   if ( (BGVAR(s16_Pos_Tune_State) > 0)                                  &&
        (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)                     &&
        ((BGVAR(s16_Pos_Tune_Av_State) == POS_TUNE_AV_FRQ2_IDENT) ||
         (BGVAR(s16_Pos_Tune_Av_State) == POS_TUNE_AV_FRQ3_IDENT)    )      ) return (AT_ACTIVE);

   if (s64_Execution_Parameter[1] >= u16_Ram_Rec_Length_Avaliable) return VALUE_TOO_HIGH;

   STORE_EXECUTION_PARAMS_0_15
   for (i = 0; i < NUMBER_OF_REC_CHANNELS; i++)   // Init channels variables
       s64_Execution_Parameter[i + 2] = 0;

   for (i = 2; i < s16_Number_Of_Parameters; i++)
   {   // Check for correct channel names
       s64_Execution_Parameter[i] = (long long)SearchRecordVariable(u8_Execution_String[i - 2]);
       if (s64_Execution_Parameter[i] == 0)
       {
         RESTORE_EXECUTION_PARAMS_0_15
         return VAR_NOT_RECORDABLE;
       }
   }

   ret_val = SalRecordCommand(u8_Axis_Id, s64_Execution_Parameter[0], s64_Execution_Parameter[1], &s64_Execution_Parameter[2]);

   RESTORE_EXECUTION_PARAMS_0_15
   ++drive; // avoid compiler comment
   return ret_val;
}

// This code runs in the RAM to shorten it execution time (down from 52msec to 3msec)
// #pragma CODE_SECTION(ClearRecordBuffer, "ramfunc_4");
#pragma CODE_SECTION(ClearRecordBuffer, "ramfunc_3");
void ClearRecordBuffer(void)
{
   int i;
   for (s16_Debug_Ram_Ptr = 0; s16_Debug_Ram_Ptr < u16_Ram_Rec_Length_Avaliable; s16_Debug_Ram_Ptr++)
   { // Zero the Record Buffer to avoid "dirt"
      for (i = 0; i < s16_Record_Channel_Number; i++)   // Copy Data to buffer
      s16_Debug_Ram[i][s16_Debug_Ram_Ptr] = 0;
   }
}

//**********************************************************
// Function Name: SalRecordCommand
// Description:
//   This function initialize the record process
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalRecordCommand(int drive, long long rec_gap, long long rec_duration, long long* ch)
{
   int word_index, i,s16_psd_code,s16_psd_offset;
   long rt_len, s32_rec_bw, code1 , code2 , code3;
   // rt record max bw allocation in words(16b)

   if RECORDING_PENDING return REC_ACTIVE;
   if ( (rec_gap <= 0) || (rec_gap > 1000000) ) return VALUE_OUT_OF_RANGE;
   if ( (rec_duration < 0) || (rec_duration > u16_Ram_Rec_Length_Avaliable) ) return VALUE_OUT_OF_RANGE;

   // Set gap value and counter
   s32_Gap_Value = rec_gap;
   s32_Gap_Cntr = s32_Gap_Value;

   s16_Recording_Length = rec_duration;    //Set record length

   for (i = 0; i < NUMBER_OF_REC_CHANNELS; i++)    //Init channels table
      s32_Rec_Channel[i] = 0;

   s16_Recorded_Variables_Number = 0;
   word_index = 0;

   ConvertMnemonic2Code("RECPSD", &code1, &code2, &code3, 0) ;
   s16_psd_code = SearchMnemonic(code1, code2, code3)-1;
   s16_psd_offset = 0;

   for (i = 0; i < NUMBER_OF_REC_CHANNELS-s16_psd_offset; i++)
   {
     if (ch[i] == 0x00LL) break;
     if (ch[i] != s16_psd_code)
     {
      if (!AllocateRecordChannel(drive, ch[i], &word_index))
           return VALUE_OUT_OF_RANGE;
         s32_Rec_Channel[i-s16_psd_offset] = (long)ch[i];
         s16_Recorded_Variables_Number++;
     }
     else
      s16_psd_offset = 2;
   }
   s16_Record_Channel_Number = word_index;

   ClearRecordBuffer();

   s16_Debug_Ram_Ptr = 0;

   if (rec_duration == 0x000) //RT recording
   {  // check max bw
      s32_rec_bw = F_DRV * 1000 * s16_Record_Channel_Number / s32_Gap_Value;
      if (s32_rec_bw > GetRecordBw(drive)) return VALUE_OUT_OF_RANGE;

      rt_len =  F_DRV * T_GET / s32_Gap_Value;   // calculate data packet length for RT recording
      // len modulo s16_Record_Channel_Number must be 0
      rt_len = rt_len * s16_Record_Channel_Number;
      if (rt_len < s16_Record_Channel_Number) rt_len = s16_Record_Channel_Number;
      s16_Recording_Length = (unsigned int)rt_len;
      u16_Packet_Counter = 1;
      u16_Data_Packet_Len = s16_Recording_Length + DATA_HEADER_LEN + CHECK_SUM_LEN;
      s16_Record_Flags |= RT_RECORD_ON_MASK;
   }

   //Start recording
   s16_Record_Flags |= RECORD_CONFIGURED_MASK;
   s16_Record_Flags |= RECORD_ON_MASK;
   s16_Record_Flags &= ~DATA_AVAILABLE_MASK;
   s16_Record_Flags &= ~REC_TRIGGER_DEFINED_MASK;

   return SAL_SUCCESS;
}


long GetRecordBw(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return ((long)((float)BGVAR(u32_Serial_Baud_Rate) / 30.769 + 0.5));
}


//**********************************************************
// Function Name: AllocateRecordChannel
// Description:
//   This function allocates channels for recorded variable
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int AllocateRecordChannel(int drive, long long channel, int* word_index)
{
   // AXIS_OFF;
   unsigned int u8_var_size;
   long addr;
   int axis_offset = drive;

   if (*word_index > NUMBER_OF_REC_CHANNELS)
           return 0;

   if ((channel & REC_ADD_FLAGS) == 0) //record variable
   {
      u8_var_size = Commands_Table[channel].u8_flags & 0x0003;
      addr = (long)Commands_Table[channel].ptr_read_access;
      // if BG var read from addr for AX0 or addr + len for AX1
      if (RT_VAR(Commands_Table[channel].u8_flags))addr = addr + axis_offset;
      // if RT var read from addr for AX0 or addr + 0x40 for AX1
      else addr = addr + (drive << (u8_var_size - 1));
      if (PTR_VAR(Commands_Table[channel].u8_flags)) addr = *(long*)addr; //pointer
   }
   else if (channel & REC_IOS) // record input
   {
      u8_var_size = 1;
      if (channel & REC_ADD_SIGN) // output
         addr = (long)&u16_Output[((channel & ~(REC_IOS | REC_ADD_SIGN)) - 1)];
      else
         addr = (long)((channel & ~REC_IOS) - 1) + (long)&u16_Input_1;
   }
   else // record address
   {
      u8_var_size = (channel & REC_ADD_SIZE) >> 28;
      addr = channel & ~REC_ADD_FLAGS;
   }

   switch (u8_var_size)
   {
      case 1: //16 bit var
         p_s16_Rec_Var_Ptr[*word_index] = (int*)addr;
         *word_index = *word_index + 1;
      break;
      case 2: //32 bit var
         p_s16_Rec_Var_Ptr[*word_index] = (int*)addr;
         *word_index = *word_index + 1;
         p_s16_Rec_Var_Ptr[*word_index] = (int*)(addr + 1L);
         *word_index = *word_index + 1;
      break;
      case 3: //64 bit var
         p_s16_Rec_Var_Ptr[*word_index] = (int*)addr;
         *word_index = *word_index + 1;
         p_s16_Rec_Var_Ptr[*word_index] = (int*)(addr + 1L);
         *word_index = *word_index + 1;
         p_s16_Rec_Var_Ptr[*word_index] = (int*)(addr + 2L);
         *word_index = *word_index + 1;
         p_s16_Rec_Var_Ptr[*word_index] = (int*)(addr + 3L);
         *word_index = *word_index + 1;
      break;
   }

   if (*word_index > NUMBER_OF_REC_CHANNELS) return 0;
   return  1;
}

long ParseInput(char* str)
{
   long u32_in_num = 0x7fffffff;

   if ( (str[3] == 0) && (str[2] >= '1') && (str[2] <= '9') )
      u32_in_num = (long)(str[2] - '0');
   else if ( (str[4] == 0) && (str[3] >= '0') && (str[3] <= '9')&& (str[2] >= '1') && (str[2] <= '9') )
   {
      u32_in_num = (long)(str[2] - '0') * 10;
      u32_in_num += (long)(str[3] - '0');
   }

   return (u32_in_num);
}

long ParseOutput(char* str)
{
   long u32_in_num = 0x7fffffff;

   if ( (str[4] == 0) && (str[3] >= '1') && (str[3] <= '9') )
      u32_in_num = (long)(str[3] - '0');
   else if ( (str[5] == 0) && (str[4] >= '0') && (str[4] <= '9')&& (str[3] >= '1') && (str[3] <= '9') )
   {
      u32_in_num = (long)(str[3] - '0') * 10;
      u32_in_num += (long)(str[4] - '0');
   }

   return (u32_in_num);
}

long ParseAddress(char* str)
{
   long flags = 0;
   char* p_u16_param = str;
   int i = 0;

   while ((*p_u16_param != 0) && (*p_u16_param != '.') && (i < 15))
   {
      i++;
      p_u16_param++;
   }

   if (*p_u16_param == 0) return (HexToLong(str + 1) | REC_ADD_16);   //address of type "Hxxxx

   if (StringCompare(".S16",p_u16_param, 4))
      flags = REC_ADD_16 | REC_ADD_SIGN;
   else if (StringCompare(".U16",p_u16_param, 4))
      flags = REC_ADD_16;
   else if (StringCompare(".S32",p_u16_param, 4))
      flags = REC_ADD_32 | REC_ADD_SIGN;
   else if (StringCompare(".U32",p_u16_param, 4))
      flags = REC_ADD_32;
   else if (StringCompare(".S64",p_u16_param, 4))
      flags = REC_ADD_64 | REC_ADD_SIGN;
   else if (StringCompare(".U64",p_u16_param, 4))
      flags = REC_ADD_64;

   if (flags != 0L)
   {
      *p_u16_param = 0;//terminate string address for hex conversion
      return HexToLong(str + 1) | flags;
   }

   return 0L; //not an address
}


//**********************************************************
// Function Name: SearchRecordVariable
// Description:
//   This function search for index of record channel in Commands_Table.
//   If variable found returns his index ,otherwise returns 0
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
long SearchRecordVariable(char* param)
{
   long code1, code2, code3, value;
   int index;

   if (param[0] == 'H' && param[1] == '0') return ParseAddress(param);
   // Use the above to verify that Commands such as HWPEXT are not interpreted as Hexadecimal

   if ((param[0] == 'I') && (param[1] == 'N'))
   {
      value = ParseInput(param);
      if (value <= s16_Num_Of_Inputs) return (value | REC_IOS);
   }
   // Use the above to handle INx

   if ((param[0] == 'O') && (param[1] == 'U') && (param[2] == 'T'))
   {
      value = ParseOutput(param);
      if (value <= s16_Num_Of_Outputs) return (value | REC_IOS | REC_ADD_SIGN); // REC_ADD_SIGN is used to differ between input and output
   }
   // Use the above to handle OUTx

   ConvertMnemonic2Code(param, &code1, &code2, &code3, 0);
   index = SearchMnemonic(code1, code2, code3);
   if (!index) return 0; //not found
   index--;

   // check is variable recordable
   if (READ_TYPE(Commands_Table[index].u8_rd_wr_method) == RD_DPTR) return index;
   else return 0;
}


//**********************************************************
// Function Name: SearchRecTrigParam
// Description:
//   This function search for index of record triger in s_Record_Trigger table.
//   If trigger found returns his index ,otherwise returns 0
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
long SearchRecTrigParam(char* param)
{
   int i, index;
   long code1, code2, code3, value;

   if (param[0] == 'H' && param[1] == '0')   return ParseAddress(param);
   // Use the above to verify that Commands such as HWPEXT is not interpreted as Hexadecimal

   if ((param[0] == 'I') && (param[1] == 'N'))
   {
      value = ParseInput(param);
      if (value <= s16_Num_Of_Inputs) return (value | REC_IOS);
   }
   // Use the above to handle INx

   if ((param[0] == 'O') && (param[1] == 'U') && (param[2] == 'T'))
   {
      value = ParseOutput(param);
      if (value <= s16_Num_Of_Outputs) return (value | REC_IOS | REC_ADD_SIGN); // REC_ADD_SIGN is used to differ between input and output
   }
   // Use the above to handle OUTx

   for (i = 1; i < NUMBER_OF_REC_TRIGGER_NAMES; i++)   // check if special trigger (CMD,IMM)
   {
      if (StringCompare(param,(char*)s_Record_Trigger[i].name,s_Record_Trigger[i].length + 1))
         return i + REC_TRIG_SPECIAL;
   }

   ConvertMnemonic2Code(param, &code1, &code2, &code3, 0);
   index = SearchMnemonic(code1, code2, code3);
   if (!index) return 0; //not found
   index--;

   //check is variable triggerable
   if (READ_TYPE(Commands_Table[index].u8_rd_wr_method) == RD_DPTR) return index;
   else return 0;
}


//**********************************************************
// Function Name: RecTrigCommand
// Description:
//   This function called in response to RECTRIG command
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int RecTrigCommand(void)
{
   unsigned long rectrig;
   int i, ret_val = SAL_SUCCESS,drive = 0;;

   drive +=0;
   if ( (BGVAR(s16_Pos_Tune_State) > 0)                               &&
        (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)                  &&
        ((BGVAR(s16_Pos_Tune_Av_State) == POS_TUNE_AV_FRQ2_IDENT) ||
         (BGVAR(s16_Pos_Tune_Av_State) == POS_TUNE_AV_FRQ3_IDENT)    )  ) return (AT_ACTIVE);

   rectrig = SearchRecTrigParam(u8_Execution_String[0]);   //check for correct triger name
   if (rectrig == 0)  return VALUE_OUT_OF_RANGE;

   STORE_EXECUTION_PARAMS_0_15
   for (i = s16_Number_Of_Parameters; i <= 3; i++)
      s64_Execution_Parameter[i] = 0;   //init empty record parameters

   ret_val = SalRecTrigCommand(u8_Axis_Id, rectrig, s64_Execution_Parameter[1], s64_Execution_Parameter[2], s64_Execution_Parameter[3]);
   RESTORE_EXECUTION_PARAMS_0_15

   return ret_val;
}


int CheckTrigRange(long long s64_trig_level, int u16_Rec_Trig_Width, int u16_Rec_Trig_Sign)
{
   switch (u16_Rec_Trig_Width)
   {
      case 0x0001: // 16 bit
         if (u16_Rec_Trig_Sign) // signed
         {
            if (s64_trig_level < -32768LL) return 0;
            if (s64_trig_level > 32767LL) return 0;
         }
         else // unsigned
         {
            if ((unsigned long long)s64_trig_level > 0x010000LL) return 0;
            if (s64_trig_level < 0LL) return 0;
         }
      break;
      case 0x0002: // 32 bit
         if (u16_Rec_Trig_Sign) // signed
         {
            if (s64_trig_level < -2147483648LL) return 0;
            if (s64_trig_level > 2147483647LL) return 0;
         }
         else // unsigned
         {
            if ((unsigned long long)s64_trig_level > 0x0100000000LL) return 0;
            if (s64_trig_level < 0LL) return 0;
         }
      break;
      case 0x0003: // 64 bit
         if (!u16_Rec_Trig_Sign)
            if (s64_trig_level < 0) return 0;
      break;
   }

   return 1;
}


//**********************************************************
// Function Name: SalRecTrigCommand
// Description:
//   This function initilaize recording trigger
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalRecTrigCommand(int drive, unsigned long u32_trig_mode, long long s64_trig_level, long long s64_trig_location, long long s64_trig_direction)
{
   // AXIS_OFF;
   unsigned long addr;
   long long s64_fix = 1;
   int s16_shift = 0;
   int s16_p11_idx = -1;
   int axis_offset = drive;

   if (!(s16_Record_Flags & RECORD_CONFIGURED_MASK)) return NOT_AVAILABLE;

   if RECORDING_PENDING return REC_ACTIVE;   //Check if recording active

   //indicate that units packet should be sent on 1st get command (for RT record only)
   if (s16_Record_Flags & RT_RECORD_ON_MASK) s16_Record_Flags |= PRINT_UNITS_PACKET;

   if (!u32_trig_mode) return VALUE_OUT_OF_RANGE;   //check trigger mode

   if ((s64_trig_location > s16_Recording_Length) || (s64_trig_location < 0LL)) return VALUE_OUT_OF_RANGE; // check trigger location

   //check trigger direction
   if ( (s64_trig_direction < 0) || (s64_trig_direction > 3) ) return VALUE_OUT_OF_RANGE;

   u8_Event_Trig_Mask = 0;
   s16_Event_Trig_Val = 0;
   u16_Rec_Trig_Width = REC_TRIG_16;
   u16_Rec_Trig_Sign = 1; // signed

   switch (u32_trig_mode)
   {
      case RECORD_IMM_TRIGGER://immidiate
         p_Rec_Trig_Val = &s16_Event_Trig_Val;
         s64_Rec_Trig_Level = 0L;
         u16_Trig_Location = 0;
         s16_Rec_Trig_Direction = POS_REC_TRIG;
         s16_Record_Flags |= WAITING_FOR_TRIGGER_MASK ; // Since the settings above will result
         // in no time where the Level Condition is not met, we need to arm the Trigger.
         s16_Event_Trig_Val = 1; // Set the condition "Trigger-Value > Trigger-Level"
      break;

      case RECORD_CMD_TRIGGER://triger on next command
         // Set the trigger value pointer to point to the event trigger value
         p_Rec_Trig_Val = &s16_Event_Trig_Val;
         s16_Record_Flags |= WAITING_FOR_TRIGGER_MASK ; // Since the settings above will result
         // in no time where the Level Condition is not met, we need to arm the Trigger.
         // Set the record trigger level to one. The execution
         // handler will set the s16_Event_Trig_Val to 1 in order to effect the trigger.
         s64_Rec_Trig_Level = 0LL;

         s16_Rec_Trig_Direction = POS_REC_TRIG;         // Set the trigger direction to positive

         u8_Event_Trig_Mask = TRIG_ON_COMMAND;

         u16_Trig_Location = 0;
      break;

      default: //Address or variable
         if ((u32_trig_mode <= COMMANDS_TABLE_SIZE) ||         //variable or PParam
             ((u32_trig_mode >= 1100) &&  (u32_trig_mode < 1200)))
         {
            if( (u32_trig_mode >= 1100) &&  (u32_trig_mode < 1200))
            { //P11-xx shadow parameter
               s16_p11_idx = u32_trig_mode - 1100;
               u32_trig_mode = (unsigned long) SearchP11ParamIndex(s16_p11_idx);
            }

            u16_Rec_Trig_Width = Commands_Table[u32_trig_mode].u8_flags & REC_TRIG_WIDTH_MASK;
            u16_Rec_Trig_Sign = (Commands_Table[u32_trig_mode].u8_flags & 0x0008) != 0L;
            addr = (long)Commands_Table[u32_trig_mode].ptr_read_access;
            //if BG var read from addr for AX0 or addr + width for AX1
            //if RT var read from addr for AX0 or addr + 0x40 for AX1
            if (RT_VAR(Commands_Table[u32_trig_mode].u8_flags))  addr = addr + axis_offset;
            else addr = addr + (drive << (u16_Rec_Trig_Width - 1));
            if (PTR_VAR(Commands_Table[u32_trig_mode].u8_flags)) addr = *(long*)addr; // pointer

            // conversion factor from internal units to user units

            //   for Schnider P11-xx, use hard coded unit translation for P11-xx
            if( (s16_p11_idx >= 0) &&  (s16_p11_idx < 100))
            {
               s64_Rec_Trig_Level = (long long) ConvertDriveStatusValue(0, s64_trig_level, s16_p11_idx,0 );
            }
            else
            {
                if (GetConversionFactor(drive, (unsigned int)u32_trig_mode, &s64_fix, &s16_shift,0) != -2) // Is units conversion or decimal is used
                  s64_Rec_Trig_Level = MultS64ByFixS64ToS64(s64_trig_level, s64_fix, s16_shift);
                else
                  s64_Rec_Trig_Level = s64_trig_level / 1000; // In case no units conversion is used ignore the decimal multiplication by 1000 done by the execution handler
            }
        }
        else  //address or IO
         {
            if (u32_trig_mode & REC_IOS)
            {
               u16_Rec_Trig_Width = 1;
               u16_Rec_Trig_Sign = 0L;
               if (u32_trig_mode & REC_ADD_SIGN)
                  addr = (long)&u16_Output[((u32_trig_mode & ~REC_ADD_FLAGS) - 1)];
               else
                  addr = (long)((u32_trig_mode & ~REC_ADD_FLAGS) - 1) + (long)&u16_Input_1;
            }
            else
            {
               u16_Rec_Trig_Width = (u32_trig_mode >> 28) & REC_TRIG_WIDTH_MASK;
               u16_Rec_Trig_Sign = (u32_trig_mode & REC_ADD_SIGN) != 0L;
               addr = u32_trig_mode & ~REC_ADD_FLAGS;
            }

            s64_Rec_Trig_Level = s64_trig_level / 1000; //the value multiplyed by 1000, because parameter defined as decimal
            //check trigger range
            if (!CheckTrigRange(s64_Rec_Trig_Level, u16_Rec_Trig_Width, u16_Rec_Trig_Sign)) return VALUE_OUT_OF_RANGE;
         }

         p_Rec_Trig_Val = (int*)addr;
         s16_Rec_Trig_Direction = s64_trig_direction;
         u16_Trig_Location = s64_trig_location;
   }

   if ((s16_Rec_Trig_Direction >= 2) && (s64_Rec_Trig_Level < 0LL)) return VALUE_OUT_OF_RANGE;

   s16_Record_Flags |= REC_TRIGGER_DEFINED_MASK;
   s16_Record_Flags |= RECORD_ON_MASK;
   s16_Record_Flags &= ~DATA_AVAILABLE_MASK;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: GetRecordCommand
// Description:
//   This function called in response to GET command
//
//
// Author: Dimitry/Gil
// Algorithm:
// Revisions:
//**********************************************************
int GetRecordCommand(void)
{
   static int get_state = 0, rec_ptr = 0;
   int drive = 0;
   int return_value;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (u16_Get_Debug_Log_Flag == 1)  // Use the GET command to retreive debug log
   {
      return_value = GetDebugLogCommand();
      if (return_value == SAL_SUCCESS) u16_Get_Debug_Log_Flag = 0;
      return (return_value);
   }

   //on "GET 3" , print units packet again
   if ((s64_Execution_Parameter[0] == 3) && (s16_Number_Of_Parameters == 1))
      s16_Record_Flags |= PRINT_UNITS_PACKET;

   if (s16_Record_Flags & RT_RECORD_ON_MASK) return PrinRtData(u8_Axis_Id);
   //Two seperate stat machines, in this switch:
   // Text mode  (getMode == 0), get_state = 0-Ready, 1-Print Hader, 2-Print Data line
   // Binary mode (getMode == 3 or 4)     get_state = 0-Ready, 3-Print Unit, 4-Print Data Header 5-Print Data line
   switch (get_state)
   {
      case 0:  //Check ranges and if recording data is available
         //check if recorded data availabe
         if (!(s16_Record_Flags & DATA_AVAILABLE_MASK)) return NO_REC_DATA;
         get_state++;
         //no break

      case 1:
         //check for free space at output buffer
         if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;

         if (BGVAR(u16_Get_Mode) == 0)     PrintRecordHeader();         // Print header
         else get_state++;

         rec_ptr = u16_Record_Start;         //initilaize pointer to start of recorded data

         get_state++;         //go to next state
      break;

      case 2:
         //check for free space at output buffer
         if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;

         if (rec_ptr == s16_Debug_Ram_Ptr)         //Check if all data have been printed
         {
            get_state = 0;
            return SAL_SUCCESS;
         }

         PrintRecordValue(u8_Axis_Id,rec_ptr);         //Print recorded data

         rec_ptr++;         //increment cyclic buffer
         if (rec_ptr == u16_Ram_Rec_Length_Avaliable)   rec_ptr = 0;
      break;

      case 3:
         if (PrintUnitsPacket(u8_Axis_Id) == SAL_SUCCESS)   get_state++;
      break;

      case 4:
         if (u8_Output_Buffer_Free_Space < 10) return SAL_NOT_FINISHED;
         if (BGVAR(u16_Get_Mode) == 3)
           PrintBinaryDataHeader(STAMP_DATA_PACKET_TYPE, 0x0001, s16_Record_Channel_Number * s16_Recording_Length + DATA_HEADER_LEN + CHECK_SUM_LEN);
         if (BGVAR(u16_Get_Mode) == 4)
           PrintBinaryDataHeader(STAMP_DATA_SUB_PACKET_TYPE, 0x0001, (s16_Record_Channel_Number + 1) * s16_Recording_Length + DATA_HEADER_LEN);
         get_state++;

      break;

      case 5:
          if (BGVAR(u16_Get_Mode) == 3)
          {
                if (PrintBinaryDataPacket() == SAL_SUCCESS)
                {
                   get_state = 0;
                   return SAL_SUCCESS;
               }
           }
           if (BGVAR(u16_Get_Mode) == 4)
           {
               if (PrintBinaryDataSubPacket(0) == SAL_SUCCESS)
               {
                   get_state = 0;
                   return SAL_SUCCESS;
               }
           }
      break;
   }

   return   SAL_NOT_FINISHED;
}

//**********************************************************
// Function Name: GetRecordRptCommand
// Description:
//   This function called in response to GET command
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int GetRecordRptCommand(int drive)
{
   int rec_ptr = u16_Record_Start;

   drive += 0;
   if (BGVAR(u16_Get_Mode) == 3) return NOT_AVAILABLE;

   if (s16_Number_Of_Parameters < 1) return SYNTAX_ERROR;

   if (s64_Execution_Parameter[0]>s16_Recording_Length) return VALUE_OUT_OF_RANGE;
   if (s64_Execution_Parameter[0]<0) return VALUE_OUT_OF_RANGE;

   if (s64_Execution_Parameter[0]==0)
   {
       if (BGVAR(u16_Get_Mode) == 0)
       {
           if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;
           PrintRecordHeader();         // Print header
           return SAL_SUCCESS;
       }
       if (BGVAR(u16_Get_Mode) == 4)
       {
           if (PrintUnitsPacket(u8_Axis_Id) == SAL_SUCCESS)
               return SAL_SUCCESS;
       }

   }
   else if (s64_Execution_Parameter[0]>=1)
   {
       if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;
       if (BGVAR(u16_Get_Mode) == 0)
       {
           rec_ptr = u16_Record_Start+ (unsigned int)s64_Execution_Parameter[0]-1;
           PrintRecordValue(u8_Axis_Id,rec_ptr);         //Print recorded data
           return SAL_SUCCESS;
       }
       if (BGVAR(u16_Get_Mode) == 4)
       {
           PrintBinaryDataHeader(STAMP_DATA_SUB_PACKET_TYPE, 0x0001, s16_Record_Channel_Number + DATA_HEADER_LEN + CHECK_SUM_LEN);
           PrintRecordValue(u8_Axis_Id,rec_ptr);         //Print recorded data
           if (PrintBinaryDataSubPacket((unsigned int)s64_Execution_Parameter[0]) == SAL_SUCCESS )
              return SAL_SUCCESS;
       }

   }
   return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: PrintRecordHeader
// Description:
//   This function prints recording header
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void PrintRecordHeader(void)
{
   int i;

   PrintString("CDHD Recording", 0);   //Print recording header
   PrintCrLf();

   PrintUnsignedInteger(s16_Recording_Length);   // Print recording length and gap
   PrintChar(COMMA);
   PrintUnsignedLong(s32_Gap_Value);
   PrintCrLf();

   for(i = 0; i < NUMBER_OF_REC_CHANNELS; i++)
   {
      if (s32_Rec_Channel[i] & REC_ADD_FLAGS) //address
      {
         if (i != 0) PrintChar(COMMA);
         PrintChar(DOUBLE_QUOTE);
         if (s32_Rec_Channel[i] & REC_IOS)
         {
            if (s32_Rec_Channel[i] & REC_ADD_SIGN) // Output
               PrintString("OUT", 0);
            else
               PrintString("IN", 0);

            PrintUnsignedInteger((int)(s32_Rec_Channel[i] & (~REC_ADD_FLAGS)));
         }
         else
            PrintDecInAsciiHex(s32_Rec_Channel[i] & ~REC_ADD_FLAGS, 8);
         PrintChar(DOUBLE_QUOTE);
      }
      else if (s32_Rec_Channel[i] > 0)
      {
         if (i != 0) PrintChar(COMMA);
         PrintChar(DOUBLE_QUOTE);
         ConvertCode2Mnemonic(&u16_Var_Name[0], Commands_Table[s32_Rec_Channel[i]].u32_mnemonic_code1, Commands_Table[s32_Rec_Channel[i]].u32_mnemonic_code2, Commands_Table[s32_Rec_Channel[i]].u32_mnemonic_code3);
         PrintString(u16_Var_Name, 0);
         PrintChar(DOUBLE_QUOTE);
      }
   }

   PrintCrLf();
}


//**********************************************************
// Function Name: PrintBinaryDataHeader
// Description:
//   This function prints binary recording header
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void PrintBinaryDataHeader(unsigned int stamp_type, unsigned int cntr, unsigned int len)
{  //Print recording header
   u16_Packet_Check_Sum = 0;
   WriteOutputBuffer16(stamp_type);
   WriteOutputBuffer16(cntr);
   WriteOutputBuffer16(len);
}


//**********************************************************
// Function Name: PrintBinaryError
// Description:
//   This function prints binary error packet
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void PrintBinaryError(void)
{
   u16_Packet_Check_Sum = 0;
   WriteOutputBuffer16(STAMP_ERR_PACKET_TYPE);   //Print recording header
   WriteCheckSum(); //checksum
}


//**********************************************************
// Function Name: PrintBinaryDataPacket
// Description:
//  This function prints recorded data in binary format (GETMODE 3)
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int PrintBinaryDataPacket(void)
{
   static int rec_ptr = -1;
   int i;

   if (rec_ptr == -1) rec_ptr = u16_Record_Start;

   while ((u8_Output_Buffer_Free_Space > 24) && (rec_ptr != u16_Record_End))
   { // 24 byte is a max record row
      for (i = 0; i < s16_Record_Channel_Number; i++)
      {
         WriteOutputBuffer16(s16_Debug_Ram[i][rec_ptr]);
      }//end for
      rec_ptr++;      //increment cyclic buffer
      if (rec_ptr == u16_Ram_Rec_Length_Avaliable)   rec_ptr = 0;
   }//end while

   if ((rec_ptr == u16_Record_End) && (u8_Output_Buffer_Free_Space > 2))
   {
      WriteCheckSum(); //print checksum
      rec_ptr = -1;
      return SAL_SUCCESS; //done
   }
   else
   {
      return SAL_NOT_FINISHED; //not finished yet
   }
}

//**********************************************************
// Function Name: PrintBinaryDataSubPacket
// Description:
//  This function prints recorded data in binary format
//
//
// Author: Gil
// Algorithm:
// Revisions:
// Lines begin at index 0
// if user sent u16_offset==0,all the data will be sent
// else if u16_offset>0,only the desired line-1 will be sent
//**********************************************************
int PrintBinaryDataSubPacket(unsigned int u16_offset)
{
   static int rec_ptr = -1;
   int i;

   if (rec_ptr == -1) rec_ptr = u16_Record_Start;
   rec_ptr+=(u16_offset-1); //lines begin at index 0

   while ((u8_Output_Buffer_Free_Space > 24) && (rec_ptr != u16_Record_End))
   { // 24 byte is a max record row
      u16_Packet_Check_Sum=0;
      for (i = 0; i < s16_Record_Channel_Number; i++)
      {
         WriteOutputBuffer16(s16_Debug_Ram[i][rec_ptr]);
      }//end for
      rec_ptr++;      //increment cyclic buffer
      if (rec_ptr == u16_Ram_Rec_Length_Avaliable)   rec_ptr = 0;
     WriteCheckSum(); //print checksum
     //In case there was a need to reprint specific subpacket->return here
     if (u16_offset)  return SAL_SUCCESS; //done
   }//end while

   if ((rec_ptr == u16_Record_End) && (u8_Output_Buffer_Free_Space > 2))
   {
      rec_ptr = -1;
      return SAL_SUCCESS; //done
   }
   else
   {
      return SAL_NOT_FINISHED; //not finished yet
   }
}


//**********************************************************
// Function Name: PrintUnitsPacket
// Description:
//  This function prints units packet
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int PrintUnitsPacket(int drive)
{
   static int i = 0, state = 0;
   unsigned int u8_var_flags, j;
   long long s64_fix = 1;
   int s16_shift = 0;
   char* var_name;
   long s32_temp = 0L;

   if (u8_Output_Buffer_Free_Space < 30 ) return SAL_NOT_FINISHED;

   switch (state)
   {
      case 0:
         u16_Packet_Check_Sum = 0;
         WriteOutputBuffer16(STAMP_UNITS_PACKET_TYPE); //stamp + units type
         WriteOutputBuffer16(0); //counter
         WriteOutputBuffer16(CHANNEL_INFO_LEN * s16_Recorded_Variables_Number + UNITS_HEADER_LEN + CHECK_SUM_LEN); //length
         WriteOutputBuffer16(0); //reserved
         WriteOutputBuffer16(0); //reserved
         WriteOutputBuffer16(s32_Gap_Value); //sample time low
         WriteOutputBuffer16(s32_Gap_Value >> 16); //sample time hi
         i = 0;
         state++;
      break;

      case 1:   // get flags (16 bit)
         if (s32_Rec_Channel[i] & REC_ADD_FLAGS)
         {
            if (s32_Rec_Channel[i] & REC_IOS)
            {
               if (s32_Rec_Channel[i] & REC_ADD_SIGN) // Output
               {
                  u8_Response[0] = 'O';
                  u8_Response[1] = 'U';
                  u8_Response[2] = 'T';
                  s32_temp = (s32_Rec_Channel[i] & (~REC_ADD_FLAGS));
                  u8_Response[3] = '0' + (int)s32_temp;
                  u8_Response[4] = 0;
               }
               else
               {
                  u8_Response[0] = 'I';
                  u8_Response[1] = 'N';
                  s32_temp = (s32_Rec_Channel[i] & (~REC_IOS));
                  if (s32_temp >= 10L)
                  {
                     u8_Response[2] = '1';
                     u8_Response[3] = '0' + ((int)s32_temp % 10);
                     u8_Response[4] = 0;
                  }
                  else
                  {
                     u8_Response[2] = '0' + (int)s32_temp;
                     u8_Response[3] = 0;
                  }
               }

               var_name = u8_Response;
               u8_var_flags = 1;
            }
            else
            {
               u8_var_flags = (s32_Rec_Channel[i] >> 28) & 0x000F;
               // get variable ASCII name (16 bytes)
               var_name = DecToAsciiHex(s32_Rec_Channel[i] & ~REC_ADD_FLAGS, 8);
            }
         }
         else
         {
            u8_var_flags = Commands_Table[s32_Rec_Channel[i]].u8_flags;
            //get variable ASCII name (16 bytes)
            ConvertCode2Mnemonic(&u16_Var_Name[0], Commands_Table[s32_Rec_Channel[i]].u32_mnemonic_code1, Commands_Table[s32_Rec_Channel[i]].u32_mnemonic_code2, Commands_Table[s32_Rec_Channel[i]].u32_mnemonic_code3);
            var_name = u16_Var_Name;
            //get units coversion fix & shift
            GetConversionFactor(drive, (unsigned int)s32_Rec_Channel[i], &s64_fix, &s16_shift,1);
         }
         WriteOutputBuffer16(u8_var_flags); //Variable flags

         var_name[15] = 0xcc; // debug
         for (j = 0; j < REC_VAR_NAME_LEN; j = j + 2)
         {
            if ((var_name[j] == 0) || (var_name[j + 1] == 0))
            {
               var_name[j + 1] = 0; // zero all bytes after name
            if (j!=14)
                var_name[j + 2] = 0; // zero all bytes after name
            }
            WriteOutputBuffer16(var_name[j] | var_name[j + 1] << 8);
         }

         WriteOutputBuffer16(s64_fix); //conversion factor fix
         WriteOutputBuffer16(s64_fix >> 16); //conversion factor fix
         WriteOutputBuffer16(s64_fix >> 32); //conversion factor fix
         WriteOutputBuffer16(s64_fix >> 48); //conversion factor fix
         WriteOutputBuffer16(s16_shift); //conversion factor shift 16 bit

         i++;
         if (i == s16_Recorded_Variables_Number)
         {
            state = 0;
            WriteCheckSum(); //print checksum
            s16_Record_Flags &= ~PRINT_UNITS_PACKET; //clr flag
         }
      break;
   }

   if (state == 0)   return SAL_SUCCESS; //done

   return SAL_NOT_FINISHED; //not finished yet
}


//**********************************************************
// Function Name: LoadUnitsPacket
// Description:
//  This function loads units packet to the buffer
//
//
// Author: Gil
// Revisions:
//**********************************************************
int LoadUnitsPacket(void)
{
   static int i = 0 ;
   unsigned int u8_var_flags, j;
   char u8_Respond[30];            /* u8_Response buffer */
   long long s64_fix = 1;
   int s16_shift = 0;
   int drive =0;
   char* var_name;
   long s32_temp = 0L;
   int s16_fb_units_index = 0, ret_val = 0, s16_drive_units_index = 0;
   int s16_currentObjectType;


   u16_Packet_Check_Sum = 0;
   WriteHeaderBuffer(STAMP_UNITS_PACKET_TYPE,0); //stamp + units type
   WriteHeaderBuffer(0,1); //counter
   WriteHeaderBuffer((CHANNEL_INFO_LEN * s16_Recorded_Variables_Number + UNITS_HEADER_LEN)*2,1); //length in Bytes
   WriteHeaderBuffer(0,1); //reserved
   WriteHeaderBuffer(0,1); //reserved
   WriteHeaderBuffer(s32_Gap_Value,1); //sample time low
   WriteHeaderBuffer(s32_Gap_Value >> 16,1); //sample time hi

   for (i=0;i<s16_Recorded_Variables_Number;i++)
   {
       if (s32_Rec_Channel[i] & REC_ADD_FLAGS)
       {
            if (s32_Rec_Channel[i] & REC_IOS)
            {
                 if (s32_Rec_Channel[i] & REC_ADD_SIGN) // Output
                 {
                      u8_Respond[0] = 'O';
                      u8_Respond[1] = 'U';
                      u8_Respond[2] = 'T';
                      s32_temp = (s32_Rec_Channel[i] & (~REC_ADD_FLAGS));
                      u8_Respond[3] = '0' + (int)s32_temp;
                      u8_Respond[4] = 0;
                 }
                 else
                 {
                      u8_Respond[0] = 'I';
                      u8_Respond[1] = 'N';
                      s32_temp = (s32_Rec_Channel[i] & (~REC_IOS));
                      if (s32_temp >= 10L)
                      {
                         u8_Respond[2] = '1';
                         u8_Respond[3] = '0' + ((int)s32_temp % 10);
                         u8_Respond[4] = 0;
                      }
                      else
                      {
                         u8_Respond[2] = '0' + (int)s32_temp;
                         u8_Respond[3] = 0;
                      }
                 }

                 var_name = u8_Respond;
                 u8_var_flags = 1;
            }
            else
            {
               u8_var_flags = (s32_Rec_Channel[i] >> 28) & 0x000F;
               // get variable ASCII name (16 bytes)
               var_name = DecToAsciiHex(s32_Rec_Channel[i] & ~REC_ADD_FLAGS, 8);
            }
       }
       else
       {
            u8_var_flags = Commands_Table[s32_Rec_Channel[i]].u8_flags;
            //get variable ASCII name (16 bytes)
            ConvertCode2Mnemonic(&u16_Var_Name[0], Commands_Table[s32_Rec_Channel[i]].u32_mnemonic_code1, Commands_Table[s32_Rec_Channel[i]].u32_mnemonic_code2, Commands_Table[s32_Rec_Channel[i]].u32_mnemonic_code3);

            var_name = u16_Var_Name;
            //get units coversion fix & shift
            s16_drive_units_index = UnitsConversionIndex(Commands_Table[s32_Rec_Channel[i]].str_units_var, drive);


            // if no units conversion or current unit conversion, return mA
            if (s16_drive_units_index == -1 || s16_drive_units_index == CURRENT_CONVERSION)
            {
                 s64_fix = 1;
                 s16_shift = 0;

                 s16_currentObjectType = 0;

               /*
               // if current command (units are "A"), dont perform unit conversion
               if ((Commands_Table[s32_Rec_Channel[i]].str_units_name != NA &&
                   Commands_Table[s32_Rec_Channel[i]].str_units_name[0] == 'A' &&
                   Commands_Table[s32_Rec_Channel[i]].str_units_name[1] == '\0') ||
                   (Commands_Table[s32_Rec_Channel[i]].str_units_var == BGVAR(s8_Units_Cur) &&
                   (*(char*)(Commands_Table[s32_Rec_Channel[i]].str_units_var)) == 'A' &&
                   (*(((char*)Commands_Table[s32_Rec_Channel[i]].str_units_var)+1)) == '\0') )
               {
               */
                 // if current command (units are "A")
                 if (Commands_Table[s32_Rec_Channel[i]].str_units_name != NA &&
                      Commands_Table[s32_Rec_Channel[i]].str_units_name[0] == 'A' &&
                      Commands_Table[s32_Rec_Channel[i]].str_units_name[1] == '\0')
                 {
                      s16_currentObjectType = 1;
                 }

                 // if internal current units (not A or mA).
                 if (Commands_Table[s32_Rec_Channel[i]].str_units_var == BGVAR(s8_Units_Cur) &&
                      (*(char*)(Commands_Table[s32_Rec_Channel[i]].str_units_var)) == 'A' &&
                      (*(((char*)Commands_Table[s32_Rec_Channel[i]].str_units_var)+1)) == '\0' )
                 {
                      s16_currentObjectType = 2;
                 }

                 if (s16_currentObjectType)
                 {
                      u8_var_flags &= ~0x0040;

                      // all current units are mA, excpet standard objects (0x60XX).
                      // 0x6075 is exception, it is standard object but in mA units.
                      for(j = 1; j < (int)SDO_LAST_OBJECT; j++)
                      {
                         if (FB_objects_array[j].u16_id == Commands_Table[s32_Rec_Channel[i]].u16_fildbus1_index)
                              break;
                      }

                      if (FB_objects_array[j].u16_id == Commands_Table[s32_Rec_Channel[i]].u16_fildbus1_index)
                      {
                           if ((FB_objects_array[j].u16_index & 0x6000) == 0x6000 && FB_objects_array[j].u16_index != 0x6075)
                           {
                                // need to convert to rated/1000
                                if (s16_currentObjectType == 1)
                                {
                                     s64_fix = BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_RECORDING_CONVERSION]).s64_unit_conversion_to_user_fix;
                                     s16_shift = BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_RECORDING_CONVERSION]).u16_unit_conversion_to_user_shr;
                                }
                                else if (s16_currentObjectType == 2)
                                {
                                     s64_fix = BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix;
                                     s16_shift = BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr;
                                }
                           }
                           else
                           {
                                // here we have manufacturer specific object (i.e. object index starts with 0x2000), so need to convert to mA
                                if (s16_currentObjectType == 1)
                                {
                                     s64_fix = 1;
                                     s16_shift = 0;
                                }
                                else if (s16_currentObjectType == 2)
                                {
                                     s64_fix = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix;
                                     s16_shift = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr;
                                }
                           }
                      }
                      else
                      {
                           // not found
                           return FAL_INTERNAL_FW_FAULT;
                      }
                 }
            }
            else
            {
                 if(s16_drive_units_index >= 0)
                 {
                      ret_val = FalConvertUnitIndex(s16_drive_units_index, &s16_fb_units_index);
                      if(ret_val != FAL_SUCCESS)
                           return ret_val;
                 }
                 else
                 {
                      return FAL_INTERNAL_FW_FAULT;
                 }

                 if(s16_fb_units_index != -1)
                     GetFieldBusConversionFactor(drive, s16_fb_units_index, &s64_fix, &s16_shift);
                 else if (s16_fb_units_index == FB_CAN_TORQUE_CONVERSION ||
                          s16_fb_units_index == FB_CAN_CURRENT_CONVERSION)
                 {
                     s64_fix = 1;
                     s16_shift = 0;
                 }
                 else
                 {
                     ret_val= GetConversionFactor(drive, (unsigned int)s32_Rec_Channel[i], &s64_fix, &s16_shift,1);
                     if (ret_val == -1)
                     {
                        s64_fix = 1;
                        s16_shift = 0;
                     }
                 }
            }
       }

       if (s16_fb_units_index == FB_CAN_ACC_DEC_USER_CONVERSION ||
           s16_fb_units_index == FB_CAN_ACC_DEC_PTP_USER_CONVERSION ||
           s16_fb_units_index == FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION ||
           s16_fb_units_index == FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION ||
           s16_fb_units_index == FB_CAN_POSITION_USER_CONVERSION ||
           s16_fb_units_index == FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION ||
           s16_fb_units_index == VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPM ||
           s16_fb_units_index == FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION ||
           s16_fb_units_index == POSITION_CONVERSION_LIN_ROT ||
           s16_fb_units_index == FB_CAN_TORQUE_CONVERSION ||
           s16_fb_units_index == FB_CAN_CURRENT_CONVERSION)
       {
           u8_var_flags &= ~0x0040;
       }



       WriteHeaderBuffer(u8_var_flags,1); //Variable flags

       var_name[15] = 0xcc; // debug
       for (j = 0; j < REC_VAR_NAME_LEN; j = j + 2)
       {
            if ((var_name[j] == 0) || (var_name[j + 1] == 0))
            {
                 var_name[j + 1] = 0; // zero all bytes after name
                 if (j!=14)
                      var_name[j + 2] = 0; // zero all bytes after name
            }
            WriteHeaderBuffer(var_name[j] | var_name[j + 1] << 8,1);
       }

       WriteHeaderBuffer(s64_fix,1); //conversion factor fix
       WriteHeaderBuffer(s64_fix >> 16,1); //conversion factor fix
       WriteHeaderBuffer(s64_fix >> 32,1); //conversion factor fix
       WriteHeaderBuffer(s64_fix >> 48,1); //conversion factor fix
       WriteHeaderBuffer(s16_shift,1); //conversion factor shift 16 bit

       if (i == (s16_Recorded_Variables_Number-1))
            s16_Record_Flags &= ~PRINT_UNITS_PACKET; //clr flag
   }

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: WriteHeaderBuffer
// Description:
//   Writes word to Header buffer
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
void WriteHeaderBuffer(unsigned int data_word,int index)
{
    static unsigned int u16_Index=0;
    u16_Index=index?u16_Index:0;//0 will cause writing since the begining of the array,else from the last location
//  Copy data to the output buffer
    u16_General_Domain[u16_Index++] = data_word ;
    u16_Index%=264;
    u16_General_Domain[264]=u16_Index;//contains the length
}

//**********************************************************
// Function Name: BinaryDataHeader
// Description:
//   Writes word Binary Data Header
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
void BinaryDataHeader(void)
{  //Print recording header
//  Copy data to the output buffer
    WriteHeaderBuffer(STAMP_DATA_PACKET_TYPE,0);
    WriteHeaderBuffer(0x0001,1); //offset for begining of data
    WriteHeaderBuffer((s16_Recording_Length + DATA_HEADER_LEN)*2,1); //length in Bytes
}

//**********************************************************
// Function Name: PrinRtData
// Description:
//  This function initialize transmission of RT recorded data
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int PrinRtData(int drive)
{
   static int state = 0;

   if (s16_Record_Flags & PRINT_UNITS_PACKET) return PrintUnitsPacket(drive);

   switch (state)
   {
      case 0: //init
         if (u8_Output_Buffer_Free_Space != COMMS_BUFFER_SIZE)
            return SAL_NOT_FINISHED; //verify that output buffer is empty
         u16_Packet_Start_Ptr = u16_Data_Packet_Ptr;
         //check if packet available
         if ( ((p_s16_Debug_Ram[u16_Packet_Start_Ptr]&0x00FF) != BIN_STAMP) ||
              ((s16_Record_Flags & REC_TRIGGERED_MASK) == 0)                  )
         {
            PrintBinaryError();
            return SAL_SUCCESS; //done
         }
         u16_Packet_End_Ptr = u16_Packet_Start_Ptr + s16_Recording_Length + DATA_HEADER_LEN; //calculate packet length
         if (u16_Packet_End_Ptr >= u16_Ram_Rec_Length_Word_Avaliable)
            u16_Packet_End_Ptr -= u16_Ram_Rec_Length_Word_Avaliable ; // cyclic buffer
         if (s16_Record_Flags & RT_RECORD_BUFFER_OVERRUN)
            p_s16_Debug_Ram[u16_Packet_Start_Ptr] |= BUFFER_OVERRUN_TYPE;
         s16_Record_Flags &= ~RT_RECORD_BUFFER_OVERRUN;
         u16_Packet_Check_Sum = 0; //initilize checksum
         s16_Record_Flags |= PRINT_RT_RECORD; //sign rt ro print recorded packet
         state++;
      break;

      case 1:
         if ((s16_Record_Flags & PRINT_RT_RECORD) == 0)
         {
            state = 0;
            return SAL_SUCCESS; //done
         }
      break;
   }

   return SAL_NOT_FINISHED; //not finished yet
}

// USED FOR RT CAN RECORDING.
// POSTPONED TILL REPAIREMENT
/**********************************************************
// Function Name: PrinRtDataCAN
// Description:
//  This function initialize transmission of RT recorded data
//
//
// Author: Gil
// Algorithm:
// Revisions:
//
void PrinRtDataCAN(void)
{
   manu_spec_Rec_Grab_command.DataStatus=0;
   u16_Packet_Start_Ptr = u16_Data_Packet_Ptr;

   //check if packet available
   if ( ((p_s16_Debug_Ram[u16_Packet_Start_Ptr]&0x00FF) != BIN_STAMP) ||
              ((s16_Record_Flags & REC_TRIGGERED_MASK) == 0))
   {

   manu_spec_Rec_Grab_command.DataStatus=1; //
   return;
   }
   u16_Packet_End_Ptr = u16_Packet_Start_Ptr + s16_Recording_Length + DATA_HEADER_LEN; //calculate packet length

   if (u16_Packet_End_Ptr >= u16_Ram_Rec_Length_Word_Avaliable)
   {
      manu_spec_Rec_Grab_command.DataStatus=2;
      u16_Packet_End_Ptr -= u16_Ram_Rec_Length_Word_Avaliable ; // cyclic buffer
   }
   if (s16_Record_Flags & RT_RECORD_BUFFER_OVERRUN)
      p_s16_Debug_Ram[u16_Packet_Start_Ptr] |= BUFFER_OVERRUN_TYPE;
   s16_Record_Flags &= ~RT_RECORD_BUFFER_OVERRUN;
//   s16_Record_Flags |= PRINT_RT_RECORD; //sign rt ro print recorded packet

   return; //not finished yet
}
*/

//**********************************************************
// Function Name: GetConversionFactor
// Description:
//   This function return variable conversion factor
//   To User or To Interanl
//
//   return value >0 is conversion found
//          -2 if no conversion decimal vlaue
//          -1 no conversion integer
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int GetConversionFactor(int drive, unsigned int u16_Cmd_Index, long long* s64_fix, int* s16_shift,  int s16_To_User)
{
   int i = 0;

   
   AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   *s64_fix = 1L;
   *s16_shift = 0;

   if (u16_Cmd_Index > COMMANDS_TABLE_SIZE) return -1;

   while (BGVAR(Unit_Conversion_Table[i]).units_type != 0)
   {
      if (Commands_Table[u16_Cmd_Index].str_units_var == BGVAR(Unit_Conversion_Table[i]).units_type)
      {
         if ((1 == Commands_Table[u16_Cmd_Index].u16_dual_mode_units) && (IS_DUAL_LOOP_ACTIVE))
         {
            i += 1; //In case Dual loop is active, use Load's units. otherwise, use Motor's units.
         }
         if(s16_To_User)
         {
            *s64_fix = BGVAR(Unit_Conversion_Table[i]).s64_unit_conversion_to_user_fix;
            *s16_shift = BGVAR(Unit_Conversion_Table[i]).u16_unit_conversion_to_user_shr;
         }
         else
         {
            *s64_fix = BGVAR(Unit_Conversion_Table[i]).s64_unit_conversion_to_internal_fix;
            *s16_shift = BGVAR(Unit_Conversion_Table[i]).u16_unit_conversion_to_internal_shr;
         }
         if ((1LL == *s64_fix) && (0 == *s16_shift)) //There was never a unit conversion here, so return (-2)
            return -2;                               //to make sure the value is devided by 1000 since it was falsly multuplied by 1000
         else   
         return i;
      }
      i++;
   }

   // Differ the return value if decimal is used or not
   if  (!DECIMAL(Commands_Table[u16_Cmd_Index].u8_flags))
      return -2;

   return -1;
}



int GetFieldBusConversionFactor(int drive, unsigned int u16_conv_index, long long* s64_fix, int* s16_shift)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   *s64_fix = BGVAR(Unit_Conversion_Table[u16_conv_index]).s64_unit_conversion_to_user_fix;
   *s16_shift = BGVAR(Unit_Conversion_Table[u16_conv_index]).u16_unit_conversion_to_user_shr;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: ConvertValue
// Description:
//   This function converts  value of recorded variable
//    Extracted from function PrintValue. to allow re-use in Modbus
//
// Author: D.R. /Udi
// Algorithm:
// Revisions:
//**********************************************************
void ConvertValue(int drive, unsigned int index, unsigned long long *converted_val)
{
   long long s64_fix = 1;
   int s16_shift = 0;
   if ((index > 0) && (index != 0x8000))
   {
      GetConversionFactor(drive, index, &s64_fix, &s16_shift, 1);

      *converted_val = MultS64ByFixS64ToS64(*converted_val, s64_fix, s16_shift);
   }
}


//**********************************************************
// Function Name: PrintValue
// Description:
//   This function prints value of recorded variable
//
//
// Author: D.R. /Udi
// Algorithm:
// Revisions:
//**********************************************************
void PrintValue( unsigned int u8_flags, unsigned long long u64_converted_val)
{

   if (UNSIGNED_PARAM(u8_flags))   //if parameter is unsigned, print unsigned value
   {
      if (DECIMAL(u8_flags))
         PrintUnsignedLongLong(u64_converted_val);
      else
         PrintUnsignedInt64(u64_converted_val);
   }
   else
   {
      if (DECIMAL(u8_flags))
         PrintSignedLongLong(u64_converted_val);
      else
         PrintSignedInt64(u64_converted_val);
   }
}


//**********************************************************
// Function Name: GetNextValue
// Description:
//   return value of recorded chanel, as u 64 bit
//     Also return index and flag to help unit translation
//
//   Extracted from function PrintRecordValue. to allow re-use in Modbus
//
// Author: D.R. / Udi
// Algorithm:
// Revisions:
//**********************************************************
void GetNextValue ( int rec_ptr,int *ch_ptr,unsigned int *u16_index,
                    unsigned int *u8_var_flags, int i,unsigned long long *u64_value )
{
   //int ch_ptr = 0;
   unsigned int  u8_var_size;
      if (s32_Rec_Channel[i] & REC_ADD_FLAGS) //address
      {
         if (s32_Rec_Channel[i] & REC_IOS)
         {
            *u8_var_flags = 1;
            u8_var_size =  1;
            *u16_index = 0x8000;
         }
         else
         {
            *u8_var_flags = (s32_Rec_Channel[i] >> 28) & 0x000F;
            u8_var_size =  (s32_Rec_Channel[i] & REC_ADD_SIZE) >> 28;
            *u16_index = 0;
         }
      }
      else  //variable
      {
         *u8_var_flags = Commands_Table[s32_Rec_Channel[i]].u8_flags;
         u8_var_size = Commands_Table[s32_Rec_Channel[i]].u8_flags & 0x0003;
         *u16_index = s32_Rec_Channel[i];
      }

      switch (u8_var_size)
      {
         case 0x0001: //16 bit var
            *u64_value = (long long)s16_Debug_Ram[(*ch_ptr)][rec_ptr];
            if (UNSIGNED_PARAM(*u8_var_flags)) *u64_value &= 0x000000000000FFFFLL; //clr sign bits
            (*ch_ptr)++;
         break;
         case 0x0002: //32 bit variable
            *u64_value = ( long long)s16_Debug_Ram[(*ch_ptr) + 1][rec_ptr]; //MSB
            *u64_value <<= 16;
            *u64_value |= ( long long)s16_Debug_Ram[(*ch_ptr)][rec_ptr] & 0x0FFFFLL;  //LSB
            if (UNSIGNED_PARAM(*u8_var_flags)) *u64_value &= 0x00000000FFFFFFFFLL; //clr sign bits
            (*ch_ptr) += 2;
         break;
         case 0x0003: //64 bit variable
            *u64_value = ( long long)s16_Debug_Ram[(*ch_ptr) + 3][rec_ptr] ; //MSB
            *u64_value <<= 16;
            *u64_value |= ( long long)s16_Debug_Ram[(*ch_ptr) + 2][rec_ptr] & 0x0FFFFLL;
            *u64_value <<= 16;
            *u64_value |= ( long long)s16_Debug_Ram[(*ch_ptr) + 1][rec_ptr] & 0x0FFFFLL;
            *u64_value <<= 16;
            *u64_value |= ( long long)s16_Debug_Ram[(*ch_ptr)][rec_ptr] & 0x0FFFFLL;   //LSB
            (*ch_ptr) += 4;
         break;
      }//end switch
}



//**********************************************************
// Function Name: PrintRecordValue
// Description:
//   This function prints value of recorded variable
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
void PrintRecordValue(int drive, int rec_ptr)
{
   int i, ch_ptr;
   unsigned int u8_var_flags;//, u8_var_size, u16_index;
   unsigned int u16_index;
   unsigned long long u64_temp;

   ch_ptr = 0;

   for (i = 0; i < NUMBER_OF_REC_CHANNELS; i++)
   {
       if (s32_Rec_Channel[i] == 0) break;
       if (i !=0 )
           PrintChar(',');
       GetNextValue(rec_ptr,&ch_ptr,&u16_index,&u8_var_flags, i, &u64_temp);
       ConvertValue(drive, u16_index,  &u64_temp);
       PrintValue(u8_var_flags, u64_temp);

   }//end for
   PrintCrLf();
}


/**********************************************************
* Function Name: StringCompare
* Description:
   This function is used to compare two strings OF EQUAL LENGTH.
* Author:
* Input Parameters:
   *string1: pointer to the first string
   *string2: pointer to the second string
   len: string length
* Output Parameters: None
* Return Value:
   0: Compare was sucessful
   1: Compare was not successful
* Algorithm:
* Global Variables Used: None
*
**********************************************************/
char StringCompare(char *string1, char *string2, int len)
{
   int i;      /* Loop counter */

   for (i = 0; i < len; i ++)
   {
      if ( *(string1 ++) != *(string2 ++) ) return 0;
   }
   return( 1 );
}


//**********************************************************
// Function Name: RecDoneCommand
// Description:
//   This function called in response to RECDONE command
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalRecdoneCommand(long long* recdone)
{
   if (!(s16_Record_Flags & DATA_AVAILABLE_MASK))
      *recdone = 0;
   else
      *recdone = 1;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: RecordingReadyCommand
// Description:
//   This function called in response to RECRDY command
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalRecordingReadyCommand(long long* ready)
{
   if (RECORDING_PENDING)
      *ready = 0;
   else
      *ready = 1;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: RecordingStatusCommand
// Description:
//   This function called in response to RECING command
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalRecordingStatusCommand(long long* stat)
{
   if (RECORDING_PENDING)
      *stat = 1;
   else
      *stat = 0;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: RecordOffCommand
// Description:
//   This function called in response to RECOFF command
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int RecordOffCommand(void)
{
   int drive = 0;
   drive +=0;
   if ( (BGVAR(s16_Pos_Tune_State) > 0)                               &&
     (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)                     &&
     ((BGVAR(s16_Pos_Tune_Av_State) == POS_TUNE_AV_FRQ2_IDENT) ||
      (BGVAR(s16_Pos_Tune_Av_State) == POS_TUNE_AV_FRQ3_IDENT)       )  ) return (AT_ACTIVE);

   ClrRecord();
   return SAL_SUCCESS;
}

void ClrRecord(void)
{
   s16_Record_Flags = 0;
   s16_Debug_Ram_Ptr = 0;
}

//**********************************************************
// Function Name: RecListCommand
// Description:
//   This function called in response to RECLIST command
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int RecListCommand(void)
{
   static int index = 0;
   static int index2 = 0;

   if (index2 < s16_Num_Of_Inputs)
   {
      PrintString("IN",0);
      PrintUnsignedInteger(index2 + 1);
      PrintCrLf();
      index2++;
      return SAL_NOT_FINISHED;
   }
   else if (index2 < (s16_Num_Of_Inputs + s16_Num_Of_Outputs))
   {
      PrintString("OUT",0);
      PrintUnsignedInteger(index2 - s16_Num_Of_Inputs + 1);
      PrintCrLf();
      index2++;
      return SAL_NOT_FINISHED;
   }

   while ((index < COMMANDS_TABLE_SIZE)  && (u8_Output_Buffer_Free_Space > 30))
   {
      if ((READ_TYPE(Commands_Table[index].u8_rd_wr_method) == RD_DPTR)&&
   (((Commands_Table[index].u16_validity_checks & 0x0001) == 0) || s16_Password_Flag))
      {
         if (PrintMnemonic(index)) PrintCrLf();
      }
      index++;
   }

   if (index >= COMMANDS_TABLE_SIZE)
   {
      index = 0;
      index2 = 0;
      return SAL_SUCCESS;
   }

   return SAL_NOT_FINISHED;
}


//**********************************************************
// Function Name: RecTrigListCommand
// Description:
//   This function called in response to RECTRIGLIST command
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int RecTrigListCommand(void)
{
   int i;

   if (RecListCommand() == SAL_NOT_FINISHED) return SAL_NOT_FINISHED;

   for (i = 1; i < NUMBER_OF_REC_TRIGGER_NAMES; i++)
      PrintStringCrLf((char*)s_Record_Trigger[i].name, 0);

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadRecordParameters
// Description:
//   This function called in response to P9-34 - P9-37 call
//   to get the  Modbus address to record - From internal P-Param
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadRecordParameters(long long *data,int drive)
{
   long long index = s64_Execution_Parameter[0];
   // to avoid remark
   drive +=0;
   // *data = Commands_Table[s32_Rec_Channel[index]].u16_p_param_number;
 // Temp - allow recording addresses - asume if more then 0xfff it is address.
   if(s32_Schneider_Rec_Channel[index] ==  0)
      *data = 0;
   else if(s32_Schneider_Rec_Channel[index] > 0xFFF)
      *data = s32_Schneider_Rec_Channel[index];
   else
      *data = PParamToModbus( s32_Schneider_Rec_Channel[index]);

   return SAL_SUCCESS;
}



//**********************************************************
// Function Name: SearchP11ParamIndex
// Description:
//  Return index in the command table for some P11 parameter.
//  Use to find the real time value to record for some p11-xx parameters.
//  Use with care:
//  The P11 and the entry in the table have same "Meaning" but
//  may have different units!.
//  See Spec in manuel for p0-02, which also defines P11-xx.
// Author: Udi
//**********************************************************
int SearchP11ParamIndex(int s16_P11_Index)
{
   switch(s16_P11_Index)
   {
      case 0:
      case 3: return ConvertMnemonic2Index("PFB");

      case 1:
      case 4: return ConvertMnemonic2Index("PCMD");

      case 2:
      case 5: return ConvertMnemonic2Index("PE");

      case 7: return ConvertMnemonic2Index("V");

      case 8: return ConvertMnemonic2Index("ANIN1");
      case 10: return ConvertMnemonic2Index("ANIN2");

      case 9:
      case 50: return ConvertMnemonic2Index("VCMD");

      case 11:
      case 53: return ConvertMnemonic2Index("ICMD");

      case 14: return ConvertMnemonic2Index("VBUSREADOUT");

      case 54:
      case 55: return ConvertMnemonic2Index("IQ");

      case 77: return ConvertMnemonic2Index("PTPVCMD");

      default: return -1;

   }
}



//**********************************************************
// Function Name: SalWriteRecordParametersCommand
// Description:
//   This function called in response to P10-20 - P10-23 call
//   Set the P Parameter to record From Input: Index and modbus address.
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteRecordParametersCommand(int drive)
{

  // Get input idx and value
  unsigned int  u16_p_param_number = 0;
  unsigned int u16_cmd_index,u16_param_index;

  long long index = s64_Execution_Parameter[0];
  unsigned long long   u64_modbus_addr = s64_Execution_Parameter[1];


   drive +=0; // to avoid remark

   //Check index range
   if((index < 0LL) || (index >= (long long) 4)) return VALUE_OUT_OF_RANGE; /*(Max number of variables to record - 4 channels)*/
   if(u64_modbus_addr == 0)
   {
      s32_Schneider_Rec_Channel[index] = 0;
      return SAL_SUCCESS;
   }

   // Translate input from Modbus to PPAram, and check if recordable.
   u16_p_param_number = ModbusToPParam((int) u64_modbus_addr);

   u16_cmd_index = SearchPParamIndex(u16_p_param_number, &u16_param_index);

   if (!u16_cmd_index) // If cmd not found
   {
      // return VALUE_OUT_OF_RANGE;
      //For future advanced debug option: If not P-Param - assume it is an address in FW code, from MAP file.
      s32_Schneider_Rec_Channel[index] = (((long)u64_modbus_addr) |  REC_ADD_16);
      return SAL_SUCCESS;
   }
   u16_cmd_index --;

  // check is variable recordable
   if (!READ_TYPE(Commands_Table[u16_cmd_index].u8_rd_wr_method) == RD_DPTR)
      return VAR_NOT_RECORDABLE;


    s32_Schneider_Rec_Channel[index] = u16_p_param_number;

   return SAL_SUCCESS;
}





//**********************************************************
// Function Name: GetRecordCommandIndex
// Description:
//   Return the index to the command to be recorded for each P-Param
//   Or address of the memory location.
// Author: Udi
// Algorithm:
//      Normal P-Param, in the groups P0-xx to P9-xx have entry in the commands-Table.
//         P10-xx are invisible and not recordable,
//         P11-xx are Shadow-Parameter, used to record real time data, as also avilable in P0-02
//          Unit Translation is hard-coded in ConvertDriveStatusValue
// Revisions:
//**********************************************************

unsigned long long GetRecordCommandIndex(unsigned long u16_P_Param )
{
   int s16_Param_Idx = 0;
   unsigned int u16_P_Param_Index = 0;

   s16_Param_Idx = SearchPParamIndex(u16_P_Param,&u16_P_Param_Index);

   if(s16_Param_Idx > 0)
          return s16_Param_Idx -1;
   // To help debug:
   //If not a valid P-param - assume it is an address to 16 bit, for now.
    return ( u16_P_Param | REC_ADD_16);


  // return s16_Param_Idx;
}








//**********************************************************
// Function Name: RecordCommandByPparam
// Description:
//   Copy of RecordCommand for record by Pparam
// Author: Moshe/Udi
// Algorithm:
// Revisions:
//**********************************************************
int RecordCommandByPparam(void)
{
   int i;

   for (i = 0; i < NUMBER_OF_REC_CHANNELS; i++)   // Init channels variables
       s64_Execution_Parameter[i + 2] = 0;

   if (BGVAR_COMM(u16_P10_12_NUMOFCHANNELS) < 1) return VALUE_TOO_LOW;

   if (BGVAR_COMM(u16_P10_16_BUFFERSIZE) >= u16_Ram_Rec_Length_Avaliable) return VALUE_TOO_HIGH;

   for (i = 0; i < BGVAR_COMM(u16_P10_12_NUMOFCHANNELS); i++)
   {   // Check for correct channel names
      s64_Execution_Parameter[i+2] = GetRecordCommandIndex(s32_Schneider_Rec_Channel[i]);
      if (s64_Execution_Parameter[i+2] == 0)  return VAR_NOT_RECORDABLE;
   }

   // time interval value error
   if(!setModbusSampleTime(BGVAR_COMM(u32_P10_14_TIMEBASE))) return VALUE_OUT_OF_RANGE;

   // set Schneider's interval time & numbr of points
   return SalRecordCommand(u8_Axis_Id, (long long)s32_Gap_Value, (long long)BGVAR_COMM(u16_P10_16_BUFFERSIZE), &s64_Execution_Parameter[2]);
}

//**********************************************************
// Function Name: SalReadTrigger
// Description:
//   This function called in response to P9-38 trigger readding
//   Translate PParam to Modbus address.
//
// Author: Udi
//**********************************************************
int SalReadTrigger(long long* data, int drive)
{
   drive += 0;
   if(u32_P10_24_LOADRTRI == 0)
       *data = 0LL;
   else
       *data = (long long) PParamToModbus(BGVAR(u32_P10_24_LOADRTRI) );
    return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteTrigger
// Description:
//   This function called in response to P10-24 trigger setting
//   Input is Modbus address of the triger
//   If valid trigger - keep it as PParam
//
// Author: Udi
//**********************************************************
int SalWriteTrigger(long long param,int drive)
{
  // Get input idx and value
  unsigned int  u16_p_param_number = 0;
  unsigned int u16_cmd_index,u16_param_index;


   if(param == 0)
   {
       BGVAR(u32_P10_24_LOADRTRI)  = 0LL;
       return SAL_SUCCESS;
   }

   drive +=0; // to avoid remarks

   // Translate input from Modbus to PPAram, and check if recordable.
   u16_p_param_number = ModbusToPParam((unsigned int)param);

   //Special case of Digital inputs and outputs as triggr
   if((u16_p_param_number >= 1200) && (u16_p_param_number <= 1220))
   {
      //Valid value are P12-01 (IN1) to P12-08 (s16_Num_Of_Inputs)   and
      //                P12-11 (Out1) to P12-16 (s16_Num_Of_Outputs)

      if(((u16_p_param_number >= 1201) && (u16_p_param_number <= (1200 + s16_Num_Of_Inputs) )) ||
         ((u16_p_param_number >= 1211) && (u16_p_param_number <= (1210 + s16_Num_Of_Outputs) )))
      {
            BGVAR(u32_P10_24_LOADRTRI)  = u16_p_param_number;
      }
      else
         return VALUE_OUT_OF_RANGE;
   }
   else //check if PParam can be used as trigger
   {

       u16_cmd_index = SearchPParamIndex(u16_p_param_number, &u16_param_index);


       if (!u16_cmd_index) // If cmd not found
           return VALUE_OUT_OF_RANGE;
       u16_cmd_index --;

       // check is variable recordable
       if (!READ_TYPE(Commands_Table[u16_cmd_index].u8_rd_wr_method) == RD_DPTR)
           return VAR_NOT_RECORDABLE;
   }
   BGVAR(u32_P10_24_LOADRTRI)  = u16_p_param_number;


   return SAL_SUCCESS;



}



//**********************************************************
// Function Name: SalWriteRecordCommand
// Description:
//   This function called in response to P10-11 record command call
//   to activate the record trigger or turn off the recording
//
// Author: Moshe
// Algorithm:
//   1: Start record: Call the CDHD code for "record" and "rectrig" commands,
//                    With appropiate parameters and unit conversion, to configure and arm the recording
//   0: turn off the record (recoff)

// Revisions:
//**********************************************************
int WriteRecordCommandViaPParam(long long param,int drive)
{
   int s16_return_value = SAL_SUCCESS;
   unsigned int   u16_param_index =0;
   unsigned long  u32_trig_mode = 0L; //Trigger-Index, IO, or Immidiate
   long long s64_value = 0LL;
   long long s64_pre_points = 0LL;
   long long s64_dir = 0LL;
   long long s64_auto_tune_value = 0LL;

  // Denay any recording activity during Auto tune
   PosTuneActiveCommand(&s64_auto_tune_value,0); // Only drive Zero for SE
   if (s64_auto_tune_value)
      return (REC_ACTIVE);


   u16_P10_11_COMMAND = (unsigned int)param;
   u16_MB_Read_Scope_Done = 0;
   if(!u16_P10_11_COMMAND)
   {
       return RecordOffCommand();
   }
   // Call record command to configure repording
   s16_return_value = RecordCommandByPparam();
   if(s16_return_value != SAL_SUCCESS) return s16_return_value;


   //Prepare for rectrig command
   if(BGVAR(u16_P10_18_DELAYTRIG) > 2000) return (VALUE_OUT_OF_RANGE);

   //Translate PPAram To the CDHD command index
   s64_dir = (long long)BGVAR(u16_P10_17_TRIGOP);
   if(s64_dir != 0)//0 Is Immidiate
   {   //P11-xx are special case of Shadow Parameters
       if((BGVAR(u32_P10_24_LOADRTRI) >= 1100 )&& ( BGVAR(u32_P10_24_LOADRTRI) < 1200))
           u32_trig_mode = BGVAR(u32_P10_24_LOADRTRI);
       else//P12-xx are IO for trigger only.
          if((BGVAR(u32_P10_24_LOADRTRI) >= 1200 )&& ( BGVAR(u32_P10_24_LOADRTRI) < 1210)) //Inputs
              u32_trig_mode = BGVAR(u32_P10_24_LOADRTRI - 1200);
          else if((BGVAR(u32_P10_24_LOADRTRI) >= 1210 )&& ( BGVAR(u32_P10_24_LOADRTRI) <= 1220)) //Outputs
              u32_trig_mode = BGVAR(u32_P10_24_LOADRTRI - 1210);

       else
       {
           u32_trig_mode = SearchPParamIndex(BGVAR(u32_P10_24_LOADRTRI), &u16_param_index);

           if(u32_trig_mode == 0)
               return VALUE_OUT_OF_RANGE;

           u32_trig_mode --;
       }


       // Translate =S= values in the PParam to internal values:
          s64_value = (long long)BGVAR(s32_P10_19_TRIGVAL1);
          s64_pre_points =  (long long)BGVAR(u16_P10_18_DELAYTRIG);




       //p12-00 to p12-20 are Inputs and outpus:
       if((BGVAR(u32_P10_24_LOADRTRI) >= 1200 )&& ( BGVAR(u32_P10_24_LOADRTRI) <= 1220))
       {
           u32_trig_mode |= REC_IOS;         //Mark as IO
           if(BGVAR(u32_P10_24_LOADRTRI) >= 1210 )
              u32_trig_mode |= REC_ADD_SIGN;  //Mark as output;

           s64_value *= 1000; //To meet decimal convention in cdhd

       }



   }



   //In CDHD "immidiate is set by trigger Address, In =S= by TRIGOP = 0
   switch (s64_dir)
   {
       case 0: u32_trig_mode = RECORD_IMM_TRIGGER; break; //Immidiate
       case 1: s64_dir = 1;break;//Going up
       case 2: s64_dir = 0;break; //Going Down
       default:
           return (VALUE_OUT_OF_RANGE);
   }
   // Call rectrig command to configure and arm the trigger.
   return SalRecTrigCommand(drive,u32_trig_mode ,s64_value,s64_pre_points,s64_dir);
}

//**********************************************************
// Function Name: ReadRecordCommandViaPParam
// Description:
//   This function called in response to P10-11 record read command call
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int ReadRecordCommandViaPParam(long long *data,int drive)
{
   drive+=0;
   *data = (long long)u16_P10_11_COMMAND;
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadFscopeStatus
// Description:
//          This function is called in response to the read P10-15 command.
//          This function returns the status of the internal scope in a
//          specific Lexium format.
//          0- Watting: Record running, wating for trigger
//          1- Trigering: trigger has occurred
//          2 - Ready: recording done
//          3 - Stopped: Wating a new command.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadFscopeStatus(long long* data)
{
   unsigned int u16_temp_val = 0;
   long long s64_temp_recdone_value;
   long long s64_temp_recing_value;
   long long s64_auto_tune_value;

   SalRecdoneCommand(&s64_temp_recdone_value);
   SalRecordingStatusCommand(&s64_temp_recing_value);
   PosTuneActiveCommand(&s64_auto_tune_value,0); // Only drive Zero for SE

   // Hide any recording activity during Auto tune
   if (s64_auto_tune_value)
       u16_temp_val = 3; // Stopped: Wating a new command
   //If no recording, Or recording exists but already read by UMAS
   else if(((s64_temp_recdone_value == 0)  || (u16_MB_Read_Scope_Done == 1)) &&
      (s64_temp_recing_value == 0))
   {
      u16_temp_val = 3; // Stopped: Wating a new command
   }
   else if ((s64_temp_recdone_value == 0) && (s64_temp_recing_value == 1))
   {
       if((s16_Record_Flags & WAITING_FOR_TRIGGER_MASK))
           u16_temp_val = 0; // Waitting for trigger
      else
           u16_temp_val = 1; // Recorder in progress
   }
   else if ((s64_temp_recdone_value == 1) && (s64_temp_recing_value == 0))
   {
      u16_temp_val = 2; // Data ready
   }
   else // Unknown state, not suppose to happen
   {
      u16_temp_val = 3; // Stopped: Wating a new command.
   }

   *data = u16_temp_val;

   return SAL_SUCCESS;
}


