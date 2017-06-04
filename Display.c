 //###########################################################################
//
// FILE:    Display.c
//
// TITLE:   Display control
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.01| 25 Sep 2002 | D.R. | Creation
//
//##################################################################

#include "DSP2834x_Device.h"
#include <string.h>
#include <cal_conf.h>
#include "nmt.h"
#include "led.h"


#include "Display.def"
#include "FltCntrl.def"
#include "ModCntrl.def"
#include "Design.def"
#include "FPGA.def"
#include "i2c.def"
#include "Current.def"
#include "Ser_Comm.def"
#include "Err_Hndl.def"
#include "CommFdbk.def"
#include "Exe_IO.def"
#include "AutoTune.def"
#include "Flash.def"
#include "PtpGenerator.def"
#include "Init.def"
#include "ExFbVar.def"
#include "402fsa.def"
#include "MotorParamsEst.def"
#include "PhaseFind.def"
#include "Homing.def"

#include "FltCntrl.var"
#include "User_Var.var"
#include "Extrn_Asm.var"
#include "Display.var"
#include "Drive.var"
#include "MotorSetup.var"
#include "PtpGenerator.var"
#include "Units.var"
#include "Foldback.var"
#include "i2c.var"
#include "Drive_Table.var"
#include "Ser_Comm.var"
#include "CommFdbk.var"
#include "An_fltr.var"
#include "Exe_IO.var"
#include "objects.h"
#include "FlashHandle.var"
#include "AutoTune.var"
#include "nvram_tabl.var"
#include "Position.var"
#include "ExFbVar.var"
#include "Lxm_profile.var"
#include "Init.var"
#include "MotorParamsEst.var"
#include "Exe_Hndl.var"
#include "Velocity.var"
#include "PhaseFind.var"
#include "Homing.var"
#include "Exe_Hndl.var"
#include "ModCntrl.var"

#include "Prototypes.pro"
#include "FltCntrl.pro"
#include "AutoTune.pro"
#include "Exe_IO.pro"

#include "PhaseFind.def"
#include "Homing.def"


void BlinkAllSegments(void)
{
   int i = 0;
   if (u8_Test_Hmi == 1)
   {
      s32_Display_Test_Timer = Cntr_1mS;
      u8_Test_Hmi++;
   }
   for (i = 0; i < CDHD_HMI_SIZE; ++i )
   {
      HWDisplay(ALL,i);
      BGVAR(u8_Local_Hmi_Blink_Display) = 0xFF;
   }
   if ( PassedTimeMS(2000L, s32_Display_Test_Timer))
   {
      s32_Display_Test_Timer = Cntr_1mS;
      u8_Test_Hmi = 0;
      BGVAR(u8_Local_Hmi_Blink_Display) = 0;
   }   
}
void TestIndividualSegmentsLed(void)
{
   int i = 0;
   if (u8_Test_Hmi_Seq == 1)
   {
      s32_Display_Test_Timer = Cntr_1mS;
      u8_Test_Hmi_Seq++;
   }
   for (i = 0; i < CDHD_HMI_SIZE; ++i )
   {
      if ( u8_Test_Hmi_Seq <= 10 )HWDisplay(~(1L<<(u8_Test_Hmi_Seq-2)),i);
      else HWDisplay((1L<<(u8_Test_Hmi_Seq-11)),i);
   }
   if ( PassedTimeMS(250L, s32_Display_Test_Timer))
   {
      s32_Display_Test_Timer = Cntr_1mS;
      u8_Test_Hmi_Seq++;
      if (u8_Test_Hmi_Seq == 19) u8_Test_Hmi_Seq = 0;
   }
}
int  SalReadInModeCommandHmi(long long *data,int drive)
{
   
   unsigned int input_num = GetHmiIndexFromCommandTable(GetCurrentIndexForExec())+1;
   //AXIS_OFF;
   REFERENCE_TO_DRIVE;
   s64_Execution_Parameter[0] = input_num;
   s16_Number_Of_Parameters = 1;
   *data = (long long)VAR(AX0_u16_Input_Mode_Arr[input_num]);
   return SAL_SUCCESS; 
}
int SalInModeCommandHmi(void)
{
   unsigned int input_num = GetHmiIndexFromCommandTable(GetCurrentIndexForExec())+1;
   s64_Execution_Parameter[1] = s64_User_Entered_Value;
   s64_Execution_Parameter[0] = input_num;
   s16_Number_Of_Parameters = 2;
   return SalInModeCommand();  
}
int SalReadInPolarCommandHmi(long long *data,int drive)
{
   int index =  GetHmiIndexFromCommandTable(GetCurrentIndexForExec())-10;
   int u16_mask = 0;
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   u16_mask = (1 << (int)(index - 1));
   if ((BGVAR(u16_Dig_In_Polar) & u16_mask) != 0) *data = 1;
   else *data = 0;
   return SAL_SUCCESS;
}
int SalInPolarCommandHmi(void)
{
   unsigned int input_num = GetHmiIndexFromCommandTable(GetCurrentIndexForExec())-10;
   s64_Execution_Parameter[1] = s64_User_Entered_Value;
   s64_Execution_Parameter[0] = input_num;
   s16_Number_Of_Parameters = 2;
   return SalInPolarCommand();
}
int SalReadOutModeCommandHmi(long long *data,int drive)
{
   long long index;
   REFERENCE_TO_DRIVE;
   index = GetHmiIndexFromCommandTable(GetCurrentIndexForExec())-21;
   *data = (u16_Out_Mode[index - 1LL]);
   return (SAL_SUCCESS);
}
int SalOutModeCommandHmi(void)
{
   unsigned int input_num = GetHmiIndexFromCommandTable(GetCurrentIndexForExec())-21;
   s64_Execution_Parameter[1] = s64_User_Entered_Value;
   s64_Execution_Parameter[0] = input_num;
   s16_Number_Of_Parameters = 2;
   return SalOutModeCommand();
}
int SalReadOutPolarCommandHmi(long long *data,int drive)
{
   unsigned int u16_mask;
   long long index = GetHmiIndexFromCommandTable(GetCurrentIndexForExec())-28;
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   u16_mask = (1 << (int)(index - 1));
   if ((BGVAR(u16_Dig_Out_Polar) & u16_mask) != 0) *data = 1;
   else *data = 0;
   return (SAL_SUCCESS);
}
int SalOutPolarCommandHmi(void)
{
   unsigned int input_num = GetHmiIndexFromCommandTable(GetCurrentIndexForExec())-28;
   s64_Execution_Parameter[1] = s64_User_Entered_Value;
   s64_Execution_Parameter[0] = input_num;
   s16_Number_Of_Parameters = 2;
   return SalOutPolarCommand();
}
//this function handles sal function read for the HMI returns the SAL function
// ret val and the result in read_result
int HmiExecuteReadCommand(int cmd_index,int drive, long long* read_result)
{
   int index = -1;
   int return_value;

   return_value = ExecuteReadCommand(cmd_index, drive, read_result);
   if (return_value == SAL_SUCCESS)
   {
      if (Commands_Table[cmd_index].str_units_var != 0)   // Handle units conversion
      {
         index = UnitsConversionIndex(Commands_Table[cmd_index].str_units_var, drive);

         if (index >= 0) // Unit conversion found
         {
            *read_result = MultS64ByFixS64ToS64(*read_result,
                                       BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_user_fix,
                                       BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_user_shr);
         }
      }
   }  
   return return_value;
}

//////////////////////////////////////////////////////////////////////////////////////
//                 Start of Sal Functions related to HMI                            //
//////////////////////////////////////////////////////////////////////////////////////     
//function added fr the display of actual speed via HMI
int SalActualSpeedHmiDeg(long long *data,int drive)
{
   // AXIS_OFF;
   drive+=0;
   //unit conversion
   *data = 360*MultS64ByFixS64ToS64(LVAR(AX0_s32_Vel_Var_Fb_0),
                                      BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPS]).s64_unit_conversion_to_user_fix,
                                      BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPS]).u16_unit_conversion_to_user_shr);                                      
   return (SAL_SUCCESS);
}
//function added fr the display of actual position via HMI
int SalActualPositionHmiDeg(long long *data,int drive)
{
   // AXIS_OFF;
   drive+=0;
   *data = MultS64ByFixS64ToS64( LLVAR(AX0_u32_Pfb_Internal_After_Mod_Lo), (long long)1474560000, (unsigned int)44);                                      
   
   return (SAL_SUCCESS);
}
int SalActualPositionHmiRevs(long long *data,int drive)
{
   // AXIS_OFF;
   long long     s64_fix = 1LL;
   unsigned int  u16_shift = 0, u16_mpitch_factor_shift = 0;
   long          s32_mpitch_factor_fix = 1000L;//changed since mpitch changed to decimal
   drive+=0;
   MultFix64ByFix32ToFix32((long*)&s64_fix, &u16_shift,(long long)1073741824, (unsigned int)62,s32_mpitch_factor_fix,u16_mpitch_factor_shift);
   *data = MultS64ByFixS64ToS64( LLVAR(AX0_u32_Pfb_Internal_After_Mod_Lo), s64_fix, u16_shift);                                      
   return (SAL_SUCCESS);
}
int SalSwitchEnableDisable(long long *data,int drive)
{    
   
   if ((u16_User_Editing_Mode > VIEW_INDEX_VALUE))
   {
      if (s64_User_Entered_Value)
      {
         EnableCommand(drive);
         *data = (long long)1;
      }
      else
      {
         DisableCommand(drive);
         *data = (long long)0;
      }              
   }
   else
      *data = (long long)Enabled(drive);

   return (SAL_SUCCESS);
}
int SalActualSpeedHmiRpm(long long *data,int drive)
{
   // AXIS_OFF;
   drive+=0;
   *data = MultS64ByFixS64ToS64(LVAR(AX0_s32_Vel_Var_Fb_0),
                                       BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPM]).s64_unit_conversion_to_user_fix,
                                       BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPM]).u16_unit_conversion_to_user_shr);
   *data = *data/1000LL;      
   return (SAL_SUCCESS);
}

//disable keys and 
int SalSoftwareControlCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;
   if (!s16_Password_Flag)   return (PASSWORD_PROTECTED);
   u16_Hmi_Key_Disable = param;
   return SAL_SUCCESS;
}
int SalCanStatusWord(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;
   *data = p402_statusword;
   return SAL_SUCCESS;
}
int SalCanControlWord(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;
   *data = p402_controlword;
   return SAL_SUCCESS;
}
int SalForceKey(int drive)
{
   REFERENCE_TO_DRIVE;
   if (!s16_Password_Flag)   return (PASSWORD_PROTECTED);
   return BGVAR(u16_P4_08_Push_Button_Status);
}
//this func enables key pressing using serail connection
int SalWriteForceKeyCommand(int drive)
{
   char key = (char)s64_Execution_Parameter[0];
   int duration = 0;
   REFERENCE_TO_DRIVE;
   if (!s16_Password_Flag)   return (PASSWORD_PROTECTED);
   if (s16_Number_Of_Parameters == 1) duration = 1;
   if (s16_Number_Of_Parameters == 2) duration = (int)s64_Execution_Parameter[1];
   if ( duration == 1 )
   {
      switch (key)
      {
         case (8): BGVAR(u16_P4_08_Push_Button_Status) = 
            BGVAR(u16_Push_Button_Status_Temp1) = 
            BGVAR(u16_Push_Button_Status_Temp2) = BUTTON_UP_STATUS;//Up
            break;
         case (2): BGVAR(u16_P4_08_Push_Button_Status) = 
            BGVAR(u16_Push_Button_Status_Temp1) = 
            BGVAR(u16_Push_Button_Status_Temp2) = BUTTON_DOWN_STATUS;//Down
            break;
         case (5): BGVAR(u16_P4_08_Push_Button_Status) = 
            BGVAR(u16_Push_Button_Status_Temp1) = 
            BGVAR(u16_Push_Button_Status_Temp2) = BUTTON_M_STATUS;//Mode 
            break;
         case (4): BGVAR(u16_P4_08_Push_Button_Status) = 
            BGVAR(u16_Push_Button_Status_Temp1) = 
            BGVAR(u16_Push_Button_Status_Temp2) = BUTTON_S_STATUS;//Left Shift
            break;
         case (6): BGVAR(u16_P4_08_Push_Button_Status) = 
            BGVAR(u16_Push_Button_Status_Temp1) = 
            BGVAR(u16_Push_Button_Status_Temp2) = BUTTON_M_S_STATUS;//Left Shift + M
            break;
         default : break;
      }
   }
   if (duration == 2)
   {
      u16_Hmi_Key_Long_Press = 1;
   }
   else
   {
      u16_Hmi_Key_Long_Press = 0;
   }
   return SAL_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////////////
//                   End of Sal Functions related to HMI                            //
//////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////
// handling the command table search and display     //
///////////////////////////////////////////////////////
unsigned int GetHmiGroupFromCommandTable(unsigned int command_table_index)
{
   unsigned int hmi_group_index = Commands_Table[command_table_index].u16_hmi_group_index;
   return (hmi_group_index >> 13);
}
unsigned int GetHmiSubGroupFromCommandTable(unsigned int command_table_index)
{
   unsigned int hmi_sub_group_index = Commands_Table[command_table_index].u16_hmi_group_index;
   hmi_sub_group_index = hmi_sub_group_index >> 8;
   hmi_sub_group_index = hmi_sub_group_index & 0x1F;
   return hmi_sub_group_index;
}
unsigned int GetHmiIndexFromCommandTable(unsigned int command_table_index)
{
   return (Commands_Table[command_table_index].u16_hmi_group_index & 0xFF); 
}



unsigned int GetSizeOfSubGroup(unsigned int group_index, unsigned int sub_group_index)
{
   int command_table_index = 0;
   unsigned int size = 0;
   for (command_table_index = 0; command_table_index < COMMANDS_TABLE_SIZE; ++command_table_index)
   {
      if ( (sub_group_index == GetHmiSubGroupFromCommandTable(command_table_index) && group_index == GetHmiGroupFromCommandTable(command_table_index)) )
      {
         ++size;  
      }  
   } 
   return size;
}


//HMI Init Indexes Arrays 
//Bubble Sort
void SortHmiMenuArrays(unsigned int* index_arr,int size_of_array)
{
   unsigned int i,j,temp = 0;
   unsigned int u16_is_swapped = 0;
   for (i = 1; i < size_of_array; ++i)
   {
      for (j = 1; j < size_of_array; ++j)
      { 
         if ( Commands_Table[index_arr[j-1]].u16_hmi_group_index > Commands_Table[index_arr[j]].u16_hmi_group_index )
         {
            temp = index_arr[j-1];
            index_arr[j-1] = index_arr[j];
            index_arr[j] = temp;
            u16_is_swapped = 1; 
         }
      }
      if (!u16_is_swapped) break;
   }  
}
//this function goes over the command xls and count params, commands and insert the indexes into arrays
void InitHmiMenuArrays(void)
{
   int command_table_index = 0;
   unsigned int temp_hmi_group = 0;
   for (command_table_index = 1; command_table_index < COMMANDS_TABLE_SIZE; ++command_table_index)
   {
      temp_hmi_group = GetHmiGroupFromCommandTable(command_table_index);
      switch (temp_hmi_group)
      {
         case(0):break;
         case(HMI_GROUP_PARAMS):
            u16_Menu_Params_Index_Arr[u16_Hmi_Params_Group_Size] = command_table_index;
            ++u16_Hmi_Params_Group_Size; 
            break;   
         case(HMI_GROUP_COMMANDS):
            u16_Menu_Commands_Index_Arr[u16_Hmi_Commands_Group_Size] = command_table_index;
            ++u16_Hmi_Commands_Group_Size; 
            break;
         case(HMI_GROUP_MONITORS):
            u16_Menu_Monitors_Index_Arr[u16_Hmi_Monitors_Group_Size] = command_table_index;
            ++u16_Hmi_Monitors_Group_Size; 
            break;
         default: 
            break; 
      }
   } 
}

void InitHmiMenuIndexArrays(void)
{ 
   InitHmiMenuArrays();

   SortHmiMenuArrays(u16_Menu_Params_Index_Arr,u16_Hmi_Params_Group_Size);
   SortHmiMenuArrays(u16_Menu_Monitors_Index_Arr,u16_Hmi_Monitors_Group_Size);
   SortHmiMenuArrays(u16_Menu_Commands_Index_Arr,u16_Hmi_Commands_Group_Size);
}

//HMI Init Indexes Arrays end

//////////////////////////////////////////////////////////////
// End of handling the command table search and display     //
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// Display helper fuctions                                  //
//////////////////////////////////////////////////////////////
//for easy display of five letter strings
void HmiDisplayFiveLetterString(char* str,unsigned int len)
{
   int i = 0;
   for (i = 0; i <= 4; ++i)
   {
      BGVAR(u16_Local_Hmi_Display_Array)[i] = GetHexDecValForCDHD7SegDisplay((unsigned int)str[len - i -1],0);
      HWDisplay(GetHexDecValForCDHD7SegDisplay((unsigned int)str[len - i -1],0),i);
   }   
}
//this function display a 20 digit nuber (64bit) 5 digits at the 
//time according to offset set
//decimal points represents the MSB LSB offset
void DisplayMsbLsb(unsigned int offset,unsigned int show_digit )
{
   int i = 0;
   unsigned int u16_display_array[4*CDHD_HMI_SIZE] = {0};
   
   for (i = 0; i < 4*CDHD_HMI_SIZE; ++i)
   {
      u16_display_array[i] = GetHexDecValForCDHD7SegDisplay(u16_Display_Array_Msb_Lsb[i],DECIMAL_POINT_OFF); 
      //turn decimal point according to LSB -> MSB representation 
      if ( (i % 5 == 0) ||
           ((i % 5 == 1) && (i > 5)) || 
           ((i % 5 < 3) && (i > 10)) ||
           ((i % 5 < 4) && (i > 15)))
      {
         u16_display_array[i] = GetHexDecValForCDHD7SegDisplay(u16_Display_Array_Msb_Lsb[i],DECIMAL_POINT_ON); 
      }
   }
    
   
   if ( u16_Display_Neg_Val_Flag == 1 )
   {
      u16_display_array[4] = GetHexDecValForCDHD7SegDisplay(u16_Display_Array_Msb_Lsb[4],DECIMAL_POINT_ON); 
      u16_display_array[9] = GetHexDecValForCDHD7SegDisplay(u16_Display_Array_Msb_Lsb[9],DECIMAL_POINT_ON); 
      u16_display_array[14] = GetHexDecValForCDHD7SegDisplay(u16_Display_Array_Msb_Lsb[14],DECIMAL_POINT_ON); 
      u16_display_array[19] = GetHexDecValForCDHD7SegDisplay(u16_Display_Array_Msb_Lsb[19],DECIMAL_POINT_ON); 
   }
   if ( u16_Display_Dec_Val_Flag == 1 )
   {
      u16_display_array[3] = GetHexDecValForCDHD7SegDisplay(u16_Display_Array_Msb_Lsb[3],DECIMAL_POINT_ON); 
   }
   for (i = 0 ; i < CDHD_HMI_SIZE; ++i)
   {
      if ( i < show_digit )
      {
         BGVAR(u16_Local_Hmi_Display_Array)[i] = u16_display_array[i+offset];
         HWDisplay(u16_display_array[i+offset],i);
      }
      else
      {
         BGVAR(u16_Local_Hmi_Display_Array)[i] = NONE;
         HWDisplay(NONE,i);
      }
   } 
}
void Conv20DigLongLongToArr(unsigned int* dest_arr,long long* source)
{
   long long temp = *source;
   unsigned int i = 0;
   while ( (i < (CDHD_HMI_SIZE*4)) )
   {
      dest_arr[i] = (temp)%(long long)10;
      temp = (temp)/(long long)10;
      ++i;
   }
}
void PrepLongLongForDisplay(int drive, long long* source)
{
   REFERENCE_TO_DRIVE;
   if ( *source < (long long)0 )
   {
      u16_Display_Neg_Val_Flag = 1;
      *source = *source * (-1);
   }
   else if ( u16_User_Editing_Mode !=  EDIT_ENABLE ) 
   {
      u16_Display_Neg_Val_Flag = 0;
   }
   if ( DECIMAL(Commands_Table[GetCurrentIndexForExec()].u8_flags) )
   {
      u16_Display_Dec_Val_Flag = 1;
   }
   Conv20DigLongLongToArr(u16_Display_Array_Msb_Lsb,source);
}

void HmiDisplayRetVal(int drive,unsigned int val,int initial_char)
{
   unsigned int u16_display_array[CDHD_HMI_SIZE] = {0};
   unsigned int i = 0;
   REFERENCE_TO_DRIVE;
   u16_display_array[4] = initial_char;
   //if sal function returns sal success display inserted val
   if (val == SAL_SUCCESS)
   {
      //HmiExecuteReadCommand(GetCurrentIndexForExec(),drive, (long long*)&s64_User_Entered_Value);
      PrepLongLongForDisplay(drive,&s64_User_Entered_Value);
      DisplayMsbLsb(u16_User_Editing_Msb_Lsb_Offset, CDHD_HMI_SIZE);
      return;
   }
   while (val)
   {
      u16_display_array[i] = val%10;
      val = val/10;
      ++i;
   }
   for (i = 0; i < CDHD_HMI_SIZE; ++i)
   {
      BGVAR(u16_Local_Hmi_Display_Array)[i] = Cdhd2ConvertDigitTo7SegDisplay(u16_display_array[i]);
      HWDisplay(Cdhd2ConvertDigitTo7SegDisplay(u16_display_array[i]),i);
   }
}



//power function recursive implementation 
long long llpow(int base, int ex)
{
   // power of 0
   if (ex == 0)
   {
      return 1;
   }
   else
   {
     return base * llpow(base, ex - 1);
   }
}

//this func converts decimal representation 	to hexadecimal repr.
void IntToHex(int int_number, char *str)//TOD comment 
{    
    char temp_str='0';
    char i=5;
    do {
        i--;
        temp_str = (char)  ((int_number >> i*4 ) & 0x000f);
        if(temp_str < 10) temp_str = temp_str + 48;//digits 0-9
        else temp_str=temp_str+55;//digits a-f
        *str ++= temp_str;    
    } while(i > 0); 
    str[0] = 'h'; 
}


                                                                                                                       
unsigned int GetSubGroupIndexForDisplay(unsigned int sub_group)
{
   unsigned int corrected_index = 0;
   switch (sub_group)
   {
      case(0):corrected_index = 0;break;
      case(1):corrected_index = 10;break;
      case(2):corrected_index = 11;break;
      case(3):corrected_index = 12;break;
      case(4):corrected_index = 13;break;
      case(5):corrected_index = 20;break;
      case(6):corrected_index = 21;break;
      case(7):corrected_index = 30;break;
      case(8):corrected_index = 40;break;
      case(9):corrected_index = 42;break;
      case(10):corrected_index = 50;break;
      case(11):corrected_index = 60;break;
      case(12):corrected_index = 70;break;
      case(13):corrected_index = 71;break;
      case(14):corrected_index = 72;break;
      case(15):corrected_index = 73;break;
      default: break;
   }
   return corrected_index;
}

unsigned int GetItemIndexForDisplay()
{
   unsigned int item_index = 0;
   unsigned int sub_group = 0;
   unsigned int sub_group_display_index = 0;
   unsigned int index_for_display = 0;   
   int hmi_group = u16_Current_Display_mode;  
   unsigned int* index_arr; 
   unsigned int index; 
   
   switch (hmi_group)
   {
      case(MENU_MODE_COMMANDS): index_arr = u16_Menu_Commands_Index_Arr; index = u16_Current_Commands_Menu_Item_Index;
      break;
      case(MENU_MODE_PARAMS):index_arr = u16_Menu_Params_Index_Arr; index = u16_Current_Params_Menu_Item_Index;
      break;
      case(MENU_MODE_MONITORS):index_arr = u16_Menu_Monitors_Index_Arr; index = u16_Current_Monitors_Menu_Item_Index;
      break;
      case(MENU_MODE_FAULTS):return u16_Fault_Menu_Mode;
      default:
      break;
   }
   
   sub_group = GetHmiSubGroupFromCommandTable(index_arr[index]);
   if ( ((hmi_group == MENU_MODE_COMMANDS) || (hmi_group ==  MENU_MODE_PARAMS)) )
   {
      item_index = GetHmiIndexFromCommandTable(index_arr[index]);
      sub_group_display_index = GetSubGroupIndexForDisplay(sub_group);
      index_for_display = sub_group_display_index*100 + item_index;
   }
   else index_for_display =  index;
   return index_for_display; 
}

int HandleReadOnlyParamIndication(void)
{
   if ( MIN_WRITE_PARAMS(Commands_Table[GetCurrentIndexForExec()].u16_min_max_arg) == 0 && u16_Current_Display_mode == MENU_MODE_PARAMS) return DECIMAL_POINT_ON;
   else return DECIMAL_POINT_OFF;
}
void DisplayMenuModeIndex(void)
{
   unsigned int u16_display_array[CDHD_HMI_SIZE] = {0};
   unsigned int temp_item_index = 0; 
   unsigned int i = 0;
   
   switch (u16_Current_Display_mode)
   {
      case (MENU_MODE_COMMANDS):
         u16_display_array[4] = 'C';
         break;
      case (MENU_MODE_PARAMS):
         u16_display_array[4] = 'P';
         break;
      case (MENU_MODE_MONITORS):
         u16_display_array[4] = 'D';
         break;
      case (MENU_MODE_FAULTS):
         u16_display_array[4] = 'F';
         temp_item_index = u16_Fault_Menu_Mode; 
         break;
      default:
         return;
   }
   temp_item_index = GetItemIndexForDisplay();
   while (temp_item_index)
   {
      u16_display_array[i] = temp_item_index%10;
      temp_item_index = temp_item_index/10;
      ++i;
   }
   BGVAR(u16_Local_Hmi_Display_Array)[4] = GetHexDecValForCDHD7SegDisplay(u16_display_array[4],HandleReadOnlyParamIndication());
   HWDisplay(GetHexDecValForCDHD7SegDisplay(u16_display_array[4],HandleReadOnlyParamIndication()),4);
   for (i = 0; i < CDHD_HMI_SIZE-1 ; ++i)
   {
      BGVAR(u16_Local_Hmi_Display_Array)[i] = Cdhd2ConvertDigitTo7SegDisplay(u16_display_array[i]);
      HWDisplay(Cdhd2ConvertDigitTo7SegDisplay(u16_display_array[i]),i);
   }
   return;
}




//////////////////////////////////////////////////////////////
// HMI Fault Display                                        // 
//////////////////////////////////////////////////////////////
void HmiDisplayFault(int index)
{
   unsigned int u16_display_array[CDHD_HMI_SIZE] = {0};
   int i = 0;
   int erase_flag = 0;
   u16_display_array[4] = F_LET;
   //unknown fault handle
   if (index == 143)
   {
      u16_display_array[3] = F_LET;
      u16_display_array[2] = F_LET;
      u16_display_array[1] = F_LET;
      u16_display_array[0] = NONE;
   }
   else
   {
      u16_display_array[3] = Display_Table[index-1].u16_char1;
      u16_display_array[2] = Display_Table[index-1].u16_char2;
      u16_display_array[1] = Display_Table[index-1].u16_char3;
      u16_display_array[0] = Display_Table[index-1].u16_char4;                                   
   }
   //The 7 segment display are numbered 0-4 from left to right
   //the string needs to be set into diplay string at reverse order 
   //according to the proper length
   //check if number is less then 4 digit and erase the rest
   for (i = 2; i >= 0; --i)                             
   {                                                    
      if ( erase_flag == 1) 
         u16_display_array[i] = NONE; 
      if ( u16_display_array[i] == NONE )                                
         erase_flag = 1;                                                                               
   }                                                    
   //Display the message                                                      
   for (i = 0; i < CDHD_HMI_SIZE; ++i)
   {
      BGVAR(u16_Local_Hmi_Display_Array)[i] = u16_display_array[i];
      HWDisplay(u16_display_array[i],i);
   }
   return;
}
void DisplayRecentFault(int drive)
{
   int fault_index = 0; 
   REFERENCE_TO_DRIVE;
   ReadMostRecentFault(drive, &fault_index);
   HmiDisplayFault(fault_index);
}


void DisplayFPGAVer(int drive)
{
   char fpga_ver[128];
   REFERENCE_TO_DRIVE;
   PrintFpgaVersion(fpga_ver);
   HmiDisplayFiveLetterString(fpga_ver+u16_User_Editing_Msb_Lsb_Offset,5);
}



void DisplayFWVer(int drive)
{
   REFERENCE_TO_DRIVE;
   HmiDisplayFiveLetterString((char*)p_s8_CDHD_Drive_Version+u16_User_Editing_Msb_Lsb_Offset,5);
}


void DisplayFaultHistory(int drive)
{
   int index = BGVAR(s_Fault_Log_Image)[u16_Fault_History_Index].fault_id; 
   if ((BGVAR(s_Fault_Log_Image)[u16_Fault_History_Index].fault_id & 0x00FF) == 0xFF) u16_Fault_History_Index = 0;
   REFERENCE_TO_DRIVE;   
   HmiDisplayFault(index);
}



int GetCurrentIndexForExec(void)
{
   switch (u16_Current_Display_mode)
   {
      case(MENU_MODE_COMMANDS):return u16_Menu_Commands_Index_Arr[u16_Current_Commands_Menu_Item_Index];
      case(MENU_MODE_PARAMS):return u16_Menu_Params_Index_Arr[u16_Current_Params_Menu_Item_Index];
      case(MENU_MODE_MONITORS):return u16_Menu_Monitors_Index_Arr[u16_Current_Monitors_Menu_Item_Index];
      default : break;
   }
   return 0;  
}

long GetCurrentCommandMnemonic()
{
   return Commands_Table[GetCurrentIndexForExec()].u32_mnemonic_code1;
}
void DigitalInputsOutputsRepresentation(unsigned int* u16_display_array,unsigned int* u16_output_repr_array)
{
   unsigned int i = 0;
   //the u16_output_repr_array contains the state of each output
   //each 2 outputs are represented on a single 7 segment display
   //also the order of representation is switched since the 7 segments are ordered right to left
   //and the represantation is from left to right 
   //example input 1 and 2 are summed up into u16_display_array[4] 
   //there is also an offset to acount to, inputs numbered from 1 to... and the 7 segment starts at 0
   for (i = 0; i < CDHD_HMI_SIZE; ++i)
   {
      u16_display_array[i] = u16_output_repr_array[2*(CDHD_HMI_SIZE-i-1)] & u16_output_repr_array[2*(CDHD_HMI_SIZE-i)-1];
   }
   for (i = CDHD_HMI_SIZE; i < 2*CDHD_HMI_SIZE; ++i)
   {
      u16_display_array[i] = u16_output_repr_array[2*(CDHD_HMI_SIZE*2 -(i-4)*2)+1] & u16_output_repr_array[2*(CDHD_HMI_SIZE*2 -(i-2)*2+1)];
   }
   for (i = 0 ; i < CDHD_HMI_SIZE; ++i)
   {
      BGVAR(u16_Local_Hmi_Display_Array)[i] = u16_display_array[i];
      HWDisplay(u16_display_array[i],i);
   }
}
int CdhdHmiDisplayCurrentOutputs(void)
{
   unsigned int u16_display_array[2*CDHD_HMI_SIZE] = {0};
   unsigned int u16_output_repr_array[2*CDHD_HMI_SIZE] = {0};
   unsigned int i = 0;
   //output representation array fill up
   
   for (i = 1; i <= 2*CDHD_HMI_SIZE; ++i)
   {
      if ( i < s16_Num_Of_Outputs )
      {
         if ( u16_Output[i-1] == 1 )//output is on
         {
            if ( (i-1)%2 == 0 )//output is even number
            {
               u16_output_repr_array[i-1] = EVEN_NUMBER_OUTPUT_ON; //e+f
            }
            else//output is odd number
            {
               u16_output_repr_array[i-1] = ODD_NUMBER_OUTPUT_ON; //c+b
            }
         }
         else//output is off
         {
            if ( (i-1)%2 == 0 )//output number is even 
            {
               u16_output_repr_array[i-1] = EVEN_NUMBER_OUTPUT_OFF; //e
            }
            else//output number is odd 
            {
               u16_output_repr_array[i-1] = ODD_NUMBER_OUTPUT_OFF; //c
            }
         }
      }
      else
      {
         u16_output_repr_array[i-1] = NONE;//off
      }
   }
   //summing of two outputs (even number & odd number) representations into display array 
   DigitalInputsOutputsRepresentation(u16_display_array,u16_output_repr_array);
   return SAL_SUCCESS;
}

int CdhdHmiDisplayCurrentInputs(void)
{
   unsigned int u16_display_array[2*CDHD_HMI_SIZE] = {0};
   unsigned int u16_inputs_repr_array[2*CDHD_HMI_SIZE] = {0};
   unsigned int i = 0;
   //inputs representation array fill up
   for (i = 1; i <= 2*CDHD_HMI_SIZE; ++i)//TODO num of inputs not 2*HMI_SIZE
   {
      if ( i <= s16_Num_Of_Inputs )
      {
         if ( InxValue(i,0) )//input is on
         {
            if ( (i-1)%2 == 0 )//input is even number
            {
               u16_inputs_repr_array[i-1] = EVEN_NUMBER_OUTPUT_ON; //e+f
            }
            else//input is odd number
            {
               u16_inputs_repr_array[i-1] = ODD_NUMBER_OUTPUT_ON; //c+b
            }
         }
         else//input is off
         {
            if ( (i-1)%2 == 0 )//input number is even 
            {
               u16_inputs_repr_array[i-1] = EVEN_NUMBER_OUTPUT_OFF; //e
            }
            else//input number is odd 
            {
               u16_inputs_repr_array[i-1] = ODD_NUMBER_OUTPUT_OFF; //c
            }
         }
      }
   }
   DigitalInputsOutputsRepresentation(u16_display_array,u16_inputs_repr_array);
   return SAL_SUCCESS;
}
int CdhdHmiDisplayCurrentInputsEx(void)
{
   unsigned int u16_display_array[CDHD_HMI_SIZE] = {0};
   unsigned int u16_inputs_repr_array[2] ={0};
   unsigned int i = 0;
   if ( InxValue(11,0) )//input is on
   {
      u16_inputs_repr_array[0] = EVEN_NUMBER_OUTPUT_ON; //e+f
   }
   else//input is off
   {
      u16_inputs_repr_array[0] = EVEN_NUMBER_OUTPUT_OFF; //e       
   }
   u16_inputs_repr_array[1] = NONE; 
   u16_display_array[4] = u16_inputs_repr_array[0] & u16_inputs_repr_array[1];
   u16_display_array[3] = NONE;
   u16_display_array[2] = NONE;
   u16_display_array[1] = NONE;
   u16_display_array[0] = NONE;
   for (i = 0 ; i < CDHD_HMI_SIZE; ++i)
   {
      BGVAR(u16_Local_Hmi_Display_Array)[i] = u16_display_array[i];
      HWDisplay(u16_display_array[i],i);
   }
   return SAL_SUCCESS;
}


void DisplayCurrentCommandName(int drive)
{
   long u16_current_command_mnemonic = GetCurrentCommandMnemonic();
   if ( u16_Current_Display_mode == MENU_MODE_COMMANDS )
   {
      switch (u16_current_command_mnemonic)
      {
         case(0x1B):HmiDisplayFiveLetterString("Jog  ",5);break;
         case(0x1956599F): HmiDisplayFiveLetterString("Atune",5); break;
         case(0x1481F5DA): HmiDisplayFiveLetterString("confg",5); break;
         case(0x1E825823): HmiDisplayFiveLetterString("nnSet",5);break;
         case(0x1981E594): HmiDisplayFiveLetterString("honne",5);break;
         case(0x1E827592): HmiDisplayFiveLetterString("abSnn",5);break;
         case(0x1E82759A): HmiDisplayFiveLetterString("1ncnn",5);break;
         case(0x21652916): HmiDisplayFiveLetterString("PhaSe",5);break;
         case(0x17494960): HmiDisplayFiveLetterString("FrStr",5);break;
         case(0x2469F69F): HmiDisplayFiveLetterString("S1n1n",5);break;
         case(0x9129D6): DisplayMsbLsb(u16_User_Editing_Msb_Lsb_Offset, CDHD_HMI_SIZE);break;//save
         case(0x147564A3):DisplayMsbLsb(u16_User_Editing_Msb_Lsb_Offset, CDHD_HMI_SIZE);break;//clearfaults
         default:
            HmiExecuteReadCommand(GetCurrentIndexForExec(),drive, &s64_User_Entered_Value);
            PrepLongLongForDisplay(drive,&s64_User_Entered_Value);
            Conv20DigLongLongToArr(u16_Display_Array_Msb_Lsb,&s64_User_Entered_Value);
            DisplayMsbLsb(u16_User_Editing_Msb_Lsb_Offset, CDHD_HMI_SIZE);
            break;
      }
   }
}

void ResetUserEditingParams(void) 
{
   int i = 0;
   for (i=0; i<4*CDHD_HMI_SIZE;++i)
   {
      u16_Display_Array_Msb_Lsb[i] = 0;
   }
   if ( u16_User_Editing_Mode == VALUE_INSERTED || u16_User_Editing_Mode == VIEW_INDEX_NUMBER)
   {
      u16_Display_Neg_Val_Flag = 0;
      s64_User_Entered_Value = 0;
      BGVAR(u8_Local_Hmi_Blink_Display) = 0;
      u16_Hmi_Commands_Steps_Counter = 0;
      u16_Execute_Continuous_Command = 0;
   }
   if ( u16_Saved_Ret_Val_Test >= 1 )
   {
      u16_Execute_Continuous_Command = 0;
      s64_User_Entered_Value = 0;
   }
   if ( u16_Hmi_Jog_Flag == 1 )
   {
      u16_Hmi_Jog_Flag = 0;
      u16_Execute_Continuous_Command = 0;
   }
   if (u16_Current_Display_mode == MENU_MODE_FAULTS && u16_Fault_Menu_Mode == FAULT_MENU_MODE_RECENT )
   {
      u16_Hmi_Flt_Flag_1 = 0;
      u16_Hmi_Flt_Flag_2 = 0;
   }
   u16_User_Editing_Mode = VIEW_INDEX_NUMBER;
   u16_User_Editing_Msb_Lsb_Offset = 0;
   u16_User_Digit_Edit = 0;
   u16_Saved_Ret_Val_Test = 0;
   u16_Display_Dec_Val_Flag = 0; 
   BGVAR(u8_Local_Hmi_Blink_Display) = 0;
   u16_Warning_Index = 0;  
}

void DisplayDone(void)
{
   HmiDisplayFiveLetterString("done ",5);
}
void DisplayAutoTuneProgress(int drive)
{
   long long progress = (long long)0;
   drive+=0;
   HdTuneProgressBarCommand(&progress, drive);
   if ( (progress != 100) && (u16_Execute_Continuous_Command == 1) && (u16_Saved_Ret_Val_Test != VALUE_OUT_OF_RANGE) )
   {
      Conv20DigLongLongToArr(u16_Display_Array_Msb_Lsb,&progress);
      DisplayMsbLsb(0,2);
   }
   else if ( progress == 100 )
   {
      DisplayDone();
      return;
   }
   else if ( u16_Saved_Ret_Val_Test > SAL_SUCCESS )
   {
      HmiDisplayRetVal(drive,u16_Saved_Ret_Val_Test,'E');
      //u16_Execute_Continuous_Command = 0;
      return;
   }
   else 
   {
      HmiDisplayRetVal(drive,u16_Saved_Ret_Val_Test,'E');
      //u16_Execute_Continuous_Command = 0;
      return;
   }
}
void DisplayHomeProgress(int drive)
{
   long long progress = BGVAR(u8_Homing_State); 
   drive +=0;
  
   if ( progress == HOMING_TARGET_REACHED && u16_Hmi_Move_Home_Flag == 1)
   {
      DisplayDone();
      return;
   }
   else if ( progress < HOMING_TARGET_REACHED )
   {
      u16_Hmi_Move_Home_Flag = 1;
      progress = progress*(long long)100;
      progress = progress/(long long)HOMING_TARGET_REACHED;
      Conv20DigLongLongToArr(u16_Display_Array_Msb_Lsb,(long long*)&progress);
      DisplayMsbLsb(0,2);
      return;
   }
   
   else if ( u16_Saved_Ret_Val_Test > SAL_NOT_FINISHED )
   {
      HmiDisplayRetVal(drive,u16_Saved_Ret_Val_Test,'E');
      return;
   }
}

void DisplayConfigProgress(int drive)
{
   long long progress = (long long)50;
   drive+=0;
   
   if ( u16_Saved_Ret_Val_Test == SAL_NOT_FINISHED )
   {
      Conv20DigLongLongToArr(u16_Display_Array_Msb_Lsb,&progress);
      DisplayMsbLsb(0,2);
   }
   else if ( u16_Saved_Ret_Val_Test == SAL_SUCCESS )
   {
      DisplayDone();
      u16_Execute_Continuous_Command = 0;
   }
   else if ( u16_Saved_Ret_Val_Test > SAL_NOT_FINISHED )
   {
      HmiDisplayRetVal(drive,u16_Saved_Ret_Val_Test,'E');
      u16_Execute_Continuous_Command = 0;
      return;
   }
}
void DisplayJogProgress(int drive)
{
   // AXIS_OFF;
   drive += 0;
   if (0 != VAR(AX0_s16_Stopped))HmiDisplayFiveLetterString("Jog S",5);
   else HmiDisplayFiveLetterString("Jog r",5);
}
void DisplayMotorSetupProgress(int drive)
{
   long long progress = (long long)BGVAR(u16_Motor_Setup_State);
   drive +=0;
   if ( BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_ACTIVE > 0 && progress > 0/* progress < 47 && progress > 0*/ )//TODE replace with define 
   {
      progress = progress*(long long)100;
      progress = progress/(long long)MOTOR_SETUP_WAIT_FOR_CONFIG_END;
      Conv20DigLongLongToArr(u16_Display_Array_Msb_Lsb,(long long*)&progress);
      DisplayMsbLsb(0,2);
      return;
   }
   else if ( (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_FAILED) || (u16_Saved_Ret_Val_Test > SAL_SUCCESS+1) )
   {
      HmiDisplayRetVal(drive,u16_Saved_Ret_Val_Test,'E');
      u16_Execute_Continuous_Command = 0;
      return;
   }
   else if (u16_Saved_Ret_Val_Test == SAL_SUCCESS ) 
   {
      DisplayDone();
      return;
   }
   
}
void DisplayMoveProgress(int drive,char* move_string)
{
   // AXIS_OFF;
   drive += 0;
   
   if ( VAR(AX0_u16_Motor_Stopped) == 2 && (u16_Hmi_Commands_Steps_Counter == 0))
   {
      DisplayDone();
   }
   else HmiDisplayFiveLetterString(move_string,5);
}

void DisplayPhaseFindProgress(int drive)
{   
   if ( (BGVAR(u16_PhaseFind_Bits) == 1) )
   {
      HmiDisplayFiveLetterString("PhaSe",5);
   }
   else if ( (BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_SUCCESS_MASK) == 0 )//if( u16_Saved_Ret_Val_Test == 0 && u16_Hmi_Commands_Steps_Counter == 0 )
   {
      DisplayDone();
      return;
   } 
   else if ( u16_Saved_Ret_Val_Test > SAL_NOT_FINISHED )
   {
      HmiDisplayRetVal(drive,u16_Saved_Ret_Val_Test,'E');
      u16_Execute_Continuous_Command = 0;
      return;
   }
}

void DisplayFactoryRestoreProgress(int drive)
{
   if ( s64_User_Entered_Value == 1111LL )
   {
      u16_Execute_Continuous_Command = 1;
      s64_Execution_Parameter[0] = 0;
      s16_Number_Of_Parameters = 0;
      u16_Saved_Ret_Val_Test =  SalRestoreFactorySettingsCommand(drive);
   }
   else
   {
      u16_Saved_Ret_Val_Test = VALUE_OUT_OF_RANGE;
   }
   
   if ( u16_Saved_Ret_Val_Test == SAL_NOT_FINISHED )  
      HmiDisplayFiveLetterString("-----",5);
   if ( u16_Saved_Ret_Val_Test == SAL_SUCCESS )
   {
      u16_Execute_Continuous_Command = 0;
      DisplayDone();
      u16_User_Editing_Mode = VALUE_INSERTED;
   }
   else if ( u16_Saved_Ret_Val_Test > SAL_SUCCESS )
   {
      HmiDisplayRetVal(drive,u16_Saved_Ret_Val_Test,'E');
      u16_Execute_Continuous_Command = 0;
   }
   
}
void  DisplaySininitProgress(int drive)
{
   long long progress = 0;
   SalReadSinInitStatusCommand(&progress, drive);
   drive +=0; 
   if ( progress == 1LL )
   {
      Conv20DigLongLongToArr(u16_Display_Array_Msb_Lsb,(long long*)&progress);
      DisplayMsbLsb(0,2);
   }
   else if ( u16_Saved_Ret_Val_Test == SAL_SUCCESS )
   {
      DisplayDone();
      u16_Execute_Continuous_Command = 0;
   } 
   else
   {
      HmiDisplayRetVal(drive,u16_Saved_Ret_Val_Test,'E');
      u16_Execute_Continuous_Command = 0;
   }

}

void DisplaySaveProgress(int drive)
{
   
   if ( u16_Saved_Ret_Val_Test == SAL_NOT_FINISHED && s16_Hmi_Save_To_Flash_Flag == 1)
   {
      u16_Saved_Ret_Val_Test = SalSaveToFlash(drive);
   } 
   else if ((u16_Saved_Ret_Val_Test == SAL_SUCCESS || u16_Saved_Ret_Val_Test == NO_ERROR) || s16_Hmi_Save_To_Flash_Flag == 0)
   {
      DisplayDone();
      u16_Execute_Continuous_Command = 0;
      s16_Hmi_Save_To_Flash_Flag = 0;
   }

}

void DisplayCommandProgressBar(int drive)
{
   long s32_menu_commands_mnemonic = GetCurrentCommandMnemonic();
   BGVAR(u8_Local_Hmi_Blink_Display) = 0;
   switch (s32_menu_commands_mnemonic)
   {
      case(0x1956599F): DisplayAutoTuneProgress(drive); break;
      case(0x1E825823): DisplayMotorSetupProgress(drive); break;
      case(0x1481F5DA): DisplayConfigProgress(drive); break;
      case(0x1981E594): DisplayHomeProgress(drive); break;
      case(0x1E827592): DisplayMoveProgress(drive,"abSnn"); break;
      case(0x1E82759A): DisplayMoveProgress(drive,"1ncnn"); break;
      case(0x21652916): DisplayPhaseFindProgress(drive); break;
      case(0x17494960): DisplayFactoryRestoreProgress(drive); break;
      case(0x2469F69F): DisplaySininitProgress(drive);break;
      case(0x1B): DisplayJogProgress(drive); break;
      case(0x9129D6): DisplaySaveProgress(drive);break;
      default: 
      break;
   }  
}
void HandleMonitorItemDisplay(int drive)
{ 
   long s32_current_mnemomic = GetCurrentCommandMnemonic();
   int  s16_current_index = GetCurrentIndexForExec();
   //can status and control  words
   //check the value of current mnemomic for CANSTATUSWORD & CANCONTROLWORD
   if ( ((s32_current_mnemomic == 0x1449F925) || (s32_current_mnemomic == 0x1449F520)) )//TODO add comment what is this numbers
   { 
      HmiExecuteReadCommand(s16_current_index,drive, &s64_User_Entered_Value);
      IntToHex(s64_User_Entered_Value, s8_Hex_Str_For_Display );
      s8_Hex_Str_For_Display[0] = 'h';
      HmiDisplayFiveLetterString(s8_Hex_Str_For_Display,5);
   }
   else if ( (Commands_Table[s16_current_index].u16_hmi_group_index == 16388)  )
   { 
     CdhdHmiDisplayCurrentInputs();
   }
   else if ( (Commands_Table[s16_current_index].u16_hmi_group_index == 16389)  )
   { 
     CdhdHmiDisplayCurrentInputsEx();
   }
   else if ( (Commands_Table[s16_current_index].u16_hmi_group_index == 16390)  )
   { 
     CdhdHmiDisplayCurrentOutputs();
   }
   else
   {
      HmiExecuteReadCommand(s16_current_index,drive, &s64_User_Entered_Value);
      PrepLongLongForDisplay(drive,&s64_User_Entered_Value);
      DisplayMsbLsb(u16_User_Editing_Msb_Lsb_Offset, CDHD_HMI_SIZE);
   }
}
//This function returns the string of warning for display for the faults menu mode
int GetWarningStringForDisplay(int drive, unsigned long long warning_id, char* wrn_str)
{
   drive+=0;
   switch (warning_id)
   {
      case UNDER_VOLTAGE_WRN_MASK: // u
         wrn_str[1] = 'u';
      break;

      case DRIVE_FOLDBACK_WRN_MASK: // F
      case MOTOR_FOLDBACK_WRN_MASK: // F
         wrn_str[1] = 'F';
      break;

      case STO_WRN_MASK: // n
         wrn_str[1] = 'n';
         wrn_str[2] = ' ';
      break;

      case MOTOR_OT_WRN_MASK: // H
         wrn_str[1] = 'H';
      break;

      case DRIVE_POW_OT_WRN_MASK: // t
      case DRIVE_DIG_OT_WRN_MASK:
      case DRIVE_IPM_OT_WRN_MASK:
         wrn_str[1] = 't';
      break;

     case TM_BATT_LOW_VOLTAGE_WRN: // b
        wrn_str[1] = 'b';
      break;

     case CW_HW_LS_WRN_MASK: // L1
         wrn_str[1] = 'L';
         wrn_str[2] = '1';
      break;

      case CCW_HW_LS_WRN_MASK: // L2
         wrn_str[1] = 'L';
         wrn_str[2] = '2';
      break;

      case CW_CCW_HW_LS_WRN_MASK: // L3
         wrn_str[1] = 'L';
         wrn_str[2] = '3';
      break;

      case CW_SW_LS_WRN_MASK: // L4
         wrn_str[1] = 'L';
         wrn_str[2] = '4';
      break;

      case CCW_SW_LS_WRN_MASK: // L5
         wrn_str[1] = 'L';
         wrn_str[2] = '5';
      break;

      case CW_CCW_SW_LS_WRN_MASK: // L6
         wrn_str[1] = 'L';
         wrn_str[2] = '6';
      break;

      case SININIT_WARNING_MASK: // r
         wrn_str[1] = 'r';
      break;

      case BUS_AC_LOSS_WRN_MASK: // o
         wrn_str[1] = 'o';
      break;

      case REGEN_OVER_LOAD_WRN_MASK: // c
         wrn_str[1] = 'c';
      break;

      case RT_OVERLOAD_WRN_MASK: // _
         wrn_str[1] = '_';
      break;

      case SFB_MODE_EXCLUDE_WRN_MASK: // S1
         wrn_str[1] = 'S';
         wrn_str[2] = '1';
      break;
      
      case PHASE_FIND_REQ_WRN_MASK: // PF
         wrn_str[1] = 'P';
         wrn_str[2] = 'F';
      break;
      default:
      return 0;
   }

   return 1;

}

void GetNextWarningIndex(void)
{
   int i = 0;
   for (i = u16_Warning_Index; i < 64; i++)
   {
      if (BGVAR(u64_Sys_Warnings) & (((unsigned long long)0x1 << i)))
      {
        u16_Warning_Index = i;
      }
   }
}
void GetPrevWarningIndex(void)
{
   int i = 0;
   for (i = u16_Warning_Index; i >= 0; i--)
   {
      if (BGVAR(u64_Sys_Warnings) & (((unsigned long long)0x1 << i)))
      {
         u16_Warning_Index = i;
      }
   }
}
//This function display the warnings for the faults menu mode
int DisplayRecentWarning(int drive)
{
   char wrn_str[5];
   int i = 0;
   REFERENCE_TO_DRIVE;
   for (i = 0; i <= 4; ++i)
   {
      wrn_str[i] = ' ';
   }
   wrn_str[0] = 'F';
   if ( BGVAR(u64_Sys_Warnings) != 0 )
   {
      GetWarningStringForDisplay(drive, ((unsigned long long)0x1 << BGVAR(u16_Warning_Index)),wrn_str);
   }
   //fix bug wrong display of STO warning when entering warning display submenu
   if ( !(BGVAR(u64_Sys_Warnings) & STO_WRN_MASK) && BGVAR(u16_Warning_Index) == 0 )
   {
      for (i = 0; i <= 4; ++i)
      {
         wrn_str[i] = ' ';
      }
      wrn_str[0] = 'F';   
   }
   HmiDisplayFiveLetterString(wrn_str,5);
   return 0;   
}

void HandleFaultsDisplay(int drive)
{
   switch(u16_Fault_Menu_Mode)
   {
   case(FAULT_MENU_MODE_RECENT):DisplayRecentFault(drive);
      break;
   case(FAULT_MENU_MODE_HISTORY):DisplayFaultHistory(drive);
      break;
   case(FAULT_MENU_MODE_FW_VER):DisplayFWVer(drive);
      break;
   case(FAULT_MENU_MODE_FPGA_VER):DisplayFPGAVer(drive);
      break;
   case(FAULT_MENU_MODE_RECENT_WRN):DisplayRecentWarning(drive);
      break;
   default: 
      break;
   }         
}
void HmiFaultDisplayOverwrite(void)
{
   //check if new faults occured (other than "config needed")
   if ((s64_Flt_Prev_1 != (BGVAR(s64_SysNotOk) & ~NO_COMP_FLT_MASK)) && 
      ((BGVAR(s64_SysNotOk) & ~NO_COMP_FLT_MASK) != 0))
   {
      //if so update prev faults status and raise flag
      u16_Hmi_Flt_Flag_1 = 1;
      s64_Flt_Prev_1 = (BGVAR(s64_SysNotOk) & ~NO_COMP_FLT_MASK);
   }
   if (((s64_Flt_Prev_2 != BGVAR(s64_SysNotOk_2))  &&  
      (BGVAR(s64_SysNotOk_2)!= 0)))
   {
      s64_Flt_Prev_2 = BGVAR(s64_SysNotOk_2);
      u16_Hmi_Flt_Flag_2 = 1;
   }

   //if display was taken by fault and clearfaults was issued release display 
   if (!(BGVAR(s64_SysNotOk) & ~NO_COMP_FLT_MASK) && !BGVAR(s64_SysNotOk_2) && (u16_Hmi_Flt_Flag_1 == 2 || u16_Hmi_Flt_Flag_2 == 2))
   {
      u16_Hmi_Flt_Flag_1 = 0;
      u16_Hmi_Flt_Flag_2 = 0;
      s64_Flt_Prev_1 = 0;
      s64_Flt_Prev_2 = 0;
      ResetUserEditingParams();
      u16_Current_Display_mode = MENU_MODE_DRIVE_STATUS;
   }
   //steal display 
   if ((u16_Hmi_Flt_Flag_1 == 1 || u16_Hmi_Flt_Flag_2 == 1) && 
      (u16_Current_Display_mode != MENU_MODE_PARAMS))
   {
      u16_Current_Display_mode = MENU_MODE_FAULTS;
      u16_Fault_Menu_Mode = FAULT_MENU_MODE_RECENT;
      u16_User_Editing_Mode = VIEW_INDEX_VALUE;  
      u16_Hmi_Flt_Flag_1 = 2;
      u16_Hmi_Flt_Flag_2 = 2;
      BGVAR(u8_Local_Hmi_Blink_Display) = 0;  
   }
}



void HmiDisplayManager(int drive)
{
   REFERENCE_TO_DRIVE;
   //If fault happens "steal" display
   if ( BGVAR(s64_SysNotOk) & NO_COMP_FLT_MASK && u16_User_Editing_Mode == VALUE_INSERTED ) BGVAR(u8_Local_Hmi_Blink_Display) = 0xFF ;
   //if test all leds
   if (u8_Test_Hmi)
   {
      BlinkAllSegments();
      return;   
   }
   if (u8_Test_Hmi_Seq)
   {
      TestIndividualSegmentsLed();
      return;   
   }
   HmiFaultDisplayOverwrite();

   if ( (((u16_Execute_Continuous_Command == 1) || (u16_Hmi_Commands_Steps_Counter != 0))&& 
          (u16_Current_Display_mode == MENU_MODE_COMMANDS)) ) 
   {  
      DisplayCommandProgressBar(drive);
      return;
   }
   if ( u16_Current_Display_mode == MENU_MODE_DRIVE_STATUS )
   {
      DisplayDriveStatus(drive);
      return;
   }
   if ( u16_User_Editing_Mode == VIEW_INDEX_NUMBER )            
   {                                                    
      DisplayMenuModeIndex();                   
      return;                                         
   } 
   if ( u16_User_Editing_Mode == VIEW_INDEX_VALUE )
   {
      switch (u16_Current_Display_mode)
      {
         case(MENU_MODE_FAULTS): HandleFaultsDisplay(drive); return;
         case(MENU_MODE_PARAMS):
         {
            u16_Saved_Ret_Val_Test = HmiExecuteReadCommand(GetCurrentIndexForExec(),drive, (long long*)&s64_User_Entered_Value);
            PrepLongLongForDisplay(drive,&s64_User_Entered_Value);
            DisplayMsbLsb(u16_User_Editing_Msb_Lsb_Offset, CDHD_HMI_SIZE);
            return;//
         }
         case(MENU_MODE_MONITORS):
         {
            HandleMonitorItemDisplay(drive);   
            return;
         }
      }
      if ( u16_Current_Display_mode == MENU_MODE_COMMANDS )
      {
         DisplayCurrentCommandName(drive);
         return;
      }
   }

   if ( ((u16_Current_Display_mode != MENU_MODE_DRIVE_STATUS) && (u16_User_Editing_Mode == EDIT_ENABLE)) )
   {
      PrepLongLongForDisplay(drive,&s64_User_Entered_Value);
      DisplayMsbLsb(u16_User_Editing_Msb_Lsb_Offset, CDHD_HMI_SIZE);
      BGVAR(u8_Local_Hmi_Blink_Display) = u16_User_Digit_Edit - u16_User_Editing_Msb_Lsb_Offset +1;
      return;
   }
   if ( ((u16_Current_Display_mode != MENU_MODE_DRIVE_STATUS) /*&& (u16_Current_Display_mode != MENU_MODE_COMMANDS)*/ && (u16_User_Editing_Mode == VALUE_INSERTED) && (u16_Execute_Continuous_Command == 0)) )
   {
      HmiDisplayRetVal(drive,u16_Saved_Ret_Val_Test,'E');
      BGVAR(u8_Local_Hmi_Blink_Display) = 0; 
   }
   return;
}

void DisplayModeSwitch(void)
{
   
   if ( u16_User_Editing_Mode != VALUE_INSERTED )
   {
      u16_Current_Display_mode = (u16_Current_Display_mode+1)%MENU_NUMBER_OF_DISPLAY_MODES;
   }

   ResetUserEditingParams();
}

void ConvertArrayToLongLong(unsigned int* arr,long long* output_value)
{
   unsigned int i = 0;
   long long s64_temp_value = 0;
   for (i = 0; i < 4*CDHD_HMI_SIZE; ++i)
   {
      s64_temp_value += arr[i]*llpow(10,i);
      if ( s64_temp_value > (llpow(10,18) - (long long)1) ) s64_temp_value = 0;
   }
   if ( u16_Display_Neg_Val_Flag == 1 ) s64_temp_value = s64_temp_value*(-1);
   *output_value = s64_temp_value;
}

void IndexUpdate(unsigned int* index, int direction, int limit,int multiplier)
{
   if ( ((direction == MENU_DIRECTION_DOWN) && (*index == 0)) )
   {
      *index = limit-1*multiplier;
   }
   else
   {
      *index = (*index+direction*multiplier)%(limit);
   }
}
 
void HmiPreMotorSetup(int drive)
{
   REFERENCE_TO_DRIVE;
   s64_Execution_Parameter[0] = s64_User_Entered_Value; 
   u16_Execute_Continuous_Command = 1;
   s16_Number_Of_Parameters = 0;
   u16_Hmi_Commands_Steps_Counter = 1;
}
void HmiMotorSetup(int drive)
{
   if ( s64_User_Entered_Value != 0 )
   {
      s64_Execution_Parameter[0] = s64_User_Entered_Value;
      s16_Number_Of_Parameters = 1;
   }
   else
   {
      s16_Number_Of_Parameters = 0;
   }
   u16_Saved_Ret_Val_Test = MotorSetupCommand(drive);
}


void HmiHandleMotorSetup(int drive)
{ 
   if ( u16_Hmi_Commands_Steps_Counter == 0 ) HmiPreMotorSetup(drive);
   if ( u16_Hmi_Commands_Steps_Counter == 200 ) HmiMotorSetup(drive);
   if ( u16_Hmi_Commands_Steps_Counter == 400 ) EnableCommand(drive); 
}



void HmiHandleJog(int drive)
{
   // AXIS_OFF;
   unsigned int index ;
   u16_Execute_Continuous_Command = 1;
   s16_Number_Of_Parameters = 1;
   u16_Hmi_Jog_Flag = 1;   
   if ( VAR(AX0_s16_Opmode) == 0 )
   {
      index = UnitsConversionIndex(&s8_Units_Vel_Out_Loop, drive);
      s64_User_Entered_Value =  (long long)(((s64_User_Entered_Value) * BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix)>> BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);
   }
}
void HandleJogRun(int drive,int direction)
{
   // AXIS_OFF;
   long long temp;
   if ( VAR(AX0_s16_Opmode) == 0 )
   {
      temp = direction*s64_User_Entered_Value*1;
      s64_Execution_Parameter[0] =  temp;
      if ( (0 != VAR(AX0_s16_Stopped)) )u16_Saved_Ret_Val_Test = SalJogCommand(drive);// ExecuteWriteCommand(GetCurrentIndexForExec(u16_Menu_Commands_Index_Arr,u16_Current_Commands_Menu_Item_Index), drive, &temp); 
   }
   if ( VAR(AX0_s16_Opmode) == 8 )
   {
      s64_Execution_Parameter[0] = direction*100000000;//position
      s64_Execution_Parameter[1] = s64_User_Entered_Value;//spd
      s16_Number_Of_Parameters = 2;
      if ( (0 != VAR(AX0_s16_Stopped)) ) u16_Saved_Ret_Val_Test = SalMoveIncCommand(drive);
   }
}

void HandleJogStop(int drive)
{
   // AXIS_OFF;
   long long temp = 0;
   if ( VAR(AX0_s16_Opmode) == 0 )JogCommand(temp, drive);
   if ( VAR(AX0_s16_Opmode) == 8 )StopCommand(drive);
}
void HmiHandleConfig(int drive)
{
   u16_Execute_Continuous_Command = 1;
   DisableCommand(drive);
   if ( s64_User_Entered_Value != 0 )
   {
      u16_Saved_Ret_Val_Test = VALUE_OUT_OF_RANGE; 
      u16_Execute_Continuous_Command = 0;
   
   }
   else
   {
      u16_Saved_Ret_Val_Test = SalConfigCommand(drive);    
   }
}


void HmiAtuneStepOne(int drive)
{
   u16_Execute_Continuous_Command = 1;
   s16_Number_Of_Parameters = 1;
   u16_Hmi_Commands_Steps_Counter = 1;
   u16_Execute_Continuous_Command = 1;
   switch (s64_User_Entered_Value)
   {
      case (0):
         SalHdTuneAvModeCommand(0LL,drive);
         PosTuneIgravEnCommand(0,drive);
         BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 0;
         PosTuneStiffnessCommand(1LL,drive);
         BGVAR(s16_Pos_Tune_Cycle) = 0;
         break;   
      case (1):
         SalHdTuneAvModeCommand(6LL,drive);
         PosTuneIgravEnCommand(0,drive);
         BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 2;
         PosTuneStiffnessCommand(1LL,drive);
         BGVAR(s16_Pos_Tune_Cycle) = 0;
         break;
      case (2):
         SalHdTuneAvModeCommand(6LL,drive);
         PosTuneIgravEnCommand(1,drive);
         BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 2;
         PosTuneStiffnessCommand(1LL,drive);
         BGVAR(s16_Pos_Tune_Cycle) = 0;
         break;
      case (10):
         SalHdTuneAvModeCommand(0LL,drive);
         PosTuneIgravEnCommand(0,drive);
         BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 0;
         PosTuneStiffnessCommand(1LL,drive);
         BGVAR(s16_Pos_Tune_Cycle) = 0;
         break;
      case (11):
         SalHdTuneAvModeCommand(6LL,drive);
         PosTuneIgravEnCommand(0,drive);
         BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 2;
         PosTuneStiffnessCommand(1LL,drive);
         BGVAR(s16_Pos_Tune_Cycle) = 0;
         break;
      case (12):
         SalHdTuneAvModeCommand(6LL,drive);
         PosTuneIgravEnCommand(1,drive);
         BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 2;
         PosTuneStiffnessCommand(1LL,drive);
         BGVAR(s16_Pos_Tune_Cycle) = 0;
         break;  
       case (13):
         SalHdTuneAvModeCommand(6LL,drive);
         PosTuneIgravEnCommand(0,drive);
         BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 2;
         PosTuneStiffnessCommand(0LL,drive);
         BGVAR(s16_Pos_Tune_Cycle) = 0;
         break;
       case (14):
         SalHdTuneAvModeCommand(6LL,drive);
         PosTuneIgravEnCommand(0,drive);
         BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 2;
         PosTuneStiffnessCommand(0LL,drive);
         BGVAR(s16_Pos_Tune_Cycle) = 0;
         break;
      default:
         u16_Saved_Ret_Val_Test = VALUE_OUT_OF_RANGE;
         break;
         
   }
}

void HmiAtuneStepTwo(int drive)
{ 
   u16_Execute_Continuous_Command = 1;
   s16_Number_Of_Parameters = 1;
   switch (s64_User_Entered_Value)
   {
      case (0):
         s64_Execution_Parameter[0] = 5;
         u16_Saved_Ret_Val_Test = PosTuneCommand(drive);
         break;   
      case (1):
      case (2):
         s64_Execution_Parameter[0] = 6;
         u16_Saved_Ret_Val_Test = PosTuneCommand(drive);
         break;
      case (10):
         s64_Execution_Parameter[0] = 7;
         u16_Saved_Ret_Val_Test = PosTuneCommand(drive);
         break;
      case (11):
      case (12):
      case (13):
      case (14):
         s64_Execution_Parameter[0] = 8;
         u16_Saved_Ret_Val_Test = PosTuneCommand(drive);
         break;
      default:
         u16_Saved_Ret_Val_Test = VALUE_OUT_OF_RANGE;
         break;

   }
   
}


void HmiHandleAtune(int drive)
{
   if ( u16_Hmi_Commands_Steps_Counter == 0 ) HmiAtuneStepOne(drive);
   if ( u16_Hmi_Commands_Steps_Counter == 200 ) HmiAtuneStepTwo(drive);
   if ( u16_Hmi_Commands_Steps_Counter == 400 ) EnableCommand(drive);

}

void HmiMoveAbsPhaseOne(int drive)
{
   REFERENCE_TO_DRIVE;
   u16_Hmi_Move_Home_Flag = 0;
   u16_Hmi_Commands_Steps_Counter = 1;
   u16_Execute_Continuous_Command = 1;
}

void HmiMoveAbsPhaseThree(int drive)
{
   int index  = UnitsConversionIndex(&s8_Units_Vel_Out_Loop, drive);
   u16_Hmi_Commands_Steps_Counter = 0;
   s64_Execution_Parameter[0] = s64_User_Entered_Value*1000;//position
   //spd
   s64_Execution_Parameter[1] = MultS64ByFixS64ToS64(BGVAR(s64_User_Jog_Vel),
                  BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_user_fix,
                  BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_user_shr);
   s16_Number_Of_Parameters = 2;
   u16_Saved_Ret_Val_Test = SalMoveAbsCommand(drive);       
}

void HmiHandleMoveAbs(int drive)
{
   if ( u16_Hmi_Commands_Steps_Counter == 0 ) HmiMoveAbsPhaseOne(drive);
   if ( u16_Hmi_Commands_Steps_Counter == 400 ) HmiMoveAbsPhaseThree(drive);
}

void HmiMoveIncPhaseThree(int drive)
{
   int index  = UnitsConversionIndex(&s8_Units_Vel_Out_Loop, drive);
   u16_Hmi_Commands_Steps_Counter = 0;
   s64_Execution_Parameter[0] = s64_User_Entered_Value*1000;//position
   s64_Execution_Parameter[1] = MultS64ByFixS64ToS64(BGVAR(s64_User_Jog_Vel),
                  BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_user_fix,
                  BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_user_shr);
   s16_Number_Of_Parameters = 2;
   u16_Saved_Ret_Val_Test = SalMoveIncCommand(drive);  
   
}


void HmiHandleMoveInc(int drive)
{
   if ( u16_Hmi_Commands_Steps_Counter == 0 ) HmiMoveAbsPhaseOne(drive);
   if ( u16_Hmi_Commands_Steps_Counter == 400 ) HmiMoveIncPhaseThree(drive);
}

void HmiPhaseFindOne(int drive)
{
   REFERENCE_TO_DRIVE;
   u16_Hmi_Commands_Steps_Counter = 1;
   u16_Execute_Continuous_Command = 1;
}

void HmiPhaseFindTwo(int drive)
{
   REFERENCE_TO_DRIVE;
   u16_Saved_Ret_Val_Test = PhaseFindCommand(drive);  
}

void HmiPhaseFindThree(int drive)
{
   REFERENCE_TO_DRIVE;
   u16_Hmi_Commands_Steps_Counter = 0;
   //EnableCommand(drive);    
}
void HmiHandlePhaseFind(drive)
{
   if ( u16_Hmi_Commands_Steps_Counter == 0 ) HmiPhaseFindOne(drive);
   if ( u16_Hmi_Commands_Steps_Counter == 200 ) HmiPhaseFindTwo(drive);
   if ( u16_Hmi_Commands_Steps_Counter == 600 ) HmiPhaseFindThree(drive);
}
void HmiHandleFactoryRestore(int drive)
{
   DisableCommand(drive);
   if ( s64_User_Entered_Value == 1111LL )
   {
      u16_Execute_Continuous_Command = 1;
      s64_Execution_Parameter[0] = 0;
      s16_Number_Of_Parameters = 0;
      u16_Saved_Ret_Val_Test =  SalRestoreFactorySettingsCommand(drive);
   }
   else
   {
      u16_Saved_Ret_Val_Test = SAL_NOT_FINISHED;
   }
   
}



void  HmiHandleSave(int drive)
{
   if ( u16_Hmi_Commands_Steps_Counter == 0 )
   {
      if (s64_User_Entered_Value == 0)
      {
         s64_Execution_Parameter[0] = 0;
         s16_Number_Of_Parameters = 0;
         u16_Hmi_Commands_Steps_Counter = 1;
         u16_Execute_Continuous_Command = 1;
         s16_Hmi_Save_To_Flash_Flag = 1;
         u16_Saved_Ret_Val_Test = SalSaveToFlash(drive);
      }
      else
      {
         u16_Saved_Ret_Val_Test = VALUE_OUT_OF_RANGE;
      }
   }
}
void HmiHandleClearfaults(int drive)
{
   if (s64_User_Entered_Value == 0)
   {
      ClearFaultsCommand(drive);
      u16_User_Editing_Mode = VIEW_INDEX_NUMBER;
      ResetUserEditingParams();
   }
   else
   {
      u16_Saved_Ret_Val_Test = VALUE_OUT_OF_RANGE;
   }
}

void HmiMoveHomeThree(int drive)
{
   u16_Hmi_Commands_Steps_Counter = 0;
   s16_Number_Of_Parameters = 0;
   u16_Hmi_Move_Home_Flag = 0;
   u16_Saved_Ret_Val_Test = HomeCommand(drive);  
   u16_Execute_Continuous_Command = 1;
}
void HmiHandleMoveHome(int drive)
{
   if ( u16_Hmi_Commands_Steps_Counter == 0 ) HmiMoveAbsPhaseOne(drive);
   if ( u16_Hmi_Commands_Steps_Counter == 600 ) HmiMoveHomeThree(drive);
}
int CallContinuousComandHandler(int drive)
{
   long s32_menu_commands_mnemonic = GetCurrentCommandMnemonic();
   switch (s32_menu_commands_mnemonic)
   {
      case(0x1B): HmiHandleJog(drive); 
         break;
      case(0x1956599F): HmiHandleAtune(drive); 
         break;
      case(0x1E825823): HmiHandleMotorSetup(drive); 
         break;
      case(0x1481F5DA): HmiHandleConfig(drive); 
         break;
      case(0x1E827592): HmiHandleMoveAbs(drive);
         break;
      case(0x1E82759A): HmiHandleMoveInc(drive);
         break;
      case(0x1981E594): HmiHandleMoveHome(drive); 
         break;
      case(0x21652916): HmiHandlePhaseFind(drive);
         break;
      case(0x17494960): HmiHandleFactoryRestore(drive);
         break;
      case(0x9129D6): HmiHandleSave(drive);
         break;
      case(0x147564A3): HmiHandleClearfaults(drive);
         break;
      case(0x2469F69F): u16_Execute_Continuous_Command = 1;
         //dont break
      default: 
         if (s64_User_Entered_Value == 0 || s64_User_Entered_Value == 1) u16_Saved_Ret_Val_Test = HmiExecuteReadCommand(GetCurrentIndexForExec(), drive, &s64_User_Entered_Value);      
            else u16_Saved_Ret_Val_Test = VALUE_OUT_OF_RANGE;
         PrepLongLongForDisplay(drive,&s64_User_Entered_Value);
         Conv20DigLongLongToArr(u16_Display_Array_Msb_Lsb,&s64_User_Entered_Value);
         DisplayMsbLsb(u16_User_Editing_Msb_Lsb_Offset, CDHD_HMI_SIZE);
         u16_User_Editing_Mode =  VALUE_INSERTED;//VALUE_INSERTED
         BGVAR(u8_Local_Hmi_Blink_Display) = 0;
         break;
   }     
   return u16_Saved_Ret_Val_Test;
}

void HandleShiftKeyRisingEdge(int drive)
{
   REFERENCE_TO_DRIVE;
   //user edit digit 
   if ( ((u16_Current_Display_mode > MENU_MODE_DRIVE_STATUS) && (u16_User_Editing_Mode == EDIT_ENABLE)) )
   {
      u16_User_Digit_Edit = (u16_User_Digit_Edit+1)%20;
      u16_User_Editing_Msb_Lsb_Offset = u16_User_Digit_Edit/5;
      u16_User_Editing_Msb_Lsb_Offset = u16_User_Editing_Msb_Lsb_Offset*5; 
   }
   //view value or view return value
   if ( (u16_Current_Display_mode != MENU_MODE_DRIVE_STATUS) && ((u16_User_Editing_Mode == VIEW_INDEX_VALUE)||(u16_User_Editing_Mode == VALUE_INSERTED)) )
   {
      u16_User_Editing_Msb_Lsb_Offset += 5;
      if ( u16_User_Editing_Msb_Lsb_Offset > 15 ) u16_User_Editing_Msb_Lsb_Offset = 0;
   }
   if ( ((u16_Current_Display_mode == MENU_MODE_MONITORS) || (u16_Current_Display_mode == MENU_MODE_FAULTS)))
   {
      u16_User_Editing_Mode = VIEW_INDEX_VALUE;
   }
   else if ( ((u16_Current_Display_mode == MENU_MODE_COMMANDS)||(u16_Current_Display_mode == MENU_MODE_PARAMS)) )
   {
      switch (u16_User_Editing_Mode)
      {
         case(VIEW_INDEX_NUMBER):u16_User_Editing_Mode = VIEW_INDEX_VALUE; 
            break;
         case(VIEW_INDEX_VALUE):
         {
            u16_User_Editing_Mode = EDIT_ENABLE;
            u16_User_Editing_Msb_Lsb_Offset = 0;
            break;
         }
         case(VALUE_INSERTED):ResetUserEditingParams();
            break;
         default: 
            break;
      }
   }   
}

void HandleLongShiftPress(int drive)
{
   int index = -1;
   REFERENCE_TO_DRIVE;
   if ( ((u16_Current_Display_mode == MENU_MODE_MONITORS) || (u16_Current_Display_mode == MENU_MODE_FAULTS)))
   {
      u16_User_Editing_Mode = VIEW_INDEX_VALUE;
   }
   else if ( ((u16_Current_Display_mode == MENU_MODE_COMMANDS)||(u16_Current_Display_mode == MENU_MODE_PARAMS)) )
   {
      if (EDIT_ENABLE == u16_User_Editing_Mode)
      {
         ConvertArrayToLongLong(u16_Display_Array_Msb_Lsb,&s64_User_Entered_Value);
         u16_User_Editing_Mode = VALUE_INSERTED;
         if ( u16_Current_Display_mode == MENU_MODE_COMMANDS )
         {
            CallContinuousComandHandler(drive); 
         }
         else if ( u16_Current_Display_mode == MENU_MODE_PARAMS )
         {
            if (u16_Display_Neg_Val_Flag == 1) s64_User_Entered_Value = s64_User_Entered_Value*(-1);
            if (WRITE_TYPE(Commands_Table[GetCurrentIndexForExec()].u8_rd_wr_method) == WR_FCUST)
            {
               if (Commands_Table[GetCurrentIndexForExec()].str_units_var != 0)   // Handle units conversion
               {
                  index = UnitsConversionIndex(Commands_Table[GetCurrentIndexForExec()].str_units_var, drive);
            
                  if (index >= 0) // Unit conversion found
                  {
                     s64_Execution_Parameter[0] = s64_User_Entered_Value;
                     s64_Execution_Parameter[0] =   MultS64ByFixS64ToS64(s64_Execution_Parameter[0],
                                                BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                                                BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr); 
                  }
               }
            }
            s64_Execution_Parameter[0] = s64_User_Entered_Value;
            s16_Number_Of_Parameters = 1;
            
            if ((s64_User_Entered_Value > Commands_Table[GetCurrentIndexForExec()].s64_max_value) || 
               (s64_User_Entered_Value < Commands_Table[GetCurrentIndexForExec()].s64_min_value))
            {
               u16_Saved_Ret_Val_Test = VALUE_OUT_OF_RANGE;
            }
            else
            {
               u16_Saved_Ret_Val_Test = ExecuteWriteCommand(GetCurrentIndexForExec(), drive, &s64_Execution_Parameter[0]);   
            }
         }
      }
   }
}

int IsArrayZero(unsigned int* arr,int arr_size)
{
   int i = 0;
   for (i = 0; i < arr_size; ++i)
   {
      if (arr[i] != 0) return 0;
   }
   return 1;
}
void HandleDownAndUpKeyRisingEdge(int drive,int direction)
{
   REFERENCE_TO_DRIVE;
   if ( u16_User_Editing_Mode == VALUE_INSERTED )
   {
      ResetUserEditingParams();
   }
   if ( u16_User_Editing_Mode == VIEW_INDEX_NUMBER )
   {
      switch (u16_Current_Display_mode)
      {
         case(MENU_MODE_COMMANDS):
            IndexUpdate(&u16_Current_Commands_Menu_Item_Index, direction, u16_Hmi_Commands_Group_Size,1);
            break;
         case(MENU_MODE_MONITORS):
            IndexUpdate(&u16_Current_Monitors_Menu_Item_Index, direction, u16_Hmi_Monitors_Group_Size,1);
            break;
         case(MENU_MODE_PARAMS):
            IndexUpdate(&u16_Current_Params_Menu_Item_Index, direction, u16_Hmi_Params_Group_Size,1);
            break;
         case(MENU_MODE_FAULTS):
            IndexUpdate(&u16_Fault_Menu_Mode, direction, MENU_FAULTS_SIZE,1);
            break;

         default: 
            break;
      }
   }
   else if ( u16_User_Editing_Mode == VIEW_INDEX_VALUE )
   {
      if ( u16_Current_Display_mode == MENU_MODE_FAULTS && (u16_Fault_Menu_Mode == FAULT_MENU_MODE_HISTORY) ) IndexUpdate(&u16_Fault_History_Index, direction, FAULT_LOG_LEN, 1); 
      else if ( (u16_Current_Display_mode == MENU_MODE_FAULTS) && (u16_Fault_Menu_Mode == FAULT_MENU_MODE_RECENT_WRN) )
      {
         if ( direction == MENU_DIRECTION_UP )
         GetNextWarningIndex();
      else
         GetPrevWarningIndex();
      }   
   }

   else if ( ((u16_Current_Display_mode == MENU_MODE_FAULTS) && ((u16_Fault_Menu_Mode > 1) && (u16_Fault_Menu_Mode < FAULT_MENU_MODE_RECENT_WRN))) )
   {
      if ( u16_Fault_Menu_Mode > 0 )
      {
         IndexUpdate(&u16_User_Editing_Msb_Lsb_Offset, direction, 3*CDHD_HMI_SIZE,5);
      }
   }
   else if ( ((u16_Current_Display_mode != MENU_MODE_DRIVE_STATUS) && (u16_User_Editing_Mode == EDIT_ENABLE)) )
   {
      //if user wats to insert a negative number using LEAST signifacant bit
      if ( IsArrayZero(u16_Display_Array_Msb_Lsb,4*CDHD_HMI_SIZE) && direction == MENU_DIRECTION_DOWN && u16_User_Digit_Edit == 0 ) u16_Display_Neg_Val_Flag = 1;
      //if user wats to insert a positive number using LEAST signifacant bit
      if ( IsArrayZero(u16_Display_Array_Msb_Lsb,4*CDHD_HMI_SIZE) && direction == MENU_DIRECTION_UP && u16_User_Digit_Edit == 0 ) u16_Display_Neg_Val_Flag = 0;
      IndexUpdate(&u16_Display_Array_Msb_Lsb[u16_User_Digit_Edit], direction, 10,1); 
      DisplayMsbLsb(u16_User_Editing_Msb_Lsb_Offset,CDHD_HMI_SIZE);//for user edit mode
      ConvertArrayToLongLong(u16_Display_Array_Msb_Lsb,&s64_User_Entered_Value);//for user edit mode
   } 
}




void HmiJogRequestHandler(int drive)
{
   if ( u16_Hmi_Jog_Flag == 1 )
   {
      if ( BGVAR(u16_P4_08_Push_Button_Status) == BUTTON_UP_STATUS )
      {
         HandleJogRun(drive,MENU_DIRECTION_UP);
         u16_Stop_Jog = 0;
      }
      else if ( BGVAR(u16_P4_08_Push_Button_Status) == BUTTON_DOWN_STATUS )
      {
         HandleJogRun(drive,MENU_DIRECTION_DOWN);
         u16_Stop_Jog = 0;
      }
      HmiDisplayManager(drive);
   }
}
void HmiJogStopRequestHandler(int drive)
{
   if ( u16_Hmi_Jog_Flag == 1 )
   {
      HmiDisplayManager(drive);
      if ( BGVAR(u16_P4_08_Push_Button_Status) == 0x0000 )
         ++u16_Stop_Jog;       
      if ( ((BGVAR(u16_P4_08_Push_Button_Status) == 0x0000) && (u16_Stop_Jog >= 2)) )
         HandleJogStop(drive);
   }
}


int HmiManager(int drive)
{      
   //timer for display refresh rate
   static  long s32_Display_timer;
   static  long s32_Long_Up_Down_timer_1;

   if (u16_Foe_ParamsFile_State != FOE_PARAMS_FILE_DWLD_IDLE)
   {
      HmiDisplayFiveLetterString("FoE  ",5);
      return 1; 
   }
   
   HmiJogRequestHandler(drive);//If HMI issued Jog request 
   
   //handles all continuous command handler after user's request
   if ( u16_Hmi_Commands_Steps_Counter >= 1 )
   {
      ++u16_Hmi_Commands_Steps_Counter;
      CallContinuousComandHandler(drive);
   }
   if( (u16_Hmi_Jog_Flag != 1) && ButtonUpRisingEdge(drive) /*||(BGVAR(u16_P4_08_Push_Button_Status) & BUTTON_UP_STATUS &&  u8_Cont_Press == 2)*/ )
   {
      HandleDownAndUpKeyRisingEdge(drive, MENU_DIRECTION_UP);
      HmiDisplayManager(drive);
   }
   //check if user pressed up or down only if user DID NOT request jogging
   if( (u16_Hmi_Jog_Flag != 1) && ButtonDownRisingEdge(drive)  )
   {
      HandleDownAndUpKeyRisingEdge(drive, MENU_DIRECTION_DOWN);
      HmiDisplayManager(drive); 
   }
   //check if user pressed mode
   if( ButtonMRisingEdge(drive) )
   {
      DisplayModeSwitch(); 
      HmiDisplayManager(drive);
   }
      //check if user pressed up Shift for 1000 ms
   if ( ButtonSPressed500ms(drive) )
   {
      HandleLongShiftPress(drive);
      HmiDisplayManager(drive);
   }
   //check if user pressed up Mode and Shift for 1000 ms
   if ( ButtonMSPressed500ms(drive) )
   {
      HmiHandleConfig(drive);
   }
   //check if user pressed up Mode and Shift for 4000 ms
   if ( ButtonMSPressed2000ms(drive) || s16_Hmi_Save_To_Flash_Flag == 1)
   {
      s16_Hmi_Save_To_Flash_Flag = 1;
      //HmiHandleSave(drive);
      DisplaySaveProgress(drive);
      return SAL_SUCCESS;
   }
   //check if user pressed up shift 
   if ( ButtonSRisingEdge(drive) )
   {
      HandleShiftKeyRisingEdge(drive);
      HmiDisplayManager(drive);
   }
   //if no key pressed refresh display every 250 ms
   //if save command under way refreah each BG cycle 
   //if long up/down press reduce refresh rate
   if (u16_Execute_Continuous_Command == 1)
      s16_Hmi_Refresh_Rate = 0L;
   if (u8_Long_Up_Down > 1)
      s16_Hmi_Refresh_Rate = 250L-(long)u8_Long_Up_Down*35L;   
   else 
      s16_Hmi_Refresh_Rate = 250L;
      
   if( ((BGVAR(u16_P4_08_Push_Button_Status) & (BUTTON_UP_STATUS | BUTTON_DOWN_STATUS)) != 0) && (u8_Long_Up_Down == 0) && (u16_Hmi_Jog_Flag != 1) ) 
   {
      s32_Long_Up_Down_timer_1 = Cntr_1mS;
      u8_Long_Up_Down++; 
   }
   if ( PassedTimeMS(1000LL, s32_Long_Up_Down_timer_1) )
   {
      s32_Long_Up_Down_timer_1 = Cntr_1mS;
      u8_Long_Up_Down++;
   }
   else if ( BGVAR(u16_P4_08_Push_Button_Status) == 0 )
   {
      u8_Long_Up_Down = 0;
   }
   
   if ( PassedTimeMS(s16_Hmi_Refresh_Rate, s32_Display_timer) )
   {
      if( ((u16_Hmi_Jog_Flag != 1) && (BGVAR(u16_P4_08_Push_Button_Status) & BUTTON_UP_STATUS) &&  u8_Long_Up_Down > 1) )
      {
         HandleDownAndUpKeyRisingEdge(drive, MENU_DIRECTION_UP);
      }
      if( ((u16_Hmi_Jog_Flag != 1) && (BGVAR(u16_P4_08_Push_Button_Status) & BUTTON_DOWN_STATUS) &&  u8_Long_Up_Down > 1) )
      {
         HandleDownAndUpKeyRisingEdge(drive, MENU_DIRECTION_DOWN); 
      }
      s32_Display_timer = Cntr_1mS;
      HmiDisplayManager(drive);
   }
   //when calling jog through hmi if up or down keys
   //are no longer pressed handle jog stop  
   HmiJogStopRequestHandler(drive);    
   return SAL_SUCCESS;
}


//Menu STATUS!!!!
//for CDHD2 HMI Shachar C.
//returns the hexdecimal value for the 7seg display with decimal point
unsigned int GetHexDecValForCDHD7SegDisplay(unsigned int digit,unsigned int decimal_point_flag)
{
   unsigned int hexa_decimal_repr;
   hexa_decimal_repr = Cdhd2ConvertDigitTo7SegDisplay(digit);
   if ( DECIMAL_POINT_ON == decimal_point_flag )
   {
      hexa_decimal_repr &= DC_POINT;
   }
   return hexa_decimal_repr;
}

unsigned int GetEnableStatusForDisplay(int drive)
{
   return Enabled(drive);
}
unsigned int GetOpmodeStatusForDisplay(int drive)
{
   // AXIS_OFF;                                                 
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   return VAR(AX0_s16_Opmode);
}
unsigned int GetCanObj6060StatusForDisplay(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   return p402_modes_of_operation_display;
}

unsigned int GetSyncIndecStatusForDisplay(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error 
   return(u16_Pll_State == PLL_LOCKED);
}

unsigned int GetMotionStatusForDisplay(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   //if motor not stopped take time measurement raise flag
   if (BGVAR(s16_Motor_Moving) != 0 && u8_Display_Movment_Flag == 0)
   {
      s32_Display_Movment_Timer = Cntr_1mS;
      u8_Display_Movment_Flag = 1; 
   }
   if ( PassedTimeMS(50L, s32_Display_Movment_Timer))
   {
      //if time passed since time measurement check movement status again
      if ( (0 == VAR(AX0_s16_Stopped)) || BGVAR(s16_Motor_Moving) != 0) 
         u8_Display_Movment_Flag = 2;
      else 
         u8_Display_Movment_Flag = 0;
      s32_Display_Movment_Timer = Cntr_1mS;
   }
   if (u8_Display_Movment_Flag == 2) 
      return 'r'; 
   else
      return ' ';         

}

unsigned int FaultsDisplay(int drive)
{

   int u16_index_in_table = 0;
   int s16_set = 0;
   long long i,s64_mask =0;
   int fault = FaultDisplay(drive);
   if ( fault == 0 )return NONE;
    
   REFERENCE_TO_DRIVE;
   //check sys_not_ok
   for (i = 0; i < 64; i++)
   {
      if (((BGVAR(s64_SysNotOk) >> i) == 0) && (BGVAR(s64_SysNotOk_2) == 0)) return NONE; // no more faults to display

      s64_mask = 0x01LL << i;
      if (BGVAR(s64_SysNotOk) & s64_mask)
      {
         s16_set = 0;
         break;
      }
   }
   //check SysNotOk_2
   if (BGVAR(s64_SysNotOk) == 0)
   {
      for (i = 0; i < 64; i++)
      {
         if ((BGVAR(s64_SysNotOk_2) >> i) == 0) return NONE; // no more faults to display

         s64_mask = 0x01LL << i;
         if (BGVAR(s64_SysNotOk_2) & s64_mask)
         {
            s16_set = 1;
            break;
         }
      }
   }

   
   while (s64_mask != (1LL << (unsigned long long)u16_index_in_table))
      u16_index_in_table++;
   //set fault chars into string
   u16_index_in_table += s16_set * 64;    // support another var of 64 faults. s16_set can be 0 or 1
   BGVAR(u16_Display_Char)[0] = Display_Table[u16_index_in_table].u16_char1;
   BGVAR(u16_Display_Char)[1] = Display_Table[u16_index_in_table].u16_char2;
   BGVAR(u16_Display_Char)[2] = Display_Table[u16_index_in_table].u16_char3;
   BGVAR(u16_Display_Char)[3] = Display_Table[u16_index_in_table].u16_char4;
   BGVAR(u16_Display_Char)[4] = NONE;
   //set the nuber of chars in the string
   if (!BGVAR(u16_Display_Char)[1])
      BGVAR(u16_Display_Char_Number) = 1;
   else if ((!BGVAR(u16_Display_Char)[2]) || (BGVAR(u16_Display_Char)[1] == NONE))
      BGVAR(u16_Display_Char_Number) = 2;
   else if ((!BGVAR(u16_Display_Char)[3]) || (BGVAR(u16_Display_Char)[2] == NONE))
      BGVAR(u16_Display_Char_Number) = 3;
   else if (BGVAR(u16_Display_Char)[3] == NONE)
      BGVAR(u16_Display_Char_Number) = 4;
   else BGVAR(u16_Display_Char_Number) = 5;
   //if only 1 char in string do not switch chars on 7seg no.3 (2nd from the left)
   if (BGVAR(u16_Display_Char_Number) > 1)
   {
      if ( PassedTimeMS(500L, s32_Display_Sync_Timer))
      {
         s32_Display_Sync_Timer = Cntr_1mS;
         u32_Display_500ms_Counter++;
         ++u8_Digit_Blink_Switch;
         if ( u8_Digit_Blink_Switch%2 == 0  && u16_IndexInString != 0 && u16_IndexInString != u16_Display_Char_Number )
         {
    
            BGVAR(u16_Local_Hmi_Display_Array)[3] = NONE;
            return NONE;
         }
         else
         {
            u16_IndexInString = (u16_IndexInString+1)%(BGVAR(u16_Display_Char_Number));
            BGVAR(u16_Local_Hmi_Display_Array)[3] = BGVAR(u16_Display_Char)[u16_IndexInString];
            return  BGVAR(u16_Display_Char)[u16_IndexInString];
         }
      }
   }
   //more than 1 chars blink chars in order on 7seg no.3 (2nd from the left)
   else
   {
      BGVAR(u16_Local_Hmi_Display_Array)[3] = BGVAR(u16_Display_Char)[0];
   }
   return  BGVAR(u16_Local_Hmi_Display_Array)[3];
}
//display warning for status mode
unsigned int WarningsDisplay(int drive)
{
   int warning = WarningDisplay(drive);
   REFERENCE_TO_DRIVE;
   if ( warning == 0 )return NONE;//if no warning 
   //if only 1 char in string do not switch chars on 7seg no.3 (2nd from the left)
   if (BGVAR(u16_Display_Char_Number) > 1)
   {
      if ( PassedTimeMS(500L, s32_Display_Sync_Timer))
      {
         s32_Display_Sync_Timer = Cntr_1mS;
         u32_Display_500ms_Counter++;
         //more than 1 chars blink chars in order on 7seg no.3 (2nd from the left)
         {
            u16_IndexInString = (u16_IndexInString+1)%(BGVAR(u16_Display_Char_Number));
            BGVAR(u16_Local_Hmi_Display_Array)[3] = BGVAR(u16_Display_Char)[u16_IndexInString];
            return  BGVAR(u16_Display_Char)[u16_IndexInString];
         }
      }
   }
   else
   {
      BGVAR(u16_Local_Hmi_Display_Array)[3] = BGVAR(u16_Display_Char)[0];
   }
   return  BGVAR(u16_Local_Hmi_Display_Array)[3];
}
unsigned int MotorSetupForStatusDisplay(int drive)
{
   
   REFERENCE_TO_DRIVE; 
   u16_Display_Char_Number = 3;
   //if only 1 char in string do not switch chars on 7seg no.3 (2nd from the left)
   BGVAR(u16_Display_Char)[0] = A_LET;
   BGVAR(u16_Display_Char)[1] = t_LET;
   BGVAR(u16_Display_Char)[2] = ONE;
   
   if (BGVAR(u16_Display_Char_Number) > 1)
   {
      if ( PassedTimeMS(500L, s32_Display_Sync_Timer))
      {
         s32_Display_Sync_Timer = Cntr_1mS;
         u32_Display_500ms_Counter++;
         ++u8_Digit_Blink_Switch;
         if ( u8_Digit_Blink_Switch%2 == 0  && u16_IndexInString != 0 && u16_IndexInString != u16_Display_Char_Number )
         {
            BGVAR(u16_Local_Hmi_Display_Array)[3] = NONE;
            return NONE;
         }
         else
         //more than 1 chars blink chars in order on 7seg no.3 (2nd from the left)
         {
            u16_IndexInString = (u16_IndexInString+1)%(BGVAR(u16_Display_Char_Number));
            BGVAR(u16_Local_Hmi_Display_Array)[3] = BGVAR(u16_Display_Char)[u16_IndexInString];
            return  BGVAR(u16_Display_Char)[u16_IndexInString];
         }
      }
   }
   else
   {
      BGVAR(u16_Local_Hmi_Display_Array)[3] = BGVAR(u16_Display_Char)[0];
   }
   return  BGVAR(u16_Local_Hmi_Display_Array)[3];
}

unsigned int StausScreenFaultsAndWarningHandler(int drive)
{
   //if fault exist 
   if ( ((BGVAR(s64_SysNotOk) != 0) || (BGVAR(s64_SysNotOk_2) != 0)) )
      return FaultsDisplay(drive);
   //if warning exist
   if ( BGVAR(u64_Sys_Warnings) != 0 )
      return WarningsDisplay(drive);
   if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_ACTIVE )
      return MotorSetupForStatusDisplay(drive);
   else 
      return NONE; 
}


//set the values for all decimal points according to statuses
void DecimalPointArrayForStatusDisplay(int drive)
{
    u16_Decimal_Point_Flag_Array[ENABLE_STATUS_7SEG_POS]= GetEnableStatusForDisplay(drive);
    u16_Decimal_Point_Flag_Array[SYNC_STATUS_7SEG_POS] = GetSyncIndecStatusForDisplay(drive);
    u16_Decimal_Point_Flag_Array[MOTION_STATUS_7SEG_POS] = DECIMAL_POINT_OFF;
    u16_Decimal_Point_Flag_Array[WARNINGS_STATUS_7SEG_POS] = DECIMAL_POINT_OFF;
    u16_Decimal_Point_Flag_Array[MENU_LETTER_7SEG_POS] = DECIMAL_POINT_OFF;
}

void HandleCommodeForStatusDisplay(int drive)
{
   // AXIS_OFF;
   if ( VAR(AX0_u16_Com_Mode1) == 0 )
   {
      u16_Drive_Status_Array[OPMODE_STATUS_7SEG_POS] = GetOpmodeStatusForDisplay(drive);
      u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = 0;
   }
   else if ( VAR(AX0_u16_Com_Mode1) == 1 )
   {
      u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = GetCanObj6060StatusForDisplay(drive);
      switch ( u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] )
      {
         case 0:
         case 2:
         case 5:
         {
           u16_Drive_Status_Array[OPMODE_STATUS_7SEG_POS] = ' ';
           u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = ' ';
           break;
         }
         case 1:  
         {
           u16_Drive_Status_Array[OPMODE_STATUS_7SEG_POS] = 'P';
           u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = 'P';
           break;
         }
         case 3:  
         {
           u16_Drive_Status_Array[OPMODE_STATUS_7SEG_POS] = 'S';
           u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = 'P';
           break;
         }
         case 4:  
         {
           u16_Drive_Status_Array[OPMODE_STATUS_7SEG_POS] = 't';
           u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = 'P';
           break;
         }
         
         case 6:  
         {
           u16_Drive_Status_Array[OPMODE_STATUS_7SEG_POS] = 'H';
           u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = 'H';
           break;
         }
         case 7:
         {  
           u16_Drive_Status_Array[OPMODE_STATUS_7SEG_POS] = 'P';
           u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = 'I';
           break;
         }
         case 8:  
         {
           u16_Drive_Status_Array[OPMODE_STATUS_7SEG_POS] = 'P';
           u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = 'S';
           break;
         }
         case 9:  
         {
           u16_Drive_Status_Array[OPMODE_STATUS_7SEG_POS] = 'S';
           u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = 'S';
           break;
         }
         case 10:  
         {
           u16_Drive_Status_Array[OPMODE_STATUS_7SEG_POS] = 't';
           u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = 'S';
           break;
         }
         default:
         {
           u16_Drive_Status_Array[OPMODE_STATUS_7SEG_POS] = 1;
           u16_Drive_Status_Array[CAN_OBJ_6060_STATUS_7SEG_POS] = '-';
           break;;
         }            
      }
   }  
}
//set the values for all statuses
void GetStatusValuesForDisplay(int drive)//TODE set drive status values for display
{
   HandleCommodeForStatusDisplay(drive); 
   u16_Drive_Status_Array[MOTION_STATUS_7SEG_POS] = GetMotionStatusForDisplay(drive);
   u16_Drive_Status_Array[WARNINGS_STATUS_7SEG_POS] = StausScreenFaultsAndWarningHandler(drive);//StausScreenFaultsHandler(drive);
   u16_Drive_Status_Array[MENU_LETTER_7SEG_POS] = 'S';
}


//display ststus 5 digit 7 segments
int DisplayDriveStatus(int drive)
{
  int i = 0;
  unsigned int u16_display_array[CDHD_HMI_SIZE] = {0};
  
  GetStatusValuesForDisplay(drive);
  DecimalPointArrayForStatusDisplay(drive);
 
  for (i = 0; i < CDHD_HMI_SIZE; ++i)
  {
      if ( i != WARNINGS_STATUS_7SEG_POS )
      {
         BGVAR(u16_Local_Hmi_Display_Array)[i] = u16_display_array[i] = GetHexDecValForCDHD7SegDisplay(BGVAR(u16_Drive_Status_Array)[i],BGVAR(u16_Decimal_Point_Flag_Array)[i]);
         HWDisplay(u16_display_array[i],i);
      }
      else
      {
         BGVAR(u16_Local_Hmi_Display_Array)[i] = u16_display_array[i] = BGVAR(u16_Drive_Status_Array)[i];
         HWDisplay(u16_display_array[i],i);
      }
         
  }
  return 1;
}







//This functions diplay sequence of characters on 7segment display
//between each character 0.5 sec delay.
void DisplayCode(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_Display_Code) = BGVAR(u16_Display_Char)[0];

   if (BGVAR(u16_Display_Char_Number) > 1)
   {
      BGVAR(u16_Display_Code) = BGVAR(u16_Display_Char)[BGVAR(u16_Display_Index)]; // display it for 0.5 sec
      if (u32_Display_State_Timer != u32_Display_State_Timer_Prev)
      {
         u32_Display_State_Timer_Prev = u32_Display_State_Timer;
         BGVAR(u16_Display_Index)++;
         if (BGVAR(u16_Display_Index) >= BGVAR(u16_Display_Char_Number))
            BGVAR(u16_Display_Index) = 0;
      }
   }

   if (Enabled(DRIVE_PARAM)) BGVAR(u16_Display_Code) &= DC_POINT;   // dc point
      else BGVAR(u16_Display_Code) |= 0x80;
}


unsigned int DigitCode(unsigned int u16_num)
{
   unsigned int u16_ret_val = NONE;
   switch (u16_num)
   {
      case 0:  u16_ret_val = ZERO;   break;
      case 1:  u16_ret_val = ONE;    break;
      case 2:  u16_ret_val = TWO;    break;
      case 3:  u16_ret_val = THREE;  break;
      case 4:  u16_ret_val = FOUR;   break;
      case 5:  u16_ret_val = FIVE;   break;
      case 6:  u16_ret_val = SIX;    break;
      case 7:  u16_ret_val = SEVEN;  break;
      case 8:  u16_ret_val = EIGHT;  break;
      case 9:  u16_ret_val = NINE;   break;
      case 10: u16_ret_val = A_LET;  break;
      case 11: u16_ret_val = b_LET;  break;
      case 12: u16_ret_val = C_LET;  break;
      case 13: u16_ret_val = d_LET;  break;
      case 14: u16_ret_val = E_LET;  break;
      case 15: u16_ret_val = F_LET;  break;
   }
   return u16_ret_val;
}
#pragma CODE_SECTION(HandleDigitBlinking, "ramfunc_3");
unsigned int HandleDigitBlinking(unsigned int u16_code, unsigned char u8_display_number)
{

   if ( BGVAR(u8_Local_Hmi_Blink_Display) == u8_display_number+1 || BGVAR(u8_Local_Hmi_Blink_Display) == 0xFF )
   {
      if ( PassedTimeMS(250L, s32_Display_Sync_Timer))
      {
         s32_Display_Sync_Timer = Cntr_1mS;
         u32_Display_500ms_Counter++;
         ++u8_Digit_Blink_Switch;
      }
   }
   if ( u8_Digit_Blink_Switch%2 == 0 && ((BGVAR(u8_Local_Hmi_Blink_Display) == u8_display_number+1)||BGVAR(u8_Local_Hmi_Blink_Display) == 0xFF) ) 
      return 0xFF;
   else return u16_code;
}



//**********************************************************
// Function Name: HWDisplay
// Description:
//          This function displays code on 7segment display.
//
//          For SHNDR it forwards a digit-code to the right FPGA registers.
//          Basically the content of the display array "BGVAR(u16_Local_Hmi_Display_Array)"
//          needs to be forwarded.
//
//          u16_code - Display code (bit-inverted for the FPGA register)
//          u8_Display_Number - 7-segment which is supposed to be accessed, with
//                              0 = Digit with the least significant value
//                              4 = Digit with the most significant value.
//
// Display code is
// DP,SEG G,SEG F,SEG E,SEG D,SEG C,SEG B,SEG A
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(HWDisplay, "ramfunc_3");
void HWDisplay(unsigned int u16_code, unsigned char u8_display_number)
{
   if ( BGVAR(u8_Local_Hmi_Blink_Display) != 0 &&  u8_Is_Retro_Display == 0 )
   {
      u16_code = HandleDigitBlinking(u16_code, u8_display_number);
   }
   if ( u8_Is_Retro_Display == 1 )u8_display_number = 0;
   switch (u8_display_number)
   {
      case (4):
         *(int*)(FPGA_DSPLY_5_REG_ADD) = ~u16_code;
      break;
      case (3):
         *(int*)(FPGA_DSPLY_4_REG_ADD) = ~u16_code;
      break;
      case (2):
         *(int*)(FPGA_DSPLY_3_REG_ADD) = ~u16_code;
      break;
      case (1):
         *(int*)(FPGA_DSPLY_2_REG_ADD) = ~u16_code;
      break;
      case (0):
      default:
         if (u16_Product == SHNDR_HW)
         {
            // Fix IPR#1384 No communication indicator available for Modbus:
            // toggle the last bit acording to the MODBUS traffic
            if (!Test_Led_On)
            {
            if(BGVAR(u16_ModBus_Ind) & MODBUS_IND_TOGGLE_STATE)
            {// toggle OFF
               u16_code |= ~DC_POINT;
            }
            else
            {//toggle ON
               u16_code &= DC_POINT;
            }
         }
     }

         *(int*)(FPGA_DISPLAY_REG_ADD) = ~u16_code;
         u16_Seven_Segment_Value = (~u16_code & 0x007F);
      break;
   }
}

// In normal operation always display opmode 2
void NormalDisplay(int drive)
{
   // AXIS_OFF;
   int u16_est_motor_param = 0;
   unsigned int u16_disp = 0;
   REFERENCE_TO_DRIVE;
   if (BGVAR(s8_BurninParam) == 1)
   {
      BGVAR(u16_Display_Char)[0] = ONE;
      BGVAR(u16_Display_Char)[1] = I_LET;
      BGVAR(u16_Display_Char_Number) = 2;
      return;
   }

   // Display "AT1" on MOTORSETUP, "AT2" on CLTUNE and "AT3" on ESTMOTORPARAM

   /*
   if ( (BGVAR(s16_Param_Est_State) != PARAM_EST_STATE_IDLE) &&
        (BGVAR(s16_Param_Est_State) != PARAM_EST_STATE_DONE) &&
        (BGVAR(s16_Param_Est_State) != PARAM_EST_STATE_FAULT) )
      u16_est_motor_param = 1;
   */

   if ( (BGVAR(s16_Motor_Params_Est_State) != MOTOR_PARAMS_EST_STATE_IDLE) &&
        (BGVAR(s16_Motor_Params_Est_State) != MOTOR_PARAMS_EST_STATE_DONE) &&
        (BGVAR(s16_Motor_Params_Est_State) > 0)/*Not fault*/ )
      u16_est_motor_param = 1;


   if ((BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_IDLE) || u16_est_motor_param)
   {
      BGVAR(u16_Display_Char)[0] = A_LET;
      BGVAR(u16_Display_Char)[1] = t_LET;

      if (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_IDLE)
         BGVAR(u16_Display_Char)[2] = ONE;
      else if (!u16_est_motor_param)
         BGVAR(u16_Display_Char)[2] = TWO;
      else
         BGVAR(u16_Display_Char)[2] = THREE;

      BGVAR(u16_Display_Char)[3] = NONE;
      BGVAR(u16_Display_Char_Number) = 4;
      return;
   }
   
   //FOE
   if (u16_Foe_ParamsFile_State != FOE_PARAMS_FILE_DWLD_IDLE)
   {
      BGVAR(u16_Display_Char)[0] = F_LET;
      BGVAR(u16_Display_Char)[1] = o_LET;
      BGVAR(u16_Display_Char)[2] = E_LET;
      BGVAR(u16_Display_Char_Number) = 3;
      return;
   }
   
   //Display mode == 0 --> display opmode
   if(BGVAR(u16_Display_Mode) == 0)
   {
      u16_disp = VAR(AX0_s16_Opmode);
   }
   //Display mode == 1 --> display fieldbus opmode
   else if(BGVAR(u16_Display_Mode) == 1)
   {
      u16_disp = p402_modes_of_operation_display;
   }

   ////Display mode == 2 --> display fieldbus node ID
   else if(BGVAR(u16_Display_Mode) == 2)
   {
      if(IS_CAN_DRIVE)
      {
         u16_disp = GL_ARRAY(coNodeId);
      }
      else if(IS_EC_DRIVE)
      {
         u16_disp = (*(unsigned int*)p_u16_tx_station_address);
      }
   }

   if(u16_disp < 10)//one digit
   {
      BGVAR(u16_Display_Char)[0] = ConvertDigitTo7SegDisplay(u16_disp);
      BGVAR(u16_Display_Char)[1] = NONE;
     BGVAR(u16_Display_Char_Number) = 1;
   }
   else if(u16_disp < 100) //2 digits
   {
      BGVAR(u16_Display_Char)[0] = ConvertDigitTo7SegDisplay(u16_disp/10);
      BGVAR(u16_Display_Char)[1] = ConvertDigitTo7SegDisplay(u16_disp%10);
     BGVAR(u16_Display_Char)[2] = NONE;
     BGVAR(u16_Display_Char_Number) = 3;
   }
   else //3 digits
   {
      BGVAR(u16_Display_Char)[0] = ConvertDigitTo7SegDisplay(u16_disp/100);
      BGVAR(u16_Display_Char)[1] = ConvertDigitTo7SegDisplay((u16_disp%100)/10);
      BGVAR(u16_Display_Char)[2] = ConvertDigitTo7SegDisplay(u16_disp%10);
     BGVAR(u16_Display_Char)[3] = NONE;
     BGVAR(u16_Display_Char_Number) = 4;
   }

   if (BGVAR(u64_Sys_Warnings) & PHASE_FIND_REQ_WRN_MASK)
      BGVAR(u16_Display_Char_Number) = 2;
   // u16_Hold_User_Hide_7Seg_Display is used to disable the blinking of the display
   // this is used in canopen because setting the halt bit in control word will enable the hold feature.
   else if (BGVAR(u16_Hold) && BGVAR(u16_Hold_User_Hide_7Seg_Display) == 0)
   {
      BGVAR(u16_Display_Char)[1] = BGVAR(u16_Display_Char)[0];
      BGVAR(u16_Display_Char)[2] = NONE;
      BGVAR(u16_Display_Char)[3] = NONE;
      BGVAR(u16_Display_Char_Number) = 4;
   }
}


int WarningDisplay(int drive)
{
   unsigned long i;
   for (i = 0; i < 64; i++)
   {
      if (BGVAR(u64_Sys_Warnings) & (((unsigned long long)0x1 << i)))
      {
        if (DisplayWarning(drive, ((unsigned long long)0x1 << i)) == 1)
          return 1;
      }
   }
   return 0;
}


int FaultDisplay(int drive)
{
   long long i, s64_mask;

   for (i = 0; i < 64; i++)
   {
      if (((BGVAR(s64_SysNotOk) >> i) == 0) && (BGVAR(s64_SysNotOk_2) == 0)) return 0; // no more faults to display

      s64_mask = 0x01LL << i;
      if (BGVAR(s64_SysNotOk) & s64_mask)
      {
         // 5V fault happens on every power off. Dispaly it after delay, since if the drive is still running the fault is real.
         // Other fault display immediately
         if (  (s64_mask != DRIVE_5V_FLT_MASK)                                                  ||
              ((s64_mask == DRIVE_5V_FLT_MASK) && (BGVAR(u16_Delayed_5V_Log_Fault_Flag) == 0) )    )
         {
            DisplayFault(drive, s64_mask, (int)0);
            return 1;
         }
      }
   }

   for (i = 0; i < 64; i++)
   {
      if ((BGVAR(s64_SysNotOk_2) >> i) == 0) return 0; // no more faults to display

      s64_mask = 0x01LL << i;
      if (BGVAR(s64_SysNotOk_2) & s64_mask)
      {
         // AC loss faults happen on every power off. Dispaly it after delay, since if the drive is still running the fault is real.
         // Other fault display immediately
         if ( ((s64_mask != BUS_AC_LOSS_MASK) && (s64_mask != LOGIC_AC_LOSS_MASK))                         ||
              ((s64_mask == BUS_AC_LOSS_MASK)   && (BGVAR(u16_Delayed_Line_Loss_Log_Fault_Flag) == 0) )    ||
              ((s64_mask == LOGIC_AC_LOSS_MASK) && (BGVAR(u16_Delayed_Logic_AC_Loss_Log_Fault_Flag) == 0) )    )
         {
            DisplayFault(drive, s64_mask, (int)1);
            return 1;
         }
      }
   }

   return 0;
}


void DisplayControl(int drive)
{
   static int u16_stored_display_state = 0, u16_i = 0;
   static unsigned char u8_current_segment=1,u8_current_CAN_led_segment=1;
   // In order to sync the display of the two axes of DDHD, a 500ms counter is defined.
   // The first DDHD axis passes it to the second DDHD axis.
   // The display control is modified to use the 500ms counter instead of using PassedTimeMS().
   // This change is not applicable to SE.

   // Check if was successful data transfer between axes. Only in DDHD second axes a successful data transfer can happen.
   // If successful data transfer is detected (current does not equal prev), DDHD second axis is assumed, and the display counter
   // is taken from the transferred data.
   // If no data transfer is detected (current equals prev), no DDHD, DDHD first axis, or DDHD second axis with failuer in the
   // data transfer mechanism is assumed, and the display counter is incremented here.

   if (u16_FPGA_BG_Buffer_Transfer_Success == u16_FPGA_BG_Buffer_Transfer_Success_Prev)
   {
      if (u16_No_FPGA_BG_Buffer_Transfer < 10)
      {
         u16_No_FPGA_BG_Buffer_Transfer++;
      }
   }
   else
   {
      u16_No_FPGA_BG_Buffer_Transfer = 0;
   }
   u16_FPGA_BG_Buffer_Transfer_Success_Prev = u16_FPGA_BG_Buffer_Transfer_Success;

   // Only if for 10 background cycles no successfull data-transfer has happened
   // (so either no DDHD or DDHD plus second axis with data-transfer error). Otherwise
   // the "u32_Display_500ms_Counter" counter is incremented somewhere else.
   if (u16_No_FPGA_BG_Buffer_Transfer >= 10)
   {
      if (PassedTimeMS(500L, s32_Display_Sync_Timer))
      {
         s32_Display_Sync_Timer = Cntr_1mS;
         u32_Display_500ms_Counter++;
      }
   }

   // If the Drive is not busy displaying the baud-rate on the 7-segment upon
   // boot. This if-condition is needed to not interfere displaying the
   // baud-rate with by the display-test command.
   if (Display_State != 11)
   {
      // Highest priority is to display the baud-rate setting
      if (u16_Display_Baud_Rate_Index < DISPLAY_BAUD_RATE_ARRAY_SIZE)
      {
         u16_stored_display_state = Display_State;
         Display_State = 11;
      }
      else if (Test_Led_On == 1) // To handle TESTLED
      {
         Test_Led_On = 2;
         u16_stored_display_state = Display_State;
         u16_i = 0;
         Display_State = 8;
      }
      else if(Test_Led_On == 4)
      {
         u16_stored_display_state = Display_State;
         Display_State = 12;
      }
   }

   switch (Display_State)
   {
      case 0:
         // Check if there is a change in the rotary switch
         // If no change in the rotary switch the state remains 0, and the display is handled in the "if" statement below this switch case.
         if (u32_Display_State_Timer != u32_Display_500ms_Counter)
         {
            u32_Display_State_Timer = u32_Display_500ms_Counter;

            if (IS_ROTARY_SWITCH)
            {
               // Rotary switch needs to be read after accessing 7-Seg (FPGA constrain)
               Rotary_Switch = ReadRotarySwitch();
               
               if (Prev_Rotary_Switch != Rotary_Switch)   // Show change in Rotary switch on change
               {
                  Display_State++;
                  Prev_Rotary_Switch = Rotary_Switch;
               }
            }
         }
      break;

      case 1: // Handle rotary switch display
         u32_Display_State_Timer = u32_Display_500ms_Counter;
         HWDisplay(DigitCode((Rotary_Switch & 0x00F0) >> 4), 4);
         Display_State++;
      break;

      case 2:
         if (u32_Display_State_Timer != u32_Display_500ms_Counter)
         {
            u32_Display_State_Timer = u32_Display_500ms_Counter;
            HWDisplay(NONE, 4);
            Display_State++;
         }
      break;

      case 3:
         if (u32_Display_State_Timer != u32_Display_500ms_Counter)
         {
            u32_Display_State_Timer = u32_Display_500ms_Counter;
            HWDisplay(DigitCode(Rotary_Switch & 0x000F), 4);
            Display_State++;
         }
      break;

      case 4:
         if (u32_Display_State_Timer != u32_Display_500ms_Counter)
         {
            u32_Display_State_Timer = u32_Display_500ms_Counter;
            HWDisplay(NONE, 4);
            Display_State++;
         }
      break;

      case 5: // End of rotary switch display
         if (u32_Display_State_Timer != u32_Display_500ms_Counter)
         {
            Display_State = 0;
         }
      break;

      case 6:      // This will handle the display of an unsecured drive
         if (u32_Display_State_Timer != u32_Display_500ms_Counter)
         {
            u32_Display_State_Timer = u32_Display_500ms_Counter;
            if( u8_Is_Retro_Display == 1 ) HWDisplay(b_LET, 4);
            Display_State++;
         }
      break;

      case 7:
         if (u32_Display_State_Timer != u32_Display_500ms_Counter)
         {
            u32_Display_State_Timer = u32_Display_500ms_Counter;
            HWDisplay(NONE, 4);
            Display_State--;
         }
      break;

      case 8:      // This will handle the TESTLED
      case 9:      // case 8 starts in the middle of 500ms so it may be less than 500ns, while case 9 provides full 500ms period
         if (u32_Display_State_Timer != u32_Display_500ms_Counter)
         {
            u32_Display_State_Timer = u32_Display_500ms_Counter;
            HWDisplay(ALL, 4);
            CANLed((unsigned int)1);
            Display_State++;
         }
      break;

      case 10:
         if (u32_Display_State_Timer != u32_Display_500ms_Counter)
         {
            u32_Display_State_Timer = u32_Display_500ms_Counter;
            HWDisplay(NONE, 4);
            CANLed((unsigned int)2);
            Display_State--;

            u16_i++;
            if (u16_i == 4)
            {
               Test_Led_On = 0;
               Display_State = u16_stored_display_state;
               CANLed((unsigned int)0);
            }
         }
      break;

      case 11:
         // If we reached the end of the baud-rate display array
         if(u16_Display_Baud_Rate_Index >= DISPLAY_BAUD_RATE_ARRAY_SIZE)
         {
            // restore previous display state
            Display_State = u16_stored_display_state;
         }
         else
         {
            // Display the baud-rate on the 7-segment
            HWDisplay(s8_Baud_Rate_Setting[u16_Display_Baud_Rate_Index], 4);
         }

         // After 0.5[s] jump to the next character to display
         if (u32_Display_State_Timer != u32_Display_500ms_Counter)
         {
            u32_Display_State_Timer = u32_Display_500ms_Counter;  // Reload timer
            u16_Display_Baud_Rate_Index++;                        // Jump to next 7-segment character
         }
      break;

      // new display test for CDHD

      case 12:
         u8_current_segment=0x1;
         u8_current_CAN_led_segment=0;
         Test_Led_On = 2;
      case 13:// turn ON each segment separatly
      case 14:// turn OFF each segment separatly
         if (PassedTimeMS(250L, u32_Display_State_Timer) || (Display_State == 12)/*if == 12 bypass the timer for first time*/)
         {
            if(Display_State == 12)
               Display_State= 13;

            u32_Display_State_Timer = Cntr_1mS;

            if(u8_current_segment > 0xFF)
            {
               u8_current_segment=0x1;
               if(Display_State == 13)
               {// state 2 finished reset and run state 3
                  Display_State = 14;
                  break;
               }
               else if(Display_State == 14)
               {// state 3 finished reset and stop
                  Test_Led_On = 0;

                  // restore previous display state
                  Display_State = u16_stored_display_state;

                  break;
               }
            }

            if(u8_current_CAN_led_segment > 0x3)
            {
               u8_current_CAN_led_segment = 0;
            }

            if(Display_State == 13)
            {//Display_State == 13: ON
               HWDisplay(~u8_current_segment, 4);
            }
            else if(Display_State == 14)
            {//Display_State == 14 OFF
               HWDisplay(u8_current_segment, 4);
            }
            CANLed(u8_current_CAN_led_segment);

            u8_current_CAN_led_segment++;
            u8_current_segment <<= 1;
         }
      break;
   }

   if (Display_State == 0)
   {
      if(u16_fault_control_delay > FAULT_CONTROL_DELAY) // leave display with "8." until all faults
      {  // have time to latch.  This is done to avoid displaying "garbage" in the display
         if (!FaultDisplay(drive)) if ( !WarningDisplay(drive) && u8_Is_Retro_Display == 1 )  NormalDisplay(drive);
         DisplayCode(drive);
         HWDisplay((unsigned int)(BGVAR(u16_Display_Code)), 4);
      }
      else //faults did not have time to be fully latched
      {
         HWDisplay(ALL, 4);
      }
   }
}


// led: 1 error led, 2 run led
#ifdef CONFIG_CO_LED
#pragma CODE_SECTION(ledInd, "ramfunc_2");
void ledInd (
   UNSIGNED8 led, /**< which CANopen LED */
   UNSIGNED8 action /**< turn LED on or off */
   )
{
   if (led == CO_ERR_LED)
   {
      if (action == CO_LED_ON)
      {
         /* switch Error LED on */
         u16_Can_Leds_Value |= 2;
      }
      else
      {
         /* switch Error LED off */
         u16_Can_Leds_Value &= ~2;
      }
   }

   if (led == CO_RUN_LED)
   {
      if (action == CO_LED_ON)
      {
         /* switch Status LED on */
         u16_Can_Leds_Value |= 1;
      }
      else
      {
         /* switch Status LED off */
         u16_Can_Leds_Value &= ~1;
      }
   }

   // update CAN leds only if display test is not running
   if (Test_Led_On == 0)
   {
      CANLed(u16_Can_Leds_Value);
   }
}
#endif /* CONFIG_CO_LED */


#pragma CODE_SECTION(CANLed, "ramfunc_3");
void CANLed(unsigned int u16_color)
{
   if (!IS_CAN_DRIVE_AND_COMMODE_1)
   {// if drive is IO drive keep CAN leds OFF
      u16_color=0;
   }

   if (u16_color <=3)
   {
      if (u16_Product == DDHD)
      {
         EALLOW;
         switch (u16_color)
         {
            case 0:        // Green LED - off, Red LED - off
               GpioCtrlRegs.GPBDIR.bit.GPIO62 = 0;    // Green High Z (off)

               GpioCtrlRegs.GPBDIR.bit.GPIO63 = 0;    // Red High Z (off)
            break;

            case 1:        // Green LED - on, Red LED - off
               GpioDataRegs.GPBDAT.bit.GPIO62 = 0;
               GpioCtrlRegs.GPBDIR.bit.GPIO62 = 1;    // Green active low (on)

               GpioCtrlRegs.GPBDIR.bit.GPIO63 = 0;    // Red High Z (off)
            break;

            case 2:        // Green LED - off, Red LED - on
               GpioCtrlRegs.GPBDIR.bit.GPIO62 = 0;    // Green High Z (off)

               GpioDataRegs.GPBDAT.bit.GPIO63 = 0;
               GpioCtrlRegs.GPBDIR.bit.GPIO63 = 1;    // Red active low (on)
            break;

            case 3:        // Green LED - on, Red LED - on
               GpioDataRegs.GPBDAT.bit.GPIO62 = 0;
               GpioCtrlRegs.GPBDIR.bit.GPIO62 = 1;    // Green active low (on)

               GpioDataRegs.GPBDAT.bit.GPIO63 = 0;
               GpioCtrlRegs.GPBDIR.bit.GPIO63 = 1;    // Red active low (on)
            break;
         }
         EDIS;
      }
      else if (u16_Product == SHNDR_HW)
      {
         // leds work with opposite polarity, so invert the value (in lxm28)
         *(int*)FPGA_CAN_LED_COLOR_ADD = (~u16_color) & 0x3;
      }
      else
      {
         *(int*)FPGA_CAN_LED_COLOR_ADD = u16_color;
      }
   }
}

//this function is used by CDHD2 status display only for the display of warnings
//for the faults menu mod warning scrolling menu see : GetWarningStringForDisplay
//this function set the display warning
int DisplayWarning(int drive, unsigned long long warning_id)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch (warning_id)
   {
      case PHASE_FIND_REQ_WRN_MASK: // PF
         BGVAR(u16_Display_Char)[0] = P_LET;
         BGVAR(u16_Display_Char)[1] = F_LET;
         BGVAR(u16_Display_Char_Number) = 2;
      break;
      
      case UNDER_VOLTAGE_WRN_MASK: // u
         BGVAR(u16_Display_Char)[0] = u_LET;
         BGVAR(u16_Display_Char_Number) = 1;
      break;

      case DRIVE_FOLDBACK_WRN_MASK: // F
      case MOTOR_FOLDBACK_WRN_MASK: // F
         BGVAR(u16_Display_Char)[0] = F_LET;
         BGVAR(u16_Display_Char_Number) = 1;
      break;

      case STO_WRN_MASK: // n
         BGVAR(u16_Display_Char)[0] = n_LET;
         BGVAR(u16_Display_Char)[1] = NONE;
         BGVAR(u16_Display_Char_Number) = 1;
      break;

      case MOTOR_OT_WRN_MASK: // H
         BGVAR(u16_Display_Char)[0] = H_LET;
         BGVAR(u16_Display_Char_Number) = 1;
      break;

      case DRIVE_POW_OT_WRN_MASK: // t
      case DRIVE_DIG_OT_WRN_MASK:
      case DRIVE_IPM_OT_WRN_MASK:
         BGVAR(u16_Display_Char)[0] = t_LET;
         BGVAR(u16_Display_Char_Number) = 1;
      break;

     case TM_BATT_LOW_VOLTAGE_WRN: // b
         BGVAR(u16_Display_Char)[0] = b_LET;
         BGVAR(u16_Display_Char_Number) = 1;
      break;

     case CW_HW_LS_WRN_MASK: // L1
         BGVAR(u16_Display_Char)[0] = L_LET;
         BGVAR(u16_Display_Char)[1] = ONE;
         BGVAR(u16_Display_Char_Number) = 2;
      break;

      case CCW_HW_LS_WRN_MASK: // L2
         BGVAR(u16_Display_Char)[0] = L_LET;
         BGVAR(u16_Display_Char)[1] = TWO;
         BGVAR(u16_Display_Char_Number) = 2;
      break;

      case CW_CCW_HW_LS_WRN_MASK: // L3
         BGVAR(u16_Display_Char)[0] = L_LET;
         BGVAR(u16_Display_Char)[1] = THREE;
         BGVAR(u16_Display_Char_Number) = 2;
      break;

      case CW_SW_LS_WRN_MASK: // L4
         BGVAR(u16_Display_Char)[0] = L_LET;
         BGVAR(u16_Display_Char)[1] = FOUR;
         BGVAR(u16_Display_Char_Number) = 2;
      break;

      case CCW_SW_LS_WRN_MASK: // L5
         BGVAR(u16_Display_Char)[0] = L_LET;
         BGVAR(u16_Display_Char)[1] = FIVE;
         BGVAR(u16_Display_Char_Number) = 2;
      break;

      case CW_CCW_SW_LS_WRN_MASK: // L6
         BGVAR(u16_Display_Char)[0] = L_LET;
         BGVAR(u16_Display_Char)[1] = SIX;
         BGVAR(u16_Display_Char_Number) = 2;
      break;

      case SININIT_WARNING_MASK: // r
         BGVAR(u16_Display_Char)[0] = r_LET;
         BGVAR(u16_Display_Char_Number) = 1;
      break;

      case BUS_AC_LOSS_WRN_MASK: // o
         BGVAR(u16_Display_Char)[0] = o_LET;
         BGVAR(u16_Display_Char_Number) = 1;
      break;

      case REGEN_OVER_LOAD_WRN_MASK: // c
         BGVAR(u16_Display_Char)[0] = c_LET;
         BGVAR(u16_Display_Char_Number) = 1;
      break;

      case RT_OVERLOAD_WRN_MASK: // _
         BGVAR(u16_Display_Char)[0] = LOWER_DASH;
         BGVAR(u16_Display_Char_Number) = 1;
      break;

      case SFB_MODE_EXCLUDE_WRN_MASK: // S1
         BGVAR(u16_Display_Char)[0] = S_LET;
         BGVAR(u16_Display_Char)[1] = ONE;
         BGVAR(u16_Display_Char_Number) = 2;
      break;

      case FAN_CIRCUIT_WRN_MASK:
         BGVAR(u16_Display_Char)[0] = o_LET;
         BGVAR(u16_Display_Char)[1] = ONE;
         BGVAR(u16_Display_Char_Number) = 2;
      break;
      
      
      
      default:
      return 0;
   }

   return 1;
}

//this function set the display fault by fault_id
void DisplayFault(int drive, unsigned long long s64_fault_id, int s16_set)
{
   unsigned int u16_index_in_table = 0;

   REFERENCE_TO_DRIVE;
   // when params over FOE active, dont display faults
   if (u16_Foe_ParamsFile_State != FOE_PARAMS_FILE_DWLD_IDLE)
   {
      return; 
   }
   
   while (s64_fault_id != (1LL << (unsigned long long)u16_index_in_table))
      u16_index_in_table++;

   u16_index_in_table += s16_set * 64;    // support another var of 64 faults. s16_set can be 0 or 1

   if (u16_index_in_table < 128)
   {
      BGVAR(u16_Display_Char)[0] = Display_Table[u16_index_in_table].u16_char1;
      BGVAR(u16_Display_Char)[1] = Display_Table[u16_index_in_table].u16_char2;
      BGVAR(u16_Display_Char)[2] = Display_Table[u16_index_in_table].u16_char3;
      BGVAR(u16_Display_Char)[3] = Display_Table[u16_index_in_table].u16_char4;
      BGVAR(u16_Display_Char)[4] = NONE;

      if (!BGVAR(u16_Display_Char)[1])
         BGVAR(u16_Display_Char_Number) = 1;
      else if ((!BGVAR(u16_Display_Char)[2]) || (BGVAR(u16_Display_Char)[1] == NONE))
         BGVAR(u16_Display_Char_Number) = 2;
      else if ((!BGVAR(u16_Display_Char)[3]) || (BGVAR(u16_Display_Char)[2] == NONE))
         BGVAR(u16_Display_Char_Number) = 3;
      else if (BGVAR(u16_Display_Char)[3] == NONE)
         BGVAR(u16_Display_Char_Number) = 4;
      else BGVAR(u16_Display_Char_Number) = 5;
   }
}
//this function does not update the value of serial communication addr rather 
//it stores the new user input value on nvram, when save and restart the addr would update
int SalSerComAddrWriteCommand(long long param,int drive)
{
   
   unsigned char u8_Tens = 0;
   unsigned char u8_Units = 0;
   drive += 0;
   if ( IS_RETRO_DISPLAY == 1 ) return NOT_PROGRAMMABLE;
   if ( ((param < 0) || (param > 99)) ) return VALUE_OUT_OF_RANGE;
   else
   {
        u8_Tens = (unsigned char)((unsigned char)param / (unsigned char)10);
        u8_Units = (unsigned char)((unsigned char)param % (unsigned char)10);
        s8_Drive_Address = (unsigned int)((u8_Tens << 4) + u8_Units);
   }
   return (SAL_SUCCESS);

}
// This function is called in response to ADDR command
//int SalRotarySwitch(void)
int SalRotarySwitch(long long *data,int drive)
{
   
   unsigned char u8_Tens = 0;
   unsigned char u8_Units = 0;
   drive+=0;
   if ((u16_Product == SHNDR_HW)  || (IS_EC_LITE_DRIVE))
   {
      PrintChar('0');
   }
   else 
   {
      u8_Tens = ((u8_Drive_Id & 0x00F0) >> 4);
      u8_Units = (u8_Drive_Id & 0x000F);
      *data = (unsigned int)((u8_Tens *10) + u8_Units);
   }
   return (SAL_SUCCESS);
}

/*************************************************************/
/**** From here start functions for the local HMI support ****/
/*************************************************************/



//**********************************************************
// Function Name: SalReadSevenSegmentDigit1To4
// Description:
//   This function reads the content of digits 1 to 4 from ther right,
//   of the local HMI display  on the Schnider LXM28 drive.
//**********************************************************
int SalReadSevenSegmentDigit1To4(long long *data, int drive)
{
   long s32_temp = 0;
   drive += 0;
   s32_temp |= (long)((*(int*)(FPGA_DSPLY_5_REG_ADD)) & 0x00FF);        // Digit on the right
   s32_temp |= ((long)((*(int*)(FPGA_DSPLY_4_REG_ADD)) & 0x00FF)) << 8; // Second digit from the right
   s32_temp |= ((long)((*(int*)(FPGA_DSPLY_3_REG_ADD)) & 0x00FF)) << 16; // Third digit from the right
   s32_temp |= ((long)((*(int*)(FPGA_DSPLY_2_REG_ADD)) & 0x00FF)) << 24; // Fourth digit from the right


   *data = (unsigned long long)s32_temp;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadSevenSegmentDigit5
// Description:
//   This function reads the content of digits 5 from ther right,
//   of the local HMI display  on the Schnider LXM28 drive.
//**********************************************************
int SalReadSevenSegmentDigit5(long long *data, int drive)
{
   long s32_temp = 0;
   drive += 0;
   s32_temp |= ((long)((*(int*)(FPGA_DISPLAY_REG_ADD)) & 0x00FF)); // Digit from the Left

   *data = (unsigned long long)s32_temp;

   return (SAL_SUCCESS);
}




//**********************************************************
// Function Name: IsPParamValueValid
// Description:
//    This function is meant to indicate that a certain setting is allowed for a
//    specific P-Parameter. This function is called in local HMI mode "Edit Parameter"
//    and serves as some kind of small data-base about what settings are permitted.
//    Setting a P-Parameter to a non-valid value is blocked in mode "Edit Parameter".
//
//       s64_value       - Value that would usually been set by the user
//       u16_p_parameter - P-Parameter which is supposed to be changed
//
//    Return value: 0 = Invalid value, 1 = Valid value
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int IsPParamValueValid (long long s64_value, unsigned int u16_p_parameter)
{
   int s16_p_param_value_valid = 1; // State that P-Parameter value is valid
   s64_value += 0;
   u16_p_parameter += 0;

   return s16_p_param_value_valid;
}


//**********************************************************
// Function Name: GetDriveStatusValue
// Description:
//    This function retruns the vaule to be displayed in the 7-segments
//    or by the P-parameters P0-09...P0-13 depending on the specification
//    of P0-02.
//    Some values of P0-02 are used as P11-xx shadow parameters, to record real-time data.
//    See function ConvertDriveStatusValue below.
//    s16_Value_To_Display - Value specified in P0-02 of the Lexium document.
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
long s32_ret_val_debug = -1;
long GetDriveStatusValue(int drive, int s16_Value_To_Display)
{
   // AXIS_OFF;

   long s32_ret_val = 0;
   long s32_temp = 0; // Temporary help variable
   int drive_backup = drive;
   long s32_Rounding_Offset;

   switch (s16_Value_To_Display)
   {
      case (0): // Motor feedback pulse number in user unit PUU
         UpdateS64PfbInPuuUnit(drive); // Update the variable
         s32_ret_val = (long)BGVAR(s64_PFB_In_PUU_Unit);
      break;

      case (1): // Input pulse number of pulse command in user unit PUU
         UpdateS64PositionCommandInPuuUnit(drive); // Update the variable
         s32_ret_val = (long)BGVAR(s64_Position_Command_In_PUU_Unit);
      break;

      case (2): // Position error counts in user unit PUU
         UpdateS64PositionErrorInPuuUnit(drive); // Update the variable
         s32_ret_val = (long)BGVAR(s64_Position_Error_In_PUU_Unit);
      break;

      case (3): // Motor feedback pulse number (encoder unit, 1280000 pulse per rev)
         UpdateS64PfbInPulseUnit(drive); // Update the variable
         s32_ret_val = (long)BGVAR(s64_PFB_In_Pulse_Unit);
      break;

      case (4): // Input pulse number of pulse command (encoder unit, 1280000 pulse per rev)
         UpdateS64PositionCommandInPulseUnit(drive); // Update the variable
         s32_ret_val = (long)BGVAR(s64_Position_Command_In_Pulse_Unit);
      break;

      case (5): // Position error counts (encoder unit, 1280000 pulse per rev)
         UpdateS64PositionErrorInPulseUnit(drive); // Update the variable
         s32_ret_val = (long) BGVAR(s64_Position_Error_In_Pulse_Unit);
      break;

      case (6): // Input frequency of the pulse commands in [pps]. Note that this entry is supposed to return "evaluated pules per second".
                // At this place the documentation of the Lexium 23 is wrong, which states that the unit is in [kpps].
                // The calculation corresponds to the function one in "CalcLexAnalogOutputVaules" for the analog
                // output mode 2, since this mode represents an analog output voltage according to the input-pulse frequency.
            switch (VAR(AX0_u8_Gear_Mode))
            {
               case 0: // Use "s32_Encoder_Follower2_Counter_Delta_1s"
               case 1: // Use "s32_Encoder_Follower2_Counter_Delta_1s"
               case 2: // Use "s32_Encoder_Follower2_Counter_Delta_1s"
                  s32_ret_val = BGVAR(s32_Encoder_Follower2_Counter_Delta_1s);
               break;
               default:  // Use "s32_Encoder_Follower3_Counter_Delta_1s"
                         // This is "Place holder for future use" : as of Dec 2013, only Encoder_Follower2 active for =S=
                  s32_ret_val = BGVAR(s32_Encoder_Follower3_Counter_Delta_1s);
               break;
            }

            // Reduce the value in case that the gearing input interpolation has been activated (see GEARINMODE).
            // Since the input pulse frequency is in reality lower by the factor in the formula below.
            s32_ret_val = s32_ret_val >> (VAR(AX0_u16_Qep_Out_Scale) - VAR(AX0_u16_Qep_In_Scale_Design));
      break;

      case (7): // Motor rotation speed [rpm] with 1 decimal place (600[rpm] = 6001 on the 7-segment)
         // Convert the actual velocity from [Counts32/125us] into [0.1*rpm].
         // 8000 * 60 * 10 / 2^32 = 0.0011175 = 18750/2^24
         s32_ret_val = (long)(((long long)LVAR(AX0_s32_Vel_Var_Fb_0) * 18750) >> 24);
      break;

      case (8): // Speed input command in Volt --> Analog input value 1 in [V] in x.yy format (ANIN1)
         // Convert the internal analog input 1 variable into [mV] via the global units-conversion fix-shift values
         s32_ret_val = (long)( ((long long)VAR(AX0_s16_An_In_1_Filtered) * BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix) >>
                                             BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr );
         // Multiply the value with 0.1, since this is the way the voltage is represented by the Lexium (x.yy format)
         s32_ret_val = (long)(((long long)s32_ret_val * 26215) >> 18);
      break;

      case (9):  // Speed (input) command in [rpm]
      case (50): // Speed (input) command in [0.1*rpm]
      {
         long s32_Rounding_Offset = 0; // Rounding offset variable for data-representation (1 count less). This offset has been identified as needed during testing

         // First convert the the velocity command value from the unit "in the loop" (22750=VLIM)
         // into the unit "out of the loop" in [Counts32/125us].
         s32_ret_val = (long)(((long long)VAR(AX0_s16_Vel_Var_Ref_0) * LVAR(AX0_s32_In_To_Out_Vel_Fix)) >> VAR(AX0_u16_In_To_Out_Vel_Shr));
         if (s16_Value_To_Display == 9) // Unit [rpm]
         {
            s32_Rounding_Offset = 67108864L; // 0.5*2^27
            // Now convert the command velocity from [Counts32/125us] into [rpm].
            // 8000 * 60 / 2^32 = 0.00011175 = 15000/2^27 (with 67108864L >> 27 = 0.5 to avoid being 1 [Count] away due to rounding issues).
            s32_ret_val = (long)(((long long)s32_ret_val * 15000 + s32_Rounding_Offset) >> 27);
         }
         else // Unit [0.1*rpm]
         {
            s32_Rounding_Offset = 8388608L; // 0.5*2^24
            // Now convert the command velocity from [Counts32/125us] into [0.1*rpm].
            // 10 * 8000 * 60 / 2^32 = 0.0011175 = 18750/2^24 (with 8388608L >> 24 = 0.5 to avoid being 1 [Count] away due to rounding issues).
            s32_ret_val = (long)(((long long)s32_ret_val * 18750 + s32_Rounding_Offset) >> 24);
         }
      }
      break;

      case (10): // Torque input command in Volt --> Analog input value 2 in [V] in x.yy format (ANIN2)
         // Convert the internal analog input 2 variable into [mV] via the global units-conversion fix-shift values
         s32_ret_val = (long)( ((long long)VAR(AX0_s16_An_In_2_Filtered) * BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix) >>
                                             BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr );
         // Multiply the value with 0.1, since this is the way the voltage is represented by the Lexium (x.yy format)
         s32_ret_val = (long)(((long long)s32_ret_val * 26215) >> 18);
      break;

      case (11): // Torque input command in [%]
      case (53): // Torque input command in [0.1*%]
      {
         s32_Rounding_Offset = 0; // Rounding offset variable for data-representation (1 count less)

         // Avoid problems of the representation of one count less
         if (VAR(AX0_s16_Icmd) < 0)
         {
            s32_Rounding_Offset = -BGVAR(s32_Motor_I_Cont_Internal)>>1;
         }
         else
         {
            s32_Rounding_Offset = BGVAR(s32_Motor_I_Cont_Internal)>>1;
         }


         if (s16_Value_To_Display == 11) // Unit [%]
         {
            // Motor I cont represents 100.0%. Both variables ("AX0_s16_Icmd" and "s32_Motor_I_Cont_Internal") are both in
            // internal current units (DIPEAK = 26214). Add 0.5 * motor I cont to avoid rounding issues.
            s32_ret_val = (((long)VAR(AX0_s16_Icmd) * 100) + s32_Rounding_Offset) / BGVAR(s32_Motor_I_Cont_Internal);
         }
         else
         {// Unit [0.1 %]
            // Motor I cont represents 100.0%. Both variables ("AX0_s16_Icmd" and "s32_Motor_I_Cont_Internal") are both in
            // internal current units (DIPEAK = 26214). Add 0.5 * motor I cont to avoid rounding issues.
            s32_ret_val = (((long)VAR(AX0_s16_Icmd) * 1000) + s32_Rounding_Offset) / BGVAR(s32_Motor_I_Cont_Internal);
         }

      }
      break;

      case (12): // Motor Load Percent.
         s32_ret_val = BGVAR(u32_Motor_Load_Pct);
      break;

      case (13): // Motor Load Percent Max
         s32_ret_val = BGVAR(u32_Motor_Load_Pct_Max);
      break;

      case (14): // Main circuit voltage in [V]
         s32_ret_val = (long)BGVAR(u16_Vbus_Volts);
      break;

      case (15): // Ratio of load inertia to motor inertia (CDHD-command LMJR, P1-37)
         ExecutePParam(drive, (unsigned int)137, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (16): // IGBT temperature
         if (VAR(AX0_u16_IPM_OT_DC_Enable) == 1)
         {
            s32_ret_val = (long)BGVAR(u16_IPM_Temperature);
         }
         else
         {
            drive = 0;
            s32_ret_val = (long)BGVAR(s16_Power_Temperature_Deg);
            drive = drive_backup;
         }
      break;
      case (17): //resonance frequencies from nlantivibhz2 and nlantivibhz3
         // If f1 is 123.4 Hz, it is 123400 internally
         // And If f2 is 345.6, Than final result will disply 345601234
          s32_ret_val = (u32_Antivib3_Fcenter * 1000) +
                        (((long long)u32_Pe_Filt_Fcenter * 20972) >> 21); //Divide by 100
      break;
      case (18): //Absolute Pulse Number : MECHANGLE, with units translation from [-32768..+32767] to [-5000..+4999]
         s32_ret_val = ((long)AX0_s16_Feedback_Comm_Pos * 5000) >> 15 ;
      break;

      case (19): // Content of P0-25
         // Perform 32-bit read access to the P-parameter P0-25
         ExecutePParam(drive, (unsigned int)25, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (20): // Content of P0-26
         // Perform 32-bit read access to the P-parameter P0-26
         ExecutePParam(drive, (unsigned int)26, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (21): // Content of P0-27
         // Perform 32-bit read access to the P-parameter P0-27
         ExecutePParam(drive, (unsigned int)27, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (22): // Content of P0-28
         // Perform 32-bit read access to the P-parameter P0-28
         ExecutePParam(drive, (unsigned int)28, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (23): // Content of P0-09
         // Perform 32-bit read access to the P-parameter P0-09
         ExecutePParam(drive, (unsigned int)9, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (24): // Content of P0-10
         // Perform 32-bit read access to the P-parameter P0-10
         ExecutePParam(drive, (unsigned int)10, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (25): // Content of P0-11
         // Perform 32-bit read access to the P-parameter P0-11
         ExecutePParam(drive, (unsigned int)11, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (26): // Content of P0-12
         // Perform 32-bit read access to the P-parameter P0-12
         ExecutePParam(drive, (unsigned int)12, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (27): // Control-board temperature
         drive = 0;
         s32_ret_val = (long)BGVAR(s16_Control_Temperature_Deg);
         drive = drive_backup;
      break;

      case (39): // Digital input status (Content of P4-07)
         // Perform 32-bit read access to the P-parameter P4-07
         ExecutePParam(drive, (unsigned int)407, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (40): // Digital output status (Content of P4-09)
         // Perform 32-bit read access to the P-parameter P4-09
         ExecutePParam(drive, (unsigned int)409, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (41): // Drive status (Content of P0-46)
         // Perform 32-bit read access to the P-parameter P0-46
         ExecutePParam(drive, (unsigned int)46, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (42): // Currently adjusted mode of operation (which is active after save & re-boot). requirement raised by J. Fiess.
         s32_ret_val = BGVAR(u16_P1_01_CTL_Current);
      break;

      case (49): // Pulse count input - Raw-pulses in Pt mode, represented in P5-18
         // Perform 32-bit read access to the P-parameter P5-18
         ExecutePParam(drive, (unsigned int)518, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
      break;

      case (54): // Feedback torque in unit 0.1% with 100%=MICONT
         // 1000 * actual_current / motor_continuous
         s32_ret_val = 1000 * (long)VAR(AX0_s16_Crrnt_Q_Act_0) / BGVAR(s32_Motor_I_Cont_Internal);
      break;

      case (55): // Feedback torque in 0.01[A]
         // Since "s32_Drive_I_Peak" is given in [mA] but the returned value needs to be in [10*mA],
         // divide by 10*26214 in order to get the correct value.
         s32_ret_val = (long long)VAR(AX0_s16_Crrnt_Q_Act_0) * BGVAR(s32_Drive_I_Peak) / 262140;
      break;


      case (77): // PTPVCMD   in [0.1*rpm]

         s32_ret_val = GetS32ValueByUnit(drive, AX0_s32_Pos_Vcmd, VELOCITY_0_1_RPM_PTP_CONVERSION,1);
      break;







      case (96): // Drive software version number P0-00 and P5-00
         // Perform 32-bit read access to the P-parameter P0-00
         ExecutePParam(drive, (unsigned int)0, OP_TYPE_READ, &s32_ret_val, LEX_CH_LOCAL_HMI);
         // Perform 32-bit read access to the P-parameter P5-00
         ExecutePParam(drive, (unsigned int)500, OP_TYPE_READ, &s32_temp, LEX_CH_LOCAL_HMI);
         // Now combine both variables. High word P5-00, low-word P0-00. Request A. Badura.
         s32_ret_val = s32_ret_val + (s32_temp<<16);
      break;

      case (111): // Get the "Drive alarm code"
         s32_ret_val = GetLexiumFaultNumber (drive, 0LL);
      break;

      default: // Unsupported mode, switch off the 7-segments
         s32_ret_val = 0;
      break;
   }

   s32_ret_val_debug = s32_ret_val;
   return (s32_ret_val);
}


//**********************************************************
// Function Name: SalWriteDriveStatusCommand
// Description:
//    This function sets the P-parameter P0-02 (STS = Drive status)
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteDriveStatusCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s16_P0_02_Drive_Status) = BGVAR(s16_Hmi_Mode_Dspl_Stat) = (int)lparam;

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteDisplayStatusMonitor
// Description:
//    This function writes the P-param P0-17 to P0-21
//    to an array.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteDisplayStatusMonitor(int drive)
{
   long long index = s64_Execution_Parameter[0];
   long long value = s64_Execution_Parameter[1];

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((index < 0LL) || (index > 4LL))
       return (VALUE_OUT_OF_RANGE);

   // set error codes to be compatiable with canopen (fix bug: IPR 340)
   if (value < 0LL) return VALUE_TOO_LOW;
   if (value > 123LL) return VALUE_TOO_HIGH;

   BGVAR(s16_P0_17to21_Display_Status_Monitor)[index] = (int)value;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadDisplayStatusMonitor
// Description:
//    This function reads the P-param P0-17 to P0-21
//    from an array.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadDisplayStatusMonitor(long long *data,int drive)
{
   long long index = s64_Execution_Parameter[0];

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((index < 0LL) || (index > (long long)4))
       return (VALUE_OUT_OF_RANGE);

   *data = (BGVAR(s16_P0_17to21_Display_Status_Monitor)[index]);
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalReadStatusMonitor
// Description:
//    This function reads the P-param P0-09 to P0-13.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadStatusMonitor(long long *data,int drive)
{
   long long index = s64_Execution_Parameter[0];

   if ((index < 0LL) || (index > (long long)4))
       return (VALUE_OUT_OF_RANGE);

   // Here avoid a dead-lock that the Drive does not stuck in the boot process
   // due to calling the same function endless. Solution for BYang00000666.
   // If access to P0-09 via P0-17...P0-21=23 ||
   // If access to P0-10 via P0-17...P0-21=24 ||
   // If access to P0-11 via P0-17...P0-21=25 ||
   // If access to P0-12 via P0-17...P0-21=26
   if( (BGVAR(s16_P0_17to21_Display_Status_Monitor)[index] == 23) ||
       (BGVAR(s16_P0_17to21_Display_Status_Monitor)[index] == 24) ||
       (BGVAR(s16_P0_17to21_Display_Status_Monitor)[index] == 25) ||
       (BGVAR(s16_P0_17to21_Display_Status_Monitor)[index] == 26)
      )
   {
      *data = 0; // Just return 0 and do not call GetDriveStatusValue since otherwise we come back to this place in the code.
   }
   else
   {
      *data = (long long)GetDriveStatusValue(drive, BGVAR(s16_P0_17to21_Display_Status_Monitor)[index]);
   }

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: GetLocalHmiDigitCode
// Description:
//          This function is used for the Lexium local HMI
//          functionality and returns the digit-code of a
//          character value for the seven segment display.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
unsigned int GetLocalHmiDigitCode (char digit)
{
   unsigned int u16_ret_val = NONE;
   switch (digit)
   {
      case '-': u16_ret_val = DASH_CHAR;  break;      // -
      case '0': u16_ret_val = ZERO;       break;      // 0
      case '1': u16_ret_val = ONE;        break;      // 1
      case '2': u16_ret_val = TWO;        break;      // 2
      case '3': u16_ret_val = THREE;      break;      // 3
      case '4': u16_ret_val = FOUR;       break;      // 4
      case '5': u16_ret_val = FIVE;       break;      // 5
      case '6': u16_ret_val = SIX;        break;      // 6
      case '7': u16_ret_val = SEVEN;      break;      // 7
      case '8': u16_ret_val = EIGHT;      break;      // 8
      case '9': u16_ret_val = NINE;       break;      // 9
      case 'A': u16_ret_val = A_LET;      break;      // A
      case 'B': u16_ret_val = b_LET;      break;      // b
      case 'b': u16_ret_val = b_LET;      break;      // b
      case 'c': u16_ret_val = c_LET;      break;      // c
      case 'C': u16_ret_val = C_LET;      break;      // C
      case 'D': u16_ret_val = d_LET;      break;      // d
      case 'd': u16_ret_val = d_LET;      break;      // d
      case 'e': u16_ret_val = e_LET;      break;      // E
      case 'E': u16_ret_val = E_LET;      break;      // E
      case 'F': u16_ret_val = F_LET;      break;      // F
      case 'G': u16_ret_val = G_LET;      break;      // G
      case 'h': u16_ret_val = h_LET;      break;      // h
      case 'H': u16_ret_val = h_LET;      break;      // H
      case 'I': u16_ret_val = I_LET;      break;      // I
      case 'J': u16_ret_val = J_LET;      break;      // J
      case 'K': u16_ret_val = H_LET;      break;      // In a Lexium 28 a capital K is shown as a capital H of the CDHD (digit code comes from, A. Badura).
      case 'L': u16_ret_val = L_LET;      break;      // L
      case 'N': u16_ret_val = n_LET;      break;      // N
      case 'n': u16_ret_val = n_LET;      break;      // n
      case 'M': u16_ret_val = N_LET;      break;      // In a Lexium 28 a capital M is shown as a capital N of the CDHD (digit code comes from, A. Badura).
      case 'O': u16_ret_val = ZERO;       break;      // O = 0
      case 'o': u16_ret_val = o_LET;      break;      // o
      case 'P': u16_ret_val = P_LET;      break;      // P
      case 'p': u16_ret_val = P_LET;      break;      // P
      case 'q': u16_ret_val = q_LET;      break;      // q
      case 'r': u16_ret_val = r_LET;      break;      // r
      case 'S': u16_ret_val = S_LET;      break;      // S
      case 't': u16_ret_val = t_LET;      break;      // t
      case 'U': u16_ret_val = V_LET;      break;      // U
      case 'u': u16_ret_val = u_LET;      break;      // u
      case 'V': u16_ret_val = V_LET;      break;      // V
      case 'v': u16_ret_val = v_LET;      break;      // v
      case 'W': u16_ret_val = W_LET;      break;      // W
      case 'Y': u16_ret_val = Y_LET;      break;      // Y
      default:  /* Return NONE* (do nothing) */ break;// E.g. for ' '
   }
   return u16_ret_val;
}

//**********************************************************
// Function Name: GetLocalHmiHexDisplayValue
// Description:
//          This function is called/used for the Lexium local HMI
//          function and returns the digit-code for the seven segment
//          display for a corresponding 32-bit value.
//
//    u32_value - Value to be displayed in a hex-format on the 7-segment.
//    u16_Pointer_To_Array - Pointer to an array of type U16 with 10 entries.
//    s16_Buffer_Size - For protection to not exceed the buffer provide also the
//                      length of the array where the pointer points to (e.g. 10
//                      for 10 buffer-entries).
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void GetLocalHmiHexDisplayValue(unsigned long u32_value, unsigned int *u16_Pointer_To_Array, int s16_Buffer_Size)
{
   unsigned int u16_index;

   char u8_digit;

   for (u16_index=0; u16_index<10; u16_index++)
   {
      if(u16_index >= s16_Buffer_Size) // Do not exceed buffer size
      {
         return;
      }

      if(u16_index == 4)
      {
         // Indicate low-word with an 'L'
         u16_Pointer_To_Array[u16_index] = GetLocalHmiDigitCode('L');
      }
      else  if(u16_index == 9)
      {
         // Indicate high-word with an 'h'
         u16_Pointer_To_Array[u16_index] = GetLocalHmiDigitCode('h');
      }
      else
      {
         // Get the character to be displayed
         u8_digit = u32_value % 16;

         if(u8_digit<=9)
         {
            u16_Pointer_To_Array[u16_index] = GetLocalHmiDigitCode('0' + u8_digit);
         }
         else
         {
            u16_Pointer_To_Array[u16_index] = GetLocalHmiDigitCode('A' + u8_digit - 10);
         }

         u32_value = u32_value >> 4; // Divide by 16
      }
   }
}

//**********************************************************
// Function Name: GetLocalHmiDecimalDisplayValue
// Description:
//          This function is called/used for the Lexium local HMI
//          function and returns the digit-code for the seven segment
//          display for a corresponding 32-bit value.
//
//    s32_value - Value to be displayed on the 7-segment
//    u16_Pointer_To_Array - Pointer to an array of type U16 with 10 entries.
//    s16_Buffer_Size - For protection to not exceed the buffer provide also the
//                      length of the array where the pointer points to (e.g. 10
//                      for 10 buffer-entries).
//    s16_Involve_Decimal_Points - A value unequal 0 indicates if the function is supposed
//                                 to involve the decimal points for representing negative
//                                 values or to distinguish between lower 5-digits and upper
//                                 5 digits.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void GetLocalHmiDecimalDisplayValue(long s32_value, unsigned int *u16_Pointer_To_Array, int s16_Buffer_Size, int s16_Involve_Decimal_Points)
{
   unsigned int u16_index;
   char u8_digit;

   unsigned long u32_Temp_Value;
   if(s32_value < 0)
   {
      u32_Temp_Value = -s32_value;
   }
   else
   {
      u32_Temp_Value = s32_value;
   }


   for (u16_index=0; u16_index<10; u16_index++)
   {
      if(u16_index >= s16_Buffer_Size) // Do not exceed buffer size
      {
         break; // Leave for loop
      }

      u8_digit = u32_Temp_Value % 10; // Extract value to be displayed

      u16_Pointer_To_Array[u16_index] = GetLocalHmiDigitCode('0' + u8_digit);

      u32_Temp_Value /= 10; // Divide by 10
   }

   if (s16_Involve_Decimal_Points != 0)
   {
      // Decimal point in digit 0 displays lower 5 digits.
      u16_Pointer_To_Array[0] &= DC_POINT; // Set bit 7 to Low in order to light up the decimal point

      // Decimal point in digit 6 displays upper 5 digits
      u16_Pointer_To_Array[6] &= DC_POINT; // Set bit 7 to Low in order to light up the decimal point

      // If the value to be displayed is negative
      if (s32_value < 0)
      {
         // Light up the decimal points in digit 3, 4, 8 and 9
         u16_Pointer_To_Array[3] &= DC_POINT; // Set bit 7 to Low in order to light up the decimal point

         // IPR#1384 FIX: remove the sign negative indication from digit 4 in order to use it as modbus comm indicator
         // u16_Pointer_To_Array[4] &= DC_POINT; // Set bit 7 to Low in order to light up the decimal point

         u16_Pointer_To_Array[8] &= DC_POINT; // Set bit 7 to Low in order to light up the decimal point

         // IPR#1384 FIX: remove the sign negative indication from digit 9 in order to use it as modbus comm indicator
         //u16_Pointer_To_Array[9] &= DC_POINT; // Set bit 7 to Low in order to light up the decimal point
      }
   }
}

//**********************************************************
// Function Name: GetLocalHmiCombinedDecimalDisplayValue
// Description:
//          This function is called/used for the Lexium local HMI
//          function and returns the digit-code for the seven segment
//          display for a corresponding 32-bit value in combined decimal fomat.
//          Combined decimal format means:
//          The lower 16-bits are displayed as unsigned decimal in the lower 5 digits.
//          The upper 16-bits are displayed as unsigned decimal in the upper 5 digits.
//
//          Example
//          -------
//          0x ABCD 5432 is displayed as "43981" in the upper 5 digits and "21554" in the lower 5 digits.
//
//    s32_value - Value to be displayed on the 7-segment
//    u16_Pointer_To_Array - Pointer to an array of type U16 with 10 entries.
//    s16_Buffer_Size - For protection to not exceed the buffer provide also the
//                      length of the array where the pointer points to (e.g. 10
//                      for 10 buffer-entries).
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void GetLocalHmiCombinedDecimalDisplayValue(long s32_value, unsigned int *u16_Pointer_To_Array, int s16_Buffer_Size)
{
   unsigned int u16_index;
   char u8_digit;

   unsigned long u32_temp_value;

   for (u16_index=0; u16_index<10; u16_index++)
   {
      // Load the value to be converted
      if((u16_index == 0) || (u16_index == 5))
      {
         // Type cast into unsigned variable since the value for combined decimal
         // is supposed to be unsigned for the upper and the lower 16-bit value.
         u32_temp_value = (unsigned long)s32_value;

         if(u16_index == 0) // In the 1st iteration use the lower 16-bits
         {
            u32_temp_value = u32_temp_value & 0x0000FFFF;
         }
         else if(u16_index == 5) // In the 2nd iteration use the upper 16-bits
         {
            u32_temp_value = (u32_temp_value >> 16) & 0x0000FFFF;
         }
      }

      if(u16_index >= s16_Buffer_Size) // Do not exceed buffer size
      {
         break; // Leave for loop
      }

      u8_digit = u32_temp_value % 10; // Extract value to be displayed

      u16_Pointer_To_Array[u16_index] = GetLocalHmiDigitCode('0' + u8_digit);

      u32_temp_value /= 10; // Divide by 10
   }

   // Decimal point in digit 0 displays lower 5 digits.
   u16_Pointer_To_Array[0] &= DC_POINT; // Set bit 7 to Low in order to light up the decimal point

   // Decimal point in digit 6 displays upper 5 digits
   u16_Pointer_To_Array[6] &= DC_POINT; // Set bit 7 to Low in order to light up the decimal point
}

//**********************************************************
// Function Name: GetLocalHmiStringDisplayValue
// Description:
//          This function
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void GetLocalHmiStringDisplayValue(char *s8_Source_Ptr, unsigned int *u16_Destination_Ptr, int s16_Buffer_Size)
{
   int s16_index  = 0;
   unsigned int u16_temp_value = 0;
   unsigned int *u16_Temp_Destination_Ptr = u16_Destination_Ptr;

   // Copy the digit-code into the destination buffer
   while ((*s8_Source_Ptr != '\0') && (s16_index <s16_Buffer_Size))
   {
      *u16_Temp_Destination_Ptr = GetLocalHmiDigitCode(*s8_Source_Ptr);
      u16_Temp_Destination_Ptr++;
      s8_Source_Ptr++;
      s16_index++;
   }

   // The string is now mirror-inverted in the destination array. Re-Construct
   // the array (first entry at last place), second entry at second last place
   // and so on...
   s16_index = s16_index >> 1;  // Divide by 2
   u16_Temp_Destination_Ptr--;  // Go back to highest valid buffer entry
   while (s16_index > 0)
   {
      u16_temp_value = *u16_Temp_Destination_Ptr;
      *u16_Temp_Destination_Ptr = *u16_Destination_Ptr;
      *u16_Destination_Ptr = u16_temp_value;
      u16_Destination_Ptr++;
      u16_Temp_Destination_Ptr--;
      s16_index--;
   }
}


//**********************************************************
// Function Name: LocalHmiUpdateKeyStates
// Description:
//          This function needs to be called continuously
//          and evaluates the state of the 5 buttons on the
//          Lexium. A button-state = true (e.g. for a rising
//          edge or for a pressed button) will be indicated for
//          one function call. At the second function call, the
//          state disappears. In case of a continuously pressed
//          button, the state PRESSED will appear after 5[ms]
//          again for one cycle.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void LocalHmiUpdateKeyStates (drive)
{
   unsigned int u16_Buttons_Edges;
   unsigned int u16_Buttons_Rising_Edges;
   unsigned int u16_P4_08_Push_Button_Status_temp = 0x0001;//u16_P4_08_Push_Button_Status;
   static long s32_Button_S_Pressed_Timer1      = 0;
   
   static long s32_status_temp_timer1           = 0;
   static long s32_status_temp_timer2           = 0;
   
   static long s32_Button_S_M_Pressed_Timer1    = 0;
   static long s32_Button_S_M_Pressed_Timer2    = 0;
   
   static long u16_Local_Hmi_Prev_Button_State = 0x0000;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Get the state of the buttons and detect rising edges
   //if (u16_Hmi_Key_Disable == 0) BGVAR(u16_P4_08_Push_Button_Status) = (*(int*)(FPGA_SW_REG_ADD)) /*-0x74*/ & 0x001F;  // Get the status of the push-buttons
   
   if ( PassedTimeMS(11, s32_status_temp_timer1) && u16_Hmi_Key_Disable == 0 )
   {
      s32_status_temp_timer1 = Cntr_1mS;
      BGVAR(u16_Push_Button_Status_Temp1) = (*(int*)(FPGA_SW_REG_ADD))& 0x001F;
   }
   if ( PassedTimeMS(17, s32_status_temp_timer2) && u16_Hmi_Key_Disable == 0 )
   {
      s32_status_temp_timer2 = Cntr_1mS;
      BGVAR(u16_Push_Button_Status_Temp2) = (*(int*)(FPGA_SW_REG_ADD))& 0x001F;

   }
   if ( BGVAR(u16_Push_Button_Status_Temp2) == BGVAR(u16_Push_Button_Status_Temp1)&& u16_Hmi_Key_Disable == 0 )
   {
      BGVAR(u16_P4_08_Push_Button_Status) = BGVAR(u16_Push_Button_Status_Temp1);
   }
   else
   {
      BGVAR(u16_Push_Button_Status_Temp1) = 0;
      BGVAR(u16_Push_Button_Status_Temp2) = 0;
   }
   
   BGVAR(s32_Local_Hmi_No_Button_Pressed_Timer) = Cntr_1mS;
         
   // The following if-condition is based on a request by A.Badura when being in lHMI mode JogMove but here applied to all modes.
   // All other keys are ignored in case that the user presses the UP/DOWN buttons, so no other keys are evaluated by that time.
   if((BGVAR(u16_P4_08_Push_Button_Status) & (BUTTON_UP_STATUS | BUTTON_DOWN_STATUS)) != 0) // If the user presses UP/DOWN
   {
      // Ignore the other keys --> Requested for JogMode but applied to all modes in order to be consistent.
      BGVAR(u16_P4_08_Push_Button_Status) = BGVAR(u16_P4_08_Push_Button_Status) & (BUTTON_UP_STATUS | BUTTON_DOWN_STATUS);
   }

   u16_Buttons_Edges                   = BGVAR(u16_P4_08_Push_Button_Status) ^ u16_Local_Hmi_Prev_Button_State; // Detect edges (rising/falling)
   u16_Buttons_Rising_Edges            = BGVAR(u16_P4_08_Push_Button_Status) & u16_Buttons_Edges;               // Filter rising edges
   u16_Local_Hmi_Prev_Button_State     = BGVAR(u16_P4_08_Push_Button_Status);                                   // Save the state of the digital inputs variable



   /*** Now evaluate the state of each button and set the flags accordingly.  ***/
   /*** It is up to the Local HMI state machine functions to use the flags in ***/
   /*** the variable, that contains the evaluated state or to ignore them.    ***/
   if ( u16_Hmi_Key_Long_Press == 1 ) BGVAR(u16_P4_08_Push_Button_Status) = u16_P4_08_Push_Button_Status_temp;
   switch(BGVAR(u16_P4_08_Push_Button_Status))
   {
      case (BUTTON_M_S_STATUS):   // 
         BGVAR(u32_Local_Hmi_Evaluated_Button_State) = BGVAR(u16_P4_08_Push_Button_Status) | (u16_Buttons_Rising_Edges << 5);

         if (PassedTimeMS(500L, s32_Button_S_M_Pressed_Timer1))
         {
            s32_Button_S_M_Pressed_Timer1 = Cntr_1mS-50L;
            (BGVAR(u32_Local_Hmi_Evaluated_Button_State) |= BGVAR(u16_P4_08_Push_Button_Status) << 9);
         }
         if (PassedTimeMS(2000L, s32_Button_S_M_Pressed_Timer2))
         {

            s32_Button_S_M_Pressed_Timer2 = Cntr_1mS-5000L-50L;
            u16_Hmi_Key_Long_Press = 0 ;
            (BGVAR(u32_Local_Hmi_Evaluated_Button_State) |= BGVAR(u16_P4_08_Push_Button_Status) << 11);
         }
         BGVAR(s32_Local_Hmi_No_Button_Pressed_Timer) = Cntr_1mS;
         break;
      case (BUTTON_M_STATUS):     // Only one of the 5 buttons is pressed
         BGVAR(u32_Local_Hmi_Evaluated_Button_State) = BGVAR(u16_P4_08_Push_Button_Status) | (u16_Buttons_Rising_Edges << 5);
         BGVAR(s32_Local_Hmi_No_Button_Pressed_Timer) = Cntr_1mS;
         
      break;

      case (BUTTON_S_STATUS):     // Only one of the 5 buttons is pressed
         BGVAR(u32_Local_Hmi_Evaluated_Button_State) = BGVAR(u16_P4_08_Push_Button_Status) | (u16_Buttons_Rising_Edges << 5);
         if (PassedTimeMS(500L, s32_Button_S_Pressed_Timer1))
         {
            s32_Button_S_Pressed_Timer1 = Cntr_1mS-50L;
            u16_Hmi_Key_Long_Press = 0 ;
            BGVAR(u32_Local_Hmi_Evaluated_Button_State) |= BGVAR(u16_P4_08_Push_Button_Status) << 9;
         }

         BGVAR(s32_Local_Hmi_No_Button_Pressed_Timer) = Cntr_1mS;
      break;

      case (BUTTON_UP_STATUS):    // Only one of the 5 buttons is pressed
      case (BUTTON_DOWN_STATUS):  // Only one of the 5 buttons is pressed
         BGVAR(u32_Local_Hmi_Evaluated_Button_State) = BGVAR(u16_P4_08_Push_Button_Status) | (u16_Buttons_Rising_Edges << 5);
      break;

      default: // 0 or more than 1 button pressed. More than one buttons at the same time will be ignored.
         s32_Button_S_Pressed_Timer1      = Cntr_1mS;
         
         s32_Button_S_M_Pressed_Timer2    = Cntr_1mS;
         s32_Button_S_M_Pressed_Timer1    = Cntr_1mS;
         BGVAR(u32_Local_Hmi_Evaluated_Button_State) = 0x00000000;
         BGVAR(u16_Push_Button_Status_Temp1) = 0L;
         BGVAR(u16_Push_Button_Status_Temp2) = 0L;
         
         
      break;
   }
   if ( u16_Hmi_Key_Long_Press == 1 ) BGVAR(u16_P4_08_Push_Button_Status) = u16_P4_08_Push_Button_Status_temp;
   if ( (u16_Hmi_Key_Disable == 1) && (u16_Hmi_Key_Long_Press == 0) ) BGVAR(u16_P4_08_Push_Button_Status) = 0;
}
//**********************************************************
// Function Name: ButtonEntRisingEdge
// Description:
//          Returns a true in case that there was a rising-edge
//          detected at button ENT.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonEntRisingEdge(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return (BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_ENT_RISING_EDGE);
}
//**********************************************************
// Function Name: ButtonUpPressed
// Description:
//          Returns the instantaneous value of the UP button
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonUpPressed(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return (BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_UP_STATUS);
}
//**********************************************************
// Function Name: ButtonUpRisingEdge
// Description:
//          Returns a true in case that there was a rising-edge
//          detected at button UP.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonUpRisingEdge(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return (BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_UP_RISING_EDGE);
}
//**********************************************************
// Function Name: ButtonUpPressed100ms
// Description:
//          Returns a true for one background cycle after 1.1[s]
//          and every further 100[ms] in case that there was a
//          continuously pressed button UP.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonUpPressed100ms(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return (BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_UP_PRESSED_100MS);
}
//**********************************************************
// Function Name: ButtonUpPressed500ms
// Description:
//          Returns a true for one background cycle after 1.5[s]
//          and every further 500[ms] in case that there was a
//          continuously pressed button UP.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonUpPressed500ms(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return (BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_UP_PRESSED_500MS);
}
//**********************************************************
// Function Name: ButtonDownPressed
// Description:
//          Returns the instantaneous value of the DOWN button
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonDownPressed(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return (BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_DOWN_STATUS);
}
//**********************************************************
// Function Name: ButtonDownRisingEdge
// Description:
//          Returns a true in case that there was a rising-edge
//          detected at button DOWN.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonDownRisingEdge(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return (BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_DOWN_RISING_EDGE);
}
//**********************************************************
// Function Name: ButtonDownPressed100ms
// Description:
//          Returns a true for one background cycle after 1.1[s]
//          and every further 100[ms] in case that there was a
//          continuously pressed button DOWN.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonDownPressed100ms(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return (BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_DOWN_PRESSED_100MS);
}
//**********************************************************
// Function Name: ButtonDownPressed500ms
// Description:
//          Returns a true for one background cycle after 1.5[s]
//          and every further 500[ms] in case that there was a
//          continuously pressed button DOWN.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonDownPressed500ms(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return (BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_DOWN_PRESSED_500MS);
}
//**********************************************************
// Function Name: ButtonMRisingEdge
// Description:
//          Returns a true in case that there was a rising-edge
//          detected at button M.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonMRisingEdge(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return ( (BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_M_RISING_EDGE) && !(BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_S_RISING_EDGE) );
}
//**********************************************************
// Function Name: ButtonSRisingEdge
// Description:
//          Returns a true in case that there was a rising-edge
//          detected at button S.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonSRisingEdge(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   return ( BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_S_RISING_EDGE && !(BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_M_RISING_EDGE) );
}

//**********************************************************
// Function Name: ButtonSPressed1000ms
// Description:
//          Returns a true for one background cycle after 1000[ms]
//          and every further 1000[ms] in case that there was a
//          continuously pressed button S.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int ButtonSPressed500ms(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   return (BGVAR(u32_Local_Hmi_Evaluated_Button_State) & BUTTON_S_PRESSED_500MS);
}


int ButtonMSRisingEdge(drive)
{
   unsigned int temp = BGVAR(u32_Local_Hmi_Evaluated_Button_State);
   REFERENCE_TO_DRIVE;     // Defeat compilation error   

   return ( (temp & BUTTON_S_RISING_EDGE) && (temp & BUTTON_M_RISING_EDGE) );
}
int ButtonMSPressed500ms(drive)
{
   unsigned int temp = BGVAR(u32_Local_Hmi_Evaluated_Button_State);
   REFERENCE_TO_DRIVE;     // Defeat compilation error   
   return ( (temp & BUTTON_S_PRESSED_500MS) && (temp & BUTTON_M_PRESSED_500MS) );
}
int ButtonMSPressed2000ms(drive)
{
   unsigned int temp = BGVAR(u32_Local_Hmi_Evaluated_Button_State);
   REFERENCE_TO_DRIVE;     // Defeat compilation error   
   return ( (temp & BUTTON_S_PRESSED_2000MS) && (temp & BUTTON_M_PRESSED_2000MS) );
}

//**********************************************************
// Function Name: LocalHmiModeAlarm
// Description:
//          This function handles the Lexium local HMI mode:
//          "HMI Mode Alarm"
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void LocalHmiModeAlarm(drive)
{
   // If NO error is pending or the user pressed button M
   if (!BGVAR(u8_Local_Hmi_Error_Pending) || ButtonMRisingEdge(drive))
   {
      // Switch the state-machine to HMI-mode monitor
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_MONITOR;
      return;
   }

   // If an access-rights hold is pending and the 1-ms Counter has a certain value (for changing the display)
   if ((BGVAR(u16_Hold_Bits) & HOLD_LXM_ACCESS_RIGHTS_MASK) && (Cntr_1mS & 0x00000400))
   {
      // Write " StoP" to the local HMI display array
      GetLocalHmiStringDisplayValue(" StoP", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
   }
   // If an fieldbus HALT is pending and the 1-ms Counter has a certain value (for changing the display)
   else if ((BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_SHOW_HALT_IN_7_SEGMENTS) && (Cntr_1mS & 0x00000400))
   {
      // Write " HALt" to the local HMI display array
      GetLocalHmiStringDisplayValue(" HALt", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
   }
   // nitsan: do nothing. dont show CAN bus off warning on display.
   // as dicussed with Joachim 29/10/2014
   else if (BGVAR(u16_Lexium_Alarm_Code) != 0) // If a Lexium alarm (warning or fault) is pending
   {
      // Fill the 7-segment array with the AL/Wn error number.
      GetLocalHmiHexDisplayValue((long)BGVAR(u16_Lexium_Alarm_Code), BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);

      // If the Lexium message number is larger or equal 0x700 OR
      // If the alarm value is 0x023 (MOTOR_FOLDBACK_WRN_MASK)
      //   --> Show Wn for Warning
      if((BGVAR(u16_Lexium_Alarm_Code) >= 0x700) || (BGVAR(u16_Lexium_Alarm_Code) == 0x23))
      {
         // Display "Wn." in the digit 3 & 4
         BGVAR(u16_Local_Hmi_Display_Array)[4] = GetLocalHmiDigitCode('W');
         BGVAR(u16_Local_Hmi_Display_Array)[3] = GetLocalHmiDigitCode('n');
      }
      else // If the Lexium message number is smaller than 700 --> Show AL for Alarm
      {
         // Display "AL." in the digit 3 & 4
         BGVAR(u16_Local_Hmi_Display_Array)[4] = GetLocalHmiDigitCode('A');
         BGVAR(u16_Local_Hmi_Display_Array)[3] = GetLocalHmiDigitCode('L');
      }
      BGVAR(u16_Local_Hmi_Display_Array)[3] &= DC_POINT; // Light up the decimal-point in digit 3, which is
                                                         // specific for this display the Alarm or Warning
   }

   // If an alarm AND a hold is pending at the same time
   //if((BGVAR(u16_Hold_Lexium_Access_Rights) != 0) && (BGVAR(u16_Lexium_Alarm_Code) != 0))
   if(((BGVAR(u16_Hold_Bits) & HOLD_LXM_ACCESS_RIGHTS_MASK) || (BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_SHOW_HALT_IN_7_SEGMENTS)) && (BGVAR(u16_Lexium_Alarm_Code) != 0))
   {
      BGVAR(u8_Local_Hmi_Blink_Display) = 0xFF; // Let all five 7-segements blink
   }
   else
   {
      // Do not let the display blink
      BGVAR(u8_Local_Hmi_Blink_Display) = 0;
   }

   // Display LSW of the array (digits 0...4)
   BGVAR(u8_Local_Hmi_Display_LSW) = 1;
}


//**********************************************************
// Function Name: SwitchToLocalHmiModeAlarm
// Description:
//          This function can theoretically do some preparation
//          work before switching to HMI Mode Alarm if needed.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void SwitchToLocalHmiModeAlarm(drive)
{
   long s32_temp;
   if((BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_ALL_KEEP_ACCESS_RIGHTS) == 0)
   {
      // Release access-rights if given to the local HMI
      s32_temp = 0;
      ExecutePParam(drive, 1032, OP_TYPE_WRITE, &s32_temp, LEX_CH_LOCAL_HMI);
   }

   // If an error is pending
   if (BGVAR(u8_Local_Hmi_Error_Pending))
   {
      // Switch the state-machine to HMI-mode alarm
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_ALARM;
   }
   else
   {
      // Switch the state-machine to HMI-mode monitor
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_MONITOR;
   }
}


//**********************************************************
// Function Name: LocalHmiModeMonitor
// Description:
//          This function handles the Lexium local HMI mode:
//          "HMI Mode Monitor"
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void LocalHmiModeMonitor(drive)
{
   long s32_Value_To_Be_Displayed = 0; // This value is supposed to be monitored

   unsigned int u16_Update_Actual_Value = 0; // Values on the 7-segment have to be updated if this variable gets true

   // Check if a switch to HMI Mode Parameter was detected
   if(ButtonMRisingEdge(drive))
   {
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_PARAMETER;
   }


   // Check if the user wants to switch from displaying the lower or upper 5 digits
   if(ButtonSRisingEdge(drive))
   {
      if (BGVAR(u8_Local_Hmi_Display_LSW) == 0)
      {
         BGVAR(u8_Local_Hmi_Display_LSW) = 1;
      }
      else
      {
         BGVAR(u8_Local_Hmi_Display_LSW) = 0;
      }
      u16_Update_Actual_Value = 1; // Force a direct update of the value
   }

   // Check if the user wants to see the values in decimal or hexadecimal format
   if(ButtonEntRisingEdge(drive))
   {
      if (BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation) == 0)
      {
         BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation) = 1;
      }
      else
      {
         BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation) = 0;
      }
   }

   // Check if user wants to increment P0-02
   if (ButtonUpRisingEdge(drive) || ButtonUpPressed500ms(drive))
   {
      BGVAR(s16_Hmi_Mode_Dspl_Stat)++;
      // Handle roll-over
      if (BGVAR(s16_Hmi_Mode_Dspl_Stat) > 26)
      {
         BGVAR(s16_Hmi_Mode_Dspl_Stat) = 0;
      }
      // When switching to a new monitor value, reset the representation to decimal
      // like in the Lexium 23.
      BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation) = 1;

      BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed) = 1;     // Indicate that the display has just been changed
      BGVAR(s32_Hmi_Mode_Monitor_Dspl_Changed_Timer) = Cntr_1mS; // Capture the current time
   }
   // Check if user wants to increment P0-02
   if (ButtonDownRisingEdge(drive) || ButtonDownPressed500ms(drive))
   {
      BGVAR(s16_Hmi_Mode_Dspl_Stat)--;
      // Handle roll-over
      if (BGVAR(s16_Hmi_Mode_Dspl_Stat) < 0)
      {
         BGVAR(s16_Hmi_Mode_Dspl_Stat) = 26;
      }
      // When switching to a new monitor value, reset the representation to decimal
      // like in the Lexium 23.
      BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation) = 1;

      BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed) = 1;     // Indicate that the display has just been changed
      BGVAR(s32_Hmi_Mode_Monitor_Dspl_Changed_Timer) = Cntr_1mS; // Capture the current time
   }


   // Clear "s16_Hmi_Mode_Monitor_Dspl_Just_Changed" bit after 2[s]
   if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed) != 0)
   {
      if (PassedTimeMS(2000L, BGVAR(s32_Hmi_Mode_Monitor_Dspl_Changed_Timer)))
      {
         BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed) = 0;
      }
   }

   // Update the actual values every 500[ms] or if a previous button S trigger has set "u16_Update_Actual_Value" to 1
   if ((PassedTimeMS(500L, BGVAR(s32_Hmi_Mode_Monitor_Update_Timer))) || (u16_Update_Actual_Value == 1))
   {
      BGVAR(s32_Hmi_Mode_Monitor_Update_Timer) = Cntr_1mS; // Update timer variable
      // Get value to be displayed
      s32_Value_To_Be_Displayed = GetDriveStatusValue(drive, BGVAR(s16_Hmi_Mode_Dspl_Stat));
      // Raise a flag for this function to update the display array
      u16_Update_Actual_Value = 1;
   }

   // Here update the array, which is forwarded to the 7-segments
   switch (BGVAR(s16_Hmi_Mode_Dspl_Stat))
   {
      case (0): // Motor feedback pulse number in user unit PUU
      case (1): // Input pulse number of pulse command in user unit PUU
      case (2): // Positition error counts in user unit PUU
      case (3): // Motor feedack pulse number (encoder unit, 1280000 pulse per rev)
      case (4): // Input pulse number of pulse command (encoder unit, 1280000 pulse per rev)
      case (5): // Position error counts (encoder unit, 1280000 pulse per rev)
      case (6): // Input frequency of the pulse commands in [kpps].
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            if(BGVAR(s16_Hmi_Mode_Dspl_Stat) == 0)
            {
               // Write "Fb.PUU" to the local HMI display array
               GetLocalHmiStringDisplayValue("FbPUU", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
               // Light up the decimal-point in digit 3
               BGVAR(u16_Local_Hmi_Display_Array)[3] &= DC_POINT;
            }
            else if(BGVAR(s16_Hmi_Mode_Dspl_Stat) == 1)
            {
               // Write "C-PUU" to the local HMI display array
               GetLocalHmiStringDisplayValue("C-PUU", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
            else if(BGVAR(s16_Hmi_Mode_Dspl_Stat) == 2)
            {
               // Write "Er.PUU" to the local HMI display array
               GetLocalHmiStringDisplayValue("ErPUU", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
               // Light up the decimal-point in digit 3
               BGVAR(u16_Local_Hmi_Display_Array)[3] &= DC_POINT;
            }
            else if(BGVAR(s16_Hmi_Mode_Dspl_Stat) == 3)
            {
               // Write "Fb.PLS" to the local HMI display array
               GetLocalHmiStringDisplayValue("FbPLS", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
               // Light up the decimal-point in digit 3
               BGVAR(u16_Local_Hmi_Display_Array)[3] &= DC_POINT;
            }
            else if(BGVAR(s16_Hmi_Mode_Dspl_Stat) == 4)
            {
               // Write "C-PLS" to the local HMI display array
               GetLocalHmiStringDisplayValue("C-PLS", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
            else if(BGVAR(s16_Hmi_Mode_Dspl_Stat) == 5)
            {
               // Write "Er.PLS" to the local HMI display array
               GetLocalHmiStringDisplayValue("ErPLS", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
               // Light up the decimal-point in digit 3
               BGVAR(u16_Local_Hmi_Display_Array)[3] &= DC_POINT;
            }
            else if(BGVAR(s16_Hmi_Mode_Dspl_Stat) == 6)
            {
               // Write "CP-Fr" to the local HMI display array
               GetLocalHmiStringDisplayValue("CP-Fr", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }

            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

      case (7): // Motor rotation speed [rpm] with 1 decimal place (600[rpm] = 6001 on the 7-segment)
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "SPEEd" to the local HMI display array
            GetLocalHmiStringDisplayValue("SPEEd", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            // Light up the decimal-point in digit 1, which is specific for this display mode since it represents xxx.y[rpm]
            BGVAR(u16_Local_Hmi_Display_Array)[1] &= DC_POINT;
            // Show ONLY lower 5 digits and let NOT the user decide. This is specific for this mode.
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
      break;

      case (8): // Speed input command in Volt --> Analog input value 1 in [V] in x.yy format
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "CSPD1" to the local HMI display array
            GetLocalHmiStringDisplayValue("CSPD1", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            // Light up the decimal-point in digit 2, which is specific for this display mode since it represents x.yy[V]
            BGVAR(u16_Local_Hmi_Display_Array)[2] &= DC_POINT;
            // Show ONLY lower 5 digits and let NOT the user decide. This is specific for this mode.
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
      break;

      case (9): // Speed (input) command in [rpm]
      {
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "CSPD2" to the local HMI display array
            GetLocalHmiStringDisplayValue("CSPD2", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      }
      break;

      case (10): // Torque input command in Volt --> Analog input value 2 in [V] in x.yy format
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "C-tq1" to the local HMI display array
            GetLocalHmiStringDisplayValue("C-tq1", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            // Light up the decimal-point in digit 2, which is specific for this display mode since it represents x.yy[V]
            BGVAR(u16_Local_Hmi_Display_Array)[2] &= DC_POINT;
            // Show ONLY lower 5 digits and let NOT the user decide. This is specific for this mode.
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
      break;

      case (11): // Torque input command in [%]
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "C-tq2" to the local HMI display array
            GetLocalHmiStringDisplayValue("C-tq2", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;
      case (12): // Motor Load Percent
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "AuG-L" to the local HMI display array
            GetLocalHmiStringDisplayValue("AuG-L", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;
      case (13): // Motor Load Percent Max
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "PE-L" to the local HMI display array
            GetLocalHmiStringDisplayValue("PE-L ", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

      case (14): // Main circuit voltage in [V]
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "U BUS" to the local HMI display array
            GetLocalHmiStringDisplayValue("U BUS", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
                GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
                GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

      case (15):
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "J-L  " to the local HMI display array
            GetLocalHmiStringDisplayValue("J-L  ", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            // Light up the decimal-point in digit 1, which is specific for this display mode since it represents x.y
            BGVAR(u16_Local_Hmi_Display_Array)[1] &= DC_POINT;
            // Show ONLY lower 5 digits and let NOT the user decide. This is specific for this mode.
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
      break;

      case (16): // IGBT temperature
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "IGbtt" to the local HMI display array
            GetLocalHmiStringDisplayValue("IGbtt", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Light up the decimal-point in digit 1, which is specific for the string to be displayed
            BGVAR(u16_Local_Hmi_Display_Array)[1] &= DC_POINT;
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

    case (17): //resonance frequencies from nlantivibhz2 and nlantivibhz3
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "rSn.Fr" to the local HMI display array
            GetLocalHmiStringDisplayValue("rSnFr", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
          // Light up the decimal-point in digit 2, which is specific for this display mode since it represents x.yy[V]
            BGVAR(u16_Local_Hmi_Display_Array)[2] &= DC_POINT;

            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {

            // Fill the 7-segment array
            GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);

         }
      break;
    case (18): //Absolute Pulse Number : MECHANGLE
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "dIFF.2" to the local HMI display array
            GetLocalHmiStringDisplayValue("dIFF2", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
          // Light up the decimal-point in digit 1, which is specific for this display mode
            BGVAR(u16_Local_Hmi_Display_Array)[1] &= DC_POINT;

            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {

            // Fill the 7-segment array
            GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);

         }
      break;





      case (19): // Content of P0-25
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "NNAP1" to the local HMI display array
            GetLocalHmiStringDisplayValue("NNAP1", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

      case (20): // Content of P0-26
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "NNAP2" to the local HMI display array
            GetLocalHmiStringDisplayValue("NNAP2", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

      case (21): // Content of P0-27
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "NNAP3" to the local HMI display array
            GetLocalHmiStringDisplayValue("NNAP3", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

      case (22): // Content of P0-28
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "NNAP4" to the local HMI display array
            GetLocalHmiStringDisplayValue("NNAP4", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

      case (23): // Content of P0-09
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "UAr-1" to the local HMI display array
            GetLocalHmiStringDisplayValue("UAr-1", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

      case (24): // Content of P0-10
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "UAr-2" to the local HMI display array
            GetLocalHmiStringDisplayValue("UAr-2", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

      case (25): // Content of P0-11
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "UAr-3" to the local HMI display array
            GetLocalHmiStringDisplayValue("UAr-3", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

      case (26): // Content of P0-12
         if (BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed))
         {
            // Write "UAr-4" to the local HMI display array
            GetLocalHmiStringDisplayValue("UAr-4", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show lower 5 digits
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
         else if (u16_Update_Actual_Value)
         {
            // Fill the 7-segment array
            if(BGVAR(u8_Hmi_Mode_Monitor_Dec_Representation))
            {
               GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            }
            else
            {
               GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
      break;

      /*******************************************************************************************************************/
      /*** From here starts the extension variables, which can only be selected via P0-02 but not via the UP/DOWN keys ***/
      /*******************************************************************************************************************/

      case (39): // Content of P4-07
      case (40): // Content of P4-09
      case (41): // Content of P0-46
      case (42): // Currently applied mode of operation (which is active after save & re-boot).
         if (u16_Update_Actual_Value)
         {
            // Show only in hexadecimal format (specific for this mode)
            GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            // Show only lower 5 digits, specific for this mode
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
      break;

      case (49): // Pulse count input - Raw-pulses in Pt mode
         if (u16_Update_Actual_Value)
         {
            // Show only in decimal format
            GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
         }
      break;

      case (50): // Speed (input) command in [0.1*rpm]
      case (53): // Command torque in the unit 0.1% with MICONT = 100%
      case (54): // Feedback torque in the unit 0.1% with MICONT = 100%
      case (77): // ptpvcmd Speed (input) command in Position mode,[0.1*rpm]
         if (u16_Update_Actual_Value)
         {
            // Show only in decimal format (specific for this mode)
            GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            // Light up the decimal-point in digit 1, which is specific for this mode since the unit is 0.1%
            BGVAR(u16_Local_Hmi_Display_Array)[1] &= DC_POINT;
            // Show only lower 5 digits, specific for this mode
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
      break;

      case (55): // Feedback torque in 0.01[A]
         if (u16_Update_Actual_Value)
         {
            // Show only in decimal format (specific for this mode)
            GetLocalHmiDecimalDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);
            // Light up the decimal-point in digit 2, which is specific for this mode since the unit is 0.01A
            BGVAR(u16_Local_Hmi_Display_Array)[2] &= DC_POINT;
            // Show only lower 5 digits, specific for this mode
            BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         }
      break;

      case (96):  // Content of P0-00 and P5-00
      case (111): // Get the "Lexium Drive Alarm Code"
         if (u16_Update_Actual_Value)
         {
            // Show only in hexadecimal format (specific for this mode)
            GetLocalHmiHexDisplayValue(s32_Value_To_Be_Displayed, BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
         }
      break;

      default: // Unsupported mode, switch off the 7-segments
         // Show " n A " for not available
         GetLocalHmiStringDisplayValue(" n A ", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
         // Show lower 5 digits
         BGVAR(u8_Local_Hmi_Display_LSW) = 1;
      break;
   }

   // Do not let the display blink
   BGVAR(u8_Local_Hmi_Blink_Display) = 0;
}

//**********************************************************
// Function Name: SwitchToLocalHmiModeMonitor
// Description:
//          This function can theoretically do some preparation
//          work before switching to HMI Mode Monitor if needed.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void SwitchToLocalHmiModeMonitor(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Show lower 5 digits when switching to HMI mode monitor
   BGVAR(u8_Local_Hmi_Display_LSW) = 1;

   // If an error is pending and you switch to HMI mode monitor
   if(BGVAR(u8_Local_Hmi_Error_Pending))
   {
      // Indicate for 2[s] what is displayed on the 7-segments (see Lexium 23 behaviour)
      BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed) = 1;            // Act as P0-02 has just been changed
      BGVAR(s32_Hmi_Mode_Monitor_Dspl_Changed_Timer) = Cntr_1mS; // Capture the current time
   }

   // Switch the state-machine to HMI-mode parameter
   BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_MONITOR;
}


//**********************************************************
// Function Name: LocalHmiModeParameter
// Description:
//          This function handles the Lexium local HMI mode:
//          "HMI Mode Parameter"
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void LocalHmiModeParameter(drive)
{
   unsigned int u16_Param_Group   = BGVAR(u16_P_Param_Selection) / 100;
   unsigned int u16_Param_Id_Code = BGVAR(u16_P_Param_Selection) % 100;

   unsigned char u8_Key_Up = 1; // Distinguish between button UP or DOWN for roll-over

   // If there is something urgent that needs to be done by this first
   if(BGVAR(u8_Hmi_Mode_Parameter_Perform_Action) != 0)
   {
      if (BGVAR(u8_Hmi_Mode_Parameter_Perform_Action) == 1)
      {
         // Write the word " n A " to the 7-segment
         GetLocalHmiStringDisplayValue(" n A ", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
      }
      else if (BGVAR(u8_Hmi_Mode_Parameter_Perform_Action) == 2)
      {
         // Write the word "Prot " to the 7-segment (Protected = no access-rights message according to A. Badura)
         GetLocalHmiStringDisplayValue("Prot ", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
      }
      else
      {
         // Write the word "Error" to the 7-segment (unknown error message).
         GetLocalHmiStringDisplayValue("Error", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
      }

      BGVAR(u8_Local_Hmi_Blink_Display) = 0xFF; // Let all five 7-segements blink

      BGVAR(u8_Local_Hmi_Display_LSW) = 1; // Display LSW of the array (digits 0...4)

      if(!PassedTimeMS(2000L, BGVAR(s32_Hmi_Mode_Parameter_Perform_Action_Timer)))
      {
         return; // Return from function until time has been elapsed
      }
   }

   // Check if a switch to HMI Mode Monitor or HMI Mode Alarm was detected
   if(ButtonMRisingEdge(drive))
   {
      // Switch to HMI Mode Alarm or Monitor. The HMI Mode Monitor is automatically displayed if no Error is pending.
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_ALARM;
   }

   // Check if a switch to HMI Mode Edit Parameter was detected
   if(ButtonEntRisingEdge(drive))
   {
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_EDIT_PARAMETER;
   }

   // Check if the user wanted to change the parameter group
   if(ButtonSRisingEdge(drive))
   {
      u16_Param_Group += 1; // Jump to next group
   }

   // Check if the user wanted to change the parameter within the group via button UP
   if(ButtonUpRisingEdge(drive) || ButtonUpPressed100ms(drive))
   {
      u16_Param_Id_Code += 1;
      u8_Key_Up = 1;
   }

   // Check if the user wanted to change the parameter id-code within the group via button DOWN
   if(ButtonDownRisingEdge(drive) || ButtonDownPressed100ms(drive))
   {
      u16_Param_Id_Code -= 1;
      u8_Key_Up = 0;
   }

   // Roll over from P9-xy to P0-xy
   if (u16_Param_Group > 9)
      u16_Param_Group = 0;

   // If there is a roll-over in the ID-Code
   if (u16_Param_Id_Code > u16_Max_Param_Id_Code[u16_Param_Group])
   {
      if (u8_Key_Up == 0) // If key-down pressed
      {
         u16_Param_Id_Code = u16_Max_Param_Id_Code[u16_Param_Group];
      }
      else // key-up pressed
      {
         u16_Param_Id_Code = 0;
      }
   }

   // Now re-construct the P-parameter
   BGVAR(u16_P_Param_Selection) = (100 * u16_Param_Group) + u16_Param_Id_Code;

   // Fill the array that holds the 7-segment data
   GetLocalHmiDecimalDisplayValue((long)BGVAR(u16_P_Param_Selection), (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 0);

   // Display the P-Parameter in the following format: Px-yz
   BGVAR(u16_Local_Hmi_Display_Array)[3] = BGVAR(u16_Local_Hmi_Display_Array)[2];
   BGVAR(u16_Local_Hmi_Display_Array)[2] = GetLocalHmiDigitCode('-');
   BGVAR(u16_Local_Hmi_Display_Array)[4] = GetLocalHmiDigitCode('P');

   // Do not let the display blink
   BGVAR(u8_Local_Hmi_Blink_Display) = 0;

   // Display LSW of the array (digits 0...4)
   BGVAR(u8_Local_Hmi_Display_LSW) = 1;
}

//**********************************************************
// Function Name: SwitchToLocalHmiModeParameter
// Description:
//          This function can theoretically do some preparation
//          work before switching to HMI Mode Parameter if needed.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void SwitchToLocalHmiModeParameter(drive)
{
   long s32_temp;
   // Show lower 5 digits when switching to HMI mode parameter
   BGVAR(u8_Local_Hmi_Display_LSW) = 1;

   if((BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_ALL_KEEP_ACCESS_RIGHTS) == 0)
   {
      // Release access-rights if given to the local HMI
      s32_temp = 0;
      ExecutePParam(drive, 1032, OP_TYPE_WRITE, &s32_temp, LEX_CH_LOCAL_HMI);
   }

   // Switch the state-machine to HMI-mode parameter
   BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_PARAMETER;
}

//**********************************************************
// Function Name: LocalHmiModeEditParameter
// Description:
//          This function handles the Lexium local HMI mode:
//          "HMI Mode Edit Parameter"
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void LocalHmiModeEditParameter(drive)
{
   int s16_temp; // General purpose variable
   unsigned long u32_Offset_Value; // Value to be added / subtracted
   long long s64_Value = (signed long long)BGVAR(s32_Hmi_Mode_Edit_Param_Value);
   // Here correct the value in case that we are entering a hexadecimal 32-bit value or a combined decimal value.
   //  These values can never be negative, therefore add 2^32 to a negative value in order to correct the value.
   if(((BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 0) || (BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 2)) && ((BGVAR(u8_Hmi_Mode_Edit_Parameter_Property) & HMI_MODE_EDIT_PARAM_16_BIT_PARAM) == 0) && (s64_Value < 0LL))
   {
      // Here the value is hexadecimal 32-bit but negative, which is not intended.
      s64_Value += 0x100000000LL; // add 2^32 to the value
   }

   // If something needs to be done (like saving params or displaying a message).
   if(BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) != 0)
   {
      if(BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) == 1) // If the user pressed previously the ENT button
      {
         // If the save is still in progress -> Do nothing
         if((BGVAR(u16_Lexium_Bg_Action_Applied) & LEX_DO_NV_SAVE) == LEX_DO_NV_SAVE)
         {
            BGVAR(u8_Local_Hmi_Blink_Display) = 0; // Let no 7-segements blink
            BGVAR(u8_Local_Hmi_Display_LSW) = 1; // Display LSW of the array (digits 0...4)

            return; // Return from function
         }
         else // Show Saved on the 7-segment
         {
            BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) = 254;
            // Save the timer value for displaying the message for a few seconds
            BGVAR(s32_Hmi_Mode_Edit_Param_Perform_Action_Timer) = Cntr_1mS;

            return; // Return from function
         }
      }
      else // Just display a message
      {
         if (BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) == 2)
         {
            // Write the word "Out-r" to the 7-segment
            GetLocalHmiStringDisplayValue("Out-r", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
         }
         else if (BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) == 3)
         {
            // Write the word "SruOn" to the 7-segment
            GetLocalHmiStringDisplayValue("SruOn", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
         }
         else if (BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) == 4)
         {
            // Write the word "r-OLY" to the 7-segment
            GetLocalHmiStringDisplayValue("r-OLY", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
         }
         else if (BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) == 5)
         {
            // Write the word "Prot " to the 7-segment (Protected = no access-rights message according to A. Badura)
            GetLocalHmiStringDisplayValue("Prot ", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
         }
         else if (BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) == 254)
         {
            if((BGVAR(u16_P_Param_Selection) == 101) ||                                                     // For P1-01, which is CTL (mode of operation)
               ((BGVAR(u16_P_Param_Selection) == 208) && (BGVAR(s32_Hmi_Mode_Edit_Param_Value) == 10)) ||   // If someone issued a factory-restore upon the next power up
               (BGVAR(u16_P_Param_Selection) == 268) ||                                                     // For P2-68, which is auto-enable or limit-swtich handling
               (BGVAR(u16_P_Param_Selection) == 300) ||                                                     // For P3-00, which is the Modbus communication address
               (BGVAR(u16_P_Param_Selection) == 305) ||                                                     // For P3-05, which is the CANOpen Communication address
               (BGVAR(u16_P_Param_Selection) == 301) ||                                                     // Same for P3-01 and P3-02 (Serial comuication baudrate and protocol)
               (BGVAR(u16_P_Param_Selection) == 302))                                                       // Which are active only after power cycle.
            {
               // Write the word "Po-0n" to the 7-segment in order to display that this parameter is valid after re-boot.
               GetLocalHmiStringDisplayValue("Po-0n", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
            else if ( (BGVAR(u16_P_Param_Selection) == 405) ||       // For P4-05, which is the jog move speed (where we also jump to local HMI mode jog-move)
                      (BGVAR(u16_P_Param_Selection) == 232) ||       // For P2-32, which is the auto-tune mode (where we also jump to local HMI mode auto-tune)
                      (BGVAR(u16_P_Param_Selection) == 2)   ||       // For P0-02 do not show "SAvEd"
                      (BGVAR(u8_Hmi_Mode_Edit_Parameter_Property) & HMI_MODE_EDIT_PARAM_NO_NV_PARAMETER) )  // For read/write P-Parameter, which are not stored on the NV memory
            {
               // Reduce the action timer by 2100 in order to directly jump to local HMI mode jog-move or auto-tune and skip showing "SAvEd" on the display.
               // This request comes from Alexander Badura.
               BGVAR(s32_Hmi_Mode_Edit_Param_Perform_Action_Timer) = BGVAR(s32_Hmi_Mode_Edit_Param_Perform_Action_Timer) - 2100;
            }
            else // For all other P-parameter
            {
               // Write the word "SAvEd" to the 7-segment
               GetLocalHmiStringDisplayValue("SAvEd", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
            }
         }
         else // 255
         {
            // Write the word "Error" to the 7-segment (unknown error message).
            GetLocalHmiStringDisplayValue("Error", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
         }

         BGVAR(u8_Local_Hmi_Blink_Display) = 0xFF; // Let all five 7-segements blink

         BGVAR(u8_Local_Hmi_Display_LSW) = 1; // Display LSW of the array (digits 0...4)

         if(!PassedTimeMS(2000L, BGVAR(s32_Hmi_Mode_Edit_Param_Perform_Action_Timer)))
         {
            return; // Return from function until time has been elapsed
         }
      }

      // If writing to P4-05 was successfull, especially in terms of access-rights handling
      if ((BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) == 254) &&
          (BGVAR(u16_P_Param_Selection) == 405))
      {
         // Switch to JOG move handling.
         BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_JOG_MOVE;
      }
      // If writing to P2-32 was successfull, especially in terms of access-rights handling
      else if ((BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) == 254) &&
               (BGVAR(u16_P_Param_Selection) == 232))
      {
         // Switch to auto-tuner handling.
         BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_AUTOTUNE;
      }
      // If writing to P0-02 was successfull
      else if ((BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) == 254) &&
               (BGVAR(u16_P_Param_Selection) == 2))
      {
         // Indicate for 2[s] what is displayed on the 7-segments (see Lexium 23 behaviour)
         BGVAR(s16_Hmi_Mode_Monitor_Dspl_Just_Changed) = 1;            // Act as P0-02 has just been changed
         BGVAR(s32_Hmi_Mode_Monitor_Dspl_Changed_Timer) = Cntr_1mS;    // Capture the current time
         // Switch to auto-tuner handling.
         BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_MONITOR;
      }
      else
      {
         // Switch to HMI Mode Parameter.
         BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_PARAMETER;
      }

      // Reset flag that controls the action
      BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) = 0;

      return; // Return from function
   }

   // If an ExecutePParam call is supposed to be triggered
   if(BGVAR(u8_Hmi_Mode_Edit_Param_Execute_P_Param) != 0)
   {
      // Perform 32-bit write access to the selected P-parameter
      s16_temp = ExecutePParam(drive, BGVAR(u16_P_Param_Selection), OP_TYPE_WRITE, &BGVAR(s32_Hmi_Mode_Edit_Param_Value), LEX_CH_LOCAL_HMI);

      // If the function is not yet finish -> Continue next BG cycle
      if(s16_temp == SAL_NOT_FINISHED)
      {
         return; // Leave function
      }
      else
      {
         // Clear the ExecuteP-param request
         BGVAR(u8_Hmi_Mode_Edit_Param_Execute_P_Param) = 0;
      }

      // If there was a successful write to any parameter
      if (s16_temp == SAL_SUCCESS)
      {
         // If Parameter P2-30 has been accessed
         if(BGVAR(u16_P_Param_Selection) == 230)
         {
            if(BGVAR(s32_Hmi_Mode_Edit_Param_Value) == 1) // Upon enable via local HMI
            {
               // Keep lHMI access-rights (Request for IPR747: When lHMI enables the Drive, then keep AR to lHMI until disable via lHMI)
               BGVAR(u16_Local_Hmi_General_Purpose) |= HMI_MODE_ALL_KEEP_ACCESS_RIGHTS;
            }
            else if (BGVAR(s32_Hmi_Mode_Edit_Param_Value) == 0) // Upon disable via local HMI
            {
               // Allow releasing lHMI access-rights (Request for IPR747: When lHMI enables the Drive, then keep AR to lHMI until disable via lHMI)
               BGVAR(u16_Local_Hmi_General_Purpose) &= ~HMI_MODE_ALL_KEEP_ACCESS_RIGHTS;
            }
         }

         if((BGVAR(u16_P_Param_Selection) == 232) ||     // If an auto-tuning is supposed to be started via local HMI
            (BGVAR(u8_Hmi_Mode_Edit_Parameter_Property) & HMI_MODE_EDIT_PARAM_NO_NV_PARAMETER)) // or if the parameter is not stored on the NV memory
         {
            // Omit the NV save procedure
            BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) = 254;
         }
         else
         {
            // Indicate that a SAVE has to be performed in case that the write command has been succeeded.
            BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) = 1;
            BGVAR(u16_Lexium_Bg_Action) |= LEX_DO_NV_SAVE;
         }
      }
      else // Changing P-parameter has failed, show the user why
      {
         if ((s16_temp == VALUE_OUT_OF_RANGE) || (s16_temp == VALUE_TOO_LOW) || (s16_temp == VALUE_TOO_HIGH))
         {
            // Indicate that the "Out-r" message has to be displayed.
            BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) = 2;
         }
         else if(s16_temp == DRIVE_ACTIVE)
         {
            // Indicate that the "SruOn" (Servo running/on) message has to be displayed.
            BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) = 3;
         }
         else if(s16_temp == NOT_PROGRAMMABLE)
         {
            // Indicate that the "r-OLY" (Servo running/on) message has to be displayed.
            BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) = 4;
         }
         else if(s16_temp == LEXIUM_INVALID_ACCESS_RIGHTS)
         {
            // Indicate that the "Prot " (Protected = no Access rights) message has to be displayed.
            BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) = 5;
         }
         else
         {
            // Indicate that the "Error" message has to be displayed (unknown error response / should usually not happen).
            BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) = 255;
         }

         // Save the timer value for displaying the message for a few seconds
         BGVAR(s32_Hmi_Mode_Edit_Param_Perform_Action_Timer) = Cntr_1mS;
      }
   }

   // Determine the offset to add/subtract upon button UP/DOWN, determine which 7-segment
   // digit to blink and determine if lower 5-digits or upper 5-digits to display.
   switch (BGVAR(u8_Hmi_Mode_Edit_Param_Digit_Number))
   {
      case (0): // Change digit 0
         u32_Offset_Value = 0x00000001;
         BGVAR(u8_Local_Hmi_Display_LSW) = 1; // Display LSW of the array (digits 0...4)
         BGVAR(u8_Local_Hmi_Blink_Display) = 1; // Blink 7-segment 0
      break;

      case (1): // Change digit 1
         if(BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) > 0)
         {
            u32_Offset_Value = 10L;
         }
         else
         {
            u32_Offset_Value = 0x00000010;
         }
         BGVAR(u8_Local_Hmi_Display_LSW) = 1; // Display LSW of the array (digits 0...4)
         BGVAR(u8_Local_Hmi_Blink_Display) = 2; // Blink 7-segment 1
      break;

      case (2): // Change digit 2
         if(BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) > 0)
         {
            u32_Offset_Value = 100L;
         }
         else
         {
            u32_Offset_Value = 0x00000100;
         }
         BGVAR(u8_Local_Hmi_Display_LSW) = 1; // Display LSW of the array (digits 0...4)
         BGVAR(u8_Local_Hmi_Blink_Display) = 3; // Blink 7-segment 2
      break;

      case (3): // Change digit 3
         if(BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) > 0)
         {
            u32_Offset_Value = 1000L;
         }
         else
         {
            u32_Offset_Value = 0x00001000;
         }
         BGVAR(u8_Local_Hmi_Display_LSW) = 1; // Display LSW of the array (digits 0...4)
         BGVAR(u8_Local_Hmi_Blink_Display) = 4; // Blink 7-segment 3
      break;

      case (4): // Change digit 4
         if(BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) > 0)
         {
            u32_Offset_Value = 10000L;
         }
         else
         {
            u32_Offset_Value = 0x00000000; // No offset since digit 4 holds the letter 'L'
         }
         BGVAR(u8_Local_Hmi_Display_LSW) = 1; // Display LSW of the array (digits 0...4)
         BGVAR(u8_Local_Hmi_Blink_Display) = 5; // Blink 7-segment 4
      break;

      case (5): // Change digit 5
         if(BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 1)
         {
            u32_Offset_Value = 100000L;
         }
         else if (BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 2)
         {
            u32_Offset_Value = 65536L * 1L;
         }
         else
         {
            u32_Offset_Value = 0x00010000;
         }
         BGVAR(u8_Local_Hmi_Display_LSW) = 0; // Display MSW of the array (digits 5...9)
         BGVAR(u8_Local_Hmi_Blink_Display) = 1; // Blink 7-segment 0
      break;

      case (6): // Change digit 6
         if(BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 1)
         {
            u32_Offset_Value = 1000000L;
         }
         else if (BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 2)
         {
            u32_Offset_Value = 65536L * 10L;
         }
         else
         {
            u32_Offset_Value = 0x00100000;
         }
         BGVAR(u8_Local_Hmi_Display_LSW) = 0; // Display MSW of the array (digits 5...9)
         BGVAR(u8_Local_Hmi_Blink_Display) = 2; // Blink 7-segment 1
      break;

      case (7): // Change digit 7
         if(BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 1)
         {
            u32_Offset_Value = 10000000L;
         }
         else if (BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 2)
         {
            u32_Offset_Value = 65536L * 100L;
         }
         else
         {
            u32_Offset_Value = 0x01000000;
         }
         BGVAR(u8_Local_Hmi_Display_LSW) = 0; // Display MSW of the array (digits 5...9)
         BGVAR(u8_Local_Hmi_Blink_Display) = 3; // Blink 7-segment 2
      break;

      case (8): // Change digit 8
         if(BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 1)
         {
            u32_Offset_Value = 100000000L;
         }
         else if (BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 2)
         {
            u32_Offset_Value = 65536L * 1000L;
         }
         else
         {
            u32_Offset_Value = 0x10000000;
         }
         BGVAR(u8_Local_Hmi_Display_LSW) = 0; // Display MSW of the array (digits 5...9)
         BGVAR(u8_Local_Hmi_Blink_Display) = 4; // Blink 7-segment 3
      break;

      default:  // Change digit 9
         if(BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 1)
         {
            u32_Offset_Value = 1000000000L;
         }
         else if (BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 2)
         {
            u32_Offset_Value = 65536L * 10000L;
         }
         else
         {
            u32_Offset_Value = 0x00000000; // No offset since digit 9 holds the letter 'h'
         }
         BGVAR(u8_Local_Hmi_Display_LSW) = 0; // Display MSW of the array (digits 5...9)
         BGVAR(u8_Local_Hmi_Blink_Display) = 5; // Blink 7-segment 4
      break;
   }

   // If the parameter is of tye read-only
   if(BGVAR(u8_Hmi_Mode_Edit_Param_Read_Only))
   {
      BGVAR(u8_Local_Hmi_Blink_Display) = 0; // Do not let the display blink (undo setting from code before)
   }

   // Check if user switched to next digit
   if (ButtonSRisingEdge(drive))
   {
      // Jump to next digit or jump between high/low word if read-only (no blinking of digits)
      if(BGVAR(u8_Hmi_Mode_Edit_Param_Read_Only))
      {
         BGVAR(u8_Hmi_Mode_Edit_Param_Digit_Number)+=5;
      }
      else
      {
         BGVAR(u8_Hmi_Mode_Edit_Param_Digit_Number)++;
      }

      if( (BGVAR(u8_Hmi_Mode_Edit_Param_Digit_Number) > 9) ||
          ((BGVAR(u8_Hmi_Mode_Edit_Param_Digit_Number) > 4) && (BGVAR(u8_Hmi_Mode_Edit_Parameter_Property) & HMI_MODE_EDIT_PARAM_16_BIT_PARAM))
        )
      {
         BGVAR(u8_Hmi_Mode_Edit_Param_Digit_Number) = 0;
      }
   }

   // If the parameter is NOT read-only --> Allow modification
   if(!BGVAR(u8_Hmi_Mode_Edit_Param_Read_Only))
   {
      // Check if user wants to change the parameter via button UP
      if(ButtonUpRisingEdge(drive) || ButtonUpPressed100ms(drive))
      {
         s64_Value += u32_Offset_Value;
      }

      // Check if user wants to change the parameter via button DOWN
      if(ButtonDownRisingEdge(drive) || ButtonDownPressed100ms(drive))
      {
         s64_Value -= u32_Offset_Value;
      }

      // Here invert the algebraic sign if allowed
      if(ButtonSPressed500ms(drive))
      {
          // If the parameter is represented in decimal format AND if the Drive table holds valid entries for minVal and maxVal (MinVal != MaxVal)
         if((BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 1) && (BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value) != BGVAR(s64_Hmi_Mode_Edit_Param_Min_Value)))
         {
            // If we would not exceed the valid range in case that we negate the value
            if( (-s64_Value <= BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value)) && (-s64_Value >= BGVAR(s64_Hmi_Mode_Edit_Param_Min_Value)) )
            {
               s64_Value = -s64_Value;
               // After converting reset the digit number to 0
               BGVAR(u8_Hmi_Mode_Edit_Param_Digit_Number) = 0;
            }
         }
      }

      // Check if the selected value of the P-Paramneter is allowed at all. The
      // value can be invalid just for some specific P-Parameter.
      if(IsPParamValueValid(s64_Value, BGVAR(u16_P_Param_Selection)))
      {
         // Check if Minimum or Maximum value has been exceeded in case that the Drive table holds valid entries (MinVal != MaxVal)
         if ((s64_Value > BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value)) && (BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value) != BGVAR(s64_Hmi_Mode_Edit_Param_Min_Value)))
         {
            BGVAR(s32_Hmi_Mode_Edit_Param_Value) = BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value);
         }
         else if ((s64_Value < BGVAR(s64_Hmi_Mode_Edit_Param_Min_Value)) && (BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value) != BGVAR(s64_Hmi_Mode_Edit_Param_Min_Value)))
         {
            BGVAR(s32_Hmi_Mode_Edit_Param_Value) = BGVAR(s64_Hmi_Mode_Edit_Param_Min_Value);
         }
         else
         {
            BGVAR(s32_Hmi_Mode_Edit_Param_Value) = s64_Value;
         }
      }
   }


   // Check if the user wants to switch back to HMI mode parameter
   if(ButtonMRisingEdge(drive))
   {
      // Switch to HMI Mode Parameter.
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_PARAMETER;
   }

   // Check if the user wants to save the parameter to the nv memory
   if(ButtonEntRisingEdge(drive))
   {
      // Handle the action at the beginning of this function
      BGVAR(u8_Hmi_Mode_Edit_Param_Execute_P_Param) = 1;
   }

   // Fill the array that holds the 7-segment data with the current value
   if(BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 1)
   {
      GetLocalHmiDecimalDisplayValue(BGVAR(s32_Hmi_Mode_Edit_Param_Value), (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 1);

      // If it turns out that the variable is of type 16-bits
      if(BGVAR(u8_Hmi_Mode_Edit_Parameter_Property) & HMI_MODE_EDIT_PARAM_16_BIT_PARAM)
      {
         // Do not set the decimal points in digit 0 and show only the low signifiant word
         BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         BGVAR(u16_Local_Hmi_Display_Array)[0] |= ~DC_POINT; // Switch off the decimal-point in digit 0 like in the Lexium 23
      }
   }
   else if(BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) == 2)
   {
      GetLocalHmiCombinedDecimalDisplayValue(BGVAR(s32_Hmi_Mode_Edit_Param_Value), (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
   }
   else
   {
      GetLocalHmiHexDisplayValue(BGVAR(s32_Hmi_Mode_Edit_Param_Value), BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);

      // If it turns out that the variable is of type 16-bits
      if(BGVAR(u8_Hmi_Mode_Edit_Parameter_Property) & HMI_MODE_EDIT_PARAM_16_BIT_PARAM)
      {
         // Do not show the L in the digit 4 and show only the low significant word
         BGVAR(u8_Local_Hmi_Display_LSW) = 1;
         BGVAR(u16_Local_Hmi_Display_Array)[4] = NONE; // Switch off completely the digit 4 of the LSW like in the Lexium 23.
         // Special handling for P0-00 (software version). Light up decimal point in digit 2 in order to seperate between
         // major and minor value. This code has been added due to "BYang00000421".
         if(BGVAR(u16_P_Param_Selection) == 0)
         {
            BGVAR(u16_Local_Hmi_Display_Array)[2] &= DC_POINT;
         }
      }
   }
}

//**********************************************************
// Function Name: SwitchToLocalHmiModeEditParameter
// Description:
//          This function can theoretically do some preparation
//          work before switching to HMI Mode Edit Parameter if
//          needed.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void SwitchToLocalHmiModeEditParameter(drive)
{
   unsigned int u16_cmd_index, u16_param_index,u16_p_param_factor=1;
   int s16_return_value;
   signed long s32_temp;

   // Search for the Drive-table index. index=1 --> Drive-table entry 0; index=2 --> Drive-table entry 1...
   u16_cmd_index = SearchPParamIndex(BGVAR(u16_P_Param_Selection), &u16_param_index);

   // Perform 32-bit read access to the selected P-parameter
   s16_return_value = ExecutePParam(drive, BGVAR(u16_P_Param_Selection), OP_TYPE_READ, &BGVAR(s32_Hmi_Mode_Edit_Param_Value), LEX_CH_LOCAL_HMI);
   if (s16_return_value == SAL_SUCCESS)
   {
      // Figure out if the P-parameter is read-only OR if the P-parameter is an action-command.
      // If no table-entry for the P-parameter was found OR if the parameter is an action-command or read-only
      if ((u16_cmd_index == 0) || (MAX_WRITE_PARAMS(Commands_Table[u16_cmd_index-1].u16_min_max_arg) == 0))
      {
         BGVAR(u8_Hmi_Mode_Edit_Param_Read_Only) = 1; // State that the P-parameter is read-only
      }
      else
      {
         BGVAR(u8_Hmi_Mode_Edit_Param_Read_Only) = 0; // State that the P-parameter is writable
         // Get minimum and maximum values out of the table
         BGVAR(s64_Hmi_Mode_Edit_Param_Min_Value) = Commands_Table[u16_cmd_index-1].s64_min_value;
         BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value) = Commands_Table[u16_cmd_index-1].s64_max_value;

         // Here we set the MaxVal for P2-08 hard-coded to 999 since the returned value can be 999 in case that a certain
         // action is currently running although setting the value to 999 is not allowed. This is very specific just for this P-Parameter.
         if(BGVAR(u16_P_Param_Selection) == 208)
         {
            if(BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value) < 999)
               BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value) = 999;
         }

         if(u16_Flash_Type == 1)   // 2 * 64Mb flash, LXM28E
         {
            if(BGVAR(u16_P_Param_Selection) == 427)
            {
               BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value) = 15;
            }
            if(BGVAR(u16_P_Param_Selection) == 428)
            {
               BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value) = 15;
            }
         }
         else
         {
            if(BGVAR(u16_P_Param_Selection) == 427)
            {
               BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value) = 31;
            }
            if(BGVAR(u16_P_Param_Selection) == 428)
            {
               BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value) = 31;
            }
         }

         // Check if the read-write P-Parameter is stored on the NV memory or not
         if(Commands_Table[u16_cmd_index-1].u16_P_Param_Properties & 0x0004)
         {
            BGVAR(u8_Hmi_Mode_Edit_Parameter_Property) &= ~HMI_MODE_EDIT_PARAM_NO_NV_PARAMETER; // Indicate that the parameter is stored in the NV memory
         }
         else
         {
            BGVAR(u8_Hmi_Mode_Edit_Parameter_Property) |= HMI_MODE_EDIT_PARAM_NO_NV_PARAMETER;  // Indicate that the parameter is NOT stored in the NV memory
         }

         u16_p_param_factor = getPParamFactor(u16_cmd_index-1);
         // Divide by factor (because decimal parameter is already multiplied by 1000)
         BGVAR(s64_Hmi_Mode_Edit_Param_Min_Value) /= u16_p_param_factor;
         BGVAR(s64_Hmi_Mode_Edit_Param_Max_Value) /= u16_p_param_factor;
      }

      // Check if the variable is of type 8/16-bit (used for special representation)
      if(LEN1(Commands_Table[u16_cmd_index-1].u8_flags))
      {
         BGVAR(u8_Hmi_Mode_Edit_Parameter_Property) |= HMI_MODE_EDIT_PARAM_16_BIT_PARAM;
      }
      else
      {
         BGVAR(u8_Hmi_Mode_Edit_Parameter_Property) &= ~HMI_MODE_EDIT_PARAM_16_BIT_PARAM;
      }

      // Show lower 5 digits when switching to HMI mode parameter
      BGVAR(u8_Local_Hmi_Display_LSW) = 1;

      // Check if the local HMI is supposed to be represent the parameter in
      // decimal or in hexadecimal format.
      if ((Commands_Table[u16_cmd_index-1].u16_P_Param_Properties & 0x0003) == 0)
      {
         // Bit 0 in the P-parameter properties is low --> Choose decimal representation.
         BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) = 1;
      }
      else if ((Commands_Table[u16_cmd_index-1].u16_P_Param_Properties & 0x0003) == 1)
      {
         // Choose hexadecimal representation.
         BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) = 0;
      }
      else // Here the bit-combination is representing a 2
      {
         // Choose combined decimal representation.
         BGVAR(u8_Hmi_Mode_Edit_Param_Dec_Representation) = 2;
      }

      // Start with the lowest digit when switching to mode edit parameter.
      BGVAR(u8_Hmi_Mode_Edit_Param_Digit_Number) = 0;

      // Clear variable that performs a certain action (just to be sure)
      BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) = 0;

      // Ask for exclusive access-rights for specific P-parameter to be edited
      if( (BGVAR(u16_P_Param_Selection) == 1)   || // Fault reset
          (BGVAR(u16_P_Param_Selection) == 230) || // P2-30: Enable / Disable; Trigger Hold (QuickStop)
          (BGVAR(u16_P_Param_Selection) == 232) || // P2-32: Start auto-tuning
          (BGVAR(u16_P_Param_Selection) == 405) || // P4-05: Start Jog via lHMI
          (BGVAR(u16_P_Param_Selection) == 507)    // P5-07: Start Homing / Start PS
        )
      {
         s32_temp = 1;
         ExecutePParam(drive, 1032, OP_TYPE_WRITE, &s32_temp, LEX_CH_LOCAL_HMI);
      }

      // Clear the ExecuteP-param request (just to be sure)
      BGVAR(u8_Hmi_Mode_Edit_Param_Execute_P_Param) = 0;

      // Switch the state-machine to HMI-mode edit parameter
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_EDIT_PARAMETER;
   }
   else
   {
      // Now tell the HMI Mode Parameter function that there is something
      // to display for a certain amount of time.
      if (s16_return_value == UNKNOWN_COMMAND)
      {
         BGVAR(u8_Hmi_Mode_Parameter_Perform_Action) = 1;
      }
      else if (s16_return_value == LEXIUM_INVALID_ACCESS_RIGHTS)
      {
         BGVAR(u8_Hmi_Mode_Parameter_Perform_Action) = 2;
      }
      else
      {
         BGVAR(u8_Hmi_Mode_Parameter_Perform_Action) = 0xFF;
      }

      // Capture the 1ms counter value
      BGVAR(s32_Hmi_Mode_Parameter_Perform_Action_Timer) = Cntr_1mS;

      // Stay in the state-machine HMI-mode parameter
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_PARAMETER;
   }
}


//**********************************************************
// Function Name: PrepareCdhdJogMoveVariables
// Description:
//          This function adjusts some CDHD internal parameters required for the
//          Lexium JOG move or restores the parameters.
//
//          s16_setup: 0 = Restore parameters in the CDHD.
//                     1 = Change parameters in the CDHD in order to allow the JOG move.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void PrepareCdhdJogMoveVariables(int drive, int s16_setup)
{
   // AXIS_OFF;

   if(s16_setup == 1) // Prepare CDHD variables and store their previous values
   {
      // Tell the Lexium speed and torque command functions to not interfere the jog move.
      BGVAR(u16_Lex_Jog_Or_Autotune_Active) = 1;

      // First backup all variables, which are supposed to be changed by this function.
      // The restore of the variables is done when leaving "LocalHmiModeJogMove"
      BGVAR(s16_Hmi_Mode_Jog_Jogmode_Backup) = BGVAR(u16_P4_83_Jog_Method);

      // Here try to switch the OPMODE to 8, because the jog move is done in this OPMODE.
      if( ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_SERCOS)        ||
          ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERNET_IP)   ||
          ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERCAT)         )
      {
      }
      else if ((BGVAR(u16_P1_01_CTL_Current) & 0xff) == SE_OPMODE_CANOPEN)
      {
         // Capture the mode of operation
         BGVAR(s16_Hmi_Mode_Jog_Opmode_Backup) = p402_modes_of_operation_display;

         // Here try to switch the canopen opmode to 1
         BGVAR(s16_CAN_Opmode_Temp) = -1;  // -1 is =s= jog mode (internal opmode 8)
         BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
         CheckCanOpmode(drive);
      }
      else
      {
         BGVAR(s16_Hmi_Mode_Jog_Opmode_Backup) = VAR(AX0_s16_Opmode);
         SetOpmode(drive, 8);
         // fix IPR 1407: I/O mode: Jog isn't working if CAN is connected to the drive
         // object 0x6061 was mapped to tpdo and p402_modes_of_operation_display was overwritten by s16_CAN_Opmode
         BGVAR(s16_CAN_Opmode) = -1;   
         // Now set "p402_modes_of_operation" to -1, because this is required for the jog-move (see also "LexiumJogMoveHandler")
         
         p402_modes_of_operation_display = -1;
      }


      // Adjust for local HMI always the jog-move mode 0
      BGVAR(u16_P4_83_Jog_Method) = 0;
   }
   else // Restore the variables after finishing/terminating the jog move
   {
      // Allow the Lexium speed and torque command functions to do their regular job.
      BGVAR(u16_Lex_Jog_Or_Autotune_Active) = 0;

      // First restore the original settings (undo the changes when stepping out of JOG mode)
      if( ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_SERCOS)        ||
          ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERNET_IP)   ||
          ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERCAT)         )
      {
      }
      else if ((BGVAR(u16_P1_01_CTL_Current) & 0xff) == SE_OPMODE_CANOPEN)
      {
         BGVAR(s16_CAN_Opmode_Temp) = BGVAR(s16_Hmi_Mode_Jog_Opmode_Backup);
         BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
         CheckCanOpmode(drive);
      }
      else
      {
         SetOpmode(drive, BGVAR(s16_Hmi_Mode_Jog_Opmode_Backup));
         // fix IPR 1407: I/O mode: Jog isn't working if CAN is connected to the drive
         // object 0x6061 was mapped to tpdo and p402_modes_of_operation_display was overwritten by s16_CAN_Opmode
         BGVAR(s16_CAN_Opmode) = 0;
         p402_modes_of_operation_display = 0;
      }

      BGVAR(u16_P4_83_Jog_Method) = BGVAR(s16_Hmi_Mode_Jog_Jogmode_Backup);

      // Here command 0 speed for 1 time
      BGVAR(u16_Lexium_Jog_Move_Activate) = 0;  // Clear first jog move activate (required for jog move handler function)
      BGVAR(u16_Lexium_Jog_Move_Init_Stop) = 1; // Raise init stop flag
   }
}

//**********************************************************
// Function Name: SwitchToLocalHmiModeJogMove
// Description:
//          This function does some preparation work before switching to local
//          HMI Mode Jog Move.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void SwitchToLocalHmiModeJogMove(drive)
{
   static unsigned int u16_opmode_switch = 0;
   
   // If the access-rights allow a JOG move
   if (CheckLexiumAccessRights(drive, LEX_AR_FCT_ACTIVATE_JOG_MODE, LEX_CH_LOCAL_HMI) == SAL_SUCCESS)
   {
      if (u16_opmode_switch == 0)
      {
         // Setup CDHD parameters for Lexium jog move
         PrepareCdhdJogMoveVariables(drive, 1);
         u16_opmode_switch = 1;
         return;
      }
      else if (u16_opmode_switch == 1)
      {
         if (IsOpmodeChangeInProgress(drive)) return;
      }

      // Check if axis was enabled when stepping into jog-move mode
      if(!Enabled(drive))
      {
         EnableCommand(drive);
         BGVAR(u16_Local_Hmi_General_Purpose) |= HMI_MODE_JOG_AXIS_WAS_DISABLED;
      }
      else
      {
         BGVAR(u16_Local_Hmi_General_Purpose) &= ~HMI_MODE_JOG_AXIS_WAS_DISABLED;
      }

      // First clear the enable state indication for the jog move mode
      BGVAR(u16_Local_Hmi_General_Purpose) &= ~HMI_MODE_JOG_ENABLE_STATE;

      // Clear variable which is supposed to be used to indicate something
      BGVAR(u8_Hmi_Mode_Jog_Perform_Action) = 0;
   }
   else // No access-rights are granted
   {
      // Load action timer
      BGVAR(s32_Hmi_Mode_Jog_Perform_Action_Timer) = Cntr_1mS;
      // Show "Prot" and leave mode
      BGVAR(u8_Hmi_Mode_Jog_Perform_Action) = 1;
   }
   // Switch the state-machine to HMI-mode jog move
   BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_JOG_MOVE;
   u16_opmode_switch = 0;
}

//**********************************************************
// Function Name: LocalHmiModeJogMove
// Description:
//          This function handles the Lexium local HMI mode:
//          "HMI Mode Jog Move", which is a hidden (not documented) mode.
//          The local HMI enters this mode only in case that exclusive
//          access-rights have been granted to the local HMI.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void LocalHmiModeJogMove(drive)
{
   // AXIS_OFF;

   if(BGVAR(u8_Hmi_Mode_Jog_Perform_Action) == 1) // If Prot has to be displayed for 2[s]
   {
      // Write "Prot " to the local HMI display array
      GetLocalHmiStringDisplayValue("Prot ", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);

      // Show lower 5 digits of the "u16_Local_Hmi_Display_Array" array.
      BGVAR(u8_Local_Hmi_Display_LSW) = 1;

      // Let all five 7-segments blink
      BGVAR(u8_Local_Hmi_Blink_Display) = 0xFF;

      // After 2[s]
      if (PassedTimeMS(2000L, BGVAR(s32_Hmi_Mode_Jog_Perform_Action_Timer)))
      {
         /*** Leave the JOG-Move mode ***/
         // Switch to HMI Mode Parameter. Access rights are released when entering this transition.
         BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_PARAMETER;
      }

      return;
   }

   if(ButtonUpPressed(drive))
   {
      // Here initiate jog in positive direction
      BGVAR(u16_Lexium_Jog_Move_Activate) = 1;
   }
   else if(ButtonDownPressed(drive))
   {
      // Here initiate jog in negative direction
      BGVAR(u16_Lexium_Jog_Move_Activate) = 2;
   }
   else
   {
      // Here command 0 speed
      BGVAR(u16_Lexium_Jog_Move_Activate) = 0;
   }

   // ****************************************
   // Identify what to write to the 7-segments
   // ****************************************
   if (BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_LS_WARNING_PENDING)
   {
      // Show AL014 & AL015 specifically in this HMI mode since we are not supposed
      // to switch away from local HMI mode "jog move" due to active limit switches.
      // This is a request from Alexander Badura.
      // Fill the 7-segment array with the AL/Wn error number.
      GetLocalHmiHexDisplayValue((long)BGVAR(u16_Lexium_Alarm_Code), BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
      // Display "AL." in the digit 3 & 4
      BGVAR(u16_Local_Hmi_Display_Array)[4] = GetLocalHmiDigitCode('A');
      BGVAR(u16_Local_Hmi_Display_Array)[3] = GetLocalHmiDigitCode('L');
      BGVAR(u16_Local_Hmi_Display_Array)[3] &= DC_POINT; // Light up the decimal-point in digit 3, which is specific for displaying an Alarm
   }
   else if (BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_SHOW_HALT_IN_7_SEGMENTS)
   {
      // Write " HALt" to the local HMI display array
      GetLocalHmiStringDisplayValue(" HALt", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
   }
   else if(LVAR(AX0_s32_Pos_Vcmd) < 0)
   {
      // Write " JOG-" to the local HMI display array
      GetLocalHmiStringDisplayValue(" JOG-", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
   }
   else if(LVAR(AX0_s32_Pos_Vcmd) > 0)
   {
      // Write "-JOG " to the local HMI display array
      GetLocalHmiStringDisplayValue("-JOG ", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
   }
   else
   {
      // Write " JOG " to the local HMI display array
      GetLocalHmiStringDisplayValue(" JOG ", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
   }
   // Show lower 5 digits of the "u16_Local_Hmi_Display_Array" array.
   BGVAR(u8_Local_Hmi_Display_LSW) = 1;
   // Do not let the display blink
   BGVAR(u8_Local_Hmi_Blink_Display) = 0;

   // If user presses M -> Leave jog mode
   if(ButtonMRisingEdge(drive))
   {
      /*** Leave the JOG-Move mode ***/

      // First restore the original settings (undo the changes when stepping out of JOG mode)
      PrepareCdhdJogMoveVariables(drive, 0);

      // If the axis was NOT enabled when entering the local HMI mode Jog --> Axis was automatically enabled by drive
      if (BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_JOG_AXIS_WAS_DISABLED)
      {
         // Disable when leaving the jog-move mode
         DisableCommand(drive);
      }

      // Switch to HMI Mode Parameter. Access rights are released when entering this transition.
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_PARAMETER;
   }

   // If the Drive is disabled (e.g. by an external source) during the JOG move
   if(!Enabled(drive))
   {
      // If the axis was previously enabled. This means if there is a falling edge on the enable signal.
      if(BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_JOG_ENABLE_STATE)
      {
         // First restore the original settings (undo the changes when stepping out of JOG mode)
         PrepareCdhdJogMoveVariables(drive, 0);
         // Switch to HMI Mode Parameter. Access rights are released when entering this transition.
         BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_PARAMETER;
      }
   }
   else
   {
      // State that the axis was at least for 1 BG cycle enabled
      BGVAR(u16_Local_Hmi_General_Purpose) |= HMI_MODE_JOG_ENABLE_STATE;
   }
}

//**********************************************************
// Function Name: SwitchToLocalHmiModeAutotune
// Description:
//          This function does some preparation work before switching to local
//          HMI Mode Autotune.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void SwitchToLocalHmiModeAutotune(drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(BGVAR(u16_Auto_Adaptive_Tuning_Triggered) == 0)
   {
      // Clear variable that allows leaving the auto-tuning mode
      BGVAR(u16_Local_Hmi_General_Purpose) &= ~(HMI_MODE_ATUNE_ALLOW_LEAVING_MODE | HMI_MODE_ATUNE_ENABLE_STATE | HMI_MODE_ATUNE_FINALIZE_BY_LHMI);

      // Switch to lHMI mode auto-tune
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_AUTOTUNE;
   }
   else // For auto-adaptive tuning that runs continuously in background
   {
      // Switch to HMI Mode Parameter.
      BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_PARAMETER;
   }
}


//**********************************************************
// Function Name: LocalHmiModeAutotune
// Description:
//          This function handles the Lexium local HMI mode:
//          "HMI Mode Autotune".
//          The local HMI enters this mode only in case that exclusive
//          access-rights have been granted to the local HMI.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void LocalHmiModeAutotune(drive)
{
   // AXIS_OFF;

   // Ask for the tuning progress in [%]
   int s16_pos_tune_progress = HdTuneProgressBar(drive,0);

   // Variable that holds the information if the auto-tuner is finished
   unsigned int u16_auto_tuning_done;

   // Fix IPR 1456: [AutoTuning] Enhancements When starting an AutoTuning with the HMI (P2-32=1) the HMI displays "Done" quickly at the beginning
   // consider u16_Lexium_Autotune_State for determine if auto tune is done or not. 
   // This way, lHMI will display auto tune percentegae even when auto tune has not started 
   // and preperation work is in progress (chnaging opmode, enable drive and so on).
   // Also set percentage to zero to prevent showing 100% on start until the auto tune process actually starts
   if (BGVAR(u16_Lexium_Autotune_State) <= LEX_ATUNE_STATE_EC_TRIGGER_HDTUNE)
   {
      u16_auto_tuning_done = 0;
      s16_pos_tune_progress = 0;
   }
   else if ((VAR(AX0_s16_Cycle_Bits) & POS_TUNE_RT_ACTIVE) && (BGVAR(s16_Pos_Tune_State) < POS_TUNE_SAVE_MODE))
   {
      u16_auto_tuning_done = 0;
   }
   else
   {
      u16_auto_tuning_done = 1;
   }

   // *** Here decide what to display on the 7-segments ***
   if (u16_auto_tuning_done == 0) // If the auto tuning is running
   {
      // Fill the array that holds the 7-segment data with the decimal value if the progress.
      GetLocalHmiDecimalDisplayValue((long)s16_pos_tune_progress, (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE, 0);
      // Write "tn" to the local HMI display array at digit 3 & 4
      BGVAR(u16_Local_Hmi_Display_Array)[4] = GetLocalHmiDigitCode ('t');
      BGVAR(u16_Local_Hmi_Display_Array)[3] = GetLocalHmiDigitCode ('n');
   }
   else
   {
      // Write "Done" to the local HMI display array
      GetLocalHmiStringDisplayValue("donE ", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);
   }

   // Show lower 5 digits of the "u16_Local_Hmi_Display_Array" array.
   BGVAR(u8_Local_Hmi_Display_LSW) = 1;
   // Do not let the display blink
   BGVAR(u8_Local_Hmi_Blink_Display) = 0;

   // If user presses the M button while auto-tuner is active --> Abort
   if(ButtonMRisingEdge(drive))
   {
      // If the auto-tuner is running
      if (u16_auto_tuning_done == 0)
      {
         // Trigger an auto-tune abort requested by the lHMI
         BGVAR(u16_Lexium_Autotune_General_Purpose) |= LEX_ABORT_AUTOTUNE;
         // Indicate that the finalize action has been triggered by the local HMI
         BGVAR(u16_Local_Hmi_General_Purpose) |= HMI_MODE_ATUNE_FINALIZE_BY_LHMI;
      }
      else
      {
         // Trigger an auto-tune discard requested by the lHMI
         BGVAR(u16_Lexium_Autotune_General_Purpose) |= LEX_FINALIZE_AUTOTUNE_AND_DISCARD;
         // Indicate that the finalize action has been triggered by the local HMI
         BGVAR(u16_Local_Hmi_General_Purpose) |= HMI_MODE_ATUNE_FINALIZE_BY_LHMI;
      }
   }

   // If user presses ENT and auto-tuner is finished
   if(ButtonEntRisingEdge(drive))
   {
      // Trigger an auto-tune finalization requested by the lHMI
      BGVAR(u16_Lexium_Autotune_General_Purpose) |= LEX_FINALIZE_AUTOTUNE_AND_SAVE;
      // Indicate that the finalize action has been triggered by the local HMI
      BGVAR(u16_Local_Hmi_General_Purpose) |= HMI_MODE_ATUNE_FINALIZE_BY_LHMI;
   }

   // If the user pressed any button to leave the mode and if the state-machine is finished (do not release access-rights earlier
   // due to an unexpected Stop-condition)
   if((BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_ATUNE_FINALIZE_BY_LHMI)   &&    // Finalize request from lHMI
      (BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_ATUNE_ALLOW_LEAVING_MODE) &&    // Auto-tuner was at least for one cycle active
      (BGVAR(u16_Lexium_Autotune_State) == LEX_ATUNE_STATE_IDLE))                      // Auto-tuner in state idle
   {
      // As long as a "Save" is pending for the Lexium
      if((BGVAR(u16_Lexium_Bg_Action_Applied) & LEX_DO_NV_SAVE) ||
                 (BGVAR(u16_Lexium_Bg_Action) & LEX_DO_NV_SAVE))

      {
         // Write the word "donE" to the 7-segment
         GetLocalHmiStringDisplayValue("donE ", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);

         BGVAR(u8_Local_Hmi_Blink_Display) = 0xFF; // Let all five 7-segements blink
         BGVAR(u8_Local_Hmi_Display_LSW) = 1; // Display LSW of the array (digits 0...4)

         // Load the Timer used for displaying something for 2[s]
         BGVAR(s32_Hmi_Mode_Auto_Tune_Perform_Action_Timer) = Cntr_1mS;
      }
      else if(!PassedTimeMS(2000L, BGVAR(s32_Hmi_Mode_Auto_Tune_Perform_Action_Timer)))
      {
         // Display for 2[s] "SAvEd"
         GetLocalHmiStringDisplayValue("SAvEd", (unsigned int *)BGVAR(u16_Local_Hmi_Display_Array), LOCAL_HMI_DISPLAY_ARRAY_SIZE);

         BGVAR(u8_Local_Hmi_Blink_Display) = 0xFF; // Let all five 7-segements blink
         BGVAR(u8_Local_Hmi_Display_LSW) = 1; // Display LSW of the array (digits 0...4)
      }
      else
      {
         // Leave auto-tune mode
         BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_PARAMETER;
      }
   }
   else
   {
         // Load the Timer with the value -2000[ms] in order to not accidentally show "SAvEd" on the 7-segments
         // in case that someone external issues the Lexium auto-tuner to stop. So here we put the "action timer"
         // to a defined state!
         BGVAR(s32_Hmi_Mode_Auto_Tune_Perform_Action_Timer) = Cntr_1mS - 2000;
   }

   // If the Drive is disabled (e.g. by an external source) during the auto-tune
   if(!Enabled(drive))
   {
      // If the axis was previously enabled. This means if there is a falling edge on the enable signal.
      if((BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_ATUNE_ENABLE_STATE) &&             // If falling edge on enable signal
         ((BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_ATUNE_FINALIZE_BY_LHMI) == 0))    // and if no putton pressed in the lHMI to finish the auto-tuner
      {
         // Switch to HMI Mode Parameter. Access rights are released when entering this transition.
         BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_PARAMETER;
      }
   }
   else
   {
      // State that the axis was at least for 1 BG cycle enabled
      BGVAR(u16_Local_Hmi_General_Purpose) |= HMI_MODE_ATUNE_ENABLE_STATE;
   }

   // If the Lexium auto-tuner is in progress
   if(BGVAR(u16_Lexium_Autotune_State) > LEX_ATUNE_STATE_IDLE)
   {
      // Indicate that it is allowed to leave the mode when reaching next time state idle.
      BGVAR(u16_Local_Hmi_General_Purpose) |= HMI_MODE_ATUNE_ALLOW_LEAVING_MODE;
   }
}

//**********************************************************
// Function Name: LocalHmiControl
// Description:
//          This function is handles the Lexium local HMI,
//          which are the 7-segments and the push-buttons.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void LocalHmiControl(int drive)
{
   static long s32_display_state_timer = 0;
   static unsigned char u8_current_segment=1,u8_current_CAN_led_segment=1, u8_current_digit=0;
   static unsigned int u16_display_state_counter = 0;
   /******************************************************************/
   /*** Local variables used to forward the digit-code to the FPGA ***/
   /*** registers and blinking certain 7-segments on demand.       ***/
   /******************************************************************/
   unsigned char u8_Temp_Index;
   unsigned int u16_Digit_Code;
   int s16_halt_active_pt_mode;
   int s16_halt_active_lxm_profile_mode;
   int s16_halt_active_canopen_mode;
   unsigned int u16_alarm_code;
   
   if ((u16_Product != SHNDR_HW) || (IS_EC2_DRIVE_KEYPAD)) // Handle the local HMI only if it's a Schneider hardware Drive
   {
      return;
   }
   // check the ModBus LED indication state. reset bit if no traffic after 500ms
   if (PassedTimeMS(500L,BGVAR(s32_ModBus_Ind_TimeStamp)))
   {

      // if new packet received
      if(BGVAR(u16_ModBus_Ind) & MODBUS_IND_MSG_INDICATION)
      {// if there is still modbus traffic after 500ms toggle the LED an

         BGVAR(u16_ModBus_Ind) ^= MODBUS_IND_TOGGLE_STATE;

         // reset new packet received bit (assume that BGVAR(u16_ModBus_Ind) is only modified in BG).
         BGVAR(u16_ModBus_Ind) &= ~MODBUS_IND_MSG_INDICATION;
      }
      // take a new time stamp
      BGVAR(s32_ModBus_Ind_TimeStamp) = Cntr_1mS;
   }


   //*******************************************************
   // Update the state of the keys on the front of the Drive
   // ******************************************************
   LocalHmiUpdateKeyStates(drive);

   // **************************************************************************
   // Here identify if a "HALt" string is supposed to be shown in the 7-segments.
   // if drive is enabled and halt is set vias canopen or HALT input
   // IPR 777: if in drive profile lexium only show HALT when halt bit is requested
   // (ignore halt when issued by Lxm_Handle_Drive_Ctrl() function)
   // **************************************************************************
   BGVAR(u16_Local_Hmi_General_Purpose) &= ~HMI_MODE_SHOW_HALT_IN_7_SEGMENTS;
   s16_halt_active_pt_mode = (BGVAR(u16_Hold_Input_State) == 1) && (BGVAR(u16_LXM28_Opmode) == SE_OPMODE_PT);
   s16_halt_active_lxm_profile_mode = (u16_Lxm_DriveStat.bit.Halt && (BGVAR(s16_P3_10_PlcEn) == 1) && ((BGVAR(u16_P1_01_CTL_Current) & 0xff) == SE_OPMODE_CANOPEN));
   s16_halt_active_canopen_mode = ((BGVAR(u16_Hold_Bits) & HOLD_HALT_MASK) && (BGVAR(s16_P3_10_PlcEn) == 0) && ((BGVAR(u16_P1_01_CTL_Current) & 0xff) == SE_OPMODE_CANOPEN));
   if ((s16_halt_active_canopen_mode || s16_halt_active_pt_mode || s16_halt_active_lxm_profile_mode) &&
       ((p402_statusword & STATUSWORD_STATE) == OPERATION_ENABLED))
   {
      BGVAR(u16_Local_Hmi_General_Purpose) |= HMI_MODE_SHOW_HALT_IN_7_SEGMENTS;
   }

   // **************************************************************
   // Check if an Error or user-hold is pending and determine if the
   // state-machine has to switch automatically to "HMI Mode Alarm"

   // nitsan: do nothing. dont show CAN bus off warning on display (0x0185).
   // as dicussed with Joachim 29/10/2014
   // mask wn185 to fix IPR 1024 (Wn737 dissapears after conecting to PLC).
   // **************************************************************
   u16_alarm_code = GetLexiumFaultNumber(drive, WRN_CAN_BUS_ERROR);
   if (u16_alarm_code != 0x0185)
   {
      BGVAR(u16_Lexium_Alarm_Code) = u16_alarm_code;
   }

   // Check if a limit switch warning is pending and indicate this in the general purpose variable
   if((BGVAR(u16_Lexium_Alarm_Code) == 0x0014) || (BGVAR(u16_Lexium_Alarm_Code) == 0x0015) || (BGVAR(u16_Lexium_Alarm_Code) == 0x0283) || (BGVAR(u16_Lexium_Alarm_Code) == 0x0285))
   {
      BGVAR(u16_Local_Hmi_General_Purpose) |= HMI_MODE_LS_WARNING_PENDING;
   }
   else
   {
      BGVAR(u16_Local_Hmi_General_Purpose) &= ~HMI_MODE_LS_WARNING_PENDING;
   }

   // **************************************************************************
   // At this place the code checks if the local HMI has to switch automatically
   // to local HMI mode Alarm in case that an unexpected situation appears, such
   // as a fault, a certain warning or a fieldbus triggered HALT situation.
   // **************************************************************************
   if ((BGVAR(u16_Lexium_Alarm_Code) != 0)  || (BGVAR(u16_Hold_Bits) & HOLD_LXM_ACCESS_RIGHTS_MASK) || (BGVAR(u16_Local_Hmi_General_Purpose) & HMI_MODE_SHOW_HALT_IN_7_SEGMENTS))
   {
      if (BGVAR(u8_Local_Hmi_Error_Pending) == 0) // If the error just happens -> Switch IMMEDIATELY to HMI Mode Alarm
      {
         // If there is a SW or HW limit-switch warning or a field bus HALT during local HMI mode "jog move"
       // Or default param during Auto tune
         if( ((BGVAR(u16_Local_Hmi_State) == STATE_HMI_MODE_JOG_MOVE) &&
             (BGVAR(u16_Local_Hmi_General_Purpose) & (HMI_MODE_SHOW_HALT_IN_7_SEGMENTS | HMI_MODE_LS_WARNING_PENDING))) ||
             ((BGVAR(u16_Local_Hmi_State) == STATE_HMI_MODE_AUTOTUNE) && (u16_Lexium_Alarm_Code == 0x737)) )
         {
            // Do nothing, just stay in local HMI mode "jog move". Request from Alexander Badura.
         }
         else
         {
            // If an error/warning happens while being in jog-move
            if(BGVAR(u16_Local_Hmi_State) == STATE_HMI_MODE_JOG_MOVE)
            {
               // Restore CDHD settings when leaving jog move
               PrepareCdhdJogMoveVariables(drive, 0);
            }

            BGVAR(u8_Local_Hmi_Error_Pending) = 1;
            BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_ALARM;
         }
      }
      // User switched to another mode although error was pending. If we are currently NOT
      // in HMI mode 'Alarm' or about to switch to mode 'Alarm'.
      else if ( (BGVAR(u16_Local_Hmi_State) != STATE_HMI_MODE_SWITCH_TO_ALARM) &&
                (BGVAR(u16_Local_Hmi_State) != STATE_HMI_MODE_ALARM) )
      {
         // Check if 20[s] without pressing a button happened
         if (PassedTimeMS(20000L, BGVAR(s32_Local_Hmi_No_Button_Pressed_Timer)))
         {
            // If there is a SW or HW limit-switch warning or a field bus HALT during local HMI mode "jog move"
            if( ((BGVAR(u16_Local_Hmi_State) == STATE_HMI_MODE_JOG_MOVE) &&
                 (BGVAR(u16_Local_Hmi_General_Purpose) & (HMI_MODE_SHOW_HALT_IN_7_SEGMENTS | HMI_MODE_LS_WARNING_PENDING))) ||
                ((BGVAR(u16_Local_Hmi_State) == STATE_HMI_MODE_AUTOTUNE) && (u16_Lexium_Alarm_Code == 0x737)) )
            {
               // Do nothing, just stay in local HMI mode "jog move" . Request from Alexander Badura.
                // Same for Autotune
            }
            else
            {

               BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_ALARM;
            }
         }
      }
   }
   else
   {
      // Clear variable that states that an error is pending
      BGVAR(u8_Local_Hmi_Error_Pending) = 0;
   }

   // ****************************************
   // State machine of the local HMI functions
   // ****************************************
   switch(BGVAR(u16_Local_Hmi_State))
   {
      case (STATE_HMI_MODE_SWITCH_TO_ALARM):
         // Call the appropriate switch to function
         SwitchToLocalHmiModeAlarm(drive);
      break;

      case (STATE_HMI_MODE_ALARM):
         // Call the appropriate HMI mode function
         LocalHmiModeAlarm(drive);
      break;

      case (STATE_HMI_MODE_SWITCH_TO_MONITOR):
         // Call the appropriate switch to function
         SwitchToLocalHmiModeMonitor(drive);
      break;

      case (STATE_HMI_MODE_MONITOR):
         // Call the appropriate HMI mode function
         LocalHmiModeMonitor(drive);
      break;

      case (STATE_HMI_MODE_SWITCH_TO_PARAMETER):
         // Call the appropriate switch to function
         SwitchToLocalHmiModeParameter(drive);
      break;

      case (STATE_HMI_MODE_PARAMETER):
         // Call the appropriate HMI mode function
         LocalHmiModeParameter(drive);
      break;

      case (STATE_HMI_MODE_SWITCH_TO_EDIT_PARAMETER):
         // Call the appropriate switch to function
         SwitchToLocalHmiModeEditParameter(drive);
      break;

      case (STATE_HMI_MODE_EDIT_PARAMETER):
         // Call the appropriate HMI mode function
         LocalHmiModeEditParameter(drive);
      break;

      case (STATE_HMI_MODE_SWITCH_TO_JOG_MOVE):
         // Call the appropriate switch to function
         SwitchToLocalHmiModeJogMove(drive);
      break;

      case (STATE_HMI_MODE_JOG_MOVE):
         // Call the appropriate HMI mode function
         LocalHmiModeJogMove(drive);
      break;

      case (STATE_HMI_MODE_SWITCH_TO_AUTOTUNE):
         // Call the appropriate HMI mode function
         SwitchToLocalHmiModeAutotune(drive);
      break;

      case (STATE_HMI_MODE_AUTOTUNE):
         // Call the appropriate HMI mode function
         LocalHmiModeAutotune(drive);
      break;

      default:
         // Invalid state, this is never supposed to happen
         BGVAR(u16_Local_Hmi_State) = STATE_HMI_MODE_SWITCH_TO_MONITOR;
      break;
   }
   
   /*************************************************************************/
   /*** Here forward the digit-code to the FPGA registers and let several ***/
   /*** 7-segments blink on demand with a 2[Hz] frequency.                ***/
   /*************************************************************************/
   if ((u16_Product == SHNDR_HW) && (Test_Led_On >= 1)) // To handle DISPLAYTEST
   {
      switch (Test_Led_On)
            {
                case 1:
                case 2:
                   if ( (PassedTimeMS(250L, s32_display_state_timer)) || (Test_Led_On == 1) )
                   {
                      s32_display_state_timer = Cntr_1mS;
                      for (u8_Temp_Index=0; u8_Temp_Index<5; u8_Temp_Index++)
                      {
                         HWDisplay(ALL, u8_Temp_Index);
                      }

                      CANLed(3);
                      Test_Led_On = 3;
                   }
                break;

                case 3:
                   if (PassedTimeMS(250L, s32_display_state_timer))
                   {
                      s32_display_state_timer = Cntr_1mS;
                      for (u8_Temp_Index=0; u8_Temp_Index<5; u8_Temp_Index++)
                      {
                         HWDisplay(NONE, u8_Temp_Index);
                      }

                      CANLed(0);
                      Test_Led_On = 2;

                      u16_display_state_counter ++;
                      if (u16_display_state_counter == 4)
                      {
                         Test_Led_On = 0;
                         u16_display_state_counter = 0;

                         // restore last known state of CAN leds (before DISPLAYTEST)
                         CANLed(u16_Can_Leds_Value);
                      }
                   }
                break;


               case 4:
                  u8_current_segment=0x1;
                  u8_current_CAN_led_segment=0;
                  u8_current_digit=0;

               case 5:// turn ON each segment separatly
               case 6:// turn OFF each segment separatly
                  if (PassedTimeMS(250L, s32_display_state_timer) || (Test_Led_On == 4)/*if == 4 bypass the timer for first time*/)
                  {
                     if(Test_Led_On == 4)
                        Test_Led_On=5;

                     s32_display_state_timer = Cntr_1mS;

                     if(u8_current_segment > 0xFF)
                     {
                        u8_current_digit++;
                        u8_current_segment=0x1;

                        if(u8_current_digit > 0x4)
                        {
                           u8_current_digit = 0;
                           if(Test_Led_On == 5)
                           {// state 2 finished reset and run state 3
                              Test_Led_On = 6;
                              break;
                           }
                           else if(Test_Led_On == 6)
                           {// state 3 finished reset and stop
                              Test_Led_On = 0;
                              break;
                           }
                        }
                     }

                     if(u8_current_CAN_led_segment > 0x3)
                     {
                        u8_current_CAN_led_segment = 0;
                     }

                     // loop over all 5 7-segments in the LXM display.
                     for (u8_Temp_Index=0; u8_Temp_Index<5; u8_Temp_Index++)
                     {
                        if(Test_Led_On == 5)
                        {//state 5: ON
                           if(u8_current_digit == u8_Temp_Index)
                           {
                              HWDisplay(~u8_current_segment, u8_Temp_Index);
                           }
                           else
                           {
                              HWDisplay(NONE, u8_Temp_Index);
                           }
                        }
                        else if(Test_Led_On == 6)
                        {// state 6: OFF
                           if(u8_current_digit == u8_Temp_Index)
                           {
                              HWDisplay(u8_current_segment, u8_Temp_Index);
                           }
                           else
                           {
                              HWDisplay(ALL, u8_Temp_Index);
                           }
                        }
                     }
                     CANLed(u8_current_CAN_led_segment);

                     u8_current_CAN_led_segment++;
                     u8_current_segment <<= 1;
                  }
               break;
            }
   }
   else
   {
      for (u8_Temp_Index=0; u8_Temp_Index<5; u8_Temp_Index++)
      {
         // Determine the digit code to be forwarded to the FPGA register
         if(BGVAR(u8_Local_Hmi_Display_LSW))
         {
            u16_Digit_Code = BGVAR(u16_Local_Hmi_Display_Array)[u8_Temp_Index];
         }
         else
         {
            u16_Digit_Code = BGVAR(u16_Local_Hmi_Display_Array)[u8_Temp_Index+5];
         }
         u16_Digit_Code += 0;
         // If one or all 7-segments are supposed to blink
         if((BGVAR(u8_Local_Hmi_Blink_Display) == (u8_Temp_Index + 1)) || (BGVAR(u8_Local_Hmi_Blink_Display) == 0xFF))
         {
            if (Cntr_1mS & 0x00000100) // 7-segment is supposed to be off
            {
                if( u8_Is_Retro_Display ) HWDisplay(NONE ,u8_Temp_Index);
            }
            else // 7-segment is supposed to display the value
            {
                if( u8_Is_Retro_Display ) HWDisplay(u16_Digit_Code ,u8_Temp_Index);
            }
         }
         else // 7-segment is supposed to display the value
         {
              if( u8_Is_Retro_Display ) HWDisplay(u16_Digit_Code ,u8_Temp_Index);
         }
      }
   }
}



//**********************************************************
// Function Name: ConvertDriveStatusValue
// Description:
//    adapted of GetDriveStatusValue.
//    Converts recorded data from internal units to user units,
//    AND triggere level from user units to internal units.
//    for Shadow-parameters in the P11 group,
//    Duplicate P0-02 functionality
//
//    s16_Value_To_Display - Value specified in P0-02 of the Lexium document - Same as xx in P11-xx param.
// Author: APH / Udi
// Algorithm:
// Revisions:
//**********************************************************
long s32_Debug_Ret_Val;
long ConvertDriveStatusValue(int drive, long long s64_Raw_Data,  int s16_Value_To_Display, int s16_To_User)
{
   // AXIS_OFF;
   long s32_ret_val = 0;
   long long s64_temp;
   int s16_Unit_Idx = -1;
   int s16_unit_factor = 1;
   long s32_Rounding_Offset = 0; // Rounding offset variable for data-representation (1 count less). This offset has been identified as needed during testing
   s32_Debug_Ret_Val = 0;


   switch (s16_Value_To_Display)
   {
       case (0): // Motor feedback   in PUU
       case (1): // Position command in PUU
       case (2): // Positition error in PUU
           s16_Unit_Idx = POSITION_PUU_CONVERSION;    break;

       case (3):  // Motor feedback  in counts, 1280000 counts per rev
       case (4): // Position command
       case (5): // Position error
           s16_Unit_Idx = POSITION_PULSE_CONVERSION;    break;

       case (6): // Input frequency of the pulse commands in [kpps].
                 // As of Dec 2013, only Encoder_Follower2 active for =S=
           if(s16_To_User)
           {
               s32_ret_val = (long)s64_Raw_Data;

               // Reduce the value in case that the gearing input interpolation has been activated (see GEARINMODE).
               // Since the input pulse frequency is in reality lower by the factor in the formula below.
               s32_ret_val = s32_ret_val >> (VAR(AX0_u16_Qep_Out_Scale) - VAR(AX0_u16_Qep_In_Scale_Design));

               if ((VAR(AX0_u8_Gear_Mode) == 0) || (VAR(AX0_u8_Gear_Mode) == 3))
                {
                   // Reduce value by 4 since the FPGA counts edges (4 edges per A_quad_B pulse)
                   s32_ret_val = s32_ret_val >> 2;
                }

           }
           else
           { //Same code, reverse shift
               s32_ret_val = (long)s64_Raw_Data;
               s32_ret_val = s32_ret_val << (VAR(AX0_u16_Qep_Out_Scale) - VAR(AX0_u16_Qep_In_Scale_Design));

               if ((VAR(AX0_u8_Gear_Mode) == 0) || (VAR(AX0_u8_Gear_Mode) == 3))
               {
                   // Reduce value by 4 since the FPGA counts edges (4 edges per A_quad_B pulse)
                   s32_ret_val = s32_ret_val << 2;
               }

           }
           break;
       case (7): // Motor rotation speed [rpm] with 1 decimal place (600[rpm] = 6001 on the 7-segment)

           //   s32_ret_val = (long)((s64_Raw_Data * 18750) >> 24);
           s16_Unit_Idx =  VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION; //!!Test only May 12 2014
           break;
       case (8): // Speed input command in Volt --> Analog input value 1 in [V] in x.yy format (ANIN1)
       case (10): // Torque input command in Volt --> Analog input value 2 in [V] in x.yy format (ANIN2)
           // Also Divided by 10 after the unit conversion, to show value in 1/100V not 1/1000V
           s16_Unit_Idx = ANALOG_IO_CONVERSION;
           s16_unit_factor = 10;
           break;


       case (9):  // Speed command in [rpm] (VCMD)
           s16_Unit_Idx = VELOCITY_IN_LOOP_CONVERSION; //( 0.001 RPM).
           s16_unit_factor = 1000;
           break;
       case (50): // Speed (input) command in [0.1*rpm] (VCMD)
           s16_Unit_Idx = VELOCITY_IN_LOOP_CONVERSION; // RPM with 3 decimal digits  ( 0.001 RPM).
           s16_unit_factor = 100;
           break;


       case (11): // Torque input command in [%]
       case (53): // Torque input command in [0.1*%]
         // Avoid problems of the representation of one count less
           if(s16_To_User)
           {
               if (VAR(AX0_s16_Icmd) < 0)
               {
                   s32_Rounding_Offset = -BGVAR(s32_Motor_I_Cont_Internal)>>1;
               }
               else
               {
                   s32_Rounding_Offset = BGVAR(s32_Motor_I_Cont_Internal)>>1;
               }

               if (s16_Value_To_Display == 11) // Unit [%]
               {
                  // Motor I cont represents 100.0%. Both variables ("AX0_s16_Icmd" and "s32_Motor_I_Cont_Internal") are both in
                  // internal current units (DIPEAK = 26214). Add 0.5 * motor I cont to avoid rounding issues.
                  s32_ret_val = (((long)VAR(AX0_s16_Icmd) * 100) + s32_Rounding_Offset) / BGVAR(s32_Motor_I_Cont_Internal);
               }
               else
               {// Unit [0.1 %]
                  // Motor I cont represents 100.0%. Both variables ("AX0_s16_Icmd" and "s32_Motor_I_Cont_Internal") are both in
                  // internal current units (DIPEAK = 26214). Add 0.5 * motor I cont to avoid rounding issues.
                  s32_ret_val = (((long)VAR(AX0_s16_Icmd) * 1000) + s32_Rounding_Offset) / BGVAR(s32_Motor_I_Cont_Internal);
               }


           }
           else
           {
              if (s16_Value_To_Display == 11) // Unit [0.1%]
                 s32_ret_val = (long)(s64_Raw_Data * BGVAR(s32_Motor_I_Cont_Internal)/ 100 );
              else // Unit [0.1%]
                 s32_ret_val = (long)(s64_Raw_Data * BGVAR(s32_Motor_I_Cont_Internal)/ 10 );
           }

       break;
       case (14): // Main circuit voltage in [V]
           s32_ret_val = (long)s64_Raw_Data;
           break;
    // case (15): // LMJR have its own PParam P1-37
       case (16): // IGBT (Power) temperature
       case (27): // Control-board temperature
           s32_ret_val = (long)s64_Raw_Data;
           break;
       case (39): // Digital input status (Content of P4-07)
          if(s16_To_User)
          {
            s32_ret_val = (long)VAR(AX0_u16_P4_07_Read);//(long)s64_temp;
          }
          else //Trigger
          {    // Todo: Add conversion, also handle mask !
              s32_ret_val = (long)(s64_Raw_Data);
          }
          break;
       case (40): // Digital output status (Content of P4-09)
         if(s16_To_User)
          {

           GetOutputStatusDisplay(&s64_temp, (unsigned int)(s64_Raw_Data));
           s32_ret_val = (long)s64_temp;
          }
          else //Trigger
          {   // Todo: Add conversion, also handle mask !
              s32_ret_val = (long)(s64_Raw_Data);
          }
          break;
       case (54): // Feedback torque in unit 0.1% with 100%=MICONT
           if(s16_To_User)
           {
               s32_ret_val = (long)(((1000LL * s64_Raw_Data) / BGVAR(s32_Motor_I_Cont_Internal)));
           }
           else
           {
              s32_ret_val =  (long)((s64_Raw_Data * BGVAR(s32_Motor_I_Cont_Internal)) / 1000LL);
           }
           break;
       case (55): // Feedback torque in 0.01[A]
           if(s16_To_User)
           {
               s32_ret_val = (long)(s64_Raw_Data * BGVAR(s32_Drive_I_Peak) / 262140LL);
           }
           else
           {
               s32_ret_val = (long)(s64_Raw_Data *  262140LL / BGVAR(s32_Drive_I_Peak) );
           }

       break;

       case (77):   //PTPVCMD
           s16_Unit_Idx = VELOCITY_0_1_RPM_PTP_CONVERSION; // 0.1 RPM
           break;

       default: // Unsupported mode
          s32_ret_val = 0;
          break;

   }
   if(s16_Unit_Idx != -1)
      s32_ret_val = GetS32ValueByUnit(drive, s64_Raw_Data, s16_Unit_Idx, s16_To_User);

   if(s16_unit_factor != 1)
   {
       if(s16_To_User)
           s32_ret_val /= s16_unit_factor;
       else
           s32_ret_val *= s16_unit_factor;

   }
   s32_Debug_Ret_Val = s32_ret_val;
   return (s32_ret_val);
}

unsigned int ConvertDigitTo7SegDisplay(unsigned int u16_digit)
{
   unsigned int u16_ret = 0;

   switch (u16_digit)
   {
      case 0:
        u16_ret = ZERO;
     break;

     case 1:
        u16_ret = ONE;
     break;

     case 2:
        u16_ret = TWO;
     break;

     case 3:
        u16_ret = THREE;
     break;

     case 4:
        u16_ret = FOUR;
     break;

     case 5:
        u16_ret = FIVE;
     break;

     case 6:
        u16_ret = SIX;
     break;

     case 7:
        u16_ret = SEVEN;
     break;

     case 8:
        u16_ret = EIGHT;
     break;

     case 9:
        u16_ret = NINE;
     break;
   }
   return u16_ret;
}

//**********************************************************
// Function Name: SalReadCANOpmode
// Description:
//    This function reads CANOpen current opmode Object 0x6061.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
/*int SalReadCANOpmode(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error


   // Return the value of CAN opmode only for product
   // LXM28E ECAT or LXM28A in CAN mode.
   if(IS_LXM28A_DRIVE_CANOPEN || IS_LXM28E_DRIVE_ECAT)
   {
      *data = (long long)p402_modes_of_operation_display;
   }
   else
   {
      return (NOT_AVAILABLE);
   }

   return (SAL_SUCCESS);
}*/

unsigned int Cdhd2ConvertDigitTo7SegDisplay(unsigned int u16_digit)
{
   unsigned int u16_ret = 0;

   switch (u16_digit)
   {
      case 'O': 
      case '0':   
      case 0:
        u16_ret = ZERO;
     break;
     
     case '1': 
     case 1:
        u16_ret = ONE;
     break;
     
     case '2':
     case 2:
        u16_ret = TWO;
     break;

     case '3':
     case 3:
        u16_ret = THREE;
     break;
     
     case '4':
     case 4:
        u16_ret = FOUR;
     break;

     case '5':
     case 5:
        u16_ret = FIVE;
     break;
     
     case '6':
     case 6:
        u16_ret = SIX;
     break;

     case '7':
     case 7:
        u16_ret = SEVEN;
     break;

     case '8':
     case 8:
        u16_ret = EIGHT;
     break;

     case '9':
     case 9:
        u16_ret = NINE;
     break;
     
     case 'a':
        u16_ret = a_LET;
     break;
     case 'A':
        u16_ret = A_LET;
     break;
     
     case 'b':
     case 'B':
        u16_ret = b_LET;
     break;
     case 'C':
      u16_ret = C_LET;
     break;
     case 'c':
        u16_ret = c_LET;
     break;
     
     case 'd':
     case 'D':
        u16_ret = d_LET;
     break;
     
     case 'E':
        u16_ret = E_LET;
     break;
     case 'e':
        u16_ret = e_LET;
     break;
     
     case 'f':
     case 'F':
        u16_ret = F_LET;
     break;
     
     case 'G':
     case 'g':
        u16_ret = G_LET;
     break;
     
     case 'h':
        u16_ret = h_LET;
     break;
     case 'H':
        u16_ret = H_LET;
     break;
     
     case 'i':
     case 'I':
        u16_ret = I_LET;
     break;
     
     case 'j':
     case 'J':
        u16_ret = J_LET;
     break;
     
     case 'L':
     case 'l':
        u16_ret = L_LET;
     break;
     
     case 'N':
     case 'n':
        u16_ret = n_LET;
     break;
     
     case 'o':
        u16_ret = o_LET;
     break;
     
     case 'P':
     case 'p':
        u16_ret = P_LET;
     break;
     
     case 'Q':
     case 'q':
        u16_ret = q_LET;
     break;
     
     case 'R':
     case 'r':
        u16_ret = r_LET;
     break;
     
     case 's':
     case 'S':
        u16_ret = S_LET;
     break;
     
     case 'T':
     case 't':
        u16_ret = t_LET;
     break;
     
     
     case 'U':
     case 'u':
        u16_ret = u_LET;
     break;
     
     case 'Y':
     case 'y':
        u16_ret = Y_LET;
     break;
     
     case ' ':
        u16_ret = NONE;
     break;

     case '-':
        u16_ret = DASH_CHAR;
     break;

     case '.':
        u16_ret = 0x7F;
     break;
     
   }
   return u16_ret;
}

