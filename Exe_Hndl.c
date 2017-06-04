#include <string.h>
#include "ser_comm.def"
#include "Record.def"
#include "Err_Hndl.def"
#include "FltCntrl.def"
#include "MultiAxis.def"
#include "i2c.def"
#include "Design.def"
#include "PtpGenerator.def"

#include "Ser_Comm.var"
#include "Record.var"
#include "Err_Hndl.var"
#include "FltCntrl.var"
#include "Drive_Table.var"
#include "Drive.var"
#include "Init.var"
#include "Units.var"
#include "PtpGenerator.var"
#include "Flash.def"
#include "Flash.var"
#include "Motor.var"
#include "User_Var.var"

#include "Prototypes.pro"
#include "Display.pro"

#define IS_LOAD_UNITS (1 == Commands_Table[cmd].u16_dual_mode_units) && IS_DUAL_LOOP_ACTIVE
/**********************************************************
* Function Name: ExecutionHandler
* Description:
   This function is the execution handler manager. It manages the execution
   of the functions.
* Called From:
   CommsProcessor/communic.c
* Author: Hanan
* Input Parameters: None
* Output Parameters: None
* Return Value:
   0: Execution not yet finished
   1: Execution successful: finished

**********************************************************/
int ExecutionHandler(void)
{
   int execution_result;

   /*
   * Check whether to trigger the recording.
   * This is done BEFORE the execution handler is executed, since
   * otherwise a RECTRIG "CMD command would result in an immediate
   * trigger.
   * --------------------------------------
   */
   if (u8_Event_Trig_Mask == TRIG_ON_COMMAND)
   {
      /*
      * Set the s16_Event_Trig_Val to 1 in order to trigger the recording
      * -------------------------------------------------------------
      */
         s16_Event_Trig_Val = 1;
      /*
      * Clear the event trigger mask
      * ----------------------------
      */
      u8_Event_Trig_Mask = 0;
   }

   /*
   * Execute the function
   * --------------------
   */
      execution_result = ExecuteSerialCmd(s16_Command_Index, u8_Axis_Id);

   /*
   * Return the result: indicates to the Communication Processor how
   * to continue
   * ---------------------------------------------------------------
   */
   return( execution_result );
}


//**********************************************************
// Function Name: AsyncErrorMessage
// Description:
//         This function is called to send the asynchronous error message to the host.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void AsyncErrorMessage(void)
{
   char* p_local_response_ptr;
   unsigned int u16_error_code;

   //  Check if the async error message function is enabled AND
   //  the drive is addressed AND the drive is not globally addressed.
   if (BGVAR_COMM(u8_Comms_Options) & ERR_MSG_MASK)
   {
       PrintMostRecentError(u8_Axis_Id);
   }
    PrintMostRecentFault(u8_Axis_Id);

   if(!u8_FPGA_ok)//if FPGA was not downloaded correctlly - show fault after each user command
   {
      PrintString("FLT ",4);

      u16_error_code = GetFaultId((unsigned long long)FPGA_CONFIG_FLT_MASK, 0);

      p_local_response_ptr = GetFaultMsgPtr(u16_error_code);

      PrintUnsignedInteger(u16_error_code);

      PrintChar(SPACE);

      PrintString(p_local_response_ptr, 0);

      PrintCrLf();
   }
}


//**********************************************************
// Function Name: PrintMostRecentError
// Description:
//         This function print most recent error message to the host.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void PrintMostRecentError(int drive)
{
   int error_code;
   char* local_response_ptr;

//  Check if a new error has occurred
   if ( BGVAR(s16_New_Error) != BGVAR(s16_Most_Recent_Error) )
   {
      // Set the new error equal to the most recent, so that it will not be reported next time
      BGVAR(s16_New_Error) = BGVAR(s16_Most_Recent_Error);

      // Get the error code and the corresponding string
      local_response_ptr = ReadMostRecentError(drive, &error_code);

      PrintString("ERR ",4);

      PrintUnsignedInteger(error_code);

      PrintChar(SPACE);

      PrintString(local_response_ptr, 0);

      PrintCrLf();
   }
}


//**********************************************************
// Function Name: PrintMostRecentFault
// Description:
//         This function print most recent fault message to the host.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
void PrintMostRecentFault(int drive)
{
   int error_code;
   char* local_response_ptr;

   if ( BGVAR(u8_New_Fault) != BGVAR(s16_Most_Recent_Fault) )
   {
      // Set the new fault equal to the most recent, so that it will not be reported next time
      BGVAR(u8_New_Fault) = BGVAR(s16_Most_Recent_Fault);

      // Get the fault code and the corresponding string
      local_response_ptr = ReadMostRecentFault(drive, &error_code);

      PrintString("FLT ",4);

      PrintUnsignedInteger(error_code);

      PrintChar(SPACE);

      PrintString(local_response_ptr, 0);

      PrintCrLf();
   }
}

int ExecuteSerialCmd(int index, int drive)
{
   int return_value = 1;

   // Detect if it is a read or write command
   if ( (MIN_READ_PARAMS(Commands_Table[index].u16_min_max_arg) <= s16_Number_Of_Parameters) &&
        (MAX_READ_PARAMS(Commands_Table[index].u16_min_max_arg) >= s16_Number_Of_Parameters)   )
   {
      // Call the validation check function
      return_value = ValidateSerialRead(index, drive);

      if (return_value == SAL_SUCCESS)
         return_value = ExecuteSerialRead(index, drive); // call to read handler
   }
   else
   {
      // Call the validation check function
      return_value = ValidateSerialWrite(index, drive);

      if (return_value == SAL_SUCCESS)
         return_value = ExecuteSerialWrite(index, drive); // call to write handler
   }

   return return_value;   //return the result
}


//**********************************************************
// Function Name: ExecuteSerialRead
// Description:
//         This function reads and print parameter via serial communication
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int ExecuteReadCommand(int cmd, int drive, long long* ret_data)
{
   long long res = 0LL, temp = 0LL;
   long addr;
   int return_value = 1, temp_u8_flags;
   int s16_cntr_3125_capture;
   // AXIS_OFF;
   int axis_offset = drive;

   temp_u8_flags = Commands_Table[cmd].u8_flags;


   switch (READ_TYPE(Commands_Table[cmd].u8_rd_wr_method))
   {
      case RD_DPTR:   //read from variable
         addr = (long)Commands_Table[cmd].ptr_read_access;

         if (RT_VAR(temp_u8_flags))    // if RT var read from addr for AX0 or addr + 0x40 for AX1
         {
            addr = addr + axis_offset;
         }
         else                          // if BG var read from addr for AX0 or addr + len for AX1
         {
            if (LEN2(temp_u8_flags))
               addr = addr + (drive << 1);
            else
            {
               if (LEN4(temp_u8_flags))
                  addr = addr + (drive << 2);
               else
                  addr = addr + drive;
            }
         }

         if (PTR_VAR(temp_u8_flags))
         {
            if (LEN2(temp_u8_flags))
            {
               if (UNSIGNED_PARAM(temp_u8_flags))
                  res = (unsigned long)*(long *)(*(long *)addr);
               else
                  res = *(long *)(*(long *)addr);
            }
            else
            {
               if (LEN4(temp_u8_flags))
               {
                  s16_cntr_3125_capture = Cntr_3125;

                  if (UNSIGNED_PARAM(temp_u8_flags))
                     res = (unsigned long long)*(long long*)(*(long *)addr);
                  else
                     res = *(long long*)(*(long *)addr);

                  while (s16_cntr_3125_capture != Cntr_3125) // To maintain 64bit read consistancy
                  {
                     s16_cntr_3125_capture = Cntr_3125;
                     if (UNSIGNED_PARAM(temp_u8_flags))
                        res = (unsigned long long)*(long long*)(*(long *)addr);
                     else
                        res = *(long long*)(*(long *)addr);
                  }
               }
               else // read from int
               {
                  if (UNSIGNED_PARAM(temp_u8_flags))
                     res = (unsigned int)*(int *)(*(long *)addr);
                  else
                     res = *(int *)(*(long *)addr);
               }
            }
         }
         else
         {
            if (LEN2(temp_u8_flags))       // if len=2 read from long
            {
               if (UNSIGNED_PARAM(temp_u8_flags))
                  res = (unsigned long)*(long *)(addr);
               else
                  res = *(long *)(addr);
            }
            else
            {
               if (LEN4(temp_u8_flags))
               {
                  s16_cntr_3125_capture = Cntr_3125;

                  if (UNSIGNED_PARAM(temp_u8_flags))
                     res = (unsigned long long)*(long long*)(addr);
                  else
                     res = *(long long*)(addr);

                  while (s16_cntr_3125_capture != Cntr_3125) // To maintain 64bit read consistancy
                  {
                     s16_cntr_3125_capture = Cntr_3125;

                     if (UNSIGNED_PARAM(temp_u8_flags))
                        res = (unsigned long long)*(long long*)(addr);
                     else
                        res = *(long long*)(addr);
                  }
               }
               else //read from int
               {
                  if (UNSIGNED_PARAM(temp_u8_flags))
                     res = (unsigned int)*(int *)(addr);
                  else
                     res = *(int *)(addr);
               }
            }
         }
      break;

      case RD_FPTR0:   //read from function int func(int *data, int drive)
         return_value = ((int(*)(long long*, int))((long)Commands_Table[cmd].ptr_read_access))(&temp, drive);
         res = temp;
      break;

      case RD_FPTR1://read from function int func(int drive)
                    //usualy used for commands like CONFIG,CLREEPROM ... etc

         return_value = ((int(*)(int))((long)Commands_Table[cmd].ptr_read_access))(drive);
      break;

      case RD_FPTR2:   //read from function int func(int *data)
         return_value = ((int(*)(long long*))((long)Commands_Table[cmd].ptr_read_access))(&temp);

         res = temp;
      break;

      case RD_FCUST://custom function int func(int drive)
         return_value = ((int(*)(int))((long)Commands_Table[cmd].ptr_read_access))(drive);
      break;

      case RD_FCUST0://custom function int func(void)
         return_value = ((int(*)(void))((long)Commands_Table[cmd].ptr_read_access))();
      break;
   }

   if (return_value == 1)   // If read succeeded return the result
      *ret_data = res;

   return return_value;
}


// This functions returns the index for the relevant unit conversion (-1 if not found)
int UnitsConversionIndex(void *units_type, int drive)
{
   int i = 0, index = -1;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   while ((BGVAR(Unit_Conversion_Table[i]).units_type != 0) && (index == -1))
   {
      if (BGVAR(Unit_Conversion_Table[i]).units_type == units_type)
         index = i;
      i++;
   }
   return index;
}


//**********************************************************
// Function Name: ExecuteSerialRead
// Description:
//         This function reads and print parameter via serial communication
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int ExecuteSerialRead(int cmd, int drive)
{
   long long res, u64_temp_modulu = 0;
   int index = -1;
   int return_value;
   
   AXIS_OFF;
   return_value = ExecuteReadCommand(cmd, drive, &res);

   if (return_value == 1)
   {
      if (Commands_Table[cmd].str_units_var != 0)   // Handle units conversion
      {
         index = UnitsConversionIndex(Commands_Table[cmd].str_units_var, drive);
         if (IS_LOAD_UNITS)
         {
            index += 1; //In case Dual loop is active, use Load's units. otherwise, use Motor's units.
         }

         if (index >= 0) // Unit conversion found
         {
            res = MultS64ByFixS64ToS64(res,
                                       BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_user_fix,
                                       BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_user_shr);
         }
      }

      //for RD_DPTR | RD_FPTR0 | RD_FPTR2 print the return value over RS232
      if ( (READ_TYPE(Commands_Table[cmd].u8_rd_wr_method) == RD_DPTR) ||
           (READ_TYPE(Commands_Table[cmd].u8_rd_wr_method) == RD_FPTR0)||
           (READ_TYPE(Commands_Table[cmd].u8_rd_wr_method) == RD_FPTR2)  )
      {
         /* round decimal presentation for [Counts] units*/
         if (!strncmp((char*)Commands_Table[cmd].str_units_var,((char*)"counts"),7))
         {
            if (0 < res)
               res += 500LL;
            else
               res -= 500LL;
            u64_temp_modulu = res%1000;
            res -=  u64_temp_modulu;
         }
         //if parameter is unsigned, print unsigned value
         if (UNSIGNED_PARAM(Commands_Table[cmd].u8_flags))
         {
            if (DECIMAL(Commands_Table[cmd].u8_flags))
               PrintUnsignedLongLong(res);
            else
            {
               if (LEN2(Commands_Table[cmd].u8_flags))
                  PrintUnsignedLong(res);
               else if (LEN4(Commands_Table[cmd].u8_flags))
                  PrintUnsignedInt64(res);
               else
                  PrintUnsignedInteger((unsigned int)res);
            }
         }
         else
         {
            if (DECIMAL(Commands_Table[cmd].u8_flags))
               PrintSignedLongLong(res);
            else
            {
               if (LEN2(Commands_Table[cmd].u8_flags)) PrintSignedLong(res);
               else if (LEN4(Commands_Table[cmd].u8_flags)) PrintSignedInt64(res);
               else PrintSignedInteger((int)(res));
            }
         }

         if ( (Commands_Table[cmd].str_units_var == NULL) &&
              (Commands_Table[cmd].str_units_name != NULL)  )         // Print the units
         {
            PrintChar(SPACE);
            PrintChar('[');
            PrintString(Commands_Table[cmd].str_units_name, 0);
            PrintChar(']');
         }
         if (Commands_Table[cmd].str_units_var != NULL)
         {
            PrintChar(SPACE);
            PrintChar('[');
            if (IS_LOAD_UNITS) //print the Load's units rather than the motor's
               PrintString((char *)Unit_Conversion_Table[index].units_type + (drive * UNITS_STRING_SIZE), 0);
            else   
               PrintString((char *)Commands_Table[cmd].str_units_var + (drive * UNITS_STRING_SIZE), 0);
            PrintChar(']');
         }

         PrintCrLf();
      }
   }

   return return_value;
}


//**********************************************************
// Function Name: ExecuteWriteCommand
// Description:
//         This function writes a parameter from Drive Table
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int ExecuteWriteCommand(int cmd, int drive, long long* data)
{
   int return_value = 1, index = -1, s16_i;
   long addr = 0L;
   long long s64_current_value = 0LL;
   // AXIS_OFF;
   int axis_offset = drive;

   if(!BGVAR(u8_Is_Internal_Units))   // Handle units conversion
   {
      if (Commands_Table[cmd].str_units_var != 0)
      {
         index = UnitsConversionIndex(Commands_Table[cmd].str_units_var, drive);
		 if (IS_LOAD_UNITS)
         {
            index += 1; //In case Dual loop is active, use Load's units. otherwise, use Motor's units.
         }

         if (index >= 0) // Unit conversion found
         {
            if(WRITE_TYPE(Commands_Table[cmd].u8_rd_wr_method) == WR_FCUST1)
            {// preform unit conversion only on the 2nd argument
            if ((data[1] % 1000) != 0)//Check if argument is decimal or integer for sal verification
                  BGVAR(u16_Is_Not_Integer_Flag) = 1;

               data[1] = MultS64ByFixS64ToS64(data[1],
                         BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                         BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);
            }
            else
            {
               if ((data[0] % 1000) != 0)//Check if argument is decimal or integer for sal verification
                  BGVAR(u16_Is_Not_Integer_Flag) = 1;

               data[0] = MultS64ByFixS64ToS64(data[0],
                         BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                         BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);

               if (WRITE_TYPE(Commands_Table[cmd].u8_rd_wr_method) == WR_FPTR2)
               {
                  data[1] = MultS64ByFixS64ToS64(data[1],
                            BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                            BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);
               }
            }
         }
      }
   }

   // Don't allow modifying a parameter stored on the MTP when MTPMODE <> 0 as it will be oversritten
   // by the MTP data.
   // This is not done in ValidateSerialWrite as setting the same value as in the MTP is allowed
   // for the SSV to download without errors
   if (BGVAR(u16_MTP_Mode) != 0)
   {
      // Check if the write pointer appears in the MTP table
      for (s16_i = 0; s16_i < s16_MTP_Data_Table_Inc_Dummies_Size; s16_i++)
      {
         if (Commands_Table[cmd].ptr_read_access == p_MTP_Data_Table[s16_i].p_var_name)
         {
            // Check if the value is to be modified
            if (ExecuteReadCommand(cmd, drive, &s64_current_value) == SAL_SUCCESS)
            {
               if (s64_current_value != data[0])
               {
                  if (DECIMAL(Commands_Table[cmd].u8_flags))
                  // Due to numerical inaccuracies of unit conversion allow a small deviation in decimal values
                  {
                     if (llabs(s64_current_value - data[0]) > 50)
                        return (MTP_USED);
                  }
                  else
                     return (MTP_USED);
               }
            }
         }
      }
   }

   switch (WRITE_TYPE(Commands_Table[cmd].u8_rd_wr_method))
   {
      case WR_DPTR:
         addr = (long)Commands_Table[cmd].ptr_write_access;

         if (RT_VAR(Commands_Table[cmd].u8_flags))
         { // if RT var write to addr for AX0 or addr + 0x40 for AX1
            addr = addr + axis_offset;
         }
         else
         {//if BG var write to addr for AX0 or addr + len for AX1
            if (LEN2(Commands_Table[cmd].u8_flags))
               addr = addr + (drive << 1);
            else
            {
               if (LEN4(Commands_Table[cmd].u8_flags))
                  addr = addr + (drive << 2);
               else
                  addr = addr + drive;
            }
         }

         if (LEN2(Commands_Table[cmd].u8_flags)) // if len=2 write to long
            *(long *)(addr) = data[0];
         else
         {
            if (LEN4(Commands_Table[cmd].u8_flags)) // if len = 4 write to long long
               *(long long*)(addr) = data[0];
            else // write to int
               *(int *)(addr) = data[0];
         }
      break;

      case WR_FPTR0:// Write using function int func(long long param,int drive)
         return_value = ((int(*)(long long, int))((long)Commands_Table[cmd].ptr_write_access))(data[0], (drive));
      break;

      case WR_FPTR1:// Write using function int func(int drive)
         return_value = ((int(*)(int))((long)Commands_Table[cmd].ptr_write_access))(drive);
      break;

      case WR_FPTR2:// Write using function int func(long long param1,long long param2)
         return_value = ((int(*)(long long, long long))((long)Commands_Table[cmd].ptr_write_access))(data[0], data[1]);
      break;

      case WR_FCUST:// Write using function int func(int drive)
         return_value = ((int(*)(int))((long)Commands_Table[cmd].ptr_write_access))(drive);
      break;

      case WR_FCUST0:// Write using function int func(void)
         return_value = ((int(*)(void))((long)Commands_Table[cmd].ptr_write_access))();
      break;

      case WR_FCUST1:// Write using function int func(int drive)
         return_value = ((int(*)(int))((long)Commands_Table[cmd].ptr_write_access))(drive);
      break;
   }

   BGVAR(s16_Write_Cmd_Cntr)++; // This is to indicate to the GUI that a write command was executed

   BGVAR(u16_Is_Not_Integer_Flag) = 0;
   return return_value;
}


//**********************************************************
// Function Name: ExecuteSerialWrite
// Description:
//     This function writes parameter that arrived via serial communication
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int ExecuteSerialWrite(int cmd, int drive)
{
   return ExecuteWriteCommand(cmd, drive, s64_Execution_Parameter);
}


//**********************************************************
// Function Name: ValidateSerialRead
// Description:
//     This function validates if the command might be read
//     according to the various validation flags
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int ValidateSerialRead(int cmd, int drive)
{
   if ((Commands_Table[cmd].u16_validity_checks & 0x0001) && (s16_password_override==0)) // PASSWORD_PROTECTED
   {
      if (!s16_Password_Flag)   return (PASSWORD_PROTECTED);
   }

   drive++;   // just to defeat the compilation remark
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: ValidateSerialWrite
// Description:
//     This function validates if the command might be changed
//     according to the various validation flags
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int ValidateSerialWrite(int cmd, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((Commands_Table[cmd].u16_validity_checks & 0x0001) && (s16_password_override==0)) // PASSWORD_PROTECTED
   {
      if (!s16_Password_Flag)   return (PASSWORD_PROTECTED);
   }

   if (Commands_Table[cmd].u16_validity_checks & 0x0040) // DRIVE_ACTIVE
   {
      if (Enabled(DRIVE_PARAM)) return (DRIVE_ACTIVE);
   }

   if (Commands_Table[cmd].u16_validity_checks & 0x0020) // DRIVE_INACTIVE
   {
      if (!Enabled(DRIVE_PARAM)) return (DRIVE_INACTIVE);
   }

   if (Commands_Table[cmd].u16_validity_checks & 0x0010) // VALUE_TOO_HIGH
   {
      if (s64_Execution_Parameter[0] > Commands_Table[cmd].s64_max_value)
         return (VALUE_TOO_HIGH);
   }

   if (Commands_Table[cmd].u16_validity_checks & 0x0008) // VALUE_TOO_LOW
   {
      if (s64_Execution_Parameter[0] < Commands_Table[cmd].s64_min_value)
         return (VALUE_TOO_LOW);
   }

   if (Commands_Table[cmd].u16_validity_checks & 0x0004) // BURNIN_ACTIVE
   {
      if (BGVAR(s8_BurninParam) != 0) return (BURNIN_ACTIVE);
   }

   return (SAL_SUCCESS);
}


// int s16_ModSim_Debug1 = 0, s16_ModSim_Debug2 = 0, s16_ModSim_Debug3 = 0, s16_ModSim_Debug4 = 0;
// Schneider (P_Parameter/Modbus/Can Object) execution handler
//**********************************************************
// Function Name: SearchPParamIndex
// Description:
//   This function is called to search for a specific
//   p_parameter in the drive table.
//
//
// Author: S.F
// Algorithm:
// Return Value:
//   0: p_param not found in table
//   else: p_param index in table+1 & send the index in the list by ref
// Revisions:
//**********************************************************
int SearchPParamIndex(unsigned int u16_p_param, unsigned int *u16_p_param_index)
{
   int i,j;
   int Param_Idx;

   if((u16_p_param >= 1100 ) && (u16_p_param < 1200))
   {//Some P1-xx are hard coded, some in the excel. first check if hard coded.
      Param_Idx = SearchP11ParamIndex(u16_p_param-1100);
      if(Param_Idx >= 0)
      {
           *u16_p_param_index = 0xFFFF;
           return (Param_Idx+1);
      }
   }

   for (i = 0; i < COMMANDS_TABLE_SIZE; i++)
   {
      if (Commands_Table[i].u16_p_param_number == u16_p_param)
      {
         *u16_p_param_index = 0xFFFF;
         return (i + 1);
      }
      else if (Commands_Table[i].u16_p_param_list_index) // Search if part of list of p_params
      {
         for (j = 0; j < SUB_INDEX_SIZE; j++)
         {
            if (P_Param_List[(Commands_Table[i].u16_p_param_list_index) - 1][j] == u16_p_param)
            {
               *u16_p_param_index = j;
               return (i + 1);
            }
         }
      }
   }
   return 0;
}


int ExecuteModbusRead(int cmd, int drive, long long *s64_result)
{
   long long res;
   int index = -1;
   int return_value;

   return_value = ExecuteReadCommand(cmd, drive, &res);

   if (return_value == 1)
   {
      if (Commands_Table[cmd].str_units_var != 0)   // Handle units conversion
      {
         index = UnitsConversionIndex(Commands_Table[cmd].str_units_var, drive);

         if (index >= 0) // Unit conversion found
         {
            res = MultS64ByFixS64ToS64(res,
                                       BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_user_fix,
                                       BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_user_shr);
         }
      }
      s32_debug_result = (long)res;
      *s64_result   = res;
   }
   return return_value;
}


// This is for debug purposes but it a good start
//**********************************************************
// Function Name: ExecutePParamCmd
// Description: lower level of Excute for PParam (and modbus) read and write.
//
//
// Author: Udi
// Algorithm:
// Return Value: Return sal response
// Revisions:
//**********************************************************
int ExecutePParamCmd(int drive, int index, int u16_op_type, long *u32_out_value)
{
   int return_value = UNKNOWN_COMMAND;
   unsigned int u16_p_param_factor=1;
   long long s64_result = 0;

   // Detect if it is a read or write command
   if (u16_op_type == OP_TYPE_WRITE)
   {
       if(Commands_Table[index].u8_flags & 0x40)   // if decimal point is supported
       {
            u16_p_param_factor = getPParamFactor(index);
            s64_Execution_Parameter[0] *= (long long)u16_p_param_factor;
       }

      if (MAX_WRITE_PARAMS(Commands_Table[index].u16_min_max_arg) == 0)
       // If trying to write to Not_Programmable...
         return_value = NOT_PROGRAMMABLE;

      // Call the validation check function
      if (return_value != NOT_PROGRAMMABLE)
         return_value = ValidateSerialWrite(index, drive);

      if (return_value == SAL_SUCCESS)
         return_value = ExecuteSerialWrite(index, drive); // call to write handler
   }
   else if (u16_op_type == OP_TYPE_READ) // Read command
   {
      // Call the validation check function
      return_value = ValidateSerialRead(index, drive);

      if (return_value == SAL_SUCCESS)
         return_value = ExecuteModbusRead(index, drive, &s64_result); // call to read handler
      if (return_value == SAL_SUCCESS)
      {
         if(Commands_Table[index].u8_flags & 0x40)   // if decimal point is supported
         {
            u16_p_param_factor = getPParamFactor(index);
            s64_result /= u16_p_param_factor;
         }
         *u32_out_value = (long)s64_result; // PParam is always 32 bit
      }
   }

   return return_value;   //return the status
}

//**********************************************************
// Function Name: getPParamFactor
// Description: get the DIV/MUL factor for P Parameters according to the "Decimal Factor"
// coulmn in the commands excel sheet
//
// Author: Moshe
// Algorithm:
// Return Value: Return sal response
// Revisions:
//**********************************************************
unsigned int getPParamFactor(unsigned int index)
{
   unsigned int u16_factor =1;
   switch(Commands_Table[index].u16_P_Param_Decimal_Multiplier)
   { // div the p value 4 = div by 1000 ; 1 = div by 100 ; 2 = div by 10 ; 3,0 = div by 1

      case 1:// excel factor "10"
        u16_factor= 100;
      break;

      case 2:// excel factor "100"
        u16_factor= 10;
      break;

      case 3:// excel factor "1000"
      break;

      case 4:// excel factor "1"
        u16_factor= 1000;
      break;

      default:
      break;
   }
   return u16_factor;
}

//**********************************************************
// Function Name: CheckLexiumAccessRights
// Description:
//          The function is only allowed to be called out of a background task, since
//          it uses the switch-case instruction.
//          This function checks the access-rights of some specific Lexium functions
//          according to the Schneider access-rights management table described
//          in the document: "Bai_Yang_DES04_Access_Channels.doc".
//          The following things needs be be considered for some specific functions:
//             1) Which functionality needs to be checked - see u16_Lexium_Functionality
//             2) What is the type of Lexium Drive - see BGVAR(u8_Lexium_Drive_Type)
//             3) Which channel asks for P-parameter access - see s16_Access_Channel
//             4) What is the current access-rights state - see BGVAR(u16_Lexium_Acces_Rights_State),
//                which is inside of the function seperated into "s16_Temp_Exclusive_Access" and
//                "s16_Temp_Current_Access_Rights".
//
//    u16_Lexium_Functionality - Lexium functionality, which is supposed to be checked and which
//                               corresponds to a function in the table. The Lexium functionalities,
//                               which are subject of the access-rights management, are listed in the
//                               "LexiumAccessRightsFunctions" enum.
//    s16_Access_Channel - Channel that wants to check if an action is allowed to be triggered (see enum "LexiumAccessChannel")
//
//    Return value: SAL_SUCCESS = Access is allowed; LEXIUM_INVALID_ACCESS_RIGHTS = Access is NOT allowed
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int CheckLexiumAccessRights(int drive, int s16_Lexium_Functionality, int s16_Access_Channel)
{
   int s16_Access_Allowed = LEXIUM_INVALID_ACCESS_RIGHTS; // Per default access not allowed

   // Split the access-rights state from the variable "u16_Lexium_Acces_Rights_State" in "
   // exclusive-access yes/no" and "channel who has currently the access-rights" information.
   int s16_Temp_Exclusive_Access = BGVAR(u16_Lexium_Acces_Rights_State) & 0x00FF; // Get low-byte of access-right state
   int s16_Temp_Current_Access_Rights = (BGVAR(u16_Lexium_Acces_Rights_State) >> 8) & 0x00FF; // Get high-byte of access-right state

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // If not a Schneider product/hardware OR if the access-channel information is not valid.
   if (((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) || (s16_Access_Channel == LEX_CH_RESERVED))
   {
      return (SAL_SUCCESS); // Allow access and do not consider the Lexium access-rights management.
   }

   /******************************************************************************/
   /*** Here implement ONLY paths where the access is allowed depending on the ***/
   /*** function arguments and depending on the matrix given by Schneider (see ***/
   /*** document "Bai_Yang_DES04_Access_Channels.doc")                         ***/
   /******************************************************************************/

   switch (s16_Lexium_Functionality)
   {
      case(LEX_AR_FCT_ACTIVATE_PT):
         // If the digital IO wants to access the function and if the Drive type is an IO Drive and if there is no exclusive access by someone else.
         if((s16_Access_Channel == LEX_CH_IO_CONTROL) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_IO_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // If the local HMI wants to access the function and if there is exclusive access by the local HMI in case of a fieldbus Drive
         else if((s16_Access_Channel == LEX_CH_LOCAL_HMI) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_IO_DRIVE) && (s16_Temp_Current_Access_Rights == LEX_CH_LOCAL_HMI) && (s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // If the CT wants to access the function and if there is exclusive access by the commissioning tool independent of the Drive type
         else if((s16_Access_Channel == LEX_CH_MODBUS_RS485) && (s16_Temp_Current_Access_Rights == LEX_CH_MODBUS_RS485) && (s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case(LEX_AR_FCT_ACTIVATE_PS):
         // If the digital IO wants to access the function and if the Drive type is an IO Drive and if there is no exclusive access by someone else.
         if((s16_Access_Channel == LEX_CH_IO_CONTROL) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_IO_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // If the local HMI wants to access the function and if there is exclusive access by the local HMI independent of the Drive type
         else if((s16_Access_Channel == LEX_CH_LOCAL_HMI) && (s16_Temp_Current_Access_Rights == LEX_CH_LOCAL_HMI) && (s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // If the CT wants to access the function and if there is exclusive access by the commissioning tool independent of the Drive type
         else if((s16_Access_Channel == LEX_CH_MODBUS_RS485) && (s16_Temp_Current_Access_Rights == LEX_CH_MODBUS_RS485) && (s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // If the fieldbus wants to access the function and if the Drive type is an fieldbus Drive and if there is no exclusive access by someone else.
         else if((s16_Access_Channel == LEX_CH_FIELDBUS) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_FIELDBUS_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case(LEX_AR_FCT_ACTIVATE_S):
      case(LEX_AR_FCT_ACTIVATE_T):
      case(LEX_AR_FCT_ACTIVATE_SZ):
      case(LEX_AR_FCT_ACTIVATE_TZ):
         // If the digital IO wants to access the function and if the Drive type is an IO Drive and if there is no exclusive access by someone else.
         if((s16_Access_Channel == LEX_CH_IO_CONTROL) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_IO_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // If there is exclusive access by the commissioning tool independent of the Drive type
         else if((s16_Access_Channel == LEX_CH_MODBUS_RS485) && (s16_Temp_Current_Access_Rights == LEX_CH_MODBUS_RS485) && (s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case(LEX_AR_FCT_ACTIVATE_JOG_MODE):
         // If the local HMI wants to activate this function and if the access-rights are obtained by the lHMI OR
         // If the commissioning tool (CT) wants to activate this function and if the access-rights are obtained
         // by the CT, independent of the Drive type.
         if( ((s16_Access_Channel == LEX_CH_LOCAL_HMI) && (s16_Temp_Current_Access_Rights == LEX_CH_LOCAL_HMI)) ||
             ((s16_Access_Channel == LEX_CH_MODBUS_RS485) && (s16_Temp_Current_Access_Rights == LEX_CH_MODBUS_RS485)) )
         {
            if(s16_Temp_Exclusive_Access)
            {
               s16_Access_Allowed = SAL_SUCCESS; // Allow access
            }
         }
         // If the fieldbus wants to access the function and if the Drive type is an fieldbus Drive and if there is no exclusive access by someone else.
         else if((s16_Access_Channel == LEX_CH_FIELDBUS) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_FIELDBUS_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // If the digital IOs wants to access the function and if the Drive type is an IO Drive and if there is no exclusive access by someone else.
         else if((s16_Access_Channel == LEX_CH_IO_CONTROL) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_IO_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case (LEX_AR_FCT_ACTIVATE_GEARING_MANU):
         // For both Drive types, if the CT has exclusive access-rights
         if((s16_Access_Channel == LEX_CH_MODBUS_RS485) && (s16_Temp_Current_Access_Rights == LEX_CH_MODBUS_RS485))
         {
            if(s16_Temp_Exclusive_Access)
            {
               s16_Access_Allowed = SAL_SUCCESS; // Allow access
            }
         }
         // If the lHMI wants to activate manufacturer specific gearing for a filedbus Drive and if the lHMI is in possession of exclusive access-rights
         else if ((s16_Access_Channel == LEX_CH_LOCAL_HMI) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_FIELDBUS_DRIVE) && (s16_Temp_Current_Access_Rights == LEX_CH_LOCAL_HMI))
         {
            if(s16_Temp_Exclusive_Access)
            {
               s16_Access_Allowed = SAL_SUCCESS; // Allow access
            }
         }
         // If the fieldbus wants to access the function and if the Drive type is an fieldbus Drive and if there is no exclusive access by someone else.
         else if((s16_Access_Channel == LEX_CH_FIELDBUS) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_FIELDBUS_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case(LEX_AR_FCT_ACTIVATE_DS402_PROF_POS):
      case(LEX_AR_FCT_ACTIVATE_DS402_PROF_VEL):
      case(LEX_AR_FCT_ACTIVATE_DS402_PROF_TRQ):
      case(LEX_AR_FCT_ACTIVATE_DS402_HOMING):
      case(LEX_AR_FCT_ACTIVATE_DS402_IP_MODE):
      case(LEX_AR_FCT_ACTIVATE_DS402_CSP_MODE):
         // If there is exclusive access by the commissioning tool independent of the Drive type
         if((s16_Access_Channel == LEX_CH_MODBUS_RS485) && (s16_Temp_Current_Access_Rights == LEX_CH_MODBUS_RS485) && (s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // If the fieldbus wants to access the function and if the Drive type is an fieldbus Drive and if there is no exclusive access by someone else.
         else if((s16_Access_Channel == LEX_CH_FIELDBUS) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_FIELDBUS_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case(LEX_AR_FCT_ACTIVATE_AUTO_TUNING):
         // If the local HMI wants to activate this function and if the access-rights are obtained by the lHMI OR
         // If the commissioning tool (CT) wants to activate this function and if the access-rights are obtained by the CT.
         if( ((s16_Access_Channel == LEX_CH_LOCAL_HMI) && (s16_Temp_Current_Access_Rights == LEX_CH_LOCAL_HMI)) ||
             ((s16_Access_Channel == LEX_CH_MODBUS_RS485) && (s16_Temp_Current_Access_Rights == LEX_CH_MODBUS_RS485)) )
         {
            if(s16_Temp_Exclusive_Access)
            {
               s16_Access_Allowed = SAL_SUCCESS; // Allow access
            }
         }
         // If the fieldbus wants to access the function and if the Drive type is an fieldbus Drive and if there is no exclusive access by someone else.
         else if((s16_Access_Channel == LEX_CH_FIELDBUS) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_FIELDBUS_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case(LEX_AR_FCT_READ_STAT_INFO):
         if(s16_Access_Channel != LEX_CH_IO_CONTROL)
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case(LEX_AR_FCT_ENABLE):
         if((s16_Access_Channel == LEX_CH_IO_CONTROL) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_IO_DRIVE))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // Also valid for both Drive-types (IO and fieldbus).
         else if( ((s16_Access_Channel == LEX_CH_LOCAL_HMI) && (s16_Temp_Current_Access_Rights == LEX_CH_LOCAL_HMI)) ||
                  ((s16_Access_Channel == LEX_CH_MODBUS_RS485) && (s16_Temp_Current_Access_Rights == LEX_CH_MODBUS_RS485)) )
         {
            if(s16_Temp_Exclusive_Access)
            {
               s16_Access_Allowed = SAL_SUCCESS; // Allow access
            }
         }
         // Valid only for Fieldbus Drive when no one else has exclusive access-rights
         else if ((s16_Access_Channel == LEX_CH_FIELDBUS) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_FIELDBUS_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case(LEX_AR_FCT_FLT_RESET):
      case(LEX_AR_FCT_CONTINUE):
         // Valid for both Drive-types (IO and fieldbus). Always possible via digital IO.
         if(s16_Access_Channel == LEX_CH_IO_CONTROL)
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // Also valid for both Drive-types (IO and fieldbus).
         else if( ((s16_Access_Channel == LEX_CH_LOCAL_HMI) && (s16_Temp_Current_Access_Rights == LEX_CH_LOCAL_HMI)) ||
                  ((s16_Access_Channel == LEX_CH_MODBUS_RS485) && (s16_Temp_Current_Access_Rights == LEX_CH_MODBUS_RS485)) )
         {
            if(s16_Temp_Exclusive_Access)
            {
               s16_Access_Allowed = SAL_SUCCESS; // Allow access
            }
         }
         else if((s16_Access_Channel == LEX_CH_FIELDBUS) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_FIELDBUS_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case(LEX_AR_FCT_DISABLE):
         // Valid only for IO Drive
         if((s16_Access_Channel == LEX_CH_IO_CONTROL) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_IO_DRIVE))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // Valid for both Drive-types (IO and fieldbus) without even getting exclusive access-rights.
         else if((s16_Access_Channel == LEX_CH_LOCAL_HMI) || (s16_Access_Channel == LEX_CH_MODBUS_RS485))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         // Valid only for Fieldbus Drive when no one else has exclusive access-rights
         else if ((s16_Access_Channel == LEX_CH_FIELDBUS) && (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_FIELDBUS_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
            s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case(LEX_AR_FCT_HALT):
      case(LEX_AR_FCT_QUICK_STOP):
         // Valid for both Drive-types (IO and fieldbus). If IO, localHMI or CT wants to perform this feature
         if((s16_Access_Channel == LEX_CH_IO_CONTROL) || (s16_Access_Channel == LEX_CH_LOCAL_HMI) || (s16_Access_Channel == LEX_CH_MODBUS_RS485))
         {
             s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
         else if ((BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_FIELDBUS_DRIVE) && (!s16_Temp_Exclusive_Access))
         {
             s16_Access_Allowed = SAL_SUCCESS; // Allow access
         }
      break;

      case(LEX_AR_FCT_SET_MOTION_PARAM):
      default:                   // Function, which is not mentioned in the matrix. Allow access.
         s16_Access_Allowed = SAL_SUCCESS;
      break;
   }

   return(s16_Access_Allowed);
}

//**********************************************************
// Function Name: GetLexiumAccessRightsFunctionNumber
// Description:
//          This function converts a P-parameter number into a so-called functionality
//          number. The document "Bai_Yang_DES04_Access_Channels.doc" holds certain
//          functionalities, which are subject of the access-rights management. So if
//          there is a P-parameter connected to one of the functionalities inside of the
//          document, then this P-parameter needs to be converted into a certain functionality
//          number.
//
//    u16_P_Parameter_Number - Lexium P-parameter
//
//    Return value: Functionality number, which is connected to the Lexium P-parameter
//                  (see enum "LexiumAccessRightsFunctions").
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int GetLexiumAccessRightsFunctionNumber(unsigned int u16_P_Parameter_Number)
{
   int s16_Lexium_Functionality_Number = LEX_AR_FCT_RESERVED;

   switch (u16_P_Parameter_Number)
   {
      case (1): // P0-01 Clear errors upon writing to P0-01
         s16_Lexium_Functionality_Number = LEX_AR_FCT_FLT_RESET;
      break;

      case (232): // P2-32 AUTO TUNE MODE (e.g. via local HMI enter autotune mode when writing to P2-32)
         s16_Lexium_Functionality_Number = LEX_AR_FCT_ACTIVATE_AUTO_TUNING;
      break;

      case (1025): // P10-25: Triggers a JOG MOVE (via commissioning tool)
            s16_Lexium_Functionality_Number = LEX_AR_FCT_ACTIVATE_JOG_MODE;
      break;
   }

   return(s16_Lexium_Functionality_Number);
}


// This is for debug purposes but it a good start
//**********************************************************
// Function Name: ExecutePParam
// Description: Top level of Excute for PParam (and modbus) read and write.
//
//    u16_pparam_number - Number of the P-parameter (e.g. 109 for P1-09).
//    u16_op_type - Either OP_TYPE_WRITE for write or OP_TYPE_READ for read.
//    *u32_value - Pointer to the 32-bit value either for read or for write access.
//    s16_Access_Channel - Indicates who calls the function for the access-rights
//                         management (must be an enum member of "LexiumAccessChannel")
//
// Author: Udi
// Algorithm:
// Return Value: Return sal response
// Revisions:
//**********************************************************
int ExecutePParam(int drive, unsigned int u16_pparam_number, int s16_op_type, long *s32_value, int s16_Access_Channel)
{
   int return_value = UNKNOWN_COMMAND;
   unsigned int u16_cmd_index, u16_param_index, u8_flags_local;
   int s16_access_rights_permission = SAL_SUCCESS;

   // Here do special handling for hidden P11 shadow parameters.
   // If there is a read-access to any of the P11 shadow parameters
   if((u16_pparam_number >= 1100) && (u16_pparam_number < 1200) && (s16_op_type == OP_TYPE_READ))
   {
      *s32_value = GetDriveStatusValue(drive, u16_pparam_number-1100);
      return (SAL_SUCCESS);
   }


   u16_cmd_index = SearchPParamIndex(u16_pparam_number, &u16_param_index);
   u8_flags_local = Commands_Table[u16_cmd_index-1].u8_flags;

   if (u16_cmd_index) // If cmd found
   {
      STORE_EXECUTION_PARAMS_0_1;

      if (s16_op_type == OP_TYPE_WRITE)
      {
         s16_Number_Of_Parameters = 1;
         if (UNSIGNED_PARAM(u8_flags_local))
            s64_Execution_Parameter[0] = (unsigned long long)(unsigned long)(*s32_value);
         else
            s64_Execution_Parameter[0] = (long long)(*s32_value);

         if (u16_param_index != 0xFFFF)
         {
            s16_Number_Of_Parameters = 2;
            s64_Execution_Parameter[0] = (long long)(u16_param_index);
            //s64_Execution_Parameter[1] = (long long)(*u32_value);

            if (UNSIGNED_PARAM(u8_flags_local))
               s64_Execution_Parameter[1] = (unsigned long long)(unsigned long)(*s32_value);
            else
               s64_Execution_Parameter[1] = (long long)(*s32_value);
         }
      }
      else if (s16_op_type == OP_TYPE_READ) // Read command
      {
         s16_Number_Of_Parameters = 0;
         if (u16_param_index != 0xFFFF)
         {
            s16_Number_Of_Parameters = 1;
            s64_Execution_Parameter[0] = (long long)(u16_param_index);
         }
      }

      // Check access-rights upon write request since certain actions (like e.g. clear faults)
      // are usually handled only upon a write request to a P-parameter.
      if(s16_op_type == OP_TYPE_WRITE)
      {
         // Check if the access-rights for a certain P-parameter are given
         s16_access_rights_permission = CheckLexiumAccessRights(drive, GetLexiumAccessRightsFunctionNumber(u16_pparam_number), s16_Access_Channel);
      }
      if (s16_access_rights_permission != SAL_SUCCESS)
      {

          RESTORE_EXECUTION_PARAMS_0_1;

         // This P-parameter has currently no access-rights. Reject the command.
         return (s16_access_rights_permission);
      }

      // Indicate globally which access-channel asks specifically for a P-parameter access
      BGVAR(s16_Lexium_ExecPParam_Acces_Channel) = s16_Access_Channel;

      return_value = ExecutePParamCmd(drive, u16_cmd_index - 1, s16_op_type, &(*s32_value));

      // Clear the global indication about which access-channel asks specifically for a P-parameter access
      BGVAR(s16_Lexium_ExecPParam_Acces_Channel) = LEX_CH_RESERVED;

      RESTORE_EXECUTION_PARAMS_0_1;
   }

   return return_value;   //return the status
}


int SalDelayCommand(long long param,int drive)
{
   static int s16_delay_state = 0;
   static long s32_delat_time_stamp;
   int ret_val = SAL_NOT_FINISHED;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch (s16_delay_state)
   {
      case 0:
         s32_delat_time_stamp = Cntr_1mS;
         BGVAR(u16_DelayTime) = (int)param;
         s16_delay_state = 1;
      break;

      case 1:
         if (PassedTimeMS((long)BGVAR(u16_DelayTime),s32_delat_time_stamp))
         {
            s16_delay_state = 0;
            ret_val = SAL_SUCCESS;
         }
      break;
   }

   return ret_val;
}
