#include "ser_comm.def"
#include "Err_Hndl.def"
#include "Position.def"

#include "Drive_Table.var"
#include "Ser_Comm.var"
#include "init.var"
#include "Exe_IO.def"
#include "Exe_IO.var"
#include "Drive.var"

#include "Prototypes.pro"
#include "Design.def"

//**********************************************************
// Function Name: PrintMnemonic
// Description:
//   This function prints mnemonic from mnemonic table
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int PrintMnemonic(int index)
{
   unsigned long mnem_code1,mnem_code2,mnem_code3;
   char mnem_let;
   int i;

   mnem_code1  = Commands_Table[index].u32_mnemonic_code1;
   mnem_code2  = Commands_Table[index].u32_mnemonic_code2;
   mnem_code3  = Commands_Table[index].u32_mnemonic_code3;

   // If a P_Parameter with no ascii equivalent (P_x) dont print it
   if ((mnem_code1 == 0xFFFFFFFF) && (mnem_code2 == 0xFFFFFFFF) && (mnem_code3 == 0xFFFFFFFF))
      return 0;

   for (i=4; i>=0; i--)
   {
      mnem_let = ((mnem_code1 >> (i*6L)) & 0x0000003F);
      if (mnem_let != 0) PrintChar(mnem_let + 47);
   }
   for (i=4; i>=0; i--)
   {
      mnem_let = ((mnem_code2 >> (i*6L)) & 0x0000003F);
      if (mnem_let != 0) PrintChar(mnem_let + 47);
   }
   for (i=4; i>=0; i--)
   {
      mnem_let = ((mnem_code3 >> (i*6L)) & 0x0000003F);
      if (mnem_let != 0) PrintChar(mnem_let + 47);
   }

   return 1;
}

//**********************************************************
// Function Name: ListCommand
// Description:
//   This function is called in response List command
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int ListCommand(void)
{
   static int i = 0;

   while (i< COMMANDS_TABLE_SIZE)
   {
      // If there are not enough free space at output buffer , come back next background
      if (u8_Output_Buffer_Free_Space < 20) return SAL_NOT_FINISHED;

      // Print mnemonic if not password protected
      if (((Commands_Table[i].u16_validity_checks & 0x0001) == 0) || s16_Password_Flag)
      {
         if (PrintMnemonic(i)) PrintCrLf();
      }

      //increment pointer to mnemonic table
      i++;
   }

   // Set pointer to the beginning of mnemonic table
   i = 0;

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: DumpCommand
// Description:
//   This function is called in response to DUMP command
//
//  Will print the commands according to their dump priority
//   then according to their alphabetical order
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int DumpCommand(int drive)
{
   static int dump_state = 0;
   static int highest_priority = 0;
   static int next_highest_priority = 0;
   static int index = 0;
   static int special_index = 0,edge_pol=0;
   static int io_index = 0;
   static int path_index = 0;
   static int script_index = 0;
   static int display_mlist = 0;
   static int special_dump = 0;
   static int nldg_index = 0;
   static int insert_CONFIG_0 = 0;
   static int s16_something_printed_on_last_priority = 0;
   static int s16_repeat_print_value = 0; // This variable is responsible for repeating "PrintNextDumpVar"
                                          // until the function returns a value unequal SAL_NOT_FINIHED.
   int i, ret_val;

   if (display_mlist)
   {
      ret_val = MotorListCommand(drive);
      if (ret_val == SAL_SUCCESS) display_mlist = 0;
      return ret_val;
   }

   // Check if "DUMP 1" is used which means MLIST display
   if (dump_state == 0)
   {
      if ((s16_Number_Of_Parameters == 1) && (s64_Execution_Parameter[0] == 1))
      {
         display_mlist = 1;
         return SAL_NOT_FINISHED;
      }
      
      // Check if "DUMP 2" was issued (non-applicaton related mnemonics should be displayed)
      if ((s16_Number_Of_Parameters == 1) && (s64_Execution_Parameter[0] == 2)) special_dump = 1;
   }

   switch (dump_state)
   {
      case 0: // Search highest priority
         for (i=0; i<COMMANDS_TABLE_SIZE; i++)
            // Dont show Modbus Dump only parameters
            if ((Commands_Table[i].u16_dump_group > highest_priority) && (Commands_Table[i].u16_dump_group != 5))
               highest_priority = Commands_Table[i].u16_dump_group;

         index = 0;
         next_highest_priority = 0;
         dump_state++;

         if (highest_priority == 0)
         {
            dump_state = 15;
            special_index = 0;
         }
          
         // This will indicate if a mnemonic was printed on the last priority group and hence a "CONFIG 0" should be printed or not         
         s16_something_printed_on_last_priority = 1 - special_dump;
         
         //mark that dump command is active
         u8_is_dump_command_active = 1;
      break;

      case 1: // Print only the params with specified priority level + calculate next priority level to be printed
         // Here we search for the index of the next command to print for the current "highest_priority" level
         // and in addition search for the "next_highest_priority" level.
         while ((Commands_Table[index].u16_dump_group != highest_priority) && (index < COMMANDS_TABLE_SIZE))
         {
            // Dont show Modbus Dump only parameters
            if ((Commands_Table[index].u16_dump_group > next_highest_priority) && (Commands_Table[index].u16_dump_group < highest_priority) && (Commands_Table[index].u16_dump_group!=5))
               next_highest_priority = Commands_Table[index].u16_dump_group;
            index++;
         }

         if (index >= COMMANDS_TABLE_SIZE)
         {
            highest_priority = next_highest_priority;
            next_highest_priority = 0;
            index = 0;
            if (highest_priority <= 1) // End of dump
            {
               dump_state = 3;
               special_index = 0;
            }
            else
            {
               if (s16_something_printed_on_last_priority)
                  insert_CONFIG_0 = 1;                  
            }
            s16_something_printed_on_last_priority = 0;
         }
         else
         {
            if ((Commands_Table[index].u16_dump_group > next_highest_priority) && (Commands_Table[index].u16_dump_group < highest_priority) && (Commands_Table[index].u16_dump_group!=5))
               next_highest_priority = Commands_Table[index].u16_dump_group;
            dump_state++;
         }
      break;

      case 2: // Print the command from a DUMP group
         if (u8_Output_Buffer_Free_Space < 50) return SAL_NOT_FINISHED;

         if (insert_CONFIG_0)
         {
            insert_CONFIG_0 = 0;
            // Here print CONFIG due to the Bugzilla 3952 (Labman issue)
            PrintStringCrLf("CONFIG 0",0);
         }
         else
         {
            // print command only if not password protected
            if (((Commands_Table[index].u16_validity_checks & 0x0001) == 0) || s16_Password_Flag)
            {
               if (Commands_Table[index].u16_dump_general == 0) s16_something_printed_on_last_priority = 1;
               
               if(PrintNextDumpVar(index,drive,0, special_dump) != SAL_NOT_FINISHED)
               {
                  // Go to next command
                  index++;
                  dump_state = 1;
               }
            }
            else
            {
               // Go to next command
               index++;
               dump_state = 1;
            }
         }
      break;

      case 3: // Handle special cases: INXMODE, INXINV, OXMODE, OXINV and PATHPOS, PATHACC, PATHDEC, PATHDLY, PATHSPD, PATHCTRL
         index = COMMANDS_TABLE_SIZE;
         io_index = 0;
         special_index++;

         if (special_index < 50)
         {
            // Here determine the new dump_state via special_index, starting from dump_state=4 upwards
            switch (special_index)
            {
               case 1: // INMODE
                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                     if (Commands_Table[i].ptr_read_access == &SalReadInModeCommand) index = i;
               break;

               case 2: // ININV
                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                     if (Commands_Table[i].ptr_read_access == &SalReadInPolarCommand) index = i;
               break;

               case 3: // OUTMODE
                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                     if (Commands_Table[i].ptr_read_access == &SalReadOutModeCommand) index = i;
               break;

               case 4: // OUTINV
                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                     if (Commands_Table[i].ptr_read_access == &SalReadOutPolarCommand) index = i;
               break;

               case 5: // DRIVESCRIPT
                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                     if (Commands_Table[i].ptr_read_access == &SalReadScript) index = i;
                     script_index=0;
               break;

               case 6: // PATH POS
                     for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                        if (Commands_Table[i].ptr_read_access == &SalReadPathPositionCommand) index = i;
                   path_index=0;
               break;

               case 7: // PATH ACC
                     for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                        if (Commands_Table[i].ptr_read_access == &SalReadPathAccelerationCommand) index = i;
                   path_index=0;
               break;

               case 8: // PATH DEC
                     for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                        if (Commands_Table[i].ptr_read_access == &SalReadPathDecelerationCommand) index = i;
                   path_index=0;
               break;

               case 9: // PATH SPD
                     for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                        if (Commands_Table[i].ptr_read_access == &SalReadPathSpeedCommand) index = i;
                   path_index=0;
               break;

               case 10: // PATH DLY
                     for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                        if (Commands_Table[i].ptr_read_access == &SalReadPathDelayCommand) index = i;
                   path_index=0;
               break;

               case 11: // PATH CTRL
                     for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                        if (Commands_Table[i].ptr_read_access == &SalReadPathControlCommand) index = i;
                   path_index=0;
               break;

               case 12: // NLFILTDAMPING
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadFiltDampingGTCommand) index = i;
                  nldg_index = 1;
                  break;
               case 13: // NLFILTT1DG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadFiltt1GTCommand) index = i;
                  nldg_index = 1;
                  break;

               case 14: // KNLDDG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadKnldGTCommand) index = i;
                  nldg_index = 1;
                  break;

               case 15: // KNLIDG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadKnliGTCommand) index = i;
                  nldg_index = 1;
               break;

               case 16: // KNLUSERGAINDG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadKnlUserGainGTCommand) index = i;
                  nldg_index = 1;
                  break;

               case 17: // KNLIVDG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadKnlivGTCommand) index = i;
                  nldg_index = 1;
               break;

               case 18: // KNLPDG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadKnlpGTCommand) index = i;
                  nldg_index = 1;
               break;

               case 19: // LMJRGT
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadLmjrGTCommand) index = i;
                  nldg_index = 1;
               break;

               case 20: // NLANTIVIBHZDG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadAVibHzGTCommand) index = i;
                  nldg_index = 1;
               break;

               case 21: // NLANTIVIBHZ2DG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadAVibHz2GTCommand) index = i;
                  nldg_index = 1;
               break;

               case 22: // NLANTIVIBSHARPDG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadAVibSharpGTCommand) index = i;
                  nldg_index = 1;
               break;

               case 23: // NLANTIVIBSHARP2DG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadAVibSharp2GTCommand) index = i;
                  nldg_index = 1;
               break;

               case 24: // NLANTIVIBGAINDG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadAVibGainGTCommand) index = i;
                  nldg_index = 1;
               break;

               case 25: // NLANTIVIBGAIN2DG
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadAVibGain2GTCommand) index = i;
                  nldg_index = 1;
               break;

               case 26: // NLPEAFFGT
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadNlPeAffGTCommand) index = i;
                  nldg_index = 1;
               break;

               case 27: // NLAFFLPFHZGT
//                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
//                     if (Commands_Table[i].ptr_read_access == &SalReadAffLpfHzGTCommand) index = i;
                  nldg_index = 1;
               break;

               case 28: // PROTARY
                  for (i=0; i<COMMANDS_TABLE_SIZE; i++)
                     if (Commands_Table[i].ptr_read_access == &SalReadPositionModuloCommand) index = i;
                  nldg_index = 1;
               break;
               default:
                  special_index = 50;
                  break;
            }
            if (index < COMMANDS_TABLE_SIZE) dump_state = 3 + special_index;
         }
         else // special_index is >= 50
         {
            index = 0;
            dump_state = 99;
         }
      break;

      case 4: // INXMODE
      case 5: // INXINV
         if (u8_Output_Buffer_Free_Space < 50) return SAL_NOT_FINISHED;

         // If printing the value was not finalized in the previous stage
         if(s16_repeat_print_value == 1)
         {
            if(PrintNextDumpVar(index,drive,io_index, special_dump) != SAL_NOT_FINISHED)
            {
               s16_repeat_print_value = 0;
            }
         }
         else
         {
            // If input supported print it
            io_index++;
            if (io_index > s16_Num_Of_Inputs) dump_state = 3;
            else if (u16_Supported_Dig_Inputs_Mask & (1<<(int)(io_index-1)))
            {
               if (((Commands_Table[index].u16_validity_checks & 0x0001) == 0) || s16_Password_Flag)
               {
                  if(PrintNextDumpVar(index,drive,io_index, special_dump) != SAL_NOT_FINISHED)
                  {
                     s16_repeat_print_value = 0;
                  }
                  else
                  {
                     s16_repeat_print_value = 1; // Repeat the current "PrintNextDumpVar" call until finished.
                  }
               }
            }
         }
      break;

      case 6: // OXMODE
      case 7: // OXINV
         if (u8_Output_Buffer_Free_Space < 50) return SAL_NOT_FINISHED;

         // If printing the value was not finalized in the previous stage
         if(s16_repeat_print_value == 1)
         {
            if(PrintNextDumpVar(index,drive,io_index, special_dump) != SAL_NOT_FINISHED)
            {
               s16_repeat_print_value = 0;
            }
         }
         else
         {
            // If output supported print it
            io_index++;
            if (io_index > s16_Num_Of_Outputs) dump_state = 3;
            else if (u16_Supported_Dig_Outputs_Mask & (1<<(int)(io_index-1)))
            {
               if (((Commands_Table[index].u16_validity_checks & 0x0001) == 0) || s16_Password_Flag)
               {
                  if(PrintNextDumpVar(index,drive,io_index, special_dump) != SAL_NOT_FINISHED)
                  {
                     s16_repeat_print_value = 0;
                  }
                  else
                  {
                     s16_repeat_print_value = 1; // Repeat the current "PrintNextDumpVar" call until finished.
                  }
               }
            }
         }
      break;

      case 8: //DRIVESCRIPT
         if (u8_Output_Buffer_Free_Space < 100) return SAL_NOT_FINISHED;

         if (script_index > 31)
         {
            dump_state = 3;
            script_index=0;
            edge_pol = 0;
         }
         else
         {
            if (((Commands_Table[index].u16_validity_checks & 0x0001) == 0) || s16_Password_Flag)
            {
               PrintNextDumpString(index,drive,script_index,edge_pol);

               if (edge_pol)
               {
                  edge_pol=0;
                  script_index++;
               }
               else
                  edge_pol++;
            }
         }
         break;

     case 9: // PATH POS
     case 10: // PATH ACC
     case 11: // PATH DEC
     case 12: // PATH SPD
     case 13: // PATH DLY
     case 14: // PATH CTRL
         if (u8_Output_Buffer_Free_Space < 50) return SAL_NOT_FINISHED;
         if (path_index > 31)
         {
            dump_state = 3;
            path_index=0;
         }
         else
        {

            if (((Commands_Table[index].u16_validity_checks & 0x0001) == 0) || s16_Password_Flag)
                  PrintNextPathVar(index,drive,path_index);
            path_index++;
         }
         break;
      case 15: // NLFILTDAMPINGDG
      case 16: // NLFILTT1DG
      case 17: // KNLDDG
      case 18: // KNLIDG
      case 19: // KNLUSERGAINDG
      case 20: // KNLIVDG
      case 21: // KNLPDG
      case 22: // LMJRGT
      case 23: // NLANTIVIBHZDG
      case 24: // NLANTIVIBHZ2DG
      case 25: // NLANTIVIBSHARPDG
      case 26: // NLANTIVIBSHARP2DG
      case 27: // NLANTIVIBGAINDG
      case 28: // NLANTIVIBGAIN2DG
      case 29: // NLPEAFF
      case 30: // NLAFFLPFHZ
         if (u8_Output_Buffer_Free_Space < 50) return SAL_NOT_FINISHED;
         if(PrintNextDumpVar(index,drive,nldg_index, special_dump) != SAL_NOT_FINISHED)
         {
            nldg_index++;
            if (nldg_index > NUM_OF_GT) dump_state = 3;
         }
         break;
      case 31: // PROTARY
         if (u8_Output_Buffer_Free_Space < 50) return SAL_NOT_FINISHED;
         if (PrintNextDumpVar(index,drive,nldg_index, special_dump) != SAL_NOT_FINISHED)
         {
            nldg_index++;
            if (nldg_index > 2) dump_state = 3;
         }
         break;

      case 99: // Second last stage, print lowest priority commands
         if (u8_Output_Buffer_Free_Space < 50) return SAL_NOT_FINISHED;

         if (index < COMMANDS_TABLE_SIZE)
         {
            // print lowest priority commands (CONFIG) so they will appear at the end
            if (Commands_Table[index].u16_dump_group == 1)
            {
               if (((Commands_Table[index].u16_validity_checks & 0x0001) == 0) || s16_Password_Flag)
               {
                  if(PrintNextDumpVar(index,drive,0, special_dump) == SAL_NOT_FINISHED)
                  {
                     index--; // Undo the upcoming index++ instruction because we need to repeat the command
                  }
               }
            }
            index++;
         }
         else // End of table reached
         {
            PrintCrLf();
            dump_state = 100;
            index = COMMANDS_TABLE_SIZE;
         }
      break;

      // stop the dump command
      case 100:
         PrintStringCrLf("CLEARFAULTS",0); // Insert CLEARFAULTS at the very end of the dump file
         dump_state = 0;
         special_index = 0;
         highest_priority = 0;
         next_highest_priority = 0;
         index = 0;
         insert_CONFIG_0 = 0;
      break;
   }

   if (dump_state == 0)
   {
      //mark that dump command is not active
      u8_is_dump_command_active = 0;
      special_dump = 0;
      return SAL_SUCCESS;
   }
   else
      return SAL_NOT_FINISHED;
}

//**********************************************************
// Function Name: PrintNextDumpVar
// Description:
//   This function writes the command name and its value during a dump.
//   s16_special_case is used to determine if to actually print the vlaue
// Author: Yuval / APH
// Algorithm:
// Revisions:
//**********************************************************
int PrintNextDumpVar(int index,int drive, unsigned int array_index, int s16_special_case)
{
   static unsigned int s16_repeat_print_value = 0;
   int s16_ret_val = SAL_SUCCESS;

   if (s16_special_case && (Commands_Table[index].u16_dump_general)) return SAL_SUCCESS;
   
   if(s16_repeat_print_value == 0)
   {
      
      //The OVTHRESH should not be dumped if a default customer ID is used S.C
      if ( (DEFAULT_CUSTOMER_ID == u16_Customer_Id) && (0x209E5663 == Commands_Table[index].u32_mnemonic_code1) && (0x16919 == Commands_Table[index].u32_mnemonic_code2) )
      {
         return s16_ret_val;
      }
      // Print  mnemonic
      if (PrintMnemonic(index))
      {
         STORE_EXECUTION_PARAM_0
         PrintChar(SPACE);

         if (array_index)
         {
            PrintUnsignedInt16(array_index);
            PrintChar(SPACE);
            s64_Execution_Parameter[0] = array_index;
            s16_Number_Of_Parameters = 1;
         }
         else
         {
            s16_Number_Of_Parameters = 0;
         }

         //Execute function to query parameter value.
         s16_ret_val = ExecuteSerialRead(index,drive);
         RESTORE_EXECUTION_PARAM_0
      }
   }
   else
   {
      // Previous print was not finalized, so repeat the action
      STORE_EXECUTION_PARAM_0
      if (array_index)
      {
         s64_Execution_Parameter[0] = array_index;
         s16_Number_Of_Parameters = 1;
      }
      else
      {
         s16_Number_Of_Parameters = 0;
      }

      //Execute function to query parameter value.
      s16_ret_val = ExecuteSerialRead(index,drive);

      RESTORE_EXECUTION_PARAM_0
   }

   if (s16_ret_val == SAL_NOT_FINISHED)
   {
      s16_repeat_print_value = 1;
   }
   else
   {
      s16_repeat_print_value = 0;
   }

   return s16_ret_val;
}

//**********************************************************
// Function Name: PrintNextPathVar
// Description:
//   Bassed on Yuval's PrintNextDumpVar but handles also index number 0
//   This function determines which is the next variable to dump
//   if none - then return -1. if found prints it
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void PrintNextPathVar(int index,int drive, unsigned int array_index)
{
   // Print  mnemonic
   if (PrintMnemonic(index))
   {
      PrintChar(SPACE);
      PrintUnsignedInt16(array_index);
      PrintChar(SPACE);
      STORE_EXECUTION_PARAM_0
      s64_Execution_Parameter[0] = array_index;
      s16_Number_Of_Parameters = 1;
      //Execute function to query parameter value.
      ExecuteSerialRead(index,drive);
      RESTORE_EXECUTION_PARAM_0
   }
}

//**********************************************************
// Function Name: PrintNextDumpString
// Description:
//   This function prints the script vector
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
void PrintNextDumpString(int index,int drive, unsigned int array_index,unsigned int Edge_Pol)
{
   // Print  mnemonic
   PrintMnemonic(index);

   PrintChar(SPACE);
   PrintUnsignedInt16(array_index);

   PrintChar(SPACE);
   PrintUnsignedInt16(Edge_Pol);
   PrintChar(SPACE);
   STORE_EXECUTION_PARAMS_0_1
   s64_Execution_Parameter[0] = array_index;
   s64_Execution_Parameter[1] = Edge_Pol;
   s16_Number_Of_Parameters = 2;
   //Execute function to query parameter value.
   ExecuteSerialRead(index,drive);
   RESTORE_EXECUTION_PARAMS_0_1
}

//**********************************************************
// Function Name: MotorListCommand
// Description:
//   This function is called in response to MLIST command
//
//  Will print the commands according to their mlist priority
//   then according to their alphabetical order
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int MotorListCommand(int drive)
{
   static int mlist_state = 0;
   static int highest_priority = 0;
   static int index = 0;
   unsigned int u16_dump_group;
   int i;

   switch (mlist_state)
   {
      case 0: // Search highest priority
         for (i=0; i<COMMANDS_TABLE_SIZE; i++)
         {
            u16_dump_group = Commands_Table[i].u16_dump_group;
            if ((u16_dump_group >= 40) && (u16_dump_group < 60) && (u16_dump_group > highest_priority)) highest_priority = u16_dump_group;
         }

         index = 0;
         mlist_state++;

         if (highest_priority == 0) mlist_state = 4;
      break;

      case 1:
         while ((Commands_Table[index].u16_dump_group != highest_priority) && (index < COMMANDS_TABLE_SIZE))
            index++;

         if (index >= COMMANDS_TABLE_SIZE)
         {
            highest_priority--;
            index = 0;
            if (highest_priority < 40) mlist_state = 4; // End of dump
         }
         else mlist_state++;
      break;

      case 2:
         if (u8_Output_Buffer_Free_Space < 50) return SAL_NOT_FINISHED;
         if(PrintNextDumpVar(index,drive,0, 0) != SAL_NOT_FINISHED)
         {
            // Go to next stage
            index++;
            mlist_state = 1;
         }
      break;

      case 4:
         mlist_state = 0;
         highest_priority = 0;
         index = 0;
      break;
   }

   if (mlist_state == 0)
      return SAL_SUCCESS;
   else
      return SAL_NOT_FINISHED;
}

