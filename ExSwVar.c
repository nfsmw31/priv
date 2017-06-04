#include <string.h>
#include <DSP2834x_EPwm_defines.h>
#include "DSP2834x_Device.h"


#include "co_util.h"
#include "co_type.h"
#include "access.h"
#include "objects.h"

#include "Current.def"
#include "Design.def"
#include "Display.def"
#include "Endat.def"
#include "Err_Hndl.def"
#include "FltCntrl.def"
#include "FPGA.def"
#include "i2c.def"
#include "Modbus_Comm.def"
#include "ModCntrl.def"
#include "MultiAxis.def"
#include "ResCnfg.def"
#include "Ser_Comm.def"
#include "SysInit.def"
#include "Flash.def"
#include "AutoTune.def"
#include "Homing.def"
#include "Exe_IO.def"
#include "CommFdbk.def"
#include "ExFbVar.def"
#include "PtpGenerator.def"
#include "Velocity.def"
#include "402fsa.def"
#include "BiSS_C.def"

#include "Display.var"
#include "Drive.var"
#include "Endat.var"
#include "Exe_Hndl.var"
#include "Extrn_Asm.var"
#include "FltCntrl.var"
#include "Foldback.var"
#include "i2c.var"
#include "Init.var"
#include "Modbus_Comm.var"
#include "ModCntrl.var"
#include "Motor.var"
#include "MotorSetup.var"
#include "Position.var"
#include "PtpGenerator.var"
#include "Runtime.var"
#include "Ser_Comm.var"
#include "Units.var"
#include "User_Var.var"
#include "Zeroing.var"
#include "AutoTune.var"
#include "ExFbVar.var"
#include "Prototypes.pro"
#include "Homing.var"
#include "Exe_IO.var"
#include "An_Fltr.var"
#include "CommFdbk.var"
#include "Velocity.var"
#include "BiSS_C.var"
#include "MotorParamsEst.var"
#include "FlashHandle.var"
#include "utility.h"

#include "BiSS_C.pro"
#include "Display.pro"

typedef void (*fptr_void)(void);
typedef int  (*fptr_int) (void);
extern unsigned int BGVAR_DEFINE(u16_fb_curr_obj_id);

extern PDO_MAPPING4_T	p301_n1_rpdo_map;
extern PDO_MAPPING4_T	p301_n2_rpdo_map;
extern PDO_MAPPING4_T	p301_n3_rpdo_map;
extern PDO_MAPPING4_T	p301_n4_rpdo_map;
extern PDO_MAPPING4_T	p301_n1_tpdo_map;
extern PDO_MAPPING4_T	p301_n2_tpdo_map;
extern PDO_MAPPING4_T	p301_n3_tpdo_map;
extern PDO_MAPPING4_T	p301_n4_tpdo_map;

//**********************************************************
// Function Name: SalDriveIPeakCommand
// Description:
//    This function is called in response to the DIPEAK message.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
// DIPEAK SAL
int SalDriveIPeakCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_Drive_I_Peak) != (long)lparam) return (NOT_PROGRAMMABLE);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalDriveIContCommand
// Description:
//    This function is called in response to the DICONT message, to set or query the
//        drive continuous current.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalDriveIContCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_Drive_I_Cont) != (long)lparam) return (NOT_PROGRAMMABLE);

   return (SAL_SUCCESS);
}


// PWMFRQ
int SalPwmFreqCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u32_Pwm_Freq) != (long)param) return (NOT_PROGRAMMABLE);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMotorIPeakCommand
// Description:
//    This function is called in response to the MIPEAK message, to set or query the
//        motor peak current.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorIPeakCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_Motor_I_Peak) == (long)lparam) return (SAL_SUCCESS);

   BGVAR(s32_Motor_I_Peak) = (long)lparam;

   CalcImax(DRIVE_PARAM);
   CalcILim(DRIVE_PARAM);
   CalcFoldbackParam(DRIVE_PARAM, MOTOR_FOLDBACK);

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


void UpdateTorqueSlope(int drive)
{
  // Update the units conversion for Torque Slope
   ConvertFbCanCurrentPerSecToInternal(drive);
   // Update the internal value accordingly
   //SalSetTorqueSlopeCommand((unsigned long long)BGVAR(u32_Icmd_Slp),drive);
}


//**********************************************************
// Function Name: MotorIContCommand
// Description:
//    This function is called in response to the MICONT message, to set or query the
//        motor continuous current.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorIContCommand(long long lparam, int drive)
{
   float f_tmp = 0.0;

   if (BGVAR(s32_Motor_I_Cont) == (long)lparam) return (SAL_SUCCESS);

   BGVAR(s32_Motor_I_Cont) = (long)lparam;

   BGVAR(u32_Fb_Motor_Rated_Current) = (unsigned long)BGVAR(s32_Motor_I_Cont);    // canopen micont and serial micont are both in mA

   f_tmp = (float)((float)BGVAR(u32_Fb_Motor_Rated_Current) * ((float)BGVAR(u32_Motor_Kt)/1000.0));
   p402_motor_rated_torque = (unsigned int)f_tmp;

   CalcImax(DRIVE_PARAM);
   CalcFoldbackParam(DRIVE_PARAM, MOTOR_FOLDBACK);
   UpdateTorqueSlope(drive);
   ConvertFbCanOpenTorque(drive);
   ConvertFbCanOpenCurrent(drive);
   ConvertFbCanCurrentPerSecToUser(drive);
   ConvertFbCanCurrentPerSecToInternal(drive);

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMpolesCommand
// Description:
//      This function is called in response to the MPOLES command, to set or
//      query the number of poles in the motor.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMpolesCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (!(u16_Fw_Features & MPOLES_LIMIT_MASK))
      if (20 < lparam)
         return VALUE_TOO_HIGH;

   if ((lparam % 2) != 0) return (ARGUMENT_NOT_EVEN);

   if (BGVAR(u16_Mpoles) == (int)lparam) return (SAL_SUCCESS);

   // Dont allow to set MPOLES if MTP is used (MTPMODE == 1) ||(MTPMODE == 3)
   if ( (BGVAR(u16_MTP_Mode) == 1)  || (BGVAR(u16_MTP_Mode) == 3) )
      return (MTP_USED);

   BGVAR(u16_Mpoles) = (int)lparam;

   VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMspeedCommand
// Description:
//          This function is called in response to the MSPEED command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMspeedCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u32_Mspeed) == (unsigned long)lparam) return (SAL_SUCCESS);

   if (lparam < 0LL) return (VALUE_TOO_LOW);
   if (lparam > 0x7FFFFFFF) return (VALUE_TOO_HIGH);  // max speed due to implementation is 0.5 (half of 32 bit) rev per sample time (125 uS)
                                                      // *     => 0.5rev * 8000 * 60 = 240000 rpm
   BGVAR(u32_Mspeed) = (unsigned long)lparam;

   UpdateVelocityLimits(DRIVE_PARAM);

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}

/* Locate here tests depnded on I/O, rather than on drive itself (in which case the tests run on the SAL functions)*/
int VerifyLegitConfig (void)
{
   if (!(u16_Fw_Features & MENCRES_LIMIT_MASK))
      if ((BGVAR(u16_FdbkType) == INC_ENC_FDBK) && (10000 < BGVAR(u32_User_Motor_Enc_Res))) //MENCRES validation
         return (CONFIG_FAIL_MENCRES_HIGH);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalConfigCommand
// Description:
//          This function is called in response to the CONFIG command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalConfigCommand(int drive)
{
    // AXIS_OFF;
    int drive_backup = drive;
    int u16_return_legit_value = 0;

    u16_return_legit_value = VerifyLegitConfig();

    if (SAL_SUCCESS != u16_return_legit_value)
    {
        return u16_return_legit_value;
    }

    //  ignore CONFIG command if DUMP command is active
    //  since we cannot block execution of CONFIG command when it comes from DUMP command
    if (u8_is_dump_command_active)
    {
        return SAL_SUCCESS;
    }
    else
    {
        if (Enabled(drive))
        {
            return DRIVE_ACTIVE;
        }

        // in case of menctype == 9,Donot allow config while ENDAT is busy
        // unless the s16_Endat_Init_State == ENDAT_ERR_IDLE_STATE
        if (AX0_s16_Motor_Enc_Type == 9)
        {
            drive = 0;

            if ((AX0_s32_Feedback_Ptr == &SW_SINE_ENC_FEEDBACK) &&
                (BGVAR(s16_Endat_Init_State) != ENDAT_IDLE_STATE) &&
                (BGVAR(s16_Endat_Init_State) != ENDAT_ERR_IDLE_STATE))
            {
                return ENDAT_BUSY_ERROR;
            }

            drive = drive_backup;
        }

        if (strstr(s8_Control_Drive_Model_Number_1, "SDA7") != NULL)
        {
            //  The Higen product doesn't support ENDAT feedaback type.
            if (((BGVAR(u16_FdbkType) == SW_SINE_FDBK) && (VAR(AX0_s16_Motor_Enc_Type) == 9)) ||
                ((BGVAR(u16_FdbkType) == ENDAT2X_COMM_FDBK) && (VAR(AX0_s16_Motor_Enc_Type) == 0)))
            {
                return ENDAT_2_X_NOT_SUPPORTED;
            }
        }

        return (DesignRoutine(1, DRIVE_PARAM));
    }
}


//**********************************************************
// Function Name: SalWriteConfigCommand
// Description:
//          This function has been added in order to fix a parameter restore issue
//          we had with the customer Labman. This fix is for Bugzilla 3952. When
//          calling "CONFIG 0" we issue a CONFIG but we do not return any error
//          code to the user. This "CONFIG 0" command is therefore used within a
//          DUMP file upon parameter restore.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteConfigCommand(int drive)
{
   int s16_return_value = SalConfigCommand(drive);

   if (s16_return_value == SAL_NOT_FINISHED)
   {
      return (SAL_NOT_FINISHED);
   }
   else
   {
      return (SAL_SUCCESS);
   }
}

//**********************************************************
// Function Name: SalWritePositionErrorMax
// Description:
//          This function checks the value that the user is
//          trying to write into the PEMAX variable.
//
// Author: Daniel
// Algorithm:
// Revisions:
//**********************************************************
int SalWritePositionErrorMax(long long param,int drive)
{
    //  Defeat compilation error
    REFERENCE_TO_DRIVE;

    //  Validate the Position Error Max have not passed (2^31 - 1) [revolutions]
    //  This is why we are checking bit No. 63.
    if ((param & (1LL << 63)) != 0)
    {
        return VALUE_TOO_HIGH;
    }

    //  Set the PEMAX
    BGVAR(u64_Pe_Max) = (unsigned long long)param;

    return SAL_SUCCESS;
}

// **********************************************************
// Function Name: SalReadSchneiderSoftwareVersion
// Description:
//    This function reads the P-param P0-00, which is the
//    software version of the Lexium product.
//
// Author: APH
// Algorithm:
// Revisions:
// **********************************************************
int SalReadSchneiderSoftwareVersion(long long *data, int drive)
{
   unsigned long u32_Software_Version = 0;
   unsigned char u8_index;

   // Generate the software version out of the Schneider version string
   // (Byte 8...11). Use multiplicator 16 since the software version number
   // is supposed to be represented in hexadecimal format.
   for (u8_index = 8; u8_index <= 11; u8_index++)
   {
      u32_Software_Version = 16 * u32_Software_Version;
      u32_Software_Version = u32_Software_Version + (p_s8_Shndr_Software_Version[u8_index] - '0');
   }

   drive++; // Just to remove compiler remark

   *data = (long long)u32_Software_Version;

   return (SAL_SUCCESS);
}


// **********************************************************
// Function Name: SalReadSchneiderSoftwareRevision
// Description:
//    This function is called upon a P-param P5-00 read access,
//    which is the software revision of the Lexium product.
//
// Author: APH
// Algorithm:
// Revisions:
// **********************************************************
int SalReadSchneiderSoftwareRevision(long long *data, int drive)
{
   unsigned long u32_Software_Revision = 0;
   unsigned char u8_index;

   // Generate the software revision out of the last 2 characters in the Schneider
   // version string (Byte 12...13). Use multiplicator 16 since the revision number
   // is supposed to be represented in hexadecimal format.
   for (u8_index=12; u8_index<=13; u8_index++)
   {
      u32_Software_Revision = 16 * u32_Software_Revision;
      u32_Software_Revision = u32_Software_Revision + (p_s8_Shndr_Software_Version[u8_index] - '0');
   }

   drive++; // Just to remove compiler remark

   *data = (long long)u32_Software_Revision;

   return (SAL_SUCCESS);
}


// **********************************************************
// Function Name: SalReadSchneiderProgramNumber
// Description:
//    This function is called upon a P-param P9-00 read access,
//    which is the program number of the Lexium product.
//
// Author: APH
// Algorithm:
// Revisions:
// **********************************************************
int SalReadSchneiderProgramNumber(long long *data, int drive)
{
   unsigned long u32_Program_Number = 0;
   unsigned char u8_index;

   // Generate the program number out of the 6 characters sfter the 'P' in the Schneider
   // version string (Byte 1...6). Use multiplicator 16 since the program number
   // is supposed to be represented in hexadecimal format.
   for (u8_index = 1; u8_index <= 6; u8_index++)
   {
      u32_Program_Number = 16 * u32_Program_Number;
      u32_Program_Number = u32_Program_Number + (p_s8_Shndr_Software_Version[u8_index] - '0');
   }

   drive++; // Just to remove compiler remark

   *data = (long long)u32_Program_Number;

   return (SAL_SUCCESS);
}


// **********************************************************
// Function Name: SalReadSchneiderSoftwareVersionDate
// Description:
//    This function reads the P-param P9-01, which is the
//    software version date of the Lexium product.
//
// Author: APH
// Algorithm:
// Revisions:
// **********************************************************
int SalReadSchneiderSoftwareVersionDate(long long *data, int drive)
{
   unsigned long u32_Software_Version_Date = 0;
   unsigned char u8_index;

   // Generate the software version date out of the last date-string. Use
   // the multiplicator 16 since the version string is supposed to be
   // represented in hexadecimal format.
   for (u8_index = 0; u8_index <= 7; u8_index++)
   {
      u32_Software_Version_Date = 16 * u32_Software_Version_Date;
      u32_Software_Version_Date = u32_Software_Version_Date + (p_s8_Version_Date[u8_index] - '0');
   }

   drive++; // Just to remove compiler remark

   *data = (long long)u32_Software_Version_Date;

   return (SAL_SUCCESS);
}


// VER command
int VerCommand(void)
{
   static int ver_state = 0;
   char buff[128];

   if (u8_Output_Buffer_Free_Space < 100) return SAL_NOT_FINISHED;

   switch (ver_state)
   {
      case 0: //Digital Servo Drive
         PrintString(p_s8_Ver_Command_Response[0], 0);
         PrintCrLf();
         ver_state++;
      break;

      case 1: //--------------------------------
         PrintString(p_s8_Ver_Command_Response[1], 0);
         PrintCrLf();
         ver_state++;
      break;

      case 2: //Firmware version
         PrintString(p_s8_Ver_Command_Response[2], 0);
         PrintString((char *)p_s8_CDHD_Drive_Version, 0);
         PrintCrLf();
         ver_state++;
      break;

      case 3://FieldBus version
         if (u32_Hw_Features & ETHERNET_BASED_FIELDBUS_MASK)
         {// print the FieldBus line only if the drive supports FieldBus communication
            PrintString(p_s8_Ver_Command_Response[3], 0);
            PrintString(s8_FieldBus_Version,0);
            PrintCrLf();
         }
         ver_state++;
      break;

      case 4://EtherCAT ESI version
         if (u32_Hw_Features & ETHERNET_BASED_FIELDBUS_MASK)
         {// print the ESI line only if the drive supports FieldBus communication
            PrintString(p_s8_Ver_Command_Response[4], 0);
            PrintDecInAsciiHex(u32_ESI_Eeprom_Read_Version, 8);
            PrintCrLf();
         }
         ver_state++;
      break;

      case 5://FPGA version
         PrintString(p_s8_Ver_Command_Response[5], 0);
         PrintFpgaVersion(buff);
         PrintString(buff, 0);
         PrintCrLf();
         ver_state++;
      break;

      case 6: //Resident version
         PrintString(p_s8_Ver_Command_Response[6], 0);

         GetResidentVersion(buff);
         if (buff[0])
            PrintString(buff, 0);

         PrintCrLf();
         ver_state = 0;
      break;
   }

   if (ver_state == 0) return (SAL_SUCCESS);
   else return (SAL_NOT_FINISHED);
}

void GetResidentVersion(char* buff)
{
   unsigned int u16_index;

   // if resident string exists
   if ( ((*(int*)(RESIDENT_VERSION_ADDR)) >= '0') &&
        ((*(int*)(RESIDENT_VERSION_ADDR)) <= '9')   )
   {
       //resident version is saved in memory addresses 0x100002 to 0x100020
       for(u16_index = 0; u16_index < RESIDENT_VERSION_LENGTH; u16_index++)
       {
            buff[u16_index] = *(int*)(RESIDENT_VERSION_ADDR + u16_index);
       }

       // terminate string
       buff[u16_index] = 0;
   }
   else
   {
       // return empty string
       buff[0] = 0;
   }
}

// INFO command
int SalInfoCommand(int drive)
{
   return (InfoCommand(drive, FB_NOT_IN_USE));
}


int InfoCommand(int drive,int s16_fb_use)
{
   static int first_time_running = 0;
   int number_of_chars_in_prefix = 0, label_parser_result = 0;
   unsigned int u16_leng_of_string;

   if (first_time_running == 0)
   {// preform only for first time entering to get the label type
      u16_leng_of_string = sizeof(s8_Control_Drive_Model_Number_1);
      for (number_of_chars_in_prefix = 0; number_of_chars_in_prefix < u16_leng_of_string; number_of_chars_in_prefix++)
      {// count the number of chars until '-' char or numeric char.
         if ( (s8_Control_Drive_Model_Number_1[number_of_chars_in_prefix] == '-')             ||
              ( (s8_Control_Drive_Model_Number_1[number_of_chars_in_prefix] >= ASCII_ZERO) &&
                (s8_Control_Drive_Model_Number_1[number_of_chars_in_prefix] <= ASCII_NINE)   )  )
            break; // the char is not
      }
      // find the exact label type and set the label parser ID to it's unique ID.

      switch(number_of_chars_in_prefix)
      {
         case 2:// 2 chars label prefix
            if ( (s8_Control_Drive_Model_Number_1[0] == 'M') &&
                 (s8_Control_Drive_Model_Number_1[1] == 'T')   )
            { // "MT" PBA System label prefix found
               u16_label_type_id = 2;
            break;
            }
            if ( (s8_Control_Drive_Model_Number_1[0] == 'H') &&
                 (s8_Control_Drive_Model_Number_1[1] == 'E')   )
            { // "HE" HornerAPG label prefix found
               u16_label_type_id = 6;
            }
         break;

         case 3:// 3 chars label prefix
            if ( (s8_Control_Drive_Model_Number_1[0] == 'A') &&
                 (s8_Control_Drive_Model_Number_1[1] == 'S') &&
                 (s8_Control_Drive_Model_Number_1[2] == 'D')   )
            { // "ASD" Akribis label prefix found
               u16_label_type_id = 7;
            }
         break;

         default://4 chars label prefix is the default label prefix (CDHD,MSHD,GTHD,FPRO).
            u16_label_type_id = 1;
         break;
      }

      first_time_running = 1;
      u16_info_state = 0;
   }

   // present the info according to specific label acording to the "label_type_id" value.
   label_parser_result = GlobalLabelParser(drive, u16_label_type_id,s16_fb_use);

   if (label_parser_result == SAL_SUCCESS)
   {// reset all static stuff for nex time running info command
      first_time_running = 0;
      u16_label_type_id = 0;
   }
   return label_parser_result;
}


//**********************************************************
// Function Name: OutputBufferLoader
// Description:
//          This function manages the printing of the data
//          at the info command to the uart buffer
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int OutputBufferLoader(void)
{
  int i = 0, u16_Data_Len = 0;

  if (u8_Output_Buffer_Free_Space >= COMMS_BUFFER_SIZE - 20)
  {
     u16_Data_Len = strlen((char *)&u16_General_Domain[0]);
     for (i = 0; i < u16_Data_Len; i++)
        WriteOutputBuffer(u16_General_Domain[i]);
     u16_General_Domain[0] = 0;
     return 1;
  }
  else
     return 0;
}

int debug_lior = 0;
int GlobalLabelParser(int drive, int local_label_type_id, int s16_fb_use)
{
   // AXIS_OFF;
   char u16_temp_str[10], buff[128];
   int drive_backup = drive,u16_num_of_chrcters = 0;
   int s16_index;

   // in case use of the uart port
   if (s16_fb_use == FB_NOT_IN_USE)
   {
      if (OutputBufferLoader()==0) return SAL_NOT_FINISHED;
   }

   strcpy((char*)&u16_General_Domain[0],"\0");
   switch (u16_info_state)
   {
      case 0:// Digital Servo Drive
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[0]);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 1:// -------------------------------------
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[1]);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 2:// Drive model number
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[2]);

         //in case of higen drive ("SDA7"),there might be spaces by the end of each string.they wont be printed.
         if (strstr(s8_Control_Drive_Model_Number_1, "SDA7") == NULL)
         {
            strcat((char*)&u16_General_Domain[0],(char *)&s8_Control_Drive_Model_Number_1[0]);
            strcat((char*)&u16_General_Domain[0],(char *)&s8_Power_Drive_Model_Number_2[0]);
//            if (DDHD == u16_Product)
//               strncat((char*)&u16_General_Domain[0],(char *)&s8_DDHD__Drive_Model_6th_Char[0],1);
            strcat((char*)&u16_General_Domain[0],(char *)&s8_Control_Drive_Model_Number_3[0]);
         }
         else
         {
            u16_num_of_chrcters = strchr(s8_Control_Drive_Model_Number_1,' ') - s8_Control_Drive_Model_Number_1;
            strncat((char*)&u16_General_Domain[0],(char *)&s8_Control_Drive_Model_Number_1[0],u16_num_of_chrcters);
            u16_num_of_chrcters = strchr(s8_Power_Drive_Model_Number_2,' ') - s8_Power_Drive_Model_Number_2;
            strncat((char*)&u16_General_Domain[0],(char *)&s8_Power_Drive_Model_Number_2[0],u16_num_of_chrcters);
            u16_num_of_chrcters = strchr(s8_Control_Drive_Model_Number_3,' ') - s8_Control_Drive_Model_Number_3;
            strncat((char*)&u16_General_Domain[0],(char *)&s8_Control_Drive_Model_Number_3[0],u16_num_of_chrcters);
         }
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 3:// Peak current
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[3]);
         strcat((char*)&u16_General_Domain[0],DecimalPoint32ToAscii(BGVAR(s32_Drive_I_Peak)));
         strcat((char*)&u16_General_Domain[0]," A / ");
         strcat((char*)&u16_General_Domain[0],DecimalPoint32ToAscii(100 *(long)BGVAR(s16_Drive_I_Peak_Arms)));
         strcat((char*)&u16_General_Domain[0]," ");
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[4]);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 4:// Cont curr
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[5]);
         strcat((char*)&u16_General_Domain[0],DecimalPoint32ToAscii((long)BGVAR(s32_Drive_I_Cont)));
//                                strcat((char*)&u16_General_Domain[0],DecimalPoint32ToAscii(100 * (long)BGVAR(s32_Drive_I_Cont)));
         strcat((char*)&u16_General_Domain[0]," A / ");
         strcat((char*)&u16_General_Domain[0],DecimalPoint32ToAscii(100 * (long)BGVAR(s16_Drive_I_Cont_Arms)));
         strcat((char*)&u16_General_Domain[0]," ");
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[4]);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 5:// feedback type
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[6]);
         switch (BGVAR(u16_FdbkType))
         {
            case RESOLVER_FDBK:strcat((char*)&u16_General_Domain[0],"Resolver");
            break;
            case INC_ENC_FDBK:strcat((char*)&u16_General_Domain[0],"Incremental encoder");
               if (VAR(AX0_s16_Motor_Enc_Type) == 0) strcat((char*)&u16_General_Domain[0]," (A/B/I/Halls)");
               if ((VAR( AX0_s16_Motor_Enc_Type) == 1) || (VAR(AX0_s16_Motor_Enc_Type) == 2)) strcat((char*)&u16_General_Domain[0]," (A/B/I)");
               if ((VAR(AX0_s16_Motor_Enc_Type) == 3) || (VAR(AX0_s16_Motor_Enc_Type) == 4) || (VAR(AX0_s16_Motor_Enc_Type) == 12)) strcat((char*)&u16_General_Domain[0]," (A/B)");
               if (VAR(AX0_s16_Motor_Enc_Type) == 5) strcat((char*)&u16_General_Domain[0]," (Halls only)");
               if (VAR(AX0_s16_Motor_Enc_Type) == 6) strcat((char*)&u16_General_Domain[0]," (A/B/Halls)");
               if (VAR(AX0_s16_Motor_Enc_Type) == 11) strcat((char*)&u16_General_Domain[0]," (A/B/I/Halls Tamagawa)");
            break;

            case SW_SINE_FDBK:strcat((char*)&u16_General_Domain[0],"Sine encoder");
               if (VAR(AX0_s16_Motor_Enc_Type) == 0) strcat((char*)&u16_General_Domain[0]," (A/B/I/Halls)");
               if ((VAR(AX0_s16_Motor_Enc_Type) == 1) || (VAR(AX0_s16_Motor_Enc_Type) == 2)) strcat((char*)&u16_General_Domain[0]," (A/B/I)");
               if ((VAR(AX0_s16_Motor_Enc_Type) == 3) || (VAR(AX0_s16_Motor_Enc_Type) == 4)) strcat((char*)&u16_General_Domain[0]," (A/B)");
               if (VAR(AX0_s16_Motor_Enc_Type) == 6) strcat((char*)&u16_General_Domain[0]," (A/B/Halls)");
               if (VAR(AX0_s16_Motor_Enc_Type) == 9) strcat((char*)&u16_General_Domain[0]," (Endat)");
               if (VAR(AX0_s16_Motor_Enc_Type) == 10) strcat((char*)&u16_General_Domain[0]," (Hiperface)");
            break;

            case NK_COMM_FDBK:strcat((char*)&u16_General_Domain[0],"Nikon MAR-A40A (SD) Feedback");
            break;
            case SL_FDBK:strcat((char*)&u16_General_Domain[0],"Sensorless");
            break;
            case TAMAGAWA_COMM_MULTI_TURN_FDBK:strcat((char*)&u16_General_Domain[0],"Tamagawa 17Bit Feedback Multi Turn");
            break;
            case TAMAGAWA_COMM_SINGLE_TURN_FDBK:strcat((char*)&u16_General_Domain[0],"Tamagawa 17Bit Feedback Single Turn");
            break;
            case FORCED_COMM_FDBK:strcat((char*)&u16_General_Domain[0],"Forced Commutation");
            break;
            case PS_P_G_COMM_FDBK:strcat((char*)&u16_General_Domain[0],"Custom (Ps) Incremental Feedback");
            break;
            case FANUC_COMM_FDBK:strcat((char*)&u16_General_Domain[0],"Custom (Fa) Feedback");
            break;
            case ENDAT2X_COMM_FDBK:strcat((char*)&u16_General_Domain[0],"EnDat2.1/2.2 Comm. Only");
            break;
            case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
            case SERVOSENSE_MULTI_TURN_COMM_FDBK:
               strcat((char*)&u16_General_Domain[0],"sensAR Magnetic Encoder ");
               if(BGVAR(u16_SrvSns_ProductType) == SRVSNS_SINGLE_TURN)
                  strcat((char*)&u16_General_Domain[0],"Single Turn");
               else if(BGVAR(u16_SrvSns_ProductType) == SRVSNS_MULTI_TURN)
                  strcat((char*)&u16_General_Domain[0],"Multi Turn");
               else
               {
                  strcat((char*)&u16_General_Domain[0],"Unknown Model: ");
                  strcat((char*)&u16_General_Domain[0],UnsignedInt64ToAscii((unsigned long long)BGVAR(u16_SrvSns_ProductType)));
               }
            break;
            case PS_S_COMM_FDBK:strcat((char*)&u16_General_Domain[0],"Custom (PS) Absolute Feedback");
            break;
            case SANKYO_COMM_FDBK:strcat((char*)&u16_General_Domain[0],"Sankyo 20Bit Feedback");
            break;
            case YASKAWA_ABS_COMM_FDBK:strcat((char*)&u16_General_Domain[0],"Custom (Ys) Absolute Feedback");
            break;
            case BISSC_COMM_FDBK:strcat((char*)&u16_General_Domain[0],"Generic BiSS-C Feedback");
            break;
            case YASKAWA_INC_COMM_FDBK:strcat((char*)&u16_General_Domain[0],"Custom (Ys) Incremental Feedback");
            break;
            case TAMAGAWA_CID0_SINGLE_TURN:strcat((char*)&u16_General_Domain[0],"Tamagawa (CID0) Feedback Single Turn");
            break;
            default:
               strcat((char*)&u16_General_Domain[0],"Unknown");
            break;
         }
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 6: // inteface and
      case 7: // voltage are preformed per label type ID
         switch(local_label_type_id)
         {
            case 2:// "MT" PBA System label prefix
               u16_info_state = PBATypeLabelParser(u16_info_state);
            break;
            case 6:// "HE" HornerAPG label prefix
               u16_info_state = HornerTypeLabelParser(u16_info_state );
            break;
            case 7:// "ASD" Akribis label prefix
               u16_info_state = AkribisTypeLabelParser(u16_info_state );
            break;

            default: // 1: // "CDHD" Servotronix label prefix, "MSHD" Moons label prefix,"GTHD" Googoltech label prefix,"FPRO" MPC label prefix,"SDA7" for Higen
               u16_info_state = ServotronixTypeLabelParser(u16_info_state );
            break;
         }
      break;

      case 8: //product serial number and manufacturing date
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[9]);
         if (s8_Product_Serial_Number[0] != 0)
         {
            strcat((char*)&u16_General_Domain[0],(char*)&s8_Product_Serial_Number[0]);
            strcat((char*)&u16_General_Domain[0],", ");

            //print month
            s16_index = (int)s8_Product_Serial_Number[3];
            if (s16_index < 74)// - "I" is skipped therefor we use the if
               s16_index -= 'A';
            else
               s16_index -= 'B';

            // protect the range of the index
            if (s16_index < 0)  s16_index = 0;
            if (s16_index > 11) s16_index = 11;

            strcat((char*)&u16_General_Domain[0],(p_s8_Month_Of_The_Year[s16_index]));

            //print year
            strcat((char*)&u16_General_Domain[0]," 20");
            u16_temp_str[0] = s8_Product_Serial_Number[1];
            u16_temp_str[1] = s8_Product_Serial_Number[2];
            u16_temp_str[2]= 0;
            strcat((char*)&u16_General_Domain[0],(char*)&u16_temp_str[0]);
         }
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 9: //Control board title
         strcat((char*)&u16_General_Domain[0],"\r\n");
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[10]);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 10: //Control board product number
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[11]);
         strcat((char*)&u16_General_Domain[0],(char*)&s8_Control_Board_Part_Number[0]);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 11: //Control board serial number
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[12]);
//       strcat((char*)&u16_General_Domain[0],(char*)&s8_Control_Board_Serial_Number[0]);
         strcat((char*)&u16_General_Domain[0],s8_Control_Board_Serial_Number);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 12: //Control board HW revision
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[13]);
         if (u16_Control_Board_Rev == 0xFFFF)
            strcat((char*)&u16_General_Domain[0],"NA");
         else
         {
            u16_temp_str[0]=u16_Control_Board_Rev & 0x00ff;
            u16_temp_str[1]=(u16_Control_Board_Rev >> 8) & 0x00ff;
            u16_temp_str[2]= 0;
            strcat((char*)&u16_General_Domain[0],(char*)&u16_temp_str[0]);
         }
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 13: //Control board EEPROM version
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[14]);
         strcat((char*)&u16_General_Domain[0],SignedIntegerToAscii((long)u16_Control_Board_EE_Version));
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 14: // Control board Flash Vendor & Device ID
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[22]);
         s16_index = 0;
         while (s16_index < s16_Number_Of_Flashes)
         {
            if (u16_Vendor_ID[s16_index] == 0x00BF)      // SST Vendor ID
            {
               strcat((char*)&u16_General_Domain[0],p_s8_Flash_Vendor_Options[0]);
               strcat((char*)&u16_General_Domain[0]," ");
               if (u16_Device_ID[s16_index] == 0x234F)
                  strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[0]);
               else if (u16_Device_ID[s16_index] == 0x234B)
                  strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[1]);
               else if (u16_Device_ID[s16_index] == 0x234A)
                  strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[2]);
               else if (u16_Device_ID[s16_index] == 0x234E)
                  strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[3]);
               else if (u16_Device_ID[s16_index] == 0x235B)
                  strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[4]);
               else if (u16_Device_ID[s16_index] == 0x235D)
                  strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[5]);
               else if (u16_Device_ID[s16_index] == 0x235A)
                  strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[6]);
               else if (u16_Device_ID[s16_index] == 0x235C)
                  strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[7]);
               else if (u16_Device_ID[s16_index] == 0x235F)
                  strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[8]);
               else if (u16_Device_ID[s16_index] == 0x235E)
                  strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[9]);
               else if (u16_Device_ID[s16_index] == 0x227E)
               {
                  // Subinfo_1   Subinfo_2   Device ID
                  // --------    --------    ------------
                  //  0x220C      0x2200     SST38VF6401B
                  //  0x220C      0x2201     SST38VF6402B
                  //  0x2210      0x2200     SST38VF6403B - this device should be used
                  //  0x2210      0x2201     SST38VF6404B
                  if ( (u16_Device_ID_Subinfo_1[s16_index] == 0x220C) && (u16_Device_ID_Subinfo_2[s16_index] == 0x2200) )
                  {
                     strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[10]);
                  }
                  else if ( (u16_Device_ID_Subinfo_1[s16_index] == 0x220C) && (u16_Device_ID_Subinfo_2[s16_index] == 0x2201) )
                  {
                     strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[11]);
                  }
                  else if ( (u16_Device_ID_Subinfo_1[s16_index] == 0x2210) && (u16_Device_ID_Subinfo_2[s16_index] == 0x2200) )
                  {
                     strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[12]);
                  }
                  else if ( (u16_Device_ID_Subinfo_1[s16_index] == 0x2210) && (u16_Device_ID_Subinfo_2[s16_index] == 0x2201) )
                  {
                     strcat((char*)&u16_General_Domain[0],p_s8_SST_Device_Options[13]);
                  }
                  else
                  {
                     strcat((char*)&u16_General_Domain[0],"Unknown Device: ");
                     strcat((char*)&u16_General_Domain[0],DecToAsciiHex(u32_Product_ID[s16_index], 8));
                  }
               }
               else
               {
                  strcat((char*)&u16_General_Domain[0],"Unknown Device: ");
                  strcat((char*)&u16_General_Domain[0],DecToAsciiHex(u32_Product_ID[s16_index], 8));
               }
            }
            else if (u16_Vendor_ID[s16_index] == 0x0001) // Spansion Vendor ID
            {
               strcat((char*)&u16_General_Domain[0],p_s8_Flash_Vendor_Options[1]);
               strcat((char*)&u16_General_Domain[0]," ");
               if (u16_Device_ID[s16_index] == 0x2249)
               {
                  strcat((char*)&u16_General_Domain[0],p_s8_Spansion_Device_Options[0]);
               }
               else if (u16_Device_ID[s16_index] == 0x227E)
                  strcat((char*)&u16_General_Domain[0],p_s8_Spansion_Device_Options[1]);
               else
               {
                  strcat((char*)&u16_General_Domain[0],"Unknown Device: ");
                  strcat((char*)&u16_General_Domain[0],DecToAsciiHex(u32_Product_ID[s16_index],8));
               }
            }
            else if (u16_Vendor_ID[s16_index] == 0x0020) // MICRON Vendor ID
            {
               strcat((char*)&u16_General_Domain[0],p_s8_Flash_Vendor_Options[2]);
               strcat((char*)&u16_General_Domain[0]," ");
               if (u16_Device_ID[s16_index] == 0x22FD)
               {
                  strcat((char*)&u16_General_Domain[0],p_s8_Micron_Device_Options[0]);
               }
               else if (u16_Device_ID[s16_index] == 0x22ED)
               {
                  strcat((char*)&u16_General_Domain[0],p_s8_Micron_Device_Options[1]);
               }
               else
               {
                  strcat((char*)&u16_General_Domain[0],"Unknown Device: ");
                  strcat((char*)&u16_General_Domain[0],DecToAsciiHex(u32_Product_ID[s16_index],8));
               }
            }
            else if (u16_Vendor_ID[s16_index] == 0x00C2) // MACRONIX Vendor ID
            {
               strcat((char*)&u16_General_Domain[0],p_s8_Flash_Vendor_Options[3]);
               strcat((char*)&u16_General_Domain[0]," ");
               if (u16_Device_ID[s16_index] == 0x22CB)
               {
                  strcat((char*)&u16_General_Domain[0],p_s8_Macronix_Device_Options[0]);
               }
               else if (u16_Device_ID[s16_index] == 0x22C9)
               {
                  strcat((char*)&u16_General_Domain[0],p_s8_Macronix_Device_Options[1]);
               }
               else
               {
                  strcat((char*)&u16_General_Domain[0],"Unknown Device: ");
                  strcat((char*)&u16_General_Domain[0],DecToAsciiHex(u32_Product_ID[s16_index],8));
               }
            }
            else                                       // unknown Vendor
            {
               strcat((char*)&u16_General_Domain[0],"Unknown Device: ");
               strcat((char*)&u16_General_Domain[0],DecToAsciiHex(u32_Product_ID[s16_index],8));
            }

            if ((s16_Number_Of_Flashes == 2) && (s16_index == 0))  // if there are 2 flashes, determine the delimiter after the first flash ID
            {
               if ( (u16_Vendor_ID[0] == u16_Vendor_ID[1])                      &&
                    (u16_Device_ID[0] == u16_Device_ID[1])                      &&
                    (u16_Device_ID_Subinfo_1[0] == u16_Device_ID_Subinfo_1[1])  &&
                    (u16_Device_ID_Subinfo_2[0] == u16_Device_ID_Subinfo_2[1])      )
               {
                  // if the two flashes are identical, write " x2" and skip writing the second flash ID
                  strcat((char*)&u16_General_Domain[0]," x2");
                  s16_index++;
               }
               else
               {
                  // if the two flashes not identical, write ", " and write the second flash ID
                  strcat((char*)&u16_General_Domain[0],", ");
               }
            }
            s16_index++;
         }
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 15:
         // DO NOT REMOVE SPACES FROM STRING BELOW
         // in CANopen domain protocol it may be a problem to send line that is less than 8 bytes, so add 8 spaces
         strcat((char*)&u16_General_Domain[0],"        \r\n");
      break;

      case 16: // Power board title
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[15]);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 17: // Power board part number
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[11]);
         strcat((char*)&u16_General_Domain[0],(char*)&s8_Power_Board_Part_Number);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 18: // Power board serial number
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[12]);
         strcat((char*)&u16_General_Domain[0],(char*)&s8_Power_Board_Serial_Number);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 19: // Power board HW revision of axis 0
         drive = 0;
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[13]);
         if (BGVAR(u16_Power_Board_Rev) == 0xFFFF)
            strcat((char*)&u16_General_Domain[0],"NA");
         else
         {
            u16_temp_str[0] = BGVAR(u16_Power_Board_Rev) & 0x00ff;
            u16_temp_str[1] = (BGVAR(u16_Power_Board_Rev) >> 8) & 0x00ff;
            u16_temp_str[2] = 0;
            strcat((char*)&u16_General_Domain[0],(char*)&u16_temp_str[0]);
         }
         strcat((char*)&u16_General_Domain[0],"\r\n");
         drive = drive_backup;

         if ((u16_Num_Of_Axes) != 2)    u16_info_state++;
      break;

      case 20: // Power board HW revision 2
         drive = 1;
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[13]);
         if (BGVAR(u16_Power_Board_Rev) == 0xFFFF)
            strcat((char*)&u16_General_Domain[0],"NA");
         else
         {
            u16_temp_str[0] = BGVAR(u16_Power_Board_Rev) & 0x00ff;
            u16_temp_str[1] = (BGVAR(u16_Power_Board_Rev) >> 8) & 0x00ff;
            u16_temp_str[2] = 0;
            strcat((char*)&u16_General_Domain[0],(char*)&u16_temp_str[0]);
         }
         drive = drive_backup;
      break;

      case 21: // Power board EEPROM version
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[14]);
         strcat((char*)&u16_General_Domain[0],SignedIntegerToAscii((long)u16_Power_Board_EE_Version));
         strcat((char*)&u16_General_Domain[0],"\r\n\r\n");
      break;

      case 22: // firmware version
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[16]);
         strcat((char*)&u16_General_Domain[0],p_s8_CDHD_Drive_Version);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 23: // FieldBus version
         if (u32_Hw_Features & ETHERNET_BASED_FIELDBUS_MASK)
         {// print the FieldBus line only if the drive supports FieldBus communication
            strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[17]);
            strcat((char*)&u16_General_Domain[0],(char*)&s8_FieldBus_Version);
            strcat((char*)&u16_General_Domain[0],"\r\n");
            break;
         }
         else
         {
            u16_info_state++;
         }
         // no break - fall to next case.

      case 24: // ESI version
         if (u32_Hw_Features & ETHERNET_BASED_FIELDBUS_MASK)
         {// print the FieldBus line only if the drive supports FieldBus communication
            strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[18]);
            strcat((char*)&u16_General_Domain[0],DecToAsciiHex(u32_ESI_Eeprom_Read_Version,8));
            strcat((char*)&u16_General_Domain[0],"\r\n");
            break;
         }
         else
         {
            u16_info_state++;
         }
         // no break - fall to next case.

      case 25: // FPGA version
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[19]);
         PrintFpgaVersion(buff);
         strcat((char*)&u16_General_Domain[0],(char*)&buff);
         strcat((char*)&u16_General_Domain[0],"\r\n");
         break;

      case 26: //Resident version
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[20]);
         GetResidentVersion(buff);
         if (buff[0])
            strcat((char*)&u16_General_Domain[0],(char*)&buff);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 27: // Runtime
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[21]);
         strcat((char*)&u16_General_Domain[0],UnsignedIntegerToAscii((unsigned long)u16_RunTimeHours));
         strcat((char*)&u16_General_Domain[0],":");
         if (u16_RunTimeMins < 10) strcat((char*)&u16_General_Domain[0],"0");
            strcat((char*)&u16_General_Domain[0],UnsignedIntegerToAscii((unsigned long)u16_RunTimeMins));
         strcat((char*)&u16_General_Domain[0],":");
         if (u16_RunTimeSecs < 10) strcat((char*)&u16_General_Domain[0],"0");
            strcat((char*)&u16_General_Domain[0],UnsignedIntegerToAscii((unsigned long)u16_RunTimeSecs));
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;
   }

   if (u16_info_state >= 28)
   {
      if (0 == debug_lior)
      {
       //  strcat((char*)&u16_General_Domain[0],"\r\n");
         debug_lior++;
      }
      // in case use of the uart port
      if (s16_fb_use == FB_NOT_IN_USE)
      {
         if (OutputBufferLoader() == 0) return SAL_NOT_FINISHED;
      }
      debug_lior = 0;
      u16_info_state = 0;
      return (SAL_SUCCESS);
   }

   u16_info_state++;

   return (SAL_NOT_FINISHED);
}


//**********************************************************
// Function Name: ServotronixTypeLabelParser
// Description:
//          This function is called from GlobalLabelParser
//          function for parsing:
//          Servotronix ("CDHD"), Moons("MSHD"),
//          Googoltech("GTHD") and MPC("FPRO") Labels.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int ServotronixTypeLabelParser(int local_info_state )
{
   unsigned int u16_temp_str[2];

   strcpy((char*)&u16_General_Domain[0],"\0");
   switch(local_info_state)
   {
      case 6: //Interface
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[7]);
         switch(s8_Control_Drive_Model_Number_3[0])
         {
             case 'A':
               if (s8_Control_Drive_Model_Number_3[1] == 'P')
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[0]);
               else if (s8_Control_Drive_Model_Number_3[1] == 'C')
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[1]);
               else if (s8_Control_Drive_Model_Number_3[1] == 'F')
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[2]);
            break;

            case 'E':
               strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[3]);
               strcat((char*)&u16_General_Domain[0],", ");
               strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[12]);
               strcat((char*)&u16_General_Domain[0],", ");
               strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[10]);
            break;

            case 'G':
               if (s8_Control_Drive_Model_Number_3[1] == 'L')
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[5]);
               else if (s8_Control_Drive_Model_Number_3[1] == 'T')
                 strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[6]);
            break;

            case 'P':
               strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[13]);
               strcat((char*)&u16_General_Domain[0],", ");
               strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[12]);
               strcat((char*)&u16_General_Domain[0],", ");
               strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[10]);
            break;

            default:
               strcat((char*)&u16_General_Domain[0],"Unknown");
            break;
         }
         strcat((char*)&u16_General_Domain[0],"\r\n");
         break;

      case 7: //Voltage
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[8]);

         //In case of Higen product, if the total length of Higen Motor SDA model type
         // is 12 occupying characters,then it's 400V device,
         // else .if 10 characters in use, it's 200V device,
         if (DDHD == u16_Product)
         {
            strcat((char*)&u16_General_Domain[0],"200 V");
         }
         else if (strstr(s8_Control_Drive_Model_Number_1, "SDA7") != NULL)
         {
            if (strchr(s8_Power_Drive_Model_Number_2,'H') != NULL)
               strcat((char*)&u16_General_Domain[0],"400 V");
            else
               strcat((char*)&u16_General_Domain[0],"200 V");
         }
         else
         {
            if (s8_Power_Drive_Model_Number_2[0] != 0)
            {
                  if ((s8_Power_Drive_Model_Number_2[3] >= ASCII_ZERO )&&(s8_Power_Drive_Model_Number_2[3] <= ASCII_NINE ))
                  {
                     u16_temp_str[0]=s8_Power_Drive_Model_Number_2[3];
                     u16_temp_str[1]=0;
                     strcat((char*)&u16_General_Domain[0],(char *)&u16_temp_str[0]);
                     strcat((char*)&u16_General_Domain[0],"00 V");
                  }
                  else
                  {// not numeric value
                     strcat((char*)&u16_General_Domain[0],"Unknown");
                  }
             }
         }
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

     default:
         local_info_state--;
     break;
     }
     return local_info_state;
}


//**********************************************************
// Function Name: PBATypeLabelParser
// Description:
//          This function is called from GlobalLabelParser
//          function for parsing PBA ("MT") Label.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int PBATypeLabelParser(int local_info_state )
{
   unsigned int u16_temp_index = 0;
   unsigned int u16_temp_str[2];

   strcpy((char*)&u16_General_Domain[0],"\0");
   switch (local_info_state)
   {
      case 6: //Interface
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[7]);
         for(u16_temp_index=0;u16_temp_index<5;u16_temp_index++)
         {
            if ((s8_Control_Drive_Model_Number_3[u16_temp_index] >= 'A') && (s8_Control_Drive_Model_Number_3[u16_temp_index+1] <= 'Z'))
            {// find the interface 2 char type indicator
               break;
            }
         }
         switch(s8_Control_Drive_Model_Number_3[u16_temp_index])
         {
            case 'A':
               if (s8_Control_Drive_Model_Number_3[u16_temp_index+1] == 'P')// "AP - interface"
               {
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[7]);
                  strcat((char*)&u16_General_Domain[0],", ");
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[8]);
                  strcat((char*)&u16_General_Domain[0],", ");
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[10]);
               }
               else if (s8_Control_Drive_Model_Number_3[u16_temp_index+1] == 'F')// "AF - interface"
               {
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[7]);
                  strcat((char*)&u16_General_Domain[0],", ");
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[9]);
                  strcat((char*)&u16_General_Domain[0],", ");
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[11]);
                  strcat((char*)&u16_General_Domain[0],", ");
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[10]);
                  strcat((char*)&u16_General_Domain[0],", ");
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[12]);
               }
            break;
            case 'E':
               if (s8_Control_Drive_Model_Number_3[u16_temp_index+1] == 'C')// "EC - interface"
                  strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[3]);
            break;
            default:
               strcat((char*)&u16_General_Domain[0],"Unknown");
            break;
      }
      strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 7: //Voltage
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[8]);
         for(u16_temp_index=0;u16_temp_index<5;u16_temp_index++)
         {
            if (s8_Power_Drive_Model_Number_2[u16_temp_index] == '-')
            {// find the '-' before the voltage
               // inc to the start of the voltage text
               u16_temp_index++;
               break;
            }
         }
         if ((s8_Power_Drive_Model_Number_2[u16_temp_index] >= ASCII_ZERO )&&(s8_Power_Drive_Model_Number_2[u16_temp_index] <= ASCII_NINE ))
         {
            u16_temp_str[0]=s8_Power_Drive_Model_Number_2[u16_temp_index];
            u16_temp_str[1]=0;
            strcat((char*)&u16_General_Domain[0],(char *)&u16_temp_str[0]);
            strcat((char*)&u16_General_Domain[0],"00 V");
         }
         else
         {// not numeric value
            strcat((char*)&u16_General_Domain[0],"Unknown");
         }
         strcat((char*)&u16_General_Domain[0],"\r\n");
         break;

      default:
         local_info_state--;
      break;
   }
   return local_info_state;
}


//**********************************************************
// Function Name: HornerTypeLabelParser
// Description:
//          This function is called from GlobalLabelParser
//          function for parsing Horner ("HE") Label.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int HornerTypeLabelParser(int local_info_state )
{
   unsigned int u16_temp_str[2];
   strcpy((char*)&u16_General_Domain[0],"\0");
   switch (local_info_state)
   {
      case 6: //Interface
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[7]);
       switch(s8_Control_Drive_Model_Number_3[0])
         {
          case 'C':
            strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[11]);
            strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[15]);
               break;

          case 'E':
            strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[3]);
            strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[15]);
               break;

          case 'P':
            strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[13]);
            strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[15]);

               break;

          case 'S':
            strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[10]);
               break;

         default:
            strcat((char*)&u16_General_Domain[0],"Unknown");
            break;
       }
            strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

      case 7: //Voltage
         strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[8]);
       if ((s8_Power_Drive_Model_Number_2[4] >= ASCII_ZERO )&&(s8_Power_Drive_Model_Number_2[4] <= ASCII_NINE ))
       {
           u16_temp_str[0]=s8_Power_Drive_Model_Number_2[4];
           u16_temp_str[1]=0;
           strcat((char*)&u16_General_Domain[0],(char *)&u16_temp_str[0]);
                       strcat((char*)&u16_General_Domain[0],"00 V");

       }
       else
       {// not numeric value
           strcat((char*)&u16_General_Domain[0],"Unknown");
       }
         strcat((char*)&u16_General_Domain[0],"\r\n");
      break;

     default:
        local_info_state--;
     break;
   }
   return local_info_state;

}


//**********************************************************

// Function Name: AkribisTypeLabelParser
// Description:
//          This function is called from GlobalLabelParser
//          function for parsing Akribis ("ASD") Label.
//
// Author: Moshe
// Algorithm:
// Revisions:

//**********************************************************
int AkribisTypeLabelParser(int local_info_state )
{
   strcpy((char*)&u16_General_Domain[0],"\0");
   switch (local_info_state)
   {
      case 6: //Interface
       strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[7]);
       switch(s8_Control_Drive_Model_Number_3[1])
       {
          case 'C':
               strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[11]);
               break;

          case 'E':
               strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[3]);
               break;

          case 'P':
               strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[13]);
               break;

          case 'S':
               strcat((char*)&u16_General_Domain[0],p_s8_Interface_Options[14]);
               break;

          default:
               strcat((char*)&u16_General_Domain[0],"Unknown");
               break;
       }

       strcat((char*)&u16_General_Domain[0],"\r\n");
       break;

      case 7: //Voltage
        strcat((char*)&u16_General_Domain[0],p_s8_Info_Command_Response[8]);
        switch(s8_Control_Drive_Model_Number_1[3])
        {
          case '2':
            strcat((char*)&u16_General_Domain[0],"240 V");
            break;

          case '4':
            strcat((char*)&u16_General_Domain[0],"480 V");
            break;

          default:
            strcat((char*)&u16_General_Domain[0],"Unknown");
            break;
       }

          strcat((char*)&u16_General_Domain[0],"\r\n");
          break;

     default:
           local_info_state--;
     break;

   }
   return local_info_state;

}


//**********************************************************
// Function Name: SalIlimCommand
// Description:
//          This function is called in response to the ILIM command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalIlimCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   if (lparam < 0LL) return (VALUE_TOO_LOW);
   if (lparam > (long long)BGVAR(s32_Imax)) return (VALUE_TOO_HIGH);

   BGVAR(s32_Ilim_User) = (long)lparam;

   if (VAR(AX0_s16_An_In_2_Mode) != 2) // if the chosen mode is not current limit
   {
       BGVAR(s32_Ilim_Actual) = BGVAR(s32_Ilim_User);
   }

   BGVAR(s32_Ilim_Active_Disable) = BGVAR(s32_Ilim_User);

   CalcILim(DRIVE_PARAM);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalIMaxHaltCommand
// Description:
//          This function is called in response to the P9-36_IMAXHALT command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalIMaxHaltCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL) return (VALUE_TOO_LOW);
   if (lparam > (long long)BGVAR(s32_Imax)) return (VALUE_TOO_HIGH);
   //Only when all the following occurs:
   //{
   // * The drive is at CAN Schneider state,
   // * SDO 605d content equals 3
   // * Halt occurs
   //}
   //then the function UpdateCurrentLimit loads the s
   BGVAR(s32_P1_70_IMAXHALT) = (long)lparam ;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalOutILevel1Command
// Description:
//          This function is called in response to the OUTILVL1 command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalOutILevel1Command(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s32_Out_I_Level_1) = (long)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalHomeIHardStopCommand
// Description:
//          This function is called in response to the HOMEIHARDSTOP command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalHomeIHardStopCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u32_Home_I_HardStop) = (long)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalOutILevel2Command
// Description:
//          This function is called in response to the OUTILVL2 command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalOutILevel2Command(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s32_Out_I_Level_2) = (long)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadHallsCommand
// Description:
//          This function is called in response to the HALLS command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalReadHallsCommand(int drive)
{
   int param;

   ReadHalls(&param, drive);
   PrintString("Hu Hv Hw", 0);
   PrintCrLf();
   if ((param & 0x0004) != 0) PrintString(" 1 ", 0);
   else PrintString(" 0 ", 0);
   if ((param & 0x0002) != 0) PrintString(" 1 ", 0);
   else PrintString(" 0 ", 0);
   if ((param & 0x0001) != 0) PrintString(" 1 ", 0);
   else PrintString(" 0 ", 0);
   PrintCrLf();

   return SAL_SUCCESS;
}


int ReadHalls(int *halls,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (FEEDBACK_TAMAGAWA && (VAR(AX0_u16_Tamagawa_Timer) == 0xFFFF))
   {
      *halls = (VAR(AX0_s16_Tamagawa_Halls) & 0x7);
   }
   else
   {
      *halls = (VAR(AX0_s16_Halls) & 0x7);
   }

   return SAL_SUCCESS;
}

// This is doing the actual setting of the VLIM
int VLimCommand(long long lparam, int drive, int *s16_config_needed)
{
   int result;
   long s32_original_v_lim_design_value;

   *s16_config_needed = 0;
   if (lparam < 89478LL) return (VALUE_TOO_LOW); // Minimum: 10 RPM
   if (lparam > (long long)BGVAR(s32_Vmax)) return (VALUE_TOO_HIGH);

   s32_original_v_lim_design_value = BGVAR(s32_V_Lim_Design);
   BGVAR(s32_V_Lim_Design) = (long)lparam;

   result = UpdateAfterVlim(DRIVE_PARAM);
   if (result != SAL_SUCCESS) return (result);

   if (BGVAR(s32_V_Lim_Design) != s32_original_v_lim_design_value)
      *s16_config_needed = 1;

   ConvertInternalToVel1000InLoop(drive);
   ConvertVelToInternalInLoop(drive);

   ConvertInternalToAccDec1000ForVel(drive);
   ConvertAccDecToInternalForVel(drive);

   ConvertInternalToMsAccDecVel(drive);
   ConvertMsAccDecVelToInternal(drive);

   //need to convert CanOpen conversion in the loop functions
   CalcFieldBusUserToIntern(drive);
   CalcInternToFieldBusUser(drive);
   
   ConvertFbCanOpenVelToInternalInLoop(drive);

   return (SAL_SUCCESS);
}

int SalSfbVLimCommand(long long lparam, int drive)
{
   int result = SAL_SUCCESS;
   BGVAR(s32_Sfb_V_Lim) = (long)lparam;

   UpdateFieldbusVelocityLimit(drive);
   
   return (result);
}

//**********************************************************
// Function Name: VLimCommand
// Description:
//          This function is called in response to the VLIM command.
//
//
// Author: Yuval
// Algorithm:
// Revisions: S.F: On SE this will implicitly call CONFIG to avoid No-Comp
//**********************************************************
int SalVLimCommand(long long lparam, int drive)
{
    static int s16_state = 0, s16_stored_ret_val;
    static long s32_prev_value = 0;
    int s16_ret_val = SAL_SUCCESS, s16_config_needed = 0;

    switch (s16_state)
    {
        case 0:
        {
            s32_prev_value = BGVAR(s32_V_Lim_Design);
            s16_ret_val = VLimCommand(lparam, drive, &s16_config_needed);

            if ((s16_ret_val == SAL_SUCCESS) &&
                (s16_config_needed))
            {
                //  set no-comp fault
                BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
            }
            break;
        }
        case 1:
        {
            // Using direct call to DesignRoutine to avoid Feedback re-configuration when not needed
            s16_ret_val = DesignRoutine(0, drive);
            if (s16_ret_val == SAL_SUCCESS)
            {
                s16_state = 0;
            }
            // If CONFIG failed - restore previous VLIM value
            else if (s16_ret_val != SAL_NOT_FINISHED)
            {
                // In config lock, ignore config failure.
                if (u16_Config_Lock == CONFIG_LOCK_WRITE)
                {
                    s16_state = 0;
                    return SAL_SUCCESS;
                }
                s16_stored_ret_val = s16_ret_val;
                VLimCommand((long long)s32_prev_value, drive, &s16_config_needed);
                s16_ret_val = SAL_NOT_FINISHED;
                s16_state = 2;
            }
            break;
        }
        case 2:
        {
            // Using direct call to DesignRoutine to avoid Feedback re-configuration when not needed
            s16_ret_val = DesignRoutine(0, drive);
            if (s16_ret_val != SAL_NOT_FINISHED)
            {
                s16_state = 0;
                if (s16_ret_val == SAL_SUCCESS)
                {
                    s16_ret_val = s16_stored_ret_val;
                }
            }
            break;
        }
    }

    return s16_ret_val;
}


//**********************************************************
// Function Name: SalOutVLevel1Command
// Description:
//          This function is called in response to the OUTVLVL1 command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************

int SalOutVLevel1Command(long long lparam, int drive)
{
   long long s64_out_v_level_lim = 0x66666667;   // 80% * 240,000{max 32bit presentation} * 2^32/ (8,000{counts per round} * 60{minutes})

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ( -s64_out_v_level_lim > lparam)
   {
      return (VALUE_TOO_LOW);
   }

   else if (s64_out_v_level_lim < lparam)
   {
      return (VALUE_TOO_HIGH);
   }

   else
      BGVAR(s32_Out_V_Level_1) = (long)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalOutVLevel2Command
// Description:
//          This function is called in response to the OUTVLVL2 command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalOutVLevel2Command(long long lparam, int drive)
{
   long long s64_out_v_level_lim = 0x66666667;   // 80% * 240,000{max 32bit presentation} * 2^32/ (8,000{counts per round} * 60{minutes})

   REFERENCE_TO_DRIVE;     // Defeat compilation error

    if ( -s64_out_v_level_lim > lparam)
   {
      return (VALUE_TOO_LOW);
   }

   else if (s64_out_v_level_lim < lparam)
   {
      return (VALUE_TOO_HIGH);
   }

   else
      BGVAR(s32_Out_V_Level_2) = (long)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalStatCommand
// Description:
//          This function is called in response to the VLIM command.
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int SalReadStatCommand(int* param, int drive)
{
   int result = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //bit 0 - DisableStatus  1 = drive is Disabled, 0 = drive is ENabled
   if (BGVAR(s16_DisableInputs)) result |= 0x0001;

   //bit 1 - FaultStatus    1 = fault exists, 0 = no fault exists
   if (BGVAR(s64_SysNotOk) || BGVAR(s64_SysNotOk_2)) result |= 0x0002;

   //bit 2 - SafetyStatus  1 = safety feature triggered/inactive, 0 = drive is safe
   // TBD

   //bit 3 - Special Mode Status   1 = Step, Burnin, or Zero is active, 0 = normal
    if (BGVAR(s8_BurninParam) || BGVAR(s16_Zero_Mode))  result |= 0x0008;

   //bit 4 - Hold Mode Status       1 = drive is in hold position,0 = drive is not in hold position mode
   if (BGVAR(u16_Hold)) result |= 0x0010;

   //bit 5 - WarningsStatus    1 = Warning exists, 0 = no Warning exists
   if (BGVAR(u64_Sys_Warnings)) result |= 0x0020;

   //bit 6 - RemarksStatus    1 = Remark exists, 0 = no Remark exists
   if (BGVAR(u16_Sys_Remarks)) result |= 0x0040;

    *param = result;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: PrintMessages
// Description:
//          This function is prints messages according to bit number in status variable
//       it prints one message per call
//       it uses MSG_LEN constant to find the message.
//       NOTE::: Use this function only with messages table that defined with MSG_LEN
//
//
// Author: D.R. / Gil
// Algorithm:
// Revisions:
//**********************************************************
int PrintMessages(unsigned long long u64_status, long ptr, int len, int print_prefix, unsigned int u16_offset)
{
   static int i = 0;
   unsigned long long u64_mask = 0;

   if ((u64_status >> i) == 0x00LL)
   {
      i = 0;
      return 1;
   }

   u64_mask = 0x0001LL << (long long)i;
   if ((u64_status & u64_mask) != 0LL)
   {
      strcat((char*)&u16_General_Domain[0],"   ");

      if (print_prefix)
      {
         if (print_prefix == 1) strcat((char*)&u16_General_Domain[0],"FLT ");
         if (print_prefix == 2) strcat((char*)&u16_General_Domain[0],"WRN ");
         if (print_prefix == 3) strcat((char*)&u16_General_Domain[0],"RMK ");
         strcat((char*)&u16_General_Domain[0],UnsignedIntegerToAscii((unsigned long )(i + 1 + u16_offset)));
         strcat((char*)&u16_General_Domain[0],"  ");
      }

      strcat((char*)&u16_General_Domain[0],(char*)(ptr + (i + 1 + u16_offset) * MSG_LEN));
      strcat((char*)&u16_General_Domain[0],"\r\n");
   }
   i++;

   if (i == len)
   {
      i = 0;
      return 1; //done
   }

   return 0; //not finished yet
}

//**********************************************************
// Function Name: StatCommand
// Description:
//          This function is called in response to the STAT command.
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int StatCommand(int drive)
{
   static int s16_temp;
   if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;

   SalReadStatCommand(&s16_temp, drive);
   PrintDecInAsciiHex((unsigned long)s16_temp, 4);
   PrintCrLf();

    return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: StatusCommand
// Description:
//          This function is called in response to the STATUS command.
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int StatusCommand(int drive)
{
    int u16_mode = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (u8_Output_Buffer_Free_Space < 100) return SAL_NOT_FINISHED;


   PrintDecInAsciiHex((unsigned long)BGVAR(s16_DisableInputs), 4); // word 1 - disable inputs
   PrintString(" ", 0);

   PrintDecInAsciiHex(BGVAR(s64_SysNotOk_2), 16); // long long word 2 - Faults - 64 bits MSB
   PrintString("-", 0);

   PrintDecInAsciiHex(BGVAR(s64_SysNotOk), 16); // long long word 2 - Faults - 64 bits LSB
   PrintString(" ", 0);

   PrintDecInAsciiHex(BGVAR(u64_Sys_Warnings), 16);  // long word 3 - Warnings Status Word
   PrintString(" ", 0);

   PrintDecInAsciiHex(BGVAR(u16_Sys_Remarks), 4);  // word 4 - Remarks Status Word
   PrintString(" ", 0);

   PrintDecInAsciiHex(0x0000L, 4); // word 5 - Safety Status Word, TBD - for now print 0000
   PrintString(" ", 0);

   if (BGVAR(s8_BurninParam)) u16_mode |= 0x0002;  // word 6 - Special Mode Status Word
   if (BGVAR(s16_Zero_Mode))  u16_mode |= 0x0004;
   PrintDecInAsciiHex(u16_mode, 4);
   PrintString(" ", 0);

   PrintDecInAsciiHex(0x0000L, 4); // word 7 - Hold Mode Status Word, TBD - for now print 0000
   PrintString(" ", 0);

   PrintCrLf();

   return (SAL_SUCCESS);
}



//**********************************************************
// Function Name: SalStCommand
// Description:
//          This function is calls StCommand from serial
//
// Author: D.R. / Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalStCommand(int drive)
{
   return (StCommand(drive,FB_NOT_IN_USE));
}

//**********************************************************
// Function Name: StCommand
// Description:
//          This function is called in response to the ST command
//          according to CAN / Serial
//
// Author: D.R. / Gil
// Algorithm:
// Revisions:
//**********************************************************
int StCommand(int drive,int s16_fb_use)
{
   // AXIS_OFF;
   static int status_state = 0;
   static unsigned int  u16_mode;
   long long s64_temp = 0LL;

   // in case use of the uart port
   if (s16_fb_use == FB_NOT_IN_USE)
   {
      if (OutputBufferLoader()==0) return SAL_NOT_FINISHED;
   }
   strcpy((char*)&u16_General_Domain[0],"\0");

   if (s16_Wd_State) //check if WD occured
   {
      strcat((char*)&u16_General_Domain[0],"WATCHDOG !!!!!\r\n");
      return SAL_SUCCESS;
   }

   switch (status_state)
   {
      case 0:
         if (Enabled(drive))
         {
            strcat((char*)&u16_General_Domain[0],"Drive Active\r\n");
            if (BGVAR(u16_AD_State) > 0)
            {
               if (BGVAR(u16_AD_State) >= AD_START_DISTIME_COUNTER_REGULAR)
                  strcat((char*)&u16_General_Domain[0],"Waiting for DISTIME to expire\r\n");
               else
                  strcat((char*)&u16_General_Domain[0],"Active Disable In Process\r\n");
            }
         }
         else
         {
            strcat((char*)&u16_General_Domain[0],"Drive Inactive\r\n");

            if (VAR(AX0_s16_Controller_Ptr) == (int)(0xffff & (long)&CURRENT_CONTROLLER_DB))  //changed to avoid Remark
               strcat((char*)&u16_General_Domain[0]," (Dynamic Brake)\r\n");
            else
               strcat((char*)&u16_General_Domain[0],"\r\n");

            strcat((char*)&u16_General_Domain[0],"Drive not ready:\r\n");
         }

         status_state++;
      break;

      case 1:
         if (PrintMessages((unsigned long long)BGVAR(s16_DisableInputs), (long)&u16_Disable_Message, 16, 0 ,0))
            status_state++;

      break;

      case 2:
         if (BGVAR(u16_Fold) != 0) strcat((char*)&u16_General_Domain[0],"Drive Foldback active\r\n");
         if (BGVAR(u16_Motor_Fold) != 0) strcat((char*)&u16_General_Domain[0],"Motor Foldback active\r\n");
         if ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_SERVICE_MASK) != 0) strcat((char*)&u16_General_Domain[0],"HDTune active\r\n");
         status_state++;
      break;

      case 3: // Specify the correct FPGA version
         if (u16_FPGA_ver_mismatch)
         {
            strcat((char*)&u16_General_Domain[0],"Needed FPGA Version: \r\n");
            strcat((char*)&u16_General_Domain[0],UnsignedIntegerToAscii((unsigned long)(u16_FPGA_Supported_Ver_Hi)));
            strcat((char*)&u16_General_Domain[0],".");
            if (u16_FPGA_Supported_Ver_Lo < 10) strcat((char*)&u16_General_Domain[0],"0");
            strcat((char*)&u16_General_Domain[0],UnsignedIntegerToAscii((unsigned long)(u16_FPGA_Supported_Ver_Lo)));
            strcat((char*)&u16_General_Domain[0],"\r\n");
         }
         status_state++;
      break;

      case 4:
         if (BGVAR(u16_Motor_Setup_State) > MOTOR_SETUP_IDLE)
            strcat((char*)&u16_General_Domain[0],"Motor Setup In Process\r\n");
         if (BGVAR(u16_Hold))
            strcat((char*)&u16_General_Domain[0],"Hold Mode Active\r\n");
         status_state++;
      break;

      case 5:
         /*
         if ( (BGVAR(s16_Param_Est_State) != PARAM_EST_STATE_IDLE) &&
              (BGVAR(s16_Param_Est_State) != PARAM_EST_STATE_DONE) &&
              (BGVAR(s16_Param_Est_State) != PARAM_EST_STATE_FAULT)  )
            strcat((char*)&u16_General_Domain[0],"Indetify Process Active\r\n");
         */

         if (BGVAR(u16_Est_Motor_Params))
            strcat((char*)&u16_General_Domain[0],"Motor Parameters Estimation Process Active\r\n");


         PhaseFindStatusCommand(&s64_temp,drive);
         if (s64_temp == 1LL) strcat((char*)&u16_General_Domain[0],"Phase Find Issued\r\n");

         if ((BGVAR(s64_SysNotOk) != 0x00LL) || (BGVAR(s64_SysNotOk_2) != 0x00LL))
            strcat((char*)&u16_General_Domain[0],"Fault exists:\r\n");
         status_state++;
      break;

      case 6:
         if (PrintMessages((unsigned long long)BGVAR(s64_SysNotOk), (long)&u16_Fault_Message, 64, 1 ,0))
            status_state++;
      break;

      case 7:
         if (PrintMessages((unsigned long long)BGVAR(s64_SysNotOk_2), (long)&u16_Fault_Message, 64, 1, 64))
         {
            strcat((char*)&u16_General_Domain[0],"\r\n");
            status_state++;
         }
      break;

      case 8:
         if (BGVAR(u64_Sys_Warnings) != 0x00LL) strcat((char*)&u16_General_Domain[0],"Warnings Exist:\r\n");
         status_state++;
      break;

      case 9:
         if (PrintMessages(BGVAR(u64_Sys_Warnings), (long)&u16_Warning_Message, 64, 2 ,0))
         {
            if (BGVAR(u64_Sys_Warnings) != 0x00LL) strcat((char*)&u16_General_Domain[0],"\r\n");
            status_state++;
         }
      break;

      case 10:
         if (BGVAR(u16_Sys_Remarks) != 0x00) strcat((char*)&u16_General_Domain[0],"Remarks Exist:\r\n");
         status_state++;
      break;

      case 11:
         if (PrintMessages((unsigned long long)BGVAR(u16_Sys_Remarks), (long)&u16_Remark_Message, 32, 3 ,0))
         {
            if (BGVAR(u16_Sys_Remarks) != 0x00) strcat((char*)&u16_General_Domain[0],"\r\n");
            status_state++;
         }
      break;

      case 12: // Specify the correct micro blaze version
         if (u16_UBLAZE_ver_mismatch)
         {
            strcat((char*)&u16_General_Domain[0],"Needed Fieldbus Version: ");
            /*if (IS_LXM28E_DRIVE)   // 2 * 64Mb flash
            {
               switch (u16_Fieldbus_Type)
               {
                  case FB_SERCOS:
                     strcat((char*)&u16_General_Domain[0],(char *)&s8_Micro_Blaze_Supported_Lxm28e_Sercos_Ver[0]);
                  break;

                  case FB_ETHERNET_IP:
                     strcat((char*)&u16_General_Domain[0],(char *)&s8_Micro_Blaze_Supported_Lxm28e_EthernetIP_Ver[0]);
                  break;

                  case FB_ETHERCAT:
                  default:
                     strcat((char*)&u16_General_Domain[0],(char *)&s8_Micro_Blaze_Supported_Lxm28e_EtherCAT_Ver[0]);
                  break;
               }
            }
            else*/
            {
               strcat((char*)&u16_General_Domain[0],(char *)&s8_Micro_Blaze_Supported_Ver[0]);
            }
            strcat((char*)&u16_General_Domain[0],"\r\n");
         }
         status_state++;
      break;
      case 13: // Specify the correct ESI version
         if (u16_ESI_ver_mismatch)
         {
            strcat((char*)&u16_General_Domain[0],"Needed ESI Version: ");
            strcat((char*)&u16_General_Domain[0],DecToAsciiHex(u32_ESI_Supported_Ver,8));
            strcat((char*)&u16_General_Domain[0],"\r\n");
         }
         status_state++;
      break;

      case 14:
         u16_mode = 0;
         if (BGVAR(s8_BurninParam)) u16_mode |= 0x0002;
         if (BGVAR(s16_Zero_Mode))  u16_mode |= 0x0004;
         if (u16_mode != 0 ) strcat((char*)&u16_General_Domain[0],"Motion inhibited:\r\n");
         status_state++;

      case 15:
        if (PrintMessages((unsigned long long)u16_mode, (long)&u16_Motion_Inhibited_Message, 16, 0 ,0))
        {
           if (u16_mode != 0 )strcat((char*)&u16_General_Domain[0],"\r\n");
           status_state = 0;
           if (s16_fb_use == FB_NOT_IN_USE)
           {
              OutputBufferLoader();
           }
        }

      break;
   }

   if (status_state == 0)
      return (SAL_SUCCESS);
   else
      return (SAL_NOT_FINISHED);
}


//**********************************************************
// Function Name: PrintCurrentFaults
// Description:
//          This function is called in response to the FLT command.
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int PrintCurrentFaults(int drive)
{
   static int i = 0;
   long long s64_mask;
   unsigned int print_flt_msg=1;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (u8_Output_Buffer_Free_Space < 60) return SAL_NOT_FINISHED;

   if (s16_Number_Of_Parameters==1) print_flt_msg = (unsigned int)s64_Execution_Parameter[0];

   if ((0 == i) && (0x000LL == BGVAR(s64_SysNotOk)) && (0x000LL == BGVAR(s64_SysNotOk_2))) // check if active faults exist
   {
      PrintStringCrLf((char*)u16_Fault_Message[i], 0);
      return (SAL_SUCCESS);
   }

   if (i<64)
   {
      s64_mask = 0x0001LL << i;

      if ((BGVAR(s64_SysNotOk) & s64_mask) != 0)
      {
         PrintString("FLT ", 0);
         if (print_flt_msg == 1)
         {
            PrintUnsignedInteger(i + 1);
            PrintString("  ", 0);
            PrintStringCrLf((char*)u16_Fault_Message[i + 1], 0);
         }
         else
         {
            PrintDecInAsciiHex(i + 1,2);
            PrintCrLf();
         }
      }

      if (BGVAR(s64_SysNotOk) == 0LL) i=64;
   }

   if ( (i >= 64) && (i < 128) )
   {
      s64_mask = 0x0001LL << (i-64);

      if ((BGVAR(s64_SysNotOk_2) & s64_mask) != 0)
      {
         PrintString("FLT ", 0);
         if (print_flt_msg == 1)
         {
            PrintUnsignedInteger(i + 1);
            PrintString("  ", 0);
            PrintStringCrLf((char*)u16_Fault_Message[i + 1], 0);
         }
         else
         {
            PrintDecInAsciiHex(i + 1, 2);
            PrintCrLf();
         }
      }

      if (BGVAR(s64_SysNotOk_2) == 0LL) i = 127;
   }

   i++;

   if (i == 128) i = 0;

   if (i == 0) return (SAL_SUCCESS);
   else return (SAL_NOT_FINISHED);
}


//**********************************************************
// Function Name: PrintCANCurrentFaults
// Description:
//          This function is called in response to the FLT command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int PrintCANCurrentFaults(int drive)
{
   static int i = 0;
   long long s64_mask;
   unsigned int print_flt_msg=1;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   strcpy((char*)&u16_General_Domain[0],"\r\n");

   if ((0 == i) && (0x000LL == BGVAR(s64_SysNotOk))) // check if active faults exist
   {
      strcat((char*)&u16_General_Domain[0],"\r\n");
      return (SAL_SUCCESS);
   }

   s64_mask = 0x0001LL << i;

   if ((BGVAR(s64_SysNotOk) & s64_mask) != 0)
   {
      strcat((char*)&u16_General_Domain[0],"FLT ");
      if (print_flt_msg == 1)
      {
         strcat((char*)&u16_General_Domain[0],UnsignedIntegerToAscii(i+1));
         strcat((char*)&u16_General_Domain[0],"  ");
         strcat((char*)&u16_General_Domain[0],(char*)u16_Fault_Message[i + 1]);
         strcat((char*)&u16_General_Domain[0],"\r\n");
      }
      else
      {
         strcat((char*)&u16_General_Domain[0],DecToAsciiHex(i + 1,2));
         strcat((char*)&u16_General_Domain[0],"\r\n");
      }
   }
   i++;

   if (i == 63)
      i = 0;

   if (i == 0)
      return (SAL_SUCCESS);
   else return (SAL_NOT_FINISHED);
}

//**********************************************************
// Function Name: PrintCurrentWarnings
// Description:
//          This function is called in response to the WRN command.
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int PrintCurrentWarnings(int drive)
{
   static int i = 0;
   unsigned long long u64_mask = 0LL;
   unsigned int print_wrn_msg=1;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (u8_Output_Buffer_Free_Space < 60) return SAL_NOT_FINISHED;

   if (s16_Number_Of_Parameters==1) print_wrn_msg = (unsigned int)s64_Execution_Parameter[0];

   if ((0 == i) && (0x0000 == BGVAR(u64_Sys_Warnings))) // check if active warnings exist
   {
      PrintStringCrLf((char*)u16_Warning_Message[i], 0);
      strcpy((char*)manu_spec_Print_Current_Warnings_command, (const char*)u16_Warning_Message[i]);
      strcat((char*)manu_spec_Print_Current_Warnings_command,"\0");
      return (SAL_SUCCESS);
   }

   u64_mask = 0x0000000000000001LL << i;
   if ((BGVAR(u64_Sys_Warnings) & u64_mask) != 0)
   {
      PrintString("WRN ", 0);
      strcpy((char*)manu_spec_Print_Current_Warnings_command, "WRN ");
      if (print_wrn_msg == 1)
      {
         PrintUnsignedInteger(i + 1);
         strcat((char*)manu_spec_Print_Current_Warnings_command,UnsignedIntegerToAscii(i+1) );
         PrintString("  ", 0);
         strcat((char*)manu_spec_Print_Current_Warnings_command, "  ");
         PrintStringCrLf((char*)u16_Warning_Message[i + 1], 0);
         strcat((char*)manu_spec_Print_Current_Warnings_command, (const char*)u16_Warning_Message[i+1]);
         strcat((char*)manu_spec_Print_Current_Warnings_command,"\n");
      }
      else
      {
         PrintDecInAsciiHex(i + 1,2);
         PrintCrLf();
      }
   }
   i++;
   if (i == 64) i = 0;

   if (i == 0)
   {
      strcat((char*)manu_spec_Print_Current_Warnings_command,"\0");
      return (SAL_SUCCESS);
   }
   else
      return (SAL_NOT_FINISHED);
}


//**********************************************************
// Function Name: SalPfbBackupModeCommand
// Description:
//       This function is called in response to the PFBBACKUPMODE command.
//
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SalPfbBackupModeCommand(long long param,int drive)
{
   unsigned int u16_temp = (unsigned int)param;

   if (((u16_Support_Axis_1_On_Flash == 0) && (drive != 0)) && (u16_temp != 0)) return (NOT_SUPPORTED_ON_HW);

   if ((u16_temp != 0) &&  (BGVAR(u16_Pfb_Backup_Mode) == 0))
   {
      if (u16_Flash_Type == 1)          // 8 4KW bottom blocks, and 32KW blocks all the rest
      {
         if (EraseBlock(AX0_FLASH_TYPE_1_PFB_OFF_BLOCK) != FLASH_DONE) return (SAL_NOT_FINISHED);
      }
      else if (u16_Vendor_ID[0] == 0x0001) // Spansion Manufacturer's ID is 0x01, SST's is 0xBF
      { // Spansion uses Block (32KWord) Arrangement
         return (NOT_SUPPORTED_ON_HW);  // not supported on Spansion 16Mb or 32Mb
      }
      else
      { // SST uses Sector (2KWord) Arrangement
         if (EraseSector(AX0_PFB_OFF_SECTOR + (6 * drive)) != FLASH_DONE) return (SAL_NOT_FINISHED);
      }
   }
   BGVAR(u16_Pfb_Backup_Mode) = u16_temp;
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalPfbOffCommand
// Description:
//       This function is called in response to the SFBOFF command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalSfbOffCommand(long long param,int drive)
{
   AXIS_OFF;
   if ((!s16_Password_Flag) && (Enabled(drive))) return (DRIVE_ACTIVE);

   if ( ((BGVAR(u16_Units_Pos_Rotary) ==  ROT_COUNTS_UNITS) && (ROTARY_MOTOR == LOAD_TYPE)) ||
        ((BGVAR(u16_Units_Pos_Linear) ==  LIN_COUNTS_UNITS) && (LINEAR_MOTOR == LOAD_TYPE))   )
   {
     if (BGVAR(u16_Is_Not_Integer_Flag))
        return VALUE_MUST_BE_INTEGER;
   }

   LVAR(AX0_u32_Sfb_Pos_Fdbk_Offset_Lo) = (unsigned long)(param & 0x00000000FFFFFFFF);
   LVAR(AX0_s32_Sfb_Pos_Fdbk_Offset_Hi) = (long)(param >> 32LL);

// after a jump of pfb, the roll over modulation calculation does not work anymore
// it is important to do the calculation from scratch again
   if ((BGVAR(s64_Modulo_Range) != 0) && (BGVAR(u16_Modulo_Mode) > 0))
     VAR(AX0_u16_Position_Modulo_Active) = 1;
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalPfbOffCommand
// Description:
//       This function is called in response to the PFBOFF command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalPfbOffCommand(long long param,int drive)
{
    // AXIS_OFF;
    if ((!s16_Password_Flag) && (Enabled(drive)))
    {
        return (DRIVE_ACTIVE);
    }

    if ((((BGVAR(u16_Units_Pos_Rotary) ==  ROT_COUNTS_UNITS) && (ROTARY_MOTOR == BGVAR(u16_MotorType))) ||
         ((BGVAR(u16_Units_Pos_Linear) ==  LIN_COUNTS_UNITS) && (LINEAR_MOTOR == BGVAR(u16_MotorType)))) &&
        (BGVAR(u16_Is_Not_Integer_Flag)))
    {
        return VALUE_MUST_BE_INTEGER;
    }

    LVAR(AX0_u32_Pos_Fdbk_Offset_Lo) = (unsigned long)(param & 0x00000000FFFFFFFF);
    LVAR(AX0_s32_Pos_Fdbk_Offset_Hi) = (long)(param >> 32LL);

    //  after a jump of pfb, the roll over modulation calculation does not work anymore
    //  it is important to do the calculation from scratch again
    //  Initialize the Modulo Mode State Machine
    InitializeModuloModeStateMachine(drive);

    return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalPeInposCommand
// Description:
//       This function is called in response to the PEINPOS command.
//
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int SalPeInposCommand(long long param, int drive)
{
   unsigned int u16_temp_time;
    // AXIS_OFF;

//    unsigned long long u64_peinpos_in_user;
   REFERENCE_TO_DRIVE;
   if ((unsigned long long)(param) > 0x7FFFFFFF00000001)
   {
       return (VALUE_TOO_HIGH);
      }
// removed the check because it causes problem in high resulotions
/*    if (BGVAR(u16_Units_Pos_Rotary) == ROT_COUNTS_UNITS)
    {
        //move to user units in order to verify the value is integer for counts
        u64_peinpos_in_user = MultS64ByFixU64ToS64(param,
                                        BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                                        BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);

        if ((u64_peinpos_in_user % 1000) != 0)
        {
            return VALUE_MUST_BE_INTEGER;
        }
    }
*/
   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Inpos_Threshold_Lo) = param;
   } while (u16_temp_time != Cntr_3125);

    return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalSavedHomeOfstCommand
// Description:
//       This function is called in response to the REFOFFSETVAL command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalSavedHomeOfstCommand(long long param, int drive)
{
   unsigned int u16_temp_time;
    // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Home_Offset_User_Lo) = param;

      // This updates the operational homing offset with the same value as user variable
      // regardless if the feedback is multi turn absolute
      LLVAR(AX0_u32_Home_Offset_Lo) = param;
   } while (u16_temp_time != Cntr_3125);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalPERRead
// Description:
//          This function is called in response P1-54 (PER) read command.
//
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPERRead(long long *data, int drive)
{
   long long s64_temp;
   int s16_temp;

   // AXIS_OFF;

   //convert from CDHD internal position units into PUU.
   REFERENCE_TO_DRIVE;
   do {
      s16_temp = Cntr_3125;
      s64_temp = LLVAR(AX0_u32_Inpos_Threshold_Lo);
   } while (s16_temp != Cntr_3125);
   *data = MultS64ByFixS64ToS64(s64_temp,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalPERWrite
// Description:
//          This function is called in response P1-54 (PER) write command.
//
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPERWrite(long long param,int drive)
{
  if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return NOT_AVAILABLE;

  BGVAR(u32_P1_54_PER_User_Value) = (unsigned long)param;
  Recalculate_PER_PUU_Units(drive);
  return SAL_SUCCESS;
}


//**********************************************************
// Function Name: Recalculate_PER_PUU_Units
// Description:
//          This function is recalculating the new P1-54(PER)value according to the new gear ratio.
//
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void Recalculate_PER_PUU_Units(int drive)
{
   // AXIS_OFF;
   int s16_temp;
   long long s64_temp;
   REFERENCE_TO_DRIVE;
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   //convert from PUU into CDHD internal position units and set it in the POSLIMPOS variable.
   s64_temp = MultS64ByFixS64ToS64((long long)BGVAR(u32_P1_54_PER_User_Value),
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr);

   do {
      s16_temp = Cntr_3125;
      LLVAR(AX0_u32_Inpos_Threshold_Lo) = s64_temp;
   } while (s16_temp != Cntr_3125);
}


//**********************************************************
// Function Name: MotorCommTypeCommand
// Description:
//          This function is called in response to the MOTORCOMMTYPE command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorCommTypeCommand(long long param, int drive)
{
   // AXIS_OFF;

   //support only DC_VCM and BRUSHLESS motors
   REFERENCE_TO_DRIVE;
   if (VAR(AX0_u16_Motor_Comm_Type) == (int)param) return (SAL_SUCCESS);

   VAR(AX0_u16_Motor_Comm_Type) = (int)param;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: MotorTypeCommand
// Description:
//          This function is called in response to the MOTORTYPE command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorTypeCommand(long long lparam, int drive)
{
   // AXIS_OFF;

   if (!(u16_Fw_Features & MOTORTYPE_LIMIT_MASK)) //MOTORTYPE validation
      if (ROTARY_MOTOR != lparam)
         return (VALUE_IS_NOT_ALLOWED);

   //support only linear and rotary motors
   if ((lparam != ROTARY_MOTOR) && (lparam != LINEAR_MOTOR)) return (VALUE_OUT_OF_RANGE);

   if (BGVAR(u16_MotorType) == (unsigned int)lparam) return (SAL_SUCCESS);

   BGVAR(u16_MotorType) = (unsigned int)lparam;

   //set units according to motor type
   if (BGVAR(u16_MotorType) == LINEAR_MOTOR)// linear motor
   {
      SalUnitsVelLinearCommand((long long)BGVAR(u16_Units_Vel_Linear), drive);
      SalUnitsAccDecLinearCommand((long long)BGVAR(u16_Units_Acc_Dec_Linear), drive);
      SalUnitsPosLinearCommand((long long)BGVAR(u16_Units_Pos_Linear), drive);
   }
   else// rotary motor or any other motor
   {
      SalUnitsVelRotaryCommand((long long)BGVAR(u16_Units_Vel_Rotary), drive);
      SalUnitsAccDecRotaryCommand((long long)BGVAR(u16_Units_Acc_Dec_Rotary), drive);
      SalUnitsPosRotaryCommand((long long)BGVAR(u16_Units_Pos_Rotary), drive);
   }

   //handle commands that have only one unit type (one for linear and one for rotary)
   UnitsVelRotLinRPMCommand(drive);
   UnitsVelRotLinRPSCommand(drive);
   UnitsPosRotLinCommand(drive);
   UnitsRotLinKPPCommand(drive);
   UnitsRotLinENCRESCommand(drive);
   UnitsAmpVelRotLinCommand(drive);
   UnitsMechAngleRotLinCommand(drive);

   PositionModuloConfig(drive);

   VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;   //reset enc state machine

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMpitchCommand
// Description:
//          This function is called in response to the MPITCH command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMpitchCommand(long long lparam, int drive)
{
   if (BGVAR(u32_Mpitch) == (unsigned long)lparam)
      return (SAL_SUCCESS);

   // Dont allow to set a value other than default if on rotary drive
   if (!(u16_Fw_Features & MPITCH_LIMIT_MASK))
   {
      return (NOT_SUPPORTED_ON_HW);
   }

   BGVAR(u32_Mpitch) = (unsigned long)lparam;

   SalUnitsVelLinearCommand((long long)BGVAR(u16_Units_Vel_Linear), drive);
   SalUnitsAccDecLinearCommand((long long)BGVAR(u16_Units_Acc_Dec_Linear), drive);
   SalUnitsPosLinearCommand((long long)BGVAR(u16_Units_Pos_Linear), drive);

   UnitsPosRotLinCommand(drive);
   UnitsVelRotLinRPSCommand(drive);
   UnitsVelRotLinRPMCommand(drive);

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   //make update - TBD

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: EmberCommand
// Description:
//          This function is called in response to the EMBER command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int EmberCommand(void)
{
   void (*ResidentFw)(void) ;

   if (u16_Scan_Device_State != I2C_IDLE_STATE) return SAL_NOT_FINISHED;

   ResidentFw=(fptr_void)0x100000; //address of Resident FW

   if (Enabled(0))
      return (DRIVE_ACTIVE);

   if (u8_Is_Retro_Display == 0)
   {
      // Write to a specific memory location as indicator to Resident that we jumped from Drive, because
      // for in box programming the FPGA is not loaded (to reduce power consumption)
      //u32_Stamp_From_Ember = 0xAC1389A5;

      // Show FLOAd on ember
      HWDisplay(F_LET, 4);
      HWDisplay(L_LET, 3);
      HWDisplay(ZERO,  2);
      HWDisplay(A_LET, 1);
      HWDisplay(d_LET,  0);

   }
   else
   {
      HWDisplay(E_LET, 4);
   }

   EALLOW;
   SysCtrlRegs.WDCR= 0x0068; // Disable watchdog module
   DINT; // Disable and clear all CPU interrupts:
   IER = 0x0000;
   IFR = 0x0000;
   EDIS;

   REGEN_RELAY_OFF;

   if ( (IS_EC_DRIVE) || (IS_PN_DRIVE) )     // Take control on the SPI signals to be able to access the serial flash from resident.
   {
      InitSpiaGpio();
   }

   // Write to FPGA Register as Indicator to Resident that we jumped from Drive
   *((int*)FPGA_TEST_REG_ADD) = (0x00A5 | (Rotary_Switch << 8));
   *((int*)FPGA_WATCH_DOG_ENABLE_REG_ADD) = 0x0; // disable FPGA watchdog

   ResidentFw(); //go to Resident FW

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: ResidentCommand
// Description:
//          This function is called in response to the RESIDENT command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int ResidentCommand(void)
{
   int (*ResidentFw)(void); //address of Resident FW

   if (u16_Scan_Device_State != I2C_IDLE_STATE) return SAL_NOT_FINISHED;

   ResidentFw=(fptr_int)0x100000;

   if (Enabled(0))
      return (DRIVE_ACTIVE);

   if (IS_RETRO_DISPLAY)
   {
      // Write to a specific memory location as indicator to Resident that we jumped from Drive, because
      // for in box programming the FPGA is not loaded (to reduce power consumption)
      //u32_Stamp_From_Ember = 0xAC1389A5;

      // Show FLOAd on ember
      HWDisplay(F_LET, 4);
      HWDisplay(L_LET, 3);
      HWDisplay(ZERO,  2);
      HWDisplay(A_LET, 1);
      HWDisplay(d_LET,  0);
   }
   else
   {
      HWDisplay(r_LET, 4);
   }

   EALLOW;
   SysCtrlRegs.WDCR= 0x0068; // Disable watchdog module
   DINT; // Disable and clear all CPU interrupts:
   IER = 0x0000;
   IFR = 0x0000;
   EDIS;

   REGEN_RELAY_OFF;

   if ( (IS_EC_DRIVE) || (IS_PN_DRIVE) )     // Take control on the SPI signals to be able to access the serial flash from resident.
   {
      InitSpiaGpio();
   }

   // Write to FPGA Register as Indicator to Resident that we jumped from Drive, but NOT intending
   *((int*)FPGA_TEST_REG_ADD) = (0x005A | (Rotary_Switch << 8)); // to load to Flash Memory
   *((int*)FPGA_WATCH_DOG_ENABLE_REG_ADD) = 0x0; // disable FPGA watchdog

   ResidentFw(); //go to Resident FW

   return SAL_SUCCESS;
}



//**********************************************************
// Function Name: RestartDriveFirmware
// Description:
//          This function is called to restart the drive firmware.
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int RestartDriveFirmware(void)
{
   void (*DriveFw)(void) ;

   if (u16_Scan_Device_State != I2C_IDLE_STATE) return SAL_NOT_FINISHED;

   DriveFw=(fptr_void)0x158000; //address of Drive FW
   if (Enabled(0))
      return (DRIVE_ACTIVE);


   *((int*)FPGA_WATCH_DOG_ENABLE_REG_ADD) = 0x0; // disable FPGA watchdog


   EALLOW;
   SysCtrlRegs.WDCR= 0x0068; // Disable watchdog module
   DINT; // Disable and clear all CPU interrupts:
   IER = 0x0000;
   IFR = 0x0000;
   EDIS;

   REGEN_RELAY_OFF;

   DriveFw(); //go to Resident FW

   return SAL_SUCCESS; //This should never happend
}



//**********************************************************
// Function Name: SalResBwCommand
// Description:
//          This function is called in response to the RESBW command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalResBwCommand(long long lparam,int drive)
{
   // AXIS_OFF;

   BGVAR(u16_ResBw) = (int)lparam;
   CalcSwr2dCoef(drive);

   VAR(AX0_s16_Crrnt_Run_Code) |= RESET_SWR2D_MASK;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalOutOfRangeCommand
// Description:
//          This function is called in response to the RESAMPLRANGE command.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalOutOfRangeCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   BGVAR(u16_Out_of_Range_Percent) = (int)lparam;

   // Maintain Out-of-Range Limits based on Sine Peak and Cosine Peak normalized to 0x6666 = 26,214,
   // thus Sin^2 + Cos^2 = 0x28F570A4.  Use only Upper 16-Bits for nominal value 0x28F5.
   VAR(AX0_s16_Out_Of_Range_High) = (int)((1.0 + BGVAR(u16_Out_of_Range_Percent) / 100.0) * 10485.0);
   VAR(AX0_s16_Out_Of_Range_Low) = (int)((1.0 - BGVAR(u16_Out_of_Range_Percent) / 100.0) * 10485.0);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalUnitsVelRotaryCommand
// Description:
//          This function is called in response to the UNITSROTVEL command.
//          u16_Units_Vel_Rotary = 0  => rps
//                            1  => rpm
//                            2  => deg per sec
//                            3  => user per sec
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalUnitsVelRotaryCommand(long long lparam, int drive)
{
   BGVAR(u16_Units_Vel_Rotary) = (int)lparam;

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      //set the variable units string
      switch (BGVAR(u16_Units_Vel_Rotary))
      {
         case RPS_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop), "rps");
            strcpy(BGVAR(s8_Units_Vel_In_Loop), "rps");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale), "rps/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp), "rps");
         break;
         case RPM_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop), "rpm");
            strcpy(BGVAR(s8_Units_Vel_In_Loop), "rpm");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale), "rpm/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp), "rpm");
         break;
         case DEG_PER_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop), "deg/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop), "deg/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale), "(deg/s)/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp), "deg/s");
         break;
         case USER_PER_SEC_ROT_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop), "USER/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop), "USER/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale), "(USER/s)/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp), "USER/s");
         break;
      }
   }
   
   if (LOAD_TYPE == ROTARY_MOTOR)
   {
      //set the variable units string
      switch (BGVAR(u16_Units_Vel_Rotary))
      {
         case RPS_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "rps");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "rps");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "rps/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp_Sfb), "rps");
         break;
         case RPM_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "rpm");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "rpm");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "rpm/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp_Sfb), "rpm");
         break;
         case DEG_PER_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "deg/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "deg/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "(deg/s)/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp_Sfb), "deg/s");
         break;
         case USER_PER_SEC_ROT_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "USER/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "USER/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "(USER/s)/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp_Sfb), "USER/s");
         break;
      }
   }
   
   ConvertInternalToVel1000OutLoop(drive);
   ConvertVelToInternalOutLoop(drive);

   ConvertInternalToVel1000InLoop(drive);
   ConvertVelToInternalInLoop(drive);
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalUnitsPosRotaryCommand
// Description:
//          This function is called in response to the UNITSROTPOS command.
//          u16_Units_Pos_Rotary = 0  => rev
//                            1  => Counts
//                            2  => deg
//                            3  => user
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalUnitsPosRotaryCommand(long long lparam, int drive)
{
   BGVAR(u16_Units_Pos_Rotary) = (int)lparam;

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      //set the variable units string
      switch (BGVAR(u16_Units_Pos_Rotary))
      {
         case REV_UNITS:
            strcpy(BGVAR(s8_Units_Pos), "rev");
         break;
         case ROT_COUNTS_UNITS:
            strcpy(BGVAR(s8_Units_Pos), "counts");
         break;
         case DEG_UNITS:
            strcpy(BGVAR(s8_Units_Pos), "deg");
         break;
         case USER_ROT_UNITS:
            strcpy(BGVAR(s8_Units_Pos), "USER");
         break;
      }      
   }
   
   if (LOAD_TYPE == ROTARY_MOTOR)
   {
      //set the variable units string
      switch (BGVAR(u16_Units_Pos_Rotary))
      {
         case REV_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "rev");
         break;
         case ROT_COUNTS_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "counts");
         break;
         case DEG_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "deg");
         break;
         case USER_ROT_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "USER");
         break;
      }
   }

   ConvertInternalToPos1000(drive);
   ConvertPosToInternal(drive);
   ConvertCountsToInternal(drive);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalUnitsAccDecRotaryCommand
// Description:
//          This function is called in response to the UNITSROTACC command.
//          u16_Units_Vel_Rotary = 0  => rps/s
//                            1  => rpm/s
//                            2  => deg per sec^2
//                            3  => user per sec^2
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalUnitsAccDecRotaryCommand(long long lparam, int drive)
{
   BGVAR(u16_Units_Acc_Dec_Rotary) = (int)lparam;

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      //set the variable units string
      switch (BGVAR(u16_Units_Acc_Dec_Rotary))
      {
         case RPS_TO_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "rps/s");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos), "rps/s");
         break;
         case RPM_TO_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "rpm/s");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos), "rpm/s");
         break;
         case DEG_PER_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "deg/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos), "deg/s^2");
         break;
         case USER_PER_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "USER/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos), "USER/s^2");
         break;
      }
   }
   
   if (LOAD_TYPE == ROTARY_MOTOR)
   {
      //set the variable units string
      switch (BGVAR(u16_Units_Acc_Dec_Rotary))
      {
         case RPS_TO_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel_Sfb), "rps/s");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "rps/s");
         break;
         case RPM_TO_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel_Sfb), "rpm/s");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "rpm/s");
         break;
         case DEG_PER_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel_Sfb), "deg/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "deg/s^2");
         break;
         case USER_PER_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel_Sfb), "USER/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "USER/s^2");
         break;
      }    
   }
   
   ConvertInternalToAccDec1000ForVel(drive);
   ConvertAccDecToInternalForVel(drive);
   
   ConvertInternalToAccDec1000ForPos(drive);
   ConvertAccDecToInternalForPos(drive);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalUnitsVelLinearCommand
// Description:
//          This function is called in response to the UNITSLINVEL command.
//          u16_Units_Vel_Linear = 0  => Micrometers per sec.
//                                 1  => Millimeters per second.
//                                 2  => user per sec.
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalUnitsVelLinearCommand(long long lparam, int drive)
{
   if (lparam == UM_PER_SEC_UNITS) return (VALUE_OUT_OF_RANGE);
   BGVAR(u16_Units_Vel_Linear) = (int)lparam;

  if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
  {
         //set the variable units string
         switch (BGVAR(u16_Units_Vel_Linear))
         {
            case UM_PER_SEC_UNITS:
               strcpy(BGVAR(s8_Units_Vel_Out_Loop), "um/s");
               strcpy(BGVAR(s8_Units_Vel_In_Loop), "um/s");
               strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale), "(um/s)/V");
            break;
            case MM_PER_SEC_UNITS:
               strcpy(BGVAR(s8_Units_Vel_Out_Loop), "mm/s");
               strcpy(BGVAR(s8_Units_Vel_In_Loop), "mm/s");
               strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale), "(mm/s)/V");
            break;
            case USER_PER_SEC_LIN_UNITS:
               strcpy(BGVAR(s8_Units_Vel_Out_Loop), "USER/s");
               strcpy(BGVAR(s8_Units_Vel_In_Loop), "USER/s");
               strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale), "(USER/s)/V");
            break;
         }
   }  
  
   if (LOAD_TYPE == LINEAR_MOTOR)
   {
      switch (BGVAR(u16_Units_Vel_Linear))
      {
         case UM_PER_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "um/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "um/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "(um/s)/V");
         break;
         case MM_PER_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "mm/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "mm/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "(mm/s)/V");
         break;
         case USER_PER_SEC_LIN_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "USER/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "USER/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "(USER/s)/V");
         break;
      }
   }
   ConvertInternalToVel1000OutLoop(drive);
   ConvertVelToInternalOutLoop(drive);

   ConvertInternalToVel1000InLoop(drive);
   ConvertVelToInternalInLoop(drive);
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalUnitsPosLinearCommand
// Description:
//          This function is called in response to the UNITSLINPOS command.
//          u16_Units_Pos_Linear = 0  => Pitch
//                         1  => Counts
//                                 2  => �m
//                                 3  => mm
//                                 4  => user
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalUnitsPosLinearCommand(long long lparam, int drive)
{
   if (lparam == UM_UNITS) return (VALUE_OUT_OF_RANGE);

   BGVAR(u16_Units_Pos_Linear) = (int)lparam;

   if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
   {
      //set the variable units string
      switch (BGVAR(u16_Units_Pos_Linear))
      {
         case PITCH_UNITS:
            strcpy(BGVAR(s8_Units_Pos), "Pitch");
         break;
         case LIN_COUNTS_UNITS:
            strcpy(BGVAR(s8_Units_Pos), "counts");
         break;
         case UM_UNITS:
            strcpy(BGVAR(s8_Units_Pos), "um");
         break;
         case MM_UNITS:
            strcpy(BGVAR(s8_Units_Pos), "mm");
         break;
         case USER_LIN_UNITS:
            strcpy(BGVAR(s8_Units_Pos), "USER");
         break;
      }
   }
   
   if (LOAD_TYPE == LINEAR_MOTOR)
   {
      //set the variable units string
      switch (BGVAR(u16_Units_Pos_Linear))
      {
         case PITCH_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "Pitch");
         break;
         case LIN_COUNTS_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "counts");
         break;
         case UM_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "um");
         break;
         case MM_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "mm");
         break;
         case USER_LIN_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "USER");
         break;
      }
   }   

   ConvertInternalToPos1000(drive);
   ConvertPosToInternal(drive);
   ConvertCountsToInternal(drive);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalUnitsAccDecLinearCommand
// Description:
//          This function is called in response to the UNITSLINACC command.
//          u16_Units_Vel_Linear = 0  => �m/sec^2
//                                 1  => mm/sec^2
//                                 2  => user/sec^2
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalUnitsAccDecLinearCommand(long long lparam, int drive)
{
   if (lparam == UM_TO_SEC_2_POWER_UNITS) return (VALUE_OUT_OF_RANGE);

   BGVAR(u16_Units_Acc_Dec_Linear) = (int)lparam;

   if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
   {
      //set the variable units string
      switch (BGVAR(u16_Units_Acc_Dec_Linear))
      {
         case UM_TO_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "um/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos), "um/s^2");
         break;
         case MM_TO_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "mm/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos), "mm/s^2");
         break;
         case USER_TO_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "USER/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos), "USER/s^2");
         break;
      }
   } 
   
   if (LOAD_TYPE == LINEAR_MOTOR)
   {
      //set the variable units string
      switch (BGVAR(u16_Units_Acc_Dec_Linear))
      {
         case UM_TO_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "um/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "um/s^2");
         break;
         case MM_TO_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "mm/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "mm/s^2");
         break;
         case USER_TO_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "USER/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "USER/s^2");
         break;
      }
   }  

      ConvertInternalToAccDec1000ForVel(drive);
      ConvertAccDecToInternalForVel(drive);

      ConvertInternalToAccDec1000ForPos(drive);
      ConvertAccDecToInternalForPos(drive);
      
   return (SAL_SUCCESS);
}


int SalTestLedCommand(void)
{
   int i = 0;
   //if retro or CDHD1
   if (IS_RETRO_DISPLAY == 1 || (!IS_CDHD2_DRIVE))
   {
      if (!s16_Number_Of_Parameters)
      {
         if (Test_Led_On == 0)
            Test_Led_On = 1;
      }
      else
      {
         if (s64_Execution_Parameter[0] == -1LL)
         {// if parameter == -1 run new display test sequance else (print to screen 7-seg binary value)
            // run test only if dislay test is idle
            if (Test_Led_On == 0)
               Test_Led_On = 4;
         }
         else
         {
            PrintUnsignedInt16(u16_Seven_Segment_Value);
            PrintCrLf();
         }
      }
   }
   else if ( u8_Is_Retro_Display == 0 )//CDHD2 HMI
   {
      if (!s16_Number_Of_Parameters)
      {
         u8_Test_Hmi = 1;
      }
      else
      {
         if (s64_Execution_Parameter[0] == -1LL)
         {
            u8_Test_Hmi_Seq = 1;
         }
         else
         {
            for (i = CDHD_HMI_SIZE - 1; i >= 0; --i )
            {
               PrintUnsignedInt16(BGVAR(u16_Local_Hmi_Display_Array[i]));
               PrintChar(' ');
            }
            PrintCrLf();
         }
      }   
   }
   return (SAL_SUCCESS);
}


int SalPasswordCommand(void)
{
   int u16_dynamicPass = 0;
   if (!s16_Number_Of_Parameters)
   {
      if (s16_Password_Flag)
         PrintString("Password Protection Inactive", 0);
      else
         PrintString("Password Protection Active", 0);

      PrintCrLf();
   }
   else
   {
      u16_dynamicPass = ((((u16_Dynamic_Pass + 1) << 4) & 0xff) + 0xa);
      if (s64_Execution_Parameter[0] == 0xe110)
      {
         s16_Password_Flag = 1;
         PrintString("Password Protection Inactive", 0);
         PrintCrLf();
      }

      else if ((s16_Password_Flag) && (u16_dynamicPass == ((int)s64_Execution_Parameter[0])))
         s16_Password_Flag = 2;
      else
         s16_Password_Flag = 0;
   }
   return (SAL_SUCCESS);
}

// Index duration.
// Address 0x4047:
///Bit0 => select between regular index and time extended index where: 0 � regular index, 1 � time extended index
///Bit1 => stop index output �1� : write �1� to this bit at idle and �0� when you want index to output �0�.
///Bit2 => read index output.
#pragma CODE_SECTION(IndexDuration, "ramfunc_4");
void IndexDuration(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_Index_Duration) ==0) return;

   switch (BGVAR(u16_Index_FSM_State))
   {
      case 0:
         if ((*((unsigned int*)FPGA_INDEX_TIME_EXTENSION_REG_ADD) & 0x04))
         {
            *((unsigned int*)FPGA_INDEX_TIME_EXTENSION_REG_ADD) |= 0x02;
            BGVAR(u16_Index_FSM_State) = 1;
            BGVAR(u32_Index_Event_Time) = Cntr_1mS;
         }
      break;

      case 1:
         if (PassedTimeMS((long)(BGVAR(u16_Index_Duration)), BGVAR(u32_Index_Event_Time)) == 1)
         {
            *((unsigned int*)FPGA_INDEX_TIME_EXTENSION_REG_ADD) &= 0xFD;
            BGVAR(u16_Index_FSM_State) = 0;
         }
      break;
   }

   return;
}

//**********************************************************
// Function Name: SalSetRunawayVelocityThresholdCommand
// Description:
//    This function is called upon a RAVTHRESH ASCII-command.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalSetRunawayVelocityThresholdCommand(long long param,int drive)
{
   static int s16_state = 0, s16_stored_ret_val;
   static unsigned long u32_prev_value = 0;
   long long u64_max_valid_value = 0x7FFFFFFF; // default max value for CDHD
   int s16_ret_val = SAL_SUCCESS;
   REFERENCE_TO_DRIVE;

   switch (s16_state)
   {
      case 0:
         // On SE max value can be 6000 rpm
         if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
         {// max valid value can be 6000 rpm
            u64_max_valid_value = 0x3333333;
         }

         if (param > u64_max_valid_value) return (VALUE_TOO_HIGH);
         if (param < 0) return (VALUE_TOO_LOW);

         u32_prev_value = BGVAR(u32_Runaway_Check_Vel_Thresh);
         BGVAR(u32_Runaway_Check_Vel_Thresh) = param;
         // On SE avoid using CONFIG by running it implicitly
         if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
         {
            s16_ret_val = SAL_NOT_FINISHED;
            s16_state = 1;
         }
         else /* set no-comp fault */
            BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;

      break;

      case 1:
//         s16_ret_val = SalConfigCommand(drive);
         // Using direct call to DesignRoutine to avoid Feedback re-configuration when not needed
         s16_ret_val = DesignRoutine(0, drive);
         if (s16_ret_val == SAL_SUCCESS) s16_state = 0;
         else if (s16_ret_val != SAL_NOT_FINISHED) // If CONFIG failed - restore previous RAVTHRESH value
         {
             // In config lock, ignore config failure.
              if (u16_Config_Lock == CONFIG_LOCK_WRITE)
              {
                  s16_state = 0;
                  return SAL_SUCCESS;
              }
            s16_stored_ret_val = s16_ret_val;
            BGVAR(u32_Runaway_Check_Vel_Thresh) = u32_prev_value;
            s16_ret_val = SAL_NOT_FINISHED;
            s16_state = 2;
         }
      break;

      case 2:
//         s16_ret_val = SalConfigCommand(drive);
         // Using direct call to DesignRoutine to avoid Feedback re-configuration when not needed
         s16_ret_val = DesignRoutine(0, drive);
         if (s16_ret_val != SAL_NOT_FINISHED)
         {
            s16_state = 0;
            if (s16_ret_val == SAL_SUCCESS)
               s16_ret_val = s16_stored_ret_val;
         }
      break;
   }

   return s16_ret_val;
}

//**********************************************************
// Function Name: RunawayDetection
// Description:
//    This function checks if there is a runaway condition pending.
//    A fault bit is set after x consecutive error conditions.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(RunawayDetection, "ramfunc_4");
void RunawayDetection(drive)
{
   // AXIS_OFF;

   long s32_Actual_Acceleration = LVAR(AX0_s32_Vel_Var_Fb_0) - BGVAR(s32_Previous_Actual_Velocity); // Calculate the acceleration
   BGVAR(s32_Previous_Actual_Velocity) = LVAR(AX0_s32_Vel_Var_Fb_0);

   // Dont monitor this fault on disable
   if (!Enabled(DRIVE_PARAM))
   {
      BGVAR(u16_Runaway_Error_Counter) = 0; // Clear error counter
      return;
   }

   // Start checking the fault condition for the runaway-situation in case that:
   //  1) The runaway-detection feature has been activated by setting the parameter RATTHRESH to a value larger 0.
   //  2) The actual velocity has reached a certain value (a fault situation can easily happen when moving very
   //     slow due to noise on the actual current and/or actual velocity)
   //  3) The actual current is above a certain threshold (12.5% of the currently applied current-limit) since
   //     in a runaway situation the current increases very fast in the windings. We use the applied current-limit
   //     and no fixed value since a user may unintentionally switch off the feature by lowering ILIM to a very low value.
   if ( (BGVAR(u16_Runaway_Error_Counter_Thresh) > 0) &&
       (labs(LVAR(AX0_s32_Vel_Var_Fb_0)) > BGVAR(u32_Runaway_Check_Vel_Thresh)) &&
       (abs(VAR(AX0_s16_Crrnt_Q_Act_0)) > (BGVAR(u16_Runaway_Check_Curr_Thresh)>>4))
     )
   {
      /* Check for fault condition:                                                             */
      /* --------------------------                                                             */
      /*  1) If the actual acceleration and the actual current have opposite algebraic signs.   */
      /*  2) If the actual velocity is actually moving in opposite direction. This additional   */
      /*     if-condition is added since the error-condition can also appear depending on the   */
      /*     loop and load-properties while running continuously in one direction. So the       */
      /*     error will only be detected when the motor really moves to the opposite direction. */
      if (
           ((VAR(AX0_s16_Crrnt_Q_Act_0)>0) && (s32_Actual_Acceleration<0) && (LVAR(AX0_s32_Vel_Var_Fb_0)<0)) ||
           ((VAR(AX0_s16_Crrnt_Q_Act_0)<0) && (s32_Actual_Acceleration>0) && (LVAR(AX0_s32_Vel_Var_Fb_0)>0))
        )
      {
         if (BGVAR(u16_Runaway_Error_Counter) > BGVAR(u16_Runaway_Error_Counter_Thresh)) // x-times consecutively in an error-condition
         {
            // Indicate the the error is supposed to be generated
            BGVAR(u16_Runaway_Error_Detected) = 1;
         }
         else if (BGVAR(u8_CDHD_Normal_Operation)) // Only if the Drive is in normal operation
         {
            BGVAR(u16_Runaway_Error_Counter)++; // Increment error counter
            // For debug purposes in order to see how much the counter increments in normal operation.
            // With this variable the user can analyze the error counter in normal operation and adjust
            // the threshold afterwards to a reasonable value.
            if (BGVAR(u16_Runaway_Error_Counter) > BGVAR(u16_Max_Runaway_Error_Counter))
            {
               BGVAR(u16_Max_Runaway_Error_Counter) = BGVAR(u16_Runaway_Error_Counter);
            }
         }
      }
      else
      {
         BGVAR(u16_Runaway_Error_Counter) = 0; // Clear error counter
      }
   }
   else
   {
      BGVAR(u16_Runaway_Error_Counter) = 0; // Clear error counter
   }
}


// Check if value is same as MENCRES/SFBENCRES - if so, the handling of the encoder sim will be done by
// The FPGA, otherwise the FW will handle it.
void EncoderSimSetup(int drive)
{
   long s32_Abs_Enc_Sim_Res;
   int s16_temp;
   unsigned int u16_fdbk_type = 0, u16_fdbk_enc_res = 0, u16_is_fdbk_with_index = 0, u16_is_fdbk_abs_single_turn = 0, u16_fpga_encoutmode2_eeo_mode_add = 0;
   unsigned long u32_counts_per_rev = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (IS_DUAL_LOOP_ACTIVE)
   {
      u16_fdbk_type = BGVAR(u16_SFBType);
      u16_fdbk_enc_res = BGVAR(u32_User_Sec_Enc_Res);
      u16_is_fdbk_with_index = SFB_WITH_INDEX;
      u16_is_fdbk_abs_single_turn = SFB_ABS_SINGLE_TURN;
      u32_counts_per_rev = LVAR(AX0_u32_Counts_Per_Rev_2); 
      u16_fpga_encoutmode2_eeo_mode_add = EEO_SFB_INDEX_VIA_FPGA_BIT; //0x08;
      VAR(AX0_s16_DF_Run_Code_2) |= DUAL_LOOP_ENTER_ENCODER_SIMULATION_MASK;
      VAR(AX0_s16_DF_Run_Code) &= ~DUAL_LOOP_ENTER_ENCODER_SIMULATION_MASK;
   }
   else
   {
      u16_fdbk_type = BGVAR(u16_FdbkType);
      u16_fdbk_enc_res = BGVAR(u32_User_Motor_Enc_Res);
      u16_is_fdbk_with_index = FEEDBACK_WITH_INDEX;
      u16_is_fdbk_abs_single_turn = FEEDBACK_ABS_SINGLE_TURN;
      u32_counts_per_rev = LVAR(AX0_u32_Counts_Per_Rev); 
      u16_fpga_encoutmode2_eeo_mode_add = EEO_MFB_INDEX_VIA_FPGA_BIT;//0x04;
      VAR(AX0_s16_DF_Run_Code)   |= DUAL_LOOP_ENTER_ENCODER_SIMULATION_MASK;
      VAR(AX0_s16_DF_Run_Code_2) &= ~DUAL_LOOP_ENTER_ENCODER_SIMULATION_MASK;
   }

   if (BGVAR(u8_EncoutMode) == 0) // Inhibit encoder simulation
   {
      VAR(AX0_u16_Encsim_Freq_Flags) |= INHIBIT_ENC_SIM_MASK | INHIBIT_ENC_SIM_PFB_NOT_READY_MASK;
      VAR(AX0_u16_Encsim_Freq_Flags) &= ~SKIP_ENC_SIM_DIR_MASK;

      // Inhibit the enc sim on the FPGA
      *(int*)FPGA_EMULATED_ENCODER_MODE_ADD = 0;
      *(int*)FPGA_ENC_SIM_SPACE_PRE_LOAD_ADD = 100;
      *(int*)FPGA_ENC_SIM_NUM_OF_COUNTS_ADD = 0;
      *(int*)FPGA_ZPOS_CONTROL_REG_ADD = 0;

      BGVAR(s64_Faults_Mask) &= ~ENC_SIM_FREQ_FLT_MASK;
   }
   else
   {
      // This ensures there will be no overrun fault on the initial activation of the encoder sim
      VAR(AX0_u16_Encsim_Freq_Flags) = (INHIBIT_ENC_SIM_MASK | SKIP_FLT_CHECK_MASK);
      // to handle the case in which s32_Enc_Sim_Res is negative
      s32_Abs_Enc_Sim_Res = BGVAR(s32_Enc_Sim_Res);
      if (s32_Abs_Enc_Sim_Res < 0)
         s32_Abs_Enc_Sim_Res = -s32_Abs_Enc_Sim_Res;

      // If enc sim resolution equals the encoder res, let the hardware control the output
      if ( (s32_Abs_Enc_Sim_Res == u16_fdbk_enc_res) &&
           ( (u16_fdbk_type == INC_ENC_FDBK))/*                ||
             (BGVAR(u16_FdbkType) == SW_SINE_FDBK)*/              )    // Need to understand from Baruch why isnt the FPGA outputing the pulses in sine encoder case
      {
         *(int*)FPGA_EMULATED_ENCODER_MODE_ADD = 2;
         VAR(AX0_u16_Encsim_Freq_Flags) |= SKIP_ENC_SIM_DIR_MASK;

         if (BGVAR(s32_Enc_Sim_Res) == u16_fdbk_enc_res)
            *(int*)(FPGA_ENC_SIM_DIR_PRE_LOAD_ADD) = 0;
         else
            *(int*)(FPGA_ENC_SIM_DIR_PRE_LOAD_ADD) = 2;              
      }
      // If enc sim resolution equals the encoder res with interpolation, let the hardware control the output
      else if ( (u16_fdbk_type == INC_ENC_FDBK)                        &&
                (s32_Abs_Enc_Sim_Res == (((long)u32_counts_per_rev) >> 2L))  )
      {
         VAR(AX0_u16_Encsim_Freq_Flags) |= SKIP_ENC_SIM_DIR_MASK;

         *(int*)FPGA_EMULATED_ENCODER_MODE_ADD = 3;

         if (BGVAR(s32_Enc_Sim_Res) == (((long)u32_counts_per_rev) >> 2L))
            *(int*)(FPGA_ENC_SIM_DIR_PRE_LOAD_ADD) = 0;
         else
            *(int*)(FPGA_ENC_SIM_DIR_PRE_LOAD_ADD) = 2;
      }
      else // DSP controls the encoder simulation
      {
         *(int*)FPGA_EMULATED_ENCODER_MODE_ADD = 1;

            do
            {
               s16_temp = Cntr_3125;
               LVAR(AX0_u32_Index_Pos_For_Encsim) = 0L;
            } while (s16_temp != Cntr_3125);

         // Logic is as follows:
         // Feedback with index: use the real index to for the encoder simulation index alignment
         // Feedback without index, absolute in one turn: no need to init the alignment as the raw position data sufficient (index will output at pfb single turn rollover)
         // Feedback without index, not absolute in one turn: Init according to electrical angle rollover
         if (u16_is_fdbk_with_index)
            VAR(AX0_u16_Encsim_ZPos_Offset_Flags) |= ENCSIM_INDEX_BY_REAL_INDEX_MASK; // This will initiate encout zero position according to the real index
         else
         {
            if (u16_is_fdbk_abs_single_turn)
               VAR(AX0_u16_Encsim_ZPos_Offset_Flags) |= ENCSIM_INDEX_JUST_ALIGNED_MASK;
            else
               VAR(AX0_u16_Encsim_ZPos_Offset_Flags) |= ENCSIM_INDEX_BY_ELECTANGLE_ROLLOVER_MASK; // This will initiate encout zero position according to elect-angle rollover
         }
      }

      // On ENCOUTMODE=2 route the index directly from the feedback (done by the FPGA)
      *(int*)FPGA_EMULATED_ENCODER_MODE_ADD &= ~(EEO_MFB_INDEX_VIA_FPGA_BIT | EEO_SFB_INDEX_VIA_FPGA_BIT);
      if (BGVAR(u8_EncoutMode) == 2)
          *(int*)FPGA_EMULATED_ENCODER_MODE_ADD |= u16_fpga_encoutmode2_eeo_mode_add;

      VAR(AX0_u16_Encsim_Freq_Flags) |= END_INHIBIT_ENC_SIM_MASK;
      VAR(AX0_u16_Encsim_Freq_Flags) &= ~INHIBIT_ENC_SIM_MASK;

      BGVAR(s64_Faults_Mask) |= ENC_SIM_FREQ_FLT_MASK;
   }
}

int EncoderSimulationConfig(int drive)
{
   int s16_ret_val = SAL_SUCCESS;
   long s32_temp_enc_sim_res = BGVAR(s32_Enc_Sim_Res);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Test if encoder simulation freq is not too high
   // Max freq by HW is 16M counts/sec
   // Check if 1.25*VLIM[rpm]*ENCOUTRES*4/60 <= 16M which means: 1.25 * s32_V_Lim_Design * 8000/2^32 * s32_Enc_Sim_Res*4 <= 16M
   //
   if (0 > s32_temp_enc_sim_res)
      s32_temp_enc_sim_res = -s32_temp_enc_sim_res; //handles negative ENCOUTRES values

   if (BGVAR(u8_EncoutMode) != 0)
   {
      if (((float)BGVAR(s32_V_Lim_Design) * (float)s32_temp_enc_sim_res / 107374.1824) > 16000000.5)
         s16_ret_val = CONFIG_FAIL_ENCOUTRES;
   }

   return s16_ret_val;
}

int EncoderSimResCommand(long long param, int drive)
{
   // AXIS_OFF;

   float f_temp;
   int u16_enc_res_chagned = 0;

   if (param == 0LL) return VALUE_IS_NOT_ALLOWED;

   if (BGVAR(s32_Enc_Sim_Res) != (long)param)
   {
      u16_enc_res_chagned = 1;
   }

   BGVAR(s32_Enc_Sim_Res) = (long)param;

   VAR(AX0_u16_Encsim_Freq_Flags) |= INHIBIT_ENC_SIM_MASK;

   // Encoder simulation design
   f_temp = (float)BGVAR(s32_Enc_Sim_Res) / (float)0x40000000; //32bit myltiply by 4,cuz lines.

   if (BGVAR(s32_Enc_Sim_Res) >= 0L)
   {
      FloatToFixS32Shift16(&LVAR(AX0_s32_Encsim_Fix) , (unsigned int *)&VAR(AX0_u16_Encsim_Shr), f_temp);
      LVAR(AX0_u32_Encsim_Counts_Per_Rev) = BGVAR(s32_Enc_Sim_Res) << 2;

      if (BGVAR(s32_Enc_Sim_Res) >= 300L) LVAR(AX0_u32_Encsim_Index_Window) = 400;
      else LVAR(AX0_u32_Encsim_Index_Window) = BGVAR(s32_Enc_Sim_Res);
   }
   else
   {
      FloatToFixS32Shift16(&LVAR(AX0_s32_Encsim_Fix) , (unsigned int *)&VAR(AX0_u16_Encsim_Shr), -f_temp);
      LVAR(AX0_s32_Encsim_Fix) = -LVAR(AX0_s32_Encsim_Fix);
      LVAR(AX0_u32_Encsim_Counts_Per_Rev) = (-BGVAR(s32_Enc_Sim_Res)) << 2;

      if (BGVAR(s32_Enc_Sim_Res) <= -300L) LVAR(AX0_u32_Encsim_Index_Window) = 400;
      else LVAR(AX0_u32_Encsim_Index_Window) = -BGVAR(s32_Enc_Sim_Res);
   }

   EncoderSimSetup(drive);

   if (u16_enc_res_chagned)
   {
      // If index was already captured, just set a bit indicating that a unit conversion should be made
      if (VAR(AX0_u16_Encsim_ZPos_Offset_Flags) == ENCSIM_INDEX_ALIGNED_MASK) VAR(AX0_u16_Encsim_ZPos_Offset_Flags) = ENCSIM_INDEX_JUST_ALIGNED_MASK;
   }

   return (SAL_SUCCESS);
}

// S.F: On SE this will implicitly call CONFIG to avoid No-Comp
int SalEncoderSimResCommand(long long param, int drive)
{
   static int s16_state = 0, s16_stored_ret_val;
   static long s32_prev_value = 0;
   int s16_ret_val = SAL_SUCCESS;

   if (param == 0LL) return VALUE_IS_NOT_ALLOWED;

   // range is checked here and not in excel file due to LXM28 SQA (for high vlim the actual range is lower and specified as NA in the excel)
   if (param < -10000000LL) return VALUE_TOO_LOW;
   if (param > 10000000LL) return VALUE_TOO_HIGH;

   switch (s16_state)
   {
      case 0:
         s32_prev_value = BGVAR(s32_Enc_Sim_Res);
         if (BGVAR(s32_Enc_Sim_Res) != (long)param)
         {
            // On SE avoid using CONFIG by running it implicitly
            if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
            {
               s16_ret_val = SAL_NOT_FINISHED;
               s16_state = 1;
            }
            else /* set no-comp fault */
               BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
         }

         EncoderSimResCommand(param, drive);
      break;

      case 1:
//         s16_ret_val = SalConfigCommand(drive);
         // Using direct call to DesignRoutine for simplicity
         s16_ret_val = DesignRoutine(1, drive);
         if (s16_ret_val == SAL_SUCCESS) s16_state = 0;
         else if (s16_ret_val != SAL_NOT_FINISHED) // If CONFIG failed - restore previous VLIM value
         {
            s16_stored_ret_val = s16_ret_val;
            EncoderSimResCommand((long long)s32_prev_value, drive);
            s16_ret_val = SAL_NOT_FINISHED;
            s16_state = 2;
         }
      break;

      case 2:
//         s16_ret_val = SalConfigCommand(drive);
         // Using direct call to DesignRoutine for simplicity
         s16_ret_val = DesignRoutine(1, drive);
         if (s16_ret_val != SAL_NOT_FINISHED)
         {
            s16_state = 0;
            if (s16_ret_val == SAL_SUCCESS)
               s16_ret_val = s16_stored_ret_val;
         }
      break;
   }

   return s16_ret_val;
}

int SalWritePulseOutputPolaritySettingCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // test AOUT bit 0 - analog out 2 polarity.
   if ((param & 0x01) != (BGVAR(u16_P1_03_AOUT) & 0x01)) //AOUT analog out 2 field "A" bit0 was "1" and changed to "0" or vice versa.
   {
       BGVAR(s16_AOUT2_Polarity) = -BGVAR(s16_AOUT2_Polarity);
   }

   // test AOUT bit 1 - analog out 1 polarity.
   if ((param & 0x02) != (BGVAR(u16_P1_03_AOUT) & 0x02)) //AOUT analog out 1 field "A" bit1 was "1" and changed to "0" or vice versa.
   {
       BGVAR(s16_AOUT1_Polarity) = -BGVAR(s16_AOUT1_Polarity);
   }

   // test AOUT bit 4 - ENCOUTRES (pulse output) polarity
   if ((param & 0x010) != (BGVAR(u16_P1_03_AOUT) & 0x010)) //AOUT B field was "1" and changed to "0" or vice versa
   {// need to reverse ENCOUTRES polarity
       BGVAR(s32_Enc_Sim_Res) = -BGVAR(s32_Enc_Sim_Res);
   }
   BGVAR(u16_P1_03_AOUT) = (unsigned int)param;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalIndexDurationTimeCommand
// Description:
//          This function is called in response to the INDEXDURATE command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalIndexDurationTimeCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_Index_Duration) = (unsigned int)param;

   if (*((unsigned int*)FPGA_INDEX_TIME_EXTENSION_REG_ADD) & 0x4)
   {
      *((unsigned int*)FPGA_INDEX_TIME_EXTENSION_REG_ADD) |=0x2;
      *((unsigned int*)FPGA_INDEX_TIME_EXTENSION_REG_ADD) |=0x2;
      *((unsigned int*)FPGA_INDEX_TIME_EXTENSION_REG_ADD) |=0x2;
      *((unsigned int*)FPGA_INDEX_TIME_EXTENSION_REG_ADD) &= 0xFD;
   }

   //In case there's use of the index duration control
   if (param)
      *((unsigned int*)FPGA_INDEX_TIME_EXTENSION_REG_ADD) |=1;
   else
      *((unsigned int*)FPGA_INDEX_TIME_EXTENSION_REG_ADD) &= 0xFE;

   return (SAL_SUCCESS);
}

int SalEncoderSimMode(long long param, int drive)
{
  if ((!IS_HW_FUNC_ENABLED(EEO_MASK)) && (0LL != param)) return NOT_SUPPORTED_ON_HW;

   if (BGVAR(u8_EncoutMode) != (int)param)
      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;

   BGVAR(u8_EncoutMode) = (int)param;

   EncoderSimSetup(drive);

   return (SAL_SUCCESS);
}


int SalReadMotorName(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   PrintChar(DOUBLE_QUOTE);
   PrintString(BGVAR(u8_MotorName), 0);
   PrintCrLf();

   return SAL_SUCCESS;
}


int SalReadDriveName(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   PrintChar(DOUBLE_QUOTE);
   PrintString(BGVAR(u8_DriveName), 0);
   PrintCrLf();

   return SAL_SUCCESS;
}


int SalSetMotorName(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // If MTP is used (MTPMODE == 1) || (MTPMODE == 3) then allow only clearing Motor-Name, but not setting it
   // to any other String
   if ( ( (BGVAR(u16_MTP_Mode) == 1) || (BGVAR(u16_MTP_Mode) == 3) ) && (u8_Execution_String[0][0] != 0) )
   {
      if (strncmp(BGVAR(u8_MotorName), u8_Execution_String[0], 20) != 0) return (MTP_USED);
      else                                                               return SAL_SUCCESS;
   }

   strncpy(BGVAR(u8_MotorName), u8_Execution_String[0], 20); // Copy no more than 20 characters...

   return SAL_SUCCESS;
}


int SalSetDriveName(int drive)
{
   int i = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   do {
      BGVAR(u8_DriveName)[i] = u8_Execution_String[0][i];
      i++;
   } while ((u8_Execution_String[0][i]!=0) && (i < (21 - 1)));
   BGVAR(u8_DriveName)[i] = 0;

   return SAL_SUCCESS;
}


int SalMotorResPolesCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((lparam % 2) != 0) return (ARGUMENT_NOT_EVEN);

   if (BGVAR(u16_Resolver_Poles) == (int)lparam) return (SAL_SUCCESS);

   BGVAR(u16_Resolver_Poles) = (int)lparam;

   // set no-comp fault
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalMotorKtCommand
// Description:
//          This function is called in response to the MKT command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorKtCommand(long long lparam, int drive)
{
   if (!(u16_Fw_Features & MKT_LIMIT_MASK))
      if (3000LL < lparam)
         return VALUE_TOO_HIGH;

   if (BGVAR(u32_Motor_Kt) == (unsigned long)lparam) return (SAL_SUCCESS);

   BGVAR(u32_Motor_Kt) = (unsigned long)lparam;

   UpdateVelocityLimits(DRIVE_PARAM);
   ConvertFbCanOpenTorque(drive);

//   BGVAR(u32_fb_motor_rated_torque) = (unsigned long)(BGVAR(s32_Motor_I_Cont) * BGVAR(u32_Motor_Kt) / 1000);


   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMotorKfCommand
// Description:
//          This function is called in response to the MKF command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorKfCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u32_Motor_Kf) == (unsigned long)lparam) return (SAL_SUCCESS);
   
   if (!(u16_Fw_Features & MKF_LIMIT_MASK))
      return (NOT_SUPPORTED_ON_HW);   

   BGVAR(u32_Motor_Kf) = (unsigned long)lparam;

   UpdateVelocityLimits(DRIVE_PARAM);

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalAnin1OffsetCommand
// Description:
//          This function is called in response to the ANIN1OFFSET command.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalAnin1OffsetCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (lparam > 26214) return (VALUE_TOO_HIGH);
   if (lparam < -26214) return (VALUE_TOO_LOW);

   BGVAR(s16_An_In_1_Offset_User_Setting) = (int)lparam;
   VAR(AX0_s16_An_In_1_Offset) = (int)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalAnin2OffsetCommand
// Description:
//          This function is called in response to the ANIN2OFFSET command.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalAnin2OffsetCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (lparam > 26214) return (VALUE_TOO_HIGH);
   if (lparam < -26214) return (VALUE_TOO_LOW);

   BGVAR(s16_An_In_2_Offset_User_Setting) = (int)lparam;
   VAR(AX0_s16_An_In_2_Offset) = (int)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalAnin2ModeCommand
// Description:
//          This function is called in response to the ANIN2MODE command.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalAnin2ModeCommand(long long lparam, int drive)
{
   // AXIS_OFF;

   if ((u16_Product == DDHD) && ((int)lparam != 0))
   {
      return (NOT_ALLOWED_IN_DDHD);
   }

   if (IS_HW_FUNC_ENABLED(INTERNAL_DUAL_GAIN_MASK))
   { // if internal dual gain exist value must be -1
      if ((int)lparam != -1)
      {
            return (INTERNAL_DUAL_GAIN_EXISTS);
      }
   }

   else //internal dual gain does not exist
   {
      if ((int)lparam == -1)
      { // internal dual gain does not exist - user can not set value to be internal dual gain
         return (INTERNAL_DUAL_GAIN_NOT_EXIST);
      }
      else
      {
         VAR(AX0_s16_An_In_2_Mode) = (int)lparam;

         //if the mode is not current limit set the limit to max
         if (VAR(AX0_s16_An_In_2_Mode) != 2)
         {
            VAR(AX0_u16_Analog_Crrnt_Limit) = 26214;
            BGVAR(s32_Ilim_Actual) = BGVAR(s32_Ilim_User);
            CalcILim(DRIVE_PARAM);
         }
      }
   }

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalAnin2User
// Description:
//          This function is called in response to the ANIN2USER command.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalAnin2User(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Here convert the analog input voltage into user-unit defined by the customer.
   // Note that the variable "BGVAR(s16_Voltage_Correct_2_Result)" holds either the
   // value of "VAR(AX0_s16_An_In_2_Filtered)" or a coltage-corrected value of
   // "VAR(AX0_s16_An_In_2_Filtered)", depending on if the voltage correction feature
   // is active or not. This command was requested by the customer Frencken, therefore
   // we need to use the variable "BGVAR(s16_Voltage_Correct_2_Result)".
   *data = (((long long)BGVAR(s16_Voltage_Correct_2_Result) * (long long)BGVAR(s32_Anin2_To_User_Fix)) >> BGVAR(u16_Anin2_To_User_Shr)) + (long long)BGVAR(s32_Anin2_User_Offset);

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalAnin2UserNumCommand
// Description:
//          This function is called in response to the ANIN2USERNUM command.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalAnin2UserNumCommand(long long lparam, int drive)
{
   BGVAR(s32_Anin2_User_Num) = (long)lparam;
   ConversionAninToUserUnitsPreperation(drive);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalAnin2UserDenCommand
// Description:
//          This function is called in response to the ANIN2USERDEN command.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalAnin2UserDenCommand(long long lparam, int drive)
{
   BGVAR(s32_Anin2_User_Den) = (long)lparam;
   ConversionAninToUserUnitsPreperation(drive);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalAnin1DeadbandCommand
// Description:
//          This function is called in response to the ANIN1DB command.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalAnin1DeadbandCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (lparam > 26214) return (VALUE_TOO_HIGH);
   if (lparam < 0) return (VALUE_TOO_LOW);

   VAR(AX0_u16_An_In_1_Deadband) = (unsigned int)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalAnin2DeadbandCommand
// Description:
//          This function is called in response to the ANIN2DB command.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalAnin2DeadbandCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (lparam > 26214) return (VALUE_TOO_HIGH);
   if (lparam < 0) return (VALUE_TOO_LOW);

   VAR(AX0_u16_An_In_2_Deadband) = (unsigned int)lparam;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMessageCommand
// Description:
//          This function is called in response to the MSG command.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalMessageCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_Message) = (unsigned int)lparam;

   if (BGVAR(u16_Message) == 1) // enable messages and propt
   {
      BGVAR(u8_Comms_Options) |= PROMPT_MASK;
      BGVAR(u8_Comms_Options) |= ERR_MSG_MASK;
   }

   if (BGVAR(u16_Message) == 0) // disable messages and propt
   {
      BGVAR(u8_Comms_Options) &= ~PROMPT_MASK;
      BGVAR(u8_Comms_Options) &= ~ERR_MSG_MASK;
   }

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalEchoCommand
// Description:
//          This function is called in response to the ECHO command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalEchoCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_Echo) = (unsigned int)lparam;

   if (BGVAR(u16_Echo) == 1) // enable echo
      BGVAR(u8_Comms_Options) |= ECHO_CONTROL_MASK;

   if (BGVAR(u16_Echo) == 0) // disable echo
      BGVAR(u8_Comms_Options) &= ~ECHO_CONTROL_MASK;

   return (SAL_SUCCESS);
}


int SalReadDriveTemp(void)
{
   // AXIS_OFF;

   PrintString("Control: ", 0);
   PrintSignedInteger(BGVAR(s16_Control_Temperature_Deg));
   PrintString(" [deg C]  Power: ", 0);
   PrintSignedInteger(BGVAR(s16_Power_Temperature_Deg));
   PrintString(" [deg C]", 0);
   if (VAR(AX0_u16_IPM_OT_DC_Enable) == 1)
   {
      PrintString("  IGBT: ", 0);

      if (!u16_STO_Flt_Indication_Actual_And_Delay)
      {
         PrintSignedInteger(BGVAR(u16_IPM_Temperature));
         PrintString(" [deg C]", 0);
      }
      else
      {
         PrintString(" STO not connected", 0);
      }
   }
   PrintCrLf();

   return SAL_SUCCESS;
}


int ClearFaultsCommand(int drive)
{
   // AXIS_OFF;

   if (Enabled(drive)) return (DRIVE_ACTIVE);

   if ( (BGVAR(s64_SysNotOk) != 0) || (BGVAR(s64_SysNotOk_2) != 0) )
   {
      // Next line is to make sure that clear faults will not automaticaly cause the drive to enable
//      if ((BGVAR(s16_DisableInputs) & (FLT_EXISTS_MASK | SW_EN_MASK)) == BGVAR(s16_DisableInputs))
        // Changed Logic to impose SW DIS whenever CLEARFAULTS is used, avoiding possible
        // ENABLE is no Input is defined as Remote Enable or Remote Enable is set.
      if ( ((BGVAR(s16_DisableInputs) & FLT_EXISTS_MASK) != 0) &&
           ((BGVAR(s16_DisableInputs) & SW_EN_MASK) == 0)        )
         DisableCommand(drive);

      FaultsHandler(drive, FLT_CLR);
   }

   // Panasonic Absolute Battery Low-Voltage Warning is latched, requires Encoder Resetting;
   // Avoid resetting this Warning if Battery Low-Voltage Fault is in effect!
   if ( ((BGVAR(u64_Sys_Warnings) & TM_BATT_LOW_VOLTAGE_WRN) != 0)  &&
        ((BGVAR(s64_SysNotOk) & ABS_ENC_BATT_LOW_MASK) == 0)        &&
        (LVAR(AX0_s32_Feedback_Ptr) == &PS_S_COMMUNICATION_FEEDBACK)  )
   {
      BGVAR(u16_CommFdbk_Init_Select) = 0;
      BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;
   }

   return SAL_SUCCESS;
}


//**********************************************************************
// Function Name: SalClearAlarmViaPParam
// Description:
//    This function is called upon writing a 0 to the P-parameter P0-01.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************************
int SalClearAlarmViaPParam(long long lparam, int drive)
{
   // AXIS_OFF;

   if (lparam != 0)
   {
      return (VALUE_IS_NOT_ALLOWED);
   }

   else
   {
      // For Schneider when issuing ARST in case that a LS is active --> align one time the "gearing
      // not-lim-position" with the PTP hunter output. This is a bug-fix for "BYang00000572". The sequence
      // that exposed the issue was as follows:
      //   a) Generate positive master pulses and trigger a positive LS. Axis stops.
      //   b) Generate negative master pulses, motor still does not move.
      //   c) Trigger ARST (clear-fault). Axis is not supposed to move, therefore align the positions for one time.
      if (BGVAR(u16_Hold_Bits) & (HOLD_LXM_LIMIT_SWITCH_MASK | HOLD_LXM_ACCESS_RIGHTS_MASK))
      {
         // Only for gearing mode without compensation move.
         if ((VAR(AX0_s16_Opmode) == 4) && ((VAR(AX0_u8_Gear_Limits_Mode) & 0x0001) == 0x0000))
         {
            // Instruct the gearing function to clear the position deviation.
            VAR(AX0_u16_Gear_BG_Flags) |= 0x0004; // Set bit 2, the flag is cleared automatically in the gearing code.
         }
         else if (VAR(AX0_s16_Opmode) == 8) // In OPMODE=8 set the input of the PTP hunter to the output of the PTP hunter in order to avoid motion
         {
            LLVAR(AX0_u32_Target_Position_Lo) = LLVAR(AX0_u32_Pos_Cmd_Ptp_Lo);
         }
      }

      // For Schneider reset a Lexium-hold in case of a clear-faults command.
      // This request was raised in order to get out of a so-called Quick-Stop
      // scenario after releasing exclusive access-rights (releasing exclusive
      // access-rights initiates a hold).
      BGVAR(u16_Hold_Bits) &= ~HOLD_LXM_ACCESS_RIGHTS_MASK;   // BGVAR(u16_Hold_Lexium_Access_Rights) = 0;

      // For Schneider a AL014 or AL015 (Lexium limit switch) indication is
      // supposed to be cleared upon a clear-fault command (although a HOLD
      // could be still active at that time). The Lexium handles the limit
      // switches in a special manner.
      BGVAR(u8_Show_Lex_LS_Warning) = LEX_LS_NO_WARNING;

      // clear all warnings
      BGVAR(u64_Sys_Warnings) = 0;
      BGVAR(u64_Sys_Warnings_Prev) = 0;
      BGVAR(u16_P0_47_Last_Wrn_Code) = 0;

      // If no fault is pending then just return SAL_SUCCESS, since the clear-fault command can
      // also be called in order to clear the lexium hold variable (see instruction above) but
      // in case of an enabled Drive the following clear-faults function call would return an
      // "DRIVE_ACTIVE" error which, is confusing for the local HMI and the commissioning tool.
      if ((BGVAR(s64_SysNotOk) == 0) && (BGVAR(s64_SysNotOk_2) == 0)) return SAL_SUCCESS;

      // Trigger the clear-faults command
      return (ClearFaultsCommand(drive));
   }
}


// buffer nust be long enough to cotain string
void PrintFpgaVersion(char* buff)
{
   unsigned int u16_temp;
   unsigned int u16_temp1;

    if (u32_Fpga_Version == 0xFFFFFFFF)
    {
       strcpy(buff, "NA");
       return;
    }

   u16_temp = ((u32_Fpga_Version & 0xF0000000) >> 28L) * 1000;
   u16_temp += ((u32_Fpga_Version & 0x0F000000) >> 24L) * 100;
   u16_temp += ((u32_Fpga_Version & 0x00F00000) >> 20L) * 10;
   u16_temp += ((u32_Fpga_Version & 0x000F0000) >> 16L);
   // PrintUnsignedInteger(u16_temp);
//    PrintChar(DOT);
   u16_temp1 = ((u32_Fpga_Version & 0x0000F000) >> 12L) * 1000;
   u16_temp1 += ((u32_Fpga_Version & 0x00000F00) >> 8L) * 100;
   u16_temp1 += ((u32_Fpga_Version & 0x000000F0) >> 4L) * 10;
   u16_temp1 += (u32_Fpga_Version & 0x0000000F);
 //   if (u16_temp1 <= 9)
//        PrintChar('0');
//
//    PrintUnsignedInteger(u16_temp);
//    PrintChar(SPACE);

   strcpy(buff, UnsignedInt64ToAscii(u16_temp));
   strcat(buff, ".");

   if (u16_temp1 <= 9)
       strcat(buff, "0");

   strcat(buff, UnsignedInt64ToAscii(u16_temp1));
   strcat(buff, " ");
   strcat(buff, p_s8_Month_Of_The_Year[u16_Fpga_Month - 1]);
   strcat(buff, " ");
   strcat(buff, UnsignedInt64ToAscii(u16_Fpga_Day));
   strcat(buff, " ");
   strcat(buff, UnsignedInt64ToAscii(u16_Fpga_Year));


//   else
//       sprintf(buff, "%u.%u %s %u %u", u16_temp, u16_temp1, p_s8_Month_Of_The_Year[u16_Fpga_Month - 1], u16_Fpga_Day, u16_Fpga_Year);

   //print FPGA dateu16_Fpga_Month
   // PrintString(p_s8_Month_Of_The_Year[u16_Fpga_Month - 1], 0);
//    PrintChar(SPACE);
//    PrintUnsignedInteger(u16_Fpga_Day);
//    PrintChar(SPACE);
//    PrintUnsignedInteger(u16_Fpga_Year);
}


int SalMotorFeedbackDirectionCommand(long long param, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (BGVAR(u16_Motor_Feedback_Direction) == (unsigned int)param) return (SAL_SUCCESS);

   BGVAR(u16_Motor_Feedback_Direction) = (unsigned int)param;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
   VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;

   return (SAL_SUCCESS);
}

// S.F: On SE this will implicitly call CONFIG to avoid No-Comp
int SalDirectionCommand(long long param, int drive)
{
   static int s16_state = 0, s16_stored_ret_val;
   static int s16_prev_value = 0;
   int s16_ret_val = SAL_SUCCESS;

   if ((BGVAR(s16_Direction) == (int)param) && (s16_state == 0)) return (SAL_SUCCESS);

   switch (s16_state)
   {
      case 0:
         s16_prev_value = BGVAR(s16_Direction);
         BGVAR(s16_Direction) = (int)param;

         // On SE avoid using CONFIG by running it implicitly
         if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
         {
            s16_ret_val = SAL_NOT_FINISHED;
            s16_state = 1;
         }
         else /* set no-comp fault */
            BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
      break;

      case 1:
//         s16_ret_val = SalConfigCommand(drive);
         // Using direct call to DesignRoutine for simplicity
         s16_ret_val = DesignRoutine(1, drive);
         if (s16_ret_val == SAL_SUCCESS) s16_state = 0;
         else if (s16_ret_val != SAL_NOT_FINISHED) // If CONFIG failed - restore previous VLIM value
         {
            s16_stored_ret_val = s16_ret_val;
            BGVAR(s16_Direction) = s16_prev_value;
            s16_ret_val = SAL_NOT_FINISHED;
            s16_state = 2;
         }
      break;

      case 2:
//         s16_ret_val = SalConfigCommand(drive);
         // Using direct call to DesignRoutine for simplicity
         s16_ret_val = DesignRoutine(1, drive);
         if (s16_ret_val != SAL_NOT_FINISHED)
         {
            s16_state = 0;
            if (s16_ret_val == SAL_SUCCESS)
               s16_ret_val = s16_stored_ret_val;
         }
      break;
   }

   return s16_ret_val;
}

int SalSfbDirectionCommand(long long param, int drive)
{
   //AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (BGVAR(s16_SFB_Direction) == (int)param)  return (SAL_SUCCESS);
   BGVAR(s16_SFB_Direction) = (int)param;
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
   VAR(AX0_s16_Skip_Flags) |= SEC_ENC_CONFIG_NEEDED_MASK;   
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalExternalEncResCommand
// Description:
//          This function is called in response to the XENCRES command.
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
int SalExternalEncResCommand(long long param, int drive)
{
   BGVAR(u32_External_Enc_Res) = (unsigned long)param;
   DesignGearMsqFilter(drive); // update filter

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalComModeCommand
// Description:
//          This function is called in response to the COMMODE command.
//          0 - Serail mode
//          1 - CANOpen
//       2 - for future use
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int SalComModeCommand(long long lparam, int drive)
{
   // AXIS_OFF;

   //check if CANOpen mode supported by HW
   if ((lparam == 1LL) && ((u32_Hw_Features & (CAN_TRANSCEIVER_MASK | ETHERNET_BASED_FIELDBUS_MASK) ) == 0))
   {
      return NOT_AVAILABLE;
   }

   // dont allow disabling commode when transeffing commands via ECAT FOE   
   if ((lparam == 0LL) && (BGVAR(u16_Foe_ParamsFile_State) != FOE_PARAMS_FILE_DWLD_IDLE))
   {
      return FOE_ACTIVE;
   }

   BGVAR(u8_Comm_Mode) = (int)lparam;

   if (lparam == 0)
   {
	  BGVAR(s16_DisableInputs) |= SW_EN_MASK;  //Set sw enable bit so drive will not be enabled
	  BGVAR(s16_DisableInputs) &= ~FB_EN_MASK; //Clear fieldbus enable so drive could be enabled via ASCII
      s16_Fieldbus_Flags &= 0xFFFE;          // Clear bit 0 that states that COMMODE = 1
      SetOpmode(drive, VAR(AX0_s16_Opmode)); // Call OPMODE function in order to re-initialize command-source pointers
      BGVAR(s16_Move_Abs_Issued) = 0;        // Set variable to default=0 since the fieldbus sets this variable to 1 in the target position real-time function.
      VAR(AX0_u16_Com_Mode1) = 0;
   }
   else
   {
	  //BZ# 6347 When switching from commode 0 to commode 1 a RT overload fault occurs. 
     //the reason was that a significat differance between pointers exsist and that closing the differance will cause a momentarely 
     //a RT overload
     u16_rt_rx_ctrl_out_index = u16_rt_rx_ctrl_in_index;
     BGVAR(s16_DisableInputs) |= FB_EN_MASK;
      s16_Fieldbus_Flags |= 0x0001;          // Set bit 0 that states that COMMODE = 1
      SetOpmode(drive, VAR(AX0_s16_Opmode)); // Call OPMODE function in order to re-initialize command-source pointers
      FalSetFBSyncOpmode(drive, (int)p402_modes_of_operation_display); // Needs to be called as well in case that the fieldbus
                                                                       // is activated, otherwise e.g. the "AX0_s16_Pre_Pos_Input_Ptr"
                                                                       // pointer is not restored properly.
      VAR(AX0_u16_Com_Mode1) = 1;
   }

   // disable/enable can interrupt via FPGA when changing commode
   // currently disable via fgpa is implemented for schinder fpga only
   if (u16_Product == SHNDR || u16_Product == SHNDR_HW)
   {
       if (lparam == 0)
       {
            // disable CAN interrupts
            SET_FPGA_DISABLE_CAN_MODE;
       }
       else if ((BGVAR(s64_SysNotOk_2) & CAN_BUSOFF_FLT_MASK) == 0)
       {
            // enable CAN interrupts if there is no busoff fault
            SET_FPGA_NORMAL_MODE;
       }
   }

   return (SAL_SUCCESS);
}


int SalHallsTypeCommand (long long param, int drive)
{
   if ((u16_Product != DDHD) && (param > 2)) return (VALUE_TOO_HIGH);
   BGVAR(u16_Halls_Type) = (int)param;
   *((int*)FPGA_DIFFERENTIAL_HALL_SEL_REG_ADD) = HallsTypeSelect(BGVAR(u16_Halls_Type), drive);
   return (SAL_SUCCESS);
}


int HallsTypeSelect(int Type, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if ((VAR(AX0_s16_Motor_Enc_Type) == 11) && (BGVAR(u16_FdbkType) == INC_ENC_FDBK))
   {
      if (Type == 1)
         return 0; // For Tamagawa Incremental (8-Wire) Encoder, prevent setting Halls Type to
      else         // Complementary using MFB Index Input, so Index is available to DSP QEP1
         return Type;
   }
   else // Allow setting of Complementary Halls Type for other Feedback setups.
     return Type;
}


int SalHallsInversionCommand(int drive)
{
   // AXIS_OFF;
   
   int s16_temp = 0;
   REFERENCE_TO_DRIVE;
   if (!s16_Number_Of_Parameters)   // Read command
   {
      PrintUnsignedInteger((VAR(AX0_s16_Halls_Inv) & 0x04) > 0);
      PrintChar(SPACE);
      PrintUnsignedInteger((VAR(AX0_s16_Halls_Inv) & 0x02) > 0);
      PrintChar(SPACE);
      PrintUnsignedInteger((VAR(AX0_s16_Halls_Inv) & 0x01) > 0);
      PrintCrLf();
   }
   else
   {
      if ((s64_Execution_Parameter[0] < 0LL) || (s64_Execution_Parameter[1] < 0LL) || (s64_Execution_Parameter[2] < 0LL))
         return (VALUE_TOO_LOW);
      if ((s64_Execution_Parameter[0] > 1LL) || (s64_Execution_Parameter[1] > 1LL) || (s64_Execution_Parameter[2] > 1LL))
         return (VALUE_TOO_HIGH);

      s16_temp = (int)s64_Execution_Parameter[0];
      s16_temp <<= 1;
      s16_temp |= (int)s64_Execution_Parameter[1];
      s16_temp <<= 1;
      s16_temp |= (int)s64_Execution_Parameter[2];
      VAR(AX0_s16_Halls_Inv) = s16_temp;
   }

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalErrorCountersCommand
// Description:
//          This function is called in response to the ERRCNTRS command.
//  1st Execution Parameter:  0 - Time-Out Cumulative Error Counter
//                            1 - CRC Cumulative Error Counter
//  2nd Execution Parameter:  None - Read Value
//                            0 - Reset Counter
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalErrorCountersCommand(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (s64_Execution_Parameter[0] < 0LL)
      return VALUE_TOO_LOW;
   else if (s64_Execution_Parameter[0] > 1LL)
      return VALUE_TOO_HIGH;

   if (s16_Number_Of_Parameters == 1)   // Read command
   {
      if (s64_Execution_Parameter[0] == 0LL)
         PrintUnsignedLong(LVAR(AX0_u32_Cumul_TO_Err_Cntr));
      else if (s64_Execution_Parameter[0] == 1LL)
         PrintUnsignedLong(LVAR(AX0_u32_Cumul_Crc_Err_Cntr));
      PrintCrLf();
   }
   else // Reset Command
   {
      if (s64_Execution_Parameter[1] != 0LL)
         return VALUE_OUT_OF_RANGE;
      else if (s64_Execution_Parameter[0] == 0LL)
         LVAR(AX0_u32_Cumul_TO_Err_Cntr) = 0L;
      else if (s64_Execution_Parameter[0] == 1LL)
         LVAR(AX0_u32_Cumul_Crc_Err_Cntr) = 0L;
   }

   return SAL_SUCCESS;
}


void UnitsPnumCommand(int drive)
{
   ConvertInternalToVel1000OutLoop(drive);
   ConvertVelToInternalOutLoop(drive);
   ConvertInternalToVel1000InLoop(drive);
   ConvertVelToInternalInLoop(drive);
   ConvertInternalToAccDec1000ForVel(drive);
   ConvertAccDecToInternalForVel(drive);
   ConvertInternalToAccDec1000ForPos(drive);
   ConvertAccDecToInternalForPos(drive);
}


void UnitsPdenCommand(int drive)
{
   ConvertInternalToVel1000OutLoop(drive);
   ConvertVelToInternalOutLoop(drive);
   ConvertInternalToVel1000InLoop(drive);
   ConvertVelToInternalInLoop(drive);
   ConvertInternalToAccDec1000ForVel(drive);
   ConvertAccDecToInternalForVel(drive);
   ConvertInternalToAccDec1000ForPos(drive);
   ConvertAccDecToInternalForPos(drive);
   ConvertPosToInternal(drive);
   ConvertInternalToPos1000(drive);
   ConvertCountsToInternal(drive);
}


int SalFbGearDrivingShaftRevs(long long param, int drive)
{
   BGVAR(u32_Fb_Gear_Driving_Shaft_Revs) = (unsigned long)param;
   CalcFieldBusUserToIntern(drive);
   CalcInternToFieldBusUser(drive);
   return (SAL_SUCCESS);
}


int SalFbGearMotorShaftRevs(long long param,int drive)
{
   BGVAR(u32_Fb_Gear_Motor_Shaft_Revs) = (unsigned long)param;
   CalcFieldBusUserToIntern(drive);
   CalcInternToFieldBusUser(drive);
   return (SAL_SUCCESS);
}

int SalFbInterpolationType(long long param, int drive)
{
   // AXIS_OFF;

   int ret_val = FAL_SUCCESS;
   REFERENCE_TO_DRIVE;
   if(FalIsInOperationMode()) // This variable is not allowed to be wriiten on OPERATION mode
   {
      ret_val = FAL_WRONG_NMT_STATE;
   }
   else // not in operational mode
   {   
      BGVAR(u16_Interpolation_Type) = (unsigned int)param;
      p402_interpolation_sub_mode_select = BGVAR(u16_Interpolation_Type);

      //save data in a rt variable
      VAR(AX0_u16_FieldBus_Int_Mode) = p402_interpolation_sub_mode_select;      
   }
   
   return(ret_val);
}


/*
 *->for time index and time period, need to set the PLL and fieldbus cycle as follows:
*/
int SalFbInterpTimeIdx(long long param, int drive)
{
   int ret_val = 0;
   char u8_tmp = 0;

   u8_tmp = BGVAR(s8_Fb_Interp_Time_Idx);
   BGVAR(s8_Fb_Interp_Time_Idx) = (char)param;

   ret_val = ConvertCanOpenFbusCycleTimeToSeconds(drive);

   if(ret_val == FAL_FAIL)
   {
      BGVAR(s8_Fb_Interp_Time_Idx) = u8_tmp;
      return FAL_FAIL;
   }



   UpdateFieldbusVelocityLimit(drive);
//   UpdateFieldbusAccLimit(drive); // not in use anymore
//   UpdateFieldbusDecLimit(drive); // not in use anymore
   return (SAL_SUCCESS);
}


int SalFbInterpTimePeriod(long long param, int drive)
{
   int ret_val = 0;
   char u8_tmp = 0;

   u8_tmp = BGVAR(u8_Fb_Interp_Time_Period);
   BGVAR(u8_Fb_Interp_Time_Period) = (unsigned char)param;

   ret_val = ConvertCanOpenFbusCycleTimeToSeconds(drive);

   if(ret_val == FAL_FAIL)
   {
      BGVAR(u8_Fb_Interp_Time_Period) = u8_tmp;
      return FAL_FAIL;
   }

   UpdateFieldbusVelocityLimit(drive);
//   UpdateFieldbusAccLimit(drive); // not in use anymore
//   UpdateFieldbusDecLimit(drive); // not in use anymore
   return (SAL_SUCCESS);
}

int SalScalingPdenominator(long long param, int drive)
{
   BGVAR(u32_Scaling_Pdenominator) = (unsigned long)param;

   //call functions for CanOpen scaling
   CalcFieldBusUserToIntern(drive);
   CalcInternToFieldBusUser(drive);

   UnitsPdenCommand(drive);   //call functions for service channel scalings

   return (SAL_SUCCESS);
}


int SalScalingPnumerator(long long param, int drive)
{
   BGVAR(u32_Scaling_Pnumerator) = (unsigned long)param;

   //call functions for CanOpen scaling
   CalcFieldBusUserToIntern(drive);
   CalcInternToFieldBusUser(drive);

   UnitsPnumCommand(drive);   //call functions for service channel scalings

   return (SAL_SUCCESS);
}


int SalFbscale(long long param, int drive)
{
   BGVAR(u8_Fbscale) = (unsigned char)param;
   ConvertFieldBusCountsToIntern(drive);
   ConvertInternToFieldBusCounts(drive);
   return (SAL_SUCCESS);
}


///////////////////////////////////////////////////////////
// function:ExpandString ,according to high/low
// Rise edge is 1
// Fall edge is 0
//
// The function does basically the opposite of the function "StoreString".
// The function copies the DRIVESCRIPT string from the "BGVAR(u16_Script)"
// array into the pointer location of the "OutputString" argument.
// The location of the DRIVESCRIPT string inside the "BGVAR(u16_Script) is defined
// by index (array elemnt where the string starts) and HighLow (take high-byte or
// low-byte from the array entry).
///////////////////////////////////////////////////////////
void ExpandString(int index, int HighLow, char* OutputString, int drive)
{
   int i = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   do
   {
      *(OutputString + i) = 0x0000;
      *(OutputString + i) = (BGVAR(u16_Script)[index * 128 + i] & (HighLow ? 0xff00 : 0x00ff)) >> (8 * HighLow);
      i++;
   }
   while (*(OutputString + i - 1) != 0);
   *(OutputString + i) = 0;
}


///////////////////////////////////////////////////////////
// function:StoreString ,according to high/low
// Rise edge is 1
// Fall edge is 0
//
// This function copies the DRIVESCRIPT string from "BGVAR(u8_Execution_String_Script)"
// array into the low or high byte (this depends on the edge) at the defined place
// (this depends on the index) inside the BGVAR(u16_Script) array.
///////////////////////////////////////////////////////////
void StoreString(int index, int Edge, int drive)
{
   int i = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

  /////////////////////////////////////////////////
  //ERASING THE SUB ARRAY ACCORDING TO HIGH/LOW
  /////////////////////////////////////////////////
   for (i = 0; i < 128; i++)
   {
    if (Edge)
         BGVAR(u16_Script)[index * 128 + i] &= 0x00ff;
    else
         BGVAR(u16_Script)[index * 128 + i] &= 0xff00;
   }
  /////////////////////////////////////////////////
   i = 0;
   do
   {
      // Add the user entered drive script string to the array, either in the high-byte or the
      // low-byte of each word depending on the edge.
      BGVAR(u16_Script)[index * 128 + i] |= (BGVAR(u8_Execution_String_Script)[i] << (8 * Edge));
      i++;
   }
   while ((BGVAR(u8_Execution_String_Script)[i] != 0) && (i < DRV_SCRIPT_EXECUTION_STRING_LENGTH)); // Protect from exceeding the array

   BGVAR(u8_Execution_String_Script)[0]=0; // Terminate the first character of the BGVAR(u8_Execution_String_Script)
   u8_Message_Buffer[0]=0;
}


//////////
int SalReadScript(int drive)
{
   int ScriptIndex = s64_Execution_Parameter[0];
   int ActivateSlope = s64_Execution_Parameter[1];

   ///////////////////////////////////////////////////////////////////
   // checking if parameter 1,index not higher than 31 or lower than 0
   ///////////////////////////////////////////////////////////////////
   if ((ScriptIndex < 0) || (ScriptIndex >= DRV_SCRIPT_NUM_OF_SCRIPTS))
   {
      PrintString("Invalid script index value", 0); // Script selector input can only select from 0...31 via 5 inputs.
      PrintCrLf();

      return SAL_SUCCESS;
   }

   ////////////////////////////////////////////////////////////////
   // checking if parameter 2,of Rise/Fall is Valid
   ////////////////////////////////////////////////////////////////
   if (ActivateSlope != 1 && ActivateSlope != 0)
   {
      PrintString("Invalid R/F Value", 0); // Invalid Rise / Fall setting
      PrintCrLf();

      return SAL_SUCCESS;
   }

   //////////////////////////////////////////////////////////////////
   // PRESENTING THE ARRAY CONTENT AFTER EXPANSION
   //////////////////////////////////////////////////////////////////
   ExpandString(ScriptIndex, ActivateSlope, &BGVAR(u8_Execution_String_Script)[0], drive);
   PrintChar('"');
   PrintString(&BGVAR(u8_Execution_String_Script)[0], 0);
   PrintCrLf();

   return SAL_SUCCESS;
}


int SalWriteScript(int drive)
{
   int ScriptIndex = s64_Execution_Parameter[0];
   int ActivateSlope = s64_Execution_Parameter[1];
   int i = 0,j, k, l, flag = 1;

   ///////////////////////////////////////////////////////////////////
   // checking if parameter 1,index not higher than 31 or lower than 0
   ///////////////////////////////////////////////////////////////////
   if ((ScriptIndex < 0) || (ScriptIndex >= DRV_SCRIPT_NUM_OF_SCRIPTS))
   {
      PrintString("Invalid script index value", 0); // Script selector input can only select from 0...31 via 5 inputs.
      PrintCrLf();

      return SAL_SUCCESS;
   }

   ////////////////////////////////////////////////////////////////
   // checking if parameter 2,of Rise/Fall is Valid
   ////////////////////////////////////////////////////////////////
   if (ActivateSlope != 1 && ActivateSlope != 0)
   {
      PrintString("Invalid R/F Value", 0); // Invalid Rise / Fall setting
      PrintCrLf();

      return SAL_SUCCESS;
   }
   //////////////////////////////////////////////////////////////////
   // PRESENTING THE ARRAY CONTENT AFTER EXPANSION
   //////////////////////////////////////////////////////////////////
   if (s16_Number_Of_Parameters == 3)
   {
      ///////////////////////////////////////////////
      //COPYING THE ENTERED STRING TO COMPRESSED ARRAY
      ///////////////////////////////////////////////
      StoreString(ScriptIndex,ActivateSlope, drive);
      ExpandString(ScriptIndex,ActivateSlope, &BGVAR(u8_Execution_String_Script)[0], drive);
      u8_Message_Buffer[0] = 0;
      ///////////////////////////////////////////////
      // measuring the string length
      ///////////////////////////////////////////////
      for (i = 0; BGVAR(u8_Execution_String_Script)[i] != 0; i++)
         j = i;
       ///////////////////////////////////////////////
      i = 0;
      while (i < j && flag)
      {
         k = i;
         ///////////////////////////////////////////////
         //CHECKING EACH STRING BETWEEN ~ OR TILL NULL
         ///////////////////////////////////////////////
         while (BGVAR(u8_Execution_String_Script)[k] != '~' && k <= j)
            k++;

         for (l = 0; l < (k - i); l++)
            u8_Message_Buffer[l] = BGVAR(u8_Execution_String_Script)[i + l];
         u8_Message_Buffer[l] = 0;

         PrintString(u8_Message_Buffer, 0);
         PrintCrLf();
         ///////////////////////////////////////////////
         //CHECKING EACH STRING
         ///////////////////////////////////////////////
         p_u8_Message_Buffer_Ptr = u8_Message_Buffer;

         while((flag = ScriptParserManager(drive, 0)) == 0);
         if (flag != 1)
         {
            PrintString("Unrecognized drive command. The script isn't valid", 0);
            PrintCrLf();
            BGVAR(u8_Execution_String_Script)[0] = '~';
            BGVAR(u8_Execution_String_Script)[1] = 0;
            StoreString(ScriptIndex, ActivateSlope, drive);
            flag = 0;
         }
         i = k + 1;
      }
   }

   u8_Message_Buffer[0] = '\0';
   p_u8_Message_Buffer_Ptr = u8_Message_Buffer;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalCanBitRate
// Description:
//          This function is called in response to the CANBITRATE command.
//
// Author: Itai(CAN master)
// Algorithm:
// Revisions:
//**********************************************************
int SalCanBitRate(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_Can_Bit_Rate) = (unsigned int)lparam;
   return SAL_SUCCESS;
}


int SalHallsFiltT1Command(long long param, int drive)
{
   unsigned long u32_temp1;
   unsigned int u16_temp2;

   u32_temp1 = (unsigned long)param;
   u16_temp2 = (unsigned int)(u32_temp1 / 125);
   if ( ((unsigned long)u16_temp2 * 125) == u32_temp1 ) // check if user value is muliple of 125 (0.125 * 1000)
   {
      BGVAR(u32_Halls_Msq_Filt_User_T1) = u32_temp1;
      DesignHallsMsqFilter(drive);
      return (SAL_SUCCESS);
   }
   else
      return (ARGUMENT_NOT_125);
}


int SalHallsFiltT2Command(long long param, int drive)
{
   unsigned int u16_temp1, u16_temp2;

   u16_temp1 = (unsigned int)param;
   u16_temp2 = (unsigned int)(u16_temp1 / 125);
   if ( (u16_temp2 * 125) == u16_temp1 ) // check if user value is muliple of 125 (0.125 * 1000)
   {
      BGVAR(u16_Halls_Msq_Filt_User_T2) = u16_temp1;
      DesignHallsMsqFilter(drive);
      return (SAL_SUCCESS);
   }
   else
      return (ARGUMENT_NOT_125);
}


int SalHallsFiltAffCommand(long long param, int drive)
{
   BGVAR(s16_Halls_Msq_Filt_User_Aff) = (int)param;
   DesignHallsMsqFilter(drive);
   return (SAL_SUCCESS);
}


int SalHallsFiltVffCommand(long long param, int drive)
{
   BGVAR(s32_Halls_Msq_Filt_User_Vff) = (long)param;
   DesignHallsMsqFilter(drive);
   return (SAL_SUCCESS);
}



//**********************************************************
// Function Name: SalReadSerialNumber
// Description:
//    This function reads theSerial Number
//    by the P-Param P_P9-41 to P9-44.
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int SalReadSerialNumber (long long *data,int drive)
{
   int i,s16_temp;
   long s32_temp = 0;
   char * s8_SerialNum;
   int pparam_index = (int)s64_Execution_Parameter[0]; // Ask for index
   drive += 0;
   if ((pparam_index < 0) || (pparam_index > 3))
       return (VALUE_OUT_OF_RANGE);

   s8_SerialNum = GetSerialNum();
  //Get bytes from words
   for(i=0; i < 4; i++)
   {
       s32_temp =  (s32_temp << 8);
       s16_temp = s8_SerialNum[((pparam_index)*4)+i];
       s32_temp |= (long) (s16_temp & 0x00FF);
   }
   *data = s32_temp;

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalReadUserName
// Description:
//    This function reads the Modbus User-Apllication name
//    by the P-Param P_P9-08_UNAME to P9-11.
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************
int SalReadUserName (long long *data,int drive)
{
 long long index = s64_Execution_Parameter[0]; // Ask for index

   REFERENCE_TO_DRIVE;     // Defeat compilation error

  if ((index < 0LL) || (index > 3LL))
       return (VALUE_OUT_OF_RANGE);
   //Todo - get only related bytes
  *data = (long long) BGVAR(u32_User_App_Name)[index];

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalWriteUserName
// Description:
//    This function Writes the Modbus User-Apllication name
//    by the P-Param P_P9-08_UNAME to P9-11.
// Author: Udi
// Algorithm:
// Revisions:
//**********************************************************

int SalWriteUserName (int drive)
{

   long long index = s64_Execution_Parameter[0];
   long long value = s64_Execution_Parameter[1];

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((index < 0LL) || (index > 3LL))
       return (VALUE_OUT_OF_RANGE);

//   if ((value<0LL) || (value > 4294967295LL))
//      return (VALUE_OUT_OF_RANGE);

   // Get the value out of the buffer (from variable P0-35...P0-42)
   BGVAR(u32_User_App_Name)[index] = (long) value;





   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadParameterMappingCommand
// Description:
//    This function reads the parameters mapped in P0-35 to P0-42
//    by the parameter P0-25 to P0-32. This function is called upon
//    a parameter access of P0-25...P0-32.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadParameterMappingCommand (long long *data,int drive)
{
   long long index = s64_Execution_Parameter[0]; // Ask for index
   // Save temporarily the global access-channel variable. This is special since
   // a P-parameter function calls a further P-parameter function (ExecutePParam
   // is called within the ExecutePParam function itself --> Re-intrant) and the
   // global access-channel variable is automatically cleared when leaving the
   // ExecutePParam function.
   int s16_Temp_Access_Channel = BGVAR(s16_Lexium_ExecPParam_Acces_Channel);

   unsigned long u32_Temp_Table_Value;
   unsigned int  u16_P_Param_Value1;
   unsigned int  u16_P_Param_Value2;

   unsigned int u16_temp_ret_Val = SAL_SUCCESS;
   unsigned int u16_ret_Val = SAL_SUCCESS;
   long s32_Result1 = 0;
   long s32_Result2 = 0;

   if ((index < 0LL) || (index > 7LL))
       return (VALUE_OUT_OF_RANGE);

   // Get the value out of the buffer (from variable P0-35...P0-42)
   u32_Temp_Table_Value = BGVAR(u32_P0_35_To_42)[index];

   // Convert the 32-bit buffer value into two valid P-parameter values (e.g. buffer-value
   // 0x01040103 = P-Parameters P1-04 & P1-03 = 104 & 103 for the "ExecutePParam" function.
   u16_P_Param_Value1 = (u32_Temp_Table_Value & 0x000000FF) +
                        100 * ((u32_Temp_Table_Value & 0x0000FF00)>>8);
   u16_P_Param_Value2 = ((u32_Temp_Table_Value & 0x00FF0000)>>16) +
                        100 * ((u32_Temp_Table_Value & 0xFF000000)>>24);

   // If the upper 16-bit value equals the lower 16-bit value --> 1 x 32-bit read access
   if (u16_P_Param_Value1 == u16_P_Param_Value2)
   {
      // Perform 32-bit read access. Use the global variable for the access-channel insformation, since you do not know who called this Sal-function.
      u16_temp_ret_Val = ExecutePParam(drive, u16_P_Param_Value1, OP_TYPE_READ, &s32_Result1, s16_Temp_Access_Channel);
      // If the access was not successfull
      if (u16_temp_ret_Val != SAL_SUCCESS)
      {
         u16_ret_Val = u16_temp_ret_Val;
      }
   }
   else // 2x16-Bit read-access
   {
      // Perform first read-access. Use the global variable for the access-channel insformation, since you do not know who called this Sal-function.
      u16_temp_ret_Val = ExecutePParam(drive, u16_P_Param_Value1, OP_TYPE_READ, &s32_Result1, s16_Temp_Access_Channel);
      // If the access was not successfull
      if (u16_temp_ret_Val != SAL_SUCCESS)
      {
         u16_ret_Val = u16_temp_ret_Val;
      }

      // Perform second read access. Use the global variable for the access-channel insformation, since you do not know who called this Sal-function.
      u16_temp_ret_Val = ExecutePParam(drive, u16_P_Param_Value2, OP_TYPE_READ, &s32_Result2, s16_Temp_Access_Channel);
      // If the access was not successfull
      if (u16_temp_ret_Val != SAL_SUCCESS)
      {
         u16_ret_Val = u16_temp_ret_Val;
      }

      // Now merge both results together into s32_result1
      s32_Result1 &= 0x0000FFFF; // Clear upper 16-bits and keep lower 16-bits of result 1
      s32_Result1 |= ((s32_Result2<<16) & 0xFFFF0000); // Put result 2 into upper 16-bits
   }

   // Give the result back
   *data = s32_Result1;

   return (u16_ret_Val);
}

//**********************************************************
// Function Name: SalWriteParameterMappingCommand
// Description:
//    This function writes the parameters mapped in P0-35 to P0-42
//    by the parameter P0-25 to P0-32. This function is called upon
//    a parameter access of P0-25...P0-32.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteParameterMappingCommand(int drive)
{
   long long index = s64_Execution_Parameter[0];
   long long value = s64_Execution_Parameter[1];

   // Save temporarily the global access-channel variable. This is special since
   // a P-parameter function calls a further P-parameter function (ExecutePParam
   // is called within the ExecutePParam function itself --> Re-intrant) and the
   // global access-channel variable is automatically cleared when leaving the
   // ExecutePParam function..
   int s16_Temp_Access_Channel = BGVAR(s16_Lexium_ExecPParam_Acces_Channel);

   unsigned long u32_Temp_Table_Value;
   unsigned int  u16_P_Param_Value1;
   unsigned int  u16_P_Param_Value2;

   unsigned int u16_temp_ret_Val = SAL_SUCCESS;
   unsigned int u16_ret_Val = SAL_SUCCESS;
   long s32_dataToBeWritten = 0;

   if ((index < 0LL) || (index > 7LL))
       return (VALUE_OUT_OF_RANGE);
/* There is no need to check th limits of 32 bit if it's full range
   if ((value<0LL) || (value > 4294967295LL))
      return (VALUE_OUT_OF_RANGE);
*/
   // Get the value out of the buffer (from variable P0-35...P0-42)
   u32_Temp_Table_Value = BGVAR(u32_P0_35_To_42)[index];

   // Convert the 32-bit buffer value into two valid P-parameter values (e.g. buffer-value
   // 0x01040103 = P-Parameters P1-04 & P1-03 = 104 & 103 for the "ExecutePParam" function.
   u16_P_Param_Value1 = (u32_Temp_Table_Value & 0x000000FF) +
                        100 * ((u32_Temp_Table_Value & 0x0000FF00)>>8);
   u16_P_Param_Value2 = ((u32_Temp_Table_Value & 0x00FF0000)>>16) +
                        100 * ((u32_Temp_Table_Value & 0xFF000000)>>24);


   // If the upper 16-bit value equals the lower 16-bit value --> 1 x 32-bit write access
   if (u16_P_Param_Value1 == u16_P_Param_Value2)
   {
      // Determine value to be written
      s32_dataToBeWritten = (long)value;
      // Perform 32-bit write access. Use the global variable for the access-channel insformation, since you do not know who called this Sal-function.
      u16_temp_ret_Val = ExecutePParam(drive, u16_P_Param_Value1, OP_TYPE_WRITE, &s32_dataToBeWritten, s16_Temp_Access_Channel);
      // If the access was not successfull
      if (u16_temp_ret_Val != SAL_SUCCESS)
      {
         u16_ret_Val = u16_temp_ret_Val;
      }
   }
   else // 2x16-Bit write-access
   {
      // Determine first value to be written
      s32_dataToBeWritten = (long)value & 0x0000FFFF;
      // Perform first write-access. Use the global variable for the access-channel insformation, since you do not know who called this Sal-function.
      u16_temp_ret_Val = ExecutePParam(drive, u16_P_Param_Value1, OP_TYPE_WRITE, &s32_dataToBeWritten, s16_Temp_Access_Channel);
      // If the access was not successfull
      if (u16_temp_ret_Val != SAL_SUCCESS)
      {
         u16_ret_Val = u16_temp_ret_Val;
      }

      // Determine second value to be written
      s32_dataToBeWritten = ((long)value>>16) & 0x0000FFFF;
      // Perform second write-access. Use the global variable for the access-channel insformation, since you do not know who called this Sal-function.
      u16_temp_ret_Val = ExecutePParam(drive, u16_P_Param_Value2, OP_TYPE_WRITE, &s32_dataToBeWritten, s16_Temp_Access_Channel);
      // If the access was not successfull
      if (u16_temp_ret_Val != SAL_SUCCESS)
      {
         u16_ret_Val = u16_temp_ret_Val;
      }
   }

   return (u16_ret_Val);
}


//**********************************************************
// Function Name: SalReadBlockDataRwValueCommand
// Description:
//    This function reads the parameters P0-35 to P0-42
//    from an array.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadBlockDataRwValueCommand (long long *data,int drive)
{
   long long index = s64_Execution_Parameter[0];

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((index < 0LL) || (index > 7LL))
       return (VALUE_OUT_OF_RANGE);

   *data = (BGVAR(u32_P0_35_To_42)[index]);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalWriteBlockDataRwValueCommand
// Description:
//    This function writes the param P0-35 to P0-42 into an array.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteBlockDataRwValueCommand(int drive)
{
   long long index = s64_Execution_Parameter[0];
   long long value = s64_Execution_Parameter[1];

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((index < 0LL) || (index > 7LL))
       return (VALUE_OUT_OF_RANGE);
/* There is no need to check th limits of 32 bit if it's full range
   if ((value<0LL) || (value > 4294967295LL))
      return (VALUE_OUT_OF_RANGE);
*/

   // Write the value into the array
   BGVAR(u32_P0_35_To_42)[index] = (long)value;

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: UpdateServoOutputStatusDisplay
// Description:
//    For a Lexium Drive this function is called continuously in background
//    and generatest he content of the Lexium parameter P0-46. The parameter
//    P0-46 is actually a Drive Status bit-variable, where each bit has a
//    unique meaning. The global variable can also be used by several
//    digital output functions if required.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void UpdateServoOutputStatusDisplay (int drive)
{
   // AXIS_OFF; // Define AXIS_OFF for and VAR / LVAR access

   unsigned int u16_Drive_Status = 0x0000;

   /*** Handle Bit 0 - SRDY (Servo ready) ***/
   if (BGVAR(s16_DisableInputs) & 0xFFFC)  // If something prevents the Drive from getting enabled except the HW and SW enable (what the Lexium usually uses).
   {
      // State that the Drive is not ready
      u16_Drive_Status &= ~0x0001;
   }
   else
   {
      // State that the Drive is ready to get enabled
      u16_Drive_Status |= 0x0001;
   }


   /*** Handle Bit 1 - SON (Servo on) ***/
   if (Enabled(drive))
   {
      // State that the Drive is enabled
      u16_Drive_Status |= 0x0002;
   }
   else
   {
      // State the the Drive is disabled
      u16_Drive_Status &= ~0x0002;
   }


   // FIX IPR#1396: ZSPD check moved to RT
   // move the output control the RT - in BG handle all the requested states and
   // the actual write to the FPGA will be preformed in RT MTS-1 & MTS-5

   /*** Handle Bit 2 - ZSPD (At zero speed)*/
   if (VAR(AX0_s16_ZSPD_Out_Num_Forced) & 0x0800)
   {
      // State that the signal ZSPD is active
      u16_Drive_Status |= 0x0004;
   }
   else
   {
      // State that the signal ZSPD is not active
      u16_Drive_Status &= ~0x0004;
   }


   /*** Handle Bit 3 - TSPD (At speed reached)          ***/
   if (labs(LVAR(AX0_s32_Vel_Var_Fb_0)) >= BGVAR(u32_P1_39_Target_Rotation_Speed))
   {
      // State that the signal TSPD is active
      u16_Drive_Status |= 0x0008;
   }
   else
   {
      // State that the signal TSPD is not active
      u16_Drive_Status &= ~0x0008;
   }


   /*** Handle Bit 4 - TPOS (At positioning completed)  ***/
   // TPOS will be handled in RT and the ON/OFF indication will be used here just like for ZSPD
   if ( VAR(AX0_s16_TPOS_Out_Num_Forced) & 0x0800)
   {
      // State that the signal TPOS is active
      u16_Drive_Status |= 0x0010;
   }
   else
   {
      u16_Drive_Status &= ~0x0010;
   }


   /*** Handle Bit 5 - TQL (At torque limit)            ***/
   if (
         BGVAR(s16_I_Sat_Hi_En)    && (VAR(AX0_s16_Opmode) != 2) &&
        (VAR(AX0_s16_Opmode) != 3) && VAR(AX0_s16_TRQ_Output)
      )
   {
      // State that the signal TQL is active
      u16_Drive_Status |= 0x0020;
   }
   else
   {
      // State that the signal TQL is not active
      u16_Drive_Status &= ~0x0020;
   }


   /*** Handle Bit 6 - ALRM (Alarm signal). Set the bit also if AL014/015 (limit switch warning) is active ***/
   if ((BGVAR(s16_DisableInputs) & FLT_EXISTS_MASK) || (BGVAR(u8_Show_Lex_LS_Warning)))
   {
      // State that an error/alarm is pending
      u16_Drive_Status |= 0x0040;
   }
   else
   {
      // State that there is no error
      u16_Drive_Status &= ~0x0040;
   }


   /*** Handle Bit 7 - BRKR (Holding brake control) ***/
   if (VAR(AX0_u16_BrakeOn)) // If Brake-on is true (Brake is closed). The logic in the CDHD and the Lexium is inverted.
   {                       // CDHD: true = brake is engaged; Lexium: true = Brake is disengaged
      // State that no power to the brake is applied (the brake is engaged)
      u16_Drive_Status &= ~0x0080;
   }
   else
   {
      // State that power to the brake is applied (the brake is disengaged)
      u16_Drive_Status |= 0x0080;
   }


   /*** Handle Bit 8 - HOME (Homing completed) ***/
   if (BGVAR(u8_Homing_State) == HOMING_TARGET_REACHED)
   {
      // State that the homing has been completed
      u16_Drive_Status |= 0x0100;
   }
   else
   {
      // State that homing has not yet been started or finished
      u16_Drive_Status &= ~0x0100;
   }


   /*** Handle Bit 9 - OLW (Output overload warning) ***/
   if (BGVAR(u64_Sys_Warnings) & (DRIVE_FOLDBACK_WRN_MASK | MOTOR_FOLDBACK_WRN_MASK))
   {
      // State that an overload waring (foldback warning) exists
      u16_Drive_Status |= 0x0200;
   }
   else
   {
      // State that no overload waring (foldback warning) exists
      u16_Drive_Status &= ~0x0200;
   }


   /*** Handle Bit 10 - WARN (Warning signal) ***/
   if (BGVAR(u64_Sys_Warnings) != 0)
   {
      // State that a warning is pending
      u16_Drive_Status |= 0x0400;
   }
   else
   {
      // State that no waring is pending
      u16_Drive_Status &= ~0x0400;
   }

   // Update the gloal Drive status variable
   BGVAR(u16_P0_46_SVSTS) = u16_Drive_Status;
}

//**********************************************************
// Function Name: SalWriteAuxiliaryFunctionsCommand
// Description:
//    This function is called for P2-30 INH parameter change.
//    valid values:
//                    0, -1, -5 - Disable all aux functions
//                    1 - Force servo drive to be "ON" (active)
//                    5 - Don't save changed parameters when drive switched OFF - N/A in CDHD
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************

int SalWriteAuxiliaryFunctionsCommand(long long param,int drive)
{
   int s16_sal_response = SAL_SUCCESS;
   switch(param)
   {
       case -1:
       case -2:
       case 0:// disable all AUX functions
            s16_sal_response = DisableCommand(drive);
       break;

       case 1:// force drive to be enable
         // Check if it is allowed to enable the Drive according to the access-rights management
         s16_sal_response = CheckLexiumAccessRights(drive,LEX_AR_FCT_ENABLE,BGVAR(s16_Lexium_ExecPParam_Acces_Channel));
         if (s16_sal_response == SAL_SUCCESS)
         {
            s16_sal_response = EnableCommand(drive);
         }
       break;

       case 5:// Don't save changed parameters to eeprom when drive switched OFF
             // TBD
       break;

       default:
       break;
   }
   BGVAR(s16_P2_30_INH) = (int) param;
   return s16_sal_response;
}

//**********************************************************
// Function Name: SalWriteSpecialFunction1Command
// Description:
//    This function is called for P2-65 GBIT parameter change.
//    valid BITs:
//               6:
//               9:  U, V, W wiring cut-off detection
//               10: DI ZCLAMP function selection
//               11: NL(CWL)/PL(CCWL) pulse input inhibit function
//               12: Detection of missing input power phase
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteSpecialFunction1Command(long long param,int drive)
{
   // AXIS_OFF;
   if ( ((int)param) & ~(0x3E40)) return (VALUE_OUT_OF_RANGE);// accept only valid bits combination.
   BGVAR(u16_P2_65_GBIT) = (unsigned int) param;
   if (u16_Product == SHNDR_HW)
   {// those features are available only for Schneider drive mode
       if (BGVAR(u16_P2_65_GBIT) & 0x200) // test BIT 9 - enable or disable motor phase disconnected alarm
       {// turn on

       }
       else
       {// turn off

       }

       if (BGVAR(u16_P2_65_GBIT) & 0x400) // test BIT 10
       {// turn on

       }
       else
       {// turn off

       }
   }
   // this feature is also for CDHD drive mode
   if (BGVAR(u16_P2_65_GBIT) & 0x800) // test BIT 11
   {// turn on pulse inhibit on limit switch (dont log pulses)
      SalGearLimitsModeCommand((long long)(VAR(AX0_u8_Gear_Limits_Mode) & ~(0x02)),drive);
   }
   else
   {// turn off pulse inhibit on limit switch (log pulses)
      SalGearLimitsModeCommand((long long)(VAR(AX0_u8_Gear_Limits_Mode)|(0x02)),drive);
   }
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalWriteBounceFilterCommand
// Description:
//    This function is called for P2-09 DRT to set the debounce filter.
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteBounceFilterCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_P2_09_Bounce_Filter) = (unsigned int)param;
   *(unsigned int *)FPGA_P2_09_REG_ADD = BGVAR(u16_P2_09_Bounce_Filter);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteFastBounceFilterCommand
// Description:
//    This function is called for P2-24 FDRT to set
//    the fast inputs debounce filter value.
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteFastBounceFilterCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_P2_24_Fast_Bounce_Filter) = (unsigned int)param;
   *(unsigned int *)FPGA_P_FAST_REG_ADD = BGVAR(u16_P2_24_Fast_Bounce_Filter);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadPtPulseBounceFilter
// Description:
//    This function is called for P2-36 PT_PULSE_FLTR to get
//    the Pulse-train pulse input debounce filter value.
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPtPulseBounceFilter(long long *data,int drive)
{
   drive = drive;
   if ((*(unsigned int *)FPGA_PULSE_FILTER_REG_ADD) -  BGVAR(u16_P2_36_Pt_Pulse_Bounce_Filter) == 1)
   {// the FPGA is rounding up by "1" even numbers so if the diff between the FPGA value and the parameter value is 1 return the parameter value
      *data = (long long)BGVAR(u16_P2_36_Pt_Pulse_Bounce_Filter);
   }
   else
   {// else return the FPGA register value
      *data = (long long)*(unsigned int *)FPGA_PULSE_FILTER_REG_ADD;
   }

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: WritePtPulseBounceFilterCommand
// Description:
//    This function is to set
//    the Pulse-train pulse input debounce filter value.
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int WritePtPulseBounceFilterCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_P2_36_Pt_Pulse_Bounce_Filter) = (unsigned int)param;
   *(unsigned int *)FPGA_PULSE_FILTER_REG_ADD = BGVAR(u16_P2_36_Pt_Pulse_Bounce_Filter);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadPtDirectionBounceFilter
// Description:
//    This function is called for P2-37 PT_DIRECT_FLTR to get
//    the Pulse-train direction input debounce filter value.
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPtDirectionBounceFilter(long long *data,int drive)
{
   drive = drive;
   if (((*(unsigned int *)FPGA_DIRECTION_FILTER_REG_ADD) - BGVAR(u16_P2_37_Pt_Direction_Bounce_Filter)) == 1)
   {// the FPGA is rounding up by "1" even numbers so if the diff between the FPGA value and the parameter value is 1 return the parameter value
       *data = (long long)BGVAR(u16_P2_37_Pt_Direction_Bounce_Filter);
   }
   else
   {// else return the FPGA register value
       *data = (long long)*(unsigned int *)FPGA_DIRECTION_FILTER_REG_ADD;
   }
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: WritePtDirectionBounceFilterCommand
// Description:
//    This function is called to set
//    the Pulse-train direction input debounce filter value.
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int WritePtDirectionBounceFilterCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_P2_37_Pt_Direction_Bounce_Filter) = (unsigned int)param;
   *(unsigned int *)FPGA_DIRECTION_FILTER_REG_ADD = BGVAR(u16_P2_37_Pt_Direction_Bounce_Filter);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalWriteSpecialFunction2Command
// Description:
//    This function is called for P2-66 GBIT2 parameter change.
//    valid BITs:
//               2: under voltage clear mode selection
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteSpecialFunction2Command(long long param,int drive)
{
   drive += 0; // compilation remark

   if ((param != 0) && (param != 4)) return (VALUE_OUT_OF_RANGE);
   BGVAR(u16_P2_66_GBIT2) = (unsigned int) param;

   if (BGVAR(u16_P2_66_GBIT2) & 0x04) // test BIT 2
   {// set under voltage to be cleared automaticly
      BGVAR(u16_Uv_Recover) = 0x1;
   }
   else
   {// set under voltage to not be cleared automaticly
      BGVAR(u16_Uv_Recover) = 0;
   }
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalWriteCRSHACommand
// Description: Set CRSHA_Internal value as % (CRSHA) % from MICONT
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteCRSHACommand(long long param,int drive)
{
   long s32_fix = 0L;
   unsigned int  u16_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (param > 300 || param < 0 ) return VALUE_OUT_OF_RANGE;

   BGVAR(s16_P1_57_CRSHA) = (int)param;

   // s16_P1_57_CRSHA_Internal = (CRSHA/100) * MICONT
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, (long)0xA3D7,(unsigned int)22,(long)BGVAR(s16_P1_57_CRSHA), (unsigned int)0);
   BGVAR(s32_P1_57_CRSHA_Internal) = MultS64ByFixU32ToS64((long long)((BGVAR(s32_Motor_I_Cont)*26214L) / BGVAR(s32_Drive_I_Peak)),s32_fix,u16_shift);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalSetExclusiveAccessRights
// Description:
//    This function is called upon writing the P-parameter P10-32, which is known
//    AccessExcl. Writing a 1 means, that exclusive access-rigths are requested
//    by a specific access-channel. Writing a 0 means, that exclusive access-rights
//    are being released by the specific access-channel.
//    The way how access-rights are being permitted is described in the document
//    "Bai_Yang_DES04_Access_Channels.doc".
//    This function must only be called by the function that handles the P-parameters
//    since a global variable called "s16_Lexium_ExecPParam_Acces_Channel" needs
//    to be set in advance!
//
// Author: APH
// Algorithm:
// Revisions: Nitsan, 25/1/2015: decline giving access rights if in canopen mode
//                    and x_end bit equals zero (IPR 855).
//**********************************************************
int SalSetExclusiveAccessRights(long long lparam, int drive)
{
   int s16_Return_Value = SAL_SUCCESS;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // If exclusive request is requested
   if (lparam == 1)
   {

      // Return success in case that an access-channel, which is already owner of
      // the access-rights, asks for access-rights again. ClearQuest BYang00000325
      if ( (BGVAR(u16_Lexium_Acces_Rights_State) & 0x00FF) &&                                                        // Exclusive access-rights owned by someone
           (BGVAR(s16_Lexium_ExecPParam_Acces_Channel) == ((BGVAR(u16_Lexium_Acces_Rights_State) >> 8) & 0x00FF)) )  // Current owner of access rights askes again
      {
         return (SAL_SUCCESS); // Return success but do nothing
      }

      // If there are no exclusive access-rights yet assigned        AND
      // if the motor considered as being in standstill              AND
      // if the Drive-type is known                                  AND
      // if obtaining the access rights is not blocked (by fieldbus) AND
      // if in canopen mode and x_end bit equals 1 (on other mode, the bit does not mater). IPR 855.
      if ( ((BGVAR(u16_Lexium_Acces_Rights_State) & 0x00FF) == 0)  &&
          (BGVAR(u16_P0_46_SVSTS) & 0x0004)                       &&
          (BGVAR(u8_Lexium_Drive_Type) != LEX_TYPE_RESERVED)      &&
          (BGVAR(u8_Lexium_Access_Rights_Locked) == 0)            &&
          (((BGVAR(u16_P1_01_CTL_Current) & 0xff) != SE_OPMODE_CANOPEN) || (p402_statusword & X_END_BIT))
        )
      {
         if ((BGVAR(s16_Lexium_ExecPParam_Acces_Channel) == LEX_CH_LOCAL_HMI) ||
            (BGVAR(s16_Lexium_ExecPParam_Acces_Channel) == LEX_CH_MODBUS_RS485))
         {
            if ( ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_SERCOS)        ||
                ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERNET_IP)   ||
                ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERCAT)         )
            {
            }
            else if ((BGVAR(u16_P1_01_CTL_Current) & 0xff) == SE_OPMODE_CANOPEN)
            {
               BGVAR(s16_CAN_Opmode_Temp) = PROFILE_POSITION_MODE;
               BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
               SetCanOpmode(drive);
            }
            else
            {
               SetOpmode(drive, 8);
            }
            // Perform access-rights switch and set low-byte to 1 (exclusive access)
            BGVAR(u16_Lexium_Acces_Rights_State) = ((BGVAR(s16_Lexium_ExecPParam_Acces_Channel)<<8) & 0xFF00) | 0x0001;
            BGVAR(u16_Hold_Bits) &= ~HOLD_HALT_MASK;
         }
         else // Only CT & lHMI gets exclusive access at the moment
         {
            s16_Return_Value = LEXIUM_INVALID_ACCESS_RIGHTS;
         }
      }
      else
      {
         // Exclusive access-rights are currently assigned
         s16_Return_Value = LEXIUM_INVALID_ACCESS_RIGHTS;
      }
   }
   else if (lparam == 0) // If the access rights are supposed to be released.
   {
      // If the channel that asks for the access-rights release is currently in possession of exclusive access rights
      if ( (BGVAR(s16_Lexium_ExecPParam_Acces_Channel) == ((BGVAR(u16_Lexium_Acces_Rights_State)>>8) & 0x00FF)) &&
           ((BGVAR(u16_Lexium_Acces_Rights_State) & 0x00FF) == 1)                                                 )
      {
         // Release exclusive access-rights and establish the default access-rights depending on the Drive type
         if (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_IO_DRIVE)
         {
            BGVAR(u16_Lexium_Acces_Rights_State) = (LEX_CH_IO_CONTROL<<8) & 0xFF00;
         }
         else if (BGVAR(u8_Lexium_Drive_Type) == LEX_TYPE_FIELDBUS_DRIVE)
         {
            BGVAR(u16_Lexium_Acces_Rights_State) = (LEX_CH_FIELDBUS<<8) & 0xFF00;
         }
         else
         {
            s16_Return_Value = LEXIUM_INVALID_ACCESS_RIGHTS; // Unknown Drive Type, not supposed to happen
         }

         // If no disable request is currently pending, which means that the Drive is
         // supposed to stay enable when the access-rights are going to be released).
         if (BGVAR(s16_DisableInputs) == 0)
         {
            BGVAR(u16_Deceleration_Event_ID) = EVENT_DECSTOP;
            BGVAR(u16_Hold_Bits) |= HOLD_LXM_ACCESS_RIGHTS_MASK; // BGVAR(u16_Hold_Lexium_Access_Rights) = 1;// Triggger a user-hold (Schneider calls it QuickStop) after successfully releasing access-rights.
                                       // This is a Schneider request. Hold needs to be cleared via a Schneider Clear-Fault.
            HoldScan(drive);
         }

         // if gear mode, need to restore opmdoe back to gear, otherwise drive is stuck in position mode.
         if ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_PT)
         {
            SetOpmode(drive, 4);
         }

         // signal dual mods to setup the correct opmode on access rights release
         BGVAR(u16_Lexium_Acces_Rights_Released) = 1;
      }
      else
      {
         // Exclusive access rights can only be released by current owner
         s16_Return_Value = LEXIUM_INVALID_ACCESS_RIGHTS;
      }
   }
   else
   {
      // Value is out of range
      s16_Return_Value = VALUE_OUT_OF_RANGE;
   }

   return s16_Return_Value;
}

//**********************************************************
// Function Name: SalGetExclusiveAccessRights
// Description:
//    This function is called upon reading the P-parameter P10-32, which is known
//    AccessExcl. Reading a 1 means, that exclusive access-rigths are assigned
//    to a certain access-channel. Reading a 0 means, that exclusive access-rights
//    are not assigned and can therefore be requested by the specific access-channel.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalGetExclusiveAccessRights(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   *data = (long long)(BGVAR(u16_Lexium_Acces_Rights_State) & 0x00FF);

   return (SAL_SUCCESS);
}



//**********************************************************
// Function Name: SalLineLossTypeCommand
// Description:
//          This function is called in response to the LINELOSSTYPE command.
//
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SalLineLossTypeCommand(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((u16_Product == DDHD) || (BGVAR(u16_Power_Hw_Features) & 0x0002))
   {
      if ( (BGVAR(u16_Power_Hw_Features) & 0x0004) && (lparam == 2LL) )  return (NOT_SUPPORTED_ON_HW);  // 3 phase bus power supply is not supported

      BGVAR(u16_Line_Loss_Type) = (unsigned int)lparam;
      return (SAL_SUCCESS);
   }
   else if (lparam == 0)
   {
      return (SAL_SUCCESS);
   }
   else
   {
      return (NOT_SUPPORTED_ON_HW);
   }
}


int SalWriteEtherCATPdoMappingsToRs232(void)
{
   static int s16_state_machine = 0;
   static unsigned int u16_index = 0;     // Index that counts from 0...3 (Max. 4 RxPDOs and 4 TxPDOs)
   static unsigned int u16_index_2 = 0;   // Index that countes from 0...(X-1) (X = NumberOfObjects within one PDO)
   unsigned int u16_index_3 = 0;          // Index which is used to search through the "FB_objects_array"
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (u8_Output_Buffer_Free_Space < 250)
   {
      return SAL_NOT_FINISHED;
   }

   // If no EtherCAT Drive or COMMODE unequal 1
   if (!IS_EC_DRIVE_AND_COMMODE_1 && !IS_CAN_DRIVE_AND_COMMODE_1)
   {
      PrintStringCrLf("No EtherCAT/CAN Drive or communication-mode unequal 1.",0);
      s16_state_machine = 8; // Jump to the default case
   }
   if(IS_EC_DRIVE_AND_COMMODE_1)
   {
   switch(s16_state_machine)
   {
      case (0): // Print "RxPDO X" string
         PrintCrLf();PrintString("RxPDO ",0);PrintUnsignedInt16(u16_index+1);PrintCrLf();
         s16_state_machine += 1;
      break;

      case(1): // Print number of objects
         if (p_rpdo_entries[u16_index].u16_num_of_objects > 0)
         {
            PrintString(" Number of objects: ",0);PrintUnsignedInt16(p_rpdo_entries[u16_index].u16_num_of_objects);PrintCrLf();
            s16_state_machine += 1;
         }
         else
         {
            PrintStringCrLf(" No objects mapped.",0);
            s16_state_machine += 2;
         }
      break;

      case(2): // Print entries in the mapping
         if (u16_index_2 < p_rpdo_entries[u16_index].u16_num_of_objects)
         {
            // Here search for the matching entry within "FB_objects_array" in order to display the Object Index-number, SubIndex number and size.
            for(u16_index_3 = 1; u16_index_3 < SDO_LAST_OBJECT; u16_index_3++)
            {
               if (p_rpdo_entries[u16_index].map_array[u16_index_2].u32_addr == FB_objects_array[u16_index_3].u32_addr)
               {
                  PrintString("  Index: ",0); PrintDecInAsciiHex((unsigned long)FB_objects_array[u16_index_3].u16_index, 4);
                  PrintString(", Sub-Index: ",0); PrintDecInAsciiHex((unsigned long)FB_objects_array[u16_index_3].u8_sub_index, 2);
                  PrintString(", Size (in Bytes): ",0); PrintDecInAsciiHex((unsigned long)(FB_Pdos_Array[p_rpdo_entries[u16_index].map_array[u16_index_2].u16_id].u16_var_size & 0x00FF),2);
                  break;
               }
            }
            // If matching object was not found within "FB_objects_array"
            if (u16_index_3 >= SDO_LAST_OBJECT)
            {
               // Should usually not happen but print at leastPrint at least entry in the "FB_Pdos_Array"
               PrintString("  Entry: ",0);PrintUnsignedInt16(p_rpdo_entries[u16_index].map_array[u16_index_2].u16_id);
            }

            PrintCrLf();
            u16_index_2++;
         }
         else
         {
            s16_state_machine += 1;
         }
      break;

      case(3): // Restart printing RxPDOs until maximum number of PDOs reached
         u16_index++;
         u16_index_2 = 0;
        if (u16_index < RPDO_MAX_NUM)
         {
            s16_state_machine = 0;
         }
         else
         {
            s16_state_machine += 1;
            u16_index = 0;
         }
      break;

      case (4): // Print "TxPDO X" string
         PrintCrLf();PrintString("TxPDO ",0);PrintUnsignedInt16(u16_index+1);PrintCrLf();
         s16_state_machine += 1;
      break;

      case(5):  // Print number of objects
         if (p_tpdo_entries[u16_index].u16_num_of_objects > 0)
         {
            PrintString(" Number of objects: ",0);PrintUnsignedInt16(p_tpdo_entries[u16_index].u16_num_of_objects);PrintCrLf();
            s16_state_machine += 1;
         }
         else
         {
            PrintStringCrLf(" No objects mapped.",0);
            s16_state_machine += 2;
         }
      break;

      case(6): // Print entries in the mapping
         if (u16_index_2 < p_tpdo_entries[u16_index].u16_num_of_objects)
         {
            // Here search for the matching entry within "FB_objects_array" in order to display the Object Index-number, SubIndex number and size.
            for(u16_index_3 = 1; u16_index_3 < SDO_LAST_OBJECT; u16_index_3++)
            {
               if (p_tpdo_entries[u16_index].map_array[u16_index_2].u32_addr == FB_objects_array[u16_index_3].u32_addr)
               {
                  PrintString("  Index: ",0); PrintDecInAsciiHex((unsigned long)FB_objects_array[u16_index_3].u16_index, 4);
                  PrintString(", Sub-Index: ",0); PrintDecInAsciiHex((unsigned long)FB_objects_array[u16_index_3].u8_sub_index, 2);
                  PrintString(", Size (in Bytes): ",0); PrintDecInAsciiHex((unsigned long)(FB_Pdos_Array[p_tpdo_entries[u16_index].map_array[u16_index_2].u16_id].u16_var_size & 0x00FF),2);
                  break;
               }
            }
            // If matching object was not found within "FB_objects_array"
            if (u16_index_3 >= SDO_LAST_OBJECT)
            {
               // Should usually not happen but print at leastPrint at least entry in the "FB_Pdos_Array"
               PrintString("  Entry: ",0);PrintUnsignedInt16(p_tpdo_entries[u16_index].map_array[u16_index_2].u16_id);
            }
            PrintCrLf();
            u16_index_2++;
         }
         else
         {
            s16_state_machine += 1;
         }
      break;

      case(7): // Restart printing RxPDOs until maximum number of PDOs reached
         u16_index++;
         u16_index_2 = 0;
         if (u16_index < TPDO_MAX_NUM)
         {
            s16_state_machine = 4;
         }
         else
         {
            s16_state_machine += 1;
            u16_index = 0;
         }
      break;

      default:
         // Indicate end of function
         s16_state_machine = 0xFFFF;
      break;
   }
   }
   else// if(IS_CAN_DRIVE_AND_COMMODE_1)
   {
   switch(s16_state_machine)
   {
      case(0): // Print RPDO 1
         PrintCrLf();PrintString("RxPDO 1",0);
         if (p301_n1_rpdo_map.numOfEntries > 0)
         {
            PrintString(" Number of objects: ",0);PrintUnsignedInt16(p301_n1_rpdo_map.numOfEntries);PrintCrLf();

            for(u16_index_3 = 0; u16_index_3 < p301_n1_rpdo_map.numOfEntries; u16_index_3++)
            {
               PrintString("  Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n1_rpdo_map.map[u16_index_3] >>16) & 0xFFFF), 4);
               PrintString(", Sub-Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n1_rpdo_map.map[u16_index_3] >>8) & 0xFF), 2);
               PrintString(", Size (in Bytes): ",0); PrintDecInAsciiHex((unsigned long)(((p301_n1_rpdo_map.map[u16_index_3]) & 0xFF) >> 3),2);
               PrintCrLf();
            }                        
         }
         else
         {
            PrintStringCrLf(" No objects mapped.",0);
         }
		 s16_state_machine++;
      break;

      case(1): // Print RPDO 2
         PrintCrLf();PrintString("RxPDO 2",0);
         if (p301_n2_rpdo_map.numOfEntries > 0)
         {
            PrintString(" Number of objects: ",0);PrintUnsignedInt16(p301_n2_rpdo_map.numOfEntries);PrintCrLf();

            for(u16_index_3 = 0; u16_index_3 < p301_n2_rpdo_map.numOfEntries; u16_index_3++)
            {
               PrintString("  Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n2_rpdo_map.map[u16_index_3] >>16) & 0xFFFF), 4);
               PrintString(", Sub-Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n2_rpdo_map.map[u16_index_3] >>8) & 0xFF), 2);
               PrintString(", Size (in Bytes): ",0); PrintDecInAsciiHex((unsigned long)(((p301_n2_rpdo_map.map[u16_index_3]) & 0xFF) >> 3),2);
               PrintCrLf();
            }                        
         }
         else
         {
            PrintStringCrLf(" No objects mapped.",0);
         }
		 s16_state_machine++;
      break;

	  case(2): // Print RPDO 3
         PrintCrLf();PrintString("RxPDO 3",0);
         if (p301_n3_rpdo_map.numOfEntries > 0)
         {
            PrintString(" Number of objects: ",0);PrintUnsignedInt16(p301_n3_rpdo_map.numOfEntries);PrintCrLf();

            for(u16_index_3 = 0; u16_index_3 < p301_n1_rpdo_map.numOfEntries; u16_index_3++)
            {
               PrintString("  Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n1_rpdo_map.map[u16_index_3] >>16) & 0xFFFF), 4);
               PrintString(", Sub-Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n1_rpdo_map.map[u16_index_3] >>8) & 0xFF), 2);
               PrintString(", Size (in Bytes): ",0); PrintDecInAsciiHex((unsigned long)(((p301_n1_rpdo_map.map[u16_index_3]) & 0xFF) >> 3),2);
               PrintCrLf();
            }                        
         }
         else
         {
            PrintStringCrLf(" No objects mapped.",0);
         }
		 s16_state_machine++;
      break;

	     case(3): // Print RPDO 4
            PrintCrLf();PrintString("RxPDO 4",0);
            if (p301_n4_rpdo_map.numOfEntries > 0)
            {
               PrintString(" Number of objects: ",0);PrintUnsignedInt16(p301_n4_rpdo_map.numOfEntries);PrintCrLf();

               for(u16_index_3 = 0; u16_index_3 < p301_n4_rpdo_map.numOfEntries; u16_index_3++)
               {
                  PrintString("  Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n4_rpdo_map.map[u16_index_3] >>16) & 0xFFFF), 4);
                  PrintString(", Sub-Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n4_rpdo_map.map[u16_index_3] >>8) & 0xFF), 2);
                  PrintString(", Size (in Bytes): ",0); PrintDecInAsciiHex((unsigned long)(((p301_n4_rpdo_map.map[u16_index_3]) & 0xFF) >> 3),2);
                  PrintCrLf();
               }                        
            }
            else
            {
               PrintStringCrLf(" No objects mapped.",0);
            }
		    s16_state_machine++;
      break;

      case(4): // Print TPDO 1
         PrintCrLf();PrintString("TxPDO 1",0);
         if (p301_n1_tpdo_map.numOfEntries > 0)
         {
            PrintString(" Number of objects: ",0);PrintUnsignedInt16(p301_n1_tpdo_map.numOfEntries);PrintCrLf();

            for(u16_index_3 = 0; u16_index_3 < p301_n1_tpdo_map.numOfEntries; u16_index_3++)
            {
               PrintString("  Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n1_tpdo_map.map[u16_index_3] >>16) & 0xFFFF), 4);
               PrintString(", Sub-Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n1_tpdo_map.map[u16_index_3] >>8) & 0xFF), 2);
               PrintString(", Size (in Bytes): ",0); PrintDecInAsciiHex((unsigned long)(((p301_n1_tpdo_map.map[u16_index_3]) & 0xFF) >> 3),2);
               PrintCrLf();
            }                        
         }
         else
         {
            PrintStringCrLf(" No objects mapped.",0);
         }
		 s16_state_machine++;
      break;

      case(5): // Print TPDO 2
         PrintCrLf();PrintString("TxPDO 2",0);
         if (p301_n2_tpdo_map.numOfEntries > 0)
         {
            PrintString(" Number of objects: ",0);PrintUnsignedInt16(p301_n2_tpdo_map.numOfEntries);PrintCrLf();

            for(u16_index_3 = 0; u16_index_3 < p301_n2_tpdo_map.numOfEntries; u16_index_3++)
            {
               PrintString("  Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n2_tpdo_map.map[u16_index_3] >>16) & 0xFFFF), 4);
               PrintString(", Sub-Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n2_tpdo_map.map[u16_index_3] >>8) & 0xFF), 2);
               PrintString(", Size (in Bytes): ",0); PrintDecInAsciiHex((unsigned long)(((p301_n2_tpdo_map.map[u16_index_3]) & 0xFF) >> 3),2);
               PrintCrLf();
            }                        
         }
         else
         {
            PrintStringCrLf(" No objects mapped.",0);
         }
		 s16_state_machine++;
      break;

	  case(6): // Print TPDO 3
         PrintCrLf();PrintString("TxPDO 3",0);
         if (p301_n3_tpdo_map.numOfEntries > 0)
         {
            PrintString(" Number of objects: ",0);PrintUnsignedInt16(p301_n3_tpdo_map.numOfEntries);PrintCrLf();

            for(u16_index_3 = 0; u16_index_3 < p301_n1_tpdo_map.numOfEntries; u16_index_3++)
            {
               PrintString("  Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n1_tpdo_map.map[u16_index_3] >>16) & 0xFFFF), 4);
               PrintString(", Sub-Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n1_tpdo_map.map[u16_index_3] >>8) & 0xFF), 2);
               PrintString(", Size (in Bytes): ",0); PrintDecInAsciiHex((unsigned long)(((p301_n1_tpdo_map.map[u16_index_3]) & 0xFF) >> 3),2);
               PrintCrLf();
            }                        
         }
         else
         {
            PrintStringCrLf(" No objects mapped.",0);
         }
		 s16_state_machine++;
      break;

	     case(7): // Print TPDO 4
            PrintCrLf();PrintString("TxPDO 4",0);
            if (p301_n4_tpdo_map.numOfEntries > 0)
            {
               PrintString(" Number of objects: ",0);PrintUnsignedInt16(p301_n4_tpdo_map.numOfEntries);PrintCrLf();

               for(u16_index_3 = 0; u16_index_3 < p301_n4_tpdo_map.numOfEntries; u16_index_3++)
               {
                  PrintString("  Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n4_tpdo_map.map[u16_index_3] >>16) & 0xFFFF), 4);
                  PrintString(", Sub-Index: ",0); PrintDecInAsciiHex((unsigned long)((p301_n4_tpdo_map.map[u16_index_3] >>8) & 0xFF), 2);
                  PrintString(", Size (in Bytes): ",0); PrintDecInAsciiHex((unsigned long)(((p301_n4_tpdo_map.map[u16_index_3]) & 0xFF) >> 3),2);
                  PrintCrLf();
               }                        
            }
            else
            {
               PrintStringCrLf(" No objects mapped.",0);
            }
		    s16_state_machine++;
      break;
      default:
         // Indicate end of function
         s16_state_machine = 0xFFFF;
      break;
   }
   }
   if (s16_state_machine == 0xFFFF)
   {
      // Re-initialize all relevant variables
      s16_state_machine = 0;
      u16_index = 0;
      u16_index_2 = 0;
      // Return success
      return (SAL_SUCCESS);
   }
   else
   {
      return (SAL_NOT_FINISHED);
   }
}


//**********************************************************
// Function Name: PrintOnFdbkCommFaultMessage
// Description:
//   This function prints a message when feedback communication fault occurs.
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
void PrintOnFdbkCommFaultMessage(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_TIME_OUT_ERROR_MASK)
      PrintStringCrLf("Timeout fault detected on protocol level", 0);

   if (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_ERROR_MASK)
      PrintStringCrLf("CRC fault detected on protocol level", 0);

   PrintStringCrLf("Issue CLEARFAULTS", 0);
}


//**********************************************************
// Function Name: SalSrvSnsFWDnLdCommand
// Description:
//   This function called in response to SRVSNSFWDNLD command
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int SalSrvSnsFWDnLdCommand(int drive)
{
   static unsigned long u32_sem = 0;
   static long s32_encoder_comm_timer = 0;
   int ret_val = SAL_NOT_FINISHED, timeout_occurred = 0;
   unsigned char u8_data_byte = 0;
   static volatile struct SCI_REGS* pSCI;
   static unsigned int enc_source = 0;
   // AXIS_OFF;

    if (Enabled(drive))
        return DRIVE_ACTIVE;

    /* Do not continue if ServoSense is not in use */
    if (  (!FEEDBACK_SERVOSENSE) )
    {
        BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_PREACQUIRE;
        return NOT_AVAILABLE;
    }

   /* Do not continue if ServoSense comm fault is detected and FW presents of the device*/
   if ((VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PRESENT_MASK) && (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK))
   {
      InitSerCommunication();

      PrintOnFdbkCommFaultMessage(drive);

      // release ServoSense driver
      SrvSns_Release(&u32_sem, drive);
      VAR(AX0_u16_SrvSns_FWStatus) &= SRVSNS_FW_STATUS_ERROR_MASK; // clear all bits, but error bit
      BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_PREACQUIRE;

      if (BGVAR(s64_SysNotOk_2) & SRVSNS_ENCODER_FLT_MASK)// if going from boot mode, clear ServoSense fault
      {
         SrvSnsFault(drive, FLT_CLR);
      }
      else
      {
         BGVAR(u16_CommFdbk_Init_Select) = 0;
         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;     // Restart feedback state machine
      }

      return SAL_SUCCESS;
   }

   if (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK)
   {
      return NOT_AVAILABLE;
   }

   if (VAR(AX0_u16_SrvSns_FWStatus) & (SRVSNS_FW_STATUS_OP_CORRUPTED_MASK | SRVSNS_FW_STATUS_BOOT_CORRUPTED_MASK))
   {
      return NOT_AVAILABLE;
   }

   switch(BGVAR(u16_SrvSns_FWUpgradeState))
   {
      case SRVSNS_FWDNLD_STATE_PREACQUIRE:
         // take timeout for acquire
         s32_encoder_comm_timer = Cntr_1mS;
         BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_ACQUIRE;
      break;

      case SRVSNS_FWDNLD_STATE_ACQUIRE:
         if (SrvSns_Acquire(&u32_sem, drive) != SAL_SUCCESS)
         {
           if (PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC, s32_encoder_comm_timer))
                ret_val = SRVSNS_DRV_ACQ_TIMEOUT;
           break;
         }

         // stop RT communication
         VAR(AX0_u16_Comm_Fdbk_Flags_A_1) &= ~(COMM_FDBK_INIT_ABS_POS_MASK | COMM_FDBK_READY_MASK);

         // init ServoSense state machines
         SrvSns_InitStateMachines(drive);

         BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_INIT_PC_PORT;
      break;

      case SRVSNS_FWDNLD_STATE_INIT_PC_PORT:// initialize PC communication
         VAR(AX0_u16_SrvSns_FWStatus) |= SRVSNS_FW_STATUS_FWDNLD_IDLE_MASK;
         VAR(AX0_u16_SrvSns_FWStatus) &= ~SRVSNS_FW_STATUS_ERROR_MASK;

         InitSerCommunication();

         pSCI = (u16_Product == SHNDR_HW) ? &ScicRegs : &ScibRegs;
         // clear serial input FIFO
         while(pSCI->SCIFFRX.bit.RXFFST)
            u8_data_byte = pSCI->SCIRXBUF.bit.RXDT;

         BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_INIT_FPGA_PORT;
      break;

      case SRVSNS_FWDNLD_STATE_INIT_FPGA_PORT:// initialize FPGA communication
         s32_encoder_comm_timer = Cntr_1mS;
         // wait till previous transmission finished
         while((*(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD) & 0x0002)
         {
            if (PassedTimeMS(SRVSNS_TX_DONE_TIMEOUT_mSEC, s32_encoder_comm_timer))
            {
               VAR(AX0_u16_SrvSns_FWStatus) |= SRVSNS_FW_STATUS_ERROR_MASK;
               timeout_occurred = 1;
               break;
            }
         }
         if (timeout_occurred)
         {
            ret_val = SRVSNS_BUSY_TIMEOUT;
            break;
         }

         s32_encoder_comm_timer = Cntr_1mS;

         // Clear encoder last response
         while(*(unsigned int *)FPGA_RX_DATA_READY_REG_ADD == 1)
         {
            // trig to reset data ready register
            *(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD = 1;
            *(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD = 0;

            u8_data_byte = *(unsigned int *)FPGA_MFB_RX_BUFFER_ADD;
            *(unsigned int *)FPGA_MFB_RX_BUFFER_ADD = 0;

            if (PassedTimeMS(100L, s32_encoder_comm_timer))
            {
               timeout_occurred = 1;
               break;
            }
         }
         if (timeout_occurred)
         {
            ret_val = SRVSNS_BUSY_TIMEOUT;
            break;
         }
         if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PRESENT_MASK)
         {
            BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_RESET_ENC;
         }
         else if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_BOOT_PRESENT_MASK)
         {
            BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_CONNECT;
         }
         ret_val = SAL_NOT_FINISHED;
      break;

      case SRVSNS_FWDNLD_STATE_RESET_ENC:// command FW download
         // load FW download command
         *(unsigned int*)FPGA_MFB_TX_BUFFER_ADD       = SRVSNS_FWDNLD_CMD1;
         *(unsigned int*)(FPGA_MFB_TX_BUFFER_ADD + 1) = SRVSNS_FWDNLD_CMD2;

         // toggle --> transmit data
         *(unsigned int *)FPGA_SANYO_DENKY_EN_REG_ADD = 0;
         *(unsigned int *)FPGA_SANYO_DENKY_EN_REG_ADD = 1;
         s32_encoder_comm_timer = Cntr_1mS;
         // wait till transmission finished
         while((*(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD) & 0x0002)
         {
            if (PassedTimeMS(SRVSNS_TX_DONE_TIMEOUT_mSEC, s32_encoder_comm_timer))
            {
                  timeout_occurred = 1;
                  break;
            }
         }
         if (timeout_occurred)
         {
            ret_val = SRVSNS_BUSY_TIMEOUT;
            break;
         }

         BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_CONFIG_FPGA;
         ret_val = SAL_NOT_FINISHED;
      break;

      case SRVSNS_FWDNLD_STATE_CONFIG_FPGA:// reconfigure FPGA
         SRVSNS_FPGA_CONFIG_BOOT_MODE;
         s32_encoder_comm_timer = Cntr_1mS;
         BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_CONNECT;
      break;

      case SRVSNS_FWDNLD_STATE_CONNECT:// get acknowledge byte 0xFC
         if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_PRESENT_MASK)
         {
            if (*(unsigned int *)FPGA_RX_DATA_READY_REG_ADD == 1)
            {
               // trig to reset data ready register
               *(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD = 1;
               *(unsigned int *)FPGA_RX_DATA_READY_RESET_REG_ADD = 0;

               // get byte from Encoder
               u8_data_byte = *(unsigned int *)FPGA_MFB_RX_BUFFER_ADD;

               // eliminate UART frame
               u8_data_byte >>= 1;
               u8_data_byte &= 0xFF;

               if (u8_data_byte == SRVSNS_FWDNLD_ACK_BYTE)
               {
                  VAR(AX0_u16_SrvSns_FWStatus) |= SRVSNS_FW_STATUS_FWDNLD_RETRANSMIT_MASK;
                  BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_RETRANSMIT;
               }
            }
            if (PassedTimeMS(100L, s32_encoder_comm_timer))
            {
               ret_val = SRVSNS_BUSY_TIMEOUT;
            }
         }
         else
         {
            // emulate ServoSense bootloader ACK to provide PC upgrader connection hook:

            //  Copy data to the output buffer
            *p_u8_Out_Buf_In_Ptr = SRVSNS_FWDNLD_ACK_BYTE;

            //  Increment output buffer inp pointer
            IncrementPtr(p_u8_Out_Buf_In_Ptr,u8_Output_Buffer,p_u8_Out_Buf_End);

            //  Decrement free space counter
            u8_Output_Buffer_Free_Space--;

            VAR(AX0_u16_SrvSns_FWStatus) |= SRVSNS_FW_STATUS_FWDNLD_RETRANSMIT_MASK;
            BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_RETRANSMIT;
         }
      break;

      case SRVSNS_FWDNLD_STATE_RETRANSMIT:// start retransmit, wait for finish
         if ((VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_FWDNLD_RETRANSMIT_MASK) == 0)
            ret_val = SAL_SUCCESS;
      break;

      default:
         ret_val = SRVSNS_DRV_ERROR;
      break;
   }

   if (ret_val != SAL_NOT_FINISHED)
   {
      InitSerCommunication();
      // release ServoSense driver
      SrvSns_Release(&u32_sem, drive);
      VAR(AX0_u16_SrvSns_FWStatus) &= SRVSNS_FW_STATUS_ERROR_MASK; // clear all bits, but error bit
      BGVAR(u16_SrvSns_FWUpgradeState) = SRVSNS_FWDNLD_STATE_PREACQUIRE;

      if (BGVAR(s64_SysNotOk_2) & SRVSNS_ENCODER_FLT_MASK)// if going from boot mode, clear ServoSense fault
      {
         SrvSnsFault(drive, FLT_CLR);
      }
      else
      {
         BGVAR(u16_CommFdbk_Init_Select) = 0;
         BGVAR(s16_Comm_Fdbk_Init_State) = COMM_FDBK_START_INIT;     // Restart feedback state machine
      }

      u32_sem = 0;
   }

   return ret_val;
}


//**********************************************************
// Function Name: SalSrvSnsInfoCommand
// Description:
//   This function is called in response to SRVSNSINFO serial command
//
//
// Author: Anatoly Odler / Gil
// Algorithm:
// Revisions:
// Note:
//********************************************************************
int SalSrvSnsInfoCommand(int drive)
{
   return (SrvSnsInfoCommand(drive, FB_NOT_IN_USE, 0));
}


//**********************************************************
// Function Name: SrvSnsInfoCommand
// Description:
//   This function is called in response to SRVSNSINFO command
//
//
// Author: Anatoly Odler / Gil
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int SrvSnsInfoCommand(int drive,int s16_fb_use, int u16_fdbk_source)
{
   static int state = 0;
   static long s32_srvsns_response = 0, s32_acquisition_timer = 0;
   static unsigned long u32_owner = 0;
   static char s8_product_info[SRVSNS_PRD_INFO_AREA_RSRVD - SRVSNS_PRODUCT_INFO_AREA_BASE];
   static unsigned int u16_product_info_addr = SRVSNS_PRODUCT_INFO_AREA_BASE;
   int ret_val = SAL_NOT_FINISHED, s16_velocity = 0;
   unsigned int u16_secs, u16_mins, u16_hours, u16_days;

   // AXIS_OFF;

   /* Do not continue if ServoSense is not in use or not ready */
   if (  (!FEEDBACK_SERVOSENSE)  )
   {
       state = 0;
       return NOT_AVAILABLE;
   }

   if (s16_fb_use == FB_NOT_IN_USE)
   {
      if (OutputBufferLoader()==0) return SAL_NOT_FINISHED;
   }
   strcpy((char*)&u16_General_Domain[0],"\0");

   if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_ACQUIRE_FAILED_MASK)
   {
      strcat((char*)&u16_General_Domain[0],"sensAR acquisition failed on power up!\r\n");
      if (s16_fb_use == FB_NOT_IN_USE)
      {  OutputBufferLoader();     }
      state = 0;
      return SAL_SUCCESS;
   }

   if ((VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_ON_INIT_PROC_FAILED_MASK) && (BGVAR(u16_SrvSns_InitProcErrorCode) == SRVSNS_BUSY_TIMEOUT))
   {
      strcat((char*)&u16_General_Domain[0],"sensAR encoder initialization failed with code: ");
      strcat((char*)&u16_General_Domain[0],UnsignedIntegerToAscii(SRVSNS_BUSY_TIMEOUT));
      strcat((char*)&u16_General_Domain[0],"\r\n");

      if (s16_fb_use == FB_NOT_IN_USE)
      {  OutputBufferLoader();     }

      state = 0;
      return SAL_SUCCESS;
   }

   if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_NOT_CONNECTED_MASK)
   {
      strcat((char*)&u16_General_Domain[0],"sensAR encoder is neither connected nor FW is installed on it.\r\n");
      if (s16_fb_use == FB_NOT_IN_USE)
      {  OutputBufferLoader();     }
      state = 0;
      return SAL_SUCCESS;
   }


   if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_BOOT_PRESENT_MASK)
   {
      strcat((char*)&u16_General_Domain[0],"sensAR encoder is in boot mode\r\n");
      strcat((char*)&u16_General_Domain[0],"Issue FW download\r\n");
      if (s16_fb_use == FB_NOT_IN_USE)
      {  OutputBufferLoader();     }
      state = 0;
      return SAL_SUCCESS;
   }

   if (VAR(AX0_u16_SrvSns_FWStatus) & (SRVSNS_FW_STATUS_OP_CORRUPTED_MASK | SRVSNS_FW_STATUS_BOOT_CORRUPTED_MASK))
   {
       strcat((char*)&u16_General_Domain[0],"The sensAR FW is corrupted or not present\r\n");
       if (s16_fb_use == FB_NOT_IN_USE)
       {  OutputBufferLoader();     }
       state = 0;
       return SAL_SUCCESS;
   }

   if (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK)
   {
       if (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_TIME_OUT_ERROR_MASK)
       {
            strcat((char*)&u16_General_Domain[0],"Timeout fault detected on protocol level\r\n");
       }
       if (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_ERROR_MASK)
       {
            strcat((char*)&u16_General_Domain[0],"CRC fault detected on protocol level\r\n");
       }
       strcat((char*)&u16_General_Domain[0],"Issue CLEARFAULTS\r\n");

      /* Release ServoSense device */
      SrvSns_Release(&u32_owner, drive);
      u32_owner = 0;
      s32_acquisition_timer = 0;
      s32_srvsns_response = 0;
      u16_product_info_addr = SRVSNS_PRODUCT_INFO_AREA_BASE;
      state = 0;
      if (s16_fb_use == FB_NOT_IN_USE)
      {  OutputBufferLoader();     }

      return SAL_SUCCESS;
   }

   if (BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK)
   {
      state = 0;
      return NOT_AVAILABLE;
   }

   switch(state)
   {
      case 0:// take acquisition timer
            // check if we're communicating to the encoder
            if ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) == 0)
            {
               ret_val = NOT_AVAILABLE;
               break;
            }
            // initialize product info array
            memset((void*)s8_product_info, 0, SRVSNS_PRD_INFO_AREA_RSRVD - SRVSNS_PRODUCT_INFO_AREA_BASE);
            u16_product_info_addr = SRVSNS_PRODUCT_INFO_AREA_BASE;
            // Print label
            strcat((char*)&u16_General_Domain[0],"sensAR Magnetic Encoder\r\n");
            strcat((char*)&u16_General_Domain[0],"------------------------------\r\n");

            if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_ON_INIT_PROC_FAILED_MASK)
            {
               strcat((char*)&u16_General_Domain[0],"sensAR encoder initialization procedure error code: ");
               strcat((char*)&u16_General_Domain[0],UnsignedIntegerToAscii((long)BGVAR(u16_SrvSns_InitProcErrorCode)));
               strcat((char*)&u16_General_Domain[0],"\r\n");
               strcat((char*)&u16_General_Domain[0],"\r\n");
            }
            s32_acquisition_timer = Cntr_1mS;
            state++;

            break;

       case 1: // Acquire ServoSense
            // Acquire ServoSense device
            ret_val = SrvSns_Acquire(&u32_owner, drive);
            if (ret_val == SAL_SUCCESS)
            {
               // Print production info label
               strcat((char*)&u16_General_Domain[0],"Production Information:\r\n");
               ret_val = SAL_NOT_FINISHED;
               state++;
               break;
            }
            if (PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC, s32_acquisition_timer))
               ret_val = SRVSNS_DRV_ACQ_TIMEOUT;
       break;

       case 2:
            ret_val = SrvSnsReadAddr(u32_owner, u16_product_info_addr, &s32_srvsns_response, drive, u16_fdbk_source);
            if (ret_val == SAL_SUCCESS)
            {
               unpack_memcpy((UNSIGNED8*)&s8_product_info[(u16_product_info_addr - SRVSNS_PRODUCT_INFO_AREA_BASE)],
                              (UNSIGNED8*)&s32_srvsns_response, SRVSNS_ADDRESS_ALIGNMENT, 1);

               u16_product_info_addr += SRVSNS_ADDRESS_ALIGNMENT;
               if (u16_product_info_addr == SRVSNS_PRD_INFO_AREA_RSRVD)
               {
                  /* Split into three states to prevent output buffer overflow */
                  state++;
               }
               ret_val = SAL_NOT_FINISHED;
            }
            else if ((ret_val == SRVSNS_ADDR_OUT_OF_RANGE) || (ret_val == NOT_AVAILABLE))
            {
               strcat((char*)&u16_General_Domain[0],"Not Available\r\n");
               state = 5;
               ret_val = SAL_NOT_FINISHED;
            }
            break;

       case 3:
            strcat((char*)&u16_General_Domain[0],"\r\nEncoder:\r\n");
            strcat((char*)&u16_General_Domain[0],"P/N:      ");
            strcat((char*)&u16_General_Domain[0],(char *)&s8_product_info[SRVSNS_PRD_INFO_AREA_ENCPN_OFF]);
            strcat((char*)&u16_General_Domain[0],"\r\nRev:      ");
            strcat((char*)&u16_General_Domain[0],(char *)&s8_product_info[SRVSNS_PRD_INFO_AREA_ENCREV_OFF]);
            strcat((char*)&u16_General_Domain[0],"\r\nS/N:      ");
            strcat((char*)&u16_General_Domain[0],(char *)&s8_product_info[SRVSNS_PRD_INFO_AREA_ENCSN_OFF]);
            strcat((char*)&u16_General_Domain[0],"\r\n");
            state++;
            break;

       case 4:
            strcat((char*)&u16_General_Domain[0],"\r\nStator Assembly:\r\nP/N:      ");
            strcat((char*)&u16_General_Domain[0],(char *)&s8_product_info[SRVSNS_PRD_INFO_AREA_STRPN_OFF]);
            strcat((char*)&u16_General_Domain[0],"\r\nRev:      ");
            strcat((char*)&u16_General_Domain[0],(char *)&s8_product_info[SRVSNS_PRD_INFO_AREA_STRREV_OFF]);
            strcat((char*)&u16_General_Domain[0],"\r\nS/N:      ");
            strcat((char*)&u16_General_Domain[0],(char *)&s8_product_info[SRVSNS_PRD_INFO_AREA_STRSN_OFF]);
            strcat((char*)&u16_General_Domain[0],"\r\n");
            state++;
            break;

       case 5: // Print HW revision value
            ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_PRD_INFO_AREA_HWREV, &s32_srvsns_response, drive, u16_fdbk_source);
            if (ret_val == SAL_SUCCESS)
            {
               // Print HW revision label
               strcat((char*)&u16_General_Domain[0],"\r\nHardware:");
               strcat((char*)&u16_General_Domain[0],"\r\nPCB Rev:   ");
               strcat((char*)&u16_General_Domain[0], DecToAsciiHex(((unsigned long long)s32_srvsns_response) >> 16, 4));
               strcat((char*)&u16_General_Domain[0],"\r\nBOM Rev:   ");
               strcat((char*)&u16_General_Domain[0], DecToAsciiHex((unsigned long long)s32_srvsns_response & 0x0000FFFF, 4));
               s32_srvsns_response = 0;
               ret_val = SAL_NOT_FINISHED;
               state++;
            }
            break;

       case 6:// CPU silicon revision
          ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_PRD_INFO_AREA_CPU_REV, &s32_srvsns_response, drive, u16_fdbk_source);
          if (ret_val == SAL_SUCCESS)
          {
             strcat((char*)&u16_General_Domain[0],"\r\nCPU Rev:   ");
             strcat((char*)&u16_General_Domain[0], DecToAsciiHex(((unsigned long long)s32_srvsns_response), 4));
             s32_srvsns_response = 0;
             ret_val = SAL_NOT_FINISHED;
             state++;
          }
          else if ((ret_val == SRVSNS_ADDR_OUT_OF_RANGE) || (ret_val == NOT_AVAILABLE))
          {
             strcat((char*)&u16_General_Domain[0],"\r\nCPU Rev:   Not Available");
             ret_val = SAL_NOT_FINISHED;
             state++;
          }
          break;

       case 7:// Product type: single turn/multi turn
         strcat((char*)&u16_General_Domain[0],"\nProduct Type:   ");
         if(BGVAR(u16_SrvSns_ProductType) == 0)
                strcat((char*)&u16_General_Domain[0],"Absolute Single-Turn");
         else if(BGVAR(u16_SrvSns_ProductType) == 1)
            strcat((char*)&u16_General_Domain[0],"Absolute Battery Buffered Multi-Turn");
         else
            strcat((char*)&u16_General_Domain[0],"Unknown Model");

             strcat((char*)&u16_General_Domain[0],"\r\n\r\n");// new line
             u16_product_info_addr = SRVSNS_PRD_INFO_AREA_FWVER;
             s32_srvsns_response = 0;
             ret_val = SAL_NOT_FINISHED;
             state++;
          break;


       case 8: // FW Version
            ret_val = SrvSnsReadAddr(u32_owner, u16_product_info_addr, &s32_srvsns_response, drive, u16_fdbk_source);
            if (ret_val == SAL_SUCCESS)
            {
               unpack_memcpy((UNSIGNED8*)&s8_product_info[(u16_product_info_addr - SRVSNS_PRD_INFO_AREA_FWVER)],
                              (UNSIGNED8*)&s32_srvsns_response, SRVSNS_ADDRESS_ALIGNMENT, 1);

               u16_product_info_addr += SRVSNS_ADDRESS_ALIGNMENT;
               if (u16_product_info_addr == (SRVSNS_PRD_INFO_AREA_FWVER + SRVSNS_PRD_INFO_AREA_FWVER_LEN))
               {
                  // FW Version label
                  strcat((char*)&u16_General_Domain[0],"Software:\nFW Version:    ");
                  strcat((char*)&u16_General_Domain[0],(char *)&s8_product_info[0]);
                  strcat((char*)&u16_General_Domain[0],"\r\n");
                  u16_product_info_addr = SRVSNS_PRD_INFO_AREA_FWDATE;
                  s32_srvsns_response = 0;
                  state++;
               }
               ret_val = SAL_NOT_FINISHED;
            }
            break;

      case 9: // FW Version date
            ret_val = SrvSnsReadAddr(u32_owner, u16_product_info_addr, &s32_srvsns_response, drive, u16_fdbk_source);
            if (ret_val == SAL_SUCCESS)
            {
               unpack_memcpy((UNSIGNED8*)&s8_product_info[(u16_product_info_addr - SRVSNS_PRD_INFO_AREA_FWDATE)],
                              (UNSIGNED8*)&s32_srvsns_response, SRVSNS_ADDRESS_ALIGNMENT, 1);

               u16_product_info_addr += SRVSNS_ADDRESS_ALIGNMENT;
               if (u16_product_info_addr == (SRVSNS_PRD_INFO_AREA_FWDATE + SRVSNS_PRD_INFO_AREA_FWDATE_LEN))
               {
                  strcat((char*)&u16_General_Domain[0],"FW Version Date:    ");
                  strcat((char*)&u16_General_Domain[0],(char *)&s8_product_info[0]);
                  strcat((char*)&u16_General_Domain[0],"\r\n");
                  u16_product_info_addr = SRVSNS_PRD_INFO_AREA_PRTCLV;
                  s32_srvsns_response = 0;
                  state++;
               }
               ret_val = SAL_NOT_FINISHED;
            }
            break;

       case 10: // Protocol Version
            ret_val = SrvSnsReadAddr(u32_owner, u16_product_info_addr, &s32_srvsns_response, drive, u16_fdbk_source);
            if (ret_val == SAL_SUCCESS)
            {
               unpack_memcpy((UNSIGNED8*)&s8_product_info[(u16_product_info_addr - SRVSNS_PRD_INFO_AREA_PRTCLV)],
                              (UNSIGNED8*)&s32_srvsns_response, SRVSNS_ADDRESS_ALIGNMENT, 1);

               u16_product_info_addr += SRVSNS_ADDRESS_ALIGNMENT;
               if (u16_product_info_addr == (SRVSNS_PRD_INFO_AREA_PRTCLV + SRVSNS_PRD_INFO_AREA_PRTCLV_LEN))
               {
                  // Protocol Version label
                  strcat((char*)&u16_General_Domain[0],"Comm Protocol Version:   ");
                  strcat((char*)&u16_General_Domain[0],(char *)&s8_product_info[0]);
                  strcat((char*)&u16_General_Domain[0],"\r\n");

                  // Boot version label
                  strcat((char*)&u16_General_Domain[0],"Boot FW Version:   ");
                  u16_product_info_addr = SRVSNS_PRD_INFO_AREA_BOOT_VER;
                  s32_srvsns_response = 0;
                  state++;
               }
               ret_val = SAL_NOT_FINISHED;
            }
            break;

       case 11: // Boot Version
            ret_val = SrvSnsReadAddr(u32_owner, u16_product_info_addr, &s32_srvsns_response, drive, u16_fdbk_source);
            if (ret_val == SAL_SUCCESS)
            {
               unpack_memcpy((UNSIGNED8*)&s8_product_info[(u16_product_info_addr - SRVSNS_PRD_INFO_AREA_BOOT_VER)],
                              (UNSIGNED8*)&s32_srvsns_response, SRVSNS_ADDRESS_ALIGNMENT, 1);

               s8_product_info[SRVSNS_ADDRESS_ALIGNMENT] = '\0';
               strcat((char*)&u16_General_Domain[0],(char *)&s8_product_info[0]);
               strcat((char*)&u16_General_Domain[0],"\r\n");
                  s32_srvsns_response = 0;
                  state++;
               ret_val = SAL_NOT_FINISHED;
               }
            else if ((ret_val == SRVSNS_ADDR_OUT_OF_RANGE) || (ret_val == NOT_AVAILABLE))
            {
               strcat((char*)&u16_General_Domain[0], "Not Available\r\n");
               ret_val = SAL_NOT_FINISHED;
               state++;
            }
            break;

       case 12: // Get Temperature value
            // Device Temperature label
            strcat((char*)&u16_General_Domain[0],"\r\nOnline data:\r\nDevice Temperature:    ");
            strcat((char*)&u16_General_Domain[0],SignedInt64ToAscii((long long)BGVAR(s16_SrvSns_Temperature)));
            strcat((char*)&u16_General_Domain[0],"C\r\n");
            state++;
            break;

       case 13:// Raw position and velocity readings
            strcat((char*)&u16_General_Domain[0],"Raw in-turn position value:    ");
            strcat((char*)&u16_General_Domain[0],UnsignedInt64ToAscii(VAR(AX0_u32_SrvSns_Abs_Pos_Raw)));

            if (SRVSNS_GET_SUBMODE() == SRVSNS_SUBMODE_VELOCITY)
            {
               strcat((char*)&u16_General_Domain[0],"\r\nRaw velocity value:    ");
               s16_velocity = (int)(LVAR(AX0_s32_SrvSns_Velocity_Raw) & 0x0000FFFF);
               strcat((char*)&u16_General_Domain[0],SignedInt64ToAscii((long long)s16_velocity));
            }
            else
            {
               strcat((char*)&u16_General_Domain[0],"\r\nRaw multi-turn position value:    ");
               strcat((char*)&u16_General_Domain[0], SignedInt64ToAscii((long long)VAR(AX0_s32_Comm_Abs_Pos_32_Hi)));
            }
            s32_srvsns_response = 0;
            strcat((char*)&u16_General_Domain[0],"\r\n");
            state++;
            break;

      case 14:// Command to get faults mask
            ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_INT_PAR_FLTS_MASK, (long*)&BGVAR(u32_SrvSns_FaultsIndications), drive, u16_fdbk_source);
            if (ret_val == SAL_SUCCESS)
            {
               state++;
               ret_val = SAL_NOT_FINISHED;
            }
            break;

       case 15: // Identify faults
            if (SrvSns_GetStatus(drive) & SRVSNS_STATUS_FAULT_MASK)
            {
               ret_val = SrvSnsSendCmd(u32_owner, SRVSNS_CMD_GET_FAULTS, 0, &s32_srvsns_response, drive, u16_fdbk_source);
               if (ret_val == SAL_SUCCESS)
               {
                  // remove the power down fault from the output
                  if ((s32_srvsns_response & SRVSNS_FAULT_POWER_DOWN_MASK) && (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_HELP_TASK_MASK))
                     s32_srvsns_response &= ~SRVSNS_FAULT_POWER_DOWN_MASK;

                  // Faults exist which are hidden by the internal mask
                  if (s32_srvsns_response & (SRVSNS_FAULT_INDICATIONS & (~BGVAR(u32_SrvSns_FaultsIndications))))
                  {
                     long s32_temp = s32_srvsns_response;
                     s32_temp &= (SRVSNS_FAULT_INDICATIONS & (~BGVAR(u32_SrvSns_FaultsIndications)));
                     strcat((char*)&u16_General_Domain[0],"Masked faults exist on sensAR: ");
                     strcat((char*)&u16_General_Domain[0],DecToAsciiHex((unsigned long)s32_temp,8));
                     strcat((char*)&u16_General_Domain[0],"\n");
                     s32_srvsns_response &= ~s32_temp;
                     state = 17;
                  }

                  // Faults exist which are not defined
                  if ((s32_srvsns_response & (~SRVSNS_FAULT_INDICATIONS)) != 0)
                  {
                     long s32_temp = s32_srvsns_response;
                     s32_temp &= ~SRVSNS_FAULT_INDICATIONS;
                     strcat((char*)&u16_General_Domain[0],"Unknown faults exist on sensAR: ");
                     strcat((char*)&u16_General_Domain[0],DecToAsciiHex((unsigned long)s32_temp,8));
                     strcat((char*)&u16_General_Domain[0],"\r\n");
                     s32_srvsns_response &= ~s32_temp;
                     state = 17;
                  }

                  // Faults exist which are defined both by internal mask and protocol
                  if (s32_srvsns_response & BGVAR(u32_SrvSns_FaultsIndications))
                  {
                     strcat((char*)&u16_General_Domain[0],"Faults exist on sensAR:\r\n");
                     state = 16;
                  }

                  if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_HELP_TASK_MASK)
                     strcat((char*)&u16_General_Domain[0],"*");

                  ret_val = SAL_NOT_FINISHED;
               }
            }
            else
            {
               strcat((char*)&u16_General_Domain[0],"No faults exist on sensAR");
               if (VAR(AX0_u16_SrvSns_FWStatus) & SRVSNS_FW_STATUS_HELP_TASK_MASK)
                  strcat((char*)&u16_General_Domain[0],"*\r\n");
               else
                  strcat((char*)&u16_General_Domain[0],"\r\n");
               state = 17;
            }
            break;

       case 16:// Print fault messages
            if (PrintMessages(s32_srvsns_response, (long)&u16_SrvSns_Fault_Message, 32, 1 ,0))
            {
               s32_srvsns_response = 0;
               state++;
            }
            break;

      case 17:// Command to get warnings mask
            ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_INT_PAR_WRNS_MASK, (long*)&BGVAR(u32_SrvSns_WarnsIndications), drive, u16_fdbk_source);
            if (ret_val == SAL_SUCCESS)
            {
               state++;
               ret_val = SAL_NOT_FINISHED;
            }
            break;

       case 18:// Command to get warnings: we ingore the status word and request to get warnings word because we might not catch the warning bit
            ret_val = SrvSnsSendCmd(u32_owner, SRVSNS_CMD_GET_WARNINGS, 0, &s32_srvsns_response, drive, u16_fdbk_source);
            if (ret_val == SAL_SUCCESS)
            {
               // Warnings exist which are hidden by the internal mask
               if (s32_srvsns_response & (SRVSNS_WRNS_INDICATIONS & (~BGVAR(u32_SrvSns_WarnsIndications))))
               {
                  long s32_temp = s32_srvsns_response;
                  s32_temp &= (SRVSNS_WRNS_INDICATIONS & (~BGVAR(u32_SrvSns_WarnsIndications)));
                  strcat((char*)&u16_General_Domain[0],"Masked warnings exist on sensAR: ");
                  strcat((char*)&u16_General_Domain[0],DecToAsciiHex((unsigned long)s32_temp,8));
                  strcat((char*)&u16_General_Domain[0],"\r\n");
                  s32_srvsns_response &= ~s32_temp;
                  state = 20;
               }

               // Warnings exist which are not defined
               if ((s32_srvsns_response & (~SRVSNS_WRNS_INDICATIONS)) != 0)
               {
                  long s32_temp = s32_srvsns_response;
                  s32_temp &= ~SRVSNS_WRNS_INDICATIONS;
                  strcat((char*)&u16_General_Domain[0],"Unknown warnings exist on sensAR: ");
                  strcat((char*)&u16_General_Domain[0],DecToAsciiHex((unsigned long)s32_temp,8));
                  strcat((char*)&u16_General_Domain[0],"\r\n");
                  s32_srvsns_response &= ~s32_temp;
                  state = 20;
               }

               // Warnings exist which are defined both by internal mask and protocol
               if (s32_srvsns_response & BGVAR(u32_SrvSns_WarnsIndications))
               {
                  strcat((char*)&u16_General_Domain[0],"Warnings exist on sensAR:\r\n");
                  state = 19;
               }

               if (s32_srvsns_response == 0)
               {
                  strcat((char*)&u16_General_Domain[0],"No warnings exist on sensAR\r\n");
                  state = 20;
               }
               ret_val = SAL_NOT_FINISHED;
            }
            break;

       case 19:// Print warning messages
            if (PrintMessages(s32_srvsns_response, (long)&u16_SrvSns_Warning_Message, 32, 2 ,0))
            {
               s32_srvsns_response = 0;
               state++;
            }
           break;

       case 20:// Run Time
            ret_val = SrvSnsReadAddr(u32_owner, SRVSNS_INT_PAR_RUNTIME, &s32_srvsns_response, drive, u16_fdbk_source);
            if (ret_val == SAL_SUCCESS)
            {
               // parse the received runtime:
               u16_secs    = ((unsigned long)s32_srvsns_response) & SRVSNS_RUNTIME_SECS_MASK;
               u16_mins    = (((unsigned long)s32_srvsns_response) & SRVSNS_RUNTIME_MINS_MASK) >> SRVSNS_RUNTIME_MINS_SHIFT;
               u16_hours   = (((unsigned long)s32_srvsns_response) & SRVSNS_RUNTIME_HOURS_MASK) >> SRVSNS_RUNTIME_HOURS_SHIFT;
               u16_days    = (((unsigned long)s32_srvsns_response) & SRVSNS_RUNTIME_DAYS_MASK) >> SRVSNS_RUNTIME_DAYS_SHIFT;

               // print run-time label
               strcat((char*)&u16_General_Domain[0],"Run Time:   ");

               strcat((char*)&u16_General_Domain[0], UnsignedIntegerToAscii(u16_days));
               strcat((char*)&u16_General_Domain[0],":");

               if (u16_hours < 10)
                  strcat((char*)&u16_General_Domain[0],"0");
               strcat((char*)&u16_General_Domain[0], UnsignedIntegerToAscii(u16_hours));
               strcat((char*)&u16_General_Domain[0],":");

               if (u16_mins < 10)
                  strcat((char*)&u16_General_Domain[0],"0");
               strcat((char*)&u16_General_Domain[0], UnsignedIntegerToAscii(u16_mins));
               strcat((char*)&u16_General_Domain[0],":");

               if (u16_secs < 10)
                  strcat((char*)&u16_General_Domain[0],"0");
               strcat((char*)&u16_General_Domain[0], UnsignedIntegerToAscii(u16_secs));
               strcat((char*)&u16_General_Domain[0],"\r\n");
            }
            else if ((ret_val == SRVSNS_ADDR_OUT_OF_RANGE) || (ret_val == NOT_AVAILABLE))
            {
               strcat((char*)&u16_General_Domain[0],"Run Time:   ");
               strcat((char*)&u16_General_Domain[0], "Not Available\r\n");
               ret_val = SAL_NOT_FINISHED;
               state++;
            }
            if (ret_val != SAL_NOT_FINISHED)
            {
               ret_val = SAL_NOT_FINISHED;
               state++;
            }
       break;

       case 21:// Temperature Histogram Label
          // Print label of the next state
          strcat((char*)&u16_General_Domain[0],"Temperature Histogram:\r\n");
          u16_product_info_addr = SRVSNS_TEMP_HIST_BASE;
          ret_val = SAL_NOT_FINISHED;
          state++;
       break;

       case 22:// Get Temperature Histogram
          ret_val = SrvSnsReadAddr(u32_owner, u16_product_info_addr, (long*)&s32_srvsns_response, drive, u16_fdbk_source);
          if (ret_val == SAL_SUCCESS)
          {
             strcat((char*)&u16_General_Domain[0], UnsignedIntegerToAscii((unsigned long)s32_srvsns_response));// bar value
             u16_product_info_addr += SRVSNS_ADDRESS_ALIGNMENT;
             if (u16_product_info_addr == (SRVSNS_TEMP_HIST_BASE + SRVSNS_TEMP_HIST_NUMOFBARS * SRVSNS_ADDRESS_ALIGNMENT))// if we got to histogram end
             {
                strcat((char*)&u16_General_Domain[0], "\r\n");
                state++;
             }
             else
             {
                strcat((char*)&u16_General_Domain[0], "     ");
             }
             ret_val = SAL_NOT_FINISHED;
          }
          else if ((ret_val == SRVSNS_ADDR_OUT_OF_RANGE) || (ret_val == NOT_AVAILABLE))
          {
             strcat((char*)&u16_General_Domain[0], "Not Available\r\n");
             ret_val = SAL_NOT_FINISHED;
             state++;
          }
       break;

       case 23:// Velocity Histogram Label
          // Print label of the next state
          strcat((char*)&u16_General_Domain[0],"Velocity Histogram:\r\n");
          u16_product_info_addr = SRVSNS_VEL_HIST_BASE;
          ret_val = SAL_NOT_FINISHED;
          state++;
       break;

       case 24:// Get Velocity Histogram
          ret_val = SrvSnsReadAddr(u32_owner, u16_product_info_addr, (long*)&s32_srvsns_response, drive, u16_fdbk_source);
          if (ret_val == SAL_SUCCESS)
          {
             strcat((char*)&u16_General_Domain[0], UnsignedIntegerToAscii((unsigned long)s32_srvsns_response));// bar value
             u16_product_info_addr += SRVSNS_ADDRESS_ALIGNMENT;
             if (u16_product_info_addr == (SRVSNS_VEL_HIST_BASE + SRVSNS_VEL_HIST_NUMOFBARS * SRVSNS_ADDRESS_ALIGNMENT))// if we got to histogram end
             {
                strcat((char*)&u16_General_Domain[0], "\r\n");
                state++;
             }
             else
             {
                strcat((char*)&u16_General_Domain[0], "     ");
             }
             ret_val = SAL_NOT_FINISHED;
          }
          else if ((ret_val == SRVSNS_ADDR_OUT_OF_RANGE) || (ret_val == NOT_AVAILABLE))
          {
             strcat((char*)&u16_General_Domain[0], "Not Available\r\n");
             ret_val = SAL_NOT_FINISHED;
             state++;
          }
       break;

       case 25:// finalize
          ret_val = SAL_SUCCESS;
       break;
   }

   if (ret_val != SAL_NOT_FINISHED)
   {
      /* Release ServoSense device */
      SrvSns_Release(&u32_owner, drive);
      u32_owner = 0;
      s32_acquisition_timer = 0;
      s32_srvsns_response = 0;
      u16_product_info_addr = SRVSNS_PRODUCT_INFO_AREA_BASE;
      state = 0;
   }

   // At the last iteration,printing will be handled here.
   if (ret_val == SAL_SUCCESS)
   {
      if (s16_fb_use == FB_NOT_IN_USE)
      {
         OutputBufferLoader();
      }
   }
   return ret_val;
}


//**********************************************************
// Function Name: SalSrvSnsReadAddr
// Description:
//   This function called in response to SRVSNSREAD command
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int SalSrvSnsReadAddr(int drive)
{
   static int state = 0;
   static unsigned long u32_owner = 0;
   static long s32_acquisition_timer = 0;
   unsigned int ret_val = SAL_NOT_FINISHED;
   long s32_value = 0L;
   // AXIS_OFF;

   /* Do not continue if ServoSense is not in use or not ready */
   if (     (!FEEDBACK_SERVOSENSE)                          ||
            (BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK)   )
   {
      state = 0;
      return NOT_AVAILABLE;
   }

   if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;

   if (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK)
   {
      PrintOnFdbkCommFaultMessage(drive);
      /* Release ServoSense device */
      SrvSns_Release(&u32_owner, drive);
      s32_acquisition_timer = 0L;
      state = 0;
      return SAL_SUCCESS;
   }

   switch(state)
   {
      case 0:
         if ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) == 0)
         {
            ret_val = NOT_AVAILABLE;
            break;
         }
         if (s64_Execution_Parameter[0] > 0x000000000000FFFFL)
         {
            ret_val = SRVSNS_ADDR_OUT_OF_RANGE;
            break;
         }
         s32_acquisition_timer = Cntr_1mS;
         state++;
      break;

      case 1: // Acquire ServoSense
         // Acquire ServoSense device
         ret_val = SrvSns_Acquire(&u32_owner, drive);
         if (ret_val == SAL_SUCCESS)
         {
            ret_val = SAL_NOT_FINISHED;
            state++;
            break;
         }
         if (PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC, s32_acquisition_timer))
            ret_val = SRVSNS_DRV_ACQ_TIMEOUT;
      break;

      case 2:
         ret_val = SrvSnsReadAddr(u32_owner, (unsigned int)s64_Execution_Parameter[0], &s32_value, drive, 0);
	 s32_Servo_Sense_Read_Value = s32_value;
         if (ret_val == SAL_SUCCESS)
         {
            // Identify the address range to provide proper response
            if ( ((unsigned int)s64_Execution_Parameter[0] ==  SRVSNS_APDXA_SUBMODEADDR)                           || // Sub Mode Address
                 ((unsigned int)s64_Execution_Parameter[0] == SRVSNS_INT_PAR_FLTS_MASK)                            || // Faults mask
                 ((unsigned int)s64_Execution_Parameter[0] == SRVSNS_INT_PAR_WRNS_MASK)                            || // Warns mask
                 ((unsigned int)s64_Execution_Parameter[0] <= (SRVSNS_EEPROM_END_ADDR + SRVSNS_ADDRESS_ALIGNMENT)) || // EEPROM address print HEX
                 ( ((unsigned int)s64_Execution_Parameter[0] >= SRVSNS_FLT_HSTRY_START) &&
                   ((unsigned int)s64_Execution_Parameter[0] <= SRVSNS_FLT_HSTRY_END)     )                        || // Faults history records
                 ( ((unsigned int)s64_Execution_Parameter[0] >= SRVSNS_PRODUCT_INFO_AREA_BASE) &&
                   ((unsigned int)s64_Execution_Parameter[0] < SRVSNS_PRODUCT_INFO_AREA_END)     )                   ) // Production Info address print HEX
            {
               PrintDecInAsciiHex((unsigned long)s32_value, 8);
            }
            else // any other print decimal
            {
               if ( ((unsigned int)s64_Execution_Parameter[0] == SRVSNS_INT_PAR_TEMPERATURE)                           ||
                    ((unsigned int)s64_Execution_Parameter[0] == SRVSNS_INT_PAR_TEMP_LOW_THR)                          ||
                    ( (((unsigned int)s64_Execution_Parameter[0] & SRVSNS_FLT_APDXA_MASK) == SRVSNS_FLT_APDXA_MASK) &&
                      ((unsigned int)s64_Execution_Parameter[0] != SRVSNS_FLT_APDXA_FAULTS)                         &&
                      ((unsigned int)s64_Execution_Parameter[0] != SRVSNS_FLT_APDXA_WARNINGS)                         )  )
               {
                  PrintSignedInt32(s32_value);
               }
               else
                 PrintUnsignedInt32((unsigned long)s32_value);
            }
            PrintCrLf();
         }
      break;
   }

   if (ret_val != SAL_NOT_FINISHED)
   {
       /* Release ServoSense device */
       SrvSns_Release(&u32_owner, drive);
       s32_acquisition_timer = 0;
       state = 0;
   }
   return ret_val;
}


//**********************************************************
// Function Name: SalSrvSnsWriteAddr
// Description:
//   This function called in response to SRVSNSWRITE command
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
//***********************************************************
int SalSrvSnsWriteAddr(int drive)
{
   static int state = 0;
   static unsigned long u32_owner = 0;
   static long s32_acquisition_timer = 0;
   int ret_val = SAL_NOT_FINISHED;
   // AXIS_OFF;

   /* Do not continue if ServoSense is not in use or not ready */
   if (     (!FEEDBACK_SERVOSENSE)                          ||
            (BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK)   )
   {
      state = 0;
      return NOT_AVAILABLE;
   }

   if (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK)
   {
      if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;

      PrintOnFdbkCommFaultMessage(drive);

      /* Release ServoSense device */
      SrvSns_Release(&u32_owner, drive);
      s32_acquisition_timer = 0L;
      state = 0;
      return SAL_SUCCESS;
   }

   switch(state)
   {
      case 0:
         if ((VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) == 0)
         {
            ret_val = NOT_AVAILABLE;
            break;
         }
         if (s64_Execution_Parameter[0] > 0x000000000000FFFFL)
         {
            ret_val = SRVSNS_ADDR_OUT_OF_RANGE;
            break;
         }
         s32_acquisition_timer = Cntr_1mS;
         state++;
      break;

      case 1: // Acquire ServoSense
         // Acquire ServoSense device
         ret_val = SrvSns_Acquire(&u32_owner, drive);
         if (ret_val == SAL_SUCCESS)
         {
            ret_val = SAL_NOT_FINISHED;
            state++;
            break;
         }
         if (PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC, s32_acquisition_timer))
            ret_val = SRVSNS_DRV_ACQ_TIMEOUT;
      break;

      case 2:
         ret_val = SrvSnsWriteAddr(u32_owner, (unsigned int)s64_Execution_Parameter[0], (unsigned long)s64_Execution_Parameter[1], drive, 0);
      break;
   }

   if (ret_val != SAL_NOT_FINISHED)
   {
      /* Release ServoSense device */
      SrvSns_Release(&u32_owner, drive);
      s32_acquisition_timer = 0;
      state = 0;
   }

   return ret_val;
}


//**********************************************************
// Function Name: SalSrvSnsSendCmd
// Description:
//   This function called in response to SRVSNSSENDCMD command
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//************************************************************
int SalSrvSnsSendCmd(int drive)
{
   static int state = 0;
   static unsigned long u32_owner = 0, u32_arg = 0UL;
   static long s32_acquisition_timer = 0L;
   static unsigned int u16_cmd = 0;
   unsigned int u16_tmp_cmd = 0;
   long s32_value = 0L;
   int ret_val = SAL_NOT_FINISHED;
   unsigned int u16_secs, u16_mins, u16_hours, u16_days;
   // AXIS_OFF;

   /* Do not continue if ServoSense is not in use or not ready */
   if (     (!FEEDBACK_SERVOSENSE)                          ||
            (BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK)   )
   {
      state = 0;
      return NOT_AVAILABLE;
   }

   if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;

   if ( ((BGVAR(u16_SrvSns_Cmd) != SRVSNS_CMD_RESET) && (BGVAR(u16_SrvSns_Cmd) != SRVSNS_CMD_FW_DNLD)) &&
        (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK)                                                       )
   {
      PrintOnFdbkCommFaultMessage(drive);

      /* Release ServoSense device */
      SrvSns_Release(&u32_owner, drive);
      s32_acquisition_timer = 0L;
      u16_cmd = 0;
      u32_arg = 0UL;
      state = 0;
      return SAL_SUCCESS;
   }

   switch(state)
   {
      case 0:
         /* Validation stage */
         u16_cmd = (unsigned int)s64_Execution_Parameter[0];
         if (s16_Number_Of_Parameters > 1)
            u32_arg = (unsigned long)s64_Execution_Parameter[1];
         else
            u32_arg = 0;

         if (  ((u16_cmd != SRVSNS_CMD_GET_SET_COMM) && (u16_cmd != SRVSNS_CMD_PWR_CNTRL))   &&
               (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_READY_MASK) == 0  )
         {
            ret_val = NOT_AVAILABLE;
            break;
         }

         /* Validate command specific options */
         switch (u16_cmd)
         {
            case SRVSNS_CMD_GET_SET_COMM:
            case SRVSNS_CMD_CALIB_SYNC:
            case SRVSNS_CMD_HALLS_SYNC_REC_DONE:
            case SRVSNS_CMD_FACTORY_RESTORE:
            case SRVSNS_CMD_SETSUBMODEFUNC:
            case SRVSNS_CMD_PWR_CNTRL:
               if (!s16_Password_Flag)
               {
                  ret_val = PASSWORD_PROTECTED;
               }
               else
               {
                  if (s16_Number_Of_Parameters > 1)
                     u16_cmd |= SRVSNS_CMD_WRITE_MASK;
               }
               if((u16_cmd == SRVSNS_CMD_PWR_CNTRL) || (u16_cmd == SRVSNS_CMD_GET_SET_COMM))
               {
                  if(Enabled(drive))
                     return (DRIVE_ACTIVE);
               }
            break;

            case SRVSNS_CMD_EE_ACCESS_ST:
            case SRVSNS_CMD_EE_WRITE_ENABLE:
            case SRVSNS_CMD_EE_WRITE_DISABLE:
            case SRVSNS_CMD_SETZEROPOS:
            case SRVSNS_CMD_GET_TEMP_HIST_BAR:
            case SRVSNS_CMD_GET_VEL_HIST_BAR:
               if (s16_Number_Of_Parameters < 2)
                  ret_val = SYNTAX_ERROR;
            break;

            case SRVSNS_CMD_MT_CALIB:
               if (s16_Number_Of_Parameters > 1)
                  u16_cmd |= SRVSNS_CMD_WRITE_MASK;
               if(Enabled(drive))
                  return (DRIVE_ACTIVE);
            break;

            case SRVSNS_CMD_MT_CONFIG:
            case SRVSNS_CMD_RESET_MT_CNTR:
               if(Enabled(drive))
                  return (DRIVE_ACTIVE);
            break;

            default:
            break;
         }

         if (ret_val == SAL_NOT_FINISHED)
         {
            s32_acquisition_timer = Cntr_1mS;
            state++;
         }
      break;

      case 1: // Acquire ServoSense
         // Acquire ServoSense device
         ret_val = SrvSns_Acquire(&u32_owner, drive);
         if (ret_val == SAL_SUCCESS)
         {
            ret_val = SAL_NOT_FINISHED;
            state++;
            break;
         }
         if (PassedTimeMS(SRVSNS_ACQUISITION_TIMEOUT_mSEC, s32_acquisition_timer))
            ret_val = SRVSNS_DRV_ACQ_TIMEOUT;
      break;

      case 2:
         /* Execution stage */
         if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;

         ret_val = SrvSnsSendCmd(u32_owner, u16_cmd, u32_arg, &s32_value, drive, 0);
         if (ret_val == SAL_SUCCESS)
         {
            u16_tmp_cmd = u16_cmd & (~SRVSNS_CMD_WRITE_MASK);
            switch(u16_tmp_cmd)
            {
               case SRVSNS_CMD_EE_ACCESS_ST:
               case SRVSNS_CMD_GET_SET_COMM:
               case SRVSNS_CMD_CALIB_SYNC:
               case SRVSNS_CMD_SETSUBMODEFUNC:
               case SRVSNS_CMD_GET_TEMP_HIST_BAR:
               case SRVSNS_CMD_GET_VEL_HIST_BAR:
               case SRVSNS_CMD_PWR_CNTRL:
               case SRVSNS_CMD_MT_CALIB:
                  PrintUnsignedInt32((unsigned long)s32_value);
                  PrintCrLf();
               break;

               case SRVSNS_CMD_GET_DATA:
                  PrintDecInAsciiHex((unsigned long)s32_value, 8);
                  PrintCrLf();
               break;

               case SRVSNS_CMD_HALLS_SYNC_REC_DONE:
               case SRVSNS_CMD_ZEROPOSST:
                  PrintSignedInt32(s32_value);
                  PrintCrLf();
               break;

               case SRVSNS_CMD_RESET:
               case SRVSNS_CMD_FW_DNLD:
                  PrintStringCrLf("Reseting sensAR encoder...", 0);
               break;

               case SRVSNS_CMD_GET_FAULTS:
                  if (s32_value)
                  {
                     PrintString("Faults exist on sensAR: ", 0);
                     PrintDecInAsciiHex((unsigned long)s32_value, 8);
                     PrintCrLf();
                  }
                  else
                  {
                     PrintString("No faults exist on sensAR. ", 0);
                     PrintCrLf();
                  }
               break;

               case SRVSNS_CMD_GET_WARNINGS:
                  if (s32_value)
                  {
                     PrintString("Warnings exist on sensAR: ", 0);
                     PrintDecInAsciiHex((unsigned long)s32_value, 8);
                     PrintCrLf();
                  }
                  else
                  {
                     PrintString("No warnings exist on sensAR. ", 0);
                     PrintCrLf();
                  }
               break;

               case SRVSNS_CMD_GETRUNTIME:
                  // parse the received runtime:
                  u16_secs  = ((unsigned long)s32_value) & SRVSNS_RUNTIME_SECS_MASK;
                  u16_mins  = (((unsigned long)s32_value) & SRVSNS_RUNTIME_MINS_MASK) >> SRVSNS_RUNTIME_MINS_SHIFT;
                  u16_hours = (((unsigned long)s32_value) & SRVSNS_RUNTIME_HOURS_MASK) >> SRVSNS_RUNTIME_HOURS_SHIFT;
                  u16_days  = (((unsigned long)s32_value) & SRVSNS_RUNTIME_DAYS_MASK) >> SRVSNS_RUNTIME_DAYS_SHIFT;

                  PrintString(UnsignedIntegerToAscii(u16_days), 0);
                  PrintChar(':');

                  if (u16_hours < 10)
                     PrintChar('0');
                  PrintString(UnsignedIntegerToAscii(u16_hours), 0);
                  PrintChar(':');

                  if (u16_mins < 10)
                     PrintChar('0');
                  PrintString(UnsignedIntegerToAscii(u16_mins), 0);
                  PrintChar(':');

                  if (u16_secs < 10)
                     PrintChar('0');
                  PrintString(UnsignedIntegerToAscii(u16_secs), 0);
                  PrintChar('\n');
               break;

               default:
               break;
            }
         }
         else if ((ret_val == SRVSNS_ADDR_OUT_OF_RANGE) || (ret_val == NOT_AVAILABLE))
         {
            PrintString("Not Available\n", 0);
            ret_val = SAL_SUCCESS;
         }
      break;
   }

   if (ret_val != SAL_NOT_FINISHED)
   {
      /* Release ServoSense device */
      SrvSns_Release(&u32_owner, drive);
      s32_acquisition_timer = 0L;
      u16_cmd = 0;
      u32_arg = 0UL;
      state = 0;
   }

   return ret_val;
}

void CopyDualFeedbackDesignVariables (int drive)
{
   //AXIS_OFF;
   REFERENCE_TO_DRIVE;
   VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr) = VAR(AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr_Design);
   VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr)                 = VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr_Design);
   VAR(AX0_s16_Vel_Loop_Cmnd_Ptr)                  = VAR(AX0_s16_Vel_Loop_Cmnd_Ptr_Design);
   LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr)                  = LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr_Design);
}

void EncSimDualLoopInit (int u16_old_DF_mode)
{
   int u16_encout_mode = BGVAR(u8_EncoutMode);
   
   if (IS_DUAL_LOOP_ACTIVE == u16_old_DF_mode)  return;
   
   else
   {
       //1st: disable the encoder simulation output to avoid wrong delta position calculations resulting false simulated pulses on RT
       if (0 != u16_encout_mode)
         SalEncoderSimMode(0LL, 0); 
       
       //2nd: set the encoder simulation position pointer properly according the new SFBMODE
       if (IS_DUAL_LOOP_ACTIVE)
         VAR(AX0_u16_DualLoop_EncSim_Pos_Lo_Ptr)  = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_Lo_2 & 0xFFFF);//set Load's encoder pulses as simulation input
       else
         VAR(AX0_u16_DualLoop_EncSim_Pos_Lo_Ptr)  = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_Lo & 0xFFFF);//set Motor's pulses as simulation input 
       
       //3rd: let the 'prev delta' RT variable be updated according to the new position pointer, by waiting one MTS
       
       //4th: re-enable the encoder simulation output
       if (0 != u16_encout_mode)
         SalEncoderSimMode((long long)u16_encout_mode, 0);        
   }
}

void PcomDualLoopInit (int drive)
{
   REFERENCE_TO_DRIVE;
   if (!IS_DUAL_LOOP_ACTIVE)
   {
      VAR(AX0_s16_Encres_Fix_Ptr) = (int)((long)&AX0_s32_Mencres_Fix & 0xFFFF);
      VAR(AX0_u16_Encres_Shr_Ptr) = (int)((long)&AX0_u16_Mencres_Shr & 0xFFFF); 
   }
   else
   {
      VAR(AX0_s16_Encres_Fix_Ptr) = (int)((long)&AX0_s32_Sfb_Encres_Fix & 0xFFFF);
      VAR(AX0_u16_Encres_Shr_Ptr) = (int)((long)&AX0_u16_Sfb_Encres_Shr & 0xFFFF);
   }
}

/* ************ DUAL LOOP *********** */
void DualLoopInit(int drive)
{
   int u16_old_DF_mode = IS_DUAL_LOOP_ACTIVE;
   
   VAR(AX0_u16_Dual_Loop_Control_Bits) &= ~(DUAL_LOOP_ENABLED_MASK | DUAL_LOOP_DIS_SFB_MONITOR_ONLY_MASK);//zero variable before setting
   switch (BGVAR(s16_SFBMode))
   {
      case 0:
         BGVAR(u16_Fdbk_Source) = 0; // Set SFBTYPE to No-Feedback to allow successful CONFIG on Power-Up
         //BGVAR(u16_SFBType_User) = BGVAR(u16_SFBType) = NO_SEC_FDBK; // Set SFBTYPE to No-Feedback to allow successful CONFIG on Power-Up
         VAR(AX0_u16_Dual_Loop_PFB_Ptr) = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_User_Lo & 0xFFFF);
         // Direct PFB Pointer to Primary Feedback Variable
         BGVAR(u16_Dual_Loop_Mode) = 0; // Set Position-Loop Configuration
         // Primary Feedback sources:
         VAR(AX0_u16_FPGA_TxRx_Regs_PFB) = 0x04480; // 1st Channel (C4 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_PFB) = 0x04100; // 1st Channel (C4 Connector) Base Registers
         // Secondary Feedback Source:
         VAR(AX0_u16_FPGA_TxRx_Regs_SFB) = 0x04500; // 2nd Channel (C3 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_SFB) = 0x04640; // 2nd Channel (C4 Connector) Base Registers
         VAR(AX0_u16_TouchProbe_DualLoop_Pfb_Ptr) = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_User_Lo & 0xFFFF);//set TP capture MFB
         VAR(AX0_s16_TouchProbe_DualLoop_Vel_Ptr) = (int)((long)&AX0_s32_Vel_Var_Fb_0 & 0xFFFF);//set TP capture V 
         BGVAR(s64_Faults_Mask) &= ~SEC_LINE_BRK_FLT_MASK;
      break;

      case 1:
         BGVAR(u16_Fdbk_Source) = 0;
         VAR(AX0_u16_Dual_Loop_Control_Bits) |= DUAL_LOOP_ENABLED_MASK; // Set Dual-Loop Enable bit.
         VAR(AX0_u16_Dual_Loop_PFB_Ptr) = (unsigned int)((unsigned long)&VAR(AX0_u32_SFB_Pos_Fdbk_Lo) & 0xFFFF);
         // Direct PFB Pointer to Secondary Feedback Variable
         // Direct PFB Pointer to Secondary Feedback Variable
         BGVAR(u16_Dual_Loop_Mode) = 1; // Set Position-Loop Configuration
         // LDP      AX0_POS_4_VAR_PAGE
         // MOV      @_AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr,#_AX0_s32_Dual_Loop_Vcmd
         // Primary Feedback sources:
         VAR(AX0_u16_FPGA_TxRx_Regs_PFB) = 0x04480; // 1st Channel (C4 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_PFB) = 0x04100; // 1st Channel (C4 Connector) Base Registers
         // Secondary Feedback Source:
         VAR(AX0_u16_FPGA_TxRx_Regs_SFB) = 0x04500; // 2nd Channel (C3 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_SFB) = 0x04640; // 2nd Channel (C4 Connector) Base Registers
         VAR(AX0_u16_TouchProbe_DualLoop_Pfb_Ptr) = (unsigned int)((unsigned long)&VAR(AX0_u32_SFB_Pos_Fdbk_Lo) & 0xFFFF);//set TP capture SFB
         VAR(AX0_s16_TouchProbe_DualLoop_Vel_Ptr) = (int)((long)&AX0_s32_Sfb_Vel_Var_Fb_0 & 0xFFFF);//set TP capture SFBVEL
         BGVAR(s64_Faults_Mask) |= SEC_LINE_BRK_FLT_MASK;
      break;

      case 2:
         BGVAR(u16_Fdbk_Source) = 0;
         VAR(AX0_u16_Dual_Loop_Control_Bits) |= DUAL_LOOP_DIS_SFB_MONITOR_ONLY_MASK; // Set SFB monitor-only bit.
         VAR(AX0_u16_Dual_Loop_PFB_Ptr) = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_User_Lo & 0xFFFF);
         // Direct PFB Pointer to Primary Feedback Variable
         BGVAR(u16_Dual_Loop_Mode) = 0; // Set Position-Loop Configuration
         // Primary Feedback sources:
         VAR(AX0_u16_FPGA_TxRx_Regs_PFB) = 0x04480; // 1st Channel (C4 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_PFB) = 0x04100; // 1st Channel (C4 Connector) Base Registers
         // Secondary Feedback Source:
         VAR(AX0_u16_FPGA_TxRx_Regs_SFB) = 0x04500; // 2nd Channel (C3 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_SFB) = 0x04640; // 2nd Channel (C4 Connector) Base Registers
         VAR(AX0_u16_TouchProbe_DualLoop_Pfb_Ptr) = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_User_Lo & 0xFFFF);//set TP capture MFB
         VAR(AX0_s16_TouchProbe_DualLoop_Vel_Ptr) = (int)((long)&AX0_s32_Vel_Var_Fb_0 & 0xFFFF);//set TP capture V
         BGVAR(s64_Faults_Mask) |= SEC_LINE_BRK_FLT_MASK;
      break;

      case 4:
         BGVAR(u16_Fdbk_Source) = 1;
         //BGVAR(u16_SFBType_User) = BGVAR(u16_SFBType) = NO_SEC_FDBK; // Set SFBTYPE to No-Feedback to allow successful CONFIG on Power-Up
         VAR(AX0_u16_Dual_Loop_PFB_Ptr) = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_User_Lo & 0xFFFF);
         // Direct PFB Pointer to Primary Feedback Variable
         BGVAR(u16_Dual_Loop_Mode) = 0; // Set Position-Loop Configuration
         // Primary Feedback sources:
         VAR(AX0_u16_FPGA_TxRx_Regs_PFB) = 0x04500; // 2nd Channel (C3 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_PFB) = 0x04640; // 2nd Channel (C4 Connector) Base Registers
         // Secondary Feedback Source:
         VAR(AX0_u16_FPGA_TxRx_Regs_SFB) = 0x04480; // 1st Channel (C4 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_SFB) = 0x04100; // 1st Channel (C4 Connector) Base Registers
         VAR(AX0_u16_TouchProbe_DualLoop_Pfb_Ptr) = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_User_Lo & 0xFFFF);//set TP capture MFB
         VAR(AX0_s16_TouchProbe_DualLoop_Vel_Ptr) = (int)((long)&AX0_s32_Vel_Var_Fb_0 & 0xFFFF);//set TP capture V
         BGVAR(s64_Faults_Mask) |= SEC_LINE_BRK_FLT_MASK; //currently locating Sec_Line_Break detection for C3 Main feedback
      break;

      case 5:
         BGVAR(u16_Fdbk_Source) = 1;
         VAR(AX0_u16_Dual_Loop_Control_Bits) |= DUAL_LOOP_ENABLED_MASK; // Set Dual-Loop Enable bit.
         VAR(AX0_u16_Dual_Loop_PFB_Ptr) = (unsigned int)((unsigned long)&AX0_u32_SFB_Pos_Fdbk_Lo & 0xFFFF);
         // Direct PFB Pointer to Secondary Feedback Variable
         BGVAR(u16_Dual_Loop_Mode) = 1; // Set Position-Loop Configuration
         // LDP      AX0_POS_4_VAR_PAGE
         // MOV      @_AX0_s16_Pos_Loop_Main_Controller_Vcmnd_Ptr,#_AX0_s32_Dual_Loop_Vcmd
         // Primary Feedback sources:
         VAR(AX0_u16_FPGA_TxRx_Regs_PFB) = 0x04500; // 2nd Channel (C3 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_PFB) = 0x04640; // 2nd Channel (C4 Connector) Base Registers
         // Secondary Feedback Source:
         VAR(AX0_u16_FPGA_TxRx_Regs_SFB) = 0x04480; // 1st Channel (C4 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_SFB) = 0x04100; // 1st Channel (C4 Connector) Base Registers
         VAR(AX0_u16_TouchProbe_DualLoop_Pfb_Ptr) = (unsigned int)((unsigned long)&VAR(AX0_u32_SFB_Pos_Fdbk_Lo) & 0xFFFF);//set TP capture SFB
         VAR(AX0_s16_TouchProbe_DualLoop_Vel_Ptr) = (int)((long)&AX0_s32_Sfb_Vel_Var_Fb_0 & 0xFFFF);//set TP capture SFBVEL
         BGVAR(s64_Faults_Mask) |= SEC_LINE_BRK_FLT_MASK; //currently locating Sec_Line_Break detection for C3 Main feedback
      break;

      case 6:
         BGVAR(u16_Fdbk_Source) = 1;
         VAR(AX0_u16_Dual_Loop_Control_Bits) |= DUAL_LOOP_DIS_SFB_MONITOR_ONLY_MASK; // Set SFB monitor-only bit.
         VAR(AX0_u16_Dual_Loop_PFB_Ptr) = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_User_Lo & 0xFFFF);
         // Direct PFB Pointer to Primary Feedback Variable
         BGVAR(u16_Dual_Loop_Mode) = 0; // Set Position-Loop Configuration
         // Primary Feedback sources:
         VAR(AX0_u16_FPGA_TxRx_Regs_PFB) = 0x04500; // 2nd Channel (C3 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_PFB) = 0x04640; // 2nd Channel (C4 Connector) Base Registers
         // Secondary Feedback Source:
         VAR(AX0_u16_FPGA_TxRx_Regs_SFB) = 0x04480; // 1st Channel (C4 Connector) Transmit/Receive Block-RAM Registers
         VAR(AX0_u16_FPGA_Base_Regs_SFB) = 0x04100; // 1st Channel (C4 Connector) Base Registers
         VAR(AX0_u16_TouchProbe_DualLoop_Pfb_Ptr) = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_User_Lo & 0xFFFF);//set TP capture MFB
         VAR(AX0_s16_TouchProbe_DualLoop_Vel_Ptr) = (int)((long)&AX0_s32_Vel_Var_Fb_0 & 0xFFFF);//set TP capture V
         BGVAR(s64_Faults_Mask) |= SEC_LINE_BRK_FLT_MASK; //currently locating Sec_Line_Break detection for C3 Main feedback
      break;
   }
   
   UpdateAccDec(drive,ACC_UPDATE);
   UpdateAccDec(drive,DEC_UPDATE);
   IsPositionLoopNeeded(drive);
   IsLinearVelocityNeeded(drive);
   SetPosLoopMainControllerVcmdPtr(drive,VAR(AX0_s16_Opmode));
   SetVelocityLoopPtr(drive,VAR(AX0_s16_Opmode));
   SetCurrLoopPtr(drive,VAR(AX0_s16_Opmode),0);
   CopyDualFeedbackDesignVariables(drive);
   EncSimDualLoopInit(u16_old_DF_mode);
   PcomDualLoopInit(drive);
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
}



//**********************************************************
// Function Name: SalSFBModeCommand
// Description:
//          This function is called in response to the SFBMODE command.
//
//
// Author: Lior
// Algorithm:
// Revisions:
//**********************************************************
int SalSFBModeCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   unsigned int u16_was_dl_active = 0;

   if ((int)lparam == 3)
      return VALUE_OUT_OF_RANGE; // SFBMODE 3 not defined.
   if ((0 != (int)lparam) && ((VAR(AX0_u8_Gear_Mode) == 3) || (VAR(AX0_u8_Gear_Mode) == 4)))
      return (INVALID_DF_GEARMODE); //Dual Loop is only allowed with GearModes 0/1
      
   u16_was_dl_active = IS_DUAL_LOOP_ACTIVE;//save old dual loop state.
   
   BGVAR(s16_SFBMode) = (int)lparam;
   DualLoopInit(drive);
   SFBTypeCommand(BGVAR(u16_SFBType_User), drive);   
   SfbUnitsStringsUpdate();
   
   if (IS_DUAL_LOOP_ACTIVE != u16_was_dl_active) 
   {  // in case the dual loop state has changed:   
      if (u16_was_dl_active)
      /**** changed from active DF to non active DF  ****/
      {  
         // first store Home indication and Calculated Home offset of SFB
         BGVAR(u16_Prev_Sfb_home_Ind) = VAR(AX0_u16_Home_Ind);
         BGVAR(s64_Prev_Sfb_Home_Offset) = LLVAR(AX0_u32_Home_Offset_Lo); 
         // then restore Home indication and Calculated Home offset of MFB
         VAR(AX0_u16_Home_Ind) = BGVAR(u16_Prev_Mfb_home_Ind);
         LLVAR(AX0_u32_Home_Offset_Lo) = BGVAR(s64_Prev_Mfb_Home_Offset); 
      }
      else
      /**** changed from non active DF to active DF  ****/
      {
         // first store Home indication and Calculated Home offset of MFB
         BGVAR(u16_Prev_Mfb_home_Ind) = VAR(AX0_u16_Home_Ind);
         BGVAR(s64_Prev_Mfb_Home_Offset) = LLVAR(AX0_u32_Home_Offset_Lo); 
         // then restore Home indication and Calculated Home offset of SFB
         VAR(AX0_u16_Home_Ind) = BGVAR(u16_Prev_Sfb_home_Ind);
         LLVAR(AX0_u32_Home_Offset_Lo) = BGVAR(s64_Prev_Sfb_Home_Offset);
      }         
   }   
//   BGVAR(u8_Sfb_Pfb_Realignment_Counter) = 2; // Do realignment in 2[ms]. This ensures that SFB is definitely updated in the 31.25[us] task.

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalSFBTypeCommand
// Description:
//          This function is called in response to the SFBTYPE command.
//
//
// Author: Lionel
// Algorithm:
// Revisions:
//**********************************************************
/* int SalSFBTypeCommand(long long lparam, int drive)
{
   BGVAR(u16_SFBType) = (int)lparam;
   DualLoopInit(drive);
   return SAL_SUCCESS;
} */




//**********************************************************
// Function Name: SalSFBUnitsNumerator, SalSFBUnitsDenominator
// Description:
//    These functions set the ratio from secondary feedback position to user unit
//
// Author: Lionel
// Algorithm:
// Revisions:
//**********************************************************
int SalSFBUnitsDenominator(long long param, int drive)
{
   BGVAR(s32_SFBUnitsDen) = (unsigned long)param;
   ConversionSecondaryEncoder(drive);
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault
   return (SAL_SUCCESS);
}


int SalSFBUnitsNumerator(long long param, int drive)
{
   if (param == 0LL) return VALUE_OUT_OF_RANGE;
   BGVAR(s32_SFBUnitsNum) = (unsigned long)param;
   ConversionSecondaryEncoder(drive);
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalSFB2MotorNumerator, SalSFB2MotorDenominator
// Description:
//    These functions set the ratio from secondary feedback position to primary encoder
//
// Author: Lionel
// Algorithm:
// Revisions:
//**********************************************************

int SalSFB2MotorDenominator(long long param, int drive)
{
   BGVAR(s32_SFB2MotorDen) = (unsigned long)param;
   ConversionSecondaryEncoder(drive);
   return (SAL_SUCCESS);
}


int SalSFB2MotorNumerator(long long param, int drive)
{
   if (param == 0LL) return VALUE_OUT_OF_RANGE;
   BGVAR(s32_SFB2MotorNum) = (unsigned long)param;
   ConversionSecondaryEncoder(drive);
   return (SAL_SUCCESS);
}


int SalHSaveToFlash (int drive)
{
   // AXIS_OFF;
   //Hiperface Feedback
   if ((10 == VAR(AX0_s16_Motor_Enc_Type)) && (3 ==u16_FdbkType))
   {
      return SalHSaveToHifaceFlash(drive);
   }
   //Endat Feedback
   else if (((9 == VAR(AX0_s16_Motor_Enc_Type)) && (3 == u16_FdbkType)) || ((0 == VAR(AX0_s16_Motor_Enc_Type)) && (11 == u16_FdbkType)))
   {
      return SalHSaveToEndatFlash(drive);
   }
   else
      return NOT_SUPPORTED_ON_FEEDBACK;
}


int SalTimeStampLocations(int drive)
{
    unsigned int u16_temp_time;
    // AXIS_OFF;
    REFERENCE_TO_DRIVE;
    if (3 == s16_Number_Of_Parameters)
    {
      if (s64_Execution_Parameter[0] >= s64_Execution_Parameter[1])
         return VALUE_IS_NOT_ALLOWED;
       if ((s64_Execution_Parameter[2] < 10) && (s64_Execution_Parameter[2] > -1))
       {
          VAR(AX0_u16_MtsIsr_TimeStamp_Iteration) = s64_Execution_Parameter[2];
          if ((0 != s64_Execution_Parameter[0] || 9 != s64_Execution_Parameter[1]) && (8 > s64_Execution_Parameter[2]))
          {
             PrintStringCrLf("3rd arg should be 8 if chosen locations are not 0 and 9...!", 0);
          }
       }
       else
          return VALUE_TOO_HIGH;


       do {
             u16_temp_time = Cntr_3125;
             VAR(AX0_u16_First_Time_Stamp_Location) = s64_Execution_Parameter[0];
             VAR(AX0_u16_Second_Time_Stamp_Location) = s64_Execution_Parameter[1];
             VAR(AX0_u32_Max_Time_Stamp_Period) = 0;
             VAR(AX0_u32_Min_Time_Stamp_Period) = 0xFFFFFFFF;
          } while (u16_temp_time != Cntr_3125);
    }

    else //0
    {
       PrintString("1st location: ", 0);
       PrintUnsignedInt16(AX0_u16_First_Time_Stamp_Location);
       PrintString(". 2nd location: ", 0);
       PrintUnsignedInt16(AX0_u16_Second_Time_Stamp_Location);
       PrintCrLf();
       PrintString(" Min Period: ", 0);
       PrintUnsignedInt32(AX0_u32_Min_Time_Stamp_Period/0.3);
       PrintString("ns.  Max Period: ", 0);
       PrintUnsignedInt32(AX0_u32_Max_Time_Stamp_Period/0.3);
       PrintString("ns.  MTS Iteration: #", 0);
       PrintUnsignedInt16(AX0_u16_MtsIsr_TimeStamp_Iteration);
       PrintCrLf();
    }
    return SAL_SUCCESS;
}


int SalTimeStampBgLocations(int drive)
{
   REFERENCE_TO_DRIVE;

   if (2 == s16_Number_Of_Parameters)
   {
      if (s64_Execution_Parameter[0] >= s64_Execution_Parameter[1])
         return VALUE_IS_NOT_ALLOWED;

      u16_Bg_First_Time_Stamp = (unsigned int)s64_Execution_Parameter[0];
      u16_Bg_Last_Time_Stamp = (unsigned int)s64_Execution_Parameter[1];
      u16_Bg_Max_Interval_Time_3125 = 0;
      u16_Bg_Min_Interval_Time_3125 = 0x7FFF;
   }

   else // 0
   {
      PrintString("1st Stamp: ", 0);
      PrintUnsignedInt16(u16_Bg_First_Time_Stamp);
      PrintString(".  2nd Stamp: ", 0);
      PrintUnsignedInt16(u16_Bg_Last_Time_Stamp);
      PrintCrLf();
      PrintString(" Min. Interval: ", 0);
      PrintUnsignedLongLong((unsigned long long)u16_Bg_Min_Interval_Time_3125 * 32);
      PrintString("mSec.  Max. Interval: ", 0);
      PrintUnsignedLongLong((unsigned long long)u16_Bg_Max_Interval_Time_3125 * 32);
      PrintString("mSec.", 0);
      PrintCrLf();
   }
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalBiSSCInfoCommand
// Description:
//   This function called in response to BISSCINFO command
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//************************************************************
int SalBiSSCInfoCommand(int drive)
{
   return (BiSSCInfoCommand(drive, FB_NOT_IN_USE));
}


//**********************************************************
// Function Name: BiSSCInfoCommand
// Description:
//   This function is called in response to BISSCINFO command
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//***********************************************************
int BiSSCInfoCommand(int drive, int s16_fb_use)
{
#define BISSC_INFO_DRIVER_REQUEST_STATE      40
#define BISSC_INFO_DRIVER_RESPONSE_STATE     (BISSC_INFO_DRIVER_REQUEST_STATE + 1)
   // AXIS_OFF;
   static int state = 0, req_state = 0;
   static long s32_acquisition_timer = 0L;
   static unsigned long u32_lock = 0UL;
   static unsigned int u16_address = 0, u16_high_address = 0;
   static char s8_value = 0;
   static unsigned char u8_user_selected_bank = 0;
   static int s16_bissc_drv_response = SAL_NOT_FINISHED;
   static BiSSC_DrvReq bissc_req = BISSC_DRVREQ_READ;
   unsigned int fields[4];
   int ret_val = SAL_NOT_FINISHED;

   if (s16_fb_use == FB_NOT_IN_USE)
   {
      if (OutputBufferLoader() == 0)
         return SAL_NOT_FINISHED;
   }

   // If not yet ready
   if(BGVAR(s16_DisableInputs) & BISSC_DIS_MASK)
   {
      state = 0;
      return NOT_AVAILABLE;
   }

   // Do not continue if BiSS-C is not in use
   if(!FEEDBACK_BISSC && (SFB_BISSC && !IS_SECONDARY_FEEDBACK_ENABLED))
   {
      state = 0;
      return NOT_AVAILABLE;
   }

   // Initialize the output buffer
   strcpy((char*)&u16_General_Domain[0],"\0");

   if (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK)
   {
       if (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_TIME_OUT_ERROR_MASK)
       {
            strcat((char*)&u16_General_Domain[0],"Timeout fault detected on protocol level\r\n");
       }
       if (VAR(AX0_u16_Comm_Fdbk_Flags_A_1) & COMM_FDBK_CRC_ERROR_MASK)
       {
            strcat((char*)&u16_General_Domain[0],"CRC fault detected on protocol level\r\n");
       }
       strcat((char*)&u16_General_Domain[0],"Issue CLEARFAULTS\r\n");

      /* Release BiSS-C device */
      BiSSC_Release(&u32_lock, drive);
      u32_lock = 0;
      s32_acquisition_timer = 0;
      state = 0;
      if (s16_fb_use == FB_NOT_IN_USE)
         OutputBufferLoader();

      return SAL_SUCCESS;
   }

   /* The following state machine contains the BISSC_INFO_DRIVER_REQUEST_STATE and
    * BISSC_INFO_DRIVER_RESPONSE_STATE states, which are common for each address read/write
    * request.
    * Each information state sets the addresses range to be read or written, a value to be
    * written in the case of a write request, and requests the BiSS-C driver to handle it.
    */

   switch(state)
   {
      case 0:// Initialization state
         // Print label
         strcat((char*)&u16_General_Domain[0],"Generic BiSS-C Device Information\r\n");
         strcat((char*)&u16_General_Domain[0],"------------------------------\r\n");
         // Initialize static variables
         req_state               = 0;
         s8_value                = 0;
         u32_lock                = 0;
         s32_acquisition_timer   = 0;
         state                   = 0;
         u16_address             = 0;
         u16_high_address        = 0;
         s16_bissc_drv_response  = SAL_NOT_FINISHED;
         bissc_req               = BISSC_DRVREQ_READ;
         // take acquisition timer
         s32_acquisition_timer = Cntr_1mS;
         state++;
      break;

      case 1:// Acquire the BiSS-C device
         ret_val = BiSSC_Acquire(&u32_lock, drive);
         if (ret_val == SAL_SUCCESS)
         {
            ret_val = SAL_NOT_FINISHED;
            state++;
            break;
         }
         if (PassedTimeMS(BISSC_ACQ_TIMEOUT_mSEC, s32_acquisition_timer))
            ret_val = BISSC_DRV_ACQ_TIMEOUT;
      break;

      case 2:// read user selected bank in order to set it back after we finish
         strcat((char*)&u16_General_Domain[0],"Selected register bank:\r\n");
         u16_address       = BISSC_REG_BANK_SELECT_ADDR;
         u16_high_address  = BISSC_REG_BANK_SELECT_ADDR;
         bissc_req         = BISSC_DRVREQ_READ;
         req_state         = state;
         state             = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
      break;

      case 3:// store the user selected bank requested from the previous state
         u8_user_selected_bank = (unsigned char)(s8_value & 0xFF);
         state++;
      break;

      case 4:// read the Device ID
         strcat((char*)&u16_General_Domain[0],"Device ID:\r\n");
         u16_address       = BISSC_REG_DEVICE_ID_LOW_ADDR;
         u16_high_address  = BISSC_REG_DEVICE_ID_HIGH_ADDR;
         bissc_req         = BISSC_DRVREQ_READ;
         req_state         = state;
         state             = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
      break;

      case 5:// read the Manufacturer ID
         strcat((char*)&u16_General_Domain[0],"Manufacturer ID:\r\n");
         u16_address       = BISSC_REG_MANU_ID_LOW_ADDR;
         u16_high_address  = BISSC_REG_MANU_ID_HIGH_ADDR;
         bissc_req         = BISSC_DRVREQ_READ;
         req_state         = state;
         state             = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
      break;

      case 6:// read the Profile ID
         strcat((char*)&u16_General_Domain[0],"Profile ID:\r\n");
         u16_address       = BISSC_REG_PROFILE_ID1_ADDR;
         u16_high_address  = BISSC_REG_PROFILE_ID2_ADDR;
         bissc_req         = BISSC_DRVREQ_READ;
         req_state         = state;
         state             = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
      break;

      case 7:// read the Serial Number
         strcat((char*)&u16_General_Domain[0],"Serial Number:\r\n");
         u16_address       = BISSC_REG_SER_NUM_LOW_ADDR;
         u16_high_address  = BISSC_REG_SER_NUM_HIGH_ADDR;
         bissc_req         = BISSC_DRVREQ_READ;
         req_state         = state;
         state             = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
      break;

      case 8:// read the EDS version 1
         // read the EDS common bank address register BISSC_REG_EDS_COMMON_BANK_ADDR to get the EDS common bank number
         strcat((char*)&u16_General_Domain[0],"EDS version:\r\n");
         u16_address       = BISSC_REG_EDS_COMMON_BANK_ADDR;
         u16_high_address  = BISSC_REG_EDS_COMMON_BANK_ADDR;
         bissc_req         = BISSC_DRVREQ_READ;
         req_state         = state;
         state = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
      break;

      case 9:// read the EDS version 2
         // write the resulted value to the bank selection register BISSC_REG_BANK_SELECT_ADDR
         u16_address = BISSC_REG_BANK_SELECT_ADDR;
         bissc_req   = BISSC_DRVREQ_WRITE;
         s8_value    = s8_value; // for better readability
         req_state   = state;
         state       = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
      break;

      case 10:// read the EDS version 3
         // read the EDS version from EDS common bank at address BISSC_REG_EDS_VERSION_ADDR
         u16_address       = BISSC_REG_EDS_VERSION_ADDR;
         u16_high_address  = BISSC_REG_EDS_VERSION_ADDR;
         bissc_req         = BISSC_DRVREQ_READ;
         req_state         = state;
         state             = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
      break;

      case 11:// read the Profile version 1
         // read the profile bank address register BISSC_REG_PROFILE_BANK_ADDR
         strcat((char*)&u16_General_Domain[0],"Profile version:\r\n");
         u16_address       = BISSC_REG_PROFILE_BANK_ADDR;
         u16_high_address  = BISSC_REG_PROFILE_BANK_ADDR;
         bissc_req         = BISSC_DRVREQ_READ;
         req_state         = state;
         state             = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
         // write the resulted value to the bank selection register BISSC_REG_BANK_SELECT_ADDR
         // read the profile version from profile bank at address BISSC_REG_PRFL_BANK_VERSION_ADDR
      break;

      case 12:// read the Profile version 2
         // write the resulted value to the bank selection register BISSC_REG_BANK_SELECT_ADDR
         u16_address = BISSC_REG_BANK_SELECT_ADDR;
         bissc_req   = BISSC_DRVREQ_WRITE;
         s8_value    = s8_value; // for better readability
         req_state   = state;
         state = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
         // read the profile version from profile bank at address BISSC_REG_PRFL_BANK_VERSION_ADDR
      break;

      case 13:// read the Profile version 3
         // read the profile version from profile bank at address BISSC_REG_PRFL_BANK_VERSION_ADDR
         u16_address       = BISSC_REG_PRFL_BANK_VERSION_ADDR;
         u16_high_address  = BISSC_REG_PRFL_BANK_VERSION_ADDR;
         bissc_req         = BISSC_DRVREQ_READ;
         req_state         = state;
         state             = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
      break;

      case 14:// read the Encoder Type
         // here we don't need to change the register bank,
         // since the Encoder Type register is located at the same EDS profile bank as previous register (Profile version)
         strcat((char*)&u16_General_Domain[0],"Encoder Type:\r\n");
         u16_address       = BISSC_REG_PRFL_BANK_EN_TYPE_ADDR;
         u16_high_address  = BISSC_REG_PRFL_BANK_EN_TYPE_ADDR;
         bissc_req         = BISSC_DRVREQ_READ;
         req_state         = state;
         state             = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
      break;

      case 15:// read the MT resolution
         // Anatoly TODO print MT resolution, read from regisers???
         if (BiSSC_GetDataFields(fields, drive) != SAL_SUCCESS)
         {
            strcat((char*)&u16_General_Domain[0],"Not Available\r\n");
         }
         else
         {
            strcat((char*)&u16_General_Domain[0],"Multi-turn resolution:\r\n");
            strcat((char*)&u16_General_Domain[0],UnsignedIntegerToAscii(fields[1]));
            strcat((char*)&u16_General_Domain[0], " Bits\r\n");
         }
         state++;
      break;

      case 16:// read the ST resolution
         // Anatoly TODO print ST resolution, read from regisers???
         if (BiSSC_GetDataFields(fields, drive) != SAL_SUCCESS)
         {
            strcat((char*)&u16_General_Domain[0],"Not Available\r\n");
         }
         else
         {
            strcat((char*)&u16_General_Domain[0],"Single-turn resolution:\r\n");
            strcat((char*)&u16_General_Domain[0],UnsignedIntegerToAscii(fields[3]));
            strcat((char*)&u16_General_Domain[0], " Bits\r\n");
         }
         state++;
      break;

      case 17:// read the Temperature
         // Anatoly TODO print ST resolution, read from regisers??? Manufacturer specific?
         strcat((char*)&u16_General_Domain[0],"Temperature:\r\n");
         strcat((char*)&u16_General_Domain[0],"Not Available\r\n");
         state++;
      break;

      case 18:// Faults and Warnings
         // capture the status
         u16_address = BiSSC_GetStatus(drive, 0);
         // parse the status
         if (u16_address & BISSC_STATUS_FAULT_MASK)
            strcat((char*)&u16_General_Domain[0],"BiSS-C device indicates a fault !!!\r\n");
         if (u16_address & BISSC_STATUS_WARN_MASK)
            strcat((char*)&u16_General_Domain[0],"BiSS-C device indicates a warning !!!\r\n");
         if (u16_address == 0)
            strcat((char*)&u16_General_Domain[0],"No faults/No warnings are detected on BiSS-C device.\r\n");
         state++;
      break;

      case 19:// Switch back to the user selected bank
         // write the resulted value to the bank selection register BISSC_REG_BANK_SELECT_ADDR
         u16_address = BISSC_REG_BANK_SELECT_ADDR;
         bissc_req   = BISSC_DRVREQ_WRITE;
         s8_value    = u8_user_selected_bank;
         req_state   = state;
         state       = BISSC_INFO_DRIVER_REQUEST_STATE;// driver request common state
      break;

      case 20:
         ret_val = SAL_SUCCESS;
      break;

      case BISSC_INFO_DRIVER_REQUEST_STATE:// driver request common state
         s16_bissc_drv_response = BiSSC_Request(bissc_req, u32_lock, u16_address, s8_value, &s8_value, drive);
         if (s16_bissc_drv_response == SAL_SUCCESS)
         {
            if (bissc_req == BISSC_DRVREQ_READ)
            {
               strcat((char*)&u16_General_Domain[0], DecToAsciiHex((unsigned long long)u16_address, 2));
               strcat((char*)&u16_General_Domain[0]," = ");
               strcat((char*)&u16_General_Domain[0], DecToAsciiHex((unsigned long long)s8_value, 2));
               strcat((char*)&u16_General_Domain[0],"  ");
            }
            u16_address++;
            if (u16_address > u16_high_address)// if we finished --> go to the state next to the one which requested register access
            {
               if (bissc_req == BISSC_DRVREQ_READ)
                  strcat((char*)&u16_General_Domain[0], "\r\n");
               state = req_state + 1;// transition to the next information state
            }
         }
         else if (s16_bissc_drv_response != SAL_NOT_FINISHED)
         {
            state = BISSC_INFO_DRIVER_RESPONSE_STATE;
         }
      break;

      case BISSC_INFO_DRIVER_RESPONSE_STATE:// driver response common state
         if ((s16_bissc_drv_response == BISSC_ADDR_OUT_OF_RANGE) || (s16_bissc_drv_response == NOT_AVAILABLE))
            strcat((char*)&u16_General_Domain[0],"Not Available\r\n");
         else if (s16_bissc_drv_response != SAL_SUCCESS)
            ret_val = s16_bissc_drv_response;// Anatoly TODO debug here we stop the operation of the command??
         state = req_state + 1;// transition to the next information state
      break;

      default:
         ret_val = BISSC_DRV_ERROR;
      break;
   }

   if (ret_val != SAL_NOT_FINISHED)
   {  // Release BiSS-C device
      BiSSC_Release(&u32_lock, drive);
      u32_lock                = 0;
      s32_acquisition_timer   = 0;
      state                   = 0;
   }

   // At the last iteration,printing will be handled here.
   if (ret_val == SAL_SUCCESS)
   {
      if (s16_fb_use == FB_NOT_IN_USE)
      {
         OutputBufferLoader();
      }
   }
   return ret_val;
}


//**********************************************************
// Function Name: SalGearDeadBandLimit
// Description://
//
// Author: Lior
// Algorithm:
// Revisions:
// Note:
//************************************************************
int SalGearDeadBandLimit (int drive)
{
   // AXIS_OFF;   
   int s16_temp_value = 0;
   REFERENCE_TO_DRIVE;
   if (s16_Number_Of_Parameters == 0)
   {
      PrintSignedInteger(VAR(AX0_s16_One_Dir_Gear_DB_Limit_Value)>>1);
      PrintCrLf();
   }
   else
   {
      s16_temp_value = (int)s64_Execution_Parameter[0];
      VAR(AX0_s16_One_Dir_Gear_DB_Limit_Value) = s16_temp_value<<1;
   }
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalBiSSCRead
// Description:
//   This function called in response to BISSCREAD serial command
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//************************************************************
int SalBiSSCRead(int drive)
{
   int value, ret_val;

   if (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK)
   {
      PrintOnFdbkCommFaultMessage(drive);
      return SAL_SUCCESS;
   }

   if ((ret_val = BiSSC_ReadRegister((unsigned int)s64_Execution_Parameter[0], &value, drive)) == SAL_SUCCESS)
   {
      PrintDecInAsciiHex((unsigned long long)value, 2);
      PrintCrLf();
   }
   return ret_val;
}


//**********************************************************
// Function Name: SalBiSSCWrite
// Description:
//   This function called in response to BISSCWRITE serial command
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//************************************************************
int SalBiSSCWrite(int drive)
{
   if (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK)
   {
      PrintOnFdbkCommFaultMessage(drive);
      return SAL_SUCCESS;
   }

   return(BiSSC_WriteRegister((unsigned int)s64_Execution_Parameter[0], (unsigned int)s64_Execution_Parameter[1], drive));
}


//**********************************************************
// Function Name: SalBiSSCCmd
// Description:
//   This function called in response to BISSCCMD serial command
//
//
// Author: Anatoly Odler
// Algorithm:
// Revisions:
// Note:
//************************************************************
int SalBiSSCCmd(int drive)
{
   if (BGVAR(s64_SysNotOk) & COMM_FDBK_FLT_MASK)
   {
      PrintOnFdbkCommFaultMessage(drive);
      return SAL_SUCCESS;
   }

   return (BiSSC_SendCommand((unsigned int)s64_Execution_Parameter[0], drive));
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function Name: SalBisscDataFieldsWriteCommand
//
// Description: This function is called in response to the BISSCFIELDS <Params> command.
//
// BISSCFIELDS <Multi-Turn Length> <MT Resolution> <Single-Turn Length> <ST Resolution>
//
// * Multi-Turn Length - Number of Bits dedicated to Multi-Turn Data
// * MT Resolution - MT Resolution expressed in Bits; if this is less than MT Length
//   then the Data is Right-Aligned and padded with Zeros.
// * Single-Turn Length - Number of Bits dedicated to Single-Turn Data
// * ST Resolution - ST Resolution expressed in Bits; if this is less than ST Length
//   then the Data is Left-Aligned and padded with Zeros.
// "Resolution" must be equal to or less than respective "Length".
// MT Length (and MT Resolution) may be Zero for Signle-Turn or Linear Devices.
//
// Example : BISSCFIELDS 12 12 24 19 for 4,096 Multi-Turn, 19-Bits / Revolution Device.
//
// Author: Anatoly Odler/ A.H.
// Algorithm:
// Revisions:
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int SalBisscDataFieldsWriteCommand(int drive)
{
   unsigned int index, fields[4];

   if (s16_Number_Of_Parameters != 4)
      return SYNTAX_ERROR;

   for (index = 0; index < 4; index++)   // Set the BiSS-C fields
      fields[index] = (unsigned int)s64_Execution_Parameter[index];

   return(BiSSC_SetDataFields(fields, drive));
}
int SalHmiBiss(long long param,int drive)
{
   
   s16_Number_Of_Parameters = 4;
   s64_Execution_Parameter[0] = s64_Biss_1;
   s64_Execution_Parameter[1] = s64_Biss_2;
   s64_Execution_Parameter[2] = s64_Biss_3;
   s64_Execution_Parameter[3] = s64_Biss_4 = param;
   return SalBisscDataFieldsWriteCommand(drive);
}

//////////////////////////////////////////////////////////////////////////
// Function Name: BisscDataFieldsReadCommand
// Description: This function is called in response to the BISSCFIELDS command.
// The Function returns the Data Field values:  Multi-Turn Length, MT Resolution,
// Single-Turn Length, and ST Resolution.
//
// Author: Anatoly Odler/ A.H.
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalBisscDataFieldsReadCommand(int drive)
{
   return (BisscDataFieldsReadCommand(drive, FB_NOT_IN_USE));
}


//////////////////////////////////////////////////////////////////////////
// Function Name: BisscDataFieldsReadCommand
// Description: This function is called in response to the BISSCFIELDS command.
// The Function returns the Data Field values:  Multi-Turn Length, MT Resolution,
// Single-Turn Length, and ST Resolution.
//
// Author: Anatoly Odler/ A.H.
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int BisscDataFieldsReadCommand(int drive, int s16_fb_use)
{
   unsigned int ret_val, fields[4];

   if (s16_fb_use == FB_NOT_IN_USE)
   {
      if (OutputBufferLoader() == 0)
         return SAL_NOT_FINISHED;
   }

   if ((ret_val = BiSSC_GetDataFields(fields, drive)) == SAL_SUCCESS)
   {
      strcpy((char*)&u16_General_Domain[0],"\0");      // Initialize the output buffer

      // Write response buffer
      strcat((char*)&u16_General_Domain[0], UnsignedIntegerToAscii(fields[0]));
      strcat((char*)&u16_General_Domain[0], " ");
      strcat((char*)&u16_General_Domain[0], UnsignedIntegerToAscii(fields[1]));
      strcat((char*)&u16_General_Domain[0], " ");
      strcat((char*)&u16_General_Domain[0], UnsignedIntegerToAscii(fields[2]));
      strcat((char*)&u16_General_Domain[0], " ");
      strcat((char*)&u16_General_Domain[0], UnsignedIntegerToAscii(fields[3]));
      strcat((char*)&u16_General_Domain[0], "\r\n");

      if (s16_fb_use == FB_NOT_IN_USE)      // Write output buffer
      {
         OutputBufferLoader();
      }
   }
   return ret_val;
}


int SalRelayModeCommand (long long param,int drive)
{
   REFERENCE_TO_DRIVE;
   if ((!IS_HW_FUNC_ENABLED(FAULT_RELAY_MASK)) && (2 != param)) return NOT_SUPPORTED_ON_HW;

   BGVAR(u16_Relay_Mode) = (int)param;
   return SAL_SUCCESS;
}


//////////////////////////////////////////////////////////////////////////
// Function Name: SalCustomerIdCommand
// Description: Set CUSTOMERID paramter

// Author: S.C.
// Algorithm:
// Revisions:
//////////////////////////////////////////////////////////////////////////
int SalCustomerIdCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;
   //new customer id is different from current val
   if ( u16_Customer_Id != (unsigned int)param )
   {
      //revert to default values     
      if ( DEFAULT_CUSTOMER_ID == param )
      {
         u16_OV_Threshold = u16_Power1_Board_Eeprom_Buffer[OV_THRESH_ADDR];
         u16_Regen_Off_Value = u16_Power1_Board_Eeprom_Buffer[REGEN_LO_TIME_ADDR];
         u16_Regen_On_Value = u16_Power1_Board_Eeprom_Buffer[REGEN_HI_TIME_ADDR];
      }
      u16_Customer_Id = (unsigned int)param;
   }
   return SAL_SUCCESS;
}

/* I/F between drive and GUI
   read command with one parameter GUIPARAM <index>

   index         meaning
     1           Present a counter which increments everytime a command is written
*/
int GuiParamCommand(int drive)
{
   int s16_index = (int)s64_Execution_Parameter[0];
   REFERENCE_TO_DRIVE;

   if (s16_Number_Of_Parameters != 1) return (NOT_AVAILABLE);


   if (s16_index != 1) return (VALUE_OUT_OF_RANGE);

   switch (s16_index)
   {
      case  1: // Show the write commands counter
         PrintUnsignedInt16(BGVAR(s16_Write_Cmd_Cntr));
      break;
   }

   PrintCrLf();
   return (SAL_SUCCESS);
}

/* I/F for Testing
   read command with one parameter TESTVAR <index>

   index         meaning
	TESTVAR 1 - Motor serial number
	TESTVAR 2 - Database version
	TESTVAR 3 - Power
	TESTVAR 4 - Maximum temperature 
	TESTVAR 5 - Brake type
	TESTVAR 6 - Brake current
	TESTVAR 7 - Brake close Time
	TESTVAR 8 - Brake open Time
*/
int TestVarCommand(int drive)
{
   int s16_index = (int)s64_Execution_Parameter[0];
   REFERENCE_TO_DRIVE;

   if (s16_Number_Of_Parameters != 1) return (NOT_AVAILABLE);

   if (s16_index < 1) return VALUE_TOO_LOW;
   if (s16_index > 8) return VALUE_TOO_HIGH;

   switch (s16_index)
   {
      case  1: // Serial number
         PrintString(u8_MotorSerialNumber, 0);
      break;
      case 2: // Database version
         PrintUnsignedInt16(u16_MTP_Database_Version);
      break;
      case 3: // Power
         PrintUnsignedInt32(u32_MTP_Power);
      break;
      case 4: // Max temperature
         PrintUnsignedInt16(s16_ServoSense_Therm_Trip_Level);
      break;
      case 5: // Brake type
         PrintUnsignedInt16(u16_Motor_Brake_Type);
      break;
      case 6: // Brake current
         PrintUnsignedInt32(u32_Motor_Brake_I);
      break;
      case 7: // Brake close time
         PrintUnsignedInt16(u16_Motor_Brake_Engage_Time);
      break;
      case 8: // Brake open time
         PrintUnsignedInt16(u16_Motor_Brake_Release_Time);
      break;
      default: 
         PrintStringCrLf("NA",0);
      break;
   }

   PrintCrLf();
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalEcReadSdoCommand
// Description:
//          This function is called in response to ASCII command ECSENDSDO
//
//
// Author: Itai Raz
// Algorithm:
// Revisions:
//**********************************************************
int SalEcReadSdoCommand(void)
{
   int drive = 0;
   unsigned int u16_index = 0;
   unsigned int u16_sub_index = 0;
   int ret_val = SAL_SUCCESS;
   int* p_s16_bg_rx_data = 0;
   long s32_value = 0;
   static int i = 0;
   static int s16_state = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error
   if(s16_state == 0)
   {
      // If no EtherCAT Drive or COMMODE unequal 1
      if((!IS_EC_DRIVE_AND_COMMODE_1)  && (!IS_CAN_DRIVE_AND_COMMODE_1))
      {
         PrintStringCrLf("No EtherCAT Drive or communication-mode unequal 1.",0);
         return SAL_SUCCESS ;
      }

      // no limits on parameter 1 (SDO index) & parameter 2 (SDO sub index)   
      u16_index = (unsigned int)s64_Execution_Parameter[0];
      u16_sub_index = (unsigned int)s64_Execution_Parameter[1];

      // search for the object ID in the local Field Bus objects array
      for(i = 1; i < ((int)SDO_LAST_OBJECT) && ((FB_objects_array[i].u16_index != u16_index) || (FB_objects_array[i].u8_sub_index != u16_sub_index)); i++);

      if(i >= (int)SDO_LAST_OBJECT)// return the response immediately if the object is not in use of the Field Bus FW unit
      {
         // initiate abort on fail            
         PrintStringCrLf("SDO not found.",0);
         return SAL_SUCCESS ;
	  }
   
      // Write internal CDHD-id to a global variable due to the "FalExecuteFieldBusCommandBg"
      // later in the code (called via function pointer "FB_objects_array[SDO_GENERIC_OBJECT].fp_bg".
      BGVAR(u16_fb_curr_obj_id) = i;

      ret_val = FalCheckObjectDefinition(FB_objects_size_array[i], 0, 0);
   
      if(ret_val == FAL_SUCCESS)
      {
         ret_val = FalCheckObjectValidity(0, (unsigned int)FB_objects_size_array[i], p_s16_bg_rx_data, drive);
      }   

      if(ret_val != FAL_SUCCESS)
      {
         // initiate abort on fail            
         PrintStringCrLf("SDO not valid.",0);
         return SAL_SUCCESS ;
      }
      s16_state = 1;      
   }        
   
   if(s16_state == 1)
   {
      //execute the matching function with the data recived  (for both read & write)
      // Call "FalExecuteFieldBusCommandBg" with data, CAN object-id and operation-variable and 0xFFFF in message ID to mark that this is a fake  SDO coming from ASCII (bugzilla 5830)
      ret_val = FB_objects_array[i].fp_bg(p_s16_bg_rx_data,0xFFFF, 0);

      if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
      {    
    	 s16_state = 0;
         return ret_val;
      }
      else if(ret_val == SAL_NOT_FINISHED)
      {
         //the command execution has not been finished yet
         //request to handle the command again:
         return SAL_NOT_FINISHED;
      }

      else//SAL_SUCCESS
	  {
	     if(FB_objects_size_array[i] == 1)
            s32_value = (*(int *)(FB_objects_array[i].u32_addr));
	     if(FB_objects_size_array[i] == 2)
            s32_value = (*(long *)(FB_objects_array[i].u32_addr));

         //Extra printing
         //PrintString("SDO Index: ", 0);
         //PrintDecInAsciiHex(u16_index,4);
         //PrintString(", Sub Index: ", 0);
         //PrintDecInAsciiHex(u16_sub_index,4);
         //PrintString("  Value = ", 0);
  
         PrintSignedLong(s32_value);PrintCrLf();
	     s16_state = 0;
		 return (SAL_SUCCESS);
      }        
   }
   //s16_state is not 0 and not 1 - error state
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalEcWriteSdoCommand
// Description:
//          This function is called in response to ASCII command ECSENDSDO
//
//
// Author: Itai Raz
// Algorithm:
// Revisions:
//**********************************************************

int SalEcWriteSdoCommand(void)
{
   int drive = 0;
   unsigned int u16_index = 0;
   unsigned int u16_sub_index = 0;
   long s32_value = 0L;
   int ret_val = SAL_SUCCESS;
   int* p_s16_bg_rx_data = 0;
   static int i = 0;
   static int s16_state = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error
   if(s16_state == 0)
   {
      // If no EtherCAT Drive or COMMODE unequal 1
      if((!IS_EC_DRIVE_AND_COMMODE_1)  && (!IS_CAN_DRIVE_AND_COMMODE_1))
      {
         PrintStringCrLf("No EtherCAT Drive or communication-mode unequal 1.",0);
         return SAL_SUCCESS ;
      }

      // no limits on parameter 1 (SDO index) & parameter 2 (SDO sub index)  & parameter 3 (SDO value)
      u16_index = (unsigned int)s64_Execution_Parameter[0];
      u16_sub_index = (unsigned int)s64_Execution_Parameter[1];
	  s32_value = (long)s64_Execution_Parameter[2];

      // search for the object ID in the local Field Bus objects array
      for(i = 1; i < ((int)SDO_LAST_OBJECT) && ((FB_objects_array[i].u16_index != u16_index) || (FB_objects_array[i].u8_sub_index != u16_sub_index)); i++);

      if(i >= (int)SDO_LAST_OBJECT)// return the response immediately if the object is not in use of the Field Bus FW unit
      {
         // initiate abort on fail            
         PrintStringCrLf("SDO not found.",0);
         return SAL_SUCCESS ;
	  }
   
      // Write internal CDHD-id to a global variable due to the "FalExecuteFieldBusCommandBg"
      // later in the code (called via function pointer "FB_objects_array[SDO_GENERIC_OBJECT].fp_bg".
      BGVAR(u16_fb_curr_obj_id) = i;
      
      ret_val = FalCheckObjectDefinition(FB_objects_size_array[i], 0, 0);
   
      if(ret_val == FAL_SUCCESS)
      {
         ret_val = FalCheckObjectValidity(0, (unsigned int)FB_objects_size_array[i], p_s16_bg_rx_data, drive);
      }   

      if(ret_val != FAL_SUCCESS)
      {
         // initiate abort on fail            
         PrintStringCrLf("SDO not valid.",0);
         return SAL_SUCCESS ;
      }
      s16_state = 1;      
   }        
   
   if(s16_state == 1)
   {
      p_s16_bg_rx_data[0] = s32_value & 0xFFFF;        //set 16 LSBits
      p_s16_bg_rx_data[1] = s32_value >> 16;           //set 16 MSBits

      //execute the matching function with the data recived  (for both read & write)
      // Call "FalExecuteFieldBusCommandBg" with data, CAN object-id  and 0xFFFF in message ID to mark that this is a fake  SDO coming from ASCII (bugzilla 5830)
      ret_val = FB_objects_array[i].fp_bg(p_s16_bg_rx_data,0xFFFF, 1);

      if((ret_val != FAL_SUCCESS) && (ret_val != SAL_NOT_FINISHED))
      {    
    	 s16_state = 0;
         return ret_val;
      }
      else if(ret_val == SAL_NOT_FINISHED)
      {
         //the command execution has not been finished yet
         //request to handle the command again:
         return SAL_NOT_FINISHED;
      }

      else//SAL_SUCCESS
	  {
	     s16_state = 0;
		 return (SAL_SUCCESS);
      }        
   }
   //s16_state is not 0 and not 1 - error state
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalIgnoreAbsFdbkBattFlt
// Description:
//          This function is called in response to ASCII command IGNOREBATTFLT
//          The function is responsible for the configuration of the
//          ignorance of the battery and other MT related faults on the encoder.
//          See design document for IGNOREBATTFLT for more information.
//          long long param - the ignorance mode:
//             1 - ignorance active.
//             0 - ignorance inactive.
//          Return value:
//          SAL_SUCCESS       - on success.
//          SAL_NOT_FINISHED  - not finished yet.
//          Error otherwise.
//          The error indicates that actual configuration on the the encoder side failed.
//          The error code can be observed over a global variable u16_Ignore_Abs_Fdbk_Batt_Faults_Error.
//          The ignorance mode is written regardless the
// Author: A.O.
// Algorithm:
// Revisions:
//**********************************************************
int SalIgnoreAbsFdbkBattFlt(long long param,int drive)
{
   register int ret_val = SAL_NOT_FINISHED;
   register unsigned int u16_mode = (unsigned int)param;// caching

   //BGVAR(u16_Ignore_Abs_Fdbk_Batt_Faults) = (unsigned int)param;

   switch(BGVAR(u16_FdbkType))
   {
      case NK_COMM_FDBK:
      case TAMAGAWA_COMM_MULTI_TURN_FDBK:
      case FANUC_COMM_FDBK:
      case PS_S_COMM_FDBK:
      case SANKYO_COMM_FDBK:
      case YASKAWA_ABS_COMM_FDBK:
      case YASKAWA_INC_COMM_FDBK:
         ret_val = CommonFdbkIgnoreMTFaults(u16_mode, drive);
      break;

      case SERVOSENSE_MULTI_TURN_COMM_FDBK:
         ret_val = SrvSns_IgnoreMTFaults(u16_mode, drive, 0); //added 0 to function call since the SWITCH only relates Feedbacktype and not SFBTYPE 
      break;

      default:
         if ( 0 == u16_mode ) 
            ret_val = SAL_SUCCESS;
         else
            ret_val = NOT_AVAILABLE; 
      break;
   }

   if(ret_val == SAL_SUCCESS)
      BGVAR(u16_Ignore_Abs_Fdbk_Batt_Faults) = u16_mode;
   else
      BGVAR(u16_Ignore_Abs_Fdbk_Batt_Faults_Error) = ret_val;

   return ret_val;
}
//**********************************************************
// Function Name: SalActualFBSyncTimeRead
// Description:
//          This function returns the measured Fieldbus sync time in ms
// Author: S.C.
// Algorithm:
// Revisions:
//**********************************************************
int SalActualFBSyncTimeRead(long long *data,int drive)
{
   drive+=0;
   //if EtherCAT
   if (s16_Fieldbus_Flags == 0x0005)
      *data = (unsigned int)((unsigned int)u32_Controller_Sync_Period_in_32Khz/**(unsigned int)100*//(unsigned int)32);
   //else if CAN
   else if (s16_Fieldbus_Flags == 0x0003)
      *data = (unsigned int)((unsigned int)u32_Controller_Sync_Period_in_32Khz/(unsigned int)1600);
   return SAL_SUCCESS;

}
