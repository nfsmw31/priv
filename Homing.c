#include "Math.h"
#include "co_util.h"
#include "objects.h"
#include "402fsa.def"
#include "co_emcy.h"

#include "Design.def"
#include "Err_Hndl.def"
#include "EXE_IO.def"
#include "FltCntrl.def"
#include "FPGA.def"
#include "Homing.def"
#include "ModCntrl.def"
#include "MultiAxis.def"
#include "PtpGenerator.def"
#include "Init.def"
#include "PhaseFind.def"

#include "burnin.var"
#include "Drive.var"
#include "Exe_IO.var"
#include "Extrn_Asm.var"
#include "FltCntrl.var"
#include "Foldback.var"
#include "Homing.var"
#include "ModCntrl.var"
#include "MotorSetup.var"
#include "Position.var"
#include "PtpGenerator.var"
#include "Ser_Comm.var"
#include "Units.var"
#include "PhaseFind.var"

#include "Prototypes.pro"


//**********************************************************
// Function Name: HomeCommand
// Description:
//          This function is called in response to the HOMECMD command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int HomeCommand(int drive)
{
   // AXIS_OFF;
   unsigned int u16_Trigger_Switch, u16_Mask, u16_trans_bits_ls, u16_local_cw_ls, u16_local_ccw_ls;
   int s16_fail_reason;   
   unsigned u16_temp_movesmooth_src = 0;
   //assaign the relevant signed pfb. if dual loop active - sfb. if not active - mfb
   
   if (35 == BGVAR(s16_Home_Type)) 
   {
   /* This section is moved to the begining of HomeCommand, due to BZ#6485 (run-away when homecmd while motor is in motion): 
      Here we test if the motor is in motion, and if so, the HomeCmd command is rejected - without failing the Homing state machine. */
      if ( (!Enabled(DRIVE_PARAM) && (BGVAR(s16_Motor_Moving) != 0)) ||
           (Enabled(DRIVE_PARAM) && (LVAR(AX0_s32_Pos_Vcmd) != 0L))    )
      { 
         p402_error_code = ERRCODE_CAN_FIELDBUS_HOMING_MOTOR_IN_MOTION;
         HomeFailureIndication(drive, 0, 0, 0, MOTOR_IN_MOTION);      // set failure indication.
         return MOTOR_IN_MOTION;
      }
   }
   
   if (!IS_DUAL_LOOP_ACTIVE)
   {
      VAR(AX0_u16_DF_Pos_Fdbk_Sign_Ptr) = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_Sign_Lo & 0xFFFF);
   }
   else
   {
      VAR(AX0_u16_DF_Pos_Fdbk_Sign_Ptr) = (unsigned int)((unsigned long)&AX0_u32_Pos_Fdbk_Sign_Lo_2 & 0xFFFF);
   }

   if (s16_Number_Of_Parameters == 1) // HOMECMD 0 will abort the proceess
   {
      if (s64_Execution_Parameter[0] != 0) return VALUE_OUT_OF_RANGE;
      else
      {
         if (BGVAR(u8_Homing_State) != HOMING_IDLE)
         {
            BGVAR(u8_Homing_State) = HOMING_IDLE;
            PtpAbort(drive);

            // Homing doesn't work correctly after it is interrupted while looking for the IndexPulse
            // cancel searching for index (in case it was not reset because of homing cancellation)
            VAR(AX0_s16_Crrnt_Run_Code) &= ~ARM_HOMING_CAPTURE_MASK;
         }
         BGVAR(u8_Homing_Not_Started) = 0;
         // mark failure indication to zero (no error).
         HomeFailureIndication(drive, 0, 0, 0, 0);
         return SAL_SUCCESS;
      }
   }

   if (IsHomingProcessRunning(drive))
   {
      // set failure indication.
      HomeFailureIndication(drive, 0, 0, 0, HOMING_IN_PROGRESS);
      return HOMING_IN_PROGRESS; // do not restart Homing when in the middle of the process
   }

   //Set Homing state machine to Idle to avoid prior success indication in case homing not run
   BGVAR(u8_Homing_State) = HOMING_IDLE;

     // Check if no other procedure is running in the drive
   if (ProcedureRunning(DRIVE_PARAM) != PROC_NONE)
   {
      BGVAR(u8_Homing_Not_Started) = 1;
      p402_error_code = ERRCODE_CAN_FIELDBUS_HOMING_PROCEDURE_RUNNING;
      // set failure indication.
      HomeFailureIndication(drive, 0, 0, 0, OTHER_PROCEDURE_RUNNING);
      return OTHER_PROCEDURE_RUNNING;
   }

   if (BGVAR(s16_Home_Type) != 35)
   { // For all Home-Types other than 35, Drive must be enabled and in Opmodes 8 or 4.
      if (!Enabled(DRIVE_PARAM)) // Allow Homing with Drive Disabled
      {
         BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;         // set home state to failure

         HomeFailureIndication(drive, 0, 0, 0, DRIVE_INACTIVE);   // set failure indication.

         return DRIVE_INACTIVE; // only in Type 35 (Declare Present Position as Home)
      }
      
      if ((VAR(AX0_s16_Opmode) != 8) && (VAR(AX0_s16_Opmode) != 4)) // Allow Homing with
      {
         BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;         // set home state to failure

         HomeFailureIndication(drive, 0, 0, 0, INVALID_OPMODE);    // set failure indication.

         // Drive OPMODE other than Position Control or Gearing only in Type 35 (Declare
         return INVALID_OPMODE; // Present Position as Home)
      }
   }
   else  // here home method is 35 (home in current position).
   {
      // Execution of MC_SetPosition during a Limit Switch is active leads to an axis jump
      // if limit switch is on or software limit is on, fail homing

      // When setting Homing Limits Mode to ignore Transient (Phantom) Bits
      if ((VAR(AX0_u16_SW_Pos_lim) & 0x04) == 0x04)
      {  // Do not use the hardware limit-switch phantom bits (and sw limit)
         u16_trans_bits_ls  = 0x03;
      }
      else
      {  // Involve the hardware limit-switch phantom bits (and sw limit)
         u16_trans_bits_ls  = 0x07;
      }
      
      // take local copy of limit switch and transient bits.
      u16_local_cw_ls = VAR(AX0_u16_CW_LS) & u16_trans_bits_ls;
      u16_local_ccw_ls = VAR(AX0_u16_CCW_LS) & u16_trans_bits_ls;

      if (u16_local_cw_ls || u16_local_ccw_ls)
      {  // find the exact reason for failure
         if (VAR(u16_local_cw_ls) & 1) // positive sw limit
         {
            p402_error_code = ERRCODE_CAN_POS_SW_LIMIT;
            s16_fail_reason = CMD_EXCEEDS_SW_LIMITS;
         }
         else if (u16_local_cw_ls)  // pos limit switch or phantom bit
         {
            p402_error_code = ERRCODE_CAN_POS_LIMIT_SWITCH;
            s16_fail_reason = CW_LIMIT_SWITCH_ACTIVE;
         }
         else if (VAR(u16_local_ccw_ls) & 1) // negative sw limit
         {
            p402_error_code = ERRCODE_CAN_NEG_SW_LIMIT;
            s16_fail_reason = CMD_EXCEEDS_SW_LIMITS;
         }
         else if (u16_local_ccw_ls) // neg limit switch or phantom bit
         {
            p402_error_code = ERRCODE_CAN_NEG_LIMIT_SWITCH;
            s16_fail_reason = CCW_LIMIT_SWITCH_ACTIVE;   // bugzilla 5038 (wrong err msg)
         }

         BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;            // set home state to failure

         BGVAR(u8_Homing_Not_Started) = 1;

         HomeFailureIndication(drive, 0, 0, 0, s16_fail_reason);      // set failure indication.
         return s16_fail_reason;
      }   
   }

   if (BGVAR(s16_DisableInputs) & FLT_EXISTS_MASK)  return FAULTS_INHIBITION;

   if ( ( (((BGVAR(u64_1st_Target) >> 8) & POS_LIM_ON) != 0)        &&
          (IsInFunctionalityConfigured(CW_LIMIT_SW_INP, drive) == 0)  )                 ||
        ( (((BGVAR(u64_1st_Target) >> 8) & NEG_LIM_ON) != 0)         &&
          (IsInFunctionalityConfigured(CCW_LIMIT_SW_INP, drive) == 0)  )                ||
        ( (((BGVAR(u64_1st_Target) >> 8) & (HOME_SWITCH_ON | HOME_SWITCH_OFF)) != 0) &&
          (IsInFunctionalityConfigured(HOME_SWITCH_INP, drive) == 0)                   )  )
   {  // If any Switch is required for the Process but is not defined set home state to failure
      BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
         
      BGVAR(u8_Homing_Not_Started) = 1;
      p402_error_code = ERRCODE_CAN_FIELDBUS_HOMING_NO_DIRECTION;

      HomeFailureIndication(drive, 0, 0, 0, MISSING_DIR_REV_SOURCE);  // set failure indication.
      return MISSING_DIR_REV_SOURCE;
   }

   if (((BGVAR(u64_End_Target) >> 16) & 0xFF) == INDEX)
   {  // If Feedback has well-defined Zero-Position and has no Index Signal
     if (!IS_DUAL_LOOP_ACTIVE)
     {
         VAR(AX0_s16_DF_Run_Code) &= ~DUAL_LOOP_HOMING_INITIATE_SOFT_INDEX_MASK;
         VAR(AX0_s16_DF_Run_Code_2) &= ~DUAL_LOOP_HOMING_INITIATE_SOFT_INDEX_MASK;         
         VAR(AX0_s16_Crrnt_Run_Code) &= ~HOMING_SOFT_INDEX_MASK;
         if ( ( (!FEEDBACK_WITH_ENCODER) || FEEDBACK_STEGMANN || FEEDBACK_ENDAT ) && (!FEEDBACK_INC_YASKAWA) )
         {
            VAR(AX0_s16_Crrnt_Run_Code) |= HOMING_SOFT_INDEX_MASK; // Allow "Soft Index"
            VAR(AX0_s16_DF_Run_Code) |= DUAL_LOOP_HOMING_INITIATE_SOFT_INDEX_MASK;// allow only MFB enter rellevant code
         }
         if (FEEDBACK_INC_YASKAWA)
            *(int *)FPGA_INPUT_CAPTURE_SELECT_REG_ADD = 15; // Trigger Source Yaskawa Index-Bit
         else
         {
            *(int *)FPGA_INPUT_CAPTURE_SELECT_REG_ADD = 12; // Trigger Source Hardware Index
            VAR(AX0_u16_Fpga_Capture_Select_Value) = 12;
         }
         *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_REG_ADD = 0; // Rising Edge
     }
     else
     {
         VAR(AX0_s16_DF_Run_Code_2) &= ~DUAL_LOOP_HOMING_INITIATE_SOFT_INDEX_MASK;
         VAR(AX0_s16_DF_Run_Code) &= ~DUAL_LOOP_HOMING_INITIATE_SOFT_INDEX_MASK;
         VAR(AX0_s16_Crrnt_Run_Code) &= ~HOMING_SOFT_INDEX_MASK;
         if (ROTARY_MOTOR == LOAD_TYPE)//rotary load encoder
         {
            if (SFB_WITH_INDEX)
            {
               *(int *)FPGA_INPUT_CAPTURE_SELECT_REG_ADD = 13; // Trigger Source SFB Hardware Index 
               VAR(AX0_u16_Fpga_Capture_Select_Value) = 13;
            }
            else
            {
               VAR(AX0_s16_Crrnt_Run_Code) |= HOMING_SOFT_INDEX_MASK; // Allow "Soft Index" 
               VAR(AX0_s16_DF_Run_Code_2) |= DUAL_LOOP_HOMING_INITIATE_SOFT_INDEX_MASK; // allow only SFB enter rellevant code
            }
         }
         else //linear load encoder
         {
            *(int *)FPGA_INPUT_CAPTURE_SELECT_REG_ADD = 13; // Trigger Source SFB Hardware Index 
            VAR(AX0_u16_Fpga_Capture_Select_Value) = 13;
         }
     }
   }
   else if (((BGVAR(u64_End_Target) >> 16) & 0xFF) == NEG_LIM_OFF)
   {
      if (u16_Trigger_Switch = IsInFunctionalityConfigured(CCW_LIMIT_SW_INP, drive))
      {
         VAR(AX0_s16_Crrnt_Run_Code) &= ~HOMING_SOFT_INDEX_MASK; // Prevent "Soft Index"...
         *(int *)FPGA_INPUT_CAPTURE_SELECT_REG_ADD = u16_Trigger_Switch; // Trigger Source Limit Switch
         u16_Mask = (1 << (int)(u16_Trigger_Switch - 1));
         *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_REG_ADD = (((BGVAR(u16_Dig_In_Polar) ^ VAR(AX0_u16_LX28_LS_Location)) & u16_Mask) == 0);
          // Falling Edge, matching to Input Invert Status.
      }
      else
      {
         BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;         // set home state to failure

         p402_error_code = ERRCODE_CAN_FIELDBUS_HOMING_NO_TRIGGER;
         BGVAR(u8_Homing_Not_Started) = 1;

         HomeFailureIndication(drive, 0, 0, 0, NO_TRIGGER_SOURCE_SET);      // set failure indication.
         return NO_TRIGGER_SOURCE_SET;
      }
   }
   else if (((BGVAR(u64_End_Target) >> 16) & 0xFF) == POS_LIM_OFF)
   {
      if (u16_Trigger_Switch = IsInFunctionalityConfigured(CW_LIMIT_SW_INP, drive))
      {
         VAR(AX0_s16_Crrnt_Run_Code) &= ~HOMING_SOFT_INDEX_MASK; // Prevent "Soft Index"...
         *(int *)FPGA_INPUT_CAPTURE_SELECT_REG_ADD = u16_Trigger_Switch; // Trigger Source Limit Switch
         u16_Mask = (1 << (int)(u16_Trigger_Switch - 1));
         *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_REG_ADD = (((BGVAR(u16_Dig_In_Polar) ^ VAR(AX0_u16_LX28_LS_Location)) & u16_Mask) == 0);
          // Falling Edge, matching to Input Invert Status.
      }
      else
      {
         BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;         // set home state to failure

         p402_error_code = ERRCODE_CAN_FIELDBUS_HOMING_NO_TRIGGER;
         BGVAR(u8_Homing_Not_Started) = 1;

         HomeFailureIndication(drive, 0, 0, 0, NO_TRIGGER_SOURCE_SET);   // set failure indication.
         return NO_TRIGGER_SOURCE_SET;
      }
   }
   else if (((BGVAR(u64_End_Target) >> 16) & 0xFF) == HOME_SWITCH_ON)
   {
      if (u16_Trigger_Switch = IsInFunctionalityConfigured(HOME_SWITCH_INP, drive))
      {
         VAR(AX0_s16_Crrnt_Run_Code) &= ~HOMING_SOFT_INDEX_MASK; // Prevent "Soft Index"...
         *(int *)FPGA_INPUT_CAPTURE_SELECT_REG_ADD = u16_Trigger_Switch; // Trigger Source Home Switch
         u16_Mask = (1 << (int)(u16_Trigger_Switch - 1));
         *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_REG_ADD = ((BGVAR(u16_Dig_In_Polar) & u16_Mask) == 0);
          // Falling Edge, matching to Input Invert Status.
      }
      else
      {
         BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;         // set home state to failure
         p402_error_code = ERRCODE_CAN_FIELDBUS_HOMING_NO_TRIGGER;
         BGVAR(u8_Homing_Not_Started) = 1;
         HomeFailureIndication(drive, 0, 0, 0, NO_TRIGGER_SOURCE_SET);   // set failure indication.
         return NO_TRIGGER_SOURCE_SET;
      }
   }
   else if (((BGVAR(u64_End_Target) >> 16) & 0xFF) == HOME_SWITCH_OFF)
   {
      if (u16_Trigger_Switch = IsInFunctionalityConfigured(HOME_SWITCH_INP, drive))
      {
         VAR(AX0_s16_Crrnt_Run_Code) &= ~HOMING_SOFT_INDEX_MASK; // Prevent "Soft Index"...
         *(int *)FPGA_INPUT_CAPTURE_SELECT_REG_ADD = u16_Trigger_Switch; // Trigger Source Home Switch

         u16_Mask = (1 << (int)(u16_Trigger_Switch - 1));
         *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_REG_ADD = ((BGVAR(u16_Dig_In_Polar) & u16_Mask) != 0);
          // Rising Edge, matching to Input Invert Status.
      }
      else
      {
         BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;         // set home state to failure
         p402_error_code = ERRCODE_CAN_FIELDBUS_HOMING_NO_TRIGGER;
         BGVAR(u8_Homing_Not_Started) = 1;
         HomeFailureIndication(drive, 0, 0, 0, NO_TRIGGER_SOURCE_SET);    // set failure indication.
         return NO_TRIGGER_SOURCE_SET;
      }
   }
   else // A different Homing-Trigger Source, select no Trigger-Capture at FPGA
   {
      *(int *)FPGA_INPUT_CAPTURE_SELECT_REG_ADD = 0; // Select no Trigger-Source
      *(int *)FPGA_CAPTURE_SIGNAL_POLARITY_REG_ADD = 0;
   }

   BGVAR(u16_MoveSmooth_Src) = VAR(AX0_u16_Move_Smooth_Source); //save old movesmoothsrc to retain when homing is done   
   if (4 == VAR(AX0_s16_Opmode))
   {
      //here we make sure ptp mode has the same movesmoothsrc as gear mode, since opmode is about to change
      u16_temp_movesmooth_src = (VAR(AX0_u16_Move_Smooth_Source) & 0xE); //clear bit 0 (movesmooth in ptp mode)
      u16_temp_movesmooth_src = (u16_temp_movesmooth_src | ((VAR(AX0_u16_Move_Smooth_Source) & 0x2) >> 1)); //save bit 1 (movesmooth in gear mode) to bit 0
      MoveSmoothSourceCommand((long long)u16_temp_movesmooth_src, drive);
   }
   BGVAR(u8_Homing_State) = HOMING_SETUP;
   BGVAR(u8_Homing_Not_Started) = 0;
   HomeFailureIndication(drive, 0, 0, 0, NO_ERROR);   // mark failure indication to zero (no error).
   return SAL_SUCCESS;
}


int ReadCurrentLevel(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ( (BGVAR(u32_EqCrrnt_Avg) > BGVAR(u32_Home_I_HardStop)) &&
        (BGVAR(s16_Motor_Moving) == 0)                          )
      return 1;
   else
      return 0;
}


//**********************************************************
// Function Name: AutoHomeModeCommand
// Description:
//          This function is called in response to the AUTOHOMEMODE command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalAutoHomeModeCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u8_Auto_Home_Mode) = (int)param;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: HomeAccCommand
// Description:
//          This function is called in response to the HOMEACC command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalHomeAccCommand(long long lparam, int drive)
{
   int ret_val = SAL_SUCCESS;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

    //value must be between 0.06 rpm/sec to 1,000,000 rpm/sec
    ret_val = CheckAccDecLimits(lparam);
    if (ret_val != SAL_SUCCESS)
    {
      return ret_val;
    }

   BGVAR(u64_HomeAccRate) = (long long)lparam;
   // This Function is called ONLY in configuration where separate HOMEACC and HOMEDEC
   // are not required.
   BGVAR(u64_HomeDecRate) = (long long)lparam;

   // update P7-00 (acc and dec for homing in lxm28)
   BGVAR(u64_Path_Acceleration)[0] = BGVAR(u64_HomeAccRate);
   BGVAR(u64_Path_Deceleration)[0] = BGVAR(u64_HomeDecRate);

   return ret_val;
}


//**********************************************************
// Function Name: SalHomeSpeed1Command
// Description:
//          This function is called in response to the HOMESPEED1 command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalHomeSpeed1Command(long long lparam, int drive)
{
   long long s64_min_value;
   // AXIS_OFF;
   
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // get min value according to pos control mode
   if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)
   {
      s64_min_value = HOME_VEL_1_RPM_PTP_UNITS_TSP_125;
   }
   else if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC)
   {
      s64_min_value = HOME_VEL_1_RPM_PTP_UNITS_TSP_250;
   }
   else
   {
      return INVALID_POSCONTROLMODE; 
   }
   
   if (lparam < s64_min_value) return VALUE_TOO_LOW; // Minimum: 1 RPM
   if (lparam > (long long)BGVAR(s32_Vmax)) return VALUE_TOO_HIGH;

   BGVAR(u32_Home_Switch_Speed) = (long)lparam;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalHomeSpeed2Command
// Description:
//          This function is called in response to the HOMESPEED2 command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalHomeSpeed2Command(long long lparam, int drive)
{
   long long s64_min_value;
   // AXIS_OFF;
   
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // get min value according to pos control mode
   if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)
   {
      s64_min_value = HOME_VEL_1_RPM_PTP_UNITS_TSP_125;
   }
   else if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC)
   {
      s64_min_value = HOME_VEL_1_RPM_PTP_UNITS_TSP_250;
   }
   else
   {
      return INVALID_POSCONTROLMODE; 
   }
   
   if (lparam < s64_min_value) return VALUE_TOO_LOW; // Minimum: 1 RPM
   if (lparam > (long long)BGVAR(s32_Vmax)) return VALUE_TOO_HIGH;

   BGVAR(u32_Home_Zero_Speed) = (long)lparam;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SearchHomeType
// Description:
//   This function is called to search for a specific Home-Type in the Home-Type Table.
//   Homing State-Machine Stage Descriptions will be initialized accordingly.
//
// Author: A. H.
// Algorithm:
// Return Value:
//   -1: Home-Type not found in Table
//   0 through NUMBER_OF_HOMETYPES:  Home-Type found and Valid
//   -128 and lower:  Home-Type reserved (CANopen).
// Revisions:
//**********************************************************
int SearchHomeType(int HomeType)
{
   int i;

   for (i = 0; i < NUMBER_OF_HOMETYPES; i++)
   {
      if (s_HomeTypes_Init_Table[i].Home_Type == HomeType)
      {
         if (s_HomeTypes_Init_Table[i].State_Machine == 0)
         // State_Machine parameter set to Zero means Reserved Type
            return (HomeType - 256);
         else
            return i;
      }
   }
   return -1;
}


//**********************************************************
// Function Name: SalHomeTypeCommand
// Description:
//          This function is called in response to the HOMETYPE command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalHomeTypeCommand(long long param, int drive)
{
   int Detected_Home_Type_Index;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (IsHomingProcessRunning(drive))
      return HOMING_IN_PROGRESS; // do not change Home Type when in the middle of the process

   BGVAR(u64_1st_Target) = 0LL;
   BGVAR(u64_2nd_Target) = 0LL;
   BGVAR(u64_3rd_Target) = 0LL;
   BGVAR(u64_4th_Target) = 0LL;
   BGVAR(u64_End_Target) = 0LL;

   Detected_Home_Type_Index = SearchHomeType((int)param);
   if (Detected_Home_Type_Index < -127)
      return RESERVED_HOMING_TYPE;
   else if (Detected_Home_Type_Index == -1)
      return INVALID_HOMING_TYPE;

   BGVAR(s16_Home_Type) = (int)param;
   // Target Value indicates the Move Direction (no shift), the Switch Transition to seek
   // ( << 8), the Homing Trigger Source ( << 16), the Failure Conditions ( << 24), and
   // additional Failure Conditions in case of SE Configuration ( << 32).
   BGVAR(u64_1st_Target) = (unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].First_Direction |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].First_Switch_Transition << 8)     |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].First_Trigger_Source << 16)       |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].First_Failure_Conditions << 24);
   BGVAR(u64_2nd_Target) = (unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Second_Direction |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Second_Switch_Transition << 8)     |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Second_Trigger_Source << 16)       |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Second_Failure_Conditions << 24);
   BGVAR(u64_3rd_Target) = (unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Third_Direction |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Third_Switch_Transition << 8)     |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Third_Trigger_Source << 16)       |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Third_Failure_Conditions << 24);
   BGVAR(u64_4th_Target) = (unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Fourth_Direction |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Fourth_Switch_Transition << 8)     |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Fourth_Trigger_Source << 16)       |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].Fourth_Failure_Conditions << 24);
   BGVAR(u64_End_Target) = (unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].End_Direction |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].End_Switch_Transition << 8)     |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].End_Trigger_Source << 16)       |
        ((unsigned long long)s_HomeTypes_Init_Table[Detected_Home_Type_Index].End_Failure_Conditions << 24);
   return SAL_SUCCESS;
}


void InitHomeParam(int drive)
{
   *(unsigned int *)FPGA_CAPTURE1_FILTER_REG_ADD = 60000;
   // Filtering of Trigger-Source set to 1mSec. in FPGA Clocks.

   if (VAR(AX0_s16_Opmode) == 4) // in case of GEAR mode automatically switch to position mode
   {
      BGVAR(s16_Prev_Home_Opmode) = VAR(AX0_s16_Opmode);
      VAR(AX0_s16_Opmode) = 8;
      SetOpmode(drive,VAR(AX0_s16_Opmode));
   }
}


void TerminateHome(int drive)
{
   // AXIS_OFF;
   int s16_temp_cntr;

   // cancel searching for index (in case it was not reset because of homing failure)
   VAR(AX0_s16_Crrnt_Run_Code) &= ~ARM_HOMING_CAPTURE_MASK;
   
   if (BGVAR(s16_Prev_Home_Opmode) != -1) // switch back to original mode
   {
      do {  // To avoid jump at the end of gearing
         s16_temp_cntr = Cntr_3125;
         LLVAR(AX0_u32_Gear_Not_Lim_Pos_Lo) = LLVAR(AX0_u32_Pos_Cmd_Ptp_Lo);
      } while (s16_temp_cntr != Cntr_3125); // To maintain 64bit read consistency

      SetOpmode(drive, BGVAR(s16_Prev_Home_Opmode));
      BGVAR(s16_Prev_Home_Opmode) = -1;
   }
}


//**********************************************************
// Function Name: SalWriteHomingTypeCommand
// Description:
//          This function is called in response to the P-Parameter P5-04 write command.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteHomingTypeCommand(long long param, int drive)
{
   unsigned int u16_temp_value = (unsigned int)param, u16_Cdhd_Home_Type;

   // check range for each byte
   if ( ((u16_temp_value & 0x000f) > 0x0008) || ((u16_temp_value & 0x00f0) > 0x0020) ||
        ((u16_temp_value & 0x0f00) > 0x0100) || ((u16_temp_value & 0xf000) > 0x0000)   )
      return VALUE_OUT_OF_RANGE;
   else
   {
      BGVAR(u16_P5_04_Homing_Mode) = u16_temp_value;

      u16_Cdhd_Home_Type = s_HomeMode_To_HomeType[(u16_temp_value & 0x000f)][((u16_temp_value & 0x00f0) >> 4)][((u16_temp_value & 0x0f00) >> 8)];

      return SalHomeTypeCommand((long long)u16_Cdhd_Home_Type, drive);
   }
}


int LimitSwitchIgnore(unsigned long HmProcessTarget)
{
   if ((((HmProcessTarget >> 8) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON)
      return 1; // Set bit 0 to indicate Neg. LS
   else if ((((HmProcessTarget >> 8) & 0xFF) & POS_LIM_ON) == POS_LIM_ON)
      return 2; // Set bit 1 to indicate Pos. LS
   else // No LS to be ignored...
      return 0;
}


void AutoHomeHandler(int drive)
{
   REFERENCE_TO_DRIVE;
   // Conditions for Performing Homing:  Drive is Enabled, Auto-Home-Mode is 1, and
   // 1st attempt after Power-Up (u16_Homing_Attempted_Once is initialized to 0).
   if ( Enabled(drive) && (BGVAR(u8_Auto_Home_Mode) == 1)                             &&
       (BGVAR(u16_Homing_Attempted_Once) == 0) && (IS_PHASEFIND_PROCEDURE_NOT_RUNNING)  )//TRY PUT ONLY FAIL & DONE!!
   {
      BGVAR(u16_Homing_Attempted_Once) = 1; // Mark Power-Up Event, allowing Homing Attempt
                                            // only on the 1st Enable after Power-Up...
      STORE_EXECUTION_PARAM_0;
      HomeCommand(drive);
      RESTORE_EXECUTION_PARAM_0;
   }
}


//**********************************************************
// Function Name: IsHomingProcessRunning
// Description:
//    This function returns a 0 in case that the homing procedure is currently
//    not busy and a 1 in case that the homing procedure is currently running.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int IsHomingProcessRunning(int drive)
{
   int s16_ret_val = 0; // Homing process is not running

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Check is the homing process is currently busy
   if ( (BGVAR(u8_Homing_State) != HOMING_IDLE) && (BGVAR(u8_Homing_State) != HOMING_FAILED) &&
        (BGVAR(u8_Homing_State) != HOMING_TARGET_REACHED)                                      )
   {
      s16_ret_val = 1; // Homing process is running
   }

   return s16_ret_val;
}


void HomingHandler(int drive)
{
   long long s64_Temp;
   unsigned long u32_HomeAccRounded, u32_HomeDecRounded, u32_start_vel, u32_speed;
   unsigned int u16_temp = 0, Detected_State_Machine, u16_local_cw_ls, u16_local_ccw_ls;
   static unsigned int u16_trans_bits_ls;
   int s16_sal_result = 0;

   // When setting Homing Limits Mode to ignore Transient (Phantom) Bits
   if ((VAR(AX0_u16_SW_Pos_lim) & 0x04) == 0x04)
   {  // Do not use the hardware limit-switch phantom bits
      u16_trans_bits_ls  = 0x02;
   }
   else
   {  // Involve the hardware limit-switch phantom bits
      u16_trans_bits_ls  = 0x06;
   }

   // get local copy of limit switches with the correct mask
   u16_local_ccw_ls = VAR(AX0_u16_CCW_LS) & u16_trans_bits_ls;
   u16_local_cw_ls = VAR(AX0_u16_CW_LS) & u16_trans_bits_ls;

   Detected_State_Machine = s_HomeTypes_Init_Table[SearchHomeType(BGVAR(s16_Home_Type))].State_Machine;
   // If the Hold/Resume input (INMODE x = 30) requires a switch to homing failure
   // in case that the homing process is currently running but paused due to a HOLD.
   if (BGVAR(u16_Switch_To_Homing_Failed) != 0)
   {
      BGVAR(u16_Switch_To_Homing_Failed) = 0;
      if (IsHomingProcessRunning(drive))
      {
         BGVAR(u32_Home_Fail_Ind) = ((long)Detected_State_Machine << 16) | ((long)BGVAR(u8_Homing_State) << 8) | u16_temp;
         BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
      }
   }

   if (Detected_State_Machine == 1)
   {
      switch (BGVAR(u8_Homing_State)) // State-Machine for "simple" Homing Sequences
      {
         case HOMING_IDLE:
            u16_temp = 0;
            VAR(AX0_u16_Home_Ind) = 0;
            BGVAR(u8_Homing_Ignore_LS) = 0; // LS in use Indicator...
         break;

         case HOMING_SETUP:
            VAR(AX0_u16_Home_Ind) = 0;
            InitHomeParam(drive);
            BGVAR(u8_Homing_State) = HOMING_INITIAL_STATE_DETECTION;
            LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev) = LLVAR(AX0_u32_Home_Delta_Offset_Lo);

            if (!VAR(AX0_u16_Abs_Fdbk_Device))// ABS Feedback - no need to home PCOM
            {  //PCOM support
               if (Pcom_1_Cntrl_Word.u16_en)
                  PcomDisable(1);
               if (Pcom_2_Cntrl_Word.u16_en)
                  PcomDisable(2);
            }
         break;

         case HOMING_INITIAL_STATE_DETECTION:
            BGVAR(u8_Homing_Ignore_LS) = LimitSwitchIgnore(BGVAR(u64_1st_Target));

            if ( (((BGVAR(u64_1st_Target) >> 8) & 0xFF) == 0)                     ||
                 ( (((BGVAR(u64_1st_Target) >> 8) & 0xFF) == NEG_LIM_ON) &&
                   ((VAR(AX0_u16_CCW_LS) & 2) != 0)                        )      ||
                 ( (((BGVAR(u64_1st_Target) >> 8) & 0xFF) == POS_LIM_ON) &&
                   ((VAR(AX0_u16_CW_LS) & 2) != 0)                         )      ||
                 ( (((BGVAR(u64_1st_Target) >> 8) & 0xFF) == HOME_SWITCH_ON)  &&
                   (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)          ) ||
                 ( (((BGVAR(u64_1st_Target) >> 8) & 0xFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)         )   )
               BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
            else if ( (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0)) )
              // Reaching this State with Drive Disabled is possible only with HOMETYPE 35,
            {
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }
               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
            else
               BGVAR(u8_Homing_State) = HOMING_INITIAL_DIR_MOVE;
         break;

         case HOMING_INITIAL_DIR_MOVE:
            // Ensuring that Motion starts with Low Velocity if Capture Detection may occur.
            u32_start_vel = BGVAR(u32_Home_Switch_Speed);
            if (((BGVAR(u64_1st_Target) >> 16) & 0xFF) != 0) // If Homing Type calls for Capturing
            {
               u32_start_vel = BGVAR(u32_Home_Zero_Speed);
            }

            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_1st_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, u32_start_vel, u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_1st_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, u32_start_vel, u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);

            // Set a delay to make sure that the Axis moves away from Index if already on Iindex...
            BGVAR(s32_Homing_Initial_Dir_Timestamp) = Cntr_1mS;
            BGVAR(u8_Homing_State) = HOMING_INITIAL_DIR_MOVE_DELAY;
         break;

         case HOMING_INITIAL_DIR_MOVE_DELAY:
            if (PassedTimeMS(20L, BGVAR(s32_Homing_Initial_Dir_Timestamp)))
            {  
               if (((BGVAR(u64_1st_Target) >> 16) & 0xFF) != 0) // If Homing Type calls for Capturing
               // on initial Move, arm Homing Capture at this point.
                  VAR(AX0_s16_Crrnt_Run_Code) |= ARM_HOMING_CAPTURE_MASK;
               BGVAR(u8_Homing_State) = HOMING_SEEK_1ST_SWITCH;
            }
         break;

         case HOMING_SEEK_1ST_SWITCH:
            BGVAR(u8_Homing_Ignore_LS) = LimitSwitchIgnore(BGVAR(u64_1st_Target));

            if ( ((((BGVAR(u64_1st_Target) >> 16) & 0xFF) & INDEX) == INDEX)   &&
                 ((VAR(AX0_s16_Crrnt_Run_Code) & ARM_HOMING_CAPTURE_MASK) == 0)  )
            { // If Capture on Trigger-Source occurred after it was previously set
               PtpAbort(drive);
               BGVAR(u8_Homing_State) = HOMING_SET_HOME_OFFSET;
            }
            else if ( ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                            &&
                        ( (IsHoldActive(drive) == 1)  )                                        ) ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                             &&
                        ( (IsHoldActive(drive) == 1)  )                                        ) ||
                      ( (((BGVAR(u64_1st_Target) >> 8) & 0xFF) == HOME_SWITCH_ON)  &&
                        (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)          )                           ||
                      ( (((BGVAR(u64_1st_Target) >> 8) & 0xFF) == HOME_SWITCH_OFF) &&
                         (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)         )                           ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON) &&
                        ReadCurrentLevel(drive)                                                  )                 )
            { // Conditions for Direction Reversal...
               VAR(AX0_s16_Crrnt_Run_Code) &= ~ARM_HOMING_CAPTURE_MASK; // Remove Capture Arming
               // to avoid Capture where not intended...
               if ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON)
                  BGVAR(u8_Homing_State) = HOMING_HARDSTOP_DECEL;
               else
                  BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
            }
            else if ( ( (((BGVAR(u64_1st_Target) >> 24) & 0xFF) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)         ) ||
                      ( (((BGVAR(u64_1st_Target) >> 24) & 0xFF) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)          ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))     )
              // Reaching this State with Drive Disabled is possible only with HOMETYPE 35,
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_HARDSTOP_DECEL:
//          Use higher Deceleration for this Stop to allow quicker response to Hardstop
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            BGVAR(s16_Move_Abs_Issued) = 0;
            InitiatePtpMove(drive, 0x0LL, 0x0L, u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            BGVAR(u8_Homing_State) = HOMING_HRDSTP_DECEL_DETECT;
         break;

         case HOMING_HRDSTP_DECEL_DETECT:
            if (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))
            {
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
            else if (VAR(AX0_s16_Stopped) >= 1)
               BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
         break;

         case HOMING_FLIP_1ST_SWITCH_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_End_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // Negative LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
  
               // fix BZ6091 based on IPR 1784: DS402: No smooth start (Acceleration) of axis when drive started homing mode.
               // if moving to find index with no Direction Reversal, load slow velocity
               if (((BGVAR(u64_1st_Target)>> 8) & 0xFF) == NO_TRIG)  // if no Direction Reversal, use slow speed
                  u32_speed = BGVAR(u32_Home_Zero_Speed);
               else
                  u32_speed = BGVAR(u32_Home_Switch_Speed);
               
               InitiatePtpMove(drive, 0xF000000000000000LL, u32_speed, u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_End_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // Positive LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
  
               // fix IPR BZ6091 based on 1784: DS402: No smooth start (Acceleration) of axis when drive started homing mode.
               // if moving to find index with no Direction Reversal, load slow velocity
               if (((BGVAR(u64_1st_Target)>> 8) & 0xFF) == NO_TRIG)  // if no Direction Reversal, use slow speed
                  u32_speed = BGVAR(u32_Home_Zero_Speed);
               else
                  u32_speed = BGVAR(u32_Home_Switch_Speed);
                  
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, u32_speed, u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else // Neither Pos. nor Neg. Direction means Home-Type 35 (Home-in-Place)
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_FLIP_1ST_SWITCH;
         break;

         case HOMING_SEEK_FLIP_1ST_SWITCH:
            if ( ( (BGVAR(u8_Homing_Ignore_LS) && 0x01)                   &&
                   (((BGVAR(u64_End_Target) >> 8) & 0xFF) == NEG_LIM_OFF) &&
                   ((u16_local_ccw_ls) == 0)                                ) ||
                 ( (BGVAR(u8_Homing_Ignore_LS) && 0x02)                   &&
                   (((BGVAR(u64_End_Target) >> 8) & 0xFF) == POS_LIM_OFF) &&
                   ((u16_local_cw_ls) == 0)                                 )   )
               BGVAR(u8_Homing_Ignore_LS) = 0; // Zeroing the LS Ignore Variable is conditioned on the process
               // identifying LS Falling Edge, to correctly manage Homing on LS Falling Edge.

            if ( (((BGVAR(u64_End_Target) >> 8) & 0xFF) == 0)                     ||
                 ( (((BGVAR(u64_End_Target) >> 8) & 0xFF) == NEG_LIM_OFF) &&
                   ((u16_local_ccw_ls) == 0)                                )     ||
                 ( (((BGVAR(u64_End_Target) >> 8) & 0xFF) == POS_LIM_OFF) &&
                   ((u16_local_cw_ls) == 0)                                 )     ||
                 ( (((BGVAR(u64_End_Target) >> 8) & 0xFF) == HOME_SWITCH_ON)  &&
                   (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)          ) ||
                 ( (((BGVAR(u64_End_Target) >> 8) & 0xFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)         )   )
               BGVAR(u8_Homing_State) = HOMING_TRIGGER_ARMING;
            else if ( ( (((BGVAR(u64_End_Target) >> 24) & 0xFF) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                ) ||
                      ( (((BGVAR(u64_End_Target) >> 24) & 0xFF) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                 ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))     )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35.
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_TRIGGER_ARMING:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_End_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_End_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else // Neither Pos. nor Neg. Direction means Home-Type 35 (Home-in-Place)
              PtpAbort(drive);
            VAR(AX0_s16_Crrnt_Run_Code) |= ARM_HOMING_CAPTURE_MASK; // ARM_HOMING_TRIGGER;
            BGVAR(u8_Homing_State) = HOMING_SEEK_TRIGGER;
         break;

         case HOMING_SEEK_TRIGGER:
            if (((BGVAR(u64_End_Target) >> 16) & 0xFF) == 0)
            // No Trigger Source means Homing at Present Position.
            {  // Remove Trigger Arming thus capturing Present Position.
               VAR(AX0_s16_Crrnt_Run_Code) &= ~ARM_HOMING_CAPTURE_MASK;
               LLVAR(AX0_u32_Prev_Captured_Pfb_Lo) = LLVAR(AX0_u32_Captured_Pfb_Lo) = LLVAR(AX0_u32_Temp_Captured_Pfb_Lo);
            }
            else if ( (((BGVAR(u64_End_Target) >> 16) & 0xFF) == HARDSTOP_ON) && ReadCurrentLevel(drive) )
            // Infrastructure for Homing on Hardstop at HOMESPEED2...
            { // Stop, remove Trigger Arming to capture Present Position
               //          Use higher Deceleration for this Stop to allow quicker response to Hardstop
               u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
               u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x0LL, 0x0L, u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
               VAR(AX0_s16_Crrnt_Run_Code) &= ~ARM_HOMING_CAPTURE_MASK;
               LLVAR(AX0_u32_Prev_Captured_Pfb_Lo) = LLVAR(AX0_u32_Captured_Pfb_Lo) = LLVAR(AX0_u32_Temp_Captured_Pfb_Lo);
            }

            if ((VAR(AX0_s16_Crrnt_Run_Code) & ARM_HOMING_CAPTURE_MASK) == 0)
            {
               PtpAbort(drive);
               BGVAR(u8_Homing_State) = HOMING_SET_HOME_OFFSET;
            }
            else if ( ( (((BGVAR(u64_End_Target) >> 24) & 0xFF) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                ) ||
                      ( (((BGVAR(u64_End_Target) >> 24) & 0xFF) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                 ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0)      )  )
              // Reaching this State with Drive Disabled is possible only with HOMETYPE 35,
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SET_HOME_OFFSET:
            if ( (VAR(AX0_s16_Stopped) >= 1) || (((BGVAR(u64_End_Target) >> 16) & 0xFF) == 0) )
            {  // Set Home_Offset per s64_Home_Offset, Move to Zero Position if required,
               s64_Temp = LLVAR(AX0_u32_Prev_Captured_Pfb_Lo) - LLVAR(AX0_u32_Captured_Pfb_Lo);
               s64_Temp = BGVAR(s64_Home_Offset) + (s64_Temp * VAR(AX0_u16_Home_Capture_Time) / 1875) - LLVAR(AX0_u32_Prev_Captured_Pfb_Lo);
               LLVAR(AX0_u32_Home_Offset_Lo_Temp) = s64_Temp;
               s64_Temp -= LLVAR(AX0_u32_Home_Offset_Lo) - LLVAR(AX0_u32_Home_Delta_Offset_Lo);
               LLVAR(AX0_u32_Home_Delta_Offset_Lo_Temp) = s64_Temp;
               VAR(AX0_s16_Crrnt_Run_Code) |= HOMING_OFFSET_READY_MASK;
               BGVAR(u8_Homing_State) = HOMING_MOVE_ZERO_POS;
            }
            else if ( ( (VAR(AX0_s16_Stopped) == -1) || (!Enabled(drive)) ) &&
                      ((BGVAR(u64_End_Target) & 0xFF) != 0)                   )
            { // If Drive is Disabled, or Stopped Indication is incorrect, declare Failure
               PtpAbort(drive);
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }
               
               if (VAR(AX0_s16_Stopped) == -1)
               {
                  u16_temp |= MOTION_INTERRUPTED_BIT;
                  s16_sal_result = MOTION_INTERRUPTED;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_MOVE_ZERO_POS:
            if ((VAR(AX0_s16_Crrnt_Run_Code) & HOMING_OFFSET_READY_MASK) == 0)
            {
               u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
               u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
               do {
                  u16_temp = Cntr_3125;
                  BGVAR(s64_Temp_Pos_LS) = -LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
               } while (u16_temp != Cntr_3125);
               // Initialize LS Hold Target Position to provide for Limit Switch Stop
               if (((BGVAR(u64_End_Target) >> 16) & 0xFF) != 0)
               // No Trigger-Source means Homing-in-Place, avoid moving to Home-Offset.
               {
                  BGVAR(s16_Move_Abs_Issued) = 0;
                  if (BGVAR(u16_Home_Ofst_Move) == 1) // Move to Home-Offset, PFB will be 0.
                     InitiatePtpMove(drive, 0x0LL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
                  else // Move to Trigger-Source Position, PFB will be Home-Offset
                     InitiatePtpMove(drive, BGVAR(s64_Home_Offset), BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
               }
               BGVAR(u8_Homing_State) = HOMING_ATTAINED;
            }
         break;

         case HOMING_ATTAINED:
            if ( (Enabled(drive) && (LVAR(AX0_s32_Pos_Vcmd) == 0L)) ||
                 (((BGVAR(u64_End_Target) >> 16) & 0xFF) == 0)        )
               // Verify arrival at Target Position, or ignore if Homing-in-Place...
               BGVAR(u16_homing_pcmd_settled)++;
            else
               BGVAR(u16_homing_pcmd_settled) = 0;

            if (BGVAR(u16_homing_pcmd_settled) > 2)
            {
               BGVAR(u8_Homing_State) = HOMING_SUCCESS_TERMINATE_HOME;
               VAR(AX0_u16_Home_Ind) = 1;
            }

            if ( ((u16_local_ccw_ls) != 0) || ((u16_local_cw_ls) != 0)     ||
                 (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))  )
            { // Limits engaged, or Disabled and not Home-in-Place
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SUCCESS_TERMINATE_HOME:
            BGVAR(u8_Homing_Ignore_LS) = 0;
            TerminateHome(drive);
            MoveSmoothSourceCommand((long long)BGVAR(u16_MoveSmooth_Src), drive);
            BGVAR(u8_Homing_State) = HOMING_TARGET_REACHED;

            //  Re-Initialize Modulo State Machine bit mask
            //  The right most bit of this variable indicates if the modulo is active
            //  If the modulo is active then bitwise AND with 1 will Restart the Modulo State Machine (clearing all the other bits in the bit mask)
            //  Else the Modulo is not active hence bitwise AND with 1 wont change anything
            VAR(AX0_u16_Position_Modulo_Active) &= 1;

         break;

         case HOMING_TARGET_REACHED:
            if ( ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0)    ||
                 ((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0)  )
               BGVAR(u8_Homing_State) = HOMING_IDLE;   // Feedback Loss invalidates "Homed" Indication

            LLVAR(AX0_u32_Home_Offset_User_Lo) = LLVAR(AX0_u32_Home_Offset_Lo);   // Set homing offset user variable
         break;

         case HOMING_FAILURE_TERMINATE_HOME:
            BGVAR(u8_Homing_Ignore_LS) = 0;
            TerminateHome(drive);
            MoveSmoothSourceCommand((long long)BGVAR(u16_MoveSmooth_Src), drive);
            BGVAR(u8_Homing_State) = HOMING_FAILED;

            UpdateRefOffsetValue(drive);            // Restore homing offset operational variable with user variable
         break;

         case HOMING_FAILED:
            LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev) = 0;
            // TBD
         break;

         default:
            BGVAR(u8_Homing_State) = HOMING_IDLE;
         break;
      }
   } // (Detected_State_Machine == 1)

   if (Detected_State_Machine == 2)
   {
      switch (BGVAR(u8_Homing_State)) // State-Machine for "Complex" Homing Sequences
      {
         case HOMING_IDLE:
            u16_temp = 0;
            VAR(AX0_u16_Home_Ind) = 0;
            BGVAR(u8_Homing_Ignore_LS) = 0; // LS in use Indicator...
            LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev) = 0;
         break;

         case HOMING_SETUP:
            VAR(AX0_u16_Home_Ind) = 0;
            InitHomeParam(drive);
            BGVAR(u8_Homing_State) = HOMING_INITIAL_STATE_DETECTION;
            LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev) = LLVAR(AX0_u32_Home_Delta_Offset_Lo);

            if (!VAR(AX0_u16_Abs_Fdbk_Device))// ABS Feedback - no need to home PCOM
            {  //PCOM support
               if (Pcom_1_Cntrl_Word.u16_en)
                  PcomDisable(1);
               if (Pcom_2_Cntrl_Word.u16_en)
                  PcomDisable(2);
            }
         break;

         case HOMING_INITIAL_STATE_DETECTION:
            BGVAR(u8_Homing_Ignore_LS) = LimitSwitchIgnore(BGVAR(u64_1st_Target));

            if ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                              )
               BGVAR(u8_Homing_State) = HOMING_OFF_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                              ) ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                               )   )
               BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
            else if ( !Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0) )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35,
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
            else
               BGVAR(u8_Homing_State) = HOMING_INITIAL_DIR_MOVE;
         break;

         case HOMING_INITIAL_DIR_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_1st_Target) & 0xFF) == NEG_DIR)     // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_1st_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_1ST_SWITCH;
         break;

         case HOMING_SEEK_1ST_SWITCH:
            BGVAR(u8_Homing_Ignore_LS) = LimitSwitchIgnore(BGVAR(u64_1st_Target));

            if ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                              )
               BGVAR(u8_Homing_State) = HOMING_OFF_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                            &&
                        ( (IsHoldActive(drive) == 1)  )                                        ) ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                             &&
                        ( (IsHoldActive(drive) == 1)  )                                        ) ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON) &&
                        ReadCurrentLevel(drive)                                                  )                 )
            {
               if ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON)
                  BGVAR(u8_Homing_State) = HOMING_HARDSTOP_DECEL;
               else
                  BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
            }
            else if ( ( (((BGVAR(u64_1st_Target) >> 24) & 0xFF) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                ) ||
                      ( (((BGVAR(u64_1st_Target) >> 24) & 0xFF) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                 ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))     )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not with this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_HARDSTOP_DECEL:
//          Use higher Deceleration for this Stop to allow quicker response to Hardstop
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 24);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            BGVAR(s16_Move_Abs_Issued) = 0;
            InitiatePtpMove(drive, 0x0LL, 0x0L, u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            BGVAR(u8_Homing_State) = HOMING_HRDSTP_DECEL_DETECT;
         break;

         case HOMING_HRDSTP_DECEL_DETECT:
            if (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))
            {
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
            else if (VAR(AX0_s16_Stopped) >= 1)
               BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
         break;

         case HOMING_FLIP_1ST_SWITCH_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_2nd_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_2nd_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_FLIP_1ST_SWITCH;
         break;

         case HOMING_SEEK_FLIP_1ST_SWITCH:
            if ( ( (BGVAR(u8_Homing_Ignore_LS) && 0x01)                   &&
                   (((BGVAR(u64_2nd_Target) >> 8) & 0xFF) == NEG_LIM_OFF) &&
                   ((u16_local_ccw_ls) == 0)                                ) ||
                 ( (BGVAR(u8_Homing_Ignore_LS) && 0x02)                   &&
                   (((BGVAR(u64_2nd_Target) >> 8) & 0xFF) == POS_LIM_OFF) &&
                   ((u16_local_cw_ls) == 0)                                 )   )
               BGVAR(u8_Homing_Ignore_LS) = 0; // Zeroing the LS Ignore Variable is conditioned on the process
               // identifying LS Falling Edge, to correctly manage Homing on LS Falling Edge.

            if ( ( ((((BGVAR(u64_2nd_Target) >> 8) & 0xFF) & NEG_LIM_OFF) == NEG_LIM_OFF) && ((u16_local_ccw_ls) == 0) ) ||
                 ( ((((BGVAR(u64_2nd_Target) >> 8) & 0xFF) & POS_LIM_OFF) == POS_LIM_OFF) && ((u16_local_cw_ls) == 0) )    )
               BGVAR(u8_Homing_State) = HOMING_SEEK_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               )           ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                )           ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                        (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                               ) ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                        (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                               ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                              )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                             )
               {
                  u16_temp |= HOME_SWITCH_ON_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               if ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                               )
               {
                  u16_temp |= HOME_SWITCH_OFF_BIT;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SEEK_HOME_SWITCH_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_3rd_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_3rd_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_HOME_SWITCH;
         break;

         case HOMING_SEEK_HOME_SWITCH:
            if ( ((((BGVAR(u64_3rd_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )
               BGVAR(u8_Homing_State) = HOMING_OFF_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_3rd_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               ) ||
                      ( ((((BGVAR(u64_3rd_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                    )
              // Reaching this State with Drive Disabled is possible only with HOMETYPE 35,
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_OFF_HOME_SWITCH_MOVE:
            BGVAR(u8_Homing_Ignore_LS) = 0;

            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_End_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_End_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_OFF_HOME_SWITCH;
         break;

         case HOMING_SEEK_OFF_HOME_SWITCH:
            BGVAR(u8_Homing_Ignore_LS) = 0;

            if ( (((BGVAR(u64_End_Target) >> 8) & 0xFF) == NO_TRIG)                                ||
                 // No Switch Transition means the next Transition is a Homing Trigger
               ( ((((BGVAR(u64_End_Target) >> 8) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                  (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                             )  )
               BGVAR(u8_Homing_State) = HOMING_TRIGGER_ARMING;
            else if ( ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               ) ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                    )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this Sstate-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_TRIGGER_ARMING:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_End_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_End_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            VAR(AX0_s16_Crrnt_Run_Code) |= ARM_HOMING_CAPTURE_MASK; // ARM_HOMING_TRIGGER;
            BGVAR(u8_Homing_State) = HOMING_SEEK_TRIGGER;
         break;

         case HOMING_SEEK_TRIGGER:
            if ((VAR(AX0_s16_Crrnt_Run_Code) & ARM_HOMING_CAPTURE_MASK) == 0)
            {
               PtpAbort(drive);
               BGVAR(u8_Homing_State) = HOMING_SET_HOME_OFFSET;
            }
            else if ( ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               ) ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                    )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not with this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SET_HOME_OFFSET:
            if (VAR(AX0_s16_Stopped) >= 1)
            {  // Set Home_Offset per s64_Home_Offset, Move to Zero Position if required,
               s64_Temp = LLVAR(AX0_u32_Prev_Captured_Pfb_Lo) - LLVAR(AX0_u32_Captured_Pfb_Lo);
               s64_Temp = BGVAR(s64_Home_Offset) + (s64_Temp * VAR(AX0_u16_Home_Capture_Time) / 1875) - LLVAR(AX0_u32_Prev_Captured_Pfb_Lo);
               LLVAR(AX0_u32_Home_Offset_Lo_Temp) = s64_Temp;
               s64_Temp -= LLVAR(AX0_u32_Home_Offset_Lo) - LLVAR(AX0_u32_Home_Delta_Offset_Lo);
               LLVAR(AX0_u32_Home_Delta_Offset_Lo_Temp) = s64_Temp;
               VAR(AX0_s16_Crrnt_Run_Code) |= HOMING_OFFSET_READY_MASK;
               BGVAR(u8_Homing_State) = HOMING_MOVE_ZERO_POS; // 18
            }
            else if ( ( (VAR(AX0_s16_Stopped) == -1) || (!Enabled(drive)) ) &&
                      ((BGVAR(u64_End_Target) & 0xFF) != 0)                   )
            { // If Drive is Disabled, or Stopped Indication is incorrect, declare Failure
               PtpAbort(drive);
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               if (VAR(AX0_s16_Stopped) == -1)
               {
                  u16_temp |= MOTION_INTERRUPTED_BIT;
                  s16_sal_result = MOTION_INTERRUPTED;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_MOVE_ZERO_POS: // 18
            if ((VAR(AX0_s16_Crrnt_Run_Code) & HOMING_OFFSET_READY_MASK) == 0)
            {
               u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
               u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
               do {
                  u16_temp = Cntr_3125;
                  BGVAR(s64_Temp_Pos_LS) = -LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
               } while (u16_temp != Cntr_3125);
               BGVAR(s16_Move_Abs_Issued) = 0;
               if (((BGVAR(u64_End_Target) >> 16) & 0xFF) != 0) // Move only if a Trigger-Source exists (not HOMETYPE 35).
               {
                  if (BGVAR(u16_Home_Ofst_Move) == 1) // Move to Home-Offset, PFB will be 0.
                     InitiatePtpMove(drive, 0x0LL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
                  else // Move to Trigger-Source Position, PFB will be Home-Offset
                     InitiatePtpMove(drive, BGVAR(s64_Home_Offset), BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
               }
               BGVAR(u8_Homing_State) = HOMING_ATTAINED;
               VAR(AX0_u16_Home_Ind) = 1;
            }
         break;

         case HOMING_ATTAINED:
            if ( (Enabled(drive) && (LVAR(AX0_s32_Pos_Vcmd) == 0L)) ||
                 (((BGVAR(u64_End_Target) >> 16) & 0xFF) == 0)        )
               // Verify arrival at Target Position, or ignore if Homing-in-Place...
               BGVAR(u16_homing_pcmd_settled)++;
            else
               BGVAR(u16_homing_pcmd_settled) = 0;

            if (BGVAR(u16_homing_pcmd_settled) > 2)
               BGVAR(u8_Homing_State) = HOMING_SUCCESS_TERMINATE_HOME;

            if ( ((u16_local_ccw_ls) != 0) || ((u16_local_cw_ls) != 0)     ||
                 (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))  )
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SUCCESS_TERMINATE_HOME:
            BGVAR(u8_Homing_Ignore_LS) = 0;
            TerminateHome(drive);
            MoveSmoothSourceCommand((long long)BGVAR(u16_MoveSmooth_Src), drive);
            BGVAR(u8_Homing_State) = HOMING_TARGET_REACHED;

            //  Re-Initialize Modulo State Machine bit mask
            //  The right most bit of this variable indicates if the modulo is active
            //  If the modulo is active then bitwise AND with 1 will Restart the Modulo State Machine (clearing all the other bits in the bit mask)
            //  Else the Modulo is not active hence bitwise AND with 1 wont change anything
            VAR(AX0_u16_Position_Modulo_Active) &= 1;

            if (!VAR(AX0_u16_Abs_Fdbk_Device))            //  ABS Feedback - no need to home PCOM
            {  //PCOM support
               if (Pcom_1_Cntrl_Word.u16_en)
                  PcomInit(1);
               if (Pcom_2_Cntrl_Word.u16_en)
                  PcomInit(2);
            }
         break;

         case HOMING_TARGET_REACHED:
            if ( ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0)    ||
                 ((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0)  )
                   // Lost Feedback invalidates "Homed" Indication
               BGVAR(u8_Homing_State) = HOMING_IDLE;

            // Set homing offset user variable
            LLVAR(AX0_u32_Home_Offset_User_Lo) = LLVAR(AX0_u32_Home_Offset_Lo);
         break;

         case HOMING_FAILURE_TERMINATE_HOME:
            BGVAR(u8_Homing_Ignore_LS) = 0;
            TerminateHome(drive);
            MoveSmoothSourceCommand((long long)BGVAR(u16_MoveSmooth_Src), drive);
            BGVAR(u8_Homing_State) = HOMING_FAILED;

            UpdateRefOffsetValue(drive);            // Restore homing offset operational variable with user variable
         break;

         case HOMING_FAILED:
            LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev) = 0;
            // TBD
         break;

         default:
            BGVAR(u8_Homing_State) = HOMING_IDLE;
         break;
      }
   } // else if (Detected_State_Machine == 2)

   if (Detected_State_Machine == 3)
   {
      switch (BGVAR(u8_Homing_State)) // State-Machine for "Complex" Homing Sequences
      {
         case HOMING_IDLE:
            u16_temp = 0;
            VAR(AX0_u16_Home_Ind) = 0;
            BGVAR(u8_Homing_Ignore_LS) = 0; // LS in use Indicator...
         break;

         case HOMING_SETUP: // 1
            VAR(AX0_u16_Home_Ind) = 0;
            InitHomeParam(drive);
            BGVAR(u8_Homing_State) = HOMING_INITIAL_STATE_DETECTION;
            LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev) = LLVAR(AX0_u32_Home_Delta_Offset_Lo);

            if (!VAR(AX0_u16_Abs_Fdbk_Device))// ABS Feedback - no need to home PCOM
            {  //PCOM support
               if (Pcom_1_Cntrl_Word.u16_en)
                  PcomDisable(1);
               if (Pcom_2_Cntrl_Word.u16_en)
                  PcomDisable(2);
            }
         break;

         case HOMING_INITIAL_STATE_DETECTION:
            BGVAR(u8_Homing_Ignore_LS) = LimitSwitchIgnore(BGVAR(u64_1st_Target));

            if ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )
               BGVAR(u8_Homing_State) = HOMING_OFF_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                              ) ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                               )   )
               BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
            else if (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0) )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
            else
               BGVAR(u8_Homing_State) = HOMING_INITIAL_DIR_MOVE;
         break;

         case HOMING_INITIAL_DIR_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_1st_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_1st_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_1ST_SWITCH;
            if ( ((((BGVAR(u64_1st_Target) >> 16) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) ||
                  ((((BGVAR(u64_1st_Target) >> 16) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON)        )
               // Arm Homing Trigger if expect to engage Homing-Trigger on 1st Move, such as
               VAR(AX0_s16_Crrnt_Run_Code) |= ARM_HOMING_CAPTURE_MASK; // Home-Switch or Hardstop
         break;

         case HOMING_SEEK_1ST_SWITCH:
            BGVAR(u8_Homing_Ignore_LS) = LimitSwitchIgnore(BGVAR(u64_1st_Target));

            if ( ((((BGVAR(u64_1st_Target) >> 16) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                             )
            { // If Trigger-Source is Home-Switch, respond immediately even though this is 1st Move
               PtpAbort(drive);
               BGVAR(u8_Homing_State) = HOMING_SET_HOME_OFFSET;
               break; // break after sensing Homing-Switch On-Transition to avoid branching incorrectly.
            }

            if ( ((((BGVAR(u64_1st_Target) >> 16) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON) &&
                 ReadCurrentLevel(drive)                                                   )
            { // If Trigger-Source is Hardstop, respond immediately even though this is 1st Move
               //          Use higher Deceleration for this Stop to allow quicker response to Hardstop
               u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
               u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x0LL, 0x0L, u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
               VAR(AX0_s16_Crrnt_Run_Code) &= ~ARM_HOMING_CAPTURE_MASK;
               LLVAR(AX0_u32_Prev_Captured_Pfb_Lo) = LLVAR(AX0_u32_Captured_Pfb_Lo) = LLVAR(AX0_u32_Temp_Captured_Pfb_Lo);
               BGVAR(u8_Homing_State) = HOMING_SET_HOME_OFFSET;
               break; // break after sensing Hardstop On-Transition to avoid branching incorrectly.
            }

            if ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )
            {
               if (((BGVAR(u64_1st_Target) >> 16) & 0xFF) != 0) // Indicating Immediate Approach to
                                                              // Index after Home Transition
                  BGVAR(u8_Homing_State) = HOMING_SEEK_TRIGGER_CONDITIONS; // 14
               else // Zero at Bits 8 - 11 means retract from Home Switch and approach again for Trigger
                  BGVAR(u8_Homing_State) = HOMING_OFF_HOME_SWITCH_MOVE;
            }
            else if ( ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                            &&
                        ( (IsHoldActive(drive) == 1)  )                                        )  ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                             &&
                        ( (IsHoldActive(drive) == 1)  )                                        )  ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON) &&
                        ReadCurrentLevel(drive)                                                  )  )
            {
               if ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON)
                  BGVAR(u8_Homing_State) = HOMING_HARDSTOP_DECEL;
               else
                  BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
            }
            else if (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)
               BGVAR(u8_Homing_State) = HOMING_TRIGGER_ARMING;
            else if ( ( (((BGVAR(u64_1st_Target) >> 24) & 0xFF) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                ) ||
                      ( (((BGVAR(u64_1st_Target) >> 24) & 0xFF) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                 ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))     )
              // Reaching this State with Drive Disabled is possible only with HOMETYPE 35,
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_HARDSTOP_DECEL:
//          Use higher Deceleration for this Stop to allow quicker response to Hardstop
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 24);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            BGVAR(s16_Move_Abs_Issued) = 0;
            InitiatePtpMove(drive, 0x0LL, 0x0L, u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            BGVAR(u8_Homing_State) = HOMING_HRDSTP_DECEL_DETECT;
         break;

         case HOMING_HRDSTP_DECEL_DETECT:
            if (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))
            {
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
            else if (VAR(AX0_s16_Stopped) >= 1)
               BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
         break;

         case HOMING_FLIP_1ST_SWITCH_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_2nd_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_2nd_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_FLIP_1ST_SWITCH;
         break;

         case HOMING_SEEK_FLIP_1ST_SWITCH:
            if ( ( (BGVAR(u8_Homing_Ignore_LS) && 0x01)                   &&
                   (((BGVAR(u64_2nd_Target) >> 8) & 0xFF) == NEG_LIM_OFF) &&
                   ((u16_local_ccw_ls) == 0)                                ) ||
                 ( (BGVAR(u8_Homing_Ignore_LS) && 0x02)                   &&
                   (((BGVAR(u64_2nd_Target) >> 8) & 0xFF) == POS_LIM_OFF) &&
                   ((u16_local_cw_ls) == 0)                                 )   )
               BGVAR(u8_Homing_Ignore_LS) = 0; // Zeroing the LS Ignore Variable is conditioned on the process
               // identifying LS Falling Edge, to correctly manage Homing on LS Falling Edge.

            if ( ( ((((BGVAR(u64_2nd_Target) >> 8) & 0xFF) & NEG_LIM_OFF) == NEG_LIM_OFF) && ((u16_local_ccw_ls) == 0) ) ||
                 ( ((((BGVAR(u64_2nd_Target) >> 8) & 0xFF) & POS_LIM_OFF) == POS_LIM_OFF) && ((u16_local_cw_ls) == 0) )    )
               BGVAR(u8_Homing_State) = HOMING_SEEK_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               )           ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                )           ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                        (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                             )   ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                        (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                               ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                              )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                               )
               {
                  u16_temp |= HOME_SWITCH_ON_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               if ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                               )
               {
                  u16_temp |= HOME_SWITCH_OFF_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SEEK_HOME_SWITCH_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_3rd_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_3rd_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_HOME_SWITCH;
         break;

         case HOMING_SEEK_HOME_SWITCH:
            if ( ((((BGVAR(u64_3rd_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )
               BGVAR(u8_Homing_State) = HOMING_OFF_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_3rd_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               ) ||
                      ( ((((BGVAR(u64_3rd_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                    )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_OFF_HOME_SWITCH_MOVE:
            BGVAR(u8_Homing_Ignore_LS) = 0;

            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_4th_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_4th_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_OFF_HOME_SWITCH;
         break;

         case HOMING_SEEK_OFF_HOME_SWITCH:
            if ( ((((BGVAR(u64_4th_Target) >> 8) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                  (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                             )
               BGVAR(u8_Homing_State) = HOMING_ON_HOME_SWITCH_MOVE; // 13
            else if ( ( ((((BGVAR(u64_4th_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               ) ||
                      ( ((((BGVAR(u64_4th_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                    )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_ON_HOME_SWITCH_MOVE: // 13
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_End_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_End_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_TRIGGER_CONDITIONS; // 14
         break;

         case HOMING_SEEK_TRIGGER_CONDITIONS:
            BGVAR(u8_Homing_Ignore_LS) = 0;

            if ( (((BGVAR(u64_End_Target) >> 8) & 0xFF) == NO_TRIG)                                ||
                 // No Switch Transition means the next transition is a Homing Trigger
                 ( ((((BGVAR(u64_End_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                   (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )  )
               BGVAR(u8_Homing_State) = HOMING_TRIGGER_ARMING;
            else if ( ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               ) ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                    )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }
               
               if ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                               )
               {
                  u16_temp |= HOME_SWITCH_ON_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }
               
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }
               
               if ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                               )
               {
                  u16_temp |= HOME_SWITCH_OFF_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_TRIGGER_ARMING:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_End_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_End_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            VAR(AX0_s16_Crrnt_Run_Code) |= ARM_HOMING_CAPTURE_MASK;   // ARM_HOMING_TRIGGER;
            BGVAR(u8_Homing_State) = HOMING_SEEK_TRIGGER;
         break;

         case HOMING_SEEK_TRIGGER:
            if ((VAR(AX0_s16_Crrnt_Run_Code) & ARM_HOMING_CAPTURE_MASK) == 0)
            {
               PtpAbort(drive);
               BGVAR(u8_Homing_State) = HOMING_SET_HOME_OFFSET;
            }
            else if ( ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               )           ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                )           ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                        (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                             )   ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                         (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                              ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                              )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }
               
               if ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                               )
               {
                  u16_temp |= HOME_SWITCH_ON_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }
               
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }
               
               if ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                               )
               {
                  u16_temp |= HOME_SWITCH_OFF_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }
               
               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SET_HOME_OFFSET:
            BGVAR(u8_Homing_Ignore_LS) = 0;

            if (VAR(AX0_s16_Stopped) >= 1)
            {  // Set Home_Offset per s64_Home_Offset, Move to Zero Position if required,
               s64_Temp = LLVAR(AX0_u32_Prev_Captured_Pfb_Lo) - LLVAR(AX0_u32_Captured_Pfb_Lo);
               s64_Temp = BGVAR(s64_Home_Offset) + (s64_Temp * VAR(AX0_u16_Home_Capture_Time) / 1875) - LLVAR(AX0_u32_Prev_Captured_Pfb_Lo);
               LLVAR(AX0_u32_Home_Offset_Lo_Temp) = s64_Temp;
               s64_Temp -= LLVAR(AX0_u32_Home_Offset_Lo) - LLVAR(AX0_u32_Home_Delta_Offset_Lo);
               LLVAR(AX0_u32_Home_Delta_Offset_Lo_Temp) = s64_Temp;
               VAR(AX0_s16_Crrnt_Run_Code) |= HOMING_OFFSET_READY_MASK;
               BGVAR(u8_Homing_State) = HOMING_MOVE_ZERO_POS; // 18
            }
            else if ( ( (VAR(AX0_s16_Stopped) == -1) || (!Enabled(drive)) ) &&
                      ((BGVAR(u64_End_Target) & 0xFF) != 0)                   )
            { // If Drive is Disabled, or Stopped Indication is incorrect, declare Failure
               PtpAbort(drive);
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               if (VAR(AX0_s16_Stopped) == -1)
               {
                  u16_temp |= MOTION_INTERRUPTED_BIT;
                  s16_sal_result = MOTION_INTERRUPTED;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_MOVE_ZERO_POS: // 18
            if ((VAR(AX0_s16_Crrnt_Run_Code) & HOMING_OFFSET_READY_MASK) == 0)
            {
               u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
               u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
               do {
                  u16_temp = Cntr_3125;
                  BGVAR(s64_Temp_Pos_LS) = -LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
               } while (u16_temp != Cntr_3125);
               // Initialize LS Hold Target Position to provide for Limit Switch Stop
               BGVAR(s16_Move_Abs_Issued) = 0;
               if (((BGVAR(u64_End_Target) >> 16) & 0xFF) != 0)
                // No Trigger-Source means Homing-in-Place, avoid moving to Home-Offset.
               {
                  if (BGVAR(u16_Home_Ofst_Move) == 1) // Move to Home-Offset, PFB will be 0.
                     InitiatePtpMove(drive, 0x0LL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
                  else // Move to Trigger-Source Position, PFB will be Home-Offset
                     InitiatePtpMove(drive, BGVAR(s64_Home_Offset), BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
               }
               BGVAR(u8_Homing_State) = HOMING_ATTAINED;
               VAR(AX0_u16_Home_Ind) = 1;
            }
         break;

         case HOMING_ATTAINED:
            if ( (Enabled(drive) && (LVAR(AX0_s32_Pos_Vcmd) == 0L)) ||
                 (((BGVAR(u64_End_Target) >> 16) & 0xFF) == 0)        )
               // Verify arrival at Target Position, or ignore if Homing-in-Place...
               BGVAR(u16_homing_pcmd_settled)++;
            else
               BGVAR(u16_homing_pcmd_settled) = 0;

            if (BGVAR(u16_homing_pcmd_settled) > 2)
               BGVAR(u8_Homing_State) = HOMING_SUCCESS_TERMINATE_HOME;

            if ( ((u16_local_ccw_ls) != 0) || ((u16_local_cw_ls) != 0)     ||
                 (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))  )
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SUCCESS_TERMINATE_HOME:
            BGVAR(u8_Homing_Ignore_LS) = 0;
            TerminateHome(drive);
            MoveSmoothSourceCommand((long long)BGVAR(u16_MoveSmooth_Src), drive);
            BGVAR(u8_Homing_State) = HOMING_TARGET_REACHED;

            //  Re-Initialize Modulo State Machine bit mask
            //  The right most bit of this variable indicates if the modulo is active
            //  If the modulo is active then bitwise AND with 1 will Restart the Modulo State Machine (clearing all the other bits in the bit mask)
            //  Else the Modulo is not active hence bitwise AND with 1 wont change anything
            VAR(AX0_u16_Position_Modulo_Active) &= 1;

            if (!VAR(AX0_u16_Abs_Fdbk_Device))            //  ABS Feedback - no need to home PCOM
            {
               if (Pcom_1_Cntrl_Word.u16_en)
                  PcomInit(1);
               if (Pcom_2_Cntrl_Word.u16_en)
                  PcomInit(2);
            }
         break;

         case HOMING_TARGET_REACHED:
            if ( ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0)    ||  // Lost Feedback invalidates
                 ((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0)  ) // "Homed" Indication
               // Lost Feedback invalidates Homed" Indication
               BGVAR(u8_Homing_State) = HOMING_IDLE;

            LLVAR(AX0_u32_Home_Offset_User_Lo) = LLVAR(AX0_u32_Home_Offset_Lo);  // Set homing offset user variable
         break;

         case HOMING_FAILURE_TERMINATE_HOME:
            BGVAR(u8_Homing_Ignore_LS) = 0;
            TerminateHome(drive);
            MoveSmoothSourceCommand((long long)BGVAR(u16_MoveSmooth_Src), drive);
            BGVAR(u8_Homing_State) = HOMING_FAILED;

            UpdateRefOffsetValue(drive);            // Restore homing offset operational variable with user variable
         break;

         case HOMING_FAILED:
            LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev) = 0;
            // TBD
         break;

         default:
            BGVAR(u8_Homing_State) = HOMING_IDLE;
         break;
      }
   } // else if (Detected_State_Machine == 3)

   if (Detected_State_Machine == 4)
   {
      switch (BGVAR(u8_Homing_State)) // State-Machine for "Complex" Homing Sequences
      {
         case HOMING_IDLE:
            u16_temp = 0;
            VAR(AX0_u16_Home_Ind) = 0;
            BGVAR(u8_Homing_Ignore_LS) = 0; // LS in use Indicator...
         break;

         case HOMING_SETUP: // 1
            VAR(AX0_u16_Home_Ind) = 0;
            InitHomeParam(drive);
            BGVAR(u8_Homing_State) = HOMING_INITIAL_STATE_DETECTION;
            LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev) = LLVAR(AX0_u32_Home_Delta_Offset_Lo);

            if (!VAR(AX0_u16_Abs_Fdbk_Device))// ABS Feedback - no need to home PCOM
            {  //PCOM support
               if (Pcom_1_Cntrl_Word.u16_en)
                  PcomDisable(1);
               if (Pcom_2_Cntrl_Word.u16_en)
                  PcomDisable(2);
            }
         break;

         case HOMING_INITIAL_STATE_DETECTION:
            BGVAR(u8_Homing_Ignore_LS) = LimitSwitchIgnore(BGVAR(u64_1st_Target));

            if ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )
               BGVAR(u8_Homing_State) = HOMING_OFF_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                              ) ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                               )   )
               BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
            else if (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))
            {
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
            else
               BGVAR(u8_Homing_State) = HOMING_INITIAL_DIR_MOVE;
         break;

         case HOMING_INITIAL_DIR_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_1st_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_1st_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_1ST_SWITCH;
         break;

         case HOMING_SEEK_1ST_SWITCH:
            BGVAR(u8_Homing_Ignore_LS) = LimitSwitchIgnore(BGVAR(u64_1st_Target));

            if ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )
               BGVAR(u8_Homing_State) = HOMING_OFF_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                            &&
                        ( (IsHoldActive(drive) == 1)  )                                        ) ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                             &&
                        ( (IsHoldActive(drive) == 1)  )                                        )  ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON) &&
                        ReadCurrentLevel(drive)                                                  )                 )
            {
               if ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON)
                  BGVAR(u8_Homing_State) = HOMING_HARDSTOP_DECEL;
               else
                  BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
            }
            else if (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)
               BGVAR(u8_Homing_State) = HOMING_TRIGGER_ARMING;
            else if ( ( (((BGVAR(u64_1st_Target) >> 24) & 0xFF) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                ) ||
                      ( (((BGVAR(u64_1st_Target) >> 24) & 0xFF) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                 ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))     )
              // Reaching this State with Drive Disabled is possible only with HOMETYPE 35,
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_HARDSTOP_DECEL:
//          Use higher Deceleration for this Stop to allow quicker response to Hardstop
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 24);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            BGVAR(s16_Move_Abs_Issued) = 0;
            InitiatePtpMove(drive, 0x0LL, 0x0L, u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            BGVAR(u8_Homing_State) = HOMING_HRDSTP_DECEL_DETECT;
         break;

         case HOMING_HRDSTP_DECEL_DETECT:
            if (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))
            {
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
            else if (VAR(AX0_s16_Stopped) >= 1)
               BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
         break;

         case HOMING_FLIP_1ST_SWITCH_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_2nd_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_2nd_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_FLIP_1ST_SWITCH;
         break;

         case HOMING_SEEK_FLIP_1ST_SWITCH:
            if ( ( (BGVAR(u8_Homing_Ignore_LS) && 0x01)                   &&
                   (((BGVAR(u64_2nd_Target) >> 8) & 0xFF) == NEG_LIM_OFF) &&
                   ((u16_local_ccw_ls) == 0)                                ) ||
                 ( (BGVAR(u8_Homing_Ignore_LS) && 0x02)                   &&
                   (((BGVAR(u64_2nd_Target) >> 8) & 0xFF) == POS_LIM_OFF) &&
                   ((u16_local_cw_ls) == 0)                                 )   )
               BGVAR(u8_Homing_Ignore_LS) = 0; // Zeroing the LS Ignore Variable is conditioned on the process
               // identifying LS Falling Edge, to correctly manage Homing on LS Falling Edge.

            if ( ( ((((BGVAR(u64_2nd_Target) >> 8) & 0xFF) & NEG_LIM_OFF) == NEG_LIM_OFF) && ((u16_local_ccw_ls) == 0) ) ||
                 ( ((((BGVAR(u64_2nd_Target) >> 8) & 0xFF) & POS_LIM_OFF) == POS_LIM_OFF) && ((u16_local_cw_ls) == 0) )    )
               BGVAR(u8_Homing_State) = HOMING_SEEK_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               )           ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                )           ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                        (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                             )   ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                         (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                              ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                              )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                               )
               {
                  u16_temp |= HOME_SWITCH_ON_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               if ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                               )
               {
                  u16_temp |= HOME_SWITCH_OFF_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SEEK_HOME_SWITCH_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_3rd_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_3rd_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_HOME_SWITCH;
         break;

         case HOMING_SEEK_HOME_SWITCH:
            if ( (((BGVAR(u64_End_Target) >> 8) & 0xFF) == NO_TRIG)                                ||
                 // No Switch Transition means the next transition is a Homing Trigger
                 ( ((((BGVAR(u64_3rd_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                   (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )  )
               BGVAR(u8_Homing_State) = HOMING_TRIGGER_ARMING;
            else if ( ( ((((BGVAR(u64_3rd_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               ) ||
                      ( ((((BGVAR(u64_3rd_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                    )
              // Reaching this State with Drive Disabled is possible only with HOMETYPE 35,
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_OFF_HOME_SWITCH_MOVE:
            BGVAR(u8_Homing_Ignore_LS) = 0;

            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_4th_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_4th_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_OFF_HOME_SWITCH;
         break;

         case HOMING_SEEK_OFF_HOME_SWITCH:
            if ( ((((BGVAR(u64_4th_Target) >> 8) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                  (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                             )
               BGVAR(u8_Homing_State) = HOMING_ON_HOME_SWITCH_MOVE; // 13
            else if ( ( ((((BGVAR(u64_4th_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               ) ||
                      ( ((((BGVAR(u64_4th_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                    )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_ON_HOME_SWITCH_MOVE: // 13
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_End_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_End_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_TRIGGER_CONDITIONS; // 14
         break;

         case HOMING_SEEK_TRIGGER_CONDITIONS: // 14
            if ( (((BGVAR(u64_End_Target) >> 8) & 0xFF) == NO_TRIG)                                ||
                 // No Switch Transition means the next transition is a Homing Trigger
                 ( ((((BGVAR(u64_End_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                   (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )  )
               BGVAR(u8_Homing_State) = HOMING_TRIGGER_ARMING;
            else if ( ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               )           ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                )           ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                        ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) != HOME_SWITCH_OFF) &&
                        (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                               ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                              )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }
               
               if ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                               )
               {
                  u16_temp |= HOME_SWITCH_ON_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }
               
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }
               
               if ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                               )
               {
                  u16_temp |= HOME_SWITCH_OFF_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_TRIGGER_ARMING:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_End_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_End_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            VAR(AX0_s16_Crrnt_Run_Code) |= ARM_HOMING_CAPTURE_MASK; // ARM_HOMING_TRIGGER;
            BGVAR(u8_Homing_State) = HOMING_SEEK_TRIGGER;
         break;

         case HOMING_SEEK_TRIGGER:
            if ((VAR(AX0_s16_Crrnt_Run_Code) & ARM_HOMING_CAPTURE_MASK) == 0)
            {
               PtpAbort(drive);
               BGVAR(u8_Homing_State) = HOMING_SET_HOME_OFFSET;
            }
            else if ( ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               )           ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                )           ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                        (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                             )   ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                         (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                              ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                              )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }
               
               if ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                               )
               {
                  u16_temp |= HOME_SWITCH_ON_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }
               
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }
               
               if ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                               )
               {
                  u16_temp |= HOME_SWITCH_OFF_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }
               
               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SET_HOME_OFFSET:
            if (VAR(AX0_s16_Stopped) >= 1)
            {  // Set Home_Offset per s64_Home_Offset, Move to Zero Position if required,
               s64_Temp = LLVAR(AX0_u32_Prev_Captured_Pfb_Lo) - LLVAR(AX0_u32_Captured_Pfb_Lo);
               s64_Temp = BGVAR(s64_Home_Offset) + (s64_Temp * VAR(AX0_u16_Home_Capture_Time) / 1875) - LLVAR(AX0_u32_Prev_Captured_Pfb_Lo);
               LLVAR(AX0_u32_Home_Offset_Lo_Temp) = s64_Temp;
               s64_Temp -= LLVAR(AX0_u32_Home_Offset_Lo) - LLVAR(AX0_u32_Home_Delta_Offset_Lo);
               LLVAR(AX0_u32_Home_Delta_Offset_Lo_Temp) = s64_Temp;
               VAR(AX0_s16_Crrnt_Run_Code) |= HOMING_OFFSET_READY_MASK;
               BGVAR(u8_Homing_State) = HOMING_MOVE_ZERO_POS; // 18
            }
            else if ( ( (VAR(AX0_s16_Stopped) == -1) || (!Enabled(drive)) ) &&
                      ((BGVAR(u64_End_Target) & 0xFF) != 0)                   )
            { // If Drive is Disabled, or Stopped Indication is incorrect, declare Failure
               PtpAbort(drive);
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               if (VAR(AX0_s16_Stopped) == -1)
               {
                  u16_temp |= MOTION_INTERRUPTED_BIT;
                  s16_sal_result = MOTION_INTERRUPTED;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_MOVE_ZERO_POS: // 18
            if ((VAR(AX0_s16_Crrnt_Run_Code) & HOMING_OFFSET_READY_MASK) == 0)
            {
               u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
               u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
               do {
                  u16_temp = Cntr_3125;
                  BGVAR(s64_Temp_Pos_LS) = -LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
               } while (u16_temp != Cntr_3125);
               BGVAR(s16_Move_Abs_Issued) = 0;
               // Initialize LS Hold Target Position to provide for Limit Switch Stop
               if (((BGVAR(u64_End_Target) >> 16) & 0xFF) != 0)
                // No Trigger-Source means Homing-in-Place, avoid moving to Home-Offset.
               {
                  if (BGVAR(u16_Home_Ofst_Move) == 1) // Move to Home-Offset, PFB will be 0.
                     InitiatePtpMove(drive, 0x0LL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
                  else // Move to Trigger-Source Position, PFB will be Home-Offset
                     InitiatePtpMove(drive, BGVAR(s64_Home_Offset), BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
               }
               BGVAR(u8_Homing_State) = HOMING_ATTAINED;
               VAR(AX0_u16_Home_Ind) = 1;
            }
         break;

         case HOMING_ATTAINED:
            if ( (Enabled(drive) && (LVAR(AX0_s32_Pos_Vcmd) == 0L)) ||
                 (((BGVAR(u64_End_Target) >> 16) & 0xFF) == 0)        )
               // Verify arrival at Target Position, or ignore if Homing-in-Place...
               BGVAR(u16_homing_pcmd_settled)++;
            else
               BGVAR(u16_homing_pcmd_settled) = 0;

            if (BGVAR(u16_homing_pcmd_settled) > 2)
               BGVAR(u8_Homing_State) = HOMING_SUCCESS_TERMINATE_HOME;

            if ( ((u16_local_ccw_ls) != 0) || ((u16_local_cw_ls) != 0)     ||
                 (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))  )
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SUCCESS_TERMINATE_HOME:
            BGVAR(u8_Homing_Ignore_LS) = 0;
            TerminateHome(drive);
            MoveSmoothSourceCommand((long long)BGVAR(u16_MoveSmooth_Src), drive);
            BGVAR(u8_Homing_State) = HOMING_TARGET_REACHED;

            //  Re-Initialize Modulo State Machine bit mask
            //  The right most bit of this variable indicates if the modulo is active
            //  If the modulo is active then bitwise AND with 1 will Restart the Modulo State Machine (clearing all the other bits in the bit mask)
            //  Else the Modulo is not active hence bitwise AND with 1 wont change anything
            VAR(AX0_u16_Position_Modulo_Active) &= 1;

            if (!VAR(AX0_u16_Abs_Fdbk_Device))            //  ABS Feedback - no need to home PCOM
            {  //PCOM support
               if (Pcom_1_Cntrl_Word.u16_en)
                  PcomInit(1);
               if (Pcom_2_Cntrl_Word.u16_en)
                  PcomInit(2);
            }
         break;

         case HOMING_TARGET_REACHED:
            if ( ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0)    ||
                 ((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0)  )
                  // Lost Feedback invalidates Homed" Indication
               BGVAR(u8_Homing_State) = HOMING_IDLE;

            LLVAR(AX0_u32_Home_Offset_User_Lo) = LLVAR(AX0_u32_Home_Offset_Lo);  // Set homing offset user variable
         break;

         case HOMING_FAILURE_TERMINATE_HOME:
            BGVAR(u8_Homing_Ignore_LS) = 0;
            TerminateHome(drive);
            MoveSmoothSourceCommand((long long)BGVAR(u16_MoveSmooth_Src), drive);
            BGVAR(u8_Homing_State) = HOMING_FAILED;

            UpdateRefOffsetValue(drive);            // Restore homing offset operational variable with user variable
         break;

         case HOMING_FAILED:
            LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev) = 0;
            // TBD
         break;

         default:
            BGVAR(u8_Homing_State) = HOMING_IDLE;
         break;
      }
   } // else if (Detected_State_Machine == 4)

   if (Detected_State_Machine == 5)
   {
      switch (BGVAR(u8_Homing_State)) // State-Machine for "Complex" Homing Sequences
      {
         case HOMING_IDLE:
            u16_temp = 0;
            VAR(AX0_u16_Home_Ind) = 0;
            BGVAR(u8_Homing_Ignore_LS) = 0; // LS in use Indicator...
         break;

         case HOMING_SETUP: // 1
            VAR(AX0_u16_Home_Ind) = 0;
            InitHomeParam(drive);
            BGVAR(u8_Homing_State) = HOMING_INITIAL_STATE_DETECTION;
            LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev) = LLVAR(AX0_u32_Home_Delta_Offset_Lo);

            if (!VAR(AX0_u16_Abs_Fdbk_Device))// ABS Feedback - no need to home PCOM
            {  //PCOM support
               if (Pcom_1_Cntrl_Word.u16_en)
                  PcomDisable(1);
               if (Pcom_2_Cntrl_Word.u16_en)
                  PcomDisable(2);
            }
         break;

         case HOMING_INITIAL_STATE_DETECTION:
            BGVAR(u8_Homing_Ignore_LS) = LimitSwitchIgnore(BGVAR(u64_1st_Target));

            if ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )
               BGVAR(u8_Homing_State) = HOMING_OFF_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                              ) ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                               )   )
               BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
            else if (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
            else
               BGVAR(u8_Homing_State) = HOMING_INITIAL_DIR_MOVE;
         break;

         case HOMING_INITIAL_DIR_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_1st_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_1st_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_1ST_SWITCH;
         break;

         case HOMING_SEEK_1ST_SWITCH:
            BGVAR(u8_Homing_Ignore_LS) = LimitSwitchIgnore(BGVAR(u64_1st_Target));

            if ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )
               BGVAR(u8_Homing_State) = HOMING_OFF_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                            &&
                        ( (IsHoldActive(drive) == 1)  )                                        )  ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                             &&
                        ( (IsHoldActive(drive) == 1)  )                                        )  ||
                      ( ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON) &&
                        ReadCurrentLevel(drive)                                                  )  )
            {
               if ((((BGVAR(u64_1st_Target) >> 8) & 0xFF) & HARDSTOP_ON) == HARDSTOP_ON)
                  BGVAR(u8_Homing_State) = HOMING_HARDSTOP_DECEL;
               else
                  BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
            }
            else if ( ( (((BGVAR(u64_1st_Target) >> 24) & 0xFF) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)         ) ||
                      ( (((BGVAR(u64_1st_Target) >> 24) & 0xFF) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)          ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))     )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_HARDSTOP_DECEL:
//          Use higher Deceleration for this Stop to allow quicker response to Hardstop
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 24);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            BGVAR(s16_Move_Abs_Issued) = 0;
            InitiatePtpMove(drive, 0x0LL, 0x0L, u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            BGVAR(u8_Homing_State) = HOMING_HRDSTP_DECEL_DETECT;
         break;

         case HOMING_HRDSTP_DECEL_DETECT:
            if (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))
            {
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
            else if (VAR(AX0_s16_Stopped) >= 1)
               BGVAR(u8_Homing_State) = HOMING_FLIP_1ST_SWITCH_MOVE;
         break;

         case HOMING_FLIP_1ST_SWITCH_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_2nd_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_2nd_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_FLIP_1ST_SWITCH;
         break;

         case HOMING_SEEK_FLIP_1ST_SWITCH:
            if ( ( (BGVAR(u8_Homing_Ignore_LS) && 0x01)                   &&
                   (((BGVAR(u64_2nd_Target) >> 8) & 0xFF) == NEG_LIM_OFF) &&
                   ((u16_local_ccw_ls) == 0)                                ) ||
                 ( (BGVAR(u8_Homing_Ignore_LS) && 0x02)                   &&
                   (((BGVAR(u64_2nd_Target) >> 8) & 0xFF) == POS_LIM_OFF) &&
                   ((u16_local_cw_ls) == 0)                                 )   )
               BGVAR(u8_Homing_Ignore_LS) = 0; // Zeroing the LS Ignore Variable is conditioned on the process
               // identifying LS Falling Edge, to correctly manage Homing on LS Falling Edge.

            if ( ( ((((BGVAR(u64_2nd_Target) >> 8) & 0xFF) & NEG_LIM_OFF) == NEG_LIM_OFF) && ((u16_local_ccw_ls) == 0) ) ||
                 ( ((((BGVAR(u64_2nd_Target) >> 8) & 0xFF) & POS_LIM_OFF) == POS_LIM_OFF) && ((u16_local_cw_ls) == 0) )    )
               BGVAR(u8_Homing_State) = HOMING_SEEK_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               )           ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                )           ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                        (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                             )   ||
                      ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                         (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                              ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                              )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                               )
               {
                  u16_temp |= HOME_SWITCH_ON_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               if ( ((((BGVAR(u64_2nd_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                               )
               {
                  u16_temp |= HOME_SWITCH_OFF_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SEEK_HOME_SWITCH_MOVE:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_3rd_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_3rd_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_HOME_SWITCH;
         break;

         case HOMING_SEEK_HOME_SWITCH:
            if ( ((((BGVAR(u64_3rd_Target) >> 8) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                 (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                            )
               BGVAR(u8_Homing_State) = HOMING_OFF_HOME_SWITCH_MOVE;
            else if ( ( ((((BGVAR(u64_3rd_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               ) ||
                      ( ((((BGVAR(u64_3rd_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                    )
            { // Reaching this State with Drive Disabled is possible only with HOMETYPE 35, not in this State-Machine
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_OFF_HOME_SWITCH_MOVE:
            BGVAR(u8_Homing_Ignore_LS) = 0;

            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_End_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_End_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Switch_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            BGVAR(u8_Homing_State) = HOMING_SEEK_OFF_HOME_SWITCH;
         break;

         case HOMING_SEEK_OFF_HOME_SWITCH:
            if ( (((BGVAR(u64_End_Target) >> 8) & 0xFF) == NO_TRIG)                                  ||
                 // No Switch Transition means the next transition is a Homing Trigger
                 ( ((((BGVAR(u64_End_Target) >> 8) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                             )  )
               BGVAR(u8_Homing_State) = HOMING_TRIGGER_ARMING;
            else if ( ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                                               ) ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                                                ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                    )
              // Reaching this State with Drive Disabled is possible only with HOMETYPE 35,
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_TRIGGER_ARMING:
            u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
            u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
            if ((BGVAR(u64_End_Target) & 0xFF) == NEG_DIR)      // GO_NEGATIVE;
            { // Move to "Negative Infinity", set Limit-Switch Hold Target Position to allow Negative
               BGVAR(s64_Temp_Pos_LS) = 0xF000000000000000LL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0xF000000000000000LL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else if ((BGVAR(u64_End_Target) & 0xFF) == POS_DIR) // GO_POSITIVE;
            { // Move to "Positive Infinity", set Limit-Switch Hold Target Position to allow Positive
               BGVAR(s64_Temp_Pos_LS) = 0x3FFFFFFFFFFFFFFFLL; // LS to Hold (stop) Move
               BGVAR(s16_Move_Abs_Issued) = 0;
               InitiatePtpMove(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
            }
            else
               PtpAbort(drive);
            VAR(AX0_s16_Crrnt_Run_Code) |= ARM_HOMING_CAPTURE_MASK; // ARM_HOMING_TRIGGER;
            BGVAR(u8_Homing_State) = HOMING_SEEK_TRIGGER;
         break;

         case HOMING_SEEK_TRIGGER:
            if ((VAR(AX0_s16_Crrnt_Run_Code) & ARM_HOMING_CAPTURE_MASK) == 0)
            {
               PtpAbort(drive);
               BGVAR(u8_Homing_State) = HOMING_SET_HOME_OFFSET;
            }
            else if ( ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & NEG_LIM_ON) == NEG_LIM_ON) &&
                        ((u16_local_ccw_ls) != 0)                        )           ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & POS_LIM_ON) == POS_LIM_ON) &&
                        ((u16_local_cw_ls) != 0)                         )           ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON) &&
                        (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                             )   ||
                      ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                         (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                              ) ||
                      (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))                              )
              // Reaching this State with Drive Disabled is possible only with HOMETYPE 35,
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }
               
               if ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_ON) == HOME_SWITCH_ON)   &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) != 0)                               )
               {
                  u16_temp |= HOME_SWITCH_ON_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }
               
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }
               
               if ( ((((BGVAR(u64_End_Target) >> 24) & 0xFF) & HOME_SWITCH_OFF) == HOME_SWITCH_OFF) &&
                    (VAR(AX0_u16_Mode_Input_Arr[HOME_SWITCH_INP]) == 0)                               )
               {
                  u16_temp |= HOME_SWITCH_OFF_BIT;
                  s16_sal_result = OPPOSITE_HOME_SWITCH_EXPECTED;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SET_HOME_OFFSET:
            if (VAR(AX0_s16_Stopped) >= 1)
            {  // Set Home_Offset per s64_Home_Offset, Move to Zero Position if required,
               s64_Temp = LLVAR(AX0_u32_Prev_Captured_Pfb_Lo) - LLVAR(AX0_u32_Captured_Pfb_Lo);
               s64_Temp = BGVAR(s64_Home_Offset) + (s64_Temp * VAR(AX0_u16_Home_Capture_Time) / 1875) - LLVAR(AX0_u32_Prev_Captured_Pfb_Lo);
               LLVAR(AX0_u32_Home_Offset_Lo_Temp) = s64_Temp;
               s64_Temp -= LLVAR(AX0_u32_Home_Offset_Lo) - LLVAR(AX0_u32_Home_Delta_Offset_Lo);
               LLVAR(AX0_u32_Home_Delta_Offset_Lo_Temp) = s64_Temp;
               VAR(AX0_s16_Crrnt_Run_Code) |= HOMING_OFFSET_READY_MASK;
               BGVAR(u8_Homing_State) = HOMING_MOVE_ZERO_POS; // 18
            }
            else if ( ( (VAR(AX0_s16_Stopped) == -1) || (!Enabled(drive)) ) &&
                      ((BGVAR(u64_End_Target) & 0xFF) != 0)                   )
            { // If Drive is Disabled, or Stopped Indication is incorrect, declare Failure
               PtpAbort(drive);
               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               if (VAR(AX0_s16_Stopped) == -1)
               {
                  u16_temp |= MOTION_INTERRUPTED_BIT;
                  s16_sal_result = MOTION_INTERRUPTED;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_MOVE_ZERO_POS: // 18
            if ((VAR(AX0_s16_Crrnt_Run_Code) & HOMING_OFFSET_READY_MASK) == 0)
            {
               u32_HomeAccRounded = (long)((BGVAR(u64_HomeAccRate) + (unsigned long long)0x080000000) >> 32);
               u32_HomeDecRounded = (long)((BGVAR(u64_HomeDecRate) + (unsigned long long)0x080000000) >> 32);
               do {
                  u16_temp = Cntr_3125;
                  BGVAR(s64_Temp_Pos_LS) = -LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
               } while (u16_temp != Cntr_3125);
               BGVAR(s16_Move_Abs_Issued) = 0;
               // Initialize LS Hold Target Position to provide for Limit Switch Stop
               if (((BGVAR(u64_End_Target) >> 16) & 0xFF) != 0)
                // No Trigger-Source means Homing-in-Place, avoid moving to Home-Offset.
               {
                  if (BGVAR(u16_Home_Ofst_Move) == 1) // Move to Home-Offset, PFB will be 0.
                     InitiatePtpMove(drive, 0x0LL, BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
                  else // Move to Trigger-Source Position, PFB will be Home-Offset
                     InitiatePtpMove(drive, BGVAR(s64_Home_Offset), BGVAR(u32_Home_Zero_Speed), u32_HomeAccRounded, u32_HomeDecRounded, PTP_IMMEDIATE_TRANSITION);
               }
               BGVAR(u8_Homing_State) = HOMING_ATTAINED;
               VAR(AX0_u16_Home_Ind) = 1;
            }
         break;

         case HOMING_ATTAINED:
            if ( (Enabled(drive) && (LVAR(AX0_s32_Pos_Vcmd) == 0L)) ||
                 (((BGVAR(u64_End_Target) >> 16) & 0xFF) == 0)        )
               // Verify arrival at Target Position, or ignore if Homing-in-Place...
               BGVAR(u16_homing_pcmd_settled)++;
            else
               BGVAR(u16_homing_pcmd_settled) = 0;

            if (BGVAR(u16_homing_pcmd_settled) > 2)
               BGVAR(u8_Homing_State) = HOMING_SUCCESS_TERMINATE_HOME;

            if ( ((u16_local_ccw_ls) != 0) || ((u16_local_cw_ls) != 0)     ||
                 (!Enabled(drive) && ((BGVAR(u64_End_Target) & 0xFF) != 0))  )
            {
               PtpAbort(drive);
               if ((u16_local_ccw_ls) != 0)
               {
                  u16_temp |= CCW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CCW_LIMIT_SWITCH_ACTIVE;
               }

               if ((u16_local_cw_ls) != 0)
               {
                  u16_temp |= CW_LIMIT_SWITCH_ACTIVE_BIT;
                  s16_sal_result = CW_LIMIT_SWITCH_ACTIVE;
               }

               if (!Enabled(drive))
               {
                  u16_temp |= DRIVE_INACTIVE_BIT;
                  s16_sal_result = DRIVE_INACTIVE;
               }

               HomeFailureIndication(drive, Detected_State_Machine, BGVAR(u8_Homing_State), u16_temp, s16_sal_result);
               BGVAR(u8_Homing_State) = HOMING_FAILURE_TERMINATE_HOME;
            }
         break;

         case HOMING_SUCCESS_TERMINATE_HOME:
            BGVAR(u8_Homing_Ignore_LS) = 0;
            TerminateHome(drive);
            MoveSmoothSourceCommand((long long)BGVAR(u16_MoveSmooth_Src), drive);
            BGVAR(u8_Homing_State) = HOMING_TARGET_REACHED;

            //  Re-Initialize Modulo State Machine bit mask
            //  The right most bit of this variable indicates if the modulo is active
            //  If the modulo is active then bitwise AND with 1 will Restart the Modulo State Machine (clearing all the other bits in the bit mask)
            //  Else the Modulo is not active hence bitwise AND with 1 wont change anything
            VAR(AX0_u16_Position_Modulo_Active) &= 1;

            if (!VAR(AX0_u16_Abs_Fdbk_Device))            //  ABS Feedback - no need to home PCOM
            {  //PCOM support
               if (Pcom_1_Cntrl_Word.u16_en)
                  PcomInit(1);
               if (Pcom_2_Cntrl_Word.u16_en)
                  PcomInit(2);
            }
         break;

         case HOMING_TARGET_REACHED:
            if ( ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0)    ||
                 ((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0)  )
                  // Lost Feedback invalidates "Homed" Indication
               BGVAR(u8_Homing_State) = HOMING_IDLE;

            LLVAR(AX0_u32_Home_Offset_User_Lo) = LLVAR(AX0_u32_Home_Offset_Lo);    // Set homing offset user variable
         break;

         case HOMING_FAILURE_TERMINATE_HOME:
            BGVAR(u8_Homing_Ignore_LS) = 0;
            TerminateHome(drive);
            MoveSmoothSourceCommand((long long)BGVAR(u16_MoveSmooth_Src), drive);
            BGVAR(u8_Homing_State) = HOMING_FAILED;

            UpdateRefOffsetValue(drive);            // Restore homing offset operational variable with user variable
         break;

         case HOMING_FAILED:
            LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev) = 0;
            // TBD
         break;

         default:
            BGVAR(u8_Homing_State) = HOMING_IDLE;
         break;
      }
   } // else if (Detected_State_Machine == 5)

   return;
}


// This function updates the internal AX0_u32_Home_Offset_Lo variable with the user
// non-volatile Variable if the feedback device supports absolute multi-turn response.
void UpdateRefOffsetValue(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if (IS_DUAL_LOOP_ACTIVE)
   { // For Active Dual-Loop Setup, test if Load Feedback is Absolute or not
      if ((VAR(AX0_u16_Abs_Fdbk_Device) & 0x0002) == 0)
      {// When Load Feedback is not Absolute or not Homed already, avoid restoring Home Offset
         LLVAR(AX0_u32_Home_Offset_Lo_Temp) = 0;
         LLVAR(AX0_u32_Home_Delta_Offset_Lo_Temp) = 0;
      }
      else
      {
         LLVAR(AX0_u32_Home_Offset_Lo_Temp) = LLVAR(AX0_u32_Home_Offset_User_Lo);
         LLVAR(AX0_u32_Home_Delta_Offset_Lo_Temp) = LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev);
      }
      VAR(AX0_s16_Crrnt_Run_Code) |= HOMING_OFFSET_READY_MASK;
   }
   else
   { // For Inactive Dual-Loop Setup, test if Motor Feedback is Absolute or not
      if ((VAR(AX0_u16_Abs_Fdbk_Device) & 0x0001) == 0)
      {// When Motor Feedback is not Absolute or not Homed already, avoid restoring Home Offset
         LLVAR(AX0_u32_Home_Offset_Lo_Temp) = 0;
         LLVAR(AX0_u32_Home_Delta_Offset_Lo_Temp) = 0;
      }
      else
      {
         LLVAR(AX0_u32_Home_Offset_Lo_Temp) = LLVAR(AX0_u32_Home_Offset_User_Lo);
         LLVAR(AX0_u32_Home_Delta_Offset_Lo_Temp) = LLVAR(AX0_u32_Home_Delta_Offset_Lo_Prev);
      }
      VAR(AX0_s16_Crrnt_Run_Code) |= HOMING_OFFSET_READY_MASK;
   }
}


//**********************************************************
// Function Name: HomeCmdStat
// Description:
//          This function is called in response to the HOMECMDST command.
//
// Author: A. H.
// Algorithm:
// Revisions: nitsan 27/4/2015: change the bits of BGVAR(u32_Home_Fail_Ind) as follows:
//            detected state machine bits 29 to 31
//            failure state bits 24 - 28
//            failure reason bits 16 - 23
//            failure code (from SAL return codes) lower 16 bits (not used in this function).
//**********************************************************
int HomeCmdSt(int drive)
{
   int u16_temp = 0;
   REFERENCE_TO_DRIVE;

   if (u8_Output_Buffer_Free_Space < 60) return SAL_NOT_FINISHED;

   switch (BGVAR(u8_Homing_State))
   {
      case HOMING_IDLE:
         if (PAUSE_INPUT_HOMING_INTERRUPTED != BGVAR(u16_Interrupted_Motion)) PrintStringCrLf("Homing Not Issued", 0);
         else PrintStringCrLf("Homing Canceled by HOLD, disable HOLD to restart", 0);
      break;

      case HOMING_TARGET_REACHED:
         PrintStringCrLf("Homing Succeeded", 0);
      break;

      case HOMING_FAILED:
         PrintString("State-Machine used: ", 0);
         PrintUnsignedInt16((int)((BGVAR(u32_Home_Fail_Ind) >> 29) & 0x0007));   // using 3 highest bits
         PrintCrLf();
         PrintString("Failure at Homing State: ", 0);
         PrintUnsignedInt16((int)((BGVAR(u32_Home_Fail_Ind) >> 24) & 0x001F));   // using next highest 5 bits
         PrintCrLf();
         PrintStringCrLf("Failure Cause: ", 0);
         u16_temp = (int)((BGVAR(u32_Home_Fail_Ind) >> 16) & 0x00FFL);           // using next highest 8 bits
         if ((u16_temp & CCW_LIMIT_SWITCH_ACTIVE_BIT) != 0)  PrintStringCrLf("Neg. Limit-Switch", 0);
         if ((u16_temp & CW_LIMIT_SWITCH_ACTIVE_BIT ) != 0)  PrintStringCrLf("Pos. Limit-Switch", 0);
         if ((u16_temp & HOME_SWITCH_ON_BIT         ) != 0)  PrintStringCrLf("Home-Switch not Engaged", 0);
         if ((u16_temp & DRIVE_INACTIVE_BIT         ) != 0)  PrintStringCrLf("Drive Disabled", 0);
         if ((u16_temp & MOTION_INTERRUPTED_BIT     ) != 0)  PrintStringCrLf("Incorrect Stopping Indication", 0);
         if ((u16_temp & HOME_SWITCH_OFF_BIT        ) != 0)  PrintStringCrLf("Home-Switch not Disengaged", 0);
      break;

      default: // Homing in progress...
         PrintStringCrLf("Homing Process Active", 0);
         PrintString("Currently at State: ", 0);
         PrintUnsignedInt16(BGVAR(u8_Homing_State));
         PrintString(", using State-Machine: ", 0);
         PrintUnsignedInt16(s_HomeTypes_Init_Table[SearchHomeType(BGVAR(s16_Home_Type))].State_Machine);
         PrintCrLf();
   }
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: HomeFailureIndication
// Description: Compose the indication dword according to Homing detected stae machine, homing state, 
//              failure bits and equivalent SAL result value
//
//            BGVAR(u32_Home_Fail_Ind) structure:
//            detected state machine bits 29 to 31 (3 bits)
//            failure state bits 24 - 28           (5 bits)
//            failure reason bits 16 - 23          (8 bits)
//            failure code (from SAL return codes) lower 16 bits (not used in this function).
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void HomeFailureIndication(int drive, unsigned int u16_detected_state_machine, unsigned int u16_homing_state, unsigned int u16_failure_bits, int s16_sal_result)
{
   REFERENCE_TO_DRIVE;

   BGVAR(u32_Home_Fail_Ind) = (((unsigned long)u16_detected_state_machine & 0x7) << 29) |
                              (((unsigned long)u16_homing_state & 0x1F) << 24)          |
                              (((unsigned long)u16_failure_bits & 0x00ff) << 16)        |
                              ((unsigned long)s16_sal_result & 0x0000FFFF);
}


//**********************************************************
// Function Name: SalReadHomeFailureInd
// Description: return the homing failure indication value.
//              Note: 0x1000 is added to the sal result value to meet the lxm28 error list spec
//
//            detected state machine bits 29 to 31
//            failure state bits 24 - 28
//            failure reason bits 16 - 23
//            failure code (from SAL return codes) lower 16 bits (not used in this function).
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalReadHomeFailureInd(long long *data, int drive)
{
   unsigned long u32_tmp = 0UL;

   REFERENCE_TO_DRIVE;

   u32_tmp = BGVAR(u32_Home_Fail_Ind);

   *data = (long long)u32_tmp;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SetHomePosition
// Description: Sets PFB immediately to the requested value.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SetHomePosition(long long s64_expected_pos, int drive)
{

   int s16_temp;
   long long s64_Temp;

   if (LVAR(AX0_s32_Pos_Vcmd)!= 0)
   {
      return MOTOR_IN_MOTION;
   }

   if (IsHomingProcessRunning(drive))
   {
      HomeFailureIndication(drive, 0, 0, 0, HOMING_IN_PROGRESS);      // set failure indication.
      return HOMING_IN_PROGRESS; // do not restart Homing when in the middle of the process
   }

   do {   // Read PFB thread safe
      s16_temp = Cntr_3125;
      s64_Temp = LLVAR(AX0_u32_Pos_Fdbk_Dual_Lo);
   } while (s16_temp != Cntr_3125);

   // s64_temp holds the position difference we need to move PFB with
   s64_Temp = (s64_expected_pos - s64_Temp);

   // Set upcoming value for "LLVAR(AX0_u32_Home_Offset_Lo)", move by the requested difference
   LLVAR(AX0_u32_Home_Offset_Lo_Temp) = LLVAR(AX0_u32_Home_Offset_Lo) + s64_Temp;

   // Calculate the current difference between the homing PCMD_offset and PFB_offset
   s64_Temp = LLVAR(AX0_u32_Home_Delta_Offset_Lo) - LLVAR(AX0_u32_Home_Offset_Lo);
   // Set the upcoming PCMD_offset to the upcoming PFB_offset plus current delta between these two offsets.
   LLVAR(AX0_u32_Home_Delta_Offset_Lo_Temp) = LLVAR(AX0_u32_Home_Offset_Lo_Temp) + s64_Temp;

   // Now ensure that the shadow variables are being applied and setting the position is performed
   VAR(AX0_s16_Crrnt_Run_Code) |= HOMING_OFFSET_READY_MASK;

   return SAL_SUCCESS;
}
