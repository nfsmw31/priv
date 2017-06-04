#include "DSP2834x_Device.h"

#include "Design.def"
#include "Err_Hndl.def"
#include "Current.def"
#include "FltCntrl.def"
#include "FPGA.def"
#include "Hiface.def"
#include "Endat.def"
#include "MultiAxis.def"
#include "ModCntrl.def"

#include "Drive.var"
#include "EncCnfg.var"
#include "Endat.var"
#include "Extrn_Asm.var"
#include "FltCntrl.var"
#include "Hiface.var"
#include "init.var"
#include "ModCntrl.var"
#include "Motor.var"
#include "PhaseFind.var"
#include "Position.var"
#include "Ser_Comm.var"

#include "Prototypes.pro"


int EncoderConfig(int drive)
{
   // AXIS_OFF;

   int ret_val = SUCCESS;

   /* test if enc config needed */
   if ((VAR(AX0_s16_Skip_Flags) & ENC_CONFIG_NEEDED_MASK) == 0)  return ret_val;

   CalcFbFilterGain(DRIVE_PARAM);
   VAR(AX0_s16_Feedback_Commutation_Adj) = 0;

   /* set according to menctype */
   /* supported :
   * 0 - index and halls - don't skip state 1 , don't skip state 2,
   * 2 - index , no halls
   * 4 - no index , no halls - after init set to state 3
   * 5 - Halls only
   * 6 - no index , with halls - skip state 2
   * 7 - Sine cd + index
   * 8 - Sine cd
   * 9 - EnDat
   * 10 - Hiperface
   * 11 - Tamagawa
   */
   VAR(AX0_s16_Skip_Flags)  &= ~(SKIP_ENC_STATE_2_MASK | ENC_CONFIG_NEEDED_MASK);
   BGVAR(s16_DisableInputs) &= ~(ENDAT_DIS_MASK | HIFACE_DIS_MASK);
   LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Feedback_Comm_Pos);

   switch (VAR(AX0_s16_Motor_Enc_Type))
   {
      case 0:
         DetectFeedbackFaults(drive, ENC_TYPE0_FAULTS_MASK, ENC_TYPE0_FAULTS_2_MASK);
      break;

      case 1:
      case 2:
         if (VAR(AX0_s16_Motor_Enc_Type) == 2)
         {
            BGVAR(u16_PhaseFind_Bits) |= PHASE_FIND_REQUESTED_MASK; // initiate PhaseFind
            BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_SUCCESS_MASK; // initiate PhaseFind
         }

         if (BGVAR(u16_FdbkType) == SW_SINE_FDBK)
         {
            if (VAR(AX0_u16_Motor_Comm_Type))
               DetectFeedbackFaults(drive, SW_SINE_ENC_FAULTS_MASK | INDEX_BRK_FLT_MASK, 0LL);
            else
               DetectFeedbackFaults(drive, SW_SINE_TYPE2_FAULT_MASK, 0LL);
         }
         else
         {
            if (VAR(AX0_u16_Motor_Comm_Type))
               DetectFeedbackFaults(drive, INC_ENC_FAULTS_MASK | INDEX_BRK_FLT_MASK, 0LL);
            else
               DetectFeedbackFaults(drive, ENC_TYPE2_FAULTS_MASK, ENC_TYPE2_FAULTS_2_MASK);
         }
      break;

      case 3:
      case 4:// support sine encoder and encoder
      case 12:// support sine encoder and encoder
         if ((VAR(AX0_s16_Motor_Enc_Type) == 4) || (VAR(AX0_s16_Motor_Enc_Type) == 12))
         {
            BGVAR(u16_PhaseFind_Bits) |= PHASE_FIND_REQUESTED_MASK; // initiate PhaseFind
            BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_SUCCESS_MASK; // initiate PhaseFind
         }
         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;

         if (VAR(AX0_u16_Motor_Comm_Type))
         {
            if (BGVAR(u16_FdbkType) == SW_SINE_FDBK)
               DetectFeedbackFaults(drive, SW_SINE_ENC_FAULTS_MASK, 0LL);
            else
               DetectFeedbackFaults(drive, INC_ENC_FAULTS_MASK, 0LL);
         }
         else
         {
            if (BGVAR(u16_FdbkType) == SW_SINE_FDBK)
               DetectFeedbackFaults(drive, SW_SINE_TYPE4_FAULT_MASK, 0LL);
            else
               DetectFeedbackFaults(drive, ENC_TYPE4_FAULTS_MASK, ENC_TYPE4_FAULTS_2_MASK);
         }

         if (u16_Product == SHNDR_HW) BGVAR(s64_Faults_Mask) &= ~LINE_BRK_FLT_MASK;
      break;

      case 5:
         DetectFeedbackFaults(drive, ENC_TYPE5_FAULTS_MASK, ENC_TYPE5_FAULTS_2_MASK);
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Halls_Comm_Pos);
      break;

      case 6:
         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;
         if (BGVAR(u16_FdbkType) == SW_SINE_FDBK)
         {
            DetectFeedbackFaults(drive, SW_SINE_TYPE6_FAULT_MASK, DIFF_HALLS_LINE_BRK_MASK);
         }
         else if (BGVAR(u16_FdbkType) == INC_ENC_FDBK)
            DetectFeedbackFaults(drive, ENC_TYPE6_FAULTS_MASK, ENC_TYPE6_FAULTS_2_MASK);
      break;

      case 7:
         DetectFeedbackFaults(drive, SW_SINE_TYPE7_FAULT_MASK, 0LL);
      break;

      case 8:
         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;
         DetectFeedbackFaults(drive, SW_SINE_TYPE8_FAULT_MASK, 0LL);
      break;

      case 9:
         DetectFeedbackFaults(drive, SW_SINE_TYPE9_FAULT_MASK, 0LL);
         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;
      break;

      case 10:
         DetectFeedbackFaults(drive, SW_SINE_TYPE10_FAULT_MASK, 0LL);
         VAR(AX0_s16_Skip_Flags) |= SKIP_ENC_STATE_2_MASK;
      break;

      case 11:
         DetectFeedbackFaults(drive, ENC_TYPE11_FAULTS_MASK, ENC_TYPE11_FAULTS_2_MASK);
      break;
    }

   *((int*)FPGA_DIFFERENTIAL_HALL_SEL_REG_ADD) = HallsTypeSelect(BGVAR(u16_Halls_Type), drive);
   ResetEncState(DRIVE_PARAM);   // handle state machine reset
   return ret_val;
}


void SineCDInitHandler(int drive)
{ /* read CD signals and update r.t. */
   // AXIS_OFF;
   int temp;
   unsigned long u_long;
   int s16_val_0, s16_val_1;
   int drive_backup = drive;

   // since actual fbtype is not readable - do not allow both state machines to be active
   drive = 0;
   s16_val_0 = BGVAR(s16_SineCDInitState);
   drive = 1;
   s16_val_1 = BGVAR(s16_SineCDInitState);
   drive = drive_backup;

   if (u16_Num_Of_Axes > 1)
   {
      if ( (drive == 0) && (s16_val_1 > 1) ) return;
      if ( (drive == 1) && (s16_val_0 > 1) ) return;
   }

   switch (BGVAR(s16_SineCDInitState))
   {
      case 0:
      break;

      case 1:
         ++BGVAR(s16_SineCDInitState);
      break;

      case 2:// config Fbtype mux to read sine cd signals
      break;

      case 3:
      case 4:
      case 5:
         ++BGVAR(s16_SineCDInitState);
      break;

      case 6:
         temp =  VAR(AX0_s16_Atan_Value);
         if (temp < 0) temp = -temp;
         s16_SineCDCntr = 0;
         s32_SineCDAcc = 0;
         if (temp > 0x4000) BGVAR(s16_SineCDInitState) = 7;
         else  BGVAR(s16_SineCDInitState) = 8;
      break;

      case 7:
         u_long = (unsigned long)(VAR(AX0_s16_Atan_Value));
         s32_SineCDAcc += (long)(u_long);
         s16_SineCDCntr++;
         BGVAR(s16_SineCDInitState) = 9;
      break;

      case 8:
         s32_SineCDAcc += (long)(VAR(AX0_s16_Atan_Value));
         s16_SineCDCntr++;
         if (s16_SineCDCntr == 128) BGVAR(s16_SineCDInitState) = 9;
      break;

      case 9:// read atan value and calc comm_angl * mencres_2 >> 14
         //Math16 = /*(int)(s32_SineCDAcc >> 7)*/VAR(AX0_s16_Atan_Value);
         //Math16 +=0x2AAA;
         //Shifts = -15;
         //Math32 = 0;//LVAR(AX0_s32_Motor_Enc_Res_2);
         //Mult32By16to32Shifted();
         /////VAR(AX0_s16_Enc_Acc_Lo) = (int)(Math32);
         ////VAR(AX0_s16_Enc_Acc_Hi) = (int)(Math32>>16);
         ++BGVAR(s16_SineCDInitState);
      break;

      case 10:       // signal r.t. to move ahead
         VAR(AX0_s16_Sine_Enc_Bits) |= SINE_CD_INIT_DONE_MASK;
         BGVAR(s16_SineCDInitState) = 0;
      break;
   }
}


void ResetEncState(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   // don't reset enc state when in burnin
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command)) return;

   VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff);

   if (FEEDBACK_WITH_ENCODER)
   {
      switch (VAR(AX0_s16_Motor_Enc_Type))
      {
         case 4:
         case 2:
            BGVAR(u16_PhaseFind_Bits) |= PHASE_FIND_REQUESTED_MASK;
            BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_SUCCESS_MASK; // initiate PhaseFind
         case 3:
         case 1:

            VAR(AX0_s16_Power_Up_Timer) = 0x7FFF;
            if (BGVAR(u16_FdbkType) == SW_SINE_FDBK)
            {
               VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_SW_SINE_ENC_CD & 0xffff);
               VAR(AX0_s16_Sine_Enc_Bits) |= SINE_CD_INIT_DONE_MASK; // "Cheating" to complete Encoder Initialization
               VAR(AX0_s16_Sine_Enc_Bits) &= ~SW_SINE_QUAD_MISMATCH_MASK;
            }
            else if (BGVAR(u16_FdbkType) == INC_ENC_FDBK)
               VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&PHASE_FIND_STATE_0 & 0xffff);
            BGVAR(u16_Halls_Invalid_Value) = 0; // Reset Halls Validity Check Counter
            // to avoid Halls-Invalid indication before Halls are initialized
         break;

         case 0:
         case 6:
            VAR(AX0_s16_Power_Up_Timer) = 0x7FFF;
            if (BGVAR(u16_FdbkType) == SW_SINE_FDBK)
            {
               VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_SW_SINE_ENC_CD & 0xffff);
               VAR(AX0_s16_Sine_Enc_Bits) |= SINE_CD_INIT_DONE_MASK; // "Cheating" to complete Encoder Initialization
               VAR(AX0_s16_Sine_Enc_Bits) &= ~SW_SINE_QUAD_MISMATCH_MASK;
            }
            else
               VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_INC_ENC & 0xffff);
            BGVAR(u16_Halls_Invalid_Value) = 0; // Reset Halls Validity Check Counter
            // to avoid Halls-Invalid indication before Halls are initialized
         break;

         case 5:
            VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&STATE_0_HALLS_ONLY & 0xffff);
         break;

         case 7:
         case 8:
            BGVAR(s16_SineCDInitState) = 1; // sine cd init
            VAR(AX0_s16_Enc_State_Ptr) =  (int)((long)&ENC_STATE_0_SW_SINE_ENC_CD & 0xffff);
         break;

         case 9:
            BGVAR(s16_DisableInputs) |= ENDAT_DIS_MASK;
            VAR(AX0_s16_Sine_Enc_Bits) &= ~SW_SINE_QUAD_MISMATCH_MASK;

            BGVAR(s16_Endat_Init_State) = ENDAT_REVERSAL_RESET_STATE; // start endat state machine

            VAR(AX0_s16_Sine_Enc_Bits) &= ~SW_SINE_QUAD_MISMATCH_MASK;     // clear r14 indication
            VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_SW_SINE_ENC_ENDAT & 0xffff);
         break;

         case 10:
            BGVAR(s16_DisableInputs) |= HIFACE_DIS_MASK;
            VAR(AX0_s16_Sine_Enc_Bits) &= ~SW_SINE_QUAD_MISMATCH_MASK;
            BGVAR(u16_Hiface_Init_State) = HIFACE_REVERSAL_RESET_STATE;
            VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_SW_SINE_ENC_STEGMANN & 0xffff);
         break;

         case 11:
            VAR(AX0_s16_Power_Up_Timer) = 0x7FFF;
            VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_ENC_TAMAGAWA & 0xffff);
            BGVAR(u16_Halls_Invalid_Value) = 0; // Reset Halls Validity Check Counter
            // to avoid Halls-Invalid indication before Halls are initialized
         break;
      }
   }

   if ( (BGVAR(u16_FdbkType) == PS_P_G_COMM_FDBK)     ||
        (BGVAR(u16_FdbkType) == YASKAWA_INC_COMM_FDBK)  )
   {
      VAR(AX0_s16_Power_Up_Timer) = 0x7FFF;
//      VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_INC_ENC & 0xffff);
      VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_SKIP & 0xffff); //Changed to avoid Remark
   }
}


//**********************************************************
// Function Name: SalMotorEncResCommand
// Description:
//          This function is called in response to the MENCRES command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorEncResCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   float f_temp = 0.0;
   long s32_fix = 1L;
   unsigned int u16_shr = 0;
   unsigned long u32_mencres = 0;

   // PCOM Encoder simulation design
   f_temp = (float)lparam / (float)0x40000000; //32bit myltiply by 4,cuz lines.
   FloatToFixS32Shift16(&LVAR(AX0_s32_Mencres_Fix) , (unsigned int *)&VAR(AX0_u16_Mencres_Shr), f_temp);
   LVAR(AX0_u32_User_Motor_Enc_Res) = BGVAR(u32_User_Motor_Enc_Res) * 4;

   // If not changed , do nothing
   if (BGVAR(u32_User_Motor_Enc_Res) == (long)lparam)
   {
      //revert PCOM data
      LVAR(AX0_s32_Mencres_Fix) = s32_fix;
      VAR(AX0_u16_Mencres_Shr) = u16_shr;
      LVAR(AX0_u32_User_Motor_Enc_Res) = u32_mencres;
      return SAL_SUCCESS;
   }

   BGVAR(u32_User_Motor_Enc_Res) = (unsigned long)lparam;

   VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;   // reset enc_state to 0
   UpdateVelocityLimits(DRIVE_PARAM);

   //update position conversion that uses mencres
   ConvertInternalToPos1000(drive);
   ConvertPosToInternal(drive);
   ConvertCountsToInternal(drive);

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // Set no-comp fault

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalMotorEncResCommand
// Description:
//          This function is called in response to the SFBRES command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalSecEncResCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   float f_temp = 0.0;
   // If not changed , do nothing
   if (BGVAR(u32_User_Sec_Enc_Res) == (long)lparam) return SAL_SUCCESS;

    // PCOM Encoder simulation design
   f_temp = (float)lparam / (float)0x40000000; //32bit myltiply by 4,cuz lines.
   FloatToFixS32Shift16(&LVAR(AX0_s32_Sfb_Encres_Fix) , (unsigned int *)&VAR(AX0_u16_Sfb_Encres_Shr), f_temp);
   LVAR(AX0_u32_User_Sec_Enc_Res) = BGVAR(u32_User_Sec_Enc_Res) * 4;
   
   BGVAR(u32_User_Sec_Enc_Res) = (unsigned long)lparam;

   VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;   // reset enc_state to 0
   UpdateVelocityLimits(DRIVE_PARAM);

   //update position conversion that uses mencres
   ConvertInternalToPos1000(drive);
   ConvertPosToInternal(drive);
   ConvertCountsToInternal(drive);

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // Set no-comp fault

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalMotorEncInterpolationModeCommand
// Description:
//          This function is called in response to the MFBMODE command.
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorEncInterpolationModeCommand(long long param, int drive)
{
   // AXIS_OFF;

   // If not changed , do nothing
   if (BGVAR(u16_Motor_Enc_Interpolation_Mode) == (unsigned int)param) return SAL_SUCCESS;

   if ((u16_Product == SHNDR_HW) && ((unsigned int)param > 0)) return VALUE_TOO_HIGH;

   BGVAR(u16_Motor_Enc_Interpolation_Mode) = (unsigned int)param;

   VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;   // reset enc_state to 0
   UpdateVelocityLimits(DRIVE_PARAM);

   //update position conversion that uses mencres
   ConvertInternalToPos1000(drive);
   ConvertPosToInternal(drive);
   ConvertCountsToInternal(drive);

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // Set no-comp fault

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalMotorEncInterpolationCommand
// Description:
//          This function is called in response to the MFBINT command.
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorEncInterpolationCommand(long long param, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   // If not changed , do nothing
   if (BGVAR(u16_Motor_Enc_Interpolation_Override_Value) == (unsigned int)param) return SAL_SUCCESS;

   BGVAR(u16_Motor_Enc_Interpolation_Override_Value) = (unsigned int)param;

   if (BGVAR(u16_Motor_Enc_Interpolation_Override_Value) == 0)
      BGVAR(u16_Motor_Enc_Interpolation_Override) = 0;
   else
      BGVAR(u16_Motor_Enc_Interpolation_Override) = 1;

   VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;   // reset enc_state to 0

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // Set no-comp fault

   return SAL_SUCCESS;
}

int SalLoadEncInterpolationCommand(long long param, int drive)
{
   AXIS_OFF;
   REFERENCE_TO_DRIVE;
   // If not changed , do nothing
   if (BGVAR(u16_Load_Enc_Interpolation_Override_Value) == (unsigned int)param) return SAL_SUCCESS;

   BGVAR(u16_Load_Enc_Interpolation_Override_Value) = (unsigned int)param;

   if (BGVAR(u16_Load_Enc_Interpolation_Override_Value) == 0)
      BGVAR(u16_Load_Enc_Interpolation_Override) = 0;
   else
      BGVAR(u16_Load_Enc_Interpolation_Override) = 1;

   VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;   // reset enc_state to 0

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // Set no-comp fault

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalMsinintCommand
// Description:
//          This function is called in response to the MSININT command.
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalMsinintCommand(long long param, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   // If not changed , do nothing
   if (BGVAR(u16_Sine_Interpolation) == (unsigned int)param) return SAL_SUCCESS;

   BGVAR(u16_Sine_Interpolation) = (unsigned int)param;

   VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;   // reset enc_state to 0

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // Set no-comp fault

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalIndexFindCommand
// Description:
//    This function is called in response to the INDEXFIND command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalIndexFindCommand(int drive)
{
   // AXIS_OFF;
   long long u64_status = 0LL;

   if (Enabled(DRIVE_PARAM)) return DRIVE_ACTIVE;

   if ( (!FEEDBACK_WITH_ENCODER) || (BGVAR(s64_SysNotOk) & NO_COMP_FLT_MASK) || (!s16_Password_Flag) )
      return NOT_AVAILABLE;

   if ( (VAR(AX0_s16_Motor_Enc_Type) != 0) && (VAR(AX0_s16_Motor_Enc_Type) != 7) &&
        (VAR(AX0_s16_Motor_Enc_Type) != 1) && (VAR(AX0_s16_Motor_Enc_Type) != 2) &&
        (VAR(AX0_s16_Motor_Enc_Type) != 11) )
      return NOT_AVAILABLE;

   // On Tamagawa - dont allow index find before phase find is issued (since otherwise the index position accuracy will be +-30deg)
   if (VAR(AX0_s16_Motor_Enc_Type) == 11)
   {
      PhaseFindStatusCommand(&u64_status, drive);
      if (u64_status != 2) return NO_PHASE_FIND_ON_TAMAGAWA;
   }

   SalReadIndexFindStCommand(&u64_status, drive);

   // If already running dont activate again
   if (u64_status != 1LL)
   {
      EQep1Regs.QCLR.bit.IEL = 1; // Clear previous index capture indication bit
      VAR(AX0_s16_Crrnt_Run_Code) |= LOOK_FOR_INDEX_MASK;      // Set look for index bit
      VAR(AX0_s16_Crrnt_Run_Code) &= ~INDEX_CAPTURE_DONE_MASK;    // Reset index found bit

     if ( (VAR(AX0_s16_Motor_Enc_Type) != 1) && (VAR(AX0_s16_Motor_Enc_Type) != 2)  &&
          (VAR(AX0_s16_Motor_Enc_Type) != 3) && (VAR(AX0_s16_Motor_Enc_Type) != 4)  &&
          (VAR(AX0_s16_Motor_Enc_Type) != 11) && (VAR(AX0_s16_Motor_Enc_Type) != 12)  )
        ResetEncState(DRIVE_PARAM);
   }
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadIndexFindStCommand
// Description:
//          This function is called in response to the ENCINITST command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalReadIndexFindStCommand(long long* status, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if ((VAR(AX0_s16_Crrnt_Run_Code) & INDEX_CAPTURE_DONE_MASK) != 0) *status = 2LL;
   else if ((VAR(AX0_s16_Crrnt_Run_Code) & LOOK_FOR_INDEX_MASK) != 0) *status = 1LL;
   else *status = 0LL;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadIndexStCommand
// Description:
//          This function is called in response to the INDEXST command.
//       0 - enocder index signal inactive
//       1 - enocder index signal active
//
//
// Author: D.R.
// Algorithm:
// Revisions:
//**********************************************************
int SalReadIndexStCommand(long long* status, int drive)
{

   if (drive == 0)
   {
         *status = *(unsigned int*)FPGA_INDEX_CAPTURE_REG_ADD;

      *(unsigned int*)FPGA_INDEX_CAPTURE_REG_ADD = 1;
      *(unsigned int*)FPGA_INDEX_CAPTURE_REG_ADD = 0;
   }

    return SAL_SUCCESS;
}


//**********************************************************
// Function Name: MotorEncTypeCommand
// Description:
//          This function is called in response to the MENCTYPE command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMotorEncTypeCommand(long long lparam, int drive)
{
   // AXIS_OFF;

   if ( (LVAR(AX0_s32_Feedback_Ptr) == &SW_SINE_ENC_FEEDBACK)                 &&
        ( (lparam != 0) && (lparam != 1) && (lparam != 2) && (lparam != 3) &&
          (lparam != 4) && (lparam != 6) && (lparam != 9) && (lparam != 10)  && (lparam != 12) )  )
      return VALUE_OUT_OF_RANGE;

   if ( (LVAR(AX0_s32_Feedback_Ptr) == &INC_ENCODER_FEEDBACK)                 &&
        ( (lparam != 0) && (lparam != 1) && (lparam != 2) && (lparam != 3) &&
          (lparam != 4) && (lparam != 5) && (lparam != 6) && (lparam != 11)  && (lparam != 12) )  )
      return VALUE_OUT_OF_RANGE;

   if (VAR(AX0_s16_Motor_Enc_Type) == (int)lparam) return SAL_SUCCESS;

   VAR(AX0_s16_Motor_Enc_Type) = (int)lparam;
   
   // menctype&feedbacktype combination for Endat with sine signals --> requires prior configuration of 'idle' buffer  
   if ((SW_SINE_FDBK == BGVAR(u16_FdbkType)) && (9 == VAR(AX0_s16_Motor_Enc_Type)))
      PriorFeedbackConfig(0, drive);

   if ( (LVAR(AX0_s32_Feedback_Ptr) == &INC_ENCODER_FEEDBACK) ||
        (LVAR(AX0_s32_Feedback_Ptr) == &SW_SINE_ENC_FEEDBACK)   )
   {
      VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;

      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;      // set no-comp fault
   }

   return SAL_SUCCESS;
}

int SalSfbEncTypeCommand(long long lparam, int drive)
{
   AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if ( (lparam != 1) && (lparam != 3) )
      return VALUE_OUT_OF_RANGE; 
      
   if (VAR(AX0_s16_Motor_Enc_Type) == (int)lparam) return SAL_SUCCESS;
   
   if ( (LVAR(AX0_s32_Feedback_Ptr_2) == &INC_ENCODER_FEEDBACK) ||
        (LVAR(AX0_s32_Feedback_Ptr_2) == &SW_SINE_ENC_FEEDBACK)   )
   {
      VAR(AX0_s16_Sfb_Enc_Type) = (int)lparam;
      VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;

      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;      // set no-comp fault
   }

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalAqBFilterCommand
// Description:
//          This function is called in response to the MENCAQBFILT command.
//
//
// Author: Avishay
// Algorithm:
// Revisions:
//**********************************************************
int SalAqBFilterCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (BGVAR(u16_AqB_Filter) == (int)lparam) return SAL_SUCCESS;

   BGVAR(u16_AqB_Filter) = (int)lparam;

   if (LVAR(AX0_s32_Feedback_Ptr) == &INC_ENCODER_FEEDBACK)
   {
      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;      // set no-comp fault
   }

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: GetBitNumber
// Description:
//    This function is called to calculate bit number of binary word
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
unsigned int GetBitNumber(unsigned long mask)
{
   int i = 0;

   while (mask != 0)
   {
      i++;
      mask >>= 1;
   }

   return i;
}


// Convert from 0-65535 (internal elect pos) to 0-359 (degs)
int SalReadMencoff(long long *data, int drive)
{
   // AXIS_OFF;
   float f_temp = ((float)VAR(AX0_s16_Index_Elect_Pos) / 65536.0) * 360.0 + 0.5;
   REFERENCE_TO_DRIVE;

   if (f_temp > 359.5) f_temp = 0.0;

   *data = (unsigned long long)f_temp;

   return SAL_SUCCESS;
}


// Convert from 0-359 (degs) to 0-65535 (internal elect pos)
int SalWriteMencoff(long long param, int drive)
{
   // AXIS_OFF;
   
   float f_temp = ((float)param / 360.0) * 65536.0;
   REFERENCE_TO_DRIVE;
   VAR(AX0_s16_Index_Elect_Pos) = (unsigned int)f_temp;

   return SAL_SUCCESS;
}


void HallsOnlyCommSource(int drive)
{
   unsigned int u16_Halls_Thresh_Sin = (int)(32000.0 / 6.0 / (float)u16_Halls_Comm_Switch_Thresh);
   unsigned int u16_Halls_Thresh_Step = (int)(32000.0 / 6.0 / (float)u16_Halls_Comm_Switch_Thresh * 4.0 / 3.0);
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if ( (BGVAR(u16_FdbkType) == INC_ENC_FDBK)    && // FEEDBACKTYPE Incremental Encoder
        (VAR(AX0_s16_Motor_Enc_Type) == 5)         ) // MENCTYPE Halls-Only
   { // Manipulate Halls-Only Commutation source only at Relevant Feedback Setup and when Commanded
      if ( (VAR(AX0_s16_Halls_Only_Prev_Time_Cntr) > u16_Halls_Thresh_Step) || // Low Speed
           (BGVAR(u16_Halls_Only_Comm_Source) == 0)         )// HALLSONLYCOMM 0
      // If slower than 3/4 Threshold or Default Halls-Comm Source, resort to "Six-Step"
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Halls_Comm_Pos);

      if ( (VAR(AX0_s16_Halls_Only_Prev_Time_Cntr) < u16_Halls_Thresh_Sin) && // High-Speed
           (BGVAR(u16_Halls_Only_Comm_Source) == 1)         )// HALLSONLYCOMM 1
      // If faster than Threshold and Halls-Only Sine-Comm, Commutate per Exgtrapolated Position
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Halls_Only_Pos);
   }
}




