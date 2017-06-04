//###########################################################################
//
// FILE: Zeroing.c
//
// TITLE:   Dump and List commands
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.01| 03 DEC 2002 | D.R. | Creation
//
//##################################################################

#include "Current.def"
#include "Zeroing.def"
#include "ModCntrl.def"
#include "FltCntrl.def"
#include "Err_Hndl.def"
#include "PhaseFind.def"

#include "Motor.var"
#include "Drive.var"
#include "FltCntrl.var"
#include "Zeroing.var"
#include "Extrn_Asm.var"
#include "Endat.var"
#include "Ser_Comm.var"
#include "ExFbVar.var"
#include "PhaseFind.var"
#include "User_Var.var"

#include "Prototypes.pro"
#include "402fsa.def"


//**********************************************************
// Function Name: ZeroingHandler
// Description:
//        This function is called from the background to manage the Zeroing process.
//
//
// Author: Dimitry
// Algorithm:
//   During the Zeroing process the motor shaft is held in a specified position
//   in order that the feedback sensor may be positioned correctly. This
//   position is achieved by applying a constant current between two phases
//   and zero current through the third phase.
//   However, at the time zeroing is enabled the motor shaft may be in a
//   position such that the phase currents pull equally in both directions,
//   and the shaft does not move to the specified position. To overcome this
//   we first move the shaft to an intermediate position by applying current
//   through different phases. This is done when the drive is enabled.
//   This function manages this process.
// Revisions: S.F: When KCMODE>=4 to lock the motor on the same elect angle
//                 as with KCMODE=2 (this means adding a 30 elect deg offset to the location).
//**********************************************************
void ZeroingHandler(int drive)
{
   // AXIS_OFF;
   long s32_actual_velocity,s32_timeout;
   int s16_minimal_zero_legit_mov = 0, s16_feedback_comm_pos_delta = 0;

   if (BGVAR(s16_Zero_Mode) == 0) return;

   if (Enabled(DRIVE_PARAM) == 0) BGVAR(s16_Zero_State) = ZERO_IDLE_STATE;

   switch (BGVAR(s16_Zero_State))
   {
      case ZERO_IDLE_STATE:
         //if drive enabled go to next state
         if (Enabled(DRIVE_PARAM)) BGVAR(s16_Zero_State) = ZERO_INIT_STATE;
      break;

      case ZERO_INIT_STATE:
         VAR(AX0_s16_Zero_Crrnt_Cmnd) = 0;               // set crrnt cmnd to zero
         CalcZeroCurrent(drive);
         LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Zero_Crrnt_Cmnd);

         // This is to avoid accepting an analogue command during zero, will be restored on the Zero termination
         if (VAR(AX0_s16_Acc_Dec_Cmnd_Ptr) == (int)((long)&VAR(AX0_s16_Analog_Vel_Cmnd) & 0xffff))
            VAR(AX0_s16_Acc_Dec_Cmnd_Ptr) = (int)((long)&VAR(AX0_s16_Serial_Vel_Cmnd)  & 0xffff);

         BGVAR(s16_Zero_State) = ZERO_POS1_CALC_STATE;
      break;

      case ZERO_POS1_CALC_STATE:
         // Calculate the Zero position for the intermediate stage
         // A to B current is 210 electrical degrees
         VAR(AX0_s16_Skip_Flags) |= (ZERO_PHASE_ADV_MASK | MOTOR_SETUP_MASK);
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) = 38229.0;

         if (VAR(AX0_S16_Kc_Mode) >= 4) VAR(AX0_s16_Elect_Pos_With_Phase_Adv) -= 0x1555;

         BGVAR(s32_Zero_Timer) = Cntr_1mS;
         BGVAR(s16_Zero_State) = ZERO_WAIT_STATE;
      break;

      case ZERO_WAIT_STATE:
        s32_timeout = 30000L;
        // Check if phasefindmode=12 is active and use apropriate timeout
        // Use phasefindtime, but Set minimum timeout as 30 second for phasefindmode = 12
        if (((long)BGVAR(u16_PhaseFind_Time) > s32_timeout) && ((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_IN_PROCESS_MASK) == 1))
            s32_timeout = (long)BGVAR(u16_PhaseFind_Time);

        if (PassedTimeMS(s32_timeout,BGVAR(s32_Zero_Timer)))
        {
            BGVAR(s16_Zero_State) = ZERO_TIMEOUT_FAIL_STATE;
        }
        else
        {
            if ((PassedTimeMS(500L,BGVAR(s32_Zero_Timer))) &&  (2 == VAR(AX0_u16_Motor_Stopped)))
            BGVAR(s16_Zero_State) = ZERO_POS2_CALC_STATE;
        }
      break;

      case ZERO_POS2_CALC_STATE:

         BGVAR(s16_Feedback_Comm_Pos_Temp) = VAR(AX0_s16_Feedback_Comm_Pos);
         // Calculate the Zero position for the permanent stage: C to B current is
         // 270 electrical degrees
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) = 49152.0;

         if (VAR(AX0_S16_Kc_Mode) >= 4) VAR(AX0_s16_Elect_Pos_With_Phase_Adv) -= 0x1555;

         BGVAR(s32_Zero_Timer) = Cntr_1mS;
         BGVAR(s32_Zero_Timer2) = Cntr_1mS;
         BGVAR(s16_Zero_State) = ZERO_WAIT_FOR_STOP_STATE;
      break;

      case ZERO_WAIT_FOR_STOP_STATE: // Wait for the velocity to stablalize or timeout
         s32_timeout = 30000L;
         // Check if phasefindmode=12 is active and use apropriate timeout
         // Use phasefindtime, but Set minimum timeout as 30 second for phasefindmode = 12
         if (((long)BGVAR(u16_PhaseFind_Time) > s32_timeout) && ((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_IN_PROCESS_MASK) == 1))
            s32_timeout = (long)BGVAR(u16_PhaseFind_Time);

         if (PassedTimeMS(s32_timeout,BGVAR(s32_Zero_Timer)))
         {
            //BGVAR(s16_Zero_State) = ZERO_POS2_CALC_STATE;
            BGVAR(s16_Zero_State) = ZERO_TIMEOUT_FAIL_STATE;
         }
         else
         {
            s32_actual_velocity = LVAR(AX0_s32_Vel_Var_Fb_0);
            if (s32_actual_velocity < 0L) s32_actual_velocity = -s32_actual_velocity;

            if (s32_actual_velocity > 150000L) BGVAR(s32_Zero_Timer2) = Cntr_1mS;
            if (PassedTimeMS(1500L,BGVAR(s32_Zero_Timer2)) &&  (2 == VAR(AX0_u16_Motor_Stopped)))
               BGVAR(s16_Zero_State) = ZERO_STOPPED_STATE;
         }
      break;

      case ZERO_STOPPED_STATE:
         // Calculate the suggested MPHASE as the diff between the known zero elect angle
         // and the elect angle as resolved from the feedback
         s16_minimal_zero_legit_mov = 65535 / (12 * VAR(AX0_s16_Num_Of_Poles_Ratio));  //65535 * (30/60) /  (mpoles/2)
         s16_feedback_comm_pos_delta = VAR(AX0_s16_Feedback_Comm_Pos) - BGVAR(s16_Feedback_Comm_Pos_Temp);
         if (0 > s16_feedback_comm_pos_delta)
            s16_feedback_comm_pos_delta = -s16_feedback_comm_pos_delta;

         if (s16_minimal_zero_legit_mov > s16_feedback_comm_pos_delta)
         {
            BGVAR(s16_Zero_State) = ZERO_MOVEMENT_FAIL_STATE;
         }
         else
         {
            BGVAR(s16_Zero_Calc_Mphase) = VAR(AX0_s16_Elect_Pos_With_Phase_Adv) + 0x4000 - VAR(AX0_s16_Elect_Pos_Prev);

            if(BGVAR(s16_Direction) != 0) // Add 180deg to calculated mphase if DIR<>0
               BGVAR(s16_Zero_Calc_Mphase) += 0x8000;

            BGVAR(s16_Zero_State) = ZERO_DONE_STATE;
         }
      break;

      case ZERO_DONE_STATE:
      case ZERO_MOVEMENT_FAIL_STATE:
      case ZERO_TIMEOUT_FAIL_STATE:
      default:

      break;
   }
}

void ZeroingTermination(int drive)
{
   // AXIS_OFF;

   BGVAR(s16_Zero_Mode) = 0;
   BGVAR(s16_Zero_State) = ZERO_IDLE_STATE;
   VAR(AX0_s16_Skip_Flags) &= ~(ZERO_PHASE_ADV_MASK | MOTOR_SETUP_MASK);

   // If ZERO is used from PHASEFIND then restore current pointer only
   if ((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_IN_PROCESS_MASK) != 0)
   {
      SetCurrLoopPtr(drive,VAR(AX0_s16_Opmode),1);
      VAR(AX0_s16_PhaseFindRTBits) &= ~(PHASE_FIND_SET_PE_0_MASK);
   }
   else
   {
       SetOpmode(drive,VAR(AX0_s16_Opmode));

       //set back Fieldbus related parametrs
       if((BGVAR(s16_CAN_Opmode) == CYCLIC_SYNCHRONOUS_POSITION_MODE) ||
          (BGVAR(s16_CAN_Opmode) == CYCLIC_SYNCHRONOUS_VELOCITY_MODE) ||
          (BGVAR(s16_CAN_Opmode) == INTRPOLATED_POSITION_MODE) ||
          (BGVAR(s16_CAN_Opmode) == CYCLIC_SYNCHRONOUS_TORQUE_MODE))
       {
          FalSetFBSyncOpmode(drive, BGVAR(s16_CAN_Opmode));
       }
   }
}

void CalcZeroCurrent(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   VAR(AX0_s16_Zero_Crrnt_Cmnd) = (int)((float)BGVAR(u32_Zero_Current)*26214.0/(float)BGVAR(s32_Drive_I_Peak));
   // Use PHASEFINDI if PHASEFIND is active and PHASEFINDMODE = 12
   if ((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_IN_PROCESS_MASK) != 0)
   {
       // Set IZERO = PHASEFINDI
       VAR(AX0_s16_Zero_Crrnt_Cmnd) = (int)((float)BGVAR(s32_PhaseFind_Current)*26214.0/(float)BGVAR(s32_Drive_I_Peak));
       // if PHASEFINDI > IMAX then IZERO = IMAX
       if (BGVAR(s32_PhaseFind_Current) > BGVAR(s32_Imax))
            VAR(AX0_s16_Zero_Crrnt_Cmnd) = (int)((float)BGVAR(s32_Imax)*26214.0/(float)BGVAR(s32_Drive_I_Peak));
   }
}

//**********************************************************
// Function Name: ZeroCommand
// Description:
//          This function is called in response to the ZERO command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalZeroCommand(long long lparam,int drive)
{
   
   if ((0LL == lparam) && (0L == BGVAR(s32_Ilim_User))) return (SAL_SUCCESS);
   
    // Permit to use Ilim=0 for Phasefind process
   if ((0L == BGVAR(s32_Ilim_User)) && ((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_IN_PROCESS_MASK) == 0))
   {
      return (ZERO_FAIL_ILIM_ZEROED);
   }

   if ((BGVAR(s64_SysNotOk) & NO_COMP_FLT_MASK) != 0) return (NOT_AVAILABLE);

   if (lparam == 0LL) ZeroingTermination(drive);
   else BGVAR(s16_Zero_Mode) = (int)lparam;

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: IzeroCommand
// Description:
//          This function is called in response to the IZERO command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalIzeroCommand(long long lparam,int drive)
{
   if (lparam > (long long)BGVAR(s32_Drive_I_Peak)) return (VALUE_TOO_HIGH);

   BGVAR(u32_Zero_Current) = (long)lparam;
   CalcZeroCurrent(drive);

   return (SAL_SUCCESS);
}

int ZeroStatusCommand(int drive)
{
   int s16_mphase_degs = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (u8_Output_Buffer_Free_Space < 120) return SAL_NOT_FINISHED;

   if (BGVAR(s16_Zero_State) == ZERO_IDLE_STATE) PrintStringCrLf("Zero did not run yet", 0);
   else if (BGVAR(s16_Zero_State) == ZERO_DONE_STATE)
   {
      PrintString("Zero Ended, MPHASE = ", 0);

	  s16_mphase_degs = CalcZeroMphaseDegs(drive);

      PrintUnsignedInt16(s16_mphase_degs);
      PrintCrLf();
      if (BGVAR(u16_Sys_Remarks) & (HALL_NOT_FOUND_RMK_MASK | INDEX_NOT_FOUND_RMK_MASK))
      {
         PrintStringCrLf("Better to run ZERO after Index found/Halls switch", 0);
      }
   }

   else if (ZERO_MOVEMENT_FAIL_STATE == (BGVAR(s16_Zero_State)))
   {
      PrintStringCrLf("Zero Fail: Insufficient movement. Check sufficient ILIM and IZERO, then re-try", 0);
   }
   else if (ZERO_TIMEOUT_FAIL_STATE == (BGVAR(s16_Zero_State)))
   {
      PrintStringCrLf("Zero Fail: Motor Settling Timeout", 0);
   }

   else PrintStringCrLf("Running", 0);

   return SAL_SUCCESS;
}

int CalcZeroMphaseDegs(int drive)
{
   // AXIS_OFF;
   int s16_mphase_degs = 0;
   REFERENCE_TO_DRIVE;

   if (BGVAR(s16_Zero_State) == ZERO_DONE_STATE)
   {
      // Convert mphase to degs
      if (VAR(AX0_S16_Kc_Mode) >= 4)
         s16_mphase_degs = 0x1555;

      s16_mphase_degs +=  BGVAR(s16_Zero_Calc_Mphase) + 91; // Round the mphase value
      s16_mphase_degs = (int)((float)s16_mphase_degs / 65536.0 * 360.0);
      if (s16_mphase_degs == 360) s16_mphase_degs = 0;
      if (s16_mphase_degs < 0) s16_mphase_degs = 360 + s16_mphase_degs;
   }
   else//not started or running or failed
   {
      s16_mphase_degs = -1;
   }

   return s16_mphase_degs;
}
