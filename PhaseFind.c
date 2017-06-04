#include <string.h>
#include "co_util.h"
#include "co_type.h"
#include "access.h"
#include "objects.h"

#include "DSP2834x_Device.h"
#include "Math.h"

#include "MultiAxis.def"
#include "PhaseFind.def"
#include "Err_Hndl.def"
#include "FltCntrl.def"
#include "Fpga.def"
#include "Exe_IO.def"
#include "Design.def"
#include "Zeroing.def"
#include "MotorParamsEst.def"

#include "Extrn_Asm.var"
#include "PhaseFind.var"
#include "FltCntrl.var"
#include "Drive.var"
#include "Motor.var"
#include "Exe_IO.var"
#include "Units.var"
#include "MotorSetup.var"
#include "Ser_Comm.var"
#include "SensorlessBG.var"
#include "ExFbVar.var"
#include "Zeroing.var"
#include "User_Var.var"
#include "Foldback.var"
#include "MotorParamsEst.var"

#include "Prototypes.pro"
#include "402fsa.def"

int debug_phase_findvar, pf_debug = 0;
float f_scale_factor = 65536.0 / 6.0 / 25108.0;
void PhaseFindHandler(int drive)
{
   // AXIS_OFF;
   unsigned char u8_max_phase;
   int s16_a_diff, s16_b_diff, s16_c_diff, s16_a_abs, s16_b_abs, s16_c_abs, s16_var1, s16_var2;
   int s16_elect_phase, s16_phase_find_captured_ib_abs, s16_phase_find_max, s16_temp;
   static int prev_fault_state = 0, s16_mode3support = 1;
   long s32_time_capture;
   unsigned int u16_update_done = 0, u16_mipeak = 0, u16_ipeak = 0;
   long long s64_temp;
   static unsigned int  u16_ignore_remote_en = 1, u16_remote_en_backup = 0;

   // If motor setup or burning running avoid running the PhaseFind
   if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_ACTIVE) return;
   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command)) return;
   if (BGVAR(u16_PhaseFind_Mode) == 3)//dbgSS
   {
      s16_temp = CalculateSaliency(drive);
      if (s16_temp != SAL_SUCCESS) return;

      /*
      if (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_FAULT)
      {
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE;
      }
      */
      if (BGVAR(s16_Motor_Params_Est_State) < 0) //motor params est fault
      {
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE;
      }

      if (BGVAR(s32_Param_Est_SLfactor) > 850L)
      {
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE;
      }
   }
   if (!Enabled(drive))
   {
      if ( ( (BGVAR(u8_PhaseFind_State) != PHASE_FIND_IDLE)            &&
             (BGVAR(u8_PhaseFind_State) != PHASE_FIND_DONE_STATE)      &&
             (BGVAR(u8_PhaseFind_State) != PHASE_FIND_WAIT_FOR_EN)     &&
             (BGVAR(u8_PhaseFind_State) != PHASE_FIND_WAIT_FOR_DIS)    &&
             (BGVAR(u8_PhaseFind_State) != PHASE_FIND_SMOOTH_WAIT_EN)  &&
             (BGVAR(u8_PhaseFind_State) != PHASE_FIND_SMOOTH_WAIT_DIS) &&
             (BGVAR(u8_PhaseFind_State) != PHASE_FIND_ZERO_START)      &&
             (BGVAR(u8_PhaseFind_State) != PHASE_FIND_ZERO_FINAL_DIS)    )    ||
           ( (!prev_fault_state)                                           &&
             ( ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0)    ||
               ((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0)  )  )  )
      {
         if ( (VAR(AX0_s16_Motor_Enc_Type) == 1) || (VAR(AX0_s16_Motor_Enc_Type) == 3) ||
              (VAR(AX0_s16_Motor_Enc_Type) == 11) || (BGVAR(u8_Sl_Mode) != 0)            )
            BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;

         VAR(AX0_s16_PhaseFindRTBits)= 0;
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_IDLE;
         BGVAR(u16_PhaseFindSmooth_RT_Mode)  =  PH_FND_SM_RT_MODE_IDLE_MASK; // Set RT to Idle and reset other bits
         VAR(AX0_s16_Skip_Flags) &= ~(ZERO_PHASE_ADV_MASK | SKIP_COMM_ADJ_MASK | WNS_ON_MASK);
         BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_SUCCESS_MASK;
         if ( (BGVAR(u16_PhaseFind_Mode) == 2) || (BGVAR(u16_PhaseFind_Mode) == 3) ||
              (BGVAR(u16_PhaseFind_Mode) == 9) || (BGVAR(u16_PhaseFind_Mode) == 22)  )
            BGVAR(u16_WnsError) |= WNS_STOPPED;
      }
   }
   prev_fault_state = ( ((BGVAR(s64_SysNotOk) & FEEDBACK_LOSS_FLT_MASK) != 0)    ||
                        ((BGVAR(s64_SysNotOk_2) & FEEDBACK_LOSS_FLT_2_MASK) != 0)  );

   if ( ( (VAR(AX0_s16_Motor_Enc_Type) < 1) || (VAR(AX0_s16_Motor_Enc_Type) > 4) )   &&
          (VAR(AX0_s16_Motor_Enc_Type) != 11) && (VAR(AX0_s16_Motor_Enc_Type) != 12) &&
          (BGVAR(u8_Sl_Mode) == 0) && (BGVAR(u16_FdbkType) != FANUC_COMM_FDBK)       &&
          ((BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_REQUESTED_MASK) != 0)               )
   {
      BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;
      VAR(AX0_s16_Skip_Flags) &= ~(ZERO_PHASE_ADV_MASK | SKIP_COMM_ADJ_MASK | WNS_ON_MASK);
      // Resetting all Phase-Find related Bits to avoid Phase-Find processes
      VAR(AX0_s16_PhaseFindRTBits)= 0;
      LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Feedback_Comm_Pos);
      // Returning Commutation Source to Position.
      return;
   }

   switch (BGVAR(u8_PhaseFind_State))
   {
      case PHASE_FIND_IDLE:  // If inactive and phase_find required update state
         if (VAR(AX0_s16_Ignore_Ilim)) VAR(AX0_s16_Ignore_Ilim) = 0;

         VAR(AX0_s16_PhaseFindRTBits) &= ~PHASE_FIND_SET_PE_0_MASK;

         if ((!Enabled(drive)) && (BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_REQUESTED_MASK) && !(BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_SUCCESS_MASK))
         {
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_WAIT_FOR_EN;
            if((BGVAR(u16_PhaseFind_Mode) == 4) || (BGVAR(u16_PhaseFind_Mode)==5) || (BGVAR(u16_PhaseFind_Mode) == 12))
            {
                BGVAR(u16_WnsError) &= ~(WNS_DESIGN_FAILED|WNS_STOPPED|WNS_FAILED_MASK|MAX_VEL_ERROR|TOO_MUCH_MOTION|MOTION_PROFILE|TOO_LITTLE_MOTION|WNS_RAN|TOO_MUCH_OSC|SETTLE_TIMEOUT|FOLDBACK_OCCURED);
                VAR(AX0_s16_Wns_Crrnt_Cmnd) = 0;
                LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Wns_Crrnt_Cmnd);
                BGVAR(u16_Motor_Setup_Allow_Enable) = 1; //dbgSS - allow enable
                VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&PHASE_FIND_STATE_0 & 0xffff);
                do {
                   s16_temp = Cntr_3125;
                   LLVAR(AX0_u32_Wns_Pfb_0_Lo) = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
                } while (s16_temp != Cntr_3125);
            }

            if ( ((BGVAR(u16_PhaseFind_Mode) == 2) || (BGVAR(u16_PhaseFind_Mode) == 9)) && (BGVAR(u8_Sl_Mode) == 0) || (BGVAR(u16_PhaseFind_Mode) == 22) )
            {
               VAR(AX0_u16_Wns_Status) &= ~WNS_ENDED_MASK;  // clr ended bit
               BGVAR(u16_WnsError) &= ~(WNS_DESIGN_FAILED|WNS_STOPPED|WNS_FAILED_MASK|MAX_VEL_ERROR|TOO_MUCH_MOTION|MOTION_PROFILE|TOO_LITTLE_MOTION|WNS_RAN|TOO_MUCH_OSC|SETTLE_TIMEOUT|FOLDBACK_OCCURED);
               VAR(AX0_s16_Wns_Crrnt_Cmnd) = 0;           // set crrnt cmnd due to wns to zero
               VAR(AX0_s16_Skip_Flags) |= (ZERO_PHASE_ADV_MASK | SKIP_COMM_ADJ_MASK | WNS_ON_MASK);//skip phase adv
               BGVAR(u16_WnsRetry) = 3;

               if (s16_mode3support == 2) EnableCommand(drive); // Enable in phasefindmode 3 only
               BGVAR(u16_Motor_Setup_Allow_Enable) = 1; //dbgSS - allow enable
               LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Wns_Crrnt_Cmnd);
               if (BGVAR(u16_PhaseFind_Mode) == 9)  VAR(AX0_u16_Wns_Status)|= WNS_SMOOTH_MODE_MASK;
               if (BGVAR(u16_PhaseFind_Mode) == 22)
               {
                  VAR(AX0_s16_Wns_Dir_Angle) = (int)((float)BGVAR(u16_PhaseFind_Delta) * 65535.0 / 359);
                  VAR(AX0_u16_Wns_Status)|= WNS_TUNING_MODE_MASK; // Allow infite iteration (use "k" for stopping)
                  VAR(AX0_u16_Wns_Status) |= WNS_SKIP_SEC_ITER_MASK; // Issue one wns iteration only
               }

               LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Wns_Comm);
            }
            else if ((BGVAR(u16_PhaseFind_Mode) == 4) || (BGVAR(u16_PhaseFind_Mode) == 5))
            {
               VAR(AX0_s16_Skip_Flags) |= (ZERO_PHASE_ADV_MASK | SKIP_COMM_ADJ_MASK | WNS_ON_MASK);//skip phase adv
               LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Wns_Comm);
               BGVAR(u16_PhaseFindSmooth_RT_Mode)  =  PH_FND_SM_RT_MODE_IDLE_MASK; // // Set RT to Idle and reset other bits
            }
            else if ((BGVAR(u16_PhaseFind_Mode) == 11) || (BGVAR(u16_PhaseFind_Mode) == 12))
            {
               VAR(AX0_s16_Feedback_Commutation_Adj) = 0;
            }
            else
            {
               VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&PHASE_FIND_STATE_0 & 0xffff);
               VAR(AX0_s16_PhaseFindRTBits) |= PHASE_FIND_ACTIVATE_MASK; // signal r.t to use phase find when dis->en transtion occurs
               BGVAR(f_Pulse_Factor) = 1.0;
            }
            //if (BGVAR(u16_PhaseFind_Mode) == 0) EnableCommand(drive);
         }
      break;

      case PHASE_FIND_WAIT_FOR_EN:
         if (!Enabled(drive))
         {
            // On power up ignore remote enable input to perform phase find
            if ((u16_ignore_remote_en == 1) && (VAR(AX0_s16_Motor_Enc_Type) == 12))
            {
               u16_remote_en_backup = IsInFunctionalityConfigured(REMOTE_ENABLE_INP, drive);
               UpdateIndexFunc(drive, 0, u16_remote_en_backup);
               u16_ignore_remote_en = 2;
            }
            break;
         }

         VAR(AX0_s16_PhaseFindRTBits) |= PHASE_FIND_SET_PE_0_MASK; // This is to avoid calculating and refering to PE during PHASEFIND process

         if (!Enabled(drive)) break;
         // Dispatch state machine according to PHASEFINDMODE
         if ((BGVAR(u16_PhaseFind_Mode) == 0) || (BGVAR(u16_PhaseFind_Mode) == 3) || (BGVAR(u8_Sl_Mode) != 0))// Voltage pulse
         {
            VAR(AX0_s16_Skip_Flags) |= ZERO_PHASE_ADV_MASK;
            CalculatePulseDuration(drive);
            if (!GeneratePhaseFindPulse(drive, PHASE_FIND_NEG_PULSE, PHASE_FIND_PHASE_A))
               {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

            BGVAR(u8_PhaseFind_State) = PHASE_FIND_IDENTIFY_MAX_PULSE;
         }
         else if (BGVAR(u16_PhaseFind_Mode) == 11) // Force commutation value
         {
            VAR(AX0_u16_PhaseFind_EPos) = BGVAR(u16_PhaseFindEPos);
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_UPDATE_COMM;
         }

         else if (BGVAR(u16_PhaseFind_Mode) == 12) // Force commutation (ZERO method)
         {
            VAR(AX0_s16_Ignore_Ilim) = 1; // Ignore ILIM during phase find
            BGVAR(u16_PhaseFind_Bits) |= PHASE_FIND_IN_PROCESS_MASK;
            // Init timer
            BGVAR(s32_PhaseFind_Timer) = Cntr_1mS;
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_ZERO_START;
         }
         else if ((BGVAR(u16_PhaseFind_Mode) == 2) || (BGVAR(u16_PhaseFind_Mode) == 9) || (BGVAR(u16_PhaseFind_Mode) == 22)) // WNS
         {
           if (CalculateWnsGains(DRIVE_PARAM))
           {
              BGVAR(u16_WnsError) |= WNS_DESIGN_FAILED;
              BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE;
              break;
            }
            
            // phasefindmode = 4, allow smooth mode WNS algorithm
            if (BGVAR(u16_PhaseFind_Mode) == 9)  VAR(AX0_u16_Wns_Status)|= WNS_SMOOTH_MODE_MASK;
            // phasefindmode = 22, tuning mode,same as mode 2, Allow infite iteration (use "k" for stopping)
            if (BGVAR(u16_PhaseFind_Mode) == 22)
            {
               VAR(AX0_u16_Wns_Status)|= WNS_TUNING_MODE_MASK; // Allow infite iteration (use "k" for stopping)
               VAR(AX0_u16_Wns_Status) |= WNS_SKIP_SEC_ITER_MASK; // Issue one wns iteration only
            }
            LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Wns_Comm);

            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Wns_Crrnt_Cmnd);

            VAR(AX0_s16_Ignore_Ilim) = 1; // Ignore ILIM during phase find (request from Dima)
            if (s16_mode3support != 2)
            {
               VAR(AX0_s16_Wns_Comm) = 0; //zero start commutation value if phasefindmode != 3
               s16_mode3support = 0;
            }
            VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_WNS_0 & 0xffff);     // Start r.t.
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_WNS_WAIT_FOR_DONE;
         }
         else if ((BGVAR(u16_PhaseFind_Mode) == 4) || (BGVAR(u16_PhaseFind_Mode) == 5))
         {
            // Set Limit to MICONT
            VAR(AX0_s16_Ignore_Ilim) = 1; // Ignore ILIM during phase find (request from Dima)

            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Wns_Crrnt_Cmnd);

            // Use Imax if PHASEFINDMODE = 5, else use MICONT
            if (BGVAR(u16_PhaseFind_Mode) == 5)
            VAR(AX0_s16_Phase_Find_Current) = (int)((BGVAR(s32_Imax)*26214L) / BGVAR(s32_Drive_I_Peak));
            else
            VAR(AX0_s16_Phase_Find_Current) = (int)((BGVAR(s32_Motor_I_Cont)*26214L) / BGVAR(s32_Drive_I_Peak));

            if(VAR(AX0_s16_Phase_Find_Current) > 26214L) VAR(AX0_s16_Phase_Find_Current) = 26214L;
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_SMOOTH_WAIT_FOR_DONE;
            BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_START;
         }
      break;

      case PHASE_FIND_SMOOTH_WAIT_FOR_DONE:
         if (!PhaseFindSmooth(drive)) break;

         VAR(AX0_s16_Skip_Flags) &= ~(ZERO_PHASE_ADV_MASK | SKIP_COMM_ADJ_MASK | WNS_ON_MASK);
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Feedback_Comm_Pos);
         VAR(AX0_s16_Serial_Crrnt_Cmnd) = 0;
         if (VAR(AX0_s16_Ignore_Ilim)) VAR(AX0_s16_Ignore_Ilim) = 0;
         DisableCommand(drive);
         PhaseFindCurrentCommand((long long)BGVAR(s32_PhaseFind_Current), drive); // restore original phasefindi
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_SMOOTH_WAIT_DIS;
      break;

      case PHASE_FIND_SMOOTH_WAIT_DIS:
         if (Enabled(drive)) break;
         SetOpmode(drive, VAR(AX0_s16_Opmode));

         //set back Fieldbus related parametrs
         if ( (BGVAR(s16_CAN_Opmode) == CYCLIC_SYNCHRONOUS_POSITION_MODE) ||
              (BGVAR(s16_CAN_Opmode) == CYCLIC_SYNCHRONOUS_VELOCITY_MODE) ||
              (BGVAR(s16_CAN_Opmode) == INTRPOLATED_POSITION_MODE)        ||
              (BGVAR(s16_CAN_Opmode) == CYCLIC_SYNCHRONOUS_TORQUE_MODE)     )
         {
            FalSetFBSyncOpmode(drive, BGVAR(s16_CAN_Opmode));
         }

         EnableCommand(drive);
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_SMOOTH_WAIT_EN;
         if (BGVAR(u16_WnsError) & WNS_FAILED_MASK)
         {
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE;
//            BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;
         }
      break;

      case PHASE_FIND_SMOOTH_WAIT_EN:
         if (!Enabled(drive)) break;
         BGVAR(u16_Motor_Setup_Allow_Enable) = 0; //dbgSS - allow enable
         BGVAR(u16_WnsError) |= WNS_RAN;
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_UPDATE_COMM;
      break;

      case PHASE_FIND_IDENTIFY_MAX_PULSE:
         s16_phase_find_captured_ib_abs = -VAR(AX0_s16_PhaseFind_Captured_Ia) - VAR(AX0_s16_PhaseFind_Captured_Ic);
         s16_phase_find_captured_ib_abs = abs(s16_phase_find_captured_ib_abs);
         s16_phase_find_max = abs(VAR(AX0_s16_PhaseFind_Captured_Ia));
         if (abs(VAR(AX0_s16_PhaseFind_Captured_Ic)) > s16_phase_find_max)
            s16_phase_find_max = abs(VAR(AX0_s16_PhaseFind_Captured_Ic));
         if (s16_phase_find_captured_ib_abs > s16_phase_find_max)
            s16_phase_find_max = s16_phase_find_captured_ib_abs;

         // Take the minimum between 0.7*MIPEAK and 0.7*(DIPEAK/2) - to avoid OC and damaging the motor
         u16_mipeak = (BGVAR(s32_Motor_I_Peak) * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
             >> BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
         u16_mipeak = (unsigned int)(((unsigned long)u16_mipeak * 7L) / 10L);

         u16_ipeak = (BGVAR(s32_Drive_I_Peak) * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
             >> BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
         u16_ipeak = u16_ipeak >> 1;
         u16_ipeak = (unsigned int)(((unsigned long)u16_ipeak * 7L) / 10L);

         if (BGVAR(s32_Motor_I_Peak) < BGVAR(s32_Drive_I_Peak)) u16_ipeak = u16_mipeak;

         if (s16_phase_find_max > u16_ipeak) // = 0.7*0x3333 assume ml varies with position 30%
         {
            if (!GeneratePhaseFindPulse(drive,PHASE_FIND_IDLE_PULSE,PHASE_FIND_IDLE_PULSE))
               {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

            BGVAR(u8_PhaseFind_State) = PHASE_FIND_START_INJECTING_PULSES;
            break;
         }

         // current is too small - increase injection
         if (BGVAR(f_Pulse_Factor) == 1.0)
         { // Estimate the injection on the first try according to the previous resulted current
            BGVAR(f_Pulse_Factor) = ((float)u16_ipeak / (float)s16_phase_find_max) * 0.75;
         }
         else
            BGVAR(f_Pulse_Factor) = BGVAR(f_Pulse_Factor) * 1.05;

         CalculatePulseDuration(drive);
         if (BGVAR(u16_PhaseFind_Pulse_Duration) >= (unsigned int)45000) // the limit here is less than 65535 - as the injection on other phases might cause higher current
         {
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_START_INJECTING_PULSES;
            break;
         }
         else
         {
            if (!GeneratePhaseFindPulse(drive, PHASE_FIND_IDLE_PULSE, PHASE_FIND_IDLE_PULSE))
               {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

            if (!GeneratePhaseFindPulse(drive, PHASE_FIND_NEG_PULSE, PHASE_FIND_PHASE_A))
               {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}
         }
      break;

      case PHASE_FIND_START_INJECTING_PULSES:
         if (!GeneratePhaseFindPulse(drive, PHASE_FIND_POS_PULSE, PHASE_FIND_PHASE_A))
            {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

         BGVAR(s16_PhaseFind_A_Pos_Crrnt) = VAR(AX0_s16_PhaseFind_Captured_Ia);

         if (!GeneratePhaseFindPulse(drive, PHASE_FIND_IDLE_PULSE, PHASE_FIND_IDLE_PULSE))
            {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

         BGVAR(u8_PhaseFind_State) = PHASE_FIND_PHASE_A_NEG;
      break;

      case PHASE_FIND_PHASE_A_NEG:
         if (!GeneratePhaseFindPulse(drive, PHASE_FIND_NEG_PULSE, PHASE_FIND_PHASE_A))
            {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

         BGVAR(s16_PhaseFind_A_Neg_Crrnt) = VAR(AX0_s16_PhaseFind_Captured_Ia);

         if (!GeneratePhaseFindPulse(drive, PHASE_FIND_IDLE_PULSE, PHASE_FIND_IDLE_PULSE))
            {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

         BGVAR(u8_PhaseFind_State) = PHASE_FIND_PHASE_B_POS;
      break;

      case PHASE_FIND_PHASE_B_POS:
         if (!GeneratePhaseFindPulse(drive, PHASE_FIND_POS_PULSE, PHASE_FIND_PHASE_B))
            {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

         BGVAR(s16_PhaseFind_B_Pos_Crrnt) = -VAR(AX0_s16_PhaseFind_Captured_Ia) - VAR(AX0_s16_PhaseFind_Captured_Ic);

         if (!GeneratePhaseFindPulse(drive, PHASE_FIND_IDLE_PULSE, PHASE_FIND_IDLE_PULSE))
            {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

         BGVAR(u8_PhaseFind_State) = PHASE_FIND_PHASE_B_NEG;
      break;

      case PHASE_FIND_PHASE_B_NEG:
         if (!GeneratePhaseFindPulse(drive, PHASE_FIND_NEG_PULSE, PHASE_FIND_PHASE_B))
            {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

         BGVAR(s16_PhaseFind_B_Neg_Crrnt) = -VAR(AX0_s16_PhaseFind_Captured_Ia) - VAR(AX0_s16_PhaseFind_Captured_Ic);

         if (!GeneratePhaseFindPulse(drive, PHASE_FIND_IDLE_PULSE, PHASE_FIND_IDLE_PULSE))
            {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

         BGVAR(u8_PhaseFind_State) = PHASE_FIND_PHASE_C_POS;
      break;

      case PHASE_FIND_PHASE_C_POS:
         if (!GeneratePhaseFindPulse(drive, PHASE_FIND_POS_PULSE, PHASE_FIND_PHASE_C))
            {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

         BGVAR(s16_PhaseFind_C_Pos_Crrnt) = VAR(AX0_s16_PhaseFind_Captured_Ic);

         if (!GeneratePhaseFindPulse(drive, PHASE_FIND_IDLE_PULSE, PHASE_FIND_IDLE_PULSE))
            {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

         BGVAR(u8_PhaseFind_State) = PHASE_FIND_PHASE_C_NEG;
      break;

      case PHASE_FIND_PHASE_C_NEG:
         if (!GeneratePhaseFindPulse(drive, PHASE_FIND_NEG_PULSE, PHASE_FIND_PHASE_C))
            {BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE; break;}

         BGVAR(s16_PhaseFind_C_Neg_Crrnt) = VAR(AX0_s16_PhaseFind_Captured_Ic);
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_CALC_POS_FROM_PULSES;
      break;

      case PHASE_FIND_CALC_POS_FROM_PULSES:
         s16_a_diff = BGVAR(s16_PhaseFind_A_Pos_Crrnt) - BGVAR(s16_PhaseFind_A_Neg_Crrnt);
         s16_b_diff = BGVAR(s16_PhaseFind_B_Pos_Crrnt) - BGVAR(s16_PhaseFind_B_Neg_Crrnt);
         s16_c_diff = BGVAR(s16_PhaseFind_C_Pos_Crrnt) - BGVAR(s16_PhaseFind_C_Neg_Crrnt);

         s16_a_abs = abs(s16_a_diff);
         s16_b_abs = abs(s16_b_diff);
         s16_c_abs = abs(s16_c_diff);

         if ( (s16_a_abs >  s16_b_abs) && (s16_a_abs > s16_c_abs) )
         {
            s16_var1 = s16_c_diff;
            s16_var2 = s16_b_diff;
            u8_max_phase = 1;
         }
         else if ( (s16_b_abs >  s16_a_abs) && (s16_b_abs > s16_c_abs) )
         {
            s16_var1 = s16_a_diff;
            s16_var2 = s16_c_diff;
            u8_max_phase = 2;
         }
         else // phase c diff is the largest
         {
            s16_var1 = s16_a_diff;
            s16_var2 = s16_b_diff;
            u8_max_phase = 3;
         }

         // calc 2*32768/pi*(cos(2*pi/3)/sin(2*pi/3)*(var1-var2)/(var1+var2))
         // cos(2*pi/3) = -0.5 ,  sin(2*pi/3) = 0.866
         // const = -0.5/0.866 * 65536/pi = -12044.32
         s16_elect_phase = (int)(-12044.32 * ((float)s16_var1 - (float)s16_var2) / ((float)s16_var1 + (float)s16_var2));

         // now shift the result according to its place in the 360 degrees circle
         debug_phase_findvar = -1;
         if (VAR(AX0_s16_Skip_Flags) & SWAP_UV_COMM_MASK)
         {
            debug_phase_findvar = 6;
            if ( (u8_max_phase == 1) && ((BGVAR(s16_PhaseFind_A_Pos_Crrnt) + BGVAR(s16_PhaseFind_A_Neg_Crrnt)) > 0) )
            {
               debug_phase_findvar = 1;
               s16_elect_phase += 27306;    // +150 0deg
            }
            else if ( (u8_max_phase == 3) && ((BGVAR(s16_PhaseFind_C_Pos_Crrnt) + BGVAR(s16_PhaseFind_C_Neg_Crrnt)) < 0) )
            {
               debug_phase_findvar = 2;
               s16_elect_phase += 43690;// +240 deg
            }
            else if ( (u8_max_phase == 3) && ((BGVAR(s16_PhaseFind_C_Pos_Crrnt) + BGVAR(s16_PhaseFind_C_Neg_Crrnt)) > 0) )
            {
               debug_phase_findvar = 3;
               s16_elect_phase += 5461; // +30 deg
            }
            else if ( (u8_max_phase == 2) && ((BGVAR(s16_PhaseFind_B_Pos_Crrnt) + BGVAR(s16_PhaseFind_B_Neg_Crrnt)) < 0) )
            {
               debug_phase_findvar = 4;
               s16_elect_phase += 16384;// +90 deg
            }
            else if ( (u8_max_phase == 2) && ((BGVAR(s16_PhaseFind_B_Pos_Crrnt) + BGVAR(s16_PhaseFind_B_Neg_Crrnt)) > 0) )
            {
               debug_phase_findvar = 5;
               s16_elect_phase += 0;
            }
            s16_elect_phase += 30038; // +165 deg
         }
         else
         {
            s16_elect_phase = (int)(f_scale_factor*(float)s16_elect_phase);
            if ( (u8_max_phase == 2) && ((BGVAR(s16_PhaseFind_B_Pos_Crrnt) + BGVAR(s16_PhaseFind_B_Neg_Crrnt)) > 0) )
            {
               pf_debug = 1;
               s16_elect_phase += 10923; // 60
            }
            else if ( (u8_max_phase == 3) && ((BGVAR(s16_PhaseFind_C_Pos_Crrnt) + BGVAR(s16_PhaseFind_C_Neg_Crrnt)) < 0) )
            {
               pf_debug = 2;
               s16_elect_phase = 21845 - s16_elect_phase; // 120 - x
            }
            else if ( (u8_max_phase == 1) && ((BGVAR(s16_PhaseFind_A_Pos_Crrnt) + BGVAR(s16_PhaseFind_A_Neg_Crrnt)) > 0) )
            {
               pf_debug = 3;
               s16_elect_phase += 32768; // 180
            }
            else if ( (u8_max_phase == 2) && ((BGVAR(s16_PhaseFind_B_Pos_Crrnt) + BGVAR(s16_PhaseFind_B_Neg_Crrnt)) < 0) )
            {
               pf_debug = 4;
               s16_elect_phase += 43691; // 240
            }
            else if ( (u8_max_phase == 3) && ((BGVAR(s16_PhaseFind_C_Pos_Crrnt) + BGVAR(s16_PhaseFind_C_Neg_Crrnt)) > 0) )
            {
               pf_debug = 5;
               s16_elect_phase = 54613 - s16_elect_phase; // 300 - x
            }
            else
               pf_debug = 0;

//            s16_elect_phase = (int)(acos((float)s16_elect_phase/12554.0)/6.283185 * 10922.0 +  *10922.0);
         }

         VAR(AX0_u16_PhaseFind_EPos) = s16_elect_phase;
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_UPDATE_COMM;
      break;

      case PHASE_FIND_ZERO_START:
         // Support for Z axis:
         // Wait for XXX el. degree movement after brake release
         // XXX is defined by PHASEFINDDELTA parameter
         // Start phasefind process if 1 sec timeout is passed but motion was not detected
         if(!is_Motion_Detected(drive,BGVAR(u16_PhaseFind_Delta)) && !PassedTimeMS(1000L,BGVAR(s32_PhaseFind_Timer))) break;
         SalZeroCommand(1LL,drive);
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_ZERO_WAIT_FINISH;
      break;

      case PHASE_FIND_ZERO_WAIT_FINISH:
         // Check if zero is done
         if (BGVAR(s16_Zero_State) == ZERO_DONE_STATE)
         {
           VAR(AX0_s16_Feedback_Commutation_Adj) = BGVAR(s16_Zero_Calc_Mphase);
           VAR(AX0_s16_Feedback_Commutation_Adj) -= VAR(AX0_s16_Electrical_Phase_Offset);
           BGVAR(u8_PhaseFind_State) = PHASE_FIND_ZERO_FINAL_DIS;
         }
         else if ((BGVAR(s16_Zero_State)) == ZERO_MOVEMENT_FAIL_STATE)
         {
            BGVAR(u16_WnsError) |= (TOO_LITTLE_MOTION | WNS_FAILED_MASK); // Hard limit detected
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_ZERO_FINAL_DIS;
         }
         else if ((BGVAR(s16_Zero_State)) == ZERO_TIMEOUT_FAIL_STATE)
         {
            BGVAR(u16_WnsError) |= (SETTLE_TIMEOUT | WNS_FAILED_MASK); // Hard limit detected
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_ZERO_FINAL_DIS;
         }
      break;

      case PHASE_FIND_ZERO_FINAL_DIS:
         SalZeroCommand(0LL,drive);
         BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_IN_PROCESS_MASK;

         if (BGVAR(u16_WnsError) & WNS_FAILED_MASK)
         {
            DisableCommand(drive);
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE;
//            BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;
         }
         else
         {
            BGVAR(u16_Motor_Setup_Allow_Enable) = 0; //
            BGVAR(u16_WnsError) |= WNS_RAN;
            BGVAR(u16_PhaseFind_Bits) |= PHASE_FIND_SUCCESS_MASK;
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_DONE_STATE;
         }
      break;

      case PHASE_FIND_UPDATE_COMM: // Update commutation according to _AX0_u16_PhaseFind_EPos
         VAR(AX0_s16_Skip_Flags) &= ~(ZERO_PHASE_ADV_MASK | SKIP_COMM_ADJ_MASK | WNS_ON_MASK);

         if (BGVAR(u8_Sl_Mode) == 0)
            VAR(AX0_s16_PhaseFindRTBits) |= (PHASE_FIND_DONE_MASK | PHASE_FIND_UPDATE_COMM_MASK);
         else
            VAR(AX0_s16_PhaseFindRTBits) |= (PHASE_FIND_DONE_MASK);

         BGVAR(u16_PhaseFind_Bits) |= PHASE_FIND_SUCCESS_MASK;
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_DONE_STATE;

         s32_time_capture = Cntr_1mS;
         u16_update_done = 0;
         // Wait for the phase update to complete (or timeout)
         while ((!PassedTimeMS(10L,s32_time_capture)) && (!u16_update_done) && (BGVAR(u8_Sl_Mode) == 0) )
            u16_update_done = !(VAR(AX0_s16_PhaseFindRTBits) & PHASE_FIND_UPDATE_COMM_MASK);

         // This will clear the velocity D memory as the velocity loop ran during the phase find process
         VAR(AX0_s16_PhaseFindRTBits) |= PHASE_FIND_ISSUE_D_MEM_CLR_MASK;

         if (Enabled(drive))
         {
            VAR(AX0_s16_PhaseFindRTBits) &= ~PHASE_FIND_ACTIVATE_MASK;
            VAR(AX0_s16_Transition_To_Enable) |= ASK_HW_ENABLE_MASK;
            VAR(AX0_u16_BrakeOn) = 0;
         }
         if ((BGVAR(u16_PhaseFind_Mode) == 3) && (s16_mode3support == 1))  // check if it is first time
         {
            DisableCommand(drive);
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_WAIT_FOR_DIS;
         }
      break;

      case PHASE_FIND_WAIT_FOR_DIS:
         if(Enabled(drive)) break;
         BGVAR(u16_PhaseFind_Mode) = 2;
         s16_mode3support = 2; // Use it for enable in next case
         VAR(AX0_s16_Wns_Comm) = VAR(AX0_s16_Elect_Pos_With_Phase_Adv);//AX0_u16_PhaseFind_EPos);
         VAR(AX0_u16_Wns_Status) |= WNS_SKIP_SEC_ITER_MASK; // Issue one wns iteration only
         BGVAR(u8_PhaseFind_State) = PHASE_FIND_IDLE;
         BGVAR(u16_PhaseFind_Bits) |= PHASE_FIND_REQUESTED_MASK;
         BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_SUCCESS_MASK;
      break;

      case PHASE_FIND_APPLY_CURRENT:
      break;

      case PHASE_FIND_FAULT_STATE:
         //restore remote enable input
         if (u16_ignore_remote_en == 2)
         {
            UpdateIndexFunc(drive, REMOTE_ENABLE_INP, u16_remote_en_backup);
            u16_ignore_remote_en = 0;
         }

         if (VAR(AX0_s16_Ignore_Ilim)) VAR(AX0_s16_Ignore_Ilim) = 0;
         BGVAR(u16_PhaseFind_Bits) &= ~(PHASE_FIND_SUCCESS_MASK);
         VAR(AX0_s16_PhaseFindRTBits) &= ~(PHASE_FIND_SET_PE_0_MASK);
         s16_mode3support = 1;
         BGVAR(u16_Motor_Setup_Allow_Enable) = 0; //dbgSS - prohibit enable
         VAR(AX0_u16_Wns_Status) &= ~(WNS_SMOOTH_MODE_MASK | WNS_SKIP_SEC_ITER_MASK);
         VAR(AX0_s16_Skip_Flags)   &= ~(ZERO_PHASE_ADV_MASK | SKIP_COMM_ADJ_MASK | WNS_ON_MASK);
      break;

      case PHASE_FIND_DONE_STATE:
        //restore remote enable input
       if (u16_ignore_remote_en == 2)
       {
         UpdateIndexFunc(drive, REMOTE_ENABLE_INP, u16_remote_en_backup);
         u16_ignore_remote_en = 0;
       }
         if (VAR(AX0_s16_Ignore_Ilim)) VAR(AX0_s16_Ignore_Ilim) = 0;
         VAR(AX0_u16_Wns_Status) &= ~(WNS_SMOOTH_MODE_MASK | WNS_SKIP_SEC_ITER_MASK | WNS_TUNING_MODE_MASK);
         VAR(AX0_s16_PhaseFindRTBits) &= ~(PHASE_FIND_ACTIVATE_MASK | PHASE_FIND_SET_PE_0_MASK);
      break;

      case PHASE_FIND_WNS_WAIT_FOR_DONE:
         if ((VAR(AX0_u16_Wns_Status) & WNS_ENDED_MASK) != 0)
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_WNS_TEST_RESULT;
      break;

      case PHASE_FIND_WNS_TEST_RESULT:
         if ((BGVAR(u16_PhaseFind_Mode) == 2) || (BGVAR(u16_PhaseFind_Mode) == 3))
         {
            // Check if motion too fast
            if (ConvertVelocityToRpm(drive, VAR(AX0_u16_Wns_Max_Vel_Smpl)) > 60)
               BGVAR(u16_WnsError) |= MAX_VEL_ERROR;

            // Test motion > 45 elect degrees
            do {
               s16_temp = Cntr_3125;
               s64_temp = LLVAR(AX0_u32_Pos_Fdbk_User_Lo) - LLVAR(AX0_u32_Wns_Pfb_0_Lo);
            } while (s16_temp != Cntr_3125);

            if (s64_temp < 0LL) s64_temp = -s64_temp;      // Make sure its positive
            s64_temp *= (long long)VAR(AX0_s16_Num_Of_Poles_Ratio);
            if (s64_temp > (long long)0x20000000)
               BGVAR(u16_WnsError) |= TOO_MUCH_MOTION; // motion > 45 elect deg

            // Min motion should be at least 2 elect degrees
            if (s16_mode3support == 2) s64_temp = s64_temp << 1LL;
            if (s64_temp < (long long)0x016C16C1)
            {
               BGVAR(u16_WnsError) |= TOO_LITTLE_MOTION;
            }
         }
         // Motion profile pfb(0-0.5) > pfb(0.5-1)*4
         // Delta_pfb_at_middle = LVAR(AX0_Wns_Mid_Pfb) - LVAR(AX0_Wns_Pfb_0);
         // Delta_pfb_at_end = LVAR(AX0_Pos_Fdbk_Reported_32) - LVAR(AX0_Wns_Mid_Pfb);
         //if (delta_pfb_at_middle<0) delta_pfb_at_middle=-delta_pfb_at_middle;
         //if (delta_pfb_at_end<0) delta_pfb_at_end=-delta_pfb_at_end;
         //if (delta_pfb_at_middle < (delta_pfb_at_end<<2L)) BGVAR(u16_WnsError) |= MOTION_PROFILE;

         if (BGVAR(u16_WnsError) != 0)
         {
            if ((!(BGVAR(u16_WnsError) & WNS_STOPPED)) && (BGVAR(u16_WnsRetry) > 0))
            {            // Test for retry
               if(s16_mode3support == 2) BGVAR(u16_PhaseFind_Mode) = 3;
               BGVAR(u16_WnsRetry)--;
               BGVAR(u8_PhaseFind_State) = PHASE_FIND_WNS_RETRY_DELAY;
               BGVAR(s32_PhaseFind_Timer) = Cntr_1mS;
               break;
            }
            else
            {
               BGVAR(u16_WnsRetry) = 3;
               //HandleFault(drive,FEEDBACK_LOSS_FLT_MASK,ENC_NOT_INITIALIZE_MASK, 0);
               BGVAR(u16_WnsError) |= WNS_FAILED_MASK;
               //if (VAR(AX0_Motor_Enc_Type) == 3) BGVAR(EncInit_State) = ENC_INIT_NOT_STARTED;
               BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;
               BGVAR(u8_PhaseFind_State) = PHASE_FIND_FAULT_STATE;

               //restore remote enable input
               if (u16_ignore_remote_en == 2)
               {
                  UpdateIndexFunc(drive, REMOTE_ENABLE_INP, u16_remote_en_backup);
                  u16_ignore_remote_en = 0;
               }
            }
         }
         else BGVAR(u8_PhaseFind_State) = PHASE_FIND_UPDATE_COMM;

         VAR(AX0_s16_Serial_Crrnt_Cmnd) = 0;
         VAR(AX0_s16_Wns_Crrnt_Cmnd) = 0;
         VAR(AX0_s16_Skip_Flags) &= ~(ZERO_PHASE_ADV_MASK | SKIP_COMM_ADJ_MASK | WNS_ON_MASK); //skip phase adv
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Feedback_Comm_Pos);

         if (VAR(AX0_s16_Ignore_Ilim)) VAR(AX0_s16_Ignore_Ilim) = 0;
         s16_mode3support = 1;
         BGVAR(u16_Motor_Setup_Allow_Enable) = 0; //dbgSS - prohibit enable
         SetOpmode(drive, VAR(AX0_s16_Opmode));

         //set back Fieldbus related parametrs
         if( (BGVAR(s16_CAN_Opmode) == CYCLIC_SYNCHRONOUS_POSITION_MODE) ||
             (BGVAR(s16_CAN_Opmode) == CYCLIC_SYNCHRONOUS_VELOCITY_MODE) ||
             (BGVAR(s16_CAN_Opmode) == INTRPOLATED_POSITION_MODE)        ||
             (BGVAR(s16_CAN_Opmode) == CYCLIC_SYNCHRONOUS_TORQUE_MODE)     )
         {
            FalSetFBSyncOpmode(drive, BGVAR(s16_CAN_Opmode));
         }

         BGVAR(u16_WnsError) |= WNS_RAN;
      break;

      case PHASE_FIND_WNS_RETRY_DELAY:
         if (PassedTimeMS(500L, BGVAR(s32_PhaseFind_Timer)))
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_WAIT_FOR_EN;
      break;
   }
}


int PhaseFindSmooth(int drive)
{
   // AXIS_OFF;
   int retval = 0;
   int s16_temp = 0;

   if ((BGVAR(u16_Fold) == 1) || (BGVAR(u16_Motor_Fold) == 1))
      BGVAR(u16_PhaseFindSmooth_Foldback_Occured) = 1;

   switch(BGVAR(u16_PhaseFindSmooth_State))
   {
      // The procedure will begin when the drive will enable
      case PHASE_FIND_SMOOTH_START:
         BGVAR(u16_PhaseFindSmooth_RT_Mode)  =  PH_FND_SM_RT_MODE_IDLE_MASK; // // Set RT to Idle and reset other bits
         VAR(AX0_s16_Wns_Comm) = 32767;
         VAR(AX0_s16_Feedback_Commutation_Adj) = 0;
         BGVAR(s16_PhaseFindSm_Error_Angle) = 32767;
         BGVAR(s16_PhaseFindSm_Dir) = 0;
         BGVAR(s64_PhaseFindSm_Max_Pos) = 0x8000000000000000;
         BGVAR(s64_PhaseFindSm_Min_Pos) = 0x7FFFFFFFFFFFFFFF;
         BGVAR(s16_PhaseFindSm_Best_Wns_Comm) = 0;
         BGVAR(s16_PhaseFindSm_Crrnt_Cmnd_Max) = 0;
         BGVAR(s16_PhaseFindSm_Crrnt_Cmnd_Max_Prev) = 0;
         BGVAR(u16_PhaseFindSmooth_Foldback_Occured) = 0;
         BGVAR(u16_PhaseFindSmooth_RunAway_Cntr1) = 0; // Zero counters
         BGVAR(u16_PhaseFindSmooth_RunAway_Cntr2) = 0;
         if (BGVAR(u16_PhaseFind_Time) < 100)
            BGVAR(u16_PhaseFind_Pos_Stop_Time) = 100; // Set time to define when motor stop condition and limit to min 100ms
         else BGVAR(u16_PhaseFind_Pos_Stop_Time) = BGVAR(u16_PhaseFind_Time);
         BGVAR(s64_PhaseFindSm_Comp_Angle) = (long long)((11963697.0/(float)VAR(AX0_s16_Num_Of_Poles_Ratio))*1000.0/(float)BGVAR(s16_PhaseFind_Gain));
         do {
            s16_temp = Cntr_3125;
            BGVAR(s64_PhaseFindSm_First_Pos) = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
         } while (s16_temp != Cntr_3125);
         BGVAR(s64_PhaseFindSm_Start_Pos) = BGVAR(s64_PhaseFindSm_First_Pos);
         if (BGVAR(s64_PhaseFindSm_First_Pos) < BGVAR(s64_PhaseFindSm_Min_Pos)) BGVAR(s64_PhaseFindSm_Min_Pos) = BGVAR(s64_PhaseFindSm_First_Pos);
         if (BGVAR(s64_PhaseFindSm_First_Pos) >= BGVAR(s64_PhaseFindSm_Max_Pos)) BGVAR(s64_PhaseFindSm_Max_Pos) = BGVAR(s64_PhaseFindSm_First_Pos);
         if (Enabled(DRIVE_PARAM)) BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_START_ANGLE;
      break;

      case PHASE_FIND_SMOOTH_START_ANGLE:
         if (abs(BGVAR(s16_PhaseFindSm_Error_Angle)) >= 182) // Set Limit to 1 Elect angle
         {
            BGVAR(s16_PhaseFindSm_Dir_Prev) = BGVAR(s16_PhaseFindSm_Dir);
            do {
               s16_temp = Cntr_3125;
               BGVAR(s64_PhaseFindSm_Start_Pos) = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);// Get Start position
            } while (s16_temp != Cntr_3125);
            if (BGVAR(s64_PhaseFindSm_Start_Pos) < BGVAR(s64_PhaseFindSm_Min_Pos))
               BGVAR(s64_PhaseFindSm_Min_Pos) = BGVAR(s64_PhaseFindSm_Start_Pos);
            if (BGVAR(s64_PhaseFindSm_Start_Pos) >= BGVAR(s64_PhaseFindSm_Max_Pos))
               BGVAR(s64_PhaseFindSm_Max_Pos) = BGVAR(s64_PhaseFindSm_Start_Pos);
            BGVAR(u32_PhaseFindSmPos_Fdbk_Lo_Last) = LVAR(AX0_u32_Pos_Fdbk_Lo);
            BGVAR(s16_PhaseFindSm_Curr_Step) = 10; // Set step to increase current
            BGVAR(u16_PhaseFindSmooth_RT_Result) = 0; // Reset all results
            BGVAR(u16_PhaseFindSmooth_RT_Mode)  =   PH_FND_SM_RT_MODE_INC_CURR_MASK;// Reset Idle bit and Start to increase current (RT)
            BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_INC_CURR;
         }
         else
         {
            VAR(AX0_s16_Wns_Comm) = BGVAR(s16_PhaseFindSm_Best_Wns_Comm);
            BGVAR(s16_PhaseFindSm_Dir) = 0;
            BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_FINAL_INIT_STOP;
         }
      break;

      case PHASE_FIND_SMOOTH_INC_CURR:
         // Wait for result from PhaseFindSmoothIncCurr(drive)
         if ((BGVAR(u16_PhaseFindSmooth_RT_Mode) & PH_FND_SM_RT_MODE_IDLE_MASK) == 0) break;

         if ((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_SETTLING_TIMEOUT) != 0)
            BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_END;
         else
         {
            if ((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_RES_DEG_REACHED) != 0)
               BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_DEG_REACHED;
            else if ((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_RES_MAX_CURR_REACHED) != 0)
               BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_FINAL_INIT_STOP;
         }
      break;

      case PHASE_FIND_SMOOTH_DEG_REACHED:
         if (BGVAR(s16_PhaseFindSm_Crrnt_Cmnd_Max) > BGVAR(s16_PhaseFindSm_Crrnt_Cmnd_Max_Prev))
         {
            BGVAR(s16_PhaseFindSm_Best_Wns_Comm) = VAR(AX0_s16_Wns_Comm);
            BGVAR(s16_PhaseFindSm_Crrnt_Cmnd_Max_Prev) = BGVAR(s16_PhaseFindSm_Crrnt_Cmnd_Max);
         }

         if (BGVAR(s16_PhaseFindSm_Dir)==1)
         {
            if (BGVAR(s16_PhaseFindSm_Dir_Prev) != 1)
               BGVAR(s16_PhaseFindSm_Error_Angle) = BGVAR(s16_PhaseFindSm_Error_Angle)/2;
            VAR(AX0_s16_Wns_Comm) += (-BGVAR(s16_PhaseFindSm_Error_Angle));
         }
         else if (BGVAR(s16_PhaseFindSm_Dir)==-1)
         {
            if (BGVAR(s16_PhaseFindSm_Dir_Prev) != -1)
               BGVAR(s16_PhaseFindSm_Error_Angle) = BGVAR(s16_PhaseFindSm_Error_Angle)/2;
            VAR(AX0_s16_Wns_Comm) += BGVAR(s16_PhaseFindSm_Error_Angle);
         }
         else
         {
            if (BGVAR(s16_PhaseFindSm_Dir_Prev)==0)
            {
               BGVAR(s16_PhaseFindSm_Error_Angle) = BGVAR(s16_PhaseFindSm_Error_Angle)/2;
               VAR(AX0_s16_Wns_Comm) += BGVAR(s16_PhaseFindSm_Error_Angle);
            }
            else
            {
               BGVAR(s16_PhaseFindSm_Error_Angle) = 0;
            }
         }
         BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_START_ANGLE;
      break;

      case PHASE_FIND_SMOOTH_FINAL_INIT_STOP:
         do {
            s16_temp = Cntr_3125;
            BGVAR(s64_PhaseFindSm_Start_Pos) = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);// Get Start position
         } while (s16_temp != Cntr_3125);
         BGVAR(u16_PhaseFind_Pos_Stop_Time) = BGVAR(u16_PhaseFind_Pos_Stop_Time) << 1; //Enlarge stop time to min 200 ms
         if (BGVAR(u16_PhaseFind_Pos_Stop_Time) > 32000)
            BGVAR(u16_PhaseFind_Pos_Stop_Time) = 32000;
         BGVAR(s16_PhaseFindSm_Pos_Cntr) = 0;
         BGVAR(u32_PhaseFindSm_Pos_TimeOut_Cntr) = 0;
         BGVAR(u16_PhaseFindSmooth_RT_Mode) = PH_FND_SM_RT_MODE_WAIT_STOP_MASK;// Start waiting for stop and reset other bits
         BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_FINAL_WAIT_FOR_STOP;
      break;

      case PHASE_FIND_SMOOTH_FINAL_WAIT_FOR_STOP:
         if ((BGVAR(u16_PhaseFindSmooth_RT_Mode) & PH_FND_SM_RT_MODE_IDLE_MASK) == 0) break;

         if ((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_SETTLING_TIMEOUT) != 0)
            BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_END;
         else BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_FINAL;
      break;

      case PHASE_FIND_SMOOTH_FINAL:
         do {
            s16_temp = Cntr_3125;
            BGVAR(s64_PhaseFindSm_Curr_Pos) = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);// Get Current position
         } while (s16_temp != Cntr_3125);
         if (BGVAR(s64_PhaseFindSm_Curr_Pos) < BGVAR(s64_PhaseFindSm_Min_Pos))
            BGVAR(s64_PhaseFindSm_Min_Pos) = BGVAR(s64_PhaseFindSm_Curr_Pos);
         if (BGVAR(s64_PhaseFindSm_Curr_Pos) >= BGVAR(s64_PhaseFindSm_Max_Pos))
            BGVAR(s64_PhaseFindSm_Max_Pos) = BGVAR(s64_PhaseFindSm_Curr_Pos);
         BGVAR(s16_PhaseFindSm_Delta_Release) = (int)((int)((LVAR(AX0_u32_Pos_Fdbk_Lo) - BGVAR(u32_PhaseFindSmPos_Fdbk_Lo_Last)) >> 16L) * VAR(AX0_s16_Num_Of_Poles_Ratio));
         VAR(AX0_s16_Feedback_Commutation_Adj) = VAR(AX0_s16_Elect_Pos_With_Phase_Adv) - (int)((int)(LVAR(AX0_u32_Pos_Fdbk_Lo) >> 16L) * VAR(AX0_s16_Num_Of_Poles_Ratio)) + BGVAR(s16_PhaseFindSm_Delta_Release);
         VAR(AX0_s16_Feedback_Commutation_Adj) -= VAR(AX0_s16_Electrical_Phase_Offset);
         // fix 30 deg offset in new Iabc->dq transformation
         if (VAR(AX0_S16_Kc_Mode) >= 4) VAR(AX0_s16_Feedback_Commutation_Adj) += 0x1555;
         VAR(AX0_s16_Skip_Flags) &= ~(ZERO_PHASE_ADV_MASK | SKIP_COMM_ADJ_MASK | WNS_ON_MASK);
         BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_ADJ_ANGLE;
      break;

      case PHASE_FIND_SMOOTH_ADJ_ANGLE:
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Feedback_Comm_Pos);
         VAR(AX0_s16_Feedback_Commutation_Adj) += 16384;
         BGVAR(s64_PhaseFindSm_Diff_Pos) = (BGVAR(s64_PhaseFindSm_Curr_Pos)) - (BGVAR(s64_PhaseFindSm_First_Pos));
         BGVAR(s16_PhaseFindSm_Curr_Step) = -10;
         BGVAR(u16_PhaseFindSmooth_RT_Result) &= ~(PH_FND_SM_RT_RES_MAX_CURR_REACHED|PH_FND_SM_RT_RES_DEG_REACHED);
         BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_MOVE_BACK;
         if (BGVAR(s64_PhaseFindSm_Diff_Pos) < 0LL)
         {
            BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_MOVE_FORTH;
            BGVAR(s16_PhaseFindSm_Curr_Step) = 10;
         }
         BGVAR(s64_PhaseFindSm_Start_Pos) = BGVAR(s64_PhaseFindSm_Curr_Pos);
         BGVAR(u16_PhaseFindSmooth_RT_Mode) = PH_FND_SM_RT_MODE_INC_CURR_MASK;// Start to increase current
      break;

      case PHASE_FIND_SMOOTH_MOVE_BACK:
         if ((BGVAR(u16_PhaseFindSmooth_RT_Mode) & PH_FND_SM_RT_MODE_IDLE_MASK) == 0) break;

         if ((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_SETTLING_TIMEOUT) != 0)
            BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_END;
         else
         {
            if (((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_RES_DEG_REACHED) != 0) &&
                 (BGVAR(s16_PhaseFindSm_Dir) == 1))
            {
               VAR(AX0_s16_Feedback_Commutation_Adj) -=  32767;
               BGVAR(u16_PhaseFindSmooth_RunAway_Cntr2)++;
            }
            else if ((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_RES_MAX_CURR_REACHED) != 0)
            {
               BGVAR(s16_PhaseFindSm_Dir) = 0; //max current reached
            }

            BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_RETEST_RUNAWAY;
         }
      break;

      case PHASE_FIND_SMOOTH_MOVE_FORTH:
         if ((BGVAR(u16_PhaseFindSmooth_RT_Mode) & PH_FND_SM_RT_MODE_IDLE_MASK) == 0) break;

         if ((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_SETTLING_TIMEOUT) != 0)
            BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_END;
         else
         {
            if (((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_RES_DEG_REACHED) != 0) &&
                 (BGVAR(s16_PhaseFindSm_Dir) == -1))
            {
               VAR(AX0_s16_Feedback_Commutation_Adj) -=  32767;
               BGVAR(u16_PhaseFindSmooth_RunAway_Cntr2)++;
            }
            else if ((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_RES_MAX_CURR_REACHED) != 0)
            {
               BGVAR(s16_PhaseFindSm_Dir) = 0; //max current reached
            }
            BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_RETEST_RUNAWAY;
         }
      break;

      case PHASE_FIND_SMOOTH_RETEST_RUNAWAY:
         BGVAR(u16_PhaseFindSmooth_RunAway_Cntr1)++;
         if (BGVAR(u16_PhaseFindSmooth_RunAway_Cntr1) >= 3)
         {
            if (BGVAR(u16_PhaseFindSmooth_RunAway_Cntr2) >= 1)
            {
               if ((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_WRONG_DETECTION) != 0)
               {
                  BGVAR(u16_PhaseFindSmooth_RT_Result) |= PH_FND_SM_RT_SEC_WRONG_DETECTION; // If wrong detection occurs twice then fail
                  BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_END;
                  break;
               }
               BGVAR(u16_PhaseFind_Pos_Stop_Time) = BGVAR(u16_PhaseFind_Pos_Stop_Time) >> 1; //Reduce stop time twice
               BGVAR(u16_PhaseFindSmooth_RunAway_Cntr1) = 0; // Zero counters and start test again
               BGVAR(u16_PhaseFindSmooth_RunAway_Cntr2) = 0;
               BGVAR(u16_PhaseFindSmooth_RT_Result) |= PH_FND_SM_RT_WRONG_DETECTION;
               //break;
            }
            else
            {
               BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_END;
               break;
            }
         }
         else
         {
            BGVAR(s64_PhaseFindSm_Diff_Pos) = (BGVAR(s64_PhaseFindSm_Curr_Pos)) - (BGVAR(s64_PhaseFindSm_First_Pos));
            BGVAR(s16_PhaseFindSm_Curr_Step) = -10;
            BGVAR(u16_PhaseFindSmooth_RT_Result) &= ~(PH_FND_SM_RT_RES_MAX_CURR_REACHED | PH_FND_SM_RT_RES_DEG_REACHED);
            BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_MOVE_BACK;
            if (BGVAR(s64_PhaseFindSm_Diff_Pos) < 0LL)
            {
               BGVAR(u16_PhaseFindSmooth_State) = PHASE_FIND_SMOOTH_MOVE_FORTH;
               BGVAR(s16_PhaseFindSm_Curr_Step) = 10;
            }
            BGVAR(s64_PhaseFindSm_Start_Pos) = BGVAR(s64_PhaseFindSm_Curr_Pos);
            BGVAR(u16_PhaseFind_Pos_Stop_Time) = BGVAR(u16_PhaseFind_Pos_Stop_Time) << 1; //Enlarge stop time twice
            if (BGVAR(u16_PhaseFind_Pos_Stop_Time) > 32000)
               BGVAR(u16_PhaseFind_Pos_Stop_Time) = 32000;
            BGVAR(u16_PhaseFindSmooth_RT_Mode)  =   PH_FND_SM_RT_MODE_INC_CURR_MASK;// Start to increase current
         }
      break;

      case PHASE_FIND_SMOOTH_END:
         BGVAR(f_PhaseFindSm_Total_Movement) = (float)(((float)(BGVAR(s64_PhaseFindSm_Max_Pos) - BGVAR(s64_PhaseFindSm_Min_Pos))/11963697.0)*(float)VAR(AX0_s16_Num_Of_Poles_Ratio));
         retval = 1;
         if ((BGVAR(s16_PhaseFindSm_Dir) == 0) && (BGVAR(f_PhaseFindSm_Total_Movement) < 0.01))
            BGVAR(u16_WnsError) |= (TOO_LITTLE_MOTION | WNS_FAILED_MASK); // Hard limit detected
         if (BGVAR(f_PhaseFindSm_Total_Movement) >= 90.0)
            BGVAR(u16_WnsError) |= (TOO_MUCH_MOTION | WNS_FAILED_MASK); // Too much motion detected
         if ((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_SEC_WRONG_DETECTION) != 0)
            BGVAR(u16_WnsError) |= (TOO_MUCH_OSC | WNS_FAILED_MASK); // Too much oscillations detected
         if ((BGVAR(u16_PhaseFindSmooth_RT_Result) & PH_FND_SM_RT_SETTLING_TIMEOUT) != 0)
            BGVAR(u16_WnsError) |= (SETTLE_TIMEOUT | WNS_FAILED_MASK); //
         if (BGVAR(u16_PhaseFindSmooth_Foldback_Occured))
            BGVAR(u16_WnsError) |= (FOLDBACK_OCCURED | WNS_FAILED_MASK); // Foldback occured during the process, results are meaningless
      break;
   }
   return retval;
}


/**********************************************************
* Function Name: PhaseFindSmoothIncCurr
* Description:
*  This function increases current until (1 el. degree)*factor or max current is reached
*  The function runs within 1 ms RT.
* Author: Sergey

**********************************************************/
//#pragma CODE_SECTION(PhaseFindSmoothIncCurr, "ramfunc_4");
void PhaseFindSmoothIncCurr(int drive)
{
   // AXIS_OFF;
   int s16_temp;
   REFERENCE_TO_DRIVE;
   if ((BGVAR(u16_PhaseFindSmooth_RT_Mode) & PH_FND_SM_RT_MODE_IDLE_MASK) != 0) return;
   do {
      s16_temp = Cntr_3125;
      BGVAR(s64_PhaseFindSm_Curr_Pos) = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);// Get Current position
   } while (s16_temp != Cntr_3125);
   if (BGVAR(s64_PhaseFindSm_Curr_Pos) < BGVAR(s64_PhaseFindSm_Min_Pos))
      BGVAR(s64_PhaseFindSm_Min_Pos) = BGVAR(s64_PhaseFindSm_Curr_Pos);
   if (BGVAR(s64_PhaseFindSm_Curr_Pos) >= BGVAR(s64_PhaseFindSm_Max_Pos))
      BGVAR(s64_PhaseFindSm_Max_Pos) = BGVAR(s64_PhaseFindSm_Curr_Pos);
   BGVAR(s64_PhaseFindSm_Diff_Pos) = (BGVAR(s64_PhaseFindSm_Curr_Pos))-(BGVAR(s64_PhaseFindSm_Start_Pos));
   if ((BGVAR(u16_PhaseFindSmooth_RT_Mode) & PH_FND_SM_RT_MODE_INC_CURR_MASK) != 0)
   {
      BGVAR(s16_PhaseFindSm_Dir) = 1;
      if(BGVAR(s64_PhaseFindSm_Diff_Pos) < 0) BGVAR(s16_PhaseFindSm_Dir) = -1;

      if (llabs(BGVAR(s64_PhaseFindSm_Diff_Pos)) < BGVAR(s64_PhaseFindSm_Comp_Angle))
      {
         if (abs(VAR(AX0_s16_Wns_Crrnt_Cmnd) + BGVAR(s16_PhaseFindSm_Curr_Step)) < VAR(AX0_s16_Phase_Find_Current))
         {
            VAR(AX0_s16_Wns_Crrnt_Cmnd) += BGVAR(s16_PhaseFindSm_Curr_Step);
         }
         else
         {
            VAR(AX0_s16_Wns_Crrnt_Cmnd) = 0;
            BGVAR(s16_PhaseFindSm_Pos_Cntr) = 0;
            BGVAR(u32_PhaseFindSm_Pos_TimeOut_Cntr) = 0;
            BGVAR(s64_PhaseFindSm_Start_Pos) = BGVAR(s64_PhaseFindSm_Curr_Pos);
            BGVAR(u16_PhaseFindSmooth_RT_Result) |=  PH_FND_SM_RT_RES_MAX_CURR_REACHED;
            BGVAR(u16_PhaseFindSmooth_RT_Mode)   =  PH_FND_SM_RT_MODE_WAIT_STOP_MASK; // Start waiting for stop and reset other bits
         }
      }
      else
      {
         BGVAR(s16_PhaseFindSm_Crrnt_Cmnd_Max) = VAR(AX0_s16_Wns_Crrnt_Cmnd);
         VAR(AX0_s16_Wns_Crrnt_Cmnd) = 0;
         BGVAR(s16_PhaseFindSm_Pos_Cntr) = 0;
         BGVAR(u32_PhaseFindSm_Pos_TimeOut_Cntr) = 0;
         BGVAR(s64_PhaseFindSm_Start_Pos) = BGVAR(s64_PhaseFindSm_Curr_Pos);
         BGVAR(u16_PhaseFindSmooth_RT_Result) |= PH_FND_SM_RT_RES_DEG_REACHED;
         BGVAR(u16_PhaseFindSmooth_RT_Mode)   = PH_FND_SM_RT_MODE_WAIT_STOP_MASK;// Start waiting for stop and reset other bits
      }
   }
   // Wait until motor is stopped
   else if ((BGVAR(u16_PhaseFindSmooth_RT_Mode) & PH_FND_SM_RT_MODE_WAIT_STOP_MASK) != 0)
   {
      BGVAR(u32_PhaseFindSm_Pos_TimeOut_Cntr)++;
      if (llabs(BGVAR(s64_PhaseFindSm_Diff_Pos)) >= BGVAR(s64_PhaseFindSm_Comp_Angle))
      {
         BGVAR(s64_PhaseFindSm_Start_Pos) = BGVAR(s64_PhaseFindSm_Curr_Pos);
         BGVAR(s16_PhaseFindSm_Pos_Cntr) = 0;
      }

      BGVAR(s16_PhaseFindSm_Pos_Cntr)++;
      // Stop if no movement more than specified time
      if (BGVAR(s16_PhaseFindSm_Pos_Cntr) >= BGVAR(u16_PhaseFind_Pos_Stop_Time))
         BGVAR(u16_PhaseFindSmooth_RT_Mode)  =  PH_FND_SM_RT_MODE_IDLE_MASK; // Set RT to Idle and reset other bits
      // Stop if time-out occurred
      if (BGVAR(u32_PhaseFindSm_Pos_TimeOut_Cntr) >= (BGVAR(u16_PhaseFind_Pos_Stop_Time)<<4))
      {
         BGVAR(u16_PhaseFindSmooth_RT_Result) |= PH_FND_SM_RT_SETTLING_TIMEOUT;
         BGVAR(u16_PhaseFindSmooth_RT_Mode)  =  PH_FND_SM_RT_MODE_IDLE_MASK; // Set RT to Idle and reset other bits
      }
   }

   return;
}


/**********************************************************
* Function Name: CalculateSaliency
* Description:
*  This function calculates motor saliency (implemented in estmotorparam)
* Author: Sergey
* Calculates:           Motor Saliency
**********************************************************/
int CalculateSaliency(int drive)
{
   switch (BGVAR(u8_SalFind_State))
   {
      case SALFIND_IDLE:
         return (SAL_SUCCESS);

      case SALFIND_WAIT_FOR_USER_EN:
         if(!Enabled(drive)) break;
         BGVAR(u8_SalFind_State) = SALFIND_ISSUE_DIS;
      break;

      case SALFIND_ISSUE_DIS:
         DisableCommand(drive);
         BGVAR(u8_SalFind_State) = SALFIND_WAIT_FOR_DIS;
      break;
/*
      case SALFIND_WAIT_FOR_DIS:
         if(Enabled(drive)) break;
         BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_SETUP;
         BGVAR(u8_SalFind_State) = SALFIND_ESTMOTORPARAM_WAIT_FOR_EN_STATE;
      break;
 */
      case SALFIND_WAIT_FOR_DIS:
         if(Enabled(drive)) break;
         BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_INIT;
         BGVAR(u8_SalFind_State) = SALFIND_ESTMOTORPARAM_WAIT_FOR_EN_STATE;
      break;
/*
      case SALFIND_ESTMOTORPARAM_WAIT_FOR_EN_STATE:
         if(BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_WAIT_FOR_EN)
         {
            EnableCommand(drive);
            BGVAR(u8_SalFind_State) = SALFIND_ESTMOTORPARAM_WAIT_FOR_DONE;
         }
         else if (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_FAULT)
         {
            return (SAL_SUCCESS);
         }
      break;
*/
      case SALFIND_ESTMOTORPARAM_WAIT_FOR_EN_STATE:
         if(BGVAR(s16_Motor_Params_Est_State) == MOTOR_PARAMS_EST_STATE_WAIT_FOR_ENABLE)
         {
            EnableCommand(drive);
            BGVAR(u8_SalFind_State) = SALFIND_ESTMOTORPARAM_WAIT_FOR_DONE;
         }
         else if (BGVAR(s16_Motor_Params_Est_State) < 0) //motor parameters estimation fault
         {
            return (SAL_SUCCESS);
         }
      break;
/*
      case SALFIND_ESTMOTORPARAM_WAIT_FOR_DONE:
         if(BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_DONE)
         {
            DisableCommand(drive);
            BGVAR(u8_SalFind_State) = SALFIND_WAIT_FOR_DIS_2;
         }
         else if (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_FAULT)
         {
            return (SAL_SUCCESS);
         }
      break;
*/

      case SALFIND_ESTMOTORPARAM_WAIT_FOR_DONE:
         if(BGVAR(s16_Motor_Params_Est_State) == MOTOR_PARAMS_EST_STATE_DONE)
         {
            DisableCommand(drive);
            BGVAR(u8_SalFind_State) = SALFIND_WAIT_FOR_DIS_2;
         }
         else if (BGVAR(s16_Motor_Params_Est_State) < 0 ) //motor parameters estimation fault
         {
             return (SAL_SUCCESS);
         }
      break;

      case SALFIND_WAIT_FOR_DIS_2:
         if(Enabled(drive)) break;
         BGVAR(u8_SalFind_State) = SALFIND_IDLE;
         return (SAL_SUCCESS);
   }
   return (SAL_NOT_FINISHED);
}


/**********************************************************
* Function Name: CalculateWnsGains
* Description:
*  This function calculates Wns gains for the wake no shake
*  encinit
* Author: Yuval
* Calculates:           Wns_Kp
*                       Wns_Ki
*                       Wns_Out_Shr
**********************************************************/
int CalculateWnsGains(int drive)
{
   // AXIS_OFF;

   int s16_fix_kp, s16_fix_ki;
   unsigned int u16_shr_kp, u16_shr_ki;

   float f_temp, f_temp_num, f_temp_den;
   int s16_min_current;
   // workaround to fix bug 2827
   s16_min_current = ((long long)min(BGVAR(s32_Drive_I_Cont), BGVAR(s32_Motor_I_Cont)) * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
            >> (long long)BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   s16_min_current = (int)((float)s16_min_current * 0.15);
   if(s16_min_current <= 500) s16_min_current = 500;
   if (BGVAR(s32_PhaseFind_Current) == 0L) PhaseFindCurrentCommand((long long)s16_min_current, drive);

   LVAR(AX0_s32_Wns_Pfb_Lim_0_Hi) = 0L;
   // 4294967296.0*2/360.0/poles_ratio=23860929.42; set limit 2 elect angles
   // LVAR(AX0_u32_Wns_Pfb_Lim_0_Lo) = 23860929.42/((float)VAR(AX0_s16_Num_Of_Poles_Ratio));
   // 11930464.71 * 0.5 => set limit to 0.5 elect angle
   LVAR(AX0_u32_Wns_Pfb_Lim_0_Lo) = 11930464.71 * 0.5 / ((float)VAR(AX0_s16_Num_Of_Poles_Ratio));

   /* wco=120*2*pi = 753.6 ~ 754
    * p_gain=wn^2*15/vlim/wco
    * g_integ=1/exp(-ts*40*2*pi)-1 ~ ts*40*2*pi = 0.1256
    *
    * max_adv=25
    * sat=65536/num_of_poles_ratio*max_adv/360 = 4551/num_of_poles_ratio
    *
    * kp    = 1/p_gain = vlim*wco/15/wn^2
    * ki    = g_integ*kp
    *
    * kp=wco*vlim*4/wn^2/mpitch
    *
    */


   /* Calculate Wn^2
   * --------------
   * Rotary: Wn^2 = WnsGain * Mkt * PhaseFindI * Num_Poles_Ratio / Mj
   *
   *
   *                Vlim * 754
   *           Kp = ----------       ;   Ki = 0.1256 * Kp
   *                15 * Wn^2
   *
   *                464 * WnsGain                               Micont
   * Linear: Wn^2 = -------------- * Mkt * EncInitCurrent * ---------------
   *                   1770                                   Mj * Mpitch
   *
   *                Vlim * 4 * 754
   *           Kp = --------------     ;  Ki = 0.1256 * Kp   ;   MBEMF = MKT * 0.06042
   *                Wn^2 * Mpitch
   *
   * Note: Num_Poles_Ratio in linear motor == 1
   */

   /*
   *  Replacing Wn^2 in the formula of Kp yields the following:
   *
   *  Rotary:
   *                               Vlim * 754 * MJ
   *           Kp = --------------------------------------------------------------
   *                15 * WnsGain * Mkt * PhaseFindI * Num_Poles_Ratio
   *
   *  Linear:
   *                         Vlim * 754 * MJ
   *           Kp = ---------------------------------------------
   *                116 * WnsGain * Mkt * PhaseFindI
   *
   *  Rotary and linear:
   *
   *           Ki = 0.1256 * Kp
   *
   */
   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      if ( (BGVAR(s32_V_Lim_Design) == 0L) || (BGVAR(s32_Motor_J) == 0L)           ||
           (BGVAR(s16_PhaseFind_Gain) == 0) || (BGVAR(u32_Motor_Kt) == 0L)         ||
           (BGVAR(s32_PhaseFind_Current) == 0L) || (BGVAR(s32_Motor_I_Cont) == 0L) ||
           (VAR(AX0_s16_Num_Of_Poles_Ratio) == 0)                                    )
      {
         VAR(s16_fix_kp) = 0;
         VAR(s16_fix_ki) = 0;
         VAR(u16_shr_kp) = 0;
         VAR(u16_shr_ki) = 0;
         VAR(AX0_u16_Wns_Kp)    = (unsigned int)s16_fix_kp;
         VAR(AX0_u16_Wns_Ki)    = (unsigned int)s16_fix_ki;
         VAR(AX0_u16_Wns_Out_Shr)= u16_shr_kp;
         return 1;
      }
      else
      {
         f_temp_den = 15.0 * (float)VAR(AX0_s16_Num_Of_Poles_Ratio);
         f_temp_num = (float)BGVAR(s32_V_Lim_Design) * 0.0001117587 * 754.0 * (float)BGVAR(s32_Motor_J);
         // if PHASEFINDI > IMAX then use IMAX
         if (BGVAR(s32_PhaseFind_Current) > BGVAR(s32_Imax))
            f_temp_den *= ((float)BGVAR(s16_PhaseFind_Gain) / 1000.0) * (float)BGVAR(u32_Motor_Kt) * (float)BGVAR(s32_Imax);
         else
            f_temp_den *= ((float)BGVAR(s16_PhaseFind_Gain) / 1000.0) * (float)BGVAR(u32_Motor_Kt) * (float)BGVAR(s32_PhaseFind_Current);
      }

      f_temp = f_temp_num / f_temp_den;
   }

   if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
   {
      if ( (BGVAR(s32_V_Lim_Design) == 0L) || (BGVAR(s32_Motor_Mass) == 0L)       ||
           (BGVAR(s16_PhaseFind_Gain) == 0) || (BGVAR(u32_Motor_Kf) == 0L)        ||
           (BGVAR(s32_PhaseFind_Current) == 0L) || (BGVAR(s32_Motor_I_Cont) == 0L)  )
      {
         VAR(s16_fix_kp) = 0;
         VAR(s16_fix_ki) = 0;
         VAR(u16_shr_kp) = 0;
         VAR(u16_shr_ki) = 0;
         VAR(AX0_u16_Wns_Kp)    = (unsigned int)s16_fix_kp;
         VAR(AX0_u16_Wns_Ki)    = (unsigned int)s16_fix_ki;
         VAR(AX0_u16_Wns_Out_Shr)= u16_shr_kp;
         return 1;
      }
      else
      {
         f_temp_den = 116.0;
         f_temp_num = (float)BGVAR(s32_V_Lim_Design) * 0.0001117587 * 754.0 * (float)BGVAR(s32_Motor_Mass);
         // if PHASEFINDI > IMAX then use IMAX
         if (BGVAR(s32_PhaseFind_Current) > BGVAR(s32_Imax))
                f_temp_den *= ((float)BGVAR(s16_PhaseFind_Gain) / 1000.0) * (float)BGVAR(u32_Motor_Kf) * (float)BGVAR(s32_Imax);
         else
                f_temp_den *= ((float)BGVAR(s16_PhaseFind_Gain) / 1000.0) * (float)BGVAR(u32_Motor_Kf) * (float)BGVAR(s32_PhaseFind_Current);
      }

      //factor for s32_Motor_Mass is:  (pitch ^ 2) / (2 * PIE)^2 = (pitch ^ 2) / 39.4786
      //factor for u32_Motor_Kf   is:  pitch / (2 * PIE) = pitch / 6.2832
      //sum of factor: pitch * 0.00015915
      f_temp = (f_temp_num / f_temp_den)  * 0.00015915 * (float)BGVAR(u32_Mpitch);//mpitch changed to decimal
   }

   // Calc kp
   FloatToFix16Shift16(&s16_fix_kp, &u16_shr_kp, f_temp);

   // Calc ki
   f_temp *= 0.1256;
   FloatToFix16Shift16(&s16_fix_ki, &u16_shr_ki, f_temp);

   // if shifts > 15 decrease kp
   if (u16_shr_kp > 15)
   {
      s16_fix_kp >>= (u16_shr_kp - 15);
      u16_shr_kp = 15;
   }

   // if shifts > 15 decrease ki
   if (u16_shr_ki > 15)
   {
      s16_fix_ki >>= (u16_shr_ki - 15);
      u16_shr_ki = 15;
   }

   if (s16_fix_ki == 0) BGVAR(u16_WnsError) |= WNS_DESIGN_FAILED; // ki always > kp => no need to test kp

   // Combine the shifts
   if (u16_shr_kp < u16_shr_ki)
   {
      s16_fix_ki >>= (u16_shr_ki - u16_shr_kp);
      u16_shr_ki = u16_shr_kp;
   }
   if (u16_shr_ki < u16_shr_kp)
   {
      s16_fix_kp >>= (u16_shr_kp - u16_shr_ki);
      u16_shr_kp = u16_shr_ki;
   }

   VAR(AX0_u16_Wns_Kp)    = (unsigned int)s16_fix_kp;
   VAR(AX0_u16_Wns_Ki)    = (unsigned int)s16_fix_ki;
   VAR(AX0_u16_Wns_Out_Shr)= u16_shr_kp;

   //VAR(AX0_s16_Crrnt_Run_Code) |= COPY_WNS_COEF_MASK;
   return 0;
}


void CalculatePulseDuration(int drive)
{
   long s32_temp;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // estimate pulse duration based on DIPEAK,ML and VBUS
   // use a factor of 0.5 for first pulse and decide on acual pulse size
   // according to the final result
   // current rise in one Sec = Vbus[V]/ML[Hn] * time[Sec] = Dipeak[A]
   // time[Sec] = Dipeak[A]*ML[Hn]/Vbus[V]
   // setup factor of 0.5.scale to FPGA clocks 1875 per 1 MTS
   // time[MTS] = Dipeak[A]*ML[Hn]/Vbus[V]/4*32000
   // div ML by 2 sine ML is line to line inductance
   // time[FPGA Clks] = Dipeak[A]*ML[Hn]/2/Vbus[V]/2*32000*1875
   s32_temp = (long)(((float)BGVAR(s32_Drive_I_Peak) * (float)BGVAR(u32_Mlmin) / ((float)BGVAR(s16_Vbus)) * 0.015 * BGVAR(f_Pulse_Factor)));
   if (s32_temp > 65535L) s32_temp = 65535L;

   BGVAR(u16_PhaseFind_Pulse_Duration) = (unsigned int)s32_temp;
}


int GeneratePhaseFindPulse(int drive, unsigned char u8_pulse_polarity, unsigned char u8_phase)
{
   // AXIS_OFF;

   unsigned int u16_delay_value, u16_pulse_ended = 0;
   long s32_time_capture;
   REFERENCE_TO_DRIVE;

   VAR(AX0_s16_PhaseFindRTBits) &= ~(PHASE_FIND_A_PULSE_MASK | PHASE_FIND_B_PULSE_MASK | PHASE_FIND_C_PULSE_MASK);
   if (u8_phase == PHASE_FIND_PHASE_A)
   {
      if (u8_pulse_polarity == PHASE_FIND_POS_PULSE)
         VAR(AX0_s16_PhaseFindRTBits) |= (PHASE_FIND_B_PULSE_MASK | PHASE_FIND_C_PULSE_MASK);
      else
         VAR(AX0_s16_PhaseFindRTBits) |= PHASE_FIND_A_PULSE_MASK;
   }

   if (u8_phase == PHASE_FIND_PHASE_B)
   {
      if (u8_pulse_polarity == PHASE_FIND_POS_PULSE)
         VAR(AX0_s16_PhaseFindRTBits) |= (PHASE_FIND_A_PULSE_MASK | PHASE_FIND_C_PULSE_MASK);
      else
         VAR(AX0_s16_PhaseFindRTBits) |= PHASE_FIND_B_PULSE_MASK;
   }

   if (u8_phase == PHASE_FIND_PHASE_C)
   {
      if (u8_pulse_polarity == PHASE_FIND_POS_PULSE)
         VAR(AX0_s16_PhaseFindRTBits) |= (PHASE_FIND_B_PULSE_MASK | PHASE_FIND_A_PULSE_MASK);
      else
         VAR(AX0_s16_PhaseFindRTBits) |= PHASE_FIND_C_PULSE_MASK;
   }

   if (u8_phase == PHASE_FIND_IDLE_PULSE)
   {
//    VAR(AX0_s16_PhaseFindRTBits) |= 0x0000;
   }

   u16_delay_value = BGVAR(u16_PhaseFind_Pulse_Duration);

   VAR(AX0_Phase_Find_MTS_Cntr) = 1;
   while (u16_delay_value > 1875)
   {
      u16_delay_value-=1875;
      ++VAR(AX0_Phase_Find_MTS_Cntr);
   }
   u16_delay_value = 1875-u16_delay_value;

   // write delay value to FPGA
   *((int*)FPGA_PHASE_FIND_DELAY_REG_ADD) = u16_delay_value;

   // write length value to FPGA
   *((int*)FPGA_PHASE_FIND_LENGTH_REG_ADD) = BGVAR(u16_PhaseFind_Pulse_Duration);

   // write phase commands
   *((int*)FPGA_PWM_PHASE_REG_ADD) = (VAR(AX0_s16_PhaseFindRTBits) & (PHASE_FIND_A_PULSE_MASK | PHASE_FIND_B_PULSE_MASK | PHASE_FIND_C_PULSE_MASK));

   // signal r.t to start the pulse
   VAR(AX0_s16_PhaseFindRTBits) |= PHASE_FIND_START_PULSE_MASK;

   s32_time_capture = Cntr_1mS;
   while ((!PassedTimeMS(10L, s32_time_capture)) && (!u16_pulse_ended))
   {
      u16_pulse_ended = (VAR(AX0_s16_PhaseFindRTBits) & PHASE_FIND_START_PULSE_MASK) == 0;
   }
   if (!u16_pulse_ended) return 0; // This will indicate a timeout

   s32_time_capture = Cntr_1mS;

   // Wait for the current reading to complete (or 10 msec timeout)
   while (!PassedTimeMS(10L, s32_time_capture))
   {
      if ( ((*((int*)FPGA_PHASE_FIND_START_REG_ADD) & 0x0001) == 0) &&
           (VAR(AX0_Phase_Find_MTS_Cntr) == 0)                        )
         return 1;
   }
   return 0;
}


//**********************************************************
// Function Name: PhaseFindCommand
// Description:
//          This function is called in response to the PhaseFind command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int PhaseFindCommand(int drive)
{
   // AXIS_OFF;
   if (VAR(AX0_u16_Motor_Comm_Type) != BRUSHLESS_MOTOR) return MOTOR_COMM_TYPE_INVALID;
   if ( !FEEDBACK_WITH_ENCODER && !SL_FEEDBACK  &&
        (BGVAR(u16_FdbkType) != FANUC_COMM_FDBK)  )
      return FDBKTYPE_MISMATCH;
   if ( (BGVAR(u8_Sl_Mode) == 0) && (BGVAR(u16_FdbkType) != FANUC_COMM_FDBK) )
   {
      if ( (VAR(AX0_s16_Motor_Enc_Type) != 1) && (VAR(AX0_s16_Motor_Enc_Type) != 2)  &&
           (VAR(AX0_s16_Motor_Enc_Type) != 3) && (VAR(AX0_s16_Motor_Enc_Type) != 4)  &&
           (VAR(AX0_s16_Motor_Enc_Type) != 11) && (VAR(AX0_s16_Motor_Enc_Type) != 12)  )
         return MENCTYPE_MISMATCH;
   }

   if (Enabled(drive)) return DRIVE_ACTIVE;
   if ( (BGVAR(u16_PhaseFind_Mode) == 0) || (BGVAR(u16_PhaseFind_Mode) == 3) )
      BGVAR(u8_SalFind_State) = SALFIND_WAIT_FOR_USER_EN;
   BGVAR(u8_PhaseFind_State) = PHASE_FIND_IDLE;
   BGVAR(u16_PhaseFind_Bits) |= PHASE_FIND_REQUESTED_MASK;
   BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_SUCCESS_MASK;

   return (SAL_SUCCESS);
}


int PhaseFindModeCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ( /*&& (param != 10LL)*/ (param != 11LL) && (param != 12LL) && (param != 2LL) && (param != 3LL) && (param != 4LL) && (param != 5LL) && (param != 22LL))
      return (VALUE_OUT_OF_RANGE);

   if ((param == 3LL) || (param == 22LL) || (param == 9LL)) return (NOT_AVAILABLE); // these modes are implemented but not tested yet
   BGVAR(u16_PhaseFind_Mode) = (unsigned int)param;

   // Added no-comp to make sure PHASEFIND mode is not used with KCMODE<6
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;

   return SAL_SUCCESS;
}


int PhaseFindStatusCommand(long long *data, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   *data = 1;

   if((BGVAR(u8_PhaseFind_State) == PHASE_FIND_IDLE) || (BGVAR(u8_PhaseFind_State) == PHASE_FIND_WAIT_FOR_EN))
   {
      if (BGVAR(s64_SysNotOk) & PHASE_FIND_FLT_MASK)
         *data = 3;
      else
         *data = 0;
   }
   else if (BGVAR(u8_PhaseFind_State) == PHASE_FIND_FAULT_STATE)
      *data = 3;

   if (BGVAR(u16_PhaseFind_Bits) & PHASE_FIND_SUCCESS_MASK) *data = 2;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: PhaseFindCurrentCommand
// Description:
//          This function is called in response to the PHASEFINDI command.
//
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int PhaseFindCurrentCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if (lparam > (long long)BGVAR(s32_Drive_I_Peak)) return (VALUE_TOO_HIGH);
   BGVAR(s32_PhaseFind_Current) = (long)lparam;

   VAR(AX0_s16_Phase_Find_Current) = (int)(((long)lparam * 26214L) / BGVAR(s32_Drive_I_Peak));

   return (SAL_SUCCESS);
}


int PhaseFindTimeCommand(long long param, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   BGVAR(u16_PhaseFind_Time) = (unsigned int)param;

   LVAR(AX0_u32_Wns_Time) = (unsigned long)(param << 1LL);

   return (SAL_SUCCESS);
}


int PhaseFindDeltaCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   BGVAR(u16_PhaseFind_Delta) = (int)param;

   return (SAL_SUCCESS);
}


// WNSERR command
int WNSErrorCommand(int drive)
{
   static int wnserr_state = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;

   switch (wnserr_state)
   {
      case 0:
         if (BGVAR(u16_WnsError) & WNS_RAN)
         {
            PrintStringCrLf("PhaseFind executed", 0);
            strcpy((char*)manu_spec_WNS_Error_command,"PhaseFind executed");
         }
         else
         {
            PrintStringCrLf("PhaseFind not executed", 0);
            strcpy((char*)manu_spec_WNS_Error_command,"PhaseFind not executed");
         }

         if (BGVAR(u16_WnsError) & WNS_STOPPED)
         {
            PrintStringCrLf("Procedure aborted", 0);
            strcpy((char*)manu_spec_WNS_Error_command, "Procedure aborted");
         }

         if (BGVAR(u16_WnsError) & WNS_FAILED_MASK)
         {
            PrintStringCrLf("PhaseFind Failed", 0);
            strcpy((char*)manu_spec_WNS_Error_command, "PhaseFind Failed");
         }
      break;

      case 1:
         if (BGVAR(u16_WnsError) & MAX_VEL_ERROR)
         {
            PrintStringCrLf("Velocity too high", 0);
            strcpy((char*)manu_spec_WNS_Error_command, "Velocity too high");
         }

         if (BGVAR(u16_WnsError) & TOO_MUCH_MOTION)
         {
            PrintStringCrLf("Too much motion detected", 0);
            strcpy((char*)manu_spec_WNS_Error_command, "Too much motion detected");
         }
      break;

      case 2:
         if (BGVAR(u16_WnsError) & TOO_LITTLE_MOTION)
         {
            PrintStringCrLf("Not enough motion detected. Please check mechanics or PHASEFINDI", 0);
            strcpy((char*)manu_spec_WNS_Error_command, "Not enough motion detected. Please check mechanics or PHASEFINDI");
         }

         if (BGVAR(u16_WnsError) & MOTION_PROFILE)
         {
            PrintStringCrLf("Bad motion profile detected", 0);
            strcpy((char*)manu_spec_WNS_Error_command, "Bad motion profile detected");
         }
      break;

      case 3:
         if (BGVAR(u16_WnsError) & WNS_DESIGN_FAILED)
         {
            PrintStringCrLf("PhaseFind (MODE 2) Design Failed", 0);
            strcpy((char*)manu_spec_WNS_Error_command, "PhaseFind (MODE 2) Design Failed");
         }
         if (BGVAR(u16_WnsError) & TOO_MUCH_OSC)
         {
            PrintStringCrLf("Too much oscillation detected", 0);
            strcpy((char*)manu_spec_WNS_Error_command, "Too much oscillation detected");
         }
      break;

      case 4:
         if (BGVAR(u16_WnsError) & SETTLE_TIMEOUT)
         {
            PrintStringCrLf("Motor Settling Timeout", 0);
            strcpy((char*)manu_spec_WNS_Error_command, "Motor Settling Timeout");
         }
         if (BGVAR(u16_WnsError) & FOLDBACK_OCCURED)
         {
            PrintStringCrLf("Foldback occured during Phasefind", 0);
            strcpy((char*)manu_spec_WNS_Error_command, "Foldback occured during Phasefind");
         }
      break;
   }

   wnserr_state++;
   if (wnserr_state > 4)
   {
      wnserr_state = 0;
      return (SAL_SUCCESS);
   }
   else return (SAL_NOT_FINISHED);
}

int is_Motion_Detected(int drive, int u16_elect_degree)
{
    // AXIS_OFF;
    int s16_temp, retval = 0;
    long long s64_temp, s64_temp2;
    REFERENCE_TO_DRIVE;
    // internal elect angle = elangle * 2^32 / 360 = elangle * 11930464.711...
    s64_temp2 = (long long)((float)u16_elect_degree * 11930464.711);
    // Test motion > 45 elect degrees
    do {
       s16_temp = Cntr_3125;
       s64_temp = LLVAR(AX0_u32_Pos_Fdbk_User_Lo) - LLVAR(AX0_u32_Wns_Pfb_0_Lo);
    } while (s16_temp != Cntr_3125);

    if (s64_temp < 0LL) s64_temp = -s64_temp;      // Make sure its positive
    s64_temp *= (long long)VAR(AX0_s16_Num_Of_Poles_Ratio);
    if (s64_temp > s64_temp2) retval = 1;
    // return "1" if motion is detected. Delta motion is defined by PHASEFINDDELTA parameter in electrical degree
    return retval;
}
