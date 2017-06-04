#include <string.h>
#include "co_util.h"
#include "co_type.h"
#include "access.h"
#include "objects.h"

#include "Math.h"

#include "Current.def"
#include "Design.def"
#include "Err_Hndl.def"
#include "Exe_IO.def"
#include "Fltcntrl.def"
#include "i2c.def"
#include "ModCntrl.def"
#include "MultiAxis.def"
#include "Position.def"
#include "ser_comm.def"
#include "Velocity.def"
#include "PtpGenerator.def"
#include "Init.def"
#include "402fsa.def"

#include "AutoTune.var"
#include "Drive.var"
#include "Endat.var"
#include "Exe_IO.var"
#include "Extrn_Asm.var"
#include "Fltcntrl.var"
#include "ModCntrl.var"
#include "Motor.var"
#include "Position.var"
#include "PtpGenerator.var"
#include "Ser_Comm.var"
#include "Units.var"
#include "User_Var.var"
#include "Velocity.var"
#include "Foldback.var"
#include "Lxm_profile.var"
//#include "MotorSetup.var"
#include "MotorParamsEst.var"
#include "Prototypes.pro"


// Internal position feedback value is stored as 32.32
// This scaling is used to convert whatever feedback with its resolution to 32 PRD value
int CalcPosFbFilterGain(int drive)
{
   // AXIS_OFF;
   long long s64_temp = 0LL;
   float f_count_in_32bits = 0.0;

   switch (BGVAR(u16_FdbkType))
   {
      case SW_SINE_FDBK:
         if ( (GetBitNumber(BGVAR(u32_User_Motor_Enc_Res)) + BGVAR(u16_Sine_Interpolation)) > 29)  // calc interpolation to get AX0_s32_Counts_Per_Rev max positive
            BGVAR(u16_Sine_Interpolation) = 29 - GetBitNumber(BGVAR(u32_User_Motor_Enc_Res));
//         VAR(AX0_s16_Sw_Sine_Shr) = 0; //In CDHD decided remove MSININT support and use max resolution
//         Reduce Interpolation Level to ensure that Bits per Revolution is less than 2^31.
         VAR(AX0_s16_Sw_Sine_Shr) = 16 - BGVAR(u16_Sine_Interpolation);
         VAR(AX0_s16_Sw_Sine_Frac_Mask) = (1 << VAR(AX0_s16_Sw_Sine_Shr)) - 1;
         LVAR(AX0_s32_Counts_Per_Rev) = BGVAR(u32_User_Motor_Enc_Res) << (long)BGVAR(u16_Sine_Interpolation);
         LVAR(AX0_s32_Counts_Per_Rev_2) = (LVAR(AX0_s32_Counts_Per_Rev) >> 1L);
         s32_Sdo_Counts_Per_Rev = LVAR(AX0_s32_Counts_Per_Rev);
         // Shift according to Sine-Interpolation Depth.
      break;

      case INC_ENC_FDBK:
         if (VAR(AX0_s16_Motor_Enc_Type) == 5)
         {
            LVAR(AX0_s32_Counts_Per_Rev) = 0x0008000 * (long)BGVAR(u16_Mpoles); // 0x0010000
            s32_Sdo_Counts_Per_Rev = LVAR(AX0_s32_Counts_Per_Rev);
            LVAR(AX0_s32_Counts_Per_Rev_2) = 0x00004000 * (long)BGVAR(u16_Mpoles); // 0x0008000
         }
         else
         {
            LVAR(AX0_s32_Counts_Per_Rev) = BGVAR(u32_User_Motor_Enc_Res) * BGVAR(u16_Motor_Enc_Interpolation) << 2L;
            s32_Sdo_Counts_Per_Rev = BGVAR(u32_User_Motor_Enc_Res) << 2L;
            LVAR(AX0_s32_Counts_Per_Rev_2) = (LVAR(AX0_s32_Counts_Per_Rev) >> 1L);
            if (0 < (VAR(AX0_s16_DF_Run_Code) & DUAL_LOOP_CHANGE_PFB_RES_MASK))
            {
			//here we change AX0_u32_Fdbk_Accu with the same ratio that AX0_s32_Counts_Per_Rev has changed (due to u16_Motor_Enc_Interpolation change)
			//this is done to avoid a change of the electric position                
			   VAR(AX0_u32_Fdbk_Accu) = (VAR(AX0_u32_Fdbk_Accu) * (16 + BGVAR(s16_Motor_Enc_Interpolation_Delta))) >> 4;
               BGVAR(s16_Motor_Enc_Interpolation_Delta) = 0;
               BGVAR(u16_Init_Commutation_Fault_Flag) = 1; // initialize the AqB fault detection SM
               VAR(AX0_s16_DF_Run_Code) &= ~DUAL_LOOP_CHANGE_PFB_RES_MASK;
            }
         }
       break;

      case NK_COMM_FDBK:
      case TAMAGAWA_COMM_MULTI_TURN_FDBK:
      case TAMAGAWA_COMM_SINGLE_TURN_FDBK:
      case TAMAGAWA_CID0_SINGLE_TURN:
      case PS_P_G_COMM_FDBK:
      case PS_S_COMM_FDBK:
      case FANUC_COMM_FDBK:
      case ENDAT2X_COMM_FDBK:
      case SERVOSENSE_SINGLE_TURN_COMM_FDBK:
      case SERVOSENSE_MULTI_TURN_COMM_FDBK:
      case SANKYO_COMM_FDBK:
      case YASKAWA_ABS_COMM_FDBK:
      case BISSC_COMM_FDBK:
      case YASKAWA_INC_COMM_FDBK:
         LVAR(AX0_s32_Counts_Per_Rev) = BGVAR(u32_User_Motor_Enc_Res) * BGVAR(u16_Motor_Enc_Interpolation) << 2L;
         LVAR(AX0_s32_Counts_Per_Rev_2) = LVAR(AX0_s32_Counts_Per_Rev) >> 1L;
         s32_Sdo_Counts_Per_Rev = BGVAR(u32_User_Motor_Enc_Res) << 2L;
         
         if (0 < (VAR(AX0_s16_DF_Run_Code) & DUAL_LOOP_CHANGE_PFB_RES_MASK))
         {
	 	 //here we change AX0_u32_Fdbk_Accu with the same ratio that AX0_s32_Counts_Per_Rev has changed (due to u16_Motor_Enc_Interpolation change)
		 //this is done to avoid a change of the electric position                
		    VAR(AX0_u32_Fdbk_Accu) = (VAR(AX0_u32_Fdbk_Accu) * (16 + BGVAR(s16_Motor_Enc_Interpolation_Delta))) >> 4;
            BGVAR(s16_Motor_Enc_Interpolation_Delta) = 0;
            BGVAR(u16_Init_Commutation_Fault_Flag) = 1; // initialize the AqB fault detection SM
            VAR(AX0_s16_DF_Run_Code) &= ~DUAL_LOOP_CHANGE_PFB_RES_MASK;
         }
      break;

      case SL_FDBK:
         LVAR(AX0_s32_Counts_Per_Rev) = 0x00010000 * (long)VAR(AX0_s16_Num_Of_Poles_Ratio);
         s32_Sdo_Counts_Per_Rev = LVAR(AX0_s32_Counts_Per_Rev);
         LVAR(AX0_s32_Counts_Per_Rev_2) = 0x00008000 * (long)VAR(AX0_s16_Num_Of_Poles_Ratio);
      break;

      case RESOLVER_FDBK:
         LVAR(AX0_s32_Counts_Per_Rev) = 0x00010000L * BGVAR(u16_Resolver_Poles) >> 1;
         s32_Sdo_Counts_Per_Rev = LVAR(AX0_s32_Counts_Per_Rev);
         // Counts_Per_Rev_2 unrelated to Counts_Per_Rev, only used for Rollover Calc.
         LVAR(AX0_s32_Counts_Per_Rev_2) = 0x00008000;
       break;
   }

   LVAR(AX0_u32_Counts_Per_Rev) = LVAR(AX0_s32_Counts_Per_Rev);
   LVAR(AX0_u32_Counts_Per_Rev_Half) = LVAR(AX0_s32_Counts_Per_Rev_2);

   LVAR(AX0_u32_Counts_Per_Rev_2) = ((BGVAR(u32_User_Sec_Enc_Res) * (unsigned long)BGVAR(u16_Load_Enc_Interpolation)) << 2);
   LVAR(AX0_u32_Counts_Per_Rev_Half_2) = (LVAR(AX0_u32_Counts_Per_Rev_2) >> 1L);   
  

   if ( (LVAR(AX0_s32_Counts_Per_Rev) != BGVAR(s32_Prev_Counts_Per_Rev))   ||
        (BGVAR(u32_Prev_Sec_Counts_Per_Rev) != (BGVAR(u32_User_Sec_Enc_Res) * (unsigned long)BGVAR(u16_Load_Enc_Interpolation)))  )
   {
      // Dont allow Encoder Simulation handling
      VAR(AX0_u16_Encsim_Freq_Flags) = (INHIBIT_ENC_SIM_MASK | INHIBIT_ENC_SIM_PFB_NOT_READY_MASK | SKIP_FLT_CHECK_MASK);

      // Normalize to 32bit
      OneDivS64ToFixU32Shift16((long*)&LVAR(AX0_s32_Pfb_Scale_Fix),(unsigned int *)&VAR(AX0_s16_Pfb_Scale_Shr),(long long)LVAR(AX0_s32_Counts_Per_Rev));
      VAR(AX0_s16_Pfb_Scale_Shr) -= 32; //same as multipling by 2^32
      // in order to make sure that the pcmd when standing is without counts fractures
      // calc counts_per_rev/2^32
      LVAR(AX0_u32_Pfb_Scale_Fix) = LVAR(AX0_s32_Pfb_Scale_Fix);
      VAR(AX0_u16_Pfb_Scale_Shr) = VAR(AX0_s16_Pfb_Scale_Shr);

      OneDivS64ToFixU32Shift16((long*)&LVAR(AX0_u32_Pfb_Scale_Fix_2),(unsigned int *)&VAR(AX0_u16_Pfb_Scale_Shr_2),(long long)LVAR(AX0_u32_Counts_Per_Rev_2));
      VAR(AX0_u16_Pfb_Scale_Shr_2) -= 32; //same as multipling by 2^32
      // in order to make sure that the pcmd when standing is without counts fractures
      // calc counts_per_rev/2^32

      PositionModuloConfig(drive);

      if (BGVAR(u16_FdbkType) == RESOLVER_FDBK)
      {
         f_count_in_32bits = 65536.0; // = 2^32 / 65536
         LVAR(AX0_u32_1Count_In_32Bits) = (long)f_count_in_32bits;
         FloatToFix16Shift16(&VAR(AX0_s16_Pcmd_Round_Fix),(unsigned int *)&VAR(AX0_s16_Pcmd_Round_Shr),1.0/f_count_in_32bits);
         OneDivS64ToFixU32Shift16((long*)&LVAR(AX0_u32_Pcmd_Round2_Fix),(unsigned int *)&VAR(AX0_s16_Pcmd_Round2_Shr),65536LL);
      }
      else if (BGVAR(u16_FdbkType) == SW_SINE_FDBK)
      {
//         f_count_in_32bits = 16384.0 / (float)(BGVAR(u32_User_Motor_Enc_Res)); // = 2^32 / (4*MENCRES*2^16)
         f_count_in_32bits = 1073741824.0 / (float)(LVAR(AX0_s32_Counts_Per_Rev)); // = 2^32 / (4*MENCRES*2^u16_Sine_Interpolation) = 2^32 / (4*AX0_s32_Counts_Per_Rev)
         LVAR(AX0_u32_1Count_In_32Bits) = (long)f_count_in_32bits;
         FloatToFix16Shift16(&VAR(AX0_s16_Pcmd_Round_Fix),(unsigned int *)&VAR(AX0_s16_Pcmd_Round_Shr),1.0/f_count_in_32bits);
//         OneDivS64ToFixU32Shift16((long*)&LVAR(AX0_u32_Pcmd_Round2_Fix),(unsigned int *)&VAR(AX0_s16_Pcmd_Round2_Shr),((long long)(BGVAR(u32_User_Motor_Enc_Res)) << 18));
         OneDivS64ToFixU32Shift16((long*)&LVAR(AX0_u32_Pcmd_Round2_Fix),(unsigned int *)&VAR(AX0_s16_Pcmd_Round2_Shr),((long long)(LVAR(AX0_s32_Counts_Per_Rev)) << 2));
      }
      else // encoder
      {
         f_count_in_32bits = 1073741824.0 / (float)(BGVAR(u32_User_Motor_Enc_Res)); // = 2^32 / (4*MENCRES)
         LVAR(AX0_u32_1Count_In_32Bits) = (long)f_count_in_32bits;
         FloatToFix16Shift16(&VAR(AX0_s16_Pcmd_Round_Fix),(unsigned int *)&VAR(AX0_s16_Pcmd_Round_Shr),1.0/f_count_in_32bits);
         OneDivS64ToFixU32Shift16((long*)&LVAR(AX0_u32_Pcmd_Round2_Fix),(unsigned int *)&VAR(AX0_s16_Pcmd_Round2_Shr),((long long)(BGVAR(u32_User_Motor_Enc_Res)) << 2));
      }

      s64_temp = ((long long)LVAR(AX0_u32_1Count_In_32Bits)) << (long long)BGVAR(u16_Mencres_Shr);
      if (s64_temp > 0x7FFFFFFF) s64_temp = (long long)LVAR(AX0_u32_1Count_In_32Bits);
      LVAR(AX0_u32_1Count_In_32Bits) = (unsigned long)s64_temp;

      VAR(AX0_s16_Pcmd_Round2_Shr) -= 32; //same as multipling by 2^32
      VAR(AX0_s16_Pcmd_Round_Shr) = VAR(AX0_s16_Pcmd_Round_Shr) - 1;

      //allow Encoder Simulation handling
      VAR(AX0_u16_Encsim_Freq_Flags) &= ~INHIBIT_ENC_SIM_MASK;
   }
   BGVAR(s32_Prev_Counts_Per_Rev) = LVAR(AX0_s32_Counts_Per_Rev);
   BGVAR(u32_Prev_Sec_Counts_Per_Rev) =( BGVAR(u32_User_Sec_Enc_Res) * (unsigned long)BGVAR(u16_Load_Enc_Interpolation));

   return (SUCCESS);
}

int SalVfiltFrqCommand(long long param,int drive)
{
   BGVAR(u16_Vfilt_Frq) = (unsigned int)param;
   CalcFbFilterGain(drive);
   return (SAL_SUCCESS);
}

void CalcFbFilterGain(int drive)
{
   // AXIS_OFF;
   float f_vlim_rpm = (float)BGVAR(s32_V_Lim_Design) * 0.00011176;
   long s32_temp;

   CalcPosFbFilterGain(drive);

    // calc the speed observer coeffs
   VAR(AX0_s16_SpdObsrvr_G_Scale_Factor_Fix) = 0;

   //2.5425/Vlim (convert from Delta(fbc32) to spd internal 1/2^32 * 1/Tsv * 60 * 22750/Vlim
   FloatToFix16Shift16(&VAR(AX0_s16_SpdObsrvr_Scale_Factor_Fix),(unsigned int *)&VAR(AX0_s16_SpdObsrvr_Scale_Factor_Shr),2.5425/f_vlim_rpm);
   LVAR(AX0_s32_SpdObsrvr_Scale_Factor_Mask) = (1L << VAR(AX0_s16_SpdObsrvr_Scale_Factor_Shr)) - 1L;

   // dead zone - one encoder count/Ts = spd of 1*1/Ts*60/BGVAR(u32_User_Motor_Enc_Res)/4*22750/Vlim
   //                                  = 2730000000/BGVAR(u32_User_Motor_Enc_Res)/Vlim
   s32_temp = 2730000000.0/(float)(BGVAR(u32_User_Motor_Enc_Res))/f_vlim_rpm;
   if (s32_temp > 32767L) s32_temp = 32767;
   if (s32_temp == 0) s32_temp = 1;
   VAR(AX0_s16_Dead_Zone) = (int)s32_temp;

   VAR(AX0_s16_SpdObsrvr_Slow_Beta) = 0x056A;
   VAR(AX0_s16_SpdObsrvr_Slow_Alpha) = 0x7A96;

   //Vlim/2.5425
   FloatToFix16Shift16(&VAR(AX0_s16_SpdObsrvr_Int_Factor_Fix),(unsigned int *)&VAR(AX0_s16_SpdObsrvr_Int_Factor_Shr),f_vlim_rpm/2.5425);
   VAR(AX0_s16_SpdObsrvr_Int_Factor_Mask) = (1 << VAR(AX0_s16_SpdObsrvr_Int_Factor_Shr)) - 1;

   if (BGVAR(u8_VelFiltMode) == 0)
   {
      VAR(AX0_s16_Dead_Zone) = 0x7fff;
      VAR(AX0_s16_SpdObsrvr_Slow_Beta) = 0x7FFF;
      VAR(AX0_s16_SpdObsrvr_Slow_Alpha) = 1;
   }
   else if (BGVAR(u8_VelFiltMode) == 1)
   {
      VAR(AX0_s16_Dead_Zone) = 0x7fff;


      // 440Hz alpha = 0.7078
      VAR(AX0_s16_SpdObsrvr_Slow_Alpha) = (int)(32768*exp(-(float)BGVAR(u16_Vfilt_Frq)*7.8539816339744830961566084581988e-4));
      VAR(AX0_s16_SpdObsrvr_Slow_Beta) = 0x8000 - VAR(AX0_s16_SpdObsrvr_Slow_Alpha);
   }
   else if (BGVAR(u8_VelFiltMode) == 3)
   {
      //2.5425/Vlim
      FloatToFix16Shift16(&VAR(AX0_s16_SpdObsrvr_Scale_Factor_Fix),(unsigned int *)&VAR(AX0_s16_SpdObsrvr_Scale_Factor_Shr),2.5425/f_vlim_rpm);
      LVAR(AX0_s32_SpdObsrvr_Scale_Factor_Mask) = (1L << VAR(AX0_s16_SpdObsrvr_Scale_Factor_Shr)) - 1L;
      //0.0254/Vlim
      FloatToFix16Shift16(&VAR(AX0_s16_SpdObsrvr_G_Scale_Factor_Fix),(unsigned int *)&VAR(AX0_s16_SpdObsrvr_G_Scale_Factor_Shr),0.0254/f_vlim_rpm);

      // 6000/Mencres
      // improve resolution result = res_fix,shr
      FloatToFix16Shift16(&VAR(AX0_s16_Dead_Zone),(unsigned int *)&VAR(AX0_s16_SpdObsrvr_DZ_Shr),(24000.0/((float)(LVAR(AX0_s32_Counts_Per_Rev)))));

      VAR(AX0_s16_SpdObsrvr_G_Scale_Factor_Shr) = VAR(AX0_s16_SpdObsrvr_G_Scale_Factor_Shr) - VAR(AX0_s16_SpdObsrvr_DZ_Shr);
      if (VAR(AX0_s16_SpdObsrvr_G_Scale_Factor_Shr) < 0)
      {
        VAR(AX0_s16_Dead_Zone) = VAR(AX0_s16_Dead_Zone) >> (-VAR(AX0_s16_SpdObsrvr_G_Scale_Factor_Shr));

        VAR(AX0_s16_SpdObsrvr_DZ_Shr) = VAR(AX0_s16_SpdObsrvr_G_Scale_Factor_Shr) + VAR(AX0_s16_SpdObsrvr_DZ_Shr);
        VAR(AX0_s16_SpdObsrvr_G_Scale_Factor_Shr) = 0;
      }
      LVAR(AX0_s32_SpdObsrvr_G_Scale_Factor_Mask) = (1L << (long)VAR(AX0_s16_SpdObsrvr_G_Scale_Factor_Shr)) - 1L;
      //VAR(AX0_s16_SpdObsrvr_DZ_Mask) = (1 << VAR(AX0_s16_SpdObsrvr_DZ_Shr)) - 1;
      //Vlim/2.5425
      FloatToFix16Shift16(&VAR(AX0_s16_SpdObsrvr_Int_Factor_Fix),(unsigned int *)&VAR(AX0_s16_SpdObsrvr_Int_Factor_Shr),f_vlim_rpm/2.5425);
      VAR(AX0_s16_SpdObsrvr_Int_Factor_Mask) = (1 << VAR(AX0_s16_SpdObsrvr_Int_Factor_Shr)) - 1;
   }

   SpeedObserverDesign(drive);
   SpeedObserverReset(drive);
}

int UpdateVelocityLimits(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   /* This function is called to update the VMAX, VLIM, and VOSPD parameters,
   *  as a result of a change in either VBUS, MKT, MSPEED, or MENCRES.
   *
   * VMAX equals to the minimum of the following terms:
   *
   *  1. MSPEED
   *
   *
   *  2. max speed due to implementation is 0.5 (half of 32 bit) rev per sample time (125 uS)
   *     => 0.5rev * 8000 * 60 = 240000 rpm
   *
   *  In addition : Limit VLIM to VMAX.
   *
   *  velocity internal units : counts32/sample rate
   */

   BGVAR(s32_Vmax) = (long)BGVAR(u32_Mspeed);

   // make sure s32_Vmax>10 rpm
   if (BGVAR(s32_Vmax) < 89478L) BGVAR(s32_Vmax) = 89478L;

   // limit vlim to vmax
   if (BGVAR(s32_V_Lim_Design)>BGVAR(s32_Vmax)) BGVAR(s32_V_Lim_Design) = BGVAR(s32_Vmax);

   // Update parameters that are dependent on VLIM
   return (UpdateAfterVlim(DRIVE_PARAM));
}

int UpdateAfterVlim(int drive)
{
   CalcFbFilterGain(DRIVE_PARAM);
   SpeedPhaseAdvDesign(DRIVE_PARAM);
   UpdateAccDec(drive,ACC_UPDATE);
   UpdateAccDec(drive,DEC_UPDATE);
   ConvertWeightCompVelToInternalInTheLoop(DRIVE_PARAM);
   UpdateNCTVelLoopScale(drive);
   UpdateSLScale(drive);
   UpdateFieldbusVelocityLimit(drive);

   UpdateVelLimitPiLoopParams(drive);  // Update the parameters of the PI-loop, which prevents the
                                       // motor from overspeed in OPMODE torque (SPDLM feature for Lexium)
   CalcSpeedCmdLimitTable(drive); // Check if the speed limits of P1-09, P1-10 or P1-11
                                  // exceed the new VLIM setting

   CalcVelThresholdInLoop(drive);

   //SalWriteZeroSpeedWindowCommand(BGVAR(s32_P1_38_Zero_Speed_Window_Out_Loop), drive);
   return (VelocityConfig(DRIVE_PARAM));
}


//**********************************************************
// Function Name: CalcVelThresholdInLoop
// Description:
//    This function convert V threshold value to in_lopp vel units.
//    Used in drive profile lexium mode.
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
void CalcVelThresholdInLoop(int drive)
{
   // AXIS_OFF;
   long long s64_half_for_rounding = 0;
   REFERENCE_TO_DRIVE;

   // convert from internal velocity (out of the loop) to internal units (in the loop)
   if (VAR(AX0_u16_Out_To_In_Vel_Shr) > 0)
      s64_half_for_rounding = 1LL << ((long long)VAR(AX0_u16_Out_To_In_Vel_Shr) - 1);

   BGVAR(u16_Lxm_Vel_Threshold_In_Loop) = (unsigned int)(((BGVAR(u32_Lxm_Vel_Threshold_Can_Obj) * (long long)LVAR(AX0_s32_Out_To_In_Vel_Fix)) + s64_half_for_rounding) >> (long long)VAR(AX0_u16_Out_To_In_Vel_Shr));
}

int ConvertVelocityToRpm(int drive,int vel)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   /* RPM = vel_in_loop * VLIM * 60 * 8000 / 2^32 / 22750 = vel_in_loop * VLIM / 203563554.13333 */
   return (int)((float)vel*(float)BGVAR(s32_V_Lim_Design) / 203563554.13333);
}

void PIPDFFDesign(int drive)
{
   float f_kp_scale_factor,f_ki_scale_factor,f_design_kvfr,f_ki,f_kv,f_vlim;
   unsigned long u32_pitch = 1000L;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == LINEAR_MOTOR) u32_pitch = BGVAR(u32_Mpitch);

   //move vlim from internal to user
   f_vlim = (float)BGVAR(s32_V_Lim_Design) * 0.00011176 * ((float)u32_pitch/1000.0);

   if (BGVAR(u32_Kv) == 0)
   {
      BGVAR(f_H_Poly)[0] = BGVAR(f_H_Poly)[1] = BGVAR(f_H_Poly)[2] = BGVAR(f_H_Poly)[3] = 0;
      BGVAR(f_R_Poly)[0] = BGVAR(f_R_Poly)[1] = BGVAR(f_R_Poly)[2] = 0;
      BGVAR(f_D_Poly)[1] = BGVAR(f_D_Poly)[2] = BGVAR(f_D_Poly)[3] = BGVAR(f_D_Poly)[4] = BGVAR(f_D_Poly)[5] = 0;
      return;
   }

   // scale from user to internal speed and current = 26214/dipeak/(22750/vlim)/60
   //                                               = 0.0192*Vlim/dipeak
   f_kp_scale_factor = 19.2044*f_vlim/(float)(BGVAR(s32_Drive_I_Peak));
   f_ki_scale_factor = 0.000125;//=Tsv

   if (BGVAR(u8_CompMode) == 0) f_design_kvfr = 1.0;
   else f_design_kvfr = (((float)BGVAR(s16_Kvfr))/1000.0);

   if (f_design_kvfr < 0.0) f_design_kvfr = 0.0;
   else if (f_design_kvfr>1.0) f_design_kvfr = 1.0;

   f_kv = ((float)BGVAR(u32_Kv))/1000.0;
   f_ki = ((float)BGVAR(u32_Kvi))/1000.0;

   BGVAR(f_D_Poly)[1] = -1.0;
   BGVAR(f_R_Poly)[0] = (f_design_kvfr+f_ki*f_ki_scale_factor)*f_kv*f_kp_scale_factor;
   BGVAR(f_R_Poly)[1] =  -f_design_kvfr*f_kv*f_kp_scale_factor;
   BGVAR(f_H_Poly)[0] = (1.0+f_ki*f_ki_scale_factor)*f_kv*f_kp_scale_factor;
   BGVAR(f_H_Poly)[1] = -f_kv*f_kp_scale_factor;

   if (BGVAR(u32_Kvi) == 0) BGVAR(f_R_Poly)[1] = BGVAR(f_H_Poly)[1] = BGVAR(f_D_Poly)[1] = 0;

   BGVAR(f_H_Poly)[2] = BGVAR(f_H_Poly)[3] = 0;
   BGVAR(f_R_Poly)[2] = 0;
   BGVAR(f_D_Poly)[2] = BGVAR(f_D_Poly)[3] = BGVAR(f_D_Poly)[4] = BGVAR(f_D_Poly)[5] = 0;
}


void PolePlacmentDesign(int drive)
{
   float f_fmech = 0.5,//Hz
   f_c1,f_c2,f_c3,f_c4,f_c1_tag,f_c2_tag,f_c3_tag,f_a1,f_tf,f_tf_factor,
   f_kb3,f_h0_temp,f_h1_temp,f_r0_temp,f_r1_temp,f_r_scale,f_h_scale,f_hr_scale,f_vlim_rpm;

   unsigned long u32_mpitch = 1000L;
   unsigned int u16_far_freq; // Hz

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
      u32_mpitch = BGVAR(u32_Mpitch);

   f_vlim_rpm = (float)BGVAR(s32_V_Lim_Design) * 0.00011176 *((float)u32_mpitch/1000.0);//mpitch changed to decimal

   if (BGVAR(u8_CompMode) == 2) u16_far_freq = 250;
   else u16_far_freq = BGVAR(u16_Bw)<<2;

   if (u16_far_freq>500) u16_far_freq = 500;

   f_c1_tag = -2*exp(-(float)(u16_far_freq)*0.0006283)*cos((float)(u16_far_freq)*0.00047125);
   f_c2_tag = exp(-(float)(u16_far_freq)*0.0012566);
   f_c3_tag = -exp(-(float)(BGVAR(u16_Bw))*0.000785);

   f_c1 = f_c1_tag+2*f_c3_tag;
   f_c2 = 2*f_c1_tag*f_c3_tag + f_c2_tag + f_c3_tag*f_c3_tag;
   f_c3 = f_c1_tag*f_c3_tag*f_c3_tag + 2*f_c2_tag*f_c3_tag;
   f_c4 = f_c2_tag*f_c3_tag*f_c3_tag;

   f_a1 = -exp(-2*3.1416*f_fmech*0.000125);

   BGVAR(f_D_Poly)[1] = f_c1-f_a1;
   BGVAR(f_D_Poly)[2] = f_c2-f_a1*BGVAR(f_D_Poly)[1];
   BGVAR(f_D_Poly)[3] = -1-BGVAR(f_D_Poly)[2]-BGVAR(f_D_Poly)[1];
   BGVAR(f_D_Poly)[4] = BGVAR(f_D_Poly)[5] = 0;

   f_kb3 = (1+f_a1)*0.0125/0.8/3/3.1416; // 0.0125 = TsV*100

   f_h0_temp = (f_c3-BGVAR(f_D_Poly)[3]-f_a1*BGVAR(f_D_Poly)[2])/f_kb3;
   f_h1_temp = (f_c4 - f_a1*BGVAR(f_D_Poly)[3])/f_kb3;

   f_r_scale = f_vlim_rpm * 6.001465e-6; // vlim/60*Tsv*65536/16384*16384/22750

   f_r0_temp = (f_h0_temp + f_h1_temp)/(1-exp(-2*3.1416*(float)BGVAR(u16_Bw)*0.000125))*f_r_scale;
   f_r1_temp = -f_r0_temp*exp(-2*3.1416*(float)BGVAR(u16_Bw)*0.000125);

   f_h_scale = f_r_scale;
   f_h0_temp = f_h0_temp * f_h_scale;
   f_h1_temp = f_h1_temp * f_h_scale;

   // mj/10000 (and MMASS/10) is not understood compared to xls design (this came from matlab sim)
   if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
      f_hr_scale = f_vlim_rpm*(((float)BGVAR(s32_Motor_Mass) / 10.0 * (1+(float)BGVAR(u32_LMJR_User)/1000.0)/((float)BGVAR(u32_Motor_Kf)*1000.0)) * (float)(((float)BGVAR(u32_Mpitch) / 1000000.0) * 0.15915)) / (((float)BGVAR(s32_Drive_I_Peak)/sqrt(2.0))/100.0);//mpitch changed to decimal
   else
      f_hr_scale = f_vlim_rpm*(float)BGVAR(s32_Motor_J)/10000.0*(1.0+((float)BGVAR(u32_LMJR_User))/1000.0)/( (float)BGVAR(u32_Motor_Kt) * (((float)BGVAR(s32_Drive_I_Peak)/sqrt(2.0))/100.0) );

   BGVAR(f_R_Poly)[0] = f_r0_temp * f_hr_scale;
   BGVAR(f_R_Poly)[1] = f_r1_temp * f_hr_scale;
   BGVAR(f_R_Poly)[2] = 0;

   BGVAR(f_H_Poly)[0] = f_h0_temp * f_hr_scale;
   BGVAR(f_H_Poly)[1] = f_h1_temp * f_hr_scale;
   BGVAR(f_H_Poly)[2] = BGVAR(f_H_Poly)[3] = 0;

   f_tf = BGVAR(u16_TF);
   if (f_tf > 200) f_tf=200;
   else if (f_tf < 0) f_tf=0;

   f_r1_temp = BGVAR(f_R_Poly)[1];
   if (f_tf < 100)
   {
      BGVAR(f_R_Poly)[1] = BGVAR(f_R_Poly)[1] * f_tf/100;
   }
   else if (f_tf>100)
   {
      f_tf_factor = 1+(BGVAR(f_H_Poly)[1]/BGVAR(f_R_Poly)[1]-1)*(f_tf-100)/100;
      BGVAR(f_R_Poly)[1] = BGVAR(f_R_Poly)[1] * f_tf_factor;
   }
   BGVAR(f_R_Poly)[0] = BGVAR(f_R_Poly)[0] + f_r1_temp - BGVAR(f_R_Poly)[1];
}

void VfoDesign(int drive)
{
   float f_temp1,f_temp2;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch(BGVAR(u8_VFO_Mode))
   {
      case 0: // transparent
         BGVAR(f_VFO_Poly)[VFO_POLY_A1] = BGVAR(f_VFO_Poly)[VFO_POLY_A2] = 0;
         BGVAR(f_VFO_Poly)[VFO_POLY_B1] = BGVAR(f_VFO_Poly)[VFO_POLY_B2] = 0;
         BGVAR(f_VFO_Poly)[VFO_POLY_B0] = 1;
         break;

      case 1: // single pole lpf
         BGVAR(f_VFO_Poly)[VFO_POLY_A1] = -exp(-2*3.1416*(float)BGVAR(u16_VFO_Hz1)*0.000125);
         BGVAR(f_VFO_Poly)[VFO_POLY_A2] = 0;

         BGVAR(f_VFO_Poly)[VFO_POLY_B0] = 1 + BGVAR(f_VFO_Poly)[VFO_POLY_A1];
         BGVAR(f_VFO_Poly)[VFO_POLY_B1] = BGVAR(f_VFO_Poly)[VFO_POLY_B2] = 0;
         break;

      case 2: // 2 cascaded lpfs
         f_temp1 = exp(-2*3.1416*(float)BGVAR(u16_VFO_Hz1)*0.000125);
         f_temp2 = exp(-2*3.1416*(float)BGVAR(u16_VFO_Hz2)*0.000125);

         BGVAR(f_VFO_Poly)[VFO_POLY_A1] = -f_temp1 - f_temp2;
         BGVAR(f_VFO_Poly)[VFO_POLY_A2] = f_temp1*f_temp2;

         BGVAR(f_VFO_Poly)[VFO_POLY_B0] = 1 + BGVAR(f_VFO_Poly)[VFO_POLY_A1] + BGVAR(f_VFO_Poly)[VFO_POLY_A2];
         BGVAR(f_VFO_Poly)[VFO_POLY_B1] = BGVAR(f_VFO_Poly)[VFO_POLY_B2] = 0;
         break;

      case 3: // notch
         BGVAR(f_VFO_Poly)[VFO_POLY_B0] = 1/(1+tan(3.1416*(float)BGVAR(u16_VFO_Hz1)*0.000125));//bw
         BGVAR(f_VFO_Poly)[VFO_POLY_B1] = -BGVAR(f_VFO_Poly)[VFO_POLY_B0]*2*cos(2*3.1416*(float)BGVAR(u16_VFO_Hz2)*0.000125);//center
         BGVAR(f_VFO_Poly)[VFO_POLY_B2] = BGVAR(f_VFO_Poly)[VFO_POLY_B0];
         BGVAR(f_VFO_Poly)[VFO_POLY_A1] = BGVAR(f_VFO_Poly)[VFO_POLY_B1];
         BGVAR(f_VFO_Poly)[VFO_POLY_A2] = 2*BGVAR(f_VFO_Poly)[VFO_POLY_B0]-1;
         break;

      case 4: // hpf
         BGVAR(f_VFO_Poly)[VFO_POLY_A1] = -exp(-2*3.1416*(float)BGVAR(u16_VFO_Hz1)*0.000125);
         BGVAR(f_VFO_Poly)[VFO_POLY_A2] = 0;
         BGVAR(f_VFO_Poly)[VFO_POLY_B0] = -BGVAR(f_VFO_Poly)[VFO_POLY_A1];
         BGVAR(f_VFO_Poly)[VFO_POLY_B1] = BGVAR(f_VFO_Poly)[VFO_POLY_A1];
         BGVAR(f_VFO_Poly)[VFO_POLY_B2] = 0;
         break;

      case 5: // bpf
         BGVAR(f_VFO_Poly)[VFO_POLY_B1] = 1/(1+tan(3.1416*(float)BGVAR(u16_VFO_Hz1)*0.000125));//bw
         BGVAR(f_VFO_Poly)[VFO_POLY_B0] = 1-BGVAR(f_VFO_Poly)[VFO_POLY_B1];
         BGVAR(f_VFO_Poly)[VFO_POLY_B2] = -BGVAR(f_VFO_Poly)[VFO_POLY_B0];
         BGVAR(f_VFO_Poly)[VFO_POLY_A2] = 2*BGVAR(f_VFO_Poly)[VFO_POLY_B1]-1;
         BGVAR(f_VFO_Poly)[VFO_POLY_A1] = -BGVAR(f_VFO_Poly)[VFO_POLY_B1]*2*cos(2*3.1416*(float)BGVAR(u16_VFO_Hz2)*0.000125);//center
         BGVAR(f_VFO_Poly)[VFO_POLY_B1] = 0;
         break;

      case 6: // user defined - using VF
         BGVAR(f_VFO_Poly)[VFO_POLY_A1] = (float)BGVAR(s16_Vf)[A1] / (float)(1L << BGVAR(s16_Vf)[A_SHR]);
         BGVAR(f_VFO_Poly)[VFO_POLY_A2] = (float)BGVAR(s16_Vf)[A2] / (float)(1L << BGVAR(s16_Vf)[A_SHR]);
         BGVAR(f_VFO_Poly)[VFO_POLY_B0] = (float)BGVAR(s16_Vf)[B0] / (float)(1L << BGVAR(s16_Vf)[B_SHR]);
         BGVAR(f_VFO_Poly)[VFO_POLY_B1] = (float)BGVAR(s16_Vf)[B1] / (float)(1L << BGVAR(s16_Vf)[B_SHR]);
         BGVAR(f_VFO_Poly)[VFO_POLY_B2] = (float)BGVAR(s16_Vf)[B2] / (float)(1L << BGVAR(s16_Vf)[B_SHR]);
         break;

      case 7: // Anti-Vibe filter
         // coefficients are calculated in LinearPosLoopAntiVib3Design(drive)
         LinearPosLoopAntiVib3Design(drive);
         break;
   }
}

void VfiDesign(int drive)
{
      int s16_max_b,s16_max_a,s16_b_shifts,s16_a_shifts;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
      // push b0,b1,b2 left
      s16_max_b = abs(BGVAR(s16_Vfi)[B0]);
      if (s16_max_b < abs(BGVAR(s16_Vfi)[B1])) s16_max_b = abs(BGVAR(s16_Vfi)[B1]);
      if (s16_max_b < abs(BGVAR(s16_Vfi)[B2])) s16_max_b = abs(BGVAR(s16_Vfi)[B2]);
      FloatToFix16Shift16(&s16_max_b,(unsigned int *)&s16_b_shifts,(float)s16_max_b);

   s16_max_a = abs(BGVAR(s16_Vfi)[A1]);
   if (s16_max_a < abs(BGVAR(s16_Vfi)[A1])) s16_max_a = abs(BGVAR(s16_Vfi)[A2]);
   FloatToFix16Shift16(&s16_max_a,(unsigned int *)&s16_a_shifts,(float)s16_max_a);

   VAR(AX0_s16_Ext_Vel_Coef_Lpf_A2) = -BGVAR(s16_Vfi)[A2] << s16_a_shifts;
   VAR(AX0_s16_Ext_Vel_Coef_Lpf_A1) = -BGVAR(s16_Vfi)[A1] << s16_a_shifts;

   VAR(AX0_s16_Ext_Vel_Coef_Lpf_B2) = BGVAR(s16_Vfi)[B2] << s16_b_shifts;
   VAR(AX0_s16_Ext_Vel_Coef_Lpf_B1) = BGVAR(s16_Vfi)[B1] << s16_b_shifts;
   VAR(AX0_s16_Ext_Vel_Coef_Lpf_B0) = BGVAR(s16_Vfi)[B0] << s16_b_shifts;


   VAR(AX0_s16_Ext_Vel_Coef_Lpf_Mem1_Shr) = VAR(AX0_s16_Ext_Vel_Coef_Lpf_In_Shr) = VAR(AX0_s16_Ext_Vel_Coef_Lpf_Shl) = VAR(AX0_s16_Ext_Vel_Coef_Lpf_Out_Shr) = 0;

   if ((VAR(AX0_s16_Ext_Vel_Coef_Lpf_B2) != 0) ||
        (VAR(AX0_s16_Ext_Vel_Coef_Lpf_B1) != 0) ||
       (VAR(AX0_s16_Ext_Vel_Coef_Lpf_B0) != 0)   )
   {
      VAR(AX0_s16_Ext_Vel_Coef_Lpf_Out_Shr) = -s16_b_shifts - BGVAR(s16_Vfi)[B_SHR];
      if (VAR(AX0_s16_Ext_Vel_Coef_Lpf_Out_Shr)<-14)
         {
         VAR(AX0_s16_Ext_Vel_Coef_Lpf_In_Shr) = VAR(AX0_s16_Ext_Vel_Coef_Lpf_Out_Shr) + 14;
         VAR(AX0_s16_Ext_Vel_Coef_Lpf_Out_Shr) = -14;
         }
   }

   if ((VAR(AX0_s16_Ext_Vel_Coef_Lpf_A2) != 0) ||
        (VAR(AX0_s16_Ext_Vel_Coef_Lpf_A1) != 0)   )
   {
      VAR(AX0_s16_Ext_Vel_Coef_Lpf_Mem1_Shr) =  -s16_a_shifts - BGVAR(s16_Vfi)[A_SHR] - VAR(AX0_s16_Ext_Vel_Coef_Lpf_In_Shr);
      VAR(AX0_s16_Ext_Vel_Coef_Lpf_Shl) = 16 + VAR(AX0_s16_Ext_Vel_Coef_Lpf_Mem1_Shr) + VAR(AX0_s16_Ext_Vel_Coef_Lpf_In_Shr);
      if (VAR(AX0_s16_Ext_Vel_Coef_Lpf_Shl) < 0)
         {
         VAR(AX0_s16_Ext_Vel_Coef_Lpf_Mem1_Shr) -= VAR(AX0_s16_Ext_Vel_Coef_Lpf_Shl);
         VAR(AX0_s16_Ext_Vel_Coef_Lpf_Shl) = -VAR(AX0_s16_Ext_Vel_Coef_Lpf_Shl);
         VAR(AX0_s16_Ext_Vel_Coef_Lpf_A1) >>=VAR(AX0_s16_Ext_Vel_Coef_Lpf_Shl);
         VAR(AX0_s16_Ext_Vel_Coef_Lpf_A2) >>=VAR(AX0_s16_Ext_Vel_Coef_Lpf_Shl);
         VAR(AX0_s16_Ext_Vel_Coef_Lpf_Shl) = 0;
         }
   }

   VAR(AX0_s16_Ext_Vel_Coef_Lpf_Mem1_Shr) = -VAR(AX0_s16_Ext_Vel_Coef_Lpf_Mem1_Shr);
   VAR(AX0_s16_Ext_Vel_Coef_Lpf_In_Shr) = -VAR(AX0_s16_Ext_Vel_Coef_Lpf_In_Shr);
   VAR(AX0_s16_Ext_Vel_Coef_Lpf_Out_Shr) = -VAR(AX0_s16_Ext_Vel_Coef_Lpf_Out_Shr);
}

int VelocityLoopDesign(int drive)
{
   // AXIS_OFF;
   int s16_temp,s16_d_shifts,s16_rh_shifts,s16_out_shr,s16_hr_shr1,s16_d_shl,s16_d_shr,s16_hrl_shr;
   float f_temp,f_max_rh,f_max_d,f_temp2,f_vlim_rpm,f_inertia_factor;
   signed char s8_design_ok = 1;
   int* p_s16_temp_ptr;
   long s32_rh_poly_sum=0L,s32_sum_d=0L;
   unsigned int u16_overflow_cntr1=0, u16_overflow_cntr2=0;

   f_vlim_rpm = (float)BGVAR(s32_V_Lim_Design) * 0.00011176;

   if ( (BGVAR(u8_CompMode) == 5) || (BGVAR(u8_CompMode) == 6) || (BGVAR(u8_CompMode) == 7))// nct vel loop
   {
      VAR(AX0_s16_Ext_Vel_Coef_Lpf_Mem1_Shr) = VAR(AX0_s16_Ext_Vel_Coef_Lpf_In_Shr) = 0;
      VAR(AX0_s16_Ext_Vel_Coef_Lpf_A1) = VAR(AX0_s16_Ext_Vel_Coef_Lpf_A2) = VAR(AX0_s16_Ext_Vel_Coef_Lpf_Shl) = 0;
      VAR(AX0_s16_Ext_Vel_Coef_Lpf_B2)= VAR(AX0_s16_Ext_Vel_Coef_Lpf_B1) = 0;
      VAR(AX0_s16_Ext_Vel_Coef_Lpf_B0) = 0x4000;
      VAR(AX0_s16_Ext_Vel_Coef_Lpf_Out_Shr) = 14;

      return (SAL_SUCCESS);
   }

   VfiDesign(drive);

   //KPAFRC update - support afrc on linear velocity loop opmode 0 / 1
   //internal units = user*8000*vlim/22750/60*2*pi*26214/dipeak
   //               = 965.31821 * user *f_vlim_rpm *  f_inertia_factor/dipeak
   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
      f_inertia_factor = (float)BGVAR(s32_Motor_J)*(1+(float)BGVAR(u32_LMJR)/1000.0)/((float)BGVAR(u32_Motor_Kt)*1000);
   else//Linear
      //factor for s32_Motor_Mass is:  (pitch ^ 2) / (2 * PI)^2 = (pitch ^ 2) / 39.4786
      //factor for u32_Motor_Kf   is:  pitch / (2 * PI) = pitch / 6.2832
      //sum of factor: pitch * 0.15915
      f_inertia_factor = (((float)BGVAR(s32_Motor_Mass) * 1000.0 * (1+(float)BGVAR(u32_LMJR)/1000.0)/((float)BGVAR(u32_Motor_Kf)*1000.0)) * (float)(((float)BGVAR(u32_Mpitch) / 1000000.0) * 0.15915));//mpitch changed to decimal

   FloatToFix16Shift16(&VAR(AX0_s16_KPAFRC_Linear_Vel_Loop_Fix_Design),
                       (unsigned int *)&VAR(AX0_s16_KPAFRC_Linear_Vel_Loop_Shr_Design),
                        (float)((float)BGVAR(s32_Kpafrcurrent) * f_vlim_rpm * f_inertia_factor *965.31821 / (float)BGVAR(s32_Drive_I_Peak)));

   if (BGVAR(u8_CompMode) == 3) // advanced pole placement
   {
      // s16_Vh - h0 h0_shr h1 h1_shr h2 h2_shr h3 h3_shr h4 h4_shr h5 h5_shr
      BGVAR(f_H_Design)[0] = (float)BGVAR(s16_Vh)[0] / (float)(1L << BGVAR(s16_Vh)[1]);
      BGVAR(f_H_Design)[1] = (float)BGVAR(s16_Vh)[2] / (float)(1L << BGVAR(s16_Vh)[3]);
      BGVAR(f_H_Design)[2] = (float)BGVAR(s16_Vh)[4] / (float)(1L << BGVAR(s16_Vh)[5]);
      BGVAR(f_H_Design)[3] = (float)BGVAR(s16_Vh)[6] / (float)(1L << BGVAR(s16_Vh)[7]);
      BGVAR(f_H_Design)[4] = (float)BGVAR(s16_Vh)[8] / (float)(1L << BGVAR(s16_Vh)[9]);
      BGVAR(f_H_Design)[5] = (float)BGVAR(s16_Vh)[10] / (float)(1L << BGVAR(s16_Vh)[11]);

      // s16_Vr - r0 r0_shr r1 r1_shr r2 r2_shr r3 r3_shr r4 r4_shr
      BGVAR(f_R_Design)[0] = (float)BGVAR(s16_Vr)[0] / (float)(1L << BGVAR(s16_Vr)[1]);
      BGVAR(f_R_Design)[1] = (float)BGVAR(s16_Vr)[2] / (float)(1L << BGVAR(s16_Vr)[3]);
      BGVAR(f_R_Design)[2] = (float)BGVAR(s16_Vr)[4] / (float)(1L << BGVAR(s16_Vr)[5]);
      BGVAR(f_R_Design)[3] = (float)BGVAR(s16_Vr)[6] / (float)(1L << BGVAR(s16_Vr)[7]);
      BGVAR(f_R_Design)[4] = (float)BGVAR(s16_Vr)[8] / (float)(1L << BGVAR(s16_Vr)[9]);

      // s16_Vd - d1 d2 d3 d4 d5 d6 d7 d_shr
      BGVAR(f_D_Design)[1] = (float)BGVAR(s16_Vd)[0] / (float)(1L << BGVAR(s16_Vd)[7]);
      BGVAR(f_D_Design)[2] = (float)BGVAR(s16_Vd)[1] / (float)(1L << BGVAR(s16_Vd)[7]);
      BGVAR(f_D_Design)[3] = (float)BGVAR(s16_Vd)[2] / (float)(1L << BGVAR(s16_Vd)[7]);
      BGVAR(f_D_Design)[4] = (float)BGVAR(s16_Vd)[3] / (float)(1L << BGVAR(s16_Vd)[7]);
      BGVAR(f_D_Design)[5] = (float)BGVAR(s16_Vd)[4] / (float)(1L << BGVAR(s16_Vd)[7]);
      BGVAR(f_D_Design)[6] = (float)BGVAR(s16_Vd)[5] / (float)(1L << BGVAR(s16_Vd)[7]);
      BGVAR(f_D_Design)[7] = (float)BGVAR(s16_Vd)[6] / (float)(1L << BGVAR(s16_Vd)[7]);
   }
   else
   {
      if (BGVAR(u8_CompMode) <=1) PIPDFFDesign(drive);
      else if ((BGVAR(u8_CompMode) == 2) || (BGVAR(u8_CompMode) == 4)) PolePlacmentDesign(drive);
      VfoDesign(drive);

      BGVAR(f_H_Design)[0] = BGVAR(f_H_Poly)[0]*BGVAR(f_VFO_Poly)[VFO_POLY_B0];
      BGVAR(f_H_Design)[1] = BGVAR(f_H_Poly)[0]*BGVAR(f_VFO_Poly)[VFO_POLY_B1] + BGVAR(f_H_Poly)[1]*BGVAR(f_VFO_Poly)[VFO_POLY_B0];
      BGVAR(f_H_Design)[2] = BGVAR(f_H_Poly)[0]*BGVAR(f_VFO_Poly)[VFO_POLY_B2] + BGVAR(f_H_Poly)[1]*BGVAR(f_VFO_Poly)[VFO_POLY_B1] + BGVAR(f_H_Poly)[2]*BGVAR(f_VFO_Poly)[VFO_POLY_B0];
      BGVAR(f_H_Design)[3] = BGVAR(f_H_Poly)[1]*BGVAR(f_VFO_Poly)[VFO_POLY_B2] + BGVAR(f_H_Poly)[2]*BGVAR(f_VFO_Poly)[VFO_POLY_B1] + BGVAR(f_H_Poly)[3]*BGVAR(f_VFO_Poly)[VFO_POLY_B0];
      BGVAR(f_H_Design)[4] = BGVAR(f_H_Poly)[2]*BGVAR(f_VFO_Poly)[VFO_POLY_B2] + BGVAR(f_H_Poly)[3]*BGVAR(f_VFO_Poly)[VFO_POLY_B1];
      BGVAR(f_H_Design)[5] = BGVAR(f_H_Poly)[3]*BGVAR(f_VFO_Poly)[VFO_POLY_B2];

      BGVAR(f_R_Design)[0] = BGVAR(f_R_Poly)[0]*BGVAR(f_VFO_Poly)[VFO_POLY_B0];
      BGVAR(f_R_Design)[1] = BGVAR(f_R_Poly)[0]*BGVAR(f_VFO_Poly)[VFO_POLY_B1] + BGVAR(f_R_Poly)[1]*BGVAR(f_VFO_Poly)[VFO_POLY_B0];
      BGVAR(f_R_Design)[2] = BGVAR(f_R_Poly)[0]*BGVAR(f_VFO_Poly)[VFO_POLY_B2] + BGVAR(f_R_Poly)[1]*BGVAR(f_VFO_Poly)[VFO_POLY_B1] + BGVAR(f_R_Poly)[2]*BGVAR(f_VFO_Poly)[VFO_POLY_B0];
      BGVAR(f_R_Design)[3] = BGVAR(f_R_Poly)[1]*BGVAR(f_VFO_Poly)[VFO_POLY_B2] + BGVAR(f_R_Poly)[2]*BGVAR(f_VFO_Poly)[VFO_POLY_B1];
      BGVAR(f_R_Design)[4] = BGVAR(f_R_Poly)[2]*BGVAR(f_VFO_Poly)[VFO_POLY_B2];

      BGVAR(f_D_Design)[1] = BGVAR(f_VFO_Poly)[VFO_POLY_A1] + BGVAR(f_D_Poly)[1];
      BGVAR(f_D_Design)[2] = BGVAR(f_D_Poly)[2] + BGVAR(f_VFO_Poly)[VFO_POLY_A1]*BGVAR(f_D_Poly)[1] + BGVAR(f_VFO_Poly)[VFO_POLY_A2];
      BGVAR(f_D_Design)[3] = BGVAR(f_D_Poly)[3] + BGVAR(f_VFO_Poly)[VFO_POLY_A1]*BGVAR(f_D_Poly)[2] + BGVAR(f_VFO_Poly)[VFO_POLY_A2]*BGVAR(f_D_Poly)[1];
      BGVAR(f_D_Design)[4] = BGVAR(f_D_Poly)[4] + BGVAR(f_VFO_Poly)[VFO_POLY_A1]*BGVAR(f_D_Poly)[3] + BGVAR(f_VFO_Poly)[VFO_POLY_A2]*BGVAR(f_D_Poly)[2];
      BGVAR(f_D_Design)[5] = BGVAR(f_D_Poly)[5] + BGVAR(f_VFO_Poly)[VFO_POLY_A1]*BGVAR(f_D_Poly)[4] + BGVAR(f_VFO_Poly)[VFO_POLY_A2]*BGVAR(f_D_Poly)[3];
      BGVAR(f_D_Design)[6] = BGVAR(f_VFO_Poly)[VFO_POLY_A1]*BGVAR(f_D_Poly)[5] + BGVAR(f_VFO_Poly)[VFO_POLY_A2]*BGVAR(f_D_Poly)[4];
      BGVAR(f_D_Design)[7] = BGVAR(f_VFO_Poly)[VFO_POLY_A2]*BGVAR(f_D_Poly)[5];
   }
   f_max_rh = BGVAR(f_H_Design)[0];
   if (f_max_rh < 0) f_max_rh =-f_max_rh;
   for (s16_temp=1;s16_temp<=5;++s16_temp)
   {
      f_temp = BGVAR(f_H_Design)[s16_temp];
      if (f_temp < 0) f_temp=-f_temp;
      if (f_temp > f_max_rh) f_max_rh = f_temp;
   }

   for (s16_temp=0;s16_temp<=4;++s16_temp)
   {
      f_temp = BGVAR(f_R_Design)[s16_temp];
      if (f_temp < 0) f_temp=-f_temp;
      if (f_temp > f_max_rh) f_max_rh = f_temp;
   }

   s16_rh_shifts = -1;
   while ((f_max_rh < 2147483647.0) && (f_max_rh != 0.0) && (u16_overflow_cntr1<=99))
   {
      u16_overflow_cntr1++;
      f_max_rh = f_max_rh * 2.0;
      s16_rh_shifts++;
   }
   if (s16_rh_shifts < 0) s16_rh_shifts = 0;
   if (u16_overflow_cntr1>99) u16_Internal_OverFlow |= 0x04; // Signal internal error

   f_max_d = BGVAR(f_D_Design)[1];
   if (f_max_d < 0.0) f_max_d=-f_max_d;
   for (s16_temp=2;s16_temp<=7;++s16_temp)
   {
      f_temp = BGVAR(f_D_Design)[s16_temp];
      if (f_temp < 0) f_temp=-f_temp;
      if (f_temp > f_max_d) f_max_d = f_temp;
   }

   s16_d_shifts = -1;
   while ((f_max_d < 32767.0) && (f_max_d != 0) && (u16_overflow_cntr2<=99))
   {
      u16_overflow_cntr2++;
      f_max_d = f_max_d * 2.0;
      s16_d_shifts++;
   }
   if (s16_d_shifts < 0) s16_d_shifts = 0;
   if (u16_overflow_cntr2>99) u16_Internal_OverFlow |= 0x08; // Signal internal error

   if (s16_rh_shifts > 30)
   {
      f_temp = (float)((long long)(1LL << (long long)(s16_rh_shifts-30)));
      f_temp2 = 1073741824;
   }
   else
   {
      f_temp = (float)((long long)(1LL << (long long)s16_rh_shifts));
      f_temp2 = 1;
   }

   for(s16_temp=0;s16_temp<=5;s16_temp++) BGVAR(s32_H_Poly)[s16_temp]   = -(long)(BGVAR(f_H_Design)[s16_temp] * f_temp * f_temp2);
   for(s16_temp=0;s16_temp<=4;s16_temp++) BGVAR(s32_R_Poly)[s16_temp]   = (long)(BGVAR(f_R_Design)[s16_temp] * f_temp * f_temp2);

   f_temp = (float)((long)(1L << (long)s16_d_shifts));
   for(s16_temp=1;s16_temp<=7;s16_temp++) BGVAR(s16_D_Poly)[s16_temp]   = -(int)(BGVAR(f_D_Design)[s16_temp] * f_temp);

   if (s16_d_shifts == 0) BGVAR(s16_D_Poly)[1] = 0;

   s16_rh_shifts = s16_rh_shifts - 16;
   s16_out_shr = s16_rh_shifts;
   if (s16_out_shr > 15) s16_out_shr = 15;

   s16_hr_shr1 = s16_rh_shifts - s16_out_shr;
   s16_d_shl = 16 - s16_d_shifts;
   s16_d_shr = s16_d_shifts - s16_hr_shr1;

   if ( (BGVAR(s16_D_Poly)[1] == 0) &&
         (BGVAR(s16_D_Poly)[2] == 0) &&
         (BGVAR(s16_D_Poly)[3] == 0) &&
         (BGVAR(s16_D_Poly)[4] == 0) &&
         (BGVAR(s16_D_Poly)[5] == 0) &&
         (BGVAR(s16_D_Poly)[6] == 0) &&
         (BGVAR(s16_D_Poly)[7] == 0)   ) s16_d_shr = 0;

   s16_hrl_shr = 13;// - s16_d_shr; // 16 because it is low part of RH, -3 because HRl coefs shifted 3 shr to avoid numerical overflow when the RT products are summed

   if (s16_hr_shr1 < 0) s8_design_ok = -1;
   else if (s16_d_shl < 0) s8_design_ok = -2;
   else if (s16_d_shr < 0) s8_design_ok = -3;
   else if (s16_hrl_shr < 0) s8_design_ok = -4;
   else if (BGVAR(s32_R_Poly)[0] < 0) s8_design_ok = -5;
   else if (s16_out_shr < 0) s8_design_ok = -6;
   if (s8_design_ok != 1) return (s8_design_ok);

   f_temp2 = (float)((long)(1L << (long)s16_out_shr))*BGVAR(f_R_Design)[0];
   f_temp = 1/f_temp2;
   f_temp2 = f_temp;
   if (f_temp < 0) f_temp=-f_temp;

   VAR(AX0_s16_Vel_Var_InvR0_Shr_Design) = -1;
   while ((f_temp < 32767.0L) && (VAR(AX0_s16_Vel_Var_InvR0_Shr_Design) < 31))
   {
      f_temp = f_temp * 2;
      VAR(AX0_s16_Vel_Var_InvR0_Shr_Design) = VAR(AX0_s16_Vel_Var_InvR0_Shr_Design)+1;
   }
   if (VAR(AX0_s16_Vel_Var_InvR0_Shr_Design) < 0) VAR(AX0_s16_Vel_Var_InvR0_Shr_Design) = 0;
   VAR(AX0_s16_Vel_Var_InvR0_Fix_Design) = (int)(f_temp2 * (float)((long)(1L << (long)VAR(AX0_s16_Vel_Var_InvR0_Shr_Design))));

   VAR(AX0_s16_Vel_Var_Out_Shr_Design) = s16_out_shr;
   VAR(AX0_s16_Vel_Var_Hrl_Shr_Design) = s16_hrl_shr;
   VAR(AX0_s16_Vel_Var_D_Shr_Design) = s16_d_shr;
   VAR(AX0_s16_Vel_Var_Hr_Shr1_Design) = s16_hr_shr1;
   VAR(AX0_s16_Vel_Var_D_Shl_Design) = s16_d_shl;

   VAR(AX0_u16_Hrl_Residue_Mask_Design) = ((unsigned int)1 << s16_hrl_shr) - 1;
   VAR(AX0_u16_D_Residue_Mask_Design) = ((unsigned int)1 << s16_d_shr) - 1;
   VAR(AX0_u16_Hr_Residue_Mask_Design) = ((unsigned int)1 << s16_hr_shr1) - 1;

   // since the low part is shr by 3 - make sure DC gain is not affected
   // do not fix dc gain when pdff and kvi = 0
   if ((BGVAR(u8_CompMode) != 1) || (BGVAR(u32_Kvi) != 0))
   {
      for (s16_temp=0;s16_temp<=4;s16_temp++) s32_rh_poly_sum += (BGVAR(s32_R_Poly)[s16_temp] & 0xFFFFFFF8);
      for (s16_temp=0;s16_temp<=5;s16_temp++) s32_rh_poly_sum += (BGVAR(s32_H_Poly)[s16_temp] & 0xFFFFFFF8);
      BGVAR(s32_H_Poly)[5] -= s32_rh_poly_sum;
   }

    // make sure sum of D is -1 incase there is intergrator in the design
   if ((BGVAR(u8_CompMode) == 2)                                ||
       (BGVAR(u8_CompMode) == 4)                                ||
       ( (BGVAR(u8_CompMode) == 0) && (BGVAR(u32_Kvi) != 0)  )  ||
      ( (BGVAR(u8_CompMode) == 1) && (BGVAR(u32_Kvi) != 0)  )    )
   {
      for (s16_temp=1;s16_temp<=7;s16_temp++) s32_sum_d += (long)BGVAR(s16_D_Poly)[s16_temp];
      s32_sum_d -= (long)(1L << (long)s16_d_shifts);
      BGVAR(s16_D_Poly)[7] -=s32_sum_d;
   }


   p_s16_temp_ptr = &VAR(AX0_s16_Vel_Coef_D1_Design);
   for (s16_temp=1;s16_temp<=7;s16_temp++) *p_s16_temp_ptr++ = BGVAR(s16_D_Poly)[s16_temp];

   p_s16_temp_ptr = &VAR(AX0_s16_Vel_Coef_R0h_Design);
   for (s16_temp=0;s16_temp<=4;s16_temp++)
   {
      *p_s16_temp_ptr = (int)(BGVAR(s32_R_Poly)[s16_temp]>>16);
      *(p_s16_temp_ptr-18) = (int)((BGVAR(s32_R_Poly)[s16_temp] >> 3) & 0x1fff);    // shift the low part (unsigned) 3 shr to avoid numerical
                                                                               // overflow when the RT products are summed
      p_s16_temp_ptr++;
   }

   p_s16_temp_ptr = &VAR(AX0_s16_Vel_Coef_H0h_Design);
   for (s16_temp=0;s16_temp<=5;s16_temp++)
   {
      *p_s16_temp_ptr = (int)(BGVAR(s32_H_Poly)[s16_temp]>>16);
      *(p_s16_temp_ptr-18) = (int)((BGVAR(s32_H_Poly)[s16_temp] >> 3) & 0x1fff);    // shift the low part (unsigned) 3 shr to avoid numerical
                                                                               // overflow when the RT products are summed
      p_s16_temp_ptr++;
   }

   if (s8_design_ok == 1) VAR(AX0_s16_Crrnt_Run_Code) |= COPY_VEL_COEF_MASK;
   else s8_design_ok = CONFIG_VELOCITY_FAILED;

   return (s8_design_ok);
}

void LinearPosLoopAntiVib3Design(drive)
{
   //// AXIS_OFF;

   float f_LR, f_RC,f_d,f_sharp,f_freq,f_r_out,f_c_out,f_LRdt,f_RCdt;
   REFERENCE_TO_DRIVE;
   // PCMD RLc3 (notch) filter design

   //f_freq*1000 = ANTIVIBHZ3*sqrt(1+ANTIVIBGAIN3)
   //KPANTIVIBHZ3 and KPANTIVIBGAIN3 already multiplyed by 1000, therefore result is f_freq*1000
   f_freq  = (float)BGVAR(u32_PL_Antivib3_Fcenter) * sqrt(1.0 + (float)BGVAR(s32_PL_Antivib3_Gain)*0.001);

   //f_sharp*1000 = ANTIVIBSHARP3*sqrt(1+ANTIVIBGAIN3)
   //ANTIVIBHZ3 and ANTIVIBGAIN3 already multiplyed by 1000, therefore result is f_sharp*1000
   f_sharp = (float)BGVAR(u16_PL_Antivib3_Sharpness);

   //f_sharp and f_freq both multiplyed by 1000, therefore no need scaling
   f_LR = f_sharp / f_freq;

   // RC = 1/(4*pi*pi*F_filter*F_filter*LR) = 0.02533029 / (F_filter*F_filter*LR)
   // 25330.29 =  0.02533029 * 1000000 (from f_freq*1000*f_freq*1000)
   f_RC = 25330.29 / (f_freq * f_freq * f_LR);


   // LRdt = LR/dt // dt = 125uS
   f_LRdt = f_LR * 8000.0;

   // RCdt = (2^14)*dt/RC (multiplyed by 2^14)
   f_RCdt = 1.0 / (f_RC * 8000.0);

   // D = 1+LRdt
   f_d = 1.0 + f_LRdt;
   f_d = 1.0 / f_d;


   //R_Out = ANTIVIBQ3/1000
   f_r_out = 0.001*(float)BGVAR(s32_PL_Antivib3_Q_Gain);

   //C_out = 1 / (1+ANTIVIBGAIN3)
   if (BGVAR(s32_PL_Antivib3_Gain) == 0)
      f_c_out = 0; // in this case r.t overrides the filter
   else
      f_c_out = 1.0 / (1.0 + (float)BGVAR(s32_PL_Antivib3_Gain)/1000.0);

   BGVAR(f_VFO_Poly)[VFO_POLY_A1] = - (1.0 - f_RCdt * f_d + f_LRdt * f_d);
   BGVAR(f_VFO_Poly)[VFO_POLY_A2] = f_LRdt * f_d;
   BGVAR(f_VFO_Poly)[VFO_POLY_B0] = f_d * (f_RCdt * f_c_out + f_r_out + f_LRdt);
   BGVAR(f_VFO_Poly)[VFO_POLY_B1] = f_d * (- f_r_out - 2.0 * f_LRdt);
   BGVAR(f_VFO_Poly)[VFO_POLY_B2] = f_d * f_LRdt;

}



int VelocityConfig(int drive)
{
   // AXIS_OFF;

   int s16_result;

   if ((BGVAR(s64_SysNotOk) & NO_COMP_FLT_MASK) != 0) return(SAL_SUCCESS);

   /* Wait till last design is copied so global design vars will not be overrun */
   if (u16_background_ran) while ((VAR(AX0_s16_Crrnt_Run_Code) & COPY_VEL_COEF_MASK) != 0);

   if (BGVAR(s32_V_Lim_Design) < 89478L)
   {
      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
      return CONFIG_FAIL_VLIM;
   }
   s16_result = VelocityLoopDesign(drive);

   AnalogVelocityConfig(drive);
   PositionConfig(DRIVE_PARAM,0); // This needs to be called as VELCONTROLMODE=5/6/7 uses position
   return (s16_result);
}


void UpdateFieldbusVelocityLimit(int drive)
{
   // AXIS_OFF;
   long long     s64_fix = 1LL;
   long long     s64_tmp = 1LL;
   long          s32_vlim = 0L;
   unsigned int  u16_shift = 0, u16_temp_time;
   REFERENCE_TO_DRIVE;
   
   if (IS_DUAL_LOOP_ACTIVE)
   {
      s32_vlim = BGVAR(s32_Sfb_V_Lim);
   }
   else
   {
      s32_vlim = BGVAR(s32_V_Lim_Design);
   }
   

   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, (long long)BGVAR(fbus_cycle_time_in_secs).s32_fbus_cycle_time_fix,BGVAR(fbus_cycle_time_in_secs).u16_fbus_cycle_time_shr, s32_vlim, (unsigned int)0);

   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix,u16_shift,(long)8000, 0);

   //mult by 1.2
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix,u16_shift,(long)161061273, 27);

   s64_tmp = MultS64ByFixS64ToS64(1LL,s64_fix,u16_shift);

   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Fieldbus_Vlim_Design_Lo) = s64_tmp;
   } while (u16_temp_time != Cntr_3125);
}

void UpdateAccDec(int drive, char mode)
{
    // AXIS_OFF;

    /*
        We need to move from internal units of acc/dec used in position loop to
        internal units of velocity used in the loop devided to Tsv

        internal units of velocity used in the loop / Tsv
        = u64_AccRate * 4000^2 * 60 / 2^64 / 8000 * 22750 / (vlim_internal_out_loop * 8000 / 2^32 * 60) * 2^32
        = u64_AccRate / vlim_internal_out_loop * 5687.5

        (same calculation for u64_DecRate)

        // for Tsp 125 uSec
        Multiply above by 4 : 5687.5 > 22750
    */

    long long s64_tmp = 0LL,  s64_dec_tmp = 0LL;
   
    REFERENCE_TO_DRIVE;
    /**********DualLoop ACC/DEC values adjust****************/
    if (ACC_UPDATE == mode)
    {
        if (IS_DUAL_LOOP_ACTIVE)
        {
            BGVAR(u64_AccRate) = BGVAR(u64_Sfb_AccRate);
        }
        else
        {
            BGVAR(u64_AccRate) = BGVAR(u64_Mfb_AccRate);
        }
    }
    else if (DEC_UPDATE == mode)
    {
        if (IS_DUAL_LOOP_ACTIVE)
        {
            BGVAR(u64_DecRate) = BGVAR(u64_Sfb_DecRate);
        }
        else
        {
            BGVAR(u64_DecRate) = BGVAR(u64_Mfb_DecRate);
        }
    }
    /*******************************************************/
   
   
    s64_dec_tmp = BGVAR(u64_DecRate);
   
    switch (mode)
    {
        case ACC_UPDATE:
        {
            s64_tmp = BGVAR(u64_AccRate);
            break;
        }
        case DEC_UPDATE:
        {
            s64_tmp =  BGVAR(u64_DecRate);
            break;
        }
        case HOLD_UPDATE:
        {
            s64_tmp = BGVAR(u64_Event_Dec_Val);
            s64_dec_tmp = s64_tmp;
            break;
        }
    }

    s64_tmp = s64_tmp / BGVAR(s32_V_Lim_Design) * 22750;

    if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC)
    {
        s64_tmp = s64_tmp / 4;
    }


    if (s64_tmp > 195421011968000LL)
    {
        s64_tmp = 195421011968000LL;//22750*2*2^32
    }

    if (mode == ACC_UPDATE)
    {
        LVAR(AX0_s32_Vel_Acc_Delta_Sat_Hi) = (long)(s64_tmp>>32);
        LVAR(AX0_u32_Vel_Acc_Delta_Sat_Lo) = (long)(s64_tmp);
    }
    else
    {
        LVAR(AX0_s32_Vel_Dec_Delta_Sat_Hi) = (long)(s64_tmp>>32);
        LVAR(AX0_u32_Vel_Dec_Delta_Sat_Lo) = (long)(s64_tmp);
    }

    /*
        Accel/Decel Limits for Gearing:
        The Gearing Process is performed at the Sampling Rate of the Position Processes,
        4,000 times per second.  Thus the Gearing Acceleration & Dceleration Limits will be
        the Background Variables (s64_AccRate and s64_DecRate) divided by 2^32.
    */

    //   if (s64_tmp > 195421011968000LL) s64_tmp = 195421011968000LL; // 22750*2*2^32

    // Multiply ACC and DEC by 16 since the real-time takes delta over 16 samples (sort of averaging)

    BGVAR(s32_Ptp_Acc_Rounded) = (long)((BGVAR(u64_AccRate) + (unsigned long long)0x080000000) >> 32);
    BGVAR(s32_Ptp_Dec_Rounded) = (long)((s64_dec_tmp + (unsigned long long)0x080000000) >> 32);

    // PTP generator variables. This instruction is done in order to allow an
    // immediate update of the ACC/DEC/VLIM PTP-hunter variables in OPMODE 8 CSP mode.
    if ((VAR(AX0_s16_Opmode) == 4) ||
        (((p402_modes_of_operation == CYCLIC_SYNCHRONOUS_POSITION_MODE) ||
          (p402_modes_of_operation == INTRPOLATED_POSITION_MODE)) &&
         (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)/* Bug 4861 */))
    {
        // This will allow to run the PTP hunting during Gearing with the ACC/DEC/VLIM settings
        LVAR(AX0_s32_Ptp_Acc_Rounded_Bg_Shadow) = LVAR(AX0_s32_Ptp_Acc_Rounded) = BGVAR(s32_Ptp_Acc_Rounded);
        LVAR(AX0_s32_Ptp_Dec_Rounded_Bg_Shadow) = LVAR(AX0_s32_Ptp_Dec_Rounded) = BGVAR(s32_Ptp_Dec_Rounded);
        LVAR(AX0_s32_Vlimit_Bg_Shadow) = LVAR(AX0_s32_Vlimit) = BGVAR(s32_V_Lim_Design);

        if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC)
        {
            // <<1 due to conversion from [Counts32/125us] into [Counts32/250us]
            LVAR(AX0_s32_Vlimit_Bg_Shadow) = LVAR(AX0_s32_Vlimit_Bg_Shadow) << 1;
            LVAR(AX0_s32_Vlimit) = LVAR(AX0_s32_Vlimit) << 1;
        }

        // At gear mode,if the gearlimitsmode is 0,the internal deceleration will be 0x7fffffff,
        // otherwise ,the chosen deceleration till this part will be in use.
        if ((VAR(AX0_s16_Opmode) == 4) && ((VAR(AX0_u8_Gear_Limits_Mode) & 0x0004) == 0x0000))
        {
            // Code has been added by Gil as a bug-fix for BZ 3970
            LVAR(AX0_s32_Ptp_Dec_Rounded) = 0x7fffffff;
            LVAR(AX0_s32_Ptp_Dec_Rounded_Bg_Shadow)=0x7fffffff;
        }
    }
}


//**********************************************************
// Function Name: KvCommand
// Description:
//          This function is called in response to the KV command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalKvCommand(long long lparam,int drive)
{
   unsigned long u32_temp = BGVAR(u32_Kv);

   REFERENCE_TO_DRIVE;     // Defeat compilation error


   BGVAR(u32_Kv) = (long)lparam;
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      BGVAR(u32_Kv) = u32_temp;
      VelocityConfig(DRIVE_PARAM);
      return (VELOCITY_CONFIG_FAIL);
   }

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalWriteVelMovingAverageFilterTime
// Description: Moving average filter time constant
//          This function is called in response to the P-Parameter P8-23 command.
//          And sets two real-time variables, which are needed for a moving average
//          filter.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteVelMovingAverageFilterTime(long long param,int drive)
{
   // AXIS_OFF;

   unsigned long u32_temp_value;
   REFERENCE_TO_DRIVE;

   u32_temp_value = (unsigned long)param;

   // If the user entry is not a multiple of 125[us]
   if ((u32_temp_value % 125) != 0)
   {
      return (VALUE_OUT_OF_RANGE);
   }

   // Apply the value if valid
   BGVAR(u32_Vel_Moving_Average_Filter_Time) = u32_temp_value;

   // Calculate the buffer depth to be used. Buffer depth always + 1 since for a filter-time of 125[us] delay, you need two buffer entries and so on.
   VAR(AX0_u16_Vel_Moving_Average_Buffer_Depth) = (u32_temp_value / 125) + 1;

   // Calculate the Fix-Shift values out of 1/buffer_depth
   FloatToFixS32Shift16(&LVAR(AX0_s32_Vel_Moving_Average_Fix), &VAR(AX0_u16_Vel_Moving_Average_Shift_Right), 1/(float)VAR(AX0_u16_Vel_Moving_Average_Buffer_Depth));

   // Load the init counter with the buffer-depth, which starts the initialization process
   VAR(AX0_s16_Vel_Moving_Average_Init_Cntr) = VAR(AX0_u16_Vel_Moving_Average_Buffer_Depth);

   return (SAL_SUCCESS);
}


int SalExtAdditiveVcmdCommand(long long lparam,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (lparam > 22750) lparam = 22750;
   if (lparam < -22750) lparam = -22750;
   VAR(AX0_s16_Extrn_Vcmd_FF) = (int)lparam;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: KvfrCommand
// Description:
//          This function is called in response to the KVFR command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************

// s16_Kvfr command
int SalKvfrCommand(long long lparam,int drive)
{
   int s16_temp = BGVAR(s16_Kvfr);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s16_Kvfr) = (int)lparam;

   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      BGVAR(s16_Kvfr) = s16_temp;
      VelocityConfig(DRIVE_PARAM);
      return (VELOCITY_CONFIG_FAIL);
   }

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: KviCommand
// Description:
//          This function is called in response to the KVI command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalKviCommand(long long lparam,int drive)
{
   unsigned long u32_temp = BGVAR(u32_Kvi);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u32_Kvi) = (unsigned long)lparam;
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
       BGVAR(u32_Kvi) = u32_temp;
      VelocityConfig(DRIVE_PARAM);
       return (VELOCITY_CONFIG_FAIL);
   }

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: VfiCommand
// Description:
//          This function is called in response to the VFI command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalReadVfiCommand(int drive)
{
   int s16_i;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for(s16_i=B0;s16_i<A_SHR;++s16_i)
   {
      PrintSignedInteger(BGVAR(s16_Vfi)[s16_i]);
      PrintChar(SPACE);
   }
   PrintSignedInteger(BGVAR(s16_Vfi)[A_SHR]);
   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalVfiCommand(int drive)
{
   int s16_i, s16_l_limit;
   int u16_temp[7];

   for (s16_i=B0; s16_i<=A_SHR; ++s16_i)
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if (s64_Execution_Parameter[s16_i] < (long long)s16_l_limit) return (VALUE_TOO_LOW);
      if (s64_Execution_Parameter[s16_i] > 32767LL) return (VALUE_TOO_HIGH);
   }

   /* Accept new params */
   for (s16_i=B0; s16_i<=A_SHR; ++s16_i)
   {
      u16_temp[s16_i] = BGVAR(s16_Vfi)[s16_i];
      BGVAR(s16_Vfi)[s16_i] = (int)s64_Execution_Parameter[s16_i];
   }

   // Configure the velocity loop - and if failed, restore previous parameters
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      for (s16_i=B0; s16_i<=A_SHR; ++s16_i)
         BGVAR(s16_Vfi)[s16_i] = u16_temp[s16_i];

      VelocityConfig(DRIVE_PARAM);
      return (VELOCITY_CONFIG_FAIL);
   }

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: VfCommand
// Description:
//          This function is called in response to the VF command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalReadVfCommand(int drive)
{
   int s16_i;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for(s16_i=B0;s16_i<A_SHR;++s16_i)
   {
      PrintSignedInteger(BGVAR(s16_Vf)[s16_i]);
      PrintChar(SPACE);
   }
   PrintSignedInteger(BGVAR(s16_Vf)[A_SHR]);
   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalVfCommand(int drive)
{
   int s16_i, s16_l_limit;
   int u16_temp[7];

   for (s16_i=B0; s16_i<=A_SHR; ++s16_i)
   {
      if ((s16_i == B_SHR) || (s16_i == A_SHR)) s16_l_limit = 0;
      else s16_l_limit = -32768;

      if (s64_Execution_Parameter[s16_i] < (long long)s16_l_limit) return (VALUE_TOO_LOW);
      if (s64_Execution_Parameter[s16_i] > 32767LL) return (VALUE_TOO_HIGH);
   }

   /* Accept new params */
   for (s16_i=B0; s16_i<=A_SHR; ++s16_i)
   {
      u16_temp[s16_i] = BGVAR(s16_Vf)[s16_i];
      BGVAR(s16_Vf)[s16_i] = (int)s64_Execution_Parameter[s16_i];
   }

   // Configure the velocity loop - and if failed, restore previous parameters
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      for (s16_i=B0; s16_i<=A_SHR; ++s16_i)
         BGVAR(s16_Vf)[s16_i] = u16_temp[s16_i];

      VelocityConfig(DRIVE_PARAM);
      return (VELOCITY_CONFIG_FAIL);
   }

   return (SAL_SUCCESS);
}


int SalReadVdCommand(int drive)
{
   int s16_i;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for (s16_i=0;s16_i<=6;++s16_i)
   {
      PrintSignedInteger(BGVAR(s16_Vd)[s16_i]);
      PrintChar(SPACE);
   }
   PrintSignedInteger(BGVAR(s16_Vd)[7]);
   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalVdCommand(int drive)
{
   int s16_i, s16_l_limit;
   int s16_temp[8];

   for (s16_i=0; s16_i<=7; ++s16_i)
   {
      s16_l_limit = -32768;
      if (s16_i == 7) s16_l_limit = 0;

      if (s64_Execution_Parameter[s16_i] < (long long)s16_l_limit) return (VALUE_TOO_LOW);
      if (s64_Execution_Parameter[s16_i] > 32767LL) return (VALUE_TOO_HIGH);
   }

   /* Accept new params */
   for (s16_i=0; s16_i<=7; ++s16_i)
   {
      s16_temp[s16_i] = BGVAR(s16_Vd)[s16_i];
      BGVAR(s16_Vd)[s16_i] = (int)s64_Execution_Parameter[s16_i];
   }

   // Configure the velocity loop - and if failed, restore previous parameters
      if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
    {
      // Restore previous VD values
      for (s16_i=0; s16_i<=8; ++s16_i)
         BGVAR(s16_Vd)[s16_i] = s16_temp[s16_i];

      VelocityConfig(DRIVE_PARAM);
      return (VELOCITY_CONFIG_FAIL);
    }

   return (SAL_SUCCESS);
}

int SalReadVhCommand(int drive)
{
   int s16_i;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for (s16_i=0;s16_i<=10;++s16_i)
   {
      PrintSignedInteger(BGVAR(s16_Vh)[s16_i]);
      PrintChar(SPACE);
   }
   PrintSignedInteger(BGVAR(s16_Vh)[11]);
   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalVhCommand(int drive)
{
   int s16_i, s16_l_limit;
   int s16_temp[12];

   for (s16_i=0; s16_i<=11; ++s16_i)
   {
      s16_l_limit = -32768;
      if ((s16_i == 1) || (s16_i == 3) || (s16_i == 5) || (s16_i == 7) || (s16_i == 9) || (s16_i == 11) ) s16_l_limit = 0;

      if (s64_Execution_Parameter[s16_i] < (long long)s16_l_limit) return (VALUE_TOO_LOW);
      if (s64_Execution_Parameter[s16_i] > 32767LL) return (VALUE_TOO_HIGH);
   }

   /* Accept new params */
   for (s16_i=0;s16_i<=11;++s16_i)
   {
      s16_temp[s16_i] = BGVAR(s16_Vh)[s16_i];
      BGVAR(s16_Vh)[s16_i] = (int)s64_Execution_Parameter[s16_i];
   }

   // Configure the velocity loop - and if failed, restore previous parameters
      if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
    {
      // Restore previous VH values
      for (s16_i=0; s16_i<=11; ++s16_i)
         BGVAR(s16_Vh)[s16_i] = s16_temp[s16_i];

      VelocityConfig(DRIVE_PARAM);
      return (VELOCITY_CONFIG_FAIL);
    }

   return (SAL_SUCCESS);
}


int SalReadVrCommand(int drive)
{
   int s16_i;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for (s16_i=0;s16_i<=8;++s16_i)
   {
      PrintSignedInteger(BGVAR(s16_Vr)[s16_i]);
      PrintChar(SPACE);
   }
   PrintSignedInteger(BGVAR(s16_Vr)[9]);
   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalVrCommand(int drive)
{
   int s16_i, s16_l_limit;
   int s16_temp[10];

   for (s16_i=0; s16_i<=9; ++s16_i)
   {
      s16_l_limit = -32768;
      if ((s16_i == 1) || (s16_i == 3) || (s16_i == 5) || (s16_i == 7) || (s16_i == 9)) s16_l_limit = 0;

      if (s64_Execution_Parameter[s16_i] < (long long)s16_l_limit) return (VALUE_TOO_LOW);
      if (s64_Execution_Parameter[s16_i] > 32767LL) return (VALUE_TOO_HIGH);
   }

   /* Accept new params */
   for (s16_i=0; s16_i<=9; ++s16_i)
   {
      s16_temp[s16_i] = BGVAR(s16_Vr)[s16_i];
      BGVAR(s16_Vr)[s16_i] = (int)s64_Execution_Parameter[s16_i];
   }

   // Configure the velocity loop - and if failed, restore previous parameters
      if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
    {
      // Restore previous VR values
      for (s16_i=0; s16_i<=9; ++s16_i)
         BGVAR(s16_Vr)[s16_i] = s16_temp[s16_i];

      VelocityConfig(DRIVE_PARAM);
      return (VELOCITY_CONFIG_FAIL);
    }

   return (SAL_SUCCESS);
}


int SalStepCommand(int drive)
{
   // AXIS_OFF;


   long long s64_temp,s64_temp2;
   int index = 0;

   if ((s16_Number_Of_Parameters == 1) || (s16_Number_Of_Parameters > 4)) return SYNTAX_ERROR;

   if (s16_Number_Of_Parameters == 0) return (NOT_AVAILABLE);

   if (FILEDBUS_MODE)            return (WRONG_COMMODE);
   if (VAR(AX0_s16_Opmode) != 0) return (INVALID_OPMODE);

   s64_temp = s64_Execution_Parameter[0];

   // to bypass schneider limitations
   // if t1 command is zero, disable bypass.
   if (s64_temp == 0)
   {
       BGVAR(u16_Schneider_Bypass) &= ~2;
   }

   if (s64_temp <= 0) return (VALUE_TOO_LOW);

   index = UnitsConversionIndex(&s8_Units_Vel_Out_Loop, drive);

   s64_temp = MultS64ByFixS64ToS64(s64_Execution_Parameter[1],
            BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);

   if (s64_temp > (long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_HIGH);
   if (s64_temp < -(long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_LOW);

   BGVAR(s32_Time1) = (long)s64_Execution_Parameter[0];
   BGVAR(s32_Jog1) = (long)s64_temp;

   if (s16_Number_Of_Parameters == 2)
   {
      BGVAR(s32_Time2) = 0L;
      BGVAR(s32_Jog2) = 0L;
   }
   else if (s16_Number_Of_Parameters == 3)
   {
      if (s64_Execution_Parameter[2] <= 0) return (VALUE_TOO_LOW);

      BGVAR(s32_Time2) = (long)s64_Execution_Parameter[2];
      BGVAR(s32_Jog2)  = 0L;
   }
   else if (s16_Number_Of_Parameters == 4)
   {
      s64_temp2 = s64_Execution_Parameter[2];
      if (s64_temp2 <= 0) return (VALUE_TOO_LOW);

      s64_temp2 = MultS64ByFixS64ToS64(s64_Execution_Parameter[3],
            BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
               BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);

      if (s64_temp2 > (long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_HIGH);
      if (s64_temp2 < -(long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_LOW);

      BGVAR(s32_Time2) = (long)s64_Execution_Parameter[2];
      BGVAR(s32_Jog2)  = (long)s64_temp2;
   }

   // to bypass schneider limitations, enable it.
   BGVAR(u16_Schneider_Bypass) |= 2;
   BGVAR(StepHandlerState) = STEP_JOG1;

   return (SAL_SUCCESS);
}

// Same as SalStepCommand just without the units conversion to be called internaly
int StepCommand(int drive)
{
   // AXIS_OFF;
   long long s64_temp,s64_temp2;
   REFERENCE_TO_DRIVE;


   if ((s16_Number_Of_Parameters == 1) || (s16_Number_Of_Parameters > 4)) return SYNTAX_ERROR;

   if (s16_Number_Of_Parameters == 0) return (NOT_AVAILABLE);

   if (VAR(AX0_s16_Opmode) != 0) return (INVALID_OPMODE);

   s64_temp = s64_Execution_Parameter[0];
   if (s64_temp <= 0) return (VALUE_TOO_LOW);

   s64_temp = s64_Execution_Parameter[1];
   if (s64_temp > (long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_HIGH);
   if (s64_temp < -(long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_LOW);

   BGVAR(s32_Time1) = (long)s64_Execution_Parameter[0];
   BGVAR(s32_Jog1) = (long)s64_temp;

   if (s16_Number_Of_Parameters == 2)
   {
      BGVAR(s32_Time2) = 0L;
      BGVAR(s32_Jog2) = 0L;
   }
   else if (s16_Number_Of_Parameters == 3)
   {
      if (s64_Execution_Parameter[2] <= 0) return (VALUE_TOO_LOW);

      BGVAR(s32_Time2) = (long)s64_Execution_Parameter[2];
      BGVAR(s32_Jog2)  = 0L;
   }
   else if (s16_Number_Of_Parameters == 4)
   {
      s64_temp2 = s64_Execution_Parameter[2];
      if (s64_temp2 <= 0) return (VALUE_TOO_LOW);

      s64_temp2 = s64_Execution_Parameter[3];

      if (s64_temp2 > (long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_HIGH);
      if (s64_temp2 < -(long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_LOW);

      BGVAR(s32_Time2) = (long)s64_Execution_Parameter[2];
      BGVAR(s32_Jog2)  = (long)s64_temp2;
      }

      BGVAR(StepHandlerState) = STEP_JOG1;

      return (SAL_SUCCESS);
}


int JogCommand(long long lparam,int drive)
{
   // AXIS_OFF;
   long long s64_half_for_rounding = 0LL;
   REFERENCE_TO_DRIVE;

//   if (VAR(AX0_s16_Opmode) != 0) return (INVALID_OPMODE);

   if (BGVAR(u16_AD_In_Process)) return (AD_IN_PROCCESS);

   if (lparam > (long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_HIGH);
   if (lparam < -(long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_LOW);

   BGVAR(s32_Jog) = (long)lparam;

   // convert from internal velocity (out of the loop) to internal units (in the loop)

   if (VAR(AX0_u16_Out_To_In_Vel_Shr) > 0)
      s64_half_for_rounding = 1LL << ((long long)VAR(AX0_u16_Out_To_In_Vel_Shr) - 1);

   VAR(AX0_s16_Serial_Vel_Cmnd) = (int)((((long long)BGVAR(s32_Jog) * (long long)LVAR(AX0_s32_Out_To_In_Vel_Fix)) + s64_half_for_rounding) >> (long long)VAR(AX0_u16_Out_To_In_Vel_Shr));
   return (SAL_SUCCESS);
}
int SalJogSpd1Command(int drive)
{
   drive+=0;
   if (s64_Execution_Parameter[0] > ((long long)BGVAR(s32_V_Lim_Design))) return (VALUE_TOO_HIGH);
   else
   {
      s64_User_Jog_Vel =  s64_Execution_Parameter[0];
      return (SAL_SUCCESS);
   }
}

int SalJogSpd2Command(int drive)
{   
   drive+=0;
   if (s64_Execution_Parameter[0] > ((long long)BGVAR(s32_V_Lim_Design))) return (VALUE_TOO_HIGH);
   else
   {
      s64_User_Jog_Vel2 = s64_Execution_Parameter[0];
      return (SAL_SUCCESS);
   }
}

// J command
int SalJogCommand(int drive)
{
   // AXIS_OFF;
   long long s64_temp;
   int ret_value = SAL_SUCCESS;

   if (FILEDBUS_MODE)                  return (WRONG_COMMODE);

   if (VAR(AX0_s16_Opmode) != 0) return (INVALID_OPMODE);

   if (BGVAR(u16_Est_Motor_Params) != 0) return (MOTOR_PARAMS_EST_IN_PROCESS);

   ret_value = TestJogCommand(s64_Execution_Parameter[0]);

   if(ret_value != SAL_SUCCESS)
      return ret_value; //return reject reason: hold or limit

   if (s16_Number_Of_Parameters == 1)
   {
      BGVAR(StepHandlerState) = STEP_IDLE;

      // to bypass schneider limitations
      // if t1 command is zero, disable bypass. other command will eanble it.
      if (s64_Execution_Parameter[0])
            BGVAR(u16_Schneider_Bypass) |= 2;
      else
            BGVAR(u16_Schneider_Bypass) &= ~2;
      return JogCommand(s64_Execution_Parameter[0], drive);
   }
   else if (s16_Number_Of_Parameters == 2)
   {
      STORE_EXECUTION_PARAMS_0_1
      s64_temp = s64_Execution_Parameter[0];
      s64_Execution_Parameter[0] = s64_Execution_Parameter[1];
      s64_Execution_Parameter[1] = s64_temp;

      // to bypass schneider limitations, enable it.
      BGVAR(u16_Schneider_Bypass) |= 2;

      ret_value = StepCommand(drive);
      RESTORE_EXECUTION_PARAMS_0_1
      return ret_value;
   }

   return (SAL_SUCCESS);
}

#pragma CODE_SECTION(TestJogCommand, "ramcan");
int TestJogCommand(long long s64_data)
{
   // AXIS_OFF;

   int ret_value = SAL_SUCCESS;

   unsigned int u16_temp_cw_ls;
   unsigned int u16_temp_ccw_ls;

   // For Schneider, or when setting Limits Mode to ignore Transient (Phantom) Bits
   if ( (u16_Product == SHNDR) || (u16_Product == SHNDR_HW) ||
        ((VAR(AX0_u16_SW_Pos_lim) & 0x02) == 0x02)            )
   {  // Do not use the hardware limit-switch phantom bits
      u16_temp_cw_ls  = VAR(AX0_u16_CW_LS)  & 0x03;
      u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS) & 0x03;
   }
   else
   {  // Involve the hardware limit-switch phantom bits
      u16_temp_cw_ls  = VAR(AX0_u16_CW_LS);
      u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS);
   }

   //if (BGVAR(u16_Hold_User))
   if (BGVAR(u16_Hold_Bits) & HOLD_USER_MASK)
   {
      ret_value  = HOLD_MODE;
   }

   else if ((u16_temp_cw_ls != 0) || (u16_temp_ccw_ls != 0))
   {
       if ( ((u16_temp_cw_ls != 0)  && (s64_data > 0)) ||
            ((u16_temp_ccw_ls != 0) && (s64_data < 0))  )
       {
            // alarm for lxm28 in canopen mode only (can alarams task)
            if (((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)) &&
                (((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_CANOPEN)       ||
                 ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_SERCOS)        ||
                 ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_ETHERNET_IP)   ||
                 ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_ETHERCAT)         )   )
            {
               if (u16_temp_cw_ls & 0x01)
               {
                  // schnieder alarm 283 (command towards positive software limit)
                  BGVAR(u64_Sys_Warnings) |= WRN_COMMAND_TOWARDS_CW_SW_LIMIT_MASK;
               }

               if (u16_temp_ccw_ls & 0x01)
               {
                  // schnieder alarm 285 (command towards negative software limit)
                  BGVAR(u64_Sys_Warnings) |= WRN_COMMAND_TOWARDS_CCW_SW_LIMIT_MASK;
               }
            }

            ret_value = COMMAND_TOWARDS_LIMIT;
       }
   }

   // clear can alarm
   BGVAR(u64_Sys_Warnings) &= ~WRN_COMMAND_TOWARDS_CW_SW_LIMIT_MASK;
   BGVAR(u64_Sys_Warnings) &= ~WRN_COMMAND_TOWARDS_CCW_SW_LIMIT_MASK;

   return ret_value;
}


//**********************************************************
// Function Name: CheckAccDecLimits
// Description:
//          This function checks that acc value in in min/max limits.
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int CheckAccDecLimits(long long lparam)
{
    // AXIS_OFF;
    long long s64_max_acc_dec,s64_min_acc_dec;

    if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC)
    {
       s64_max_acc_dec = MAX_ACC_DEC_TSP250;
    }
    else
    {
       s64_max_acc_dec = MAX_ACC_DEC_TSP125;
    }

    // value must be between 0.06 rpm/sec to 1,000,000 rpm/sec
    if (lparam > s64_max_acc_dec) // internal value for 1,000,000 rpm/sec
    {
        return VALUE_TOO_HIGH;
    }

    if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC)
    {
        s64_min_acc_dec = MIN_ACC_DEC_TSP250;
    }
    else
    {
        s64_min_acc_dec = MIN_ACC_DEC_TSP125;
    }

    // internal value for 0.23 rpm/sec
    if (lparam < s64_min_acc_dec)
    {
        return VALUE_TOO_LOW;
    }

    return SAL_SUCCESS;
}

int SalMotorAccCommand (long long lparam,int drive)
{
   int retVal = CheckAccDecLimits(lparam);

   if (retVal != SAL_SUCCESS)
   {
      return retVal;
   }

//TODO: need to discuss with Nitsan if the CAN limit check is needed not in CAN mode and if it possible to seperate it - With Avishay.
   if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
   {
      // in DS402 there is limit function on acc/dec
     if (lparam > BGVAR(u64_CAN_Max_Acc))
     {
        lparam = BGVAR(u64_CAN_Max_Acc);
     }
   }   
   BGVAR(u64_Mfb_AccRate) = lparam;
  
   UpdateAccDec(drive,ACC_UPDATE);
//   UpdateFieldbusAccLimit(drive); // Not in use anymore

   return (SAL_SUCCESS);
}

int SalMotorDecCommand (long long lparam,int drive)
{
   int retVal = CheckAccDecLimits(lparam);

   if (retVal != SAL_SUCCESS)
   {
      return retVal;
   }

   if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
   {
      // in DS402 there is limit function on acc/dec
     if (lparam > BGVAR(u64_CAN_Max_Dec))
     {
        lparam = BGVAR(u64_CAN_Max_Dec);
     }
   }

   BGVAR(u64_Mfb_DecRate) = lparam;
     
   UpdateAccDec(drive,DEC_UPDATE);
//   UpdateFieldbusDecLimit(drive); // Not in use anymore

   return (SAL_SUCCESS);
}

int SalSfbAccCommand (long long lparam,int drive)
{
   int retVal = CheckAccDecLimits(lparam);

   if (retVal != SAL_SUCCESS)
   {
      return retVal;
   }

//TODO: need to discuss with Nitsan if the CAN limit check is needed not in CAN mode and if it possible to seperate it - With Avishay.
   if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
   {
      // in DS402 there is limit function on acc/dec
     if (lparam > BGVAR(u64_CAN_Max_Acc))
     {
        lparam = BGVAR(u64_CAN_Max_Acc);
     }
   }
   
   BGVAR(u64_Sfb_AccRate) = lparam;
  
   UpdateAccDec(drive,ACC_UPDATE);
//   UpdateFieldbusAccLimit(drive); // Not in use anymore

   return (SAL_SUCCESS);
} 

int SalSfbDecCommand (long long lparam,int drive)
{
   int retVal = CheckAccDecLimits(lparam);

   if (retVal != SAL_SUCCESS)
   {
      return retVal;
   }

   if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
   {
      // in DS402 there is limit function on acc/dec
     if (lparam > BGVAR(u64_CAN_Max_Dec))
     {
        lparam = BGVAR(u64_CAN_Max_Dec);
     }
   }

   BGVAR(u64_Sfb_DecRate) = lparam;
     
   UpdateAccDec(drive,DEC_UPDATE);
//   UpdateFieldbusDecLimit(drive); // Not in use anymore

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: AccRateCommand
// Description:
//          This function is called in response to the ACC command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalAccCommand(long long lparam,int drive)
{
    int retVal = CheckAccDecLimits(lparam);

    if (retVal != SAL_SUCCESS)
    {
        return retVal;
    }

    //  TODO: need to discuss with Nitsan if the CAN limit check is needed not in CAN mode and if it possible to seperate it - With Avishay.
    if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
    {
        // in DS402 there is limit function on acc/dec
        if (lparam > BGVAR(u64_CAN_Max_Acc))
        {
            lparam = BGVAR(u64_CAN_Max_Acc);
        }
    }
   
    if (IS_DUAL_LOOP_ACTIVE)
    {
        BGVAR(u64_Sfb_AccRate) = lparam;
    }
    else
    {
        BGVAR(u64_Mfb_AccRate) = lparam;
    }

    UpdateAccDec(drive, ACC_UPDATE);

    return SAL_SUCCESS;
}

//**********************************************************
// Function Name: DecRateCommand
// Description:
//          This function is called in response to the DEC command.
//
//
// Author: Dimitry
// Algorithm:
// Revisions:
//**********************************************************
int SalDecCommand(long long lparam,int drive)
{
   int retVal = CheckAccDecLimits(lparam);

   if (retVal != SAL_SUCCESS)
   {
      return retVal;
   }

   if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
   {
      // in DS402 there is limit function on acc/dec
     if (lparam > BGVAR(u64_CAN_Max_Dec))
     {
        lparam = BGVAR(u64_CAN_Max_Dec);
     }
   }

   if (IS_DUAL_LOOP_ACTIVE) 
     BGVAR(u64_Sfb_DecRate) = lparam;
   else
     BGVAR(u64_Mfb_DecRate) = lparam;
     
   UpdateAccDec(drive,DEC_UPDATE);
//   UpdateFieldbusDecLimit(drive); // Not in use anymore

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalWriteAccFromMs
// Description:
//          This function is called in response to the P1-34 (TACC) write in Schneider drive mode.
//          this function convert the entered ms to acc rate from 0 to 6000 RPM
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteAccFromMs(long long param,int drive)
{
   // convert from ms to CDHD acc units and then use the regular CDHD ACC write function
   long long tempAcc = convertMsToAccDecInternalUnitsForPos(drive,param);
   int retVal = SalAccCommand(tempAcc,drive);
   // 1/ACC -> if ms too low acc too high.
   if(retVal == (VALUE_TOO_HIGH))
   {
      retVal = (VALUE_TOO_LOW);
   }
   else if(retVal == (VALUE_TOO_LOW))
   {
      retVal = (VALUE_TOO_HIGH);
   }
   return retVal;
}


//**********************************************************
// Function Name: SalReadAccInMs
// Description:
//          This function is called in response to the P1-34 (TACC) read in Schneider drive mode.
//          this function convert the current acc rate back to ms
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadAccInMs(long long *data,int drive)
{
   *data =(long long)convertAccDecInternalUnitsForPosToMs(drive,BGVAR(u64_AccRate));
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalWriteDecFromMs
// Description:
//          This function is called in response to the P1-35 (TDEC) write in Schneider drive mode.
//          this function convert the entered ms to dec rate from 6000 to 0 RPM
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteDecFromMs(long long param,int drive)
{
   // convert from ms to CDHD acc units and then use the regular CDHD ACC write function
   long long tempDec = convertMsToAccDecInternalUnitsForPos(drive,param);
   int retVal = SalDecCommand(tempDec,drive);
   // 1/DEC -> if ms too low dec too high.
   if(retVal == (VALUE_TOO_HIGH))
   {
      retVal = (VALUE_TOO_LOW);
   }
   else if(retVal == (VALUE_TOO_LOW))
   {
      retVal = (VALUE_TOO_HIGH);
   }
   return retVal;
}

//**********************************************************
// Function Name: SalReadDecInMs
// Description:
//          This function is called in response to the P1-35 (TDEC) read in Schneider drive mode.
//          this function convert the current dec rate back to ms
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadDecInMs(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;
   *data =(long long)convertAccDecInternalUnitsForPosToMs(drive,BGVAR(u64_DecRate));
   return (SAL_SUCCESS);
}

int SalDecStopCommand(long long lparam,int drive)
{
   // AXIS_OFF;

   int retVal = CheckAccDecLimits(lparam);
   REFERENCE_TO_DRIVE;

   if (retVal != SAL_SUCCESS)
   {
      return retVal;
   }

   if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
   {
      // in DS402 there is limit function on acc/dec
     if (lparam > BGVAR(u64_CAN_Max_Dec))
     {
        lparam = BGVAR(u64_CAN_Max_Dec);
     }
   }

   BGVAR(u64_DecStop) = (long long)lparam;   
   BGVAR(u64_CAN_Dec_Stop) = (long long)lparam;

   LVAR(AX0_s32_Ptp_Dec_Stop_Rounded) = (long)((BGVAR(u64_DecStop) + (unsigned long long)0x080000000) >> 32);

   return (SAL_SUCCESS);
}

float CalculateDecStopTime1OverT(int drive)
{
   // AXIS_OFF;
   float f_1_over_t = 0.0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_DecStopTime))
   {
      f_1_over_t = 1.0 / 8.94784853; // Translate to RPM
      f_1_over_t = f_1_over_t * 1000.0 / (float)BGVAR(u16_DecStopTime);
      f_1_over_t *= 19215358.41; // Translate to internal units

      //consider POSCONTROLMODE
      if(TSP_125_USEC == VAR(AX0_u16_Pos_Loop_Tsp))
	  {
	     f_1_over_t /= 4;
	  }
   }
   return f_1_over_t;
}

int SalDecStopTimeCommand(long long param,int drive)
{
   // AXIS_OFF;

   BGVAR(u16_DecStopTime) = (unsigned int)param;

   FloatToFixS32Shift16(&LVAR(AX0_s32_DecStopTime_1_Over_T_Fix),&VAR(AX0_u16_DecStopTime_1_Over_T_Shr),CalculateDecStopTime1OverT(drive));

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalFiltModeCommand
// Description:
//          This function is called in response to the FiltMode command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalFiltModeCommand(long long lparam,int drive)
{
   unsigned char u8_temp = BGVAR(u8_VFO_Mode);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u8_VFO_Mode) = (unsigned char)lparam;
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      BGVAR(u8_VFO_Mode) = u8_temp;
      return (VELOCITY_CONFIG_FAIL);
   }

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalSchneiderControlModeCommand
// Description:
//          This function is called in response to the P8-35 parameter in Schneider drive mode.
//
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalSchneiderControlModeCommand(long long lparam,int drive)
{
   int s16_vel_param, s16_pos_param, s16_return_value;
   unsigned char u8_backup_vel;

   s16_vel_param = ((int)lparam) & 0x00ff;
   s16_pos_param = (((int)lparam) >> 8) & 0x00ff;
   if((s16_vel_param > 7) || (s16_vel_param < 5)) return (VALUE_OUT_OF_RANGE);
   if((s16_pos_param != 0) && (s16_pos_param != 2) && (s16_pos_param != 5)) return (VALUE_OUT_OF_RANGE);

   // backup current variables, so in case of reject, it can be restored
   u8_backup_vel = BGVAR(u8_CompMode);

   // Set VELCONTROLMODE
   s16_return_value = SalCompModeCommand((long long)s16_vel_param, drive);

   if (s16_return_value == SAL_SUCCESS)
   {
      // Set POSCONTROLMODE
      if (s16_pos_param == 0) s16_pos_param = 2;     // handle 0 as POSCONTROLMODE=2 for backwards compatibility
      s16_return_value = SalPoscontrolModeCommand((long long)s16_pos_param, drive);

      if (s16_return_value != SAL_SUCCESS)
      {
         // restore VELCONTROLMODE
         SalCompModeCommand((long long)u8_backup_vel, drive);
         return (s16_return_value);
      }

   }
   else
   {
      return (s16_return_value);
   }

   // Setting VELCONTROLMODE and POSCONTROLMODE passed successfully.
   BGVAR(u16_Pos_Vel_Control_Modes) = (unsigned int)lparam;

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalCompModeCommand
// Description:
//          This function is called in response to the Compmode command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalCompModeCommand(long long lparam,int drive)
{
   // AXIS_OFF;
   unsigned char u8_temp = BGVAR(u8_CompMode);

   BGVAR(u8_CompMode) = (unsigned char)lparam;
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      BGVAR(u8_CompMode) = u8_temp;
      return (VELOCITY_CONFIG_FAIL);
   }

   // set curret loop ptr apprpriate source according to u8_CompMode
   if (VAR(AX0_s16_Opmode) == 0) SetOpmode(drive,0);
   if (VAR(AX0_s16_Opmode) == 1) SetOpmode(drive,1);

   IsLinearVelocityNeeded(drive);
   IsPositionLoopNeeded(drive);

   return (SAL_SUCCESS);
}

// Test if linear velocity loop should be activated, if not this will save some RT execution time
// Used if (opmode=0,1 and compmode<>5,6) or (opmode=4,8  and  (poscontrolmode=0 or dual loop active) )
void IsLinearVelocityNeeded(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if ((VAR(AX0_s16_Opmode) <= 1) && (BGVAR(u8_CompMode) != 5) && (BGVAR(u8_CompMode) != 6) && (BGVAR(u8_CompMode) != 7))
      VAR(AX0_u16_Use_Linear_Vel) = 1;
   else
   {
      if (((VAR(AX0_s16_Opmode) == 4) || (VAR(AX0_s16_Opmode) == 8)) && ((VAR(AX0_u16_Pos_Control_Mode)==LINEAR_POSITION) || (IS_DUAL_LOOP_ACTIVE)))
         VAR(AX0_u16_Use_Linear_Vel) = 1;
      else
         VAR(AX0_u16_Use_Linear_Vel) = 0;
   }
}

// Test if position loop should be activated, if not this will save some RT execution time
// Used if (opmode=4,8) or (opmode=0/1 and compmode=5,6)
void IsPositionLoopNeeded(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if ((VAR(AX0_s16_Opmode) <= 1) && ((BGVAR(u8_CompMode) == 5) || (BGVAR(u8_CompMode) == 6) || (BGVAR(u8_CompMode) == 7)))
   {
      VAR(AX0_u16_Use_Position_Loop) |= USE_POSITION_LOOP_SETTINGS_MASK;
      VAR(AX0_u16_Use_Position_Loop) |= ENTER_POS_TASK_MASK;
   }
   else
   {
      if ((VAR(AX0_s16_Opmode) == 4) || (VAR(AX0_s16_Opmode) == 8)) 
      {
         VAR(AX0_u16_Use_Position_Loop) |= USE_POSITION_LOOP_SETTINGS_MASK;
         
         if (IS_DUAL_LOOP_ACTIVE)
            VAR(AX0_u16_Use_Position_Loop) &= ~ENTER_POS_TASK_MASK;
         else
            VAR(AX0_u16_Use_Position_Loop) |= ENTER_POS_TASK_MASK;
      }
      else
      {
         VAR(AX0_u16_Use_Position_Loop) &= ~USE_POSITION_LOOP_SETTINGS_MASK;
         VAR(AX0_u16_Use_Position_Loop) &= ~ENTER_POS_TASK_MASK;
      }
   }
}
//**********************************************************
// Function Name: SalVelFiltModeCommand
// Description:
//          This function is called in response to the VELFILTMODE command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalVelFiltModeCommand(long long lparam,int drive)
{
   if (lparam > 1LL) return (NOT_AVAILABLE); // Modes 2 and 3 is temporary limited S.S.
   BGVAR(u8_VelFiltMode) = (unsigned char)lparam;

   CalcFbFilterGain(drive);
   return (SAL_SUCCESS);
}

int SalLinPe3FcenterCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   BGVAR(u32_PL_Antivib3_Fcenter) = (unsigned long)param;
   VelocityConfig(DRIVE_PARAM);
   return (SAL_SUCCESS);
}

int SalLinPe3SharpnessCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   BGVAR(u16_PL_Antivib3_Sharpness) = (unsigned int)param;
   VelocityConfig(DRIVE_PARAM);
   return (SAL_SUCCESS);
}

int SalLinPe3GainCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   BGVAR(s32_PL_Antivib3_Gain) = (long)param;
   VelocityConfig(DRIVE_PARAM);
   return (SAL_SUCCESS);
}

int SalLinPe3QGainCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   BGVAR(s32_PL_Antivib3_Q_Gain) = (long)param;
   VelocityConfig(DRIVE_PARAM);
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalLmjrCommand
// Description:
//          This function is called in response to the LMJR command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalLmjrCommand(long long param,int drive)
{
   if (BGVAR(s16_LMJR_Mode) != 0) return (NOT_AVAILABLE);

   return (UpdateLmjr(param,drive));
}

int UpdateLmjr(long long param,int drive)
{
   unsigned long u32_temp = BGVAR(u32_LMJR_User);
   BGVAR(u32_LMJR_User)  = (unsigned long)param;
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      BGVAR(u32_LMJR_User) =  u32_temp;
      return (VELOCITY_CONFIG_FAIL);
   }

   PositionConfig(drive,0);

   return (SAL_SUCCESS);
}

int SalLmjrResCommand(long long param,int drive)
{
   BGVAR(u32_LMJR_Resonance) = (unsigned long)param;

   PositionConfig(drive,0);

   return (SAL_SUCCESS);
}

int SalVFOHz1Command(long long param,int drive)
{
   unsigned int u16_temp = BGVAR(u16_VFO_Hz1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_VFO_Hz1) = (unsigned int)param;
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      BGVAR(u16_VFO_Hz1) = u16_temp;
      return (VELOCITY_CONFIG_FAIL);
   }

   return (SAL_SUCCESS);
}

int SalVFOHz2Command(long long param,int drive)
{
   unsigned int u16_temp = BGVAR(u16_VFO_Hz2);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_VFO_Hz2) = (unsigned int)param;
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      BGVAR(u16_VFO_Hz2) = u16_temp;
      return (VELOCITY_CONFIG_FAIL);
   }

   return (SAL_SUCCESS);
}

void StepHandler(int drive)
{
   // AXIS_OFF;

   if ((!Enabled(drive)) || (VAR(AX0_s16_Opmode) != 0)) BGVAR(StepHandlerState) = STEP_IDLE;

   switch (BGVAR(StepHandlerState))
   {
      case STEP_IDLE:
      break;

      case STEP_JOG1:
         JogCommand((long long)BGVAR(s32_Jog1), drive);
         BGVAR(StepHandlerTimer) = Cntr_1mS;
         BGVAR(StepHandlerState) = STEP_TIME1;
      break;

      case STEP_TIME1:
         if (PassedTimeMS(BGVAR(s32_Time1),BGVAR(StepHandlerTimer)))
         {
            if (BGVAR(s32_Time2) == 0L)
            {
               JogCommand(0LL, drive);
               BGVAR(StepHandlerState) = STEP_IDLE;
            }
            else BGVAR(StepHandlerState) = STEP_JOG2;
         }
      break;

      case STEP_JOG2:
         JogCommand((long long)BGVAR(s32_Jog2), drive);
         BGVAR(StepHandlerTimer) = Cntr_1mS;
         BGVAR(StepHandlerState) = STEP_TIME2;
      break;

      case STEP_TIME2:
         if (PassedTimeMS(BGVAR(s32_Time2),BGVAR(StepHandlerTimer)))
            BGVAR(StepHandlerState) = STEP_JOG1;
      break;
   }
}


int SalIGravCommand(long long param,int drive)
{
    // AXIS_OFF;
   REFERENCE_TO_DRIVE;

    if (param > (long long)26214) return (VALUE_TOO_HIGH);
    if (param < (long long)-26214) return (VALUE_TOO_LOW);

    VAR(AX0_s16_Igrav) = (int)param;

    return (SAL_SUCCESS);
}


void CurrentFfLpfConfig(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   VAR(AX0_s16_Crrnt_Ff_Lpf_Alpha) = (int)((float)(32768.0 * (float)exp((float)((-6.2831853) * (float)BGVAR(s16_Iff_Lpf_Hz) * 0.000125))));
   VAR(AX0_s16_Crrnt_Ff_Lpf_Beta) = 0x8000 - VAR(AX0_s16_Crrnt_Ff_Lpf_Alpha);  // Alpha can not be 0 (not in the range)
}


int SalIffLpfHzCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;
   BGVAR(s16_Iff_Lpf_Hz) = (int)param;
   CurrentFfLpfConfig(drive);

   return (SAL_SUCCESS);
}


int SalFricNegCommand(long long lparam,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (lparam < 0) return (VALUE_OUT_OF_RANGE);
   VAR(AX0_s16_Ifric_Neg) = (int)lparam;
   return (SAL_SUCCESS);
}


int SalFricPosCommand(long long lparam,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (lparam < 0) return (VALUE_OUT_OF_RANGE);
   VAR(AX0_s16_Ifric_Pos) = (int)lparam;
   return (SAL_SUCCESS);
}


int SalWeightCompPosVelHystCommand(long long lparam,int drive)
{
   long long s64_upper_limit = 0x888889LL;    // 1000 rpm expressed in internal units (velocity out of the loop)

   if (lparam < -s64_upper_limit) return (VALUE_TOO_LOW);
   if (lparam >  s64_upper_limit) return (VALUE_TOO_HIGH);

   BGVAR(s32_Weight_Pos_Vel_Hyst) = (long)lparam;
   ConvertWeightCompVelToInternalInTheLoop(drive);

   return (SAL_SUCCESS);
}


int SalWeightCompNegVelHystCommand(long long lparam,int drive)
{
   long long s64_upper_limit = 0x888889LL;    // 1000 rpm expressed in internal units (velocity out of the loop)
   REFERENCE_TO_DRIVE;

   if (lparam < -s64_upper_limit) return (VALUE_TOO_LOW);
   if (lparam >  s64_upper_limit) return (VALUE_TOO_HIGH);

   BGVAR(s32_Weight_Neg_Vel_Hyst) = (long)lparam;
   ConvertWeightCompVelToInternalInTheLoop(drive);

   return (SAL_SUCCESS);
}


void ConvertWeightCompVelToInternalInTheLoop(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if (BGVAR(s32_Weight_Neg_Vel_Hyst) >= BGVAR(s32_V_Lim_Design))
   {
      VAR(AX0_s16_Weight_Neg_Vel_Hyst) = 22750;
   }
   else if (BGVAR(s32_Weight_Neg_Vel_Hyst) <= -BGVAR(s32_V_Lim_Design))
   {
      VAR(AX0_s16_Weight_Neg_Vel_Hyst) = -22750;
   }
   else
   {
      VAR(AX0_s16_Weight_Neg_Vel_Hyst) = (int)((float)BGVAR(s32_Weight_Neg_Vel_Hyst) * 22750.0 / (float)BGVAR(s32_V_Lim_Design));
   }


   if (BGVAR(s32_Weight_Pos_Vel_Hyst) >= BGVAR(s32_V_Lim_Design))
   {
      VAR(AX0_s16_Weight_Pos_Vel_Hyst) = 22750;
   }
   else if (BGVAR(s32_Weight_Pos_Vel_Hyst) <= -BGVAR(s32_V_Lim_Design))
   {
      VAR(AX0_s16_Weight_Pos_Vel_Hyst) = -22750;
   }
   else
   {
      VAR(AX0_s16_Weight_Pos_Vel_Hyst) = (int)((float)BGVAR(s32_Weight_Pos_Vel_Hyst) * 22750.0 / (float)BGVAR(s32_V_Lim_Design));
   }

   // support non lienar controller - different units
   if (AX0_u16_Pos_Control_Mode < NON_LINEAR_POSITION_5)
   {
      LVAR(AX0_s32_Fric_Hyst_Neg) = -BGVAR(s32_Weight_Neg_Vel_Hyst) << 1;
      LVAR(AX0_s32_Fric_Hyst_Pos) = BGVAR(s32_Weight_Pos_Vel_Hyst) << 1;
   }
   else
   {
      LVAR(AX0_s32_Fric_Hyst_Neg) = BGVAR(s32_Weight_Neg_Vel_Hyst);
      LVAR(AX0_s32_Fric_Hyst_Pos) = BGVAR(s32_Weight_Pos_Vel_Hyst);
   }
}

void UpdateNCTVelLoopScale(int drive)
{
   // support for nct vel loop - generate delta poscmd to match current _AX0_s16_Vel_Acc_Dec_Cmnd
   // store value in AX0_s32_Pos_Vcmd[fbc32/Tsp]
    // fbc32/Tsp = _AX0_s16_Vel_Acc_Dec_Cmnd/22750*vlim(rpm)/60*2^32*Tsp
   //           = _AX0_s16_Vel_Acc_Dec_Cmnd*f_vlim_rpm(rpm)*0.78662404688644688644688644688645
   // For Tsp 125uSec need to divide above by 2
   // AXIS_OFF;
   float f_vlim_rpm = (float)BGVAR(s32_V_Lim_Design) * 0.00011176,f_temp = 0.5;
   REFERENCE_TO_DRIVE;

   if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) f_temp = 1;

   FloatToFix16Shift16(&VAR(AX0_s16_Vel_Var_NCT_Scale_Fix),(unsigned int *)&VAR(AX0_s16_Vel_Var_NCT_Scale_Shr),f_temp*f_vlim_rpm*0.78662404688644688644688644688645);
}

//**********************************************************
// Function Name: SalBWCommand
// Description:
//          This function is called in response to the BW command.
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int SalBWCommand(long long lparam,int drive)
{
   unsigned int u16_temp = BGVAR(u16_Bw);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_Bw) = (unsigned int)lparam;
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      BGVAR(u16_Bw) = u16_temp;
      VelocityConfig(DRIVE_PARAM);
      return (VELOCITY_CONFIG_FAIL);
   }

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalMJCommand
// Description:
//          This function is called in response to the MJ command.
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int SalMJCommand(long long lparam,int drive)
{
   long s32_temp = BGVAR(s32_Motor_J);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s32_Motor_J) = (long)lparam;
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      BGVAR(s32_Motor_J) = s32_temp;
      return (VELOCITY_CONFIG_FAIL);
   }

   if(s32_temp != BGVAR(s32_Motor_J))
      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault, to verify that MJ is not zero.

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMJCommand
// Description:
//          This function is called in response to the MJ command.
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int SalMMASSCommand(long long lparam,int drive)
{
   long s32_temp = BGVAR(s32_Motor_Mass);

   if (BGVAR(s32_Motor_Mass) == (long)lparam) return (SAL_SUCCESS);

   if (!(u16_Fw_Features & MMASS_LIMIT_MASK))
      return (NOT_SUPPORTED_ON_HW);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s32_Motor_Mass) = (long)lparam;
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      BGVAR(s32_Motor_Mass) = s32_temp;
      return (VELOCITY_CONFIG_FAIL);
   }
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalTFCommand
// Description:
//          This function is called in response to the TF command.
//
// Author: S.F
// Algorithm:
// Revisions:
//**********************************************************
int SalTFCommand(long long lparam,int drive)
{
   unsigned int u16_temp = BGVAR(u16_TF);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(u16_TF) = (unsigned int)lparam;
   if (VelocityConfig(DRIVE_PARAM) != SAL_SUCCESS)
   {
      BGVAR(u16_TF) = u16_temp;
      VelocityConfig(DRIVE_PARAM);
      return (VELOCITY_CONFIG_FAIL);
   }

   return (SAL_SUCCESS);
}

int SalPrbParamWriteCommand(int drive)
{
   //PRBPARAM <Type> <AmpI> <AmpV> <Counter>
   long long s64_temp;
   int index = 0;

   // AXIS_OFF;

   VAR(AX0_s16_I_AMP)=0;
   VAR(AX0_s16_V_AMP)=0;
   VAR(AX0_u16_PRB_Counter)=0;

   if (s16_Number_Of_Parameters != 4)
     return SYNTAX_ERROR;

   //Loading the Type
   // 0 - 8 bit random noise
   // 1 - 10 bit random noise
   // 2 - sine wave
   // 3 - square wave
   if (!((s64_Execution_Parameter[0]>=0) && (s64_Execution_Parameter[0]<4)))
   {
     PrintString("Type",0);
     return (VALUE_OUT_OF_RANGE);
   }
   VAR(AX0_u16_PRB_Type)=(int)s64_Execution_Parameter[0];

   //Loading the Current noise amplitude
   if (((long)s64_Execution_Parameter[1] <=BGVAR(s32_Ilim_User)) && ((long)s64_Execution_Parameter[1]>=0))
   {
     index = UnitsConversionIndex(&s8_Units_Cur, drive);

     s64_temp = MultS64ByFixS64ToS64(s64_Execution_Parameter[1],
           BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
              BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);
     VAR(AX0_s16_I_AMP)=s64_temp & 0xffff;
   }
   else
   return (VALUE_OUT_OF_RANGE);

   //Loading the Velocity noise amplitude
   index = UnitsConversionIndex(&s8_Units_Vel_Out_Loop, drive);

   s64_temp = MultS64ByFixS64ToS64(s64_Execution_Parameter[2],
           BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
              BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);

   if (s64_temp > (long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_HIGH);
   if (s64_temp < 0) return (VALUE_TOO_LOW);

   BGVAR(u32_PRB_V_Amp)=(unsigned long)s64_temp;

   index = UnitsConversionIndex(&s8_Units_Vel_In_Loop, drive);
   s64_temp = MultS64ByFixS64ToS64(s64_Execution_Parameter[2],
           BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
              BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);
   VAR(AX0_s16_V_AMP)=(int)s64_temp & 0xffff;

   //Loading the time interval= counter*31.25u[sec]
   if (s64_Execution_Parameter[3] > 0xffffLL) return (VALUE_TOO_HIGH);
   VAR(AX0_u16_PRB_Counter_Period)=(int)s64_Execution_Parameter[3];

   SalPRBFreqCommand((long long)BGVAR(u32_PRB_Freq), drive);
   return(SAL_SUCCESS);

}

int SalPrbParamReadCommand(int drive)
{
   // <Type>  <AmpI> <AmpV> <Counter>
   long long s64_temp;
   int index=0;
   REFERENCE_TO_DRIVE;
   // AXIS_OFF;
   PrintString("Type AmpI  AmpV  Period",0);
   PrintCrLf();
   PrintChar(SPACE);
   PrintUnsignedInteger(VAR(AX0_u16_PRB_Type));
   PrintString("   ",0);
   PrintUnsignedLongLong(((int)VAR(AX0_s16_I_AMP)*BGVAR(s32_Drive_I_Peak)+13107)/26214);
   PrintChar(SPACE);

   index = UnitsConversionIndex(&s8_Units_Vel_Out_Loop, drive);
   s64_temp = MultS64ByFixS64ToS64((long long)BGVAR(u32_PRB_V_Amp),
           BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_user_fix,
           BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_user_shr);
   PrintUnsignedLongLong(s64_temp);
   PrintChar(SPACE);
   PrintUnsignedInt16(VAR(AX0_u16_PRB_Counter_Period));
   PrintChar(SPACE);
   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalPRBFreqCommand(long long param,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   BGVAR(u32_PRB_Freq) = (unsigned long)param;
    // if PRB_Type is 0 or 1 then use PRB_Freq as cut-off frequency for HPF
   if ((VAR(AX0_u16_PRB_Type) == 0)||(VAR(AX0_u16_PRB_Type) == 1))
      {
       // Use PRB_Freq for alpha of HPF
      // HPF: out = alpha*(in - (in - out)*z^-1)
      // alpha = exp(-2*pi*Fc/Fs); Fc - cut-off freq; Fs = 32k/prb_counter_period
       VAR(AX0_s16_PRB_Freq) = (int)(16384.0*exp(-6.283185307*(float)BGVAR(u32_PRB_Freq)*(float)VAR(AX0_u16_PRB_Counter_Period)/32000000.0));
     }
   if (VAR(AX0_u16_PRB_Type) == 2)
      {
      if (BGVAR(u32_PRB_Freq) == 0L) BGVAR(u32_PRB_Freq) = 100;
       VAR(AX0_s16_PRB_Freq) = (int)((float)BGVAR(u32_PRB_Freq)*65536/32000000.0);
     }
   else if (VAR(AX0_u16_PRB_Type) == 3)
      {
      if (BGVAR(u32_PRB_Freq) == 0L) BGVAR(u32_PRB_Freq) = 100;
      VAR(AX0_s16_PRB_Freq) = (int)(1.0/(((float)BGVAR(u32_PRB_Freq) / 1000.0) *62.5e-6));
      }

   return (SAL_SUCCESS);
}

void DisplayFloat(float f_temp)
{
   long s32_temp;
   int s16_temp;
   float f_temp2;

   // Scale the value so it can be printed as X.YYY E+/-Z
   f_temp2 = f_temp;
   s16_temp = 0;

   if (f_temp2 < 0.0) f_temp2 = -f_temp2;

   if (f_temp2 != 0.0)
   {
      if (f_temp2 < 0.001)
      {
         while (f_temp2 < 1.0) {s16_temp--; f_temp2 *= 10.0;}
      }
      else if (f_temp > (float)(0x7FFFFFFF / 1000))
      {
         while (f_temp2 > 1.0) {s16_temp++; f_temp2 /= 10.0;}
      }
   }

   if (f_temp < 0.0) f_temp2 = -f_temp2;

   s32_temp = (long)(((float)1000 * f_temp2) + 0.5);
   PrintString(DecimalPoint32ToAscii(s32_temp),0);

   if (s16_temp)
   {
      PrintChar('E');
      PrintSignedInt16(s16_temp);
   }
}

// Display the VD,VH,VR,VF values


int VelDesignCommand(int drive)
{
   // AXIS_OFF;
   static int vel_design_state = 0;
   unsigned int u16_index = 0;
   unsigned int u16_max_index, u16_temp_shr[7];
   int s16_temp_fix[7];
   REFERENCE_TO_DRIVE;

   if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;

   if (VAR(AX0_u16_Pos_Control_Mode) != LINEAR_POSITION)
   {
      PrintStringCrLf("Non-Linear Loop Is Used", 0);
     strcpy((char*)manu_spec_Vel_Design_command, "Non-Linear Loop Is Used");
      vel_design_state = 0;
      return (SAL_SUCCESS);
   }

   switch (vel_design_state)
   {
      case 0: // VD
         PrintString("VD = ", 0);
       strcpy((char*)manu_spec_Vel_Design_command, "VD ");
         // find min shr - the shr of the largest number
       u16_max_index = 1;
       for (u16_index=2; u16_index<=7; u16_index++)
         if (abs(BGVAR(f_D_Design)[u16_index]) > abs(BGVAR(f_D_Design)[u16_max_index]) ) u16_max_index = u16_index;
         FloatToFix16Shift16(&s16_temp_fix[u16_max_index], &u16_max_index, abs(BGVAR(f_D_Design)[u16_max_index]));

       for (u16_index=1; u16_index<=7; u16_index++)
       {
         PrintSignedInteger((int)(BGVAR(f_D_Design)[u16_index] *(float)((1LL << (long)u16_max_index))));
         PrintChar(SPACE);
         strcat((char*)manu_spec_Vel_Design_command,SignedIntegerToAscii((BGVAR(f_D_Design)[u16_index] *(float)((1LL << (long)u16_max_index)))) );
       strcat((char*)manu_spec_Vel_Design_command, " ");
       }
         PrintSignedInteger(u16_max_index);
         PrintCrLf();
         strcat((char*)manu_spec_Vel_Design_command,SignedIntegerToAscii(u16_max_index));
       strcat((char*)manu_spec_Vel_Design_command, "\n ");
         vel_design_state++;
      break;

      case 1: // VH
         PrintString("VH = ", 0);
       strcat((char*)manu_spec_Vel_Design_command, "VH ");
         for (u16_index=0; u16_index<6; u16_index++)
         {
            FloatToFix16Shift16(&s16_temp_fix[u16_index], &u16_temp_shr[u16_index], BGVAR(f_H_Design)[u16_index]);
            PrintSignedInteger(s16_temp_fix[u16_index]);
            PrintChar(SPACE);
            PrintSignedInteger(u16_temp_shr[u16_index]);
            PrintChar(SPACE);

            strcat((char*)manu_spec_Vel_Design_command,SignedIntegerToAscii(s16_temp_fix[u16_index]));
         strcat((char*)manu_spec_Vel_Design_command, " ");
            strcat((char*)manu_spec_Vel_Design_command,SignedIntegerToAscii(s16_temp_fix[u16_index]));
         strcat((char*)manu_spec_Vel_Design_command, " ");
         }
         PrintCrLf();
       strcat((char*)manu_spec_Vel_Design_command, "\n");
         vel_design_state++;
      break;

      case 2: // VR
         PrintString("VR = ", 0);
       strcat((char*)manu_spec_Vel_Design_command, "VR ");
         for (u16_index=0; u16_index<5; u16_index++)
         {
            FloatToFix16Shift16(&s16_temp_fix[u16_index], &u16_temp_shr[u16_index], BGVAR(f_R_Design)[u16_index]);
            PrintSignedInteger(s16_temp_fix[u16_index]);
            PrintChar(SPACE);
            PrintSignedInteger(u16_temp_shr[u16_index]);
            PrintChar(SPACE);

            strcat((char*)manu_spec_Vel_Design_command,SignedIntegerToAscii(s16_temp_fix[u16_index]));
         strcat((char*)manu_spec_Vel_Design_command, " ");
            strcat((char*)manu_spec_Vel_Design_command,SignedIntegerToAscii(s16_temp_fix[u16_index]));
         strcat((char*)manu_spec_Vel_Design_command, " ");
         }
         PrintCrLf();
       strcat((char*)manu_spec_Vel_Design_command, "\n");
         vel_design_state = 0;
      break;
   }

      if (vel_design_state == 0) return (SAL_SUCCESS);
      else return (SAL_NOT_FINISHED);
}

// This will calculate the conversion factor from internal vel to velocity forced commutation
void UpdateSLScale(int drive)
{
   // AXIS_OFF;

   float f_temp = 0.0;
   REFERENCE_TO_DRIVE;

   f_temp = 65536.0/22750.0*BGVAR(s32_V_Lim_Design)/536870.912/8000.0*VAR(AX0_s16_Num_Of_Poles_Ratio);

   FloatToFix16Shift16(&VAR(AX0_s16_Sl_Forced_Delta_Factor_Fix), &VAR(AX0_u16_Sl_Forced_Delta_Factor_Shr), f_temp);
}



int SalNlVelLimCommand(long long lparam, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if (lparam > 10000LL) return (VALUE_TOO_HIGH);
   if (lparam < -10000LL) return (VALUE_TOO_LOW);

   VAR(AX0_s16_Nl_Vel_Lim) = (int)lparam;

   return (SAL_SUCCESS);
}

void UpdateEventDecelerations(drive)
{
   unsigned long long u64_local_decstop;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
   {// CDHD drive configuration,
    // update all event decelerations to decstop value to maintain the decstop rate in all events
      u64_local_decstop = BGVAR(u64_DecStop); // using local var to avoid calling the "BGVR" macro
      BGVAR(u64_Motor_Stop_Dec_Rate) = u64_local_decstop;
      BGVAR(u64_Ser_Comm_Time_Out_Dec_Rate) = u64_local_decstop;
      BGVAR(u64_Pos_Comand_Overflow_Dec_Rate) = u64_local_decstop;
      BGVAR(u64_N_SW_Limit_Dec_Rate) = u64_local_decstop;
      BGVAR(u64_P_SW_Limit_Dec_Rate) = u64_local_decstop;
      BGVAR(u64_P_HW_Limit_Dec_Rate) = u64_local_decstop;
      BGVAR(u64_N_HW_Limit_Dec_Rate) = u64_local_decstop;
   }
   else// It's a Schneider drive configuration, reset all decs to =S= default decelerations.
   {// for now init to random values in ACC/DEC range
      BGVAR(u64_Motor_Stop_Dec_Rate) = 0x83126E978D4FE; // 50ms deceleration
      BGVAR(u64_Ser_Comm_Time_Out_Dec_Rate) =  0x83126E978D4FE; // 50ms deceleration
      BGVAR(u64_Pos_Comand_Overflow_Dec_Rate) = 0xDA740DA740DA4; // 30ms deceleration
      BGVAR(u64_N_SW_Limit_Dec_Rate) = 0x83126E978D4FE; // 50ms deceleration
      BGVAR(u64_P_SW_Limit_Dec_Rate) = 0x83126E978D4FE; // 50ms deceleration
      BGVAR(u64_P_HW_Limit_Dec_Rate) = 0xDA740DA740DA4; // 30ms deceleration
      BGVAR(u64_N_HW_Limit_Dec_Rate) = 0xDA740DA740DA4; // 30ms deceleration
   }
   BGVAR(u64_Original_Dec_Stop_Rate) = BGVAR(u64_DecStop);// set valid value in any case.
}


//*****************************************************************************************
// Function Name: UpdateVelLimitPiLoopParams
// Description: This function changes all required PI-loop variables to be used by
//              the function "RunVelLimitPiLoop" upon a change of the VLIM parameter.
//              For further information check the header of the PI-Loop Run function.
//
// Author: APH
// Input:
// Return value:
// Algorithm:
// Revisions:
//*****************************************************************************************
void UpdateVelLimitPiLoopParams(int drive)
{
   long s32_temp; // temp variable
   float f_current_limit;

   // Proportional gain outputs motor continuous current at a deviation of 250[rpm] (s32_Motor_I_Cont is in [mA])
   float f_proportionalGain = (float)BGVAR(s32_Motor_I_Cont)/250; // aph77: Needs fine-tuning for several Drive/motor combinations

   // Convert the parameter VLIM from [Bits/125us] to rpm: vlim[rpm] =  vlim[internal] * 8000 * 60 / 2^32
   // Note that the variable s32_V_Lim_Design does not change in case that the user changes the unit since
   // this variable is always given in internal counts per sample-rate of the velocity loop
   float f_vlim_rpm = (float)BGVAR(s32_V_Lim_Design) * 1.1175871 / 10000.0;


   // Convert the proportional gain from [mA]/[rpm] into [CurrentUnitInternal/VelocityUnitInternal_InTheLoop].
   // The variable f_proportionalGain is given in the unit [mA/rpm], the formula to be used is:
   //    f_proportionalGain * ((26214 / DIPEAK[mA]) / (22750 / s32_vlim_rpm)) // with 26214/22750=1.1522637
   float temp = f_proportionalGain * 1.1522637 * f_vlim_rpm / (float)BGVAR(s32_Drive_I_Peak);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   FloatToFix16Shift16(&BGVAR(s16_Torque_Mode_Vel_Limit_Prop_Gain_Fix), &BGVAR(u16_Torque_Mode_Vel_Limit_Prop_Gain_Shift), (float)temp);

   u16_Integral_Time_Ms = 80; // aph77: Integrator time = 20[ms] (running at 250 us) --> Needs fine-tuning

   // Here calculate 1/integralTime in 1[ms] samples
   FloatToFix16Shift16(&BGVAR(s16_Torque_Mode_Vel_Limit_Integral_Time_Inverse_Fix), &BGVAR(u16_Torque_Mode_Vel_Limit_Integral_Time_Inverse_Shift), (float)1/(float)u16_Integral_Time_Ms);

   // aph77: Set the clamping value "s16_Torque_Mode_Vel_Limit_Max_Pos_Pi_Output" of the integrator to a reasonable
   // value in order to not let the integral part accumulate to a very large value and to make the PI-loop less responsive.
   // Set default value of the maximum output for the PI-loop
   s32_temp = 3 * BGVAR(s32_Motor_I_Cont_Internal);
   if(s32_temp > (long)26214)
   {
      // 3-time motor continuous exceeds the DIPEAK, therefore set DIPEAK to the maximum PI-loop output.
      BGVAR(s16_Torque_Mode_Vel_Limit_Max_Pos_Pi_Output) = 26214;
   }
   else
   {
      // Per default set the maximum limit of the PI loop to 3-times the motor continuous, which is the maximum for the Lexium.
      BGVAR(s16_Torque_Mode_Vel_Limit_Max_Pos_Pi_Output) = (int)s32_temp;
   }
   BGVAR(s16_Torque_Mode_Vel_Limit_Max_Neg_Pi_Output) = 0;    // Min. limit = 0, current limit is not supposed to drop below 0[A]!!!

   // AZ, Sep 30, 2015, continuation of CQ1299
   // Set current limit to avoid huge velocity overshoot in current mode, especially if the velocity limit is due to low VLIM.
   // Without this limit, the initial current limit is DIPEAK. Since this PI loop is relatively slow, the motor accelerates very
   // fast and exceeds the velocity limit. With velocity limit of VLIM=10rpm, the measurable velocity is 14.4 rpm, which is below
   // the actual velocity. This PI loop sees smaller error than actual error, and it reduces the current limit slowly, and the effect
   // is high velocity for a while before the velocity drops to the velocity limit.
   // The idea here is to set the current limit such that it allows acceleration to the velocity limit in two sample times of 250 us,
   // assuming no load (two is an arbitrary choise). Comment: if this current limit is too low to overcome the friction, the motor
   // will not move.
   // So:
   // Il[A] = J[Kg*m^2] * a[rad/s^2] / Kt[Nm/A].
   // Where:
   // Il is the current limit.
   // J is the motor inertia MJ.
   // Kt is the torque constant MKT.
   // a is the acceleration Vl/(2*250us), where Vl is the velocity limit, in "in the loop" velocity units.
   // Hence:
   // Il[A] = Vl/250us * J[Kg*m^2] / Kt[Nm/A].
   // Il[internal] = (Vl[in loop]/22750*VLIM[int]*8000*60/2^32*2*pi/60*4000/2) * (MJ/1,000,000) / (MKT/1,000) * (1,000*26,214/DIPEAK)
   //                                   +--------------------+         +--+       +----------+     +-------+     +-----------------+
   //                                         VLIM[rpm]                 Ts          J[Kg*m^2]       Kt[Nm/A]      conv A to internal
   //                 +--------------------------------------+
   //                                 Vl[rpm]
   //                 +----------------------------------------------+
   //                                    Vl[rad/s]
   //                 +---------------------------------------------------+
   //                                      a[rad/s^2]
   //
   // Il[internal] = Vl[in loop] * VLIM[int]*MJ/MKT/DIPEAK*0.02697068...
   //
   // Calculate here the factor that multiplies Vl, to be used in RT to calculate the current limit.

   f_current_limit = (float)BGVAR(s32_V_Lim_Design) * (float)BGVAR(s32_Motor_J) / (float)BGVAR(u32_Motor_Kt) / (float)BGVAR(s32_Drive_I_Peak) * 0.02697068;
   FloatToFix16Shift16(&BGVAR(s16_Torque_Mode_Vel_Limit_Current_Limit_Factor_Fix), &BGVAR(u16_Torque_Mode_Vel_Limit_Current_Limit_Factor_Shr), f_current_limit);
}

//**********************************************************
// Function Name: SalWriteDecStopViaPParam
// Description:
//          This function is called in case that the parameter
//          DECSTOP needs to be accessed via P-parameter.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteDecStopViaPParam(long long lparam,int drive)
{
   unsigned long long u64_temp;

   // Convert first from the unit "ms from 0[rpm] to 6000[rpm] into internal acc/dec
   // values used in the position loop (used in RT Tsp).
   u64_temp = convertMsToAccDecInternalUnitsForPos(drive, (unsigned int)lparam);

   // Now call the DECSTOP Sal-function
   return(SalDecStopCommand((long long)u64_temp, drive));
}
//**********************************************************
// Function Name: SalReadDecStopViaPParam
// Description:
//          This function is called in case that the parameter
//          DECSTOP needs to be read via P-parameter.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalReadDecStopViaPParam(long long* lparam,int drive)
{
   // Convert the internal C-Variable, that represents DECSTOP in the unit
   // "internal acc/dec used in RT position loop (tsp)" into "ms from
   // 0[rpm] to 6000[rpm]"
   *lparam = convertAccDecInternalUnitsForPosToMs(drive, BGVAR(u64_DecStop));
   return SAL_SUCCESS;
}
