#include "math.h"
#include "DSP2834x_Device.h"

#include "Err_Hndl.def"
#include "ModCntrl.def"
#include "SensorlessBG.def"
#include "PhaseFind.def"
#include "Ser_Comm.def"
#include "design.def"
#include "MotorParamsEst.def"

#include "SensorlessBG.var"
#include "PhaseFind.var"
#include "Extrn_Asm.var"
#include "Drive.var"
#include "Units.var"
#include "Motor.var"
#include "Ser_Comm.var"
#include "Velocity.var"
#include "FltCntrl.var"
#include "MotorSetup.var"
#include "MotorParamsEst.var"

#include "Prototypes.pro"

float f_P_Minus[5][5],f_Kalman_E_Vel,f_Q11,f_Q22,f_Q33,f_Q44,f_R11,f_R22,f_Kalman_Volts_Factor_Over_L,
      f_R_Mpy_InvL_Neg,f_Lambda_Mpy_InvL,f_Klmn_Internal_To_Amp,f_Klmn_Amp_To_Internal,f_Klmn_Lambda_Over_L,
      f_Klmn_R_Over_L,f_Sl_Klmn_Delta;
float f_hph_r_inv[3][3], f_p_correction[5][5], f_f[5][5], f_p_times_f_transpose[5][5], f_f_times_p[5][5], f_k_k[5][5];
int s16_i_beta = 0, s16_iu, s16_Kalman_I_Alpha,s16_Kalman_I_Beta;

void CalcRTInjectionAmpAndFreq(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   VAR(AX0_s16_Injection_Frequncy) = (int)((float)BGVAR(u16_Sl_Excitation_Freq)*65536.0/32000.0);
   VAR(AX0_s16_Pwm_Excitation) = (int)((float)VAR(AX0_s16_Pwm_Half_Period) * 54.0 / (float)BGVAR(s16_Vbus) * (float)BGVAR(u16_Sl_Excitation_Gain) / 1000.0);
}

void CalcTrackingFilterCoef(int drive)
{
   /*
   *  When the algorithm is executed at 125uS rate, use:
   *  AX0_s16_Swr2d_W_Sqr = 0.025735927 * u16_ResBw^2
   *  AX0_s16_Swr2d_Bw_Inv_W = 57041.1316 / u16_ResBw
   *  (AX0_s16_Swr2d_Bw_Inv_W includes extra 5 shl, that will be compensated in real-time)
   *  For now, we execute the algorithm at 125uS rate.
   *  0.025735927 = 0x696a >> 20
   *  114082.2632 = 0x6F6890D8 >> 15
   */
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   VAR(AX0_s16_Sl_Tracking_W_Sqr) = (int)(((float)BGVAR(s16_Sl_Hf_Bw))*((float)BGVAR(s16_Sl_Hf_Bw)) * 0.025735927);
   VAR(AX0_s16_Sl_Tracking_Bw_Inv_W) = (int)(57041.131591796875/((float)BGVAR(s16_Sl_Hf_Bw)));
}

void CalcSlBemfVariables(int drive)
{
/* Kb*w = Kpwm*U - Kr*i - Kldi*delta(i)
 *
 *  Kr*Iphase =s32_Sl_MR/1000*Iphase/0x6666*dipeak/1000*pwm_half_preiod/Vbus
 *  Kldi*delta(i) = s32_Sl_Mlmin/1000*delta(i)/0x6666*dipeak/1000*pwm_half_preiod/Vbus*Ts_sl
 */
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   FloatToFix16Shift16(&VAR(AX0_s16_Sl_Resistance_Fix),(unsigned int *)&VAR(AX0_s16_Sl_Resistance_Shr),
       (float)BGVAR(s32_Sl_MR) * (float)BGVAR(s32_Drive_I_Peak) * VAR(AX0_s16_Pwm_Half_Period) / (float)BGVAR(s16_Vbus) * 3.8147555e-11);
   FloatToFix16Shift16(&VAR(AX0_s16_Sl_Inductance_Fix),(unsigned int *)&VAR(AX0_s16_Sl_Inductance_Shr),
       (float)BGVAR(s32_Sl_Mlmin) * (float)BGVAR(s32_Drive_I_Peak) * VAR(AX0_s16_Pwm_Half_Period) / (float)BGVAR(s16_Vbus) * 4.76844434e-15);

   VAR(AX0_s16_Sl_Bemf_Lpf_Alpha) = (int)(32768.0*exp(-2*3.14*(float)BGVAR(u16_Sl_Bemf_Lpf)*0.000125));
   VAR(AX0_s16_Sl_Bemf_Lpf_Beta) = (int)(32768L-(long)VAR(AX0_s16_Sl_Bemf_Lpf_Alpha));
}

void CalcSaliancyGainOffset(int s16_factor, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   LVAR(AX0_s32_Sl_Offset_Sin) = LVAR(AX0_s32_Sl_Offset_Cos) = 0L;
   FloatToFix16Shift16(&VAR(AX0_s16_Sl_Saliency_Cos_Gain_Fix_Design),(unsigned int *)&VAR(AX0_s16_Sl_Saliency_Cos_Gain_Shr_Design),(float)s16_factor / 1000.0); // 0.016 is based on our experience till now
   FloatToFix16Shift16(&VAR(AX0_s16_Sl_Saliency_Sin_Gain_Fix_Design),(unsigned int *)&VAR(AX0_s16_Sl_Saliency_Sin_Gain_Shr_Design),(float)s16_factor / 1000.0);

   VAR(AX0_s16_Sensorless_Control_Bits) |= SL_COPY_SINCOS_GAINS_MASK;
}

void CalcRateLimiter(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   LVAR(AX0_s32_Sl_Icmd_Rate_Limiter) = (long)((float)BGVAR(s32_Sl_Rate_Limiter)/(float)BGVAR(s32_Drive_I_Peak)*26214.0*65536.0);
}

int SalSlExcitationGainCommand(long long param,int drive)
{
   BGVAR(u16_Sl_Excitation_Gain) = (unsigned int)param;
   CalcRTInjectionAmpAndFreq(drive);
   return (SAL_SUCCESS);
}

int SalSlExcitationFreqCommand(long long param,int drive)
{
   BGVAR(u16_Sl_Excitation_Freq) = (unsigned int)param;
   CalcRTInjectionAmpAndFreq(drive);
   return (SAL_SUCCESS);
}

int SalSlModeCommand(long long param,int drive)
{
   // AXIS_OFF;

   if (VAR(AX0_u16_Motor_Comm_Type) != BRUSHLESS_MOTOR) return (MOTOR_COMM_TYPE_INVALID);

   if (BGVAR(u8_Sl_Mode) == (unsigned char)param) return (SAL_SUCCESS);

   BGVAR(u8_Sl_Mode) = (unsigned char)param;
   SpeedPhaseAdvDesign(DRIVE_PARAM);
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;

   if (BGVAR(u8_Sl_Mode) == 2)
   {
      VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_HF_INJECTION_MASK|SL_EN_FORCED_COMM_MASK|SL_EN_BEMF_MASK|SL_KALMAN_MASK;
      BGVAR(s16_Sensorless_Handler_State) = SL_STATE_EXPERT_MODE;
   }
   return (SAL_SUCCESS);
}


int SalSlCommModeCommand(long long param,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (BGVAR(u8_Sl_Mode) != 2) return (VALUE_OUT_OF_RANGE);

   BGVAR(u8_Sl_Comm_Mode) = (unsigned char)param;

   if (BGVAR(u8_Sl_Comm_Mode) == 1)
      LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Vforced_Torque_Cmd);
   else
   {
      if (VAR(AX0_s16_Opmode) == 2)
         LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Serial_Crrnt_Cmnd);
      else
      {
         if ((BGVAR(u8_CompMode) == 5) || (BGVAR(u8_CompMode) == 6) || (BGVAR(u8_CompMode) == 7))
         {
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Nct_Icmd);
         }
         else LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Crrnt_Lp_Inp);
      }
   }

   switch (BGVAR(u8_Sl_Comm_Mode))
   {
      case 1: // forced
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Forced_Comm);
      break;

      case 2:// saliancy
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Saliancy_Comm_With_Phase_Adv);
      break;

      case 3:// bemf
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Bemf_Comm_Locked_With_Phase_Adv);
      break;

      case 4: // kalman
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Klmn_Int_Pos);
      break;
   }

   return (SAL_SUCCESS);
}

int SalSlVelModeCommand(long long param,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (BGVAR(u8_Sl_Mode) != 2) return (VALUE_OUT_OF_RANGE);
   BGVAR(u8_Sl_Vel_Mode) = (unsigned char)param;
   switch (BGVAR(u8_Sl_Vel_Mode))
   {  //VAR(var) *(&var+axis_offset)
      case 1: // Forced          (unsigned int)((unsigned long)&VAR(AX0_u32_SFB_Pos_Fdbk_Lo) & 0xFFFF);
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Sl_Forced_Delta) & 0xFFFF);
      break;

      case 2:// Saliancy
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&LVAR(AX0_s32_Sl_Tracking_Vel_Int)+1 & 0xFFFF);
      break;

      case 3:// Bemf
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Sl_Bemf_Spd) & 0xFFFF);
      break;

      case 4: // Kalman
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Kalman_E_Vel) & 0xFFFF);
      break;
   }
   return (SAL_SUCCESS);
}

int SalReadSlSpeedsCommand(int drive)
{
   long long s64_temp, s64_fix;
   int index, s16_shr, i;

   index    = UnitsConversionIndex(&s8_Units_Vel_Out_Loop, drive);
   s64_fix  = BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_user_fix;
   s16_shr  = BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_user_shr;

   for (i=0; i<4; i++)
   {
      s64_temp = MultS64ByFixS64ToS64((long)BGVAR(s32_Sl_Speeds_For_User)[i], s64_fix, s16_shr);
      PrintString(DecimalPoint32ToAscii((long)s64_temp), 0);
      PrintChar(SPACE);
   }

   PrintChar('['); PrintString((char *)s8_Units_Vel_Out_Loop, 0); PrintChar(']');
   PrintCrLf();
   return (SAL_SUCCESS);
}

int CalcInternalSlpeeds(int drive)
{
   long long s64_temp;
   int i = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Not using Unit_Conversion_Table as it is not updted yet in the CONFIG sequence
   for (i = 0; i<4; i++)
   {
      s64_temp = (long long)((float)BGVAR(s32_Sl_Speeds_For_User)[i] / (float)BGVAR(s32_V_Lim_Design) * 22750.0);
      if (s64_temp > 22750LL) return CONFIG_FAIL_SLSPEEDS;
      BGVAR(s16_Sl_Speeds)[i] = (int)s64_temp & 0xFFFF;
   }

   return SAL_SUCCESS;
}

int SalSlSpeedsCommand(int drive)
{
   long long s64_temp, s64_fix;
   int index_out, i, s16_shr;
   long s32_spd_out_of_loop[4];

   index_out= UnitsConversionIndex(&s8_Units_Vel_Out_Loop, drive);
   s64_fix  = BGVAR(Unit_Conversion_Table[index_out]).s64_unit_conversion_to_internal_fix;
   s16_shr  = BGVAR(Unit_Conversion_Table[index_out]).u16_unit_conversion_to_internal_shr;

   for (i=0; i<4; i++)
   {
      s64_temp = MultS64ByFixS64ToS64(s64_Execution_Parameter[i], s64_fix, s16_shr);
      s32_spd_out_of_loop[i] = (long)s64_temp & 0xffffffff;
   }

   if ( (s32_spd_out_of_loop[3] < s32_spd_out_of_loop[2]) ||
        (s32_spd_out_of_loop[2] < s32_spd_out_of_loop[1]) ||
        (s32_spd_out_of_loop[1] < s32_spd_out_of_loop[0])   ) return (VALUE_OUT_OF_RANGE);

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;

   for (i=0; i<4; i++)
      BGVAR(s32_Sl_Speeds_For_User)[i] = s32_spd_out_of_loop[i];

   return (SAL_SUCCESS);
}

int SalReadSlStatesCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   PrintSignedInteger(BGVAR(s16_Sl_States)[0]);
   PrintChar(SPACE);
   PrintSignedInteger(BGVAR(s16_Sl_States)[1]);
   PrintChar(SPACE);
   PrintSignedInteger(BGVAR(s16_Sl_States)[2]);
   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalSlStatesCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ( (s64_Execution_Parameter[0] < 1LL) ||
        (s64_Execution_Parameter[1] < 1LL) ||
        (s64_Execution_Parameter[2] < 1LL) ||
        (s64_Execution_Parameter[0] > 13LL) ||
        (s64_Execution_Parameter[1] > 13LL) ||
        (s64_Execution_Parameter[2] > 13LL)   ) return (VALUE_OUT_OF_RANGE);

   // states:1 forced   all on
   //        2 hf       stdby=bemf
   //        3 hf       stdby=kalman
   //        4 bemf     stdby=hf
   //        5 bemf     stdby=kalman
   //        6 kalman   stdby=bemf
   //        7 kalman   stdby=hf
   //        8 forced   stdby=bemf
   //        9 bemf     stdby=forced
   //        10 forced  stdby=hf
   //        11 hf      stdby=forced
   //        12 forced  stdby=kalman

   BGVAR(s16_Sl_States)[0] = (int)s64_Execution_Parameter[0];
   BGVAR(s16_Sl_States)[1] = (int)s64_Execution_Parameter[1];
   BGVAR(s16_Sl_States)[2] = (int)s64_Execution_Parameter[2];

   return (SAL_SUCCESS);
}


int SalSlHfBwCommand(long long param,int drive)
{
   BGVAR(s16_Sl_Hf_Bw) = (int)param;
   CalcTrackingFilterCoef(drive);
   return (SAL_SUCCESS);
}

int SalSaliancyPhaseOfstCommand(long long lparam,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   VAR(AX0_s16_Sl_Saliancy_Phase_Offset) = (int)(((long)lparam * 0x5B06) >> 7);
   return (SAL_SUCCESS);
}

int SalReadSaliancyPhaseOfstCommand(long long* param,int drive)
{
   // AXIS_OFF;
   long s32_temp;
   REFERENCE_TO_DRIVE;
   s32_temp = (((long)VAR(AX0_s16_Sl_Saliancy_Phase_Offset) * 0x5A00)+0x200000)>>22;
   if( s32_temp >= 360 ) s32_temp-=360;
   else if( s32_temp < 0 ) s32_temp+=360;
   *param = (long long)s32_temp;
   return (SAL_SUCCESS);
}

int SalEKFPhaseOfstCommand(long long lparam,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   VAR(AX0_s16_Sl_EKF_Phase_Offset) = (int)(((long)lparam * 0x5B06) >> 7);
   return (SAL_SUCCESS);
}

int SalReadEKFPhaseOfstCommand(long long* param,int drive)
{
   // AXIS_OFF;
   long s32_temp;
   REFERENCE_TO_DRIVE;
   s32_temp = (((long)VAR(AX0_s16_Sl_EKF_Phase_Offset) * 0x5A00)+0x200000)>>22;
   if( s32_temp >= 360 ) s32_temp-=360;
   else if( s32_temp < 0 ) s32_temp+=360;
   *param = (long long)s32_temp;
   return (SAL_SUCCESS);
}

int SalBemfPhaseOfstCommand(long long lparam,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   VAR(AX0_s16_Sl_Bemf_Phase_Offset) = (int)(((long)lparam * 0x5B06) >> 7);
   return (SAL_SUCCESS);
}

int SalReadBemfPhaseOfstCommand(long long* param,int drive)
{
   // AXIS_OFF;
   long s32_temp;
   REFERENCE_TO_DRIVE;
   s32_temp = (((long)VAR(AX0_s16_Sl_Bemf_Phase_Offset) * 0x5A00)+0x200000)>>22;
   if( s32_temp >= 360 ) s32_temp-=360;
   else if( s32_temp < 0 ) s32_temp+=360;
   *param = (long long)s32_temp;
   return (SAL_SUCCESS);
}

int SalSlRateLimiterCommand(long long lparam,int drive)
{
   BGVAR(s32_Sl_Rate_Limiter) = (long)lparam;
   CalcRateLimiter(drive);
   return (SAL_SUCCESS);
}

int SalSlMRCommand(long long lparam,int drive)
{
   BGVAR(s32_Sl_MR) = (long)lparam;
   CalcSlBemfVariables(drive);
   return (SAL_SUCCESS);
}

int SalSlMlCommand(long long lparam,int drive)
{
   BGVAR(s32_Sl_Mlmin) = (long)lparam;
   CalcSlBemfVariables(drive);
   return (SAL_SUCCESS);
}

int SalSlBemfLpfCommand(long long param,int drive)
{
   BGVAR(u16_Sl_Bemf_Lpf) = (unsigned int)param;
   CalcSlBemfVariables(drive);
   return (SAL_SUCCESS);
}

void SetSlPtrsAccordingToState(int drive)
{
   // AXIS_OFF;
   unsigned int u16_timer_capture_3125;
   REFERENCE_TO_DRIVE;
   if ((BGVAR(s16_Sensorless_Handler_State) == SL_STATE_ACTIVE_FORCED_STDBY_BEMF) ||
       (BGVAR(s16_Sensorless_Handler_State) == SL_STATE_ACTIVE_FORCED_STDBY_HF)   ||
       (BGVAR(s16_Sensorless_Handler_State) == SL_STATE_FORCED_ALL_ON)            ||
       (BGVAR(s16_Sensorless_Handler_State) == SL_STATE_ACTIVE_FORCED_STDBY_KLMN)   )
      LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Vforced_Torque_Cmd);
   else
   {
      if (VAR(AX0_s16_Opmode) == 2)
         LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Serial_Crrnt_Cmnd);
      else
      {
         if ((BGVAR(u8_CompMode) == 5) || (BGVAR(u8_CompMode) == 6) || (BGVAR(u8_CompMode) == 7))
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Nct_Icmd);
         else
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Crrnt_Lp_Inp);
      }
   }

   switch (BGVAR(s16_Sensorless_Handler_State))
   {
      case SL_STATE_FORCED_ALL_ON:
         WaitForNext32kHzTaskStart();
         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_HF_INJECTION_MASK|SL_EN_BEMF_MASK|SL_EN_FORCED_COMM_MASK|SL_KALMAN_MASK;
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Forced_Comm);
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Sl_Forced_Delta) & 0xFFFF);
      break;

      case SL_STATE_ACTIVE_HF_STDBY_BEMF:
         u16_timer_capture_3125 = Cntr_3125;
         while (u16_timer_capture_3125 == Cntr_3125){}

         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_HF_INJECTION_MASK|SL_STDBY_BEMF_MASK;

         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Saliancy_Comm_With_Phase_Adv);
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&LVAR(AX0_s32_Sl_Tracking_Vel_Int)+1 & 0xFFFF);
      break;

      case SL_STATE_ACTIVE_HF_STDBY_FORCED:
         u16_timer_capture_3125 = Cntr_3125;
         while (u16_timer_capture_3125 == Cntr_3125){}

         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_HF_INJECTION_MASK|SL_EN_FORCED_COMM_MASK;

         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Saliancy_Comm_With_Phase_Adv);
//         VAR(AX0_s16_Sl_Forced_Delta) = VAR(AX0_s16_Sl_Saliancy_Comm_With_Phase_Adv);
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&LVAR(AX0_s32_Sl_Tracking_Vel_Int)+1 & 0xFFFF);
      break;

      case SL_STATE_ACTIVE_HF_STDBY_KLMN:
         u16_timer_capture_3125 = Cntr_3125;
         while (u16_timer_capture_3125 == Cntr_3125){}

         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_HF_INJECTION_MASK|SL_STDBY_KALMAN_MASK;

         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Saliancy_Comm_With_Phase_Adv);
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&LVAR(AX0_s32_Sl_Tracking_Vel_Int)+1 & 0xFFFF);
      break;

      case SL_STATE_ACTIVE_BEMF_STDBY_HF:
         u16_timer_capture_3125 = Cntr_3125;
         while (u16_timer_capture_3125 == Cntr_3125){}

         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_BEMF_MASK|SL_STDBY_HF_MASK|SL_EN_HF_INJECTION_MASK|SL_HF_TO_BEMF_TRANSITION_MASK;

         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Bemf_Comm_Locked_With_Phase_Adv);
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Sl_Bemf_Spd) & 0xFFFF);
      break;

      case SL_STATE_ACTIVE_BEMF_HF_OFF:
         u16_timer_capture_3125 = Cntr_3125;
         while (u16_timer_capture_3125 == Cntr_3125){}

         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_BEMF_MASK;

         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Bemf_Comm_Locked_With_Phase_Adv);
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Sl_Bemf_Spd) & 0xFFFF);
      break;

      case SL_STATE_ACTIVE_KLMN_STDBY_BEMF:
         WaitForNext32kHzTaskStart();
         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_KALMAN_MASK|SL_STDBY_BEMF_MASK;
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Kalman_E_Vel) & 0xFFFF);
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Klmn_Int_Pos);
      break;

      case SL_STATE_ACTIVE_KLMN_STDBY_HF:
         WaitForNext32kHzTaskStart();
         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_KALMAN_MASK|SL_STDBY_HF_MASK|SL_EN_HF_INJECTION_MASK;
      break;

      case SL_STATE_ACTIVE_FORCED_STDBY_BEMF:
         WaitForNext32kHzTaskStart();
         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_STDBY_BEMF_MASK|SL_EN_BEMF_MASK|SL_EN_FORCED_COMM_MASK;
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Forced_Comm);
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Sl_Forced_Delta) & 0xFFFF);
      break;

      case SL_STATE_ACTIVE_FORCED_STDBY_HF:
         WaitForNext32kHzTaskStart();
         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_FORCED_COMM_MASK|SL_STDBY_HF_MASK|SL_EN_HF_INJECTION_MASK;
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Forced_Comm);
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Sl_Forced_Delta) & 0xFFFF);
      break;

      case SL_STATE_ACTIVE_BEMF_STDBY_FORCED:
         u16_timer_capture_3125 = Cntr_3125;
         while (u16_timer_capture_3125 == Cntr_3125){}

         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_BEMF_MASK|SL_EN_FORCED_COMM_MASK;

         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Bemf_Comm_Locked_With_Phase_Adv);
         VAR(AX0_s16_Sl_Forced_Comm) = VAR(AX0_s16_Sl_Bemf_Comm_Locked_With_Phase_Adv);
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Sl_Bemf_Spd) & 0xFFFF);
      break;

      case SL_STATE_ACTIVE_FORCED_STDBY_KLMN:
         WaitForNext32kHzTaskStart();
         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_STDBY_KALMAN_MASK|SL_EN_FORCED_COMM_MASK;
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Forced_Comm);
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Sl_Forced_Delta) & 0xFFFF);
      break;

      case SL_STATE_ACTIVE_KALMAN_STDBY_FORCED:
         WaitForNext32kHzTaskStart();
         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_ALL_STATES_MASK;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_KALMAN_MASK|SL_EN_FORCED_COMM_MASK;
         VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Kalman_E_Vel) & 0xFFFF);
         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Klmn_Int_Pos);
      break;

      default:
      break;
   }
}

void SensrolessHandler(int drive)
{
   // AXIS_OFF;
   int s16_ret_val, s16_temp, s16_v_act, return_val;
   long long s64_temp;
   static int u16_first_time = 1;

   SensorlessSetupHandler(drive);

   if (BGVAR(u8_Sl_Mode) == 0)
   {
      BGVAR(s16_Sensorless_Handler_State) = SL_STATE_IDLE;
      return;
   }

   switch (BGVAR(s16_Sensorless_Handler_State))
   {
      case SL_STATE_IDLE:
         VAR(AX0_s16_Sl_Forced_Delta) = 0;
         if (BGVAR(u8_Sl_Mode) == 1)
            VAR(AX0_s16_Sl_Vel_Src_Ptr) = (int)((long)&VAR(AX0_s16_Sl_Forced_Delta) & 0xFFFF);
      break;

      /* old motor parameters estimation usage */

     /*
      case SL_STATE_ENABLE_REQUEST:
         // Check if ESTMOTORPARAM should be activated
         if ((BGVAR(u16_Sl_Est_Motor_Param_Mode)==1) || ((BGVAR(u16_Sl_Est_Motor_Param_Mode)==2) && u16_first_time))
         {
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_SETUP;
            BGVAR(s16_Sensorless_Handler_State) = SL_WAIT_FOR_ESTMOTORPARAM;
         }
         else
            BGVAR(s16_Sensorless_Handler_State) = SL_STATE_ENABLE_REQUEST_AFTER_ESTMOTORPARAM;
      break;

      case SL_WAIT_FOR_ESTMOTORPARAM:
         if (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_WAIT_FOR_EN) EnableCommand(drive);

         if ((BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_FAULT) ||
             ((BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_DONE) &&
             ((BGVAR(s32_Param_Est_ML) > 1000000L) || (BGVAR(s32_Param_Est_ML) < 1L) ||
             (BGVAR(s32_Param_Est_MR) > 50000L)   || (BGVAR(s32_Param_Est_MR) < 0L))))
         {
            BGVAR(s16_Sensorless_Handler_State) = SL_FAILED_ESTMOTORPARAM_FAILED;
         }
         else if (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_DONE)
         {
            // If ESTMOTORPARAM succeeded set ML, MR, SLML, SLMR accordingly
            SalMlminCommand((long long)BGVAR(s32_Param_Est_ML),drive);
            SalSlMlCommand((long long)BGVAR(s32_Param_Est_ML),drive);
            SalMresCommand((long long)BGVAR(s32_Param_Est_MR),drive);
            SalSlMRCommand((long long)BGVAR(s32_Param_Est_MR),drive);
            BGVAR(s16_Sensorless_Handler_State) = SL_STATE_CONFIG;
         }
      break;
*/
      case SL_STATE_ENABLE_REQUEST:
      // Check if ESTMOTORPARAM should be activated
         if ((BGVAR(u16_Sl_Est_Motor_Param_Mode)==1) || ((BGVAR(u16_Sl_Est_Motor_Param_Mode)==2) && u16_first_time))
         {
            BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_INIT;
            BGVAR(s16_Sensorless_Handler_State) = SL_WAIT_FOR_ESTMOTORPARAM;
         }
         else
            BGVAR(s16_Sensorless_Handler_State) = SL_STATE_ENABLE_REQUEST_AFTER_ESTMOTORPARAM;

      break;

      case SL_WAIT_FOR_ESTMOTORPARAM:
         if (BGVAR(s16_Motor_Params_Est_State) == MOTOR_PARAMS_EST_STATE_WAIT_FOR_ENABLE) EnableCommand(drive);

         if ((BGVAR(s16_Motor_Params_Est_State) < 0 ) || //motor paremeters estimation fault
             ((BGVAR(s16_Motor_Params_Est_State) == MOTOR_PARAMS_EST_STATE_DONE) &&
             ((BGVAR(s32_Motor_Params_Est_Ml) > 1000000L) || (BGVAR(s32_Motor_Params_Est_Ml) < 1L) ||
             (BGVAR(s32_Motor_Params_Est_Mr) > 50000L)   || (BGVAR(s32_Motor_Params_Est_Mr) < 0L))))
         {
            BGVAR(s16_Sensorless_Handler_State) = SL_FAILED_ESTMOTORPARAM_FAILED;
         }
         else if (BGVAR(s16_Motor_Params_Est_State) == MOTOR_PARAMS_EST_STATE_DONE)
         {
            // If ESTMOTORPARAM succeeded set ML, MR, SLML, SLMR accordingly
            SalMlminCommand((long long)BGVAR(s32_Motor_Params_Est_Ml),drive);
            SalSlMlCommand((long long)BGVAR(s32_Motor_Params_Est_Ml),drive);
            SalMresCommand((long long)BGVAR(s32_Motor_Params_Est_Mr),drive);
            SalSlMRCommand((long long)BGVAR(s32_Motor_Params_Est_Mr),drive);
            BGVAR(s16_Sensorless_Handler_State) = SL_STATE_CONFIG;
         }
      break;

      case SL_STATE_CONFIG:
         return_val = SalConfigCommand(drive);
         if (return_val == SAL_NOT_FINISHED) break;
         if (return_val != SAL_SUCCESS)   BGVAR(s16_Sensorless_Handler_State) = SL_FAILED_ESTMOTORPARAM_FAILED;
         else
         {
            if (BGVAR(u16_Sl_Est_Motor_Param_Mode)==2) u16_first_time = 0;
            BGVAR(s16_Sensorless_Handler_State) = SL_STATE_ENABLE_REQUEST_AFTER_ESTMOTORPARAM;
         }
      break;

      case SL_STATE_ENABLE_REQUEST_AFTER_ESTMOTORPARAM:
         // If HF is not used
         if ((BGVAR(u8_Sl_Mode) == 1) &&
             (BGVAR(s16_Sl_States)[0]!=2) && (BGVAR(s16_Sl_States)[1]!=2) && (BGVAR(s16_Sl_States)[2]!=2) &&
             (BGVAR(s16_Sl_States)[0]!=11) && (BGVAR(s16_Sl_States)[1]!=11) && (BGVAR(s16_Sl_States)[2]!=11))
         {
            EnableCommand(drive);
            EnableDriveHW(drive);
            BGVAR(s16_Sensorless_Handler_State) = SL_STATE_WAIT_PHASE_FIND_DONE;
         }
         else
         {
            if (ProcedureRunning(DRIVE_PARAM) != PROC_NONE)
            {
               BGVAR(s16_Sensorless_Handler_State) = SL_FAILED_PROC_RUNNING;
               return;
            }

            // Set up phase find process
            BGVAR(u8_PhaseFind_State) = PHASE_FIND_IDLE;
            s16_ret_val = PhaseFindCommand(drive);
            if (s16_ret_val != SAL_SUCCESS)
            {
               BGVAR(s16_Sensorless_Handler_State) = SL_FAILED_PHASE_FIND_COMMAND;
               return;
            }
            BGVAR(s16_Sensorless_Handler_State) = SL_STATE_EN_DRIVE;
         }
      break;

      case SL_STATE_EN_DRIVE:
         if (BGVAR(u8_PhaseFind_State) != PHASE_FIND_WAIT_FOR_EN) return;
         EnableCommand(drive);
         EnableDriveHW(drive);
         BGVAR(s16_Sensorless_Handler_State) = SL_STATE_WAIT_PHASE_FIND_DONE;
      break;

      case SL_STATE_WAIT_PHASE_FIND_DONE:
         PhaseFindStatusCommand(&s64_temp,drive); // 0 -idle. 1 - in proccess. 2-done 3 -fault
         // If HF is not used skip the next check
         if ((BGVAR(u8_Sl_Mode) == 1) &&
             (BGVAR(s16_Sl_States)[0]!=2) && (BGVAR(s16_Sl_States)[1]!=2) && (BGVAR(s16_Sl_States)[2]!=2) &&
             (BGVAR(s16_Sl_States)[0]!=11) && (BGVAR(s16_Sl_States)[1]!=11) && (BGVAR(s16_Sl_States)[2]!=11))
            s64_temp = 2LL;
         if (s64_temp == 3LL)
         {
            BGVAR(s16_Sensorless_Handler_State) = SL_FAILED_PHASE_FIND_PROC_FAILED_3;
            return;
         }
         else if (s64_temp == 0LL)
         {
            BGVAR(s16_Sensorless_Handler_State) = SL_FAILED_PHASE_FIND_PROC_FAILED_0;
            return;
         }
         else if (s64_temp == 1LL) return;

         // init injection decoder variable
         WaitForNext32kHzTaskStart();
         LVAR(AX0_s32_Sl_Tracking_Vel_Int) = 0;

         LVAR(AX0_s32_Sl_Tracking_Pos_Int) = ((long)(VAR(AX0_u16_PhaseFind_EPos) - 0x2AAA)) << 16L;

         // start injection
         BGVAR(s16_Sensorless_Handler_State) = SL_STATE_HF_INJECTION_INIT;

         if (BGVAR(u8_Sl_Mode) == 1)
         {
            if ((BGVAR(s16_Sl_States)[0]!=2) && (BGVAR(s16_Sl_States)[1]!=2) && (BGVAR(s16_Sl_States)[2]!=2) &&
                (BGVAR(s16_Sl_States)[0]!=11) && (BGVAR(s16_Sl_States)[1]!=11) && (BGVAR(s16_Sl_States)[2]!=11))
            {
//               VAR(AX0_s16_Sensorless_Control_Bits) |= SL_ICMD_RATE_LIMITER_EN_MASK;
               BGVAR(s16_Sensorless_Handler_State) = BGVAR(s16_Sl_States)[0]+SL_STATE_FORCED_ALL_ON-1;
               SetSlPtrsAccordingToState(drive);
            }
            else
            {
               SalSlVelModeCommand(2LL,drive);
               SalSlCommModeCommand(2LL,drive);
               VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_HF_INJECTION_MASK | SL_ICMD_RATE_LIMITER_EN_MASK;
            }
         }
         else VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_HF_INJECTION_MASK | SL_ICMD_RATE_LIMITER_EN_MASK;

//         SalForcedTorqueCommand((long long)BGVAR(s32_Forced_Torque_Cmd),drive); - Return when SL implementation is back

         BGVAR(s32_Sl_Timer) = Cntr_1mS;

//         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Sl_Saliancy_Comm_With_Phase_Adv);
      break;

      case SL_STATE_HF_INJECTION_INIT:
         if(!(PassedTimeMS(100L,(long)BGVAR(s32_Sl_Timer)))) return;

         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_HF_INIT_MASK;

         BGVAR(s32_Sl_Timer) = Cntr_1mS;
         BGVAR(s16_Sensorless_Handler_State) = SL_STATE_HF_INJECTION_INIT_VERIFY;
      break;

      case SL_STATE_HF_INJECTION_INIT_VERIFY:
         if(!(PassedTimeMS(100L,(long)BGVAR(s32_Sl_Timer)))) return;
         //s16_temp = VAR(AX0_s16_Sl_Saliancy_Comm_With_Phase_Adv) - VAR(AX0_s16_Sl_Saliancy_Phase_Offset) - VAR(AX0_u16_PhaseFind_EPos);
         s16_temp = VAR(AX0_s16_HF_Commutation_Variable) - VAR(AX0_u16_PhaseFind_EPos) + 0x2AAA;
         if (s16_temp < 0) s16_temp=-s16_temp;
         if ( (s16_temp > 16384) && (BGVAR(u8_Sl_Mode) != 2) )
         {
            BGVAR(s16_Sensorless_Handler_State) = SL_FAILED_HF_INIT_PHASE_FAILED;
            return;
         }

         if (BGVAR(u8_Sl_Mode) == 2)
         {
            VAR(AX0_s16_Sensorless_Control_Bits) |= SL_EN_HF_INJECTION_MASK|SL_EN_BEMF_MASK|SL_EN_FORCED_COMM_MASK;
            BGVAR(s16_Sensorless_Handler_State) = SL_STATE_EXPERT_MODE;
         }
         else
         {  // set state according to s16_Sl_States
            BGVAR(s16_Sensorless_Handler_State) = BGVAR(s16_Sl_States)[0]+SL_STATE_FORCED_ALL_ON-1;
            SetSlPtrsAccordingToState(drive);

            // Initiate HF sin/cos calibration
            if (BGVAR(s16_SlSetup_State) == SL_SETUP_IDLE) BGVAR(s16_SlSetup_State) = SL_SETUP_START_HF_CALIBRATION;
         }
      break;

      case SL_STATE_FORCED_ALL_ON:
      case SL_STATE_ACTIVE_HF_STDBY_BEMF:
      case SL_STATE_ACTIVE_HF_STDBY_KLMN:
      case SL_STATE_ACTIVE_BEMF_STDBY_HF:
      case SL_STATE_ACTIVE_BEMF_HF_OFF:
      case SL_STATE_ACTIVE_KLMN_STDBY_BEMF:
      case SL_STATE_ACTIVE_KLMN_STDBY_HF:
      case SL_STATE_ACTIVE_FORCED_STDBY_BEMF:
      case SL_STATE_ACTIVE_BEMF_STDBY_FORCED:
      case SL_STATE_ACTIVE_FORCED_STDBY_HF:
      case SL_STATE_ACTIVE_HF_STDBY_FORCED:
      case SL_STATE_ACTIVE_FORCED_STDBY_KLMN:
      case SL_STATE_ACTIVE_KALMAN_STDBY_FORCED:
         if ((!Enabled(drive)) && (BGVAR(s16_Sensorless_Handler_State) > 0))
         {
            BGVAR(s16_Sensorless_Handler_State) = SL_STATE_RESET_MACHINE;
            return;
         }

         s16_v_act = VAR(AX0_s16_Vel_Var_Ref_0); //VAR(AX0_s16_Vel_Var_Fb_0);
         if (s16_v_act < 0) s16_v_act = -s16_v_act;

         // Start Kalman when speed is high enough
         if (BGVAR(s16_Sensorless_Handler_State) == SL_STATE_ACTIVE_FORCED_STDBY_KLMN)
         {
            if ((s16_v_act >= BGVAR(s16_Sl_Speeds)[0]) && ((AX0_s16_Sensorless_Control_Bits & SL_KALMANING_STARTED_MASK)==0))
               AX0_s16_Sensorless_Control_Bits |= SL_START_KALMANING_MASK;
         }

         s16_temp =  BGVAR(s16_Sensorless_Handler_State)-SL_STATE_FORCED_ALL_ON+1;

         if ( (s16_v_act > BGVAR(s16_Sl_Speeds)[3]) &&
                    (s16_temp == BGVAR(s16_Sl_States)[1])  )
         {
            BGVAR(s16_Sensorless_Handler_State) = BGVAR(s16_Sl_States)[2]+SL_STATE_FORCED_ALL_ON-1;
            SetSlPtrsAccordingToState(drive);
         }
         else if ( (s16_v_act > BGVAR(s16_Sl_Speeds)[1]) &&
              (s16_temp == BGVAR(s16_Sl_States)[0])    )
         {
            BGVAR(s16_Sensorless_Handler_State) = BGVAR(s16_Sl_States)[1]+SL_STATE_FORCED_ALL_ON-1;
            SetSlPtrsAccordingToState(drive);
         }
         else if ( (s16_v_act < BGVAR(s16_Sl_Speeds)[0]) &&
                   (s16_temp == BGVAR(s16_Sl_States)[1])   )
         {
            BGVAR(s16_Sensorless_Handler_State) = BGVAR(s16_Sl_States)[0]+SL_STATE_FORCED_ALL_ON-1;
            SetSlPtrsAccordingToState(drive);
         }
         else if ( (s16_v_act < BGVAR(s16_Sl_Speeds)[2]) &&
                    (s16_temp == BGVAR(s16_Sl_States)[2])  )
         {
            BGVAR(s16_Sensorless_Handler_State) = BGVAR(s16_Sl_States)[1]+SL_STATE_FORCED_ALL_ON-1;
            SetSlPtrsAccordingToState(drive);
         }
         break;

      case SL_STATE_EXPERT_MODE:
         if ((!Enabled(drive)) && (BGVAR(s16_Sensorless_Handler_State) > 0))
         {
            BGVAR(s16_Sensorless_Handler_State) = SL_STATE_RESET_MACHINE;
            return;
         }
      break;

      case SL_STATE_RESET_MACHINE:
         VAR(AX0_s16_Sensorless_Control_Bits) &= ~(SL_ALL_STATES_MASK|SL_ICMD_RATE_LIMITER_EN_MASK);
         BGVAR(s16_Sensorless_Handler_State) = SL_STATE_IDLE;
         break;

      case SL_FAILED_PROC_RUNNING:
      case SL_FAILED_PHASE_FIND_COMMAND:
      case SL_FAILED_PHASE_FIND_PROC_FAILED_0:
      case SL_FAILED_PHASE_FIND_PROC_FAILED_3:
      case SL_FAILED_HF_INIT_PHASE_FAILED:
      case SL_FAILED_ESTMOTORPARAM_FAILED:
      break;
   }
}

int SalReadEKFPCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   PrintString(DecimalPoint32ToAscii(BGVAR(s32_P_Minus_Init)[0]), 0);
   PrintChar(SPACE);
   PrintString(DecimalPoint32ToAscii(BGVAR(s32_P_Minus_Init)[1]), 0);
   PrintChar(SPACE);
   PrintString(DecimalPoint32ToAscii(BGVAR(s32_P_Minus_Init)[2]), 0);
   PrintChar(SPACE);
   PrintString(DecimalPoint32ToAscii(BGVAR(s32_P_Minus_Init)[3]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalEKFPCommand(int drive)
{
   if ((s64_Execution_Parameter[0] < 0LL) ||
       (s64_Execution_Parameter[1] < 0LL) ||
       (s64_Execution_Parameter[2] < 0LL) ||
       (s64_Execution_Parameter[3] < 0LL)   ) return (VALUE_TOO_LOW);

   if ((s64_Execution_Parameter[0] > 2147483647LL) ||
       (s64_Execution_Parameter[1] > 2147483647LL) ||
       (s64_Execution_Parameter[2] > 2147483647LL) ||
       (s64_Execution_Parameter[3] > 2147483647LL)   ) return (VALUE_TOO_HIGH);

   BGVAR(s32_P_Minus_Init)[0] = (long)s64_Execution_Parameter[0];
   BGVAR(s32_P_Minus_Init)[1] = (long)s64_Execution_Parameter[1];
   BGVAR(s32_P_Minus_Init)[2] = (long)s64_Execution_Parameter[2];
   BGVAR(s32_P_Minus_Init)[3] = (long)s64_Execution_Parameter[3];

   KalmanFilterDesign(drive);
   return (SAL_SUCCESS);
}

int SalReadEKFQCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   PrintString(DecimalPoint32ToAscii(BGVAR(s32_Sl_EKF_Q)[0]), 0);
   PrintChar(SPACE);
   PrintString(DecimalPoint32ToAscii(BGVAR(s32_Sl_EKF_Q)[1]), 0);
   PrintChar(SPACE);
   PrintString(DecimalPoint32ToAscii(BGVAR(s32_Sl_EKF_Q)[2]), 0);
   PrintChar(SPACE);
   PrintString(DecimalPoint32ToAscii(BGVAR(s32_Sl_EKF_Q)[3]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalEKFQCommand(int drive)
{
   if ((s64_Execution_Parameter[0] < 0LL) ||
       (s64_Execution_Parameter[1] < 0LL) ||
       (s64_Execution_Parameter[2] < 0LL) ||
       (s64_Execution_Parameter[3] < 0LL)   ) return (VALUE_TOO_LOW);

   if ((s64_Execution_Parameter[0] > 2147483647LL) ||
       (s64_Execution_Parameter[1] > 2147483647LL) ||
       (s64_Execution_Parameter[2] > 2147483647LL) ||
       (s64_Execution_Parameter[3] > 2147483647LL)   ) return (VALUE_TOO_HIGH);

   BGVAR(s32_Sl_EKF_Q)[0] = (long)s64_Execution_Parameter[0];
   BGVAR(s32_Sl_EKF_Q)[1] = (long)s64_Execution_Parameter[1];
   BGVAR(s32_Sl_EKF_Q)[2] = (long)s64_Execution_Parameter[2];
   BGVAR(s32_Sl_EKF_Q)[3] = (long)s64_Execution_Parameter[3];

   KalmanFilterDesign(drive);
   return (SAL_SUCCESS);
}

int SalReadEKFRCommand  (int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   PrintString(DecimalPoint32ToAscii(BGVAR(s32_Sl_EKF_R)[0]), 0);
   PrintChar(SPACE);
   PrintString(DecimalPoint32ToAscii(BGVAR(s32_Sl_EKF_R)[1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalEKFRCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 0LL) ||
       (s64_Execution_Parameter[1] < 0LL)   ) return (VALUE_TOO_LOW);

   if ((s64_Execution_Parameter[0] > 2147483647LL) ||
       (s64_Execution_Parameter[1] > 2147483647LL)   ) return (VALUE_TOO_HIGH);

   BGVAR(s32_Sl_EKF_R)[0] = (long)s64_Execution_Parameter[0];
   BGVAR(s32_Sl_EKF_R)[1] = (long)s64_Execution_Parameter[1];

   KalmanFilterDesign(drive);
   return (SAL_SUCCESS);
}

int SalSlEKFMlCommand(long long param,int drive)
{
   BGVAR(s32_Sl_EKF_Mlmin) = (long)param;
   KalmanFilterDesign(drive);
   return (SAL_SUCCESS);
}

int SalSlEKFMotorKtCommand(long long param,int drive)
{
   BGVAR(u32_Sl_EKF_Motor_Kt) = (unsigned long)param;
   KalmanFilterDesign(drive);
   return (SAL_SUCCESS);
}

int SalSlEKFMresCommand(long long param,int drive)
{
   BGVAR(s32_Sl_EKF_Motor_Res) = (long)param;
   KalmanFilterDesign(drive);
   return (SAL_SUCCESS);
}

void KalmanFilterDesign(int drive)
{
   // AXIS_OFF;
   float f_r_kalman = (float)BGVAR(s32_Sl_EKF_Motor_Res)/1000.0;
   float f_inv_l_kalman = 1000000.0/(float)BGVAR(s32_Sl_EKF_Mlmin);
   float f_tkalman_mpy_inv_l = TS_KALMAN*f_inv_l_kalman;
   // Mkt/1000/(3*Mpoles/2)
   float f_lambda_kalman = (float)BGVAR(u32_Sl_EKF_Motor_Kt)/(float)VAR(AX0_s16_Num_Of_Poles_Ratio)/750.0;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   f_R11 = BGVAR(s32_Sl_EKF_R)[0]/1000.0;
   f_R22 = BGVAR(s32_Sl_EKF_R)[1]/1000.0;
   f_Q11 = BGVAR(s32_Sl_EKF_Q)[0]/1000.0;
   f_Q22 = BGVAR(s32_Sl_EKF_Q)[1]/1000.0;
   f_Q33 = BGVAR(s32_Sl_EKF_Q)[2]/1000.0;
   f_Q44 = BGVAR(s32_Sl_EKF_Q)[3]/1000.0;

   f_R_Mpy_InvL_Neg = -f_r_kalman*f_inv_l_kalman;
   f_Lambda_Mpy_InvL = f_lambda_kalman*f_inv_l_kalman;

   f_Kalman_Volts_Factor_Over_L = (float)BGVAR(s16_Vbus)/(float)AX0_s16_Pwm_Half_Period * f_tkalman_mpy_inv_l;

   f_Klmn_Internal_To_Amp = (float)BGVAR(s32_Drive_I_Peak)/26214000.0;
   f_Klmn_Amp_To_Internal = 1.0/f_Klmn_Internal_To_Amp;
   f_Klmn_Lambda_Over_L = f_lambda_kalman * f_tkalman_mpy_inv_l;
   f_Klmn_R_Over_L = f_r_kalman * f_tkalman_mpy_inv_l * f_Klmn_Internal_To_Amp;
}

/*#pragma CODE_SECTION(KalmanFilterRT, "ramfunc_2");
void KalmanFilterRT(void)
{
   float f_temp1;
   int i, j, s16_iv;//,s16_i_beta;

   if ((AX0_s16_Sensorless_Control_Bits & (SL_KALMAN_MASK|SL_STDBY_KALMAN_MASK)) == 0)
   {
      AX0_s16_Kalman_E_Vel = 0;
      AX0_s16_Sensorless_Control_Bits &= ~SL_KALMANING_STARTED_MASK;
      return;
   }

   s16_iu = AX0_s16_Crrnt_Srvo_Cmnd_U_Act_0;
   s16_iv = AX0_s16_Crrnt_Srvo_Cmnd_V_Act_0;
   // calc i_alpha i_beta from iu iv (also convert from internal to amp) ialpha = iu
   s16_i_beta  = (int)((18918L *(((long)s16_iv<<1L) + (long)s16_iu))>>15L);

   if ((AX0_s16_Sensorless_Control_Bits & SL_START_KALMANING_MASK) != 0)
   {
      AX0_s16_Sensorless_Control_Bits &= ~SL_START_KALMANING_MASK;
      AX0_s16_Sensorless_Control_Bits |= SL_KALMANING_STARTED_MASK;

      for (i=1; i<=4; ++i)
         for (j=1; j<=4; ++j) f_P_Minus[i][j] = 0.0;

      f_P_Minus[1][1] = s32_P_Minus_Init[0][0]/1000.0;
      f_P_Minus[2][2] = s32_P_Minus_Init[0][1]/1000.0;
      f_P_Minus[3][3] = s32_P_Minus_Init[0][2]/1000.0;
      f_P_Minus[4][4] = s32_P_Minus_Init[0][3]/1000.0;

      f_temp1 = (float)s32_V_Lim_Design[0] * 0.00011176;//vlim rpm
      f_Kalman_E_Vel = f_temp1*2.3015330e-6*(float)AX0_s16_Vel_Var_Fb_0*(float)(VAR(AX0_s16_Num_Of_Poles_Ratio)<<1);
      s16_Kalman_I_Beta = s16_i_beta;
      s16_Kalman_I_Alpha = s16_iu;
      AX0_s16_Kalman_E_Pos = AX0_s16_Elect_Pos_With_Phase_Adv;// + 32768;
      AX0_s16_Sl_Klmn_Delta = 0;
      f_Sl_Klmn_Delta = 0.0;
   }
}

#pragma CODE_SECTION(KalmanFilterRT_1, "ramfunc_2");
void KalmanFilterRT_1(void)
{
   float f_sin_epos,f_cos_epos;
   int j;

   KalmanFilterRT();

   if ((AX0_s16_Sensorless_Control_Bits & SL_KALMANING_STARTED_MASK) == 0) return;

   f_sin_epos = sin(DRIVE_ANGLE_TO_RAD * (float)AX0_s16_Kalman_E_Pos);
   f_cos_epos = cos(DRIVE_ANGLE_TO_RAD * (float)AX0_s16_Kalman_E_Pos);

   // Note v_alpha v_beta from current loop outputs(also convert from internal to volts)
   s16_Kalman_I_Alpha += f_Klmn_Amp_To_Internal*(f_sin_epos*f_Kalman_E_Vel*f_Klmn_Lambda_Over_L - (float)s16_iu*f_Klmn_R_Over_L + (float)AX0_s16_V_Beta*f_Kalman_Volts_Factor_Over_L);
   s16_Kalman_I_Beta += f_Klmn_Amp_To_Internal*(-f_cos_epos*f_Kalman_E_Vel*f_Klmn_Lambda_Over_L - (float)s16_i_beta*f_Klmn_R_Over_L + (float)AX0_s16_V_Alpha*f_Kalman_Volts_Factor_Over_L);
   AX0_s16_Kalman_E_Pos += (int)(f_Kalman_E_Vel*TS_KALMAN*RAD_TO_DRIVE_ANGLE);

   // calc P_minus
   f_f[1][3] = f_sin_epos*f_Lambda_Mpy_InvL;
   f_f[2][4] = f_Kalman_E_Vel*f_f[1][3];
   f_f[2][3] = -f_cos_epos*f_Lambda_Mpy_InvL;
   f_f[1][4] = -f_Kalman_E_Vel*f_f[2][3];

   for (j=1; j<=4; j++)
   {
         f_f_times_p[1][j] = f_R_Mpy_InvL_Neg*f_P_Minus[1][j]+f_f[1][3]*f_P_Minus[3][j]+f_f[1][4]*f_P_Minus[4][j];
         f_f_times_p[2][j] = f_R_Mpy_InvL_Neg*f_P_Minus[2][j]+f_f[2][3]*f_P_Minus[3][j]+f_f[2][4]*f_P_Minus[4][j];
         f_f_times_p[3][j] = f_P_Minus[4][j];
         f_f_times_p[4][j] = f_P_Minus[3][j];
   }

   f_p_times_f_transpose[1][1] = f_P_Minus[1][1]*f_R_Mpy_InvL_Neg+f_P_Minus[1][3]*f_f[1][3]+f_P_Minus[1][4]*f_f[1][4];
   f_p_times_f_transpose[1][2] = f_P_Minus[1][2]*f_R_Mpy_InvL_Neg+f_P_Minus[1][3]*f_f[2][3]+f_P_Minus[1][4]*f_f[2][4];
   f_p_times_f_transpose[1][3] = f_P_Minus[1][4];
   f_p_times_f_transpose[1][4] = f_P_Minus[1][3];

   f_p_times_f_transpose[2][1] = f_P_Minus[2][1]*f_R_Mpy_InvL_Neg+f_P_Minus[2][3]*f_f[1][3]+f_P_Minus[2][4]*f_f[1][4];
   f_p_times_f_transpose[2][2] = f_P_Minus[2][2]*f_R_Mpy_InvL_Neg+f_P_Minus[2][3]*f_f[2][3]+f_P_Minus[2][4]*f_f[2][4];
   f_p_times_f_transpose[2][3] = f_P_Minus[2][4];
   f_p_times_f_transpose[2][4] = f_P_Minus[2][3];

   f_p_times_f_transpose[3][1] = f_P_Minus[3][1]*f_R_Mpy_InvL_Neg+f_P_Minus[3][3]*f_f[1][3]+f_P_Minus[3][4]*f_f[1][4];
   f_p_times_f_transpose[3][2] = f_P_Minus[3][2]*f_R_Mpy_InvL_Neg+f_P_Minus[3][3]*f_f[2][3]+f_P_Minus[3][4]*f_f[2][4];
   f_p_times_f_transpose[3][3] = f_P_Minus[3][4];
   f_p_times_f_transpose[3][4] = f_P_Minus[3][3];

   f_p_times_f_transpose[4][1] = f_P_Minus[4][1]*f_R_Mpy_InvL_Neg+f_P_Minus[4][3]*f_f[1][3]+f_P_Minus[4][4]*f_f[1][4];
   f_p_times_f_transpose[4][2] = f_P_Minus[4][2]*f_R_Mpy_InvL_Neg+f_P_Minus[4][3]*f_f[2][3]+f_P_Minus[4][4]*f_f[2][4];
   f_p_times_f_transpose[4][3] = f_P_Minus[4][4];
   f_p_times_f_transpose[4][4] = f_P_Minus[4][3];

   f_P_Minus[1][1] += TS_KALMAN*(f_f_times_p[1][1] + f_p_times_f_transpose[1][1] + f_Q11);
   f_P_Minus[1][2] += TS_KALMAN*(f_f_times_p[1][2] + f_p_times_f_transpose[1][2]);
   f_P_Minus[1][3] += TS_KALMAN*(f_f_times_p[1][3] + f_p_times_f_transpose[1][3]);
   f_P_Minus[1][4] += TS_KALMAN*(f_f_times_p[1][4] + f_p_times_f_transpose[1][4]);

   f_P_Minus[2][1] += TS_KALMAN*(f_f_times_p[2][1] + f_p_times_f_transpose[2][1]);
   f_P_Minus[2][2] += TS_KALMAN*(f_f_times_p[2][2] + f_p_times_f_transpose[2][2] + f_Q22);
   f_P_Minus[2][3] += TS_KALMAN*(f_f_times_p[2][3] + f_p_times_f_transpose[2][3]);
   f_P_Minus[2][4] += TS_KALMAN*(f_f_times_p[2][4] + f_p_times_f_transpose[2][4]);

   f_P_Minus[3][1] += TS_KALMAN*(f_f_times_p[3][1] + f_p_times_f_transpose[3][1]);
   f_P_Minus[3][2] += TS_KALMAN*(f_f_times_p[3][2] + f_p_times_f_transpose[3][2]);
   f_P_Minus[3][3] += TS_KALMAN*(f_f_times_p[3][3] + f_p_times_f_transpose[3][3] + f_Q33);
   f_P_Minus[3][4] += TS_KALMAN*(f_f_times_p[3][4] + f_p_times_f_transpose[3][4]);

   f_P_Minus[4][1] += TS_KALMAN*(f_f_times_p[4][1] + f_p_times_f_transpose[4][1]);
   f_P_Minus[4][2] += TS_KALMAN*(f_f_times_p[4][2] + f_p_times_f_transpose[4][2]);
   f_P_Minus[4][3] += TS_KALMAN*(f_f_times_p[4][3] + f_p_times_f_transpose[4][3]);
   f_P_Minus[4][4] += TS_KALMAN*(f_f_times_p[4][4] + f_p_times_f_transpose[4][4] + f_Q44);
}

#pragma CODE_SECTION(KalmanFilterRT_2, "ramfunc_2");
void KalmanFilterRT_2(void)
{
   float f_temp1, f_temp2, f_hph_r_inv_factor;
   int i,j,s16_temp1,s16_temp2;

   if ((AX0_s16_Sensorless_Control_Bits & SL_KALMANING_STARTED_MASK) == 0) return;

   // Calc K matrix
   f_temp1 = f_P_Minus[1][1] + f_R11;
   f_temp2 = f_P_Minus[2][2] + f_R22;
   f_hph_r_inv_factor = (f_temp1*f_temp2-f_P_Minus[1][2]*f_P_Minus[2][1]);
   if (f_hph_r_inv_factor != 0) f_hph_r_inv_factor = 1/f_hph_r_inv_factor;

   f_hph_r_inv[1][1] = f_hph_r_inv_factor*f_temp2;
   f_hph_r_inv[1][2] = -f_hph_r_inv_factor*f_P_Minus[1][2];
   f_hph_r_inv[2][1] = -f_hph_r_inv_factor*f_P_Minus[2][1];
   f_hph_r_inv[2][2] = f_hph_r_inv_factor*f_temp1;

   f_k_k[1][1] = f_P_Minus[1][1]*f_hph_r_inv[1][1]+f_P_Minus[1][2]*f_hph_r_inv[2][1];
   f_k_k[1][2] = f_P_Minus[1][1]*f_hph_r_inv[1][2]+f_P_Minus[1][2]*f_hph_r_inv[2][2];

   f_k_k[2][1] = f_P_Minus[2][1]*f_hph_r_inv[1][1]+f_P_Minus[2][2]*f_hph_r_inv[2][1];
   f_k_k[2][2] = f_P_Minus[2][1]*f_hph_r_inv[1][2]+f_P_Minus[2][2]*f_hph_r_inv[2][2];

   f_k_k[3][1] = f_P_Minus[3][1]*f_hph_r_inv[1][1]+f_P_Minus[3][2]*f_hph_r_inv[2][1];
   f_k_k[3][2] = f_P_Minus[3][1]*f_hph_r_inv[1][2]+f_P_Minus[3][2]*f_hph_r_inv[2][2];

   f_k_k[4][1] = f_P_Minus[4][1]*f_hph_r_inv[1][1]+f_P_Minus[4][2]*f_hph_r_inv[2][1];
   f_k_k[4][2] = f_P_Minus[4][1]*f_hph_r_inv[1][2]+f_P_Minus[4][2]*f_hph_r_inv[2][2];

   // calc corrected P
   for (i=1; i<=4; i++)
      for (j=1; j<=4; j++)
      {
         f_p_correction[i][j] = f_k_k[i][1]*f_P_Minus[1][j]+f_k_k[i][2]*f_P_Minus[2][j];
      }
   for (i=1; i<=4; i++)
      for (j=1; j<=4; j++)
      {
         f_P_Minus[i][j] -= f_p_correction[i][j];
      }

   // calc corrected X
   s16_temp1 = s16_iu - s16_Kalman_I_Alpha;
   s16_temp2 = s16_i_beta - s16_Kalman_I_Beta;
   s16_Kalman_I_Alpha += f_k_k[1][1]*s16_temp1+f_k_k[1][2]*s16_temp2;
   s16_Kalman_I_Beta += f_k_k[2][1]*s16_temp1+f_k_k[2][2]*s16_temp2;
   f_Kalman_E_Vel += f_Klmn_Internal_To_Amp*(f_k_k[3][1]*s16_temp1+f_k_k[3][2]*s16_temp2);
   AX0_s16_Kalman_E_Pos += (int)(RAD_TO_DRIVE_ANGLE*f_Klmn_Internal_To_Amp*(f_k_k[4][1]*s16_temp1+f_k_k[4][2]*s16_temp2));

   #define KALMAN_DELTA_FILT_ALPHA  0.990 // 25 Hz
   #define KALMAN_DELTA_FILT_BETA   0.010 // 1-ALPHA
   f_temp1   = -8.192 * f_Kalman_E_Vel * INV_2_TIMES_PI; // units = rps electrical, 8.192=65536/8000
   f_Sl_Klmn_Delta = (KALMAN_DELTA_FILT_ALPHA*f_Sl_Klmn_Delta + KALMAN_DELTA_FILT_BETA*f_temp1);
   AX0_s16_Sl_Klmn_Delta = (int)(f_Sl_Klmn_Delta + 0.5);

   // Why 3.0?
   AX0_s16_Kalman_E_Vel = (int)(3.0 * (f_Sl_Klmn_Delta/(float)(VAR(AX0_s16_Num_Of_Poles_Ratio))) + 0.5);

   AX0_s16_Sensorless_Control_Bits |= SL_KALMAN_SYNC_MASK;
}*/

// SLVANGLF
int SalSlMspeedSpeedAdvCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s16_Sl_Mspeed_Speed_Advance) == (int)lparam) return (SAL_SUCCESS);;

   BGVAR(s16_Sl_Mspeed_Speed_Advance) = (int)lparam;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
   SpeedPhaseAdvDesign(DRIVE_PARAM);

   return (SAL_SUCCESS);
}

// SLVANGLH
int SalSlHalfMspeedSpeedAdvCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s16_Sl_Half_Mspeed_Speed_Advance) == (int)lparam) return (SAL_SUCCESS);;

   BGVAR(s16_Sl_Half_Mspeed_Speed_Advance) = (int)lparam;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
   SpeedPhaseAdvDesign(DRIVE_PARAM);

   return (SAL_SUCCESS);
}

int SensorlessDesign(int drive)
{
   CalcRTInjectionAmpAndFreq(drive);
   CalcTrackingFilterCoef(drive);
   CalcSaliancyGainOffset(16, drive);
   CalcRateLimiter(drive);
   CalcSlBemfVariables(drive);

   return (SAL_SUCCESS);
}

int SlHfCalibrationCommand(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (VAR(AX0_s16_Sl_Saliancy_Gain_Offset_State) == 1) return (SL_HF_CALIBRATE_ACTIVE);

   WaitForNext32kHzTaskStart();

   LVAR(AX0_s32_Sl_Calibration_Sin_Max) = LVAR(AX0_s32_Sl_Calibration_Cos_Max) = 0L;
   LVAR(AX0_s32_Sl_Calibration_Sin_Min) = LVAR(AX0_s32_Sl_Calibration_Cos_Min) = 0x7FFFFFFF;
   VAR(AX0_s16_Sl_Saliancy_Elect_Angle_Cycles) = 64;

   VAR(AX0_s16_Sl_Saliancy_Gain_Offset_State) = 1;

   return (SAL_SUCCESS);
}

void SensrolessSetupGainOffsetCalibrationCalc(int drive)
{
   // AXIS_OFF;

   float f_temp;
   long s32_sin_offset=0L, s32_cos_offset=0L;
   int  s16_sin_gain_fix=0,  s16_cos_gain_fix=0;
   unsigned int u16_sin_gain_shr=0, u16_cos_gain_shr=0;
   REFERENCE_TO_DRIVE;
   s32_sin_offset = -LVAR(AX0_s32_Sl_Calibration_Sin_Max) - LVAR(AX0_s32_Sl_Calibration_Sin_Min);

   s32_cos_offset = -LVAR(AX0_s32_Sl_Calibration_Cos_Max) - LVAR(AX0_s32_Sl_Calibration_Cos_Min);

   f_temp = 55000.0 / (float)(LVAR(AX0_s32_Sl_Calibration_Sin_Max) - LVAR(AX0_s32_Sl_Calibration_Sin_Min));
   FloatToFix16Shift16(&s16_sin_gain_fix,&u16_sin_gain_shr,f_temp);

   f_temp = 55000.0 / (float)(LVAR(AX0_s32_Sl_Calibration_Cos_Max) - LVAR(AX0_s32_Sl_Calibration_Cos_Min));
   FloatToFix16Shift16(&s16_cos_gain_fix,&u16_cos_gain_shr,f_temp);

   WaitForNext32kHzTaskStart();

   LVAR(AX0_s32_Sl_Offset_Sin) = s32_sin_offset;
   LVAR(AX0_s32_Sl_Offset_Cos) = s32_cos_offset;
   VAR(AX0_s16_Sl_Saliency_Sin_Gain_Fix_Design) = s16_sin_gain_fix;
   VAR(AX0_s16_Sl_Saliency_Sin_Gain_Shr_Design) = u16_sin_gain_shr;
   VAR(AX0_s16_Sl_Saliency_Cos_Gain_Fix_Design) = s16_cos_gain_fix;
   VAR(AX0_s16_Sl_Saliency_Cos_Gain_Shr_Design) = u16_cos_gain_shr;

   VAR(AX0_s16_Sensorless_Control_Bits) |= SL_COPY_SINCOS_GAINS_MASK;
}

int s16_sincos_factor = 16;
void SensorlessSetupHandler(int drive)
{
   int s16_temp;
   static long s32_calibration_timer;
   //static int s16_sincos_factor = 16;
   long s32_temp, s32_temp2;
   // AXIS_OFF;

   if ((BGVAR(u8_Sl_Mode) == 0) || (!Enabled(drive)))
   {
      BGVAR(s16_SlSetup_State) = SL_SETUP_IDLE;
      BGVAR(u64_Sys_Warnings) &= ~SLHF_CALIBRATION_PENDING;
      VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_INHIBIT_LOOPS_TILL_CALIB_END_MASK;
      return;
   }

   switch(BGVAR(s16_SlSetup_State))
   {
      case SL_SETUP_IDLE:
         s16_sincos_factor = 16;
      break;

      case SL_SETUP_START_HF_CALIBRATION:
         BGVAR(u64_Sys_Warnings) |= SLHF_CALIBRATION_PENDING;
         VAR(AX0_s16_Sensorless_Control_Bits) |= SL_INHIBIT_LOOPS_TILL_CALIB_END_MASK;
         CalcSaliancyGainOffset(s16_sincos_factor, drive);

         s32_calibration_timer = Cntr_1mS;
         BGVAR(s16_SlSetup_State) = SL_CALIBRATION_DELAY1;
      break;

      case SL_CALIBRATION_DELAY1: // Wait till commutation angle stabilizes
         if (PassedTimeMS(200L,s32_calibration_timer))
         {
            BGVAR(s16_SlSetup_State) = SL_INCREASE_GAIN_CALIBRATION;
         }
      break;

      case SL_INCREASE_GAIN_CALIBRATION:
         // Test if sin^2+cos^2 > threshold if not increase the gain
         s32_temp = (long)VAR(AX0_s16_Sl_Sin_Tracking_Act_Max);
         s32_temp = s32_temp*s32_temp;
         s32_temp2 = (long)VAR(AX0_s16_Sl_Cos_Tracking_Act_Max);
         s32_temp += s32_temp2*s32_temp2;
         if (s32_temp < 0x8000000)
         {
            s16_sincos_factor *= 2;
            if (s16_sincos_factor < 1025)
               BGVAR(s16_SlSetup_State) = SL_SETUP_START_HF_CALIBRATION;
            else
               BGVAR(s16_SlSetup_State) = SL_SETUP_FAILED;
         }
         else BGVAR(s16_SlSetup_State) = SL_START_SINCOS_SCAN_CALIBRATION;
      break;

      case SL_START_SINCOS_SCAN_CALIBRATION:
         s16_temp = SlHfCalibrationCommand(drive);
         if (s16_temp != SAL_SUCCESS)
         {
            BGVAR(s16_SlSetup_State) = SL_SETUP_FAILED;
            return;
         }
         s32_calibration_timer = Cntr_1mS;
         BGVAR(s16_SlSetup_State) = SL_CALIBRATION_DELAY2;
      break;

      case SL_CALIBRATION_DELAY2: // Wait till commutation angle stabilizes
         if (PassedTimeMS(100L,s32_calibration_timer))
         {
            VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_INHIBIT_LOOPS_TILL_CALIB_END_MASK;
            BGVAR(s16_SlSetup_State) = SL_SETUP_WAIT_FOR_HF_CALIBRATION;
         }
      break;

      case SL_SETUP_WAIT_FOR_HF_CALIBRATION:
         if (VAR(AX0_s16_Sl_Saliancy_Gain_Offset_State) == 1) return;

         BGVAR(u64_Sys_Warnings) &= ~SLHF_CALIBRATION_PENDING;

         SensrolessSetupGainOffsetCalibrationCalc(drive);
         BGVAR(s16_SlSetup_State) = SL_SETUP_DONE;
      break;

      case SL_SETUP_DONE:
      case SL_SETUP_FAILED:
         VAR(AX0_s16_Sensorless_Control_Bits) &= ~SL_INHIBIT_LOOPS_TILL_CALIB_END_MASK;
      break;
   }
}

int SlHfCalibrationStCommand(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (VAR(AX0_s16_Sl_Saliancy_Gain_Offset_State) == 0)  PrintString("Process Idle",0);
   else if (VAR(AX0_s16_Sl_Saliancy_Gain_Offset_State) == 1)
   {
      PrintString("Process Active\nCycles to go: ",0);
      PrintSignedInteger(VAR(AX0_s16_Sl_Saliancy_Elect_Angle_Cycles));
   }
   else PrintString("Process Done",0);

   PrintString("\nSin Values:\n",0);
   PrintDecInAsciiHex((unsigned long)LVAR(AX0_s32_Sl_Offset_Sin), 8);
   PrintChar(SPACE);
   PrintDecInAsciiHex((unsigned long)VAR(AX0_s16_Sl_Saliency_Sin_Gain_Fix), 4);
   PrintChar(SPACE);
   PrintDecInAsciiHex((unsigned long)VAR(AX0_s16_Sl_Saliency_Sin_Gain_Shr), 4);

   PrintString("\nCos Values:\n",0);
   PrintDecInAsciiHex((unsigned long)LVAR(AX0_s32_Sl_Offset_Cos), 8);
   PrintChar(SPACE);
   PrintDecInAsciiHex((unsigned long)VAR(AX0_s16_Sl_Saliency_Cos_Gain_Fix), 4);
   PrintChar(SPACE);
   PrintDecInAsciiHex((unsigned long)VAR(AX0_s16_Sl_Saliency_Cos_Gain_Shr), 4);
   PrintCrLf();

   return (SAL_SUCCESS);
}

void PrintFloat(float f_value)
{
   long s32_temp;
   int s16_temp;
   float f_temp = f_value;
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

/*   if (s16_temp)
   {
      PrintChar('E');
      PrintSignedInt16(s16_temp);
   }
   PrintChar(SPACE);

   // Display Fix-Shr value as well
   FloatToFixS32Shift16(&s32_temp, &u16_temp, f_temp);
   PrintDecInAsciiHex((unsigned long)s32_temp,8);
   PrintChar('>');
   PrintChar('>');
   PrintDecInAsciiHex((unsigned long)u16_temp,4);
*/
}

int StReadEKFStatusCommand(int drive)
{
   static int u16_ekf_status_state = 0;

   if (u8_Output_Buffer_Free_Space < 60) return (SAL_NOT_FINISHED);

   drive += 0;
   switch (u16_ekf_status_state)
   {
      case 0:
         PrintString("P_Minus:\n",0);
         PrintFloat(f_P_Minus[1][1]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[1][2]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[1][3]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[1][4]);
         PrintCrLf();
         u16_ekf_status_state++;
      break;

      case 1:
         PrintFloat(f_P_Minus[2][1]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[2][2]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[2][3]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[2][4]);
         PrintCrLf();
         u16_ekf_status_state++;
      break;

      case 2:
         PrintFloat(f_P_Minus[3][1]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[3][2]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[3][3]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[3][4]);
         PrintCrLf();
         u16_ekf_status_state++;
      break;

      case 3:
         PrintFloat(f_P_Minus[4][1]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[4][2]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[4][3]);
         PrintChar(SPACE);
         PrintFloat(f_P_Minus[4][4]);
         PrintCrLf();
         u16_ekf_status_state++;
      break;
   }

   if (u16_ekf_status_state >= 4)
   {
      u16_ekf_status_state = 0;
      return (SAL_SUCCESS);
   }

   return (SAL_NOT_FINISHED);
}
