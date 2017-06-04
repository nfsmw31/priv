#include <string.h>
#include "co_util.h"
#include "Math.h"
#include "objects.h"

#include "Design.def"
#include "Err_Hndl.def"
#include "ExFbVar.def"
#include "FltCntrl.def"
#include "Homing.def"
#include "i2c.def"
#include "ModCntrl.def"
#include "MultiAxis.def"
#include "Position.def"
#include "PtpGenerator.def"
#include "ser_comm.def"
#include "Velocity.def"
#include "AutoTune.def"
#include "Init.def"

#include "Drive.var"
#include "Endat.var"
#include "ExFbVar.var"
#include "Extrn_Asm.var"
#include "FltCntrl.var"
#include "Homing.var"
#include "Motor.var"
#include "Position.var"
#include "Prototypes.pro"
#include "ExSwVar.pro"
#include "PtpGenerator.var"
#include "Ser_Comm.var"
#include "Units.var"
#include "User_Var.var"
#include "Velocity.var"
#include "AutoTune.var"
#include "ModCntrl.var"
#include "init.var"


// flags will be used to signal management bits
// bit 0 - autotune
// bit 1 - if "SetOpmode" needs to be called (to avoid calling it recursively inside SetOpmode), and if the copy
//          of design->operative vars should not be initiated
void PositionConfig(int drive, int flags)
{
   // AXIS_OFF;
   float f_inertia_factor;

   /* Wait till last design is copied so global design vars will not be overrun */
   if (u16_background_ran) while ((VAR(AX0_s16_Crrnt_Run_Code) & (COPY_POS_COEF_MASK|COPY_NCT_COEF_MASK)) != 0);

//   if (BGVAR(u16_KnlGT_Mode) == 0)
  // {
      // use user set in position config
      BGVAR(u32_Nl_Kpd) = BGVAR(u32_Nl_Kpd_User);
      BGVAR(u32_Nl_Kpi) = BGVAR(u32_Nl_Kpi_User);
      BGVAR(u32_Nl_Kpiv) = BGVAR(u32_Nl_Kpiv_User);
      BGVAR(u32_Nl_Kpp) = BGVAR(u32_Nl_Kpp_User);
      BGVAR(u32_Nl_Kpgf) = BGVAR(u32_Nl_Kpgf_User);
      BGVAR(s16_Nl_Out_Filter_1) = BGVAR(s16_Nl_Out_Filter_1_User);
      BGVAR(s16_Nl_Out_Filter_2) = BGVAR(s16_Nl_Out_Filter_2_User);
      BGVAR(u32_Nl_K_Anti_Vibration) = BGVAR(u32_Nl_K_Anti_Vibration_User);
      BGVAR(s32_Pe_Filt_Gain) = BGVAR(s32_Pe_Filt_Gain_User);
      BGVAR(s32_Antivib3_Gain) = BGVAR(s32_Antivib3_Gain_User);
      BGVAR(u32_Nl_K_Anti_Resonance_Fcenter) = BGVAR(u32_Nl_K_Anti_Resonance_Fcenter_User);
      BGVAR(u32_Pe_Filt_Fcenter) = BGVAR(u32_Pe_Filt_Fcenter_User);
      BGVAR(u32_Antivib3_Fcenter) = BGVAR(u32_Antivib3_Fcenter_User);
      BGVAR(u16_Nl_K_Anti_Resonance_Sharpness) = BGVAR(u16_Nl_K_Anti_Resonance_Sharp_User);
      BGVAR(u16_Pe_Filt_Sharpness) = BGVAR(u16_Pe_Filt_Sharpness_User);
      BGVAR(u16_Antivib3_Sharpness) = BGVAR(u16_Antivib3_Sharpness_User);
      BGVAR(s16_Kbff_Spring_LPF) = BGVAR(s16_Kbff_Spring_LPF_User);
      BGVAR(u32_Nl_KffSpring) = BGVAR(u32_Nl_KffSpring_User);
      BGVAR(u32_LMJR) = BGVAR(u32_LMJR_User);
 //  }

   //else variables are written by gain table code

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
      f_inertia_factor = (float)BGVAR(s32_Motor_J)*(1+(float)BGVAR(u32_LMJR)/1000.0)/((float)BGVAR(u32_Motor_Kt)*1000);
   else//Linear
      //factor for s32_Motor_Mass is:  (pitch ^ 2) / (2 * PI)^2 = (pitch ^ 2) / 39.4786
      //factor for u32_Motor_Kf   is:  pitch / (2 * PI) = pitch / 6.2832
      //sum of factor: pitch * 0.15915
      f_inertia_factor = (((float)BGVAR(s32_Motor_Mass) * 1000.0 * (1+(float)BGVAR(u32_LMJR)/1000.0)/((float)BGVAR(u32_Motor_Kf)*1000.0)) * (float)(((float)BGVAR(u32_Mpitch) / 1000000.0) * 0.15915));//mpitch changed to decimal

   if (VAR(AX0_u16_Pos_Control_Mode) != LINEAR_POSITION)
      NonLinearPositionLoopConfig(drive, f_inertia_factor, flags); // Replaced autotune by bit 0 of flags argument
   else
      LinearPositionLoopconfig(drive, f_inertia_factor);
 
   if (BGVAR(u16_Dual_Loop_Mode))  DualLoopDesign(drive);

   // This will set the loop pointers according to the POSCONTROLMODE
   if ((LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) == &VAR(AX0_s16_Burnin_Command)) || (Enabled(drive))) return;

   if (flags & COPY_POS_MASK)
   {
      SetOpmode(drive, VAR(AX0_s16_Opmode));
      FalSetFBSyncOpmode(drive, (int)p402_modes_of_operation_display);
   }
}

void NonLinearPositionLoopConfig(int drive, float f_inertia_factor, int flags)
{
   // AXIS_OFF;

   unsigned long u32_mencres_equivalent;
   float f_Kiv_nlp_design, f_Ki_nlp_design, f_Kp_nlp_design, f_Kd_nlp_design, f_temp,f_temp_kd,
         f_Kff_spring_nlp_design,f_user_gain,f_ki_kiv_factor,f_nominal_acc,f_sample_rate_factor = (float)VAR(AX0_u16_Pos_Loop_Tsp)/25000.0;
   static float f_Ggp_nlp;
   long long s64_temp;
   unsigned long u32_kpiv, u32_kpi, u32_kpp, u32_nlpeaff, u32_temp;
   unsigned int u16_knlvff;
   long s32_temp;

   u32_kpiv    = BGVAR(u32_Nl_Kpiv);
   u32_kpi     = BGVAR(u32_Nl_Kpi);
   u32_kpp     = BGVAR(u32_Nl_Kpp);
   u32_nlpeaff = BGVAR(u32_Nl_KffSpring);

   u16_knlvff = BGVAR(u16_Nl_Vff);
   if ((VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_1) ||
       (VAR(AX0_u16_Pos_Control_Mode) == LINEAR_POSITION)         ) u16_knlvff = 1000;

   if (((VAR(AX0_s16_Opmode) == 0) || (VAR(AX0_s16_Opmode) == 1) || (BGVAR(u16_Dual_Loop_Mode) == 1)) &&
      (/*(BGVAR(u8_CompMode) == 5) ||*/ (BGVAR(u8_CompMode) == 6) || (BGVAR(u8_CompMode) == 7)))
   {
      if (BGVAR(u8_CompMode) == 6) // VELCONTROLMODE=6 zero KNLI,KNLIV
      {
         u32_kpiv = u32_kpi = 0;
      }
      if (BGVAR(u8_CompMode) == 7) // VELCONTROLMODE=7, KNLI=KNLD zero KNLP,KNLIV
      {
         u32_kpiv = u32_kpp;
         u32_kpp = u32_kpi = u32_nlpeaff = 0;
      }
   }
   // zero or not integrals according to gain switch logic
   u32_kpiv *= BGVAR(u32_Gain_Switch_Kpiv_SE);
   u32_kpi *= BGVAR(u32_Gain_Switch_Kpi_SE);

   if ((flags & AUTOTUNE_MASK) == 0) // No need to do in autotune \ gain table - will not change. to shorten execution time
   {
      if (AX0_u16_Pos_Control_Mode >= NON_LINEAR_POSITION_4)
         VAR(AX0_s16_Knld_62_5_Enable) = 1;
      else
         VAR(AX0_s16_Knld_62_5_Enable) = 0;

      LVAR(AX0_s32_Ptp_Filter_One_Feedback_Count) = (long)(1073741824.0 / (float)BGVAR(u32_User_Motor_Enc_Res));
      if (BGVAR(u16_FdbkType) == RESOLVER_FDBK)
      {
         u32_mencres_equivalent = 16384;
         LVAR(AX0_s32_Ptp_Filter_One_Feedback_Count) = 65536L;
      }
      else if (BGVAR(u16_FdbkType) == SW_SINE_FDBK)
      {
         LVAR(AX0_s32_Ptp_Filter_One_Feedback_Count) >>= 10;    // consider interpolation (roughly)
         u32_mencres_equivalent = BGVAR(u32_User_Motor_Enc_Res);
         if (BGVAR(u16_MotorType) == LINEAR_MOTOR) u32_mencres_equivalent = BGVAR(u32_User_Motor_Enc_Res) >> 8;
         else u32_mencres_equivalent = BGVAR(u32_User_Motor_Enc_Res) >> 16;
      }
      else // encoder
         u32_mencres_equivalent = BGVAR(u32_User_Motor_Enc_Res) >> 4;

      u32_mencres_equivalent = u32_mencres_equivalent >> (unsigned long)BGVAR(u16_Mencres_Shr);

      s64_temp = (long long)LVAR(AX0_s32_Ptp_Filter_One_Feedback_Count) << (long long)BGVAR(u16_Mencres_Shr);
      if (s64_temp > 0x7FFFFFFF) s64_temp = (long long)LVAR(AX0_s32_Ptp_Filter_One_Feedback_Count);
      LVAR(AX0_s32_Ptp_Filter_One_Feedback_Count) = (long)s64_temp;

      // 1 shr to take 0.5 fb counts (including MENCRESSHR) + 10 shr due to implementation. Totally 11 shr.
      LVAR(AX0_s32_Tqf_Resolution) = LVAR(AX0_s32_Ptp_Filter_One_Feedback_Count) >> 11;


      f_Ggp_nlp = u32_mencres_equivalent * 0.00000000055879354476928710936;

      // NCT vel-loop: Variable gain dependence on pe should be cancelled (depends only on speed)"
      if ((VAR(AX0_s16_Opmode) == 0) || (VAR(AX0_s16_Opmode) == 1) || (BGVAR(u16_Dual_Loop_Mode) == 1))
      {
         if ( ((BGVAR(u8_CompMode) == 5) || (BGVAR(u8_CompMode) == 6)) && (u32_kpiv == 0) && (u32_kpi == 0))
             f_Ggp_nlp = 0;
         if (BGVAR(u8_CompMode) == 7) f_Ggp_nlp = 0;
      }

      // variable gain limit is 1 upto 5
      // when PeVG 0 - 3 VcmdVG 1-2
      // 20480 >> 12 = 5 => all numbers will be scaled by 12 shr
      // PeVG 0-3 => 0-12288 VcmdVG 4096-8192

      // calc Pe variable gain
      FloatToFix16Shift16(&VAR(AX0_s16_Ggp_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Ggp_nlp_Shr_Design), 4096*f_Ggp_nlp);

      if ( (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_1) && (VAR(AX0_s16_Opmode) != 0) && (VAR(AX0_s16_Opmode) == 1) )
         FloatToFix16Shift16(&VAR(AX0_s16_Ggv_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Ggv_nlp_Shr_Design), 4096/178957.0); //poscontrolmode 1 - calc Vcmd varaible gain. minVc = 2^32*Tsp/6 = 178957 (== 10RPM)
      else
      {
         // revised variable gain to look at ve and not vcmd
         // calc Vcmd varaible gain. minVc = 10*2^32*Tsp/6 = 1789570 (== 100RPM)
         FloatToFix16Shift16(&VAR(AX0_s16_Ggv_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Ggv_nlp_Shr_Design), 4096/1789570.0);

         // Acc cmd variable gain
         // convert from rad/sec^2 to internal x[rad/s^2]*2^32/2/pi/4000/4000
         if (BGVAR(s32_Motor_J) == 0)
            f_temp = 0;
         else
         {
            f_temp = f_sample_rate_factor*f_sample_rate_factor*0.6041*(float)BGVAR(s32_Motor_I_Cont)*(float)BGVAR(u32_Motor_Kt)/(float)BGVAR(s32_Motor_J); // micont*sqrt(2)*mkt/(2*mj*50) 2*mj=>assume lmjr 1
            f_temp = 2500.0/f_temp;
         }
         FloatToFix16Shift16(&VAR(AX0_s16_Ggacc_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Ggacc_nlp_Shr_Design), f_temp);
      }

      // below the velocity error vcmd - based varigable gain is 0. note - speed units are at vel loop rate
      // default is one feedback pulse per servo period. mpy this by user variable.default is 0 for backwards compatible
      LVAR(AX0_s32_Pos_Vcmd_Tresh) = (long)(f_sample_rate_factor*(float)LVAR(AX0_s32_Ptp_Filter_One_Feedback_Count) * (float)BGVAR(u32_Nl_Speed_Gain_User) / 2000.0);

      // calc RT sat value
      VAR(AX0_s16_Variable_Gain_Max_Design) =  (int)((float)BGVAR(u32_Nl_Kpgmax)*4096.0/1000.0);


   }

   VAR(AX0_s16_Nlpeaff_Filter_Alpha_Design) = 32768*exp(-0.000000062832*(float)VAR(AX0_u16_Pos_Loop_Tsp)*(float)BGVAR(s16_Nl_KffSpring_LPF));
   VAR(AX0_s16_Nlpeaff_Filter_Beta_Design) = (int)(32768L - (long)VAR(AX0_s16_Nlpeaff_Filter_Alpha_Design));

   //  0 - active only during acceleration
   //  2 - active only during deceleration
   //  1 - active on both
   //  0< X <1 - During Acceleration NLPEAFF, During Deceleration x*NLpeaff
   //  2> X >1 - During Acceleration (2-x)*NLPEAFF, During Deceleration NLPEAFF
   VAR(AX0_s16_Kff_Spring_Acc_nlp_Fix_Design) = 0x1000;
   VAR(AX0_s16_Kff_Spring_Dec_nlp_Fix_Design) = 0x1000;
   if (BGVAR(u16_Nl_Dff_Ratio) == 0)
      VAR(AX0_s16_Kff_Spring_Dec_nlp_Fix_Design) = 0;
   else if (BGVAR(u16_Nl_Dff_Ratio) == 2000)
      VAR(AX0_s16_Kff_Spring_Acc_nlp_Fix_Design) = 0;
   else if (BGVAR(u16_Nl_Dff_Ratio) <= 1000)
      VAR(AX0_s16_Kff_Spring_Dec_nlp_Fix_Design) = (int)(4.096 * (float)(BGVAR(u16_Nl_Dff_Ratio)));
   else // if (BGVAR(u16_Nl_Dff_Ratio) > 1000)
      VAR(AX0_s16_Kff_Spring_Acc_nlp_Fix_Design) = (int)(4.096 * (float)(2000 - BGVAR(u16_Nl_Dff_Ratio)));

   // acceleration feed forward design
   VAR(AX0_s16_Aff_Lpf_Alpha_nlp_Design) = 32768*exp(-0.000000062832*(float)VAR(AX0_u16_Pos_Loop_Tsp)*(float)BGVAR(s16_Kbff_Spring_LPF));
   VAR(AX0_s16_Aff_Lpf_Beta_nlp_Design) = (int)(32768L - (long)VAR(AX0_s16_Aff_Lpf_Alpha_nlp_Design));
   if (VAR(AX0_s16_Aff_Lpf_Alpha_nlp_Design) <= 0)
   {
      VAR(AX0_s16_Aff_Lpf_Alpha_nlp_Design) = 0;
      VAR(AX0_s16_Aff_Lpf_Beta_nlp_Design) = 32767;
   }

   if (AX0_u16_Pos_Control_Mode >= NON_LINEAR_POSITION_4)
   {
      VAR(AX0_s16_Knld_62_5_Enable) = 1;
   }
   else
   {
      VAR(AX0_s16_Knld_62_5_Enable) = 0;
   }

   if (u32_nlpeaff == 0L)
      f_Kff_spring_nlp_design = 0;
   else
      f_Kff_spring_nlp_design = 405284734569.35108577551785283891/((float)u32_nlpeaff*(float)u32_nlpeaff*f_sample_rate_factor);
   if (f_Kff_spring_nlp_design > 32767.0) f_Kff_spring_nlp_design = 32767.0;
   FloatToFix16Shift16(&VAR(AX0_s16_Kff_Spring_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Kff_Spring_nlp_Shr_Design), f_Kff_spring_nlp_design);

   // calc acceleration feed forward. user value is % whrer 100 % = optimal value
   // optimal value J*acc = Kt*Iff => Iff = mj*(1+lmjr)/mkt*acc
   // BGVAR(Motor_J)/1000

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
      f_temp = 1e-3*(float)BGVAR(s32_Motor_J)*(1+0.001*(float)BGVAR(u32_LMJR))/(float)BGVAR(u32_Motor_Kt);
   else//Linear
      f_temp = (((float)BGVAR(s32_Motor_Mass) * 1000.0 * (1+0.001*(float)BGVAR(u32_LMJR))/((float)BGVAR(u32_Motor_Kf)*1000.0)) * (float)(((float)BGVAR(u32_Mpitch) / 1000000.0) * 0.15915));//mpitch changed to decimal

   s32_temp = BGVAR(s32_Motor_I_Cont);
   if (BGVAR(s32_Ilim_Actual) < s32_temp) s32_temp = BGVAR(s32_Ilim_Actual);
   f_nominal_acc = 1e-3*(float)s32_temp/f_temp;  // rad/sec^2 - needed for anti windup

   // concider acc units and dipeak
   f_temp = (0.01*(float)BGVAR(u16_KNLAFF)*f_temp/(f_sample_rate_factor *f_sample_rate_factor *42.723)) * 26214000/(float)BGVAR(s32_Drive_I_Peak); // 42.723=2^32/4000/4000/2/pi
   FloatToFix16Shift16(&VAR(AX0_s16_Acc_FF_Fix_Design),(unsigned int *)&VAR(AX0_s16_Acc_FF_Shr_Design),f_temp);

   // VG - 1-5
   // VG shr = 12 maxVG = 0x5000
   f_user_gain = 0.001*(float)(BGVAR(f_Gain_Switch_Factor) * (float)BGVAR(u32_Nl_Kpgf));
   //f_user_gain /= (1000.0 + (float)BGVAR(s32_Antivib3_Gain));
   //f_user_gain = (float)BGVAR(u32_Nl_Kpgf)/1000.0;

   f_Kd_nlp_design = (float)(BGVAR(u32_Nl_Kpd))*f_inertia_factor * 3.6767e-008;
   f_Kp_nlp_design = (float)u32_kpp*(float)u32_kpp*f_inertia_factor * 5.775369e-14;
   if ((VAR(AX0_s16_Variable_Gain_Max_Design) == 0x1000) || (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_1))
   {
      f_Kiv_nlp_design = (float)(u32_kpiv)*(float)(u32_kpiv)*f_inertia_factor*2.3101e-10;
      f_Ki_nlp_design = (float)(u32_kpi)*(float)(u32_kpi)*(float)(u32_kpi)*f_inertia_factor * 3.62877e-16;
   }
   else
   {
      f_temp = (float)VAR(AX0_s16_Variable_Gain_Max_Design)*(float)u32_kpiv/((float)(VAR(AX0_s16_Variable_Gain_Max_Design) - 4096));
      f_Kiv_nlp_design = f_temp*f_temp*f_inertia_factor*2.3101e-10;

      f_temp = (float)VAR(AX0_s16_Variable_Gain_Max_Design)*(float)u32_kpi/((float)(VAR(AX0_s16_Variable_Gain_Max_Design) - 4096));
      f_Ki_nlp_design = f_temp*f_temp*f_temp*f_inertia_factor * 3.62877e-16;
   }

   f_Kiv_nlp_design = f_Kiv_nlp_design/f_sample_rate_factor;

   f_temp_kd = 26214000/((float)BGVAR(s32_Drive_I_Peak))*f_Kd_nlp_design*f_user_gain/4096.0;
   // Do not consider Tsp in the calculation of Pure_Kd as it is compensated in RT.
   FloatToFix16Shift16((int *)&VAR(AX0_u16_Pure_Kd_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Pure_Kd_nlp_Shr_Design),f_temp_kd);
   f_temp_kd = f_temp_kd/f_sample_rate_factor ;
   f_temp = 0.001*(float)u16_knlvff*f_temp_kd;
   FloatToFix16Shift16(&VAR(AX0_s16_Kd_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Kd_nlp_Shr_Design),f_temp);
   // knlvff support - KD
   if (u16_knlvff == 1000)
   {
      VAR(AX0_s16_Kvff_Kd_Part_nlp_Fix_Design) = 0;
      VAR(AX0_s16_Kvff_Kd_Part_nlp_Shr_Design) = VAR(AX0_s16_Kd_nlp_Shr_Design);
   }
   else
   {
      f_temp = (1-0.001*(float)u16_knlvff)*f_temp_kd;
      FloatToFix16Shift16(&VAR(AX0_s16_Kvff_Kd_Part_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Kvff_Kd_Part_nlp_Shr_Design),-f_temp);

      if (u16_knlvff == 0)
      {
         VAR(AX0_s16_Kd_nlp_Fix_Design) = 0;
         VAR(AX0_s16_Kd_nlp_Shr_Design) = VAR(AX0_s16_Kvff_Kd_Part_nlp_Shr_Design);
      }
   }
   // AW design
   // units conversiotn from rad/sec^2  (1/2/pi) * 2^32/4000/4000 = 42.723
   f_temp = f_temp_kd * (float)VAR(AX0_s16_Variable_Gain_Max_Design) * sqrt(f_sample_rate_factor *f_sample_rate_factor*85.446*f_nominal_acc); // sqrt(2*nom_acc with units conversion)
   FloatToFix16Shift16(&VAR(AX0_s16_AW_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_AW_nlp_Shr_Design),f_temp);

   f_temp = 26214000/((float)BGVAR(s32_Drive_I_Peak))*f_Kp_nlp_design*f_user_gain*f_user_gain/1024.0;
   FloatToFix16Shift16(&VAR(AX0_s16_Kp_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Kp_nlp_Shr_Design),f_temp);

   if (u16_knlvff == 1000)
   {
      VAR(AX0_s16_AW_VFF_nlp_Fix_Design) = 0;
      VAR(AX0_s16_AW_VFF_nlp_Shr_Design) = 0;
      VAR(AX0_s16_AW_Vc_nlp_Fix_Design) = 0;
      VAR(AX0_s16_AW_Vc_nlp_Shr_Design) = 0;
   }
   else
   {
      // AW vff part
      f_temp = 4*(1-0.001*(float)u16_knlvff)*f_temp_kd/f_temp;
      FloatToFix16Shift16(&VAR(AX0_s16_AW_VFF_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_AW_VFF_nlp_Shr_Design),-f_temp);

      // AW Vc part
      f_temp = (1-0.001*(float)u16_knlvff)*f_temp_kd * (float)VAR(AX0_s16_Variable_Gain_Max_Design);
      FloatToFix16Shift16(&VAR(AX0_s16_AW_Vc_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_AW_Vc_nlp_Shr_Design),f_temp);
   }

   if (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_1)
      f_ki_kiv_factor = 4.2949e+008;
   else
      f_ki_kiv_factor = 214745000;


   f_temp = (f_ki_kiv_factor/((float)BGVAR(s32_Drive_I_Peak))*f_Ki_nlp_design*f_user_gain*f_user_gain*f_user_gain)/256.0;
   FloatToFix16Shift16(&VAR(AX0_s16_Ki_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Ki_nlp_Shr_Design), f_temp);

   f_temp = (f_ki_kiv_factor/((float)BGVAR(s32_Drive_I_Peak))*f_Kiv_nlp_design*f_user_gain*f_user_gain)/1024;
   FloatToFix16Shift16(&VAR(AX0_s16_Kiv_nlp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Kiv_nlp_Shr_Design), f_temp);

   // determine KpKd shifts. leave shifts at kd_shr,kp_shr for error scale calc
   // do not consider AX0_s16_Pure_Kd_nlp_Shr_Design in the min calculation, since it is not in the position controller
   VAR(AX0_s16_KdKp_nlp_Shr_Design) = VAR(AX0_s16_Kd_nlp_Shr_Design);
   if (VAR(AX0_s16_Kp_nlp_Shr_Design) < VAR(AX0_s16_KdKp_nlp_Shr_Design)) VAR(AX0_s16_KdKp_nlp_Shr_Design) = VAR(AX0_s16_Kp_nlp_Shr_Design);
   if (VAR(AX0_s16_Kvff_Kd_Part_nlp_Shr_Design) < VAR(AX0_s16_KdKp_nlp_Shr_Design)) VAR(AX0_s16_KdKp_nlp_Shr_Design) = VAR(AX0_s16_Kvff_Kd_Part_nlp_Shr_Design);
   if (VAR(AX0_s16_KdKp_nlp_Shr_Design) < 5) VAR(AX0_s16_KdKp_nlp_Shr_Design) = 0;
   else
   {
      VAR(AX0_s16_KdKp_nlp_Shr_Design) = VAR(AX0_s16_KdKp_nlp_Shr_Design)-5;
      VAR(AX0_s16_Kd_nlp_Shr_Design) -= VAR(AX0_s16_KdKp_nlp_Shr_Design);
      VAR(AX0_s16_Kp_nlp_Shr_Design) -= VAR(AX0_s16_KdKp_nlp_Shr_Design);
      VAR(AX0_s16_Kvff_Kd_Part_nlp_Shr_Design) -= VAR(AX0_s16_KdKp_nlp_Shr_Design);
      VAR(AX0_s16_Pure_Kd_nlp_Shr_Design) -= VAR(AX0_s16_KdKp_nlp_Shr_Design);
      if (VAR(AX0_s16_Pure_Kd_nlp_Shr_Design) < 0)
      {
         // only "-1" is possible because Pure_Kd = Kvff_Kd_Part + Kd, so max(Kvff_Kd_Part, Kd) >= Pure_Kd/2
         VAR(AX0_s16_Pure_Kd_nlp_Shr_Design) = 0;
         VAR(AX0_u16_Pure_Kd_nlp_Fix_Design) <<= 1;  // The value is unsigned, but calculated as signed, so there is a room for 1 shl.
      }   }

   // determine Ki Kiv shifts
   if (u32_kpiv == 0L)
   {
      VAR(AX0_s16_Kiv_nlp_Shr_Design) = 5;
      VAR(AX0_s16_Kiv_nlp_Fix_Design) = 0;
   }

   if (u32_kpi == 0L)
   {
      VAR(AX0_s16_Ki_nlp_Shr_Design) = 5;
      VAR(AX0_s16_Ki_nlp_Fix_Design) = 0;
   }

   // determine ki kiv shr. leave shifts for error scale calc
   VAR(AX0_s16_KiKiv_nlp_Shr_Design) = VAR(AX0_s16_Ki_nlp_Shr_Design);
   if (VAR(AX0_s16_Kiv_nlp_Shr_Design) < VAR(AX0_s16_KiKiv_nlp_Shr_Design))
      VAR(AX0_s16_KiKiv_nlp_Shr_Design) = VAR(AX0_s16_Kiv_nlp_Shr_Design);

   if (VAR(AX0_s16_KiKiv_nlp_Shr_Design) < 5) VAR(AX0_s16_KiKiv_nlp_Shr_Design) = 0;
   else
   {
      VAR(AX0_s16_KiKiv_nlp_Shr_Design) = VAR(AX0_s16_KiKiv_nlp_Shr_Design) - 5;
      VAR(AX0_s16_Kiv_nlp_Shr_Design) -= VAR(AX0_s16_KiKiv_nlp_Shr_Design);
      VAR(AX0_s16_Ki_nlp_Shr_Design) -= VAR(AX0_s16_KiKiv_nlp_Shr_Design);
   }

   if (u32_kpiv == 0L) VAR(AX0_s16_Kiv_nlp_Shr_Design) = 5;
   if (u32_kpi == 0L) VAR(AX0_s16_Ki_nlp_Shr_Design) = 5;
   if (u32_kpp == 0L) VAR(AX0_s16_Kp_nlp_Shr_Design) = 5;

   // find min ki,kp shr - this is used by the scaled pe
   VAR(AX0_s16_Pe_Scale_Factor_Limit_Design) = VAR(AX0_s16_Ki_nlp_Shr_Design);
   if (VAR(AX0_s16_Kp_nlp_Shr_Design) < VAR(AX0_s16_Pe_Scale_Factor_Limit_Design)) VAR(AX0_s16_Pe_Scale_Factor_Limit_Design) = VAR(AX0_s16_Kp_nlp_Shr_Design);
   VAR(AX0_s16_Pe_Scale_Factor_Limit_Design) = 32 - VAR(AX0_s16_Pe_Scale_Factor_Limit_Design);

   u32_temp = u32_kpi;
   VAR(AX0_s16_KivKi_Switch_Bits) &= ~COPY_KI_SWITCH_TO_KIV_SWITCH_MASK;
   // Use KNLIV in NonLinear Velocity control loop (poscontrolmode 5 only)
   if (((VAR(AX0_s16_Opmode) == 0) || (VAR(AX0_s16_Opmode) == 1) || (BGVAR(u16_Dual_Loop_Mode) == 1)) &&
       (BGVAR(u8_CompMode) == 7)                                  &&
       (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5)) 
   {
        u32_temp = u32_kpiv;
        // Signal to RT to use Ki switch
        VAR(AX0_s16_KivKi_Switch_Bits) |= COPY_KI_SWITCH_TO_KIV_SWITCH_MASK;
   }
   // Calc (1.0 + KNLI[Hz]*Usergain*2pi*Servoperiod) for integral saturation switch
   // Use // Use KNLIV instead KNLI in NonLinear Velocity control loop (poscontrolmode 5 only)
   FloatToFix16Shift16(&VAR(AX0_s16_Ki_Int_Sat_Sw_Fix_Design),
                       &VAR(AX0_u16_Ki_Int_Sat_Sw_Shr_Design),
                       (1.0 + (float)u32_temp*(float)BGVAR(u32_Nl_Kpgf_User)*6.283185307*(float)VAR(AX0_u16_Pos_Loop_Tsp)*1e-14));
   NLTorqueFilterDesign(drive);
   NLAntiResFilter(drive);
   NLNotchFilterDesign(drive);
   NLPeAntiVibDesign(drive);
   NLPeAntiVib3Design(drive);

   //Generate a fault if num of shifts too high.
   if (VAR(AX0_s16_KdKp_nlp_Shr_Design)> 63) u16_Internal_OverFlow |= 0x20; // Signal internal error

   SpeedObserverDesign(drive);

   // If called from autotune or from SetOpmode no need to wait for the copy, it will be done explicitly
   if (flags) return;

   // Set position coef bit to indicate RT to start coef copying
   VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
}


void DualLoopDesign(int drive)
{

  // AXIS_OFF;
  long s32_temp;
  float f_temp,f_sample_rate_factor = (float)VAR(AX0_u16_Pos_Loop_Tsp)/25000.0,
        f_vlim_rpm = (float)BGVAR(s32_V_Lim_Design) * 0.00011176;
  REFERENCE_TO_DRIVE;
  
  // dual loop scale - the factor between 1 count in vel loop / one count in pos loop = SFBUNITSNUM/SFBUNITSDEN 
  BGVAR(f_Dual_Loop_Scale) = (float)BGVAR(s32_SFBUnitsNum) / (float)BGVAR(s32_SFBUnitsDen);
  
  // calc velocity feed forward rt coeff
  // input is pcmd/dTsp ( only 125usec tsp is supported here  = > user_value_range_0_to_1 * Dual_loop_scale
  // output units are the same
  // user value 0-2 unitsless gain
  FloatToFix16Shift16(&VAR(AX0_s16_Dual_Loop_Vff_Fix_Design), (unsigned int *)&VAR(AX0_s16_Dual_Loop_Vff_Shr_Design), 
                      0.001*(float)BGVAR(u16_Dual_Loop_Vff)*BGVAR(f_Dual_Loop_Scale)                                 );

  // calc kp rt coeff
  // user value is in Hz
  // Kp_dual_loop  = 2*pi*user_value_in_hz *Dual_loop_scale 
  FloatToFix16Shift16(&VAR(AX0_s16_Dual_Loop_Kp_Fix_Design), (unsigned int *)&VAR(AX0_s16_Dual_Loop_Kp_Shr_Design), 
                      0.0062832*(float)BGVAR(s32_Dual_Loop_Kp)*BGVAR(f_Dual_Loop_Scale)*250e-6*f_sample_rate_factor); 

  // calc acceleration feed forward
  if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
      f_temp = 1e-3*(float)BGVAR(s32_Motor_J)*(1+0.001*(float)BGVAR(u32_LMJR))/(float)BGVAR(u32_Motor_Kt);
  else//Linear
      f_temp = (((float)BGVAR(s32_Motor_Mass) * 1000.0 * (1+0.001*(float)BGVAR(u32_LMJR))/((float)BGVAR(u32_Motor_Kf)*1000.0)) * (float)(((float)BGVAR(u32_Mpitch) / 1000000.0) * 0.15915));//mpitch changed to decimal

   // concider acc units and dipeak
   f_temp = (0.01*(float)BGVAR(u16_Dual_Loop_AFF)*f_temp/(f_sample_rate_factor *f_sample_rate_factor *42.723)) * 26214000/(float)BGVAR(s32_Drive_I_Peak); // 42.723=2^32/4000/4000/2/pi
   FloatToFix16Shift16(&VAR(AX0_s16_Dual_Loop_AFF_Fix_Design),(unsigned int *)&VAR(AX0_s16_Dual_Loop_AFF_Shr_Design),f_temp*BGVAR(f_Dual_Loop_Scale));


   // calc scalijng from dual loop vcmd to linear velocity loop units
   // dual_loop_vcmd/fbc32 * 8000 * 60 /vlim_rpm*22750
   //f_temp = 8000*60/4294967296 * 22750/f_vlim_rpm;

   f_temp = 0.000111758708953857421875 * 22750/f_vlim_rpm;

   FloatToFix16Shift16(&VAR(AX0_s16_Dual_Loop_LinearVel_Scale_Fix_Design),
                       (unsigned int *)&VAR(AX0_s16_Dual_Loop_LinearVel_Scale_Shr_Design),f_temp);

   s32_temp = (1LL << (long)VAR(AX0_s16_Dual_Loop_LinearVel_Scale_Shr_Design)) - 1; 
   
   VAR(AX0_s16_Dual_Loop_LinearVel_Scale_Residue_Mask_Lo) = (int)s32_temp;
   VAR(AX0_s16_Dual_Loop_LinearVel_Scale_Residue_Mask_Hi) = (int)(s32_temp >> 16);

   VAR(AX0_u16_Dual_Loop_Control_Bits) |= DUAL_LOOP_DESIGN_COPY_MASK;
}

int SalKnlDualLoopAffCommand(long long param,int drive)
{
   BGVAR(u16_Dual_Loop_AFF) = (unsigned int)param;
   DualLoopDesign(drive);
   return (SAL_SUCCESS);   
}

int SalKnlDualLoopKpCommand(long long param,int drive)
{
   BGVAR(s32_Dual_Loop_Kp) = (long)param;
   DualLoopDesign(drive);
   return (SAL_SUCCESS);
}

int SalKnlDualLoopVffCommand(long long param,int drive)
{  
   BGVAR(u16_Dual_Loop_Vff) = (unsigned int)param;
   DualLoopDesign(drive);
   return (SAL_SUCCESS);
}


void SpeedObserverDesign(int drive)
{
   // AXIS_OFF;

   float f_temp,f_vlim_rpm = (float)BGVAR(s32_V_Lim_Design) * 0.00011176;
   REFERENCE_TO_DRIVE;

   // @_AX0_s16_Observer_Acc_Factor_Fix,shr = 0.8*Kt/Jtotal/26214*dipeak
   // mpy by 100 to maximize resolution
   f_temp = (float)BGVAR(s32_Motor_J)*(1+(float)BGVAR(u32_Speed_Observer_Spd_LMJR)/1000.0)/((float)BGVAR(u32_Motor_Kt)*1000);
   f_temp = 100*(0.001/26214)*(float)BGVAR(s32_Drive_I_Peak)/f_temp;
   FloatToFix16Shift16(&VAR(AX0_s16_Observer_Acc_Factor_Fix),(unsigned int *)&VAR(AX0_s16_Observer_Acc_Factor_Shr),f_temp);

   // observer_gain_acc is used to also scale speed to rad/sec*100
   // observer_gain_acc = gain_value * 100*2*pi/2^32/62.5e-6
   f_temp = 100.0*6.2832/4294967296.0/62.5e-6;
   f_temp = 0.001*(float)BGVAR(s32_Speed_Observer_Acc_Gain)*f_temp; // 200

   if (f_temp == 0)
   {
     VAR(AX0_s16_Observer_Gain_Acc_Fix) = 0;
     VAR(AX0_s16_Observer_Gain_Acc_Shr) = VAR(AX0_s16_Observer_Acc_Factor_Shr);
   }
   else
    FloatToFix16Shift16(&VAR(AX0_s16_Observer_Gain_Acc_Fix),(unsigned int *)&VAR(AX0_s16_Observer_Gain_Acc_Shr),f_temp);

   // optimize accuracy by doing shr as late as possible
   VAR(AX0_s16_Observer_Gain_Acc_Out_Shr) = VAR(AX0_s16_Observer_Acc_Factor_Shr);
   if (VAR(AX0_s16_Observer_Gain_Acc_Shr) <  VAR(AX0_s16_Observer_Gain_Acc_Out_Shr)) VAR(AX0_s16_Observer_Gain_Acc_Out_Shr) = VAR(AX0_s16_Observer_Gain_Acc_Shr);
   VAR(AX0_s16_Observer_Gain_Acc_Shr) -= VAR(AX0_s16_Observer_Gain_Acc_Out_Shr);
   VAR(AX0_s16_Observer_Acc_Factor_Shr) -= VAR(AX0_s16_Observer_Gain_Acc_Out_Shr);


   // observer_gain_spd is used to also scale speed to rad/sec*10000

   f_temp = 10000*6.2832/4294967296.0;// spd units =rad/sec*10000
   f_temp = 0.001*(float)BGVAR(s32_Speed_Observer_Spd_Gain)*f_temp;// 10
   FloatToFix16Shift16(&VAR(AX0_s16_Observer_Gain_Spd_Fix),(unsigned int *)&VAR(AX0_s16_Observer_Gain_Spd_Shr),f_temp);

   // AX0_s16_Observer_Pos_Scale_Fix,Shr = VelocityServoPeriod*2^32/2/pi/10000
   f_temp = 62.5e-6*4294967296.0/6.2832/10000;
   FloatToFix16Shift16(&VAR(AX0_s16_Observer_Pos_Scale_Fix),(unsigned int *)&VAR(AX0_s16_Observer_Pos_Scale_Shr),f_temp);

   // units conversion for vel_var_fb_0
   FloatToFix16Shift16(&VAR(AX0_s16_Observer_V_Scale_Fix),(unsigned int *)&VAR(AX0_s16_Observer_V_Scale_Shr),21.72465/f_vlim_rpm);

}

void SpeedObserverReset(int drive)
{
   // AXIS_OFF;
   int s16_temp;
   REFERENCE_TO_DRIVE;
   do {
      s16_temp = Cntr_3125;
      LLVAR(AX0_u32_Observer_Pfb_Lo) = LLVAR(AX0_u32_Pos_Fdbk_Dual_Lo);
      LVAR(AX0_s32_Observer_Acc) = 0;
      LVAR(AX0_s32_Observer_Spd) = 0;
   } while (s16_temp != Cntr_3125);

}

int SalSpeedObserverAccGainCommand(long long param,int drive)
{
   BGVAR(s32_Speed_Observer_Acc_Gain) = (long)param;
   SpeedObserverDesign(drive);
   SpeedObserverReset(drive);
   return (SAL_SUCCESS);
}
int SalSpeedObserverSpdGainCommand(long long param,int drive)
{
   BGVAR(s32_Speed_Observer_Spd_Gain) = (long)param;
   SpeedObserverDesign(drive);
   SpeedObserverReset(drive);
   return (SAL_SUCCESS);
}

int SalSpeedObserverLMJRCommand(long long param,int drive)
{
   BGVAR(u32_Speed_Observer_Spd_LMJR) = (long)param;
   SpeedObserverDesign(drive);
   SpeedObserverReset(drive);
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: InitializeModuloModeStateMachine
// Description: Initializes the Modulo Mode State Machine if the Modulo Mode is Active, i.e: 1,3,5
//
//      It is important to Initialize the State Machine after major changes to the Configuration.
//      This was added to run when the "Config" command is executed.
//
// Author: Daniel
// Algorithm:
// Revisions:
//**********************************************************
void InitializeModuloModeStateMachine(int drive)
{
    // AXIS_OFF;
   REFERENCE_TO_DRIVE;

    //  Re-Initialize Modulo State Machine bit mask
    //  The right most bit of this variable indicates if the modulo is active
    //  If the modulo is active then bitwise AND with 1 will Restart the Modulo State Machine (clearing all the other bits in the bit mask)
    //  Else the Modulo is not active hence bitwise AND with 1 wont change anything
    VAR(AX0_u16_Position_Modulo_Active) &= 1;
}

void LinearPositionLoopconfig(int drive, float f_inertia_factor)
{
   // AXIS_OFF;
   long long s64_sigma_r = 0;
   int s16_temp;
   REFERENCE_TO_DRIVE;

   //KPP update
   //internal units for [rps/rev] = (user/1000 * 22750/VLIM_INT * 2^32/8000) / 2^32 = user * 0.00284375 / VLIM_INT
   FloatToFix16Shift16(&VAR(AX0_s16_Kpp_Fix_Design),
                        &VAR(AX0_u16_Kpp_Shr_Design),
                        (float)((float)BGVAR(u32_Kpp)* 0.00284375 / (float)BGVAR(s32_V_Lim_Design)));

   //KPI update
   //internal units for [Hz]*KPP = (user/1000000 * 22750/VLIM_INT * 2^32/8000/4000) / 2^32 = user * KPP * 0.0000000007109375 / VLIM_INT
   FloatToFix16Shift16(&VAR(AX0_s16_Kpi_Fix_Design),
                        &VAR(AX0_u16_Kpi_Shr_Design),
                        (float)((float)BGVAR(u32_Kpi)*(float)BGVAR(u32_Kpp)* 0.0000000007109375 / (float)BGVAR(s32_V_Lim_Design)));

   //KPISATOUT update
   //internal units for [rps] = user / 1000 * 22750 / VLIM_INT * 2^32 / 8000 = user * 12213813.248 / VLIM_INT
   LVAR(AX0_u32_Sat_Out_Design) = (unsigned long)((float)BGVAR(u32_KpiSatOut) * 12213813.248 / (float)BGVAR(s32_V_Lim_Design));

   //KPE update
   //unitless - just multiplies 100*KPE*(PE^2)
   FloatToFix16Shift16(&VAR(AX0_s16_Kpe_Fix_Design),
                        &VAR(AX0_u16_Kpe_Shr_Design),
                        ((float)BGVAR(u16_Kpe) / 10.0));

   //KPD update
   //internal units for [rps/rev] = (user/1000 * 22750/VLIM_INT * 2^32/8000 * 4000) / 2^32 = user * 11.375 / VLIM_INT
   FloatToFix16Shift16(&VAR(AX0_s16_Kpd_Fix_Design),
                        &VAR(AX0_u16_Kpd_Shr_Design),
                        (float)((float)BGVAR(u32_Kpd)* 11.375 / (float)BGVAR(s32_V_Lim_Design)));

   //KPVFR update
   //internal units = (user/1000 * 22750/VLIM_INT * 2^32/8000 * 4000) / 2^32 = user * 11.375 / VLIM_INT
   FloatToFix16Shift16(&VAR(AX0_s16_Kpvfr_Fix_Design),
                        &VAR(AX0_u16_Kpvfr_Shr_Design),
                        (float)((float)BGVAR(s32_Kpvfr)* 11.375 / (float)BGVAR(s32_V_Lim_Design)));

   //KPVFR2 update
   //internal units = sigma(Vel_R) * (user/1000 * 22750/VLIM_INT * 2^32/8000 * 4000) / 2^32 = user * 11.375 / VLIM_INT
   for(s16_temp=0;s16_temp<=4;s16_temp++) s64_sigma_r += (long long)BGVAR(s32_R_Poly);
   s64_sigma_r  = (s64_sigma_r >> 16LL);
   FloatToFix16Shift16(&VAR(AX0_s16_Kpvfr2_Fix_Design),
                        &VAR(AX0_u16_Kpvfr2_Shr_Design),
                        (float)((float)s64_sigma_r * /*(float)(1L << (long)VAR(AX0_s16_Vel_Var_Hr_Shr1_Design))**/(float)BGVAR(u32_Kpvfr2)* 11.375 / ((float)BGVAR(s32_V_Lim_Design)) ));

   //KPAFR update
   //internal units (user * 22750/VLIM_INT * 2^32/8000 * 4000^2) / 2^32 = user * 45.5 / VLIM_INT
   FloatToFix16Shift16(&VAR(AX0_s16_Kpafr_Fix_Design),
                        &VAR(AX0_u16_Kpafr_Shr_Design),
                        (float)((float)BGVAR(s32_Kpafr)* 45.5 / (float)BGVAR(s32_V_Lim_Design)));

   //KPAFRC update
   //internal units (user/1000 * 4000^2 * f_inertia_factor * 26214 / (DIPEAK/1000)) / 2^32 = 97.6547598 / DIPEAK
   FloatToFix16Shift16(&VAR(AX0_s16_Kpafr_Current_Fix_Design),
                        &VAR(AX0_u16_Kpafr_Current_Shr_Design),
                        (float)((float)BGVAR(s32_Kpafrcurrent) * f_inertia_factor * 97.6547598 / (float)BGVAR(s32_Drive_I_Peak)));

   // Channel the source for the KPAFR/KPVFR - on gearing with GEARFILT use the filtered acceleration/velocity value
   if ((VAR(AX0_s16_Opmode) == 4) && VAR(AX0_u16_Qep_Msq_Fltr_Mode) && (VAR(AX0_u16_Move_Smooth_Mode)==0))
   {
      VAR(AX0_s16_KPAFR_Source_Ptr) = (int)((long)&LVAR(AX0_s32_Gear_Acc_Out) & 0xffff);
      VAR(AX0_s16_KPVFR_Source_Ptr) = (int)((long)&LVAR(AX0_s32_Gear_Vel_Out) & 0xffff);
   }
   else
   {
      VAR(AX0_s16_KPAFR_Source_Ptr) = (int)((long)&LVAR(AX0_s32_Delta_Delta_Pos_Cmd) & 0xffff);
      VAR(AX0_s16_KPVFR_Source_Ptr) = (int)((long)&LVAR(AX0_s32_Delta_Pos_Cmd) & 0xffff);
   }

   //FRIC Compensation
   //internal units = (1/1000 * 22750/VLIM_INT * 2^32/8000 * 4000) / 2^32 = 11.375 / VLIM_INT
   FloatToFix16Shift16(&VAR(AX0_s16_Fric_Fix_Design),
                        &VAR(AX0_u16_Fric_Shr_Design),
                        (float)(11.375 / (float)BGVAR(s32_V_Lim_Design)));

   // make sure nlp Aff is inactive in case of linear controller
   VAR(AX0_s16_Kff_Spring_nlp_Fix_Design) = VAR(AX0_s16_Kff_Spring_nlp_Shr_Design) = 0;

   // Set position coef bit to indicate RT to start coef copying
   VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_POS_COEF_MASK);

}


void NLPeAntiVibDesign(int drive)
{
   // PCMD RLc (notch) filter design
   // AXIS_OFF;
   float f_LR, f_RC, f_pe_notch_out, f_d,f_sample_rate_factor ;
   // protect from divide by zero fault - reproduced with gtmode != 0 (Yuval, 12/2013)
   long s32_pe_filt_gain = BGVAR(s32_Pe_Filt_Gain);
   unsigned long u32_pe_fcenter = BGVAR(u32_Pe_Filt_Fcenter);
   REFERENCE_TO_DRIVE;

   f_sample_rate_factor = (float)VAR(AX0_u16_Pos_Loop_Tsp)/25000.0;
   

   if (BGVAR(u32_Pe_Filt_Fcenter) == 0)
   {
      u32_pe_fcenter = 400000;
      s32_pe_filt_gain = 0;
   }

   f_LR = (float)BGVAR(u16_Pe_Filt_Sharpness) / (float)u32_pe_fcenter;

   // RC = 1/(4*pi*pi*F_filter*F_filter*LR) = 0.02533029 / (F_filter*F_filter*LR)
   f_RC = 25330.29 / ((float)u32_pe_fcenter * (float)u32_pe_fcenter * f_LR);

   // LRdt = LR/dt
   VAR(AX0_s16_Pe_Filt_LRdt_Fix_Design) = (unsigned int)(f_LR * 4000.0)/f_sample_rate_factor;

   // RCdt = (2^14)*dt/RC
   VAR(AX0_s16_Pe_Filt_RCdt_Fix_Design) = (unsigned int)(f_sample_rate_factor * 4.096 / f_RC);

   f_pe_notch_out = -(float)VAR(AX0_s16_Pe_Filt_RCdt_Fix_Design)*(float)s32_pe_filt_gain/20000000.0;
   FloatToFix16Shift16(&VAR(AX0_s16_Pe_Filt_Out_Fix_Design), (unsigned int *)&VAR(AX0_s16_Pe_Filt_Out_Shr_Design), f_pe_notch_out);
   VAR(AX0_s16_Pe_Filt_Out_Shr_Design) +=12; // VG is mpy by the output and it has 12 shr

   // D = 1+LRdt+RCdt/2^14
   f_d = 1.0 + (float)VAR(AX0_s16_Pe_Filt_LRdt_Fix_Design) + (float)VAR(AX0_s16_Pe_Filt_RCdt_Fix_Design)/16384.0;
   f_d = 1 / f_d;

   FloatToFix16Shift16(&VAR(AX0_s16_Pe_Filt_D_Fix_Design), (unsigned int *)&VAR(AX0_u16_Pe_Filt_D_Shr_Design), f_d);
   VAR(AX0_s16_Pe_Filt_RCdt_Fix_Design) =-VAR(AX0_s16_Pe_Filt_RCdt_Fix_Design);
}


void NLPeAntiVib3Design(int drive)
{
   // AXIS_OFF;
   float f_LR, f_RC,f_d,f_sharp,f_freq,f_r_out,f_c_out,f_sample_rate_factor = (float)VAR(AX0_u16_Pos_Loop_Tsp)/25000.0;
   REFERENCE_TO_DRIVE;
   // PCMD RLc3 (notch) filter design

   //f_freq*1000 = NLANTIVIBHZ3*sqrt(1+NLANTIVIBGAIN3)
   //NLANTIVIBHZ3 and NLANTIVIBGAIN3 already multiplyed by 1000, therefore result is f_freq*1000
   f_freq  = (float)BGVAR(u32_Antivib3_Fcenter) * sqrt(1.0 + (float)BGVAR(s32_Antivib3_Gain)/1000.0);
   //f_sharp*1000 = NLANTIVIBSHARP3*sqrt(1+NLANTIVIBGAIN3)
   //NLANTIVIBHZ3 and NLANTIVIBGAIN3 already multiplyed by 1000, therefore result is f_sharp*1000
   f_sharp = (float)BGVAR(u16_Antivib3_Sharpness)* sqrt(1.0 + (float)BGVAR(s32_Antivib3_Gain)/1000.0);
   //f_sharp and f_freq both multiplyed by 1000, therefore no need scaling
   f_LR = f_sharp / f_freq;
   // RC = 1/(4*pi*pi*F_filter*F_filter*LR) = 0.02533029 / (F_filter*F_filter*LR)
   // 25330.29 =  0.02533029 * 1000000 (from f_freq*1000*f_freq*1000)
   f_RC = 25330.29 / (f_freq * f_freq * f_LR);

   // LRdt = LR/dt
   VAR(AX0_s16_Antivib3_LRdt_Fix_Design) = (int)(f_LR * 4000.0)/f_sample_rate_factor;

   // RCdt = (2^14)*dt/RC (multiplyed by 2^14)
   VAR(AX0_s16_Antivib3_RCdt_Fix_Design) = (int)(f_sample_rate_factor * 4.096 / f_RC);

   // D = 1+LRdt
   f_d = 1.0 + (float)VAR(AX0_s16_Antivib3_LRdt_Fix_Design);
   f_d = 1 / f_d;

   FloatToFix16Shift16(&VAR(AX0_s16_Antivib3_D_Fix_Design), (unsigned int *)&VAR(AX0_u16_Antivib3_D_Shr_Design), f_d);

   //R_Out = ANTIVIBQ/1000
   f_r_out = 0.001*(float)BGVAR(s32_Antivib3_Q_Gain_User);
   FloatToFix16Shift16(&VAR(AX0_s16_Antivib3_R_Out_Fix_Design), (unsigned int *)&VAR(AX0_u16_Antivib3_R_Out_Shr_Design), f_r_out);

   //C_out = 1 / (1+NLANTIVIBGAIN3)
   if (BGVAR(s32_Antivib3_Gain) == 0)
   {
      f_c_out = 0; // in this case r.t overrides the filter
      VAR(AX0_s16_Antivib3_C_Out_Inv_Fix) = 0;
      VAR(AX0_s16_Antivib3_C_Out_Inv_Shr) = 0;
   }
   else
   {
      f_c_out = 1.0 / (1.0 + (float)BGVAR(s32_Antivib3_Gain)/1000.0);
      FloatToFix16Shift16(&VAR(AX0_s16_Antivib3_C_Out_Inv_Fix), (unsigned int *)&VAR(AX0_s16_Antivib3_C_Out_Inv_Shr), 16384.0/f_c_out/(float)VAR(AX0_s16_Antivib3_RCdt_Fix_Design));
   }

   FloatToFix16Shift16(&VAR(AX0_s16_Antivib3_C_Out_Fix_Design), (unsigned int *)&VAR(AX0_u16_Antivib3_C_Out_Shr_Design), f_c_out*(float)VAR(AX0_s16_Antivib3_RCdt_Fix_Design));
   VAR(AX0_u16_Antivib3_C_Out_Shr_Design) += 14; // due to 14 shl in RCdt

   FloatToFix16Shift16(&VAR(AX0_s16_Antivib3_Vl_Fix_Design), (unsigned int *)&VAR(AX0_u16_Antivib3_Vl_Shr_Design), (float)VAR(AX0_s16_Antivib3_LRdt_Fix_Design));

   // find out_shr = min(AX0_u16_Antivib3_Vl_Shr_Design,AX0_u16_Antivib3_C_Out_Shr_Design,AX0_u16_Antivib3_R_Out_Shr_Design)
   VAR(AX0_u16_Antivib3_Out_Shr_Design) = VAR(AX0_u16_Antivib3_Vl_Shr_Design);
   if (VAR(AX0_u16_Antivib3_Out_Shr_Design) > VAR(AX0_u16_Antivib3_C_Out_Shr_Design)) VAR(AX0_u16_Antivib3_Out_Shr_Design) = VAR(AX0_u16_Antivib3_C_Out_Shr_Design);
   if (VAR(AX0_u16_Antivib3_Out_Shr_Design) > VAR(AX0_u16_Antivib3_R_Out_Shr_Design)) VAR(AX0_u16_Antivib3_Out_Shr_Design) = VAR(AX0_u16_Antivib3_R_Out_Shr_Design);

   VAR(AX0_u16_Antivib3_Vl_Shr_Design)-=VAR(AX0_u16_Antivib3_Out_Shr_Design);
   VAR(AX0_u16_Antivib3_C_Out_Shr_Design)-=VAR(AX0_u16_Antivib3_Out_Shr_Design);
   VAR(AX0_u16_Antivib3_R_Out_Shr_Design)-=VAR(AX0_u16_Antivib3_Out_Shr_Design);

   VAR(AX0_s16_Antivib3_RCdt_Fix_Design) = -VAR(AX0_s16_Antivib3_RCdt_Fix_Design);


   VAR(AX0_s16_I_Sat_Nl_Aw_Fix) = (int)((1.0 + 0.001*(float)BGVAR(s32_Antivib3_Gain))*2048.0);
}


int SalNAdaptiveVcmdGainfCommand(long long param, int drive)
{
   BGVAR(u32_Nl_Speed_Gain_User) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}


int SalPoscontrolModeCommand(long long param, int drive)
{
   // AXIS_OFF;
   int s16_temp = VAR(AX0_u16_Pos_Loop_Tsp);
/*   //unsupported SFBMODE=1 in non linear position
   if ((BGVAR(s16_SFBMode) == 1) && (param != 0LL))
         return SFBMODE_POSCTRL_UNSUPPORTED; */

   // For TSP_125_USEC, max moving average filter is 128 ms.
   // Reject POSCONTROLMODE of TSP_125_USEC if the moving average filter is set above 128 ms
   if ((BGVAR(u32_Move_Smooth_Avg) > 128000) && (param >= NON_LINEAR_POSITION_5))
         return BAD_POSCONTROLMODE_MOVESMOOTH;
         
   if (IS_DUAL_LOOP_ACTIVE && (param != NON_LINEAR_POSITION_5))
         return CONFIG_FAIL_DL_POSCONTROLMODE;

   if ( ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)) &&
        (param != 2LL) && (param != 5LL)                         ) return (NOT_AVAILABLE);

      VAR(AX0_u16_Pos_Control_Mode) = (int)param;
    if (VAR(AX0_u16_Pos_Control_Mode) >= NON_LINEAR_POSITION_5)
    {
      VAR(AX0_u16_Pos_Loop_Tsp) = TSP_125_USEC;
      BGVAR(u64_CAN_Max_Acc) = MAX_ACC_DEC_TSP125;
      BGVAR(u64_CAN_Max_Dec) = MAX_ACC_DEC_TSP125;
    }
    else
      VAR(AX0_u16_Pos_Loop_Tsp) = TSP_250_USEC;

//
   if (s16_temp != VAR(AX0_u16_Pos_Loop_Tsp))
   {

      if((u16_Init_State == 0) && (!BGVAR(u16_Is_Factory_Restore_Running)))// if INIT done - Bugzila 5015 - The value of ACC/DEC... was devided in 4 with each power cycle.
	  {
         PrePosControlModeCommand();

         if(VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)
         {
            BGVAR(u64_CAN_Max_Acc) = MAX_ACC_DEC_TSP125;
            BGVAR(u64_CAN_Max_Dec) = MAX_ACC_DEC_TSP125;
         }
         else
         {
            BGVAR(u64_CAN_Max_Acc) = MAX_ACC_DEC_TSP250;
            BGVAR(u64_CAN_Max_Dec) = MAX_ACC_DEC_TSP250;
         }
	  }

      ConvertInternalToVel1000OutLoop(drive);
      ConvertVelToInternalOutLoop(drive);
      ConvertInternalToAccDec1000ForPos(drive);
      ConvertAccDecToInternalForPos(drive);

      CalcFieldBusUserToIntern(drive);      
      CalcInternToFieldBusUser(drive);

	  SalDecStopTimeCommand(BGVAR(u16_DecStopTime), drive);
   
      if((u16_Init_State == 0) && (!BGVAR(u16_Is_Factory_Restore_Running)))// if INIT done - Bugzila 5015 - The value of ACC/DEC... was devided in 4 with each power cycle.
	  {
         PostPosControlModeCommand(0);
	  }

      DesignGearMsqFilter(drive);
      PtpFilterDesign(drive);
      UpdateAccDec(drive,ACC_UPDATE);
      UpdateAccDec(drive,DEC_UPDATE);
      SalMoveSmoothAvgNumCommand((long long)BGVAR(u32_Move_Smooth_Avg),drive);
      //UpdateFieldbusAccLimit(drive);
      //UpdateFieldbusDecLimit(drive);
      UpdateNCTVelLoopScale(drive);
   }

    PositionConfig(drive, COPY_POS_MASK);
    if (s16_temp != VAR(AX0_u16_Pos_Loop_Tsp)) PositionConfig(drive, 0); // support poscontromode mode change 4<->5 without power cycle

    IsLinearVelocityNeeded(drive);
    return SAL_SUCCESS;
}

void PrePosControlModeCommand()
{
   int i;

   //ACC
   BGVAR(u64_Temp_Acc_Rate) = MultS64ByFixS64ToS64(BGVAR(u64_AccRate),
                       BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                       BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);

   //DEC
   BGVAR(u64_Temp_Dec_Rate) = MultS64ByFixS64ToS64(BGVAR(u64_DecRate),
                       BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                       BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);

   //DECSTOP
   BGVAR(u64_Temp_Dec_Stop) = MultS64ByFixS64ToS64(BGVAR(u64_DecStop),
                       BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                       BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);

   //GEARACCTHRESH
   BGVAR(u64_Temp_Gear_Acc_Thresh) = MultS64ByFixS64ToS64(BGVAR(u64_Gear_Acc_Tresh),
                              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);
   //HOMEACC
   BGVAR(u64_Temp_Home_Acc_Rate)   = MultS64ByFixS64ToS64(BGVAR(u64_HomeAccRate),
                              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);

   //PATHACC & PATHDEC
   for(i=0;i<NUM_OF_PATHS;i++)
   {

      BGVAR(u64_Temp_Path_Acc_Rate)[i] = MultS64ByFixS64ToS64(BGVAR(u64_Path_Acceleration)[i],
                                      BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                      BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);

      BGVAR(u64_Temp_Path_Dec_Rate)[i] = MultS64ByFixS64ToS64(BGVAR(u64_Path_Deceleration)[i],
                                      BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                      BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);
   }
}

void PostPosControlModeCommand(int drive)
{
   unsigned long long u64_temp = 0LL;
   int i;

   //ACC
   u64_temp = MultS64ByFixS64ToS64(BGVAR(u64_Temp_Acc_Rate),
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_internal_fix,
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_internal_shr);

   SalAccCommand(u64_temp, drive);

   //DEC
   u64_temp = MultS64ByFixS64ToS64(BGVAR(u64_Temp_Dec_Rate),
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_internal_fix,
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_internal_shr);

   SalDecCommand(u64_temp, drive);

   //DECSTOP
   u64_temp = MultS64ByFixS64ToS64(BGVAR(u64_Temp_Dec_Stop),
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_internal_fix,
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_internal_shr);

   SalDecStopCommand(u64_temp, drive);

   //GEARACCTHRESH
   u64_temp = MultS64ByFixS64ToS64(BGVAR(u64_Temp_Gear_Acc_Thresh),
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_internal_fix,
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_internal_shr);

   SalGearAccTreshCommand(u64_temp, drive);

   //HOMEACC
   u64_temp = MultS64ByFixS64ToS64(BGVAR(u64_Temp_Home_Acc_Rate),
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_internal_fix,
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_internal_shr);

   SalHomeAccCommand(u64_temp, drive);

   //PATHACC & PATHDEC
   for(i=0;i<NUM_OF_PATHS;i++)
   {
      s64_Execution_Parameter[0] = i;

      s64_Execution_Parameter[1] = MultS64ByFixS64ToS64(BGVAR(u64_Temp_Path_Acc_Rate)[i],
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_internal_fix,
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_internal_shr);

      SalPathAccelerationCommand(drive);

      s64_Execution_Parameter[1] = MultS64ByFixS64ToS64(BGVAR(u64_Temp_Path_Dec_Rate)[i],
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_internal_fix,
              BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_internal_shr);

      SalPathDecelerationCommand(drive);
   }
}

int SalNlNotchBWCommand(long long param, int drive)
{
   BGVAR(s16_Nl_Notch_BW) = (int)param;
   PositionConfig(drive, 0);
   return SAL_SUCCESS;
}


int SalNlNotchCenterCommand(long long param, int drive)
{
   BGVAR(s16_Nl_Notch_Center) = (int)param;
   PositionConfig(drive, 0);
   return SAL_SUCCESS;
}


int SalNlNotch2BWCommand(long long param, int drive)
{
   BGVAR(s16_Nl_Notch2_BW) = (int)param;
   PositionConfig(drive, 0);
   return SAL_SUCCESS;
}


int SalNlNotch2CenterCommand(long long param, int drive)
{
   BGVAR(s16_Nl_Notch2_Center) = (int)param;
   PositionConfig(drive, 0);
   return SAL_SUCCESS;
}


int SalNlKpdCommand(long long param, int drive)
{
   BGVAR(u32_Nl_Kpd_User) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}


int SalNlKffSpringCommand(long long param, int drive)
{
   BGVAR(u32_Nl_KffSpring_User) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlKffSpringDecRatioCommand(long long param, int drive)
{
   BGVAR(u16_Nl_Dff_Ratio) = (unsigned int)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalKbffLpfCommand(long long param, int drive)
{
   BGVAR(s16_Kbff_Spring_LPF_User) = (int)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlKpgfCommand(long long param, int drive)
{
   // AXIS_OFF;
   BGVAR(u32_Nl_Kpgf_User) = (unsigned long)param;
   LVAR(AX0_u32_At_Control_Effort_Max) = 0;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlVffCommand(long long param, int drive)
{
   BGVAR(u16_Nl_Vff) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlKpgmaxCommand(int drive)
{
   long long param = s64_Execution_Parameter[0];
   // Allow "NLMAXGAIN = 1" if "NLMAXGAIN 1 1" was used
   if ((s16_Number_Of_Parameters > 1) && (s64_Execution_Parameter[1] == 1LL))
   {
        if ((param > 1000) && (param < 1600)) return (VALUE_OUT_OF_RANGE);
   }
   else
   {
        if (param < 1600) return (VALUE_OUT_OF_RANGE);
   }

   BGVAR(u32_Nl_Kpgmax) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlKpiCommand(long long param, int drive)
{
   if ((param == 0LL) && (Enabled(drive)) ) return (DRIVE_ACTIVE);
   BGVAR(u32_Nl_Kpi_User) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlKpivCommand(long long param, int drive)
{
   if ((param == 0LL) && (Enabled(drive)) ) return (DRIVE_ACTIVE);
   BGVAR(u32_Nl_Kpiv_User) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalKnlafrcCommand(long long param, int drive)
{
   BGVAR(u16_KNLAFF) = (unsigned int)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}




int SalNlKppCommand(long long param, int drive)
{
   BGVAR(u32_Nl_Kpp_User) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlKAntiVibrationCommand(long long param, int drive)
{
   BGVAR(u32_Nl_K_Anti_Vibration_User) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalKpdCommand(long long param, int drive)
{
   BGVAR(u32_Kpd) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalKppCommand(long long param, int drive)
{
   BGVAR(u32_Kpp) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalKpiCommand(long long param, int drive)
{
   BGVAR(u32_Kpi) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}


int SalNlOutFilter1Command(long long param, int drive)
{
   BGVAR(s16_Nl_Out_Filter_1_User) = (int)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlOutFilter2Command(long long param, int drive)
{
   BGVAR(s16_Nl_Out_Filter_2_User) = (int)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}


int SalKpiSatInCommand(long long param, int drive)
{
   // AXIS_OFF;

   if (param > 0x271000000000LL) // internal value for 10000 rev or 10000 pitch
   {
       return (VALUE_TOO_HIGH);
   }

   if (param < 0LL)
   {
       return (VALUE_TOO_LOW);
   }

   LVAR(AX0_u32_Sat_In_Lo_Design) = (unsigned long)(param & 0x00000000FFFFFFFF);
   LVAR(AX0_u32_Sat_In_Hi_Design) = (unsigned long)(param >> 32LL);

   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalKpiSatOutCommand(long long param, int drive)
{

   if (param > 5368709121LL) // internal value for 10000 rps
   {
      return (VALUE_TOO_HIGH);
   }

   if (param < 0LL)
    {
      return (VALUE_TOO_LOW);
   }

   BGVAR(u32_KpiSatOut) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalKpeCommand(long long param, int drive)
{
   BGVAR(u16_Kpe) = (unsigned int)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalKpafrCommand(long long param, int drive)
{
   BGVAR(s32_Kpafr) = (long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalKpafrToCurrentCommand(long long param, int drive)
{
   // AXIS_OFF;
   BGVAR(s32_Kpafrcurrent) = (long)param;
   PositionConfig(drive, 0);
   if (VAR(AX0_u16_Use_Linear_Vel) != 0) VelocityLoopDesign(drive);
   return (SAL_SUCCESS);
}

int SalKpvfrCommand(long long param, int drive)
{
   BGVAR(s32_Kpvfr) = (long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalKpvfr2Command(long long param, int drive)
{
   BGVAR(u32_Kpvfr2) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlKffSpringFilterCommand(long long param, int drive)
{
   BGVAR(s16_Nl_KffSpring_LPF) = (unsigned int)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlKAntiResonanceFcenterDivCommand(long long param, int drive)
{
   BGVAR(s32_Anti_Res_Hz_Div) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlKAntiResonanceFcenterCommand(long long param, int drive)
{
   BGVAR(u32_Nl_K_Anti_Resonance_Fcenter_User)  = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlKAntiResonanceSharpnessCommand(long long param, int drive)
{
   BGVAR(u16_Nl_K_Anti_Resonance_Sharp_User) = (unsigned int)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlPeFcenterCommand(long long param, int drive)
{
   BGVAR(u32_Pe_Filt_Fcenter_User) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlPe3FcenterCommand(long long param, int drive)
{
   BGVAR(u32_Antivib3_Fcenter_User) = (unsigned long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

int SalNlPeSharpnessCommand(long long param, int drive)
{
   BGVAR(u16_Pe_Filt_Sharpness_User) = (unsigned int)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}


int SalNlPe3SharpnessCommand(long long param, int drive)
{
   BGVAR(u16_Antivib3_Sharpness_User) = (unsigned int)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}


int SalNlPeGainCommand(long long param, int drive)
{
   BGVAR(s32_Pe_Filt_Gain_User) = (long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}


int SalNlPe3GainCommand(long long param, int drive)
{
   BGVAR(s32_Antivib3_Gain_User) = (long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}


int SalNlPe3QGainCommand(long long param, int drive)
{
   BGVAR(s32_Antivib3_Q_Gain_User) = (long)param;
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}


int SalNLTuneCommand(long long param, int drive)
{
   // AXIS_OFF;
   int i;

   if (param == 2) return (VALUE_OUT_OF_RANGE);

   if ((param == 1) && (VAR(AX0_u16_NL_Tune_Status) != 1) && Enabled(drive))
      return DRIVE_ACTIVE;

   if ((param == 1) && (VAR(AX0_u16_NL_Tune_Status) != 1))
   {
      if (Enabled(drive)) return (DRIVE_ACTIVE);
      else
      {
         VAR(AX0_s16_Pe_Sign_Prev) = 0;
         VAR(AX0_s16_Pe_Sign) = 0;
         VAR(AX0_u16_Zero_Crossing_Cntr) = 0;

         // Activate DB during stop
         BGVAR(u32_NlTune_Stored_Istop) = BGVAR(u32_Stop_Current);
         BGVAR(u16_NlTune_Stored_Dismode) = BGVAR(u16_Disable_Mode);
         SalIstopCommand((long long)BGVAR(s32_Drive_I_Peak), drive);
         SalDisableModeCommand((long long)2, drive);
      }
   }
   if ((param == 3) && (VAR(AX0_u16_NL_Tune_Status) != 3))
   {
      do {
         i = Cntr_3125;
         LLVAR(AX0_u32_TQF_Acc_Lo) = 0LL;
      } while (i != Cntr_3125);
      // 10 shr to match tqf scalings, and 1 shr to take 0.5 fb counts (including MENCRESSHR). Totally 11 shr.
      LVAR(AX0_s32_Tqf_Resolution) = LVAR(AX0_s32_Ptp_Filter_One_Feedback_Count) >> 11;
      LVAR(AX0_u32_VBL_Result_Lo) = 0L;
      LVAR(AX0_s32_VBL_Result_Hi) = 0L;
      LVAR(AX0_u32_VBL_Max) = 0L;
      VAR(AX0_s16_I_Derive_Sign) = 0;
      VAR(AX0_s16_I_Derive_Sign_Prev) = 0;
      LVAR(AX0_u32_PE_Extremum_Cntr) = 0;
      VAR(AX0_u16_Tqf_Flag) = 1;
      VAR(AX0_s16_I_First_Extremum) = 3;
      if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC) 
         LVAR(AX0_u32_NL_Tune_Settled_Cntr) = BGVAR(u32_NlTune_Timeout) << 3L;
      else 
      LVAR(AX0_u32_NL_Tune_Settled_Cntr) = BGVAR(u32_NlTune_Timeout) << 2L;

      for (i = 0; i < HISTOGRAM_SIZE; i++)
         BGVAR(s32_NlTune_Histogram)[i] = 0L;
      for (i = 0; i < HISTOGRAM_CYC_BUFF_SIZE; i++)
      {
         *(unsigned long*)((unsigned int)((long)&AX0_u32_NlTune_Histogram_Cyc_Buffer & 0xffff) + (i << 1)) = 0L;
      }

      BGVAR(u16_NlTune_Histogram_Cyc_Buffer_Read_Index) = 0;
      VAR(AX0_u16_NlTune_Histogram_Cyc_Buffer_Write_Index) = 0;

   }

   VAR(AX0_u16_NL_Tune_Status) = (int)param;

   return SAL_SUCCESS;
}


void NLTuneHanlder(int drive)
{
   // AXIS_OFF;
   unsigned long u32_temp1, u32_temp2, u32_temp3, u32_temp4;
   unsigned int u16_last_write_index = 0, u16_read_index = 0;
   long s32_actual_velocity = 0;

   if ((BGVAR(u16_NlTune_State_Prev) != 2) && (VAR(AX0_u16_NL_Tune_Status) == 2))
   {
      DisableCommand(drive);
      BGVAR(u16_NlTune_Hanlder_State) = 1;
   }

   if (!Enabled(drive)) VAR(AX0_u16_NL_StandStill_Disable) = 0;

   switch (BGVAR(u16_NlTune_Hanlder_State))
   {
      case 0:
      break;

      case 1: // Wait for motion to stop to restore original DISMODE and ISTOP
         s32_actual_velocity = LVAR(AX0_s32_Vel_Var_Fb_0);
         if (s32_actual_velocity < 0L) s32_actual_velocity = -s32_actual_velocity;
         if (s32_actual_velocity < 0x50000L)
         {
            SalDisableModeCommand((long long)BGVAR(u16_NlTune_Stored_Dismode), drive);
            SalIstopCommand((long long)BGVAR(u32_NlTune_Stored_Istop), drive);
            BGVAR(u16_NlTune_Hanlder_State) = 0;
         }
      break;
   }

   // Handle the Histogram
   // RT writes the values in a cyc-buffer, read them and update the histogram buffer
   if ((VAR(AX0_u16_NL_Tune_Status) == 4) || (VAR(AX0_u16_NL_Tune_Status) == 5))
   {
      u16_last_write_index = VAR(AX0_u16_NlTune_Histogram_Cyc_Buffer_Write_Index);
      u16_read_index = BGVAR(u16_NlTune_Histogram_Cyc_Buffer_Read_Index);

      while ( (u16_read_index != u16_last_write_index)                                   &&
              (((u16_read_index + 1) % HISTOGRAM_CYC_BUFF_SIZE) != u16_last_write_index) &&
              (((u16_read_index + 2) % HISTOGRAM_CYC_BUFF_SIZE) != u16_last_write_index) &&
              (((u16_read_index + 3) % HISTOGRAM_CYC_BUFF_SIZE) != u16_last_write_index)   )
      {
         u32_temp1 = *(unsigned long*)((unsigned int)((long)&AX0_u32_NlTune_Histogram_Cyc_Buffer  & 0xffff) + ((unsigned long)u16_read_index<<1));
         u32_temp2 = *(unsigned long*)((unsigned int)((long)&AX0_u32_NlTune_Histogram_Cyc_Buffer  & 0xffff) + (((unsigned long)((u16_read_index + 1) % HISTOGRAM_CYC_BUFF_SIZE)) << 1));
         u32_temp3 = *(unsigned long*)((unsigned int)((long)&AX0_u32_NlTune_Histogram_Cyc_Buffer  & 0xffff) + (((unsigned long)((u16_read_index + 2) % HISTOGRAM_CYC_BUFF_SIZE)) << 1));
         u32_temp4 = *(unsigned long*)((unsigned int)((long)&AX0_u32_NlTune_Histogram_Cyc_Buffer  & 0xffff) + (((unsigned long)((u16_read_index + 3) % HISTOGRAM_CYC_BUFF_SIZE)) << 1));
         
         u32_temp1 += (u32_temp2 + u32_temp3 + u32_temp4);

         u16_read_index++;
         if (u16_read_index >= HISTOGRAM_CYC_BUFF_SIZE)
            u16_read_index = 0;
         BGVAR(u16_NlTune_Histogram_Cyc_Buffer_Read_Index) = u16_read_index;

         u32_temp1 /= (MAX_TIME_DELTA / HISTOGRAM_SIZE);
         if (u32_temp1 >= HISTOGRAM_SIZE) u32_temp1 = HISTOGRAM_SIZE - 1;
         BGVAR(s32_NlTune_Histogram)[u32_temp1]++;
      }
   }

   BGVAR(u16_NlTune_State_Prev) = VAR(AX0_u16_NL_Tune_Status);
}

// 0 - Returns the Histogram
// 1 - Returns TQF value
// 2 - Returns VBL Value
// 3 - Returns Histogram sample depth (used to be 2 now 4)
int SalNLTuneStatusCommand(int drive)
{
   REFERENCE_TO_DRIVE;
   return NLTuneStatusCommand(drive,FB_NOT_IN_USE);
}

int NLTuneStatusCommand(int drive,int s16_fb_use)
{
   // AXIS_OFF;
   static int nltune_status_state = 0;
   unsigned int u16_temp = 0;
   int ret_val = SAL_SUCCESS;
   REFERENCE_TO_DRIVE;

   // Only if there's enough room at the terminal buffer,
   // then reloading the general_domain and printing it
   // to the terminal will be possible.
   if (s16_fb_use == FB_NOT_IN_USE)
   {
     if (OutputBufferLoader()==0) return SAL_NOT_FINISHED;
   }
   strcpy((char*)&u16_General_Domain[0],"\0");

   if ((s64_Execution_Parameter[0] < 0) || (s64_Execution_Parameter[0] > 4))
      return VALUE_OUT_OF_RANGE;

   if (s64_Execution_Parameter[0] == 0)
   {
      if (nltune_status_state == HISTOGRAM_SIZE)
      {
         nltune_status_state = 0;
         return SAL_SUCCESS;
      }

      u16_temp = MAX_TIME_DELTA/HISTOGRAM_SIZE;
      if (nltune_status_state == 0)
         strcpy((char*)&u16_General_Domain[0],"Histogram:\r\n");

      strcat((char*)&u16_General_Domain[0],UnsignedInt64ToAscii(u16_temp*nltune_status_state & 0x0FFFFLL));
      strcat((char*)&u16_General_Domain[0]," - ");

      if (nltune_status_state == (HISTOGRAM_SIZE - 1))
         strcat((char*)&u16_General_Domain[0],"...");
      else
         strcat((char*)&u16_General_Domain[0],UnsignedInt64ToAscii((u16_temp * (nltune_status_state + 1) - 1) & 0x0FFFFLL));
      strcat((char*)&u16_General_Domain[0]," : ");

      strcat((char*)&u16_General_Domain[0],UnsignedInt64ToAscii(BGVAR(s32_NlTune_Histogram)[nltune_status_state] & 0x00FFFFFFFFLL));
      strcat((char*)&u16_General_Domain[0],"\r\n");

      nltune_status_state++;
      ret_val = SAL_NOT_FINISHED;
   }
   else if (s64_Execution_Parameter[0] == 1)
   {
      strcpy((char*)&u16_General_Domain[0],"TQF = ");
      strcat((char*)&u16_General_Domain[0],UnsignedInt64ToAscii(LLVAR(AX0_u32_TQF_Acc_Lo)));

      strcat((char*)&u16_General_Domain[0],"\r\n");
      ret_val = SAL_SUCCESS;
   }
   else if (s64_Execution_Parameter[0] == 2)
   {
      strcpy((char*)&u16_General_Domain[0],"VBL = ");
      strcat((char*)&u16_General_Domain[0],UnsignedInt64ToAscii(*(long long*)(&LVAR(AX0_u32_VBL_Result_Lo))));
      strcat((char*)&u16_General_Domain[0],"\r\n");
      ret_val = SAL_SUCCESS;
   }
   else if (s64_Execution_Parameter[0] == 3)
   {
      strcpy((char*)&u16_General_Domain[0],"Histogram Sample Depth = ");
      strcat((char*)&u16_General_Domain[0],UnsignedInt64ToAscii(4));
      strcat((char*)&u16_General_Domain[0],"\r\n");
      ret_val = SAL_SUCCESS;
   }
   else if (s64_Execution_Parameter[0] == 4)
   {
      strcpy((char*)&u16_General_Domain[0],"Max VBL = ");
      strcat((char*)&u16_General_Domain[0],UnsignedInt64ToAscii(LVAR(AX0_u32_VBL_Max) & 0x00000000FFFFFFFFLL));
      strcat((char*)&u16_General_Domain[0],"\r\n");
      ret_val = SAL_SUCCESS;
   }

   if (s16_fb_use == FB_NOT_IN_USE)
   {
      OutputBufferLoader();
   }

   return ret_val;
}


int SalNegLimCommand (long long param, int drive)
{
   unsigned int u16_temp_time;
   // AXIS_OFF;

   if( ((BGVAR(u16_Units_Pos_Rotary) ==  ROT_COUNTS_UNITS) && (ROTARY_MOTOR == BGVAR(u16_MotorType))) ||
       ((BGVAR(u16_Units_Pos_Linear) ==  LIN_COUNTS_UNITS) && (LINEAR_MOTOR == BGVAR(u16_MotorType)))   )
   {
      if (BGVAR(u16_Is_Not_Integer_Flag))
         return VALUE_MUST_BE_INTEGER;
   }

   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Pos_Min_Lim_Lo) = param;
   } while (u16_temp_time != Cntr_3125);

   UpdateHystVal(drive);
   return SAL_SUCCESS;
}


int SalPosLimCommand (long long param, int drive)
{
   // AXIS_OFF;
   unsigned int u16_temp_time;

   if( ((BGVAR(u16_Units_Pos_Rotary) ==  ROT_COUNTS_UNITS) && (ROTARY_MOTOR == BGVAR(u16_MotorType))) ||
       ((BGVAR(u16_Units_Pos_Linear) ==  LIN_COUNTS_UNITS) && (LINEAR_MOTOR == BGVAR(u16_MotorType)))   )
   {
      if (BGVAR(u16_Is_Not_Integer_Flag))
         return VALUE_MUST_BE_INTEGER;
   }

   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Pos_Max_Lim_Lo) = param;
   } while (u16_temp_time != Cntr_3125);

   UpdateHystVal(drive);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalPosLimHystCommand
// Description:
//          This function is called in response to the POSLIMHYST command.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
int SalPosLimHystCommand (long long param, int drive)
{

   if( ((BGVAR(u16_Units_Pos_Rotary) ==  ROT_COUNTS_UNITS) && (ROTARY_MOTOR == BGVAR(u16_MotorType))) ||
       ((BGVAR(u16_Units_Pos_Linear) ==  LIN_COUNTS_UNITS) && (LINEAR_MOTOR == BGVAR(u16_MotorType)))   )
   {
      if (BGVAR(u16_Is_Not_Integer_Flag))
         return VALUE_MUST_BE_INTEGER;
   }

   if (param > 119304647) //10 degress
      return VALUE_TOO_HIGH;

   BGVAR(s64_Pos_Lim_Hyst) = param;
   UpdateHystVal(drive);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalPulsePosLimHystCommand
// Description:
//          This function is called in response to P8-65 write command.
//
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPulsePosLimHystCommand(long long param, int drive)
{
/* Need to be tested if required (limit checked in Xcel)
   if (param > 119304647) //10 degress
      return VALUE_TOO_HIGH;
*/
   BGVAR(s64_Pos_Lim_Hyst) = param;
   UpdateHystVal(drive);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: UpdateHystVal
// Description:
//          This function updates the MAX/MIN hysteresis limits.
//
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
void UpdateHystVal(int drive)
{
   unsigned int u16_temp_time;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Pos_Max_Hyst_Lo) = LLVAR(AX0_u32_Pos_Max_Lim_Lo) - BGVAR(s64_Pos_Lim_Hyst);
      LLVAR(AX0_u32_Pos_Min_Hyst_Lo) = LLVAR(AX0_u32_Pos_Min_Lim_Lo) + BGVAR(s64_Pos_Lim_Hyst);
   } while (u16_temp_time != Cntr_3125);
}


//**********************************************************
// Function Name: SalReadPUUPosLim
// Description:
//   This function returns the POSLIMPOS value in PUU units.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************

int SalReadPUUPosLim(long long *data,int drive)
{
   long long s64_temp;
   int s16_temp_time;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   //convert from CDHD internal position units into PUU.
      do {
         s16_temp_time = Cntr_3125;
         s64_temp = LLVAR(AX0_u32_Pos_Max_Lim_Lo);
      } while (s16_temp_time != Cntr_3125);
   *data = MultS64ByFixS64ToS64(s64_temp,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalWritePUUPosLim
// Description:
//   This function sets POSLIMPOS internal value from PUU units.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWritePUUPosLim(long long param,int drive)
{
   BGVAR(s32_PUU_SW_Pos_Limit) = (long)param;
   RecalculatePUUSWLim(drive);
   UpdateHystVal(drive);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadAxen
// Description:
//   This function returns P_P5-16_AXEN.
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SalReadAxen(long long *data,int drive)
{
   long long s64_temp;
   int s16_temp_time;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   do {
         s16_temp_time = Cntr_3125;
         s64_temp = LLVAR(AX0_u32_Pos_Fdbk_Offset_Lo);
      } while (s16_temp_time != Cntr_3125);

   *data = MultS64ByFixS64ToS64(s64_temp,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteAxenCommand
// Description:
//   This function write P_P5-16_AXEN.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteAxenCommand(long long param,int drive)
{
   if (Enabled(drive)) return (DRIVE_ACTIVE);

   BGVAR(s32_P5_16_AXEN) = (long) param;

   RecalculatePUUAXEN(drive);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: RecalculatePUUAXEN
// Description:
//   This function updates the internal PFB offset according to the new gear ratio (PUU).
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void RecalculatePUUAXEN(int drive)
{
   // AXIS_OFF;

   // reject the recalculation while the drive is enable to prevent jump (change of PFB while drive enable).
   if (Enabled(drive)) return;

   LLVAR(AX0_u32_Pos_Fdbk_Offset_Lo) = MultS64ByFixS64ToS64((long long)BGVAR(s32_P5_16_AXEN),
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr);

   // after a jump of pfb, the roll over modulation calculation does not work anymore
   // it is important to do the calculation from scratch again
   if ((BGVAR(s64_Modulo_Range) != 0) && (BGVAR(u16_Modulo_Mode) > 0))
     VAR(AX0_u16_Position_Modulo_Active) = 1;
}


//**********************************************************
// Function Name: SalReadPosLimHyst
// Description:
//   This function returns P_P8-65_POSLIMHYST.
//
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPosLimHyst(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   *data = BGVAR(s64_Pos_Lim_Hyst);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadPUUNegLim
// Description:
//   This function returns the POSLIMNEG value in PUU units.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPUUNegLim(long long *data,int drive)
{
   long long s64_temp;
   int s16_temp_time;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   //convert from CDHD internal position units into PUU.
      do {
         s16_temp_time = Cntr_3125;
         s64_temp = LLVAR(AX0_u32_Pos_Min_Lim_Lo);
      } while (s16_temp_time != Cntr_3125);
   *data = MultS64ByFixS64ToS64(s64_temp,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalWritePUUNegLim
// Description:
//   This function sets POSLIMNEG internal value from PUU units.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWritePUUNegLim(long long param,int drive)
{
   BGVAR(s32_PUU_SW_Neg_Limit) = (long)param;
   RecalculatePUUSWLim(drive);
   UpdateHystVal(drive);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: recalculatePUUSWLim
// Description:
//   This function recalculate POSLIMPOS and POSLIMNEG from last entered user value.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void RecalculatePUUSWLim(int drive)
{
   long long s64_temp1, s64_temp2;
   int s16_temp_time;
   // AXIS_OFF;

   //convert from PUU into CDHD internal position units and set it in the POSLIMPOS variable.
   s64_temp1 = MultS64ByFixS64ToS64((long long)BGVAR(s32_PUU_SW_Pos_Limit),
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr);

   //convert from PUU into CDHD internal position units and set it in the POSLIMNEG variable.
   s64_temp2 = MultS64ByFixS64ToS64((long long)BGVAR(s32_PUU_SW_Neg_Limit),
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr);
   do {
      s16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Pos_Max_Lim_Lo) = s64_temp1;
      LLVAR(AX0_u32_Pos_Min_Lim_Lo) = s64_temp2;
   } while (s16_temp_time != Cntr_3125);

   // Recalculate the LS hystVal to new gear ratio
   UpdateHystVal(drive);
}

int SalPosLimModeCommand (long long param, int drive)
{
   // AXIS_OFF;
   long s32_Timer_Start_Capture_1ms = Cntr_1mS;
   REFERENCE_TO_DRIVE;

   if ((param & 0x01) == 0LL)
   {
      VAR(AX0_u16_InMode_Update) = 0xFFFF;
      // Wait till the update register is cleared at RT
      while ((VAR(AX0_u16_InMode_Update) != 0) && !PassedTimeMS(1, s32_Timer_Start_Capture_1ms));

      BGVAR(u64_Sys_Warnings) &= (~CW_SW_LS_WRN_MASK & ~CCW_SW_LS_WRN_MASK & ~CW_CCW_SW_LS_WRN_MASK);
   }

   VAR(AX0_u16_SW_Pos_lim) = (unsigned int)(param & 0x07);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalLxmPosLimModeWriteCommand
// Description: writes the p5-13 value.
//              Note: p5-13 bit #1 is poslimmode bit #2, since in lxm26/28 poslimmode bit #1 is always set.
//              so p5-13 bits are as follows:
//              bit 0: 0 = Software position limits disabled, 1 = Software position limits enabled.
//              bit 1: 0 = Transient Limits are used for the Homing Process, 1 = Transient Limits are ignored during the Homing Process.
//
//              so bits 1 and 2 are swapped in this function before setting the internal poslimmode
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalLxmPosLimModeWriteCommand(long long param, int drive)
{
   long long s64_tmp = 0;

   s64_tmp = (long long)LxmPosLimModeConvert((unsigned int)param);

   // update the internal poslimmode.
   SalPosLimModeCommand(s64_tmp, drive);
   BGVAR(u16_P5_13_SW_Pos_lim) = param;
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: LxmPosLimModeConvert
// Description: convert the 3 bits of poslimmode to 2 bits of p5-13
//
//      Note: p5-13 bit #1 is poslimmode bit #2, since in lxm26/28 poslimmode bit #1 is always set.
//      so p5-13 bits are as follows:
//      bit 0: 0 = Software position limits disabled, 1 = Software position limits enabled.
//      bit 1: 0 = Transient Limits are used for the Homing Process, 1 = Transient Limits are
//                   ignored during the Homing Process.
//
//      so bits 1 and 2 are swapped in this function before setting the internal poslimmode
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
unsigned int LxmPosLimModeConvert(unsigned int u16_param)
{
   unsigned int u16_tmp;

   // set bit zero of poslimmode (bit zero in p5-13)
   u16_tmp = (u16_param & 1) ? 1 : 0;

   // set bit 1 of poslimmode (no bit in p5-13)
   u16_tmp |= 0x2;

   // set bit 2 of poslimmode (bit 1 in p5-13)
   if (u16_param & 2)
   {
      u16_tmp |= 0x4;
   }
   else
   {
      u16_tmp &= ~0x4;
   }

   return (u16_tmp);
}


void NLTorqueFilterDesign(int drive)
{
   // AXIS_OFF;
   float f_outf_in_coef, f_outf_mem1_coef, f_outf_mem2_coef;
   int s16_outf_in_fix, s16_outf_in_shr, s16_outf_mem1_fix, s16_outf_mem1_shr,
       s16_outf_mem2_fix, s16_outf_mem2_shr, s16_outf_out_shifts,f_damping_slope;
   REFERENCE_TO_DRIVE;

   f_outf_in_coef = 31.25 / (float)BGVAR(s16_Nl_Out_Filter_1);
   f_outf_mem1_coef = 1-f_outf_in_coef+(float)BGVAR(s16_Nl_Out_Filter_2) / 100.0;
   f_outf_mem2_coef = -(float)BGVAR(s16_Nl_Out_Filter_2) / 100.0;

   if (f_outf_in_coef > 1)
   {
      f_outf_in_coef = 1.0;
      f_outf_mem1_coef = 0.0;
      f_outf_mem2_coef = 0.0;
   }

   FloatToFix16Shift16(&s16_outf_in_fix, (unsigned int *)&s16_outf_in_shr, f_outf_in_coef);
   FloatToFix16Shift16(&s16_outf_mem1_fix, (unsigned int *)&s16_outf_mem1_shr, f_outf_mem1_coef);
   FloatToFix16Shift16(&s16_outf_mem2_fix, (unsigned int *)&s16_outf_mem2_shr, f_outf_mem2_coef);

   if (BGVAR(s16_Nl_Out_Filter_2) == 0) s16_outf_mem2_shr = 30;

   s16_outf_out_shifts = s16_outf_in_shr;
   if (s16_outf_mem1_shr < s16_outf_out_shifts)  s16_outf_out_shifts = s16_outf_mem1_shr;
   if (s16_outf_mem2_shr < s16_outf_out_shifts)  s16_outf_out_shifts = s16_outf_mem2_shr;
   if (s16_outf_out_shifts > 15)  s16_outf_out_shifts = 15;

   s16_outf_in_shr -= s16_outf_out_shifts;
   s16_outf_mem1_shr -= s16_outf_out_shifts;
   s16_outf_mem2_shr -= s16_outf_out_shifts;

   VAR(AX0_s16_Crrnt_Cmnd_In_Coef_Design) = s16_outf_in_fix >> s16_outf_in_shr;
   VAR(AX0_s16_Crrnt_Cmnd_Mem_1_Coef_Design) = s16_outf_mem1_fix >> s16_outf_mem1_shr;
   VAR(AX0_s16_Crrnt_Cmnd_Mem_2_Coef_Design) = s16_outf_mem2_fix >> s16_outf_mem2_shr;
   VAR(AX0_s16_Crrnt_Cmnd_Out_Shr_Coef_Design) = s16_outf_out_shifts;
   VAR(AX0_s16_Crrnt_Cmnd_Residue_Mask_Design) = (1 << s16_outf_out_shifts) - 1;

   if (f_outf_in_coef > 1)
      VAR(AX0_s16_Crrnt_Cmnd_Damping_Slope_Design) = 0;
   else
   {
      // Prevent divide by zero
      if(VAR(AX0_s16_Variable_Gain_Max_Design) == 4096)
        f_damping_slope = 0;
      else
        f_damping_slope = 2048.0*(float)VAR(AX0_s16_Crrnt_Cmnd_Mem_2_Coef_Design) /((float)VAR(AX0_s16_Variable_Gain_Max_Design) - 4096.0);
      VAR(AX0_s16_Crrnt_Cmnd_Damping_Slope_Design) = (int)f_damping_slope;
      VAR(AX0_s16_Crrnt_Cmnd_Mem_1_Coef_Design) +=VAR(AX0_s16_Crrnt_Cmnd_Mem_2_Coef_Design);
   }
}


void NLNotchFilterDesign(int drive)
{
   // AXIS_OFF;
   float f_notch_b0, f_notch_a1, f_notch_a2, f_sigma,f_sample_rate_factor = (float)VAR(AX0_u16_Pos_Loop_Tsp)/25000.0;
   int s16_notch_b0_fix, s16_notch_b0_shr, s16_notch_a1_fix, s16_notch_a1_shr, s16_notch_a2_fix, s16_notch_a2_shr, s16_filter_shr, s16_notch_b2_fix;
   int s16_notch_center, s16_notch_bw, s16_filter_number, s32_sigma, s16_temp_shr, s16_round;
   REFERENCE_TO_DRIVE;

   // notch filter design
   for (s16_filter_number = 1; s16_filter_number <= 2; s16_filter_number++)
   {
      // read specific input parameters
      if (s16_filter_number == 1)
      {
         s16_notch_center = BGVAR(s16_Nl_Notch_Center);
         s16_notch_bw = BGVAR(s16_Nl_Notch_BW);
      }
      else if (s16_filter_number == 2)
      {
         s16_notch_center = BGVAR(s16_Nl_Notch2_Center);
         s16_notch_bw = BGVAR(s16_Nl_Notch2_BW);
      }

      // common design
      if ( (s16_notch_bw == 0) || (s16_notch_center == 0) )
      {
         s16_notch_b0_fix = 0x4000;
         s16_notch_b2_fix = 0;
         s16_notch_a1_fix = 0;
         s16_notch_a2_fix = 0;
         s16_filter_shr = 14;
      }
      else
      {
         f_notch_b0 = 1 / (1 + tan(3.14159265 * (float)s16_notch_bw * 0.00003125 * 8.0 * f_sample_rate_factor ));                  // bw, b2 = b0
         f_notch_a1 = f_notch_b0 * 2 * cos(2 * 3.14159265 * (float)s16_notch_center * 0.00003125 * 8.0 * f_sample_rate_factor );   //center, b1 = -a1
         f_notch_a2 = -(2 * f_notch_b0 - 1);

         FloatToFix16Shift16(&s16_notch_b0_fix, (unsigned int *)&s16_notch_b0_shr, f_notch_b0);   // b2 = b0
         FloatToFix16Shift16(&s16_notch_a1_fix, (unsigned int *)&s16_notch_a1_shr, f_notch_a1);   // a1 = -b1
         FloatToFix16Shift16(&s16_notch_a2_fix, (unsigned int *)&s16_notch_a2_shr, f_notch_a2);   // a2

         // Use the same shr for A and B in order to match the DC gain. Find min shr.
         s16_filter_shr = s16_notch_b0_shr;
         if (s16_notch_a1_shr < s16_filter_shr) s16_filter_shr = s16_notch_a1_shr;
         if (s16_notch_a2_shr < s16_filter_shr) s16_filter_shr = s16_notch_a2_shr;

         // Calc sigma(b) (by definition sigma(a) equals sigma(b)).
         // Multiply by min shr, and make sure that the value is at least 1.
         // Since (b2 = b0) and (b1 = - a1), then sigma = (b0 + b1 + b2) = (b0 - a1 + b0)
         f_sigma = (f_notch_b0 - f_notch_a1 + f_notch_b0) * (float)((long)1 << s16_filter_shr) + 0.5;      // 0.5 is for rounding
         s32_sigma = (long)f_sigma;
         if (s32_sigma == 0) s32_sigma = 1;

         s16_temp_shr = s16_notch_b0_shr - s16_filter_shr;
         if (s16_temp_shr > 0)
         {
            s16_round = 1 << (s16_temp_shr - 1);
            s16_notch_b0_fix = (s16_notch_b0_fix + s16_round) >> s16_temp_shr;
         }
         s16_notch_b2_fix = s16_notch_b0_fix;

         // Note that A coefficients in the differential equation of the filter are minus the A coefficients of the transfer function.
         // A coefficients in this function are of the differential equation of the filter.
         s16_notch_a1_fix = -(int)(s32_sigma - (long)s16_notch_b0_fix - (long)s16_notch_b0_fix);
         s16_notch_a2_fix = -(int)(s32_sigma - ((long)1 << s16_filter_shr) + (long)s16_notch_a1_fix);
      }

      // write specific output parameters
      if (s16_filter_number == 1)
      {
         VAR(AX0_s16_NL_Vf_Coef_B0_Design) = s16_notch_b0_fix;
         VAR(AX0_s16_NL_Vf_Coef_B1_Design) = -s16_notch_a1_fix;
         VAR(AX0_s16_NL_Vf_Coef_B2_Design) = s16_notch_b2_fix;
         VAR(AX0_s16_NL_Vf_Coef_A1_Design) = s16_notch_a1_fix;
         VAR(AX0_s16_NL_Vf_Coef_A2_Design) = s16_notch_a2_fix;
         VAR(AX0_s16_NL_Vf_Coef_Out_Shr_Design) = s16_filter_shr;
         VAR(AX0_s16_NL_Vf_Coef_A_Shr_Design) = s16_filter_shr;
      }
      else if (s16_filter_number == 2)
      {
         VAR(AX0_s16_NL_Vf2_Coef_B0_Design) = s16_notch_b0_fix;
         VAR(AX0_s16_NL_Vf2_Coef_B1_Design) = -s16_notch_a1_fix;
         VAR(AX0_s16_NL_Vf2_Coef_B2_Design) = s16_notch_b2_fix;
         VAR(AX0_s16_NL_Vf2_Coef_A1_Design) = s16_notch_a1_fix;
         VAR(AX0_s16_NL_Vf2_Coef_A2_Design) = s16_notch_a2_fix;
         VAR(AX0_s16_NL_Vf2_Coef_Out_Shr_Design) = s16_filter_shr;
         VAR(AX0_s16_NL_Vf2_Coef_A_Shr_Design) = s16_filter_shr;
      }
   }

}


void NLAntiResFilter(int drive)
{
   // AXIS_OFF;
   float f_DC, f_LR, f_RC, f_d, f_current_to_torque, f_inertia_factor_anti_vib, f_anti_vib;
   float f_sample_rate_factor = (float)VAR(AX0_u16_Pos_Loop_Tsp)/25000.0;
   REFERENCE_TO_DRIVE;

      // Anti-Vibration
   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      f_inertia_factor_anti_vib = ((float)BGVAR(s32_Motor_J) / 1000000.0) * ((float)BGVAR(u32_LMJR_Resonance) / 1000.0);
      // Scale to dipeak/1000/0x6666*mkt/1000
      f_current_to_torque = ((float)BGVAR(s32_Drive_I_Peak) / 1000.0) * ((float)BGVAR(u32_Motor_Kt) / 1000.0) * 3.81475547e-5;
   }
   else//Linear
   {
      f_inertia_factor_anti_vib = ((float)BGVAR(s32_Motor_Mass) / 1000.0) * ((float)BGVAR(u32_LMJR_Resonance) / 1000.0) * ((float)BGVAR(u32_Mpitch) / 1000000.0) * ((float)BGVAR(u32_Mpitch) / 1000000.0) / 39.4784176;//mpitch changed to decimal
      // Scale to dipeak/1000/0x6666*mkf/1000*pitch/10000/(2*PI) // pitch here should me in [m] that is why we divide by 1000
      f_current_to_torque = ((float)BGVAR(s32_Drive_I_Peak) / 1000.0) * ((float)BGVAR(u32_Motor_Kf) / 1000.0) * ((float)BGVAR(u32_Mpitch) / 1000000.0) * 6.071371904e-6;//mpitch changed to decimal
   }

   // Scale to 4000^2*2pi/2^32 = 0.023406689
   f_inertia_factor_anti_vib *= (0.023406689/f_sample_rate_factor/f_sample_rate_factor);
   // This is just to increase resolution
   f_inertia_factor_anti_vib *= RESOLUTION_INCREASE;
   FloatToFix16Shift16(&VAR(AX0_s16_Acc_To_Torque_Fix_Design), (unsigned int *)&VAR(AX0_s16_Acc_To_Torque_Shr_Design), f_inertia_factor_anti_vib);

   // Scale to: 2^32/2pi (/ 1000) = 683565275.576
   f_anti_vib = ((float)BGVAR(u32_Nl_K_Anti_Vibration) / 1000000.0) * (683565275.576/RESOLUTION_INCREASE);

   // Scale down as we increased resolution to the input terms
   //f_anti_vib /= RESOLUTION_INCREASE;
   FloatToFixS32Shift16(&LVAR(AX0_s32_nlp_K_Anti_Vib_Fix_Design), (unsigned int *)&VAR(AX0_s16_nlp_K_Anti_Vib_Shr_Design), f_anti_vib);

   // This is just to increase resolution
   f_current_to_torque *= RESOLUTION_INCREASE;
   FloatToFix16Shift16((int *)&VAR(AX0_s16_Current_To_Torque_Fix_Design), (unsigned int *)&VAR(AX0_s16_Current_To_Torque_Shr_Design), f_current_to_torque);
   // Make sure no overflow in shr value
   if (VAR(AX0_s16_Current_To_Torque_Shr_Design) > 31)
   {
      VAR(AX0_s16_Current_To_Torque_Fix_Design) >>= (VAR(AX0_s16_Current_To_Torque_Shr_Design) - 31);
      VAR(AX0_s16_Current_To_Torque_Shr_Design) = 31;
   }

   // Anti Resonant filter
   f_LR = (float)BGVAR(u16_Nl_K_Anti_Resonance_Sharpness) / (float)BGVAR(u32_Nl_K_Anti_Resonance_Fcenter);
   // RC = 1/(4*pi*pi*F_filter*F_filter*LR) = 0.02533029 / (F_filter*F_filter*LR)
   f_RC = 25330.29 / ((float)BGVAR(u32_Nl_K_Anti_Resonance_Fcenter) * (float)BGVAR(u32_Nl_K_Anti_Resonance_Fcenter) * f_LR);
   // LRdt = LR/dt
   VAR(AX0_s16_Anti_Resonant_LRdt_Fix_Design) = (unsigned int)(f_LR * 4000.0)/f_sample_rate_factor;
   // RCdt = (2^14)*dt/RC
   VAR(AX0_s16_Anti_Resonant_RCdt_Fix_Design) = (unsigned int)(f_sample_rate_factor * 4.096 / f_RC);

   // D = 1+LRdt+RCdt/2^14
   f_d = (float)((unsigned int)1 + VAR(AX0_s16_Anti_Resonant_LRdt_Fix_Design) + (unsigned int)(VAR(AX0_s16_Anti_Resonant_RCdt_Fix_Design) / 16384));
   f_d = 1 / f_d;
   FloatToFix16Shift16(&VAR(AX0_s16_Anti_Resonant_D_Fix_Design),&VAR(AX0_u16_Anti_Resonant_D_Shr_Design), f_d);
   // Create mask for Residue
   if (VAR(AX0_u16_Anti_Resonant_D_Shr_Design) < 32)
   {
      LVAR(AX0_u32_Anti_Resonant_Mask_Lo_Design) = (1L << (long)VAR(AX0_u16_Anti_Resonant_D_Shr_Design)) - 1L;
      LVAR(AX0_s32_Anti_Resonant_Mask_Hi_Design) = 0L;
   }
   else
   {
      LVAR(AX0_u32_Anti_Resonant_Mask_Lo_Design) = 0xFFFFFFFF;
      LVAR(AX0_s32_Anti_Resonant_Mask_Hi_Design) = (1L << (long)(VAR(AX0_u16_Anti_Resonant_D_Shr_Design) - 32L)) - 1L;
   }

   // This is for the DC handling
   // dt * F_filter
   f_DC = (float)BGVAR(u32_Nl_K_Anti_Resonance_Fcenter) * (float)BGVAR(s32_Anti_Res_Hz_Div) / 1000000.0 / 4000.0;
   FloatToFix16Shift16(&VAR(AX0_s16_Anti_Resonant_DC_Fix_Design), (unsigned int *)&VAR(AX0_u16_Anti_Resonant_DC_Shr_Design), f_DC);
   FloatToFix16Shift16(&VAR(AX0_s16_Anti_Resonant_DC_Output_Fix_Design), (unsigned int *)&VAR(AX0_u16_Anti_Resonant_DC_Output_Shr_Design),f_sample_rate_factor * 4.096 / f_RC);
   VAR(AX0_u16_Anti_Resonant_DC_Output_Shr_Design) += 14; // To compensate for the 2^14 factor in the fix

   if (VAR(AX0_u16_Anti_Resonant_DC_Shr_Design) < 32)
   {
      LVAR(AX0_u32_Anti_Resonant_DC_Mask_Lo_Design) = (1L << (long)VAR(AX0_u16_Anti_Resonant_DC_Shr_Design)) - 1L;
      LVAR(AX0_s32_Anti_Resonant_DC_Mask_Hi_Design) = 0L;
   }
   else
   {
      LVAR(AX0_u32_Anti_Resonant_DC_Mask_Lo_Design) = 0xFFFFFFFF;
      LVAR(AX0_s32_Anti_Resonant_DC_Mask_Hi_Design) = (1L << (long)(VAR(AX0_u16_Anti_Resonant_DC_Shr_Design) - 32L)) - 1L;
   }

   if (VAR(AX0_u16_Anti_Resonant_DC_Output_Shr_Design) < 32)
   {
      LVAR(AX0_u32_Anti_Resonant_DC_Output_Mask_Lo_Design) = (1L << (long)VAR(AX0_u16_Anti_Resonant_DC_Output_Shr_Design)) - 1L;
      LVAR(AX0_s32_Anti_Resonant_DC_Output_Mask_Hi_Design) = 0L;
   }
   else
   {
      LVAR(AX0_u32_Anti_Resonant_DC_Output_Mask_Lo_Design) = 0xFFFFFFFF;
      LVAR(AX0_s32_Anti_Resonant_DC_Output_Mask_Hi_Design) = (1L << (long)(VAR(AX0_u16_Anti_Resonant_DC_Output_Shr_Design) - 32L)) - 1L;
   }
}


//**********************************************************
// Function Name: SalReadPathPositionCommand
// Description:
//    This function returns the selected path index position value
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPathPositionCommand(long long *data, int drive)
{
   long long index = (s64_Execution_Parameter[0]+1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;

   *data = (BGVAR(s64_Path_Position)[index]);

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadHomeDefValCommand
// Description:
//    This function returns the Home defenition value (P6-00)
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadHomeDefValCommand(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   *data = BGVAR(s64_Path_Position_PUU)[0];
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalHomeDefValCommand
// Description:
//    This function sets the Home defenition value (P6-00)
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalHomeDefValCommand(long long param,int drive)
{
   if (param < (long long)LXM28_PUU_MIN) return (VALUE_TOO_LOW);
   if (param > (long long)LXM28_PUU_MAX) return (VALUE_TOO_HIGH);

   (BGVAR(s64_Path_Position_PUU)[0]) = param;

    RculateSchneidrPUUPositionToGearRatio(drive, 0);
    BGVAR(s64_Home_Offset) = BGVAR(s64_Path_Position)[0];

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadPParameterPathPositionCommand
// Description:
//    This function returns the selected path index position value for P parameters
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPParameterPathPositionCommand(long long *data, int drive)
{
// index +1 to skip the first array parameter which is homing parameter (the first path is in index 1 in the array)
   long long index = (s64_Execution_Parameter[0] + 1LL);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;

   *data = BGVAR(s64_Path_Position_PUU)[index];
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalPathPositionCommand
// Description:
//    This function sets the position value for the selected path index
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPathPositionCommand(int drive)
{
   long long index = (s64_Execution_Parameter[0]+1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;

   (BGVAR(s64_Path_Position)[index]) = s64_Execution_Parameter[1];

   if((u16_Product != SHNDR) && (u16_Product != SHNDR_HW))
   {// if not Schneider drive, update the PUU even if the PATHPOS is updated from serial command
       (BGVAR(s64_Path_Position_PUU)[index]) = MultS64ByFixS64ToS64((BGVAR(s64_Path_Position)[index]),
                           BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                           BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);
   }

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalPParameterPathPositionCommand
// Description:
//    This function sets the position value for the selected path index from P parameter
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPParameterPathPositionCommand(int drive)
{
// index +1 to skip the first array parameter which is homing parameter (the first path is in index 1 in the array)
   long long index = (s64_Execution_Parameter[0] + 1LL), value = s64_Execution_Parameter[1];

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;
   if (value < (long long)LXM28_PUU_MIN) return (VALUE_TOO_LOW);
   if (value > (long long)LXM28_PUU_MAX) return (VALUE_TOO_HIGH);

   (BGVAR(s64_Path_Position_PUU)[index]) = value;

   RculateSchneidrPUUPositionToGearRatio(drive, index);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadPathAccelerationCommand
// Description:
//    This function returns the selected path index acceleration value
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPathAccelerationCommand(long long *data, int drive)
{
   long long index = (s64_Execution_Parameter[0]+1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;

   *data = (long long)(BGVAR(u64_Path_Acceleration)[index]);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalPathAccelerationCommand
// Description:
//    This function sets the acceleration value for the selected path index
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPathAccelerationCommand(int drive)
{
   int ret_val = 0;
   long long index = (s64_Execution_Parameter[0]+1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;
   ret_val = CheckAccDecLimits(s64_Execution_Parameter[1]);
   if(ret_val != SAL_SUCCESS)
   {
       return ret_val;
   }
   (BGVAR(u64_Path_Acceleration)[index]) = (unsigned long long)s64_Execution_Parameter[1];
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadPathDecelerationCommand
// Description:
//    This function returns the selected path index deceleration value
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPathDecelerationCommand(long long *data, int drive)
{
   long long index = (s64_Execution_Parameter[0]+1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;
   *data = (long long)(BGVAR(u64_Path_Deceleration)[index]);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalPathAccelerationCommand
// Description:
//    This function sets the deceleration value for the selected path index
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPathDecelerationCommand(int drive)
{
   int ret_val;
   long long index = (s64_Execution_Parameter[0]+1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;
   ret_val = CheckAccDecLimits(s64_Execution_Parameter[1]);
   if(ret_val != SAL_SUCCESS)
       return ret_val;
   BGVAR(u64_Path_Deceleration)[index] = (unsigned long long)s64_Execution_Parameter[1];
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadPathSpeedCommand
// Description:
//    This function returns the selected path index speed value
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPathSpeedCommand(long long *data, int drive)
{
   long long index = (s64_Execution_Parameter[0]+1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;

   *data = (long long)(BGVAR(u32_Path_Speed)[index]);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalPathSpeedCommand
// Description:
//    This function sets the speed value for the selected path index
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPathSpeedCommand(int drive)
{
   long long index = (s64_Execution_Parameter[0]+1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;
   if (s64_Execution_Parameter[1] > (long long)BGVAR(s32_V_Lim_Design))
      return VALUE_TOO_HIGH;
   if (s64_Execution_Parameter[1] < -(long long)BGVAR(s32_V_Lim_Design))
      return VALUE_TOO_LOW;

   (BGVAR(u32_Path_Speed)[index]) = (unsigned long)s64_Execution_Parameter[1];
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadPathDelayCommand
// Description:
//    This function returns the selected path index delay value
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPathDelayCommand(long long *data, int drive)
{
   long long index = (s64_Execution_Parameter[0]+1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;

   *data = (long long)(BGVAR(u16_Path_Delay)[index]);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalPathDelayCommand
// Description:
//    This function sets the delay value for the selected path index
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPathDelayCommand(int drive)
{
   long long index = (s64_Execution_Parameter[0]+1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS))
      return VALUE_OUT_OF_RANGE;
   if((s64_Execution_Parameter[1] < 0)||(s64_Execution_Parameter[1] > 32767))
      return VALUE_OUT_OF_RANGE;

   BGVAR(u16_Path_Delay)[index] = (unsigned int)s64_Execution_Parameter[1];
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadPathControlCommand
// Description:
//    This function returns the selected path control ascii commands (CDHD)
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPathControlCommand(long long *data, int drive)
{
   long long index = (s64_Execution_Parameter[0]+1);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;

   *data = (long long)(BGVAR(u16_Path_Control)[index]);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadP_PathControlCommand
// Description:
//    This function returns the selected path control (command) for P parameters (P6-03 - P6-65)
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadP_PathControlCommand(long long *data, int drive)
{
//removed -> index +1 to skip the first array parameter which is homing parameter (the first path is in index 1 in the array)
   return SalReadPathControlCommand(data, drive);
}


//**********************************************************
// Function Name: SalPathControlCommand
// Description:
//    This function sets the control for the selected path index using ascii commands (CDHD)
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPathControlCommand(int drive)
{
   // index +1 to skip the first array parameter which is homing parameter (the first path is in index 1 in the array)
   long long index = (s64_Execution_Parameter[0]+1);
   unsigned int u16_value = (unsigned int)s64_Execution_Parameter[1];

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((index < 1LL) || (index >= (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;
   if((u16_value != 0) &&                            // ABS move with no insertion.
      (u16_value != 16) &&                           // ABS move with insertion.
      (u16_value != 128) &&                          // INC move with no inseration.
      (u16_value != 144) &&                          // INC move with insertion.
      (u16_value != 208))                            // Relative move with inseration(inc from current position).
      {
            return VALUE_OUT_OF_RANGE;
      }

   (BGVAR(u16_Path_Control)[index]) = u16_value;

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalP_PathControlCommand
// Description:
//    This function sets the control for the selected path index using P parameters(P6-03 - P6-65)
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalP_PathControlCommand(int drive)
{
   //removed -> index +1 to skip the first array parameter which is homing parameter (the first path is in index 1 in the array)
   return SalPathControlCommand(drive);
}


//**********************************************************
// Function Name: SalReadHomeDefCommand
// Description:
//    This function return the value of P6-01
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadHomeDefCommand(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   (BGVAR(u16_Path_Control)[0]) &= 0xFF00; // clear the low 8 bits of P6-01
   (BGVAR(u16_Path_Control)[0]) |= BGVAR(u8_Auto_Home_Mode);// write the low 8 bits of P6-01 with the current auto home mode (maybe updated from ASCII command)
   *data = (long long)(BGVAR(u16_Path_Control)[0]);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalHomeDefCommand (P6-01)
// Description:
//    This function sets the Homing Mode (bits 0 - 7) and
//    next Path to preform when homing is completed (Bits 8 - 15)
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalHomeDefCommand(long long param,int drive)
{
   int s16_home_mode = (int)(param & 0x00FF);
   int s16_next_path = (int)((param >> 8) & 0x00FF) ;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if((s16_home_mode != 0)&&(s16_home_mode != 1)) return VALUE_OUT_OF_RANGE;

   if(s16_next_path < 0)  return VALUE_TOO_LOW;
   if(s16_next_path > 32) return VALUE_TOO_HIGH;

   (BGVAR(u16_Path_Control)[0]) = (unsigned int) param;
   BGVAR(u8_Auto_Home_Mode) = (unsigned char) s16_home_mode;

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: PathHandler
// Description:
//    This Handler call from the BG and deals with the paths at execution time
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void PathHandler(int drive)
{
   // AXIS_OFF;
   int s16_local_index = -1, s16_execute_move_result = SAL_NOT_FINISHED,s16_temp_time;
   long long s64_target_position = 0LL,s64_actual_pos = 0LL;
   int s16_transition_mode;
   long s32_vcmd;

   if (VAR(AX0_s16_Opmode) != 8) return; // valid only in PS mode

   // check if there is an internal PATH or home command execution (PRCM P5-07)
   s16_local_index = checkForInternalPathExecution(drive);

   // check if there is a trigger PATH execution
   if (BGVAR(s16_Path_Triggered))
   {
      s16_local_index = BGVAR(s16_Selected_Path_Index);
   }

   if(s16_local_index != -1) // check if the sTate machine is in delay waiting mode and
   { // trigger performed.  If the new path is INS perfrom it immediately!
      if (s16_local_index == 0) // If Homing Path...
      {
         if(BGVAR(u16_Path_Exec_State) == PATH_STATE_MACHINE_IDLE)
          // reset the state machine and preform the Homing path immidatly
            BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_HOMING_EXECUTE;
         else // the Homing path will wait for the current path to finish
         { // save the next path index to run after running path done
            BGVAR(s16_Next_Path_Index) = s16_local_index; // save to perform later
            BGVAR(s16_Path_Triggered) = 0;
         }
      }
      else
      {
         if(BGVAR(u16_Path_Exec_State) == PATH_STATE_MACHINE_IDLE)
         { // reset the state machine and preform this new path immidatly
            BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_EXECUTE;
         }
         // if insertion or overlap to next path (no stop in position) (BIT 4- insertion) OR if AutoR immidiate execution is required
         else if( ( ( (BGVAR(u16_Path_Control)[s16_local_index]) & 0x10) == 0x10) || (BGVAR(u16_AutoR_Immidiate_Path_Exec) == 1))
         {// check if the selected path INS is active interupt the current running path
            BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_EXECUTE; //reset the state machine and preform this new path immidatly
            BGVAR(s16_Next_Path_Index) = NO_WAITING_PATH; // there is no waiting path
         }
         else // the new path is waiting to current path to stop
         {// save the next path index to run after running path done
            BGVAR(s16_Next_Path_Index) = s16_local_index; // save the next path to preform later
            BGVAR(s16_Path_Triggered) = 0;
         }
      }
   }
   else if ( (BGVAR(s16_Next_Path_Index) != NO_WAITING_PATH)        &&
             (BGVAR(u16_Path_Exec_State) == PATH_STATE_MACHINE_IDLE)  )
   // not triggered and in idle state, check if there is a waiting Path
   { // perform the waiting Path
      s16_local_index = BGVAR(s16_Next_Path_Index);
      BGVAR(s16_Next_Path_Index) = NO_WAITING_PATH;
      if (s16_local_index == 0) // If Homing Path...
         BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_HOMING_EXECUTE;
      else
         BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_EXECUTE;
   }

   s64_target_position = (BGVAR(s64_Path_Position)[s16_local_index]);

   switch(BGVAR(u16_Path_Exec_State))
   {
      case PATH_STATE_MACHINE_IDLE:// idle
      break;

      case PATH_STATE_MACHINE_EXECUTE:// trigger -> perform position
     {
         BGVAR(s16_Path_Triggered) = 0;
         BGVAR(s16_Running_Path_Index) = s16_local_index;

         BGVAR(u16_Path_Internal_Execution_Read_Value) = (BGVAR(s16_Running_Path_Index));
         // indicate the path number to perform
         BGVAR(u64_Temp_Acc) = BGVAR(u64_AccRate);
         BGVAR(u64_Temp_Dec) = BGVAR(u64_DecRate);

         SalAccCommand((BGVAR(u64_Path_Acceleration)[s16_local_index]), drive);
         SalDecCommand((BGVAR(u64_Path_Deceleration)[s16_local_index]), drive);

         if(((BGVAR(u16_Path_Control)[s16_local_index]) & 0x80) == 0x0) // BIT 7: "0" - ABS position,
         {//absolute position command                                   //        "1" - INC position
            BGVAR(s16_Move_Abs_Issued) = 1;
         }
         else
         {//inc position command
            BGVAR(s16_Move_Abs_Issued) = 0;
         }

         //issue the move command
         BGVAR(s16_Sal_Move_Ind) = 1;

         if((((BGVAR(u16_Path_Control)[s16_local_index]) & 0x10) == 0x10) || (BGVAR(u16_AutoR_Immidiate_Path_Exec) == 1)) // BIT 4: "0" - wait for the running path to end and the execute the new path
         {// BIT 4: "1" - INS - immidiate
          //        "1" - the new path is using INS so preform it immidiate

            //test this reset!
            BGVAR(u16_AutoR_Immidiate_Path_Exec) = 0;

            if(((BGVAR(u16_Path_Control)[s16_local_index]) & 0xD0) == 0xD0)
            {//Close IPR#1179: if BIT 6="1" and BIT 4="1" and BIT 7="1" -> command is INC and insertion so
             // preform the relative position command from current position.
               do {
                  s16_temp_time = Cntr_3125;
                  s64_actual_pos = LLVAR(AX0_u32_Pfb_Internal_After_Mod_Lo);
               } while (s16_temp_time != Cntr_3125);

               // recalculate the new position command from current position
               s64_target_position += s64_actual_pos;

               // preform the new calculated command as ABS command.
               BGVAR(s16_Move_Abs_Issued) = 1;
            }

            s16_transition_mode = PTP_IMMEDIATE_TRANSITION;
         }
         else// END - wait until end
         {// BIT 4: "0"
            s16_transition_mode = PTP_MOTION_END_TRANSITION;
         }

         // IPR 1300: Velocity limitation (P1:55) in PS mode does not work
         // fix: saturate vel command to VLIM (P1-55) and start path
            s32_vcmd = (long)(BGVAR(u32_Path_Speed)[s16_local_index]);

         s16_execute_move_result = ExecuteMoveCommand(drive, s64_target_position, s32_vcmd, s16_transition_mode);

      /* waiting for schneider to describe the overlap function for PATH in PS mode.
          case 3:// PASS - dont stop and move to new position
               need to set s16_Move_Abs_Issued properly before calling ExecuteMoveCommand
               ExecuteMoveCommand(drive,s64_target_position , (BGVAR(u32_Path_Speed)[s16_local_index]), PTP_MOTION_PASS_TRANSITION);
          break;
         */

         // retrive the original ACC DEC values
         SalAccCommand(BGVAR(u64_Temp_Acc), drive);
         SalDecCommand(BGVAR(u64_Temp_Dec), drive);

         // FIX Bug#4364 - check the return value from "ExecuteMoveCommand" function and reset state machine if not success
         // to prevent motion buffering and unhandled motion
         if(s16_execute_move_result == SAL_SUCCESS)
         {// motion command accepted
            // signal to the AutoR outmode (when P2-44 = 1) changing index ondition
            BGVAR(u16_AutoR_OutMode) = AUTOR_OUT_MODE_CHANGING_INDEX;
            BGVAR(u16_Path_Internal_Execution_Read_Value) = ((BGVAR(s16_Running_Path_Index)) + 10000); // indicate that this path is curently running
            BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_RUNNING;
         }
         else
         {// motion command rejected reset state machine
            BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE;
         }
      }
      break;

      case PATH_STATE_MACHINE_RUNNING:// wait for the position to end
      {
         // Look for drive is stopped. Look also on PTP_Abort_Flag is 1 to be sure STOP command is issued
         if( (VAR(AX0_s16_Stopped) == 2) || (VAR(AX0_s16_Stopped) == -1) ||
             (BGVAR(u16_Ptp_Abort_Flags) & 0x0001) || (!Enabled(drive))  ||
             ((BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_IDLE) && (VAR(AX0_s16_Stopped) == 1)) ||
             ((BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_IDLE) && (!BGVAR(s16_Pos_Tune_Traj_Enable))))
         { // if position done or interrupted finish this path
           // for postune support: added an option to transit between pathes without waiting to peinpos

            // signal to the AutoR outmode (when P2-44 = 1) that PATH is in position
            BGVAR(u16_AutoR_OutMode) = AUTOR_OUT_MODE_INDEX_IN_POS;

            BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_DELAYED;
            BGVAR(s32_PathDelayTimer) = Cntr_1mS; // capture the time for the delay count
         }
      }
      break;

      case PATH_STATE_MACHINE_HOMING_EXECUTE:// trigger -> perform position
         BGVAR(s16_Path_Triggered) = 0;
         BGVAR(s16_Running_Path_Index) = s16_local_index;

         // signal to the AutoR outmode (when P2-44 = 1) that homing is running
         BGVAR(u16_AutoR_OutMode) = AUTOR_OUT_MODE_HOMING_RUNNING;

         BGVAR(u16_Path_Internal_Execution_Read_Value) = (BGVAR(s16_Running_Path_Index));
         // indicate the path number to perform

         BGVAR(u16_Path_Internal_Execution_Read_Value) = ((BGVAR(s16_Running_Path_Index)) + 10000); // indicate that this path is curently running
         BGVAR(s16_Number_Of_Parameters) = 0;
         if(HomeCommand(drive) != SAL_SUCCESS)
         {// home command rejected so reset the state machine and show "0" in P5-07 (home command failed)
            BGVAR(u16_Path_Internal_Execution_Read_Value) = BGVAR(s16_Running_Path_Index);
            BGVAR(s16_Next_Path_Index) = NO_WAITING_PATH;
            BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE; // set the state machine to idle
         }
         else
         {
            BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_HOMING_RUNNING;
         }
      break;

      case PATH_STATE_MACHINE_HOMING_RUNNING:// wait for the position to end
      {
         if(BGVAR(u8_Homing_State) == HOMING_TARGET_REACHED)
         {// homing completed check if there is a path to execute after homing and if there is delay and execute it

            // signal to the AutoR outmode (when P2-44 = 1) Homing completed
            BGVAR(u16_AutoR_OutMode) = AUTOR_OUT_MODE_HOMING_COMPLETED;

            if(((BGVAR(u16_Path_Control)[0]) >> 8) & 0xFF)
            {// set the next path to preform after homing is completed
               BGVAR(s16_Path_Internal_Execution) = (((BGVAR(u16_Path_Control)[0]) >> 8) & 0xFF);
               BGVAR(u16_Unhandled_Path_Internal_Execution) = 1;

               BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_DELAYED;
               BGVAR(s32_PathDelayTimer) = Cntr_1mS; // capture the time for the delay count
            }
            else
            {// stop and finish the state machine
               BGVAR(u16_Path_Internal_Execution_Read_Value) = ((BGVAR(s16_Running_Path_Index)) + 20000); // indicate that this path is stoped sucessfuly
               BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE; // set the state machine to idle
            }
         }
         else if (BGVAR(u8_Homing_State) == HOMING_FAILED)
         {// else if failed or no path to execute finish the stae machine
            BGVAR(s16_Next_Path_Index) = NO_WAITING_PATH;
            BGVAR(s16_Path_Internal_Execution) = 1000;
            PtpAbort(drive);
            BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE; // set the state machine to idle
         }
      }
      break;

      case PATH_STATE_MACHINE_DELAYED:// check delay
      {
         // Skip delay if motion is aborted by "STOP" cmd or by "K" etc
         if(PassedTimeMS((BGVAR(u16_Path_Delay)[BGVAR(s16_Running_Path_Index)]), BGVAR(s32_PathDelayTimer)))
         {
            if(BGVAR(u16_AutoR_Running_Path) == BGVAR(s16_Running_Path_Index))
            {// if Path executed from AUTOR signal path finished.
               BGVAR(u16_AutoR_Finished_Path) = BGVAR(s16_Running_Path_Index);
            }

            s16_local_index = -1;
            BGVAR(u16_Path_Internal_Execution_Read_Value) = ((BGVAR(s16_Running_Path_Index)) + 20000); // indicate that this path is stoped sucessfuly
            BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE; // set the state machine to idle
         }
         else if((VAR(AX0_s16_Stopped) != 2) || (!Enabled(drive)) ||
            ((BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_IDLE) && (!BGVAR(s16_Pos_Tune_Traj_Enable))))
         {
            s16_local_index = -1;
            if((BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_IDLE) && (!BGVAR(s16_Pos_Tune_Traj_Enable)))
            {
               BGVAR(s16_Next_Path_Index) = NO_WAITING_PATH;
               BGVAR(s16_Path_Internal_Execution) = 1000;
               BGVAR(u16_Unhandled_Path_Internal_Execution) = 1;
               PtpAbort(drive);

            }
         }
      }
      break;

      default:
         BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE;
      break;
   }
}


//**********************************************************
// Function Name: checkForInternalPathExecution
// Description: this function checks if an internal PATH execution occurred
// and if so it act like an path execution trigger with the entered path value
// this is the PRCM (P5-07) execution in the PathHandler function
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int checkForInternalPathExecution(int drive)
{
   if(BGVAR(u16_Unhandled_Path_Internal_Execution) == 1)
   {
      if(BGVAR(s16_Path_Internal_Execution) != 1000)// its a path index
      {//0-32
           BGVAR(u16_Unhandled_Path_Internal_Execution) = 0;
           return BGVAR(s16_Path_Internal_Execution); // enter into the path hendler sate machine
      }
     else if(BGVAR(s16_Path_Internal_Execution) == 1000) // stop position operation
      {// preform stop position - disable (motor stop)
         if ((BGVAR(u8_Homing_State) != HOMING_IDLE) &&
             (BGVAR(u8_Homing_State) != HOMING_TARGET_REACHED) &&
             (BGVAR(u8_Homing_State) != HOMING_FAILED))
            {// FIX BUG#4412 If Homing in progress and not in final state
               STORE_EXECUTION_PARAMS_0_1;
               s16_Number_Of_Parameters = 1;
               s64_Execution_Parameter[0] = 0;
               HomeCommand(0);     // cancel homing
               RESTORE_EXECUTION_PARAMS_0_1;
            }
         if (BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_IDLE) MotorStop(drive); // Don't use stop in internal trajectory mode
         else PtpAbort(drive);
         
         //BZ5552: Auto tuning falls with fault "ICMD Saturation" and return PFB to zero.
         BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE; 
         BGVAR(u16_Path_Internal_Execution_Read_Value) = (BGVAR(s16_Path_Internal_Execution) + 20000); // motor stop used! (21000)
         BGVAR(u16_Unhandled_Path_Internal_Execution) = 0; // Reset to avoid infinite STOP (Serj)
      }
   }
   return -1; //do nothing
}
//**********************************************************
// Function Name: PosTuneTrajEn
// 0: Stop running.
// 1: Enable postunetraj running in specified postunetrajmode
// Author: Sergey
// Algorithm:
// Revisions: Udi Oct 20 2014: Check velocity limit and trapeze motion
//**********************************************************
int PosTuneTrajEn(long long lparam, int drive)
{
   unsigned long u32_at_min_speed_limit_int;
   float f_temp, f_min_dist;
   REFERENCE_TO_DRIVE;

   //REFERENCE_TO_DRIVE;     // Defeat compilation error
   // AXIS_OFF;

   if(BGVAR(s16_Pos_Tune_Traj_Enable) == (int)lparam) return SAL_SUCCESS;

   if(lparam == 0LL) // stop running
   {
        //BZ5552: Auto tuning falls with fault "ICMD Saturation" and return PFB to zero.
        BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE;
        // set path = 1000 to signal about stop running
        BGVAR(s16_Path_Internal_Execution) = 1000;
        BGVAR(s16_Next_Path_Index) = NO_WAITING_PATH; // there is no waiting path
        // signal there is new path
        BGVAR(u16_Unhandled_Path_Internal_Execution) = 1;
        BGVAR(s16_Pos_Tune_Traj_Enable) = 0;
        // Indicate parameter change
        BGVAR(s16_Pos_Tune_En_or_Mode_Changed) = 1;        
        return SAL_SUCCESS;
   }

   if((BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE) ||
       BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE_1_CYCLE)
          return (NOT_AVAILABLE);

   // Check the distances are equal If internal user defined trajectory profile is used
   if(BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_EXT_MODE)
   {

      if(llabs(BGVAR(s64_PosTuneTraj_Pos)[0]) != llabs(BGVAR(s64_PosTuneTraj_Pos)[1]))
         return (HDTUNE_DIST_NOT_EQUAL);

      //Check Vel limits for User defined traj
      u32_at_min_speed_limit_int = 0.1*BGVAR(u32_Mspeed);

      //Vel
      if( BGVAR(u32_PosTuneTraj_Spd)[0] <  (u32_at_min_speed_limit_int))
         return (HDTUNE_VCRUISE_LOW);

      if( BGVAR(u32_PosTuneTraj_Spd)[1] <  (u32_at_min_speed_limit_int))
         return (HDTUNE_VCRUISE_LOW);

      //Check if user profile will result in a trapeze shape
      // Min dist for Trapeze is: (V^2 / 2a) + (V^2 / 2d)
      // Additional Multiplication by 4 because Tsp^2 = (2*Tsv)^2 = 4*Tsv^2 (for Units conversion).
      // Min dist for Trapeze is: V^2 * 2 * (1/a + 1/d)
      
      // for tsp 125 usec
      // ACC[rpm/s] -> ACC(internal units) = ACC*2^64/60/8000^2
      // V[rpm] -> V(internal units) = V*2^32/60/8000
      // for tsp 250 usec
      // ACC[rpm/s] -> ACC(internal units) = ACC*2^64/60/4000^2
      // V[rpm] -> V(internal units) = V*2^32/60/8000

      if(BGVAR(u32_PosTuneTraj_Spd)[0] > BGVAR(u32_PosTuneTraj_Spd)[1])
         f_temp = (float)BGVAR(u32_PosTuneTraj_Spd)[0];
      else
         f_temp = (float)BGVAR(u32_PosTuneTraj_Spd)[1];
      f_temp *= f_temp;

      if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) f_temp *= 2.0; // Pos loop Tsp = 250uS
      else f_temp *= 0.5;        // Pos loop Tsp = 125uS

      //4294967296 is 2^32, because of ACC internal units
      f_min_dist = (f_temp) *(4294967296.0/(float)BGVAR(u64_PosTuneTraj_AccDec)[0] + 4294967296.0/(float)BGVAR(u64_PosTuneTraj_AccDec)[1]);
      if(llabs(BGVAR(s64_PosTuneTraj_Pos)[0]) < (long long)f_min_dist)
         return (HDTUNE_WRONG_PROFILE);

      //Check if Bi directional but the 2 positions are the same sign
      if((BGVAR(s16_Pos_Tune_Cycle) < 2) &&
         (((BGVAR(s64_PosTuneTraj_Pos)[0] > 0) && (BGVAR(s64_PosTuneTraj_Pos)[1] > 0)) ||
          ((BGVAR(s64_PosTuneTraj_Pos)[0] < 0) && (BGVAR(s64_PosTuneTraj_Pos)[1] < 0))))
         return(HDTUNE_DIST_DIFF_SIGN);

      //Check if Uni directional but the 2 positions have different sign
      if((BGVAR(s16_Pos_Tune_Cycle) >= 2) &&
         (((BGVAR(s64_PosTuneTraj_Pos)[0] > 0) && (BGVAR(s64_PosTuneTraj_Pos)[1] < 0)) ||
          ((BGVAR(s64_PosTuneTraj_Pos)[0] < 0) && (BGVAR(s64_PosTuneTraj_Pos)[1] > 0))))
         return(HDTUNE_DIST_SAME_SIGN);
      if  (BGVAR(u64_DecStop) <= BGVAR(u64_PosTuneTraj_AccDec)[0]) 
      {
         BGVAR(s32_Pos_Tune_Bits) |= AT_DEC_DECSTOP_WARNING;
         return (HDTUNE_DEC_DECSTOP_ERROR);
      }
   }

   BGVAR(s16_Pos_Tune_Traj_Enable) = (int)lparam;
   // Indicate parameter change
   BGVAR(s16_Pos_Tune_En_or_Mode_Changed) = 1;

   return SAL_SUCCESS;
}
//**********************************************************
// Function Name: PosTuneTrajMode
//
// 0: Stop running.
// 1: Use Path 1 and Path 2 with external values
// 2: Use Path 1 and Path 2 with internal values
// 3: Use Path 1 and Path 2 with internal values (one cycle)
// Author: Sergey
// Algorithm: Based on SalWritePRCMParameterValue
// Revisions:
//**********************************************************
int PosTuneTrajMode(long long lparam, int drive)
{
//   // AXIS_OFF;
   long long index = lparam;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(BGVAR(s16_Pos_Tune_Traj_Mode) == (int)lparam) return SAL_SUCCESS;

   // prohibit mode changing, except mode 0 (used for stop)
   if((index != 0LL) && BGVAR(s16_Pos_Tune_Traj_Enable)) return PTT_IS_ACTIVE;
   //if (!Enabled(DRIVE_PARAM)) return DRIVE_INACTIVE; // if drive not active return drive inactive
   //if((index < 0LL) || (index >= 4)) return VALUE_OUT_OF_RANGE;

   //if((index != 0LL) && (BGVAR(u16_Cont_Run_Path_State) != RUN_PATH_HNDLR_IDLE)) return FUNCTIONALITY_OCCUPIED;
   // if index == 0 then stop running
   if(index == 0LL)
   {
        //BZ5552: Auto tuning falls with fault "ICMD Saturation" and return PFB to zero.
        BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE;
        // set path = 1000 to signal about stop running
        BGVAR(s16_Path_Internal_Execution) = 1000;
        BGVAR(s16_Next_Path_Index) = NO_WAITING_PATH; // there is no waiting path

        // signal there is new path
        BGVAR(u16_Unhandled_Path_Internal_Execution) = 1;
        BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_DISABLED;
        BGVAR(s16_Pos_Tune_Traj_Mode) = (int)index;
   }
   // if index == 1 or 2 then start path handler, what will continuously run path 1 and 2 with predefined path params
   //else if(index > 0LL) BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_START_WITH_BACKUP;
   else
   BGVAR(s16_Pos_Tune_Traj_Mode) = (int)index;
   BGVAR(s16_Pos_Tune_Traj_Mode_User) = (int)index; // Keep original user value for reading (SAL)
   // Indicate parameter change
   BGVAR(s16_Pos_Tune_En_or_Mode_Changed) = 1;
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: PathContRunHandler
// Description:
//    This Handler is called from the BG and serves continuous execution of PATH 1 and 2
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
void PathContRunHandler(int drive)
{
    // AXIS_OFF;
    static int s16_prev_path = 0;
    int index = 0;
    // Restore original PATH parameters if drive became disabled, traj mode changed or traj disabled
    if(((!Enabled(drive)) || (BGVAR(s16_Pos_Tune_En_or_Mode_Changed) == 1)) &&
        (BGVAR(u16_Cont_Run_Path_State) > RUN_PATH_HNDLR_START_WITH_BACKUP))
        {
            BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_RESTORE;
        }
    BGVAR(s16_Pos_Tune_En_or_Mode_Changed) = 0;

    switch (BGVAR(u16_Cont_Run_Path_State))
    {
        case RUN_PATH_HNDLR_IDLE:
        if(!BGVAR(s16_Pos_Tune_Traj_Enable) || (BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_IDLE) || (!Enabled(drive))) return;
        BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_START_WITH_BACKUP;
        break;

        case RUN_PATH_HNDLR_START_WITH_BACKUP:
        // backup parameters
        for(index=0; index <= 1; index++)
        {
            BGVAR(u64_Path_Acceleration_BackUp)[index] = BGVAR(u64_Path_Acceleration)[index+1];
            BGVAR(u64_Path_Deceleration_BackUp)[index] = BGVAR(u64_Path_Deceleration)[index+1];
            BGVAR(u16_Path_Control_BackUp)[index]      = BGVAR(u16_Path_Control)[index+1];
            BGVAR(u16_Path_Delay_BackUp)[index]        = BGVAR(u16_Path_Delay)[index+1];
            BGVAR(s64_Path_Position_BackUp)[index]     = BGVAR(s64_Path_Position)[index+1];
            BGVAR(u32_Path_Speed_BackUp)[index]        = BGVAR(u32_Path_Speed)[index+1];
        }
        BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_SET_EXT_PARAM;
        // Select which parameter to use for trajectory - automatic or user-defined
        if (BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_EXT_MODE){              BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_SET_EXT_PARAM;}
        else if ((BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE) || (BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE_1_CYCLE)){
                                                              BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_SET_INT_PARAM;}
        break;

        case RUN_PATH_HNDLR_SET_EXT_PARAM:
        // Set ACC,DEC,Control,Delay after Path execution, Position, Speed for PATH 1
        BGVAR(u64_Path_Acceleration)[1] = BGVAR(u64_PosTuneTraj_AccDec)[0]; //Set ACC
        BGVAR(u64_Path_Deceleration)[1] = BGVAR(u64_PosTuneTraj_AccDec)[1]; //Set DEC
        BGVAR(u16_Path_Control)[1]      = 128;  // Incremental moving without blending
        // if Delay is not set internally, use default value
        BGVAR(u16_Path_Delay)[1] = 500;
        //Set equal delay if movement is unidirectional
        if(BGVAR(s16_Pos_Tune_Cycle) >= 2) BGVAR(u16_Path_Delay)[1] = 1000;
        BGVAR(s64_Path_Position)[1]     = BGVAR(s64_PosTuneTraj_Pos)[0];
        BGVAR(u32_Path_Speed)[1]        = BGVAR(u32_PosTuneTraj_Spd)[0];
        // Set ACC,DEC,Control,Delay after Path execution, Position, Speed for PATH 2
        BGVAR(u64_Path_Acceleration)[2] = BGVAR(u64_PosTuneTraj_AccDec)[0]; //Set ACC
        BGVAR(u64_Path_Deceleration)[2] = BGVAR(u64_PosTuneTraj_AccDec)[1]; //Set DEC
        BGVAR(u16_Path_Control)[2]      = 128;  // Incremental moving without blending
        // if Delay is not set internally, use default value
        BGVAR(u16_Path_Delay)[2] = 1000;
        BGVAR(s64_Path_Position)[2]     = BGVAR(s64_PosTuneTraj_Pos)[1];
        BGVAR(u32_Path_Speed)[2]        = BGVAR(u32_PosTuneTraj_Spd)[1];
        BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_START;

        // update test trajecotyr data
        BGVAR(u32_Hdtune_Act_Traj_Spd) = BGVAR(u32_Path_Speed)[1];
        BGVAR(s64_Hdtune_Act_Traj_Dist) = BGVAR(s64_Path_Position)[1];
        BGVAR(u64_Hdtune_Act_Traj_Acc) = BGVAR(u64_Path_Deceleration)[1];

        break;

        case RUN_PATH_HNDLR_SET_INT_PARAM:
        // Set ACC,DEC,Control,Delay after Path execution, Position, Speed for PATH 1
        BGVAR(u64_Path_Acceleration)[1] = BGVAR(u64_PosTuneTraj_AccDec_Int)[0];
        BGVAR(u64_Path_Deceleration)[1] = BGVAR(u64_PosTuneTraj_AccDec_Int)[0];
        BGVAR(u16_Path_Control)[1]      = 128;  // Incremental moving without blending
        // if Delay is not set internally, use default value
        if(BGVAR(s16_PosTuneTraj_Delay_Int)[0] == 0) BGVAR(u16_Path_Delay)[1] = 500;
        else BGVAR(u16_Path_Delay)[1]   = BGVAR(s16_PosTuneTraj_Delay_Int)[0];//Set Delay between path to path iteration
        BGVAR(s64_Path_Position)[1]     = BGVAR(s64_PosTuneTraj_Pos_Int)[0];
        BGVAR(u32_Path_Speed)[1]        = BGVAR(u32_PosTuneTraj_Spd_Int)[0];
        // Set ACC,DEC,Control,Delay after Path execution, Position, Speed for PATH 2
        BGVAR(u64_Path_Acceleration)[2] = BGVAR(u64_PosTuneTraj_AccDec_Int)[1];
        BGVAR(u64_Path_Deceleration)[2] = BGVAR(u64_PosTuneTraj_AccDec_Int)[1];
        BGVAR(u16_Path_Control)[2]      = 128;  // Incremental moving without blending
        // if Delay is not set internally, use default value
        if(BGVAR(s16_PosTuneTraj_Delay_Int)[1] == 0) BGVAR(u16_Path_Delay)[2] = 1000;
        else BGVAR(u16_Path_Delay)[2]   = BGVAR(s16_PosTuneTraj_Delay_Int)[1];//Set Delay between path to path iteration
        BGVAR(s64_Path_Position)[2]     = BGVAR(s64_PosTuneTraj_Pos_Int)[1];
        BGVAR(u32_Path_Speed)[2]        = BGVAR(u32_PosTuneTraj_Spd_Int)[1];
        BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_START;

        // update test trajecotyr data
        BGVAR(u32_Hdtune_Act_Traj_Spd) = BGVAR(u32_Path_Speed)[1];
        BGVAR(s64_Hdtune_Act_Traj_Dist) = BGVAR(s64_Path_Position)[1];
        BGVAR(u64_Hdtune_Act_Traj_Acc) = BGVAR(u64_Path_Deceleration)[1];

        break;

        case RUN_PATH_HNDLR_START:
        // set path 1
        BGVAR(s16_Path_Internal_Execution) = 1;
        // signal there is new path
        BGVAR(u16_Unhandled_Path_Internal_Execution) = 1;
        BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_WAIT_FOR_READY;
        break;

        case RUN_PATH_HNDLR_WAIT_FOR_READY:
        if (BGVAR(u16_Path_Exec_State) == PATH_STATE_MACHINE_IDLE) break;
        // set path 2
        BGVAR(s16_Path_Internal_Execution) = 2;
        s16_prev_path = 2;
        // signal there is new path
        BGVAR(u16_Unhandled_Path_Internal_Execution) = 1;
        BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_WAIT_FOR_FREE_PATH;
        break;

        case RUN_PATH_HNDLR_WAIT_FOR_FREE_PATH:
        if ((BGVAR(s16_Next_Path_Index) != NO_WAITING_PATH) &&
            (BGVAR(s16_Path_Internal_Execution) != 1000)  &&
            (Enabled(drive))) break;
        // Stop if disabled or runpath==1000
        if ((BGVAR(s16_Path_Internal_Execution) == 1000) ||
            (!BGVAR(s16_Pos_Tune_Traj_Enable))||
            ((BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE_1_CYCLE) && (BGVAR(s16_Path_Internal_Execution) == 2)))
            BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_END;
        else
        {
            // Switch PATH - 1->2->1->2...
            BGVAR(s16_Path_Internal_Execution) = 1;
            if (s16_prev_path == 1) BGVAR(s16_Path_Internal_Execution) = 2;
            s16_prev_path = BGVAR(s16_Path_Internal_Execution);
            // signal there is new path
            BGVAR(u16_Unhandled_Path_Internal_Execution) = 1;
        }
        break;

        case RUN_PATH_HNDLR_END:
        if ((BGVAR(u16_Path_Exec_State) != PATH_STATE_MACHINE_IDLE) &&
            (!(BGVAR(u16_Ptp_Abort_Flags) & 0x0001)) &&
            (Enabled(drive)) && (BGVAR(s16_Pos_Tune_Traj_Enable))) break;


        // restore parameters if needed (path 1 and 2 only)
         //BGVAR(s16_Pos_Tune_Traj_Mode) = PTT_IDLE;

        //BZ5552: Auto tuning falls with fault "ICMD Saturation" and return PFB to zero.
        BGVAR(u16_Path_Exec_State) = PATH_STATE_MACHINE_IDLE;
        BGVAR(s16_Next_Path_Index) = NO_WAITING_PATH; // there is no waiting path
        BGVAR(s16_Path_Internal_Execution) = 1000;
        BGVAR(u16_Unhandled_Path_Internal_Execution) = 1;

        if(BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_EXT_MODE) BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_DISABLED;
        //BGVAR(s16_Pos_Tune_Traj_Mode) = PTT_IDLE;
        BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_RESTORE;
        break;

        case RUN_PATH_HNDLR_RESTORE:
        if (VAR(AX0_s16_Stopped) == 0) break;
        for(index=0; index <= 1; index++)
        {
            BGVAR(u64_Path_Acceleration)[index+1] = BGVAR(u64_Path_Acceleration_BackUp)[index];
            BGVAR(u64_Path_Deceleration)[index+1] = BGVAR(u64_Path_Deceleration_BackUp)[index];
            BGVAR(u16_Path_Control)[index+1]      = BGVAR(u16_Path_Control_BackUp)[index];
            BGVAR(u16_Path_Delay)[index+1]        = BGVAR(u16_Path_Delay_BackUp)[index];
            BGVAR(s64_Path_Position)[index+1]     = BGVAR(s64_Path_Position_BackUp)[index];
            BGVAR(u32_Path_Speed)[index+1]        = BGVAR(u32_Path_Speed_BackUp)[index];
        }
        BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_IDLE;
        break;

        default:
        BGVAR(u16_Cont_Run_Path_State) = RUN_PATH_HNDLR_IDLE;
    }

}

//**********************************************************
// Function Name: SalWritePRCMParameterValue
// Description: set value into RPCM parameter (P5-07)
// 1-32: execute the selected path position command.
// 1000: Stop positioning.
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPRCMParamWriteCommand(int drive)
{
   long long index = s64_Execution_Parameter[0];
   int s16_access_rights_permission;

   // FIX BUG#4422 P5-07 = 1000 is active only if drive is in "SE_OPMODE_PS"
   if(BGVAR(u16_LXM28_Opmode) != SE_OPMODE_PS) return INVALID_OPMODE; // valid only in SE PS mode

   if (!Enabled(DRIVE_PARAM)) return DRIVE_INACTIVE; // if drive not active return drive inactive

   if((index < 0LL) || ((index >= (long long)NUM_OF_PATHS) && (index != 1000)))
      return VALUE_OUT_OF_RANGE; // 1000 = stop position immediately

   // Check if it is allowed to trigger motion in PS mode according to the access-rights management
   s16_access_rights_permission = CheckLexiumAccessRights(drive, LEX_AR_FCT_ACTIVATE_PS, BGVAR(s16_Lexium_ExecPParam_Acces_Channel));
   // If the user did not command a STOP (via the value of 1000) and
   // if there are no access-rights for starting motion in mode PS.
   if((index != 1000) && (s16_access_rights_permission != SAL_SUCCESS))
   {
      return (s16_access_rights_permission);
   }


   // set new value to RPCM (P5-07) parameter
   BGVAR(s16_Path_Internal_Execution) = (unsigned int)index;
   // signal there is new PRCM value
   BGVAR(u16_Unhandled_Path_Internal_Execution) = 1;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadPathAccDec
// Description: read the path acc and dec into 32 bit value
// for P7-02 - P7-64 (even)
// 0xAAAABBBB
// AAAA - path acceleration rate (16 bit) user data
// BBBB - path deceleration rate (16 bit) user data
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPathAccDec(long long *data, int drive)
{
   unsigned int u16_temp_acc = 0, u16_temp_dec = 0, u16_path_index = 0;

   // index +1 to skip the first parameter P7-00 that belong to homing and handled diffrent and start from P7-02
   u16_path_index = (unsigned int)(s64_Execution_Parameter[0] + 1);
   // check path index value
   if(u16_path_index > (unsigned int)NUM_OF_PATHS) return VALUE_OUT_OF_RANGE;

   u16_temp_acc = convertAccDecInternalUnitsForPosToMs(drive, BGVAR(u64_Path_Acceleration)[u16_path_index]);
   u16_temp_dec = convertAccDecInternalUnitsForPosToMs(drive, BGVAR(u64_Path_Deceleration)[u16_path_index]);
   *data = (long long)(((long)u16_temp_acc << 16)|(u16_temp_dec & 0xFFFF));
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalWritePathAccDec
// Description: write the path acc and dec from 32 bit value
// for P7-02 - P7-64 (even)
// 0xAAAABBBB into internal acc/dec for pos
// AAAA - path acceleration rate (16 bit) user data
// BBBB - path deceleration rate (16 bit) user data
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWritePathAccDec(int drive)
{
   unsigned int u16_path_index = 0, u16_temp_acc = 0, u16_temp_dec = 0;

   // index +1 to skip the first parameter P7-00 that belong to homing and handled diffrent and start from P7-02
   u16_path_index = (unsigned int)(s64_Execution_Parameter[0] + 1);

   // check path index value
   if(u16_path_index > (unsigned int)NUM_OF_PATHS) return VALUE_OUT_OF_RANGE;

   if (((s64_Execution_Parameter[1] & 0xFFFF) > MIN_ACC_DEC_IN_MS) || (((s64_Execution_Parameter[1] >> 16) & 0xFFFF) > MIN_ACC_DEC_IN_MS)) return (VALUE_TOO_HIGH);
   if (((s64_Execution_Parameter[1] & 0xFFFF) < MAX_ACC_DEC_IN_MS) || (((s64_Execution_Parameter[1] >> 16) & 0xFFFF) < MAX_ACC_DEC_IN_MS))     return (VALUE_TOO_LOW);

   u16_temp_acc = (unsigned int)((s64_Execution_Parameter[1] >> 16) & 0xFFFF);
   u16_temp_dec = (unsigned int)(s64_Execution_Parameter[1] & 0xFFFF);

   BGVAR(u64_Path_Acceleration)[u16_path_index] = convertMsToAccDecInternalUnitsForPos(drive, u16_temp_acc);
   BGVAR(u64_Path_Deceleration)[u16_path_index] = convertMsToAccDecInternalUnitsForPos(drive, u16_temp_dec);

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadHomeAccDec
// Description: read the home acc and dec in ms from P7-00
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadHomeAccDec(long long *data, int drive)
{
   unsigned int u16_temp_acc = 0, u16_temp_dec = 0;

   u16_temp_acc = convertAccDecInternalUnitsForPosToMs(drive, BGVAR(u64_Path_Acceleration)[0]);
   u16_temp_dec = convertAccDecInternalUnitsForPosToMs(drive, BGVAR(u64_Path_Deceleration)[0]);


//   *data = (long long)(u16_temp_acc & 0xFFFF);

   *data = (long long)(((unsigned long)u16_temp_acc << 16) |(unsigned long)u16_temp_dec);

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteHomeAccDec
// Description: write the path acc and dec for P7-00 (Home ACC/DEC parameter)
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteHomeAccDec(long long param,int drive)
{
   unsigned int u16_temp_acc = 0, u16_temp_dec = 0;

   u16_temp_dec = (unsigned int) (param & 0xFFFF);
   u16_temp_acc = (unsigned int) ((param >> 16) & 0xFFFF);

   if((u16_temp_dec > MIN_ACC_DEC_IN_MS)||(u16_temp_acc > MIN_ACC_DEC_IN_MS))
   {
      return VALUE_TOO_HIGH;
   }
   else if((u16_temp_dec < MAX_ACC_DEC_IN_MS)||(u16_temp_acc < MAX_ACC_DEC_IN_MS))
   {
      return VALUE_TOO_LOW;
   }

   BGVAR(u64_Path_Acceleration)[0] = convertMsToAccDecInternalUnitsForPos(drive, u16_temp_acc);
   BGVAR(u64_Path_Deceleration)[0] = convertMsToAccDecInternalUnitsForPos(drive, u16_temp_dec);

   // Use Path 0 parameters for Homing...
   BGVAR(u64_HomeAccRate) = BGVAR(u64_Path_Acceleration)[0];
   BGVAR(u64_HomeDecRate) = BGVAR(u64_Path_Deceleration)[0];

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadHomeDly
// Description: get the home delay in ms (P7-01)
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadHomeDly(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   *data = ((long long)BGVAR(u16_Path_Delay)[0]) & 0x000000000000FFFF;
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteHomeDly
// Description: set the home delay [ms] after homing comleted and befor next path is executed (P7-01)
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteHomeDly(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // limits are tested in the excel.
   BGVAR(u16_Path_Delay)[0] = (unsigned int)param;
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadPathSpdDly
// Description: get the speed and delay for relevant path according to it's index (P7-03 - P7-65)
// when speed in 0.1*rpm units and delay in ms units.
// into 0xAAAABBBB format:
// AAAA: speed in 0.1*rpm
// BBBB: delay in ms
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPathSpdDly(long long *data, int drive)
{
   unsigned int u16_temp_speed = 0, u16_path_index = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // index +1 to skip the first array parameter which is homing parameter (the first path is in index 1 in the array)
   u16_path_index = (unsigned int)(s64_Execution_Parameter[0]+1);
   if((u16_path_index > (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;

   // Convert units:
   // speed: internal velocity into 0.1rpm.
   // delay: no conevrsion is needed.

   u16_temp_speed = (unsigned int) MultS64ByFixS64ToS64((long long)BGVAR(u32_Path_Speed)[u16_path_index],
                                                         BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                         BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

   *data = (long long)(((long)u16_temp_speed << 16) |(BGVAR(u16_Path_Delay)[u16_path_index]));
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalWritePathSpdDly
// Description: set the speed and delay for relevant path according to it's index (P7-03 - P7-65)
// when speed in 0.1*rpm units and delay in ms units.
// from 0xAAAABBBB format:
// AAAA: speed in 0.1*rpm
// BBBB: delay in ms
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWritePathSpdDly(int drive)
{
   unsigned int u32_temp_speed=0 , u16_temp_delay=0;
   unsigned int u16_path_index = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // index +1 to skip the first array parameter which is homing parameter (the first path is in index 1 in the array)
   u16_path_index = (unsigned int)(s64_Execution_Parameter[0]+1);
   if((u16_path_index > (long long)NUM_OF_PATHS)) return VALUE_OUT_OF_RANGE;

   // delay no conevsion needed
   // set the delay value
   u16_temp_delay = (s64_Execution_Parameter[1] &  0xFFFF);
   if((u16_temp_delay > (long long)SCHNEIDER_MAX_DELAY_FOR_MS)) return VALUE_OUT_OF_RANGE;

   BGVAR(u16_Path_Delay)[u16_path_index] = u16_temp_delay;

   // speed: from user speed units 0.1rpm into internal velocity out loop units
   u32_temp_speed = ((s64_Execution_Parameter[1] >> 16) & 0xFFFF);
   // for now the speed is limited to 6000 rpm but if it's greater then VLIM it won't work for now
   if((u32_temp_speed > (long long)SCHNEIDER_MAX_VELOCITY_FOR_0_1_RPM)) return VALUE_OUT_OF_RANGE;

   // convert and set the speed value
    BGVAR(u32_Path_Speed)[u16_path_index] = (unsigned long) MultS64ByFixS64ToS64((long long)u32_temp_speed,
                                                         BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                                         BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: RculateSchneidrPUUPositionToGearRatio
// Description: recalculate all Schneider PUU position variables int the new gear ratio
// if pathIndex == -1 run on all the pathes else if < 32 recalculate only this index
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void RculateSchneidrPUUPositionToGearRatio(int drive, int pathIndex)
{// multiple the current value with 1/(old gear ratio)
 // and multiple in the new gear ratio
   int s16_i = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) || pathIndex > (NUM_OF_PATHS-1) ) return;

   if(pathIndex < 0)
   {// run on all path positions
   // update home (P6-00) and PATH 1- PATH 32 position value
       for (s16_i = 0; s16_i < NUM_OF_PATHS; s16_i++)
       {
       // multiple the original position with new gear ratio to get the new position value
            (BGVAR(s64_Path_Position)[s16_i]) = MultS64ByFixS64ToS64((BGVAR(s64_Path_Position_PUU)[s16_i]),
                                          BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                          BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr);
       }
   }
   else
   {// run only on individual path positions
            (BGVAR(s64_Path_Position)[pathIndex]) = MultS64ByFixS64ToS64((BGVAR(s64_Path_Position_PUU)[pathIndex]),
                                          BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                          BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr);
   }
}

void RecalculateSchneiderInternalProfilePositionToGearRatio(int drive)
{
    REFERENCE_TO_DRIVE;     // Defeat compilation error
    // recalculate internal profile position
    BGVAR(s64_IntProfIncDist_InternalUnits) = MultS64ByFixS64ToS64((long long)BGVAR(s32_IntProfIncDist_UserUnits),
                                       BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                       BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr);
}

int SalReadExcessiveDeviationConditionCommand(long long *data, int drive)//PDEV (P2-35) read
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //convert from CDHD internal position units into PULSE units and return for buid in PULSE to PUU convertion
   *data = MultS64ByFixS64ToS64(BGVAR(u64_Pe_Max),
            BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
            BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);
   return SAL_SUCCESS;
}


int SalWriteExcessiveDeviationConditionCommand(long long param, int drive) //PDEV (P2-35) write
{
       //save the original entered value in PULSE units for case of gear ratio changes
   BGVAR(u32_P2_35_PDEV) = (unsigned long)param;
   RecalculateExcessiveDeviationConditionToCDHDPositionUnits(drive);
   return SAL_SUCCESS;
}


void RecalculateExcessiveDeviationConditionToCDHDPositionUnits(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

       //convert from PUU into CDHD internal position units and set it in the PEMAX variable.
       BGVAR(u64_Pe_Max) = MultS64ByFixS64ToS64((long long)BGVAR(u32_P2_35_PDEV),
                           BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix,
                           BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr);
}


//**********************************************************
// Function Name: SalModuloModeCommand
// Description:

// Author: Lionel
// Algorithm:
// Revisions:
//**********************************************************

int SalModuloModeCommand (long long param, int drive)
{
   if(((param & 0x1) != 0) && (p402_modes_of_operation_display >= 7) && (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1))
   {
      return (INVALID_OPMODE);
   }
    if ((0 != (param & 0x7)) && (1 != (param & 0x7)) && (3 != (param & 0x7)) && (5 != (param & 0x7)))
      return (VALUE_OUT_OF_RANGE);
      
   BGVAR(u16_Modulo_Mode)=(unsigned int)(param & 0x7);
   PositionModuloConfig(drive);
   return SAL_SUCCESS;
}

// Write function for PROTARY command
int SalPositionModuloCommand(int drive)
{
   // The range was selected arbitrarily. It was assessed to be enough.
   // However the function can handle more.  +-((2^63)-1)/1000
   REFERENCE_TO_DRIVE;
   if (s64_Execution_Parameter[1] > 2147483647000LL) return (VALUE_TOO_HIGH); // (2^31-1)*1000
   if (s64_Execution_Parameter[1] < -2147483647000LL) return (VALUE_TOO_LOW); // (2^31-1)*1000
   if (s64_Execution_Parameter[0] == 1)
      BGVAR(s64_Position_Modulo)[0] = s64_Execution_Parameter[1];
   else if (s64_Execution_Parameter[0] == 2)
      BGVAR(s64_Position_Modulo)[1] = s64_Execution_Parameter[1];
   else
      return (VALUE_OUT_OF_RANGE);
   PositionModuloConfig(drive);

   return SAL_SUCCESS;
}

void PositionModuloConfig(int drive)
{
   // AXIS_OFF;
   long long s64_temp;
   REFERENCE_TO_DRIVE;

   if (BGVAR(s64_Position_Modulo)[0] > BGVAR(s64_Position_Modulo)[1])
   {
      BGVAR(s64_Modulo_Range) = BGVAR(s64_Position_Modulo)[0] - BGVAR(s64_Position_Modulo)[1];
      BGVAR(s64_Modulo_Low) = BGVAR(s64_Position_Modulo)[1];
   }
   else
   {
      BGVAR(s64_Modulo_Range) = BGVAR(s64_Position_Modulo)[1] - BGVAR(s64_Position_Modulo)[0];
      BGVAR(s64_Modulo_Low) = BGVAR(s64_Position_Modulo)[0];
   }

   s64_temp = BGVAR(s64_Modulo_Low)/* / 1000*/;
   LVAR(AX0_u32_Position_Modulo_Low_Limit_Lo) = (long)(s64_temp & 0x00000000FFFFFFFF);
   LVAR(AX0_s32_Position_Modulo_Low_Limit_Hi) = (long)((s64_temp >> 32) & 0x00000000FFFFFFFF);

   s64_temp = BGVAR(s64_Modulo_Range)/* / 1000*/;
   LLVAR(AX0_u32_Counts_Per_Rev_For_Mod_Lo) = ((long long)(LVAR(AX0_s32_Counts_Per_Rev) / BGVAR(u16_Motor_Enc_Interpolation)) * 1000L);
   LVAR(AX0_u32_Position_Modulo_Range_Lo) = (long)(s64_temp & 0x00000000FFFFFFFF);
   LVAR(AX0_s32_Position_Modulo_Range_Hi) = (long)((s64_temp >> 32) & 0x00000000FFFFFFFF);
   //test
   LVAR(AX0_u32_Rev_In_Mod_Range) = (long)(s64_temp / LLVAR(AX0_u32_Counts_Per_Rev_For_Mod_Lo));
   LLVAR(AX0_u32_Pfb_Counts_Mod_Mem_Update_Lo) = LLVAR(AX0_u32_Counts_Per_Rev_For_Mod_Lo) * (long long)LVAR(AX0_u32_Rev_In_Mod_Range);

   if ((BGVAR(s64_Modulo_Range) != 0) && (BGVAR(u16_Modulo_Mode) > 0) && (BGVAR(u16_MotorType) == ROTARY_MOTOR))
     VAR(AX0_u16_Position_Modulo_Active) = 1;
   else
      VAR(AX0_u16_Position_Modulo_Active) = 0;

   OneDivS64ToFixU32Shift16((long*)&LVAR(AX0_u32_Pfb_Mod_Scale_Fix),(unsigned int *)&VAR(AX0_s16_Pfb_Mod_Scale_Shr),LLVAR(AX0_u32_Counts_Per_Rev_For_Mod_Lo));
   VAR(AX0_s16_Pfb_Mod_Scale_Shr) -= 32; //same as multipling by 2^32

}

// Read function for PROTARY command
int SalReadPositionModuloCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (s64_Execution_Parameter[0] == 1)
      PrintSignedLongLong(BGVAR(s64_Position_Modulo)[0]);
   else if (s64_Execution_Parameter[0] == 2)
      PrintSignedLongLong(BGVAR(s64_Position_Modulo)[1]);
   else
      return (VALUE_OUT_OF_RANGE);
   PrintChar(SPACE);
   PrintChar('[');
   PrintString("Counts",0);
   PrintChar(']');
   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalMencResShrCommand(long long param,int drive)
{
   BGVAR(u16_Mencres_Shr) = (unsigned int)param;
   CalcPosFbFilterGain(drive);
   PositionConfig(drive, 0);
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalPosTuneTrajAccDecCommand
// Description:
//    PLease note - PATH1 = PATH2
//    POSTUNETRAJACCDEC [ACC] [DEC]
//    This function sets the acc/dec value for the pathes 1 and 2
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalPosTuneTrajAccDecCommand(int drive)
{
   int ret_val = 0,index = 0,index_acc = 0;
   unsigned long long u64_temp = 0LL;
   if (BGVAR(s16_Pos_Tune_Traj_Enable)) return PTT_IS_ACTIVE;
   if (s16_Number_Of_Parameters > 2) return SYNTAX_ERROR;
   //Convert position to internal units

   index = UnitsConversionIndex(&s8_Units_Acc_Dec_For_Pos, drive);
   //u64_PosTuneTraj_AccDec[0] <= ACC
   //u64_PosTuneTraj_AccDec[1] <= DEC
   for(index_acc = 0; index_acc <= 1; index_acc++)
   {
       u64_temp = MultS64ByFixS64ToS64(s64_Execution_Parameter[index_acc],
                      BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                      BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);
       ret_val = CheckAccDecLimits(u64_temp);
       if(ret_val != SAL_SUCCESS) return ret_val;
       BGVAR(u64_PosTuneTraj_AccDec)[index_acc] = u64_temp;
   }

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalPosTuneTrajAccDecCommandViaPParam
// Description:
//    Please note - acc/dec for PATH1 and PATH2 are equal
//    0xAAAABBBB:
//    0xAAAA - ACC (6 ms - 65500 ms)
//    0xBBBB - DEC (6 ms - 65500 ms)
//    This function sets the acc/dec value for pathes 1 and 2 via P param
//    in Schneider's ACC DEC in ms. (6 - 65500)
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPosTuneTrajAccDecCommandViaPParam(long long param,int drive)
{
   int ret_val = 0;
   unsigned int u16_path_dec , u16_path_acc;
   unsigned long long u64_temp = 0LL;
   if (BGVAR(s16_Pos_Tune_Traj_Enable)) return PTT_IS_ACTIVE;
   //Convert position to internal units

   u16_path_dec = (unsigned int) (param & 0x0000FFFF);
   u16_path_acc = (unsigned int) ((param >> 16)& 0x0000FFFF);

   // check min/max for acc and dec (fix bug: IPR 340)
   if (u16_path_dec > MIN_ACC_DEC_IN_MS) return VALUE_TOO_HIGH;
   if (u16_path_dec < MAX_ACC_DEC_IN_MS) return VALUE_TOO_LOW;
   if (u16_path_acc > MIN_ACC_DEC_IN_MS) return VALUE_TOO_HIGH;
   if (u16_path_acc < MAX_ACC_DEC_IN_MS) return VALUE_TOO_LOW;

   //u64_PosTuneTraj_AccDec[0] <= ACC
   //u64_PosTuneTraj_AccDec[1] <= DEC

   // convert from ms to CDHD acc units and then use the regular CDHD ACC write function
   u64_temp = convertMsToAccDecInternalUnitsForPos(drive,(long long)u16_path_dec);
   ret_val = CheckAccDecLimits(u64_temp);
   if(ret_val != SAL_SUCCESS) return ret_val;
   BGVAR(u64_PosTuneTraj_AccDec)[1] = u64_temp;


   u64_temp = convertMsToAccDecInternalUnitsForPos(drive,(long long)u16_path_acc);
   ret_val = CheckAccDecLimits(u64_temp);
   if(ret_val != SAL_SUCCESS) return ret_val;
   BGVAR(u64_PosTuneTraj_AccDec)[0] = u64_temp;

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadPosTuneTrajAccDec
// Description:
//    This function returns trajectory ACC/DEC params for PATH 1 and 2
//    acc [units] dec [units]
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPosTuneTrajAccDec(int drive)
{
   long long s64_temp;
   int index;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for(index = 0; index <= 1; index++)
   {
       s64_temp = MultS64ByFixS64ToS64((long long)BGVAR(u64_PosTuneTraj_AccDec)[index],
                                       BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                       BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);
       PrintSignedLongLong(s64_temp);
       PrintChar(SPACE);
       PrintChar('[');
       PrintString((char *)s8_Units_Acc_Dec_For_Pos,0);
       PrintChar(']');
       PrintChar(SPACE);
   }
   PrintCrLf();

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadPosTuneTrajAccDecViaPParam
// Description:
//    Please note - ACC=DEC
//    0xAAAABBBB:
//    0xAAAA - Path2 ACC/DEC (1 ms - 65500 ms)
//    0xBBBB - Path1 ACC/DEC (1 ms - 65500 ms)
//    This function gets the acc/dec value for pathes 1 and 2 via P param
//    in Schneider's ACC DEC in ms. (1 - 65500)
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPosTuneTrajAccDecViaPParam(long long *data,int drive)
{
   unsigned int u16_path_dec , u16_path_acc;
   u16_path_dec = convertAccDecInternalUnitsForPosToMs(drive, BGVAR(u64_PosTuneTraj_AccDec)[1]);
   u16_path_acc = convertAccDecInternalUnitsForPosToMs(drive, BGVAR(u64_PosTuneTraj_AccDec)[0]);
   *data = (long long)(((long)u16_path_acc << 16)|((unsigned long)u16_path_dec & 0xFFFF));
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalPosTuneTrajPosCommand
// Description:
//    POSTUNETRAJPOS [path1] [path2]
//    This function sets the position value for the pathes 1 and 2
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************

int SalPosTuneTrajPosCommand(int drive)
{
   int index = 0,index_acc = 0;
   if (BGVAR(s16_Pos_Tune_Traj_Enable)) return PTT_IS_ACTIVE;
   if (s16_Number_Of_Parameters > 2) return SYNTAX_ERROR;
   //Convert position to internal units

   index = UnitsConversionIndex(&s8_Units_Pos, drive);
   for(index_acc = 0; index_acc <= 1; index_acc++)
   {
       BGVAR(s64_PosTuneTraj_Pos)[index_acc] = MultS64ByFixS64ToS64(s64_Execution_Parameter[index_acc],
                      BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                      BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);
   }

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadPosPosTuneTrajPos
// Description:
//   This function returns the Pos Tune positive trajectory position value in PUU units.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPosPosTuneTrajPos(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //convert from CDHD internal position units into PUU.
   *data = MultS64ByFixS64ToS64(BGVAR(s64_PosTuneTraj_Pos)[0],
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalPosTuneTrajPosPosCommand
// Description:
//   This function sets Pos Tune positive trajectory position value from PUU units.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPosTuneTrajPosPosCommand(long long param,int drive)
{
   BGVAR(s32_PUU_PT_Pos) = (long)param;
   RecalculatePUUPTPos(drive);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadNegPosTuneTrajPos
// Description:
//   This function returns the Pos Tune positive trajectory position value in PUU units.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadNegPosTuneTrajPos(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //convert from CDHD internal position units into PUU.
   *data = MultS64ByFixS64ToS64(BGVAR(s64_PosTuneTraj_Pos)[1],
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalPosTuneTrajNegPosCommand
// Description:
//   This function sets Pos Tune positive trajectory position value from PUU units.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPosTuneTrajNegPosCommand(long long param,int drive)
{
   BGVAR(s32_PUU_PT_Neg) = (long)param;
   RecalculatePUUPTPos(drive);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: RecalculatePUUPTPos
// Description:
//   This function recalculate Pos Tune trajectories position from last entered user value.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void RecalculatePUUPTPos(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //convert from PUU into CDHD internal position units and set it in the Positive Pos Tune variable.
   BGVAR(s64_PosTuneTraj_Pos)[0] = MultS64ByFixS64ToS64((long long)BGVAR(s32_PUU_PT_Pos),
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr);

   //convert from PUU into CDHD internal position units and set it in the Negative Pos Tune variable.
   BGVAR(s64_PosTuneTraj_Pos)[1] = MultS64ByFixS64ToS64((long long)BGVAR(s32_PUU_PT_Neg),
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix,
                      BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr);
}


//**********************************************************
// Function Name: SalReadPosTuneTrajPos
// Description:
//    This function returns trajectory position params for PATH 1 and 2
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPosTuneTrajPos(int drive)
{
   long long s64_temp;
   int index;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for(index = 0; index <= 1; index++)
   {
       s64_temp = MultS64ByFixS64ToS64(BGVAR(s64_PosTuneTraj_Pos)[index],
                                       BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                                       BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);
       PrintSignedLongLong(s64_temp);
       PrintChar(SPACE);
       PrintChar('[');
       PrintString((char *)s8_Units_Pos,0);
       PrintChar(']');
       PrintChar(SPACE);
   }
   PrintCrLf();

   return SAL_SUCCESS;
}
//**********************************************************
// Function Name: SalPosTuneTrajSpdCommand
// Description:
//    POSTUNETRAJPOS [path1] [path2]
//    This function sets the speed value for the pathes 1 and 2
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************

int SalPosTuneTrajSpdCommand(int drive)
{
   int index = 0,index_acc = 0;
   long long s64_profile_velocity = 0LL;
   if (BGVAR(s16_Pos_Tune_Traj_Enable)) return PTT_IS_ACTIVE;
   if (s16_Number_Of_Parameters > 2) return SYNTAX_ERROR;
   //Convert position to internal units

   index = UnitsConversionIndex(&s8_Units_Vel_Out_Loop, drive);
   for(index_acc = 0; index_acc <= 1; index_acc++)
   {
       s64_profile_velocity = MultS64ByFixS64ToS64(s64_Execution_Parameter[index_acc],
                      BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                      BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);
       if (s64_profile_velocity <= 0LL) return VALUE_TOO_LOW;
       if (s64_profile_velocity > (long long)BGVAR(s32_V_Lim_Design)) return VALUE_TOO_HIGH;
       BGVAR(u32_PosTuneTraj_Spd)[index_acc] = (unsigned long)s64_profile_velocity;
   }

   return SAL_SUCCESS;
}




//**********************************************************
// Function Name: SalPosTuneTrajSpdCommandViaPParam
// Description:
//    0xAAAABBBB:
//    0xAAAA - Path2 Speed 0 - 60000 (0.1*RPM)
//    0xBBBB - Path1 Speed 0 - 60000 (0.1*RPM)
//    This function sets the speed value for pathes 1 and 2 via P param
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalPosTuneTrajSpdCommandViaPParam(long long param,int drive)
{
   long long s64_profile_velocity = 0LL;
   unsigned int u16_path1_speed , u16_path2_speed;
   int value1_err = 0,  value2_err = 0;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s16_Pos_Tune_Traj_Enable)) return PTT_IS_ACTIVE;
   //Convert position to internal units

   u16_path1_speed = (unsigned int) (param & 0x0000FFFF);
   u16_path2_speed = (unsigned int) ((param >> 16)& 0x0000FFFF);

   s64_profile_velocity = MultS64ByFixS64ToS64((long long) u16_path1_speed,
                                          BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                          BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

   if (s64_profile_velocity <= 0LL) value1_err = VALUE_TOO_LOW;
   else
       if (s64_profile_velocity > (long long)BGVAR(s32_V_Lim_Design)) value1_err = VALUE_TOO_HIGH;

   if(value1_err == 0)
       BGVAR(u32_PosTuneTraj_Spd)[0] = (unsigned long)s64_profile_velocity;


   s64_profile_velocity = MultS64ByFixS64ToS64((long long) u16_path2_speed,
                                          BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                          BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr);

   if (s64_profile_velocity <= 0LL) value2_err =  VALUE_TOO_LOW;
   if (s64_profile_velocity > (long long)BGVAR(s32_V_Lim_Design)) value2_err = VALUE_TOO_HIGH;

   if(value2_err == 0)
      BGVAR(u32_PosTuneTraj_Spd)[1] = (unsigned long)s64_profile_velocity;

   //Report error only if both are wrong, If one value is OK - accept it quietly.
   // Values are checked again in SAL for hftunerefen
   if((value1_err != 0) && (value2_err != 0) )
       return value1_err;

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadPosTuneTrajSpd
// Description:
//    This function returns trajectory speed params for PATH 1 and 2
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPosTuneTrajSpd(int drive)
{
   long long s64_temp;
   int index;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for(index = 0; index <= 1; index++)
   {
       s64_temp = MultS64ByFixS64ToS64((long long)BGVAR(u32_PosTuneTraj_Spd)[index],
                                       BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                       BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);
       PrintSignedLongLong(s64_temp);
       PrintChar(SPACE);
       PrintChar('[');
       PrintString((char *)s8_Units_Vel_Out_Loop,0);
       PrintChar(']');
       PrintChar(SPACE);
   }
   PrintCrLf();

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: SalReadPosTuneTrajSpdViaPParam
// Description:
//    0xAAAABBBB:
//    0xAAAA - Path2 Speed 0 - 60000 (0.1*RPM)
//    0xBBBB - Path1 Speed 0 - 60000 (0.1*RPM)
//    This function return the speed value for pathes 1 and 2 via P param
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadPosTuneTrajSpdViaPParam(long long *data,int drive)
{
   unsigned int u16_path1_spd , u16_path2_spd;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   u16_path1_spd = (unsigned int) MultS64ByFixS64ToS64((long long) BGVAR(u32_PosTuneTraj_Spd)[0],
                                          BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                          BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

   u16_path2_spd = (unsigned int) MultS64ByFixS64ToS64((long long) BGVAR(u32_PosTuneTraj_Spd)[1],
                                          BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                          BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

   *data = (long long)(((long)u16_path2_spd << 16)|((unsigned long)u16_path1_spd & 0xFFFF));
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadHDtuneIntTrajInfo
// Description:
//    This function returns internal trajectory info for HDTUNEREFERENCE = 2
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalReadHDtuneIntTrajInfo(int drive)
{
   long long s64_temp_acc, s64_temp_vcruse, s64_temp_dist;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Since back and forth parameters are same then use first parameter only
   s64_temp_acc =       MultS64ByFixS64ToS64((long long)BGVAR(u64_PosTuneTraj_AccDec_Int)[0],
                                   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);

   s64_temp_vcruse =    MultS64ByFixS64ToS64((long long)BGVAR(u32_PosTuneTraj_Spd_Int)[0],
                                   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

   s64_temp_dist =      MultS64ByFixS64ToS64(BGVAR(s64_PosTuneTraj_Pos_Int)[0],
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);
   PrintChar(SPACE);

   // Print internal trajectory ACC/DEC
   PrintString("Acc/Dec: ",0);
   PrintSignedLongLong(s64_temp_acc);
   PrintChar(SPACE);
   PrintChar('[');
   PrintString((char *)s8_Units_Acc_Dec_For_Pos,0);
   PrintChar(']');
   PrintChar(',');
   PrintChar(SPACE);

   // Print internal trajectory VCRUISE
   PrintString("Vcruise: ",0);
   PrintSignedLongLong(s64_temp_vcruse);
   PrintChar(SPACE);
   PrintChar('[');
   PrintString((char *)s8_Units_Vel_Out_Loop,0);
   PrintChar(']');
   PrintChar(',');
   PrintChar(SPACE);

   // Print internal trajectory DISTANCE
   PrintString("Distance: ",0);
   PrintSignedLongLong(s64_temp_dist);
   PrintChar(SPACE);
   PrintChar('[');
   PrintString((char *)s8_Units_Pos,0);
   PrintChar(']');
   PrintChar(SPACE);
   PrintCrLf();

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteInternalProfileIncDistance
// Description:
//    This function writes the internal profile distance in PUU P10-33
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteInternalProfileIncDistance(long long param,int drive)
{
   BGVAR(s32_IntProfIncDist_UserUnits) = (long) param;
   RecalculateSchneiderInternalProfilePositionToGearRatio(drive);

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteInternalProfileVelocity
// Description:
//    This function writes the internal profile velocity P10-34
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteInternalProfileVelocity(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   BGVAR(u16_IntProfVel_UserUnits) = (unsigned int) param;
   // convert and set the speed value
   BGVAR(u16_IntProfVel_InternalUnits) = (BGVAR(u16_IntProfVel_UserUnits) * BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix)
                        >> (long)BGVAR(Unit_Conversion_Table[VELOCITY_0_1_RPM_OUT_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr;
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalReadInternalProfileAccDec
// Description:
//    This function returns the internal profile ACC and DEC rates P10-35
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalReadInternalProfileAccDec(long long *data,int drive)
{
   unsigned int u16_temp_acc = 0, u16_temp_dec = 0;

   u16_temp_acc = convertAccDecInternalUnitsForPosToMs(drive, BGVAR(u64_IntProf_InternalAccUnits));
   u16_temp_dec = convertAccDecInternalUnitsForPosToMs(drive, BGVAR(u64_IntProf_InternalDecUnits));

   *data = (long long)(((unsigned long)u16_temp_acc << 16) |(unsigned long)u16_temp_dec);
   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteInternalProfileAccDec
// Description:
//    This function writes the internal profile ACC and DEC rates P10-35
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteInternalProfileAccDec(long long param,int drive)
{
   unsigned int u16_temp_acc = 0, u16_temp_dec = 0;

   u16_temp_dec = (unsigned int) (param & 0xFFFF);
   u16_temp_acc = (unsigned int) ((param >> 16) & 0xFFFF);

   if((u16_temp_dec > MIN_ACC_DEC_IN_MS)||(u16_temp_acc > MIN_ACC_DEC_IN_MS))
   {
      return VALUE_TOO_HIGH;
   }
   else if((u16_temp_dec < MAX_ACC_DEC_IN_MS)||(u16_temp_acc < MAX_ACC_DEC_IN_MS))
   {
      return VALUE_TOO_LOW;
   }

   BGVAR(u32_IntProfAccDec_UserUnits) = (unsigned long) param;

   BGVAR(u64_IntProf_InternalAccUnits) = convertMsToAccDecInternalUnitsForPos(drive, u16_temp_acc);
   BGVAR(u64_IntProf_InternalDecUnits) = convertMsToAccDecInternalUnitsForPos(drive, u16_temp_dec);

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: SalWriteInternalProfileTrigger
// Description:
//    This function writes the internal profile trigger state P10-36
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteInternalProfileTrigger(long long param,int drive)
{
   int s16_access_rights_permission;


   // Check if it is allowed to trigger motion in PS mode according to the access-rights management
   s16_access_rights_permission = CheckLexiumAccessRights(drive, LEX_AR_FCT_ACTIVATE_PS, BGVAR(s16_Lexium_ExecPParam_Acces_Channel));
   if(s16_access_rights_permission != SAL_SUCCESS)
   {
      return (s16_access_rights_permission);
   }

   // check the new trigger value if it's valid
   if(param == 0LL)
   {// 0 means abort motion (at the end of the cycle)
      BGVAR(s16_Internal_Profile_Trigger) = 0;
      return SAL_SUCCESS;
   }


   // Check if currently an auto-tuning process or a jog-move is running. This code
   // has been introduced due to "BYang00000752" & "BYang00000753", in which
   // Juergen Fiess starts several kinds of motion with his commissioning tool
   // without aborting a different currently pending motion.
   if( ((BGVAR(u16_Lexium_Autotune_State) != LEX_ATUNE_STATE_IDLE) && (BGVAR(u16_Lexium_Autotune_State) < LEX_ATUNE_STATE_A_ADAPT_ACIVATE)) ||
       (BGVAR(u16_Lexium_Jog_Move_State)  != LEX_JOG_MOVE_IDLE) ||
       ((BGVAR(s16_Pos_Tune_State) > POS_TUNE_IDLE )  &&  BGVAR(s16_Pos_Tune_State) < POS_TUNE_DONE )  )
   {
      return OTHER_PROCEDURE_RUNNING;
   }

   if(BGVAR(u16_Internal_Profile_State) == INTERNAL_PROFILE_IDLE)
   {// allow trigger change only if state machine in idle
      BGVAR(s16_Internal_Profile_Trigger) = (int) param;
      BGVAR(s16_Internal_Profile_Trigger_Write_Indication) = 1;
      return SAL_SUCCESS;
   }
   return OTHER_PROCEDURE_RUNNING;
}

//**********************************************************
// Function Name: InternalProfileHandler
// Description:
//    This Handler call from Schneider BG ("LexiumBackgroundHandler") and deals with the internal profile at execution time
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void InternalProfileHandler(int drive)
{
   // AXIS_OFF;

   if(BGVAR(s16_Internal_Profile_Trigger_Write_Indication) == 1)
   {
      // new value was written
      BGVAR(s16_Internal_Profile_Trigger_Write_Indication) = 0;                   // clear the write indication
      BGVAR(s16_Internal_Profile_Counter) = BGVAR(s16_Internal_Profile_Trigger);  // copy user value to the counter
      BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_INIT;                  // trigger motion
   }

   if ( ((!Enabled(drive)) || BGVAR(u16_Hold))                       &&
        (BGVAR(u16_Internal_Profile_State) != INTERNAL_PROFILE_IDLE) &&
        (BGVAR(u16_Internal_Profile_State) != INTERNAL_PROFILE_INIT)      )
   {
      // If an OPMODE change has been requested via involving the new state machine handler
      if( (BGVAR(u16_Internal_Profile_State) == INTERNAL_PROFILE_EXECUTE_FW_MOVE) &&
          IsOpmodeChangeInProgress(drive) )
      {
         // Do nothing if the Opmode change background handler issued a HOLD,
         // because this is part of the OPMODE change procedure.
      }
      else
      {
         BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_TERMINATE;
      }
   }

   if ( (BGVAR(s16_Internal_Profile_Trigger) == 0)                        &&
        (BGVAR(u16_Internal_Profile_State) != INTERNAL_PROFILE_IDLE)      &&
        (BGVAR(u16_Internal_Profile_State) != INTERNAL_PROFILE_INIT)      &&
        (BGVAR(u16_Internal_Profile_State) != INTERNAL_PROFILE_TERMINATE) &&
        (BGVAR(u16_Internal_Profile_State) != INTERNAL_PROFILE_ABORT_END)     )
   {
      BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_ABORT;
   }

   switch(BGVAR(u16_Internal_Profile_State))
   {
      case INTERNAL_PROFILE_IDLE:
      break;

      case INTERNAL_PROFILE_INIT:
         // Save current opmode and change opmode to position

         if( ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_SERCOS)        ||
             ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERNET_IP)   ||
             ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERCAT)         )
         {
         }
         else if ((BGVAR(u16_P1_01_CTL_Current) & 0xff) == SE_OPMODE_CANOPEN)
         {
            // Capture the mode of operation
            BGVAR(s16_Lexium_Internal_Profile_Opmode_Backup) = p402_modes_of_operation_display;

            // Here try to switch the canopen opmode to 1
            BGVAR(s16_CAN_Opmode_Temp) = 1;  // 1 is profile position (internal opmode 8)
            BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
            CheckCanOpmode(drive);
         }
         else
         {
            // Capture the mode of operation
            BGVAR(s16_Lexium_Internal_Profile_Opmode_Backup) = VAR(AX0_s16_Opmode);
            // Here try to switch the OPMODE to 8.
            SetOpmode(drive, 8);
         }

         // capture current position (use abs moves because many back and forth inc moves might drift)
         BGVAR(s64_Current_Pos_Backup) = LLVAR(AX0_u32_Pcmd_Internal_After_Mod_Lo);

         // advance state machine
         BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_EXECUTE_FW_MOVE;
      break;

      case INTERNAL_PROFILE_EXECUTE_FW_MOVE:

         // Wait until the OPMODE change, initiated in the state before, is finished.
         if(IsOpmodeChangeInProgress(drive))
         {
            // Do nothing if OPMODE-change is in progress
         }
         else
         {
         BGVAR(s16_Move_Abs_Issued) = 0;
         BGVAR(s16_Sal_Move_Ind) = 1;

         // backup system acc/dec
         BGVAR(u64_Temp_Acc) = BGVAR(u64_AccRate);
         BGVAR(u64_Temp_Dec) = BGVAR(u64_DecRate);

         // use acc / dec of the internal profile
         SalAccCommand(BGVAR(u64_IntProf_InternalAccUnits), drive);
         SalDecCommand(BGVAR(u64_IntProf_InternalDecUnits), drive);

         ExecuteMoveCommand(drive, BGVAR(s64_IntProfIncDist_InternalUnits) , BGVAR(u16_IntProfVel_InternalUnits), PTP_IMMEDIATE_TRANSITION);

         // restore system acc/dec
         SalAccCommand(BGVAR(u64_Temp_Acc), drive);
         SalDecCommand(BGVAR(u64_Temp_Dec), drive);

         // advance state machine
         BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_WAIT_TO_FW_MOVE_END;
         }
      break;

      case INTERNAL_PROFILE_WAIT_TO_FW_MOVE_END:
         if( (VAR(AX0_s16_Stopped) >= 1) || (VAR(AX0_s16_Stopped) == -1) || (BGVAR(u16_Ptp_Abort_Flags) & 0x0001) )
         {
            if(BGVAR(s16_Internal_Profile_Counter) == -1)
            {
               BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_TERMINATE;
            }
            else
            {
               BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_EXECUTE_BW_MOVE;
            }
         }
      break;

      case INTERNAL_PROFILE_EXECUTE_BW_MOVE:

         BGVAR(s16_Move_Abs_Issued) = 1;   // go back with absolute move
         BGVAR(s16_Sal_Move_Ind) = 1;

         // backup system acc/dec
         BGVAR(u64_Temp_Acc) = BGVAR(u64_AccRate);
         BGVAR(u64_Temp_Dec) = BGVAR(u64_DecRate);

         // use acc / dec of the internal profile
         SalAccCommand(BGVAR(u64_IntProf_InternalAccUnits), drive);
         SalDecCommand(BGVAR(u64_IntProf_InternalDecUnits), drive);

         ExecuteMoveCommand(drive, BGVAR(s64_Current_Pos_Backup) , BGVAR(u16_IntProfVel_InternalUnits), PTP_IMMEDIATE_TRANSITION);

         // restore system acc/dec
         SalAccCommand(BGVAR(u64_Temp_Acc), drive);
         SalDecCommand(BGVAR(u64_Temp_Dec), drive);

         // advance state machine
         BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_WAIT_TO_BW_MOVE_END;
      break;

      case INTERNAL_PROFILE_WAIT_TO_BW_MOVE_END:
         if( (VAR(AX0_s16_Stopped) >= 1) || (VAR(AX0_s16_Stopped) == -1) || (BGVAR(u16_Ptp_Abort_Flags) & 0x0001) )
         {
            BGVAR(s16_Internal_Profile_Counter)--;

            if (BGVAR(s16_Internal_Profile_Counter) == 0)
            {
               BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_TERMINATE;
            }
            else
            {
               BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_EXECUTE_FW_MOVE;
            }
         }
      break;

      case INTERNAL_PROFILE_ABORT:

         BGVAR(s16_Sal_Move_Ind) = 1;

         // backup system acc/dec
         BGVAR(u64_Temp_Acc) = BGVAR(u64_AccRate);
         BGVAR(u64_Temp_Dec) = BGVAR(u64_DecRate);

         // use acc / dec of the internal profile
         SalAccCommand(BGVAR(u64_IntProf_InternalAccUnits), drive);
         SalDecCommand(BGVAR(u64_IntProf_InternalDecUnits), drive);

         PtpAbort(drive);

         // restore system acc/dec
         SalAccCommand(BGVAR(u64_Temp_Acc), drive);
         SalDecCommand(BGVAR(u64_Temp_Dec), drive);

         // advance state machine
         BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_ABORT_END;
      break;

      case INTERNAL_PROFILE_ABORT_END:
         if( (VAR(AX0_s16_Stopped) >= 1) || (VAR(AX0_s16_Stopped) == -1) )
         {
            BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_TERMINATE;
         }
      break;

      case INTERNAL_PROFILE_TERMINATE:
         // Restore opmode.

         // Here restore the previos OPMODE.
         if( ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_SERCOS)        ||
             ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERNET_IP)   ||
             ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERCAT)         )
         {
         }
         else if ((BGVAR(u16_P1_01_CTL_Current) & 0xff) == SE_OPMODE_CANOPEN)
         {
            BGVAR(s16_CAN_Opmode_Temp) = BGVAR(s16_Lexium_Internal_Profile_Opmode_Backup);
            BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
            CheckCanOpmode(drive);
         }
         else
         {
            SetOpmode(drive, BGVAR(s16_Lexium_Internal_Profile_Opmode_Backup));
         }

         // reset state machine
         BGVAR(u16_Internal_Profile_State) = INTERNAL_PROFILE_IDLE;
      break;

      default:
      break;
   }
}

//**********************************************************
// Function Name: AutoRHandler
// Description:
//    This Handler call from Schneider BG ("LexiumBackgroundHandler") and deals with the AutoR feature
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void AutoRHandler(int drive)
{
   // AXIS_OFF;
   if ((VAR(AX0_s16_Opmode) != 8) ||
      (!Enabled(drive)) ||
      (BGVAR(u8_Homing_State) != HOMING_TARGET_REACHED)) return; // valid only in PS mode and after home completed

   // Update AutoR inputs conditions
   ProcessAutoRInputs(drive);
   switch(BGVAR(u16_AutoR_State))
   {
      case AUTOR_IDLE_STAT:
         // do nothing for now
         break;

      case AUTOR_EXEC_STAT:
      {// run the PATHs icrementaly according to BGVAR(u16_AutoR_Running_Path)
         AutoRPathExec(drive, 0);
         BGVAR(u16_AutoR_State) = AUTOR_RUN_STAT;
      }// continue to the next case automaticly
      //break;

      case AUTOR_RUN_STAT:
      {// When PATH is finished,
       // run the next PATH icrementaly according to BGVAR(u16_AutoR_Running_Path)
         if(BGVAR(u16_AutoR_Running_Path) == BGVAR(u16_AutoR_Finished_Path))
         {
            BGVAR(u16_AutoR_Running_Path)++;
            if(BGVAR(u16_AutoR_Running_Path) >= NUM_OF_PATHS)
            {// roll over and start from path 1
               BGVAR(u16_AutoR_Running_Path) = 1;
            }
            // execute the next PATH
            BGVAR(u16_AutoR_State) = AUTOR_EXEC_STAT;
         }
      }
      break;

      default:
         break;
   }
}

//**********************************************************
// Function Name: AutoRPathExec
// Description:
//    This function execute the PATH index from "u16_AutoR_Running_Path"
//    The "insertion" flag indicate if the path need to be preformed immediately or
//    when the current path is finished.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void AutoRPathExec(int drive, int insertion)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Check if it is allowed to trigger motion in PS mode according to the access-rights management
   // if there are no access-rights for starting motion in mode PS.
   if(CheckLexiumAccessRights(drive, LEX_AR_FCT_ACTIVATE_PS, BGVAR(s16_Lexium_ExecPParam_Acces_Channel)) != SAL_SUCCESS) return;

   BGVAR(u16_AutoR_Immidiate_Path_Exec) = insertion;
   BGVAR(s16_Path_Internal_Execution) = BGVAR(u16_AutoR_Running_Path);
   BGVAR(u16_Unhandled_Path_Internal_Execution) = 1;
   PathHandler(drive);
}
