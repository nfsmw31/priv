#include "DSP2834x_Device.h"
#include "Math.h"
#include "cal_conf.h"
#include "objects.h"

#include "Current.def"
#include "ModCntrl.def"
#include "FltCntrl.def"
#include "SysInit.def"
#include "Err_Hndl.def"
#include "design.def"
#include "MultiAxis.def"
#include "Ser_Comm.def"
#include "402fsa.def"
#include "FPGA.def"
#include "Exe_IO.def"
#include "PtpGenerator.def"
#include "Init.def"

#include "Motor.var"
#include "Drive.var"
#include "ModCntrl.var"
#include "FltCntrl.var"
#include "Burnin.var"
#include "Extrn_Asm.var"
#include "Ser_Comm.var"
#include "An_Fltr.var"
#include "Units.var"
#include "MotorSetup.var"
#include "MotorParamsEst.var"
#include "PtpGenerator.var"
#include "Exe_IO.var"
#include "Prototypes.pro"
#include "Position.pro"
#include "i2c.def"
#include "User_Var.var"
#include "AutoTune.var"


void UpdateCurrentLimit(int drive)
{
   long s32_ilim_min_value = 0,s32_temp;
   // AXIS_OFF;

   unsigned int u16_shr;
   long long s64_data,s64_fix, s64_half_for_rounding;

   if (VAR(AX0_s16_An_In_2_Mode) == 2) //if the chosen mode is current limit
   {
      s64_fix = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix;
      u16_shr = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr;
      s64_half_for_rounding = 0LL;
      if (u16_shr > 0) s64_half_for_rounding = 1LL << (u16_shr - 1);
      BGVAR(s32_Ilim_Analog) = (long)((VAR(AX0_u16_Analog_Crrnt_Limit) * s64_fix + s64_half_for_rounding) >> u16_shr);


      if(BGVAR(s32_Ilim_Analog) < BGVAR(s32_Ilim_User))
         BGVAR(s32_Ilim_Actual) = BGVAR(s32_Ilim_Analog);
      else
         BGVAR(s32_Ilim_Actual) = BGVAR(s32_Ilim_User);
   }
   else
   {

      //At CAN (Schneider),when SDO 0x605d content,equals 3,
      //Slow down on current limit and stay in Operation Enabled.
      //The current is limited by the miniumum out of (user current,mipeak,dipeak)
      if (((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)) && (p402_halt_option_code == 3) && (p402_controlword & HALT_MOVE))
      {
         s32_ilim_min_value = min(min(BGVAR(s32_Motor_I_Peak),BGVAR(s32_Drive_I_Peak)),BGVAR(s32_P1_70_IMAXHALT));
         BGVAR(s32_Ilim_Actual) = s32_ilim_min_value;

      }
      else
         BGVAR(s32_Ilim_Actual) = BGVAR(s32_Ilim_User);
   }

   if(BGVAR(s32_Ilim_Active_Disable) < BGVAR(s32_Ilim_Actual))
      BGVAR(s32_Ilim_Actual) = BGVAR(s32_Ilim_Active_Disable);

   if(BGVAR(s32_Derated_Drive_I_Peak) < BGVAR(s32_Ilim_Actual))
      BGVAR(s32_Ilim_Actual) = BGVAR(s32_Derated_Drive_I_Peak);

   CalcILim(DRIVE_PARAM);

   // if there was a large change in ilim actual than force pos loop config - to update AW - when ilim actual is below micont
   if ((BGVAR(s32_Ilim_Actual) > BGVAR(s32_Motor_I_Cont))     &&
       (BGVAR(s32_Ilim_Actual_Prev) > BGVAR(s32_Motor_I_Cont))   ) BGVAR(s32_Ilim_Actual_Prev) = BGVAR(s32_Ilim_Actual);
   else
   {
      s32_temp = labs(BGVAR(s32_Ilim_Actual_Prev) - BGVAR(s32_Ilim_Actual)) - (long)(0.2*(float)BGVAR(s32_Motor_I_Cont));

      if ( (s32_temp > 0)                                                        ||
           ( (BGVAR(s32_Ilim_Actual_Prev) <= 0) && (BGVAR(s32_Ilim_Actual) > 0) )  )
      { // Run Position-Tuning only if ILIM changed from Zero, of a change of more than 0.2*ICONT.
         BGVAR(s32_Ilim_Actual_Prev) = BGVAR(s32_Ilim_Actual);
         PosTuneActiveCommand(&s64_data,drive); // Check if the auto-tuner is active or done
         if (s64_data != 1) PositionConfig(drive, 0);
      }
   }
}


// Limit the current as function of IPM temperature.
// Perform the algorithm once per second (because the temperature low time constant), if HW supports this feature.
void CurrentDerating(int drive)
{
   // AXIS_OFF;
   unsigned int u16_factor;
   REFERENCE_TO_DRIVE;
   if (VAR(AX0_u16_IPM_OT_DC_Enable) == 1)
   {
      if (PassedTimeMS(1000L,BGVAR(s32_Current_Derating_Timer)))
      {
         BGVAR(s32_Current_Derating_Timer) = Cntr_1mS;

         // If the IPM temperature is above threshold
         if (BGVAR(u16_IPM_Temperature) > BGVAR(u16_Current_Derating_Start_Temp))
         {
            // Derated current = [I * (1 - X*deltaT/1000)] = [I * (1000 - X*deltaT) /1000]
            // First calculate X*deltaT, and limit by 900
            u16_factor = BGVAR(u16_Current_Derating_Slope) * (BGVAR(u16_IPM_Temperature) - BGVAR(u16_Current_Derating_Start_Temp)); // practically, no numeric issue is expected
            if (u16_factor > 900)
               u16_factor = 900;

            u16_factor = 1000 - u16_factor;

            BGVAR(s32_Derated_Drive_I_Peak) = (BGVAR(s32_Drive_I_Peak) * u16_factor) /1000;
            BGVAR(s32_Derated_Drive_I_Cont) = (BGVAR(s32_Drive_I_Cont) * u16_factor) /1000;
         }
         else
         {
            BGVAR(s32_Derated_Drive_I_Peak) = BGVAR(s32_Drive_I_Peak);
            BGVAR(s32_Derated_Drive_I_Cont) = BGVAR(s32_Drive_I_Cont);
         }
      }
   }
}


void CalcPolesRatio(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if ( (BGVAR(u16_MotorType) == LINEAR_MOTOR)     ||
        (BGVAR(u16_MotorType) == DC_BRUSH_MOTOR)/* ||
        (SL_FEEDBACK)*/                             )
      VAR(AX0_s16_Num_Of_Poles_Ratio) = 1;
   else
   {
      if (BGVAR(u16_FdbkType) == RESOLVER_FDBK)
         VAR(AX0_s16_Num_Of_Poles_Ratio) = (int)(BGVAR(u16_Mpoles) / BGVAR(u16_Resolver_Poles));
      else
         VAR(AX0_s16_Num_Of_Poles_Ratio) = BGVAR(u16_Mpoles) >> 1;
   }
}


int CrrntConfigValidation(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_Drive_I_Peak) == 0)
      return (CONFIG_FAIL_DIPEAK);

   if ((BGVAR(s32_Motor_I_Peak) == 0) || (BGVAR(s32_Motor_I_Peak) > 300000))
      return (CONFIG_FAIL_MIPEAK);

   if ((BGVAR(s32_Motor_I_Cont) > BGVAR(s32_Motor_I_Peak)))
      return (MIPEAK_MICONT_CONFLICT);

   if ((BGVAR(s16_Vbus) <= 1) || (BGVAR(s16_Vbus) > 0x7fff))
      return (CONFIG_FAIL_VBUS);

   if ((BGVAR(u32_Mlmin) == 0) || (BGVAR(u32_Mlmin) > 1000000))
      return (CONFIG_FAIL_MLMIN);

   if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
   {
      if ((BGVAR(s32_Motor_Mass) == 0L) || (BGVAR(s32_Motor_Mass) > 100000L))
         return (CONFIG_FAIL_MMASS);
   }
   else
   {
      if ((BGVAR(s32_Motor_J) == 0) || (BGVAR(s32_Motor_J) > 2000000000))
         return (CONFIG_FAIL_MJ);
   }

   if ((BGVAR(u16_Mpoles) == 0) || (BGVAR(u16_Mpoles) > 0x7fff))
      return (CONFIG_FAIL_MPOLES);

   if ((BGVAR(u32_User_Motor_Enc_Res) <= 0) && (FEEDBACK_WITH_ENCODER))
      return (CONFIG_FAIL_MENCRES);

   if (BGVAR(s32_Drive_I_Cont) > BGVAR(s32_Drive_I_Peak))
        return (DIPEAK_DICONT_CONFLICT);

   return (SUCCESS);
}


int CrrntConfig(int drive)
{
   //// AXIS_OFF;
   unsigned int u16_result;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   u16_result = CrrntConfigValidation(DRIVE_PARAM);

   if ( u16_result == SUCCESS )
   {
      CalcPolesRatio(DRIVE_PARAM);
      u16_result = CrrntContDesign(DRIVE_PARAM);
      if ( u16_result != SUCCESS ) return (u16_result);

      CalcCurrentAdaptiveGain(DRIVE_PARAM);

      TorquePhaseAdvDesign(DRIVE_PARAM);
      SpeedPhaseAdvDesign(DRIVE_PARAM);

      AnalogTorqueConfig1(DRIVE_PARAM);
      AnalogTorqueConfig2(DRIVE_PARAM);

      CalcDynamicBrakingGain(DRIVE_PARAM);
      return (SUCCESS);
   }
   return (u16_result);
}


int CrrntContDesign(int drive)
{
   // AXIS_OFF;
   float f_sample_time,f_cl_kp,f_cl_kff,f_cl_ki,f_cl_kiv,f_cl_kp_scale,f_cl_bemfcomp,f_temp,f_vlim_rpm = (float)BGVAR(s32_V_Lim_Design) * 0.00011176,
         f_cl_coef_com,f_cl_coef_ld,f_cl_coef_lq;

   int s16_cl_kp_fix,s16_cl_kp_shr,s16_cl_kci_fix,s16_cl_kciv_fix,s16_cl_kci_shr,s16_cl_kciv_shr,s16_mres_fix,
       s16_mres_shr,s16_dead_time_comp,s16_kci_kciv_shr,s16_cl_coef_vq_fix,s16_cl_coef_vq_shr,s16_cl_coef_vd_fix,
       s16_cl_coef_vd_shr,s16_bemf_fix,s16_bemf_shr,s16_mem_shr,s16_cl_kff_fix,s16_cl_kff_shr,cl_kp_kff_shr,
       max_shr_cnt,s16_dt_comp_level,s16_adapt_vg_slope;
   unsigned int u16_pwm_half_period_sat;   
   long s32_temp, u32_Mlmin_design, s32_pwm_half_period_sqr;
   REFERENCE_TO_DRIVE;
   BGVAR(s16_Vbus_Design) = BGVAR(s16_Vbus);
   AX0_s16_Vbus_RT_Factor = 16384;
   if (BGVAR(s16_Adaptive_Vbus) != 0) BGVAR(s16_Vbus_Design) = (int)((float)BGVAR(s16_Vbus_Design) * 0.75);

   u32_Mlmin_design = (long)((float)BGVAR(s32_CL_User_Gain)*(float)BGVAR(u32_Mlmin)/1000.0);

   f_sample_time = (4686.0/(float)VAR(AX0_s16_Pwm_Half_Period))/31250;
   f_cl_kp_scale = (float)BGVAR(s32_Drive_I_Peak)/26214000*(float)VAR(AX0_s16_Pwm_Half_Period)/(float)BGVAR(s16_Vbus_Design);
   f_temp = (float)u32_Mlmin_design*f_sample_time*f_cl_kp_scale;

   f_cl_kp   = ((float)BGVAR(s32_CL_Kp_User))*f_temp*CL_KP_FACTOR;
   f_cl_ki   = (float)BGVAR(s32_CL_Ki_User)*f_temp*CL_KI_FACTOR;
   if (BGVAR(s32_CL_Kiv_User) == 0L) f_cl_kiv  = (float)BGVAR(s32_CL_Ki_User)*f_temp*CL_KIV_FACTOR;
   else f_cl_kiv  = (float)BGVAR(s32_CL_Kiv_User)*f_temp*CL_KIV_FACTOR;
   f_cl_kff   = ((float)BGVAR(s32_CL_Kff_User))*f_temp*CL_KFF_FACTOR;

   switch (VAR(AX0_S16_Kc_Mode))
   {
      case 0: default: // KCMODE 0
         //do nothing
      break;

      case 1: // KCMODE 1
         f_cl_kff = f_cl_kff * 1.5244;
         f_cl_ki  = f_cl_ki * 0.706;
         f_cl_kiv = 0;
         f_cl_kp  = f_cl_kp *0.424 ;
      break;

      case 2: // KCMODE 2
         f_cl_kp  = f_cl_kp * 0.5 ; //KCP adaptive gain support(KCVG)
      break;

      case 3: // KCMODE 3
         // Check if shneider product, if no - use gains from kcmode 2
         if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
         {
            f_cl_kff = f_cl_kff * 0.516;
            f_cl_kp  = f_cl_kp * 0.2645 ; // 0.5125 * 0.516 = 0.2645
            f_cl_ki  = f_cl_ki * 1.419; // 2.75 * 0.516 = 1.419
            if (BGVAR(s32_CL_Kiv_User) == 0L) f_cl_kiv  = f_cl_kiv * 0.516;
         }
         else
         {
            f_cl_kp = f_cl_kp * 0.5 ; //KCP adaptive gain support(KCVG)
         }
      break;

      case 4: // with no id iq eq
      case 5: // improved torque at sat
      case 6: // 5+ used icmd for compensations
         f_cl_kp = f_cl_kp * 0.5 ; //KCP adaptive gain support(KCVG)
      break;
   }

   FloatToFix16Shift16(&s16_cl_kp_fix,(unsigned int *)&s16_cl_kp_shr,f_cl_kp);
   FloatToFix16Shift16(&s16_cl_kff_fix,(unsigned int *)&s16_cl_kff_shr,f_cl_kff);

   if (f_cl_kp == 0)
   {
      cl_kp_kff_shr = s16_cl_kff_shr;
      s16_cl_kp_fix = 0;
   }
   else if (f_cl_kff == 0)
   {
      cl_kp_kff_shr = s16_cl_kp_shr;
      s16_cl_kff_fix = 0;
   }
   else
   {
      cl_kp_kff_shr = s16_cl_kp_shr;
      if (cl_kp_kff_shr > s16_cl_kff_shr) cl_kp_kff_shr = s16_cl_kff_shr;
      s16_cl_kp_shr -= cl_kp_kff_shr;
      s16_cl_kff_shr -=cl_kp_kff_shr;
      s16_cl_kff_fix >>= s16_cl_kff_shr;
      s16_cl_kp_fix >>= s16_cl_kp_shr;
   }

   FloatToFix16Shift16(&s16_cl_kci_fix,(unsigned int *)&s16_cl_kci_shr,f_cl_ki);
   FloatToFix16Shift16(&s16_cl_kciv_fix,(unsigned int *)&s16_cl_kciv_shr,f_cl_kiv);
   if (f_cl_ki == 0)
   {
      s16_cl_kci_fix = 0;
      s16_kci_kciv_shr =  s16_cl_kciv_shr;
   }
   else if (f_cl_kiv == 0)
   {
      s16_cl_kciv_fix = 0;
      s16_kci_kciv_shr =  s16_cl_kci_shr;
   }
   else
   {
      s16_kci_kciv_shr = s16_cl_kci_shr;
      if (s16_cl_kciv_shr < s16_kci_kciv_shr) s16_kci_kciv_shr = s16_cl_kciv_shr;
      s16_cl_kciv_shr -= s16_kci_kciv_shr;
      s16_cl_kci_shr   -= s16_kci_kciv_shr;
      s16_cl_kci_fix >>= s16_cl_kci_shr;
      s16_cl_kciv_fix   >>= s16_cl_kciv_shr;
   }
   // Calc max shifts according to PWM frequency
   max_shr_cnt = 0;
   s32_temp = (long)VAR(AX0_s16_Pwm_Half_Period);
   if(s32_temp <= 0L) s32_temp = 0x40000000; // avoid infinite loop
   while((s32_temp & 0x80000000) == 0L)
   {
      s32_temp <<= 1L;
      ++max_shr_cnt;
   };
   max_shr_cnt -=2; // spare for sat

   if (s16_kci_kciv_shr < max_shr_cnt)
   {
      s16_mem_shr =  s16_kci_kciv_shr;
      s16_kci_kciv_shr = 0;
   }
   else
   {
      s16_mem_shr = max_shr_cnt;
      s16_kci_kciv_shr = s16_kci_kciv_shr - s16_mem_shr;
   }

   // dead time compensation
   s16_dead_time_comp = (int)(0.7*(float)BGVAR(u16_Dead_Time)*(f_sample_time/2.0)*(float)VAR(AX0_s16_Pwm_Half_Period));

   // cross compensation
   f_cl_coef_com = (((float)BGVAR(u16_Mpoles)*(float)BGVAR(s32_V_Lim_Design))/3887777504.0)*(float)BGVAR(s32_Drive_I_Peak)/26214000*(float)VAR(AX0_s16_Pwm_Half_Period)/(float)BGVAR(s16_Vbus_Design);
   f_cl_coef_ld = (float)BGVAR(u32_Mlmin)/1000000.0; //internal to physical units [Henry]
   f_cl_coef_lq = f_cl_coef_ld;// Currently we assume Lq = Ld
   FloatToFix16Shift16(&s16_cl_coef_vq_fix,(unsigned int *)&s16_cl_coef_vq_shr,f_cl_coef_ld*f_cl_coef_com*(float)BGVAR(s32_CL_DQ_Axis_Comp_Gain)/1000.0);
   FloatToFix16Shift16(&s16_cl_coef_vd_fix,(unsigned int *)&s16_cl_coef_vd_shr,f_cl_coef_lq*f_cl_coef_com*(float)BGVAR(s32_CL_DQ_Axis_Comp_Gain)/1000.0);

   // calc mres_fix,shr = mres[ohm]/0x6666*dipeak/vbus*pwm_half_period
   f_temp = (float)BGVAR(u32_Motor_Res)/1000.0 * (float)BGVAR(s32_Drive_I_Peak)/26214000*(float)VAR(AX0_s16_Pwm_Half_Period)/(float)BGVAR(s16_Vbus_Design);
   FloatToFix16Shift16(&s16_mres_fix,(unsigned int *)&s16_mres_shr,f_temp);

   // bemf compnsation
   if((BGVAR(u16_MotorType) == LINEAR_MOTOR))
      f_cl_bemfcomp = ((float)BGVAR(u32_Motor_Kf)*(float)BGVAR(u32_Mpitch)*0.000159)/1000.0*f_vlim_rpm/22750*(float)VAR(AX0_s16_Pwm_Half_Period)/(float)BGVAR(s16_Vbus_Design)/1000.0; //mpitch changed to decimal
   else
      f_cl_bemfcomp = (float)BGVAR(u32_Motor_Kt)*1.4142/1000.0*f_vlim_rpm/22750*(float)VAR(AX0_s16_Pwm_Half_Period)/(float)BGVAR(s16_Vbus_Design);

   FloatToFix16Shift16(&s16_bemf_fix,(unsigned int *)&s16_bemf_shr,(f_cl_bemfcomp*(float)BGVAR(s32_CL_Bemf_Gain)*CL_BEMF_FACTOR/10000.0));

   if (s16_bemf_shr > 31)
   {
      s16_bemf_fix = 0;
      s16_bemf_shr = 0;
   }

   // support est motor params
   BGVAR(f_Current_Loop_Gains_Sum) = f_cl_kff+f_cl_kp+f_cl_ki+f_cl_kiv;

   // dead time comp level support (KCD)
   f_temp = ((float)BGVAR(s16_Vbus))/((float)BGVAR(u32_Mlmin)/1000000.0);
   f_temp = 48*f_temp/28703.7;
   if (f_temp >= 48*1.333) f_temp = 48*1.333;
   if (f_temp < 48) f_temp = 48;

   if (VAR(AX0_S16_Kc_Mode) == 1)
      s16_dt_comp_level = 32767;
   else if ((VAR(AX0_S16_Kc_Mode) >  5)                                                                              && 
            (u16_Product != SHNDR)                                                                                   && 
            (u16_Product != SHNDR_HW)                                                                                && 
            ((BGVAR(s16_Bundle_Design_Lock) <  0) || (BGVAR(s16_Bundle_Design_Lock) > CDHD_PRO2_MOTOR_DATA_ENTRIES))   )
   {  // bugzilla 3273  - fix kcd scaling based on data from SE motors - fix for non bundle data
      s32_temp = BGVAR(s32_Drive_I_Peak);
      if (s32_temp < 12000) s32_temp = 12000;

      s16_dt_comp_level = (int)(2*f_temp*(float)BGVAR(u32_Kcd_Gain)*26214.0/(((float)s32_temp)*1000.0));
   }
   else
      s16_dt_comp_level = (int)(f_temp*(float)BGVAR(u32_Kcd_Gain)*26214.0/(((float)BGVAR(s32_Drive_I_Peak))*1000.0));

   // KCP adaptive gain support(KCVG)   ; 2^14=16384
   if ((s16_dt_comp_level !=0) && (VAR(AX0_S16_Kc_Mode) > 1))
       f_temp = 16384.0*(((float)BGVAR(s32_CL_Kvg_User)/1000.0-1)/((float)s16_dt_comp_level));
   else
       f_temp = 0;

   s16_adapt_vg_slope = (int)(f_temp);

   f_temp = 0.00115 * (float)BGVAR(s32_PWM_Sat_Factor) * (float)VAR(AX0_s16_Pwm_Half_Period);
   if (f_temp > 37488) f_temp = 37488;// general numerical protection 2*18744 (4 kHz case)
   u16_pwm_half_period_sat = (unsigned int)f_temp;
   // 37488*37488 = 1,405,350,144 < 2^31
   s32_pwm_half_period_sqr = (long)u16_pwm_half_period_sat*(long)u16_pwm_half_period_sat;

   if((cl_kp_kff_shr > 31)||(s16_kci_kciv_shr > 31)||(s16_mem_shr > 31)||
      (s16_cl_coef_vq_shr > 63)||(s16_cl_coef_vd_shr > 63)||
      (s16_mres_shr > 31)||(s16_bemf_shr > 31)||
      (cl_kp_kff_shr < 0)||(s16_kci_kciv_shr < 0)||(s16_mem_shr < 0)||
      (s16_cl_coef_vq_shr < 0)||(s16_cl_coef_vd_shr < 0)||
      (s16_mres_shr < 0)||(s16_bemf_shr < 0)) return (CONFIG_INTERNAL_FAIL);

   if (VAR(AX0_S16_Kc_Mode) > 3)
   {
      s16_cl_coef_vq_fix =-s16_cl_coef_vq_fix;
      s16_cl_coef_vd_fix =-s16_cl_coef_vd_fix;
   }
   // update RT variables
   WaitForNext32kHzTaskStart();
   VAR(AX0_s16_Crrnt_Coef_Kp_Fix)               = s16_cl_kp_fix;
   VAR(AX0_s16_Crrnt_Coef_Kff_Fix)              = s16_cl_kff_fix;
   VAR(AX0_s16_Crrnt_Coef_Kp_Kff_Shr)           = cl_kp_kff_shr;

   VAR(AX0_s16_Crrnt_Coef_Kiv_Fix)              = s16_cl_kciv_fix;
   VAR(AX0_s16_Crrnt_Coef_Ki_Fix)               = s16_cl_kci_fix;
   VAR(AX0_s16_Crrnt_KiKiv_Shr)                 = s16_kci_kciv_shr;
   VAR(AX0_s16_Crrnt_Coef_Mem_Shr)              = s16_mem_shr;

   VAR(AX0_s16_Crrnt_Coef_Vq_Fix)               = s16_cl_coef_vq_fix;
   VAR(AX0_s16_Crrnt_Coef_Vq_Shr)               = s16_cl_coef_vq_shr;
   VAR(AX0_s16_Crrnt_Coef_Vd_Fix)               = -s16_cl_coef_vd_fix;
   VAR(AX0_s16_Crrnt_Coef_Vd_Shr)               = s16_cl_coef_vd_shr;
   VAR(AX0_s16_Crrnt_Coef_Bemf_Comp_Fix)        = s16_bemf_fix;
   VAR(AX0_s16_Crrnt_Coef_Bemf_Comp_Shr)        = s16_bemf_shr;
   VAR(AX0_s16_Mres_Fix)                        = s16_mres_fix;
   VAR(AX0_s16_Mres_Shr)                        = s16_mres_shr;

   VAR(AX0_s16_Dead_Time_Comp)                  = s16_dead_time_comp;
   VAR(AX0_s16_Crrnt_DT_Comp_Min_Level)         = s16_dt_comp_level;
   VAR(AX0_s16_Crrnt_Adaptive_Vg_Slope)         = s16_adapt_vg_slope;

   VAR(AX0_u16_Pwm_Half_Period_Sat) = u16_pwm_half_period_sat;
   LVAR(AX0_s32_Pwm_Half_Period_Sqr) = s32_pwm_half_period_sqr;

   return (SUCCESS);
}


void CurrentLoopInit(int drive)
{
   // AXIS_OFF;

   InrushRelayOff();
   BGVAR(s16_DisableInputs) |= INRUSH_INHIBIT_MASK;

   VAR(AX0_s16_Phase_Adv_Angl) = 0;
   VAR(AX0_s16_Crrnt_Run_Code) = RES_EXC_INITIALIZED | RESET_SWR2D_MASK; // indicate working in 16khz & reset resolver

   BGVAR(s16_DisableInputs) |=HW_EN_MASK;

   LVAR(AX0_s32_Feedback_Ptr) = &INC_ENCODER_FEEDBACK;
   LVAR(AX0_s32_Feedback_Ptr_2) = &NO_FEEDBACK;
   VAR(AX0_s16_Active_Indication) = 0;

   VAR(AX0_s16_Controller_Ptr) = (int)((long)&CURRENT_CONTROLLER_DIS & 0xffff);
   LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Feedback_Comm_Pos);
   LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Serial_Crrnt_Cmnd);
   VAR(AX0_s16_Vel_Loop_Cmnd_Ptr) = (int)((long)&VAR(AX0_s16_Serial_Vel_Cmnd)  & 0xffff);
   ResetEncState(DRIVE_PARAM);
   VAR(AX0_s16_Sine_Enc_Bits) = 0;
   VAR(AX0_s16_Out_Of_Range_Bits) = 0;
}


void CalcCurrentAdaptiveGain(int drive)
{
   // AXIS_OFF;
   //convert micont and mipeak to internal units
   unsigned long u32_micont = (unsigned long)(((BGVAR(s32_Motor_I_Cont))* BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
            >> BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);
   unsigned long u32_mipeak = (unsigned long)(((BGVAR(s32_Motor_I_Peak)) * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
            >> BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);
   unsigned int u16_dipeak = (unsigned int)(((BGVAR(s32_Drive_I_Peak)) * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
            >> BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);
   REFERENCE_TO_DRIVE;
   VAR(AX0_s16_Mlgainc_Crrnt) = (int)u32_micont;
   if(BGVAR(s32_Motor_I_Cont) > BGVAR(s32_Drive_I_Peak))
      VAR(AX0_s16_Mlgainc_Crrnt) = (int)u16_dipeak;

   if (VAR(AX0_s16_Mlgainc_Crrnt) != 0)
      VAR(AX0_s16_Mlgainc_Slope) = (int)(16384.0*16384.0 * (-1000.0+(float)(BGVAR(s16_Mcont_Current_Gain)))/1000/(float)u32_micont);
   else
      VAR(AX0_s16_Mlgainc_Slope) = 0;

   if (u32_micont == u32_mipeak)
      VAR(AX0_s16_Mlgainp_Slope) = 0;
   else
      VAR(AX0_s16_Mlgainp_Slope) = (int)(16384.0*16384.0*((float)(BGVAR(s16_Mpeak_Current_Gain))-(float)(BGVAR(s16_Mcont_Current_Gain)))/(1000.0*((float)u32_mipeak-(float)u32_micont)));
}


int SalExtAdditiveIcmdCommand(long long lparam,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (lparam > VAR(AX0_s16_I_Sat_Hi)) lparam = VAR(AX0_s16_I_Sat_Hi);
   if (lparam < VAR(AX0_s16_I_Sat_Lo)) lparam = VAR(AX0_s16_I_Sat_Lo);
   VAR(AX0_s16_Extrn_Icmd_FF) = (int)lparam;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMcontCurrentGainCommand
// Description:
//       This function is called in response to the MLGAINC command, to set
//   or query the current loop gain at motor continuous current.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMcontCurrentGainCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s16_Mcont_Current_Gain) == (int)lparam) return (SAL_SUCCESS);

   BGVAR(s16_Mcont_Current_Gain) = (int)lparam;
   /* set no-comp fault */
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalMpeakCurrentGainCommand
// Description:
//       This function is called in response to the MLGAINP command, to set
//   or query the current loop gain at motor peak current.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMpeakCurrentGainCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s16_Mpeak_Current_Gain) == (int)lparam) return (SAL_SUCCESS);

   BGVAR(s16_Mpeak_Current_Gain) = (int)lparam;

   /* set no-comp fault */
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalVbusCommand
// Description:
//    This function is called in response to the VBUS message, to set
//    or query the bus voltage.
//
//
// Author: Yuval
// Algorithm:
// Revisions: S.F: On SE this will implicitly call CONFIG to avoid No-Comp
//**********************************************************
// As VBUS is common to both axes - setting its value will affect the 2 axes
int SalVbusCommand(long long lparam,int drive)
{
   static int s16_state = 0, s16_stored_ret_val;
   static int s16_prev_value = 0;
   int s16_ret_val = SAL_SUCCESS;

   if ((BGVAR(s16_Vbus) == (int)lparam) && (s16_state == 0)) return (SAL_SUCCESS);

   switch (s16_state)
   {
      case 0:
         s16_prev_value = BGVAR(s16_Vbus);

         BGVAR(s16_Vbus) = (int)lparam;
         UpdateVelocityLimits(drive); // so vmax will be updated when dump
         ConvertInternalPWMCmdToVolts1000(drive);

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
         else if (s16_ret_val != SAL_NOT_FINISHED) // If CONFIG failed - restore previous VLIM value
         {
            s16_stored_ret_val = s16_ret_val;
            BGVAR(s16_Vbus) = s16_prev_value;
            UpdateVelocityLimits(drive); // so vmax will be updated when dump
            ConvertInternalPWMCmdToVolts1000(drive);

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
// Function Name: SalMlminCommand
// Description:
//    This function is called in response to the MLMIN command. to set or
//   query the minimum line-to-line inductance.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalMlminCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u32_Mlmin) == (unsigned long)lparam) return (SAL_SUCCESS);

   BGVAR(u32_Mlmin) = (unsigned long)lparam;
   /* set no-comp fault */
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;

   return (SAL_SUCCESS);
}


int SalAdaptiveVbusCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s16_Adaptive_Vbus) == (int)lparam) return (SAL_SUCCESS);

   BGVAR(s16_Adaptive_Vbus) = (int)lparam;
   /* set no-comp fault */
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;

   return (SAL_SUCCESS);
}


int SalClUserGainCommand(long long lparam,int drive)
{
//   int s16_temp;
//   long s32_temp;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_CL_User_Gain) == (long)lparam) return (SAL_SUCCESS);

   /*if (Enabled(drive))
   {
      s32_temp = BGVAR(s32_CL_User_Gain);
      BGVAR(s32_CL_User_Gain) = (long)lparam;
      s16_temp = CrrntContDesign(drive);
      if ( s16_temp != SUCCESS)
      {
         BGVAR(s32_CL_User_Gain) = s32_temp;
         return s16_temp;
      }
   }
   else
   {
      BGVAR(s32_CL_User_Gain) = (long)lparam;
      // set no-comp fault
      BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
   }*/
   return (SAL_SUCCESS);
}


int SalMresCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u32_Motor_Res) == (long)lparam) return (SAL_SUCCESS);

   BGVAR(u32_Motor_Res) = (unsigned long)lparam;
   /* set no-comp fault */
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalForcedTorqueCommand
// Description:
//          This function is called in response to the SLTFORCED command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalForcedTorqueCommand(long long lparam,int drive)
{
   long long half_for_rounding = 0LL;
   int shr_val;
   REFERENCE_TO_DRIVE;
   // AXIS_OFF;

   if (lparam > (long long)BGVAR(s32_Ilim_Actual)) return (VALUE_TOO_HIGH);
   if (lparam < (long long)-BGVAR(s32_Ilim_Actual)) return (VALUE_TOO_LOW);

   BGVAR(s32_Forced_Torque_Cmd) = (long)lparam;

   shr_val = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   if (shr_val > 0)
     half_for_rounding = 1LL << ((long long)shr_val - 1);

   VAR(AX0_s16_Vforced_Torque_Cmd) =
      (int)(((long)lparam * (long long)BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix + half_for_rounding) >> (long long)shr_val);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalReadTorqueCommand
// Description:
//          This function is called in response to the T command without parameters.
//          to print the T command last parameters value.
//
// Author: Moshe
// Algorithm:
// Revisions:
//*********************************************************
int SalReadTorqueCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch(BGVAR(s16_Torque_Number_Of_Params))
   {
      case 1:
         PrintString(DecimalPoint32ToAscii(BGVAR(s32_Torque_Params[0])),0);// print t1 (torque 1 value)
         PrintString(" [A]",0);// print t1 (torque 1 value)
         PrintCrLf();
      break;

      case 2:
         PrintString(DecimalPoint32ToAscii(BGVAR(s32_Torque_Params[0])),0);// print t1 (torque 1 value)
         PrintString(" [A] ",0);// print t1 (torque 1 value)
         PrintString(UnsignedIntegerToAscii(BGVAR(s32_Torque_Params[1])),0);// print T1(time 1 value)
         PrintString(" [ms]",0);// print t1 (torque 1 value)
         PrintCrLf();
      break;

      case 4:
         PrintString(DecimalPoint32ToAscii(BGVAR(s32_Torque_Params[0])),0);// print t1 (torque 1 value)
         PrintString(" [A] ",0);// print t1 (torque 1 value)
         PrintString(UnsignedIntegerToAscii(BGVAR(s32_Torque_Params[1])),0);// print T1(time 1 value)
         PrintString(" [ms] ",0);// print t1 (torque 1 value)
         PrintString(DecimalPoint32ToAscii(BGVAR(s32_Torque_Params[2])),0);// print t2 (torque 2 value)
         PrintString(" [A] ",0);// print t1 (torque 1 value)
         PrintString(UnsignedIntegerToAscii(BGVAR(s32_Torque_Params[3])),0);// print T2(time 2 value)
         PrintString(" [ms]",0);// print t1 (torque 1 value)
         PrintCrLf();
      break;

      default:
      break;
   }

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalTorqueCommand
// Description:
//          This function is called in response to the T command.
//
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int SalTorqueCommand(int drive)
{
   int s16_retVal;

   if(s16_Number_Of_Parameters == 3)
   {
      return SYNTAX_ERROR;
   }

   if (FILEDBUS_MODE)   return (WRONG_COMMODE);

   s16_retVal = TestTorqueCommand(drive);
   if (s16_retVal != SAL_SUCCESS)
   {
      return s16_retVal;
   }

   if (s16_Number_Of_Parameters == 1)
   {
      // to bypass schneider limitations
      // if t command is zero, disable bypass. other command will eanble it.
      if (s64_Execution_Parameter[0] != 0)
         BGVAR(u16_Schneider_Bypass) |= 1;
      else
         BGVAR(u16_Schneider_Bypass) &= ~1;

      return TorqueCommand(s64_Execution_Parameter[0], drive);
   }
   else if((s16_Number_Of_Parameters == 2) || (s16_Number_Of_Parameters == 4))
   {
      // to bypass schneider limitations
      // if t command is zero, disable bypass. other command will eanble it.
      if (s64_Execution_Parameter[1] != 0)
         BGVAR(u16_Schneider_Bypass) |= 1;
      else
         BGVAR(u16_Schneider_Bypass) &= ~1;
         
      return (TorqueStepCommand(drive));
   }

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: TestTorqueCommand
// Description:
//          This function is called in response to the T command.
//          SalTorqueCommand() is splitted to SalTorqueCommandInternal() in
//          order to allow CAnopen call the T command and avoid WRONG_COMMODE error
//
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int TestTorqueCommand(int drive)
{
   // AXIS_OFF;

   unsigned int u16_temp_cw_ls;
   unsigned int u16_temp_ccw_ls;
   REFERENCE_TO_DRIVE;
   // For Schneider, or when setting Limits Mode to ignore Transient (Phantom) Bits
   if ( (u16_Product == SHNDR) || (u16_Product == SHNDR_HW) ||
        ((VAR(AX0_u16_SW_Pos_lim) & 0x02) == 0x02)            )
   {  // Do not use the Hardware Limit-Switch Phantom bits
      u16_temp_cw_ls  = VAR(AX0_u16_CW_LS)  & 0x03;
      u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS) & 0x03;
   }
   else
   {  // Involve the hardware limit-switch phantom bits
      u16_temp_cw_ls  = VAR(AX0_u16_CW_LS);
      u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS);
   }

   if (VAR(AX0_s16_Opmode) != 2) return (INVALID_OPMODE);

   if (BGVAR(u16_Est_Motor_Params) != 0) return (MOTOR_PARAMS_EST_IN_PROCESS);

   if (s16_Number_Of_Parameters == 1)
   {
      // test T value validity
// As opmode change be changed on the fly now, allow to set serial torque command in other opmodes as well
//      if (VAR(AX0_s16_Opmode) != 2) return INVALID_OPMODE;

//      if (BGVAR(u16_Hold_User) && s64_Execution_Parameter[0] != 0)
      if ((BGVAR(u16_Hold_Bits) & HOLD_USER_MASK) && s64_Execution_Parameter[0] != 0)
         return (HOLD_MODE);

      if ((u16_temp_cw_ls != 0) || (u16_temp_ccw_ls != 0))
      {
         if ( ((u16_temp_cw_ls != 0)  && (s64_Execution_Parameter[0] > 0)) ||
              ((u16_temp_ccw_ls != 0) && (s64_Execution_Parameter[0] < 0))   )
         { // alarm for lxm28 in canopen mode only (can alarams task)
            if ( ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))                 &&
                 ( ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_CANOPEN)     ||
                   ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_SERCOS)      ||
                   ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_ETHERNET_IP) ||
                   ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_ETHERCAT)      )  )
            {
               if (u16_temp_cw_ls & 0x01)
               {  // schneider alarm 283 (command towards positive software limit)
                  BGVAR(u64_Sys_Warnings) |= WRN_COMMAND_TOWARDS_CW_SW_LIMIT_MASK;
               }

               if (u16_temp_ccw_ls & 0x01)
               {  // schneider alarm 285 (command towards negative software limit)
                  BGVAR(u64_Sys_Warnings) |= WRN_COMMAND_TOWARDS_CCW_SW_LIMIT_MASK;
               }
            }

            return (COMMAND_TOWARDS_LIMIT);
         }
      }

      // clear warning
      BGVAR(u64_Sys_Warnings) &= ~WRN_COMMAND_TOWARDS_CW_SW_LIMIT_MASK;
      BGVAR(u64_Sys_Warnings) &= ~WRN_COMMAND_TOWARDS_CCW_SW_LIMIT_MASK;

      if (BGVAR(u16_AD_In_Process)) return (AD_IN_PROCCESS);

      if (s64_Execution_Parameter[0] > (long long)BGVAR(s32_Ilim_Actual)) return (VALUE_TOO_HIGH);
      if (s64_Execution_Parameter[0] < (long long)-BGVAR(s32_Ilim_Actual)) return (VALUE_TOO_LOW);

      BGVAR(s16_Torque_Number_Of_Params) = s16_Number_Of_Parameters;
      BGVAR(s32_Torque_Params[0]) = s64_Execution_Parameter[0];
      BGVAR(s16_Torque_Step_State) = CURRENT_STEP_IDLE;
   }

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: CurrentStepHandler
// Description:
//          This function is called from backGround to set time for current timed commands .
//
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void TorqueStepHandler(int drive)
{
   // AXIS_OFF;

   if ((!Enabled(drive)) || (VAR(AX0_s16_Opmode) != 2))
      BGVAR(s16_Torque_Step_State) = CURRENT_STEP_IDLE;

   switch (BGVAR(s16_Torque_Step_State))
   {
      case CURRENT_STEP_IDLE:
      break;

      case CURRENT_STEP_TORQUE1:
         TorqueCommand((long long)BGVAR(s32_Torque1), drive);
         BGVAR(s32_Torque_Step_Timer) = Cntr_1mS;
         BGVAR(s16_Torque_Step_State) = CURRENT_STEP_TIME1;
      break;

      case CURRENT_STEP_TIME1:
         if (PassedTimeMS(BGVAR(s32_Torque_Time1),BGVAR(s32_Torque_Step_Timer)))
         {
            if (BGVAR(s32_Torque_Time2) == 0L)
            {
               TorqueCommand(0LL, drive);
               BGVAR(s16_Torque_Step_State) = CURRENT_STEP_IDLE;
            }
            else BGVAR(s16_Torque_Step_State) = CURRENT_STEP_TORQUE2;
         }
      break;

      case CURRENT_STEP_TORQUE2:
         TorqueCommand((long long)BGVAR(s32_Torque2), drive);
         BGVAR(s32_Torque_Step_Timer) = Cntr_1mS;
         BGVAR(s16_Torque_Step_State) = CURRENT_STEP_TIME2;
      break;

      case CURRENT_STEP_TIME2:
         if (PassedTimeMS(BGVAR(s32_Torque_Time2),BGVAR(s32_Torque_Step_Timer)))
            BGVAR(s16_Torque_Step_State) = CURRENT_STEP_TORQUE1;
      break;
   }
}


//**********************************************************
// Function Name: CurrentStepCommand
// Description:
//          This function is the step function for use in the Current file .
//
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int TorqueStepCommand(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if ((s16_Number_Of_Parameters == 1) || (s16_Number_Of_Parameters > 4)) return SYNTAX_ERROR;

   if (s16_Number_Of_Parameters == 0) return (NOT_AVAILABLE);

   if (VAR(AX0_s16_Opmode) != 2) return (INVALID_OPMODE);

   if (s64_Execution_Parameter[1] <= 0) return (VALUE_TOO_LOW); //check t1

   //check T1 limits
   if (s64_Execution_Parameter[0] >  (long long)BGVAR(s32_Ilim_Actual)) return (VALUE_TOO_HIGH);
   if (s64_Execution_Parameter[0] < -(long long)BGVAR(s32_Ilim_Actual)) return (VALUE_TOO_LOW);

   BGVAR(s32_Torque_Time1) = (long)s64_Execution_Parameter[1];// t1
   BGVAR(s32_Torque1) = (long)s64_Execution_Parameter[0];      // T1

   BGVAR(s16_Torque_Number_Of_Params) = s16_Number_Of_Parameters;
   BGVAR(s32_Torque_Params[0]) = (long)s64_Execution_Parameter[0];
   BGVAR(s32_Torque_Params[1]) = (long)s64_Execution_Parameter[1];

   BGVAR(s32_Torque_Time2) = 0L;
   BGVAR(s32_Torque2)      = 0L;

   if (s16_Number_Of_Parameters == 4)
   {
      if (s64_Execution_Parameter[3] <= 0) return (VALUE_TOO_LOW);

      if (s64_Execution_Parameter[2] >  (long long)BGVAR(s32_Ilim_Actual)) return (VALUE_TOO_HIGH);
      if (s64_Execution_Parameter[2] < -(long long)BGVAR(s32_Ilim_Actual)) return (VALUE_TOO_LOW);

     BGVAR(s32_Torque_Params[2]) = (long)s64_Execution_Parameter[2];
     BGVAR(s32_Torque_Params[3]) = (long)s64_Execution_Parameter[3];

      BGVAR(s32_Torque_Time2) = (long)s64_Execution_Parameter[3];
      BGVAR(s32_Torque2)      = (long)s64_Execution_Parameter[2];
   }

   BGVAR(s16_Torque_Step_State) = CURRENT_STEP_TORQUE1;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: InternalTorqueCommand
// Description:
//          This function is called from the in the code to execute the TORQUE command.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int InternalTorqueCommand(long long lparam,int drive)
{//stop all Torque step execution if there any
 //reset all the Torque params to default and execute the requested torque command from in the code.
   // AXIS_OFF;

   unsigned int u16_temp_cw_ls, u16_temp_ccw_ls;

   // For Schneider, or when setting Limits Mode to ignore Transient (Phantom) Bits
   if ( (u16_Product == SHNDR) || (u16_Product == SHNDR_HW) ||
        ((VAR(AX0_u16_SW_Pos_lim) & 0x02) == 0x02)            )
   {
      u16_temp_cw_ls  = VAR(AX0_u16_CW_LS)  & 0x03;
      u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS) & 0x03;
   }
   else
   {
      u16_temp_cw_ls  = VAR(AX0_u16_CW_LS);
      u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS);
   }

   resetTorqueParamsToDefault(drive);

   if (VAR(AX0_s16_Opmode) != 2)  return INVALID_OPMODE;

//   if (BGVAR(u16_Hold_User) && (lparam != 0))  return HOLD_MODE;
   if ((BGVAR(u16_Hold_Bits) & HOLD_USER_MASK) && (lparam != 0))  return HOLD_MODE;

   if ((u16_temp_cw_ls != 0) || (u16_temp_ccw_ls != 0))
   {
      if ( ((u16_temp_cw_ls != 0)  && (lparam > 0)) ||
           ((u16_temp_ccw_ls != 0) && (lparam < 0))  )
      {   // alarm for lxm28 in canopen mode only (can alarams task)
         if ( ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))                 &&
              ( ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_CANOPEN)     ||
                ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_SERCOS)      ||
                ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_ETHERNET_IP) ||
                ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_ETHERCAT)      )  )
         {
            if (u16_temp_cw_ls & 0x01)
            {  // schnieder alarm 283 (command towards positive software limit)
               BGVAR(u64_Sys_Warnings) |= WRN_COMMAND_TOWARDS_CW_SW_LIMIT_MASK;
            }

            if (u16_temp_ccw_ls & 0x01)
            {  // schnieder alarm 285 (command towards negative software limit)
               BGVAR(u64_Sys_Warnings) |= WRN_COMMAND_TOWARDS_CCW_SW_LIMIT_MASK;
            }
         }

         return COMMAND_TOWARDS_LIMIT;
      }
   }

   // clear can alarms
   BGVAR(u64_Sys_Warnings) &= ~WRN_COMMAND_TOWARDS_CW_SW_LIMIT_MASK;
   BGVAR(u64_Sys_Warnings) &= ~WRN_COMMAND_TOWARDS_CCW_SW_LIMIT_MASK;

   if (BGVAR(u16_AD_In_Process)) return AD_IN_PROCCESS;

   if (lparam > (long long) BGVAR(s32_Ilim_Actual))  return VALUE_TOO_HIGH;
   if (lparam < (long long)-BGVAR(s32_Ilim_Actual))  return VALUE_TOO_LOW;

   return (TorqueCommand(lparam,drive));
}


//**********************************************************
// Function Name: resetTorqueParamsToDefault
// Description:
//          Reset all the TORQUE command params to default values..
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void resetTorqueParamsToDefault(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s16_Torque_Step_State) = CURRENT_STEP_IDLE;
   BGVAR(s32_Torque_Time1) = 0;
   BGVAR(s32_Torque1)      = 0;
   BGVAR(s32_Torque_Time2) = 0;
   BGVAR(s32_Torque2)      = 0;

   BGVAR(s16_Torque_Number_Of_Params) = 1;
   BGVAR(s32_Torque_Params[0]) = 0L;
   BGVAR(s32_Torque_Params[1]) = 0L;
   BGVAR(s32_Torque_Params[2]) = 0L;
   BGVAR(s32_Torque_Params[3]) = 0L;
}


//**********************************************************
// Function Name: TorqueCommand
// Description:
//          This function is called from the SalTorqueCommand.
//          This function execute the t command after the function SalTorqueCommand
//          handled the 1, 2 or 4 t command parameters.
//
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
int TorqueCommand(long long lparam,int drive)
{
   long long half_for_rounding = 0LL;
   int shr_val;

   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   BGVAR(s32_Serial_Torque_Cmd) = (long)lparam;

   shr_val = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   if (shr_val > 0)
     half_for_rounding = 1LL << ((long long)shr_val - 1);

   VAR(AX0_s16_Serial_Crrnt_Cmnd) =
   (int)(((long)lparam * (long long)BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix + half_for_rounding) >> (long long)shr_val);

   return (SAL_SUCCESS);
}


int SalClBemfCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_CL_Bemf_Gain) == (long)param) return (SAL_SUCCESS);

   if ( (BGVAR(s16_Bundle_Design_Lock) >= 0) && (BGVAR(s16_Bundle_Design_Lock) <= CDHD_PRO2_MOTOR_DATA_ENTRIES) )
      return NOT_PROG_BUNDLE_DATA;

   BGVAR(s32_CL_Bemf_Gain) = (long)param;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


int SalClDAxisCompCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_CL_DQ_Axis_Comp_Gain) == (long)param) return (SAL_SUCCESS);

   BGVAR(s32_CL_DQ_Axis_Comp_Gain) = (long)param;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


int SalClKivCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_CL_Kiv_User) == (long)param) return (SAL_SUCCESS);

   BGVAR(s32_CL_Kiv_User) = (long)param;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


int SalClKiCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_CL_Ki_User) == (long)param) return (SAL_SUCCESS);

   if ( (BGVAR(s16_Bundle_Design_Lock) >= 0) && (BGVAR(s16_Bundle_Design_Lock) <= CDHD_PRO2_MOTOR_DATA_ENTRIES) )
      return NOT_PROG_BUNDLE_DATA;

   BGVAR(s32_CL_Ki_User) = (long)param;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


int SalClKpCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_CL_Kp_User) == (long)param) return (SAL_SUCCESS);

   if ( (BGVAR(s16_Bundle_Design_Lock) >= 0) && (BGVAR(s16_Bundle_Design_Lock) <= CDHD_PRO2_MOTOR_DATA_ENTRIES) )
      return NOT_PROG_BUNDLE_DATA;

   BGVAR(s32_CL_Kp_User) = (long)param;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


int SalClSatFactorCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_PWM_Sat_Factor) == (long)param) return (SAL_SUCCESS);

   BGVAR(s32_PWM_Sat_Factor) = (long)param;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


int SalClKffCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_CL_Kff_User) == (long)param) return (SAL_SUCCESS);

   if ( (BGVAR(s16_Bundle_Design_Lock) >= 0) && (BGVAR(s16_Bundle_Design_Lock) <= CDHD_PRO2_MOTOR_DATA_ENTRIES) )
      return NOT_PROG_BUNDLE_DATA;

   BGVAR(s32_CL_Kff_User) = (long)param;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: KCVG
// Description:
//          This function is called in response to the KCVG command.
//
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalClVgCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s32_CL_Kvg_User) == (long)param) return (SAL_SUCCESS);

   BGVAR(s32_CL_Kvg_User) = (long)param;

   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;   // set no-comp fault

   return (SAL_SUCCESS);
}


void CalcDynamicBrakingGain(int drive)
{
   // AXIS_OFF;
   int s16_temp, s16_max_current = 0;
   long long s64_temp;
   float f_pwmfrq_factor = 1.0;
   REFERENCE_TO_DRIVE;
   //  Max value = 100% PWM = 2 * Pwm_Half_Period.
   //  Gain = max_value /  Stop_Current  = Pwm_Half_Period*2 / Stop_Current.

   if ( (VAR(AX0_s16_Stop_Crrnt_Cmnd) == 0) || (VAR(AX0_s16_Pwm_Half_Period) == 0) )
   {
      VAR(AX0_u16_Dynamic_Brake_Gain) = 0;
      VAR(AX0_u16_Dynamic_Brake_Shr) = 0;
      do {
         s16_temp = Cntr_3125;
         LLVAR(AX0_u32_DB_Saturation_Lo) = 0LL;
      } while (s16_temp != Cntr_3125);
   }
   else
   {
      FloatToFix16Shift16((int *)&VAR(AX0_u16_Dynamic_Brake_Gain), &VAR(AX0_u16_Dynamic_Brake_Shr), 2.0*((float)VAR(AX0_s16_Pwm_Half_Period)/(float)VAR(AX0_s16_Stop_Crrnt_Cmnd)));

      // This is the value used to limit the PWM value according to the velocity to avoid
      // exceeding ISTOP by more than 1Amp
      // dt = di*L/V = 1Amp * ML / (Vel * MBemf)
      // as Max_PWM = dt[sec]*16000(or 8000)*2*AX0_s16_Pwm_Half_Period  => L/MBEMF *16000(or 8000)*2*AX0_s16_Pwm_Half_Period
      // => ML*32000(or 16000)*AX0_s16_Pwm_Half_Period/(60.42 * MKT) * 2^32 / 8000
      f_pwmfrq_factor = (float)(16000L / BGVAR(u32_Pwm_Freq));
      s64_temp = (long long)((float)BGVAR(u32_Mlmin) * (float)VAR(AX0_s16_Pwm_Half_Period) * 284340.7674 / (float)BGVAR(u32_Motor_Kt) / f_pwmfrq_factor); // / The last term is because the loop is running at 32KHx while the PWMFRQ is 8/16Khz
      do {
         s16_temp = Cntr_3125;
         LLVAR(AX0_u32_DB_Saturation_Lo) = s64_temp;
      } while (s16_temp != Cntr_3125);

      // If actual current exceeds ISTOP be more than 10% of min(MIPEAK,DIPEAK) then PWM will shut-down
      s16_max_current = min(BGVAR(s32_Drive_I_Peak),BGVAR(s32_Motor_I_Peak));
      s16_max_current /= 10;
      VAR(AX0_s16_DB_Overshoot_Limit) = -(s16_max_current * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
         >> BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
   }
}


int SalIstopCommand(long long param,int drive)
{
   // AXIS_OFF;

   if (param > (long long)BGVAR(s32_Drive_I_Peak)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u32_Stop_Current) = (long)param;

   VAR(AX0_s16_Stop_Crrnt_Cmnd) = (int)((float)BGVAR(u32_Stop_Current)*26214.0/(float)BGVAR(s32_Drive_I_Peak));

   VAR(AX0_u16_Dynamic_Brake_Integrator) = 0;

   CalcDynamicBrakingGain(DRIVE_PARAM);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: DTCOMPLEVEL
// Description:
//          This function is called in response to the DTCOMPLEVEL command.
//
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalKCDCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u32_Kcd_Gain) == (long)lparam) return (SAL_SUCCESS);

   if ( (BGVAR(s16_Bundle_Design_Lock) >= 0) && (BGVAR(s16_Bundle_Design_Lock) <= CDHD_PRO2_MOTOR_DATA_ENTRIES) )
      return NOT_PROG_BUNDLE_DATA;

   BGVAR(u32_Kcd_Gain) = (long)lparam;

   /* set no-comp fault */
   BGVAR(s64_SysNotOk) |= NO_COMP_FLT_MASK;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: KCMODE
// Description:
//          This function is called in response to the KCMODE command.
//          0 - New Current loop
//          1 - Old Current loop
//          2 - New Current loop with adaptive KP
//
//
// Author: Sergey
// Algorithm:
// Revisions:
//**********************************************************
int SalKCMODECommand(long long lparam,int drive)
{
   // AXIS_OFF;
   static int s16_state = 0, s16_stored_ret_val;
   static int s16_prev_value = 0;
   int s16_ret_val = SAL_SUCCESS;

   switch (s16_state)
   {
      case 0:
         s16_prev_value = BGVAR(AX0_S16_Kc_Mode);
         if (VAR(AX0_S16_Kc_Mode) == (int)lparam) return (SAL_SUCCESS);
         VAR(AX0_S16_Kc_Mode) = (int)lparam;
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
         else if (s16_ret_val != SAL_NOT_FINISHED) // If CONFIG failed - restore previous KCMODE value
         {
            s16_stored_ret_val = s16_ret_val;
            VAR(AX0_S16_Kc_Mode) = s16_prev_value;
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


int SalMotorPhaseFaultCommand(long long lparam,int drive)
{
   lparam++; // Just to avoid remark
   drive++; // Just to avoid remark
/*   // AXIS_OFF;
   long s32_temp;
   BGVAR(s16_Phase_Disconnect_E_Pos_Err_Gain) = (int)lparam;
   s32_temp = (int)(10922.66666 * (float)BGVAR(s16_Phase_Disconnect_E_Pos_Err_Gain)/1000.0);
   if ((s32_temp > 0x07FFF) || (s32_temp == 0)) s32_temp = 0x07FFF;
   VAR(AX0_s16_Phase_Disconnect_E_Pos_Err_Thresh) = (int)s32_temp;
*/   return (SAL_SUCCESS);
}

/*
unsigned int u16_Bundle_Debug1;
unsigned long u32_Bundle_Debug2; */
//**********************************************************
// Function Name: SearchBundleCurrentTuning
// Description:
//   This function is called to search for an entry in the CDHD Pro2 Motor Data Table;
//   Current-Loop Parameters will be set accordingly, and User Access allowed or denied as well.
//
// Author: A. H.
// Algorithm:
// Return Value:
//   2000: Data not found in Table, User Access to Current-Loop Parameters allowed.
//   1 through CDHD_PRO2_MOTOR_DATA_ENTRIES:  Data found and valid, Parameters set accordingly, and
//         User Access to Current-Loop Parameters disabled.
// Revisions:
//**********************************************************
int SearchBundleCurrentTuning(int drive)
{
   int i;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for (i = 0; i < CDHD_PRO2_MOTOR_DATA_ENTRIES; i++)
   {
      if ( (s_Pro2_CDHD_Bundle_Data[i].motor_file_name_cdhd == BGVAR(u32_MotorFileName)) &&
           (s_Pro2_CDHD_Bundle_Data[i].drive_vbus_scale     == BGVAR(u16_Vbus_Scale))    &&
           (s_Pro2_CDHD_Bundle_Data[i].drive_i_cont         == BGVAR(s32_Drive_I_Cont))    )
      {
         return i;
      }
   }

   return 2000;
}



