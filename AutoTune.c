#include "DSP2834x_Device.h"
#include "DSP2834x_EQep.h"
#include <string.h>
#include "Math.h"

#include "Prototypes.pro"


#include "MultiAxis.def"
#include "design.def"
#include "Err_Hndl.def"
#include "AutoTune.def"
#include "Record.def"
#include "FltCntrl.def"
#include "PhaseAdv.def"
#include "Current.def"
#include "Exe_IO.def"
#include "Modbus_Comm.def"
#include "Flash.def"
#include "PtpGenerator.def"
#include "Init.def"
#include "ExFbVar.def"
#include "i2c.def"
#include "record.def"

#include "init.var"
#include "Extrn_Asm.var"
#include "Drive.var"
#include "AutoTune.var"
#include "Motor.var"
#include "Velocity.var"
#include "Ser_Comm.var"
#include "Units.var"
#include "Position.var"
#include "Record.var"
#include "FltCntrl.var"
#include "PhaseAdv.var"
#include "PtpGenerator.var"
#include "MotorSetup.var"
#include "Exe_IO.var"
#include "Modbus_Comm.var"
#include "FlashHandle.var"
#include "ExFbVar.var"
#include "Foldback.var"
#include "ModCntrl.var"
#include "User_Var.var"

//#define KI_KP_RATIO 0.43
//#define KP_KIV_RATIO 1.280
//#define KP_KIV_RATIO 0.8

extern int p402_modes_of_operation_display;


void CalculateNLParameters(int drive,int s16_mode)
{
   #define FLEXIBLE_K 1.0
   #define R13BIT 8192
   #define CURRENT_LOOP_SERVO_PERIOD 31.25e-6
   #define MIN_VG 1.6
   #define FINAL_FACTOR 1
   #define MAX_MOVESMOOTHLPFHZ 5000

   float f_security_factor,f_load,f_denc,f_j,f_rated_torque,f_x,f_loadx,f_df11,f_max_Vg,f_temp,
         f_MaxKd_cycle,f_MaxKd_enc,f_recommendedKd,f_Kislimit,f_recommendedKi,f_Kplimit,f_recommendedKp,f_recommendedKiv,f_recommendednlaffHZ,
         f_recommendedMOVESMOOTHLPFHZ,f_recommendedMaxGain;
   // AXIS_OFF;

   //unsigned long u32_temp;
   //int s16_tsp_factor = (int)((float)VAR(AX0_u16_Pos_Loop_Tsp)*0.01);
   f_load = 1.0 + BGVAR(u32_LMJR_User)/1000.0;
   f_security_factor = 1.0/(1 + log(f_load));

   if (BGVAR(u16_FdbkType) == INC_ENC_FDBK) f_denc = 0.25*6.2832/(float)BGVAR(u32_User_Motor_Enc_Res);
   else f_denc = 6.2832/(float)LVAR(AX0_s32_Counts_Per_Rev);

   if ( (BGVAR(u16_FdbkType) == INC_ENC_FDBK)        &&
        (BGVAR(u16_Motor_Enc_Interpolation) >=4)     &&
        (BGVAR(u16_Motor_Enc_Interpolation_Mode) > 0)  )
   { // in encoder interpolation - dont trust high 4 interpolated bits
      f_denc = f_denc/4.0;
   }

   f_j = BGVAR(s32_Motor_J)*f_load*1e-6;
   f_rated_torque = (float)BGVAR(u32_Motor_Kt)*(float)BGVAR(s32_Motor_I_Cont)/1414213.5624;
   f_x =  0.060667*BGVAR(s32_Motor_J)/f_rated_torque;
   f_loadx = f_x*f_load;
   f_df11 = sqrt(8.0*f_rated_torque/(f_j * f_denc))/FLEXIBLE_K;


   //LIMIT VALUES FOR Kd DUE TO SYSTEM LIMITATIONS
   f_MaxKd_cycle = f_df11 /6.2832;
   f_MaxKd_enc = f_rated_torque /(6.2832*f_j*f_denc*f_df11);
   if (f_MaxKd_enc > 300) f_MaxKd_enc = 300;

   //FIRST TRY TO SET Kd TO MaxKd_enc, BECAUSE THIS IS USUALLY THE LOWER LIMIT
   f_recommendedKd = f_security_factor * f_MaxKd_enc / FLEXIBLE_K;

   if (f_recommendedKd > f_MaxKd_cycle) f_recommendedKd = f_MaxKd_cycle;
   f_Kplimit = 0.333333*f_recommendedKd * sqrt(f_load);
   f_recommendedKp = 1.05 * f_recommendedKd * sqrt(f_load/f_loadx);
   if (f_recommendedKp > f_Kplimit) f_recommendedKp = f_Kplimit;

   if (BGVAR(u16_HdTune_Adv_Mode) != 0)
   {
      f_recommendedKiv = f_recommendedKp/BGVAR(f_Kp_Kiv_Ratio);
      f_recommendedKi = BGVAR(f_Ki_Kp_Ratio)*f_recommendedKp;
   }
   else
   {
      //INTEGRAL FEEDBACK PARAMETER
      f_Kislimit = 0.33333333*f_recommendedKd * pow(f_load,0.6666666);
      if (f_Kislimit > 0.25*f_recommendedKp) f_Kislimit = 0.25*f_recommendedKp;

      f_recommendedKi = 0.4 * f_recommendedKd * pow(f_load/f_loadx,0.6666666);
      if (f_recommendedKi > f_Kislimit) f_recommendedKi = f_Kislimit;

      f_recommendedKiv = f_recommendedKp * 0.75;
   }

   //MAXIMUM VARIABLE GAIN DEPENDS ON RESOLUTION FORMULA IS EMPIRIC FOR CONVENTIONAL MOTORS
   if (BGVAR(u16_FdbkType) == INC_ENC_FDBK) f_temp = (float)BGVAR(u32_User_Motor_Enc_Res)*4;
   else f_temp = (float)LVAR(AX0_s32_Counts_Per_Rev);

   f_max_Vg = 4.0*(2 * R13BIT + f_temp) / (R13BIT + 3 * f_temp);
   if (f_max_Vg > 3.0) f_max_Vg = 3.0;


   f_recommendedMaxGain = 1.6 * 800 /(sqrt((float)BGVAR(u32_LMJR_User)/1000.0) * f_recommendedKd);

   if (f_recommendedMaxGain > f_max_Vg) f_recommendedMaxGain = f_max_Vg;
   if (f_recommendedMaxGain < MIN_VG) f_recommendedMaxGain = MIN_VG;

   if (BGVAR(u16_HdTune_Adv_Mode) != 0)   f_recommendedKi = f_recommendedKi * f_recommendedMaxGain/1.6;
   f_recommendedKiv = f_recommendedKiv * f_recommendedMaxGain/1.6;

   f_recommendedKd  *= FINAL_FACTOR;
   f_recommendedKp  *= FINAL_FACTOR;
   f_recommendedKi  *= FINAL_FACTOR;
   f_recommendedKiv *= FINAL_FACTOR;

   f_recommendedKp *= 0.5;
   if (f_recommendedKp < 2.0) f_recommendedKp = 2.0;

   f_recommendedKi *= 0.5;
   if (f_recommendedKi < 2.0) f_recommendedKi = 2.0;

   f_recommendedKiv *= 0.5;
   if (f_recommendedKiv < 2.0) f_recommendedKiv = 2.0;

   f_recommendedMOVESMOOTHLPFHZ = f_recommendedKd;// * 3;
   if (f_recommendedMOVESMOOTHLPFHZ > MAX_MOVESMOOTHLPFHZ) f_recommendedMOVESMOOTHLPFHZ = MAX_MOVESMOOTHLPFHZ;
   if (f_recommendedMOVESMOOTHLPFHZ < 25) f_recommendedMOVESMOOTHLPFHZ = 25;

   f_recommendednlaffHZ = f_recommendedKd * 3.0;
   if (f_recommendednlaffHZ < 10) f_recommendednlaffHZ = 10;
   if (f_recommendednlaffHZ > 600) f_recommendednlaffHZ = 600;

   if (s16_mode == 0)
   {
      BGVAR(u32_Nl_Kpiv_User) = (unsigned long)(f_recommendedKiv * 1000);
      BGVAR(u32_Nl_Kpi_User) = (unsigned long)(f_recommendedKi * 1000);
      BGVAR(u32_Nl_Kpp_User) = (unsigned long)(f_recommendedKp * 1000);
      BGVAR(u32_Nl_Kpd_User) = (unsigned long)(f_recommendedKd * 1000);

      if (BGVAR(s16_Pos_Tune_Stiffness) == 0)
      {
         SalMoveSmoothLpfCommand((long long)f_recommendedMOVESMOOTHLPFHZ,drive);
         //gearfiltt1=1000/(2.movesmoothlpfhz) (ms); gearfiltt2 * 1.5*gearfiltt1
         /*u32_temp = (unsigned long)(GEARFILT_FACTOR/f_recommendedMOVESMOOTHLPFHZ);
         u32_temp = u32_temp / s16_tsp_factor;
         u32_temp = u32_temp*s16_tsp_factor;
         if(u32_temp < 750L) u32_temp = 750L; // Limit GEARFILTT1 to 0.75 mS
         BGVAR(u32_Gear_Filt_User_T1) = u32_temp; // do not use the sat command so as not to desingmsq more times - only takes longer
         u32_temp = (3*u32_temp) >> 1;
         u32_temp = u32_temp / s16_tsp_factor;
         u32_temp = u32_temp*s16_tsp_factor;
         SalGearFiltT2Command((long long)u32_temp,drive);*/
         BGVAR(u32_Gear_Filt_User_T1) = 1500; // yuval 1/5/16 - decided since movesmoothavg is used in ext trajectory use
         SalGearFiltT2Command(3000LL,drive);  // gear filter to minimize the quatization affecrs only. this means that gear filt is a function of
                                              // command resolution . we have than selected constant values as a mid way

      }
   }

   BGVAR(u32_Nl_Kpgmax) = (unsigned long)(f_recommendedMaxGain * 1000);
   BGVAR(s16_Kbff_Spring_LPF_User) = (int)f_recommendednlaffHZ;

   PositionConfig(drive,0);
}


#pragma CODE_SECTION(LowFreqOScilationDetector, "ramfunc_5");
int LowFreqOScilationDetector(int drive,int reset)
{
//#define PE_FILT_COEF 0.2 //100Hz*0.002(Ts_algorithm)
//#define PE_FILT_COEF 0.8 //400Hz*0.002(Ts_algorithm)
#define PE_FILT_COEF 0.3 //150Hz*0.002(Ts_algorithm)
#define OSC_TRESHOLD 1000
#define VIB_ENERGY_LIM 70
#define DELTA_I_LIM 100 /* units = 0.1% of miconts */

#define VIB_DECAY_FACTOR 0.95

   static long long s64_pe_filt,s64_prev_cross_over;
   static long s32_ptpvcmd_prev;
   static int s16_icmd_max,s16_icmd_min,s16_icmd_max_prev,s16_icmd_min_prev;
   static float f_delta_i_lim;
   float f_vib_evergy,f_temp,f_highest_param;
   long long s64_pe,s64_pe_filt_prev,s64_temp;
   long s32_ptpvcmd;
   int s16_temp,s16_icmd,s16_ret_val = 0;
   // AXIS_OFF;

   do {
         s16_temp = Cntr_3125;
         s64_pe  = LLVAR(AX0_u32_Pos_Err_Lo);
         s16_icmd = VAR(AX0_s16_Crrnt_Q_Ref_0);
         s32_ptpvcmd = LVAR(AX0_s32_Pos_Vcmd);
   } while (s16_temp != Cntr_3125);

   if (reset == 1)
   {
      s64_pe_filt = s64_pe;
      s64_prev_cross_over = -1;
      s16_icmd_max_prev = s16_icmd_max = -32768;
      s16_icmd_min_prev = s16_icmd_min = 32767;
      BGVAR(s32_Hdtune_Low_Frq_Osc) = 0;
      BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) = 0;
      s32_ptpvcmd_prev = s32_ptpvcmd;
      f_delta_i_lim = (float)DELTA_I_LIM * (float)BGVAR(s32_Motor_I_Cont)* 1e-6;
      return 0;
   }

   //lpf on PE
   s64_pe_filt_prev = s64_pe_filt;
   s64_pe_filt = s64_pe_filt + (long long)(PE_FILT_COEF*(float)(s64_pe - s64_pe_filt));

   if (s16_icmd > s16_icmd_max) s16_icmd_max = s16_icmd;
   if (s16_icmd < s16_icmd_min) s16_icmd_min = s16_icmd;

   // find zero cross - the differentcec between pe and filtered pe
   s64_temp = s64_pe - s64_pe_filt;
   if (((s64_temp < 0) && (s64_prev_cross_over > 0)) ||
       ((s64_temp > 0) && (s64_prev_cross_over < 0))   )
   { // cross over detected
      if (s16_icmd_max > s16_icmd_max_prev)
         f_vib_evergy = (float)fabs((float)s16_icmd_max-(float)s16_icmd_min_prev);
      else
         f_vib_evergy = (float)fabs((float)s16_icmd_max_prev-(float)s16_icmd_min);

      // scale value to amperes
      f_vib_evergy = f_vib_evergy*(float)BGVAR(s32_Drive_I_Peak)/26214000.0;
      if (f_vib_evergy < f_delta_i_lim) f_vib_evergy = 0;


      // identify cross speed. remove command variation from cross speed
     if (VAR(AX0_u16_Pos_Loop_Tsp) != TSP_250_USEC)
         f_temp = 1.17e-5;
     else
        f_temp = 5.8517e-6;
      f_temp = (float)labs(s32_ptpvcmd-s32_ptpvcmd_prev)*f_temp - 1.17e-8*(float)BGVAR(u32_Mspeed); // PTPVCMD -> rad/sec -> 4000*2*pi/2^32 = 5.8517e-6
      if (f_temp < 0) f_temp = 0;                                                                    // mspeed -> rad/sec -> 8000*2*pi/2^32 = 1.17e-5
                                                                                                     // deltaPE/Ts ->rad/sec->2*pi/2^32/0.002 = 7.3146e-7
      f_temp = (float)llabs(s64_pe_filt - s64_pe_filt_prev)*7.3146e-7 - f_temp;
      if (f_temp < 0) f_temp = 0;

      f_highest_param = (float)PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0);
      if (PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0) > f_highest_param) f_highest_param = (float)PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0);
      if (PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0) > f_highest_param) f_highest_param = (float)PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0);
      if (PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0) > f_highest_param) f_highest_param = (float)PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0);
      f_highest_param = f_highest_param * PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0) * 1e-6;
      f_highest_param = f_highest_param*f_highest_param;

      f_vib_evergy = f_vib_evergy*f_temp*f_highest_param;

      // age variables
      s16_icmd_max_prev = s16_icmd_max;
      s16_icmd_min_prev = s16_icmd_min;
      s16_icmd_max = -32768;
      s16_icmd_min = 32767;
      s32_ptpvcmd_prev = s32_ptpvcmd;

      // calc indicator - do not calc if still during reset "affect"
      if ((s16_icmd_max_prev == -32768) || (s16_icmd_min_prev == 32767))
      {
         BGVAR(s32_Hdtune_Low_Frq_Osc) = 0;
         BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) = 0;
      }
      else
      {
         // limit vib energy
         if (f_vib_evergy > VIB_ENERGY_LIM) f_vib_evergy = VIB_ENERGY_LIM;

         BGVAR(s32_Hdtune_Low_Frq_Osc) = (long)(VIB_DECAY_FACTOR*(float)BGVAR(s32_Hdtune_Low_Frq_Osc) + f_vib_evergy);
         if (BGVAR(s32_Hdtune_Low_Frq_Osc) > OSC_TRESHOLD) BGVAR(s32_Hdtune_Low_Frq_Osc) = OSC_TRESHOLD;
         if (BGVAR(s32_Hdtune_Low_Frq_Osc) < 0) BGVAR(s32_Hdtune_Low_Frq_Osc) = 0x7FFFFFFF;

         BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) = (long)(VIB_DECAY_FACTOR*(float)BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) + f_vib_evergy);
         if (BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) > OSC_TRESHOLD) BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) = OSC_TRESHOLD;
         if (BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) < 0) BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) = 0x7FFFFFFF;

         if (BGVAR(s32_Hdtune_Low_Frq_Osc) >= OSC_TRESHOLD) s16_ret_val = 1;
      }
   }
   s64_prev_cross_over = s64_temp;
   return s16_ret_val;
}

#pragma CODE_SECTION(ResetCycleIdentification, "ramfunc_5");
void ResetCycleIdentification(int drive)
{
   int s16_temp;
   // AXIS_OFF;

   BGVAR(u16_Cycle_Counter) = 0;
   BGVAR(u16_Cycle_Counter_Mask) = 0x3;
   VAR(AX0_s16_StandStill_Counter) = 0;
   BGVAR(s16_Cycle_StandStill_Found) = 0;

   VAR(AX0_s16_Cycle_Bits) &= ~(CYCLE_POSSIBLE_CYCLE_TESTED | CYCLE_POSSIBLE_CYCLE_FOUND | CYCLE_NEW_CYCLE_DATA_READY | CYCLE_END_AND_STANDING | CYCLE_PARAM_UPDATE | CYCLE_LENGTH_ERROR);
   BGVAR(s16_Cycle_And_Standing_Ticks) = 0;
   BGVAR(u16_Cycle_Max_STicks_Counter) = 0;
   BGVAR(u16_Cycle_Max_STicks_Best) = 0;
   // Dont reset LMJR in ext trajectory
   if ((BGVAR(s16_Pos_Tune_Traj_Mode_User) != PTT_IDLE)) BGVAR(s32_Cycle_LMJR) = 0;
   BGVAR(s32_Cycle_Fric_Coeff) = -1;
   BGVAR(s32_Pos_Vcmd_Prev) = 0;
   BGVAR(s16_Cycle_LMJR_Counter) = 0;
   CalculateLMJR(0,0,0,0); // reset static vars
   BGVAR(s32_Pos_Tune_Bits) &= ~POS_TUNE_LONGEST_CYCLE_WARN;
   LowFreqOScilationDetector(drive,1); // reset low frq oscilator

   // update prev value and init buffer
    do {
         s16_temp = Cntr_3125;
         VAR(AX0_s16_RT_To_1mSec_Capture_Pcmd_Raw_Tail) = VAR(AX0_s16_RT_To_1mSec_Capture_Pcmd_Raw_Head) >> 2;
         BGVAR(s64_Prev_Cycle_Data) = LLVAR(AX0_s64_RT_To_1mSec_Capture_Pcmd_Raw_Buff[VAR(AX0_s16_RT_To_1mSec_Capture_Pcmd_Raw_Tail)]);
    } while (s16_temp != Cntr_3125);
}


// #pragma CODE_SECTION(CycleArrayOverFlow, "ramfunc");
#pragma CODE_SECTION(CycleArrayOverFlow, "ramfunc_3");
void CycleArrayOverFlow(int drive)
{
   // AXIS_OFF;

   BGVAR(u16_Cycle_Max_STicks_Best) = 0;
   BGVAR(u16_Cycle_Counter) = 0;
   VAR(AX0_s16_StandStill_Counter) = 0;
   BGVAR(s16_Cycle_StandStill_Found) = 0;
   BGVAR(u32_Cycle_Standing_Counter) = 0;
   BGVAR(u32_Cycle_OverAll_Counter) = 0;
   BGVAR(u32_Cycle_OverAll_Counter_GUI) = 0;
   LVAR(AX0_u32_At_Control_Effort_Max) = 0;
   VAR(AX0_s16_Cycle_Bits) &= ~(CYCLE_POSSIBLE_CYCLE_TESTED|CYCLE_POSSIBLE_CYCLE_FOUND|CYCLE_NEW_CYCLE_DATA_READY|CYCLE_END_AND_STANDING|CYCLE_PARAM_UPDATE);
   VAR(AX0_s16_Cycle_Bits) |= CYCLE_LENGTH_ERROR;
   BGVAR(s32_Pos_Tune_Bits) &= ~POS_TUNE_LONGEST_CYCLE_WARN;

   BGVAR(s16_Cycle_And_Standing_Ticks) = 0;
   BGVAR(u16_Cycle_Max_STicks_Counter) = 0;
   BGVAR(u16_StandStill_Ticks) = BGVAR(u16_HDTUNE_Dwell_Time_mS);
   BGVAR(s32_Cycle_Fric_Coeff) = -1;
   // Dont reset LMJR in ext trajectory
   if ((BGVAR(s16_Pos_Tune_Traj_Mode_User) != PTT_IDLE)) BGVAR(s32_Cycle_LMJR) = 0;
   BGVAR(s16_Cycle_LMJR_Counter) = 0;
   CalculateLMJR(0,0,0,0); // reset static vars
   LowFreqOScilationDetector(drive,1); // reset low frq oscilator
}

#pragma CODE_SECTION(GetExtrema, "ramfunc");
long long GetExtrema(int s16_index,int s16_min_max)
{
   // calc address of s16_index in s16_Debug_Ram
   // every array row had 3 64 bit variable [extrema_a(64) extrema_b(64) minmaxtresh_a(32) minmaxtresh_b(32)
   //
   // s16_min_max = 1 => write to minmax array
   //
   // ret value < 0  - buffer overflow
   int s16_array_col_index,
       s16_array_row_index = u16_Ram_Rec_Length_Avaliable + (s16_index >> 1);
   long long s64_value;


   if (s16_array_row_index > REC_RAM_LENGTH) return 0;

   if ((s16_index & 0x01) == 0) s16_array_col_index = 3;
   else  s16_array_col_index = 7;

   if (s16_min_max == 1)
   {
      if (s16_array_col_index == 3) s16_array_col_index = 9;
      else s16_array_col_index = 11;
   }

   s64_value  = (long long)s16_Debug_Ram[s16_array_col_index--][s16_array_row_index];
   s64_value <<=16;
   s64_value  |= (long long)s16_Debug_Ram[s16_array_col_index--][s16_array_row_index] & 0x0FFFFLL;;

   if (s16_min_max == 1) return s64_value;

   s64_value <<=16;
   s64_value  |= (long long)s16_Debug_Ram[s16_array_col_index--][s16_array_row_index] & 0x0FFFFLL;;
   s64_value <<=16;
   s64_value  |= (long long)s16_Debug_Ram[s16_array_col_index][s16_array_row_index] & 0x0FFFFLL;;

   return s64_value;
}

#pragma CODE_SECTION(StoreExtrema, "ramfunc");
int StoreExtrema(long long s64_value,int s16_index,int s16_min_max)
{
   // calc address of s16_index in s16_Debug_Ram
   // every array row had 3 64 bit variable [extrema_a(64) extrema_b(64) minmaxtresh_a(32) minmaxtresh_b(32)
   //
   // s16_min_max = 1 => write to minmax array
   //
   // ret value < 0  - buffer overflow
   int s16_array_col_index,
       s16_array_row_index = u16_Ram_Rec_Length_Avaliable + (s16_index >> 1);

   if (s16_array_row_index > REC_RAM_LENGTH) return -1;

   if ((s16_index & 0x01) == 0) s16_array_col_index = 0;
   else  s16_array_col_index = 4;

   if (s16_min_max == 1)
   {
      if (s16_array_col_index == 0) s16_array_col_index = 8;
      else s16_array_col_index = 10;
   }

   s16_Debug_Ram[s16_array_col_index++][s16_array_row_index] = (int)(s64_value & 0x0FFFFLL);
   s64_value >>=16;
   s16_Debug_Ram[s16_array_col_index++][s16_array_row_index] = (int)(s64_value & 0x0FFFFLL);

   if (s16_min_max == 1) return 1;

   s64_value >>=16;
   s16_Debug_Ram[s16_array_col_index++][s16_array_row_index] = (int)(s64_value & 0x0FFFFLL);
   s64_value >>=16;
   s16_Debug_Ram[s16_array_col_index][s16_array_row_index] = (int)(s64_value & 0x0FFFFLL);

   return 1;
}

#pragma CODE_SECTION(CheckCycleValid, "ramfunc");
int CheckCycleValid(int drive)
{
   int s16_middle = BGVAR(u16_Cycle_Counter) >> 1;
   long long s64_temp,s64_temp2,s64_acc_minmax_errors = 0;
   int s16_pairs_fault = 0;
   int s16_temp,s16_temp2;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   for(s16_temp=0;((s16_temp<s16_middle) && (s16_pairs_fault == 0)) ;++s16_temp)
   {
      s64_temp = GetExtrema(s16_temp,0);
      s64_temp2 = GetExtrema(s16_temp+s16_middle,0);

      // verify both are of the same type : min or max
      s16_temp2 = (int)s64_temp ^ (int)s64_temp2;
      if ((s16_temp2 & 0x01) != 0) s16_pairs_fault = 1;

      s64_temp  &= 0xFFFFFFFFFFFFFFFE;
      s64_temp2 &= 0xFFFFFFFFFFFFFFFE;

      // accomulate error of the same point in the two suspected cycles
      s64_acc_minmax_errors += llabs(s64_temp-s64_temp2);
      s64_acc_minmax_errors -= GetExtrema(s16_temp,1)<<1;
      s64_acc_minmax_errors -= GetExtrema(s16_temp+s16_middle,1)<<1;
   }

   if ( (s64_acc_minmax_errors <= 0) && // min max errors
           (s16_pairs_fault == 0)      ) // pairs fault
      return 1;

  return 0;
}


void CycleidentificationFlashCode1(int drive)
{ // due to memory issue move part of the cycle identification to flash
  // the load casuesd WD during AT with DDHD
   int s16_temp;
   unsigned long u32_temp;
   float f_cost_scale_stand;
   long long s64_temp,s64_tqf_noise_hi,s64_tqf_noise_lo;
   // AXIS_OFF;

  // every 2 mSec - execute the low freq vibration detector code
    if ((Cntr_1mS & 0x0001L) == 0)
    {
      LowFreqOScilationDetector(drive,0);
      // capture the time from beging of cycle in which instability is at its peak indicattion
      // AV recorder need to record around this point
      if (BGVAR(s32_Hdtune_Low_Frq_Osc_Max_Capture) <  BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle))
            BGVAR(s32_Hdtune_Low_Frq_Osc_Max_Capture) = BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle);


      if (BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) > BGVAR(s32_Hdtune_Low_Frq_Osc_Max))
      {
         BGVAR(s32_Hdtune_Low_Frq_Osc_Max) = BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle);
         if (BGVAR(u32_Cycle_OverAll_Counter) > 250)
            BGVAR(u32_Hdtune_Low_Frq_Osc_Max_Time) = BGVAR(u32_Cycle_OverAll_Counter) - 250;
         else
            BGVAR(u32_Hdtune_Low_Frq_Osc_Max_Time) = 10;
      }
   }
   // signal Av osc trigger
   if (BGVAR(u32_Cycle_OverAll_Counter) > BGVAR(u32_Hdtune_Low_Frq_Osc_Max_Time)) u16_AntiVib_Osc_Trig = 10;

   if ((VAR(AX0_s16_Cycle_Bits) & CYCLE_POSSIBLE_CYCLE_FOUND) != 0)
   {// compare patterns
      VAR(AX0_s16_Cycle_Bits) &= ~CYCLE_POSSIBLE_CYCLE_FOUND;
      VAR(AX0_s16_Cycle_Bits) |= CYCLE_POSSIBLE_CYCLE_TESTED;

      if (CheckCycleValid(drive) == 1)
      {  // valid cycle found

         // inertia ident for cyclic motion
         // integral(J*acc) = 0 = integral(Tm)-intgeral(Tunbalance) - B*intgeral(v)
         // for complete cycle (i.e. end points = start points) Tf = Mkt/Time*integral(icmd)
         // integral*4/26214*dipeak/1000*31.25e-6*Mkt/1000*cycle_overall_counter*4/1000. mpy the output by 1000 for scaling
         // = 1.9073777370870527199206530861372e-14*s32_drive_i_peak*u32_motor_kt*cycle_overall_counter*integral1
         do {
             s16_temp = Cntr_3125;
             s64_temp = LLVAR(AX0_u32_Abs_Icmd_Acc_Lo);
             u32_temp = LVAR(AX0_u32_Abs_Icmd_ACC_Cntr);
         } while (s16_temp != Cntr_3125);

         // igrav tuning
         if (BGVAR(s16_HdTune_Igrav_En) > 0)
         {
            if (u32_temp == 0)
               s64_temp = 0;
            else
               s64_temp = (long long)(4.0*(float)s64_temp/(float)u32_temp);
            if  (s64_temp > 26214LL) s64_temp = 26214LL;
            if  (s64_temp < -26214LL) s64_temp = -26214LL;
            BGVAR(s16_Igrav_Tuning) = (int)s64_temp;
         }
         else
         {
            // unbalanced mpy by 1000 for better resolution. units are 0.001*Nm
            if (BGVAR(s16_Pos_Tune_Cycle) >= 2)
               BGVAR(s32_Cycle_T_Unbalance) = 0;
            else
               BGVAR(s32_Cycle_T_Unbalance) = (long)(4.76844e-9*(float)BGVAR(u32_Motor_Kt)*(float)s64_temp * (float)BGVAR(s32_Drive_I_Peak)/(float)BGVAR(u32_Cycle_OverAll_Counter));
            if (BGVAR(u32_Pwm_Freq) == 8000L) BGVAR(s32_Cycle_T_Unbalance) <<= 1; // pwm 8 tsc is 62.5e-6 and not 31.25e-6
            else if (BGVAR(u32_Pwm_Freq) == 4000L) BGVAR(s32_Cycle_T_Unbalance) <<= 2; // pwm 4 tsc is 125e-6 and not 31.25e-6
         }

         // calc standstill to cycle ratio
         f_cost_scale_stand = 1.0 - (50.0 / (float)BGVAR(s16_Pos_Tune_Weight)) * (float)BGVAR(u32_Cycle_Standing_Counter) / (float)BGVAR(u32_Cycle_OverAll_Counter);
         VAR(AX0_s16_At_Cost_Stand_Scale) = (int)(16384.0*f_cost_scale_stand);
         VAR(AX0_s16_At_Cost_Move_Scale) = 16384-VAR(AX0_s16_At_Cost_Stand_Scale);
         // match the higher number to 0x4000
         s16_temp = VAR(AX0_s16_At_Cost_Stand_Scale);
         if (s16_temp < VAR(AX0_s16_At_Cost_Move_Scale)) s16_temp = VAR(AX0_s16_At_Cost_Move_Scale);
         f_cost_scale_stand = 16384.0/(float)s16_temp;
         VAR(AX0_s16_At_Cost_Stand_Scale) = (int)((float)VAR(AX0_s16_At_Cost_Stand_Scale) * f_cost_scale_stand);
         VAR(AX0_s16_At_Cost_Move_Scale) = (int)((float)VAR(AX0_s16_At_Cost_Move_Scale) * f_cost_scale_stand) ;

          BGVAR(u32_Cycle_OverAll_Counter_GUI) = BGVAR(u32_Cycle_OverAll_Counter);
         BGVAR(u32_Cycle_Standing_Counter) = 0;
         BGVAR(u32_Cycle_OverAll_Counter) = 0;

         if ( (BGVAR(u16_Cycle_Length) != 0)                        &&
              (BGVAR(u16_Cycle_Counter) != BGVAR(u16_Cycle_Length))   )
         {
            // cycle length mismatch - signal auto tune and restart cycle identification
            VAR(AX0_s16_Cycle_Bits) |= CYCLE_LENGTH_ERROR;
            BGVAR(u16_Cycle_Counter) = 0;
            VAR(AX0_s16_StandStill_Counter) = 0;
            BGVAR(s16_Cycle_StandStill_Found) = 0;
            BGVAR(u16_StandStill_Ticks) = BGVAR(u16_HDTUNE_Dwell_Time_mS);
            BGVAR(s16_Cycle_And_Standing_Ticks) = 0;
            BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_CYCLE_WARN;
            BGVAR(u16_Cycle_Max_STicks_Best) = 0;
            BGVAR(s16_Cycle_LMJR_Counter) = 0;
            BGVAR(u16_Cycle_Length) = 0;
            BGVAR(u32_Cycle_OverAll_Counter_GUI) = 0;
            CalculateLMJR(0,0,0,0); // reset static vars
         }
         else
         {
            // Verify the current level is enough for LMJR calculation
            // Issue fault if current level is lower than 5% of MICONT
            if(VAR(AX0_s16_Icmd_100Hz_Filtered_Max) < BGVAR(s16_Motor_I_Cont_5_Prcnt))
                BGVAR(s16_Low_Current_Level_Cntr)++;
            else
                BGVAR(s16_Low_Current_Level_Cntr) = 0;
            VAR(AX0_s16_Icmd_100Hz_Filtered_Max) = 0;
            
            VAR(AX0_s16_Cycle_Bits) |= CYCLE_END_AND_STANDING;
            BGVAR(s16_Cycle_And_Standing_Ticks) = 0;
            BGVAR(u16_Cycle_Length) = BGVAR(u16_Cycle_Counter);
            BGVAR(u16_Cycle_Counter) = BGVAR(u16_Cycle_Counter) >> 1;
            ++BGVAR(u16_Cycle_Cycles_Counter);
            do {
                s16_temp = Cntr_3125;
                BGVAR(s64_Cost_Function_Final_Capture) = LLVAR(AX0_u32_At_Final_Cost_Lo);
                BGVAR(s64_At_TQF) = LLVAR(AX0_u32_TQF_Acc_Lo);
                BGVAR(u32_Tqf_Extrema_Cntr) = LVAR(AX0_u32_PE_Extremum_Cntr);
            } while (s16_temp != Cntr_3125);


            if ((BGVAR(u16_RT_Tqf_Control) == 1)                              &&
                (BGVAR(u32_Tqf_Extrema_Cntr) > MIN_TQF_EXTREMA)               &&
                (BGVAR(s64_Cost_Function_Final_Capture) < 0x7FFFFFFFFFFFFFF0) /*&&
                (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_2)        */)
            {
               BGVAR(s64_Tune_Min_Tqf) = 0x7FFFFFFFFFFFFFFF;
               StoreMinTQFResult(drive,1);
               BGVAR(s32_Pos_Tune_Bits) |= TQF_FUNC_ENABLED;
               s32_TuneSearchTableMinTQF[0] = -1;
               BGVAR(u16_RT_Tqf_Control) = 0;
            }

            if ( (BGVAR(s64_Cost_Function_Final_Capture) < 0x7FFFFFFFFFFFFFF0) &&
                 ((BGVAR(s32_Pos_Tune_Bits) & TQF_FUNC_ENABLED) != 0)            )
            {
               s64_temp = (long long)(1.5*(float)BGVAR(s64_At_TQF)/(float)BGVAR(u32_Tqf_Extrema_Cntr));
               s64_tqf_noise_hi = BGVAR(s64_At_TQF)+s64_temp;
               s64_tqf_noise_lo = BGVAR(s64_At_TQF)-s64_temp;

               if (s64_tqf_noise_lo < 0) s64_tqf_noise_lo = 0;

               if (BGVAR(u32_Tqf_Extrema_Cntr) < MIN_TQF_EXTREMA) BGVAR(s64_At_TQF) = BGVAR(s64_Tune_Min_Tqf);
               else
               {
                  BGVAR(s64_Tune_Min_Tqf) = 0x7FFFFFFFFFFFFFFF;
                  StoreMinTQFResult(drive,1);
                  s32_TuneSearchTableMinTQF[0] = -1;
               }


               if (s64_tqf_noise_hi < BGVAR(s64_Tune_Min_Tqf))
               {
                  BGVAR(s64_Tune_Min_Tqf) = BGVAR(s64_At_TQF);
                  StoreMinTQFResult(drive,0);
               }
               if (BGVAR(s64_Tune_Min_Tqf) != 0x7FFFFFFFFFFFFFFF)
               {
                  s64_temp = (5*BGVAR(s64_Tune_Min_Tqf)) >> 1;
                  //   s64_temp = (7*BGVAR(s64_Tune_Min_Tqf)) >> 2;
                  if (s64_temp < 0) s64_temp = 0x7FFFFFFFFFFFFFFF; // protect from OV
                  if (s64_tqf_noise_lo > s64_temp) BGVAR(s64_Cost_Function_Final_Capture) =  0x7FFFFFFFFFFFFFF1;
               }
            }
            BGVAR(u32_At_Control_Effort_Max_Captured) = LVAR(AX0_u32_At_Control_Effort_Max);
            if ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_EFFORT_RESET) == 0) LVAR(AX0_u32_At_Control_Effort_Max) = 0;

            if ((VAR(AX0_s16_Cycle_Bits) & (CYCLE_PARAM_UPDATE|CYCLE_LENGTH_ERROR)) == 0) VAR(AX0_s16_Cycle_Bits) |= CYCLE_NEW_CYCLE_DATA_READY;
            VAR(AX0_s16_Cycle_Bits) &= ~CYCLE_LENGTH_ERROR;

            if (BGVAR(s16_HdTune_Igrav_En) != 0) AX0_AutoTune_Bits |= AT_RT_STANDSTILL_INTEGRATE_MASK;
            else if (BGVAR(s16_Pos_Tune_Cycle) >= 2)
            {
               if ((AX0_AutoTune_Bits & AT_RT_STANDSTILL_INTEGRATE_MASK) != 0)
                  AX0_AutoTune_Bits &= ~AT_RT_STANDSTILL_INTEGRATE_MASK;
               else
                  AX0_AutoTune_Bits |= AT_RT_STANDSTILL_INTEGRATE_MASK;
            }
            else AX0_AutoTune_Bits &= ~AT_RT_STANDSTILL_INTEGRATE_MASK;
         }
      }
   }

   if ((VAR(AX0_s16_Cycle_Bits) & CYCLE_END_AND_STANDING) != 0)
   {
      VAR(AX0_AutoTune_Bits) &= ~AT_DURING_CYCLE_MASK;

      if ((VAR(AX0_s16_Cycle_Bits) & CYCLE_PARAM_UPDATE) != 0)
      {
         VAR(AX0_s16_Cycle_Bits) &=~CYCLE_PARAM_UPDATE;
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         VAR(AX0_s16_Cycle_Bits) &= ~(CYCLE_NEW_CYCLE_DATA_READY);
      }
      do {
          s16_temp = Cntr_3125;
          LLVAR(AX0_u32_At_Cost_Function_Lo) = 0LL;
          LLVAR(AX0_u32_At_Final_Cost_Lo) = 0LL;
          LLVAR(AX0_u32_TQF_Acc_Lo) = 0LL;
          LLVAR(AX0_u32_Abs_Icmd_Acc_Lo) = 0LL;
          LVAR(AX0_u32_Abs_Icmd_ACC_Cntr) = 0L;
          BGVAR(s16_At_Vel_T0) = VAR(AX0_s16_Vel_Var_Fb_0);
      } while (s16_temp != Cntr_3125);
      BGVAR(u32_Cycle_Standing_Counter) = 0;
      BGVAR(u32_Cycle_OverAll_Counter) = 0;
      LVAR(AX0_u32_PE_Extremum_Cntr) = 0;
      VAR(AX0_u16_Tqf_Flag) = 1;
      VAR(AX0_s16_Cycle_Bits) &= ~EFFORT_EXCEEDED;
      BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) = 0;
      BGVAR(s32_Hdtune_Low_Frq_Osc_Max) = 0;
      u16_AntiVib_Osc_Trig = 0;
      ++BGVAR(s16_Cycle_And_Standing_Ticks);
      VAR(AX0_AutoTune_Bits) &= ~(AT_PARTIAL_CYCLE_MASK|AT_ACC_PARTIAL_CYCLE_MASK);
   }
   else
   {
      VAR(AX0_AutoTune_Bits) |= AT_DURING_CYCLE_MASK;


      s16_temp = 0;
      if ((BGVAR(s16_Pos_tune_Mode) >= 9) && (BGVAR(s16_Pos_tune_Mode) <= 11))
      {
         if ((BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) >= BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim)) &&
             (BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) > 0)                                    &&
             (BGVAR(s16_Pos_Tune_State) != POS_TUNE_CONT_INITIAL_COST_SETUP)                       ) s16_temp = 1;
         else if ((VAR(AX0_AutoTune_Bits) & AT_ICMD_OV_MASK) != 0) s16_temp = 2;
      }
      if ((BGVAR(u16_At_Pretubation_En) != 0)                                                 &&
          (BGVAR(s32_Hdtune_Low_Frq_Osc_Cycle) >= BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim)) &&
          (BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) > 0)                                      ) s16_temp = 3;

      if ( (BGVAR(u16_At_Foldback_Capture) != 0)                                                   ||
           (BGVAR(u16_At_Fold) != 0)                                                               ||
           (s16_temp != 0)                                        )
      {
         do {
            s16_temp = Cntr_3125;
            LLVAR(AX0_u32_At_Final_Cost_Lo) = 0x7FFFFFFFFFFFFFFF;
            } while (s16_temp != Cntr_3125);
         VAR(AX0_s16_Cycle_Bits) |= EFFORT_EXCEEDED;
      }
   }
}

#pragma CODE_SECTION(CycleIndetificationRAM, "ramfunc_3");

long long CycleIndetificationRAM(int drive)
{
    int s16_temp_arr_cntr;
    int s16_buffer_size,s16_temp;
    long long s64_cycle_data,s64_temp,s64_temp2;
    // AXIS_OFF;

    // Cyclic 8 position buffer is used to provide the latest 8 samples of data from RT
   // move 8 last data samples from cyclic buffer to array
   // If Tsp is 250 us than take only 4 values, else take 8 values
   REFERENCE_TO_DRIVE;
   s16_buffer_size = 8;
   if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) s16_buffer_size = 4;
   BGVAR(s16_At_Buff_Size) = 0;
   //s16_tmp_max_dss = 0;
    for(s16_temp_arr_cntr = 0 ; BGVAR(s16_At_Buff_Size) < s16_buffer_size  ; s16_temp_arr_cntr++)
    {
       do {
         s16_temp = Cntr_3125;
         //s16_temp2 = VAR(AX0_s16_RT_To_1mSec_Capture_Pcmd_Raw_Tail) - (VAR(AX0_s16_RT_To_1mSec_Capture_Pcmd_Raw_Head)>>2);
         s64_cycle_data = LLVAR(AX0_s64_RT_To_1mSec_Capture_Pcmd_Raw_Buff[VAR(AX0_s16_RT_To_1mSec_Capture_Pcmd_Raw_Tail)]);
         } while (s16_temp != Cntr_3125);
         
       //if(s16_temp2 < 0) s16_temp2 = -s16_temp2;
       //s16_tmp_dss = s16_temp2;
       //if (s16_tmp_max_dss<s16_tmp_dss)  s16_tmp_max_dss = s16_tmp_dss;
        // Check if Tail lags Head more then buffer size. This may cause to cycle buffer corruption
       //if(s16_temp2 > (s16_buffer_size))     BGVAR(s32_Pos_Tune_Bits) |= CYCLE_BUFF_CORRUPT_WARN_MASK;

       ++VAR(AX0_s16_RT_To_1mSec_Capture_Pcmd_Raw_Tail);
       if (VAR(AX0_s16_RT_To_1mSec_Capture_Pcmd_Raw_Tail) == 15) VAR(AX0_s16_RT_To_1mSec_Capture_Pcmd_Raw_Tail) = 0;
       ++BGVAR(s16_At_Buff_Size);

       if (BGVAR(s16_Pos_Tune_Cycle) >= 2) s64_cycle_data &= 0xFFFFFFFFFFFFFFF0; // dont use 4 lower bit of vcmd - unidirectional cycle

   s64_temp = s64_cycle_data - BGVAR(s64_Prev_Cycle_Data);
   if (s64_temp < 0) s64_temp =-s64_temp;

   if ( (s64_cycle_data < BGVAR(s64_Prev_Cycle_Data))        &&
        (VAR(AX0_s16_Cycle_Dir) > 0)                             )
   { // BGVAR(s64_Prev_Cycle_Data) is possibly max value. lsbit = 1 => max 0=> min
      if (BGVAR(u16_Cycle_Counter) != 0)
         s64_temp2 = (GetExtrema((int)BGVAR(u16_Cycle_Counter)-1,0) & 0xFFFFFFFFFFFFFFFE) - BGVAR(s64_Prev_Cycle_Data);
      else if (BGVAR(s16_Cycle_StandStill_Found) != 0) s64_temp2 = BGVAR(s64_Cycle_Last_Extrema)- BGVAR(s64_Prev_Cycle_Data);// if standstill found use last extrema inorder not to select a filtered position

      if (s64_temp2 < 0) s64_temp2 =-s64_temp2;
      if ((s64_temp2 >= 0) || (BGVAR(s16_Cycle_StandStill_Found) == 0))
      {
         StoreExtrema(((BGVAR(s64_Prev_Cycle_Data) /*& 0xFFFFFFFFFFFFFFFE*/) | 0x01),BGVAR(u16_Cycle_Counter),0);
         StoreExtrema((long long)(labs((long)(s64_cycle_data - BGVAR(s64_Prev_Cycle_Data)))),BGVAR(u16_Cycle_Counter),1);
         BGVAR(u16_Cycle_Counter)++;
      }
      else if (BGVAR(u16_Cycle_Counter) > 0) BGVAR(u16_Cycle_Counter)--;
      VAR(AX0_s16_Cycle_Dir) = -1;
   }

   if ( (s64_cycle_data > BGVAR(s64_Prev_Cycle_Data))        &&
        (VAR(AX0_s16_Cycle_Dir) < 0)                             )
   { // BGVAR(s64_Prev_Cycle_Data) is possibly min value. lsbit = 1 => max 0=> min
      if (BGVAR(u16_Cycle_Counter) != 0)
         s64_temp2 = (GetExtrema((int)BGVAR(u16_Cycle_Counter)-1,0) & 0xFFFFFFFFFFFFFFFE) - BGVAR(s64_Prev_Cycle_Data);
      else if (BGVAR(s16_Cycle_StandStill_Found) != 0) s64_temp2 = BGVAR(s64_Cycle_Last_Extrema)- BGVAR(s64_Prev_Cycle_Data);// if standstill found use last extrema inorder not to select a filtered position

      if (s64_temp2 < 0) s64_temp2 =-s64_temp2;
      if ((s64_temp2 >= 0) || (BGVAR(s16_Cycle_StandStill_Found) == 0))
      {
         StoreExtrema((BGVAR(s64_Prev_Cycle_Data) & 0xFFFFFFFFFFFFFFFE),BGVAR(u16_Cycle_Counter),0);
         StoreExtrema((long long)(labs((long)(s64_cycle_data - BGVAR(s64_Prev_Cycle_Data)))),BGVAR(u16_Cycle_Counter),1);
         BGVAR(u16_Cycle_Counter)++;
      }
      else if (BGVAR(u16_Cycle_Counter) > 0) BGVAR(u16_Cycle_Counter)--;
      VAR(AX0_s16_Cycle_Dir) = 1;
   }

   // find stand still condition and count stand still time
   if (s64_temp == 0LL)
   { // axis did not move enough
      ++BGVAR(u32_Cycle_Standing_Counter);
      if ((VAR(AX0_s16_Cycle_Bits) & PARAM_UPDATE_AT_STANDSTILL) != 0)
      {  // copy pos loop gains at stands still
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         VAR(AX0_s16_Cycle_Bits) &= ~PARAM_UPDATE_AT_STANDSTILL;
      }

      if (VAR(AX0_s16_StandStill_Counter) == 0)
      {
         BGVAR(s64_Cycle_Standstiil_Data) = s64_cycle_data;
         VAR(AX0_s16_StandStill_Counter) = 1;
         VAR(AX0_s16_Cycle_Bits) |= CYCLE_POSSIBLE_STANDING;
      }
      else
      {
         s64_temp = BGVAR(s64_Cycle_Standstiil_Data) - s64_cycle_data;
         if (s64_temp < 0) s64_temp=-s64_temp;
         if (s64_temp > 0LL) // not moving but not at start\end cycle pcmd
         {
            if (((VAR(AX0_s16_Cycle_Bits) & CYCLE_END_AND_STANDING) != 0) &&
                ((VAR(AX0_s16_Cycle_Bits) & CYCLE_LENGTH_ERROR) == 0)       )
            {
               if (((BGVAR(s16_Pos_Tune_Cycle) == 0) || (BGVAR(s16_Pos_Tune_Cycle) == 2) ) &&
                   (BGVAR(u16_StandStill_Ticks) == BGVAR(u16_HDTUNE_Dwell_Time_mS))          )
               {
                  s16_temp = BGVAR(s16_Cycle_And_Standing_Ticks) - 140; // allow 140 mSec dwell time for pos config etc...
                  if (s16_temp < 0) s16_temp = 0;
                  BGVAR(u16_StandStill_Ticks) += s16_temp;
                  if (BGVAR(u16_StandStill_Ticks) >  1000) BGVAR(u16_StandStill_Ticks) = 1000;
               }
            }
            VAR(AX0_s16_StandStill_Counter) = 0;
            VAR(AX0_s16_Cycle_Bits) &= ~(CYCLE_END_AND_STANDING|CYCLE_POSSIBLE_CYCLE_TESTED);
         }
         else
            ++VAR(AX0_s16_StandStill_Counter);

         if (VAR(AX0_s16_StandStill_Counter) > BGVAR(u16_StandStill_Ticks))
         {
            // filter last extrema if its too close to standstill

            VAR(AX0_s16_StandStill_Counter) = BGVAR(u16_StandStill_Ticks);
            ++BGVAR(u16_Cycle_Max_STicks_Counter);
            if (((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_LONGEST_CYCLE_WARN) != 0)                &&
                (BGVAR(u16_Cycle_Counter) > 1)                                                 &&
                (BGVAR(u16_Cycle_Length) != 0)                                                  )
            {
               s64_temp = llabs(BGVAR(s64_Cycle_Max_STicks_Pos) - s64_cycle_data);
               if (s64_temp == 0LL)
               {
                  BGVAR(s16_Cycle_LMJR_Counter) = 0;
                  BGVAR(s32_Pos_Tune_Bits) &= ~POS_TUNE_LONGEST_CYCLE_WARN;
                  BGVAR(s64_Cycle_Last_Extrema) = GetExtrema(BGVAR(u16_Cycle_Counter)-1,0);
                  BGVAR(s16_Cycle_StandStill_Found) = 1;
                  BGVAR(u16_Cycle_Counter) = 0;
                  BGVAR(s64_Cycle_Standstill_Pos) = BGVAR(s64_Cycle_Standstiil_Data);
                  BGVAR(u16_Cycle_Length) = 0;
                  BGVAR(u32_Cycle_Standing_Counter) = 0;
                  BGVAR(u32_Cycle_OverAll_Counter) = 0;
                  BGVAR(u32_Cycle_OverAll_Counter_GUI) = 0;
                  CalculateLMJR(0,0,0,0); // reset static vars
               }
            }
         }
      }
   }
   else // axis is not standing
   {
      if (((VAR(AX0_s16_Cycle_Bits) & CYCLE_END_AND_STANDING) != 0) &&
          ((VAR(AX0_s16_Cycle_Bits) & CYCLE_LENGTH_ERROR) == 0)       )
      {
         if ((BGVAR(s16_Pos_Tune_Cycle) == 0)                                          &&
             (BGVAR(u16_StandStill_Ticks) == BGVAR(u16_HDTUNE_Dwell_Time_mS))          &&
             ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_LONGEST_CYCLE_WARN) == 0)             )
         {
            s16_temp = BGVAR(s16_Cycle_And_Standing_Ticks) - 140; // allow 140 mSec dwell time for pos config etc...
            if (s16_temp < 0) s16_temp = 0;
            BGVAR(u16_StandStill_Ticks) += s16_temp;
            if (BGVAR(u16_StandStill_Ticks) >  1000) BGVAR(u16_StandStill_Ticks) = 1000;
         }
      }
      VAR(AX0_s16_StandStill_Counter) = 0;
      VAR(AX0_s16_Cycle_Bits) &= ~(CYCLE_END_AND_STANDING|CYCLE_POSSIBLE_CYCLE_TESTED);

      s64_temp = llabs(BGVAR(s64_Prev_Cycle_Data) - BGVAR(s64_Cycle_Standstill_Pos));

      if ((BGVAR(u16_Cycle_Max_STicks_Counter) > BGVAR(u16_Cycle_Max_STicks_Best)) &&
          (s64_temp > 0LL)                                   )
      {
         BGVAR(u16_Cycle_Max_STicks_Best) = BGVAR(u16_Cycle_Max_STicks_Counter);
         BGVAR(s64_Cycle_Max_STicks_Pos) = BGVAR(s64_Prev_Cycle_Data);
         BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_LONGEST_CYCLE_WARN;
         BGVAR(u16_StandStill_Ticks) = BGVAR(u16_HDTUNE_Dwell_Time_mS);
      }
      BGVAR(u16_Cycle_Max_STicks_Counter) = 0;
   }

   if ( (VAR(AX0_s16_StandStill_Counter) == BGVAR(u16_StandStill_Ticks)) &&
        (BGVAR(u16_Cycle_Counter) > 1)                       &&
        (BGVAR(s16_Cycle_StandStill_Found) == 0)                )
   {
      BGVAR(s64_Cycle_Last_Extrema) = GetExtrema((int)BGVAR(u16_Cycle_Counter)-1,0);
      BGVAR(s16_Cycle_StandStill_Found) = 1;
      BGVAR(u16_Cycle_Counter) = 0;
      BGVAR(s64_Cycle_Standstill_Pos) = BGVAR(s64_Cycle_Standstiil_Data);
      BGVAR(u16_Cycle_Length) = 0;
      BGVAR(u32_Cycle_Standing_Counter) = 0;
      BGVAR(u32_Cycle_OverAll_Counter) = 0;
      BGVAR(u32_Cycle_OverAll_Counter_GUI) = 0;
   }
   else if ( (VAR(AX0_s16_StandStill_Counter) == 0)    &&
             (BGVAR(s16_Cycle_StandStill_Found) == 1)   ) BGVAR(s16_Cycle_StandStill_Found) = 2;
   else if ( (VAR(AX0_s16_StandStill_Counter) == BGVAR(u16_StandStill_Ticks))    &&
             (BGVAR(s16_Cycle_StandStill_Found) == 2)   ) BGVAR(s16_Cycle_StandStill_Found) = 3;
   else if ( (VAR(AX0_s16_StandStill_Counter) == 0)    &&
             (BGVAR(s16_Cycle_StandStill_Found) == 3)   ) BGVAR(s16_Cycle_StandStill_Found) = 4;


   s64_temp = s64_cycle_data - BGVAR(s64_Cycle_Standstill_Pos);
   if (s64_temp < 0) s64_temp=-s64_temp;

   if ( (BGVAR(s16_Cycle_StandStill_Found) == 4)                              &&
        (s64_temp == 0LL )                             &&
        (VAR(AX0_s16_StandStill_Counter) == BGVAR(u16_StandStill_Ticks))      &&
        (BGVAR(u16_Cycle_Counter) != 0)                                       &&
        (((BGVAR(u16_Cycle_Counter) & BGVAR(u16_Cycle_Counter_Mask)) == 0))   &&
        ((VAR(AX0_s16_Cycle_Bits) & CYCLE_POSSIBLE_CYCLE_TESTED) == 0)           ) VAR(AX0_s16_Cycle_Bits) |= CYCLE_POSSIBLE_CYCLE_FOUND;

       BGVAR(s64_Prev_Cycle_Data) = s64_cycle_data; // age data
   }
   return s64_temp;
} 



void CycleIndetification(int drive)
{
   long long s64_temp, s64_temp2, s64_temp3;
   int s16_temp,s16_vel;
   float f_temp;
   long s32_temp,s32_delta_v,s32_delta_v_thershold,s32_temp_thershold;
   unsigned long u32_temp;
   // AXIS_OFF;

   // Get Icmd integral. Used for EASY profile calculation
   if((VAR(AX0_AutoTune_Bits) & AT_PTT_VEL_START_CAPTURE_MASK) != 0)
   {
       // Check if profile reached plateau
       if ((LVAR(AX0_s32_Vm) <= BGVAR(s32_PTT_Pos_Vcmd_Prev)) &&
           (LVAR(AX0_s32_Vm) > 0L))
          {
              do {
                    s16_temp = Cntr_3125;
                    BGVAR(s64_PTT_Captured_Curr_Int)  = LLVAR(AX0_u32_Abs_Icmd_Acc_Lo);
                    BGVAR(u32_PTT_Captured_Curr_Int_Cntr) = LVAR(AX0_u32_Abs_Icmd_ACC_Cntr);
                    BGVAR(s32_Capt_Vel_Var_Fb_0) = LVAR(AX0_s32_Vel_Var_Fb_0);
                } while (s16_temp != Cntr_3125);
              VAR(AX0_AutoTune_Bits) &= ~AT_DURING_CYCLE_MASK;
              VAR(AX0_AutoTune_Bits) &= ~AT_PTT_VEL_START_CAPTURE_MASK;
          }
       else
           {
              // Use raw ptpvcmd
              BGVAR(s32_PTT_Pos_Vcmd_Prev) = LVAR(AX0_s32_Vm);
           }
       if (LVAR(AX0_s32_Vm) == 0L)
       {
         do {
              s16_temp = Cntr_3125;
              LLVAR(AX0_u32_Abs_Icmd_Acc_Lo) = 0LL;
              LVAR(AX0_u32_Abs_Icmd_ACC_Cntr) = 0L;
              BGVAR(s32_PTT_Pos_Vcmd_Prev) = 0L;
            } while (s16_temp != Cntr_3125);
       }
   }
   // Init BG buffer tail with RT buffer head while not enabled
   if ( (!Enabled(drive))                                            ||
        ( (VAR(AX0_s16_Opmode) != 8) && (VAR(AX0_s16_Opmode) != 4) ) ||
        (VAR(AX0_s16_Cycle_Dir) == 0x7FFF)                             )
   {
      BGVAR(s16_BG_Isr_Cntr) = Cntr_3125;
      ResetCycleIdentification(drive);
      return;
   }
   
   s16_temp = (int)Cntr_3125 - BGVAR(s16_BG_Isr_Cntr);
   if (s16_temp < 0) s16_temp = -s16_temp;
   // timeout warning at 1.25 mS. 40 = 1.25m / 31.25u. This may cause to cycle buffer corruption
   if (s16_temp >= 40)       BGVAR(s32_Pos_Tune_Bits) |= CYCLE_BUFF_CORRUPT_WARN_MASK;
   BGVAR(s16_BG_Isr_Cntr) = Cntr_3125;  
      
   CycleidentificationFlashCode1(drive);

   // find min/max points and capture the pcmd\ptpvcmd of these points
   if ( (BGVAR(u16_Cycle_Counter) == CYCLE_DATA_ARRAY_SIZE)                                                              ||
        ((BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_IDLE) && (BGVAR(s16_Pos_Tune_Traj_Enable)) && (BGVAR(u16_Cycle_Counter) >= 10))   )
      CycleArrayOverFlow(drive);
    
   s64_temp = CycleIndetificationRAM(drive);
   s64_temp2 = s64_temp;
   if(s64_temp < 0LL) s64_temp = s64_temp;
   ++BGVAR(u32_Cycle_OverAll_Counter);

   // setup antivib trigger
   if ((VAR(AX0_s16_StandStill_Counter) == 0) || (s64_temp > 0LL)) u16_AntiVib_End_Cycle_Trig = 0;
   else u16_AntiVib_End_Cycle_Trig = 10;

   s32_temp = labs(LVAR(AX0_s32_Pos_Vcmd));

   if ((VAR(AX0_AutoTune_Bits) & AT_RT_STANDSTILL_INTEGRATE_MASK) == 0)
   {
      // inertia ident for cyclic motion
      // integral(J*acc) = 0 = integral(Tm)-intgeral(Tunbalance) - B*intgeral(v)
      // for incomplete cycle (i.e. current point is standing && stand points != start points)
      // 0 = integral(Tm)-intgeral(Tunbalance) - B*intgeral(v)
      do {
          s16_temp = Cntr_3125;
          s64_temp3 = LLVAR(AX0_u32_Abs_Icmd_Acc_Lo);
          u32_temp = LVAR(AX0_u32_Abs_Icmd_ACC_Cntr);
          s16_vel = VAR(AX0_s16_Vel_Var_Fb_0);

      } while (s16_temp != Cntr_3125);

      s16_temp = 0;
      if ((BGVAR(s16_Pos_Tune_Cycle) < 2) && (VAR(AX0_s16_StandStill_Counter) != 0) && (s64_temp > 0LL) ) s16_temp = 1;
      else if ( ((BGVAR(s16_Pos_Tune_Cycle) >= 2) && (s32_temp < 5) && (VAR(AX0_u16_Pos_Loop_Tsp) != TSP_250_USEC)) ||
                ((BGVAR(s16_Pos_Tune_Cycle) >= 2) && (s32_temp < 10) && (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC))   )s16_temp = 1;

      if ( (s16_temp == 1)                                         &&
           (BGVAR(s16_Cycle_StandStill_Found) == 4)                &&
           ((VAR(AX0_AutoTune_Bits) & AT_PARTIAL_CYCLE_MASK) == 0) && // standing but not at end of cycle
           (BGVAR(s16_HdTune_Igrav_En) == 0)                         )
      {
         if (BGVAR(s16_LMJR_Mode) == 2) VAR(AX0_AutoTune_Bits) &= ~AT_ACC_PARTIAL_CYCLE_MASK;
         VAR(AX0_AutoTune_Bits) |= AT_PARTIAL_CYCLE_MASK;
         //4*31.25u/26214*1e-6 = 4.76844e-15
         f_temp = (4.76844e-15*(float)BGVAR(u32_Motor_Kt)*(float)s64_temp3 * (float)BGVAR(s32_Drive_I_Peak));
         if (BGVAR(u32_Pwm_Freq) == 8000) f_temp = f_temp*2; // pwm 8 tsc is 62.5e-6 and not 31.25e-6
         f_temp = f_temp  - (1e-6*(float)BGVAR(s32_Cycle_T_Unbalance)*(float)BGVAR(u32_Cycle_OverAll_Counter));
         // B = MKT*integral(Icmd)/integral(w)
         // integral(w) = displacement
         // Convert internal position units to rad
         // 2pi/2^32 = 1.462918e-9
         f_temp = f_temp / ((float)s64_temp2*1.462918e-9);
         // multiply by 1e8 to support floating point numbers
		 // (assume the worst case B = 1 Nm/rad/s,
		 // typical value is ~1e-3 Nm/rad/s
         // Make abs(fric_coeff) since we have abs(itegral(w))
         if(f_temp < 0.0) f_temp = 0.0; 
         if(f_temp > 1.0) f_temp = 1.0;        
         // hence the max num will be 1e8 what will not exceed 2^31
         BGVAR(s32_Cycle_Fric_Coeff) = (long)(f_temp*1e8) ; 
         if (BGVAR(s16_Pos_Tune_Cycle) >= 2) BGVAR(s32_Cycle_Fric_Coeff) = 0;
      }

      // inertia ident for cyclic motion
      // integral(J*acc) = 0 = integral(Tm)-intgeral(Tunbalance) - B*intgeral(v)
      // for incomplete the acceleration stage. start points = standstill point. end point = end of acceleration point
      // J * (vend-0) = mkt*integral(icmd)-Tf*Time-B*deltaMotion
      // J = (mkt*integral(icmd)-Tf*Time-B*deltaMotion)/Vend
      //   = (31.25e-6*mkt/1000*4*integral(icmd)/26214*dipeak/1000-Tf/1000*overallTime*4/1000-B/1000*deltapfb/2^32*2*pi)/(vel_var_fb_o)/22750*vlim[rpm]/60*2*pi)
      // mpy by 1e6 for accuracy
      if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC)
      {
          s32_delta_v_thershold = 357914; // 357914 = 20rpm
          s32_temp_thershold = 894784L; // 894784 = 50 rpm (ptpvcmd)
      }
      else
      {
          s32_delta_v_thershold = 178957; // 178957 = 20rpm
          s32_temp_thershold = 447392; // 447392 = 50 rpm (ptpvcmd)
      }
      s16_temp = 0;
      if ((BGVAR(s16_Pos_Tune_Cycle) < 2) && (VAR(AX0_s16_StandStill_Counter) == 0)) s16_temp = 1;
      else if( ((BGVAR(s16_Pos_Tune_Cycle) >= 2) && (s32_temp > 89479) && (VAR(AX0_u16_Pos_Loop_Tsp) != TSP_250_USEC))  ||  // 89479 = 10rpm
               ((BGVAR(s16_Pos_Tune_Cycle) >= 2) && (s32_temp > 178957) && (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC))    )s16_temp = 1;  // 178957 = 10rpm

      if ((s32_temp  <= ((3*BGVAR(s32_Capture_Ptpvcmd)) >> 2)) || (BGVAR(s32_Capture_Ptpvcmd) == 0))
         s32_delta_v = 0x7FFFFFFF;
      else
         s32_delta_v = labs(s32_temp - BGVAR(s32_Pos_Vcmd_Prev));

      if ((s64_temp > 0LL)                          && // far from cycle start\stop pos
          (s16_temp == 1)                                                     && // not standing
          (BGVAR(s16_Cycle_StandStill_Found) == 4)                            && // cycle ident fully operetional
          ((s32_temp <=  BGVAR(s32_Pos_Vcmd_Prev)) || (s32_delta_v < s32_delta_v_thershold)) &&
          (s32_temp > s32_temp_thershold)                                                &&
          ((VAR(AX0_AutoTune_Bits) & AT_ACC_PARTIAL_CYCLE_MASK) == 0)         &&
          (BGVAR(s32_Cycle_Fric_Coeff)>=0)                                    &&
          (BGVAR(s16_HdTune_Igrav_En) == 0)                                      )
      {
         if (BGVAR(s32_Capture_Ptpvcmd) == 0) BGVAR(s32_Capture_Ptpvcmd) = s32_temp;
         VAR(AX0_AutoTune_Bits) |= AT_ACC_PARTIAL_CYCLE_MASK;
         if (BGVAR(u16_At_Pretubation_En) == 2)
         {
            if (VAR(AX0_s16_Igrav) == 0)
            {
               u32_temp = (BGVAR(s32_Motor_I_Cont) * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
                        >> (long)BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
               if (u32_temp > VAR(AX0_s16_I_Sat_Hi)) u32_temp =  VAR(AX0_s16_I_Sat_Hi);

               VAR(AX0_s16_Igrav) = (int)u32_temp;
            }
            else
               VAR(AX0_s16_Igrav) =-VAR(AX0_s16_Igrav);

            BGVAR(s16_Cycle_LMJR_Counter) = 0;
            CalculateLMJR(0,0,0,0); // reset static vars
         }
         else
         {
            f_temp = CalculateLMJR(s64_temp3,s64_temp2,u32_temp,s16_vel-BGVAR(s16_At_Vel_T0));
            if (f_temp >= 0) // valid data is avaliable
            {
               ++BGVAR(s16_Cycle_LMJR_Counter);
               if (BGVAR(s16_Cycle_LMJR_Counter) > CYCLE_LMJR_LIM) BGVAR(s16_Cycle_LMJR_Counter) = CYCLE_LMJR_LIM;
               if (BGVAR(s16_Pos_Tune_Traj_Mode_User) != PTT_IDLE) BGVAR(s32_Cycle_LMJR)  = (long)f_temp; // when in ext traj - use simplex
            }
         }
      }
   }
   BGVAR(s32_Pos_Vcmd_Prev) = s32_temp;
}

#pragma CODE_SECTION(CalculateLMJR, "ramfunc_2");
float CalculateLMJR(long long s64_captured_integral,long long s64_motion_length,unsigned long u32_integral_counter,int s16_captured_vel)
{

   float f_temp;
   long s32_temp;
   static unsigned int u16_calc_lmjr_counter = 0;
   static long s32_calc_lmjr_vel = 0;
   static long long s64_calc_lmjr_integral = 0;
   static long long s64_calc_lmjr_motion = 0;
   static unsigned long long u64_calc_lmjr_integral_counter = 0;

   if ((s64_captured_integral == 0) && (s64_motion_length == 0) && (u32_integral_counter == 0) && (s16_captured_vel == 0))
   {
      u16_calc_lmjr_counter = 0;
      s64_calc_lmjr_integral  = 0;
      u64_calc_lmjr_integral_counter  = 0;
      s32_calc_lmjr_vel = 0;
      s64_calc_lmjr_motion = 0;
      BGVAR(s32_Capture_Ptpvcmd) = 0;
      BGVAR(f_Avg_Acc) = 0;
      AX0_AutoTune_Bits |= AT_ACC_PARTIAL_CYCLE_MASK;
      return -2;
   }

   // verify integral data is enough - avg current should be above 0.3 micont
   // 3.3333 * 4 / 26214 = 5.09e-4
   // change limit to 0.5
   // 2 * 4 / 26214 = 3.052e-4
   // change limit to 0.5
   // 1 * 4 / 26214 = 1.526e-4

   // calc acc
   // Use ~100 hz LP Filter (alpha = 0.6)
   BGVAR(f_Avg_Acc) += 0.6*((float)abs(s16_captured_vel)/(float)u32_integral_counter - BGVAR(f_Avg_Acc));

   ++u16_calc_lmjr_counter;
   s64_calc_lmjr_integral += s64_captured_integral;
   u64_calc_lmjr_integral_counter += (unsigned long long)u32_integral_counter;
   s32_calc_lmjr_vel += (long)s16_captured_vel;
   s64_calc_lmjr_motion += s64_motion_length;

   s32_temp = labs((long)(1.526e-4*(float)BGVAR(s32_Drive_I_Peak)*(float)s64_calc_lmjr_integral/(float)u32_integral_counter)); // integral value is divided by 4 in RT
   if ((s32_temp < BGVAR(s32_Motor_I_Cont)) || (u64_calc_lmjr_integral_counter < 1500)) return -1;

   f_temp = 4.76844e-15*(float)BGVAR(u32_Motor_Kt)*(float)s64_calc_lmjr_integral * (float)BGVAR(s32_Drive_I_Peak)/(float)u16_calc_lmjr_counter;
   if (BGVAR(u32_Pwm_Freq) == 8000L) f_temp = f_temp*2; // pwm 8 tsc is 62.5e-6 and not 31.25e-6
   else if (BGVAR(u32_Pwm_Freq) == 4000L) f_temp = f_temp*4; // pwm 4 tsc is 125e-6 and not 31.25e-6

   f_temp = f_temp - 1e-6*(float)BGVAR(s32_Cycle_T_Unbalance)*(float)BGVAR(u32_Cycle_OverAll_Counter);
   // 1.462918e-9 is to conver back from rad to internal units 2pi/2^32
   // 1e-8 is restore original B (multiplied by 1e8 to support floating point number)
   f_temp = f_temp-1.462918e-9*(float)BGVAR(s32_Cycle_Fric_Coeff)*1e-8*(float)s64_calc_lmjr_motion/(float)u16_calc_lmjr_counter;
   // convert internal velocity to rad/s/ (pi*0.00011176*s32_V_Lim)/(30*22750)=5.1444*e-10
   f_temp = f_temp*(float)u16_calc_lmjr_counter/((float)s32_calc_lmjr_vel*5.1444e-10*(float)BGVAR(s32_V_Lim_Design));
   f_temp = 1e9*f_temp/(float)BGVAR(s32_Motor_J)-1000;
   if (f_temp < 0) f_temp = 0;
   if (f_temp > 50000) f_temp = 50000;

   u16_calc_lmjr_counter = 0;
   s64_calc_lmjr_integral  = 0;
   u64_calc_lmjr_integral_counter  = 0;
   s32_calc_lmjr_vel = 0;
   s64_calc_lmjr_motion = 0;

   return (f_temp);
}

int AutoTuneAntiVibWithPSD(int drive,int s16_a_vib_type, int reset)
{  // 1 - finished 0 - working
   static int s16_av_psd = 0;
   static long s32_timer;
   int s16_bin_hz1,s16_bin_hz2;
   unsigned long rectrig;
   long code1,code2,code3,s32_freq_final1,s32_freq_final2,s32_freq_in_use;
   float f_temp;
   // AXIS_OFF;

   if (reset == 1)
   {
      s16_av_psd = 0;
      return 1;   
   }

   switch (s16_av_psd)
   {
      case 0: // setup record
         BGVAR(s16_FFT_State) = FFT_STATE_IDLE;
         STORE_EXECUTION_PARAMS_0_15
         ClrRecord();
         ConvertMnemonic2Code("PE", &code1, &code2, &code3, 0);
         s64_Execution_Parameter[2] = SearchMnemonic(code1, code2, code3)-1;
         ConvertMnemonic2Code("ICMD", &code1, &code2, &code3, 0);
         s64_Execution_Parameter[3] = SearchMnemonic(code1, code2, code3)-1;
         s64_Execution_Parameter[4] = 0;
         SalRecordCommand(drive,40LL, 400LL,&s64_Execution_Parameter[2]); // detect upto 400hz , 1 Hz resolution
         RESTORE_EXECUTION_PARAMS_0_15
         s32_timer = Cntr_1mS;
         VAR(AX0_s16_Cycle_Bits) &= ~CYCLE_NEW_CYCLE_DATA_READY;
         s16_av_psd = 1;
         break;

      case 1: // arm trigger
         if ((VAR(AX0_s16_Cycle_Bits) & CYCLE_NEW_CYCLE_DATA_READY) == 0) return 0;
         VAR(AX0_s16_Cycle_Bits) &= ~CYCLE_NEW_CYCLE_DATA_READY;
         if (!PassedTimeMS(500L, s32_timer)) return 0;

         // trigger when max instability
         rectrig = ((unsigned long)(&u16_AntiVib_Osc_Trig) & 0xFFFFFF) | REC_ADD_16;
         SalRecTrigCommand(drive,rectrig,5*1000L,0,1);
         s16_av_psd = 2;
         break;

      case 2:
         if ((s16_Record_Flags & DATA_AVAILABLE_MASK) == 0) break;
         // data is avaliable - invoke psd analysis on PE
         STORE_EXECUTION_PARAMS_0_15
         s64_Execution_Parameter[0] = 2;
         RecPSDCommand(drive);
         RESTORE_EXECUTION_PARAMS_0_15
         s16_av_psd = 3;
         break;

      case 3:
         if (BGVAR(s16_FFT_State) != FFT_STATE_DONE) break; // wait till fft is done

         f_temp = (float)BGVAR(s32_Psd_Max_Val) / (float)BGVAR(s64_Psd_Avg);
         s16_bin_hz1 = (int)((float)BGVAR(s16_Psd_Max_Bin) / ((float)s32_Gap_Value*31.25e-6*(float)s16_Recording_Length) + 0.5);

         if ((s16_bin_hz1 < 10)  ||
             (s16_bin_hz1 > 400) ||
             (f_temp < 2)                    )
            s32_freq_final1 = 400000; // invalid value
         else
            s32_freq_final1 = 1000L*(long)s16_bin_hz1;

         f_temp = (float)BGVAR(s32_Psd_Max2_Val) / (float)BGVAR(s64_Psd_Avg);
         s16_bin_hz2 = (int)((float)BGVAR(s16_Psd_Max2_Bin) / ((float)s32_Gap_Value*31.25e-6*(float)s16_Recording_Length) + 0.5);

         if ((s16_bin_hz2 < 10)  ||
             (s16_bin_hz2 > 400) ||
             (f_temp < 2)                    )
            s32_freq_final2 = 400000; // invalid value
         else
            s32_freq_final2 = 1000L*(long)s16_bin_hz2;


          if ((s32_freq_final2 < 400000) && (s32_freq_final1 == 400000)) s32_freq_in_use = s32_freq_final2;
          else if ((s32_freq_final2 == 400000) && (s32_freq_final1 < 400000)) s32_freq_in_use = s32_freq_final1;
          else if ((s32_freq_final2 == 400000) && (s32_freq_final1 == 400000)) s32_freq_in_use = 400000;
          else if (s32_freq_final2 < s32_freq_final1) s32_freq_in_use = s32_freq_final1;
          else if (s32_freq_final1 < s32_freq_final2) s32_freq_in_use = s32_freq_final2;
          else s32_freq_in_use = s32_freq_final1; // both values are the same 

         if (s32_freq_in_use > 400000) s32_freq_in_use = 400000; 

         if (s16_a_vib_type == 1)
            SalNlKAntiResonanceFcenterCommand((long long)s32_freq_final1,drive);
         else if (s16_a_vib_type == 2)
         {
            if (BGVAR(s16_Pos_tune_Mode) == 90)
            {
               SalNlPeFcenterCommand((long long)s32_freq_final1,drive);
               SalNlPe3FcenterCommand((long long)s32_freq_final2,drive);
            }
            else
               SalNlPeFcenterCommand((long long)s32_freq_in_use,drive);
         }
         else if (s16_a_vib_type == 3)
         {
            if (BGVAR(s16_Pos_tune_Mode) == 90)
            {
               SalNlPe3FcenterCommand((long long)s32_freq_final1,drive);
               SalNlPeFcenterCommand((long long)s32_freq_final2,drive);
            }
            else
               SalNlPe3FcenterCommand((long long)s32_freq_in_use,drive);
         }

         if (BGVAR(s16_Pos_tune_Mode) != 90) ClrRecord();// AVHZ test mode
         s16_av_psd = 0;
         return 1;
         //break;
   }
   return 0;
}




int AutoTuneGainNormalization(int drive)
{
   float f_temp =  (float)BGVAR(u32_Nl_Kpgf)/1000;
   unsigned long u32_kpiv,u32_kpi,u32_kpp,u32_kpd,u32_movesmoothavg,/*u32_temp,*/u32_kpd_lim = 400000;
   int s16_movesmoothlpfhz;
   //int s16_tsp_factor = (int)((float)VAR(AX0_u16_Pos_Loop_Tsp)*0.01);
   // AXIS_OFF;

   if  ((BGVAR(u16_HdTune_Adv_Mode) == 0) || (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5))
   {
       f_temp =  (float)BGVAR(u32_Nl_Kpgf_User)/1000;
       u32_kpd_lim = 1750000;
   }

   u32_kpiv = (unsigned long)((float)BGVAR(u32_Nl_Kpiv_User) * f_temp);
   u32_kpi  = (unsigned long)((float)BGVAR(u32_Nl_Kpi_User)  * f_temp);
   u32_kpp  = (unsigned long)((float)BGVAR(u32_Nl_Kpp_User) * f_temp);
   u32_kpd  = (unsigned long)((float)BGVAR(u32_Nl_Kpd_User) * f_temp);
   s16_movesmoothlpfhz = (int)((float)u32_kpd*0.001);

   if ( (u32_kpiv <= 400000)     &&
        (u32_kpi <= 200000)      &&
        (u32_kpp <= 400000)      &&
        (u32_kpd <= u32_kpd_lim)   )
   {
      BGVAR(s32_At_Usergain_Max) = (long)((float)BGVAR(s32_At_Usergain_Max)*(float)BGVAR(u32_Nl_Kpd_User)/(float)u32_kpd);
      BGVAR(s32_At_Usergain_Min) = (long)((float)BGVAR(s32_At_Usergain_Min)*(float)BGVAR(u32_Nl_Kpd_User)/(float)u32_kpd);

      BGVAR(u32_Nl_Kpgf_User) = 1000;
      BGVAR(u32_Nl_Kpiv_User) = u32_kpiv;
      BGVAR(u32_Nl_Kpi_User)  = u32_kpi;
      BGVAR(u32_Nl_Kpp_User)  = u32_kpp;
      BGVAR(u32_Nl_Kpd_User)  = u32_kpd;
      PositionConfig(drive,0);
      while ((VAR(AX0_s16_Crrnt_Run_Code) & (COPY_POS_COEF_MASK|COPY_NCT_COEF_MASK)) != 0);
      if (s16_movesmoothlpfhz < 25) s16_movesmoothlpfhz = 25;
      if (BGVAR(s16_Pos_Tune_Stiffness) == 0)
      {
         SalMoveSmoothLpfCommand((long long)s16_movesmoothlpfhz,drive);
         u32_movesmoothavg = 250*(long)(sqrt(0.196196+(250e-6*(float)s16_movesmoothlpfhz)*(250e-6*(float)s16_movesmoothlpfhz))/((float)s16_movesmoothlpfhz*250e-6));
         SalMoveSmoothAvgNumCommand((long long)u32_movesmoothavg,drive);

         //gearfiltt1=1000/(2.movesmoothlpfhz) (ms); gearfiltt2 * 1.5*gearfiltt1
         /*u32_temp = (unsigned long)(GEARFILT_FACTOR/(float)s16_movesmoothlpfhz);
         u32_temp = u32_temp / s16_tsp_factor;
         u32_temp = u32_temp*s16_tsp_factor;
         if(u32_temp < 750L) u32_temp = 750L; // Limit GEARFILTT1 to 0.75 mS
         BGVAR(u32_Gear_Filt_User_T1) = u32_temp; // do not use the sat command so as not to desingmsq more times - only takes longer
         u32_temp = (3*u32_temp) >> 1;
         u32_temp = u32_temp / s16_tsp_factor;
         u32_temp = u32_temp*s16_tsp_factor;
         SalGearFiltT2Command((long long)u32_temp,drive);*/

      }
      return 1;
   }
   return 0;
}

void AutoTuneLMJRNormalization(int drive,int s16_mode)
{
   //int s16_tsp_factor = (int)((float)VAR(AX0_u16_Pos_Loop_Tsp)*0.01);
   float f_factor,f_sqrt_factor;
   unsigned long u32_kpiv,u32_kpi,u32_kpp,u32_kpd,u32_movesmoothavg/*,u32_temp*/;
   int s16_movesmoothlpfhz;
   long s32_lmjr = (long)(1.25 *(float)BGVAR(s32_Cycle_LMJR));
   // AXIS_OFF;

   if (BGVAR(s16_Pos_Tune_LMJR) != 0) s32_lmjr = (long)(1.25 *(float)BGVAR(u32_AT_LMJR));

   do{
      s32_lmjr = (long)(0.8*(float)s32_lmjr);
      f_factor = (float)(BGVAR(u32_LMJR_User)+1000)/(float)(s32_lmjr+1000);
      f_sqrt_factor =  sqrt(f_factor);
      u32_kpiv = (unsigned long)((float)BGVAR(u32_Nl_Kpiv) * f_sqrt_factor);
      u32_kpi = (unsigned long)((float)BGVAR(u32_Nl_Kpi) * pow(f_factor,0.33333333333333));
      u32_kpp = (unsigned long)((float)BGVAR(u32_Nl_Kpp) * f_sqrt_factor);
      u32_kpd = (unsigned long)((float)BGVAR(u32_Nl_Kpd) * f_factor);
      s16_movesmoothlpfhz = (int)((float)u32_kpd*0.001);
     } while (((u32_kpiv > 400000) || (u32_kpi > 200000) || (u32_kpp > 400000) || (u32_kpd > 800000))  && (s32_lmjr != 0));
   if  ( (s32_lmjr == 0) && (s16_mode == 0)                                                    &&
         ((u32_kpiv > 400000) || (u32_kpi > 200000) || (u32_kpp > 400000) || (u32_kpd > 800000))  ) KnlusergainToLMJRConversion(drive);
   else
   {
      BGVAR(s32_At_Usergain_Max) = (long)((float)BGVAR(s32_At_Usergain_Max)*(float)BGVAR(u32_Nl_Kpd_User)/(float)u32_kpd*(1000 + (float)BGVAR(u32_LMJR_User))/(1000+(float)s32_lmjr));
      BGVAR(s32_At_Usergain_Min) = (long)((float)BGVAR(s32_At_Usergain_Min)*(float)BGVAR(u32_Nl_Kpd_User)/(float)u32_kpd*(1000 + (float)BGVAR(u32_LMJR_User))/(1000+(float)s32_lmjr));
      BGVAR(u32_Nl_Kpiv_User) = u32_kpiv;
      BGVAR(u32_Nl_Kpi_User)  = u32_kpi;
      BGVAR(u32_Nl_Kpp_User)  = u32_kpp;
      BGVAR(u32_Nl_Kpd_User)  = u32_kpd;
      BGVAR(u32_LMJR_User) = (unsigned long)s32_lmjr;
      PositionConfig(drive,0);
      while ((VAR(AX0_s16_Crrnt_Run_Code) & (COPY_POS_COEF_MASK|COPY_NCT_COEF_MASK)) != 0);
       if (s16_movesmoothlpfhz < 25) s16_movesmoothlpfhz = 25;
      if ( (BGVAR(s16_Pos_Tune_Stiffness) == 0) && (s16_mode == 0) )
      {
         SalMoveSmoothLpfCommand((long long)s16_movesmoothlpfhz,drive);
         u32_movesmoothavg = 250*(long)(sqrt(0.196196+(float)s16_movesmoothlpfhz*250e-6*(float)s16_movesmoothlpfhz*250e-6)/((float)s16_movesmoothlpfhz*250e-6));
         SalMoveSmoothAvgNumCommand((long long)u32_movesmoothavg,drive);

         //gearfiltt1=1000/(2.movesmoothlpfhz) (ms); gearfiltt2 * 1.5*gearfiltt1
         /*u32_temp = (unsigned long)(GEARFILT_FACTOR/(float)s16_movesmoothlpfhz);
         u32_temp = u32_temp / s16_tsp_factor;
         u32_temp = u32_temp*s16_tsp_factor;
         if(u32_temp < 750L) u32_temp = 750L; // Limit GEARFILTT1 to 0.75 mS
         BGVAR(u32_Gear_Filt_User_T1) = u32_temp; // do not use the sat command so as not to desingmsq more times - only takes longer
         u32_temp = (3*u32_temp) >> 1;
         u32_temp = u32_temp / s16_tsp_factor;
         u32_temp = u32_temp*s16_tsp_factor;
         SalGearFiltT2Command((long long)u32_temp,drive);*/
      }
   }
}

void KnlusergainToLMJRConversion(int drive)
{
   // this function handels knlusergain sat by increasing lmjr and calulating the gains
   // so that all internal values will not change
   //// AXIS_OFF;
   //int s16_tsp_factor = (int)((float)VAR(AX0_u16_Pos_Loop_Tsp)*0.01);
   float f_factor,f_sqrt_factor,f_temp =  (float)BGVAR(u32_Nl_Kpgf)/1000;
   unsigned long u32_kpiv,u32_kpi,u32_kpp,u32_kpd,u32_movesmoothavg/*,u32_temp*/;
   int s16_movesmoothlpfhz;
   long s32_lmjr,s32_lmjr_origin;

   if (AutoTuneGainNormalization(drive) == 1) return; // if normalization was ok that quit

   u32_kpiv = (unsigned long)((float)BGVAR(u32_Nl_Kpiv_User) * f_temp);
   u32_kpi  = (unsigned long)((float)BGVAR(u32_Nl_Kpi_User)  * f_temp);
   u32_kpp  = (unsigned long)((float)BGVAR(u32_Nl_Kpp_User) * f_temp);
   u32_kpd  = (unsigned long)((float)BGVAR(u32_Nl_Kpd_User) * f_temp);
   s16_movesmoothlpfhz = (int)((float)u32_kpd*0.001);

   s32_lmjr = BGVAR(u32_LMJR_User);
   s32_lmjr_origin = BGVAR(u32_LMJR_User);

   while (((u32_kpiv > 400000) || (u32_kpi > 200000) || (u32_kpp > 400000) || (u32_kpd > 800000)) && (s32_lmjr < 1000000))
   {
      s32_lmjr_origin = s32_lmjr;
      s32_lmjr = (long)(1.25*(float)s32_lmjr_origin);
      if (s32_lmjr == 0) s32_lmjr = 100;
      f_factor = (float)(s32_lmjr_origin+1000)/(float)(s32_lmjr+1000);
      f_sqrt_factor =  sqrt(f_factor);

      u32_kpiv = (unsigned long)((float)u32_kpiv * f_sqrt_factor);
      u32_kpi  = (unsigned long)((float)u32_kpi * pow(f_factor,0.33333333333333));
      u32_kpp  = (unsigned long)((float)u32_kpp * f_sqrt_factor);
      u32_kpd  = (unsigned long)((float)u32_kpd * f_factor);
      s16_movesmoothlpfhz = (int)((float)u32_kpd*0.001);
   }

   BGVAR(s32_At_Usergain_Max) = (long)((float)BGVAR(s32_At_Usergain_Max)*(float)BGVAR(u32_Nl_Kpd_User)/(float)u32_kpd*(1000 + (float)BGVAR(u32_LMJR_User))/(1000+(float)s32_lmjr));
   BGVAR(s32_At_Usergain_Min) = (long)((float)BGVAR(s32_At_Usergain_Min)*(float)BGVAR(u32_Nl_Kpd_User)/(float)u32_kpd*(1000 + (float)BGVAR(u32_LMJR_User))/(1000+(float)s32_lmjr));


   BGVAR(u32_Nl_Kpgf_User) = 1000;
   BGVAR(u32_Nl_Kpiv_User) = u32_kpiv;
   BGVAR(u32_Nl_Kpi_User)  = u32_kpi;
   BGVAR(u32_Nl_Kpp_User)  = u32_kpp;
   BGVAR(u32_Nl_Kpd_User)  = u32_kpd;
   BGVAR(u32_LMJR_User) = (unsigned long)s32_lmjr;
   PositionConfig(drive,0);
    if (s16_movesmoothlpfhz < 25) s16_movesmoothlpfhz = 25;
   if (BGVAR(s16_Pos_Tune_Stiffness) == 0)
   {
      SalMoveSmoothLpfCommand((long long)s16_movesmoothlpfhz,drive);
      u32_movesmoothavg = 250*(long)(sqrt(0.196196+(float)s16_movesmoothlpfhz*250e-6*(float)s16_movesmoothlpfhz*250e-6)/((float)s16_movesmoothlpfhz*250e-6));
      SalMoveSmoothAvgNumCommand((long long)u32_movesmoothavg,drive);

      //gearfiltt1=1000/(2.movesmoothlpfhz) (ms); gearfiltt2 * 1.5*gearfiltt1
      /*u32_temp = (unsigned long)(GEARFILT_FACTOR/(float)s16_movesmoothlpfhz);
      u32_temp = u32_temp / s16_tsp_factor;
      u32_temp = u32_temp*s16_tsp_factor;
      if(u32_temp < 750L) u32_temp = 750L; // Limit GEARFILTT1 to 0.75 mS
      BGVAR(u32_Gear_Filt_User_T1) = u32_temp; // do not use the sat command so as not to desingmsq more times - only takes longer
      u32_temp = (3*u32_temp) >> 1;
      u32_temp = u32_temp / s16_tsp_factor;
      u32_temp = u32_temp*s16_tsp_factor;
      SalGearFiltT2Command((long long)u32_temp,drive);*/
   }
}

/*int SalReadKnldGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u32_Nl_Kpd_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalKnldGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 2000000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u32_Nl_Kpd_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadKnliGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u32_Nl_Kpi_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalKnliGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 200000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u32_Nl_Kpi_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadKnlivGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u32_Nl_Kpiv_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalKnlivGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 400000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u32_Nl_Kpiv_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadKnlpGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u32_Nl_Kpp_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalKnlpGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 400000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u32_Nl_Kpp_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadKnlUserGainGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u32_Nl_Kpgf_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalKnlUserGainGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 3000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u32_Nl_Kpgf_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadFiltt1GTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(s16_Nl_Out_Filter_1_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalFiltt1GTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 30000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(s16_Nl_Out_Filter_1_GT)[s64_Execution_Parameter[0]-1] = (int)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadFiltDampingGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintUnsignedLong(BGVAR(s16_Nl_Out_Filter_2_GT)[s64_Execution_Parameter[0]-1]);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalFiltDampingGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 100)) return (VALUE_OUT_OF_RANGE);

   BGVAR(s16_Nl_Out_Filter_2_GT)[s64_Execution_Parameter[0]-1] = (int)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadAVibGain2GTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(s32_Pe_Filt_Gain_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalAVibGain2GTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 10000000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(s32_Pe_Filt_Gain_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/


/*int SalReadAVibGainGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u32_Nl_K_Anti_Vibration_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalAVibGainGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 10000000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u32_Nl_K_Anti_Vibration_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/


/*int SalReadAVibHz2GTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u32_Pe_Filt_Fcenter_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalAVibHz2GTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 5000) || (s64_Execution_Parameter[1] > 400000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u32_Pe_Filt_Fcenter_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadAVibHzGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u32_Nl_K_Anti_Resonance_Fcenter_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalAVibHzGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 5000) || (s64_Execution_Parameter[1] > 400000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u32_Nl_K_Anti_Resonance_Fcenter_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadAVibSharp2GTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u16_Pe_Filt_Sharpness_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalAVibSharp2GTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 10) || (s64_Execution_Parameter[1] > 10000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u16_Pe_Filt_Sharpness_GT)[s64_Execution_Parameter[0]-1] = (unsigned int)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadAVibSharpGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u16_Nl_K_Anti_Resonance_Sharpness_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalAVibSharpGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 10) || (s64_Execution_Parameter[1] > 10000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u16_Nl_K_Anti_Resonance_Sharpness_GT)[s64_Execution_Parameter[0]-1] = (unsigned int)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadLmjrGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u32_LMJR_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalLmjrGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 600000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u32_LMJR_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadAffLpfHzGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintSignedInteger(BGVAR(s16_Kbff_Spring_LPF_GT)[s64_Execution_Parameter[0]-1]);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalAffLpfHzGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 7000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(s16_Kbff_Spring_LPF_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalReadNlPeAffGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // verify index is valid
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   PrintString(DecimalPoint32ToAscii((long)BGVAR(u32_Nl_KffSpring_GT)[s64_Execution_Parameter[0]-1]), 0);
   PrintCrLf();
   return (SAL_SUCCESS);
}*/

/*int SalNlPeAffGTCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > NUM_OF_GT)) return (VALUE_OUT_OF_RANGE);
   if ((s64_Execution_Parameter[1] < 0) || (s64_Execution_Parameter[1] > 200000000)) return (VALUE_OUT_OF_RANGE);

   BGVAR(u32_Nl_KffSpring_GT)[s64_Execution_Parameter[0]-1] = (unsigned long)s64_Execution_Parameter[1];
   return (SAL_SUCCESS);
}*/

/*int SalKnlGTModeCommand(long long param,int drive)
{
   ++param;++drive;
   return (SAL_SUCCESS);
}*/


void KnlGTBackup(int drive,int s16_index)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   s16_index++;
   /*BGVAR(u32_Nl_Kpd_GT)[s16_index] = BGVAR(u32_Nl_Kpd);
   BGVAR(u32_Nl_Kpi_GT)[s16_index] = BGVAR(u32_Nl_Kpi);
   BGVAR(u32_Nl_Kpiv_GT)[s16_index] = BGVAR(u32_Nl_Kpiv);
   BGVAR(u32_Nl_Kpp_GT)[s16_index] = BGVAR(u32_Nl_Kpp);
   BGVAR(u32_Nl_Kpgf_GT)[s16_index] = BGVAR(u32_Nl_Kpgf);
   BGVAR(s16_Nl_Out_Filter_1_GT)[s16_index] = BGVAR(s16_Nl_Out_Filter_1);
   BGVAR(s16_Nl_Out_Filter_2_GT)[s16_index] = BGVAR(s16_Nl_Out_Filter_2);
   BGVAR(u32_Nl_K_Anti_Vibration_GT)[s16_index] = BGVAR(u32_Nl_K_Anti_Vibration);
   BGVAR(s32_Pe_Filt_Gain_GT)[s16_index] = BGVAR(s32_Pe_Filt_Gain);
   BGVAR(u32_Nl_K_Anti_Resonance_Fcenter_GT)[s16_index] = BGVAR(u32_Nl_K_Anti_Resonance_Fcenter);
   BGVAR(u32_Pe_Filt_Fcenter_GT)[s16_index] = BGVAR(u32_Pe_Filt_Fcenter);
   BGVAR(u16_Nl_K_Anti_Resonance_Sharpness_GT)[s16_index] = BGVAR(u16_Nl_K_Anti_Resonance_Sharpness);
   BGVAR(u16_Pe_Filt_Sharpness_GT)[s16_index] = BGVAR(u16_Pe_Filt_Sharpness);
   BGVAR(u32_LMJR_GT)[s16_index] = BGVAR(u32_LMJR);
   BGVAR(s16_Kbff_Spring_LPF_GT)[s16_index] = BGVAR(s16_Kbff_Spring_LPF);
   BGVAR(u32_Nl_KffSpring_GT)[s16_index] = BGVAR(u32_Nl_KffSpring);*/
}


void KnlGTRestore(int drive,int s16_index)
{
   REFERENCE_TO_DRIVE;
   s16_index++;
  // // AXIS_OFF;
   /*BGVAR(u32_Nl_Kpd) = BGVAR(u32_Nl_Kpd_GT)[s16_index];
   BGVAR(u32_Nl_Kpi) = BGVAR(u32_Nl_Kpi_GT)[s16_index];
   BGVAR(u32_Nl_Kpiv) = BGVAR(u32_Nl_Kpiv_GT)[s16_index];
   BGVAR(u32_Nl_Kpp) = BGVAR(u32_Nl_Kpp_GT)[s16_index];
   BGVAR(u32_Nl_Kpgf) = BGVAR(u32_Nl_Kpgf_GT)[s16_index];
   BGVAR(s16_Nl_Out_Filter_1) = BGVAR(s16_Nl_Out_Filter_1_GT)[s16_index];
   BGVAR(s16_Nl_Out_Filter_2) = BGVAR(s16_Nl_Out_Filter_2_GT)[s16_index];
   BGVAR(u32_Nl_K_Anti_Vibration) = BGVAR(u32_Nl_K_Anti_Vibration_GT)[s16_index];
   BGVAR(s32_Pe_Filt_Gain) = BGVAR(s32_Pe_Filt_Gain_GT)[s16_index];
   BGVAR(u32_Nl_K_Anti_Resonance_Fcenter) = BGVAR(u32_Nl_K_Anti_Resonance_Fcenter_GT)[s16_index];
   BGVAR(u32_Pe_Filt_Fcenter) = BGVAR(u32_Pe_Filt_Fcenter_GT)[s16_index];
   BGVAR(u16_Nl_K_Anti_Resonance_Sharpness) = BGVAR(u16_Nl_K_Anti_Resonance_Sharpness_GT)[s16_index];
   BGVAR(u16_Pe_Filt_Sharpness) = BGVAR(u16_Pe_Filt_Sharpness_GT)[s16_index];
   BGVAR(u32_LMJR) = BGVAR(u32_LMJR_GT)[s16_index];
   BGVAR(s16_Kbff_Spring_LPF) = BGVAR(s16_Kbff_Spring_LPF_GT)[s16_index];
   BGVAR(u32_Nl_KffSpring) = BGVAR(u32_Nl_KffSpring_GT)[s16_index];
   PositionConfig(drive,1);
   // Set position coef bit to indicate RT to start coef copying
   VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);*/

}

/*
int SalKnlGTDataCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_KnlGT_Mode) == 0)
   {
      PrintString("Inctive",0);
      PrintCrLf();
   }
   else
   {
      PrintSignedLong(BGVAR(s32_KnlGT_Factor));
      PrintChar(SPACE);
      if (BGVAR(s16_KnlGT_Index) == 0)
         PrintString("1$1",0);
      else
      {
         PrintSignedLong((long)BGVAR(s16_KnlGT_Index));
         PrintChar('$');
         PrintSignedLong((long)BGVAR(s16_KnlGT_Index)+1);
      }
      PrintChar(SPACE);
      PrintString("LMJRST ",0);
      PrintSignedLong((long)(100.0*((float)BGVAR(u16_J_Ident_Counter)/J_IDENT_READY_TRESH)));
      PrintString(" %",0);
      PrintCrLf();
   }
   return (SAL_SUCCESS);
}*/

/*int SalKnlGtVelLimCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (param < 0) return (VALUE_TOO_LOW);
   if (param > (long long)BGVAR(s32_V_Lim_Design)) return (VALUE_TOO_HIGH);
   BGVAR(s32_Knl_Gt_Vel_Lim) = (long)param;
   return (SAL_SUCCESS);
}*/

void GainInterpolationByLMJR(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   BGVAR(u64_Sys_Warnings) &= ~(GT_LMJR_LO_WRN_MASK|GT_LMJR_HI_WRN_MASK|GT_INVALID_ORDER_WRN_MASK);
   //BGVAR(s32_KnlGT_Factor) = -2;
   //BGVAR(s16_KnlGT_Index) = -1;
   return;
}

int PostuneAntiVibHandler(int drive,int s16_mode)
{
   int s16_temp,s16_temp_avmode = BGVAR(s16_HdTune_Av_Mode);
   unsigned long u32_temp;
   static unsigned long u32_effort_before_av;
   // AXIS_OFF;

   // 0-ongoing 1-done -1-no freq found
   if (s16_temp_avmode > 3) s16_temp_avmode-=3;
   switch (BGVAR(s16_Pos_Tune_Av_State))
   {
      case POS_TUNE_AV_IDLE:
         if ( (BGVAR(s16_HdTune_Av_Mode) <= 0)                  ||
              ((BGVAR(s32_Pos_Tune_Bits) & COST_0_MASK) != 0)   ) return 1;

         u32_effort_before_av = LVAR(AX0_u32_At_Control_Effort_Tresh);

         if (s16_mode == ATAVHNDLR_MODE_TUNE_ONLY)
         {  // tune avgain2 and avgain3
            if (BGVAR(s16_HdTune_Av_Mode)  > 3) return 1; // no fine tune when avmode > 3
            if (s16_temp_avmode == 1)
            {
               BGVAR(s16_Pos_Tune_Weight) = 50;//95;
               VAR(AX0_s16_Cycle_Bits) &= ~POS_TUNE_MIN_SETTLE_TIME_SCALE;

               BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_GAIN2_SEARCH;
               PosTuneAvSearch(drive,POS_TUNE_AV_GAIN2,s16_mode);
               TuneSearchInit(drive,0);
            }
            else
            {
               BGVAR(s16_Pos_Tune_Weight) = 50;//95;
               VAR(AX0_s16_Cycle_Bits) &= ~POS_TUNE_MIN_SETTLE_TIME_SCALE;

               BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_GAIN3_SEARCH;
               PosTuneAvSearch(drive,POS_TUNE_AV_GAIN3,s16_mode);
               TuneSearchInit(drive,0);
            }
         }
         else // find frq and all AV values
         {
            if (s16_temp_avmode == 1)
               BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_FRQ2_IDENT;
            else
               BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_FRQ3_IDENT;
         }
         VAR(AX0_AutoTune_Bits) |= AT_AV_ACTIVE_MASK;
         VAR(AX0_AutoTune_Effort_Gain) = 0x3CCC; // when effort exceeded reduce gain by 0.95
         break;

      case POS_TUNE_AV_FRQ2_IDENT:
         if (AutoTuneAntiVibWithPSD(drive,2,0) != 1) break;
         // set effort twice the max effort during freq detection
         // vibrating system can vibrate due to no av. hencee effort will reduce when
         // av is set. do not use the "usual" effort here !
         u32_temp = BGVAR(u32_At_Control_Effort_Max_Captured);
         u32_temp += (u32_temp >> 2);
         if (u32_temp < LVAR(AX0_u32_At_Control_Effort_Tresh)) u32_temp = LVAR(AX0_u32_At_Control_Effort_Tresh);
         if (u32_temp > PercentToEffort(drive,150000)) u32_temp = PercentToEffort(drive,150000); 
         LVAR(AX0_u32_At_Control_Effort_Tresh) =  u32_temp;

         if (BGVAR(u32_Pe_Filt_Fcenter) >  ATAV_FREQ_LIMIT) // no tunable frq was found
         {
            BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_IDLE;
            LVAR(AX0_u32_At_Control_Effort_Tresh) = u32_effort_before_av;
            BGVAR(s16_Pos_Tune_Weight) = BGVAR(s16_Pos_Tune_Weight_User); // recover after av done
            VAR(AX0_AutoTune_Bits) &= ~(AT_AV_ACTIVE_MASK|AT_AV_ACTIVE_2_MASK|AT_AV_ACTIVE_3_MASK);
            VAR(AX0_AutoTune_Effort_Gain) = 0x2000;
            return -1;
         }
         SalNlPeSharpnessCommand((long long)1000*AT_AV_SHARP_FOR_POSTUNE2,drive);
         BGVAR(s16_Pos_Tune_Weight) = 50;//95;
         VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE;
         PosTuneAvSearch(drive,POS_TUNE_AV_GAIN2,s16_mode);
         TuneSearchInit(drive,0);
         BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_GAIN2_INIT;
         break;

      case POS_TUNE_AV_FRQ3_IDENT:
         if (AutoTuneAntiVibWithPSD(drive,3,0) != 1) break;
         // set effort twice the max effort during freq detection
         // vibrating system can vibrate due to no av. hencee effort will reduce when
         // av is set. do not use the "usual" effort here !
         u32_temp = BGVAR(u32_At_Control_Effort_Max_Captured);
         u32_temp += (u32_temp >> 2);
         if (u32_temp < LVAR(AX0_u32_At_Control_Effort_Tresh)) u32_temp = LVAR(AX0_u32_At_Control_Effort_Tresh);
         if (u32_temp > PercentToEffort(drive,150000)) u32_temp = PercentToEffort(drive,150000); 
         LVAR(AX0_u32_At_Control_Effort_Tresh) =  u32_temp;

         if (BGVAR(u32_Antivib3_Fcenter_User) > ATAV_FREQ_LIMIT) // no tunable frq was found
         {
            BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_IDLE;
            LVAR(AX0_u32_At_Control_Effort_Tresh) = u32_effort_before_av;
            BGVAR(s16_Pos_Tune_Weight) = BGVAR(s16_Pos_Tune_Weight_User); // recover after av done
            VAR(AX0_AutoTune_Bits) &= ~(AT_AV_ACTIVE_MASK|AT_AV_ACTIVE_2_MASK|AT_AV_ACTIVE_3_MASK);
            VAR(AX0_AutoTune_Effort_Gain) = 0x2000;
            return -1;
         }
         SalNlPe3SharpnessCommand((long long)1000*AT_AV_SHARP_FOR_POSTUNE3,drive);
         SalNlPe3QGainCommand(1000LL,drive);
         BGVAR(s16_Pos_Tune_Weight) = 50;//95;
         VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE;
         PosTuneAvSearch(drive,POS_TUNE_AV_GAIN3,s16_mode);
         TuneSearchInit(drive,0);
         BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_GAIN3_INIT;
         break;

      case POS_TUNE_AV_GAIN2_INIT:
      case POS_TUNE_AV_SHRP2_INIT:
      case POS_TUNE_AV_GAIN3_INIT:
      case POS_TUNE_AV_SHRP3_INIT:
      case POS_TUNE_AV_Q3_INIT:
            s16_temp = PosTuneAvInitSearch(drive);
            if (s16_temp == 0) break;
            else if (s16_temp == -1)
            {
               TuneSearchInit(drive,0);
               break;
            }

         if (BGVAR(s16_Pos_Tune_Av_State) == POS_TUNE_AV_GAIN2_INIT)
         {
            if (PosTuneReadParam(POS_TUNE_AV_GAIN2,drive,0) == 0) // if gain init value is zero - no av2 needed.
            {
               if (BGVAR(s64_Tune_Min_Cost) == 0x7FFFFFFFFFFFFFFF) // avg init stopped due to effort
                  BGVAR(s32_Pos_Tune_Bits) |= AT_AV_GAIN_INIT_EFFORT_MASK;
               else
                  BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_EFFORT_WARN_AV2;
               BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_DONE;
            }
            else
            {
               s32_Pos_Tune_Av_Shrp_Init_value = PosTuneReadParam(POS_TUNE_AV_SHRP2,drive,0);
               BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_SHRP2_INIT;
               PosTuneAvSearch(drive,POS_TUNE_AV_SHRP2,s16_mode);
               TuneSearchInit(drive,0);
            }
         }
         else if (BGVAR(s16_Pos_Tune_Av_State) == POS_TUNE_AV_SHRP2_INIT)
         {
            BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_DONE;
         }
         else if (BGVAR(s16_Pos_Tune_Av_State) == POS_TUNE_AV_GAIN3_INIT)
         {
            if (PosTuneReadParam(POS_TUNE_AV_GAIN3,drive,0) == 0) // if gain init value is zero - no av3
            {
               if (BGVAR(s64_Tune_Min_Cost) == 0x7FFFFFFFFFFFFFFF) // avg init stopped due to effort
                  BGVAR(s32_Pos_Tune_Bits) |= AT_AV_GAIN_INIT_EFFORT_MASK;
               else
                  BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_EFFORT_WARN_AV3;

               if ( (s16_temp_avmode == 3)               /*  &&
                    (s16_mode == ATAVHNDLR_MODE_TUNE_ONLY)*/  )
                  BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_FRQ2_IDENT;
               else
                  BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_DONE;
            }
            else
            {
               s32_Pos_Tune_Av_Shrp_Init_value = PosTuneReadParam(POS_TUNE_AV_SHRP3,drive,0);
               BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_SHRP3_INIT;
               PosTuneAvSearch(drive,POS_TUNE_AV_SHRP3,s16_mode);
               TuneSearchInit(drive,0);
            }
         }
         else if (BGVAR(s16_Pos_Tune_Av_State) == POS_TUNE_AV_SHRP3_INIT)
         {
            BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_Q3_INIT;
            PosTuneAvSearch(drive,POS_TUNE_AV_Q3,s16_mode);
            TuneSearchInit(drive,0);
         }
         else if (BGVAR(s16_Pos_Tune_Av_State) == POS_TUNE_AV_Q3_INIT)
         {
               if (s16_temp_avmode == 3)
                  BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_FRQ2_IDENT;
               else
                  BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_DONE;
         }
         else
         {
            // if code here than its a sw bug
         }
         break;

      case POS_TUNE_AV_GAIN2_SEARCH:
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,0);
            break;
         }
         BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_DONE;
         break;

      case POS_TUNE_AV_GAIN3_SEARCH:
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,0);
            break;
         }
         // if av3 only than exits
         if (s16_temp_avmode != 3)
         {
            BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_DONE;
            break;
         }
         // start tune for avgain2
         BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_GAIN2_SEARCH;
         PosTuneAvSearch(drive,POS_TUNE_AV_GAIN2,s16_mode);
         TuneSearchInit(drive,0);
         break;

      case POS_TUNE_AV_DONE:
         BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_IDLE;
         LVAR(AX0_u32_At_Control_Effort_Tresh) = u32_effort_before_av;
         BGVAR(s16_Pos_Tune_Weight) = BGVAR(s16_Pos_Tune_Weight_User); // recover after av done
         VAR(AX0_AutoTune_Bits) &= ~(AT_AV_ACTIVE_MASK|AT_AV_ACTIVE_2_MASK|AT_AV_ACTIVE_3_MASK);
         VAR(AX0_AutoTune_Effort_Gain) = 0x2000;
         VAR(AX0_s16_Cycle_Bits) &= ~POS_TUNE_MIN_SETTLE_TIME_SCALE;
         return 1;
         //break;
   }
   return 0;
}

int PosTuneTrajHandler(int drive)
{ // ret value 0 - on going 1- done neg number - error code
  int ret_val = 0;
  int s16_temp = 0;
  long s32_temp = 0L;
  unsigned long long u64_temp = 0LL,u64_max_acc_dec;
  long long s64_pos_tune_auto_traj_pos_lim;
  float f_tsp_factor;
  // AXIS_OFF;

  switch(BGVAR(s16_Pos_Tune_Traj_Handler_State))
  {
     case POSTUNE_TRAJ_HNDLR_IDLE:
        if(BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE)
            BGVAR(s16_Pos_Tune_Traj_Handler_State) = POSTUNE_TRAJ_HNDLR_SET_PARAM;
        else ret_val = (POSTUNE_AUTO_TRAJ_MODE_INACTIVE);
     break;

     case POSTUNE_TRAJ_HNDLR_SET_PARAM:
        // Check if VLIM is larger than 10% of MSPEED to avoid overspeed fault  
        s32_temp = (long)((float)BGVAR(u32_Mspeed)*0.1);
        // Change infinite mode to 1 cycle mode for max acc measurement
        BGVAR(s16_Pos_Tune_Traj_Mode) = PTT_INT_MODE_1_CYCLE;
        // Backup nlmaxgain and nlpeaff
        BGVAR(u32_PTT_Nl_Kpgmax_BackUp) = BGVAR(u32_Nl_Kpgmax);
        BGVAR(u32_PTT_Nl_KffSpring_User_BackUp) = BGVAR(u32_Nl_KffSpring_User);
        BGVAR(u32_Nl_Kpgmax) = 1000L; // Set nlmaxgain = 1;
        BGVAR(u32_Nl_KffSpring_User) = 7000000L; // Set nlpeaff = 7000 Hz
        PositionConfig(drive, 0);

        // Set ACC,DEC,Control,Delay after Path execution, Position, Speed for PATH 1
        // Tsp = 250u
        // ACC[internal] = ACC[RPM/S] * 2^64 / 60 / 4000^2 = ACC[RPM/S] * 19215358410.114116266666666666667
        BGVAR(u64_PosTuneTraj_AccDec_Int)[0]       = 19215358410114LL;// Start from 1000 rpm/s
        // Tsp = 125u
        // ACC[internal] = ACC[RPM/S] * 2^64 / 60 / 8000^2 = ACC[RPM/S] * 4803839602.5285290666666666666667
        if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)
            BGVAR(u64_PosTuneTraj_AccDec_Int)[0]       = 4803839602528LL;// Start from 1000 rpm/s
        //Set half rev to Perform dummy half profile motion for balancing movements around start point
        // Perform only half distance of PATH1
        //          |-----------|-----------|
        //                      *>>>>>>>>>v
        //                                 V
        //           <<<<<<<<<<<<<<<<<<<<<v
        //
        BGVAR(s64_PosTuneTraj_Pos_Int)[0]          = 2147483647LL; // 0.5 revolution (pos)
        BGVAR(u32_PosTuneTraj_Spd_Int)[0]          = s32_temp;//4473924UL; // 500 rpm
        BGVAR(s16_PosTuneTraj_Delay_Int)[0]        = 500; // Set 200 ms Delay after 1st path
        // Set ACC,DEC,Control,Delay after Path execution, Position, Speed for PATH 2
        BGVAR(u64_PosTuneTraj_AccDec_Int)[1]       = BGVAR(u64_PosTuneTraj_AccDec_Int)[0];// Start from 1000 rpm/s
        if(BGVAR(s16_Pos_Tune_Cycle) < 2) BGVAR(s64_PosTuneTraj_Pos_Int)[1] = -4294967295LL; // Check of profile is forth-back or forth only
        else BGVAR(s64_PosTuneTraj_Pos_Int)[1]     = 4294967295LL;
        BGVAR(u32_PosTuneTraj_Spd_Int)[1]          = s32_temp;//8947849UL;// 1000 rpm
        BGVAR(s16_PosTuneTraj_Delay_Int)[1]        = 500; // Set 200 ms Delay after 2nd path

        //Set limit to 0.7*(min(ILIM, min(micont,dicont))
        if (BGVAR(s32_Drive_I_Cont) > BGVAR(s32_Motor_I_Cont))
        {
            // set limit to 70% ILIM if ilim is lower
            if(BGVAR(s32_Motor_I_Cont) > BGVAR(s32_Ilim_Actual))
                BGVAR(s16_PTT_Current_Limit) = (int)(((float)BGVAR(s32_Ilim_Actual)  * 0.7 * 26214L) / (float)BGVAR(s32_Drive_I_Peak));
        else
            {
                // set limit to 70% MICONT if micont/ilim > 0.7 -> micont near ilim can cause to ICMD sat fault
                if(((float)BGVAR(s32_Motor_I_Cont)/(float)BGVAR(s32_Ilim_Actual) ) > 0.7)
                    BGVAR(s16_PTT_Current_Limit) = (int)(((float)BGVAR(s32_Motor_I_Cont) * 0.7 * 26214L) / (float)BGVAR(s32_Drive_I_Peak));
                else
                    BGVAR(s16_PTT_Current_Limit) = (int)(((float)BGVAR(s32_Motor_I_Cont) * 26214L) / (float)BGVAR(s32_Drive_I_Peak));
            }
        }
        else
        {
            // set limit to 70% ILIM if ilim is lower
            if(BGVAR(s32_Drive_I_Cont) > BGVAR(s32_Ilim_Actual))
                BGVAR(s16_PTT_Current_Limit) = (int)(((float)BGVAR(s32_Ilim_Actual)  * 0.7 * 26214L) / (float)BGVAR(s32_Drive_I_Peak));
            else
            {
                // set limit to 70% DICONT if dicont/ilim > 0.7 -> dicont near ilim can cause to ICMD sat fault
                if(((float)BGVAR(s32_Drive_I_Cont)/(float)BGVAR(s32_Ilim_Actual) ) > 0.7)
                    BGVAR(s16_PTT_Current_Limit) = (int)(((float)BGVAR(s32_Drive_I_Cont) * 0.7 * 26214L) / (float)BGVAR(s32_Drive_I_Peak));
                else
                    BGVAR(s16_PTT_Current_Limit) = (int)(((float)BGVAR(s32_Drive_I_Cont) * 26214L) / (float)BGVAR(s32_Drive_I_Peak));
            }
        }
        BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_ENABLED;
        BGVAR(s32_Capt_Vel_Var_Fb_0) = 0L;
        BGVAR(s64_PTT_Captured_Curr_Int) = 0LL;
        BGVAR(u32_PTT_Captured_Curr_Int_Cntr) = 0L;
        VAR(AX0_s16_Cycle_Dir) = 0x7FFF;
        AX0_AutoTune_Bits &= ~AT_RT_STANDSTILL_INTEGRATE_MASK;
        do {
          s16_temp = Cntr_3125;
          LLVAR(AX0_u32_Abs_Icmd_Acc_Lo) = 0LL;
          LVAR(AX0_u32_Abs_Icmd_ACC_Cntr) = 0L;
          BGVAR(s32_PTT_Pos_Vcmd_Prev) = 0L;
        } while (s16_temp != Cntr_3125);

        VAR(AX0_AutoTune_Bits) |= AT_DURING_CYCLE_MASK;
        VAR(AX0_AutoTune_Bits) |= AT_PTT_VEL_START_CAPTURE_MASK; // Start velocity capture
        BGVAR(s16_Pos_Tune_Traj_Handler_State) = POSTUNE_TRAJ_HNDLR_RUN_HALF_TRAJ;
     break;

     case POSTUNE_TRAJ_HNDLR_RUN_HALF_TRAJ:
        // if unexpected disable is occured then break (AT will care about restoring)
        if ((BGVAR(s16_Pos_tune_Mode) == 0) || (!Enabled(drive))) break;
        // Wait for cycle finish
        if (BGVAR(s16_Pos_Tune_Traj_Enable)) break;

        // I_integral [A] = I_integral(internal units) * DIPEAK / 26214 * 31.25e-6 * 4
        // V[rpm] = V(internal units) * 60 * 8000 / 2^32 ; 2^32 = 4294967296
        //
        //         MKT * I_integral * 30
        // LMJR = ----------------------- - 1
        //          MJ * V * pi
        // KEEP THIS VALUE FOR DEBUG PURPOSE
        // Sampling time correction for 4/8/16 kHz drive
        if (BGVAR(u32_Pwm_Freq) == 4000L)      s16_temp = 4;
        else if (BGVAR(u32_Pwm_Freq) == 8000L) s16_temp = 2;
        else                             s16_temp = 1;
        BGVAR(u32_PTT_LMJR) = (unsigned long)(((((float)BGVAR(u32_Motor_Kt) * (float)BGVAR(s64_PTT_Captured_Curr_Int) * (float)BGVAR(s32_Drive_I_Peak) * 4294967296.0 * (float)s16_temp * 62.5e-6) /
                                      ((float)BGVAR(s32_Motor_J) * (float)BGVAR(s32_Capt_Vel_Var_Fb_0) * 8000.0 * 26214.0 * 3.14159)) - 1.0) * 1000.0);

        // J * ACC = MKT * I
        // apply Integral on both sides
        // integral(J * ACC) = integral(MKT * I)
        // integral(ACC) = V
        // J * V = MKT * integral(Icmd)
        // J = (MKT * I_integral)/V
        // Subtitute J
        // (MKT * I_integral)/V * ACC = MKT * I
        // ACC = V * Icmd / I_integral
        //      V           MKT
        // ---------- = ----------
        // I_integral     J_total
        // This ratio is constant, hence we can use it for ACC or Icmd calculations
        // And this allows us not to be depended on controller gains. Whe we have this ratio we can "think" that we have ideal profile behavior where velocity is linear
        // Calc ACC for Icmd = min(micont,dicont)
        // for tsp 125 usec
        // ACC[rpm/s] -> ACC(internal units) = ACC*2^64/60/8000^2
        // V[rpm] = V(internal units) * 60 * 8000 / 2^32
        // ACC[int. units] = V(int. units) * 2^32 / 8000 * Icmd_Limit / Icmd_Integral
        // for tsp 250 usec
        // ACC[rpm/s] -> ACC(internal units) = ACC*2^64/60/4000^2
        // V[rpm] = V(internal units) * 60 * 8000 / 2^32
        // ACC[int. units] = V(int. units) * 2^32 / 2000 * Icmd_Limit / Icmd_Integral
        //

        if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) f_tsp_factor = 2000;
        else f_tsp_factor = 8000;
        // calc ACC for current = min(ILIM, min(micont,dicont)
        BGVAR(u64_PTT_Acc_Calculated) = (unsigned long long)(((float)BGVAR(s32_Capt_Vel_Var_Fb_0) * 4294967296.0 * (float)BGVAR(s16_PTT_Current_Limit)) /
                                                (f_tsp_factor * (float)BGVAR(s64_PTT_Captured_Curr_Int) * 4.0 * 31.25e-6));
        // Sampling time correction for 8kHz drive
        if (BGVAR(u32_Pwm_Freq) == 8000L) BGVAR(u64_PTT_Acc_Calculated) = BGVAR(u64_PTT_Acc_Calculated) >> 1LL;
        // Sampling time correction for 4kHz drive
        if (BGVAR(u32_Pwm_Freq) == 4000L) BGVAR(u64_PTT_Acc_Calculated) = BGVAR(u64_PTT_Acc_Calculated) >> 2LL;


        // Verify that minimum current command is greater than min(micont,dicont)*0.3 to be sure we have enough data for ACC calculation
        // If not, then enlarge ACC for getting higher Icmd
        // Min_Current_Limit = Icmd_Integral(Int. units) / (Num of integration samples)
        BGVAR(s16_PTT_Min_Current_Limit) = (int)((float)BGVAR(s64_PTT_Captured_Curr_Int) * 4.0 /
                                                 (float)BGVAR(u32_PTT_Captured_Curr_Int_Cntr));
        BGVAR(s16_PTT_Real_Min_Current_Limit) = BGVAR(s16_PTT_Min_Current_Limit);
        // workaround for motor bq0603-3
        // this motor has low mj and ACC is wrong calculated and cuses to ICMD sat
        //
        // don't limit this workaround to speceific motors but keep it for the future
        /*s16_temp = RowInEasyTuningEntries();
        if((s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17920004) || (s_Easy_Tuning_Data[s16_temp].motor_file_name_cdhd == 17920004) ||
           (s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17921004) || (s_Easy_Tuning_Data[s16_temp].motor_file_name_cdhd == 17921004) ||
           (s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17910002) || (s_Easy_Tuning_Data[s16_temp].motor_file_name_cdhd == 17910002) ||
           (s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17911002) || (s_Easy_Tuning_Data[s16_temp].motor_file_name_cdhd == 17911002))
        {*/
                BGVAR(s16_PTT_Real_Min_Current_Limit) = (int)(((float)BGVAR(u64_PosTuneTraj_AccDec_Int)[0] * (f_tsp_factor * (float)BGVAR(s64_PTT_Captured_Curr_Int) * 4.0 * 31.25e-6))/
                                                         ((float)BGVAR(s32_Capt_Vel_Var_Fb_0) * 4294967296.0));

         //  }

        s16_temp = (int)((float)BGVAR(s16_PTT_Current_Limit) * 0.3);
        // Compare min current command and min(micont,dicont)*0.3
        // Stop If ACC > 62500 rpm/s

        if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) u64_max_acc_dec = MAX_ACC_DEC_TSP250;
        else u64_max_acc_dec = MAX_ACC_DEC_TSP125;

        if(BGVAR(u64_PTT_Acc_Calculated) > (u64_max_acc_dec>>4LL))
        {
            // Limit calculated ACC to 62500 rmp/s
            BGVAR(u64_PTT_Acc_Calculated) = u64_max_acc_dec>>4LL;
        }

        if((BGVAR(s16_PTT_Min_Current_Limit) <= s16_temp) && (BGVAR(s16_PTT_Real_Min_Current_Limit) <= s16_temp) &&
           (BGVAR(u64_PosTuneTraj_AccDec_Int)[0] != (u64_max_acc_dec>>4LL)))
        {
            // Enlarge ACC by ratio (min(micont,dicont)*0.3) / (min current command)
            // use ratio 1.1 If ratio is less than 1.1. This step reduces a number of iterations of ACC enlarging process
            s16_temp = (int)((float)s16_temp*1000.0/(float)BGVAR(s16_PTT_Min_Current_Limit));
            if(s16_temp < 1100) s16_temp = 1100;

            u64_temp = (unsigned long long)((float)BGVAR(u64_PosTuneTraj_AccDec_Int)[0] * (float)s16_temp/1000.0);

            // Stop If ACC > 62500 rpm/s
            if(u64_temp > (u64_max_acc_dec>>4LL))
            {
                // restore full distance value for PATH1
                // Limit ACC to 62500 rmp/s
                BGVAR(u64_PosTuneTraj_AccDec_Int)[0] = u64_max_acc_dec>>4LL;
                BGVAR(u64_PosTuneTraj_AccDec_Int)[1] = u64_max_acc_dec>>4LL;
                BGVAR(s64_PosTuneTraj_Pos_Int)[0]          = 4294967295LL; // 1 revolution (pos)
                BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_ENABLED;
                BGVAR(s16_Pos_Tune_Traj_Handler_State) = POSTUNE_TRAJ_HNDLR_WAIT_FOR_END;
                break;
            }

            BGVAR(u64_PosTuneTraj_AccDec_Int)[0]       = u64_temp;
            BGVAR(u64_PosTuneTraj_AccDec_Int)[1]       = u64_temp;
            BGVAR(s64_PosTuneTraj_Pos_Int)[0]          = 4294967295LL; // 1 revolution (pos)
            BGVAR(s32_Capt_Vel_Var_Fb_0) = 0L;
            BGVAR(s64_PTT_Captured_Curr_Int) = 0LL;
            BGVAR(u32_PTT_Captured_Curr_Int_Cntr) = 0L;
            do {
                s16_temp = Cntr_3125;
                LLVAR(AX0_u32_Abs_Icmd_Acc_Lo) = 0LL;
                LVAR(AX0_u32_Abs_Icmd_ACC_Cntr) = 0L;
            } while (s16_temp != Cntr_3125);
            BGVAR(s32_PTT_Pos_Vcmd_Prev) = 0L;
            VAR(AX0_AutoTune_Bits) |= AT_DURING_CYCLE_MASK;
            VAR(AX0_AutoTune_Bits) |= AT_PTT_VEL_START_CAPTURE_MASK; // Start velocity capture
            BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_ENABLED;
        }

        else
        {
            // restore full distance value for PATH1
            BGVAR(u64_PosTuneTraj_AccDec_Int)[0] = BGVAR(u64_PTT_Acc_Calculated);
            BGVAR(u64_PosTuneTraj_AccDec_Int)[1] = BGVAR(u64_PTT_Acc_Calculated);
            BGVAR(s64_PosTuneTraj_Pos_Int)[0]          = 4294967295LL; // 1 revolution (pos)
            BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_ENABLED;
            BGVAR(s16_Pos_Tune_Traj_Handler_State) = POSTUNE_TRAJ_HNDLR_WAIT_FOR_END;
        }
     break;

     case POSTUNE_TRAJ_HNDLR_WAIT_FOR_END:
        // if unexpected disable is occured then break (AT will care about restoring)
        if ((BGVAR(s16_Pos_tune_Mode) == 0) || (!Enabled(drive))) break;
        // Wait for cycle finish
        if (BGVAR(s16_Pos_Tune_Traj_Enable)) break;

        BGVAR(s16_Pos_Tune_Traj_Handler_State) = POSTUNE_TRAJ_HNDLR_CALC;

             // Perform only half distance of PATH2 (Come back to centre)
             //          |-----------|-----------|
             //           >>>>>>>>>>>>>>>>>>>>>v
             //                                 V
             //                      *<<<<<<<<<v
             //
        if(BGVAR(s16_Pos_Tune_Cycle) < 2) BGVAR(s64_PosTuneTraj_Pos_Int)[1] = -2147483647LL; // Check of profile is forth-back or forth only
        else BGVAR(s64_PosTuneTraj_Pos_Int)[1]     = 2147483647LL;
        BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_ENABLED;
     break;

     case POSTUNE_TRAJ_HNDLR_CALC:
        // if unexpected disable is occured then break (AT will care about restoring)
        if ((BGVAR(s16_Pos_tune_Mode) == 0) || (!Enabled(drive))) break;
        // Wait for cycle finish
        if (BGVAR(s16_Pos_Tune_Traj_Enable)) break;

        s32_temp = (long)((float)BGVAR(u32_Mspeed)*0.1); //set max speed to (mspeed*0.1)
        //ACC_ext[rpm/s] = (V[rpm] - V_prev[rpm])/dt = (V[rpm] - V_prev[rpm])/1mSec
        //ACC_int = (V_int_32 - V_int_32_prev)*60*8000/2^32/1e-3*2^64/60/4000^2 =
        //        = (V_int_32 - V_int_32_prev)*2^31 = (V_int_32 - V_int_32_prev)<<31;
        //u64_temp = (unsigned long long)((BGVAR(s32_Capt_Vel_Var_Fb_0) - BGVAR(s32_Capt_Vel_Var_Fb_0_Prev)));
        //u64_temp = (unsigned long long)(u64_temp<<31LL);
        // Calc position param for trajectory
//                    ^
//                    |
//        mspeed*0.25 +         ___________
//                    |        /|          |\
//                    |       / |          | \
//                    |      /  |          |  \
//                    |     /   |          |   \
//                    |    /    |          |    \
//                    |   /     |          |     \
//                    |  /  1/3 |   1/3    | 1/3  \
//                  --|-+-------+----------+-------+-----> t
//                      |<-x1-> |  <-x2->  |<-x3-> |
//
// x(t) = x(0) + v(0)*t + 0.5a*t^2
// x1 = x3 = 0.5*a*t^2  ;   x2 = v(0)*t
// x_total = x1+x2+x3
// x_total = a*t^2 + v(0)*t
// t = v(0)/a
// ==> x_total = a*v^2/a^2 + v*v/a = 2*v^2/a
// internal units
// x1 = x3 = 2*V_int^2/ACC_int [revs]
// x2 = 4*V_int^2/ACC_int [revs]
// x_total = 8*V_int^2/ACC_int [revs]
// revs to internal units
// x_total[revs]*2^32

        BGVAR(u32_PosTuneTraj_Spd_Int)[0]          = s32_temp; //min(vlim,mspeed*0.1)
        BGVAR(u32_PosTuneTraj_Spd_Int)[1]          = s32_temp; //min(vlim,mspeed*0.1)
        BGVAR(s16_PosTuneTraj_Delay_Int)[0]    = 500; //200 ms
        BGVAR(s16_PosTuneTraj_Delay_Int)[1]    = 1000;//500;//1000 ms

        // Reduce ACC to fit 8mS profile rise time
        // t[s] = V[RPM]/ACC[RPM/s]

        // for tsp 125 usec
        // ACC[rpm/s] -> ACC(internal units) = ACC*2^64/60/8000^2
        // V[rpm] = V(internal units) * 60 * 8000 / 2^32
        //
        //          V(internal units) * 60 * 8000 / 2^32        V(internal units) * 2^32
        // t[s] = ------------------------------------------ = ----------------------------
        //          ACC(internal units) * 60 * 8000^2 / 2^64    ACC(internal units) * 8000
        //
        //          V(internal units) * 2^32      V(internal units) * 2^29
        // t[ms] = --------------------------- = --------------------------
        //          ACC(internal units) * 8       ACC(internal units)
        //
        // for tsp 250 usec
        // ACC[rpm/s] -> ACC(internal units) = ACC*2^64/60/4000^2
        // V[rpm] = V(internal units) * 60 * 8000 / 2^32
        //
        //          V(internal units) * 60 * 8000 / 2^32        V(internal units) * 2^32
        // t[s] = ------------------------------------------ = ----------------------------
        //          ACC(internal units) * 60 * 4000^2 / 2^64    ACC(internal units) * 2000
        //
        //          V(internal units) * 2^32      V(internal units) * 2^31
        // t[ms] = --------------------------- = --------------------------
        //          ACC(internal units) * 2       ACC(internal units)
        //
        //

        if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) f_tsp_factor = 2147483648.0; // 2^31
        else f_tsp_factor = 536870912.0;                                            // 2^29

        s16_temp = (int)((float)BGVAR(u32_PosTuneTraj_Spd_Int)[0] * f_tsp_factor /
                         (float)BGVAR(u64_PosTuneTraj_AccDec_Int)[0]);
        // check if rise time is less than 8mS
        // if less then reduce ACC by formulae:
        // ACC = V/t = V/8e-3
        // Tsp = 250u
        // ACC(internal units) = V(internal units) * 2^31 / 8 = V(internal units) * 268435456
        // Tsp = 125u
        // ACC(internal units) = V(internal units) * 2^29 / 8 = V(internal units) * 67108864
        if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) f_tsp_factor = 268435456.0 ;
        else f_tsp_factor = 67108864.0;
        if(s16_temp < 8)
        {

            BGVAR(u64_PosTuneTraj_AccDec_Int)[0] = (unsigned long long)((float)BGVAR(u32_PosTuneTraj_Spd_Int)[0] * f_tsp_factor);
            BGVAR(u64_PosTuneTraj_AccDec_Int)[1] = BGVAR(u64_PosTuneTraj_AccDec_Int)[0];
        }

        // 8*V_int^2/ACC_int * 2^32
        // 8*2^32 = 34359738368
        // yuval - increase const speed area - to x1 = 0.25 x2 = 0.5 and x3 = 0.25 dec
        // 9*2^32 38654705664
        // Calc distance
        if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) f_tsp_factor = 38654705664.0 ;
        else f_tsp_factor = 9663676416; //38654705664.0/4.0;
        u64_temp = (long long)((f_tsp_factor*((float)s32_temp*(float)s32_temp)/((float)BGVAR(u64_PosTuneTraj_AccDec_Int)[0])));

        if ((BGVAR(u16_HdTune_Adv_Mode) != 0) && (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_2))
        {
            if((long long)u64_temp < 8589934590LL) u64_temp = 8589934590LL; // Set minimum position to 2 rev
            if(u64_temp > 19327352832)// 4.5 motor rev
            {
                ret_val = (POSTUNE_AUTO_TRAJ_MODE_TOO_LONG);
                break;
            }
        }
        else
        {
             if((long long)u64_temp < 4294967295LL) u64_temp = 4294967295LL; // Set minimum position to 1 rev

             s64_pos_tune_auto_traj_pos_lim = 12884901885LL; // 3 motor rev
             if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) s64_pos_tune_auto_traj_pos_lim = 30064771065LL; // 7 motor rev
             if (BGVAR(s64_Pos_Tune_AutoTraj_Pos_Lim) != 0) s64_pos_tune_auto_traj_pos_lim = BGVAR(s64_Pos_Tune_AutoTraj_Pos_Lim);

             if(u64_temp > s64_pos_tune_auto_traj_pos_lim)
             {
                ret_val = (POSTUNE_AUTO_TRAJ_MODE_TOO_LONG);
                break;
             }
         }

        // Perform only half distance of PATH1
        //          |-----------|-----------|
        //                      *>>>>>>>>>v
        //                                 V
        //           <<<<<<<<<<<<<<<<<<<<<v
        BGVAR(s64_PosTuneTraj_Pos_Int)[0]           = ((long long)u64_temp)>>1LL;
        if(BGVAR(s16_Pos_Tune_Cycle) < 2) u64_temp  = -(long long)(u64_temp); // Check of profile is forth-back or forth only
        BGVAR(s64_PosTuneTraj_Pos_Int)[1]           = (long long)(u64_temp);
        BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_ENABLED;
        BGVAR(s16_Pos_Tune_Traj_Handler_State) = POSTUNE_TRAJ_HNDLR_CENTREING;
     break;

     case POSTUNE_TRAJ_HNDLR_CENTREING:
        // if unexpected disable is occured then break (AT will care about restoring)
        if ((BGVAR(s16_Pos_tune_Mode) == 0) || (!Enabled(drive))) break;
        // Wait for cycle finish
        if (BGVAR(s16_Pos_Tune_Traj_Enable)) break;

        // Restore trajectory after centreing
        BGVAR(s64_PosTuneTraj_Pos_Int)[0] = BGVAR(s64_PosTuneTraj_Pos_Int)[1];
        if(BGVAR(s16_Pos_Tune_Cycle) < 2) BGVAR(s64_PosTuneTraj_Pos_Int)[0] = -BGVAR(s64_PosTuneTraj_Pos_Int)[1]; // Check of profile is forth-back or forth only

        // Change mode back to infinite forth and back profile
        BGVAR(s16_Pos_Tune_Traj_Mode) = PTT_INT_MODE;
        // restore parameters
        BGVAR(u32_Nl_Kpgmax) = BGVAR(u32_PTT_Nl_Kpgmax_BackUp);
        BGVAR(u32_Nl_KffSpring_User) = BGVAR(u32_PTT_Nl_KffSpring_User_BackUp);
        PositionConfig(drive,0);
        // Reset ICMD sat mask
        VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;
        BGVAR(s16_Pos_Tune_Traj_Handler_State) = POSTUNE_TRAJ_HNDLR_IDLE;
        if  (BGVAR(u64_DecStop) <= BGVAR(u64_PosTuneTraj_AccDec_Int)[0]) BGVAR(s32_Pos_Tune_Bits) |= AT_DEC_DECSTOP_WARNING;
        ret_val = (POSTUNE_AUTO_TRAJ_MODE_ACTIVE);
     break;
  }

   return (ret_val);
}

int PosTuneTrajReduceACC(int drive)
{
    unsigned long long u64_temp;
    int s16_temp, ret_val = 0;
    float f_tsp_factor = 0.0;
    // AXIS_OFF;

    //-1 - No need in ACC reducing
    // 0 - On going
    // 1 - ACC is reduced
    if((BGVAR(u16_Cycle_Cycles_Counter) - BGVAR(u16_Cycle_Cycles_Counter_Prev)) <= 1) return 0;

    // do until AT_SKIP_MAX_ACC_CALC_MASK is not set
    switch(BGVAR(s16_Pos_Tune_ACC_Reduce_State))
    {
        case POSTUNE_ACC_RED_START:
            // Skip ACC reducing if:
            // 1. Trajectory mode is external or user defined
            // 2. ACC reduced in the past
            // 3. Max caught current is below ILIM*0.9
            s16_temp = (int)((float)VAR(AX0_s16_Max_Crrnt_Peak)*1.1);

            if((BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_INT_MODE) && (BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_INT_MODE_1_CYCLE) ||
               (((VAR(AX0_AutoTune_Bits) & AT_SKIP_MAX_ACC_CALC_MASK) != 0) ||
                 (s16_temp < VAR(AX0_s16_I_Sat_Hi))))       ret_val = -1;
            else
                BGVAR(s16_Pos_Tune_ACC_Reduce_State) = POSTUNE_ACC_RED_FINISH_CYCLE;
            break;

        case POSTUNE_ACC_RED_FINISH_CYCLE:
            // Gentle cycle finishing
            BGVAR(s16_Pos_Tune_Traj_Mode) = PTT_INT_MODE_1_CYCLE;
            // Wait to cycle finish
            if (BGVAR(s16_Pos_Tune_Traj_Enable)) break;

             // Perform only half distance of PATH2 (Come back to centre)
             //          |-----------|-----------|
             //           >>>>>>>>>>>>>>>>>>>>>v
             //                                 V
             //                      *<<<<<<<<<v
             //
            BGVAR(s64_PosTuneTraj_Pos_Int)[1]          = (BGVAR(s64_PosTuneTraj_Pos_Int)[1])>>1LL;
            BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_ENABLED;
            BGVAR(s16_Pos_Tune_ACC_Reduce_State) = POSTUNE_ACC_RED_RECALC;
        break;

        case POSTUNE_ACC_RED_RECALC:
            // Wait to cycle finish
            if (BGVAR(s16_Pos_Tune_Traj_Enable)) break;
            // reduce ACC by 40% (same ratio as ILIM - ILIM*0.6)
            u64_temp = (unsigned long long)(((float)BGVAR(u64_PosTuneTraj_AccDec_Int)[0])*0.6);
            BGVAR(u64_PosTuneTraj_AccDec_Int)[0]       = u64_temp;
            BGVAR(u64_PosTuneTraj_AccDec_Int)[1]       = u64_temp;
            // 8*V_int^2/ACC_int * 2^32
            // 8*2^32 = 34359738368
            // yuval - increase const speed area - to x1 = 0.25 x2 = 0.5 and x3 = 0.25 dec
            // 9*2^32 38654705664

            if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) f_tsp_factor = 38654705664.0 ;
            else f_tsp_factor = 9663676416; //38654705664.0/4.0;

            u64_temp = (long long)((f_tsp_factor*((float)BGVAR(u32_PosTuneTraj_Spd_Int)[0]*(float)BGVAR(u32_PosTuneTraj_Spd_Int)[0])/((float)u64_temp)));
            if((long long)u64_temp < 8589934590LL) u64_temp = 8589934590LL; // Set minimum position to 2 rev

             // Perform only half distance of PATH1
             //          |-----------|-----------|
             //                      *>>>>>>>>>v
             //                                 V
             //           <<<<<<<<<<<<<<<<<<<<<v
             //
            BGVAR(s64_PosTuneTraj_Pos_Int)[0]           = ((long long)u64_temp)>>1LL;
            if(BGVAR(s16_Pos_Tune_Cycle) < 2) u64_temp  = -(long long)(u64_temp); // Check of profile is forth-back or forth only
            BGVAR(s64_PosTuneTraj_Pos_Int)[1]           = ((long long)u64_temp);
            BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_ENABLED;
            BGVAR(s16_Pos_Tune_ACC_Reduce_State) = POSTUNE_ACC_RED_CENTREING;
        break;

        case POSTUNE_ACC_RED_CENTREING:
            if (BGVAR(s16_Pos_Tune_Traj_Enable)) break;
            // Restore trajectory to full path
            u64_temp = BGVAR(s64_PosTuneTraj_Pos_Int)[1];
            if(BGVAR(s16_Pos_Tune_Cycle) < 2) u64_temp  = -(long long)(u64_temp); // Check of profile is forth-back or forth only
            BGVAR(s64_PosTuneTraj_Pos_Int)[0]           =  (long long)(u64_temp);

            // Reset ICMD sat mask
            VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;
            VAR(AX0_AutoTune_Bits) |= AT_SKIP_MAX_ACC_CALC_MASK; // Skip max ACC detection
            VAR(AX0_AutoTune_Bits)  &= ~AT_AUTO_TRAJ_ICMD_OV_MASK;
            CycleArrayOverFlow(drive); // restart cycle ident module
            // Come back to regular cycle mode
            BGVAR(s16_Pos_Tune_Traj_Mode) = PTT_INT_MODE;
            BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_ENABLED;
            BGVAR(s16_Pos_Tune_ACC_Reduce_State) = POSTUNE_ACC_DONE;
            ret_val = 1;
        break;

      case POSTUNE_ACC_DONE:
         ret_val = -1;
         break;

    }
    return ret_val;
}

void PosTuneTrajZeroingParam(int drive)
{
    // AXIS_OFF;
    if( (BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_IDLE)  ||
        (u16_P2_32_Lex_Auto_Tune_Mode == 2)          ||
        (u16_P2_32_Lex_Auto_Tune_Mode == 3)          ||
        (u16_P2_32_Lex_Auto_Tune_Mode == 52)         ||
        (u16_P2_32_Lex_Auto_Tune_Mode == 53)           )
    {
        BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_DISABLED;
        VAR(AX0_AutoTune_Bits) &= ~AT_PTT_VEL_START_CAPTURE_MASK; // End velocity capture
        BGVAR(s16_Pos_Tune_Traj_Handler_State) = POSTUNE_TRAJ_HNDLR_IDLE;
    }
    BGVAR(s16_Pos_Tune_Traj_Mode) = BGVAR(s16_Pos_Tune_Traj_Mode_User); // restore HDTUNEREFMODE
    if(BGVAR(s16_Pos_Tune_Traj_Mode_User) == PTT_INT_MODE) PosTuneTrajMode(0LL,drive);               // hdtunereference 0

}



void AutotTuneHandlerV2(int drive)
{
   int s16_temp;
   long s32_temp;
   unsigned int u16_temp;
   unsigned long u32_temp;
   long long s64_temp;
   float f_temp;

   static int cont_tune_osc_flag = 0;
   // AXIS_OFF;

   PSDHandler(drive);           // placed here since this handler is called many times during one background cycle
   //OnLineLMJRHandler(drive);// placed here since this handler is called many times during one background cycle


   // if drive is inactive do not hang due to paramters update
   if (((VAR(AX0_s16_Cycle_Bits) & PARAM_UPDATE_AT_STANDSTILL) != 0) && (!Enabled(drive))) VAR(AX0_s16_Cycle_Bits) &= ~PARAM_UPDATE_AT_STANDSTILL;
   
   // if cycle not identified - clear this bit 
   if (((VAR(AX0_s16_Cycle_Bits) & PARAM_UPDATE_AT_STANDSTILL) != 0) && 
       (Enabled(drive))                                              && 
       (BGVAR(s16_Cycle_StandStill_Found) == 0)                         ) VAR(AX0_s16_Cycle_Bits) &= ~PARAM_UPDATE_AT_STANDSTILL;


   if ( (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE) && (BGVAR(s16_Pos_Tune_State) > POS_TUNE_IDLE) && ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_SERVICE_MASK) == 0))
   { // enable cycle and lmjr est
      EN_CYCLE_IDENT;
      PosTuneCEffortDesign(drive); // update conrtol effort Zscale
      BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_SERVICE_MASK;
      BGVAR(s32_Pos_Tune_Bits) &= ~(INITIAL_RESULTS_MISMATCH_MASK|POS_TUNE_LOW_COST_MASK|AT_TRAJ_LO_SPEED_MASK|NO_DATA_FOR_LMJR_EST_MASK|AT_TRAJ_BELOW_ACC_MIN_MASK|AT_TRAJ_ABOVE_ACC_MAX_MASK|POS_TUNE_KNLP_DELTA_WARN|POS_TUNE_KNLIV_DELTA_WARN|POS_TUNE_CYCLE_WARN|POS_TUNE_USER_GAIN_MODIFED|POS_TUNE_EFFORT_WARN_AV3|POS_TUNE_EFFORT_WARN_AV2|POS_TUNE_KNLI_KNLIV_WARN|POS_TUNE_COST_FATOR_FAULT|POS_TUNE_KNLP_KNLIV_WARN|POS_TUNE_EFFORT_RESET);
      VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_RT_ACTIVE;
      VAR(AX0_AutoTune_Bits) &=~(AT_RT_STANDSTILL_INTEGRATE_MASK|AT_AUTO_TRAJ_ICMD_OV_MASK|AT_ICMD_OV_MASK|AT_SKIP_MAX_ACC_CALC_MASK|AT_AV_ACTIVE_2_MASK|AT_AV_ACTIVE_3_MASK);
      BGVAR(s32_Pos_Tune_Bits) &=~(CYCLE_TIMEOUT_WARN_MASK|CYCLE_BUFF_CORRUPT_WARN_MASK|AT_DEC_DECSTOP_WARNING|IGRAV_TUNING_RECOMMENDED_MASK|TQF_FUNC_ENABLED|UNKNOWN_MOTOR_ID_MASK|AT_AV_GAIN_INIT_EFFORT_MASK|EXTREME_EFFORT_MASK|EFFORT_EVENT_MASK|COST_0_MASK|HIGH_LMJR_DETECTED);
      VAR(AX0_s16_Cycle_Bits) &= ~MIN_MAX_RATIO_HIGH;

      BGVAR(s32_Pos_Tune_Av_Param_Init) = 0;
      PosTunePeMaskDesign(drive);
      s16_PosTune_Knliv_Step2 = s16_PosTune_Knliv_Step = POSTUNE_KNLIV_STEP_MIN;

      // Init auto-trajectory state machine
      BGVAR(s16_Pos_Tune_Traj_Handler_State) = POSTUNE_TRAJ_HNDLR_IDLE;
      BGVAR(s16_Pos_Tune_ACC_Reduce_State) = POSTUNE_ACC_RED_START;

      BGVAR(u16_StandStill_Ticks) = BGVAR(u16_HDTUNE_Dwell_Time_mS);
      BGVAR(s16_Pos_Tune_Av_State) = POS_TUNE_AV_IDLE;

      VAR(AX0_AutoTune_Effort_Gain) = 0x2000;
      HdTuneProgressBar(drive,1); // reset progress bar
      // enable TQF meas
      LVAR(AX0_u32_VBL_Thresh) = 0x7FFFFFFF;
      VAR(AX0_u16_NL_Tune_Status) = 3;
      BGVAR(s64_Tune_Min_Tqf) = 0x7FFFFFFFFFFFFFFF;
      BGVAR(s64_Cost_Before_At) = 0x7FFFFFFFFFFFFFFF;
      s32_TuneSearchTableFirstTQF[0] = s32_TuneSearchTableMinTQF[0] = -1;
      BGVAR(u16_RT_Tqf_Control) = 0;
      BGVAR(u16_Cycle_Counter_Mask) = 0x3;

      AutoTuneAntiVibWithPSD(drive,0,1); // reset psd av state machine

      BGVAR(s16_HdTune_Igrav_En) = 0;
      BGVAR(s16_Igrav_Tuning) = 0;
      BGVAR(u16_At_Foldback_Capture) = 0;
      BGVAR(s16_Cost_Improvement) = 0;
      //to be removed after testing VAR(AX0_s16_AT_Safe_Window_Cntr) = 0; // Init safe window counter
      BGVAR(s32_At_Fold_Lim) = (((long)BGVAR(f_I_Rail) - BGVAR(s32_I_Lim_Internal)) >> 1) + BGVAR(s32_I_Lim_Internal);

      BGVAR(s16_Tune_Table_Size) = -1; // protect from WD due to call to hdtunetable
      BGVAR(u16_Cycle_Cycles_Counter_Previous) = BGVAR(u16_Cycle_Cycles_Counter);
      BGVAR(s16_Cont_Tune_Flags) = CONT_TUNE_INC_USERGAIN_MASK;
      BGVAR(s64_Cont_Tune_Init_Cost) = 0;
      BGVAR(s32_Hdtune_Low_Frq_Osc_Max_Capture) = 0;
      BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) = 0;

      ATPertubationsHandler(drive,1);

      BGVAR(s16_FFT_State) = FFT_STATE_IDLE;
      s16_Knld_Max = 0;

      VAR(AX0_s16_Icmd_100Hz_Filtered_Max) = 0;
      // Calc 5% of MICONT
      // 26214*0.05=1310.7
      s32_temp = (long)(((float)BGVAR(s32_Motor_I_Cont) * 1310.7) / (float)BGVAR(s32_Drive_I_Peak));
      if(s32_temp >= 32767L) s32_temp = 32767L;
      BGVAR(s16_Motor_I_Cont_5_Prcnt) = (int)s32_temp;
      BGVAR(s16_Low_Current_Level_Cntr) = 0;

   }
   else if ( ((BGVAR(s16_Pos_Tune_State) <= POS_TUNE_IDLE) ||
              (BGVAR(s16_Pos_Tune_State) == POS_TUNE_DONE)   )         &&
             ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_SERVICE_MASK) != 0) &&
             ((VAR(AX0_s16_Cycle_Bits) & PARAM_UPDATE_AT_STANDSTILL) == 0) )
   { // disbale cycle and lmjr est
      //DIS_INERTIA_IDENT;
      
      if (s16_LMJREstSimplexState != LMJR_SIMPLEX_ST_DONE)
      {
         BGVAR(s16_Lmjr_Simplex_Reset) = -32767;                    // abort simplex
	      LMJREstimationSimplex(drive,BGVAR(s16_Lmjr_Simplex_Reset));
      }
      // stop lmjr estimation if active
      STOP_CYCLE_IDENT;
      BGVAR(s32_Pos_Tune_Bits) &= ~(POS_TUNE_SERVICE_MASK);
      VAR(AX0_s16_Cycle_Bits) &= ~(POS_TUNE_RT_ACTIVE|EFFORT_EXCEEDED|POS_TUNE_MIN_SETTLE_TIME_SCALE);
      VAR(AX0_AutoTune_Bits) &=~AT_NO_ACC_OPTIMIZE_MASK;
      BGVAR(s16_Pos_tune_Mode) = 0;
      AX0_u32_VBL_Thresh = 0x0300000;
      VAR(AX0_u16_NL_Tune_Status) = 0;
      BGVAR(s16_HdTune_Igrav_En) = 0;
      BGVAR(u16_RT_Tqf_Control) = 0;
   }

   if ((BGVAR(s16_Pos_Tune_State) > POS_TUNE_WAIT_ACTIVE) &&
       (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)         )
   {
      s16_temp = 0;
      

      /*if ( ((BGVAR(s32_Pos_Tune_Bits) & AT_TRAJ_BELOW_ACC_MIN_MASK) != 0)    &&
           (BGVAR(s16_Pos_Tune_Traj_Mode_User) == PTT_IDLE)                  &&
           (s16_temp == 0)                                                     )
      {
         BGVAR(s16_Pos_Tune_State) = AT_TRAJ_BELOW_ACC_MIN_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }*/

      /*if ( ((BGVAR(s32_Pos_Tune_Bits) & AT_TRAJ_ABOVE_ACC_MAX_MASK) != 0)    &&
           (BGVAR(s16_Pos_Tune_Traj_Mode_User) == PTT_IDLE)                  &&
           (s16_temp == 0)                                                     )
      {
         BGVAR(s16_Pos_Tune_State) = AT_TRAJ_ABOVE_ACC_MAX_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }*/
      
      /*if ( ((BGVAR(s32_Pos_Tune_Bits) & AT_TRAJ_LO_SPEED_MASK) != 0)    &&
           (BGVAR(s16_Pos_Tune_Traj_Mode_User) == PTT_IDLE)             &&
           (s16_temp == 0)                                                 )
      {
         BGVAR(s16_Pos_Tune_State) = AT_TRAJ_LO_SPEED_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }*/      
      
      if ( ((BGVAR(s32_Pos_Tune_Bits) & HIGH_LMJR_DETECTED) != 0)    &&
           (s16_temp == 0)                                                 )
      {
         BGVAR(s16_Pos_Tune_State) = AT_HIGH_LMJR_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }
      
      if ( ((BGVAR(s32_Pos_Tune_Bits) & AT_DEC_DECSTOP_WARNING) != 0)    &&
           (s16_temp == 0)                                                 )
      {
         BGVAR(s16_Pos_Tune_State) = AT_DEC_DECSTOP_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }

      if ( ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_COST_FATOR_FAULT) != 0) &&
           (s16_temp == 0)                                                 )
      {
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_COSTFACOR_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }


      if (((BGVAR(s32_Pos_Tune_Bits) & COST_0_MASK) != 0) &&
          (BGVAR(u16_HDTUNE_Warn_Mode) == 1)              &&
          (s16_temp == 0)                                   )
      {
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_COST_0_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }

      if ( ((BGVAR(s32_Pos_Tune_Bits) & UNKNOWN_MOTOR_ID_MASK) != 0)                                   &&  // in shndr fault if motor is not identifiedzz
           ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW) || (BGVAR(u16_HdTune_Adv_Mode) != 0))  &&
           (s16_temp == 0)                                                                               )
      {
         BGVAR(s32_Pos_Tune_Bits) &= ~UNKNOWN_MOTOR_ID_MASK;
         BGVAR(s16_Pos_Tune_State) = UNKNOWN_MOTOR_ID_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }


      if (((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_LOW_COST_MASK) != 0) && (s16_temp == 0))
      {
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_LOWEFFORT_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }

      if ((BGVAR(s16_Pos_Tune_State) > POS_TUNE_LMJR_EST) && (s16_temp == 0))
      {
         if ( (PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0)  < PosTuneReadParam(POS_TUNE_KNLI_ID,drive,2))  ||
              (PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0) < PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,2)) ||
             (PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0)  < PosTuneReadParam(POS_TUNE_KNLP_ID,drive,2))    )
         {
             BGVAR(s16_Pos_Tune_State) = AUTO_TUNE_GAINS_INVALID_FAULT;
             PosTuneRestore(drive,1);
             s16_temp = 1;
         }
      }

      if ((!Enabled(drive)) && (s16_temp == 0))
      {
         s16_Pos_Tune_State_Captured = BGVAR(s16_Pos_Tune_State);
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_STOPPED_FAULT;
         PosTuneRestore(drive,0);
         /*
         // Commented to fix BZ5208
         // It is possible because the code below is for supporting AT rerun without re-enable
         // Now we can do it without re-enable, hence this code will be commented
         // support FC - if need to re-initialize
         if ( (BGVAR(u16_Cycle_Cycles_Counter) == 0) && (BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_IDLE) ) BGVAR(s16_Pos_Tune_State) = POS_TUNE_SETUP;*/
         s16_temp = 1;
      }

      if ((BGVAR(u16_Hold) == 1) && (s16_temp == 0))
      {
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_HOLD_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }

      if (  (BGVAR(u16_HDTUNE_Sat_Mode) != 0)                           &&
            ((VAR(AX0_AutoTune_Bits) & AT_ICMD_OV_MASK) != 0)           &&
            (BGVAR(s16_Pos_Tune_State) > POS_TUNE_ADV_SEARCH_4_1)  &&
            (s16_temp == 0)                                               )
      {
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_ICMDSAT_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }


      if ( ((BGVAR(s16_Bundle_Design_Lock) <  0) || (BGVAR(s16_Bundle_Design_Lock) > CDHD_PRO2_MOTOR_DATA_ENTRIES)) &&
           (u16_Product != SHNDR)                                                                                   &&
           (u16_Product != SHNDR_HW)                                                                                &&
           (BGVAR(u16_HdTune_Adv_Mode) != 0)                                                                        &&
           (BGVAR(u16_MTP_Mode) != 0)                                                                                  )
      {
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_INVALID_AT_BUNDLE_FAULT;
         PosTuneRestore(drive,1);
         s16_temp = 1;
      }

      if ( (BGVAR(u64_Sys_Warnings) & (UNDER_VOLTAGE_WRN_MASK | PLL_WRN_MASK | SININIT_WARNING_MASK | BUS_AC_LOSS_WRN_MASK | REGEN_OVER_LOAD_WRN_MASK | SRVSNS_ENCODER_WRN_MASK))  &&
           (s16_temp == 0)                                                                                                                                                           )
      { // fault on drive warnings
          s16_Pos_Tune_State_Captured = BGVAR(s16_Pos_Tune_State);
          BGVAR(s16_Pos_Tune_State) = POS_TUNE_WARNINGS_EXSISTS_FAULT;
          PosTuneRestore(drive,1);
          s16_temp = 1;
      }

      if ( ((BGVAR(u64_Sys_Warnings) & (DRIVE_FOLDBACK_WRN_MASK | MOTOR_FOLDBACK_WRN_MASK)) || (BGVAR(u16_At_Fold) != 0)) &&
            (s16_temp == 0)                                                                                                 )
      {
         if (BGVAR(u16_At_Foldback_Capture) == 0) BGVAR(u16_At_Foldback_Capture) = BGVAR(u16_Cycle_Cycles_Counter) +15;
         if (BGVAR(u16_Cycle_Cycles_Counter) > BGVAR(u16_At_Foldback_Capture))
         {
          s16_Pos_Tune_State_Captured = BGVAR(s16_Pos_Tune_State);
          BGVAR(s16_Pos_Tune_State) = POS_TUNE_FOLDBACK_FAULT;
          PosTuneRestore(drive,1);
         }
         s16_temp = 1;
      }
      else
       BGVAR(u16_At_Foldback_Capture) = 0;

       // Test if number of cycles is greater than 750
      if(BGVAR(u16_Cycle_Cycles_Counter) >= 750)
        {
             s16_Pos_Tune_State_Captured = BGVAR(s16_Pos_Tune_State);
             BGVAR(s16_Pos_Tune_State) = POS_TUNE_OVER_COUNT_FAULT;
             PosTuneRestore(drive,1);
        }

      if ((VAR(AX0_AutoTune_Bits)  & AT_ICMD_OV_MASK) != 0)
         BGVAR(s32_Pos_Tune_Bits) |= AT_ICMD_OV_WARNING_MASK;
      else
         BGVAR(s32_Pos_Tune_Bits) &= ~AT_ICMD_OV_WARNING_MASK;

      if (BGVAR(u16_Cycle_Length) == 0)
         BGVAR(s32_Pos_Tune_Bits) |= NO_CYCLE_IDENTIFIED;
      else
         BGVAR(s32_Pos_Tune_Bits) &= ~NO_CYCLE_IDENTIFIED;

   if ((BGVAR(s16_Pos_Tune_State) >= POS_TUNE_LMJR_EST) &&
       (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE) )
       {
          // check if cycles are identified
          if((BGVAR(u16_Cycle_Cycles_Counter) != BGVAR(u16_Cycle_Cycles_Counter_Previous)))
            {
                 //reset timer if cycles are counting
                 BGVAR(s32_Cycle_Ident_Tout_Timer) = Cntr_1mS;
                 // Update previous cycle counter
                 BGVAR(u16_Cycle_Cycles_Counter_Previous) = BGVAR(u16_Cycle_Cycles_Counter);
            }
          else
            {
                 //issue warning if no change in cycles counting within 30 sec
                 if (PassedTimeMS(30000L, BGVAR(s32_Cycle_Ident_Tout_Timer)))
                    {
                        BGVAR(s32_Pos_Tune_Bits) |= CYCLE_TIMEOUT_WARN_MASK;
                    }
                 //issue fault if no change in cycles counting within 60 sec
                 if (PassedTimeMS(75000L, BGVAR(s32_Cycle_Ident_Tout_Timer)))
                    {
                        s16_Pos_Tune_State_Captured = BGVAR(s16_Pos_Tune_State);
                        BGVAR(s16_Pos_Tune_State) = POS_TUNE_CYCLE_CNT_TOUT_FAULT;
                        PosTuneRestore(drive,1);
                    }
            }
            
        if(BGVAR(s16_Low_Current_Level_Cntr) > 10)
        {
                     BGVAR(s16_Pos_Tune_State) = POS_TUNE_LOW_CURR_LEVEL_FAULT;
                     PosTuneRestore(drive,1);
        }  
           
       }
   }

   switch(BGVAR(s16_Pos_Tune_State))
   {
      case POS_TUNE_RESTART:
         s32_temp = PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0)>>1;
         ResetPosGainsForPosTune(drive);
         if (s32_temp > PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))
         {
            PosTuneWriteParam(POS_TUNE_KNLUSER_ID,s32_temp,drive);
            PositionConfig(drive,1);
            VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         }
         // Restart LMJR Estimator
         BGVAR(s16_Cycle_LMJR_Counter) = 0;
         CalculateLMJR(0,0,0,0); // reset static vars
         // Init prev counter - (need few cycles for CalcAtLimits - to update cruise speed and calc avg acc)
         BGVAR(u16_Cycle_Cycles_Counter_Prev_AT_Limits) = BGVAR(u16_Cycle_Cycles_Counter);
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_LMJR_EST;
      break;

      case POS_TUNE_SIMPLEX_LMJR_FAULT:
      case POS_TUNE_LOW_CURR_LEVEL_FAULT:
      case POS_TUNE_INVALID_AT_BUNDLE_FAULT:
      case POS_TUNE_MODE_2_TUNE_FAULT:
      case POS_TUNE_CYCLE_CNT_TOUT_FAULT:
      case POS_TUNE_OVER_COUNT_FAULT:
      case POS_TUNE_WARNINGS_EXSISTS_FAULT:
      case POS_TUNE_FOLDBACK_FAULT:
      case AUTO_TUNE_GAINS_INVALID_FAULT:
      case AUTO_TRAJ_TOO_LONG:
      case UNKNOWN_MOTOR_ID_FAULT:
      case POS_TUNE_HOLD_FAULT:
      case POS_TUNE_LOWEFFORT_FAULT:
      case POS_TUNE_COST_0_FAULT:
      case POS_TUNE_COSTFACOR_FAULT:
      case POS_TUNE_ICMDSAT_FAULT:
      case POS_TUNE_STOPPED_FAULT:
      case POS_TUNE_STOPPED_USER:
      case POS_TUNE_IDLE:
      break;

      case POS_TUNE_SETUP:
         if ((BGVAR(s16_Pos_Tune_Enable_Mode) == 0) && (Enabled(drive))) break;
         if ((BGVAR(s16_Pos_Tune_Enable_Mode) != 0) && (LVAR(AX0_s32_Pos_Vcmd) != 0L)) break;

         if (BGVAR(s16_Pos_tune_Mode) != 2) VAR(AX0_AutoTune_Bits) |= AT_NO_ACC_OPTIMIZE_MASK;

         BGVAR(s16_Pos_Tune_Weight) = BGVAR(s16_Pos_Tune_Weight_User);
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_WAIT_ACTIVE;
         if (BGVAR(s16_Pos_tune_Mode) == 90) break;
         if (BGVAR(u16_HdTune_Defaults) == 0) ResetPosGainsForPosTune(drive);
         // Restart LMJR Estimator
         BGVAR(s16_Cycle_LMJR_Counter) = 0;
         CalculateLMJR(0,0,0,0); // reset static vars
      break;

      case POS_TUNE_WAIT_ACTIVE:
         if (!Enabled(drive)) break;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_TRAJ_SETUP;
         //break; no break here - to save cycles

       case POS_TUNE_TRAJ_SETUP:
         // Skip max ACC detection in case ACC reduced
         if((VAR(AX0_AutoTune_Bits)  & AT_SKIP_MAX_ACC_CALC_MASK) == 0)
         {
            s16_temp = PosTuneTrajHandler(drive);
            if (s16_temp == 0) break;
            if (s16_temp == POSTUNE_AUTO_TRAJ_MODE_TOO_LONG)
            {
               BGVAR(s16_Pos_Tune_State) = AUTO_TRAJ_TOO_LONG;
               PosTuneRestore(drive,1);
               break;
            }
            
            // re-start trajectory ( traj handler stops it when it calculated the data
            STOP_CYCLE_IDENT; // Reset cycle identification
            u16_temp = BGVAR(u16_Cycle_Cycles_Counter);
            EN_CYCLE_IDENT;
            BGVAR(u16_Cycle_Cycles_Counter) = u16_temp;
            if (s16_temp > 0) BGVAR(s16_Pos_Tune_Traj_Enable) = PTT_ENABLED;
         }

         if (BGVAR(s16_Pos_tune_Mode) == 90) // AVHZ test mode
         {
            SalNlKAntiVibrationCommand(0LL,drive); // zero av
            SalNlPeGainCommand(0LL,drive);         // zero av2
            SalNlPe3GainCommand(0LL,drive);         // zero av3

            //reset timer if cycles are counting
            BGVAR(s32_Cycle_Ident_Tout_Timer) = Cntr_1mS;
            // Update previous cycle counter
            BGVAR(u16_Cycle_Cycles_Counter_Previous) = BGVAR(u16_Cycle_Cycles_Counter);
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_AVHZ_TEST_MODE;
            break;
         }

         if (BGVAR(s16_Pos_tune_Mode) == 4) BGVAR(u32_AT_LMJR) = 0; // lmjr est only

         //reset timer if cycles are counting
         BGVAR(s32_Cycle_Ident_Tout_Timer) = Cntr_1mS;
         // Update previous cycle counter
         BGVAR(u16_Cycle_Cycles_Counter_Previous) = BGVAR(u16_Cycle_Cycles_Counter);
         // Init prev counter - (need few cycles for CalcAtLimits - to update cruise speed and calc avg acc)
         BGVAR(u16_Cycle_Cycles_Counter_Prev_AT_Limits) = BGVAR(u16_Cycle_Cycles_Counter);
         BGVAR(u16_Cycle_Cycles_Counter_Limit_Cntr) = BGVAR(u16_Cycle_Cycles_Counter);
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_LMJR_EST;

         /*if (BGVAR(s16_Pos_Tune_Traj_Mode_User) == PTT_IDLE)*/ 
         s16_Number_Of_Parameters = 1;
         s64_Execution_Parameter[0] = 1;
         SalLmjrEstCommand(drive);// use simpelx lmjr est on ext trajectory - always en for debug
         break;


      case POS_TUNE_LMJR_EST:
         if (!Enabled(drive)) break;
         u32_temp = BGVAR(u16_Cycle_Cycles_Counter) - BGVAR(u16_Cycle_Cycles_Counter_Limit_Cntr);
         if ((BGVAR(s16_Cycle_LMJR_Counter)!=CYCLE_LMJR_LIM) &&
             (BGVAR(s16_Pos_Tune_LMJR) == 0)                 &&
             (BGVAR(u16_HdTune_Defaults) == 0)           &&
             ((BGVAR(s16_Pos_Tune_Traj_Mode_User) != PTT_IDLE))) break;
         // Issue fault if LMJR analyzing takes more than 5 cycles    
         if((s16_LMJREstSimplexState == LMJR_SIMPLEX_ST_ANALYZING) &&
              (BGVAR(s16_Pos_Tune_Traj_Mode_User) == PTT_IDLE))
         {
            if(u32_temp > 30) // Simplex est process may take more than 10 sec, hence the number of cycles enlarged to 25 to avoid false timeout
            {
                BGVAR(s16_Pos_Tune_State) = POS_TUNE_SIMPLEX_LMJR_FAULT;
                PosTuneRestore(drive,1);
                break;
            }
         }
        // Reset counter if simplex state is not analyzing. This is to support internal simplex retries
        else
         BGVAR(u16_Cycle_Cycles_Counter_Limit_Cntr) = BGVAR(u16_Cycle_Cycles_Counter);            
         // for ext traj - simplex lmjr is used
         if ((BGVAR(s16_Pos_Tune_Traj_Mode_User) == PTT_IDLE) &&
             (s16_LMJREstSimplexState != LMJR_SIMPLEX_ST_DONE) &&
             (s16_LMJREstSimplexState >= LMJR_SIMPLEX_ST_ON_GOING)) break;
             
         if ((BGVAR(s16_Pos_Tune_Traj_Mode_User) == PTT_IDLE) && (s16_LMJREstSimplexState >= LMJR_SIMPLEX_ST_ON_GOING)) 
            BGVAR(s32_Cycle_LMJR) = (long)BGVAR(u16_Simplex_LMJR);
         else if ((BGVAR(s16_Pos_Tune_Traj_Mode_User) == PTT_IDLE) && (s16_LMJREstSimplexState < LMJR_SIMPLEX_ST_ON_GOING))  
         {
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_SIMPLEX_LMJR_FAULT;
            PosTuneRestore(drive,1);
            break;
         }
                     
         
         u32_temp = BGVAR(u16_Cycle_Cycles_Counter) - BGVAR(u16_Cycle_Cycles_Counter_Prev_AT_Limits);
         if (u32_temp <= 6) break; // need few cycles for CalcAtLimits - to update cruise speed and calc avg acc

         if (BGVAR(s16_Pos_tune_Mode) == 4) // lmjr est only
         {
            BGVAR(s16_Cycle_End_First_Waiting) = 1;  // Use s16_Cycle_End_First_Waiting to be sure we are not at the end of standstill period
            //Set warning if LMJR > 30
            if(BGVAR(s32_Cycle_LMJR) > 35000L) BGVAR(s32_Pos_Tune_Bits) |= HIGH_LMJR_DETECTED;
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_TRAJ_STOP;
            break;
         }

         // set torque filter accrding to DB \ estiamtion
         u32_temp = BGVAR(u32_LMJR_User);
         if (BGVAR(s16_Pos_Tune_LMJR) == 0)
            BGVAR(u32_LMJR_User) = BGVAR(s32_Cycle_LMJR);
         else
            BGVAR(u32_LMJR_User) = BGVAR(u32_AT_LMJR);
         //Set warning if LMJR > 30
         if(BGVAR(u32_LMJR_User) > 35000L) BGVAR(s32_Pos_Tune_Bits) |= HIGH_LMJR_DETECTED;

         if (SetTorqueFilterAccordingToMotor(drive,0) != 1) BGVAR(s32_Pos_Tune_Bits) |= UNKNOWN_MOTOR_ID_MASK;

         // if (lmjr > 20) increase user gain for 0.25 to 0.5
         // gain too low may casue invalid results
         //if ((BGVAR(u32_LMJR_User) > 20000) && (BGVAR(u16_HdTune_Defaults) == 0)) PosTuneWriteParam(POS_TUNE_KNLUSER_ID,(long)(2*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))),drive);

         BGVAR(u32_LMJR_User) = u32_temp;
         PositionConfig(drive,1);
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         // for knld max- calc max knlusergain (when AT uses lmjr = 1)
         if (BGVAR(s16_Pos_Tune_LMJR) == 0)
            u32_temp = BGVAR(s32_Cycle_LMJR);
         else
            u32_temp = BGVAR(u32_AT_LMJR);

         BGVAR(s32_At_Usergain_Max) = (long)((float)BGVAR(s16_Knld_Max)*(1000 + (float)u32_temp)/((1.0+0.001*(float)BGVAR(u32_LMJR_User))*0.001*(float)PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0)));
         BGVAR(s32_At_Usergain_Min) = (long)(                      12.0*(1000 + (float)u32_temp)/((1.0+0.001*(float)BGVAR(u32_LMJR_User))*0.001*(float)PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0)));

         if ((BGVAR(u16_HdTune_Defaults) == 0) && (BGVAR(s32_At_Usergain_Min) > 250) && (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5)) 
         {
            PosTuneWriteParam(POS_TUNE_KNLUSER_ID,BGVAR(s32_At_Usergain_Min),drive);
            PositionConfig(drive,1);
            VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK); 
         }

         if ((BGVAR(s16_Pos_Tune_NlPeAff_Mode) != 0) && (BGVAR(u16_HdTune_Defaults) == 0))
         {
            PosTuneWriteParam(POS_TUNE_NLPEAFF_ID,PosTuneReadParam(POS_TUNE_NLPEAFF_ID,drive,1),drive);
            PosTuneWriteParam(POS_TUNE_NLAFFLPFHZ_ID,PosTuneReadParam(POS_TUNE_NLAFFLPFHZ_ID,drive,1),drive);
            PositionConfig(drive,1);
            VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         }

         CalcAtLimitsATTrajectoryGenerator(drive);

         BGVAR(s16_Pos_Tune_State) = POS_TUNE_IGRAV_SETUP;
         BGVAR(s16_HdTune_Igrav_En) = 2;
         VAR(AX0_s16_Cycle_Bits) &= ~CYCLE_NEW_CYCLE_DATA_READY;
         break;

   case POS_TUNE_IGRAV_SETUP:
         if ((VAR(AX0_s16_Cycle_Bits) & CYCLE_NEW_CYCLE_DATA_READY) == 0) break;
         VAR(AX0_s16_Cycle_Bits) &= ~CYCLE_NEW_CYCLE_DATA_READY;
         if (BGVAR(s16_HdTune_Igrav_En) > 1)
         {
            --BGVAR(s16_HdTune_Igrav_En);
            break;
         }

         // if value found is less than 5% of micont = ignore it
         s32_temp = (long)(1310.7*(float)BGVAR(s32_Motor_I_Cont)/(float)BGVAR(s32_Drive_I_Peak));
         if (s32_temp > 26214) s32_temp = 26214;
         if (abs(BGVAR(s16_Igrav_Tuning)) < (int)s32_temp) BGVAR(s16_Igrav_Tuning) = 0;
         if ((BGVAR(s16_Pos_Tune_Igrav) != 0)  &&
             (BGVAR(u16_HdTune_Adv_Mode) != 2) &&
             (BGVAR(s16_HdTune_Igrav_En) != 0) &&
             (BGVAR(u16_HdTune_Defaults) == 0)   ) VAR(AX0_s16_Igrav) = BGVAR(s16_Igrav_Tuning);

         if (BGVAR(s16_Igrav_Tuning) != VAR(AX0_s16_Igrav)) BGVAR(s32_Pos_Tune_Bits) |= IGRAV_TUNING_RECOMMENDED_MASK;
         BGVAR(s16_HdTune_Igrav_En) = 0;

         if (BGVAR(u16_HdTune_Defaults) == 0)
         {
            s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_INITIAL_COST_CAPTURE;
         }
         else
         {
            VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE;// set flag so that initi cost is in the same units as final cost
            s32_TuneSearchTable[0][0] = POS_TUNE_KNLUSER_ID;
            s32_TuneSearchTable[0][1] = POS_TUNE_KNLUSER_ID;
            s32_TuneSearchTable[0][2] = POS_TUNE_NO_ID;
            s16_Tune_Delta_Table[0] = 0;
            s16_Tune_Delta_Table[1] = 0;
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_INITIAL_COST_CAPTURE;
            BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
            TuneSearchInit(drive,0);
         }
         break;

      case POS_TUNE_INITIAL_COST_CAPTURE:
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;

         if (BGVAR(s64_Cost_Before_At) == 0x7FFFFFFFFFFFFFFF) BGVAR(s64_Cost_Before_At) = BGVAR(s64_Tune_Min_Cost);

         if (BGVAR(u16_HdTune_Defaults) != 0) ResetPosGainsForPosTune(drive);

         VAR(AX0_s16_Cycle_Bits) &= ~POS_TUNE_MIN_SETTLE_TIME_SCALE;// clr min settle time flag
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_USERGAIN_SEARCH;
         s32_TuneSearchTable[0][0] = POS_TUNE_KNLUSER_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;//table with one gain
         s16_Tune_Delta_Table[0] = 10;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_USERGAIN_SEARCH;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 0; // first generate will include the initial t1 value
         StoreInitialGains(drive);
         TuneSearchInit(drive,0);
         BGVAR(s16_Min_Cost_Found_Cntr) = BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 3;

         if ( (s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID) &&
              (BGVAR(u16_RT_Tqf_Control) == 0)                 ) BGVAR(u16_RT_Tqf_Control) = 1;

         BGVAR(s16_Cycle_LMJR_Counter) = 0;
         break;

      case POS_TUNE_USERGAIN_SEARCH:
         s16_temp = PosTuneSearch(drive,1,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }

         // search table - user 90% of effort- to leave room for the rest of the tuning
         LVAR(AX0_u32_At_Control_Effort_Tresh) = (long)((float)LVAR(AX0_u32_At_Control_Effort_Tresh)*0.9);

         s32_TuneSearchTable[0][0] = POS_TUNE_KNLUSER_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;//table with one gain
         s16_Tune_Delta_Table[0] = 0;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 2;
         s16_Pos_Tune_Verify_Limit = 5;
         if ((BGVAR(u16_HdTune_Adv_Mode) != 0) && (BGVAR(u16_Cycle_Counter_Mask) != 0x3)) 
         {
            BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_KNLUSER_VERIFY;
         TuneSearchInit(drive,0);
         break;

      case POS_TUNE_KNLUSER_VERIFY:
         s16_temp = PosTuneSearch(drive,2,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,0);
            break;
         }
         else if (BGVAR(s16_Cycle_LMJR_Counter) !=CYCLE_LMJR_LIM)
         {
            BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
            TuneSearchInit(drive,0);
            break; // make sure lmjr data is valid
         }

         // when in ineternal mode use long time till gain is set
         // the longer time needed to make sure next cycle starts with pe~0. other  wise there may be consistency issues
         if (BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE)
         {
            BGVAR(u16_Path_Delay)[2] = BGVAR(s16_PosTuneTraj_Delay_Int)[1] = 500;
            // Keep LMJR from zeroing (STOP sets dirflag to 7fff what causes to LMJR=0)
            u32_temp = BGVAR(s32_Cycle_LMJR);
            CycleArrayOverFlow(drive); // restart cycl ident module
         // Restore LMJR
            BGVAR(s32_Cycle_LMJR) = u32_temp;
         }

         if (s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID)
         {
            LVAR(AX0_u32_At_Control_Effort_Tresh) = (long)((float)LVAR(AX0_u32_At_Control_Effort_Tresh)*1.11);
            if (BGVAR(u16_HdTune_Adv_Mode) == 0) // if adv != 0 gain is reduced at the end of AT
               PosTuneWriteParam(POS_TUNE_KNLUSER_ID,(long)(0.8*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))),drive);
            else // reduce gain by 10% - if gain is clode to effort it might cause AT not to tune the rest of the parameters
               PosTuneWriteParam(POS_TUNE_KNLUSER_ID,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))),drive);

            AutoTuneGainNormalization(drive);
            AutoTuneLMJRNormalization(drive,0); // normalize results with cycle lmjr

            f_temp = 1e-6*(float)PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0) * (float)PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0);
            if (f_temp > (float)BGVAR(s16_Knld_Max))
            {
               f_temp = (float)s16_Knld_Max/f_temp;
               PosTuneWriteParam(POS_TUNE_KNLUSER_ID,(long)(f_temp*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))),drive);
               PositionConfig(drive,1); // must be before normalization for decrease to work
               AutoTuneGainNormalization(drive);
            }

            VAR(AX0_s16_Cycle_Bits) |= CYCLE_PARAM_UPDATE;

            // update effort if 2.5*(max value) < initial threshold
            s64_temp = ((5*(long long)BGVAR(u32_At_Control_Effort_Max_Captured))>>1) - (long long)LVAR(AX0_u32_At_Control_Effort_Tresh);
            if ((s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID) && (s64_temp < 0)) LVAR(AX0_u32_At_Control_Effort_Tresh) += (unsigned long)s64_temp;

            // verify effort is at least 15%
            if (EffortToPercent(drive,LVAR(AX0_u32_At_Control_Effort_Tresh)) < 15000) LVAR(AX0_u32_At_Control_Effort_Tresh) = PercentToEffort(drive,15000);
         }
         s32_TuneSearchTable[0][0] = POS_TUNE_NLFILTT1_AND_DAMPING_ID;
         if (s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID)
         {
            BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_FILTER_VERIFY;
            StoreInitialGains(drive);
            TuneSearchInit(drive,0);
            if ((BGVAR(u16_HdTune_Adv_Mode) != 0) ||
                (BGVAR(s16_Tune_Table_Size) == 1)   ) s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;

            if ((BGVAR(u16_HdTune_Adv_Mode) != 0) && (BGVAR(u16_Cycle_Counter_Mask) != 0x3)) 
            {
               BGVAR(u16_Cycle_Counter_Mask) = 0x3;
               CycleArrayOverFlow(drive); // restart cycle ident module
            }



            if ( (s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID) &&
                 (BGVAR(u16_RT_Tqf_Control) == 0)                 )BGVAR(u16_RT_Tqf_Control) = 1;
         }
         break;

      case POS_TUNE_FILTER_VERIFY:
         s16_temp = PosTuneSearch(drive,100,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }
         // check if need new knld max
         s16_temp = s16_Knld_Max;
         SetTorqueFilterAccordingToMotor(drive,3);
         if (s16_temp != s16_Knld_Max)
         {
            // for knld max- calc max knlusergain
            if (BGVAR(s16_Pos_Tune_LMJR) == 0)
               u32_temp = BGVAR(s32_Cycle_LMJR);
            else
               u32_temp = BGVAR(u32_AT_LMJR);

            BGVAR(s32_At_Usergain_Max) = (long)((float)BGVAR(s16_Knld_Max)*(1000 + (float)u32_temp)/((1.0+0.001*(float)BGVAR(u32_LMJR_User))*0.001*(float)PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0)));
            BGVAR(s32_At_Usergain_Min) = (long)(                      12.0*(1000 + (float)u32_temp)/((1.0+0.001*(float)BGVAR(u32_LMJR_User))*0.001*(float)PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0)));
            s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;;
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_INITIAL_COST_CAPTURE;
            break;
         }


         s32_TuneSearchTable[0][0] = POS_TUNE_KNLIV_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = s16_PosTune_Knliv_Step;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_ADV_KNLIV_SEARCH;

         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 0; // first generate will include the initial knliv value
         StoreInitialGains(drive);
         TuneSearchInit(drive,0);
         BGVAR(s16_Min_Cost_Found_Cntr) = BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 3;
         if ((BGVAR(u16_HdTune_Adv_Mode) != 0) && (BGVAR(u16_Cycle_Counter_Mask) != 0x3)) 
         {
            BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         else if (BGVAR(u16_HdTune_Adv_Mode) == 0) VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE;// set min settle time flag

         break;

      case POS_TUNE_ADV_KNLIV_SEARCH:
         s16_temp = PosTuneSearch(drive,0,1);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }

         // reduce gain by 10% - if gain is close to effort it might cause AT not to tune the rest of the parameters
         PosTuneWriteParam(POS_TUNE_KNLIV_ID,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0))),drive);
         if (BGVAR(u16_HdTune_Adv_Mode) != 0)
         {
            PosTuneWriteParam(POS_TUNE_KNLP_ID,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0))),drive);
            PosTuneWriteParam(POS_TUNE_KNLI_ID,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0))),drive);
         }
         PositionConfig(drive,1);
         VAR(AX0_s16_Cycle_Bits) |= CYCLE_PARAM_UPDATE;

         BGVAR(s16_Pos_Tune_State) = POS_TUNE_ADV_ANTIVIB_INIT;
         break;

      case POS_TUNE_ADV_ANTIVIB_INIT:
         if (PostuneAntiVibHandler(drive,ATAVHNDLR_MODE_INIT_ONLY) == 0) break; // 0-ongoing 1-done -1-no freq found
         s32_TuneSearchTable[0][0] = POS_TUNE_KNLIV_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = s16_PosTune_Knliv_Step;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_ADV_KNLIV_AFTER_ANTIVIB_INIT;
         if (BGVAR(u16_HdTune_Adv_Mode) == 0)  VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE;// set min settle time flag
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 0;
         StoreInitialGains(drive);
         TuneSearchInit(drive,0);
         BGVAR(s16_Min_Cost_Found_Cntr) = BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 3;

         // skip if no av was done
         if ((BGVAR(s16_HdTune_Av_Mode) <= 0)                                    ||
             ((PosTuneReadParam(POS_TUNE_AV_GAIN3,drive,0) == 0) &&
              (PosTuneReadParam(POS_TUNE_AV_GAIN2,drive,0) == 0)   )               )  s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;


         if ( (s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID) &&
              (BGVAR(u16_RT_Tqf_Control) == 0)                 )BGVAR(u16_RT_Tqf_Control) = 1;
         break;

      case POS_TUNE_ADV_KNLIV_AFTER_ANTIVIB_INIT:
         s16_temp = PosTuneSearch(drive,0,1);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }

         // if knliv after antivib was done ( => av was done and av value was found - take knliv one step back
         if (s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID)
         {
            // reduce gain by 10% - if gain is close to effort it might cause AT not to tune the rest of the parameters
            PosTuneWriteParam(POS_TUNE_KNLIV_ID,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0))),drive);
            if (BGVAR(u16_HdTune_Adv_Mode) != 0)
            {
               PosTuneWriteParam(POS_TUNE_KNLP_ID,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0))),drive);
               PosTuneWriteParam(POS_TUNE_KNLI_ID,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0))),drive);
            }
            PositionConfig(drive,1);
            VAR(AX0_s16_Cycle_Bits) |= CYCLE_PARAM_UPDATE;
         }

         if (VerifyKnlivKnlpcondition(drive) == 1) break;

         // search table
         s32_TuneSearchTable[0][0] = POS_TUNE_KNLP_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = 10;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_ADV_KNLP_SEARCH;
         if ((BGVAR(u16_HdTune_Adv_Mode) != 0) && (BGVAR(u16_Cycle_Counter_Mask) != 0x3)) 
         {
            BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE;// set min settle time flag
         // override knlp knli when knliv knlp knli dependency defined
         if (BGVAR(u16_HdTune_Adv_Mode) != 0)
         {
            s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID; // to shorted execution time - the skip is compelete and not just POS_TUNE_NO_ID
            if (BGVAR(u16_HdTune_Adv_Mode) == 2)
            {
               VAR(AX0_AutoTune_Bits)  &= ~AT_AUTO_TRAJ_ICMD_OV_MASK; // Reset AUTO TRAJ ICMD sat bit
               VAR(AX0_s16_Max_Crrnt_Peak) = 0x0; // Set the minimum current
               BGVAR(u16_Cycle_Cycles_Counter_Prev) = BGVAR(u16_Cycle_Cycles_Counter);
               if (BGVAR(u16_HDTUNE_Sat_Mode) != 0) VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_ADV_SEARCH_4_1;
            }
         }

         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 0; // first generate will include the initial kp value
         StoreInitialGains(drive);
         TuneSearchInit(drive,0);
         BGVAR(s16_Min_Cost_Found_Cntr) = BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 3;
         if ( (s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID) &&
              (BGVAR(u16_RT_Tqf_Control) == 0)                 )BGVAR(u16_RT_Tqf_Control) = 1;
         break;

       case POS_TUNE_ADV_KNLP_SEARCH:
         s16_temp = PosTuneSearch(drive,0,2);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }
         if (VerifyKnlivKnlpcondition(drive) == 1) break;

         s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;
         BGVAR(s16_Pos_Tune_State) = KNLIV_KNLP_SUM_SEARCH;
         if ((PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0) > PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0)) &&
             (BGVAR(u16_HdTune_Adv_Mode) == 0)                                                            )
         {
            s32_TuneSearchTable[0][0] = POS_TUNE_KNLP_ID;
            s32_TuneSearchTable[0][1] = POS_TUNE_KNLIV_ID;
            s32_TuneSearchTable[0][2] = POS_TUNE_NO_ID;
            s16_Tune_Delta_Table[0] = 0;
            s16_Tune_Delta_Table[1] = 0;
            BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
            StoreInitialGains(drive);
            TuneSearchInit(drive,0);
            // if no validation range - do not perform - to shorten AT time
            if (BGVAR(s16_Tune_Table_Size) == 1) s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;

         }
         if ( (s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID) &&
              (BGVAR(u16_RT_Tqf_Control) == 0)                 )BGVAR(u16_RT_Tqf_Control) = 1;
         break;

      case KNLIV_KNLP_SUM_SEARCH:
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }
         /*BGVAR(s16_Pos_Tune_State) = POS_TUNE_AV_FINE_TUNE;
         break;

   case POS_TUNE_AV_FINE_TUNE:
         if (PostuneAntiVibHandler(drive,ATAVHNDLR_MODE_TUNE_ONLY) == 0) break; // 0-ongoing 1-done -1-no freq found

         s32_TuneSearchTable[0][0] = POS_TUNE_KNLUSER_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;//table with one gain
         s16_Tune_Delta_Table[0] = 10;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 2;
         BGVAR(s16_Pos_Tune_State) = KNLUSERGAIN_VALIDITY_CHECK;
         BGVAR(s16_Pos_Tune_Weight) = 95;
         VAR(AX0_s16_Cycle_Bits) &= ~POS_TUNE_MIN_SETTLE_TIME_SCALE;// clr min settle time flag
         StoreInitialGains(drive);
         TuneSearchInit(drive,0);
         if (BGVAR(u16_HdTune_Adv_Mode) != 0) s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;
         break;

      case KNLUSERGAIN_VALIDITY_CHECK:
         // if the initial knliv knlp knli ratio does not match the motor knld could go too high
         // in this test set weight 95% + MST flag and test if reducing usergain reduces cost
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }
         BGVAR(s16_Pos_Tune_Weight) = BGVAR(s16_Pos_Tune_Weight_User);
*/
         VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE;// integral tuning is so with min settle time flag. so is the rest of the tuning
         // search table
         s32_TuneSearchTable[0][0] = POS_TUNE_KNLI_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = 0; // define in generate
         if ((BGVAR(u16_HdTune_Adv_Mode) == 2)                                      ||
             ((BGVAR(u16_HdTune_Adv_Mode) == 0) && (BGVAR(s16_HdTune_Av_Mode) == 0))  ) s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;

         BGVAR(s16_Pos_Tune_State) = POS_TUNE_ADV_KNLI_SEARCH;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         StoreInitialGains(drive);
         TuneSearchInit(drive,0);

         if ( (s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID) &&
              (BGVAR(u16_RT_Tqf_Control) == 0)                 )BGVAR(u16_RT_Tqf_Control) = 1;
         break;

      case POS_TUNE_ADV_KNLI_SEARCH:
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }

         // search table
         s32_TuneSearchTable[0][0] = POS_TUNE_KNLUSER_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;//table with one gain
         s16_Tune_Delta_Table[0] = 10;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 2;
         BGVAR(s16_Pos_Tune_State) = KNLUSERGAIN_VALIDITY_CHECK_2;
         BGVAR(s16_Pos_Tune_Weight) = 95;
         VAR(AX0_s16_Cycle_Bits) &= ~POS_TUNE_MIN_SETTLE_TIME_SCALE;// clr min settle time flag
         StoreInitialGains(drive);
         TuneSearchInit(drive,0);
         if ((BGVAR(u16_HdTune_Adv_Mode) != 0) ||
             ((BGVAR(u16_HdTune_Adv_Mode) == 0) && (BGVAR(s16_HdTune_Av_Mode) == 0))  ) s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;
         break;

      case KNLUSERGAIN_VALIDITY_CHECK_2:
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }
         BGVAR(s16_Pos_Tune_Weight) = BGVAR(s16_Pos_Tune_Weight_User);

         VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE;
         if (BGVAR(s16_Pos_tune_Mode) == 3)  VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_OVERSHOOT_SCALE;

         if (BGVAR(u16_HDTUNE_Sat_Mode) != 0) VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;
         VAR(AX0_AutoTune_Bits)  &= ~AT_AUTO_TRAJ_ICMD_OV_MASK; // Reset AUTO TRAJ ICMD sat bit
         VAR(AX0_s16_Max_Crrnt_Peak) = 0x0; // Set the minimum current
         BGVAR(u16_Cycle_Cycles_Counter_Prev) = BGVAR(u16_Cycle_Cycles_Counter);
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_ADV_SEARCH_4_1;
         break;

      case POS_TUNE_ADV_SEARCH_4_1:
        s16_temp = PosTuneTrajReduceACC(drive);
        if (s16_temp == 0) break; // 0 - on going
        else if (s16_temp == 1)   // 1 - ACC is reduced
        {
           BGVAR(s32_Pos_Tune_Bits) &= ~(POS_TUNE_KNLI_KNLIV_WARN|POS_TUNE_KNLP_KNLIV_WARN);
           VAR(AX0_s16_Cycle_Bits) &= ~POS_TUNE_MIN_SETTLE_TIME_SCALE;//
           BGVAR(s16_Pos_Tune_State) = POS_TUNE_RESTART;
           break;
        }

        CalcAtLimitsEXTTrajectory(drive);

        BGVAR(s16_Pos_Tune_State) = POS_TUNE_PERTUBATION;
        break;

      case POS_TUNE_PERTUBATION:
         if (ATPertubationsHandler(drive , 0) == 1) break;

        // search table
        s32_TuneSearchTable[0][0] = POS_TUNE_NLPEAFF_ID;
        s32_TuneSearchTable[0][1] = POS_TUNE_NLAFFLPFHZ_ID;
        s32_TuneSearchTable[0][2] = POS_TUNE_NO_ID;
        s16_Tune_Delta_Table[0] = 20;
        s16_Tune_Delta_Table[1] = 10;
        BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 3;
        if (BGVAR(s16_Pos_Tune_NlPeAff_Mode) != 0) s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID; // override aff
        StoreInitialGains(drive);
        TuneSearchInit(drive,0);
        BGVAR(s16_Pos_Tune_State) = POS_TUNE_NLPEAFF_SEARCH;
        break;

      case POS_TUNE_NLPEAFF_SEARCH:
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }

         BGVAR(s16_Pos_Tune_State) = POS_TUNE_KNLI_VALIDATION;

         // lower gains to get a softer response - user can increae gain later with the GUI if he wishes
         // if gains are not lowered (adv = 0) , knli validation needed due to usergain validation phase
         if (BGVAR(u16_HdTune_Adv_Mode) != 0)
         {
            PosTuneWriteParam(POS_TUNE_KNLUSER_ID,(long)(0.75*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))),drive);
            // further reduce knliv to reduce oscilations
            if (BGVAR(u16_HdTune_Adv_Mode) == 2) PosTuneWriteParam(POS_TUNE_KNLIV_ID,(long)(0.66*(float)(PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0))),drive);
            PositionConfig(drive,1);
            VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         }
         // since resuction of gain might open a large pe on settling time - validate knli value
         VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE; // set min settle time flag to help AT look at settling time

         s32_TuneSearchTable[0][0] = POS_TUNE_KNLI_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = 0;

         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         StoreInitialGains(drive);
         TuneSearchInit(drive,0);
         // if no validation range - do not perform - to shorten AT time
         if (BGVAR(s16_Tune_Table_Size) == 2) s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;

         if ( (s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID) &&
              (BGVAR(u16_RT_Tqf_Control) == 0)                 )BGVAR(u16_RT_Tqf_Control) = 1;
         break;

      case POS_TUNE_KNLI_VALIDATION:
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }

         // search table
         s32_TuneSearchTable[0][0] = POS_TUNE_NLAFRC_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = 0; // delta is predefined
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 2;
         VAR(AX0_AutoTune_Bits) &= ~AT_NO_ACC_OPTIMIZE_MASK; // clear no acc optimize
         VAR(AX0_s16_Cycle_Bits) &= ~POS_TUNE_MIN_SETTLE_TIME_SCALE;
         if (BGVAR(s16_Pos_Tune_KnlAfrc_Mode) == 0) s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID; // override knlafrc
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_NLAFRC_SEARCH_1;
         StoreInitialGains(drive);
         TuneSearchInit(drive,0);
         break;

      case POS_TUNE_NLAFRC_SEARCH_1:
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }
         if (s32_TuneSearchTable[0][0] != POS_TUNE_NO_ID)
         {
            // if hdtune is express (auto trajectory) than do not fine tune knlafrc - to reduce AT time
            if ( (BGVAR(s16_Pos_tune_Mode) == 1)                                 &&   // hdtune 1
                 ((BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE)         ||          // auto internal traj
                  (BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE_1_CYCLE)  )     &&   // auto internal traj
                 (BGVAR(s16_Pos_Tune_LMJR) == 0)                                   )  // lmjr extiamtion as part of AT
            {
               s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID; // override next knlafrc phase (fine tune phase)
            }
            else
            {
               PosTuneWriteParam(POS_TUNE_NLAFRC_ID,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_NLAFRC_ID,drive,0))),drive);
               PositionConfig(drive,1);
               VAR(AX0_s16_Cycle_Bits) |= CYCLE_PARAM_UPDATE;
            }
         }
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_NLAFRC_SEARCH;
         StoreInitialGains(drive);
         TuneSearchInit(drive,0);
         break;

      case POS_TUNE_NLAFRC_SEARCH:
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,1);
            break;
         }

         if (BGVAR(s16_Pos_Tune_KnlAfrc_Mode) == 2)
         {  // allow acc ff 66% of besst value found - to support cases where lmjr changes
            PosTuneWriteParam(POS_TUNE_NLAFRC_ID,(long)(0.66*(float)(PosTuneReadParam(POS_TUNE_NLAFRC_ID,drive,0))),drive);
            PositionConfig(drive,1);
            VAR(AX0_s16_Cycle_Bits) |= CYCLE_PARAM_UPDATE;
         }

         if (BGVAR(u16_HdTune_Defaults) == 0)
         {
            s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_FINAL_COST_CAPTURE;
         }
         else
         {
            // when in ineternal mode fine tune - set the same path delay used for initial cost est
            if (BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE)
            {
               BGVAR(u16_Path_Delay)[2] = BGVAR(s16_PosTuneTraj_Delay_Int)[1] = 1000;
               // Keep LMJR from zeroing (STOP sets dirflag to 7fff what causes to LMJR=0)
               u32_temp = BGVAR(s32_Cycle_LMJR);
               CycleArrayOverFlow(drive); // restart cycle ident module
            // Restore LMJR
               BGVAR(s32_Cycle_LMJR) = u32_temp;
            }

            if (BGVAR(s16_Pos_tune_Mode) != 2) VAR(AX0_AutoTune_Bits) |= AT_NO_ACC_OPTIMIZE_MASK;
            VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE;// set flag so that initil and final cost with the same flags
            s32_TuneSearchTable[0][0] = POS_TUNE_KNLUSER_ID;
            s32_TuneSearchTable[0][1] = POS_TUNE_KNLUSER_ID;
            s32_TuneSearchTable[0][2] = POS_TUNE_NO_ID;
            s16_Tune_Delta_Table[0] = 0;
            s16_Tune_Delta_Table[1] = 0;
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_FINAL_COST_CAPTURE;
            BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
            TuneSearchInit(drive,0);
         }
         break;

      case POS_TUNE_FINAL_COST_CAPTURE:
         s16_temp = PosTuneSearch(drive,0,0);
         if (s16_temp == 0) break;

         if (BGVAR(u16_HdTune_Defaults) != 0)
         {
            BGVAR(s16_Cost_Improvement) = (int)(100.0 * (1.0-(float)BGVAR(s64_Tune_Max_Cost)/(float)BGVAR(s64_Cost_Before_At)));
            if (BGVAR(s16_Cost_Improvement) < 15)
            {  // mode 2 at could not improve
               BGVAR(s16_Cost_Improvement) = 0;
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_MODE_2_TUNE_FAULT;
               PosTuneRestore(drive,1);
               break;
            }
            else if (BGVAR(s16_Cost_Improvement) > 2000) BGVAR(s16_Cost_Improvement) = 2000;
         }
         BGVAR(s16_Cycle_End_First_Waiting) = 1;  // Use s16_Cycle_End_First_Waiting to be sure we are not at the end of standstill period
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_TRAJ_STOP;
         break;

      case POS_TUNE_TRAJ_STOP:
         // Wait for cycle done
         // Use s16_Cycle_End_First_Waiting to be sure we are not at the end of standstill period
         // Init s16_Cycle_End_First_Waiting to 1 in every pre-POS_TUNE_TRAJ_STOP case
         if((VAR(AX0_s16_StandStill_Counter) == BGVAR(u16_StandStill_Ticks)) && BGVAR(s16_Cycle_End_First_Waiting))
         {
             break;
         }
         else BGVAR(s16_Cycle_End_First_Waiting) = 0;
         // Gentle cycle finishing (except gear/FB trajectory)
         if(BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_IDLE)
         {
             BGVAR(s16_Pos_Tune_Traj_Mode) = PTT_INT_MODE_1_CYCLE;
             // Wait to cycle finish
             if (BGVAR(s16_Pos_Tune_Traj_Enable)) break;
         }

         if(VAR(AX0_s16_StandStill_Counter) != BGVAR(u16_StandStill_Ticks)) break;
         if (BGVAR(s16_Pos_tune_Mode) == 4) // lmjr est only
         {
             // update lmjr
             BGVAR(u32_AT_LMJR) = BGVAR(s32_Cycle_LMJR);
             PosTuneRestore(drive,0);
             PositionConfig(drive,1);
             VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
             // Restore initial mode
             if(BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_IDLE)
             {
                 BGVAR(s16_Pos_Tune_Traj_Mode) = BGVAR(s16_Pos_Tune_Traj_Mode_User); // restore HDTUNEREFMODE
             }
             BGVAR(s16_Pos_Tune_State) = POS_TUNE_DONE;
         }
         else
         {
         PosTuneDone(drive);
         // At this time we assume that we are in begining of the standstill period
         if ((BGVAR(s16_Pos_Tune_Stiffness) == 0) /*&& (BGVAR(s16_Pos_Tune_Traj_Mode) != PTT_IDLE)*/ /*&& ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))*/ )
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_MOVESMOOTH2_MODE_STATE;
            BGVAR(s32_MoveSmoothMode2_Timer) = Cntr_1mS;
         }
         break;

      case POS_TUNE_MOVESMOOTH2_MODE_STATE:
         s32_temp = labs(Cntr_1mS - BGVAR(s32_MoveSmoothMode2_Timer));
         if (s32_temp < 500) break;
         // init move smooth mode 2 filter memories
         VAR(AX0_s16_Cycle_Bits) |= MOVE_SMOOTH_MODE_2_SWITCH;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_SAVE_MODE;
         break;

      case POS_TUNE_CONT_INITIAL_COST_SETUP:
         BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) = 0;
         VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;
         VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE;// set min settle time flag
         s32_TuneSearchTable[0][0] = POS_TUNE_KNLUSER_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;//table with one gain
         s16_Tune_Delta_Table[0] = 0;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         if (BGVAR(u16_Cycle_Counter_Mask) != 0x3)
         {
         BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_INITIAL_COST_CALC;
         TuneSearchInit(drive,0);
         BGVAR(s32_Hdtune_Low_Frq_Osc_Max_Capture) = 0;
         break;

      case POS_TUNE_CONT_INITIAL_COST_CALC:
         s16_temp = PosTuneSearch(drive,10,0);
         if (s16_temp == 0) break;

         // capture inital cost
         if (BGVAR(s64_Cont_Tune_Init_Cost) == 0)
         {
            if (BGVAR(s64_Cont_Tune_Cost_Min) == 0x7FFFFFFFFFFFFFFF)
            {
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_STOPPED;
               break;
            }
            BGVAR(s64_Cont_Tune_Init_Cost) = BGVAR(s64_Cont_Tune_Cost_Min);
            BGVAR(s16_Cont_Tune_Improvement_Cntr) = 0;
         }

         // update effot thresh with 30% above current value
         // change user gain and verify effort is ok
         u32_temp = (unsigned long)(0.3*(float)u32_Cont_Tune_Effort);
         if (EffortToPercent(drive,u32_temp) < 15000) u32_temp = PercentToEffort(drive,15000);

         u32_temp = u32_temp + u32_Cont_Tune_Effort;
         if (EffortToPercent(drive,u32_temp) > BGVAR(s32_Pos_Tune_C_Effort)) u32_temp = PercentToEffort(drive,BGVAR(s32_Pos_Tune_C_Effort));

         LVAR(AX0_u32_At_Control_Effort_Tresh) = u32_temp;

         BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) = (long)(1.3*(float)BGVAR(s32_Hdtune_Low_Frq_Osc_Max_Capture));
         if (BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) > OSC_TRESHOLD) BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) = OSC_TRESHOLD;

         VAR(AX0_s16_Cycle_Bits) |= POS_TUNE_MIN_SETTLE_TIME_SCALE;// set min settle time flag
         s32_TuneSearchTable[0][0] = POS_TUNE_KNLUSER_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;//table with one gain
         s16_Tune_Delta_Table[0] = 0;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         if (BGVAR(u16_Cycle_Counter_Mask) != 0x3)
         {
         BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_EFFORT_VERIFY;
         VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;
         TuneSearchInit(drive,0);
         break;

      case POS_TUNE_CONT_EFFORT_VERIFY:
         s16_temp = PosTuneSearch(drive,10,0);
         if (s16_temp == 0) break;


         if (BGVAR(s64_Cont_Tune_Cost_Min) == 0x7FFFFFFFFFFFFFFF)
         {
            if ((BGVAR(s16_Cont_Tune_Flags) & CONT_TUNE_INC_USERGAIN_MASK) != 0)
            {
               BGVAR(s16_Cont_Tune_Flags) &= ~CONT_TUNE_INC_USERGAIN_MASK;
               BGVAR(s16_Cont_Tune_Flags) |= CONT_TUNE_DEC_USERGAIN_MASK;
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_INITIAL_COST_SETUP;
               ++BGVAR(s16_Cont_Tune_Improvement_Cntr);
               if (BGVAR(s16_Cont_Tune_Improvement_Cntr) > 3) BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_STOPPED;
               break;
            }
            else
            {
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_STOPPED;
               break;
            }
         }

         BGVAR(s64_Cont_Tune_Before_Param_Init) = BGVAR(s64_Cont_Tune_Cost_Min);


         if ((BGVAR(s16_Cont_Tune_Flags) & CONT_TUNE_INC_USERGAIN_MASK) != 0)
            PosTuneWriteParam(POS_TUNE_KNLUSER_ID,(long)(1.1*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))),drive);
         else if ((BGVAR(s16_Cont_Tune_Flags) & CONT_TUNE_DEC_USERGAIN_MASK) != 0)
            PosTuneWriteParam(POS_TUNE_KNLUSER_ID,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))),drive);


         PositionConfig(drive,1);
         VAR(AX0_s16_Cycle_Bits) |= CYCLE_PARAM_UPDATE;

         VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_AV3_DEC_TUNE;
         s32_TuneSearchTable[0][0] = POS_TUNE_AV_GAIN3;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = 0;
         if (BGVAR(u16_Cycle_Counter_Mask) != 0x3)
         {
         BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         TuneSearchInit(drive,0);
         cont_tune_osc_flag = 0;
         break;

    case POS_TUNE_CONT_AV3_DEC_TUNE:
         if (cont_tune_osc_flag == 0)
         {
            cont_tune_osc_flag = 1;
            BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) = (long)(1.2*(float)BGVAR(s32_Hdtune_Low_Frq_Osc_Max_Capture));
            if (BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) > OSC_TRESHOLD) BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) = OSC_TRESHOLD;
         }


         s16_temp = BGVAR(s16_HdTune_Av_Mode);
         if (s16_temp > 3) s16_temp-=3;
         if (s16_temp >= 2)
         {
            s16_temp = PosTuneSearch(drive,10,0);
            if (s16_temp == 0) break;
         }

         BGVAR(s64_Cont_Tune_Param_Dec_Max) = BGVAR(s64_Cont_Tune_Cost_Max);
         BGVAR(s64_Cont_Tune_Param_Dec_Min) = BGVAR(s64_Cont_Tune_Cost_Min);

         BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_AV3_INC_TUNE;
         if (BGVAR(u16_Cycle_Counter_Mask) != 0x3)
         {
         BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;
         TuneSearchInit(drive,0);
         break;

    case POS_TUNE_CONT_AV3_INC_TUNE:
         s16_temp = BGVAR(s16_HdTune_Av_Mode);
         if (s16_temp > 3) s16_temp-=3;

         if (s16_temp >= 2)
         {
            s16_temp = PosTuneSearch(drive,10,0);
            if (s16_temp == 0) break;
            VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_AV3_DEC_TUNE;

            if ((BGVAR(s64_Cont_Tune_Cost_Max) < BGVAR(s64_Cont_Tune_Param_Dec_Min))       &&
                (BGVAR(s64_Cont_Tune_Cost_Max) < BGVAR(s64_Cont_Tune_Before_Param_Init))     )
            {
               PosTuneWriteParam(POS_TUNE_AV_GAIN3,(long)(1.1*(float)(PosTuneReadParam(POS_TUNE_AV_GAIN3,drive,0))),drive); // knlp inc is the best
               BGVAR(s64_Cont_Tune_Before_Param_Init) = BGVAR(s64_Cont_Tune_Cost_Min);
            }
            else if ((BGVAR(s64_Cont_Tune_Param_Dec_Max) < BGVAR(s64_Cont_Tune_Cost_Min))          &&
                     (BGVAR(s64_Cont_Tune_Param_Dec_Max) < BGVAR(s64_Cont_Tune_Before_Param_Init))    )
            {
               PosTuneWriteParam(POS_TUNE_AV_GAIN3,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_AV_GAIN3,drive,0))),drive); // knlp dec is the best
               BGVAR(s64_Cont_Tune_Before_Param_Init) = BGVAR(s64_Cont_Tune_Param_Dec_Min);
            }
            else //origianl was the best - finish stage
            {
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_AV2_DEC_TUNE;
               s32_TuneSearchTable[0][0] = POS_TUNE_AV_GAIN2;
               s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
            }
         }
         else
         {
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_AV2_DEC_TUNE;
               s32_TuneSearchTable[0][0] = POS_TUNE_AV_GAIN2;
               s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         }

         PositionConfig(drive,1);
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         if (BGVAR(u16_Cycle_Counter_Mask) != 0x3)
         {
         BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         TuneSearchInit(drive,0);
         break;

    case POS_TUNE_CONT_AV2_DEC_TUNE:
         s16_temp = BGVAR(s16_HdTune_Av_Mode);
         if (s16_temp > 3) s16_temp-=3;

         if (s16_temp >= 1)
         {
            s16_temp = PosTuneSearch(drive,10,0);
            if (s16_temp == 0) break;
            VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;
         }

         BGVAR(s64_Cont_Tune_Param_Dec_Max) = BGVAR(s64_Cont_Tune_Cost_Max);
         BGVAR(s64_Cont_Tune_Param_Dec_Min) = BGVAR(s64_Cont_Tune_Cost_Min);

         BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_AV2_INC_TUNE;
         if (BGVAR(u16_Cycle_Counter_Mask) != 0x3)
         {
         BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         TuneSearchInit(drive,0);
         break;

    case POS_TUNE_CONT_AV2_INC_TUNE:
         s16_temp = BGVAR(s16_HdTune_Av_Mode);
         if (s16_temp > 3) s16_temp-=3;

         if (s16_temp >= 1)
         {
            s16_temp = PosTuneSearch(drive,10,0);
            if (s16_temp == 0) break;
            VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;

            BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_AV2_DEC_TUNE;

            if ((BGVAR(s64_Cont_Tune_Cost_Max) < BGVAR(s64_Cont_Tune_Param_Dec_Min))       &&
                (BGVAR(s64_Cont_Tune_Cost_Max) < BGVAR(s64_Cont_Tune_Before_Param_Init))     )
            {
               PosTuneWriteParam(POS_TUNE_AV_GAIN2,(long)(1.1*(float)(PosTuneReadParam(POS_TUNE_AV_GAIN2,drive,0))),drive); // knlp inc is the best
               BGVAR(s64_Cont_Tune_Before_Param_Init) = BGVAR(s64_Cont_Tune_Cost_Min);
            }
            else if ((BGVAR(s64_Cont_Tune_Param_Dec_Max) < BGVAR(s64_Cont_Tune_Cost_Min))          &&
                     (BGVAR(s64_Cont_Tune_Param_Dec_Max) < BGVAR(s64_Cont_Tune_Before_Param_Init))    )
            {
               PosTuneWriteParam(POS_TUNE_AV_GAIN2,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_AV_GAIN2,drive,0))),drive); // knlp dec is the best
               BGVAR(s64_Cont_Tune_Before_Param_Init) = BGVAR(s64_Cont_Tune_Param_Dec_Min);
            }
            else //origianl was the best - finish stage
            {
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_DEC_KNLP_TUNE;
               s32_TuneSearchTable[0][0] = POS_TUNE_KNLP_ID;
               s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
            }
         }
         else
         {
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_DEC_KNLP_TUNE;
               s32_TuneSearchTable[0][0] = POS_TUNE_KNLP_ID;
               s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         }

         PositionConfig(drive,1);
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         //while ((VAR(AX0_s16_Crrnt_Run_Code) & (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK)) != 0);
         if (BGVAR(u16_Cycle_Counter_Mask) != 0x3)
         {
         BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         TuneSearchInit(drive,0);
         break;

      case POS_TUNE_CONT_DEC_KNLP_TUNE:
         s16_temp = PosTuneSearch(drive,10,0);
         if (s16_temp == 0) break;
         VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;

         BGVAR(s64_Cont_Tune_Param_Dec_Max) = BGVAR(s64_Cont_Tune_Cost_Max);
         BGVAR(s64_Cont_Tune_Param_Dec_Min) = BGVAR(s64_Cont_Tune_Cost_Min);

         BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_INC_KNLP_TUNE;
         if (BGVAR(u16_Cycle_Counter_Mask) != 0x3)
         {
         BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         TuneSearchInit(drive,0);
         break;

      case POS_TUNE_CONT_INC_KNLP_TUNE:
         s16_temp = PosTuneSearch(drive,10,0);
         if (s16_temp == 0) break;
         VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;

         BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_DEC_KNLP_TUNE;

         if ((BGVAR(s64_Cont_Tune_Cost_Max) < BGVAR(s64_Cont_Tune_Param_Dec_Min))       &&
             (BGVAR(s64_Cont_Tune_Cost_Max) < BGVAR(s64_Cont_Tune_Before_Param_Init))     )
         {
            PosTuneWriteParam(POS_TUNE_KNLP_ID,(long)(1.1*(float)(PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0))),drive); // knlp inc is the best
            BGVAR(s64_Cont_Tune_Before_Param_Init) = BGVAR(s64_Cont_Tune_Cost_Min);
         }
         else if ((BGVAR(s64_Cont_Tune_Param_Dec_Max) < BGVAR(s64_Cont_Tune_Cost_Min))          &&
                  (BGVAR(s64_Cont_Tune_Param_Dec_Max) < BGVAR(s64_Cont_Tune_Before_Param_Init))    )
         {
            PosTuneWriteParam(POS_TUNE_KNLP_ID,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0))),drive); // knlp dec is the best
            BGVAR(s64_Cont_Tune_Before_Param_Init) = BGVAR(s64_Cont_Tune_Param_Dec_Min);
         }
         else //origianl was the best - finish stage
         {
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_KNLI_DEC_TUNE;
            s32_TuneSearchTable[0][0] = POS_TUNE_KNLI_ID;
            s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         }

         PositionConfig(drive,1);
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         //while ((VAR(AX0_s16_Crrnt_Run_Code) & (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK)) != 0);
         s16_Tune_Delta_Table[0] = 0;
         if (BGVAR(u16_Cycle_Counter_Mask) != 0x3)
         {
         BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         TuneSearchInit(drive,0);
         break;

      case POS_TUNE_KNLI_DEC_TUNE:
         s16_temp = PosTuneSearch(drive,10,0);
         if (s16_temp == 0) break;
         VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;

         BGVAR(s64_Cont_Tune_Param_Dec_Max) = BGVAR(s64_Cont_Tune_Cost_Max);
         BGVAR(s64_Cont_Tune_Param_Dec_Min) = BGVAR(s64_Cont_Tune_Cost_Min);

         BGVAR(s16_Pos_Tune_State) = POS_TUNE_KNLI_INC_TUNE;
         if (BGVAR(u16_Cycle_Counter_Mask) != 0x3)
         {
         BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         TuneSearchInit(drive,0);
         break;

      case POS_TUNE_KNLI_INC_TUNE:
         s16_temp = PosTuneSearch(drive,10,0);
         if (s16_temp == 0) break;
         VAR(AX0_AutoTune_Bits) &=~AT_ICMD_OV_MASK;

         BGVAR(s16_Pos_Tune_State) = POS_TUNE_KNLI_DEC_TUNE;

         // determine which result is better
         if ((BGVAR(s64_Cont_Tune_Cost_Max) < BGVAR(s64_Cont_Tune_Param_Dec_Min)) &&
             (BGVAR(s64_Cont_Tune_Cost_Max) < BGVAR(s64_Cont_Tune_Before_Param_Init))  )
         {
            PosTuneWriteParam(POS_TUNE_KNLI_ID,(long)(1.1*(float)(PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0))),drive); // knlp inc is the best
            BGVAR(s64_Cont_Tune_Before_Param_Init) = BGVAR(s64_Cont_Tune_Cost_Min);
         }
         else if ((BGVAR(s64_Cont_Tune_Param_Dec_Max) < BGVAR(s64_Cont_Tune_Cost_Min))                &&
                  (BGVAR(s64_Cont_Tune_Param_Dec_Max) < BGVAR(s64_Cont_Tune_Before_Param_Init))    )
         {
            PosTuneWriteParam(POS_TUNE_KNLI_ID,(long)(0.9*(float)(PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0))),drive); // knlp dec is the best
            BGVAR(s64_Cont_Tune_Before_Param_Init) = BGVAR(s64_Cont_Tune_Param_Dec_Min);
         }
         else //origianl was the best - finish stage
         {
            // compare result to the one before tuning started
            if (BGVAR(s64_Cont_Tune_Before_Param_Init) < BGVAR(s64_Cont_Tune_Init_Cost))
            {
               PosTuneBackup(drive);
               BGVAR(s64_Cont_Tune_Init_Cost) = BGVAR(s64_Cont_Tune_Before_Param_Init);
               BGVAR(s16_Cont_Tune_Improvement_Cntr) = 0;
            }
            else
            { // change knlusergain direction
               ++BGVAR(s16_Cont_Tune_Improvement_Cntr);
               if (BGVAR(s16_Cont_Tune_Improvement_Cntr) > 3)
               {
                  BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_STOPPED;
                  break;
               }

               if ((BGVAR(s16_Cont_Tune_Flags) & CONT_TUNE_INC_USERGAIN_MASK) != 0)
               {
                  BGVAR(s16_Cont_Tune_Flags) &= ~CONT_TUNE_INC_USERGAIN_MASK;
                  BGVAR(s16_Cont_Tune_Flags) |= CONT_TUNE_DEC_USERGAIN_MASK;
               }
               else
               {
                  BGVAR(s16_Cont_Tune_Flags) |= CONT_TUNE_INC_USERGAIN_MASK;
                  BGVAR(s16_Cont_Tune_Flags) &= ~CONT_TUNE_DEC_USERGAIN_MASK;
               }
            }
            PosTuneRestore(drive,0); //Restore param from backup
            PositionConfig(drive,1);
            VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_INITIAL_COST_SETUP;
            break;
         }

         PositionConfig(drive,1);
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         //while ((VAR(AX0_s16_Crrnt_Run_Code) & (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK)) != 0);

         s16_Tune_Delta_Table[0] = 0;
         if (BGVAR(u16_Cycle_Counter_Mask) != 0x3)
         {
         BGVAR(u16_Cycle_Counter_Mask) = 0x3;
            CycleArrayOverFlow(drive); // restart cycle ident module
         }
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
         TuneSearchInit(drive,0);
         break;

      case POS_TUNE_CONT_STOPPED:
         PosTuneRestore(drive,0); //Restore param from backup
         PositionConfig(drive,1);
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_DONE;
         break;


      case POS_TUNE_AVHZ_TEST_MODE:
         if (AutoTuneAntiVibWithPSD(drive,3,0) != 1) break;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_DONE;
         break;

      case POS_TUNE_SAVE_MODE:
         // if mode 0 than keep hdtune result to ram variables, and Overide backup
         // if mode 1 than terminate hdtune and trough away hdtune results
         // if mode 2 Do Nothing, In local HMI wait for user to click OK or Cancel
         // if mode 3 than keep like mode 0, but leave backup avilable for restore later by hdtune=0.
         // can add here more modes - to save to flash etc
         switch (u16_HDTUNE_Save_Mode)
         {
            case HDTUNESAVE_KEEP_RES_OVRRIDE_BKUP://Keep results, overide backup
               PosTuneBackup(drive); //Overide backup with new values
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_DONE;
               break;

            case HDTUNESAVE_MODE_IGNORE:  //Ignore, use backup
               if (s_PosTuneBackup.movesmoothmode != 2)
            {
               VAR(AX0_u16_Move_Smooth_Source) = 0xF; // always use move smooth filter
               MoveSmoothModeCommand(0LL,drive);      // move smooth moving lpfhz - done in two stages to avoid jump
               WaitForNext32kHzTaskStart();
               }
               PosTuneRestore(drive,0); //Restore param from backup
               PositionConfig(drive,1);
               VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_DONE;
               break;

            case HDTUNESAVE_MODE_WAIT: // Do Nothing, In local HMI wait for user to click OK or Cancel
               break;

            case HDTUNESAVE_KEEP_RES_KEEP_BKUP:
               BGVAR(s16_Pos_Tune_State) = POS_TUNE_DONE;
               s16_Allow_Restore_Param = 1;
               break;

         }
         break;

      case POS_TUNE_DONE:
         break;
   }
}


int ATPertubationsHandler(int drive,int reset)
{
   // set up recorder to record on IGRAV change
   // wait till acc has finished and than issue jerk via Igrav equal to micont
   // get the recorded data and analyze with the matlab simulation to find the resonance freqeuncy
   static int at_pertubations_state = 0;
   static int i_grav_backup;
   static unsigned int u16_cycle_capture;
   int s16_temp;
   // AXIS_OFF;

   if (reset == 1)
   {
      at_pertubations_state = 0;
      return 0;
   }

   switch (at_pertubations_state)
   {
      case 0:
         ////-----------debug debug debug
         // disable pertubations till it is finilized
         at_pertubations_state = 0;
         return 0;
         ////-----------debug debug debug

         // zero igrav
         //i_grav_backup = VAR(AX0_s16_Igrav);
         //VAR(AX0_s16_Igrav) = 0;
         //u16_cycle_capture = BGVAR(u16_Cycle_Cycles_Counter)+2;
         //at_pertubations_state++;
         //break;

      case 1:
         if (u16_cycle_capture > BGVAR(u16_Cycle_Cycles_Counter)) break;
         BGVAR(s32_Hdtune_Low_Frq_Osc_Max_Capture) = 0;
         BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) = 0;
         u16_cycle_capture = BGVAR(u16_Cycle_Cycles_Counter)+2;
         at_pertubations_state++;
         break;

      case 2:
         if (u16_cycle_capture > BGVAR(u16_Cycle_Cycles_Counter)) break;
         BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) = (long)(1.2*(float)BGVAR(s32_Hdtune_Low_Frq_Osc_Max_Capture));
         if (BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) > OSC_TRESHOLD) BGVAR(s32_Hdtune_Low_Frq_Osc_ContTune_Lim) = OSC_TRESHOLD;

         BGVAR(u16_At_Pretubation_En) = 1; // ask AT to inject pertubation

         s32_TuneSearchTable[0][0] = POS_TUNE_KNLUSER_ID;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;//table with one gain
         s16_Tune_Delta_Table[0] = 0;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 2;
         s16_Pos_Tune_Verify_Limit = 5;
         TuneSearchInit(drive,0);

         s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;
         at_pertubations_state++;
         break;

      case 3:
         s16_temp = PosTuneSearch(drive,2,0);
         if (s16_temp == 0) break;
         else if (s16_temp == -1)
         {
            TuneSearchInit(drive,0);
            break;
         }
         TuneSearchInit(drive,0);

         BGVAR(u16_At_Pretubation_En) = 0;
         at_pertubations_state = 0;
         VAR(AX0_s16_Igrav) = i_grav_backup;
         return 0;
         //break;
   }
   return 1;
}

void CalcAtLimitsATTrajectoryGenerator(int drive)
{
   unsigned long long u64_at_min_acc_int, u64_at_max_acc_int;
   float f_nom_acc,f_max_acc,f_j_tot;
   long s32_lmjr;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //Check Acc Dec limits, only for User defined traj
   if (BGVAR(s16_Pos_Tune_Traj_Mode_User) != PTT_EXT_MODE) return;

   // calc nominal and max acc
   if (BGVAR(s16_Pos_Tune_LMJR) == 1)
      s32_lmjr = BGVAR(u32_AT_LMJR);
   else
      s32_lmjr = BGVAR(s32_Cycle_LMJR);

   f_j_tot = 1e-6*(float)BGVAR(s32_Motor_J)*(1 + 0.001*(float)s32_lmjr);
   f_nom_acc = 1e-6*(float)BGVAR(u32_Motor_Kt)*(float)BGVAR(s32_Motor_I_Cont)/f_j_tot;// rad/sec^2
   f_max_acc = 1e-6*(float)BGVAR(u32_Motor_Kt)*(float)BGVAR(s32_Motor_I_Peak)/f_j_tot;// rad/sec^2
   // Tsp correction
   if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) f_nom_acc = f_nom_acc * 4;
   u64_at_min_acc_int = (unsigned long long)((0.3 * f_nom_acc)*45873289113.78 );//45873289113=2^32*2^32/8000/8000/2/pi
   u64_at_max_acc_int = (unsigned long long)((0.9 * f_max_acc)*45873289113.78 );//45873289113=2^32*2^32/8000/8000/2/pi

   
   // acc / dec / vcruise is given by trajectry generator
   //Acc
   if(BGVAR(u64_PosTuneTraj_AccDec)[0] < u64_at_min_acc_int)
      BGVAR(s32_Pos_Tune_Bits) |= AT_TRAJ_BELOW_ACC_MIN_MASK;

   if(BGVAR(u64_PosTuneTraj_AccDec)[0] > u64_at_max_acc_int)
      BGVAR(s32_Pos_Tune_Bits) |= AT_TRAJ_ABOVE_ACC_MAX_MASK;

   //Dec
   if(BGVAR(u64_PosTuneTraj_AccDec)[1] < u64_at_min_acc_int)
      BGVAR(s32_Pos_Tune_Bits) |= AT_TRAJ_BELOW_ACC_MIN_MASK;

   if(BGVAR(u64_PosTuneTraj_AccDec)[1] > u64_at_max_acc_int)
      BGVAR(s32_Pos_Tune_Bits) |= AT_TRAJ_ABOVE_ACC_MAX_MASK;
   
}

void CalcAtLimitsEXTTrajectory(int drive)
{
   unsigned long long u64_at_min_acc_int, u64_at_max_acc_int,u64_actual_acc;
   float f_nom_acc,f_max_acc,f_j_tot;
   long s32_min_spd_lim,s32_lmjr;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //Check Acc Dec limits, only for ext traj
   if (BGVAR(s16_Pos_Tune_Traj_Mode_User) != PTT_IDLE) return;

   // calc nominal and max acc
   if (BGVAR(s16_Pos_Tune_LMJR) == 1)
      s32_lmjr = BGVAR(u32_AT_LMJR);
   else
      s32_lmjr = BGVAR(s32_Cycle_LMJR);

   f_j_tot = 1e-6*(float)BGVAR(s32_Motor_J)*(1 + 0.001*(float)s32_lmjr);
   f_nom_acc = 1e-6*(float)BGVAR(u32_Motor_Kt)*(float)BGVAR(s32_Motor_I_Cont)/f_j_tot;// rad/sec^2
   f_max_acc = 1e-6*(float)BGVAR(u32_Motor_Kt)*(float)BGVAR(s32_Motor_I_Peak)/f_j_tot;// rad/sec^2
   // Tsp correction
   if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) f_nom_acc = f_nom_acc * 4;
   u64_at_min_acc_int = (unsigned long long)((0.3 * f_nom_acc)*45873289113.78 );//45873289113=2^32*2^32/8000/8000/2/pi
   u64_at_max_acc_int = (unsigned long long)((0.9 * f_max_acc)*45873289113.78 );//45873289113=2^32*2^32/8000/8000/2/pi

   s32_min_spd_lim = (long)((float)BGVAR(u32_Mspeed) * 0.1);
   if (BGVAR(s32_Capture_Ptpvcmd) < s32_min_spd_lim)  BGVAR(s32_Pos_Tune_Bits) |= AT_TRAJ_LO_SPEED_MASK;

   // f_Avg_Acc is deltaV[internal units]/delta_integral
   // Tsp = 250uS
   // conversion factor 32000/22750*vlim_rpm/60*2^32*2^32/4000/4000 = 27028196444.995680023443223443223*vlim_rpm  = (float)BGVAR(s32_V_Lim_Design)*3020636.34
   // 
   // Tsp = 125uS
   // conversion factor 32000/22750*vlim_rpm/60*2^32*2^32/8000/8000 = 6757049111.24892*vlim_rpm  = (float)BGVAR(s32_V_Lim_Design)*755159.085
  
   u64_actual_acc = (long long)(BGVAR(f_Avg_Acc) * (float)BGVAR(s32_V_Lim_Design)*755159.085);
   // Tsp correction
   // 3020636.34 = 4*755159.085
   if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) u64_actual_acc = u64_actual_acc << 2;  
   //       
   // if pwm 8 than mpy by 2
   if (BGVAR(u32_Pwm_Freq) == 8000L)      u64_actual_acc = u64_actual_acc << 1;
   // if pwm 4 than mpy by 4
   else if (BGVAR(u32_Pwm_Freq) == 4000L) u64_actual_acc = u64_actual_acc << 2;

   //Acc
   if(u64_actual_acc < u64_at_min_acc_int)
      BGVAR(s32_Pos_Tune_Bits) |= AT_TRAJ_BELOW_ACC_MIN_MASK;

   if(u64_actual_acc > u64_at_max_acc_int)
      BGVAR(s32_Pos_Tune_Bits) |= AT_TRAJ_ABOVE_ACC_MAX_MASK;

}




void ResetPosGainsForPosTune(int drive)
{
   int s16_temp = 0,s16_at_done_flag = 0;
   float f_kiv_kp_factor,f_temp;
   long s32_temp;
   // AXIS_OFF;

   SetTorqueFilterAccordingToMotor(drive,1); // set kp ki factor

   if (BGVAR(s16_HdTune_Av_Mode) >= 0)
   {
      SalNlKAntiVibrationCommand(0LL,drive); // zero av
      SalNlPeGainCommand(0LL,drive);         // zero av2
      SalNlPe3GainCommand(0LL,drive);         // zero av3

      SalNlPeSharpnessCommand((long long)1000*AT_AV_SHARP_FOR_POSTUNE2,drive);
      SalNlPe3SharpnessCommand((long long)1000*AT_AV_SHARP_FOR_POSTUNE3,drive);
      SalNlPe3QGainCommand(1000LL,drive);
   }

   if ( (BGVAR(u16_HdTune_Defaults) == 1)                    &&
        ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))  )
   {
      // reduce usergain by 0.25
      PosTuneWriteParam(POS_TUNE_KNLUSER_ID,(long)(0.75*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))),drive);
      PositionConfig(drive,1);
      VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);

      f_kiv_kp_factor = (float)PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0)/(float)PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0);
      if (f_kiv_kp_factor < BGVAR(f_Kp_Kiv_Ratio))
         f_temp = f_kiv_kp_factor / BGVAR(f_Kp_Kiv_Ratio);
      else
         f_temp = BGVAR(f_Kp_Kiv_Ratio) / f_kiv_kp_factor;

      if (f_temp < 0.975)
      {
         s16_at_done_flag = 0;
         f_kiv_kp_factor = f_kiv_kp_factor*0.66;
         if (f_kiv_kp_factor < BGVAR(f_Kp_Kiv_Ratio))
            f_temp = f_kiv_kp_factor / BGVAR(f_Kp_Kiv_Ratio);
         else
            f_temp = BGVAR(f_Kp_Kiv_Ratio) / f_kiv_kp_factor;
         if (f_temp > 0.975)
         {
            s16_at_done_flag = 2;
            // mpy knliv by 1/0.666 = 1.5151 - to restore knliv after the end of easy tuning
            PosTuneWriteParam(POS_TUNE_KNLIV_ID,(long)(1.5151*(float)(PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0))),drive);
            PositionConfig(drive,1);
            VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         }
      }
      else
         s16_at_done_flag = 1;

      if (s16_at_done_flag == 0) BGVAR(s32_Pos_Tune_Bits) |= INITIAL_RESULTS_MISMATCH_MASK;

      // load knlp and knli according to knliv

      // reduce all gains by 0.8
      s32_temp = (long)(0.8 * (float)PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0));
      PosTuneWriteParam(POS_TUNE_KNLIV_ID,s32_temp,drive);
      s32_temp = (long)(BGVAR(f_Kp_Kiv_Ratio)*(float)s32_temp);
      PosTuneWriteParam(POS_TUNE_KNLP_ID,s32_temp,drive);
      s32_temp = (long)(BGVAR(f_Ki_Kp_Ratio)*(float)s32_temp);
      PosTuneWriteParam(POS_TUNE_KNLI_ID,s32_temp,drive);
      PositionConfig(drive,1);
      VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
      return;
   }


   SalNlKpgfCommand((long)((float)BGVAR(u16_HdTune_Gain) * 0.25),drive);

   BGVAR(u32_AT_LMJR) = BGVAR(u32_LMJR_User); // backup lmjr to support hdtunelmjr 1
   SalLmjrCommand(1000LL,drive);
   SalNlKffSpringCommand(5000000,drive);  // nlpeaff = 1000 - has little

   SetStiffness(drive);
   SetTorqueFilterAccordingToMotor(drive,1); // set kp ki factor
   CalculateNLParameters(drive,0); // set defaults with lmjr=1
   SalNlNotchCenterCommand(1333LL,drive); // set nlnotch
   SalNlNotchBWCommand(300LL,drive);
   if (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5)
   {
      s16_temp = RowInEasyTuningEntries();
      if ((s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17950013) || (s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17951013))    // 1301-1

      {
         SalNlNotchCenterCommand(1000LL,drive); // set nlnotch
      }
      if ((s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17960025) || (s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17961025) ||      // 1804-1
          (s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17950017) || (s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17951017)   )  // 1306-2
      {
         SalNlNotchCenterCommand(885LL,drive); // set nlnotch
      }
      if ((s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17960026) || (s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17961026))      // 1805-1
      {
         SalNlNotchCenterCommand(715LL,drive); // set nlnotch
      }  
      if ((s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17950015) || (s_Easy_Tuning_Data[s16_temp].motor_file_name_se == 17951015))     // 1305-1
      {
         SalNlNotchCenterCommand(1100LL,drive); // set nlnotch
      }      
   }
   SalNAdaptiveVcmdGainfCommand(0LL,drive);
   SalKnlafrcCommand(0LL,drive);

   // set MENCRESSHR
   // for encoder - use real bits and no inetrpolation
   // for sine encoder use only 8 bits of interpolation
   // for senrvo sense use 9 bits
   // for resolver use 14 bits
   if (BGVAR(s16_Pos_Tune_Mencres_Shr_Mode) == 0)
   {
      if (FEEDBACK_SERVOSENSE) s16_temp = 8;
      else if (BGVAR(u16_FdbkType) == RESOLVER_FDBK) s16_temp = 2;
      else if (BGVAR(u16_FdbkType) == SW_SINE_FDBK) s16_temp = 6;
      else if (BGVAR(u16_FdbkType) == INC_ENC_FDBK) s16_temp = 0;
      else
      {
         s16_temp = 0;
         s32_temp = LVAR(AX0_s32_Counts_Per_Rev);
         while (s32_temp > 8192)
         {
            s32_temp >>=1;
            ++s16_temp;
         }
      }
      SalMencResShrCommand((long long)s16_temp,drive);
   }

   // if filter is set by hdtune determine initial value
   if (SetTorqueFilterAccordingToMotor(drive,2) != 1) BGVAR(s32_Pos_Tune_Bits) |= UNKNOWN_MOTOR_ID_MASK;
   VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK); // dont wait for cycle detection - imm update
   VAR(AX0_s16_Igrav) = 0;
}



int VerifyKnlivKnlpcondition(int drive)
{ // force kniv > knlp
   // AXIS_OFF;
   if ( (PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0) > PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0))     &&
        ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_KNLP_KNLIV_WARN) == 0)                                   &&
        (BGVAR(u16_HdTune_Adv_Mode) == 0)                                                                )
   {
      BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_KNLP_KNLIV_WARN;

      // reduce knlp
      PosTuneWriteParam(POS_TUNE_KNLP_ID,PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0)>>1,drive);
      // if knlp is still higher than knliv than set knliv = knlp
      if (PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0) > PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0))
         PosTuneWriteParam(POS_TUNE_KNLIV_ID,PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0),drive);
      else
         PosTuneWriteParam(POS_TUNE_KNLIV_ID,(long)(1.1*(float)PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0)),drive);

      PositionConfig(drive,1);
      VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);

      if (BGVAR(s16_HdTune_Av_Mode) >= 0)
      {
         SalNlKAntiVibrationCommand(0LL,drive); // zero av
         SalNlPeGainCommand(0LL,drive);         // zero av
         SalNlPe3GainCommand(0LL,drive);         // zero av
      }

      VAR(AX0_s16_Cycle_Bits) &= ~POS_TUNE_MIN_SETTLE_TIME_SCALE;
      // search table
      s32_TuneSearchTable[0][0] = POS_TUNE_NO_ID;
      BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 4;
      TuneSearchInit(drive,0);
      BGVAR(s16_Pos_Tune_State) = POS_TUNE_KNLUSER_VERIFY;
      return 1;
   }
   return 0;
}

void PosTuneDone(int drive)
{
   AutoTuneGainNormalization(drive);
   PosTuneTrajZeroingParam(drive);
   BGVAR(s16_Pos_Tune_State) = POS_TUNE_SAVE_MODE;
}

void SetStiffness(int drive)
{
   // AXIS_OFF;
   if (BGVAR(s16_Pos_Tune_Stiffness) == 0)
   {
      VAR(AX0_u16_Move_Smooth_Source) = 0xF; // always use move smooth filter
      MoveSmoothModeCommand(0LL,drive);      // move smooth moving lpfhz - done in two stages to avoid jump
      if (VAR(AX0_s16_Opmode) == 4)
          SalGearFiltModeCommand(1LL,drive);     // enable gearfiltmode
      else
          SalGearFiltModeCommand(0LL,drive);     // disable gearfiltmode
      WaitForNext32kHzTaskStart();
      MoveSmoothModeCommand(1LL,drive);      // move smooth moving lpfhz - done in two stages to avoid jump
      SalGearFiltVffCommand(0LL,drive);
      SalGearFiltAffCommand(0LL,drive);
   }
}


int PosTuneAvInitSearch(int drive)
{
   long long s64_final_cost,s64_temp;
   long s32_temp;
   static int s16_first_run_flag = 1;
   int s16_temp;
   // AXIS_OFF;

   do {
   s16_temp = Cntr_3125;
   s64_temp = LLVAR(AX0_u32_At_Final_Cost_Lo);
   } while (s16_temp != Cntr_3125);

   // no need to do anything here - r.t kills av gains till next cycle
   //if (s64_temp == 0x7FFFFFFFFFFFFFFF)
   //{ // => effort function maxed out - best initial param found.
   //   if ( (BGVAR(u32_At_Control_Effort_Max_Captured) > LVAR(AX0_u32_At_Control_Effort_Tresh)) &&
   //        (s16_Extreemly_High_Effort == 0)                           )
   //   {
   //      s16_Extreemly_High_Effort = 1;
   //      s32_High_Effort_User_Gain = PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0);
   //      PosTuneWriteParam(POS_TUNE_KNLUSER_ID,(long)(0.5*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))),drive);
   //      PositionConfig(drive,1);
   //      VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
   //      BGVAR(s32_Pos_Tune_Bits) |=EXTREME_EFFORT_MASK_AV_INIT_MASK;
   //   }
   //}

   // test if new result is ready
   if ((BGVAR(s16_Pos_Tune_Cycle) <= 3) && ((VAR(AX0_s16_Cycle_Bits) & CYCLE_NEW_CYCLE_DATA_READY) != 0))
   {
      s64_final_cost = BGVAR(s64_Cost_Function_Final_Capture);
      BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_DATA_READY_MASK;
      VAR(AX0_s16_Cycle_Bits) &= ~(CYCLE_NEW_CYCLE_DATA_READY);
   }

   // test timer/cycle indication is done. if not exit
   if ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_DATA_READY_MASK) == 0) return 0;
   BGVAR(s32_Pos_Tune_Bits) &= ~POS_TUNE_DATA_READY_MASK;

   if (s16_first_run_flag == 1)
   {  // one cycle makes sure weight and cost are in sync
      s16_first_run_flag = 0;
      BGVAR(s32_Pos_Tune_Av_Param_Init) = 0;
      s32_TuneSearchTableBest[0] = BGVAR(s32_Pos_Tune_Av_Param_Init);// update best value for easier debug
      if ((s32_TuneSearchTable[0][0] == POS_TUNE_AV_SHRP) || (s32_TuneSearchTable[0][0] == POS_TUNE_AV_SHRP2) || (s32_TuneSearchTable[0][0] == POS_TUNE_AV_SHRP3) )
         PosTuneWriteParam(s32_TuneSearchTable[0][0],s32_Pos_Tune_Av_Shrp_Init_value,drive);// write init value for sharp
      else
         PosTuneWriteParam(s32_TuneSearchTable[0][0],PosTuneReadParam(s32_TuneSearchTable[0][0],drive,2),drive);// write min value (init value)
      return 0;
   }

   // verify no cost overflow
   if (s64_final_cost < 0 )
   {  // reduce weight. signal handler to restart.
      BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_COST_FATOR_FAULT;
      return 0;//execution halted due to cost overflow
   }

   if (s64_final_cost < BGVAR(s64_Tune_Min_Cost)) // better value found - continue search
   {
      BGVAR(s64_Tune_Min_Cost) = s64_final_cost;
      BGVAR(s32_Pos_Tune_Av_Param_Init) =  PosTuneReadParam(s32_TuneSearchTable[0][0],drive,0);
      s32_TuneSearchTableBest[0] = BGVAR(s32_Pos_Tune_Av_Param_Init);// update best value for easier debug
   }

   s64_temp = s64_final_cost >> 1;
   if ( (s64_temp > BGVAR(s64_Tune_Min_Cost))           || //cost limit (mincost*2<current cost) exceeded
        (s64_final_cost == 0x7FFFFFFFFFFFFFFF)          || // search terminated due to effort limit
        (PosTuneReadParam(s32_TuneSearchTable[0][0],drive,1) == PosTuneReadParam(s32_TuneSearchTable[0][0],drive,0))  ) // max gain reached
   {
      PosTuneWriteParam(s32_TuneSearchTable[0][0],BGVAR(s32_Pos_Tune_Av_Param_Init),drive);
      s16_first_run_flag = 1;
      if (BGVAR(s32_Pos_Tune_Av_Param_Init) == 0)
      {
         if (s32_TuneSearchTable[0][0]  == POS_TUNE_AV_Q3) PosTuneWriteParam(POS_TUNE_AV_Q3,PosTuneReadParam(POS_TUNE_AV_Q3,drive,2),drive);// write min value (init value)
         // test result - if need be - change sharp and re-try
         if ((s32_TuneSearchTable[0][0]  == POS_TUNE_AV_GAIN2) && (PosTuneReadParam(POS_TUNE_AV_SHRP2,drive,0) == 1000*AT_AV_SHARP_FOR_POSTUNE2))
         {
            SalNlPeSharpnessCommand((long long)10000*AT_AV_SHARP_FOR_POSTUNE2,drive);
            return -1; // restart with higher sharp value
         }
         if ((s32_TuneSearchTable[0][0]  == POS_TUNE_AV_GAIN3) && (PosTuneReadParam(POS_TUNE_AV_SHRP3,drive,0) == 1000*AT_AV_SHARP_FOR_POSTUNE3))
         {
            SalNlPe3SharpnessCommand((long long)10000*AT_AV_SHARP_FOR_POSTUNE3,drive);
            return -1; // restart with higher sharp value
         }
      }
      return 1;
   }

   // increase init value
   s32_temp = PosTuneReadParam(s32_TuneSearchTable[0][0],drive,0);
   if ((s32_temp == 0) && (s32_TuneSearchTable[0][0]  == POS_TUNE_AV_GAIN3))
      s32_temp = 100;
   else if ((s32_temp == 0) && (s32_TuneSearchTable[0][0]  == POS_TUNE_AV_GAIN2))
      s32_temp = 1000; // only applicable to gain. sharp will never be 0
   else
   {
      if ((s32_TuneSearchTable[0][0] == POS_TUNE_AV_GAIN2) && (s32_temp < 2000))
         s32_temp+=500;
      else if (((s32_TuneSearchTable[0][0] == POS_TUNE_AV_SHRP) || (s32_TuneSearchTable[0][0] == POS_TUNE_AV_SHRP2)) &&
               (s32_temp < 250)                                                                                              )
         s32_temp+=50; // speed up param increase if searching for SHARP,SHARP2
      else
         s32_temp = (long)(1.2 * (float)s32_temp);
   }

   // protect from numerical lockup
   if (s32_temp == PosTuneReadParam(s32_TuneSearchTable[0][0],drive,0)) ++s32_temp;

   // sat to max value
   if (s32_temp > PosTuneReadParam(s32_TuneSearchTable[0][0],drive,1))
      s32_temp = PosTuneReadParam(s32_TuneSearchTable[0][0],drive,1);

   PosTuneWriteParam(s32_TuneSearchTable[0][0],s32_temp,drive);
   PositionConfig(drive,1);
   VAR(AX0_s16_Cycle_Bits) |= CYCLE_PARAM_UPDATE;
   return 0;
}

/*
void SearchTableSwap(int s16_row1,int s16_row2)
{
   long s32_temp;
   int s16_temp;

   for (s16_temp = 0;((s16_temp < TUNE_SEARCH_TABLE_MAX_COL) && (s32_TuneSearchTable[s16_row1][s16_temp] != POS_TUNE_NO_ID)); ++s16_temp)
   {
      s32_temp = s32_TuneSearchTable[s16_row2][s16_temp];
      s32_TuneSearchTable[s16_row2][s16_temp] = s32_TuneSearchTable[s16_row1][s16_temp];
      s32_TuneSearchTable[s16_row1][s16_temp] = s32_temp;
   }
}
*/

/*void ConvertSearcTableToGreyCode(int s16_table_length)
{
   if (s16_table_length >= 4)
   { //00, 01, 11, 10
      SearchTableSwap(2,3);
   }
   if (s16_table_length >= 8)
   { //  000 001 011 010 110 111 101 100
     SearchTableSwap(6,8);
     SearchTableSwap(5,7);
     SearchTableSwap(8,7);
   }
   if (s16_table_length >= 16)
   { // 0000 0001 0011 0010 0110 0111 0101 0100  1100 1101 1111 1110 1010 1011 1001 1000
     SearchTableSwap(9,13);
     SearchTableSwap(10,14);
     SearchTableSwap(11,16);
     SearchTableSwap(12,15);
     SearchTableSwap(13,16);
     SearchTableSwap(14,16);
   }
}*/

/*void FiltDampingSearchTable(int drive)
{ // search table 10,20,45,60,75,90
   int s16_temp;
   BGVAR(s16_Pos_Tune_State) = POS_TUNE_DAMPING_SEARCH;
   s32_TuneSearchTable[0][0] = POS_TUNE_NLFILT_DAMPING_ID;
   s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
   s16_Tune_Delta_Table[1] = 10;
   BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 1;
   TuneSearchInit(drive,0);

   s32_TuneSearchTable[1][0] = 15;
   s64_TuneSearchTableResult[1] = 0;
   u32_PosTuneSearchEffort[1] = 0;
   s64_PosTuneSearchTQF[1] = 0;
   // bypass search table with special table
   for (s16_temp = 2;s16_temp <=6;++s16_temp)
   {
      s64_TuneSearchTableResult[s16_temp] = 0;
      u32_PosTuneSearchEffort[s16_temp] = 0;
      s64_PosTuneSearchTQF[s16_temp] = 0;
      s32_TuneSearchTable[s16_temp][0] = s32_TuneSearchTable[s16_temp-1][0]+ 15;
   }
   BGVAR(s16_TuneSearchTableBest_RowId) = -1;
   BGVAR(s16_Tune_Table_Size) = 6;
   BGVAR(s16_Tune_Table_Index) = 0;
}
*/

int GenerateTuneSearchTable(int drive)
{ // generate the multi dim search table
  // param1_id | param2_id |...|paramx_id | paramid_none
  // p1_val1   | p2_val1   |...|px_val1
  // ......

  // the code sets up row 0 which is the param id and the function generates the table
  // paramid_none determines the number of permutations

   int s16_temp,s16_table_length = 1,s16_column=0,s16_sgn_changer_mask = 0x0001,s16_param_id;
   long s32_param_value,s32_param_id_delta,s32_param_val_plus_delta,s32_param_val_minus_delta,
        s32_param_max,s32_param_min,s32_param_val_plus_plus_delta,s32_param_val_minus_minus_delta,s32_sum_search_knliv,s32_sum_search_knlp,
        s32_knli_validate_knli,s32_knli_validate_knlp,s32_knli_search_knli,s32_knli_search_max,s32_knliv_max,s32_knlp_max,s32_temp;

   if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_FILTER_VERIFY)
   {
      s32_param_max = PosTuneReadParam(POS_TUNE_NLFILTT1_ID,drive,0);
      s32_param_id_delta = (long)(0.25*((float)s32_param_max - 0.25));
      if (s32_param_id_delta < 500) s32_param_id_delta = 500;
      s32_param_min = 100;
      if (s32_param_max < s32_param_min) s32_param_min = s32_param_min;

      s16_temp = 1;
      s32_param_value = s32_param_max;
      while ((s16_temp < TUNE_SEARCH_TABLE_MAX_ROW) && (s32_param_value > s32_param_min))
      {
         s32_TuneSearchTable[s16_temp][0] = s32_param_value;
         s64_TuneSearchTableResult[s16_temp] = 0;
         u32_PosTuneSearchEffort[s16_temp] = 0;
         s64_PosTuneSearchTQF[s16_temp] = 0;
         s32_param_value = s32_param_value-s32_param_id_delta;
         ++s16_temp;
      }
      --s16_temp;
      BGVAR(s16_TuneSearchTableBest_RowId) = -1;
      return (s16_temp);
   }
   if (BGVAR(s16_Pos_Tune_State) == KNLIV_KNLP_SUM_SEARCH)
   {
      // knliv > 2*knlp knliv knlp such that knliv+knlp is the same. use steps of 10% of knliv which is the highest
      s32_sum_search_knliv = PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0);
      s32_sum_search_knlp = PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0);
      s32_param_id_delta = (long)(0.05*(float)s32_sum_search_knliv);
      if (s32_param_id_delta < 1000) s32_param_id_delta = 1000;

      s16_temp = 1;
      while ((s16_temp < TUNE_SEARCH_TABLE_MAX_ROW) && (s32_sum_search_knliv > s32_sum_search_knlp))
      {
         s32_TuneSearchTable[s16_temp][0] = s32_sum_search_knlp;
         s32_TuneSearchTable[s16_temp][1] = s32_sum_search_knliv;
         s32_sum_search_knliv -= s32_param_id_delta;
         s32_sum_search_knlp += s32_param_id_delta;
         s64_TuneSearchTableResult[s16_temp] = 0;
         u32_PosTuneSearchEffort[s16_temp] = 0;
         s64_PosTuneSearchTQF[s16_temp] = 0;
         ++s16_temp;
      }
      --s16_temp;
      BGVAR(s16_TuneSearchTableBest_RowId) = -1;
      return (s16_temp);
   }
   else if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_KNLI_VALIDATION)
   {  // search between current knli value and knlp value
      s32_knli_validate_knli = PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0);

      s32_knli_validate_knlp = PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0);
      s32_knli_validate_knlp = (long)(0.7*(float)s32_knli_validate_knlp);

      if (s32_knli_validate_knlp > PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0)) s32_knli_validate_knlp = PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0);

      if (s32_knli_validate_knlp <= s32_knli_validate_knli) s32_knli_validate_knlp = s32_knli_validate_knli + 5000;
      s32_param_id_delta = (long) (0.25*(float)(s32_knli_validate_knlp - s32_knli_validate_knli));

      if (s32_param_id_delta < 2500) s32_param_id_delta = 2500;

      s16_temp = 1;
      while ((s16_temp < (TUNE_SEARCH_TABLE_MAX_ROW-1)) && (s32_knli_validate_knli <= s32_knli_validate_knlp))
      {
         s32_TuneSearchTable[s16_temp][0] = s32_knli_validate_knli;
         s64_TuneSearchTableResult[s16_temp] = 0;
         u32_PosTuneSearchEffort[s16_temp] = 0;
         s64_PosTuneSearchTQF[s16_temp] = 0;
         s32_TuneSearchTable[s16_temp+1][0] = s32_knli_validate_knli;
         s64_TuneSearchTableResult[s16_temp+1] = 0;
         u32_PosTuneSearchEffort[s16_temp+1] = 0;
         s64_PosTuneSearchTQF[s16_temp+1] = 0;
         s32_knli_validate_knli += s32_param_id_delta;
         s16_temp+=2;
      }
      --s16_temp;
      BGVAR(s16_TuneSearchTableBest_RowId) = -1;
      return (s16_temp);
   }
   else if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_ADV_KNLI_SEARCH)
   { // search between current knli value and max(knlp,knliv) value
      s32_knli_search_knli = PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0);

      s32_knli_search_max = PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0);
      s32_knli_search_max = 0.7*s32_knli_search_max;

      if (s32_knli_search_max > PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0)) s32_knli_search_max = PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0);

      if (s32_knli_search_max <= s32_knli_search_knli) s32_knli_search_max = s32_knli_search_knli + 5000;
      s32_param_id_delta = (long)(0.25*(float)(s32_knli_search_max - s32_knli_search_knli));

      if (s32_param_id_delta < 2500) s32_param_id_delta = 2500;

      s16_temp = 1;
      while ((s16_temp < TUNE_SEARCH_TABLE_MAX_ROW) && (s32_knli_search_knli <= s32_knli_search_max))
      {
         s32_TuneSearchTable[s16_temp][0] = s32_knli_search_knli;
         s64_TuneSearchTableResult[s16_temp] = 0;
         u32_PosTuneSearchEffort[s16_temp] = 0;
         s64_PosTuneSearchTQF[s16_temp] = 0;
         s32_TuneSearchTable[s16_temp+1][0] = s32_knli_search_knli;
         s64_TuneSearchTableResult[s16_temp+1] = 0;
         u32_PosTuneSearchEffort[s16_temp+1] = 0;
         s64_PosTuneSearchTQF[s16_temp+1] = 0;
         s32_knli_search_knli += s32_param_id_delta;
         s16_temp+=2;
      }
      --s16_temp;
      BGVAR(s16_TuneSearchTableBest_RowId) = -1;
      return (s16_temp);
   }
   else if ((BGVAR(s16_Pos_Tune_State) >= POS_TUNE_CONT_INITIAL_COST_CALC) &&
            (BGVAR(s16_Pos_Tune_State) <= POS_TUNE_CONT_STOPPED)             )
   {
      if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_CONT_AV3_DEC_TUNE)
      {
         s32_param_value = PosTuneReadParam(POS_TUNE_AV_GAIN3,drive,0)-250;
         if (s32_param_value < 0) s32_param_value = 0;
      }
      else if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_CONT_AV3_INC_TUNE)
         s32_param_value = PosTuneReadParam(POS_TUNE_AV_GAIN3,drive,0)+250;
      else if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_CONT_AV2_DEC_TUNE)
      {
         s32_param_value = PosTuneReadParam(POS_TUNE_AV_GAIN2,drive,0)-750;
         if (s32_param_value < 0) s32_param_value = 0;
      }
      else if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_CONT_AV2_INC_TUNE)
         s32_param_value = PosTuneReadParam(POS_TUNE_AV_GAIN3,drive,0)+750;
      else if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_CONT_DEC_KNLP_TUNE)
      {
         s32_param_value = PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0);
         s32_param_value = (long)(0.9*(float)s32_param_value);
      }
      else if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_CONT_INC_KNLP_TUNE)
      {
         s32_param_value = PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0);
         s32_param_value = (long)(1.1*(float)s32_param_value);
      }
      else if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_KNLI_DEC_TUNE)
      {
         s32_param_value = PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0);
         s32_param_value = (long)(0.9*(float)s32_param_value);
      }
      else if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_KNLI_INC_TUNE)
      {
         s32_param_value = PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0);
         s32_param_value = (long)(1.1*(float)s32_param_value);
      }
      else if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_CONT_INITIAL_COST_CALC)
      {
         s32_param_value = PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0);
      }
      else
      {
         s32_param_value = PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0);

         if ((BGVAR(s16_Cont_Tune_Flags) & CONT_TUNE_INC_USERGAIN_MASK) != 0)
            s32_param_value = (long)(1.1*(float)s32_param_value);
         else if ((BGVAR(s16_Cont_Tune_Flags) & CONT_TUNE_DEC_USERGAIN_MASK) != 0)
            s32_param_value = (long)(0.9*(float)s32_param_value);

      }
      s16_temp = 1;
      s16_table_length = BGVAR(s16_Pos_tune_Mode);
      if (s16_table_length == 7) s16_table_length = 4;
      while (s16_temp <=s16_table_length)
      {
         s32_TuneSearchTable[s16_temp][0] = s32_param_value;
         s64_TuneSearchTableResult[s16_temp] = 0;
         u32_PosTuneSearchEffort[s16_temp] = 0;
         s64_PosTuneSearchTQF[s16_temp] = 0;
         ++s16_temp;
      }
      --s16_temp;
      BGVAR(s16_TuneSearchTableBest_RowId) = -1;
      return (s16_temp);
   }


   for (s16_temp = 0;((s16_temp < TUNE_SEARCH_TABLE_MAX_COL) && (s32_TuneSearchTable[0][s16_temp] != POS_TUNE_NO_ID)); ++s16_temp) s16_table_length <<=1;
   //s16_table_length >>= 1;

   while(s32_TuneSearchTable[0][s16_column] != POS_TUNE_NO_ID)
   {
      s16_param_id = s32_TuneSearchTable[0][s16_column];
      s32_param_value = PosTuneReadParam(s16_param_id,drive,0);
      s32_param_max = PosTuneReadParam(s16_param_id,drive,1);
      s32_param_min = PosTuneReadParam(s16_param_id,drive,2);
      s32_param_id_delta = CalcParamDelta(s16_column,s32_param_value);

      if ((s16_param_id == POS_TUNE_KNLUSER_ID) && (BGVAR(s16_Pos_Tune_State) == POS_TUNE_USERGAIN_SEARCH))
      {
         // min value is usergainmax / 8 - to shorten AT incases of high load
         s32_temp = BGVAR(s32_At_Usergain_Max) >> 3;
         if (s32_param_value < s32_temp) s32_param_id_delta = (3 * s32_param_id_delta)>>1;
      }

      if ((s16_param_id == POS_TUNE_AV_HZ2) ||
          (s16_param_id == POS_TUNE_AV_HZ3)   ) s32_param_id_delta = 1000;

      if (((s16_param_id == POS_TUNE_AV_SHRP2) || (s16_param_id == POS_TUNE_AV_SHRP3) ) &&
          (s32_param_id_delta < 25)                                                                                              )
         s32_param_id_delta = 25;

      if (s16_param_id == POS_TUNE_AV_GAIN2) s32_param_id_delta = 500;
      else if (s16_param_id == POS_TUNE_AV_GAIN3) s32_param_id_delta = 250;


      if ((s16_param_id == POS_TUNE_AV_Q3) &&  (s32_param_id_delta > 50)) s32_param_id_delta = 50;

      if ( ((s16_param_id == POS_TUNE_KNLIV_ID) ||
            (s16_param_id == POS_TUNE_KNLD_ID)  ||
            (s16_param_id == POS_TUNE_KNLP_ID)     ) &&
           (s32_param_id_delta > 15000)                ) s32_param_id_delta = 15000;

      if ((s16_param_id == POS_TUNE_KNLIV_ID) && (s32_param_id_delta < 3500)) s32_param_id_delta = 3500;
      if ((s16_param_id == POS_TUNE_KNLP_ID)  && (s32_param_id_delta < 3500)) s32_param_id_delta = 3500;

      if (s16_param_id == POS_TUNE_NLAFRC_ID)
      {
         s32_param_id_delta = 10;
         if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_NLAFRC_SEARCH_1) s32_param_id_delta = 25;
      }

      s32_param_val_plus_delta = s32_param_value+s32_param_id_delta;
      if ((s32_param_val_plus_delta > s32_param_max) &&
          (s16_param_id != POS_TUNE_KNLUSER_ID)        ) s32_param_val_plus_delta = s32_param_max;

      s32_param_val_plus_plus_delta = s32_param_val_plus_delta+s32_param_id_delta;
      if ((s32_param_val_plus_plus_delta > s32_param_max) &&
          (s16_param_id != POS_TUNE_KNLUSER_ID)             ) s32_param_val_plus_plus_delta = s32_param_max;

      s32_param_val_minus_delta = s32_param_value-s32_param_id_delta;
      if (s32_param_val_minus_delta < s32_param_min) s32_param_val_minus_delta = s32_param_min;

      s32_param_val_minus_minus_delta = s32_param_val_minus_delta-s32_param_id_delta;
      if (s32_param_val_minus_minus_delta < s32_param_min) s32_param_val_minus_minus_delta = s32_param_min;

      if ((s16_param_id == POS_TUNE_KNLIV_ID)                                       &&
          ((BGVAR(s16_Pos_Tune_State) == POS_TUNE_ADV_KNLIV_SEARCH) || (BGVAR(s16_Pos_Tune_State) == POS_TUNE_ADV_KNLIV_AFTER_ANTIVIB_INIT))  )
      {
         if (BGVAR(s16_Min_Cost_Found_Cntr_Tresh) == 0)
         {
            s32_param_val_minus_delta = s32_param_value;
            s32_param_val_plus_delta = s32_param_val_plus_delta;
         }
         else
         {
            s32_param_val_minus_delta = s32_param_val_plus_delta;
            s32_param_val_plus_delta = s32_param_id_delta>>1;
            if (s32_param_val_plus_delta < 2500) s32_param_val_plus_delta = 2500;
            s32_param_val_plus_delta += s32_param_val_minus_delta;
         }

         s32_knliv_max = (long)(0.8*(float)(PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0)));
         if (s32_param_val_plus_delta > s32_knliv_max) s32_param_val_plus_delta = s32_knliv_max;
         if (s32_param_val_minus_delta > s32_knliv_max) s32_param_val_minus_delta = s32_knliv_max;
      }


      if ((s16_param_id == POS_TUNE_KNLUSER_ID)                          &&
          ((BGVAR(s16_Pos_Tune_State) == KNLUSERGAIN_VALIDITY_CHECK) || (BGVAR(s16_Pos_Tune_State) == KNLUSERGAIN_VALIDITY_CHECK_2))   )
      {
         s32_param_val_minus_delta = s32_param_val_minus_delta;
         s32_param_val_plus_delta = s32_param_value;
      }

      if ( ((s16_param_id == POS_TUNE_KNLP_ID)    && (BGVAR(s16_Pos_Tune_State) == POS_TUNE_ADV_KNLP_SEARCH)) ||
           ((s16_param_id == POS_TUNE_KNLUSER_ID) && (BGVAR(s16_Pos_Tune_State) == POS_TUNE_USERGAIN_SEARCH))   )
      {
         if (BGVAR(s16_Min_Cost_Found_Cntr_Tresh) == 0)
         {
            s32_param_val_minus_delta = s32_param_value;
            s32_param_val_plus_delta = s32_param_val_plus_delta;
         }
         else
         {
            s32_param_val_minus_delta = s32_param_val_plus_delta;
            s32_param_val_plus_delta = s32_param_val_plus_plus_delta;
         }

         if (s16_param_id == POS_TUNE_KNLP_ID)
         {
            s32_knlp_max = (long)(0.8*(float)(PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0)));
            if (s32_param_val_plus_delta > s32_knlp_max) s32_param_val_plus_delta = s32_knlp_max;
            if (s32_param_val_minus_delta > s32_knlp_max) s32_param_val_minus_delta = s32_knlp_max;
         }

      }

      if (s16_param_id == POS_TUNE_NLAFRC_ID) s32_param_val_minus_delta = s32_param_value;

      if ((s16_param_id == POS_TUNE_KNLUSER_ID) && (BGVAR(s16_Pos_Tune_State) == POS_TUNE_USERGAIN_SEARCH) )
       { // make sure will not search in invalid area of knld
          if (s32_param_val_minus_delta > BGVAR(s32_At_Usergain_Max)) s32_param_val_minus_delta = BGVAR(s32_At_Usergain_Max);
          if (s32_param_val_plus_delta > BGVAR(s32_At_Usergain_Max)) s32_param_val_plus_delta = BGVAR(s32_At_Usergain_Max);
       }


      for (s16_temp = 1;s16_temp <= s16_table_length;++s16_temp)
      {
         if (((s16_temp-1) & s16_sgn_changer_mask) == 0)
           s32_TuneSearchTable[s16_temp][s16_column] = s32_param_val_minus_delta;
         else
            s32_TuneSearchTable[s16_temp][s16_column] = s32_param_val_plus_delta;
         s64_TuneSearchTableResult[s16_temp] = 0;
         u32_PosTuneSearchEffort[s16_temp] = 0;
         s64_PosTuneSearchTQF[s16_temp] = 0;
      }

      ++s16_column;
      s16_sgn_changer_mask <<= 1;
   }
   BGVAR(s16_TuneSearchTableBest_RowId) = -1;
   //ConvertSearcTableToGreyCode(s16_table_length);
   return s16_table_length;
}


int ExecutePosTuneSearchTable(int drive,int s16_gain_recover_flag)
{
   int s16_temp,s16_column=0,s16_effort_flag=0;
   long s32_temp;
   long long s64_temp,s64_final_cost,s64_tqf;
   unsigned long u32_max_effort;
   // AXIS_OFF;

   do {
       s16_temp = Cntr_3125;
       s64_temp = LLVAR(AX0_u32_At_Final_Cost_Lo);
   } while (s16_temp != Cntr_3125);

   if ( ((s64_temp > 0x7FFFFFFFFFFFFFF0) && (BGVAR(s16_Tune_Table_Index) != 0) )        ||
        ((BGVAR(u16_At_Foldback_Capture) != 0) && (BGVAR(s16_Tune_Table_Index) == 0) )    )
   { // => effort function maxed out - gains are reduced at rt (unless it is antivib)
     // here swap gains with best values
      RestoreBestSearchResults(drive);
      if (s64_temp == 0x7FFFFFFFFFFFFFFF) BGVAR(s32_Pos_Tune_Bits) |= EFFORT_EVENT_MASK;

      // when effort exceeded and potune in avmode zero avgain till next itteration
      // zero av gainc
      if ((VAR(AX0_AutoTune_Bits) & AT_AV_ACTIVE_2_MASK) != 0) SalNlPeGainCommand(0LL,drive);
      if ((VAR(AX0_AutoTune_Bits) & AT_AV_ACTIVE_3_MASK) != 0) SalNlPe3GainCommand(0LL,drive);
      if ((VAR(AX0_AutoTune_Bits) & AT_AV_ACTIVE_MASK) == 0)
      {
         u32_max_effort = (unsigned long)(0.66*(float)BGVAR(u32_At_Control_Effort_Max_Captured));
         if ( ((u32_max_effort > LVAR(AX0_u32_At_Control_Effort_Tresh)) || (BGVAR(u16_At_Foldback_Capture) != 0)) &&
               (s16_Extreemly_High_Effort == 0)                        &&
               (BGVAR(u16_HdTune_Adv_Mode) == 0)                          )
         { // extreemly high effort - reduce usergain by 50% and wait till effort settles
            s16_Extreemly_High_Effort = 1;
            s32_High_Effort_User_Gain = PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0);
            PosTuneWriteParam(POS_TUNE_KNLUSER_ID,(long)(0.5*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))),drive);
            BGVAR(s32_Pos_Tune_Bits) |=EXTREME_EFFORT_MASK;
            // on verify and extreme effort - uct final user gain
            if ((s16_gain_recover_flag == 2) || (s16_gain_recover_flag == 3)) s32_High_Effort_User_Gain = (long)(0.9*(float)(s32_High_Effort_User_Gain));
         }
      }
      PositionConfig(drive,1);
      VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
   }

   // test if new result is ready
   if  ((VAR(AX0_s16_Cycle_Bits) & CYCLE_NEW_CYCLE_DATA_READY) != 0)
   {
      s64_final_cost = BGVAR(s64_Cost_Function_Final_Capture);

      BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_DATA_READY_MASK;
      VAR(AX0_s16_Cycle_Bits) &= ~(CYCLE_NEW_CYCLE_DATA_READY);
      u32_max_effort = BGVAR(u32_At_Control_Effort_Max_Captured);

      s64_tqf = BGVAR(s64_At_TQF);
   }

   // test timer/cycle indication is done. if not exit
   if ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_DATA_READY_MASK) == 0) return 0;
   BGVAR(s32_Pos_Tune_Bits) &= ~POS_TUNE_DATA_READY_MASK;

   BGVAR(s32_Pos_Tune_Bits) &= ~EFFORT_EVENT_MASK;

   if (s64_final_cost < 0)
   {
      if (BGVAR(u16_At_Pretubation_En) == 2) BGVAR(u16_At_Pretubation_En) = 1;
      return -1;//execution halted due to cost overflow
   }

   // if not recovering from effort - log result
   if ( (s16_Extreemly_High_Effort == 0) || (s16_Extreemly_High_Effort == 1)   )
   {
      if (BGVAR(s16_Tune_Table_Index) != 0)
      {
         s64_TuneSearchTableResult[BGVAR(s16_Tune_Table_Index)] = s64_final_cost;
         u32_PosTuneSearchEffort[BGVAR(s16_Tune_Table_Index)] = u32_max_effort;
         s64_PosTuneSearchTQF[BGVAR(s16_Tune_Table_Index)] = s64_tqf;
         if (u32_max_effort > LVAR(AX0_u32_At_Control_Effort_Tresh)) u32_PosTuneSearchEffort[0] = 1;
      }
      else
      {
        s64_TuneSearchTableResult[0] = 0x7FFFFFFFFFFFFFFF;
        u32_PosTuneSearchEffort[0] = 0;
        s64_PosTuneSearchTQF[0] = 0;
        if (BGVAR(u16_At_Pretubation_En) == 1) BGVAR(u16_At_Pretubation_En) = 2;
      }
   }

   if (s16_Extreemly_High_Effort == 1)
   {
      s16_Extreemly_High_Effort = 2;
      if (BGVAR(u16_At_Pretubation_En) == 2) BGVAR(u16_At_Pretubation_En) = 1;
      return 0;
   }

   if ((s16_Extreemly_High_Effort > 1) && (s16_Extreemly_High_Effort <32767))
   {
      //u32_max_effort = (unsigned long)(0.66*(float)u32_max_effort);
      if (BGVAR(u16_At_Pretubation_En) == 2) BGVAR(u16_At_Pretubation_En) = 1;
      if (((u32_max_effort > LVAR(AX0_u32_At_Control_Effort_Tresh)) || (BGVAR(u16_At_Foldback_Capture) != 0))  && (s16_Extreemly_High_Effort < 4))
      {
         PosTuneWriteParam(POS_TUNE_KNLUSER_ID,(long)(0.82*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))),drive);
         ++s16_Extreemly_High_Effort;
      }
      else if ((u32_max_effort < LVAR(AX0_u32_At_Control_Effort_Tresh)) && (BGVAR(u16_At_Foldback_Capture) == 0))
      {
         s16_Extreemly_High_Effort = 32767;
         PosTuneWriteParam(POS_TUNE_KNLUSER_ID,((3*PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))>>1),drive);
      }
      PositionConfig(drive,1);
      VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
      return 0;
   }
   else if (s16_Extreemly_High_Effort == 32767)
   {
      s32_temp = ((3*PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0))>>1);
      if (s32_temp > s32_High_Effort_User_Gain)
      {
         s32_temp = s32_High_Effort_User_Gain;
         s16_Extreemly_High_Effort = 0;
         s16_effort_flag = 1;
      }
      PosTuneWriteParam(POS_TUNE_KNLUSER_ID,s32_temp,drive);

      PositionConfig(drive,1);
      VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
      if (s16_Extreemly_High_Effort != 0)
      {
         if (BGVAR(u16_At_Pretubation_En) == 2) BGVAR(u16_At_Pretubation_En) = 1;
         return 0;
      }
   }

   if (BGVAR(s16_Tune_Table_Index) == BGVAR(s16_Tune_Table_Size))
   {
      BGVAR(s16_Tune_Table_Index) = 0;
      if (BGVAR(u16_At_Pretubation_En) == 2) BGVAR(u16_At_Pretubation_En) = 1;
      return 1; // table execution done
   }


   // on verify - if effot event and index not the last one - do not continue the verify - give all other results
   // the max cost to reduce noises
   if ( ((s16_gain_recover_flag == 2)            ||
         (s16_gain_recover_flag == 3)            ||
         (s16_gain_recover_flag == 10)/*cont tune*/   )  &&
        ((s64_final_cost == 0x7FFFFFFFFFFFFFFF)  ||
         (s16_effort_flag == 1)                       )    )
   {
      do{
         ++BGVAR(s16_Tune_Table_Index);
         s64_TuneSearchTableResult[BGVAR(s16_Tune_Table_Index)] = s64_final_cost;
         u32_PosTuneSearchEffort[BGVAR(s16_Tune_Table_Index)] = u32_max_effort;
         s64_PosTuneSearchTQF[BGVAR(s16_Tune_Table_Index)] = s64_tqf;
         if (u32_max_effort > LVAR(AX0_u32_At_Control_Effort_Tresh)) u32_PosTuneSearchEffort[0] = 1;
        }while (BGVAR(s16_Tune_Table_Index) < BGVAR(s16_Tune_Table_Size));
      BGVAR(s16_Tune_Table_Index) = 0;
      if (BGVAR(u16_At_Pretubation_En) == 2) BGVAR(u16_At_Pretubation_En) = 1;
      return 1; // table execution done
   }

   ++BGVAR(s16_Tune_Table_Index);

   for (s16_column = 0;s32_TuneSearchTable[0][s16_column] !=  POS_TUNE_NO_ID;++s16_column)
   {
      PosTuneWriteParam(s32_TuneSearchTable[0][s16_column],s32_TuneSearchTable[BGVAR(s16_Tune_Table_Index)][s16_column],drive);
      if (s32_TuneSearchTable[0][s16_column] == POS_TUNE_KNLIV_ID)
      {
         if (BGVAR(u16_HdTune_Adv_Mode) != 0)
         {
            s32_temp = (long)(BGVAR(f_Kp_Kiv_Ratio)*(float)(PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0)));
          PosTuneWriteParam(POS_TUNE_KNLP_ID,s32_temp,drive);
          s32_temp = (long)(BGVAR(f_Ki_Kp_Ratio)*(float)s32_temp);
          PosTuneWriteParam(POS_TUNE_KNLI_ID,s32_temp,drive);
      }
         else
         {
            s32_temp = (long)(0.4*(float)(PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0)));
            if (s32_temp > PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0)) PosTuneWriteParam(POS_TUNE_KNLP_ID,s32_temp,drive);
         }
      }
   }

   PositionConfig(drive,1);
   VAR(AX0_s16_Cycle_Bits) |= CYCLE_PARAM_UPDATE;
   return 0;
}

long CalcParamDelta(int s16_column,long s32_param_value)
{
   long s32_temp = (long)(((float)s16_Tune_Delta_Table[s16_column]/100.0)*(float)s32_param_value);
   if ((s32_temp == 0) && (s16_Tune_Delta_Table[s16_column] != 0)) s32_temp = 1;
   return s32_temp;
}


// #pragma CODE_SECTION(PosTuneReadParam, "ramfunc");
#pragma CODE_SECTION(PosTuneReadParam, "ramfunc_3");
long PosTuneReadParam(int s16_param_id,int drive, int mode)
{
   // mode = 0 - value
   //        1 - max
   //        2 - min

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch (s16_param_id)
   {
      case POS_TUNE_KNLUSER_ID:
         if (mode == 1) return 3000;
         else if (mode == 2) return 100;
         else return (long)(BGVAR(u32_Nl_Kpgf_User));

      case POS_TUNE_KNLI_ID:
         if (mode == 1) return 200000;
         else if (mode == 2) return 100;
         else return (long)(BGVAR(u32_Nl_Kpi_User));

      case POS_TUNE_KNLD_ID:
         if (mode == 1) return 2000000;
         else if (mode == 2) return 1;
         else return (long)(BGVAR(u32_Nl_Kpd_User));

      case POS_TUNE_KNLIV_ID:
         if (mode == 1) return 400000;
         else if (mode == 2) return 100;
         else return (long)(BGVAR(u32_Nl_Kpiv_User));

      case POS_TUNE_KNLP_ID:
         if (mode == 1) return 400000;
         else if (mode == 2) return 100;
         else return (long)(BGVAR(u32_Nl_Kpp_User));

      case POS_TUNE_NLPEAFF_ID:
         if (mode == 1) return 5000000;
         else if (mode == 2) return 1;
         else return (long)(BGVAR(u32_Nl_KffSpring_User));

      case POS_TUNE_NLAFFLPFHZ_ID:
         if (mode == 1) return 5000;
         else if (mode == 2) return 10;
         else return (long)(BGVAR(s16_Kbff_Spring_LPF_User));

      case POS_TUNE_NLFILTT1_ID:
      case POS_TUNE_NLFILTT1_AND_DAMPING_ID:
         if (mode == 1) return 30000;
         else if (mode == 2) return 50;
         else return (long)(BGVAR(s16_Nl_Out_Filter_1_User));

      case POS_TUNE_NLFILT_DAMPING_ID:
         if (mode == 1) return 85;
         else if (mode == 2) return 15;
         else return (long)(BGVAR(s16_Nl_Out_Filter_2_User));

      case POS_TUNE_NLAFRC_ID:
         if (mode == 1) return 200;
         else if (mode == 2) return 0;
         else return (long)(BGVAR(u16_KNLAFF));

      case POS_TUNE_AV_HZ:
         if (mode == 1) return 400000;
         else if (mode == 2) return 5000;
         else return (long)(BGVAR(u32_Nl_K_Anti_Resonance_Fcenter_User));

      case POS_TUNE_AV_HZ2:
         if (mode == 1) return 400000;
         else if (mode == 2) return 5000;
         else return (long)(BGVAR(u32_Pe_Filt_Fcenter_User));

       case POS_TUNE_AV_HZ3:
         if (mode == 1) return 400000;
         else if (mode == 2) return 5000;
         else return (long)(BGVAR(u32_Antivib3_Fcenter_User));

      case POS_TUNE_AV_GAIN:
         if (mode == 1) return 10000000;
         else if (mode == 2) return 0;
         else return (long)(BGVAR(u32_Nl_K_Anti_Vibration_User));

      case POS_TUNE_AV_GAIN2:
         if (mode == 1) return 5000;
         else if (mode == 2) return 0;
         else return (long)(BGVAR(s32_Pe_Filt_Gain_User));

      case POS_TUNE_AV_GAIN3:
         if (mode == 1) return 5000;
         else if (mode == 2) return 0;
         else return (long)(BGVAR(s32_Antivib3_Gain_User));

      case POS_TUNE_AV_SHRP:
         if (mode == 1) return 3000;
         else if (mode == 2) return 10;
         else return (long)(BGVAR(u16_Nl_K_Anti_Resonance_Sharp_User));

      case POS_TUNE_AV_SHRP2:
         if (mode == 1) return 1000;
         else if (mode == 2) return 30;
         else return (long)(BGVAR(u16_Pe_Filt_Sharpness_User));

      case POS_TUNE_AV_SHRP3:
         if (mode == 1) return 1000;
         else if (mode == 2) return 30;
         else return (long)(BGVAR(u16_Antivib3_Sharpness_User));

      case POS_TUNE_AV_Q3:
         if (mode == 1) return 10000;
         else if (mode == 2) return 1000;
         else return (long)(BGVAR(s32_Antivib3_Q_Gain_User));

      default:
         return -1;
   }
}

void PosTuneWriteParam(int s16_param_id,long s32_param_val,int drive)
{
   float f_temp;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch (s16_param_id)
   {
      case POS_TUNE_KNLUSER_ID:
         BGVAR(u32_Nl_Kpgf_User) = (unsigned long)s32_param_val;
         break;

      case POS_TUNE_KNLI_ID:
         BGVAR(u32_Nl_Kpi_User) = (unsigned long)s32_param_val;
         break;

      case POS_TUNE_KNLD_ID:
         BGVAR(u32_Nl_Kpd_User) = (unsigned long)s32_param_val;
         break;

      case POS_TUNE_KNLIV_ID:
         BGVAR(u32_Nl_Kpiv_User) = (unsigned long)s32_param_val;
         break;

      case POS_TUNE_KNLP_ID:
         BGVAR(u32_Nl_Kpp_User) = (unsigned long)s32_param_val;
         break;

      case POS_TUNE_NLPEAFF_ID:
         BGVAR(u32_Nl_KffSpring_User) = (unsigned long)s32_param_val;
         break;

      case POS_TUNE_NLAFFLPFHZ_ID:
         BGVAR(s16_Kbff_Spring_LPF_User) = (int)s32_param_val;
         break;

      case POS_TUNE_NLFILTT1_ID:
         BGVAR(s16_Nl_Out_Filter_1_User) = (int)s32_param_val;
         break;

      case POS_TUNE_NLFILT_DAMPING_ID:
         BGVAR(s16_Nl_Out_Filter_2_User) = (int)s32_param_val;
         break;

      case POS_TUNE_NLAFRC_ID:
         BGVAR(u16_KNLAFF) = (unsigned int)s32_param_val;
         break;

      case POS_TUNE_NLFILTT1_AND_DAMPING_ID:
          BGVAR(s16_Nl_Out_Filter_1_User) = (int)s32_param_val;
          if (s32_param_val < 500) s32_param_val = 15;
          else
          {
            if (s32_param_val < 550) s32_param_val = 550; // numerical protecion for the below formula
            f_temp = 31.25 / (float)(s32_param_val-500);
            f_temp = f_temp + 1 - 2*sqrt(f_temp);
            s32_param_val = (long)(85 * f_temp); // critical samping * 0.85
          }
          if (s32_param_val < 15) s32_param_val = 15;
          if (s32_param_val > 85) s32_param_val = 85;
          BGVAR(s16_Nl_Out_Filter_2_User) = (int)s32_param_val;
          break;

      case POS_TUNE_AV_HZ:
         BGVAR(u32_Nl_K_Anti_Resonance_Fcenter_User) = (unsigned long)s32_param_val;
         break;

      case POS_TUNE_AV_HZ2:
         BGVAR(u32_Pe_Filt_Fcenter_User) = (unsigned long)s32_param_val;
         break;

      case POS_TUNE_AV_HZ3:
         BGVAR(u32_Antivib3_Fcenter_User) = (unsigned long)s32_param_val;
         break;

      case POS_TUNE_AV_GAIN:
         BGVAR(u32_Nl_K_Anti_Vibration_User) = (unsigned long)s32_param_val;
         break;

      case POS_TUNE_AV_GAIN2:
         BGVAR(s32_Pe_Filt_Gain_User) = s32_param_val;
         break;

      case POS_TUNE_AV_GAIN3:
         BGVAR(s32_Antivib3_Gain_User) = s32_param_val;
         break;

      case POS_TUNE_AV_SHRP:
         BGVAR(u16_Nl_K_Anti_Resonance_Sharp_User) = (unsigned int)s32_param_val;
         break;

      case POS_TUNE_AV_SHRP2:
         BGVAR(u16_Pe_Filt_Sharpness_User) = (unsigned int)s32_param_val;
         break;

      case POS_TUNE_AV_SHRP3:
         BGVAR(u16_Antivib3_Sharpness_User) = (unsigned int)s32_param_val;
         break;

      case POS_TUNE_AV_Q3:
         BGVAR(s32_Antivib3_Q_Gain_User) = (unsigned int)s32_param_val;
         break;
   }
}

int PosTuneCommand(int drive)
{ // for cont tune drive may be active
  // else drive must be inactive
  // stop can be done at anytime
  //
  // 1 - optimal tunning min settle time
  // 2 - optimal tunning min pe
  // 3 - optimal tunning min overshoot
  // 4 - LMJR Est
  // 5 - easy tunning (fast)
  // 6 - advanced user defined trajectory
  // 7 - advanced external trajectory - express
  // 8 - advanced external trajectory
  // 9 - cont tunning min settle time
  // 10 - cont tunning min pe
  // 11 - cont tunning min overshoot
  // 90 - AVHZ test mode
  // 100 RTAT test mode
   long s32_temp;
   long long param = s64_Execution_Parameter[0];
   int s16_av_backup = BGVAR(s16_HdTune_Av_Mode),
       s16_igrav_backup = BGVAR(s16_Pos_Tune_Igrav),
       s16_knlafrc_backup = BGVAR(s16_Pos_Tune_KnlAfrc_Mode),
       s16_stiff_backup = BGVAR(s16_Pos_Tune_Stiffness),
       s16_cycle_backup = BGVAR(s16_Pos_Tune_Cycle),
       s16_pos_tune_lmjr_backup = BGVAR(s16_Pos_Tune_LMJR),
       ret_val,s16_temp;
   // AXIS_OFF;

   if (VAR(AX0_u16_Pos_Control_Mode) == LINEAR_POSITION) return (INVALID_POSCONTROLMODE);
   if (BGVAR(s16_DisableInputs) & MTP_READ_DIS_MASK)            return NOT_AVAILABLE;
   // Init start with enable mode variation
   BGVAR(s16_Pos_Tune_Enable_Mode) = 0;
   if (param == 0)
   {
      if  (BGVAR(s16_Pos_Tune_State) > 0)
      {
         if ((s16_Number_Of_Parameters != 1) && (BGVAR(s16_Pos_Tune_State) > POS_TUNE_USERGAIN_SEARCH))
         {                                      // number of parameters not 0 - special mode
            RestoreBestSearchResults(drive);    // get out of tunning with best search if possible
            PosTuneBackup(drive);
            PositionConfig(drive,1);
            VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         }
         if (BGVAR(s16_Pos_Tune_State) < POS_TUNE_DONE)
         {
            PosTuneRestore(drive,1);
            BGVAR(s16_Pos_Tune_State) = POS_TUNE_STOPPED_USER;
         }
         BGVAR(s16_Pos_tune_Mode) = 0;
      }
   }
   else
   {
      if ((s16_Record_Flags & RECORD_ON_MASK) != 0) return (REC_ACTIVE);

      if(BGVAR(s16_DisableInputs) & MTP_READ_DIS_MASK) return NOT_AVAILABLE;
      if (VAR(AX0_u16_Pos_Control_Mode) == LINEAR_POSITION) return (INVALID_POSCONTROLMODE);
      if  ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
      {
         if ((VAR(AX0_u16_Pos_Control_Mode) != NON_LINEAR_POSITION_2) &&
             (VAR(AX0_u16_Pos_Control_Mode) != NON_LINEAR_POSITION_5)    ) return (INVALID_POSCONTROLMODE);
      }
      else
      {
          if ((param < 6) && (BGVAR(u16_HdTune_Adv_Mode) == 0)         &&
               (VAR(AX0_u16_Pos_Control_Mode) != NON_LINEAR_POSITION_3) &&
               (VAR(AX0_u16_Pos_Control_Mode) != NON_LINEAR_POSITION_4) &&
               (VAR(AX0_u16_Pos_Control_Mode) != NON_LINEAR_POSITION_5)     ) return (INVALID_POSCONTROLMODE);

          if ((param < 6) && (BGVAR(u16_HdTune_Adv_Mode) == 2)         &&
               (VAR(AX0_u16_Pos_Control_Mode) != NON_LINEAR_POSITION_3) &&
               (VAR(AX0_u16_Pos_Control_Mode) != NON_LINEAR_POSITION_5)     ) return (INVALID_POSCONTROLMODE);
      }

      if ((BGVAR(s16_Pos_Tune_State) > 0) && (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)) return (AT_ACTIVE);
      if (BGVAR(u16_Hold) == 1) return (HOLD_MODE);
      if (ProcedureRunning(DRIVE_PARAM) != PROC_NONE) return OTHER_PROCEDURE_RUNNING;
      // Avoid using Unidirectional profile in P&D gear modes or external MC profile  
      if(((BGVAR(s16_Pos_Tune_Cycle) == 2) && (VAR(AX0_s16_Opmode) == 4))||
         ((BGVAR(s16_Pos_Tune_Cycle) == 2) && (BGVAR(u8_Comm_Mode) == 1)))
            return NOT_ALLOWED_UNIDIR_PROFILE;
      s32_temp = (long)((float)BGVAR(u32_Mspeed)*0.1);
      if(BGVAR(s32_V_Lim_Design) <= s32_temp) return VLIM_LESS_THAN_VCRUISE;

      if ((param == 1) || (param == 2) || (param == 3)  || (param == 4) || (param == 90) )
      {
         if ((s16_Number_Of_Parameters == 1) && (Enabled(drive))) return (DRIVE_ACTIVE);
         PosTuneBackup(drive);
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_SETUP;
         BGVAR(s16_Pos_tune_Mode) = (int)param;
      }
      else if ((param == 9) || (param == 10) || (param == 11))
      {
         if (!Enabled(drive)) return (DRIVE_INACTIVE);
         PosTuneBackup(drive);
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_CONT_INITIAL_COST_SETUP;
         BGVAR(s16_Pos_tune_Mode) = (int)param;
      }
      else if (param == 5)
      {  // execute easy tuning mode:
         if ((s16_Number_Of_Parameters == 1) && (Enabled(drive))) return (DRIVE_ACTIVE);
         if (VAR(AX0_s16_Opmode) != 8) return (INVALID_OPMODE);

         ResetHdTuneConfiguration(drive);

         if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
         {
            SalHdTuneAdvMode(2LL,drive);              // set hdtuneadvmode to 2
            BGVAR(s16_Pos_Tune_Igrav) = 0;
            BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 0;
         }
         else
         {
            if (BGVAR(u16_MTP_Mode_User) == 3)
            {
               SalHdTuneAdvMode(2LL,drive);
               BGVAR(s16_Pos_Tune_Igrav) = 0;
               BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 0;
            }
            else
            {
               SalHdTuneAdvMode(0LL,drive);
               BGVAR(s16_Pos_Tune_Igrav) = 1;
               BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 2;
            }
         }
         PosTuneBackup(drive);
         BGVAR(s16_Pos_Tune_Enable_Mode) = 1;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_SETUP;
         BGVAR(s16_Pos_tune_Mode) = 1;

      }
      else if (param == 6) // internal user defined trajectory
      {
         if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW)) return (NOT_AVAILABLE);
         ret_val = HdtuneConfigurationErrors(drive);
         if (ret_val != SAL_SUCCESS) return (ret_val);

         if ( (s16_av_backup != 0) && (s16_av_backup != 6) ) return (NOT_AVAILABLE);
         if ( (s16_igrav_backup != 0) && (s16_igrav_backup != 1) ) return (NOT_AVAILABLE);
         if ( (s16_knlafrc_backup != 0) && (s16_knlafrc_backup != 2) ) return (NOT_AVAILABLE);
         if ( (s16_stiff_backup != 0) && (s16_stiff_backup != 1) ) return (NOT_AVAILABLE);
         if ( (s16_cycle_backup != 0) /*&& (s16_cycle_backup != 2)*/ ) return (NOT_AVAILABLE); // unidirectional not released

         ResetHdTuneConfiguration(drive);
         
         SalHdTuneAdvMode(1LL,drive);                                // adv mode 1 - bundle mode - "comfort"
         PosTuneTrajMode(1LL,drive);                                 // select userdefined trajectory
         s16_temp = PosTuneTrajEn(1LL,drive);                        // start trajectory generator
         if (s16_temp != SAL_SUCCESS) return (s16_temp); 

         BGVAR(s16_HdTune_Av_Mode) = s16_av_backup;          
         BGVAR(s16_Pos_Tune_Igrav) = s16_igrav_backup;          
         BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = s16_knlafrc_backup;  
         PosTuneStiffnessCommand((long long)s16_stiff_backup,drive); 
         BGVAR(s16_Pos_Tune_Cycle) = s16_cycle_backup;            
         PosTuneLMJREnCommand((long long)s16_pos_tune_lmjr_backup,drive);          // en LMJR estimation during AT

         BGVAR(u16_HDTUNE_Sat_Mode) = 0;           // warning  on ICMD sat

         PosTuneBackup(drive);
         BGVAR(s16_Pos_Tune_Enable_Mode) = 1;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_SETUP;
         BGVAR(s16_Pos_tune_Mode) = 1;
      }
      else if (param == 7) // external user defined trajectory - express
      {
         if ( (VAR(AX0_s16_Opmode) != 8) && (VAR(AX0_s16_Opmode) != 4) ) return (INVALID_OPMODE);
         if ( (u16_Product == SHNDR) || (u16_Product == SHNDR_HW) ) return (NOT_AVAILABLE);

         ret_val = HdtuneConfigurationErrors(drive);
         if (ret_val != SAL_SUCCESS) return (ret_val);

         if ( (s16_cycle_backup != 0) /*&& (s16_cycle_backup != 2)*/ ) return (NOT_AVAILABLE); // unidirectional not released

         ResetHdTuneConfiguration(drive);

         SalHdTuneAdvMode(1LL,drive);                                // adv mode 1 - bundle mode - "comfort"
         PosTuneTrajMode(0LL,drive);                                 // select userdefined trajectory

         BGVAR(u16_HDTUNE_Sat_Mode) = 0;           // warning  on ICMD sat

         PosTuneBackup(drive);
         BGVAR(s16_Pos_Tune_Enable_Mode) = 1;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_SETUP;
         BGVAR(s16_Pos_tune_Mode) = 1;
      }
      else if (param == 8) // external user defined trajectory
      {
         if ( (VAR(AX0_s16_Opmode) != 8) && (VAR(AX0_s16_Opmode) != 4) ) return (INVALID_OPMODE);
         if ( (u16_Product == SHNDR) || (u16_Product == SHNDR_HW) ) return (NOT_AVAILABLE);

         ret_val = HdtuneConfigurationErrors(drive);
         if (ret_val != SAL_SUCCESS) return (ret_val);

         if ( (s16_av_backup != 0) && (s16_av_backup != 6) ) return (NOT_AVAILABLE);
         if ( (s16_igrav_backup != 0) && (s16_igrav_backup != 1) ) return (NOT_AVAILABLE);
         if ( (s16_knlafrc_backup != 0) && (s16_knlafrc_backup != 2) ) return (NOT_AVAILABLE);
         if ( (s16_stiff_backup != 0) && (s16_stiff_backup != 1) ) return (NOT_AVAILABLE);
         if ( (s16_cycle_backup != 0) /*&& (s16_cycle_backup != 2)*/ ) return (NOT_AVAILABLE); // unidirectional not released

         ResetHdTuneConfiguration(drive);

         SalHdTuneAdvMode(1LL,drive);                                // adv mode 1 - bundle mode - "comfort"
         PosTuneTrajMode(0LL,drive);                                 // select userdefined trajectory

         BGVAR(s16_HdTune_Av_Mode) = s16_av_backup;          
         BGVAR(s16_Pos_Tune_Igrav) = s16_igrav_backup;          
         BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = s16_knlafrc_backup;  
         PosTuneStiffnessCommand((long long)s16_stiff_backup,drive); 
         BGVAR(s16_Pos_Tune_Cycle) = s16_cycle_backup;            

         BGVAR(u16_HDTUNE_Sat_Mode) = 0;           // warning  on ICMD sat

         PosTuneBackup(drive);
         BGVAR(s16_Pos_Tune_Enable_Mode) = 1;
         BGVAR(s16_Pos_Tune_State) = POS_TUNE_SETUP;
         BGVAR(s16_Pos_tune_Mode) = 1;
      }
      else if (param == 100)
      {
         SalHdTuneAdvMode(2LL,drive);              // set hdtuneadvmode to 2 - force filter update according to data base
         BGVAR(s16_Pos_Tune_Igrav) = 0;
         BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 0;
         HdTuneGainCommand(1000LL,drive);          // HDTUNEGAIN 1
         BGVAR(s16_Pos_Tune_Mencres_Shr_Mode) = 0; // HDTUNEENCSHRMODE 0
         BGVAR(s16_HdTune_Av_Mode) = 0;            // hdtuneavmode 0
         BGVAR(s16_Pos_tune_Mode) = 100;
         PosTuneTrajEn(0LL,drive);                 // hdtunerefen 0
         PosTuneTrajMode(2LL,drive);               // hdtunereference 2
         HdTuneDefaultsCommand(0LL,drive);         // HDTUNEDEFAULTS 0
         PosTuneBackup(drive);
         BGVAR(s16_RTAT_State) = RTAT_SETUP;
         BGVAR(s16_Hdtune_Debug) = 3;
      }
      else
         return (NOT_AVAILABLE);
   }

   // "Postune x 1" will allow postune while drive in enable
   // Be sure motor is not moving

   if ((s16_Number_Of_Parameters > 1) && (s64_Execution_Parameter[1] == 1LL)) BGVAR(s16_Pos_Tune_Enable_Mode) = 1;

   return (SAL_SUCCESS);
}

void ResetHdTuneConfiguration(int drive)
{
   BGVAR(s64_Pos_Tune_AutoTraj_Pos_Lim) = 0; // default trajecotry limitations
   SalHdTuneAdvMode(2LL,drive);              // adv mode 2 - bundle mode
   BGVAR(s16_Pos_Tune_Igrav) = 0;            // no igrav tuning
   BGVAR(s16_Pos_Tune_KnlAfrc_Mode) = 0;     // no knlafrc tuning
   BGVAR(s16_HdTune_Av_Mode) = 0;            // no antivib
   BGVAR(s16_Pos_Tune_Cycle) = 0;            // bi-directional cycle
   HdTuneDefaultsCommand(0LL,drive);         // reset gains to defaults
   HdTuneDwellTimeCommand(200LL,drive);      // cycle min dwell time 200 mSec
   BGVAR(s32_Pos_Tune_C_Effort) = 100000;    // set effort to 100%
   BGVAR(s16_Pos_Tune_Mencres_Shr_Mode) = 0; // AT overwrites mencressht
   HdTuneGainCommand(1000LL,drive);          // initial AT gain unchanged
   PosTuneLMJREnCommand(0LL,drive);          // en LMJR estimation during AT
   BGVAR(s16_Pos_Tune_NlPeAff_Mode) = 1;     // do not tune nlpeaff module 
   PosTuneTrajEn(0LL,drive);                 // stop trajectory reference
   PosTuneTrajMode(2LL,drive);               // select auto trajectory mode
   BGVAR(u16_HDTUNE_Sat_Mode) = 1;           // fault on ICMD sat
   BGVAR(u16_HDTUNE_Save_Mode) = HDTUNESAVE_KEEP_RES_OVRRIDE_BKUP;    // accept results automatically
   PosTuneStiffnessCommand(0LL,drive);       // AT controls smooth filter
   BGVAR(u16_HDTUNE_Warn_Mode) = 0;          // cost 0 is warning only
   BGVAR(s16_Pos_Tune_Weight_User) = 50;     // set weight to 50%
}


int HdtuneConfigurationErrors(int drive)
{
   long s32_temp = (long)((float)BGVAR(u32_Mspeed)*0.1);
   // AXIS_OFF;
   
   if ((BGVAR(s16_Pos_Tune_State) > 0) && (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)) return (AT_ACTIVE);
   if(BGVAR(s16_DisableInputs) & MTP_READ_DIS_MASK) return NOT_AVAILABLE;
   if ((s16_Record_Flags & RECORD_ON_MASK) != 0) return (REC_ACTIVE);
   if (VAR(AX0_u16_Pos_Control_Mode) != NON_LINEAR_POSITION_5) return (INVALID_POSCONTROLMODE);
   if (BGVAR(u16_Hold) == 1) return (HOLD_MODE);
   if (BGVAR(s32_V_Lim_Design) <= s32_temp) return VLIM_LESS_THAN_VCRUISE;
      // Avoid using Unidirectional profile in P&D gear modes or external MC profile  
   if(((BGVAR(s16_Pos_Tune_Cycle) == 2) && (VAR(AX0_s16_Opmode) == 4))||
      ((BGVAR(s16_Pos_Tune_Cycle) == 2) && (BGVAR(u8_Comm_Mode) == 1)))
       return NOT_ALLOWED_UNIDIR_PROFILE;
   if ((VAR(AX0_s16_Opmode) != 8) && (VAR(AX0_s16_Opmode) != 4)) return (INVALID_OPMODE);
   if (ProcedureRunning(DRIVE_PARAM) != PROC_NONE) return OTHER_PROCEDURE_RUNNING;

   return (SAL_SUCCESS);
}


void CalcKnldMax(int drive,int s16_knld_max)
{
   float f_filtt1 =  1e-3*PosTuneReadParam(POS_TUNE_NLFILTT1_ID,drive,0),
         f_damping = PosTuneReadParam(POS_TUNE_NLFILT_DAMPING_ID,drive,0),
         f_knld_damping_0,f_knld_damping_35,f_knld_damping_60,f_knld_damping_85,
         f_interpolation,f_in_1,f_in_2,
         f_knld_1_0,f_knld_2_0,f_knld_1_35,f_knld_2_35,f_knld_1_60,f_knld_2_60,f_knld_1_85,f_knld_2_85;
   // AXIS_OFF;


    if (f_damping > 85) f_damping = 85;

   if (f_filtt1 > 30) f_filtt1 = 30;
   else if (f_filtt1 < 0.01) f_filtt1 = 0.01;

   if (f_filtt1 < 0.03)
   {
      f_in_1 = 0.01;
      f_in_2 = 0.03;
      if ((VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_4) ||
          (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5)   )
      {
        f_knld_1_0 = 525;
         f_knld_2_0 = 467;
         f_knld_1_35 = 573;
         f_knld_2_35 = 518;
         f_knld_1_60 = 600;
         f_knld_2_60 = 560;
         f_knld_1_85 = 600;
         f_knld_2_85 = 580;
      }
      else
      {
         f_knld_1_0 = 195;
         f_knld_2_0 = 186;
         f_knld_1_35 = 205;
         f_knld_2_35 = 196;
         f_knld_1_60 = 215;
         f_knld_2_60 = 208;
         f_knld_1_85 = 230;
         f_knld_2_85 = 230;
     }
   }
   else if (f_filtt1 < 0.1)
   {
      f_in_1 = 0.03;
     f_in_2 = 0.1;
      if ((VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_4) ||
          (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5)   )
      {
        f_knld_1_0 = 467;
         f_knld_2_0 = 347;
         f_knld_1_35 = 518;
         f_knld_2_35 = 399;
         f_knld_1_60 = 560;
         f_knld_2_60 = 451;
         f_knld_1_85 = 580;
         f_knld_2_85 = 443;
      }
      else
      {
         f_knld_1_0 = 186;
         f_knld_2_0 = 162;
         f_knld_1_35 = 196;
         f_knld_2_35 = 179;
         f_knld_1_60 = 208;
         f_knld_2_60 = 187;
         f_knld_1_85 = 230;
         f_knld_2_85 = 208;
      }
   }
   else if (f_filtt1 < 0.3)
   {
     f_in_1 = 0.1;
      f_in_2 = 0.3;
      if ((VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_4) ||
          (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5)   )
      {
         f_knld_1_0 = 347;
         f_knld_2_0 = 205;
         f_knld_1_35 = 399;
         f_knld_2_35 = 258;
         f_knld_1_60 = 451;
         f_knld_2_60 = 312;
         f_knld_1_85 = 443;
         f_knld_2_85 = 240;
      }
      else
      {
         f_knld_1_0 = 162;
         f_knld_2_0 = 122;
         f_knld_1_35 = 179;
         f_knld_2_35 = 140;
         f_knld_1_60 = 187;
         f_knld_2_60 = 158;
         f_knld_1_85 = 208;
         f_knld_2_85 = 184;
      }
   }
   else if (f_filtt1 < 1)
   {
      f_in_1 = 0.3;
      f_in_2 = 1;
      if ((VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_4) ||
          (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5)   )
      {
         f_knld_1_0 = 205;
         f_knld_2_0 = 99;
         f_knld_1_35 = 258;
         f_knld_2_35 = 131;
         f_knld_1_60 = 312;
         f_knld_2_60 = 166;
         f_knld_1_85 = 240;
         f_knld_2_85 = 148;
      }
      else
      {
         f_knld_1_0 = 122;
         f_knld_2_0 = 70;
         f_knld_1_35 = 140;
         f_knld_2_35 = 90;
         f_knld_1_60 = 158;
         f_knld_2_60 = 110;
         f_knld_1_85 = 184;
         f_knld_2_85 = 143;
     }
   }
   else if (f_filtt1 < 3)
   {
      f_in_1 = 1;
      f_in_2 = 3;
      if ( (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_4) ||
           (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5)   )
      {
         f_knld_1_0 = 99;
         f_knld_2_0 = 41;
         f_knld_1_35 = 131;
         f_knld_2_35 = 59;
         f_knld_1_60 = 166;
         f_knld_2_60 = 84;
         f_knld_1_85 = 148;
         f_knld_2_85 = 120;
      }
      else
      {
          f_knld_1_0 = 70;
          f_knld_2_0 = 35;
          f_knld_1_35 = 90;
          f_knld_2_35 = 45;
          f_knld_1_60 = 110;
          f_knld_2_60 = 62;
          f_knld_1_85 = 143;
          f_knld_2_85 = 98;
      }
   }
   else if (f_filtt1 < 10)
   {
      f_in_1 = 3;
      f_in_2 = 10;
      if ((VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_4) ||
          (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5)   )
      {
         f_knld_1_0 = 41;
         f_knld_2_0 = 13;
         f_knld_1_35 = 59;
         f_knld_2_35 = 20;
         f_knld_1_60 = 84;
         f_knld_2_60 = 31;
         f_knld_1_85 = 120;
         f_knld_2_85 = 69;
      }
      else
      {
         f_knld_1_0 = 35;
         f_knld_2_0 = 14;
         f_knld_1_35 = 45;
         f_knld_2_35 = 19;
         f_knld_1_60 = 62;
         f_knld_2_60 = 28;
         f_knld_1_85 = 98;
         f_knld_2_85 = 50;
     }
   }
   else
   {
      f_in_1 = 10;
      f_in_2 = 30;
      if ((VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_4) ||
          (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5)   )
      {
         f_knld_1_0 = 13;
         f_knld_2_0 = 4;
         f_knld_1_35 = 20;
         f_knld_2_35 = 7;
         f_knld_1_60 = 31;
         f_knld_2_60 = 11;
         f_knld_1_85 = 69;
         f_knld_2_85 = 46;
      }
      else
      {
         f_knld_1_0 = 14;
         f_knld_2_0 = 5;
         f_knld_1_35 = 19;
         f_knld_2_35 = 8;
         f_knld_1_60 = 28;
         f_knld_2_60 = 12;
         f_knld_1_85 = 50;
         f_knld_2_85 = 25;
      }
   }

   f_knld_damping_0  = f_knld_1_0 + (f_knld_2_0-f_knld_1_0)/(f_in_2-f_in_1)*(f_filtt1-f_in_1);
   f_knld_damping_35 = f_knld_1_35 + (f_knld_2_35-f_knld_1_35)/(f_in_2-f_in_1)*(f_filtt1-f_in_1);
   f_knld_damping_60  = f_knld_1_60 + (f_knld_2_60-f_knld_1_60)/(f_in_2-f_in_1)*(f_filtt1-f_in_1);
   f_knld_damping_85 = f_knld_1_85 + (f_knld_2_85-f_knld_1_85)/(f_in_2-f_in_1)*(f_filtt1-f_in_1);

   if (f_damping <= 35)
   {
      f_interpolation = (f_knld_damping_35 - f_knld_damping_0)/35;
      BGVAR(s16_Knld_Max) =(int)(f_knld_damping_0 + f_interpolation*f_damping);
   }
   else if (f_damping <= 60)
   {
      f_interpolation = (f_knld_damping_60 - f_knld_damping_35)/25;
      BGVAR(s16_Knld_Max) =(int)(f_knld_damping_35 + f_interpolation*(f_damping-35));
   }
   else
   {
      f_interpolation = (f_knld_damping_85 - f_knld_damping_60)/25;
      BGVAR(s16_Knld_Max) =(int)(f_knld_damping_60 + f_interpolation*(f_damping-60));
   }
   if (BGVAR(s16_Knld_Max) > s16_knld_max) BGVAR(s16_Knld_Max) = s16_knld_max;

}

//**********************************************************
// Function Name: RowInEasyTuningEntries
// Description: Find motor row in EasyTuning table, by u32_MotorFileName && u32_Pwm_Freq
// Author: Udi
//**********************************************************

int RowInEasyTuningEntries(void)
{
   int s16_row = 0;
   unsigned long u32_temp;

   if (BGVAR(u16_HdTune_Adv_Mode) == 0) return -1;

   do{
         if ((u16_Product == SHNDR) || (u16_Product == SHNDR_HW))
            u32_temp = s_Easy_Tuning_Data[s16_row].motor_file_name_se;
         else
            u32_temp = s_Easy_Tuning_Data[s16_row].motor_file_name_cdhd;

         ++s16_row;
     } while ((s16_row < NUMBER_OF_EASY_TUNING_ENTRIES) && (u32_temp != BGVAR(u32_MotorFileName)));

   if (u32_temp != BGVAR(u32_MotorFileName)) return -1;
   --s16_row;
   return s16_row;
}


int SetTorqueFilterAccordingToMotor(int drive,int mode)
{
  // mode 0 = calc fiilter values and knldmax
  // mode 1 = update gain ratios
  // mode 2 = init value with lmjr = 10
  // mode 3 = calc knldmax only
   int  ret_val, s16_row,s16_data_t1_lmjr_0,s16_data_t1_lmjr_10,s16_data_damping_lmjr_0,s16_data_damping_lmjr_10,s16_knld_max = 125;
   float f_interpolation,f_filtt1,f_filtdamping,f_sqrt_j_total_no_load,f_sqrt_j_toal_lmjr_10,f_sqrt_j_total_identified,f_interpolation2;
   long s32_lmjr;
   s16_row = RowInEasyTuningEntries();
   // AXIS_OFF;

   if (s16_row < 0 )
   {
      // calc index = micont*mkt/mj
      // calc s16_data_t1_lmjr_0 with polynom 7.6211e-10*index^2-1.3776e-05*index+0.2106
      // calc s16_data_t1_lmjr_10 with polynom 2.6855e-09*index^2+5.4053e-05*index+1.3264
      // damping  9.0274e-09*index^2+9.0411e-04*index+13.527
      // after 16khz bug fix
      // calc s16_data_t1_lmjr_0 with polynom 8.73e-10*index^2-6.3822e-05*index+1.2366
      // calc s16_data_t1_lmjr_10 with polynom 3.7433e-09*index^2-3.1601e-04*index+7.7007

      // data obtained by fitting the polynom to exsisting data from schnider project

      f_interpolation = (float)BGVAR(s32_Motor_I_Cont)*(float)BGVAR(u32_Motor_Kt)/(float)BGVAR(s32_Motor_J);
      s16_data_t1_lmjr_0 = (int)(8.7300e-7*f_interpolation*f_interpolation-6.3822e-02*f_interpolation+1236.6);
      s16_data_t1_lmjr_10 = (int)(3.7433e-06*f_interpolation*f_interpolation-3.1601e-01*f_interpolation+7700.7);

      // no intepolation for damping - depends on filter t1 parameter
      s16_data_damping_lmjr_0 = s16_data_damping_lmjr_10 = 15; // avoid complier warning - value not used by sw

      if (BGVAR(s32_Motor_J) < 10)
      {
         if (s16_data_t1_lmjr_0 > 200) s16_data_t1_lmjr_0 = 200;
         if (s16_data_t1_lmjr_10 > 1500) s16_data_t1_lmjr_10 = 1500;
      }
      else
      {
         if (s16_data_t1_lmjr_0 > 2000) s16_data_t1_lmjr_0 = 2000;
         if (s16_data_t1_lmjr_10 > 15000) s16_data_t1_lmjr_10 = 15000;
      }

      if (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_4) s16_knld_max = 150;
      else if (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5) s16_knld_max = 220;

      ret_val = 0;
   }
   else
   {

      if (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5) // se/ cdhd pro2 bundles
      {
         s16_data_t1_lmjr_0 = s_Easy_Tuning_Data[s16_row].filtt1_lmjr_0;
         s16_data_t1_lmjr_10 = s_Easy_Tuning_Data[s16_row].filtt1_lmjr_10;
         s16_data_damping_lmjr_0 = s_Easy_Tuning_Data[s16_row].filtdamping_lmjr_0;
         s16_data_damping_lmjr_10 = s_Easy_Tuning_Data[s16_row].filtdamping_lmjr_10;
         s16_knld_max = s_Easy_Tuning_Data[s16_row].knld_max_lmjr_0;
      }
      else // poscontrolmode 2 - se bundles
      {
         s16_data_t1_lmjr_0 = s_Easy_Tuning_Data[s16_row].filtt1_lmjr_0_poscontrol2;
         s16_data_t1_lmjr_10 = s_Easy_Tuning_Data[s16_row].filtt1_lmjr_10_poscontrol2;
         s16_data_damping_lmjr_0 = s_Easy_Tuning_Data[s16_row].filtdamping_lmjr_0_poscontrol2;
         s16_data_damping_lmjr_10 = s_Easy_Tuning_Data[s16_row].filtdamping_lmjr_10_poscontrol2;
         s16_knld_max = s_Easy_Tuning_Data[s16_row].knld_max_lmjr_0_poscontrol2;
      }

      ret_val = 1;
   }

   if (mode == 1)
   {
      if (ret_val == 0)
      {
         BGVAR(f_Ki_Kp_Ratio) = 0.43;
         BGVAR(f_Kp_Kiv_Ratio) = 1.28;
      }
      else
      {
         if (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_5) // se/ cdhd pro2 bundles
         {
         BGVAR(f_Ki_Kp_Ratio) = 0.001*(float)s_Easy_Tuning_Data[s16_row].ki_kp_ratio;
         BGVAR(f_Kp_Kiv_Ratio) = 0.001*(float)s_Easy_Tuning_Data[s16_row].kp_kiv_ratio;
      }
         else // poscontrolmode 2 - se bundles
         {
            BGVAR(f_Ki_Kp_Ratio) = 0.001*(float)s_Easy_Tuning_Data[s16_row].ki_kp_ratio_poscontrol2;
            BGVAR(f_Kp_Kiv_Ratio) = 0.001*(float)s_Easy_Tuning_Data[s16_row].kp_kiv_ratio_poscontrol2;
         }

      }
      return (ret_val);
   }

   if (mode == 2) s32_lmjr = 10000;
   else s32_lmjr = BGVAR(u32_LMJR_User);
   if (s32_lmjr < 500) s32_lmjr = 0;

   // interpolate LMJR - special case where we do linear interpolation
   if ((ret_val == 1)                                                                                                                 &&
       ((s_Easy_Tuning_Data[s16_row].motor_file_name_se == 17930006) || (s_Easy_Tuning_Data[s16_row].motor_file_name_se == 17931006)) &&
       (s32_lmjr < 10000)                                                                                                    )
   {
      f_interpolation = s32_lmjr/10000.0;// linear interpolation (BGVAR(u32_LMJR_User) - 0)/(10000-0)
      if (f_interpolation > 1) f_interpolation = 1;

      f_filtt1 = (float)s16_data_t1_lmjr_0 + f_interpolation*((float)s16_data_t1_lmjr_10 - (float)s16_data_t1_lmjr_0);
      f_filtdamping = (float)s16_data_damping_lmjr_0 + f_interpolation*((float)s16_data_damping_lmjr_10 - (float)s16_data_damping_lmjr_0);
      if (f_filtt1 < (float)s16_data_t1_lmjr_0) f_filtt1 = (float)s16_data_t1_lmjr_0;
      if (f_filtt1 > (float)s16_data_t1_lmjr_10) f_filtt1 = (float)s16_data_t1_lmjr_10;

      if (f_filtdamping < (float)s16_data_damping_lmjr_0) f_filtdamping = (float)s16_data_damping_lmjr_0;
   }
   else
   {
      f_sqrt_j_total_no_load = sqrt(1e-6*BGVAR(s32_Motor_J));
      f_sqrt_j_toal_lmjr_10 = sqrt(1e-6*BGVAR(s32_Motor_J)* 11);
      f_sqrt_j_total_identified = sqrt(1e-6*BGVAR(s32_Motor_J)*(0.001*(float)s32_lmjr+1));

      f_interpolation = ((float)s16_data_t1_lmjr_10 - (float)s16_data_t1_lmjr_0)/(f_sqrt_j_toal_lmjr_10-f_sqrt_j_total_no_load);
      f_interpolation2 = (float)s16_data_t1_lmjr_0 - f_sqrt_j_total_no_load*f_interpolation;
      f_filtt1 = f_interpolation*f_sqrt_j_total_identified+f_interpolation2;

      f_interpolation = ((float)s16_data_damping_lmjr_10 - (float)s16_data_damping_lmjr_0)/(f_sqrt_j_toal_lmjr_10-f_sqrt_j_total_no_load);
      f_interpolation2 = (float)s16_data_damping_lmjr_0 - f_sqrt_j_total_no_load*f_interpolation;
      f_filtdamping = f_interpolation*f_sqrt_j_total_identified+f_interpolation2;
   }

   //Limit damping result to damping@LMJR_0 if damping@LMJR_0 > damping@LMJR_10
   //This is to avoid negative damping results
   if ((float)s16_data_damping_lmjr_0 > (float)s16_data_damping_lmjr_10)
   {
        if (f_filtdamping < (float)s16_data_damping_lmjr_10) f_filtdamping = (float)s16_data_damping_lmjr_10;
   }
   else
   {
        if (f_filtdamping > (float)s16_data_damping_lmjr_10) f_filtdamping = (float)s16_data_damping_lmjr_10;
   }

   if (f_filtdamping > 85) f_filtdamping = 85;
   if (mode != 3)
   {
      if (ret_val == 0)
      {
         if (VAR(AX0_u16_Pos_Control_Mode) < NON_LINEAR_POSITION_4)
            f_filtt1 = f_filtt1 * 0.5;
         else
            f_filtt1 = f_filtt1 * 0.75;
         PosTuneWriteParam(POS_TUNE_NLFILTT1_AND_DAMPING_ID,(long)f_filtt1,drive);
      }
      else
      {
         PosTuneWriteParam(POS_TUNE_NLFILTT1_ID,(long)f_filtt1,drive);
         PosTuneWriteParam(POS_TUNE_NLFILT_DAMPING_ID,(long)f_filtdamping,drive);
      }

      PositionConfig(drive,1);
      VAR(AX0_s16_Cycle_Bits) |= CYCLE_PARAM_UPDATE;
   }
   CalcKnldMax(drive,s16_knld_max);
   return (ret_val);
}

int SalHdTuneContModeCommand(long long param,int drive)
{  // 0 - knlusergain
   // 1 - knlusergain + knliv
   // 2 - all gains
   // AV according to hdtuneavmode + if gain is not 0

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((param < 0) || (param > 2)) return (VALUE_OUT_OF_RANGE);
   if ((BGVAR(s16_Pos_Tune_State) > 0) && (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)) return (AT_ACTIVE);
   BGVAR(s16_HDtune_Cont_Mode) = (int)param;

   return (SAL_SUCCESS);
}

int SalHdTuneAvModeCommand(long long param,int drive)
{  // 0 - no av
   // 1 - av2 only
   // 2 - av3 only
   // 3 - av2 and av3

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //if ((param < 0) || (param > 6)) return (VALUE_OUT_OF_RANGE);
   // support only 0 and 6 - all other modes are not used anyway
   if ((param != 0) && (param != 6)) return (VALUE_OUT_OF_RANGE);

   if ((BGVAR(s16_Pos_Tune_State) > 0) && (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)) return (AT_ACTIVE);
   BGVAR(s16_HdTune_Av_Mode) = (unsigned int)param;

   return (SAL_SUCCESS);
}

int SalHdTuneAdvMode(long long param,int drive)
{  // 0 - no search dependencies
   // 1 - knliv = 1.25*knlp knli = 0.5*knlp
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((BGVAR(s16_Pos_Tune_State) > 0) && (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)) return (AT_ACTIVE);
   BGVAR(u16_HdTune_Adv_Mode) = (unsigned int)param;

   return (SAL_SUCCESS);
}




//**********************************************************
// Function Name: HdTuneDwellTimeCommand
// Description:
//       Set u16_HDTUNE_Dwell_Time_mS if AutoTune NOT active
// Author:
//**********************************************************
int HdTuneDwellTimeCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((BGVAR(s16_Pos_Tune_State) > 0) && (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)) return (AT_ACTIVE);
   BGVAR(u16_HDTUNE_Dwell_Time_mS) = (unsigned int)param;
   return (SAL_SUCCESS);
}


int HdTuneGainCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((BGVAR(s16_Pos_Tune_State) > 0) && (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)) return (AT_ACTIVE);
   BGVAR(u16_HdTune_Gain) = (unsigned int)param;
   return (SAL_SUCCESS);
}

int HdTuneDefaultsCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((BGVAR(s16_Pos_Tune_State) > 0) && (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)) return (AT_ACTIVE);
   BGVAR(u16_HdTune_Defaults) = (unsigned int)param;
   return (SAL_SUCCESS);
}



int PosTuneTableCommand(int drive)
{
   static int s16_row = 0;
   int s16_column = 0,s16_temp;
   long long s64_temp;
   // AXIS_OFF;

   if (u8_Output_Buffer_Free_Space < 80) return (SAL_NOT_FINISHED);

   if ((BGVAR(s16_Pos_Tune_State) >= POS_TUNE_IDLE) && (BGVAR(s16_Pos_Tune_State) <= POS_TUNE_IGRAV_SETUP))
   {
      PrintString("No Data\nCCntr ",0);PrintUnsignedInteger(BGVAR(u16_Cycle_Counter));
      PrintCrLf();
      s16_row = 0;
      return (SAL_SUCCESS);
   }

   if ((s16_row<TUNE_SEARCH_TABLE_MAX_ROW) && (s16_row <= BGVAR(s16_Tune_Table_Size)))
   {
      for (s16_column = 0;((s32_TuneSearchTable[0][s16_column] != POS_TUNE_NO_ID) && (s16_column < TUNE_SEARCH_TABLE_MAX_COL));++s16_column)
      {
         if (s16_row != 0)
            PrintSignedLong(s32_TuneSearchTable[s16_row][s16_column]);
         else
            PrintPosTuneGain(s32_TuneSearchTable[s16_row][s16_column]);
         PrintChar(SPACE);
      }
      if (s16_row != 0)
      {
         if (s64_TuneSearchTableResult[s16_row] == 0x7FFFFFFFFFFFFFF1) PrintString("------TQF-------",0);
         else if (s64_TuneSearchTableResult[s16_row] == 0x7FFFFFFFFFFFFFF2) PrintString("------VGSUM-------",0);
         else if (s64_TuneSearchTableResult[s16_row] == 0x7FFFFFFFFFFFFFF3) PrintString("------TQFRT-------",0);
         else PrintDecInAsciiHex(s64_TuneSearchTableResult[s16_row],16);
         PrintChar(SPACE);PrintString(DecimalPoint32ToAscii(EffortToPercent(drive,u32_PosTuneSearchEffort[s16_row])),0);
         PrintString(" % ",0);PrintDecInAsciiHex(s64_PosTuneSearchTQF[s16_row],10);
      }
      PrintCrLf();
      ++s16_row;
      return (SAL_NOT_FINISHED);
   }

   s16_temp = BGVAR(s16_Tune_Table_Size) + 1;
   if (s16_row == s16_temp)
   {
      s16_column = 0;
      PrintString("Best:\n",0);
      while((s32_TuneSearchTable[0][s16_column] != POS_TUNE_NO_ID) && (s16_column < TUNE_SEARCH_TABLE_MAX_COL))
      {
         PrintSignedLong(s32_TuneSearchTableBest[s16_column]);
         PrintChar(SPACE);
         s16_column++;
      }
      ++s16_row;
      return (SAL_NOT_FINISHED);
   }

   s16_temp = BGVAR(s16_Tune_Table_Size) + 2;

   if (s16_row == s16_temp)
   {
      PrintDecInAsciiHex(BGVAR(s64_Tune_Min_Cost),16);
      PrintString("\nEffort(Lim/Act): ",0);PrintSignedLong(EffortToPercent(drive,(500+LVAR(AX0_u32_At_Control_Effort_Tresh))/1000));//PrintString(DecimalPoint32ToAscii(EffortToPercent(drive,LVAR(AX0_u32_At_Control_Effort_Tresh))),0);
      PrintString(" , ",0);PrintSignedLong(EffortToPercent(drive,(500+BGVAR(u32_At_Control_Effort_Max_Captured))/1000));//PrintString(DecimalPoint32ToAscii(EffortToPercent(drive,BGVAR(u32_At_Control_Effort_Max_Captured))),0);
      PrintString(" %\nDmax: ",0);PrintSignedInteger(BGVAR(s16_Knld_Max));
      PrintString(" UGmax: ",0);PrintSignedLong(BGVAR(s32_At_Usergain_Max));PrintString(" UGmin: ",0);PrintSignedLong(BGVAR(s32_At_Usergain_Min));;
      ++s16_row;
      return (SAL_NOT_FINISHED);
   }

   s16_temp = BGVAR(s16_Tune_Table_Size) + 3;
   if (s16_row == s16_temp)
   {
      PrintString(", STicks: ",0);PrintUnsignedInteger(BGVAR(u16_StandStill_Ticks));
      PrintString(", CCntr: ",0);PrintUnsignedInteger(BGVAR(u16_Cycle_Counter));
      PrintString("\nTQF(Lim/Act): ",0); PrintDecInAsciiHex(((5*BGVAR(s64_Tune_Min_Tqf))>>1)>>8,8);
      PrintString(" , ",0);PrintDecInAsciiHex(BGVAR(s64_At_TQF)>>8,8);
      ++s16_row;
      return (SAL_NOT_FINISHED);
   }

   s16_temp = BGVAR(s16_Tune_Table_Size) + 4;
   if (s16_row == s16_temp)
   {
      if (BGVAR(s16_Cycle_StandStill_Found) == 0)
         PrintString("\nSStillPos: NA",0);
      else
      {
         s64_temp = MultS64ByFixS64ToS64(BGVAR(s64_Cycle_Standstill_Pos),
                                      BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                                      BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);
         PrintString(" \nSStillPos: ",0);PrintSignedLongLong(s64_temp);PrintString(" [",0); PrintString((char *)s8_Units_Pos, 0);
         PrintString("]\nMaxMin: ",0);PrintSignedLong(BGVAR(s32_Pos_Tune_Search_Max_Min_Ratio));
      }
      ++s16_row;
      return (SAL_NOT_FINISHED);
   }

   s16_temp = BGVAR(s16_Tune_Table_Size) + 5;
   if (s16_row == s16_temp)
   {
      s16_column = 0;
      PrintString("\nmTQF: ",0);
      while((s32_TuneSearchTable[0][s16_column] != POS_TUNE_NO_ID)  && (s16_column < TUNE_SEARCH_TABLE_MAX_COL))
      {
         PrintSignedLong(s32_TuneSearchTableMinTQF[s16_column]);
         PrintChar(SPACE);
         s16_column++;
      }
      PrintString(" , fTQF: ",0);
      s16_column = 0;
      while((s32_TuneSearchTable[0][s16_column] != POS_TUNE_NO_ID) && (s16_column < TUNE_SEARCH_TABLE_MAX_COL))
      {
         PrintSignedLong(s32_TuneSearchTableFirstTQF[s16_column]);
         PrintChar(SPACE);
         s16_column++;
      }

      if ((BGVAR(s16_Pos_tune_Mode) >= 9) &&
          (BGVAR(s16_Pos_tune_Mode) <= 11)   )
      {
         PrintString("\nCT: ",0);
         PrintSignedInteger(BGVAR(s16_Cont_Tune_Improvement_Cntr));
      }
      ++s16_row;
      return (SAL_NOT_FINISHED);
   }

   s16_temp = BGVAR(s16_Tune_Table_Size) + 6;
   if (s16_row == s16_temp)
   {
       PrintString("\nFlg: ",0);
       if ((VAR(AX0_s16_Cycle_Bits)  & MIN_MAX_RATIO_HIGH) != 0) PrintString("minmax, ",0);
       if ((BGVAR(s32_Pos_Tune_Bits) & TQF_FUNC_ENABLED) != 0) PrintString("TQF, ",0);
       if ((VAR(AX0_s16_Cycle_Bits) & POS_TUNE_MIN_SETTLE_TIME_SCALE) != 0) PrintString("MST, ",0);
       if ((VAR(AX0_s16_Cycle_Bits) & POS_TUNE_MIN_OVERSHOOT_SCALE) != 0) PrintString("MOS, ",0);
       if ((VAR(AX0_AutoTune_Bits)  & AT_NO_ACC_OPTIMIZE_MASK) != 0) PrintString("NAO, ",0);
       PrintSignedInteger(BGVAR(s16_Pos_Tune_Weight));PrintString(" MSK: ",0);
       PrintUnsignedInteger(BGVAR(u16_Cycle_Counter_Mask));
       PrintCrLf();

       if ((BGVAR(s16_Pos_Tune_State) == POS_TUNE_STOPPED_FAULT)        ||
           (BGVAR(s16_Pos_Tune_State) == POS_TUNE_CYCLE_CNT_TOUT_FAULT) ||
           (BGVAR(s16_Pos_Tune_State) == POS_TUNE_OVER_COUNT_FAULT))
       {
           PrintString("SC: ",0); PrintSignedInteger(s16_Pos_Tune_State_Captured);
           PrintCrLf();
       }
       PrintString("KNLIV Step:",0);
       PrintSignedLong(s16_PosTune_Knliv_Step);PrintChar(SPACE);
       PrintSignedLong(s16_PosTune_Knliv_Step2);PrintCrLf();
       PrintString("TQFe:",0);PrintSignedLong(BGVAR(u32_Tqf_Extrema_Cntr));PrintCrLf();
       ++s16_row;
      return (SAL_NOT_FINISHED);
   }

   if (BGVAR(u16_HdTune_Defaults) != 0)
   {
      PrintString("InitC: ",0);
      PrintDecInAsciiHex(BGVAR(s64_Cost_Before_At),16);
      PrintCrLf();
   }

   PrintString("LMJR Cycle User Final: ",0);
   PrintSignedLong(BGVAR(s32_Cycle_LMJR));PrintChar(SPACE);PrintSignedLong(BGVAR(u32_LMJR_User));PrintChar(SPACE);PrintSignedLong(BGVAR(u32_LMJR));
   PrintCrLf();
   PrintString("D P I IV: ",0);
   PrintSignedLong(BGVAR(u32_Nl_Kpd));PrintChar(SPACE);PrintSignedLong(BGVAR(u32_Nl_Kpp));PrintChar(SPACE);PrintSignedLong(BGVAR(u32_Nl_Kpi));PrintChar(SPACE);PrintSignedLong(BGVAR(u32_Nl_Kpiv));
   PrintCrLf();
   PrintString("User D P I IV: ",0);
   PrintSignedLong(BGVAR(u32_Nl_Kpd_User));PrintChar(SPACE);PrintSignedLong(BGVAR(u32_Nl_Kpp_User));PrintChar(SPACE);PrintSignedLong(BGVAR(u32_Nl_Kpi_User));PrintChar(SPACE);PrintSignedLong(BGVAR(u32_Nl_Kpiv_User));
   PrintCrLf();
   s16_row = 0;
   return (SAL_SUCCESS);
}

int HdTuneGuiCommand(int drive)
{// I/F between hdtune and GUI
 // read command with one parameter HDTUNEGUI <index>
 // (password protected)
 //
 // index       meaning
 //   1          speed that will be used in hdtunereference 2 mode (auto trajectory)
 //   2          max distance that will be used in hdtunereference 2 mode (auto trajectory)
 //   3          max acc in internal trajectory
 //   4          LVAR(AX0_s32_Counts_Per_Rev)
 //   10         identitifed cycle start\end point
 //   11         identitifed cycle over all time
 //   12         identitifed cycle dwell time
 //   13         trajectory validation
 //   20         load AT defaults  (if AT inactive)
 //   21         PosTuneBackup     (if AT inactive)
 //   22         PosTuneRestore    (if AT inactive)
 //   30         AT lmjr variable u32_AT_LMJR
 //   31         AT acc max/min based on u32_AT_LMJR
 //   40         set\get BGVAR(s64_Pos_Tune_AutoTraj_Pos_Lim)
 //   50         indicate if CDHD budle is supported by AT
 //   61         AT test motion speed
 //   62         AT test motion distance
 //   63         AT test motion ACC (=DEC)
   int s16_index = (int)s64_Execution_Parameter[0],
       s16_hdtune_data_ready = 0;
   long long s64_temp,s64_auto_tune_active_flag;
   float f_temp,f_j_tot,f_nom_acc,f_max_acc;
   unsigned long u32_temp;
   unsigned long long u64_at_min_acc_int,u64_at_max_acc_int,u64_max_acc_dec;
   // AXIS_OFF;

   if ((BGVAR(s16_Pos_Tune_State) >= POS_TUNE_LMJR_EST)                              &&
       (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)                                    ) s16_hdtune_data_ready = 1;

   PosTuneActiveCommand(&s64_auto_tune_active_flag,0);


   if ((s16_Number_Of_Parameters == 2) && (s16_index != 40) ) return (NOT_AVAILABLE);


   if (s64_Execution_Parameter[0] > 32767) return (VALUE_OUT_OF_RANGE);
   switch (s16_index)
   {
      case  1: // max auto traj speed command
         if (s16_hdtune_data_ready)
            s64_temp = (long long)BGVAR(u32_PosTuneTraj_Spd_Int)[0];
         else
            s64_temp = (long long)(min(BGVAR(s32_V_Lim_Design), (float)BGVAR(u32_Mspeed)*0.1));

         s64_temp  =    MultS64ByFixS64ToS64((long long)s64_temp,
                                BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);
         PrintSignedLongLong(s64_temp);
         PrintChar(SPACE);
         PrintChar('[');
         PrintString((char *)s8_Units_Vel_Out_Loop,0);
         PrintChar(']');
         break;

      case 2: // max auto traj dist command
         if (s16_hdtune_data_ready)
            s64_temp = BGVAR(s64_PosTuneTraj_Pos_Int)[0];
         else
         {
            s64_temp = 12884901885LL; // 3 motor rev
            if (BGVAR(s64_Pos_Tune_AutoTraj_Pos_Lim) != 0) s64_temp = BGVAR(s64_Pos_Tune_AutoTraj_Pos_Lim);
         }
         s64_temp =      MultS64ByFixS64ToS64(s64_temp,
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);
         // Print internal trajectory DISTANCE
         PrintSignedLongLong(s64_temp);
         PrintChar(SPACE);
         PrintChar('[');
         PrintString((char *)s8_Units_Pos,0);
         PrintChar(']');
         break;

      case  3:  // max auto traj acc command
         if (s16_hdtune_data_ready)
            s64_temp = (long long)BGVAR(u64_PosTuneTraj_AccDec_Int)[0];
         else
         {
            if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) s64_temp = MAX_ACC_DEC_TSP250;
            else s64_temp = MAX_ACC_DEC_TSP125;
            s64_temp = s64_temp>>4LL; // 62500 rpm/sec
         }

         s64_temp =       MultS64ByFixS64ToS64(s64_temp,
                                   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);

         PrintSignedLongLong(s64_temp);
         PrintChar(SPACE);
         PrintChar('[');
         PrintString((char *)s8_Units_Acc_Dec_For_Pos,0);
         PrintChar(']');
         break;

      case 4: // LVAR(AX0_s32_Counts_Per_Rev)
         PrintSignedLong(LVAR(AX0_s32_Counts_Per_Rev));
         break;

      case 10: // stand still position
         if ((s64_auto_tune_active_flag) && (BGVAR(s16_Cycle_StandStill_Found) != 0))
            s64_temp = BGVAR(s64_Cycle_Standstill_Pos);
         else
            s64_temp = 0;
         s64_temp =      MultS64ByFixS64ToS64(s64_temp,
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);
         // Print internal trajectory DISTANCE
         PrintSignedLongLong(s64_temp);
         PrintChar(SPACE);
         PrintChar('[');
         PrintString((char *)s8_Units_Pos,0);
         PrintChar(']');
         break;

      case 11: // identified cycle time
         if (s64_auto_tune_active_flag)
            PrintUnsignedLong(BGVAR(u32_Cycle_OverAll_Counter_GUI));
         else
            PrintUnsignedLong(0);
         PrintString(" ms",0);
         break;

      case 12: // identified cycle dwell time
         if (s64_auto_tune_active_flag)
            PrintUnsignedInteger(BGVAR(u16_StandStill_Ticks));
         else
            PrintUnsignedInteger(0);
         PrintString(" ms",0);
         break;

      case 13: // trajectory validation
         //calc vel limit
         u32_temp = (unsigned long)(0.1*(float)BGVAR(u32_Mspeed));

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
          f_temp = (f_temp) *(4294967296.0/(float)BGVAR(u64_PosTuneTraj_AccDec)[0] + 4294967296.0/(float)BGVAR(u64_PosTuneTraj_AccDec)[1]);

          s64_temp = 0;
          if(llabs(BGVAR(s64_PosTuneTraj_Pos)[0]) != llabs(BGVAR(s64_PosTuneTraj_Pos)[1]))
          {
            PrintString("Dist Not Eq",0);
            s64_temp = 1;
          }
          if ((BGVAR(u32_PosTuneTraj_Spd)[0] <  u32_temp) || (BGVAR(u32_PosTuneTraj_Spd)[1] <  u32_temp))
          {
            PrintString("\nVc too low",0);
            s64_temp = 1;
          }

          if(llabs(BGVAR(s64_PosTuneTraj_Pos)[0]) < (long long)f_temp)
          {
            PrintString("\nDist short or acc low",0);
            s64_temp = 1;
          }

         //Check if Bi directional but the 2 positions are the same sign
         if((BGVAR(s16_Pos_Tune_Cycle) < 2) &&
            (((BGVAR(s64_PosTuneTraj_Pos)[0] > 0) && (BGVAR(s64_PosTuneTraj_Pos)[1] > 0)) ||
             ((BGVAR(s64_PosTuneTraj_Pos)[0] < 0) && (BGVAR(s64_PosTuneTraj_Pos)[1] < 0))))
         {
            PrintString("\nDist sgn mismatch",0);
            s64_temp = 1;
          }

         //Check if Uni directional but the 2 positions have different sign
         if((BGVAR(s16_Pos_Tune_Cycle) >= 2) &&
            (((BGVAR(s64_PosTuneTraj_Pos)[0] > 0) && (BGVAR(s64_PosTuneTraj_Pos)[1] < 0)) ||
             ((BGVAR(s64_PosTuneTraj_Pos)[0] < 0) && (BGVAR(s64_PosTuneTraj_Pos)[1] > 0))))
         {
            PrintString("\nDist sgn mismatch",0);
            s64_temp = 1;
         }

         if (s64_temp == 0) PrintString("passed",0);
         break;

      case 20: // load AT defaults
         if (s64_auto_tune_active_flag != 0)
         {
            PrintString("AT active",0);
            break;
         }
         ResetPosGainsForPosTune(drive);
         PrintString("Done",0);
         break;

      case 21:
         if (s64_auto_tune_active_flag != 0)
         {
            PrintString("AT active",0);
            break;
         }
         PosTuneBackup(drive);
         break;

      case 22:
         if (s64_auto_tune_active_flag != 0)
         {
            PrintString("AT active",0);
            break;
         }
         PosTuneRestore(drive,0);
         PositionConfig(drive, 1);
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         break;


      case 30: // AT lmjr variable u32_AT_LMJR
         PrintUnsignedLong(BGVAR(u32_AT_LMJR));
         break;

      case 31: // AT acc max/min based on u32_AT_LMJR
         // calc nominal and max acc
         f_j_tot = 1e-6*(float)BGVAR(s32_Motor_J)*(1 + 0.001*(float)BGVAR(u32_AT_LMJR));
         f_nom_acc = 1e-6*(float)BGVAR(u32_Motor_Kt)*(float)BGVAR(s32_Motor_I_Cont)/f_j_tot;// rad/sec^2
         f_max_acc = 1e-6*(float)BGVAR(u32_Motor_Kt)*(float)BGVAR(s32_Motor_I_Peak)/f_j_tot;// rad/sec^2
         u64_at_min_acc_int = (unsigned long long)((0.33 * f_nom_acc)*183493156455 );//183493156455=2^32*2^32/4000/4000/2/pi
         u64_at_max_acc_int = (unsigned long long)((0.9 * f_max_acc)*183493156455 );//183493156455=2^32*2^32/4000/4000/2/pi

         s64_temp =       MultS64ByFixS64ToS64(u64_at_min_acc_int,
                                   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);

         PrintString("Min=",0);
         PrintSignedLongLong(s64_temp);
         PrintChar(SPACE);
         PrintChar('[');
         PrintString((char *)s8_Units_Acc_Dec_For_Pos,0);
         PrintChar(']');

         // Stop If ACC > 62500 rpm/s
         if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) u64_max_acc_dec = MAX_ACC_DEC_TSP250;
         else u64_max_acc_dec = MAX_ACC_DEC_TSP125;

         if(u64_at_max_acc_int > (u64_max_acc_dec>>4LL)) u64_at_max_acc_int = u64_max_acc_dec>>4LL;

         s64_temp =       MultS64ByFixS64ToS64(u64_at_max_acc_int,
                                   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);

         PrintString("Max=",0);
         PrintSignedLongLong(s64_temp);
         PrintChar(SPACE);
         PrintChar('[');
         PrintString((char *)s8_Units_Acc_Dec_For_Pos,0);
         PrintChar(']');
         break;

      case 40: //set\get BGVAR(s64_Pos_Tune_AutoTraj_Pos_Lim)
         if (s16_Number_Of_Parameters == 1)
         {
            s64_temp = BGVAR(s64_Pos_Tune_AutoTraj_Pos_Lim);
            if (s64_temp == 0) s64_temp = 12884901885LL; // 3 motor rev
            s64_temp =      MultS64ByFixS64ToS64(s64_temp,
                                      BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                                      BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);
            // Print internal trajectory DISTANCE
            PrintSignedLongLong(s64_temp);
            PrintChar(SPACE);
            PrintChar('[');
            PrintString((char *)s8_Units_Pos,0);
            PrintChar(']');
         }
         else
         {

            s64_temp = MultS64ByFixS64ToS64(s64_Execution_Parameter[1],
                       BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_internal_fix,
                       BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_internal_shr);
            // check if distance is greater than 1 rev
            if (s64_temp >= 4294967295LL)
                BGVAR(s64_Pos_Tune_AutoTraj_Pos_Lim) = s64_temp;
            else
                return (VALUE_TOO_LOW);
         }
         break;

      case 50:
         if ((BGVAR(s16_Bundle_Design_Lock) <  0) || (BGVAR(s16_Bundle_Design_Lock) > CDHD_PRO2_MOTOR_DATA_ENTRIES))
            PrintChar('0');
         else
            PrintChar('1');
         break;

      case 61: // GUI AT test motion - speed
         s64_temp =    MultS64ByFixS64ToS64((long long)BGVAR(u32_Hdtune_Act_Traj_Spd),
                                BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix,
                                BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr);

         PrintSignedLongLong(s64_temp);
         PrintChar(SPACE);
         PrintChar('[');
         PrintString((char *)s8_Units_Vel_Out_Loop,0);
         PrintChar(']');
         break;

      case 62: // GUI AT test motion - distance
         s64_temp =      MultS64ByFixS64ToS64(BGVAR(s64_Hdtune_Act_Traj_Dist),
                             BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).s64_unit_conversion_to_user_fix,
                             BGVAR(Unit_Conversion_Table[POSITION_CONVERSION]).u16_unit_conversion_to_user_shr);
         PrintSignedLongLong(s64_temp);
         PrintChar(SPACE);
         PrintChar('[');
         PrintString((char *)s8_Units_Pos,0);
         PrintChar(']');
         break;

      case 63:
         s64_temp =       MultS64ByFixS64ToS64((long long)BGVAR(u64_Hdtune_Act_Traj_Acc),
                                BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_POS_CONVERSION]).u16_unit_conversion_to_user_shr);

         PrintSignedLongLong(s64_temp);
         PrintChar(SPACE);
         PrintChar('[');
         PrintString((char *)s8_Units_Acc_Dec_For_Pos,0);
         PrintChar(']');
         break;


      default:
         return (VALUE_OUT_OF_RANGE);
   }

   PrintCrLf();
   return (SAL_SUCCESS);
}

int SalPosTuneCycleCommand(int drive)
{
   long long param = s64_Execution_Parameter[0];

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((param < 0) || (param > 3)) return (VALUE_OUT_OF_RANGE);
   if ((BGVAR(s16_Pos_Tune_State) > 0) && (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)) return (AT_ACTIVE);

   if ( (s16_Number_Of_Parameters == 1)   &&
        ((param == 1LL) || (param == 3LL))  ) return (NOT_AVAILABLE);

   BGVAR(s16_Pos_Tune_Cycle) = (int)param;
   return (SAL_SUCCESS);
}

void PosTuneRestore(int drive,int mode)
{
   // AXIS_OFF;
   PosTuneWriteParam(POS_TUNE_KNLUSER_ID,s_PosTuneBackup.knlusergain,drive);
   PosTuneWriteParam(POS_TUNE_KNLI_ID,s_PosTuneBackup.knli,drive);
   PosTuneWriteParam(POS_TUNE_KNLIV_ID,s_PosTuneBackup.knliv,drive);
   PosTuneWriteParam(POS_TUNE_KNLD_ID,s_PosTuneBackup.knld,drive);
   PosTuneWriteParam(POS_TUNE_KNLP_ID,s_PosTuneBackup.knlp,drive);
   PosTuneWriteParam(POS_TUNE_NLFILT_DAMPING_ID,s_PosTuneBackup.knldamping,drive);
   PosTuneWriteParam(POS_TUNE_NLFILTT1_ID,s_PosTuneBackup.knlt1,drive);
   PosTuneWriteParam(POS_TUNE_NLPEAFF_ID,s_PosTuneBackup.knlpeaff,drive);
   PosTuneWriteParam(POS_TUNE_NLAFFLPFHZ_ID,s_PosTuneBackup.knlafflpf,drive);
   PosTuneWriteParam(POS_TUNE_NLAFRC_ID,s_PosTuneBackup.knlafrc,drive);
   PosTuneWriteParam(POS_TUNE_AV_HZ,s_PosTuneBackup.nlantivibhz,drive);
   PosTuneWriteParam(POS_TUNE_AV_HZ2,s_PosTuneBackup.nlantivibhz2,drive);
   PosTuneWriteParam(POS_TUNE_AV_HZ3,s_PosTuneBackup.nlantivibhz3,drive);
   PosTuneWriteParam(POS_TUNE_AV_SHRP,s_PosTuneBackup.nlantivibsharp,drive);
   PosTuneWriteParam(POS_TUNE_AV_SHRP2,s_PosTuneBackup.nlantivibsharp2,drive);
   PosTuneWriteParam(POS_TUNE_AV_SHRP3,s_PosTuneBackup.nlantivibsharp3,drive);
   PosTuneWriteParam(POS_TUNE_AV_GAIN,s_PosTuneBackup.nlantivibgain,drive);
   PosTuneWriteParam(POS_TUNE_AV_GAIN2,s_PosTuneBackup.nlantivibgain2,drive);
   PosTuneWriteParam(POS_TUNE_AV_GAIN3,s_PosTuneBackup.nlantivibgain3,drive);
   PosTuneWriteParam(POS_TUNE_AV_Q3,s_PosTuneBackup.nlantivibq3,drive);
   SalMoveSmoothLpfCommand((long long)s_PosTuneBackup.movesmoothlpfhz,drive);
   SalMoveSmoothAvgNumCommand((long long)s_PosTuneBackup.movesmoothavg,drive);
   MoveSmoothModeCommand((long long)s_PosTuneBackup.movesmoothmode,drive);
   SalIGravCommand((long long)s_PosTuneBackup.s16_igrav,drive);
   BGVAR(u32_Nl_Speed_Gain_User) = s_PosTuneBackup.knluservcmdgain;
   BGVAR(u16_Mencres_Shr) = s_PosTuneBackup.mencresshr;
   BGVAR(u32_LMJR_User) = s_PosTuneBackup.u32_lmjr_user;
   SalGearFiltT1Command((long long)s_PosTuneBackup.gearfiltt1,drive);
   SalGearFiltT2Command((long long)s_PosTuneBackup.gearfiltt2,drive);
   SalGearFiltVffCommand((long long)s_PosTuneBackup.gearfiltvelff,drive);
   SalGearFiltAffCommand((long long)s_PosTuneBackup.gearfiltaff,drive);
   // Simulate ASCII command "NLMAXGAIN xxx"
   s64_Execution_Parameter[0] = (long long)s_PosTuneBackup.nlmaxgain;
   s16_Number_Of_Parameters = 1;
   SalNlKpgmaxCommand(drive);
   SalNlNotchBWCommand((long long)s_PosTuneBackup.nlnotchbw,drive);
   SalNlNotchCenterCommand((long long)s_PosTuneBackup.nlnotchcenter,drive);
   MoveSmoothSourceCommand((long long)s_PosTuneBackup.nlmovesmoothsrc,drive);
   SalGearFiltModeCommand((long long)s_PosTuneBackup.gearfiltmode,drive);
   if (mode != 0)
   {
      // resore after AT fault signal AT to update gains after it has stopped
      PositionConfig(drive,1);
      VAR(AX0_s16_Cycle_Bits) |= PARAM_UPDATE_AT_STANDSTILL;
   }
   PosTuneTrajZeroingParam(drive);
}

void PosTuneBackup(int drive)
{
   // AXIS_OFF;
   s_PosTuneBackup.knlusergain = PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0);
   s_PosTuneBackup.knli = PosTuneReadParam(POS_TUNE_KNLI_ID,drive,0);
   s_PosTuneBackup.knliv = PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0);
   s_PosTuneBackup.knld = PosTuneReadParam(POS_TUNE_KNLD_ID,drive,0);
   s_PosTuneBackup.knlp = PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0);
   s_PosTuneBackup.knldamping = PosTuneReadParam(POS_TUNE_NLFILT_DAMPING_ID,drive,0);
   s_PosTuneBackup.knlt1 = PosTuneReadParam(POS_TUNE_NLFILTT1_ID,drive,0);
   s_PosTuneBackup.knlpeaff = PosTuneReadParam(POS_TUNE_NLPEAFF_ID,drive,0);
   s_PosTuneBackup.knlafflpf = PosTuneReadParam(POS_TUNE_NLAFFLPFHZ_ID,drive,0);
   s_PosTuneBackup.knlafrc = PosTuneReadParam(POS_TUNE_NLAFRC_ID,drive,0);
   s_PosTuneBackup.nlantivibhz = PosTuneReadParam(POS_TUNE_AV_HZ,drive,0);
   s_PosTuneBackup.nlantivibhz2 = PosTuneReadParam(POS_TUNE_AV_HZ2,drive,0);
   s_PosTuneBackup.nlantivibhz3 = PosTuneReadParam(POS_TUNE_AV_HZ3,drive,0);
   s_PosTuneBackup.nlantivibsharp = PosTuneReadParam(POS_TUNE_AV_SHRP,drive,0);
   s_PosTuneBackup.nlantivibsharp2 = PosTuneReadParam(POS_TUNE_AV_SHRP2,drive,0);
   s_PosTuneBackup.nlantivibsharp3 = PosTuneReadParam(POS_TUNE_AV_SHRP3,drive,0);
   s_PosTuneBackup.nlantivibgain = PosTuneReadParam(POS_TUNE_AV_GAIN,drive,0);
   s_PosTuneBackup.nlantivibgain2 = PosTuneReadParam(POS_TUNE_AV_GAIN2,drive,0);
   s_PosTuneBackup.nlantivibgain3 = PosTuneReadParam(POS_TUNE_AV_GAIN3,drive,0);
   s_PosTuneBackup.nlantivibq3 = PosTuneReadParam(POS_TUNE_AV_Q3,drive,0);
   s_PosTuneBackup.s16_igrav = VAR(AX0_s16_Igrav);
   s_PosTuneBackup.movesmoothavg = BGVAR(u32_Move_Smooth_Avg);
   s_PosTuneBackup.movesmoothlpfhz = BGVAR(s16_Move_Smooth_Lpf_Hz);
   s_PosTuneBackup.movesmoothmode = VAR(AX0_u16_Move_Smooth_Mode);
   s_PosTuneBackup.knluservcmdgain = BGVAR(u32_Nl_Speed_Gain_User);
   s_PosTuneBackup.mencresshr = BGVAR(u16_Mencres_Shr);
   s_PosTuneBackup.u32_lmjr_user = BGVAR(u32_LMJR_User);
   s_PosTuneBackup.gearfiltt1 = BGVAR(u32_Gear_Filt_User_T1);
   s_PosTuneBackup.gearfiltt2 = BGVAR(u16_Gear_Filt_User_T2);
   s_PosTuneBackup.gearfiltvelff = BGVAR(s32_Gear_Filt_User_Vff);
   s_PosTuneBackup.gearfiltaff = BGVAR(s16_Gear_Filt_User_Aff);
   s_PosTuneBackup.nlmaxgain = BGVAR(u32_Nl_Kpgmax);
   s_PosTuneBackup.nlnotchbw = BGVAR(s16_Nl_Notch_BW);
   s_PosTuneBackup.nlnotchcenter = BGVAR(s16_Nl_Notch_Center);
   s_PosTuneBackup.nlmovesmoothsrc = VAR(AX0_u16_Move_Smooth_Source);
   s_PosTuneBackup.gearfiltmode = VAR(AX0_u16_Qep_Msq_Fltr_Mode);
}

long EffortToPercent(int drive,unsigned long effort)
{
   float f_inertia_factor,f_temp,f_pwm_factor;
   long s32_mj = BGVAR(s32_Motor_J);
   unsigned long u32_mkt = BGVAR(u32_Motor_Kt);
   // AXIS_OFF;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u32_Pwm_Freq) == 16000)
   {
      f_pwm_factor = 0.1;
      if ((s32_mj > 300) && (s32_mj <= 500)) s32_mj = 300; // inital setup was done based on demo station motor with mj of 0.029. limit effort increase due to mj
      if ((s32_mj > 500)  && (s32_mj <= 1000)) s32_mj = 300 + (s32_mj-500)>>3;
      if ((s32_mj > 1000) && (s32_mj <= 2000)) s32_mj = 363 + (s32_mj-1000)>>3;
      if (s32_mj > 2000) s32_mj = 488;
   }
   else
   {
      f_pwm_factor = 0.5;
      if ((s32_mj > 300) && (s32_mj <= 500)) s32_mj = 300; // inital setup was done based on demo station motor with mj of 0.029. limit effort increase due to mj
      if ((s32_mj > 500)  && (s32_mj <= 1000)) s32_mj = 300 + (s32_mj-500);
      if ((s32_mj > 1000) && (s32_mj <= 2000)) s32_mj = 800 + (s32_mj-1000)>>1;
      if (s32_mj > 2000) s32_mj = 1300;
      if ((u32_mkt < 100) && (s32_mj > 100)) s32_mj = 100;
   }

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
      f_inertia_factor = (float)s32_mj/(float)u32_mkt;
   else//Linear
      f_inertia_factor = 3e5*(((float)BGVAR(s32_Motor_Mass)/((float)BGVAR(u32_Motor_Kf)*1000.0)) * (float)(((float)BGVAR(u32_Mpitch) / 1000000.0) * 0.15915)); //mpitch changed to decimal

   f_temp = (float)BGVAR(s32_Drive_I_Peak)/26214.0*f_pwm_factor;
   f_temp = f_temp / f_inertia_factor;

   if (VAR(AX0_S16_Kc_Mode) > 3) f_temp = f_temp/POS_TUNE_EFFORT_DEFAULT;
   else f_temp = f_temp/POS_TUNE_EFFORT_DEFAULT_KCMODE_3;

   if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC) effort = effort*2;

   return (long long)((float)effort*100000.0*f_temp);
}

unsigned long PercentToEffort(int drive,long percent_effort)
{
   float f_temp,f_inertia_factor,f_pwm_factor;
   long s32_mj = BGVAR(s32_Motor_J);
   unsigned long u32_mkt = BGVAR(u32_Motor_Kt);
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (BGVAR(u32_Pwm_Freq) == 16000)
   {
      f_pwm_factor = 0.1;
      if ((s32_mj > 300) && (s32_mj <= 500)) s32_mj = 300; // inital setup was done based on demo station motor with mj of 0.029. limit effort increase due to mj
      if ((s32_mj > 500)  && (s32_mj <= 1000)) s32_mj = 300 + (s32_mj-500)>>3;
      if ((s32_mj > 1000) && (s32_mj <= 2000)) s32_mj = 363 + (s32_mj-1000)>>3;
      if (s32_mj > 2000) s32_mj = 488;
   }
   else
   {
      f_pwm_factor = 0.5;
      if ((s32_mj > 300) && (s32_mj <= 500)) s32_mj = 300; // inital setup was done based on demo station motor with mj of 0.029. limit effort increase due to mj
      if ((s32_mj > 500)  && (s32_mj <= 1000)) s32_mj = 300 + (s32_mj-500);
      if ((s32_mj > 1000) && (s32_mj <= 2000)) s32_mj = 800 + (s32_mj-1000)>>1;
      if (s32_mj > 2000) s32_mj = 1300;
      if ((u32_mkt < 100) && (s32_mj > 100)) s32_mj = 100;
   }

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
      f_inertia_factor = (float)s32_mj/(float)u32_mkt;
   else//Linear
      f_inertia_factor = 3e5*(((float)BGVAR(s32_Motor_Mass)/((float)BGVAR(u32_Motor_Kf)*1000.0)) * (float)(((float)BGVAR(u32_Mpitch) / 1000000.0) * 0.15915)); //mpitch changed to decimal

   f_temp = (float)BGVAR(s32_Drive_I_Peak)/26214.0*f_pwm_factor;
   f_temp = f_temp / f_inertia_factor;

   if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC) f_temp = f_temp*2;

   if (VAR(AX0_S16_Kc_Mode) > 3) return (unsigned long)(POS_TUNE_EFFORT_DEFAULT*(float)percent_effort/(100000.0*f_temp));
   else return (unsigned long)(POS_TUNE_EFFORT_DEFAULT_KCMODE_3*(float)percent_effort/(100000.0*f_temp));
}

void PosTuneCEffortDesign(int drive)
{
   // setup Zsacle for the control effort function
   // z(k)  =  z(k-1)*w + abs(Icmd(k)-Icmd(k-1))*Zscale
   // Zscale * MKT*PWMFRQ/16/MJ this will help normilize the control effort signal
   // large MKT - more toque effort. PWM8 - expected more ripple. Large MJ - motor can tolarate more torque ripple (usually)
   //
   // in implementation , instead of calulating Zscale - the threshold is normalized to avoid numerical issues
   // with the effort function
   // AXIS_OFF;
   LVAR(AX0_u32_At_Control_Effort_Tresh) = PercentToEffort(drive,BGVAR(s32_Pos_Tune_C_Effort));
}

int SalReadPosEffortMax(long long *data,int drive)
{
   // AXIS_OFF;
   *data = (long long)EffortToPercent(drive,LVAR(AX0_u32_At_Control_Effort_Max));
   return (SAL_SUCCESS);
}


void PosTunePeMaskDesign(int drive)
{
   // implementation - 8 bits of the 32 btis are already masked in the RT
   // we are left with 24 bits
   int s16_unused_bits = 0,s16_total_bits;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if (FEEDBACK_SERVOSENSE) s16_unused_bits = 11;
   else if (BGVAR(u16_FdbkType) == SW_SINE_FDBK) s16_unused_bits = 4;
   else if (BGVAR(u16_FdbkType) == RESOLVER_FDBK) s16_unused_bits = 2;

   if (BGVAR(u16_Mencres_Shr)  > s16_unused_bits) s16_unused_bits = BGVAR(u16_Mencres_Shr);

   s16_total_bits = GetBitNumber((unsigned long)LVAR(AX0_s32_Counts_Per_Rev)) - s16_unused_bits;
   if (s16_total_bits > 24) s16_total_bits = 24;

   // calc mask
   s16_total_bits = s16_total_bits - 8; // mask is only on lower part of RT (higher is 8 bits revs and 8 bits inside rev
   if (s16_total_bits <= 0) VAR(AX0_u16_At_Cost_Pe_Noise_Mask) = 0;
   else
   {
      s16_total_bits = 16-s16_total_bits; // mask is the removed bits and not the bits left
      VAR(AX0_u16_At_Cost_Pe_Noise_Mask) = ~((1 << s16_total_bits) - 1);
   }
   // Use max possible bits in ext trajectory(gives more data for cost calc)
}


int PosTuneActiveCommand(long long *data,int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   if ((VAR(AX0_s16_Cycle_Bits) & POS_TUNE_RT_ACTIVE) != 0)
      *data = 1;
   else
      *data = 0;
   return (SAL_SUCCESS);
}


int UpdateGainAccordingToSearch(int drive,int s16_gain_recover_flag)
{
   int s16_temp,s16_index = 1,ret_val = 0;
   long long s64_min,s64_max,s64_max_ratio,s64_result_sum,s64_cost;
   long s32_param_value,s32_temp;
   unsigned long u32_effort_min;
   float f_temp;
   // AXIS_OFF;

   s64_TuneSearchTableResult[0] = 0;
   s64_min = s64_TuneSearchTableResult[1];
   s64_max = s64_TuneSearchTableResult[1];
   s64_max_ratio = s64_TuneSearchTableResult[1];
   s64_result_sum = s64_TuneSearchTableResult[1];
   u32_effort_min = u32_PosTuneSearchEffort[1];

   if ((s16_gain_recover_flag == 100) && (s64_min < 0x7FFFFFFFFFFFFFF0))// use min tqf and not in cost
   {
      s64_min = s64_PosTuneSearchTQF[1];
      s64_max = s64_PosTuneSearchTQF[1];
   }

   // find row with best cost - verify no cost overflow
   for(s16_temp = 2;s16_temp <= BGVAR(s16_Tune_Table_Size);++s16_temp)
   {
      s64_cost = s64_TuneSearchTableResult[s16_temp];
      if ((s16_gain_recover_flag == 100) && (s64_cost < 0x7FFFFFFFFFFFFFF0)) s64_cost =  s64_PosTuneSearchTQF[s16_temp];

      if (s64_min > s64_cost)
      {
         s64_min = s64_cost;
         s16_index = s16_temp;
      }
      if (s64_max < s64_cost) s64_max = s64_cost;

      if ( (s64_max_ratio < s64_TuneSearchTableResult[s16_temp])     &&
           (s64_TuneSearchTableResult[s16_temp] < 0x7FFFFFFFFFFFFFF0)  )
         s64_max_ratio = s64_TuneSearchTableResult[s16_temp];

      s64_result_sum += s64_TuneSearchTableResult[s16_temp];

      if (u32_effort_min > u32_PosTuneSearchEffort[s16_temp]) u32_effort_min = u32_PosTuneSearchEffort[s16_temp];
   }

   BGVAR(s64_Tune_Max_Cost) = s64_max;

   if ((s16_gain_recover_flag == 2) || (s16_gain_recover_flag == 3)) // verfiy search mode - no effrot exceeded should have occured
   {
      if (s16_Pos_Tune_Verify_Limit == 0) return 1; // limit the number of verify cycles
      if ( ((s16_gain_recover_flag == 2) && ((s64_min == 0x7FFFFFFFFFFFFFFF) || (s64_min == 0x7FFFFFFFFFFFFFF1)/* || (s64_min == 0x7FFFFFFFFFFFFFF3)*/ || (s64_max == 0x7FFFFFFFFFFFFFFF) || (s64_max == 0x7FFFFFFFFFFFFFF1)/* || (s64_max == 0x7FFFFFFFFFFFFFF3)*/)) ||
           ((s16_gain_recover_flag == 3) && (u32_PosTuneSearchEffort[0] != 0))                                      )
      { //verify failed - reduce gains and restart
         s16_temp = POS_TUNE_KNLUSER_ID;

         //else
         //{
         // } pertunabtions
         if (BGVAR(u16_HdTune_Adv_Mode) != 0) s16_temp = POS_TUNE_KNLUSER_ID;

         if (BGVAR(u16_At_Pretubation_En) == 0)
            PosTuneWriteParam(s16_temp,(long)(0.95*(float)(PosTuneReadParam(s16_temp,drive,0))),drive);
         else
            PosTuneWriteParam(s16_temp,(long)(0.8*(float)(PosTuneReadParam(s16_temp,drive,0))),drive);

         PositionConfig(drive,1);
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 2;
         s16_Pos_Tune_Verify_Limit--;
         return -1;
      }
      --BGVAR(s16_Min_Cost_Found_Cntr_Tresh);
      if (BGVAR(s16_Min_Cost_Found_Cntr_Tresh) != 0) return 0;

      return 1; // verify succeeded
   }

   if ((BGVAR(s16_Pos_Tune_State) >= POS_TUNE_CONT_INITIAL_COST_CALC) &&
       (BGVAR(s16_Pos_Tune_State) <= POS_TUNE_CONT_STOPPED)              )
   { // calc avg cost and verify no effort fault
      if ((s64_min >= 0x7FFFFFFFFFFFFFF0) || (s64_max >= 0x7FFFFFFFFFFFFFFF))
      {
         BGVAR(s64_Cont_Tune_Cost_Min) = 0x7FFFFFFFFFFFFFFF;
         BGVAR(s64_Cont_Tune_Cost_Max) = 0x7FFFFFFFFFFFFFFF;
         BGVAR(u32_Cont_Tune_Effort) = 1000000;
      }
      else
      {
         BGVAR(u32_Cont_Tune_Effort) = u32_effort_min;
         BGVAR(s64_Cont_Tune_Cost_Min) = s64_min;
         BGVAR(s64_Cont_Tune_Cost_Max) = s64_max;
      }
      return 1;
   }

   if (s64_min == 0) BGVAR(s32_Pos_Tune_Bits) |= COST_0_MASK;

   if ((s64_min == 0x7FFFFFFFFFFFFFFF)       ||
       (s64_max_ratio == 0x7FFFFFFFFFFFFFFF) ||
       (s64_min == s64_max_ratio)            ||
       (s64_min == 0LL)                        )
      BGVAR(s32_Pos_Tune_Search_Max_Min_Ratio) =-1;
   else
      BGVAR(s32_Pos_Tune_Search_Max_Min_Ratio) = (long)(100.0*(float)(s64_max_ratio>>4)/(float)(s64_min>>4));

   // if result found (with valid effort - load this result
   if (s64_min < 0x7FFFFFFFFFFFFFF0)
   {
      s16_temp = 0;
      while (s32_TuneSearchTable[0][s16_temp] != POS_TUNE_NO_ID)
      {
         PosTuneWriteParam(s32_TuneSearchTable[0][s16_temp],s32_TuneSearchTable[s16_index][s16_temp],drive);
         if (s32_TuneSearchTable[0][s16_temp] == POS_TUNE_KNLIV_ID)
         {
            if (BGVAR(u16_HdTune_Adv_Mode) != 0)
            { // knliv knlp knli dependency
               s32_temp = (long)(BGVAR(f_Kp_Kiv_Ratio)*(float)s32_TuneSearchTable[s16_index][s16_temp]);
               PosTuneWriteParam(POS_TUNE_KNLP_ID,s32_temp,drive);
               s32_temp = (long)(BGVAR(f_Ki_Kp_Ratio)*(float)s32_temp);
               PosTuneWriteParam(POS_TUNE_KNLI_ID,s32_temp,drive);
            }
            else
            {
               s32_temp = (long)(0.4*(float)(PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0)));
               if (s32_temp > PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0)) PosTuneWriteParam(POS_TUNE_KNLP_ID,s32_temp,drive);
            }
         }
         ++s16_temp;
      }

      PositionConfig(drive,1);
      VAR(AX0_s16_Cycle_Bits) |= CYCLE_PARAM_UPDATE;
      s64_TuneSearchTableResult[0] = s64_min;
      //u16_PosTuneSettleTimeResult[0] = u16_PosTuneSettleTimeResult[s16_index];
      //s64_PosTuneOverShootResult[0] = s64_PosTuneOverShootResult[s16_index];

      // check if selected result for this search is better than previous searches
      // if in usergain validity check - if cost is ~the same prefer the lower gain value
      f_temp = 1;
      if ((BGVAR(s16_Pos_Tune_State) == POS_TUNE_USERGAIN_SEARCH) && (BGVAR(u16_HdTune_Adv_Mode) == 0))
      {
         f_temp = (float)s64_TuneSearchTableResult[0]/(float)BGVAR(s64_Tune_Min_Cost);
         if  (f_temp < 0.75)
         {
            BGVAR(s64_Tune_Min_Cost) = s64_TuneSearchTableResult[0];
            BGVAR(s16_Min_Cost_Found_Cntr) = BGVAR(s16_Min_Cost_Found_Cntr_Tresh);
            StoreBestSearchResults(drive,s16_index);
         }
         f_temp = 0;
      }
      else if ((BGVAR(s16_Pos_Tune_State) == KNLUSERGAIN_VALIDITY_CHECK)   ||
               (BGVAR(s16_Pos_Tune_State) == KNLUSERGAIN_VALIDITY_CHECK_2)   )
      {
         f_temp = (float)s64_TuneSearchTableResult[0]/(float)BGVAR(s64_Tune_Min_Cost);
         if  (f_temp < 1)
         {
            BGVAR(s64_Tune_Min_Cost) = s64_TuneSearchTableResult[0];
            BGVAR(s16_Min_Cost_Found_Cntr) = BGVAR(s16_Min_Cost_Found_Cntr_Tresh);
            StoreBestSearchResults(drive,s16_index);
         }
         else if  ((f_temp < 1.1) && (s32_TuneSearchTableBest[0] != PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0)))
         {
            BGVAR(s16_Min_Cost_Found_Cntr) = BGVAR(s16_Min_Cost_Found_Cntr_Tresh);
            StoreBestSearchResults(drive,s16_index);
         }
         else
          if (BGVAR(s16_Min_Cost_Found_Cntr) != 0x7FFF) --BGVAR(s16_Min_Cost_Found_Cntr);
         f_temp = 0;
      }
      else if ((BGVAR(s16_Pos_Tune_State) == POS_TUNE_KNLI_VALIDATION) ||
               (BGVAR(s16_Pos_Tune_State) == POS_TUNE_ADV_KNLI_SEARCH)   )
      {
         BGVAR(s64_Tune_Min_Cost) = s64_TuneSearchTableResult[1];
         if (BGVAR(s64_Tune_Min_Cost) > s64_TuneSearchTableResult[2]) BGVAR(s64_Tune_Min_Cost) = s64_TuneSearchTableResult[2];
         s16_index = 1;
         // find row with best cost - verify no cost overflow
         s16_temp = 3;
         while (s16_temp <= BGVAR(s16_Tune_Table_Size)-1)
         {
            // every two entries are with the same knli
            // require that the higher value will be better than the initial min to be decalred as teh new min value
            f_temp = (float)s64_TuneSearchTableResult[s16_temp];
            if ((float)s64_TuneSearchTableResult[s16_temp+1] > (float)s64_TuneSearchTableResult[s16_temp]) f_temp = (float)s64_TuneSearchTableResult[s16_temp+1];

            f_temp = f_temp/(float)BGVAR(s64_Tune_Min_Cost);
            if ((s64_TuneSearchTableResult[s16_temp]   < 0x7FFFFFFFFFFFFFF0) &&
                (s64_TuneSearchTableResult[s16_temp+1] < 0x7FFFFFFFFFFFFFF0) &&
                (f_temp < 0.9)                                                 )
            {
               BGVAR(s64_Tune_Min_Cost) = s64_TuneSearchTableResult[s16_temp];
               if (BGVAR(s64_Tune_Min_Cost) > s64_TuneSearchTableResult[s16_temp+1]) BGVAR(s64_Tune_Min_Cost) = s64_TuneSearchTableResult[s16_temp+1];
               s16_index = s16_temp;
            }
            s16_temp += 2;
         }
         BGVAR(s16_Min_Cost_Found_Cntr) = BGVAR(s16_Min_Cost_Found_Cntr_Tresh);
         StoreBestSearchResults(drive,s16_index);
         f_temp = 0;
      }

      if ((s64_TuneSearchTableResult[0] < BGVAR(s64_Tune_Min_Cost)) && (f_temp == 1))
      {
         // filter results in which cost is "too good to be true"
         if ( (BGVAR(s32_Pos_Tune_Search_Max_Min_Ratio) > 300)        &&
              ((VAR(AX0_s16_Cycle_Bits) & MIN_MAX_RATIO_HIGH) == 0)   /*&&
              (BGVAR(u16_HdTune_Adv_Mode) == 0)                       */  )
            VAR(AX0_s16_Cycle_Bits) |= MIN_MAX_RATIO_HIGH;
         else
         {
            BGVAR(s16_Min_Cost_Found_Cntr) = BGVAR(s16_Min_Cost_Found_Cntr_Tresh);
            BGVAR(s64_Tune_Min_Cost) = s64_TuneSearchTableResult[0];
            VAR(AX0_s16_Cycle_Bits) &= ~MIN_MAX_RATIO_HIGH;
            StoreBestSearchResults(drive,s16_index);
         }
      }
      else
         VAR(AX0_s16_Cycle_Bits) &= ~MIN_MAX_RATIO_HIGH;

      if ( (BGVAR(s16_Min_Cost_Found_Cntr) != 0x7FFF) &&
           ((VAR(AX0_s16_Cycle_Bits) & MIN_MAX_RATIO_HIGH) == 0) )  --BGVAR(s16_Min_Cost_Found_Cntr);

      // when only one parameter is tuned do not try to push more - only makes noise
      if ((s64_max >= 0x7FFFFFFFFFFFFFF0)                             &&
          (s32_TuneSearchTable[0][1] == POS_TUNE_NO_ID)               &&
          (BGVAR(s16_Pos_Tune_State) != KNLUSERGAIN_VALIDITY_CHECK)   &&
          (BGVAR(s16_Pos_Tune_State) != KNLUSERGAIN_VALIDITY_CHECK_2)   ) BGVAR(s16_Min_Cost_Found_Cntr) = 0;

      // when search max out (val+ = val- = best so far) than quit search
      // protect against at stuck state
      if ((BGVAR(s16_Tune_Table_Size) == 2)                         && // one param , 2 entries
          (s32_TuneSearchTable[0][1] == POS_TUNE_NO_ID)             &&
          (s32_TuneSearchTable[1][0] == s32_TuneSearchTableBest[0]) && // both searches are the same and equal to best
          (s32_TuneSearchTable[2][0] == s32_TuneSearchTableBest[0])   ) BGVAR(s16_Min_Cost_Found_Cntr) = 0;

      if (BGVAR(s16_Min_Cost_Found_Cntr) == 0x7FFF)
      {
         f_temp = (float)s64_TuneSearchTableResult[0]/(float)BGVAR(s64_Tune_Min_Cost);

         if ((f_temp > 1.25)                                                                                             ||
             (PosTuneReadParam(s32_TuneSearchTable[0][0],drive,0) == PosTuneReadParam(s32_TuneSearchTable[0][0],drive,1)) )
            ret_val = 1;
         else
            BGVAR(s16_Tune_Table_Size) = GenerateTuneSearchTable(drive);
      }
      else if (BGVAR(s16_Min_Cost_Found_Cntr) > 0)
      {
         RestoreBestSearchResults(drive);
         PositionConfig(drive,1);
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         if ((VAR(AX0_s16_Cycle_Bits) & MIN_MAX_RATIO_HIGH) == 0) BGVAR(s16_Tune_Table_Size) = GenerateTuneSearchTable(drive);
      }
      else
        ret_val = 1;
   }
   else //effort exceeded in all search cycle
   {
      if ( (BGVAR(s64_Tune_Min_Cost) == 0x7FFFFFFFFFFFFFFF) && (s16_gain_recover_flag == 1) )// current search cost exceeds effort - check if the best cost is the same - than need to reduce global gain
      {
         RestoreBestSearchResults(drive);
         BGVAR(s16_Min_Cost_Found_Cntr) = BGVAR(s16_Min_Cost_Found_Cntr_Tresh);
         BGVAR(s16_Tune_Table_Size) = GenerateTuneSearchTable(drive);
         s32_param_value = (long)(0.9*(float)(PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0)));
         BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_USER_GAIN_MODIFED;
         if (s32_param_value < 100)
         {
            s32_param_value = 100;
            BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_LOW_COST_MASK;
         }
         PosTuneWriteParam(POS_TUNE_KNLUSER_ID,s32_param_value,drive);
         PositionConfig(drive,1);
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         if (s32_TuneSearchTable[0][0] == POS_TUNE_KNLUSER_ID) ret_val = -1;
         return (ret_val);
      }
      else
         ret_val = 1; // valid result not found in this search cycle but min cost data exsists - end search
   }
   return (ret_val);
}

void StoreBestSearchResults(int drive,int s16_best_row_id)
{
   int s16_column = 0,s16_param_id;
   long s32_param_value;

   while((s32_TuneSearchTable[0][s16_column] != POS_TUNE_NO_ID) && (s16_column < TUNE_SEARCH_TABLE_MAX_COL))
   {
      s16_param_id = s32_TuneSearchTable[0][s16_column];
      s32_param_value = PosTuneReadParam(s16_param_id,drive,0);
      s32_TuneSearchTableBest[s16_column] = s32_param_value;
      BGVAR(s16_TuneSearchTableBest_RowId) = s16_best_row_id;
      s16_column++;
   }
}

void StoreInitialGains(int drive)
{
   int s16_column = 0,s16_param_id;
   long s32_param_value;

   while((s32_TuneSearchTable[0][s16_column] != POS_TUNE_NO_ID) && (s16_column < TUNE_SEARCH_TABLE_MAX_COL))
   {
      s16_param_id = s32_TuneSearchTable[0][s16_column];
      s32_param_value = PosTuneReadParam(s16_param_id,drive,0);
      s32_TuneSearchInitialGains[s16_column] = s32_param_value;
      s16_column++;
   }
}

void ReStoreInitialGains(int drive)
{
   int s16_column = 0,s16_param_id;
   long s32_param_value,s32_temp;
   while(s32_TuneSearchTable[0][s16_column] != POS_TUNE_NO_ID)
   {
      s16_param_id = s32_TuneSearchTable[0][s16_column];
      s32_param_value = s32_TuneSearchInitialGains[s16_column];
      PosTuneWriteParam(s16_param_id,s32_param_value,drive);

      if (s16_param_id == POS_TUNE_KNLIV_ID)
      { // knliv knlp knli dependency
         if (BGVAR(u16_HdTune_Adv_Mode) != 0)
         {
         s32_temp = (long)(BGVAR(f_Kp_Kiv_Ratio)*(float)(PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0)));
         PosTuneWriteParam(POS_TUNE_KNLP_ID,s32_temp,drive);
         s32_temp = (long)(BGVAR(f_Ki_Kp_Ratio)*(float)s32_temp);
         PosTuneWriteParam(POS_TUNE_KNLI_ID,s32_temp,drive);
      }
         else
         {
            s32_temp = (long)(0.4*(float)(PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0)));
            if (s32_temp > PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0)) PosTuneWriteParam(POS_TUNE_KNLP_ID,s32_temp,drive);
         }
      }
      s16_column++;
   }
   BGVAR(s64_Tune_Min_Cost) = 0x7FFFFFFFFFFFFFFF;
}


// #pragma CODE_SECTION(StoreMinTQFResult, "ramfunc");
#pragma CODE_SECTION(StoreMinTQFResult, "ramfunc_3");
void StoreMinTQFResult(int drive, int mode)
{
   int s16_column = 0,s16_param_id;
   long s32_param_value;

   while((s32_TuneSearchTable[0][s16_column] != POS_TUNE_NO_ID) && (s16_column < TUNE_SEARCH_TABLE_MAX_COL))
   {
      s16_param_id = s32_TuneSearchTable[0][s16_column];
      s32_param_value = PosTuneReadParam(s16_param_id,drive,0);
      if (mode == 0)
      s32_TuneSearchTableMinTQF[s16_column] = s32_param_value;
      else
         s32_TuneSearchTableFirstTQF[s16_column] = s32_param_value;
      s16_column++;
   }
}

void RestoreBestSearchResults(int drive)
{
   int s16_column = 0,s16_param_id;
   long s32_param_value,s32_temp;
   while(s32_TuneSearchTable[0][s16_column] != POS_TUNE_NO_ID)
   {
      s16_param_id = s32_TuneSearchTable[0][s16_column];
      s32_param_value = s32_TuneSearchTableBest[s16_column];
      PosTuneWriteParam(s16_param_id,s32_param_value,drive);

      if (s16_param_id == POS_TUNE_KNLIV_ID)
      { // knliv knlp knli dependency
         if (BGVAR(u16_HdTune_Adv_Mode) != 0)
         {
         s32_temp = (long)(BGVAR(f_Kp_Kiv_Ratio)*(float)(PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0)));
         PosTuneWriteParam(POS_TUNE_KNLP_ID,s32_temp,drive);
         s32_temp = (long)(BGVAR(f_Ki_Kp_Ratio)*(float)s32_temp);
         PosTuneWriteParam(POS_TUNE_KNLI_ID,s32_temp,drive);
      }
         else
         {
            s32_temp = (long)(0.4*(float)(PosTuneReadParam(POS_TUNE_KNLIV_ID,drive,0)));
            if (s32_temp > PosTuneReadParam(POS_TUNE_KNLP_ID,drive,0)) PosTuneWriteParam(POS_TUNE_KNLP_ID,s32_temp,drive);
         }
      }
      s16_column++;
   }
}


void TuneSearchInit(int drive,int s16_restart_flag)
{
   if (s32_TuneSearchTable[0][0] == POS_TUNE_NO_ID) return;
   if (s16_restart_flag == 0) BGVAR(s64_Tune_Min_Cost) = 0x7FFFFFFFFFFFFFFF;
   BGVAR(s16_Tune_Table_Index) = 0;
   BGVAR(s16_Min_Cost_Found_Cntr) = BGVAR(s16_Min_Cost_Found_Cntr_Tresh);
   StoreBestSearchResults(drive,-1);
   BGVAR(s16_Tune_Table_Size) = GenerateTuneSearchTable(drive);
   s16_Extreemly_High_Effort = 0;
   BGVAR(s32_Pos_Tune_Bits) &=~EFFORT_EVENT_MASK;
   BGVAR(s32_Hdtune_Low_Frq_Osc) = 0;
}

int PosTuneSearch(int drive,int s16_gain_recover_flag,int s16_delta_flag)
{
   int s16_ret_val,s16_temp;
   // AXIS_OFF;

   if (s32_TuneSearchTable[0][0] == POS_TUNE_NO_ID) return 1;

   s16_ret_val = ExecutePosTuneSearchTable(drive,s16_gain_recover_flag);// 1 - done , 0 ongoing , -1 cost overflow
   if (s16_ret_val == 0) return 0;//execution ongoing. no issues
   else if (s16_ret_val == 1)  // table complete ->select best value and decide if search is done
   {
      s16_ret_val = UpdateGainAccordingToSearch(drive,s16_gain_recover_flag); // also updates min_cost

      // suport knliv step modifying - advmode 2 - with knliv all other change => no need kniv is best + step and best + 2*step . min value if 2*step is 20%
      if ( (s16_delta_flag == 1)                                          &&
           (BGVAR(s32_Pos_Tune_Search_Max_Min_Ratio) < 110)               &&
           (BGVAR(s32_Pos_Tune_Search_Max_Min_Ratio) > 0)                 &&
           (s16_PosTune_Knliv_Step < POSTUNE_KNLIV_STEP_MAX)              &&
           (BGVAR(u16_HdTune_Adv_Mode) == 0)                                )
      { // max_cost/min_cost is too small increase knliv delta and re-try
         BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_KNLIV_DELTA_WARN;
         s16_PosTune_Knliv_Step +=4;
         if (s16_PosTune_Knliv_Step > POSTUNE_KNLIV_STEP_MAX) s16_PosTune_Knliv_Step = POSTUNE_KNLIV_STEP_MAX;
         s16_Tune_Delta_Table[0] = s16_PosTune_Knliv_Step;
         RestoreBestSearchResults(drive);
         return -1;
      }

      if ( (s16_delta_flag == 2)                                          &&
           (BGVAR(s32_Pos_Tune_Search_Max_Min_Ratio) < 110)               &&
           (BGVAR(s32_Pos_Tune_Search_Max_Min_Ratio) > 0)                 &&
           ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_KNLP_DELTA_WARN) == 0)    )
      { // max_cost/min_cost is too small increase knliv delta and re-try
         BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_KNLP_DELTA_WARN;
         s16_Tune_Delta_Table[0] += 5;
         RestoreBestSearchResults(drive);
         return -1;
      }

      if ( (s16_delta_flag == 3)                                          &&
           (BGVAR(s32_Pos_Tune_Search_Max_Min_Ratio) < 110)               &&
           (BGVAR(s32_Pos_Tune_Search_Max_Min_Ratio) > 0)                 &&
           (s16_PosTune_Knliv_Step2 < POSTUNE_KNLIV_STEP_MAX)                )
      { // max_cost/min_cost is too small increase knliv and knlp delta
         BGVAR(s32_Pos_Tune_Bits) |= (POS_TUNE_KNLIV_DELTA_WARN|POS_TUNE_KNLP_DELTA_WARN);
         s16_PosTune_Knliv_Step2 +=2;
         if (s16_PosTune_Knliv_Step2 > POSTUNE_KNLIV_STEP_MAX) s16_PosTune_Knliv_Step2 = POSTUNE_KNLIV_STEP_MAX;
         s16_Tune_Delta_Table[0] = s16_PosTune_Knliv_Step2;
         s16_Tune_Delta_Table[1] = 12;
         RestoreBestSearchResults(drive);
         return -1;
      }

      if ( (BGVAR(s16_Pos_Tune_State) == POS_TUNE_USERGAIN_SEARCH) &&
           (PosTuneReadParam(POS_TUNE_KNLUSER_ID,drive,0) > 20000)   )
      {
         RestoreBestSearchResults(drive);
         KnlusergainToLMJRConversion(drive);
         return -1;
      }
   }
   else// cost overflow detected ret_val = -1 - generate fault
   {
      BGVAR(s32_Pos_Tune_Bits) |= POS_TUNE_COST_FATOR_FAULT;
      return 0;
   }

   if (s16_ret_val == 1) // search done - load best
   {
      if ( ((BGVAR(u16_RT_Tqf_Control) == 1)                                                                          ||  // tqf was requirested but never started and ssearch is done
            ((BGVAR(s64_Tune_Min_Tqf) == 0x7FFFFFFFFFFFFFFF) && ((BGVAR(s32_Pos_Tune_Bits) & TQF_FUNC_ENABLED) != 0))  ) &&
            (BGVAR(u16_Cycle_Counter_Mask) < 0x07)                                                                       /*&&
            (VAR(AX0_u16_Pos_Control_Mode) == NON_LINEAR_POSITION_2)                                                       */ )
      {
         BGVAR(s32_Pos_Tune_Bits) &= ~TQF_FUNC_ENABLED;
         ReStoreInitialGains(drive);
         PositionConfig(drive,1);
         VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
         
         CycleArrayOverFlow(drive); // restart cycle ident module
            BGVAR(u16_Cycle_Counter_Mask) = (BGVAR(u16_Cycle_Counter_Mask) << 1) + 1;
            BGVAR(u16_RT_Tqf_Control) = 1;
         return -1;
      }
      BGVAR(s32_Pos_Tune_Bits) &= ~TQF_FUNC_ENABLED;
      BGVAR(u16_RT_Tqf_Control) = 0;

      RestoreBestSearchResults(drive);
      s16_temp = 0;
      while ((s32_TuneSearchTable[0][s16_temp] != POS_TUNE_NO_ID) && (s16_gain_recover_flag != 10) ) // do not normalize in cont tune
      {
         if (s32_TuneSearchTable[0][s16_temp] == POS_TUNE_KNLUSER_ID) AutoTuneGainNormalization(drive);
         ++s16_temp;
      }
      PositionConfig(drive,1);
      VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
      VAR(AX0_s16_Cycle_Bits) &= ~MIN_MAX_RATIO_HIGH;
   }

   return (s16_ret_val);
}


//**********************************************************
// Function Name: PrintStringsFromTable
// Description: Print one or few strings from a given table.
//              if mode == 0 s32_flags is bits-array, and all relavent bits will be printed
//              if mode == PRINT_BY_ID s32_flag is integer, and zero or one string will be pronted.
// Author: Udi
//**********************************************************
int PrintStringsFromTable(long s32_flags, const Strings_Struct strings_struct[], PrintStringMode mode, unsigned int u16_level)
{
   static int s16_index = 0;

   if(mode == PRINT_BY_BIT_ARRAY) // Print all strings by Bit-Mask
   {
      while(strings_struct[s16_index].u16_level < 0xFFFF )
      {
         if((s32_flags & strings_struct[s16_index].u32_string_id) != 0)
         {

            if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;
            if(strings_struct[s16_index].u16_level <= u16_level )
            {
               PrintStringCrLf(strings_struct[s16_index].p_string ,0);
               //PrintStringCrLf(strings_struct[s16_index].p_string ,0);
            }
         }
         s16_index ++;
      }
      s16_index = 0;
      return (SAL_SUCCESS);
  }

   if(mode == PRINT_BY_ID) // Print one string by ID
   {
      if (u8_Output_Buffer_Free_Space < 40) return SAL_NOT_FINISHED;
      while(strings_struct[s16_index].u16_level < 0xFFFF )
      {
         if(s32_flags == strings_struct[s16_index].u32_string_id )
         {
            PrintStringCrLf(strings_struct[s16_index].p_string ,0);
            s16_index = 0;
            return (SAL_SUCCESS);
         }
         s16_index ++;
      }
      s16_index = 0;
      return (SAL_SUCCESS);
  }

 return (SAL_SUCCESS);
}


void PrintPosTuneGain(long s32_id)
{
   switch (s32_id)
   {
      case POS_TUNE_KNLUSER_ID:
         PrintString("KNLUSERGAIN",0);
         break;
      case POS_TUNE_KNLI_ID:
         PrintString("KNLI",0);
         break;
      case POS_TUNE_KNLD_ID:
         PrintString("KNLD",0);
         break;
      case POS_TUNE_KNLIV_ID:
         PrintString("KNLIV",0);
         break;
      case POS_TUNE_KNLP_ID:
         PrintString("KNLP",0);
         break;
      case POS_TUNE_NLPEAFF_ID:
         PrintString("NLPEAFF",0);
         break;
      case POS_TUNE_NLAFFLPFHZ_ID:
         PrintString("NLAFFLPFHZ",0);
         break;
      case POS_TUNE_NLFILTT1_ID:
      case POS_TUNE_NLFILTT1_AND_DAMPING_ID:
         PrintString("NLFILTT1",0);
         break;
      case POS_TUNE_NLFILT_DAMPING_ID:
         PrintString("NLFILTDAMPING",0);
         break;
      case POS_TUNE_NLAFRC_ID:
         PrintString("KNLAFRC",0);
         break;
      case POS_TUNE_AV_HZ2:
         PrintString("NLANTIVIBHZ2",0);
         break;
      case POS_TUNE_AV_HZ3:
         PrintString("NLANTIVIBHZ3",0);
         break;
      case POS_TUNE_AV_GAIN2:
         PrintString("NLANTIVIBGAIN2",0);
         break;
      case POS_TUNE_AV_GAIN3:
         PrintString("NLANTIVIBGAIN3",0);
         break;
      case POS_TUNE_AV_SHRP2:
         PrintString("NLANTIVIBSHARP2",0);
         break;
      case POS_TUNE_AV_SHRP3:
         PrintString("NLANTIVIBSHARP3",0);
         break;
      case POS_TUNE_AV_Q3:
         PrintString("NLANTIVIBQ3",0);
         break;
   }
}

void PosTuneAvSearch(int drive,int s16_av_search,int s16_mode)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   s16_mode++; // avoid compiler warning
   s16_mode--; // avoid compiler warning
   switch (s16_av_search)
   {
      case POS_TUNE_AV_GAIN2:
         s32_TuneSearchTable[0][0] = POS_TUNE_AV_GAIN2;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = 0;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 2;
         VAR(AX0_AutoTune_Bits) |= AT_AV_ACTIVE_2_MASK;
         VAR(AX0_AutoTune_Bits) &= ~AT_AV_ACTIVE_3_MASK;
         break;

      case POS_TUNE_AV_GAIN3:
         s32_TuneSearchTable[0][0] = POS_TUNE_AV_GAIN3;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = 5;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 2;
         VAR(AX0_AutoTune_Bits) &= ~AT_AV_ACTIVE_2_MASK;
         VAR(AX0_AutoTune_Bits) |= AT_AV_ACTIVE_3_MASK;
         break;

      case POS_TUNE_AV_SHRP2:
         s32_TuneSearchTable[0][0] = POS_TUNE_AV_SHRP2;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = 10;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 2;
         VAR(AX0_AutoTune_Bits) |= AT_AV_ACTIVE_2_MASK;
         VAR(AX0_AutoTune_Bits) &= ~AT_AV_ACTIVE_3_MASK;
         break;

      case POS_TUNE_AV_SHRP3:
         s32_TuneSearchTable[0][0] = POS_TUNE_AV_SHRP3;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = 10;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 2;
         VAR(AX0_AutoTune_Bits) &= ~AT_AV_ACTIVE_2_MASK;
         VAR(AX0_AutoTune_Bits) |= AT_AV_ACTIVE_3_MASK;
         break;

      case POS_TUNE_AV_Q3:
         s32_TuneSearchTable[0][0] = POS_TUNE_AV_Q3;
         s32_TuneSearchTable[0][1] = POS_TUNE_NO_ID;
         s16_Tune_Delta_Table[0] = 7;
         BGVAR(s16_Min_Cost_Found_Cntr_Tresh) = 2;
         VAR(AX0_AutoTune_Bits) &= ~AT_AV_ACTIVE_2_MASK;
         VAR(AX0_AutoTune_Bits) |= AT_AV_ACTIVE_3_MASK;
         break;

      default:
         break;
   }
}

int HdTuneProgressBarCommand(long long *data, int drive)
{
    long long s64_data;

    // Ask if the auto-tuner is active
    PosTuneActiveCommand(&s64_data,drive);

    // Check if the auto-tuner is active or done
    if ((s64_data == 1) || (BGVAR(s16_Pos_Tune_State) == POS_TUNE_DONE))
        *data = HdTuneProgressBar(drive,0);
    else
        *data = 0;

    return (SAL_SUCCESS);
}

int HdTuneProgressBar(int drive,int s16_reset)
{ // value is 0-100
   int s16_state_progress_bar;

   static int s16_progress_bar = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (s16_reset)
   {
      s16_progress_bar = 0;
      return 0;
   }

   if ((BGVAR(s16_Pos_Tune_State) < 0) || (BGVAR(s16_Pos_Tune_State) == POS_TUNE_DONE)) return 100; // if in fault state progress bar is 100%
   if ((BGVAR(s16_Pos_Tune_State) == POS_TUNE_SETUP)  || (BGVAR(s16_Pos_Tune_State) == POS_TUNE_IDLE)) return 0;

   switch (BGVAR(s16_Pos_Tune_State))
   {
      case POS_TUNE_WAIT_ACTIVE:
         s16_state_progress_bar = 1;
         break;
      case POS_TUNE_LMJR_EST:
         s16_state_progress_bar = 5;
         break;
      case POS_TUNE_TRAJ_SETUP:
         s16_state_progress_bar = 7;
         break;
      case POS_TUNE_IGRAV_SETUP:
      case POS_TUNE_INITIAL_COST_CAPTURE:
         s16_state_progress_bar = 10;
         break;
      case POS_TUNE_USERGAIN_SEARCH:
         s16_state_progress_bar = 11;
         break;
      case POS_TUNE_KNLUSER_VERIFY:
      case POS_TUNE_FILTER_VERIFY:
         s16_state_progress_bar = 13;
         break;
      case POS_TUNE_ADV_KNLIV_SEARCH:
         s16_state_progress_bar = 17;
         break;
      case POS_TUNE_ADV_ANTIVIB_INIT:
         s16_state_progress_bar = 27;
         break;
      case POS_TUNE_ADV_KNLIV_AFTER_ANTIVIB_INIT:
         s16_state_progress_bar = 32;
         break;
      case POS_TUNE_ADV_KNLP_SEARCH:
         s16_state_progress_bar = 35;
         break;
      case KNLUSERGAIN_VALIDITY_CHECK:
         s16_state_progress_bar = 37;
         break;
      case KNLIV_KNLP_SUM_SEARCH:
      case POS_TUNE_AV_FINE_TUNE:
         s16_state_progress_bar = 41;
         break;
      case POS_TUNE_ADV_KNLI_SEARCH:
         s16_state_progress_bar = 45;
         break;
      case KNLUSERGAIN_VALIDITY_CHECK_2:
         s16_state_progress_bar = 55;
         break;
      case POS_TUNE_ADV_SEARCH_4_1:
      case POS_TUNE_PERTUBATION:
         s16_state_progress_bar = 70;
         break;
      case POS_TUNE_NLPEAFF_SEARCH:
      case POS_TUNE_NLAFRC_SEARCH:
      case POS_TUNE_NLAFRC_SEARCH_1:
      case POS_TUNE_FINAL_COST_CAPTURE:
      case POS_TUNE_TRAJ_STOP:
      case POS_TUNE_MOVESMOOTH2_MODE_STATE:
      case POS_TUNE_KNLI_VALIDATION:
         s16_state_progress_bar = 82;
         break;
      case POS_TUNE_RESTART:
         s16_state_progress_bar = 85;
         break;
      case POS_TUNE_SAVE_MODE:
         s16_state_progress_bar = 100; //Auto tune completed, user may save or discard.
         break;
      default:
         s16_state_progress_bar =-1;
   }
   if (BGVAR(u16_HdTune_Adv_Mode) == 2)
   {   // spread progress bar evenly when hdtune is in easy mode
       s16_state_progress_bar = (s16_state_progress_bar << 2);
       if (s16_state_progress_bar > 80) s16_state_progress_bar = 80;
   }
   if (s16_progress_bar < s16_state_progress_bar) s16_progress_bar = s16_state_progress_bar;
   return (s16_progress_bar);
}

int CycleLmjrCommand(int drive)
{
   int s16_temp,s16_vel;
   long long s64_temp;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   PrintSignedLong(BGVAR(s32_Cycle_LMJR));
   PrintChar(SPACE);
   PrintSignedLong(BGVAR(s32_Cycle_Fric_Coeff));
   PrintChar(SPACE);
   PrintSignedLong(BGVAR(s32_Cycle_T_Unbalance));
   PrintChar(SPACE);

   do {
      s16_temp = Cntr_3125;
      s64_temp = LLVAR(AX0_u32_Abs_Icmd_Acc_Lo);
      s16_vel = VAR(AX0_s16_Vel_Var_Fb_0);
   } while (s16_temp != Cntr_3125);

   PrintSignedLongLong(s64_temp);
   PrintChar(SPACE);
   PrintSignedInteger(s16_vel);
   PrintCrLf();
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: PrintPosTuneMode
// Description:
// Author:
//**********************************************************
int PrintPosTuneMode(int drive)
{
  int ret_val;
  
  REFERENCE_TO_DRIVE;     // Defeat compilation error
  
  PrintString("HDTune Mode:\n ",0);

  ret_val = PrintStringsFromTable((long)(BGVAR( s16_Pos_tune_Mode)) ,AT_Mode_Strings, PRINT_BY_ID, 0);
  if (ret_val == SAL_SUCCESS)
  {
      if (BGVAR(u16_HdTune_Defaults) != 0) 
         PrintString("Start From Current Gains\n",0);
      else 
         PrintString("Start From Default Gains\n",0);
  }
  return ret_val;
}



int CycleIdentStatus(long long *data,int drive)
{
   // 0 - no cycle
   // 2 - identifying
   // 9 - identified
   long long s64_temp;
   // check if AT is active
   PosTuneActiveCommand(&s64_temp,drive);
    // Check if the auto-tuner is active or done
    if ((s64_temp == 0) || (BGVAR(s16_Cycle_StandStill_Found) == 0)) *data = 0LL;
    else if (BGVAR(s16_Cycle_StandStill_Found) == 4) *data = 9LL;
    else *data = 2LL;

    return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: PrintPosTuneState
// Description:
// Author:
//**********************************************************

void PrintPosTuneState(int drive)
{
   int s16_temp;
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   PrintString("\nState: ",0);
   s16_temp = BGVAR(s16_Pos_Tune_State);
     
   PrintString("(",0);
   if(s16_temp < 0)
   {
      s16_temp = -s16_temp;
      PrintString("F",0);
   }
   else 
      PrintString("S",0);

   PrintSignedInteger(s16_temp);   
   PrintString(") ",0);   
   switch (BGVAR(s16_Pos_Tune_State))
   {
  
     case AT_TRAJ_LO_SPEED_FAULT:
         PrintString("Trajectory speed too low",0);
         break;     
     case AT_TRAJ_BELOW_ACC_MIN_FAULT:
         PrintString("Trajectory acceleration too low",0);
         break;
     case AT_TRAJ_ABOVE_ACC_MAX_FAULT:	
         PrintString("Trajectory acceleration too high",0);
         break;
      case AT_HIGH_LMJR_FAULT:
         PrintString("LMJR too high",0);
         break;
      case AT_DEC_DECSTOP_FAULT:
         PrintString("DECSTOP too low",0);
         break;
      case POS_TUNE_SIMPLEX_LMJR_FAULT:
         PrintString("LMJR Estimation Failed",0);
         break;   
      case POS_TUNE_LOW_CURR_LEVEL_FAULT:
         PrintString("Low Acc or Velocity",0);
         break;
      case POS_TUNE_INVALID_AT_BUNDLE_FAULT:
         PrintString("Bundle AT Not supported",0);
         break;
      case POS_TUNE_MODE_2_TUNE_FAULT:
         PrintString("No Improvment",0);
         break;
      case POS_TUNE_CYCLE_CNT_TOUT_FAULT:
         PrintString("Cycle identify timeout",0);
         break;
      case POS_TUNE_OVER_COUNT_FAULT:
         PrintString("Cycle count value is high",0);
         break;
      case POS_TUNE_WARNINGS_EXSISTS_FAULT:
         PrintString("Unexpected drive warning",0);
         break;
      case POS_TUNE_FOLDBACK_FAULT:
         PrintString("Foldback Occured",0);
         break;
      case AUTO_TUNE_GAINS_INVALID_FAULT:
         PrintString("AT Gains Too Low",0);
         break;
      case AUTO_TRAJ_TOO_LONG:
         PrintString("Trajectory Too Long",0);
         break;
      case UNKNOWN_MOTOR_ID_FAULT:
         PrintString("Motor Was Not Recognized",0);
         break;
      case POS_TUNE_LOWEFFORT_FAULT:
         PrintString("Low Effort Fault",0);
         break;
      case POS_TUNE_COST_0_FAULT:
         PrintString("Motion Too Small Fault",0);
         break;
      case POS_TUNE_HOLD_FAULT:
         PrintString("Unexpected Hold Fault",0);
         break;
      case POS_TUNE_COSTFACOR_FAULT:
         PrintString("Cost Facotr Fault",0);
         break;
      case POS_TUNE_ICMDSAT_FAULT:
         PrintString("ICMD Sat Fault",0);
         break;
      case POS_TUNE_STOPPED_FAULT:
         PrintString("Unexpected Fault",0);
         break;
      case POS_TUNE_STOPPED_USER:
         PrintString("Stopped",0);
         break;
      case POS_TUNE_IDLE:
         PrintString("Idle",0);
         break;
      case POS_TUNE_SETUP:
         PrintString("Setup",0);
         break;
      case POS_TUNE_LMJR_EST:
         PrintString("Estimating LMJR : ",0);
         PrintSignedLong((long)(100.0*((float)BGVAR(s16_Cycle_LMJR_Counter)/CYCLE_LMJR_LIM)));
         PrintString(" %",0);
         break;
      case POS_TUNE_WAIT_ACTIVE:
         PrintString("Waiting For En",0);
         break;
      case POS_TUNE_TRAJ_SETUP:
         PrintString("Updating Trajectory",0);
         break;
      case POS_TUNE_IGRAV_SETUP:
         PrintString("Advanced Tuning : 0.8",0);
         break;
      case POS_TUNE_INITIAL_COST_CAPTURE:
         PrintString("Advanced Tuning : 0.9",0);
         break;
      case POS_TUNE_USERGAIN_SEARCH:
         PrintString("Advanced Tuning : 1",0);
         break;
      case POS_TUNE_KNLUSER_VERIFY:
         PrintString("Advanced Tuning : 1.1",0);
         break;
      case POS_TUNE_FILTER_VERIFY:
         PrintString("Advanced Tuning : 1.2",0);
         break;
      case POS_TUNE_ADV_KNLIV_SEARCH:
         PrintString("Advanced Tuning : 2",0);
         break;

      case POS_TUNE_ADV_ANTIVIB_INIT:
         PrintString("Advanced Tuning : 3",0);
         break;

      case POS_TUNE_ADV_KNLIV_AFTER_ANTIVIB_INIT:
         PrintString("Advanced Tuning : 4",0);
         break;

      case POS_TUNE_ADV_KNLP_SEARCH:
         PrintString("Advanced Tuning : 6",0);
         break;

      case KNLIV_KNLP_SUM_SEARCH:
         PrintString("Advanced Tuning : 6.2",0);
         break;
      case POS_TUNE_AV_FINE_TUNE:
         PrintString("Advanced Tuning : 6.6",0);
         break;

      case KNLUSERGAIN_VALIDITY_CHECK:
         PrintString("Advanced Tuning : 7",0);
         break;

      case POS_TUNE_ADV_KNLI_SEARCH:
         PrintString("Advanced Tuning : 8",0);
         break;

      case KNLUSERGAIN_VALIDITY_CHECK_2   :
         PrintString("Advanced Tuning : 8.2",0);
         break;

      case POS_TUNE_ADV_SEARCH_4_1:
         PrintString("Advanced Tuning : 10",0);
         break;

      case POS_TUNE_PERTUBATION:
         PrintString("Advanced Tuning : 10.1",0);
         break;

      case POS_TUNE_NLPEAFF_SEARCH:
         PrintString("Advanced Tuning : 10.2",0);
         break;

      case POS_TUNE_KNLI_VALIDATION:
         PrintString("Advanced Tuning : 10.3",0);
         break;

      case POS_TUNE_NLAFRC_SEARCH_1:
      case POS_TUNE_NLAFRC_SEARCH:
      case POS_TUNE_FINAL_COST_CAPTURE:
      case POS_TUNE_TRAJ_STOP:
      case POS_TUNE_MOVESMOOTH2_MODE_STATE:
         PrintString("Advanced Tuning : 11",0);
         break;
      case POS_TUNE_CONT_INITIAL_COST_SETUP:
      case POS_TUNE_CONT_INITIAL_COST_CALC:
         PrintString("Continuous Tune : Grading",0);
         break;

      case POS_TUNE_CONT_EFFORT_VERIFY:
         PrintString("Continuous Tune : Effort",0);
         break;

      case POS_TUNE_CONT_AV3_DEC_TUNE:
      case POS_TUNE_CONT_AV3_INC_TUNE:
         PrintString("Continuous Tune : AV3",0);
         break;

      case POS_TUNE_CONT_AV2_DEC_TUNE:
      case POS_TUNE_CONT_AV2_INC_TUNE:
         PrintString("Continuous Tune : AV2",0);
         break;

      case POS_TUNE_CONT_INC_KNLP_TUNE:
      case POS_TUNE_CONT_DEC_KNLP_TUNE:
         PrintString("Continuous Tune : Gain 1",0);
         break;
      case POS_TUNE_KNLI_DEC_TUNE:
      case POS_TUNE_KNLI_INC_TUNE:
         PrintString("Continuous Tune : Gain 2",0);
         break;

      case POS_TUNE_CONT_STOPPED:
         PrintString("Continuous Tune : Stopped",0);
         break;

      case POS_TUNE_AVHZ_TEST_MODE:
         PrintString("AV Test Mode",0);
         break;


      case POS_TUNE_SAVE_MODE:  //Udi July 24 2014: Save is AT done.
      case POS_TUNE_DONE:
         PrintString("Done",0);
         break;
      default:
         PrintString("Unknown",0);
   }

   PrintString("\n",0);
}


//**********************************************************
// Function Name: PrintPosTuneAV
// Description:
// Author:
//**********************************************************
int PrintPosTuneAV(int drive)
{

   int s16_temp = BGVAR(s16_HdTune_Av_Mode);
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   PrintString("AntiVib: ",0);
   if (s16_temp > 3) s16_temp-=3;
   if ((s16_temp <= 0) || (BGVAR(s16_Pos_tune_Mode) == 5)) PrintString("NA",0);
   else if (s16_temp == 1) PrintString("AV2",0);
   else if (s16_temp == 2) PrintString("AV3",0);
   else if (s16_temp == 3) PrintString("AV2 and AV3",0);
   else PrintString("SW Bug",0);

   return 1;
}


//**********************************************************
// Function Name: PrintPosTuneWarnings
// Description:
// Author:
//**********************************************************
int PrintPosTuneWarnings(int drive)
{
   int debug_level = 0;
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   if(s16_Number_Of_Parameters != 0)
      debug_level = 1;

   PrintString("Warnings:\n",0);
   return PrintStringsFromTable((long)(BGVAR( s32_Pos_Tune_Bits)) ,AT_Warning_Strings, PRINT_BY_BIT_ARRAY, debug_level);

 }

//**********************************************************
// Function Name: PrintPosTuneDebugWarnings
// Description:
// Author:
//**********************************************************
int PrintPosTuneDebugWarnings(int drive)
{
   int s16_warning_exists = 0;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   //Todo: find way to print this  !!
   if (BGVAR(u16_Cycle_Length) == 0)
   {
      PrintString("Identifying Cycle\n",0);
      s16_warning_exists = 1;
   }
   if ((VAR(AX0_AutoTune_Bits)  & AT_ICMD_OV_MASK) != 0)
   {
      PrintString("ICMD Sat\n",0);
      s16_warning_exists = 1;
   }

   //Todo: find way to print this  !!
   if (((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_KNLIV_DELTA_WARN) != 0) && (s16_Number_Of_Parameters != 0))
   {
      PrintString("KNLIV Step Updated :",0);
      PrintSignedLong(s16_PosTune_Knliv_Step);PrintChar(SPACE);
      PrintSignedLong(s16_PosTune_Knliv_Step2);PrintCrLf();
      s16_warning_exists = 1;
   }
   //             //Todo: find way to print this  !!
   if ((BGVAR(s16_Pos_Tune_State) == POS_TUNE_STOPPED_FAULT) && (s16_Number_Of_Parameters != 0))
   {
      PrintString("SC: ",0);
      PrintSignedInteger(s16_Pos_Tune_State_Captured);
      PrintCrLf();
      s16_warning_exists = 1;
   }

     //Todo: find way to print this  !!
   if (s16_warning_exists == 0) PrintString("None\n",0);
   return 0;
}

//**********************************************************
// Function Name: PosTuneStCommand
// Description:
// Author:
//**********************************************************
int PosTuneStCommand(int drive)
{
 //  // AXIS_OFF;
   static int s16_state = 0;
   if (u8_Output_Buffer_Free_Space < 80) return (SAL_NOT_FINISHED);

   switch (s16_state)
   {
      case 0:
         s16_state = 1;
         PrintPosTuneMode(drive);
         PrintPosTuneAV(drive);
         PrintPosTuneState(drive);
         break;
      case 1:
         s16_state = 2;
         if ((BGVAR(s16_Pos_Tune_State) != POS_TUNE_SAVE_MODE) && (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE))
         {
         if ( (BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE) ||
              (BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_INT_MODE_1_CYCLE) )
            PrintString("Trajectory : Internal,Auto \n",0);
         else if (BGVAR(s16_Pos_Tune_Traj_Mode) == PTT_EXT_MODE) PrintString("Trajectory : Internal,User defined\n",0);
         else
            PrintString("Trajectory : External\n",0);
         }
         break;
      case 2:
         s16_state = 3;
         PrintPosTuneWarnings(drive);
        // PrintPosTuneDebugWarnings(drive);
         break;

      case 3:
         PrintString("Cycles: ",0);PrintUnsignedInteger(BGVAR(u16_Cycle_Cycles_Counter));PrintCrLf();
         if ((BGVAR(s16_Pos_tune_Mode) == 4) && // lmjr est only
             (BGVAR(s16_Pos_Tune_State) == POS_TUNE_DONE) )
         {
            PrintString("LMJR: ",0);
            PrintUnsignedLong(BGVAR(u32_AT_LMJR));
            PrintCrLf();
         }
         s16_state = -1;
         if (BGVAR(u16_HdTune_Defaults) != 0) s16_state = 4;
         break;

      case 4:
         PrintString("Quality: ",0);
         if (BGVAR(s16_Pos_Tune_State) >= POS_TUNE_SAVE_MODE)
            PrintSignedInteger(BGVAR(s16_Cost_Improvement));
         else
            PrintString("NA",0);
         PrintCrLf();
         s16_state = -1;
         break;
   }
   if (s16_state > 0)  return (SAL_NOT_FINISHED);
   s16_state = 0;
   return (SAL_SUCCESS);
}




//**********************************************************
// Function Name: HighestTuneWarning
// Description:
//    return the last AutoTuning warning.
//    Loop on all warnings in the string table and find first warning now exists in s32_Pos_Tune_Bits
//    This warnings are reported with offset of 30, as defined for p9-30 values.
// Author: Udi
//**********************************************************
unsigned int HighestTuneWarning(int drive)
{
   int s16_index = 0;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   while(AT_Warning_Strings[s16_index].u16_level < 0xFFFF )
   {
      if(((BGVAR(s32_Pos_Tune_Bits) & AT_Warning_Strings[s16_index].u32_string_id) != 0) &&
           (AT_Warning_Strings[s16_index].u16_level < 1 )) // Do not show "debug" messages.
      {
         return  (30 + s16_index);
      }
      s16_index ++;
   }
   return 0;
}



//**********************************************************
// Function Name: PosTuneWarning
// Description:
//    return the last AutoTuning warning.
// Author: Udi
//**********************************************************
int PosTuneWarning(long long *data,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   *data = (long long)HighestTuneWarning(drive);

   return (SAL_SUCCESS);
}



//**********************************************************
// Function Name: PosTuneStCommandViaPParam
// Description:
//    This function return the Pos tune status in binary code in response to P9-30 read.
//    The meaning of the return value is as follows:
//       0 =         Auto-tune in idle mode
//       1 =         Auto-tune is active
//       2 =         Auto-tune is successfully finished
//       10...29 =   Lexium specific failure code in case that in the Lexium procedure something fails (implemented by Andreas).
//       30...49 =   Auto-tune specific warnings generated by code from Yuval.
//       50..69 =    Auto-tune failure generated by code from Yuval.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int PosTuneStCommandViaPParam(long long *data,int drive)
{


   unsigned int u16_pos_tune_status = 0xFFFF; // Set value to 0xFFFF, which is not defined. If this value is returned, we know that we did not
                                              // cover all potential cases in the "if / else if" conditions below.

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // 1. Check if activation via P-232 failed
   if(BGVAR(u16_Lex_AT_Err) != LEX_AT_ERR_NO_ERROR)
   {
      // If there is a Lexium pre activation error.
      *data = (long long)BGVAR(u16_Lex_AT_Err);
       return (SAL_SUCCESS);
   }

   // 2. Check if finished successfully
   if ((BGVAR(s16_Pos_Tune_State) == POS_TUNE_DONE) ||
             (BGVAR(s16_Pos_Tune_State) == POS_TUNE_SAVE_MODE))
   {
      *data = 2LL; // Indiacte position-tuning is finished successfully
      return (SAL_SUCCESS);
   }


   // 3. Check if AT  faild (Yuval code).
   if (BGVAR(s16_Pos_Tune_State) < 0)
   {
      *data = (long long)((-s16_Pos_Tune_State) + 49); // Indiacte Failure
      return (SAL_SUCCESS);
   }



   // 4. Loop on all warnings in the string table and find first warning now exists in s32_Pos_Tune_Bits
   // This warnings are reported with offset of 30, as defined for p9-30 values.
   u16_pos_tune_status = HighestTuneWarning(drive);

   if(u16_pos_tune_status != 0)
   {
      *data = (long long)u16_pos_tune_status; // Runing, warning exists.
      return (SAL_SUCCESS);
   }

   // 5. If not finished and no warnings - check if working ..
   if ((BGVAR(s16_Pos_Tune_State) > POS_TUNE_IDLE) && (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE))
      u16_pos_tune_status = 1; // Indiacte position-tuning is active

   else if (BGVAR(s16_Pos_Tune_State) == POS_TUNE_IDLE)
      u16_pos_tune_status = 0; // Indiacte position-tuning is inactive

   *data = (long long)u16_pos_tune_status;

   return (SAL_SUCCESS);
}

int PosTuneEffortCommand(int drive)
{
   long long effort = s64_Execution_Parameter[0];
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_SERVICE_MASK) != 0) return (AT_ACTIVE);
   if ((effort < 10000) || (effort > 1000000)) return (VALUE_OUT_OF_RANGE);
   BGVAR(s32_Pos_Tune_C_Effort) = (long)effort;
   return (SAL_SUCCESS);
}

int PosTuneStiffnessCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_SERVICE_MASK) != 0) return (AT_ACTIVE);
   BGVAR(s16_Pos_Tune_Stiffness) = (int)param;
   return (SAL_SUCCESS);
}


int PosTuneLMJREnCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_SERVICE_MASK) != 0) return (AT_ACTIVE);
   BGVAR(s16_Pos_Tune_LMJR) = (int)param;
   return (SAL_SUCCESS);
}

int PosTuneIgravEnCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((BGVAR(s32_Pos_Tune_Bits) & POS_TUNE_SERVICE_MASK) != 0) return (AT_ACTIVE);
   BGVAR(s16_Pos_Tune_Igrav) = (int)param;
   return (SAL_SUCCESS);
}






// support lmjr update for pick and place
// lmjr is identified every acceleration phase from standstill
// lmjrmode - 0 default user defined
// lmjrmode - 1 start lmjr normalization - replace lmjr with identified one and
//            update controller gains. also save fric and unbalance. at the end
//            lmjr mode chaages to 2. if on powerup lmjrmode is 1 it will reset to 0
// lmjrmode - 2 - lmjr is updated with lmjr indetified each acceleration from standstill



int SalLMJRMODECommand(long long param, int drive)
{
   //// AXIS_OFF;
   //int s32_temp;
   ++drive;


   if (param == 2) return (NOT_AVAILABLE);
   if ((int)param == BGVAR(s16_LMJR_Mode)) return (SAL_SUCCESS);
/*
   if ((BGVAR(s16_Pos_Tune_State) > 0)             &&
       (BGVAR(s16_Pos_Tune_State) != POS_TUNE_DONE)  ) return (AT_ACTIVE);

   if ((param == 1LL)                     &&
       (VAR(AX0_s16_Cycle_Dir) != 0x7FFF) &&
       (BGVAR(s16_LMJR_Mode) == 0)          ) return (NOT_AVAILABLE);

   if ((param == 0LL)                     &&
       (VAR(AX0_s16_Cycle_Dir) != 0x7FFF) &&
       (BGVAR(s16_LMJR_Mode) == 1)          ) STOP_CYCLE_IDENT;

   BGVAR(s16_LMJR_Mode) = (int)param;

   if (BGVAR(s16_LMJR_Mode) == 1)
   {
      STOP_CYCLE_IDENT;
      s32_temp = Cntr_1mS;
      while (s32_temp == Cntr_1mS){};
      s32_temp = Cntr_1mS;
      while (s32_temp == Cntr_1mS){};
      EN_CYCLE_IDENT;
   }
   else // lmjrmode = 0
     {STOP_CYCLE_IDENT;}*/

   return (SAL_SUCCESS);
}


/*void OnLineLMJRHandler(int drive)
{
   // AXIS_OFF;
   int s16_temp;
   long s32_vcmd_data,s32_temp;

   if (BGVAR(s16_LMJR_Mode) == 0)
   {
      BGVAR(u64_Sys_Warnings) &= ~ONLINE_LMJR_ACTIVE;
      return;
   }

   if ((BGVAR(s16_LMJR_Mode) == 1) && (!Enabled(drive)))
   {
      BGVAR(s16_LMJR_Mode) = 0;
      STOP_CYCLE_IDENT;
   }

   BGVAR(u64_Sys_Warnings) |= ONLINE_LMJR_ACTIVE;

   if ((BGVAR(s16_LMJR_Mode) == 2) && (BGVAR(s16_Online_Lmjr_State) == ONLINE_LMJR_UPDATE))
   {
     // there could be a timing issue where the state changes to ONLINE_LMJR_WAIT_FOR_ACC_START
     // but it should not affect the proccess - writing it here just to be reminded of the timing issue (yuval , may 2014)
     s32_temp = (long)((float)((long)BGVAR(u32_Online_Lmjr_Est)  - (long)BGVAR(u32_LMJR_User))*(float)BGVAR(s16_Online_Lmjr_Factor)/1000.0) + (long)BGVAR(u32_LMJR_User);

     // make sure total knld does not change - change knlusergain
     BGVAR(u32_Nl_Kpgf_User) = (unsigned long)((float)BGVAR(u32_Nl_Kpgf_User)*((float)BGVAR(u32_LMJR)+1000)/((float)s32_temp + 1000));
     if (BGVAR(u32_Nl_Kpgf_User) > 3000) BGVAR(u32_Nl_Kpgf_User) = 3000;
     UpdateLmjr((long long)s32_temp,drive); // this will also force pos\vel config

     WaitForNext32kHzTaskStart();   // make sure 1ms isr will not occur during next command
     if (BGVAR(s16_Online_Lmjr_State) == ONLINE_LMJR_UPDATE) BGVAR(s16_Online_Lmjr_State) = ONLINE_LMJR_WAIT_FOR_STANDSTILL;
   }

   if ((BGVAR(s16_LMJR_Mode) == 1) && (BGVAR(s16_Cycle_LMJR_Counter) == CYCLE_LMJR_LIM))
   { // normalization must be done over a cycle
      AutoTuneLMJRNormalization(drive,1);
      BGVAR(s32_Online_Lmjr_Tub) = BGVAR(s32_Cycle_T_Unbalance);
      BGVAR(s32_Online_Lmjr_Fric) = BGVAR(s32_Cycle_Fric_Coeff);
      BGVAR(s16_LMJR_Mode) = 2;
      // keep fric and unbalance data
      BGVAR(s16_Online_Lmjr_State) = ONLINE_LMJR_WAIT_FOR_STANDSTILL;
      STOP_CYCLE_IDENT;
      do {
         s16_temp = Cntr_3125;
         s32_vcmd_data = LVAR(AX0_s32_Pos_Vcmd);
         } while (s16_temp != Cntr_3125);

      BGVAR(s32_Online_Lmjr_Prev_Vcmd_Data) = labs(s32_vcmd_data);
      return;
   }
}*/


// #pragma CODE_SECTION(OnLineLmjrEst, "ramfunc");
/*void OnLineLmjrEst(int drive)
{ // if new lmjr found - update user value and redesign position loop
   // AXIS_OFF;
   int s16_vel,s16_temp;
   long s32_temp,s32_vcmd_data;
   long long s64_captured_icmd_int,s64_temp;
   float f_temp;

   if (BGVAR(s16_LMJR_Mode) != 2) return;
   if (!Enabled(drive)) BGVAR(s16_Online_Lmjr_State) = ONLINE_LMJR_WAIT_FOR_STANDSTILL;

   do {
      s16_temp = Cntr_3125;
      s32_vcmd_data = LVAR(AX0_s32_Pos_Vcmd);
      } while (s16_temp != Cntr_3125);

   s32_vcmd_data =labs(s32_vcmd_data);

   switch (BGVAR(s16_Online_Lmjr_State))
   {
      case ONLINE_LMJR_UPDATE:
      case ONLINE_LMJR_WAIT_FOR_STANDSTILL:
         if (s32_vcmd_data < 2) BGVAR(s16_Online_Lmjr_State) = ONLINE_LMJR_WAIT_FOR_ACC_START;
         break;

      case ONLINE_LMJR_WAIT_FOR_ACC_START:
         do {
            s16_temp = Cntr_3125;
            LLVAR(AX0_u32_Abs_Icmd_Acc_Lo) = 0;
             LVAR(AX0_u32_Abs_Icmd_ACC_Cntr) = 0L;
            BGVAR(s32_Online_Lmjr_Timer) = Cntr_1mS;
            BGVAR(s64_Online_Lmjr_StandStill_Pos) = LLVAR(AX0_u32_Pcmd_Raw_Lo);
            } while (s16_temp != Cntr_3125);
            if (s32_vcmd_data > 2) BGVAR(s16_Online_Lmjr_State) = ONLINE_LMJR_WAIT_FOR_ACC_END;
         break;

      case ONLINE_LMJR_WAIT_FOR_ACC_END:
         if (s32_vcmd_data > BGVAR(s32_Online_Lmjr_Prev_Vcmd_Data)) break;

         s32_temp = labs(Cntr_1mS - BGVAR(s32_Online_Lmjr_Timer));
         if (s32_temp < 5)
         {
            BGVAR(s16_Online_Lmjr_State) = ONLINE_LMJR_WAIT_FOR_STANDSTILL;
            break;
         }

         // capture timer and integral
         do {
            s16_temp = Cntr_3125;
            s64_captured_icmd_int = LLVAR(AX0_u32_Abs_Icmd_Acc_Lo);
            s32_temp = Cntr_1mS;
            s64_temp = LLVAR(AX0_u32_Pcmd_Raw_Lo);
            s16_vel = VAR(AX0_s16_Vel_Var_Fb_0);
            } while (s16_temp != Cntr_3125);

         s32_temp = labs(s32_temp - BGVAR(s32_Online_Lmjr_Timer));
         s64_temp = llabs(s64_temp - BGVAR(s64_Online_Lmjr_StandStill_Pos));

         // calc new lmjr
         f_temp = (float)BGVAR(u32_Motor_Kt) * BGVAR(s32_Drive_I_Peak) * (float)s64_captured_icmd_int * 6.55433
                  - 1943866300 * (float)BGVAR(s32_Online_Lmjr_Tub) * (float)s32_temp
                  - BGVAR(s32_Online_Lmjr_Fric) * (float)s64_temp * 2843.72;
         f_temp = 1000 * f_temp / ((float)s16_vel * (float)BGVAR(s32_V_Lim_Design) * BGVAR(s32_Motor_J)) - 1000.0;
         if (f_temp < 0) f_temp = 0;
         BGVAR(u32_Online_Lmjr_Est) = (long)f_temp;
         // signal bg toupdate lmjr and redesign pos loop
         BGVAR(s16_Online_Lmjr_State) = ONLINE_LMJR_UPDATE;
         break;
   }
   BGVAR(s32_Online_Lmjr_Prev_Vcmd_Data) = s32_vcmd_data;
}*/

//******************************************************************************
// Function Name: LexiumPrepareOrRestoreAutoTuneSettings
// Description:
//          This function is used in order to prepare the Drive in order to perform
//          the auto-tuner properly. Especially the OPMODE needs to be set to 8.
//
//          u16_action - 0 = Prepare the Drive settings (done when auto tuner starts)
//                       1 = Restore the Drive settings (done when user finalizes the tuning)
// Author: APH
// Algorithm:
// Revisions:
//     Udi Aug 13, 2014 : Also restore Drive-Enable  state here.
//******************************************************************************
void LexiumPrepareOrRestoreAutoTuneSettings(int drive, unsigned int u16_action)
{
   // AXIS_OFF;

   if(u16_action == 0) // Backup parameters
   {
      // Tell the Lexium speed and torque command functions to not interfere the jog move.
      BGVAR(u16_Lex_Jog_Or_Autotune_Active) = 1;


      // Only for easy-tuning or comfort tuning
      if( ((BGVAR(u16_P2_32_Lex_Auto_Tune_Mode) >= 1) && (BGVAR(u16_P2_32_Lex_Auto_Tune_Mode) <= 3)) ||
          ((BGVAR(u16_P2_32_Lex_Auto_Tune_Mode) >= 52) && (BGVAR(u16_P2_32_Lex_Auto_Tune_Mode) <= 53)) )
      {
         // Here try to switch the OPMODE to 8, because the auto-tuner can only be started in this OPMODE.
         if( ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_SERCOS)        ||
             ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERNET_IP)   ||
             ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERCAT)         )
         {
         }
         else if ((BGVAR(u16_P1_01_CTL_Current) & 0xff) == SE_OPMODE_CANOPEN)
         {
            // Capture the mode of operation
            BGVAR(s16_Lexium_Autotune_Opmode_Backup) = p402_modes_of_operation_display;

            // Here try to switch the canopen opmode to 1
            BGVAR(s16_CAN_Opmode_Temp) = 1;  // 1 is profile position mode (internal opmode 8)
            BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
            CheckCanOpmode(drive);
         }
         else
         {
            BGVAR(s16_Lexium_Autotune_Opmode_Backup) = VAR(AX0_s16_Opmode);
            SetOpmode(drive, 8);
         }
      }
   }
   else // Restore parameters
   {
      // First restore the original settings (undo the changes when stepping out of auto-tune mode)
      if( ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_SERCOS)        ||
          ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERNET_IP)   ||
          ((BGVAR(u16_P1_01_CTL_Current) & 0x00FF) == SE_OPMODE_ETHERCAT)         )
      {
      }
      else if ((BGVAR(u16_P1_01_CTL_Current) & 0xff) == SE_OPMODE_CANOPEN)
      {
         BGVAR(s16_CAN_Opmode_Temp) = BGVAR(s16_Lexium_Autotune_Opmode_Backup);
         BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
         CheckCanOpmode(drive);
      }
      else
      {
         SetOpmode(drive, BGVAR(s16_Lexium_Autotune_Opmode_Backup));
      }

      // Allow the Lexium speed and torque command functions to do their regular job.
      BGVAR(u16_Lex_Jog_Or_Autotune_Active) = 0;

      // If the Drive was disabled when the auto-tuner has been started
      if(BGVAR(u16_Lexium_Autotune_General_Purpose) & LEX_AUTOTUNE_AXIS_WAS_DISABLED)
         DisableCommand(drive);

   }
}

//******************************************************************************
// Function Name: LexiumAutoTuneStateMachine
// Description:
//          This function is mainly used to enable the Drive before starting the
//          auto-tuning process. This is a Schneider requirement. This function
//          is only supposed to be active in case of a so-called single-shot
//          auto-tuning method (auto-tuner that does NOT run contibuously while
//          the Drive is in regular operational mode).
//
// Author: APH
// Algorithm:
// Revisions:
//******************************************************************************
void LexiumAutoTuneStateMachine (int drive)
{
    // Temporary variables
   int s16_temp_access_channel = BGVAR(s16_Lexium_ExecPParam_Acces_Channel);
   long s32_p_param_value = 0;
   // AXIS_OFF;

   switch(BGVAR(u16_Lexium_Autotune_State))
   {
      case (LEX_ATUNE_STATE_IDLE):
         // Clear the abort / finalize request, otherwise do nothing since auto-tuning is not running
         BGVAR(u16_Lexium_Autotune_General_Purpose) &= ~(LEX_ABORT_AUTOTUNE | LEX_FINALIZE_AUTOTUNE_AND_SAVE | LEX_FINALIZE_AUTOTUNE_AND_DISCARD);
      break;

      case (LEX_ATUNE_STATE_START): // Start auto-tuning
         // Wait until the OPMODE change, initiated for the auto tuning within "LexiumPrepareOrRestoreAutoTuneSettings", is finished.
         if(IsOpmodeChangeInProgress(drive))
         {
            // Do nothing if OPMODE-change is in progress
         }
         else if(BGVAR(u16_Auto_Adaptive_Tuning_Triggered))
         {
            BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_A_ADAPT_ACIVATE; // Jump to next state
         }
         else
         {
             BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_EC_ENABLE_AXIS; // Jump to next state
         }
      break;

      case (LEX_ATUNE_STATE_EC_ENABLE_AXIS): // Enable the Drive if needed
         if(!Enabled(drive))
         {
            EnableCommand(drive);
            BGVAR(u16_Lexium_Autotune_General_Purpose) |= LEX_AUTOTUNE_AXIS_WAS_DISABLED; // State that the axis was enabled by the auto-tuning process
         }
         else
         {
            BGVAR(u16_Lexium_Autotune_General_Purpose) &= ~LEX_AUTOTUNE_AXIS_WAS_DISABLED; // State that the axis was NOT enabled by the auto-tuning process
         }
         // Capture time in order to check if enable succeeds
         BGVAR(s32_Lexium_Autotune_Timer) = Cntr_1mS;
         BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_EC_ENABLE_AXIS_DONE; // Jump to next state
      break;

      case (LEX_ATUNE_STATE_EC_ENABLE_AXIS_DONE): // Wait for the Drive to be enabled (including timeout handling)
         // wait for drive to be enabled and to brake to be released (allow motion = 1)
         if(Enabled(drive) && BGVAR(u16_Allow_Motion) == 1)
         {
            BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_EC_TRIGGER_HDTUNE; // Jump to next state
         }
         // nitsan: if brake is connected, need to consider brake release time (user time + mtp time)
         else if(PassedTimeMS(500L + (long)(BGVAR(u16_P1_42_Brake_Time_On) + BGVAR(u16_Motor_Brake_Release_Time)), BGVAR(s32_Lexium_Autotune_Timer)))
         {
            // State that enable failed
            BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_ENABLE_FAILED;

            // No enable after 0.5[s]
            BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_IDLE; // Reset state machine
         }
      break;

      case (LEX_ATUNE_STATE_EC_TRIGGER_HDTUNE): // Try to start the auto-tuner

         STORE_EXECUTION_PARAMS_0_1;

         // Simulate ASCII command "HDTUNE BGVAR(u16_Hdtune_Method) 1"
         s64_Execution_Parameter[0] = (long long)BGVAR(u16_Hdtune_Method);
         s64_Execution_Parameter[1] = 1;
         s16_Number_Of_Parameters = 2;
         if (PosTuneCommand(drive) == SAL_SUCCESS)
         {
            // Capture time in order to check if activation succeeds
            BGVAR(s32_Lexium_Autotune_Timer) = Cntr_1mS;
            BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_EC_TRIGGER_HDTUNE_DONE; // Jump to next state
         }
         else
         {
            // State that activation of the auto-tuner failed
            BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_ACTIVATION_FAILED;

            BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_IDLE; // Reset state machine
         }

         RESTORE_EXECUTION_PARAMS_0_1;

         // Capture time in order to check if the activation succeeds
         BGVAR(s32_Lexium_Autotune_Timer) = Cntr_1mS;
      break;

      case (LEX_ATUNE_STATE_EC_TRIGGER_HDTUNE_DONE): // Wait for the auto-tuner to finish the activation.
         if(VAR(AX0_s16_Cycle_Bits) & POS_TUNE_RT_ACTIVE) // If auto-tuner is active
         {
            BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_EC_FINISH; // Jump to next state
         }
         else if(PassedTimeMS(2000L, BGVAR(s32_Lexium_Autotune_Timer)))
         {
            STORE_EXECUTION_PARAMS_0_1;

            // Simulate ASCII command "HDTUNE 0" and therefore abort the tuning
            s64_Execution_Parameter[0] = 0;
            s16_Number_Of_Parameters = 1;
            PosTuneCommand(drive);

            RESTORE_EXECUTION_PARAMS_0_1;

            // State that the auto-tuner did not run after a certain time, here 2[s]
            BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_ACTIVATION2_FAILED;

            BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_IDLE; // Reset state machine
         }
      break;

      case (LEX_ATUNE_STATE_EC_FINISH): // Here wait for another function to tell what to do
         //Udi July 20 2014: If from DTM and Finished - save to ram and exit AutoTune

         if ((u16_Login == 3) && (BGVAR(s16_Pos_Tune_State) == POS_TUNE_DONE))
         {
            // Restore the original settings of opmode and Drive Enable.
            LexiumPrepareOrRestoreAutoTuneSettings(drive, 1);
            BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_IDLE; // Reset state machine
         }

         if(BGVAR(u16_Lexium_Autotune_General_Purpose) & LEX_ABORT_AUTOTUNE)
         {
            // Simulate ASCII command "HDTUNE 0" and therefore abort the tuning
            STORE_EXECUTION_PARAMS_0_1;
            s64_Execution_Parameter[0] = 0;
            s16_Number_Of_Parameters = 1;
            PosTuneCommand(drive);
            RESTORE_EXECUTION_PARAMS_0_1;

            // Restore the original settings.
            LexiumPrepareOrRestoreAutoTuneSettings(drive, 1);

            BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_IDLE; // Reset state machine
         }
         // If someone issued a finalization of the auto-tuner plus keeping the results
         else if(BGVAR(u16_Lexium_Autotune_General_Purpose) & LEX_FINALIZE_AUTOTUNE_AND_SAVE)
         {
            // If the auto-tuner is currently running
            if((VAR(AX0_s16_Cycle_Bits) & POS_TUNE_RT_ACTIVE) && (BGVAR(s16_Pos_Tune_State) < POS_TUNE_SAVE_MODE))
            {
               // Just clear the finalize request and stay in this mode
               BGVAR(u16_Lexium_Autotune_General_Purpose) &= ~LEX_FINALIZE_AUTOTUNE_AND_SAVE;
            }
            else // auto-tunner is not active any more
            {
               // Restore the original settings.
               LexiumPrepareOrRestoreAutoTuneSettings(drive, 1);

               // Set HDTUNESAVEMODE to 0
               s32_p_param_value = HDTUNESAVE_KEEP_RES_OVRRIDE_BKUP;
               ExecutePParam(drive, 918, OP_TYPE_WRITE, &s32_p_param_value, s16_temp_access_channel);
               // Temp !! Udi July 20 2014: Trigger save, but is this too early ???

               // Indicate that a SAVE has to be performed in case that the write command has been succeeded.
               //BGVAR(u8_Hmi_Mode_Edit_Param_Perform_Action) = 1;
               BGVAR(u16_Lexium_Bg_Action) |= LEX_DO_NV_SAVE;

               BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_IDLE; // Reset state machine
            }
         }
         // If someone issued a finalization of the auto-tuner plus discarding the results
         else if(BGVAR(u16_Lexium_Autotune_General_Purpose) & LEX_FINALIZE_AUTOTUNE_AND_DISCARD)
         {
            // If the auto-tuner is currently running
            if((VAR(AX0_s16_Cycle_Bits) & POS_TUNE_RT_ACTIVE) && (BGVAR(s16_Pos_Tune_State) < POS_TUNE_SAVE_MODE))
            {
               // Just clear the finalize request and stay in this mode
               BGVAR(u16_Lexium_Autotune_General_Purpose) &= ~LEX_FINALIZE_AUTOTUNE_AND_DISCARD;
            }
            else // auto-tunner is not active any more
            {
               // Restore the original settings.
               LexiumPrepareOrRestoreAutoTuneSettings(drive, 1);

               // Set HDTUNESAVEMODE to 1
               s32_p_param_value = HDTUNESAVE_MODE_IGNORE;
               ExecutePParam(drive, 918, OP_TYPE_WRITE, &s32_p_param_value, s16_temp_access_channel);

               BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_IDLE; // Reset state machine
            }
         }
      break;

      case (LEX_ATUNE_STATE_A_ADAPT_ACIVATE):
         // HERE ACTIVATE THE AUTO ADAPTIVE TUNING. IF IT SUCCEEDS THEN JUMP TO THE NEXT STATE VIA:
         // BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_A_ADAPT_FINISH; // Jump tp next state
         //
         // IF IT FAILS THEN MENTION THE ROOT CAUSE IN "BGVAR(u16_Lex_AT_Err)" AND JUMP BACK TO STATE IDLE VIA:
         // BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_IDLE; // Reset state machine
      break;

      case (LEX_ATUNE_STATE_A_ADAPT_FINISH):
         // If an auto-tune abort has been issued
         if(BGVAR(u16_Lexium_Autotune_General_Purpose) & LEX_ABORT_AUTOTUNE)
         {
            // HERE DO THE ACTION WHICH IS REQUIRED FOR ABORTING AN AUTO ADAPTIVE TUNING.

            BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_IDLE; // Reset state machine
         }
      break;

      default: // Invalid state, not supposed to happen at all
         BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_IDLE; // Reset state machine
      break;
   }
}



//******************************************************************************
// Function Name: SetPParamValue
// Description:
//          Set value for one PParam, Use for for auto-tune related parameter
// Author: Udi
// Algorithm:
//******************************************************************************
int SetPParamValue( unsigned int u16_PParam_Num,  long s32_Value, int drive)
{
   int s16_return_value;
   int s16_temp_access_channel = BGVAR(s16_Lexium_ExecPParam_Acces_Channel);  // Save the current access-channel (channel who triggered this function)

   s16_return_value = ExecutePParam(drive, u16_PParam_Num, OP_TYPE_WRITE, &s32_Value, s16_temp_access_channel);
   if (s16_return_value == SAL_SUCCESS)
   {
         return SAL_SUCCESS;
   }
   // Remeber first error by Pparam
   if(BGVAR(u16_Lex_AT_Err) == LEX_AT_ERR_NO_ERROR)
   {

      switch (u16_PParam_Num)
      {
         case 915: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_P9_15; break;
         case 916: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_P9_16; break;
         case 917: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_P9_17; break;
         case 918: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_P9_18; break;
         case 919: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_P9_19; break;
         case 921: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_P9_21; break;
         case 922: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_P9_22; break;
         case 923: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_P9_23; break;
         case 924: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_P9_24; break;
         case 932: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_P9_32; break;

         case 925:
            switch(s16_return_value)
            {
               case HDTUNE_DIST_NOT_EQUAL: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_DIST_NOT_EQUAL; break;
               case HDTUNE_VCRUISE_LOW:    BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_VCRUISE_LOW; break;
               case HDTUNE_WRONG_PROFILE:  BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_WRONG_PROFILE; break;
               case HDTUNE_DIST_DIFF_SIGN: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_DIST_DIFF_SIGN; break;
               case HDTUNE_DIST_SAME_SIGN: BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_DIST_SAME_SIGN; break;
               default : BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_DEFAULT_ERROR;
            }
            break;

         default : BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_DEFAULT_ERROR;
      }

   }
   return  LEX_AT_ERR_DEFAULT_ERROR;
}



//******************************************************************************
// Function Name: SetFewAutoTuneDefaults
// Description:
//          Set Defaults to few HDTUNExxx (P9-xx) related parameter.
// Author: Udi
// Algorithm:
// Revisions:
//******************************************************************************
int SetFewAutoTuneDefaults( int drive)
{

   int s16_return_value;

   // Set HDTUNESAVEMODE to 2 if local HMI or 0 if DTM
   //if(u16_Login == 3)
   if((BGVAR(u16_Lexium_Acces_Rights_State) == ((LEX_CH_MODBUS_RS485<<8) | 0x0001)))
     s16_return_value = SetPParamValue( 918, HDTUNESAVE_KEEP_RES_KEEP_BKUP, drive);
   else
     s16_return_value = SetPParamValue( 918, HDTUNESAVE_MODE_WAIT, drive);


   if(s16_return_value == SAL_SUCCESS)
       s16_return_value = SetPParamValue( 919,  1L, drive);

   if(s16_return_value == SAL_SUCCESS)
       s16_return_value = SetPParamValue( 921,  200L, drive);

   if(s16_return_value == SAL_SUCCESS)
      s16_return_value = SetPParamValue( 922, 0L, drive);

 //  if(s16_return_value == SAL_SUCCESS)
 //     s16_return_value = SetPParamValue( 923, 0L, drive);

// P9-24 was removed
//   if(s16_return_value == SAL_SUCCESS)
//      s16_return_value = SetPParamValue( 924, 0L, drive);

      //Set HDTUNEEFFORT to its default value: 100 (1000 in =S= units)
   if(s16_return_value == SAL_SUCCESS)
       s16_return_value = SetPParamValue( 231,  1000L, drive);




return s16_return_value;
}


//**********************************************************
// Function Name: SetPPAramByTable
// Description: Set values to few PParam, acording to hard coded table
// Author:
//**********************************************************
int SetPPAramByTable(const AT_Modes_Pparams_Struct *Pparams_Table, unsigned int u16_Mode, int drive)
{
   unsigned int u16_line;
   int s16_return_value = SAL_SUCCESS;
   unsigned int u16_column;
   //Find colums with given mode
   for(u16_column=0; u16_column <= NUM_AT_MODES; u16_column++)
    {
       if(Pparams_Table[0].values[u16_column] == u16_Mode )
           break;

    }
   if(Pparams_Table[0].values[u16_column] != u16_Mode )
       return VALUE_IS_NOT_ALLOWED;

   for(u16_line=1; u16_line < 99; u16_line++)
   {
      if(Pparams_Table[u16_line].pparam >= 9999)
         break; // Last line in the table

      if(Pparams_Table[u16_line].values[u16_column] == -1)
         continue; //-1 is Illegal value in our case.

      s16_return_value = SetPParamValue( Pparams_Table[u16_line].pparam,
                                        (long) Pparams_Table[u16_line].values[u16_column], drive);

      if(s16_return_value != SAL_SUCCESS)
         return s16_return_value;
   }

   return s16_return_value;
}


//**********************************************************
// Function Name: StopLexiumAT
// Description: Handle value 0 to P2-32
// Author:
//**********************************************************
int StopLexiumAT(int drive)
{
   // AXIS_OFF;

   SetPParamValue(925, 0L, drive);

   // Trigger an auto-tune abort
   BGVAR(u16_Lexium_Autotune_General_Purpose) |= LEX_ABORT_AUTOTUNE;

   // This instruction is just used to prevent the local HMI to jump to mode auto-tune. No other purpose.
   BGVAR(u16_Auto_Adaptive_Tuning_Triggered) = 1;

   //Setting p2-32 to Zeo from DTM after AT means to restore old parameters
   if (( BGVAR(s16_Allow_Restore_Param) == 1) && (u16_Login == 3))
   {
      PosTuneRestore(drive,0);
      PositionConfig(drive, 1);
      VAR(AX0_s16_Crrnt_Run_Code) |= (COPY_SHARED_POS_COEF_MASK | COPY_NCT_COEF_MASK);
      BGVAR(s16_Allow_Restore_Param) = 0;
   }
   return SAL_SUCCESS;

}

//******************************************************************************
// Function Name: SalSelectAndStartLexiumAutoTune
// Description:
//          This function is called by setting P2-32, with 2 purposes:
//           a) Setting the corresponding P2-32 variable to the dedicated value.
//           b) Initiate an auto-tune based on the P2-32 setting.
//
//          P2-32 is a variable that stands for a collection of HDTUNE-parameters.
//          For further details please refer to the Lexium auto-tune documentation,
//          e.g. "Proposal for Autotune HMI user entry - v1.8.doc".
//
// Author: APH
// Algorithm:
// Revisions:
// Udi 13-July-2014: internaly start and stop HDTUNEREFEN P9-25  (internal profile generator)
// Udi 17-July-2014: Set P9-32 and P9-19, and few more.
//******************************************************************************

int SalSelectAndStartLexiumAutoTune(long long lparam, int drive)
{
   int s16_return_value = LEXIUM_AUTOTUNE_ACTIVATION_ERR;                     // Initialize to a value unequal "success"
   unsigned int u16_mode = 0;
   int s16_need_restore_upon_activation_fault = 0;
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   u16_mode = (unsigned int)lparam;

   //Allways allow stop AT
   if (u16_mode == 0)
     return StopLexiumAT(drive);

   // Check if currently a internal profile process or a jog-move is running. This code
   // has been introduced due to "BYang00000752" & "BYang00000753", in which Juergen
   // Fiess starts several kinds of motion with his commissioning tool without aborting
   // a different currently pending motion.
   if( (BGVAR(u16_Internal_Profile_State) != INTERNAL_PROFILE_IDLE) ||
       (BGVAR(u16_Lexium_Jog_Move_State)  != LEX_JOG_MOVE_IDLE) )
   {
      return OTHER_PROCEDURE_RUNNING;
   }

   // Clear the variable that holds a specific error code
   BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_NO_ERROR;

   BGVAR(u16_Auto_Adaptive_Tuning_Triggered) = 0; // This variable indicates with 1, that an auto-adaptive tuning has been triggered.
                                                  // Adaptive mode not yet implemented, left here for future reference

   // If someone wants to activate an auto-tuning process (lparam != 0) AND
   // The auto-tuning is currently running
   if((lparam != 0) && (BGVAR(u16_Lexium_Autotune_State) != LEX_ATUNE_STATE_IDLE))
   {
      return AT_ACTIVE;
   }

  //Validate input
   switch(u16_mode)
   {
      case (0): return StopLexiumAT(drive);
      case (1):
      case (2):
      case (3):
      case (5):
      case (6):
      case (7):
      case (52):
      case (53):
      case (55):
      case (56): break;

      default: return VALUE_IS_NOT_ALLOWED;
   }




   BGVAR(u16_P2_32_Lex_Auto_Tune_Mode) = u16_mode;



   if ((BGVAR(u16_Hold) == 1))
   {
      BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_HOLD_ACTIVE;
      return HOLD_MODE;
      //Stop will be displayed by other code after 20 sec
   }

   if( RowInEasyTuningEntries() < 0)
   {
      BGVAR(u16_Lex_AT_Err) = LEX_AT_ERR_UNKNOWN_MOTOR;
      return LEXIUM_AUTOTUNE_ACTIVATION_ERR;
   }

   if(u16_mode != 7) // 7 is for debug - do not set any PParam
   {
      s16_return_value = SetFewAutoTuneDefaults(drive);
      if(s16_return_value != SAL_SUCCESS)
         return s16_return_value;
   }

   //Save Enable and Opmode setting, if internal reference command.
   switch(BGVAR(u16_P2_32_Lex_Auto_Tune_Mode))
   {
      case (1):
      case (2):
      case (3):
      case (52):
      case (53):
                 LexiumPrepareOrRestoreAutoTuneSettings(drive, 0);
                 s16_need_restore_upon_activation_fault = 1;
                 break;
      default:   break;
   }


    s16_return_value = SetPPAramByTable(AT_Modes_Pparams_Table, u16_mode, drive);

   // If all of the previous sequence of commands were successful triggered (successfull setup of parameters) AND
   // No abort has been triggered by the user
   if(s16_return_value == SAL_SUCCESS)
   {
      //Prevent Umas upload of a scope file which may be created during AT. (No effect if no recording)
      u16_MB_Read_Scope_Done = 1;

      // Start the Lexium auto tuning process
      BGVAR(u16_Lexium_Autotune_State) = LEX_ATUNE_STATE_START;
      return SAL_SUCCESS;
   }
   //else report failure

   // If an easy or comfort tuning has been triggered that changed the internal OPMODE setting
   if(s16_need_restore_upon_activation_fault == 1 )
   {
      // Restore the original settings which have been changed at the beginning of this function
      // in case that the function was not able to successfully start the easy/comfort auto-tuning process.
      LexiumPrepareOrRestoreAutoTuneSettings(drive, 1);
   }

   s16_return_value = LEXIUM_AUTOTUNE_ACTIVATION_ERR; // Set Lexium auto-tune specific error-code


   return s16_return_value;
}


int RecPSDCommand(int drive)
{
   int s16_temp,s16_temp2;
   long code1,code2,code3;
   if ((s16_Record_Flags & DATA_AVAILABLE_MASK) == 0) return NOT_AVAILABLE;
   if ((BGVAR(s16_FFT_State) > 0) && (BGVAR(s16_FFT_State) < FFT_STATE_DONE)) return FFT_ACTIVE;
   if ((s64_Execution_Parameter[0] < 1) || (s64_Execution_Parameter[0] > 2)) return (VALUE_OUT_OF_RANGE);


   // allocate room for FFT varaible
   s16_temp = s16_Record_Channel_Number;
   ConvertMnemonic2Code("RECPSD", &code1, &code2, &code3, 0);
   s16_temp2 = SearchMnemonic(code1, code2, code3)-1;
   if (!AllocateRecordChannel(drive, (long long)(s16_temp2), &s16_temp)) return NOT_AVAILABLE ; // no free rec channels
   s32_Rec_Channel[s16_Recorded_Variables_Number] = (long)s16_temp2;
   s16_Record_Channel_Number+=2;
   ++s16_Recorded_Variables_Number;
   BGVAR(s16_FFT_State) = FFT_STATE_INIT;
   if (s64_Execution_Parameter[0] == 2)
      BGVAR(s16_FFT_Src_Words) = 4;
   else
      BGVAR(s16_FFT_Src_Words) = 1;
   return (SAL_SUCCESS);
}

int RecPsdStCommand(int drive)
{
 //  // AXIS_OFF;
   float f_temp;
   int s16_bin_hz;
   if (u8_Output_Buffer_Free_Space < 80) return (SAL_NOT_FINISHED);
   if  (BGVAR(s16_FFT_State) == FFT_STATE_IDLE) PrintString("Idle",0);
   else if (BGVAR(s16_FFT_State) == FFT_STATE_DONE)
   {
      s16_bin_hz = (int)((float)BGVAR(s16_Psd_Max_Bin) / ((float)s32_Gap_Value*31.25e-6*(float)s16_Recording_Length) + 0.5);

      PrintString("Done:",0);
      // print avg , max bin and bin number (=assiciated freq)
      PrintString("AVG = ",0);
      PrintSignedLongLong(BGVAR(s64_Psd_Avg));
      PrintString(" MaxBinVal = ",0);
      PrintSignedLong(BGVAR(s32_Psd_Max_Val));
      PrintString(" @Frq = ",0);
      PrintSignedInteger(s16_bin_hz);
      PrintString(" Hz",0);

      if (BGVAR(s16_Psd_Max2_Bin) != 0)
      {
         s16_bin_hz = (int)((float)BGVAR(s16_Psd_Max2_Bin) / ((float)s32_Gap_Value*31.25e-6*(float)s16_Recording_Length) + 0.5);
         PrintString(" Max2BinVal = ",0);
         PrintSignedLong(BGVAR(s32_Psd_Max2_Val));
         PrintString(" @Frq = ",0);
         PrintSignedInteger(s16_bin_hz);
         PrintString(" Hz",0);
      }
      if (BGVAR(s16_FFT_Src_Words) == 4)
         PrintString(" 64Bit",0);
      else
         PrintString(" 16Bit",0);
      PrintCrLf();
   }
   else
   {
      PrintString("OnGoing : ",0);
      f_temp = 200 * (float)BGVAR(s16_Psd_Index) / (float)(s16_Recording_Length);
      PrintSignedInteger((int)f_temp);
      PrintString(" %",0);
   }
    PrintCrLf();
   ++drive;
   return (SAL_SUCCESS);
}


void PSDHandler(int drive)
{
   static int s16_fft_index;
   static float f_sum_avg,f_sum_real,f_sum_imag;

   float f_w,f_temp;
   int s16_temp,s16_temp2,s16_peak_bin,s16_last_peak_bin,s16_10hz_bin;
   long s32_temp,s32_temp_prev,s32_peak,s32_last_peak,s32_min_valid;

   switch (BGVAR(s16_FFT_State))
   {
      case FFT_STATE_IDLE:
      break;

      case FFT_STATE_INIT:
         BGVAR(s16_Psd_Index) = 0;
         BGVAR(s32_Psd_Max_Val) = 0;
         BGVAR(s16_Psd_Max_Bin) = 0;
         BGVAR(s64_Psd_Avg) = 0;
         BGVAR(s16_FFT_State) = FFT_STATE_DC_CANCEL;
      break;

      case FFT_STATE_DC_CANCEL: // find avg to cancel DC and scale to 16 bits
         f_sum_avg = 0;
         for (s16_temp2 = 0;s16_temp2<s16_Recording_Length;++s16_temp2)
         {
            s16_temp = s16_temp2+u16_Record_Start;
            if (s16_temp >= u16_Ram_Rec_Length_Avaliable) s16_temp -=u16_Ram_Rec_Length_Avaliable;
            if (BGVAR(s16_FFT_Src_Words) == 1)  // icmd
               f_temp = (float)s16_Debug_Ram[0][s16_temp];
            else // pe - shr by 10 to avoid numerical ov. also info at very low bits is not relevent
            // +4 shr
                f_temp = 0.0009765625 * (float)(((((long long)(s16_Debug_Ram[3][s16_temp]) << 16) | (long long)(s16_Debug_Ram[2][s16_temp]) & 0x0FFFFLL) << 16 | (long long)(s16_Debug_Ram[1][s16_temp])& 0x0FFFFLL) << 16 | (long long)(s16_Debug_Ram[0][s16_temp])& 0x0FFFFLL) - f_sum_avg;
            f_sum_avg = f_sum_avg + f_temp;
         }
         f_sum_avg = f_sum_avg/(float)s16_Recording_Length;
         BGVAR(s16_FFT_State) = FFT_STATE_CALC_PSD;
      break;

      case FFT_STATE_CALC_PSD:
         // calc PSD
         s16_temp = s16_Recording_Length >> 1; // = N/2
         if (BGVAR(s16_Psd_Index) >= s16_temp)
            BGVAR(s16_FFT_State) = FFT_STATE_CALC_PSD_ZERO_PAD;
         else
            BGVAR(s16_FFT_State) = FFT_STATE_CALC_PSD_POINT;

         s16_fft_index = 0;
         f_sum_real = f_sum_imag = 0;
      break;

      case FFT_STATE_CALC_PSD_POINT:
         f_w = -6.2832*(float)BGVAR(s16_Psd_Index)/(float)s16_Recording_Length; // 2*pi = 6.2832
         s16_temp2 = 0;
         for (s16_temp2 = 0;((s16_temp2 < 300) && (s16_fft_index<s16_Recording_Length));++s16_temp2)
         {
            s16_temp = s16_fft_index+u16_Record_Start;
                if (s16_temp >= u16_Ram_Rec_Length_Avaliable) s16_temp -=u16_Ram_Rec_Length_Avaliable;
            if (BGVAR(s16_FFT_Src_Words) == 1)  // icmd
                f_temp = (float)s16_Debug_Ram[0][s16_temp] - f_sum_avg;
            else // pe - shr by 10 to avoid numerical ov. also info at very low bits is not relevent
                f_temp = 0.0009765625 * (float)(((((long long)(s16_Debug_Ram[3][s16_temp]) << 16) | (long long)(s16_Debug_Ram[2][s16_temp]) & 0x0FFFFLL) << 16 | (long long)(s16_Debug_Ram[1][s16_temp])& 0x0FFFFLL) << 16 | (long long)(s16_Debug_Ram[0][s16_temp])& 0x0FFFFLL) - f_sum_avg;

            f_temp = f_temp - f_sum_avg;
            f_sum_real = f_sum_real + f_temp*cos(f_w*s16_fft_index);
            f_sum_imag = f_sum_imag + f_temp*sin(f_w*s16_fft_index);
            ++s16_fft_index;
         }

         if (s16_fft_index == s16_Recording_Length)
         {
            s16_temp = BGVAR(s16_Psd_Index) + u16_Record_Start;
            if (s16_temp >= u16_Ram_Rec_Length_Avaliable) s16_temp -=u16_Ram_Rec_Length_Avaliable;

            f_temp = sqrt(f_sum_real*f_sum_real+f_sum_imag*f_sum_imag)/(float)s16_Recording_Length;
            if (BGVAR(s16_Psd_Index) != 0) f_temp =2*f_temp;
            f_temp = f_temp*f_temp;
            s32_temp = (long)f_temp;
            s16_Debug_Ram[s16_Record_Channel_Number-2][s16_temp] = (int)(s32_temp); // place of the recpsd data
            s16_Debug_Ram[s16_Record_Channel_Number-1][s16_temp] = (int)(s32_temp>>16); // place of the recpsd data
            if (s32_temp > BGVAR(s32_Psd_Max_Val))
            {
               BGVAR(s32_Psd_Max_Val) = s32_temp;
               BGVAR(s16_Psd_Max_Bin) = BGVAR(s16_Psd_Index);
            }
            BGVAR(s64_Psd_Avg) += (long long)s32_temp;

            ++BGVAR(s16_Psd_Index);
            BGVAR(s16_FFT_State) = FFT_STATE_CALC_PSD;
         }
      break;

      case FFT_STATE_CALC_PSD_ZERO_PAD:
         for (s16_temp2 = 0;((BGVAR(s16_Psd_Index) < s16_Recording_Length) && (s16_temp2 < 1000));++s16_temp2)
         {
            s16_temp = BGVAR(s16_Psd_Index) + u16_Record_Start;
            if (s16_temp >= u16_Ram_Rec_Length_Avaliable) s16_temp -=u16_Ram_Rec_Length_Avaliable;
            s16_Debug_Ram[s16_Record_Channel_Number-2][s16_temp] = 0;// place of the recpsd data
            s16_Debug_Ram[s16_Record_Channel_Number-1][s16_temp] = 0;
            ++BGVAR(s16_Psd_Index);
         }
         if (BGVAR(s16_Psd_Index) == s16_Recording_Length)
         {
            BGVAR(s64_Psd_Avg) = 2*BGVAR(s64_Psd_Avg)/s16_Recording_Length;
            BGVAR(s16_FFT_State) = FFT_DC_TAIL_CLEANUP;
         }
      break;

      case FFT_DC_TAIL_CLEANUP:
         // when a tail exists , the peaks will be decreasing. tail ends when peaks are not descreasing
         // this is needed only when the low dc frequencies are interesting - like in AV

         // if max bin is below s16_last_peak_bin than max is not valid - search after the first valid bin
         s32_min_valid = (long)(2*BGVAR(s64_Psd_Avg));
         s32_temp_prev = 0;
         s32_last_peak = 0x7FFFFFFF;
         s16_last_peak_bin = 0;

         s16_10hz_bin = (int)(0.5+10.0*((float)s32_Gap_Value*31.25e-6*(float)s16_Recording_Length));

         for (s16_temp2 = s16_10hz_bin;s16_temp2<s16_Recording_Length;++s16_temp2)
         {
            s16_temp = s16_temp2+u16_Record_Start;
            if (s16_temp >= u16_Ram_Rec_Length_Avaliable) s16_temp -=u16_Ram_Rec_Length_Avaliable;

            s32_temp = ((long)(s16_Debug_Ram[s16_Record_Channel_Number-1][s16_temp]) << 16) | ((long)(s16_Debug_Ram[s16_Record_Channel_Number-2][s16_temp]) & 0x0FFFFL);

            s32_temp = (long)((long long)s32_temp - s32_min_valid);
            if (s32_temp < 0) s32_temp = 0;

            if (s32_temp >= s32_temp_prev) // data increasing - look for peak
            {
               s32_peak = s32_temp;
               s16_peak_bin = s16_temp2;
            }
            else // data is decending => peak was found
            {
               if (s32_peak <= s32_last_peak)
               {
                  s32_last_peak =  s32_peak;
                  s16_last_peak_bin = s16_peak_bin;
               }
               else if (s32_peak > (s32_last_peak + s32_min_valid) ) break;
            }
            s32_temp_prev = s32_temp;
         }

         if (BGVAR(s16_Psd_Max_Bin) > s16_last_peak_bin)
            BGVAR(s16_FFT_State) = FFT_SECOND_BEST;
         else
         {
            BGVAR(s16_FFT_State) = FFT_DC_TAIL_CLEANUP_UPDATE_MAX;
            BGVAR(s16_Psd_Max_Bin) = s16_last_peak_bin;
         }
         break;

      case FFT_DC_TAIL_CLEANUP_UPDATE_MAX:
         s16_peak_bin = BGVAR(s16_Psd_Max_Bin);
         BGVAR(s32_Psd_Max_Val) = 0;
         s32_min_valid = (long)(2*BGVAR(s64_Psd_Avg));
         for (s16_temp2 = s16_peak_bin;s16_temp2<s16_Recording_Length;++s16_temp2)
         {
            s16_temp = s16_temp2+u16_Record_Start;
            if (s16_temp >= u16_Ram_Rec_Length_Avaliable) s16_temp -=u16_Ram_Rec_Length_Avaliable;

            s32_temp = ((long)(s16_Debug_Ram[s16_Record_Channel_Number-1][s16_temp]) << 16) | ((long)(s16_Debug_Ram[s16_Record_Channel_Number-2][s16_temp]) & 0x0FFFFL);

            if (s32_temp > BGVAR(s32_Psd_Max_Val) && (s32_temp > s32_min_valid) )
            {
               BGVAR(s32_Psd_Max_Val) = s32_temp;
               BGVAR(s16_Psd_Max_Bin) = s16_temp2;
            }
         }
         BGVAR(s16_FFT_State) = FFT_SECOND_BEST;

         break;

      case FFT_SECOND_BEST:
         s16_10hz_bin = (int)(0.5+10.0*((float)s32_Gap_Value*31.25e-6*(float)s16_Recording_Length));
         s32_min_valid = (long)(2*BGVAR(s64_Psd_Avg));
         s32_temp_prev = 0;
         s32_last_peak = 0x7FFFFFFF;
         s16_last_peak_bin = 0;
         // cleanup second best
         for (s16_temp2 = BGVAR(s16_Psd_Max_Bin)+s16_10hz_bin;s16_temp2<s16_Recording_Length;++s16_temp2)
         {
            s16_temp = s16_temp2+u16_Record_Start;
            if (s16_temp >= u16_Ram_Rec_Length_Avaliable) s16_temp -=u16_Ram_Rec_Length_Avaliable;

            s32_temp = ((long)(s16_Debug_Ram[s16_Record_Channel_Number-1][s16_temp]) << 16) | ((long)(s16_Debug_Ram[s16_Record_Channel_Number-2][s16_temp]) & 0x0FFFFL);

            s32_temp = (long)((long long)s32_temp - s32_min_valid);
            if (s32_temp < 0) s32_temp = 0;

            if (s32_temp >= s32_temp_prev) // data increasing - look for peak
            {
               s32_peak = s32_temp;
               s16_peak_bin = s16_temp2;
            }
            else // data is decending => peak was found
            {
               if (s32_peak <= s32_last_peak)
               {
                  s32_last_peak =  s32_peak;
                  s16_last_peak_bin = s16_peak_bin;
               }
               else if (s32_peak > (s32_last_peak + s32_min_valid) ) break;
            }
            s32_temp_prev = s32_temp;
         }

         // find second best
         BGVAR(s32_Psd_Max2_Val) = 0;
         BGVAR(s16_Psd_Max2_Bin) = 0;
         for (s16_temp2 = s16_last_peak_bin;s16_temp2<s16_Recording_Length;++s16_temp2)
         {
            s16_temp = s16_temp2+u16_Record_Start;
            if (s16_temp >= u16_Ram_Rec_Length_Avaliable) s16_temp -=u16_Ram_Rec_Length_Avaliable;

            s32_temp = ((long)(s16_Debug_Ram[s16_Record_Channel_Number-1][s16_temp]) << 16) | ((long)(s16_Debug_Ram[s16_Record_Channel_Number-2][s16_temp]) & 0x0FFFFL);
            if ((s32_temp > BGVAR(s32_Psd_Max2_Val)) && (s32_temp > s32_min_valid) )
            {
               BGVAR(s32_Psd_Max2_Val) = s32_temp;
               BGVAR(s16_Psd_Max2_Bin) = s16_temp2;
            }
         }
         BGVAR(s16_FFT_State) = FFT_STATE_DONE;

         break;
      case FFT_STATE_DONE:
      break;
   }
   ++drive;
}

int debug_state;


int LMJREstimationSimplex(int drive,int reset)
{  // 32767 - finished 0 - ongoing  0< fault
   static int s16_lmjr_simplex_state;
   static long s32_timer;
   int s16_temp,s16_ret_val = 0;
   long s32_ptpvcmd,code1,code2,code3,s32_temp;
   unsigned long rectrig,u32_pitch;
   // AXIS_OFF;
   
   if (reset == 1) 
   {
      s16_lmjr_simplex_state = LMJR_SIMPLEX_IDLE;
      s16_Lmjr_Simplex_Reset = 0;
      BGVAR(u16_LMJREstimationSimplex_Retries_Counter) = 0; //will be incremented to 0 on the first try.
      s16_ret_val = 32767;
   }
   else if (reset == -1)  
   {
      s16_Lmjr_Simplex_Reset = 0;
      s16_lmjr_simplex_state = LMJR_SIMPLEX_RECORD_SETUP;
   }
   else if (reset == -32767)
   {
	   s16_Lmjr_Simplex_Reset = 0;
	   s16_lmjr_simplex_state = LMJR_SIMPLEX_ABORTED;
   }

   switch (s16_lmjr_simplex_state)
   {
      case LMJR_SIMPLEX_IDLE:
         s16_ret_val = 32767;
         break;

      case LMJR_SIMPLEX_RECORD_SETUP:
         STORE_EXECUTION_PARAMS_0_15
         ClrRecord();
         ConvertMnemonic2Code("VINT", &code1, &code2, &code3, 0);
         s64_Execution_Parameter[2] = SearchMnemonic(code1, code2, code3)-1;
         ConvertMnemonic2Code("ICMD", &code1, &code2, &code3, 0);
         s64_Execution_Parameter[3] = SearchMnemonic(code1, code2, code3)-1;
         ConvertMnemonic2Code("VCMD", &code1, &code2, &code3, 0);
         s64_Execution_Parameter[4] = SearchMnemonic(code1, code2, code3)-1;
         s64_Execution_Parameter[5] = 0;
         SalRecordCommand(drive,8LL, 1900LL,&s64_Execution_Parameter[2]);
         RESTORE_EXECUTION_PARAMS_0_15
         s32_timer = Cntr_1mS;
         VAR(AX0_s16_Cycle_Bits) &= ~CYCLE_NEW_CYCLE_DATA_READY;
         s16_lmjr_simplex_state = LMJR_SIMPLEX_ARM_REC_TRIG;
         s16_ret_val = 1;
         break;   
      
      case LMJR_SIMPLEX_ARM_REC_TRIG:
         if (!PassedTimeMS(500L, s32_timer)) break;
         
         do {
            s16_temp = Cntr_3125;
            s32_ptpvcmd = LVAR(AX0_s32_Pos_Vcmd);
         } while (s16_temp != Cntr_3125);

         s32_ptpvcmd = labs(s32_ptpvcmd);
         if (s32_ptpvcmd > 0x10) break;
         // trigger on abs value
         rectrig = (((unsigned long)(&AX0_s32_Pos_Vcmd) & 0xFFFFFF) | REC_ADD_32) | REC_ADD_SIGN;
         // set trigger to abs(1% of MSPEED) in UP direction
         // Convert 30 rpm to internal unit:
         // 30 * 2^32*125u/60 = 30 * 8,947.8485333 = 268,435.456
         // Value is multiplied by 1000 since here is a division by 1000 inside the rectrig function
         //
         u32_pitch = 1000L;        
         if(BGVAR(u16_MotorType) == LINEAR_MOTOR) u32_pitch = BGVAR(u32_Mpitch);
         // 10 = 1% * 1000 = 0.01 * 1000
         s32_temp = (long)((float)BGVAR(u32_Mspeed)*10);        
         s32_temp = (long)((float)s32_temp /((float)u32_pitch/1000.0));
                
         // verify here is no MTS between PTPVCMD = 0 and RECTRIG arming
         // This may cause to missing trigger point
         if(s16_temp != Cntr_3125)
            {
                s16_lmjr_simplex_state = LMJR_SIMPLEX_RECORD_SETUP;
                break;
            }
         SalRecTrigCommand(drive,rectrig,(long long)s32_temp,0LL,ABS_MORE_THAN_REC_TRIG); // trig on abs value
         BGVAR(u16_LMJREstimationSimplex_Retries_Counter)++;
         s16_lmjr_simplex_state = LMJR_SIMPLEX_REC_PROCCESS; 
         s16_ret_val = 2;
         break;
         
      case LMJR_SIMPLEX_REC_PROCCESS:
         s16_ret_val = 2;
         if ((s16_Record_Flags & DATA_AVAILABLE_MASK) == 0) break; // wait for recdone
         s16_ret_val = 3;

         // verify record can be used by simplex
         s16_temp = SimplexRecordValidation(); 
         if (s16_temp != 1) 
         {

             if (s16_temp == -1) s16_lmjr_simplex_state = LMJR_SIMPLEX_FAULT_DELTA_V;
             else s16_lmjr_simplex_state = LMJR_SIMPLEX_FAULT;
             // If failed, try again
             if(BGVAR(u16_LMJREstimationSimplex_Retries_Counter) <= 5)
                                s16_lmjr_simplex_state = LMJR_SIMPLEX_RECORD_SETUP;
             break;
         }

         // data is avaliable - start simplex algorithm
         SimplexLMJRHandler(drive,1); // reset
         s16_lmjr_simplex_state = LMJR_SIMPLEX_EXE; 
         break;

      case LMJR_SIMPLEX_EXE:   
         s16_ret_val = 4;
         if (SimplexLMJRHandler(drive,0) == 0) break;  
         s16_lmjr_simplex_state = LMJR_SIMPLEX_DONE;
         s16_Lmjr_Simplex_Reset = 32767;
         break;

      case LMJR_SIMPLEX_DONE:
         s16_ret_val = 32767;
         break;
         
      case LMJR_SIMPLEX_FAULT:
         s16_ret_val = -1;
         break;         

      case LMJR_SIMPLEX_FAULT_DELTA_V:
    	 s16_ret_val = -2;
         break;              

      case LMJR_SIMPLEX_ABORTED:
    	  s16_ret_val = -32767;
    	  break;


   }



   return (s16_ret_val);
}


unsigned int PseudoRandomBinaryNumber(int s16_reset)
{ // generates a 16 bit "random" number - based on matlab simulation yuval\hybrid\SimplexRND 
  // reset - any non zero value - should be called once. consecutive calls use 0
  // PSD is 16 bits - i.e. repreat itself after 65536 executions
  static unsigned int u16_seed;
  int s16_lsb;

  if (s16_reset) u16_seed = (unsigned int)s16_reset; 
   
  s16_lsb = u16_seed & 0x0001;
  u16_seed >>= 1;

  if (s16_lsb) u16_seed = u16_seed ^ 0xb400;

  return u16_seed;
}


#pragma CODE_SECTION(SimplexMotorMechanicalModel, "ramfunc_5");
int SimplexMotorMechanicalModel(int s16_icmd, float f_j_total,float f_fric_coeff, float f_t_ub,int s16_reset)
{
   // based on matlab simulation yuval\hybrid\SimplexMotorMechanicalModel
   // simulate simplce motor -> load module
   // j_total kg*m^2
   // B - fric coeff N*m/(rad/sec)
   // Tl - load torque N*m
   // s16_icmd - drive units . need to convert to amps
   // f_motor_speed rad/sec
   // Simplex_ts - depending on record gap of data_in. gap=8 => ts = 250e-6

   #define SIMPLEX_TS 250e-6

   static float f_motor_speed,f_speed_conversion_factor;
   float f_torque;
   unsigned long u32_mpitch;

   if (s16_reset) 
   {
      f_motor_speed = 0;
      
      // done here to speed up calc time
      u32_mpitch = 1000L;
      if (BGVAR(u16_MotorType) == LINEAR_MOTOR) u32_mpitch = BGVAR(u32_Mpitch);
      // f_motor_speed/2/pi*60/f_vlim_rpm*22750
      f_speed_conversion_factor = (float)BGVAR(s32_V_Lim_Design) * 0.00000011176 *(float)u32_mpitch; // vlim_rpm
      f_speed_conversion_factor = 217246.497320/f_speed_conversion_factor;
      // Get initial velocity V0 and convert to rad/s
      // Vel_Internal * s32_V_Lim_Design * 60/(2^32*125u) / 22750 * 2pi /60 = Vel[rad/s]
      f_motor_speed = (float)s16_Debug_Ram[0][u16_Record_Start]/f_speed_conversion_factor;
      return 0;
   }

   f_torque = 3.8147554742e-11*(float)BGVAR(s32_Drive_I_Peak)*(float)BGVAR(u32_Motor_Kt)*(float)s16_icmd - f_fric_coeff*f_motor_speed - f_t_ub;
   f_motor_speed = f_motor_speed + f_torque/f_j_total * SIMPLEX_TS;

   return (int)(f_motor_speed*f_speed_conversion_factor);
}


#pragma CODE_SECTION(SimplexCostCalc, "ramfunc_3");
long long SimplexCostCalc(float f_j_total,float f_fric_coeff, float f_t_ub)
{
   int s16_index;
   int s16_simulated_vact,s16_vact,s16_icmd,s16_data_index = u16_Record_Start+1; // Start with record+1 since we took first data sample for initialization
   long long s64_simplex_cost = 0;

   float f_mj = 1e-6*(float)BGVAR(s32_Motor_J);


   if (f_fric_coeff < 0) return -1;
   if (f_j_total < f_mj) return -2;


   SimplexMotorMechanicalModel(0,0,0,0,1); // assume initial V = 0 if this is not the case 

   for (s16_index = 0;s16_index < 1900;++s16_index)
   {
      if (s16_data_index >= u16_Ram_Rec_Length_Avaliable) s16_data_index -=u16_Ram_Rec_Length_Avaliable;
      
      s16_vact = s16_Debug_Ram[0][s16_data_index];
      s16_icmd = s16_Debug_Ram[1][s16_data_index];
     
      s16_simulated_vact = SimplexMotorMechanicalModel(s16_icmd,f_j_total,f_fric_coeff,f_t_ub,0);

      s16_Debug_Ram[2][s16_data_index] = s16_simulated_vact; // for debug only !!
           
      s64_simplex_cost = s64_simplex_cost + (long long)((long)(s16_simulated_vact-s16_vact)*(long)(s16_simulated_vact-s16_vact));

      ++s16_data_index;
   }

   return s64_simplex_cost;
}




int SimplexLMJRHandler(int drive, int reset)
{
   #define SIMPLEX_ITTR_MAX      3

   static long long s64_cost_reflection;
   static long long s64_cost_contraction;
   static float f_simplex_matrix[6][3];
   static float f_simplex_centroid[3];
   static float f_simplex_reflection[3];
   static float f_simplex_contraction[3];
   static int s16_best_cost_row;
   static int s16_worse_cost_row;
   static int s16_Simplex_LMJR_Handler_After_Centroid;
   static int s16_simplex_itteration;
   static long long s64_simplex_cost_vector[6];
   static int s16_restart_counter;
   static long long s64_cost_final;
   static long long s64_cost_before_restart;

   int s16_index_row,s16_index_col,ret_val = 0;
   long long s64_temp;
   float f_simplex_extention[3],f_stop_codition;

   REFERENCE_TO_DRIVE;

   if (reset)
   {
      s16_Simplex_LMJR_Handler_State = SIMPLEX_INIT;
      PseudoRandomBinaryNumber(1);
   }
      
   switch (s16_Simplex_LMJR_Handler_State)
   {
      case SIMPLEX_INIT:
         BGVAR(f_Simplex_Result_B) = BGVAR(f_Simplex_Result_Tub) = 0; 
         BGVAR(u16_Simplex_LMJR) = 0;

         s64_cost_final = 0x7FFFFFFFFFFFFFFF;
         s16_restart_counter = 0;
         s16_simplex_itteration = 0;
         s16_worse_cost_row = -1;
         s16_Simplex_LMJR_Handler_State = SIMPLEX_RANDOMIZE_MATRIX_1;
         s64_cost_before_restart = s64_cost_final;
         break;

      case SIMPLEX_RANDOMIZE_MATRIX_1:
         // calc initial simplex matrix and the associated cost
         for (s16_index_row = 0 ; s16_index_row<3 ; ++s16_index_row)
         { 
            f_simplex_matrix[s16_index_row][0] = 1e-6*(float)BGVAR(s32_Motor_J)*(1+0.001*(float)PseudoRandomBinaryNumber(0));  // mj  lmjr 0-65
            f_simplex_matrix[s16_index_row][1] = 0.000000152587890625*(float)PseudoRandomBinaryNumber(0);  
            f_simplex_matrix[s16_index_row][2] = 0.00000000152587890625*(float)PseudoRandomBinaryNumber(0);  
            
            s64_simplex_cost_vector[s16_index_row] = SimplexCostCalc(f_simplex_matrix[s16_index_row][0],f_simplex_matrix[s16_index_row][1], f_simplex_matrix[s16_index_row][2]);
         } 

         s16_Simplex_LMJR_Handler_State = SIMPLEX_RANDOMIZE_MATRIX_2;
         break;

      case SIMPLEX_RANDOMIZE_MATRIX_2:
         for (s16_index_row = 3 ; s16_index_row<6 ; ++s16_index_row)
         { 
            f_simplex_matrix[s16_index_row][0] = 1e-6*(float)BGVAR(s32_Motor_J)*(1+0.001*(float)PseudoRandomBinaryNumber(0));  // mj  lmjr 0-65
            f_simplex_matrix[s16_index_row][1] = 0.000000152587890625*(float)PseudoRandomBinaryNumber(0);
            f_simplex_matrix[s16_index_row][2] = 0.00000000152587890625*(float)PseudoRandomBinaryNumber(0);  
            
            s64_simplex_cost_vector[s16_index_row] = SimplexCostCalc(f_simplex_matrix[s16_index_row][0],f_simplex_matrix[s16_index_row][1], f_simplex_matrix[s16_index_row][2]);
         } 
         

            
         s16_Simplex_LMJR_Handler_State = SIMPLEX_CENTROID_REFLECTION_AND_CONTRACTION;
         s16_Simplex_LMJR_Handler_After_Centroid = SIMPLEX_LOOKING_FOR_GRADIENT;    
         break;

      case SIMPLEX_CENTROID_REFLECTION_AND_CONTRACTION:
         // calc centriod
         f_simplex_centroid[0] = f_simplex_centroid[1] = f_simplex_centroid[2] = 0;
         for (s16_index_row = 0 ; s16_index_row<6 ; ++s16_index_row)
         {
            f_simplex_centroid[0] += f_simplex_matrix[s16_index_row][0];
            f_simplex_centroid[1] += f_simplex_matrix[s16_index_row][1];
            f_simplex_centroid[2] += f_simplex_matrix[s16_index_row][2];
         }

         f_simplex_centroid[0] /= 6;
         f_simplex_centroid[1] /= 6;
         f_simplex_centroid[2] /= 6;
      
         // find worse cost 
         
         s16_worse_cost_row = 0;
         for (s16_index_row = 1 ; s16_index_row<6 ; ++s16_index_row) 
            if (s64_simplex_cost_vector[s16_index_row] > s64_simplex_cost_vector[s16_worse_cost_row]) s16_worse_cost_row = s16_index_row;
         
         // calc reflection and contraction points
         for (s16_index_col=0 ; s16_index_col<3 ; ++s16_index_col)
         {
            f_simplex_reflection[s16_index_col] = 2*f_simplex_centroid[s16_index_col] - f_simplex_matrix[s16_worse_cost_row][s16_index_col];
            f_simplex_contraction[s16_index_col] = 0.5*(f_simplex_centroid[s16_index_col] + f_simplex_matrix[s16_worse_cost_row][s16_index_col]);
         }
         s16_Simplex_LMJR_Handler_State = s16_Simplex_LMJR_Handler_After_Centroid;    


         if (s16_Simplex_LMJR_Handler_State == SIMPLEX_LOOKING_FOR_GRADIENT)
         {
            s16_best_cost_row = 0;
            for (s16_index_row = 1 ; s16_index_row<6 ; ++s16_index_row) 
               if (s64_simplex_cost_vector[s16_index_row] < s64_simplex_cost_vector[s16_best_cost_row]) s16_best_cost_row = s16_index_row;

            // if new result is better - capture it
            if (s64_simplex_cost_vector[s16_best_cost_row] < s64_cost_final)
            {
               s64_cost_final = s64_simplex_cost_vector[s16_best_cost_row];
               BGVAR(u16_Simplex_LMJR) = (unsigned int)1000*(1e6*f_simplex_matrix[s16_best_cost_row][0]/(float)BGVAR(s32_Motor_J) - 1);
               BGVAR(f_Simplex_Result_B)    = f_simplex_matrix[s16_best_cost_row][1];
               BGVAR(f_Simplex_Result_Tub)  = f_simplex_matrix[s16_best_cost_row][2];    
            }

            // check for matrix convergence
            f_stop_codition = (float)s64_simplex_cost_vector[s16_best_cost_row]/(float)s64_simplex_cost_vector[s16_worse_cost_row];
            

            if ((f_stop_codition < 0.9999) && (s16_simplex_itteration < 40)) break;

            // code here => matrix converged or code "timeout" triggered

            if (s64_simplex_cost_vector[s16_best_cost_row] == s64_cost_final)
            { // repalce all matrix elements but the best row - and restart
               for (s16_index_row = 0 ; s16_index_row<6 ; ++s16_index_row)
               { 
                  if  (s16_best_cost_row != s16_index_row)
                  {
               
                     f_simplex_matrix[s16_index_row][0] = 1e-6*(float)BGVAR(s32_Motor_J)*(1+0.001*(float)PseudoRandomBinaryNumber(0));  // mj  lmjr 0-65
                     f_simplex_matrix[s16_index_row][1] = 0.000000152587890625*(float)PseudoRandomBinaryNumber(0);
                     f_simplex_matrix[s16_index_row][2] = 0.00000000152587890625*(float)PseudoRandomBinaryNumber(0);

                     s64_simplex_cost_vector[s16_index_row] = SimplexCostCalc(f_simplex_matrix[s16_index_row][0],f_simplex_matrix[s16_index_row][1], f_simplex_matrix[s16_index_row][2]);
                  }
               }   
            }
            else
            {
               s16_Simplex_LMJR_Handler_State = SIMPLEX_RANDOMIZE_MATRIX_1;  
            }
            
            s16_simplex_itteration = 0;
            ++s16_restart_counter;
            f_stop_codition = s64_cost_final/s64_cost_before_restart;
            s64_cost_before_restart = s64_cost_final;
            if ((f_stop_codition > 0.95) || (s16_restart_counter > SIMPLEX_ITTR_MAX))
            {  // for debug plotting only
               SimplexCostCalc(f_simplex_matrix[s16_best_cost_row][0],f_simplex_matrix[s16_best_cost_row][1], f_simplex_matrix[s16_best_cost_row][2]);
               ret_val  = 1;   
            }
         }
         break;

      case SIMPLEX_LOOKING_FOR_GRADIENT:
         // calc reflection cost
         s64_cost_reflection = SimplexCostCalc(f_simplex_reflection[0],f_simplex_reflection[1], f_simplex_reflection[2]);
         if (s64_cost_reflection < 0) s64_cost_reflection = 10*s64_simplex_cost_vector[s16_worse_cost_row];

         if (s64_cost_reflection < s64_simplex_cost_vector[s16_worse_cost_row])
         {
            s64_simplex_cost_vector[s16_worse_cost_row] = s64_cost_reflection;
            f_simplex_matrix[s16_worse_cost_row][0] = f_simplex_reflection[0];
            f_simplex_matrix[s16_worse_cost_row][1] = f_simplex_reflection[1];
            f_simplex_matrix[s16_worse_cost_row][2] = f_simplex_reflection[2];
            s16_Simplex_LMJR_Handler_After_Centroid = SIMPLEX_EXTENTION;
            s16_Simplex_LMJR_Handler_State = SIMPLEX_CENTROID_REFLECTION_AND_CONTRACTION;  
            break;
         }

         ++s16_simplex_itteration;

         s16_Simplex_LMJR_Handler_State = SIMPLEX_CENTROID_REFLECTION_AND_CONTRACTION;

         // calc cost contraction
         s64_cost_contraction = SimplexCostCalc(f_simplex_contraction[0],f_simplex_contraction[1], f_simplex_contraction[2]);
         if (s64_cost_contraction < 0) s64_cost_contraction = 10*s64_simplex_cost_vector[s16_worse_cost_row];
         if (s64_cost_contraction < s64_simplex_cost_vector[s16_worse_cost_row])
         {
            s64_simplex_cost_vector[s16_worse_cost_row] = s64_cost_contraction;
            f_simplex_matrix[s16_worse_cost_row][0] = f_simplex_contraction[0];
            f_simplex_matrix[s16_worse_cost_row][1] = f_simplex_contraction[1];
            f_simplex_matrix[s16_worse_cost_row][2] = f_simplex_contraction[2];
            break;
         }

         // need a new point
         s16_best_cost_row = 0;
         for (s16_index_row = 1 ; s16_index_row<6 ; ++s16_index_row) 
            if (s64_simplex_cost_vector[s16_index_row] < s64_simplex_cost_vector[s16_best_cost_row]) s16_best_cost_row = s16_index_row;

         for (s16_index_col=0 ; s16_index_col<3 ; ++s16_index_col)
            f_simplex_matrix[s16_worse_cost_row][s16_index_col] = 0.5*(f_simplex_matrix[s16_worse_cost_row][s16_index_col] + f_simplex_matrix[s16_best_cost_row][s16_index_col]);
         s64_temp = SimplexCostCalc(f_simplex_matrix[s16_worse_cost_row][0],f_simplex_matrix[s16_worse_cost_row][1], f_simplex_matrix[s16_worse_cost_row][2]);
         if (s64_temp < 0) s64_temp = 10*s64_simplex_cost_vector[s16_worse_cost_row];
         if (s64_temp < s64_simplex_cost_vector[s16_worse_cost_row]) s64_simplex_cost_vector[s16_worse_cost_row] = s64_temp;
         break;

      case SIMPLEX_EXTENTION:
         for (s16_index_col=0 ; s16_index_col<3 ; ++s16_index_col)
            f_simplex_extention[s16_index_col] = -f_simplex_centroid[s16_index_col] + 2*f_simplex_reflection[s16_index_col];

         s64_temp = SimplexCostCalc(f_simplex_extention[0],f_simplex_extention[1], f_simplex_extention[2]);
         if (s64_temp < 0) s64_temp = 10*s64_simplex_cost_vector[s16_worse_cost_row];

         if (s64_temp < s64_simplex_cost_vector[s16_worse_cost_row]) 
         {
            s64_simplex_cost_vector[s16_worse_cost_row] = s64_temp;
            f_simplex_matrix[s16_worse_cost_row][0] = f_simplex_extention[0];
            f_simplex_matrix[s16_worse_cost_row][1] = f_simplex_extention[1];
            f_simplex_matrix[s16_worse_cost_row][2] = f_simplex_extention[2];
            s16_Simplex_LMJR_Handler_State = SIMPLEX_CENTROID_REFLECTION_AND_CONTRACTION;  
            break;
         }
         ++s16_simplex_itteration;
         s16_Simplex_LMJR_Handler_State = SIMPLEX_CENTROID_REFLECTION_AND_CONTRACTION;
         s16_Simplex_LMJR_Handler_After_Centroid = SIMPLEX_LOOKING_FOR_GRADIENT;  
         break;
   }

   

   return (ret_val);
}


int SalLmjrEstCommand(int drive)
{

   if (BGVAR(s16_Number_Of_Parameters) !=  1)
		   return SYNTAX_ERROR;
               
   if ((BGVAR(s64_Execution_Parameter[0]) > 1) || (BGVAR(s64_Execution_Parameter[0]) < 0))
   		   return VALUE_OUT_OF_RANGE;

   if (BGVAR(s64_Execution_Parameter[0]) == 0 )
   {

	   if ((BGVAR(s16_LMJREstSimplexState) > LMJR_SIMPLEX_ST_ON_GOING) && (BGVAR(s16_LMJREstSimplexState) < LMJR_SIMPLEX_ST_ANALYZING))
	   	   RecordOffCommand();

	   BGVAR(s16_Lmjr_Simplex_Reset) = -32767;
	   LMJREstimationSimplex(drive,BGVAR(s16_Lmjr_Simplex_Reset));
	   return (SAL_SUCCESS);
   }

   if (BGVAR(s64_Execution_Parameter[0]) == 1)
   {
	   if (RECORDING_PENDING) return (REC_ACTIVE);

	   BGVAR(s16_Lmjr_Simplex_Reset) = 1;
	   s16_LMJREstSimplexState = LMJREstimationSimplex(drive,BGVAR(s16_Lmjr_Simplex_Reset));
	   BGVAR(s16_Lmjr_Simplex_Reset) = -1;
	   return (SAL_SUCCESS);
   }

   return (SAL_SUCCESS);
}


void PrintLMJRData(float f_num)
{
   float f_temp;
   int s16_temp;
   long s32_temp;

   s16_temp = 0;
   f_temp = f_num;
   if (f_temp < 0.0) f_temp = -f_temp;

   if (f_temp != 0.0)
   {
      if (f_temp < 0.001)
      {
         while (f_temp < 1.0) {s16_temp--; f_temp *= 10.0;}
      }
      else if (f_temp > (float)(0x7FFFFFFF / 1000))
      {
         while (f_temp > 1.0) {s16_temp++; f_temp /= 10.0;}
      }
   }
   if (f_num < 0.0) f_temp = -f_temp;

   s32_temp = (long)(((float)1000 * f_temp) + 0.5);
   PrintString(DecimalPoint32ToAscii(s32_temp),0);

   if (s16_temp)
   {
      PrintChar('E');
      PrintSignedInt16(s16_temp);
   }
}

int SalLmjrEstStCommand(int drive)
{
   REFERENCE_TO_DRIVE;

   if (s16_LMJREstSimplexState == LMJR_SIMPLEX_ST_ON_GOING) PrintString("On Going",0);
   else if (s16_LMJREstSimplexState == LMJR_SIMPLEX_ST_WAIT_FOR_ARM)  PrintString("Waiting For Arm",0);
   else if (s16_LMJREstSimplexState == LMJR_SIMPLEX_ST_WAIT_FOR_MOT)  PrintString("Waiting For Motion",0);
   else if (s16_LMJREstSimplexState == LMJR_SIMPLEX_ST_STARTING)  PrintString("Starting...",0);
   else if (s16_LMJREstSimplexState == LMJR_SIMPLEX_ST_ANALYZING)  PrintString("Analyzing...",0);
   else if (s16_LMJREstSimplexState == LMJR_SIMPLEX_ST_UNKNOWN_FAULT) PrintString("Unknown fault",0);
   else if (s16_LMJREstSimplexState == LMJR_SIMPLEX_ST_FAULT_DELTA_V) PrintString("Insufficient speed change",0);
   else if (s16_LMJREstSimplexState == LMJR_SIMPLEX_ST_ABORTED) PrintString("LMJR estimation aborted",0);
   else if (s16_LMJREstSimplexState < LMJR_SIMPLEX_ST_ON_GOING) PrintString("Unknown state",0);

   else
   {
      PrintString("LMJR=",0); PrintString(DecimalPoint32ToAscii((long)BGVAR(u16_Simplex_LMJR)),0); 
      PrintString(" , FricCoeff=",0); PrintLMJRData(BGVAR(f_Simplex_Result_B));
      PrintString(" , Tub=",0); PrintLMJRData(BGVAR(f_Simplex_Result_Tub));
   }
   PrintCrLf();
   return (SAL_SUCCESS); 
}

int SimplexRecordValidation(void)
{
       // to validate simplex - first test if the record has enough power compared to the rated power 
       // and than check if the change in veloicty is enough to identify acceleration phase
       //
       // Motor power is T[Nm]*w[rad/s] = I[A]*Mkt[Nm/A]*w[rad/s], the function aim to calculate
       // calculates:
       // Pact[W] = sum(abs(Icmd[A]*Mkt[Nm/A]*w[rad/s])*Ts)/(Ts*u16_number_of_samples) = sum(abs(Icmd[A]*Mkt[Nm/A]*w[rad/s])/u16_number_of_samples)
       // Pnom[W] is givven in f_Motor_Rated_Power
       // Pact/Pnom  = sum(abs(Icmd[A]*Mkt[Nm/A]*w[rad/s])/u16_number_of_samples) / (Micont[A]*Mkt[Nm/A]*Nominal_speed[rad/s]) =
       //            = sum(abs(Icmd[A]*w[rad/s]))/ (u16_number_of_samples * (Micont[A]*Nominal_speed[rad/s])) =
	   //            = sum(abs(Icmd[A]*w[rad/s])) / ( u16_number_of_samples * f_Motor_Rated_Power / mkt)
       

       int s16_vact,s16_index = 0,s16_data_index = u16_Record_Start,s16_vmax,s16_vmin;
       long s32_simplex_delta_v;
       float f_mspeed_ratio =  0.05;


       s16_vmax = s16_vmin = s16_Debug_Ram[0][u16_Record_Start];
       
       for(s16_index=0;s16_index < s16_Recording_Length;++s16_index)
       {
          if (s16_data_index >= u16_Ram_Rec_Length_Avaliable) s16_data_index -=u16_Ram_Rec_Length_Avaliable;

          s16_vact = s16_Debug_Ram[0][s16_data_index];

          if (s16_vact > s16_vmax) s16_vmax = s16_vact;
          else if (s16_vact < s16_vmin) s16_vmin = s16_vact;

          ++s16_data_index;
       }

       //Icmd[A] = Icmd[s8_Units_Cur]*Dipeak[A]/26214 = Icmd[s8_Units_Cur] * 0.001 * s32_Drive_I_Peak[mA] / 26214 = Icmd[s8_Units_Cur] * s32_Drive_I_Peak[mA] / 26.214
       //s16_vact[rad/s] = s16_vact * Vlim[rad/sec]/22750

        // speed change should be above 5%mspeed
       s32_simplex_delta_v = (long)((float)((long)s16_vmax-(long)s16_vmin)/22750.0*(float)BGVAR(s32_V_Lim_Design));
       if (s32_simplex_delta_v < 0) s32_simplex_delta_v=-s32_simplex_delta_v;

       if (s32_simplex_delta_v < (long)(f_mspeed_ratio*(float)BGVAR(u32_Mspeed))) return -1;
       else return 1;
}













