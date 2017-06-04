#include "DSP2834x_Device.h"
#include "Math.h"

#include "AutoHc.def"
#include "Err_Hndl.def"
#include "Record.def"

#include "Ser_Comm.pro"
#include "AutoHc.pro"
#include "Record.pro"
#include "Parser.pro"
#include "Design.pro"
#include "ModCntrl.pro"
#include "Sal.pro"

#include "Ser_Comm.var"
#include "AutoHc.var"
#include "Extrn_Asm.var"
#include "Drive.var"
#include "Record.var"
#include "Units.var"
#include "Exe_Hndl.pro"
#include "MathSupport.pro"

int SalAutoHcCommand(int drive)
{
   // Which hc to use
   // What signal is monitored
   // Injection amplitude
   // Record interval is calculated automatically according to speed
   int s16_current_value, shr_val, s16_index;
   long long s64_half_for_rounding = 0LL;

   // Remove this when completed
   //if (!drive) return (NOT_AVAILABLE);

   if ((u8_Execution_String[0][0] != 'H') && (u8_Execution_String[0][1] != 'C')) return (VALUE_OUT_OF_RANGE);
   // freeze FB option for future
   /*if ((u8_Execution_String[0][2] == 'F') && (u8_Execution_String[0][3] == 'B'))
   {
      if (u8_Execution_String[0][4] == '1') BGVAR(s16_Auto_Hc_Signal) = AUTOHC_MODE_FB1;
      else if (u8_Execution_String[0][4] == '2') BGVAR(s16_Auto_Hc_Signal) = AUTOHC_MODE_FB2;
      else
        return (VALUE_OUT_OF_RANGE);       
   }*/
   if ((u8_Execution_String[0][2] == 'I') && (u8_Execution_String[0][3] == 'C') &&
       (u8_Execution_String[0][4] == 'M') && (u8_Execution_String[0][5] == 'D')   )
   {
      if (u8_Execution_String[0][6] == '1') BGVAR(s16_Auto_Hc_Signal) = AUTOHC_MODE_ICMD1;
      else if (u8_Execution_String[0][6] == '2') BGVAR(s16_Auto_Hc_Signal) = AUTOHC_MODE_ICMD2;
      else
        return (VALUE_OUT_OF_RANGE);  
   }
   else
      return (VALUE_OUT_OF_RANGE);
   // freeze other options for future, keep ICMD only
   /*if ((u8_Execution_String[1][0] == 'P') && (u8_Execution_String[1][1] == 'E')) BGVAR(s16_Auto_Hc_Rec) = AUTOHC_REC_PE;*/
   if ((u8_Execution_String[1][0] == 'I') && (u8_Execution_String[1][1] == 'C') &&
            (u8_Execution_String[1][2] == 'M') && (u8_Execution_String[1][3] == 'D')   ) BGVAR(s16_Auto_Hc_Rec) = AUTOHC_REC_ICMD;
   /*else if ((u8_Execution_String[1][0] == 'V') && (u8_Execution_String[1][1] == 'E')) BGVAR(s16_Auto_Hc_Rec) = AUTOHC_REC_VE;*/
   else
      return (VALUE_OUT_OF_RANGE);

   // Convert the unit according to the type of injection
   if ((BGVAR(s16_Auto_Hc_Signal) == AUTOHC_MODE_ICMD1) || (BGVAR(s16_Auto_Hc_Signal) == AUTOHC_MODE_ICMD2))
   {
      // Convert to internal current
      if (s64_Execution_Parameter[2] > (long long)BGVAR(s32_Drive_I_Peak)) return (VALUE_TOO_HIGH);
      if (s64_Execution_Parameter[2] < 0LL) return (VALUE_TOO_LOW);

      s16_current_value = (int)s64_Execution_Parameter[2];

      shr_val = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
      if (shr_val > 0) s64_half_for_rounding = 1LL << ((long long)shr_val - 1);

      BGVAR(s16_AutoHcIAmp) = (int)(((long long)s16_current_value * (long long)BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix + s64_half_for_rounding) >> (long long)shr_val);
   }
   else // Convert to internal position
   {
      s16_index = UnitsConversionIndex(&s8_Units_Pos, drive);
      BGVAR(s64_AutoHcPAmp) = MultS64ByFixS64ToS64(s64_Execution_Parameter[2],
                                   BGVAR(Unit_Conversion_Table[s16_index]).s64_unit_conversion_to_internal_fix,
                                   BGVAR(Unit_Conversion_Table[s16_index]).u16_unit_conversion_to_internal_shr);

   }
   // Harmonic number
   BGVAR(s16_AutoHcNum) = (int)s64_Execution_Parameter[3];
   
   BGVAR(s16_AutoHcState) = AUTOHC_INIT;

   return SAL_SUCCESS;
}

int SalReadAutoHcCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch(BGVAR(s16_Auto_Hc_Signal))
   {
      case AUTOHC_MODE_ICMD1:
         PrintString("Use: HCICMD1 ",0);
      break;
      case AUTOHC_MODE_ICMD2:
         PrintString("Use: HCICMD2 ",0);
      break;
      case AUTOHC_MODE_FB1:
         PrintString("Use: HCFB1 ",0);
      break;
      case AUTOHC_MODE_FB2:
         PrintString("Use: HCFB2 ",0);
      break;
      default:
         PrintString("No Data",0);
         PrintCrLf();
         return SAL_SUCCESS;
   }

   if (BGVAR(s16_Auto_Hc_Rec) == AUTOHC_REC_ICMD) PrintString("Rec: ICMD\n",0);
   else PrintString("Rec: PE\n",0);

   return SAL_SUCCESS;
}


void CalcAutoHcResult(int drive)
{
   // the autohc have yielded 4 results:
   // f_AutoHcCos_0
   // f_AutoHcCos_90
   // f_AutoHcSin_0
   // f_AutoHcSin_90
   //
   // In addtion we have the injected amp need the injected amplitude s16_AutoHcIAmp or s64_AutoHcPAmp
   float f_den,f_rot_real,f_rot_imag,f_hc_real,f_hc_imag,
         f_err_cos = BGVAR(f_AutoHcCos_90) - BGVAR(f_AutoHcCos_0),
         f_err_sin = BGVAR(f_AutoHcSin_90) - BGVAR(f_AutoHcSin_0);
   int s16_hc_amp,s16_hc_phase;
   // AXIS_OFF;

   f_den = (float)(f_err_cos*f_err_cos+f_err_sin*f_err_sin);
   f_rot_real = (BGVAR(f_AutoHcCos_90)*f_err_cos+BGVAR(f_AutoHcSin_90)*f_err_sin)/f_den;
   f_rot_imag = (BGVAR(f_AutoHcSin_90)*f_err_cos-BGVAR(f_AutoHcCos_90)*f_err_sin)/f_den;

   if ((BGVAR(s16_Auto_Hc_Signal) == AUTOHC_MODE_ICMD1) || (BGVAR(s16_Auto_Hc_Signal) == AUTOHC_MODE_ICMD2))
   {
      f_hc_real = (float)BGVAR(s16_AutoHcIAmp)*(f_rot_imag+f_rot_real);
      f_hc_imag = (float)BGVAR(s16_AutoHcIAmp)*(f_rot_imag-f_rot_real+1);
   }
   else
   {
      f_hc_real = (float)BGVAR(s64_AutoHcPAmp)*(f_rot_imag+f_rot_real);
      f_hc_imag = (float)BGVAR(s64_AutoHcPAmp)*(f_rot_imag-f_rot_real+1);
   }

   s16_hc_amp = (int)(sqrt(f_hc_real*f_hc_real+f_hc_imag*f_hc_imag));
   //s16_hc_phase = (int)(atan(f_hc_imag/f_hc_real)*360.0/3.14);
   // 180.0/(3.14))*65536.0/360.0 = 10430.37835
   s16_hc_phase = (int)(-atan2_cust(f_hc_imag,f_hc_real)*10430.37835);

   switch(BGVAR(s16_Auto_Hc_Signal))
   {
      case AUTOHC_MODE_ICMD1:
         VAR(AX0_s16_Icmd_Harmonic_Phase_1) = (int)s16_hc_phase;
         s16_hc_amp = (int)((float)s16_hc_amp * (float)BGVAR(s32_Drive_I_Peak) / 26214.0);
         SalHcIcmd1AmpCommand((long long)s16_hc_amp,drive);       
      break;

      case AUTOHC_MODE_ICMD2:
         VAR(AX0_s16_Icmd_Harmonic_Phase_2) = (int)s16_hc_phase;
         s16_hc_amp = (int)((float)s16_hc_amp * (float)BGVAR(s32_Drive_I_Peak) / 26214.0);
         SalHcIcmd2AmpCommand((long long)s16_hc_amp,drive);           
      break;

      case AUTOHC_MODE_FB1:
         VAR(AX0_s16_Fb_Harmonic_Phase_1) = (int)s16_hc_phase;
         LVAR(AX0_s32_Fb_Harmonic_Amp_1) = (long)s16_hc_amp;   
      break;

      case AUTOHC_MODE_FB2:
         VAR(AX0_s16_Fb_Harmonic_Phase_2) = (int)s16_hc_phase;
         LVAR(AX0_s32_Fb_Harmonic_Amp_2) = (long)s16_hc_amp;           
      break;
   }
}

int InitAutoHc(int drive)
{
   // Avg speed and determine record interval
   float f_temp,f_vlim_rpm = (float)BGVAR(s32_V_Lim_Design) * 0.00011176;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   BGVAR(s32_Spd_Avg_Acc) += (long)VAR(AX0_s16_Vel_Var_Fb_0);
   ++BGVAR(s16_Spd_Avg_Counter);

   if (BGVAR(s16_Spd_Avg_Counter) == 4096)
   {
      BGVAR(s32_Spd_Avg_Acc) >>= 12L;
      // convert spd to rps
      f_temp = 1.0/((float)BGVAR(s32_Spd_Avg_Acc)/22750.0*f_vlim_rpm/60.0);// units sec/rev
      f_temp = f_temp*32000/2000;// convert to record gap
      BGVAR(s16_AutoHc_Rec_Gap) = (int)f_temp;
      if (!BGVAR(s16_AutoHc_Rec_Gap)) BGVAR(s16_AutoHc_Rec_Gap) = 1;

      return 0;
   }

   return 1;
}

void AutoHcRecorder(int drive)
{
   long code1, code2, code3;

   //arm recorder
   RecordOffCommand();
   STORE_EXECUTION_PARAMS_0_15
   ConvertMnemonic2Code("PE", &code1, &code2, &code3, 0);
   s64_Execution_Parameter[2] = SearchMnemonic(code1, code2, code3)-1;
   ConvertMnemonic2Code("ICMD", &code1, &code2, &code3, 0);
   s64_Execution_Parameter[3] = SearchMnemonic(code1, code2, code3)-1;
   ConvertMnemonic2Code("V", &code1, &code2, &code3, 0);
   s64_Execution_Parameter[4] = SearchMnemonic(code1, code2, code3)-1;
   ConvertMnemonic2Code("MECHANGLE", &code1, &code2, &code3, 0);
   s64_Execution_Parameter[5] = SearchMnemonic(code1, code2, code3)-1;
   ConvertMnemonic2Code("VE", &code1, &code2, &code3, 0);
   s64_Execution_Parameter[6] = SearchMnemonic(code1, code2, code3)-1;   
   s64_Execution_Parameter[7] = 0;
   SalRecordCommand(drive,(long)BGVAR(s16_AutoHc_Rec_Gap), 2000LL,&s64_Execution_Parameter[2]);
   RESTORE_EXECUTION_PARAMS_0_15
   ++drive;
//   SalRecTrigCommand(drive,RECORD_IMM_TRIGGER,0,0,0);
}

void InjectTestSignal(int drive,int s16_inject_point)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   switch(BGVAR(s16_Auto_Hc_Signal))
   {
      case AUTOHC_MODE_ICMD1:
       
         VAR(AX0_s16_Icmd_Harmonic_Phase_1) = s16_inject_point;
         VAR(AX0_s16_Icmd_Harmonic_Num_1) = BGVAR(s16_AutoHcNum);
         VAR(AX0_s16_Icmd_Harmonic_Amp_1) = BGVAR(s16_AutoHcIAmp);
         break;
      case AUTOHC_MODE_ICMD2:
         VAR(AX0_s16_Icmd_Harmonic_Phase_2) = s16_inject_point;
         VAR(AX0_s16_Icmd_Harmonic_Num_2) = BGVAR(s16_AutoHcNum);
         VAR(AX0_s16_Icmd_Harmonic_Amp_2) = BGVAR(s16_AutoHcIAmp);   
         break;

      case AUTOHC_MODE_FB1:
         VAR(AX0_s16_Fb_Harmonic_Phase_1) = s16_inject_point;
         VAR(AX0_s16_Fb_Harmonic_Num_1) = BGVAR(s16_AutoHcNum);
         LVAR(AX0_s32_Fb_Harmonic_Amp_1) = (long)BGVAR(s64_AutoHcPAmp);
         break;
         
      case AUTOHC_MODE_FB2:
         VAR(AX0_s16_Fb_Harmonic_Phase_2) = s16_inject_point;
         VAR(AX0_s16_Fb_Harmonic_Num_2) = BGVAR(s16_AutoHcNum);
         LVAR(AX0_s32_Fb_Harmonic_Amp_2) = (long)BGVAR(s64_AutoHcPAmp);
      break;
   }

}

long long GetAutoHcSignal(int rec_flag,int rec_ptr)
{
   long long ret_value;
   //if((BGVAR(s16_Auto_Hc_Signal) == AUTOHC_MODE_ICMD1) || (BGVAR(s16_Auto_Hc_Signal) == AUTOHC_MODE_ICMD2))
   //rec_flag = AUTOHC_REC_ICMD;
   //else rec_flag = AUTOHC_REC_PE;
   //rec_ptr = u16_Record_Start;
   switch (rec_flag)
   {
      case AUTOHC_REC_ICMD:
         ret_value = (long long)(s16_Debug_Ram[4][rec_ptr]);
      break;

      case AUTOHC_REC_PE:
         ret_value = (long long)s16_Debug_Ram[3][rec_ptr] ; //MSB
         ret_value <<= 16;
         ret_value |= (long long)s16_Debug_Ram[2][rec_ptr] & 0x0FFFFLL;
         ret_value <<= 16;
         ret_value |= (long long)s16_Debug_Ram[1][rec_ptr] & 0x0FFFFLL;
         ret_value <<= 16;
         ret_value |= (long long)s16_Debug_Ram[0][rec_ptr] & 0x0FFFFLL;
      break;
      
      case AUTOHC_REC_VE:
         ret_value = (long long)s16_Debug_Ram[8][rec_ptr] ; 
      break;      
   }

   return ret_value;
}

void GetAutoHcData(int drive,float *f_cos_acc,float *f_sin_acc)
{
   
   int s16_harmonic_number,s16_rec_ptr,s16_temp,s16_mech_angle;
   float f_sin,f_cos,f_angle,f_sig_avg,f_temp;
   long long s64_avg_acc;
   
   REFERENCE_TO_DRIVE;
   
   s64_avg_acc = 0LL;
   *f_cos_acc = 0.0;
   *f_sin_acc = 0.0;

   s16_harmonic_number = BGVAR(s16_AutoHcNum);

   s16_rec_ptr = u16_Record_Start;
   for (s16_temp=0; s16_temp<2000; ++s16_temp)
   {
      s64_avg_acc += GetAutoHcSignal(BGVAR(s16_Auto_Hc_Rec),s16_rec_ptr);
      s16_rec_ptr++;
      if (s16_rec_ptr >= u16_Ram_Rec_Length_Avaliable) s16_rec_ptr = s16_rec_ptr-u16_Ram_Rec_Length_Avaliable;
   }
   f_sig_avg = (float)s64_avg_acc/2000.0;

   s16_rec_ptr = u16_Record_Start;
   for (s16_temp=0; s16_temp<2000; ++s16_temp)
   {
      s64_avg_acc = GetAutoHcSignal(BGVAR(s16_Auto_Hc_Rec),s16_rec_ptr);
      //s64_avg_acc_dSS += s64_avg_acc;
      //f_temp = (float)(GetAutoHcSignal(drive,s16_rec_ptr)) - f_sig_avg;
      
      f_temp = (float)s64_avg_acc - f_sig_avg;
      s16_mech_angle = s16_Debug_Ram[7][s16_rec_ptr];
      f_angle = (float)s16_harmonic_number*(float)s16_mech_angle*9.58738e-5;//2*3.14159265358979323846/65536
      f_sin = sin(f_angle); f_cos = cos(f_angle);

      *f_cos_acc += f_temp*f_cos;
      *f_sin_acc += f_temp*f_sin;

      s16_rec_ptr++;
      if (s16_rec_ptr >= u16_Ram_Rec_Length_Avaliable) s16_rec_ptr = s16_rec_ptr-u16_Ram_Rec_Length_Avaliable;
   }
}

void AutoHcHandler(int drive)
{
	//drive+=0;
    REFERENCE_TO_DRIVE;
   if (!Enabled(drive)) BGVAR(s16_AutoHcState) = AUTOHC_IDLE;

   switch (BGVAR(s16_AutoHcState))
   {
      case AUTOHC_IDLE:
         BGVAR(s16_Spd_Avg_Counter) = 0;
         BGVAR(s32_Spd_Avg_Acc) = 0L;
      break;

      case AUTOHC_INIT:
         if (InitAutoHc(drive) == 1) break;
         InjectTestSignal(drive,AUTOHC_INJECT_PHASE_0);
         AutoHcRecorder(drive);
         BGVAR(f_AutoHcCos_0) = BGVAR(f_AutoHcSin_0) = BGVAR(f_AutoHcCos_90) = BGVAR(f_AutoHcSin_90) = 0.0;
         BGVAR(s32_AutoHc_Timer) = Cntr_1mS;
         BGVAR(s16_AutoHcState) = AUTOHC_RECTRIG1;
      break;

      case AUTOHC_RECTRIG1: // Wait 1sec before starting to record, to allow the system to stabilize
      case AUTOHC_RECTRIG2:
         if (!PassedTimeMS(1000L,BGVAR(s32_AutoHc_Timer)));
         SalRecTrigCommand(drive,RECORD_IMM_TRIGGER,0,0,0);
         BGVAR(s16_AutoHcState) += 10;
      break;
      

      case AUTOHC_PH_0_DONE:
         if ((s16_Record_Flags & DATA_AVAILABLE_MASK) == 0) break;
         GetAutoHcData(drive,&BGVAR(f_AutoHcCos_0),&BGVAR(f_AutoHcSin_0));
         InjectTestSignal(drive,-AUTOHC_INJECT_PHASE_90);
         AutoHcRecorder(drive);
         BGVAR(s32_AutoHc_Timer) = Cntr_1mS;
         BGVAR(s16_AutoHcState) = AUTOHC_RECTRIG2;
      break;

      case AUTOHC_PH_90_DONE:
         if ((s16_Record_Flags & DATA_AVAILABLE_MASK) == 0) break;
         InjectTestSignal(drive,AUTOHC_INJECT_OFF);
         GetAutoHcData(drive,&BGVAR(f_AutoHcCos_90),&BGVAR(f_AutoHcSin_90));
      
         BGVAR(s16_AutoHcState) = AUTOHC_CALC;
      break;

      case AUTOHC_CALC:
         CalcAutoHcResult(drive);
         BGVAR(s16_AutoHcState) = AUTOHC_DONE;
      break;

      case AUTOHC_DONE:
         BGVAR(s16_Spd_Avg_Counter) = 0;
         BGVAR(s32_Spd_Avg_Acc) = 0L;      
      break;

      default:
         BGVAR(s16_AutoHcState) = AUTOHC_IDLE;
      break;
   }
}

int HcTuneStatusCommand(long long *data, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(BGVAR(s16_AutoHcState) == AUTOHC_IDLE)
      *data = AUTOHC_ST_IDLE; // HCTUNE is idle

   else if ((BGVAR(s16_AutoHcState) > AUTOHC_IDLE) && (BGVAR(s16_AutoHcState) < AUTOHC_DONE))
      *data = AUTOHC_ST_IN_PROCESS; // HCTUNE is in process

   else if (BGVAR(s16_AutoHcState) == AUTOHC_DONE)
      *data = AUTOHC_ST_DONE; // HCTUNE is done
      
   else
      *data = AUTOHC_ST_RESERVED; // Reserved

   return SAL_SUCCESS;
}

/****************************************************************************/
/*                                                                          */
/*  THIS FUNCTION IS COPIED AS IS FROM ORIGINAL TI LIBRARY                  */
/*  THE REASON IS TO SAVE RAM (TI MATH FUNCTIONS RUN FROM RAM)              */
/*                                                                          */
/*  ATAN2() - Arctangent2                                                   */
/*                                                                          */
/*  Based on the algorithm from "Software Manual for the Elementary         */
/*  Functions", Cody and Waite, Prentice Hall 1980, chapter 11.             */
/*                                                                          */
/*  if x >= 0, result = atan(y / x)                                         */
/*  if x < 0 & y >= 0, result = 3.14159265358979323846 + atan(y / x)        */
/*  if x < 0 & y < 0, result = atan (y / x) - 3.14159265358979323846        */
/*                                                                          */
/****************************************************************************/
double atan2_cust(double y, double x)
{
    double g, p, q, r;
    int    sign;
    int    t = 0;
    int   ys = (y >= 0);
    int   xs = (x >= 0);
    int    n = 0;
    int   yn = 0;
    int   xn = 0;

    static const _DATA_ACCESS double a[4] = {0.0, 0.52359877559829887308, 
                                                  1.57079632679489661923,
                                                  1.04719755119659774615};

    /*********************************************************************/
    /* check for error in domain                                         */
    /*********************************************************************/
    if (x == 0)
    {
       if (y == 0) { return (0.0); }
       else          return (ys ? 1.57079632679489661923 : -1.57079632679489661923);
    }

    /*********************************************************************/
    /* Determine whether overflow is possible; if so, return (+/-) 3.14159265358979323846/2  */
    /*********************************************************************/
    frexp(y, &yn);
    frexp(x, &xn);

    if ((yn - xn) > (DBL_MAX_EXP - 2))  /* Use (DBL_MAX_EXP-2) for safety */
    {
       return (ys ? 1.57079632679489661923 : -1.57079632679489661923);
    }

    /*********************************************************************/
    /* check for negative                                                */
    /*********************************************************************/
    sign = ((x = y / x) < 0.0);

    if ((x = fabs(x)) > 1.0)
    {
       x = 1.0 / x;	
       n = 2;
       t = 1;                                    /* negate partial result */
    }

    /**********************************************************************/
    /* if (x > (2 - sqrt(3)) x = (x * sqrt(3) -1) / (sqrt(3) + x)         */
    /**********************************************************************/
    if (x > 0.26794919243112270647)
    {
       x = (x * 1.73205080756887729353 - 1.0) / (1.73205080756887729353 + x); 
       ++n;
    }

    /**********************************************************************/
    /* determine polynomial expression                                    */
    /**********************************************************************/
    g = x * x;

    p = (-0.427444985367930329e1 * g - 0.427432672026241096e1) * g;
    q = g + 0.128229801607919841e2;


    /*********************************************************************/
    /* calculate the result multiplied by the correct sign               */
    /*********************************************************************/
    r = ((p / q) * x + x);
    r = (t ? -r : r) + a[n];
    r = (sign ? -r : r); 

    /*********************************************************************/
    /* adjust result to be in correct quadrant                           */
    /*********************************************************************/
    if (!xs && ys)  r = (3.14159265358979323846 + r);
    if (!xs && !ys) r = (r - 3.14159265358979323846);

    return (r);
}

