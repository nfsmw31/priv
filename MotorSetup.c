#include "DSP2834x_Device.h"
#include <string.h>
#include "Current.def"
#include "CommFdbk.def"
#include "design.def"
#include "Err_Hndl.def"
#include "Flash.def"
#include "FltCntrl.def"
#include "MultiAxis.def"
#include "i2c.def"
#include "402fsa.def"
#include "ExFbVar.def"
#include "MotorParamsEst.def"

#include "Burnin.var"
#include "CommFdbk.var"
#include "Drive.var"
#include "Endat.var"
#include "Extrn_Asm.var"
#include "FltCntrl.var"
#include "FlashHandle.var"
#include "Hiface.var"
#include "Motor.var"
#include "MotorSetup.var"
#include "PhaseFind.var"
#include "Position.var"
#include "Ser_Comm.var"
#include "Units.var"
#include "User_Var.var"
#include "ExFbVar.var"
#include "Zeroing.var"
#include "MotorParamsEst.var"

#include "Prototypes.pro"

extern int Enc_Hall_Switch_Tbl_4to5to1to3to2to6;
extern int Enc_Hall_Switch_Tbl_4to6to2to3to1to5;

long min(long a, long b)
{
   if (a <= b) return a;
   else return b;
}


long labs(long a)
{
   if (a < 0L) return -a;
   else return a;
}


long long llabs(long long a)
{
   if (a < 0LL) return -a;
   else return a;
}


int abs(int a)
{
   if (a < 0) return -a;
   else return a;
}


int pos_modulu(int a, int modulu)
{
   if (modulu == 0) return 0;
   if (modulu < 0) modulu = -modulu;
   while (a < 0) a += modulu;

   return (a%modulu);
}


// This will handle reading the halls from Tamagawa and regular encoders 0-when done 1-while processing
int ReadHallsFromFeedback(int *halls, int drive)
{
   int halls_temp = 0, ret_val = 1;

   if (FEEDBACK_TAMAGAWA)
   {
      switch (BGVAR(u16_Read_Halls_State))
      {
         case 0:
            BGVAR(u16_Motor_Setup_Allow_Enable) = 1; // This flag will inhibit ModCntrl from disabling the drive while Tamagawa inits
            VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_ENC_TAMAGAWA & 0xffff);  //Changed to avoid Remark
            BGVAR(u16_Read_Halls_State) = 1;
         break;

         case 1:
            if (VAR(AX0_s16_Enc_State_Ptr) > (int)((long)&ENC_STATE_0_INC_ENC & 0xffff)) //Changed to avoid Remark
            {
               ReadHalls(&halls_temp, drive);
               // If bad halls read - retry once
               if (((halls_temp == 0) || (halls_temp == 7)) && (!BGVAR(u16_Read_Halls_Trials)))
                  BGVAR(u16_Read_Halls_Trials)++;
               else
               {
                  BGVAR(u16_Motor_Setup_Allow_Enable) = 0;
                  BGVAR(u16_Read_Halls_Trials) = 0;
                  *halls = halls_temp;
                  ret_val = 0;
               }
               BGVAR(u16_Read_Halls_State) = 0;
            }
         break;
      }
   }
   else
   {
      ReadHalls(halls, drive);
      ret_val = 0;
   }
   return ret_val;
}


// Adjust MENCRES to closest 100X or 2^X
int FixMencres(unsigned long *u32_adj_mencres)
{
   long s32_diff = 0, s32_100X = (*u32_adj_mencres / 100) * 100, s32_2_pow_X;
   long s32_mencres_temp = *u32_adj_mencres;
   int ret_val = 0;
   #define DIFF_THRESH  1

   while (s32_mencres_temp > 0)   // Calculate closest 2^X
   {
      s32_2_pow_X++;
      s32_mencres_temp >>= 1;
   }
   s32_2_pow_X = 1L << s32_2_pow_X;

   s32_mencres_temp = *u32_adj_mencres;

   // Check which value to fix MENCRES
   s32_diff = labs(s32_mencres_temp - s32_100X);
   if (s32_diff <= DIFF_THRESH) {s32_mencres_temp = s32_100X; ret_val = 1;}

   s32_diff = labs(s32_mencres_temp - (s32_100X + 100));
   if (s32_diff <= DIFF_THRESH) {s32_mencres_temp = (s32_100X + 100); ret_val = 1;}

   s32_diff = labs(s32_mencres_temp - s32_2_pow_X);
   if (s32_diff <= DIFF_THRESH) {s32_mencres_temp = s32_2_pow_X; ret_val = 1;}

   s32_diff = labs(s32_mencres_temp - (s32_2_pow_X << 1));
   if (s32_diff <= DIFF_THRESH) {s32_mencres_temp = (s32_2_pow_X << 1); ret_val = 1;}

   s32_diff = labs(s32_mencres_temp - (s32_2_pow_X >> 1));
   if (s32_diff <= DIFF_THRESH) {s32_mencres_temp = (s32_2_pow_X >> 1); ret_val = 1;}

   if (ret_val) *u32_adj_mencres = s32_mencres_temp;

   return ret_val;
}


// This will return the index in the halls switch table and the direction of halls propagation
// 0 - if calculation OK
// 1 - if illegal halls transition
int HallsDirection(int s16_neg_hall, int s16_pos_hall, int* s16_halls_table_index, int* s16_halls_direction)
{
   int s16_halls_sum = s16_neg_hall + s16_pos_hall, ret_val = 0;

   if ((s16_halls_sum < 4) || (s16_halls_sum > 10) || (s16_halls_sum == 7))
      return 1;

   if ((s16_neg_hall==1) && (s16_pos_hall==3)) {*s16_halls_table_index = 0; *s16_halls_direction = 0;}
   else if ((s16_neg_hall==3) && (s16_pos_hall==2)) {*s16_halls_table_index = 1; *s16_halls_direction = 0;}
   else if ((s16_neg_hall==5) && (s16_pos_hall==1)) {*s16_halls_table_index = 2; *s16_halls_direction = 0;}
   else if ((s16_neg_hall==2) && (s16_pos_hall==6)) {*s16_halls_table_index = 4; *s16_halls_direction = 0;}
   else if ((s16_neg_hall==4) && (s16_pos_hall==5)) {*s16_halls_table_index = 5; *s16_halls_direction = 0;}
   else if ((s16_neg_hall==6) && (s16_pos_hall==4)) {*s16_halls_table_index = 6; *s16_halls_direction = 0;}
   else if ((s16_pos_hall==1) && (s16_neg_hall==3)) {*s16_halls_table_index = 0; *s16_halls_direction = 1;}
   else if ((s16_pos_hall==3) && (s16_neg_hall==2)) {*s16_halls_table_index = 1; *s16_halls_direction = 1;}
   else if ((s16_pos_hall==5) && (s16_neg_hall==1)) {*s16_halls_table_index = 2; *s16_halls_direction = 1;}
   else if ((s16_pos_hall==2) && (s16_neg_hall==6)) {*s16_halls_table_index = 4; *s16_halls_direction = 1;}
   else if ((s16_pos_hall==4) && (s16_neg_hall==5)) {*s16_halls_table_index = 5; *s16_halls_direction = 1;}
   else if ((s16_pos_hall==6) && (s16_neg_hall==4)) {*s16_halls_table_index = 6; *s16_halls_direction = 1;}
   else ret_val = 1;

   return ret_val;
}


void MotorSetupHandler(int drive)
{
   // AXIS_OFF;

   static unsigned int u16_pole_pairs_count = 0, u16_pole_pairs_count_first = 1;
   static long long s64_delta_position = 0LL, s64_first_position = 0LL, s64_second_position = 0LL;
   static long long s64_first_position_fdbk_endat_fix = 0, s64_second_position_fdbk_endat_fix = 0;
   static long long s64_first_position_endat_endat_fix = 0, s64_second_position_endat_endat_fix = 0;
   static int s16_direction_test = -1, s16_halls_direction = -1, s16_small_pitch_dir_test = -1;
   static int s16_endat_dir_endat_fix = 0, s16_fdbk_dir_endat_fix = 0, u16_skip_endat_fix = 0;
   static int s16_init_current_value_endat_fix = 0, s16_current_halls = 0, s16_prev_halls;
   static int s16_movement_counter = 0;
   static int s16_current_value = 0, s16_commutation_delta, u16_velocity_filter = 0, s16_current_delta = 0;
   static int u16_halls_index = 0, u16_resolver_mphase_set = 0, s16_halls_filter = -1;
   static long s32_last_pos = 0L;
   static unsigned int u16_sincos_direction_endat_fix = 0, u16_mphase_calc = 0;
   static unsigned int u16_electangle_pos, u16_electangle_neg[7], u16_index_polarity = 0, u16_menczpos = 0;
   static long s32_start_position = 0L, s32_start_position_hi = 0L;
   static unsigned long u32_calculated_mencres = 0L, u32_comm_only_mencres = 0L, u32_iterations;
   static unsigned int index_counter = 0;
   int s16_iu = 0, s16_iv = 0, i, i_prev, i_next, halls_fault, return_val;
   unsigned int u16_mfbdir_build = 0, u16_hwpos_num_of_bits = 16, u16_mphase_calc_2 = 0;
   long long s64_temp = 0LL, s64_mencres_per_pole, s64_hwpos, s64_rollover_value;
   unsigned long long u64_div_factor = 1LL;
   int s16_halls_table_index = 0, mpoles_fault = 0, hall_index_num, s16_dummy;
   unsigned long u32_temp = 65536L;
   long s32_temp = 0;

   if ( (!Enabled(DRIVE_PARAM))                                                    &&
        (BGVAR(u16_Motor_Setup_State) > MOTOR_SETUP_WAIT_FOR_ENABLE)               &&
        (BGVAR(u16_Motor_Setup_State) < MOTOR_SETUP_TERMINATE)                     &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_TORQUE_COMMAND_SETUP)         &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_TORQUE_COMMAND_CONFIG)        &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_TORQUE_WAIT_FOR_ENABLE)       &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_CHANGE_DIRECTION)             &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_CHANGE_DIRECTION_ENABLE)      &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_STEGMANN_CONFIG)              &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_STEGMANN_WAIT_FOR_ENABLE)     &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_INDEX_ONLY_LOOK_FOR_INDEX)    &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_INDEX_ONLY_LOOK_FOR_INDEX_END)&&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_ENDAT_DIR)                    &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_ENDAT_DIR_DISABLE)            &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_ENDAT_DIR_CONFIG)             &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_ENDAT_DIR_ENABLE)             &&
        (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_TORQUE_WAIT_ENC_INIT)           )
   {
      BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_DISABLED);
      BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
   }

   switch (BGVAR(u16_Motor_Setup_State))
   {
      case MOTOR_SETUP_IDLE:
      break;

      // Start by issuing ESTMOTORPARAM to identify the ML & MR
      case MOTOR_SETUP_START:
         BGVAR(u32_Motor_Setup_Status) = (MOTOR_SETUP_ISSUED | MOTOR_SETUP_ACTIVE | MOTOR_SETUP_STILL_RUNNING);

         BGVAR(s32_ParamsBackup)[MPOLES_STORE_INDEX]        = (long)BGVAR(u16_Mpoles);
         BGVAR(s32_ParamsBackup)[MENCZPOS_STORE_INDEX]      = (long)VAR(AX0_s16_Index_Elect_Pos);
         BGVAR(s32_ParamsBackup)[MPHASE_STORE_INDEX]        = (long)VAR(AX0_s16_Electrical_Phase_Offset);
         BGVAR(s32_ParamsBackup)[VLIM_STORE_INDEX]          = BGVAR(s32_V_Lim_Design);
         BGVAR(s32_ParamsBackup)[ILIM_STORE_INDEX]          = BGVAR(s32_Ilim_User);
         BGVAR(s32_ParamsBackup)[MFBDIR_STORE_INDEX]        = (long)BGVAR(u16_Motor_Feedback_Direction);
         BGVAR(s32_ParamsBackup)[DIR_STORE_INDEX]           = (long)BGVAR(s16_Direction);
         BGVAR(s32_ParamsBackup)[MENCRES_STORE_INDEX]       = BGVAR(u32_User_Motor_Enc_Res);
         BGVAR(s32_ParamsBackup)[DISMODE_STORE_INDEX]       = (long)BGVAR(u16_Disable_Mode);
         BGVAR(s32_ParamsBackup)[OPMODE_STORE_INDEX]        = (long)VAR(AX0_s16_Opmode);
         BGVAR(s32_ParamsBackup)[FB_OPMODE_STORE]           = (long)BGVAR(s16_CAN_Opmode);
         BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX]      = (long)VAR(AX0_s16_Motor_Enc_Type);
         BGVAR(s32_ParamsBackup)[PEMAX_LO_STORE_INDEX]      = (long)(BGVAR(u64_Pe_Max) & 0xFFFFFFFF);
         BGVAR(s32_ParamsBackup)[PEMAX_HI_STORE_INDEX]      = (long)((BGVAR(u64_Pe_Max)>>32LL) & 0xFFFFFFFF);
         BGVAR(s32_ParamsBackup)[ISTOP_STORE_INDEX]         = BGVAR(u32_Stop_Current);
         BGVAR(s32_ParamsBackup)[MFBMODE_STORE_INDEX]       = (long)BGVAR(u16_Motor_Enc_Interpolation_Mode);
         BGVAR(s32_ParamsBackup)[ABSOFFSET_LO_STORE_INDEX]  = (long)(BGVAR(s64_Abs_Fdbk_Offset) & 0xFFFFFFFF);
         BGVAR(s32_ParamsBackup)[ABSOFFSET_HI_STORE_INDEX]  = (long)((BGVAR(s64_Abs_Fdbk_Offset)>>32LL) & 0xFFFFFFFF);
         BGVAR(s32_ParamsBackup)[ML_STORE_INDEX]            = BGVAR(u32_Mlmin);
         BGVAR(s32_ParamsBackup)[MR_STORE_INDEX]            = BGVAR(u32_Motor_Res);
         BGVAR(s32_ParamsBackup)[COMMODE_STORE_INDEX]       = BGVAR(u8_Comm_Mode);
         BGVAR(s32_ParamsBackup)[SFB_MODE_INDEX]            = BGVAR(s16_SFBMode);
         BGVAR(s32_ParamsBackup)[SFB_DIR_INDEX]             = BGVAR(s16_SFB_Direction);

         //BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_SETUP;
         //BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_WAIT_FOR_ESTMOTOR_PARAM;
         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_SETUP;
      break;

      case MOTOR_SETUP_WAIT_FOR_ESTMOTOR_PARAM:
         /*
         if ( (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_FAULT)                            ||
              ( (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_DONE)                        &&
                ( (BGVAR(s32_Param_Est_ML) > 1000000L) || (BGVAR(s32_Param_Est_ML) < 1L) ||
                  (BGVAR(s32_Param_Est_MR) > 50000L)   || (BGVAR(s32_Param_Est_MR) < 0L)   )  )  )*/

         if ( (BGVAR(s16_Motor_Params_Est_State) < 0)                                                        ||
              ( (BGVAR(s16_Motor_Params_Est_State) == MOTOR_PARAMS_EST_STATE_DONE)                        &&
                ( (BGVAR(s32_Motor_Params_Est_Ml) > 1000000L) || (BGVAR(s32_Motor_Params_Est_Ml) < 1L) ||
                  (BGVAR(s32_Motor_Params_Est_Mr) > 50000L)   || (BGVAR(s32_Motor_Params_Est_Mr) < 0L)   )  )  )

         { //not sure this is required since the motor params est procedure does not change the values
            SalMlminCommand((long long)BGVAR(s32_ParamsBackup)[ML_STORE_INDEX],drive);
            SalMresCommand((long long)BGVAR(s32_ParamsBackup)[MR_STORE_INDEX],drive);

            BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_ESTMOTORPARAM_ERR;
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_SETUP; // Check if failure caused by phases dis-connected
            break;
         }
         else if (BGVAR(s16_Motor_Params_Est_State) == MOTOR_PARAMS_EST_STATE_DONE) //if (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_DONE)
         {
            s32_temp = labs(BGVAR(s32_Motor_Params_Est_Mr) - BGVAR(u32_Motor_Res)); //s32_temp = labs(BGVAR(s32_Param_Est_MR) - BGVAR(u32_Motor_Res));
            if ((s32_temp > (BGVAR(u32_Motor_Res) >> 2L)) && ((s32_temp > 1000) || (BGVAR(u32_Motor_Res) == 0L)))
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MR_ADJUST;
/*
            if (labs(BGVAR(s32_Param_Est_ML) - BGVAR(u32_Mlmin)) > (BGVAR(u32_Mlmin) >> 2L))
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_ML_ADJUST;
 */
            if (labs(BGVAR(s32_Motor_Params_Est_Ml) - BGVAR(u32_Mlmin)) > (BGVAR(u32_Mlmin) >> 2L))
                         BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_ML_ADJUST;

            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_SETUP;
         }
      break;

      // Store drive parameters which will be changed during the procedure
      // And set various state flags and parameters
      case MOTOR_SETUP_SETUP:
         BGVAR(s16_Max_Current) = ((long long)min(BGVAR(s32_Drive_I_Cont), BGVAR(s32_Motor_I_Cont)) * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
            >> (long long)BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;

         s16_current_value = BGVAR(s16_Max_Current) >> 6;
         if (!s16_current_value) s16_current_value = BGVAR(s16_Max_Current) >> 4;
         if (!s16_current_value) s16_current_value = BGVAR(s16_Max_Current) >> 2;
         s16_init_current_value_endat_fix        = s16_current_value; // Keep original current_value
         s16_endat_dir_endat_fix                 = 0;
         s16_fdbk_dir_endat_fix                  = 0;
         u16_sincos_direction_endat_fix          = 0;
         s16_commutation_delta                   = 75;
         u32_iterations                          = 0;
         u16_pole_pairs_count                    = 0;
         s16_direction_test                      = -1;
         u16_index_polarity                      = 0;
         s16_halls_direction                     = -1;
         u16_mphase_calc                         = 0;
         u16_menczpos                            = 0;
         index_counter                           = 0;
         u16_electangle_pos                      = 0;
         u16_velocity_filter                     = 0;
         u16_halls_index                         = 0;
         u16_resolver_mphase_set                 = 0;
         s16_halls_filter                        = -1;
         BGVAR(u16_Read_Halls_State)             = 0;
         BGVAR(u16_Read_Halls_Trials)            = 0;
         u16_skip_endat_fix                      = 0;

         for (i = 0; i < 7; i++)
         {
            BGVAR(s16_Halls_Neg)[i] = 0;
            u16_electangle_neg[i]   = 0;
         }

         BGVAR(u64_Pe_Max) = 0LL;
         BGVAR(s64_Abs_Fdbk_Offset) = 0LL;

         SalIstopCommand((long long)BGVAR(s32_Drive_I_Peak), drive);
         SalDisableModeCommand((long long)2, drive); // Activate DB during stop

         // Increase ILIM, VLIM to avoid limitations during the procedure
         SalIlimCommand((long long)BGVAR(s32_Imax), drive);
         VLimCommand((long long)BGVAR(s32_Vmax), drive, &s16_dummy);
         SalMotorFeedbackDirectionCommand(0LL, drive);         // Zero MFBDIR
         BGVAR(s16_Direction) = 0;                             // Zero DIR
         
         if (IS_DUAL_LOOP_ACTIVE)
            SalSFBModeCommand(2LL,drive); //switch to SFBMODE=2 - monitoring SFB
         
         SalSfbDirectionCommand(0LL,drive); // zero SFBDIR
         

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_ML_ADJUST)
               SalMlminCommand((long long)BGVAR(s32_Motor_Params_Est_Ml),drive); //SalMlminCommand((long long)BGVAR(s32_Param_Est_ML),drive);

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MR_ADJUST)
            SalMresCommand((long long)BGVAR(s32_Motor_Params_Est_Mr),drive);//SalMresCommand((long long)BGVAR(s32_Param_Est_MR),drive);

         if (BGVAR(u16_FdbkType) == INC_ENC_FDBK)     // Set MFBMODE to 0 (no interpolation)
            SalMotorEncInterpolationModeCommand(0LL, drive);

         BGVAR(s16_Motor_Setup_Current) = 0;

         VAR(AX0_s16_Skip_Flags) |= MOTOR_SETUP_MASK;
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) = 0;

         // Set commode to 0, to allow enabling without FB intervention
         SalComModeCommand(0LL, drive);

         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_CONFIG;
      break;

      case MOTOR_SETUP_CONFIG:
         return_val = SalConfigCommand(drive);
         if (return_val == SAL_NOT_FINISHED) break;
         if (return_val != SAL_SUCCESS)
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_CONFIG_FAIL);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            break;
         }
         if ( (FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_ERROR_STATE)) break;
         if ( (FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) == COMM_FDBK_ERROR_STATE) )
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_COMM_FDBK_ERROR);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
         if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(u16_Read_Mtp_State) != MTP_INIT_INIT) ) break;
         if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(s16_MTP_Error) != 0) )
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ERROR);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }

         LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &BGVAR(s16_Motor_Setup_Current);

         // This is to allow enable without phase find
         if ( FEEDBACK_WITH_ENCODER                                &&
              (BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX] >= 1) &&
              (BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX] <= 4)   )
         {
            BGVAR(u16_Motor_Setup_Allow_Enable) = 1; //allow enable when menctype with phasefind
            VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_2 & 0xffff);
            BGVAR(s64_Faults_Mask) &= ~PHASE_FIND_FLT_MASK;
            BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;
            VAR(AX0_s16_PhaseFindRTBits) = 0;
            if (BGVAR(s32_ParamsBackup)[MPHASE_STORE_INDEX])
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MPHASE_ADJUST;
         }

         // Return next line when ML/MR calculation is back
         //EnableCommand(drive);

         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_WAIT_FOR_ENABLE;
      break;

      // The procedure will begin when the drive will enable
      case MOTOR_SETUP_WAIT_FOR_ENABLE:
         if (Enabled(DRIVE_PARAM))
         {
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_FORCE_BUILD;
            // In "small pitch" mode rise current slowly to avoid jumps.
            if(BGVAR(u16_Motor_Setup_Small_Pitch)) BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_PRE_TEST_SP;
         }
      break;

      case MOTOR_SETUP_PRE_TEST_SP:
         if (!Enabled(drive)) break;
         do {
            s16_dummy = Cntr_3125;
            s64_first_position  = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
         } while (s16_dummy != Cntr_3125);
         s16_current_value = 10;
         s16_movement_counter = 10;
         s16_current_delta = 25;
         s16_small_pitch_dir_test = -1;
         //VAR(AX0_s16_Elect_Pos_With_Phase_Adv) = 0;
         BGVAR(s16_Motor_Setup_Current) = s16_current_value;
         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TEST_SP;
      break;

      case MOTOR_SETUP_TEST_SP:
         do {
            s16_dummy = Cntr_3125;
            s64_second_position  = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
         } while (s16_dummy != Cntr_3125);
         s16_small_pitch_dir_test = 0; // set 1 if dir is positive
         if (s64_second_position > s64_first_position) s16_small_pitch_dir_test = 1;
          // set 1 if dir is positive, else 0
         if(((abs(s16_current_value << 2)) < BGVAR(s16_Max_Current)))
         {
            s16_current_value+= s16_current_delta;
            u32_iterations++;
            if (u32_iterations == s16_movement_counter)
            {
               u32_iterations = 0;
               s16_movement_counter++;
               s16_current_delta = -s16_current_delta;
            }
            BGVAR(s16_Motor_Setup_Current) = abs(s16_current_value);
         }
         else
         {
            BGVAR(s16_Motor_Setup_Current) = 0;
            s16_movement_counter = 0;
            s16_current_value = BGVAR(s16_Max_Current) >> 6;
            if (!s16_current_value) s16_current_value = BGVAR(s16_Max_Current) >> 4;
            if (!s16_current_value) s16_current_value = BGVAR(s16_Max_Current) >> 2;
            do {
               s16_dummy = Cntr_3125;
               s64_first_position  = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
            } while (s16_dummy != Cntr_3125);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_FORCE_BUILD;
         }
      break;

      // Move back and forth with forced cummutation with increasing current till MIN(MICONT, DICONT)
      case MOTOR_SETUP_FORCE_BUILD:
         // In "small pitch" mode, control direction to avoid overall movement
         if (BGVAR(u16_Motor_Setup_Small_Pitch) && (s16_small_pitch_dir_test >= 0))
         {
            do {
               s16_dummy = Cntr_3125;
               s64_second_position  = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
            } while (s16_dummy != Cntr_3125);

             //CHeck motion direction vs prev. motion direction, if motion continues to be same - invert current
             if ((((s64_second_position > s64_first_position) && s16_small_pitch_dir_test)   ||
                  ((s64_second_position < s64_first_position) && !s16_small_pitch_dir_test)) &&
                  ((llabs(s64_second_position - s64_first_position)) >= 10000000LL)) s16_current_value = -s16_current_value;
             if ((llabs(s64_second_position - s64_first_position)) >= 10000000LL)   s16_small_pitch_dir_test = -1; // Do it until 1st few counts
         }
         BGVAR(s16_Motor_Setup_Current) = s16_current_value;
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;
         u32_iterations++;

         if (BGVAR(u16_MotorType) == LINEAR_MOTOR) u32_temp = 32768L;
         if (u32_iterations >= (unsigned long)(u32_temp / abs(s16_commutation_delta)))
         {
            s16_current_value <<= 1;
            u32_iterations = 0;
            s16_commutation_delta = -s16_commutation_delta;

            if (abs(s16_current_value << 1) > BGVAR(s16_Max_Current))
            {
               // Make sure last movement is in negative direction to shorten the overall movement distance
               if (s16_commutation_delta < 0) break;
               if (BGVAR(u16_Motor_Setup_Small_Pitch) && (s16_current_value < 0))
                  VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += 32767;
               s16_current_value = BGVAR(s16_Max_Current) >> 2;
               BGVAR(s16_Motor_Setup_Current) = s16_current_value;
               s16_commutation_delta = abs(s16_commutation_delta);
               BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_GET_FIRST_ENDAT_POSIT;//MOTOR_SETUP_DELAY_AND_CHECK_CURRENTS;
               // Allow one iteration only
               if (u16_skip_endat_fix == 1) BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_DELAY_AND_CHECK_CURRENTS;
            }
         }
      break;

      case MOTOR_SETUP_GET_FIRST_ENDAT_POSIT:
         if (BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX] == 9)
         {
            // Workaround for Endat S.Enc. with inverted sine/cosine signals.
            if (!PassedTimeMS(1L, BGVAR(s32_Motor_Setup_Timer))) break;
            return_val = SalHwPosCommand(&s64_first_position_endat_endat_fix, drive); // Get first position by Endat
            do {
               s16_dummy = Cntr_3125;
               s64_first_position_fdbk_endat_fix = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
            } while (s16_dummy != Cntr_3125);
            if (return_val == SAL_NOT_FINISHED) BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
            else if (return_val == SAL_SUCCESS) BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_MOVE_FORTH; // Do forth movement
         }
         else
         {
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_DELAY_AND_CHECK_CURRENTS;
         }
      break;

      case MOTOR_SETUP_MOVE_FORTH:
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;
         u32_iterations++;
         if(u32_iterations == 100) BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_GET_SECOND_ENDAT_POSIT;
      break;

      case MOTOR_SETUP_GET_SECOND_ENDAT_POSIT:
         if (!PassedTimeMS(1L, BGVAR(s32_Motor_Setup_Timer))) break;
         return_val = SalHwPosCommand(&s64_second_position_endat_endat_fix, drive); // Get second position by Endat
         do {
            s16_dummy = Cntr_3125;
            s64_second_position_fdbk_endat_fix = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
         } while (s16_dummy != Cntr_3125);
         if (return_val == SAL_NOT_FINISHED) BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
         else if (return_val == SAL_SUCCESS) BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_MOVE_BACK; // Do back movement
      break;

      case MOTOR_SETUP_MOVE_BACK:
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) -= s16_commutation_delta;
         u32_iterations--;
         if(u32_iterations == 0) BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_ENDAT_DIR;
      break;

      case MOTOR_SETUP_ENDAT_DIR:
         s16_endat_dir_endat_fix = 1;
         s16_fdbk_dir_endat_fix = 1;
         // Set -1 if first position is bigger than second
         if (s64_first_position_endat_endat_fix > s64_second_position_endat_endat_fix) s16_endat_dir_endat_fix = -1;
         if (s64_first_position_fdbk_endat_fix > s64_second_position_fdbk_endat_fix) s16_fdbk_dir_endat_fix = -1;
         // Compare directions of position received from Endat and Sin/Cos
         if (s16_endat_dir_endat_fix != s16_fdbk_dir_endat_fix) u16_sincos_direction_endat_fix = 1;
         // Continue regular MotorSetup
         s16_current_value = BGVAR(s16_Max_Current) >> 2;
         BGVAR(s16_Motor_Setup_Current) = s16_current_value;
         s16_commutation_delta = abs(s16_commutation_delta);
         BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_DELAY_AND_CHECK_CURRENTS;
         // If directions of positions are different, then update MFBDIR to 8 and repeat motorsetup
         if (u16_sincos_direction_endat_fix == 1)
         {
            DisableCommand(drive);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_ENDAT_DIR_DISABLE;
         }
      break;

      case MOTOR_SETUP_ENDAT_DIR_DISABLE:
         if (!Enabled(DRIVE_PARAM))
         {
            u16_skip_endat_fix = 1; // Skip Endat-Sin/Cos direction test
            SalMotorFeedbackDirectionCommand(8LL, drive);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_ENDAT_DIR_CONFIG;
         }
      break;

      case MOTOR_SETUP_ENDAT_DIR_CONFIG:
         if (SalConfigCommand(drive) == SAL_NOT_FINISHED) break;
         if ( (FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_ERROR_STATE) ) break;
         if ( (FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) == COMM_FDBK_ERROR_STATE) )
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_COMM_FDBK_ERROR);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
         if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(u16_Read_Mtp_State) != MTP_INIT_INIT) ) break;
         if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(s16_MTP_Error) != 0) )
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ERROR);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
         LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &BGVAR(s16_Motor_Setup_Current);
         EnableCommand(drive);
         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_ENDAT_DIR_ENABLE;
      break;

      case MOTOR_SETUP_ENDAT_DIR_ENABLE:
         if (Enabled(DRIVE_PARAM))
         {
            s16_current_value = s16_init_current_value_endat_fix;
            u32_iterations = 0;
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_FORCE_BUILD;
         }
      break;

      // Wait for 1 sec, then check if currents applied are close to what is injected
      case MOTOR_SETUP_DELAY_AND_CHECK_CURRENTS:
         if (PassedTimeMS(1000L, BGVAR(s32_Motor_Setup_Timer)))
         {
            s16_iu = abs(AX0_s16_Crrnt_Srvo_Cmnd_U_Act_0);
            s16_iv = abs(AX0_s16_Crrnt_Srvo_Cmnd_V_Act_0);

            if ((abs(s16_iu) < (s16_current_value >> 1)) && (abs(s16_iv) < (s16_current_value >> 1)))
            {
               BGVAR(u32_Motor_Setup_Status) &= ~MOTOR_SETUP_ESTMOTORPARAM_ERR; // If phases not connected ignore the ML&MR detection failure
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MISSING_PHASE);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
               break;
            }
            if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_ESTMOTORPARAM_ERR)
            {
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
               break;
            }

            s16_current_value = BGVAR(s16_Max_Current);
            BGVAR(s16_Motor_Setup_Current) = s16_current_value;
            if (s16_direction_test == -1)
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_CHECK_DIRECTION;
            else
            {
               if (FEEDBACK_RESOLVER)
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_RESOLVER_SININIT_SETUP;
               else if (FEEDBACK_WITH_HALLS)
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_HALLS_NEG_SWEEP_INIT;
               else if (FEEDBACK_AB_ONLY)
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND_SETUP;
               else if (FEEDBACK_ABS_INTURN)
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_STEGMANN_INIT;
               else
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_NEG_SWEEP_INIT;
            }
         }
      break;

      // Check the direction of movement and (A/B)-I swap
      case MOTOR_SETUP_CHECK_DIRECTION:
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;
         u32_iterations++;

         if (FEEDBACK_WITH_INDEX && (!BGVAR(u16_Motor_Setup_Small_Pitch)))
         {
            if (u32_iterations == 50)
            {
               s64_first_position = (long long)LVAR(AX0_u32_Pos_Fdbk_Lo);
               index_counter = 0;
               VAR(AX0_s16_Crrnt_Run_Code) &= ~MOTOR_SETUP_INDEX_CAPTURED_MASK;
               VAR(AX0_s16_Crrnt_Run_Code) |= MOTOR_SETUP_LOOK_FOR_INDEX_MASK;
            }
            if (u32_iterations == 150)
            {
               if (VAR(AX0_s16_Crrnt_Run_Code) & MOTOR_SETUP_INDEX_CAPTURED_MASK)
               {
                  index_counter++;
                  VAR(AX0_s16_Crrnt_Run_Code) &= ~MOTOR_SETUP_INDEX_CAPTURED_MASK;
                  VAR(AX0_s16_Crrnt_Run_Code) |= MOTOR_SETUP_LOOK_FOR_INDEX_MASK;
               }
            }
            if (u32_iterations == 400)
            {
               if (VAR(AX0_s16_Crrnt_Run_Code) & MOTOR_SETUP_INDEX_CAPTURED_MASK) index_counter++;
               if (index_counter > 1)
               {
                  BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_AB_I_SWAP);
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
               }
            }
         }
         else // No Index
         {
            if ( (u32_iterations == 50)                                        ||
                 ((u32_iterations == 10) && BGVAR(u16_Motor_Setup_Small_Pitch))  )
               s64_first_position = (long long)LVAR(AX0_u32_Pos_Fdbk_Lo);
         }

         if ( (u32_iterations == 250)                                        ||
              ((u32_iterations == 100) && BGVAR(u16_Motor_Setup_Small_Pitch))  )
         {
            s64_second_position = (long long)LVAR(AX0_u32_Pos_Fdbk_Lo);
            if (llabs(s64_second_position - s64_first_position) < 1000)
            {
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_NO_MOVEMENT);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            }
         }

         if ( (u32_iterations == 350)                                        ||
              ((u32_iterations == 200) && BGVAR(u16_Motor_Setup_Small_Pitch))  )
         {
            s64_second_position = (long long)LVAR(AX0_u32_Pos_Fdbk_Lo);
            if ( ( (s64_second_position < 0x80000000) &&
                   (s64_first_position >= 0x80000000)   ) ||
                 ( (s64_second_position >= 0x80000000) &&
                   (s64_first_position < 0x80000000)     )  )
            {
               s64_first_position = s64_second_position;
            }
            else
            {
               if (s64_second_position > s64_first_position) s16_direction_test = 0;
               else s16_direction_test = 1;
            }
         }
         if ( ((u32_iterations == 500) && (s16_direction_test == -1))    ||
              ( (u32_iterations == 300) && (s16_direction_test == -1) &&
                 BGVAR(u16_Motor_Setup_Small_Pitch)                     )  )
         {
            s64_second_position = (long long)LVAR(AX0_u32_Pos_Fdbk_Lo);
            if (s64_second_position > s64_first_position) s16_direction_test = 0;
            else s16_direction_test = 1;
         }

         if (s16_direction_test == 1)
         {
            DisableCommand(drive);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_CHANGE_DIRECTION;
         }
         else if (s16_direction_test == 0)
         {
            BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
            if (FEEDBACK_RESOLVER)
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_RESOLVER_SININIT_SETUP;
            else if (FEEDBACK_WITH_HALLS)
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_HALLS_NEG_SWEEP_INIT;
            else if (FEEDBACK_AB_ONLY)
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND_SETUP;
            else if (FEEDBACK_ABS_INTURN)
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_STEGMANN_INIT;
            else
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_NEG_SWEEP_INIT;
         }
      break;

      case MOTOR_SETUP_CHANGE_DIRECTION: // If needed swap the phases direction
         if (Enabled(DRIVE_PARAM)) break;

         u16_mfbdir_build = 0;
         if (s16_direction_test == 1)
            u16_mfbdir_build |= 0x01;
         if (s16_halls_direction == 1)
            u16_mfbdir_build |= 0x02;
         if (u16_index_polarity == 1)
            u16_mfbdir_build |= 0x04;
         if (u16_sincos_direction_endat_fix == 1)
            u16_mfbdir_build |= 0x08;
         if (BGVAR(u16_Motor_Feedback_Direction) != u16_mfbdir_build)
         {
            BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MFBDIR_ADJUST;
            if ( (BGVAR(u16_MTP_Mode) != 1) && (BGVAR(u16_MTP_Mode) != 3) )
               SalMotorFeedbackDirectionCommand((long long)u16_mfbdir_build, drive);
            else
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ACTIVE);
         }
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) = 0;

         if (SalConfigCommand(drive) != SAL_NOT_FINISHED)
         {
            LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &BGVAR(s16_Motor_Setup_Current);

            // This is to allow enable without phase find
            if ((BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX] >= 1) && (BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX] <= 4))
            {
               BGVAR(s64_Faults_Mask) &= ~PHASE_FIND_FLT_MASK;
               BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;
               VAR(AX0_s16_PhaseFindRTBits)= 0;
               VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_INC_ENC & 0xffff);
            }
            if ( (FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_ERROR_STATE) ) break;
            if ( (FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) == COMM_FDBK_ERROR_STATE) )
            {
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_COMM_FDBK_ERROR);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            }
            if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(u16_Read_Mtp_State) != MTP_INIT_INIT) ) break;
            if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(s16_MTP_Error) != 0) )
            {
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ERROR);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            }
            EnableCommand(drive);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_CHANGE_DIRECTION_ENABLE;
            BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
         }
      break;

      // Enable the drive and repeat the force build up (this time with U,V swapped)
      case MOTOR_SETUP_CHANGE_DIRECTION_ENABLE:
         if (!PassedTimeMS(100L, BGVAR(s32_Motor_Setup_Timer))) break;
         if (!Enabled(DRIVE_PARAM)) break;

         u32_iterations = 0;
         s16_current_value = BGVAR(s16_Max_Current) >> 8;
         if (!s16_current_value) s16_current_value = BGVAR(s16_Max_Current) >> 4;

         BGVAR(s16_Motor_Setup_Current) = s16_current_value;
         LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &BGVAR(s16_Motor_Setup_Current);
         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_FORCE_BUILD;
         if(BGVAR(u16_Motor_Setup_Small_Pitch)) BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_PRE_TEST_SP;
      break;

      // Move back & forth till SININIT is done
      case MOTOR_SETUP_RESOLVER_SININIT_SETUP:
         //SininitCommand(drive);
         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_RESOLVER_SININIT_RUN;
      break;

      case MOTOR_SETUP_RESOLVER_SININIT_RUN:
         //SalReadSinInitStatusCommand(&s64_temp, drive);
         if (s64_temp == 0LL)
         {
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_RESOLVER_MPHASE_MPOLES_DETECT;
            s32_start_position = LVAR(AX0_u32_Pos_Fdbk_Lo);
            s32_start_position_hi = LVAR(AX0_s32_Pos_Fdbk_Hi);
            s32_last_pos = s32_start_position;
            u16_pole_pairs_count = 0;
            u32_iterations = 0;
            s16_commutation_delta = s16_commutation_delta >> 2;
         }
      break;

      // Check when the feedback crosses 0x7FFF
      case MOTOR_SETUP_RESOLVER_MPHASE_MPOLES_DETECT:
         s32_temp = LVAR(AX0_u32_Pos_Fdbk_Lo);
         if (((s32_temp > 0L) != (s32_last_pos > 0L)) && (!u16_resolver_mphase_set))
         {
            u16_resolver_mphase_set = 1;
            u16_mphase_calc = VAR(AX0_s16_Elect_Pos_With_Phase_Adv) + DEG_90_OFFSET;
            if (VAR(AX0_S16_Kc_Mode) > 3) u16_mphase_calc += DEG_30_OFFSET; // Used to compensate for the 30deg offset in old Iuvw-Idq transformation

            // Round MPHASE to 10 degrees
            u16_mphase_calc = ((int)(u16_mphase_calc + 910) / 1820) * 1820;
            if (abs((int)((int)BGVAR(s32_ParamsBackup)[MPHASE_STORE_INDEX] / 182) - ((int)u16_mphase_calc / 182)) > 10)
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MPHASE_ADJUST;
         }
         else s32_last_pos = s32_temp;

         // Count # of electrical cycle to count MPOLES value
         if ((VAR(AX0_s16_Elect_Pos_With_Phase_Adv) < 0x8000) && ((VAR(AX0_s16_Elect_Pos_With_Phase_Adv) + s16_commutation_delta) >= 0x8000))
            u16_pole_pairs_count++;

         // Wait for full rev to complete
         if ((LVAR(AX0_s32_Pos_Fdbk_Hi) > s32_start_position_hi) && (LVAR(AX0_u32_Pos_Fdbk_Lo) > s32_start_position))
         {  // Adjust MPOLES if needed
            if (((u16_pole_pairs_count << 1) != (int)BGVAR(s32_ParamsBackup)[MPOLES_STORE_INDEX]) && (u16_pole_pairs_count > 0))
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MPOLES_ADJUST;

            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND_SETUP;
         }

         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;
         u32_iterations++;
      break;

      case MOTOR_SETUP_HALLS_NEG_SWEEP_INIT:   // Init sweep in negative direction (Halls)
         if (PassedTimeMS(1000L, BGVAR(s32_Motor_Setup_Timer)))
         {
            if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
               s16_commutation_delta = -20;
            else
               s16_commutation_delta = -30;

            u32_iterations = 0;
            u16_halls_index = 1;
            if (ReadHallsFromFeedback(&BGVAR(s16_Halls_Neg)[0],drive)) return;
            // Store starting position to check if there is any movement
            s32_start_position = LVAR(AX0_u32_Pos_Fdbk_Lo);
            s32_last_pos = LVAR(AX0_u32_Pos_Fdbk_Lo);
            s16_movement_counter = 0;
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_HALLS_NEG_SWEEP_RUN;
         }
      break;

      // Detect all halls switches and (only if there is no index) move additional 3 electrical revolutions
      case MOTOR_SETUP_HALLS_NEG_SWEEP_RUN: // (total of 4) to calculate MENCRES/MPOLES ratio
         if (FEEDBACK_HALLS_ONLY)
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND_SETUP;
         // Accumulate delta position
         if (HwPosCommand(&s64_hwpos, &u16_hwpos_num_of_bits, drive) == SAL_NOT_FINISHED)  break;
         s64_temp = s64_hwpos - s64_first_position;
         s64_first_position = s64_hwpos;

         // Differ between the sizes of the HWPOS for the roll-over handling
         s64_rollover_value = 1LL << (long long)(u16_hwpos_num_of_bits - 1);
         if (s64_temp < -s64_rollover_value) s64_temp += (s64_rollover_value << 1LL);
         else if (s64_temp > s64_rollover_value) s64_temp -= (s64_rollover_value << 1LL);

         s64_delta_position += s64_temp;

         hall_index_num = 7;
         if (BGVAR(u16_Motor_Setup_Small_Pitch)) hall_index_num = 2;
         if (u16_halls_index < hall_index_num)
         {
            if (ReadHallsFromFeedback(&BGVAR(s16_Halls_Neg)[u16_halls_index], drive)) return;
            if (BGVAR(s16_Halls_Neg)[u16_halls_index] != BGVAR(s16_Halls_Neg)[u16_halls_index - 1])
            {
               if (s16_halls_filter < 0)
               {
                  if (u16_halls_index == 1)
                  {
                     if (SalHwPosCommand(&s64_first_position, drive) == SAL_NOT_FINISHED)  break;
                     s64_delta_position = 0LL;
                  }

                  u16_electangle_neg[u16_halls_index - 1] = VAR(AX0_s16_Elect_Pos_With_Phase_Adv);
               }

               s16_halls_filter++;
               if (s16_halls_filter >= 2)
               {
                  s16_halls_filter = -1;
                  u16_halls_index++;
                  if ((u16_halls_index == 7) && (FEEDBACK_WITH_INDEX)) // All halls have been read
                     BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_HALLS_NEG_SWEEP_RUN_END;
                  else s16_prev_halls = BGVAR(s16_Halls_Neg)[6];
               }
            }
         }
         else
         {
            if (ReadHallsFromFeedback(&s16_current_halls, drive)) return;
            if (s16_current_halls != s16_prev_halls)
            {
               s16_halls_filter++;
               if (s16_halls_filter >= 2)
               {
                  s16_halls_filter = -1;
                  s16_prev_halls = s16_current_halls;
                  u16_halls_index++;
               }
            }

            if ((u16_halls_index > 25) || ((BGVAR(u16_MotorType) == LINEAR_MOTOR) && (u16_halls_index > hall_index_num)))
            {
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_HALLS_NEG_SWEEP_RUN_END;
            }
         }
         // If not all 6 halls have been read for 4.5 electrical revs (1.5 rev for Linear motor) - fault
         if ( (u32_iterations >= (294912L / (long)abs(s16_commutation_delta)))                                           ||
              ((u32_iterations >= (98304L / (long)abs(s16_commutation_delta))) && (BGVAR(u16_MotorType) == LINEAR_MOTOR))  )
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MISSING_HALLS);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }

         // Test for no movement (probably due to hard stop)
         s16_movement_counter++;
         if (s16_movement_counter > 100)
         {
            s16_movement_counter = 0;

            if (s32_last_pos == LVAR(AX0_u32_Pos_Fdbk_Lo))
            {
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_HARD_STOP);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            }
            s32_last_pos = LVAR(AX0_u32_Pos_Fdbk_Lo);
         }
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;
         u32_iterations++;
      break;

      case MOTOR_SETUP_HALLS_NEG_SWEEP_RUN_END:    // Check validity of halls
         i = 0;
         halls_fault = 0;
         if(!BGVAR(u16_Motor_Setup_Small_Pitch))
         {
            while ((i < 6) && (BGVAR(s16_Halls_Neg)[i] != 4)) i++;
            if (i == 6) halls_fault = 1;
            else
            {  // Check the sequence 4->5->1->3->2->6
               i_next = pos_modulu((i + 1), 6);
               i_prev = pos_modulu((i - 1), 6);
               if ((BGVAR(s16_Halls_Neg)[i_next] != 5) && (BGVAR(s16_Halls_Neg)[i_prev] != 5)) halls_fault = 1;
               else
               {
                  if (BGVAR(s16_Halls_Neg)[i_next] == 5)
                  {
                     if ( (BGVAR(s16_Halls_Neg)[pos_modulu((i_next + 1), 6)] != 1) || (BGVAR(s16_Halls_Neg)[pos_modulu((i_next + 2), 6)] != 3) ||
                          (BGVAR(s16_Halls_Neg)[pos_modulu((i_next + 3), 6)] != 2) || (BGVAR(s16_Halls_Neg)[pos_modulu((i_next + 4), 6)] != 6)   ) halls_fault = 1;
                  }
                  else
                  {
                     if ( (BGVAR(s16_Halls_Neg)[pos_modulu((i_prev - 1), 6)] != 1) || (BGVAR(s16_Halls_Neg)[pos_modulu((i_prev - 2), 6)] != 3) ||
                          (BGVAR(s16_Halls_Neg)[pos_modulu((i_prev - 3), 6)] != 2) || (BGVAR(s16_Halls_Neg)[pos_modulu((i_prev - 4), 6)] != 6)   ) halls_fault = 1;
                  }
               }
            }
         }
         s16_halls_direction = 0;
         if (HallsDirection(BGVAR(s16_Halls_Neg)[1], BGVAR(s16_Halls_Neg)[0], &s16_halls_table_index, &s16_halls_direction)) halls_fault = 1;

         if (halls_fault)
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MISSING_HALLS);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
         else
         {  // Detect MPHASE
            u16_mphase_calc = u16_electangle_neg[0] - (int)(*((int*)(&Enc_Hall_Switch_Tbl_4to5to1to3to2to6) + s16_halls_table_index));
            if (s16_halls_direction == 1)
               u16_mphase_calc = u16_electangle_neg[0] - (int)(*((int*)(&Enc_Hall_Switch_Tbl_4to6to2to3to1to5) + s16_halls_table_index));
            u16_mphase_calc += DEG_90_OFFSET;
            if (VAR(AX0_S16_Kc_Mode) > 3) u16_mphase_calc += DEG_30_OFFSET; // Used to compensate for the 30deg offset in old Iuvw-Idq transformation

            // Round MPHASE to 10 degrees
            u16_mphase_calc = ((int)(u16_mphase_calc + 910) / 1820) * 1820;
            if (abs((int)((int)BGVAR(s32_ParamsBackup)[MPHASE_STORE_INDEX] / 182) - ((int)u16_mphase_calc / 182)) > 10)
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MPHASE_ADJUST;

            if (BGVAR(u16_Motor_Setup_Small_Pitch))
            {
                BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND_SETUP;
                break;
            }

            if (FEEDBACK_WITH_INDEX)
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_NEG_SWEEP_INIT;
            else
            {  // Check if MENCRES/MPOLES is correct
               // If not try to fix MPOLES according to MENCRES, otherwise terminate with fault
               s64_mencres_per_pole = (long long)(BGVAR(s32_ParamsBackup)[MENCRES_STORE_INDEX] / (BGVAR(s32_ParamsBackup)[MPOLES_STORE_INDEX] >> 1L));
               s64_delta_position = llabs(s64_delta_position);

               if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
                  u64_div_factor = 2LL;
               else
                  u64_div_factor = 4LL;

               if (llabs((s64_delta_position >> u64_div_factor) - s64_mencres_per_pole) > (s64_mencres_per_pole >> 6LL))
               {
                  mpoles_fault = 1;
                  u16_pole_pairs_count = 0;
                  while ((u16_pole_pairs_count < 100) && (mpoles_fault))
                  {
                     u16_pole_pairs_count++;
                     s64_mencres_per_pole = (long long)(BGVAR(s32_ParamsBackup)[MENCRES_STORE_INDEX] / (long)u16_pole_pairs_count);
                     if (llabs((s64_delta_position >> u64_div_factor) - s64_mencres_per_pole) <= (s64_mencres_per_pole >> 7LL))
                     {
                        mpoles_fault = 0;
                        // Adjust MPOLES if needed
                        if ((u16_pole_pairs_count << 1) != (int)BGVAR(s32_ParamsBackup)[MPOLES_STORE_INDEX])
                           BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MPOLES_ADJUST;
                     }
                  }
                  if (mpoles_fault)
                  {
                     BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MENCRES_PER_MPOLES);
                     BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
                  }
                  else BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND_SETUP;
               }
               else
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND_SETUP;
            }
         }
      break;

      case MOTOR_SETUP_STEGMANN_INIT:      // Read the MENCRES value from the Feedback
         if (BGVAR(u16_MotorType) == LINEAR_MOTOR)
            s16_commutation_delta = -20;
         else
            s16_commutation_delta = -30;

         u32_iterations = 0;
         s16_movement_counter = 0;
         s32_last_pos = LVAR(AX0_u32_Pos_Fdbk_Lo);
         u16_pole_pairs_count = 1;
         u16_pole_pairs_count_first = 1;
         s64_delta_position = 0LL;
         u32_calculated_mencres = BGVAR(s32_ParamsBackup)[MENCRES_STORE_INDEX];

         if (FEEDBACK_ABS_INTURN_EXCL_STEGMANN)
         {
            if (!PassedTimeMS(1L, BGVAR(s32_Motor_Setup_Timer))) break;
            return_val = SalHwPosCommand(&s64_first_position, drive);
            if (return_val == SAL_NOT_FINISHED)
            {
               BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
               break;
            }
            else if (return_val != SAL_SUCCESS)
            {
               if ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_HWPOS_1ST_TRY) == 0)
               {
                  BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_HWPOS_1ST_TRY;
                  break;
               }
               else
               {
                  BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_STEGMANN_COMM_ERR);
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
                  break;
               }
            }
            BGVAR(u32_Motor_Setup_Status) &= ~MOTOR_SETUP_HWPOS_1ST_TRY;
         }
         else
            s64_first_position = (long long)VAR(AX0_s16_Qep);

         BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
         // Read MENCRES from encoder (or pre-set where known)
/*         if (FEEDBACK_YASKAWA) u32_calculated_mencres = 262144L;
         else */
         if (FEEDBACK_PANASONIC_S || FEEDBACK_SANKYO || FEEDBACK_NIKON)
            u32_calculated_mencres = 32768L; // 17 bit feedbacks, although SANKYO is 20bits we are using only 17;
            // Nikon allowing only 17-Bit Resolution devices; Panasonic-S limited to 17-Bit devices.
         else if (FEEDBACK_TAMAGAWA_ABS) // Read MENCRES from Encoder-ID Data-Field
            u32_calculated_mencres = (1LL << ((VAR(AX0_u16_Abs_Enc_Data_1Frame) >> 8) - 2));
         else if (FEEDBACK_SERVOSENSE) u32_calculated_mencres = 1048576L;
         else if (FEEDBACK_ENDAT)
         {
            u32_comm_only_mencres = BGVAR(u32_EnDat_Digital_Step);
            u32_calculated_mencres = BGVAR(u32_EnDat_Enc_Res);
         }
         else if (FEEDBACK_STEGMANN)
            u32_calculated_mencres = BGVAR(u32_Hiface_Resolution);

         if (u32_calculated_mencres <= 0)
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_STEGMANN_COMM_ERR);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
         else
         {
            if (u32_calculated_mencres != BGVAR(s32_ParamsBackup)[MENCRES_STORE_INDEX])
            {
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MENCRES_ADJUST;
               // Set correct MENCRES so PFB can be used to set commutation angle
               DisableCommand(drive);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_STEGMANN_CONFIG;
            }
            else BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_STEGMANN_SWEEP_RUN;
         }
      break;

      case MOTOR_SETUP_STEGMANN_CONFIG:      // Set the MENCRES as read from the feedback
         if (Enabled(DRIVE_PARAM)) break;

         SalMotorEncResCommand((long long)u32_calculated_mencres, drive);
         if (SalConfigCommand(drive) == SAL_NOT_FINISHED) break;
         if ( (FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_ERROR_STATE)) break;
         if ( (FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) == COMM_FDBK_ERROR_STATE) )
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_COMM_FDBK_ERROR);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
         if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(u16_Read_Mtp_State) != MTP_INIT_INIT) ) break;
         if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(s16_MTP_Error) != 0) )
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ERROR);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }

         LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &BGVAR(s16_Motor_Setup_Current);
         EnableCommand(drive);
         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_STEGMANN_WAIT_FOR_ENABLE;
         BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
      break;

      case MOTOR_SETUP_STEGMANN_WAIT_FOR_ENABLE:
         if (!PassedTimeMS(50L, BGVAR(s32_Motor_Setup_Timer))) break;
         if (Enabled(drive))
         {
            BGVAR(s16_Motor_Setup_Current) = BGVAR(s16_Max_Current);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_STEGMANN_SWEEP_RUN;
         }
      break;

      case MOTOR_SETUP_STEGMANN_SWEEP_RUN:    // Turn a full revolution to detect the MPOLES value
         if (FEEDBACK_ABS_INTURN_EXCL_STEGMANN)
         {
            if (!PassedTimeMS(1L, BGVAR(s32_Motor_Setup_Timer))) break;
            return_val = HwPosCommand(&s64_hwpos, &u16_hwpos_num_of_bits, drive);
            if (return_val == SAL_NOT_FINISHED)
            {
               BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
               break;
            }
            else if (return_val != SAL_SUCCESS) // If HWPOS failure...
            {
               if ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_HWPOS_1ST_TRY) == 0)
               { // Allow one more try of HWPOS.
                  BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_HWPOS_1ST_TRY;
                  break;
               }
               else
               {
                  BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_STEGMANN_COMM_ERR);
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
                  break;
               }
            }
            BGVAR(u32_Motor_Setup_Status) &= ~MOTOR_SETUP_HWPOS_1ST_TRY;
         }
         else
            s64_hwpos = (long long)VAR(AX0_s16_Qep);

         s64_temp = s64_hwpos - s64_first_position;         // Accumulate position
         // Handle roll-over in HWPOS
         if (FEEDBACK_ENDAT)
            s64_rollover_value = (unsigned long long)((u32_comm_only_mencres << 1) * (1L << VAR(AX0_u16_ED_Turns_Bits)));
         else
            s64_rollover_value = 1LL << (long long)(u16_hwpos_num_of_bits - 1);

         if (s64_temp < -s64_rollover_value) s64_temp += (s64_rollover_value << 1LL);
         else if (s64_temp > s64_rollover_value) s64_temp -= (s64_rollover_value << 1LL);

         s64_delta_position += s64_temp;
         s64_first_position = s64_hwpos;

         // Count # of electrical cycle to count MPOLES value
         if ((VAR(AX0_s16_Elect_Pos_With_Phase_Adv) > (unsigned int)0x8000) && ((VAR(AX0_s16_Elect_Pos_With_Phase_Adv) + s16_commutation_delta) <= (unsigned int)0x8000))
         {
            if (!u16_pole_pairs_count_first) u16_pole_pairs_count++;
            u16_pole_pairs_count_first = 0;
         }

         u32_iterations++;
         s16_movement_counter++;

         // Wait for Full motor revolution
         // for EnDat with Sine, HwPos response does NOT match Encoder Resolution, use the
         // Digital Word Resolution.
         if ( (FEEDBACK_ENDAT && (llabs(s64_delta_position) > (unsigned long long)(u32_comm_only_mencres << 2L)))  ||
              (!FEEDBACK_ENDAT && (llabs(s64_delta_position) > (unsigned long long)(u32_calculated_mencres << 2L)))  )
         {
            if (u16_pole_pairs_count != (BGVAR(s32_ParamsBackup)[MPOLES_STORE_INDEX] >> 1L))
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MPOLES_ADJUST;
            s16_movement_counter = 0;
            s64_delta_position = 0LL;
            s16_commutation_delta = -s16_commutation_delta;
            u16_mphase_calc = VAR(AX0_s16_Elect_Pos_With_Phase_Adv) - (int)((int)(LVAR(AX0_u32_Pos_Fdbk_Lo) >> 16L) * u16_pole_pairs_count);

            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_STEGMANN_SWEEP_NEG_RUN;
            break;
         }

         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;

         if (s16_movement_counter > 100)     // Test for no movement (probably due to hard stop)
         {
            s16_movement_counter = 0;

            if (s32_last_pos == LVAR(AX0_u32_Pos_Fdbk_Lo))
            {
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_HARD_STOP);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            }
            s32_last_pos = LVAR(AX0_u32_Pos_Fdbk_Lo);
         }
      break;

      // Move back quarter Revolution to average the MPHASE value
      case MOTOR_SETUP_STEGMANN_SWEEP_NEG_RUN:
         if (FEEDBACK_ABS_INTURN_EXCL_STEGMANN)
         {
            if (!PassedTimeMS(1L, BGVAR(s32_Motor_Setup_Timer))) break;
            return_val = HwPosCommand(&s64_hwpos, &u16_hwpos_num_of_bits, drive);
            if (return_val == SAL_NOT_FINISHED)
            {
               BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
               break;
            }
            else if (return_val != SAL_SUCCESS) // If failure of HWPOS...
            {
               if ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_HWPOS_1ST_TRY) == 0)
               { // Allow one more try after an error
                  BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_HWPOS_1ST_TRY;
                  break;
               }
               else
               {
                  BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_STEGMANN_COMM_ERR);
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
                  break;
               }
            }
            BGVAR(u32_Motor_Setup_Status) &= ~MOTOR_SETUP_HWPOS_1ST_TRY;
         }
         else
            s64_hwpos = (long long)VAR(AX0_s16_Qep);

         s64_temp = s64_hwpos - s64_first_position;         // Accumulate position
         // Handle roll-over in HWPOS
         if (FEEDBACK_ENDAT)
            s64_rollover_value = (unsigned long long)((u32_comm_only_mencres << 1) * (1L << VAR(AX0_u16_ED_Turns_Bits)));
         else
            s64_rollover_value = 1LL << (long long)(u16_hwpos_num_of_bits - 1);

         if (s64_temp < -s64_rollover_value) s64_temp += (s64_rollover_value << 1LL);
         else if (s64_temp > s64_rollover_value) s64_temp -= (s64_rollover_value << 1LL);

         s64_delta_position += llabs(s64_temp);
         s64_first_position = s64_hwpos;

         u32_iterations++;
         s16_movement_counter++;

         // Wait for Quarter motor revolution
         // for EnDat with Sine, HwPos response does NOT match Encoder Resolution, use the
         // Digital Word Resolution.
         if ( (FEEDBACK_ENDAT && (llabs(s64_delta_position) > (unsigned long long)u32_comm_only_mencres))  ||
              (!FEEDBACK_ENDAT && (llabs(s64_delta_position) > (unsigned long long)u32_calculated_mencres))  )
         {
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_STEGMANN_SWEEP_END;
            break;
         }

         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;

         if (s16_movement_counter > 100)   // Test for no movement (probably due to hard stop)
         {
            s16_movement_counter = 0;

            if (s32_last_pos == LVAR(AX0_u32_Pos_Fdbk_Lo))
            {
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_HARD_STOP);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            }
            s32_last_pos = LVAR(AX0_u32_Pos_Fdbk_Lo);
         }
      break;

      case MOTOR_SETUP_STEGMANN_SWEEP_END:      // Calculate the MPHASE value
         u16_mphase_calc_2 = u16_mphase_calc;
         u16_mphase_calc   = VAR(AX0_s16_Elect_Pos_With_Phase_Adv) - (int)((int)(LVAR(AX0_u32_Pos_Fdbk_Lo) >> 16L) * u16_pole_pairs_count);
         // Average mphase and handle sign roll-over
         u16_mphase_calc = (unsigned int)(((unsigned long)u16_mphase_calc + (unsigned long)u16_mphase_calc_2) >> 1L);
         if (abs(u16_mphase_calc - u16_mphase_calc_2) > 0x4000) u16_mphase_calc += 0x8000;

         u16_mphase_calc += DEG_90_OFFSET;
         if (VAR(AX0_S16_Kc_Mode) > 3) u16_mphase_calc += DEG_30_OFFSET; // Used to compensate for the 30deg offset in old Iuvw-Idq transformation

         // Round MPHASE to 10 degrees
         u16_mphase_calc = ((int)(u16_mphase_calc + 910) / 1820) * 1820;
         if (abs((int)((int)BGVAR(s32_ParamsBackup)[MPHASE_STORE_INDEX] / 182) - ((int)u16_mphase_calc / 182)) > 10)
            BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MPHASE_ADJUST;

         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND_SETUP;
      break;

      case MOTOR_SETUP_INDEX_NEG_SWEEP_INIT:      // Init sweep in negative direciton (Tamagawa)
         s16_commutation_delta = -40;
         u32_iterations = 0;
         index_counter = 0;

         VAR(AX0_s16_Crrnt_Run_Code) &= ~(MOTOR_SETUP_INDEX_CAPTURED_MASK | MOTOR_SETUP_LOOK_FOR_INDEX_MASK);

         // Store starting position to check if there is any movement
         s32_start_position = LVAR(AX0_u32_Pos_Fdbk_Lo);

         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_NEG_SWEEP_FIRST_RUN;
      break;

      case MOTOR_SETUP_INDEX_NEG_SWEEP_FIRST_RUN:      // Check index polarity (swap if needed)
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;
         u32_iterations++;

         if (u32_iterations == 100) u16_index_polarity += INDEX_SIGNAL;
         if (u32_iterations == 200) u16_index_polarity += INDEX_SIGNAL;
         if (u32_iterations == 300) u16_index_polarity += INDEX_SIGNAL;
         if (u32_iterations == 400) u16_index_polarity += INDEX_SIGNAL;
         if (u32_iterations == 500)
         {
            u16_index_polarity += INDEX_SIGNAL; // This is the index signal
            if (u16_index_polarity >= 3) u16_index_polarity = 1;
            else u16_index_polarity = 0;

            if (u16_index_polarity)
            {
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_INVERTED_INDEX);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            }
            if (u16_index_polarity) EQep1Regs.QDECCTL.bit.QIP = 1; // This will swap index polarity

            if (labs(LVAR(AX0_u32_Pos_Fdbk_Lo) - s32_start_position) < 1000)
            {
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_NO_MOVEMENT);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            }
            else
            {
               u32_iterations = 0;
               VAR(AX0_s16_Crrnt_Run_Code) &= ~MOTOR_SETUP_INDEX_CAPTURED_MASK;
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_NEG_SWEEP_INDEX_RUN;
            }
         }
      break;

      // Check if there is any movement and the direction of movement
      case MOTOR_SETUP_INDEX_NEG_SWEEP_INDEX_RUN:
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;
         u32_iterations++;

         if (u32_iterations == 500)
            VAR(AX0_s16_Crrnt_Run_Code) |= MOTOR_SETUP_LOOK_FOR_INDEX_MASK;

         if (VAR(AX0_s16_Crrnt_Run_Code) & MOTOR_SETUP_INDEX_CAPTURED_MASK)
         {
            VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += (s16_commutation_delta << 3);
            if (FEEDBACK_TAMAGAWA)
            {
               BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
//               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TAMAGAWA_READ_NEG_HALLS;
               s16_commutation_delta = 40;
               u32_iterations = 0;
               VAR(AX0_s16_Crrnt_Run_Code) &= ~MOTOR_SETUP_INDEX_CAPTURED_MASK;
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TM_POSITIVE_TEMP_INDEX_SEARCH;
            }
            else
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_POS_SWEEP_INIT;
         }
         else if (u32_iterations >= (65536L * BGVAR(s32_ParamsBackup)[MPOLES_STORE_INDEX] * 2L / (long)abs(s16_commutation_delta)))
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MISSING_INDEX);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
      break;

      case MOTOR_SETUP_TM_POSITIVE_TEMP_INDEX_SEARCH:
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;
         u32_iterations++;

         if (u32_iterations == 500)
            VAR(AX0_s16_Crrnt_Run_Code) |= MOTOR_SETUP_LOOK_FOR_INDEX_MASK;

         if ( (VAR(AX0_s16_Crrnt_Run_Code) & MOTOR_SETUP_INDEX_CAPTURED_MASK) && (u32_iterations > 1500) )
         {// This is to Tamagawa 8-Wires only!!!
            VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += (s16_commutation_delta << 3);
//            BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
            s16_commutation_delta = -40;
            u32_iterations = 0;
            VAR(AX0_s16_Crrnt_Run_Code) &= ~MOTOR_SETUP_INDEX_CAPTURED_MASK;
//            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TAMAGAWA_READ_NEG_HALLS;
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TM_NEGATIVE_TEMP_INDEX_SEARCH;
         }
         else if (u32_iterations >= (65536L * BGVAR(s32_ParamsBackup)[MPOLES_STORE_INDEX] * 2L / (long)abs(s16_commutation_delta)))
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MISSING_INDEX);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
      break;

      case MOTOR_SETUP_TM_NEGATIVE_TEMP_INDEX_SEARCH:
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;
         u32_iterations++;

         if (u32_iterations == 500)
            VAR(AX0_s16_Crrnt_Run_Code) |= MOTOR_SETUP_LOOK_FOR_INDEX_MASK;

         if (VAR(AX0_s16_Crrnt_Run_Code) & MOTOR_SETUP_INDEX_CAPTURED_MASK)
         {// This is to Tamagawa 8-Wires only!!!
            VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += (s16_commutation_delta << 2);
            BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TAMAGAWA_READ_NEG_HALLS;
         }
         else if (u32_iterations >= (65536L * BGVAR(s32_ParamsBackup)[MPOLES_STORE_INDEX] * 2L / (long)abs(s16_commutation_delta)))
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MISSING_INDEX);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
      break;

      // Read Halls right after index crossing
      case MOTOR_SETUP_TAMAGAWA_READ_NEG_HALLS:
         if (PassedTimeMS(800L, BGVAR(s32_Motor_Setup_Timer)))
         {
            if (ReadHallsFromFeedback(&BGVAR(s16_Halls_Neg)[0], drive)) return;
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_POS_SWEEP_INIT;
         }
      break;

      // Init sweep in positive direction
      case MOTOR_SETUP_INDEX_POS_SWEEP_INIT:
         s16_commutation_delta = 40;
         u32_iterations = 0;
         u16_pole_pairs_count = 0;

         index_counter = 0;
         VAR(AX0_s16_Crrnt_Run_Code) &= ~MOTOR_SETUP_INDEX_CAPTURED_MASK;
         VAR(AX0_s16_Crrnt_Run_Code) |= MOTOR_SETUP_LOOK_FOR_INDEX_MASK;

         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_POS_SWEEP_RUN;
      break;

      // Count # of electrical cycles between the indexes (to calculate MPOLES)
      // Move till index is crossed again this time in positive direction
      case MOTOR_SETUP_INDEX_POS_SWEEP_RUN:
         if (index_counter == 1)
         {
            // Count # of electrical cycle to count MPOLES value
            if ((VAR(AX0_s16_Elect_Pos_With_Phase_Adv) < 0x8000) && ((VAR(AX0_s16_Elect_Pos_With_Phase_Adv) + s16_commutation_delta) >= 0x8000))
               u16_pole_pairs_count++;

            // Arm Index after some movement to avoid jitter around the index pulse
            if (u32_iterations == 1000)
               VAR(AX0_s16_Crrnt_Run_Code) |= MOTOR_SETUP_LOOK_FOR_INDEX_MASK;
         }

         u32_iterations++;

         // Wait for second index or timeout for index missing
         if (VAR(AX0_s16_Crrnt_Run_Code) & MOTOR_SETUP_INDEX_CAPTURED_MASK)
         {
            if (index_counter == 0)
            {
               s64_first_position = (long long)VAR(AX0_s16_Index_Qep_Capture);
               VAR(AX0_s16_Crrnt_Run_Code) &= ~MOTOR_SETUP_INDEX_CAPTURED_MASK;
               VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += (s16_commutation_delta << 2);
               u32_iterations = 0;
               index_counter = 1;
            }
            else
            {
               u16_menczpos = VAR(AX0_s16_Elect_Pos_With_Phase_Adv) + DEG_90_OFFSET;

               u16_electangle_pos = VAR(AX0_s16_Elect_Pos_With_Phase_Adv) + DEG_90_OFFSET;
               VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += (s16_commutation_delta << 2);
               if (FEEDBACK_TAMAGAWA)
               {
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TAMAGAWA_READ_POS_HALLS;
                  VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += 1000;
                  BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
               }
               else
                  BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_POS_SWEEP_END;
            }
         }
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;
      break;

      // Read Halls right after index crossing, and decide halls table direction accordingly
      // Also set MPHASE and MENCZPOS according to the index electrical position
      case MOTOR_SETUP_TAMAGAWA_READ_POS_HALLS:
         if (PassedTimeMS(2000L, BGVAR(s32_Motor_Setup_Timer)))
         {
            if (ReadHallsFromFeedback(&BGVAR(s16_Halls_Neg)[1], drive)) return;

            // Index signal should be between 101("negative" side) and 001("positive" side)
            // Make sure this is the case and fix if not
            s16_halls_direction = 0;
            if (HallsDirection(BGVAR(s16_Halls_Neg)[0], BGVAR(s16_Halls_Neg)[1], &s16_halls_table_index, &s16_halls_direction))
            {
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MISSING_HALLS);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
               break;
            }

            // Detect MPHASE
            u16_mphase_calc = u16_electangle_pos - (int)(*((int*)(&Enc_Hall_Switch_Tbl_4to5to1to3to2to6) + s16_halls_table_index));
            if (s16_halls_direction == 1)
               u16_mphase_calc = u16_electangle_pos - (int)(*((int*)(&Enc_Hall_Switch_Tbl_4to6to2to3to1to5) + s16_halls_table_index));

            // Round MPHASE to 10 degs
            u16_mphase_calc = ((int)(u16_mphase_calc + 910) / 1820) * 1820;
            if (VAR(AX0_S16_Kc_Mode) > 3) u16_mphase_calc += DEG_30_OFFSET; // Used to compensate for the 30deg offset in old Iuvw-Idq transformation

            if (abs((int)((int)BGVAR(s32_ParamsBackup)[MPHASE_STORE_INDEX] / 182) - ((int)u16_mphase_calc / 182)) > 10)
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MPHASE_ADJUST;

            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_POS_SWEEP_END;
         }
      break;

      // Calculate MENCRES according to the delta between index positions
      case MOTOR_SETUP_INDEX_POS_SWEEP_END:
         s64_second_position = (long long)VAR(AX0_s16_Index_Qep_Capture);
         s64_delta_position = s64_second_position - s64_first_position;
         if (s64_delta_position < 0LL) s64_delta_position = -s64_delta_position;
         // Handle sign extensions
         if (s64_delta_position > 0x7FFFLL) s64_delta_position = 0x0000FFFFLL - s64_delta_position;
         if (s64_delta_position < 0LL) s64_delta_position = -s64_delta_position;

         if (s64_delta_position < (long long)((BGVAR(s32_ParamsBackup)[MENCRES_STORE_INDEX] >> 4) / BGVAR(s32_ParamsBackup)[MPOLES_STORE_INDEX]))
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_NO_MOVEMENT);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            break;
         }

         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND_SETUP;

         // Adjust MENCRES if needed
         if (((s64_delta_position + 2LL) >> 2) != BGVAR(u32_User_Motor_Enc_Res))
         {
            u32_calculated_mencres = (unsigned long)((s64_delta_position + 2LL) >> 2);
            if (FixMencres(&u32_calculated_mencres))
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MENCRES_ADJUST;
            else BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MENCRES_BAD;
         }

         // Adjust MPOLES if needed
         if (((u16_pole_pairs_count << 1) != (int)BGVAR(s32_ParamsBackup)[MPOLES_STORE_INDEX]) && (u16_pole_pairs_count > 0))
            BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MPOLES_ADJUST;

         // Udjust MENCZPOS if needed (more than 10deg deviation)
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MPHASE_ADJUST)
            u16_menczpos -= u16_mphase_calc;
         else u16_menczpos -= (int)BGVAR(s32_ParamsBackup)[MPHASE_STORE_INDEX];
         if (VAR(AX0_S16_Kc_Mode) > 3) u16_menczpos += DEG_30_OFFSET; // Used to compensate for the 30deg offset in old Iuvw-Idq transformation

         if (abs(u16_menczpos - VAR(AX0_s16_Index_Elect_Pos)) > 1820)
            BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MENCZPOS_ADJUST;
      break;

      // Prepare for torque command to set positive torque causes positive velocity
      case MOTOR_SETUP_TORQUE_COMMAND_SETUP: // Set all the swaps detected till now
         DisableCommand(drive);
         if (Enabled(DRIVE_PARAM)) break;

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MENCRES_ADJUST)
         {
            if ( (BGVAR(u16_MTP_Mode) != 1) && (BGVAR(u16_MTP_Mode) != 3) )
               SalMotorEncResCommand((long long)u32_calculated_mencres, drive);
            else
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ACTIVE);
         }

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MPOLES_ADJUST)
         {
            if ( (BGVAR(u16_MTP_Mode) != 1) && (BGVAR(u16_MTP_Mode) != 3) )
               SalMpolesCommand((long long)(u16_pole_pairs_count << 1), drive);
            else
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ACTIVE);
         }

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MENCZPOS_ADJUST)
         {
            VAR(AX0_s16_Index_Elect_Pos) = u16_menczpos;
            // Round to 10 degs
            SalReadMencoff(&s64_temp, drive);
            s64_temp = (s64_temp + 5LL) / 10LL;
            s64_temp *= 10LL;
            if (s64_temp >= 360) s64_temp = 0LL;
            SalWriteMencoff(s64_temp, drive);
         }

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MPHASE_ADJUST)
         {
            if (BGVAR(u16_MTP_Mode) == 0)
               BGVAR(u16_Electrical_Phase_Offset_User) = VAR(AX0_s16_Electrical_Phase_Offset) = u16_mphase_calc;
            else
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ACTIVE);
         }

         u16_mfbdir_build = 0;
         if (s16_direction_test == 1)
            u16_mfbdir_build |= 0x01;
         if (s16_halls_direction == 1)
            u16_mfbdir_build |= 0x02;
         if (u16_index_polarity == 1)
            u16_mfbdir_build |= 0x04;
         if (u16_sincos_direction_endat_fix == 1)
            u16_mfbdir_build |= 0x08;
         if (u16_mfbdir_build != (int)BGVAR(s32_ParamsBackup)[MFBDIR_STORE_INDEX])
         {
            BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_MFBDIR_ADJUST;
            if ( (BGVAR(u16_MTP_Mode) != 1) && (BGVAR(u16_MTP_Mode) != 3) )
               SalMotorFeedbackDirectionCommand((long long)u16_mfbdir_build, drive);
            else
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ACTIVE);
         }
         BGVAR(s16_Direction) = (int)BGVAR(s32_ParamsBackup)[DIR_STORE_INDEX];

         VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;

         if (!FEEDBACK_INDEX_ONLY)
         {
            LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Feedback_Comm_Pos);
            VAR(AX0_s16_Skip_Flags) &= ~MOTOR_SETUP_MASK;

            // change opmode for fieldbus and internaly
            if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
            {
                 BGVAR(s16_CAN_Opmode) = PROFILE_TORQUE_MODE;
                 BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
            }
            else
            {
                 SetOpmode(drive, 2);
            }
         }
         if (FEEDBACK_AB_ONLY)         // Without halls do not initiate the move
         {
            BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_SUCCEEDED;
            BGVAR(u32_Motor_Setup_Status) &= ~MOTOR_SETUP_STILL_RUNNING;
            PhaseFindCommand(drive);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
         else
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND_CONFIG;
      break;

      case MOTOR_SETUP_TORQUE_COMMAND_CONFIG:      // Issue a config then enable the drive
         if (SalConfigCommand(drive) != SAL_NOT_FINISHED)
         {
            if ( (FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_ERROR_STATE)) break;
            if ( (FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) == COMM_FDBK_ERROR_STATE) )
            {
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_COMM_FDBK_ERROR);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            }
            if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(u16_Read_Mtp_State) != MTP_INIT_INIT) ) break;
            if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(s16_MTP_Error) != 0) )
            {
               BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ERROR);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            }

            if (!FEEDBACK_INDEX_ONLY) BGVAR(s16_Motor_Setup_Current) = 0;
            else
            {
               VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_2 & 0xffff);
               BGVAR(s64_Faults_Mask) &= ~PHASE_FIND_FLT_MASK;
               BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;
               VAR(AX0_s16_PhaseFindRTBits)= 0;
               LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &BGVAR(s16_Motor_Setup_Current);
            }
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_WAIT_ENC_INIT;
         }
      break;

      case MOTOR_SETUP_TORQUE_WAIT_ENC_INIT:      // Issue a config then enable the drive
         EnableCommand(drive);
         u32_iterations = 0;
         BGVAR(s32_Motor_Setup_Timer) = Cntr_1mS;
         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_WAIT_FOR_ENABLE;
      break;

      case MOTOR_SETUP_TORQUE_WAIT_FOR_ENABLE:   // Wait for the drive to enable
         if (!PassedTimeMS(50L, BGVAR(s32_Motor_Setup_Timer))) break;
         if (Enabled(drive))
         {
            if (FEEDBACK_INDEX_ONLY && (VAR(AX0_s16_Skip_Flags) & MOTOR_SETUP_MASK))
            {
               s16_commutation_delta = -s16_commutation_delta;
               BGVAR(s16_Motor_Setup_Current) = BGVAR(s16_Max_Current);
               BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_ONLY_LOOK_FOR_INDEX;
            }
            else
            {
               if (FEEDBACK_TAMAGAWA_ABS || FEEDBACK_PANASONIC_P_G || FEEDBACK_PANASONIC_S || FEEDBACK_SERVOSENSE || FEEDBACK_YASKAWA || FEEDBACK_SANKYO)
               { // Wait for the Commfeedback startup to end
                  if (!(BGVAR(s16_DisableInputs) & COMM_FDBK_DIS_MASK))
                     BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND;
               }
               else
                 BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_COMMAND;
               s32_Fb_Vel_Average = 0; // Reset Averaging Variable of Feedback Velocity...
            }
         }
         BGVAR(s32_Motor_Setup_Pulse_Duration) = Cntr_1mS;
      break;

      // Wait for the index to initialize the commutation to allow  motion with MENCTYPEs with no halls
      case MOTOR_SETUP_INDEX_ONLY_LOOK_FOR_INDEX:
         if (VAR(AX0_s16_Enc_State_Ptr) == (int)((long)&ENC_STATE_3 & 0xffff))
         {
            DisableCommand(drive);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_INDEX_ONLY_LOOK_FOR_INDEX_END;
         }

         u32_iterations++;
         VAR(AX0_s16_Elect_Pos_With_Phase_Adv) += s16_commutation_delta;
      break;

      case MOTOR_SETUP_INDEX_ONLY_LOOK_FOR_INDEX_END:
         if (Enabled(drive)) break;

         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Feedback_Comm_Pos);
         VAR(AX0_s16_Skip_Flags) &= ~MOTOR_SETUP_MASK;

         if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
         {
            BGVAR(s16_CAN_Opmode) = PROFILE_TORQUE_MODE;
            BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
         }
         else
         {
            SetOpmode(drive, 2);
         }

         BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;
         VAR(AX0_s16_PhaseFindRTBits)= 0;
         BGVAR(s16_Motor_Setup_Current) = 0;

         EnableCommand(drive);
         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TORQUE_WAIT_FOR_ENABLE;
      break;

      // Increase current (this time with normal commutation) till velocity is detected
      // If velocity is with negative direction raise a flag to swap MPHASE by 180degs
      case MOTOR_SETUP_TORQUE_COMMAND: // Increase current till some movement is detected to check
         // that positive torque yields positive velocity
         InternalTorqueCommand((long long)BGVAR(s16_Motor_Setup_Current), drive);
         s32_Fb_Vel_Average = (long)(0.98 * s32_Fb_Vel_Average + 0.02 * LVAR(AX0_s32_Vel_Var_Fb_0));
         // To avoid noisy indication at the RT Variable, use Filtering...
         if (labs(s32_Fb_Vel_Average) < 50000L)
         {
            u16_velocity_filter = 0;
            BGVAR(s16_Motor_Setup_Current)++;
         }
         else u16_velocity_filter++;

         if (u16_velocity_filter >= 5)
         {
            // Capture the pulse time
            BGVAR(s32_Motor_Setup_Pulse_Duration) = Cntr_1mS - BGVAR(s32_Motor_Setup_Pulse_Duration);
            if (BGVAR(s32_Motor_Setup_Pulse_Duration) < 0L)
               BGVAR(s32_Motor_Setup_Pulse_Duration) = -BGVAR(s32_Motor_Setup_Pulse_Duration);
            if (BGVAR(s32_Motor_Setup_Pulse_Duration) > 5000)
               BGVAR(s32_Motor_Setup_Pulse_Duration) = 5000;
            if (s32_Fb_Vel_Average < 0L)
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_TORQUE_VEL_DIR_ADJUST;
            if (AX0_s32_Sfb_Vel_Var_Fb_0 < 0L) //if Load's velocity is negative - change SFBDIR
               SalSfbDirectionCommand(1LL,drive);
            if (((long long)BGVAR(s32_ParamsBackup)[SFB_DIR_INDEX]) != ((long long)s16_SFB_Direction))
               BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_SFBDIR_CHANGED;

            BGVAR(u32_Motor_Setup_Status) |= MOTOR_SETUP_SUCCEEDED;
            BGVAR(u32_Motor_Setup_Status) &= ~MOTOR_SETUP_STILL_RUNNING;
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
            break;
         }

         if (BGVAR(s16_Motor_Setup_Current) > BGVAR(s16_Max_Current))
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_NO_MOVEMENT);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
      break;

      case MOTOR_SETUP_TERMINATE:      // All cleanup and restores are done here
         DisableCommand(drive);
         if (Enabled(DRIVE_PARAM)) break;

         // Wait for DB to stop the motion
         if (labs(LVAR(AX0_s32_Vel_Var_Fb_0)) > 0x3000L) break;

         // This is to activate 1/T interpolation in the FPGA
         if (BGVAR(u16_FdbkType) == INC_ENC_FDBK)
            SalMotorEncInterpolationModeCommand((long long)BGVAR(s32_ParamsBackup)[MFBMODE_STORE_INDEX], drive);

         BGVAR(u16_Read_Halls_State) = 0;

         // Restore original ILIM, VLIM, DISMODE, OPMODE values
         SalIlimCommand((long long)BGVAR(s32_ParamsBackup)[ILIM_STORE_INDEX], drive);
         VLimCommand((long long)BGVAR(s32_ParamsBackup)[VLIM_STORE_INDEX], drive, &s16_dummy);
         SalIstopCommand((long long)BGVAR(s32_ParamsBackup)[ISTOP_STORE_INDEX], drive);
         SalDisableModeCommand((long long)BGVAR(s32_ParamsBackup)[DISMODE_STORE_INDEX], drive);
         SalSFBModeCommand((long long)BGVAR(s32_ParamsBackup)[SFB_MODE_INDEX],drive);

         VAR(AX0_s16_Opmode) = (int)BGVAR(s32_ParamsBackup)[OPMODE_STORE_INDEX];
/*
         if ( ((BGVAR(u32_Motor_Setup_Status) & (MOTOR_SETUP_FAILED | MOTOR_SETUP_STILL_RUNNING)) == 0)                                    &&
              ((BGVAR(u32_Motor_Setup_Status) & (MOTOR_SETUP_MPOLES_ADJUST | MOTOR_SETUP_MPHASE_ADJUST | MOTOR_SETUP_MFBDIR_ADJUST)) != 0) &&
              (BGVAR(u16_MTP_Mode_User) != 0)                                                                                                )
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ACTIVE); */

         // If procedure failed or terminated by user restore previous parameters
         if (BGVAR(u32_Motor_Setup_Status) & (MOTOR_SETUP_FAILED | MOTOR_SETUP_STILL_RUNNING))
         {
            BGVAR(s16_Direction) = (int)BGVAR(s32_ParamsBackup)[DIR_STORE_INDEX];
            SalMotorFeedbackDirectionCommand((long long)BGVAR(s32_ParamsBackup)[MFBDIR_STORE_INDEX], drive);
            SalMpolesCommand((long long)BGVAR(s32_ParamsBackup)[MPOLES_STORE_INDEX], drive);
            SalMotorEncResCommand((long long)BGVAR(s32_ParamsBackup)[MENCRES_STORE_INDEX], drive);
            VAR(AX0_s16_Index_Elect_Pos) = (int)BGVAR(s32_ParamsBackup)[MENCZPOS_STORE_INDEX];
            BGVAR(u16_Electrical_Phase_Offset_User) = VAR(AX0_s16_Electrical_Phase_Offset) = (int)BGVAR(s32_ParamsBackup)[MPHASE_STORE_INDEX];

            SalMlminCommand((long long)BGVAR(s32_ParamsBackup)[ML_STORE_INDEX],drive);
            SalMresCommand((long long)BGVAR(s32_ParamsBackup)[MR_STORE_INDEX],drive); 
            SalSfbDirectionCommand((long long)BGVAR(s32_ParamsBackup)[SFB_DIR_INDEX],drive);
         }
         else
         {
            if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_TORQUE_VEL_DIR_ADJUST)
               BGVAR(u16_Electrical_Phase_Offset_User) = VAR(AX0_s16_Electrical_Phase_Offset) += 0x8000;
         }

         BGVAR(u64_Pe_Max) = (((long long)BGVAR(s32_ParamsBackup)[PEMAX_LO_STORE_INDEX]) & 0xFFFFFFFF);
         BGVAR(u64_Pe_Max) |= ((long long)BGVAR(s32_ParamsBackup)[PEMAX_HI_STORE_INDEX]) << 32LL;

         BGVAR(s64_Abs_Fdbk_Offset) = (((long long)BGVAR(s32_ParamsBackup)[ABSOFFSET_LO_STORE_INDEX]) & 0xFFFFFFFF);
         BGVAR(s64_Abs_Fdbk_Offset) |= ((long long)BGVAR(s32_ParamsBackup)[ABSOFFSET_HI_STORE_INDEX]) << 32LL;

         VAR(AX0_s16_Motor_Enc_Type) = -1; // This is to cause the SalMotorEncTypeCommand to have affect
         SalMotorEncTypeCommand((long long)BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX], drive);

         if ((BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX] >= 1) && (BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX] <= 4))
            BGVAR(s64_Faults_Mask) |= PHASE_FIND_FLT_MASK;

         LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Feedback_Comm_Pos);

         // Restore COMMODE
         SalComModeCommand((long long)BGVAR(s32_ParamsBackup)[COMMODE_STORE_INDEX], drive);

         // restore opmode
         if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
         {    // background func will chaneg opmode
              BGVAR(s16_CAN_Opmode) = (int)(BGVAR(s32_ParamsBackup)[FB_OPMODE_STORE]);
              BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
         }
         else
         {
              SetOpmode(drive, VAR(AX0_s16_Opmode));
         }
         BGVAR(u16_Motor_Setup_Allow_Enable) = 0;
         BGVAR(u16_Motor_Setup_Small_Pitch) = 0;
         VAR(AX0_s16_Skip_Flags) |= ENC_CONFIG_NEEDED_MASK;

         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_WAIT_FOR_CONFIG_END;
      break;

      case MOTOR_SETUP_WAIT_FOR_CONFIG_END:      // Config and end procedure
         if (SalConfigCommand(drive) == SAL_NOT_FINISHED) break;
         if ((FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_IDLE_STATE) && (BGVAR(s16_Comm_Fdbk_Init_State) != COMM_FDBK_ERROR_STATE)) break;         
       
         if ( (FEEDBACK_ABS_INTURN) && (BGVAR(s16_Comm_Fdbk_Init_State) == COMM_FDBK_ERROR_STATE) )
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_COMM_FDBK_ERROR);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }
         if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(u16_Read_Mtp_State) != MTP_INIT_INIT) ) break;
         if ( (BGVAR(u16_MTP_Mode) != 0) && (BGVAR(s16_MTP_Error) != 0) )
         {
            BGVAR(u32_Motor_Setup_Status) |= (MOTOR_SETUP_FAILED | MOTOR_SETUP_MTPMODE_ERROR);
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;
         }

         VAR(AX0_s16_Skip_Flags) &= ~MOTOR_SETUP_MASK;
         BGVAR(u32_Motor_Setup_Status) &= ~MOTOR_SETUP_ACTIVE;

         BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_IDLE;
      break;
   }

   return;
}


int MotorSetupStatusCommand(int drive)
{
   // AXIS_OFF;

   static int u16_motor_setup_status_state = 0;
   int shr_val = 0, curr_value = 0, i, u16_halls_sampled;
   long long half_for_rounding = 0LL;
   REFERENCE_TO_DRIVE;

   if (u8_Output_Buffer_Free_Space < 60) return SAL_NOT_FINISHED;

   switch (u16_motor_setup_status_state)
   {
      case 0:
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_ACTIVE)
            PrintStringCrLf("Motor Setup Active", 0);
         else
         {
            if (!(BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_ISSUED))
               PrintStringCrLf("Motor Setup Not Issued", 0);
            else if ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_FAILED) || (!(BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_SUCCEEDED)))
               PrintStringCrLf("Motor Setup Failed", 0);
            else
            {
               shr_val = BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;
               if (shr_val > 0) half_for_rounding = 1LL << ((long long)shr_val - 1);
               curr_value = (int)((BGVAR(s16_Motor_Setup_Current) * (long long)BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix + half_for_rounding) >> (long long)shr_val);

               PrintStringCrLf("Motor Setup Succeeded", 0);
               PrintString("Current Pulse: ", 0);
               PrintUnsignedInt16(curr_value);
               PrintString(" mA ", 0);
               PrintUnsignedInt32(BGVAR(s32_Motor_Setup_Pulse_Duration));
               PrintStringCrLf(" ms", 0);
            }
         }
         PrintString("Stage: ", 0);
         PrintUnsignedInt16(BGVAR(u16_Motor_Setup_State));
         PrintChar('/');
         PrintUnsignedInt16(MOTOR_SETUP_WAIT_FOR_CONFIG_END); // This should be the last phase
         PrintCrLf();
         u16_motor_setup_status_state++;
      break;

      case 1:
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_NO_MOVEMENT)
            PrintStringCrLf("No Movement Detected - Check if Feedback and Phases Connected, and No Movement Limitations", 0);

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_HARD_STOP)
            PrintStringCrLf("Motion Obstructed - Check if not on Hard Stop", 0);

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_AB_I_SWAP)
            PrintStringCrLf("I is swapped with A or B Signal", 0);

         u16_motor_setup_status_state++;
      break;

      case 2:
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MISSING_PHASE)
            PrintStringCrLf("At least 1 motor phase is not connected", 0);

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MISSING_HALLS)
            PrintStringCrLf("Bad Halls Reading - Check if halls are connected", 0);
         u16_motor_setup_status_state++;
      break;

      case 3:
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_ESTMOTORPARAM_ERR)
            PrintStringCrLf("ML & MR Detection Failure", 0);

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MISSING_HALLS)
         {
            PrintString("Detected Halls: ", 0);
            if (FEEDBACK_TAMAGAWA) u16_halls_sampled = 1;
            else u16_halls_sampled = 6;

            for (i = 0; i < (u16_halls_sampled + 1); i++)
            {
               PrintUnsignedInt16(BGVAR(s16_Halls_Neg)[i]);
               if (i < u16_halls_sampled) PrintChar(DASH);
               else PrintCrLf();
            }
         }
         u16_motor_setup_status_state++;
      break;

      case 4:
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MISSING_INDEX)
            PrintStringCrLf("Missing Index - Check if Index signal is connected properly", 0);

         if ( ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MFBDIR_ADJUST) != 0) &&
              ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MTPMODE_ACTIVE) == 0)  )
            PrintStringCrLf("MFBDIR updated", 0);
         u16_motor_setup_status_state++;
      break;

      case 5:
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MENCRES_BAD)
            PrintStringCrLf("MENCRES did not fit calculated feedback resolution", 0);
         else if ( ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MENCRES_ADJUST) != 0) &&
                   ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MTPMODE_ACTIVE) == 0)   )
            PrintStringCrLf("MENCRES updated", 0);
         u16_motor_setup_status_state++;
      break;

      case 6:
         if ( ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MPOLES_ADJUST) != 0) &&
              ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MTPMODE_ACTIVE) == 0)  )
            PrintStringCrLf("MPOLES updated", 0);

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MENCZPOS_ADJUST)
            PrintStringCrLf("MENCZPOS updated", 0);

         u16_motor_setup_status_state++;
      break;

      case 7:
         if ( ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MPHASE_ADJUST) != 0) &&
              ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MTPMODE_ACTIVE) == 0)  )
            PrintStringCrLf("MPHASE updated", 0);

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_TORQUE_VEL_DIR_ADJUST)
            PrintStringCrLf("MPHASE +180 to swap movement direction", 0);

         u16_motor_setup_status_state++;
      break;

      case 8:
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_INVERTED_INDEX)
            PrintStringCrLf("Invert Index Signal", 0);

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_DISABLED)
            PrintStringCrLf("Drive Disabled During Procedure", 0);

         u16_motor_setup_status_state++;
      break;

      case 9:
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MENCRES_PER_MPOLES)
            PrintStringCrLf("Wrong MENCRES/MPOLES Ratio", 0);

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_STEGMANN_COMM_ERR)
            PrintStringCrLf("Cannot Communicate with Feedback,Check Feedback Wiring", 0);

         u16_motor_setup_status_state++;
      break;

      case 10:
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_ML_ADJUST)
            PrintStringCrLf("ML updated", 0);

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MR_ADJUST)
            PrintStringCrLf("MR updated", 0);
         
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_SFBDIR_CHANGED)
            PrintStringCrLf("SFBDIR updated", 0); 

         u16_motor_setup_status_state++;
      break;

      case 11:
         if (FEEDBACK_INDEX_ONLY)
            PrintStringCrLf("MPHASE Cannot Be Verified with This Feedback", 0);

         if (FEEDBACK_AB_ONLY)
            PrintStringCrLf("MPHASE, MENCRES and MPOLES Cannot Be Verified with This Feedback", 0);

         if (FEEDBACK_RESOLVER)
            PrintStringCrLf("MRESPOLES Cannot Be Verified", 0);

         u16_motor_setup_status_state++;
      break;

      case 12:
         if ((BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MTPMODE_ACTIVE) != 0)
         {
            PrintStringCrLf("Detected Parameters do not match MTP Database:", 0);

            if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MPOLES_ADJUST)
               PrintStringCrLf("Detected Motor-Poles different from MTP Data", 0);

            if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MFBDIR_ADJUST)
               PrintStringCrLf("Detected Motor-Phase Sequence different from MTP Data, re-wiring required", 0);

            if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MPHASE_ADJUST)
               PrintStringCrLf("Detected MPHASE different from MTP Data, re-wiring required", 0);

            if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MENCRES_ADJUST)
               PrintStringCrLf("Detected MENCRES different from MTP Data", 0);
         }

         u16_motor_setup_status_state++;
      break;

      case 13:
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_MTPMODE_ERROR)
            PrintStringCrLf("MTP Data Read failed", 0);

         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_COMM_FDBK_ERROR)
            PrintStringCrLf("Communication Encoder Initialization failed", 0); 
         
         if (BGVAR(u32_Motor_Setup_Status) & MOTOR_SETUP_CONFIG_FAIL)
            PrintStringCrLf("Motor Setup Config failed", 0);

         u16_motor_setup_status_state++;
      break;
   }

   if (u16_motor_setup_status_state == 14)
   {
      u16_motor_setup_status_state = 0;
      return SAL_SUCCESS;
   }
   else return SAL_NOT_FINISHED;
}


int MotorSetupCommand(int drive)
{
   if (VAR(AX0_u16_Motor_Comm_Type) != BRUSHLESS_MOTOR) return (MOTOR_COMM_TYPE_INVALID);

   // Do not allow if Zero is running
   if (BGVAR(s16_Zero_Mode) == 1) return (DRIVE_IN_ZERO_MODE);

   if (s16_Number_Of_Parameters == 1) // MOTORSETUP 0 will abort the procedure
   {
      if (s64_Execution_Parameter[0] > 1LL) return (VALUE_OUT_OF_RANGE);
      if (s64_Execution_Parameter[0] == 0LL)
      {
         if (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_IDLE)
            BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_TERMINATE;

         return SAL_SUCCESS;
      }
   }
//
   if (BGVAR(u16_Motor_Setup_State) != MOTOR_SETUP_IDLE) return (SAL_SUCCESS);

   if ( ( (!FEEDBACK_WITH_ENCODER) && (!FEEDBACK_RESOLVER) && (!FEEDBACK_PANASONIC_P_G) && (!FEEDBACK_SANKYO)  &&
          (!FEEDBACK_TAMAGAWA_ABS) && (!FEEDBACK_SERVOSENSE) && (!FEEDBACK_BISSC) && (!FEEDBACK_ABS_YASKAWA)   &&
          (!FEEDBACK_ENDAT) && (!FEEDBACK_NIKON) && (BGVAR(u16_FdbkType) != PS_S_COMM_FDBK)                      ) ||
        ( (FEEDBACK_ABS_YASKAWA) && (BGVAR(u32_User_Motor_Enc_Res) != 262144L) )                                   ||
          // Support MotorSetup for Yaskawa 17bit only
        ( (FEEDBACK_NIKON) && (BGVAR(u32_User_Motor_Enc_Res) != 32768L) )                                          ||
          // Support MotorSetup for Nikon 17bit only
        ( (1 == s16_Number_Of_Parameters) && (s64_Execution_Parameter[0] == 1LL) &&
          (BGVAR(u16_MotorType) != LINEAR_MOTOR)                                   )                                 )
      return NOT_AVAILABLE;

   // Check if no other procedure is running in the drive
   if (ProcedureRunning(DRIVE_PARAM) != PROC_NONE) return (OTHER_PROCEDURE_RUNNING);

   if (Enabled(DRIVE_PARAM)) return (DRIVE_ACTIVE);

   //if (BGVAR(s16_DisableInputs) & FLT_EXISTS_MASK) return (FAULTS_INHIBITION);

   BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_START;

   // MOTORSETUP 1, will execute MOTORSETUP in limited "Small Pitch" variation
   if (s64_Execution_Parameter[0] == 1LL)
   {
      //BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_PRE_TEST_SP;
      //BGVAR(u16_Motor_Setup_State) = MOTOR_SETUP_WAIT_FOR_DISABLED_SP;
      BGVAR(u16_Motor_Setup_Small_Pitch) = 1;
   }
   return (SAL_SUCCESS);
}


/*
int EstMotorParamsStCommand(int drive,int s16_fb_use )
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error
   if (s16_fb_use == FB_NOT_IN_USE)
   {
      if (OutputBufferLoader()==0) return SAL_NOT_FINISHED;
   }
   strcpy((char*)&u16_General_Domain[0],"\0");

   if (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_IDLE) strcat((char*)&u16_General_Domain[0],"Process Idle");
   else if (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_FAULT) strcat((char*)&u16_General_Domain[0],"Process Failed");
   else if (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_DONE)
   {
      strcat((char*)&u16_General_Domain[0],"Process Done\r\n");
      strcat((char*)&u16_General_Domain[0],"MR = ");
      strcat((char*)&u16_General_Domain[0],DecimalPoint32ToAscii(BGVAR(s32_Param_Est_MR)));
      strcat((char*)&u16_General_Domain[0],"[Ohm], ML = ");
      strcat((char*)&u16_General_Domain[0],DecimalPoint32ToAscii(BGVAR(s32_Param_Est_ML)));
      strcat((char*)&u16_General_Domain[0],"[mH], DT = ");
      strcat((char*)&u16_General_Domain[0],SignedIntegerToAscii(BGVAR(s16_Param_Est_V_Dead_Time)));
      strcat((char*)&u16_General_Domain[0],", SL Factor = ");
      strcat((char*)&u16_General_Domain[0],DecimalPoint32ToAscii(BGVAR(s32_Param_Est_SLfactor)));
      strcat((char*)&u16_General_Domain[0],"\r\n");
   }
   else strcat((char*)&u16_General_Domain[0],"Process Running");

   if (s16_fb_use == FB_NOT_IN_USE)
   {
      OutputBufferLoader();
   }
   return (SAL_SUCCESS);
}


void ParamEstCalcMr(void)
{
   long s32_temp;
   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // for Vpwm = Vdeadtime + i*r and i = imax and imax/4 find r and Vdeadtime
   s32_temp = (long)(4.0/3.0*1000000*(float)BGVAR(u16_Vbus_Volts)/(float)AX0_s16_Pwm_Half_Period * (float)(s16_Param_Est_Vq_At_Imax - s16_Param_Est_Vq_At_Quarter_Imax) / (float)(s32_Param_Est_Icmd));
   if (s32_temp >= 50L) s32_temp = ((s32_temp + 50) / 100L) * 100L;
   BGVAR(s32_Param_Est_MR) = s32_temp;

   BGVAR(s16_Param_Est_V_Dead_Time) = (int)((float)BGVAR(u16_Vbus_Volts)/(float)AX0_s16_Pwm_Half_Period * (float)(s16_Param_Est_Vq_At_Imax) -  (float)BGVAR(s32_Param_Est_MR) * (float)s32_Param_Est_Icmd / 1000000.0);
   if (BGVAR(s16_Param_Est_V_Dead_Time) < 0) BGVAR(s16_Param_Est_V_Dead_Time) = 0;
}
*/

void LowerCurrentto0(int drive)
{
   InternalTorqueCommand((long long)(BGVAR(s32_Serial_Torque_Cmd) >> 2), drive);
   WaitForNext32kHzTaskStart();
   InternalTorqueCommand((long long)(BGVAR(s32_Serial_Torque_Cmd) >> 1), drive);
   WaitForNext32kHzTaskStart();
   InternalTorqueCommand((long long)(BGVAR(s32_Serial_Torque_Cmd) >> 1), drive);
   WaitForNext32kHzTaskStart();
   InternalTorqueCommand(0LL, drive);
   WaitForNext32kHzTaskStart();

   return;
}


/* ***** Old motor parameters estimation process *******/
/*
void ParamEstRestore(int drive)
{
   // AXIS_OFF;

   SalIlimCommand((long long)BGVAR(s32_ParamsBackup)[ILIM_STORE_INDEX], drive);
   // restore opmode for fieldbus and internaly
   if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
   {
       BGVAR(s16_CAN_Opmode) = (int)(BGVAR(s32_ParamsBackup)[FB_OPMODE_STORE]);
       BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
   }
   else
   {
       SetOpmode(drive, (int)BGVAR(s32_ParamsBackup)[OPMODE_STORE_INDEX]);
   }
   SalClBemfCommand((long long)(BGVAR(s32_ParamsBackup)[KCBEMF_STORE_INDEX]), drive);
   SalMcontCurrentGainCommand((long long)(BGVAR(s32_ParamsBackup)[MLGAINC_STORE_INDEX]), drive);
   SalMpeakCurrentGainCommand((long long)(BGVAR(s32_ParamsBackup)[MLGAINP_STORE_INDEX]), drive);
   BGVAR(u16_DisTime) = (long long)(BGVAR(s32_ParamsBackup)[DISTIME_STORE_INDEX]);
   SalKCDCommand((long long)(BGVAR(s32_ParamsBackup)[KCD_STORE_INDEX]),drive);
   SalClDAxisCompCommand((long long)(BGVAR(s32_ParamsBackup)[KCDQCOMP_STORE_INDEX]),drive);
   SalClKiCommand((long long)(BGVAR(s32_ParamsBackup)[KCI_STORE_INDEX]),drive);
   SalClKivCommand((long long)(BGVAR(s32_ParamsBackup)[KCIV_STORE_INDEX]),drive);
   SalClKpCommand((long long)(BGVAR(s32_ParamsBackup)[KCP_STORE_INDEX]),drive);
   SalClKffCommand((long long)(BGVAR(s32_ParamsBackup)[KCFF_STORE_INDEX]),drive);
   SalDisableModeCommand((long long)BGVAR(s32_ParamsBackup)[DISMODE_STORE_INDEX], drive);
   SalMlminCommand((long long)BGVAR(s32_ParamsBackup)[ML_STORE_INDEX],drive);
   SalMresCommand((long long)BGVAR(s32_ParamsBackup)[MR_STORE_INDEX],drive);

   VAR(AX0_s16_Skip_Flags) &= ~MOTOR_SETUP_MASK;
}


void ParametersEstimationHandler(int drive)
{
   // AXIS_OFF;
   unsigned int u16_timer_capture_3125;
   int s16_ret_val ,s16_iq_act, s16_vq_actual, s16_delay_time = 0;
   float f_vq_actual, f_i_amp_actual;
   long s32_actual_velocity;
   long s32_temp;

   switch (BGVAR(s16_Param_Est_State))
   {
      case PARAM_EST_STATE_IDLE:
      case PARAM_EST_STATE_DONE:
      case PARAM_EST_STATE_FAULT:
      break;

      case PARAM_EST_STATE_SETUP:
         BGVAR(s32_ParamsBackup)[ILIM_STORE_INDEX]    = BGVAR(s32_Ilim_User);
         BGVAR(s32_ParamsBackup)[OPMODE_STORE_INDEX]  = (long)VAR(AX0_s16_Opmode);
         BGVAR(s32_ParamsBackup)[FB_OPMODE_STORE]     = (long)BGVAR(s16_CAN_Opmode);
         BGVAR(s32_ParamsBackup)[MLGAINC_STORE_INDEX] = (long)BGVAR(s16_Mcont_Current_Gain);
         BGVAR(s32_ParamsBackup)[MLGAINP_STORE_INDEX] = (long)BGVAR(s16_Mpeak_Current_Gain);
         BGVAR(s32_ParamsBackup)[DISTIME_STORE_INDEX] = (long)BGVAR(u16_DisTime);
         BGVAR(s32_ParamsBackup)[KCBEMF_STORE_INDEX]  = BGVAR(s32_CL_Bemf_Gain);
         BGVAR(s32_ParamsBackup)[KCD_STORE_INDEX]     = BGVAR(u32_Kcd_Gain);
         BGVAR(s32_ParamsBackup)[KCDQCOMP_STORE_INDEX]= BGVAR(s32_CL_DQ_Axis_Comp_Gain);
         BGVAR(s32_ParamsBackup)[KCI_STORE_INDEX]     = BGVAR(s32_CL_Ki_User);
         BGVAR(s32_ParamsBackup)[KCIV_STORE_INDEX]    = BGVAR(s32_CL_Kiv_User);
         BGVAR(s32_ParamsBackup)[KCP_STORE_INDEX]     = BGVAR(s32_CL_Kp_User);
         BGVAR(s32_ParamsBackup)[KCFF_STORE_INDEX]    = BGVAR(s32_CL_Kff_User);
         BGVAR(s32_ParamsBackup)[DISMODE_STORE_INDEX] = (long)BGVAR(u16_Disable_Mode);
         BGVAR(s32_ParamsBackup)[ML_STORE_INDEX]      = BGVAR(u32_Mlmin);
         BGVAR(s32_ParamsBackup)[MR_STORE_INDEX]      = BGVAR(u32_Motor_Res);
         BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX]= (long)VAR(AX0_s16_Motor_Enc_Type);

         BGVAR(s32_Param_Est_ML) = BGVAR(u32_Mlmin);
         BGVAR(s32_Param_Est_MR) = BGVAR(u32_Motor_Res);

         SetOpmode(drive, 2);
         SalMcontCurrentGainCommand(1000LL,drive);
         SalMpeakCurrentGainCommand(1000LL,drive);
         SalClBemfCommand(0LL,drive);
         SalDisableModeCommand(0LL, drive);
         SalMlminCommand(1000LL,drive);
         SalMresCommand(0LL,drive);
         // This is to allow enable without phase find
         if ((BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX] >= 1) && (BGVAR(s32_ParamsBackup)[MENCTYPE_STORE_INDEX] <= 4))
         {
           BGVAR(s64_Faults_Mask) &= ~PHASE_FIND_FLT_MASK;
           BGVAR(u16_PhaseFind_Bits) &= ~PHASE_FIND_REQUESTED_MASK;
           VAR(AX0_s16_PhaseFindRTBits)= 0;
           VAR(AX0_s16_Enc_State_Ptr) = (int)((long)&ENC_STATE_0_INC_ENC & 0xffff);
         }
         s32_Param_Est_Icmd = BGVAR(s32_Motor_I_Cont);
         if (s32_Param_Est_Icmd > (BGVAR(s32_Drive_I_Peak) >> 1L)) s32_Param_Est_Icmd = (long)((float)(BGVAR(s32_Imax) * 0.85));
         s16_Param_Est_Cntr = 0;
         BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_CONFIG;
      break;

      case PARAM_EST_STATE_CONFIG:
         s16_ret_val = SalConfigCommand(drive);
         if (s16_ret_val == SAL_NOT_FINISHED) break;
         if (s16_ret_val == SAL_SUCCESS)
         {
            SalIlimCommand((long long)BGVAR(s32_Imax), drive);

            // change opmode for fieldbus and internaly
            if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
            {
                 BGVAR(s16_CAN_Opmode) = PROFILE_TORQUE_MODE;
                 BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
            }
            else
            {
                 SetOpmode(drive, 2);
            }
            VAR(AX0_s16_Dead_Time_Comp) = 0;

            // Lock rotor. Set up elect angle to 0
            VAR(AX0_s16_Skip_Flags) |= MOTOR_SETUP_MASK;
            VAR(AX0_s16_Elect_Pos_With_Phase_Adv) = 0;

            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_WAIT_FOR_EN;
         }
         else
         {
            ParamEstRestore(drive);
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_RESTORE_AND_FAULT;
         }
      break;

      case PARAM_EST_STATE_WAIT_FOR_EN:
         if (!Enabled(drive)) return;
         BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_RAMP_CURRENT;
      break;

      case PARAM_EST_STATE_RAMP_CURRENT:
         if (BGVAR(s32_Serial_Torque_Cmd) < (s32_Param_Est_Icmd >> 1))
         {
            InternalTorqueCommand((long long)BGVAR(s32_Serial_Torque_Cmd) + 10, drive);
            BGVAR(s32_Param_Est_Timer) = Cntr_1mS;
            break;
         }
         // Wait for the motor to stop moving (or timeout)
         s32_actual_velocity = LVAR(AX0_s32_Vel_Var_Fb_0);
         if (s32_actual_velocity < 0L) s32_actual_velocity = -s32_actual_velocity;
         if (s32_actual_velocity > 50000L) BGVAR(s32_Param_Est_Timer2) = Cntr_1mS;

         if ((PassedTimeMS(10000L,BGVAR(s32_Param_Est_Timer))) || (PassedTimeMS(1500L,BGVAR(s32_Param_Est_Timer2))))
         {
            s16_Param_Est_Vq_At_Imax = s16_Param_Est_Vq_At_Quarter_Imax = -1;
            InternalTorqueCommand((long long)s32_Param_Est_Icmd,drive);
            BGVAR(s32_Param_Est_Timer) = Cntr_1mS;
            s16_Vq_Filtered = 0;
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_WAIT_FOR_FILTER;
         }
      break;

      case PARAM_EST_STATE_WAIT_FOR_FILTER:
         if (!Enabled(drive))
         {
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_RESTORE_AND_FAULT;
            ParamEstRestore(drive);
            return;
         }

         if (!PassedTimeMS(100L, BGVAR(s32_Param_Est_Timer))) return;

         if (s16_Param_Est_Vq_At_Imax < 0)
         {
               s16_Param_Est_Vq_At_Imax = s16_Vq_Filtered;
               LowerCurrentto0(drive);
               s16_Vq_Filtered = 0;
               InternalTorqueCommand((long long)(s32_Param_Est_Icmd>>2),drive);
               BGVAR(s32_Param_Est_Timer) = Cntr_1mS;
         }
         else
         {
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_ML_DISABLE;
            s16_Param_Est_Vq_At_Quarter_Imax = s16_Vq_Filtered;

            DisableCommand(drive);

            ParamEstCalcMr();

            BGVAR(s32_Param_Est_Timer) = Cntr_1mS;
         }
      break;

      case PARAM_EST_STATE_ML_DISABLE:
         if (!Enabled(drive)) BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_ML_SETUP;
         else if (PassedTimeMS(100L + (long)BGVAR(u16_DisTime), BGVAR(s32_Param_Est_Timer)))
         {
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_RESTORE_AND_FAULT;
            ParamEstRestore(drive);
            return;
         }
      break;

      case PARAM_EST_STATE_ML_SETUP:
         BGVAR(u16_DisTime) = 0;
         SalKCDCommand(0LL,drive);
         SalClDAxisCompCommand(0LL,drive);
         SalClKiCommand(0LL,drive);
         SalClKivCommand(0L,drive);
         SalClKpCommand(0LL,drive);
         SalClKffCommand(1000LL,drive);
         BGVAR(s16_Current_Value) = 0;

         BGVAR(s16_Max_Current) = ((long long)min(BGVAR(s32_Drive_I_Cont), BGVAR(s32_Motor_I_Cont)) * BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix)
               >> (long long)BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr;

         BGVAR(s16_Local_Max_Current) = 0;
         BGVAR(s16_Local_Max_Cntr) = 0;
         BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_ML_CONFIG;
      break;

      case PARAM_EST_STATE_ML_CONFIG:
         s16_ret_val = SalConfigCommand(drive);
         if (s16_ret_val == SAL_NOT_FINISHED) break;
         if (s16_ret_val == SAL_SUCCESS)
         {
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_ML_DELAY_AFTER_CONFIG;
            BGVAR(s32_Param_Est_Timer) = Cntr_1mS;
         }
         else
         {
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_RESTORE_AND_FAULT;
            ParamEstRestore(drive);
         }
      break;

      case PARAM_EST_STATE_ML_DELAY_AFTER_CONFIG:
         if (FEEDBACK_ENDAT) s16_delay_time = 3000;
         if (PassedTimeMS((long)s16_delay_time, BGVAR(s32_Param_Est_Timer)))
         {
            EnableCommand(drive);
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_ML_ENABLE;
            BGVAR(s32_Param_Est_Timer) = Cntr_1mS;
         }
      break;

      case PARAM_EST_STATE_ML_ENABLE:
         if (!Enabled(drive))
         {
            if (!PassedTimeMS(1000L, BGVAR(s32_Param_Est_Timer))) return;
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_RESTORE_AND_FAULT;
            ParamEstRestore(drive);
            return;
         }
         BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_ML_CURR_INC;
      break;

      case PARAM_EST_STATE_ML_CURR_INC:
         if (!Enabled(drive))
         {
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_RESTORE_AND_FAULT;
            ParamEstRestore(drive);
            return;
         }

         // Increase current (which means increase voltage as only KCFF is used)
         BGVAR(s16_Current_Value) += 500;

         // Issue command (for one current loop cycle) and meas resultant current.
         WaitForNext32kHzTaskStart();
         InternalTorqueCommand((long long)BGVAR(s16_Current_Value), drive);
         if(VAR(AX0_s16_Crrnt_Q_Ref_0)==0)
         {
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
         }

         if (BGVAR(u32_Pwm_Freq) == 8000L)
         {
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
         }
         // This is theoretical assumption, need to be verified when ML/MR measuring module will be on
         else if (BGVAR(u32_Pwm_Freq) == 4000L)
         {
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
         }         

         s16_vq_actual = VAR(AX0_s16_Vq);

         u16_timer_capture_3125 = Cntr_3125;
         while (u16_timer_capture_3125 == Cntr_3125){}
         u16_timer_capture_3125 = Cntr_3125;
         while (u16_timer_capture_3125 == Cntr_3125){}

         // If PWMFRQ=8 wait another sample
         if (BGVAR(u32_Pwm_Freq) == 8000L)
         {
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
         }
         // If PWMFRQ=4 wait another sample
         // This is theoretical assumption, need to be verified when ML/MR measuring module will be on
         else if (BGVAR(u32_Pwm_Freq) == 4000L)
         {
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
         }         

         s16_iq_act = VAR(AX0_s16_Crrnt_Q_Act_0); // This value is also used for saliency calculation in next case
         InternalTorqueCommand(0LL, drive);

         // V - Dead_time - IR = Ldi/dt
         f_i_amp_actual = (float)s16_iq_act*(float)BGVAR(s32_Drive_I_Peak) / 26214000.0;
         f_vq_actual = (float)s16_vq_actual * (float)BGVAR(u16_Vbus_Volts) / (float)VAR(AX0_s16_Pwm_Half_Period) - BGVAR(s16_Param_Est_V_Dead_Time) - f_i_amp_actual*(float)(BGVAR(s32_Param_Est_MR)) / 1000.0;

         if (f_i_amp_actual <= 0)
            s32_temp = -1L;
         else
         {
            if (BGVAR(u32_Pwm_Freq) == 4000L) // This is theoretical assumption, need to be verified when ML/MR measuring module will be on
               s32_temp = (long)(f_vq_actual / f_i_amp_actual*125 + 0.5);            
            else if (BGVAR(u32_Pwm_Freq) == 8000L)
               s32_temp = (long)(f_vq_actual / f_i_amp_actual*62.5 + 0.5);
            else
               s32_temp = (long)(f_vq_actual / f_i_amp_actual*31.25 + 0.5);

            // Round ML value
            if (s32_temp >= 50L) s32_temp = ((s32_temp + 50L) / 100L) * 100L;
         }

         // Test when the increase in voltage does not cause an increase in current
         // by checking of 3 increases there is not change in Max value
         if (s16_iq_act > BGVAR(s16_Local_Max_Current))
         {
            BGVAR(s16_Local_Max_Cntr) = 0;
            BGVAR(s16_Local_Max_Current) = s16_iq_act;
            BGVAR(s32_Param_Est_ML) = s32_temp;
         }
         else BGVAR(s16_Local_Max_Cntr)++;

         // Stop increasing voltage when max current does not increase for 3 conseq samples
         if ((s16_iq_act > (BGVAR(s16_Max_Current) >> 1)) || ((float)BGVAR(s16_Current_Value)*CL_KFF_FACTOR > (float)VAR(AX0_s16_Pwm_Half_Period)) || (BGVAR(s16_Local_Max_Cntr)>=3) || (BGVAR(s16_Current_Value) > (BGVAR(s32_Ilim_Actual)-500L)) )
         {
            s16_Param_Est_Sal_Iq = s16_iq_act;

            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_SALIENCY_MEAS;
            BGVAR(s32_Param_Est_Timer) = Cntr_1mS;
         }
      break;

      case PARAM_EST_STATE_SALIENCY_MEAS:
         if (!Enabled(drive))
         {
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_RESTORE_AND_FAULT;
            ParamEstRestore(drive);
            return;
         }
         if (!PassedTimeMS(25L, BGVAR(s32_Param_Est_Timer))) return;

         // Issue command (for one current loop cycle) and meas resultant current.
         VAR(AX0_s16_DQ_Axis_Switch) = 1;// Switch from Q to D axis in RT

         WaitForNext32kHzTaskStart();
         InternalTorqueCommand((long long)BGVAR(s16_Current_Value), drive);

         if(VAR(AX0_s16_Crrnt_D_Ref_0)==0)
         {
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
         }
         u16_timer_capture_3125 = Cntr_3125;
         while (u16_timer_capture_3125 == Cntr_3125){}

         u16_timer_capture_3125 = Cntr_3125;
         while (u16_timer_capture_3125 == Cntr_3125){}

         // If PWMFRQ=8 wait another sample
         if (BGVAR(u32_Pwm_Freq) == 8000L)
         {
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
         }
         // If PWMFRQ=4 wait another sample
          // This is theoretical assumption, need to be verified when ML/MR measuring module will be on
         else if (BGVAR(u32_Pwm_Freq) == 4000L)
         {
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
            u16_timer_capture_3125 = Cntr_3125;
            while (u16_timer_capture_3125 == Cntr_3125){}
         }         

         s16_Param_Est_Sal_Id = VAR(AX0_s16_Crrnt_D_Act_0);
         InternalTorqueCommand(0LL, drive);

         BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_SALIENCY_CALC;
      break;

      case PARAM_EST_STATE_SALIENCY_CALC:
         BGVAR(s32_Param_Est_SLfactor) = (long)((float)s16_Param_Est_Sal_Id)*1000.0/((float)s16_Param_Est_Sal_Iq);

         if ((BGVAR(s32_Param_Est_SLfactor)> 1000L) && (s16_Param_Est_Cntr <= 5))
         {
               s16_Param_Est_Cntr++;
               VAR(AX0_s16_DQ_Axis_Switch) = 0; // Switch back from D to Q axis in RT
               DisableCommand(drive);
               BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_ML_DISABLE;
               BGVAR(s32_Param_Est_Timer) = Cntr_1mS;
               break;
         }

         DisableCommand(drive);
         VAR(AX0_s16_DQ_Axis_Switch) = 0;// Switch back from D to Q axis in RT
         s16_Param_Est_Cntr = 0;
         BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_ML_WAIT_FOR_DISABLE;
      break;
      case PARAM_EST_STATE_ML_WAIT_FOR_DISABLE:
         if (!Enabled(drive))
         {
            ParamEstRestore(drive);
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_ML_SETUP_COMPLETE;
         }
      break;

      case PARAM_EST_STATE_RESTORE_AND_FAULT:
      case PARAM_EST_STATE_ML_SETUP_COMPLETE:
         s16_ret_val = SalConfigCommand(drive);
         if (s16_ret_val == SAL_NOT_FINISHED) break;

         if ((s16_ret_val == SAL_SUCCESS) && (BGVAR(s16_Param_Est_State) == PARAM_EST_STATE_ML_SETUP_COMPLETE))
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_DONE;
         else
            BGVAR(s16_Param_Est_State) = PARAM_EST_STATE_FAULT;
      break;
   }
}


#pragma CODE_SECTION(ParametersEstimationFilter, "ramfunc_2");
void ParametersEstimationFilter(void)
{
//    #define PARAM_EST_FILT_ALPHA 0x7834// 10 Hz
//    #define PARAM_EST_FILT_BETA 0x7CC

   #define PARAM_EST_FILT_ALPHA 0x6D64// 25 Hz
   #define PARAM_EST_FILT_BETA 0x129C

   int drive = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s16_Param_Est_State) != PARAM_EST_STATE_WAIT_FOR_FILTER) return;
   s16_Vq_Filtered = (int)((PARAM_EST_FILT_ALPHA * (long)s16_Vq_Filtered + PARAM_EST_FILT_BETA * (long)AX0_s16_Vq + 0x4000) >> 15);
}
*/
int SalEstMotorParamsCommand(long long param,int drive)
{
   if ((int)param > 0)
   {
      if (Enabled(drive)) return (DRIVE_ACTIVE);
      if (BGVAR(s64_SysNotOk) & NO_COMP_FLT_MASK) return (INVALID_CONFIG);
      if ( (ProcedureRunning(DRIVE_PARAM) != PROC_NONE) && (ProcedureRunning(DRIVE_PARAM) != PROC_ML_EST_AUTOTUNE) )
         return OTHER_PROCEDURE_RUNNING;

      BGVAR(u16_Est_Motor_Params) = 1;
      BGVAR(s16_Est_Motor_Params_Avrerage_Cycles_Multiplier) = (int)param;
      BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_INIT;
   }
   else
   {
      BGVAR(u16_Est_Motor_Params) = 0;
      DisableCommand(drive);
      MotorParametersEstimationFailureProcedure(drive);
      s16_Motor_Params_Est_State = MOTOR_PARAMS_EST_STATE_IDLE;
   }

  return SAL_SUCCESS;
}


int SalEstMotorParamsStCommand(int drive)
{

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s16_Motor_Params_Est_State) == MOTOR_PARAMS_EST_STATE_IDLE)
   {
      PrintStringCrLf("Process had not been initiated",0);
      return SAL_SUCCESS;
   }

   if (BGVAR(s16_Motor_Params_Est_State) == MOTOR_PARAMS_EST_STATE_DONE)
   {
      PrintStringCrLf("Process finished successfully",0);
      PrintString("Mr: ",0); PrintString(DecimalPoint32ToAscii(BGVAR(s32_Motor_Params_Est_Mr)),0); PrintString(" Ohm",0);PrintCrLf();
      PrintString("Ml: ",0); PrintString(DecimalPoint32ToAscii(BGVAR(s32_Motor_Params_Est_Ml)),0); PrintString(" mH",0); PrintCrLf();

      return SAL_SUCCESS;
   }

   if (BGVAR(s16_Motor_Params_Est_State) == MOTOR_PARAMS_EST_STATE_WAIT_FOR_ENABLE)
   {
      PrintStringCrLf("Process initiated, waiting for enable",0);
      return SAL_SUCCESS;
   }

   if (BGVAR(s16_Motor_Params_Est_State) > MOTOR_PARAMS_EST_STATE_WAIT_FOR_ENABLE)
   {
      PrintStringCrLf("Process in progress",0);
      PrintString("Stage ",0); PrintSignedInt16(BGVAR(s16_Est_Motor_Params_Number_Of_Cycles_To_Average) - BGVAR(s16_Est_Motor_Params_Average_Counter)); PrintString(" / ",0); PrintSignedInt16(BGVAR(s16_Est_Motor_Params_Number_Of_Cycles_To_Average));PrintString("\n",0);
      return SAL_SUCCESS;
   }

   if (BGVAR(s16_Motor_Params_Est_State) < 0 )
   {
      PrintStringCrLf("Process failed or aborted",0);
      PrintString("Stage ",0);PrintSignedInt16(BGVAR(s16_Est_Motor_Params_Number_Of_Cycles_To_Average) - BGVAR(s16_Est_Motor_Params_Average_Counter));PrintString(" / ",0);PrintSignedInt16(BGVAR(s16_Est_Motor_Params_Number_Of_Cycles_To_Average));PrintString("\n",0);
      PrintString("Internal stage ",0); PrintSignedInt16(-BGVAR(s16_Motor_Params_Est_State)); //when failed or aborted, the stage is saved in its negative value
      PrintString("\n",0);
      return SAL_SUCCESS;
   }

   return SAL_SUCCESS;
}


void MotorParametersEstimationFailureProcedure(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   BGVAR(u16_Force_Regular_Pwm_Freq) = 0;
   BGVAR(u16_Est_Motor_Params) = 0;
   VAR(AX0_u16_Motor_Params_Detection_Process) = MOTOR_PARAMS_DETECTION_INACTIVE_MASK;
   BGVAR(s16_Motor_Params_Est_State) *= -1;
}


void MotorParametersEstimationHandler(int drive)
{
   // AXIS_OFF;

   static long s32_timeout_time_capture;
   static unsigned int u16_pwm_quarter_precent;
   static unsigned int u16_winner_pwm;
   static int s16_current_stability_th_backup;
   static float f_pwm_set_current_th;
   float f_numerator;
   float f_denumarator;
   float f_tau_in_seconds;
   static int s16_tau_measure_retry;
   float f_mr;
   float f_ml;
   int s16_current_is_stable;
   float f_delta_tau;

   if (BGVAR(s16_Motor_Params_Est_State) < 0) //checking if process failed
      return;

   if (  (!Enabled(drive))                                                            && //checing if fault occured / user disabled the drive during the process or
         (BGVAR(s16_Motor_Params_Est_State) > MOTOR_PARAMS_EST_STATE_WAIT_FOR_ENABLE) &&
         (BGVAR(s16_Motor_Params_Est_State) != MOTOR_PARAMS_EST_STATE_DONE)   )//this condition is relevant when fault occure or if disabled during operation
   {
      MotorParametersEstimationFailureProcedure(drive);
   }

   //determine current stability
   if (VAR(AX0_u16_Motor_Params_Detection_Process) & MOTOR_PARAMS_DETECTION_CHECK_CURRENT_STABILITY_MASK)
   {
      if ( (VAR(AX0_s16_Motor_Params_Est_Current_Stable) == VAR(AX0_s16_Motor_Params_Est_Current_Stability_TH)) &&
           (VAR(AX0_s16_Iu_Derivative_Acumulator) <= VAR(AX0_s16_Motor_Params_Est_Derive_Max_Noise))            &&
           (VAR(AX0_s16_Iu_Derivative_Acumulator) >= VAR(AX0_s16_Motor_Params_Est_Derive_Min_Noise))                  )
      {
         s16_current_is_stable = 1;
         s32_timeout_time_capture = Cntr_1mS; //capturing current time for timeout protection, all stage that are checking current stability are right after each other
      }
      else
      {
         if (PassedTimeMS(10000L ,s32_timeout_time_capture))  //allowing 10s until timeout
         {
            s16_current_is_stable = 1; //assuming that after the timeout the current is stable for sure (otherwise the motor is not in the spec)
            s32_timeout_time_capture = Cntr_1mS; //capturing current time for timeout protection, all stage that are checking current stability are right after each other
         }
         else
            s16_current_is_stable = 0;
      }
   }

   //state machine
   switch (BGVAR(s16_Motor_Params_Est_State))
   {
      case MOTOR_PARAMS_EST_STATE_IDLE :
      break;

      case MOTOR_PARAMS_EST_STATE_INIT:
         //initiating results
         BGVAR(s32_Motor_Params_Est_Mr) = 0;
         BGVAR(s32_Motor_Params_Est_Ml) = 0;

         u16_pwm_quarter_precent = (unsigned int)(0.25*(float)(VAR(AX0_s16_Pwm_Half_Period) * 2.0) / 100.0); //0.25% pwm

         if (BGVAR(s32_Motor_I_Cont) < BGVAR(s32_Drive_I_Cont))         //checking what would be the target current
            f_pwm_set_current_th = 0.9 * 32768.0 * (float)s32_Motor_I_Cont/((float)s32_Drive_I_Peak*1.25); //90% micont
         else
            f_pwm_set_current_th = 0.9 * 32768.0 * (float)s32_Drive_I_Cont/((float)s32_Drive_I_Peak*1.25); //90% dicont

         VAR(AX0_s16_Current_Derive_Mts_Gap) = 32000 / BGVAR(u32_Pwm_Freq) - 1; //setting the required MTS gap for the real time cycle time according to PWM freq
         BGVAR(u16_Force_Regular_Pwm_Freq) = 1;                                  //disabling pwm change during operation
         VAR(AX0_s16_Motor_Params_Est_Current_Stability_TH) = 100;               //setting current stability TH to 100 PWM cycles
         s16_current_stability_th_backup  =  VAR(AX0_s16_Motor_Params_Est_Current_Stability_TH);
         VAR(AX0_s16_Motor_Params_Est_Derive_Min_Noise) = 32767;                 //initiating min derive value before measurement
         VAR(AX0_s16_Motor_Params_Est_Derive_Max_Noise) = -32768;                //initiating max derive value before measurement
         VAR(AX0_s16_Motor_Params_Est_Current_To_Capture) = 0;                   //nulling captured current settings and results
         VAR(AX0_s16_Motor_Params_Est_Captured_Current) = 0;
         VAR(AX0_s16_Motor_Params_Est_Captured_Current_Derivaitve) = 0;
         VAR(AX0_s16_Motor_Params_Est_Current_Stable) = 0;                      //nulling stable current indicator
         VAR(AX0_s16_Motor_Params_Est_Tau) = 0;                                  //nulling tau measurement
         VAR(AX0_u16_Motor_Params_Est_VW_PWM) = (unsigned int)(VAR(AX0_s16_Pwm_Half_Period) * 2);  //setting V and W to 100% PWM
         VAR(AX0_u16_Motor_Params_Est_U_PWM) = VAR(AX0_u16_Motor_Params_Est_VW_PWM);               //ensuring 0 voltage
         u16_winner_pwm = VAR(AX0_u16_Motor_Params_Est_U_PWM);                                     //initializing pwm operating point
         BGVAR(s16_Est_Motor_Params_Number_Of_Cycles_To_Average) = 20 * BGVAR(s16_Est_Motor_Params_Avrerage_Cycles_Multiplier); //number of cycles to average
         BGVAR(s16_Est_Motor_Params_Average_Counter) = BGVAR(s16_Est_Motor_Params_Number_Of_Cycles_To_Average);                 //initializing average counter
         s16_tau_measure_retry = 0;    //nulling tau measurements retry counter for "close to 0 derivative" condition in the tau estimation
         VAR(AX0_u16_Motor_Params_Detection_Process) = MOTOR_PARAMS_DETECTION_ACTIVE_MASK; // = 1 - set bit 0 to enable special current loop for the process
         VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_NEW_PWM_VALUE_MASK; // set bit 5 to have this PWM values sent
         BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_WAIT_FOR_ENABLE;
      break;

      case MOTOR_PARAMS_EST_STATE_WAIT_FOR_ENABLE:
         if (Enabled(drive))
         {
            VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_MEASURE_DERIVE_NOISE_MASK; // = 3 - set bit 1 in addition to bit 0 to initiate the derivative noise estimation
            BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_MEASURE_DERIVE_NOISE;
         }
      break;

      case MOTOR_PARAMS_EST_STATE_MEASURE_DERIVE_NOISE:
         if (AX0_s16_Motor_Params_Est_Current_Stability_TH <= -9 * s16_current_stability_th_backup ) //waiting for min and max to be determined for at least 10 periods of the TH (one will reduce it to 0 and additonal 9)
         {
            VAR(AX0_u16_Motor_Params_Detection_Process) -= MOTOR_PARAMS_DETECTION_MEASURE_DERIVE_NOISE_MASK; //clear the derive noise estimation process
            VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_CHECK_CURRENT_STABILITY_MASK; // set the current stability check bit
            VAR(AX0_u16_Motor_Params_Est_U_PWM) -= u16_pwm_quarter_precent;                         //applying minimal voltage to create cuurent
     /**/   VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_NEW_PWM_VALUE_MASK; // set bit 5 to have this PWM values sent
            VAR(AX0_s16_Motor_Params_Est_Current_Stability_TH) = s16_current_stability_th_backup; //reassigning AX0_s16_Motor_Params_Est_Current_Stability_TH to its original value after reductions
            s32_timeout_time_capture = Cntr_1mS; //capturing current time for timeout protection
            BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_SET_PWM_FOR_90_PERCENT_OF_ICONT;
         }
      break;

      case MOTOR_PARAMS_EST_STATE_SET_PWM_FOR_90_PERCENT_OF_ICONT:
         if (BGVAR(s16_Est_Motor_Params_Average_Counter) == 0 ) //checking if we finsihed all the measuremetns
         {
            BGVAR(u16_Force_Regular_Pwm_Freq) = 0;
            VAR(AX0_u16_Motor_Params_Detection_Process) = MOTOR_PARAMS_DETECTION_INACTIVE_MASK;
            VAR(AX0_u16_Motor_Params_Est_U_PWM) = VAR(AX0_u16_Motor_Params_Est_VW_PWM);               //ensring 0 voltage
     /**/   VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_NEW_PWM_VALUE_MASK; // set bit 5 to have this PWM values sent
            BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_DONE;
            break;
         }

         //if we got here, we need to measure again
         if (s16_current_is_stable) //current derivative was stable for 100 pwm cycles
         {
            if (f_pwm_set_current_th <= (float)VAR(AX0_s16_Previous_Iu)) //checking if we reached the desired threshold
            {
               //storing the required data
               BGVAR(s16_Motor_Params_Est_I_90_Percent_Icont) = VAR(AX0_s16_Previous_Iu);
               BGVAR(s16_Motor_Params_Est_V_90_Percent_Icont) = VAR(AX0_u16_Motor_Params_Est_U_PWM);
               VAR(AX0_u16_Motor_Params_Est_U_PWM) -= u16_pwm_quarter_precent;
        /**/   VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_NEW_PWM_VALUE_MASK; // set bit 5 to have this PWM values sent
               BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_SET_PWM_BEYOND_90_PERCENT_OF_ICONT;
            }
            else//threshold was not reached, need to increase voltage
            {
               VAR(AX0_u16_Motor_Params_Est_U_PWM) -= u16_pwm_quarter_precent; //increasing voltage
        /**/   VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_NEW_PWM_VALUE_MASK; // set bit 5 to have this PWM values sent
               if (VAR(AX0_u16_Motor_Params_Est_U_PWM) <= 40 * u16_pwm_quarter_precent) //limiting the voltage to 90% of bus voltage (100%-40*0.25%)
               {
                  MotorParametersEstimationFailureProcedure(drive);
                  break;
               }

            }

            //nulling the accumulators for the next measurement
            VAR(AX0_s16_Iu_Derivative_Acumulator) = 0;
            VAR(AX0_s16_Motor_Params_Est_Current_Stable) = 0;

         }
         //current still not stabilized, need to wait
      break;

      case MOTOR_PARAMS_EST_STATE_SET_PWM_BEYOND_90_PERCENT_OF_ICONT:
         if (s16_current_is_stable) //current derivative was stable for 100 pwm cycles
         {
            //storing the required data
            BGVAR(s16_Motor_Params_Est_I_Beyond_90_Percent_Icont) = VAR(AX0_s16_Previous_Iu) ;
            BGVAR(s16_Motor_Params_Est_V_Beyond_90_Percent_Icont) = VAR(AX0_u16_Motor_Params_Est_U_PWM);
            u16_winner_pwm = VAR(AX0_u16_Motor_Params_Est_U_PWM) ; //storing the required pwm
            VAR(AX0_u16_Motor_Params_Est_U_PWM) = VAR(AX0_u16_Motor_Params_Est_VW_PWM); //0 voltage to reduce current to 0
     /**/   VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_NEW_PWM_VALUE_MASK; // set bit 5 to have this PWM values sent

            //nulling the accumulators for the next measurement
            VAR(AX0_s16_Motor_Params_Est_Current_Stable) = 0;
            VAR(AX0_s16_Iu_Derivative_Acumulator) = 0;
            BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_WAIT_FOR_ZERO_CURRENT;
         }

         //current still not stabilized, need to wait
      break;

      case MOTOR_PARAMS_EST_STATE_WAIT_FOR_ZERO_CURRENT:
         if (s16_current_is_stable) //current derivative was stable for 100 pwm cycles
         {
            VAR(AX0_s16_Motor_Params_Est_Current_To_Capture) =(int)(0.63212 * (float)BGVAR(s16_Motor_Params_Est_I_Beyond_90_Percent_Icont)); //the higher current * (1-e^-1), this is the value after 1 tau

            VAR(AX0_u16_Motor_Params_Detection_Process) -= MOTOR_PARAMS_DETECTION_CHECK_CURRENT_STABILITY_MASK; //clear the current stability check bit

            //nulling the accumulators for the next measurement
            VAR(AX0_s16_Motor_Params_Est_Current_Stable) = 0;
            VAR(AX0_s16_Iu_Derivative_Acumulator) = 0;

            VAR(AX0_s16_Motor_Params_Est_Tau) = 0;
            VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_MEASURE_TAU_MASK; //set the tau measurement bit
            VAR(AX0_u16_Motor_Params_Est_U_PWM) = u16_winner_pwm; //applying the required voltage
     /**/   VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_NEW_PWM_VALUE_MASK; // set bit 5 to have this PWM values sent
            BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_MEASURE_TAU;
         }
      break;

      case MOTOR_PARAMS_EST_STATE_MEASURE_TAU:
        if (VAR(AX0_s16_Previous_Iu) > VAR(AX0_s16_Motor_Params_Est_Current_To_Capture) ) //cheching if we passed tau
        {
           VAR(AX0_u16_Motor_Params_Est_U_PWM) = VAR(AX0_u16_Motor_Params_Est_VW_PWM); //0 voltage to reduce current
    /**/   VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_NEW_PWM_VALUE_MASK; // set bit 5 to have this PWM values sent
           VAR(AX0_u16_Motor_Params_Detection_Process) -= MOTOR_PARAMS_DETECTION_MEASURE_TAU_MASK; //claer the tau measurement bit
           BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_CALCULATE;
        }
      break;

      case MOTOR_PARAMS_EST_STATE_CALCULATE:
         //calculating Mr
         f_numerator = (float)BGVAR(u16_Vbus_Volts) * ((float)(BGVAR(s16_Motor_Params_Est_V_90_Percent_Icont)) - (float)(BGVAR(s16_Motor_Params_Est_V_Beyond_90_Percent_Icont))) / ((float)(VAR(AX0_s16_Pwm_Half_Period)) * 2.0) ;
         f_denumarator = ((float)(BGVAR(s16_Motor_Params_Est_I_Beyond_90_Percent_Icont)) - (float)(BGVAR(s16_Motor_Params_Est_I_90_Percent_Icont))) * (float)(BGVAR(s32_Drive_I_Peak))*0.001* 1.25 / 32768.0;
         f_mr = (2/1.5) * f_numerator /  f_denumarator; // (2/1.5) is the factor to convert for l-l

        /*
           we add an estimation on the real tau according to the required cuurent to capture, the real captured current and the derivative when capture.
           we add this to the pwm cycles we counted until we captured and divide by the pwm frequency since this is our time units
        */

         //tau estimation and inductance calculation
         f_delta_tau = ( (float)VAR(AX0_s16_Motor_Params_Est_Current_To_Capture) - (float)VAR(AX0_s16_Motor_Params_Est_Captured_Current) ) / (float)VAR(AX0_s16_Motor_Params_Est_Captured_Current_Derivaitve) ;

         if ((f_delta_tau >= 1) || (f_delta_tau < 0))   //if the estimated delta is bigger than one pwm cycle or  is negative --> the derivative was to noisey
         {
            s16_tau_measure_retry++;
            if (s16_tau_measure_retry>=5) //we may be in this stage if there was a problem measuring tau (low dertivative)
            {
               MotorParametersEstimationFailureProcedure(drive);
            }
            else
            {
               VAR(AX0_u16_Motor_Params_Est_U_PWM) = VAR(AX0_u16_Motor_Params_Est_VW_PWM); //0 voltage to reduce current to 0
        /**/   VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_NEW_PWM_VALUE_MASK; // set bit 5 to have this PWM values sent
               VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_CHECK_CURRENT_STABILITY_MASK; // raising the current stability bit
               BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_WAIT_FOR_ZERO_CURRENT; //wait for 0 current and retry measuring tau
            }

            break;
         }

         f_tau_in_seconds = ((float)VAR(AX0_s16_Motor_Params_Est_Tau) + f_delta_tau ) / (float)BGVAR(u32_Pwm_Freq);

         //Tau = Ml/Mr
         f_ml = 1000.0 * f_tau_in_seconds * f_mr; // this include conversion to mH

         //adding to average

         BGVAR(s32_Motor_Params_Est_Mr) += (long)(1000.0*f_mr/(float)BGVAR(s16_Est_Motor_Params_Number_Of_Cycles_To_Average));
         BGVAR(s32_Motor_Params_Est_Ml) += (long)(1000.0*f_ml/(float)BGVAR(s16_Est_Motor_Params_Number_Of_Cycles_To_Average));

         BGVAR(s16_Est_Motor_Params_Average_Counter) = BGVAR(s16_Est_Motor_Params_Average_Counter) - 1; //decrementing average counter befor next itteration

         //reinitiating required parameters (see explanation in the INIT stage)
         VAR(AX0_s16_Motor_Params_Est_Current_Stability_TH) = 100;
         s16_current_stability_th_backup  =  VAR(AX0_s16_Motor_Params_Est_Current_Stability_TH);
         VAR(AX0_s16_Motor_Params_Est_Tau) = 0;
         VAR(AX0_u16_Motor_Params_Est_VW_PWM) = (unsigned int)(VAR(AX0_s16_Pwm_Half_Period) * 2);
         VAR(AX0_u16_Motor_Params_Est_U_PWM) = u16_winner_pwm + u16_pwm_quarter_precent; //no need to check again whic PWM to issue, we can assign directly
  /**/   VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_NEW_PWM_VALUE_MASK; // set bit 5 to have this PWM values sent
         s16_tau_measure_retry = 0;
         VAR(AX0_s16_Motor_Params_Est_Current_Stable) = 0;
         VAR(AX0_s16_Iu_Derivative_Acumulator) = 0;

         s32_timeout_time_capture = Cntr_1mS; //capturing current time for timeout protection
         VAR(AX0_u16_Motor_Params_Detection_Process) += MOTOR_PARAMS_DETECTION_CHECK_CURRENT_STABILITY_MASK; //set bit 0 and 2 to enable special curennt loop for the process with current stability check
         BGVAR(s16_Motor_Params_Est_State) = MOTOR_PARAMS_EST_STATE_SET_PWM_FOR_90_PERCENT_OF_ICONT;
      break;


      case (MOTOR_PARAMS_EST_STATE_DONE):
         if (!Enabled(drive))           //if after done, k is issued, turn off the indication
            BGVAR(u16_Est_Motor_Params) = 0;
      break;

   }
}


