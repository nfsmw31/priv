#include "cal_conf.h"
#include <co_emcy.h>

#include "Math.h"
#include "co_util.h"
#include "objects.h"
#include "402fsa.def"
#include "Init.def"
#include "ExFbVar.def"  //ITAI -  Position 125 usec



#include "Err_Hndl.def"
#include "MultiAxis.def"
#include "Position.def"
#include "PtpGenerator.def"
#include "Homing.def"
#include "i2c.def"
#include "Exe_IO.def"
#include "design.def"
#include "Homing.def"

#include "Drive.var"
#include "Extrn_Asm.var"
#include "PtpGenerator.var"
#include "Ser_Comm.var"
#include "Units.var"
#include "Velocity.def"
#include "Velocity.var"
#include "Position.var"
#include "ModCntrl.var"
#include "Motor.var"
#include "Homing.var"
#include "User_Var.var"
#include "Lxm_Profile.var"
#include "FltCntrl.var"
#include "init.var"
#include "ExFbVar.var"
#include "Exe_IO.var"
//#include "MotorSetup.var"
#include "MotorParamsEst.var"

#include "Prototypes.pro"


void InitiatePtpMove(int drive, long long s64_target_position, long s32_vel, long s32_acc, long s32_dec, int s16_transition_mode)
{
   // AXIS_OFF;
   unsigned int u16_temp_time;   
   BGVAR(s32_Stopped_Armed_Timer) = Cntr_1mS;
   BGVAR(u16_Stopped_Flag_Armed) = 1;
   REFERENCE_TO_DRIVE;

   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Target_Position_Lo_Bg_Shadow) = s64_target_position;
   } while (u16_temp_time != Cntr_3125);
   LVAR(AX0_s32_Vlimit_Bg_Shadow) = s32_vel;
   LVAR(AX0_s32_Ptp_Dec_Rounded_Bg_Shadow) = s32_dec;
   LVAR(AX0_s32_Ptp_Acc_Rounded_Bg_Shadow) = s32_acc;
   VAR(AX0_s16_Ptp_Transition_Mode_Bg) = s16_transition_mode;
}


int ExecuteMoveCommand(int drive, long long s64_target_position, long long s64_profile_velocity, long long s64_transition_mode)
{
   int s16_paramInvalid;
   return ExecuteMoveCommandInternal(drive, s64_target_position, s64_profile_velocity, s64_transition_mode, &s16_paramInvalid);
}


// nitsan:
// for =s= lexium drive profile, we need to distingish which parameter caused the motin command to fail (if return !=SAL_SUCCESS).
// s16_paramInvalid is output param with values:
//    0: other param is invalid (opmode, etc').
//    1: velocity param is invalid
//    2: position param is invalid
int ExecuteMoveCommandInternal(int drive, long long s64_target_position, long long s64_profile_velocity, long long s64_transition_mode, int* s16_paramInvalid)
{
    // AXIS_OFF;
    long long s64_distance, s64_temp, res, s64_local_temp_pos_ls;
    long long s64_pcmd = 0;

    long s32_temp_vel;
    int s16_transition_mode, SW_ls, HW_ls, Pos_Ls_Subs;
    unsigned int u16_temp_time;

    unsigned int u16_temp_cw_ls;
    unsigned int u16_temp_ccw_ls;

    //  When setting Limits Mode to ignore Transient (Phantom) Bits
    if ((VAR(AX0_u16_SW_Pos_lim) & 0x02) == 0x02)
    {
        //  Do not use the hardware limit-switch phantom bits
        u16_temp_cw_ls  = VAR(AX0_u16_CW_LS)  & 0x03;
        u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS) & 0x03;
    }
    else
    {
        //  Involve the hardware limit-switch phantom bits
        u16_temp_cw_ls  = VAR(AX0_u16_CW_LS);
        u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS);
    }

    //  s64_target_position need to be saved unchanged so it can be used in "s64_Temp_Pos_LS"
    //  only if the movement command is valid and the function not returning with error.
    s64_local_temp_pos_ls = s64_target_position;

    *s16_paramInvalid = 0;

    if (VAR(AX0_s16_Opmode) != 8)
    {
        return INVALID_OPMODE;
    }

    if (VAR(AX0_s16_Ptp_Transition_Mode_Bg) != PTP_IDLE_TRANSITION)
    {
        return MOTION_PENDING;
    }

    if ((s64_transition_mode < 1) || (s64_transition_mode > 3))
    {
        return INVALID_PTP_MODE;
    }

    s16_transition_mode = (int)s64_transition_mode;

    *s16_paramInvalid = 1;

    if (s64_profile_velocity <= 0)
    {
        return VALUE_TOO_LOW;
    }

    if (!IS_DUAL_LOOP_ACTIVE)
    {
        if (s64_profile_velocity > (long long)BGVAR(s32_V_Lim_Design))
        {
            return VALUE_TOO_HIGH;
        }
    }
    else
    {
        if (s64_profile_velocity > (long long)BGVAR(s32_Sfb_V_Lim))
        {
            return VALUE_TOO_HIGH;
        }
    }
   
    s32_temp_vel = (long)s64_profile_velocity;

    //////////////////////////////////////////////////////////////////
    //  Dealing with the User Hold command
    //////////////////////////////////////////////////////////////////

    *s16_paramInvalid = 0;

    if (BGVAR(u16_Hold_Bits) & HOLD_USER_MASK)
    {
        return HOLD_MODE;
    }

    //////////////////////////////////////////////////////////////////
    //  Dealing with the HW and the SW limits
    //////////////////////////////////////////////////////////////////

    SW_ls = (VAR(AX0_u16_SW_Pos_lim) & 0x01) && ((u16_temp_ccw_ls & 0x1) || (u16_temp_cw_ls & 0x1));
    HW_ls = VAR(AX0_u16_CW_CCW_Inputs) && ((u16_temp_ccw_ls & 0xE) || (u16_temp_cw_ls & 0xE));

    //  FIX Bug#4364 - if LS is active and position is ABS or position is INC and !=0 check it's direction.
    if ((SW_ls || HW_ls) &&
        ((s64_target_position != 0x0LL)||(BGVAR(s16_Move_Abs_Issued))))
    {
        if (u16_temp_cw_ls != 0)
        {
            Pos_Ls_Subs = 1;

            if ((u16_temp_cw_ls & 0x1) != 0)
            {
                do {
                    u16_temp_time = Cntr_3125;
                    s64_temp = LLVAR(AX0_u32_Pos_Max_Lim_Lo);
                } while (u16_temp_time != Cntr_3125);

                if (BGVAR(s16_Move_Abs_Issued) == 0)
                {
                    Pos_Ls_Subs = s64_target_position >> 63 & 0x1;
                }
                else
                {
                    Pos_Ls_Subs = (s64_target_position - s64_temp) >> 63 & 0x1;
                }
            }

            if ((u16_temp_cw_ls & 0xE) != 0)
            {
                do {
                    u16_temp_time = Cntr_3125;
                    s64_temp = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
                } while (u16_temp_time != Cntr_3125);

                if (BGVAR(s16_Move_Abs_Issued) == 0)
                {
                    Pos_Ls_Subs &= s64_target_position >> 63 & 0x1;
                }
                else
                {
                    Pos_Ls_Subs &= (s64_target_position - s64_temp) >> 63 & 0x1;
                }
            }

            *s16_paramInvalid = 2;

            if (Pos_Ls_Subs == 0)
            {
                return(COMMAND_TOWARDS_LIMIT);
            }
        }

        //  clear can alarm
        BGVAR(u64_Sys_Warnings) &= ~WRN_COMMAND_TOWARDS_CW_SW_LIMIT_MASK;

        if (u16_temp_ccw_ls != 0)
        {
            Pos_Ls_Subs = 0;

            if ((u16_temp_ccw_ls & 0x1) != 0)
            {
                do {
                    u16_temp_time = Cntr_3125;
                    s64_temp = LLVAR(AX0_u32_Pos_Min_Lim_Lo);
                } while (u16_temp_time != Cntr_3125);

                if (BGVAR(s16_Move_Abs_Issued) == 0)
                {
                    Pos_Ls_Subs = s64_target_position >> 63 & 0x1;
                }
                else
                {
                    Pos_Ls_Subs = (s64_target_position - s64_temp) >> 63 & 0x1;
                }
            }

            if ((u16_temp_ccw_ls & 0xE) != 0)
            {
                do {
                    u16_temp_time = Cntr_3125;
                    s64_temp = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
                } while (u16_temp_time != Cntr_3125);

                if (BGVAR(s16_Move_Abs_Issued) == 0)
                {
                    Pos_Ls_Subs |= s64_target_position >> 63 & 0x1;
                }
                else
                {
                    Pos_Ls_Subs |= (s64_target_position - s64_temp) >> 63 & 0x1;
                }
            }

            *s16_paramInvalid = 2;

            if (Pos_Ls_Subs == 1)
            {
                return(COMMAND_TOWARDS_LIMIT);
            }
        }
    }

    //  clear can alarms
    BGVAR(u64_Sys_Warnings) &= ~WRN_COMMAND_TOWARDS_CW_SW_LIMIT_MASK;
    BGVAR(u64_Sys_Warnings) &= ~WRN_COMMAND_TOWARDS_CCW_SW_LIMIT_MASK;
    ////////////////////////////////////////////////

    if (BGVAR(s16_Move_Abs_Issued))
    {
        //  if we are using modulo, we have to figure out the shortest way from the starting position
        if (VAR(AX0_u16_Position_Modulo_Active) != 0)
        {
            res = MultS64ByFixS64ToS64(s64_target_position,
                                       BGVAR(Unit_Conversion_Table[POSITION_INTERNAL_COUNTS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                       BGVAR(Unit_Conversion_Table[POSITION_INTERNAL_COUNTS_CONVERSION]).u16_unit_conversion_to_user_shr);

            *s16_paramInvalid = 2;

            if ((res < BGVAR(s64_Modulo_Low)) || (res > (BGVAR(s64_Modulo_Low) + BGVAR(s64_Modulo_Range))))
            {
                return VALUE_OUT_OF_RANGE;
            }

            // Validate this is the first move after abort or hold, take the current position command as the starting point.
            if ((BGVAR(u16_Ptp_Abort_Flags) & 0x0001) || (VAR(AX0_u16_Ptp_Hold_Flags) & 0x0001))
            {
                do {
                   u16_temp_time = Cntr_3125;
                   s64_pcmd = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
                } while (u16_temp_time != Cntr_3125);
            }
            // Take the current target position as the starting point.
            else
            {
                do {
                   u16_temp_time = Cntr_3125;
                   s64_pcmd = LLVAR(AX0_u32_Target_Position_Lo);
                } while (u16_temp_time != Cntr_3125);
            }

            s64_distance = s64_target_position - s64_pcmd;
            s64_distance = MultS64ByFixS64ToS64(s64_distance,
                                                BGVAR(Unit_Conversion_Table[POSITION_INTERNAL_COUNTS_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                BGVAR(Unit_Conversion_Table[POSITION_INTERNAL_COUNTS_CONVERSION]).u16_unit_conversion_to_user_shr);
            s64_distance = s64_distance % BGVAR(s64_Modulo_Range);

            // need to think: what if very large motion?
            //  MOVEABS by shortest path
            if (1 == u16_Modulo_Mode)
            {
                if (s64_distance < ((-BGVAR(s64_Modulo_Range)) >> 1))
                {
                    s64_distance = s64_distance + BGVAR(s64_Modulo_Range);
                }

                if (s64_distance > (BGVAR(s64_Modulo_Range) >> 1))
                {
                    s64_distance = s64_distance - BGVAR(s64_Modulo_Range);
                }
            }
            //  MOVEABS positive direction only
            else if (3 == u16_Modulo_Mode)
            {
                if (s64_distance < 0LL)
                {
                    s64_distance = s64_distance + BGVAR(s64_Modulo_Range);
                }
            }
            //  MOVEABS negative direction only
            else if (5 == u16_Modulo_Mode)
            {
                if (s64_distance > 0LL)
                {
                    s64_distance = s64_distance - BGVAR(s64_Modulo_Range);
                }
            }
        
            s64_distance = MultS64ByFixS64ToS64(s64_distance,
                                                BGVAR(Unit_Conversion_Table[POSITION_INTERNAL_COUNTS_CONVERSION]).s64_unit_conversion_to_internal_fix,
                                                BGVAR(Unit_Conversion_Table[POSITION_INTERNAL_COUNTS_CONVERSION]).u16_unit_conversion_to_internal_shr);

            BGVAR(s64_Move_Abs_Position) = s64_target_position;
            s64_target_position = s64_pcmd + s64_distance;
        }
        else
        {
            BGVAR(s64_Move_Abs_Position) = s64_target_position;
        }

        BGVAR(s32_Move_Abs_Speed) = s32_temp_vel;
        BGVAR(s16_Move_Abs_Mode) = s16_transition_mode;
    }
    else
    {
        BGVAR(s64_Move_Inc_Position) = s64_target_position;
        BGVAR(s32_Move_Inc_Speed) = s32_temp_vel;
        BGVAR(s16_Move_Inc_Mode) = s16_transition_mode;

        //  convert the incremental position to absolute.
        if ((BGVAR(u16_Ptp_Abort_Flags) & 0x0001) || (VAR(AX0_u16_Ptp_Hold_Flags) & 0x0001))
        {  // If this is the first move after abort or hold, take the current position command as the starting point.
            do {
                u16_temp_time = Cntr_3125;
                s64_pcmd = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
            } while (u16_temp_time != Cntr_3125);
        }
        else
        {
            // else, take the current target position as the starting point.

            // handling profile position mode according to SDO x60F2
            if (p402_controlword & PPM_ABS_REL)
            {
                switch (p402_position_option_code & PPM_RELATIVE_OPTION)
                {
                    case 0x00:
                    {
                        //  Take the current target position as the starting point.
                        do {
                            u16_temp_time = Cntr_3125;
                            s64_pcmd = LLVAR(AX0_u32_Target_Position_Lo);
                        } while (u16_temp_time != Cntr_3125);
                        break;
                    }
                    case 0x01:
                    {
                        //  Take the current position command as the starting point.
                        do {
                            u16_temp_time = Cntr_3125;
                            s64_pcmd = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
                        } while (u16_temp_time != Cntr_3125);
                        break;
                    }
                    case 0x02:
                    {
                        //  Take the actual position as the starting point.
                        //                XXXX if dual loop take PEXT
                        do {
                            u16_temp_time = Cntr_3125;
                            s64_pcmd = LLVAR(AX0_u32_Pos_Fdbk_Dual_Lo);
                        } while (u16_temp_time != Cntr_3125);
                        break;
                    }
                }
            }
            else
            {  // Take the current target position as the starting point.
                do {
                    u16_temp_time = Cntr_3125;
                    s64_pcmd = LLVAR(AX0_u32_Target_Position_Lo);
                } while (u16_temp_time != Cntr_3125);
            }
        }

        s64_target_position += s64_pcmd;
    }

    BGVAR(u16_Ptp_Abort_Flags) = 0;
    VAR(AX0_u16_Ptp_Hold_Flags) = 0;

    BGVAR(s64_Gen_Target_Position) = s64_target_position;
    BGVAR(s32_Ptp_Move_Trgt_Vel) = s32_temp_vel;

    //  the if condition moved to here to prevent situation when "s64_Temp_Pos_LS" is updated to "s64_target_position"
    //  but the command function return with error and the value of "s64_Temp_Pos_LS" already changed to the invalid value.
    if (BGVAR(s16_Sal_Move_Ind))
    {
        BGVAR(s64_Temp_Pos_LS) = s64_local_temp_pos_ls;
        BGVAR(s16_Sal_Move_Ind) = 0;
    }
   
    if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC)
    {
        s32_temp_vel <<= 1;
    }

    InitiatePtpMove(drive, s64_target_position, s32_temp_vel, BGVAR(s32_Ptp_Acc_Rounded), BGVAR(s32_Ptp_Dec_Rounded), s16_transition_mode);

    return SAL_SUCCESS;
}


int StartMoveCommand(int drive)
{
   long long s64_temp_vel, s64_temp_pos;
   int index;
   // AXIS_OFF;

   if (FILEDBUS_MODE)                  return (WRONG_COMMODE);
   //if (BGVAR(u16_Hold_User))           return (HOLD_MODE);
   if (BGVAR(u16_Hold_Bits) & HOLD_USER_MASK)           return (HOLD_MODE);

   if (s16_Number_Of_Parameters == 2) s64_Execution_Parameter[2] = PTP_MOTION_END_TRANSITION;
   BGVAR(s16_Sal_Move_Ind)=1;
   //Convert velocity to internal units

   index = UnitsConversionIndex(&s8_Units_Vel_Out_Loop, drive) + IS_DUAL_LOOP_ACTIVE;
   
   s64_temp_vel = MultS64ByFixS64ToS64(s64_Execution_Parameter[1],
                  BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                  BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);

   //Convert position to internal units
   index = UnitsConversionIndex(&s8_Units_Pos, drive) + IS_DUAL_LOOP_ACTIVE;

   s64_temp_pos = MultS64ByFixS64ToS64(s64_Execution_Parameter[0],
                  BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                  BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);

   return ExecuteMoveCommand(drive,s64_temp_pos ,s64_temp_vel, s64_Execution_Parameter[2]);
}


int SalMoveIncCommand(int drive)
{
   int ret;

   if (BGVAR(u16_Est_Motor_Params) != 0) return (MOTOR_PARAMS_EST_IN_PROCESS);
   BGVAR(s16_Move_Abs_Issued) = 0;
   ret = StartMoveCommand(drive);
   return ret;
}


int SalMoveAbsCommand(int drive)
{
   int ret;

   if (BGVAR(u16_Est_Motor_Params) != 0) return (MOTOR_PARAMS_EST_IN_PROCESS);
   BGVAR(s16_Move_Abs_Issued) = 1;
   ret = StartMoveCommand(drive);
   return ret;
}


// Read function for MOVEABS command
int MoveAbsCommand(int drive)
{
   long long s64_temp;
   // AXIS_OFF;   
   REFERENCE_TO_DRIVE;
  
   s64_temp = MultS64ByFixS64ToS64(BGVAR(s64_Move_Abs_Position),
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION + IS_DUAL_LOOP_ACTIVE]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION + IS_DUAL_LOOP_ACTIVE]).u16_unit_conversion_to_user_shr);
   PrintSignedLongLong(s64_temp);
   PrintChar(SPACE);
   PrintChar('[');
   if (IS_DUAL_LOOP_ACTIVE)
      PrintString((char *)s8_Units_Pos_Sfb,0);
   else
      PrintString((char *)s8_Units_Pos,0);
   PrintChar(']');
   PrintChar(SPACE);

   s64_temp = MultS64ByFixS64ToS64((long long)BGVAR(s32_Move_Abs_Speed),
                                   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION + IS_DUAL_LOOP_ACTIVE]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION + IS_DUAL_LOOP_ACTIVE]).u16_unit_conversion_to_user_shr);
   PrintSignedLongLong(s64_temp);
   PrintChar(SPACE);
   PrintChar('[');
   if (IS_DUAL_LOOP_ACTIVE)
      PrintString((char *)s8_Units_Vel_Out_Loop_Sfb,0);
   else
   PrintString((char *)s8_Units_Vel_Out_Loop,0);
   PrintChar(']');
   PrintChar(SPACE);

   PrintSignedInteger(BGVAR(s16_Move_Abs_Mode));
   PrintCrLf();

   return SAL_SUCCESS;
}


// Read function for MOVEINC command
int MoveIncCommand(int drive)
{
   long long s64_temp;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   
   s64_temp = MultS64ByFixS64ToS64(BGVAR(s64_Move_Inc_Position),
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION + IS_DUAL_LOOP_ACTIVE]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION + IS_DUAL_LOOP_ACTIVE]).u16_unit_conversion_to_user_shr);
   PrintSignedLongLong(s64_temp);
   PrintChar(SPACE);
   PrintChar('[');
   if (IS_DUAL_LOOP_ACTIVE)
      PrintString((char *)s8_Units_Pos_Sfb,0);
   else
      PrintString((char *)s8_Units_Pos,0);
   PrintChar(']');
   PrintChar(SPACE);
   s64_temp = MultS64ByFixS64ToS64((long long)BGVAR(s32_Move_Inc_Speed),
                                   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION + IS_DUAL_LOOP_ACTIVE]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION + IS_DUAL_LOOP_ACTIVE]).u16_unit_conversion_to_user_shr);
   PrintSignedLongLong(s64_temp);
   PrintChar(SPACE);
   PrintChar('[');
   if (IS_DUAL_LOOP_ACTIVE)
      PrintString((char *)s8_Units_Vel_Out_Loop_Sfb,0);
   else
      PrintString((char *)s8_Units_Vel_Out_Loop,0);
   PrintChar(']');
   PrintChar(SPACE);

   PrintSignedInteger(BGVAR(s16_Move_Inc_Mode));
   PrintCrLf();

   return SAL_SUCCESS;
}

int SalMoveSineCommand(int drive)
{
    int ret = SAL_SUCCESS;
    float f_tmp = 0.0;
    float f_samples_per_second = SAMPLES_PER_SECOND_FOR_TSP_250;
    float f_2_Pie_Freq;
    int index;
    // AXIS_OFF;

    if (FILEDBUS_MODE)
    {
        return WRONG_COMMODE;
    }
    if (BGVAR(u16_Hold_Bits) & HOLD_USER_MASK)
    {
        return HOLD_MODE;
    }
    if (LVAR(AX0_s32_Pos_Vcmd) != 0L)
    {
        return MOTOR_IN_MOTION;
    }
    if (VAR(AX0_s16_Opmode) != 8)
    {
        return INVALID_OPMODE;
    }

    //  Validate Amplitude, Frequency are too low
    if ((s64_Execution_Parameter[0] < 1LL) ||
        (s64_Execution_Parameter[1] < 1LL))
    {
        return VALUE_TOO_LOW;
    }
    //  Validate Amplitude, Frequency are too high
    if ((s64_Execution_Parameter[0] > 0xffffffff) ||
        (s64_Execution_Parameter[1] > 400LL))
    {
        return VALUE_TOO_HIGH;
    }
    //  Validate number of parameters
    if (s16_Number_Of_Parameters == 3)
    {
        //  Validate Repetitions are too low
        if (s64_Execution_Parameter[2] < 1LL)
        {
            return VALUE_TOO_LOW;
        }
        //  Validate Repetitions are too high
        if (s64_Execution_Parameter[2] > 0xffffffff)
        {
            return VALUE_TOO_HIGH;
        }

        LVAR(AX0_u32_Move_Sine_Repeats) = (long)s64_Execution_Parameter[2];
    }
    else
    {
        //  Command was sent without Repetitions - set endless sine move
        LVAR(AX0_u32_Move_Sine_Repeats) = 0xFFFFFFFF;
    }

    /************* We use formula: Position command = A(mplitude) * SIN[2 * PIE * F(requency) / Samples Per Second * t(ime)]  ***********/
    /*  We divide the calculations into BG & RT          */
    /*  BG - 2 * PIE * F(requency) / Samples Per Second  */
    /*  RT - Amplitude * sin(time * BG calculation)      */

   //Convert first data (Amplitude) to internal units for later use in RT
   index = UnitsConversionIndex(&s8_Units_Pos, drive) + IS_DUAL_LOOP_ACTIVE;

    BGVAR(s64_Move_Sine_Amp) = MultS64ByFixS64ToS64(s64_Execution_Parameter[0],
                                                    BGVAR(Unit_Conversion_Table[index]).s64_unit_conversion_to_internal_fix,
                                                    BGVAR(Unit_Conversion_Table[index]).u16_unit_conversion_to_internal_shr);

    //   Validate Sample Rate
    if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)
    {
        f_samples_per_second = SAMPLES_PER_SECOND_FOR_TSP_125;
    }

    //  Calculate 2.0 * PI * (F)requency
    f_2_Pie_Freq = 2.0 * 3.14159265 * (float)s64_Execution_Parameter[1];
    //  Calculate: 2 * PI * (F)requency / Samples Per Second
    BGVAR(f_Move_Sine_2_Pi_Freq) = (float)(f_2_Pie_Freq / f_samples_per_second);

    //   Validate Sample Rate
    if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)
    {
        //  When we are sampling twice as fast we need to multiply the frequency by 2
        BGVAR(f_Move_Sine_2_Pi_Freq) *= 2;
    }

    //  Velocity Limit - 2 * PI * F * A
    /*if ((float)(f_2_Pie_Freq * (float)BGVAR(s64_Move_Sine_Amp)) > (float)BGVAR(s32_V_Lim_Design))
    {
        return (CMD_EXCEEDS_SW_LIMITS);
    }*/

    //Velocity Limit - 2 * PI * F * A
    if ((float)(BGVAR(f_Move_Sine_2_Pi_Freq) / 2.0 * (float)BGVAR(s64_Move_Sine_Amp)) > (float)BGVAR(s32_V_Lim_Design))
    {
       return (CMD_EXCEEDS_SW_LIMITS);
    }

    //ACC/DEC limit - (2 * PI * F) ^ 2 * A --> to move to ACC/DEC internal units multiply by (4000 / F)^2
    //f_tmp = (float)(BGVAR(f_Move_Sine_2_Pi_Freq) * BGVAR(f_Move_Sine_2_Pi_Freq) / 4.0 * (float)BGVAR(s64_Move_Sine_Amp) * (4000.0 / (float)s64_Execution_Parameter[1]) * (4000.0 / (float)s64_Execution_Parameter[1]));

    //  ACC/DEC limit - (2 * PI * F) ^ 2 * A --> to move to ACC/DEC internal units multiply by (Samples Per Second / F)^2
    f_tmp = ((f_2_Pie_Freq) * (f_2_Pie_Freq)) * (float)BGVAR(s64_Move_Sine_Amp);

    if ((f_tmp > (float)BGVAR(u64_DecRate)) || (f_tmp > (float)BGVAR(u64_AccRate)))
    {
        return (CMD_EXCEEDS_SW_LIMITS);
    }

    BGVAR(u16_Move_Sine_State) = MOVE_SINE_IDLE_STATE;
    BGVAR(s32_Move_Sine_ctr) = 0L;

    //  Calculate time steps needed for "one" sine wave =  Samples Per Second * (1 / F)
    f_tmp = f_samples_per_second / (float)(s64_Execution_Parameter[1]);
    //  Rounding
    f_tmp += 0.5;

    LVAR(AX0_u32_Move_Sine_Steps) = (unsigned long)f_tmp;

    //  Save pointer in order to restore it when MOVESINE ends
    BGVAR(s16_Move_Sine_Saved_Vel_Ptr_Value) = VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr);

    //  Set pointer to set to SINEMOVE value
    POS_LOOP_VCMD_PTR_ASSIGN((int)((long)&LVAR(AX0_s32_Pos_Vcmd_Sine) & 0xffff));

    //  Copy the Variables of the Dual Feedback
    CopyDualFeedbackDesignVariables(drive);

    //  Activate the feature via setting "AX0_u32_Move_Sine_Steps_Counter" only after
    //  setting "AX0_s16_Pos_Loop_Vcmnd_Ptr".
    LVAR(AX0_u32_Move_Sine_Steps_Counter) = LVAR(AX0_u32_Move_Sine_Steps);

    //  For read
    BGVAR(s32_Move_Sine_Freq) = (long)s64_Execution_Parameter[1];
    BGVAR(s32_Move_Sine_Repeat) = (long)s64_Execution_Parameter[2];

    return ret;
}

// Read function for MOVESINE command
int MoveSineCommand(int drive)
{  
 
   long long s64_temp;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   
   s64_temp = MultS64ByFixS64ToS64(BGVAR(s64_Move_Sine_Amp),
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION + IS_DUAL_LOOP_ACTIVE]).s64_unit_conversion_to_user_fix,
                                   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION + IS_DUAL_LOOP_ACTIVE]).u16_unit_conversion_to_user_shr);
   PrintSignedLongLong(s64_temp);
   PrintChar(SPACE);
   PrintChar('[');
   if (IS_DUAL_LOOP_ACTIVE)
      PrintString((char *)s8_Units_Pos_Sfb,0);
   else
      PrintString((char *)s8_Units_Pos,0);
   PrintChar(']');
   PrintChar(SPACE);


   PrintSignedLong(BGVAR(s32_Move_Sine_Freq));
   PrintChar(SPACE);

   PrintSignedInteger(BGVAR(s32_Move_Sine_Repeat));
   PrintCrLf();

   return SAL_SUCCESS;
}


void MotionBufferHandler(int drive)
{
   // AXIS_OFF;
   unsigned int u16_temp_time;
   long long s64_pcmd;

   if (!Enabled(drive)) BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_IDLE;

   switch (BGVAR(u16_MotionBuffer_Handler_State))
   {
      case MB_HNDLR_IDLE:
         BGVAR(u16_Moveinc_Iter) = BGVAR(u16_Moveinc_Counter);
      break;

      case MB_HNDLR_COMMAND_MOVE1:
         if (BGVAR(u16_Moveinc_Iter) == BGVAR(u16_Moveinc_Counter))
         {
            BGVAR(s32_MotionBuffer_Overall_Time) = 0L;
            BGVAR(s32_MotionBuffer_Start_Time) = Cntr_1mS;
         }

         BGVAR(u16_Moveinc_Iter)--;
         BGVAR(s64_Move_Inc_Position) = BGVAR(s64_Moveinc_Dist1);

         // Make sure not to exceed VLIM
         if (BGVAR(s32_Moveinc_Speed1) > BGVAR(s32_V_Lim_Design))
            BGVAR(s32_Move_Inc_Speed) = BGVAR(s32_V_Lim_Design);
         else
            BGVAR(s32_Move_Inc_Speed) = BGVAR(s32_Moveinc_Speed1);

         BGVAR(s16_Move_Inc_Mode) = PTP_MOTION_END_TRANSITION;

         // convert the incremental position to absolute
         do {
            u16_temp_time = Cntr_3125;
            s64_pcmd = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
         } while (u16_temp_time != Cntr_3125);

         BGVAR(s16_Move_Abs_Issued) = 0;
         InitiatePtpMove(drive, BGVAR(s64_Move_Inc_Position) + s64_pcmd, (BGVAR(s32_Move_Inc_Speed) << 1), BGVAR(s32_Ptp_Acc_Rounded), BGVAR(s32_Ptp_Dec_Rounded), BGVAR(s16_Move_Inc_Mode));

         BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_WAIT_MOVE1_START;
         BGVAR(s32_MotionBuffer_Timer) = Cntr_1mS;
      break;

      case MB_HNDLR_WAIT_MOVE1_START:
         if ((VAR(AX0_s16_Stopped) != 2) || (PassedTimeMS(4L, BGVAR(s32_MotionBuffer_Timer)))) BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_WAIT_MOVE1_STOPPED;
      break;

      case MB_HNDLR_WAIT_MOVE1_STOPPED:
         if (VAR(AX0_s16_Stopped) == 2)
         {
            if (BGVAR(s32_Moveinc_Delay))
               BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_MOVE1_DELAY;
            else
               BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_COMMAND_MOVE2;
            BGVAR(s32_MotionBuffer_Timer) = Cntr_1mS;
         }
      break;

      case MB_HNDLR_MOVE1_DELAY:
         if (PassedTimeMS(BGVAR(s32_Moveinc_Delay), BGVAR(s32_MotionBuffer_Timer)))
            BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_COMMAND_MOVE2;
      break;

      case MB_HNDLR_COMMAND_MOVE2:
         BGVAR(s64_Move_Inc_Position) = BGVAR(s64_Moveinc_Dist2);

         // Make sure not to exceed VLIM
         if (BGVAR(s32_Moveinc_Speed2) > BGVAR(s32_V_Lim_Design))
            BGVAR(s32_Move_Inc_Speed) = BGVAR(s32_V_Lim_Design);
         else
            BGVAR(s32_Move_Inc_Speed) = BGVAR(s32_Moveinc_Speed2);

         BGVAR(s16_Move_Inc_Mode) = PTP_MOTION_END_TRANSITION;

         // convert the incremental position to absolute
         do {
            u16_temp_time = Cntr_3125;
            s64_pcmd = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
         } while (u16_temp_time != Cntr_3125);

         BGVAR(s16_Move_Abs_Issued) = 0;
         InitiatePtpMove(drive, BGVAR(s64_Move_Inc_Position) + s64_pcmd, (BGVAR(s32_Move_Inc_Speed) << 1), BGVAR(s32_Ptp_Acc_Rounded),  BGVAR(s32_Ptp_Dec_Rounded), BGVAR(s16_Move_Inc_Mode));

         BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_WAIT_MOVE2_START;
         BGVAR(s32_MotionBuffer_Timer) = Cntr_1mS;
      break;

      case MB_HNDLR_WAIT_MOVE2_START:
         if ((VAR(AX0_s16_Stopped) != 2) || (PassedTimeMS(4L, BGVAR(s32_MotionBuffer_Timer)))) BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_WAIT_MOVE2_STOPPED;
      break;

      case MB_HNDLR_WAIT_MOVE2_STOPPED:
         if (VAR(AX0_s16_Stopped) == 2)
         {
            if (BGVAR(u16_Moveinc_Iter) == 0)
            {
               BGVAR(s32_MotionBuffer_Overall_Time) = Cntr_1mS - BGVAR(s32_MotionBuffer_Start_Time);
               //if (BGVAR(s32_MotionBuffer_Overall_Time) < 0L) BGVAR(s32_MotionBuffer_Overall_Time) = (long long)0x00000000FFFFFFFF
               BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_IDLE;
            }
            else
            {
               if (BGVAR(s32_Moveinc_Delay))
                  BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_MOVE2_DELAY;
               else
                  BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_COMMAND_MOVE1;
               BGVAR(s32_MotionBuffer_Timer) = Cntr_1mS;
            }
         }
      break;

      case MB_HNDLR_MOVE2_DELAY:
         if (PassedTimeMS(BGVAR(s32_Moveinc_Delay), BGVAR(s32_MotionBuffer_Timer)))
            BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_COMMAND_MOVE1;
      break;
   }
}


int MotionBufferCommand(int drive)
{
   // AXIS_OFF;

   if (FILEDBUS_MODE)   return (WRONG_COMMODE);
   if (BGVAR(s16_SFBMode)) return (SFBMODE_USED); // !!!!!!!!!!!!!!!!!!!!!!!!!
   if (!Enabled(DRIVE_PARAM)) return (DRIVE_INACTIVE);
   if (VAR(AX0_s16_Opmode) != 8) return (INVALID_OPMODE);
   if (VAR(AX0_s16_Ptp_Transition_Mode_Bg) != PTP_IDLE_TRANSITION) return (MOTION_PENDING);

   if (BGVAR(u16_MotionBuffer_Handler_State) == MB_HNDLR_IDLE)
      BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_COMMAND_MOVE1;

   return (SAL_SUCCESS);
}


int SalMotionBufferStatusCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotionBuffer_Handler_State) == MB_HNDLR_IDLE)
   {
      PrintString("Done. Execution Time: ",0);
      PrintUnsignedLong(BGVAR(s32_MotionBuffer_Overall_Time));
      PrintStringCrLf("mS",0);
   }
   else
   {
      PrintString("Running. Iteration: ",0);
      PrintUnsignedInt16(BGVAR(u16_Moveinc_Counter) - BGVAR(u16_Moveinc_Iter));
      PrintChar('/');
      PrintUnsignedInt16(BGVAR(u16_Moveinc_Counter));
      PrintString(" Execution Time: ",0);
      PrintUnsignedLong(Cntr_1mS - BGVAR(s32_MotionBuffer_Start_Time));
      PrintStringCrLf("mS", 0);
   }
   return SAL_SUCCESS;
}


unsigned long long CalculateNewDecStop(int drive)
{
   // AXIS_OFF;

   float f_decstop_value = 0.0;
   unsigned long long u64_decstop_value = 0LL;

   f_decstop_value = (float)LVAR(AX0_s32_Vel_Var_Fb_0);
   if (f_decstop_value < 0.0) f_decstop_value = -f_decstop_value;
   f_decstop_value *= CalculateDecStopTime1OverT(drive);
   u64_decstop_value = (unsigned long long)(f_decstop_value);

   if (u64_decstop_value > BGVAR(u64_DecStop)) u64_decstop_value = BGVAR(u64_DecStop);
   if (u64_decstop_value < 0x100000000) u64_decstop_value = 0x100000000;

   return u64_decstop_value;
}

/////////////////////////////////////////////////////////////////
// ActivateHold
/////////////////////////////////////////////////////////////////
// This function handles which deceleration
// the drive will use ,according to u16_Deceleration_Event_ID.
// The programmer has to load u16_Deceleration_Event_ID with the desired value.
//
// Afterwards, the internal deceleration variables are loaded at UpdateInternalDecValue function.
// UpdateInternalDecValue is activated by three different configurations:
// *UPDATE_LOAD_DEC ,*UPDATE_LOAD_DECSTOP ,*UPDATE_LOAD_DECSTOPTIME
//
// ActivateHold ,according to u16_Deceleration_Event_ID is assigned with one of the tree mentioned above configuration.
// Therefore,
// 1) u16_Deceleration_Event_ID == EVENT_DEC -> UpdateInternalDecValue(UPDATE_LOAD_DEC)
//    deceleration will be according to u16_Decrate
// 2) u16_Deceleration_Event_ID != EVENT_DEC -> UpdateInternalDecValue(UPDATE_LOAD_DECSTOP)
//    deceleration will be according to u16_Deceleration_Event_ID (default->decstop)
// 3) u16_DecStopTime != 0 -> UpdateInternalDecValue(UPDATE_LOAD_DECSTOPTIME)
//    deceleration will be according to calculated time
//
/////////////////////////////////////////////////////////////////
void ActivateHold(int drive)
{
   // AXIS_OFF;
   unsigned int u16_dec_transition = UPDATE_LOAD_DEC;
   // Zero command to avoid moving when HOLD is off
   switch (VAR(AX0_s16_Opmode))
   {
      case 0:
      case 1:
         if (BGVAR(u16_Deceleration_Event_ID) != EVENT_DEC)
         {
            u16_dec_transition = UPDATE_LOAD_DECSTOP;
         }

         // If CDHD drive and decstoptime>0 choose if to use DECSTOP or DECSTOPTIME.
         // FIX BUG#4297 if LXM drive set "UPDATE_LOAD_DECSTOPTIME" only if no event deceleration is required.
         if (BGVAR(u16_DecStopTime) &&
            (((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) ||
            ((BGVAR(u16_Deceleration_Event_ID) == EVENT_DECSTOP) && (u16_dec_transition == UPDATE_LOAD_DECSTOP))))
            {// if DECSTOP and DECSTOPTIME > 0 use UPDATE_LOAD_DECSTOPTIME
             // to calculate decstoptime and decide what deceleration to use.
               u16_dec_transition = UPDATE_LOAD_DECSTOPTIME;
            }

         UpdateInternalDecValue(drive, u16_dec_transition);

         BGVAR(StepHandlerState) = STEP_IDLE;

         VAR(AX0_u16_HP_Flags) |= HOLD_IN_PROCESS_MASK; // Indicate that a HOLD is in process in order to clear the command value

         JogCommand(0LL, drive); // Clear the jog in order to not do motion after terminating the HOLD
      break;

      case 2:
      case 3:
         VAR(AX0_u16_HP_Flags) |= HOLD_IN_PROCESS_MASK; // Indicate that a HOLD is in process in order to clear the command value

         InternalTorqueCommand(0LL, drive);  // Clear the serial torque command in order to not do motion after terminating the HOLD
      break;

      case 4:
      case 8:
         BGVAR(u16_MotionBuffer_Handler_State) = MB_HNDLR_IDLE;

         if (BGVAR(u16_Deceleration_Event_ID) != EVENT_DEC)
            u16_dec_transition = UPDATE_LOAD_DECSTOP;

         UpdateInternalDecValue(drive, u16_dec_transition);

         // Choose if to use DECSTOP or DECSTOPTIME
         if (BGVAR(u16_DecStopTime)) VAR(AX0_u16_HP_Flags) |= CALCULATE_DECSTOP_MASK;
         else VAR(AX0_u16_HP_Flags) &= ~CALCULATE_DECSTOP_MASK;

         
         // BZ 5576: The deceleration of HW Limits does not updated during deceleration in JOG mode
         // if hold is in progress, terminate it and wait for termination to finish (wait for bit to reset, not for zero speed).
         // Then, start new hold process.
         // this can happen if need to decelerate on a higher dec (e.g. when hold is active and another hold with higher dec happens).
         if (VAR(AX0_u16_HP_Flags) & HOLD_IN_PROCESS_MASK)
         {
            VAR(AX0_u16_HP_Flags) &= ~ACTIVATE_HOLD_MASK;
            VAR(AX0_u16_HP_Flags) |= DEACTIVATE_HOLD_MASK;
         
            // happens every 8 MTS or 4MTS (depends on AX0_u16_Pos_Loop_Tsp)
            while (VAR(AX0_u16_HP_Flags) & HOLD_IN_PROCESS_MASK)
            {}         
         }
         
         VAR(AX0_u16_HP_Flags) |= ACTIVATE_HOLD_MASK;
         VAR(AX0_u16_HP_Flags) &= ~DEACTIVATE_HOLD_MASK;
      break;
   }
}


void TerminateHold(int drive)
{
   // AXIS_OFF;
   UpdateInternalDecValue(drive, UPDATE_LOAD_DEC);

   switch (VAR(AX0_s16_Opmode))
   {
      case 0:
      case 1:
      case 2:
      case 3:
         VAR(AX0_u16_HP_Flags) &= ~HOLD_IN_PROCESS_MASK;
      break;

      // Specifically for OPMODE position initiate a terminate sequence
      case 4:
      case 8:
         VAR(AX0_u16_HP_Flags) &= ~ACTIVATE_HOLD_MASK;
         VAR(AX0_u16_HP_Flags) |= DEACTIVATE_HOLD_MASK;
      break;
   }
}
unsigned long long GetDecByHoldEvent(int drive, unsigned int u16_hold_event)
{
   unsigned long long u64_dec_val;
   int retval;

   drive +=0;

   switch(u16_hold_event)
   {
      case EVENT_COMM_TIMED_OUT_DEC:// Communication Timed Out
         u64_dec_val = BGVAR(u64_Ser_Comm_Time_Out_Dec_Rate);
      break;

      case EVENT_MOTOR_STOP_DEC:
         u64_dec_val = BGVAR(u64_Motor_Stop_Dec_Rate);
      break;

      case EVENT_SOFTWARE_NEGATIVE_LS_DEC:// Software negative limit switch CDHD and Schneider rotation direction are reversed
         u64_dec_val = BGVAR(u64_N_SW_Limit_Dec_Rate);
      break;

      case EVENT_POS_COMMAND_OVRFLW_DEC:
      break;

      case EVENT_SOFTWARE_POSITIVE_LS_DEC:// Software positive limit switch CDHD and Schneider rotation direction are reversed
         u64_dec_val = BGVAR(u64_P_SW_Limit_Dec_Rate);
      break;

      case EVENT_HARDWARE_NEGATIVE_LS_DEC:// Hardware negative limit switch CDHD and Schneider rotation direction are reversed
         u64_dec_val = BGVAR(u64_N_HW_Limit_Dec_Rate);
      break;

      case EVENT_HARDWARE_POSITIVE_LS_DEC:// Hardware positive limit switch CDHD and Schneider rotation direction are reversed
         u64_dec_val = BGVAR(u64_P_HW_Limit_Dec_Rate);
      break;

      case EVENT_DEC:// Hardware positive limit switch
         u64_dec_val = BGVAR(u64_DecRate);
      break;

      default://EVENT_DECSTOP or unknow event
         //restore the original dec stop rate to prevent invalid DecStop
         u64_dec_val = BGVAR(u64_DecStop);
      break;
   }

   retval = CheckAccDecLimits(u64_dec_val);

   if (retval != SAL_SUCCESS)
      u64_dec_val = BGVAR(u64_DecStop);

   return u64_dec_val;
}

//***********************************************************************************************************
// Function Name: UpdateInternalDecValue
//
//  Description: The function loads the internal values used for dec and decstop when events occur.
//  The user variables u64_DecRate,u64_DecStop aren't effected by this event handling.
//  (Events<=> user hold,limit switch hold event,motorstop.)
//
//  According to function input argument->u16_code ,the internal variables for dec and decstop are caclulated.
//  0: DEC
//  1: DECSTOPSTIME
//  2: DECSTOP.
//     At DECSTOP the varialbe u16_Deceleration_Event_ID determines the value for "DECSTOP"
//  After the event was handled,and the motion is stopped,the user can reassign u64_DecRate,u64_DecStop
//  into the internal variables,by setting u16_code to DEC
//
// Author: Gil
// Algorithm:
// Revisions:
//***********************************************************************************************************
void UpdateInternalDecValue(int drive, unsigned int u16_code)
{
   // AXIS_OFF;

   switch (u16_code)
   {
      case UPDATE_LOAD_DEC: //ordinary DEC
         BGVAR(u64_Event_Dec_Val) = BGVAR(u64_DecRate);
       break;

      case UPDATE_LOAD_DECSTOPTIME: // DECSTOPSTIME
         BGVAR(u64_Event_Dec_Val) = CalculateNewDecStop(drive);
      break;

      case UPDATE_LOAD_DECSTOP: // DECSTOP
      {
         BGVAR(u64_Event_Dec_Val) = GetDecByHoldEvent(drive, BGVAR(u16_Deceleration_Event_ID));
      }
   }

   //checking
   if (IS_CAN_DRIVE_AND_COMMODE_1 || IS_EC_DRIVE_AND_COMMODE_1)
   {
      // in DS402 there is limit function on acc/dec
     if (BGVAR(u64_Event_Dec_Val) > BGVAR(u64_CAN_Max_Dec))
     {
        BGVAR(u64_Event_Dec_Val) = BGVAR(u64_CAN_Max_Dec);
     }
   }

   // In case decstop was updated not by the user,but per scenario
   LVAR(AX0_s32_Ptp_Dec_Stop_Rounded) = (long)(( BGVAR(u64_Event_Dec_Val) + (unsigned long long)0x080000000) >> 32);
   UpdateAccDec(drive, HOLD_UPDATE);
}

void MotorStop(int drive)
{
   BGVAR(u16_Deceleration_Event_ID) = EVENT_MOTOR_STOP_DEC;
   // use the u64_Motor_Stop_Dec_Rate (P2-08)
   UpdateInternalDecValue(drive,UPDATE_LOAD_DECSTOP );
   StopCommand(drive);
   // restore original dec rate
   UpdateInternalDecValue(drive,UPDATE_LOAD_DEC );
}

int SalHoldCommand(long long param, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   BGVAR(u16_Deceleration_Event_ID) = EVENT_DECSTOP;
   // nitsan: halt bit in control word (bit 8) set/reset u16_Hold_User in BG.
   // so set the bit to prevent it override this setting
   if ((unsigned int)param)
   {
//       p402_controlword |= 0x100;
       BGVAR(u16_Hold_Bits) |= HOLD_USER_MASK;
   }
   else
   {
      // Bug-fix for Bugzilla item 4598
      // Only for gearing mode without compensation move. If OPMODE 4 and if the Drive is
      // requested to discard pulses upon disable or when releasing user HOLD.
      if((VAR(AX0_s16_Opmode) == 4) && ((VAR(AX0_u8_Gear_Limits_Mode) & 0x0001) == 0x0000))
      {
         // Instruct the gearing function to clear the position deviation.
         VAR(AX0_u16_Gear_BG_Flags) |= 0x0004; // Set bit 2, the flag is cleared automatically in the gearing code.
      }

//       p402_controlword &= ~0x100;
       BGVAR(u16_Hold_Bits) &= ~HOLD_USER_MASK;
   }

   //BGVAR(u16_Hold_User) = (unsigned int)param;
   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: IsHoldActive
// Description: This function returns a value that indicates the state of the
//              hold based on the variable "AX0_u16_HP_Flags".
//
//              Return value:
//                0 = No HOLD is pending
//                1 = HOLD is activated
//                2 = HOLD is about to be activated in OPMODE 4 & 8,
//                    which takes a few MTS cycles.
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(IsHoldActive, "ramfunc_3")
int IsHoldActive(int drive)
{
   // AXIS_OFF;

   int s16_return_value = 0;
   int s16_temp_val = VAR(AX0_u16_HP_Flags) & (ACTIVATE_HOLD_MASK | HOLD_IN_PROCESS_MASK | FIRST_POS_AFTER_HOLD_MASK);
   REFERENCE_TO_DRIVE;

   if(s16_temp_val == HOLD_IN_PROCESS_MASK) // If a hold is being activate and the HOLD initiation is finished
   {
      s16_return_value = 1;
   }
   else if(s16_temp_val != 0) // If a hold is supposed to be activated
   {
      s16_return_value = 2;
   }

   return (s16_return_value);
}


//**********************************************************
// Function Name: UseVelControlledRampDownInOpmodeTorque
//
// Description: This function returns a true in case that the feature "velocity
//              controlled ramp-down in OPMODE torque" is supposed to be used
//              upon a certain HOLD.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
/*int UseVelControlledRampDownInOpmodeTorque(int drive)
{
   int s16_return_val = 0;

   // AXIS_OFF;

   if ((BGVAR(u16_Lexium_PX_YZ) & 0x0001) && ((VAR(AX0_s16_Opmode) == 2) || (VAR(AX0_s16_Opmode) == 3)))
   {
      s16_return_val = 1;
   }

   return (s16_return_val);
}*/

//**********************************************************
// Function Name: IsVelocityControlledRampDownTrqFeatureActive
//
// Description: This function returns a true in case that the feature "velocity
//              controlled ramp-down in OPMODE torque" is currently running.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int IsVelControlledRampDownInOpmodeTrqActive(int drive)
{
   int s16_vel_controlled_ramp_down_active = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // If the state machine inside HoldScan reached a state that is used for the
   // feature "velocity controlled ramp-down in OPMODE torque"
   if (BGVAR(u16_hold_scan_FSM) >= HOLD_SCAN_SWITCH_TO_VEL_MODE_ON_THE_FLY)
   {
      s16_vel_controlled_ramp_down_active = 1;
   }

   return (s16_vel_controlled_ramp_down_active);
}

//**********************************************************
// Function Name: HoldScan
//
// Description:
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void HoldScan(int drive)
{
   static unsigned int u16_hold_bits_prev = 0;
   unsigned long long u64_new_dec;
   // AXIS_OFF;

   /////////////////////////////////////////////////////////////////////////////////////////////
   // Hold is composed of bit-wise OR of user hold command, limit-switch hold command and so on.
   /////////////////////////////////////////////////////////////////////////////////////////////
   BGVAR(u16_Hold) = BGVAR(u16_Hold_Bits) ? 1 : 0;

   /////////////////////////////////////////////////////////////////////////////////
   // Evaluate a potential change in the hold only in case that the state machine
   // is in idle state. Otherwise a HOLD activation/deactivation process is pending
   // and is not supposed to be interrupted.
   /////////////////////////////////////////////////////////////////////////////////
   if(BGVAR(u16_hold_scan_FSM) == HOLD_SCAN_IDLE)
   {
      // If a HOLD is requested and neither a HOLD is supposed to be activated nor pending.
      if ((BGVAR(u16_Hold) > 0)) // && (IsHoldActive(drive) == 0))
      {
         if (IsHoldActive(drive) == 0)
         {
            BGVAR(u16_hold_scan_FSM) = HOLD_SCAN_INIT_HOLD_ACTIVATION; // Initiate state machine to activate a hold
         }
         else
         {
            // here deceleration is in progress
            // check if hold_bits have changed (i.e. if a new hold event occurred)
            if (BGVAR(u16_Hold_Bits) != u16_hold_bits_prev)
            {
               // need to check if deceleration should be modified. take the fastest deceleration of all hold events
               // currently just check if the latest deceleration event requires a faster dec than the one that is in progress.
               u64_new_dec = GetDecByHoldEvent(drive, BGVAR(u16_Deceleration_Event_ID));
               if (u64_new_dec > BGVAR(u64_Event_Dec_Val))
               {
                  BGVAR(u16_hold_scan_FSM) = HOLD_SCAN_INIT_HOLD_ACTIVATION; // Initiate state machine to activate a hold
               }
            }
         }
      }
      // If no HOLD is anymore requested and a HOLD is pending and there is no termination request of a HOLD
      else if( (BGVAR(u16_Hold) == 0) && (IsHoldActive(drive)) &&
               ((VAR(AX0_u16_HP_Flags) & DEACTIVATE_HOLD_MASK) == 0) )
      {
         BGVAR(u16_hold_scan_FSM) = HOLD_SCAN_INIT_HOLD_TERMINATION; // Initiate state machine to terminate a hold
      }
   }

   ///////////////////////////////////////////////////////////////////////////////////
   // Hold_Scan state machine
   ///////////////////////////////////////////////////////////////////////////////////
   switch (BGVAR(u16_hold_scan_FSM))
   {
      case HOLD_SCAN_IDLE:
         // Do nothing

         // Temporary code, variable "BGVAR(u16_Hold_Stop_In_Process)" is supposed to be removed
         if((VAR(AX0_u16_Motor_Stopped) == 2) || IsHoldActive(drive))
         {
            BGVAR(u16_Hold_Stop_In_Process) = 0;
         }
      break;

      case HOLD_SCAN_INIT_HOLD_TERMINATION:
         TerminateHold(drive);
         BGVAR(u16_hold_scan_FSM) = HOLD_SCAN_IDLE;
      break;

      case HOLD_SCAN_INIT_HOLD_ACTIVATION:
         // If the feature "controlled ramp-down in OPMODE torque" is activated and
         // if the current CDHD OPMODE is adjusted to torque mode
         /*if(UseVelControlledRampDownInOpmodeTorque(drive))
         {
            // If CAN or ECAT adjusted a torque OPMODE. This is condition is important for restoring the previous situation
            if( ((IS_CAN_DRIVE_AND_COMMODE_1) || (IS_EC_DRIVE_AND_COMMODE_1)) &&
                ((p402_modes_of_operation_display == ANALOG_TORQUE_MODE) || (p402_modes_of_operation_display == PROFILE_TORQUE_MODE) || (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_TORQUE_MODE))
              )
            {
               // Save DS402 troque mode
               BGVAR(s16_Previous_Torque_Mode) = p402_modes_of_operation_display;
               // Indicate that "BGVAR(s16_Previous_Torque_Mode)" holds a DS402 OPMODE value
               BGVAR(u16_Previous_Torque_Mode_Owner) = 1;
            }
            else // regular CDHD mode (no fieldbus)
            {
               // Save CDHD torque mode
               BGVAR(s16_Previous_Torque_Mode) = VAR(AX0_s16_Opmode);
               // Indicate that "BGVAR(s16_Previous_Torque_Mode)" holds a CDHD OPMODE value
               BGVAR(u16_Previous_Torque_Mode_Owner) = 0;
            }

            // Ensure that another OPMODE change request does not interfere the
            // ongoing HOLD actions
            BGVAR(u16_Block_Opmode_Change) |= BLOCK_OPMODE_CHANGE_BY_HOLD;

            BGVAR(u16_hold_scan_FSM) = HOLD_SCAN_SWITCH_TO_VEL_MODE_ON_THE_FLY;
         }
         else*/
         {
            ActivateHold(drive);
            BGVAR(u16_Hold_Stop_In_Process) = 1;
            BGVAR(u16_hold_scan_FSM) = HOLD_SCAN_IDLE;
         }
      break;

      case HOLD_SCAN_SWITCH_TO_VEL_MODE_ON_THE_FLY:

         // Allow OPMODE change on the fly to velocity (smmoth transition)
         VAR(AX0_s16_Opmode_Change_Flags) |= OPMODE_CHANGE_ON_THE_FLY_MASK;

         if(BGVAR(u16_Previous_Torque_Mode_Owner) == 1) // If a fieldbus drives the CDHD in torque mode
         {
            BGVAR(s16_CAN_Opmode_Temp) = PROFILE_VELOCITY_MODE;
            BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
            SetCanOpmode(drive);
         }
         else // Regular CDHD OPMODE torque
         {
            SetOpmode(drive, 0);
         }

         // Activate the HOLD straight away
         ActivateHold(drive);
         BGVAR(u16_Hold_Stop_In_Process) = 1;

         BGVAR(u16_hold_scan_FSM) = HOLD_SCAN_SWITCH_TO_PREVIOUS_MODE;
      break;

      case HOLD_SCAN_SWITCH_TO_PREVIOUS_MODE:
         if(VAR(AX0_u16_Motor_Stopped) == 2) // If a standstill condition has been identified
         {
            if(BGVAR(u16_Previous_Torque_Mode_Owner) == 1) // If the fieldbus has previously adjusted the OPMODE
            {
               BGVAR(s16_CAN_Opmode_Temp) = BGVAR(s16_Previous_Torque_Mode);
               BGVAR(s16_CAN_Flags) |= CANOPEN_OPMODE_CHANGED_MASK;
               SetCanOpmode(drive);
            }
            else // a regular CDHD OPMODE (0...8) was adjusted (no fieldbus)
            {
               SetOpmode(drive, BGVAR(s16_Previous_Torque_Mode));
               BGVAR(u16_hold_scan_FSM) = HOLD_SCAN_IDLE;
            }

            BGVAR(u16_Block_Opmode_Change) &= ~BLOCK_OPMODE_CHANGE_BY_HOLD;

            BGVAR(u16_hold_scan_FSM) = HOLD_SCAN_IDLE;
         }
      break;

      default: // Unknown state, not supposed to happen
         BGVAR(u16_hold_scan_FSM) = HOLD_SCAN_IDLE;
      break;
   }
   u16_hold_bits_prev = BGVAR(u16_Hold_Bits);
}

void PtpAbort(int drive)
{
   BGVAR(u16_Ptp_Abort_Flags) |= 1;    // indicate abort command
   BGVAR(s16_Move_Abs_Issued) = 0;
   InitiatePtpMove(drive, 0LL /*pos*/, 0L /*vel*/, BGVAR(s32_Ptp_Acc_Rounded), BGVAR(s32_Ptp_Dec_Rounded), (int)PTP_IMMEDIATE_TRANSITION);
}


void MoveSmoothFilterDesign(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   FloatToFixS32Shift16(&LVAR(AX0_s32_Move_Smooth_Fix_Design), &VAR(AX0_u16_Move_Smooth_Shr_Design), 1.0 / (float)LVAR(AX0_u32_Move_Smooth_Avg_Num_Design));

   // Signal rt to copy design to operational
   if (u16_background_ran)
      VAR(AX0_u16_Move_Smooth_Aux_Flags) |= COPY_PRE_POS_VAR_MASK;
   else
   {
      LVAR(AX0_u32_Move_Smooth_Avg_Num) = LVAR(AX0_u32_Move_Smooth_Avg_Num_Design);
      LVAR(AX0_s32_Move_Smooth_Fix) = LVAR(AX0_s32_Move_Smooth_Fix_Design);
      VAR(AX0_u16_Move_Smooth_Shr) = VAR(AX0_u16_Move_Smooth_Shr_Design);
   }
}

//**********************************************************
// Function Name: CheckPtpHunterMode
// Description: This function checks in which mode the PTP hunter is supposed to
//              work. The PTP hunter can operate in two different modes.
//
//              Mode 0: The PTP-hunter always tries to decelerate into the target
//                      position with the selected deceleration ramp. Therefore
//                      The PTP-hunter always keeps a distance to a moving target
//                      position (like in gearing or CSP mode).
//              Mode 1: The PTP-hunter is allowed to run synchronously with a
//                      moving target position. In this case the PTP-hunter would
//                      overshoot and move backwards into the target position in
//                      case that the moving target position would come to an
//                      immediate stop (like in gearing or CSP mode).
//              The PTP-hunter gets the indication in which mode to operate via
//              a RT variable, which is set inside of this function.
//
//              u16_immediate_update:
//                   0 - The variable "AX0_u16_PTP_Hunter_Mode" is updated via
//                       "AX0_u16_PTP_Hunter_Mode_Bg_Shadow" by code that is supposed
//                       to run after this function call, such as an OPMODE change.
//                   1 - The variable AX0_u16_PTP_Hunter_Mode" is updated within this function.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void CheckPtpHunterMode(int drive, unsigned int u16_immediate_update)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if((VAR(AX0_s16_Opmode) == 4) && (VAR(AX0_u8_Gear_Limits_Mode) & 0x0010))
   {
      VAR(AX0_u16_PTP_Hunter_Mode_Bg_Shadow) = 1;
   }
   else if ((VAR(AX0_s16_Opmode) == 8) && (BGVAR(u8_Comm_Mode) == 1) &&
            // Use "p402_modes_of_operation" and NOT "p402_modes_of_operation_display" because this variable is set later after this function is called
            ((p402_modes_of_operation == CYCLIC_SYNCHRONOUS_POSITION_MODE) || (p402_modes_of_operation == INTRPOLATED_POSITION_MODE)) &&
            (VAR(AX0_u16_CSP_General_Purpose_Rt_Flags) & 0x0002))
   {
      VAR(AX0_u16_PTP_Hunter_Mode_Bg_Shadow) = 1;
   }
   else
   {
      VAR(AX0_u16_PTP_Hunter_Mode_Bg_Shadow) = 0;
   }

   // If an update of "AX0_u16_PTP_Hunter_Mode" is required in this function.
   if(u16_immediate_update != 0)
   {
      VAR(AX0_u16_PTP_Hunter_Mode) = VAR(AX0_u16_PTP_Hunter_Mode_Bg_Shadow);
   }
}

void PtpFilterDesign(int drive)
{
   // AXIS_OFF;

   long s32_alpha,s32_beta;

// LPF design
   if (BGVAR(s16_Move_Smooth_Lpf_Hz) >= 5000)
   {  // consider transparent filter, resulting in trapez profile.
      s32_alpha = 0;
      s32_beta = 0;
      // Do not perform any actions regarding the LPF during an OPMODE change
      // on the fly in case that the filter is transparent.
      VAR(AX0_u16_Pcmd_Lpf_Opmode_Change_Init_Val) = 0;

   }
   else
   {
      s32_alpha = (long)((float)(2147483648.0 * (float)exp((float)((-6.2831853) * (float)BGVAR(s16_Move_Smooth_Lpf_Hz) * (float)VAR(AX0_u16_Pos_Loop_Tsp) *1e-8))));;
      s32_beta = 0x80000000 - s32_alpha;
      // Here calculate for how long the OPMODE change on-the-fly code waits until the LPF is being considered as initialized.
      // A LPF reaches 99.3% of a step response after 5 tau. So here we we calculate:
      //  (5 * tau) / SampleRate_posLoop = 5 * (1 / (2 * pi * MOVESMOOTHLPFHZ)) * (1 / 0.00025[s]) =
      //   5 / (2 * pi * MOVESMOOTHLPFHZ * 0.00025) = 5 / (0.0015708 * MOVESMOOTHLPFHZ)
      VAR(AX0_u16_Pcmd_Lpf_Opmode_Change_Init_Val) = (unsigned int)((float)5 / (float)(0.0015708*(float)BGVAR(s16_Move_Smooth_Lpf_Hz)));
      if(VAR(AX0_u16_Pcmd_Lpf_Opmode_Change_Init_Val) == 0) // If init counter is 0
      {
         // Set value at least to 1.
         VAR(AX0_u16_Pcmd_Lpf_Opmode_Change_Init_Val) = 1;
      }
   }

   // SS: Limit AX0_s16_Clear_Integral factor to 1, to avoid oscillations in cases where LPF_Hz > 635 Hz
   if ((BGVAR(s16_Move_Smooth_Lpf_Hz) >= 635) || (VAR(AX0_u16_Move_Smooth_Mode) == 0))
   {
      VAR(AX0_s16_Clear_Integral_Fix) = 1;
      VAR(AX0_u16_Clear_Integral_Shr) = 0;
   }
   else
   {
      if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC)
      FloatToFix16Shift16(&VAR(AX0_s16_Clear_Integral_Fix), &VAR(AX0_u16_Clear_Integral_Shr), 6.2831853 * (float)BGVAR(s16_Move_Smooth_Lpf_Hz) / 4000.0);
      else
          FloatToFix16Shift16(&VAR(AX0_s16_Clear_Integral_Fix), &VAR(AX0_u16_Clear_Integral_Shr), 6.2831853 * (float)BGVAR(s16_Move_Smooth_Lpf_Hz) / 8000.0);
   }
   // Copy variables when rt is not running
   LVAR(AX0_s32_Smooth_Alpha_Design) = s32_alpha;
   LVAR(AX0_s32_Smooth_Beta_Design) = s32_beta;
   VAR(AX0_s16_Crrnt_Run_Code) |= COPY_SHARED_POS_COEF_MASK;
   MoveSmoothFilterDesign(drive);
}


int SalMoveSmoothLpfCommand(long long param, int drive)
{
   // AXIS_OFF;
   long long s64_data;
   
   PosTuneActiveCommand(&s64_data,drive);

   // don't allow changes when the motor in motion and MOVESMOOTHMODE==1 to avoid "jumps".
   if ( Enabled(drive) && (VAR(AX0_u16_Motor_Stopped) != 2) && (VAR(AX0_u16_Move_Smooth_Mode) == 1) && (s64_data == 0))
       return MOTOR_IN_MOTION;

   BGVAR(s16_Move_Smooth_Lpf_Hz_User) = (int)param;
   SetMoveSmoothLpf(BGVAR(s16_Move_Smooth_Lpf_Hz_User), drive);

   return SAL_SUCCESS;
}


int SetMoveSmoothLpf(int value, int drive)
{
   BGVAR(s16_Move_Smooth_Lpf_Hz) = value;
   PtpFilterDesign(drive);
   SetGearingFFPointers(drive);

   return SAL_SUCCESS;
}


int SalMoveSmoothAvgNumCommand(long long param, int drive)
{
   // AXIS_OFF;

   unsigned long u32_temp_value;
   int s16_temp;

   // don't allow changes when the motor in motion and MOVESMOOTHMODE==2 to avoid "jumps".
   if ( Enabled(drive) && (VAR(AX0_u16_Motor_Stopped) != 2) && (VAR(AX0_u16_Move_Smooth_Mode) == 2))
       return MOTOR_IN_MOTION;
   
   if ( (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC) && (param > 128000LL) ) return VALUE_TOO_HIGH;

   u32_temp_value = (unsigned long)param;

   s16_temp = (int)((float)VAR(AX0_u16_Pos_Loop_Tsp) *0.01);

   if (((u32_temp_value / s16_temp) * s16_temp) != u32_temp_value) return VALUE_OUT_OF_RANGE;

   BGVAR(u32_Move_Smooth_Avg) = u32_temp_value;
   LVAR(AX0_u32_Move_Smooth_Avg_Num_Design) = (u32_temp_value / s16_temp);
   MoveSmoothFilterDesign(drive);

   return SAL_SUCCESS;
}


//**********************************************************
// Function Name: MotorMoving
// Description:
//
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(MotorMoving, "ramfunc_5");
void MotorMoving(int drive)
{
   // Ts = 1 m[sec]-> Fs = 1K[Hz]
   // Fc = 100Hz
   long long s64_tmp ;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   s64_tmp =  (long long)(VEL_VAR_FB_BETA * (long long)LVAR(AX0_s32_Vel_Var_Fb_0));
   s64_tmp += (long long)(VEL_VAR_FB_ALPHA * (long long)BGVAR(s32_Vel_Var_Fb_Fltr));
   BGVAR(s32_Vel_Var_Fb_Fltr)=(long)(s64_tmp >> VEL_VAR_FB_SHR);

   if (labs(BGVAR(s32_Vel_Var_Fb_Fltr))> VEL_VAR_FB_THRESHOLD)
   {
     BGVAR(u16_Vel_Zero_Time) = 0;
//   (BGVAR(s16_Motor_Moving) = (BGVAR(s32_Vel_Var_Fb_Fltr) < 0)?0xffff:1;
      if (BGVAR(s32_Vel_Var_Fb_Fltr) < 0)
         BGVAR(s16_Motor_Moving) = 0xffff;
      else
         BGVAR(s16_Motor_Moving) = 1;
   }
   else
   {
      if (BGVAR(u16_Vel_Zero_Time) < MOTOR_MOVING_TIME_THRESHOLD)
         BGVAR(u16_Vel_Zero_Time)++;
      else
         BGVAR(s16_Motor_Moving) = 0;
   }
}

// Desired algorithm described at I:\eng\CD-HD\Firmware\Design\Documents\Motor Stopped Block\Motor Stopped_X.doc
// But this algorithm is a compromised one, based on PEINPOS/TIME to detect end of motion
#pragma CODE_SECTION(MotorStoppedBlock, "ramfunc_5");
void MotorStoppedBlock(int drive)
{
   // AXIS_OFF;
   long s32_command = 0L;
   unsigned int u16_command_moving = 0;
   REFERENCE_TO_DRIVE;

   switch (VAR(AX0_s16_Opmode))
   {
      case 0:
      case 1:
         s32_command = (long)(*(int*)(VAR(AX0_s16_Vel_Loop_Cmnd_Ptr) & 0x0000FFFF));
         if (s32_command < 0L) s32_command = -s32_command;
         if (s32_command >> 1L)
            u16_command_moving = 1;
      break;

      case 4:
      case 8:
         s32_command = (long)(*(long*)(VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr) & 0x0000FFFF));
         if (s32_command < 0L) s32_command = -s32_command;
         if (s32_command > 1L)
            u16_command_moving = 1;
      break;

      case 2:
      case 3:
      default:
         u16_command_moving = 0;
      break;

   }

   if (u16_command_moving)
   {
      VAR(AX0_u16_Motor_Stopped) = 0;
   }
   else
   {
      // Wait for RT to signal position feedback inpos for inpostime
      if (VAR(AX0_u16_Motor_Stopped_Flags) & MOTOR_STOPPED_INDICATION_MASK)
         VAR(AX0_u16_Motor_Stopped) = 2;
      else
         VAR(AX0_u16_Motor_Stopped) = 1;
   }
}

//**********************************************************
// Function Name: GetExpectedMovementDirection
// Description: This function is used by limit switch code in order to determine
//              the expected movement. With the help of this function the limit
//              switch code can determine if there is a HOLD supposed to be triggered
//              or not. Only the command-value is considered in this function.
//
//              Return value: 0 = standstill, 1 = positive direction, -1 = negative direction
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(GetExpectedMovementDirection, "ramfunc_3")
int GetExpectedMovementDirection(int drive)
{
   // AXIS_OFF;

   int s16_expected_direction = 0; // 0 means no expected motion

   long long s64_temp;           // help variable
   unsigned int u16_temp_time;   // help variable
   REFERENCE_TO_DRIVE;

   // The following code is needed in order to perform the limit switch handling
   // in DS402 cyclic synchronous position mode only in case that all required
   // variables have refreshed/updated values. The code must run in this task
   // because the trigger of the alignment needs to be thread-safe and this
   // function can interrupt the BG function between changing the DS402 OPMODE
   // and the alignment trigger.
   if(p402_modes_of_operation_display != BGVAR(s8_Previous_DS402_Opmode_LS))
   {
      if (IS_CANOPEN_SYNC_POS_MODES)
      {
         // If DS402 opmode changed to CSP mode
         BGVAR(u8_Align_Prev_Temp_Pos_LS) = 1; // Set alignment request to true
      }
      BGVAR(s8_Previous_DS402_Opmode_LS) = p402_modes_of_operation_display;
   }

   // Use if/else instead of switch/case in RT.
   if (VAR(AX0_s16_Opmode) == 0)
   {
      if(p402_modes_of_operation_display != CYCLIC_SYNCHRONOUS_VELOCITY_MODE) //not sync velocity
      {
         s64_temp = (long long)VAR(AX0_s16_Serial_Vel_Cmnd);
      }
      else //sync velocity
      {
         // Read the fieldbus command value, make it thread-safe
         do
         {
            u16_temp_time = Cntr_3125;
            s64_temp = LLVAR(AX0_u32_FieldBus_Int_Lo);
         } while (u16_temp_time != Cntr_3125);
      }

      if (s64_temp > 0)
      {
         s16_expected_direction = 1; // expect positive movement
      }
      else if (s64_temp < 0)
      {
         s16_expected_direction = -1; // expect negative movement
      }
   }
   else if (VAR(AX0_s16_Opmode) == 1)
   {
      if (VAR(AX0_s16_Analog_Vel_Cmnd) > 0)
      {
         s16_expected_direction = 1; // expect positive movement
      }
      else if (VAR(AX0_s16_Analog_Vel_Cmnd) < 0)
      {
         s16_expected_direction = -1; // expect negative movement
      }
   }
   else if (VAR(AX0_s16_Opmode) == 2)
   {
      if(((p402_statusword & STATUSWORD_STATE) == OPERATION_ENABLED) && (p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_TORQUE_MODE))
      {
         // Read the fieldbus command value, make it thread-safe
         do
         {
            u16_temp_time = Cntr_3125;
            s64_temp = LLVAR(AX0_u32_FieldBus_Int_Lo);
         } while (u16_temp_time != Cntr_3125);
      }
      else
      {
         s64_temp = (long long)VAR(AX0_s16_Serial_Crrnt_Cmnd);
      }

      if (s64_temp > 0)
      {
         s16_expected_direction = 1; // expect positive movement
      }
      else if (s64_temp < 0)
      {
         s16_expected_direction = -1; // expect negative movement
      }
   }
   else if (VAR(AX0_s16_Opmode) == 3)
   {
      if(VAR(AX0_s16_Analog_Crrnt_Cmnd) > 0)
      {
         s16_expected_direction = 1; // expect positive movement
      }
      else if(VAR(AX0_s16_Analog_Crrnt_Cmnd) < 0)
      {
         s16_expected_direction = -1; // expect negative movement
      }
   }
   else if (VAR(AX0_s16_Opmode) == 4)
   {
      // For electronic gearing the expected movement is reported via bit 0
      // in "AX0_u16_Gear_Polarity_Flags".
      if ((VAR(AX0_u16_Gear_Polarity_Flags) & 0x0001)  == 0x0000)
         {
            s16_expected_direction = 1; // expect positive movement
         }
      else if ((VAR(AX0_u16_Gear_Polarity_Flags) & 0x0001) == 0x0001)
         {
            s16_expected_direction = -1; // expect negative movement
         }
   }
   else if (VAR(AX0_s16_Opmode) == 8)
   {
      // If the Drive is in cyclic synchronous position mode, which means that
      // PCMD is "usually" just a few RT cycles behind "BGVAR(s64_Temp_Pos_LS)".
      // Check also if the variable "BGVAR(s64_Previous_Temp_Pos_LS)" has been initialized.
      if((BGVAR(u8_Comm_Mode) == 1) && (IS_CANOPEN_SYNC_POS_MODES) && (!BGVAR(u8_Align_Prev_Temp_Pos_LS)))
      {
         // Get fieldbus position command value thread-safe
         do
         {
            u16_temp_time = Cntr_3125;
            s64_temp = BGVAR(s64_Temp_Pos_LS);
         } while (u16_temp_time != Cntr_3125);

         // Determine the motion direction in synchronous position mode based on
         // the pure derivative of the incoming position command values of the master.
         // This is needed since this code runs in a different thread (1[ms] task)
         // than the fieldbus position command is handled (250[us] task) and it is
         // "theoretically" possible that this function runs "exactly" at the time
         // where the position command value of the position loop reached the fieldbus
         // position command BGVAR(s64_Temp_Pos_LS) and therefore we would accidentally
         // report standstill.
         if((s64_temp - BGVAR(s64_Previous_Temp_Pos_LS)) > 0)
         {
            s16_expected_direction = 1; // expect positive movement
         }
         else if((s64_temp - BGVAR(s64_Previous_Temp_Pos_LS)) < 0)
         {
            s16_expected_direction = -1; // expect negative movement
         }
         // Save the current value
         BGVAR(s64_Previous_Temp_Pos_LS) = s64_temp;
      }
      else // No sycnhronous position mode
      {
         // Get PCMD thread-safe
         do
         {
            u16_temp_time = Cntr_3125;
            s64_temp = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
         } while (u16_temp_time != Cntr_3125);

         // Now check the expected direction of travel
         if (BGVAR(s16_Move_Abs_Issued) == 0) // If a relative motion has been triggered
         {
            if (BGVAR(s64_Temp_Pos_LS) > 0)
            {
               s16_expected_direction = 1; // expect positive movement
            }
            else if (BGVAR(s64_Temp_Pos_LS) < 0)
            {
               s16_expected_direction = -1; // expect negative movement
            }
         }
         else // An absolute motion has been triggered
         {
            if((BGVAR(s64_Temp_Pos_LS) - s64_temp) > 0)
            {
               s16_expected_direction = 1; // expect positive movement
            }
            else if((BGVAR(s64_Temp_Pos_LS) - s64_temp) < 0)
            {
               s16_expected_direction = -1; // expect negative movement
            }
         }
      }
   }
   else // Unknown mode of operation, not suppose to happen
   {
      s16_expected_direction = 0;
   }

   return s16_expected_direction;
}


//**********************************************************
// Function Name: LexiumLimitSwitchHold
// Description: This function is a Lexium specific limit switch function and
//              handles the limit switches in the Lexium manner.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(LexiumLimitSwitchHold, "ramfunc_3")
void LexiumLimitSwitchHold(int drive, int s16_trd_context)
{
   // AXIS_OFF;

   static int s16_previous_enable_state = 0;

   UNSIGNED8 manuFltCode[] = {0,0,0,0,0};
   unsigned int u16_can_err_code;

   int s16_expected_movement = GetExpectedMovementDirection(drive);
   unsigned int u16_rising_edge_CW_LS;
   unsigned int u16_rising_edge_CCW_LS;

   unsigned int u16_local_CW_LS  = VAR(AX0_u16_CW_LS)  & 0x03;
   unsigned int u16_local_CCW_LS = VAR(AX0_u16_CCW_LS) & 0x03;

   unsigned int u16_temp_time, u16_allow_releasing_LS_hold = 1; // Needs to be true in order to allow to release the limit-switch hold at all
   unsigned char u8_allow_P2_68_handling = 0x03; // Bit 0 for handling in CW, bit 1 for CCW direction
   long long s64_position_delta;                                // Delta between Master position and PCMD

   // start checking LS only after initialization is finshed (to allow sending EMCY packets only after bootup phase is finished).
   if (u16_Init_State) return;

   // Read the fieldbus command value, make it thread-safe
   do {
      u16_temp_time = Cntr_3125;
      s64_position_delta = BGVAR(s64_Temp_Pos_LS) - LLVAR(AX0_u32_Pos_Cmd_User_Lo); // Delta between Master position and PCMD
   } while (u16_temp_time != Cntr_3125);


   // At this place the homing code tells us to eventually discard a limit-switch event.
   if(BGVAR(u8_Homing_Ignore_LS) & 0x01) // If negative limit switch is supposed to be ignored
   {
      u16_local_CCW_LS = 0;
   }
   if(BGVAR(u8_Homing_Ignore_LS) & 0x02) // If positive limit switch is supposed to be ignored
   {
      u16_local_CW_LS = 0;
   }

   // Check for rising edge in CW LS
   u16_rising_edge_CW_LS = u16_local_CW_LS ^ BGVAR(u16_Prev_CW_LS_State);        // Bitwise XOR = differences in the variables
   u16_rising_edge_CW_LS = u16_rising_edge_CW_LS & u16_local_CW_LS;              // Determine rising edge in the lower 2 bits of "VAR(AX0_u16_CW_LS)"

   // Check for rising edge in CCW LS
   u16_rising_edge_CCW_LS = u16_local_CCW_LS ^ BGVAR(u16_Prev_CCW_LS_State);     // Bitwise XOR = differences in the variables
   u16_rising_edge_CCW_LS = u16_rising_edge_CCW_LS & u16_local_CCW_LS;           // Determine rising edge in the lower 2 bits of "VAR(AX0_u16_CCW_LS)"

   // Save old state of the limit switches
   BGVAR(u16_Prev_CW_LS_State)  = u16_local_CW_LS;
   BGVAR(u16_Prev_CCW_LS_State) = u16_local_CCW_LS;


   // If the Drive is in cyclic synchronous position mode --> Determine specific actions.
   if((BGVAR(u8_Comm_Mode) == 1) && (IS_CANOPEN_SYNC_POS_MODES))
   {
      if( ((u16_local_CW_LS)  && (s64_position_delta >= 0)) ||
          ((u16_local_CCW_LS) && (s64_position_delta <= 0)) )
      {
         // Do not allow releasing a hold in case that the fieldbus command would
         // cause motion in direction of an active LS. This is required since the
         // direction of motion in cyclic sycnhronous position mode is based on the
         // pure derivative of the fieldbus position command value.
         u16_allow_releasing_LS_hold = 0;
      }

      // Here avoid that due to jitter in the fieldbus position command caused by the position alignment
      // of the master (fieldBusPosCmd = PFB) and nibble Z set in P2-68, that the alarms of the LS are
      // immediately triggered again after an ARST.
      if(s64_position_delta < 1073741824LL)
      {
         u8_allow_P2_68_handling &= ~0x01; // Clear bit 0
      }
      if(s64_position_delta > -1073741824LL)
      {
         u8_allow_P2_68_handling &= ~0x02; // Clear bit 1
      }
   }

   // If there is a rising edge in the enable state
   if ((!s16_previous_enable_state) && (Enabled(drive)))
   {
      // If a positive LS is active while Drive gets enabled
      if(u16_local_CW_LS)
      {
         // Fake one time a rising edge in the CW HW LS activation bit, since this
         // triggers an Alarm, which is the situation that is demanded in such a case.
         u16_rising_edge_CW_LS = 0x0002;
      }
      // If a negative LS is active while Drive gets enabled
      if (u16_local_CCW_LS)
      {
         // Fake one time a rising edge in the CCW HW LS activation bit, since this
         // triggers an Alarm, which is the situation that is demanded in such a case.
         u16_rising_edge_CCW_LS = 0x0002;
      }
   }
   s16_previous_enable_state = Enabled(drive); // Update the previous enable state variable

   // If there is any movement into an active CW limit switch (software LS or hardware LS) or is there an edge in the CW limit switch detection
   if (((u16_local_CW_LS) && (s16_expected_movement == 1)) || (u16_rising_edge_CW_LS != 0))
   {
      BGVAR(u16_Hold_Bits) |= HOLD_LXM_LIMIT_SWITCH_MASK;  // Issue a "LS hold triggered by Lexium"

      // nitsan 21/5/14: with johachim eichner it was decided that limit switch and sw limit events will be ignored
      // in canopen mode and handled according to ds402 standard
      if ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_CANOPEN)
      {
         // in canopen, use decstop for quick stop deceleration
         BGVAR(u16_Deceleration_Event_ID) = EVENT_DECSTOP;
      }
      else
      {
         // get the specific event ID
         if (u16_local_CW_LS & 0x1)  // CW (positive) software limit switch event
              BGVAR(u16_Deceleration_Event_ID) = EVENT_SOFTWARE_POSITIVE_LS_DEC;

         if (u16_local_CW_LS & 0x2)  // CW (positive) hardware limit switch event
         {
              BGVAR(u16_Deceleration_Event_ID) = EVENT_HARDWARE_POSITIVE_LS_DEC;
         }
      }

      // The following if-condition has only an impact on OPMODE = 4 and is the pulse inhibit feature
      if((VAR(AX0_u8_Gear_Limits_Mode) & 0x0002) == 0x0000) // If pulse-inhibit in direction of the limit-switch is active
      {
         // Allow pulse-inhibit only in case that the hold process has been started. Otherwise we already decelerate
         // although hold process is not yet active.
         if(VAR(AX0_u16_HP_Flags) & HOLD_IN_PROCESS_MASK)
         {
            VAR(AX0_u16_Gear_BG_Flags) |= 0x0001;  // Tell electronic gearing to discard pulses in positive direction
         }
      }

      // If an edge was detected in the CW limit switches --> Set a warning for one time OR
      // If nibble Z in P2-68 shows 1, needed for the 3S fieldbus Driver, new reauest from A. Badura
      // Fix IPR 1458. Consider P2-68 nibble Z only for CANopen sync positon modes
      if ((u16_rising_edge_CW_LS != 0) || 
          ((IS_CANOPEN_SYNC_POS_MODES) && (BGVAR(u16_P2_68_Auto_Enable_Actual) & 0x0100) && (u8_allow_P2_68_handling & 0x01)))
      {
         if(u16_local_CW_LS & 0x0002) // If HW LS tripped
         {
            BGVAR(u8_Show_Lex_LS_Warning) |= LEX_LS_AL015_WARNING;  // Set bit 1 in order to show AL015 for CW HW LS
            u16_can_err_code = ERRCODE_CAN_POS_LIMIT_SWITCH;
            manuFltCode[3] = 0x15;
            manuFltCode[4] = 0x00;
         }
         else
         {
            BGVAR(u8_Show_Lex_LS_Warning) |= LEX_LS_AL283_WARNING;  // Set bit 3 in order to show AL283 for CCW HW LS
            u16_can_err_code = ERRCODE_CAN_POS_SW_LIMIT;
            manuFltCode[3] = 0x83;
            manuFltCode[4] = 0x02;
         }

         if (IS_CAN_DRIVE_AND_COMMODE_1)
         {
            p301_error_register |= (0x80 | 0x20);   // manufacturer-specific | device-profile-specific
            p402_error_code = u16_can_err_code;
            //do not use: GetEmcyManufcturerCode(drive, u64_err_bit, 2, manuFltCode);
            // becuase it uses const array while this is ISR context
            writeEmcyReq(s16_trd_context, u16_can_err_code, manuFltCode CO_COMMA_LINE_PARA);
         }
      }

      // If the speed was negative when the positive LS has been activated
      if(s16_expected_movement == -1)
      {
         BGVAR(u8_Show_Lex_LS_Warning) |= LEX_LS_UNEXP_LIMIT_SWITCH;
      }
   }
   // If there is any movement into an active CCW limit switch (software LS or hardware LS) or is there an edge in the CCW limit switch detection
   else if (((u16_local_CCW_LS) && (s16_expected_movement == -1)) || (u16_rising_edge_CCW_LS != 0))
   {
      BGVAR(u16_Hold_Bits) |= HOLD_LXM_LIMIT_SWITCH_MASK;  // Issue a "LS hold triggered by Lexium"

      // nitsan 21/5/14: with johachim eichner it was decided that limit switch and sw limit events will be ignored
      // in canopen mode and handled according to ds402 standard
      if ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_CANOPEN)
      {
         // in canopen, use decstop for quick stop deceleration
         BGVAR(u16_Deceleration_Event_ID) = EVENT_DECSTOP;
      }
      else
      {
         // get the specific event ID
         if (u16_local_CCW_LS & 0x1)  // CCW (negative) software limit switch event
            BGVAR(u16_Deceleration_Event_ID) = EVENT_SOFTWARE_NEGATIVE_LS_DEC;

         if (u16_local_CCW_LS & 0x2)  // CCW (negative) hardware limit switch event
         {
               BGVAR(u16_Deceleration_Event_ID) = EVENT_HARDWARE_NEGATIVE_LS_DEC;
         }
      }

      // The following if-condition has only an impact on OPMODE = 4 and is the pulse inhibit feature
      if((VAR(AX0_u8_Gear_Limits_Mode) & 0x0002) == 0x0000) // If pulse-inhibit in direction of the limit-switch is active
      {
         // Allow pulse-inhibit only in case that the hold process has been started. Otherwise we already decelerate
         // although hold process is not yet active.
         if(VAR(AX0_u16_HP_Flags) & HOLD_IN_PROCESS_MASK)
         {
            VAR(AX0_u16_Gear_BG_Flags) |= 0x0002;  // Tell electronic gearing to discard pulses in negative direction
         }
      }

      // If an edge was detected in the CCW limit switches --> Set a warning for one time OR
      // If nibble Z in P2-68 shows 1, needed for the 3S fieldbus Driver, new reauest from A. Badura
      // Fix IPR 1458. Consider P2-68 nibble Z only for CANopen sync positon modes
      if ((u16_rising_edge_CCW_LS != 0) || 
          ((IS_CANOPEN_SYNC_POS_MODES) && (BGVAR(u16_P2_68_Auto_Enable_Actual) & 0x0100) && (u8_allow_P2_68_handling & 0x02)))
      {
         if(u16_local_CCW_LS & 0x0002) // If HW LS tripped
         {
            BGVAR(u8_Show_Lex_LS_Warning) |= LEX_LS_AL014_WARNING;  // Set bit 0 in order to show AL014 for CCW HW LS
            u16_can_err_code = ERRCODE_CAN_NEG_LIMIT_SWITCH;
            manuFltCode[3] = 0x14;
            manuFltCode[4] = 0x00;
         }
         else
         {
            BGVAR(u8_Show_Lex_LS_Warning) |= LEX_LS_AL285_WARNING;  // Set bit 4 in order to show AL285 for CCW HW LS
            u16_can_err_code = ERRCODE_CAN_NEG_SW_LIMIT;
            manuFltCode[3] = 0x85;
            manuFltCode[4] = 0x02;
         }

         if (IS_CAN_DRIVE_AND_COMMODE_1)
         {
            p301_error_register |= (0x80 | 0x20);   // manufacturer-specific | device-profile-specific
            p402_error_code = u16_can_err_code;
            //do not use: GetEmcyManufcturerCode(drive, u64_err_bit, 2, manuFltCode);
            // becuase it uses const array while this is ISR context
            writeEmcyReq(s16_trd_context, u16_can_err_code, manuFltCode CO_COMMA_LINE_PARA);
         }
      }

      // If the speed was positive when the negative LS has been activated
      if(s16_expected_movement == 1)
      {
         BGVAR(u8_Show_Lex_LS_Warning) |= LEX_LS_UNEXP_LIMIT_SWITCH;
      }
   }

   // Check if a limit-switch hold is pending (save execution time) and check if it is allowed releasing the LS hold
   if((BGVAR(u16_Hold_Bits) & HOLD_LXM_LIMIT_SWITCH_MASK) && (u16_allow_releasing_LS_hold != 0))
   {
      // If there is an attempt of moving out of the active LS
      if( ((u16_local_CW_LS)  && (s16_expected_movement == -1) && !(u16_local_CCW_LS)) ||  // Active CW LS, negative movement, Inactive CCW LS
          ((u16_local_CCW_LS) && (s16_expected_movement == 1)  && !(u16_local_CW_LS))      // Active CCW LS, positive movement, Inactive CW LS
        )
      {
         if( (BGVAR(u16_P2_68_Auto_Enable_Actual) & 0x0010) &&                   // If it is allowed moving out of the LS without a previous clear-fault AND
             (!(BGVAR(u8_Show_Lex_LS_Warning) & LEX_LS_UNEXP_LIMIT_SWITCH)) )    // If there hasn't been a unexpected LS indication
         {
            BGVAR(u16_Hold_Bits) &= ~HOLD_LXM_LIMIT_SWITCH_MASK;  // Release the hold
            BGVAR(u8_Show_Lex_LS_Warning) = LEX_LS_NO_WARNING;    // Clear the waring AL014/015
            if (IS_CAN_DRIVE_AND_COMMODE_1)
            {
               if ( (BGVAR(s64_SysNotOk) == 0) && (BGVAR(s64_SysNotOk_2) == 0) )
               {
                  p301_error_register = 0;
                  writeEmcyReq(s16_trd_context, 0,(UNSIGNED8 *)"\0\0\0\0\0" CO_COMMA_LINE_PARA);
               }
            }
         }
         else // Move out of limit switch requires previous ARST (alarm-reset = clear-fault)
         {
            if(BGVAR(u8_Show_Lex_LS_Warning) == LEX_LS_NO_WARNING) // If the LS warning has been previously cleared by a ARST
            {
               BGVAR(u16_Hold_Bits) &= ~HOLD_LXM_LIMIT_SWITCH_MASK;  // Release the hold
            }
         }
         // The next instruction has only an impact on OPMODE = 4 and is the pulse inhibit feature
         VAR(AX0_u16_Gear_BG_Flags) &= ~0x0003; // Do not discard pulses in positive or negative direction
      }
      // If no limit-switches are active anymore AND if the LS warning has been previously cleared by a ARST.
      // This use-case happens in case that a limit-switch is crossed due to a low deceleration ramp (a ARST is mandatory)
      else if ((u16_local_CW_LS == 0) && (u16_local_CCW_LS == 0) && (BGVAR(u8_Show_Lex_LS_Warning) == LEX_LS_NO_WARNING))
      {
         BGVAR(u16_Hold_Bits) &= ~HOLD_LXM_LIMIT_SWITCH_MASK;  // Release the hold

         // The next instruction has only an impact on OPMODE = 4 and is the pulse inhibit feature
         VAR(AX0_u16_Gear_BG_Flags) &= ~0x0003; // Do not discard pulses in positive or negative direction
      }
   }
}


//**********************************************************
// Function Name: LimitSwitchHold
// Description:
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(LimitSwitchHold, "ramfunc_3")
void LimitSwitchHold(int drive)
{
   // AXIS_OFF;

   int s16_expected_movement = GetExpectedMovementDirection(drive);
   int u16_is_pos_still_in_cw_ls = 0, u16_is_pos_still_in_ccw_ls = 0;
   unsigned int u16_temp_time, u16_temp_cw_ls, u16_temp_ccw_ls;
   unsigned int u16_allow_releasing_LS_hold = 1; // Needs to be true in order to allow to release the limit-switch hold at all
   long long s64_position_delta;                 // Position delta between fieldbus master position and PCMD

   // Ignore Transient (Phantom) Bits when setting Limits Mode
   if ((VAR(AX0_u16_SW_Pos_lim) & 0x02) == 0x02)
   {  // Do not use the hardware limit-switch phantom bits
      u16_temp_cw_ls  = VAR(AX0_u16_CW_LS)  & 0x03;
      u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS) & 0x03;
      u16_is_pos_still_in_cw_ls =  (((LLVAR(AX0_u32_Temp_Pos_Lo) > LLVAR(AX0_u32_Pos_Max_Hyst_Lo)) && (VAR(AX0_u16_CW_LS)  & 0x01)) || (VAR(AX0_u16_CW_LS)  & 0x02));
      u16_is_pos_still_in_ccw_ls = (((LLVAR(AX0_u32_Temp_Pos_Lo) < LLVAR(AX0_u32_Pos_Min_Hyst_Lo)) && (VAR(AX0_u16_CCW_LS)  & 0x01))|| (VAR(AX0_u16_CCW_LS)  & 0x02));
   }
   else
   {  // Involve the hardware limit-switch phantom bits
      u16_temp_cw_ls  = VAR(AX0_u16_CW_LS);
      u16_temp_ccw_ls = VAR(AX0_u16_CCW_LS);
      u16_is_pos_still_in_cw_ls =  (((LLVAR(AX0_u32_Temp_Pos_Lo) > LLVAR(AX0_u32_Pos_Max_Hyst_Lo)) && (VAR(AX0_u16_CW_LS)  & 0x01)) || (VAR(AX0_u16_CW_LS)  & 0xE)  );
      u16_is_pos_still_in_ccw_ls = (((LLVAR(AX0_u32_Temp_Pos_Lo) < LLVAR(AX0_u32_Pos_Min_Hyst_Lo)) && (VAR(AX0_u16_CCW_LS)  & 0x01))|| (VAR(AX0_u16_CCW_LS) & 0xE)  );
   }

   // If there is an attempt moving into the active CW limit switch. Consider also the phantom bits bit 2 & 3!
   if ((u16_temp_cw_ls  && (s16_expected_movement == 1)) && (u16_is_pos_still_in_cw_ls))
   {
      BGVAR(u16_Hold_Bits) |= HOLD_LIMIT_SWITCH_MASK;

      // If OPMODE = electronic gearing and pulse inhibit feature is on
      if ((VAR(AX0_s16_Opmode)==4) && ((VAR(AX0_u8_Gear_Limits_Mode) & 0x0002) == 0x0000))
      {
         // Allow pulse-inhibit only in case that the hold process has been started in the RT task.
         // Otherwise we already prevent motion although hold process is not yet active.
         if(VAR(AX0_u16_HP_Flags) & HOLD_IN_PROCESS_MASK)
         {
            VAR(AX0_u16_Gear_BG_Flags) |= 0x0001;  // Tell electronic gearing to discard pulses in positive direction
            VAR(AX0_u16_Gear_BG_Flags) &= ~0x0002; // and to evaluate pulses in negative direction.
         }
      }
   }
   // If there is an attempt moving into the active CCW limit switch. Consider also the phantom bits bit 2 & 3!
   else if ((u16_temp_ccw_ls && (s16_expected_movement == -1)) && (u16_is_pos_still_in_ccw_ls))
   {
      BGVAR(u16_Hold_Bits) |= HOLD_LIMIT_SWITCH_MASK;

      // If OPMODE = electronic gearing and pulse inhibit feature is on
      if ((VAR(AX0_s16_Opmode)==4) && ((VAR(AX0_u8_Gear_Limits_Mode) & 0x0002) == 0x0000))
      {
         // Allow pulse-inhibit only in case that the hold process has been started in the RT task.
         // Otherwise we already prevent motion although hold process is not yet active.
         if(VAR(AX0_u16_HP_Flags) & HOLD_IN_PROCESS_MASK)
         {
            VAR(AX0_u16_Gear_BG_Flags) |= 0x0002;  // Tell electronic gearing to discard pulses in negative direction
            VAR(AX0_u16_Gear_BG_Flags) &= ~0x0001; // and to evaluate pulses in positive direction.
         }
      }
   }
   else // Here check if we are allowed to release the limit switch hold.
   {
      // If no LS is active anymore
      if (((!u16_temp_cw_ls) && (!u16_temp_ccw_ls)) || ((!u16_is_pos_still_in_cw_ls) && (!u16_is_pos_still_in_ccw_ls)))
      {
         BGVAR(u16_Hold_Bits) &= ~HOLD_LIMIT_SWITCH_MASK;
         VAR(AX0_u16_Gear_BG_Flags) &= ~0x0003; // Do not discard pulses in positive or negative direction (just for electronic gearing)
      }
      // If there is an attempt moving out of the limit switch
      else if( (u16_temp_cw_ls  && (s16_expected_movement == -1)) ||
               (u16_temp_ccw_ls && (s16_expected_movement == 1)) )
      {
         // If the Drive is in cyclic synchronous position mode --> Determine specific actions.
         if((BGVAR(u8_Comm_Mode) == 1) && (IS_CANOPEN_SYNC_POS_MODES))
         {
            // Read the fieldbus command value minus PCMD in a thread-safe manner
            do
            {
               u16_temp_time = Cntr_3125;
               s64_position_delta = BGVAR(s64_Temp_Pos_LS) - LLVAR(AX0_u32_Pos_Cmd_User_Lo); // Delta between Master position and PCMD
            }
            while (u16_temp_time != Cntr_3125);

            if( (u16_temp_cw_ls  && (s64_position_delta >= 0)) ||
                (u16_temp_ccw_ls && (s64_position_delta <= 0))   )
            {
               // Do not allow releasing a hold in case that the fieldbus command would
               // cause motion in direction of an active LS. This is required since the
               // expected direction of motion in cyclic sycnhronous position mode is based
               // on the pure derivative of the fieldbus position command value.
               u16_allow_releasing_LS_hold = 0;
            }
         }

         if(u16_allow_releasing_LS_hold) // If it is allowed releasing the LS hold
         {
            BGVAR(u16_Hold_Bits) &= ~HOLD_LIMIT_SWITCH_MASK;
         }
      }
   }

   // Set/clear bit 0 in a RT variable depending on if the LS hold is activated or not.
   if (BGVAR(u16_Hold_Bits) & HOLD_LIMIT_SWITCH_MASK)
   {
       VAR(AX0_u16_LS_Hold) = 1;
   }
   else
   {
       VAR(AX0_u16_LS_Hold) = 0;
   }

   if (VAR(AX0_u16_LS_Hold) != 0)
   {
       BGVAR(u16_Deceleration_Event_ID) = EVENT_DECSTOP;
   }
}


// This function is called to init the fieldbus interpolation block
// It calculates the # of samples needs to be interpolated and init the prev target value to the
// current trget.
// This should be called only once before starting the interpolation mechanism
void FieldBusInterpolationInit(int drive, unsigned int u16_interpolation_type)
{
   // AXIS_OFF;
   long s32_fix;
   unsigned int u16_shift, u16_tmp = 0;
   long long half_for_rounding = 0LL;
   float f_N;
   REFERENCE_TO_DRIVE;

   VAR(AX0_u16_FieldBus_Int_Type) = u16_interpolation_type;

   if (VAR(AX0_u16_FieldBus_Int_Type) == POSITION_INTERPOLATION)//ITAI -  Position 125 usec
   {
      if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)
         u16_tmp = 8000;
     else //VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC
        u16_tmp = 4000;
   }
   else //VELOCITY_INTERPOLATION or CURRENT_INTERPOLATION //ITAI -  Position 125 usec
   {
      u16_tmp = (unsigned int)((unsigned int)32000 / VAR(AX0_u16_FieldBus_Int_Type));
   }

   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(fbus_cycle_time_in_secs).s32_fbus_cycle_time_fix, BGVAR(fbus_cycle_time_in_secs).u16_fbus_cycle_time_shr, (long)u16_tmp, (unsigned int)0);

   if (u16_shift > 0)
      half_for_rounding = 1LL << ((long long)u16_shift - 1);

   VAR(AX0_u16_FieldBus_Int_Time) = (int)(((long long)s32_fix + half_for_rounding) >> (long long)u16_shift);

   if (!VAR(AX0_u16_FieldBus_Int_Time))
      VAR(AX0_u16_FieldBus_Int_Time) = u16_Num_Of_Mts_In_Sync_Period / u16_interpolation_type;

   if (!VAR(AX0_u16_FieldBus_Int_Time))
      VAR(AX0_u16_FieldBus_Int_Time) = 1;

   f_N = VAR(AX0_u16_FieldBus_Int_Time);
   LVAR(AX0_f_One_Over_N) = (float)1 / f_N;
}


//**********************************************************
// Function Name: FieldBusLinearInterpolationSetTarget
// Description:
//    This function is called in cyclic synchronous position, velocity and torque
//    OPMODE. This function is responsible for setting / initializing all relevant
//    variables, which are needed for the linear interpolation function in the
//    assembly code to work properly.
//
// Author:
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(FieldBusLinearInterpolationSetTarget, "ramfunc");
void FieldBusLinearInterpolationSetTarget(int drive, long long s64_abs_target_pos)
{
   // AXIS_OFF;
   long long s64_delta_pos, s64_temp/*, s64_delta_pos_prev*/;
   int s16_neg_sign = 0;
   REFERENCE_TO_DRIVE;

   // If a mode of operation change via fieldbus is pending
   if ((VAR(AX0_s16_Opmode_Change_Flags) & OPMODE_CHANGE_VIA_FB_PENDING_MASK) != 0)
   {
      // Just update the Fieldbus Index but do not load the interpolator.
      VAR(AX0_s16_FieldBus_Int_Index) = (int)VAR(AX0_u16_FieldBus_Int_Time);
      //a new command arrived - reset the missing command packets counter
      VAR(AX0_u16_FieldBus_Missing_Cmd_Ctr) = 0;
      return;
   }

   //  Build the difference between the new command and the current output of the interpolator
   s64_delta_pos = s64_abs_target_pos - LLVAR(AX0_u32_FieldBus_Int_Lo);

   VAR(AX0_u16_FieldBus_Missing_Cmd_Ctr) = 0;  //a new command arrived - reset the missing command packets counter

   //get abs delta position from previos 2 position commands in order to know if we are accing or decing
//   s64_delta_pos_prev = LLVAR(AX0_u32_FieldBus_Int_Lo) - LLVAR(AX0_u32_FieldBus_Int_Linear_Prev_Target_Lo_2);

//   if(s64_delta_pos_prev < 0)
//      s64_delta_pos_prev = -s64_delta_pos_prev;

   //For ACC/DEC check - a(i) = v(i) - v(i-1) = p(i) - p(i-1) -(p(i-1) - p(i-2)) = p(i) - 2*p(i-1) + p(i-2)
   //s64_acc_dec = (s64_abs_target_pos - 2 * LLVAR(AX0_u32_FieldBus_Int_Target_Lo) + LLVAR(AX0_u32_FieldBus_Int_Linear_Prev_Target_Lo_2));

      //abs
//      if(s64_acc_dec < 0)
//         s64_acc_dec = -s64_acc_dec;

   //update position command from 2 syncs ago
//   LLVAR(AX0_u32_FieldBus_Int_Linear_Prev_Target_Lo_2) = LLVAR(AX0_u32_FieldBus_Int_Target_Lo);

   // Indicate to the interpolation function if delta-pos is positive or negative.
   // This is needed for the fraction handling.
   VAR(AX0_u16_FieldBus_Int_Sign) = (unsigned int)(s64_delta_pos < 0LL);

   //check to see if VLIM is exceeded on sync position mode
   if (s64_delta_pos < 0)
   {
      s64_delta_pos = -s64_delta_pos;
      s16_neg_sign = 1;
   }

   // Fault generation in CSP mode
   //  a) Drive is in enable operation state
   //  b) Drive is in cyclic sycnchronous position mode
   //  c) Flag "command ignored", which is low active, is set
   if ( ((p402_statusword & STATUSWORD_STATE) == OPERATION_ENABLED)                       &&
        //(p402_modes_of_operation_display == CYCLIC_SYNCHRONOUS_POSITION_MODE) &&
        (IS_CANOPEN_SYNC_POS_MODES) && (p402_statusword & LOW_ACTIVE_COMMAND_IGNORED_MASK)  )
   {
      // If the velocity is too high
      if (s64_delta_pos > LLVAR(AX0_u32_Fieldbus_Vlim_Design_Lo))
      {
         VAR(AX0_s16_Fieldbus_Target_Postion_Flag) += 1;
         // Modified Flag to count Excessive Velocity events in lower 8-Bits...
         LVAR(AX0_u32_Fieldbus_Target_Pos_Err_Cntr) += 1; // Accumulating Counter
         VAR(AX0_s16_Fieldbus_Postion_Cmd_Err_Flag) = 1;
      }
      else
      {
         VAR(AX0_s16_Fieldbus_Target_Postion_Flag) &= 0xFF00;
         VAR(AX0_s16_Fieldbus_Postion_Cmd_Err_Flag) = 0;
      }

      // Decided to remove ACC/DEC fault monitoring
      // check to see if ACC/DEC is exceeded on sync position mode
/*
      if((s64_delta_pos - s64_delta_pos_prev) > 0)// if ACC
      {
         //test for fault only if drive is in enable operation state
         if((s64_acc_dec > (long long)LLVAR(AX0_u32_Fieldbus_Acc_Design_Lo)) && ((p402_controlword & 0x000F) == 0x0000F))
         {
            VAR(AX0_s16_Fieldbus_Target_Postion_Flag) += 0x0100;
            // Modified Flag to count Excessive Acc/Dec events in upper 8-Bits...
            VAR(AX0_s16_Fieldbus_Postion_Cmd_Err_Flag) = 1;
         }
         else
         {
            VAR(AX0_s16_Fieldbus_Target_Postion_Flag) &= 0x00FF;
            VAR(AX0_s16_Fieldbus_Postion_Cmd_Err_Flag) = 0;
         }
      }
      else if((s64_delta_pos - s64_delta_pos_prev) < 0) // if DEC
      {
         //test for fault only if drive is in enable operation state
         if((s64_acc_dec > (long long)LLVAR(AX0_u32_Fieldbus_Dec_Design_Lo)) && ((p402_controlword & 0x000F) == 0x0000F))
         {
            VAR(AX0_s16_Fieldbus_Target_Postion_Flag) += 0x0100;
            // Modified Flag to count Excessive Acc/Dec events in upper 8-Bits...
            LVAR(AX0_u32_Fieldbus_Target_Pos_Err_Cntr) += 1; // Accumulating Counter
            VAR(AX0_s16_Fieldbus_Postion_Cmd_Err_Flag) = 1;
         }
         else
         {
            VAR(AX0_s16_Fieldbus_Target_Postion_Flag) &= 0x00FF;
            VAR(AX0_s16_Fieldbus_Postion_Cmd_Err_Flag) = 0;
         }
      } */
   }

   if (VAR(AX0_s16_Fieldbus_Postion_Cmd_Err_Flag) == 1) // if vlim or ACC/DEC exceeded
   {
      s64_delta_pos = LLVAR(AX0_u32_Delta_Pos_Error_Lo); //replace "broken" delta with the last valid delta

      //update position command from 1 syncs ago
      LLVAR(AX0_u32_FieldBus_Int_Target_Lo) = s64_delta_pos + LLVAR(AX0_u32_FieldBus_Int_Lo);
   }
   else if (VAR(AX0_s16_Fieldbus_Postion_Cmd_Err_Flag) == 0) //if vlim and ACC/DEC not exceeded
   {
      if (s16_neg_sign)//if sign was changed for s64_delta_pos then change it back
      {
         s64_delta_pos = -s64_delta_pos;
      }
      LLVAR(AX0_u32_Delta_Pos_Error_Lo) = s64_delta_pos;
      LLVAR(AX0_u32_FieldBus_Int_Target_Lo) = s64_abs_target_pos;
   }

   //calculate the delta needed for current cycle. This value is used in the interpolator function.
   LLVAR(AX0_u32_FieldBus_Delta_Int_Lo) = s64_delta_pos / (long long)VAR(AX0_u16_FieldBus_Int_Time);

   //handle residue. This is the value that is after the decimal point for each delta
   s64_temp = s64_delta_pos - (LLVAR(AX0_u32_FieldBus_Delta_Int_Lo) * (long long)VAR(AX0_u16_FieldBus_Int_Time));
   if (s64_temp < 0) s64_temp = -s64_temp;

   //move 16 to the left and spread to all cycles. Scale by 65536 and divide afterwards by fieldbus samle time. Every time
   //when accumulationg the fraction and a 16-bit overflow happens, we need to add 1 count to the interpolation result.
   VAR(AX0_u16_FieldBus_Delta_Int_Frac) = (int)((s64_temp * 65536LL) / (long long)VAR(AX0_u16_FieldBus_Int_Time));
   // Clear the variable that holds the accumulated fraction values.
   VAR(AX0_u16_FieldBus_Residue) = 0;

   VAR(AX0_s16_FieldBus_Int_Index) = (int)VAR(AX0_u16_FieldBus_Int_Time);
}


#pragma CODE_SECTION(FieldBusCubicInterpolationSetTarget, "ramfunc");
void FieldBusCubicInterpolationSetTarget(int drive, long long s64_abs_target_pos, long s32_target_vel, int s16_type)
{
   // AXIS_OFF;
   int s16_temp_time, s16_extrapolation_i, s16_cycle_nbr_capture, s16_param_received = 0x1;
   long s32_FieldBus_End_Vel = 0;
   long long s64_2dp = 0LL, s64_extrapolation_compensation = 0LL, s64_d = 0LL, s64_max,
    s64_value_at_end, s64_temp1, s64_temp2, s64_b = 0LL;
   float f_a = 0.0, f_b = 0.0, f_c = 0.0, f_N, f_One_Over_N2, f_max, f_Xmax, dt_to_dT = 0.0, dt_to_dT_square = 0.0;
   REFERENCE_TO_DRIVE;

   // Don't forget to limit interpolation time to 1 second
   // I think that it could handle around 15 sec but I didn't test it
   //
   // Determine what we received
   if (s16_type == POS_VALUE)
   {
      VAR(AX0_u16_FieldBus_Cubic_Msg_Flag) |= 0x1;
      LLVAR(AX0_u32_FieldBus_Int_Target_Lo) = s64_abs_target_pos;
   }
   else // if (s16_type == VEL_VALUE)
   {
      VAR(AX0_u16_FieldBus_Cubic_Msg_Flag) |= 0x2;
      LLVAR(AX0_u32_FieldBus_Int_2Vel_Lo) = (((long long) s32_target_vel) * VAR(AX0_u16_FieldBus_Int_Time)) << 2;
      // This is to translate from velocity internal units in 32bits/Tsv to 32bits/Tsp
   }
   // End Determine what we received

   // the two lines bellow could be moved to init
   if (VAR(AX0_u16_FieldBus_Int_Mode) == CUBIC_2PV_INTERPOLATION)
      s16_param_received = 0x3;

   if (VAR(AX0_u16_FieldBus_Cubic_Msg_Flag) == s16_param_received)
   {
      // Compute compensation for extrapolation
      VAR(AX0_u16_FieldBus_Int_change_counter) = 65535;  // prevent process new target point while computing next trajectory
      VAR(AX0_s16_FieldBus_Int_Index) = (int)VAR(AX0_u16_FieldBus_Int_Time) + 1;
      // Use Int_Index to detect Missing Command with Cubic Interpolation as well...
      VAR(AX0_u16_FieldBus_Missing_Cmd_Ctr) = 0; // Zero Missed-Command Counter, set Fault only on consecutive missing commands...
      s16_cycle_nbr_capture = VAR(AX0_u16_FieldBus_Int_n);
      s16_extrapolation_i = s16_cycle_nbr_capture - VAR(AX0_u16_FieldBus_Int_Time);
      // This includes an advance of 1 in case this computation is interupted by the interpolation
      if (VAR(AX0_u16_FieldBus_Int_Time) < 2)
         // for short cycles, cancel advance (the advance would be problematic for cycle time == 1)
         s16_extrapolation_i -= 1;        // Those short cycle times can only be reached with ethercat
      if (s16_extrapolation_i < 0)        // no need for advance in this case
         s16_extrapolation_i = 0;
      else if (s16_extrapolation_i >= (VAR(AX0_u16_FieldBus_Int_Time) * 3))
         s16_extrapolation_i = (VAR(AX0_u16_FieldBus_Int_Time) * 3 - 1);
      s64_extrapolation_compensation = (long long)LVAR(AX0_s32_FieldBus_End_Vel) * s16_extrapolation_i;
      // End compute compensation for extrapolation
      // Add compensation value
      LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1) += s64_extrapolation_compensation;
      LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_2) += s64_extrapolation_compensation;
      LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_3) += s64_extrapolation_compensation;

      // Target position handling / defining position and velocity conditions
      if (VAR(AX0_u16_FieldBus_Int_Mode) == CUBIC_2PV_INTERPOLATION)
      {
         s64_2dp = (LLVAR(AX0_u32_FieldBus_Int_Target_Lo) - LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1)) << 1;
      }
      else if (VAR(AX0_u16_FieldBus_Int_Mode) == CUBIC_4P_INTERPOLATION)
      {
         // v0 = (p1-p3)/2,  v1 = (p-p2)/2 - where p is current position
         LLVAR(AX0_u32_FieldBus_Int_2Vel_Lo) = LLVAR(AX0_u32_FieldBus_Int_Target_Lo) - LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_2);
         s64_2dp = (LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1) - LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_2)) << 1;
         LLVAR(AX0_u32_FieldBus_Int_Target_Lo) = LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1);
      }
      else if (VAR(AX0_u16_FieldBus_Int_Mode) == CUBIC_4P_AVG_INTERPOLATION)
      // To avoid odd profile during velocity change caused due to the cubic interpolation, instead
      // of using the raw position commands, take the average. This will make the profile smoother but
      // it will not necessarily pass via the raw command points
      {
         LLVAR(AX0_u32_FieldBus_Int_2Vel_Lo) = (LLVAR(AX0_u32_FieldBus_Int_Target_Lo) - LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1)) << 1;
         s64_2dp = LLVAR(AX0_u32_FieldBus_Int_Target_Lo) - LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_2);
         LLVAR(AX0_u32_FieldBus_Int_Target_Lo) = (LLVAR(AX0_u32_FieldBus_Int_Target_Lo) + LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1)) >> 1;
      }
      else if (VAR(AX0_u16_FieldBus_Int_Mode) == CUBIC_4P_AVG_INTERPOLATION_NO_DELAY)
      {
         LLVAR(AX0_u32_FieldBus_Int_2Vel_Lo) = (LLVAR(AX0_u32_FieldBus_Int_Target_Lo) - LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1)) << 1;
      }

      s32_FieldBus_End_Vel = ((long)((float)LLVAR(AX0_u32_FieldBus_Int_2Vel_Lo) * LVAR(AX0_f_One_Over_N)) + 1) >> 1;

      if (VAR(AX0_u16_FieldBus_Int_Mode) != CUBIC_4P_AVG_INTERPOLATION_NO_DELAY)
      {
      // End target position handling / defining position and velocity conditions

      // Compute a,b,c,d
      /*-------------------------------
      a = ((V0 + V1) / N^2) - (2 * (P1 - P0) / N^3)
      b = (3 * (P1 - P0) / N^2) - ((V1 + 2*V0) / N)
      c = V0
      d = P1

      Note that:
      V0 = AX0_u32_FieldBus_Int_2Vel_Lo / (2 * N)
      V1 = AX0_u32_FieldBus_Int_2Vel_Lo / (2 * N)
      -------------------------------*/
         if (VAR(AX0_u16_FieldBus_Int_Time) > 1) //avoid unecessary heavy calculation if cycle time == 1
         {
            s64_d = LLVAR(AX0_u32_FieldBus_Int_Current_Target_Lo) + s64_extrapolation_compensation;
            s64_temp1 = LLVAR(AX0_u32_FieldBus_Int_2Vel_Lo);
            s64_temp2 = LLVAR(AX0_u32_FieldBus_Int_Prev_2Vel_Lo);
            f_c = (float)((s64_temp2 + 1) >> 1LL) * LVAR(AX0_f_One_Over_N);
            f_One_Over_N2 = LVAR(AX0_f_One_Over_N) * LVAR(AX0_f_One_Over_N);
            f_b = ((float)((((s64_2dp * 3LL) - s64_temp1) >> 1LL) - s64_temp2)) * f_One_Over_N2;
            f_a = ((float)(((s64_temp1 + s64_temp2) >> 1LL) - s64_2dp)) * f_One_Over_N2 * LVAR(AX0_f_One_Over_N);
         }  // End compute a,b,c,d
      }
      else //CUBIC_4P_AVG_INTERPOLATION_NO_DELAY
      {
         s64_b =(LLVAR(AX0_u32_FieldBus_Int_Target_Lo)+ LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_2)-(LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1)<<1))>>1;
         f_a =((float)((s64_b << 2)- LLVAR(AX0_u32_FieldBus_Int_Target_Lo)+LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1)+LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_2)-LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_3)))/6.0;
         f_c = -f_a + ((float)((LLVAR(AX0_u32_FieldBus_Int_Target_Lo) - LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_2))>>1));
         s64_d = LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1);
         dt_to_dT = (1.0/((float)VAR(AX0_u16_FieldBus_Int_Time)));
         dt_to_dT_square = dt_to_dT * dt_to_dT;
         f_c= f_c * dt_to_dT;
         f_b=((float)s64_b)* dt_to_dT_square;
         f_a= f_a * dt_to_dT * dt_to_dT_square;
      }
      // Test values
      // Check that Vel does not exceed max value in the next sync
      // As P = at3+bt2+ct+d
      //   P' = 3at2+2bt+c
      //   P''= 6at+2b
      // P'ext, is defined when P''=0=6at+2b => t=-b/3a, if this t is in the range then
      // P'ext = c - b2/3a
      // p'extremum = max(|v0|, |v1|, p'ext(if in range)
      f_N = (float)VAR(AX0_u16_FieldBus_Int_Time);
      s64_max = 0LL;
      if (f_a)
      {
         f_Xmax = -f_b / (3.0 * f_a);
         if ((f_Xmax > 0.0) && (f_Xmax < f_N)) // Check if the minimum location is in the next section
         {
            f_max = f_c + f_b * f_Xmax;
            if (f_max < 0.0) f_max = -f_max;
            f_max = f_max * f_N;
            s64_max = (long long) f_max;
         }
      }

      do {
         s16_temp_time = Cntr_3125;
         s64_value_at_end = LLVAR(AX0_u32_FieldBus_Int_2Vel_Lo) >> 1;
      } while (s16_temp_time != Cntr_3125);
      if (s64_value_at_end < 0LL)  s64_value_at_end = -s64_value_at_end;
      if (s64_max < s64_value_at_end)  s64_max = s64_value_at_end;

      // If no error, validate the coefficients
      // Raise the Fb1 fault only if
      //  a) Velocity is too high
      //  b) Drive is in enable operation state
      //  c) Flag "command ignored", which is low active, is set.
      if( (s64_max > LLVAR(AX0_u32_Fieldbus_Vlim_Design_Lo)) && ((p402_statusword & STATUSWORD_STATE) == OPERATION_ENABLED) &&
          (p402_statusword & LOW_ACTIVE_COMMAND_IGNORED_MASK) )
      { // Exceed VLIM - issue a fault
         VAR(AX0_s16_Fieldbus_Target_Postion_Flag) += 1; // Count Excessive Velocity Events...
         LVAR(AX0_u32_Fieldbus_Target_Pos_Err_Cntr) += 1; // Accumulating Counter
      }
      else
      {
         LVAR(AX0_f_FieldBus_Int_a_tmp) = f_a;
         LVAR(AX0_f_FieldBus_Int_b_tmp) = f_b;
         LVAR(AX0_f_FieldBus_Int_c_tmp) = f_c;
         LVAR(AX0_s32_FieldBus_End_Vel_tmp) = s32_FieldBus_End_Vel;
         LLVAR(AX0_u32_FieldBus_Int_d_tmp_Lo) = s64_d;
         LLVAR(AX0_u32_FieldBus_Int_Current_Target_tmp_Lo) = LLVAR(AX0_u32_FieldBus_Int_Target_Lo);
         if (VAR(AX0_u16_FieldBus_Int_Mode) != CUBIC_2PV_INTERPOLATION)
            LLVAR(AX0_u32_FieldBus_Int_Target_Lo) = s64_abs_target_pos;
         VAR(AX0_u16_FieldBus_Int_change_counter) = s16_extrapolation_i + VAR(AX0_u16_FieldBus_Int_Time);
         VAR(AX0_s16_Fieldbus_Target_Postion_Flag) &= 0xFF00; // Zero Excessive Velocity Counter...
      }
   // End if no error, age the positions, validate the coefficients
   }
}


#pragma CODE_SECTION(FieldBusCubicInterpolationRt, "ramfunc_2");
   // Comment : dual axis cannot work! because of drive = 0
void FieldBusCubicInterpolationRt(void)
{
   unsigned int u16_n, u16_cycle_time;
   // AXIS_OFF;

   u16_n = VAR(AX0_u16_FieldBus_Int_n);
   u16_cycle_time = VAR(AX0_u16_FieldBus_Int_Time);
//   u16_4cycle_time = u16_cycle_time << 2;

   if (u16_n > VAR(AX0_u16_FieldBus_Int_change_counter))  // new coefficients?
   {  // Copy coefficients and age variables
      LVAR(AX0_f_FieldBus_Int_a) = LVAR(AX0_f_FieldBus_Int_a_tmp);
      LVAR(AX0_f_FieldBus_Int_b) = LVAR(AX0_f_FieldBus_Int_b_tmp);
      LVAR(AX0_f_FieldBus_Int_c) = LVAR(AX0_f_FieldBus_Int_c_tmp);
      LLVAR(AX0_u32_FieldBus_Int_d_Lo) = LLVAR(AX0_u32_FieldBus_Int_d_tmp_Lo);
      LVAR(AX0_s32_FieldBus_End_Vel) = LVAR(AX0_s32_FieldBus_End_Vel_tmp);
      LLVAR(AX0_u32_FieldBus_Int_Current_Target_Lo) = LLVAR(AX0_u32_FieldBus_Int_Current_Target_tmp_Lo);
      LLVAR(AX0_u32_FieldBus_Int_Prev_2Vel_Lo) = LLVAR(AX0_u32_FieldBus_Int_2Vel_Lo);
      LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_3) = LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_2);
      LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_2) = LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1);

/*    asm(" ;..... comment");
      asm("   PUSH   XAR7");
      asm("   PUSH   XAR2");
      asm("   MOVL   XAR7,#_AX0_f_FieldBus_Int_a_tmp");   //   ; XAR7 = pointer to Array1
      asm("   MOVL   XAR2,#_AX0_f_FieldBus_Int_a");       //   ; XAR2 = pointer to Array2
      asm("   RPT    #23 ||PREAD  *XAR2++,*XAR7");        //   ; Repeat next instruction N times: Array2[i] = Array1[i], i++
      asm("   POP    XAR2");
      asm("   POP    XAR7"); */

      LLVAR(AX0_u32_FieldBus_Int_Prev_Target_Lo_1) = LLVAR(AX0_u32_FieldBus_Int_Target_Lo);

      u16_n = 1;
      VAR(AX0_u16_FieldBus_Int_change_counter) = 65535;
   }
/*
   if (u16_n < u16_4cycle_time)
   { */
      if (u16_n < u16_cycle_time)
      {  // Interpolation
         LLVAR(AX0_u32_FieldBus_Int_Lo) = LLVAR(AX0_u32_FieldBus_Int_d_Lo) + (long long)(((LVAR(AX0_f_FieldBus_Int_a) * u16_n + LVAR(AX0_f_FieldBus_Int_b)) * u16_n + LVAR(AX0_f_FieldBus_Int_c)) * u16_n);
      }
      else if (u16_n == u16_cycle_time)
      {  // Target position
         LLVAR(AX0_u32_FieldBus_Int_Lo) = LLVAR(AX0_u32_FieldBus_Int_Current_Target_Lo);
      }
      else
      {  // Extrapolation
         // Move this to 1Khz
         LLVAR(AX0_u32_FieldBus_Int_Lo) += LVAR(AX0_s32_FieldBus_End_Vel);
      }
      VAR(AX0_u16_FieldBus_Int_n) = ++u16_n; /*
//      if (u16_n == u16_4cycle_time)
//         VAR(AX0_u16_FieldBus_Missing_Cmd_Ctr) = 4;
   } */

   VAR(AX0_s16_FieldBus_Int_Index) -= 1; // Decrement Index,
   if (VAR(AX0_s16_FieldBus_Int_Index) == 0) //  reaching Zero means a Missed FieldBus Position Command
   {
      VAR(AX0_u16_FieldBus_Missing_Cmd_Ctr) += 1; // Increment Missed-Command Counter, restart Index
      VAR(AX0_s16_FieldBus_Int_Index) += (int)VAR(AX0_u16_FieldBus_Int_Time); // for another cycle...
   }
}


int SerialCommunicationEventDecReadCommand(long long *data, int drive)
{
   *data = (long long)convertAccDecInternalUnitsForPosToMs(drive, (long long)BGVAR(u64_Ser_Comm_Time_Out_Dec_Rate));
   return SAL_SUCCESS;
}


int SerialCommunicationEventDecCommand(int drive)
{
   BGVAR(u64_Ser_Comm_Time_Out_Dec_Rate) = convertMsToAccDecInternalUnitsForPos(drive, (unsigned int)s64_Execution_Parameter[0]);
   return SAL_SUCCESS;
}


int MotorStopEventDecReadCommand(long long *data,int drive)
{
   *data = (long long)convertAccDecInternalUnitsForPosToMs(drive, (long long)BGVAR(u64_Motor_Stop_Dec_Rate));
   return SAL_SUCCESS;
}


int MotorStopEventDecCommand(int drive)
{
   BGVAR(u64_Motor_Stop_Dec_Rate) = convertMsToAccDecInternalUnitsForPos(drive , (unsigned int)s64_Execution_Parameter[0]);

   return (SAL_SUCCESS);
}


int PositionCommandOverflowEventDecReadCommand(long long *data, int drive)
{
   *data = (long long)convertAccDecInternalUnitsForPosToMs(drive, (long long)BGVAR(u64_Pos_Comand_Overflow_Dec_Rate));
   return SAL_SUCCESS;
}


int PositionCommandOverflowEventDecCommand(int drive)
{
   BGVAR(u64_Pos_Comand_Overflow_Dec_Rate) = convertMsToAccDecInternalUnitsForPos(drive, (unsigned int)s64_Execution_Parameter[0]);
   return SAL_SUCCESS;
}


int NegativeSoftLimitSwitchEventDecReadCommand(long long *data, int drive)
{
   *data = (long long)convertAccDecInternalUnitsForPosToMs(drive, (long long)BGVAR(u64_N_SW_Limit_Dec_Rate));
   return SAL_SUCCESS;
}


int NegativeSoftLimitSwitchEventDecCommand(int drive)
{
   BGVAR(u64_N_SW_Limit_Dec_Rate) = convertMsToAccDecInternalUnitsForPos(drive, (unsigned int)s64_Execution_Parameter[0]);
   return SAL_SUCCESS;
}


int NegativeHardLimitSwitchEventDecReadCommand(long long *data, int drive)
{
   *data = (long long)convertAccDecInternalUnitsForPosToMs(drive, (long long)BGVAR(u64_N_HW_Limit_Dec_Rate));
   return (SAL_SUCCESS);
}


int NegativeHardLimitSwitchEventDecCommand(int drive)
{
   // position deceleration
   BGVAR(u64_N_HW_Limit_Dec_Rate) = convertMsToAccDecInternalUnitsForPos(drive , (unsigned int)s64_Execution_Parameter[0]);
   return (SAL_SUCCESS);
}

int PositiveSoftLimitSwitchEventDecReadCommand(long long *data,int drive)
{
   *data = (long long)convertAccDecInternalUnitsForPosToMs(drive, (long long)BGVAR(u64_P_SW_Limit_Dec_Rate));
   return SAL_SUCCESS;
}


int PositiveSoftLimitSwitchEventDecCommand(int drive)
{
   BGVAR(u64_P_SW_Limit_Dec_Rate) = convertMsToAccDecInternalUnitsForPos(drive, (unsigned int)s64_Execution_Parameter[0]);
   return SAL_SUCCESS;
}


int PositiveHardLimitSwitchEventDecReadCommand(long long *data, int drive)
{
   *data = (long long)convertAccDecInternalUnitsForPosToMs(drive, (long long)BGVAR(u64_P_HW_Limit_Dec_Rate));
   return SAL_SUCCESS;
}


int PositiveHardLimitSwitchEventDecCommand(int drive)
{
   BGVAR(u64_P_HW_Limit_Dec_Rate) = convertMsToAccDecInternalUnitsForPos(drive, (unsigned int)s64_Execution_Parameter[0]);
   return SAL_SUCCESS;
}



int MoveSmoothModeCommand(long long param, int drive)
{
   // AXIS_OFF;
   VAR(AX0_u16_Move_Smooth_Mode) = (int)param;
   PtpFilterDesign(drive);
   SetGearingFFPointers(drive);
   return (SAL_SUCCESS);
}



int MoveSmoothSourceCommand(long long param, int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   VAR(AX0_u16_Move_Smooth_Source) = (int)param;
   PtpFilterDesign(drive);
   SetGearingFFPointers(drive);
   return (SAL_SUCCESS);
}


void SetGearingFFPointers(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   // If smoothing is used the ACC output from the MSQ is not valid hence take the 2nd derivative of position instead
   if (VAR(AX0_s16_Opmode) == 4)
   {
      if ( VAR(AX0_u16_Qep_Msq_Fltr_Mode) &&
          ((VAR(AX0_u16_Move_Smooth_Mode)==0) || ((VAR(AX0_u16_Move_Smooth_Mode)==1) && (BGVAR(s16_Move_Smooth_Lpf_Hz)==5000)) || ((VAR(AX0_u16_Move_Smooth_Source) & 0x02)==0)))
      {
         VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr) = (int)((long)&LVAR(AX0_s32_Qep_Msq_Fltr_Acc_Out) & 0xffff);
         VAR(AX0_s16_KPAFR_Source_Ptr) = (int)((long)&LVAR(AX0_s32_Gear_Acc_Out) & 0xffff);
         VAR(AX0_s16_KPVFR_Source_Ptr) = (int)((long)&LVAR(AX0_s32_Gear_Vel_Out) & 0xffff);
      }
      else
      {
         VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr) = (int)((long)&LVAR(AX0_s32_Delta_Delta_Pos_Cmd) & 0xffff);
         VAR(AX0_s16_KPAFR_Source_Ptr) = (int)((long)&LVAR(AX0_s32_Delta_Delta_Pos_Cmd) & 0xffff);
         VAR(AX0_s16_KPVFR_Source_Ptr) = (int)((long)&LVAR(AX0_s32_Delta_Pos_Cmd) & 0xffff);
      }
      VAR(AX0_s16_Hold_Pos_Acccmnd_Ptr) = VAR(AX0_s16_Pos_Loop_Acccmnd_Ptr);
   }

   return;
}


//**********************************************************
// Function Name: SalWriteJogStepCommand
// Description:
//    This function handles p param P4-82 (jog step)
//    alexander badura request to have canopen units. p4-82 will not exist, only he canopen objct at 0x4452
//
// Author: Nitsan
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteJogStepCommand(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if ((unsigned long)param > 0x7fffffff) return VALUE_TOO_HIGH;

   BGVAR(u32_P4_82_Jog_Step) = (unsigned long)param;

   //convert from PUU into CDHD internal position units and set it in the POSLIMPOS variable.
   BGVAR(s64_Lexium_Jog_Step) = MultS64ByFixS64ToS64(param,
                      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix,
                      BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr);

   return SAL_SUCCESS;
}

//**********************************************************
// Function Name: LexiumJogMoveHandler
// Description:
//    This function handles the special Lexium 32 JOG move feature. The function
//    is called in background. A jog-move needs to be handled in OPMODE 8 and not
//    OPMODE 0 since in jog-mode 1 an incremental move is supposed to be triggered.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void LexiumJogMoveHandler (int drive)
{
   // AXIS_OFF;
   static int s16_mode1_new_command = 0;
   unsigned int u16_temp_time;
   long long s64_pcmd;
   long s32_vcmd = 0;
   int retVal, s16_paramInvalid;
   unsigned long u32_slow_speed;
   unsigned int u16_retrigger_motion = 0; // Local variable that indicates that motion is supposed to be re-triggered

   // IPR 1350: Parameter P1-42 (ON Delay Time of Holding Brake) has no effect.
   // delay any motion befroe brake on time (p1-42) has elapsed
   if (BGVAR(u16_Allow_Motion) == 0)
   {
      return;
   }
   
   // The following code is used in order to apply a new speed immediately during a jog move (BugFix for Bugzilla 3907).
   if(BGVAR(s32_P4_05_Previous_Jog_SpeedSlow_Out_Loop) != BGVAR(s32_P4_05_Jog_SpeedSlow_Out_Loop))
   {
      u16_retrigger_motion = 1;
      BGVAR(s32_P4_05_Previous_Jog_SpeedSlow_Out_Loop) = BGVAR(s32_P4_05_Jog_SpeedSlow_Out_Loop);
   }

   // A jog will only be handled if
   //   a) CDHD OPMODE = 8
   //   b) DS402 OPMODE = -1 (needs to be temporarily set by local HMI?)
   //   c) Drive is enabled
   // or jog will also be 1-time handled if someone (local HMI) initiated a 1-time init-stop
   if ( (((p402_modes_of_operation_display != -1) || (VAR(AX0_s16_Opmode) != 8) || (!Enabled(drive))) && (BGVAR(u16_Lexium_Jog_Move_Init_Stop) == 0)) ||
         (BGVAR(u16_Internal_Profile_State) != INTERNAL_PROFILE_IDLE) )  // fix bugzilla 4169: Can not move in Internal Profile mode after the motor moved in Jog mode before
   {
       // Reset the internal jog-move variables
       BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_IDLE;

       // nitsan: reset prev value to allow motion on transition from disable to enable
       // (e.g. the sqeuence: set jog forward command -> disable drive -> enable drive; should cause the motor to move)
       // zero both variables to allow the jog to start from SoMove via P10-25
       BGVAR(u16_Lexium_Prev_Jog_Move_Activate) = BGVAR(u16_Lexium_Jog_Move_Activate) = 0;

       // cancel hold if not in jog mode
       if (BGVAR(u16_Hold_Bits) & HOLD_JOG_MODE_STOP_MASK)
       {
           BGVAR(u16_Hold_Bits) &= ~HOLD_JOG_MODE_STOP_MASK;
           HoldScan(drive);
       }

       return;
   }

   // If local HMI initiated a 1-time stop
   if (BGVAR(u16_Lexium_Jog_Move_Init_Stop) != 0)
   {
      // Clear variable that is set to 1 when stepping out of local HMI mode Jog Move
      BGVAR(u16_Lexium_Jog_Move_Init_Stop) = 0;
      // Set the jog move activation variables to 0 in order to not accidentally jump to next state
      BGVAR(u16_Lexium_Jog_Move_Activate) = BGVAR(u16_Lexium_Prev_Jog_Move_Activate) = 0;
      // Go to idle state in order to stop motion
      BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_IDLE;
   }

   // set slow speed according to opmode (p4-05 on IO drive, object 0x4454 on CANopen drive)
   // fix bug IPR: 558
   // take speed from P4-84 only if in CANopen moed and has access rights
   // (to allow DTM to use speed from P4-05 even if P1-01==0xb (i.e. in canopen mode) )
   if ( ((BGVAR(u16_P1_01_CTL_Current) & 0xFF) == SE_OPMODE_CANOPEN) &&
      (CheckLexiumAccessRights(drive, LEX_AR_FCT_ACTIVATE_JOG_MODE, LEX_CH_FIELDBUS) == SAL_SUCCESS) )
   {
      u32_slow_speed = BGVAR(s32_P4_84_Jog_SpeedSlow_Out_Loop);
   }
   else
   {
      u32_slow_speed = BGVAR(s32_P4_05_Jog_SpeedSlow_Out_Loop);
   }

   switch (BGVAR(u16_Lexium_Jog_Move_State))
   {
     case(LEX_JOG_MOVE_IDLE):

      // Set DEC as the deceleration for Jog
      BGVAR(u16_Deceleration_Event_ID) = EVENT_DEC;

      // fix IPR 966: The execution of Autotuning can not be started if an AL539 (Motor phase missing) occurred previously.
      // here opmode is 8 (position mode), so ptp generator is active
      // use hold for stop, if motor is moving (ptp generator has not finished)
      if (VAR(AX0_u16_Motor_Stopped) == 0)
      {
         BGVAR(u16_Hold_Bits) |= HOLD_JOG_MODE_STOP_MASK;
         if (BGVAR(u16_Hold) == 0)
            HoldScan(drive);
      }
      else
      {
         BGVAR(u16_Hold_Bits) &= ~HOLD_JOG_MODE_STOP_MASK;
         if (BGVAR(u16_Hold))
            HoldScan(drive);
      }

       // If the positive/negative bits were set in state idle
       if(BGVAR(u16_Lexium_Jog_Move_Activate) & 0x0003)
       {
          if(BGVAR(u16_P4_83_Jog_Method) == 0)
          {
             BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_MODE_0;
          }
          else
          {
             BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_MODE_1_PTP_INIT;
          }
       }
     break;

     case(LEX_JOG_MOVE_MODE_0):
       // Take the current position command as the starting point. Try to make it thread-safe.
       do
       {
          u16_temp_time = Cntr_3125;
          s64_pcmd = ((long long)LVAR(AX0_s32_Pos_Cmd_User_Hi) << 32) | ((long long)LVAR(AX0_u32_Pos_Cmd_User_Lo) & 0x00000000FFFFFFFFLL);
       }
       while (u16_temp_time != Cntr_3125);

       // if jog control param changed, or we are in ptp,delay,jog mode (then we get here after ptp and delay finished and control param has not changed)
       if (BGVAR(u16_Lexium_Prev_Jog_Move_Activate) != BGVAR(u16_Lexium_Jog_Move_Activate) ||
          (s16_mode1_new_command == 1) || (u16_retrigger_motion == 1))
       {
            s16_mode1_new_command = 0;

            switch(BGVAR(u16_Lexium_Jog_Move_Activate) & 0x0007)
            {
               case (0x0001): // Positive direction with slow speed
                  s32_vcmd = u32_slow_speed;
                  s64_pcmd = s64_pcmd + 0x0100000000000000LL;
               break;

               case (0x0002): // Negative direction with slow speed
                  s32_vcmd = u32_slow_speed;
                  s64_pcmd = s64_pcmd - 0x0100000000000000LL;
               break;

               case (0x0005): // Positive direction with high speed
                  s32_vcmd = BGVAR(s32_P4_80_Jog_SpeedFast_Out_Loop);
                  s64_pcmd = s64_pcmd + 0x0100000000000000LL;
               break;

               case (0x0006): // Negative direction with high speed
                  s32_vcmd = BGVAR(s32_P4_80_Jog_SpeedFast_Out_Loop);
                  s64_pcmd = s64_pcmd - 0x0100000000000000LL;
               break;

               default:
                  BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_IDLE;
                  BGVAR(u16_Lexium_Prev_Jog_Move_Activate) = BGVAR(u16_Lexium_Jog_Move_Activate);
               break;
            }

            if (BGVAR(u16_Lexium_Jog_Move_State) != LEX_JOG_MOVE_IDLE)
            {
               if (BGVAR(u16_Hold_Bits) & HOLD_JOG_MODE_STOP_MASK)
               {
                  BGVAR(u16_Hold_Bits) &= ~HOLD_JOG_MODE_STOP_MASK;
                  HoldScan(drive);
               }

               // Fix IPR#1330: If Velocity Limitation P1:55 is lower than Jog Velocity P4:05, the axis cannot be moved by Jog command
               // Saturate velocity command if above VLIM.
               if (BGVAR(s32_V_Lim_Design) < s32_vcmd)
               {
                  s32_vcmd = BGVAR(s32_V_Lim_Design);
               }

               // motion must be absolute position
               BGVAR(s16_Move_Abs_Issued) = 1;
               // allow correct behavior of software limits. fix bug 3541
               BGVAR(s16_Sal_Move_Ind) = 1;
               retVal = ExecuteMoveCommandInternal(drive, s64_pcmd, s32_vcmd, (long long)PTP_IMMEDIATE_TRANSITION, &s16_paramInvalid);
               if (retVal == SAL_SUCCESS)
               {
                  BGVAR(u16_Lexium_Prev_Jog_Move_Activate) = BGVAR(u16_Lexium_Jog_Move_Activate);
               }
               else
               {
                  // support for lexium profile drive
                  BGVAR(u16_Lxm_ModeError_Can_Obj) = retVal;
                  BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = s16_paramInvalid+1;
                  u16_Lxm_MfStat.bit.ME = 1;
                  BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_IDLE;

                  // allow trying to send move comamnd if inside limit switch.
                  if (retVal == COMMAND_TOWARDS_LIMIT)
                  {
                     BGVAR(u16_Lexium_Prev_Jog_Move_Activate) = 0;
                  }
                  else
                  {
                     BGVAR(u16_Lexium_Prev_Jog_Move_Activate) = BGVAR(u16_Lexium_Jog_Move_Activate);
                  }
               }
            }
       }

      //BGVAR(u16_Lexium_Prev_Jog_Move_Activate) = BGVAR(u16_Lexium_Jog_Move_Activate);
      break;

     case (LEX_JOG_MOVE_MODE_1_PTP_INIT):
       // Take the current position command as the starting point. Try to make it thread-safe.
       do
       {
         u16_temp_time = Cntr_3125;
         s64_pcmd = ((long long)LVAR(AX0_s32_Pos_Cmd_User_Hi) << 32) | ((long long)LVAR(AX0_u32_Pos_Cmd_User_Lo) & 0x00000000FFFFFFFFLL);
       }
       while (u16_temp_time != Cntr_3125);

       switch(BGVAR(u16_Lexium_Jog_Move_Activate) & 0x0007)
       {
          case (0x0001): // Positive direction with slow speed
             s32_vcmd = u32_slow_speed;
             s64_pcmd = s64_pcmd + BGVAR(s64_Lexium_Jog_Step);
             BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_MODE_1_PTP_INIT_FINISHED;
          break;

          case (0x0002): // Negative direction with slow speed
             s32_vcmd = u32_slow_speed;
             s64_pcmd = s64_pcmd - BGVAR(s64_Lexium_Jog_Step);
             BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_MODE_1_PTP_INIT_FINISHED;
          break;

          case (0x0005): // Positive direction with high speed
             s32_vcmd = BGVAR(s32_P4_80_Jog_SpeedFast_Out_Loop);
             s64_pcmd = s64_pcmd + BGVAR(s64_Lexium_Jog_Step);
             BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_MODE_1_PTP_INIT_FINISHED;
          break;

          case (0x0006): // Negative direction with high speed
             s32_vcmd = BGVAR(s32_P4_80_Jog_SpeedFast_Out_Loop);
             s64_pcmd = s64_pcmd - BGVAR(s64_Lexium_Jog_Step);
             BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_MODE_1_PTP_INIT_FINISHED;
          break;

          default:
             BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_IDLE;
          break;
       }

       if (BGVAR(u16_Lexium_Jog_Move_State) != LEX_JOG_MOVE_IDLE)
       {
         if (BGVAR(u16_Hold_Bits) & HOLD_JOG_MODE_STOP_MASK)
         {
            BGVAR(u16_Hold_Bits) &= ~HOLD_JOG_MODE_STOP_MASK;
            HoldScan(drive);
         }

         // Fix IPR#1330: If Velocity Limitation P1:55 is lower than Jog Velocity P4:05, the axis cannot be moved by Jog command
         // Saturate velocity command if above VLIM.
         if (BGVAR(s32_V_Lim_Design) < s32_vcmd)
         {
            s32_vcmd = BGVAR(s32_V_Lim_Design);
         }

         // motion must be absolute position
         BGVAR(s16_Move_Abs_Issued) = 1;
         // allow correct behavior of software limits. fix bug 3541
         BGVAR(s16_Sal_Move_Ind) = 1;
         retVal = ExecuteMoveCommandInternal(drive, s64_pcmd, s32_vcmd, (long long)PTP_IMMEDIATE_TRANSITION, &s16_paramInvalid);
         if (retVal == SAL_SUCCESS)
         {
            BGVAR(u16_Lexium_Prev_Jog_Move_Activate) = BGVAR(u16_Lexium_Jog_Move_Activate);
         }
         else
         {
            // support for lexium profile drive
            BGVAR(u16_Lxm_ModeError_Can_Obj) = retVal;
            BGVAR(u16_Lxm_ModeErrorInfo_Can_Obj) = s16_paramInvalid+1;
            u16_Lxm_MfStat.bit.ME = 1;
            BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_IDLE;

            // allow trying to send move comamnd if inside limit switch.
            if (retVal == COMMAND_TOWARDS_LIMIT)
            {
               BGVAR(u16_Lexium_Prev_Jog_Move_Activate) = 0;
            }
            else
            {
               BGVAR(u16_Lexium_Prev_Jog_Move_Activate) = BGVAR(u16_Lexium_Jog_Move_Activate);
            }
         }
      }

      break;

     case(LEX_JOG_MOVE_MODE_1_PTP_INIT_FINISHED):
       // If the PTP hunter reached the target position
       if(LLVAR(AX0_u32_Pos_Cmd_Ptp_Lo) == LLVAR(AX0_u32_Target_Position_Lo))
       {
          // Capture the current 1ms timer
          BGVAR(s32_Lexium_Jog_Move_Wait_Time_Capture) = Cntr_1mS;
          // Wait the selected jog-time
          BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_MODE_1_PTP_FINISHED_WAIT;
       }
       else if((BGVAR(u16_Lexium_Jog_Move_Activate) & 0x0003) != (BGVAR(u16_Lexium_Prev_Jog_Move_Activate) & 0x0003))
       {
          // The status of the lower 2 bits changed, so do not continue with the regular jog
          BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_MODE_1_PTP_FINISHED_STOP;
       }
     break;

     case(LEX_JOG_MOVE_MODE_1_PTP_FINISHED_WAIT):
       if (PassedTimeMS((long)BGVAR(u32_P4_81_Jog_Time), BGVAR(s32_Lexium_Jog_Move_Wait_Time_Capture)))
       {
          // signal that new command of mode 1 arrives
          s16_mode1_new_command = 1;
          BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_MODE_0;
       }
       else if((BGVAR(u16_Lexium_Jog_Move_Activate) & 0x0003) != (BGVAR(u16_Lexium_Prev_Jog_Move_Activate) & 0x0003))
       {
          // The status of the lower 2 bits changed --> finish motion
          BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_IDLE;
       }
     break;

     case(LEX_JOG_MOVE_MODE_1_PTP_FINISHED_STOP):
       // If the PTP hunter reached the target position
       if(LLVAR(AX0_u32_Pos_Cmd_Ptp_Lo) == LLVAR(AX0_u32_Target_Position_Lo))
       {
          BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_IDLE;
       }
       // Allow the user to leave this state in case that a HOLD process prevents
       // the Drive from reaching the target position.
       else if ((BGVAR(u16_Lexium_Jog_Move_Activate) != BGVAR(u16_Lexium_Prev_Jog_Move_Activate)) &&
                (VAR(AX0_u16_HP_Flags) & HOLD_IN_PROCESS_MASK))
       {
          BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_IDLE;
       }
     break;


     default: // Unknown state (not suppose to happen
       BGVAR(u16_Lexium_Jog_Move_State) = LEX_JOG_MOVE_IDLE;
     break;
   }
}


//**********************************************************
// Function Name: SalSFBNumberOfSectors
// Description:
//    This function Sets a variable, which defines the number of sectors we will
//    split the whole voltage range of the potentiometer during the calibration
//    process.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalSFBNumberOfSectors(long long lparam,int drive)
{
   BGVAR(u16_Voltage_Correct_Number_Of_Sections) = (int)lparam;

   // Sanity check since when writing 0 or 1 the correction code gets disabled.
   SFBVoltageCorrectionSanityCheck(drive, 1);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalSFBNumberOfSectors
// Description:
//    This function Sets a variable, which defines the number of sectors we will
//    split the whole voltage range of the potentiometer during the "manual" calibration
//    process for analog input 2.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalSFBNumberOfSectors2(long long lparam,int drive)
{
   BGVAR(u16_Voltage_Correct_2_Number_Of_Sections) = (int)lparam;

   // Sanity check since when writing 0 or 1 the correction code gets disabled.
   SFBVoltageCorrectionSanityCheck(drive, 2);

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalManualCalibrationProcess
// Description:
//    This function handles the manual calibration process for Frencken. Please
//    note also the STX specification on the Servotronix Server.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalManualCalibrationProcess(int drive)
{
   // AXIS_OFF;

   static unsigned int u16_state = 0;

   if(BGVAR(u16_Voltage_Correct_State) != VOLTAGE_CORRECT_STATE_IDLE)
   {
      // Return "Motor in motion" in case that voltage correction feature is running
      return (MOTOR_IN_MOTION);
   }
   else if ((s64_Execution_Parameter[0] >= 0) && (s64_Execution_Parameter[0] < VOLTAGE_CORRECT_ARRAY_SIZE)) // Write voltage to the array
   {
      if(u16_state == 0)
      {
         BGVAR(s64_Voltage_Correct_General_Purpose_01) = 0;
         BGVAR(s64_Voltage_Correct_General_Purpose_02) = 0;
         u16_state = 1;
         return (SAL_NOT_FINISHED);
      }
      else
      {
         BGVAR(s64_Voltage_Correct_General_Purpose_01)++;
         BGVAR(s64_Voltage_Correct_General_Purpose_02) += (long long)VAR(AX0_s16_An_In_2_Filtered);  // Measure and increment voltage at analog input 2
         // After 8 measurements
         if(BGVAR(s64_Voltage_Correct_General_Purpose_01) >= 8)
         {
            // Write the averaged voltage
            BGVAR(s16_Voltage_Correct_2_Array)[s64_Execution_Parameter[0]] = (int)(BGVAR(s64_Voltage_Correct_General_Purpose_02) >> 3);
            u16_state = 0; // Reset state
            return (SAL_SUCCESS);
         }
         else
         {
            return (SAL_NOT_FINISHED);
         }
      }
   }
   else if(s64_Execution_Parameter[0] == 0xFFFF) // Save data in special SINPARAM section
   {
      // Use the variable to indicate a save-error -> Can be queried by a developer
      BGVAR(u16_Voltage_Correct_NvSave_Status) = SaveSineZeroParams(drive);
      // If the NV save is done
      if(BGVAR(u16_Voltage_Correct_NvSave_Status) == SAL_SUCCESS)
      {
         // Check analog input 2 voltage correction array and enable the feature
         // if everything is OK.
         SFBVoltageCorrectionSanityCheck(drive, 2);
      }
      return(BGVAR(u16_Voltage_Correct_NvSave_Status));
   }
   else
   {
      return (VALUE_OUT_OF_RANGE);
   }

   //return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalSFBStartCalibrationProcess
// Description:
//    This function starts the Frencken calibration procedure.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalSFBStartCalibrationProcess(int drive)
{
   // AXIS_OFF;
   if(VAR(AX0_s16_Opmode) != 8)
   {
      return (INVALID_OPMODE);
   }
   else if (!Enabled(drive))
   {
      return (DRIVE_INACTIVE);
   }
   else if(BGVAR(s16_SFBMode) != 0)
   {
      return (SFBMODE_USED);
   }
   else if (ProcedureRunning(DRIVE_PARAM) != PROC_NONE)
   {
      return (OTHER_PROCEDURE_RUNNING);
   }
   else if (VAR(AX0_s16_Ptp_Transition_Mode_Bg) != PTP_IDLE_TRANSITION)
   {
      return(MOTION_PENDING);
   }
   else if (BGVAR(u16_Voltage_Correct_Number_Of_Sections) < 2)
   {
      return(NOT_AVAILABLE);
   }
   else
   {
      BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_PREPARE_PROCEDURE;
   }
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SFBVoltageCorrectionCalibrationProcess
// Description:
//    This function is used in order to run an autonomous process in OPMODE = 8.
//    The function is supposed to measure the analog voltage at specific points
//    and is introduced due to a request from the customer Frencken. Please refer
//    to the detailed documentation about the voltage correction feature for the
//    secondary feedback on the STX server.
//
//    The calibration process must run in SFBMODE = 0, so no dual loop.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void SFBVoltageCorrectionCalibrationProcess (int drive)
{
   // AXIS_OFF;
   int s16_paramInvalid, u16_temp_cw_ls, u16_temp_ccw_ls;
   unsigned int u16_temp_time; // For reading 64-bit variables thread-safe
   long long s64_temp;         // For reading 64-bit variables thread-safe

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


   // Check for an unexpected condition like disable while the process is running. Do not interrupt the process at the very last steps,
   // since especially saving the voltage table on the NV memory needs to be successfully completed and this action needs several function calls!
   if((BGVAR(u16_Voltage_Correct_State) > VOLTAGE_CORRECT_PREPARE_PROCEDURE) && (BGVAR(u16_Voltage_Correct_State) < VOLTAGE_CORRECT_SAVE_DATA))
   {
      // Abort process if:
      //  - Opmode changed
      //  - Drive gets disabled
      //  - At least one digital input is assigned to act as a "allow calibration process" input and if this input is low level
      if((VAR(AX0_s16_Opmode) != 8) || (!Enabled(drive)) || (BGVAR(u16_Voltage_Correct_Consider_Input) && !VAR(AX0_u16_Mode_Input_Arr[ALLOW_SFB_CALIBRATION])))
      {
         // Stop motion that might be pending
         BGVAR(s16_Move_Abs_Issued) = 0;
         InitiatePtpMove(drive, 0x0LL, 0x0L, BGVAR(s32_Ptp_Acc_Rounded), BGVAR(s32_Ptp_Dec_Rounded), PTP_IMMEDIATE_TRANSITION);

         BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_FINISH_PROCEDURE;

         // Discard all intermediate results
         for (BGVAR(s64_Voltage_Correct_General_Purpose_01) = 0LL; BGVAR(s64_Voltage_Correct_General_Purpose_01) < VOLTAGE_CORRECT_ARRAY_SIZE; BGVAR(s64_Voltage_Correct_General_Purpose_01)++)
         {
            BGVAR(s16_Voltage_Correct_Array)[BGVAR(s64_Voltage_Correct_General_Purpose_01)] = 0;
         }
      }
   }

   switch(BGVAR(u16_Voltage_Correct_State))
   {
      case (VOLTAGE_CORRECT_STATE_IDLE):
         // Do nothing
      break;

      case(VOLTAGE_CORRECT_PREPARE_PROCEDURE):
         // If the user wants to apply a specific current limit
         if(BGVAR(s32_Voltage_Correction_Current_Limit) > 0L)
         {
            // Save previous user current limit setting
            BGVAR(s32_Voltage_Correction_Prev_Curr_Limit) = BGVAR(s32_Ilim_User);
            // Change user current limit setting
            SalIlimCommand((long long)BGVAR(s32_Voltage_Correction_Current_Limit), drive);
         }

         // Save the previous user following-error limit
         BGVAR(u64_Voltage_Correction_Prev_Pe_Max) = BGVAR(u64_Pe_Max);
         // Change user following-error limit setting to 0 since we move against a hard stop
         BGVAR(u64_Pe_Max) = 0LL;

         // Here figure out if any input has been selected as an "allow calibration process" input
         BGVAR(u16_Voltage_Correct_Consider_Input) = 0;
         if(IsInFunctionalityConfigured(ALLOW_SFB_CALIBRATION, drive))
         {
            BGVAR(u16_Voltage_Correct_Consider_Input) = 1;
         }

         // Clear NV save status variable
         BGVAR(u16_Voltage_Correct_NvSave_Status) = 0;

         // Jump to next state
         BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_INIT_POS_MOVE;
      break;

      case (VOLTAGE_CORRECT_INIT_POS_MOVE):

         // If we are getting closer to the hard stop
         if((VAR(AX0_s16_An_In_1_Filtered) >= BGVAR(s16_Slow_Movement_Upper_Voltage)) ||
            (VAR(AX0_s16_An_In_1_Filtered) <= BGVAR(s16_Slow_Movement_Lower_Voltage)))
         {
            // motion must be absolute
            BGVAR(s16_Move_Abs_Issued) = 1;
            // allow correct behavior of limit switches.
            BGVAR(s16_Sal_Move_Ind) = 1;
            ExecuteMoveCommandInternal(drive, 0x3FFFFFFFFFFFFFFFLL, BGVAR(s32_Voltage_Correct_Slow_Speed), (long long)PTP_IMMEDIATE_TRANSITION, &s16_paramInvalid);
            BGVAR(s64_Voltage_Correct_General_Purpose_02) = 0LL; // Indicate slow motion triggered
         }
         else
         {
            // motion must be absolute
            BGVAR(s16_Move_Abs_Issued) = 1;
            // allow correct behavior of limit switches.
            BGVAR(s16_Sal_Move_Ind) = 1;
            ExecuteMoveCommandInternal(drive, 0x3FFFFFFFFFFFFFFFLL,  BGVAR(s32_Voltage_Correct_Fast_Speed), (long long)PTP_IMMEDIATE_TRANSITION, &s16_paramInvalid);
            BGVAR(s64_Voltage_Correct_General_Purpose_02) = 1LL; // Indicate fast motion triggered
         }

         // Jump to next state
         BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_WAIT_FOR_POS_STOP;
      break;

      case (VOLTAGE_CORRECT_WAIT_FOR_POS_STOP):
         do
         {
            u16_temp_time = Cntr_3125; // the same RT Interrupt
            s64_temp = LLVAR(AX0_u32_Int_Loop_Pos_Err_Lo);
         }while (u16_temp_time != Cntr_3125);

         if(u16_temp_cw_ls || (s64_temp > 0x200000000LL))
         {
            // Stop motion
            BGVAR(s16_Move_Abs_Issued) = 0;
            InitiatePtpMove(drive, 0x0LL, 0x0L, BGVAR(s32_Ptp_Acc_Rounded), BGVAR(s32_Ptp_Dec_Rounded), PTP_IMMEDIATE_TRANSITION);
            // Capture the 1ms timer
            BGVAR(s64_Voltage_Correct_General_Purpose_01) = (long long)Cntr_1mS;
            // If a backlash move is supposed tp be triggered
            if(BGVAR(s64_Voltage_Correct_Backlash_Position) > 0LL)
            {
               // Jump to next state
               BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_INIT_NEG_BL_MOVE;
            }
            else
            {
               // Jump to next state
               BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_WAIT_MEASURE_UPOS;
            }
         }
         else if(BGVAR(s16_Slow_Movement_Upper_Voltage) != BGVAR(s16_Slow_Movement_Lower_Voltage)) // Check if velocity must be changed
         {
            // If we are getting closer to the hard stop
            if((VAR(AX0_s16_An_In_1_Filtered) >= BGVAR(s16_Slow_Movement_Upper_Voltage)) ||
               (VAR(AX0_s16_An_In_1_Filtered) <= BGVAR(s16_Slow_Movement_Lower_Voltage)))
            {
                if(BGVAR(s64_Voltage_Correct_General_Purpose_02)) // If fast motion was previously triggered
                {
                  // Jump to previous state
                  BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_INIT_POS_MOVE;
                }
            }
            else
            {
               if(!BGVAR(s64_Voltage_Correct_General_Purpose_02)) // If slow motion was previously triggered
               {
                  // Jump to previous state
                  BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_INIT_POS_MOVE;
               }
            }
         }
      break;

      case (VOLTAGE_CORRECT_INIT_NEG_BL_MOVE):
         if(LVAR(AX0_s32_Pos_Vcmd) == 0L) // Wait the PTP generator to come to a stop
         {
            do
            {
               u16_temp_time = Cntr_3125; // the same RT Interrupt
               s64_temp = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
            }while (u16_temp_time != Cntr_3125);

            // Calculate target position of the backlash move --> 2[rev] away from actual position
            BGVAR(s64_Voltage_Correct_General_Purpose_01) = s64_temp - BGVAR(s64_Voltage_Correct_Backlash_Position);
            // motion must be absolute position
            BGVAR(s16_Move_Abs_Issued) = 1;
            // allow correct behavior of software limits.1
            BGVAR(s16_Sal_Move_Ind) = 1;
            if(ExecuteMoveCommandInternal(drive, BGVAR(s64_Voltage_Correct_General_Purpose_01), BGVAR(s32_Voltage_Correct_Slow_Speed), (long long)PTP_IMMEDIATE_TRANSITION, &s16_paramInvalid) == SAL_SUCCESS)
            {
               // Jump to next state
               BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_WAIT_FOR_NEG_BL_END;
            }
         }
      break;

      case (VOLTAGE_CORRECT_WAIT_FOR_NEG_BL_END):
         do
         {
            u16_temp_time = Cntr_3125; // the same RT Interrupt
            s64_temp = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
         }while (u16_temp_time != Cntr_3125);

         if(BGVAR(s64_Voltage_Correct_General_Purpose_01) == s64_temp) // If PCMD reaches the target position of the backlash move
         {
            // Capture the 1ms timer
            BGVAR(s64_Voltage_Correct_General_Purpose_01) = (long long)Cntr_1mS;
            // Jump to next state
            BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_WAIT_MEASURE_UPOS;
         }
      break;

      case (VOLTAGE_CORRECT_WAIT_MEASURE_UPOS):
         if(LVAR(AX0_s32_Pos_Vcmd) == 0)
         {
            // Wait for a certain time of standstill before continuing (dwell time)
            if(PassedTimeMS(BGVAR((long)s16_Voltage_Correct_Dwell_Time),(long)BGVAR(s64_Voltage_Correct_General_Purpose_01)))
            {
               // Clear general purpose variables
               BGVAR(s64_Voltage_Correct_General_Purpose_01) = BGVAR(s64_Voltage_Correct_General_Purpose_02) = 0LL;
               // Clear Variable that holds the PFB at the positive stop
               BGVAR(s64_Voltage_Correct_Position_Positive) = 0LL;
               // Jump to next state
               BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_MEASURE_UPOS;
            }
         }
         else
         {
            // Capture the 1ms timer
            BGVAR(s64_Voltage_Correct_General_Purpose_01) = (long long)Cntr_1mS;
         }
      break;

      case (VOLTAGE_CORRECT_MEASURE_UPOS):
         // If the motor is in standstill
         //if(BGVAR(s16_Motor_Moving) == 0)
         if(LVAR(AX0_s32_Pos_Vcmd) == 0)
         {
            BGVAR(s64_Voltage_Correct_General_Purpose_01)++;
            BGVAR(s64_Voltage_Correct_General_Purpose_02) += (long long)VAR(AX0_s16_An_In_1_Filtered);  // Measure and increment voltage at analog input 1
            do
            {
               u16_temp_time = Cntr_3125; // the same RT Interrupt
               s64_temp = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
            }while (u16_temp_time != Cntr_3125);
            BGVAR(s64_Voltage_Correct_Position_Positive) += s64_temp; // Measure and increment PFB (before modulo)

            // After 8 measurements
            if(BGVAR(s64_Voltage_Correct_General_Purpose_01) >= 8)
            {
               // Write in the highest field the averaged voltage
               BGVAR(s16_Voltage_Correct_Array)[BGVAR(u16_Voltage_Correct_Number_Of_Sections)] = (int)(BGVAR(s64_Voltage_Correct_General_Purpose_02) >> 3);
               // Generate the averaged value of PFB at the upper stop condition
               BGVAR(s64_Voltage_Correct_Position_Positive) = BGVAR(s64_Voltage_Correct_Position_Positive) >> 3;
               // Set "BGVAR(s64_Voltage_Correct_General_Purpose_01)" to 1 in order to allow right now only fast motion
               BGVAR(s64_Voltage_Correct_General_Purpose_01) = 1LL;
               // Jump to next state
               BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_INIT_NEG_MOVE;
            }
         }
      break;

      case (VOLTAGE_CORRECT_INIT_NEG_MOVE):

         // If slow motion is permitted and if we are getting closer to the hard stop
         if( (BGVAR(s64_Voltage_Correct_General_Purpose_01) == 0LL) &&
             ((VAR(AX0_s16_An_In_1_Filtered) >= BGVAR(s16_Slow_Movement_Upper_Voltage)) ||
              (VAR(AX0_s16_An_In_1_Filtered) <= BGVAR(s16_Slow_Movement_Lower_Voltage)))
           )
         {
            // motion must be absolute
            BGVAR(s16_Move_Abs_Issued) = 1;
            // allow correct behavior of limit switches.
            BGVAR(s16_Sal_Move_Ind) = 1;
            ExecuteMoveCommandInternal(drive, 0xF000000000000000LL, BGVAR(s32_Voltage_Correct_Slow_Speed), (long long)PTP_IMMEDIATE_TRANSITION, &s16_paramInvalid);

            BGVAR(s64_Voltage_Correct_General_Purpose_02) = 0LL; // Indicate slow motion triggered
         }
         else
         {
            // motion must be absolute
            BGVAR(s16_Move_Abs_Issued) = 1;
            // allow correct behavior of limit switches.
            BGVAR(s16_Sal_Move_Ind) = 1;
            ExecuteMoveCommandInternal(drive, 0xF000000000000000LL, BGVAR(s32_Voltage_Correct_Fast_Speed), (long long)PTP_IMMEDIATE_TRANSITION, &s16_paramInvalid);

            BGVAR(s64_Voltage_Correct_General_Purpose_02) = 1LL; // Indicate fast motion triggered
         }

         // Jump to next state
         BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_WAIT_FOR_NEG_STOP;
      break;

      case (VOLTAGE_CORRECT_WAIT_FOR_NEG_STOP):
         do
         {
            u16_temp_time = Cntr_3125; // the same RT Interrupt
            s64_temp = LLVAR(AX0_u32_Int_Loop_Pos_Err_Lo);
         }while (u16_temp_time != Cntr_3125);

         if(u16_temp_ccw_ls || (s64_temp < -0x200000000LL))
         {
            // Stop motion
            BGVAR(s16_Move_Abs_Issued) = 0;
            InitiatePtpMove(drive, 0x0LL, 0x0L, BGVAR(s32_Ptp_Acc_Rounded), BGVAR(s32_Ptp_Dec_Rounded), PTP_IMMEDIATE_TRANSITION);
            // Capture the 1ms timer
            BGVAR(s64_Voltage_Correct_General_Purpose_01) = (long long)Cntr_1mS;
            // If a backlash move is supposed tp be triggered
            if(BGVAR(s64_Voltage_Correct_Backlash_Position) > 0)
            {
               // Jump to next state
               BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_INIT_POS_BL_MOVE;
            }
            else
            {
               // Jump to next state
               BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_WAIT_MEASURE_UNEG;
            }
         }
         else if(BGVAR(s16_Slow_Movement_Upper_Voltage) != BGVAR(s16_Slow_Movement_Lower_Voltage)) // Check if velocity must be changed
         {
            // If we are getting closer to the hard stop
            if((VAR(AX0_s16_An_In_1_Filtered) >= BGVAR(s16_Slow_Movement_Upper_Voltage)) ||
               (VAR(AX0_s16_An_In_1_Filtered) <= BGVAR(s16_Slow_Movement_Lower_Voltage)))
            {
                if(BGVAR(s64_Voltage_Correct_General_Purpose_02) && !BGVAR(s64_Voltage_Correct_General_Purpose_01)) // If fast motion was previously triggered and is slow motion is allowed at all
                {
                  // Jump to previous state
                  BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_INIT_NEG_MOVE;
                }
            }
            else // Here we are inside of the voltage level window
            {
               BGVAR(s64_Voltage_Correct_General_Purpose_01) = 0LL; // Allow now slow motion when being next time out of the voltage level window

               if(!BGVAR(s64_Voltage_Correct_General_Purpose_02)) // If slow motion was previously triggered
               {
                  // Jump to previous state
                  BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_INIT_NEG_MOVE;
               }
            }
         }
      break;

      case (VOLTAGE_CORRECT_INIT_POS_BL_MOVE):
         if(LVAR(AX0_s32_Pos_Vcmd) == 0L) // Wait the PTP generator to come to a stop
         {
            do
            {
               u16_temp_time = Cntr_3125; // the same RT Interrupt
               s64_temp = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
            }while (u16_temp_time != Cntr_3125);

            // Calculate target position of the backlash move --> 2[rev] away from actual position
            BGVAR(s64_Voltage_Correct_General_Purpose_01) = s64_temp + BGVAR(s64_Voltage_Correct_Backlash_Position);
            // motion must be absolute position
            BGVAR(s16_Move_Abs_Issued) = 1;
            // allow correct behavior of software limits.1
            BGVAR(s16_Sal_Move_Ind) = 1;
            if(ExecuteMoveCommandInternal(drive, BGVAR(s64_Voltage_Correct_General_Purpose_01), BGVAR(s32_Voltage_Correct_Slow_Speed), (long long)PTP_IMMEDIATE_TRANSITION, &s16_paramInvalid) == SAL_SUCCESS)
            {
               // Jump to next state
               BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_WAIT_FOR_POS_BL_END;
            }
         }
      break;

      case (VOLTAGE_CORRECT_WAIT_FOR_POS_BL_END):
         do
         {
            u16_temp_time = Cntr_3125; // the same RT Interrupt
            s64_temp = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
         }while (u16_temp_time != Cntr_3125);

         if(BGVAR(s64_Voltage_Correct_General_Purpose_01) == s64_temp) // If PCMD reaches the target position of the backlash move
         {
            // Capture the 1ms timer
            BGVAR(s64_Voltage_Correct_General_Purpose_01) = (long long)Cntr_1mS;
            // Jump to next state
            BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_WAIT_MEASURE_UNEG;
         }
      break;

      case (VOLTAGE_CORRECT_WAIT_MEASURE_UNEG):
         if(LVAR(AX0_s32_Pos_Vcmd) == 0)
         {
            // Wait for a certain time of standstill before continuing (dwell time)
            if(PassedTimeMS((long)BGVAR(s16_Voltage_Correct_Dwell_Time),(long)BGVAR(s64_Voltage_Correct_General_Purpose_01)))
            {
               // Clear general purpose variables
               BGVAR(s64_Voltage_Correct_General_Purpose_01) = BGVAR(s64_Voltage_Correct_General_Purpose_02) = 0LL;
               // Clear Variable that holds the PFB at the negative stop
               BGVAR(s64_Voltage_Correct_Position_Negative) = 0LL;
               // Jump to next state
               BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_MEASURE_UNEG;
            }
         }
         else
         {
            // Capture the 1ms timer
            BGVAR(s64_Voltage_Correct_General_Purpose_01) = (long long)Cntr_1mS;
         }
      break;

      case (VOLTAGE_CORRECT_MEASURE_UNEG):
         // If the motor is in standstill
         //if(BGVAR(s16_Motor_Moving) == 0)
         if(LVAR(AX0_s32_Pos_Vcmd) == 0)
         {
            BGVAR(s64_Voltage_Correct_General_Purpose_01)++;
            BGVAR(s64_Voltage_Correct_General_Purpose_02) += (long long)VAR(AX0_s16_An_In_1_Filtered);  // Measure and increment voltage at analog input 1

            do
            {
               u16_temp_time = Cntr_3125; // the same RT Interrupt
               s64_temp = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
            }while (u16_temp_time != Cntr_3125);

            BGVAR(s64_Voltage_Correct_Position_Negative) += s64_temp; // Measure and increment PFB (before modulo)

            // After 8 measurements
            if(BGVAR(s64_Voltage_Correct_General_Purpose_01) >= 8)
            {
               // Write in the lowest field the averaged voltage
               BGVAR(s16_Voltage_Correct_Array)[0] = (int)(BGVAR(s64_Voltage_Correct_General_Purpose_02) >> 3);
               // Generate the averaged value of PFB at the lower stop condition
               BGVAR(s64_Voltage_Correct_Position_Negative) = BGVAR(s64_Voltage_Correct_Position_Negative) >> 3;
               // Set stroke index variable to 1 (start value)
               BGVAR(u16_Voltage_Correct_Stroke_Index) = 1;
               // Jump to next state
               BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_INIT_STROKE_MOVE;
            }
         }
      break;

      case (VOLTAGE_CORRECT_INIT_STROKE_MOVE):
         // Calculate here the size of one sector
         BGVAR(s64_Voltage_Correct_General_Purpose_01) = (BGVAR(s64_Voltage_Correct_Position_Positive) - BGVAR(s64_Voltage_Correct_Position_Negative)) / BGVAR(u16_Voltage_Correct_Number_Of_Sections);
         if(BGVAR(u16_Voltage_Correct_Stroke_Index) == 1) // If first stroke, init a motion from the actual position
         {
            // motion must be absolute
            BGVAR(s16_Move_Abs_Issued) = 1;
            // allow correct behavior of limit switches.
            BGVAR(s16_Sal_Move_Ind) = 1;
            ExecuteMoveCommandInternal(drive, LLVAR(s64_Voltage_Correct_Position_Negative) + BGVAR(s64_Voltage_Correct_General_Purpose_01), BGVAR(s32_Voltage_Correct_Fast_Speed), (long long)PTP_IMMEDIATE_TRANSITION, &s16_paramInvalid);
         }
         else
         {
            do
            {
               u16_temp_time = Cntr_3125; // the same RT Interrupt
               s64_temp = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
            }while (u16_temp_time != Cntr_3125);

            // motion must be absolute
            BGVAR(s16_Move_Abs_Issued) = 1;
            // allow correct behavior of limit switches.
            BGVAR(s16_Sal_Move_Ind) = 1;
            ExecuteMoveCommandInternal(drive, s64_temp + BGVAR(s64_Voltage_Correct_General_Purpose_01), BGVAR(s32_Voltage_Correct_Fast_Speed), (long long)PTP_IMMEDIATE_TRANSITION, &s16_paramInvalid);
         }
         // Jump to next state
         BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_WAIT_FOR_MOTION;
      break;

      case (VOLTAGE_CORRECT_WAIT_FOR_MOTION):
         if(LVAR(AX0_s32_Pos_Vcmd) != 0L)
         {
            // Jump to next state
            BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_WAIT_FOR_STOP;
         }
      break;

      case (VOLTAGE_CORRECT_WAIT_FOR_STOP):
         if(LVAR(AX0_s32_Pos_Vcmd) == 0L)
         {
            // Capture the 1ms timer
            BGVAR(s64_Voltage_Correct_General_Purpose_01) = (long long)Cntr_1mS;
            // Jump to next state
            BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_WAIT_MEASURE_USECTOR;
         }
      break;

      case(VOLTAGE_CORRECT_WAIT_MEASURE_USECTOR):
         if(LVAR(AX0_s32_Pos_Vcmd) == 0)
         {
            // Wait for a certain time of standstill before continuing (dwell time)
            if(PassedTimeMS((long)BGVAR(s16_Voltage_Correct_Dwell_Time),(long)BGVAR(s64_Voltage_Correct_General_Purpose_01)))
            {
               // Clear general purpose variables
               BGVAR(s64_Voltage_Correct_General_Purpose_01) = BGVAR(s64_Voltage_Correct_General_Purpose_02) = 0LL;
               // Jump to next state
               BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_MEASURE_USECTOR;
            }
         }
         else
         {
            // Capture the 1ms timer
            BGVAR(s64_Voltage_Correct_General_Purpose_01) = (long long)Cntr_1mS;
         }
      break;

      case (VOLTAGE_CORRECT_MEASURE_USECTOR):
         // If the motor is in standstill
         //if(BGVAR(s16_Motor_Moving) == 0)
         if(LVAR(AX0_s32_Pos_Vcmd) == 0)
         {
            BGVAR(s64_Voltage_Correct_General_Purpose_01)++;
            BGVAR(s64_Voltage_Correct_General_Purpose_02) += (long long)VAR(AX0_s16_An_In_1_Filtered);  // Measure and increment voltage at analog input 1

            // After 8 measurements
            if(BGVAR(s64_Voltage_Correct_General_Purpose_01) >= 8)
            {
               // Write in the assigned section entry the averaged voltage
               BGVAR(s16_Voltage_Correct_Array)[BGVAR(u16_Voltage_Correct_Stroke_Index)] = (int)(BGVAR(s64_Voltage_Correct_General_Purpose_02) >> 3);
               // Increment stroke index variable
               BGVAR(u16_Voltage_Correct_Stroke_Index)++;
               // Check if number of strokes are done
               if(BGVAR(u16_Voltage_Correct_Stroke_Index) >=  BGVAR(u16_Voltage_Correct_Number_Of_Sections))
               {
                  // Jump to next state
                  BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_SAVE_DATA;
               }
               else
               {
                  // Restart a stroke move
                  BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_INIT_STROKE_MOVE;
               }
            }
         }
      break;

      case (VOLTAGE_CORRECT_SAVE_DATA):
         // Use the variable to indicate a save-error -> Can be queried by a developer
         BGVAR(u16_Voltage_Correct_NvSave_Status) = SaveSineZeroParams(drive);
         if(BGVAR(u16_Voltage_Correct_NvSave_Status) != SAL_NOT_FINISHED)
         {
            // Jump to next state
            BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_CALC_RATIO;
         }
      break;

      case (VOLTAGE_CORRECT_CALC_RATIO):
         // Calculate SFB2MOTORNUM based on the delta voltage of the whole range
         BGVAR(s64_Voltage_Correct_General_Purpose_02) = (long long)BGVAR(s16_Voltage_Correct_Array)[BGVAR(u16_Voltage_Correct_Number_Of_Sections)] - (long long)BGVAR(s16_Voltage_Correct_Array)[0];

         // Convert into 1000 * [Volt] as resolution
         BGVAR(s64_Voltage_Correct_General_Purpose_02) = (BGVAR(s64_Voltage_Correct_General_Purpose_02) * 10000) / 26214; // 26214 = 10[V], 2621 = 1[V]
         SalSFB2MotorNumerator(BGVAR(s64_Voltage_Correct_General_Purpose_02), drive);


         // Calculate SFB2MOTORDEN based on the delta position of the whole range
         BGVAR(s64_Voltage_Correct_General_Purpose_02) = BGVAR(s64_Voltage_Correct_Position_Positive) - BGVAR(s64_Voltage_Correct_Position_Negative);
         // Convert into 1000 * [motor-rev] as resolution
         BGVAR(s64_Voltage_Correct_General_Purpose_02) = (BGVAR(s64_Voltage_Correct_General_Purpose_02) * 1000) >> 32; // 2^32 = 1 motor revolution
         // Denominator must be positive
         if(BGVAR(s64_Voltage_Correct_General_Purpose_02) < 0)
         {
            // Change algebraic sign of numerator
            BGVAR(s64_Voltage_Correct_General_Purpose_02) = -BGVAR(s64_Voltage_Correct_General_Purpose_02);
            SalSFB2MotorNumerator(-BGVAR(s32_SFB2MotorNum), drive);
         }
         SalSFB2MotorDenominator(BGVAR(s64_Voltage_Correct_General_Purpose_02), drive);


         // Jump to next state
         BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_FINISH_PROCEDURE;
      break;

      case(VOLTAGE_CORRECT_FINISH_PROCEDURE): // Finish process such as undo changes of ILIM at the beginning of the calibration process
         // If the process changed the current limit at the beginning of the process
         if(BGVAR(s32_Voltage_Correction_Current_Limit) > 0)
         {
            // Restore the original user current limit setting
            SalIlimCommand((long long)BGVAR(s32_Voltage_Correction_Prev_Curr_Limit), drive);
         }

         // Restore the following error parameter after the process
         BGVAR(u64_Pe_Max) = BGVAR(u64_Voltage_Correction_Prev_Pe_Max);

         // Call one time the sanity check function that SFB voltage correct info
         // command displays right afterwards the correct sanity-check error-state.
         SFBVoltageCorrectionSanityCheck(drive, 1); // Check analog input 1

         // Finish process
         BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_STATE_IDLE;
      break;

      default: // Not supposed to happen, unknown state
         BGVAR(u16_Voltage_Correct_State) = VOLTAGE_CORRECT_STATE_IDLE;
      break;
   }
}

//**********************************************************
// Function Name: SFBVoltageCorrectionSanityCheck
// Description:
//    This function has actually several purposes:
//       1) Check if the content of the array that holds the measured voltages
//          contains reasonable values.
//       2) If the sanity check of the voltage array succeeds, then it calculates
//          the fix/shift gain factors of each sector.
//       3) If the sanity check of the voltage array succeeds, then it activates
//          the voltage correction feature. If the sanity check fails, then it
//          deactivates the voltage correction feature.
//       4) The function initializes the RT pointers to the beginning of the
//          dedicated arrays.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void SFBVoltageCorrectionSanityCheck (int drive, int checkAnalogInput)
{
   unsigned int u16_index;
   signed long s32_delta_volt_sector_ideal;
   signed long s32_delta_volt_sector;
   int s16_voltage_progress = 0; // Progress in table. 0 = not yet defined, 1 = increasing values, -1 = decreasing values

   unsigned int *ptr_u16_Voltage_Correct_Sanity_Check_Err_Code;
   unsigned int *ptr_u16_Voltage_Correct_Number_Of_Sections;
   int          *ptr_s16_Voltage_Correct_Array;
   int          *ptr_s16_Voltage_Correct_Ideal;
   unsigned int *ptr_u16_Voltage_Correct_Enable;
   int          *ptr_s16_Voltage_Correct_Fix_Array;
   unsigned int *ptr_u16_Voltage_Correct_Shift_Array;
   REFERENCE_TO_DRIVE;

   // AXIS_OFF;

   if(checkAnalogInput == 1)
   {
      ptr_u16_Voltage_Correct_Sanity_Check_Err_Code = &BGVAR(u16_Voltage_Correct_Sanity_Check_Err_Code);
      ptr_u16_Voltage_Correct_Number_Of_Sections    = &BGVAR(u16_Voltage_Correct_Number_Of_Sections);
      ptr_s16_Voltage_Correct_Array                 = &BGVAR(s16_Voltage_Correct_Array)[0];
      ptr_s16_Voltage_Correct_Ideal                 = &BGVAR(s16_Voltage_Correct_Ideal);
      ptr_u16_Voltage_Correct_Enable                = &BGVAR(u16_Voltage_Correct_Enable);
      ptr_s16_Voltage_Correct_Fix_Array             = &BGVAR(s16_Voltage_Correct_Fix_Array)[0];
      ptr_u16_Voltage_Correct_Shift_Array           = &BGVAR(u16_Voltage_Correct_Shift_Array)[0];
   }
   else if (checkAnalogInput == 2)
   {
      ptr_u16_Voltage_Correct_Sanity_Check_Err_Code = &BGVAR(u16_Voltage_Correct_2_Sanity_Check_Err_Code);
      ptr_u16_Voltage_Correct_Number_Of_Sections    = &BGVAR(u16_Voltage_Correct_2_Number_Of_Sections);
      ptr_s16_Voltage_Correct_Array                 = &BGVAR(s16_Voltage_Correct_2_Array)[0];
      ptr_s16_Voltage_Correct_Ideal                 = &BGVAR(s16_Voltage_Correct_2_Ideal);
      ptr_u16_Voltage_Correct_Enable                = &BGVAR(u16_Voltage_Correct_2_Enable);
      ptr_s16_Voltage_Correct_Fix_Array             = &BGVAR(s16_Voltage_Correct_2_Fix_Array)[0];
      ptr_u16_Voltage_Correct_Shift_Array           = &BGVAR(u16_Voltage_Correct_2_Shift_Array)[0];
   }
   else
   {
      return;
   }

   *ptr_u16_Voltage_Correct_Sanity_Check_Err_Code = 0; // Clear error code variable

   /************************************************************************/
   /*** Part 1: Analyze the content of the array and the user parameter. ***/
   /************************************************************************/

   // Check user parameter, the whole range must be at least splitted in 2 sections. Otherwise skip the correction process.
   if((*ptr_u16_Voltage_Correct_Number_Of_Sections < 2) || (*ptr_u16_Voltage_Correct_Number_Of_Sections >= VOLTAGE_CORRECT_ARRAY_SIZE))
   {
      *ptr_u16_Voltage_Correct_Sanity_Check_Err_Code = 1; // Error, wrong user parameter setting
   }

   // If previous test succeeded
   if (*ptr_u16_Voltage_Correct_Sanity_Check_Err_Code == 0)
   {
      // Take the whole deltaVoltage and divide this value by the number of sectors. This is the delta-voltage of an ideal/linear progress of the voltage.
      s32_delta_volt_sector_ideal = ((long)ptr_s16_Voltage_Correct_Array[*ptr_u16_Voltage_Correct_Number_Of_Sections] - (long)ptr_s16_Voltage_Correct_Array[0]) / *ptr_u16_Voltage_Correct_Number_Of_Sections;
      // Now check if each deltaU in each sector is within a certain range
      for(u16_index = 0; u16_index < *ptr_u16_Voltage_Correct_Number_Of_Sections; u16_index++)
      {
         s32_delta_volt_sector = (long)ptr_s16_Voltage_Correct_Array[u16_index+1] - (long)ptr_s16_Voltage_Correct_Array[u16_index];

         // Determine on the first sample if the values are increasing or decreasing
         if((s16_voltage_progress == 0) && (s32_delta_volt_sector > 0))
         {
            s16_voltage_progress = 1; // Signalize increasing values
         }
         else if((s16_voltage_progress == 0) && (s32_delta_volt_sector < 0))
         {
            s16_voltage_progress = -1; // Signalize decreasing values
         }

         if ( (s32_delta_volt_sector > (s32_delta_volt_sector_ideal + VOLTAGE_CORRECT_DELTA_U_TOLERANCE)) ||
              (s32_delta_volt_sector < (s32_delta_volt_sector_ideal - VOLTAGE_CORRECT_DELTA_U_TOLERANCE)) ||
              (s32_delta_volt_sector == 0)
            )
         {
            *ptr_u16_Voltage_Correct_Sanity_Check_Err_Code = 2; // Error, too big voltage tolerance between the sectors or no delta voltage
         }
      }
   }

   // If previous test succeeded
   if (*ptr_u16_Voltage_Correct_Sanity_Check_Err_Code == 0)
   {
      // /*If the position pointer to the secondary feedback does not point to the analog voltage variable or*/ if the secondary feedback is not active at all
      if(/*(VAR(AX0_s16_Pext_Fdbk_Ptr) != (int)(long)&VAR(AX0_s16_An_In_1_Filtered)) ||*/ (VAR(AX0_s16_Pext_Fdbk_Flag) == 0))
      {
         *ptr_u16_Voltage_Correct_Sanity_Check_Err_Code = 3; // Error, secondary feedback pointer does not point to an analogue voltage
      }
   }

   // No error in the sanity checks before
   if (*ptr_u16_Voltage_Correct_Sanity_Check_Err_Code == 0)
   {
      /********************************************************************************/
      /*** Part 2: If no error then calculate the values for the fix/shift array's. ***/
      /********************************************************************************/
      for(u16_index = 0; u16_index < *ptr_u16_Voltage_Correct_Number_Of_Sections; u16_index++)
      {
         s32_delta_volt_sector = (long)ptr_s16_Voltage_Correct_Array[u16_index+1] - (long)ptr_s16_Voltage_Correct_Array[u16_index];
         FloatToFix16Shift16(&ptr_s16_Voltage_Correct_Fix_Array[u16_index], &ptr_u16_Voltage_Correct_Shift_Array[u16_index], (float)((float)s32_delta_volt_sector_ideal / (float)s32_delta_volt_sector));
      }

      /***************************************************************************************/
      /*** Part 3: Calculate the deltaVoltage  per section in case that the potentiometer  ***/
      /***         would have a linear progress over distance (linear progress). Therefore ***/
      /***         we calculate: (Upos - Uneg) / numberOfSections                          ***/
      /***************************************************************************************/
      *ptr_s16_Voltage_Correct_Ideal = (int)(((long)ptr_s16_Voltage_Correct_Array[*ptr_u16_Voltage_Correct_Number_Of_Sections] - (long)ptr_s16_Voltage_Correct_Array[0]) / *ptr_u16_Voltage_Correct_Number_Of_Sections);

      /******************************************************/
      /*** Part 4: Enable the voltage correction feature. ***/
      /******************************************************/
      if(s16_voltage_progress == 1)
      {
         *ptr_u16_Voltage_Correct_Enable = 1; // Enable voltage correction and indicate increasing values
      }
      else
      {
         *ptr_u16_Voltage_Correct_Enable = 2; // Enable voltage correction and indicate decreasing values
      }

   }
   else // Errors in the sanity checks before
   {
      *ptr_u16_Voltage_Correct_Enable = 0; // Disable voltage correction
   }
}

//**********************************************************
// Function Name: PrintS32With4DigitsViaRs232
// Description:
//    This function prints a S32 value with 4 digits via RS232. This is just a
//    small help function in order to fulfill the requested x.yyyy format from
//    Frencken to display e.g. voltage values.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void PrintS32With4DigitsViaRs232(long s32_value)
{
   unsigned long u32_temp;

   // If the value is negative
   if(s32_value < 0)
   {
      // print algebraic sign
      PrintString("-",0);
      // Generate value without algebraic sign
      u32_temp = -s32_value;
   }
   else
   {
      u32_temp = s32_value;
   }

   // Print value before decimal point
   PrintSignedInt32(u32_temp/10000);

   // print decimal point
   PrintString(".",0);

   // Calculate remainder without algebraic sign (always positive)
   u32_temp = u32_temp % 10000L;

   // Now print out digit per digit after the decimal point
   PrintSignedInt32(u32_temp/1000);
   u32_temp = u32_temp % 1000;
   PrintSignedInt32(u32_temp/100);
   u32_temp = u32_temp % 100;
   PrintSignedInt32(u32_temp/10);
   u32_temp = u32_temp % 10;
   PrintSignedInt32(u32_temp);
}

//**********************************************************
// Function Name: SalWriteVoltageCorrectionInformationToRs232
// Description:
//    This function prints debug information about the voltage correction feature
//    through the RS232. The voltage correction feature and the output of this
//    function is described in an internal Word document located on the STX Server.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteVoltageCorrectionInformationToRs232(int drive, int printAnalogInput)
{
   // AXIS_OFF;
   static int s16_state_machine = 0;
   static int s16_index = 0;
   int s16_anin1_filtered, s16_anin1_corrected;
   long  s32_temp;
   long long s64_temp;

   // Declare the pointers
   unsigned int *ptr_u16_Voltage_Correct_Enable;
   unsigned int *ptr_u16_Voltage_Correct_Number_Of_Sections;
   int          *ptr_s16_Sector_Identified;
   unsigned int *ptr_u16_Voltage_Correct_Sanity_Check_Err_Code;
   int          *ptr_s16_Voltage_Correct_Array;
   int          *ptr_s16_Voltage_Correct_Fix_Array;
   unsigned int *ptr_u16_Voltage_Correct_Shift_Array;
   int          *ptr_s16_Voltage_Correct_Result;
   int          *ptr_AX0_s16_An_In_Filtered;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (printAnalogInput == 1)
   {
      ptr_u16_Voltage_Correct_Enable                  = &BGVAR(u16_Voltage_Correct_Enable);
      ptr_u16_Voltage_Correct_Number_Of_Sections      = &BGVAR(u16_Voltage_Correct_Number_Of_Sections);
      ptr_s16_Sector_Identified                       = &BGVAR(s16_Sector_Identified);
      ptr_u16_Voltage_Correct_Sanity_Check_Err_Code   = &BGVAR(u16_Voltage_Correct_Sanity_Check_Err_Code);
      ptr_s16_Voltage_Correct_Array                   = &BGVAR(s16_Voltage_Correct_Array)[0];
      ptr_s16_Voltage_Correct_Fix_Array               = &BGVAR(s16_Voltage_Correct_Fix_Array)[0];
      ptr_u16_Voltage_Correct_Shift_Array             = &BGVAR(u16_Voltage_Correct_Shift_Array)[0];
      ptr_s16_Voltage_Correct_Result                  = &BGVAR(s16_Voltage_Correct_Result);
      ptr_AX0_s16_An_In_Filtered                      = &VAR(AX0_s16_An_In_1_Filtered);
   }
   else if (printAnalogInput == 2)
   {
      ptr_u16_Voltage_Correct_Enable                  = &BGVAR(u16_Voltage_Correct_2_Enable);
      ptr_u16_Voltage_Correct_Number_Of_Sections      = &BGVAR(u16_Voltage_Correct_2_Number_Of_Sections);
      ptr_s16_Sector_Identified                       = &BGVAR(s16_Sector_2_Identified);
      ptr_u16_Voltage_Correct_Sanity_Check_Err_Code   = &BGVAR(u16_Voltage_Correct_2_Sanity_Check_Err_Code);
      ptr_s16_Voltage_Correct_Array                   = &BGVAR(s16_Voltage_Correct_2_Array)[0];
      ptr_s16_Voltage_Correct_Fix_Array               = &BGVAR(s16_Voltage_Correct_2_Fix_Array)[0];
      ptr_u16_Voltage_Correct_Shift_Array             = &BGVAR(u16_Voltage_Correct_2_Shift_Array)[0];
      ptr_s16_Voltage_Correct_Result                  = &BGVAR(s16_Voltage_Correct_2_Result);
      ptr_AX0_s16_An_In_Filtered                      = &VAR(AX0_s16_An_In_2_Filtered);
   }
   else
   {
      return (SAL_SUCCESS); // Do nothing, can never happen...
   }


   if (u8_Output_Buffer_Free_Space < COMMS_BUFFER_SIZE - 10)
   {
      return SAL_NOT_FINISHED;
   }

   switch(s16_state_machine)
   {
      case(0): // Print if feature in the RT code is active or not
         PrintString("Feature: ",0);
         if(*ptr_u16_Voltage_Correct_Enable == 0)
         {
            PrintStringCrLf("Off",0);
         }
         else
         {
            PrintStringCrLf("On",0);
         }
         s16_state_machine++;
      break;

      case(1): // Print the state of the calibration process
         if (printAnalogInput == 1) // Just available for analog input 1
         {
            PrintString("Calibration state: ",0);
            PrintUnsignedInt16(BGVAR(u16_Voltage_Correct_State));
            PrintCrLf();
         }
         s16_state_machine++;
      break;

      case(2): // Print the number of sector variable
         PrintString("Number of sectors: ",0);
         PrintUnsignedInt16(*ptr_u16_Voltage_Correct_Number_Of_Sections);
         PrintCrLf();
         s16_state_machine++;
      break;

      case (3): // Print the sector that has been identified (0 and SFBSETCT means out of range)
         if(*ptr_u16_Voltage_Correct_Enable != 0)
         {
            PrintString("Identified sector: ",0);
            PrintSignedInt16(*ptr_s16_Sector_Identified);
            PrintCrLf();
         }
         s16_state_machine++;
      break;

      case(4): // Print error-code of the sanity-check function
         PrintString("Sanity check error-code: ",0);
         PrintUnsignedInt16(*ptr_u16_Voltage_Correct_Sanity_Check_Err_Code);
         PrintCrLf();
         s16_index = 0;
         s16_state_machine++;
      break;

      case(5): // Print the voltages measured during the calibration process in case that at least 2 sectors or selected
         if(*ptr_u16_Voltage_Correct_Number_Of_Sections < 2)
         {
            s16_index = 0;
            s16_state_machine++;
         }
         else
         {
            if(s16_index == 0)
            {
               PrintString("Voltage array: ",0);
            }

            if((s16_index < VOLTAGE_CORRECT_ARRAY_SIZE) && (s16_index <= *ptr_u16_Voltage_Correct_Number_Of_Sections))
            {
               // Frencken wants Volts with 4 digits and 26214 represents 10[V]. Therefore multiply with 10 * 10 * 1000.
               s32_temp = (long)(100000LL * (long long)ptr_s16_Voltage_Correct_Array[s16_index] / 26214);
               // Print the voltage values in unit [V] with 4 decimal places.
               PrintS32With4DigitsViaRs232(s32_temp);
               PrintString("[V],  ",0);
            }

            s16_index++;

            if(s16_index >= VOLTAGE_CORRECT_ARRAY_SIZE)
            {
               PrintCrLf();
               s16_index = 0;
               s16_state_machine++;
            }
         }
      break;

      case(6): // Print the gains calculated out of the measured voltages in case that at least 2 sectors or selected
         if(s16_index == 0)
         {
            PrintString("Gains: ",0);
         }

         if((s16_index < VOLTAGE_CORRECT_ARRAY_SIZE-1) && (s16_index < *ptr_u16_Voltage_Correct_Number_Of_Sections))
         {
            // Calculate gain multiplied with 1000 due to representation with 3 decimal places
            s32_temp = ((long)ptr_s16_Voltage_Correct_Fix_Array[s16_index] * 1000) >> ptr_u16_Voltage_Correct_Shift_Array[s16_index];
            PrintString(DecimalPoint32ToAscii(s32_temp),0);
            PrintString(",  ",0);
         }

         s16_index++;

         if(s16_index >= VOLTAGE_CORRECT_ARRAY_SIZE-1)
         {
            PrintCrLf();
            s16_index = 0;
            s16_state_machine++;
         }
      break;

      case(7): // Print the position at the positive and negative stop in user units (just for analog input 1)
         if((BGVAR(s64_Voltage_Correct_Position_Positive) != BGVAR(s64_Voltage_Correct_Position_Negative)) && (printAnalogInput == 1))
         {
            // Get position conversion index
            s16_index = UnitsConversionIndex(&s8_Units_Pos, drive);
            // Convert PFB_pos:
            s64_temp = MultS64ByFixS64ToS64(BGVAR(s64_Voltage_Correct_Position_Positive),
                                            BGVAR(Unit_Conversion_Table[s16_index]).s64_unit_conversion_to_user_fix,
                                            BGVAR(Unit_Conversion_Table[s16_index]).u16_unit_conversion_to_user_shr);
            PrintString("PFB_pos: ",0);
            PrintSignedLongLong(s64_temp);
            PrintString("   /   PFB_neg: ",0);
            // Convert PFB_neg:
            s64_temp = MultS64ByFixS64ToS64(BGVAR(s64_Voltage_Correct_Position_Negative),
                                            BGVAR(Unit_Conversion_Table[s16_index]).s64_unit_conversion_to_user_fix,
                                            BGVAR(Unit_Conversion_Table[s16_index]).u16_unit_conversion_to_user_shr);
            PrintSignedLongLong(s64_temp);

            // Print unit
            PrintString("   in [",0);
            PrintString(BGVAR(s8_Units_Pos) ,0);
            PrintString("]",0);

            PrintCrLf();
         }
         s16_state_machine++;
      break;

      case(8): // Print current result of the correction feature
         if(*ptr_u16_Voltage_Correct_Enable != 0)
         {
            do
            {
               s16_index = Cntr_3125;
               s16_anin1_filtered = *ptr_AX0_s16_An_In_Filtered;
               s16_anin1_corrected = *ptr_s16_Voltage_Correct_Result;
            } while (s16_index != Cntr_3125);

            PrintString("U_measured: ",0);
            PrintSignedInt16(s16_anin1_filtered);
            PrintString("[Counts] = ",0);
            // Frencken wants Volts with 4 digits and 26214 represents 10[V]. Therefore multiply with 10 * 10 * 1000.
            s32_temp = (long)(100000LL * (long long)s16_anin1_filtered / 26214);
            // Print the voltage values in unit [V] with 4 decimal places.
            PrintS32With4DigitsViaRs232(s32_temp);
            PrintString("[V]",0);

            PrintCrLf();

            PrintString("U_corrected: ",0);
            PrintSignedInt16(s16_anin1_corrected);
            PrintString("[Counts] = ",0);
            // Frencken wants Volts with 4 digits and 26214 represents 10[V]. Therefore multiply with 10 * 10 * 1000.
            s32_temp = (long)(100000LL * (long long)s16_anin1_corrected / 26214);
            // Print the voltage values in unit [V] with 4 decimal places.
            PrintS32With4DigitsViaRs232(s32_temp);
            PrintString("[V]",0);

            PrintCrLf();
         }

         s16_state_machine++;
      break;

      case(9): // Print error-code of the NV save command
         PrintString("NV save status: ",0);
         PrintUnsignedInt16(BGVAR(u16_Voltage_Correct_NvSave_Status));
         PrintCrLf();
         s16_state_machine++;
      break;


      default:
         // Indicate end of function
         s16_state_machine = 0xFFFF;
      break;
   }

   if(s16_state_machine == 0xFFFF)
   {
      // Re-initialize all relevant variables
      s16_state_machine = 0;
      s16_index = 0;
      // Return success
      return (SAL_SUCCESS);
   }
   else
   {
      return (SAL_NOT_FINISHED);
   }
}

//**********************************************************
// Function Name: SalWriteVoltageCorrectionInfoToRs232
// Description:
//    This function prints debug information about the voltage correction feature
//    of analog input 1 through the RS232.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteVoltageCorrectionInfoToRs232(int drive)
{
   return (SalWriteVoltageCorrectionInformationToRs232(drive, 1));
}

//**********************************************************
// Function Name: SalWriteVoltageCorrection2InfoToRs232
// Description:
//    This function prints debug information about the voltage correction feature
//    of analog input 1 through the RS232.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteVoltageCorrection2InfoToRs232(int drive)
{
   return (SalWriteVoltageCorrectionInformationToRs232(drive, 2));
}

//**********************************************************
// Function Name: SalSFBVoltageCorrectSlowSpeedUpperVoltLevel
// Description:
//          This function is called in response to the SFBVCVPOS command.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalSFBVoltageCorrectSlowSpeedUpperVoltLevel(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

//    if (lparam > 26214) return (VALUE_TOO_HIGH);
//    if (lparam < -26214) return (VALUE_TOO_LOW);

   BGVAR(s16_Slow_Movement_Upper_Voltage) = (int)lparam;

   return (SAL_SUCCESS);
}
//**********************************************************
// Function Name: SalSFBVoltageCorrectSlowSpeedLowerVoltLevel
// Description:
//          This function is called in response to the SFBVCVPOS command.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalSFBVoltageCorrectSlowSpeedLowerVoltLevel(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

//    if (lparam > 26214) return (VALUE_TOO_HIGH);
//    if (lparam < -26214) return (VALUE_TOO_LOW);

   BGVAR(s16_Slow_Movement_Lower_Voltage) = (int)lparam;

   return (SAL_SUCCESS);
}
//**********************************************************
// Function Name: SalSFBVoltageCorrectFastSpeed
// Description:
//          This function is called in response to the SFBVCSPDFAST command.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalSFBVoltageCorrectFastSpeed(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL) return (VALUE_TOO_LOW);
   if (lparam > (long long)53687091) return (VALUE_TOO_HIGH); // 100[rps] in [Counts/125us] is the maximum

   BGVAR(s32_Voltage_Correct_Fast_Speed) = (long)lparam;

   return (SAL_SUCCESS);
}
//**********************************************************
// Function Name: SalSFBVoltageCorrectSlowSpeed
// Description:
//          This function is called in response to the SFBVCSPDSLOW command.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalSFBVoltageCorrectSlowSpeed(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL) return (VALUE_TOO_LOW);
   if (lparam > (long long)53687091) return (VALUE_TOO_HIGH); // 100[rps] in [Counts/125us] is the maximum

   BGVAR(s32_Voltage_Correct_Slow_Speed) = (long)lparam;

   return (SAL_SUCCESS);
}
//**********************************************************
// Function Name: SalSFBVoltageCorrectCurrentLimit
// Description:
//          This function is called in response to the SFBVCILIM command.
//
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
int SalSFBVoltageCorrectCurrentLimit(long long lparam, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (lparam < 0LL) return (VALUE_TOO_LOW);
   if (lparam > (long long)BGVAR(s32_Imax)) return (VALUE_TOO_HIGH);
   // Do not change the current limit while the procedure is running in order to properly restore the original user current limit setting
   if (ProcedureRunning(DRIVE_PARAM) == PROC_VOLT_CALIBRATION) return (OTHER_PROCEDURE_RUNNING);

   BGVAR(s32_Voltage_Correction_Current_Limit) = (long)lparam;

   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalWriteSfbVcBacklashDistance
// Description:
//       This function is called in response to the SFBVCBLDIST command.
//
//
// Author: A. H.
// Algorithm:
// Revisions:
//**********************************************************
int SalWriteSfbVcBacklashDistance(long long param, int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(param < 0) return VALUE_TOO_LOW;

   BGVAR(s64_Voltage_Correct_Backlash_Position) = param;

   return SAL_SUCCESS;
}

void MoveSineAbort(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   LVAR(AX0_u32_Move_Sine_Repeats) = 1;
   BGVAR(u16_Move_Sine_State) = MOVE_SINE_AFTER_SINE_CYCLE;   
}

//**********************************************************
// Function Name: MoveSine
// Description:
//   This function calculates the position commands that creates the sine wave executed by ASCII command MOVESINE
//
//
// Author: Itai
// Algorithm:
// Revisions:
//**********************************************************
#pragma CODE_SECTION(MoveSine, "ramfunc_2");
void MoveSine(void)
{

   // AXIS_OFF;

   if(!Enabled(0))//if drive moved to Disabled due to a Command/Fault/... 
   {
      //imitate the state in which there are more not repeats and function need to end
      BGVAR(u16_Move_Sine_State) = MOVE_SINE_AFTER_SINE_CYCLE;
      LVAR(AX0_u32_Move_Sine_Repeats) = 1;
   }


   //Compute Sine: SIN(PI /2 + 2 * PI * f / Samples Per Second * T) * A
   BGVAR(s64_Move_Sine_Val) = (long long)((float)sin(1.57079632 + BGVAR(f_Move_Sine_2_Pi_Freq) * BGVAR(s32_Move_Sine_ctr)) * (float)BGVAR(s64_Move_Sine_Amp));

   //   Validate Sample Rate
   if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)
   {
       BGVAR(s64_Move_Sine_Val) >>= 1;
   }

   //IDLE State
   if(BGVAR(u16_Move_Sine_State) == MOVE_SINE_IDLE_STATE)
   {
      BGVAR(s64_Move_Sine_Prev_Val) = BGVAR(s64_Move_Sine_Val); //Init previous sine value to current sine value
      BGVAR(u16_Move_Sine_State) = MOVE_SINE_IN_SINE_CYCLE;            //Go to next state
   }
   else if(BGVAR(u16_Move_Sine_State) == MOVE_SINE_IN_SINE_CYCLE) // Within a sine cycle state
   {
      if(LVAR(AX0_u32_Move_Sine_Steps_Counter) == 1) //If the sine cycle ended - go to next state
      {
         BGVAR(u16_Move_Sine_State) = MOVE_SINE_AFTER_SINE_CYCLE;
      }
   }
   else if(BGVAR(u16_Move_Sine_State) == MOVE_SINE_AFTER_SINE_CYCLE) //After sine cycle ended state
   {
	  LVAR(AX0_u32_Move_Sine_Repeats) = LVAR(AX0_u32_Move_Sine_Repeats) - 1;// decrement one from number of repeats

      if(LVAR(AX0_u32_Move_Sine_Repeats) == 0) //No more cycles to execute - this is the last cycle
      {
         LVAR(AX0_s32_Pos_Vcmd_Sine) = 0L;
         LVAR(AX0_u32_Move_Sine_Steps) = 0L;
         BGVAR(u16_Move_Sine_State) = 0;
    
         //Restore the pointer back to the value it was before MOVESINE command
         //VAR(AX0_s16_Pos_Loop_Vcmnd_Ptr) = BGVAR(s16_Move_Sine_Saved_Vel_Ptr_Value);	
		 POS_LOOP_VCMD_PTR_ASSIGN(BGVAR(s16_Move_Sine_Saved_Vel_Ptr_Value));
		 

         return;
      }
      else //This is the end of 1 sine cycle but there are more cycles to come - Go back for another cycle
      {
         LVAR(AX0_u32_Move_Sine_Steps_Counter) = LVAR(AX0_u32_Move_Sine_Steps);
         BGVAR(u16_Move_Sine_State) = MOVE_SINE_IN_SINE_CYCLE;
         BGVAR(s32_Move_Sine_ctr) = 0L;
      }
   }

   // Derive sine wave and inject as position loop velocity command
   LVAR(AX0_s32_Pos_Vcmd_Sine) = (long)(BGVAR(s64_Move_Sine_Val) - BGVAR(s64_Move_Sine_Prev_Val));

   //Set previos sine value to current sine value
   BGVAR(s64_Move_Sine_Prev_Val) = BGVAR(s64_Move_Sine_Val);

   //increment counter
   BGVAR(s32_Move_Sine_ctr)++;   
}

