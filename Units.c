#include <string.h>

#include "Design.def"
#include "i2c.def"

#include "Units.var"
#include "Drive.var"
#include "Extrn_Asm.var"
#include "Motor.var"
#include "ExFbVar.var"
#include "ModCntrl.var"
#include "Position.var"

#include "Prototypes.pro"
#include "ExFbVar.def"


/**********************************************************
*
*            Local Functions
*
**********************************************************/
void ConvertAmpToInternal(int drive)
{
   long s32_fix = 0L;
   unsigned int  u16_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //internal = user / DIPEAK * 26214
   OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_Drive_I_Peak));
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)26214, (unsigned int)0);

   BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;
   BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION_SCALE]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION_SCALE]).u16_unit_conversion_to_internal_shr = u16_shift;

   //recalculate FB_CAN_CURRENT_RECORDING_CONVERSION conversion
   MultFixS32ByFixS32ToFixS32(&s32_fix,&u16_shift,s32_fix,u16_shift,
                             (long)BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix,
                            BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr);

   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_RECORDING_CONVERSION]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_RECORDING_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;
}


void ConvertInternalToAmp1000(int drive)
{
   long s32_fix = 0L;
   unsigned int  u16_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //user = internal / 26214 * DIPEAK
   // 1/26214 = 1342197760>>45
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(s32_Drive_I_Peak),0,(long)1342197760, (unsigned int)45);

   BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;
   BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION_SCALE]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION_SCALE]).u16_unit_conversion_to_user_shr = u16_shift;
}


void ConvertVelToInternalOutLoop(int drive)
{
   // AXIS_OFF;
   long          s32_fix = 1L, s32_pitch_factor_fix = 1099511627L;//changed since mpitch changed to decimal
   unsigned int  u16_shift = 0, u16_pitch_factor_shift = 40, u16_Actual_Units = 0, u16_convert_load_units_also = 0;
   unsigned int  u16_motor_load_type = 0, u16_unit_conversion_table_index = 0, u16_unit_conversion_table_index2 = 0, u16_unit_conversion_table_index3 = 0;
   unsigned long u32_pitch = 0L;
   
   while (IS_SECONDARY_FEEDBACK_ENABLED >= u16_convert_load_units_also)
   {
      s32_pitch_factor_fix = 1099511627L;
	  u16_pitch_factor_shift = 40;
      s32_fix = 1L;
      u16_shift = 0;      
   
      if (0 == u16_convert_load_units_also) // Motor units convertion
      {
         u16_motor_load_type = BGVAR(u16_MotorType);
         u32_pitch = BGVAR(u32_Mpitch);
         u16_unit_conversion_table_index = VELOCITY_OUT_OF_LOOP_CONVERSION;
         u16_unit_conversion_table_index2 = VELOCITY_OUT_OF_LOOP_SCALE_CONVERSION;
         u16_unit_conversion_table_index3 = VELOCITY_PTP_CONVERSION;
      }
      else // Load units convertion - Dual Loop
      {
         u16_motor_load_type = LOAD_TYPE;
         u32_pitch = 1000;
         u16_unit_conversion_table_index = VELOCITY_LOAD_OUT_OF_LOOP_CONVERSION;     
         u16_unit_conversion_table_index2 = VELOCITY_LOAD_OUT_OF_LOOP_SCALE_CONVERSION;  
         u16_unit_conversion_table_index3 = VELOCITY_LOAD_PTP_CONVERSION;
      }

      if (u16_motor_load_type == ROTARY_MOTOR)  //rotary
      {
         u16_Actual_Units = BGVAR(u16_Units_Vel_Rotary);  // actual units are one of the rotary units
      }
      else // linear motor
      {
         u16_Actual_Units = RPS_UNITS;  //since all linear units are per second we choose the rotary unit per second
                                        //plus mpitch factor

         switch(BGVAR(u16_Units_Vel_Linear))// set mpicth factor according to current linear units
         {
               case UM_PER_SEC_UNITS: // 1 / (u32_Mpitch * 1000)
                  OneDivS64ToFixU32Shift16(&s32_pitch_factor_fix, &u16_pitch_factor_shift,(long long)(u32_pitch));
                  MultFixS32ByFixS32ToFixS32(&s32_pitch_factor_fix, &u16_pitch_factor_shift, s32_pitch_factor_fix,u16_pitch_factor_shift,(long)1099511628, (unsigned int)40);
               break;

               case MM_PER_SEC_UNITS: // 1 / u32_Mpitch
                  OneDivS64ToFixU32Shift16(&s32_pitch_factor_fix, &u16_pitch_factor_shift,(long long)u32_pitch);
               break;

            case USER_PER_SEC_LIN_UNITS:
            break;
         }
      }

      switch(u16_Actual_Units)
      {
         case RPS_UNITS:
            //internal = user / 1000 * 2^32 / 8000 / MPITCH_FACTOR = user * 536870.912 / MPITCH_FACTOR
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_pitch_factor_fix,u16_pitch_factor_shift,(long)1099511628, (unsigned int)11);//changed since mpitch changed to decimal
         break;

         case RPM_UNITS:
            //internal = user / 1000 / 60 * 2^32 / 8000 = user * 8.9478485333333333333333333333333
            s32_fix = 1200959901;
            u16_shift = 27;
         break;

         case DEG_PER_SEC_UNITS:
            //internal = user / 1000 / 360 * 2^32 / 8000 = user * 1.4913080888888888888888888888889
            s32_fix = 1601279868;
            u16_shift = 30;
         break;

         case USER_PER_SEC_ROT_UNITS:
            //internal = user / 1000 / (pnum/pden) * 2^32 / 8000 = user * 536.870912 / (pnum/pden)
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(u32_Scaling_Pnumerator));
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(u32_Scaling_Pdenominator), (unsigned int)0);
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)1125899907,(unsigned int)21);
         break;
      }

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).u16_unit_conversion_to_internal_shr = u16_shift;

     //Udi May 28 2014 : Add missing conversion - See Shift -1.
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index3]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index3]).u16_unit_conversion_to_internal_shr = u16_shift;
      if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index3]).u16_unit_conversion_to_internal_shr-=1;

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index2]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index2]).u16_unit_conversion_to_internal_shr = u16_shift;
      u16_convert_load_units_also++;
   }
   
   ConvertInternalVelInToInternalVelOut(drive);
   ConvertInternalVelOutToInternalVelIn(drive);
}


void ConvertInternalToVel1000OutLoop(int drive)
{
   // AXIS_OFF;
   long          s32_fix = 1L, s32_pitch_factor_fix = 1000L;//changed since mpitch changed to decimal
   unsigned int  u16_motor_load_type = 0, u16_unit_conversion_table_index = 0, u16_convert_load_units_also = 0;
   unsigned int  u16_shift = 0, u16_pitch_factor_shift = 0, u16_Actual_Units = 0, u16_unit_conversion_table_index2 = 0, u16_unit_conversion_table_index3 = 0;
   long u32_pitch = 0L;
   
   while (IS_SECONDARY_FEEDBACK_ENABLED >= u16_convert_load_units_also)
   {
      s32_pitch_factor_fix = 1000L;
      s32_fix = 1LL;
      u16_shift = 0;
      
      if (0 == u16_convert_load_units_also) // Motor units convertion
      {
         u16_motor_load_type = BGVAR(u16_MotorType);
         u32_pitch = BGVAR(u32_Mpitch);
         u16_unit_conversion_table_index = VELOCITY_OUT_OF_LOOP_CONVERSION;
         u16_unit_conversion_table_index2 = VELOCITY_OUT_OF_LOOP_SCALE_CONVERSION;
         u16_unit_conversion_table_index3 = VELOCITY_PTP_CONVERSION;
      }
      else // Load units convertion - Dual Loop
      {
         u16_motor_load_type = LOAD_TYPE;
         u32_pitch = 1000;
         u16_unit_conversion_table_index = VELOCITY_LOAD_OUT_OF_LOOP_CONVERSION;     
         u16_unit_conversion_table_index2 = VELOCITY_LOAD_OUT_OF_LOOP_SCALE_CONVERSION;  
         u16_unit_conversion_table_index3 = VELOCITY_LOAD_PTP_CONVERSION;
      }
	   if (u16_motor_load_type == ROTARY_MOTOR)  //rotary 
	   {
	      u16_Actual_Units = BGVAR(u16_Units_Vel_Rotary);  // actual units are one of the rotary units
	   }
	   else // linear motor
	   {
	      u16_Actual_Units = RPS_UNITS;  //since all linear units are per second we choose the rotary unit per second
	                                     //plus mpitch factor

	      switch(BGVAR(u16_Units_Vel_Linear))// set mpicth factor according to current linear units
	      {
	         case UM_PER_SEC_UNITS: //* u32_Mpitch * 1000
               s32_pitch_factor_fix = (u32_pitch * 1000);
	            //MultFixS32ByFixS32ToFixS32(&s32_mpitch_factor_fix, &u16_mpitch_factor_shift, (long)1000,(unsigned int)0,(long)BGVAR(u32_Mpitch), (unsigned int)0);
	         break;

            case MM_PER_SEC_UNITS: // * u32_Mpitch
               s32_pitch_factor_fix = u32_pitch;
            break;

	         case USER_PER_SEC_LIN_UNITS:
	         break;
	      }
	   }

      switch(u16_Actual_Units)
      {
         case RPS_UNITS:
            //user = internal * 1000 * 8000 / 2^32 * MPITCH_FACTOR = internal * 0.00000186264514923095703125 * MPITCH_FACTOR
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_pitch_factor_fix,u16_pitch_factor_shift,(long)2097152000, (unsigned int)50);//changed since mpitch changed to decimal
            break;

         case RPM_UNITS:
            //user = internal * 1000 * 8000 * 60 / 2^32 = internal * 0.111758708953857421875
            s32_fix = 1920000000;
            u16_shift = 34;
            break;

         case DEG_PER_SEC_UNITS:
            //user = internal * 1000 * 8000 * 360 / 2^32 = internal * 0.67055225372314453125
            s32_fix = 1440000000;
            u16_shift = 31;
            break;

         case USER_PER_SEC_ROT_UNITS:
            //user = internal * 1000 * 8000 * (pnum/pden) / 2^32 = internal * 0.00186264 * (pnum/pden)
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(u32_Scaling_Pdenominator));
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(u32_Scaling_Pnumerator), (unsigned int)0);
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)2048000000, (unsigned int)40);
            break;
      }

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).u16_unit_conversion_to_user_shr = u16_shift;

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index2]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index2]).u16_unit_conversion_to_user_shr = u16_shift;

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index3]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index3]).u16_unit_conversion_to_user_shr = u16_shift;
      if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index3]).u16_unit_conversion_to_user_shr +=1;
      u16_convert_load_units_also++;
   }

   ConvertInternalVelInToInternalVelOut(drive);
   ConvertInternalVelOutToInternalVelIn(drive);
}


void ConvertVelToInternalInLoop(int drive)
{
   long s32_fix_tmp = 0L, s32_fix = 1L, s32_pitch_factor_fix = 1099511627LL;
   unsigned int u16_shift_tmp = 0, u16_shift = 0, u16_pitch_factor_shift = 40, u16_convert_load_units_also = 0;
   unsigned int  u16_Actual_Units = 0,  u16_motor_load_type = 0, u16_unit_conversion_table_index = 0;
   unsigned long u32_pitch = 0L;

   while (IS_SECONDARY_FEEDBACK_ENABLED >= u16_convert_load_units_also)
   {
      s32_pitch_factor_fix = 1099511627L;
	   u16_pitch_factor_shift = 40;
      s32_fix = 1L;
      u16_shift = 0;      
   
      if (0 == u16_convert_load_units_also) // Motor units convertion
      {
         u16_motor_load_type = BGVAR(u16_MotorType);
         u32_pitch = BGVAR(u32_Mpitch);
         u16_unit_conversion_table_index = VELOCITY_IN_LOOP_CONVERSION;
      }
      else // Load units convertion - Dual Loop
      {
         u16_motor_load_type = LOAD_TYPE;
         u32_pitch = 1000;
         u16_unit_conversion_table_index = VELOCITY_LOAD_IN_LOOP_CONVERSION; 
      }
      
      if (u16_motor_load_type == ROTARY_MOTOR)  //rotary motor
      {
         u16_Actual_Units = BGVAR(u16_Units_Vel_Rotary);  // actual units are one of the rotary units
      }
      else // linear motor
      {
         u16_Actual_Units = RPS_UNITS;  //since all linear units are per second we choose the rotary unit per second
                                        //plus mpitch factor

         switch(BGVAR(u16_Units_Vel_Linear))// set mpicth factor according to current linear units
         {
            case UM_PER_SEC_UNITS: // 1 / (u32_Mpitch * 1000)
            break;

            case MM_PER_SEC_UNITS: // 1 / u32_Mpitch
               OneDivS64ToFixU32Shift16(&s32_pitch_factor_fix, &u16_pitch_factor_shift,(long long)u32_pitch);
            break;

            case USER_PER_SEC_LIN_UNITS:
            break;
         }
      }

      switch(u16_Actual_Units)
      {
         case RPS_UNITS:
            //internal = user / 1000 * 22750 / VLIM_INT * 2^32 / 8000 / MPITCH_FACTOR = user * 12213813.248 / VLIM_INT / MPITCH_FACTOR
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)1563368096, (unsigned int)7);
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_pitch_factor_fix,u16_pitch_factor_shift,s32_fix,u16_shift);
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)1000,(unsigned int)0);//added since mpitch changed to decimal
         break;

         case RPM_UNITS:
            //internal = user / 1000 / 60 * 22750 / VLIM_INT * 2^32 / 8000 = user * 203563.554133 / VLIM_INT
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)1667592636, (unsigned int)13);
         break;

         case DEG_PER_SEC_UNITS:
            //internal = user / 1000 / 360 * 22750 / VLIM_INT * 2^32 / 8000 = user * 33927.259022222222222 / VLIM_INT
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)1111728424, (unsigned int)15);
         break;

         case USER_PER_SEC_ROT_UNITS:
            //internal = user / 1000 / (pnum/pden) * 22750 / VLIM_INT * 2^32 / 8000 = user * 12213813.248 / (pnum/pden) / VLIM_INT
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(u32_Scaling_Pnumerator));
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)1563368096, (unsigned int)7);
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(u32_Scaling_Pdenominator), (unsigned int)0);
            OneDivS64ToFixU32Shift16(&s32_fix_tmp, &u16_shift_tmp,(long long)BGVAR(s32_V_Lim_Design));
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,s32_fix_tmp,u16_shift_tmp);
         break;
      }

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).u16_unit_conversion_to_internal_shr = u16_shift;
      u16_convert_load_units_also++;
   }
   ConvertInternalVelInToInternalVelOut(drive);
   ConvertInternalVelOutToInternalVelIn(drive);
}


void ConvertInternalToVel1000InLoop(int drive)
{
   long          s32_fix = 1L, s32_pitch_factor_fix = 1000L;
   unsigned int  u16_shift = 0, u16_pitch_factor_shift = 0, u16_Actual_Units = 0, u16_convert_load_units_also = 0;
   unsigned int  u16_motor_load_type = 0, u16_unit_conversion_table_index = 0;
   unsigned long u32_pitch = 0L;

   while (IS_SECONDARY_FEEDBACK_ENABLED >= u16_convert_load_units_also)
   {
      s32_pitch_factor_fix = 1000L;
	   u16_pitch_factor_shift = 0;
      s32_fix = 1L;
      u16_shift = 0;
      
      if (0 == u16_convert_load_units_also) // Motor units convertion
      {
         u16_motor_load_type = BGVAR(u16_MotorType);
         u32_pitch = BGVAR(u32_Mpitch);
         u16_unit_conversion_table_index = VELOCITY_IN_LOOP_CONVERSION;
      }
      else // Load units convertion - Dual Loop
      {
         u16_motor_load_type = LOAD_TYPE;
         u32_pitch = 1000;
         u16_unit_conversion_table_index = VELOCITY_LOAD_IN_LOOP_CONVERSION;  
      }
      
      if (u16_motor_load_type == ROTARY_MOTOR)  //rotary motor
      {
         u16_Actual_Units = BGVAR(u16_Units_Vel_Rotary);  // actual units are one of the rotary units
      }
      else // linear motor
      {
           u16_Actual_Units = RPS_UNITS;  //since all linear units are per second we choose the rotary unit per second
                                          //plus mpitch factor

         switch(BGVAR(u16_Units_Vel_Linear))// set mpicth factor according to current linear units
         {
            case UM_PER_SEC_UNITS: //* u32_Mpitch * 1000
            break;

            case MM_PER_SEC_UNITS: // * u32_Mpitch
               s32_pitch_factor_fix = u32_pitch;
            break;

            case USER_PER_SEC_LIN_UNITS:
            break;
         }
       }

      switch(u16_Actual_Units)
      {
         case RPS_UNITS:
             //user = internal * 1000 * 8000 * VLIM_INT / 22750 / 2^32 * MPITCH_FACTOR = internal * VLIM_INT * 0.000000000081874 * MPITCH_FACTOR
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(s32_V_Lim_Design),0,(long)1510308720, (unsigned int)64);//changed since mpitch changed to decimal
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_pitch_factor_fix,u16_pitch_factor_shift,s32_fix,u16_shift);
         break;

         case RPM_UNITS:
             //user = internal * 1000 * 8000 * VLIM_INT * 60 / 22750 / 2^32 = internal * VLIM_INT * 0.0000049124
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(s32_V_Lim_Design),0,(long)1382737582, (unsigned int)48);
         break;

         case DEG_PER_SEC_UNITS:
             //user = internal * 1000 * 8000 * VLIM_INT * 360 / 22750 / 2^32 = internal * VLIM_INT * 0.0000294748
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(s32_V_Lim_Design),0,(long)2074106374, (unsigned int)46);
         break;

         case USER_PER_SEC_ROT_UNITS:
             //user = internal * 1000 * 8000 * VLIM_INT * (pnum/pden) / 22750 / 2^32 = internal * VLIM_INT * (pden/pnum) * 0.000000081874
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(u32_Scaling_Pdenominator));
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)1474920088, (unsigned int)54);
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(u32_Scaling_Pnumerator), (unsigned int)0);
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(s32_V_Lim_Design),(unsigned int)0);
         break;
      }

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).u16_unit_conversion_to_user_shr = u16_shift;
      u16_convert_load_units_also++;
   }
   ConvertInternalVelInToInternalVelOut(drive);
   ConvertInternalVelOutToInternalVelIn(drive);
}


/*void ConvertInternalToAccDec1000ForVel(int drive)
{
   long s32_fix = 0L;
   unsigned int  u16_shift = 0, u16_unit_conversion_table_index = 0, u16_convert_load_units_also = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error
   
   while (IS_SECONDARY_FEEDBACK_ENABLED >= u16_convert_load_units_also)
   {      
      if (0 == u16_convert_load_units_also) // Motor units convertion
      {
         u16_unit_conversion_table_index = ACC_DEC_FOR_VEL_CONVERSION;
      }
      else // Load units convertion - Dual Loop
      {
         u16_unit_conversion_table_index = LOAD_ACC_DEC_FOR_VEL_CONVERSION;  
      }

      switch(BGVAR(u16_Units_Acc_Dec_Rotary))
      {
         case RPS_TO_SEC_UNITS:
             //user = (internal * 1000 * VLIM_INT * 8000) / (22750 * 2^32) * 8000 / 2^32  = internal * VLIM_INT * 0.0000000000001525031
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(s32_V_Lim_Design),0,(long)703296, (unsigned int)62);
         break;

         case RPM_TO_SEC_UNITS:
             //user = (internal * 1000 * VLIM_INT * 8000 * 60) / (22750 * 2^32) * 8000 / 2^32  = internal * VLIM_INT * 0.000000000009150189
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(s32_V_Lim_Design),0,(long)42197799, (unsigned int)62);
         break;

         case DEG_PER_SEC_2_POWER_UNITS:
             //user = (internal * 1000 * VLIM_INT * 8000 * 360) / (22750 * 2^32) * 8000 / 2^32  = internal * VLIM_INT * 0.000000000054901138
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(s32_V_Lim_Design),0,(long)253186811, (unsigned int)62);
         break;

         case USER_PER_SEC_2_POWER_UNITS:
             //user = (internal * 1000 * (pnum/pden)* VLIM_INT * 8000) / (22750 * 2^32) * 8000 / 2^32  = internal * VLIM_INT * (pden/pnum) * 0.0000000000001525031
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(u32_Scaling_Pdenominator));
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)703296, (unsigned int)62);
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(u32_Scaling_Pnumerator), (unsigned int)0);
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(s32_V_Lim_Design), (unsigned int)0);
         break;
      }

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).u16_unit_conversion_to_user_shr = u16_shift;
      u16_convert_load_units_also++;
   }
}


void ConvertAccDecToInternalForVel(int drive)
{
   long s32_fix = 0L, s32_fix_tmp = 0L;
   unsigned int  u16_shift = 0, u16_shift_tmp = 0, u16_unit_conversion_table_index = 0, u16_convert_load_units_also = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error
   
   while (IS_SECONDARY_FEEDBACK_ENABLED >= u16_convert_load_units_also)
   {   
   
      if (0 == u16_convert_load_units_also) // Motor units convertion
      {
         u16_unit_conversion_table_index = ACC_DEC_FOR_VEL_CONVERSION;
      }
      else // Load units convertion - Dual Loop
      {
         u16_unit_conversion_table_index = LOAD_ACC_DEC_FOR_VEL_CONVERSION;    
      }

      switch(BGVAR(u16_Units_Acc_Dec_Rotary))
      {
         case RPS_TO_SEC_UNITS:
            //internal = user * 22750  * 2^32 / VLIM_INT / 8000 / 1000 *2^32 / 8000 = user * 6557241057451.442176 / VLIM_INT
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
            MultFix64ByFix32ToFix32(&s32_fix,&u16_shift,(long long)0x5F6BA0620AB71327,(unsigned int)20,s32_fix,u16_shift);
         break;

         case RPM_TO_SEC_UNITS:
            //internal = user / 60 * 22750  * 2^32 / VLIM_INT / 8000 / 1000 *2^32 / 8000 = user * 109287350957.5240362 / VLIM_INT
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
            MultFix64ByFix32ToFix32(&s32_fix,&u16_shift,(long long)0x65C8228AB6189CF2,(unsigned int)26,s32_fix,u16_shift);
         break;

         case DEG_PER_SEC_2_POWER_UNITS:
            //internal = user / 360 * 22750  * 2^32 / VLIM_INT / 8000 / 1000 *2^32 / 8000 = user * 18214558492.920672 / VLIM_INT
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
            MultFix64ByFix32ToFix32(&s32_fix,&u16_shift,(long long)0x43DAC1B1CEBB1290,(unsigned int)28,s32_fix,u16_shift);
         break;

         case USER_PER_SEC_2_POWER_UNITS:
            //internal = user / (pnum / pden)* 22750  * 2^32 / VLIM_INT / 8000 / 1000 *2^32 / 8000 = user * 6557241057451.442176 / VLIM_INT /(pnum/pden)
            OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
            MultFix64ByFix32ToFix32(&s32_fix,&u16_shift,(long long)0x5F6BA0620AB71327,(unsigned int)20,s32_fix,u16_shift);
            OneDivS64ToFixU32Shift16(&s32_fix_tmp, &u16_shift_tmp,(long long)BGVAR(u32_Scaling_Pnumerator));
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,s32_fix_tmp,u16_shift_tmp);
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(u32_Scaling_Pdenominator), (unsigned int)0);
         break;
      }

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).u16_unit_conversion_to_internal_shr = u16_shift;
      u16_convert_load_units_also++;
   }

}*/

void ConvertInternalToAccDec1000ForVel(int drive)
{
   long s32_fix = 0L;
   unsigned int  u16_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch(BGVAR(u16_Units_Acc_Dec_Rotary))
   {
      case RPS_TO_SEC_UNITS:
          //user = (internal * 1000 * VLIM_INT * 8000) / (22750 * 2^32) * 8000 / 2^32  = internal * VLIM_INT * 0.0000000000001525031
         MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(s32_V_Lim_Design),0,(long)703296, (unsigned int)62);
      break;

      case RPM_TO_SEC_UNITS:
          //user = (internal * 1000 * VLIM_INT * 8000 * 60) / (22750 * 2^32) * 8000 / 2^32  = internal * VLIM_INT * 0.000000000009150189
         MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(s32_V_Lim_Design),0,(long)42197799, (unsigned int)62);
      break;

      case DEG_PER_SEC_2_POWER_UNITS:
          //user = (internal * 1000 * VLIM_INT * 8000 * 360) / (22750 * 2^32) * 8000 / 2^32  = internal * VLIM_INT * 0.000000000054901138
         MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(s32_V_Lim_Design),0,(long)253186811, (unsigned int)62);
      break;

      case USER_PER_SEC_2_POWER_UNITS:
          //user = (internal * 1000 * (pnum/pden)* VLIM_INT * 8000) / (22750 * 2^32) * 8000 / 2^32  = internal * VLIM_INT * (pden/pnum) * 0.0000000000001525031
         OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(u32_Scaling_Pdenominator));
         MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)703296, (unsigned int)62);
         MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(u32_Scaling_Pnumerator), (unsigned int)0);
         MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(s32_V_Lim_Design), (unsigned int)0);
      break;
   }

   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_VEL_CONVERSION]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_VEL_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;
}


void ConvertAccDecToInternalForVel(int drive)
{
   long s32_fix = 0L, s32_fix_tmp = 0L;
   unsigned int  u16_shift = 0, u16_shift_tmp = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   switch(BGVAR(u16_Units_Acc_Dec_Rotary))
   {
      case RPS_TO_SEC_UNITS:
         //internal = user * 22750  * 2^32 / VLIM_INT / 8000 / 1000 *2^32 / 8000 = user * 6557241057451.442176 / VLIM_INT
         OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
         MultFix64ByFix32ToFix32(&s32_fix,&u16_shift,(long long)0x5F6BA0620AB71327,(unsigned int)20,s32_fix,u16_shift);
      break;

      case RPM_TO_SEC_UNITS:
         //internal = user / 60 * 22750  * 2^32 / VLIM_INT / 8000 / 1000 *2^32 / 8000 = user * 109287350957.5240362 / VLIM_INT
         OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
         MultFix64ByFix32ToFix32(&s32_fix,&u16_shift,(long long)0x65C8228AB6189CF2,(unsigned int)26,s32_fix,u16_shift);
      break;

      case DEG_PER_SEC_2_POWER_UNITS:
         //internal = user / 360 * 22750  * 2^32 / VLIM_INT / 8000 / 1000 *2^32 / 8000 = user * 18214558492.920672 / VLIM_INT
         OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
         MultFix64ByFix32ToFix32(&s32_fix,&u16_shift,(long long)0x43DAC1B1CEBB1290,(unsigned int)28,s32_fix,u16_shift);
      break;

      case USER_PER_SEC_2_POWER_UNITS:
         //internal = user / (pnum / pden)* 22750  * 2^32 / VLIM_INT / 8000 / 1000 *2^32 / 8000 = user * 6557241057451.442176 / VLIM_INT /(pnum/pden)
         OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
         MultFix64ByFix32ToFix32(&s32_fix,&u16_shift,(long long)0x5F6BA0620AB71327,(unsigned int)20,s32_fix,u16_shift);
         OneDivS64ToFixU32Shift16(&s32_fix_tmp, &u16_shift_tmp,(long long)BGVAR(u32_Scaling_Pnumerator));
         MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,s32_fix_tmp,u16_shift_tmp);
         MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(u32_Scaling_Pdenominator), (unsigned int)0);
      break;
   }

   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_VEL_CONVERSION]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[ACC_DEC_FOR_VEL_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;

}


void ConvertInternalToAccDec1000ForPos(int drive)
{
   // AXIS_OFF;
   long          s32_fix = 1LL, s32_pitch_factor_fix = 1000L;
   unsigned int  u16_shift = 0, u16_pitch_factor_shift = 0, u16_Actual_Units = 0, u16_convert_load_units_also = 0;
   unsigned int u16_motor_load_type = 0,  u16_unit_conversion_table_index = 0;
   unsigned long u32_pitch = 0L;
   
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   while (IS_SECONDARY_FEEDBACK_ENABLED >= u16_convert_load_units_also)
   {
      s32_pitch_factor_fix = 1000L;
	   u16_pitch_factor_shift = 0;
      s32_fix = 1L;
      u16_shift = 0;      
   
      if (0 == u16_convert_load_units_also) // Motor units convertion
      {
         u16_motor_load_type = BGVAR(u16_MotorType);
         u32_pitch = BGVAR(u32_Mpitch);
         u16_unit_conversion_table_index = ACC_DEC_FOR_POS_CONVERSION;
      }
      else // Load units convertion - Dual Loop
      {
         u16_motor_load_type = LOAD_TYPE;
         u32_pitch = 1000;
         u16_unit_conversion_table_index = LOAD_ACC_DEC_FOR_POS_CONVERSION;  
      }
      
      if (u16_motor_load_type == ROTARY_MOTOR)  //rotary motor
      {
         u16_Actual_Units = BGVAR(u16_Units_Acc_Dec_Rotary);  // actual units are one of the rotary units
      }
      else // linear motor
      {
         u16_Actual_Units = RPS_TO_SEC_UNITS;  //since all linear units are per second we choose the rotary unit per second ^ 2
                                                 //plus mpitch factor

         switch(BGVAR(u16_Units_Acc_Dec_Linear))// set mpicth factor according to current linear units
         {

            case UM_TO_SEC_2_POWER_UNITS: // u32_Mpitch * 1000
            break;

            case MM_TO_SEC_2_POWER_UNITS: // u32_Mpitch
                s32_pitch_factor_fix = (long)u32_pitch;
            break;

            case USER_TO_SEC_2_POWER_UNITS:
            break;
         }
       }

      switch(u16_Actual_Units)
      {
         case RPS_TO_SEC_UNITS:
             
            if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) 
             //user = (internal * 1000 * 4000^2) / (2^32) / (2^32) * MPITCH = internal * 0.0000000000008673617379884 * MPITCH
            MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_pitch_factor_fix,u16_pitch_factor_shift,(long)2047999999, (unsigned int)71);//changed since mpitch changed to decimal
            else
               //user = (internal * 1000 * 8000^2) / (2^32) / (2^32) * MPITCH = internal * 0.0000000000008673617379884 * MPITCH
               MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_pitch_factor_fix,u16_pitch_factor_shift,(long)2047999999, (unsigned int)69);//changed since mpitch changed to decimal
         break;

         case RPM_TO_SEC_UNITS:
               //user = (internal * 1000 * 60 * 8000^2) / (2^32) / (2^32)  = internal * 0.0000000520417042793042
            s32_fix = 1875000000;
               u16_shift = 53;
              if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) u16_shift +=2; 
         break;

         case DEG_PER_SEC_2_POWER_UNITS:
             //user = (internal * 1000 * 360 * 8000^2) / (2^32) / (2^32)  = internal * 0.0000003122502256758252
            s32_fix = 1406250000;
            u16_shift = 50;
             if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) u16_shift +=2; 
         break;

         case USER_PER_SEC_2_POWER_UNITS:
            //user = (internal * 1000 * (pnum/pden) * 8000^2) / (2^32) / (2^32)  = internal * 0.0000000008673617379884 * (pden/pnum)
            OneDivS64ToFixU32Shift16((long*)&s32_fix, &u16_shift,(long long)BGVAR(u32_Scaling_Pdenominator));
            MultFixS32ByFixS32ToFixS32((long*)&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(u32_Scaling_Pnumerator), (unsigned int)0);
            if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC)
            MultFixS32ByFixS32ToFixS32((long*)&s32_fix, &u16_shift, s32_fix,u16_shift,(long)2000000000, (unsigned int)61);
            else 
               MultFixS32ByFixS32ToFixS32((long*)&s32_fix, &u16_shift, s32_fix,u16_shift,(long)2000000000, (unsigned int)59);
         break;
      }

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).s64_unit_conversion_to_user_fix = s32_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).u16_unit_conversion_to_user_shr = u16_shift;
      u16_convert_load_units_also++;
   }
}


void ConvertAccDecToInternalForPos(int drive)
{
   // AXIS_OFF;
   long long     s64_fix = 1LL, s64_fix_tmp = 1LL;
   long          s32_pitch_factor_fix = 1099511627L;//changed since mpitch changed to decimal, and need 1/mpitch
   unsigned int  u16_shift = 0, u16_shift_tmp = 0, u16_pitch_factor_shift = 40, u16_unit_conversion_table_index = 0, u16_convert_load_units_also = 0;
   unsigned int  u16_Actual_Units = 0, u16_motor_load_type = 0;
   unsigned long u32_pitch = 0L;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   while (IS_SECONDARY_FEEDBACK_ENABLED >= u16_convert_load_units_also)
   {
      s32_pitch_factor_fix = 1099511627L;
	   u16_pitch_factor_shift = 40;
      u16_shift = 0;      
   
      if (0 == u16_convert_load_units_also) // Motor units convertion
      {
         u16_motor_load_type = BGVAR(u16_MotorType);
         u32_pitch = BGVAR(u32_Mpitch);
         u16_unit_conversion_table_index = ACC_DEC_FOR_POS_CONVERSION;
      }
      else // Load units convertion - Dual Loop
      {
         u16_motor_load_type = LOAD_TYPE;
         u32_pitch = 1000;
         u16_unit_conversion_table_index = LOAD_ACC_DEC_FOR_POS_CONVERSION;    
      }
      if (u16_motor_load_type == ROTARY_MOTOR)  //rotary motor
      {
         u16_Actual_Units = BGVAR(u16_Units_Acc_Dec_Rotary);  // actual units are one of the rotary units
      }
      else // linear motor
      {
         u16_Actual_Units = RPS_TO_SEC_UNITS;  //since all linear units are per second we choose the rotary unit per second ^ 2
                                                 //plus mpitch factor

         switch(BGVAR(u16_Units_Acc_Dec_Linear))// set mpicth factor according to current linear units
         {
            case UM_TO_SEC_2_POWER_UNITS: // 1 / (u32_Mpitch * 1000)
            break;

            case MM_TO_SEC_2_POWER_UNITS: // 1 / u32_Mpitch
               OneDivS64ToFixU32Shift16(&s32_pitch_factor_fix, &u16_pitch_factor_shift,(long long)u32_pitch);
            break;

            case USER_TO_SEC_2_POWER_UNITS:
            break;
         }
      }

      switch(u16_Actual_Units)
      {
         case RPS_TO_SEC_UNITS:
            if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) 
            //internal = user * 2^32 / 4000^2 / 1000 * 2^32 / MPITCH_FACTOR  = user * 1152921504606.846976 / MPITCH_FACTOR
               MultFix64ByFix32ToFix64(&s64_fix, &u16_shift,(long long)4835703278458516698, (unsigned int)22,s32_pitch_factor_fix,u16_pitch_factor_shift);
            else
                //internal = user * 2^32 / 8000^2 / 1000 * 2^32 / MPITCH_FACTOR  = user * 1152921504606.846976 / MPITCH_FACTOR
               MultFix64ByFix32ToFix64(&s64_fix, &u16_shift,(long long)4835703278458516698, (unsigned int)24,s32_pitch_factor_fix,u16_pitch_factor_shift);
         break;

         case RPM_TO_SEC_UNITS:
            //internal = user * 2^32 / 8000^2 / 1000 * 2^32 / 60  = user * 19215358.4101141162
            s64_fix = 5281877500950955840;
            u16_shift = 40;
            if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) u16_shift-=2; 
         break;

         case DEG_PER_SEC_2_POWER_UNITS:
            //internal = user * 2^32 / 8000^2 / 1000 * 2^32 / 360  = user * 3202559.735019019377
            s64_fix = 7042503334601274453;
            u16_shift = 43;
            if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC) u16_shift-=2; 
         break;

         case USER_PER_SEC_2_POWER_UNITS:
            //internal = user * 2^32 / 8000^2 / 1000 * 2^32 / (pnum / pden) = user * 1152921504.606846976 / (pnum / pden)
            OneDivS64ToFixU32Shift16((long*)&s64_fix_tmp, &u16_shift_tmp,(long long)BGVAR(u32_Scaling_Pnumerator));
            if (VAR(AX0_u16_Pos_Loop_Tsp) == TSP_250_USEC)
               MultFixS32ByFixS32ToFixS32((long*)&s64_fix, &u16_shift, s64_fix,u16_shift,(long)4951760157141521100,(unsigned int)32);
            else
               MultFixS32ByFixS32ToFixS32((long*)&s64_fix, &u16_shift, s64_fix,u16_shift,(long)4951760157141521100,(unsigned int)34);
            MultFixS32ByFixS32ToFixS32((long*)&s64_fix, &u16_shift, s64_fix,u16_shift,BGVAR(u32_Scaling_Pdenominator),(unsigned int)0);
         break;
      }

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).s64_unit_conversion_to_internal_fix = s64_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).u16_unit_conversion_to_internal_shr = u16_shift;
      u16_convert_load_units_also++;
   }
}


void ConvertInternalToPos1000(int drive)
{
   long long     s64_fix = 1LL;
   unsigned int  u16_shift = 0, u16_pitch_factor_shift = 0, u16_Actual_Units = 0, u16_convert_load_units_also = 0;
   unsigned int  u16_motor_load_type = 0, u16_motor_load_enc_interpolation = 0, u16_position_conversion_table_index = 0, u16_encres_conversion_table_index = 0;
   unsigned long u32_pitch = 0L, u32_counts_per_rev = 0L, u32_enc_res = 0L;
   long          s32_pitch_factor_fix = 1000L;//changed since mpitch changed to decimal

   // AXIS_OFF;   
   REFERENCE_TO_DRIVE;
  
   while (IS_SECONDARY_FEEDBACK_ENABLED >= u16_convert_load_units_also)
   {
      s32_pitch_factor_fix = 1000L;
      s64_fix = 1LL;
      u16_shift = 0;
      
      if (0 == u16_convert_load_units_also) // Motor units convertion
      {
         u16_motor_load_type = BGVAR(u16_MotorType);

         u32_pitch = BGVAR(u32_Mpitch);
         u16_motor_load_enc_interpolation = BGVAR(u16_Motor_Enc_Interpolation);
         u16_position_conversion_table_index = POSITION_CONVERSION;
         u16_encres_conversion_table_index = POSITION_MENCRES_CONVERSION;
         u32_counts_per_rev = LVAR(AX0_u32_Counts_Per_Rev);
         u32_enc_res = BGVAR(u32_User_Motor_Enc_Res);
      }
      else // Load units convertion - Dual Loop
      {
         u16_motor_load_type = LOAD_TYPE;
         u32_pitch = s32_pitch_factor_fix;
         u16_motor_load_enc_interpolation = BGVAR(u16_Load_Enc_Interpolation);
         u16_position_conversion_table_index = LOAD_POSITION_CONVERSION; 
         u16_encres_conversion_table_index = POSITION_SECONDARY_ENCRES_CONVERSION;        
         u32_counts_per_rev = LVAR(AX0_u32_Counts_Per_Rev_2);
         u32_enc_res = BGVAR(u32_User_Sec_Enc_Res);
      }

      if (u16_motor_load_type == ROTARY_MOTOR)  //rotary motor
      {
         u16_Actual_Units = BGVAR(u16_Units_Pos_Rotary);  // actual units are one of the rotary units
      }
      else // linear motor
      {
         u16_Actual_Units = ROT_COUNTS_UNITS;  //since all linear units are per second we choose the rotary counts unit
                                                 //plus mpitch factor

         switch(BGVAR(u16_Units_Pos_Linear))// set mpicth factor according to current linear units
         {

            case PITCH_UNITS: // like rev units
               u16_Actual_Units = REV_UNITS;
            break;

            case LIN_COUNTS_UNITS: // like counts units
               u16_Actual_Units = ROT_COUNTS_UNITS;
            break;

            case UM_UNITS: // u32_Mpitch * 1000
            break;

            case MM_UNITS: // u32_pitch
               u16_Actual_Units = REV_UNITS;
               s32_pitch_factor_fix = u32_pitch;
            break;

            case USER_LIN_UNITS:
            break;
         }
      }

      switch(u16_Actual_Units)
      {
         case REV_UNITS:
            //user = internal / 2^32 * 1000 * MPITCH = user * 0.00000000023283064365386962890625 * MPITCH
            MultFix64ByFix32ToFix32((long*)&s64_fix, &u16_shift, (long long)1073741824, (unsigned int)62,s32_pitch_factor_fix,u16_pitch_factor_shift);//changed since mpitch changed to decimal
         break;

         case ROT_COUNTS_UNITS:
            //user = internal / 2^32  * (counts per rev)* 1000  = 0.00000000023283064365386962890625 * (counts per rev)
            MultFix64ByFix64ToFix64(&s64_fix, &u16_shift, (long long)0x4000000000000000,(unsigned int)94,(long long)((long long)u32_counts_per_rev/(long long)u16_motor_load_enc_interpolation),(unsigned int)0);//changed since mpitch changed to decimal
            MultFix64ByFix64ToFix64(&s64_fix, &u16_shift, s64_fix,u16_shift,(long long)s32_pitch_factor_fix, u16_pitch_factor_shift);
         break;

         case DEG_UNITS:
            //user = internal / 2^32  * 1000 * 360 = user * 0.00008381903171539306640625
            s64_fix = 1474560000;
            u16_shift = 44;
         break;

         case USER_ROT_UNITS:
               //user = internal / 2^32 * (pnum / pden) * 1000 = user * 0.00000023283064365386962890625 * (pnum/pden)
   //       OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(u32_Scaling_Pdenominator));
   //       MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(u32_Scaling_Pnumerator), (unsigned int)0);
   //       MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)2097152000, (unsigned int)53);
         break;
      }

      BGVAR(Unit_Conversion_Table[u16_position_conversion_table_index]).s64_unit_conversion_to_user_fix = s64_fix;
      BGVAR(Unit_Conversion_Table[u16_position_conversion_table_index]).u16_unit_conversion_to_user_shr = u16_shift;
      
      //user(MENCRES) = internal / 2^32 / 1000 * MENCRES * 4 = user * 0.00000000023283064365386962890625 * MENCRES * 4
      MultFix64ByFix32ToFix64(&s64_fix, &u16_shift,(long long)1073741824, (unsigned int)62, u32_enc_res,0);
      //for PCOM feature
      BGVAR(Unit_Conversion_Table[u16_encres_conversion_table_index]).s64_unit_conversion_to_user_fix = s64_fix;
      BGVAR(Unit_Conversion_Table[u16_encres_conversion_table_index]).u16_unit_conversion_to_user_shr = u16_shift - 2; // (MENCRES * 4)
      
      u16_convert_load_units_also++;
   }
}


void ConvertPosToInternal(int drive)
{
   long long     s64_fix = 1LL, s64_fix1 = 1LL;
   unsigned int  u16_shift = 0, u16_shift1 = 0, u16_pitch_factor_shift = 72, u16_Actual_Units = 0, u16_convert_load_units_also = 0;
   unsigned int  u16_motor_load_type = 0, u16_motor_load_enc_interpolation = 0, u16_position_conversion_table_index = 0, u16_encres_conversion_table_index = 0;
   unsigned long u32_pitch = 0L, u32_counts_per_rev = 0L, u32_enc_res = 0L;
   long long     s64_pitch_factor_fix = 4722366482867027968LL; //since mpitch changed to decimal and we'll need 1/mpitch

   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
   
   while (IS_SECONDARY_FEEDBACK_ENABLED >= u16_convert_load_units_also)
   {
      s64_pitch_factor_fix = 4722366482867027968LL;
      u16_pitch_factor_shift = 72;
      s64_fix = 1LL;
      u16_shift = 0;
      
      if (0 == u16_convert_load_units_also) // Motor units convertion
      {
         u16_motor_load_type = BGVAR(u16_MotorType);
         u32_pitch = BGVAR(u32_Mpitch);
         u16_motor_load_enc_interpolation = BGVAR(u16_Motor_Enc_Interpolation);
         u16_position_conversion_table_index = POSITION_CONVERSION;
         u16_encres_conversion_table_index = POSITION_MENCRES_CONVERSION;
         u32_counts_per_rev = LVAR(AX0_u32_Counts_Per_Rev);
         u32_enc_res = BGVAR(u32_User_Motor_Enc_Res);
      }
      else // Load units convertion - Dual Loop
      {
         u16_motor_load_type = LOAD_TYPE;
         u32_pitch = 1000;
         u16_motor_load_enc_interpolation = BGVAR(u16_Load_Enc_Interpolation);
         u16_position_conversion_table_index = LOAD_POSITION_CONVERSION;  
         u16_encres_conversion_table_index = POSITION_SECONDARY_ENCRES_CONVERSION;       
         u32_counts_per_rev = LVAR(AX0_u32_Counts_Per_Rev_2);
         u32_enc_res = BGVAR(u32_User_Sec_Enc_Res);
      }

      if (u16_motor_load_type == ROTARY_MOTOR)  //rotary motor/Load
      {
         u16_Actual_Units = BGVAR(u16_Units_Pos_Rotary);  // actual units are one of the rotary units
      }
      else // linear motor
      {
         u16_Actual_Units = ROT_COUNTS_UNITS;  //since all linear units are per second we
                                       // choose the rotary counts unit plus mpitch factor

         switch(BGVAR(u16_Units_Pos_Linear))// set mpicth factor according to current linear units
         {
            case PITCH_UNITS: // like rev units
               u16_Actual_Units = REV_UNITS;
            break;

            case LIN_COUNTS_UNITS: // like counts units
               u16_Actual_Units = ROT_COUNTS_UNITS;
            break;

            case UM_UNITS: // 1 / (u32_Mpitch * 1000)
            break;

               case MM_UNITS: // 1 / u32_Mpitch
                  u16_Actual_Units = REV_UNITS;
                  OneDivS64ToFixU64Shift16(&s64_pitch_factor_fix, &u16_pitch_factor_shift,(long long)u32_pitch);
               break;

            case USER_LIN_UNITS:
            break;
         }
      }

      switch(u16_Actual_Units)
      {
         case REV_UNITS:
            //internal = user * 2^32  / 1000 / MPITCH = user * 4294967296 / MPITCH
            MultFix64ByFix64ToFix64(&s64_fix,&u16_shift,(long long)0x4000000000000000,(unsigned int)30,s64_pitch_factor_fix,u16_pitch_factor_shift);//changed since mpitch changed to decimal
         break;

         case ROT_COUNTS_UNITS:
            //internal = user * 2^32  / 1000 / (counts per rev) = user * 4294967296 / (counts per rev)
            OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift,(long long)u32_counts_per_rev/(long long)u16_motor_load_enc_interpolation);
            MultFix64ByFix64ToFix64(&s64_fix,&u16_shift,(long long)0x4000000000000000, (unsigned int)30,s64_fix,u16_shift);//changed since mpitch changed to decimal
            MultFix64ByFix64ToFix64(&s64_fix,&u16_shift,s64_fix,u16_shift, s64_pitch_factor_fix,u16_pitch_factor_shift);
         break;

         case DEG_UNITS:
            //internal = user * 2^32  / 1000 / 360 = user * 11930.464711
            s64_fix = 0x5D34EDEE99A62ED3;
            u16_shift = 49;
         break;

         case USER_ROT_UNITS:
            //internal = user * 2^32 / (pnum / pden) / 1000 = user * 4294967.296 / (pnum/pden)
   //       OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(u32_Scaling_Pnumerator));
   //       MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)BGVAR(u32_Scaling_Pdenominator), (unsigned int)0);
   //       MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)1099511628, (unsigned int)8);
         break;
      }

      BGVAR(Unit_Conversion_Table[u16_position_conversion_table_index]).s64_unit_conversion_to_internal_fix = s64_fix;
      BGVAR(Unit_Conversion_Table[u16_position_conversion_table_index]).u16_unit_conversion_to_internal_shr = u16_shift;
      
      //internal(MENCRES) = user * 2^32  / 1000 = user * 4294967296 MENCRES / 4
      OneDivS64ToFixU64Shift16(&s64_fix1, &u16_shift1,(long long)u32_enc_res);
      MultFix64ByFix32ToFix64(&s64_fix, &u16_shift,(long long)0x4000000000000000, 30,s64_fix1,u16_shift1);
      //for PCOM feature
      BGVAR(Unit_Conversion_Table[u16_encres_conversion_table_index]).s64_unit_conversion_to_internal_fix = s64_fix;
      BGVAR(Unit_Conversion_Table[u16_encres_conversion_table_index]).u16_unit_conversion_to_internal_shr = u16_shift + 2; //(MENCRES / 4)
      
      u16_convert_load_units_also++;
   } 
}


/**********************************************************
 Function Name: ConvertCountsToInternal
 Description: Converts from internal to counts and the other way around
 This conversion is used by the modulo function regardless
 of the position units currently selected
 Author: Lionel
**********************************************************/
void ConvertCountsToInternal(int drive)
{
   long long     s64_fix = 1LL, s64_half_for_rounding = 0LL;
   unsigned int  u16_shift = 0, u16_convert_load_units_also = 0, u16_motor_load_enc_interpolation = 0, u16_unit_conversion_table_index = 0, u16_temp_time = 0;
   unsigned long u32_counts_per_rev = 0;

   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   while (IS_SECONDARY_FEEDBACK_ENABLED >= u16_convert_load_units_also)
   {
      s64_fix = 1LL;
      u16_shift = 0;
   
      if (0 == u16_convert_load_units_also) // Motor units convertion
      {
         u16_motor_load_enc_interpolation = BGVAR(u16_Motor_Enc_Interpolation);
         u16_unit_conversion_table_index = POSITION_INTERNAL_COUNTS_CONVERSION;
         u32_counts_per_rev = LVAR(AX0_u32_Counts_Per_Rev);
      }
      else // Load units convertion - Dual Loop
      {
         u16_motor_load_enc_interpolation = BGVAR(u16_Load_Enc_Interpolation);
         u16_unit_conversion_table_index = LOAD_POSITION_INTERNAL_COUNTS_CONVERSION;         
         u32_counts_per_rev = LVAR(AX0_u32_Counts_Per_Rev_2);
      }
      //internal = counts * 2^32 / (1000*(counts per rev))
      OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift,((long long)u32_counts_per_rev * 1000LL)/(long long)u16_motor_load_enc_interpolation);

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).s64_unit_conversion_to_internal_fix = s64_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).u16_unit_conversion_to_internal_shr = u16_shift - 32;

      s64_fix = 1LL;
      u16_shift = 0;

      //user = internal / 2^32  * 1000 * (counts per rev) = 1*2^(-32) * 1000 * (counts per rev)
      MultFix64ByFix64ToFix64(&s64_fix,&u16_shift,1000LL, (unsigned int)0,(long long)(u32_counts_per_rev/u16_motor_load_enc_interpolation),(unsigned int)32);

      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).s64_unit_conversion_to_user_fix = s64_fix;
      BGVAR(Unit_Conversion_Table[u16_unit_conversion_table_index]).u16_unit_conversion_to_user_shr = u16_shift;
      
      if (0 == u16_convert_load_units_also)
      {
         //update also RT parameters (will be used for MFB RT Fal function)
         s64_fix += 0x40000000LL;  // rounding
         s64_fix += 0x40000000LL;  // rounding
         LVAR(AX0_s32_Mfb_Pos_Fix_To_User) = (long)(s64_fix >> 32);
         VAR(AX0_u16_Mfb_Pos_Shr_To_User) = u16_shift - 32;

         //set half for rounding
         do {
            u16_temp_time = Cntr_3125;
            LLVAR(AX0_u32_Mfb_Pos_Half_For_Rounding_To_User_Lo) = 0LL;
         } while (u16_temp_time != Cntr_3125);
         if (VAR(AX0_u16_Mfb_Pos_Shr_To_User) > 0)
         {
            s64_half_for_rounding = (1LL << ((long long)VAR(AX0_u16_Mfb_Pos_Shr_To_User) - 1));
            do {
               u16_temp_time = Cntr_3125;
               LLVAR(AX0_u32_Mfb_Pos_Half_For_Rounding_To_User_Lo) = s64_half_for_rounding;
            } while (u16_temp_time != Cntr_3125);
         }
      }
         
      u16_convert_load_units_also++;
   }
}


/**********************************************************
 Function Name: ConvertInternalVelInToInternalVelOut
 Description: Converts from internal in loop velocity to internal out loop velocity.

 Author: Itai.
 Algorithm: Makes the conversion in to steps:
 1) Convert from in loop internal velocity to in loop user velocity
 - since in loop user velocity = out loop user velocity
 2) Convert from out loop user velocity to out loop internal velocity
 Revisions:
**********************************************************/
void ConvertInternalVelInToInternalVelOut(int drive)
{
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   //Convert from internal in the loop to internal out of the loop
   MultFixS32ByFixS32ToFixS32(&LVAR(AX0_s32_In_To_Out_Vel_Fix), &VAR(AX0_u16_In_To_Out_Vel_Shr), (long)1546565806,(unsigned int)45,(long)BGVAR(s32_V_Lim_Design), (unsigned int)0);
}


void ConvertInternalVelOutToInternalVelIn(int drive)
{
   // AXIS_OFF;

   long s32_Out_To_In_Vel_Fix_tmp = 1L;
   unsigned int u16_Out_To_In_Vel_Shr_tmp = 0;
   REFERENCE_TO_DRIVE;

   //Convert from internal out of the loop to internal in the loop
    OneDivS64ToFixU32Shift16(&s32_Out_To_In_Vel_Fix_tmp, &u16_Out_To_In_Vel_Shr_tmp,(long long)BGVAR(s32_V_Lim_Design)); // (1 / VLIM)
    MultFixS32ByFixS32ToFixS32(&s32_Out_To_In_Vel_Fix_tmp, &u16_Out_To_In_Vel_Shr_tmp, s32_Out_To_In_Vel_Fix_tmp,u16_Out_To_In_Vel_Shr_tmp,(long)22750, (unsigned int)0); // (* 22750)

   LVAR(AX0_s32_Out_To_In_Vel_Fix) = s32_Out_To_In_Vel_Fix_tmp;
   VAR(AX0_u16_Out_To_In_Vel_Shr) = u16_Out_To_In_Vel_Shr_tmp;
}


void ConvertAnalogIOToInternal(int drive)
{
   long u32_fix = 1L;
   int  u16_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //internal = user / 12.5 * 32767 / 1000 = user * 2.62136
   u32_fix = 1407331934;
   u16_shift = 29;

   BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_internal_fix = (long long)u32_fix;
   BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;
}


void ConvertInternalToAnalogIO1000(int drive)
{
   long u32_fix = 1L;
   int  u16_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

    //user = internal * 1000 * 12.5 / 32767 = internal * 0.38148136844
   u32_fix = 1638450001;
   u16_shift = 32;

   BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).s64_unit_conversion_to_user_fix = (long long)u32_fix;
   BGVAR(Unit_Conversion_Table[ANALOG_IO_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;

   BGVAR(u32_Fb_Analog_Input_Fix) = u32_fix;
   BGVAR(u16_Fb_Analog_Input_Shr) = u16_shift;
}


void ConvertInternalPWMCmdToVolts1000(int drive)
{
    // AXIS_OFF;
   long s32_fix;
   unsigned int u16_shift;
   REFERENCE_TO_DRIVE;

    //user = internal/ax0_pwm_half_period*vbus
   FloatToFixS32Shift16(&s32_fix,&u16_shift,(float)BGVAR(s16_Vbus)/(float)VAR(AX0_s16_Pwm_Half_Period));

   BGVAR(Unit_Conversion_Table[PWM_TO_VOLTS_CONVERSION]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[PWM_TO_VOLTS_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;
}


void ConvertRotLinPosToInternal(int drive)
{
   long long     s64_fix = 1L;
   unsigned int  u16_shift = 0, u16_mpitch_factor_shift = 0;
   long          s32_mpitch_factor_fix = 1L;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
        //internal = user * 2^32  / 1000 = user * 4294967.296
      s64_fix = 0x4189374BC6A7EF9E;
      u16_shift = 40;
   }
   else// linear motor
   {
        //internal = user * 2^32  / 1000 / MPITCH = user * 4294967296 / MPITCH
      OneDivS64ToFixU32Shift16(&s32_mpitch_factor_fix, &u16_mpitch_factor_shift,(long long)BGVAR(u32_Mpitch));
      MultFix64ByFix32ToFix64(&s64_fix,&u16_shift,(long long)0x4000000000000000,(unsigned int)30,s32_mpitch_factor_fix,u16_mpitch_factor_shift);//changed since mpitch changed to decimal
   }

   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION_LIN_ROT]).s64_unit_conversion_to_internal_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION_LIN_ROT]).u16_unit_conversion_to_internal_shr = u16_shift;
}


void ConvertRotLinPosToUser(int drive)
{
    long long     s64_fix = 1LL;
   unsigned int  u16_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
       //user = internal / 2^32 * 1000 = user * 0.00000023283064365386962890625
      s64_fix = 2097152000LL;
      u16_shift = 53;
   }
   else// linear motor
   {
       //user = internal / 2^32 * 1000 = user * 0.00000000023283064365386962890625
      MultFix64ByFix32ToFix32((long*)&s64_fix, &u16_shift,(long long)1073741824, (unsigned int)62,BGVAR(u32_Mpitch),0);//changed since mpitch changed to decimal
   }

   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION_LIN_ROT]).s64_unit_conversion_to_user_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[POSITION_CONVERSION_LIN_ROT]).u16_unit_conversion_to_user_shr = u16_shift;
}


/**********************************************************
 Function Name: ConvertFieldBusCountsToIntern
 Description:
   converts fieldbus counts to drive internal
   the conversion depends on the vlue of parameter u8_Fbscale.
   This parameter represents number of revs from 32 bits.
   e.g. a value of 12 sets 12 bits for number of revs and 20 for mech angle.

   Objects using his conversion:
   6063h: Position actual internal value
   60FCh: Position demand internal value

 Author: Aviv.
 Algorithm:
 Revisions:
**********************************************************/
void ConvertFieldBusCountsToIntern(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(Unit_Conversion_Table[FB_SCALE_CONVERSION]).s64_unit_conversion_to_internal_fix = (2 << BGVAR(u8_Fbscale));
   BGVAR(Unit_Conversion_Table[FB_SCALE_CONVERSION]).u16_unit_conversion_to_internal_shr = 0;
}


/**********************************************************
 Function Name: ConvertInternToFieldBusCounts
 Description:
   converts fieldbus counts to drive internal
   the conversion depends on the vlue of parameter u8_Fbscale.
   This parameter represents number of revs from 32 bits.
   e.g. a value of 12 sets 12 bits for number of revs and 20 for mech angle.

   Objects using his conversion:
   6063h: Position actual internal value
   60FCh: Position demand internal value

 Author: Aviv.
 Algorithm:
 Revisions:
**********************************************************/
void ConvertInternToFieldBusCounts(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(Unit_Conversion_Table[FB_SCALE_CONVERSION]).s64_unit_conversion_to_user_fix = 1;
   BGVAR(Unit_Conversion_Table[FB_SCALE_CONVERSION]).u16_unit_conversion_to_user_shr = BGVAR(u8_Fbscale);
}


/**********************************************************
 Function Name: CalcFieldBusUserToIntern
 Description:
   calculates CanOpen units to internal.
   This conversion uses objects 0x6091 and 0x6092 and uses 0x6086 as max 32 bits

   Objects using this conversion:
      607Ah target position INTEGER32 R/W
      607Dh software position limit INTEGER32 R/W
      6081h profile velocity UNSIGNED32 R/W
      6083h profile acceleration UNSIGNED32 R/W
      6084h profile deceleration UNSIGNED32 R/W

 Author: Aviv.
 Algorithm:fix,shift = (0x608F * 0x6091) / 0x6092
 Revisions:
**********************************************************/
void CalcFieldBusUserToIntern(int drive)
{
   long s32_fix;
   long long s64_fix, s64_fix1, s64_denum, s64_num;
   unsigned int u16_temp_time, u16_shift, u16_shift1;
   // AXIS_OFF;

/*
 *->calculate position units
*/
//   Internal = 2^32 / [(PNUM / PDEN) * (FBGMS / FBGDS)] =
//            = (2^32 * PDEN * FBGDS) / (PNUM * FBGMS)

   OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift,(long long)((long long)BGVAR(u32_Fb_Gear_Motor_Shaft_Revs)*(long long)BGVAR(u32_Scaling_Pnumerator)));
   MultFix64ByFix64ToFix64(&s64_fix, &u16_shift,s64_fix,u16_shift,(long long)((long long)BGVAR(u32_Fb_Gear_Driving_Shaft_Revs)*(long long)BGVAR(u32_Scaling_Pdenominator)),(int)0);

   if (u16_shift >= 32)    // equivalent to mult by 2^32
   {
      u16_shift -= 32;
   }
   else
   {
      u16_shift = 0;
      s64_fix = 0x7fffffffffffffff;
   }

   if (u16_shift > 127)
      u16_shift = 127;

   //set position units
   BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_internal_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;

   //update also RT parameters for later use
   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Pos_Fix_To_Internal_Lo) = s64_fix;
   } while (u16_temp_time != Cntr_3125);
   VAR(AX0_u16_Pos_Shr_To_Internal) = u16_shift;

   // Check if conversion is an integer so it can be used to increase the accuracy of the conversion from CAN to internal by using number of turns and residue in one turn
   BGVAR(u16_Scaling_Is_Integer) = 0;
   s64_num = (long long)BGVAR(u32_Fb_Gear_Driving_Shaft_Revs)*(long long)BGVAR(u32_Scaling_Pdenominator);
   s64_denum = (long long)BGVAR(u32_Fb_Gear_Motor_Shaft_Revs)*(long long)BGVAR(u32_Scaling_Pnumerator);
   if (((s64_denum / s64_num)*s64_num) == s64_denum)
   {
      BGVAR(s64_Scaling_Denominator) = s64_denum / s64_num;
      BGVAR(u16_Scaling_Is_Integer) = 1;
   }

/* ->calculate velocity units out of loop for PTP - Motion Task like (position / 4000 for velocity in position units.)
//   Internal = (2^32 / 4000) / [(PNUM / PDEN) * (FBGMS / FBGDS)] =
//            = (1073741.824 * PDEN * FBGDS) / (PNUM * FBGMS)
*/
   FloatToFixS32Shift16(&s32_fix,&u16_shift,(float)((float)1073741.824 * (( (float)BGVAR(u32_Fb_Gear_Driving_Shaft_Revs) / (float)BGVAR(u32_Fb_Gear_Motor_Shaft_Revs) ) ) /
                            ( (float)BGVAR(u32_Scaling_Pnumerator) / (float)BGVAR(u32_Scaling_Pdenominator) )) );


   //set velocity out of loop units for PTP (position sample time 4000)
   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix = s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;

   if(VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)//ITAI -  Position 125 usec	 
      BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr++;
    

   //set velocity out of loop units for velocity loop (velocity sample time 8000)
   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix = s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift+1;

    //update also RT parameters for later use
   LVAR(AX0_s32_Velocity_Out_Loop_To_Int_Fix) = s32_fix;
   VAR(AX0_u16_Velocity_Out_Loop_To_Int_Shr)  = u16_shift+1;

/*
 *->calculate acc and dec units for PTP - Motion Task like (position / 4000^2 for velocity in position units.)
*/
   //internal = user * (2^32 / 4000^2) * 2^32 = user * 1152921504606.8516910454718255235 * (pden/pnum) * (GearM/GearD)
   OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift,(long long)BGVAR(u32_Scaling_Pnumerator));
   MultFix64ByFix64ToFix64(&s64_fix, &u16_shift,s64_fix,u16_shift,0x431BDE82D7B6821B,(unsigned int)22);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift,s64_fix,u16_shift,BGVAR(u32_Scaling_Pdenominator),(unsigned int)0);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift,s64_fix,u16_shift,BGVAR(u32_Fb_Gear_Driving_Shaft_Revs),(unsigned int)0);
   OneDivS64ToFixU64Shift16(&s64_fix1, &u16_shift1,(long long)BGVAR(u32_Fb_Gear_Motor_Shaft_Revs));
   MultFix64ByFix64ToFix64(&s64_fix, &u16_shift,s64_fix,u16_shift,s64_fix1,u16_shift1);


   //set acc/dec units for PTP (position sample time 4000 thus /4000^2)
   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_internal_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;

   if(VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)//ITAI -  Position 125 usec	
      BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_internal_shr += 2;

   //set acc/dec units for PTP (position sample time 4000 thus /4000^2)
   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_USER_CONVERSION]).s64_unit_conversion_to_internal_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_USER_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;

   if(VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)//ITAI -  Position 125 usec	
      BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_USER_CONVERSION]).u16_unit_conversion_to_internal_shr += 2;

/*
 *->calculate velocity in the loop conversion
*/
   ConvertFbCanOpenVelToInternalInLoop(drive);
}


/**********************************************************
 Function Name: ConvertFbCanOpenVelToInternalInLoop
 Description:
   calculates CanOpen units to velocity internal in the loop.

 Author: Aviv.
 Algorithm:fix,shift = (0x608F * 0x6091) / 0x6092
 Revisions:
**********************************************************/
void ConvertFbCanOpenVelToInternalInLoop(int drive)
{
   long          s32_fix,s32_fix_tmp = 1L;
   unsigned int  u16_shift,u16_shift_tmp = 0;
   long long     s64_half_for_rounding = 0LL;
   unsigned int  u16_temp_time;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;
// long          s32_mpitch_factor_fix = 1L;
// unsigned int  u16_mpitch_factor_shift = 0;

/*
  user position units are allready scaled to 2^32 = 1 rev.
  Also, velocity units out of the loop  are scalred to sample time.
  Thus I'll use (need to make sure always this comes after out of loop velocity units are set):
  s16_intern = (s32_CanOpen_user_vel) * (22750/VLIM_INT)
*/

   s32_fix_tmp   = (long)BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix;
   u16_shift_tmp = BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr;

   //need to consider here linear motor conversions
   OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)22750, (unsigned int)0);
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,s32_fix_tmp, u16_shift_tmp);

   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;

   //update also RT parameters for later use
   LVAR(AX0_s32_Velocity_In_Loop_Fix) = s32_fix;
   VAR(AX0_u16_Velocity_In_Loop_Shr)  = u16_shift;

   //set half for rounding
   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Vel_In_Loop_Half_For_Round_Lo) = 0LL;
   } while (u16_temp_time != Cntr_3125);
   if (VAR(AX0_u16_Velocity_In_Loop_Shr) > 0)
   {
       s64_half_for_rounding = (1LL << ((long long)VAR(AX0_u16_Velocity_In_Loop_Shr) - 1));
   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Vel_In_Loop_Half_For_Round_Lo) = s64_half_for_rounding;
   } while (u16_temp_time != Cntr_3125);
   }

   // fix = 0x5C2EB8AE = 1546565806 ,   shift=45,
   //   s32_CanOpen_user_vel = s16_intern * VLIM_INT / 22750
   // Convert velocity-in-loop units to canopen vel units

   s32_fix_tmp   = (long)BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix;
   u16_shift_tmp = BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr;

   s32_fix   = 1;
   u16_shift = 0;

   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,BGVAR(s32_V_Lim_Design), (unsigned int)0);
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)0x5C2EB8AE, (unsigned int)45);

   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,s32_fix_tmp, u16_shift_tmp);

   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_IN_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;
}


/**********************************************************
 Function Name: CalcInternToFieldBusUser
 Description:
   calculates internal to CanOpen units.
   This conversion uses objects 0x6091 and 0x6092 and uses 0x6086 as max 32 bits

   Objects using this conversion:
      607Ah target position INTEGER32 R/W
      607Dh software position limit INTEGER32 R/W
      6081h profile velocity UNSIGNED32 R/W
      6083h profile acceleration UNSIGNED32 R/W
      6084h profile deceleration UNSIGNED32 R/W

 Author: Aviv.
 Algorithm:
 Revisions:
**********************************************************/
void CalcInternToFieldBusUser(int drive)
{
   // AXIS_OFF;
   long s32_fix;
   unsigned int u16_temp_time, u16_shift;
   long long s64_half_for_rounding = 0LL, s64_fix, s64_temp;
   REFERENCE_TO_DRIVE;

/*
 *->calculate position units
*/
//   User = [(PNUM / PDEN) * (FBGMS / FBGDS)] / 2^32 =
//        =  (PNUM * FBGMS) / (2^32 * PDEN * FBGDS)

   OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift,(long long)((long long)BGVAR(u32_Fb_Gear_Driving_Shaft_Revs)*(long long)BGVAR(u32_Scaling_Pdenominator)));
   MultFix64ByFix64ToFix64(&s64_fix, &u16_shift,s64_fix,u16_shift,(long long)((long long)BGVAR(u32_Fb_Gear_Motor_Shaft_Revs)*(long long)BGVAR(u32_Scaling_Pnumerator)),(int)0);

   u16_shift += 32;    // equivalent to div by 2^32
   if (u16_shift > 127)
   {
      u16_shift = 127;
   }

    //set position units
   BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).s64_unit_conversion_to_user_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_POSITION_USER_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;

   //update also RT parameters for later use
   s64_fix += 0x40000000LL;  // rounding
   s64_fix += 0x40000000LL;  // rounding
   LVAR(AX0_s32_Pos_Fix_To_User) = (long)(s64_fix >> 32);
   VAR(AX0_u16_Pos_Shr_To_User) = u16_shift - 32;

   //set half for rounding
   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo) = 0LL;
   } while (u16_temp_time != Cntr_3125);
   if (VAR(AX0_u16_Pos_Shr_To_User) > 0)
   {
      s64_half_for_rounding = (1LL << ((long long)VAR(AX0_u16_Pos_Shr_To_User) - 1));
      do {
         u16_temp_time = Cntr_3125;
         LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo) = s64_half_for_rounding;
      } while (u16_temp_time != Cntr_3125);
   }

/*
 *->calculate velocity units out of loop for PTP - Motion Task like (position / 4000 for velocity in position units.)
*/
   //User =  [(PNUM / PDEN) * (FBGMS / FBGDS)] / (2^32 / 4000) =
   //     =   (PNUM * FBGMS) / (1073741.824 * PDEN * FBGDS)

   FloatToFixS32Shift16(&s32_fix,&u16_shift,(float)(( (float)BGVAR(u32_Scaling_Pnumerator) / (float)BGVAR(u32_Scaling_Pdenominator) ) /
                              ( (float)1073741.824 * ( (float)BGVAR(u32_Fb_Gear_Driving_Shaft_Revs) / (float)BGVAR(u32_Fb_Gear_Motor_Shaft_Revs)) ) ) );

   //set velocity out of loop units for PTP (position sample time 4000)
   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix = s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;

   if(VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)//ITAI -  Position 125 usec	
      BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_PTP_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr--;


   //set velocity out of loop units for velocity loop (velocity sample time 8000)
   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).s64_unit_conversion_to_user_fix = s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_VELOCITY_USER_OUT_OF_LOOP_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift - 1;

   //update also RT parameters for later use
   LVAR(AX0_s32_Velocity_Out_Loop_To_User_Fix) = s32_fix;
   VAR(AX0_u16_Velocity_Out_Loop_To_User_Shr)  = u16_shift - 1;

   //set half for rounding
   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Vel_Out_Loop_Half_For_Round_To_User_Lo) = 0LL;
   } while (u16_temp_time != Cntr_3125);
   if (VAR(AX0_u16_Velocity_Out_Loop_To_User_Shr) > 0)
   {
       s64_half_for_rounding = (1LL << ((long long)VAR(AX0_u16_Velocity_Out_Loop_To_User_Shr) - 1));
       LLVAR(AX0_u32_Vel_Out_Loop_Half_For_Round_To_User_Lo) = (long)s64_half_for_rounding;
   }

/*
 *->calculate acc and dec units for PTP - Motion Task like (position * 4000^2 for velocity in position units.)
*/
    //user = (internal * 4000^2) / (2^32) / (2^32) = internal * 0.0000000000008673617379884
    FloatToFixS32Shift16(&s32_fix,&u16_shift,(float)(0.0000000000008673617379884 * ((float)((float)BGVAR(u32_Scaling_Pnumerator) / (float)BGVAR(u32_Scaling_Pdenominator)) /
                                                                       (float)((float)BGVAR(u32_Fb_Gear_Driving_Shaft_Revs) / (float)BGVAR(u32_Fb_Gear_Motor_Shaft_Revs)))));

   //set acc/dec units for PTP (position sample time 4000 thus /4000^2)
   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).s64_unit_conversion_to_user_fix = s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;

   if(VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)//ITAI -  Position 125 usec
      BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_PTP_USER_CONVERSION]).u16_unit_conversion_to_user_shr -=2;	

   //set acc/dec units for PTP (position sample time 4000 thus /4000^2)
   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_USER_CONVERSION]).s64_unit_conversion_to_user_fix = s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_USER_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;

   if(VAR(AX0_u16_Pos_Loop_Tsp) == TSP_125_USEC)//ITAI -  Position 125 usec
      BGVAR(Unit_Conversion_Table[FB_CAN_ACC_DEC_USER_CONVERSION]).u16_unit_conversion_to_user_shr -=2; 

/*
 *->calculate velocity in the loop conversion
*/
// ConvertFbCanOpenVelToInternalInLoop(drive);

   /* calculate one motor revolution in field bus units */
   do {
      u16_temp_time = Cntr_3125;
      s64_temp = LLVAR(AX0_u32_Pos_Half_For_Rounding_To_User_Lo);
   } while (u16_temp_time != Cntr_3125);
   LVAR(AX0_u32_FB_One_Rev) =
   (unsigned long)((unsigned long long)((unsigned long long)((unsigned long long)0x100000000 * (unsigned long long)LVAR(AX0_s32_Pos_Fix_To_User))
                 + s64_temp) >> VAR(AX0_u16_Pos_Shr_To_User));
}


/**********************************************************
 Function Name: ConvertCanOpenFbusCycleTimeToSeconds
 Description:
   calculates can open cycle time information to seconds.
   units of input are:
   10^(interpulation index)   *    interpulation time
   deafault of interpulation index is -3 - hence milli seconds.
   This conversion uses object 0x60C2 sub index 1 and 2

 Author: Aviv.
 Algorithm:
 Revisions:
**********************************************************/

int ConvertCanOpenFbusCycleTimeToSeconds(int drive)
{
   float f_time_factor=1.0;

   long s32_fix;
   unsigned int u16_shift;
   unsigned long u32_temp_value;

   int i=BGVAR(s8_Fb_Interp_Time_Idx);

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //calculate time factor using power function
   if(i >= 0)  for( ; i > 0; i--) f_time_factor *= 10;
   else for( ; i < 0; i++)  f_time_factor /= 10;

   FloatToFixS32Shift16( &s32_fix,
                         &u16_shift,
                         BGVAR(u8_Fb_Interp_Time_Period) * f_time_factor  );

   //Bugzila 2694 - Limit cycle time to no more than 1000 ms.
   if(MultS64ByFixS64ToS64((long long)1,(long long)s32_fix,u16_shift) > 1)
   {
      return FAL_FAIL;
   }

   BGVAR(fbus_cycle_time_in_secs).s32_fbus_cycle_time_fix = s32_fix;
   BGVAR(fbus_cycle_time_in_secs).u16_fbus_cycle_time_shr = u16_shift;

   // Calc the sync period as number of Tsp periods (250 us), and round.
   u32_temp_value = (unsigned long)(((float)BGVAR(u8_Fb_Interp_Time_Period) * f_time_factor * 4000.0) + 0.5);
   if (u32_CAN_Sync_250us_Length != u32_temp_value)
   {
            
      //sync length is converted and save for use in FbSyncFault()
      
      u32_CAN_Sync_250us_Length = u32_temp_value;
      
      if (BGVAR(u8_Sync_Mode) == 6)  // CAN PLL
      {
         u16_Pll_State = PLL_IDLE;
      }
   }
   u32_Actual_Cyc_Time = u32_CAN_Sync_250us_Length *(unsigned long)25;
   return FAL_SUCCESS;
}


/**********************************************************
 Function Name: ConvertFbCanOpenTorque
 Description:
   object 0x6076 sets rated torque. This is used to calculate the simple conversions
    object affected : target torque, actual torque (Objects 0x6071, 0x6077)
 Author: Aviv.
 Algorithm:
 Revisions: nitsan: changes due to fix of IPR 1286
**********************************************************/
void ConvertFbCanOpenTorque(int drive)
{
   long s32_fix;
   unsigned int u16_shift;

   REFERENCE_TO_DRIVE;     // Defeat compilation error
   
// IPR 1286: 6077h(Torque Actual Value) and 6078h (Current Actual Value) are not correct
// nitsan 12/7/2015: it is decided that convertaion for current and torque are identical 
//   since both are relative to micont and mitorque (micont*Kt).
//   in the torque convertion, Kt is being eliminated and we get the same convertion formula as for current.
   
/*
  *->s16_intern = [ ( (s16_user/1000) * (Kt/1000) * ( (0x6075 /(Kt/1000))) ) / DIPEAK) ] * 26214 =
                = [ (s16_user/1000) * ( 0x6075) ) / DIPEAK) ] * 26214
// Kt is being eliminated
    0x6075 - is not a mistake. Should have been 0x6076 but 0x6076 has already KT factor in it
  DIPEAK - mA
  0x6076 - mNm
  User   - 1000 is unity
  Kt     - mNm/A
*/

   OneDivS64ToFixU32Shift16(&s32_fix,&u16_shift,(long long)BGVAR(s32_Drive_I_Peak));
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,s32_fix,u16_shift,BGVAR(u32_Fb_Motor_Rated_Current),(unsigned int)0);
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,s32_fix,u16_shift,0x68DB22D1,(unsigned int)26);

   BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).s64_unit_conversion_to_internal_fix = s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;

   //update also RT parameters for later use
   s32_Torque_To_Internal_Fix = s32_fix;
   u16_Torque_To_Internal_Shr  = u16_shift;

   //set half for rounding
   u64_Torque_To_Internal_Half_For_Round = 0LL;
   if (u16_Torque_To_Internal_Shr > 0)
   {
       u64_Torque_To_Internal_Half_For_Round = (long long)(1LL << ((long long)u16_Torque_To_Internal_Shr - 1));
   }

// IPR 1286: 6077h(Torque Actual Value) and 6078h (Current Actual Value) are not correct
// nitsan 12/7/2015: it is decided that convertaion for current and torque are identical 
//   since both are relative to micont and mitorque (micont*Kt).
//   in the torque convertion, Kt is being eliminated and we get the same convertion formula as for current.

/*
  *->s16_user = s16_intern * 1,000 * ( (Kt/1000)/(0x6075 * (Kt/1000)))  * (DIPEAK/26214) = (1000 / 26214 ) * ( DIPEAK/0x6075 )
  Kt is eliminated.
  0x6075 - is not a mistake. Should have been 0x6076 but 0x6076 has already KT factor in it
*/

   OneDivS64ToFixU32Shift16(&s32_fix,&u16_shift,(long long)BGVAR(u32_Fb_Motor_Rated_Current));
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,s32_fix,u16_shift,BGVAR(s32_Drive_I_Peak),(unsigned int)0);
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,s32_fix,u16_shift,0x4E204E20,(unsigned int)35);

   BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).s64_unit_conversion_to_user_fix = s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_TORQUE_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;

   //update also RT parameters for later use
   s32_Torque_To_User_Fix = s32_fix;
   u16_Torque_To_User_Shr  = u16_shift;

   //set half for rounding
   u64_Torque_To_User_Half_For_Round = 0LL;
   if (u16_Torque_To_User_Shr > 0)
   {
       u64_Torque_To_User_Half_For_Round = (long long)(1LL << ((long long)u16_Torque_To_User_Shr - 1));
   }
}


/**********************************************************
 Function Name: ConvertFbCanOpenCurrent
 Description:
   object 0x6075 sets rated current. This is used to calculate the simple conversions
    object affected : max current,current actual value (0x6073,0x6078)
 Author: Aviv.
 Algorithm:
 Revisions:
**********************************************************/
void ConvertFbCanOpenCurrent(int drive)
{
   long s32_fix;
   unsigned int u16_temp_time, u16_shift;
   long long s64_half_for_rounding = 0LL;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;


/*
  *-> user = [ Fb_user/1000) * BGVAR(u32_fb_motor_rated_current)
*/
   FloatToFixS32Shift16(&s32_fix, &u16_shift,((float)BGVAR(u32_Fb_Motor_Rated_Current))/1000.);
   BGVAR(s32_FB_Crrnt_To_User_Fix)=s32_fix;
   BGVAR(u16_FB_Crrnt_To_User_Shr)=u16_shift;

   BGVAR(s64_FB_To_User_Half_For_Round) = 0LL;
   if (u16_shift > 0)  BGVAR(s64_FB_To_User_Half_For_Round) = (1LL << ((long long)u16_shift - 1));

/*
  *-> FB_user = [ user*1000) / BGVAR(u32_fb_motor_rated_current)
*/
   OneDivS64ToFixU32Shift16(&s32_fix,&u16_shift,(long long)BGVAR(u32_Fb_Motor_Rated_Current));
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,s32_fix,u16_shift,1000,0);

   BGVAR(s32_User_Crrnt_To_FB_Fix)=s32_fix;
   BGVAR(u16_User_Crrnt_To_FB_Shr)=u16_shift;

   BGVAR(s64_User_To_FB_Half_For_Round) = 0LL;
   if (u16_shift > 0)  BGVAR(s64_User_To_FB_Half_For_Round) = (1LL << ((long long)u16_shift - 1));

/*
  *->s16_intern = [ (s16_user/1000) * ( 0x6075) ) / DIPEAK) ] * 26214
*/
    OneDivS64ToFixU32Shift16(&s32_fix,&u16_shift,(long long)BGVAR(s32_Drive_I_Peak));
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,s32_fix,u16_shift,BGVAR(u32_Fb_Motor_Rated_Current),(unsigned int)0);
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,s32_fix,u16_shift,0x68DB22D1,(unsigned int)26);

   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;

   //update also RT parameters for later use
   LVAR(AX0_s32_Current_To_Internal_Fix) = s32_fix;
   VAR(AX0_u16_Current_To_Internal_Shr)  = u16_shift;

   //set half for rounding
   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Current_To_Internal_Half_For_Round_Lo) = 0LL;
   } while (u16_temp_time != Cntr_3125);
   if (VAR(AX0_u16_Current_To_Internal_Shr) > 0)
   {
       s64_half_for_rounding = (1LL << ((long long)VAR(AX0_u16_Current_To_Internal_Shr) - 1));
       LVAR(AX0_u32_Current_To_Internal_Half_For_Round_Hi) = (long)(s64_half_for_rounding >>32);
       LVAR(AX0_u32_Current_To_Internal_Half_For_Round_Lo) = (long)s64_half_for_rounding;
   }

/*
  *->s16_user = s16_intern * ( 1 / 0x6075)  * (DIPEAK/26214) * 1,000 = (DIPEAK/0x6075) * 1,000 / 26214
*/

   OneDivS64ToFixU32Shift16(&s32_fix,&u16_shift,(long long)BGVAR(u32_Fb_Motor_Rated_Current));
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,s32_fix,u16_shift,BGVAR(s32_Drive_I_Peak),(unsigned int)0);
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,s32_fix,u16_shift,0x4E204E20,(unsigned int)35);

   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;

   //update also RT parameters for later use
   LVAR(AX0_s32_Current_To_User_Fix) = s32_fix;
   VAR(AX0_u16_Current_To_User_Shr)  = u16_shift;

   //set half for rounding
   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Current_To_User_Half_For_Round_Lo) = 0LL;
   } while (u16_temp_time != Cntr_3125);
   if (VAR(AX0_u16_Current_To_User_Shr) > 0)
   {
       s64_half_for_rounding = (1LL << ((long long)VAR(AX0_u16_Current_To_User_Shr) - 1));
       LVAR(AX0_u32_Current_To_User_Half_For_Round_Hi) = (long)(s64_half_for_rounding >>32);
       LVAR(AX0_u32_Current_To_User_Half_For_Round_Lo) = (long)s64_half_for_rounding;
   }

   //recalculate FB_CAN_CURRENT_RECORDING_CONVERSION conversion
   MultFixS32ByFixS32ToFixS32(&s32_fix,&u16_shift,s32_fix,u16_shift,
                             (long)BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).s64_unit_conversion_to_internal_fix,
                            BGVAR(Unit_Conversion_Table[CURRENT_CONVERSION]).u16_unit_conversion_to_internal_shr);

   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_RECORDING_CONVERSION]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_RECORDING_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;
}


void ConvertRotLinVelToInternalOutLoopRPM(int drive)
{
   long s32_fix;
   unsigned int u16_shift;
   long          s32_mpitch_factor_fix = 1L;
   unsigned int  u16_mpitch_factor_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
        //internal = user / 1000 / 60 * 2^32 / 8000 = user * 8.9478485333333333333333333333333
      s32_fix = 1200959901;
      u16_shift = 27;
   }
   else// linear motor
   {
      //internal = user / 1000 * 2^32 / 8000 / MPITCH_FACTOR = user * 536870.912 / MPITCH_FACTOR
      OneDivS64ToFixU32Shift16(&s32_mpitch_factor_fix, &u16_mpitch_factor_shift,(long long)BGVAR(u32_Mpitch));
      MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_mpitch_factor_fix,u16_mpitch_factor_shift,(long)1099511627, (unsigned int)11);//changed since mpitch changed to decimal
   }

   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPM]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPM]).u16_unit_conversion_to_internal_shr = u16_shift;
}


void ConvertRotLinVelToUserOutLoopRPM(int drive)
{
   long s32_fix, s32_mpitch_factor_fix = 1L;
   unsigned int u16_shift, u16_mpitch_factor_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      //user = internal * 1000 * 8000 * 60 / 2^32 = internal * 0.111758708953857421875
      s32_fix = 1920000000;
      u16_shift = 34;
   }
   else// linear motor
   {
      //user = internal * 1000 * 8000 / 2^32 * MPITCH_FACTOR = internal * 0.00000186264514923095703125 * MPITCH_FACTOR
      s32_mpitch_factor_fix = BGVAR(u32_Mpitch);
      MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_mpitch_factor_fix,u16_mpitch_factor_shift,(long)2097152000, (unsigned int)50);//changed since mpitch changed to decimal
   }

   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPM]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPM]).u16_unit_conversion_to_user_shr = u16_shift;
}


void ConvertRotLinVelToInternalOutLoopRPS(int drive)
{
   long s32_fix;
   unsigned int u16_shift;
   long          s32_mpitch_factor_fix = 1L;
   unsigned int  u16_mpitch_factor_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
        //internal = user / 1000 * 2^32 / 8000 = user * 536.87091199999999999999999999998
      s32_fix = 1125899907;
      u16_shift = 21;
   }
   else// linear motor
   {
      //internal = user / 1000 * 2^32 / 8000 / MPITCH_FACTOR = user * 536870.912 / MPITCH_FACTOR
      OneDivS64ToFixU32Shift16(&s32_mpitch_factor_fix, &u16_mpitch_factor_shift,(long long)BGVAR(u32_Mpitch));
      MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_mpitch_factor_fix,u16_mpitch_factor_shift,(long)1099511627, (unsigned int)11);//changed since mpitch changed to decimal
   }

   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPS]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPS]).u16_unit_conversion_to_internal_shr = u16_shift;
}


void ConvertRotLinVelToUserOutLoopRPS(int drive)
{
   long s32_fix;
   unsigned int u16_shift;
   long          s32_mpitch_factor_fix = 1L;
   unsigned int  u16_mpitch_factor_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      //user = internal * 1000 * 8000 / 2^32 = internal * 0.00186264514923095703125
      s32_fix = 2048000000;
      u16_shift = 40;
   }
   else// linear motor
   {
      //user = internal * 1000 * 8000 / 2^32 * MPITCH_FACTOR = internal * 0.00000186264514923095703125 * MPITCH_FACTOR
      s32_mpitch_factor_fix = BGVAR(u32_Mpitch);
      MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_mpitch_factor_fix,u16_mpitch_factor_shift,(long)2097152000, (unsigned int)50);//changed since mpitch changed to decimal
   }

   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPS]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[VELOCITY_OUT_OF_LOOP_CONVERSION_LIN_ROT_RPS]).u16_unit_conversion_to_user_shr = u16_shift;
}


void UnitsVelRotLinRPMCommand(int drive)
{
   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      strcpy(BGVAR(s8_Units_Vel_Out_Loop_Rot_Lin_RPM), "rpm");
   }
   else//linear
   {
      strcpy(BGVAR(s8_Units_Vel_Out_Loop_Rot_Lin_RPM), "mm/s");
   }

   //update conversion table
   ConvertRotLinVelToInternalOutLoopRPM(drive);
   ConvertRotLinVelToUserOutLoopRPM(drive);
}


void UnitsVelRotLinRPSCommand(int drive)
{
   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      strcpy(BGVAR(s8_Units_Vel_Out_Loop_Rot_Lin_RPS), "rps");
   }
   else//linear
   {
      strcpy(BGVAR(s8_Units_Vel_Out_Loop_Rot_Lin_RPS), "mm/s");
   }

   //update conversion table
   ConvertRotLinVelToInternalOutLoopRPS(drive);
   ConvertRotLinVelToUserOutLoopRPS(drive);
}


void UnitsPosRotLinCommand(int drive)
{
   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      strcpy(BGVAR(s8_Units_Pos_Rot_Lin), "rev");
   }
   else//linear
   {
      strcpy(BGVAR(s8_Units_Pos_Rot_Lin), "mm");
   }

   //update conversion table
   ConvertRotLinPosToInternal(drive);
   ConvertRotLinPosToUser(drive);
}


void UnitsRotLinKPPCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      strcpy(BGVAR(s8_Units_Rot_Lin_KPP), "rps/rev");
   }
   else//linear
   {
      strcpy(BGVAR(s8_Units_Rot_Lin_KPP), "(mm/s)/mm");
   }
}


void UnitsRotLinENCRESCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      strcpy(BGVAR(s8_Units_Rot_Lin_ENCRES), "LPR");
   }
   else//linear
   {
      strcpy(BGVAR(s8_Units_Rot_Lin_ENCRES), "LPP");
   }
}


void UnitsAmpVelRotLinCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      strcpy(BGVAR(s8_Units_Rot_Lin_AMP_VEL), "A/rps");
   }
   else//linear
   {
      strcpy(BGVAR(s8_Units_Rot_Lin_AMP_VEL), "A/(mm/s)");
   }
}


void UnitsMechAngleRotLinCommand(int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u16_MotorType) == ROTARY_MOTOR)
   {
      strcpy(BGVAR(s8_Units_Rot_Lin_MECHANGLE), "65536/rev");
   }
   else//linear
   {
      strcpy(BGVAR(s8_Units_Rot_Lin_MECHANGLE), "65536/pitch");
   }
}


void ConvertMsAccDecVelToInternal(int drive)
{
   long long s64_fix = 0L;
   long s32_fix = 0;
   unsigned int  u16_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

  //internal = 6000/user / 60 * 22750 / VLIM_INT * 2^32 / 8000 = 1/user * 655,724,105,745,144,217,200 / VLIM_INT
   OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_V_Lim_Design));
   MultFix64ByFix32ToFix64(&s64_fix,&u16_shift,(long long)0x48CCCCCCCCCCCCCA,(unsigned int)3,s32_fix,u16_shift);

   BGVAR(Unit_Conversion_Table[ACC_DEC_6000_RPM_IN_MS_VEL_CONVERSION]).s64_unit_conversion_to_internal_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[ACC_DEC_6000_RPM_IN_MS_VEL_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;
}


void ConvertInternalToMsAccDecVel(int drive)
{
   long s32_fix = 0L;
   unsigned int  u16_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   //user = 1 / internal * VLIM_INT * 0.0000000000000000015250316272
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, BGVAR(s32_V_Lim_Design),0,(long)14, (unsigned int)63);
   BGVAR(Unit_Conversion_Table[ACC_DEC_6000_RPM_IN_MS_VEL_CONVERSION]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[ACC_DEC_6000_RPM_IN_MS_VEL_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;
}


//**********************************************************
// Function Name: convertMsToAccDecInternalUnitsForPos
// Description: convert from user ms to internal acc dec for position units
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
unsigned long long convertMsToAccDecInternalUnitsForPos(int drive, unsigned int ms)
{
   unsigned long long u64_acc_dec_for_pos_Result = 0;
   long long s64_ms_fix = 0;
   unsigned int u16_ms_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

      // ms value must be checked in the sal function to return "out of range" error
   if( (ms > 65500) || (ms < 1)) return 0;

   // need to convert units:
   // ACC/DEC from ms into internal acceleration/deceleration units

   // 1/user (ms) = x
   OneDivS64ToFixU64Shift16(&s64_ms_fix, &u16_ms_shift,(long long) ms);

   // convert the user acc dec into internal position acc dec values
   // x * unit conversion
   MultFix64ByFix64ToFix64(&s64_ms_fix, &u16_ms_shift, s64_ms_fix,u16_ms_shift,
                          (BGVAR(Unit_Conversion_Table[ACC_DEC_6000_RPM_IN_MS_POS_CONVERSION]).s64_unit_conversion_to_internal_fix),
                           BGVAR(Unit_Conversion_Table[ACC_DEC_6000_RPM_IN_MS_POS_CONVERSION]).u16_unit_conversion_to_internal_shr);
   // get the calculated value out from "fix shift" format
   u64_acc_dec_for_pos_Result = MultS64ByFixS64ToS64(1LL,s64_ms_fix,u16_ms_shift);

   return u64_acc_dec_for_pos_Result;
}


//**********************************************************
// Function Name: convertAccDecInternalUnitsForPosToMs
// Description: convert from internal acc dec for position units to user ms
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
unsigned int convertAccDecInternalUnitsForPosToMs(int drive, long long s64_acc_dec_for_pos_ms)
{
   long long s64_ms_fix = 0;
   unsigned int u16_ms_shift = 0, u16_acc_dec_user_ms_result = 0;
   float f_temp = 0.0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   MultFix64ByFix64ToFix64(&s64_ms_fix,&u16_ms_shift, s64_acc_dec_for_pos_ms,0,
              (BGVAR(Unit_Conversion_Table[ACC_DEC_6000_RPM_IN_MS_POS_CONVERSION]).s64_unit_conversion_to_user_fix),
               BGVAR(Unit_Conversion_Table[ACC_DEC_6000_RPM_IN_MS_POS_CONVERSION]).u16_unit_conversion_to_user_shr);

   f_temp = (float)((float) s64_ms_fix / (float)power((float)2,u16_ms_shift));
   f_temp = 1.0 / f_temp;
   u16_acc_dec_user_ms_result = (unsigned int) f_temp;

   if(u16_acc_dec_user_ms_result > (unsigned int)65500)
   {
      u16_acc_dec_user_ms_result = 65500;
   }
   else if(u16_acc_dec_user_ms_result < (unsigned int)1)
   {
      u16_acc_dec_user_ms_result = 1;
   }

   return u16_acc_dec_user_ms_result;
}


//**********************************************************
// Function Name: UpdateS64PfbInPulseUnit
// Description: convert from internal position
// to PULSE unit and update "s64_PFB_In_Pulse_Unit" global variable (Lexium Fb.PLS)
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void UpdateS64PfbInPulseUnit(int drive)
{
   // AXIS_OFF;
   int s16_temp_time;
   long long s64_temp;
   REFERENCE_TO_DRIVE;

   do {
      s16_temp_time = Cntr_3125;
      s64_temp = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
   } while (s16_temp_time != Cntr_3125);
   BGVAR(s64_PFB_In_Pulse_Unit) = MultS64ByFixS64ToS64(s64_temp,
                                                       BGVAR(Unit_Conversion_Table[POSITION_PULSE_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                       BGVAR(Unit_Conversion_Table[POSITION_PULSE_CONVERSION]).u16_unit_conversion_to_user_shr);
}


//**********************************************************
// Function Name: ConvertPfbInPUUUnit
// Description: convert from internal pfb position to PUU (pulse user unit) and return the value
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
long ConvertPfbInPUUUnit(int drive, long long CDHD_Internal_PFB)
{
   long s32_pfb_puu = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   s32_pfb_puu = (long)MultS64ByFixS64ToS64(CDHD_Internal_PFB,
                                     BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                                     BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);
   return s32_pfb_puu;
}


//**********************************************************
// Function Name: UpdateS64PositionCommandInPulseUnit
// Description: convert from internal position command
// to PULSE unit and update "s64_Position_Command_In_Pulse_Unit" global variable (Lexium C.PLS)
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void UpdateS64PositionCommandInPulseUnit(int drive)
{
   // AXIS_OFF;
   int s16_temp_time;
   long long s64_temp;
   REFERENCE_TO_DRIVE;

   do {
      s16_temp_time = Cntr_3125;
      s64_temp = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
   } while (s16_temp_time != Cntr_3125);
   BGVAR(s64_Position_Command_In_Pulse_Unit) = MultS64ByFixS64ToS64(s64_temp,
                                                    BGVAR(Unit_Conversion_Table[POSITION_PULSE_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                    BGVAR(Unit_Conversion_Table[POSITION_PULSE_CONVERSION]).u16_unit_conversion_to_user_shr);
}


//**********************************************************
// Function Name: UpdateS64PositionErrorInPulseUnit
// Description: convert from internal position
// to PULSE unit and update "s64_Position_Error_In_Pulse_Unit" global variable (Lexium Er.PLS)
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void UpdateS64PositionErrorInPulseUnit(int drive)
{
   // AXIS_OFF;
   int s16_temp_time;
   long long s64_temp;
   REFERENCE_TO_DRIVE;

   do {
      s16_temp_time = Cntr_3125;
      s64_temp = LLVAR(AX0_u32_Pos_Err_Lo);
   } while (s16_temp_time != Cntr_3125);
   BGVAR(s64_Position_Error_In_Pulse_Unit) = MultS64ByFixS64ToS64(s64_temp,
                                                    BGVAR(Unit_Conversion_Table[POSITION_PULSE_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                    BGVAR(Unit_Conversion_Table[POSITION_PULSE_CONVERSION]).u16_unit_conversion_to_user_shr);
}


//**********************************************************
// Function Name: ConvertFbCanCurrentPerSecToInternal
// Description: convert to & from internal current per 31.25u[sec]
//              from CAN
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
void ConvertFbCanCurrentPerSecToInternal(int drive)
{
   long s32_fix = 0L;
   unsigned int  u16_shift = 0;
   long s32_temp_micont = 100;
   REFERENCE_TO_DRIVE;

   // AXIS_OFF;

   if (BGVAR(u32_Fb_Motor_Rated_Current) != 0)  {s32_temp_micont = BGVAR(u32_Fb_Motor_Rated_Current);}
/*
     conversion from .001 *MICONT/sec to internal
    The torque slope will be activated every 31.25u[sec] ->32000 times in a second
 * ->s16_intern = [ (s16_user/1000) * ( 0x6075) ) / DIPEAK) ] * 26214 / 32000
    In addition,in order to increse resolution to 32 bits ,multiplication by 32768 is required (15 shifts left)
   Therefore internal = (s16_user) * ( 0x6075) ) / DIPEAK) ] * 26214 / 32000 /1000 * 32768
 */
   OneDivS64ToFixU32Shift16(&s32_fix,&u16_shift,(long long)BGVAR(s32_Drive_I_Peak));
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,s32_fix,u16_shift,s32_temp_micont,(unsigned int)0);
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,s32_fix,u16_shift,0x6B5F5F0B,(unsigned int)26);

   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_PER_SEC_CONVERSION]).s64_unit_conversion_to_internal_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_PER_SEC_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;

   //update also RT parameters for later use
   LVAR(AX0_s32_Crrnt_Per_Sec_To_Intrn_Fix) = s32_fix;
   VAR(AX0_u16_Crrnt_Per_Sec_To_Intrn_Shr)  = u16_shift;

   if ((u16_shift > 0) && (u16_shift < 64))
   {
      LLVAR(AX0_s64_Crrnt_Per_Sec_To_Intrn_Round) = 1LL << (u16_shift - 1);
   }
   else
   {
      LLVAR(AX0_s64_Crrnt_Per_Sec_To_Intrn_Round) = 0;
   }
}


//**********************************************************
// Function Name: ConvertFbCanCurrentPerSecToUser
// Description: convert to & from internal current per 31.25u[sec]
//              from CAN
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
void ConvertFbCanCurrentPerSecToUser(int drive)
{
   long s32_fix = 0L;
   unsigned int  u16_shift = 0;
   long s32_temp_micont = 100;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(u32_Fb_Motor_Rated_Current) != 0)  {s32_temp_micont = BGVAR(u32_Fb_Motor_Rated_Current);}
/*
     conversion from internal 0.001 *MICONT/sec
     The torque slope will be activated every 31.25u[sec] ->32000 times in a second
 * ->s16_intern = [ (s16_user/1000) * ( 0x6075) ) / DIPEAK) ] * 26214 / 32000
    In addition,in order to increse resolution to 32 bits ,multiplication by 32768 is required (15 shifts left)
    internal = (s16_user) * ( 0x6075) ) / DIPEAK) ] * 26214 / 32000 /1000 * 32768
    Therfore
    user = internal * DIPEAK * 32000 * 1000 / (object 0x6075) / 26214 / 32768
 */
   OneDivS64ToFixU32Shift16(&s32_fix,&u16_shift, (long long)s32_temp_micont);
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix, u16_shift, BGVAR(s32_Drive_I_Peak), (unsigned int)0);
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix, u16_shift, 0x4C4B8C4C, (unsigned int)35);

   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_PER_SEC_CONVERSION]).s64_unit_conversion_to_user_fix = (long long)s32_fix;
   BGVAR(Unit_Conversion_Table[FB_CAN_CURRENT_PER_SEC_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;
}


//**********************************************************
// Function Name: ConvertInternalCurrentSlopeToUser
// Description: convert to & from internal current per 31.25u[sec]
// Author: Gil
// Algorithm:
// Revisions:
//**********************************************************
void ConvertInternalCurrentSlopeToUser(int drive)
{
   long s32_fix;
   unsigned int  u16_shift;
   REFERENCE_TO_DRIVE;

   // AXIS_OFF;

   // user = internal * DIPEAK * 32000 /(2^15 * 26214)    (2^15 is due to increased resolution of the slope)
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift,(long)0x4E204E20,(unsigned int)45,(long long)BGVAR(s32_Drive_I_Peak),(unsigned int)0);
   BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).s64_unit_conversion_to_user_fix = s32_fix;
   BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;

   //update also RT parameters for later use
   LVAR(AX0_s32_Intrn_To_Crrnt_Per_Sec_Fix) = s32_fix;
   VAR(AX0_u16_Intrn_To_Crrnt_Per_Sec_Shr)  = u16_shift;

   if ((u16_shift > 0) && (u16_shift < 64))
   {
      LLVAR(AX0_s64_Intrn_To_Crrnt_Per_Sec_Round) = 1LL << (u16_shift - 1);
   }
   else
   {
      LLVAR(AX0_s64_Intrn_To_Crrnt_Per_Sec_Round) = 0;
   }

}


//**********************************************************
// Function Name: ConvertInternalCurrentSlopeToInternal
// Description: convert to & from internal current per 31.25u[sec]
// Author: AZ
// Algorithm:
// Revisions:
//**********************************************************
void ConvertInternalCurrentSlopeToInternal(int drive)
{
   long s32_fix;
   unsigned int  u16_shift;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // [user] = mA / sec
   // [internal] = current bits * 2^15 / Tsc = 26214 / DIPEAK * 65536 / 32000    (2^15 is to increase the resolution of the slope)
   // internal = user * (2^15 * 26214) / (DIPEAK[mA] * 32000) =
   //          = user * 26843.136 / DIPEAK[mA]
   OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,(long long)BGVAR(s32_Drive_I_Peak));
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix,u16_shift,(long)0x68DB22D1, (unsigned int)16);
   BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).s64_unit_conversion_to_internal_fix = s32_fix;
   BGVAR(Unit_Conversion_Table[CURRENT_PER_SEC_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;
}


//**********************************************************
// Function Name: UpdateS64PfbInPuuUnit
// Description: convert from internal position
// to PUU and update "s64_PFB_In_PUU_Unit" global variable (Lexium Fb.PUU)
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************

void UpdateS64PfbInPuuUnit(int drive)
{
   // AXIS_OFF;
   int s16_temp_time;
   long long s64_temp;
   REFERENCE_TO_DRIVE;

   do {
      s16_temp_time = Cntr_3125;
      s64_temp = LLVAR(AX0_u32_Pos_Fdbk_User_Lo);
   } while (s16_temp_time != Cntr_3125);
   BGVAR(s64_PFB_In_PUU_Unit) = MultS64ByFixS64ToS64(s64_temp,
                                                       BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                       BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);
}


//**********************************************************
// Function Name: UpdateS64PositionCommandInPuuUnit
// Description: convert from internal position command
// to PUU and update "s64_Position_Command_In_PUU_Unit" global variable (Lexium C.PUU)
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void UpdateS64PositionCommandInPuuUnit(int drive)
{
   // AXIS_OFF;
   int s16_temp_time;
   long long s64_temp;
   REFERENCE_TO_DRIVE;

   do {
      s16_temp_time = Cntr_3125;
      s64_temp = LLVAR(AX0_u32_Pos_Cmd_User_Lo);
   } while (s16_temp_time != Cntr_3125);
   BGVAR(s64_Position_Command_In_PUU_Unit) = MultS64ByFixS64ToS64(s64_temp,
                                                       BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                       BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);
}


//**********************************************************
// Function Name: GetS32PfbInPuuUnit
// Description: Translate S64 Value to S32 by specific unit Idx
// Author: Moshe / Udi
// Algorithm:
// Revisions:
//**********************************************************
long GetS32ValueByUnit(int drive, long long s64_Raw_Value, int s16_Unit_Idx, int s16_To_User)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if(s16_To_User == 1)
   {
       return (long) MultS64ByFixS64ToS64(s64_Raw_Value,
                       BGVAR(Unit_Conversion_Table[s16_Unit_Idx]).s64_unit_conversion_to_user_fix,
                       BGVAR(Unit_Conversion_Table[s16_Unit_Idx]).u16_unit_conversion_to_user_shr);
   }
   else
   {
          return (long) MultS64ByFixS64ToS64(s64_Raw_Value,
                       BGVAR(Unit_Conversion_Table[s16_Unit_Idx]).s64_unit_conversion_to_internal_fix,
                       BGVAR(Unit_Conversion_Table[s16_Unit_Idx]).u16_unit_conversion_to_internal_shr);

   }
}


//**********************************************************
// Function Name: UpdateS64PositionErrorInPuuUnit
// Description: convert from internal position
// to PUU and update "s64_Position_Error_In_PUU_Unit" global variable (Lexium Er.PUU)
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void UpdateS64PositionErrorInPuuUnit(int drive)
{
   // AXIS_OFF;
   int s16_temp_time;
   long long s64_temp;
   REFERENCE_TO_DRIVE;

   do {
      s16_temp_time = Cntr_3125;
      s64_temp = LLVAR(AX0_u32_Int_Loop_Pos_Err_Lo);
   } while (s16_temp_time != Cntr_3125);
   BGVAR(s64_Position_Error_In_PUU_Unit) = MultS64ByFixS64ToS64(s64_temp,
                                                       BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix,
                                                       BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr);
}


//**********************************************************
// Function Name: ConvertToPUURecalculation
// Description: recalculate the PULSE to PUU unit conversion FIX SHIFT
//              recalculate the CDHD internal unit to PUU conversion FIX SHIFT
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void ConvertToPUURecalculation(int drive)
{
   // AXIS_OFF;
   long long s64_fix;
   unsigned int  u16_shift;
   REFERENCE_TO_DRIVE;

   // recalculate only for Schneider drive type
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   // PUU recalculated only for P1-44 or P1-45 changes

   // calc 1/P1-44
   OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift,(long long)BGVAR(s32_P1_44_Gear_In_Numerators_Arr)[0]);

   // mult 1/P1-44 by P1-45
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift,(long)LVAR(AX0_u32_Gearout_Design), 0);

   // pulse to PUU conversion save
   BGVAR(Unit_Conversion_Table[POSITION_PULSE_TO_PUU_CONVERSION]).s64_unit_conversion_to_user_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[POSITION_PULSE_TO_PUU_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;

   // CDHD internal to PUU conversion save

   // calc (1,280,000 / 2^32) * (P1-45/p1-44)
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, (long)1280000, 32);

   BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_user_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;
}


//**********************************************************
// Function Name: ConvertFromPUURecalculation
// Description: recalculate the PUU to PULSE unit conversion FIX SHIFT
//              recalculate the PUU to CDHD internal unit conversion FIX SHIFT
// Author: Moshe
// Algorithm:
// Revisions:
//**********************************************************
void ConvertFromPUURecalculation(int drive)
{
   // AXIS_OFF;
   long long s64_fix, s64_fix2;
   unsigned int  u16_shift, u16_shift2;
   REFERENCE_TO_DRIVE;

   // recalculate only for Schneider drive type
   if ((u16_Product != SHNDR) && (u16_Product != SHNDR_HW)) return;

   // PUU recalculated only for P1-44 or P1-45 changes

   // calc 1/P1-45
   OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift,(long long)LVAR(AX0_u32_Gearout_Design));

   // mult 1/P1-45 by P1-44
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, BGVAR(s32_P1_44_Gear_In_Numerators_Arr)[0], 0);

   // PUU to pulses conversion save
   BGVAR(Unit_Conversion_Table[POSITION_PULSE_TO_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[POSITION_PULSE_TO_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;

   // PUU to CDHD internal conversion save
   //calc 1/1,280,000
   OneDivS64ToFixU64Shift16(&s64_fix2, &u16_shift2,(long long)1280000);

   // mult 1/1,280,000 by (P1-44/p1-45)
   MultFix64ByFix64ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, s64_fix2, u16_shift2);

   // calc (2^32 / 1,280,000) * (P1-44/p1-45)
   u16_shift -= 32; // equivalent to mult by 2^32

   BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).s64_unit_conversion_to_internal_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[POSITION_PUU_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;
}


/**********************************************************
 Function Name: ConversionSecondaryEncoder
 Description: Converts from secondary encoder to internal,
                       from internal to user and
                       from user to internal
 Author: Lionel
**********************************************************/

void ConversionSecondaryEncoder(int drive)
{
   long long     s64_fix = 1LL;
   long          s32_fix = 1L;
   unsigned int  u16_shift = 0;
   unsigned long u32_Abs_SFB2MotorNum, u32_Abs_SFBUnitNum;
   int           s16_sign = 0;
//   long long     s64_half_for_rounding = 0LL;
//   unsigned int  u16_temp_time;
   // AXIS_OFF;
   REFERENCE_TO_DRIVE;

   if (BGVAR(s32_SFB2MotorNum) < 0L)
   {
      u32_Abs_SFB2MotorNum = - BGVAR(s32_SFB2MotorNum);
      s16_sign = 1;
   }
   else
      u32_Abs_SFB2MotorNum = BGVAR(s32_SFB2MotorNum);

// if (secondary_encoder is analog)
// external feedback to internal
// internal value for one volt = 2621.44
// internal = analog_input * (1/2621.44) * (rev/volt) *  2^32
//          = analog_input * (s32_SFB2MotorDen)/s32_SFBUnitsNum) * 0x6400 0000 >> 10
   OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift,u32_Abs_SFB2MotorNum);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, 0x64000000, 10);
   MultFix64ByFix32ToFix32(&s32_fix, &u16_shift, s64_fix, u16_shift, BGVAR(s32_SFB2MotorDen), 0);
   if (s16_sign)
      s32_fix = - s32_fix;
   LVAR(AX0_s32_Pext_Scale_Fix) = s32_fix;
   VAR(AX0_s16_Pext_Scale_Shr) = u16_shift;

   if (BGVAR(s32_SFBUnitsNum) < 0L)
   {
      u32_Abs_SFBUnitNum = - BGVAR(s32_SFBUnitsNum);
      s16_sign = !s16_sign;
   }
   else
      u32_Abs_SFBUnitNum = BGVAR(s32_SFBUnitsNum);


// position : user to internal
// internal = user * (1/1000) * 2^32 * (volt/unit) * (rev/volt)
// internal = user * (s32_SFBUnitsNum/s32_SFBUnitsDen) * (s32_SFB2MotorDen/s32_SFBUnitsNum) / 1000 << 32
// internal = user * (s32_SFBUnitsNum/s32_SFBUnitsDen) * (s32_SFB2MotorDen/s32_SFBUnitsNum) * 0x4189374C >> 8

   OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift, (long long)u32_Abs_SFB2MotorNum * BGVAR(s32_SFBUnitsDen));
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, 0x4189374CL, 8);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, BGVAR(s32_SFB2MotorDen), 0);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, u32_Abs_SFBUnitNum, 0);
   if (s16_sign)
      s64_fix = - s64_fix;
   BGVAR(Unit_Conversion_Table[POSITION_SECONDARY_FEEDBACK_CONVERSION]).s64_unit_conversion_to_internal_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[POSITION_SECONDARY_FEEDBACK_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;

// position : internal to user
// user = internal * 2^(-32) * (volt/rev) * (unit/volt) * 1000
// user = internal * (s32_SFBUnitsNum/s32_SFB2MotorDen) * (s32_SFBUnitsDen/s32_SFBUnitsNum) * 1000 >> 32

// check that we are ok here with the casting
   OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift, (long long)BGVAR(s32_SFB2MotorDen) * u32_Abs_SFBUnitNum);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, 1000L, 32);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, u32_Abs_SFB2MotorNum, 0);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, BGVAR(s32_SFBUnitsDen), 0);
   if (s16_sign)
      s64_fix = - s64_fix;
   BGVAR(Unit_Conversion_Table[POSITION_SECONDARY_FEEDBACK_CONVERSION]).s64_unit_conversion_to_user_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[POSITION_SECONDARY_FEEDBACK_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;

   //update also RT parameters for later use
 /*  s64_fix += 0x40000000LL;  // rounding
   s64_fix += 0x40000000LL;  // rounding
   LVAR(AX0_s32_Sfb_Fix_To_User) = (long)(s64_fix >> 32);
   VAR(AX0_u16_Sfb_Shr_To_User) = u16_shift - 32;

   //set half for rounding
   do {
      u16_temp_time = Cntr_3125;
      LLVAR(AX0_u32_Sfb_Half_For_Rounding_To_User_Lo) = 0LL;
   } while (u16_temp_time != Cntr_3125);
   if (VAR(AX0_u16_Sfb_Shr_To_User) > 0)
   {
      s64_half_for_rounding = (1LL << ((long long)VAR(AX0_u16_Sfb_Shr_To_User) - 1));
      do {
         u16_temp_time = Cntr_3125;
         LLVAR(AX0_u32_Sfb_Half_For_Rounding_To_User_Lo) = s64_half_for_rounding;
      } while (u16_temp_time != Cntr_3125);
   }
*/


// velocity : user to internal
// internal = user * (1/1000) * [internal]/[Vel User Unit]
// internal = user * (1/1000) * 2^32 * [rev @ 8 kHz]/[Vel UU]
// internal = user * (1/1000) * 2^32 * (1/8000) * [rev/s]/[(Pos UU)/s]
// internal = user * (1/10^6) * 2^29 * [rev]/[Pos UU]
// internal = user * (1/10^6) * 2^29 * [rev]/[volt] * [volt]/[Pos UU]
// internal = user * (2^29/10^6) * (s32_SFB2MotorDen/s32_SFB2MotorNum) * (s32_SFBUnitsNum/s32_SFBUnitsDen)
// internal = user * (s32_SFB2MotorDen/s32_SFB2MotorNum) * (s32_SFBUnitsNum/s32_SFBUnitsDen) * 0x431BDE83 >> 21
   OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift, (long long)u32_Abs_SFB2MotorNum * BGVAR(s32_SFBUnitsDen));
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, 0x431BDE83L, 21);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, BGVAR(s32_SFB2MotorDen), 0);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, u32_Abs_SFBUnitNum, 0);
   if (s16_sign)
      s64_fix = - s64_fix;
   BGVAR(Unit_Conversion_Table[VELOCITY_SECONDARY_FEEDBACK_CONVERSION]).s64_unit_conversion_to_internal_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[VELOCITY_SECONDARY_FEEDBACK_CONVERSION]).u16_unit_conversion_to_internal_shr = u16_shift;


// velocity : internal to user
// opposite of the calculation above!
// user = internal * (10^6/2^29) * (s32_SFB2MotorNum/s32_SFB2MotorDen) * (s32_SFBUnitsDen/s32_SFBUnitsNum)
// user = internal * (s32_SFB2MotorNum/s32_SFB2MotorDen) * (s32_SFBUnitsDen/s32_SFBUnitsNum) * 1000000 >> 29

   OneDivS64ToFixU64Shift16(&s64_fix, &u16_shift, (long long)BGVAR(s32_SFB2MotorDen) * u32_Abs_SFBUnitNum);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, 1000000L, 29);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, u32_Abs_SFB2MotorNum, 0);
   MultFix64ByFix32ToFix64(&s64_fix, &u16_shift, s64_fix, u16_shift, BGVAR(s32_SFBUnitsDen), 0);
   if (s16_sign)
      s64_fix = - s64_fix;
   BGVAR(Unit_Conversion_Table[VELOCITY_SECONDARY_FEEDBACK_CONVERSION]).s64_unit_conversion_to_user_fix = s64_fix;
   BGVAR(Unit_Conversion_Table[VELOCITY_SECONDARY_FEEDBACK_CONVERSION]).u16_unit_conversion_to_user_shr = u16_shift;
}

//**********************************************************
// Function Name: ConversionAninToUserUnitsPreperation
// Description: This function is used in order to calculate a Fix/Shift conversion
//              factor in order to convert an analog input voltage into a user
//              unit.
//
// Author: APH
// Algorithm:
// Revisions:
//**********************************************************
void ConversionAninToUserUnitsPreperation(int drive)
{
   long          s32_fix = 1LL;
   unsigned int  u16_shift = 0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // Here we are calculating a Fix-Shift value in order to perform a unit-conversion
   // from an analogue input value (Volt internal) into a user-unit. The conversion
   // is as follows:
   //   For the user: ANIN2USER[user-counts] = ANIN2[V] * (ANIN2NUM[user-counts] / ANIN2DEN[V]) + ANIN2USEROFFSET[user-counts]
   //   Internally:   ANIN2USER[user-counts] = ANIN2[V_internal] * (ANIN2NUM[user-counts] / (ANIN2DEN[V]*26214/10)) + ANIN2USEROFFSET[user-counts]
   //
   // First calculate: ANIN2NUM[user-unit] / (ANIN2DEN[V] * 26214 / 10[V])
   OneDivS64ToFixU32Shift16(&s32_fix, &u16_shift,((long long)BGVAR(s32_Anin2_User_Den)*26214)/(long long)10);
   MultFixS32ByFixS32ToFixS32(&s32_fix, &u16_shift, s32_fix, u16_shift, BGVAR(s32_Anin2_User_Num), 0);

   BGVAR(s32_Anin2_To_User_Fix) = s32_fix;
   BGVAR(u16_Anin2_To_User_Shr) = u16_shift;
}


void SfbUnitsStringsUpdate(void)
{
   if (LOAD_TYPE == ROTARY_MOTOR)
   {
      //set the Load's position units string
      switch (BGVAR(u16_Units_Pos_Rotary))
      {
         case REV_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "rev");
         break;
         case ROT_COUNTS_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "counts");
         break;
         case DEG_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "deg");
         break;
         case USER_ROT_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "USER");
         break;
      }   
      
      //set the Load's velocity units string
      switch (BGVAR(u16_Units_Vel_Rotary))
      {
         case RPS_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "rps");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "rps");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "rps/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp_Sfb), "rps");
         break;
         case RPM_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "rpm");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "rpm");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "rpm/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp_Sfb), "rpm");
         break;
         case DEG_PER_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "deg/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "deg/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "(deg/s)/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp_Sfb), "deg/s");
         break;
         case USER_PER_SEC_ROT_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "USER/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "USER/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "(USER/s)/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp_Sfb), "USER/s");
         break;
      }
  
      //set the Load's acceleration units string
      switch (BGVAR(u16_Units_Acc_Dec_Rotary))
      {
         case RPS_TO_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel_Sfb), "rps/s");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "rps/s");
         break;
         case RPM_TO_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel_Sfb), "rpm/s");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "rpm/s");
         break;
         case DEG_PER_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel_Sfb), "deg/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "deg/s^2");
         break;
         case USER_PER_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel_Sfb), "USER/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "USER/s^2");
         break;
      }    
   }
   
   if (LOAD_TYPE == LINEAR_MOTOR)
   {
      //set the variable units string
      switch (BGVAR(u16_Units_Pos_Linear))
      {
         case PITCH_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "Pitch");
         break;
         case LIN_COUNTS_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "counts");
         break;
         case UM_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "um");
         break;
         case MM_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "mm");
         break;
         case USER_LIN_UNITS:
            strcpy(BGVAR(s8_Units_Pos_Sfb), "USER");
         break;
      }
      
      switch (BGVAR(u16_Units_Vel_Linear))
      {
         case UM_PER_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "um/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "um/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "(um/s)/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp_Sfb), "um/s");
         break;
         case MM_PER_SEC_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "mm/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "mm/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "(mm/s)/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp_Sfb), "mm/s");
         break;
         case USER_PER_SEC_LIN_UNITS:
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Sfb), "USER/s");
            strcpy(BGVAR(s8_Units_Vel_In_Loop_Sfb), "USER/s");
            strcpy(BGVAR(s8_Units_Vel_Out_Loop_Scale_Sfb), "(USER/s)/V");
            strcpy(BGVAR(s8_Units_Vel_Ptp_Sfb), "USER/s");
         break;
      }
      
      switch (BGVAR(u16_Units_Acc_Dec_Linear))
      {
         case UM_TO_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "um/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "um/s^2");
         break;
         case MM_TO_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "mm/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "mm/s^2");
         break;
         case USER_TO_SEC_2_POWER_UNITS:
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Vel), "USER/s^2");
            strcpy(BGVAR(s8_Units_Acc_Dec_For_Pos_Sfb), "USER/s^2");
         break;
      }
   }
}

