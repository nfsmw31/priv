#include "FltCntrl.def"
#include "ModCntrl.def"
#include "Current.def"
#include "Endat.def"
#include "MultiAxis.def"
#include "Err_Hndl.def"

#include "Motor.var"
#include "Drive.var"
#include "Burnin.var"
#include "FltCntrl.var"
#include "Extrn_Asm.var"
#include "Endat.var"
#include "Foldback.var"

#include "Prototypes.pro"

void InitBurnin(int drive)
{
   // AXIS_OFF;

   BGVAR(s32_ParamsBackup)[0] = BGVAR(u32_Mlmin);
   BGVAR(s32_ParamsBackup)[1] = BGVAR(s32_Motor_I_Peak);
   BGVAR(s32_ParamsBackup)[2] = BGVAR(s32_Motor_I_Cont);
   BGVAR(s32_ParamsBackup)[4] = BGVAR(s16_Mcont_Current_Gain);
   BGVAR(s32_ParamsBackup)[5] = BGVAR(s16_Mpeak_Current_Gain);
   BGVAR(s32_ParamsBackup)[6] = BGVAR(s32_Ilim_User);
   BGVAR(s32_ParamsBackup)[7] = BGVAR(u32_Mspeed);
   BGVAR(s32_ParamsBackup)[8] = BGVAR(u16_Mpoles);
   BGVAR(s32_ParamsBackup)[9] = BGVAR(s32_Icont);
   BGVAR(s32_ParamsBackup)[10] = BGVAR(u16_Disable_Mode);
   BGVAR(s32_ParamsBackup)[11] = BGVAR(s32_V_Lim_Design);
   BGVAR(s32_ParamsBackup)[12] = BGVAR(s32_Ilim_Actual);
   BGVAR(s32_ParamsBackup)[13] = (long)BGVAR(u16_Motor_Foldback_Disable);
   BGVAR(s32_ParamsBackup)[14] = (long)BGVAR(u16_FdbkType);
   BGVAR(s32_ParamsBackup)[15] = (long)VAR(AX0_s16_Motor_Enc_Type);

   BGVAR(u32_Mlmin) = 2000;
   BGVAR(s32_Motor_I_Peak) = BGVAR(s32_Drive_I_Peak);
   BGVAR(s32_Motor_I_Cont) = BGVAR(s32_Drive_I_Cont);
   BGVAR(s16_Mcont_Current_Gain) = 1000;
   BGVAR(s16_Mpeak_Current_Gain) = 1000;
   BGVAR(s32_Ilim_User)   = 0x7fffffff;
   BGVAR(s32_Ilim_Actual) = 0x7fffffff;
   BGVAR(u32_Mspeed) = 8947848;// 1000 rpm
   BGVAR(u16_Mpoles) = 4;
   BGVAR(u16_Motor_Foldback_Disable) = 1;

   SalDisableModeCommand(0LL,drive); // Preventing manipulation of Feedback Pointers by SetOpmode
   SalFdbkCommand(2LL,drive);          // FEEDBACKTYPE=2 (incremental encoder)
   SalMotorEncTypeCommand(0LL,drive);  // MENCTYPE=0 (A/B/I + Halls)

   BGVAR(s16_DisableInputs) &=~ENDAT_DIS_MASK;

   CalcImax(DRIVE_PARAM);
   CalcILim(DRIVE_PARAM);
   DesignRoutine(1,DRIVE_PARAM);

   LVAR(AX0_s32_Commutation_Ptr) = &Burnin_Comm_Pos;
   LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) = &VAR(AX0_s16_Burnin_Command);
   VAR(AX0_s16_Skip_Flags) |= ZERO_PHASE_ADV_MASK; // disable phase adv
   BGVAR(s32_BurninTimer) = Cntr_1mS;
   ResetEncState(DRIVE_PARAM);
   ClearFaultsCommand(DRIVE_PARAM);
}



void TerminateBurnin(int drive)
{
   // AXIS_OFF;
   int u16_temp_dismode = 0;

   BGVAR(u32_Mlmin)              = BGVAR(s32_ParamsBackup)[0];
   BGVAR(s32_Motor_I_Peak)       = BGVAR(s32_ParamsBackup)[1];
   BGVAR(s32_Motor_I_Cont)       = BGVAR(s32_ParamsBackup)[2];
   BGVAR(s16_Mcont_Current_Gain) = BGVAR(s32_ParamsBackup)[4];
   BGVAR(s16_Mpeak_Current_Gain) = BGVAR(s32_ParamsBackup)[5];
   BGVAR(s32_Ilim_User)          = BGVAR(s32_ParamsBackup)[6];
   BGVAR(u32_Mspeed)             = BGVAR(s32_ParamsBackup)[7];
   BGVAR(u16_Mpoles)             = BGVAR(s32_ParamsBackup)[8];
   BGVAR(s32_Icont)              = BGVAR(s32_ParamsBackup)[9];
   u16_temp_dismode              = BGVAR(s32_ParamsBackup)[10];
   BGVAR(s32_V_Lim_Design)       = BGVAR(s32_ParamsBackup)[11];
   BGVAR(s32_Ilim_Actual)        = BGVAR(s32_ParamsBackup)[12];
   BGVAR(u16_Motor_Foldback_Disable) = (int)BGVAR(s32_ParamsBackup)[13];

   SalFdbkCommand((long long)BGVAR(s32_ParamsBackup)[14],drive);          // Restore FEEDBACKTYPE
   SalMotorEncTypeCommand((long long)BGVAR(s32_ParamsBackup)[15],drive);  // Restore MENCTYPE
   SalDisableModeCommand((long long)u16_temp_dismode,drive);
   SetOpmode(drive, VAR(AX0_s16_Opmode));
   VAR(AX0_s16_Skip_Flags) &= ~ZERO_PHASE_ADV_MASK; // allow normal phase adv

   LVAR(AX0_s32_Commutation_Ptr) = &VAR(AX0_s16_Feedback_Comm_Pos);
   CalcImax(DRIVE_PARAM);
   CalcILim(DRIVE_PARAM);
   DesignRoutine(1,DRIVE_PARAM);
   ResetEncState(drive);
}


void BurninHandler(int drive)
{
   // AXIS_OFF;

   if (LVAR(AX0_s32_Crrnt_Lp_Inp_Ptr) != &VAR(AX0_s16_Burnin_Command)) return;

   if (PassedTimeMS(BGVAR(s32_BurninCycleTime), BGVAR(s32_BurninTimer)))
   {
      BGVAR(s32_BurninTimer) = Cntr_1mS;
   }

   if ((PassedTimeMS(BGVAR(s32_BurninOnTime), BGVAR(s32_BurninTimer))) || (!Enabled(drive)))
   {
      VAR(AX0_s16_Burnin_Command) = 0;
   }
   else VAR(AX0_s16_Burnin_Command) = 0x7FFF;
}


//**********************************************************
// Function Name: SalBurninCommand
// Description:
//          This function is called in response to the BURNIN command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalBurninCommand(long long lparam,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   if (BGVAR(s8_BurninParam) == (char)lparam)  return (SAL_SUCCESS);

   // Check if no other procedure is running in the drive
   if ((lparam != 0LL) && (ProcedureRunning(DRIVE_PARAM) != PROC_NONE)) return (OTHER_PROCEDURE_RUNNING);

   BGVAR(s8_BurninParam) = (char)lparam;
   if (BGVAR(s8_BurninParam) == 0) TerminateBurnin(DRIVE_PARAM);
   else InitBurnin(DRIVE_PARAM);

   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalBurninFrqCommand
// Description:
//          This function is called in response to the BURNINFRQ command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalBurninFrqCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   // internal*32000/65536=f[hz]
   BGVAR(u16_BurninFrq) = (unsigned int)param;
   AX0_Burnin_Delta = (int)((float)(BGVAR(u16_BurninFrq))*2.048);
   return (SAL_SUCCESS);
}

//**********************************************************
// Function Name: SalBurninCycleTimeCommand
// Description:
//          This function is called in response to the BURNINCYCLETIME command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalBurninCycleTimeCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s32_BurninCycleTime) = (long)param;
   BGVAR(s32_BurninTimer) = Cntr_1mS;
   return (SAL_SUCCESS);
}


//**********************************************************
// Function Name: SalBurninOnTimeCommand
// Description:
//          This function is called in response to the BURNINONTIME command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
int SalBurninOnTimeCommand(long long param,int drive)
{
   REFERENCE_TO_DRIVE;     // Defeat compilation error

   BGVAR(s32_BurninOnTime) = (long)param;
   BGVAR(s32_BurninTimer) = Cntr_1mS;
   return (SAL_SUCCESS);
}

