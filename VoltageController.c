#include "MultiAxis.def"
#include "Err_Hndl.def"

#include "Extrn_Asm.var"
#include "Drive.var"

//**********************************************************
// Function Name: SalControllerTypeCommand
// Description:
//          This function is called in response to the CONTTYPE command.
//
//
// Author: Yuval
// Algorithm:
// Revisions:
//**********************************************************
/*int SalControllerTypeCommand(long lparam,int drive)
{
   int i=GenralDriveParamSalCommand(lparam,drive,1L,0L,&BGVAR(s16_ControllerType));
   if (i == SAL_SUCCESS)
   {
      CalcILim(DRIVE_PARAM);
      SalMContTAdvCommand(0L,drive);
      SalMPeakTAdvCommand(0L,drive);
      SetOpmode(drive,0);
   }
   return (i);
}

int VoltageControllerDesign(int drive)
{

  return 0;
}*/




