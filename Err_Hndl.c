//###########################################################################
//
// FILE: Err_Hndl.c
//
// TITLE:   This module contains functions implementing the error handler
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  0.01| 24 Jun 2002 | D.R. | Creation
//
//##################################################################


#include "Err_Hndl.def"
#include "Err_Hndl.var"
#include "Prototypes.pro"

/**********************************************************

* Function Name: SetMostRecentError
* Description:
   This function is called to register the occurrence of an error or a
   fault. The error/fault is then stored as the most recent error.
* Called From:
* Author: Hanan
* Input Parameters:
   error_code: the code of the error.
* Output Parameters: None
* Return Value: None
* Algorithm:
   The function first checks whether the specified error code is contained
   within the table of valid error codes. Only then is the error code
   copied to the Most Recent Error variable. If the code does not exist, no
   error message is given to the calling function.
* Global Variables Used:
   s16_Most_Recent_Error: write
* Revisions:
**********************************************************/
void SetMostRecentError(int drive,int error_code)
{
   /*---------- Local Variables ----------*/
   int i;         /* Loop counter */

   /*---------- Function Code ----------*/

   REFERENCE_TO_DRIVE;     // Defeat compilation error

/*
* Check that the error code is included in the list of valid error codes
* ----------------------------------------------------------------------
*/
   for (i = 0; i < NUMBER_OF_ERRORS; i ++)
   {
      if ( error_code == Error_Message[i].error_code )
      {
/*
* Error code has been identified in the table: register it
* --------------------------------------------------------
*/
         BGVAR(s16_Most_Recent_Error) = error_code;
         u16_Foe_ParamsFile_Parse_Result = error_code;
/*
* Clear the 's16_New_Error' so that it will be reported
* -------------------------------------------------
*/
         BGVAR(s16_New_Error) = 0;
/*
* Exit from search
* ----------------
*/
         break;
      }
   }
}

/**********************************************************
* Function Name: ReadMostRecentError
* Description:
   This function is called to read the value of the most recent error. The
   function returns both the value, and a pointer to the string
   associated with that error.
* Called From:
   communications processor
* Author: Hanan
* Input Parameters: None
* Output Parameters:
   *error_code: the most recent error
* Return Value:
   *error_string: pointer to a string containing the error message.
* Algorithm:
* Global Variables Used:
   s16_Most_Recent_Error: read
* Revisions:
**********************************************************/
char* ReadMostRecentError(int drive,int *error_code)
{
   /*---------- Local Variables ----------*/
   int i;               /* Loop counter */
   char *error_string;

   /*---------- Function Code ----------*/

   REFERENCE_TO_DRIVE;     // Defeat compilation error

/*
* Search for Most Recent Error in table
* -------------------------------------
*/
   for (i = 0; i < NUMBER_OF_ERRORS; i++)
   {
      if ( BGVAR(s16_Most_Recent_Error) == Error_Message[i].error_code )
      {
/*
* Error code has been identified in the table. Assign the pointer
* to point to the message string.
* ----------------------------------------------------------------
*/
         error_string = (char*)Error_Message[i].error_string;
/*
* Point to the most recent error
* ------------------------------
*/
         *error_code = BGVAR(s16_Most_Recent_Error);
/*
* Exit from search
* ----------------
*/
         break;
      }
   }
/*
* Check that the Most Recent Error was found in the table
* -------------------------------------------------------
*/
   if (i == NUMBER_OF_ERRORS )
   {
/*
* Most Recent Error was not found: we have a bug. Reset the s16_Most_Recent_Error
* ---------------------------------------------------------------------------
*/
      BGVAR(s16_Most_Recent_Error) = NO_ERROR;
      *error_code = BGVAR(s16_Most_Recent_Error);
/*
* Set the error string pointer to the NO_ERROR string
* ---------------------------------------------------
*/
      error_string = (char*)Error_Message[0].error_string;
   }
/*
* Return pointer to the error string
* ----------------------------------
*/
   return( error_string );
}

