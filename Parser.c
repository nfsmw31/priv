#include <string.h>

#include "Parser.def"
#include "ser_comm.def"
#include "Err_Hndl.def"

#include "Parser.var"
#include "Drive_Table.var"
#include "Ser_Comm.var"

#include "Prototypes.pro"

//**********************************************************
// Function Name: SearchMnemonic
// Description:
//   This function is called to search for a specific mnemonic in the
//   drive table. The mnemonic to be searched for is in the
//   Mnemonic_Code variable.
//
//
// Author: Dimitry
// Algorithm:
// Return Value:
//   0: Mnemonic not found in table
//   else: Mnemonic index in table +1
// Revisions:
//**********************************************************
int SearchMnemonic(long code1,long code2, long code3)
{
   int i;

   for (i = 0; i < COMMANDS_TABLE_SIZE; i++)
   {
      if ((Commands_Table[i].u32_mnemonic_code1 == code1) &&
          (Commands_Table[i].u32_mnemonic_code2 == code2) &&
          (Commands_Table[i].u32_mnemonic_code3 == code3) )
      {
         return (i + 1);
      }
   }
   return 0;
}


//**********************************************************
// Function Name: ParseMnemonic
// Description:
//   This function is called to parse the mnemonic.
//
//
// Author: Dimitry
// Algorithm:
// Return Value:
//   0: Mnemonic parsing not yet finished
//   1: Syntax error found in mnemonic
//   2: Mnemonic OK: no parameters
//   3: Mnemonic OK: with parameters
//   4: Undefined mnemonic
//   5: NULL command
//   6: Not programmable (Mnemonic does not allow parameters)
// Revisions:
//**********************************************************
char ParseMnemonic(void)
{
    int index;
    char return_value = 0;
    //  Last character read by the mnemonic syntax checker
    char last_char_read;

    //  Select the state of the mnemonic parsing
    switch(u8_Parse_Mnemonic_State)
    {
        case CHECK_MNEMONIC_SYNTAX:
        {
            //  Point to the beginning of the message buffer
            p_u8_Message_Buffer_Ptr = u8_Message_Buffer;

            //  Check the first character of the message buffer
            if (*(p_u8_Message_Buffer_Ptr) == END_OF_LINE)
            {
                return NULL_COMMAND;
            }

            //  Advance Buffer pointer
            p_u8_Message_Buffer_Ptr++;

            //  Check the mnemonic syntax and get the last character read
            if (CheckMnemonicSyntax(&last_char_read))
            {
                //  Examine the last character read
                if (last_char_read == NULL)
                {
                    //  Indicate that no parameters exist
                    u8_Parameters_Exist = 0;
                    //  Change parser state to identify the mnemonic
                    u8_Parse_Mnemonic_State = IDENTIFY_MNEMONIC;
                }
                else if ((last_char_read == SPACE) || (last_char_read == ASSIGNMENT))
                {
                    //  Indicate that parameters exist
                    u8_Parameters_Exist = 1;
                    //  Change parser state to identify the mnemonic
                    u8_Parse_Mnemonic_State = IDENTIFY_MNEMONIC;
                }
                //  All other characters
                else
                {
                    //  Indicate that a syntax error was found
                    return MNEMONIC_SYNTAX_ERROR;
                }
            }
            else
            {
                //  Indicate that a syntax error was found
                return MNEMONIC_SYNTAX_ERROR;
            }

            break;
        }
        case IDENTIFY_MNEMONIC:
        {
            //  Search for mnemonic in table
            index = SearchMnemonic(u32_Global_Mnemonic_Code1,u32_Global_Mnemonic_Code2,u32_Global_Mnemonic_Code3);

            //  Validate mnemonic was found in table
            if (!index)
            {
                //  Mnemonic search was unsuccessful
                return_value = MNEMONIC_UNDEFINED;
            }
            else
            {
                //  Update the command index value
                s16_Command_Index = index - 1;

                //  Validate Mnemonic has parameters
                if (u8_Parameters_Exist)
                {
                    //  One or more parameters were sent in the message: check if parameters are allowed
                    if ((MIN_WRITE_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg) == 0) &&
                        (MAX_READ_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg) == 0))
                    {
                        //  Parameters not allowed: indicate not allowed
                        return_value = MNEMONIC_PARAMS_NOT_ALLOWED;
                    }
                    else
                    {
                        return_value = MNEMONIC_OK_WITH_PARAMS;
                    }
                }
                else
                {
                    //  No parameters were sent in the message: check if parameters are required
                    if (MIN_READ_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg) > 0)
                    {
                        //  Parameters are required: indicate a syntax error
                        return_value = MNEMONIC_SYNTAX_ERROR;
                    }
                    else
                    {
                        //  Set the number of parameters to zero: used by the execution handler
                        s16_Number_Of_Parameters = 0;
                        return_value = MNEMONIC_OK_NO_PARAMS;
                    }
                }
            }

            //  Reset mnemonic parser state
            u8_Parse_Mnemonic_State = CHECK_MNEMONIC_SYNTAX;
            break;
        }
    }

    return return_value;
}


//**********************************************************
// Function Name: ConvertCode2Mnemonic
// Description:
//   This function converts the mnemonic to binary code.
//
//
// Author: D.R
// Algorithm:
// Return Value:
//   Mnemonics lengh in characters
//
// Revisions:
//**********************************************************
void ConvertCode2Mnemonic(char* p_u8_str,long code1,long code2, long code3)
{
   char data_byte;                     /* Character read from message buffer */
   int  byte_count,i = 0;              /* Counts the number of bytes read */
   char temp[16];
   int count;

    for (byte_count = 0; byte_count < 15; byte_count++)
   {
      // Convert MNEMONIC to code
      if (byte_count < 5)    data_byte = (code1 >> (long)(byte_count * 6)) & 0x003F;
      else if (byte_count < 10) data_byte = (code2 >> (long)((byte_count - 5) * 6)) & 0x003F;
      else if (byte_count < 15) data_byte = (code3 >> (long)((byte_count - 10) * 6)) & 0x003F;

      temp[byte_count] = data_byte;
      if (data_byte > 0) temp[byte_count] += 47;
      else break;
   }
   temp[byte_count + 1] = 0;

   count = byte_count;
   if (count > 5) count = 5;
   //invert the string of code 1
   for (i = 0; i < count ;i++)
      p_u8_str[i] = temp[count - i - 1];

   if (byte_count > 5)
   {
      count = byte_count - 5;
      if (count > 5) count = 5;
      //invert the string of code 2
      for (i = 0; i < count; i++)
         p_u8_str[i + 5] = temp[count + 5 - i - 1];
   }

   if (byte_count > 10)
   {
      count = byte_count - 10;
      if (count > 5) count = 5;
      //invert the string of code 3
      for (i = 0; i < count; i++)
         p_u8_str[i+10] = temp[count+10-i-1];
   }

   p_u8_str[byte_count] = 0;//terminate the string
}


//**********************************************************
// Function Name: ConvertMnemonic2Index
// Description:
//   Find index by command name (Mnemonic)
//**********************************************************
int ConvertMnemonic2Index(char * Cmd_Name)
{
   long code1, code2, code3, index;
   ConvertMnemonic2Code(Cmd_Name, &code1, &code2, &code3, 0);
   index = SearchMnemonic(code1, code2, code3);
   if (!index) return 0; //not found
   return(index-1);
}


//**********************************************************
// Function Name: ConvertMnemonic2Code
// Description:
//   This function converts the mnemonic to binary code.
//
// S.F: Added a password bypass mechanism for SST: if first and last letter in the 
//    Mnemonic name are upper-case and all the rest lower-case, password protection will 
//    overrided.
//
// Author: D.R
// Algorithm:
// Return Value:
//   Mnemonics lengh in characters
//
// Revisions: In order to support motorname conversion as well, added u16_type to specify it "_" or "-"
// are allowed in the name
//**********************************************************
int ConvertMnemonic2Code(char* p_u8_str,long* code1,long* code2, long* code3, unsigned int u16_type)
{
   char data_byte;                   /* Character read from message buffer */
   int  byte_count = 0;              /* Counts the number of bytes read */
   int  s16_control = 1;
   int s16_first_and_last_loc = 1;
   unsigned long temp_long;

   *code1 = 0L;
   *code2 = 0L;
   *code3 = 0L;

   s16_password_override = 1;
   // DO until the mnemonic syntax check is done ...
   do {
      // Read character from message buffer and increment pointer to the buffer
      data_byte = *p_u8_str;
      p_u8_str++;

      byte_count++;      // Increment the byte count

      if (byte_count > 16) 
      {
         s16_password_override = 0;
         return 0;      // Check if too many bytes have been read
      }

      // Check for last char in mnemonic
      if ((*p_u8_str == ' ') || (*p_u8_str == 0) || (*p_u8_str == '='))
         s16_first_and_last_loc = 1;
      // Check if only first and last char in mnamonic is upper-case and all the rest lower-case to override password protection
      if ((data_byte >= 'a' && data_byte <= 'z') || (data_byte >= 'A' && data_byte <= 'Z'))
      {
         if (s16_first_and_last_loc && (data_byte >= 'a' && data_byte <= 'z'))
            s16_password_override = 0;
         else if ((s16_first_and_last_loc==0) && (data_byte >= 'A' && data_byte <= 'Z'))  
            s16_password_override = 0;
      }
      
      s16_first_and_last_loc = 0;
      
      // Check that the character is an alphabetic character
      if (data_byte >= 'a' && data_byte <= 'z')
      {
         data_byte -= 0x20;         // Convert to upper case
      }
      
      if ( ((data_byte >= 'A') && (data_byte <= 'Z')) || ((data_byte >= '0') && (data_byte <= '9')) ||
           ((u16_type == 1) && ((data_byte == '_')    || (data_byte == '-')))                         )
      {
         temp_long = (unsigned long)data_byte;
         // Convert MNEMONIC to code
         if (byte_count <= 5)
         {
            *code1 <<= 6;
            *code1 += (temp_long - 47);
         }
         else if (byte_count <= 10)
         {
            *code2 <<= 6;
            *code2 += (temp_long - 47);
         }
         else if (byte_count <= 15)
         {
            *code3 <<= 6;
            *code3 += (temp_long - 47);
         }
      }
      else
      {
         // Return the last character that was read
         if (byte_count > 0) return byte_count - 1;
         else return byte_count;
      }
   }
   while (s16_control == 1);     // to defeat compiler remark. Originally it was "while(1);"
   return (0);                   // to defeat compiler warning
}


//**********************************************************
// Function Name: CheckMnemonicSyntax
// Description:
//   This function checks the sysntax of the mnemonic.
//
//
// Author: Dimitry
// Algorithm:
// Return Value:
//   0: Syntax error: too many characters
//   1: Syntax check OK
// Revisions:
//**********************************************************
char CheckMnemonicSyntax(char *last_char_read)
{
   char  data_byte;                   /* Character read from message buffer */
   int mnem_count = 0;

   mnem_count = ConvertMnemonic2Code(u8_Message_Buffer,&u32_Global_Mnemonic_Code1,&u32_Global_Mnemonic_Code2, &u32_Global_Mnemonic_Code3, 0);

   p_u8_Message_Buffer_Ptr = &u8_Message_Buffer[mnem_count]; //point to next character after mnemonics

   data_byte = *p_u8_Message_Buffer_Ptr;
   // Return the last character that was read
    *last_char_read = data_byte;

   // If the last character read was SPACE or ASSIGNMENT, read and ignore
   //any further SPACE/ASSIGNMENT characters from the buffer
   while ( (data_byte == SPACE) || (data_byte == ASSIGNMENT) )
   {
      // Check if the character being pointed to is SPACE or ASSIGNMENT
      data_byte = *p_u8_Message_Buffer_Ptr;
      // Increment the message buffer pointer if SPACE/ASSIGNMENT
      if ( (data_byte == SPACE) || (data_byte == ASSIGNMENT) )
      {
         p_u8_Message_Buffer_Ptr ++;
      }
      else
      {
         // If the character is NULL, return it as the last character.
         // This enables a query with trailing SPACE/ASSIGNMENT characters.
         if (data_byte == NULL) *last_char_read = data_byte;
      }
   }

   return 1;
}

//**********************************************************
// Function Name: RemoveRedundantSpaces
// Description:
//   This function removes redundant spaces from the command.
//
//
// Author: Daniel.G
// Algorithm:
//  Removes redundant spaces from the message buffer.
// Return Value:    None
// Revisions:
//**********************************************************
void RemoveRedundantSpaces()
{
    char current_char_read;
    char u8_first_space = 1;
    int index = 0;
    int copy_index = 0;

    //  Point to the beginning of the message buffer
    char* p_u8_Message_Buf_Ptr = u8_Message_Buffer;
    char* p_u8_Message_Buf_End_Ptr = p_u8_Message_Buf_Ptr + strlen(p_u8_Message_Buf_Ptr) - 1;

    //  Run on spaces before command
    while (p_u8_Message_Buf_Ptr[index] == SPACE)
    {
        //  Advance to next char
        index++;
    }

    //  Run on spaces at the end of the command
    while (*(p_u8_Message_Buf_End_Ptr) == SPACE)
    {
        //  Set the last space as EOL
        *(p_u8_Message_Buf_End_Ptr) = END_OF_LINE;
        //  Regress the buffer end
        p_u8_Message_Buf_End_Ptr--;
        //  Validate there are only spaces in the command
        if ((p_u8_Message_Buf_Ptr + index) == p_u8_Message_Buf_End_Ptr)
        {
            return;
        }
    }

    //  Run on whole message buffer
    while (p_u8_Message_Buf_Ptr[index] != END_OF_LINE)
    {
        //  Get current char
        current_char_read = p_u8_Message_Buf_Ptr[index];
        //  Copy letter over
        p_u8_Message_Buf_Ptr[copy_index] =  p_u8_Message_Buf_Ptr[index];

        //  Validate char is space
        if (current_char_read == SPACE)
        {
            //  Validate first occurrence of space
            if (u8_first_space)
            {
                //  Lower flag so that any trailing spaces would be overwritten
                u8_first_space = 0;
                //  Advance copy index to leave current space
                copy_index++;
            }

            //  Advance index
            index++;
        }
        else
        {
            //  Raise flag to reset space control
            u8_first_space = 1;
            //  Advance index
            index++;
            //  Advance copy index
            copy_index++;
        }
    }

    //  Set new end of line
    p_u8_Message_Buf_Ptr[copy_index] = END_OF_LINE;
}

//**********************************************************
// Function Name: ParserManager
// Description:
//   This function manages the parsing process.
//
//
// Author: Dimitry
// Algorithm:
// Return Value:
//   0: Parsing not yet completed
//   1: Parsing completed without errors
//   2: the Parser was executed, but the mnemonic was invalid
//   3: the Parser was executed, but the syntax was incorrect
//   4: a parameter was invalid (either out of range or of the wrong type)
//   5: The NULL command was received
//   6: Valid mnemonic, but not programmable
// Revisions:
//**********************************************************
char ParserManager(int drive)
{
    //  Function return value
    char return_value = 0;
    //  Result of parsing process
    char parser_result = 0;

    if (!BGVAR(s16_Command_Parsed))
    {
        // Remove redundant spaces
        RemoveRedundantSpaces();
        //  Set command was parsed
        BGVAR(s16_Command_Parsed) = 1;
    }

    //  Select parser state
    switch(u8_Parser_State)
    {
        case PARSE_MNEMONIC:
        {
            //  Parse Mnemonic
            parser_result = ParseMnemonic();

            switch (parser_result)
            {
                case MNEMONIC_PARSE_NOT_FINISHED:
                {
                    // Mnemonic parsing not yet finished: do nothing
                    break;
                }
                case MNEMONIC_SYNTAX_ERROR:
                {
                    //  Reset command parsed
                    BGVAR(s16_Command_Parsed) = 0;
                    //  Indicate that a syntax error was found
                    return_value = 0x03;
                    break;
                }
                case MNEMONIC_OK_NO_PARAMS:
                {
                    //  Reset command parsed
                    BGVAR(s16_Command_Parsed) = 0;
                    //  Keep parser state in PARSE_MNEMONIC
                    u8_Parser_State = PARSE_MNEMONIC;
                    return_value = 0x01;
                    break;
                }
                case MNEMONIC_OK_WITH_PARAMS:
                {
                    //  Set next parser state
                    u8_Parser_State = PARSE_PARAMETERS;
                    //  Initialize the number of parameters received
                    s16_Number_Of_Parameters = 0;
                    s16_String_Parameter_Counter = 0;
                    break;
                }
                case MNEMONIC_UNDEFINED:
                {
                    //  Reset command parsed
                    BGVAR(s16_Command_Parsed) = 0;
                    //  Keep parser state in PARSE_MNEMONIC
                    u8_Parser_State = PARSE_MNEMONIC;
                    return_value = 0x02;
                    break;
                }
                case NULL_COMMAND:
                {
                    //  Reset command parsed
                    BGVAR(s16_Command_Parsed) = 0;
                    //  Keep parser state in PARSE_MNEMONIC
                    u8_Parser_State = PARSE_MNEMONIC;
                    return_value = 0x05;
                    break;
                }
                case MNEMONIC_PARAMS_NOT_ALLOWED:
                {
                    //  Reset command parsed
                    BGVAR(s16_Command_Parsed) = 0;
                    //  Keep parser state in PARSE_MNEMONIC
                    u8_Parser_State = PARSE_MNEMONIC;
                    //  Indicate not programmable
                    return_value = 0x06;
                    break;
                }
                default:
                {
                    //  Reset command parsed
                    BGVAR(s16_Command_Parsed) = 0;
                    // Indicate that a syntax error was found
                    return_value = 0x03;
                }
            }

            break;
        }
        case PARSE_PARAMETERS:
        {
            //  Parse the parameters
            parser_result = ParseParameters(drive);

            switch( parser_result )
            {
                case 0:
                {
                    // Parameter parsing not yet finished: do nothing
                    break;
                }
                case 1:     /* Parameters sucessfully parsed */
                {
                    u8_Parser_State = PARSE_MNEMONIC;           // Return parser state to PARSE_MNEMONIC
                    //  Reset command parsed
                    BGVAR(s16_Command_Parsed) = 0;
                    return_value = 0x01;
                    break;
                }
                case 2:     /* Syntax error */
                {
                    return_value = 0x03;               // Indicate that a syntax error was found

                    u8_Parser_State = PARSE_MNEMONIC;   // Return parser state to PARSE_MNEMONIC
                    //  Reset command parsed
                    BGVAR(s16_Command_Parsed) = 0;
                    break;
                }
                case 3:     /* Parameter out of range */
                {
                    return_value = 0x04;

                    u8_Parser_State = PARSE_MNEMONIC;           // Return parser state to PARSE_MNEMONIC
                    //  Reset command parsed
                    BGVAR(s16_Command_Parsed) = 0;
                    break;
                }
            }

            break;
        }
    }

    return return_value;
}


char ScriptParserManager(int drive, int order)
{
      char return_value = 0;            /* Function return value */
      char parser_result = 0;           /* Result of parsing process */


   while ((parser_result = ParseMnemonic()) == 0);

   if (parser_result != 2 && parser_result != 3)
      return 3;
   if (parser_result!=2)
   {
      s16_Number_Of_Parameters = 0;
      s16_String_Parameter_Counter = 0;

      while ((parser_result = ParseParameters(drive)) == 0);
      if (parser_result !=1)
         return 3;

      // Detect if it is a read or write command
      if ((MIN_READ_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg) <= s16_Number_Of_Parameters) && (MAX_READ_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg) >= s16_Number_Of_Parameters))
         return_value = ValidateSerialRead(s16_Command_Index, drive);
      else
         // Call the validation check function
         return_value = ValidateSerialWrite(s16_Command_Index, drive);
      if (return_value != SAL_SUCCESS && return_value != DRIVE_INACTIVE)
         return 3;
   }

   if (order == 0)
      return 1;

   while ( (return_value = ExecutionHandler()) == 0 );
   if ( return_value!=SAL_SUCCESS)
      return return_value;
    return 1;
}


/**********************************************************
* Function Name: ParseParameters
* Description:
   This function is called to parse the parameters. This involves
   - checking the syntax of the parameters
   - translating the parameter from ASCII to integer
* Called From:
   ParserManager/parser.c
* Author: Hanan
* Input Parameters: None
* Output Parameters: None
* Return Value:
   0: Parameter parsing not yet finished
   1: Parameters were sucessfully parsed
   2: A syntax error was found
   3: The value is out of range (greater than the maximum signed long value)
* Algorithm:
   The function is executed once for each parameter in the message. The
   syntax is checked. Numeric parameters are translated from ASCII to
   long integers, and are stored in the 'Execution_Parameters[]' array.
   On entry to the function, the 'p_u8_Message_Buffer_Ptr' points to the first
   character of the parameter.
* Global Variables Used:
   p_u8_Message_Buffer_Ptr
   s16_Number_Of_Parameters
   Execution_Parameters
   s_Mnemonic_Table[s16_Mnemonic_Index]
* Revisions:
**********************************************************/
char ParseParameters(int drive)
{
   char *u8_Execution_String_Ptr;
   char return_value;            /* Function return value */
   char data_byte;               /* Character read from the message buffer */
   char parameter_type = 0;      /* Parameter type: 0 = numeric; 1 = string */
   char parameter_read;          /* Flag used to indicate that the entire parameter has been read */
   char number_of_characters;    /* number of characters in the parameter */
   int loop_counter=0;

   REFERENCE_TO_DRIVE;     // Defeat compilation error

   return_value = 0x0;

   // Check that the number of parameters received does not exceed the maximum
   if ( ((s16_Number_Of_Parameters + 1) > (MAX_WRITE_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg))) &&
        ((s16_Number_Of_Parameters + 1) > (MAX_READ_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg)))    )
   {
      return_value = 0x02;      // Indicate a syntax error
   }
   else
   {
      // SECTION 1: check the parameter syntax

      // Read the first character of the parameter and increment
      // the pointer to the message buffer.
      data_byte = *(p_u8_Message_Buffer_Ptr++);

      switch( data_byte )      // Select the parameter type
      {
         case DOUBLE_QUOTE:
            // Check that a string parameter is allowed
            if (M_PARAM_TYPE(Commands_Table[s16_Command_Index].u32_argument_types,s16_Number_Of_Parameters) == 3)
               parameter_type = STRING_PARAMETER;
            else
            {
               return_value = 0x02;
            }
         break;

         case 'h':
         case 'H':
            // Hexadecimal data prefix
            if (( M_PARAM_TYPE(Commands_Table[s16_Command_Index].u32_argument_types,s16_Number_Of_Parameters) == 1 ) ||
                ( M_PARAM_TYPE(Commands_Table[s16_Command_Index].u32_argument_types,s16_Number_Of_Parameters) == 2 ))
            {
               parameter_type = HEX_PARAMETER;               // Hexadecimal parameter
            }
            else
            {
               return_value = 0x02;               // Syntax error
            }
         break;

         default:
            // Check if numeric parameter
            if ( (data_byte == MINUS_SIGN || data_byte == PLUS_SIGN) ||
                (data_byte >= ASCII_ZERO && data_byte <= ASCII_NINE) ||
                (data_byte == DOT)                                     )
            {
               if (M_PARAM_TYPE(Commands_Table[s16_Command_Index].u32_argument_types,s16_Number_Of_Parameters) < 3 )
               {
                  if (M_PARAM_TYPE(Commands_Table[s16_Command_Index].u32_argument_types,s16_Number_Of_Parameters) == 2)
                  {
                     parameter_type = FLOAT_PARAMETER;
                     if (data_byte == DOT)
                     {
                        u8_Number_Of_Dots_In_Parameter++;
                     }
                  }
                  else
                  {
                     if (data_byte == DOT)
                     {
                         return_value = 0x02;
                     }
                     else parameter_type = NUMERIC_PARAMETER;
                  }
               }
               else
               {
                  return_value = 0x02;                  // Syntax error
               }
            }
            else
            {
               return_value = 0x02;  // Syntax error
            }
         break;
      }

      // Continue parsing only if a syntax error has not been detected yet
      if (return_value == 0x0)
      {
         // Initialize the parameter buffer
         p_u8_Parameter_Buffer_Ptr = u8_Parameter_Buffer;

         // Put the first character of the NUMERIC parameter
         // into the parameter buffer. The first character of STRING or HEX
         // parameters do not form part of the actual parameter.
         if ((parameter_type == NUMERIC_PARAMETER) || (parameter_type == FLOAT_PARAMETER))
         {
            *(p_u8_Parameter_Buffer_Ptr ++) = data_byte;
            number_of_characters = 0x01;     // Indicate that one character has been read
         }
         else
         {
            number_of_characters = 0x00;     // Indicate that no characters have been read
         }

         parameter_read = 0;         // Indicate that the parameter has not been entirely read

         // Now read the rest of the parameter characters from the message buffer
         do {
            // Read the next character of the parameter and increment
            // the pointer to the message buffer.
            data_byte = *(p_u8_Message_Buffer_Ptr ++);

            // Select the parameter type
            switch(parameter_type)
            {
               case NO_PARAMETER:
               break;

               case STRING_PARAMETER:  /* String */
                  // Examine the character
                  switch(data_byte)
                  {
                     case SPACE:    /* Parameter separator */
                        // Terminate the parameter buffer

                        if (u32_Global_Mnemonic_Code2 != 0x245236A1 && u32_Global_Mnemonic_Code3 != 0x25)
                        {
                           // Terminate the parameter buffer
                           *p_u8_Parameter_Buffer_Ptr = '\0';
                           // Indicate that the parameter has been read
                           parameter_read = 1;
                        }
                        else
                        {
                           *(p_u8_Parameter_Buffer_Ptr ++) = data_byte;
                           number_of_characters++;
                        }
                     break;

                     case NULL:     /* End of message indicator */
                        *p_u8_Parameter_Buffer_Ptr = '\0';           // Terminate the parameter buffer

                        parameter_read = 1;         // Indicate that the parameter has been read

                        // Check that the minimum number of parameters has been received
                        if ( (s16_Number_Of_Parameters + 1) < MIN_READ_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg)      ||
                             ( (s16_Number_Of_Parameters + 1) > MAX_READ_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg) &&
                               (s16_Number_Of_Parameters + 1) < MIN_WRITE_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg)  )  )
                        {
                           return_value = 0x02;                       // Indicate a syntax error
                        }
                     break;

                     default:
                        if (data_byte >= 'a' && data_byte <= 'z')      // Check for lower-case character
                        {
                           data_byte -= 0x20;                           // Convert to upper case
                        }
                        // Allow any character from # (22H) to Z
                        if ((data_byte >= '#' && data_byte <= 'Z') || data_byte == '~' || data_byte == '_')
                        {
                           // Increment the number of characters in the parameter
                           number_of_characters ++;

                           // Check that not more than 20 characters have been read /
                           // excecpt at script cmd when 64 characters can be read  /
//                           if (number_of_characters > (( u32_Global_Mnemonic_Code2==0x245236A1 &&  u32_Global_Mnemonic_Code3==0x25)?(char)128:15))
                           if (number_of_characters > (( u32_Global_Mnemonic_Code2==0x245236A1 &&  u32_Global_Mnemonic_Code3==0x25) ? (char)128 : 20))
                           {
                              return_value = 0x02;                              // Syntax error
                           }
                           else
                           {
                              // Write character to parameter buffer
                              *(p_u8_Parameter_Buffer_Ptr ++) = data_byte;
                           }
                        }
                        else
                        {
                           return_value = 0x02;    // Non-string character: syntax error
                        }
                     break;
                  }
               break;

               case NUMERIC_PARAMETER:  /* Numeric */
               case FLOAT_PARAMETER:
                  // Examine the character
                  switch(data_byte)
                  {
                     case SPACE:    /* Parameter separator */
                        *p_u8_Parameter_Buffer_Ptr = '\0';           // Terminate the parameter buffer

                        // end of the parameter reset the decimal DOT counter
                        u8_Number_Of_Dots_In_Parameter = 0;

                        parameter_read = 1;      // Indicate that the parameter has been read

                        // Check that the string entered for the numeric parameter does not
                        // exceed the maximum signed 'long' value'. If it does, flag
                        // an 'Out of range' error
                        if ( CheckMaxNumericValue(number_of_characters) != 0 )
                        {
                           return_value = 3;
                        }
                     break;

                     case NULL:     /* End of message indicator */
                        *p_u8_Parameter_Buffer_Ptr = '\0';     // Terminate the parameter buffer

                        // end of parameters list reset the decimal DOT counter
                        u8_Number_Of_Dots_In_Parameter = 0;

                        parameter_read = 1;        // Indicate that the parameter has been read

                        // Check for only 3 digits after the decimal point
                        if (Check3DigitsAfterDecPoint(number_of_characters) != 0 )
                        {
                           return_value = 0x02;
                        }

                        // Check that the minimum number of parameters has been received
                        if ( (s16_Number_Of_Parameters + 1) < MIN_READ_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg)      ||
                             ( (s16_Number_Of_Parameters + 1) > MAX_READ_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg) &&
                               (s16_Number_Of_Parameters + 1) < MIN_WRITE_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg)  )  )
                        {
                           return_value = 0x02;                        // Indicate a syntax error
                        }
                        else
                        {
                           // Check that the string entered for the numeric parameter does not
                           // exceed the maximum signed 'long' value'. If it does, flag
                           // an 'Out of range' error
                           if ( CheckMaxNumericValue(number_of_characters) != 0 )
                           {
                              return_value = 3;
                           }
                        }
                     break;

                     default:
                        // Check for numeric character
                        if ((data_byte >= ASCII_ZERO && data_byte <= ASCII_NINE) || ((parameter_type == FLOAT_PARAMETER) && (data_byte == DOT)))
                        {
                           if (data_byte == DOT)
                           {
                              u8_Number_Of_Dots_In_Parameter++;
                              if(u8_Number_Of_Dots_In_Parameter > 1)
                              {
                                 u8_Number_Of_Dots_In_Parameter=0;
                                 // Syntax error more the 1 decimal point in parametr !
                                 return_value = 0x02;
                              }
                           }
                           number_of_characters ++; // Increment the number of characters in the parameter

                           // Check that not more than 15 characters have been read.
                           // This allows for 10+4 (X.DDD) characters for the value and one for the sign.
                           if (number_of_characters > (char)24)
                           {
                              // Syntax error
                              return_value = 0x02;
                           }
                           else
                           {
                              // Write character to parameter buffer
                              *(p_u8_Parameter_Buffer_Ptr ++) = data_byte;
                           }
                        }
                        else
                        {
                           return_value = 0x02;            // Non-numeric character: syntax error
                        }
                     break;
                  }
               break;

               case HEX_PARAMETER:
                  switch(data_byte)                  // Examine the character
                  {
                     case SPACE:    /* Parameter separator */
                        *p_u8_Parameter_Buffer_Ptr = '\0';    // Terminate the parameter buffer

                        parameter_read = 1;                  // Indicate that the parameter has been read
                     break;

                     case NULL:     /* End of message indicator */
                        *p_u8_Parameter_Buffer_Ptr = '\0';    // Terminate the parameter buffer

                        parameter_read = 1;              // Indicate that the parameter has been read

                        // Check that the minimum number of parameters has been received
                        if ( (s16_Number_Of_Parameters + 1) < MIN_READ_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg)      ||
                             ( (s16_Number_Of_Parameters + 1) > MAX_READ_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg) &&
                               (s16_Number_Of_Parameters + 1) < MIN_WRITE_PARAMS(Commands_Table[s16_Command_Index].u16_min_max_arg)  )  )
                        {
                           return_value = 0x02;                           // Indicate a syntax error
                        }
                     break;

                     default:
                        // Check for Hexadecimal characters
                        if ( (data_byte >= ASCII_ZERO && data_byte <= ASCII_NINE) ||
                             (data_byte >= ASCII_A && data_byte <= ASCII_F)       ||
                             (data_byte >= 'a' && data_byte <= 'f')                 )
                        {
                           number_of_characters ++; // Increment the number of characters in the parameter
                           // Check that not more than 8 characters have been read
                           if (number_of_characters > (char)16)
                           {
                              return_value = 0x02;                    // Syntax error
                           }
                           else
                           {
                              // Convert lower case alphabetical characters to upper case
                              if (data_byte >= 'a' && data_byte <= 'z')
                              {
                                 data_byte -= 0x20;
                              }
                              // Write character to parameter buffer
                              *(p_u8_Parameter_Buffer_Ptr ++) = data_byte;
                           }
                        }
                        else
                        {
                           return_value = 0x02;                 // Non-hex character: syntax error
                        }
                     break;
                  }
               break;
            }
         }
         while (!parameter_read && (return_value == 0x0) );

         // SECTION 2: translate the parameter from ASCII to long integer
         if (return_value == 0x0)
         {
            switch( parameter_type )            // Select the parameter type
            {
               case NO_PARAMETER:
               break;

               case STRING_PARAMETER:
                  if ( data_byte == NULL )
                  {
                     return_value = 0x01;       // Indicate that the parameter has been parsed
                  }
                  else
                  {
                     return_value = 0x0;
                  }

                  // Store the string parameter
                  p_u8_Parameter_Buffer_Ptr = u8_Parameter_Buffer;
                  loop_counter = 0;

                  u8_Execution_String_Ptr = (u32_Global_Mnemonic_Code2 == 0x245236A1 && u32_Global_Mnemonic_Code3 == 0x25) ? &BGVAR(u8_Execution_String_Script)[0] : &u8_Execution_String[s16_String_Parameter_Counter][loop_counter];
                  do {
                     data_byte = *(p_u8_Parameter_Buffer_Ptr ++);
                     *(u8_Execution_String_Ptr++)=data_byte;
                  } while ( data_byte != '\0' );

                  // Increment the number of parameters received
                  s16_Number_Of_Parameters ++;
                  s16_String_Parameter_Counter ++;
               break;

               case NUMERIC_PARAMETER:
               case FLOAT_PARAMETER:
                  // Convert the string to a long long integer
                  s64_Execution_Parameter[s16_Number_Of_Parameters] = AsciiToUnsignedLongLong(u8_Parameter_Buffer, parameter_type==FLOAT_PARAMETER);

                  // Increment the number of parameters received
                  s16_Number_Of_Parameters ++;
                  if ( data_byte == NULL)
                     return_value = 0x01; // Indicate that the parameter has been parsed
                  else
                     return_value = 0x0;
               break;

               case HEX_PARAMETER:
                  // Convert the string to a long integer
                  s64_Execution_Parameter[s16_Number_Of_Parameters] = HexToLongLong(u8_Parameter_Buffer);

                  if (M_PARAM_TYPE(Commands_Table[s16_Command_Index].u32_argument_types,s16_Number_Of_Parameters) == 2)
                  {
                     s64_Execution_Parameter[s16_Number_Of_Parameters] *= 1000;
                  }

                  s16_Number_Of_Parameters ++;     // Increment the number of parameters received
                  if ( data_byte == NULL )
                  {
                     return_value = 0x01;          // Indicate that the parameter has been parsed
                  }
                  else
                  {
                     return_value = 0x0;
                  }
               break;
            }
         }
      }
   }
   return( return_value );
}


/**********************************************************
* Function Name: AsciiToUnsignedLongLong
* Description:
   This function translate an ASCII numeric string to a long long integer.
   Type will dictate if the value should be float or integer (resulting a *1000 factor)
* Called From:
   ParseParameters/parser.c
* Author: Hanan
* Input Parameters:
   *string: pointer to string to be converted
* Output Parameters: None
* Return Value:
   the translated value
* Algorithm:
   Starting from the last character in the string: read the character,
   subtract 0x30 to convert it to a digit, and then multiply by 10^i, where
   'i' is used to counter the digits read. If the character read is '+' or '-'
   it is not multiplied: it represents the sign of the value.
* Global Variables Used:
* Revisions:
**********************************************************/
long long AsciiToUnsignedLongLong(char *string, int float_type)
{
   long long  result = 0LL, multiplier = 1LL, decimal_mult = 1LL;
   int  digit;
   char *str_ptr;             /* Pointer to the string in which the parameter is stored */
   int  translation_done = 0;  /* Flag used to indicate that the translation is finished */
   char data_byte;            /* Character read from parameter buffer */

   if (float_type)
   {
      decimal_mult = 1000L;

      // Remove more than 3 digits after decimal point
      str_ptr = string + strlen(string) - 1;
      digit = 1;
      while (str_ptr >= string)
      {
         digit++;
         if ((*str_ptr == DOT) && (digit >= 5))
         {
            *(str_ptr + 4) = '\0';
            break;
         }
         str_ptr--;
      }
   }

   str_ptr = string + strlen(string) - 1;   // Point to the end of the string
   digit = 0;

   do {
      digit++;

      data_byte = *(str_ptr--);      // Read character from buffer

      // Check for a sign character
      if (data_byte == MINUS_SIGN || data_byte == PLUS_SIGN)
      {
         if (data_byte == MINUS_SIGN)      // Multiply result by -1 if the data_byte is a minus sign
         {
            result *= -1;
         }

         translation_done = 1;         // Set flag to exit from loop
      }
      else
      {
         if (data_byte == DOT)
         {
            // When decimal point specified return the value multiplied by 1000
            if (digit == 2) decimal_mult = 100;
            else if (digit == 3) decimal_mult = 10;
            else if (digit == 4) decimal_mult = 1;

            if (str_ptr < string) translation_done = 1;
         }
         else
         {
            result += ( (long)(data_byte - '0') * multiplier );            // Convert and add

            if (str_ptr < string) translation_done = 1;            // Check for end of buffer
            else multiplier *= 10;
         }
      }
   } while ( !translation_done );

   result *= decimal_mult;

   return result;
}


/**********************************************************
* Function Name: HexToLong
* Description:
   This function translates an HEX value to an unsigned long integer.
* Called From:
   ParseParameters/parser.c
* Author: Hanan
* Input Parameters:
   *string: pointer to string to be converted
* Output Parameters: None
* Return Value:
   The translated value
* Algorithm:
* Global Variables Used:
* Revisions:
**********************************************************/
long HexToLong(char *string)
{
   long  result, multiplier;
   char *str_ptr;          /* Pointer to the string in which the parameter is stored */
   char translation_done;  /* Flag used to indicate that the translation is finished */
   char data_byte;         /* character read from parameter buffer */

   result = 0;
   multiplier = 1;
   translation_done = 0x0;

   str_ptr = string + strlen(string) - 1;   // Point to the end of the string
   do {
      data_byte = *(str_ptr --);      // Read character from buffer

      // Convert nibble to numeric value
      if (data_byte >= ASCII_ZERO && data_byte <= ASCII_NINE)
         data_byte -= 0x30;
      else
         data_byte -= 0x37;

      result += (long)(data_byte * multiplier );      // Add to the result

      if (str_ptr < string)      // Check for end of buffer
      {
         translation_done = 0x01;
      }
      else
      {
         multiplier <<= 4;         // Multiply by 16
      }
   } while ( !translation_done );

   return( result );
}


long long HexToLongLong(char *string)
{
   long long result, multiplier;
   char *str_ptr;          /* Pointer to the string in which the parameter is stored */
   char translation_done;  /* Flag used to indicate that the translation is finished */
   char data_byte;         /* character read from parameter buffer */

   result = 0LL;
   multiplier = 1LL;
   translation_done = 0x0;

   str_ptr = string + strlen(string) - 1;   // Point to the end of the string
   do {
      data_byte = *(str_ptr --);      // Read character from buffer

      // Convert nibble to numeric value
      if (data_byte >= ASCII_ZERO && data_byte <= ASCII_NINE)
         data_byte -= 0x30;
      else
         data_byte -= 0x37;

      result += (((long long)data_byte) * multiplier );      // Add to the result

      if (str_ptr < string)      // Check for end of buffer
      {
         translation_done = 0x01;
      }
      else
      {
         multiplier <<= 4;         // Multiply by 16
      }
   } while ( !translation_done );

   return( result );
}


// Check if no more than 3 digits after the decimal point were received
char Check3DigitsAfterDecPoint(char number_of_characters)
{
   if (number_of_characters <= 4) return 0;

   // Point to the beginning of the parameter buffer
   p_u8_Parameter_Buffer_Ptr = u8_Parameter_Buffer;

   // Check whether a sign character was received
   while ((*p_u8_Parameter_Buffer_Ptr != '.') && number_of_characters)
   {
      // Point to the next character and decrement the number of (effective) characters
      p_u8_Parameter_Buffer_Ptr ++;
      number_of_characters --;
   }

   if (number_of_characters > 4) return 1;

   return 0;
}


/**********************************************************
* Function Name: CheckMaxNumericValue
* Description:
   This function is called to check that the received NUMERIC parameter
   is not greater that the maximum 'long' value. The comparison is done
   while the parameter is still in string form.
* Called From:
   ParseParameters/parser.c
* Author: Hanan
* Input Parameters: None
* Output Parameters: None
* Return Value:
   0: Value is less than or equal to the maximum 'long' value
   1: Value exceeds the maximum 'long' value
* Algorithm:
* Global Variables Used:
   u8_Parameter_Buffer: read
   p_u8_Parameter_Buffer_Ptr: read/write
   loop_counter
* Revisions:
**********************************************************/
char CheckMaxNumericValue(char number_of_characters)
{
   char result = 0, minus_fix = 0;
   int loop_counter = 0, decimal_point = 0;

   // Point to the beginning of the parameter buffer
   p_u8_Parameter_Buffer_Ptr = u8_Parameter_Buffer;

   // Check whether a sign character was received
   if ( (*p_u8_Parameter_Buffer_Ptr == '+') || (*p_u8_Parameter_Buffer_Ptr == '-') )
   {
      // Point to the next character and decrement the number of (effective) characters
      p_u8_Parameter_Buffer_Ptr ++;
      number_of_characters --;
   }

   // Check if there is a decimal point in the entered value
   for (loop_counter = 0; loop_counter < number_of_characters; loop_counter ++)
      if (*(p_u8_Parameter_Buffer_Ptr + loop_counter) == '.') decimal_point = loop_counter;

   if (!decimal_point)
   {
      // Check if the number of characters received is less than 10. If
      // so, the parameter is less than the maximum 'long' value.
      if (number_of_characters < 16)
      {
         result = 0;
      }
      else if (number_of_characters > 16)
      {
         result = 1;
      }
      else
      {
         // Check if the first digit is greater than or equal to 2
         if ( *p_u8_Parameter_Buffer_Ptr >= u8_Max_Long_String[0] )
         {
            // Compare the value of each digit in the parameter to each
            // digit in the maximum long value.
            for (loop_counter = 0; loop_counter < 16; loop_counter ++)
            {
               if ( (loop_counter == 15) && (u8_Parameter_Buffer[0] == '-') )
                  minus_fix = 1;

               if ( *(p_u8_Parameter_Buffer_Ptr) > u8_Max_Long_String[loop_counter]+minus_fix )
               {
                  result = 1;
                  break;
               }
               else if ( *(p_u8_Parameter_Buffer_Ptr) < u8_Max_Long_String[loop_counter]+minus_fix )
               {
                  result = 0;
                  break;
               }

               p_u8_Parameter_Buffer_Ptr ++;
            }
         }
         else
         {
            result = 0;
         }
      }
   }
   else //decimal_point
   {
      // Compare the value before the decimal point
      if (decimal_point < 16)
      {
         result = 0;
      }
      else if (decimal_point > 16)
      {
         result = 1;
      }
      else
      {
         // Check if the first digit is greater than or equal to 2
         if ( *p_u8_Parameter_Buffer_Ptr >= u8_Max_Long_String[0] )
         {
            // Compare the value of each digit in the parameter to each
            // digit in the maximum long value.
            for (loop_counter = 0; loop_counter < 16; loop_counter ++)
            {
               if ( (loop_counter == 15) && (u8_Parameter_Buffer[0] == '-') )
                  minus_fix = 1;

               if ( *(p_u8_Parameter_Buffer_Ptr) > u8_Max_Long_String[loop_counter]+minus_fix )
               {
                  result = 1;
                  break;
               }
               else if ( *(p_u8_Parameter_Buffer_Ptr) < u8_Max_Long_String[loop_counter]+minus_fix )
               {
                  result = 0;
                  break;
               }

               p_u8_Parameter_Buffer_Ptr ++;
            }
         }
         else
         {
            result = 0;
         }
      }
   }

   return( result );
}


/**********************************************************
* Function Name: CheckMaxHexValue
* Description:
   This function is called to check that the received HEX parameter
   is not greater that the maximum 'long' value. The comparison is done
   while the parameter is still in string form.
* Called From:
   ParseParameters/parser.c
* Author: Hanan
* Input Parameters: None
* Output Parameters: None
* Return Value:
   0: Value is less than or equal to the maximum 'long' value
   1: Value exceeds the maximum 'long' value
* Algorithm:
   If the number of characters received is less than 8, the value
   cannot be greater than '7FFFFFFF'. If 8 characters were received,
   then checking that the first character does not exceed '7' is
   enough to determine whether it exceeds the maximum value of not.
* Global Variables Used:
   u8_Parameter_Buffer: read
   p_u8_Parameter_Buffer_Ptr: read/write
   loop_counter
* Revisions:
**********************************************************/
char CheckMaxHexValue(char number_of_characters)
{
   char result;

   //Point to the beginning of the parameter buffer
   p_u8_Parameter_Buffer_Ptr = u8_Parameter_Buffer;

   // Check if the number of characters received is less than 8. If
   // so, the parameter is less than the maximum 'long' value.
   if (number_of_characters < 8)
   {
      result = 0;
   }
   else
   {
      // 8 character were received: check that the first character does
      // not exceed the value '7'.
      if ( u8_Parameter_Buffer[0] > '7' )
      {
         result = 1;
      }
      else
      {
         result = 0;
      }
   }
   return( result );
}
