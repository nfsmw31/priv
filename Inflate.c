#include "DSP2834x_Device.h"
#include "DevTools.pro"
#include "Inflate.pro"
#include "Inflate.var"
#include "Record.var"
#include "Prototypes.pro"

#pragma CODE_SECTION(InflateInit, "ram346_1");
void InflateInit(unsigned long u32_deflated_image_start_addr)
{
   int i=0;
   u64_Inflate_In_Out = 0;

   BGVAR(u16_Code_Table_Last_Index) = 1;
   BGVAR(u32_Compression_Type) = 0L;

   BGVAR(u32_Comp_Size_In_Bits) = 0;

   BGVAR(u16_Code_Len_To_Search_In_Table) = 0;
   BGVAR(s16_Search_Response) = 0;
   BGVAR(s16_Read_Data_Bit_Counter) = 0;
   BGVAR(u16_Table_Size_In_Word) = 0;

   p_u16_Inflate_In = (unsigned int*) &u64_Inflate_In_Out + 1;
   p_u32_Inflate_Out = (unsigned long*) &u64_Inflate_In_Out + 1;

   //read the first 32 bit (comprassed data len in bits).
   BGVAR(u32_Compression_Type) =*((unsigned long*)(u32_deflated_image_start_addr));

   //read the first 32 bit (comprassed data len in bits).
   BGVAR(u32_Comp_Size_In_Bits) =*((unsigned long*)(u32_deflated_image_start_addr + 2));

   BGVAR(u16_Table_Size_In_Word) = (256 + 13); // 256 symbols + 13 "0" reps.

   // point to start address of the code table - offset of 2 words from the start of the file.
   p_u32_Code_Table = (unsigned long*)(u32_deflated_image_start_addr + 4);

   // point to start address of the symbol table - offset of (2 * (256+13)) (symbols + repetitions * 32bit).
   p_u16_Symbol_Table = (unsigned int*)(u32_deflated_image_start_addr + (u16_Table_Size_In_Word * 2) + 4);

   /* init the ram pointers adresses*/
   p_u32_Code_Table_Ram = (unsigned long*)(p_s16_Debug_Ram + 4); // code table starts after 2x32bits variables (compression type & compression size in bits)
   p_u16_Symbol_Table_Ram = (unsigned int*)(p_u32_Code_Table_Ram + (BGVAR(u16_Table_Size_In_Word) * 2)); // symbol table starts when caode table ands.

   p_u32_Code_Table_Ram_Temp = p_u32_Code_Table_Ram;
   p_u16_Symbol_Table_Ram_Temp = p_u16_Symbol_Table_Ram;

   /* copy tables from flash to ram*/
   for(i=0; i < BGVAR(u16_Table_Size_In_Word); i++)
   {
      *(p_u32_Code_Table_Ram_Temp++) = *(p_u32_Code_Table++);
      *(p_u16_Symbol_Table_Ram_Temp++) = *(p_u16_Symbol_Table++);
   }

   /*change the flash pointers to point to the new ram adresses  */
   p_u32_Code_Table = p_u32_Code_Table_Ram;
   p_u16_Symbol_Table = p_u16_Symbol_Table_Ram;

   // pointer to start of compressed data
   p_u16_Comp_Data_Adddress = (unsigned int*)(u32_deflated_image_start_addr + ((256 + 13)*3) + 4);
}


// load deflated FPGA image from the flash

#pragma CODE_SECTION(InFlateProcess, "ram346_1");
int InFlateProcess()
{
   while((BGVAR(u32_Comp_Size_In_Bits)--) > 0)
   {// run on all comp bits
      if(BGVAR(s16_Read_Data_Bit_Counter) == 0)
      {
         *p_u16_Inflate_In = *p_u16_Comp_Data_Adddress++;
         BGVAR(s16_Read_Data_Bit_Counter) = 16;
      }
      u64_Inflate_In_Out <<= 1;

      BGVAR(s16_Read_Data_Bit_Counter)--;
      BGVAR(u16_Code_Len_To_Search_In_Table)++;

      if(BGVAR(s16_Search_Response) >= 0)
      {
         BGVAR(s16_Search_Response) = convertBinaryCodeToIndex(*p_u32_Inflate_Out, BGVAR(u16_Code_Len_To_Search_In_Table));
      }

      if (BGVAR(s16_Search_Response) < 0)
      {// get more more bits to match the next binary len
         if(BGVAR(u16_Code_Len_To_Search_In_Table) == 27)
         {// inflate failed requested binary code is longer than max len (27 bits)
            return -1;
         }
         BGVAR(s16_Search_Response)++;
       continue;
      }

      //reset all for next code
      *p_u32_Inflate_Out = 0;
      BGVAR(u16_Code_Table_Last_Index)=1;
      BGVAR(u16_Code_Len_To_Search_In_Table)=0;

      // return the uncompressed data that was found
      return p_u16_Symbol_Table[s16_Search_Response];
   }
   // unexpected function termination
   return -2;
}

#pragma CODE_SECTION(InFlateProcessWord, "ram346_1");
int InFlateProcessWord()
{
   int s16_ret_val=0, s16_temp=0;

   s16_ret_val = InFlateProcess();

   if(s16_ret_val >= 0)
   {
      s16_temp = s16_ret_val << 8;
      s16_ret_val = InFlateProcess();

      if(s16_ret_val >= 0)
      {
         s16_ret_val |= s16_temp;
      }
   }
   return s16_ret_val;
}

#pragma CODE_SECTION(convertBinaryCodeToIndex, "ram346_1");
// 3.6 Sec - SE ECAT FPGA IMAGE
int convertBinaryCodeToIndex(unsigned long u32_code_to_search, unsigned int u16_code_len_to_search_in_table)
{
   unsigned int u16_current_table_len;
   unsigned long u32_current_code_table_line;

   for(; BGVAR(u16_Code_Table_Last_Index) < (256+13) ; BGVAR(u16_Code_Table_Last_Index)++)
   {
      u32_current_code_table_line = p_u32_Code_Table[BGVAR(u16_Code_Table_Last_Index)];
      u16_current_table_len = (unsigned int)((u32_current_code_table_line >> 27) & 0x001F);

      if( u16_current_table_len == u16_code_len_to_search_in_table )
      {// if code len is match check if the bin code match
         if((u32_current_code_table_line & 0x7FFFFFF) == u32_code_to_search)
         {// char was found
            return BGVAR(u16_Code_Table_Last_Index);
         }
      }
      else if( u16_current_table_len > u16_code_len_to_search_in_table )
      {// get another bit

         // return the diff so the next entry to this function will be with the right code len
         return (u16_code_len_to_search_in_table - u16_current_table_len);
      }
   }
   return -1;
}

