#include "Drive.var"


/**********************************************************
*
*            Local Functions
*
**********************************************************/
void FloatToFixS32Shift16(long* s32_fix, unsigned int* u16_shift, float f)
{
   float f1;
   int neg_sign = 0;
   long s32_tmp_fix = 0;
   unsigned int u16_tmp_shift = 0, u16_overflow_cntr=0;

   if (f == 0.0)
   {
      *s32_fix = 0;
      *u16_shift = 0;
      return;
   }

   if (f < 0)
   {
      neg_sign = 1;
      f = -f;
   }
   f1 = f;

   while ((f1 < (float)0x80000000) && (u16_overflow_cntr < 127))
   {
      u16_overflow_cntr++;
      f1 = f * 2.0;

      if(f1 < (float)0x80000000)
      {
         u16_tmp_shift++;
         f = f1;
      }
   }
   if (u16_overflow_cntr >= 127) u16_Internal_OverFlow |= 0x10; // Signal internal error

   s32_tmp_fix = (long)f;
   if (neg_sign) s32_tmp_fix = -s32_tmp_fix;

    *s32_fix = s32_tmp_fix;
    *u16_shift = u16_tmp_shift;
}


void FloatToFixS16ShiftLeft16(int* s16_fix, unsigned int* u16_shift, float f)
{
   int neg_sign = 0;

   long s32_tmp_fix = 0;
   unsigned int u16_tmp_shift = 0;

   if (f == 0.0)
   {
      *s16_fix = 0;
      *u16_shift = 0;
      return;
   }

   if (f < 0)
   {
      neg_sign = 1;
      s32_tmp_fix = (long)(-f);
   }
   else
      s32_tmp_fix = (long)f;

   while((s32_tmp_fix & 0x00000001) == 0)
   {
      s32_tmp_fix = s32_tmp_fix >> 1;
      u16_tmp_shift++;
   }

   if (neg_sign) s32_tmp_fix = -s32_tmp_fix;

   *s16_fix = (int)s32_tmp_fix;
   *u16_shift = u16_tmp_shift;
}

void OneDivS64ToFixU32Shift16(long* s32_fix, unsigned int* u16_shift, long long s64_data)
{
   unsigned long long s64_numerator = 0x4000000000000000; //(2^62)
   long long s64_result = 0;
   unsigned int u16_neg_sign = 0, u16_tmp_shift = 62, u16_overflow_cntr = 0;

   if (s64_data == 0)
   {
      *s32_fix = 0;
      *u16_shift = 0;
      return;
   }

   if (s64_data < 0)
   {
      u16_neg_sign = 1;
      if(s64_data == 0x8000000000000000) // -(2^63)
      {
            s64_data = 0x4000000000000000;
         u16_tmp_shift++;
      }
      else
         s64_data = -s64_data;
   }

   s64_result = (s64_numerator / s64_data);

   while ((s64_result > 2147483647) && (u16_overflow_cntr<65))
   {
      u16_overflow_cntr++;
      s64_result >>= 1;
      u16_tmp_shift--;
   }
   if (u16_overflow_cntr>=65) u16_Internal_OverFlow |= 0x01; // Signal internal error

   if (u16_neg_sign)
      s64_result = -s64_result;

   *s32_fix = (long)s64_result;
   *u16_shift = u16_tmp_shift;
}


void OneDivS64ToFixU64Shift16(long long* s64_fix, unsigned int* u16_shift, long long s64_data)
{
   unsigned long long s64_numerator = 0x4000000000000000; //(2^62)
   unsigned long long s64_residue = 0LL;
   long long s64_result = 0;
   unsigned int u16_neg_sign = 0;

   unsigned int u16_tmp_shift = 62;
   unsigned int u16_tmp_shift2 = 0;

   if (s64_data == 0)
   {
      *s64_fix = 0;
      *u16_shift = 0;
      return;
   }

   if (s64_data < 0)
   {
      u16_neg_sign = 1;
      if(s64_data == 0x8000000000000000) // -(2^63)
      {
            s64_data = 0x4000000000000000;
         u16_tmp_shift++;
      }
      else
         s64_data = -s64_data;
   }

   s64_result = (s64_numerator / s64_data);

   // Not needed
   while(s64_result > 9223372036854775807)
   {
      s64_result >>= 1;
      u16_tmp_shift--;
   }

   s64_residue = 0x4000000000000000 - s64_data*s64_result;

   // Move the FIX as much as possible to the left
   if ((s64_result < 0x4000000000000000) && (s64_result > 0))
   {
      do
      {
         s64_result <<= 1LL;
         u16_tmp_shift2++;
      } while (s64_result < 0x4000000000000000);
   }
   // Add residue shfted by the shl value calculated on last loop
   if (u16_tmp_shift2)
   {
      s64_residue = (s64_residue<<(long long)u16_tmp_shift2) / s64_data;
      s64_result += s64_residue;
      u16_tmp_shift += u16_tmp_shift2;
   }

   if (u16_neg_sign)
      s64_result = -s64_result;

   *s64_fix = s64_result;
   *u16_shift = u16_tmp_shift;
}


void FloatToFix16Shift16(int* s16_fix, unsigned int* u16_shift, float f)
{
   long s32_temp;
   FloatToFixS32Shift16(&s32_temp, u16_shift, f);

   if (s32_temp == 0)
   {
      (*s16_fix) = 0;
      (*u16_shift) = 0;
      return;
   }

   *s16_fix = (int)(s32_temp >> 16);

   if (*s16_fix != 0x7fff)
   {
      // if the value is not 0x7fff, a round can be added without overflow risk
      *s16_fix = (int)((s32_temp + 32768L) >> 16);
   }

   (*u16_shift) -= 16;
}


float power(float a, int u16_exp)
{
   float result = 1.0;

   if (u16_exp == 0)
      return result;

   if (u16_exp > 0)
   {
      while (u16_exp > 0)
      {
         u16_exp--;
         result *= a;
      }
   }
   else // u16_exp < 0
   {
      while (u16_exp < 0)
      {
         u16_exp++;
         result /= a;
      }
   }
   return result;
}


void MultFix64ByFix32ToFix32(long* s32_result_fix, unsigned int* u16_result_shift,
                             long long s64_fix1, unsigned int u16_shift1,
                             long      s32_fix2, unsigned int u16_shift2)
{
   unsigned long u32_fix1_Msbits = 0L, u32_fix1_Lsbits = 0L;
   unsigned long long u64_tmp_Lsbits = 0LL, u64_tmp_Msbits = 0LL;
   unsigned long u32_result_LSbits = 0L;
   unsigned long long u64_result = 0LL;
   int u16_neg_sign = 0;

   long s32_tmp_result_fix = 0;
   unsigned int u16_tmp_result_shift = 0;

   if ((s64_fix1 == 0) || (s32_fix2 == 0))
      return;

    u16_tmp_result_shift = u16_shift1 + u16_shift2;

   if(s64_fix1 < 0)
   {
      u16_neg_sign++;

      if(s64_fix1 == 0x8000000000000000) // -(2^63)
      {
            s64_fix1 = 0x4000000000000000;
         u16_tmp_result_shift--;
      }
      else
         s64_fix1 = -s64_fix1;
   }

   if (s32_fix2 < 0)
   {
      u16_neg_sign++;

      if(s32_fix2 == 0x80000000) // -(2^31)
      {
            s32_fix2 = 0x40000000;
         u16_tmp_result_shift--;
      }
      else
         s32_fix2 = -s32_fix2;
   }

   //break the 64 bits parameter into two 32 bits variables
   u32_fix1_Lsbits = (unsigned long)s64_fix1;
   u32_fix1_Msbits = (unsigned long)(s64_fix1 >> 32);

   //multiply in 2 parts
   u64_tmp_Lsbits = (unsigned long long)((unsigned long long)u32_fix1_Lsbits * (unsigned long long)s32_fix2);
   u64_tmp_Msbits = (unsigned long long)((unsigned long long)u32_fix1_Msbits * (unsigned long long)s32_fix2);

   u32_result_LSbits = (unsigned long)u64_tmp_Lsbits;
   u64_result = u64_tmp_Msbits + (unsigned long long)(u64_tmp_Lsbits >> 32);

   while(u64_result < 0x4000000000000000)
   {
      u64_result <<= 1;
      if (u32_result_LSbits > 0x7FFFFFFF)
         u64_result++;

      u32_result_LSbits <<= 1;
      u16_tmp_result_shift++;
   }

   u64_result >>= 31;

   u64_result++;   //rounding

   u64_result >>= 1;


   s32_tmp_result_fix = (long)u64_result;
   u16_tmp_result_shift -= 64;

   if(u16_neg_sign == 1)
      s32_tmp_result_fix = -s32_tmp_result_fix;

   *s32_result_fix = s32_tmp_result_fix;
   *u16_result_shift = u16_tmp_result_shift;
}


void MultFix64ByFix32ToFix64(long long* s64_result_fix, unsigned int* u16_result_shift,
                             long long s64_fix1, unsigned int u16_shift1,
                             long      s32_fix2, unsigned int u16_shift2)
{
   unsigned long u32_fix1_Msbits = 0L;
   unsigned long u32_fix1_Lsbits = 0L;
   unsigned long long u64_tmp_Lsbits = 0LL;
   unsigned long long u64_tmp_Msbits = 0LL;
   unsigned long u32_result_LSbits = 0L;
   unsigned long long u64_result = 0LL;
   int u16_neg_sign = 0;

   long long s64_tmp_result_fix = 0;
   unsigned int u16_tmp_result_shift = 0;

   if ((s64_fix1 == 0) || (s32_fix2 == 0))
      return;

    u16_tmp_result_shift = u16_shift1 + u16_shift2;

   if(s64_fix1 < 0)
   {
      u16_neg_sign++;

      if(s64_fix1 == 0x8000000000000000) // -(2^63)
      {
            s64_fix1 = 0x4000000000000000;
         u16_tmp_result_shift--;
      }
      else
         s64_fix1 = -s64_fix1;
   }

   if (s32_fix2 < 0)
   {
      u16_neg_sign++;

      if(s32_fix2 == 0x80000000) // -(2^31)
      {
            s32_fix2 = 0x40000000;
         u16_tmp_result_shift--;
      }
      else
         s32_fix2 = -s32_fix2;
   }

   //break the 64 bits parameter into two 32 bits variables
   u32_fix1_Lsbits = (unsigned long)s64_fix1;
   u32_fix1_Msbits = (unsigned long)(s64_fix1 >> 32);

   //multiply in 2 parts
   u64_tmp_Lsbits = (unsigned long long)((unsigned long long)u32_fix1_Lsbits * (unsigned long long)s32_fix2);
   u64_tmp_Msbits = (unsigned long long)((unsigned long long)u32_fix1_Msbits * (unsigned long long)s32_fix2);

   u32_result_LSbits = (unsigned long)u64_tmp_Lsbits;
   u64_result = u64_tmp_Msbits + (unsigned long long)(u64_tmp_Lsbits >> 32);

   while(u64_result < 0x4000000000000000)
   {
      u64_result <<= 1;
      if (u32_result_LSbits > 0x7FFFFFFF)
         u64_result++;

      u32_result_LSbits <<= 1;
      u16_tmp_result_shift++;
   }

   s64_tmp_result_fix = u64_result;

   u16_tmp_result_shift -= 32;

   if(u16_neg_sign == 1)
      s64_tmp_result_fix = -s64_tmp_result_fix;

   *s64_result_fix = s64_tmp_result_fix;
   *u16_result_shift = u16_tmp_result_shift;
}


void MultFixS32ByFixS32ToFixS32(long* s32_result_fix, unsigned int* u16_result_shift,
                                long s32_fix1, unsigned int u16_shift1,
                                long s32_fix2, unsigned int u16_shift2)
{

   long long s64_result_tmp = 0;
   unsigned int u16_neg_sign = 0, u16_tmp_result_shift = 0;

   if ((s32_fix1 == 0) || (s32_fix2 == 0))
   {
      *s32_result_fix = 0;
      *u16_result_shift = 0;
      return;
   }

    u16_tmp_result_shift = u16_shift1 + u16_shift2;

   if (s32_fix1 < 0)
   {
      u16_neg_sign++;
      if(s32_fix1 == 0x80000000) // -(2^31)
      {
         s32_fix1 = 0x40000000;
         u16_tmp_result_shift--;
      }
      else
         s32_fix1 = -s32_fix1;
   }

   if (s32_fix2 < 0)
   {
      u16_neg_sign++;
      if(s32_fix2 == 0x80000000) // -(2^31)
      {
         s32_fix2 = 0x40000000;
         u16_tmp_result_shift--;
      }
      else
         s32_fix2 = -s32_fix2;
   }

   s64_result_tmp = ((long long)s32_fix1 * s32_fix2);

   while(s64_result_tmp < 0x4000000000000000)
   {
      s64_result_tmp <<= 1;
      u16_tmp_result_shift++;
   }

   s64_result_tmp >>= 31;

   if(s64_result_tmp != 0xFFFFFFFF)
      s64_result_tmp++; //rounding

   s64_result_tmp >>= 1;

   u16_tmp_result_shift -= 32;

   if(u16_neg_sign == 1)//consider sign
      s64_result_tmp = -s64_result_tmp;

   *s32_result_fix = (long)s64_result_tmp;
    *u16_result_shift = u16_tmp_result_shift;
}


long long MultS64ByFixU32ToS64(long long s64_multA, unsigned long u32_multB, unsigned int u16_shr)
{
   long s32_multA_Msbits = 0L;
   unsigned long u32_multA_Lsbits = 0L;
   unsigned long long u64_tmp = 0LL;
   long long s64_tmp = 0LL;
   unsigned long u32_result_LSbits = 0L;
   unsigned long long s64_result = 0LL;
   int u16_is_neg = 0;

   if(s64_multA < 0)
   {
      if((s64_multA == 0x8000000000000000) && (u16_shr != 0)) // -(2^63)
      {
            s64_multA = 0x4000000000000000;
         u16_shr--;
      }
      else
         s64_multA = -s64_multA;

      u16_is_neg = 1;
   }

   //break the 64 bits parameter into two 32 bits variables
   u32_multA_Lsbits = (unsigned long)s64_multA;
   s32_multA_Msbits = (long)(s64_multA >> 32);

   //multiply in 2 parts
   u64_tmp = (unsigned long long)((unsigned long long)u32_multA_Lsbits * (unsigned long long)u32_multB);
   s64_tmp = (long long)((long long)s32_multA_Msbits * (unsigned long long)u32_multB);

   u32_result_LSbits = (unsigned long)u64_tmp;
   s64_result = s64_tmp + (long long)(u64_tmp >> 32);

   if(u16_shr >= 32)   //apply shift right
   {
      //rounding + shift right
      s64_result = ((s64_result + (long long)(1LL << (long long)(u16_shr - 33))) >> (u16_shr - 32));
   }
   else // u16_shr < 32
   {
      if (u16_shr != 0) //0 < u16_shr < 32
      {
         //rounding + shift right
         u32_result_LSbits = ((u32_result_LSbits + (unsigned long)(1L << (long)(u16_shr - 1))) >> u16_shr);
      }

      s64_result <<= (32 - u16_shr);

      s64_result |= (unsigned long long)u32_result_LSbits;
   }

   if(u16_is_neg == 1)
      s64_result = -s64_result;

   return s64_result;
}

long long MultS64ByFixS64ToS64(long long s64_multA, long long s64_multB, unsigned int u16_shr)
{
   unsigned long u32_multA_Msbits = 0L;
   unsigned long u32_multA_Lsbits = 0L;

   unsigned long u32_multB_Msbits = 0L;
   unsigned long u32_multB_Lsbits = 0L;

   unsigned long long u64_mult1 = 0LL;
   unsigned long long u64_mult2 = 0LL;
   unsigned long long u64_mult3 = 0LL;
   unsigned long long u64_mult4 = 0LL;

   unsigned long long u64_add1 = 0LL;
   unsigned long long u64_add2 = 0LL;
   unsigned long long u64_add3 = 0LL;
   unsigned long long u64_add4 = 0LL;

   unsigned long u32_remainder = 0L;
   unsigned long long s64_result = 0LL;
   int u16_is_neg = 0;

   if(s64_multA < 0)
   {
      if(s64_multA == 0x8000000000000000) // -(2^63)
      {
            s64_multA = 0x4000000000000000;
         u16_shr--;
      }
      else
      {
         s64_multA = -s64_multA;
      }

      u16_is_neg = 1;
   }

   if(s64_multB < 0)
   {
      if(s64_multB == 0x8000000000000000) // -(2^63)
      {
            s64_multB = 0x4000000000000000;
         u16_shr--;
      }
      else
      {
         s64_multB = -s64_multB;
      }

      u16_is_neg = 1 - u16_is_neg;
   }

   //break s64_multA parameter into two 32 bits variables
   u32_multA_Lsbits = (unsigned long)(s64_multA & 0xFFFFFFFF);
   u32_multA_Msbits = (unsigned long)(s64_multA >> 32);

   //break s64_multB parameter into two 32 bits variables
   u32_multB_Lsbits = (unsigned long)(s64_multB & 0xFFFFFFFF);
   u32_multB_Msbits = (unsigned long)(s64_multB >> 32);

   //multiply in 4 parts

   //MultA low bits with MultB low bits
   u64_mult1 = (unsigned long long)((unsigned long long)u32_multA_Lsbits * (unsigned long long)u32_multB_Lsbits);

   //MultA high bits with MultB low bits
   u64_mult2 = (unsigned long long)((unsigned long long)u32_multA_Msbits * (unsigned long long)u32_multB_Lsbits);

   //MultA low bits with MultB high bits
   u64_mult3 = (unsigned long long)((unsigned long long)u32_multA_Lsbits * (unsigned long long)u32_multB_Msbits);

   //MultA high bits with MultB high bits
   u64_mult4 = (unsigned long long)((unsigned long long)u32_multA_Msbits * (unsigned long long)u32_multB_Msbits);


   //add

   //low bits of u64_mult1
   u64_add1 = (unsigned long)(u64_mult1 & 0xFFFFFFFF);


   //high bits of u64_mult1 + low bits of u64_mult2 + low bits of u64_mult3
   u64_add2 = ((unsigned long long)(u64_mult1 >> 32) + (unsigned long long)(u64_mult2 & 0xFFFFFFFF) + (unsigned long long)(u64_mult3 & 0xFFFFFFFF));
   u32_remainder = (unsigned long)(u64_add2 >> 32);

   u64_add2 = (unsigned long)(u64_add2 & 0xFFFFFFFF);


   //high bits of u64_mult2 + high bits of u64_mult3 + low bits of u64_mult4 + reminder
   u64_add3 = ((unsigned long long)(u64_mult2 >> 32) + (unsigned long long)(u64_mult3 >> 32) + (unsigned long long)(u64_mult4 & 0xFFFFFFFF) + u32_remainder);

   u32_remainder = (unsigned long)(u64_add3 >> 32);
   u64_add3 = (unsigned long)(u64_add3 & 0xFFFFFFFF);


   //high bits of u64_mult4 + reminder
   u64_add4 = ((unsigned long)(u64_mult4 >> 32) + u32_remainder);

   //sum all data into 2 64 bits registers
   u64_add1 = ((u64_add2 * 0x100000000 ) + u64_add1);
   u64_add2 = ((u64_add4 * 0x100000000 ) + u64_add3);

   //apply shift right

   if(u16_shr != 0)
   {
      if (u16_shr < 64)
      {
         u64_add1 >>= (u16_shr - 1);
         u64_add1++;      // rounding
         u64_add1 >>= 1;
      }
      else
         u64_add1 = 0;
   }


   if (u16_shr < 64)
      u64_add2 <<= (64 - u16_shr);
   else if (u16_shr > 64)
      u64_add2 =(u64_add2 + (1LL<<(u16_shr-65)))>>(u16_shr - 64);


   s64_result = u64_add1 + u64_add2;


   if(u16_is_neg == 1)
      s64_result = -s64_result;

   return s64_result;
}

void MultFix64ByFix64ToFix64(long long* s64_result_fix, unsigned int* u16_result_shift, long long s64_fix1,
                             unsigned int u16_shift1, long long s64_fix2, unsigned int u16_shift2)
{
   unsigned long u32_fix1_Msbits = 0L;
   unsigned long u32_fix1_Lsbits = 0L;
   unsigned long u32_fix2_Msbits = 0L;
   unsigned long u32_fix2_Lsbits = 0L;
   unsigned long long u64_mult1 = 0LL;
   unsigned long long u64_mult2 = 0LL;
   unsigned long long u64_mult3 = 0LL;
   unsigned long long u64_mult4 = 0LL;
   unsigned long long u64_add1 = 0LL;
   unsigned long long u64_add2 = 0LL;
   unsigned long long u64_add3 = 0LL;
   unsigned long long u64_add4 = 0LL;
   unsigned long u32_remainder = 0L;
   int u16_neg_sign = 0;
   long long s64_tmp_result_fix = 0LL;
   unsigned int u16_tmp_result_shift = 0;

   u16_tmp_result_shift = u16_shift1 + u16_shift2;

   if ((s64_fix1 == 0) || (s64_fix2 == 0))
   {
      return;
   }

   u16_tmp_result_shift = u16_shift1 + u16_shift2;

   if(s64_fix1 < 0)
   {
      u16_neg_sign++;

      if(s64_fix1 == 0x8000000000000000) // -(2^63)
      {
         s64_fix1 = 0x4000000000000000;
         u16_tmp_result_shift--;
      }
      else
         s64_fix1 = -s64_fix1;
   }

   if (s64_fix2 < 0)
   {
      u16_neg_sign++;

      if(s64_fix2 == 0x8000000000000000) // -(2^63)
      {
            s64_fix2 = 0x4000000000000000;
         u16_tmp_result_shift--;
      }
      else
         s64_fix2 = -s64_fix2;
   }

   //break s64_fix1 parameter into two 32 bits variables
   u32_fix1_Lsbits = (unsigned long)(s64_fix1 & 0x00000000FFFFFFFF);
   u32_fix1_Msbits = (unsigned long)(s64_fix1 >> 32);

   //break s64_fix2 parameter into two 32 bits variables
   u32_fix2_Lsbits = (unsigned long)(s64_fix2 & 0x00000000FFFFFFFF);
   u32_fix2_Msbits = (unsigned long)(s64_fix2 >> 32);

   //multiply in 4 parts

   //fix1 low bits with fix2 low bits
   u64_mult1 = (unsigned long long)((unsigned long long)u32_fix1_Lsbits * (unsigned long long)u32_fix2_Lsbits);

   //fix1 high bits with fix2 low bits
   u64_mult2 = (unsigned long long)((unsigned long long)u32_fix1_Msbits * (unsigned long long)u32_fix2_Lsbits);

   //fix1 low bits with fix2 high bits
   u64_mult3 = (unsigned long long)((unsigned long long)u32_fix1_Lsbits * (unsigned long long)u32_fix2_Msbits);

   //fix1 high bits with fix2 high bits
   u64_mult4 = (unsigned long long)((unsigned long long)u32_fix1_Msbits * (unsigned long long)u32_fix2_Msbits);

   //add
   //low bits of u64_mult1
   u64_add1 = (unsigned long)(u64_mult1 & 0x00000000FFFFFFFF);

   //high bits of u64_mult1 + low bits of u64_mult2 + low bits of u64_mult3
   u64_add2 = ((unsigned long long)(u64_mult1 >> 32) + (unsigned long long)(u64_mult2 & 0x00000000FFFFFFFF) + (unsigned long long)(u64_mult3 & 0x00000000FFFFFFFF));
   u32_remainder = (unsigned long)(u64_add2 >> 32);

   u64_add2 = (unsigned long)(u64_add2 & 0x00000000FFFFFFFF);

   //high bits of u64_mult2 + high bits of u64_mult3 + low bits of u64_mult4 + reminder
   u64_add3 = ((unsigned long long)(u64_mult2 >> 32) + (unsigned long long)(u64_mult3 >> 32) + (unsigned long long)(u64_mult4 & 0x00000000FFFFFFFF) + u32_remainder);
   u32_remainder = (unsigned long)(u64_add3 >> 32);
   u64_add3 = (unsigned long)(u64_add3 & 0x00000000FFFFFFFF);

   //high bits of u64_mult4 + reminder
   u64_add4 = ((unsigned long)(u64_mult4 >> 32) + u32_remainder);

   //sum all data into 2 64 bits registers
   u64_add1 = ((u64_add2 * 0x100000000 ) + u64_add1);
   u64_add2 = ((u64_add4 * 0x100000000 ) + u64_add3);

   while(u64_add2 < 0x4000000000000000)
   {
      u64_add2 <<= 1;
      if (u64_add1 > 0x7fffffffffffffff)
      {
         u64_add2++;
      }

      u64_add1 <<= 1;
      u16_tmp_result_shift++;
   }

   if ( (u64_add1 > 0x7fffffffffffffff) && (u64_add2 < 0x7fffffffffffffff) )  // rounding (with overflow protection)
   {
      u64_add2++;
   }

   s64_tmp_result_fix = u64_add2;
   if (u16_tmp_result_shift >= 64)
   {
      u16_tmp_result_shift -= 64;
   }
   else
   {
      u16_tmp_result_shift = 0;
      s64_tmp_result_fix = 0x7fffffffffffffff;
   }

   if(u16_neg_sign == 1)
   {
      s64_tmp_result_fix = -s64_tmp_result_fix;
   }

   *s64_result_fix = s64_tmp_result_fix;
   *u16_result_shift = u16_tmp_result_shift;
}

//**********************************************************
// Function Name: IsPowerOf2
// Description:
//          This function checks if the given number is power 
//          of 2.
//
// Author: Sergei.P
// Algorithm: Count the number of ON (1) bits in the number binary 
//            representation.
//            If only one bit is ON, return 1
//            else return 0
//Revisions:
//**********************************************************
int IsPowerOf2(unsigned long long u64_num)
{
   int counter;
   
   counter = 0;
   while (u64_num)
   {
       counter += (u64_num & 1LL);
       u64_num >>= 1;
       
       if (counter > 1)
       {
            return 0;
       }   
   }
   
   return (counter == 1) ? 1 : 0;
}

