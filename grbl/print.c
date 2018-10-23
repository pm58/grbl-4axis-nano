
#include "grbl.h"


void printString(const char *s)
{
  while (*s)
    serial_write(*s++);
}


// Print a string stored in PGM-memory
void printPgmString(const char *s)
{
  char c;
  while ((c = pgm_read_byte_near(s++)))
    serial_write(c);
}



// Prints an uint8 variable with base and number of desired digits.
void print_unsigned_int8(uint8_t n, uint8_t base, uint8_t digits)
{ 
  unsigned char buf[digits];
  uint8_t i = 0;

  for (; i < digits; i++) {
      buf[i] = n % base ;
      n /= base;
  }

  for (; i > 0; i--)
      serial_write('0' + buf[i - 1]);
}


// Prints an uint8 variable in base 2.
void print_uint8_base2(uint8_t n) {
  print_unsigned_int8(n,2,8);
}


// Prints an uint8 variable in base 10.
void print_uint8_base10(uint8_t n)
{   
  uint8_t digits;
  if (n < 10) { digits = 1; } 
  else if (n < 100) { digits = 2; }
  else { digits = 3; }
  print_unsigned_int8(n,10,digits);
}


void print_uint32_base10(uint32_t n)
{ 
  if (n == 0) {
    serial_write('0');
    return;
  } 

  unsigned char buf[10]; 
  uint8_t i = 0;  
  
  while (n > 0) {
    buf[i++] = n % 10;
    n /= 10;
  }
    
  for (; i > 0; i--)
    serial_write('0' + buf[i-1]);
}


void printInteger(long n)
{
  if (n < 0) {
    serial_write('-');
    print_uint32_base10(-n);
  } else {
    print_uint32_base10(n);
  }
}



void printFloat(float n, uint8_t decimal_places)
{
  if (n < 0) {
    serial_write('-');
    n = -n;
  }

  uint8_t decimals = decimal_places;
  while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
    n *= 100;
    decimals -= 2;
  }
  if (decimals) { n *= 10; }
  n += 0.5; // Add rounding factor. Ensures carryover through entire value.
    
  // Generate digits backwards and store in string.
  unsigned char buf[10]; 
  uint8_t i = 0;
  uint32_t a = (long)n;  
  buf[decimal_places] = '.'; // Place decimal point, even if decimal places are zero.
  while(a > 0) {
    if (i == decimal_places) { i++; } // Skip decimal point location
    buf[i++] = (a % 10) + '0'; // Get digit
    a /= 10;
  }
  while (i < decimal_places) { 
     buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
  }
  if (i == decimal_places) { // Fill in leading zero, if needed.
    i++;
    buf[i++] = '0'; 
  }   
  
  // Print the generated string.
  for (; i > 0; i--)
    serial_write(buf[i-1]);
}


void printFloat_CoordValue(float n) { 
  if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) { 
    printFloat(n*INCH_PER_MM,N_DECIMAL_COORDVALUE_INCH);
  } else {
    printFloat(n,N_DECIMAL_COORDVALUE_MM);
  }
}

void printFloat_RateValue(float n) { 
  if (bit_istrue(settings.flags,BITFLAG_REPORT_INCHES)) {
    printFloat(n*INCH_PER_MM,N_DECIMAL_RATEVALUE_INCH);
  } else {
    printFloat(n,N_DECIMAL_RATEVALUE_MM);
  }
}

void printFloat_SettingValue(float n) { printFloat(n,N_DECIMAL_SETTINGVALUE); }


