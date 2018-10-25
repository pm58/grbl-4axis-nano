
#ifndef print_h
#define print_h


void printString(const char *s);

void printPgmString(const char *s);

void printInteger(long n);

void print_uint32_base10(uint32_t n);

// Prints uint8 variable with base and number of desired digits.
void print_unsigned_int8(uint8_t n, uint8_t base, uint8_t digits); 

// Prints an uint8 variable in base 2.
void print_uint8_base2(uint8_t n);

// Prints an uint8 variable in base 10.
void print_uint8_base10(uint8_t n);

void printFloat(float n, uint8_t decimal_places);

void printFloat_CoordValue(float n);

void printFloat_RateValue(float n);

void printFloat_SettingValue(float n);

// Debug tool to print free memory in bytes at the called point. Not used otherwise.
void printFreeMemory();

#endif