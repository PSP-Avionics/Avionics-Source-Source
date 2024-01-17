/* vim: set tabstop=4 shiftwidth=4 fileencoding=utf-8 noexpandtab: */
#ifndef __smintf_h__
#define __smintf_h__
#include <stdint.h>

typedef enum {
	SMINTF_END,
	SMINTF_STRING,
	SMINTF_INT,
	SMINTF_FLOAT
} smintf_var_type_t;

char* smintf(const char *format, ...);
char* smintf_list(const char *format, va_list args1, va_list args2);
char* smintf_fast_v1(const char *format, uint16_t predictedLen, ...);

void fast_smintf(char* buffer, ...);

// Bonus option only:
void mintf(const char *format, ...);

#endif
