#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "smintf.h"

/*
 * Code taken from my submission for HW09 for Purdue ECE264
 * Do not cheat
 *
 * - Alexander Bowman
 *
 */
/*
*
*	if buffer is NULL, then it will only increment current index and return
*
*/
void _putc(char c, char* buffer, int *currentIndex) {
	if (*currentIndex == -1) {
		fputc(c, stdout);
//		log_cyan("character: %c", c);
	} else {
		if (buffer != NULL) {
			buffer[*currentIndex] = c;
		}
		*currentIndex = *currentIndex + 1;
	}
}


void __print_integer(int n, int radix, char* prefix, char* buffer, int *currentIndex) {
	int digitCount = 0;
	long nCopy = n; // use long here since -INT_MIN = INT_MAX + 1 and would cause an overflow

	if (nCopy < 0) {
		nCopy *= -1; // make sure nCopy is positive
	}

	while (nCopy > 0) {
		nCopy /= radix;
		digitCount ++;
	}

	if (n < 0) {
		_putc('-', buffer, currentIndex);
	}

	int prefixIdx = 0;
	while (prefix[prefixIdx] != '\0') {
		_putc(prefix[prefixIdx], buffer, currentIndex);
		prefixIdx++;
	}

	if (n == 0) {
		
		_putc('0', buffer, currentIndex);
		
	} else {

		for (int currentDigit = digitCount - 1; currentDigit >= 0; currentDigit --) {
			nCopy = n;
		
			if (nCopy < 0) {
				nCopy *= -1;
			}
		
			for (int j = 0; j < currentDigit; j ++) {
				nCopy /= radix;
			}

			int digitValue = (nCopy % radix);
			int digitChar = 0;
			
			if (digitValue < 10) {
				digitChar = digitValue + '0';
			} else {
				digitChar = digitValue - 10 + 'a';
			}
			
			_putc(digitChar, buffer, currentIndex);
		}
	
	}

}


/*
*	NOTE: if returnLength is -1, then that means print to the console. Otherwise, add to the buffer if it's not null
*/
void __operation_smintf(char* buffer, int* returnLength, const char* format, va_list args) {

	int length = 0;

	while (format[length] != '\0') {
		length ++;	
	}
	
	int currentCharacterIdx = 0;
	for (; currentCharacterIdx < length; currentCharacterIdx ++) {
		char theChar = format[currentCharacterIdx];
        if (theChar == '%' && (currentCharacterIdx != (length - 1)) /* <--- if the last character is a % then just print a % */) {
			currentCharacterIdx = currentCharacterIdx + 1;
			char specialChar = format[currentCharacterIdx];
			switch (specialChar) {
				case 'd': {
					int theArgAsInt = va_arg(args, int);
					__print_integer(theArgAsInt, 10, "", buffer, returnLength);
				}
				break;
				case 'c': {
					char theCharToPrint = (char) va_arg(args, int);
					_putc(theCharToPrint, buffer, returnLength);
				}
				break;
				case 'x': {
					int theArgAsInt = va_arg(args, int);
					__print_integer(theArgAsInt, 16, "0x", buffer, returnLength);
				}
				break;
				case 'b': {
					int theArgAsInt = va_arg(args, int);
					__print_integer(theArgAsInt, 2, "0b", buffer, returnLength);
				}
				break;
				case '$': {
					double theArgAsDouble = va_arg(args, int)/100.0;
					__print_integer((int) (theArgAsDouble), 10, "$", buffer, returnLength);
					int decimalValue = (int) (((int) (theArgAsDouble * 100)) % 100);
					if (decimalValue < 0) {
						decimalValue = -1 * decimalValue;
					}
					_putc('.', buffer, returnLength);
					_putc('0' + (decimalValue / 10), buffer, returnLength);
					_putc('0' + (decimalValue % 10), buffer, returnLength);
				}
				break;
				case 'f':{
					double theArgAsDouble = va_arg(args, double);
					__print_integer((int) (theArgAsDouble), 10, "", buffer, returnLength);
					int decimalValue = (int) (((int) (theArgAsDouble * 100)) % 100);
					if (decimalValue < 0) {
						decimalValue = -1 * decimalValue;
					}
					_putc('.', buffer, returnLength);
					_putc('0' + (decimalValue / 10), buffer, returnLength);
					_putc('0' + (decimalValue % 10), buffer, returnLength);
				}
				break;
				case '%': {
					_putc('%', buffer, returnLength);
				}
				break;
				case 's': {
					int i = 0;
					char* str = va_arg(args, char*);
					while (str[i] != '\0') {
						_putc(str[i++], buffer, returnLength);					
					}
				}
				break;
				default: {
					_putc('%', buffer, returnLength);
					_putc(specialChar, buffer, returnLength);
				}
				break;
			}
        } else {
			_putc(theChar, buffer, returnLength);		
		}
	}
	
	va_end(args);

}

void mintf(const char *format, ...) {
	va_list args;
	va_start(args, format);
	int theInt = -1; // since this int is -1, it signals to the program to print to stdout
	__operation_smintf(NULL, &theInt, format, args);
	va_end(args);
}


char* smintf(const char *format, ...) {
	int bufferLength = 0;
	va_list args;
	va_start(args, format);
	__operation_smintf(NULL, &bufferLength, format, args);
	
	char* buffer = malloc(bufferLength+1);
	if (buffer == NULL) return NULL;
	int bufferIndex = 0;
	va_start(args, format);
	__operation_smintf(buffer, &bufferIndex, format, args);
	
	buffer[bufferIndex] = '\0';
	return buffer;
}

char* smintf_fast_v1(const char *format, uint16_t predictedLen, ...) {
	char* buffer = malloc(predictedLen+1);
	if (buffer == NULL) return NULL;
	int bufferIndex = 0;

	va_list args;
	va_start(args, predictedLen);
	__operation_smintf(buffer, &bufferIndex, format, args);

	va_end(args);

	if (bufferIndex > predictedLen)
		bufferIndex = predictedLen;

	buffer[bufferIndex] = '\0';
	return buffer;
}

char* smintf_list(const char *format, va_list args1, va_list args2) { // pass the same list twice, I'm lazy
	int bufferLength = 0;
	__operation_smintf(NULL, &bufferLength, format, args1);

	char* buffer = malloc(bufferLength+1);
	if (buffer == NULL) return NULL;
	int bufferIndex = 0;
	__operation_smintf(buffer, &bufferIndex, format, args2);

	buffer[bufferIndex] = '\0';
	return buffer;
}

#include "main.h"

/*
 * I noticed in the main_heartbeat function, smintf takes like 7ms with all the replacements,
 * so this is meant to be a fast replacement for it. There is only one goal: speed.
 *
 * Instead of allocating the correct amount of bytes, this one will assume that there
 * is a giant buffer to write to (with more than enough length). This will
 * at least half the time.
 */
void fast_smintf(char* buffer, ...) {
	va_list args;
	va_start(args, buffer);

	uint32_t idx = 0;
	int returnLength = 0;

	smintf_var_type_t var_type = va_arg(args, smintf_var_type_t);
	while (var_type != SMINTF_END) {
		log_messagef("var type: %d", (int) var_type);
		switch (var_type) {
			case SMINTF_STRING: {
				char* arg = va_arg(args, char*);
				while (arg[0] != '\0') {
					buffer[idx++] = arg[0];
					arg++;
				}
			}
			break;
			case SMINTF_INT: {
				int arg = va_arg(args, int);
				returnLength = 0;
				__print_integer(arg, 10, "", buffer, &returnLength);
				idx += returnLength;
			} break;
			case SMINTF_FLOAT: {
				double arg = va_arg(args, double);

				returnLength = 0;
				__print_integer((int) (arg), 10, "", buffer, &returnLength);
				idx += returnLength;

				int decimalValue = (int) (((int) (arg * 100)) % 100);
				if (decimalValue < 0) {
					decimalValue = -1 * decimalValue;
				}

				buffer[idx++] = '.';
				buffer[idx++] = '0' + (decimalValue / 10);
				buffer[idx++] = '0' + (decimalValue % 10);
			} break;
		}
		buffer[idx++] = ',';
		var_type = va_arg(args, smintf_var_type_t);
	}
}

/* vim: set tabstop=4 shiftwidth=4 fileencoding=utf-8 noexpandtab: */
