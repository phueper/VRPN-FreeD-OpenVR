#ifndef _console_h
#define _console_h

#include <windows.h>

#define console_window_width 120
#define console_window_height 50

void console_setup(HANDLE *p_c_out, HANDLE *p_c_in);
void console_cls(HANDLE hConsole);
char console_keypress(HANDLE hStdin);
void console_put(char* str);

int asprintf(char **strp, const char *format, ...);

#endif
