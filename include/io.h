#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

int gpio_init(const char *name);
int gpio_deinit(const char *name);
int gpio_high(const char *name);
int gpio_low(const char *name);

