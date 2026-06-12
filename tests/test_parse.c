#include <stdio.h>
#include "usb_parse.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

int main(void) {
    if (failures) {
        printf("FAILED (%d)\n", failures);
    } else {
        printf("all tests passed\n");
    }
    return failures ? 1 : 0;
}
