#import <CoreGraphics/CoreGraphics.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: get_display_origin <displayID>\n");
        return 1;
    }
    uint32_t displayID = (uint32_t)atoi(argv[1]);
    CGRect bounds = CGDisplayBounds(displayID);
    printf("%.0f %.0f %.0f %.0f\n", bounds.origin.x, bounds.origin.y,
           bounds.size.width, bounds.size.height);
    return 0;
}
