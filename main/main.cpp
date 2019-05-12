#include <M5StickC.h>

extern "C" void
app_main()
{
    printf("test.\n");

    M5.begin();
    M5.Lcd.print("M5 PM PIANO");
}