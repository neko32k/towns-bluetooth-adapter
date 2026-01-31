#ifndef BLECLIENTHIDADAFRUIT_EX_H_
#define BLECLIENTHIDADAFRUIT_EX_H_

#include <bluefruit.h>

class BLEClientHidAdafruitEx : public BLEClientHidAdafruit
{
  public:
    BLEClientCharacteristic& getKeyboardReportCharacteristic(void) { return _kbd_boot_input; }
    BLEClientCharacteristic& getMouseReportCharacteristic(void)    { return _mse_boot_input; }
    BLEClientCharacteristic& getGamepadReportCharacteristic(void)  { return _gpd_report; }
};

#endif
