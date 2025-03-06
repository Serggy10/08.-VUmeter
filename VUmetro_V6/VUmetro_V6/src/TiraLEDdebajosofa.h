#ifndef _TIRALEDDEBAJOSOFA_H_
#define _TIRALEDDEBAJOSOFA_H_

#include <SinricProDevice.h>
#include <Capabilities/ModeController.h>
#include <Capabilities/ColorController.h>
#include <Capabilities/BrightnessController.h>

class TiraLEDdebajosofa
    : public SinricProDevice,
      public ModeController<TiraLEDdebajosofa>,
      public ColorController<TiraLEDdebajosofa>,
      public BrightnessController<TiraLEDdebajosofa>
{
  friend class ModeController<TiraLEDdebajosofa>;
  friend class ColorController<TiraLEDdebajosofa>;
  friend class BrightnessController<TiraLEDdebajosofa>;

public:
  TiraLEDdebajosofa(const String &deviceId) : SinricProDevice(deviceId, "TiraLEDdebajosofa") {};
};

#endif
