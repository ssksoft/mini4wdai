extern "C"
{
#include "xiao_codegen_sample.h"
#include "em_letimer.h"
#include "em_cmu.h"
#include "em_core.h"
#include "em_device.h"
}
void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  xiao_codegen_sample_initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
  xiao_codegen_sample_step();
  digitalWrite(LED_BUILTIN, xiao_codegen_sample_Y.Out1);
  delay(100);
}

// void initTimer() {
//     LETIMER_Init_TypeDef init = LETIMER_INIT_DEFAULT;
//     init.enable = false;
//     init.comp0Top = true;  // COMP0 を周期に使う

//     LETIMER_Init(LETIMER0, &init);

//     // 100 ms の周期設定
//     uint32_t freq = CMU_ClockFreqGet(cmuClock_LETIMER0);
//     uint32_t ticks = freq * 0.1;  // 100 ms

//     LETIMER_CompareSet(LETIMER0, 0, ticks);

//     LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP0);
//     NVIC_EnableIRQ(LETIMER0_IRQn);

//     LETIMER_Enable(LETIMER0, true);
// }

// extern "C" void LETIMER0_IRQHandler(void) {
//     LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);
//     xiao_codegen_sample_step();
// }