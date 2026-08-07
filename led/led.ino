extern "C"
{
#include "xiao_codegen_sample.h"
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

