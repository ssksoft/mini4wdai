// extern "C"
// {
// #include "xiao_codegen_sample.h"
// }
static const uint8_t PWM_PIN    = PC2;
static uint8_t  DUTY_50     = 128;
static uint8_t  DUTY_100     = 255;

void setup() {
  // put your setup code here, to run once:
  // pinMode(LED_BUILTIN, OUTPUT);
  // xiao_codegen_sample_initialize();
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  analogWrite(PWM_PIN, 0);
  
  // 電源投入から3秒後に走行開始
  delay(3000);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // xiao_codegen_sample_step();
  // digitalWrite(LED_BUILTIN, xiao_codegen_sample_Y.Out1);
  analogWrite(PWM_PIN, DUTY_100);
  delay(100);
}

