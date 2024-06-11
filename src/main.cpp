#include <Wire.h>
#include <heltec.h>                // Display
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <FastLED.h>
#include <algorithm>               // For std::clamp
#include <driver/ledc.h>           // For LEDC PWM

#define ADC1_CHANNEL (ADC1_CHANNEL_0)  // GPIO36 if ADC1 channel 0

#define DEFAULT_RPM  1500

// Define the pin where the TACH signal is connected
#define TACH_PIN   2
#define PWM_PIN    4

const int pwmPin = 2;

esp_adc_cal_characteristics_t *adc_chars;

unsigned long lastMillis = -1000000;
const unsigned long interval = 100000; // 1 second

#define NUM_LEDS 32
#define DATA_PIN 5

CRGB leds[NUM_LEDS];
hw_timer_t *timer = NULL;
volatile bool updateLEDs = false;  // Flag to indicate LED update

void IRAM_ATTR onTimer() {
    // Set the flag to update LEDs
    updateLEDs = true;
}

float mapValue(float x, float in_min, float in_max, float out_min, float out_max) 
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void UpdateTimers(uint millivolts, float scaler)
{
      // Measure the pulse width in microseconds
      unsigned long highTime = pulseIn(TACH_PIN, HIGH);
      unsigned long lowTime = pulseIn(TACH_PIN, LOW);

      // Calculate the period
      unsigned long period = highTime + lowTime;

      // Calculate the frequency
      float frequency = 0;
      if (period > 0) {
        frequency = 1000000.0 / period; // Frequency in Hz
      }

      frequency = 50.0f;

      // Calculate the RPM
      float rpm = frequency * 60 * scaler;
      
      // Calculate the strobe frequency
      float strobe_frequency = rpm * 9.0 / 60.0; // Convert to Hz

      // Update the timer alarm to match the new strobe frequency
      if (strobe_frequency > 0) {
        timerAlarmWrite(timer, 1000000 / strobe_frequency, true); // 1 second = 1,000,000 microseconds
        timerAlarmEnable(timer);
      } else {
        timerAlarmDisable(timer);
      }
      Heltec.display->clear();
      Heltec.display->drawString(0, 0, "Pulse Frequency (Hz):");
      Heltec.display->drawString(0, 10, String("RPM: ") + String(rpm, 0) + String("Scaler: ") + String(scaler, 2));
      Heltec.display->drawString(0, 40, String(millivolts));
      Heltec.display->display();       
}

void setup() 
{
  // Initialize serial communication
  Serial.begin(115200);

  Heltec.begin(true /*DisplayEnable Enable*/, true /*LoRa Disable*/, false /*Serial Enable*/);
  Heltec.display->screenRotate(ANGLE_180_DEGREE);
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Indigo Actionbutton...");
  Heltec.display->display();

  // Configure ADC width
  adc1_config_width(ADC_WIDTH_BIT_12);

  // Configure ADC attenuation
  adc1_config_channel_atten(ADC1_CHANNEL, ADC_ATTEN_DB_11);

  // Characterize ADC
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);

  // Initialize LEDs
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  fill_solid(leds, NUM_LEDS, CRGB::White);

  // Initialize the timer
  timer = timerBegin(0, 80, true); // 80 prescaler gives 1us tick (80MHz / 80 = 1MHz)
  timerAttachInterrupt(timer, &onTimer, true);

  // Setup LEDC for PWM on GPIO4
  ledcSetup(0, 20, 8);    // Channel 0, 20 Hz, 8-bit resolution
  ledcAttachPin(PWM_PIN, 0); // Attach PWM to GPIO4 on channel 0
  ledcWrite(0, 128);      // 50% duty cycle (128 out of 255)

  UpdateTimers(0, 0.172);  
}

void loop() 
{
     // Read the raw ADC value from the potentiometer
    int rawValue = adc1_get_raw(ADC1_CHANNEL);

    // Convert the raw ADC value to millivolts
    uint32_t mv = esp_adc_cal_raw_to_voltage(rawValue, adc_chars);
    float millivolts = mv / 1000.0f;

    // Clamp the millivolts value
    millivolts = std::min(millivolts, 3.12f);
    millivolts = std::max(millivolts, 0.0f);
    float scaler = millivolts / 3.12f;
    scaler = mapValue(scaler, 0.0f, 1.0f, 0.12f, 1.0f);

    static unsigned long lastMillis = 0;
    unsigned long currentMillis = millis();

  /*
    if (currentMillis - lastMillis >= interval) 
    {
      lastMillis = currentMillis;
      UpdateTimers(millivolts, scaler);
    }
   */

  // Check if it's time to update the LEDs
  if (updateLEDs) 
  {
    updateLEDs = false;  // Clear the flag
    FastLED.showColor(CRGB::White);
    FastLED.showColor(CRGB::Black);
  }
}
