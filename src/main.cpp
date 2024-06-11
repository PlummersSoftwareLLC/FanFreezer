#include <Wire.h>
#include <heltec.h>                // Display
#include <driver/adc.h>
#include <esp_adc_cal.h>

#define ADC1_CHANNEL (ADC1_CHANNEL_0)  // GPIO36 if ADC1 channel 0

#define DEFAULT_RPM  1500

// Define the pin where the TACH signal is connected
#define TACH_PIN   2

const int pwmPin = 2;

esp_adc_cal_characteristics_t *adc_chars;

unsigned long lastMillis = 0;
const unsigned long interval = 1000; // 1 second

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
}

void loop() 
{
  static unsigned long lastMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= interval) {
    lastMillis = currentMillis;

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

    // Print the pulse count frequency
    Serial.print("Pulse frequency: ");
    Serial.print(frequency);
    Serial.println(" Hz");

    // Read the raw ADC value from the potentiometer
    int rawValue = adc1_get_raw(ADC1_CHANNEL);

    // Convert the raw ADC value to millivolts
    uint32_t millivolts = esp_adc_cal_raw_to_voltage(rawValue, adc_chars);

    // Print the value to the Serial Monitor
    Serial.print("Raw Value: ");
    Serial.print(rawValue);
    Serial.print(" - Millivolts: ");
    Serial.println(millivolts);

    Heltec.display->clear();
    Heltec.display->drawString(0, 0, "Pulse Frequency (Hz):");
    Heltec.display->drawString(0, 10, String("RPM: ") + String(frequency * 30, 0));
    Heltec.display->drawString(0, 40, String((float)millivolts/1000.0f));
    Heltec.display->display();
  }

  // Add a small delay to prevent flickering
  delay(100);
}
