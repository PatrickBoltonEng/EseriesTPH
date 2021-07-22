/*
 * Project ParticleEseries Temperature Pressure Humidity
 * Description:  Seeed BME280 sensor
 * Author:  From https://docs.particle.io/quickstart/aqmk-project/ modified by Patrick Bolton
 * Date: 03-30-21
 */

#include "Particle.h"
#include "math.h"
#include "Adafruit_BME280.h"
#include "JsonParserGeneratorRK.h"


#define SENSOR_READING_INTERVAL 60000

Adafruit_BME280 bme;

unsigned long lastInterval;

float a, b, c, TC, TF, PPa, P, RH, HI, DP;

void setup() {
  Serial.begin(9600);

  if (bme.begin()){
    Serial.println("BME280 Sensor ready.");
  }
  else
  {
    Serial.println("BME280 Sensor ERROR!");
  }

  lastInterval = millis();
}


void loop() 
{
  
  if ((millis() - lastInterval) > SENSOR_READING_INTERVAL)
  {
    getBMEValues(TC, PPa, RH);

    TF = (9.0f * TC / 5.0f) + 32.0f;
    // TF in Fahrenheit

    P = (PPa / 6894.75729f);
    // Pressure converted from Pa to psia

    HI = -42.379f + (2.04901523f*TF) + (10.14333127f*RH) -(0.22475541f*TF*RH) 
       - (0.00683783f*TF*TF) - (0.05481717f*RH*RH) + (0.00122874f*TF*TF*RH) 
       + (0.00085282f*TF*RH*RH) - (0.00000199f*TF*TF*RH*RH);
    // Heat Index Calculation from nws
    
    if(TC < 0.0f)
    {
      // Magnus formula constants for -40 C <= T <= 0 C from NIST
      a = 6.1121f;
      // in mbar
      b = 17.966f;
      c = 247.15f;
      float Psat = a * exp(b*TC/(c+TC));
      // saturation vapor pressure in mbar
      float Pact = (RH/100.0f)*Psat;
      // actual vapor pressure in mbar
      float DPC = (c*log(Pact/a)) / (b - log(Psat/a));
      // dew point temperature in C
      DP = ((9.0f * DPC) / 5.0f) + 32.0f;
      // dew point temperature in F

      Serial.println(Time.timeStr());
      Serial.printlnf("Temperature (F): %f", TF);
      Serial.printlnf("Pressure (psia): %f", P);
      Serial.printlnf("Humidity    (%%): %f", RH);
      Serial.printlnf("Heat Index  (F): %f", HI);
      Serial.printlnf("Dewpoint    (F): %f", DP);

      createEventPayload(TF, P, RH, HI, DP);
    }

    if (TC >= 0.0f)
    {
      // Magnus formula constants fpr 0 C <= T <= 50 C from NIST
      a = 6.1121f;
      // in mbar
      b = 17.368f;
      c = 238.88f;
      float Psat = a * exp(b*TC/(c+TC));
      // saturation vapor pressure in mbar
      float Pact = (RH/100.0f)*Psat;
      // actual vapor pressure in mbar
      float DPC = (c*log(Pact/a)) / (b - log(Psat/a));
      // dew point temperature in C
      DP = ((9.0f * DPC) / 5.0f) + 32.0f;
      // dew point temperature in F
      Serial.println(Time.timeStr());
      Serial.printlnf("Temperature (F): %f", TF);
      Serial.printlnf("Pressure (psia): %f", P);
      Serial.printlnf("Humidity    (%%): %f", RH);
      Serial.printlnf("Heat Index  (F): %f", HI);
      Serial.printlnf("Dewpoint    (F): %f", DP);

      createEventPayload(TF, P, RH, HI, DP);
    }
    lastInterval = millis(); 
  }
}

float getBMEValues(float &TC, float &PPa, float &RH)
{
  TC = bme.readTemperature();
  // TC in Celcius
  PPa = bme.readPressure();
  // Pressure in Pa
  RH = bme.readHumidity();
  // RH in %
  return 1;
}

void createEventPayload(float TF, float P, float RH, float HI, float DP)
{
  JsonWriterStatic<256> jw;

  {
    JsonWriterAutoObject obj(&jw);

    jw.insertKeyValue("Temperature (F)",TF);
    jw.insertKeyValue("Pressure (psia)", P);
    jw.insertKeyValue("Humidity (%%)", RH);
    jw.insertKeyValue("Heat Index (F)", HI);
    jw.insertKeyValue("Dew Point (F)", DP);
  }

  Particle.publish("Eseries-TPH", jw.getBuffer(), PRIVATE);

}
      
