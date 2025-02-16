#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

constexpr size_t MAX_OBSERVERS = 5;

class IObserver {
public:
    virtual void update(float value) = 0;
    virtual ~IObserver() = default;
};

class SensorSubject {
protected:
    IObserver* observers[MAX_OBSERVERS];
    size_t observerCount;
public:
    SensorSubject() : observerCount(0) {}

    void attach(IObserver* observer) {
        if (observerCount < MAX_OBSERVERS) {
            observers[observerCount++] = observer;
            Serial.print("Observer attached: ");
            Serial.println((uintptr_t)observer, HEX);
        } else {
            Serial.println("Observer attach failed: max observers reached");
        }
    }

    void notifyObservers(float value) {
        Serial.print("Notifying observers with value: ");
        Serial.println(value);
        for (size_t i = 0; i < observerCount; i++) {
            observers[i]->update(value);
        }
    }
};

class ISensor {
public:
    virtual float readValue() = 0;
    virtual ~ISensor() = default;
};

class TemperatureSensor : public SensorSubject, public ISensor {
private:
    int analogPin;
public:
    TemperatureSensor(int pin) : analogPin(pin) {}

    float readValue() override {
        int rawValue = analogRead(analogPin);
        float voltage = (rawValue / 1023.0f) * 5000.0f;
        float temperature = (voltage - 500.0f) / 10.0f;
        Serial.print("TemperatureSensor (pin ");
        Serial.print(analogPin);
        Serial.print(") raw reading: ");
        Serial.print(rawValue);
        Serial.print(", voltage: ");
        Serial.print(voltage);
        Serial.print(" mV, temperature: ");
        Serial.println(temperature);
        notifyObservers(temperature);
        return temperature;
    }
};

class GasSensor : public SensorSubject, public ISensor {
private:
    int analogPin;
public:
    GasSensor(int pin) : analogPin(pin) {}

    float readValue() override {
        int rawValue = analogRead(analogPin);
        float gasValue = static_cast<float>(rawValue);
        Serial.print("GasSensor (pin ");
        Serial.print(analogPin);
        Serial.print(") raw reading: ");
        Serial.println(rawValue);
        notifyObservers(gasValue);
        return gasValue;
    }
};

// --- LED Alert Observer ---
class LEDAlert : public IObserver {
private:
    int ledPin;
    float threshold;
public:
    LEDAlert(int pin, float thresh) : ledPin(pin), threshold(thresh) {
        pinMode(ledPin, OUTPUT);
        Serial.print("LEDAlert initialized on pin ");
        Serial.print(ledPin);
        Serial.print(" with threshold ");
        Serial.println(threshold);
    }
    
    void update(float value) override {
        digitalWrite(ledPin, (value >= threshold) ? HIGH : LOW);
        Serial.print("LEDAlert: value ");
        Serial.print(value);
        Serial.print(" -> ");
        Serial.println((value >= threshold) ? "LED ON" : "LED OFF");
    }
};

// --- Piezo Alert Observer ---
class PiezoAlert : public IObserver {
private:
    int piezoPin;
    float threshold;
public:
    PiezoAlert(int pin, float thresh) : piezoPin(pin), threshold(thresh) {
        pinMode(piezoPin, OUTPUT);
        Serial.print("PiezoAlert initialized on pin ");
        Serial.print(piezoPin);
        Serial.print(" with threshold ");
        Serial.println(threshold);
    }
    
    void update(float value) override {
        digitalWrite(piezoPin, (value >= threshold) ? HIGH : LOW);
        Serial.print("PiezoAlert: value ");
        Serial.print(value);
        Serial.print(" -> ");
        Serial.println((value >= threshold) ? "Piezo ON" : "Piezo OFF");
    }
};

class SystemController {
private:
    TemperatureSensor* tempSensor;
    GasSensor* gasSensor;
    LiquidCrystal_I2C* lcd;
    
public:
    SystemController(TemperatureSensor* temp, GasSensor* gas, LiquidCrystal_I2C* display)
      : tempSensor(temp), gasSensor(gas), lcd(display)
    {}
    
    void begin() {
        Serial.println("Initializing SystemController...");
        lcd->init();
        lcd->begin(16, 2);
        lcd->backlight();
        Serial.println("LCD initialized.");
    }
    
    void update() {
        Serial.println("----- Update Cycle Start -----");
        float temperature = tempSensor->readValue();
        float gasValue    = gasSensor->readValue();
        
        Serial.print("Controller: Temperature = ");
        Serial.print(temperature);
        Serial.print(" °C, GasSensor = ");
        Serial.println(gasValue);
        
        lcd->clear();
        lcd->setCursor(0, 0);
        lcd->print("Temp: " + String(temperature) + " C");
        lcd->setCursor(0, 1);
        lcd->print("Gas: " + String(gasValue));
        Serial.println("Display updated.");
        Serial.println("----- Update Cycle End -----\n");
        
        delay(1000);
    }
};

TemperatureSensor tempSensor(A1);
GasSensor gasSensor(A0);
LEDAlert ledAlert(13, 80);      
PiezoAlert piezoAlert(7, 100);  
LiquidCrystal_I2C lcd(0x27, 16, 2);
SystemController controller(&tempSensor, &gasSensor, &lcd);

void setup() {
    Serial.begin(9600);
    while (!Serial) {} 
    Serial.println("System starting...");

    tempSensor.attach(&ledAlert);
    gasSensor.attach(&piezoAlert);

    controller.begin();
    Serial.println("SystemController started.");
}

void loop() {
    controller.update();
}
