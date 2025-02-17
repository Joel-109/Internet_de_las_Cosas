#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

class ISensor {
public:
    virtual float readValue() = 0;
    virtual ~ISensor() = default;
};

class TemperatureSensor : public ISensor {
private:
    OneWire oneWire;
    DallasTemperature sensors;
public:
    TemperatureSensor(uint8_t pin)
      : oneWire(pin), sensors(&oneWire)
    {}

    void begin() {
        sensors.begin();
    }

    float readValue() override {
        sensors.requestTemperatures();
        float temp = sensors.getTempCByIndex(0);
        Serial.print("TemperatureSensor DS18B20 reading: ");
        Serial.println(temp);
        return temp;
    }
};

class GasSensor : public ISensor {
private:
    int analogPin;
public:
    GasSensor(int pin) : analogPin(pin) {}

    float readValue() override {
        int raw = analogRead(analogPin);
        Serial.print("GasSensor (pin ");
        Serial.print(analogPin);
        Serial.print(") raw reading: ");
        Serial.println(raw);
        return static_cast<float>(raw);
    }
};

class IActuator {
public:
    virtual void setState(bool active) = 0;
    virtual ~IActuator() = default;
};

class LEDActuator : public IActuator {
private:
    int ledPin;
public:
    LEDActuator(int pin) : ledPin(pin) {
        pinMode(ledPin, OUTPUT);
    }
    void setState(bool active) override {
        digitalWrite(ledPin, active ? HIGH : LOW);
    }
};

class PiezoActuator : public IActuator {
private:
    int piezoPin;
public:
    PiezoActuator(int pin) : piezoPin(pin) {
        pinMode(piezoPin, OUTPUT);
    }
    void setState(bool active) override {
        if (active)
            tone(piezoPin, 1000, 1000);
        else
            noTone(piezoPin);
    }
};

class IDisplay {
public:
    virtual void showTemperature(float temperature) = 0;
    virtual void showGas(float gasValue) = 0;
    virtual ~IDisplay() = default;
};

class LCDDisplay : public IDisplay {
private:
    LiquidCrystal_I2C lcd;
public:
    LCDDisplay(uint8_t addr, uint8_t cols, uint8_t rows)
      : lcd(addr, cols, rows)
    {}

    void init() {
        lcd.init();
        lcd.begin(16, 2);
        lcd.backlight();
    }

    void showTemperature(float temperature) override {
        lcd.setCursor(0, 0);
        lcd.print("Temp: " + String(temperature) + " C   ");
    }
    void showGas(float gasValue) override {
        lcd.setCursor(0, 1);
        lcd.print("Gas: " + String(gasValue) + "     ");
    }
};

class Controller {
private:
    ISensor* temperatureSensor;
    ISensor* gasSensor;
    IDisplay* display;
    IActuator* led;
    IActuator* piezo;
    float temperatureThreshold;
    float gasThreshold;
public:
    Controller(ISensor* temp,
               ISensor* gas,
               IDisplay* disp,
               IActuator* ledAct,
               IActuator* piezoAct,
               float tempThresh,
               float gasThresh)
      : temperatureSensor(temp), gasSensor(gas), display(disp),
        led(ledAct), piezo(piezoAct),
        temperatureThreshold(tempThresh), gasThreshold(gasThresh)
    {}

    void init() {
        // Initialization is handled by each component.
    }

    void update() {
        float temp = temperatureSensor->readValue();
        float gas = gasSensor->readValue();
        Serial.print("Controller: Temperature = ");
        Serial.print(temp);
        Serial.print(" °C, GasSensor = ");
        Serial.println(gas);

        display->showTemperature(temp);
        display->showGas(gas);

        led->setState(temp >= temperatureThreshold);
        piezo->setState(gas >= gasThreshold);

        delay(1000);
    }
};

TemperatureSensor* tempSensor;
GasSensor* gasSensor;
LCDDisplay* lcd;
LEDActuator* led;
PiezoActuator* piezo;
Controller* controller;

void setup() {
    Serial.begin(9600);
    while (!Serial) {}
    Serial.println("System starting...");

    tempSensor = new TemperatureSensor(2);  
    tempSensor->begin();
    gasSensor = new GasSensor(A0);

    lcd = new LCDDisplay(0x27, 16, 2);  
    lcd->init();
    led = new LEDActuator(13);
    piezo = new PiezoActuator(7);

    controller = new Controller(tempSensor, gasSensor, lcd, led, piezo, 80.0, 100.0);

    Serial.println("Controller initialized.");
}

void loop() {
    controller->update();
}
