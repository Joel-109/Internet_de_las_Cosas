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
        // Serial.print("TemperatureSensor DS18B20 reading: ");
        // Serial.println(temp);
        return temp;
    }
};

class FlameSensor : public ISensor {
private:
    int digitalPin;
public:
    FlameSensor(int pin) : digitalPin(pin) {
        pinMode(digitalPin, INPUT);
    }

    float readValue() override {
        int raw = digitalRead(digitalPin);
        // Serial.print("FlameSensor (pin ");
        // Serial.print(digitalPin);
        // Serial.print(") raw reading: ");
        // Serial.println(raw);
        return static_cast<float>(raw);
    }
};

class GasSensor : public ISensor {
private:
    int analogPin;
public:
    GasSensor(int pin) : analogPin(pin) {}

    float readValue() override {
        int raw = analogRead(analogPin);
        return static_cast<float>(raw);
    }
};

class IActuator {
public:
    virtual void setState(bool active) = 0;
    virtual ~IActuator() = default;
};

class RelayActuator {
private:
    int redPin;
    int greenPin;
    int bluePin;

public:
    RelayActuator(int red, int green, int blue)
      : redPin(red), greenPin(green), bluePin(blue)
    {
        pinMode(redPin, OUTPUT);
        pinMode(greenPin, OUTPUT);
        pinMode(bluePin, OUTPUT);
    }

    void setColor(int red, int green, int blue) {
        analogWrite(redPin, red);
        analogWrite(greenPin, green);
        analogWrite(bluePin, blue);
    }
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
        {
            Serial.println("Gas detected! Beep!");
            tone(piezoPin, 1000, 1000);
        }
        else {
            Serial.println("Gas level normal.");
            noTone(piezoPin);
        }
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
    ISensor* flameSensor;
    IDisplay* display;
    IActuator* led_temp;
    IActuator* led_gas;
    IActuator* piezo;
    float temperatureThreshold;
    float gasThreshold;
public:
    Controller(ISensor* temp,
               ISensor* gas,
               ISensor* flame,
               IDisplay* disp,
               IActuator* ledTempAct,
               IActuator* ledGasAct,
               IActuator* piezoAct,
               float tempThresh,
               float gasThresh)
      : temperatureSensor(temp), gasSensor(gas), flameSensor(flame), display(disp),
        led_temp(ledTempAct), led_gas(ledGasAct), piezo(piezoAct),
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

        float flame = flameSensor->readValue();
        Serial.print("Flame sensor = ");
        Serial.println(flame);

        display->showTemperature(temp);
        display->showGas(gas);

        led_temp->setState(temp >= temperatureThreshold);
        led_gas->setState(gas >= gasThreshold);
        piezo->setState(gas >= gasThreshold && temp >= temperatureThreshold);

        delay(1000);
    }
};

TemperatureSensor* tempSensor;
GasSensor* gasSensor;
LCDDisplay* lcd;
LEDActuator* led_temp;
LEDActuator* led_gas;
PiezoActuator* piezo;
Controller* controller;
FlameSensor* flameSensor;

void setup() {
    Serial.begin(9600);
    while (!Serial) {}
    Serial.println("System starting...");

    tempSensor = new TemperatureSensor(2);  
    tempSensor->begin();
    gasSensor = new GasSensor(A0);

    flameSensor = new FlameSensor(4);

    lcd = new LCDDisplay(0x27, 16, 2);  
    lcd->init();
    led_temp = new LEDActuator(13);
    led_gas = new LEDActuator(12);
    piezo = new PiezoActuator(7);
    piezo->setState(false);

    controller = new Controller(tempSensor, gasSensor, flameSensor, lcd, led_temp, led_gas, piezo, 30.0, 500.0);

    Serial.println("Controller initialized.");
}

void loop() {
    controller->update();
}