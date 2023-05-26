#ifndef LCD_h
#define LCD_h

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Dirección I2C de la pantalla LCD y tamaño de 16 columnas y 2 filas.

class LCD_Display {
public:
    void init();
    void showDateTime(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day, uint8_t month, uint16_t year);
};

void LCD_Display::init() {
    lcd.begin(16, 2);  // Inicializar la pantalla LCD con el tamaño especificado.
    lcd.backlight();
}

void LCD_Display::showDateTime(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day, uint8_t month, uint16_t year) {
    lcd.clear();  // Limpiar la pantalla LCD.

    // Mostrar la hora en la primera fila.
    lcd.setCursor(0, 0);
    lcd.print("Hora: ");
    if (hour < 10) {
        lcd.print("0");
    }
    lcd.print(hour);
    lcd.print(":");
    if (minute < 10) {
        lcd.print("0");
    }
    lcd.print(minute);
    lcd.print(":");
    if (second < 10) {
        lcd.print("0");
    }
    lcd.print(second);

    // Mostrar la fecha en la segunda fila.
    lcd.setCursor(0, 1);
    lcd.print("Fecha:");
    if (day < 10) {
        lcd.print("0");
    }
    lcd.print(day);
    lcd.print("/");
    if (month < 10) {
        lcd.print("0");
    }
    lcd.print(month);
    lcd.print("/");
    lcd.print(year);
}

#endif
