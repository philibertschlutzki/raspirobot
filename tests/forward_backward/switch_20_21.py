#!/usr/bin/env python3
"""
GPIO Test Script für Raspberry Pi 5
Schaltet GPIO 20 und GPIO 21 für jeweils eine Sekunde
"""

import RPi.GPIO as GPIO
import time

# GPIO-Setup
GPIO.setmode(GPIO.BCM)  # BCM-Nummerierung verwenden
GPIO.setwarnings(False)  # Warnungen deaktivieren

# GPIO-Pins definieren
GPIO_PIN_20 = 20
GPIO_PIN_21 = 21

def setup_gpio():
    """GPIO-Pins als Output konfigurieren"""
    try:
        GPIO.setup(GPIO_PIN_20, GPIO.OUT)
        GPIO.setup(GPIO_PIN_21, GPIO.OUT)
        
        # Pins initial auf LOW setzen
        GPIO.output(GPIO_PIN_20, GPIO.LOW)
        GPIO.output(GPIO_PIN_21, GPIO.LOW)
        
        print("GPIO-Pins erfolgreich konfiguriert")
        return True
    except Exception as e:
        print(f"Fehler beim Setup der GPIO-Pins: {e}")
        return False

def test_gpio_pins():
    """Testet GPIO 20 und GPIO 21 für jeweils eine Sekunde"""
    print("Starte GPIO-Test...")
    
    try:
        # GPIO 20 für eine Sekunde einschalten
        print("GPIO 20 wird für 1 Sekunde eingeschaltet...")
        GPIO.output(GPIO_PIN_20, GPIO.HIGH)
        time.sleep(10)
        GPIO.output(GPIO_PIN_20, GPIO.LOW)
        print("GPIO 20 ausgeschaltet")
        
        # Kurze Pause zwischen den Tests
        time.sleep(0.5)
        
        # GPIO 21 für eine Sekunde einschalten
        print("GPIO 21 wird für 1 Sekunde eingeschaltet...")
        GPIO.output(GPIO_PIN_21, GPIO.HIGH)
        time.sleep(10)
        GPIO.output(GPIO_PIN_21, GPIO.LOW)
        print("GPIO 21 ausgeschaltet")
        
        print("GPIO-Test erfolgreich abgeschlossen")
        
    except Exception as e:
        print(f"Fehler während des GPIO-Tests: {e}")

def cleanup():
    """GPIO-Ressourcen freigeben"""
    try:
        GPIO.cleanup()
        print("GPIO-Cleanup durchgeführt")
    except Exception as e:
        print(f"Fehler beim Cleanup: {e}")

def main():
    """Hauptfunktion"""
    print("=== Raspberry Pi 5 GPIO Test Script ===")
    print("Testet GPIO 20 und GPIO 21")
    print("----------------------------------------")
    
    # GPIO-Setup
    if not setup_gpio():
        return
    
    try:
        # GPIO-Test durchführen
        test_gpio_pins()
        
    except KeyboardInterrupt:
        print("\nTest durch Benutzer unterbrochen")
    
    finally:
        # Cleanup
        cleanup()
        print("Programm beendet")

if __name__ == "__main__":
    main()