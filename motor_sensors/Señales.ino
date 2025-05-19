#include <wiringPi.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>

using namespace std;

// Pines TRIG y ECHO para 3 sensores
const int TRIG_PINS[3] = {0, 2, 4};  // GPIO 17, 27, 23 (WiringPi numbers)
const int ECHO_PINS[3] = {1, 3, 5};  // GPIO 18, 22, 24 (WiringPi numbers)

float medirDistancia(int trig, int echo) {
    digitalWrite(trig, LOW);
    delay(50);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    long startTime = micros();
    while (digitalRead(echo) == LOW) {
        if (micros() - startTime > 50000) return -1;
    }

    long start = micros();
    while (digitalRead(echo) == HIGH) {
        if (micros() - start > 30000) return -1;
    }

    long duration = micros() - start;
    float distance = duration * 0.01715;  // 34300 cm/s / 2
    return distance;
}

int main() {
    // Inicializar WiringPi
    if (wiringPiSetup() == -1) {
        cerr << "Error al inicializar WiringPi" << endl;
        return 1;
    }

    // Configurar pines
    for (int i = 0; i < 3; ++i) {
        pinMode(TRIG_PINS[i], OUTPUT);
        pinMode(ECHO_PINS[i], INPUT);
        digitalWrite(TRIG_PINS[i], LOW);
    }

    // Configurar puerto serial
    int serial = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial == -1) {
        cerr << "No se pudo abrir /dev/serial0" << endl;
        return 1;
    }

    struct termios options;
    tcgetattr(serial, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(serial, TCIFLUSH);
    tcsetattr(serial, TCSANOW, &options);

    char input;
    while (true) {
        if (read(serial, &input, 1) > 0) {
            if (input == 'I' || input == 'i') {
                for (int i = 0; i < 3; ++i) {
                    float distancia = medirDistancia(TRIG_PINS[i], ECHO_PINS[i]);
                    char buffer[64];
                    snprintf(buffer, sizeof(buffer), "Sensor %d: %.2f cm\n", i + 1, distancia);
                    write(serial, buffer, strlen(buffer));
                    delay(100);
                }
            }
        }
    }

    close(serial);
    return 0;
}
