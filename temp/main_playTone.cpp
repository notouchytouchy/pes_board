#include "mbed.h"

// Pin-Definition (PWM-fähig auf dem Nucleo F446RE)
PwmOut speaker(PA_0);

// Frequenzen der Noten (in Hz)
#define NOTE_C4  261
#define NOTE_D4  294
#define NOTE_E4  329
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define REST     0

// Die Unterfunktion zum Abspielen eines Tons
void playTone(float frequency, float duration_ms, float volume = 0.2) {
    if (frequency <= 0) {
        speaker.write(0); // Stille
    } else {
        speaker.period(1.0 / frequency); // Periode = 1 / Frequenz
        speaker.write(volume);           // "Duty Cycle" bestimmt die Energie am Speaker
    }
    
    // Warte die Dauer des Tons ab
    thread_sleep_for((int)duration_ms);
    
    // Kurze Pause zwischen den Noten, damit sie nicht verschwimmen
    speaker.write(0);
    thread_sleep_for(20); 
}

int main() {
    // Melodie-Array (Note, Dauer in ms)
    // Ausschnitt aus Fluch der Karibik (Main Theme)
    int melody[][2] = {
        {NOTE_A4, 125}, {NOTE_C5, 125}, // Auftakt
        {NOTE_D5, 250}, {NOTE_D5, 125}, {NOTE_D5, 125}, 
        {NOTE_D5, 250}, {NOTE_E5, 125}, {NOTE_F5, 250}, 
        {NOTE_F5, 125}, {NOTE_F5, 125}, {NOTE_F5, 250}, 
        {NOTE_G5, 125}, {NOTE_E5, 250}, {NOTE_E5, 125}, 
        {NOTE_D5, 125}, {NOTE_C5, 125}, {NOTE_C5, 125}, {NOTE_D5, 250}
    };

    int notes = sizeof(melody) / sizeof(melody[0]);

    while (true) {
        for (int i = 0; i < notes; i++) {
            playTone(melody[i][0], melody[i][1]);
        }
        
        thread_sleep_for(2000); // 2 Sekunden Pause vor der Wiederholung
    }
}