//---------------------------------------------------------------------------------------------------
// Inclusion des bibliothèques et definition des constantes
//---------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include "rgb_lcd.h"

#define Stop 450              // Valeur PWM neutre (comme main2.cpp)
#define Thash 900             // Période Timer5 pour fréquence PWM
#define Vmax 800
#define Vmin 0

#define MoteurG(Vg) OCR5A = Vg
#define MoteurD(Vd) OCR5B = Vd
#define MoteurGD(Vg, Vd) MoteurG(Vg); MoteurD(Vd)
#define MoteurStop() MoteurGD(Stop, Stop)

// Capteurs ultrason (pins compatibles main2.cpp)
Ultrasonic UltrasonicAv(2);  // Frontal (pin 2)
Ultrasonic UltrasonicDr(4);  // Droit (pin 4)
Ultrasonic UltrasonicGa(6);   // Gauche (pin 6)

//---------------------------------------------------------------------------------------------------
// Definition des Variables globales
//---------------------------------------------------------------------------------------------------
int Etat; 
int CaptAv, CaptDr, CaptGa;
int lastCaptAv = 0;  // Pour détection manœuvres
int Diff;
int VmaxD, VmaxG;    // Vitesses max ajustées selon batterie
int NivBat1, NivBat2, Enercon, Nm = 0;
float distanceParcourue = 0;  // Distance totale parcourue en mètres
float vitesseMoyenne = 0.3;   // Vitesse moyenne estimée en m/s
long NTime, ATime;
long int Time = 0, cTime = 0;

byte compteur = 0;

rgb_lcd lcd;  // Écran LCD RGB

enum { Avancer, Obstacle, Virage, Securite, Arret };

//---------------------------------------------------------------------------------------------------
// Fonction calcul niveau de batterie (de main2.cpp)
//---------------------------------------------------------------------------------------------------
int calcBatt() {
    unsigned long sum;
    sum = analogRead(A0);
    sum = (3 * sum * 4980 / 1023.00);
    sum = map(sum, 6000, 7400, 0, 100);
    return sum;
}

//---------------------------------------------------------------------------------------------------
// Fonction d'initialisation des moteurs
//---------------------------------------------------------------------------------------------------
void InitMoteurs() 
{
    DDRL = 0x18;  // PL3 et PL4
    DDRB = 0x80;  // PB7 LedToggle
    TCCR5A = (1 << COM5A1) + (1 << COM5B1);
    TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50);
    ICR5 = Thash;  // Utilise Thash comme main2.cpp
    MoteurStop();
    TIMSK5 = 1 << TOIE5;  // Activation interruption
}

//---------------------------------------------------------------------------------------------------
// Interruption Timer5 (de main2.cpp)
//---------------------------------------------------------------------------------------------------
ISR (TIMER5_OVF_vect) {
    compteur++;
    if (compteur == 50) {
        compteur = 0;
        cTime++;
        
        if (cTime == 14) {
            Time++;
            cTime = 0;
            // Calcul distance parcourue (estimation basée sur temps et vitesse moyenne)
            distanceParcourue += vitesseMoyenne;  // +vitesseMoyenne mètres par seconde
        }

        // Sauvegarde pour détection manœuvres
        lastCaptAv = CaptAv;
        
        // Lecture capteurs
        CaptAv = UltrasonicAv.MeasureInCentimeters();
        CaptDr = UltrasonicDr.MeasureInCentimeters();
        CaptGa = UltrasonicGa.MeasureInCentimeters();
    }
}

//---------------------------------------------------------------------------------------------------
// Fonction Arrivée (de main2.cpp)
//---------------------------------------------------------------------------------------------------
void arrive() {
    Time = ((NTime - ATime) / 1000);  // Temps en secondes

    lcd.setRGB(0, 255, 0);  // Vert
    MoteurStop();
    TIMSK5 = 0 << TOIE5;

    NivBat2 = calcBatt();
    Enercon = (NivBat1 - NivBat2) * Time * 0.6;

    int navigation = 0;

    while (analogRead(A2) <= 800) {  // Uniquement bouton A2
        
        if ((analogRead(A3)) < 400) {
            navigation++;
            while ((analogRead(A3)) < 400);
        }
        else if ((analogRead(A3)) > 600) {
            navigation--;
            while ((analogRead(A3)) > 600);
        }

        if (navigation < 0) navigation = 0;
        if (navigation > 4) navigation = 4;  // Augmenté pour inclure distance

        switch (navigation) {
            case 0:
                lcd.setCursor(0, 0);
                lcd.print("    Arrive!     ");
                lcd.setCursor(0, 1);
                lcd.print("                ");
                break;

            case 1:
                lcd.setCursor(0, 0);
                lcd.print(" Temps parcours ");
                lcd.setCursor(0, 1);
                lcd.print("     ");
                lcd.print(Time);
                lcd.print(" s       ");
                break;
            
            case 2:
                lcd.setCursor(0, 0);
                lcd.print("    Distance    ");
                lcd.setCursor(0, 1);
                lcd.print("    ");
                lcd.print(distanceParcourue, 1);  // 1 décimale
                lcd.print(" m       ");
                break;

            case 3:
                lcd.setCursor(0, 0);
                lcd.print(" Energie Cons.  ");
                lcd.setCursor(0, 1);
                lcd.print("     ");
                lcd.print(Enercon);
                lcd.print(" Ws       ");
                break;

            case 4:
                lcd.setCursor(0, 0);
                lcd.print("  Manoeuvres    ");
                lcd.setCursor(0, 1);
                lcd.print("     ");
                lcd.print(Nm);
                lcd.print(" fois       ");
                break;
        }
    }
}

//---------------------------------------------------------------------------------------------------
// Fonction Sécurité
//---------------------------------------------------------------------------------------------------
void FSeccurite() 
{
    Serial.println("Mode Sécurité activé");
    MoteurStop();
    
    lcd.setRGB(255, 0, 0);  // Rouge
    lcd.setCursor(0, 0);
    lcd.print("   SECURITE!    ");
    
    while (analogRead(A2) <= 800);  // Attente bouton uniquement
    
    lcd.clear();
    lcd.setRGB(255, 255, 255);  // Blanc
}

//---------------------------------------------------------------------------------------------------
// Fonction Virage
//---------------------------------------------------------------------------------------------------
void Fvirage() 
{
    while (CaptDr > 150 || CaptGa > 150)
    {
        CaptDr = UltrasonicDr.MeasureInCentimeters();
        CaptGa = UltrasonicGa.MeasureInCentimeters();
        
        if (CaptDr > CaptGa)
        {
            MoteurGD(VmaxG, VmaxD - 190);
            Serial.println("Virage à droite");
        }
        else
        {
            MoteurGD(600, VmaxD);
            Serial.println("Virage à gauche");
        }
    }
}

//---------------------------------------------------------------------------------------------------
// Fonction d'évitement des obstacles
//---------------------------------------------------------------------------------------------------
void FevitementObstacles() 
{
    Serial.println("Evitement obstacle");

    // Choix direction selon espace disponible
    if (CaptDr <= CaptGa) {
        while (CaptAv < 74) {
            MoteurGD(600, VmaxD);
            CaptAv = UltrasonicAv.MeasureInCentimeters();
        }
    }
    else {
        while (CaptAv < 74) {
            MoteurGD(VmaxG, 600);
            CaptAv = UltrasonicAv.MeasureInCentimeters();
        }
    }
}

//---------------------------------------------------------------------------------------------------
// Fonction d'avancer
//---------------------------------------------------------------------------------------------------
void Favancer()
{
    Serial.print("Gauche: ");
    Serial.print(CaptGa);
    Serial.print(" | Droite: ");
    Serial.println(CaptDr);

    Diff = CaptGa - CaptDr;

    float Kp = 1.5;
    float Correction = Kp * Diff;

    float Vitesse_Droite = VmaxD + Correction;
    float Vitesse_Gauche = VmaxG - Correction;

    Vitesse_Droite = constrain(Vitesse_Droite, 200, VmaxD);
    Vitesse_Gauche = constrain(Vitesse_Gauche, 200, VmaxG);

    MoteurGD(Vitesse_Gauche, Vitesse_Droite);
}

//---------------------------------------------------------------------------------------------------
// Fonction d'initialisation
//---------------------------------------------------------------------------------------------------
void setup() 
{
    ATime = millis();
    Serial.begin(9600);
    lcd.begin(16, 2);

    pinMode(A0, INPUT);  // Batterie
    pinMode(A2, INPUT);  // Bouton démarrage
    pinMode(A3, INPUT);  // Joystick
    pinMode(A4, INPUT);  // Surcharge moteur 1
    pinMode(A5, INPUT);  // Surcharge moteur 2
    pinMode(43, OUTPUT);

    digitalWrite(43, 0);
    InitMoteurs();
    digitalWrite(43, 1);

    NivBat1 = calcBatt();
    distanceParcourue = 0;  // Réinitialisation distance

    // Ajustement vitesses selon batterie
    int battLevel = calcBatt();
    VmaxD = 1250 - battLevel;
    VmaxG = 845 - battLevel;

    // Estimation vitesse moyenne selon niveau batterie
    vitesseMoyenne = 0.2 + (battLevel / 500.0);  // Entre 0.2 et 0.4 m/s

    lcd.setRGB(255, 255, 255);
    lcd.setCursor(0, 0);
    lcd.print(" Pret au depart ");
    lcd.setCursor(0, 1);
    lcd.print("Bat: ");
    lcd.print(battLevel);
    lcd.print("%");

    // Attente démarrage (bouton A2 uniquement)
    while (analogRead(A2) <= 800) {
        MoteurStop();
    }

    lcd.clear();
    Time = 0;
    sei();
}

//---------------------------------------------------------------------------------------------------
// Fonction principale loop
//---------------------------------------------------------------------------------------------------
void loop() 
{
    NTime = millis();

    // Détection timeout 10 minutes uniquement (pas d'infrarouge)
    if ((NTime - ATime) > 600000) {
        arrive();
    }

    // Détection manœuvres
    if (abs(CaptAv - lastCaptAv) > 40 && CaptAv < 100) {
        Nm++;
    }

    // Détection surcharge électrique
    if ((analogRead(A4) >= 3300) || (analogRead(A5) >= 3300)) {
        FSeccurite();
    }

    //-----------------------------------------------------------------------------------
    // Condition d'état
    //-----------------------------------------------------------------------------------
    if (CaptAv < 5 || CaptGa < 5 || CaptDr < 5)
    {
        Etat = Securite;
    }
    else if (CaptAv < 70)
    {
        Etat = Obstacle;
    }
    else if (CaptDr > 150 || CaptGa > 150)
    {
        Etat = Virage;
    }
    else
    {
        Etat = Avancer;
    }

    //-----------------------------------------------------------------------------------
    // State Machine
    //-----------------------------------------------------------------------------------
    switch (Etat)
    {
        case Avancer:
            Favancer();
            break;

        case Obstacle:
            FevitementObstacles();
            break;

        case Virage:
            Fvirage();
            break;
        
        case Securite:
            FSeccurite();
            break;

        default:
            MoteurStop();
            break;
    }
}
