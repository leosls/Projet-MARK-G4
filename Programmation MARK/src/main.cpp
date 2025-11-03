//---------------------------------------------------------------------------------------------------
// Inclusion des bibliothèques et definition des constantes
//---------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <ultrasonic.h>

#define Stop 400
#define Vmax 800
#define Vmin 0

#define MoteurG(Vg) OCR5A = Vg // Vg entre 0 et 800 (Possible [0...1999] si on modifie le prescaler)
#define MoteurD(Vd) OCR5B = Vd // Vd entre 0 et 800 (Possible [0...1999] si on modifie le prescaler)
#define MoteurGD(Vg, Vd) MoteurG(Vg); MoteurD(Vd)
#define MoteurStop() MoteurGD(Stop, Stop)


//---------------------------------------------------------------------------------------------------
// Definition des Variables globales
//---------------------------------------------------------------------------------------------------
int Etat; 
enum { Avancer, Obstacle }; // Définition des états possibles


//---------------------------------------------------------------------------------------------------
// Fonction d'initialisation des moteurs
//---------------------------------------------------------------------------------------------------
void InitMoteurs() 
{
    DDRL = 0x18; // PL3 et PL4 en sortie (PWM)
    DDRB = 0x80; // PB7 Controle led

    TCCR5A = (1 << COM5A1) + (1 << COM5B1); // Mode PWM, non inversé
    TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50); // Mode PWM, prescaler = 1
    ICR5 = Vmax; // Période de la PWM = 4000 cycles d'horloge = 4ms à 1MHz

    MoteurStop(); // Moteurs à l'arrêt

    TIMSK5 = 1 << TOIE5; // Interruption sur débordement du Timer5
}

//---------------------------------------------------------------------------------------------------
// Fonction d'évitement des obstacles
//---------------------------------------------------------------------------------------------------
void EvitementObstacles() 
{
    // Code pour éviter les obstacles
    Serial.println("Evitement des obstacles activé");

}

//---------------------------------------------------------------------------------------------------
// Fonction d'initialisation
//---------------------------------------------------------------------------------------------------
void setup() 
{
    // Initialisation des broches, de la communication série, etc.
    Serial.begin(9600);
    InitMoteurs();

}

//---------------------------------------------------------------------------------------------------
// Fonction principale loop
//---------------------------------------------------------------------------------------------------
void loop() 
{

    switch (Etat)
    {
    case Avancer:
        /* code */

    break;

    case Obstacle:
        EvitementObstacles();
        Etat = Avancer;
    break;
    
    default:
        //code ici
    break;
    }
    
}