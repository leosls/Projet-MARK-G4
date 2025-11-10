//---------------------------------------------------------------------------------------------------
// Inclusion des bibliothèques et definition des constantes
//---------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <ultrasonic.h>
#include <Wire.h> 

#define Stop 400
#define Vmax 800
#define Vmin 0

#define MoteurG(Vg) OCR5A = Vg // Vg entre 0 et 800 (Possible [0...1999] si on modifie le prescaler)
#define MoteurD(Vd) OCR5B = Vd // Vd entre 0 et 800 (Possible [0...1999] si on modifie le prescaler)
#define MoteurGD(Vg, Vd) MoteurG(Vg); MoteurD(Vd)
#define MoteurStop() MoteurGD(Stop, Stop)

Ultrasonic UltrasonicAv(2); 
Ultrasonic UltrasonicDr(4); 
Ultrasonic UltrasonicGa(6); 



//---------------------------------------------------------------------------------------------------
// Definition des Variables globales
//---------------------------------------------------------------------------------------------------
int Etat; 
int CaptAv;
int CaptDr;
int CaptGa;
int Diff;
enum { Avancer, Obstacle, Virage }; // Définition des états possibles


//---------------------------------------------------------------------------------------------------
// Fonction d'initialisation des moteurs
//---------------------------------------------------------------------------------------------------
void InitMoteurs() 
{
    DDRL = 0x18 ; // PL3 et PL4
    DDRB = 0x80 ; // PB7 LedToggle
    // COM5B_1:0 = 10   -> clear sur egalite++, set sur egalite--
    // WGM5_3:1 = 1000  -> mode 8 => ICR5 defini le TOP
    TCCR5A = (1 << COM5A1) + (1 << COM5B1);
    TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50); // CS_12:10  = 001  -> prediv par 1
    ICR5 = Vmax; // 1999 correspond a f = 4khz
    MoteurStop();
    // Interrution de débordement du timer
    // TIMSK5 = 1 << TOIE5;
}

//---------------------------------------------------------------------------------------------------
// Fonction Arret Distance
//---------------------------------------------------------------------------------------------------
void Arretdistance() 
{
    long DistAvant;
    long DistDroite;
    long DistGauche;

    DistAvant = UltrasonicAv.MeasureInCentimeters();
    DistDroite = UltrasonicDr.MeasureInCentimeters();
    DistGauche = UltrasonicGa.MeasureInCentimeters();

    if (DistAvant<5 || DistGauche<5 || DistDroite<5)
    {
        MoteurStop();
    }    
}
//---------------------------------------------------------------------------------------------------
// Fonction Arret Moteur Bloqué
//---------------------------------------------------------------------------------------------------
void ArretMB()
{
    
}


//---------------------------------------------------------------------------------------------------
// Fonction d'évitement des obstacles
//---------------------------------------------------------------------------------------------------
void EvitementObstacles() 
{
    // Code pour éviter les obstacles
    Serial.println("Evitement des obstacles activé");

    MoteurStop();
    
}

//---------------------------------------------------------------------------------------------------
// Fonction d'avancer
//----------------------------------------------------------------------------------------------

void Favancer()
{
    CaptDr = UltrasonicDr.MeasureInCentimeters();
    CaptGa = UltrasonicGa.MeasureInCentimeters();
    // CaptDr = (UltrasonicDrAr.MeasureInCentimeters() + UltrasonicDr.MeasureInCentimeters()) / 2;
    // CaptGa = (UltrasonicGaAr.MeasureInCentimeters() + UltrasonicGa.MeasureInCentimeters()) / 2;
	Serial.print("CaptGauche: ");
	Serial.print(CaptGa);
	Serial.print(" | CaptDroite: ");
	Serial.println(CaptDr);

    Diff = CaptGa - CaptDr;
	Serial.print("Diff: ");
	Serial.println(Diff);

    float Kp = 0.3;

    float Correction = Kp * Diff;

	Serial.print("Correction: ");
	Serial.println(Correction);

    float Vitesse_Droite = 250 + Correction;
    float Vitesse_Gauche = 250 - Correction;

    // Limiter les vitesses entre Vmin et Vmax
    Vitesse_Droite = constrain(Vitesse_Droite, 0, 300);
    Vitesse_Gauche = constrain(Vitesse_Gauche, 0, 300);

	Serial.print("Vitesse Gauche: ");
	Serial.print(Vitesse_Gauche);
	Serial.print(" | Vitesse Droite: ");
	Serial.println(Vitesse_Droite);

    MoteurGD(Vitesse_Gauche, Vitesse_Droite);

	Serial.println("\n\n");
}

//---------------------------------------------------------------------------------------------------
// Fonction d'initialisation
//---------------------------------------------------------------------------------------------------
void setup() 
{
    // Initialisation des broches, de la communication série, etc.
    Serial.begin(9600);

    pinMode(43,OUTPUT);
    digitalWrite(43,0);
    InitMoteurs();
    // sei();
    digitalWrite(43,1);   
}

//---------------------------------------------------------------------------------------------------
// Fonction principale loop
//---------------------------------------------------------------------------------------------------
void loop() 
{
	CaptAv = UltrasonicAv.MeasureInCentimeters();
    CaptDr = UltrasonicDr.MeasureInCentimeters();
    CaptGa = UltrasonicGa.MeasureInCentimeters();
    
    if(CaptAv < 20)
    {
        Etat = Obstacle;
    }
    else
    {
        Etat = Avancer;
    }

    switch (Etat)
    {
        case Avancer:
           	Favancer();

            break;

        case Obstacle:
            EvitementObstacles();
            break;

		case Virage:
			
			
			break;
        
        default:
            //code ici
            break;
    }
}
