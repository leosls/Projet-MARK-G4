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
enum { Avancer, Obstacle, Virage, Securite, Arret }; // Définition des états possibles


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
    // TIMSK5 = 1 << TOIE5; // Pas d'interruption pour l'instant car conflit avec le moniteur série
}

//---------------------------------------------------------------------------------------------------
// Fonction Arret Distance
//---------------------------------------------------------------------------------------------------
void FSeccurite() 
{
    CaptAv = UltrasonicAv.MeasureInCentimeters();
    CaptDr = UltrasonicDr.MeasureInCentimeters();
    CaptGa = UltrasonicGa.MeasureInCentimeters();

	Serial.println("Mode Sécurité activé");

	MoteurStop();

}

//---------------------------------------------------------------------------------------------------
// Fonction Virage
//---------------------------------------------------------------------------------------------------
void Fvirage() 
{
    while(CaptDr > 150 || CaptGa > 150)
	{
		CaptDr = UltrasonicDr.MeasureInCentimeters();
		CaptGa = UltrasonicGa.MeasureInCentimeters();
		
		if (CaptDr > CaptGa)
		{
			// Tourner à droite
			MoteurGD(275, 240); // TBV, ajuster les vitesses selon les tests
			Serial.println("Virage à droite");
		}
		else
		{
			// Tourner à gauche
			MoteurGD(240, 275); // TBV, ajuster les vitesses selon les tests
			Serial.println("Virage à gauche");
		}
	}
}

//---------------------------------------------------------------------------------------------------
// Fonction d'évitement des obstacles
//---------------------------------------------------------------------------------------------------
void FevitementObstacles() 
{
    // Code pour éviter les obstacles
    Serial.println("Evitement des obstacles activé");

    MoteurStop();
}

//---------------------------------------------------------------------------------------------------
// Fonction d'avancer
//---------------------------------------------------------------------------------------------------
void Favancer()
{
    CaptDr = UltrasonicDr.MeasureInCentimeters();
    CaptGa = UltrasonicGa.MeasureInCentimeters();
    // CaptDr = (UltrasonicDrAr.MeasureInCentimeters() + UltrasonicDr.MeasureInCentimeters()) / 2; // Moyenne des deux capteurs non utilisée car problème lors de la correction
    // CaptGa = (UltrasonicGaAr.MeasureInCentimeters() + UltrasonicGa.MeasureInCentimeters()) / 2;
	
	Serial.print("CaptGauche: ");
	Serial.print(CaptGa);
	Serial.print(" | CaptDroite: ");
	Serial.println(CaptDr);

    Diff = CaptGa - CaptDr;

	Serial.print("Diff: ");
	Serial.println(Diff);

	// Constante proportionnelle
    float Kp = 1; // TBV à terme (à ajuster selon les tests)

	// Calcul de la correction
    float Correction = Kp * Diff;

	Serial.print("Correction: ");
	Serial.println(Correction);

	// Calcul des vitesses des moteurs
    float Vitesse_Droite = 250 + Correction; // TBV, vitesse de base à ajuster car lente pour l'instant 
    float Vitesse_Gauche = 250 - Correction;

    // Limiter les vitesses entre Vmin et Vmax
    Vitesse_Droite = constrain(Vitesse_Droite, 0, 300);
    Vitesse_Gauche = constrain(Vitesse_Gauche, 0, 300);

	Serial.print("Vitesse Gauche: ");
	Serial.print(Vitesse_Gauche);
	Serial.print(" | Vitesse Droite: ");
	Serial.println(Vitesse_Droite);
	Serial.println("\n\n");

    MoteurGD(Vitesse_Gauche, Vitesse_Droite);

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
    // sei(); // Pas d'interruption pour l'instant car conflit avec le moniteur série
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
    
	//-----------------------------------------------------------------------------------
	// Condition d'état
	//-----------------------------------------------------------------------------------
    if(CaptAv < 20)
    {
        Etat = Obstacle;
    }
	else if (CaptAv < 5 || CaptGa < 5 || CaptDr < 5)
    {
        Etat = Securite;
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
			
			break;

		case Arret:
			
			break;

        default:
			MoteurStop();
			break;
    }
}