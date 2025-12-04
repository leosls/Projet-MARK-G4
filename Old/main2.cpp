/*
======================================
VIDELOT Romain : Groupe Projet MARK6
-Programme Robot Mark Groupe 6 V1.14-
======================================
*/
// Valeur fixes
#define Stop 450
#define Thash 900
#define CoefD 0.67
#define CoefG 0.42
#define CoefEncG 320
#define CoefEncD 390

// Macros
#define LedToggle digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN))
#define MoteurG(Vg) OCR5A=Vg // Vg in [0... 1999]
#define MoteurD(Vd) OCR5B=Vd // VD in [0... 1999]
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

// déclaration bibliothèques
#include <SoftwareSerial.h>
#include "Ultrasonic.h"
#include <Wire.h>
#include "rgb_lcd.h"
#include <Encoder.h>
#include <Grove_LED_Bar.h>
#include "SparkFunLSM6DS3.h"

// déclaration objet "capteur ultrason"
Ultrasonic ultrasonicG(8);
Ultrasonic ultrasonicF(10);
Ultrasonic ultrasonicD(12);

// déclaration objet "Encodeurs"
Encoder knobLeft(18, 29);
Encoder knobRight(27, 19);

// déclaration objet "Led Bar"
Grove_LED_Bar bar(5, 4, 0);

// déclaration objet "Accelerometre"
LSM6DS3 myIMU( I2C_MODE, 0x6A );

// variables pour les 3 capteur ultrason
volatile long RangeInCentimetersD;
volatile long RangeInCentimetersF;
volatile long RangeInCentimetersG;

// variables global du programme
int VmaxD, VmaxG, Pos, NivBat1, NivBat2, Enercon, Nm=0, etat_20_cm;


const byte infrared = 6;
long sumVoltage = 0;

/*long int ValEncDA = 0;
long int ValEncGA = 0;
long int ValEncDN = 1000;
long int ValEncGN = 1000;*/

long int cTime = 0, Time = 0;
long Nvit = 0,NTime,ATime;

//Déclaration variables numériques
rgb_lcd lcd;
byte ledbar;
byte compteur;
byte seconde;

void initMoteurs() {  // MoteurG :OC5A=PIN46-PL3, MoteurD : OC5B=PIN45-PL4
  DDRL = 0x18 ; // PL3 et PL4
  DDRB = 0x80 ; // PB7 LedToggle
  // COM5B_1:0 = 10   -> clear sur egalite++, set sur egalite--
  // WGM5_3:1 = 1000  -> mode 8 => ICR5 defini le TOP
  TCCR5A = (1 << COM5A1) + (1 << COM5B1);
  TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50); // CS_12:10  = 001  -> prediv par 1
  ICR5 = Thash; // 1999 correspond a f = 4khz
  StopMoteurGD;
  // Interrution de débordement du timer
  TIMSK5 = 1 << TOIE5;
}

ISR (TIMER5_OVF_vect) {   //Fonction sur débordement du TIMER5 (=500us) --*50--> (25ms)
  compteur++;
  if (compteur == 50) {
    compteur=0;
    //Pos=myIMU.readFloatGyroY()-Pos;
    


    if (cTime == 14){
      Time++;
      cTime = 0;
    }

    //Récupération valeurs capteurs ultrason
    RangeInCentimetersD = ultrasonicD.MeasureInCentimeters();
    RangeInCentimetersF = ultrasonicF.MeasureInCentimeters();
    RangeInCentimetersG = ultrasonicG.MeasureInCentimeters();
    
    //LedToggle;

    //Récupération valeurs encodeur pour détection défaut
    /*ValEncGA = ValEncGN;
    ValEncDA = ValEncDN;
    ValEncGN = knobLeft.read();
    ValEncDN = knobRight.read();*/
  }
  
}

//Fonction calcul niveau de batterie
int calcBatt(){
  unsigned long sum;
  sum = analogRead(A0);
  sum = (3 * sum * 4980 / 1023.00); //set gain to 3
  sum =  map(sum, 6000, 7400, 0, 100); //6000 mV (min voltage for 2 cell battery) to 8500mV (max)
  return sum;
}

//Interruption arrivé du robot
void arrive(){
  Time=((NTime-ATime)/250);

  lcd.setRGB(0, 255, 0);
  MoteurGD(Stop,Stop);
  TIMSK5 = 0 << TOIE5;

  NivBat2=calcBatt();
  Enercon=(NivBat2-NivBat1)*Time*0.6;

  int navigation = 0;

  //Calcul distance parcouru --> (moyenne des deux encodeurs) / (valeur 1 tour de roue) * (périmètre d'une roue en mètre)
  long int TrDr = ((knobRight.read())/CoefEncD);
  long int TrGa = ((knobLeft.read())/CoefEncG);
  float distance = (((TrDr + TrGa)/2)*0.377);

  while((analogRead(A2) <= 800) || (not(digitalRead(infrared)))){

    if ((analogRead(A3))<400){
      navigation++;
      while((analogRead(A3))<400);
    }
    else if ((analogRead(A3))>600){
      navigation--;
      while((analogRead(A3))>600);
    }

    switch (navigation) {
      case 0 : 
        lcd.setCursor(0, 0);
        lcd.print("     arrive     ");
        lcd.setCursor(0, 1);
        lcd.print("                ");
        break;

      case 1 :
        lcd.setCursor(0, 0);
        lcd.print(" distance (en m) ");
        lcd.setCursor(0, 1);
        lcd.print("      ");
        lcd.setCursor(6, 1);
        lcd.print(distance);
        lcd.setCursor(10, 1);
        lcd.print("      ");
        break;

      case 2 :
        lcd.setCursor(0, 0);
        lcd.print(" Temps parcours ");
        lcd.setCursor(0, 1);
        lcd.print("       ");
        lcd.setCursor(6, 1);
        lcd.print(Time);
        lcd.print(" s     ");
        break;
      
      case 3 :
        lcd.setCursor(0, 0);
        lcd.print(" Nb tours roues ");
        lcd.setCursor(0, 1);
        lcd.print("Dr:");
        lcd.print(TrDr);
        lcd.print(" Ga:");
        lcd.print(TrGa);
        break;

      case 4 : 
        lcd.setCursor(0, 0);
        lcd.print("  Energie Cons  ");
        lcd.setCursor(0, 1);
        lcd.print("       ");
        lcd.print(Enercon);
        lcd.print(" Ws      ");
        break;

      case 5 : 
        lcd.setCursor(0, 0);
        lcd.print("   Nombre Nm    ");
        lcd.setCursor(0, 1);
        lcd.print("       ");
        lcd.print(Nm);
        lcd.print(" Fois      ");
        break;

      default:
        lcd.setCursor(0, 0);
        lcd.print("   navigation   ");
        lcd.setCursor(0, 1);
        lcd.print("   incorrect    ");
    }
  }
}

//Interruption Arret distance
void Arret_Dist(){
  MoteurGD(Stop,Stop);
  while(not(digitalRead(infrared)));
  lcd.setRGB(255, 0, 0); //red
  lcd.setCursor(0, 0);
  lcd.print("   Arret_Dist   ");
  while((analogRead(A2) <= 800) || (not(digitalRead(infrared))));
  lcd.clear();
  lcd.setRGB(255, 255, 255); //white
}

//Interruption Arret RB
void Arret_RB(){
  MoteurGD(Stop,Stop);
  while(not(digitalRead(infrared)));
  lcd.setRGB(255, 0, 0); //red
  lcd.setCursor(0, 0);
  lcd.print("    Arret_RB    ");
  while((analogRead(A2) <= 800) || (not(digitalRead(infrared))));
  lcd.clear();
  lcd.setRGB(255, 255, 255); //white
}

//Interruption Arret Elec
void Arret_Elec(){
  MoteurGD(Stop,Stop);
  while(not(digitalRead(infrared)));
  lcd.setRGB(255, 0, 0); //red
  lcd.setCursor(0, 0);
  lcd.print("   Arret_ELEC   ");
  while((analogRead(A2) <= 800) || (not(digitalRead(infrared))));
  lcd.clear();
  lcd.setRGB(255, 255, 255); //white
}

//Interruption Arret MEC
void Arret_Mec(){
  MoteurGD(Stop,Stop);
  while(not(digitalRead(infrared)));
  lcd.setRGB(255, 0, 0); //red
  lcd.setCursor(0, 0);
  lcd.print("   Arret_MEC    ");
  while((analogRead(A2) <= 800) || (not(digitalRead(infrared))));
  lcd.clear();
  lcd.setRGB(255, 255, 255); //white
}

/*void encodeur(){
  if (ValEncGA == ValEncGN || ValEncDA == ValEncDN){
    Arret_RB();
  }
}*/

void setup() {
  ATime=millis ();
  Serial.begin(9600);
  lcd.begin(16, 2);    //Initialisation LCD
  bar.begin();         //Initialisation de la LED bar
  
  //Entrées/sorties
  pinMode(A0,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT); 
  pinMode(A4,INPUT);
  pinMode(A5,INPUT); 
  pinMode(43,OUTPUT);

  digitalWrite(43,0);
  initMoteurs();
  
  digitalWrite(43,1);

  //Config pour Led Bar
  bar.setBits(0x3ff); 
  delay(50);
  bar.setBits(0x0);

  NivBat1=calcBatt();

  //Réglage variable moteur par rapport au niveau de batterie
  sumVoltage = calcBatt();
  VmaxD = 1250 - sumVoltage;
  VmaxG = 845 - sumVoltage;
  bar.setLevel(sumVoltage/10); //print value on LED bar*/

  lcd.setCursor(0, 0);
  lcd.print(" Pret au depart ");

  //Attente pour départ
  while(digitalRead(infrared) || (analogRead(A2) <= 800)){
    MoteurGD(Stop,Stop);
  }
  lcd.clear();
  Time = 0;
  sei();
  
}

void loop() {

  /*======================================================================================================================
  Séquence de "SI ... ALORS" qui permet au robot de prendre la bonne décision suivant les mesures des 3 capteur à utrason.
  =======================================================================================================================*/

  //Si le robot détecte la ligne d'arrivé ou durée course > 10 min
  if ((digitalRead(infrared)) || ((NTime - ATime) > 600000)){
    arrive();
  }

  //Détection pour Arrêt_Dist
  if(RangeInCentimetersD < 8 || RangeInCentimetersF < 8 || RangeInCentimetersG < 8){
    Arret_Dist();
  }

  if ((analogRead(A4) >= 3300)||(analogRead(A5) >= 3300) ){
    Arret_Elec();
  }

//Détection pour Nm

 
 

  // Si mur trop prêt devant
  if (RangeInCentimetersF < 70) {

    // Prise de décision pour soit aller à droite ou à gauche
    if (RangeInCentimetersD <= RangeInCentimetersG){
      while(RangeInCentimetersF < 74){
      MoteurGD(600,VmaxD);
      //encodeur();
      }
    }
    else{
      while(RangeInCentimetersF < 74){
      MoteurGD(VmaxG,600);
      //encodeur();
      }
    }
  }

  //Double "Sinon SI ... ALORS" qui permettent au robot de rester au milieu du couloir
  else if (RangeInCentimetersD < RangeInCentimetersG && RangeInCentimetersG < 150){

    Nvit = (VmaxG - (CoefG*(int(RangeInCentimetersG - RangeInCentimetersD)^2)));

    MoteurGD(Nvit,VmaxD);
    //encodeur();
  }
  else if (RangeInCentimetersG < RangeInCentimetersD && RangeInCentimetersD < 150){

    Nvit = (VmaxD - (CoefD*(int(RangeInCentimetersG - RangeInCentimetersD)^2)));

    MoteurGD(VmaxG,Nvit);
    //encodeur();
  }

  //Le robot tourne à droite lorsque aucun mur n'est présent
  else if (RangeInCentimetersD > 150){
    MoteurGD(VmaxG,VmaxD-190);
    //encodeur();
  }

  //Le robot de capte aucun mur proche alors il avance tout droit
  else {
    MoteurGD(VmaxG,VmaxD);
    //encodeur();
  }

}
